/*
 * Quad flight controller (pcb/, ESP32-S3-WROOM-1) -- blink + Bosch BMI323
 * IMU readout over I2C, motors, BLE log.
 *
 *   LED  : status LED1, GPIO21, active-LOW.              Driven via raw
 *          GPIO register writes (no driver).
 *   IMU  : BMI323 on I2C0, SDA=GPIO11, SCL=GPIO12, 400 kHz, address 0x68.
 *          Uses ESP-IDF's i2c_master driver (writing a register-level
 *          I2C driver is a separate project).
 *   VBAT : battery / 2 on GPIO10 (ADC1 channel 9), logged with the state;
 *          arming is refused below 3.5 V; a flat pack puts the board to
 *          deep sleep instead of draining it (see VBAT_SLEEP_MV).
 *   Safety: disarm on BLE disconnect, arm always at zero throttle.
 *
 * BMI323 quirk: every register read returns 2 dummy bytes followed by
 * the 16-bit register value, LSB first. Every register write is 1
 * address byte + 16-bit value LSB-first.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include <math.h>

#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* ---------- LED ---------- */
#define LED_GPIO        21u
#define LED_BIT         (1u << LED_GPIO)

/* ---------- Motors (MOSFET gates) ----------
 * Board net -> SoC GPIO (see the pin map in pcb/index.tsx):
 *   PWM0 -> GPIO1   (motor 0, front right)
 *   PWM1 -> GPIO2   (motor 1, back right)
 *   PWM2 -> GPIO16  (motor 2, back left)
 *   PWM3 -> GPIO4   (motor 3, front left)
 * All four pins live in GPIO bank 0 (bits 0..31), so the standard
 * GPIO_OUT_W1T{S,C} / GPIO_ENABLE_W1TS registers are enough.
 */
#define MOTOR0_GPIO     1u
#define MOTOR1_GPIO     2u
#define MOTOR2_GPIO     16u
#define MOTOR3_GPIO     4u
static const uint32_t MOTOR_GPIOS[4] = {
    MOTOR0_GPIO, MOTOR1_GPIO, MOTOR2_GPIO, MOTOR3_GPIO,
};
static const ledc_channel_t MOTOR_LEDC_CH[4] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3,
};

/* ---------- PWM / control ----------
 * Brushed motors driven through MOSFETs: 20 kHz PWM, 8-bit (0..255).
 * Quad X frame mixer assumes the following physical layout (viewed
 * from above, the drone's nose pointing up the page):
 *
 *      M3 (FL, CW)   M0 (FR, CCW)
 *            \\   //
 *             |X|
 *            //   \\
 *      M2 (BL, CCW)  M1 (BR, CW)
 *
 * If your prop rotations / motor positions differ, fix the mapping
 * here -- the PID will fight itself otherwise.
 */
#define LEDC_FREQ_HZ            20000
#define LEDC_RES_BITS           LEDC_TIMER_8_BIT
#define LEDC_MAX                255
#define MOTOR_TEST_DUTY         64        /* ~25% -- visible spin, no lift  */
#define MAX_THROTTLE            220       /* head-room above this for PID    */
#define THROTTLE_STEP           5
#define ARM_TILT_LIMIT_DEG      5.0f      /* refuse to arm if tilted       */
#define KILL_TILT_LIMIT_DEG     50.0f     /* auto-disarm on big tilt       */

/* PID gains (PWM units per degree / dps). These are FIRST-GUESS values
 * for a small brushed tiny-whoop-class drone -- expect to retune. */
#define ROLL_KP   1.5f
#define ROLL_KD   0.25f
#define PITCH_KP  1.5f
#define PITCH_KD  0.25f
#define YAW_KD    0.5f

#define CTRL_RATE_HZ            200
#define CTRL_PERIOD_MS          (1000 / CTRL_RATE_HZ)

/* ---------- I2C ---------- */
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    11
#define I2C_SCL_GPIO    12
#define I2C_FREQ_HZ     400000

/* ---------- Battery sense ----------
 * VBAT / 2 (100k / 100k, 100 nF) on GPIO10 = ADC1 channel 9. */
#define VBAT_ADC_CH        ADC_CHANNEL_9
#define VBAT_MIN_ARM_MV    3500      /* refuse to arm below this at rest   */
#define VBAT_LOW_MV        3300      /* warn while flying: land now        */
/* The 3.3 V buck-boost keeps the ESP32 running down to VBAT = 1.8 V, so a
 * pack left plugged in would be drained far below a safe voltage. Disarmed
 * and below VBAT_SLEEP_MV for 10 s (or below VBAT_EMPTY_MV for 1 s): deep sleep
 * (~0.1 mA for the whole board), waking every 5 min; back to normal above
 * VBAT_WAKE_MV. This
 * also lets USB charging recover a flat pack (the charger trickles 26 mA). */
#define VBAT_SLEEP_MV      3300
#define VBAT_EMPTY_MV      3000
#define VBAT_WAKE_MV       3500
#define VBAT_SLEEP_S       300

/* ---------- BLE control ---------- */
#define BLE_DEVICE_NAME "QuadFW"

/* ---------- BMI323 ---------- */
#define BMI323_ADDR        0x68
#define BMI323_CHIP_ID     0x00
#define BMI323_ACC_DATA_X  0x03
#define BMI323_GYR_DATA_X  0x06
#define BMI323_ACC_CONF    0x20
#define BMI323_GYR_CONF    0x21
#define BMI323_CHIP_ID_VAL 0x43

static const char *TAG = "imu";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_bmi;

/* -------------------- LED (raw registers) -------------------- */
static void led_init(void)
{
    REG_WRITE(IO_MUX_GPIO21_REG, (1u << MCU_SEL_S));
    REG_WRITE(GPIO_ENABLE_W1TS_REG, LED_BIT);
}

static inline void led_set(bool on)
{
    /* Active-low: LOW = lit. */
    REG_WRITE(on ? GPIO_OUT_W1TC_REG : GPIO_OUT_W1TS_REG, LED_BIT);
}

/* -------------------- Motors (LEDC PWM) -------------------- */
static void motors_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_RES_BITS,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    for (int i = 0; i < 4; ++i) {
        ledc_channel_config_t c = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = MOTOR_LEDC_CH[i],
            .timer_sel  = LEDC_TIMER_0,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = (int)MOTOR_GPIOS[i],
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }
}

/* M0/M2 switch on at the start of each PWM period, M1/M3 end at its end,
 * so below 50 % duty the two pairs never draw battery current at the same
 * time: half the peak current (less battery sag and noise) for free.
 * LEDC sets the pin at count hpoint and clears it at hpoint + duty; the
 * 8-bit counter runs 0..255, so that sum must stay <= 255 (256 is never
 * reached and the pin would stay on): end-aligned = hpoint 255 - duty. */
static inline void motor_set_duty(int idx, int duty)
{
    if (duty < 0) duty = 0;
    if (duty > LEDC_MAX) duty = LEDC_MAX;
    uint32_t hpoint = (idx & 1) && duty > 0 ? (uint32_t)(LEDC_MAX - duty) : 0;
    ledc_set_duty_with_hpoint(LEDC_LOW_SPEED_MODE, MOTOR_LEDC_CH[idx], (uint32_t)duty, hpoint);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_LEDC_CH[idx]);
}

/* Spin each motor briefly in turn so we can verify wiring/order. */
static void motors_sweep(uint32_t on_ms, uint32_t gap_ms)
{
    ESP_LOGI(TAG, "motor sweep: %lu ms on @ duty %d, %lu ms gap",
             (unsigned long)on_ms, MOTOR_TEST_DUTY, (unsigned long)gap_ms);
    for (int i = 0; i < 4; ++i) {
        ESP_LOGI(TAG, "  motor %d (GPIO%lu) on",
                 i, (unsigned long)MOTOR_GPIOS[i]);
        motor_set_duty(i, MOTOR_TEST_DUTY);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        motor_set_duty(i, 0);
        vTaskDelay(pdMS_TO_TICKS(gap_ms));
    }
    ESP_LOGI(TAG, "motor sweep done");
}

/* -------------------- battery voltage (IO10) -------------------- */
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_adc_cali;

static void vbat_init(void)
{
    adc_oneshot_unit_init_cfg_t u = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&u, &s_adc));
    adc_oneshot_chan_cfg_t c = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, VBAT_ADC_CH, &c));
    adc_cali_curve_fitting_config_t k = {
        .unit_id = ADC_UNIT_1, .chan = VBAT_ADC_CH,
        .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&k, &s_adc_cali) != ESP_OK) {
        s_adc_cali = NULL;
        ESP_LOGW(TAG, "vbat: no ADC calibration, battery checks off");
    }
}

/* Battery voltage in mV, mean of 8 samples (the divider's 5 ms RC already
 * averages the motor PWM), 0 if it can't be measured. */
static int vbat_read_mv(void)
{
    int sum = 0;
    for (int i = 0; i < 8; ++i) {
        int raw, mv;
        if (!s_adc_cali || adc_oneshot_read(s_adc, VBAT_ADC_CH, &raw) != ESP_OK ||
            adc_cali_raw_to_voltage(s_adc_cali, raw, &mv) != ESP_OK) {
            return 0;
        }
        sum += mv;
    }
    return 2 * sum / 8;
}

/* -------------------- shared control state -------------------- */
static volatile uint8_t   s_base_throttle = 0;
static volatile bool      s_armed         = false;
static volatile int8_t    s_test_motor    = -1;
static volatile uint32_t  s_test_end_ms   = 0;
static volatile float     s_roll_deg      = 0.0f;
static volatile float     s_pitch_deg     = 0.0f;
static volatile int       s_vbat_mv       = 0;     /* 0 = not measured */
static volatile bool      s_ready         = false; /* level cal + VBAT done */
static bool               s_ble_up        = false;

/* -------------------- BMI323 over I2C -------------------- */
static esp_err_t bmi_read(uint8_t reg, uint8_t *dst, size_t n)
{
    /* 2 dummy bytes precede every register's data. */
    uint8_t rx[2 + 12];
    if (n > sizeof(rx) - 2) {
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t err = i2c_master_transmit_receive(s_bmi, &reg, 1,
                                                rx, 2 + n, 10);
    if (err == ESP_OK) {
        memcpy(dst, rx + 2, n);
    }
    return err;
}

static esp_err_t bmi_read_u16(uint8_t reg, uint16_t *out)
{
    uint8_t b[2];
    esp_err_t err = bmi_read(reg, b, 2);
    if (err == ESP_OK) {
        *out = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
    }
    return err;
}

static esp_err_t bmi_write_u16(uint8_t reg, uint16_t val)
{
    uint8_t tx[3] = { reg, (uint8_t)(val & 0xff), (uint8_t)(val >> 8) };
    return i2c_master_transmit(s_bmi, tx, sizeof(tx), 10);
}

static void i2c_bring_up(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BMI323_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_bmi));
}

/* -------------------- BLE (NimBLE) Nordic UART Service --------------------
 *
 * Exposes the Nordic UART Service (NUS):
 *   Service UUID  6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX char (W)   6E400002-B5A3-F393-E0A9-E50E24DCCA9E   <- laptop writes here
 *   TX char (N)   6E400003-B5A3-F393-E0A9-E50E24DCCA9E   <- (unused for now)
 *
 * Same single-byte protocol as before:
 *   '1'..'4' -> pulse that motor for 200 ms
 *   's','0'  -> all motors off
 *
 * NimBLE UUIDs are stored little-endian (LSB-first), so the byte arrays
 * below are the standard text UUID written backwards.
 */

static const ble_uuid128_t NUS_SVC_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t NUS_RX_UUID  = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t NUS_TX_UUID  = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint8_t s_own_addr_type;
static QueueHandle_t s_cmd_q;  /* one-byte commands from BLE -> motor task */

/* BLE log streaming: any ESP_LOG* output is duplicated to the NUS TX
 * characteristic so logs can be captured without a USB cable. */
static volatile uint16_t s_conn_handle = 0xFFFF;
/* arm (cmd task) and link loss (NimBLE host, other core) change s_armed together */
static portMUX_TYPE s_arm_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_tx_val_handle  = 0;
static bool     s_notify_enabled = false;
static StreamBufferHandle_t s_log_sb = NULL;

static void ble_advertise(void);

static void handle_cmd_byte(uint8_t c)
{
    if (s_cmd_q) {
        xQueueSend(s_cmd_q, &c, 0);
    }
}

/* GATT write callback: any data written to the RX char ends up here. */
static int nus_rx_access(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    uint8_t buf[16];
    uint16_t out_len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &out_len);
    if (rc != 0) return BLE_ATT_ERR_UNLIKELY;
    for (uint16_t i = 0; i < out_len; ++i) {
        handle_cmd_byte(buf[i]);
    }
    return 0;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &NUS_SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
                .uuid = &NUS_RX_UUID.u,
                .access_cb = nus_rx_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            }, {
                .uuid = &NUS_TX_UUID.u,
                .access_cb = nus_rx_access, /* unused, but a cb is required */
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_tx_val_handle,
            }, { 0 }
        },
    },
    { 0 },
};

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "ble: connect %s",
                 event->connect.status == 0 ? "ok" : "failed");
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
        } else {
            ble_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "ble: disconnect, reason=0x%x",
                 event->disconnect.reason);
        /* Link lost: nobody is in control any more -> motors off. */
        if (s_armed) ESP_LOGW(TAG, "ble: link lost -> DISARM");
        taskENTER_CRITICAL(&s_arm_mux);
        s_armed = false;
        s_base_throttle = 0;
        s_conn_handle = 0xFFFF;
        taskEXIT_CRITICAL(&s_arm_mux);
        if (s_cmd_q) xQueueReset(s_cmd_q);   /* no stale 'a' after the link */
        s_notify_enabled = false;
        ble_advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_val_handle) {
            s_notify_enabled = event->subscribe.cur_notify;
        }
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_advertise();
        break;
    default:
        break;
    }
    return 0;
}

static void ble_advertise(void)
{
    struct ble_hs_adv_fields fields = { 0 };
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)BLE_DEVICE_NAME;
    fields.name_len = strlen(BLE_DEVICE_NAME);
    fields.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields rc=%d", rc);
        return;
    }
    struct ble_gap_adv_params adv_params = { 0 };
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "ble: advertising as \"%s\"", BLE_DEVICE_NAME);
}

static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto rc=%d", rc);
        return;
    }
    ble_advertise();
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "ble: host reset, reason=%d", reason);
}

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ---------- BLE log streaming ----------
 *
 * Hook into esp_log via esp_log_set_vprintf: every log line is written
 * into a FreeRTOS stream buffer (also still printed to stdout for the
 * USB serial monitor). A dedicated task drains the stream buffer and
 * sends the bytes out as NUS TX notifications, chunked to fit in the
 * default 23-byte ATT MTU (20 bytes payload). When no client is
 * subscribed the bytes are simply consumed and dropped.
 *
 * Caveat: when the CPU panics, interrupts are off and the BLE host
 * task cannot run -- so the panic message itself will not reach the
 * client live. The log lines printed in the moments BEFORE the panic
 * do reach the client, which is usually what matters.
 */
#define BLE_LOG_SB_SIZE   4096
#define BLE_LOG_LINE_MAX  192
#define BLE_LOG_CHUNK     20

static int ble_log_vprintf(const char *fmt, va_list ap)
{
    char buf[BLE_LOG_LINE_MAX];
    va_list ap2;
    va_copy(ap2, ap);
    int m = vsnprintf(buf, sizeof(buf), fmt, ap2);
    va_end(ap2);

    int n = vprintf(fmt, ap);   /* also keep USB monitor working */

    if (s_log_sb && m > 0) {
        size_t len = (m >= (int)sizeof(buf)) ? sizeof(buf) - 1 : (size_t)m;
        /* Never block: if the buffer is full we'd rather drop log bytes
         * than stall the caller (which may be a high-priority task). */
        (void)xStreamBufferSend(s_log_sb, buf, len, 0);
    }
    return n;
}

static void ble_log_task(void *arg)
{
    uint8_t buf[BLE_LOG_CHUNK];
    for (;;) {
        size_t got = xStreamBufferReceive(s_log_sb, buf, sizeof(buf),
                                          portMAX_DELAY);
        if (got == 0) continue;
        if (!s_notify_enabled || s_conn_handle == 0xFFFF ||
            s_tx_val_handle == 0) {
            continue;   /* nobody listening -- drop */
        }
        struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, got);
        if (!om) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        int rc = ble_gatts_notify_custom(s_conn_handle, s_tx_val_handle, om);
        if (rc != 0) {
            /* On ENOMEM / busy, back off briefly. mbuf is consumed by
             * NimBLE regardless of return code. */
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static void ble_log_bring_up(void)
{
    s_log_sb = xStreamBufferCreate(BLE_LOG_SB_SIZE, 1);
    if (!s_log_sb) {
        ESP_LOGE(TAG, "ble_log: stream buffer alloc failed");
        return;
    }
    /* NimBLE prints an INFO line for every notification it sends. Once
     * we tee logs to BLE, that creates an exponential feedback loop
     * (send notify -> log -> send notify -> ...). Suppress its info
     * chatter; warnings and errors still get through. */
    esp_log_level_set("NimBLE",      ESP_LOG_WARN);
    esp_log_level_set("nimble",      ESP_LOG_WARN);
    esp_log_level_set("BLE_HS",      ESP_LOG_WARN);
    esp_log_level_set("BTDM_INIT",   ESP_LOG_WARN);
    xTaskCreate(ble_log_task, "ble_log", 3072, NULL, 4, NULL);
    esp_log_set_vprintf(ble_log_vprintf);
}

static void ble_bring_up(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svr_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svr_svcs));
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(BLE_DEVICE_NAME));

    nimble_port_freertos_init(ble_host_task);
    s_ble_up = true;
}

/* Flat battery: motors off, BLE off, IMU suspended, then deep sleep with a
 * timer wake (see VBAT_SLEEP_MV). */
static void deep_sleep(int mv)
{
    for (int i = 0; i < 4; ++i) motor_set_duty(i, 0);
    ESP_LOGW(TAG, "battery %d mV: deep sleep, next check in %d s", mv, VBAT_SLEEP_S);
    vTaskDelay(pdMS_TO_TICKS(200));   /* let the log line out */
    if (s_ble_up) {
        s_notify_enabled = false;   /* the log tee stops sending */
        if (nimble_port_stop() == 0) nimble_port_deinit();
    }
    bmi_write_u16(BMI323_ACC_CONF, 0x0000);   /* accel + gyro off */
    bmi_write_u16(BMI323_GYR_CONF, 0x0000);
    esp_sleep_enable_timer_wakeup((uint64_t)VBAT_SLEEP_S * 1000000u);
    esp_deep_sleep_start();
}

/* -------------------- motor command consumer -------------------- */
static void motor_cmd_task(void *arg)
{
    uint8_t c;
    for (;;) {
        if (xQueueReceive(s_cmd_q, &c, portMAX_DELAY) != pdTRUE) continue;
        switch (c) {
        case 'a':
            if (!s_ready || s_conn_handle == 0xFFFF) {
                ESP_LOGW(TAG, "cmd: arm refused, not ready");
            } else if (s_vbat_mv < VBAT_MIN_ARM_MV) {   /* 0 = can't measure it */
                ESP_LOGW(TAG, "cmd: arm refused, battery %d mV", s_vbat_mv);
            } else if (fabsf(s_roll_deg) < ARM_TILT_LIMIT_DEG &&
                fabsf(s_pitch_deg) < ARM_TILT_LIMIT_DEG) {
                bool ok = false;
                taskENTER_CRITICAL(&s_arm_mux);
                if (s_conn_handle != 0xFFFF) {   /* link still up */
                    s_base_throttle = 0;         /* always start from idle */
                    s_armed = ok = true;
                }
                taskEXIT_CRITICAL(&s_arm_mux);
                if (ok) ESP_LOGI(TAG, "cmd: ARMED (throttle=0)");
                else    ESP_LOGW(TAG, "cmd: arm refused, link lost");
            } else {
                ESP_LOGW(TAG, "cmd: arm refused, tilt r=%.1f p=%.1f deg",
                         s_roll_deg, s_pitch_deg);
            }
            break;
        case 'd': case 's': case '0':
            s_armed = false;
            s_base_throttle = 0;
            ESP_LOGI(TAG, "cmd: DISARMED");
            break;
        case 'w': case '+': case '=':
            if (s_base_throttle + THROTTLE_STEP > MAX_THROTTLE) {
                s_base_throttle = MAX_THROTTLE;
            } else {
                s_base_throttle += THROTTLE_STEP;
            }
            ESP_LOGI(TAG, "cmd: throttle=%u", (unsigned)s_base_throttle);
            break;
        case 'x': case '-':
            if (s_base_throttle <= THROTTLE_STEP) {
                s_base_throttle = 0;
            } else {
                s_base_throttle -= THROTTLE_STEP;
            }
            ESP_LOGI(TAG, "cmd: throttle=%u", (unsigned)s_base_throttle);
            break;
        case '1': case '2': case '3': case '4':
            if (!s_armed) {
                s_test_motor = (int8_t)(c - '1');
                s_test_end_ms =
                    (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) + 200;
                ESP_LOGI(TAG, "cmd: test motor %d", (int)s_test_motor);
            } else {
                ESP_LOGW(TAG, "cmd: ignored '%c' while armed", c);
            }
            break;
        default:
            ESP_LOGW(TAG, "cmd: ignore 0x%02x", (unsigned)c);
            break;
        }
    }
}

/* -------------------- 200 Hz control loop --------------------
 * 1. Read accel + gyro.
 * 2. Complementary filter -> roll/pitch (rad) and yaw (gyro-integrated).
 * 3. Auto-disarm on extreme tilt.
 * 4. If armed: angle PID + quad-X mixer -> 4 PWM outputs.
 *    If disarmed: motors 0, except an optional motor-test pulse.
 * 5. Print state at 10 Hz, blink LED at 1 Hz.
 */
static void control_task(void *arg)
{
    const float GYR_LSB_TO_RADS = (1000.0f / 32768.0f) * (float)M_PI / 180.0f;
    const float GYR_LSB_TO_DPS  = 1000.0f / 32768.0f;
    const float ACC_LSB_TO_G    = 4.0f / 32768.0f;
    const float DT              = 1.0f / (float)CTRL_RATE_HZ;
    const float ALPHA           = 0.98f;
    const float RAD_TO_DEG      = 180.0f / (float)M_PI;

    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    bool  init = false;
    bool  led_on = false;
    int   print_div = 0;
    int   blink_div = 0;
    int   i2c_err_streak = 0;
    int   vbat_div = 0, vbat_low_n = 0, vbat_empty_n = 0;
    bool  vbat_seen = false;

    /* Level calibration: collect the first CAL_N accel samples while the
     * drone is sitting still on the desk and use their mean as the
     * accelerometer's idea of "level". This compensates for the IMU
     * being mounted with a small tilt relative to the airframe. Arming
     * is blocked until calibration completes. */
    const int CAL_N = 200;            /* 200 samples @ 200 Hz = 1 s */
    int   cal_count = 0;
    float cal_roll_sum = 0.0f, cal_pitch_sum = 0.0f;
    float roll_bias = 0.0f, pitch_bias = 0.0f;
    bool  calibrated = false;

    /* If consecutive I2C reads fail this many times in a row we treat the
     * bus as compromised (most likely motor EMI) and force-disarm so the
     * controller can't keep commanding motors based on stale data. */
    const int I2C_ERR_DISARM_THRESHOLD = 5;

    for (;;) {
        uint8_t acc[6], gyr[6];
        esp_err_t e1 = bmi_read(BMI323_ACC_DATA_X, acc, sizeof(acc));
        esp_err_t e2 = bmi_read(BMI323_GYR_DATA_X, gyr, sizeof(gyr));

        if (e1 == ESP_OK && e2 == ESP_OK) {
            i2c_err_streak = 0;
            int16_t axi = (int16_t)(acc[0] | (acc[1] << 8));
            int16_t ayi = (int16_t)(acc[2] | (acc[3] << 8));
            int16_t azi = (int16_t)(acc[4] | (acc[5] << 8));
            int16_t gxi = (int16_t)(gyr[0] | (gyr[1] << 8));
            int16_t gyi = (int16_t)(gyr[2] | (gyr[3] << 8));
            int16_t gzi = (int16_t)(gyr[4] | (gyr[5] << 8));

            float ax = axi * ACC_LSB_TO_G;
            float ay = ayi * ACC_LSB_TO_G;
            float az = azi * ACC_LSB_TO_G;
            float gx_rad = gxi * GYR_LSB_TO_RADS;
            float gy_rad = gyi * GYR_LSB_TO_RADS;
            float gz_rad = gzi * GYR_LSB_TO_RADS;
            float gx_dps = gxi * GYR_LSB_TO_DPS;
            float gy_dps = gyi * GYR_LSB_TO_DPS;
            float gz_dps = gzi * GYR_LSB_TO_DPS;

            float roll_acc_raw  = atan2f(ay, az);
            float pitch_acc_raw = atan2f(-ax, sqrtf(ay*ay + az*az));

            /* Accumulate the level bias during the first CAL_N samples. */
            if (!calibrated) {
                cal_roll_sum  += roll_acc_raw;
                cal_pitch_sum += pitch_acc_raw;
                if (++cal_count >= CAL_N) {
                    roll_bias  = cal_roll_sum  / (float)CAL_N;
                    pitch_bias = cal_pitch_sum / (float)CAL_N;
                    calibrated = true;
                    ESP_LOGI(TAG,
                        "level cal done: roll_bias=%+.2f deg pitch_bias=%+.2f deg",
                        roll_bias  * RAD_TO_DEG,
                        pitch_bias * RAD_TO_DEG);
                }
            }

            float roll_acc  = roll_acc_raw  - roll_bias;
            float pitch_acc = pitch_acc_raw - pitch_bias;

            if (!init) {
                roll  = roll_acc;
                pitch = pitch_acc;
                yaw   = 0.0f;
                init  = true;
            } else {
                roll  = ALPHA * (roll  + gx_rad * DT)
                      + (1.0f - ALPHA) * roll_acc;
                pitch = ALPHA * (pitch + gy_rad * DT)
                      + (1.0f - ALPHA) * pitch_acc;
                yaw   = yaw + gz_rad * DT;
            }

            float roll_deg  = roll  * RAD_TO_DEG;
            float pitch_deg = pitch * RAD_TO_DEG;
            float yaw_deg   = yaw   * RAD_TO_DEG;
            s_roll_deg  = roll_deg;
            s_pitch_deg = pitch_deg;

            /* Safety: extreme tilt -> drop everything. */
            if (s_armed && (fabsf(roll_deg)  > KILL_TILT_LIMIT_DEG ||
                            fabsf(pitch_deg) > KILL_TILT_LIMIT_DEG)) {
                s_armed = false;
                s_base_throttle = 0;
                ESP_LOGW(TAG, "ctrl: AUTO-DISARM tilt r=%.1f p=%.1f",
                         roll_deg, pitch_deg);
            }

            int out[4] = { 0, 0, 0, 0 };
            uint32_t now_ms =
                (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

            if (s_armed && s_base_throttle == 0) {
                /* armed at idle: motors stay off until throttle comes up */
            } else if (s_armed) {
                float T = (float)s_base_throttle;
                float roll_err  = 0.0f - roll_deg;
                float pitch_err = 0.0f - pitch_deg;
                float r = ROLL_KP  * roll_err  - ROLL_KD  * gx_dps;
                float p = PITCH_KP * pitch_err - PITCH_KD * gy_dps;
                /* Damp yaw rate. +y in the mixer drives the airframe CW
                 * (increases CCW motors); gz>0 is CCW rotation, so we
                 * need y = +KD*gz to oppose it. The previous "-KD*gz"
                 * was positive feedback and caused a runaway spin. */
                float y = YAW_KD * gz_dps;

                /* Quad-X mixer (see physical layout above).
                 *   +roll  -> right side down, lift left side
                 *   +pitch -> nose up, lift back side
                 *   +yaw   -> nose right, increase CCW motors */
                float m[4] = {
                    T - r - p + y,   /* M0 FR CCW */
                    T - r + p - y,   /* M1 BR CW  */
                    T + r + p + y,   /* M2 BL CCW */
                    T + r - p - y,   /* M3 FL CW  */
                };
                for (int i = 0; i < 4; ++i) {
                    if (m[i] < 0.0f) m[i] = 0.0f;
                    if (m[i] > (float)LEDC_MAX) m[i] = (float)LEDC_MAX;
                    out[i] = (int)m[i];
                }
            } else if (s_test_motor >= 0 && s_test_motor < 4 &&
                       (int32_t)(s_test_end_ms - now_ms) > 0) {
                out[s_test_motor] = MOTOR_TEST_DUTY;
            } else if (s_test_motor >= 0) {
                s_test_motor = -1;
            }

            for (int i = 0; i < 4; ++i) motor_set_duty(i, out[i]);

            if (++print_div >= 20) {  /* 10 Hz */
                print_div = 0;
                ESP_LOGI(TAG,
                       "%s T=%3u r=%+6.1f p=%+6.1f y=%+6.1f | "
                       "%3d %3d %3d %3d | %4d mV",
                       s_armed ? "ARM" : "dis",
                       (unsigned)s_base_throttle,
                       roll_deg, pitch_deg, yaw_deg,
                       out[0], out[1], out[2], out[3], s_vbat_mv);
            }
            if (++blink_div >= 100) {  /* 1 Hz */
                blink_div = 0;
                if (s_armed && s_vbat_mv > 0 && s_vbat_mv < VBAT_LOW_MV) {
                    ESP_LOGW(TAG, "battery low (%d mV): land", s_vbat_mv);
                }
                led_on = !led_on;
                led_set(led_on);
            }
        } else {
            /* Don't spam the log -- one line per error is enough. */
            if (i2c_err_streak < 100) {
                ESP_LOGW(TAG, "i2c read failed: acc=%s gyr=%s (streak=%d)",
                         esp_err_to_name(e1), esp_err_to_name(e2),
                         i2c_err_streak + 1);
            }
            ++i2c_err_streak;
            if (s_armed && i2c_err_streak >= I2C_ERR_DISARM_THRESHOLD) {
                s_armed = false;
                s_base_throttle = 0;
                ESP_LOGE(TAG, "ctrl: I2C lost -> AUTO-DISARM");
            }
            if (!s_armed) {
                s_test_motor = -1;
                for (int i = 0; i < 4; ++i) motor_set_duty(i, 0);
            }
        }

        /* Battery, 10 Hz, also while the IMU is failing. Only counted with
         * no motor current, and only consecutive low samples put it to sleep. */
        if (++vbat_div >= 20) {
            vbat_div = 0;
            int mv = vbat_read_mv();
            s_vbat_mv = mv;
            bool quiet = !s_armed && s_test_motor < 0;
            vbat_low_n   = quiet && mv > 0 && mv < VBAT_SLEEP_MV ? vbat_low_n + 1 : 0;
            vbat_empty_n = quiet && mv > 0 && mv < VBAT_EMPTY_MV ? vbat_empty_n + 1 : 0;
            if (vbat_low_n >= 100 || vbat_empty_n >= 10) deep_sleep(mv);
            vbat_seen = true;
        }
        if (calibrated && vbat_seen) s_ready = true;

        /* Plain vTaskDelay (not vTaskDelayUntil): if one iteration runs
         * long, we just slip a cycle instead of busy-spinning to catch
         * up. That catch-up loop was starving CPU1 and tripping the
         * interrupt watchdog at higher throttles. */
        vTaskDelay(pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

/* -------------------- app -------------------- */
void app_main(void)
{
    led_init();
    motors_init();
    vbat_init();

    /* Battery first, before the motor check, BLE and anything else draws
     * current: an empty pack, or a sleep wake that USB hasn't charged back
     * above VBAT_WAKE_MV, goes straight back to sleep. */
    esp_reset_reason_t why = esp_reset_reason();
    int boot_mv = vbat_read_mv();
    bool woke = why == ESP_RST_DEEPSLEEP, low = boot_mv > 0 && boot_mv < VBAT_WAKE_MV;
    i2c_bring_up();
    if (boot_mv > 0 && (boot_mv < VBAT_EMPTY_MV || (woke && low))) {
        bmi_write_u16(BMI323_ACC_CONF, 0x0000);   /* IMU off, if it was on */
        bmi_write_u16(BMI323_GYR_CONF, 0x0000);
        esp_sleep_enable_timer_wakeup((uint64_t)VBAT_SLEEP_S * 1000000u);
        esp_deep_sleep_start();
    }

    /* One-shot motor wiring check, only on a real power-up (not after a
     * battery-sleep wake, a crash or a watchdog reset, nobody may be
     * watching then) and not on a nearly flat pack. */
    if (why == ESP_RST_POWERON && !low) motors_sweep(100, 250);

    /* Bring up BLE NUS server + motor command consumer task. */
    s_cmd_q = xQueueCreate(16, sizeof(uint8_t));
    /* 4 KB: every ESP_LOGI here goes through ble_log_vprintf, which
     * puts a 192 B buf + vsnprintf + vprintf on the stack. 2 KB overflows. */
    xTaskCreate(motor_cmd_task, "motorcmd", 4096, NULL, 5, NULL);
    ble_bring_up();
    ble_log_bring_up();   /* tee ESP_LOG -> BLE NUS TX */

    /* A brown-out shows up here after the fact (USB console). */
    if (why == ESP_RST_BROWNOUT) {
        ESP_LOGW(TAG, "reset reason: BROWN-OUT");
    } else {
        ESP_LOGI(TAG, "reset reason: %d", (int)why);
    }

    /* Let the sensor finish its power-on sequence. */
    vTaskDelay(pdMS_TO_TICKS(50));

    uint16_t chip_id = 0;
    esp_err_t err = bmi_read_u16(BMI323_CHIP_ID, &chip_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read of CHIP_ID failed: %s",
                 esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "BMI323 CHIP_ID raw=0x%04x  low=0x%02x  (expect 0x%02x)",
             chip_id, chip_id & 0xff, BMI323_CHIP_ID_VAL);

    if ((chip_id & 0xff) != BMI323_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "BMI323 not detected. Check:");
        ESP_LOGE(TAG, " 1. SDA=GPIO%d SCL=GPIO%d wiring",
                 I2C_SDA_GPIO, I2C_SCL_GPIO);
        ESP_LOGE(TAG, " 2. Address 0x68 vs 0x69 (SDO pin)");
        ESP_LOGE(TAG, " 3. VDD / VDDIO supplied");
        /* Keep blinking so we know the chip is still alive. */
        for (bool on = false;; on = !on) {
            led_set(on);
            int mv = vbat_read_mv();
            if (mv > 0 && mv < VBAT_SLEEP_MV) deep_sleep(mv);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    ESP_LOGI(TAG, "BMI323 detected, configuring for 200 Hz...");

    /* Accelerometer: normal mode, 200 Hz ODR, +/-4 g range. */
    ESP_ERROR_CHECK(bmi_write_u16(BMI323_ACC_CONF, 0x4029));
    vTaskDelay(pdMS_TO_TICKS(5));
    /* Gyroscope:     normal mode, 200 Hz ODR, +/-1000 dps range. */
    ESP_ERROR_CHECK(bmi_write_u16(BMI323_GYR_CONF, 0x4039));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Starting control loop @ %d Hz. DISARMED. "
                  "BLE keys: a=arm d=disarm w=thr+ x=thr- 1-4=test motor",
             CTRL_RATE_HZ);

    /* Pin control loop to APP_CPU (core 1) so NimBLE on core 0 cannot
     * starve it (and vice versa). */
    xTaskCreatePinnedToCore(control_task, "ctrl", 6144, NULL, 8, NULL, 1);

    /* app_main returns; control_task, motor_cmd_task, and the NimBLE
     * host task all keep running. */
    vTaskDelete(NULL);
}
