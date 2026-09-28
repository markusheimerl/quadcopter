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
 *   Safety: arming needs a level calibration, >= 3.5 V and a live client
 *          heartbeat ('h' every 200 ms, tools/motor_keys.py); disarm on
 *          link loss, heartbeat loss, 20 s idle, 50 deg tilt, or 2 s stuck
 *          beyond 30 deg with throttle up. Nothing spins at power-up.
 *   Flight: flight.c holds roll and pitch at trim + steering (arrow keys)
 *          and stops yaw rotation; the pilot flies the throttle. Trim and
 *          the gain scale are kept in NVS.
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
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"
#include "freertos/semphr.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "flight.h"

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
#define MOTOR_TEST_MS           1000      /* keys 1-4: long enough to see the spin direction */
#define LINK_TIMEOUT_MS         1000      /* armed: disarm if no heartbeat 'h' arrives */
#define ARM_HB_MAX_AGE_MS       500       /* arming needs a heartbeat this fresh */
#define ARMED_IDLE_MS           20000     /* armed at throttle 0 this long: disarm */
#define STUCK_TILT_DEG          30.0f     /* armed with throttle up, tilted beyond this ... */
#define STUCK_MS                2000      /* ... this long: disarm (no setpoints: never in normal flight) */
#define CAL_LEVEL_DEG           5.0f      /* level calibration only on a surface this level */
#define LINK_IDLE_DROP_MS       5000      /* connected, no heartbeat this long: drop the link */
#define MAX_THROTTLE            220       /* head-room above this for PID    */
#define THROTTLE_STEP           5
#define ARM_TILT_LIMIT_DEG      5.0f      /* refuse to arm if tilted       */
#define KILL_TILT_LIMIT_DEG     50.0f     /* auto-disarm on big tilt       */

#define STEER_DEG               4.0f      /* arrow key: tilt this far ... */
#define STEER_FIRST_MS          700       /* ... this long: a terminal repeats a held key only after 500-660 ms */
#define STEER_HOLD_MS           150       /* each repeat of a held key: this much longer */
#define TRIM_STEP_DEG           0.5f      /* keys i/k/j/l: level offset per press */
#define TRIM_MAX_DEG            10.0f
#define GAIN_STEP               1.25f     /* keys [ ]: rate-loop gain scale */
#define GAIN_MIN                0.25f
#define GAIN_MAX                4.0f

/* 500 Hz on 800 Hz IMU data: the motors lag 70-140 ms, so every ms of
 * sensor and loop delay costs stability margin (tools/hover_sim.c) */
#define CTRL_RATE_HZ            500
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
 * (~0.1 mA for the whole board), waking every 5 min (every 30 min below
 * VBAT_EMPTY_MV); back to normal above VBAT_WAKE_MV. This
 * also lets USB charging recover a flat pack (the charger trickles about 19 mA). */
#define VBAT_SLEEP_MV      3300
#define VBAT_EMPTY_MV      3000
#define VBAT_WAKE_MV       3500
#define VBAT_SLEEP_S       300
#define VBAT_SLEEP_EMPTY_S 1800      /* below VBAT_EMPTY_MV: check less often */

/* ---------- BLE control ---------- */
#define BLE_DEVICE_NAME "QuadFW"

/* ---------- BMI323 ---------- */
#define BMI323_ADDR        0x68
#define BMI323_CHIP_ID     0x00
#define BMI323_ERR_REG     0x01
#define BMI323_IO_I2C_IF   0x52
#define BMI323_ACC_DATA_X  0x03
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
static volatile bool      s_recal         = false; /* 'c': redo the level calibration */
static volatile float     s_steer[2]      = { 0 };  /* roll, pitch setpoint from the arrow keys (deg) */
static volatile uint32_t  s_steer_end_ms[2] = { 0 };
static volatile float     s_trim[2]       = { 0 };  /* roll, pitch level offset (deg), in NVS */
static volatile float     s_gain          = 1.0f;   /* rate-loop gain scale, in NVS */
static bool               s_tune_dirty    = false;  /* trim/gain changed, not saved yet */
static volatile uint32_t  s_last_hb_ms    = 0;     /* last heartbeat 'h' from the client */
static volatile uint32_t  s_conn_ms       = 0;     /* when the current client connected */
static uint16_t           s_imu_err       = 0;     /* ERR_REG after configuring the IMU */
static int                s_reset_reason  = 0;
static uint16_t           s_chip_id       = 0;
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
                                                rx, 2 + n, 3);   /* ms; a burst takes 0.4 */
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

/* An IMU left mid-transfer by an ESP reset can hold SDA low: clock it out
 * (up to 9 SCL pulses) and send a STOP before the driver takes the pins. */
static void i2c_unstick(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << I2C_SCL_GPIO) | (1ULL << I2C_SDA_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD, .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_set_level(I2C_SDA_GPIO, 1);   /* released before the outputs turn on */
    gpio_set_level(I2C_SCL_GPIO, 1);
    gpio_config(&io);
    esp_rom_delay_us(5);
    for (int i = 0; i < 9 && !gpio_get_level(I2C_SDA_GPIO); ++i) {
        gpio_set_level(I2C_SCL_GPIO, 0); esp_rom_delay_us(5);
        gpio_set_level(I2C_SCL_GPIO, 1); esp_rom_delay_us(5);
    }
    gpio_set_level(I2C_SCL_GPIO, 0); esp_rom_delay_us(5);   /* STOP: SDA rises while SCL is high */
    gpio_set_level(I2C_SDA_GPIO, 0); esp_rom_delay_us(5);
    gpio_set_level(I2C_SCL_GPIO, 1); esp_rom_delay_us(5);
    gpio_set_level(I2C_SDA_GPIO, 1); esp_rom_delay_us(5);
}

static void i2c_bring_up(void)
{
    i2c_unstick();
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
 *   TX char (N)   6E400003-B5A3-F393-E0A9-E50E24DCCA9E   <- log lines
 *
 * Same single-byte protocol as before:
 *   '1'..'4' -> pulse that motor for 1 s
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
static SemaphoreHandle_t    s_log_mux = NULL;   /* several tasks log; the buffer takes one writer */

static void ble_advertise(void);
static void log_status(void);

/* trim + gain survive a reboot. Saved only while disarmed: a flash write
 * stalls the control loop for a few ms. */
typedef struct { float trim[2], gain; } tune_t;

static void tune_load(void)
{
    nvs_handle_t h;
    tune_t t;
    size_t n = sizeof t;
    if (nvs_open("quad", NVS_READONLY, &h) != ESP_OK) return;
    if (nvs_get_blob(h, "tune", &t, &n) == ESP_OK && n == sizeof t &&
        fabsf(t.trim[0]) <= TRIM_MAX_DEG && fabsf(t.trim[1]) <= TRIM_MAX_DEG &&
        t.gain >= GAIN_MIN && t.gain <= GAIN_MAX) {
        s_trim[0] = t.trim[0]; s_trim[1] = t.trim[1]; s_gain = t.gain;
    }
    nvs_close(h);
}

static void tune_save(void)
{
    nvs_handle_t h;
    tune_t t = { { s_trim[0], s_trim[1] }, s_gain };
    s_tune_dirty = false;
    if (nvs_open("quad", NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_blob(h, "tune", &t, sizeof t) == ESP_OK) nvs_commit(h);
    nvs_close(h);
}

static void handle_cmd_byte(uint8_t c)
{
    if (c == 'h') {   /* heartbeat from the client (tools/motor_keys.py) */
        s_last_hb_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        return;
    }
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
            uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            s_conn_ms = now;
            s_last_hb_ms = now - 10 * LINK_TIMEOUT_MS;   /* this client must send its own */
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
            if (s_notify_enabled) log_status();   /* boot logs went nowhere */
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
        if (s_log_mux && xSemaphoreTake(s_log_mux, 2) == pdTRUE) {
            (void)xStreamBufferSend(s_log_sb, buf, len, 0);
            xSemaphoreGive(s_log_mux);
        }
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
    s_log_mux = xSemaphoreCreateMutex();
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
    tune_load();

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
    int secs = mv < VBAT_EMPTY_MV ? VBAT_SLEEP_EMPTY_S : VBAT_SLEEP_S;
    ESP_LOGW(TAG, "battery %d mV: deep sleep, next check in %d s", mv, secs);
    vTaskDelay(pdMS_TO_TICKS(200));   /* let the log line out */
    if (s_ble_up) {
        s_notify_enabled = false;   /* the log tee stops sending */
        if (nimble_port_stop() == 0) nimble_port_deinit();
    }
    bmi_write_u16(BMI323_ACC_CONF, 0x0000);   /* accel + gyro off */
    bmi_write_u16(BMI323_GYR_CONF, 0x0000);
    esp_sleep_enable_timer_wakeup((uint64_t)secs * 1000000u);
    esp_deep_sleep_start();
}

/* '?', and whenever a client subscribes: what happened at boot */
static void log_status(void)
{
    ESP_LOGI(TAG, "status: reset reason %d%s, IMU id 0x%04x err 0x%04x, level cal %s, battery %d mV, "
                  "trim r %+.1f p %+.1f, gain %.2f, %s",
             s_reset_reason, s_reset_reason == ESP_RST_BROWNOUT ? " (BROWN-OUT)" : "",
             s_chip_id, s_imu_err, s_ready ? "done" : "waiting for the board to be still and level",
             s_vbat_mv, s_trim[0], s_trim[1], s_gain, s_armed ? "ARMED" : "disarmed");
}

/* -------------------- motor command consumer -------------------- */
static void motor_cmd_task(void *arg)
{
    uint8_t c;
    for (;;) {
        if (xQueueReceive(s_cmd_q, &c, pdMS_TO_TICKS(500)) != pdTRUE) c = 0;
        if (s_tune_dirty && !s_armed) tune_save();   /* also after an automatic disarm */
        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        switch (c) {
        case 0:
            break;
        case '^': case 'v': case '<': case '>':   /* arrow keys: steer while armed */
            if (s_armed) {
                int ax = c == '^' || c == 'v';   /* 0 roll, 1 pitch */
                float v = c == '^' || c == '>' ? STEER_DEG : -STEER_DEG;   /* nose down / right side down */
                bool held = (int32_t)(s_steer_end_ms[ax] - now_ms) > 0 && s_steer[ax] == v;
                uint32_t end = now_ms + (held ? STEER_HOLD_MS : STEER_FIRST_MS);
                s_steer[ax] = v;
                if (!held || (int32_t)(end - s_steer_end_ms[ax]) > 0) s_steer_end_ms[ax] = end;
            }
            break;
        case 'i': case 'k': case 'j': case 'l': {   /* trim: toward where it should go */
            int ax = c == 'i' || c == 'k';
            float t = s_trim[ax] + (c == 'i' || c == 'l' ? TRIM_STEP_DEG : -TRIM_STEP_DEG);
            s_trim[ax] = fmaxf(-TRIM_MAX_DEG, fminf(TRIM_MAX_DEG, t));
            s_tune_dirty = true;
            ESP_LOGI(TAG, "cmd: trim r %+.1f p %+.1f deg", s_trim[0], s_trim[1]);
            break;
        }
        case '[': case ']':
            s_gain = fmaxf(GAIN_MIN, fminf(GAIN_MAX, c == ']' ? s_gain * GAIN_STEP : s_gain / GAIN_STEP));
            s_tune_dirty = true;
            ESP_LOGI(TAG, "cmd: gain %.2f", s_gain);
            break;
        case 'a':
            if (s_armed) {
                ESP_LOGW(TAG, "cmd: already armed");   /* never reset throttle mid-air */
            } else if (s_recal || !s_ready || s_conn_handle == 0xFFFF) {   /* s_recal first: see the control task */
                ESP_LOGW(TAG, "cmd: arm refused, not ready (level cal needs the board still and level)");
            } else if ((int32_t)((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - s_last_hb_ms) > ARM_HB_MAX_AGE_MS) {
                ESP_LOGW(TAG, "cmd: arm refused, no heartbeat from the client");
            } else if (s_vbat_mv < VBAT_MIN_ARM_MV) {   /* 0 = can't measure it */
                ESP_LOGW(TAG, "cmd: arm refused, battery %d mV", s_vbat_mv);
            } else if (fabsf(s_roll_deg) < ARM_TILT_LIMIT_DEG &&
                fabsf(s_pitch_deg) < ARM_TILT_LIMIT_DEG) {
                bool ok = false;
                taskENTER_CRITICAL(&s_arm_mux);
                if (s_conn_handle != 0xFFFF) {   /* link still up */
                    s_base_throttle = 0;         /* always start from idle */
                    s_test_end_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                    s_steer_end_ms[0] = s_steer_end_ms[1] = s_test_end_ms;   /* no steering left over */
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
            s_test_end_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);   /* ends a motor test */
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
                static const char *const corner[4] = { "front right", "back right", "back left", "front left" };
                int m = c - '1';   /* end time first: the control task reads both */
                s_test_end_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) + MOTOR_TEST_MS;
                s_test_motor = (int8_t)m;
                ESP_LOGI(TAG, "cmd: test motor M%d (%s)", m, corner[m]);
            } else {
                ESP_LOGW(TAG, "cmd: ignored '%c' while armed", c);
            }
            break;
        case 'c':
            if (!s_armed) {
                s_recal = true;
                ESP_LOGI(TAG, "cmd: level calibration restarts (keep the board still and level)");
            } else {
                ESP_LOGW(TAG, "cmd: 'c' ignored while armed");
            }
            break;
        case '?':
            log_status();
            break;
        default:
            ESP_LOGW(TAG, "cmd: ignore 0x%02x", (unsigned)c);
            break;
        }
    }
}

/* -------------------- 500 Hz control loop --------------------
 * 1. Read accel + gyro.
 * 2. flight_estimate -> roll/pitch; yaw gyro-integrated (log only).
 * 3. Auto-disarm on extreme tilt.
 * 4. If armed: flight_control (trim + steering setpoints) -> 4 PWM outputs.
 *    If disarmed: motors 0, except an optional motor-test pulse.
 * 5. Print state at 10 Hz, blink LED at 1 Hz.
 */
static void control_task(void *arg)
{
    const float GYR_LSB_TO_DPS  = 1000.0f / 32768.0f;
    const float ACC_LSB_TO_G    = 8.0f / 32768.0f;   /* ACC_CONF range = +-8 g */
    const float RAD_TO_DEG      = 180.0f / (float)M_PI;

    flight_t fl = { 0 };
    float yaw_deg = 0.0f;
    int64_t last_ok_us = esp_timer_get_time();   /* last good IMU sample */
    bool  led_on = false;
    int   print_div = 0;
    int   blink_div = 0;
    int   i2c_err_streak = 0;
    int   vbat_div = 0, vbat_low_n = 0, vbat_empty_n = 0;
    bool  vbat_seen = false;

    /* Level calibration: average CAL_N consecutive samples taken while the
     * board is still (every gyro axis below 5 dps, the gravity vector steady
     * within 0.035 g, i.e. about 2 degrees)
     * and roughly level (both angles within CAL_LEVEL_DEG, right side up);
     * anything else restarts the count. Their mean accel angle is "level"
     * (the IMU is never mounted perfectly flat) and their mean gyro reading
     * is the gyro bias. Arming is blocked until it is done; 'c' redoes it. */
    const int CAL_N = CTRL_RATE_HZ;   /* 1 s */
    int   cal_count = 0;
    float cal_roll_sum = 0.0f, cal_pitch_sum = 0.0f;
    float cal_gx = 0.0f, cal_gy = 0.0f, cal_gz = 0.0f;
    float roll_bias = 0.0f, pitch_bias = 0.0f;
    float gx_bias = 0.0f, gy_bias = 0.0f, gz_bias = 0.0f;
    float cal_ax0 = 0.0f, cal_ay0 = 0.0f, cal_az0 = 0.0f, cal_g = 0.0f;
    bool  calibrated = false;
    uint32_t idle_since_ms = 0, stall_since_ms = 0;

    /* No good IMU sample for this long while armed: the bus is compromised
     * (most likely motor EMI), disarm rather than hold stale motor duties. */
    const int64_t I2C_LOSS_US = 25000;

    for (;;) {
        uint8_t d[12];   /* ACC X Y Z, GYR X Y Z: one burst, one sample */
        esp_err_t err = bmi_read(BMI323_ACC_DATA_X, d, sizeof(d));
        const uint8_t *acc = d, *gyr = d + 6;

        int64_t now_us = esp_timer_get_time();

        if (err == ESP_OK) {
            /* since the last good sample: about 2 ms, longer after failed reads */
            float dt = fminf((float)(now_us - last_ok_us) * 1e-6f, 0.02f);
            last_ok_us = now_us;
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
            float gyr[3] = { gxi * GYR_LSB_TO_DPS - gx_bias, gyi * GYR_LSB_TO_DPS - gy_bias,
                             gzi * GYR_LSB_TO_DPS - gz_bias };

            float roll_acc_raw  = atan2f(ay, az);
            float pitch_acc_raw = atan2f(-ax, sqrtf(ay*ay + az*az));

            if (s_recal) {
                if (!s_armed) {
                    s_ready = false;   /* before s_recal clears: no 'a' slips in between */
                    calibrated = false; cal_count = 0; gx_bias = gy_bias = gz_bias = 0.0f;
                }
                s_recal = false;
            }
            if (!calibrated) {
                float gxr = gxi * GYR_LSB_TO_DPS, gyr_ = gyi * GYR_LSB_TO_DPS, gzr = gzi * GYR_LSB_TO_DPS;
                const float lvl = CAL_LEVEL_DEG / RAD_TO_DEG;
                bool ok = fabsf(gxr) < 5.0f && fabsf(gyr_) < 5.0f && fabsf(gzr) < 5.0f && az > 0.0f &&
                          fabsf(roll_acc_raw) < lvl && fabsf(pitch_acc_raw) < lvl;
                float dax = ax - cal_ax0, day = ay - cal_ay0, daz = az - cal_az0;
                bool moved = dax*dax + day*day + daz*daz > 0.035f * 0.035f;   /* ~2 deg; 800 Hz noise is ~5 mg */
                if (cal_count > 0 && (!ok || moved)) cal_count = 0;   /* start over */
                if (ok && cal_count == 0) {
                    cal_ax0 = ax; cal_ay0 = ay; cal_az0 = az;
                    cal_roll_sum = cal_pitch_sum = cal_gx = cal_gy = cal_gz = cal_g = 0.0f;
                }
                if (ok) {
                    cal_roll_sum  += roll_acc_raw;
                    cal_pitch_sum += pitch_acc_raw;
                    cal_gx += gxr; cal_gy += gyr_; cal_gz += gzr;
                    cal_g += sqrtf(ax*ax + ay*ay + az*az);
                    if (++cal_count >= CAL_N) {
                        roll_bias  = cal_roll_sum  / (float)CAL_N;
                        pitch_bias = cal_pitch_sum / (float)CAL_N;
                        gx_bias = cal_gx / (float)CAL_N;
                        gy_bias = cal_gy / (float)CAL_N;
                        gz_bias = cal_gz / (float)CAL_N;
                        calibrated = true;
                        fl.roll0  = roll_bias  * RAD_TO_DEG;
                        fl.pitch0 = pitch_bias * RAD_TO_DEG;
                        fl.g1 = cal_g / (float)CAL_N;   /* this IMU's 1 g, for the lift-off check */
                        fl.init = false;   /* restart the estimate from the calibrated level */
                        ESP_LOGI(TAG,
                            "level cal done: roll %+.2f pitch %+.2f deg, gyro bias %+.2f %+.2f %+.2f dps",
                            roll_bias * RAD_TO_DEG, pitch_bias * RAD_TO_DEG, gx_bias, gy_bias, gz_bias);
                    }
                }
            }

            /* the gyro bias drifts as the board warms up: keep following it
             * while the quad sits disarmed and still */
            if (calibrated && !s_armed && s_test_motor < 0 &&
                fabsf(gyr[0]) < 1.0f && fabsf(gyr[1]) < 1.0f && fabsf(gyr[2]) < 1.0f) {
                float k = dt / 5.0f;
                gx_bias += k * gyr[0]; gy_bias += k * gyr[1]; gz_bias += k * gyr[2];
            }

            float acc_g[3] = { ax, ay, az };
            flight_estimate(&fl, acc_g, gyr, dt);
            yaw_deg += gyr[2] * dt;
            float roll_deg  = fl.roll;
            float pitch_deg = fl.pitch;
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

            /* Safety while armed: the client went silent, idle too long */
            if (s_armed && (int32_t)(now_ms - s_last_hb_ms) > LINK_TIMEOUT_MS) {
                s_armed = false;
                s_base_throttle = 0;
                for (int i = 0; i < 4; ++i) motor_set_duty(i, 0);
                ESP_LOGW(TAG, "ctrl: no heartbeat for %d ms -> DISARM", LINK_TIMEOUT_MS);
            }
            /* a crashed client can leave the link up (BlueZ does), and then
             * the board never advertises again: drop a silent link */
            uint16_t conn = s_conn_handle;
            if (conn != 0xFFFF && (int32_t)(now_ms - s_conn_ms) > LINK_IDLE_DROP_MS &&
                (int32_t)(now_ms - s_last_hb_ms) > LINK_IDLE_DROP_MS) {
                s_conn_ms = now_ms;   /* once per LINK_IDLE_DROP_MS at most */
                ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
            }
            if (!s_armed || s_base_throttle != 0) idle_since_ms = now_ms;
            else if (now_ms - idle_since_ms > ARMED_IDLE_MS) {
                s_armed = false;
                ESP_LOGW(TAG, "ctrl: armed at idle for %d s -> DISARM", ARMED_IDLE_MS / 1000);
            }

            /* armed at throttle 0 the motors stay off (and the integrators clear) */
            float sp[2];
            for (int k = 0; k < 2; ++k)
                sp[k] = s_trim[k] + ((int32_t)(s_steer_end_ms[k] - now_ms) > 0 ? s_steer[k] : 0.0f);
            flight_control(&fl, s_armed ? (float)s_base_throttle : 0.0f, sp[0], sp[1], s_gain, gyr, dt, out);
            if (!s_armed && s_test_motor >= 0 && s_test_motor < 4 &&
                (int32_t)(s_test_end_ms - now_ms) > 0) {
                out[s_test_motor] = MOTOR_TEST_DUTY;
            } else if (s_test_motor >= 0) {
                s_test_motor = -1;
            }

            /* armed with throttle up and tilted well over for 2 s (held in
             * grass, against a wall): the low side's motors sit at full power
             * into a stall -> save the FETs and motors. Trim + steering stay
             * under 15 deg, so a free-flying quad never stays this tilted. */
            bool stuck = s_armed && s_base_throttle > 0 &&
                         (fabsf(roll_deg) > STUCK_TILT_DEG || fabsf(pitch_deg) > STUCK_TILT_DEG);
            if (!stuck) {
                stall_since_ms = now_ms;
            } else if (now_ms - stall_since_ms > STUCK_MS) {
                s_armed = false;
                s_base_throttle = 0;
                for (int i = 0; i < 4; ++i) out[i] = 0;
                ESP_LOGW(TAG, "ctrl: stuck tilted over %d deg for %d s -> DISARM", (int)STUCK_TILT_DEG, STUCK_MS / 1000);
            }

            for (int i = 0; i < 4; ++i) motor_set_duty(i, out[i]);

            if (++print_div >= CTRL_RATE_HZ / 10) {
                print_div = 0;
                ESP_LOGI(TAG,
                       "%s T=%3u r=%+6.1f p=%+6.1f y=%+6.1f | "
                       "%3d %3d %3d %3d | I %+5.1f %+5.1f %+5.1f %s | %4d mV",
                       s_armed ? "ARM" : "dis",
                       (unsigned)s_base_throttle,
                       roll_deg, pitch_deg, yaw_deg,
                       out[0], out[1], out[2], out[3],
                       fl.i[0], fl.i[1], fl.i[2], fl.air ? "air" : "gnd", s_vbat_mv);
            }
            if (++blink_div >= CTRL_RATE_HZ / 2) {  /* 1 Hz blink */
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
                ESP_LOGW(TAG, "i2c read failed: %s (streak=%d)",
                         esp_err_to_name(err), i2c_err_streak + 1);
            }
            ++i2c_err_streak;
            if (s_armed && now_us - last_ok_us > I2C_LOSS_US) {
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
        if (++vbat_div >= CTRL_RATE_HZ / 10) {
            vbat_div = 0;
            int mv = vbat_read_mv();
            s_vbat_mv = mv;
            bool quiet = !s_armed && s_test_motor < 0;
            vbat_low_n   = quiet && mv > 0 && mv < VBAT_SLEEP_MV ? vbat_low_n + 1 : 0;
            vbat_empty_n = quiet && mv > 0 && mv < VBAT_EMPTY_MV ? vbat_empty_n + 1 : 0;
            if (vbat_low_n >= 100 || vbat_empty_n >= 10) deep_sleep(mv);
            vbat_seen = true;
        }
        s_ready = calibrated && vbat_seen;

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

    /* Battery first, before BLE and anything else draws current: an empty
     * pack, or a sleep wake that USB hasn't charged back
     * above VBAT_WAKE_MV, goes straight back to sleep. */
    esp_reset_reason_t why = esp_reset_reason();
    s_reset_reason = (int)why;
    int boot_mv = vbat_read_mv();
    bool woke = why == ESP_RST_DEEPSLEEP, low = boot_mv > 0 && boot_mv < VBAT_WAKE_MV;
    i2c_bring_up();
    if (boot_mv > 0 && (boot_mv < VBAT_EMPTY_MV || (woke && low))) {
        bmi_write_u16(BMI323_ACC_CONF, 0x0000);   /* IMU off, if it was on */
        bmi_write_u16(BMI323_GYR_CONF, 0x0000);
        int secs = boot_mv < VBAT_EMPTY_MV ? VBAT_SLEEP_EMPTY_S : VBAT_SLEEP_S;
        esp_sleep_enable_timer_wakeup((uint64_t)secs * 1000000u);
        esp_deep_sleep_start();
    }
    /* No motor spins at boot (it used to sweep all four, which also fired
     * on every battery plug-in): test them with BLE keys 1-4. */

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
    esp_err_t err = ESP_FAIL;
    for (int t = 0; t < 5 && (chip_id & 0xff) != BMI323_CHIP_ID_VAL; ++t) {
        if (t) vTaskDelay(pdMS_TO_TICKS(20));
        err = bmi_read_u16(BMI323_CHIP_ID, &chip_id);
    }
    s_chip_id = chip_id;
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
        /* Fast blink (5 Hz): alive, but no IMU. Normal is 1 Hz. */
        for (bool on = false;; on = !on) {
            led_set(on);
            int mv = vbat_read_mv();
            s_vbat_mv = mv;
            if (mv > 0 && mv < VBAT_SLEEP_MV) deep_sleep(mv);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    ESP_LOGI(TAG, "BMI323 detected, configuring for 800 Hz...");

    /* Accelerometer: high-performance mode (the one Bosch specifies noise
     * for), 800 Hz ODR, +/-8 g range, bandwidth ODR/2 (0.95 ms group delay). */
    ESP_ERROR_CHECK(bmi_write_u16(BMI323_ACC_CONF, 0x702B));
    vTaskDelay(pdMS_TO_TICKS(5));
    /* Gyroscope: high-performance mode, 800 Hz ODR, +/-1000 dps, ODR/2
     * (1.43 ms group delay; 3.72 ms at the old 200 Hz). */
    ESP_ERROR_CHECK(bmi_write_u16(BMI323_GYR_CONF, 0x703B));
    vTaskDelay(pdMS_TO_TICKS(100));
    /* sensors on (no more suspend-mode write timing): the IMU releases SDA
     * by itself if a transfer ever hangs for more than 1.25 ms */
    bmi_write_u16(BMI323_IO_I2C_IF, 0x0002);
    vTaskDelay(pdMS_TO_TICKS(300));   /* a gyro fatal_err shows within 350 ms */
    if (bmi_read_u16(BMI323_ERR_REG, &s_imu_err) != ESP_OK) {   /* fatal_err / acc_conf_err / gyr_conf_err */
        s_imu_err = 0xFFFF;   /* unknown */
        ESP_LOGE(TAG, "BMI323 ERR_REG read failed");
    }
    if (s_imu_err & 0x0061) ESP_LOGE(TAG, "BMI323 ERR_REG 0x%04x", s_imu_err);

    ESP_LOGI(TAG, "Starting control loop @ %d Hz. DISARMED. "
                  "BLE keys: a=arm d=disarm w=thr+ x=thr- arrows=steer ijkl=trim []=gain 1-4=test M0-M3 c=level cal ?=status",
             CTRL_RATE_HZ);

    /* Pin control loop to APP_CPU (core 1) so NimBLE on core 0 cannot
     * starve it (and vice versa). */
    xTaskCreatePinnedToCore(control_task, "ctrl", 6144, NULL, 8, NULL, 1);

    /* app_main returns; control_task, motor_cmd_task, and the NimBLE
     * host task all keep running. */
    vTaskDelete(NULL);
}
