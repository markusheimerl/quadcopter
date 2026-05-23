/*
 * XIAO ESP32-S3 Sense -- blink + Bosch BMI323 IMU readout over I2C.
 *
 *   LED  : on-board user LED, GPIO21, active-LOW.       Driven via raw
 *          GPIO register writes (no driver).
 *   IMU  : BMI323 on I2C0, SDA=GPIO5, SCL=GPIO6, 400 kHz.
 *          Uses ESP-IDF's i2c_master driver (writing a register-level
 *          I2C driver is a separate project).
 *
 * BMI323 quirk: every register read returns 2 dummy bytes followed by
 * the 16-bit register value, LSB first. Every register write is 1
 * address byte + 16-bit value LSB-first.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---------- LED ---------- */
#define LED_GPIO        21u
#define LED_BIT         (1u << LED_GPIO)

/* ---------- I2C ---------- */
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    5
#define I2C_SCL_GPIO    6
#define I2C_FREQ_HZ     400000

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

/* -------------------- BMI323 over I2C -------------------- */
static esp_err_t bmi_read(uint8_t reg, uint8_t *dst, size_t n)
{
    /* 2 dummy bytes precede every register's data. */
    uint8_t rx[2 + 12];
    if (n > sizeof(rx) - 2) {
        return ESP_ERR_INVALID_SIZE;
    }
    esp_err_t err = i2c_master_transmit_receive(s_bmi, &reg, 1,
                                                rx, 2 + n, 100);
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
    return i2c_master_transmit(s_bmi, tx, sizeof(tx), 100);
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

/* -------------------- app -------------------- */
void app_main(void)
{
    led_init();
    i2c_bring_up();

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
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    ESP_LOGI(TAG, "BMI323 detected, configuring...");

    /* Accelerometer: normal mode, 100 Hz ODR, +/-4 g range. */
    ESP_ERROR_CHECK(bmi_write_u16(BMI323_ACC_CONF, 0x4028));
    vTaskDelay(pdMS_TO_TICKS(5));
    /* Gyroscope:     normal mode, 100 Hz ODR, +/-1000 dps range. */
    ESP_ERROR_CHECK(bmi_write_u16(BMI323_GYR_CONF, 0x4038));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Streaming IMU at 10 Hz, LED toggles at 1 Hz");

    bool led_on = false;
    int blink_div = 0; /* toggle LED every 5 samples == 500 ms */

    for (;;) {
        uint8_t acc[6], gyr[6];
        esp_err_t e1 = bmi_read(BMI323_ACC_DATA_X, acc, sizeof(acc));
        esp_err_t e2 = bmi_read(BMI323_GYR_DATA_X, gyr, sizeof(gyr));

        if (e1 == ESP_OK && e2 == ESP_OK) {
            int16_t ax = (int16_t)(acc[0] | (acc[1] << 8));
            int16_t ay = (int16_t)(acc[2] | (acc[3] << 8));
            int16_t az = (int16_t)(acc[4] | (acc[5] << 8));
            int16_t gx = (int16_t)(gyr[0] | (gyr[1] << 8));
            int16_t gy = (int16_t)(gyr[2] | (gyr[3] << 8));
            int16_t gz = (int16_t)(gyr[4] | (gyr[5] << 8));
            printf("ACC %7d %7d %7d | GYR %7d %7d %7d\n",
                   ax, ay, az, gx, gy, gz);
        } else {
            ESP_LOGW(TAG, "i2c read failed: acc=%s gyr=%s",
                     esp_err_to_name(e1), esp_err_to_name(e2));
        }

        if (++blink_div >= 5) {
            blink_div = 0;
            led_on = !led_on;
            led_set(led_on);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
