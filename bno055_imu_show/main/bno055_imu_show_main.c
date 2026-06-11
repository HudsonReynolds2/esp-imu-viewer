/*
 * bno055_imu_show_main.c
 *
 * Reads the fused orientation quaternion and calibration status from a Bosch
 * BNO055 (DFRobot SEN0253 "10 DOF IMU AHRS") over I2C using the ESP-IDF v5.x
 * i2c_master driver, and prints them once per cycle as a minimal, label-free,
 * comma-separated line for the imu_view.py visualizer:
 *
 *     qw,qx,qy,qz,sys,gyr,acc,mag\r\n   at 115200 baud
 *
 * qw..qz are the normalized unit quaternion (%.4f); sys/gyr/acc/mag are the
 * 0-3 calibration levels. Rotating from the quaternion avoids the gimbal lock
 * that Euler angles hit near +/-90 deg pitch.
 *
 * Structured after the ESP-IDF i2c_basic example, but targeting the BNO055
 * (addr 0x28, chip id 0xA0) instead of the MPU9250. The BNO055 performs sensor
 * fusion on-chip in NDOF mode, so we just read the fused quaternion registers.
 *
 * Steady-state loop performs no heap allocation -> nothing to leak.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"

/* ---- User configuration ------------------------------------------------- */
/* XIAO ESP32C3 silkscreen D4 = GPIO6 (SDA), D5 = GPIO7 (SCL). Change if you
 * wired the sensor to different pins. */
#define I2C_MASTER_SCL_IO        7
#define I2C_MASTER_SDA_IO        6
#define I2C_MASTER_FREQ_HZ       100000      /* 100 kHz is safe for BNO055   */
#define I2C_MASTER_TIMEOUT_MS    1000

#define BNO055_ADDR              0x28        /* COM3 low on DFRobot board    */

/* ---- BNO055 register map (page 0) --------------------------------------- */
#define BNO055_REG_CHIP_ID       0x00        /* should read 0xA0             */
#define BNO055_REG_PAGE_ID       0x07
#define BNO055_REG_EUL_HEAD_LSB  0x1A        /* Euler X (heading / yaw)      */
#define BNO055_REG_EUL_ROLL_LSB  0x1C        /* Euler Y (roll)               */
#define BNO055_REG_EUL_PITCH_LSB 0x1E        /* Euler Z (pitch)              */
#define BNO055_REG_QUA_DATA_W_LSB 0x20       /* Quaternion W,X,Y,Z (8 bytes) */
#define BNO055_REG_CALIB_STAT    0x35        /* SYS/GYR/ACC/MAG, 2 bits each */
#define BNO055_REG_UNIT_SEL      0x3B
#define BNO055_REG_OPR_MODE      0x3D
#define BNO055_REG_PWR_MODE      0x3E
#define BNO055_REG_SYS_TRIGGER   0x3F

#define BNO055_CHIP_ID_VALUE     0xA0

#define BNO055_OPR_MODE_CONFIG   0x00
#define BNO055_OPR_MODE_NDOF     0x0C
#define BNO055_PWR_MODE_NORMAL   0x00

/* Euler output: 1 degree = 16 LSB (datasheet 3.6.5.4, default unit). */
#define BNO055_EUL_LSB_PER_DEG   16.0f

/* Quaternion output: unit quaternion = 2^14 LSB (datasheet 3.6.5.5). */
#define BNO055_QUAT_LSB          16384.0f

static const char *TAG = "bno055";

static i2c_master_dev_handle_t s_dev;

/* ---- Low-level helpers -------------------------------------------------- */
static esp_err_t bno_write_u8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t bno_read(uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, dst, len,
                                       I2C_MASTER_TIMEOUT_MS);
}

/* Wait for the chip-id register to return 0xA0. The BNO055 takes ~650 ms after
 * power-on before it answers, and the DFRobot board may still be settling when
 * the ESP32 finishes booting, so we poll instead of aborting on the first read
 * (this is exactly the failure you saw with the MPU9250 example). */
static esp_err_t bno_wait_for_chip(void)
{
    uint8_t id = 0;
    for (int attempt = 0; attempt < 20; ++attempt) {
        if (bno_read(BNO055_REG_CHIP_ID, &id, 1) == ESP_OK &&
            id == BNO055_CHIP_ID_VALUE) {
            ESP_LOGI(TAG, "BNO055 detected (chip id 0x%02X)", id);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "waiting for BNO055 (attempt %d, last id 0x%02X)",
                 attempt + 1, id);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t bno_init(void)
{
    esp_err_t err;

    ESP_RETURN_ON_ERROR(bno_wait_for_chip(), TAG, "chip not found");

    /* Make sure we are on register page 0. */
    ESP_RETURN_ON_ERROR(bno_write_u8(BNO055_REG_PAGE_ID, 0x00),
                        TAG, "page select failed");

    /* Enter CONFIG mode before changing any configuration. Mode switches into
     * CONFIG need up to 19 ms (datasheet 3.3.1, table 3-6). */
    ESP_RETURN_ON_ERROR(bno_write_u8(BNO055_REG_OPR_MODE, BNO055_OPR_MODE_CONFIG),
                        TAG, "config mode failed");
    vTaskDelay(pdMS_TO_TICKS(30));

    /* Soft reset clears any stale state from a previous run, then wait for the
     * chip to come back up. */
    ESP_RETURN_ON_ERROR(bno_write_u8(BNO055_REG_SYS_TRIGGER, 0x20),
                        TAG, "reset failed");
    vTaskDelay(pdMS_TO_TICKS(700));
    ESP_RETURN_ON_ERROR(bno_wait_for_chip(), TAG, "chip not back after reset");

    /* Normal power mode. */
    ESP_RETURN_ON_ERROR(bno_write_u8(BNO055_REG_PWR_MODE, BNO055_PWR_MODE_NORMAL),
                        TAG, "power mode failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Use internal oscillator (clear external-crystal bit). */
    ESP_RETURN_ON_ERROR(bno_write_u8(BNO055_REG_SYS_TRIGGER, 0x00),
                        TAG, "osc select failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Default units (Euler in degrees, Android orientation) are already what
     * the visualizer expects, so we leave UNIT_SEL at its reset value 0x00. */

    /* Enter NDOF fusion mode. Switching out of CONFIG needs up to 7 ms. */
    err = bno_write_u8(BNO055_REG_OPR_MODE, BNO055_OPR_MODE_NDOF);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NDOF mode failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(30));

    ESP_LOGI(TAG, "BNO055 in NDOF fusion mode");
    return ESP_OK;
}

/* Read the four quaternion registers (8 bytes, contiguous from 0x20) in one
 * burst and convert to a normalized unit quaternion. Register order is
 * W, X, Y, Z, each a little-endian int16 (datasheet 4.3.x). */
static esp_err_t bno_read_quaternion(float *w, float *x, float *y, float *z)
{
    uint8_t raw[8];
    esp_err_t err = bno_read(BNO055_REG_QUA_DATA_W_LSB, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    int16_t qw = (int16_t)((raw[1] << 8) | raw[0]);   /* 0x20/0x21 */
    int16_t qx = (int16_t)((raw[3] << 8) | raw[2]);   /* 0x22/0x23 */
    int16_t qy = (int16_t)((raw[5] << 8) | raw[4]);   /* 0x24/0x25 */
    int16_t qz = (int16_t)((raw[7] << 8) | raw[6]);   /* 0x26/0x27 */

    *w = qw / BNO055_QUAT_LSB;
    *x = qx / BNO055_QUAT_LSB;
    *y = qy / BNO055_QUAT_LSB;
    *z = qz / BNO055_QUAT_LSB;
    return ESP_OK;
}

/* Read the single CALIB_STAT byte and unpack its four 2-bit fields. Each value
 * is 0 (uncalibrated) to 3 (fully calibrated). In NDOF the fused quaternion is
 * only trustworthy as an absolute orientation once these reach 3, so we stream
 * them for the visualizer to display. */
static esp_err_t bno_read_calib(uint8_t *sys, uint8_t *gyr, uint8_t *acc,
                                uint8_t *mag)
{
    uint8_t c = 0;
    esp_err_t err = bno_read(BNO055_REG_CALIB_STAT, &c, 1);
    if (err != ESP_OK) {
        return err;
    }
    *sys = (c >> 6) & 0x03;
    *gyr = (c >> 4) & 0x03;
    *acc = (c >> 2) & 0x03;
    *mag = c & 0x03;
    return ESP_OK;
}

void app_main(void)
{
    /* Belt-and-suspenders with sdkconfig: silence any IDF logging on the
     * console UART so the only thing we emit is the data lines below. A strict
     * line parser falls out of sync on any unexpected line and then drops input
     * for seconds (the symptom: model updates only every 20-30 s). The ESP-ROM
     * boot banner is printed by hardware before app_main and cannot be
     * suppressed in firmware; the visualizer ignores any line that is not a
     * valid data line, so it is robust to the banner regardless. */
    esp_log_level_set("*", ESP_LOG_NONE);

    /* ---- I2C master bus + device, created once ------------------------- */
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,                 /* auto-select a free port          */
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO055_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &s_dev));
    ESP_LOGI(TAG, "I2C initialized on SDA=%d SCL=%d", I2C_MASTER_SDA_IO,
             I2C_MASTER_SCL_IO);

    if (bno_init() != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 init failed; check wiring (SDA/SCL/3V3/GND) "
                      "and that the board is at address 0x28.");
        /* Don't abort the whole app; idle so the log stays readable. */
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    /* ---- Steady-state loop: no allocation, fixed stack buffers --------- */
    float qw, qx, qy, qz;
    uint8_t sys, gyr, acc, mag;
    char line[80];
    while (1) {
        esp_err_t qerr = bno_read_quaternion(&qw, &qx, &qy, &qz);
        esp_err_t cerr = bno_read_calib(&sys, &gyr, &acc, &mag);
        if (qerr == ESP_OK && cerr == ESP_OK) {
            /* Minimal comma-separated line, no field labels, to keep the byte
             * count low at high stream rates. Order is fixed:
             *   qw,qx,qy,qz,sys,gyr,acc,mag\r\n
             * Quaternion as %.4f (ample for a unit quaternion); cal levels are
             * single digits 0-3. We match the Arduino-style CR+LF terminator;
             * CONFIG_LIBC_STDOUT_LINE_ENDING_LF in sdkconfig.defaults stops the
             * console from adding a second CR. */
            int n = snprintf(line, sizeof(line),
                             "%.4f,%.4f,%.4f,%.4f,%u,%u,%u,%u\r\n",
                             qw, qx, qy, qz, sys, gyr, acc, mag);
            if (n > 0) {
                fwrite(line, 1, (size_t)n, stdout);
                fflush(stdout);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));   /* ~100 Hz, matches BNO055 NDOF fusion */
    }
}