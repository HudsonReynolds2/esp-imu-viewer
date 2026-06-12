/*
 * bno055_imu_show_main.c
 *
 * Reads all BNO055 outputs (fused quaternion, raw accel/gyro/mag, linear
 * acceleration, gravity, temperature, calibration status) from a Bosch BNO055
 * (DFRobot SEN0253 "10 DOF IMU AHRS") over I2C using the ESP-IDF v5.x
 * i2c_master driver, in ONE 46-byte burst read per cycle, and prints a
 * version-prefixed, positional CSV line for the imu_view.py visualizer:
 *
 *   v2,seq,t_us,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,lx,ly,lz,grx,gry,grz,temp,cs,cg,ca,cm
 *
 * seq is a uint32 sample counter (dropped-frame detection); t_us is the uint64
 * device timestamp in microseconds (authoritative for rate/jitter). The full
 * field list and units are documented at the steady-state loop below. Rotating
 * from the quaternion avoids the gimbal lock that Euler hits near +/-90 deg.
 *
 * Structured after the ESP-IDF i2c_basic example, but targeting the BNO055
 * (addr 0x28, chip id 0xA0) instead of the MPU9250. The BNO055 performs sensor
 * fusion on-chip in NDOF mode (fixed 100 Hz fusion output), so we just read its
 * data registers; nothing host-side recomputes orientation.
 *
 * Steady-state loop performs no heap allocation -> nothing to leak.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/usb_serial_jtag.h"

/* ---- User configuration ------------------------------------------------- */
/* DIAG: compile-time gate for the timing diagnostics (the boot "# diag" line
 * and the every-100th-sample "# t read_us/print_us/busy_us" line). These are
 * the instrumentation that found the blocking-USB-write bottleneck; keep the
 * code, but default OFF so the production stream carries only v2 data lines.
 * Set to 1 and rebuild to re-measure loop timing. Healthy reference numbers
 * from the working 100 Hz build: read_us ~1600-7000, print_us ~850-950,
 * busy_us ~2500-7800 (budget is 10000 us). */
#define DIAG 0

/* XIAO ESP32C3 silkscreen D4 = GPIO6 (SDA), D5 = GPIO7 (SCL). Change if you
 * wired the sensor to different pins. */
#define I2C_MASTER_SCL_IO        7
#define I2C_MASTER_SDA_IO        6
#define I2C_MASTER_FREQ_HZ       400000      /* 400 kHz (fast mode). The 46-byte
                                              * burst at 100 kHz plus the
                                              * BNO055's clock-stretching exceeds
                                              * the 10 ms window and caps the
                                              * rate (~71 Hz). 400 kHz cuts the
                                              * transfer ~4x so the read fits
                                              * inside 10 ms and the loop holds
                                              * 100 Hz. The BNO055 supports fast
                                              * mode (datasheet 4.1). */
#define I2C_MASTER_TIMEOUT_MS    1000

#define BNO055_ADDR              0x28        /* COM3 low on DFRobot board    */

/* ---- BNO055 register map (page 0) --------------------------------------- */
#define BNO055_REG_CHIP_ID       0x00        /* should read 0xA0             */
#define BNO055_REG_PAGE_ID       0x07

/* All sensor + fusion data registers live in one contiguous block on page 0,
 * each value a little-endian int16 (low byte first), except TEMP and CALIB
 * which are single bytes. Because they are contiguous and the BNO055
 * auto-increments its register pointer during a read, we pull the ENTIRE span
 * (ACC..CALIB) in ONE burst transaction and slice each sensor out by offset.
 * That is one I2C clock-stretch penalty per cycle instead of nine. */
#define BNO055_REG_ACC_DATA_X_LSB  0x08      /* raw accel   X,Y,Z  (6 bytes) */
#define BNO055_REG_MAG_DATA_X_LSB  0x0E      /* raw mag     X,Y,Z  (6 bytes) */
#define BNO055_REG_GYR_DATA_X_LSB  0x14      /* raw gyro    X,Y,Z  (6 bytes) */
#define BNO055_REG_EUL_HEAD_LSB    0x1A      /* fused Euler H,R,P  (6 bytes) */
#define BNO055_REG_QUA_DATA_W_LSB  0x20      /* fused quat  W,X,Y,Z(8 bytes) */
#define BNO055_REG_LIA_DATA_X_LSB  0x28      /* linear acc  X,Y,Z  (6 bytes) */
#define BNO055_REG_GRV_DATA_X_LSB  0x2E      /* gravity     X,Y,Z  (6 bytes) */
#define BNO055_REG_TEMP            0x34      /* temperature        (1 byte)  */
#define BNO055_REG_CALIB_STAT      0x35      /* sys/gyr/acc/mag    (1 byte)  */

/* The single burst: from ACC (0x08) through CALIB (0x35) inclusive. */
#define BNO055_BURST_START         BNO055_REG_ACC_DATA_X_LSB   /* 0x08 */
#define BNO055_BURST_LEN           (BNO055_REG_CALIB_STAT - BNO055_REG_ACC_DATA_X_LSB + 1) /* 0x35-0x08+1 = 46 */

/* Offsets of each field WITHIN the burst buffer (register - BURST_START). */
#define OFF_ACC   (BNO055_REG_ACC_DATA_X_LSB  - BNO055_BURST_START)  /* 0  */
#define OFF_MAG   (BNO055_REG_MAG_DATA_X_LSB  - BNO055_BURST_START)  /* 6  */
#define OFF_GYR   (BNO055_REG_GYR_DATA_X_LSB  - BNO055_BURST_START)  /* 12 */
#define OFF_EUL   (BNO055_REG_EUL_HEAD_LSB    - BNO055_BURST_START)  /* 18 */
#define OFF_QUA   (BNO055_REG_QUA_DATA_W_LSB  - BNO055_BURST_START)  /* 24 */
#define OFF_LIA   (BNO055_REG_LIA_DATA_X_LSB  - BNO055_BURST_START)  /* 32 */
#define OFF_GRV   (BNO055_REG_GRV_DATA_X_LSB  - BNO055_BURST_START)  /* 38 */
#define OFF_TEMP  (BNO055_REG_TEMP            - BNO055_BURST_START)  /* 44 */
#define OFF_CALIB (BNO055_REG_CALIB_STAT      - BNO055_BURST_START)  /* 45 */

#define BNO055_REG_UNIT_SEL      0x3B
#define BNO055_REG_OPR_MODE      0x3D
#define BNO055_REG_PWR_MODE      0x3E
#define BNO055_REG_SYS_TRIGGER   0x3F

#define BNO055_CHIP_ID_VALUE     0xA0

#define BNO055_OPR_MODE_CONFIG   0x00
#define BNO055_OPR_MODE_NDOF     0x0C
#define BNO055_PWR_MODE_NORMAL   0x00

/* Scale factors (datasheet 3.6.5, default units): LSB per engineering unit. */
#define BNO055_QUAT_LSB          16384.0f    /* unit quaternion = 2^14 LSB   */
#define BNO055_EUL_LSB_PER_DEG   16.0f       /* 1 deg   = 16 LSB             */
#define BNO055_ACC_LSB_PER_MS2   100.0f      /* 1 m/s^2 = 100 LSB (acc/lia/grv) */
#define BNO055_GYR_LSB_PER_DPS   16.0f       /* 1 dps   = 16 LSB             */
#define BNO055_MAG_LSB_PER_UT    16.0f       /* 1 uT    = 16 LSB             */

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

/* All BNO055 outputs for one sample, in engineering units. */
typedef struct {
    float qw, qx, qy, qz;     /* fused quaternion (normalized)        */
    float ax, ay, az;         /* raw accelerometer       (m/s^2)      */
    float gx, gy, gz;         /* raw gyroscope           (dps)        */
    float mx, my, mz;         /* raw magnetometer        (uT) @20Hz   */
    float lx, ly, lz;         /* linear acceleration     (m/s^2)      */
    float grx, gry, grz;      /* gravity vector          (m/s^2)      */
    int8_t temp;              /* temperature             (deg C)      */
    uint8_t cal_sys, cal_gyr, cal_acc, cal_mag;   /* 0..3             */
} bno_sample_t;

/* Little-endian int16 from a buffer offset. */
static inline int16_t le16(const uint8_t *b, int off)
{
    return (int16_t)((b[off + 1] << 8) | b[off]);
}

/* Read the ENTIRE data block (ACC 0x08 .. CALIB 0x35, 46 bytes) in one I2C
 * burst, then slice each field out by offset and scale to engineering units.
 * One transaction = one clock-stretch penalty, which is what keeps us inside
 * the 10 ms (100 Hz) budget even while reading every sensor. */
static esp_err_t bno_read_all(bno_sample_t *s)
{
    uint8_t raw[BNO055_BURST_LEN];
    esp_err_t err = bno_read(BNO055_BURST_START, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    /* Quaternion (W,X,Y,Z). */
    s->qw = le16(raw, OFF_QUA + 0) / BNO055_QUAT_LSB;
    s->qx = le16(raw, OFF_QUA + 2) / BNO055_QUAT_LSB;
    s->qy = le16(raw, OFF_QUA + 4) / BNO055_QUAT_LSB;
    s->qz = le16(raw, OFF_QUA + 6) / BNO055_QUAT_LSB;

    /* Raw accelerometer / gyroscope / magnetometer. */
    s->ax = le16(raw, OFF_ACC + 0) / BNO055_ACC_LSB_PER_MS2;
    s->ay = le16(raw, OFF_ACC + 2) / BNO055_ACC_LSB_PER_MS2;
    s->az = le16(raw, OFF_ACC + 4) / BNO055_ACC_LSB_PER_MS2;
    s->gx = le16(raw, OFF_GYR + 0) / BNO055_GYR_LSB_PER_DPS;
    s->gy = le16(raw, OFF_GYR + 2) / BNO055_GYR_LSB_PER_DPS;
    s->gz = le16(raw, OFF_GYR + 4) / BNO055_GYR_LSB_PER_DPS;
    s->mx = le16(raw, OFF_MAG + 0) / BNO055_MAG_LSB_PER_UT;
    s->my = le16(raw, OFF_MAG + 2) / BNO055_MAG_LSB_PER_UT;
    s->mz = le16(raw, OFF_MAG + 4) / BNO055_MAG_LSB_PER_UT;

    /* Fusion outputs: linear acceleration (gravity removed) and gravity. */
    s->lx = le16(raw, OFF_LIA + 0) / BNO055_ACC_LSB_PER_MS2;
    s->ly = le16(raw, OFF_LIA + 2) / BNO055_ACC_LSB_PER_MS2;
    s->lz = le16(raw, OFF_LIA + 4) / BNO055_ACC_LSB_PER_MS2;
    s->grx = le16(raw, OFF_GRV + 0) / BNO055_ACC_LSB_PER_MS2;
    s->gry = le16(raw, OFF_GRV + 2) / BNO055_ACC_LSB_PER_MS2;
    s->grz = le16(raw, OFF_GRV + 4) / BNO055_ACC_LSB_PER_MS2;

    /* Temperature (signed byte, deg C in default unit). */
    s->temp = (int8_t)raw[OFF_TEMP];

    /* Calibration status: four 2-bit fields, 0 (uncal) .. 3 (full). */
    uint8_t c = raw[OFF_CALIB];
    s->cal_sys = (c >> 6) & 0x03;
    s->cal_gyr = (c >> 4) & 0x03;
    s->cal_acc = (c >> 2) & 0x03;
    s->cal_mag = c & 0x03;
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

    /* Make stdout writes non-blocking. By default the USB Serial/JTAG VFS does
     * BLOCKING writes: it busy-waits until the host drains the endpoint, costing
     * ~9-12 ms/cycle (measured) and capping the loop near 71 Hz. We install the
     * interrupt-driven driver (which gives us a background TX ring buffer) and
     * write the data lines DIRECTLY with usb_serial_jtag_write_bytes(..., 0),
     * which queues bytes and returns immediately without touching the blocking
     * stdio path. (Routing stdout through the VFS driver proved unreliable on
     * this build, so we bypass stdio for the hot path entirely.) */
    usb_serial_jtag_driver_config_t usj_cfg = {
        .tx_buffer_size = 4096,
        .rx_buffer_size = 256,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usj_cfg));

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

    /* ---- Steady-state loop: no allocation, fixed stack buffers ---------
     *
     * Line format (version-prefixed positional CSV, fields comma-separated,
     * CR+LF terminated). The leading "v2" lets the visualizer reject a
     * mismatched firmware/visualizer pair instead of silently misparsing.
     * Field order is FIXED; readers slice by position:
     *
     *   v2,seq,t_us,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,lx,ly,lz,grx,gry,grz,temp,cs,cg,ca,cm
     *
     *   v2      format tag (literal)
     *   seq     uint32 sample counter, +1 per emitted line, wraps at 2^32
     *           (~497 days @100Hz). Readers use wrap-safe delta = (cur-prev).
     *   t_us    uint64 device timestamp in microseconds (esp_timer_get_time),
     *           taken right before the I2C read. Wraps after ~292,000 years.
     *           This is the authoritative clock for rate/jitter (immune to USB
     *           buffering and PC scheduling). seq catches dropped lines that a
     *           timestamp gap alone cannot distinguish from lateness.
     *   qw..qz  fused quaternion, %.4f
     *   ax..az  raw accelerometer, m/s^2, %.3f
     *   gx..gz  raw gyroscope, dps, %.3f
     *   mx..mz  raw magnetometer, uT, %.3f  (only fresh @20Hz; repeats between)
     *   lx..lz  linear acceleration (gravity removed), m/s^2, %.3f
     *   grx..grz gravity vector, m/s^2, %.3f
     *   temp    temperature, integer deg C
     *   cs,cg,ca,cm  calibration 0..3 for sys,gyr,acc,mag
     *
     * The firmware always emits every field; selecting/deselecting sensors is
     * done visualizer-side for now. We match the Arduino-style CR+LF; the
     * CONFIG_LIBC_STDOUT_LINE_ENDING_LF sdkconfig option stops the console from
     * adding a second CR. Buffer is sized for the worst-case line length.
     *
     * Timing: xTaskDelayUntil() schedules the next wake at a FIXED interval
     * from the previous wake, so the sensor-read and print time does NOT add on
     * top of the period (vTaskDelay would do that, dragging the rate down to
     * ~50-60 Hz). This requires a fine FreeRTOS tick: with the default 100 Hz
     * tick, a 10 ms period rounds to one tick and cannot resolve 100 Hz, so set
     * CONFIG_FREERTOS_HZ=1000 in menuconfig (then pdMS_TO_TICKS(10) = 10 ticks
     * and the loop holds a true 100 Hz). */
    bno_sample_t s;
    uint32_t seq = 0;
    char line[256];
    const TickType_t period = pdMS_TO_TICKS(10);   /* 100 Hz */
    TickType_t wake = xTaskGetTickCount();

#if DIAG
    /* One-time diagnostic on the data stream itself (so it shows even with IDF
     * logging disabled): report the COMPILED FreeRTOS tick rate and how many
     * ticks a 10 ms period actually resolves to. If tick_hz is not 1000 or
     * period_ticks is not 10, the CONFIG_FREERTOS_HZ=1000 change did not make it
     * into this binary, which is what caps the loop near 71 Hz. The visualizer
     * ignores any line it cannot parse, so this diag line is harmless there. */
    int dn = snprintf(line, sizeof(line), "# diag tick_hz=%d period_ticks=%u\r\n",
                      (int)configTICK_RATE_HZ, (unsigned)period);
    if (dn > 0) {
        usb_serial_jtag_write_bytes(line, (size_t)dn, pdMS_TO_TICKS(20));
    }
#endif

    while (1) {
        int64_t t_us = esp_timer_get_time();      /* device clock, pre-read   */
        esp_err_t err = bno_read_all(&s);
#if DIAG
        int64_t t_after_read = esp_timer_get_time();
#endif
        if (err == ESP_OK) {
            int n = snprintf(line, sizeof(line),
                "v2,%lu,%lld,"
                "%.4f,%.4f,%.4f,%.4f,"
                "%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,"
                "%d,%u,%u,%u,%u\r\n",
                (unsigned long)seq, (long long)t_us,
                s.qw, s.qx, s.qy, s.qz,
                s.ax, s.ay, s.az,
                s.gx, s.gy, s.gz,
                s.mx, s.my, s.mz,
                s.lx, s.ly, s.lz,
                s.grx, s.gry, s.grz,
                (int)s.temp,
                s.cal_sys, s.cal_gyr, s.cal_acc, s.cal_mag);
            if (n > 0) {
                /* Non-blocking: queue into the driver TX buffer and return at
                 * once (ticks_to_wait = 0). Replaces the blocking fwrite+fflush
                 * that was costing ~9-12 ms/cycle. */
                usb_serial_jtag_write_bytes(line, (size_t)n, 0);
            }
            seq++;   /* wrap-safe: readers compute (cur - prev) in uint32 */
        }

#if DIAG
        int64_t t_after_print = esp_timer_get_time();

        /* DIAGNOSTIC (compile-time gated): every 100th sample, emit timing in
         * microseconds for the I2C read and for the snprintf+write, plus the
         * total since loop top. Routed through the driver (non-blocking) so the
         * diagnostic itself does not stall the loop. The visualizer ignores
         * this non-data line. */
        if ((seq % 100) == 0) {
            char dbg[80];
            int dl = snprintf(dbg, sizeof(dbg),
                   "# t read_us=%lld print_us=%lld busy_us=%lld\r\n",
                   (long long)(t_after_read - t_us),
                   (long long)(t_after_print - t_after_read),
                   (long long)(t_after_print - t_us));
            if (dl > 0) {
                usb_serial_jtag_write_bytes(dbg, (size_t)dl, 0);
            }
        }
#endif

        /* Sleep until the next 10 ms boundary regardless of read/print time. */
        xTaskDelayUntil(&wake, period);
    }
}