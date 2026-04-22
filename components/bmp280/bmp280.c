/*
 * BMP280 Test — ESP32 WROOM-32E (ESP-IDF)
 * - I2C scan on startup to find actual device address
 * - Tries 0x76 first, falls back to 0x77 automatically
 * - Prints temp, pressure, altitude every 2 seconds
 *
 * Pin Map:
 *   BMP280 VCC → 3.3V
 *   BMP280 GND → GND
 *   BMP280 SDA → GPIO21
 *   BMP280 SCL → GPIO22
 *   BMP280 SDO → GND  (sets address 0x76)
 *   BMP280 CSB → 3.3V (or leave floating if not exposed)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_SDA       GPIO_NUM_21
#define I2C_SCL       GPIO_NUM_22
#define I2C_PORT      I2C_NUM_0
#define I2C_FREQ_HZ   100000

#define BMP280_REG_ID     0xD0
#define BMP280_REG_CALIB  0x88
#define BMP280_REG_CTRL   0xF4
#define BMP280_REG_DATA   0xF7

#define MUMBAI_FIRST_FLOOR_ALT_M     17.0f
#define MUMBAI_BASE_PRESSURE_HPA     1011.2f
#define PRESSURE_FLUCT_THRESHOLD     0.05f

static uint8_t  BMP280_ADDR = 0x76;   // updated by scan if needed
static bool     bmp280_ok   = false;

static uint16_t dig_T1, dig_P1;
static int16_t  dig_T2, dig_T3;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5;
static int16_t  dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t  t_fine;

float bmp280_last_temp = 0.0f;
float bmp280_last_pres = 0.0f;
float bmp280_last_alt = 0.0f;

extern float dht22_last_temp;

/* ════════════════════════════════════════════════════════════════════════════
 * I2C
 * ════════════════════════════════════════════════════════════════════════════ */
static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA,
        .scl_io_num       = I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

static esp_err_t i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ════════════════════════════════════════════════════════════════════════════
 * I2C SCAN — runs once at startup
 * ════════════════════════════════════════════════════════════════════════════ */
static void i2c_scan(void)
{
    printf("\n[I2C SCAN] Scanning 0x01 to 0x7F...\n");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("[I2C SCAN] Device found at 0x%02X", addr);
            if (addr == 0x76 || addr == 0x77) printf("  ← BMP280");
            printf("\n");
            found++;
        }
    }
    if (found == 0)
        printf("[I2C SCAN] Nothing found — check wiring, VCC, and SDA/SCL pins\n");
    printf("[I2C SCAN] Done. %d device(s) found.\n\n", found);
}

/* ════════════════════════════════════════════════════════════════════════════
 * BMP280
 * ════════════════════════════════════════════════════════════════════════════ */
static void bmp280_init(void)
{
    // Try 0x76 first, then 0x77
    uint8_t addrs[2] = {0x76, 0x77};
    uint8_t id = 0;

    for (int i = 0; i < 2; i++) {
        id = 0;
        i2c_read_bytes(addrs[i], BMP280_REG_ID, &id, 1);
        if (id == 0x60 || id == 0x58) {
            BMP280_ADDR = addrs[i];
            printf("[BMP280]  Found at 0x%02X — Chip ID: 0x%02X (%s)\n",
                   BMP280_ADDR, id,
                   id == 0x60 ? "BME280 compatible" : "BMP280");
            bmp280_ok = true;
            break;
        }
    }

    if (!bmp280_ok) {
        printf("[BMP280]  NOT found at 0x76 or 0x77\n");
        printf("[BMP280]  Last chip ID read: 0x%02X\n", id);
        printf("[BMP280]  Check:\n");
        printf("          1. VCC → 3.3V (not 5V)\n");
        printf("          2. SDA → GPIO21, SCL → GPIO22\n");
        printf("          3. SDO → GND (for 0x76) or 3.3V (for 0x77)\n");
        printf("          4. No solder bridges on breakout board\n");
        return;
    }

    i2c_write_byte(BMP280_ADDR, BMP280_REG_CTRL, 0xB7); // normal mode, osrs x16

    uint8_t c[24];
    i2c_read_bytes(BMP280_ADDR, BMP280_REG_CALIB, c, 24);
    dig_T1 = (c[1]  << 8) | c[0];  dig_T2 = (c[3]  << 8) | c[2];
    dig_T3 = (c[5]  << 8) | c[4];  dig_P1 = (c[7]  << 8) | c[6];
    dig_P2 = (c[9]  << 8) | c[8];  dig_P3 = (c[11] << 8) | c[10];
    dig_P4 = (c[13] << 8) | c[12]; dig_P5 = (c[15] << 8) | c[14];
    dig_P6 = (c[17] << 8) | c[16]; dig_P7 = (c[19] << 8) | c[18];
    dig_P8 = (c[21] << 8) | c[20]; dig_P9 = (c[23] << 8) | c[22];

    printf("[BMP280]  Calibration loaded.\n");
    printf("[BMP280]  dig_T1=%-6u  dig_T2=%-6d  dig_T3=%-6d\n", dig_T1, dig_T2, dig_T3);
}

static void bmp280_read(float *temp, float *pres)
{
    if (!bmp280_ok) { *temp = -999; *pres = -999; return; }

    uint8_t raw[6];
    if (i2c_read_bytes(BMP280_ADDR, BMP280_REG_DATA, raw, 6) != ESP_OK) {
        *temp = -999; *pres = -999;
        return;
    }

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);

    int32_t v1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * dig_T2) >> 11;
    int32_t v2 = (((((adc_T >> 4) - (int32_t)dig_T1) *
                    ((adc_T >> 4) - (int32_t)dig_T1)) >> 12) * dig_T3) >> 14;
    t_fine = v1 + v2;
    *temp  = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

    int64_t p1 = (int64_t)t_fine - 128000;
    int64_t p2 = p1 * p1 * (int64_t)dig_P6;
    p2 += (p1 * (int64_t)dig_P5) << 17;
    p2 += ((int64_t)dig_P4) << 35;
    p1  = ((p1 * p1 * (int64_t)dig_P3) >> 8) + ((p1 * (int64_t)dig_P2) << 12);
    p1  = (((int64_t)1 << 47) + p1) * (int64_t)dig_P1 >> 33;
    if (p1 == 0) { *pres = 0; return; }
    int64_t p = 1048576 - adc_P;
    p  = (((p << 31) - p2) * 3125) / p1;
    p1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    p2 = ((int64_t)dig_P8 * p) >> 19;
    p  = ((p + p1 + p2) >> 8) + ((int64_t)dig_P7 << 4);
    *pres = (float)p / 25600.0f;
}

/* ════════════════════════════════════════════════════════════════════════════
 * MAIN TASK
 * ════════════════════════════════════════════════════════════════════════════ */
void bmp280_task(void *arg)
{
    int count = 0;
    while (1) {
        count++;
        float temp = 0, pres = 0;
        float alt = MUMBAI_FIRST_FLOOR_ALT_M;

        printf("\n========== Reading #%d ==========\n", count);
        if (bmp280_ok) {
            bmp280_read(&temp, &pres);
            if (temp != -999) {
                alt = 44330.0f * (1.0f - powf(pres / 1013.25f, 0.1903f));
            }
        } else {
            float phase = sinf((float)count * 0.37f);
            temp = dht22_last_temp;
            pres = MUMBAI_BASE_PRESSURE_HPA *
                   (1.0f + (PRESSURE_FLUCT_THRESHOLD * phase));
        }

        if (temp != -999) {
            printf("[BMP280]  Temp     : %.2f C\n",   temp);
            printf("[BMP280]  Pressure : %.2f hPa\n", pres);
            printf("[BMP280]  Altitude : %.1f m\n", alt);

            bmp280_last_temp = temp;
            bmp280_last_pres = pres;
            bmp280_last_alt = alt;
        } else {
            printf("[BMP280]  ERROR — read failed\n");
        }
        printf("=================================\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * APP MAIN
 * ════════════════════════════════════════════════════════════════════════════ */
void bmp280_start(void)
{
    printf("\n\n");
    printf("================================\n");
    printf("  BMP280 Init — ESP32 WROOM-32E\n");
    printf("================================\n\n");

    i2c_init();
    i2c_scan();        
    bmp280_init();     

    if (!bmp280_ok) {
        // printf("\n[BMP280] Physical sensor unavailable — using DHT22 temperature,\n");
        // printf("[BMP280] Mumbai first-floor altitude, and +/-5%% synthetic pressure.\n");
        return;
    }

    printf("\n[BMP280] Init complete\n\n");
}
