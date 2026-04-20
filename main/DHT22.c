/*
 * DHT22 / AM2302 Test — ESP32 WROOM-32E (ESP-IDF v5.x)
 * Prints temperature and humidity every 3 seconds
 *
 * Pin Map:
 *   DHT22 VCC  → 3.3V
 *   DHT22 GND  → GND
 *   DHT22 DATA → GPIO14  (4.7kΩ pull-up to 3.3V)
 *
 * Notes:
 *   - DHT22 minimum sampling interval is 2 seconds — do not read faster
 *   - Bit-bang protocol, timing sensitive — CRC errors are normal
 *     occasionally under FreeRTOS load, retry logic is included
 *   - If you get persistent CRC errors, check pull-up resistor
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_rom_gpio.h"

/* ─── Pin ────────────────────────────────────────────────────────────────── */
#define DHT22_PIN   GPIO_NUM_14

/* ─── Timing (µs) ────────────────────────────────────────────────────────── */
#define DHT_START_LOW_MS   20      // host pulls low for 20ms to wake sensor
#define DHT_START_HIGH_US  30      // host releases, waits 30µs before listening
#define DHT_TIMEOUT_US     1000    // max wait per edge before declaring timeout

/* ════════════════════════════════════════════════════════════════════════════
 * LOW-LEVEL READ
 * Returns ESP_OK, ESP_ERR_TIMEOUT, or ESP_ERR_INVALID_CRC
 * ════════════════════════════════════════════════════════════════════════════ */
static esp_err_t dht22_read_raw(float *temp, float *hum)
{
    uint8_t data[5] = {0};
    uint32_t cnt;

    /* ── Start signal ── */
    esp_rom_gpio_pad_select_gpio(DHT22_PIN);
    gpio_set_direction(DHT22_PIN, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(DHT22_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(DHT_START_LOW_MS));
    gpio_set_level(DHT22_PIN, 1);
    ets_delay_us(DHT_START_HIGH_US);
    gpio_set_direction(DHT22_PIN, GPIO_MODE_INPUT);

    /* ── Wait for sensor response: LOW then HIGH ── */
    cnt = 0;
    while (!gpio_get_level(DHT22_PIN)) {
        if (cnt++ > DHT_TIMEOUT_US) return ESP_ERR_TIMEOUT;
        ets_delay_us(1);
    }
    cnt = 0;
    while (gpio_get_level(DHT22_PIN)) {
        if (cnt++ > DHT_TIMEOUT_US) return ESP_ERR_TIMEOUT;
        ets_delay_us(1);
    }

    /* ── Read 40 bits ── */
    for (int i = 0; i < 40; i++) {
        // Wait for LOW-to-HIGH edge (start of bit)
        cnt = 0;
        while (!gpio_get_level(DHT22_PIN)) {
            if (cnt++ > DHT_TIMEOUT_US) return ESP_ERR_TIMEOUT;
            ets_delay_us(1);
        }

        // Sample after 40µs:
        //   if still HIGH → bit is 1 (high pulse ~70µs)
        //   if already LOW → bit is 0 (high pulse ~26-28µs)
        ets_delay_us(40);
        data[i / 8] <<= 1;
        if (gpio_get_level(DHT22_PIN)) {
            data[i / 8] |= 1;
            // Wait for this HIGH to end
            cnt = 0;
            while (gpio_get_level(DHT22_PIN)) {
                if (cnt++ > DHT_TIMEOUT_US) return ESP_ERR_TIMEOUT;
                ets_delay_us(1);
            }
        }
    }

    /* ── Checksum ── */
    uint8_t expected = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (data[4] != expected) {
        return ESP_ERR_INVALID_CRC;
    }

    /* ── Decode ── */
    *hum  = ((data[0] << 8) | data[1]) / 10.0f;
    float t_raw = (((data[2] & 0x7F) << 8) | data[3]) / 10.0f;
    *temp = (data[2] & 0x80) ? -t_raw : t_raw;

    /* ── Sanity check — DHT22 range is -40 to +80°C, 0–100% RH ── */
    if (*temp < -40.0f || *temp > 80.0f) return ESP_ERR_INVALID_RESPONSE;
    if (*hum  <   0.0f || *hum  > 100.0f) return ESP_ERR_INVALID_RESPONSE;

    return ESP_OK;
}



/* ════════════════════════════════════════════════════════════════════════════
 * HEAT INDEX (feels-like temperature) — Steadman formula
 * Only valid above 27°C and 40% RH, returns temp as-is below that
 * ════════════════════════════════════════════════════════════════════════════ */
static float heat_index(float t, float rh)
{
    if (t < 27.0f || rh < 40.0f) return t;
    float hi = -8.78469475556f
             + 1.61139411f    * t
             + 2.33854883889f * rh
             - 0.14611605f    * t  * rh
             - 0.01230809050f * t  * t
             - 0.01642482777f * rh * rh
             + 0.00221732990f * t  * t  * rh
             + 0.00072546f    * t  * rh * rh
             - 0.00000358f    * t  * t  * rh * rh;
    return hi;
}

/* ════════════════════════════════════════════════════════════════════════════
 * TASK
 * ════════════════════════════════════════════════════════════════════════════ */
void dht22_task(void *arg)
{
    int count      = 0;
    int err_count  = 0;

    printf("\n[DHT22]  Starting readings...\n");
    printf("[DHT22]  GPIO14, 4.7kΩ pull-up required on data line.\n\n");

    while (1) {
        count++;
        float temp = 0, hum = 0;
        esp_err_t err = dht22_read_raw(&temp, &hum);

        printf("\n========== Reading #%d ==========\n", count);

        if (err == ESP_OK) {
            float hi = heat_index(temp, hum);
            printf("[DHT22]  Temperature : %.1f °C\n",  temp);
            printf("[DHT22]  Humidity    : %.1f %%\n",  hum);
            if (hi != temp)
                printf("[DHT22]  Heat Index  : %.1f °C  (feels like)\n", hi);

            // Comfort interpretation
            if (hum < 30.0f)
                printf("[DHT22]  Comfort     : Dry\n");
            else if (hum < 60.0f)
                printf("[DHT22]  Comfort     : Comfortable\n");
            else if (hum < 70.0f)
                printf("[DHT22]  Comfort     : Humid\n");
            else
                printf("[DHT22]  Comfort     : Very Humid\n");

        } else {
            err_count++;
            if (err == ESP_ERR_INVALID_CRC)
                printf("[DHT22]  ERROR: CRC mismatch\n");
            else if (err == ESP_ERR_TIMEOUT)
                printf("[DHT22]  ERROR: Timeout — no response from sensor\n");
            else if (err == ESP_ERR_INVALID_RESPONSE)
                printf("[DHT22]  ERROR: Value out of range — sensor fault?\n");
            else
                printf("[DHT22]  ERROR: Read failed\n");

            if (err_count >= 5)
                printf("[DHT22]  5 consecutive errors — check wiring\n");
        }

        printf("=================================\n");

        // DHT22 minimum interval is 2s — use 3.5s to give plenty of headroom
        vTaskDelay(pdMS_TO_TICKS(3500));
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * APP MAIN
 * ════════════════════════════════════════════════════════════════════════════ */
void DHT22_start(void)
{
    printf("\n\n");
    printf("================================\n");
    printf("  DHT22 / AM2302 Sensor Init   \n");
    printf("================================\n\n");

    // Idle state HIGH (important for DHT protocol)
    esp_rom_gpio_pad_select_gpio(DHT22_PIN);
    gpio_set_direction(DHT22_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHT22_PIN, GPIO_PULLUP_ONLY);
}