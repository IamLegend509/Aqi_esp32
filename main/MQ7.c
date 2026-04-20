/*
 * MQ7 Raw Test — ESP32 WROOM-32E (ESP-IDF)
 * Prints raw ADC, voltage, Rs every 2 seconds
 * NO heater switching — raw values only for now
 *
 * ─── WARNING ─────────────────────────────────────────────────────────────────
 * Without the 5V → 1.4V heater cycle, the MQ7 runs continuously at 5V.
 * Rs/R0 and CO ppm values printed here are NOT accurate.
 * This code is only useful to confirm the sensor is wired and reading.
 * Implement PWM heater switching before trusting any CO readings.
 * ─────────────────────────────────────────────────────────────────────────────
 *
 * Pin Map:
 *   MQ7 VCC  → 5V
 *   MQ7 GND  → GND
 *   MQ7 AOUT → Voltage divider → GPIO35
 *
 * Voltage divider (AOUT is 0–5V, GPIO35 max is 3.3V):
 *   AOUT ──┬── 10kΩ ──┬── GPIO35
 *          │           │
 *         GND        20kΩ
 *                      │
 *                     GND
 *
 * GPIO35 is input-only — safe ADC pin, no accidental output drive.
 * RL on board = 10kΩ (standard MQ7 breakout)
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

/* ─── ADC config ─────────────────────────────────────────────────────────── */
#define MQ7_ADC_CHANNEL   ADC1_CHANNEL_7   // GPIO35
#define MQ7_ADC_ATTEN     ADC_ATTEN_DB_11  // 0–3.9V range
#define MQ7_ADC_WIDTH     ADC_WIDTH_BIT_12 // 0–4095

/* ─── Sensor constants ───────────────────────────────────────────────────── */
#define RL_KOHM           10.0f   // load resistor on your breakout board
#define VDIV_RATIO        3.0f    // divider scales 5V → ~1.67V per volt
                                  // with 10k+20k: Vout = Vin * 20/(10+20) = Vin * 0.667
                                  // so actual AOUT voltage = GPIO voltage * 1.5
                                  // keep this as 1.5 if using 10k/20k divider
#define DIVIDER_FACTOR    1.5f    // multiply measured voltage to get actual AOUT voltage

// R0 placeholder — replace after proper calibration in clean air
// MQ7 typical R0 in clean air ≈ 27.5 kΩ with 10kΩ RL
// This is a datasheet estimate — calibrate yours before using ppm
#define R0_KOHM           27.5f

static esp_adc_cal_characteristics_t adc_chars;

/* ════════════════════════════════════════════════════════════════════════════
 * ADC INIT
 * ════════════════════════════════════════════════════════════════════════════ */
static void mq7_adc_init(void)
{
    adc1_config_width(MQ7_ADC_WIDTH);
    adc1_config_channel_atten(MQ7_ADC_CHANNEL, MQ7_ADC_ATTEN);

    esp_adc_cal_value_t cal_type = esp_adc_cal_characterize(
        ADC_UNIT_1, MQ7_ADC_ATTEN, MQ7_ADC_WIDTH, 1100, &adc_chars);

    if (cal_type == ESP_ADC_CAL_VAL_EFUSE_TP)
        printf("[MQ7]  ADC cal: Two Point from eFuse\n");
    else if (cal_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
        printf("[MQ7]  ADC cal: Vref from eFuse\n");
    else
        printf("[MQ7]  ADC cal: Default Vref (1100mV) — less accurate\n");
}

/* ════════════════════════════════════════════════════════════════════════════
 * READ
 * ════════════════════════════════════════════════════════════════════════════ */
static float mq7_read_voltage(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < 64; i++) {   // 64 samples — more averaging than MQ135
        sum += adc1_get_raw(MQ7_ADC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(1)); // 1ms between samples, no busy wait
    }
    uint32_t raw_avg = sum / 64;
    uint32_t mv = esp_adc_cal_raw_to_voltage(raw_avg, &adc_chars);

    // mv is voltage at GPIO pin after divider
    // multiply by DIVIDER_FACTOR to get actual AOUT voltage from sensor
    return (mv / 1000.0f) * DIVIDER_FACTOR;
}

/* ════════════════════════════════════════════════════════════════════════════
 * TASK
 * ════════════════════════════════════════════════════════════════════════════ */
void mq7_task(void *arg)
{
    int count = 0;

    printf("\n[MQ7]  Starting raw readings...\n");
    printf("[MQ7]  Sensor needs 5 min warmup for stable Rs.\n");
    printf("[MQ7]  Rs/R0 and ppm are NOT valid without heater switching.\n\n");

    while (1) {
        count++;

        float v_aout = mq7_read_voltage();   // actual voltage at sensor AOUT pin

        // Rs = RL * (Vc - Vout) / Vout
        // Vc = 5V (sensor supply), Vout = AOUT voltage
        float rs = (v_aout < 0.01f)
                   ? 9999.0f
                   : (RL_KOHM * (5.0f - v_aout) / v_aout);

        float rs_r0 = rs / R0_KOHM;

        // Back-calculate raw ADC for display
        int raw = (int)((v_aout / DIVIDER_FACTOR / 3.3f) * 4095.0f);

        printf("\n========== Reading #%d ==========\n", count);
        printf("[MQ7]  Raw ADC   : %d / 4095\n",  raw);
        printf("[MQ7]  GPIO V    : %.3f V  (after divider)\n", v_aout / DIVIDER_FACTOR);
        printf("[MQ7]  AOUT V    : %.3f V  (actual sensor output)\n", v_aout);
        printf("[MQ7]  Rs        : %.2f kOhm\n",   rs);
        printf("[MQ7]  Rs/R0     : %.4f\n",          rs_r0);

        if (count <= 150)
            printf("[MQ7]  Warmup    : %ds / 300s min\n", count * 2);

        printf("[MQ7]  !! No heater switching — values unreliable !!\n");
        printf("=================================\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * APP MAIN
 * ════════════════════════════════════════════════════════════════════════════ */
void mq7_start(void *arg){
    printf("\n\n");
    printf("================================\n");
    printf("  MQ7 Raw Test — ESP32 WROOM-32E\n");
    printf("================================\n\n");

    printf("[MQ7]  GPIO35 = ADC1_CH7\n");
    printf("[MQ7]  RL = %.0f kOhm\n", RL_KOHM);
    printf("[MQ7]  R0 = %.1f kOhm (uncalibrated estimate)\n\n", R0_KOHM);

    mq7_adc_init();
    printf("\n[MQ7] Init complete\n\n");
}