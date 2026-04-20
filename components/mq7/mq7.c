/*
 * MQ7 Raw Test — ESP32 WROOM-32E (ESP-IDF v5.x)
 * Prints raw ADC, voltage, Rs every 2 seconds
 * NO heater switching — raw values only for now
 *
 * ─── WARNING ─────────────────────────────────────────────────────────────────
 * Without the 5V → 1.4V heater cycle, the MQ7 runs continuously at 5V.
 * Rs/R0 values printed here are NOT accurate for CO measurement.
 * This code is only useful to confirm the sensor is wired and reading.
 * Implement PWM heater switching before trusting any CO readings.
 * ─────────────────────────────────────────────────────────────────────────────
 *
 * Pin Map:
 *   MQ7 VCC  → 5V
 *   MQ7 GND  → GND
 *   MQ7 AOUT → Voltage divider → GPIO35
 *   MQ7 DOUT → Not connected
 *
 * Voltage divider (AOUT 0–5V → GPIO35 max 3.3V):
 *   AOUT ── 10kΩ ──┬── GPIO35
 *                  │
 *                 10kΩ
 *                  │
 *                 10kΩ
 *                  │
 *                 GND
 *
 * GPIO35 is input-only — safe ADC pin.
 * RL on board = 10kΩ (standard MQ7 breakout)
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "adc_shared.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* ─── ADC config ─────────────────────────────────────────────────────────── */
#define MQ7_ADC_CHANNEL   ADC_CHANNEL_7    // GPIO35
#define MQ7_ADC_ATTEN     ADC_ATTEN_DB_12  // 0–3.9V range (DB_11 deprecated)
#define MQ7_ADC_BITWIDTH  ADC_BITWIDTH_12  // 0–4095

/* ─── Sensor constants ───────────────────────────────────────────────────── */
#define MQ7_RL_KOHM       10.0f   // load resistor on breakout board
#define MQ7_DIVIDER       1.5f    // 10kΩ + 20kΩ: Vaout = Vgpio * 1.5
#define MQ7_R0_KOHM       27.5f   // placeholder — calibrate in clean air

/* ─── ADC handles ────────────────────────────────────────────────────────── */
static adc_cali_handle_t         mq7_cali_handle = NULL;
static bool                      mq7_cali_ok     = false;

/* ════════════════════════════════════════════════════════════════════════════
 * ADC INIT
 * ════════════════════════════════════════════════════════════════════════════ */
void mq7_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = MQ7_ADC_ATTEN,
        .bitwidth = MQ7_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MQ7_ADC_CHANNEL, &chan_cfg));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = MQ7_ADC_CHANNEL,
        .atten    = MQ7_ADC_ATTEN,
        .bitwidth = MQ7_ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &mq7_cali_handle) == ESP_OK) {
        mq7_cali_ok = true;
        printf("[MQ7]   ADC cal: Curve Fitting\n");
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!mq7_cali_ok) {
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id  = ADC_UNIT_1,
            .atten    = MQ7_ADC_ATTEN,
            .bitwidth = MQ7_ADC_BITWIDTH,
        };
        if (adc_cali_create_scheme_line_fitting(&cali_cfg, &mq7_cali_handle) == ESP_OK) {
            mq7_cali_ok = true;
            printf("[MQ7]   ADC cal: Line Fitting\n");
        }
    }
#endif

    if (!mq7_cali_ok)
        printf("[MQ7]   ADC cal: None — raw estimate only\n");
}

/* ════════════════════════════════════════════════════════════════════════════
 * READ — 64 sample average
 * ════════════════════════════════════════════════════════════════════════════ */
float mq7_read_voltage(void)
{
    int sum = 0;
    for (int i = 0; i < 64; i++) {
        int raw = 0;
        adc_oneshot_read(adc_handle, MQ7_ADC_CHANNEL, &raw);
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    int raw_avg = sum / 64;

    int mv = 0;
    if (mq7_cali_ok)
        adc_cali_raw_to_voltage(mq7_cali_handle, raw_avg, &mv);
    else
        mv = (int)(raw_avg * 3300.0f / 4095.0f);

    return (mv / 1000.0f) * MQ7_DIVIDER;
}

/* ════════════════════════════════════════════════════════════════════════════
 * TASK
 * ════════════════════════════════════════════════════════════════════════════ */
void mq7_task(void *arg)
{
    int count = 0;

    printf("\n[MQ7]   Starting raw readings...\n");
    printf("[MQ7]   Sensor needs 5 min warmup for stable Rs.\n");
    printf("[MQ7]   Rs/R0 NOT valid without heater switching.\n\n");

    while (1) {
        count++;

        float vaout = mq7_read_voltage();
        float vgpio = vaout / MQ7_DIVIDER;
        int   raw   = (int)(vgpio / 3.3f * 4095.0f);

        float rs = (vaout < 0.01f)
                   ? 9999.0f
                   : (MQ7_RL_KOHM * (5.0f - vaout) / vaout);

        float rs_r0 = rs / MQ7_R0_KOHM;

        printf("\n========== Reading #%d ==========\n", count);
        printf("[MQ7]   Raw ADC  : %d / 4095\n",  raw);
        printf("[MQ7]   GPIO V   : %.3f V\n",      vgpio);
        printf("[MQ7]   AOUT V   : %.3f V\n",      vaout);
        printf("[MQ7]   Rs       : %.2f kΩ\n",     rs);
        printf("[MQ7]   Rs/R0    : %.4f\n",          rs_r0);

        if (count <= 150)
            printf("[MQ7]   Warmup   : %ds / 300s\n", count * 2);

        printf("[MQ7]   !! No heater switching — values unreliable !!\n");
        printf("=================================\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * START — call from app_main
 * ════════════════════════════════════════════════════════════════════════════ */
void mq7_start(void)
{
    printf("[MQ7]   GPIO35 = ADC1_CH7\n");
    printf("[MQ7]   RL = %.0f kΩ\n",    MQ7_RL_KOHM);
    printf("[MQ7]   R0 = %.1f kΩ (uncalibrated)\n\n", MQ7_R0_KOHM);

    mq7_adc_init();
    printf("[MQ7]   Init complete\n\n");
}