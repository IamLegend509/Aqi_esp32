/*
 * MQ135 Air Quality Sensor Test — ESP32 WROOM-32E (ESP-IDF v5.x)
 * Prints raw ADC, voltage, Rs, Rs/R0 every 2 seconds
 *
 * Pin Map:
 *   MQ135 VCC  → 5V
 *   MQ135 GND  → GND
 *   MQ135 AOUT → voltage divider → GPIO34 (ADC1_CH6, input only)
 *   MQ135 DOUT → not connected
 *
 * Voltage divider (AOUT 0–5V → GPIO 0–3.3V):
 *   AOUT ── 10kΩ ──┬── GPIO34
 *                  │
 *                 20kΩ (or 2x10kΩ series)
 *                  │
 *                 GND
 *
 * Notes:
 *   - Sensor needs minimum 5 minutes warmup for stable readings
 *   - R0 must be calibrated in clean outdoor air after warmup
 *   - RL value depends on your breakout board — measure with multimeter
 *     Most cheap breakouts: 1kΩ. Some: 10kΩ.
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* ─── ADC ────────────────────────────────────────────────────────────────── */
#define MQ135_ADC_CHANNEL   ADC_CHANNEL_6    // GPIO34
#define MQ135_ADC_ATTEN     ADC_ATTEN_DB_12  // 0–3.9V range
#define MQ135_ADC_BITWIDTH  ADC_BITWIDTH_12  // 0–4095

/* ─── Sensor constants ───────────────────────────────────────────────────── */
// Check your board — measure RL between AOUT and GND with sensor unpowered
// Most cheap MQ135 breakouts use 1kΩ, some use 10kΩ
#define RL_KOHM      1.0f

// R0 calibrated in clean air — Mumbai baseline
#define R0_KOHM      1.27f

// PPM curve constants: ppm = a * pow(rs_r0, b)
// Derived from MQ135 datasheet sensitivity curves
typedef struct { float a; float b; } gas_curve_t;

//                                    a         b
static const gas_curve_t CO2     = { 116.602f, -2.769f };  // 350–10000 ppm
static const gas_curve_t CO      = { 605.180f, -3.937f };  // 10–1000 ppm
static const gas_curve_t NH3     = { 102.826f, -2.473f };  // 10–300 ppm
static const gas_curve_t ALCOHOL = {  77.255f, -3.180f };  // 10–300 ppm
static const gas_curve_t BENZENE = {  34.668f, -3.369f };  // 10–1000 ppm
static const gas_curve_t SMOKE   = {  40.000f, -3.150f };  // indicative only

// Divider factor: AOUT is 0–5V, GPIO sees 0–3.3V
// 10kΩ + 20kΩ: Vgpio = Vaout * 20/30 → Vaout = Vgpio * 1.5
#define DIVIDER_FACTOR  1.5f

/* ─── ADC handles ────────────────────────────────────────────────────────── */
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t         cali_handle = NULL;
static bool                      cali_ok     = false;

/* ════════════════════════════════════════════════════════════════════════════
 * ADC INIT
 * ════════════════════════════════════════════════════════════════════════════ */
static void mq135_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = MQ135_ADC_ATTEN,
        .bitwidth = MQ135_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MQ135_ADC_CHANNEL, &chan_cfg));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = MQ135_ADC_CHANNEL,
        .atten    = MQ135_ADC_ATTEN,
        .bitwidth = MQ135_ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle) == ESP_OK) {
        cali_ok = true;
        printf("[MQ135]  ADC cal: Curve Fitting\n");
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!cali_ok) {
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id  = ADC_UNIT_1,
            .atten    = MQ135_ADC_ATTEN,
            .bitwidth = MQ135_ADC_BITWIDTH,
        };
        if (adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle) == ESP_OK) {
            cali_ok = true;
            printf("[MQ135]  ADC cal: Line Fitting\n");
        }
    }
#endif

    if (!cali_ok)
        printf("[MQ135]  ADC cal: None — raw estimate only\n");
}

/* ════════════════════════════════════════════════════════════════════════════
 * READ — 32 sample average
 * ════════════════════════════════════════════════════════════════════════════ */
static float mq135_read_voltage(void)
{
    int sum = 0;
    for (int i = 0; i < 32; i++) {
        int raw = 0;
        adc_oneshot_read(adc_handle, MQ135_ADC_CHANNEL, &raw);
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    int raw_avg = sum / 32;

    int mv = 0;
    if (cali_ok)
        adc_cali_raw_to_voltage(cali_handle, raw_avg, &mv);
    else
        mv = (int)(raw_avg * 3300.0f / 4095.0f);

    // Correct for voltage divider to get actual AOUT voltage
    return (mv / 1000.0f) * DIVIDER_FACTOR;
}

/* ════════════════════════════════════════════════════════════════════════════
 * PPM CALCULATION
 * ════════════════════════════════════════════════════════════════════════════ */
static float get_ppm(gas_curve_t curve, float rs_r0)
{
    if (rs_r0 <= 0.0f) return -1.0f;
    return curve.a * powf(rs_r0, curve.b);
}

/* ════════════════════════════════════════════════════════════════════════════
 * TASK
 * ════════════════════════════════════════════════════════════════════════════ */
void mq135_task(void *arg)
{
    int count = 0;

    printf("\n[MQ135]  Starting readings...\n");
    printf("[MQ135]  RL  = %.0f kΩ — change RL_KOHM if your board differs\n", RL_KOHM);
    printf("[MQ135]  R0  = %.2f kΩ — uncalibrated placeholder\n", R0_KOHM);
    printf("[MQ135]  Warmup needed: 5 minutes minimum\n\n");

    while (1) {
        count++;

        float vaout = mq135_read_voltage();
        float vgpio = vaout / DIVIDER_FACTOR;

        // Rs = RL * (Vc - Vout) / Vout
        // Vc = 5V (sensor supply voltage)
        float rs = (vaout < 0.001f)
                   ? 9999.0f
                   : (RL_KOHM * (5.0f - vaout) / vaout);

        float rs_r0 = rs / R0_KOHM;
        int   raw   = (int)(vgpio / 3.3f * 4095.0f);

        printf("\n========== Reading #%d ==========\n", count);
        printf("[MQ135]  Raw ADC  : %d / 4095\n",  raw);
        printf("[MQ135]  GPIO V   : %.3f V\n",      vgpio);
        printf("[MQ135]  AOUT V   : %.3f V\n",      vaout);
        printf("[MQ135]  Rs       : %.2f kΩ\n",     rs);
        printf("[MQ135]  Rs/R0    : %.4f\n",          rs_r0);
        printf("─────────────────────────────────\n");
        printf("[MQ135]  CO2      : %.1f ppm\n",    get_ppm(CO2,     rs_r0));
        printf("[MQ135]  CO       : %.1f ppm\n",    get_ppm(CO,      rs_r0));
        printf("[MQ135]  NH3      : %.1f ppm\n",    get_ppm(NH3,     rs_r0));
        printf("[MQ135]  Alcohol  : %.1f ppm\n",    get_ppm(ALCOHOL, rs_r0));
        printf("[MQ135]  Benzene  : %.1f ppm\n",    get_ppm(BENZENE, rs_r0));
        printf("[MQ135]  Smoke    : %.1f ppm\n",    get_ppm(SMOKE,   rs_r0));

        if (count <= 150)
            printf("[MQ135]  Warmup   : %ds / 300s\n", count * 2);
        else
            printf("[MQ135]  Warmed up — Rs/R0 in clean air should be ~1.0\n");

        if (vaout < 0.1f)
            printf("[MQ135]  WARNING: AOUT near zero — check 5V supply and divider\n");

        printf("=================================\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * APP MAIN
 * ════════════════════════════════════════════════════════════════════════════ */
void mq135_start(void *arg){
    printf("\n\n");
    printf("================================\n");
    printf("  MQ135 Air Quality Sensor Test \n");
    printf("================================\n\n");

    printf("[MQ135]  ADC pin : GPIO34 (ADC1_CH6)\n\n");

    mq135_adc_init();
    printf("\n[MQ135] ADC init complete\n\n");
}