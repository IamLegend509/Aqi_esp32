/*
 * GP2Y1010AU0F Dust Sensor Test — ESP32 WROOM-32E (ESP-IDF v5.x)
 * Prints raw ADC, voltage, and dust density (µg/m³) every 1 second
 *
 * Pin Map:
 *   Pin 1 (V-LED)  → 5V via 150Ω resistor (+ 220µF cap to GND near sensor)
 *   Pin 2 (LED-GND)→ GND
 *   Pin 3 (LED)    → GPIO32  (pulse control)
 *   Pin 4 (S-GND)  → GND
 *   Pin 5 (Vo)     → 10kΩ/20kΩ divider → GPIO33 (ADC1_CH5)
 *   Pin 6 (Vcc)    → 5V
 *
 * Timing (per Sharp datasheet):
 *   Pulse LOW 280µs → sample ADC → wait 40µs → pulse HIGH → wait 9680µs
 *
 * Voltage divider on Vo (0–5V → 0–3.3V):
 *   Vo ── 10kΩ ──┬── GPIO33
 *                │
 *               20kΩ (or 2x10kΩ series)
 *                │
 *               GND
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "rom/ets_sys.h"
#include "esp_rom_gpio.h"

/* ─── Pins ───────────────────────────────────────────────────────────────── */
#define DUST_LED_PIN      GPIO_NUM_32
#define DUST_ADC_CHANNEL  ADC_CHANNEL_5    // GPIO33 on ADC1
#define DUST_ADC_ATTEN    ADC_ATTEN_DB_12  // 0–3.9V range
#define DUST_ADC_BITWIDTH ADC_BITWIDTH_12  // 0–4095

/* ─── Timing (µs) ────────────────────────────────────────────────────────── */
#define PULSE_ON_US    280
#define SAMPLE_WAIT_US  40
#define PULSE_OFF_US  9680

/* ─── Sensor calibration ─────────────────────────────────────────────────── */
#define VOUT_CLEAN     0.9f   // Vo in clean air — calibrate after 5 min warmup
#define SENSITIVITY    0.5f   // V per 0.1 mg/m³ (Sharp datasheet)
#define DIVIDER_FACTOR 1.5f   // 10kΩ + 20kΩ: Vo = Vgpio * 1.5

/* ─── ADC handles ────────────────────────────────────────────────────────── */
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t         cali_handle = NULL;
static bool                      cali_ok     = false;

/* ════════════════════════════════════════════════════════════════════════════
 * ADC INIT — oneshot API (IDF v5.x)
 * ════════════════════════════════════════════════════════════════════════════ */
static void dust_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = DUST_ADC_ATTEN,
        .bitwidth = DUST_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, DUST_ADC_CHANNEL, &chan_cfg));

    // Try curve fitting calibration first (ESP32 supports line fitting)
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = DUST_ADC_CHANNEL,
        .atten    = DUST_ADC_ATTEN,
        .bitwidth = DUST_ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle) == ESP_OK) {
        cali_ok = true;
        printf("[DUST]  ADC cal: Curve Fitting\n");
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!cali_ok) {
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id  = ADC_UNIT_1,
            .atten    = DUST_ADC_ATTEN,
            .bitwidth = DUST_ADC_BITWIDTH,
        };
        if (adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle) == ESP_OK) {
            cali_ok = true;
            printf("[DUST]  ADC cal: Line Fitting\n");
        }
    }
#endif

    if (!cali_ok)
        printf("[DUST]  ADC cal: None — raw estimate only\n");
}

/* ════════════════════════════════════════════════════════════════════════════
 * SINGLE DUST READING — timing critical, no printf inside
 * ════════════════════════════════════════════════════════════════════════════ */
static int dust_read_raw_single(void)
{
    gpio_set_level(DUST_LED_PIN, 0);
    ets_delay_us(PULSE_ON_US);

    int raw = 0;
    adc_oneshot_read(adc_handle, DUST_ADC_CHANNEL, &raw);

    ets_delay_us(SAMPLE_WAIT_US);
    gpio_set_level(DUST_LED_PIN, 1);
    ets_delay_us(PULSE_OFF_US);

    return raw;
}

/* ════════════════════════════════════════════════════════════════════════════
 * AVERAGED READING — 10 samples (~100ms total)
 * ════════════════════════════════════════════════════════════════════════════ */
static float dust_read_voltage(void)
{
    int sum = 0;
    for (int i = 0; i < 10; i++)
        sum += dust_read_raw_single();

    int raw_avg = sum / 10;

    int mv = 0;
    if (cali_ok)
        adc_cali_raw_to_voltage(cali_handle, raw_avg, &mv);
    else
        mv = (int)(raw_avg * 3300.0f / 4095.0f);

    return (mv / 1000.0f) * DIVIDER_FACTOR;
}

/* ════════════════════════════════════════════════════════════════════════════
 * DUST DENSITY + CATEGORY
 * ════════════════════════════════════════════════════════════════════════════ */
static float dust_density(float vo)
{
    if (vo < VOUT_CLEAN) return 0.0f;
    return ((vo - VOUT_CLEAN) / SENSITIVITY) * 100.0f;
}

static const char* dust_category(float ugm3)
{
    if (ugm3 <  12.0f) return "Good";
    if (ugm3 <  35.4f) return "Moderate";
    if (ugm3 <  55.4f) return "Unhealthy for Sensitive Groups";
    if (ugm3 < 150.4f) return "Unhealthy";
    if (ugm3 < 250.4f) return "Very Unhealthy";
    return                     "Hazardous";
}

/* ════════════════════════════════════════════════════════════════════════════
 * TASK
 * ════════════════════════════════════════════════════════════════════════════ */
static void dust_task(void *arg)
{
    int count = 0;

    printf("\n[DUST]  Starting readings...\n");
    printf("[DUST]  Stabilise 5 min before trusting density values.\n");
    printf("[DUST]  If Vo ~ 0V: check 150Ω + 220µF on Pin1, and 5V on Pin6.\n\n");

    while (1) {
        count++;

        float vo      = dust_read_voltage();
        float density = dust_density(vo);
        float vo_gpio = vo / DIVIDER_FACTOR;

        printf("\n========== Reading #%d ==========\n", count);
        printf("[DUST]  GPIO Voltage : %.3f V\n",     vo_gpio);
        printf("[DUST]  Vo (actual)  : %.3f V\n",     vo);
        printf("[DUST]  Dust Density : %.1f µg/m³\n", density);
        printf("[DUST]  Category     : %s\n",           dust_category(density));

        if (count <= 150)
            printf("[DUST]  Warmup       : %ds / 300s\n", count);

        if (vo < 0.1f)
            printf("[DUST]  WARNING: Vo near zero — check Pin1 circuit and Pin6 5V\n");
        else if (vo > 4.5f)
            printf("[DUST]  WARNING: Vo very high — check voltage divider on Pin5\n");

        printf("=================================\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * APP MAIN
 * ════════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    printf("\n\n");
    printf("================================\n");
    printf("  GP2Y1010AU0F Dust Sensor Test \n");
    printf("  ESP32 WROOM-32E / IDF v5.x    \n");
    printf("  115200 baud                   \n");
    printf("================================\n\n");

    printf("[DUST]  LED pulse pin : GPIO32\n");
    printf("[DUST]  ADC pin       : GPIO33 (ADC1_CH5)\n");
    printf("[DUST]  Divider factor: %.1f (10k + 20k)\n\n", DIVIDER_FACTOR);

    esp_rom_gpio_pad_select_gpio(DUST_LED_PIN);
    gpio_set_direction(DUST_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DUST_LED_PIN, 1);   // LED off by default

    dust_adc_init();

    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskCreate(dust_task, "dust_task", 4096, NULL, 5, NULL);
}