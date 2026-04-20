#include "adc_shared.h"
#include "esp_err.h"

adc_oneshot_unit_handle_t adc_handle;

void adc_shared_init(void)
{
    adc_oneshot_unit_init_cfg_t cfg = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg, &adc_handle));
}