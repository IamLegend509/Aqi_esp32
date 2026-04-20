#pragma once
#include "esp_adc/adc_oneshot.h"

extern adc_oneshot_unit_handle_t adc_handle;

void adc_shared_init(void);