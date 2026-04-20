#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "adc_shared.h"
#include "dht22.h"
#include "bmp280.h"
#include "mq7.h"
#include "mq135.h"

void app_main(void)
{
    printf("\n=== AQI System Starting ===\n\n");
    // Create ONE shared ADC unit for all analog sensors
    adc_shared_init();


    // Init all sensors — pass shared handle to analog sensors
    bmp280_start();
    mq7_start();
    mq135_start();
    dht22_start();

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Launch tasks
    xTaskCreate(dht22_task,  "dht22",  4096, NULL, 5, NULL);
    xTaskCreate(bmp280_task, "bmp280", 4096, NULL, 5, NULL);
    xTaskCreate(mq7_task,    "mq7",    4096, NULL, 5, NULL);
    xTaskCreate(mq135_task,  "mq135",  4096, NULL, 5, NULL);
}