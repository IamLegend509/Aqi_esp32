#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "DHT22.h"
#include "BMP280.h"
#include "MQ7.h"
#include "MQ135.h"

void app_main(void)
{
    printf("\n=== AQI System Starting ===\n");

    // Init (only if these exist in your files)
    bmp280_start();   // BMP280 is lowercase in your file
    MQ7_start();      // MQ7 is uppercase
    MQ135_start();    // MQ135 is uppercase
    DHT22_start();    // DHT22 is uppercase

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Launch tasks (MUST match exact function names in .c files)
    xTaskCreate(DHT22_task,  "dht22",  4096, NULL, 5, NULL);
    xTaskCreate(bmp280_task, "bmp280", 4096, NULL, 5, NULL);
    xTaskCreate(MQ7_task,    "mq7",    4096, NULL, 5, NULL);
    xTaskCreate(MQ135_task,  "mq135",  4096, NULL, 5, NULL);
}