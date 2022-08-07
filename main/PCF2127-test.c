#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ESP32_PCF2127.h"

static const char *TAG = "Demo-PCF2127";

uint8_t PCF_SDA_PIN=18;
uint8_t PCF_SCL_PIN=19;
uint8_t PCF_I2C_PORT=0;

void PCF2127_task(){
    for(;;){
        ESP_LOGI(TAG, "PCF2127 task started");
        ESP32_PCF2127 PCF={0};
        ESP_ERROR_CHECK(PCF_init(&PCF));

        PCF.time.tm_sec=0;
        PCF.time.tm_min=25;
        PCF.time.tm_hour=8;
        PCF.time.tm_mday=3;
        PCF.time.tm_wday=5;
        PCF.time.tm_mon=3;
        PCF.time.tm_year=120;
        PCF_rtc_set_time(&PCF);
        ESP_LOGI(TAG, "PCF2127 time set to %d:%d:%d", PCF.time.tm_hour, PCF.time.tm_min, PCF.time.tm_sec);

        ESP_LOGI(TAG, "Waiting 10 secs");
        vTaskDelay(10000/portTICK_PERIOD_MS);
        PCF_rtc_read_time(&PCF);
        ESP_LOGI(TAG, "Time: %d:%d:%d", PCF.time.tm_hour, PCF.time.tm_min, PCF.time.tm_sec);

        PCF_delete();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{   
    xTaskCreate(&PCF2127_task, "PCF2127_task", 4096, NULL, 5, NULL);
}

