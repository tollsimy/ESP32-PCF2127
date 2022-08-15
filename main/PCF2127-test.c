#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ESP32_PCF2127.h"

static const char *TAG = "Demo-PCF2127";

#define SDA_PIN (27)
#define SCL_PIN (32)
#define I2C_PORT (0)

I2C_CONF={
    .mode = I2C_MODE_MASTER;
    .sda_io_num = SDA_PIN;
    .scl_io_num = SCL_PIN;
    .sda_pullup_en = GPIO_PULLUP_DISABLE;     //disable if you have external pullup
    .scl_pullup_en = GPIO_PULLUP_DISABLE;
    .master.clk_speed = 400000;               //I2C Full Speed
}

void PCF2127_task(){
    //Install I2C Driver
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &(I2C_CONF)));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    for(;;){
        ESP_LOGI(TAG, "PCF2127 task started");
        ESP32_PCF2127 PCF={0};
        ESP_ERROR_CHECK(PCF_init(&PCF, I2C_PORT));

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

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_PORT));
}

void app_main(void)
{   
    xTaskCreate(&PCF2127_task, "PCF2127_task", 4096, NULL, 5, NULL);
}

