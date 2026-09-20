#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_ldo_regulator.h"
#include "esp_vfs_fat.h"
#include "driver/gpio.h"
#include "config.h"
#include "esp-bsp.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "esp_io_expander_tca6424.h"
#include "meshpager_x2.h"

static const char *TAG = "Mesh Handler L1 test";
#define APP_SW_VERSION "meshpager_x2-example-2.0.0"
void app_main(void)
{
    esp_err_t ret = bsp_power_up_init();
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Power state is off, skip example startup");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Application software version: SW=%s", APP_SW_VERSION);
    ESP_LOGI(TAG, "Mesh Handler L1 board power up...");
    while(1)
    {
        ESP_LOGI(TAG, "Mesh Handler L1 board is running...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);  
    }
}