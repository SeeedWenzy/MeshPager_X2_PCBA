#include "app_storage.h"

#include "esp_err.h"
#include "esp_log.h"
#include "meshpager_x2.h"

static const char *TAG = "APP_STORAGE";

static bool s_sd_available = false;

void app_storage_init(void)
{
    esp_err_t ret = bsp_sdcard_mount();
    if (ret == ESP_OK) {
        s_sd_available = true;
        ESP_LOGI(TAG, "SD card mounted");
    } else {
        s_sd_available = false;
        ESP_LOGW(TAG, "SD card not available (%s); will use SPIFFS asset partition",
                 esp_err_to_name(ret));
    }

    ret = bsp_spiffs_mount();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS asset partition mount failed: %s", esp_err_to_name(ret));
    }
}

bool app_storage_sd_available(void)
{
    return s_sd_available;
}
