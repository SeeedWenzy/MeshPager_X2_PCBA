#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "wifi_ap_test.h"

static const char *TAG = "COEX_WIFI";
static bool s_started;
static esp_netif_t *s_ap_netif;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)data;
    if (base != WIFI_EVENT) {
        return;
    }
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "station connected");
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "station disconnected");
    }
}

esp_err_t wifi_ap_test_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (s_ap_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "Wi-Fi init failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                    wifi_event_handler, NULL),
                        TAG, "Wi-Fi event handler failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "Wi-Fi storage failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "Wi-Fi mode failed");

    wifi_config_t config = {
        .ap = {
            .ssid = "wireless_coex_ap",
            .channel = 1,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 4,
            .beacon_interval = 100,
        },
    };
    config.ap.ssid_len = strlen((const char *)config.ap.ssid);
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &config), TAG, "AP config failed");

    ESP_RETURN_ON_ERROR(esp_wifi_set_protocol(WIFI_IF_AP,WIFI_PROTOCOL_11B |WIFI_PROTOCOL_11G |WIFI_PROTOCOL_11N),TAG,"Wi-Fi protocol failed");

    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");
    s_started = true;

    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    ESP_LOGI(TAG, "AP started: SSID=%s channel=%u MAC=%02x:%02x:%02x:%02x:%02x:%02x",
             (char *)config.ap.ssid, config.ap.channel,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    wifi_ap_test_print_status();
    return ESP_OK;
}

esp_err_t wifi_ap_test_set_tx_power_dbm(double dbm)
{
    const double quarter_dbm = dbm * 4.0;
    if (dbm < 2.0 || dbm > 20.0 || fabs(quarter_dbm - round(quarter_dbm)) > 1e-9) {
        return ESP_ERR_INVALID_ARG;
    }

    /* ESP-IDF uses 0.25 dBm units for this API. */
    esp_err_t ret = esp_wifi_set_max_tx_power((int8_t)round(quarter_dbm));
    if (ret == ESP_OK) {
        int8_t actual = 0;
        ret = esp_wifi_get_max_tx_power(&actual);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Wi-Fi max TX power requested=%0.2f dBm, effective=%0.2f dBm",
                     dbm, actual / 4.0f);
        }
    }
    return ret;
}

void wifi_ap_test_print_status(void)
{
    if (!s_started) {
        ESP_LOGI(TAG, "AP not started");
        return;
    }
    wifi_sta_list_t stations = {0};
    esp_err_t ret = esp_wifi_ap_get_sta_list(&stations);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "AP stations=%u", (unsigned)stations.num);
    }
    if (s_ap_netif != NULL) {
        esp_netif_ip_info_t ip = {0};
        if (esp_netif_get_ip_info(s_ap_netif, &ip) == ESP_OK) {
            ESP_LOGI(TAG, "AP IPv4=" IPSTR, IP2STR(&ip.ip));
        }
    }
}
