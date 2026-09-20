#include <stdbool.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ble_coex_gatt.h"

static const char *TAG = "COEX_BLE";
static bool s_synced;
static bool s_started;
static uint8_t s_own_addr_type;

extern void ble_store_config_init(void);
int ble_coex_gatt_init(void);
int ble_coex_gatt_gap_event(struct ble_gap_event *event, void *arg);

static void ble_on_reset(int reason)
{
    s_synced = false;
    ESP_LOGE(TAG, "BLE host reset reason=%d", reason);
}

static void ble_on_sync(void)
{
    if (ble_hs_util_ensure_addr(0) != 0 || ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) {
        ESP_LOGE(TAG, "BLE address setup failed");
        return;
    }
    s_synced = true;
}

static void ble_advertise(void)
{
    struct ble_hs_adv_fields fields = {0};
    struct ble_gap_adv_params params = {0};
    const char *name = ble_svc_gap_device_name();

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    if (ble_gap_adv_set_fields(&fields) != 0) {
        ESP_LOGE(TAG, "BLE advertising data failed");
        return;
    }
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    int rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                               &params, ble_coex_gatt_gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "BLE advertising failed rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "BLE advertising: wireless_coex");
    }
}

static void ble_host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_coex_restart_advertising(void)
{
    if (s_synced && !ble_gap_adv_active()) {
        ble_advertise();
    }
}

esp_err_t ble_coex_set_tx_power_dbm(int dbm)
{
    static const esp_power_level_t levels[] = {
        ESP_PWR_LVL_N12, ESP_PWR_LVL_N9, ESP_PWR_LVL_N6, ESP_PWR_LVL_N3,
        ESP_PWR_LVL_N0, ESP_PWR_LVL_P3, ESP_PWR_LVL_P6, ESP_PWR_LVL_P9,
    };
    if (dbm != -12 && dbm != -9 && dbm != -6 && dbm != -3 &&
        dbm != 0 && dbm != 3 && dbm != 6 && dbm != 9) {
        return ESP_ERR_INVALID_ARG;
    }
    int index = (dbm + 12) / 3;
    esp_err_t ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, levels[index]);
    if (ret == ESP_OK) {
        ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, levels[index]);
    }
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BLE default/advertising TX power=%d dBm", dbm);
    }
    return ret;
}

esp_err_t ble_coex_start(void)
{
    if (s_started) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(nimble_port_init(), TAG, "NimBLE init failed");
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.gatts_register_cb = ble_coex_gatt_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_RETURN_ON_FALSE(ble_svc_gap_device_name_set("wireless_coex") == 0,
                        ESP_FAIL, TAG, "BLE device name failed");
    ESP_RETURN_ON_FALSE(ble_coex_gatt_init() == 0, ESP_FAIL, TAG, "BLE GATT init failed");
    nimble_port_freertos_init(ble_host_task);

    int64_t deadline = esp_timer_get_time() + 5000000;
    while (!s_synced && esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_RETURN_ON_FALSE(s_synced, ESP_ERR_TIMEOUT, TAG, "BLE sync timeout");
    s_started = true;
    ble_advertise();
    return ESP_OK;
}
