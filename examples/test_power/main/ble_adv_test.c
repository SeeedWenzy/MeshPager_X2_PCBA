/*
 * Minimal BLE advertising test module for power measurement.
 * Derived from examples/test_pcba/main/ble_app.c but simplified
 * to only support legacy non-connectable advertising for power testing.
 */

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ble_adv_test.h"

static const char *TAG = "BLE_ADV_TEST";

/* Provided by NimBLE host store/config module (linked from libbt) */
extern void ble_store_config_init(void);

static uint8_t s_own_addr_type;
static bool s_synced = false;
static bool s_initialized = false;

/* Forward declarations */
static void ble_adv_on_sync(void);
static void ble_adv_on_reset(int reason);
static void ble_adv_host_task(void *param);
static int ble_adv_gap_event(struct ble_gap_event *event, void *arg);

/* ------------------------------------------------------------------ */
/* GAP event handler (minimal – only handle sync/advertising events)   */
/* ------------------------------------------------------------------ */
static int ble_adv_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A connection was established — shouldn't happen in non-connectable
         * mode, but handle gracefully by disconnecting immediately. */
        ESP_LOGW(TAG, "Unexpected connection, disconnecting");
        ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected");
        break;

    default:
        break;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Sync callback – called when BLE controller + host are synchronized  */
/* ------------------------------------------------------------------ */
static void ble_adv_on_sync(void)
{
    int rc;

    /* Make sure we have a proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure addr failed: %d", rc);
        return;
    }

    /* Figure out address to use while advertising */
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "infer addr failed: %d", rc);
        return;
    }

    s_synced = true;
    ESP_LOGI(TAG, "BLE host synced, ready to advertise");
}

static void ble_adv_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset; reason=%d", reason);
    s_synced = false;
}

/* ------------------------------------------------------------------ */
/* NimBLE host task (runs until nimble_port_stop() is called)          */
/* ------------------------------------------------------------------ */
static void ble_adv_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */

esp_err_t ble_adv_test_init(void)
{
    esp_err_t ret;

    if (s_initialized) {
        return ESP_OK;
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure NimBLE host callbacks */
    ble_hs_cfg.reset_cb = ble_adv_on_reset;
    ble_hs_cfg.sync_cb = ble_adv_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Initialize BLE key store (needs NVS, which is already initialized
     * by bsp_power_up_init). Without this, IRK restore fails with status=8. */
    ble_store_config_init();

    /* Register essential GATT services */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* Set device name */
    ble_svc_gap_device_name_set("test_power");

    /* Start NimBLE host task */
    nimble_port_freertos_init(ble_adv_host_task);

    /* Wait for sync (timeout 5 seconds) */
    int64_t deadline = esp_timer_get_time() + 5000000;
    while (!s_synced && esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (!s_synced) {
        ESP_LOGE(TAG, "BLE sync timeout");
        nimble_port_stop();
        nimble_port_deinit();
        return ESP_ERR_TIMEOUT;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "BLE initialized");
    return ESP_OK;
}

esp_err_t ble_adv_test_start(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_gap_adv_params adv_params;
    const char *name;
    int rc;

    if (!s_initialized || !s_synced) {
        ESP_LOGE(TAG, "BLE not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    memset(&fields, 0, sizeof(fields));

    /* General discoverable, BLE-only */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Include TX power level */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    /* Device name */
    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "set adv fields failed: %d", rc);
        return ESP_FAIL;
    }

    /* Non-connectable undirected advertising for power measurement */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_adv_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv start failed: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BLE advertising started (non-connectable)");
    return ESP_OK;
}

void ble_adv_test_stop(void)
{
    if (ble_gap_adv_active()) {
        ble_gap_adv_stop();
        ESP_LOGI(TAG, "BLE advertising stopped");
    }
}

void ble_adv_test_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    ble_adv_test_stop();

    esp_err_t ret = nimble_port_stop();
    if (ret == ESP_OK) {
        nimble_port_deinit();
        ESP_LOGI(TAG, "BLE deinitialized");
    } else {
        ESP_LOGW(TAG, "nimble_port_stop failed: %s", esp_err_to_name(ret));
    }

    s_initialized = false;
    s_synced = false;
}
