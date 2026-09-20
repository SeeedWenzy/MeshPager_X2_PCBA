#include <assert.h>
#include <string.h>

#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ble_coex.h"
#include "ble_coex_gatt.h"

static const char *TAG = "COEX_GATT";
static uint8_t s_value[244];
static uint16_t s_value_len;
static uint16_t s_value_handle;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint32_t s_write_count;
static uint32_t s_notify_count;

static const ble_uuid128_t s_service_uuid = BLE_UUID128_INIT(
    0x9a, 0x8d, 0x3b, 0x6a, 0x7b, 0x12, 0x4d, 0x80,
    0x9e, 0xa1, 0x21, 0x44, 0x00, 0x00, 0x00, 0x01);
static const ble_uuid128_t s_characteristic_uuid = BLE_UUID128_INIT(
    0x9a, 0x8d, 0x3b, 0x6a, 0x7b, 0x12, 0x4d, 0x80,
    0x9e, 0xa1, 0x21, 0x44, 0x00, 0x00, 0x00, 0x02);

static int gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)arg;
    if (attr_handle != s_value_handle) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, s_value, s_value_len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t length = OS_MBUF_PKTLEN(ctxt->om);
        if (length > sizeof(s_value)) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if (ble_hs_mbuf_to_flat(ctxt->om, s_value, sizeof(s_value), &s_value_len) != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        s_write_count++;
        ESP_LOGI(TAG, "write len=%u count=%lu", (unsigned)s_value_len, (unsigned long)s_write_count);
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            struct os_mbuf *om = ble_hs_mbuf_from_flat(s_value, s_value_len);
            if (om != NULL && ble_gatts_notify_custom(s_conn_handle, s_value_handle, om) == 0) {
                s_notify_count++;
            }
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def s_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_characteristic_uuid.u,
                .access_cb = gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
                .val_handle = &s_value_handle,
            },
            {0}
        },
    },
    {0}
};

void ble_coex_gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    (void)arg;
    (void)ctxt;
}

int ble_coex_gatt_init(void)
{
    s_value[0] = 'r';
    s_value[1] = 'e';
    s_value[2] = 'a';
    s_value[3] = 'd';
    s_value_len = 4;
    int rc = ble_gatts_count_cfg(s_services);
    if (rc != 0) {
        return rc;
    }
    return ble_gatts_add_svcs(s_services);
}

int ble_coex_gatt_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "connected handle=%u", (unsigned)s_conn_handle);
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected reason=%u", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_coex_restart_advertising();
        break;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated mtu=%u", (unsigned)event->mtu.value);
        break;
    default:
        break;
    }
    return 0;
}
