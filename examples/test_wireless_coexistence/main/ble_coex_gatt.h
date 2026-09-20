#pragma once

#include "host/ble_hs.h"

int ble_coex_gatt_init(void);
void ble_coex_gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
int ble_coex_gatt_gap_event(struct ble_gap_event *event, void *arg);
