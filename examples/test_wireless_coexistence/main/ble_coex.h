#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ble_coex_start(void);
esp_err_t ble_coex_set_tx_power_dbm(int dbm);
void ble_coex_restart_advertising(void);

#ifdef __cplusplus
}
#endif
