#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t wifi_ap_test_start(void);
void wifi_ap_test_print_status(void);
esp_err_t wifi_ap_test_set_tx_power_dbm(double dbm);

#ifdef __cplusplus
}
#endif
