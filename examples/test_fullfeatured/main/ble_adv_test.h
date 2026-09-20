#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize NimBLE stack and prepare for advertising.
 *
 * Call once before ble_adv_test_start(). NVS must already be initialized.
 *
 * @return ESP_OK on success
 */
esp_err_t ble_adv_test_init(void);

/**
 * @brief Start BLE legacy advertising (non-connectable, general discoverable).
 *
 * @return ESP_OK on success
 */
esp_err_t ble_adv_test_start(void);

/**
 * @brief Stop BLE advertising.
 */
void ble_adv_test_stop(void);

/**
 * @brief De-initialize NimBLE stack (stop host task + deinit controller).
 *
 * After this call, ble_adv_test_init() can be called again.
 */
void ble_adv_test_deinit(void);

#ifdef __cplusplus
}
#endif
