#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "spa06.h"


#ifdef __cplusplus
extern "C" {
#endif

#define SPA06_BSP_I2C1_ADDRESS_DEFAULT   SPA06_I2C_ADDR_DEFAULT
#define SPA06_BSP_I2C1_CLK_HZ            (100000U)

typedef struct {
	void *dev_handle;
	uint8_t i2c_address;
} spa06_bsp_i2c1_context_t;

/*!
 * @brief Wire a #spa06_dev onto the shared I2C_1 bus.
 *
 * Adds an I2C device for the SPA06 on the meshpager I2C_1 bus and assigns the
 * read/write/delay_us callbacks. After this returns, call spa06_init(dev).
 * Mirrors bmm350_init_bsp_i2c1().
 *
 * @param[in,out] dev     Device instance (callbacks + intf_ptr are populated).
 * @param[in,out] context Caller-owned context (dev_handle is populated).
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t spa06_init_bsp_i2c1(struct spa06_dev *dev, spa06_bsp_i2c1_context_t *context);

/*!
 * @brief Remove the SPA06 device from the shared I2C_1 bus.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t spa06_deinit_bsp_i2c1(struct spa06_dev *dev, spa06_bsp_i2c1_context_t *context);

#ifdef __cplusplus
}
#endif
