#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "bmm350.h"


#ifdef __cplusplus
extern "C" {
#endif

#define BMM350_BSP_I2C1_ADDRESS_DEFAULT   BMM350_I2C_ADSEL_SET_HIGH
#define BMM350_BSP_I2C1_CLK_HZ            (100000U)

typedef struct {
	void *dev_handle;
	uint16_t i2c_address;
} bmm350_bsp_i2c1_context_t;

esp_err_t bmm350_init_bsp_i2c1(struct bmm350_dev *dev, bmm350_bsp_i2c1_context_t *context);
esp_err_t bmm350_deinit_bsp_i2c1(struct bmm350_dev *dev, bmm350_bsp_i2c1_context_t *context);

#ifdef __cplusplus
}
#endif