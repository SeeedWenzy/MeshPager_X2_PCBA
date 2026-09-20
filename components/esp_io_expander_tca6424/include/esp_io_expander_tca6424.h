#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_io_expander.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TCA6424_AUTO_INCREMENT_BIT (0x80) // OR with register address to enable auto-increment for sequential register access
esp_err_t esp_io_expander_new_i2c_tca6424(i2c_master_bus_handle_t i2c_bus, uint32_t dev_addr, esp_io_expander_handle_t *handle_ret);

enum esp_io_expander_tca_6424_address {
    ESP_IO_EXPANDER_I2C_TCA6424_ADDRESS_GND = 0x22,
    ESP_IO_EXPANDER_I2C_TCA6424_ADDRESS_VCC = 0x23,
};

#ifdef __cplusplus
}
#endif