#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "config.h"
#include "esp_codec_dev.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif






/**************************************************************************************************
 *
 * I2C_0 interface
 *
 * There are multiple devices connected to I2C peripheral:
 *  - IO expander TCA6424
 *  - RTC YSN8900E
 * 
 **************************************************************************************************/

/**
 * @brief Init I2C_0 driver
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *      - ESP_FAIL              I2C driver installation error
 *
 */
esp_err_t bsp_i2c_0_init(void);

/**
 * @brief Deinit I2C_0 driver and free its resources
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *
 */
esp_err_t bsp_i2c_0_deinit(void);

/**
 * @brief Get I2C_0 driver handle
 *
 * @return
 *      - I2C handle
 *
 */
i2c_master_bus_handle_t bsp_i2c_0_get_handle(void);

/**
 * @brief Scan slave devices on I2C_0
 *
 * @return
 *      - ESP_OK                Scan completed
 *      - ESP_ERR_INVALID_STATE I2C bus handle is invalid
 *
 */
esp_err_t bsp_i2c_0_scan(void);



/**************************************************************************************************
 *
 * I2C_1 interface
 *
 * There are multiple devices connected to I2C peripheral:
 *  - SHT4x temperature and humidity sensor
 *  - LSM6DSO IMU sensor
 *  - BMM350 magnetometer sensor
 *  - ADC ES7243
 *  - DAC ES8311
 **************************************************************************************************/

/**
 * @brief Init I2C_1 driver
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *      - ESP_FAIL              I2C driver installation error
 *
 */
esp_err_t bsp_i2c_1_init(void);

/**
 * @brief Deinit I2C_1 driver and free its resources
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *
 */
esp_err_t bsp_i2c_1_deinit(void);

/**
 * @brief Get I2C_1 driver handle
 *
 * @return
 *      - I2C handle
 *
 */
i2c_master_bus_handle_t bsp_i2c_1_get_handle(void);

/**
 * @brief Scan slave devices on I2C_1
 *
 * @return
 *      - ESP_OK                Scan completed
 *      - ESP_ERR_INVALID_STATE I2C bus handle is invalid
 *
 */
esp_err_t bsp_i2c_1_scan(void);





#ifdef __cplusplus
}
#endif
