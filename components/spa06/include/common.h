/**
 * @file  common.h
 * @brief SPA06-003 ESP-IDF platform bridge (standalone ESP-i2cdev path)
 *
 * Mirrors the layout of the BMM350 component's common.h: the driver core
 * (spa06.c) is interface agnostic, and this file wires the read/write/delay_us
 * callbacks of a #spa06_dev to the ESP-IDF `i2cdev` component. Use this when
 * you want to drive the sensor on its own I2C port/GPIOs.
 *
 * For the project-shared I2C_1 bus, use components/sensor/spa06/spa06_bsp_i2c1
 * instead - it does not need this file.
 *
 * @license MIT
 */

#ifndef _COMMON_H
#define _COMMON_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>

#include "spa06.h"

/***************************************************************************/
/*!                 User function prototypes                              */
/***************************************************************************/

/*!
 * @brief Read the sensor over I2C (i2cdev backend).
 * @return SPA06_INTF_RET_SUCCESS on success, non-zero on failure.
 */
SPA06_INTF_RET_TYPE spa06_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                    uint32_t length, void *intf_ptr);

/*!
 * @brief Write the sensor over I2C (i2cdev backend).
 * @return SPA06_INTF_RET_SUCCESS on success, non-zero on failure.
 */
SPA06_INTF_RET_TYPE spa06_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                     uint32_t length, void *intf_ptr);

/*!
 * @brief Microsecond delay matching the callback signature.
 *
 * Named spa06_delay (not spa06_delay_us) to avoid clashing with the core API
 * spa06_delay_us() declared in spa06.h - mirrors bmm350_delay() in
 * components/bmm350/common.h.
 */
void spa06_delay(uint32_t period_us, void *intf_ptr);

/*!
 * @brief Print a human-readable description for an SPA06 error code.
 */
void spa06_error_codes_print_result(const char api_name[], int8_t rslt);

/***************************************************************************/
/*!                 ESP-IDF Platform Specific Functions                    */
/***************************************************************************/

/*!
 * @brief Initialize an i2c_dev_t descriptor for the SPA06.
 *
 * Follows the sht4x_init_desc() / bmm350_i2cdev_init() pattern: sets the I2C
 * port, slave address and SDA/SCL GPIOs and creates the access mutex. The
 * caller owns `i2c_dev` and must free it with spa06_i2cdev_free().
 *
 * @param[in,out] i2c_dev  Descriptor to initialize.
 * @param[in]     port     I2C port (I2C_NUM_0 or I2C_NUM_1).
 * @param[in]     sda_gpio SDA GPIO.
 * @param[in]     scl_gpio SCL GPIO.
 * @param[in]     addr     Slave address (0 -> SPA06_I2C_ADDR_DEFAULT).
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t spa06_i2cdev_init(i2c_dev_t *i2c_dev, i2c_port_t port,
                             gpio_num_t sda_gpio, gpio_num_t scl_gpio,
                             uint8_t addr);

/*!
 * @brief Free an i2c_dev_t descriptor created by spa06_i2cdev_init().
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t spa06_i2cdev_free(i2c_dev_t *i2c_dev);

/*!
 * @brief One-shot convenience: allocate + wire an i2c_dev_t into a #spa06_dev.
 *
 * Allocates an i2c_dev_t (caller frees with spa06_esp_deinit()), initializes
 * it on the given port/GPIOs, and assigns the read/write/delay_us callbacks.
 * After this returns, call spa06_init(dev).
 *
 * @param[in,out] dev      Device instance to wire up.
 * @param[in]     port     I2C port.
 * @param[in]     sda_gpio SDA GPIO.
 * @param[in]     scl_gpio SCL GPIO.
 * @param[in]     addr     Slave address (0 -> SPA06_I2C_ADDR_DEFAULT).
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_esp_init(struct spa06_dev *dev, i2c_port_t port,
                       gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint8_t addr);

/*!
 * @brief Release resources allocated by spa06_esp_init().
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_esp_deinit(struct spa06_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _COMMON_H */
