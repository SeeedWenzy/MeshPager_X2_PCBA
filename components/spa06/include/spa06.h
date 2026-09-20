/**
 * @file  spa06.h
 * @brief SPA06-003 digital pressure & temperature sensor driver - public API
 *
 * C port of the Adafruit SPA06_003 Arduino driver, structured after the Bosch
 * BMM350 component in this project. The driver core (spa06.c) is interface
 * agnostic: the caller wires read/write/delay_us callbacks into a
 * #spa06_dev instance, then calls spa06_init().
 *
 * For ESP32-S3 two ready-made bridges are provided:
 *  - components/spa06/common.c        : standalone ESP-i2cdev bridge
 *    (spa06_esp_init / spa06_i2cdev_init) - own I2C port + GPIO pins.
 *  - components/sensor/spa06/spa06_bsp_i2c1.c : project integration on the
 *    shared meshpager I2C_1 bus (spa06_init_bsp_i2c1).
 *
 * Compensation math & register layout: SPA06-003 datasheet (Goermicro Ver2.0).
 * Original Arduino driver: Copyright (c) Adafruit Industries, MIT license.
 *
 * @license MIT
 */

#ifndef _SPA06_H
#define _SPA06_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include "spa06_defs.h"

/************************** Generic register access **********************/

/*!
 * @brief Read `len` bytes starting at `reg_addr`.
 * @param[in]  reg_addr : First register address.
 * @param[out] reg_data : Destination buffer (caller-allocated).
 * @param[in]  len      : Number of bytes to read.
 * @param[in]  dev      : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
                      struct spa06_dev *dev);

/*!
 * @brief Write `len` bytes starting at `reg_addr`.
 * @param[in] reg_addr : First register address.
 * @param[in] reg_data : Source buffer.
 * @param[in] len      : Number of bytes to write.
 * @param[in] dev      : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint16_t len,
                      struct spa06_dev *dev);

/*!
 * @brief Block for up to `timeout_ms` waiting for the delay callback.
 * @param[in] period_us : Delay in microseconds.
 * @param[in] dev       : Device instance.
 * @return SPA06_OK, or SPA06_E_NULL_PTR.
 */
int8_t spa06_delay_us(uint32_t period_us, const struct spa06_dev *dev);

/************************** Init / reset **********************************/

/*!
 * @brief Initialize the sensor.
 *
 * Verifies the chip ID, performs a soft reset, waits for the coefficients and
 * sensor-ready bits, reads the calibration coefficients, and applies the same
 * high-precision default configuration as the upstream Adafruit driver
 * (128x oversampling, 200 Hz rate, continuous pressure & temperature).
 *
 * The read/write/delay_us callbacks and intf_ptr must already be wired into
 * `dev` (e.g. via spa06_esp_init() or spa06_init_bsp_i2c1()).
 *
 * @param[in,out] dev : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_init(struct spa06_dev *dev);

/*!
 * @brief Perform a soft reset (same sequence as power-on reset).
 * @param[in] dev : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_soft_reset(struct spa06_dev *dev);

/*!
 * @brief Read the chip ID (ID register).
 * @param[out] chip_id : Destination for the ID byte.
 * @param[in]  dev     : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_get_chip_id(uint8_t *chip_id, struct spa06_dev *dev);

/************************** Status / readiness ****************************/

/*!
 * @brief Read the MEAS_CFG register.
 * @param[out] meas_cfg : Destination for the register value.
 * @param[in]  dev      : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_get_meas_cfg(uint8_t *meas_cfg, struct spa06_dev *dev);

/*!
 * @brief Check whether calibration coefficients are ready.
 * @param[out] ready : Non-zero when coefficients are ready.
 * @param[in]  dev   : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_is_coeff_ready(uint8_t *ready, struct spa06_dev *dev);

/*!
 * @brief Check whether the sensor (frontend) is ready.
 * @param[out] ready : Non-zero when ready.
 * @param[in]  dev   : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_is_sensor_ready(uint8_t *ready, struct spa06_dev *dev);

/*!
 * @brief Check whether a temperature result is ready.
 * @param[out] ready : Non-zero when ready.
 * @param[in]  dev   : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_is_temp_ready(uint8_t *ready, struct spa06_dev *dev);

/*!
 * @brief Check whether a pressure result is ready.
 * @param[out] ready : Non-zero when ready.
 * @param[in]  dev   : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_is_pressure_ready(uint8_t *ready, struct spa06_dev *dev);

/************************** Configuration *********************************/

/*!
 * @brief Set the pressure oversampling. Also updates the pressure result
 *        shift bit (P_SHIFT) required when oversampling exceeds 8x, and
 *        refreshes the cached value used for compensation.
 * @param[in] oversampling : Oversampling setting.
 * @param[in] dev          : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_set_pressure_oversampling(spa06_oversample_t oversampling,
                                       struct spa06_dev *dev);

/*!
 * @brief Get the pressure oversampling.
 * @param[out] oversampling : Destination for the setting.
 * @param[in]  dev          : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_get_pressure_oversampling(spa06_oversample_t *oversampling,
                                       struct spa06_dev *dev);

/*!
 * @brief Set the temperature oversampling. Also updates T_SHIFT and the
 *        cached value used for compensation.
 * @param[in] oversampling : Oversampling setting.
 * @param[in] dev          : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_set_temperature_oversampling(spa06_oversample_t oversampling,
                                          struct spa06_dev *dev);

/*!
 * @brief Get the temperature oversampling.
 * @param[out] oversampling : Destination for the setting.
 * @param[in]  dev          : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_get_temperature_oversampling(spa06_oversample_t *oversampling,
                                          struct spa06_dev *dev);

/*!
 * @brief Set the pressure measurement rate.
 * @param[in] rate : Rate setting.
 * @param[in] dev   : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_set_pressure_rate(spa06_rate_t rate, struct spa06_dev *dev);

/*!
 * @brief Get the pressure measurement rate.
 * @param[out] rate : Destination for the setting.
 * @param[in]  dev  : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_get_pressure_rate(spa06_rate_t *rate, struct spa06_dev *dev);

/*!
 * @brief Set the temperature measurement rate.
 * @param[in] rate : Rate setting.
 * @param[in] dev  : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_set_temperature_rate(spa06_rate_t rate, struct spa06_dev *dev);

/*!
 * @brief Get the temperature measurement rate.
 * @param[out] rate : Destination for the setting.
 * @param[in]  dev  : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_get_temperature_rate(spa06_rate_t *rate, struct spa06_dev *dev);

/*!
 * @brief Set the measurement mode.
 * @param[in] mode : Measurement mode.
 * @param[in] dev  : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_set_measurement_mode(spa06_meas_mode_t mode, struct spa06_dev *dev);

/*!
 * @brief Get the measurement mode.
 * @param[out] mode : Destination for the mode.
 * @param[in]  dev  : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_get_measurement_mode(spa06_meas_mode_t *mode, struct spa06_dev *dev);

/*!
 * @brief Set the interrupt polarity.
 * @param[in] polarity : Polarity setting.
 * @param[in] dev       : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_set_interrupt_polarity(spa06_int_polarity_t polarity,
                                    struct spa06_dev *dev);

/*!
 * @brief Enable / disable interrupt sources.
 * @param[in] fifo        : Enable FIFO-full interrupt.
 * @param[in] temp_ready  : Enable temperature-ready interrupt.
 * @param[in] pres_ready  : Enable pressure-ready interrupt.
 * @param[in] dev         : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_set_interrupt_source(uint8_t fifo, uint8_t temp_ready,
                                   uint8_t pres_ready, struct spa06_dev *dev);

/*!
 * @brief Enable / disable the temperature result shift bit.
 * @param[in] enable : Non-zero to enable.
 * @param[in] dev    : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_set_temp_shift(uint8_t enable, struct spa06_dev *dev);

/*!
 * @brief Enable / disable the pressure result shift bit.
 * @param[in] enable : Non-zero to enable.
 * @param[in] dev    : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_set_pressure_shift(uint8_t enable, struct spa06_dev *dev);

/************************** FIFO / status *********************************/

/*!
 * @brief Enable or disable the FIFO.
 * @param[in] enable : Non-zero to enable.
 * @param[in] dev    : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_enable_fifo(uint8_t enable, struct spa06_dev *dev);

/*!
 * @brief Check whether the FIFO is enabled.
 * @param[out] enabled : Non-zero when enabled.
 * @param[in]  dev     : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_is_fifo_enabled(uint8_t *enabled, struct spa06_dev *dev);

/*!
 * @brief Check whether the FIFO is empty.
 * @param[out] empty : Non-zero when empty.
 * @param[in]  dev   : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_is_fifo_empty(uint8_t *empty, struct spa06_dev *dev);

/*!
 * @brief Check whether the FIFO is full.
 * @param[out] full : Non-zero when full.
 * @param[in]  dev   : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_is_fifo_full(uint8_t *full, struct spa06_dev *dev);

/*!
 * @brief Read the interrupt status flags (INT_STS, lower 3 bits).
 * @param[out] status : Destination for the flags.
 * @param[in]  dev     : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_get_status(uint8_t *status, struct spa06_dev *dev);

/*!
 * @brief Flush the FIFO.
 * @param[in] dev : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_flush_fifo(struct spa06_dev *dev);

/************************** Measurement data ******************************/

/*!
 * @brief Read the 24-bit raw pressure data (2's complement, sign-extended).
 * @param[out] raw_pres : Destination for the raw value.
 * @param[in]  dev      : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_get_pressure_data(int32_t *raw_pres, struct spa06_dev *dev);

/*!
 * @brief Read the 24-bit raw temperature data (2's complement, sign-extended).
 * @param[out] raw_temp : Destination for the raw value.
 * @param[in]  dev      : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_get_temperature_data(int32_t *raw_temp, struct spa06_dev *dev);

/*!
 * @brief Read the compensated temperature in degrees Celsius.
 * @param[out] temperature : Temperature in degC.
 * @param[in]  dev         : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_read_temperature(float *temperature, struct spa06_dev *dev);

/*!
 * @brief Read the compensated pressure in hectopascals (hPa).
 * @param[out] pressure : Pressure in hPa.
 * @param[in]  dev      : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_read_pressure(float *pressure, struct spa06_dev *dev);

/*!
 * @brief Calculate the compensated temperature from a raw reading.
 * @param[in]  raw_temp : Raw 24-bit temperature value.
 * @param[in]  dev      : Device instance (uses cached temp oversampling).
 * @return Compensated temperature in degC.
 */
float spa06_calculate_temperature(int32_t raw_temp, struct spa06_dev *dev);

/*!
 * @brief Calculate the compensated pressure from raw readings.
 * @param[in] raw_pres : Raw 24-bit pressure value.
 * @param[in] raw_temp : Raw 24-bit temperature value (for compensation).
 * @param[in] dev      : Device instance (uses cached oversampling settings).
 * @return Compensated pressure in hPa.
 */
float spa06_calculate_pressure(int32_t raw_pres, int32_t raw_temp,
                               struct spa06_dev *dev);

/*!
 * @brief Get the compensation scale factor for an oversampling setting.
 * @param[in] oversample : Oversampling rate.
 * @return Scale factor (kP or kT) from datasheet Table 4.
 */
float spa06_get_scaling_factor(spa06_oversample_t oversample);

/*!
 * @brief Read the calibration coefficients from the sensor.
 *
 * Called automatically by spa06_init(); exposed for the rare case where a
 * re-read is needed after a reset.
 * @param[in] dev : Device instance.
 * @return SPA06_OK on success, negative error code otherwise.
 */
int8_t spa06_read_coefficients(struct spa06_dev *dev);

#ifdef __cplusplus
} /* extern "C" */
#endif /* End of CPP guard */

#endif /* _SPA06_H */
