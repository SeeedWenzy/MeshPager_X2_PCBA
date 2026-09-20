#ifndef _KODE_LSM6DSOX_H_
#define _KODE_LSM6DSOX_H_

#include "src/lsm6dsox_reg.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float temperature;
    stmdev_ctx_t ctx;
} kode_lsm6dsox_handle_t;

/**
 * @brief Initialize the LSM6DSOX sensor with I2C interface
 * 
 * @param bus_handle The I2C bus handle
 * @param device_addr I2C device address (0x6A or 0x6B based on SA0 pin)
 * @param handle Pointer to store the sensor handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_init_i2c(i2c_master_bus_handle_t bus_handle, uint8_t device_addr, kode_lsm6dsox_handle_t *handle);

/**
 * @brief Remove the LSM6DSOX device from its I2C bus
 *
 * @param handle Sensor handle obtained from init
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_deinit_i2c(kode_lsm6dsox_handle_t *handle);

/**
 * @brief Read the device ID to verify correct sensor connection
 * 
 * @param handle LSM6DSOX handle obtained from init
 * @param id Pointer to store the device ID (should be 0x6C for LSM6DSOX)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_check_id(kode_lsm6dsox_handle_t *handle, uint8_t *id);

/**
 * @brief Configure the sensor with default settings
 * 
 * @param handle LSM6DSOX handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_config_default(kode_lsm6dsox_handle_t *handle);

/**
 * @brief Configure accelerometer with specific settings
 * 
 * @param handle LSM6DSOX handle
 * @param odr Output data rate
 * @param fs Full scale range
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_accel_config(kode_lsm6dsox_handle_t *handle, 
                                    lsm6dsox_odr_xl_t odr, 
                                    lsm6dsox_fs_xl_t fs);

/**
 * @brief Configure gyroscope with specific settings
 * 
 * @param handle LSM6DSOX handle
 * @param odr Output data rate
 * @param fs Full scale range
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_gyro_config(kode_lsm6dsox_handle_t *handle, 
                                   lsm6dsox_odr_g_t odr, 
                                   lsm6dsox_fs_g_t fs);

/**
 * @brief Read accelerometer data
 * 
 * @param handle LSM6DSOX handle
 * @param x_mg Pointer to store x-axis acceleration in mg
 * @param y_mg Pointer to store y-axis acceleration in mg
 * @param z_mg Pointer to store z-axis acceleration in mg
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_read_accel(kode_lsm6dsox_handle_t *handle, 
                                  float *x_mg, float *y_mg, float *z_mg);

/**
 * @brief Read gyroscope data
 * 
 * @param handle LSM6DSOX handle
 * @param x_mdps Pointer to store x-axis angular rate in mdps
 * @param y_mdps Pointer to store y-axis angular rate in mdps
 * @param z_mdps Pointer to store z-axis angular rate in mdps
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_read_gyro(kode_lsm6dsox_handle_t *handle, 
                                 float *x_mdps, float *y_mdps, float *z_mdps);

/**
 * @brief Read temperature data
 * 
 * @param handle LSM6DSOX handle
 * @param temp_c Pointer to store temperature in celsius
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_read_temp(kode_lsm6dsox_handle_t *handle, float *temp_c);

/**
 * @brief Check if new accelerometer data is available
 * 
 * @param handle LSM6DSOX handle
 * @param val Pointer to store the flag (1 if data available, 0 otherwise)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_accel_data_ready(kode_lsm6dsox_handle_t *handle, uint8_t *val);

/**
 * @brief Check if new gyroscope data is available
 * 
 * @param handle LSM6DSOX handle
 * @param val Pointer to store the flag (1 if data available, 0 otherwise)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_gyro_data_ready(kode_lsm6dsox_handle_t *handle, uint8_t *val);

/**
 * @brief FIFO mode options
 */
typedef enum {
    KODE_LSM6DSOX_FIFO_BYPASS_MODE,    // FIFO disabled
    KODE_LSM6DSOX_FIFO_FIFO_MODE,      // Stops collecting when FIFO is full
    KODE_LSM6DSOX_FIFO_CONTINUOUS_MODE  // Overwrites oldest data when FIFO is full
} kode_lsm6dsox_fifo_mode_t;

/**
 * @brief Configure FIFO mode and watermark
 * 
 * @param handle LSM6DSOX handle
 * @param mode FIFO operation mode
 * @param watermark FIFO watermark level (0-511)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_fifo_config(kode_lsm6dsox_handle_t *handle, 
                                   kode_lsm6dsox_fifo_mode_t mode,
                                   uint16_t watermark);

/**
 * @brief Get number of unread samples in FIFO
 * 
 * @param handle LSM6DSOX handle
 * @param samples Pointer to store the number of samples
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_fifo_data_level(kode_lsm6dsox_handle_t *handle, uint16_t *samples);

/**
 * @brief Read accelerometer data from FIFO
 * 
 * @param handle LSM6DSOX handle
 * @param acc_mg Array to store acceleration data in mg (size should be at least [samples][3])
 * @param samples Number of samples to read
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_fifo_read_accel(kode_lsm6dsox_handle_t *handle, 
                                      float acc_mg[][3],
                                      uint16_t samples);

/**
 * @brief Read gyroscope data from FIFO
 * 
 * @param handle LSM6DSOX handle
 * @param gyro_mdps Array to store gyroscope data in mdps (size should be at least [samples][3])
 * @param samples Number of samples to read
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_fifo_read_gyro(kode_lsm6dsox_handle_t *handle, 
                                     float gyro_mdps[][3],
                                     uint16_t samples);

/**
 * @brief Flush the FIFO (clear all data)
 * 
 * @param handle LSM6DSOX handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_fifo_flush(kode_lsm6dsox_handle_t *handle);

/**
 * @brief Interrupt types that can be enabled on INT1 or INT2 pins
 */
typedef enum {
    KODE_LSM6DSOX_INT_DRDY_XL          = 0x01, /**< Accelerometer data ready */
    KODE_LSM6DSOX_INT_DRDY_G           = 0x02, /**< Gyroscope data ready */
    KODE_LSM6DSOX_INT_DRDY_TEMP        = 0x04, /**< Temperature data ready (only for INT2) */
    KODE_LSM6DSOX_INT_FIFO_TH          = 0x08, /**< FIFO threshold reached */
    KODE_LSM6DSOX_INT_FIFO_OVR         = 0x10, /**< FIFO overrun */
    KODE_LSM6DSOX_INT_FIFO_FULL        = 0x20, /**< FIFO full */
    KODE_LSM6DSOX_INT_FIFO_BDR         = 0x40, /**< FIFO batch counter threshold reached */
    KODE_LSM6DSOX_INT_DOUBLE_TAP       = 0x80, /**< Double-tap event */
    KODE_LSM6DSOX_INT_FF               = 0x100, /**< Free fall event */
    KODE_LSM6DSOX_INT_WAKE_UP          = 0x200, /**< Wake up event */
    KODE_LSM6DSOX_INT_SINGLE_TAP       = 0x400, /**< Single-tap event */
    KODE_LSM6DSOX_INT_6D               = 0x800, /**< 6D orientation change */
} kode_lsm6dsox_interrupt_type_t;

/**
 * @brief Configure general interrupt control settings
 * 
 * @param handle LSM6DSOX handle
 * @param active_low Set to 1 for active-low, 0 for active-high
 * @param open_drain Set to 1 for open-drain, 0 for push-pull
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_interrupt_pin_config(kode_lsm6dsox_handle_t *handle, 
                                          uint8_t active_low, 
                                          uint8_t open_drain);

/**
 * @brief Configure interrupts for INT1 pin
 * 
 * @param handle LSM6DSOX handle
 * @param int_types Bitwise OR of interrupt types to be enabled on INT1
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_interrupt_enable_int1(kode_lsm6dsox_handle_t *handle, 
                                           uint32_t int_types);

/**
 * @brief Configure interrupts for INT2 pin
 * 
 * @param handle LSM6DSOX handle
 * @param int_types Bitwise OR of interrupt types to be enabled on INT2
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_interrupt_enable_int2(kode_lsm6dsox_handle_t *handle, 
                                           uint32_t int_types);

/**
 * @brief Read the interrupt source to determine which interrupt occurred
 * 
 * @param handle LSM6DSOX handle
 * @param int_src Pointer to store interrupt source info
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_interrupt_source_get(kode_lsm6dsox_handle_t *handle, 
                                          lsm6dsox_all_sources_t *int_src);

/**
 * @brief Power modes for LSM6DSOX
 */
typedef enum {
    KODE_LSM6DSOX_POWER_HIGH_PERFORMANCE,    /*!< Both accel and gyro in high performance mode */
    KODE_LSM6DSOX_POWER_LOW_NORMAL,          /*!< Accel in low power mode, gyro in normal mode */
    KODE_LSM6DSOX_POWER_ULTRA_LOW,           /*!< Accel in ultra-low power mode, gyro in sleep mode */
    KODE_LSM6DSOX_POWER_SUSPEND              /*!< Both sensors in lowest power state */
} kode_lsm6dsox_power_mode_t;

/**
 * @brief Configure power mode for both accelerometer and gyroscope
 * 
 * @param handle LSM6DSOX handle
 * @param mode Power mode to set
 * @return esp_err_t ESP_OK on success, error code otherwise
 * 
 * Power consumption for different modes (typical values):
 * - HIGH_PERFORMANCE: 0.55 mA (accel) + 3.2 mA (gyro)
 * - LOW_NORMAL: 0.25 mA (accel) + 3.2 mA (gyro)
 * - ULTRA_LOW: 0.02 mA (accel) + 0.01 mA (gyro in sleep)
 * - SUSPEND: 0.02 mA (accel) + 0.01 mA (gyro in sleep), lowest ODR
 */
esp_err_t kode_lsm6dsox_set_power_mode(kode_lsm6dsox_handle_t *handle, 
                                      kode_lsm6dsox_power_mode_t mode);

/**
 * @brief Get current power mode configuration
 * 
 * @param handle LSM6DSOX handle
 * @param mode Pointer to store the current power mode
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t kode_lsm6dsox_get_power_mode(kode_lsm6dsox_handle_t *handle, 
                                      kode_lsm6dsox_power_mode_t *mode);

#ifdef __cplusplus
}
#endif

#endif /* _KODE_LSM6DSOX_H_ */