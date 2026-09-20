#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"


#ifdef __cplusplus
extern "C" {
#endif

#define SHT4X_BSP_I2C1_ADDRESS_DEFAULT   (0x44U)
#define SHT4X_BSP_I2C1_CLK_HZ            (100000U)
#define SHT4X_BSP_I2C1_RAW_DATA_SIZE     (6U)

typedef uint8_t sht4x_bsp_i2c1_raw_data_t[SHT4X_BSP_I2C1_RAW_DATA_SIZE];

typedef enum {
	SHT4X_BSP_I2C1_HEATER_OFF = 0,
	SHT4X_BSP_I2C1_HEATER_HIGH_LONG,
	SHT4X_BSP_I2C1_HEATER_HIGH_SHORT,
	SHT4X_BSP_I2C1_HEATER_MEDIUM_LONG,
	SHT4X_BSP_I2C1_HEATER_MEDIUM_SHORT,
	SHT4X_BSP_I2C1_HEATER_LOW_LONG,
	SHT4X_BSP_I2C1_HEATER_LOW_SHORT,
} sht4x_bsp_i2c1_heater_t;

typedef enum {
	SHT4X_BSP_I2C1_HIGH = 0,
	SHT4X_BSP_I2C1_MEDIUM,
	SHT4X_BSP_I2C1_LOW,
} sht4x_bsp_i2c1_repeat_t;

typedef struct {
	void *dev_handle;
	uint8_t i2c_address;
} sht4x_bsp_i2c1_context_t;

typedef struct {
	sht4x_bsp_i2c1_context_t *context;
	uint32_t serial;
	sht4x_bsp_i2c1_heater_t heater;
	sht4x_bsp_i2c1_repeat_t repeatability;
	bool meas_started;
	uint64_t meas_start_time;
	uint8_t i2c_address;
} sht4x_bsp_i2c1_t;

esp_err_t sht4x_bsp_i2c1_init(sht4x_bsp_i2c1_t *dev, sht4x_bsp_i2c1_context_t *context);
esp_err_t sht4x_bsp_i2c1_deinit(sht4x_bsp_i2c1_t *dev, sht4x_bsp_i2c1_context_t *context);
esp_err_t sht4x_bsp_i2c1_reset(sht4x_bsp_i2c1_t *dev);
esp_err_t sht4x_bsp_i2c1_measure(sht4x_bsp_i2c1_t *dev, float *temperature, float *humidity);
esp_err_t sht4x_bsp_i2c1_start_measurement(sht4x_bsp_i2c1_t *dev);
size_t sht4x_bsp_i2c1_get_measurement_duration(const sht4x_bsp_i2c1_t *dev);
esp_err_t sht4x_bsp_i2c1_get_raw_data(sht4x_bsp_i2c1_t *dev, sht4x_bsp_i2c1_raw_data_t raw_data);
esp_err_t sht4x_bsp_i2c1_compute_values(const sht4x_bsp_i2c1_raw_data_t raw_data, float *temperature, float *humidity);
esp_err_t sht4x_bsp_i2c1_get_results(sht4x_bsp_i2c1_t *dev, float *temperature, float *humidity);
esp_err_t sht4x_bsp_i2c1_get_serial(const sht4x_bsp_i2c1_t *dev, uint32_t *serial);

#ifdef __cplusplus
}
#endif