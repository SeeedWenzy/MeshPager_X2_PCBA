#include <string.h>

#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp_i2c_control.h"

#include "sht4x_bsp_i2c1.h"

#define SHT4X_CMD_RESET             (0x94U)
#define SHT4X_CMD_SERIAL            (0x89U)
#define SHT4X_CMD_MEAS_HIGH         (0xFDU)
#define SHT4X_CMD_MEAS_MED          (0xF6U)
#define SHT4X_CMD_MEAS_LOW          (0xE0U)
#define SHT4X_CMD_MEAS_H_HIGH_LONG  (0x39U)
#define SHT4X_CMD_MEAS_H_HIGH_SHORT (0x32U)
#define SHT4X_CMD_MEAS_H_MED_LONG   (0x2FU)
#define SHT4X_CMD_MEAS_H_MED_SHORT  (0x24U)
#define SHT4X_CMD_MEAS_H_LOW_LONG   (0x1EU)
#define SHT4X_CMD_MEAS_H_LOW_SHORT  (0x15U)

#define SHT4X_G_POLYNOM             (0x31U)
#define SHT4X_TIME_TO_TICKS(ms)     (1U + (((ms) + (portTICK_PERIOD_MS - 1U) + (portTICK_PERIOD_MS / 2U)) / portTICK_PERIOD_MS))

static uint8_t sht4x_bsp_i2c1_crc8(const uint8_t *data, size_t length)
{
	uint8_t crc = 0xFFU;

	for (size_t data_index = 0; data_index < length; data_index++) {
		crc ^= data[data_index];
		for (size_t bit_index = 0; bit_index < 8U; bit_index++) {
			crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ SHT4X_G_POLYNOM) : (uint8_t)(crc << 1U);
		}
	}

	return crc;
}

static esp_err_t sht4x_bsp_i2c1_validate(const sht4x_bsp_i2c1_t *dev)
{
	if (dev == NULL || dev->context == NULL || dev->context->dev_handle == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	return ESP_OK;
}

static size_t sht4x_bsp_i2c1_get_duration_ms(const sht4x_bsp_i2c1_t *dev)
{
	switch (dev->heater) {
	case SHT4X_BSP_I2C1_HEATER_HIGH_LONG:
	case SHT4X_BSP_I2C1_HEATER_MEDIUM_LONG:
	case SHT4X_BSP_I2C1_HEATER_LOW_LONG:
		return 1100U;
	case SHT4X_BSP_I2C1_HEATER_HIGH_SHORT:
	case SHT4X_BSP_I2C1_HEATER_MEDIUM_SHORT:
	case SHT4X_BSP_I2C1_HEATER_LOW_SHORT:
		return 110U;
	default:
		switch (dev->repeatability) {
		case SHT4X_BSP_I2C1_HIGH:
			return 10U;
		case SHT4X_BSP_I2C1_MEDIUM:
			return 5U;
		default:
			return 2U;
		}
	}
}

static uint8_t sht4x_bsp_i2c1_get_meas_cmd(const sht4x_bsp_i2c1_t *dev)
{
	switch (dev->heater) {
	case SHT4X_BSP_I2C1_HEATER_HIGH_LONG:
		return SHT4X_CMD_MEAS_H_HIGH_LONG;
	case SHT4X_BSP_I2C1_HEATER_HIGH_SHORT:
		return SHT4X_CMD_MEAS_H_HIGH_SHORT;
	case SHT4X_BSP_I2C1_HEATER_MEDIUM_LONG:
		return SHT4X_CMD_MEAS_H_MED_LONG;
	case SHT4X_BSP_I2C1_HEATER_MEDIUM_SHORT:
		return SHT4X_CMD_MEAS_H_MED_SHORT;
	case SHT4X_BSP_I2C1_HEATER_LOW_LONG:
		return SHT4X_CMD_MEAS_H_LOW_LONG;
	case SHT4X_BSP_I2C1_HEATER_LOW_SHORT:
		return SHT4X_CMD_MEAS_H_LOW_SHORT;
	default:
		switch (dev->repeatability) {
		case SHT4X_BSP_I2C1_HIGH:
			return SHT4X_CMD_MEAS_HIGH;
		case SHT4X_BSP_I2C1_MEDIUM:
			return SHT4X_CMD_MEAS_MED;
		default:
			return SHT4X_CMD_MEAS_LOW;
		}
	}
}

static bool sht4x_bsp_i2c1_is_measuring(const sht4x_bsp_i2c1_t *dev)
{
	uint64_t elapsed;

	if (!dev->meas_started) {
		return false;
	}

	elapsed = esp_timer_get_time() - dev->meas_start_time;
	return elapsed < (uint64_t)sht4x_bsp_i2c1_get_duration_ms(dev) * 1000ULL;
}

static esp_err_t sht4x_bsp_i2c1_send_cmd(const sht4x_bsp_i2c1_t *dev, uint8_t cmd)
{
	return i2c_master_transmit((i2c_master_dev_handle_t)dev->context->dev_handle, &cmd, 1, -1);
}

static esp_err_t sht4x_bsp_i2c1_read_raw(const sht4x_bsp_i2c1_t *dev, sht4x_bsp_i2c1_raw_data_t raw_data)
{
	esp_err_t err;

	err = i2c_master_receive((i2c_master_dev_handle_t)dev->context->dev_handle,
		raw_data,
		SHT4X_BSP_I2C1_RAW_DATA_SIZE,
		-1);
	if (err != ESP_OK) {
		return err;
	}

	if (raw_data[2] != sht4x_bsp_i2c1_crc8(raw_data, 2U) || raw_data[5] != sht4x_bsp_i2c1_crc8(&raw_data[3], 2U)) {
		return ESP_ERR_INVALID_CRC;
	}

	return ESP_OK;
}

static esp_err_t sht4x_bsp_i2c1_exec_cmd(const sht4x_bsp_i2c1_t *dev,
	uint8_t cmd,
	size_t delay_ticks,
	sht4x_bsp_i2c1_raw_data_t raw_data)
{
	esp_err_t err;

	err = sht4x_bsp_i2c1_send_cmd(dev, cmd);
	if (err != ESP_OK) {
		return err;
	}

	vTaskDelay((TickType_t)delay_ticks + 1);
	return sht4x_bsp_i2c1_read_raw(dev, raw_data);
}

esp_err_t sht4x_bsp_i2c1_init(sht4x_bsp_i2c1_t *dev, sht4x_bsp_i2c1_context_t *context)
{
	sht4x_bsp_i2c1_raw_data_t serial_raw = { 0 };
	i2c_device_config_t dev_cfg;
	i2c_master_bus_handle_t bus_handle;
	esp_err_t err;

	if (dev == NULL || context == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	err = bsp_i2c_1_init();
	if (err != ESP_OK) {
		return err;
	}

	if (context->dev_handle != NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	bus_handle = bsp_i2c_1_get_handle();
	if (bus_handle == NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	if (context->i2c_address == 0U) {
		context->i2c_address = SHT4X_BSP_I2C1_ADDRESS_DEFAULT;
	}

	dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	dev_cfg.device_address = context->i2c_address;
	dev_cfg.scl_speed_hz = SHT4X_BSP_I2C1_CLK_HZ;
	dev_cfg.scl_wait_us = 0;

	err = i2c_master_bus_add_device(bus_handle, &dev_cfg, (i2c_master_dev_handle_t *)&context->dev_handle);
	if (err != ESP_OK) {
		return err;
	}

	memset(dev, 0, sizeof(*dev));
	dev->context = context;
	dev->i2c_address = context->i2c_address;
	dev->repeatability = SHT4X_BSP_I2C1_HIGH;
	dev->heater = SHT4X_BSP_I2C1_HEATER_OFF;

	err = sht4x_bsp_i2c1_exec_cmd(dev, SHT4X_CMD_SERIAL, SHT4X_TIME_TO_TICKS(10U), serial_raw);
	if (err != ESP_OK) {
		i2c_master_bus_rm_device((i2c_master_dev_handle_t)context->dev_handle);
		context->dev_handle = NULL;
		dev->context = NULL;
		return err;
	}

	dev->serial = ((uint32_t)serial_raw[0] << 24) |
		((uint32_t)serial_raw[1] << 16) |
		((uint32_t)serial_raw[3] << 8) |
		(uint32_t)serial_raw[4];

	err = sht4x_bsp_i2c1_reset(dev);
	if (err != ESP_OK) {
		i2c_master_bus_rm_device((i2c_master_dev_handle_t)context->dev_handle);
		context->dev_handle = NULL;
		dev->context = NULL;
		return err;
	}

	return ESP_OK;
}

esp_err_t sht4x_bsp_i2c1_deinit(sht4x_bsp_i2c1_t *dev, sht4x_bsp_i2c1_context_t *context)
{
	esp_err_t err;

	if (dev == NULL || context == NULL || context->dev_handle == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	err = i2c_master_bus_rm_device((i2c_master_dev_handle_t)context->dev_handle);
	if (err != ESP_OK) {
		return err;
	}

	context->dev_handle = NULL;
	if (dev->context == context) {
		memset(dev, 0, sizeof(*dev));
	}

	return ESP_OK;
}

esp_err_t sht4x_bsp_i2c1_reset(sht4x_bsp_i2c1_t *dev)
{
	esp_err_t err = sht4x_bsp_i2c1_validate(dev);

	if (err != ESP_OK) {
		return err;
	}

	dev->meas_start_time = 0;
	dev->meas_started = false;

	err = sht4x_bsp_i2c1_send_cmd(dev, SHT4X_CMD_RESET);
	if (err != ESP_OK) {
		return err;
	}

	vTaskDelay(1);
	return ESP_OK;
}

esp_err_t sht4x_bsp_i2c1_measure(sht4x_bsp_i2c1_t *dev, float *temperature, float *humidity)
{
	sht4x_bsp_i2c1_raw_data_t raw_data = { 0 };
	esp_err_t err = sht4x_bsp_i2c1_validate(dev);

	if (err != ESP_OK || (temperature == NULL && humidity == NULL)) {
		return (err == ESP_OK) ? ESP_ERR_INVALID_ARG : err;
	}

	err = sht4x_bsp_i2c1_exec_cmd(dev,
		sht4x_bsp_i2c1_get_meas_cmd(dev),
		sht4x_bsp_i2c1_get_measurement_duration(dev),
		raw_data);
	if (err != ESP_OK) {
		return err;
	}

	return sht4x_bsp_i2c1_compute_values(raw_data, temperature, humidity);
}

esp_err_t sht4x_bsp_i2c1_start_measurement(sht4x_bsp_i2c1_t *dev)
{
	esp_err_t err = sht4x_bsp_i2c1_validate(dev);

	if (err != ESP_OK) {
		return err;
	}

	if (sht4x_bsp_i2c1_is_measuring(dev)) {
		return ESP_ERR_INVALID_STATE;
	}

	err = sht4x_bsp_i2c1_send_cmd(dev, sht4x_bsp_i2c1_get_meas_cmd(dev));
	if (err != ESP_OK) {
		return err;
	}

	dev->meas_start_time = esp_timer_get_time();
	dev->meas_started = true;
	return ESP_OK;
}

size_t sht4x_bsp_i2c1_get_measurement_duration(const sht4x_bsp_i2c1_t *dev)
{
	size_t duration_ticks;

	if (dev == NULL) {
		return 0U;
	}

	duration_ticks = SHT4X_TIME_TO_TICKS(sht4x_bsp_i2c1_get_duration_ms(dev));
	return (duration_ticks == 0U) ? 1U : duration_ticks;
}

esp_err_t sht4x_bsp_i2c1_get_raw_data(sht4x_bsp_i2c1_t *dev, sht4x_bsp_i2c1_raw_data_t raw_data)
{
	esp_err_t err = sht4x_bsp_i2c1_validate(dev);

	if (err != ESP_OK) {
		return err;
	}

	if (raw_data == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	if (sht4x_bsp_i2c1_is_measuring(dev)) {
		return ESP_ERR_INVALID_STATE;
	}

	err = sht4x_bsp_i2c1_read_raw(dev, raw_data);
	if (err != ESP_OK) {
		return err;
	}

	dev->meas_started = false;
	return ESP_OK;
}

esp_err_t sht4x_bsp_i2c1_compute_values(const sht4x_bsp_i2c1_raw_data_t raw_data, float *temperature, float *humidity)
{
	if (raw_data == NULL || (temperature == NULL && humidity == NULL)) {
		return ESP_ERR_INVALID_ARG;
	}

	if (temperature != NULL) {
		*temperature = (float)(((uint16_t)raw_data[0] << 8) | raw_data[1]) * 175.0f / 65535.0f - 45.0f;
	}

	if (humidity != NULL) {
		*humidity = (float)(((uint16_t)raw_data[3] << 8) | raw_data[4]) * 125.0f / 65535.0f - 6.0f;
	}

	return ESP_OK;
}

esp_err_t sht4x_bsp_i2c1_get_results(sht4x_bsp_i2c1_t *dev, float *temperature, float *humidity)
{
	sht4x_bsp_i2c1_raw_data_t raw_data = { 0 };
	esp_err_t err;

	if (temperature == NULL && humidity == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	err = sht4x_bsp_i2c1_get_raw_data(dev, raw_data);
	if (err != ESP_OK) {
		return err;
	}

	return sht4x_bsp_i2c1_compute_values(raw_data, temperature, humidity);
}

esp_err_t sht4x_bsp_i2c1_get_serial(const sht4x_bsp_i2c1_t *dev, uint32_t *serial)
{
	esp_err_t err = sht4x_bsp_i2c1_validate(dev);

	if (err != ESP_OK || serial == NULL) {
		return (err == ESP_OK) ? ESP_ERR_INVALID_ARG : err;
	}

	*serial = dev->serial;
	return ESP_OK;
}