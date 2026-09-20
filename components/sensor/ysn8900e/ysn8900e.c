#include <stdlib.h>

#include "driver/i2c_master.h"
#include "bsp_i2c_control.h"

#include "ysn8900e.h"

static esp_err_t ysn8900e_bsp_i2c0_read(void *context, uint8_t reg, uint8_t *buffer, size_t length)
{
	ysn8900e_bsp_i2c0_context_t *i2c_context = (ysn8900e_bsp_i2c0_context_t *)context;

	if (i2c_context == NULL || i2c_context->dev_handle == NULL || buffer == NULL || length == 0U) {
		return ESP_ERR_INVALID_ARG;
	}

	return i2c_master_transmit_receive((i2c_master_dev_handle_t)i2c_context->dev_handle,
		&reg,
		1,
		buffer,
		length,
		-1);
}

static esp_err_t ysn8900e_bsp_i2c0_write(void *context, uint8_t reg, const uint8_t *buffer, size_t length)
{
	ysn8900e_bsp_i2c0_context_t *i2c_context = (ysn8900e_bsp_i2c0_context_t *)context;
	uint8_t *write_buffer = NULL;
	esp_err_t err;

	if (i2c_context == NULL || i2c_context->dev_handle == NULL || buffer == NULL || length == 0U) {
		return ESP_ERR_INVALID_ARG;
	}

	write_buffer = (uint8_t *)malloc(length + 1U);
	if (write_buffer == NULL) {
		return ESP_ERR_NO_MEM;
	}

	write_buffer[0] = reg;
	for (size_t index = 0; index < length; index++) {
		write_buffer[index + 1U] = buffer[index];
	}

	err = i2c_master_transmit((i2c_master_dev_handle_t)i2c_context->dev_handle,
		write_buffer,
		length + 1U,
		-1);
	free(write_buffer);

	return err;
}

static esp_err_t ysn8900e_validate(const ysn8900e_t *dev)
{
	if (dev == NULL || dev->read == NULL || dev->write == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	return ESP_OK;
}

static esp_err_t ysn8900e_read_reg(const ysn8900e_t *dev, uint8_t reg, uint8_t *buffer, size_t length)
{
	esp_err_t err = ysn8900e_validate(dev);

	if (err != ESP_OK || buffer == NULL || length == 0U) {
		return (err == ESP_OK) ? ESP_ERR_INVALID_ARG : err;
	}

	return dev->read(dev->context, reg, buffer, length);
}

static esp_err_t ysn8900e_write_reg(const ysn8900e_t *dev, uint8_t reg, const uint8_t *buffer, size_t length)
{
	esp_err_t err = ysn8900e_validate(dev);

	if (err != ESP_OK || buffer == NULL || length == 0U) {
		return (err == ESP_OK) ? ESP_ERR_INVALID_ARG : err;
	}

	return dev->write(dev->context, reg, buffer, length);
}

static esp_err_t ysn8900e_set_reg_bit(const ysn8900e_t *dev, uint8_t reg, uint8_t bit, bool value)
{
	uint8_t reg_value = 0;
	esp_err_t err = ysn8900e_read_reg(dev, reg, &reg_value, 1);

	if (err != ESP_OK) {
		return err;
	}

	if (value) {
		reg_value |= (uint8_t)(1U << bit);
	} else {
		reg_value &= (uint8_t)~(1U << bit);
	}

	return ysn8900e_write_reg(dev, reg, &reg_value, 1);
}

static uint8_t ysn8900e_bcd_to_dec(uint8_t value)
{
	return (uint8_t)(((value / 16U) * 10U) + (value % 16U));
}

static uint8_t ysn8900e_dec_to_bcd(uint8_t value)
{
	return (uint8_t)(((value / 10U) * 16U) + (value % 10U));
}

esp_err_t ysn8900e_init_bsp_i2c0(ysn8900e_t *dev, ysn8900e_bsp_i2c0_context_t *context)
{
	i2c_device_config_t dev_cfg;
	i2c_master_bus_handle_t bus_handle;
	esp_err_t err;

	if (dev == NULL || context == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	err = bsp_i2c_0_init();
	if (err != ESP_OK) {
		return err;
	}

	if (context->dev_handle != NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	bus_handle = bsp_i2c_0_get_handle();
	if (bus_handle == NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	if (dev->i2c_address == 0U) {
		dev->i2c_address = YSN8900E_I2C_ADDRESS;
	}

	dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	dev_cfg.device_address = dev->i2c_address;
	dev_cfg.scl_speed_hz = YSN8900E_I2C_CLK_HZ;
	dev_cfg.scl_wait_us = 0;

	err = i2c_master_bus_add_device(bus_handle, &dev_cfg, (i2c_master_dev_handle_t *)&context->dev_handle);
	if (err != ESP_OK) {
		return err;
	}

	dev->context = context;
	dev->read = ysn8900e_bsp_i2c0_read;
	dev->write = ysn8900e_bsp_i2c0_write;

	return ESP_OK;
}

esp_err_t ysn8900e_deinit_bsp_i2c0(ysn8900e_t *dev, ysn8900e_bsp_i2c0_context_t *context)
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
		dev->context = NULL;
		dev->read = NULL;
		dev->write = NULL;
	}

	return ESP_OK;
}

esp_err_t ysn8900e_init(ysn8900e_t *dev)
{
	uint8_t extension_val = 0x01;
	uint8_t control_val = 0x50;
	uint8_t timer_cnt = YSN8900E_TIMER_DEFAULT_CNT;
	esp_err_t err = ysn8900e_validate(dev);

	if (err != ESP_OK) {
		return err;
	}

	if (dev->i2c_address == 0U) {
		dev->i2c_address = YSN8900E_I2C_ADDRESS;
	}

	err = ysn8900e_write_reg(dev, YSN8900E_EXTENSION_REG, &extension_val, 1);
	if (err != ESP_OK) {
		return err;
	}

	err = ysn8900e_write_reg(dev, YSN8900E_CONTROL_REG, &control_val, 1);
	if (err != ESP_OK) {
		return err;
	}

	err = ysn8900e_write_reg(dev, YSN8900E_TIMER_COUNTER_0, &timer_cnt, 1);
	if (err != ESP_OK) {
		return err;
	}

	// return ysn8900e_disable_timer(dev);
	return ysn8900e_enable_timer(dev);
	
}

esp_err_t ysn8900e_set_datetime(ysn8900e_t *dev, const ysn8900e_datetime_t *datetime)
{
	uint8_t buffer[7];

	if (datetime == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	buffer[0] = ysn8900e_dec_to_bcd(datetime->second) & 0x7F;
	buffer[1] = ysn8900e_dec_to_bcd(datetime->minute);
	buffer[2] = ysn8900e_dec_to_bcd(datetime->hour);
	buffer[3] = ysn8900e_dec_to_bcd(datetime->week);
	buffer[4] = ysn8900e_dec_to_bcd(datetime->day);
	buffer[5] = ysn8900e_dec_to_bcd(datetime->month);
	buffer[6] = ysn8900e_dec_to_bcd((uint8_t)(datetime->year % 100U));

	return ysn8900e_write_reg(dev, YSN8900E_SEC_REG, buffer, sizeof(buffer));
}

esp_err_t ysn8900e_get_datetime(ysn8900e_t *dev, ysn8900e_datetime_t *datetime)
{
	uint8_t buffer[7] = { 0 };
	esp_err_t err;

	if (datetime == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	err = ysn8900e_read_reg(dev, YSN8900E_SEC_REG, buffer, sizeof(buffer));
	if (err != ESP_OK) {
		return err;
	}

	datetime->available = (buffer[0] & 0x80U) == 0U;
	datetime->second = ysn8900e_bcd_to_dec(buffer[0] & 0x7FU);
	datetime->minute = ysn8900e_bcd_to_dec(buffer[1] & 0x7FU);
	datetime->hour = ysn8900e_bcd_to_dec(buffer[2] & 0x3FU);
	datetime->week = ysn8900e_bcd_to_dec(buffer[3] & 0x07U);
	datetime->day = ysn8900e_bcd_to_dec(buffer[4] & 0x3FU);
	datetime->month = ysn8900e_bcd_to_dec(buffer[5] & 0x1FU);
	datetime->year = (uint16_t)(2000U + ysn8900e_bcd_to_dec(buffer[6]));

	return ESP_OK;
}

esp_err_t ysn8900e_enable_timer(ysn8900e_t *dev)
{
	return ysn8900e_set_reg_bit(dev, YSN8900E_EXTENSION_REG, 4, true);
}

esp_err_t ysn8900e_disable_timer(ysn8900e_t *dev)
{
	return ysn8900e_set_reg_bit(dev, YSN8900E_EXTENSION_REG, 4, false);
}

esp_err_t ysn8900e_reset_timer(ysn8900e_t *dev)
{
	uint8_t timer_cnt = YSN8900E_TIMER_DEFAULT_CNT;
	esp_err_t err = ysn8900e_disable_timer(dev);

	if (err != ESP_OK) {
		return err;
	}

	err = ysn8900e_write_reg(dev, YSN8900E_TIMER_COUNTER_0, &timer_cnt, 1);
	if (err != ESP_OK) {
		return err;
	}

	return ysn8900e_enable_timer(dev);
}

esp_err_t ysn8900e_reload_timer(ysn8900e_t *dev)
{
	return ysn8900e_set_reg_bit(dev, YSN8900E_FLAG_REG, 4, false);
}

esp_err_t ysn8900e_get_device_id(ysn8900e_t *dev, uint8_t *device_id)
{
	if (device_id == NULL) {
		return ESP_ERR_INVALID_ARG;
	}

	return ysn8900e_read_reg(dev, YSN8900E_DEVICE_ID_REG, device_id, 1);
}
