#include <stdlib.h>

#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "bsp_i2c_control.h"

#include "spa06_bsp_i2c1.h"
#include "spa06_defs.h"

static SPA06_INTF_RET_TYPE spa06_bsp_i2c1_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	spa06_bsp_i2c1_context_t *context = (spa06_bsp_i2c1_context_t *)intf_ptr;
	esp_err_t err;

	if (context == NULL || context->dev_handle == NULL || reg_data == NULL || length == 0U) {
		return SPA06_E_COM_FAIL;
	}

	err = i2c_master_transmit_receive((i2c_master_dev_handle_t)context->dev_handle,
		&reg_addr,
		1,
		reg_data,
		length,
		-1);

	return (err == ESP_OK) ? SPA06_INTF_RET_SUCCESS : SPA06_E_COM_FAIL;
}

static SPA06_INTF_RET_TYPE spa06_bsp_i2c1_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	spa06_bsp_i2c1_context_t *context = (spa06_bsp_i2c1_context_t *)intf_ptr;
	uint8_t *write_buffer = NULL;
	esp_err_t err;

	if (context == NULL || context->dev_handle == NULL || reg_data == NULL || length == 0U) {
		return SPA06_E_COM_FAIL;
	}

	write_buffer = (uint8_t *)malloc((size_t)length + 1U);
	if (write_buffer == NULL) {
		return SPA06_E_COM_FAIL;
	}

	write_buffer[0] = reg_addr;
	for (uint32_t index = 0; index < length; index++) {
		write_buffer[index + 1U] = reg_data[index];
	}

	err = i2c_master_transmit((i2c_master_dev_handle_t)context->dev_handle,
		write_buffer,
		(size_t)length + 1U,
		-1);
	free(write_buffer);

	return (err == ESP_OK) ? SPA06_INTF_RET_SUCCESS : SPA06_E_COM_FAIL;
}

static void spa06_bsp_i2c1_delay_us(uint32_t period, void *intf_ptr)
{
	(void)intf_ptr;
	esp_rom_delay_us(period);
}

esp_err_t spa06_init_bsp_i2c1(struct spa06_dev *dev, spa06_bsp_i2c1_context_t *context)
{
	i2c_device_config_t dev_cfg;
	i2c_master_bus_handle_t bus_handle;
	uint8_t i2c_address;
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

	i2c_address = (context->i2c_address == 0U) ? SPA06_BSP_I2C1_ADDRESS_DEFAULT : context->i2c_address;

	dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	dev_cfg.device_address = i2c_address;
	dev_cfg.scl_speed_hz = SPA06_BSP_I2C1_CLK_HZ;
	dev_cfg.scl_wait_us = 0;

	err = i2c_master_bus_add_device(bus_handle, &dev_cfg, (i2c_master_dev_handle_t *)&context->dev_handle);
	if (err != ESP_OK) {
		return err;
	}

	context->i2c_address = i2c_address;
	dev->intf_ptr = context;
	dev->dev_addr = i2c_address;
	dev->read = spa06_bsp_i2c1_read;
	dev->write = spa06_bsp_i2c1_write;
	dev->delay_us = spa06_bsp_i2c1_delay_us;

	return ESP_OK;
}

esp_err_t spa06_deinit_bsp_i2c1(struct spa06_dev *dev, spa06_bsp_i2c1_context_t *context)
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
	if (dev->intf_ptr == context) {
		dev->intf_ptr = NULL;
		dev->read = NULL;
		dev->write = NULL;
		dev->delay_us = NULL;
	}

	return ESP_OK;
}
