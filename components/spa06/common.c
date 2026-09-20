/**
 * @file  common.c
 * @brief SPA06-003 ESP-IDF platform bridge (standalone ESP-i2cdev path)
 *
 * Mirrors components/bmm350/common.c: wires the interface-agnostic spa06 driver
 * core to the ESP-IDF `i2cdev` component. Use spa06_esp_init() for a
 * self-contained sensor on its own I2C port/GPIOs; for the project-shared
 * I2C_1 bus use components/sensor/spa06/spa06_bsp_i2c1 instead.
 *
 * @license MIT
 */

#include <esp_log.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include <esp_timer.h>

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "spa06.h"
#include "spa06_defs.h"
#include "common.h"

static const char *TAG = "spa06";

/******************************************************************************/
/*!                User interface functions                                   */
/******************************************************************************/

/*!
 * I2C read function mapped to the ESP-IDF i2cdev platform.
 */
SPA06_INTF_RET_TYPE spa06_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                    uint32_t length, void *intf_ptr)
{
    i2c_dev_t *i2c_dev = (i2c_dev_t *)intf_ptr;
    esp_err_t ret;

    if (i2c_dev == NULL || reg_data == NULL || length == 0U) {
        return SPA06_E_COM_FAIL;
    }

    ret = i2c_dev_read_reg(i2c_dev, reg_addr, reg_data, length);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return SPA06_E_COM_FAIL;
    }

    return SPA06_INTF_RET_SUCCESS;
}

/*!
 * I2C write function mapped to the ESP-IDF i2cdev platform.
 */
SPA06_INTF_RET_TYPE spa06_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                     uint32_t length, void *intf_ptr)
{
    i2c_dev_t *i2c_dev = (i2c_dev_t *)intf_ptr;
    esp_err_t ret;

    if (i2c_dev == NULL || reg_data == NULL || length == 0U) {
        return SPA06_E_COM_FAIL;
    }

    ret = i2c_dev_write_reg(i2c_dev, reg_addr, reg_data, length);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
        return SPA06_E_COM_FAIL;
    }

    return SPA06_INTF_RET_SUCCESS;
}

/*!
 * @brief Microsecond delay wrapper matching the callback signature.
 *
 * Uses a busy-loop via esp_rom_delay_us for sub-millisecond waits and
 * vTaskDelay otherwise, so callers can pass large values safely.
 */
void spa06_delay(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;

    if (period_us < 1000U) {
        esp_rom_delay_us(period_us);
        return;
    }

    uint32_t period_ms = period_us / 1000U;
    if (period_ms == 0U) {
        period_ms = 1U;
    }
    vTaskDelay(pdMS_TO_TICKS(period_ms));
}

/*!
 *  @brief Print the execution status of the APIs.
 */
void spa06_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt) {
    case SPA06_OK:
        break;
    case SPA06_E_NULL_PTR:
        ESP_LOGE(TAG, "%s Error [%d] : Null pointer", api_name, rslt);
        break;
    case SPA06_E_COM_FAIL:
        ESP_LOGE(TAG, "%s Error [%d] : Communication fail", api_name, rslt);
        break;
    case SPA06_E_DEV_NOT_FOUND:
        ESP_LOGE(TAG, "%s Error [%d] : Device not found", api_name, rslt);
        break;
    case SPA06_E_INVALID_CONFIG:
        ESP_LOGE(TAG, "%s Error [%d] : Invalid configuration", api_name, rslt);
        break;
    case SPA06_E_COEF_NOT_READY:
        ESP_LOGE(TAG, "%s Error [%d] : Coefficients not ready", api_name, rslt);
        break;
    case SPA06_E_TIMEOUT:
        ESP_LOGE(TAG, "%s Error [%d] : Timeout", api_name, rslt);
        break;
    case SPA06_E_INVALID_INPUT:
        ESP_LOGE(TAG, "%s Error [%d] : Invalid input", api_name, rslt);
        break;
    default:
        ESP_LOGE(TAG, "%s Error [%d] : Unknown error code", api_name, rslt);
        break;
    }
}

/*****************************************************************************/
/*!                ESP-IDF Platform Specific Functions                       */
/*****************************************************************************/

esp_err_t spa06_i2cdev_init(i2c_dev_t *i2c_dev, i2c_port_t port,
                             gpio_num_t sda_gpio, gpio_num_t scl_gpio,
                             uint8_t addr)
{
    if (i2c_dev == NULL) {
        ESP_LOGE(TAG, "i2c_dev pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    i2c_dev->port = port;
    i2c_dev->addr = (addr == 0U) ? SPA06_I2C_ADDR_DEFAULT : addr;
    i2c_dev->cfg.sda_io_num = sda_gpio;
    i2c_dev->cfg.scl_io_num = scl_gpio;
    i2c_dev->cfg.sda_pullup_en = true;
    i2c_dev->cfg.scl_pullup_en = true;

#if HELPER_TARGET_IS_ESP32
    i2c_dev->cfg.master.clk_speed = SPA06_I2C_FREQ_HZ;
#endif

    esp_err_t ret = i2c_dev_create_mutex(i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C device mutex: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPA06 I2C device initialized on port %d, SDA=%d, SCL=%d, addr=0x%02x",
             port, sda_gpio, scl_gpio, i2c_dev->addr);

    return ESP_OK;
}

esp_err_t spa06_i2cdev_free(i2c_dev_t *i2c_dev)
{
    if (i2c_dev == NULL) {
        ESP_LOGE(TAG, "i2c_dev pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_dev_delete_mutex(i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete I2C device mutex: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPA06 I2C device freed");
    return ESP_OK;
}

int8_t spa06_esp_init(struct spa06_dev *dev, i2c_port_t port,
                       gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint8_t addr)
{
    if (dev == NULL) {
        ESP_LOGE(TAG, "dev pointer is NULL");
        return SPA06_E_NULL_PTR;
    }

    i2c_dev_t *i2c_dev = (i2c_dev_t *)malloc(sizeof(i2c_dev_t));
    if (i2c_dev == NULL) {
        ESP_LOGE(TAG, "Failed to allocate i2c_dev_t");
        return SPA06_E_NULL_PTR;
    }

    esp_err_t ret = spa06_i2cdev_init(i2c_dev, port, sda_gpio, scl_gpio, addr);
    if (ret != ESP_OK) {
        free(i2c_dev);
        return SPA06_E_COM_FAIL;
    }

    dev->intf_ptr = i2c_dev;
    dev->dev_addr = i2c_dev->addr;
    dev->read = spa06_i2c_read;
    dev->write = spa06_i2c_write;
    dev->delay_us = spa06_delay;

    ESP_LOGI(TAG, "SPA06 device initialized with ESP-IDF I2C backend");
    return SPA06_OK;
}

int8_t spa06_esp_deinit(struct spa06_dev *dev)
{
    if (dev == NULL) {
        ESP_LOGE(TAG, "dev pointer is NULL");
        return SPA06_E_NULL_PTR;
    }

    if (dev->intf_ptr != NULL) {
        i2c_dev_t *i2c_dev = (i2c_dev_t *)dev->intf_ptr;
        esp_err_t ret = spa06_i2cdev_free(i2c_dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to free I2C device: %s", esp_err_to_name(ret));
            return SPA06_E_COM_FAIL;
        }
        free(i2c_dev);
        dev->intf_ptr = NULL;
    }

    ESP_LOGI(TAG, "SPA06 device deinitialized");
    return SPA06_OK;
}
