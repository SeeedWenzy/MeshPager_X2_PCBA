/**
* Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file  common.c
*
*/

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include <esp_timer.h>

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>


#include "bmm350.h"
#include "common.h"

#define I2C_FREQ_HZ 1000000 // 1MHz

// due to the fact that ticks can be smaller than portTICK_PERIOD_MS, one and
// a half tick period added to the duration to be sure that waiting time for
// the results is long enough
#define TIME_TO_TICKS(ms) (1 + ((ms) + (portTICK_PERIOD_MS-1) + portTICK_PERIOD_MS/2 ) / portTICK_PERIOD_MS)


static const char *TAG = "bmm350";

/******************************************************************************/
/*!                Structure definition                                       */

#define BMM350_SHUTTLE_ID  UINT16_C(0x27)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to ESP-IDF i2cdev platform
 */
BMM350_INTF_RET_TYPE bmm350_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    i2c_dev_t *i2c_dev = (i2c_dev_t *)intf_ptr;

    ESP_LOGD(TAG, "I2C read: reg=0x%02x, len=%lu", reg_addr, length);

    esp_err_t ret = i2c_dev_read_reg(i2c_dev, reg_addr, reg_data, length);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return BMM350_E_COM_FAIL;
    }

    return BMM350_INTF_RET_SUCCESS;
}

/*!
 * I2C write function map to ESP-IDF i2cdev platform
 */
BMM350_INTF_RET_TYPE bmm350_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    i2c_dev_t *i2c_dev = (i2c_dev_t *)intf_ptr;

    ESP_LOGD(TAG, "I2C write: reg=0x%02x, len=%lu", reg_addr, length);

    esp_err_t ret = i2c_dev_write_reg(i2c_dev, reg_addr, reg_data, length);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
        return BMM350_E_COM_FAIL;
    }

    return BMM350_INTF_RET_SUCCESS;
}

/*!
 * @brief Delay function wrapper that matches the function pointer signature
 * @param period_us Delay period in microseconds
 * @param intf_ptr Interface pointer (unused)
 */
static void bmm350_delay_us_wrapper(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    
    // Convert microseconds to milliseconds for vTaskDelay
    // vTaskDelay requires ticks, so convert period_us to ms first
    uint32_t period_ms = period_us / 1000;
    
    // If there are remaining microseconds, add at least 1 ms
    if (period_us % 1000 > 0 && period_ms == 0) {
        period_ms = 1;
    }
    
    vTaskDelay(pdMS_TO_TICKS(period_ms));
}

/*!
 *  @brief Prints the execution status of the APIs.
 *  @param[out] api_name    : API which triggered the error.
 *  @param[in] rslt     : Error trriggered.
 */
void bmm350_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMM350_OK:
            break;

        case BMM350_E_NULL_PTR:
            ESP_LOGE(TAG, "%s Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMM350_E_COM_FAIL:
            ESP_LOGE(TAG, "%s Error [%d] : Communication fail\r\n", api_name, rslt);
            break;
        case BMM350_E_DEV_NOT_FOUND:
            ESP_LOGE(TAG, "%s Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMM350_E_INVALID_CONFIG:
            ESP_LOGE(TAG, "%s Error [%d] : Invalid configuration\r\n", api_name, rslt);
            break;
        case BMM350_E_BAD_PAD_DRIVE:
            ESP_LOGE(TAG, "%s Error [%d] : Bad pad drive\r\n", api_name, rslt);
            break;
        case BMM350_E_RESET_UNFINISHED:
            ESP_LOGE(TAG, "%s Error [%d] : Reset unfinished\r\n", api_name, rslt);
            break;
        case BMM350_E_INVALID_INPUT:
            ESP_LOGE(TAG, "%s Error [%d] : Invalid input\r\n", api_name, rslt);
            break;
        case BMM350_E_SELF_TEST_INVALID_AXIS:
            ESP_LOGE(TAG, "%s Error [%d] : Self-test invalid axis selection\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_BOOT:
            ESP_LOGE(TAG, "%s Error [%d] : OTP boot\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_PAGE_RD:
            ESP_LOGE(TAG, "%s Error [%d] : OTP page read\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_PAGE_PRG:
            ESP_LOGE(TAG, "%s Error [%d] : OTP page prog\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_SIGN:
            ESP_LOGE(TAG, "%s Error [%d] : OTP sign\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_INV_CMD:
            ESP_LOGE(TAG, "%s Error [%d] : OTP invalid command\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_UNDEFINED:
            ESP_LOGE(TAG, "%s Error [%d] : OTP undefined\r\n", api_name, rslt);
            break;
        case BMM350_E_ALL_AXIS_DISABLED:
            ESP_LOGE(TAG, "%s Error [%d] : All axis are disabled\r\n", api_name, rslt);
            break;
        case BMM350_E_PMU_CMD_VALUE:
            ESP_LOGE(TAG, "%s Error [%d] : Unexpected PMU CMD value\r\n", api_name, rslt);
            break;
        default:
            ESP_LOGE(TAG, "%s Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

/*!
 *  @brief Function to select the interface.
 *  @param[in, out] dev     : Structure instance of bmm350_dev.
 *
 * @return Result of API execution status
 * @retval = 0 -> Success
 * @retval < 0 -> Error
 */
int8_t bmm350_interface_init(struct bmm350_dev *dev)
{
    int8_t rslt = BMM350_OK;

    if (dev != NULL)
    {
        dev_addr = BMM350_I2C_ADSEL_SET_LOW;
        dev->intf_ptr = &dev_addr;
        dev->read = bmm350_i2c_read;
        dev->write = bmm350_i2c_write;
        dev->delay_us = bmm350_delay_us_wrapper;

    }
    else
    {
        rslt = BMM350_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief Function to deinitialize the interface.
 *
 * @return Result of API execution status
 * @retval = 0 -> Success
 * @retval < 0 -> Error
 */
void bmm350_coines_deinit(void)
{
    
}

/*****************************************************************************/
/*!                ESP-IDF Platform Specific Functions                        */

/*!
 * @brief Initialize BMM350 I2C device descriptor for ESP-IDF platform
 *
 * This function initializes the I2C device structure with the specified
 * I2C port and GPIO pins. It follows the same pattern as sht4x_init_desc().
 *
 * @param[in] i2c_dev      Pointer to i2c_dev_t structure to initialize
 * @param[in] port         I2C port number (I2C_NUM_0 or I2C_NUM_1)
 * @param[in] sda_gpio     SDA GPIO pin number
 * @param[in] scl_gpio     SCL GPIO pin number
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t bmm350_i2cdev_init(i2c_dev_t *i2c_dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    if (i2c_dev == NULL)
    {
        ESP_LOGE(TAG, "i2c_dev pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    i2c_dev->port = port;
    i2c_dev->addr = dev_addr;  // Use BMM350's I2C address
    i2c_dev->cfg.sda_io_num = sda_gpio;
    i2c_dev->cfg.scl_io_num = scl_gpio;
    i2c_dev->cfg.sda_pullup_en = true;   // Enable internal pullups
    i2c_dev->cfg.scl_pullup_en = true;   // Enable internal pullups

#if HELPER_TARGET_IS_ESP32
    i2c_dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    esp_err_t ret = i2c_dev_create_mutex(i2c_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C device mutex: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BMM350 I2C device initialized on port %d, SDA=%d, SCL=%d, addr=0x%02x",
             port, sda_gpio, scl_gpio, i2c_dev->addr);

    return ESP_OK;
}

/*!
 * @brief Free BMM350 I2C device descriptor for ESP-IDF platform
 *
 * This function releases resources allocated for the I2C device.
 * It follows the same pattern as sht4x_free_desc().
 *
 * @param[in] i2c_dev      Pointer to i2c_dev_t structure to free
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t bmm350_i2cdev_free(i2c_dev_t *i2c_dev)
{
    if (i2c_dev == NULL)
    {
        ESP_LOGE(TAG, "i2c_dev pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_dev_delete_mutex(i2c_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to delete I2C device mutex: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "BMM350 I2C device freed");

    return ESP_OK;
}

/*!
 * @brief Initialize BMM350 device with ESP-IDF I2C backend
 *
 * This is a convenience function that initializes both the BMM350 device
 * structure and the I2C device descriptor for ESP-IDF platform.
 *
 * @param[in,out] dev     Pointer to bmm350_dev structure to initialize
 * @param[in] port        I2C port number (I2C_NUM_0 or I2C_NUM_1)
 * @param[in] sda_gpio    SDA GPIO pin number
 * @param[in] scl_gpio    SCL GPIO pin number
 * @return BMM350_OK on success, error code otherwise
 */
int8_t bmm350_esp_init(struct bmm350_dev *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    if (dev == NULL)
    {
        ESP_LOGE(TAG, "dev pointer is NULL");
        return BMM350_E_NULL_PTR;
    }

    // Allocate i2c_dev_t structure (caller is responsible for freeing)
    i2c_dev_t *i2c_dev = (i2c_dev_t *)malloc(sizeof(i2c_dev_t));
    if (i2c_dev == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate i2c_dev_t");
        return BMM350_E_NULL_PTR;
    }

    // Initialize I2C device
    esp_err_t ret = bmm350_i2cdev_init(i2c_dev, port, sda_gpio, scl_gpio);
    if (ret != ESP_OK)
    {
        free(i2c_dev);
        return BMM350_E_COM_FAIL;
    }

    // Set up BMM350 device structure with I2C functions
    dev->intf_ptr = i2c_dev;
    dev->read = bmm350_i2c_read;
    dev->write = bmm350_i2c_write;
    dev->delay_us = bmm350_delay_us_wrapper;

    ESP_LOGI(TAG, "BMM350 device initialized with ESP-IDF I2C backend");

    return BMM350_OK;
}

/*!
 * @brief Deinitialize BMM350 device with ESP-IDF I2C backend
 *
 * This function releases resources allocated for the BMM350 device.
 *
 * @param[in,out] dev     Pointer to bmm350_dev structure to deinitialize
 * @return BMM350_OK on success, error code otherwise
 */
int8_t bmm350_esp_deinit(struct bmm350_dev *dev)
{
    if (dev == NULL)
    {
        ESP_LOGE(TAG, "dev pointer is NULL");
        return BMM350_E_NULL_PTR;
    }

    if (dev->intf_ptr != NULL)
    {
        i2c_dev_t *i2c_dev = (i2c_dev_t *)dev->intf_ptr;
        esp_err_t ret = bmm350_i2cdev_free(i2c_dev);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to free I2C device: %s", esp_err_to_name(ret));
            return BMM350_E_COM_FAIL;
        }

        free(i2c_dev);
        dev->intf_ptr = NULL;
    }

    ESP_LOGI(TAG, "BMM350 device deinitialized");

    return BMM350_OK;
}

/*****************************************************************************/

/**
 * @brief Prints a fixed-point 48.16 value as a decimal number with 5 decimal places.
 *
 * @param[in] raw  The 64-bit signed integer representing a 48.16 fixed-point value.
 */
void print_A48_16(int64_t raw)
{
    int64_t integer = (raw >> 16); /* signed integer part */
    uint32_t frac_raw = (uint32_t)((raw < 0 ? -raw : raw) & 0xFFFF);

    /* scale fraction to 5 decimal places */
    uint32_t frac_dec = (uint32_t)((uint64_t)(frac_raw * 100000ULL) >> 16);

    if (raw >= 0)
    {
        ESP_LOGE(TAG, "%ld.%05lu ", (int32_t)integer, frac_dec);
    }
    else
    {
        /* handle negative: print integer and fraction as negative */
        if (frac_dec == 0)
        {
            ESP_LOGE(TAG, "%ld ", (int32_t)integer + 1);
        }
        else
        {
            if (integer == 0)
            {
                ESP_LOGE(TAG, "-");
            }

            ESP_LOGE(TAG, "%ld.%05lu ", (int32_t)integer + 1, frac_dec);
        }
    }
}