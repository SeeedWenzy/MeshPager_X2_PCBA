/* SPDX-FileCopyrightText: 2025 KODE DIY SOCIEDAD LIMITADA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "include/kode_lsm6dsox.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LSM6DSOX";

// I2C communication functions that will be passed to the STM32 driver
static int32_t esp_i2c_write(void *handle, uint8_t reg, const uint8_t *buffer, uint16_t length);
static int32_t esp_i2c_read(void *handle, uint8_t reg, uint8_t *buffer, uint16_t length);

// I2C write implementation using ESP-IDF
static int32_t esp_i2c_write(void *handle, uint8_t reg, const uint8_t *buffer, uint16_t length) {
    i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)handle;
    esp_err_t ret;
    
    // Prepare the buffer with register address followed by data
    uint8_t *write_buf = (uint8_t *)malloc(length + 1);
    if (write_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for write buffer");
        return -1;
    }
    
    write_buf[0] = reg;
    memcpy(&write_buf[1], buffer, length);
    
    // Transmit data (register address + data)
    ret = i2c_master_transmit(i2c_dev, write_buf, length + 1, -1);
    free(write_buf);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to device: %s", esp_err_to_name(ret));
        return -1;
    }
    
    return 0;
}

// I2C read implementation using ESP-IDF
static int32_t esp_i2c_read(void *handle, uint8_t reg, uint8_t *buffer, uint16_t length) {
    i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)handle;
    esp_err_t ret;
    
    // First write the register address
    ret = i2c_master_transmit(i2c_dev, &reg, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register address: %s", esp_err_to_name(ret));
        return -1;
    }
    
    // Then read the data
    ret = i2c_master_receive(i2c_dev, buffer, length, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from device: %s", esp_err_to_name(ret));
        return -1;
    }
    
    return 0;
}

esp_err_t kode_lsm6dsox_init_i2c(i2c_master_bus_handle_t bus_handle, uint8_t device_addr, kode_lsm6dsox_handle_t *handle) {
    esp_err_t ret;
    
    // Add device to I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addr,
        .scl_speed_hz = 100000, // 400 kHz 400000
    };
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &handle->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set up communication interface for STM32 driver
    handle->ctx.write_reg = esp_i2c_write;
    handle->ctx.read_reg = esp_i2c_read;
    handle->ctx.handle = (void *)handle->i2c_dev;
    
    // Verify device ID
    uint8_t id;
    ret = kode_lsm6dsox_check_id(handle, &id);
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(handle->i2c_dev);
        handle->i2c_dev = NULL;
        return ret;
    }

    if (id != LSM6DSOX_ID) {
        ESP_LOGE(TAG, "Device ID mismatch. Expected: 0x%X, Got: 0x%X", LSM6DSOX_ID, id);
        i2c_master_bus_rm_device(handle->i2c_dev);
        handle->i2c_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }
    
    // Reset device to ensure we start from a known state
    uint8_t rst = 1;
    lsm6dsox_reset_set(&handle->ctx, rst);
    
    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Apply default configuration
    ret = kode_lsm6dsox_config_default(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to apply default configuration");
        i2c_master_bus_rm_device(handle->i2c_dev);
        handle->i2c_dev = NULL;
        return ret;
    }
    
    ESP_LOGI(TAG, "LSM6DSOX initialized successfully");
    ESP_LOGI(TAG, "LSM6DSOX component version: %d.%d.%d", KODE_LSM6DSOX_VER_MAJOR, KODE_LSM6DSOX_VER_MINOR, KODE_LSM6DSOX_VER_PATCH);

    return ESP_OK;
}

esp_err_t kode_lsm6dsox_deinit_i2c(kode_lsm6dsox_handle_t *handle) {
    if (handle == NULL || handle->i2c_dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_bus_rm_device(handle->i2c_dev);
    if (ret == ESP_OK) {
        memset(handle, 0, sizeof(*handle));
    }
    return ret;
}

esp_err_t kode_lsm6dsox_check_id(kode_lsm6dsox_handle_t *handle, uint8_t *id) {
    int32_t ret = lsm6dsox_device_id_get(&handle->ctx, id);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to read device ID");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_config_default(kode_lsm6dsox_handle_t *handle) {
    int32_t ret;
    
    // Configure accelerometer
    ret = kode_lsm6dsox_accel_config(handle, LSM6DSOX_XL_ODR_104Hz, LSM6DSOX_2g);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure gyroscope
    ret = kode_lsm6dsox_gyro_config(handle, LSM6DSOX_GY_ODR_104Hz, LSM6DSOX_2000dps);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set block data update (BDU) to ensure data is not updated during reading
    ret = lsm6dsox_block_data_update_set(&handle->ctx, PROPERTY_ENABLE);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set block data update");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_accel_config(kode_lsm6dsox_handle_t *handle, 
                                    lsm6dsox_odr_xl_t odr, 
                                    lsm6dsox_fs_xl_t fs) {
    int32_t ret;
    
    // Set accelerometer output data rate
    ret = lsm6dsox_xl_data_rate_set(&handle->ctx, odr);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set accelerometer data rate");
        return ESP_FAIL;
    }
    
    // Set accelerometer full scale
    ret = lsm6dsox_xl_full_scale_set(&handle->ctx, fs);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set accelerometer full scale");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_gyro_config(kode_lsm6dsox_handle_t *handle, 
                                   lsm6dsox_odr_g_t odr, 
                                   lsm6dsox_fs_g_t fs) {
    int32_t ret;
    
    // Set gyroscope output data rate
    ret = lsm6dsox_gy_data_rate_set(&handle->ctx, odr);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set gyroscope data rate");
        return ESP_FAIL;
    }
    
    // Set gyroscope full scale
    ret = lsm6dsox_gy_full_scale_set(&handle->ctx, fs);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set gyroscope full scale");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_read_accel(kode_lsm6dsox_handle_t *handle, 
                                  float *x_mg, float *y_mg, float *z_mg) {
    int16_t raw_accel[3] = {0};
    int32_t ret;
    
    // Read raw accelerometer data
    ret = lsm6dsox_acceleration_raw_get(&handle->ctx, raw_accel);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
        return ESP_FAIL;
    }
    
    // Store raw values in the handle for later use if needed
    handle->accel_x = raw_accel[0];
    handle->accel_y = raw_accel[1];
    handle->accel_z = raw_accel[2];
    
    // Get current full scale to determine conversion function
    lsm6dsox_fs_xl_t fs;
    ret = lsm6dsox_xl_full_scale_get(&handle->ctx, &fs);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get accelerometer full scale");
        return ESP_FAIL;
    }
    
    // Convert raw data to mg based on the current full scale
    switch (fs) {
        case LSM6DSOX_2g:
            *x_mg = lsm6dsox_from_fs2_to_mg(raw_accel[0]);
            *y_mg = lsm6dsox_from_fs2_to_mg(raw_accel[1]);
            *z_mg = lsm6dsox_from_fs2_to_mg(raw_accel[2]);
            break;
            
        case LSM6DSOX_4g:
            *x_mg = lsm6dsox_from_fs4_to_mg(raw_accel[0]);
            *y_mg = lsm6dsox_from_fs4_to_mg(raw_accel[1]);
            *z_mg = lsm6dsox_from_fs4_to_mg(raw_accel[2]);
            break;
            
        case LSM6DSOX_8g:
            *x_mg = lsm6dsox_from_fs8_to_mg(raw_accel[0]);
            *y_mg = lsm6dsox_from_fs8_to_mg(raw_accel[1]);
            *z_mg = lsm6dsox_from_fs8_to_mg(raw_accel[2]);
            break;
            
        case LSM6DSOX_16g:
            *x_mg = lsm6dsox_from_fs16_to_mg(raw_accel[0]);
            *y_mg = lsm6dsox_from_fs16_to_mg(raw_accel[1]);
            *z_mg = lsm6dsox_from_fs16_to_mg(raw_accel[2]);
            break;
            
        default:
            ESP_LOGE(TAG, "Unknown accelerometer full scale");
            return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_read_gyro(kode_lsm6dsox_handle_t *handle, 
                                 float *x_mdps, float *y_mdps, float *z_mdps) {
    int16_t raw_gyro[3] = {0};
    int32_t ret;
    
    // Read raw gyroscope data
    ret = lsm6dsox_angular_rate_raw_get(&handle->ctx, raw_gyro);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to read gyroscope data");
        return ESP_FAIL;
    }
    
    // Store raw values in the handle for later use if needed
    handle->gyro_x = raw_gyro[0];
    handle->gyro_y = raw_gyro[1];
    handle->gyro_z = raw_gyro[2];
    
    // Get current full scale to determine conversion function
    lsm6dsox_fs_g_t fs;
    ret = lsm6dsox_gy_full_scale_get(&handle->ctx, &fs);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get gyroscope full scale");
        return ESP_FAIL;
    }
    
    // Convert raw data to mdps based on the current full scale
    switch (fs) {
        case LSM6DSOX_125dps:
            *x_mdps = lsm6dsox_from_fs125_to_mdps(raw_gyro[0]);
            *y_mdps = lsm6dsox_from_fs125_to_mdps(raw_gyro[1]);
            *z_mdps = lsm6dsox_from_fs125_to_mdps(raw_gyro[2]);
            break;
            
        case LSM6DSOX_250dps:
            *x_mdps = lsm6dsox_from_fs250_to_mdps(raw_gyro[0]);
            *y_mdps = lsm6dsox_from_fs250_to_mdps(raw_gyro[1]);
            *z_mdps = lsm6dsox_from_fs250_to_mdps(raw_gyro[2]);
            break;
            
        case LSM6DSOX_500dps:
            *x_mdps = lsm6dsox_from_fs500_to_mdps(raw_gyro[0]);
            *y_mdps = lsm6dsox_from_fs500_to_mdps(raw_gyro[1]);
            *z_mdps = lsm6dsox_from_fs500_to_mdps(raw_gyro[2]);
            break;
            
        case LSM6DSOX_1000dps:
            *x_mdps = lsm6dsox_from_fs1000_to_mdps(raw_gyro[0]);
            *y_mdps = lsm6dsox_from_fs1000_to_mdps(raw_gyro[1]);
            *z_mdps = lsm6dsox_from_fs1000_to_mdps(raw_gyro[2]);
            break;
            
        case LSM6DSOX_2000dps:
            *x_mdps = lsm6dsox_from_fs2000_to_mdps(raw_gyro[0]);
            *y_mdps = lsm6dsox_from_fs2000_to_mdps(raw_gyro[1]);
            *z_mdps = lsm6dsox_from_fs2000_to_mdps(raw_gyro[2]);
            break;
            
        default:
            ESP_LOGE(TAG, "Unknown gyroscope full scale");
            return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_read_temp(kode_lsm6dsox_handle_t *handle, float *temp_c) {
    int16_t raw_temp;
    int32_t ret;
    
    // Read raw temperature data
    ret = lsm6dsox_temperature_raw_get(&handle->ctx, &raw_temp);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to read temperature data");
        return ESP_FAIL;
    }
    
    // Convert to celsius
    *temp_c = lsm6dsox_from_lsb_to_celsius(raw_temp);
    handle->temperature = *temp_c;
    
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_accel_data_ready(kode_lsm6dsox_handle_t *handle, uint8_t *val) {
    int32_t ret = lsm6dsox_xl_flag_data_ready_get(&handle->ctx, val);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get accelerometer data ready flag");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_gyro_data_ready(kode_lsm6dsox_handle_t *handle, uint8_t *val) {
    int32_t ret = lsm6dsox_gy_flag_data_ready_get(&handle->ctx, val);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get gyroscope data ready flag");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief Configure FIFO mode and watermark
 */
esp_err_t kode_lsm6dsox_fifo_config(kode_lsm6dsox_handle_t *handle, 
                                   kode_lsm6dsox_fifo_mode_t mode,
                                   uint16_t watermark) {
    int32_t ret;
    lsm6dsox_fifo_mode_t fifo_mode;
    
    // Convert our enum to the STM driver enum
    switch (mode) {
        case KODE_LSM6DSOX_FIFO_BYPASS_MODE:
            fifo_mode = LSM6DSOX_BYPASS_MODE;
            break;
        case KODE_LSM6DSOX_FIFO_FIFO_MODE:
            fifo_mode = LSM6DSOX_FIFO_MODE;
            break;
        case KODE_LSM6DSOX_FIFO_CONTINUOUS_MODE:
            fifo_mode = LSM6DSOX_STREAM_MODE;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    // Set watermark
    ret = lsm6dsox_fifo_watermark_set(&handle->ctx, watermark);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set FIFO watermark");
        return ESP_FAIL;
    }
    
    // Set accelerometer and gyroscope FIFO batch (decimation)
    // Match to the current ODR setting for the sensor
    ret = lsm6dsox_fifo_xl_batch_set(&handle->ctx, LSM6DSOX_XL_BATCHED_AT_104Hz);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set accelerometer FIFO batch");
        return ESP_FAIL;
    }
    
    ret = lsm6dsox_fifo_gy_batch_set(&handle->ctx, LSM6DSOX_GY_BATCHED_AT_104Hz);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set gyroscope FIFO batch");
        return ESP_FAIL;
    }
    
    // Set FIFO mode
    ret = lsm6dsox_fifo_mode_set(&handle->ctx, fifo_mode);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set FIFO mode");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief Get number of unread samples in FIFO
 */
esp_err_t kode_lsm6dsox_fifo_data_level(kode_lsm6dsox_handle_t *handle, uint16_t *samples) {
    int32_t ret = lsm6dsox_fifo_data_level_get(&handle->ctx, samples);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get FIFO data level");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief Read accelerometer data from FIFO
 */
esp_err_t kode_lsm6dsox_fifo_read_accel(kode_lsm6dsox_handle_t *handle, 
                                      float acc_mg[][3],
                                      uint16_t samples) {
    int32_t ret;
    uint16_t available_samples;
    lsm6dsox_fs_xl_t fs;
    uint8_t found_samples = 0;
    uint8_t raw_data_u8[6]; // Buffer for uint8_t data as expected by lsm6dsox_fifo_out_raw_get
    int16_t raw_data[3];   // Buffer for converting to int16_t
    
    // Check available samples in FIFO
    ret = lsm6dsox_fifo_data_level_get(&handle->ctx, &available_samples);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get FIFO data level");
        return ESP_FAIL;
    }
    
    // Limit to available samples
    if (samples > available_samples) {
        samples = available_samples;
    }
    
    // Get current full scale setting for conversion
    ret = lsm6dsox_xl_full_scale_get(&handle->ctx, &fs);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get accelerometer full scale");
        return ESP_FAIL;
    }

    // Read samples from FIFO
    for (uint16_t i = 0; i < available_samples && found_samples < samples; i++) {
        lsm6dsox_fifo_tag_t tag;
        
        // Get FIFO tag to identify sensor
        ret = lsm6dsox_fifo_sensor_tag_get(&handle->ctx, &tag);
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to get FIFO tag");
            return ESP_FAIL;
        }
        
        // Read the data with the correct buffer type (uint8_t)
        ret = lsm6dsox_fifo_out_raw_get(&handle->ctx, raw_data_u8);
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to get data from FIFO");
            return ESP_FAIL;
        }
        
        // Process data only if it's accelerometer data
        if (tag == LSM6DSOX_XL_NC_TAG) {
            // Convert uint8_t buffer to int16_t values
            raw_data[0] = (int16_t)raw_data_u8[1];
            raw_data[0] = (raw_data[0] * 256) + (int16_t)raw_data_u8[0];
            raw_data[1] = (int16_t)raw_data_u8[3];
            raw_data[1] = (raw_data[1] * 256) + (int16_t)raw_data_u8[2];
            raw_data[2] = (int16_t)raw_data_u8[5];
            raw_data[2] = (raw_data[2] * 256) + (int16_t)raw_data_u8[4];
            
            // Convert based on full scale
            switch (fs) {
                case LSM6DSOX_2g:
                    acc_mg[found_samples][0] = lsm6dsox_from_fs2_to_mg(raw_data[0]);
                    acc_mg[found_samples][1] = lsm6dsox_from_fs2_to_mg(raw_data[1]);
                    acc_mg[found_samples][2] = lsm6dsox_from_fs2_to_mg(raw_data[2]);
                    break;
                    
                case LSM6DSOX_4g:
                    acc_mg[found_samples][0] = lsm6dsox_from_fs4_to_mg(raw_data[0]);
                    acc_mg[found_samples][1] = lsm6dsox_from_fs4_to_mg(raw_data[1]);
                    acc_mg[found_samples][2] = lsm6dsox_from_fs4_to_mg(raw_data[2]);
                    break;
                    
                case LSM6DSOX_8g:
                    acc_mg[found_samples][0] = lsm6dsox_from_fs8_to_mg(raw_data[0]);
                    acc_mg[found_samples][1] = lsm6dsox_from_fs8_to_mg(raw_data[1]);
                    acc_mg[found_samples][2] = lsm6dsox_from_fs8_to_mg(raw_data[2]);
                    break;
                    
                case LSM6DSOX_16g:
                    acc_mg[found_samples][0] = lsm6dsox_from_fs16_to_mg(raw_data[0]);
                    acc_mg[found_samples][1] = lsm6dsox_from_fs16_to_mg(raw_data[1]);
                    acc_mg[found_samples][2] = lsm6dsox_from_fs16_to_mg(raw_data[2]);
                    break;
                    
                default:
                    ESP_LOGE(TAG, "Unknown accelerometer full scale");
                    return ESP_FAIL;
            }
            found_samples++;
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Read gyroscope data from FIFO
 */
esp_err_t kode_lsm6dsox_fifo_read_gyro(kode_lsm6dsox_handle_t *handle, 
                                     float gyro_mdps[][3],
                                     uint16_t samples) {
    int32_t ret;
    uint16_t available_samples;
    lsm6dsox_fs_g_t fs;
    uint8_t found_samples = 0;
    uint8_t raw_data_u8[6]; // Buffer for uint8_t data as expected by lsm6dsox_fifo_out_raw_get
    int16_t raw_data[3];    // Buffer for converting to int16_t
    
    // Check available samples in FIFO
    ret = lsm6dsox_fifo_data_level_get(&handle->ctx, &available_samples);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get FIFO data level");
        return ESP_FAIL;
    }
    
    // Limit to available samples
    if (samples > available_samples) {
        samples = available_samples;
    }
    
    // Get current full scale setting for conversion
    ret = lsm6dsox_gy_full_scale_get(&handle->ctx, &fs);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get gyroscope full scale");
        return ESP_FAIL;
    }

    // Read samples from FIFO
    for (uint16_t i = 0; i < available_samples && found_samples < samples; i++) {
        lsm6dsox_fifo_tag_t tag;
        
        // Get FIFO tag to identify sensor
        ret = lsm6dsox_fifo_sensor_tag_get(&handle->ctx, &tag);
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to get FIFO tag");
            return ESP_FAIL;
        }
        
        // Read the data with the correct buffer type (uint8_t)
        ret = lsm6dsox_fifo_out_raw_get(&handle->ctx, raw_data_u8);
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to get data from FIFO");
            return ESP_FAIL;
        }
        
        // Process data only if it's gyroscope data
        if (tag == LSM6DSOX_GYRO_NC_TAG) {
            // Convert uint8_t buffer to int16_t values
            raw_data[0] = (int16_t)raw_data_u8[1];
            raw_data[0] = (raw_data[0] * 256) + (int16_t)raw_data_u8[0];
            raw_data[1] = (int16_t)raw_data_u8[3];
            raw_data[1] = (raw_data[1] * 256) + (int16_t)raw_data_u8[2];
            raw_data[2] = (int16_t)raw_data_u8[5];
            raw_data[2] = (raw_data[2] * 256) + (int16_t)raw_data_u8[4];
            
            // Convert based on full scale
            switch (fs) {
                case LSM6DSOX_125dps:
                    gyro_mdps[found_samples][0] = lsm6dsox_from_fs125_to_mdps(raw_data[0]);
                    gyro_mdps[found_samples][1] = lsm6dsox_from_fs125_to_mdps(raw_data[1]);
                    gyro_mdps[found_samples][2] = lsm6dsox_from_fs125_to_mdps(raw_data[2]);
                    break;
                    
                case LSM6DSOX_250dps:
                    gyro_mdps[found_samples][0] = lsm6dsox_from_fs250_to_mdps(raw_data[0]);
                    gyro_mdps[found_samples][1] = lsm6dsox_from_fs250_to_mdps(raw_data[1]);
                    gyro_mdps[found_samples][2] = lsm6dsox_from_fs250_to_mdps(raw_data[2]);
                    break;
                    
                case LSM6DSOX_500dps:
                    gyro_mdps[found_samples][0] = lsm6dsox_from_fs500_to_mdps(raw_data[0]);
                    gyro_mdps[found_samples][1] = lsm6dsox_from_fs500_to_mdps(raw_data[1]);
                    gyro_mdps[found_samples][2] = lsm6dsox_from_fs500_to_mdps(raw_data[2]);
                    break;
                    
                case LSM6DSOX_1000dps:
                    gyro_mdps[found_samples][0] = lsm6dsox_from_fs1000_to_mdps(raw_data[0]);
                    gyro_mdps[found_samples][1] = lsm6dsox_from_fs1000_to_mdps(raw_data[1]);
                    gyro_mdps[found_samples][2] = lsm6dsox_from_fs1000_to_mdps(raw_data[2]);
                    break;
                    
                case LSM6DSOX_2000dps:
                    gyro_mdps[found_samples][0] = lsm6dsox_from_fs2000_to_mdps(raw_data[0]);
                    gyro_mdps[found_samples][1] = lsm6dsox_from_fs2000_to_mdps(raw_data[1]);
                    gyro_mdps[found_samples][2] = lsm6dsox_from_fs2000_to_mdps(raw_data[2]);
                    break;
                    
                default:
                    ESP_LOGE(TAG, "Unknown gyroscope full scale");
                    return ESP_FAIL;
            }
            found_samples++;
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Flush the FIFO (clear all data)
 */
esp_err_t kode_lsm6dsox_fifo_flush(kode_lsm6dsox_handle_t *handle) {
    int32_t ret;
    uint16_t fifo_level;
    uint8_t dummy_u8[6]; // Use uint8_t buffer for lsm6dsox_fifo_out_raw_get
    
    // Check how many samples are in the FIFO
    ret = lsm6dsox_fifo_data_level_get(&handle->ctx, &fifo_level);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get FIFO data level");
        return ESP_FAIL;
    }
    
    // Empty the FIFO by reading all data
    for (uint16_t i = 0; i < fifo_level; i++) {
        lsm6dsox_fifo_tag_t tag;
        
        ret = lsm6dsox_fifo_sensor_tag_get(&handle->ctx, &tag);
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to get FIFO tag");
            return ESP_FAIL;
        }
        
        ret = lsm6dsox_fifo_out_raw_get(&handle->ctx, dummy_u8);
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to get data from FIFO");
            return ESP_FAIL;
        }
    }
    
    // Alternative method: reset FIFO by setting to bypass mode and then back to desired mode
    ret = lsm6dsox_fifo_mode_set(&handle->ctx, LSM6DSOX_BYPASS_MODE);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set FIFO to bypass mode");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "FIFO flushed successfully");
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_interrupt_pin_config(kode_lsm6dsox_handle_t *handle, 
                                          uint8_t active_low, 
                                          uint8_t open_drain) {
    int32_t ret;
    
    // Configure interrupt pins (active-high/low and push-pull/open-drain)
    lsm6dsox_pin_conf_t pin_conf = {0};
    lsm6dsox_int_mode_t int_mode = {0};
    
    // Get current pin configuration
    ret = lsm6dsox_pin_conf_get(&handle->ctx, &pin_conf);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get pin configuration");
        return ESP_FAIL;
    }

    // Configure push-pull or open-drain mode
    ret = lsm6dsox_pin_mode_set(&handle->ctx, open_drain ? LSM6DSOX_OPEN_DRAIN : LSM6DSOX_PUSH_PULL);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set pin mode");
        return ESP_FAIL;
    }
    
    // Configure active-high or active-low mode
    ret = lsm6dsox_pin_polarity_set(&handle->ctx, active_low ? LSM6DSOX_ACTIVE_LOW : LSM6DSOX_ACTIVE_HIGH);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set pin polarity");
        return ESP_FAIL;
    }
    
    // Configure latched/pulsed mode for interrupts (using pulsed mode)
    int_mode.base_latched = 0;  // 0 = pulsed, 1 = latched
    int_mode.active_low = active_low ? 1 : 0;
    
    ret = lsm6dsox_interrupt_mode_set(&handle->ctx, int_mode);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set interrupt mode");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Interrupt pins configured: active_%s, %s", 
             active_low ? "low" : "high", 
             open_drain ? "open_drain" : "push_pull");
    
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_interrupt_enable_int1(kode_lsm6dsox_handle_t *handle, 
                                           uint32_t int_types) {
    lsm6dsox_pin_int1_route_t int1_route = {0};
    int32_t ret;
    
    // Map the generic interrupt types to the specific INT1 register bits
    if (int_types & KODE_LSM6DSOX_INT_DRDY_XL) int1_route.drdy_xl = 1;
    if (int_types & KODE_LSM6DSOX_INT_DRDY_G) int1_route.drdy_g = 1;
    if (int_types & KODE_LSM6DSOX_INT_FIFO_TH) int1_route.fifo_th = 1;
    if (int_types & KODE_LSM6DSOX_INT_FIFO_OVR) int1_route.fifo_ovr = 1;
    if (int_types & KODE_LSM6DSOX_INT_FIFO_FULL) int1_route.fifo_full = 1;
    if (int_types & KODE_LSM6DSOX_INT_FIFO_BDR) int1_route.fifo_bdr = 1;
    if (int_types & KODE_LSM6DSOX_INT_DOUBLE_TAP) int1_route.double_tap = 1;
    if (int_types & KODE_LSM6DSOX_INT_FF) int1_route.free_fall = 1;
    if (int_types & KODE_LSM6DSOX_INT_WAKE_UP) int1_route.wake_up = 1;
    if (int_types & KODE_LSM6DSOX_INT_SINGLE_TAP) int1_route.single_tap = 1;
    if (int_types & KODE_LSM6DSOX_INT_6D) int1_route.six_d = 1;
    
    // Route the selected interrupts to INT1 pin
    ret = lsm6dsox_pin_int1_route_set(&handle->ctx, int1_route);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set INT1 pin routing");
        return ESP_FAIL;
    }
    
    // ESP_LOGI(TAG, "Interrupts enabled on INT1 pin (mask: 0x%X)", int_types);
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_interrupt_enable_int2(kode_lsm6dsox_handle_t *handle, 
                                           uint32_t int_types) {
    lsm6dsox_pin_int2_route_t int2_route = {0};
    int32_t ret;
    
    // Map the generic interrupt types to the specific INT2 register bits
    if (int_types & KODE_LSM6DSOX_INT_DRDY_XL) int2_route.drdy_xl = 1;
    if (int_types & KODE_LSM6DSOX_INT_DRDY_G) int2_route.drdy_g = 1;
    if (int_types & KODE_LSM6DSOX_INT_DRDY_TEMP) int2_route.drdy_temp = 1;
    if (int_types & KODE_LSM6DSOX_INT_FIFO_TH) int2_route.fifo_th = 1;
    if (int_types & KODE_LSM6DSOX_INT_FIFO_OVR) int2_route.fifo_ovr = 1;
    if (int_types & KODE_LSM6DSOX_INT_FIFO_FULL) int2_route.fifo_full = 1;
    if (int_types & KODE_LSM6DSOX_INT_FIFO_BDR) int2_route.fifo_bdr = 1;
    if (int_types & KODE_LSM6DSOX_INT_DOUBLE_TAP) int2_route.double_tap = 1;
    if (int_types & KODE_LSM6DSOX_INT_FF) int2_route.free_fall = 1;
    if (int_types & KODE_LSM6DSOX_INT_WAKE_UP) int2_route.wake_up = 1;
    if (int_types & KODE_LSM6DSOX_INT_SINGLE_TAP) int2_route.single_tap = 1;
    if (int_types & KODE_LSM6DSOX_INT_6D) int2_route.six_d = 1;
    
    // Route the selected interrupts to INT2 pin
    ret = lsm6dsox_pin_int2_route_set(&handle->ctx, NULL, int2_route);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set INT2 pin routing");
        return ESP_FAIL;
    }
    
    // ESP_LOGI(TAG, "Interrupts enabled on INT2 pin (mask: 0x%X)", int_types);
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_interrupt_source_get(kode_lsm6dsox_handle_t *handle, 
                                          lsm6dsox_all_sources_t *int_src) {
    int32_t ret;
    
    // Read all interrupt sources
    ret = lsm6dsox_all_sources_get(&handle->ctx, int_src);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get interrupt sources");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_set_power_mode(kode_lsm6dsox_handle_t *handle, 
                                      kode_lsm6dsox_power_mode_t mode)
{
    int32_t ret;
    
    switch (mode) {
        case KODE_LSM6DSOX_POWER_HIGH_PERFORMANCE:
            // Set accelerometer to high performance mode
            ret = lsm6dsox_xl_power_mode_set(&handle->ctx, LSM6DSOX_HIGH_PERFORMANCE_MD);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to set accelerometer to high performance mode");
                return ESP_FAIL;
            }
            
            // Set gyroscope to high performance mode
            ret = lsm6dsox_gy_power_mode_set(&handle->ctx, LSM6DSOX_GY_HIGH_PERFORMANCE);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to set gyroscope to high performance mode");
                return ESP_FAIL;
            }
            
            // Ensure gyro is not in sleep mode
            ret = lsm6dsox_gy_sleep_mode_set(&handle->ctx, 0);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to wake up gyroscope");
                return ESP_FAIL;
            }
            
            ESP_LOGI(TAG, "Set to high performance mode");
            break;
            
        case KODE_LSM6DSOX_POWER_LOW_NORMAL:
            // Set accelerometer to low power mode
            ret = lsm6dsox_xl_power_mode_set(&handle->ctx, LSM6DSOX_LOW_NORMAL_POWER_MD);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to set accelerometer to low power mode");
                return ESP_FAIL;
            }
            
            // Keep gyroscope in normal mode
            ret = lsm6dsox_gy_power_mode_set(&handle->ctx, LSM6DSOX_GY_HIGH_PERFORMANCE);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to set gyroscope to normal mode");
                return ESP_FAIL;
            }
            
            // Ensure gyro is not in sleep mode
            ret = lsm6dsox_gy_sleep_mode_set(&handle->ctx, 0);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to wake up gyroscope");
                return ESP_FAIL;
            }
            
            ESP_LOGI(TAG, "Set to low normal power mode");
            break;
            
        case KODE_LSM6DSOX_POWER_ULTRA_LOW:
            // Set accelerometer to ultra-low power mode
            ret = lsm6dsox_xl_power_mode_set(&handle->ctx, LSM6DSOX_ULTRA_LOW_POWER_MD);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to set accelerometer to ultra-low power mode");
                return ESP_FAIL;
            }
            
            // Put gyroscope in sleep mode
            ret = lsm6dsox_gy_sleep_mode_set(&handle->ctx, 1);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to put gyroscope in sleep mode");
                return ESP_FAIL;
            }
            
            ESP_LOGI(TAG, "Set to ultra-low power mode");
            break;
            
        case KODE_LSM6DSOX_POWER_SUSPEND:
            // Set accelerometer to ultra-low power mode
            ret = lsm6dsox_xl_power_mode_set(&handle->ctx, LSM6DSOX_ULTRA_LOW_POWER_MD);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to set accelerometer to ultra-low power mode");
                return ESP_FAIL;
            }
            
            // Put gyroscope in sleep mode
            ret = lsm6dsox_gy_sleep_mode_set(&handle->ctx, 1);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to put gyroscope in sleep mode");
                return ESP_FAIL;
            }
            
            // Set both ODRs to lowest possible values
            ret = lsm6dsox_xl_data_rate_set(&handle->ctx, LSM6DSOX_XL_ODR_1Hz6);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to set accelerometer to lowest ODR");
                return ESP_FAIL;
            }
            
            ret = lsm6dsox_gy_data_rate_set(&handle->ctx, LSM6DSOX_GY_ODR_12Hz5);
            if (ret != 0) {
                ESP_LOGE(TAG, "Failed to set gyroscope to lowest ODR");
                return ESP_FAIL;
            }
            
            ESP_LOGI(TAG, "Set to suspend mode");
            break;
            
        default:
            ESP_LOGE(TAG, "Invalid power mode");
            return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

esp_err_t kode_lsm6dsox_get_power_mode(kode_lsm6dsox_handle_t *handle, 
                                      kode_lsm6dsox_power_mode_t *mode)
{
    int32_t ret;
    lsm6dsox_xl_hm_mode_t xl_mode;
    uint8_t gy_sleep;
    lsm6dsox_odr_xl_t xl_odr;
    
    // Get accelerometer power mode
    ret = lsm6dsox_xl_power_mode_get(&handle->ctx, &xl_mode);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get accelerometer power mode");
        return ESP_FAIL;
    }
    
    // Get gyroscope sleep state
    ret = lsm6dsox_gy_sleep_mode_get(&handle->ctx, &gy_sleep);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get gyroscope sleep mode");
        return ESP_FAIL;
    }
    
    // Get accelerometer ODR for suspend mode check
    ret = lsm6dsox_xl_data_rate_get(&handle->ctx, &xl_odr);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get accelerometer ODR");
        return ESP_FAIL;
    }
    
    // Determine overall power mode
    if (xl_mode == LSM6DSOX_HIGH_PERFORMANCE_MD && !gy_sleep) {
        *mode = KODE_LSM6DSOX_POWER_HIGH_PERFORMANCE;
    }
    else if (xl_mode == LSM6DSOX_LOW_NORMAL_POWER_MD && !gy_sleep) {
        *mode = KODE_LSM6DSOX_POWER_LOW_NORMAL;
    }
    else if (xl_mode == LSM6DSOX_ULTRA_LOW_POWER_MD && gy_sleep) {
        if (xl_odr == LSM6DSOX_XL_ODR_1Hz6) {
            *mode = KODE_LSM6DSOX_POWER_SUSPEND;
        } else {
            *mode = KODE_LSM6DSOX_POWER_ULTRA_LOW;
        }
    }
    else {
        ESP_LOGW(TAG, "Inconsistent power mode state");
        return ESP_ERR_INVALID_STATE;
    }
    
    return ESP_OK;
}