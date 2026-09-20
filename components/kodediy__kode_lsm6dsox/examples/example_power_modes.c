/* SPDX-FileCopyrightText: 2025 KODE DIY SOCIEDAD LIMITADA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file example_power_modes.c
 * @brief Example showing how to use different power modes of LSM6DSOX
 */

#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/kode_lsm6dsox.h"

static const char *TAG = "lsm6dsox-power-example";

// I2C configuration
#define I2C_MASTER_SCL_IO       7       // GPIO pin for I2C clock
#define I2C_MASTER_SDA_IO       6       // GPIO pin for I2C data
#define I2C_MASTER_FREQ_HZ      400000  // I2C master frequency

// IMU device address (depends on SA0 pin state)
#define IMU_ADDR                0x6A    // 0x6A if SA0=0, 0x6B if SA0=1

void app_main(void) {
    ESP_LOGI(TAG, "LSM6DSOX power modes example starting...");
    
    // Initialize I2C
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));
    
    // Initialize LSM6DSOX sensor
    kode_lsm6dsox_handle_t imu_handle;
    ESP_ERROR_CHECK(kode_lsm6dsox_init_i2c(i2c_bus, IMU_ADDR, &imu_handle));
    
    // Variables for sensor data
    float accel_mg[3];
    float gyro_mdps[3];
    kode_lsm6dsox_power_mode_t current_mode;
    
    // Demonstrate different power modes
    const char* mode_names[] = {
        "High Performance",
        "Low Normal Power",
        "Ultra Low Power",
        "Suspend"
    };
    
    for (int mode = KODE_LSM6DSOX_POWER_HIGH_PERFORMANCE; 
         mode <= KODE_LSM6DSOX_POWER_SUSPEND; 
         mode++) {
        
        ESP_LOGI(TAG, "\nSetting power mode to: %s", mode_names[mode]);
        ESP_ERROR_CHECK(kode_lsm6dsox_set_power_mode(&imu_handle, mode));
        
        // Verify the mode was set correctly
        ESP_ERROR_CHECK(kode_lsm6dsox_get_power_mode(&imu_handle, &current_mode));
        ESP_LOGI(TAG, "Current power mode: %s", mode_names[current_mode]);
        
        // Take some measurements
        for (int i = 0; i < 5; i++) {
            ESP_ERROR_CHECK(kode_lsm6dsox_read_accel(&imu_handle, 
                                                    &accel_mg[0], 
                                                    &accel_mg[1], 
                                                    &accel_mg[2]));
            
            ESP_ERROR_CHECK(kode_lsm6dsox_read_gyro(&imu_handle, 
                                                   &gyro_mdps[0], 
                                                   &gyro_mdps[1], 
                                                   &gyro_mdps[2]));
            
            ESP_LOGI(TAG, "Accel (mg): X=%.2f, Y=%.2f, Z=%.2f", 
                     accel_mg[0], accel_mg[1], accel_mg[2]);
            ESP_LOGI(TAG, "Gyro (mdps): X=%.2f, Y=%.2f, Z=%.2f", 
                     gyro_mdps[0], gyro_mdps[1], gyro_mdps[2]);
            
            // Add delay based on power mode
            switch (mode) {
                case KODE_LSM6DSOX_POWER_HIGH_PERFORMANCE:
                    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay
                    break;
                case KODE_LSM6DSOX_POWER_LOW_NORMAL:
                    vTaskDelay(pdMS_TO_TICKS(20));  // 20ms delay
                    break;
                case KODE_LSM6DSOX_POWER_ULTRA_LOW:
                    vTaskDelay(pdMS_TO_TICKS(50));  // 50ms delay
                    break;
                case KODE_LSM6DSOX_POWER_SUSPEND:
                    vTaskDelay(pdMS_TO_TICKS(1000));  // 1s delay
                    break;
            }
        }
        
        // Add delay between mode changes
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    ESP_LOGI(TAG, "\nPower modes demonstration completed");
    ESP_LOGI(TAG, "Typical power consumption in each mode:");
    ESP_LOGI(TAG, "- High Performance: 3.75 mA");
    ESP_LOGI(TAG, "- Low Normal: 3.45 mA");
    ESP_LOGI(TAG, "- Ultra Low: 0.03 mA");
    ESP_LOGI(TAG, "- Suspend: 0.03 mA with minimal ODR");
    
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
} 