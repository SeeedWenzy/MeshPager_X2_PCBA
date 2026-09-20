/* SPDX-FileCopyrightText: 2025 KODE DIY SOCIEDAD LIMITADA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file example_interrupt.c
 * @brief Example showing how to use INT1 and INT2 pins with LSM6DSOX
 */

#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/kode_lsm6dsox.h"

static const char *TAG = "lsm6dsox-example";

// I2C configuration
#define I2C_MASTER_SCL_IO       7       // GPIO pin for I2C clock
#define I2C_MASTER_SDA_IO       6       // GPIO pin for I2C data
#define I2C_MASTER_FREQ_HZ      400000  // I2C master frequency

// Interrupt pins
#define INT1_PIN                10      // GPIO pin for INT1
#define INT2_PIN                11      // GPIO pin for INT2

// IMU device address (depends on SA0 pin state)
#define IMU_ADDR                0x6A    // 0x6A if SA0=0, 0x6B if SA0=1

// Interrupt counters and flags
static volatile uint32_t int1_count = 0;
static volatile uint32_t int2_count = 0;
static volatile bool int1_triggered = false;
static volatile bool int2_triggered = false;

// ISR for INT1 pin
static void IRAM_ATTR int1_isr_handler(void* arg) {
    int1_count++;
    int1_triggered = true;
}

// ISR for INT2 pin
static void IRAM_ATTR int2_isr_handler(void* arg) {
    int2_count++;
    int2_triggered = true;
}

void app_main(void) {
    ESP_LOGI(TAG, "LSM6DSOX interrupt example starting...");
    
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
    
    // Configure LSM6DSOX with custom settings
    // Higher rate for more responsive interrupts
    ESP_ERROR_CHECK(kode_lsm6dsox_accel_config(&imu_handle, LSM6DSOX_XL_ODR_416Hz, LSM6DSOX_2g));
    ESP_ERROR_CHECK(kode_lsm6dsox_gyro_config(&imu_handle, LSM6DSOX_GY_ODR_416Hz, LSM6DSOX_2000dps));
    
    // Configure GPIO pins for interrupts
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE; // Rising edge trigger
    io_conf.pin_bit_mask = (1ULL << INT1_PIN) | (1ULL << INT2_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Install GPIO ISRs
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(INT1_PIN, int1_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(INT2_PIN, int2_isr_handler, NULL));
    
    // Configure interrupts - active high, push-pull
    ESP_ERROR_CHECK(kode_lsm6dsox_interrupt_pin_config(&imu_handle, 0, 0));
    
    // Configure INT1 for data ready interrupts (accelerometer)
    ESP_ERROR_CHECK(kode_lsm6dsox_interrupt_enable_int1(&imu_handle, KODE_LSM6DSOX_INT_DRDY_XL));
    
    // Configure INT2 for data ready interrupts (gyroscope)
    ESP_ERROR_CHECK(kode_lsm6dsox_interrupt_enable_int2(&imu_handle, KODE_LSM6DSOX_INT_DRDY_G));
    
    ESP_LOGI(TAG, "Interrupt configuration complete");
    ESP_LOGI(TAG, "INT1 will trigger on accelerometer data ready");
    ESP_LOGI(TAG, "INT2 will trigger on gyroscope data ready");
    
    // Variables for storing sensor data
    float accel_mg[3];
    float gyro_mdps[3];
    
    // Main loop
    while (1) {
        if (int1_triggered) {
            int1_triggered = false;
            // Read accelerometer data
            ESP_ERROR_CHECK(kode_lsm6dsox_read_accel(&imu_handle, &accel_mg[0], &accel_mg[1], &accel_mg[2]));
            
            ESP_LOGI(TAG, "INT1 count: %lu, Accel: X=%.2f mg, Y=%.2f mg, Z=%.2f mg", 
                     int1_count, accel_mg[0], accel_mg[1], accel_mg[2]);
        }
        
        if (int2_triggered) {
            int2_triggered = false;
            // Read gyroscope data
            ESP_ERROR_CHECK(kode_lsm6dsox_read_gyro(&imu_handle, &gyro_mdps[0], &gyro_mdps[1], &gyro_mdps[2]));
            
            ESP_LOGI(TAG, "INT2 count: %lu, Gyro: X=%.2f mdps, Y=%.2f mdps, Z=%.2f mdps", 
                     int2_count, gyro_mdps[0], gyro_mdps[1], gyro_mdps[2]);
        }
        
        // Small delay to prevent busy waiting
        vTaskDelay(pdMS_TO_TICKS(10));
    }
} 