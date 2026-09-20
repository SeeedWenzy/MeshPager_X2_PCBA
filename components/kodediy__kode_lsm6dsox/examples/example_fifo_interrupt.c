/* SPDX-FileCopyrightText: 2025 KODE DIY SOCIEDAD LIMITADA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file example_fifo_interrupt.c
 * @brief Example showing how to use FIFO with interrupts on LSM6DSOX
 */

#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "include/kode_lsm6dsox.h"

static const char *TAG = "lsm6dsox-fifo-example";

// I2C configuration
#define I2C_MASTER_SCL_IO       7       // GPIO pin for I2C clock
#define I2C_MASTER_SDA_IO       6       // GPIO pin for I2C data
#define I2C_MASTER_FREQ_HZ      400000  // I2C master frequency

// Interrupt pins
#define INT1_PIN                10      // GPIO pin for INT1

// IMU device address (depends on SA0 pin state)
#define IMU_ADDR                0x6A    // 0x6A if SA0=0, 0x6B if SA0=1

// FIFO configuration
#define FIFO_WATERMARK          32      // FIFO watermark level (samples)
#define ACCEL_BATCH_SIZE        32      // Number of accelerometer samples to read per batch
#define GYRO_BATCH_SIZE         32      // Number of gyroscope samples to read per batch

// Semaphore to signal FIFO threshold reached
static SemaphoreHandle_t fifo_ready_sem;

// ISR for INT1 pin (FIFO threshold)
static void IRAM_ATTR int1_isr_handler(void* arg) {
    // Signal that FIFO threshold has been reached
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(fifo_ready_sem, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void app_main(void) {
    ESP_LOGI(TAG, "LSM6DSOX FIFO with interrupts example starting...");
    
    // Create semaphore for FIFO ready signal
    fifo_ready_sem = xSemaphoreCreateBinary();
    if (fifo_ready_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }
    
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
    ESP_ERROR_CHECK(kode_lsm6dsox_accel_config(&imu_handle, LSM6DSOX_XL_ODR_416Hz, LSM6DSOX_2g));
    ESP_ERROR_CHECK(kode_lsm6dsox_gyro_config(&imu_handle, LSM6DSOX_GY_ODR_416Hz, LSM6DSOX_2000dps));
    
    // Configure GPIO pin for interrupt
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE; // Rising edge trigger
    io_conf.pin_bit_mask = (1ULL << INT1_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Install GPIO ISR
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(INT1_PIN, int1_isr_handler, NULL));
    
    // Configure interrupts - active high, push-pull
    ESP_ERROR_CHECK(kode_lsm6dsox_interrupt_pin_config(&imu_handle, 0, 0));
    
    // Configure INT1 for FIFO threshold interrupt
    ESP_ERROR_CHECK(kode_lsm6dsox_interrupt_enable_int1(&imu_handle, KODE_LSM6DSOX_INT_FIFO_TH));
    
    // Configure FIFO - continuous mode with watermark
    ESP_ERROR_CHECK(kode_lsm6dsox_fifo_config(&imu_handle, KODE_LSM6DSOX_FIFO_CONTINUOUS_MODE, FIFO_WATERMARK));
    
    ESP_LOGI(TAG, "FIFO and interrupt configuration complete");
    ESP_LOGI(TAG, "INT1 will trigger when FIFO reaches watermark level (%u samples)", FIFO_WATERMARK);
    
    // Arrays for storing batches of sensor data
    float accel_data[ACCEL_BATCH_SIZE][3]; // [samples][x,y,z]
    float gyro_data[GYRO_BATCH_SIZE][3];   // [samples][x,y,z]
    uint16_t fifo_samples;
    
    // Main loop
    uint32_t batch_count = 0;
    
    while (1) {
        // Wait for FIFO threshold interrupt 
        if (xSemaphoreTake(fifo_ready_sem, portMAX_DELAY) == pdTRUE) {
            batch_count++;
            
            // Check how many samples are in the FIFO
            ESP_ERROR_CHECK(kode_lsm6dsox_fifo_data_level(&imu_handle, &fifo_samples));
            ESP_LOGI(TAG, "Batch %lu: FIFO contains %u samples", batch_count, fifo_samples);
            
            // Read accelerometer batch from FIFO
            ESP_ERROR_CHECK(kode_lsm6dsox_fifo_read_accel(&imu_handle, accel_data, ACCEL_BATCH_SIZE));
            
            // Read gyroscope batch from FIFO
            ESP_ERROR_CHECK(kode_lsm6dsox_fifo_read_gyro(&imu_handle, gyro_data, GYRO_BATCH_SIZE));
            
            // Print the first and last sample of each batch (for demonstration)
            ESP_LOGI(TAG, "First accel sample: X=%.2f mg, Y=%.2f mg, Z=%.2f mg", 
                     accel_data[0][0], accel_data[0][1], accel_data[0][2]);
            ESP_LOGI(TAG, "Last accel sample: X=%.2f mg, Y=%.2f mg, Z=%.2f mg", 
                     accel_data[ACCEL_BATCH_SIZE-1][0], accel_data[ACCEL_BATCH_SIZE-1][1], accel_data[ACCEL_BATCH_SIZE-1][2]);
            
            ESP_LOGI(TAG, "First gyro sample: X=%.2f mdps, Y=%.2f mdps, Z=%.2f mdps", 
                     gyro_data[0][0], gyro_data[0][1], gyro_data[0][2]);
            ESP_LOGI(TAG, "Last gyro sample: X=%.2f mdps, Y=%.2f mdps, Z=%.2f mdps", 
                     gyro_data[GYRO_BATCH_SIZE-1][0], gyro_data[GYRO_BATCH_SIZE-1][1], gyro_data[GYRO_BATCH_SIZE-1][2]);
            
            // Here you would process the batch data (e.g., activity recognition, motion analysis)
            // For this example, we just print the first and last samples
            
            // In a real application, you might want to:
            // 1. Apply filters to the data
            // 2. Calculate features (mean, variance, frequency domain features)
            // 3. Feed the features to a classifier or other algorithm
            // 4. Take action based on the classification results
        }
    }
} 