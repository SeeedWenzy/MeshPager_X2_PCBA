/**
 * BMM350 Magnetometer Sensor Test Example
 *
 * This example demonstrates basic usage of the BMM350 magnetometer sensor:
 * - Sensor initialization
 * - Configuration (ODR, performance, axes)
 * - Reading compensated magnetometer data (X, Y, Z) and temperature
 * - Self-test functionality
 *
 * Hardware connection:
 * - BMM350 I2C address: 0x10 (default) or 0x11
 * - SDA: GPIO 4 (configurable)
 * - SCL: GPIO 5 (configurable)
 */
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <string.h>
#include <esp_err.h>
#include "esp_system.h"
#include "esp_log.h"

#include "bmm350.h"
#include "common.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "bmm350_test";

// BMM350 device structure
static struct bmm350_dev dev;

/**
 * @brief Print sensor version information
 */
static void print_api_version(void)
{
    struct bmm350_version ver;
    int8_t rslt = bmm350_api_version(&ver);
    
    if (rslt == BMM350_OK) {
        ESP_LOGI(TAG, "BMM350 API Version: %d.%d.%d", ver.major, ver.minor, ver.bugfix);
    } else {
        ESP_LOGE(TAG, "Failed to get API version: %d", rslt);
    }
}

/**
 * @brief Perform self-test on the sensor
 */
static void perform_self_test(void)
{
    struct bmm350_self_test self_test_data;
    int8_t rslt = bmm350_perform_self_test(&self_test_data, &dev);
    
    if (rslt == BMM350_OK) {
        ESP_LOGI(TAG, "Self-test completed successfully");
        ESP_LOGI(TAG, "  X-axis - High: %.2f uT, Low: %.2f uT", 
                 self_test_data.out_ust_xh, self_test_data.out_ust_xl);
        ESP_LOGI(TAG, "  Y-axis - High: %.2f uT, Low: %.2f uT", 
                 self_test_data.out_ust_yh, self_test_data.out_ust_yl);
        ESP_LOGI(TAG, "  X-axis result: %.2f uT", self_test_data.out_ust_x);
        ESP_LOGI(TAG, "  Y-axis result: %.2f uT", self_test_data.out_ust_y);
    } else {
        ESP_LOGE(TAG, "Self-test failed: %d", rslt);
    }
}

/**
 * @brief Main measurement task
 * 
 * This task continuously reads magnetometer data and temperature
 * from the BMM350 sensor at 1 Hz rate.
 */
void bmm350_measurement_task(void *pvParameters)
{
    struct bmm350_mag_temp_data mag_data;
    int8_t rslt;
    TickType_t last_wakeup = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Starting measurement task...");

    while (1) {
        // Read compensated magnetometer and temperature data
        rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_data, &dev);
        
        if (rslt == BMM350_OK) {
            // Print magnetometer data in microtesla (uT)
            printf("BMM350 - Mag X: %8.2f uT, Mag Y: %8.2f uT, Mag Z: %8.2f uT, Temp: %6.2f °C\n",
                   mag_data.x, mag_data.y, mag_data.z, mag_data.temperature);
        } else {
            ESP_LOGE(TAG, "Failed to read sensor data: %d", rslt);
        }

        // Wait for 1 second
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Application entry point
 */
void app_main()
{
    int8_t rslt;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "BMM350 Magnetometer Test Example");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize I2C driver
    ESP_ERROR_CHECK(i2cdev_init());
    
    // Clear device structure
    memset(&dev, 0, sizeof(dev));
    
    // Initialize BMM350 sensor with ESP-IDF backend
    // Parameters: I2C port 0, SDA GPIO 4, SCL GPIO 5
    rslt = bmm350_esp_init(&dev, I2C_NUM_0, GPIO_NUM_4, GPIO_NUM_5);
    
    if (rslt != BMM350_OK) {
        ESP_LOGE(TAG, "BMM350 initialization failed: %d", rslt);
        ESP_LOGE(TAG, "Please check:");
        ESP_LOGE(TAG, "  - Sensor is connected properly");
        ESP_LOGE(TAG, "  - I2C SDA/SCL pins are correct");
        ESP_LOGE(TAG, "  - Sensor power supply is connected");
        return;
    }
    
    ESP_LOGI(TAG, "BMM350 initialized successfully!");
    ESP_LOGI(TAG, "Chip ID: 0x%02X", dev.chip_id);
    
    // Print API version
    print_api_version();
    
    // Set sensor to normal mode
    rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
    if (rslt == BMM350_OK) {
        ESP_LOGI(TAG, "Power mode set to NORMAL");
    } else {
        ESP_LOGE(TAG, "Failed to set power mode: %d", rslt);
    }
    
    // Configure ODR (Output Data Rate) and performance
    // ODR: 25 Hz, Performance: Low noise (averaging 4 samples)
    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_LOWNOISE, &dev);
    if (rslt == BMM350_OK) {
        ESP_LOGI(TAG, "ODR set to 25 Hz with LOW noise performance");
    } else {
        ESP_LOGE(TAG, "Failed to set ODR/performance: %d", rslt);
    }
    
    // Enable all three axes (X, Y, Z)
    rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);
    if (rslt == BMM350_OK) {
        ESP_LOGI(TAG, "All axes enabled (X, Y, Z)");
    } else {
        ESP_LOGE(TAG, "Failed to enable axes: %d", rslt);
    }
    
    // Perform self-test (optional, takes a few seconds)
    ESP_LOGI(TAG, "Performing self-test...");
    perform_self_test();
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Starting continuous measurements...");
    ESP_LOGI(TAG, "========================================");
    
    // Create measurement task
    xTaskCreatePinnedToCore(bmm350_measurement_task, "bmm350_task", 
                           configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
