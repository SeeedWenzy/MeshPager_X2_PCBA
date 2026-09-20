#include "bsp_i2c_control.h"
#include "bsp_err_check.h"

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "meshpager_x2.h"



static const char *TAG = "bsp_i2c_control";

#define BSP_I2C_SCAN_ADDR_MIN   0x03
#define BSP_I2C_SCAN_ADDR_MAX   0x77
#define BSP_I2C_SCAN_TIMEOUT_MS 50


static bool i2c_0_initialized = false;
static bool i2c_1_initialized = false;


static i2c_master_bus_handle_t i2c_0_handle = NULL;
static i2c_master_bus_handle_t i2c_1_handle = NULL;


static esp_err_t bsp_i2c_scan(i2c_master_bus_handle_t bus_handle, const char *bus_name)
{
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "%s handle is null, scan aborted", bus_name);
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t found_count = 0;

    ESP_LOGI(TAG, "Start scanning %s", bus_name);

    for (uint8_t addr = BSP_I2C_SCAN_ADDR_MIN; addr <= BSP_I2C_SCAN_ADDR_MAX; addr++) {
        esp_err_t ret = i2c_master_probe(bus_handle, addr, BSP_I2C_SCAN_TIMEOUT_MS);

        if (ret == ESP_OK) {
            found_count++;
            ESP_LOGI(TAG, "%s found slave device at 0x%02X", bus_name, addr);
            continue;
        }

        if (ret != ESP_ERR_NOT_FOUND && ret != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "%s probe 0x%02X failed: %s", bus_name, addr, esp_err_to_name(ret));
        }
    }

    if (found_count == 0) {
        ESP_LOGI(TAG, "%s scan finished, no slave device found", bus_name);
    } else {
        ESP_LOGI(TAG, "%s scan finished, %u device(s) found", bus_name, found_count);
    }

    return ESP_OK;
}

/* ------------------------------------------------ i2c0 start ------------------------------------------------ */
esp_err_t bsp_i2c_0_init(void)
{
    /* I2C was initialized before */
    if (i2c_0_initialized) {
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = BSP_I2C_0_SDA,
        .scl_io_num = BSP_I2C_0_SCL,
        .i2c_port = I2C_PORT_NUM_0,
        .glitch_ignore_cnt = 7,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_bus_conf, &i2c_0_handle));

    i2c_0_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_0_deinit(void)
{
    /* Idempotent: a caller may deinit more than once (e.g. per-test cleanup
     * followed by the deep-sleep teardown). Without this guard the second
     * call would feed a dangling handle to i2c_del_master_bus() and double-free. */
    if (!i2c_0_initialized) {
        return ESP_OK;
    }
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_0_handle));
    i2c_0_handle = NULL;
    i2c_0_initialized = false;
    ESP_LOGI(TAG, "I2C_0 deinitialized successfully");
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_0_get_handle(void)
{
    return i2c_0_handle;
}

esp_err_t bsp_i2c_0_scan(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_0_init());
    return bsp_i2c_scan(i2c_0_handle, "I2C_0");
}
/* ------------------------------------------------ i2c0 end ------------------------------------------------ */
/* ------------------------------------------------ i2c1 start ------------------------------------------------ */
esp_err_t bsp_i2c_1_init(void)
{
    /* I2C was initialized before */
    if (i2c_1_initialized) {
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = BSP_I2C_1_SDA,
        .scl_io_num = BSP_I2C_1_SCL,
        .i2c_port = I2C_PORT_NUM_1,
        .glitch_ignore_cnt = 7,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_bus_conf, &i2c_1_handle));

    i2c_1_initialized = true;
    ESP_LOGI(TAG, "I2C_1 initialized successfully");
    return ESP_OK;
}

esp_err_t bsp_i2c_1_deinit(void)
{
    /* Idempotent: a caller may deinit more than once (e.g. per-test cleanup
     * followed by the deep-sleep teardown). Without this guard the second
     * call would feed a dangling handle to i2c_del_master_bus() and double-free. */
    if (!i2c_1_initialized) {
        return ESP_OK;
    }
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_1_handle));
    i2c_1_handle = NULL;
    i2c_1_initialized = false;
    ESP_LOGI(TAG, "I2C_1 deinitialized successfully");
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_1_get_handle(void)
{
    return i2c_1_handle;
}

esp_err_t bsp_i2c_1_scan(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_1_init());
    return bsp_i2c_scan(i2c_1_handle, "I2C_1");
}

/* ------------------------------------------------ i2c1 end ------------------------------------------------ */
