#include <stdio.h>

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"

#include "bsp_i2c_control.h"

static const char *TAG = "I2C_TEST";

static struct {
    struct arg_int *bus;
    struct arg_end *end;
} i2c_scan_args;

static int i2c_scan_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&i2c_scan_args);
    esp_err_t ret = ESP_OK;

    if (nerrors != 0) {
        arg_print_errors(stderr, i2c_scan_args.end, argv[0]);
        return 1;
    }

    if (i2c_scan_args.bus->count == 0) {
        ESP_LOGI(TAG, "Scan all I2C buses");

        ret = bsp_i2c_0_scan();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C_0 scan failed: %s", esp_err_to_name(ret));
            return 1;
        }

        ret = bsp_i2c_1_scan();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C_1 scan failed: %s", esp_err_to_name(ret));
            return 1;
        }

        return 0;
    }

    switch (i2c_scan_args.bus->ival[0]) {
    case 0:
        ret = bsp_i2c_0_scan();
        break;
    case 1:
        ret = bsp_i2c_1_scan();
        break;
    default:
        ESP_LOGE(TAG, "Unsupported bus id %d, expected 0 or 1", i2c_scan_args.bus->ival[0]);
        return 1;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C_%d scan failed: %s", i2c_scan_args.bus->ival[0], esp_err_to_name(ret));
        return 1;
    }

    return 0;
}

static void register_i2c_scan_cmd(void)
{
    i2c_scan_args.bus = arg_int0("b", "bus", "<0|1>", "I2C bus id, omit to scan both buses");
    i2c_scan_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "i2c_scan",
        .help = "Scan slave devices on I2C bus 0, 1, or both",
        .hint = NULL,
        .func = &i2c_scan_cmd,
        .argtable = &i2c_scan_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void i2c_cmd_register_all(void)
{
    register_i2c_scan_cmd();

    ESP_LOGI(TAG, "I2C test commands registered:");
    ESP_LOGI(TAG, "  i2c_scan                  Scan both I2C buses");
    ESP_LOGI(TAG, "  i2c_scan -b <0|1>         Scan a specific I2C bus");
}