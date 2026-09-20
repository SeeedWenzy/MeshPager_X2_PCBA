#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "freertos/task.h"

#include "meshpager_x2.h"

#include "ble_coex.h"
#include "lora_rx_test.h"
#include "wifi_ap_test.h"
#include "iperf_cmd.h"

static const char *TAG = "COEX_MAIN";
#define APP_SW_VERSION "test_wireless_coexistence-2.0.1"

static struct {
    struct arg_str *radio;
    struct arg_dbl *dbm;
    struct arg_end *end;
} power_args;

static int power_cmd(int argc, char **argv)
{
    if (arg_parse(argc, argv, (void **)&power_args) != 0) {
        arg_print_errors(stderr, power_args.end, argv[0]);
        return 1;
    }
    const char *radio = power_args.radio->sval[0];
    double dbm = power_args.dbm->dval[0];
    esp_err_t ret;
    if (strcasecmp(radio, "wifi") == 0) {
        ret = wifi_ap_test_set_tx_power_dbm(dbm);
    } else if (strcasecmp(radio, "ble") == 0) {
        if (dbm != (int)dbm) {
            printf("BLE TX power must be an integer dBm value\n");
            return 1;
        }
        ret = ble_coex_set_tx_power_dbm((int)dbm);
    } else {
        printf("radio must be wifi or ble\n");
        return 1;
    }
    if (ret != ESP_OK) {
        printf("set %s TX power failed: %s\n", radio, esp_err_to_name(ret));
        return 1;
    }
    return 0;
}

static void register_power_cmd(void)
{
    power_args.radio = arg_str1(NULL, NULL, "<wifi|ble>", "radio");
    power_args.dbm = arg_dbl1(NULL, NULL, "<dBm>", "TX power");
    power_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
        .command = "rf_power",
        .help = "Set Wi-Fi or BLE TX power: rf_power <wifi|ble> <dBm>",
        .func = power_cmd,
        .argtable = &power_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void app_console_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "coex>";
#if CONFIG_ESP_CONSOLE_UART_DEFAULT || CONFIG_ESP_CONSOLE_UART_CUSTOM
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#else
#error "Configure a supported ESP console backend"
#endif
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void app_main(void)
{
    esp_err_t ret = bsp_power_up_init();
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "board power is off; waiting");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Application software version: SW=%s", APP_SW_VERSION);

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);
    }

    ESP_ERROR_CHECK(wifi_ap_test_start());
    ESP_ERROR_CHECK(ble_coex_start());
    ESP_ERROR_CHECK(lora_rx_test_start());
    ESP_ERROR_CHECK(iperf_cmd_register_iperf());
    register_power_cmd();
    app_console_start();

    ESP_LOGI(TAG, "Wireless coexistence test is running");
    ESP_LOGI(TAG, "Wi-Fi: connect to wireless_coex_ap and run an iPerf client against the AP IP");
    ESP_LOGI(TAG, "BLE: connect to wireless_coex and use the custom read/write/notify characteristic");
    ESP_LOGI(TAG, "LoRa: continuous RX remains active during both tests");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
