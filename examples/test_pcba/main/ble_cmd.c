
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_console.h"
#include "esp_random.h"
#include "esp_spiffs.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_pm.h"
#include "nvs.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_app.h"

static struct {
    struct arg_int *adv;
    struct arg_end *end;
} app_ble_adv_args;

static int app_ble_adv_test(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &app_ble_adv_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, app_ble_adv_args.end, argv[0]);
        return 1;
    }

    int adv = 0;
    if (app_ble_adv_args.adv->count) {
        adv = app_ble_adv_args.adv->ival[0];
        if (adv) {
            ESP_LOGI("BLE",  "Advertising begin");
            app_ble_adv_start();
        } else {
            ESP_LOGI("BLE",  "Advertising stop");
            ble_app_adv_pause();
        }
    }

    return 0;
}

static void register_app_ble_adv_test(void)
{
    app_ble_adv_args.adv =
        arg_int0("a", "adv", "<0|1>", "0: advertising stop, 1: advertising begin, default: 0");
    app_ble_adv_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "ble",
        .help = "ble application adv test",
        .hint = NULL,
        .func = &app_ble_adv_test,
        .argtable = &app_ble_adv_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

void ble_cmd_register_all(void)
{
    register_app_ble_adv_test();
}
