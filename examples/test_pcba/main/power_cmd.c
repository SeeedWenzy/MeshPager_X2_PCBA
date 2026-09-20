#include <stdio.h>

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "meshpager_x2.h"
#include "bsp_gnss_control.h"
#include "bsp_i2c_control.h"
#include "bsp_board_extra.h"

static const char *TAG = "POWER";

/* Reset every pin in an array to its default reset state (input, no pull, no
 * interrupt) — the lowest-leakage default, same technique used by the board's
 * production low-power teardown. */
static void reset_pins(const gpio_num_t *pins, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        gpio_reset_pin(pins[i]);
    }
}

/* ------------------------------------------------------------------ GNSS -- */
static struct {
    struct arg_end *end;
} power_gnss_off_args;

static const gpio_num_t gnss_pins[] = {
    BSP_GNSS_RX,
    BSP_GNSS_TX,
    BSP_GNSS_PPS0,
};

static int power_gnss_off_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&power_gnss_off_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, power_gnss_off_args.end, argv[0]);
        return 1;
    }

    ESP_LOGI(TAG, "Powering off GNSS...");

    bsp_gnss_scan_stop();   /* stop NMEA scan + deinit UART1 */
    bsp_gnss_poweroff();    /* PWR_EN=0, RST=0, VRTC_EN=0; floats UART/PPS pins */

    reset_pins(gnss_pins, sizeof(gnss_pins) / sizeof(gnss_pins[0]));

    ESP_LOGI(TAG, "GNSS powered off (PWR_EN + VRTC_EN cut, pins safe)");
    return 0;
}

static void register_power_gnss_off_cmd(void)
{
    power_gnss_off_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "power_gnss_off",
        .help = "Power off GNSS: stop scan, cut PWR_EN/VRTC_EN/RST, float UART pins",
        .hint = NULL,
        .func = &power_gnss_off_cmd,
        .argtable = &power_gnss_off_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/* -------------------------------------------------------------- SD card -- */
static struct {
    struct arg_end *end;
} power_sd_off_args;

static const gpio_num_t sd_pins[] = {
    BSP_SD_D0,
    BSP_SD_CLK,
    BSP_SD_CMD,
};

static int power_sd_off_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&power_sd_off_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, power_sd_off_args.end, argv[0]);
        return 1;
    }

    ESP_LOGI(TAG, "Powering off SD card...");

    if (bsp_sdcard != NULL) {
        esp_err_t ret = bsp_sdcard_unmount();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "SD card unmount: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGI(TAG, "SD card not mounted, skip unmount");
    }

    reset_pins(sd_pins, sizeof(sd_pins) / sizeof(sd_pins[0]));

    bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 0);

    ESP_LOGI(TAG, "SD card powered off (rail + pins safe)");
    return 0;
}

static void register_power_sd_off_cmd(void)
{
    power_sd_off_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "power_sd_off",
        .help = "Power off SD card: unmount if mounted, cut PWR_EN, float data pins",
        .hint = NULL,
        .func = &power_sd_off_cmd,
        .argtable = &power_sd_off_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/* ------------------------------------------------------------------- LCD -- */
static struct {
    struct arg_end *end;
} power_lcd_off_args;

static const gpio_num_t lcd_pins[] = {
    BSP_LCD_CS,
    BSP_LCD_SDA,
    BSP_LCD_SCK,
    BSP_LCD_A0,
    BSP_LCD_PWM,
};

static int power_lcd_off_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&power_lcd_off_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, power_lcd_off_args.end, argv[0]);
        return 1;
    }

    ESP_LOGI(TAG, "Powering off LCD...");

    lcd_bl_off();
    /* deinit frees the SPI bus + panel/io and cuts PWR_EN/RST; do it before
     * resetting the pins it owns. */
    esp_err_t ret = bsp_display_deinit();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Display deinit: %s", esp_err_to_name(ret));
    }

    reset_pins(lcd_pins, sizeof(lcd_pins) / sizeof(lcd_pins[0]));

    bsp_exp_output_io_set_level(BSP_LCD_PWR_EN, 0);
    bsp_exp_output_io_set_level(BSP_LCD_RST, 0);

    ESP_LOGI(TAG, "LCD powered off (rail + pins safe)");
    return 0;
}

static void register_power_lcd_off_cmd(void)
{
    power_lcd_off_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "power_lcd_off",
        .help = "Power off LCD: backlight off, deinit panel/SPI, cut PWR_EN/RST, float pins",
        .hint = NULL,
        .func = &power_lcd_off_cmd,
        .argtable = &power_lcd_off_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/* ------------------------------------------------- Sensor + Audio (shared) -- */
/* The sensors (LSM6DSOX/BMM350/SHT4X) and audio codecs (ES8311/ES7243E) share
 * the BSP_SEN_EN power rail and the I2C_1 bus, so they are powered off together. */
static struct {
    struct arg_end *end;
} power_sen_audio_off_args;

static const gpio_num_t sen_audio_pins[] = {
    BSP_ADC_I2S_MCLK,
    BSP_ADC_I2S_SCLK,
    BSP_ADC_I2S_LRLK,
    BSP_ADC_I2S_SDIN,
    BSP_DAC_I2S_SDOUT,
    BSP_I2C_1_SCL,
    BSP_I2C_1_SDA,
};

static int power_sen_audio_off_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&power_sen_audio_off_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, power_sen_audio_off_args.end, argv[0]);
        return 1;
    }

    ESP_LOGI(TAG, "Powering off sensor + audio...");

    esp_err_t ret = bsp_extra_player_del();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Player del: %s", esp_err_to_name(ret));
    }
    ret = bsp_extra_codec_dev_stop();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Codec stop: %s", esp_err_to_name(ret));
    }
    ret = bsp_extra_codec_deinit();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Codec deinit: %s", esp_err_to_name(ret));
    }

    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 0);   /* speaker PA off */
    bsp_exp_output_io_set_level(BSP_SEN_EN, 0);      /* sensor + codec rail off */

    /* codecs sit on I2C_1; free the bus after codec teardown, before pin reset */
    ret = bsp_i2c_1_deinit();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C_1 deinit: %s", esp_err_to_name(ret));
    }

    reset_pins(sen_audio_pins, sizeof(sen_audio_pins) / sizeof(sen_audio_pins[0]));

    ESP_LOGI(TAG, "Sensor + audio powered off (rails + pins safe)");
    return 0;
}

static void register_power_sen_audio_off_cmd(void)
{
    power_sen_audio_off_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "power_sen_audio_off",
        .help = "Power off sensor+audio: stop codec/player, cut SEN_EN+PA, free I2C_1, float pins",
        .hint = NULL,
        .func = &power_sen_audio_off_cmd,
        .argtable = &power_sen_audio_off_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/* --------------------------------------------------------------- Restart -- */
static struct {
    struct arg_end *end;
} power_restart_args;

static int power_restart_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&power_restart_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, power_restart_args.end, argv[0]);
        return 1;
    }

    ESP_LOGI(TAG, "Software restart...");
    /* small delay so the log line is flushed to the console before reset */
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();   /* does not return */
    return 0;
}

static void register_power_restart_cmd(void)
{
    power_restart_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "power_restart",
        .help = "Software restart the device (ESP32 system reset)",
        .hint = NULL,
        .func = &power_restart_cmd,
        .argtable = &power_restart_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/* --------------------------------------------------------------- aggregate */
void power_cmd_register_all(void)
{
    register_power_gnss_off_cmd();
    register_power_sd_off_cmd();
    register_power_lcd_off_cmd();
    register_power_sen_audio_off_cmd();
    register_power_restart_cmd();

    ESP_LOGI(TAG, "Power control commands registered:");
    ESP_LOGI(TAG, "  power_gnss_off         Power off GNSS domain");
    ESP_LOGI(TAG, "  power_sd_off           Power off SD card domain");
    ESP_LOGI(TAG, "  power_lcd_off          Power off LCD domain");
    ESP_LOGI(TAG, "  power_sen_audio_off    Power off sensor + audio domain");
    ESP_LOGI(TAG, "  power_restart          Software restart the device");
}
