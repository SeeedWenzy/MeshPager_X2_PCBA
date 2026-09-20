#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_console.h"
#include "driver/gpio.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GPIO_TEST";

// GPIO set command arguments
static struct {
    struct arg_int *gpio;
    struct arg_int *level;
    struct arg_end *end;
} gpio_set_args;

// GPIO get command arguments
static struct {
    struct arg_int *gpio;
    struct arg_end *end;
} gpio_get_args;

// GPIO config command arguments
static struct {
    struct arg_int *gpio;
    struct arg_int *mode;
    struct arg_int *pull;
    struct arg_end *end;
} gpio_config_args;

/**
 * @brief Set GPIO pin level
 */
static int gpio_set_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&gpio_set_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, gpio_set_args.end, argv[0]);
        return 1;
    }

    if (gpio_set_args.gpio->count == 0) {
        ESP_LOGE(TAG, "GPIO number is required");
        return 1;
    }

    if (gpio_set_args.level->count == 0) {
        ESP_LOGE(TAG, "Level is required (0 or 1)");
        return 1;
    }

    int gpio_num = gpio_set_args.gpio->ival[0];
    int level = gpio_set_args.level->ival[0];

    // Validate GPIO number
    if (gpio_num < 0 || gpio_num >= GPIO_NUM_MAX) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d (must be 0-%d)", gpio_num, GPIO_NUM_MAX - 1);
        return 1;
    }

    // Validate level
    if (level != 0 && level != 1) {
        ESP_LOGE(TAG, "Invalid level: %d (must be 0 or 1)", level);
        return 1;
    }

    // Set GPIO level
    esp_err_t ret = gpio_set_level(gpio_num, level);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO %d level: %s", gpio_num, esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG, "GPIO %d set to %d", gpio_num, level);
    return 0;
}

/**
 * @brief Get GPIO pin level
 */
static int gpio_get_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&gpio_get_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, gpio_get_args.end, argv[0]);
        return 1;
    }

    if (gpio_get_args.gpio->count == 0) {
        ESP_LOGE(TAG, "GPIO number is required");
        return 1;
    }

    int gpio_num = gpio_get_args.gpio->ival[0];

    // Validate GPIO number
    if (gpio_num < 0 || gpio_num >= GPIO_NUM_MAX) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d (must be 0-%d)", gpio_num, GPIO_NUM_MAX - 1);
        return 1;
    }

    // Get GPIO level
    int level = gpio_get_level(gpio_num);
    ESP_LOGI(TAG, "GPIO %d level = %d", gpio_num, level);
    return 0;
}

/**
 * @brief Configure GPIO mode and pull-up/down
 */
static int gpio_config_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&gpio_config_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, gpio_config_args.end, argv[0]);
        return 1;
    }

    if (gpio_config_args.gpio->count == 0) {
        ESP_LOGE(TAG, "GPIO number is required");
        return 1;
    }

    if (gpio_config_args.mode->count == 0) {
        ESP_LOGE(TAG, "Mode is required (0=input, 1=output, 2=output_od)");
        return 1;
    }

    int gpio_num = gpio_config_args.gpio->ival[0];
    int mode = gpio_config_args.mode->ival[0];
    int pull = 0; // default: no pull

    if (gpio_config_args.pull->count) {
        pull = gpio_config_args.pull->ival[0];
    }

    // Validate GPIO number
    if (gpio_num < 0 || gpio_num >= GPIO_NUM_MAX) {
        ESP_LOGE(TAG, "Invalid GPIO number: %d (must be 0-%d)", gpio_num, GPIO_NUM_MAX - 1);
        return 1;
    }

    // Validate mode
    gpio_mode_t gpio_mode;
    switch (mode) {
        case 0:
            gpio_mode = GPIO_MODE_INPUT;
            ESP_LOGI(TAG, "Mode: INPUT");
            break;
        case 1:
            gpio_mode = GPIO_MODE_OUTPUT;
            ESP_LOGI(TAG, "Mode: OUTPUT");
            break;
        case 2:
            gpio_mode = GPIO_MODE_OUTPUT_OD;
            ESP_LOGI(TAG, "Mode: OUTPUT_OD");
            break;
        default:
            ESP_LOGE(TAG, "Invalid mode: %d (0=input, 1=output, 2=output_od)", mode);
            return 1;
    }

    // Configure pull-up/down
    gpio_pull_mode_t pull_mode = GPIO_FLOATING;
    switch (pull) {
        case 0:
            pull_mode = GPIO_FLOATING;
            ESP_LOGI(TAG, "Pull: FLOATING");
            break;
        case 1:
            pull_mode = GPIO_PULLUP_ONLY;
            ESP_LOGI(TAG, "Pull: PULLUP");
            break;
        case 2:
            pull_mode = GPIO_PULLDOWN_ONLY;
            ESP_LOGI(TAG, "Pull: PULLDOWN");
            break;
        case 3:
            pull_mode = GPIO_PULLUP_PULLDOWN;
            ESP_LOGI(TAG, "Pull: PULLUP_PULLDOWN");
            break;
        default:
            ESP_LOGE(TAG, "Invalid pull mode: %d (0=floating, 1=pullup, 2=pulldown, 3=both)", pull);
            return 1;
    }

    // Configure GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = gpio_mode,
        .pull_up_en = (pull_mode == GPIO_PULLUP_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN),
        .pull_down_en = (pull_mode == GPIO_PULLDOWN_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN),
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", gpio_num, esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG, "GPIO %d configured successfully", gpio_num);
    return 0;
}

/**
 * @brief Print configuration and level of all on-chip GPIOs
 *
 * Iterates over all usable GPIOs (skipping pins 22~25, 27~32 which are used
 * for SPI Flash / PSRAM on ESP32-S3), dumps each pin's direction / pull /
 * drive / interrupt config, and prints the current input level.
 */
static int gpio_print_cmd(int argc, char **argv)
{
    ESP_LOGI(TAG, "Dumping all on-chip GPIO configurations:");
    for (int i = 0; i < GPIO_NUM_MAX; i++) {
        // ESP32-S3: GPIO 22~25, 27~32 are used by SPI Flash / PSRAM, skip them
        if ((i >= 22 && i <= 25) || (i >= 27 && i <= 32)) {
            continue;
        }
        // Dump direction / pull / drive / interrupt configuration
        gpio_dump_io_configuration(stdout, BIT64(i));
        // Current input level
        ESP_LOGI(TAG, "GPIO[%d] level=%d", i, gpio_get_level(i));
    }
    return 0;
}

/**
 * @brief Register GPIO set command
 */
static void register_gpio_set_cmd(void)
{
    gpio_set_args.gpio = arg_int1("g", "gpio", "<n>", "GPIO pin number");
    gpio_set_args.level = arg_int1("l", "level", "<0|1>", "Set level (0 or 1)");
    gpio_set_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "gpio_set",
        .help = "Set GPIO pin level",
        .hint = NULL,
        .func = &gpio_set_cmd,
        .argtable = &gpio_set_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/**
 * @brief Register GPIO get command
 */
static void register_gpio_get_cmd(void)
{
    gpio_get_args.gpio = arg_int1("g", "gpio", "<n>", "GPIO pin number");
    gpio_get_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "gpio_get",
        .help = "Get GPIO pin level",
        .hint = NULL,
        .func = &gpio_get_cmd,
        .argtable = &gpio_get_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/**
 * @brief Register GPIO config command
 */
static void register_gpio_config_cmd(void)
{
    gpio_config_args.gpio = arg_int1("g", "gpio", "<n>", "GPIO pin number");
    gpio_config_args.mode = arg_int1("m", "mode", "<0|1|2>", "GPIO mode (0=input, 1=output, 2=output_od)");
    gpio_config_args.pull = arg_int0("p", "pull", "<0|1|2|3>", "Pull mode (0=floating, 1=pullup, 2=pulldown, 3=both)");
    gpio_config_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "gpio_config",
        .help = "Configure GPIO pin mode and pull-up/down",
        .hint = NULL,
        .func = &gpio_config_cmd,
        .argtable = &gpio_config_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/**
 * @brief Register GPIO print command
 */
static void register_gpio_print_cmd(void)
{
    const esp_console_cmd_t cmd = {
        .command = "gpio_print",
        .help = "Print configuration and level of all on-chip GPIOs",
        .hint = NULL,
        .func = &gpio_print_cmd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/**
 * @brief Register all GPIO test commands
 */
void gpio_cmd_register_all(void)
{
    register_gpio_set_cmd();
    register_gpio_get_cmd();
    register_gpio_config_cmd();
    register_gpio_print_cmd();

    ESP_LOGI(TAG, "GPIO test commands registered:");
    ESP_LOGI(TAG, "  gpio_config -g <gpio> -m <mode> [-p <pull>]  Configure GPIO");
    ESP_LOGI(TAG, "  gpio_set    -g <gpio> -l <level>            Set GPIO level");
    ESP_LOGI(TAG, "  gpio_get    -g <gpio>                      Get GPIO level");
    ESP_LOGI(TAG, "  gpio_print                                Print all GPIO state");
}