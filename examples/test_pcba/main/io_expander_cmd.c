#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_console.h"
#include "driver/i2c_master.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp_err_check.h"
#include "esp_io_expander.h"
#include "esp_io_expander_tca6424.h"
#include "meshpager_x2.h"
#include "bsp_i2c_control.h"


static const char *TAG = "IO_EXPANDER_TEST";

// IO Expander set command arguments
static struct {
    struct arg_int *io;
    struct arg_int *level;
    struct arg_end *end;
} io_expander_set_args;

// IO Expander get command arguments
static struct {
    struct arg_int *io;
    struct arg_end *end;
} io_expander_get_args;

// IO Expander config command arguments
static struct {
    struct arg_int *io;
    struct arg_int *mode;
    struct arg_end *end;
} io_expander_config_args;



extern esp_io_expander_handle_t bsp_exp_io_get_handler( void );

/**
 * @brief Set IO expander pin level
 */
static int io_expander_set_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&io_expander_set_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, io_expander_set_args.end, argv[0]);
        return 1;
    }

    if (io_expander_set_args.io->count == 0) {
        ESP_LOGE(TAG, "IO number is required");
        return 1;
    }

    if (io_expander_set_args.level->count == 0) {
        ESP_LOGE(TAG, "Level is required (0 or 1)");
        return 1;
    }

    int io_num = io_expander_set_args.io->ival[0];
    int level = io_expander_set_args.level->ival[0];

    // Validate IO number (TCA6424 has 24 IOs)
    if (io_num < 0 || io_num >= 24) {
        ESP_LOGE(TAG, "Invalid IO number: %d (must be 0-23)", io_num);
        return 1;
    }

    // Validate level
    if (level != 0 && level != 1) {
        ESP_LOGE(TAG, "Invalid level: %d (must be 0 or 1)", level);
        return 1;
    }

    // Set IO level
    uint32_t pin_mask = (1ULL << io_num);
    bsp_exp_output_io_set_level(pin_mask, level);

    ESP_LOGI(TAG, "IO %d set to %d", io_num, level);
    return 0;
}

/**
 * @brief Get IO expander pin level
 */
static int io_expander_get_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&io_expander_get_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, io_expander_get_args.end, argv[0]);
        return 1;
    }

    if (io_expander_get_args.io->count == 0) {
        ESP_LOGE(TAG, "IO number is required");
        return 1;
    }

    int io_num = io_expander_get_args.io->ival[0];

    // Validate IO number
    if (io_num < 0 || io_num >= 24) {
        ESP_LOGE(TAG, "Invalid IO number: %d (must be 0-23)", io_num);
        return 1;
    }

    // Get IO level
    uint32_t pin_mask = (1ULL << io_num);
    uint32_t level_mask = 0;
    level_mask = bsp_exp_input_io_get_level(pin_mask);

    int level = (level_mask & pin_mask) ? 1 : 0;
    ESP_LOGI(TAG, "IO %d level = %d", io_num, level);

    return 0;
}

/**
 * @brief Configure IO expander pin direction
 */
static int io_expander_config_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&io_expander_config_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, io_expander_config_args.end, argv[0]);
        return 1;
    }

    if (io_expander_config_args.io->count == 0) {
        ESP_LOGE(TAG, "IO number is required");
        return 1;
    }

    if (io_expander_config_args.mode->count == 0) {
        ESP_LOGE(TAG, "Mode is required (0=input, 1=output)");
        return 1;
    }

    int io_num = io_expander_config_args.io->ival[0];
    int mode = io_expander_config_args.mode->ival[0];

    // Validate IO number
    if (io_num < 0 || io_num >= 24) {
        ESP_LOGE(TAG, "Invalid IO number: %d (must be 0-23)", io_num);
        return 1;
    }

    // Validate mode
    esp_io_expander_dir_t io_dir;
    switch (mode) {
        case 0:
            io_dir = IO_EXPANDER_INPUT;
            ESP_LOGI(TAG, "Mode: INPUT");
            break;
        case 1:
            io_dir = IO_EXPANDER_OUTPUT;
            ESP_LOGI(TAG, "Mode: OUTPUT");
            break;
        default:
            ESP_LOGE(TAG, "Invalid mode: %d (0=input, 1=output)", mode);
            return 1;
    }

    // Set direction
    uint32_t pin_mask = (1ULL << io_num);
    bsp_exp_io_set_dir(pin_mask, io_dir);

    ESP_LOGI(TAG, "IO %d configured successfully", io_num);
    return 0;
}

/**
 * @brief Print IO expander state
 */
static int io_expander_print_cmd(int argc, char **argv)
{
    esp_io_expander_handle_t io_expander = bsp_exp_io_get_handler();
    esp_err_t ret = esp_io_expander_print_state(io_expander);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to print IO expander state: %s", esp_err_to_name(ret));
        return 1;
    }

    return 0;
}

/**
 * @brief Register IO expander set command
 */
static void register_io_expander_set_cmd(void)
{
    io_expander_set_args.io = arg_int1("i", "io", "<n>", "IO pin number (0-23)");
    io_expander_set_args.level = arg_int1("l", "level", "<0|1>", "Set level (0 or 1)");
    io_expander_set_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "io_exp_set",
        .help = "Set IO expander pin level",
        .hint = NULL,
        .func = &io_expander_set_cmd,
        .argtable = &io_expander_set_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/**
 * @brief Register IO expander get command
 */
static void register_io_expander_get_cmd(void)
{
    io_expander_get_args.io = arg_int1("i", "io", "<n>", "IO pin number (0-23)");
    io_expander_get_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "io_exp_get",
        .help = "Get IO expander pin level",
        .hint = NULL,
        .func = &io_expander_get_cmd,
        .argtable = &io_expander_get_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/**
 * @brief Register IO expander config command
 */
static void register_io_expander_config_cmd(void)
{
    io_expander_config_args.io = arg_int1("i", "io", "<n>", "IO pin number (0-23)");
    io_expander_config_args.mode = arg_int1("m", "mode", "<0|1>", "IO mode (0=input, 1=output)");
    io_expander_config_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "io_exp_config",
        .help = "Configure IO expander pin direction",
        .hint = NULL,
        .func = &io_expander_config_cmd,
        .argtable = &io_expander_config_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/**
 * @brief Register IO expander print command
 */
static void register_io_expander_print_cmd(void)
{
    const esp_console_cmd_t cmd = {
        .command = "io_exp_print",
        .help = "Print IO expander state",
        .hint = NULL,
        .func = &io_expander_print_cmd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/**
 * @brief Register all IO expander test commands
 */
void io_expander_cmd_register_all(void)
{
    register_io_expander_set_cmd();
    register_io_expander_get_cmd();
    register_io_expander_config_cmd();
    register_io_expander_print_cmd();
    
    ESP_LOGI(TAG, "IO expander test commands registered:");
    ESP_LOGI(TAG, "  io_exp_config -i <io> -m <mode>            Configure IO");
    ESP_LOGI(TAG, "  io_exp_set    -i <io> -l <level>           Set IO level");
    ESP_LOGI(TAG, "  io_exp_get    -i <io>                      Get IO level");
    ESP_LOGI(TAG, "  io_exp_print                               Print IO state");
}