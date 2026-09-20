
#include "sdkconfig.h"

#include <inttypes.h>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "esp_ldo_regulator.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "esp_io_expander_tca6424.h"
#include "meshpager_x2.h"
#include "bsp_i2c_control.h"
#include "bsp_gnss_control.h"
#include "lr20xx_hal.h"
#include "lr20xx_init.h"
#include "ral.h"
#include "ralf.h"
#include "ralf_lr20xx.h"

#include "driver/usb_serial_jtag.h"
#include "driver/rtc_io.h"

static const char *TAG = "MeshPager_X2";

#define BSP_AUDIO_I2S_PORT I2S_NUM_0

sdmmc_card_t *bsp_sdcard = NULL;

esp_io_expander_handle_t io_expander = NULL;

static lr20xx_hal_context_t s_power_test_radio;
static const ralf_t s_power_test_modem_radio = RALF_LR20XX_INSTANTIATE(&s_power_test_radio);


static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;

static const audio_codec_data_if_t *i2s_adc_data_if = NULL;
static const audio_codec_data_if_t *i2s_dac_data_if = NULL;

static int adc_raw[10] = {0};
static TaskHandle_t exp_io_int_task_handle = NULL;
static TaskHandle_t button_event_task_handle = NULL;
static TaskHandle_t gpio_button_event_task_handle = NULL;
static TaskHandle_t power_button_task_handle = NULL;
static QueueHandle_t button_event_queue = NULL;
static TaskHandle_t rtc_int_task_handle = NULL;

static bool s_gpio_isr_service_installed = false;
static bsp_button_event_cb_t s_button_event_cb = NULL;
static void *s_button_event_cb_ctx = NULL;
static bsp_power_shutdown_cb_t s_power_shutdown_cb = NULL;
static void *s_power_shutdown_cb_ctx = NULL;

static bool power_on_state = false;
static bool power_on_control = false;
TickType_t pwr_press_start = 0;
TickType_t pwr_release_start = 0;
static uint8_t pwr_press_release_state = 0;



static uint32_t g_rtc_int_count = 0;

#define BSP_EXP_IO_INT_TASK_STACK_SIZE     3072
#define BSP_BUTTON_EVENT_TASK_STACK_SIZE   8192
#define BSP_POWER_BUTTON_TASK_STACK_SIZE   4096
#define BSP_RTC_INT_TASK_STACK_SIZE     1024


#define BSP_BUTTON_EVENT_QUEUE_LENGTH      8
#define BSP_SDMMC_MAX_FREQ_KHZ             40000

#define VBAT_ADC_RECHARGE_DELAY_MS    10
#define BSP_POWER_BUTTON_ACTIVE_LEVEL 1
#define BSP_POWER_BUTTON_POLL_MS      50
#define BSP_POWER_BUTTON_LONG_PRESS_MS 3000
#define BSP_POWER_BUTTON_RELEASE_ARM_MS 150
#define BSP_POWER_BUTTON_BOOT_DEBOUNCE_MS 120
#define ALL_IO_EXPANDER_OUTPUT_PIN_EXCEPT_HOLD (ALL_IO_EXPANDER_OUTPUT_PIN & ~BSP_PWR_HOLD)
#define BSP_POWER_STATE_NAMESPACE     "power"
#define BSP_POWER_STATE_KEY           "state"
#define BSP_POWER_STATE_OFF           0
#define BSP_POWER_STATE_ON            1
#define BSP_RETURN_BUTTON_EVENT_MASK  (1UL << 24)


static bool sdcard_is_mounted = false;

typedef struct {
    uint32_t mask;
    const char *name;
} bsp_button_desc_t;

typedef struct {
    uint32_t mask;
    bool is_pressed;
} bsp_button_event_t;

static const bsp_button_desc_t bsp_button_descs[] = {
    {BSP_BUTTON_UP, "UP"},
    {BSP_BUTTON_DOWN, "DOWN"},
    {BSP_BUTTON_LEFT, "LEFT"},
    {BSP_BUTTON_RIGHT, "RIGHT"},
    {BSP_BUTTON_CONFIRM, "CONFIRM"},
    {BSP_IMU_INT1, "IMU_INT"},
};

static bool vbat_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static esp_err_t bsp_power_button_gpio_init(void);
static esp_err_t bsp_return_button_gpio_init(void);
static esp_err_t bsp_gpio_install_isr_service_once(void);
static void bsp_gpio_button_isr_handler(void *arg);
static void bsp_gpio_button_event_task(void *arg);

static esp_err_t bsp_power_state_storage_init(void);
static esp_err_t bsp_power_state_get(bool *is_power_on);
static esp_err_t bsp_power_state_set(bool is_power_on);
static esp_err_t bsp_power_hold_enable(void);
static esp_err_t bsp_power_hold_disable(void);
static esp_err_t bsp_power_prepare_bootstrap(bool scan_i2c0);
static esp_err_t bsp_power_minimize_expander_outputs(void);
static esp_err_t bsp_power_test_lora_enter_sleep(void);
static esp_err_t bsp_power_configure_sleep_input(gpio_num_t gpio_num);
static esp_err_t bsp_power_apply_sleep_gpio_state(void);
static esp_err_t bsp_power_startup_peripherals(void);


static void bsp_power_button_isr_handler(void *arg);
static void bsp_power_button_task(void *arg);
static void bsp_power_shutdown_sequence(void);

static esp_err_t bsp_power_down_init(void);

#if 0
#define BSP_I2S_ADC_GPIO_CFG       \
    {                              \
        .mclk = BSP_ADC_I2S_MCLK,  \
        .bclk = BSP_ADC_I2S_SCLK,  \
        .ws = BSP_ADC_I2S_LRLK,    \
        .din = BSP_ADC_I2S_SDIN,   \
        .dout = BSP_ADC_I2S_SDOUT, \
        .invert_flags = {          \
            .mclk_inv = false,     \
            .bclk_inv = false,     \
            .ws_inv = false,       \
        },                         \
    }
#define BSP_I2S_DAC_GPIO_CFG       \
    {                              \
        .mclk = BSP_DAC_I2S_MCLK,  \
        .bclk = BSP_DAC_I2S_SCLK,  \
        .ws = BSP_DAC_I2S_LRLK,    \
        .din = BSP_DAC_I2S_SDIN,   \
        .dout = BSP_DAC_I2S_SDOUT, \
        .invert_flags = {          \
            .mclk_inv = false,     \
            .bclk_inv = false,     \
            .ws_inv = false,       \
        },                         \
    }

/* This configuration is used by default in `bsp_extra_audio_init()` */
#define BSP_I2S_ADC_DUPLEX_MONO_CFG(_sample_rate)                                                     \
    {                                                                                                   \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                            \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S_ADC_GPIO_CFG,                                                               \
    }

#define BSP_I2S_DAC_DUPLEX_MONO_CFG(_sample_rate)                                                     \
    {                                                                                                   \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                            \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S_DAC_GPIO_CFG,                                                               \
    }
#endif


#define BSP_I2S_GPIO_CFG           \
    {                              \
        .mclk = BSP_DAC_I2S_MCLK,      \
        .bclk = BSP_DAC_I2S_SCLK,      \
        .ws = BSP_DAC_I2S_LRLK,        \
        .din = BSP_ADC_I2S_SDIN,       \
        .dout = BSP_DAC_I2S_SDOUT,     \
        .invert_flags = {          \
            .mclk_inv = false,     \
            .bclk_inv = false,     \
            .ws_inv = false,       \
        },                         \
    }

/* This configuration is used by default in `bsp_extra_audio_init()` */
#if 0
#define BSP_I2S_DUPLEX_MONO_CFG(_sample_rate)                                                         \
    {                                                                                                   \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                            \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S_GPIO_CFG,                                                                   \
    }
#else
#define BSP_I2S_DUPLEX_MONO_CFG(_sample_rate)                                      \
    {                                                                              \
        .clk_cfg = {                                                               \
            .sample_rate_hz = (_sample_rate),                                      \
            .clk_src = I2S_CLK_SRC_DEFAULT,                                       \
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,                               \
        },                                                                         \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(                           \
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),                        \
        .gpio_cfg = BSP_I2S_GPIO_CFG,                                             \
    }
#endif


/* ------------------------------------------------ sdcard start ------------------------------------------------ */
esp_err_t bsp_sdcard_mount(void)
{
    bool heap_intact = heap_caps_check_integrity_all(true);
    ESP_LOGI(TAG, "SD mount enter: heap_intact=%d internal_free=%u internal_largest=%u",
             heap_intact ? 1 : 0,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    if(sdcard_is_mounted)
    {
        ESP_LOGW(TAG, "SD card is already mounted");
        return ESP_OK;
    }

    if(bsp_exp_input_io_get_level(BSP_SD_DETECT) == 1) {
        ESP_LOGW(TAG, "SD card is not detected");
        return ESP_FAIL;
    }
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        // Do NOT auto-format on mount failure. After an abnormal reboot the
        // card can look unformatted because its controller is wedged (not
        // because the FS is actually bad), and auto-format would silently
        // erase user data and then still fail. We retry with a power-cycle
        // below instead.
        .format_if_mount_failed = false,
        .max_files = 16,
        .allocation_unit_size = 64 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = BSP_SDMMC_MAX_FREQ_KHZ;
    sdmmc_slot_config_t slot_config = {
        .clk = BSP_SD_CLK,
        .cmd = BSP_SD_CMD,
        .d0  = BSP_SD_D0,
        .d1 = GPIO_NUM_NC,
        .d2 = GPIO_NUM_NC,
        .d3 = GPIO_NUM_NC,
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP,
        .width = 1,
        .flags = 0,
    };
    // Adjusting gpio drive capability
    // gpio_set_drive_capability(BSP_SD_CLK, GPIO_DRIVE_CAP_3);
    // gpio_set_drive_capability(BSP_SD_CMD, GPIO_DRIVE_CAP_3);
    // gpio_set_drive_capability(BSP_SD_D0, GPIO_DRIVE_CAP_3);   
    // sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    // slot_config.clk = BSP_SD_CLK;
    // slot_config.cmd = BSP_SD_CMD;
    // slot_config.d0 = BSP_SD_D0;
    // slot_config.d1 = GPIO_NUM_NC;
    // slot_config.d2 = GPIO_NUM_NC;
    // slot_config.d3 = GPIO_NUM_NC;
    // slot_config.width = 1;
    // slot_config.flags = 0;
    ESP_LOGI(TAG, "SD mount call: point=%s freq=%u kHz width=%u clk=%d cmd=%d d0=%d",
             BSP_SD_MOUNT_POINT,
             (unsigned)host.max_freq_khz,
             (unsigned)slot_config.width,
             (int)slot_config.clk,
             (int)slot_config.cmd,
             (int)slot_config.d0);
    heap_intact = heap_caps_check_integrity_all(true);
    ESP_LOGI(TAG, "SD mount before esp_vfs_fat_sdmmc_mount: heap_intact=%d internal_free=%u internal_largest=%u",
             heap_intact ? 1 : 0,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    /* Reset the SD bus GPIOs so leftover config from a previous run does not
     * interfere with the SDMMC host's own pin setup. */
    gpio_reset_pin(BSP_SD_CLK);
    gpio_reset_pin(BSP_SD_CMD);
    gpio_reset_pin(BSP_SD_D0);

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
    heap_intact = heap_caps_check_integrity_all(true);
    ESP_LOGI(TAG, "SD mount after esp_vfs_fat_sdmmc_mount: ret=%s heap_intact=%d internal_free=%u internal_largest=%u",
             esp_err_to_name(ret),
             heap_intact ? 1 : 0,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    /* If the first mount failed, the card may simply be wedged from a soft
     * reboot where BSP_SD_PWR_EN never dropped. Power-cycle the card once and
     * retry before giving up — do not fall back to formatting, which would
     * clobber user data on what is really just a transient init failure. */
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "first SD mount failed (%s), power-cycling card and retrying once", esp_err_to_name(ret));
        sdmmc_host_deinit_slot(SDMMC_HOST_SLOT_0);
        bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
        bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_reset_pin(BSP_SD_CLK);
        gpio_reset_pin(BSP_SD_CMD);
        gpio_reset_pin(BSP_SD_D0);
        ret = esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
        heap_intact = heap_caps_check_integrity_all(true);
        ESP_LOGI(TAG, "SD mount retry after power-cycle: ret=%s heap_intact=%d internal_free=%u internal_largest=%u",
                 esp_err_to_name(ret),
                 heap_intact ? 1 : 0,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }
    sdcard_is_mounted = true;

    return ret;
}

esp_err_t bsp_sdcard_unmount(void)
{
    ESP_LOGI(TAG, "SD unmount enter: heap_intact=%d internal_free=%u internal_largest=%u",
             heap_caps_check_integrity_all(true) ? 1 : 0,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    if(sdcard_is_mounted == false)
    {
        ESP_LOGW(TAG, "SD card is already unmounted");
        return ESP_OK;
    }
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
    }
    else
    {
        sdmmc_host_deinit_slot(SDMMC_HOST_SLOT_0);
        ESP_LOGI(TAG, "SD card unmounted successfully");
        sdcard_is_mounted = false;
        ESP_LOGI(TAG, "SD unmount done: heap_intact=%d internal_free=%u internal_largest=%u",
                 heap_caps_check_integrity_all(true) ? 1 : 0,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    }
    return ret;
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    BSP_ERROR_CHECK_RETURN_ERR(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

/* ------------------------------------------------ sdcard end ------------------------------------------------ */

/* ------------------------------------------------ audio start ------------------------------------------------ */
esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_adc_config, const i2s_std_config_t *i2s_dac_config)
{
    if (i2s_tx_chan && i2s_rx_chan) {
        /* Audio was initialized before */
        return ESP_OK;
    }

    /* Use one I2S controller for full-duplex audio so TX/RX share the same clocks. */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(BSP_AUDIO_I2S_PORT, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_chan, &i2s_rx_chan));

    // /* Setup I2S channels */
    // const i2s_std_config_t std_cfg_dav_default = BSP_I2S_DAC_DUPLEX_MONO_CFG(16000);
    // const i2s_std_config_t *p_i2s_dac_cfg = &std_cfg_dav_default;

    // const i2s_std_config_t std_cfg_adc_default = BSP_I2S_ADC_DUPLEX_MONO_CFG(16000);
    // const i2s_std_config_t *p_i2s_adc_cfg = &std_cfg_adc_default;

    /* Setup I2S channels */
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(16000);
    // i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(16000);   
    // std_cfg_default.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;  // I2S_MCLK_MULTIPLE_384 I2S_MCLK_MULTIPLE_128
    const i2s_std_config_t *p_i2s_cfg = &std_cfg_default;

    if (i2s_tx_chan != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx_chan, p_i2s_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx_chan));
    }

    if (i2s_rx_chan != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_rx_chan, p_i2s_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx_chan));
    }

    // gpio_set_drive_capability(BSP_DAC_I2S_MCLK, GPIO_DRIVE_CAP_0); 
    // gpio_set_drive_capability(BSP_DAC_I2S_SCLK, GPIO_DRIVE_CAP_0); 

    audio_codec_i2s_cfg_t i2s_cfg_dac = {
        .port = BSP_AUDIO_I2S_PORT,
        .rx_handle = NULL,
        .tx_handle = i2s_tx_chan,
    };
    i2s_dac_data_if = audio_codec_new_i2s_data(&i2s_cfg_dac);

    audio_codec_i2s_cfg_t i2s_cfg_adc = {
        .port = BSP_AUDIO_I2S_PORT,
        .rx_handle = i2s_rx_chan,
        .tx_handle = NULL,
    };
    i2s_adc_data_if = audio_codec_new_i2s_data(&i2s_cfg_adc);

    return ESP_OK;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    if (i2s_dac_data_if == NULL) {
        /* Configure I2S peripheral and Power Amplifier */
        ESP_ERROR_CHECK(bsp_audio_init(NULL, NULL));
    }
    assert(i2s_dac_data_if);

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    i2c_master_bus_handle_t audio_i2c_handle = bsp_i2c_1_get_handle();
    if (audio_i2c_handle == NULL) {
        ESP_LOGE(TAG, "Failed to get I2C_1 handle");
        return NULL;
    }

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_PORT_NUM_1,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = audio_i2c_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(i2c_ctrl_if);

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 3.3,
        .codec_dac_voltage = 3.3,
    };

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_TYPE_OUT,
        .pa_pin = -1,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain = gain,
    };
    const audio_codec_if_t *es8311_dev = es8311_codec_new(&es8311_cfg);
    assert(es8311_dev);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8311_dev,
        .data_if = i2s_dac_data_if,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    if (i2s_adc_data_if == NULL) {
        /* Configure I2S peripheral and Power Amplifier */
        ESP_ERROR_CHECK(bsp_audio_init(NULL, NULL));
    }
    assert(i2s_adc_data_if);

    i2c_master_bus_handle_t audio_i2c_handle = bsp_i2c_1_get_handle();
    if (audio_i2c_handle == NULL) {
        ESP_LOGE(TAG, "Failed to get I2C_1 handle");
        return NULL;
    }
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_PORT_NUM_1,
        .addr = (ES7243E_CODEC_ADDR << 1),
        .bus_handle = audio_i2c_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(i2c_ctrl_if);

    es7243e_codec_cfg_t es7243e_cfg = {
        .ctrl_if = i2c_ctrl_if,
    };
    const audio_codec_if_t *es7243e_dev = es7243e_codec_new(&es7243e_cfg);
    assert(es7243e_dev);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = es7243e_dev,
        .data_if = i2s_adc_data_if,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}

/* ------------------------------------------------ audio end ------------------------------------------------ */

/* ------------------------------------------------ tft lcd start ------------------------------------------------ */
void lcd_bl_init(uint8_t brightness)
{
    if(brightness > 99) {
        brightness=99;
    } else if( brightness < 1) {
        brightness=1;
    }

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           =  (8192 - 1) * brightness/ 100, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
void lcd_bl_set(int brightness )
{   
    if(brightness > 99) {
        brightness=99;
    } else if( brightness < 1) {
        brightness=1;
    }

    uint32_t duty =(uint32_t) (8192 - 1) * brightness / 100.0;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

void lcd_bl_on(void)
{
    lcd_bl_set(100);
}

void lcd_bl_off(void)
{
    ledc_stop( LEDC_MODE, LEDC_CHANNEL, 0);
}

esp_err_t bsp_display_brightness_set(uint8_t level)
{
    esp_err_t ret = ESP_OK;
    lcd_bl_set(level);
    return ret;
}
/* ------------------------------------------------ tft lcd end ------------------------------------------------ */
/* ------------------------------------------------ vbat start ------------------------------------------------ */
void bsp_bat_power_control(bool on)
{
    esp_io_expander_set_level(io_expander, BSP_BAT_ADC_EN, on ? BSP_BAT_ADC_EN_ACTIVE_LEVEL : !BSP_BAT_ADC_EN_ACTIVE_LEVEL);
}

/* ADC1 unit + calibration are created ONCE and reused across every caller.
 * The previous design created/deleted the unit on each read, which (a) races
 * when several tasks call this concurrently — surfacing as "adc1 is already in
 * use" and aborting — and (b) is fragile in a tight loop. The mutex serializes
 * the lazy one-time init and the multi-sample read. */
static StaticSemaphore_t s_vbat_mutex_buf;
static SemaphoreHandle_t s_vbat_mutex;
static portMUX_TYPE s_vbat_mutex_init_lock = portMUX_INITIALIZER_UNLOCKED;

uint16_t bsp_battery_voltage_read(void)
{
    static adc_oneshot_unit_handle_t s_adc_handle;
    static adc_cali_handle_t s_cali_handle;
    static adc_channel_t s_adc_channel = VBAT_ADC1_CHAN0;
    static bool s_cali_ok = false;
    static bool s_inited = false;
    int64_t voltage_sum = 0;
    uint8_t valid_sample_count = 0;

    /* Race-free lazy mutex creation: xSemaphoreCreateMutexStatic uses the static
     * buffer (no heap alloc), so it is safe inside a critical section. */
    portENTER_CRITICAL(&s_vbat_mutex_init_lock);
    if (s_vbat_mutex == NULL) {
        s_vbat_mutex = xSemaphoreCreateMutexStatic(&s_vbat_mutex_buf);
    }
    portEXIT_CRITICAL(&s_vbat_mutex_init_lock);
    if (s_vbat_mutex == NULL) {
        return 0;
    }

    xSemaphoreTake(s_vbat_mutex, portMAX_DELAY);

    if (!s_inited) {
        adc_unit_t adc_unit = ADC_UNIT_1;
        adc_channel_t adc_channel = VBAT_ADC1_CHAN0;
        esp_err_t map_ret = adc_oneshot_io_to_channel(BSP_VBAT_ADC, &adc_unit, &adc_channel);
        if (map_ret != ESP_OK) {
            ESP_LOGW(TAG, "VBAT gpio %d channel lookup failed, fallback to unit=%d channel=%d: %s",
                     BSP_VBAT_ADC, adc_unit, adc_channel, esp_err_to_name(map_ret));
        }

        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = adc_unit,
        };
        esp_err_t r = adc_oneshot_new_unit(&init_config, &s_adc_handle);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(r));
            xSemaphoreGive(s_vbat_mutex);
            return 0;
        }

        adc_oneshot_chan_cfg_t config = {
            .atten = VBAT_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, adc_channel, &config));

        s_cali_ok = vbat_adc_calibration_init(adc_unit, adc_channel, VBAT_ADC_ATTEN, &s_cali_handle);
        s_adc_channel = adc_channel;
        s_inited = true;
    }

    /* High-impedance battery divider nodes can droop when sampled repeatedly.
     * Prime the ADC once, then keep the highest calibrated result across a few spaced samples. */
    ESP_ERROR_CHECK(adc_oneshot_read(s_adc_handle, s_adc_channel, &adc_raw[0]));
    vTaskDelay(pdMS_TO_TICKS(VBAT_ADC_RECHARGE_DELAY_MS));

    for (uint8_t i = 0; i < VBAT_ADC_SAMPLE_COUNT; i++) {
    int raw = 0;

        esp_err_t err = adc_oneshot_read(s_adc_handle,s_adc_channel,&raw);

        if (err != ESP_OK) {
            ESP_LOGE(TAG,"VBAT ADC read failed at sample %u: %s",i,esp_err_to_name(err));
            xSemaphoreGive(s_vbat_mutex);
            return 0;
        }

        if (s_cali_ok && s_cali_handle != NULL) {
            int voltage_mv = 0;

            err = adc_cali_raw_to_voltage(s_cali_handle,raw,&voltage_mv);

            if (err != ESP_OK) {
                ESP_LOGW(TAG,"VBAT ADC calibration failed at sample %u: %s",i,esp_err_to_name(err));
                xSemaphoreGive(s_vbat_mutex);
                return 0;
            }
            
            if (i >= VBAT_ADC_DISCARD_COUNT) {
                voltage_sum += voltage_mv;
                valid_sample_count++;
            }
        }

        if ((i + 1) < VBAT_ADC_SAMPLE_COUNT) {
            vTaskDelay(pdMS_TO_TICKS(VBAT_ADC_RECHARGE_DELAY_MS));
        }
    }
    if (s_cali_ok && s_cali_handle != NULL) {
      adc_cali_delete_scheme_curve_fitting(s_cali_handle);
    }

    if (s_adc_handle != NULL) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        s_inited = false;
    }
    xSemaphoreGive(s_vbat_mutex);

    if (!s_cali_ok || valid_sample_count == 0) {
        ESP_LOGW(TAG,
                "VBAT ADC measurement invalid: calibration=%d valid_samples=%u",
                s_cali_ok,
                valid_sample_count);
        return 0;
    }

    int voltage_sense_mv = (int)(voltage_sum / valid_sample_count);

    uint16_t voltage = (uint16_t)(((uint64_t)voltage_sense_mv *BSP_VBAT_DIVIDER_NUMERATOR) /BSP_VBAT_DIVIDER_DENOMINATOR);
    ESP_LOGI(TAG,
            "VBAT channel=%d samples=%d discard=%d sense_avg=%d mV battery=%u mV",
            s_adc_channel,
            VBAT_ADC_SAMPLE_COUNT,
            VBAT_ADC_DISCARD_COUNT,
            voltage_sense_mv,
            voltage);

    return voltage;
}

static bool vbat_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

/* ------------------------------------------------ vbat end ------------------------------------------------ */

/* ------------------------------------------------ expander io start ------------------------------------------------ */
static void bsp_log_button_events(uint32_t prev_levels, uint32_t curr_levels)
{
    for (size_t i = 0; i < sizeof(bsp_button_descs) / sizeof(bsp_button_descs[0]); ++i) {
        uint32_t mask = bsp_button_descs[i].mask;

        if ((prev_levels & mask) == (curr_levels & mask)) {
            continue;
        }

        bool is_pressed = (curr_levels & mask) == 0;
        ESP_LOGI(TAG, "button %s %s", bsp_button_descs[i].name, is_pressed ? "pressed" : "released");
        if (button_event_queue != NULL) {
            bsp_button_event_t event = {
                .mask = mask,
                .is_pressed = is_pressed,
            };

            if (xQueueSend(button_event_queue, &event, 0) != pdTRUE) {
                ESP_LOGW(TAG, "button event queue full, drop mask=%" PRIx32 " pressed=%d", mask, is_pressed);
            }
        }
    }
}

static void bsp_button_event_task(void *arg)
{
    bsp_button_event_t event;

    (void)arg;

    while (true) {
        if (xQueueReceive(button_event_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (s_button_event_cb != NULL) {
            s_button_event_cb(event.mask, event.is_pressed, s_button_event_cb_ctx);
        }
    }
}

static void bsp_log_gpio_button_event(bool is_pressed)
{
    ESP_LOGI(TAG, "button RETURN %s", is_pressed ? "pressed" : "released");

    if (button_event_queue != NULL) {
        bsp_button_event_t event = {
            .mask = BSP_RETURN_BUTTON_EVENT_MASK,
            .is_pressed = is_pressed,
        };
        if (xQueueSend(button_event_queue, &event, 0) != pdTRUE) {
            ESP_LOGW(TAG, "button event queue full, drop RETURN pressed=%d", is_pressed);
        }
    }
}

static void bsp_gpio_button_event_task(void *arg)
{
    int level;
    bool pressed;
    bool previous_pressed;

    (void)arg;
    level = gpio_get_level(BSP_BUTTON_RETURN_GPIO);
    previous_pressed = (level == 0);

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(20));
        level = gpio_get_level(BSP_BUTTON_RETURN_GPIO);
        pressed = (level == 0);
        if (pressed != previous_pressed) {
            bsp_log_gpio_button_event(pressed);
            previous_pressed = pressed;
        }
    }
}

static void IRAM_ATTR bsp_gpio_button_isr_handler(void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;
    (void)arg;

    if (gpio_button_event_task_handle != NULL) {
        vTaskNotifyGiveFromISR(gpio_button_event_task_handle, &high_task_wakeup);
    }
    if (high_task_wakeup == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}


static void bsp_exp_io_int_task(void *arg)
{
    uint32_t pin_levels = 0;
    uint32_t prev_button_levels = ALL_IO_EXPANDER_INPUT_PIN;
    static uint32_t last_pin_levels = ALL_IO_EXPANDER_INPUT_PIN;
    uint32_t pin_change = 0;
    if (io_expander != NULL) {
        esp_err_t ret = esp_io_expander_get_level(io_expander, ALL_IO_EXPANDER_INPUT_PIN, &pin_levels);
        if (ret == ESP_OK) {
            prev_button_levels = pin_levels;
        }
    }

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (io_expander == NULL) {
            continue;
        }

        esp_err_t ret = esp_io_expander_get_level(io_expander, ALL_IO_EXPANDER_INPUT_PIN, &pin_levels);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "io expander get levels failed: %s", esp_err_to_name(ret));
            continue;
        }

        bsp_log_button_events(prev_button_levels, pin_levels);
        prev_button_levels = pin_levels;
        pin_change = last_pin_levels ^ pin_levels;
        if (pin_change & BSP_CHARGER_STAT) 
        {
            if (pin_levels & BSP_CHARGER_STAT) {
                // Charger status changed to high
            } else {
                // Charger status changed to low
            }
        }
        else
        {
            ESP_LOGI(TAG, "io expander get levels: %" PRIx32, pin_levels);
        }
        last_pin_levels = pin_levels;
    }
}

static void IRAM_ATTR bsp_exp_io_int_isr_handler(void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;

    if (exp_io_int_task_handle != NULL) {
        vTaskNotifyGiveFromISR(exp_io_int_task_handle, &high_task_wakeup);
    }

    if (high_task_wakeup == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void bsp_rtc_io_int_task(void *arg)
{
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        g_rtc_int_count++;
        // ESP_LOGI(TAG, "YSN8900E RTC INT ....");
    }
}

uint32_t get_rtc_int_count( void )
{
    return g_rtc_int_count;
}

static void IRAM_ATTR bsp_rtc_int_isr_handler(void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;

    if (rtc_int_task_handle != NULL) {
        vTaskNotifyGiveFromISR(rtc_int_task_handle, &high_task_wakeup);
    }

    if (high_task_wakeup == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void bsp_exp_output_io_set_level(uint32_t expander_io, uint8_t level)
{
    esp_err_t ret = esp_io_expander_set_level(io_expander, expander_io, level);   
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "io expander set level failed: %s", esp_err_to_name(ret));
        return;
    }
}

uint32_t bsp_exp_input_io_get_level(uint32_t expander_io)
{
    uint32_t level = 0;
    esp_err_t ret = esp_io_expander_get_level(io_expander, expander_io, &level);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "io expander get levels failed: %s", esp_err_to_name(ret));
        return 0;
    }
    return level;
}

void bsp_exp_io_set_dir(uint32_t pin_num_mask, uint8_t direction)
{
    esp_io_expander_set_dir(io_expander, pin_num_mask, (esp_io_expander_dir_t)direction);
}

esp_err_t bsp_button_event_callback_register(bsp_button_event_cb_t callback, void *user_ctx)
{
    s_button_event_cb = callback;
    s_button_event_cb_ctx = user_ctx;
    return ESP_OK;
}

esp_err_t bsp_power_shutdown_callback_register(bsp_power_shutdown_cb_t callback, void *user_ctx)
{
    s_power_shutdown_cb = callback;
    s_power_shutdown_cb_ctx = user_ctx;
    return ESP_OK;
}

esp_io_expander_handle_t bsp_exp_io_get_handler( void )
{  
    return io_expander;
}

static esp_err_t bsp_init_io_expander(i2c_master_bus_handle_t i2c_handle)
{
    static const uint32_t tca6424_addr_candidates[] = {
        ESP_IO_EXPANDER_I2C_TCA6424_ADDRESS_GND,
        ESP_IO_EXPANDER_I2C_TCA6424_ADDRESS_VCC,
    };

    esp_err_t last_err = ESP_ERR_NOT_FOUND;

    for (size_t i = 0; i < sizeof(tca6424_addr_candidates) / sizeof(tca6424_addr_candidates[0]); ++i) {
        uint32_t dev_addr = tca6424_addr_candidates[i];

        ESP_LOGI(TAG, "try init io expander at 0x%02" PRIx32, dev_addr);
        last_err = esp_io_expander_new_i2c_tca6424(i2c_handle, dev_addr, &io_expander);
        if (last_err == ESP_OK) {
            ESP_LOGI(TAG, "io expander init ok, addr=0x%02" PRIx32, dev_addr);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "io expander init failed at 0x%02" PRIx32 ": %s", dev_addr, esp_err_to_name(last_err));
        io_expander = NULL;
    }

    return last_err;
}
esp_err_t bsp_deinit_io_expander( void )
{
    esp_err_t Err = ESP_ERR_NOT_FOUND;
    if (io_expander != NULL) 
    {
        esp_io_expander_del(io_expander);
        io_expander = NULL;
        Err = ESP_OK;
    }
    return Err;
}

static esp_err_t bsp_power_button_gpio_init(void)
{
    const gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BSP_BUTTON_ONOFF,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    BSP_ERROR_CHECK_RETURN_ERR(ret);

    ret = bsp_gpio_install_isr_service_once();
    BSP_ERROR_CHECK_RETURN_ERR(ret);

    ret = gpio_isr_handler_add(BSP_BUTTON_ONOFF, bsp_power_button_isr_handler, (void *)BSP_BUTTON_ONOFF);
    BSP_ERROR_CHECK_RETURN_ERR(ret);

    if (power_button_task_handle == NULL) {
        BaseType_t power_task_ret = xTaskCreate(
            bsp_power_button_task,
            "pwr_btn",
            BSP_POWER_BUTTON_TASK_STACK_SIZE,
            NULL,
            10,
            &power_button_task_handle);
        BSP_ERROR_CHECK_RETURN_ERR(power_task_ret == pdPASS ? ESP_OK : ESP_FAIL);
    }
    return ESP_OK;
}

static esp_err_t bsp_return_button_gpio_init(void)
{
    const gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BSP_BUTTON_RETURN_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    BSP_ERROR_CHECK_RETURN_ERR(ret);

    ret = bsp_gpio_install_isr_service_once();
    BSP_ERROR_CHECK_RETURN_ERR(ret);
    ret = gpio_isr_handler_add(BSP_BUTTON_RETURN_GPIO, bsp_gpio_button_isr_handler,
                               (void *)BSP_BUTTON_RETURN_GPIO);
    BSP_ERROR_CHECK_RETURN_ERR(ret);

    if (gpio_button_event_task_handle == NULL) {
        BaseType_t task_ret = xTaskCreate(
            bsp_gpio_button_event_task,
            "return_btn",
            4096,
            NULL,
            9,
            &gpio_button_event_task_handle);
        BSP_ERROR_CHECK_RETURN_ERR(task_ret == pdPASS ? ESP_OK : ESP_FAIL);
    }
    ESP_LOGI(TAG, "RETURN button initialized on GPIO%d, active low", BSP_BUTTON_RETURN_GPIO);
    return ESP_OK;
}

static esp_err_t bsp_gpio_install_isr_service_once(void)
{
    if (s_gpio_isr_service_installed) {
        return ESP_OK;
    }

    esp_err_t ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        s_gpio_isr_service_installed = true;
        return ESP_OK;
    }

    return ret;
}

static esp_err_t bsp_power_state_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        BSP_ERROR_CHECK_RETURN_ERR(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

static esp_err_t bsp_power_state_get(bool *is_power_on)
{
    nvs_handle_t nvs_handle = 0;
    uint8_t state = BSP_POWER_STATE_ON;
    esp_err_t ret = ESP_OK;

    BSP_ERROR_CHECK_RETURN_ERR(bsp_power_state_storage_init());
    BSP_ERROR_CHECK_RETURN_ERR(nvs_open(BSP_POWER_STATE_NAMESPACE, NVS_READWRITE, &nvs_handle));

    ret = nvs_get_u8(nvs_handle, BSP_POWER_STATE_KEY, &state);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        state = BSP_POWER_STATE_ON;
        ret = nvs_set_u8(nvs_handle, BSP_POWER_STATE_KEY, state);
        if (ret != ESP_OK) {
            nvs_close(nvs_handle);
            return ret;
        }

        ret = nvs_commit(nvs_handle);
        if (ret != ESP_OK) {
            nvs_close(nvs_handle);
            return ret;
        }

        ret = ESP_OK;
    }
    if (ret != ESP_OK) {
        nvs_close(nvs_handle);
        return ret;
    }

    *is_power_on = (state == BSP_POWER_STATE_ON);
    nvs_close(nvs_handle);
    return ESP_OK;
}

static esp_err_t bsp_power_state_set(bool is_power_on)
{
    nvs_handle_t nvs_handle = 0;
    uint8_t state = is_power_on ? BSP_POWER_STATE_ON : BSP_POWER_STATE_OFF;

    BSP_ERROR_CHECK_RETURN_ERR(bsp_power_state_storage_init());
    BSP_ERROR_CHECK_RETURN_ERR(nvs_open(BSP_POWER_STATE_NAMESPACE, NVS_READWRITE, &nvs_handle));

    esp_err_t ret = nvs_set_u8(nvs_handle, BSP_POWER_STATE_KEY, state);
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return ret;
}

static esp_err_t bsp_power_hold_enable(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(io_expander, BSP_PWR_HOLD, IO_EXPANDER_OUTPUT));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, BSP_PWR_HOLD, 1));
    ESP_LOGI(TAG, "BSP_PWR_HOLD asserted");
    return ESP_OK;
}

static esp_err_t bsp_power_hold_disable(void)
{
    if (io_expander == NULL) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "BSP_PWR_HOLD deasserted");

    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(io_expander, BSP_PWR_HOLD, IO_EXPANDER_OUTPUT));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, BSP_PWR_HOLD, 0));

    return ESP_OK;
}

static esp_err_t bsp_power_prepare_bootstrap(bool scan_i2c0)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_0_init());
    if (scan_i2c0) {
        ESP_LOGI(TAG, "scan I2C_0 after init");
        BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_0_scan());
    }

    if (io_expander == NULL) {
        i2c_master_bus_handle_t tca6424_i2c_handle = bsp_i2c_0_get_handle();
        esp_err_t ret = bsp_init_io_expander(tca6424_i2c_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "io expander init fail");
            return ret;
        }
    }
    return ESP_OK;
}

static esp_err_t bsp_power_minimize_expander_outputs(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(io_expander, ALL_IO_EXPANDER_OUTPUT_PIN_EXCEPT_HOLD, IO_EXPANDER_OUTPUT));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, ALL_IO_EXPANDER_OUTPUT_PIN_EXCEPT_HOLD, 0));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(io_expander, ALL_IO_EXPANDER_INPUT_PIN, IO_EXPANDER_INPUT));

    ESP_LOGI(TAG, "IO expander outputs minimized, only BSP_PWR_HOLD remains asserted");
    return ESP_OK;
}

static esp_err_t bsp_power_test_lora_enter_sleep(void)
{
    ral_status_t status;

    lr20xx_init(&s_power_test_radio);

    // debug 
    vTaskDelay(pdMS_TO_TICKS(10));

    status = ral_reset(&(s_power_test_modem_radio.ral));
    ESP_RETURN_ON_FALSE(status == RAL_STATUS_OK, ESP_FAIL, TAG, "lora ral_reset failed: %u", status);

    status = ral_init(&(s_power_test_modem_radio.ral));
    ESP_RETURN_ON_FALSE(status == RAL_STATUS_OK, ESP_FAIL, TAG, "lora ral_init failed: %u", status);

    status = ral_set_sleep(&(s_power_test_modem_radio.ral), true);
    ESP_RETURN_ON_FALSE(status == RAL_STATUS_OK, ESP_FAIL, TAG, "lora ral_set_sleep failed: %u", status);

    ESP_LOGI(TAG, "LoRa radio entered sleep mode");
    return ESP_OK;
}

static esp_err_t bsp_power_configure_sleep_input(gpio_num_t gpio_num)
{
    if (gpio_num == GPIO_NUM_NC || !GPIO_IS_VALID_GPIO(gpio_num)) {
        return ESP_OK;
    }

    const gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    BSP_ERROR_CHECK_RETURN_ERR(gpio_config(&io_conf));
    BSP_ERROR_CHECK_RETURN_ERR(gpio_sleep_set_direction(gpio_num, GPIO_MODE_INPUT));
    BSP_ERROR_CHECK_RETURN_ERR(gpio_sleep_set_pull_mode(gpio_num, GPIO_FLOATING));
    BSP_ERROR_CHECK_RETURN_ERR(gpio_sleep_sel_en(gpio_num));

    return ESP_OK;
}

static esp_err_t bsp_power_apply_sleep_gpio_state(void)
{
    static const gpio_num_t sleep_input_gpios[] = {
        BSP_LORA_SPI_CS,
        BSP_LORA_SPI_SCK,
        BSP_LORA_SPI_MOSI,
        BSP_LORA_SPI_MISO,
        BSP_BUTTON_ONOFF,
        BSP_LORA_INT,
        BSP_LORA_BUSY,
        BSP_LCD_CS,
        BSP_LCD_SDA,
        BSP_LCD_SCK,
        BSP_LCD_A0,
        BSP_SD_CMD,
        BSP_SD_CLK,
        BSP_SD_D0,
        BSP_VBAT_ADC,
        BSP_I2C_0_SDA,
        BSP_I2C_0_SCL,
        BSP_EXP_IO_INT,
        BSP_GNSS_PPS0,
        BSP_ADC_I2S_MCLK,
        BSP_ADC_I2S_SCLK,
        BSP_ADC_I2S_LRLK,
        BSP_DAC_I2S_SDOUT,
        BSP_ADC_I2S_SDIN,
        BSP_LCD_PWM,
        BSP_GNSS_RX,
        BSP_GNSS_TX,
        BSP_I2C_1_SDA,
        BSP_I2C_1_SCL,
    };

    for (size_t i = 0; i < sizeof(sleep_input_gpios) / sizeof(sleep_input_gpios[0]); ++i) {
        BSP_ERROR_CHECK_RETURN_ERR(bsp_power_configure_sleep_input(sleep_input_gpios[i]));
    }

    ESP_LOGI(TAG, "GPIO sleep state applied for low-leakage deep sleep");
    return ESP_OK;
}

static esp_err_t bsp_power_startup_peripherals(void)
{
    // Set all expander IO as output and low level at the beginning.
    // TCA6424 output latches retain their state across an ESP32 software
    // reset (panic / watchdog / esp_restart()), so on an abnormal reboot
    // every controlled power rail may still be high. Drive them all low first
    // so every boot starts from the same known (discharged) state as a cold
    // boot, then enable rails one by one below with proper settle times.
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(io_expander, ALL_IO_EXPANDER_OUTPUT_PIN_EXCEPT_HOLD, IO_EXPANDER_OUTPUT));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, ALL_IO_EXPANDER_OUTPUT_PIN_EXCEPT_HOLD, 0));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(io_expander, ALL_IO_EXPANDER_INPUT_PIN, IO_EXPANDER_INPUT));
    ESP_LOGI(TAG, "ALL_IO_EXPANDER_OUTPUT_PIN at 0x%08" PRIx32, (uint32_t)ALL_IO_EXPANDER_OUTPUT_PIN_EXCEPT_HOLD);
    ESP_LOGI(TAG, "ALL_IO_EXPANDER_INPUT_PIN at 0x%08" PRIx32, (uint32_t)ALL_IO_EXPANDER_INPUT_PIN);
    vTaskDelay(pdMS_TO_TICKS(50)); // let the various VDD rails decay to 0

    // Battery
    bsp_bat_power_control(false);
    vTaskDelay(pdMS_TO_TICKS(10));

    // GNSS
    bsp_gnss_power_init();

    // Sensor
    bsp_exp_output_io_set_level(BSP_SEN_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // SD Card — power-cycle first so a soft-reboot / crash does not leave the
    // card's controller wedged (BSP_SD_PWR_EN is on the TCA6424, which keeps
    // its output latch across an ESP32 software reset). The card needs a
    // clean VDD 0->1 cold start + CMD0 GO_IDLE handshake to come back.
    bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(250)); // SD VDD must fully discharge to 0
    bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // VDD ramp + card internal power-on sequence

    // PA_POWER
    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // LCD
    bsp_exp_output_io_set_level(BSP_LCD_PWR_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    bsp_exp_output_io_set_level(BSP_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // LORA
    bsp_exp_output_io_set_level(BSP_LORA_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_config_t io_in_conf = {
        .pull_up_en = 1,
        .pull_down_en = 0,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = 1ULL << BSP_EXP_IO_INT,
    };
    
    if (button_event_queue == NULL) {
        button_event_queue = xQueueCreate(BSP_BUTTON_EVENT_QUEUE_LENGTH, sizeof(bsp_button_event_t));
        BSP_ERROR_CHECK_RETURN_ERR(button_event_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    }

    if (button_event_task_handle == NULL) {
        BaseType_t button_task_ret = xTaskCreate(
            bsp_button_event_task,
            "btn_evt",
            BSP_BUTTON_EVENT_TASK_STACK_SIZE,
            NULL,
            9,
            &button_event_task_handle);
        BSP_ERROR_CHECK_RETURN_ERR(button_task_ret == pdPASS ? ESP_OK : ESP_FAIL);
    }

    if (exp_io_int_task_handle == NULL) {
        BaseType_t task_ret = xTaskCreate(
            bsp_exp_io_int_task,
            "exp_io_int",
            BSP_EXP_IO_INT_TASK_STACK_SIZE,
            NULL,
            10,
            &exp_io_int_task_handle);
        BSP_ERROR_CHECK_RETURN_ERR(task_ret == pdPASS ? ESP_OK : ESP_FAIL);
    }

    esp_err_t ret = gpio_config(&io_in_conf);
    BSP_ERROR_CHECK_RETURN_ERR(ret);
    ret = bsp_gpio_install_isr_service_once();
    BSP_ERROR_CHECK_RETURN_ERR(ret);

    ret = gpio_isr_handler_add(BSP_EXP_IO_INT, bsp_exp_io_int_isr_handler, (void *)BSP_EXP_IO_INT);
    BSP_ERROR_CHECK_RETURN_ERR(ret);


    const gpio_config_t int_gpio_config = {
        .pull_up_en = 1,
        .pull_down_en = 0,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE,         // GPIO_INTR_NEGEDGE GPIO_INTR_POSEDGE
        .pin_bit_mask = 1ULL << BSP_RTC_OUT_INT,
    };

    if (rtc_int_task_handle == NULL) {
        BaseType_t task_ret = xTaskCreate(
            bsp_rtc_io_int_task,
            "rtc_io_int",
            BSP_RTC_INT_TASK_STACK_SIZE,
            NULL,
            4,
            &rtc_int_task_handle);
        BSP_ERROR_CHECK_RETURN_ERR(task_ret == pdPASS ? ESP_OK : ESP_FAIL);
    }

    ret = gpio_config(&int_gpio_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_isr_handler_add(BSP_RTC_OUT_INT, bsp_rtc_int_isr_handler, (void *)BSP_RTC_OUT_INT);
    if (ret != ESP_OK) {
        return ret;
    }

    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_1_init());
    ESP_LOGI(TAG, "scan I2C_1 after init");
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_1_scan());

    return ESP_OK;
}

static void bsp_power_shutdown_sequence(void)
{
    ESP_LOGI(TAG, "power key long press detected, shutting down board");

    if (s_power_shutdown_cb != NULL) {
        s_power_shutdown_cb(s_power_shutdown_cb_ctx);
    }

    esp_err_t ret = bsp_power_down_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "board power-down cleanup returned: %s", esp_err_to_name(ret));
    }

    ret = bsp_power_hold_disable();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to drop power hold: %s", esp_err_to_name(ret));
    }
}

static void bsp_power_button_task(void *arg)
{
    (void)arg;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "pwr_press_start = %d, pwr_release_start=%d", pwr_press_start, pwr_release_start);
        // Press down
        if (pwr_press_release_state == 1) {
            bsp_power_hold_enable();
        } 
        if((pwr_release_start&&pwr_press_start) && (pwr_release_start > pwr_press_start) && (pwr_release_start - pwr_press_start) >= pdMS_TO_TICKS(BSP_POWER_BUTTON_LONG_PRESS_DURATION_MS)) {
            pwr_release_start = 0;
            pwr_press_start = 0;
            ESP_LOGI(TAG, "long press detected");
            // current is poweron,turn off
            if(power_on_state) {
                bsp_power_state_set(false);
                power_on_state = false;
                ESP_LOGI(TAG, "Power button long pressed but already in power off state, ignore");
                bsp_power_shutdown_sequence();
                esp_restart();
            }
            else // current is poweroff, poweron
            {
                power_on_control = true;
                ESP_LOGI(TAG, "Long press 3s, the device is power on................");
                bsp_power_state_set(true);
                power_on_state = true;
            }
        }
        else if(pwr_release_start&&pwr_press_start)
        {
            pwr_release_start = 0;
            pwr_press_start = 0;
            if(!power_on_state) {
                ESP_LOGI(TAG, "Please press and hold 3s before turning on");
                bsp_power_hold_disable();
                bsp_power_state_set(false);
                power_on_state = false;
            }
        }
        else
        {

        }
    }
}

static void IRAM_ATTR bsp_power_button_isr_handler(void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;
    bool pwr_io_state = gpio_get_level(BSP_BUTTON_ONOFF);
    if(pwr_io_state == BSP_POWER_BUTTON_ACTIVE_LEVEL) {
        pwr_press_start = xTaskGetTickCount();  
        pwr_press_release_state = 1;   
        if(pwr_release_start) pwr_release_start = 0;  
    } 
    else 
    {
        pwr_release_start = xTaskGetTickCount();
        pwr_press_release_state = 2; 
    }
    if (power_button_task_handle!= NULL) {
        vTaskNotifyGiveFromISR(power_button_task_handle, &high_task_wakeup);
    }

    if (high_task_wakeup == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void expander_io_lr20xx_reset( void)
{
    bsp_exp_output_io_set_level(BSP_LORA_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    bsp_exp_output_io_set_level(BSP_LORA_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

esp_err_t bsp_power_up_init(void)
{
    bool is_power_on = false;

    ESP_LOGI(TAG, "Power-up hardware version: HW=%s", BSP_HW_VERSION);

    // Get device power state from NVS, if not found, default to power on and save to NVS
    esp_err_t ret = bsp_power_state_get(&is_power_on);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get power state from NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    // Init tca6424 IO expander
    ret = bsp_power_prepare_bootstrap(false);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to prepare power bootstrap: %s", esp_err_to_name(ret));
        return ret;
    }
    // Init power button GPIO and interrupt
    ret = bsp_power_button_gpio_init();
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize power button GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = bsp_return_button_gpio_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RETURN button GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (!is_power_on) {
        ESP_LOGI(TAG, "Device is in power off state, waiting for power button long press to turn on");
        bool pwr_io_state = gpio_get_level(BSP_BUTTON_ONOFF);

        if(pwr_io_state == BSP_POWER_BUTTON_ACTIVE_LEVEL) {
            pwr_press_start = xTaskGetTickCount();  
            pwr_press_release_state = 1;   
            if(pwr_release_start) pwr_release_start = 0;  
            bsp_power_hold_enable();
        } 
        
        ESP_LOGI(TAG, "pwr_io_state init state=%d", pwr_io_state);
        while(1)
        {
            // Wait for user to long-press the power button to turn on the device, and set the power state in NVS
            if(power_on_control)
            {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));        
        }
    }
    else
    {
        power_on_control = true;
        bsp_power_hold_enable();
        ESP_LOGI(TAG, "Device is in power on state, continue to power on peripherals");
    }

    power_on_state = true;
    bsp_power_startup_peripherals();

    return ESP_OK;
}


/* ------------------------------------------------ power off start ------------------------------------------------ */
void battery_control_poweroff(void)
{
    // Battery control power off
    bsp_bat_power_control(false);
    // Battery ADC
    gpio_reset_pin(BSP_VBAT_ADC);

    // gpio_config_t io_conf = {
    //     .pin_bit_mask = 1ULL << BSP_VBAT_ADC,
    //     .mode = GPIO_MODE_DISABLE,
    //     .pull_up_en = GPIO_PULLUP_DISABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type = GPIO_INTR_DISABLE,
    // };
    // gpio_config(&io_conf);

}

void normal_sensor_poweroff(void)
{
    // Sensor power off
    bsp_exp_output_io_set_level(BSP_SEN_EN, 0);

    // Interrupt
}

void sdcard_poweroff(void)
{
    bsp_sdcard_unmount();
    gpio_reset_pin(BSP_SD_D0);
    gpio_reset_pin(BSP_SD_CLK);
    gpio_reset_pin(BSP_SD_CMD);
    bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void lcd_poweroff(void)
{
    // BL
    lcd_bl_off();

    // Display
    bsp_display_deinit();
}
/* ------------------------------------------------ power off end ------------------------------------------------ */

static esp_err_t bsp_power_down_init(void)
{
    esp_err_t ret = ESP_OK;

    // Battery
    battery_control_poweroff();

    // GNSS
    bsp_gnss_poweroff();

    // SD Card
    sdcard_poweroff();

    // PA_POWER
    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // LCD
    lcd_poweroff();

    // Sensor
    normal_sensor_poweroff();
    vTaskDelay(pdMS_TO_TICKS(10));

    if (i2s_tx_chan != NULL) {
        ret = i2s_channel_disable(i2s_tx_chan);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            BSP_ERROR_CHECK_RETURN_ERR(ret);

        }
    }
    if (i2s_rx_chan != NULL) {
        ret = i2s_channel_disable(i2s_rx_chan);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            BSP_ERROR_CHECK_RETURN_ERR(ret);
        }
    }

    i2c_master_bus_handle_t i2c_1_handle = bsp_i2c_1_get_handle();
    if (i2c_1_handle != NULL) {
        BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_1_deinit());
    }

    return ret;
}

esp_err_t bsp_power_hold_deep_sleep_test(void)
{
    esp_err_t ret = ESP_OK;
    uint32_t pin_levels = 0;
    ESP_LOGI(TAG, "Start BSP power-hold deep-sleep current test");

    BSP_ERROR_CHECK_RETURN_ERR(bsp_power_prepare_bootstrap(false));
    
    esp_io_expander_set_dir(io_expander, ALL_IO_EXPANDER_OUTPUT_PIN, IO_EXPANDER_OUTPUT);

    ESP_LOGI(TAG, "bsp_power_hold");
    BSP_ERROR_CHECK_RETURN_ERR(bsp_power_hold_enable());
    
    vTaskDelay(pdMS_TO_TICKS(10000));
    ret = esp_io_expander_get_level(io_expander, ALL_IO_EXPANDER_INPUT_PIN, &pin_levels);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "io expander get levels failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "io expander levels: 0x%08x", pin_levels);


    ESP_LOGI(TAG, "Gnss power off");
    // GNSS
    /* power off gnss */
    bsp_exp_output_io_set_level( BSP_GNSS_PWR_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10)); 

    gpio_config_t input_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    input_cfg.pin_bit_mask = 1ULL << (BSP_GNSS_RX);
    gpio_config(&input_cfg);
    input_cfg.pin_bit_mask = 1ULL << (BSP_GNSS_TX);
    gpio_config(&input_cfg);
    input_cfg.pin_bit_mask = (1ULL << BSP_GNSS_PPS0);
    gpio_config(&input_cfg); 

    bsp_exp_output_io_set_level( BSP_GNSS_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    bsp_exp_output_io_set_level( BSP_GNSS_VRTC_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    esp_io_expander_set_dir(io_expander, BSP_GNSS_SLEEP_INT|BSP_GNSS_RTC_INT, IO_EXPANDER_INPUT);

    ESP_LOGI(TAG, "Battery power off");  

    // Battery
    bsp_exp_output_io_set_level(BSP_BAT_ADC_EN, 0);
    gpio_reset_pin(BSP_VBAT_ADC);    // Battery ADC

    ESP_LOGI(TAG, "SD Card power off");
    // SD Card
    gpio_reset_pin(BSP_SD_D0);
    gpio_reset_pin(BSP_SD_CLK);
    gpio_reset_pin(BSP_SD_CMD);
    bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 0);

    ESP_LOGI(TAG, "Dispaly power off");
    // LCD
    gpio_reset_pin(BSP_LCD_CS);
    gpio_reset_pin(BSP_LCD_SDA);
    gpio_reset_pin(BSP_LCD_SCK);
    gpio_reset_pin(BSP_LCD_A0);
    gpio_reset_pin(BSP_LCD_PWM);

    bsp_exp_output_io_set_level(BSP_LCD_PWR_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    bsp_exp_output_io_set_level(BSP_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Sensor power off");
    // Sensor
    gpio_reset_pin(BSP_I2C_1_SCL);
    gpio_reset_pin(BSP_I2C_1_SDA);  
    bsp_exp_output_io_set_level(BSP_SEN_EN, 0);

    // exp io
    gpio_reset_pin(BSP_EXP_IO_INT);

    ESP_LOGI(TAG, "Audio power off");
    
    // Audio
    gpio_reset_pin(BSP_ADC_I2S_MCLK);
    gpio_reset_pin(BSP_ADC_I2S_SCLK);
    gpio_reset_pin(BSP_ADC_I2S_LRLK);
    gpio_reset_pin(BSP_ADC_I2S_SDIN);
    gpio_reset_pin(BSP_DAC_I2S_SDOUT);

    // PA_POWER
    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));


    ESP_LOGI(TAG, "LORA power off");
    // LORA
    bsp_exp_output_io_set_level(BSP_LORA_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    BSP_ERROR_CHECK_RETURN_ERR(bsp_power_test_lora_enter_sleep());

    rtc_gpio_isolate(BSP_LORA_SPI_SCK);
    rtc_gpio_isolate(BSP_LORA_SPI_MOSI);
    rtc_gpio_isolate(BSP_LORA_SPI_MISO);

    gpio_set_direction(BSP_LORA_SPI_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(BSP_LORA_SPI_CS, 1);

    // hold current state
    gpio_hold_en(BSP_LORA_SPI_CS);

    // hold during Deep Sleep
    gpio_deep_sleep_hold_en();
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    // IIC
    ret = esp_io_expander_get_level(io_expander, ALL_IO_EXPANDER_INPUT_PIN, &pin_levels);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "io expander get levels failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "io expander levels: 0x%08x", pin_levels);

    ESP_LOGI(TAG, "I2C0 deinit");
    esp_io_expander_del(io_expander);
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_0_deinit());
    gpio_reset_pin(BSP_I2C_0_SCL);
    gpio_reset_pin(BSP_I2C_0_SDA);


    // POWER BUTTON
    // power on off detect
    gpio_reset_pin(BSP_BUTTON_ONOFF); 

    ESP_LOGI(TAG, "BSP_PWR_HOLD asserted, entering deep sleep in 10 seconds");
    vTaskDelay(pdMS_TO_TICKS(10000));

    BSP_ERROR_CHECK_RETURN_ERR(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));
    ESP_LOGI(TAG, "Entering deep sleep for current measurement");
    vTaskDelay(pdMS_TO_TICKS(100));


    esp_deep_sleep_start();

    return ESP_OK;
}

esp_err_t bsp_power_hold_gnss_test(void)
{
    esp_err_t ret = ESP_OK;
    uint32_t pin_levels = 0;
    ESP_LOGI(TAG, "Start BSP power-hold deep-sleep current test");

    BSP_ERROR_CHECK_RETURN_ERR(bsp_power_prepare_bootstrap(false));
    
    esp_io_expander_set_dir(io_expander, ALL_IO_EXPANDER_OUTPUT_PIN, IO_EXPANDER_OUTPUT);


    ESP_LOGI(TAG, "bsp_power_hold");
    BSP_ERROR_CHECK_RETURN_ERR(bsp_power_hold_enable());

    ret = esp_io_expander_get_level(io_expander, ALL_IO_EXPANDER_INPUT_PIN, &pin_levels);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "io expander get levels failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "io expander levels: 0x%08x", pin_levels);

    // // power on off detect
    // gpio_reset_pin(BSP_BUTTON_ONOFF); 

    ESP_LOGI(TAG, "Gnss power off");
    // GNSS
    /* power off gnss */
    bsp_gnss_power_init();
    ESP_LOGI(TAG, "Battery power off");  

    // Battery
    bsp_exp_output_io_set_level(BSP_BAT_ADC_EN, 0);
    gpio_reset_pin(BSP_VBAT_ADC);    // Battery ADC

    ESP_LOGI(TAG, "SD Card power off");
    // SD Card
    gpio_reset_pin(BSP_SD_D0);
    gpio_reset_pin(BSP_SD_CLK);
    gpio_reset_pin(BSP_SD_CMD);
    bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 0);

    ESP_LOGI(TAG, "Dispaly power off");
    // LCD
    gpio_reset_pin(BSP_LCD_CS);
    gpio_reset_pin(BSP_LCD_SDA);
    gpio_reset_pin(BSP_LCD_SCK);
    gpio_reset_pin(BSP_LCD_A0);
    gpio_reset_pin(BSP_LCD_PWM);

    bsp_exp_output_io_set_level(BSP_LCD_PWR_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    bsp_exp_output_io_set_level(BSP_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Sensor power off");
    // Sensor
    gpio_reset_pin(BSP_I2C_1_SCL);
    gpio_reset_pin(BSP_I2C_1_SDA);  
    bsp_exp_output_io_set_level(BSP_SEN_EN, 0);

    // exp io
    gpio_reset_pin(BSP_EXP_IO_INT);

    ESP_LOGI(TAG, "Audio power off");
    
    // Audio
    gpio_reset_pin(BSP_ADC_I2S_MCLK);
    gpio_reset_pin(BSP_ADC_I2S_SCLK);
    gpio_reset_pin(BSP_ADC_I2S_LRLK);
    gpio_reset_pin(BSP_ADC_I2S_SDIN);
    gpio_reset_pin(BSP_DAC_I2S_SDOUT);

    // PA_POWER
    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));


    ESP_LOGI(TAG, "LORA power off");
    // LORA
    BSP_ERROR_CHECK_RETURN_ERR(bsp_power_test_lora_enter_sleep());

    rtc_gpio_isolate(BSP_LORA_SPI_SCK);
    rtc_gpio_isolate(BSP_LORA_SPI_MOSI);
    rtc_gpio_isolate(BSP_LORA_SPI_MISO);

    gpio_set_direction(BSP_LORA_SPI_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(BSP_LORA_SPI_CS, 1);

    // hold current state
    gpio_hold_en(BSP_LORA_SPI_CS);

    // hold during Deep Sleep
    gpio_deep_sleep_hold_en();
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    // IIC
    ret = esp_io_expander_get_level(io_expander, ALL_IO_EXPANDER_INPUT_PIN, &pin_levels);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "io expander get levels failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "io expander levels: 0x%08x", pin_levels);
    ESP_LOGI(TAG, "I2C0 deinit");
    esp_io_expander_del(io_expander);
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_0_deinit());
    gpio_reset_pin(BSP_I2C_0_SCL);
    gpio_reset_pin(BSP_I2C_0_SDA);

    // POWER BUTTON
    // power on off detect
    gpio_reset_pin(BSP_BUTTON_ONOFF); 

    ESP_LOGI(TAG, "BSP_PWR_HOLD asserted, entering deep sleep in 10 seconds");
    vTaskDelay(pdMS_TO_TICKS(10000));


    return ESP_OK;
}





