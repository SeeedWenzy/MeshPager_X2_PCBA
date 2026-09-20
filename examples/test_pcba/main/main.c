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
#include "esp_wifi.h"
#include "esp_pm.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp-bsp.h"
#include "bsp_board_extra.h"

#include "app_storage.h"

#include "ping_cmd.h"
#include "wifi_cmd.h"
#include "lora_cmd.h"
#include "ble_cmd.h"
#include "gnss_cmd.h"
#include "gpio_cmd.h"
#include "i2c_cmd.h"
#include "io_expander_cmd.h"
#include "sensor_cmd.h"
#include "audio_cmd.h"
#include "lcd_cmd.h"
#include "sd_lcd_cmd.h"
#include "power_cmd.h"
#include "ble_app.h"

#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "dl_lib_coefgetter_if.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "model_path.h"
#include "ringbuf.h"
#include "esp_nsn_models.h"
#include "model_path.h"


static const char *TAG = "TEST_PCBA";
#define APP_SW_VERSION "test_pcba-2.0.11"

static void app_power_shutdown_handler(void *user_ctx)
{
    (void)user_ctx;

    esp_err_t ret = bsp_extra_player_del();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Player stop failed: %s", esp_err_to_name(ret));
    }

    ret = bsp_extra_codec_dev_stop();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Codec stop failed: %s", esp_err_to_name(ret));
    }

    ret = esp_wifi_stop();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGW(TAG, "Wi-Fi stop failed: %s", esp_err_to_name(ret));
    }

    ret = esp_wifi_deinit();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGW(TAG, "Wi-Fi deinit failed: %s", esp_err_to_name(ret));
    }

    if (bsp_sdcard != NULL) {
        ret = bsp_sdcard_unmount();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "SD card unmount failed: %s", esp_err_to_name(ret));
        }
    }
}

static void image_button_handler(uint32_t button_mask, bool pressed, void *user_ctx)
{
    esp_err_t ret = ESP_OK;

    (void)user_ctx;

    if (!pressed) {
        return;
    }

    if (button_mask == BSP_BUTTON_UP) {
        ret = sd_lcd_show_prev_image();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Show previous image failed: %s", esp_err_to_name(ret));
        }
    } else if (button_mask == BSP_BUTTON_DOWN) {
        ret = sd_lcd_show_next_image();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Show next image failed: %s", esp_err_to_name(ret));
        }
    }
}

static void app_console_start(esp_console_repl_config_t *repl_config)
{
    esp_console_repl_t *repl = NULL;

#if CONFIG_ESP_CONSOLE_UART_DEFAULT || CONFIG_ESP_CONSOLE_UART_CUSTOM
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, repl_config, &repl));
#else
#error "Console REPL requires a supported CONFIG_ESP_CONSOLE_* backend"
#endif

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

#if 1
void app_main(void)
{
    esp_err_t ret;

    ret = bsp_power_up_init();
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Power state is off, skip app startup");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Application software version: SW=%s", APP_SW_VERSION);
    ESP_ERROR_CHECK(bsp_power_shutdown_callback_register(app_power_shutdown_handler, NULL));
    app_storage_init();
    sd_lcd_cmd_init();
    ESP_ERROR_CHECK(bsp_button_event_callback_register(image_button_handler, NULL));
    bsp_extra_codec_init();
    bsp_extra_player_init();
    bsp_extra_codec_volume_set(100, NULL);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    app_wifi_initialise_config_t config = APP_WIFI_CONFIG_DEFAULT();
    config.storage = WIFI_STORAGE_RAM;
    config.ps_type = WIFI_PS_NONE;
    app_initialise_wifi(&config);
    ble_app_init();

    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "cmd>";
    repl_config.max_cmdline_length = 1024;
    repl_config.task_stack_size = 8192;

    esp_console_register_help_command();

    lora_cmd_register_all();
    ble_cmd_register_all();
    wifi_cmd_register_all();
    ping_cmd_register_ping();
    gnss_cmd_register_all();
    gpio_cmd_register_all();
    i2c_cmd_register_all();
    io_expander_cmd_register_all();
    sensor_cmd_register_all();
    audio_cmd_register_all();
    lcd_cmd_register_all();
    sd_lcd_cmd_register_all();
    power_cmd_register_all();
    
    app_console_start(&repl_config);
    ESP_LOGI(TAG, "Pcba test starting...");
    while (1) {
       vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#else
static int task_flag = 0;
static bool sd_mount_flag = false;

static TaskHandle_t mic_record_task_handle;

#define FILES_MAX   3
ringbuf_handle_t rb_debug[FILES_MAX] = {NULL};
FILE * file_save[FILES_MAX] = {NULL};

static esp_afe_sr_iface_t *afe_handle = NULL;

esp_err_t FatfsComboWrite(const void* buffer, int size, int count, FILE* stream)
{
    esp_err_t res = ESP_OK;
    res = fwrite(buffer, size, count, stream);
    res |= fflush(stream);
    res |= fsync(fileno(stream));
    return res;
}

void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_feed_channel_num(afe_data);
    int feed_channel = bsp_extra_get_feed_channel();
    assert(nch == feed_channel);
    int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff);

    while (task_flag) {
        bsp_extra_get_feed_data(false, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);

        afe_handle->feed(afe_data, i2s_buff);

        if (rb_bytes_available(rb_debug[0]) < audio_chunksize * nch * sizeof(int16_t)) {
            printf("ERROR! rb_debug[0] slow!!!\n");
        }

        rb_write(rb_debug[0], (char *)i2s_buff, audio_chunksize * nch * sizeof(int16_t), 0);
    }
    if (i2s_buff) {
        free(i2s_buff);
        i2s_buff = NULL;
    }
    vTaskDelete(NULL);
}

void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    int16_t *buff = malloc(afe_chunksize * sizeof(int16_t));
    assert(buff);
    printf("------------detect start------------\n");

    while (task_flag) {
        afe_fetch_result_t* res = afe_handle->fetch(afe_data); 
        if (res && res->ret_value != ESP_FAIL) {
            memcpy(buff, res->data, afe_chunksize * sizeof(int16_t));

            if (rb_bytes_available(rb_debug[1]) < afe_chunksize * 1 * sizeof(int16_t)) {
                printf("ERROR! rb_debug[1] slow!!!\n");
            }

            rb_write(rb_debug[1], (char *)buff, afe_chunksize * 1 * sizeof(int16_t), 0);
        }
    }
    if (buff) {
        free(buff);
        buff = NULL;
    }
    vTaskDelete(NULL);
}

void debug_pcm_save_Task(void *arg)
{
    int size = 4 * 2 * 32 * 16;   // It's 32ms for 4 channels, 4k bytes 16s
    int16_t *buf_temp = heap_caps_calloc(1, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    while (task_flag) {
        for (int i = 0; i < FILES_MAX; i++) {
            if (file_save[i] != NULL) {
                if (rb_bytes_filled(rb_debug[i]) > size) {
                    int ret = rb_read(rb_debug[i], (char *)buf_temp, size, 3000 / portTICK_PERIOD_MS);
                    if ((ret < 0) || (ret < size)) {
                        // ESP_LOGE(TAG, "rb_debug read error, ret: %d\n", ret);
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                        continue;
                    }
                    FatfsComboWrite(buf_temp, size, 1, file_save[i]);
                }
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    free(buf_temp);
    vTaskDelete(NULL);
}

void test_mic_record_task(void *arg)
{
    // srmodel_list_t *models = esp_srmodel_init("model");
    // afe_config_t *afe_config = afe_config_init(bsp_extra_get_input_format(), models, AFE_TYPE_VC, AFE_MODE_HIGH_PERF);
    afe_config_t *afe_config = afe_config_init(bsp_extra_get_input_format(), NULL, AFE_TYPE_VC, AFE_MODE_LOW_COST);

    afe_config->agc_init = true;
    afe_config->agc_compression_gain_db = 15;
    afe_config->agc_target_level_dbfs = 0;

    afe_handle = esp_afe_handle_from_config(afe_config);
    

    ESP_LOGI(TAG, "aec_init: %d", afe_config->aec_init);
    ESP_LOGI(TAG, "aec_mode: %d", afe_config->aec_mode);

    ESP_LOGI(TAG, "se_init: %d", afe_config->se_init);

    ESP_LOGI(TAG, "ns_init: %d", afe_config->ns_init);
    ESP_LOGI(TAG, "afe_ns_mode: %d", afe_config->afe_ns_mode);

    ESP_LOGI(TAG, "vad_init: %d", afe_config->vad_init);
    ESP_LOGI(TAG, "vad_mode: %d", afe_config->vad_mode);

    ESP_LOGI(TAG, "wakenet_init: %d", afe_config->wakenet_init);
    ESP_LOGI(TAG, "wakenet_mode: %d", afe_config->wakenet_mode);

    ESP_LOGI(TAG, "agc_init: %d", afe_config->agc_init);
    ESP_LOGI(TAG, "agc_mode: %d", afe_config->agc_mode);
    ESP_LOGI(TAG, "agc_compression_gain_db: %d", afe_config->agc_compression_gain_db);
    ESP_LOGI(TAG, "agc_target_level_dbfs: %d", afe_config->agc_target_level_dbfs);

    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(afe_config);
    afe_config_free(afe_config);

    rb_debug[0] = rb_create(afe_handle->get_feed_channel_num(afe_data) * 4 * 16000 * 2, 1);   // 4s ringbuf
    file_save[0] = fopen("/sdcard/feed.pcm", "w");
    if (file_save[0] == NULL) printf("can not open file\n");

    rb_debug[1] = rb_create(1 * 4 * 16000 * 2, 1);   // 4s ringbuf
    file_save[1] = fopen("/sdcard/fetch.pcm", "w");
    if (file_save[1] == NULL) printf("can not open file\n");

    xTaskCreatePinnedToCore(&debug_pcm_save_Task, "debug_pcm_save", 2 * 1024, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(&feed_Task, "feed", 8 * 1024, (void*)afe_data, 5, NULL, 0);
    xTaskCreatePinnedToCore(&detect_Task, "detect", 8 * 1024, (void*)afe_data, 5, NULL, 0);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    esp_err_t ret = ESP_OK;

    bsp_power_up_init();

     ret = bsp_sdcard_mount();
    if (ret == ESP_OK) {
        sd_mount_flag = true;
        ESP_LOGW(TAG, "bsp sdcard mount ok");
    } else {
        sd_mount_flag = false;
        ESP_LOGE(TAG, "bsp sdcard mount fail");
    }

    bsp_extra_codec_init();
    bsp_extra_player_init();
    bsp_extra_codec_volume_set(100, NULL);

    while (!sd_mount_flag) {
        ESP_LOGI(TAG, "Insert SD and reboot");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ret = bsp_extra_player_play_file("/sdcard/1.wav");
    if (!ret) {
        ESP_LOGE(TAG, "bsp music play fail");
    }

    xTaskCreate(test_mic_record_task, "record", 4096, NULL, 5, &mic_record_task_handle);

    
    task_flag = 1;

    uint8_t cnt = 0;
    while (1) {
        cnt ++;
        ESP_LOGI(TAG, "cnt: %d", cnt);
        if (cnt > 15) {
            cnt = 16;
            task_flag = 0;
            ESP_LOGI(TAG, "Remove SD ");
            bsp_extra_codec_dev_stop();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif


