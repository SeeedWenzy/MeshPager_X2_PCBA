#include <dirent.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "driver/usb_serial_jtag.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmm350.h"
#include "bmm350_bsp_i2c1.h"
#include "bsp_board_extra.h"
#include "bsp_gnss_control.h"
#include "bsp_i2c_control.h"
#include "hal/gpio_types.h"
#include "lr20xx_hal.h"
#include "lr20xx_init.h"
#include "lr20xx_pa_pwr_cfg.h"
#include "lr20xx_radio_common.h"
#include "meshpager_x2.h"
#include "nmea.h"
#include "ral.h"
#include "ral_defs.h"
#include "ral_drv.h"
#include "ral_lr20xx.h"
#include "ral_lr20xx_bsp.h"
#include "ralf.h"
#include "ralf_defs.h"
#include "ralf_drv.h"
#include "ralf_lr20xx.h"
#include "radio_utilities.h"
#include "spa06_bsp_i2c1.h"
#include "ysn8900e.h"
#include "include/kode_lsm6dsox.h"
#include "ble_adv_test.h"

/* esp-sr AFE for microphone recording (own instance — the component's built-in
 * AFE wrapper defaults to "M" which is silent on this board). Mirrors the
 * proven record path in test_pcba audio_cmd.c / test_fullfeatured peripherals.c. */
#include "esp_afe_config.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"

static const char *TAG = "TEST_POWER";
#define APP_SW_VERSION "test_power-full-lowpower-2.0.2"

#define TEST_IMAGE_DURATION_MS      (4000)
#define TEST_AUDIO_DURATION_MS      (5000)
#define TEST_WIFI_DURATION_MS       (6000)
#define TEST_GNSS_DURATION_MS       (7000)
#define TEST_SENSOR_DURATION_MS     (1000)
#define TEST_LORA_DURATION_MS       (8000)
#define TEST_SDCARD_DURATION_MS     (2000)
#define TEST_BATTERY_DURATION_MS    (3000)
#define TEST_BASELINE_DURATION_MS   (11000)
#define TEST_RECORD_DURATION_MS     (5000)
#define TEST_BLE_ADV_DURATION_MS    (10000)
#define POWER_RAIL_SETTLE_MS        (100)

#define IMAGE_BUFFER_BYTES          (BSP_LCD_H_RES * BSP_LCD_V_RES * sizeof(uint16_t))
#define DISPLAY_DRAW_STRIP_ROWS     (40)
#define GNSS_READ_TIMEOUT_MS        (100)
#define LORA_TX_INTERVAL_MS         (500)
#define LORA_TX_DONE_TIMEOUT_MS     (3000)  /* bounded wait so a stuck radio can't hang the test */

#define SD_ROOT                     "/sdcard"
#define DEFAULT_IMAGE_PATH          SD_ROOT "/frame.rgb565"
#define DEFAULT_AUDIO_PATH          SD_ROOT "/1.wav"

/* AFE record -> SD wav -> play-back test. The ES7243E mic runs through the
 * esp-sr AFE (AEC/NS/AGC), which emits mono 16 kHz / 16-bit PCM. The recording
 * is saved to SD and also installed into s_wav_cache so the later playback
 * test plays it back. Mic PGA gain is pinned to the ES7243E maximum. */
#define RECORD_SAMPLE_RATE          (16000)
#define RECORD_BITS_PER_SAMPLE      (16)
#define RECORD_NUM_CHANNELS         (1)
#define RECORD_MIC_GAIN_DB          (37.5f)
#define RECORD_WAV_PATH             SD_ROOT "/test_record.wav"

typedef struct {
    uint16_t num_channels;
    uint32_t sample_rate;
    uint16_t bits_per_sample;
    uint32_t data_size;
    uint8_t *data;
    size_t cached_bytes;
    char path[160];
} wav_cache_t;

typedef struct {
    uint16_t *frame_buffer;
    char path[160];
} image_cache_t;

typedef struct {
    ysn8900e_t rtc_dev;
    ysn8900e_bsp_i2c0_context_t rtc_ctx;
    bool rtc_ready;

    struct spa06_dev spa06_dev;
    spa06_bsp_i2c1_context_t spa06_ctx;
    bool spa06_ready;

    struct bmm350_dev bmm350_dev;
    bmm350_bsp_i2c1_context_t bmm350_ctx;
    bool bmm350_ready;
} sensor_bundle_t;

static image_cache_t s_image_cache;
static wav_cache_t s_wav_cache;
static sensor_bundle_t s_sensors;
static kode_lsm6dsox_handle_t s_lsm6dsox;
static bool s_lsm6dsox_ready;
static bool s_display_ready;
static bool s_sd_mounted;
static bool s_wifi_started;
static bool s_wifi_stack_ready;
static bool s_audio_ready;
static bool s_lora_ready;
static bool s_radio_dio_isr_ready;
static TaskHandle_t s_radio_dio_task_handle;
static volatile bool s_lora_tx_done = true;
static volatile uint32_t s_lora_tx_seq = 0;
static char s_lora_last_payload[64];
static volatile bool s_lora_rx_done = false;
static volatile bool s_lora_rx_timeout = false;
static bool s_gnss_read_line_needs_reset = false;
static esp_netif_t *s_wifi_ap_netif = NULL;

static lr20xx_hal_context_t s_radio;
static const ralf_t s_modem_radio = RALF_LR20XX_INSTANTIATE(&s_radio);

typedef struct __attribute__((packed)) {
    char id[4];
    uint32_t size;
} riff_chunk_header_t;

typedef struct __attribute__((packed)) {
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
} wav_fmt_chunk_t;

static esp_err_t find_first_file_with_extension(const char *dir_path,
                                                const char *extension,
                                                char *out_path,
                                                size_t out_path_len,
                                                int depth)
{
    DIR *dir = NULL;
    struct dirent *entry = NULL;

    if (dir_path == NULL || extension == NULL || out_path == NULL || out_path_len == 0 || depth < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    dir = opendir(dir_path);
    if (dir == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    while ((entry = readdir(dir)) != NULL) {
        char child_path[192];
        struct stat st = { 0 };

        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        if (snprintf(child_path, sizeof(child_path), "%s/%s", dir_path, entry->d_name) >= (int)sizeof(child_path)) {
            continue;
        }

        if (stat(child_path, &st) != 0) {
            continue;
        }

        if (S_ISDIR(st.st_mode)) {
            if (depth > 0 && find_first_file_with_extension(child_path, extension, out_path, out_path_len, depth - 1) == ESP_OK) {
                closedir(dir);
                return ESP_OK;
            }
            continue;
        }

        if (!S_ISREG(st.st_mode)) {
            continue;
        }

        size_t name_len = strlen(entry->d_name);
        size_t ext_len = strlen(extension);
        if (name_len < ext_len) {
            continue;
        }

        if (strcasecmp(entry->d_name + name_len - ext_len, extension) == 0) {
            strlcpy(out_path, child_path, out_path_len);
            closedir(dir);
            return ESP_OK;
        }
    }

    closedir(dir);
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t resolve_media_path(const char *preferred_path,
                                    const char *extension,
                                    char *out_path,
                                    size_t out_path_len)
{
    struct stat st = { 0 };

    if (preferred_path != NULL && stat(preferred_path, &st) == 0 && S_ISREG(st.st_mode)) {
        strlcpy(out_path, preferred_path, out_path_len);
        return ESP_OK;
    }

    if (find_first_file_with_extension(SD_ROOT, extension, out_path, out_path_len, 2) == ESP_OK) {
        return ESP_OK;
    }

    return ESP_ERR_NOT_FOUND;
}

static esp_err_t load_image_cache(void)
{
    FILE *fp = NULL;
    size_t bytes_read = 0;
    char resolved_path[160];

    if (s_image_cache.frame_buffer != NULL) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(resolve_media_path(DEFAULT_IMAGE_PATH, ".rgb565", resolved_path, sizeof(resolved_path)), TAG, "image file not found on sdcard");

    s_image_cache.frame_buffer = heap_caps_malloc(IMAGE_BUFFER_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(s_image_cache.frame_buffer != NULL, ESP_ERR_NO_MEM, TAG, "allocate image cache failed");

    fp = fopen(resolved_path, "rb");
    if (fp == NULL) {
        free(s_image_cache.frame_buffer);
        s_image_cache.frame_buffer = NULL;
        return ESP_FAIL;
    }

    bytes_read = fread(s_image_cache.frame_buffer, 1, IMAGE_BUFFER_BYTES, fp);
    fclose(fp);

    if (bytes_read != IMAGE_BUFFER_BYTES) {
        ESP_LOGE(TAG, "Image size mismatch, expected %u got %u", (unsigned)IMAGE_BUFFER_BYTES, (unsigned)bytes_read);
        free(s_image_cache.frame_buffer);
        s_image_cache.frame_buffer = NULL;
        return ESP_ERR_INVALID_SIZE;
    }

    strlcpy(s_image_cache.path, resolved_path, sizeof(s_image_cache.path));
    ESP_LOGI(TAG, "Image cached from %s", s_image_cache.path);
    return ESP_OK;
}

static esp_err_t load_wav_cache(void)
{
    FILE *fp = NULL;
    char resolved_path[160];
    char riff_id[4];
    char wave_id[4];
    wav_fmt_chunk_t fmt = { 0 };
    bool fmt_found = false;
    bool data_found = false;
    uint32_t data_offset = 0;
    struct stat st = { 0 };

    if (s_wav_cache.data != NULL) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(resolve_media_path(DEFAULT_AUDIO_PATH, ".wav", resolved_path, sizeof(resolved_path)), TAG, "wav file not found on sdcard");

    fp = fopen(resolved_path, "rb");
    ESP_RETURN_ON_FALSE(fp != NULL, ESP_FAIL, TAG, "open wav file failed");

    if (fread(riff_id, 1, sizeof(riff_id), fp) != sizeof(riff_id) ||
        fseek(fp, 4, SEEK_CUR) != 0 ||
        fread(wave_id, 1, sizeof(wave_id), fp) != sizeof(wave_id)) {
        fclose(fp);
        return ESP_FAIL;
    }

    if (memcmp(riff_id, "RIFF", 4) != 0 || memcmp(wave_id, "WAVE", 4) != 0) {
        fclose(fp);
        return ESP_ERR_INVALID_RESPONSE;
    }

    while (!fmt_found || !data_found) {
        riff_chunk_header_t chunk = { 0 };

        if (fread(&chunk, 1, sizeof(chunk), fp) != sizeof(chunk)) {
            break;
        }

        if (memcmp(chunk.id, "fmt ", 4) == 0) {
            if (chunk.size < sizeof(fmt) || fread(&fmt, 1, sizeof(fmt), fp) != sizeof(fmt)) {
                fclose(fp);
                return ESP_ERR_INVALID_RESPONSE;
            }
            if (chunk.size > sizeof(fmt) && fseek(fp, (long)(chunk.size - sizeof(fmt)), SEEK_CUR) != 0) {
                fclose(fp);
                return ESP_FAIL;
            }
            fmt_found = true;
        } else if (memcmp(chunk.id, "data", 4) == 0) {
            data_offset = (uint32_t)ftell(fp);
            s_wav_cache.data_size = chunk.size;
            if (fseek(fp, (long)chunk.size, SEEK_CUR) != 0) {
                fclose(fp);
                return ESP_FAIL;
            }
            data_found = true;
        } else {
            if (fseek(fp, (long)chunk.size, SEEK_CUR) != 0) {
                fclose(fp);
                return ESP_FAIL;
            }
        }

        if ((chunk.size & 1U) != 0U) {
            if (fseek(fp, 1, SEEK_CUR) != 0) {
                fclose(fp);
                return ESP_FAIL;
            }
        }
    }

    if (!fmt_found || !data_found || fmt.audio_format != 1 || fmt.bits_per_sample != 16) {
        fclose(fp);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (stat(resolved_path, &st) != 0 || data_offset >= (uint32_t)st.st_size) {
        fclose(fp);
        return ESP_FAIL;
    }

    s_wav_cache.num_channels = fmt.num_channels;
    s_wav_cache.sample_rate = fmt.sample_rate;
    s_wav_cache.bits_per_sample = fmt.bits_per_sample;
    s_wav_cache.cached_bytes = s_wav_cache.data_size;
    s_wav_cache.data = heap_caps_malloc(s_wav_cache.cached_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(s_wav_cache.data != NULL, ESP_ERR_NO_MEM, TAG, "allocate wav cache failed");

    if (fseek(fp, (long)data_offset, SEEK_SET) != 0 ||
        fread(s_wav_cache.data, 1, s_wav_cache.cached_bytes, fp) != s_wav_cache.cached_bytes) {
        fclose(fp);
        free(s_wav_cache.data);
        memset(&s_wav_cache, 0, sizeof(s_wav_cache));
        return ESP_FAIL;
    }

    fclose(fp);
    strlcpy(s_wav_cache.path, resolved_path, sizeof(s_wav_cache.path));
    ESP_LOGI(TAG,
             "Audio cached from %s, %u Hz, %u ch, %u bytes",
             s_wav_cache.path,
             (unsigned)s_wav_cache.sample_rate,
             (unsigned)s_wav_cache.num_channels,
             (unsigned)s_wav_cache.cached_bytes);
    return ESP_OK;
}

static esp_err_t init_display_if_needed(void)
{
    if (s_display_ready) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(bsp_display_init(), TAG, "display init failed");
    lcd_bl_on();
    ESP_RETURN_ON_ERROR(bsp_display_brightness_set(95), TAG, "display brightness failed");
    s_display_ready = true;
    return ESP_OK;
}

static esp_err_t init_audio_if_needed(void)
{
    if (s_audio_ready) {
        return ESP_OK;
    }

    /* SEN_EN powers both sensors and audio codecs (ES8311, ES7243E).
     * Must be enabled before codec I2C communication. */
    bsp_exp_output_io_set_level(BSP_SEN_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 1);
    ESP_RETURN_ON_ERROR(bsp_extra_codec_init(), TAG, "codec init failed");
    ESP_RETURN_ON_ERROR(bsp_extra_codec_volume_set(100, NULL), TAG, "codec volume failed");
    ESP_RETURN_ON_ERROR(bsp_extra_codec_mute_set(false), TAG, "codec unmute failed");
    s_audio_ready = true;
    return ESP_OK;
}

static esp_err_t init_wifi_stack_if_needed(void)
{
    if (s_wifi_stack_ready) {
        return ESP_OK;
    }

    /* esp_netif_init() and esp_event_loop_create_default() may each be called
     * only once system-wide. The baseline setup runs this stack once (via
     * init+deinit) to power the RF/PHY down before the first measurement, so
     * tolerate ESP_ERR_INVALID_STATE here when they were already created. */
    esp_err_t netif_ret = esp_netif_init();
    if (netif_ret != ESP_OK && netif_ret != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(netif_ret, TAG, "esp_netif_init failed");
    }
    esp_err_t evt_ret = esp_event_loop_create_default();
    if (evt_ret != ESP_OK && evt_ret != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(evt_ret, TAG, "event loop create failed");
    }

    if (s_wifi_ap_netif != NULL) {
        esp_netif_destroy(s_wifi_ap_netif);
        s_wifi_ap_netif = NULL;
    }
    s_wifi_ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init failed");
    s_wifi_stack_ready = true;
    return ESP_OK;
}

static esp_err_t init_sensor_bundle(sensor_bundle_t *bundle)
{
    int8_t rslt;

    if (bundle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!bundle->rtc_ready) {
        memset(&bundle->rtc_dev, 0, sizeof(bundle->rtc_dev));
        memset(&bundle->rtc_ctx, 0, sizeof(bundle->rtc_ctx));

        ESP_RETURN_ON_ERROR(ysn8900e_init_bsp_i2c0(&bundle->rtc_dev, &bundle->rtc_ctx), TAG, "YSN8900E bind failed");
        ESP_RETURN_ON_ERROR(ysn8900e_init(&bundle->rtc_dev), TAG, "YSN8900E init failed");
        bundle->rtc_ready = true;
    }

    if (!bundle->spa06_ready) {
        memset(&bundle->spa06_dev, 0, sizeof(bundle->spa06_dev));
        memset(&bundle->spa06_ctx, 0, sizeof(bundle->spa06_ctx));

        ESP_RETURN_ON_ERROR(spa06_init_bsp_i2c1(&bundle->spa06_dev, &bundle->spa06_ctx), TAG, "SPA06 bind failed");
        rslt = spa06_init(&bundle->spa06_dev);
        ESP_RETURN_ON_FALSE(rslt == SPA06_OK, ESP_FAIL, TAG, "SPA06 init failed: %d", rslt);
        bundle->spa06_ready = true;
    }

    if (!bundle->bmm350_ready) {
        memset(&bundle->bmm350_dev, 0, sizeof(bundle->bmm350_dev));
        memset(&bundle->bmm350_ctx, 0, sizeof(bundle->bmm350_ctx));

        ESP_RETURN_ON_ERROR(bmm350_init_bsp_i2c1(&bundle->bmm350_dev, &bundle->bmm350_ctx), TAG, "BMM350 bind failed");
        rslt = bmm350_init(&bundle->bmm350_dev);
        ESP_RETURN_ON_FALSE(rslt == BMM350_OK, ESP_FAIL, TAG, "BMM350 init failed: %d", rslt);
        rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &bundle->bmm350_dev);
        ESP_RETURN_ON_FALSE(rslt == BMM350_OK, ESP_FAIL, TAG, "BMM350 power mode failed: %d", rslt);
        rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_LOWNOISE, &bundle->bmm350_dev);
        ESP_RETURN_ON_FALSE(rslt == BMM350_OK, ESP_FAIL, TAG, "BMM350 odr failed: %d", rslt);
        rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &bundle->bmm350_dev);
        ESP_RETURN_ON_FALSE(rslt == BMM350_OK, ESP_FAIL, TAG, "BMM350 axis enable failed: %d", rslt);
        bundle->bmm350_ready = true;
    }

    return ESP_OK;
}

static esp_err_t init_lsm6dsox_if_needed(void)
{
    esp_err_t ret;
    /* LSM6DSOX shares I2C_1 with BMM350/SPA06. SA0 strap selects 0x6A/0x6B,
     * so probe both and accept the one returning id 0x6C. */
    static const uint8_t addr_candidates[] = { 0x6A, 0x6B };

    if (s_lsm6dsox_ready) {
        return ESP_OK;
    }

    memset(&s_lsm6dsox, 0, sizeof(s_lsm6dsox));

    i2c_master_bus_handle_t bus = bsp_i2c_1_get_handle();
    ESP_RETURN_ON_FALSE(bus != NULL, ESP_ERR_INVALID_STATE, TAG, "I2C_1 not initialized");

    bool probed = false;
    for (size_t i = 0; i < sizeof(addr_candidates) / sizeof(addr_candidates[0]); i++) {
        ret = kode_lsm6dsox_init_i2c(bus, addr_candidates[i], &s_lsm6dsox);
        if (ret != ESP_OK) {
            continue;
        }
        uint8_t id = 0;
        if (kode_lsm6dsox_check_id(&s_lsm6dsox, &id) == ESP_OK && id == 0x6C) {
            probed = true;
            ESP_LOGI(TAG, "LSM6DSOX found on I2C_1 @0x%02X", addr_candidates[i]);
            break;
        }
    }
    ESP_RETURN_ON_FALSE(probed, ESP_ERR_NOT_FOUND, TAG, "LSM6DSOX (id 0x6C) not found on I2C_1");

    ret = kode_lsm6dsox_config_default(&s_lsm6dsox);
    ESP_RETURN_ON_ERROR(ret, TAG, "LSM6DSOX config failed");

    s_lsm6dsox_ready = true;
    return ESP_OK;
}

static void deinit_sensor_bundle(sensor_bundle_t *bundle)
{
    if (bundle->bmm350_ready) {
        bmm350_set_powermode(BMM350_SUSPEND_MODE, &bundle->bmm350_dev);
        bmm350_deinit_bsp_i2c1(&bundle->bmm350_dev, &bundle->bmm350_ctx);
        bundle->bmm350_ready = false;
    }
    if (bundle->spa06_ready) {
        spa06_deinit_bsp_i2c1(&bundle->spa06_dev, &bundle->spa06_ctx);
        bundle->spa06_ready = false;
    }
    if (s_lsm6dsox_ready) {
        kode_lsm6dsox_set_power_mode(&s_lsm6dsox, KODE_LSM6DSOX_POWER_SUSPEND);
        s_lsm6dsox_ready = false;
    }
    /* NOTE: Do NOT reset I2C_1 GPIO pins or clear BSP_SEN_EN here.
     * BSP_SEN_EN powers both sensors and audio codecs (ES8311, ES7243E).
     * SEN_EN lifecycle is managed by the caller:
     *   - power_off_all_peripherals() (test baseline)
     *   - cleanup_peripherals() (deep-sleep preparation)
     */
}

static void radio_dio_isr_handler(void *arg)
{
    (void)arg;
    if (s_radio_dio_task_handle != NULL) {
        BaseType_t high_task_wakeup = pdFALSE;
        vTaskNotifyGiveFromISR(s_radio_dio_task_handle, &high_task_wakeup);
        if (high_task_wakeup == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static esp_err_t init_lora_if_needed(void)
{
    uint8_t status;
    esp_err_t ret;

    if (s_lora_ready) {
        return ESP_OK;
    }

    /* Hardware reset the LoRa module via IO expander before SPI init.
     * This is required when waking from sleep or after other peripherals
     * (e.g. BLE) may have affected the SPI bus state. */
    // LORA
    lr20xx_init(&s_radio);
    ral_reset(&(s_modem_radio.ral));
    status = ral_init(&(s_modem_radio.ral));
    ESP_RETURN_ON_FALSE(status == RAL_STATUS_OK, ESP_FAIL, TAG, "ral_init failed: %u", status);

    /* lr20xx_init() always reconfigures DIO as plain input (Intr:0),
     * so we must restore the rising-edge interrupt mode every time.
     * The ISR handler itself only needs adding once. */
    {
        const gpio_config_t int_gpio_config = {
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_POSEDGE,
            .pin_bit_mask = 1ULL << CONFIG_RADIO_INT_GPIO,
        };

        ret = gpio_config(&int_gpio_config);
        ESP_RETURN_ON_ERROR(ret, TAG, "lora dio gpio config failed");

        if (!s_radio_dio_isr_ready) {
            ret = gpio_isr_handler_add(CONFIG_RADIO_INT_GPIO, radio_dio_isr_handler, (void *)CONFIG_RADIO_INT_GPIO);
            ESP_RETURN_ON_ERROR(ret, TAG, "lora dio isr add failed");
            s_radio_dio_isr_ready = true;
        }
    }

    s_lora_ready = true;
    return ESP_OK;
}

static void radio_irq_task(void *arg)
{
    (void)arg;

    while (true) {
        ral_irq_t irq_regs = 0;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ral_get_and_clear_irq_status(&(s_modem_radio.ral), &irq_regs);

        if ((irq_regs & RAL_IRQ_TX_DONE) == RAL_IRQ_TX_DONE) {
            ESP_LOGI(TAG,
                     "radio_irq LoRa TX_DONE packet=%u payload=%s",
                     (unsigned)s_lora_tx_seq,
                     s_lora_last_payload[0] != '\0' ? s_lora_last_payload : "<empty>");
            s_lora_tx_done = true;
        }

        if ((irq_regs & RAL_IRQ_RX_DONE) == RAL_IRQ_RX_DONE) {
            ESP_LOGI(TAG, "radio_irq LoRa RX_DONE");
            s_lora_rx_done = true;
        }

        if ((irq_regs & RAL_IRQ_RX_TIMEOUT) == RAL_IRQ_RX_TIMEOUT) {
            ESP_LOGI(TAG, "radio_irq LoRa RX_TIMEOUT");
            s_lora_rx_timeout = true;
        }
    }
}

static void stop_wifi_if_running(void)
{
    if (!s_wifi_stack_ready) {
        return;
    }

    if (s_wifi_started) {
        esp_wifi_stop();
        s_wifi_started = false;
    }

    esp_wifi_deinit();
    s_wifi_stack_ready = false;
}

static void cleanup_peripherals(void *user_ctx)
{
    (void)user_ctx;
    uint32_t pin_levels = 0;
    /* ---- Wi-Fi ---- */
    stop_wifi_if_running();

    /* ---- GNSS: stop scan first, then full power off ---- */
    bsp_gnss_scan_stop();
    bsp_gnss_poweroff();
    /* Extra GNSS cleanup (matching bsp_power_hold_deep_sleep_test) */
    bsp_exp_output_io_set_level(BSP_GNSS_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    bsp_exp_output_io_set_level(BSP_GNSS_VRTC_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_reset_pin(BSP_GNSS_RX);
    gpio_reset_pin(BSP_GNSS_TX);
    gpio_reset_pin(BSP_GNSS_PPS0);

    /* ---- Sensors ---- */
    deinit_sensor_bundle(&s_sensors);

    /* ---- LoRa ---- */
    if (s_lora_ready) {
        ral_set_sleep(&(s_modem_radio.ral), true);
        s_lora_ready = false;
    }
    /* Remove DIO ISR and reset pin to release pulldown & interrupt mode */
    gpio_isr_handler_remove(CONFIG_RADIO_INT_GPIO);
    gpio_reset_pin(CONFIG_RADIO_INT_GPIO);
    rtc_gpio_isolate(BSP_LORA_SPI_SCK);
    rtc_gpio_isolate(BSP_LORA_SPI_MOSI);
    rtc_gpio_isolate(BSP_LORA_SPI_MISO);

    gpio_set_direction(BSP_LORA_SPI_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(BSP_LORA_SPI_CS, 1);

    // hold current state
    gpio_hold_en(BSP_LORA_SPI_CS);

    // hold during Deep Sleep
    gpio_deep_sleep_hold_en();

    /* ---- LoRa IRQ task ---- */
    if (s_radio_dio_task_handle != NULL) {
        vTaskDelete(s_radio_dio_task_handle);
        s_radio_dio_task_handle = NULL;
    }

    /* ---- Audio: codec + PA + I2S GPIO ---- */
    if (s_audio_ready) {
        bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 0);
        s_audio_ready = false;
    }
    gpio_reset_pin(BSP_ADC_I2S_MCLK);
    gpio_reset_pin(BSP_ADC_I2S_SCLK);
    gpio_reset_pin(BSP_ADC_I2S_LRLK);
    gpio_reset_pin(BSP_ADC_I2S_SDIN);
    gpio_reset_pin(BSP_DAC_I2S_SDOUT);

    /* ---- I2C_1 (audio codec + sensors use this bus) ---- */
    if (bsp_i2c_1_get_handle() != NULL) {
        bsp_i2c_1_deinit();
    }
    gpio_reset_pin(BSP_I2C_1_SCL);
    gpio_reset_pin(BSP_I2C_1_SDA);

    /* ---- Display: backlight + deinit + SPI GPIO ---- */
    if (s_display_ready) {
        lcd_bl_off();
        bsp_display_deinit();
        s_display_ready = false;
    }
    gpio_reset_pin(BSP_LCD_CS);
    gpio_reset_pin(BSP_LCD_SDA);
    gpio_reset_pin(BSP_LCD_SCK);
    gpio_reset_pin(BSP_LCD_A0);
    gpio_reset_pin(BSP_LCD_PWM);
    bsp_exp_output_io_set_level(BSP_LCD_PWR_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    bsp_exp_output_io_set_level(BSP_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* ---- SD Card: unmount + GPIO reset ---- */
    if (s_sd_mounted) {
        bsp_sdcard_unmount();
        s_sd_mounted = false;
    }
    gpio_reset_pin(BSP_SD_D0);
    gpio_reset_pin(BSP_SD_CLK);
    gpio_reset_pin(BSP_SD_CMD);
    bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* ---- Battery ADC ---- */
    bsp_bat_power_control(false);
    gpio_reset_pin(BSP_VBAT_ADC);

    /* ---- Expander output pins low ---- */
    bsp_exp_output_io_set_level(BSP_SEN_EN, 0);
    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 0);

    /* ---- EXP_IO_INT ---- */
    gpio_reset_pin(BSP_EXP_IO_INT);

    /* ---- I2C0 + IO Expander (last, since expander pins depend on I2C0) ---- */
    pin_levels = bsp_exp_input_io_get_level(ALL_IO_EXPANDER_INPUT_PIN);
    ESP_LOGI(TAG, "io expander levels: 0x%08x", pin_levels);
    bsp_deinit_io_expander();
    bsp_i2c_0_deinit();
    gpio_reset_pin(BSP_I2C_0_SCL);
    gpio_reset_pin(BSP_I2C_0_SDA);

    /* ---- POWER_BUTTON ---- */
    gpio_reset_pin(BSP_BUTTON_ONOFF);
}

static esp_err_t run_display_test(void)
{
    const size_t row_pixels = BSP_LCD_H_RES;
    const size_t row_bytes = row_pixels * sizeof(uint16_t);

    ESP_RETURN_ON_ERROR(init_display_if_needed(), TAG, "display not ready");
    ESP_RETURN_ON_ERROR(load_image_cache(), TAG, "image cache failed");

    ESP_LOGI(TAG,
             "Display image for %d ms: %s (%u bytes, DMA free=%u largest=%u)",
             TEST_IMAGE_DURATION_MS,
             s_image_cache.path,
             (unsigned)IMAGE_BUFFER_BYTES,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));

    /* The frame is cached in PSRAM, while the LCD SPI driver may need an
     * internal DMA-capable bounce buffer. Send only the 40-row amount allowed
     * by the shared LCD SPI bus so one draw cannot exhaust DMA memory. */
    for (int y = 0; y < BSP_LCD_V_RES; y += DISPLAY_DRAW_STRIP_ROWS) {
        int strip_rows = BSP_LCD_V_RES - y;
        if (strip_rows > DISPLAY_DRAW_STRIP_ROWS) {
            strip_rows = DISPLAY_DRAW_STRIP_ROWS;
        }

        esp_err_t ret = bsp_display_draw_bitmap(0,
                                                y,
                                                BSP_LCD_H_RES,
                                                y + strip_rows,
                                                s_image_cache.frame_buffer + ((size_t)y * row_pixels));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG,
                     "draw bitmap failed for rows %d-%d (%u bytes): %s",
                     y,
                     y + strip_rows,
                     (unsigned)((size_t)strip_rows * row_bytes),
                     esp_err_to_name(ret));
            return ret;
        }
    }

    ESP_LOGI(TAG, "Display image sent in %d-row strips", DISPLAY_DRAW_STRIP_ROWS);
    vTaskDelay(pdMS_TO_TICKS(TEST_IMAGE_DURATION_MS));
    return ESP_OK;
}

static esp_err_t run_audio_test(void)
{
    uint32_t bytes_per_second;
    uint32_t target_bytes;
    uint32_t played_bytes = 0;

    ESP_RETURN_ON_ERROR(init_audio_if_needed(), TAG, "audio not ready");
    ESP_RETURN_ON_ERROR(load_wav_cache(), TAG, "wav cache failed");

    bytes_per_second = s_wav_cache.sample_rate * s_wav_cache.num_channels * (s_wav_cache.bits_per_sample / 8U);
    target_bytes = bytes_per_second * (TEST_AUDIO_DURATION_MS / 1000U);

    ESP_RETURN_ON_ERROR(bsp_extra_codec_play_set_fs(s_wav_cache.sample_rate,
                                                    s_wav_cache.bits_per_sample,
                                                    s_wav_cache.num_channels > 1 ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO),
                        TAG,
                        "codec fs set failed");

    ESP_LOGI(TAG, "Play cached audio for %d ms: %s", TEST_AUDIO_DURATION_MS, s_wav_cache.path);

    /* Diagnostic: dump buffer content stats so we can tell whether the
     * recording holds real audio (varied samples) vs DC (min~=max) vs
     * clipped noise (full-scale but structureless). peak alone can't tell. */
    {
        const int16_t *s = (const int16_t *)s_wav_cache.data;
        size_t n = s_wav_cache.cached_bytes / sizeof(int16_t);
        int16_t mn = 0, mx = 0;
        for (size_t i = 0; i < n; i++) {
            if (i == 0 || s[i] < mn) mn = s[i];
            if (i == 0 || s[i] > mx) mx = s[i];
        }
        ESP_LOGI(TAG,
                 "play buf: first8=[%d %d %d %d %d %d %d %d] min=%d max=%d",
                 n > 0 ? s[0] : 0, n > 1 ? s[1] : 0, n > 2 ? s[2] : 0, n > 3 ? s[3] : 0,
                 n > 4 ? s[4] : 0, n > 5 ? s[5] : 0, n > 6 ? s[6] : 0, n > 7 ? s[7] : 0,
                 mn, mx);
    }

    while (played_bytes < target_bytes) {
        size_t bytes_written = 0;
        size_t remain = target_bytes - played_bytes;
        size_t chunk = s_wav_cache.cached_bytes;
        size_t offset = played_bytes % s_wav_cache.cached_bytes;

        if (chunk > remain) {
            chunk = remain;
        }
        if (offset + chunk > s_wav_cache.cached_bytes) {
            chunk = s_wav_cache.cached_bytes - offset;
        }

        ESP_RETURN_ON_ERROR(bsp_extra_i2s_write(s_wav_cache.data + offset, chunk, &bytes_written, 1000), TAG, "audio write failed");
        played_bytes += bytes_written;
        if (bytes_written == 0) {
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

static void shutdown_audio_if_needed(void)
{
    if (!s_audio_ready) {
        return;
    }

    esp_err_t ret = bsp_extra_player_del();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Audio player delete failed: %s", esp_err_to_name(ret));
    }

    ret = bsp_extra_codec_dev_stop();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Audio codec stop failed: %s", esp_err_to_name(ret));
    }

    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 0);
    s_audio_ready = false;
}

static esp_err_t run_wifi_test(void)
{
    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = "test_power_ap",
            .ssid_len = 13,
            .channel = 1,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 4,
            .beacon_interval = 100,
        },
    };

    ESP_RETURN_ON_ERROR(init_wifi_stack_if_needed(), TAG, "wifi stack init failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage set failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "wifi mode set failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg), TAG, "wifi ap config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");
    s_wifi_started = true;

    ESP_LOGI(TAG, "Wi-Fi AP broadcast for %d ms, SSID=%s", TEST_WIFI_DURATION_MS, (const char *)ap_cfg.ap.ssid);
    vTaskDelay(pdMS_TO_TICKS(TEST_WIFI_DURATION_MS));

    ESP_RETURN_ON_ERROR(esp_wifi_stop(), TAG, "wifi stop failed");
    s_wifi_started = false;
    return ESP_OK;
}

static void reset_gnss_read_line_state(void)
{
    s_gnss_read_line_needs_reset = true;
}

static void gnss_read_line(char *buffer, size_t buffer_len, size_t *line_len)
{
    static char uart_buf[GNSS_UART_RX_BUF_SIZE + 1];
    static size_t total_bytes;
    static char *last_buf_end;
    int read_bytes;

    *line_len = 0;

    if (s_gnss_read_line_needs_reset) {
        memset(uart_buf, 0, sizeof(uart_buf));
        total_bytes = 0;
        last_buf_end = NULL;
        s_gnss_read_line_needs_reset = false;
    }

    if (last_buf_end != NULL) {
        size_t remaining = total_bytes - (size_t)(last_buf_end - uart_buf);
        memmove(uart_buf, last_buf_end, remaining);
        total_bytes = remaining;
        last_buf_end = NULL;
    }

    read_bytes = gnss_uart_read_bytes(uart_buf + total_bytes,
                                      sizeof(uart_buf) - 1U - total_bytes,
                                      pdMS_TO_TICKS(GNSS_READ_TIMEOUT_MS));
    if (read_bytes <= 0) {
        return;
    }

    total_bytes += (size_t)read_bytes;
    uart_buf[total_bytes] = '\0';

    char *start = memchr(uart_buf, '$', total_bytes);
    if (start == NULL) {
        total_bytes = 0;
        return;
    }

    char *end = memchr(start, '\r', total_bytes - (size_t)(start - uart_buf));
    if (end == NULL || (end + 1) >= uart_buf + total_bytes || *(end + 1) != '\n') {
        return;
    }

    end += 2;
    *line_len = (size_t)(end - start);
    if (*line_len >= buffer_len) {
        *line_len = 0;
        return;
    }

    memcpy(buffer, start, *line_len);
    buffer[*line_len] = '\0';
    if (end < uart_buf + total_bytes) {
        last_buf_end = end;
    } else {
        total_bytes = 0;
    }
}

static esp_err_t run_gnss_test(void)
{
    int64_t deadline_us;
    uint32_t parsed_count = 0;
    uint32_t raw_count = 0;
    char line[GNSS_UART_RX_BUF_SIZE + 1];

    if (!bsp_gnss_scan_start()) {
        return ESP_FAIL;
    }

    deadline_us = esp_timer_get_time() + (int64_t)TEST_GNSS_DURATION_MS * 1000;
    ESP_LOGI(TAG, "GNSS parsing for %d ms", TEST_GNSS_DURATION_MS);

    while (esp_timer_get_time() < deadline_us) {
        size_t line_len = 0;
        nmea_s *data = NULL;

        gnss_read_line(line, sizeof(line), &line_len);
        vTaskDelay(1);
        if (line_len == 0) {
            continue;
        }

        raw_count++;
        ESP_LOGI(TAG, "GNSS raw[%u]: %s", (unsigned)raw_count, line);
        data = nmea_parse(line, line_len, 0);
        if (data != NULL) {
            parsed_count++;
            nmea_free(data);
        }
    }

    bsp_gnss_scan_stop();
    ESP_LOGI(TAG, "GNSS done, raw=%u parsed=%u", (unsigned)raw_count, (unsigned)parsed_count);
    return ESP_OK;
}

static esp_err_t run_sensor_test(void)
{
    int64_t deadline_us;
    uint32_t sample_count = 0;

    ESP_RETURN_ON_ERROR(init_sensor_bundle(&s_sensors), TAG, "sensor init failed");
    deadline_us = esp_timer_get_time() + (int64_t)TEST_SENSOR_DURATION_MS * 1000;

    ESP_LOGI(TAG, "Sensor sampling for %d ms", TEST_SENSOR_DURATION_MS);
    while (esp_timer_get_time() < deadline_us) {
        uint8_t rtc_id = 0;
        ysn8900e_datetime_t datetime = { 0 };
        float spa06_temperature = 0.0f;
        float spa06_pressure = 0.0f;
        struct bmm350_mag_temp_data mag_data = { 0 };
        int8_t rslt;

        ESP_RETURN_ON_ERROR(ysn8900e_get_device_id(&s_sensors.rtc_dev, &rtc_id), TAG, "rtc id read failed");
        ESP_RETURN_ON_ERROR(ysn8900e_get_datetime(&s_sensors.rtc_dev, &datetime), TAG, "rtc datetime read failed");
        rslt = spa06_read_temperature(&spa06_temperature, &s_sensors.spa06_dev);
        ESP_RETURN_ON_FALSE(rslt == SPA06_OK, ESP_FAIL, TAG, "spa06 temperature read failed: %d", rslt);
        rslt = spa06_read_pressure(&spa06_pressure, &s_sensors.spa06_dev);
        ESP_RETURN_ON_FALSE(rslt == SPA06_OK, ESP_FAIL, TAG, "spa06 pressure read failed: %d", rslt);
        rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_data, &s_sensors.bmm350_dev);
        ESP_RETURN_ON_FALSE(rslt == BMM350_OK, ESP_FAIL, TAG, "bmm350 read failed: %d", rslt);
        uint32_t rtc_int_count = get_rtc_int_count();
        ESP_LOGI(TAG, "rtc_int_count: %u", rtc_int_count);

        ESP_LOGI(TAG,
                 "Sensor[%u] rtc_id=0x%02X time=%04u-%02u-%02u %02u:%02u:%02u\r\n spa06=%.2fC %.2fhPa\r\n bmm350=(%.2f,%.2f,%.2f)uT %.2fC\r\n batt=%umV\r\n",
                 (unsigned)sample_count,
                 rtc_id,
                 datetime.year,
                 datetime.month,
                 datetime.day,
                 datetime.hour,
                 datetime.minute,
                 datetime.second,
                 spa06_temperature,
                 spa06_pressure,
                 mag_data.x,
                 mag_data.y,
                 mag_data.z,
                 mag_data.temperature,
                 bsp_battery_voltage_read());

        sample_count++;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    deinit_sensor_bundle(&s_sensors);
    return ESP_OK;
}

static esp_err_t run_lora_tx_test(void)
{
    uint8_t status;
    int64_t deadline_us;
    uint32_t packet_count = 0;
    const char payload[] = "test_power122222222222232154326444444444444444444444444444444444444444444444444444444444444444sgwqergfdewqerafwqr";
    ralf_params_lora_t tx_lora_param = {
        .sync_word = 0x12,
        .symb_nb_timeout = 0,
        .rf_freq_in_hz = 915000000,
        .output_pwr_in_dbm = 22,
        .mod_params.cr = RAL_LORA_CR_4_5,
        .mod_params.sf = RAL_LORA_SF7,
        .mod_params.bw = RAL_LORA_BW_125_KHZ,
        .mod_params.ldro = false,
        .pkt_params.header_type = RAL_LORA_PKT_EXPLICIT,
        .pkt_params.pld_len_in_bytes = sizeof(payload) - 1,
        .pkt_params.crc_is_on = true,
        .pkt_params.invert_iq_is_on = false,
        .pkt_params.preamble_len_in_symb = 8,
    };

    ESP_RETURN_ON_ERROR(init_lora_if_needed(), TAG, "lora init failed");

    status = ralf_setup_lora(&s_modem_radio, &tx_lora_param);
    ESP_RETURN_ON_FALSE(status == RAL_STATUS_OK, ESP_FAIL, TAG, "lora setup failed: %u", status);
    status = ral_set_dio_irq_params(&(s_modem_radio.ral), RAL_IRQ_TX_DONE);
    ESP_RETURN_ON_FALSE(status == RAL_STATUS_OK, ESP_FAIL, TAG, "lora irq cfg failed: %u", status);

    deadline_us = esp_timer_get_time() + (int64_t)TEST_LORA_DURATION_MS * 1000;
    ESP_LOGI(TAG, "LoRa TX for %d ms", TEST_LORA_DURATION_MS);

    while (esp_timer_get_time() < deadline_us) {
        /* Bounded wait for the previous packet's TX_DONE. If the radio never
         * raises DIO (TX not completing) or the SPI read path returns wrong IRQ
         * bits, dump BUSY level + the IRQ status register so we can tell which,
         * then abort the loop instead of hanging the whole test. */
        {
            int64_t tx_done_deadline_us = esp_timer_get_time() + (int64_t)LORA_TX_DONE_TIMEOUT_MS * 1000;
            while (!s_lora_tx_done && esp_timer_get_time() < tx_done_deadline_us) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            if (!s_lora_tx_done) {
                ral_irq_t dbg_irq = 0;
                ral_get_irq_status(&(s_modem_radio.ral), &dbg_irq);
                ESP_LOGE(TAG,
                         "LoRa TX_DONE timeout pkt=%u busy=%d irq=0x%x -- "
                         "irq!=0 => DIO/read path; irq==0 => radio not finishing TX",
                         (unsigned)packet_count,
                         (int)gpio_get_level(CONFIG_RADIO_BUSY_GPIO),
                         (unsigned)dbg_irq);
                break;
            }
        }

        s_lora_tx_done = false;
        s_lora_tx_seq = packet_count + 1;
        strlcpy(s_lora_last_payload, payload, sizeof(s_lora_last_payload));
        ral_set_pkt_payload(&(s_modem_radio.ral), (uint8_t *)payload, sizeof(payload) - 1);
        ral_set_tx(&(s_modem_radio.ral));
        packet_count++;
        vTaskDelay(pdMS_TO_TICKS(LORA_TX_INTERVAL_MS));
    }
    ESP_LOGI(TAG, "LoRa TX test done, duration=%d ms", TEST_LORA_DURATION_MS);
    {
        int64_t tx_done_deadline_us = esp_timer_get_time() + (int64_t)LORA_TX_DONE_TIMEOUT_MS * 1000;
        while (!s_lora_tx_done && esp_timer_get_time() < tx_done_deadline_us) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (!s_lora_tx_done) {
            ral_irq_t dbg_irq = 0;
            ral_get_irq_status(&(s_modem_radio.ral), &dbg_irq);
            ESP_LOGE(TAG, "LoRa last TX_DONE timeout busy=%d irq=0x%x",
                     (int)gpio_get_level(CONFIG_RADIO_BUSY_GPIO), (unsigned)dbg_irq);
        }
    }

    status = ral_set_sleep(&(s_modem_radio.ral), true);
    ESP_RETURN_ON_FALSE(status == RAL_STATUS_OK, ESP_FAIL, TAG, "lora sleep failed: %u", status);

    ESP_LOGI(TAG, "LoRa sent %u packets", (unsigned)packet_count);
    return ESP_OK;
}

static esp_err_t run_lora_rx_test(void)
{
    uint8_t status;
    int64_t deadline_us;
    uint32_t rx_count = 0;
    uint32_t timeout_count = 0;

    ralf_params_lora_t rx_lora_param = {
        .sync_word = 0x12,
        .symb_nb_timeout = 0,
        .rf_freq_in_hz = 915000000,
        .output_pwr_in_dbm = 22,
        .mod_params.cr = RAL_LORA_CR_4_5,
        .mod_params.sf = RAL_LORA_SF7,
        .mod_params.bw = RAL_LORA_BW_125_KHZ,
        .mod_params.ldro = false,
        .pkt_params.header_type = RAL_LORA_PKT_EXPLICIT,
        .pkt_params.pld_len_in_bytes = 255,
        .pkt_params.crc_is_on = true,
        .pkt_params.invert_iq_is_on = false,
        .pkt_params.preamble_len_in_symb = 8,
    };

    ESP_RETURN_ON_ERROR(init_lora_if_needed(), TAG, "lora init failed");

    status = ralf_setup_lora(&s_modem_radio, &rx_lora_param);
    ESP_RETURN_ON_FALSE(status == RAL_STATUS_OK, ESP_FAIL, TAG, "lora rx setup failed: %u", status);
    status = ral_set_dio_irq_params(&(s_modem_radio.ral), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT);
    ESP_RETURN_ON_FALSE(status == RAL_STATUS_OK, ESP_FAIL, TAG, "lora rx irq cfg failed: %u", status);

    deadline_us = esp_timer_get_time() + (int64_t)TEST_LORA_DURATION_MS * 1000;
    ESP_LOGI(TAG, "LoRa RX for %d ms", TEST_LORA_DURATION_MS);

    while (esp_timer_get_time() < deadline_us) {
        s_lora_rx_done = false;
        s_lora_rx_timeout = false;

        status = ral_set_rx(&(s_modem_radio.ral), 5000);
        ESP_RETURN_ON_FALSE(status == RAL_STATUS_OK, ESP_FAIL, TAG, "lora set_rx failed: %u", status);

        while (!s_lora_rx_done && !s_lora_rx_timeout) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (s_lora_rx_done) {
            uint8_t rx_buf[256];
            uint16_t rx_size = 0;

            status = ral_get_pkt_payload(&(s_modem_radio.ral), sizeof(rx_buf), rx_buf, &rx_size);
            if (status == RAL_STATUS_OK && rx_size > 0) {
                rx_count++;
                ESP_LOGI(TAG, "LoRa RX[%u] len=%u", (unsigned)rx_count, (unsigned)rx_size);
                ESP_LOG_BUFFER_HEX(TAG, rx_buf, rx_size);
            }
        } else {
            timeout_count++;
        }
    }

    status = ral_set_sleep(&(s_modem_radio.ral), true);
    ESP_RETURN_ON_FALSE(status == RAL_STATUS_OK, ESP_FAIL, TAG, "lora sleep failed: %u", status);

    ESP_LOGI(TAG, "LoRa RX test done, received=%u timeouts=%u", (unsigned)rx_count, (unsigned)timeout_count);
    return ESP_OK;
}

/* ====================== Power Isolation Helpers ====================== */

static void power_off_all_peripherals(void)
{
    uint32_t pin_levels = 0;

    /* Display */
    if (s_display_ready) {
        lcd_bl_off();
        bsp_display_deinit();
        s_display_ready = false;
    }

    /* Audio codec + PA */
    if (s_audio_ready) {
        bsp_extra_player_del();
        bsp_extra_codec_dev_stop();
        s_audio_ready = false;
    }
    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 0);

    /* WiFi */
    stop_wifi_if_running();

    /* GNSS */
    bsp_gnss_scan_stop();
    bsp_gnss_poweroff();

    /* Sensors */
    deinit_sensor_bundle(&s_sensors);

    /* LoRa: put to sleep but keep ISR + task alive */
    if (s_lora_ready) {
        ral_set_sleep(&(s_modem_radio.ral), true);
        s_lora_ready = false;
    }

    /* SD Card */
    if (s_sd_mounted) {
        bsp_sdcard_unmount();
        s_sd_mounted = false;
    }

    /* Battery ADC */
    bsp_bat_power_control(false);

    /* Cut all power rails via IO expander.
     * NOTE: Do NOT deinit the IO expander or I2C_0 here!
     * Subsequent tests need bsp_exp_output_io_set_level() to turn
     * individual rails back on.  Only cleanup_peripherals() (deep
     * sleep) may tear down I2C_0 + expander. */
    bsp_exp_output_io_set_level(BSP_SEN_EN, 0);
    bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 0);
    bsp_exp_output_io_set_level(BSP_LCD_PWR_EN, 0);
    bsp_exp_output_io_set_level(BSP_LCD_RST, 0);
    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 0);

    pin_levels = bsp_exp_input_io_get_level(ALL_IO_EXPANDER_INPUT_PIN);
    ESP_LOGI(TAG, "io expander levels: 0x%08x", pin_levels);

    /* I2C_1 is only used by sensors and audio codecs on the SEN_EN rail.
     * Deinit it so the next user gets a fresh bus. */
    if (bsp_i2c_1_get_handle() != NULL) {
        bsp_i2c_1_deinit();
    }

    ESP_LOGI(TAG, "All peripheral power rails OFF (baseline)");
}

static void shutdown_display(void)
{
    if (!s_display_ready) {
        return;
    }

    lcd_bl_off();
    bsp_display_deinit();
    s_display_ready = false;
}

/* ====================== New Isolated Test Functions ====================== */

static esp_err_t run_sdcard_test(void)
{
    int64_t deadline_us;
    uint32_t io_count = 0;
    const char *test_path = SD_ROOT "/power_test.bin";
    const size_t test_block_size = 4096;
    uint8_t *test_buf = NULL;
    FILE *fp = NULL;

    /* Power on SD card */
    bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(POWER_RAIL_SETTLE_MS));

    ESP_RETURN_ON_ERROR(bsp_sdcard_mount(), TAG, "sdcard mount failed");
    s_sd_mounted = true;

    test_buf = heap_caps_malloc(test_block_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(test_buf != NULL, ESP_ERR_NO_MEM, TAG, "sd test buffer alloc failed");

    /* Fill with a known pattern */
    for (size_t i = 0; i < test_block_size; i++) {
        test_buf[i] = (uint8_t)(i & 0xFF);
    }

    deadline_us = esp_timer_get_time() + (int64_t)TEST_SDCARD_DURATION_MS * 1000;
    ESP_LOGI(TAG, "SD card r/w test for %d ms", TEST_SDCARD_DURATION_MS);

    while (esp_timer_get_time() < deadline_us) {
        /* Write */
        fp = fopen(test_path, "wb");
        if (fp == NULL) {
            free(test_buf);
            return ESP_FAIL;
        }
        fwrite(test_buf, 1, test_block_size, fp);
        fclose(fp);

        /* Read back */
        fp = fopen(test_path, "rb");
        if (fp == NULL) {
            free(test_buf);
            return ESP_FAIL;
        }
        fread(test_buf, 1, test_block_size, fp);
        fclose(fp);

        io_count++;
        ESP_LOGI(TAG, "SD card r/w[%u] %u bytes", (unsigned)io_count, (unsigned)test_block_size);
    }

    remove(test_path);
    free(test_buf);

    bsp_sdcard_unmount();
    s_sd_mounted = false;
    bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 0);

    ESP_LOGI(TAG, "SD card test done, %u iterations", (unsigned)io_count);
    return ESP_OK;
}

static esp_err_t run_sen_en_domain_test(void)
{
    int64_t deadline_us;
    uint32_t sample_count = 0;
    esp_err_t ret = ESP_OK;

    /* Power on SEN_EN rail (sensors + audio codecs) */
    bsp_exp_output_io_set_level(BSP_SEN_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* --- Phase 1: Sensor sampling --- */
    ret = init_sensor_bundle(&s_sensors);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sensor init failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    /* LSM6DSOX IMU shares I2C_1 with the other sensors; non-fatal if absent. */
    if (init_lsm6dsox_if_needed() != ESP_OK) {
        ESP_LOGW(TAG, "LSM6DSOX not available, skipping IMU sampling");
    }

    deadline_us = esp_timer_get_time() + (int64_t)TEST_SENSOR_DURATION_MS * 1000;
    ESP_LOGI(TAG, "SEN_EN domain: sensor sampling for %d ms", TEST_SENSOR_DURATION_MS);

    while (esp_timer_get_time() < deadline_us) {
        uint8_t rtc_id = 0;
        ysn8900e_datetime_t datetime = { 0 };
        float spa06_temperature = 0.0f;
        float spa06_pressure = 0.0f;
        struct bmm350_mag_temp_data mag_data = { 0 };
        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        float gx = 0.0f, gy = 0.0f, gz = 0.0f;
        int8_t rslt;

        ret = ysn8900e_get_device_id(&s_sensors.rtc_dev, &rtc_id);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "rtc id read failed: %s", esp_err_to_name(ret));
            goto cleanup;
        }
        ret = ysn8900e_get_datetime(&s_sensors.rtc_dev, &datetime);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "rtc datetime read failed: %s", esp_err_to_name(ret));
            goto cleanup;
        }
        uint32_t rtc_int_count = get_rtc_int_count();
        ESP_LOGI(TAG, "rtc_int_count: %u", rtc_int_count);

        ret = spa06_read_temperature(&spa06_temperature, &s_sensors.spa06_dev);
        if (ret != SPA06_OK) {
            ESP_LOGE(TAG, "SPA06 temperature read failed: %d", ret);
            ret = ESP_FAIL;
            goto cleanup;
        }
        ret = spa06_read_pressure(&spa06_pressure, &s_sensors.spa06_dev);
        if (ret != SPA06_OK) {
            ESP_LOGE(TAG, "SPA06 pressure read failed: %d", ret);
            ret = ESP_FAIL;
            goto cleanup;
        }
        rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_data, &s_sensors.bmm350_dev);
        if (rslt != BMM350_OK) {
            ESP_LOGE(TAG, "bmm350 read failed: %d", rslt);
            ret = ESP_FAIL;
            goto cleanup;
        }

        if (s_lsm6dsox_ready) {
            if (kode_lsm6dsox_read_accel(&s_lsm6dsox, &ax, &ay, &az) != ESP_OK ||
                kode_lsm6dsox_read_gyro(&s_lsm6dsox, &gx, &gy, &gz) != ESP_OK) {
                ESP_LOGW(TAG, "LSM6DSOX read failed this cycle");
            }
        }

        ESP_LOGI(TAG,
                 "Sensor[%u] rtc=0x%02X spa06=%.1fC/%.1fhPa bmm=(%.1f,%.1f,%.1f)uT imu_a=(%.0f,%.0f,%.0f)mg imu_g=(%.0f,%.0f,%.0f)mdps",
                 (unsigned)sample_count, rtc_id,
                 spa06_temperature, spa06_pressure,
                 mag_data.x, mag_data.y, mag_data.z,
                 ax, ay, az, gx, gy, gz);

        sample_count++;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    deinit_sensor_bundle(&s_sensors);

    /* --- Phase 2: Audio playback --- */
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "=== MEASURE_START: run_audio_test ===\r\n");
    ESP_LOGI(TAG, "SEN_EN domain: audio playback for %d ms", TEST_AUDIO_DURATION_MS);
    ret = run_audio_test();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "audio test failed in SEN_EN domain: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    ESP_LOGI(TAG, "=== MEASURE_START: run_audio_test ===\r\n");
    vTaskDelay(pdMS_TO_TICKS(5000));
cleanup:
    shutdown_audio_if_needed();
    /* Idempotent: on the normal path sensors are already suspended after
     * phase 1; this guarantees the ready-flags are cleared on error paths
     * too, so cleanup_peripherals() won't probe sensors over a torn-down
     * I2C_1 bus later. */
    deinit_sensor_bundle(&s_sensors);

    /* Cutting SEN_EN only power-cycles the external codec/sensor chips.
     * Reset the I2S GPIOs and tear down I2C_1 so the ESP32-side I2S
     * controller and I2C_1 bus stop drawing current and measured current
     * returns to baseline -- mirroring run_record_test / cleanup_peripherals(). */
    gpio_reset_pin(BSP_ADC_I2S_MCLK);
    gpio_reset_pin(BSP_ADC_I2S_SCLK);
    gpio_reset_pin(BSP_ADC_I2S_LRLK);
    gpio_reset_pin(BSP_ADC_I2S_SDIN);
    gpio_reset_pin(BSP_DAC_I2S_SDOUT);

    if (bsp_i2c_1_get_handle() != NULL) {
        bsp_i2c_1_deinit();
    }
    gpio_reset_pin(BSP_I2C_1_SCL);
    gpio_reset_pin(BSP_I2C_1_SDA);

    /* Power off SEN_EN rail */
    bsp_exp_output_io_set_level(BSP_SEN_EN, 0);

    ESP_LOGI(TAG, "SEN_EN domain test done");
    return ret;
}

/* Own esp-sr AFE capture instance for recording. The component's built-in AFE
 * wrapper is unreliable on this board ("M" outputs silence), so we drive our
 * own instance — same approach as test_pcba microphone_read. */
typedef struct {
    const esp_afe_sr_iface_t *afe_handle;
    esp_afe_sr_data_t *afe_data;
    int feed_chunk_samples;
    int fetch_chunk_samples;
    int feed_frames_per_fetch;
    int feed_channel_count;
    int16_t *feed_buffer;
} audio_afe_capture_t;

static void audio_afe_capture_cleanup(audio_afe_capture_t *cap)
{
    if (cap == NULL) {
        return;
    }
    if (cap->afe_handle != NULL && cap->afe_data != NULL) {
        cap->afe_handle->destroy(cap->afe_data);
        cap->afe_data = NULL;
    }
    if (cap->feed_buffer != NULL) {
        free(cap->feed_buffer);
        cap->feed_buffer = NULL;
    }
    cap->afe_handle = NULL;
}

static esp_err_t audio_afe_capture_init(audio_afe_capture_t *cap)
{
    afe_config_t *afe_config = NULL;
    int feed_buffer_samples;

    if (cap == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(cap, 0, sizeof(*cap));

    afe_config = afe_config_init(bsp_extra_get_input_format(), NULL, AFE_TYPE_VC, AFE_MODE_HIGH_PERF);
    if (afe_config == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* AEC + NS + AGC, single fixed output channel — settings proven to record
     * clean audio on this board (mirrors test_pcba microphone_read). */
    afe_config->se_init = false;
    afe_config->vad_init = false;
    afe_config->wakenet_init = false;
    afe_config->aec_init = true;
    afe_config->agc_init = true;
    afe_config->agc_mode = AFE_AGC_MODE_WEBRTC;
    afe_config->agc_compression_gain_db = 20;
    afe_config->agc_target_level_dbfs = 1;
    afe_config->ns_init = true;
    afe_config->afe_linear_gain = 1.0f;
    afe_config->fixed_output_channel = true;
    afe_config->output_playback_channel = false;

    cap->afe_handle = esp_afe_handle_from_config(afe_config);
    if (cap->afe_handle == NULL) {
        afe_config_free(afe_config);
        return ESP_FAIL;
    }
    cap->afe_data = cap->afe_handle->create_from_config(afe_config);
    afe_config_free(afe_config);
    if (cap->afe_data == NULL) {
        audio_afe_capture_cleanup(cap);
        return ESP_FAIL;
    }

    cap->feed_chunk_samples = cap->afe_handle->get_feed_chunksize(cap->afe_data);
    cap->fetch_chunk_samples = cap->afe_handle->get_fetch_chunksize(cap->afe_data);
    cap->feed_frames_per_fetch =
        (cap->fetch_chunk_samples + cap->feed_chunk_samples - 1) / cap->feed_chunk_samples;
    cap->feed_channel_count = bsp_extra_get_feed_channel();
    if (cap->feed_channel_count != cap->afe_handle->get_feed_channel_num(cap->afe_data)) {
        ESP_LOGE(TAG, "AFE feed channel mismatch: board=%d afe=%d",
                 cap->feed_channel_count, cap->afe_handle->get_feed_channel_num(cap->afe_data));
        audio_afe_capture_cleanup(cap);
        return ESP_ERR_INVALID_STATE;
    }

    feed_buffer_samples = cap->feed_chunk_samples * cap->feed_channel_count;
    cap->feed_buffer = calloc((size_t)feed_buffer_samples, sizeof(int16_t));
    if (cap->feed_buffer == NULL) {
        audio_afe_capture_cleanup(cap);
        return ESP_ERR_NO_MEM;
    }

    if (cap->afe_handle->reset_buffer != NULL) {
        cap->afe_handle->reset_buffer(cap->afe_data);
    }

    ESP_LOGI(TAG, "audio record: own AFE ready feed=%d nch=%d fetch=%d",
             cap->feed_chunk_samples, cap->feed_channel_count, cap->fetch_chunk_samples);
    return ESP_OK;
}

/* Write PCM samples to a WAV file on the SD card (canonical 44-byte PCM header). */
static esp_err_t write_wav_file(const char *path, const uint8_t *pcm, size_t pcm_bytes,
                                uint32_t sample_rate, uint16_t bits_per_sample, uint16_t channels)
{
    FILE *fp = fopen(path, "wb");
    ESP_RETURN_ON_FALSE(fp != NULL, ESP_FAIL, TAG, "open %s for write failed", path);

    const uint16_t block_align = (uint16_t)(channels * (bits_per_sample / 8U));
    const uint32_t byte_rate = sample_rate * block_align;
    const uint32_t data_size = (uint32_t)pcm_bytes;
    const uint32_t fmt_size = 16;
    const uint16_t audio_format = 1; /* PCM */
    const uint32_t riff_size = 4U + (8U + fmt_size) + (8U + data_size);

    int ok = 1;
    ok &= fwrite("RIFF", 1, 4, fp) == 4;
    ok &= fwrite(&riff_size, sizeof(riff_size), 1, fp) == 1;
    ok &= fwrite("WAVE", 1, 4, fp) == 4;
    ok &= fwrite("fmt ", 1, 4, fp) == 4;
    ok &= fwrite(&fmt_size, sizeof(fmt_size), 1, fp) == 1;
    ok &= fwrite(&audio_format, sizeof(audio_format), 1, fp) == 1;
    ok &= fwrite(&channels, sizeof(channels), 1, fp) == 1;
    ok &= fwrite(&sample_rate, sizeof(sample_rate), 1, fp) == 1;
    ok &= fwrite(&byte_rate, sizeof(byte_rate), 1, fp) == 1;
    ok &= fwrite(&block_align, sizeof(block_align), 1, fp) == 1;
    ok &= fwrite(&bits_per_sample, sizeof(bits_per_sample), 1, fp) == 1;
    ok &= fwrite("data", 1, 4, fp) == 4;
    ok &= fwrite(&data_size, sizeof(data_size), 1, fp) == 1;
    ok &= fwrite(pcm, 1, pcm_bytes, fp) == pcm_bytes;
    fclose(fp);

    ESP_RETURN_ON_FALSE(ok, ESP_FAIL, TAG, "wav write short/incomplete: %s", path);
    return ESP_OK;
}

/* Record 5 s through our own esp-sr AFE, save it to /sdcard/test_record.wav,
 * and install the PCM into s_wav_cache so the later playback test plays it.
 * The AFE runs in real time, so capture blocks ~5 s. */
static esp_err_t run_record_test(void)
{
    audio_afe_capture_t cap;
    uint8_t *rec = NULL;
    size_t recorded = 0;
    int16_t peak = 0;
    esp_err_t ret = ESP_OK;

    memset(&cap, 0, sizeof(cap));

    /* --- SD card (to save the WAV) --- */
    bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(POWER_RAIL_SETTLE_MS));
    ret = bsp_sdcard_mount();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "record: sd mount failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    s_sd_mounted = true;

    /* --- SEN_EN rail + ES7243E mic codec --- */
    bsp_exp_output_io_set_level(BSP_SEN_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* I2C_1 is shared by sensors + codecs and was torn down by the baseline
     * power-off. Bring it back up before codec init: bsp_audio_codec_*_init()
     * only fetch the handle (they don't create the bus), and the dangling
     * handle left by bsp_i2c_1_deinit() would crash i2c_master_bus_add_device().
     * (When sensors ran first they re-created the bus via their BSP inits.) */
    bsp_i2c_1_init();

    /* Force a full re-probe: bsp_extra_codec_init() returns early when
     * _is_audio_init==true, leaving stale handles after a prior power cycle. */
    bsp_extra_codec_deinit();
    ret = bsp_extra_codec_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "codec init for record failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    /* Mic gain: one I2C write, survives codec stop/reopen. Non-fatal. */
    if (bsp_extra_codec_set_mic_gain(RECORD_MIC_GAIN_DB) != ESP_OK) {
        ESP_LOGW(TAG, "record: mic gain set failed");
    }

    /* --- Own AFE instance --- */
    ret = audio_afe_capture_init(&cap);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AFE capture init failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    const size_t total_bytes = (size_t)RECORD_SAMPLE_RATE *
                               (RECORD_BITS_PER_SAMPLE / 8U) *
                               RECORD_NUM_CHANNELS *
                               (TEST_RECORD_DURATION_MS / 1000U);
    rec = heap_caps_malloc(total_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (rec == NULL) {
        ESP_LOGE(TAG, "record buffer alloc failed");
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    /* Hard wall-clock cap (3x duration): a misbehaving AFE that never yields
     * fetch data can't hang the test. */
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)TEST_RECORD_DURATION_MS * 3 * 1000;
    const size_t feed_bytes = (size_t)cap.feed_chunk_samples * (size_t)cap.feed_channel_count * sizeof(int16_t);

    ESP_LOGI(TAG, "Recording for %d ms", TEST_RECORD_DURATION_MS);
    while (recorded < total_bytes && esp_timer_get_time() < deadline_us) {
        bool feed_failed = false;
        for (int i = 0; i < cap.feed_frames_per_fetch; i++) {
            if (bsp_extra_get_feed_data(false, cap.feed_buffer, (int)feed_bytes) != ESP_OK) {
                ESP_LOGW(TAG, "record: feed read failed");
                feed_failed = true;
                break;
            }
            if (cap.afe_handle->feed(cap.afe_data, cap.feed_buffer) <= 0) {
                ESP_LOGW(TAG, "record: AFE feed failed");
                feed_failed = true;
                break;
            }
        }
        if (feed_failed) {
            break;
        }

        afe_fetch_result_t *fr = (cap.afe_handle->fetch_with_delay != NULL)
                                 ? cap.afe_handle->fetch_with_delay(cap.afe_data, 0)
                                 : cap.afe_handle->fetch(cap.afe_data);
        if (fr == NULL || fr->ret_value == ESP_FAIL || fr->data == NULL || fr->data_size <= 0) {
            continue;  /* AFE not ready yet — keep feeding and retry */
        }

        size_t copy_bytes = (size_t)fr->data_size;
        if (copy_bytes > total_bytes - recorded) {
            copy_bytes = total_bytes - recorded;
        }
        memcpy(rec + recorded, fr->data, copy_bytes);

        const int16_t *samples = (const int16_t *)(rec + recorded);
        for (size_t i = 0; i < copy_bytes / sizeof(int16_t); i++) {
            int16_t val = samples[i] > 0 ? samples[i] : (int16_t)(-samples[i]);
            if (val > peak) {
                peak = val;
            }
        }
        recorded += copy_bytes;
    }

    ESP_LOGI(TAG, "Recorded %u/%u bytes peak=%d", (unsigned)recorded, (unsigned)total_bytes, (int)peak);

    if (recorded == 0) {
        ESP_LOGE(TAG, "record: AFE produced no data");
        ret = ESP_FAIL;
        goto cleanup;
    }

    /* --- Save to SD --- */
    ret = write_wav_file(RECORD_WAV_PATH, rec, recorded,
                         RECORD_SAMPLE_RATE, RECORD_BITS_PER_SAMPLE, RECORD_NUM_CHANNELS);
    if (ret != ESP_OK) {
        goto cleanup;
    }

    /* Install into s_wav_cache so run_audio_test plays the recording back. */
    if (s_wav_cache.data != NULL) {
        free(s_wav_cache.data);
    }
    s_wav_cache.data = rec;
    s_wav_cache.cached_bytes = recorded;
    s_wav_cache.data_size = recorded;
    s_wav_cache.sample_rate = RECORD_SAMPLE_RATE;
    s_wav_cache.bits_per_sample = RECORD_BITS_PER_SAMPLE;
    s_wav_cache.num_channels = RECORD_NUM_CHANNELS;
    strlcpy(s_wav_cache.path, RECORD_WAV_PATH, sizeof(s_wav_cache.path));
    rec = NULL;  /* ownership transferred to s_wav_cache */

cleanup:
    audio_afe_capture_cleanup(&cap);   /* no-op if never inited (cap zeroed) */

    if (rec != NULL) {
        free(rec);
    }

    /* Full codec deinit (not just dev_stop) so _is_audio_init is cleared — the
     * next audio test (sen_en_domain playback) must re-create fresh codec
     * handles on the new I2C_1 bus instead of reusing stale ones. */
    bsp_extra_codec_deinit();
    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 0);
    s_audio_ready = false;

    /* NOTE: do NOT gpio_reset_pin() the I2S pins here. bsp_audio_init() runs
     * only once (guarded by i2s_dac_data_if != NULL in meshpager_x2.c, and
     * never cleared), so resetting the pins would disconnect the I2S GPIO
     * matrix for good and the later playback would have no MCLK/BCK/LRCK
     * reaching the ES8311 -> silence. The codec is already stopped above, so
     * the I2S channel is idle and draws negligible current; leave the pin
     * routing intact for the playback test. The final pin reset happens in
     * run_sen_en_domain_test cleanup and cleanup_peripherals() (deep sleep). */

    if (bsp_i2c_1_get_handle() != NULL) {
        bsp_i2c_1_deinit();
    }
    gpio_reset_pin(BSP_I2C_1_SCL);
    gpio_reset_pin(BSP_I2C_1_SDA);

    bsp_exp_output_io_set_level(BSP_SEN_EN, 0);

    /* SD teardown */
    if (s_sd_mounted) {
        bsp_sdcard_unmount();
        s_sd_mounted = false;
    }
    bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 0);

    ESP_LOGI(TAG, "Record test done, ret=%s", esp_err_to_name(ret));
    return ret;
}

static esp_err_t run_battery_adc_test(void)
{
    int64_t deadline_us;
    uint32_t read_count = 0;

    bsp_bat_power_control(true);
    vTaskDelay(pdMS_TO_TICKS(30));

    deadline_us = esp_timer_get_time() + (int64_t)TEST_BATTERY_DURATION_MS * 1000;
    ESP_LOGI(TAG, "Battery ADC test for %d ms", TEST_BATTERY_DURATION_MS);

    while (esp_timer_get_time() < deadline_us) {
        uint16_t voltage = bsp_battery_voltage_read();
        ESP_LOGI(TAG, "Battery[%u] %u mV", (unsigned)read_count, (unsigned)voltage);
        read_count++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    bsp_bat_power_control(false);
    ESP_LOGI(TAG, "Battery ADC test done, %u reads", (unsigned)read_count);
    return ESP_OK;
}

/* ====================== Deep Sleep ====================== */

static esp_err_t enter_low_power(void)
{
    ESP_LOGI(TAG, "Prepare deep sleep");

    cleanup_peripherals(NULL);

    // ESP_RETURN_ON_ERROR(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL), TAG, "disable wake source failed");

    vTaskDelay(pdMS_TO_TICKS(100));
    esp_deep_sleep_start();
    return ESP_OK;
}

static void log_wakeup_reason(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    switch (cause) {
    case ESP_SLEEP_WAKEUP_EXT1:
        ESP_LOGI(TAG, "Wakeup by BSP_BUTTON_ONOFF");
        break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        ESP_LOGI(TAG, "Cold boot");
        break;
    default:
        ESP_LOGI(TAG, "Wakeup cause: %d", cause);
        break;
    }
}

void app_main(void)
{
    esp_err_t ret;
#if 0
    /* ================================================================
     * PHASE 0: Bootstrap
     * bsp_power_up_init() powers on ALL rails + I2C_1; that's OK,
     * we power them off immediately after caching media.
     * ================================================================ */
    ret = bsp_power_up_init();
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Power state is off, skip app startup");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Application software version: SW=%s", APP_SW_VERSION);

    if (s_radio_dio_task_handle == NULL) {
        xTaskCreate(radio_irq_task, "lora_dio", 4096, NULL, 3, &s_radio_dio_task_handle);
    }

    ESP_ERROR_CHECK(bsp_power_shutdown_callback_register(cleanup_peripherals, NULL));
    log_wakeup_reason();

    /* ================================================================
     * PHASE 1: Cache media from SD card to PSRAM
     * ================================================================ */
    ret = bsp_sdcard_mount();
    if (ret == ESP_OK) {
        s_sd_mounted = true;
    } else {
        ESP_LOGW(TAG, "SD card mount failed: %s", esp_err_to_name(ret));
    }

    if (s_sd_mounted) {
        if ((ret = load_image_cache()) != ESP_OK) {
            ESP_LOGW(TAG, "Image cache skipped: %s", esp_err_to_name(ret));
        }
        if ((ret = load_wav_cache()) != ESP_OK) {
            ESP_LOGW(TAG, "Audio cache skipped: %s", esp_err_to_name(ret));
        }
        /* Unmount SD -- media is now in PSRAM */
        bsp_sdcard_unmount();
        s_sd_mounted = false;
    }

    /* ================================================================
     * PHASE 2: Establish baseline (all peripheral rails OFF)
     * ================================================================ */
    power_off_all_peripherals();

    /* Put LoRa into sleep as part of baseline (radio is always powered,
     * no dedicated power pin -- use sleep mode for minimum current) */
    init_lora_if_needed();
    ral_set_sleep(&(s_modem_radio.ral), true);
    s_lora_ready = false;

    /* Power down the 2.4 GHz RF/PHY. ESP-IDF enables PHY during boot
     * (esp_phy_init in the startup phase) and it stays on (~35 mA) until a
     * modem init+deinit drops the PHY reference count to zero. None of the
     * baseline path above touches WiFi/BT, so without this the first baseline
     * (and the display test right after it) measure RF current too. A
     * throwaway wifi stack init+deinit here powers the radio down -- the same
     * path that makes the post-wifi baseline read ~38 mA. */
    init_wifi_stack_if_needed();
    stop_wifi_if_running();

    ESP_LOGI(TAG, "=== BASELINE ESTABLISHED ===\r\n");

    /* ================================================================
     * PHASE 3: Baseline current measurement (all off, ESP32 idle)
     * ================================================================ */
    ESP_LOGI(TAG, "=== MEASURE_START: baseline ===\r\n");
    vTaskDelay(pdMS_TO_TICKS(TEST_BASELINE_DURATION_MS));
    ESP_LOGI(TAG, "=== MEASURE_END: baseline ===\r\n");

    /* ================================================================
     * PHASE 4: Isolated peripheral tests
     * Each test: baseline → power on target → test → power off → baseline
     * ================================================================ */

    /* --- Display --- */
    ESP_LOGI(TAG, "=== MEASURE_START: display ===\r\n");
    if ((ret = run_display_test()) != ESP_OK) {
        ESP_LOGW(TAG, "Display test failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "=== MEASURE_END: display ===\r\n");
    shutdown_display();
    bsp_exp_output_io_set_level(BSP_LCD_PWR_EN, 0);
    bsp_exp_output_io_set_level(BSP_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* --- WiFi AP --- */
    ESP_LOGI(TAG, "=== MEASURE_START: wifi ===\r\n");
    if ((ret = run_wifi_test()) != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi test failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "=== MEASURE_END: wifi ===\r\n");
    stop_wifi_if_running();
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* --- BLE Advertising (ESP32 internal RF, no external power pin) --- */
    ESP_LOGI(TAG, "=== MEASURE_START: ble_adv ===\r\n");
    if ((ret = ble_adv_test_init()) == ESP_OK) {
        if ((ret = ble_adv_test_start()) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(TEST_BLE_ADV_DURATION_MS));
            ble_adv_test_stop();
        } else {
            ESP_LOGW(TAG, "BLE adv start failed: %s", esp_err_to_name(ret));
        }
        ble_adv_test_deinit();
    } else {
        ESP_LOGW(TAG, "BLE init failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "=== MEASURE_END: ble_adv ===\r\n");
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* --- GNSS --- */
    reset_gnss_read_line_state();
    bsp_gnss_power_init();
    ESP_LOGI(TAG, "=== MEASURE_START: gnss ===\r\n");
    if ((ret = run_gnss_test()) != ESP_OK) {
        ESP_LOGW(TAG, "GNSS test failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "=== MEASURE_END: gnss ===\r\n");
    bsp_gnss_poweroff();
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* --- SD Card (manages its own power rail) --- */
    ESP_LOGI(TAG, "=== MEASURE_START: sdcard ===\r\n");
    if ((ret = run_sdcard_test()) != ESP_OK) {
        ESP_LOGW(TAG, "SD card test failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "=== MEASURE_END: sdcard ===\r\n");
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* --- Recording (AFE -> SD wav -> installs into s_wav_cache for playback) --- */
    ESP_LOGI(TAG, "=== MEASURE_START: record ===\r\n");
    if ((ret = run_record_test()) != ESP_OK) {
        ESP_LOGW(TAG, "Record test failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "=== MEASURE_END: record ===\r\n");
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* --- SEN_EN domain (sensors + audio playback of the recording) --- */
    ESP_LOGI(TAG, "=== MEASURE_START: sen_en_domain ===\r\n");
    if ((ret = run_sen_en_domain_test()) != ESP_OK) {
        ESP_LOGW(TAG, "SEN_EN domain test failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "=== MEASURE_END: sen_en_domain ===\r\n");
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* --- Battery ADC --- */
    ESP_LOGI(TAG, "=== MEASURE_START: battery_adc ===\r\n");
    if ((ret = run_battery_adc_test()) != ESP_OK) {
        ESP_LOGW(TAG, "Battery ADC test failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "=== MEASURE_END: battery_adc ===\r\n");
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* --- LoRa TX (manages its own init/sleep) --- */
    ESP_LOGI(TAG, "=== MEASURE_START: lora_tx ===\r\n");
    if ((ret = run_lora_tx_test()) != ESP_OK) {
        ESP_LOGW(TAG, "LoRa TX test failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "=== MEASURE_END: lora_tx ===\r\n");
    s_lora_ready = false;
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* --- LoRa RX (manages its own init/sleep) --- */
    ESP_LOGI(TAG, "=== MEASURE_START: lora_rx ===\r\n");
    if ((ret = run_lora_rx_test()) != ESP_OK) {
        ESP_LOGW(TAG, "LoRa RX test failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "=== MEASURE_END: lora_rx ===\r\n");
    s_lora_ready = false;
    vTaskDelay(pdMS_TO_TICKS(5000));
    /* ================================================================
     * PHASE 5: All tests done -- enter deep sleep
     * ================================================================ */
    ESP_ERROR_CHECK(enter_low_power());
#else
    bsp_power_hold_deep_sleep_test();
#endif


}