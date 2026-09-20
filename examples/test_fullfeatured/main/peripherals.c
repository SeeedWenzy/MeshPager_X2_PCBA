/*
 * peripherals.c
 *
 * Per-peripheral init / step / suspend implementations for the periodic runner.
 * Logic is adapted from examples/test_power/main/main.c but each step is a
 * single BOUNDED cycle (not a fixed-duration while loop) so it can be called
 * repeatedly on a configurable period.
 *
 * bsp_power_up_init() (called from main) already powers every rail and brings
 * up I2C_0 / I2C_1 / the IO expander, so the init callbacks here only do
 * chip-level setup — they never toggle rail-power pins.
 */

#include <dirent.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <time.h>

#include "argtable3/argtable3.h"
#include "esp_check.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
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
#include "gpgll.h"
#include "gpgga.h"
#include "gprmc.h"
#include "gpgsa.h"
#include "gpvtg.h"
#include "gptxt.h"
#include "gpgsv.h"

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
#include "host/ble_hs.h"    /* ble_gap_adv_active() for ble_step status */
#include "peripherals.h"

/* esp-sr AFE (own instance for recording, mirroring test_pcba microphone_read).
 * The component's built-in AFE wrapper defaults to "M" (silent here) and its
 * "MR" path never produces fetch data, so we drive our own AFE instance. */
#include "esp_afe_config.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"

static const char *TAG = "PERIPH";

/* ====================== Tunables ====================== */

#define SD_ROOT                     "/sdcard"
#define DEFAULT_IMAGE_PATH          SD_ROOT "/frame.rgb565"
#define DEFAULT_AUDIO_PATH          SD_ROOT "/1.wav"

#define IMAGE_BUFFER_BYTES          (BSP_LCD_H_RES * BSP_LCD_V_RES * sizeof(uint16_t))

#define GNSS_READ_TIMEOUT_MS        (0)     /* poll UART; gnss_step yields below */
#define GNSS_STEP_WINDOW_MS         (30000)

#define AUDIO_PLAY_BURST_MS         (5000)  /* ~5s of WAV per play step      */
#define AUDIO_RECORD_CHUNK_BYTES    (6400)  /* ~200ms @16kHz/16bit mono      */
#define AUDIO_RECORD_TIMEOUT_MS     (1000)

/* AFE record -> SD wav -> play-back test.
 * The ES7243E mic runs through the esp-sr AFE (AGC/NS/AEC), which emits mono
 * 16 kHz / 16-bit PCM at the codec-default ADC format, so the saved WAV is
 * mono. Mic PGA gain is pinned to the ES7243E maximum (+37.5 dB). Each cycle
 * rolls the output file test_record1.wav..test_record10.wav and overwrites any
 * existing file of that name. */
#define AUDIO_RECORD_DURATION_MS     (5000)
#define AUDIO_RECORD_SAMPLE_RATE     (16000)
#define AUDIO_RECORD_BITS_PER_SAMPLE (16)
#define AUDIO_RECORD_NUM_CHANNELS    (1)
#define AUDIO_RECORD_MIC_GAIN_DB     (37.5f)
#define AUDIO_RECORD_FILE_COUNT      (10)
/* esp-sr's afe_create_from_config pins the WebRTC NS ring buffers in internal
 * RAM and does NOT null-check every sr_rb_create() return — when internal heap
 * is exhausted one of those allocations dereferences NULL and panics (observed
 * after ~4.6 days, see logs/COM10.log). Boot baseline largest free block is
 * only ~31 KB, so a few KB of leak/fragmentation tips this over. Skip the
 * record step (degrade, don't reboot) when internal heap drops below this. */
#define AUDIO_AFE_CREATE_MIN_INTERNAL_BYTES (30000)

/* LCD: the cached frame lives in PSRAM (not DMA-capable). Sending the whole
 * 240x320 frame in one SPI transaction makes the driver bounce it through
 * internal DMA RAM, which under concurrent DMA load (audio/sdcard) can exhaust
 * internal DMA memory and the draw fails partway -> partial/blank screen.
 * Draw the frame in vertical strips through a small internal DMA buffer so
 * every SPI transaction DMAs straight from internal RAM. */
#define DISPLAY_DRAW_STRIP_ROWS     (16)

#define LORA_TX_WAIT_MS             (1000)
#define LORA_RX_WINDOW_MS           (2000)  /* ral_set_rx symbol timeout     */
#define LORA_RX_WAIT_MS             (2500)

#define SDCARD_TEST_PATH            SD_ROOT "/periodic_test.bin"
#define SDCARD_BLOCK_SIZE           (4096)



#define WIFI_WINDOW_MS       (3000)
#define BLE_WINDOW_MS        (5000)

/* ====================== Shared types & state ====================== */

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

typedef enum {
    AUDIO_MODE_PLAY = 0,
    AUDIO_MODE_RECORD,
    AUDIO_MODE_BOTH,
} audio_mode_t;

static image_cache_t s_image_cache;
static wav_cache_t s_wav_cache;
static sensor_bundle_t s_sensors;
static kode_lsm6dsox_handle_t s_imu;
static bool s_imu_ready;

static SemaphoreHandle_t s_i2c1_lock;   /* serializes sensor + imu + audio (shared I2C_1 / codec) */
static SemaphoreHandle_t s_lora_lock;   /* serializes lora_tx vs lora_rx (one radio) */

static bool s_display_ready;
static bool s_sd_mounted;
static bool s_wifi_started;
static bool s_wifi_stack_ready;
static bool s_audio_ready;
static bool s_lora_ready;
static bool s_radio_dio_isr_ready;
static bool s_gnss_started;
static bool s_gnss_read_line_needs_reset;
static esp_netif_t *s_wifi_ap_netif = NULL;
static TaskHandle_t s_radio_dio_task_handle = NULL;

/* Default to RECORD so the AFE record -> save -> play-back test runs every
 * cycle. Switch with the `audio_mode <play|record|both>` console command. */
static volatile audio_mode_t s_audio_mode = AUDIO_MODE_RECORD;
static size_t s_play_offset = 0;

/* AFE record test: cycles test_record1.wav..test_record10.wav. */
static uint8_t s_record_index = 1;

static const char *battery_charge_status(void);

/* LoRa radio context + IRQ signaling */
static lr20xx_hal_context_t s_radio;
static const ralf_t s_modem_radio = RALF_LR20XX_INSTANTIATE(&s_radio);
static volatile bool s_lora_tx_done = true;
static volatile uint32_t s_lora_tx_seq = 0;
static char s_lora_last_payload[64];
static volatile bool s_lora_rx_done = false;
static volatile bool s_lora_rx_timeout = false;

/* ====================== Media cache (from SD → PSRAM) ====================== */

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

    ESP_RETURN_ON_ERROR(resolve_media_path(DEFAULT_IMAGE_PATH, ".rgb565", resolved_path, sizeof(resolved_path)),
                        TAG, "image file not found on sdcard");

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

    ESP_RETURN_ON_ERROR(resolve_media_path(DEFAULT_AUDIO_PATH, ".wav", resolved_path, sizeof(resolved_path)),
                        TAG, "wav file not found on sdcard");

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
    ESP_LOGI(TAG, "Audio cached from %s, %u Hz, %u ch, %u bytes",
             s_wav_cache.path, (unsigned)s_wav_cache.sample_rate,
             (unsigned)s_wav_cache.num_channels, (unsigned)s_wav_cache.cached_bytes);
    return ESP_OK;
}

/* ====================== Sensor bundle ====================== */

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
        if (rslt != SPA06_OK) {
            spa06_deinit_bsp_i2c1(&bundle->spa06_dev, &bundle->spa06_ctx);
            ESP_RETURN_ON_FALSE(false, ESP_FAIL, TAG, "SPA06 init failed: %d", rslt);
        }
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
    /* RTC left initialized (low power). SEN_EN rail lifecycle is owned by BSP. */
}

/* ====================== Wi-Fi stack ====================== */

static esp_err_t init_wifi_stack_if_needed(void)
{
    if (s_wifi_stack_ready) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop create failed");

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

/* ====================== LoRa radio ====================== */

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

/* IRQ task — created once, handles TX_DONE / RX_DONE / RX_TIMEOUT signaling. */
static void radio_irq_task(void *arg)
{
    (void)arg;
    while (true) {
        ral_irq_t irq_regs = 0;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ral_get_and_clear_irq_status(&(s_modem_radio.ral), &irq_regs);

        if ((irq_regs & RAL_IRQ_TX_DONE) == RAL_IRQ_TX_DONE) {
            ESP_LOGI(TAG, "LoRa TX_DONE packet=%lu payload=%s",
                     (unsigned long)s_lora_tx_seq,
                     s_lora_last_payload[0] != '\0' ? s_lora_last_payload : "<empty>");
            s_lora_tx_done = true;
        }
        if ((irq_regs & RAL_IRQ_RX_DONE) == RAL_IRQ_RX_DONE) {
            ESP_LOGI(TAG, "LoRa RX_DONE irq");
            s_lora_rx_done = true;
        }
        if ((irq_regs & RAL_IRQ_RX_TIMEOUT) == RAL_IRQ_RX_TIMEOUT) {
            ESP_LOGI(TAG, "LoRa RX_TIMEOUT irq");
            s_lora_rx_timeout = true;
        }
    }
}

static esp_err_t ensure_lora_ready(void)
{
    uint8_t status;
    esp_err_t ret;

    if (s_lora_ready) {
        return ESP_OK;
    }

    /* Hardware-init the LoRa module. lr20xx_init() reconfigures DIO as plain
     * input, so the rising-edge interrupt mode must be restored every time.
     * The ISR handler itself is only added once. */
    lr20xx_init(&s_radio);
    ral_reset(&(s_modem_radio.ral));
    status = ral_init(&(s_modem_radio.ral));
    ESP_RETURN_ON_FALSE(status == RAL_STATUS_OK, ESP_FAIL, TAG, "ral_init failed: %u", status);

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

static void lora_sleep_locked(void)
{
    if (s_lora_ready) {
        ral_set_sleep(&(s_modem_radio.ral), true);
        s_lora_ready = false;
    }
}

/* ====================== GNSS line reader ====================== */

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

/* ============================================================= */
/*                      JOB CALLBACKS                            */
/* ============================================================= */

/* -------------------- Display --------------------
 *
 * The LCD refreshes to a different solid color every cycle (instead of a fixed
 * image), so a glance confirms the periodic display task is actually running.
 * display_init paints the first palette color so the panel never lingers on
 * the boot image; each display_step advances to the next color.
 *
 * A full 240x320 frame would live in PSRAM (not DMA-capable), and sending it in
 * one SPI transaction makes the driver bounce it through internal DMA RAM,
 * which under concurrent DMA load (audio/sdcard) can exhaust internal DMA memory
 * and leave the screen only partially refreshed. A solid color only needs one
 * small internal DMA band, blitted once per row strip (see DISPLAY_DRAW_STRIP_ROWS). */
static const uint16_t s_display_palette[] = {
    0xF800, /* red     */
    0x07E0, /* green   */
    0x001F, /* blue    */
    0xFFE0, /* yellow  */
    0x07FF, /* cyan    */
    0xF81F, /* magenta */
    0xFFFF, /* white   */
    0x0000, /* black   */
};
static size_t s_display_color_idx = 0;
static uint16_t *s_display_strip = NULL; /* one band, internal DMA RAM, reused */

/* Fill the whole panel with a solid RGB565 color through a single reusable
 * internal DMA strip. Lazily allocates the strip once; later calls reuse it. */
static esp_err_t display_fill_color(uint16_t color)
{
    const size_t row_bytes = (size_t)BSP_LCD_H_RES * sizeof(uint16_t);
    const int strip_rows = DISPLAY_DRAW_STRIP_ROWS;

    if (s_display_strip == NULL) {
        s_display_strip = heap_caps_malloc((size_t)strip_rows * row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        ESP_RETURN_ON_FALSE(s_display_strip != NULL, ESP_ERR_NO_MEM, TAG, "display strip alloc failed");
    }

    /* Paint the reusable band once, then blit it for every row band. */
    size_t strip_pixels = (size_t)strip_rows * BSP_LCD_H_RES;
    for (size_t i = 0; i < strip_pixels; i++) {
        s_display_strip[i] = color;
    }

    esp_err_t ret = ESP_OK;
    for (int y = 0; y < BSP_LCD_V_RES; y += strip_rows) {
        int h = (y + strip_rows > BSP_LCD_V_RES) ? (BSP_LCD_V_RES - y) : strip_rows;
        esp_err_t r = bsp_display_draw_bitmap(0, y, BSP_LCD_H_RES, y + h, s_display_strip);
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "display: color fill y=%d failed (%s)", y, esp_err_to_name(r));
            ret = r;
        }
    }
    return ret;
}

esp_err_t display_init(void *ctx)
{
    (void)ctx;
    if (s_display_ready) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(bsp_display_init(), TAG, "display init failed");
    lcd_bl_on();
    ESP_RETURN_ON_ERROR(bsp_display_brightness_set(95), TAG, "display brightness failed");

    /* Paint the first palette color right away so the panel never sits on the
     * boot image waiting for the first periodic refresh. */
    uint16_t color = s_display_palette[s_display_color_idx];
    s_display_color_idx = (s_display_color_idx + 1) % (sizeof(s_display_palette) / sizeof(s_display_palette[0]));
    (void)display_fill_color(color);
    ESP_LOGI(TAG, "display: init color=0x%04X", color);

    s_display_ready = true;
    return ESP_OK;
}

esp_err_t display_step(void *ctx)
{
    (void)ctx;
    if (!s_display_ready) {
        return ESP_OK;
    }
    uint16_t color = s_display_palette[s_display_color_idx];
    s_display_color_idx = (s_display_color_idx + 1) % (sizeof(s_display_palette) / sizeof(s_display_palette[0]));
    esp_err_t r = display_fill_color(color);
    if (r == ESP_OK) {
        ESP_LOGI(TAG, "display: color=0x%04X", color);
    }
    return r;
}

void display_suspend(void *ctx)
{
    (void)ctx;
    if (s_display_ready) {
        lcd_bl_off();
        bsp_display_deinit();
        s_display_ready = false;
    }
}

/* -------------------- Audio (play / record / both) -------------------- */

esp_err_t audio_init(void *ctx)
{
    (void)ctx;
    if (s_audio_ready) {
        return ESP_OK;
    }

    /* Recording writes test_recordN.wav to SD, but the audio job runs before the
     * sdcard job in the low-power loop, and sdcard_suspend + the SD rail cut at
     * the end of each cycle leave the card unmounted. The rail is re-powered at
     * wake before this runs, so re-mount here; sdcard_init is idempotent, which
     * makes the sdcard job's own init later a no-op. */
    sdcard_init(NULL);

    /* PA on; codec (re)probe. deinit first so a previous suspend (which
     * stop()'d the device) is fully reset before re-init. */
    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 1);
    vTaskDelay(10);
    bsp_extra_codec_deinit();
    ESP_RETURN_ON_ERROR(bsp_extra_codec_init(), TAG, "codec init failed");
    ESP_RETURN_ON_ERROR(bsp_extra_codec_volume_set(80,NULL), TAG, "codec volume failed");
    ESP_RETURN_ON_ERROR(bsp_extra_codec_mute_set(false), TAG, "codec unmute failed");

    if (s_wav_cache.data == NULL) {
        esp_err_t r = load_wav_cache();
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "audio: wav cache unavailable (%s) — play disabled", esp_err_to_name(r));
        }
    }
    s_audio_ready = true;
    return ESP_OK;
}

static esp_err_t audio_play_step(void)
{
    if (s_wav_cache.data == NULL || s_wav_cache.cached_bytes == 0) {
        return ESP_OK;
    }

    uint32_t bps = s_wav_cache.sample_rate * s_wav_cache.num_channels * (s_wav_cache.bits_per_sample / 8U);
    size_t target = (bps != 0) ? (bps * AUDIO_PLAY_BURST_MS / 1000U) : s_wav_cache.cached_bytes;
    if (target == 0) {
        target = s_wav_cache.cached_bytes;
    }

    ESP_RETURN_ON_ERROR(bsp_extra_codec_play_set_fs(s_wav_cache.sample_rate,
                                                    s_wav_cache.bits_per_sample,
                                                    s_wav_cache.num_channels > 1 ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO),
                        TAG, "codec fs set failed");

    size_t written_total = 0;
    while (written_total < target) {
        size_t remain = target - written_total;
        size_t off = (s_play_offset + written_total) % s_wav_cache.cached_bytes;
        size_t this_chunk = s_wav_cache.cached_bytes - off;
        if (this_chunk > remain) {
            this_chunk = remain;
        }
        size_t bw = 0;
        esp_err_t r = bsp_extra_i2s_write(s_wav_cache.data + off, this_chunk, &bw, 1000);
        if (r != ESP_OK || bw == 0) {
            return (r != ESP_OK) ? r : ESP_FAIL;
        }
        written_total += bw;
    }
    s_play_offset = (s_play_offset + written_total) % s_wav_cache.cached_bytes;
    return ESP_OK;
}

/* Write PCM samples to a WAV file on the SD card. fopen("wb") truncates any
 * existing file of the same name, so re-recording test_recordN.wav overwrites
 * the previous take. The header is the canonical 44-byte PCM WAV. */
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

    /* Accumulate every fwrite result; bitwise-AND (not &&) so no write is
     * skipped on an earlier failure — the full file must still be produced. */
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

/* Play back a 44-byte-header PCM WAV from the SD card for up to duration_ms.
 * The format matches what write_wav_file() produces (mono 16 kHz / 16-bit), so
 * the header is skipped and the stream is handed to the play codec directly. */
static esp_err_t play_recorded_wav(const char *path, uint32_t duration_ms)
{
    FILE *fp = fopen(path, "rb");
    ESP_RETURN_ON_FALSE(fp != NULL, ESP_FAIL, TAG, "open %s for play failed", path);

    uint8_t header[44];
    bool hdr_ok = (fread(header, 1, sizeof(header), fp) == sizeof(header));

    esp_err_t ret = ESP_OK;
    if (!hdr_ok) {
        fclose(fp);
        return ESP_FAIL;
    }

    ret = bsp_extra_codec_play_set_fs(AUDIO_RECORD_SAMPLE_RATE,
                                      AUDIO_RECORD_BITS_PER_SAMPLE,
                                      AUDIO_RECORD_NUM_CHANNELS > 1 ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO);
    if (ret != ESP_OK) {
        fclose(fp);
        return ret;
    }

    const size_t bps = (size_t)AUDIO_RECORD_SAMPLE_RATE * AUDIO_RECORD_NUM_CHANNELS * (AUDIO_RECORD_BITS_PER_SAMPLE / 8U);
    const size_t target = (bps != 0) ? (bps * duration_ms / 1000U) : 0;

    uint8_t *buf = heap_caps_malloc(AUDIO_RECORD_CHUNK_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == NULL) {
        fclose(fp);
        return ESP_ERR_NO_MEM;
    }

    size_t played = 0;
    while (played < target) {
        size_t want = target - played;
        if (want > AUDIO_RECORD_CHUNK_BYTES) {
            want = AUDIO_RECORD_CHUNK_BYTES;
        }
        size_t got = fread(buf, 1, want, fp);
        if (got == 0) {
            break; /* EOF — file shorter than the requested play window */
        }
        size_t bw = 0;
        esp_err_t r = bsp_extra_i2s_write(buf, got, &bw, 1000);
        if (r != ESP_OK || bw == 0) {
            ret = (r != ESP_OK) ? r : ESP_FAIL;
            break;
        }
        played += bw;
    }
    free(buf);
    fclose(fp);
    ESP_LOGI(TAG, "Audio playback %u bytes from %s", (unsigned)played, path);
    return ret;
}

/* Own AFE capture instance (mirrors test_pcba microphone_read). The component's
 * built-in AFE wrapper is unreliable here ("M" outputs silence, "MR" never
 * yields fetch data), so we create and drive our own esp-sr AFE in the board's
 * native "MR" format (mic on slot 1, reference on slot 0). */
typedef struct {
    const esp_afe_sr_iface_t *handle;
    esp_afe_sr_data_t *data;
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
    if (cap->handle != NULL && cap->data != NULL) {
        cap->handle->destroy(cap->data);
        cap->data = NULL;
    }
    if (cap->feed_buffer != NULL) {
        free(cap->feed_buffer);
        cap->feed_buffer = NULL;
    }
    cap->handle = NULL;
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
    ESP_RETURN_ON_FALSE(afe_config != NULL, ESP_ERR_NO_MEM, TAG, "afe_config_init failed");

    /* Same processing settings as test_pcba microphone_read (proven to record
     * clean audio on this board): AEC + NS + AGC, single fixed output channel. */
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

    /* Heap-exhaustion guard: see AUDIO_AFE_CREATE_MIN_INTERNAL_BYTES. The crash
     * lives inside esp-sr's precompiled afe_create_from_config, which we cannot
     * patch to null-check; block it here instead. */
    size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    if (internal_largest < AUDIO_AFE_CREATE_MIN_INTERNAL_BYTES) {
        ESP_LOGW(TAG, "audio record: internal heap low (largest=%uB < %uB), skipping to avoid AFE-create panic",
                 (unsigned)internal_largest, (unsigned)AUDIO_AFE_CREATE_MIN_INTERNAL_BYTES);
        afe_config_free(afe_config);
        return ESP_ERR_NO_MEM;
    }

    cap->handle = esp_afe_handle_from_config(afe_config);
    if (cap->handle == NULL) {
        afe_config_free(afe_config);
        return ESP_FAIL;
    }
    cap->data = cap->handle->create_from_config(afe_config);
    afe_config_free(afe_config);
    if (cap->data == NULL) {
        audio_afe_capture_cleanup(cap);
        return ESP_FAIL;
    }

    cap->feed_chunk_samples = cap->handle->get_feed_chunksize(cap->data);
    cap->fetch_chunk_samples = cap->handle->get_fetch_chunksize(cap->data);
    cap->feed_frames_per_fetch =
        (cap->fetch_chunk_samples + cap->feed_chunk_samples - 1) / cap->feed_chunk_samples;
    cap->feed_channel_count = bsp_extra_get_feed_channel();
    if (cap->feed_channel_count != cap->handle->get_feed_channel_num(cap->data)) {
        ESP_LOGE(TAG, "AFE feed channel mismatch: board=%d afe=%d",
                 cap->feed_channel_count, cap->handle->get_feed_channel_num(cap->data));
        audio_afe_capture_cleanup(cap);
        return ESP_ERR_INVALID_STATE;
    }

    feed_buffer_samples = cap->feed_chunk_samples * cap->feed_channel_count;
    cap->feed_buffer = calloc((size_t)feed_buffer_samples, sizeof(int16_t));
    if (cap->feed_buffer == NULL) {
        audio_afe_capture_cleanup(cap);
        return ESP_ERR_NO_MEM;
    }

    if (cap->handle->reset_buffer != NULL) {
        cap->handle->reset_buffer(cap->data);
    }

    ESP_LOGI(TAG, "audio record: own AFE ready fmt=%s feed=%d nch=%d fetch=%d",
             bsp_extra_get_input_format(), cap->feed_chunk_samples,
             cap->feed_channel_count, cap->fetch_chunk_samples);
    return ESP_OK;
}

/* Full record test, run once per audio step in RECORD/BOTH mode:
 *   1. pin the ES7243E mic PGA gain to its +37.5 dB maximum
 *   2. capture 5 s through our own esp-sr AFE (MR: mic slot1 + ref slot0,
 *      mono 16 kHz / 16-bit output) into PSRAM
 *   3. save it as /sdcard/test_recordN.wav (N = 1..10, overwriting), then
 *   4. play that recording back for 5 s.
 * The AFE runs in real time, so capture and playback each block ~5 s. */
static esp_err_t audio_record_step(void)
{
    if (!s_sd_mounted) {
        ESP_LOGW(TAG, "audio record: SD not mounted, skipping");
        return ESP_ERR_INVALID_STATE;
    }

    /* The ES7243E shares I2C_1 with the sensors/IMU; their I2C activity (both
     * init and per-cycle reads) collapses the mic capture to near-silence.
     * Re-initialize the codec right before recording so the ES7243E is freshly
     * configured after the last I2C_1 disturbance. Verified to restore raw_mic
     * from ~100 back to ~5000. */
    audio_suspend(NULL);
    audio_init(NULL);

    /* Mic gain: set every cycle — one I2C write, survives codec stop/reopen. */
    esp_err_t gr = bsp_extra_codec_set_mic_gain(AUDIO_RECORD_MIC_GAIN_DB);
    if (gr != ESP_OK) {
        ESP_LOGW(TAG, "audio record: mic gain set failed (%s)", esp_err_to_name(gr));
    }

    audio_afe_capture_t cap;
    esp_err_t ret = audio_afe_capture_init(&cap);
    ESP_RETURN_ON_ERROR(ret, TAG, "AFE capture init failed");

    char path[48];
    snprintf(path, sizeof(path), SD_ROOT "/test_record%u.wav", (unsigned)s_record_index);

    s_record_index = (uint8_t)((s_record_index % AUDIO_RECORD_FILE_COUNT) + 1U);

    const size_t total_bytes = (size_t)AUDIO_RECORD_SAMPLE_RATE *
                               (AUDIO_RECORD_BITS_PER_SAMPLE / 8U) *
                               AUDIO_RECORD_NUM_CHANNELS *
                               (AUDIO_RECORD_DURATION_MS / 1000U);
    uint8_t *rec = heap_caps_malloc(total_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (rec == NULL) {
        audio_afe_capture_cleanup(&cap);
        return ESP_ERR_NO_MEM;
    }

    const size_t feed_bytes = (size_t)cap.feed_chunk_samples * (size_t)cap.feed_channel_count * sizeof(int16_t);
    /* Hard wall-clock cap (3x the requested duration) so a misbehaving AFE that
     * never yields fetch data can't hang the step — the failure mode we hit with
     * the component's built-in AFE. */
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)AUDIO_RECORD_DURATION_MS * 3 * 1000;

    size_t recorded = 0;
    int16_t peak = 0;
    int16_t raw_mic_peak = 0;   /* feed_buffer even idx = ch1 (mic) after the swap */
    int16_t raw_ref_peak = 0;   /* feed_buffer odd idx  = ch0 (ref) after the swap */
    while (recorded < total_bytes && esp_timer_get_time() < deadline_us) {
        bool feed_failed = false;
        for (int i = 0; i < cap.feed_frames_per_fetch; i++) {
            if (bsp_extra_get_feed_data(false, cap.feed_buffer, (int)feed_bytes) != ESP_OK) {
                ESP_LOGW(TAG, "audio record: feed read failed");
                feed_failed = true;
                break;
            }
            /* After the ch0/ch1 swap inside bsp_extra_get_feed_data(false), the
             * buffer is interleaved [mic, ref, mic, ref, ...]. Track each slot's
             * peak so we can tell whether the mic is actually captured and
             * whether the AEC reference channel also carries mic (which would
             * make AEC cancel the voice). */
            for (size_t s = 0; s + 1 < (size_t)cap.feed_chunk_samples * (size_t)cap.feed_channel_count; s += 2) {
                int16_t m = cap.feed_buffer[s];
                int16_t r = cap.feed_buffer[s + 1];
                if (m < 0) {
                    m = (int16_t)(-m);
                }
                if (r < 0) {
                    r = (int16_t)(-r);
                }
                if (m > raw_mic_peak) {
                    raw_mic_peak = m;
                }
                if (r > raw_ref_peak) {
                    raw_ref_peak = r;
                }
            }
            if (cap.handle->feed(cap.data, cap.feed_buffer) <= 0) {
                ESP_LOGW(TAG, "audio record: AFE feed failed");
                feed_failed = true;
                break;
            }
        }
        if (feed_failed) {
            break;
        }

        afe_fetch_result_t *fr = (cap.handle->fetch_with_delay != NULL)
                                 ? cap.handle->fetch_with_delay(cap.data, 0)
                                 : cap.handle->fetch(cap.data);
        if (fr == NULL || fr->ret_value == ESP_FAIL || fr->data == NULL || fr->data_size <= 0) {
            continue;  /* AFE not ready yet — keep feeding and retry */
        }

        size_t copy_bytes = (size_t)fr->data_size;
        if (copy_bytes > total_bytes - recorded) {
            copy_bytes = total_bytes - recorded;
        }
        memcpy(rec + recorded, fr->data, copy_bytes);

        int16_t *samples = (int16_t *)(rec + recorded);
        for (size_t i = 0; i < copy_bytes / sizeof(int16_t); i++) {
            int16_t val = samples[i] > 0 ? samples[i] : (int16_t)(-samples[i]);
            if (val > peak) {
                peak = val;
            }
        }
        recorded += copy_bytes;
    }

    audio_afe_capture_cleanup(&cap);
    ESP_LOGI(TAG, "Audio recorded %u/%u bytes (afe_peak=%d raw_mic=%d raw_ref=%d) gain=%.1fdB -> %s",
             (unsigned)recorded, (unsigned)total_bytes, (int)peak,
             (int)raw_mic_peak, (int)raw_ref_peak,
             (double)AUDIO_RECORD_MIC_GAIN_DB, path);

    if (recorded == 0) {
        free(rec);
        return ESP_FAIL;
    }

    ret = write_wav_file(path, rec, recorded,
                         AUDIO_RECORD_SAMPLE_RATE, AUDIO_RECORD_BITS_PER_SAMPLE, AUDIO_RECORD_NUM_CHANNELS);
    free(rec);
    if (ret != ESP_OK) {
        return ret;
    }

    char play_path[48];
    snprintf(play_path, sizeof(play_path), SD_ROOT "/1.wav");
    /* Play the recording back straight from the SD file (round-trips the write). */
    return play_recorded_wav(path, AUDIO_RECORD_DURATION_MS);
}

esp_err_t audio_step(void *ctx)
{
    (void)ctx;
    audio_mode_t mode = s_audio_mode;

    xSemaphoreTake(s_i2c1_lock, portMAX_DELAY);
    esp_err_t ret = ESP_OK;
    if (mode == AUDIO_MODE_PLAY) {
        /* Play-only: loop the cached WAV. */
        ret = audio_play_step();
    } else {
        /* RECORD / BOTH: full AFE record -> save test_recordN.wav -> play back. */
        ret = audio_record_step();
    }
    xSemaphoreGive(s_i2c1_lock);
    return ret;
}

void audio_suspend(void *ctx)
{
    (void)ctx;
    if (!s_audio_ready) {
        return;
    }
    /* Raw i2s_write path doesn't use the audio-player task; just stop the
     * codec/I2S stream and cut the PA. */
    bsp_extra_codec_dev_stop();
    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 0);
    s_audio_ready = false;
}

/* -------------------- Wi-Fi AP -------------------- */

esp_err_t wifi_init(void *ctx)
{
    (void)ctx;
    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = "fullfeatured_ap",
            .ssid_len = 0,
            .channel = 1,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 4,
            .beacon_interval = 100,
        },
    };
    ap_cfg.ap.ssid_len = (uint8_t)strlen((const char *)ap_cfg.ap.ssid);

    ESP_RETURN_ON_ERROR(init_wifi_stack_if_needed(), TAG, "wifi stack init failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage set failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "wifi mode set failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg), TAG, "wifi ap config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");
    s_wifi_started = true;
    ESP_LOGI(TAG, "Wi-Fi AP started, SSID=%s", (const char *)ap_cfg.ap.ssid);
    return ESP_OK;
}

esp_err_t wifi_step(void *ctx)
{
    (void)ctx;
    wifi_sta_list_t sta = { 0 };
    int ret = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)WIFI_WINDOW_MS * 1000;
    if (esp_wifi_ap_get_sta_list(&sta) == ESP_OK) {
        ESP_LOGI(TAG, "Wi-Fi AP stations=%u", (unsigned)sta.num);
    }
    while (esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(100));
        ret = esp_wifi_ap_get_sta_list(&sta);
        if (ret != ESP_OK) {
            ESP_LOGI(TAG, "Wi-Fi AP error, ret=%d", ret);
        }
    }

    return ESP_OK;
}

void wifi_suspend(void *ctx)
{
    (void)ctx;
    if (s_wifi_started) {
        esp_wifi_stop();
        s_wifi_started = false;
    }
}

/* -------------------- BLE advertising -------------------- */

esp_err_t ble_init(void *ctx)
{
    (void)ctx;
    ESP_RETURN_ON_ERROR(ble_adv_test_init(), TAG, "ble init failed");
    ESP_RETURN_ON_ERROR(ble_adv_test_start(), TAG, "ble adv start failed");
    return ESP_OK;
}

esp_err_t ble_step(void *ctx)
{
    (void)ctx;
    int64_t deadline = esp_timer_get_time() + (int64_t)BLE_WINDOW_MS * 1000;
    ESP_LOGI(TAG, "BLE adv active=%d", ble_gap_adv_active() ? 1 : 0);
    while (esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return ESP_OK;
}

void ble_suspend(void *ctx)
{
    (void)ctx;
    ble_adv_test_stop();
}

/* -------------------- GNSS -------------------- */

esp_err_t gnss_init(void *ctx)
{
    (void)ctx;
    if (!s_gnss_started) {
        // bsp_gnss_power_init();
        if (!bsp_gnss_scan_start()) {
            return ESP_FAIL;
        }
        s_gnss_started = true;
        s_gnss_read_line_needs_reset = true; /* drop stale bytes after UART re-init */
    }
    return ESP_OK;
}

esp_err_t gnss_step(void *ctx)
{
    (void)ctx;
    int64_t deadline = esp_timer_get_time() + (int64_t)GNSS_STEP_WINDOW_MS * 1000;
    uint32_t raw_count = 0;
    uint32_t parsed_count = 0;
    char line[GNSS_UART_RX_BUF_SIZE + 1];
    bool nmea_rmc_success = false;

    /* Diagnostic: capture how full the RX ring buffer was on entry, BEFORE
     * draining. In debug-idle mode gnss_suspend (which deinstalls the UART)
     * is skipped, so between GNSS windows (~60 s) the 2 KB ring buffer fills
     * in ~4 s and then overflows for the rest of the gap. The value tells us
     * which failure mode "no NMEA this window" is:
     *   buffered ≈ 2048       -> ring buffer saturated (long overflow)
     *   buffered == 0          -> no bytes arriving at the UART (pin/matrix
     *                             or module not transmitting during window)
     *   init=0                 -> driver not installed (would also read 0) */
    int entry_buffered = gnss_uart_buffered_len();
    bool entry_init = gnss_uart_is_initialized();
    ESP_LOGI(TAG, "GNSS step enter: uart buffered=%d init=%d",
             entry_buffered, entry_init ? 1 : 0);

    /* Drop stale bytes left over from the previous window (the line reader is
     * a state machine with statics; without this it would re-parse a partial
     * sentence fragment from last window forever). Also clears the ring buffer
     * overflow / HW FIFO overrun so reception starts from a clean slate. */
    gnss_uart_drain_rx();
    s_gnss_read_line_needs_reset = true;

    while (esp_timer_get_time() < deadline) {
        size_t line_len = 0;
        gnss_read_line(line, sizeof(line), &line_len);
        /* GNSS may continuously provide bytes, so uart_read_bytes() can return
         * without blocking. Yield explicitly to keep IDLE0 and the task
         * watchdog serviced while this bounded parsing window is active. */
        vTaskDelay(1);
        if (line_len == 0) {
            continue;
        }
        raw_count++;
        nmea_s *data = nmea_parse(line, line_len, 0);
        if (data != NULL) {
            parsed_count++;
            if (parsed_count == 1) {
                ESP_LOGI(TAG, "GNSS parsed[%u]: %s", (unsigned)parsed_count, line);
            }

            /* nmea_parse() transfers ownership of data to this function.
             * Copy any fields needed for decisions before releasing it, and
             * free every successfully parsed sentence exactly once. */
            nmea_t data_type = data->type;
            if (NMEA_GPRMC == data_type) 
            {
                char fmt_buf[32];
                nmea_gprmc_s *pos = (nmea_gprmc_s *) data;
                ESP_LOGI(TAG,"Longitude:\n");
                ESP_LOGI(TAG, "  Degrees: %d", pos->longitude.degrees);
                ESP_LOGI(TAG, "  Minutes: %f", pos->longitude.minutes);
                ESP_LOGI(TAG, "  Cardinal: %c", (char) pos->longitude.cardinal);
                ESP_LOGI(TAG,"Latitude:\n");
                ESP_LOGI(TAG, "  Degrees: %d", pos->latitude.degrees);
                ESP_LOGI(TAG, "  Minutes: %f", pos->latitude.minutes);
                ESP_LOGI(TAG, "  Cardinal: %c", (char) pos->latitude.cardinal);
                if (pos->date_time.tm_year >= 0 &&
                    pos->date_time.tm_mon >= 0 &&
                    pos->date_time.tm_mon <= 11 &&
                    pos->date_time.tm_mday >= 1 &&
                    pos->date_time.tm_mday <= 31 &&
                    pos->date_time.tm_hour >= 0 &&
                    pos->date_time.tm_hour <= 23 &&
                    pos->date_time.tm_min >= 0 &&
                    pos->date_time.tm_min <= 59 &&
                    pos->date_time.tm_sec >= 0 &&
                    pos->date_time.tm_sec <= 60) 
                {
                    int written = snprintf(fmt_buf,
                           sizeof(fmt_buf),
                           "%04d-%02d-%02d %02d:%02d:%02d",
                           pos->date_time.tm_year + 1900,
                           pos->date_time.tm_mon + 1,
                           pos->date_time.tm_mday,
                           pos->date_time.tm_hour,
                           pos->date_time.tm_min,
                           pos->date_time.tm_sec);

                    if (written > 0 && (size_t)written < sizeof(fmt_buf)) 
                    {
                        ESP_LOGI(TAG, "Date & Time: %s", fmt_buf);
                    } 
                    else 
                    {
                        ESP_LOGW(TAG, "GNSS date/time formatting failed");
                    }
                } 
                else 
                {
                    ESP_LOGW(TAG,
                    "Invalid GNSS date/time: "
                    "year=%d mon=%d mday=%d hour=%d min=%d sec=%d",
                    pos->date_time.tm_year,
                    pos->date_time.tm_mon,
                    pos->date_time.tm_mday,
                    pos->date_time.tm_hour,
                    pos->date_time.tm_min,
                    pos->date_time.tm_sec);
                }
                ESP_LOGI(TAG, "Date & Time: %s", fmt_buf);
                ESP_LOGI(TAG, "Speed, in Knots: %f", pos->gndspd_knots);
                ESP_LOGI(TAG, "Track, in degrees: %f", pos->track_deg);
                ESP_LOGI(TAG, "Magnetic Variation:\n");
                ESP_LOGI(TAG, "  Degrees: %f", pos->magvar_deg);
                ESP_LOGI(TAG, "  Cardinal: %c", (char) pos->magvar_cardinal);
                double adjusted_course = pos->track_deg;
                if (NMEA_CARDINAL_DIR_EAST == pos->magvar_cardinal) {
                    adjusted_course -= pos->magvar_deg;
                } else if (NMEA_CARDINAL_DIR_WEST == pos->magvar_cardinal) {
                    adjusted_course += pos->magvar_deg;
                } else {
                    ESP_LOGI(TAG, "Invalid Magnetic Variation Direction!\n");
                }
                ESP_LOGI(TAG, "Adjusted Track (heading): %f", adjusted_course);
                if ((pos->longitude.degrees != 0) && (pos->latitude.degrees != 0)) {
                    nmea_rmc_success = true;
                }
            } 
            else if (NMEA_GPGSA == data_type)
            {
                nmea_gpgsa_s *gpgsa = (nmea_gpgsa_s *) data;
                int fix_type = gpgsa->fixtype;
                ESP_LOGI(TAG, "Fix:  %d", fix_type);
                nmea_free(data);
                data = NULL;
                if ((fix_type == 2 || fix_type == 3) && nmea_rmc_success) {
                    break;
                }
                continue;
            }

            nmea_free(data);
        }
    }

    if (raw_count == 0) {
        ESP_LOGI(TAG, "GNSS: no NMEA this window");
    } else {
        ESP_LOGI(TAG, "GNSS window raw=%u parsed=%u", (unsigned)raw_count, (unsigned)parsed_count);
    }

    /* Drain whatever is still buffered so the ring buffer does not sit full
     * (and overflow for ~60 s) between GNSS windows. In debug-idle mode
     * gnss_suspend() never runs to deinstall the UART, so without this the
     * RX channel is left saturated — the most likely cause of the RX path
     * going silent after the first working window. */
    gnss_uart_drain_rx();
    return ESP_OK;
}

void gnss_suspend(void *ctx)
{
    (void)ctx;
    if (s_gnss_started) {
        bsp_gnss_scan_stop();
        /* Soft power-off: cut the GNSS main regulator (PWR_EN) but KEEP
         * VRTC_EN (backup) so the next power-on is a WARM start. The full
         * bsp_gnss_poweroff() also clears VRTC_EN, which forces a 30 s+ cold
         * start and leaves the next gnss_step with no NMEA. VRTC backup draws
         * only a few µA, so this still saves the ~20-40 mA main current during
         * sleep while keeping NMEA available right after wake. */
        bsp_exp_output_io_set_level(BSP_GNSS_PWR_EN, 0);
        s_gnss_started = false;
    }
}

/* -------------------- Sensors -------------------- */

esp_err_t sensor_init(void *ctx)
{
    (void)ctx;
    return init_sensor_bundle(&s_sensors);
}

esp_err_t sensor_step(void *ctx)
{
    (void)ctx;
    uint8_t rtc_id = 0;
    ysn8900e_datetime_t datetime = { 0 };
    float spa06_temperature = 0.0f;
    float pressure = 0.0f;
    struct bmm350_mag_temp_data mag_data = { 0 };
    int8_t rslt;

    xSemaphoreTake(s_i2c1_lock, portMAX_DELAY);
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_ERROR(ysn8900e_get_device_id(&s_sensors.rtc_dev, &rtc_id), done, TAG, "rtc id read failed");
    ESP_GOTO_ON_ERROR(ysn8900e_get_datetime(&s_sensors.rtc_dev, &datetime), done, TAG, "rtc datetime read failed");
    ESP_GOTO_ON_ERROR(spa06_read_pressure(&pressure, &s_sensors.spa06_dev), done, TAG, "SPA06 pressure read failed");
    ESP_GOTO_ON_ERROR(spa06_read_temperature(&spa06_temperature, &s_sensors.spa06_dev), done, TAG, "SPA06 temperature read failed");
    rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_data, &s_sensors.bmm350_dev);
    if (rslt != BMM350_OK) {
        ret = ESP_FAIL;
    }
    uint32_t rtc_int_count = get_rtc_int_count();
    ESP_LOGI(TAG, "rtc_int_count: %u", rtc_int_count);
done:
    xSemaphoreGive(s_i2c1_lock);

    if (ret == ESP_OK) {
        battery_init(NULL);
        ESP_LOGI(TAG,
                 "Sensor rtc=0x%02X time=%04u-%02u-%02u %02u:%02u:%02u spa06=%.2f hPa %.1fC bmm=(%.1f,%.1f,%.1f)uT %.1fC batt=%umV status=%s",
                 rtc_id, datetime.year, datetime.month, datetime.day,
                 datetime.hour, datetime.minute, datetime.second,
                 pressure, spa06_temperature, mag_data.x, mag_data.y, mag_data.z, mag_data.temperature,
                 (unsigned)bsp_battery_voltage_read(), battery_charge_status());
        battery_suspend(NULL);
    } else {
        ESP_LOGW(TAG, "Sensor read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

void sensor_suspend(void *ctx)
{
    (void)ctx;
    deinit_sensor_bundle(&s_sensors);
}

/* -------------------- IMU (LSM6DSOX, shares I2C_1) -------------------- */

esp_err_t imu_init(void *ctx)
{
    (void)ctx;
    esp_err_t ret;
    /* LSM6DSOX shares I2C_1 with the other sensors. SA0 strap selects
     * 0x6A/0x6B, so probe both and accept the one returning id 0x6C. */
    static const uint8_t addr_candidates[] = { 0x6A, 0x6B };

    if (s_imu_ready) {
        return ESP_OK;
    }

    memset(&s_imu, 0, sizeof(s_imu));

    i2c_master_bus_handle_t bus = bsp_i2c_1_get_handle();
    ESP_RETURN_ON_FALSE(bus != NULL, ESP_ERR_INVALID_STATE, TAG, "I2C_1 not initialized");

    bool probed = false;
    for (size_t i = 0; i < sizeof(addr_candidates) / sizeof(addr_candidates[0]); i++) {
        ret = kode_lsm6dsox_init_i2c(bus, addr_candidates[i], &s_imu);
        if (ret != ESP_OK) {
            continue;
        }
        uint8_t id = 0;
        if (kode_lsm6dsox_check_id(&s_imu, &id) == ESP_OK && id == 0x6C) {
            probed = true;
            break;
        }
    }
    ESP_RETURN_ON_FALSE(probed, ESP_ERR_NOT_FOUND, TAG, "LSM6DSOX (id 0x6C) not found on I2C_1");

    ret = kode_lsm6dsox_config_default(&s_imu);
    ESP_RETURN_ON_ERROR(ret, TAG, "LSM6DSOX config failed");

    s_imu_ready = true;
    return ESP_OK;
}

esp_err_t imu_step(void *ctx)
{
    (void)ctx;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    xSemaphoreTake(s_i2c1_lock, portMAX_DELAY);
    esp_err_t ret = kode_lsm6dsox_read_accel(&s_imu, &ax, &ay, &az);
    if (ret == ESP_OK) {
        ret = kode_lsm6dsox_read_gyro(&s_imu, &gx, &gy, &gz);
    }
    xSemaphoreGive(s_i2c1_lock);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "IMU accel=(%.0f,%.0f,%.0f)mg gyro=(%.0f,%.0f,%.0f)mdps",
                 ax, ay, az, gx, gy, gz);
    }
    return ret;
}

void imu_suspend(void *ctx)
{
    (void)ctx;
    if (s_imu_ready) {
        kode_lsm6dsox_set_power_mode(&s_imu, KODE_LSM6DSOX_POWER_SUSPEND);
        s_imu_ready = false;
    }
}

/* -------------------- LoRa TX / RX -------------------- */

esp_err_t lora_tx_init(void *ctx)
{
    (void)ctx;
    return ensure_lora_ready();
}

esp_err_t lora_tx_step(void *ctx)
{
    (void)ctx;
    static const char payload[] = "fullfeatured";
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

    xSemaphoreTake(s_lora_lock, portMAX_DELAY);
    esp_err_t ret = ensure_lora_ready();
    if (ret != ESP_OK) {
        goto done;
    }
    if (ralf_setup_lora(&s_modem_radio, &tx_lora_param) != RAL_STATUS_OK ||
        ral_set_dio_irq_params(&(s_modem_radio.ral), RAL_IRQ_TX_DONE) != RAL_STATUS_OK) {
        ret = ESP_FAIL;
        goto done;
    }

    s_lora_tx_done = false;
    s_lora_tx_seq++;
    strlcpy(s_lora_last_payload, payload, sizeof(s_lora_last_payload));
    ral_set_pkt_payload(&(s_modem_radio.ral), (uint8_t *)payload, sizeof(payload) - 1);
    ral_set_tx(&(s_modem_radio.ral));

    /* Fast path: wait for TX_DONE via the DIO pin interrupt. If it hasn't
     * arrived after a grace window, fall back to polling the IRQ register —
     * that still completes the step, and if the poll catches TX_DONE it means
     * the radio transmitted fine but the DIO pin interrupt isn't firing
     * (wiring / wrong GPIO / stale level holding DIO high). */
    int64_t deadline = esp_timer_get_time() + (int64_t)LORA_TX_WAIT_MS * 1000;
    int64_t poll_after = esp_timer_get_time() + 100 * 1000;
    bool tx_done = s_lora_tx_done;
    while (!tx_done && esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(5));
        if (s_lora_tx_done) {
            tx_done = true;
            break;
        }
        if (esp_timer_get_time() >= poll_after) {
            ral_irq_t irq = 0;
            ral_get_and_clear_irq_status(&(s_modem_radio.ral), &irq);
            if ((irq & RAL_IRQ_TX_DONE) == RAL_IRQ_TX_DONE) {
                ESP_LOGW(TAG, "LoRa TX_DONE via IRQ poll (0x%x) — DIO pin interrupt not firing", (unsigned)irq);
                tx_done = true;
            }
        }
    }
    if (!tx_done) {
        ESP_LOGW(TAG, "LoRa TX no TX_DONE within %dms", LORA_TX_WAIT_MS);
    }
    /* Radio stays initialized (s_lora_ready == true); after TX_DONE the chip
     * auto-falls-back to STDBY, and the next cycle's ralf_setup_lora()
     * reconfigures it. No per-cycle sleep/re-init — see peripherals_init_all(). */
done:
    xSemaphoreGive(s_lora_lock);
    return ret;
}

void lora_tx_suspend(void *ctx)
{
    (void)ctx;
    xSemaphoreTake(s_lora_lock, portMAX_DELAY);
    lora_sleep_locked();
    xSemaphoreGive(s_lora_lock);
}

esp_err_t lora_rx_init(void *ctx)
{
    (void)ctx;
    return ensure_lora_ready();
}

esp_err_t lora_rx_step(void *ctx)
{
    (void)ctx;
    ralf_params_lora_t rx_lora_param = {
        .sync_word = 0x12,
        .symb_nb_timeout = 0,
        .rf_freq_in_hz = 915000000,
        .output_pwr_in_dbm = 0,
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

    xSemaphoreTake(s_lora_lock, portMAX_DELAY);
    esp_err_t ret = ensure_lora_ready();
    if (ret != ESP_OK) {
        goto done;
    }
    if (ralf_setup_lora(&s_modem_radio, &rx_lora_param) != RAL_STATUS_OK ||
        ral_set_dio_irq_params(&(s_modem_radio.ral), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT) != RAL_STATUS_OK) {
        ret = ESP_FAIL;
        goto done;
    }

    s_lora_rx_done = false;
    s_lora_rx_timeout = false;
    ral_set_rx(&(s_modem_radio.ral), 100);  // 100ms

    /* Fast path: DIO pin interrupt. Fallback: poll the IRQ register so RX
     * still completes if the interrupt isn't firing (mirrors lora_tx_step). */
    int64_t deadline = esp_timer_get_time() + (int64_t)LORA_RX_WAIT_MS * 1000;
    int64_t poll_after = esp_timer_get_time() + 200 * 1000;
    while (!s_lora_rx_done && !s_lora_rx_timeout && esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(5));
        if (s_lora_rx_done || s_lora_rx_timeout) {
            break;
        }
        if (esp_timer_get_time() >= poll_after) {
            ral_irq_t irq = 0;
            ral_get_and_clear_irq_status(&(s_modem_radio.ral), &irq);
            if ((irq & RAL_IRQ_RX_DONE) == RAL_IRQ_RX_DONE) {
                s_lora_rx_done = true;
            } else if ((irq & RAL_IRQ_RX_TIMEOUT) == RAL_IRQ_RX_TIMEOUT) {
                s_lora_rx_timeout = true;
            }
            if (s_lora_rx_done || s_lora_rx_timeout) {
                ESP_LOGW(TAG, "LoRa RX via IRQ poll (0x%x) — DIO pin interrupt not firing", (unsigned)irq);
            }
        }
    }

    if (s_lora_rx_done) {
        uint8_t rx_buf[256];
        uint16_t rx_size = 0;
        if (ral_get_pkt_payload(&(s_modem_radio.ral), sizeof(rx_buf), rx_buf, &rx_size) == RAL_STATUS_OK && rx_size > 0) {
            ESP_LOGI(TAG, "LoRa RX len=%u", (unsigned)rx_size);
            ESP_LOG_BUFFER_HEX(TAG, rx_buf, rx_size);
        }
    } else {
        ESP_LOGI(TAG, "LoRa RX timeout");
    }
    /* Radio stays initialized between cycles; no per-cycle sleep/re-init. */
done:
    xSemaphoreGive(s_lora_lock);
    return ret;
}

void lora_rx_suspend(void *ctx)
{
    (void)ctx;
    lora_tx_suspend(ctx); /* identical: sleep the shared radio */
}

/* -------------------- SD card -------------------- */

esp_err_t sdcard_init(void *ctx)
{
    (void)ctx;
    if (s_sd_mounted) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(bsp_sdcard_mount(), TAG, "sdcard mount failed");
    s_sd_mounted = true;
    return ESP_OK;
}

esp_err_t sdcard_step(void *ctx)
{
    (void)ctx;
    if (!s_sd_mounted) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Internal + DMA-capable so the SDMMC driver DMAs straight into buf.
     * A PSRAM buffer forces the driver to allocate an internal DMA bounce
     * buffer per transfer, which fails ("esp_dma_capable_malloc: Not enough
     * heap memory" → sdmmc_read_blocks 0x101) under concurrent DMA load
     * (WiFi/BLE/audio/many tasks). 4 KB is small enough for internal RAM. */
    uint8_t *buf = heap_caps_malloc(SDCARD_BLOCK_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    for (size_t i = 0; i < SDCARD_BLOCK_SIZE; i++) {
        buf[i] = (uint8_t)(i & 0xFF);
    }

    esp_err_t ret = ESP_OK;
    FILE *fp = fopen(SDCARD_TEST_PATH, "wb");
    if (fp == NULL) {
        free(buf);
        return ESP_FAIL;
    }
    fwrite(buf, 1, SDCARD_BLOCK_SIZE, fp);
    fclose(fp);

    fp = fopen(SDCARD_TEST_PATH, "rb");
    if (fp == NULL) {
        free(buf);
        return ESP_FAIL;
    }
    size_t rd = fread(buf, 1, SDCARD_BLOCK_SIZE, fp);
    fclose(fp);

    if (rd != SDCARD_BLOCK_SIZE) {
        ret = ESP_ERR_INVALID_SIZE;
    }
    free(buf);

    ESP_LOGI(TAG, "SD r/w %u bytes ok", (unsigned)SDCARD_BLOCK_SIZE);
    return ret;
}

void sdcard_suspend(void *ctx)
{
    (void)ctx;
    if (s_sd_mounted) {
        remove(SDCARD_TEST_PATH);
        bsp_sdcard_unmount();
        s_sd_mounted = false;
    }
}

/* -------------------- Battery ADC -------------------- */

static const char *battery_charge_status(void)
{
    uint32_t charger_in = bsp_exp_input_io_get_level(BSP_CHARGER_IN);
    if ((charger_in & BSP_CHARGER_IN) == 0U) {
        return "USB disconnected";
    }

    uint32_t charger_stat = bsp_exp_input_io_get_level(BSP_CHARGER_STAT);
    return ((charger_stat & BSP_CHARGER_STAT) != 0U) ? "USB charge complete" : "USB charging";
}

esp_err_t battery_init(void *ctx)
{
    (void)ctx;
    bsp_bat_power_control(true);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t battery_step(void *ctx)
{
    (void)ctx;
    uint16_t voltage = bsp_battery_voltage_read();
    ESP_LOGI(TAG, "Battery %u mV, status=%s", (unsigned)voltage, battery_charge_status());
    return ESP_OK;
}

void battery_suspend(void *ctx)
{
    (void)ctx;
    bsp_bat_power_control(false);
}

/* ============================================================= */
/*                       BOOT HELPERS                            */
/* ============================================================= */

esp_err_t peripherals_sdcard_mount_boot(void)
{
    ESP_LOGI(TAG, "SD boot mount: begin");
    bool intact_before = heap_caps_check_integrity_all(true);
    ESP_LOGI(TAG, "SD boot mount: heap before intact=%d free=%u largest=%u",
             intact_before ? 1 : 0,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    esp_err_t ret = bsp_sdcard_mount();
    ESP_LOGI(TAG, "SD boot mount: bsp_sdcard_mount returned %s", esp_err_to_name(ret));
    bool intact_after = heap_caps_check_integrity_all(true);
    ESP_LOGI(TAG, "SD boot mount: heap after intact=%d free=%u largest=%u",
             intact_after ? 1 : 0,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    if (ret == ESP_OK) {
        s_sd_mounted = true;
    } else {
        ESP_LOGW(TAG, "SD card mount failed at boot: %s", esp_err_to_name(ret));
    }
    return ret;
}

void peripherals_load_media(void)
{
    if (!s_sd_mounted) {
        ESP_LOGW(TAG, "SD not mounted, skipping media cache");
        return;
    }
    esp_err_t r = load_image_cache();
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "Image cache skipped: %s", esp_err_to_name(r));
    }
    r = load_wav_cache();
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "Audio cache skipped: %s", esp_err_to_name(r));
    }
}

/* Initialize every peripheral once at power-on. Each *_init is idempotent
 * (guarded by s_xxx_ready), so this runs up front in app_main and the runner's
 * later lazy init becomes a no-op guard-return. Failures warn-and-continue so
 * one missing device doesn't abort the whole boot — full-featured, always-on. */
/* Short raw-mic level probe: reads the ES7243E interleaved stream directly
 * (no AFE, no file) for ~0.8 s and reports the mic (ch1) / ref (ch0) peaks.
 * Used to bisect which peripheral init collapses the mic capture. Make
 * continuous sound into the mic while it runs. */
static void mic_raw_probe(const char *label)
{
    int16_t buf[256];  /* 128 stereo frames per read */
    int16_t mic_peak = 0;
    int16_t ref_peak = 0;
    for (int n = 0; n < 100; n++) {  /* ~0.8 s (100 x ~8 ms) */
        if (bsp_extra_get_feed_data(false, buf, (int)sizeof(buf)) != ESP_OK) {
            break;
        }
        for (int i = 0; i < 128; i++) {
            int16_t m = buf[i * 2];      /* even idx = ch1 (mic) after the swap */
            int16_t r = buf[i * 2 + 1];  /* odd idx  = ch0 (ref)              */
            if (m < 0) {
                m = (int16_t)(-m);
            }
            if (r < 0) {
                r = (int16_t)(-r);
            }
            if (m > mic_peak) {
                mic_peak = m;
            }
            if (r > ref_peak) {
                ref_peak = r;
            }
        }
    }
    ESP_LOGI(TAG, "PROBE %-12s raw_mic=%-6d raw_ref=%-6d", label, (int)mic_peak, (int)ref_peak);
}

/* One-boot auto-bisection of the mic capture. Starts from codec-only (the
 * known-good audio-only state), probes raw_mic, then enables one more
 * peripheral at a time and re-probes. The step where raw_mic collapses names
 * the culprit. Make CONTINUOUS sound into the mic for the whole ~15 s run. */
void peripherals_mic_bisect(void)
{
    ESP_LOGI(TAG, "=== mic capture bisection: make CONTINUOUS sound into the mic ===");

    audio_init(NULL);
    (void)bsp_extra_codec_set_mic_gain(AUDIO_RECORD_MIC_GAIN_DB);  /* max PGA */
    vTaskDelay(pdMS_TO_TICKS(200));  /* let the codec settle */
    mic_raw_probe("audio-only");

    display_init(NULL);  mic_raw_probe("+display");
    battery_init(NULL);  mic_raw_probe("+battery");
    sensor_init(NULL);   mic_raw_probe("+sensor");
    imu_init(NULL);      mic_raw_probe("+imu");
    gnss_init(NULL);     mic_raw_probe("+gnss");
    lora_tx_init(NULL);  mic_raw_probe("+lora_tx");
    lora_rx_init(NULL);  mic_raw_probe("+lora_rx");
    sdcard_init(NULL);   mic_raw_probe("+sdcard");

    /* Test the fix: re-initialize the codec LAST, after every other peripheral
     * (so the ES7243E is the final I2C_1 device configured, after the sensor/
     * IMU bus activity that collapses the mic). If this probe's raw_mic jumps
     * back up, "audio init last" is the fix. */
    audio_suspend(NULL);
    audio_init(NULL);
    (void)bsp_extra_codec_set_mic_gain(AUDIO_RECORD_MIC_GAIN_DB);
    vTaskDelay(pdMS_TO_TICKS(200));
    mic_raw_probe("audio-REINIT");

    ESP_LOGI(TAG, "=== bisection done — the step where raw_mic collapses is the culprit ===");
}

void peripherals_init_all(void)
{
    ESP_LOGI(TAG, "Initializing all peripherals once at power-on");

    static const struct {
        const char *name;
        esp_err_t (*fn)(void *);
    } inits[] = {
        { "display", display_init },
        { "battery", battery_init },
        { "sensor",  sensor_init  },
        { "imu",     imu_init     },
        { "wifi",    wifi_init    },
        { "ble",     ble_init     },
        { "gnss",    gnss_init    },
        { "lora_tx", lora_tx_init }, /* brings up the shared radio via ensure_lora_ready */
        { "lora_rx", lora_rx_init },
        { "sdcard",  sdcard_init  }, /* no-op: SD mounted at boot */
        /* audio LAST: the ES7243E shares I2C_1 with sensor/IMU, whose bus
         * activity collapses the mic capture. Configuring the codec after all
         * other I2C_1 devices gives a clean starting state. (audio_record_step
         * still re-inits before each take to handle per-cycle sensor/imu reads.) */
        { "audio",   audio_init   },
    };

    for (size_t i = 0; i < sizeof(inits) / sizeof(inits[0]); i++) {
        bool intact_before = heap_caps_check_integrity_all(true);
        ESP_LOGI(TAG, "INIT %-8s begin heap_intact=%d free=%u largest=%u",
                 inits[i].name,
                 intact_before ? 1 : 0,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        esp_err_t r = inits[i].fn(NULL);
        bool intact_after = heap_caps_check_integrity_all(true);
        ESP_LOGI(TAG, "INIT %-8s end ret=%s heap_intact=%d free=%u largest=%u",
                 inits[i].name,
                 esp_err_to_name(r),
                 intact_after ? 1 : 0,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "%s init failed at boot: %s", inits[i].name, esp_err_to_name(r));
        }
    }
}

void peripherals_create_lora_irq_task(void)
{
    if (s_radio_dio_task_handle == NULL) {
        xTaskCreate(radio_irq_task, "lora_dio", 4096, NULL, 3, &s_radio_dio_task_handle);
    }
}

void peripherals_cleanup(void *ctx)
{
    (void)ctx;

    wifi_suspend(NULL);
    ble_adv_test_deinit();
    gnss_suspend(NULL);

    sensor_suspend(NULL);
    imu_suspend(NULL);

    xSemaphoreTake(s_lora_lock, portMAX_DELAY);
    lora_sleep_locked();
    if (s_radio_dio_isr_ready) {
        gpio_isr_handler_remove(CONFIG_RADIO_INT_GPIO);
        gpio_reset_pin(CONFIG_RADIO_INT_GPIO);
        s_radio_dio_isr_ready = false;
    }
    xSemaphoreGive(s_lora_lock);

    audio_suspend(NULL);

    if (s_display_ready) {
        lcd_bl_off();
        bsp_display_deinit();
        s_display_ready = false;
    }

    sdcard_suspend(NULL);
    battery_suspend(NULL);
}

/* ============================================================= */
/*                       CONSOLE: audio_mode                     */
/* ============================================================= */

static struct {
    struct arg_str *mode;
    struct arg_end *end;
} audio_mode_args;

static int audio_mode_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&audio_mode_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, audio_mode_args.end, argv[0]);
        return 1;
    }

    const char *m = audio_mode_args.mode->sval[0];
    audio_mode_t new_mode;
    if (strcasecmp(m, "play") == 0) {
        new_mode = AUDIO_MODE_PLAY;
    } else if (strcasecmp(m, "record") == 0) {
        new_mode = AUDIO_MODE_RECORD;
    } else if (strcasecmp(m, "both") == 0) {
        new_mode = AUDIO_MODE_BOTH;
    } else {
        printf("unknown mode '%s' (use play|record|both)\n", m);
        return 1;
    }

    s_audio_mode = new_mode;
    const char *name = (new_mode == AUDIO_MODE_PLAY) ? "play"
                       : (new_mode == AUDIO_MODE_RECORD) ? "record" : "both";
    printf("audio mode = %s\n", name);
    return 0;
}

void audio_cmd_register(void)
{
    audio_mode_args.mode = arg_str1(NULL, NULL, "<play|record|both>", "audio job mode");
    audio_mode_args.end  = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "audio_mode",
        .help = "Set audio job mode: audio_mode <play|record|both>",
        .hint = NULL,
        .func = &audio_mode_cmd,
        .argtable = &audio_mode_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
    ESP_LOGI(TAG, "audio command registered: audio_mode <play|record|both>");
}

/* ====================== Mutex lazy init (called from main) ====================== */

/* Declared here (not in header) — main calls it before registering jobs. */
esp_err_t peripherals_init_locks(void)
{
    if (s_i2c1_lock == NULL) {
        s_i2c1_lock = xSemaphoreCreateMutex();
    }
    if (s_lora_lock == NULL) {
        s_lora_lock = xSemaphoreCreateMutex();
    }
    return (s_i2c1_lock && s_lora_lock) ? ESP_OK : ESP_ERR_NO_MEM;
}
