#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "meshpager_x2.h"
#include "bsp_board_extra.h"
#include "app_storage.h"

static const char *TAG = "AUDIO_TEST";

#define AUDIO_TEST_SD_MOUNT_POINT         "/sdcard"
/* SPIFFS asset-partition fallback paths used when the SD card is unavailable
 * at boot. Audio assets live under <SPIFFS>/audio on the baked-in image. */
#define AUDIO_TEST_SPIFFS_PLAY_FILE      BSP_SPIFFS_MOUNT_POINT "/audio/test.mp3"
#define AUDIO_TEST_SPIFFS_RECORD_FILE    BSP_SPIFFS_MOUNT_POINT "/audio/mic_record.wav"
#define AUDIO_TEST_SPIFFS_LOOPBACK_FILE  BSP_SPIFFS_MOUNT_POINT "/audio/1.wav"
#define AUDIO_TEST_DEFAULT_PLAY_FILE      (app_storage_sd_available() ? (AUDIO_TEST_SD_MOUNT_POINT "/test.mp3")      : AUDIO_TEST_SPIFFS_PLAY_FILE)
#define AUDIO_TEST_DEFAULT_RECORD_FILE   (app_storage_sd_available() ? (AUDIO_TEST_SD_MOUNT_POINT "/mic_record.wav") : AUDIO_TEST_SPIFFS_RECORD_FILE)
#define AUDIO_TEST_LOOPBACK_PLAY_FILE     (app_storage_sd_available() ? (AUDIO_TEST_SD_MOUNT_POINT "/1.wav")        : AUDIO_TEST_SPIFFS_LOOPBACK_FILE)

#define AUDIO_TEST_SAMPLE_RATE_HZ         (16000)
#define AUDIO_TEST_BITS_PER_SAMPLE        (16)
#define AUDIO_TEST_SPK_CHANNEL_MODE       I2S_SLOT_MODE_STEREO
#define AUDIO_TEST_MIC_CHANNEL_MODE       I2S_SLOT_MODE_STEREO
#define AUDIO_TEST_MIC_RAW_CHANNEL_COUNT  (CODEC_DEFAULT_ADC_CHANNEL)
#define AUDIO_TEST_MIC_OUTPUT_CHANNEL_COUNT (1)
#define AUDIO_TEST_CHUNK_SAMPLES          (256)
#define AUDIO_TEST_RECORD_DURATION_MS     (10000)
#define AUDIO_TEST_RECORD_MAX_DURATION_MS (60000)
#define AUDIO_TEST_LOOPBACK_RECORD_DURATION_MS (5000)
#define AUDIO_TEST_LOOPBACK_DEFAULT_VOLUME (28)
#define AUDIO_TEST_LOOPBACK_MAX_VOLUME    (100)
#define AUDIO_TEST_DEFAULT_AFE_LINEAR_GAIN (1.0f)
#define AUDIO_TEST_AFE_LINEAR_GAIN_MIN     (1.0f)
#define AUDIO_TEST_AFE_LINEAR_GAIN_MAX     (10.0f)
#define AUDIO_TEST_DEFAULT_MIC_GAIN        (37.5f)
#define AUDIO_TEST_MIC_GAIN_MIN            (20.0f)
#define AUDIO_TEST_MIC_GAIN_MAX            (37.5f)

static struct {
    struct arg_str *path;
    struct arg_int *volume;
    struct arg_end *end;
} speaker_play_args;

static struct {
    struct arg_str *output;
    struct arg_int *duration;
    struct arg_dbl *linear_gain;
    struct arg_dbl *mic_gain;
    struct arg_end *end;
} microphone_read_args;

static struct {
    struct arg_int *volume;
    struct arg_dbl *linear_gain;
    struct arg_end *end;
} audio_loopback_args;

static struct {
    struct arg_str *mode;
    struct arg_end *end;
} afe_input_format_args;

static struct {
    struct arg_str *source;
    struct arg_end *end;
} afe_reference_source_args;

static int16_t s_record_reference_buffer[AUDIO_TEST_CHUNK_SAMPLES];
static QueueHandle_t s_audio_event_queue;
static char s_speaker_play_path[PATH_MAX];
static char s_microphone_output_path[PATH_MAX];
static char s_audio_loopback_play_path[PATH_MAX];
static char s_audio_loopback_raw_path[PATH_MAX];
static char s_audio_loopback_afe_path[PATH_MAX];
static char s_audio_loopback_ref_path[PATH_MAX];

typedef struct {
    const esp_afe_sr_iface_t *afe_handle;
    esp_afe_sr_data_t *afe_data;
    int feed_chunk_samples;
    int fetch_chunk_samples;
    int feed_frames_per_fetch;
    int feed_channel_count;
    int16_t *feed_buffer;
    FILE *raw_fp;
    FILE *afe_fp;
    FILE *ref_fp;
} audio_afe_capture_t;

static void audio_extract_channel(const int16_t *interleaved,
                                  size_t interleaved_bytes,
                                  int channel_count,
                                  int channel_index,
                                  int16_t *output)
{
    size_t frame_count;

    if (interleaved == NULL || output == NULL || channel_count <= 0 || channel_index < 0 || channel_index >= channel_count) {
        return;
    }

    frame_count = interleaved_bytes / (sizeof(int16_t) * (size_t)channel_count);
    for (size_t i = 0; i < frame_count; i++) {
        output[i] = interleaved[i * (size_t)channel_count + (size_t)channel_index];
    }
}

static void audio_close_file(FILE **fp)
{
    if (fp != NULL && *fp != NULL) {
        fclose(*fp);
        *fp = NULL;
    }
}

static void audio_afe_capture_cleanup(audio_afe_capture_t *capture)
{
    if (capture == NULL) {
        return;
    }

    audio_close_file(&capture->raw_fp);
    audio_close_file(&capture->afe_fp);
    audio_close_file(&capture->ref_fp);

    if (capture->afe_handle != NULL && capture->afe_data != NULL) {
        capture->afe_handle->destroy(capture->afe_data);
        capture->afe_data = NULL;
    }

    if (capture->feed_buffer != NULL) {
        free(capture->feed_buffer);
        capture->feed_buffer = NULL;
    }

    capture->afe_handle = NULL;
}

static esp_err_t audio_afe_capture_init(audio_afe_capture_t *capture, float linear_gain)
{
    afe_config_t *afe_config = NULL;
    int feed_buffer_samples;

    if (capture == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(capture, 0, sizeof(*capture));
    afe_config = afe_config_init(bsp_extra_get_input_format(), NULL, AFE_TYPE_VC, AFE_MODE_HIGH_PERF);
    if (afe_config == NULL) {
        return ESP_ERR_NO_MEM;
    }

    afe_config->se_init = false;
    afe_config->vad_init = false;
    afe_config->wakenet_init = false;
    afe_config->aec_init = true;
    afe_config->agc_init = true;
    afe_config->agc_mode = AFE_AGC_MODE_WEBRTC;
    afe_config->agc_compression_gain_db = 20;   // 20
    afe_config->agc_target_level_dbfs = 1;      // 1
    afe_config->ns_init = true;
    afe_config->afe_linear_gain = linear_gain;
    afe_config->fixed_output_channel = true;
    afe_config->output_playback_channel = false;

    capture->afe_handle = esp_afe_handle_from_config(afe_config);
    if (capture->afe_handle == NULL) {
        afe_config_free(afe_config);
        return ESP_FAIL;
    }

    capture->afe_data = capture->afe_handle->create_from_config(afe_config);
    afe_config_free(afe_config);
    if (capture->afe_data == NULL) {
        audio_afe_capture_cleanup(capture);
        return ESP_FAIL;
    }

    capture->feed_chunk_samples = capture->afe_handle->get_feed_chunksize(capture->afe_data);
    capture->fetch_chunk_samples = capture->afe_handle->get_fetch_chunksize(capture->afe_data);
    capture->feed_frames_per_fetch = (capture->fetch_chunk_samples + capture->feed_chunk_samples - 1) / capture->feed_chunk_samples;
    capture->feed_channel_count = bsp_extra_get_feed_channel();
    if (capture->feed_channel_count != capture->afe_handle->get_feed_channel_num(capture->afe_data)) {
        ESP_LOGE(TAG,
                 "AFE feed channel mismatch: board=%d afe=%d",
                 capture->feed_channel_count,
                 capture->afe_handle->get_feed_channel_num(capture->afe_data));
        audio_afe_capture_cleanup(capture);
        return ESP_ERR_INVALID_STATE;
    }

    feed_buffer_samples = capture->feed_chunk_samples * capture->feed_channel_count;
    capture->feed_buffer = calloc((size_t)feed_buffer_samples, sizeof(int16_t));
    if (capture->feed_buffer == NULL) {
        audio_afe_capture_cleanup(capture);
        return ESP_ERR_NO_MEM;
    }

    if (capture->afe_handle->reset_buffer != NULL) {
        capture->afe_handle->reset_buffer(capture->afe_data);
    }

    return ESP_OK;
}

static esp_err_t audio_resolve_sd_path(const char *input_path, char *resolved_path, size_t resolved_path_size)
{
    size_t written;

    if (input_path == NULL || resolved_path == NULL || resolved_path_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Already-qualified SPIFFS asset path (used as the no-SD fallback): keep
     * it verbatim so the baked-in asset is reached instead of being remounted
     * under /sdcard. */
    if (strncmp(input_path, BSP_SPIFFS_MOUNT_POINT "/", strlen(BSP_SPIFFS_MOUNT_POINT) + 1) == 0 ||
        strcmp(input_path, BSP_SPIFFS_MOUNT_POINT) == 0) {
        written = strlcpy(resolved_path, input_path, resolved_path_size);
    } else if (strncmp(input_path, AUDIO_TEST_SD_MOUNT_POINT "/", strlen(AUDIO_TEST_SD_MOUNT_POINT) + 1) == 0) {
        written = strlcpy(resolved_path, input_path, resolved_path_size);
    } else if (strcmp(input_path, AUDIO_TEST_SD_MOUNT_POINT) == 0) {
        written = strlcpy(resolved_path, input_path, resolved_path_size);
    } else if (input_path[0] == '/') {
        written = snprintf(resolved_path, resolved_path_size, "%s%s", AUDIO_TEST_SD_MOUNT_POINT, input_path);
    } else {
        written = snprintf(resolved_path, resolved_path_size, "%s/%s", AUDIO_TEST_SD_MOUNT_POINT, input_path);
    }

    if (written >= resolved_path_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

static esp_err_t audio_check_existing_file(const char *path)
{
    struct stat st;

    if (stat(path, &st) != 0) {
        ESP_LOGE(TAG, "Cannot access %s: errno=%d (%s)", path, errno, strerror(errno));
        return ESP_FAIL;
    }

    if (!S_ISREG(st.st_mode)) {
        ESP_LOGE(TAG, "Path is not a regular file: %s", path);
        return ESP_FAIL;
    }

    return ESP_OK;
}

typedef struct __attribute__((packed)) {
    char chunk_id[4];
    uint32_t chunk_size;
    char format[4];
    char subchunk1_id[4];
    uint32_t subchunk1_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char subchunk2_id[4];
    uint32_t subchunk2_size;
} wav_header_t;

static esp_err_t audio_require_ready(void)
{
    esp_err_t ret = bsp_extra_codec_init();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio codec: %s", esp_err_to_name(ret));
    }

    return ret;
}

static esp_err_t audio_prepare_speaker(int volume)
{
    esp_err_t ret;

    ret = audio_require_ready();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = bsp_extra_codec_volume_set(volume, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set speaker volume: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = bsp_extra_codec_mute_set(false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmute speaker: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

static esp_err_t audio_prepare_microphone(void)
{
    return audio_require_ready();
}

static void audio_player_event_cb(audio_player_cb_ctx_t *ctx)
{
    if (ctx == NULL || s_audio_event_queue == NULL) {
        return;
    }

    audio_player_callback_event_t event = ctx->audio_event;
    if (uxQueueSpacesAvailable(s_audio_event_queue) == 0) {
        audio_player_callback_event_t dropped_event;
        xQueueReceive(s_audio_event_queue, &dropped_event, 0);
    }
    xQueueSend(s_audio_event_queue, &event, 0);
}

static esp_err_t audio_require_player_ready(void)
{
    esp_err_t ret;

    ret = audio_prepare_speaker(bsp_extra_codec_volume_get());
    if (ret != ESP_OK) {
        return ret;
    }

    ret = bsp_extra_player_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio player: %s", esp_err_to_name(ret));
        return ret;
    }

    if (s_audio_event_queue == NULL) {
        s_audio_event_queue = xQueueCreate(8, sizeof(audio_player_callback_event_t));
        if (s_audio_event_queue == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    bsp_extra_player_register_callback(audio_player_event_cb, NULL);
    return ESP_OK;
}

static wav_header_t audio_build_wav_header(uint32_t data_size, uint16_t num_channels)
{
    wav_header_t header = {
        .chunk_id = { 'R', 'I', 'F', 'F' },
        .chunk_size = 36U + data_size,
        .format = { 'W', 'A', 'V', 'E' },
        .subchunk1_id = { 'f', 'm', 't', ' ' },
        .subchunk1_size = 16,
        .audio_format = 1,
        .num_channels = num_channels,
        .sample_rate = AUDIO_TEST_SAMPLE_RATE_HZ,
        .byte_rate = AUDIO_TEST_SAMPLE_RATE_HZ * num_channels * (AUDIO_TEST_BITS_PER_SAMPLE / 8U),
        .block_align = num_channels * (AUDIO_TEST_BITS_PER_SAMPLE / 8U),
        .bits_per_sample = AUDIO_TEST_BITS_PER_SAMPLE,
        .subchunk2_id = { 'd', 'a', 't', 'a' },
        .subchunk2_size = data_size,
    };

    return header;
}

static esp_err_t audio_write_wav_header(FILE *fp, uint32_t data_size, uint16_t num_channels)
{
    wav_header_t header;

    if (fp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    header = audio_build_wav_header(data_size, num_channels);
    if (fseek(fp, 0, SEEK_SET) != 0) {
        return ESP_FAIL;
    }

    if (fwrite(&header, 1, sizeof(header), fp) != sizeof(header)) {
        return ESP_FAIL;
    }

    return fflush(fp) == 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t audio_wait_for_playback_finished(const char *path)
{
    bool saw_playing = false;
    TickType_t start_tick = xTaskGetTickCount();

    while (true) {
        audio_player_callback_event_t event;

        if (xQueueReceive(s_audio_event_queue, &event, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (event == AUDIO_PLAYER_CALLBACK_EVENT_PLAYING || event == AUDIO_PLAYER_CALLBACK_EVENT_COMPLETED_PLAYING_NEXT) {
                saw_playing = true;
            } else if (event == AUDIO_PLAYER_CALLBACK_EVENT_UNKNOWN_FILE_TYPE) {
                ESP_LOGE(TAG, "Unsupported audio file type: %s", path);
                return ESP_FAIL;
            } else if (event == AUDIO_PLAYER_CALLBACK_EVENT_SHUTDOWN) {
                ESP_LOGE(TAG, "Audio player shut down while playing: %s", path);
                return ESP_FAIL;
            } else if (event == AUDIO_PLAYER_CALLBACK_EVENT_IDLE && saw_playing) {
                break;
            }
        }

        if (audio_player_get_state() == AUDIO_PLAYER_STATE_PLAYING) {
            saw_playing = true;
        } else if (audio_player_get_state() == AUDIO_PLAYER_STATE_IDLE && saw_playing) {
            break;
        } else if (!saw_playing && (xTaskGetTickCount() - start_tick) > pdMS_TO_TICKS(5000)) {
            ESP_LOGE(TAG, "Speaker playback did not start within timeout: %s", path);
            return ESP_ERR_TIMEOUT;
        }
    }

    return ESP_OK;
}

static esp_err_t audio_build_loopback_output_path(const char *play_path, const char *suffix, char *output_path, size_t output_path_size)
{
    const char *file_name;
    const char *extension;
    size_t directory_len;
    size_t stem_len;

    if (play_path == NULL || suffix == NULL || output_path == NULL || output_path_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    file_name = strrchr(play_path, '/');
    file_name = (file_name == NULL) ? play_path : (file_name + 1);
    extension = strrchr(file_name, '.');
    directory_len = (size_t)(file_name - play_path);
    stem_len = (extension == NULL) ? strlen(file_name) : (size_t)(extension - file_name);

    if (stem_len == 0 || directory_len >= output_path_size) {
        return ESP_ERR_INVALID_ARG;
    }

    if (snprintf(output_path,
                 output_path_size,
                 "%.*s%.*s_%s.wav",
                 (int)directory_len,
                 play_path,
                 (int)stem_len,
                 file_name,
                 suffix) >= (int)output_path_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

static int speaker_play_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&speaker_play_args);
    const char *path_arg;
    int volume;
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, speaker_play_args.end, argv[0]);
        return 1;
    }

    path_arg = speaker_play_args.path->count ? speaker_play_args.path->sval[0] : AUDIO_TEST_DEFAULT_PLAY_FILE;
    volume = speaker_play_args.volume->count ? speaker_play_args.volume->ival[0] : bsp_extra_codec_volume_get();

    if (volume < 0 || volume > 100) {
        ESP_LOGE(TAG, "Volume must be 0-100");
        return 1;
    }

    ret = audio_resolve_sd_path(path_arg, s_speaker_play_path, sizeof(s_speaker_play_path));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Invalid play path: %s", path_arg);
        return 1;
    }

    ret = audio_check_existing_file(s_speaker_play_path);
    if (ret != ESP_OK) {
        return 1;
    }

    ret = audio_require_player_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = bsp_extra_codec_volume_set(volume, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set speaker volume: %s", esp_err_to_name(ret));
        return 1;
    }

    bsp_extra_playback_rms_reset();
    bsp_extra_playback_rms_enable(true);

    xQueueReset(s_audio_event_queue);

    ret = bsp_extra_player_play_file(s_speaker_play_path);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Speaker file play failed: %s", esp_err_to_name(ret));
        bsp_extra_playback_rms_enable(false);
        return 1;
    }

    ret = audio_wait_for_playback_finished(s_speaker_play_path);
    bsp_extra_playback_rms_enable(false);
    if (ret != ESP_OK) {
        return 1;
    }

    uint64_t rms_sum_sq = 0;
    uint32_t rms_count = 0;
    esp_err_t rms_ret = bsp_extra_playback_rms_get(&rms_sum_sq, &rms_count);
    if (rms_ret == ESP_OK && rms_count > 0) {
        float rms = sqrtf((float)((double)rms_sum_sq / (double)rms_count));
        float rms_dbfs = 20.0f * log10f(rms / 32768.0f);
        ESP_LOGI(TAG, "Speaker RMS: %.1f (%u samples), %.2f dBFS", rms, (unsigned)rms_count, rms_dbfs);
    } else if (rms_ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "RMS skipped: playback bit depth is not 16-bit PCM");
    } else {
        ESP_LOGW(TAG, "RMS unavailable (no samples accumulated)");
    }

    ESP_LOGI(TAG, "Speaker playback finished: %s, volume=%d", s_speaker_play_path, volume);
    return 0;
}

static int microphone_read_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&microphone_read_args);
    const char *output_arg;
    int duration_ms;
    float linear_gain;
    float mic_gain;
    FILE *fp = NULL;
    audio_afe_capture_t capture = {0};
    size_t total_bytes_target;
    size_t total_bytes_written = 0;
    int64_t cmd_start_us = 0;
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, microphone_read_args.end, argv[0]);
        return 1;
    }

    output_arg = microphone_read_args.output->count ? microphone_read_args.output->sval[0] : AUDIO_TEST_DEFAULT_RECORD_FILE;
    duration_ms = microphone_read_args.duration->count ? microphone_read_args.duration->ival[0] : AUDIO_TEST_RECORD_DURATION_MS;
    if (duration_ms < 100 || duration_ms > AUDIO_TEST_RECORD_MAX_DURATION_MS) {
        ESP_LOGE(TAG, "Duration must be 100-%d ms", AUDIO_TEST_RECORD_MAX_DURATION_MS);
        return 1;
    }

    linear_gain = microphone_read_args.linear_gain->count ? (float)microphone_read_args.linear_gain->dval[0] : AUDIO_TEST_DEFAULT_AFE_LINEAR_GAIN;
    if (linear_gain < AUDIO_TEST_AFE_LINEAR_GAIN_MIN || linear_gain > AUDIO_TEST_AFE_LINEAR_GAIN_MAX) {
        ESP_LOGE(TAG, "Linear gain must be %.1f-%.1f", AUDIO_TEST_AFE_LINEAR_GAIN_MIN, AUDIO_TEST_AFE_LINEAR_GAIN_MAX);
        return 1;
    }
    mic_gain = microphone_read_args.mic_gain->count ? (float)microphone_read_args.mic_gain->dval[0] : AUDIO_TEST_DEFAULT_MIC_GAIN;
    if (mic_gain < AUDIO_TEST_MIC_GAIN_MIN || mic_gain > AUDIO_TEST_MIC_GAIN_MAX) {
        ESP_LOGE(TAG, "Mic gain must be %.1f-%.1f dB", AUDIO_TEST_MIC_GAIN_MIN, AUDIO_TEST_MIC_GAIN_MAX);
        return 1;
    }

    ret = audio_resolve_sd_path(output_arg, s_microphone_output_path, sizeof(s_microphone_output_path));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Invalid output path: %s", output_arg);
        return 1;
    }

    cmd_start_us = esp_timer_get_time();
    ESP_LOGI(TAG, "microphone_read start: requested=%d ms, output=%s, linear_gain=%.1f, mic_gain=%.1f dB", duration_ms, s_microphone_output_path, linear_gain, mic_gain);

    ret = audio_prepare_microphone();
    if (ret != ESP_OK) {
        return 1;
    }

    /* Drive the ES7243E mic PGA gain for this recording.
     * Must run after audio_prepare_microphone() opens the record codec. */
    esp_err_t mic_gain_ret = bsp_extra_codec_set_mic_gain(mic_gain);
    if (mic_gain_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set mic PGA: %s", esp_err_to_name(mic_gain_ret));
    } else {
        ESP_LOGI(TAG, "Mic PGA set to +%.1f dB", mic_gain);
    }

    /* Route the microphone through the AFE pipeline (NS + AGC + AEC) so the
     * recording is clean mono PCM instead of raw interleaved ADC samples. */
    ret = audio_afe_capture_init(&capture, linear_gain);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AFE capture: %s", esp_err_to_name(ret));
        return 1;
    }

    fp = fopen(s_microphone_output_path, "wb");
    if (fp == NULL) {
        ESP_LOGE(TAG, "Failed to open output file: %s, errno=%d (%s)", s_microphone_output_path, errno, strerror(errno));
        audio_afe_capture_cleanup(&capture);
        return 1;
    }

    ret = audio_write_wav_header(fp, 0, AUDIO_TEST_MIC_OUTPUT_CHANNEL_COUNT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write WAV header: %s", esp_err_to_name(ret));
        fclose(fp);
        audio_afe_capture_cleanup(&capture);
        return 1;
    }

    total_bytes_target = ((size_t)duration_ms * AUDIO_TEST_SAMPLE_RATE_HZ * AUDIO_TEST_MIC_OUTPUT_CHANNEL_COUNT * (AUDIO_TEST_BITS_PER_SAMPLE / 8U)) / 1000U;

    while (total_bytes_written < total_bytes_target) {
        afe_fetch_result_t *fetch_result;
        size_t feed_bytes = (size_t)capture.feed_chunk_samples * capture.feed_channel_count * sizeof(int16_t);
        size_t fetch_bytes;

        for (int feed_index = 0; feed_index < capture.feed_frames_per_fetch; feed_index++) {
            ret = bsp_extra_get_feed_data(false, capture.feed_buffer, (int)feed_bytes);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Microphone feed read failed: %s", esp_err_to_name(ret));
                goto cleanup;
            }

            if (capture.afe_handle->feed(capture.afe_data, capture.feed_buffer) <= 0) {
                ESP_LOGE(TAG, "AFE feed failed");
                ret = ESP_FAIL;
                goto cleanup;
            }
        }

        if (capture.afe_handle->fetch_with_delay != NULL) {
            fetch_result = capture.afe_handle->fetch_with_delay(capture.afe_data, 0);
        } else {
            fetch_result = capture.afe_handle->fetch(capture.afe_data);
        }
        if (fetch_result == NULL || fetch_result->ret_value == ESP_FAIL || fetch_result->data == NULL) {
            continue;
        }

        fetch_bytes = (size_t)fetch_result->data_size;
        if (fetch_bytes == 0) {
            continue;
        }

        if (fetch_bytes > (total_bytes_target - total_bytes_written)) {
            fetch_bytes = total_bytes_target - total_bytes_written;
        }

        if (fwrite(fetch_result->data, 1, fetch_bytes, fp) != fetch_bytes) {
            ESP_LOGE(TAG, "Failed to write audio data to SD card");
            ret = ESP_FAIL;
            goto cleanup;
        }

        total_bytes_written += fetch_bytes;
    }

    ret = ESP_OK;

cleanup:
    if (audio_write_wav_header(fp, (uint32_t)total_bytes_written, AUDIO_TEST_MIC_OUTPUT_CHANNEL_COUNT) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to finalize WAV header");
        ret = ESP_FAIL;
    }

    if (fp != NULL) {
        fclose(fp);
    }

    audio_afe_capture_cleanup(&capture);

    if (ret != ESP_OK) {
        return 1;
    }

    int64_t cmd_end_us = esp_timer_get_time();
    uint32_t elapsed_ms = (uint32_t)((cmd_end_us - cmd_start_us) / 1000);
    uint32_t audio_ms = (uint32_t)((total_bytes_written * 1000ULL) /
                                   ((uint32_t)AUDIO_TEST_SAMPLE_RATE_HZ * AUDIO_TEST_MIC_OUTPUT_CHANNEL_COUNT * (AUDIO_TEST_BITS_PER_SAMPLE / 8U)));
    ESP_LOGI(TAG,
             "Microphone record finished: %s, requested=%d ms, elapsed=%u ms, audio=%u ms, %u bytes",
             s_microphone_output_path,
             duration_ms,
             elapsed_ms,
             audio_ms,
             (unsigned)total_bytes_written);
    return 0;
}

static int audio_loopback_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&audio_loopback_args);
    int volume;
    float linear_gain;
    audio_afe_capture_t capture = {0};
    size_t total_afe_bytes_target;
    size_t total_raw_bytes_written = 0;
    size_t total_afe_bytes_written = 0;
    esp_err_t ret;
    bool saw_playing = false;
    TickType_t start_tick;

    if (nerrors != 0) {
        arg_print_errors(stderr, audio_loopback_args.end, argv[0]);
        return 1;
    }

    volume = audio_loopback_args.volume->count ? audio_loopback_args.volume->ival[0] : AUDIO_TEST_LOOPBACK_DEFAULT_VOLUME;
    if (volume < 0 || volume > AUDIO_TEST_LOOPBACK_MAX_VOLUME) {
        ESP_LOGE(TAG, "Loopback volume must be 0-%d", AUDIO_TEST_LOOPBACK_MAX_VOLUME);
        return 1;
    }

    linear_gain = audio_loopback_args.linear_gain->count ? (float)audio_loopback_args.linear_gain->dval[0] : AUDIO_TEST_DEFAULT_AFE_LINEAR_GAIN;
    if (linear_gain < AUDIO_TEST_AFE_LINEAR_GAIN_MIN || linear_gain > AUDIO_TEST_AFE_LINEAR_GAIN_MAX) {
        ESP_LOGE(TAG, "Linear gain must be %.1f-%.1f", AUDIO_TEST_AFE_LINEAR_GAIN_MIN, AUDIO_TEST_AFE_LINEAR_GAIN_MAX);
        return 1;
    }

    ret = audio_resolve_sd_path(AUDIO_TEST_LOOPBACK_PLAY_FILE, s_audio_loopback_play_path, sizeof(s_audio_loopback_play_path));
    if (ret != ESP_OK) {
        return 1;
    }
    ret = audio_check_existing_file(s_audio_loopback_play_path);
    if (ret != ESP_OK) {
        return 1;
    }

    ret = audio_build_loopback_output_path(s_audio_loopback_play_path, "raw", s_audio_loopback_raw_path, sizeof(s_audio_loopback_raw_path));
    if (ret != ESP_OK) {
        return 1;
    }

    ret = audio_build_loopback_output_path(s_audio_loopback_play_path, "afe", s_audio_loopback_afe_path, sizeof(s_audio_loopback_afe_path));
    if (ret != ESP_OK) {
        return 1;
    }

    ret = audio_build_loopback_output_path(s_audio_loopback_play_path, "ref", s_audio_loopback_ref_path, sizeof(s_audio_loopback_ref_path));
    if (ret != ESP_OK) {
        return 1;
    }

    ret = audio_prepare_microphone();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = audio_prepare_speaker(volume);
    if (ret != ESP_OK) {
        return 1;
    }

    ret = audio_require_player_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = bsp_extra_codec_play_set_fs(AUDIO_TEST_SAMPLE_RATE_HZ, AUDIO_TEST_BITS_PER_SAMPLE, AUDIO_TEST_SPK_CHANNEL_MODE);
    if (ret != ESP_OK) {
        return 1;
    }

    ret = audio_afe_capture_init(&capture, linear_gain);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AFE loopback capture: %s", esp_err_to_name(ret));
        return 1;
    }

    capture.raw_fp = fopen(s_audio_loopback_raw_path, "wb");
    if (capture.raw_fp == NULL) {
        ESP_LOGE(TAG, "Failed to open raw output file: %s, errno=%d (%s)", s_audio_loopback_raw_path, errno, strerror(errno));
        ret = ESP_FAIL;
        goto cleanup;
    }

    capture.afe_fp = fopen(s_audio_loopback_afe_path, "wb");
    if (capture.afe_fp == NULL) {
        ESP_LOGE(TAG, "Failed to open AFE output file: %s, errno=%d (%s)", s_audio_loopback_afe_path, errno, strerror(errno));
        ret = ESP_FAIL;
        goto cleanup;
    }

    capture.ref_fp = fopen(s_audio_loopback_ref_path, "wb");
    if (capture.ref_fp == NULL) {
        ESP_LOGE(TAG, "Failed to open reference output file: %s, errno=%d (%s)", s_audio_loopback_ref_path, errno, strerror(errno));
        ret = ESP_FAIL;
        goto cleanup;
    }

    ret = audio_write_wav_header(capture.raw_fp, 0, (uint16_t)capture.feed_channel_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write raw WAV header: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = audio_write_wav_header(capture.afe_fp, 0, AUDIO_TEST_MIC_OUTPUT_CHANNEL_COUNT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write AFE WAV header: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = audio_write_wav_header(capture.ref_fp, 0, AUDIO_TEST_MIC_OUTPUT_CHANNEL_COUNT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write reference WAV header: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    xQueueReset(s_audio_event_queue);

    ret = bsp_extra_player_play_file(s_audio_loopback_play_path);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Speaker file play failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    total_afe_bytes_target = ((size_t)AUDIO_TEST_LOOPBACK_RECORD_DURATION_MS * AUDIO_TEST_SAMPLE_RATE_HZ *
                              AUDIO_TEST_MIC_OUTPUT_CHANNEL_COUNT * (AUDIO_TEST_BITS_PER_SAMPLE / 8U)) / 1000U;
    start_tick = xTaskGetTickCount();

    while (total_afe_bytes_written < total_afe_bytes_target) {
        audio_player_callback_event_t event;
        afe_fetch_result_t *fetch_result;
        size_t feed_bytes = (size_t)capture.feed_chunk_samples * capture.feed_channel_count * sizeof(int16_t);
        size_t fetch_bytes;

        if (xQueueReceive(s_audio_event_queue, &event, 0) == pdTRUE) {
            if (event == AUDIO_PLAYER_CALLBACK_EVENT_PLAYING || event == AUDIO_PLAYER_CALLBACK_EVENT_COMPLETED_PLAYING_NEXT) {
                saw_playing = true;
            } else if (event == AUDIO_PLAYER_CALLBACK_EVENT_UNKNOWN_FILE_TYPE) {
                ESP_LOGE(TAG, "Unsupported audio file type: %s", s_audio_loopback_play_path);
                ret = ESP_FAIL;
                goto cleanup;
            } else if (event == AUDIO_PLAYER_CALLBACK_EVENT_SHUTDOWN) {
                ESP_LOGE(TAG, "Audio player shut down while playing: %s", s_audio_loopback_play_path);
                ret = ESP_FAIL;
                goto cleanup;
            }
        }

        if (audio_player_get_state() == AUDIO_PLAYER_STATE_PLAYING) {
            saw_playing = true;
        } else if (!saw_playing && (xTaskGetTickCount() - start_tick) > pdMS_TO_TICKS(5000)) {
            ESP_LOGE(TAG, "Speaker playback did not start within timeout: %s", s_audio_loopback_play_path);
            ret = ESP_ERR_TIMEOUT;
            goto cleanup;
        }

        for (int feed_index = 0; feed_index < capture.feed_frames_per_fetch; feed_index++) {
            size_t reference_bytes;

            ret = bsp_extra_get_feed_data(false, capture.feed_buffer, (int)feed_bytes);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Loopback feed read failed: %s", esp_err_to_name(ret));
                goto cleanup;
            }

            if (fwrite(capture.feed_buffer, 1, feed_bytes, capture.raw_fp) != feed_bytes) {
                ESP_LOGE(TAG, "Failed to write raw feed audio data to SD card");
                ret = ESP_FAIL;
                goto cleanup;
            }
            total_raw_bytes_written += feed_bytes;

            if (capture.feed_channel_count > 1) {
                audio_extract_channel(capture.feed_buffer,
                                      feed_bytes,
                                      capture.feed_channel_count,
                                      1,
                                      s_record_reference_buffer);
                reference_bytes = feed_bytes / (size_t)capture.feed_channel_count;
            } else {
                memcpy(s_record_reference_buffer, capture.feed_buffer, feed_bytes);
                reference_bytes = feed_bytes;
            }

            if (fwrite(s_record_reference_buffer, 1, reference_bytes, capture.ref_fp) != reference_bytes) {
                ESP_LOGE(TAG, "Failed to write reference audio data to SD card");
                ret = ESP_FAIL;
                goto cleanup;
            }

            if (capture.afe_handle->feed(capture.afe_data, capture.feed_buffer) <= 0) {
                ESP_LOGE(TAG, "AFE feed failed");
                ret = ESP_FAIL;
                goto cleanup;
            }
        }

        if (capture.afe_handle->fetch_with_delay != NULL) {
            fetch_result = capture.afe_handle->fetch_with_delay(capture.afe_data, 0);
        } else {
            fetch_result = capture.afe_handle->fetch(capture.afe_data);
        }
        if (fetch_result == NULL || fetch_result->ret_value == ESP_FAIL || fetch_result->data == NULL) {
            continue;
        }

        fetch_bytes = (size_t)fetch_result->data_size;
        if (fetch_bytes == 0) {
            continue;
        }

        if (fetch_bytes > (total_afe_bytes_target - total_afe_bytes_written)) {
            fetch_bytes = total_afe_bytes_target - total_afe_bytes_written;
        }

        if (fwrite(fetch_result->data, 1, fetch_bytes, capture.afe_fp) != fetch_bytes) {
            ESP_LOGE(TAG, "Failed to write AFE audio data to SD card");
            ret = ESP_FAIL;
            goto cleanup;
        }

        total_afe_bytes_written += fetch_bytes;
    }

    ret = ESP_OK;

cleanup:
    if (audio_player_get_state() != AUDIO_PLAYER_STATE_IDLE) {
        audio_player_stop();
    }

    if (capture.raw_fp != NULL) {
        esp_err_t header_ret = audio_write_wav_header(capture.raw_fp, (uint32_t)total_raw_bytes_written, (uint16_t)capture.feed_channel_count);
        if (header_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to finalize raw WAV header: %s", esp_err_to_name(header_ret));
            ret = header_ret;
        }
    }

    if (capture.afe_fp != NULL) {
        esp_err_t header_ret = audio_write_wav_header(capture.afe_fp, (uint32_t)total_afe_bytes_written, AUDIO_TEST_MIC_OUTPUT_CHANNEL_COUNT);
        if (header_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to finalize AFE WAV header: %s", esp_err_to_name(header_ret));
            ret = header_ret;
        }
    }

    if (capture.ref_fp != NULL) {
        esp_err_t header_ret = audio_write_wav_header(capture.ref_fp,
                                                      (uint32_t)(total_raw_bytes_written / (size_t)capture.feed_channel_count),
                                                      AUDIO_TEST_MIC_OUTPUT_CHANNEL_COUNT);
        if (header_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to finalize reference WAV header: %s", esp_err_to_name(header_ret));
            ret = header_ret;
        }
    }

    audio_afe_capture_cleanup(&capture);

    if (ret != ESP_OK) {
        return 1;
    }

    ESP_LOGI(TAG,
             "Audio loopback finished: play=%s, raw=%s, afe=%s, ref=%s, volume=%d, duration=%d ms, raw_bytes=%u, afe_bytes=%u",
             s_audio_loopback_play_path,
             s_audio_loopback_raw_path,
             s_audio_loopback_afe_path,
             s_audio_loopback_ref_path,
             volume,
             AUDIO_TEST_LOOPBACK_RECORD_DURATION_MS,
             (unsigned)total_raw_bytes_written,
             (unsigned)total_afe_bytes_written);
    return 0;
}

static int afe_input_format_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&afe_input_format_args);
    const char *current_mode = bsp_extra_get_afe_input_format();
    const char *board_default_mode = bsp_extra_get_input_format();

    if (nerrors != 0) {
        arg_print_errors(stderr, afe_input_format_args.end, argv[0]);
        return 1;
    }

    if (afe_input_format_args.mode->count == 0) {
        ESP_LOGI(TAG, "AFE input format current=%s board_default=%s", current_mode, board_default_mode);
        return 0;
    }

    const char *mode = afe_input_format_args.mode->sval[0];
    if (strcmp(mode, "M") != 0 && strcmp(mode, "MR") != 0) {
        ESP_LOGE(TAG, "Unsupported AFE experiment mode: %s", mode);
        return 1;
    }

    esp_err_t ret = bsp_extra_set_afe_input_format(mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to switch AFE input format to %s: %s", mode, esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG, "AFE input format set to %s, board default remains %s", bsp_extra_get_afe_input_format(),
             board_default_mode);
    return 0;
}

static int afe_reference_source_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&afe_reference_source_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, afe_reference_source_args.end, argv[0]);
        return 1;
    }

    if (afe_reference_source_args.source->count == 0) {
        ESP_LOGI(TAG, "AFE reference source current=%s", bsp_extra_get_afe_reference_source());
        return 0;
    }

    const char *source = afe_reference_source_args.source->sval[0];
    if (strcmp(source, "adc") != 0 && strcmp(source, "playback") != 0) {
        ESP_LOGE(TAG, "Unsupported AFE reference source: %s", source);
        return 1;
    }

    esp_err_t ret = bsp_extra_set_afe_reference_source(source);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to switch AFE reference source to %s: %s", source, esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG, "AFE reference source set to %s", bsp_extra_get_afe_reference_source());
    return 0;
}

static void register_speaker_play_cmd(void)
{
    speaker_play_args.path = arg_str0("p", "path", "</sdcard/file.mp3>", "Audio file path, supports wav/mp3 (defaults to SD, falls back to SPIFFS asset partition when SD is unavailable)");
    speaker_play_args.volume = arg_int0("v", "volume", "<0-93>", "Speaker volume");
    speaker_play_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "speaker_play",
        .help = "Play an audio file from SD card on the speaker",
        .hint = NULL,
        .func = &speaker_play_cmd,
        .argtable = &speaker_play_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));

    const esp_console_cmd_t legacy_cmd = {
        .command = "speaker_tone",
        .help = "Legacy alias of speaker_play",
        .hint = NULL,
        .func = &speaker_play_cmd,
        .argtable = &speaker_play_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&legacy_cmd));
}

static void register_microphone_read_cmd(void)
{
    microphone_read_args.output = arg_str0("o", "output", "</sdcard/file.wav>", "Output WAV file path (SD, or SPIFFS asset partition when SD is unavailable)");
    microphone_read_args.duration = arg_int0("d", "duration", "<100-60000>", "Capture duration in ms, default 10000");
    microphone_read_args.linear_gain = arg_dbl0("g", "linear_gain", "<1.0-10.0>", "AFE linear gain, default 1.0");
    microphone_read_args.mic_gain = arg_dbl0("m", "mic_gain", "<20.0-37.5>", "Mic PGA gain in dB, default 37.5");
    microphone_read_args.end = arg_end(5);

    const esp_console_cmd_t cmd = {
        .command = "microphone_read",
        .help = "Record microphone audio to a WAV file on SD card",
        .hint = NULL,
        .func = &microphone_read_cmd,
        .argtable = &microphone_read_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_audio_loopback_cmd(void)
{
    audio_loopback_args.volume = arg_int0("v", "volume", "<0-100>", "Loopback speaker volume, default 28");
    audio_loopback_args.linear_gain = arg_dbl0("g", "linear_gain", "<1.0-10.0>", "AFE linear gain, default 1.0");
    audio_loopback_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "audio_loopback",
        .help = "Play /sdcard/1.wav (or /spiffs/audio/1.wav with no SD) and record 5 seconds of raw and AFE microphone audio to separate WAV files",
        .hint = NULL,
        .func = &audio_loopback_cmd,
        .argtable = &audio_loopback_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_afe_input_format_cmd(void)
{
    afe_input_format_args.mode = arg_str0("m", "mode", "<M|MR>", "AFE input format experiment mode; omit to print current mode");
    afe_input_format_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "afe_input_format",
        .help = "Get or set the experimental AFE input format without reopening the codec stream",
        .hint = NULL,
        .func = &afe_input_format_cmd,
        .argtable = &afe_input_format_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_afe_reference_source_cmd(void)
{
    afe_reference_source_args.source = arg_str0("s", "source", "<adc|playback>", "MR reference source; omit to print current source");
    afe_reference_source_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "afe_reference_source",
        .help = "Get or set the experimental MR reference source",
        .hint = NULL,
        .func = &afe_reference_source_cmd,
        .argtable = &afe_reference_source_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/* ----------------------------------------------------------- Audio deinit -- */
/* Power down the audio codec: stop playback/recording, tear down the codec
 * device handles, and cut the speaker PA rail.
 *
 * Reversibility: bsp_audio_init() wires the I2S GPIO matrix exactly once at
 * boot (it early-returns once i2s_tx_chan/rx_chan exist). bsp_extra_codec_deinit()
 * drops the codec handles but leaves that I2S channel + GPIO routing intact, so
 * the next bsp_extra_codec_init() rebuilds the codec on the live I2S bus and
 * speaker_play / microphone_read / audio_loopback keep working. The default
 * invocation is therefore reversible.
 *
 * --deep additionally gpio_reset_pin()s the I2S pins to their lowest-leakage
 * reset state. Because bsp_audio_init() won't re-wire them afterwards, audio
 * commands are SILENT after --deep until the device reboots -- use it only when
 * measuring idle/standby current. To also cut the shared SEN_EN codec+sensor
 * rail and the I2C_1 bus, run `power_sen_audio_off` instead. */
static struct {
    struct arg_lit *deep;
    struct arg_end *end;
} audio_deinit_args;

static const gpio_num_t audio_i2s_pins[] = {
    BSP_ADC_I2S_MCLK,
    BSP_ADC_I2S_SCLK,
    BSP_ADC_I2S_LRLK,
    BSP_ADC_I2S_SDIN,
    BSP_DAC_I2S_SDOUT,
};

static int audio_deinit_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&audio_deinit_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, audio_deinit_args.end, argv[0]);
        return 1;
    }

    bool deep = audio_deinit_args.deep->count > 0;

    ESP_LOGI(TAG, "Deinitializing audio codec...");

    esp_err_t ret = bsp_extra_player_del();   /* stop + free the player */
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Player del: %s", esp_err_to_name(ret));
    }

    /* codec deinit also stops play/record streams and cuts the PA internally
     * (bsp_extra_pa_power_set); set the PA rail low again here for clarity. */
    ret = bsp_extra_codec_deinit();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Codec deinit: %s", esp_err_to_name(ret));
    }
    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, 0);

    if (deep) {
        ESP_LOGW(TAG, "--deep: resetting I2S pins to reset state (lowest leakage)");
        ESP_LOGW(TAG, "  audio commands will be SILENT until device reboot");
        for (size_t i = 0; i < sizeof(audio_i2s_pins) / sizeof(audio_i2s_pins[0]); i++) {
            gpio_reset_pin(audio_i2s_pins[i]);
        }
        ESP_LOGI(TAG, "Audio codec deinit (deep): codec+PA off, I2S pins reset");
    } else {
        ESP_LOGI(TAG, "Audio codec deinit: codec+PA off (I2S bus kept live, reversible)");
    }
    return 0;
}

static void register_audio_deinit_cmd(void)
{
    audio_deinit_args.deep = arg_lit0(NULL, "deep",
                                      "also reset I2S pins (lowest leakage; audio silent until reboot)");
    audio_deinit_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "audio_deinit",
        .help = "Power down audio codec + PA. --deep also resets I2S pins (audio then silent until reboot)",
        .hint = NULL,
        .func = &audio_deinit_cmd,
        .argtable = &audio_deinit_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void audio_cmd_register_all(void)
{
    register_speaker_play_cmd();
    register_microphone_read_cmd();
    register_audio_loopback_cmd();
    register_afe_input_format_cmd();
    register_afe_reference_source_cmd();
    register_audio_deinit_cmd();

    ESP_LOGI(TAG, "Audio test commands registered:");
    ESP_LOGI(TAG, "  speaker_play -p </sdcard/test.mp3> [-v <volume>]");
    ESP_LOGI(TAG, "  speaker_tone -p </sdcard/test.wav> [-v <volume>]");
    ESP_LOGI(TAG, "  microphone_read [-o </sdcard/mic_record.wav>] [-d <duration_ms>] [-g <linear_gain>] [-m <mic_gain>]");
    ESP_LOGI(TAG, "  audio_loopback [-v <volume>] [-g <linear_gain>]");
    ESP_LOGI(TAG, "  afe_input_format [-m <M|MR>]");
    ESP_LOGI(TAG, "  afe_reference_source [-s <adc|playback>]");
    ESP_LOGI(TAG, "  audio_deinit [--deep]");
}