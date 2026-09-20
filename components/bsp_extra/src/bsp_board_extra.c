/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_afe_config.h"
#include "esp_afe_sr_models.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp-bsp.h"
#include "bsp_board_extra.h"
#include "model_path.h"

static const char *TAG = "bsp_extra_board";

static esp_codec_dev_handle_t play_dev_handle;
static esp_codec_dev_handle_t record_dev_handle;
static bool s_play_dev_opened = false;
static bool s_record_dev_opened = false;
static bool s_play_fs_valid = false;
static esp_codec_dev_sample_info_t s_play_fs = {0};

static bool _is_audio_init = false;
static bool _is_player_init = false;
static int _vloume_intensity = CODEC_DEFAULT_DAC_VOLUME;
static bool s_pa_power_enabled = false;
static bool s_record_fs_afe_compatible = false;
static bool s_playback_ref_afe_compatible = false;
static size_t s_playback_ref_channel_count = 1;
static bool s_rms_enabled = false;        /* default off: zero overhead on write path */
static uint64_t s_rms_sum_sq = 0;         /* sum of squares of int16 playback samples */
static uint32_t s_rms_count = 0;          /* number of int16 playback samples accumulated */
static char s_record_afe_input_format[8] = "M";
static bool s_record_afe_use_adc_reference = true;

#define BSP_EXTRA_PA_PWR_SWITCH_DELAY_MS (10)
#define BSP_EXTRA_AFE_MODEL_PARTITION     "model"
#define BSP_EXTRA_AFE_FETCH_MAX_ATTEMPTS  (8)

typedef struct {
    bool initialized;
    const esp_afe_sr_iface_t *handle;
    esp_afe_sr_data_t *data;
    srmodel_list_t *models;
    int feed_chunksize;
    int feed_nch;
    int fetch_chunksize;
    int feed_frames_per_fetch;
    int output_credit_samples;
    size_t raw_frame_bytes;
    size_t mono_frame_bytes;
    size_t afe_feed_frame_bytes;
    size_t fetch_frame_bytes;
    int16_t *raw_frame_buffer;
    int16_t *mono_frame_buffer;
    int16_t *ref_frame_buffer;
    int16_t *afe_feed_buffer;
    int16_t *fetch_frame_buffer;
    size_t fetch_buffer_filled;
    size_t fetch_buffer_offset;
    size_t active_channel;
    int16_t *raw_output_buffer;
    size_t raw_output_capacity_samples;
    size_t raw_output_read_pos;
    size_t raw_output_write_pos;
    size_t raw_output_filled_samples;
    int16_t *raw_interleaved_output_buffer;
    size_t raw_interleaved_output_capacity_samples;
    size_t raw_interleaved_output_read_pos;
    size_t raw_interleaved_output_write_pos;
    size_t raw_interleaved_output_filled_samples;
    int16_t *playback_ref_buffer;
    size_t playback_ref_capacity_samples;
    size_t playback_ref_read_pos;
    size_t playback_ref_write_pos;
    size_t playback_ref_filled_samples;
    SemaphoreHandle_t playback_ref_lock;
} bsp_extra_afe_ctx_t;

static audio_player_cb_t audio_idle_callback = NULL;
static void *audio_idle_cb_user_data = NULL;
static char audio_file_path[128];
static bsp_extra_afe_ctx_t s_record_afe_ctx = {0};

typedef struct {
    const audio_codec_if_t      *codec_if;
    const audio_codec_data_if_t *data_if;
} bsp_extra_codec_dev_view_t;

static void bsp_extra_record_raw_output_reset(void)
{
    s_record_afe_ctx.raw_output_read_pos = 0;
    s_record_afe_ctx.raw_output_write_pos = 0;
    s_record_afe_ctx.raw_output_filled_samples = 0;
}

static void bsp_extra_record_raw_interleaved_output_reset(void)
{
    s_record_afe_ctx.raw_interleaved_output_read_pos = 0;
    s_record_afe_ctx.raw_interleaved_output_write_pos = 0;
    s_record_afe_ctx.raw_interleaved_output_filled_samples = 0;
}

static esp_err_t bsp_extra_record_raw_output_ensure(size_t sample_capacity)
{
    int16_t *new_buffer;

    if (sample_capacity == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (s_record_afe_ctx.raw_output_buffer != NULL && s_record_afe_ctx.raw_output_capacity_samples >= sample_capacity) {
        return ESP_OK;
    }

    new_buffer = realloc(s_record_afe_ctx.raw_output_buffer, sample_capacity * sizeof(int16_t));
    ESP_RETURN_ON_FALSE(new_buffer != NULL, ESP_ERR_NO_MEM, TAG, "record raw output buffer alloc failed");

    s_record_afe_ctx.raw_output_buffer = new_buffer;
    s_record_afe_ctx.raw_output_capacity_samples = sample_capacity;
    bsp_extra_record_raw_output_reset();
    return ESP_OK;
}

static esp_err_t bsp_extra_record_raw_interleaved_output_ensure(size_t sample_capacity)
{
    int16_t *new_buffer;

    if (sample_capacity == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (s_record_afe_ctx.raw_interleaved_output_buffer != NULL &&
        s_record_afe_ctx.raw_interleaved_output_capacity_samples >= sample_capacity) {
        return ESP_OK;
    }

    new_buffer = realloc(s_record_afe_ctx.raw_interleaved_output_buffer, sample_capacity * sizeof(int16_t));
    ESP_RETURN_ON_FALSE(new_buffer != NULL, ESP_ERR_NO_MEM, TAG, "record raw interleaved output buffer alloc failed");

    s_record_afe_ctx.raw_interleaved_output_buffer = new_buffer;
    s_record_afe_ctx.raw_interleaved_output_capacity_samples = sample_capacity;
    bsp_extra_record_raw_interleaved_output_reset();
    return ESP_OK;
}

static void bsp_extra_record_raw_output_push(const int16_t *samples, size_t sample_count)
{
    if (samples == NULL || sample_count == 0 || s_record_afe_ctx.raw_output_buffer == NULL ||
        s_record_afe_ctx.raw_output_capacity_samples == 0) {
        return;
    }

    for (size_t i = 0; i < sample_count; i++) {
        if (s_record_afe_ctx.raw_output_filled_samples == s_record_afe_ctx.raw_output_capacity_samples) {
            s_record_afe_ctx.raw_output_read_pos =
                (s_record_afe_ctx.raw_output_read_pos + 1) % s_record_afe_ctx.raw_output_capacity_samples;
            s_record_afe_ctx.raw_output_filled_samples--;
        }

        s_record_afe_ctx.raw_output_buffer[s_record_afe_ctx.raw_output_write_pos] = samples[i];
        s_record_afe_ctx.raw_output_write_pos =
            (s_record_afe_ctx.raw_output_write_pos + 1) % s_record_afe_ctx.raw_output_capacity_samples;
        s_record_afe_ctx.raw_output_filled_samples++;
    }
}

static void bsp_extra_record_raw_interleaved_output_push(const int16_t *samples, size_t sample_count)
{
    if (samples == NULL || sample_count == 0 || s_record_afe_ctx.raw_interleaved_output_buffer == NULL ||
        s_record_afe_ctx.raw_interleaved_output_capacity_samples == 0) {
        return;
    }

    for (size_t i = 0; i < sample_count; i++) {
        if (s_record_afe_ctx.raw_interleaved_output_filled_samples == s_record_afe_ctx.raw_interleaved_output_capacity_samples) {
            s_record_afe_ctx.raw_interleaved_output_read_pos =
                (s_record_afe_ctx.raw_interleaved_output_read_pos + 1) % s_record_afe_ctx.raw_interleaved_output_capacity_samples;
            s_record_afe_ctx.raw_interleaved_output_filled_samples--;
        }

        s_record_afe_ctx.raw_interleaved_output_buffer[s_record_afe_ctx.raw_interleaved_output_write_pos] = samples[i];
        s_record_afe_ctx.raw_interleaved_output_write_pos =
            (s_record_afe_ctx.raw_interleaved_output_write_pos + 1) % s_record_afe_ctx.raw_interleaved_output_capacity_samples;
        s_record_afe_ctx.raw_interleaved_output_filled_samples++;
    }
}

static void bsp_extra_record_raw_output_pop(int16_t *samples, size_t sample_count)
{
    if (samples == NULL || sample_count == 0) {
        return;
    }

    for (size_t i = 0; i < sample_count; i++) {
        if (s_record_afe_ctx.raw_output_filled_samples == 0) {
            break;
        }

        samples[i] = s_record_afe_ctx.raw_output_buffer[s_record_afe_ctx.raw_output_read_pos];
        s_record_afe_ctx.raw_output_read_pos =
            (s_record_afe_ctx.raw_output_read_pos + 1) % s_record_afe_ctx.raw_output_capacity_samples;
        s_record_afe_ctx.raw_output_filled_samples--;
    }
}

static void bsp_extra_record_raw_interleaved_output_pop(int16_t *samples, size_t sample_count)
{
    if (samples == NULL || sample_count == 0) {
        return;
    }

    for (size_t i = 0; i < sample_count; i++) {
        if (s_record_afe_ctx.raw_interleaved_output_filled_samples == 0) {
            break;
        }

        samples[i] = s_record_afe_ctx.raw_interleaved_output_buffer[s_record_afe_ctx.raw_interleaved_output_read_pos];
        s_record_afe_ctx.raw_interleaved_output_read_pos =
            (s_record_afe_ctx.raw_interleaved_output_read_pos + 1) % s_record_afe_ctx.raw_interleaved_output_capacity_samples;
        s_record_afe_ctx.raw_interleaved_output_filled_samples--;
    }
}

static void bsp_extra_record_playback_ref_reset(void)
{
    if (s_record_afe_ctx.playback_ref_lock == NULL) {
        return;
    }

    if (xSemaphoreTake(s_record_afe_ctx.playback_ref_lock, portMAX_DELAY) == pdTRUE) {
        s_record_afe_ctx.playback_ref_read_pos = 0;
        s_record_afe_ctx.playback_ref_write_pos = 0;
        s_record_afe_ctx.playback_ref_filled_samples = 0;
        xSemaphoreGive(s_record_afe_ctx.playback_ref_lock);
    }
}

static esp_err_t bsp_extra_record_playback_ref_ensure(size_t sample_capacity)
{
    int16_t *new_buffer;

    if (sample_capacity == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (s_record_afe_ctx.playback_ref_buffer != NULL && s_record_afe_ctx.playback_ref_capacity_samples >= sample_capacity) {
        return ESP_OK;
    }

    new_buffer = realloc(s_record_afe_ctx.playback_ref_buffer, sample_capacity * sizeof(int16_t));
    ESP_RETURN_ON_FALSE(new_buffer != NULL, ESP_ERR_NO_MEM, TAG, "playback ref buffer alloc failed");

    s_record_afe_ctx.playback_ref_buffer = new_buffer;
    s_record_afe_ctx.playback_ref_capacity_samples = sample_capacity;
    bsp_extra_record_playback_ref_reset();
    return ESP_OK;
}

// static void bsp_extra_record_playback_ref_push(const int16_t *samples, size_t sample_count)
// {
//     if (samples == NULL || sample_count == 0 || s_record_afe_ctx.playback_ref_buffer == NULL ||
//         s_record_afe_ctx.playback_ref_capacity_samples == 0 || s_record_afe_ctx.playback_ref_lock == NULL) {
//         return;
//     }

//     if (xSemaphoreTake(s_record_afe_ctx.playback_ref_lock, 0) != pdTRUE) {
//         return;
//     }

//     for (size_t i = 0; i < sample_count; i++) {
//         if (s_record_afe_ctx.playback_ref_filled_samples == s_record_afe_ctx.playback_ref_capacity_samples) {
//             s_record_afe_ctx.playback_ref_read_pos =
//                 (s_record_afe_ctx.playback_ref_read_pos + 1) % s_record_afe_ctx.playback_ref_capacity_samples;
//             s_record_afe_ctx.playback_ref_filled_samples--;
//         }

//         s_record_afe_ctx.playback_ref_buffer[s_record_afe_ctx.playback_ref_write_pos] = samples[i];
//         s_record_afe_ctx.playback_ref_write_pos =
//             (s_record_afe_ctx.playback_ref_write_pos + 1) % s_record_afe_ctx.playback_ref_capacity_samples;
//         s_record_afe_ctx.playback_ref_filled_samples++;
//     }

//     xSemaphoreGive(s_record_afe_ctx.playback_ref_lock);
// }

static void bsp_extra_record_playback_ref_pop(int16_t *samples, size_t sample_count)
{
    if (samples == NULL || sample_count == 0) {
        return;
    }

    memset(samples, 0, sample_count * sizeof(int16_t));

    if (s_record_afe_ctx.playback_ref_buffer == NULL || s_record_afe_ctx.playback_ref_capacity_samples == 0 ||
        s_record_afe_ctx.playback_ref_lock == NULL) {
        return;
    }

    if (xSemaphoreTake(s_record_afe_ctx.playback_ref_lock, portMAX_DELAY) != pdTRUE) {
        return;
    }

    for (size_t i = 0; i < sample_count; i++) {
        if (s_record_afe_ctx.playback_ref_filled_samples == 0) {
            break;
        }

        samples[i] = s_record_afe_ctx.playback_ref_buffer[s_record_afe_ctx.playback_ref_read_pos];
        s_record_afe_ctx.playback_ref_read_pos =
            (s_record_afe_ctx.playback_ref_read_pos + 1) % s_record_afe_ctx.playback_ref_capacity_samples;
        s_record_afe_ctx.playback_ref_filled_samples--;
    }

    xSemaphoreGive(s_record_afe_ctx.playback_ref_lock);
}

static void bsp_extra_record_afe_build_feed_frame(const int16_t *raw_input, const int16_t *mono_input)
{
    bool use_adc_reference = s_record_afe_use_adc_reference && CODEC_DEFAULT_ADC_CHANNEL >= 2 && raw_input != NULL;

    if (s_record_afe_ctx.feed_nch <= 1 || s_record_afe_ctx.afe_feed_buffer == NULL) {
        return;
    }

    memset(s_record_afe_ctx.afe_feed_buffer, 0, s_record_afe_ctx.afe_feed_frame_bytes);

    if (!use_adc_reference && s_record_afe_ctx.feed_nch >= 2 && s_record_afe_ctx.ref_frame_buffer != NULL) {
        bsp_extra_record_playback_ref_pop(s_record_afe_ctx.ref_frame_buffer, (size_t)s_record_afe_ctx.feed_chunksize);
    }

    if (use_adc_reference) {
        for (int sample_index = 0; sample_index < s_record_afe_ctx.feed_chunksize; sample_index++) {
            s_record_afe_ctx.afe_feed_buffer[sample_index * s_record_afe_ctx.feed_nch] =
                raw_input[sample_index * CODEC_DEFAULT_ADC_CHANNEL + 1];
            s_record_afe_ctx.afe_feed_buffer[sample_index * s_record_afe_ctx.feed_nch + 1] =
                raw_input[sample_index * CODEC_DEFAULT_ADC_CHANNEL + 0];

            for (int channel_index = 2; channel_index < s_record_afe_ctx.feed_nch; channel_index++) {
                s_record_afe_ctx.afe_feed_buffer[sample_index * s_record_afe_ctx.feed_nch + channel_index] = 0;
            }
        }
        return;
    }

    for (int sample_index = 0; sample_index < s_record_afe_ctx.feed_chunksize; sample_index++) {
        s_record_afe_ctx.afe_feed_buffer[sample_index * s_record_afe_ctx.feed_nch] = mono_input[sample_index];

        if (s_record_afe_ctx.feed_nch >= 2 && s_record_afe_ctx.ref_frame_buffer != NULL) {
            s_record_afe_ctx.afe_feed_buffer[sample_index * s_record_afe_ctx.feed_nch + 1] =
                s_record_afe_ctx.ref_frame_buffer[sample_index];
        }

        for (int channel_index = 2; channel_index < s_record_afe_ctx.feed_nch; channel_index++) {
            s_record_afe_ctx.afe_feed_buffer[sample_index * s_record_afe_ctx.feed_nch + channel_index] = 0;
        }
    }
}

// static void bsp_extra_playback_capture_reference(const void *audio_buffer, size_t len)
// {
//     const int16_t *samples = (const int16_t *)audio_buffer;
//     size_t frame_count;

//     if (!s_playback_ref_afe_compatible || audio_buffer == NULL || len < sizeof(int16_t) ||
//         (len % (sizeof(int16_t) * s_playback_ref_channel_count)) != 0) {
//         return;
//     }

//     frame_count = len / (sizeof(int16_t) * s_playback_ref_channel_count);

//     if (s_playback_ref_channel_count == 1) {
//         bsp_extra_record_playback_ref_push(samples, frame_count);
//         return;
//     }

//     for (size_t frame_index = 0; frame_index < frame_count; frame_index++) {
//         int32_t mixed = 0;

//         for (size_t channel_index = 0; channel_index < s_playback_ref_channel_count; channel_index++) {
//             mixed += samples[frame_index * s_playback_ref_channel_count + channel_index];
//         }

//         int16_t mono = (int16_t)(mixed / (int32_t)s_playback_ref_channel_count);
//         bsp_extra_record_playback_ref_push(&mono, 1);
//     }
// }

static esp_err_t bsp_extra_record_mono_buffer_ensure(size_t mono_bytes)
{
    size_t raw_bytes;
    int16_t *new_raw_buffer;
    int16_t *new_mono_buffer;

    if (mono_bytes == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    raw_bytes = mono_bytes * CODEC_DEFAULT_ADC_CHANNEL;

    if (s_record_afe_ctx.raw_frame_buffer != NULL && s_record_afe_ctx.mono_frame_buffer != NULL &&
        s_record_afe_ctx.raw_frame_bytes >= raw_bytes && s_record_afe_ctx.mono_frame_bytes >= mono_bytes) {
        return ESP_OK;
    }

    new_raw_buffer = realloc(s_record_afe_ctx.raw_frame_buffer, raw_bytes);
    ESP_RETURN_ON_FALSE(new_raw_buffer != NULL, ESP_ERR_NO_MEM, TAG, "record raw buffer alloc failed");
    s_record_afe_ctx.raw_frame_buffer = new_raw_buffer;
    s_record_afe_ctx.raw_frame_bytes = raw_bytes;

    new_mono_buffer = realloc(s_record_afe_ctx.mono_frame_buffer, mono_bytes);
    ESP_RETURN_ON_FALSE(new_mono_buffer != NULL, ESP_ERR_NO_MEM, TAG, "record mono buffer alloc failed");
    s_record_afe_ctx.mono_frame_buffer = new_mono_buffer;
    s_record_afe_ctx.mono_frame_bytes = mono_bytes;

    return ESP_OK;
}

static void bsp_extra_record_afe_deinit(void)
{
    if (s_record_afe_ctx.handle != NULL && s_record_afe_ctx.data != NULL) {
        s_record_afe_ctx.handle->destroy(s_record_afe_ctx.data);
    }

    if (s_record_afe_ctx.models != NULL) {
        esp_srmodel_deinit(s_record_afe_ctx.models);
    }

    free(s_record_afe_ctx.raw_frame_buffer);
    free(s_record_afe_ctx.mono_frame_buffer);
    free(s_record_afe_ctx.ref_frame_buffer);
    free(s_record_afe_ctx.afe_feed_buffer);
    free(s_record_afe_ctx.fetch_frame_buffer);
    free(s_record_afe_ctx.raw_output_buffer);
    free(s_record_afe_ctx.raw_interleaved_output_buffer);
    free(s_record_afe_ctx.playback_ref_buffer);

    if (s_record_afe_ctx.playback_ref_lock != NULL) {
        vSemaphoreDelete(s_record_afe_ctx.playback_ref_lock);
        s_record_afe_ctx.playback_ref_lock = NULL;
    }

    memset(&s_record_afe_ctx, 0, sizeof(s_record_afe_ctx));
}

static bool bsp_extra_record_afe_is_enabled(void)
{
    return s_record_fs_afe_compatible && s_record_afe_ctx.initialized && s_record_afe_ctx.handle != NULL && s_record_afe_ctx.data != NULL;
}

static bool bsp_extra_record_afe_is_supported_input_format(const char *input_format)
{
    if (input_format == NULL) {
        return false;
    }

    return strcmp(input_format, "M") == 0 || strcmp(input_format, "MR") == 0 || strcmp(input_format, "MMRN") == 0;
}

static bool bsp_extra_record_afe_is_supported_reference_source(const char *source)
{
    if (source == NULL) {
        return false;
    }

    return strcmp(source, "adc") == 0 || strcmp(source, "playback") == 0;
}

static void bsp_extra_record_afe_select_channel(const int16_t *input, int16_t *output, int sample_count)
{
    size_t selected_channel = (CODEC_DEFAULT_ADC_CHANNEL > 1) ? 1 : 0;

    for (int sample_index = 0; sample_index < sample_count; sample_index++) {
        output[sample_index] = input[sample_index * CODEC_DEFAULT_ADC_CHANNEL + selected_channel];
    }

    s_record_afe_ctx.active_channel = selected_channel;
}

static esp_err_t bsp_extra_record_afe_init(void)
{
    afe_config_t *afe_config = NULL;
    esp_err_t ret = ESP_OK;

    if (!s_record_fs_afe_compatible) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (bsp_extra_record_afe_is_enabled()) {
        return ESP_OK;
    }

    bsp_extra_record_afe_deinit();

    s_record_afe_ctx.models = esp_srmodel_init(BSP_EXTRA_AFE_MODEL_PARTITION);
    ESP_RETURN_ON_FALSE(s_record_afe_ctx.models != NULL, ESP_FAIL, TAG, "esp_srmodel_init failed");

    afe_config = afe_config_init(s_record_afe_input_format, s_record_afe_ctx.models, AFE_TYPE_VC, AFE_MODE_HIGH_PERF);
    ESP_RETURN_ON_FALSE(afe_config != NULL, ESP_FAIL, TAG, "afe_config_init failed");

    afe_config->se_init = false;
    afe_config->vad_init = false;
    afe_config->wakenet_init = false;
    afe_config->aec_init = true;
    afe_config->agc_init = true;
    afe_config->agc_mode = AFE_AGC_MODE_WEBRTC;
    afe_config->agc_compression_gain_db = 20;
    afe_config->agc_target_level_dbfs = 1;
    afe_config->ns_init = true;
    afe_config->afe_linear_gain = 4.0f;         //  2.5f
    afe_config->fixed_output_channel = true;
    afe_config->output_playback_channel = false;

    s_record_afe_ctx.handle = esp_afe_handle_from_config(afe_config);
    ESP_GOTO_ON_FALSE(s_record_afe_ctx.handle != NULL, ESP_FAIL, afe_err, TAG, "esp_afe_handle_from_config failed");

    s_record_afe_ctx.data = s_record_afe_ctx.handle->create_from_config(afe_config);
    ESP_GOTO_ON_FALSE(s_record_afe_ctx.data != NULL, ESP_FAIL, afe_err, TAG, "AFE create_from_config failed");

    s_record_afe_ctx.feed_chunksize = s_record_afe_ctx.handle->get_feed_chunksize(s_record_afe_ctx.data);
    s_record_afe_ctx.feed_nch = s_record_afe_ctx.handle->get_feed_channel_num(s_record_afe_ctx.data);
    s_record_afe_ctx.fetch_chunksize = s_record_afe_ctx.handle->get_fetch_chunksize(s_record_afe_ctx.data);
    s_record_afe_ctx.feed_frames_per_fetch = (s_record_afe_ctx.fetch_chunksize + s_record_afe_ctx.feed_chunksize - 1) /
                                             s_record_afe_ctx.feed_chunksize;
    s_record_afe_ctx.output_credit_samples = 0;
    s_record_afe_ctx.raw_frame_bytes = (size_t)s_record_afe_ctx.feed_chunksize * CODEC_DEFAULT_ADC_CHANNEL * sizeof(int16_t);
    s_record_afe_ctx.mono_frame_bytes = (size_t)s_record_afe_ctx.feed_chunksize * sizeof(int16_t);
    s_record_afe_ctx.afe_feed_frame_bytes = (size_t)s_record_afe_ctx.feed_chunksize * s_record_afe_ctx.feed_nch * sizeof(int16_t);
    s_record_afe_ctx.fetch_frame_bytes = (size_t)s_record_afe_ctx.fetch_chunksize * sizeof(int16_t);

    s_record_afe_ctx.raw_frame_buffer = malloc(s_record_afe_ctx.raw_frame_bytes);
    s_record_afe_ctx.mono_frame_buffer = malloc(s_record_afe_ctx.mono_frame_bytes);
    s_record_afe_ctx.ref_frame_buffer = malloc(s_record_afe_ctx.mono_frame_bytes);
    s_record_afe_ctx.afe_feed_buffer = malloc(s_record_afe_ctx.afe_feed_frame_bytes);
    s_record_afe_ctx.fetch_frame_buffer = malloc(s_record_afe_ctx.fetch_frame_bytes);
    ESP_GOTO_ON_FALSE(s_record_afe_ctx.raw_frame_buffer != NULL, ESP_ERR_NO_MEM, afe_err, TAG, "AFE raw buffer alloc failed");
    ESP_GOTO_ON_FALSE(s_record_afe_ctx.mono_frame_buffer != NULL, ESP_ERR_NO_MEM, afe_err, TAG, "AFE mono buffer alloc failed");
    ESP_GOTO_ON_FALSE(s_record_afe_ctx.ref_frame_buffer != NULL, ESP_ERR_NO_MEM, afe_err, TAG, "AFE ref buffer alloc failed");
    ESP_GOTO_ON_FALSE(s_record_afe_ctx.afe_feed_buffer != NULL, ESP_ERR_NO_MEM, afe_err, TAG, "AFE feed buffer alloc failed");
    ESP_GOTO_ON_FALSE(s_record_afe_ctx.fetch_frame_buffer != NULL, ESP_ERR_NO_MEM, afe_err, TAG, "AFE fetch buffer alloc failed");

    s_record_afe_ctx.playback_ref_lock = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(s_record_afe_ctx.playback_ref_lock != NULL, ESP_ERR_NO_MEM, afe_err, TAG, "AFE playback ref lock alloc failed");
    ret = bsp_extra_record_playback_ref_ensure((size_t)s_record_afe_ctx.feed_chunksize * (size_t)s_record_afe_ctx.feed_frames_per_fetch * 8);
    ESP_GOTO_ON_ERROR(ret, afe_err, TAG, "AFE playback ref buffer alloc failed");
    ret = bsp_extra_record_raw_output_ensure((size_t)s_record_afe_ctx.feed_chunksize * (size_t)s_record_afe_ctx.feed_frames_per_fetch * 8);
    ESP_GOTO_ON_ERROR(ret, afe_err, TAG, "AFE raw output buffer alloc failed");
    ret = bsp_extra_record_raw_interleaved_output_ensure((size_t)s_record_afe_ctx.feed_chunksize * CODEC_DEFAULT_ADC_CHANNEL *
                                                         (size_t)s_record_afe_ctx.feed_frames_per_fetch * 8);
    ESP_GOTO_ON_ERROR(ret, afe_err, TAG, "AFE raw interleaved output buffer alloc failed");

    s_record_afe_ctx.initialized = true;

    ESP_LOGI(TAG, "record AFE enabled, format=%s ref=%s feed=%d nch=%d fetch=%d cycles=%d",
             s_record_afe_input_format,
             s_record_afe_use_adc_reference ? "adc" : "playback",
             s_record_afe_ctx.feed_chunksize,
             s_record_afe_ctx.feed_nch,
             s_record_afe_ctx.fetch_chunksize,
             s_record_afe_ctx.feed_frames_per_fetch);

    afe_config_free(afe_config);
    return ESP_OK;

afe_err:
    if (afe_config != NULL) {
        afe_config_free(afe_config);
    }
    bsp_extra_record_afe_deinit();
    return ret;
}

static esp_err_t bsp_extra_record_afe_fill_buffer(void)
{
    afe_fetch_result_t *fetch_result = NULL;
    esp_err_t ret = ESP_FAIL;
    int feed_result;
    int invalid_frame_reads = 0;
    int partial_invalid_frame_logs = 0;
    int invalid_frame_head_logs = 0;

    for (int attempt = 0; attempt < BSP_EXTRA_AFE_FETCH_MAX_ATTEMPTS; attempt++) {
        do {
            bool frame_all_minus_one = true;

            ret = esp_codec_dev_read(record_dev_handle, s_record_afe_ctx.raw_frame_buffer, s_record_afe_ctx.raw_frame_bytes);
            if (ret != ESP_OK) {
                return ret;
            }

            if (attempt == 0 && s_record_afe_ctx.feed_nch > 1 && s_record_afe_ctx.feed_chunksize >= 2) {
                ESP_LOGI(TAG, "AFE raw in ch0=%d,%d ch1=%d,%d", s_record_afe_ctx.raw_frame_buffer[0],
                         s_record_afe_ctx.raw_frame_buffer[CODEC_DEFAULT_ADC_CHANNEL],
                         s_record_afe_ctx.raw_frame_buffer[1],
                         s_record_afe_ctx.raw_frame_buffer[CODEC_DEFAULT_ADC_CHANNEL + 1]);
            }

            if (s_record_afe_ctx.feed_nch > 1) {
                size_t frame_sample_count = (size_t)s_record_afe_ctx.feed_chunksize * CODEC_DEFAULT_ADC_CHANNEL;
                size_t non_minus_one_count = 0;
                size_t first_valid_index = 0;
                int16_t first_valid_sample = -1;

                for (size_t sample_index = 0; sample_index < frame_sample_count; sample_index++) {
                    if (s_record_afe_ctx.raw_frame_buffer[sample_index] != -1) {
                        frame_all_minus_one = false;
                        non_minus_one_count++;
                        if (first_valid_sample == -1) {
                            first_valid_index = sample_index;
                            first_valid_sample = s_record_afe_ctx.raw_frame_buffer[sample_index];
                        }
                    }
                }

                if (frame_all_minus_one) {
                    invalid_frame_reads++;
                    if (invalid_frame_reads == 1) {
                        ESP_LOGW(TAG, "AFE skip invalid raw frame: all samples are -1 in MR mode");
                    }
                    if (invalid_frame_reads >= (BSP_EXTRA_AFE_FETCH_MAX_ATTEMPTS * 8)) {
                        ESP_LOGE(TAG, "AFE invalid raw frame persists in MR mode");
                        return ESP_FAIL;
                    }
                    continue;
                }

                if (partial_invalid_frame_logs < 2 && s_record_afe_ctx.raw_frame_buffer[0] == -1 &&
                    s_record_afe_ctx.raw_frame_buffer[1] == -1) {
                    partial_invalid_frame_logs++;
                    ESP_LOGW(TAG,
                             "AFE accept partially invalid raw frame: non_minus_one=%u/%u first_valid[%u]=%d",
                             (unsigned)non_minus_one_count,
                             (unsigned)frame_sample_count,
                             (unsigned)first_valid_index,
                             first_valid_sample);
                }

                if (s_record_afe_ctx.feed_chunksize >= 2 &&
                    (s_record_afe_ctx.raw_frame_buffer[0] == -1 ||
                     s_record_afe_ctx.raw_frame_buffer[1] == -1 ||
                     s_record_afe_ctx.raw_frame_buffer[CODEC_DEFAULT_ADC_CHANNEL] == -1 ||
                     s_record_afe_ctx.raw_frame_buffer[CODEC_DEFAULT_ADC_CHANNEL + 1] == -1)) {
                    invalid_frame_reads++;
                    if (invalid_frame_head_logs < 2) {
                        invalid_frame_head_logs++;
                        ESP_LOGW(TAG, "AFE skip invalid raw frame head: ch0=%d,%d ch1=%d,%d",
                                 s_record_afe_ctx.raw_frame_buffer[0],
                                 s_record_afe_ctx.raw_frame_buffer[CODEC_DEFAULT_ADC_CHANNEL],
                                 s_record_afe_ctx.raw_frame_buffer[1],
                                 s_record_afe_ctx.raw_frame_buffer[CODEC_DEFAULT_ADC_CHANNEL + 1]);
                    }
                    if (invalid_frame_reads >= (BSP_EXTRA_AFE_FETCH_MAX_ATTEMPTS * 8)) {
                        ESP_LOGE(TAG, "AFE invalid raw frame head persists in MR mode");
                        return ESP_FAIL;
                    }
                    continue;
                }
            }
            invalid_frame_reads = 0;

            bsp_extra_record_raw_interleaved_output_push(s_record_afe_ctx.raw_frame_buffer,
                                                         (size_t)s_record_afe_ctx.feed_chunksize * CODEC_DEFAULT_ADC_CHANNEL);
            bsp_extra_record_afe_select_channel(s_record_afe_ctx.raw_frame_buffer, s_record_afe_ctx.mono_frame_buffer,
                                                s_record_afe_ctx.feed_chunksize);
            bsp_extra_record_raw_output_push(s_record_afe_ctx.mono_frame_buffer, (size_t)s_record_afe_ctx.feed_chunksize);
            if (s_record_afe_ctx.feed_nch > 1) {
                int16_t *ref_channel = NULL;

                bsp_extra_record_afe_build_feed_frame(s_record_afe_ctx.raw_frame_buffer, s_record_afe_ctx.mono_frame_buffer);
                if (attempt == 0 && s_record_afe_ctx.feed_chunksize >= 2) {
                    ESP_LOGI(TAG, "AFE feed in mic=%d,%d ref=%d,%d source=%s",
                             s_record_afe_ctx.afe_feed_buffer[0],
                             s_record_afe_ctx.afe_feed_buffer[s_record_afe_ctx.feed_nch],
                             s_record_afe_ctx.afe_feed_buffer[1],
                             s_record_afe_ctx.afe_feed_buffer[s_record_afe_ctx.feed_nch + 1],
                             s_record_afe_use_adc_reference ? "adc" : "playback");
                }
                ref_channel = s_record_afe_ctx.afe_feed_buffer;
                feed_result = s_record_afe_ctx.handle->feed(s_record_afe_ctx.data, ref_channel);
            } else {
                feed_result = s_record_afe_ctx.handle->feed(s_record_afe_ctx.data, s_record_afe_ctx.mono_frame_buffer);
            }

            if (feed_result <= 0) {
                ESP_LOGE(TAG, "AFE feed failed, ret=%d raw_bytes=%u feed=%d nch=%d", feed_result,
                         (unsigned)s_record_afe_ctx.raw_frame_bytes, s_record_afe_ctx.feed_chunksize,
                         s_record_afe_ctx.feed_nch);
                return ESP_FAIL;
            }

            s_record_afe_ctx.output_credit_samples += s_record_afe_ctx.feed_chunksize;
        } while (s_record_afe_ctx.output_credit_samples < s_record_afe_ctx.fetch_chunksize);

        if (s_record_afe_ctx.handle->fetch_with_delay != NULL) {
            fetch_result = s_record_afe_ctx.handle->fetch_with_delay(s_record_afe_ctx.data, pdMS_TO_TICKS(100));
        } else {
            fetch_result = s_record_afe_ctx.handle->fetch(s_record_afe_ctx.data);
        }

        if (fetch_result == NULL || fetch_result->data == NULL || fetch_result->data_size <= 0) {
            ESP_LOGW(TAG, "AFE fetch empty, attempt=%d result=%p data=%p size=%d ret=%d rb_free=%.3f credit=%d", attempt + 1,
                     fetch_result, fetch_result ? fetch_result->data : NULL,
                     fetch_result ? fetch_result->data_size : -1,
                     fetch_result ? fetch_result->ret_value : -1,
                     fetch_result ? fetch_result->ringbuff_free_pct : -1.0f,
                     s_record_afe_ctx.output_credit_samples);
            continue;
        }

        ESP_RETURN_ON_FALSE((size_t)fetch_result->data_size <= s_record_afe_ctx.fetch_frame_bytes, ESP_FAIL, TAG, "AFE fetch size overflow");

        memcpy(s_record_afe_ctx.fetch_frame_buffer, fetch_result->data, (size_t)fetch_result->data_size);
        s_record_afe_ctx.fetch_buffer_filled = (size_t)fetch_result->data_size;
        s_record_afe_ctx.fetch_buffer_offset = 0;
        s_record_afe_ctx.output_credit_samples -= (int)(fetch_result->data_size / sizeof(int16_t));
        if (s_record_afe_ctx.output_credit_samples < 0) {
            s_record_afe_ctx.output_credit_samples = 0;
        }
        return ESP_OK;
    }

    ESP_LOGE(TAG, "AFE fetch failed after %d feed attempts", BSP_EXTRA_AFE_FETCH_MAX_ATTEMPTS);
    return ret;
}

static void bsp_extra_record_codec_patch_es7243e(esp_codec_dev_handle_t handle)
{
    bsp_extra_codec_dev_view_t *codec_dev = (bsp_extra_codec_dev_view_t *)handle;
    audio_codec_if_t *codec_if;

    if (codec_dev == NULL || codec_dev->codec_if == NULL) {
        return;
    }

    codec_if = (audio_codec_if_t *)codec_dev->codec_if;
    codec_if->set_mic_gain = NULL;
    codec_if->set_mic_channel_gain = NULL;
}

static esp_err_t bsp_extra_record_codec_ensure_ready(void)
{
    if (record_dev_handle != NULL) {
        return ESP_OK;
    }

    record_dev_handle = bsp_audio_codec_microphone_init();
    ESP_RETURN_ON_FALSE(record_dev_handle, ESP_FAIL, TAG, "record_dev_handle not initialized");
    bsp_extra_record_codec_patch_es7243e(record_dev_handle);

    return ESP_OK;
}

static void bsp_extra_pa_power_set(bool enable)
{
    if (s_pa_power_enabled == enable) {
        return;
    }

    bsp_exp_output_io_set_level(BSP_PA_PWR_EN, enable ? 1 : 0);
    s_pa_power_enabled = enable;

    vTaskDelay(pdMS_TO_TICKS(BSP_EXTRA_PA_PWR_SWITCH_DELAY_MS));

    ESP_LOGI(TAG, "PA power %s, level=0x%lx", enable ? "on" : "off", (unsigned long)bsp_exp_input_io_get_level(BSP_PA_PWR_EN));
}

/**************************************************************************************************
 *
 * Extra Board Function
 *
 **************************************************************************************************/

static esp_err_t audio_mute_function(AUDIO_PLAYER_MUTE_SETTING setting)
{
    // Volume saved when muting and restored when unmuting. Restoring volume is necessary
    // as es8311_set_voice_mute(true) results in voice volume (REG32) being set to zero.

    bsp_extra_codec_mute_set(setting == AUDIO_PLAYER_MUTE ? true : false);

    // restore the voice volume upon unmuting
    if (setting == AUDIO_PLAYER_UNMUTE) {
        ESP_RETURN_ON_ERROR(esp_codec_dev_set_out_vol(play_dev_handle, _vloume_intensity), TAG, "Set Codec volume failed");
    }

    return ESP_OK;
}

static void audio_callback(audio_player_cb_ctx_t *ctx)
{
    if (audio_idle_callback) {
        ctx->user_ctx = audio_idle_cb_user_data;
        audio_idle_callback(ctx);
    }
}

#if 0
esp_err_t bsp_extra_i2s_read(void *audio_buffer, size_t len, size_t *bytes_read, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    uint8_t *output = (uint8_t *)audio_buffer;
    size_t copied = 0;

    (void)timeout_ms;

    ESP_RETURN_ON_FALSE(audio_buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "audio_buffer is NULL");
    ESP_RETURN_ON_FALSE((len % sizeof(int16_t)) == 0, ESP_ERR_INVALID_SIZE, TAG, "audio read size must align to 16-bit samples");

    ret = bsp_extra_record_codec_ensure_ready();
    if (ret != ESP_OK) {
        return ret;
    }

    if (!bsp_extra_record_afe_is_enabled()) {
        ret = bsp_extra_record_afe_init();
        if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGW(TAG, "AFE init failed, fallback to raw microphone stream");
        }
    }

    if (!bsp_extra_record_afe_is_enabled()) {
        while (copied < len) {
            size_t mono_bytes = len - copied;
            int sample_count;

            ret = bsp_extra_record_mono_buffer_ensure(mono_bytes);
            if (ret != ESP_OK) {
                break;
            }

            ret = esp_codec_dev_read(record_dev_handle, s_record_afe_ctx.raw_frame_buffer, s_record_afe_ctx.raw_frame_bytes);
            if (ret != ESP_OK) {
                break;
            }

            sample_count = (int)(mono_bytes / sizeof(int16_t));
            bsp_extra_record_afe_select_channel(s_record_afe_ctx.raw_frame_buffer, s_record_afe_ctx.mono_frame_buffer, sample_count);
            memcpy(output + copied, s_record_afe_ctx.mono_frame_buffer, mono_bytes);
            copied += mono_bytes;
        }

        if (bytes_read != NULL) {
            *bytes_read = copied;
        }
        return ret;
    }

    while (copied < len) {
        size_t pending = s_record_afe_ctx.fetch_buffer_filled - s_record_afe_ctx.fetch_buffer_offset;

        if (pending == 0) {
            ret = bsp_extra_record_afe_fill_buffer();
            if (ret != ESP_OK) {
                if (bytes_read != NULL) {
                    *bytes_read = copied;
                }
                return ret;
            }
            pending = s_record_afe_ctx.fetch_buffer_filled - s_record_afe_ctx.fetch_buffer_offset;
        }

        if (pending > 0) {
            size_t copy_bytes = len - copied;
            if (copy_bytes > pending) {
                copy_bytes = pending;
            }

            memcpy(output + copied, ((uint8_t *)s_record_afe_ctx.fetch_frame_buffer) + s_record_afe_ctx.fetch_buffer_offset, copy_bytes);
            copied += copy_bytes;
            s_record_afe_ctx.fetch_buffer_offset += copy_bytes;
        }
    }

    if (bytes_read != NULL) {
        *bytes_read = copied;
    }

    return ESP_OK;
}

#else
esp_err_t bsp_extra_i2s_read(void *audio_buffer, size_t len, size_t *bytes_read, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = esp_codec_dev_read(record_dev_handle, audio_buffer, len);
    *bytes_read = len;
    return ret;
}

#endif

esp_err_t bsp_extra_i2s_read_processed_and_raw(void *processed_buffer, void *raw_buffer, size_t len, size_t *bytes_read, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    uint8_t *processed = (uint8_t *)processed_buffer;
    uint8_t *raw = (uint8_t *)raw_buffer;
    size_t copied = 0;

    (void)timeout_ms;

    ESP_RETURN_ON_FALSE(processed_buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "processed_buffer is NULL");
    ESP_RETURN_ON_FALSE(raw_buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "raw_buffer is NULL");
    ESP_RETURN_ON_FALSE((len % sizeof(int16_t)) == 0, ESP_ERR_INVALID_SIZE, TAG, "audio read size must align to 16-bit samples");

    ret = bsp_extra_record_codec_ensure_ready();
    if (ret != ESP_OK) {
        return ret;
    }

    if (!bsp_extra_record_afe_is_enabled()) {
        ret = bsp_extra_record_afe_init();
        if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGW(TAG, "AFE init failed, fallback to raw microphone stream");
        }
    }

    if (!bsp_extra_record_afe_is_enabled()) {
        while (copied < len) {
            size_t mono_bytes = len - copied;
            int sample_count;

            ret = bsp_extra_record_mono_buffer_ensure(mono_bytes);
            if (ret != ESP_OK) {
                break;
            }

            ret = esp_codec_dev_read(record_dev_handle, s_record_afe_ctx.raw_frame_buffer, s_record_afe_ctx.raw_frame_bytes);
            if (ret != ESP_OK) {
                break;
            }

            sample_count = (int)(mono_bytes / sizeof(int16_t));
            bsp_extra_record_afe_select_channel(s_record_afe_ctx.raw_frame_buffer, s_record_afe_ctx.mono_frame_buffer, sample_count);
            memcpy(raw + copied, s_record_afe_ctx.mono_frame_buffer, mono_bytes);
            memcpy(processed + copied, s_record_afe_ctx.mono_frame_buffer, mono_bytes);
            copied += mono_bytes;
        }

        if (bytes_read != NULL) {
            *bytes_read = copied;
        }
        return ret;
    }

    while (copied < len) {
        size_t pending_processed = s_record_afe_ctx.fetch_buffer_filled - s_record_afe_ctx.fetch_buffer_offset;
        size_t pending_raw = s_record_afe_ctx.raw_output_filled_samples * sizeof(int16_t);

        if (pending_processed == 0 || pending_raw == 0) {
            ret = bsp_extra_record_afe_fill_buffer();
            if (ret != ESP_OK) {
                if (bytes_read != NULL) {
                    *bytes_read = copied;
                }
                return ret;
            }
            pending_processed = s_record_afe_ctx.fetch_buffer_filled - s_record_afe_ctx.fetch_buffer_offset;
            pending_raw = s_record_afe_ctx.raw_output_filled_samples * sizeof(int16_t);
        }

        if (pending_processed > 0 && pending_raw > 0) {
            size_t copy_bytes = len - copied;
            if (copy_bytes > pending_processed) {
                copy_bytes = pending_processed;
            }
            if (copy_bytes > pending_raw) {
                copy_bytes = pending_raw;
            }

            memcpy(processed + copied,
                   ((uint8_t *)s_record_afe_ctx.fetch_frame_buffer) + s_record_afe_ctx.fetch_buffer_offset,
                   copy_bytes);
            bsp_extra_record_raw_output_pop((int16_t *)(raw + copied), copy_bytes / sizeof(int16_t));
            copied += copy_bytes;
            s_record_afe_ctx.fetch_buffer_offset += copy_bytes;
        }
    }

    if (bytes_read != NULL) {
        *bytes_read = copied;
    }

    return ESP_OK;
}

esp_err_t bsp_extra_i2s_read_processed_raw_and_interleaved(void *processed_buffer,
                                                           void *raw_buffer,
                                                           void *raw_interleaved_buffer,
                                                           size_t len,
                                                           size_t raw_interleaved_len,
                                                           size_t *bytes_read,
                                                           size_t *raw_interleaved_bytes_read,
                                                           uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    uint8_t *processed = (uint8_t *)processed_buffer;
    uint8_t *raw = (uint8_t *)raw_buffer;
    uint8_t *raw_interleaved = (uint8_t *)raw_interleaved_buffer;
    size_t copied = 0;
    size_t raw_interleaved_copied = 0;
    size_t target_len;

    (void)timeout_ms;

    ESP_RETURN_ON_FALSE(processed_buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "processed_buffer is NULL");
    ESP_RETURN_ON_FALSE(raw_buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "raw_buffer is NULL");
    ESP_RETURN_ON_FALSE(raw_interleaved_buffer != NULL, ESP_ERR_INVALID_ARG, TAG, "raw_interleaved_buffer is NULL");
    ESP_RETURN_ON_FALSE((len % sizeof(int16_t)) == 0, ESP_ERR_INVALID_SIZE, TAG, "audio read size must align to 16-bit samples");
    ESP_RETURN_ON_FALSE((raw_interleaved_len % sizeof(int16_t)) == 0, ESP_ERR_INVALID_SIZE, TAG,
                        "raw interleaved read size must align to 16-bit samples");

    target_len = len;
    if ((raw_interleaved_len / CODEC_DEFAULT_ADC_CHANNEL) < target_len) {
        target_len = raw_interleaved_len / CODEC_DEFAULT_ADC_CHANNEL;
    }

    ret = bsp_extra_record_codec_ensure_ready();
    if (ret != ESP_OK) {
        return ret;
    }

    if (!bsp_extra_record_afe_is_enabled()) {
        ret = bsp_extra_record_afe_init();
        if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGW(TAG, "AFE init failed, fallback to raw microphone stream");
        }
    }

    if (!bsp_extra_record_afe_is_enabled()) {
        while (copied < target_len) {
            size_t mono_bytes = target_len - copied;
            size_t interleaved_bytes = mono_bytes * CODEC_DEFAULT_ADC_CHANNEL;
            int sample_count;

            ret = bsp_extra_record_mono_buffer_ensure(mono_bytes);
            if (ret != ESP_OK) {
                break;
            }

            ret = esp_codec_dev_read(record_dev_handle, s_record_afe_ctx.raw_frame_buffer, interleaved_bytes);
            if (ret != ESP_OK) {
                break;
            }

            sample_count = (int)(mono_bytes / sizeof(int16_t));
            memcpy(raw_interleaved + raw_interleaved_copied, s_record_afe_ctx.raw_frame_buffer, interleaved_bytes);
            bsp_extra_record_afe_select_channel(s_record_afe_ctx.raw_frame_buffer, s_record_afe_ctx.mono_frame_buffer, sample_count);
            memcpy(raw + copied, s_record_afe_ctx.mono_frame_buffer, mono_bytes);
            memcpy(processed + copied, s_record_afe_ctx.mono_frame_buffer, mono_bytes);
            copied += mono_bytes;
            raw_interleaved_copied += interleaved_bytes;
        }

        if (bytes_read != NULL) {
            *bytes_read = copied;
        }
        if (raw_interleaved_bytes_read != NULL) {
            *raw_interleaved_bytes_read = raw_interleaved_copied;
        }
        return ret;
    }

    while (copied < target_len) {
        size_t pending_processed = s_record_afe_ctx.fetch_buffer_filled - s_record_afe_ctx.fetch_buffer_offset;
        size_t pending_raw = s_record_afe_ctx.raw_output_filled_samples * sizeof(int16_t);
        size_t pending_raw_interleaved = s_record_afe_ctx.raw_interleaved_output_filled_samples * sizeof(int16_t);

        if (pending_processed == 0 || pending_raw == 0 || pending_raw_interleaved == 0) {
            ret = bsp_extra_record_afe_fill_buffer();
            if (ret != ESP_OK) {
                if (bytes_read != NULL) {
                    *bytes_read = copied;
                }
                if (raw_interleaved_bytes_read != NULL) {
                    *raw_interleaved_bytes_read = raw_interleaved_copied;
                }
                return ret;
            }
            pending_processed = s_record_afe_ctx.fetch_buffer_filled - s_record_afe_ctx.fetch_buffer_offset;
            pending_raw = s_record_afe_ctx.raw_output_filled_samples * sizeof(int16_t);
            pending_raw_interleaved = s_record_afe_ctx.raw_interleaved_output_filled_samples * sizeof(int16_t);
        }

        if (pending_processed > 0 && pending_raw > 0 && pending_raw_interleaved > 0) {
            size_t copy_bytes = target_len - copied;
            size_t interleaved_copy_bytes;

            if (copy_bytes > pending_processed) {
                copy_bytes = pending_processed;
            }
            if (copy_bytes > pending_raw) {
                copy_bytes = pending_raw;
            }
            if ((copy_bytes * CODEC_DEFAULT_ADC_CHANNEL) > pending_raw_interleaved) {
                copy_bytes = pending_raw_interleaved / CODEC_DEFAULT_ADC_CHANNEL;
            }

            interleaved_copy_bytes = copy_bytes * CODEC_DEFAULT_ADC_CHANNEL;

            memcpy(processed + copied,
                   ((uint8_t *)s_record_afe_ctx.fetch_frame_buffer) + s_record_afe_ctx.fetch_buffer_offset,
                   copy_bytes);
            bsp_extra_record_raw_output_pop((int16_t *)(raw + copied), copy_bytes / sizeof(int16_t));
            bsp_extra_record_raw_interleaved_output_pop((int16_t *)(raw_interleaved + raw_interleaved_copied),
                                                        interleaved_copy_bytes / sizeof(int16_t));
            copied += copy_bytes;
            raw_interleaved_copied += interleaved_copy_bytes;
            s_record_afe_ctx.fetch_buffer_offset += copy_bytes;
        }
    }

    if (bytes_read != NULL) {
        *bytes_read = copied;
    }
    if (raw_interleaved_bytes_read != NULL) {
        *raw_interleaved_bytes_read = raw_interleaved_copied;
    }

    return ESP_OK;
}

esp_err_t bsp_extra_playback_rms_reset(void)
{
    s_rms_sum_sq = 0;
    s_rms_count = 0;
    return ESP_OK;
}

esp_err_t bsp_extra_playback_rms_enable(bool enable)
{
    s_rms_enabled = enable;
    return ESP_OK;
}

esp_err_t bsp_extra_playback_rms_get(uint64_t *sum_sq, uint32_t *count)
{
    if (!s_play_fs_valid || s_play_fs.bits_per_sample != 16) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sum_sq != NULL) {
        *sum_sq = s_rms_sum_sq;
    }
    if (count != NULL) {
        *count = s_rms_count;
    }
    return ESP_OK;
}

#if 0
esp_err_t bsp_extra_i2s_write(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    bsp_extra_pa_power_set(true);
    bsp_extra_playback_capture_reference(audio_buffer, len);
    ret = esp_codec_dev_write(play_dev_handle, audio_buffer, len);
    *bytes_written = len;
    return ret;
}
#else
esp_err_t bsp_extra_i2s_write(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = esp_codec_dev_write(play_dev_handle, audio_buffer, len);

    /* RMS instrumentation: accumulate only when enabled and the stream is
     * 16-bit PCM. Single producer (audio player task), so plain accumulation
     * is sufficient for a measurement tool. Zero overhead when disabled. */
    if (ret == ESP_OK && s_rms_enabled &&
        s_play_fs_valid && s_play_fs.bits_per_sample == 16) {
        const int16_t *samples = (const int16_t *)audio_buffer;
        size_t n = len / sizeof(int16_t);
        uint64_t local_sq = 0;
        for (size_t i = 0; i < n; i++) {
            int32_t v = samples[i];
            local_sq += (uint64_t)(v * v);
        }
        s_rms_sum_sq += local_sq;
        s_rms_count += (uint32_t)n;
    }

    *bytes_written = len;
    return ret;
}
#endif
esp_err_t bsp_extra_codec_play_set_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    bsp_extra_pa_power_set(true);

    s_playback_ref_afe_compatible = (rate == CODEC_DEFAULT_ADC_SAMPLE_RATE) && (bits_cfg == CODEC_DEFAULT_ADC_BIT_WIDTH);
    s_playback_ref_channel_count = (ch == I2S_SLOT_MODE_STEREO) ? 2 : 1;

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = rate,
        .channel = ch,
        .bits_per_sample = bits_cfg,
    };

    if (play_dev_handle && s_play_dev_opened && s_play_fs_valid &&
        s_play_fs.sample_rate == fs.sample_rate &&
        s_play_fs.channel == fs.channel &&
        s_play_fs.bits_per_sample == fs.bits_per_sample) {
        return ESP_OK;
    }

    if (play_dev_handle && s_play_dev_opened) {
        ret = esp_codec_dev_close(play_dev_handle);
        if (ret == ESP_OK) {
            s_play_dev_opened = false;
            s_play_fs_valid = false;
        }
    }

    if (play_dev_handle) {
        esp_err_t open_ret = esp_codec_dev_open(play_dev_handle, &fs);
        ret |= open_ret;
        if (open_ret == ESP_OK) {
            s_play_dev_opened = true;
            s_play_fs = fs;
            s_play_fs_valid = true;
            bsp_extra_record_playback_ref_reset();
        }
    }

    return ret;
}

esp_err_t bsp_extra_codec_record_set_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    ret = bsp_extra_record_codec_ensure_ready();
    if (ret != ESP_OK) {
        return ret;
    }

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = rate,
        .channel = ch,
        .bits_per_sample = bits_cfg,
    };

    s_record_fs_afe_compatible = (rate == CODEC_DEFAULT_ADC_SAMPLE_RATE) && (bits_cfg == CODEC_DEFAULT_ADC_BIT_WIDTH);
    bsp_extra_record_afe_deinit();

    if (record_dev_handle && s_record_dev_opened) {
        ret |= esp_codec_dev_close(record_dev_handle);
        if (ret == ESP_OK) {
            s_record_dev_opened = false;
        }
    }

    if (record_dev_handle) {
        esp_err_t open_ret = esp_codec_dev_open(record_dev_handle, &fs);
        ret |= open_ret;
        if (open_ret == ESP_OK) {
            s_record_dev_opened = true;
            if (s_record_fs_afe_compatible) {
                esp_err_t afe_ret = bsp_extra_record_afe_init();
                if (afe_ret != ESP_OK) {
                    ESP_LOGW(TAG, "AFE unavailable for current microphone stream, using raw codec data");
                }
            }
        }
    }

    if (record_dev_handle) {
        ret |= esp_codec_dev_set_in_channel_gain(record_dev_handle, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0), CODEC_DEFAULT_ADC_VOLUME);
        ret |= esp_codec_dev_set_in_channel_gain(record_dev_handle, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(1), CODEC_DEFAULT_ADC_VOLUME);
    }

    return ret;
}

esp_err_t bsp_extra_codec_volume_set(int volume, int *volume_set)
{
    if (volume > 100) volume = 100;

    ESP_RETURN_ON_ERROR(esp_codec_dev_set_out_vol(play_dev_handle, volume), TAG, "Set Codec volume failed");
    _vloume_intensity = volume;

    ESP_LOGI(TAG, "Setting volume: %d", volume);

    return ESP_OK;
}

int bsp_extra_codec_volume_get(void)
{
    return _vloume_intensity;
}

esp_err_t bsp_extra_codec_set_mic_gain(float db)
{
    if (record_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /* esp_codec_dev_set_in_gain() -> codec->set_mic_gain -> es7243e_set_gain,
     * which writes the ES7243E PGA register (0x20/0x21). The driver clamps the
     * gain to the ES7243E maximum of +37.5 dB (reg 14). Requires the record
     * stream to be opened first, otherwise returns WRONG_STATE. */
    return esp_codec_dev_set_in_gain(record_dev_handle, db);
}

esp_err_t bsp_extra_codec_mute_set(bool enable)
{
    esp_err_t ret = ESP_OK;
    ret = esp_codec_dev_set_out_mute(play_dev_handle, enable);
    return ret;
}

esp_err_t bsp_extra_codec_dev_stop(void)
{
    esp_err_t ret = ESP_OK;

    if (play_dev_handle) {
        if (s_play_dev_opened) {
            ret = esp_codec_dev_close(play_dev_handle);
            if (ret == ESP_OK) {
                s_play_dev_opened = false;
                s_play_fs_valid = false;
            }
        }
    }

    if (record_dev_handle) {
        if (s_record_dev_opened) {
            ret = esp_codec_dev_close(record_dev_handle);
            if (ret == ESP_OK) {
                s_record_dev_opened = false;
            }
        }
    }

    bsp_extra_record_afe_deinit();

    bsp_extra_pa_power_set(false);
    return ret;
}

esp_err_t bsp_extra_codec_dev_resume(void)
{
    esp_err_t ret = ESP_OK;
    ret = bsp_extra_codec_play_set_fs(CODEC_DEFAULT_DAC_SAMPLE_RATE, CODEC_DEFAULT_DAC_BIT_WIDTH, CODEC_DEFAULT_DAC_CHANNEL);
    ret |= bsp_extra_codec_record_set_fs(CODEC_DEFAULT_ADC_SAMPLE_RATE, CODEC_DEFAULT_ADC_BIT_WIDTH, CODEC_DEFAULT_ADC_CHANNEL);
    return ret;
}

esp_err_t bsp_extra_codec_deinit(void)
{
    /* Stop any open streams first */
    bsp_extra_codec_dev_stop();

    /* Reset init flags so the next bsp_extra_codec_init() call will
     * re-create codec device handles from scratch.  This is needed after
     * the codec chips (ES8311 / ES7243E) have been power-cycled via
     * SEN_EN — the I2C register state is lost but the driver still
     * held stale handles. */
    play_dev_handle = NULL;
    record_dev_handle = NULL;
    s_play_dev_opened = false;
    s_record_dev_opened = false;
    s_play_fs_valid = false;
    _is_audio_init = false;
    _is_player_init = false;

    ESP_LOGI(TAG, "codec deinit ok");
    return ESP_OK;
}

esp_err_t bsp_extra_codec_init()
{
    if (_is_audio_init) {
        return ESP_OK;
    }

    play_dev_handle = bsp_audio_codec_speaker_init();
    assert((play_dev_handle) && "play_dev_handle not initialized");
    bsp_extra_codec_play_set_fs(CODEC_DEFAULT_DAC_SAMPLE_RATE, CODEC_DEFAULT_DAC_BIT_WIDTH, CODEC_DEFAULT_DAC_CHANNEL);

    record_dev_handle = bsp_audio_codec_microphone_init();
    assert((record_dev_handle) && "record_dev_handle not initialized");
    bsp_extra_codec_record_set_fs(CODEC_DEFAULT_ADC_SAMPLE_RATE, CODEC_DEFAULT_ADC_BIT_WIDTH, CODEC_DEFAULT_ADC_CHANNEL);

    _is_audio_init = true;

    ESP_LOGI(TAG, "codec init ok");

    return ESP_OK;
}

esp_err_t bsp_extra_player_init(void)
{
    if (_is_player_init) {
        return ESP_OK;
    }

    audio_player_config_t config = { .mute_fn = audio_mute_function,
                                     .write_fn = bsp_extra_i2s_write,
                                     .clk_set_fn = bsp_extra_codec_play_set_fs,
                                     .priority = 5
                                   };
    ESP_RETURN_ON_ERROR(audio_player_new(config), TAG, "audio_player_init failed");
    audio_player_callback_register(audio_callback, NULL);

    _is_player_init = true;
    ESP_LOGI(TAG, "codec player init ok");

    return ESP_OK;
}

esp_err_t bsp_extra_player_del(void)
{
    if (!_is_player_init) {
        return ESP_OK;
    }

    _is_player_init = false;

    ESP_RETURN_ON_ERROR(audio_player_delete(), TAG, "audio_player_delete failed");

    return ESP_OK;
}

esp_err_t bsp_extra_file_instance_init(const char *path, file_iterator_instance_t **ret_instance)
{
    ESP_RETURN_ON_FALSE(path, ESP_FAIL, TAG, "path is NULL");
    ESP_RETURN_ON_FALSE(ret_instance, ESP_FAIL, TAG, "ret_instance is NULL");

    file_iterator_instance_t *file_iterator = file_iterator_new(path);
    ESP_RETURN_ON_FALSE(file_iterator, ESP_FAIL, TAG, "file_iterator_new failed, %s", path);

    *ret_instance = file_iterator;

    return ESP_OK;
}

esp_err_t bsp_extra_player_play_index(file_iterator_instance_t *instance, int index)
{
    ESP_RETURN_ON_FALSE(instance, ESP_FAIL, TAG, "instance is NULL");

    ESP_LOGI(TAG, "play_index(%d)", index);
    char filename[128];
    int retval = file_iterator_get_full_path_from_index(instance, index, filename, sizeof(filename));
    ESP_RETURN_ON_FALSE(retval != 0, ESP_FAIL, TAG, "file_iterator_get_full_path_from_index failed");

    ESP_LOGI(TAG, "opening file '%s'", filename);
    FILE *fp = fopen(filename, "rb");
    ESP_RETURN_ON_FALSE(fp, ESP_FAIL, TAG, "unable to open file");

    ESP_LOGI(TAG, "Playing '%s'", filename);
    ESP_RETURN_ON_ERROR(audio_player_play(fp), TAG, "audio_player_play failed");

    memcpy(audio_file_path, filename, sizeof(audio_file_path));

    return ESP_OK;
}

esp_err_t bsp_extra_player_play_file(const char *file_path)
{
    ESP_LOGI(TAG, "opening file '%s'", file_path);
    FILE *fp = fopen(file_path, "rb");
    ESP_RETURN_ON_FALSE(fp, ESP_FAIL, TAG, "unable to open file");

    ESP_LOGI(TAG, "Playing '%s'", file_path);
    ESP_RETURN_ON_ERROR(audio_player_play(fp), TAG, "audio_player_play failed");

    memcpy(audio_file_path, file_path, sizeof(audio_file_path));

    return ESP_OK;
}

void bsp_extra_player_register_callback(audio_player_cb_t cb, void *user_data)
{
    audio_idle_callback = cb;
    audio_idle_cb_user_data = user_data;
}

bool bsp_extra_player_is_playing_by_path(const char *file_path)
{
    return (strcmp(audio_file_path, file_path) == 0);
}

bool bsp_extra_player_is_playing_by_index(file_iterator_instance_t *instance, int index)
{
    return (index == file_iterator_get_index(instance));
}

esp_err_t bsp_extra_get_feed_data(bool is_get_raw_channel, int16_t *buffer, int buffer_len)
{
    esp_err_t ret = ESP_OK;
    int audio_chunksize = buffer_len / (sizeof(int16_t) * CODEC_DEFAULT_ADC_CHANNEL);

    ret = bsp_extra_record_codec_ensure_ready();
    if (ret != ESP_OK) {
        return ret;
    }
    int16_t ref1 = 0;
    int16_t ref2 = 0;
    ret = esp_codec_dev_read(record_dev_handle, (void *)buffer, buffer_len);
    if (!is_get_raw_channel ) {
        for (int i = 0; i < audio_chunksize; i++) {
            ref2 = buffer[CODEC_DEFAULT_ADC_CHANNEL * i + 0];
            ref1 = buffer[CODEC_DEFAULT_ADC_CHANNEL * i + 1];
            buffer[CODEC_DEFAULT_ADC_CHANNEL * i + 0 ] = ref1;
            buffer[CODEC_DEFAULT_ADC_CHANNEL * i + 1 ] = ref2;
        }
    }
    return ret;
}

int bsp_extra_get_feed_channel(void)
{
    return CODEC_DEFAULT_ADC_CHANNEL;
}

esp_err_t bsp_extra_set_afe_input_format(const char *input_format)
{
    char previous_format[sizeof(s_record_afe_input_format)] = {0};
    bool should_reinit;
    esp_err_t ret;

    ESP_RETURN_ON_FALSE(bsp_extra_record_afe_is_supported_input_format(input_format), ESP_ERR_INVALID_ARG, TAG,
                        "unsupported AFE input format: %s", input_format ? input_format : "<null>");

    if (strcmp(s_record_afe_input_format, input_format) == 0) {
        ESP_LOGI(TAG, "AFE input format unchanged: %s", s_record_afe_input_format);
        return ESP_OK;
    }

    strlcpy(previous_format, s_record_afe_input_format, sizeof(previous_format));
    strlcpy(s_record_afe_input_format, input_format, sizeof(s_record_afe_input_format));

    should_reinit = s_record_fs_afe_compatible && s_record_dev_opened;
    if (!should_reinit) {
        ESP_LOGI(TAG, "AFE input format staged: %s", s_record_afe_input_format);
        return ESP_OK;
    }

    bsp_extra_record_afe_deinit();
    ret = bsp_extra_record_afe_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "AFE input format switched: %s -> %s", previous_format, s_record_afe_input_format);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "AFE input format switch failed: %s -> %s (%s), restoring previous format",
             previous_format, s_record_afe_input_format, esp_err_to_name(ret));
    strlcpy(s_record_afe_input_format, previous_format, sizeof(s_record_afe_input_format));
    if (s_record_fs_afe_compatible) {
        esp_err_t restore_ret = bsp_extra_record_afe_init();
        if (restore_ret != ESP_OK) {
            ESP_LOGE(TAG, "AFE restore failed after switch error: %s", esp_err_to_name(restore_ret));
        }
    }
    return ret;
}

const char *bsp_extra_get_afe_input_format(void)
{
    return s_record_afe_input_format;
}

esp_err_t bsp_extra_set_afe_reference_source(const char *source)
{
    ESP_RETURN_ON_FALSE(bsp_extra_record_afe_is_supported_reference_source(source), ESP_ERR_INVALID_ARG, TAG,
                        "unsupported AFE reference source: %s", source ? source : "<null>");

    s_record_afe_use_adc_reference = (strcmp(source, "adc") == 0);
    ESP_LOGI(TAG, "AFE reference source set to %s", s_record_afe_use_adc_reference ? "adc" : "playback");
    return ESP_OK;
}

const char *bsp_extra_get_afe_reference_source(void)
{
    return s_record_afe_use_adc_reference ? "adc" : "playback";
}

char *bsp_extra_get_input_format(void)
{
    return (CODEC_DEFAULT_ADC_CHANNEL == 4) ? "MMRN" : "MR";
}
