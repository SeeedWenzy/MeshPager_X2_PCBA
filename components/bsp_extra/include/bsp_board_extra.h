/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sys/cdefs.h>
#include <stdbool.h>
#include "esp_codec_dev.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "audio_player.h"
#include "file_iterator.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CODEC_DEFAULT_ADC_SAMPLE_RATE       (16000)
#define CODEC_DEFAULT_ADC_BIT_WIDTH         (16)
#define CODEC_DEFAULT_ADC_VOLUME            (32)
#define CODEC_DEFAULT_ADC_CHANNEL           (2)

#define CODEC_DEFAULT_DAC_SAMPLE_RATE       (16000)
#define CODEC_DEFAULT_DAC_BIT_WIDTH         (16)
#define CODEC_DEFAULT_DAC_CHANNEL           (1)
#define CODEC_DEFAULT_DAC_VOLUME            (70)

#define BSP_LCD_BACKLIGHT_BRIGHTNESS_MAX    (95)
#define BSP_LCD_BACKLIGHT_BRIGHTNESS_MIN    (0)

/**************************************************************************************************
 * BSP Extra interface
 * Mainly provided some I2S Codec interfaces.
 **************************************************************************************************/
/**
 * @brief Player set mute.
 *
 * @param enable: true or false
 *
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail
 */
esp_err_t bsp_extra_codec_mute_set(bool enable);

/**
 * @brief Player set volume.
 *
 * @param volume: volume set
 * @param volume_set: volume set response
 *
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail
 */
esp_err_t bsp_extra_codec_volume_set(int volume, int *volume_set);

/**
 * @brief Player get volume.
 *
 * @return
 *   - volume: volume get
 */
int bsp_extra_codec_volume_get(void);

/**
 * @brief Set the microphone record PGA gain in dB.
 *
 * Drives codec->set_mic_gain (ES7243E: es7243e_set_gain -> PGA register 0x20/0x21).
 * The ES7243E range is 0..37.5 dB and is clamped to that maximum. Must be called
 * after the record codec is opened (e.g. after bsp_extra_codec_init()).
 *
 * @param db Target gain in dB (clamped to ES7243E maximum +37.5 dB)
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail (e.g. codec not opened)
 */
esp_err_t bsp_extra_codec_set_mic_gain(float db);

/**
 * @brief Stop I2S function.
 *
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail
 */
esp_err_t bsp_extra_codec_dev_stop(void);

/**
 * @brief Fully deinitialize codec driver state.
 *
 * Stops any open streams and resets internal handles so that the next
 * call to bsp_extra_codec_init() will re-create them from scratch.
 * Call this after the codec chips have been power-cycled (e.g. SEN_EN
 * toggled) so the driver re-probes the I2C registers.
 *
 * @return
 *    - ESP_OK: Success
 */
esp_err_t bsp_extra_codec_deinit(void);

/**
 * @brief Resume I2S function.
 *
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail
 */
esp_err_t bsp_extra_codec_dev_resume(void);

/**
 * @brief Set I2S format to codec play.
 *
 * @param rate: Sample rate of sample
 * @param bits_cfg: Bit lengths of one channel data
 * @param ch: Channels of sample
 *
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail
 */
esp_err_t bsp_extra_codec_play_set_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch);

/**
 * @brief Set I2S format to codec record.
 *
 * @param rate: Sample rate of sample
 * @param bits_cfg: Bit lengths of one channel data
 * @param ch: Channels of sample
 *
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail
 */
esp_err_t bsp_extra_codec_record_set_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch);

/**
 * @brief Read data from recorder.
 *
 * When the default 16 kHz / 16-bit microphone format is active, this path
 * returns AFE-processed mono PCM for recording. Use bsp_extra_get_feed_data()
 * when raw interleaved microphone channels are required.
 *
 * @param audio_buffer: The pointer of receiving data buffer
 * @param len: Max data buffer length
 * @param bytes_read: Byte number that actually be read, can be NULL if not needed
 * @param timeout_ms: Max block time
 *
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail
 */
esp_err_t bsp_extra_i2s_read(void *audio_buffer, size_t len, size_t *bytes_read, uint32_t timeout_ms);

/**
 * @brief Read processed AFE mono data and matching pre-AFE mono data.
 *
 * When the default 16 kHz / 16-bit microphone format is active, processed_buffer
 * receives AFE output and raw_buffer receives the selected microphone channel
 * before AFE processing. Both outputs use mono 16-bit PCM and return the same
 * byte count.
 *
 * @param processed_buffer: Buffer for AFE-processed mono PCM
 * @param raw_buffer: Buffer for pre-AFE mono PCM from the selected microphone channel
 * @param len: Max buffer length for each output
 * @param bytes_read: Byte number actually written to each output buffer, can be NULL
 * @param timeout_ms: Max block time
 *
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail
 */
esp_err_t bsp_extra_i2s_read_processed_and_raw(void *processed_buffer, void *raw_buffer, size_t len, size_t *bytes_read, uint32_t timeout_ms);

/**
 * @brief Read processed AFE mono data, selected pre-AFE mono data, and true raw interleaved ADC slots.
 *
 * When the default 16 kHz / 16-bit microphone format is active, processed_buffer
 * receives AFE output, raw_buffer receives the selected microphone channel before
 * AFE processing, and raw_interleaved_buffer receives the original ES7243E ADC
 * samples in interleaved slot order from the same capture frames.
 *
 * @param processed_buffer Buffer for AFE-processed mono PCM
 * @param raw_buffer Buffer for pre-AFE mono PCM from the selected microphone channel
 * @param raw_interleaved_buffer Buffer for original interleaved ADC PCM
 * @param len Max buffer length for processed_buffer and raw_buffer
 * @param raw_interleaved_len Max buffer length for raw_interleaved_buffer
 * @param bytes_read Byte number actually written to processed_buffer and raw_buffer, can be NULL
 * @param raw_interleaved_bytes_read Byte number actually written to raw_interleaved_buffer, can be NULL
 * @param timeout_ms Max block time
 *
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail
 */
esp_err_t bsp_extra_i2s_read_processed_raw_and_interleaved(void *processed_buffer,
														   void *raw_buffer,
														   void *raw_interleaved_buffer,
														   size_t len,
														   size_t raw_interleaved_len,
														   size_t *bytes_read,
														   size_t *raw_interleaved_bytes_read,
														   uint32_t timeout_ms);

/**
 * @brief Write data to player.
 *
 * @param audio_buffer: The pointer of sent data buffer
 * @param len: Max data buffer length
 * @param bytes_written: Byte number that actually be sent, can be NULL if not needed
 * @param timeout_ms: Max block time
 *
 * @return
 *    - ESP_OK: Success
 *    - Others: Fail
 */
esp_err_t bsp_extra_i2s_write(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms);

/**
 * @brief Reset the playback RMS accumulator (sum of squares and sample count).
 *        Safe to call when playback is idle or before starting a new file.
 *
 * @return
 *      - ESP_OK: Success
 */
esp_err_t bsp_extra_playback_rms_reset(void);

/**
 * @brief Enable or disable RMS accumulation inside the playback write path.
 *        Default is disabled. When disabled the write path incurs zero overhead.
 *
 * @param enable true to start accumulating, false to stop
 * @return
 *      - ESP_OK: Success
 */
esp_err_t bsp_extra_playback_rms_enable(bool enable);

/**
 * @brief Read back the accumulated playback RMS statistics.
 *
 * @param sum_sq  Output: accumulated sum of squared int16 samples. May be NULL.
 * @param count   Output: number of int16 samples accumulated. May be NULL.
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_STATE: playback format unknown or not 16-bit PCM
 */
esp_err_t bsp_extra_playback_rms_get(uint64_t *sum_sq, uint32_t *count);


/**
 * @brief Initialize codec play and record handle.
 *
 * @return
 *      - ESP_OK: Success
 *      - Others: Fail
 */
esp_err_t bsp_extra_codec_init();

/**
 * @brief Initialize audio player task.
 *
 * @param path file path
 *
 * @return
 *      - ESP_OK: Success
 *      - Others: Fail
 */
esp_err_t bsp_extra_player_init(void);

/**
 * @brief Delete audio player task.
 *
 * @return
 *      - ESP_OK: Success
 *      - Others: Fail
 */
esp_err_t bsp_extra_player_del(void);

/**
 * @brief Initialize a file iterator instance
 *
 * @param path The file path for the iterator.
 * @param ret_instance A pointer to the file iterator instance to be returned.
 * @return
 *     - ESP_OK: Successfully initialized the file iterator instance.
 *     - ESP_FAIL: Failed to initialize the file iterator instance due to invalid parameters or memory allocation failure.
 */
esp_err_t bsp_extra_file_instance_init(const char *path, file_iterator_instance_t **ret_instance);

/**
 * @brief Play the audio file at the specified index in the file iterator
 *
 * @param instance The file iterator instance.
 * @param index The index of the file to play within the iterator.
 * @return
 *     - ESP_OK: Successfully started playing the audio file.
 *     - ESP_FAIL: Failed to play the audio file due to invalid parameters or file access issues.
 */
esp_err_t bsp_extra_player_play_index(file_iterator_instance_t *instance, int index);

/**
 * @brief Play the audio file specified by the file path
 *
 * @param file_path The path to the audio file to be played.
 * @return
 *     - ESP_OK: Successfully started playing the audio file.
 *     - ESP_FAIL: Failed to play the audio file due to file access issues.
 */
esp_err_t bsp_extra_player_play_file(const char *file_path);

/**
 * @brief Register a callback function for the audio player
 *
 * @param cb The callback function to be registered.
 * @param user_data User data to be passed to the callback function.
 */
void bsp_extra_player_register_callback(audio_player_cb_t cb, void *user_data);

/**
 * @brief Check if the specified audio file is currently playing
 *
 * @param file_path The path to the audio file to check.
 * @return
 *     - true: The specified audio file is currently playing.
 *     - false: The specified audio file is not currently playing.
 */
bool bsp_extra_player_is_playing_by_path(const char *file_path);

/**
 * @brief Check if the audio file at the specified index is currently playing
 *
 * @param instance The file iterator instance.
 * @param index The index of the file to check.
 * @return
 *     - true: The audio file at the specified index is currently playing.
 *     - false: The audio file at the specified index is not currently playing.
 */
bool bsp_extra_player_is_playing_by_index(file_iterator_instance_t *instance, int index);

/**
 * @brief 
 *
 * @return 
 */
esp_err_t bsp_extra_get_feed_data(bool is_get_raw_channel, int16_t *buffer, int buffer_len);
/**
 * @brief 
 *
 * @return 
 */
int bsp_extra_get_feed_channel(void);

/**
 * @brief Set experimental AFE input format without reopening the codec stream.
 *
 * @param input_format Supported values: "M", "MR", "MMRN".
 * @return
 *     - ESP_OK: format accepted and applied or staged.
 *     - ESP_ERR_INVALID_ARG: format is unsupported.
 *     - Others: live AFE reinit failed.
 */
esp_err_t bsp_extra_set_afe_input_format(const char *input_format);

/**
 * @brief Get current experimental AFE input format.
 *
 * @return Current AFE input format string.
 */
const char *bsp_extra_get_afe_input_format(void);

/**
 * @brief Set experimental MR reference source.
 *
 * @param source Supported values: "adc", "playback".
 * @return
 *     - ESP_OK: source accepted.
 *     - ESP_ERR_INVALID_ARG: source is unsupported.
 */
esp_err_t bsp_extra_set_afe_reference_source(const char *source);

/**
 * @brief Get current experimental MR reference source.
 *
 * @return Current MR reference source string.
 */
const char *bsp_extra_get_afe_reference_source(void);

/**
 * @brief 
 *
 * @return 
 */
char *bsp_extra_get_input_format(void);

#ifdef __cplusplus
}
#endif
