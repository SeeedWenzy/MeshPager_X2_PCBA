#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file peripherals.h
 * @brief Per-peripheral init / step / suspend callbacks for the periodic runner.
 *
 * Each peripheral exposes three callbacks matching the runner's
 * runner_init_fn / runner_step_fn / runner_suspend_fn signatures.
 * ctx is unused (passed as NULL) — shared state lives as file-scope statics
 * inside peripherals.c, mirroring examples/test_power/main/main.c.
 */

/* Display (LCD) */
esp_err_t display_init(void *ctx);
esp_err_t display_step(void *ctx);
void      display_suspend(void *ctx);

/* Audio codec (ES8311 play + ES7243E record), one job, switchable mode */
esp_err_t audio_init(void *ctx);
esp_err_t audio_step(void *ctx);
void      audio_suspend(void *ctx);

/* Wi-Fi AP */
esp_err_t wifi_init(void *ctx);
esp_err_t wifi_step(void *ctx);
void      wifi_suspend(void *ctx);

/* BLE non-connectable advertising */
esp_err_t ble_init(void *ctx);
esp_err_t ble_step(void *ctx);
void      ble_suspend(void *ctx);

/* GNSS NMEA parsing */
esp_err_t gnss_init(void *ctx);
esp_err_t gnss_step(void *ctx);
void      gnss_suspend(void *ctx);

/* Sensors: YSN8900E RTC + SPA06 + BMM350 */
esp_err_t sensor_init(void *ctx);
esp_err_t sensor_step(void *ctx);
void      sensor_suspend(void *ctx);

/* IMU: LSM6DSOX accel + gyro (shares I2C_1) */
esp_err_t imu_init(void *ctx);
esp_err_t imu_step(void *ctx);
void      imu_suspend(void *ctx);

/* LoRa TX / RX (share one radio, serialized by a mutex) */
esp_err_t lora_tx_init(void *ctx);
esp_err_t lora_tx_step(void *ctx);
void      lora_tx_suspend(void *ctx);
esp_err_t lora_rx_init(void *ctx);
esp_err_t lora_rx_step(void *ctx);
void      lora_rx_suspend(void *ctx);

/* SD card r/w (mounted once at boot) */
esp_err_t sdcard_init(void *ctx);
esp_err_t sdcard_step(void *ctx);
void      sdcard_suspend(void *ctx);

/* Battery ADC */
esp_err_t battery_init(void *ctx);
esp_err_t battery_step(void *ctx);
void      battery_suspend(void *ctx);

/* ---- boot helpers (called from main.c) ---- */

/** Mount the SD card once at boot (rail is already powered by bsp_power_up_init). */
esp_err_t peripherals_sdcard_mount_boot(void);

/** Load image + wav caches from SD into PSRAM. Warn-but-continue on failure. */
void peripherals_load_media(void);

/** Initialize every peripheral once at power-on (idempotent; warn-and-continue
 *  on per-device failure). Call after SD/media are ready, before runner start. */
void peripherals_init_all(void);

/** One-boot mic-capture bisection: probe raw mic level after enabling each
 *  peripheral one at a time, to find which one collapses the ES7243E capture. */
void peripherals_mic_bisect(void);

/** Create the shared LoRa DIO IRQ task (once). */
void peripherals_create_lora_irq_task(void);

/** Create the I2C_1 and LoRa resource mutexes. Call before registering jobs. */
esp_err_t peripherals_init_locks(void);

/** Board power-off callback (stops every peripheral). */
void peripherals_cleanup(void *ctx);

/* ---- console ---- */

/** Register the `audio_mode <play|record|both>` console command. */
void audio_cmd_register(void);

#ifdef __cplusplus
}
#endif
