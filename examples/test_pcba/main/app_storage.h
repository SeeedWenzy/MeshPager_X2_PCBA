#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize board storage: best-effort mount the SD card and mount the
 *        SPIFFS asset partition.
 *
 * The SD card is optional (no card inserted at boot is not fatal). The SPIFFS
 * asset partition is flash-backed and is expected to always be available, so
 * the test assets baked into it (audio / images / rgb565 frames) can be used
 * as a fallback when the SD card is not present.
 *
 * Call once at startup, before any audio/LCD command runs.
 */
void app_storage_init(void);

/**
 * @brief Whether the SD card was mounted successfully at boot.
 *
 * Used to decide whether asset paths should target the SD card or fall back
 * to the SPIFFS asset partition.
 */
bool app_storage_sd_available(void);

#ifdef __cplusplus
}
#endif
