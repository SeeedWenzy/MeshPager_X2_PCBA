#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register all peripheral power-off console commands.
 *
 * Registers the following commands, each of which fully tears down one
 * peripheral domain (stop driver -> cut power rail -> reconfigure data pins to
 * a low-leakage state):
 *  - power_gnss_off
 *  - power_sd_off
 *  - power_lcd_off
 *  - power_sen_audio_off
 */
void power_cmd_register_all(void);

#ifdef __cplusplus
}
#endif
