#pragma once

#include "esp_err.h"

esp_err_t sd_lcd_cmd_init(void);
esp_err_t sd_lcd_show_first_image(void);
esp_err_t sd_lcd_show_next_image(void);
esp_err_t sd_lcd_show_prev_image(void);
void sd_lcd_cmd_register_all(void);