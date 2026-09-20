#include "sdkconfig.h"

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "meshpager_x2.h"

static const char *TAG = "bsp_lcd_st7789p3";

#define BSP_LCD_SPI_HOST SPI3_HOST

static esp_lcd_panel_io_handle_t s_panel_io = NULL;
static esp_lcd_panel_handle_t s_panel = NULL;
static bool s_lcd_spi_bus_initialized = false;

static esp_err_t bsp_lcd_spi_bus_init(void)
{
	spi_bus_config_t bus_config = {
		.sclk_io_num = BSP_LCD_SCK,
		.mosi_io_num = BSP_LCD_SDA,
		.miso_io_num = GPIO_NUM_NC,
		.quadwp_io_num = GPIO_NUM_NC,
		.quadhd_io_num = GPIO_NUM_NC,
		.max_transfer_sz = BSP_LCD_H_RES * 40 * sizeof(uint16_t),
	};

	if (s_lcd_spi_bus_initialized) {
		return ESP_OK;
	}

	ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO), TAG, "init lcd spi bus failed");
	gpio_set_drive_capability(BSP_LCD_SCK, GPIO_DRIVE_CAP_0);   
	s_lcd_spi_bus_initialized = true;
	return ESP_OK;
}

static void bsp_lcd_hard_reset(void)
{
	bsp_exp_output_io_set_level(BSP_LCD_PWR_EN, 1);
	vTaskDelay(pdMS_TO_TICKS(20));
	bsp_exp_output_io_set_level(BSP_LCD_RST, 0);
	vTaskDelay(pdMS_TO_TICKS(20));
	bsp_exp_output_io_set_level(BSP_LCD_RST, 1);
	vTaskDelay(pdMS_TO_TICKS(120));
}

esp_err_t bsp_display_init(void)
{
	esp_lcd_panel_io_spi_config_t io_config = {
		.cs_gpio_num = BSP_LCD_CS,
		.dc_gpio_num = BSP_LCD_A0,
		.spi_mode = 0,
		.pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
		.trans_queue_depth = 10,
		.lcd_cmd_bits = BSP_LCD_CMD_BITS,
		.lcd_param_bits = BSP_LCD_PARAM_BITS,
	};
	esp_lcd_panel_dev_config_t panel_config = {
		.reset_gpio_num = GPIO_NUM_NC,
		.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
		.data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
		.bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
	};
	esp_err_t err;

	if (s_panel != NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	err = bsp_lcd_spi_bus_init();
	if (err != ESP_OK) {
		return err;
	}

	bsp_lcd_hard_reset();

	err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_HOST, &io_config, &s_panel_io);
	if (err != ESP_OK) {
		return err;
	}

	err = esp_lcd_new_panel_st7789(s_panel_io, &panel_config, &s_panel);
	if (err != ESP_OK) {
		esp_lcd_panel_io_del(s_panel_io);
		s_panel_io = NULL;
		return err;
	}

	err = esp_lcd_panel_reset(s_panel);
	if (err != ESP_OK) {
		goto err_cleanup;
	}

	err = esp_lcd_panel_init(s_panel);
	if (err != ESP_OK) {
		goto err_cleanup;
	}

	err = esp_lcd_panel_invert_color(s_panel, true);
	if (err != ESP_OK) {
		goto err_cleanup;
	}

	err = esp_lcd_panel_swap_xy(s_panel, false);
	if (err != ESP_OK) {
		goto err_cleanup;
	}

	err = esp_lcd_panel_mirror(s_panel, false, false);	// err = esp_lcd_panel_mirror(s_panel, true, true);
	if (err != ESP_OK) {
		goto err_cleanup;
	}

	err = esp_lcd_panel_set_gap(s_panel, 0, 0);
	if (err != ESP_OK) {
		goto err_cleanup;
	}

	err = esp_lcd_panel_disp_on_off(s_panel, true);
	if (err != ESP_OK) {
		goto err_cleanup;
	}

	lcd_bl_init(100);
	lcd_bl_off();
	ESP_LOGI(TAG, "ST7789P3 LCD initialized (%dx%d), backlight off", BSP_LCD_H_RES, BSP_LCD_V_RES);
	return ESP_OK;

err_cleanup:
	if (s_panel != NULL) {
		esp_lcd_panel_del(s_panel);
		s_panel = NULL;
	}
	if (s_panel_io != NULL) {
		esp_lcd_panel_io_del(s_panel_io);
		s_panel_io = NULL;
	}
	return err;
}

esp_err_t bsp_display_deinit(void)
{
	esp_err_t err = ESP_OK;

	if (s_panel == NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	lcd_bl_off();
	err |= esp_lcd_panel_disp_on_off(s_panel, false);
	err |= esp_lcd_panel_del(s_panel);
	s_panel = NULL;

	if (s_panel_io != NULL) {
		err |= esp_lcd_panel_io_del(s_panel_io);
		s_panel_io = NULL;
	}

	bsp_exp_output_io_set_level(BSP_LCD_RST, 0);
	bsp_exp_output_io_set_level(BSP_LCD_PWR_EN, 0);

	if (s_lcd_spi_bus_initialized) {
		err |= spi_bus_free(BSP_LCD_SPI_HOST);
		s_lcd_spi_bus_initialized = false;
	}

	return err;
}

esp_lcd_panel_handle_t bsp_display_get_panel(void)
{
	return s_panel;
}

esp_lcd_panel_io_handle_t bsp_display_get_panel_io(void)
{
	return s_panel_io;
}

esp_err_t bsp_display_draw_bitmap(int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
	if (s_panel == NULL) {
		return ESP_ERR_INVALID_STATE;
	}

	if (color_data == NULL || x_start < 0 || y_start < 0 || x_end > BSP_LCD_H_RES || y_end > BSP_LCD_V_RES || x_end <= x_start || y_end <= y_start) {
		return ESP_ERR_INVALID_ARG;
	}

	return esp_lcd_panel_draw_bitmap(s_panel, x_start, y_start, x_end, y_end, color_data);
}