
#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/sdmmc_host.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "config.h"
#include "esp_codec_dev.h"
#include "sdkconfig.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

#define BSP_HW_VERSION          "2.0.0"

/*
    Pay particular attention to:
    There are two ways to power up a device and run it:
    1. Connect the battery and usb without pressing the BSP_BUTTON_ONOFF button to activate the device
    2. Only connect battery, need to press BSP_BUTTON_ONOFF button to activate device (BSP_PWR_HOLD output high level)
*/

/* Button */
#define BSP_BUTTON_UP               (1ULL << 0) // EXP_PIN_NUM_0, INPUT
#define BSP_BUTTON_DOWN             (1ULL << 1) // EXP_PIN_NUM_1, INPUT
#define BSP_BUTTON_LEFT             (1ULL << 2) // EXP_PIN_NUM_2, INPUT
#define BSP_BUTTON_RIGHT            (1ULL << 3) // EXP_PIN_NUM_3, INPUT
#define BSP_BUTTON_CONFIRM          (1ULL << 4) // EXP_PIN_NUM_4, INPUT
#define BSP_PWR_HOLD                (1ULL << 20) // EXP_PIN_NUM_20, OUTPUT 
#define BSP_BUTTON_RETURN_GPIO      (GPIO_NUM_0)

#define BSP_BUTTON_ONOFF            (GPIO_NUM_9)

#define BSP_POWER_BUTTON_LONG_PRESS_DURATION_MS (3000) // Duration in milliseconds for a long press

#define BSP_EXP_PIN5        (1ULL << 5) // EXP_PIN_NUM_5, INPUT
#define BSP_EXP_PIN11       (1ULL << 11) // EXP_PIN_NUM_11, INPUT

// Charger
#define BSP_CHARGER_STAT            (1ULL << 6) // EXP_PIN_NUM_6, INPUT
#define BSP_CHARGER_IN              (1ULL << 19) // EXP_PIN_NUM_19, INPUT

/* Battery */
#define BSP_BAT_ADC_EN              (1ULL << 17) // EXP_PIN_NUM_17
#define BSP_BAT_ADC_EN_ACTIVE_LEVEL (1)
#define BSP_VBAT_ADC                (GPIO_NUM_1) 
#define BSP_VBAT_DIVIDER_NUMERATOR  (2)
#define BSP_VBAT_DIVIDER_DENOMINATOR (1)

/* I2C0 */
#define BSP_I2C_0_SCL               (GPIO_NUM_18)
#define BSP_I2C_0_SDA               (GPIO_NUM_17)
#define BSP_EXP_IO_INT              (GPIO_NUM_2)

/* I2C1 */
#define BSP_I2C_1_SCL               (GPIO_NUM_48)
#define BSP_I2C_1_SDA               (GPIO_NUM_47)

/* Sensor */
#define BSP_SEN_EN                  (1ULL << 12) // EXP_PIN_NUM_12
#define BSP_IMU_INT1                (1ULL << 10) // EXP_PIN_NUM_10, INPUT
#define BSP_RTC_OUT_INT             GPIO_NUM_45 

/* SD card */
#define BSP_SD_D0                   (GPIO_NUM_16)
#define BSP_SD_CLK                  (GPIO_NUM_15)
#define BSP_SD_CMD                  (GPIO_NUM_14)
#define BSP_SD_PWR_EN               (1ULL << 13) // EXP_PIN_NUM_13
#define BSP_SD_DETECT               (1ULL << 7)  // EXP_PIN_NUM_7, INPUT

/* Audio */
// ES7243E
#define BSP_ADC_I2S_MCLK            (GPIO_NUM_38)           //  GPIO_NUM_38
#define BSP_ADC_I2S_SCLK            (GPIO_NUM_39)           //  GPIO_NUM_39
#define BSP_ADC_I2S_LRLK            (GPIO_NUM_40)
#define BSP_ADC_I2S_SDIN            (GPIO_NUM_42)   
#define BSP_ADC_I2S_SDOUT           (GPIO_NUM_NC)
#define ES7243E_CODEC_ADDR          (0x14)          // Note: The I2C address coincides with the bmm350 sensor, and the I2C device address of the bmm350 will be changed in the next version

// ES8311
#define BSP_DAC_I2S_MCLK            (GPIO_NUM_38)           //  GPIO_NUM_38
#define BSP_DAC_I2S_SCLK            (GPIO_NUM_39)           //  GPIO_NUM_39
#define BSP_DAC_I2S_LRLK            (GPIO_NUM_40)
#define BSP_DAC_I2S_SDIN            (GPIO_NUM_NC)
#define BSP_DAC_I2S_SDOUT           (GPIO_NUM_41)   







#define BSP_PA_PWR_EN               (1ULL << 18) // EXP_PIN_NUM_18 // 

/* Display */
#define BSP_LCD_PWR_EN              (1ULL << 14) // EXP_PIN_NUM_14
#define BSP_LCD_RST                 (1ULL << 21) // EXP_PIN_NUM_21
#define BSP_LCD_CS                  (GPIO_NUM_10) 
#define BSP_LCD_SDA                 (GPIO_NUM_11) 
#define BSP_LCD_SCK                 (GPIO_NUM_12) 
#define BSP_LCD_A0                  (GPIO_NUM_13) 
#define BSP_LCD_PWM                 (GPIO_NUM_46) 
#define BSP_LCD_H_RES               (240)
#define BSP_LCD_V_RES               (320)
#define BSP_LCD_PIXEL_CLOCK_HZ      (40 * 1000 * 1000)
#define BSP_LCD_CMD_BITS            (8)
#define BSP_LCD_PARAM_BITS          (8)
#define BSP_LCD_BITS_PER_PIXEL      (16)

/* Debug */
#define BSP_UART1_TX                (GPIO_NUM_43)
#define BSP_UART1_RX                (GPIO_NUM_44)

/* GNSS */
#define BSP_GNSS_PWR_EN             (1ULL << 16) // EXP_PIN_NUM_16
#define BSP_GNSS_RST                (1ULL << 23) // EXP_PIN_NUM_23
#define BSP_GNSS_RX                 BSP_UART1_TX   // UART1_TX
#define BSP_GNSS_TX                 BSP_UART1_RX   // UART1_RX
#define BSP_GNSS_PPS0               (GPIO_NUM_21)  
#define BSP_GNSS_VRTC_EN            (1ULL << 15) // EXP_PIN_NUM_15
#define BSP_GNSS_RTC_INT            (1ULL << 8) // EXP_PIN_NUM_8
#define BSP_GNSS_SLEEP_INT          (1ULL << 9) // EXP_PIN_NUM_9

/* LoRa */

#define BSP_LORA_SPI_CS            (GPIO_NUM_6)
#define BSP_LORA_SPI_SCK           (GPIO_NUM_3)
#define BSP_LORA_SPI_MOSI          (GPIO_NUM_4)
#define BSP_LORA_SPI_MISO          (GPIO_NUM_5)
#define BSP_LORA_BUSY              (GPIO_NUM_8)
#define BSP_LORA_INT               (GPIO_NUM_7)
#define BSP_LORA_RESET             (1ULL << 22)

#define BSP_BOOT                    GPIO_NUM_0


#define ALL_IO_EXPANDER_OUTPUT_PIN (BSP_BAT_ADC_EN|BSP_SEN_EN|BSP_SD_PWR_EN|BSP_PA_PWR_EN|BSP_LCD_PWR_EN|BSP_LCD_RST|BSP_GNSS_PWR_EN|BSP_GNSS_RST|BSP_GNSS_VRTC_EN|BSP_GNSS_RTC_INT|BSP_GNSS_SLEEP_INT|BSP_LORA_RESET|BSP_PWR_HOLD)
#define ALL_IO_EXPANDER_INPUT_PIN  (BSP_BUTTON_UP|BSP_BUTTON_DOWN|BSP_BUTTON_LEFT|BSP_BUTTON_RIGHT|BSP_BUTTON_CONFIRM|BSP_EXP_PIN5|BSP_CHARGER_STAT| BSP_CHARGER_IN|BSP_IMU_INT1|BSP_EXP_PIN11|BSP_SD_DETECT)

#define EXPANDER_IO_DIR_INPUT       0
#define EXPANDER_IO_DIR_OUTPUT      1

// ADC
#define VBAT_ADC1_CHAN0          ADC_CHANNEL_0
#define VBAT_ADC_ATTEN           ADC_ATTEN_DB_12

// LEDC for LCD backlight PWM control
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (BSP_LCD_PWM) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz
#define VBAT_ADC_SAMPLE_COUNT   10
#define VBAT_ADC_DISCARD_COUNT  4

// UART
#define GNSS_UART_NUM  1
#define GNSS_UART_RX_BUF_SIZE        (1024)
#define GNSS_UART_TX_BUF_SIZE        (1024)

// I2C
#define I2C_PORT_NUM_0 0
#define I2C_PORT_NUM_1 1
     

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*bsp_button_event_cb_t)(uint32_t button_mask, bool pressed, void *user_ctx);
typedef void (*bsp_power_shutdown_cb_t)(void *user_ctx);


/**************************************************************************************************
 *
 * I2S audio interface
 *
 * There are two devices connected to the I2S peripheral:
 *  - Codec ES8311 for output(playback) and input(recording) path
 *
 * For speaker initialization use bsp_audio_codec_speaker_init() which is inside initialize I2S with bsp_audio_init().
 * For microphone initialization use bsp_audio_codec_microphone_init() which is inside initialize I2S with bsp_audio_init().
 * After speaker or microphone initialization, use functions from esp_codec_dev for play/record audio.
 * Example audio play:
 * \code{.c}
 * esp_codec_dev_set_out_vol(spk_codec_dev, DEFAULT_VOLUME);
 * esp_codec_dev_open(spk_codec_dev, &fs);
 * esp_codec_dev_write(spk_codec_dev, wav_bytes, bytes_read_from_spiffs);
 * esp_codec_dev_close(spk_codec_dev);
 * \endcode
 **************************************************************************************************/

/**
 * @brief Init audio
 *
 * @note There is no deinit audio function. Users can free audio resources by calling i2s_del_channel()
 * @warning The type of i2s_config param is depending on IDF version.
 * @param[in]  i2s_adc_config I2S ADC configuration. Pass NULL to use default values (Mono, duplex, 16bit, 22050 Hz)
 * @param[in]  i2s_dac_config I2S DAC configuration. Pass NULL to use default values (Mono, duplex, 16bit, 22050 Hz)
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_NOT_SUPPORTED The communication mode is not supported on the current chip
 *      - ESP_ERR_INVALID_ARG   NULL pointer or invalid configuration
 *      - ESP_ERR_NOT_FOUND     No available I2S channel found
 *      - ESP_ERR_NO_MEM        No memory for storing the channel information
 *      - ESP_ERR_INVALID_STATE This channel has not initialized or already started
 */
esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_adc_config, const i2s_std_config_t *i2s_dac_config);

/**
 * @brief Initialize speaker codec device
 *
 * @return Pointer to codec device handle or NULL when error occurred
 */
esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void);

/**
 * @brief Initialize microphone codec device
 *
 * @return Pointer to codec device handle or NULL when error occurred
 */
esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void);

/**************************************************************************************************
 *
 * SPIFFS
 *
 * After mounting the SPIFFS, it can be accessed with stdio functions ie.:
 * \code{.c}
 * FILE* f = fopen(BSP_SPIFFS_MOUNT_POINT"/hello.txt", "w");
 * fprintf(f, "Hello World!\n");
 * fclose(f);
 * \endcode
 **************************************************************************************************/
#define BSP_SPIFFS_MOUNT_POINT      CONFIG_BSP_SPIFFS_MOUNT_POINT

/**
 * @brief Mount SPIFFS to virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if esp_vfs_spiffs_register was already called
 *      - ESP_ERR_NO_MEM if memory can not be allocated
 *      - ESP_FAIL if partition can not be mounted
 *      - other error codes
 */
esp_err_t bsp_spiffs_mount(void);

/**
 * @brief Unmount SPIFFS from virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NOT_FOUND if the partition table does not contain SPIFFS partition with given label
 *      - ESP_ERR_INVALID_STATE if esp_vfs_spiffs_unregister was already called
 *      - ESP_ERR_NO_MEM if memory can not be allocated
 *      - ESP_FAIL if partition can not be mounted
 *      - other error codes
 */
esp_err_t bsp_spiffs_unmount(void);

/**************************************************************************************************
 *
 * uSD card
 *
 * After mounting the uSD card, it can be accessed with stdio functions ie.:
 * \code{.c}
 * FILE* f = fopen(BSP_MOUNT_POINT"/hello.txt", "w");
 * fprintf(f, "Hello %s!\n", bsp_sdcard->cid.name);
 * fclose(f);
 * \endcode
 **************************************************************************************************/
#define BSP_SD_MOUNT_POINT      CONFIG_BSP_SD_MOUNT_POINT
extern sdmmc_card_t *bsp_sdcard;

/**
 * @brief Mount microSD card to virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if esp_vfs_fat_sdmmc_mount was already called
 *      - ESP_ERR_NO_MEM if memory cannot be allocated
 *      - ESP_FAIL if partition cannot be mounted
 *      - other error codes from SDMMC or SPI drivers, SDMMC protocol, or FATFS drivers
 */
esp_err_t bsp_sdcard_mount(void);

/**
 * @brief Unmount microSD card from virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NOT_FOUND if the partition table does not contain FATFS partition with given label
 *      - ESP_ERR_INVALID_STATE if esp_vfs_fat_spiflash_mount was already called
 *      - ESP_ERR_NO_MEM if memory can not be allocated
 *      - ESP_FAIL if partition can not be mounted
 *      - other error codes from wear levelling library, SPI flash driver, or FATFS drivers
 */
esp_err_t bsp_sdcard_unmount(void);

/**
 * @brief Initialize LCD backlight with given brightness
 */
void lcd_bl_init(uint8_t brightness);

/**
 * @brief Set LCD backlight brightness
 */
void lcd_bl_set(int brightness );

/**
 * @brief Turn on LCD backlight with default brightness
 */
void lcd_bl_on(void);

/**
 * @brief Turn off LCD backlight
 */
void lcd_bl_off(void);

/**
 * @brief Config display brightness
 * 
 * @param level The level of backlight, from 0 - 255
 *
 * @return 
 *      - ESP_OK on success
 *      - Other on error
 */
esp_err_t bsp_display_brightness_set(uint8_t level);

/**
 * @brief Initialize the ST7789P3 LCD panel over SPI.
 *
 * @note Call this after bsp_power_up_init() so LCD power and expander outputs are available.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if panel is already initialized or board power path is not ready
 *      - Other error codes from SPI/esp_lcd stack
 */
esp_err_t bsp_display_init(void);

/**
 * @brief Deinitialize the LCD panel and free SPI resources.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if panel is not initialized
 *      - Other error codes from esp_lcd or SPI driver
 */
esp_err_t bsp_display_deinit(void);

/**
 * @brief Get initialized LCD panel handle.
 *
 * @return panel handle, or NULL if bsp_display_init() has not been called
 */
esp_lcd_panel_handle_t bsp_display_get_panel(void);

/**
 * @brief Get initialized LCD panel IO handle.
 *
 * @return panel IO handle, or NULL if bsp_display_init() has not been called
 */
esp_lcd_panel_io_handle_t bsp_display_get_panel_io(void);

/**
 * @brief Draw a rectangular RGB565 bitmap to the LCD.
 *
 * @param x_start Inclusive start x coordinate
 * @param y_start Inclusive start y coordinate
 * @param x_end Exclusive end x coordinate
 * @param y_end Exclusive end y coordinate
 * @param color_data Pointer to RGB565 pixel data
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG on invalid arguments
 *      - ESP_ERR_INVALID_STATE if panel is not initialized
 *      - Other error codes from esp_lcd stack
 */
esp_err_t bsp_display_draw_bitmap(int x_start, int y_start, int x_end, int y_end, const void *color_data);

/**
 * @brief Control battery voltage detection circuit power
 * 
 * @param on true to power on the battery voltage detection circuit, false to power off
 * 
 * @return None
 */
void bsp_bat_power_control(bool on);

/**
 * @brief Read battery voltage
 *
 * @return voltage in mV
 */
uint16_t bsp_battery_voltage_read(void);

/**
 * @brief Get the rtc int count object
 * 
 * @return uint32_t 
 */
uint32_t get_rtc_int_count( void );

/**
 * @brief Set output expander io level
 *
 * @return None
 */
void bsp_exp_output_io_set_level(uint32_t expander_io, uint8_t level);

/**
 * @brief Get input expander io level
 *
 * @return level of the input expander io
 */
uint32_t bsp_exp_input_io_get_level(uint32_t expander_io);

/**
 * @brief Set the expander io dir
 * 
 * @param pin_num_mask 
 * @param direction 
 */
void bsp_exp_io_set_dir(uint32_t pin_num_mask, uint8_t direction);

/**
 * @brief 
 * 
 * @return esp_err_t 
 */
esp_err_t bsp_deinit_io_expander( void );

/**
 * @brief Register a callback for IO expander button events
 *
 * @param callback Called from the expander interrupt handling task when a button level changes
 * @param user_ctx Opaque user pointer passed back to callback
 *
 * @return
 *     - ESP_OK on success
 */
esp_err_t bsp_button_event_callback_register(bsp_button_event_cb_t callback, void *user_ctx);

/**
 * @brief Register a callback invoked before the board drops BSP_PWR_HOLD on long-press shutdown
 *
 * @param callback Called from the BSP power-button task before board peripherals are powered down
 * @param user_ctx Opaque user pointer passed back to callback
 *
 * @return
 *     - ESP_OK on success
 */
esp_err_t bsp_power_shutdown_callback_register(bsp_power_shutdown_cb_t callback, void *user_ctx);

/**
 * @brief Board power up init
 *
 * Hold the power pin in first
 *
 * @return
 *     - ESP_OK              On success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_INVALID_STATE Device remains in power-off state, caller should stop normal app startup
 */
esp_err_t bsp_power_up_init(void);

/**
 * @brief Minimal BSP power-hold deep-sleep current test.
 *
 * Only initializes I2C0 and the TCA6424 IO expander, drives all expander outputs
 * low except BSP_PWR_HOLD, keeps BSP_PWR_HOLD asserted for 10 seconds, then enters
 * deep sleep without enabling any wake source.
 *
 * @return
 *     - ESP_OK on success. Function does not return after deep sleep starts.
 *     - Other error codes on bootstrap or sleep configuration failure.
 */
esp_err_t bsp_power_hold_deep_sleep_test(void);


/**
 * @brief 
 * 
 * @return esp_err_t 
 */
esp_err_t bsp_power_hold_gnss_test(void);

#ifdef __cplusplus
}
#endif
