#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "unity_test_utils_memory.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_io_expander_tca6424.h"

// smtc_ralf
#include "ralf_defs.h"
#include "ralf_drv.h"
#include "ralf_lr20xx.h"
#include "ralf.h"

// smtc_ral
#include "lr_fhss_v1_base_types.h"
#include "ral_defs.h"
#include "ral_drv.h"
#include "ral_lr20xx.h"
#include "ral.h"

#include "lr20xx_hal.h"
#include "lr20xx_init.h"
#include "lr20xx_pa_pwr_cfg.h"
#include "radio_utilities.h"

#include "ral_lr20xx_bsp.h"
#include "lr20xx_radio_common.h"

// Pinout for ESP32-S3-Board
#define I2C_MASTER_SCL_IO   6          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO   5          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM      I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_ADDRESS         ESP_IO_EXPANDER_I2C_TCA6424_ADDRESS_GND
#define LR20XX_EXP_RST_OUTPUT_PIN IO_EXPANDER_PIN_NUM_22

static const char *TAG = "LR20XX test";
static esp_io_expander_handle_t io_expander = NULL;
static i2c_master_bus_handle_t i2c_handle = NULL;


lr20xx_hal_context_t m_lr2021;
uint8_t m_tx_buf[256] = {0};
uint16_t m_buf_len = 20;
static const ralf_t modem_radio = RALF_LR20XX_INSTANTIATE(&m_lr2021);
static ralf_params_lora_t radio_params_lora = 
{
    .sync_word                       = 0x34,
    .symb_nb_timeout                 = 0,
    .rf_freq_in_hz                   = 868000000,
    .output_pwr_in_dbm               = 10,
    .mod_params.cr                   = RAL_LORA_CR_4_5,
    .mod_params.sf                   = RAL_LORA_SF7,
    .mod_params.bw                   = RAL_LORA_BW_125_KHZ,
    .mod_params.ldro                 = 0,
    .pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT,
    .pkt_params.pld_len_in_bytes     = 20,
    .pkt_params.crc_is_on            = true,
    .pkt_params.invert_iq_is_on      = false,
    .pkt_params.preamble_len_in_symb = 8,
};

void expander_io_lr20xx_reset( void)
{
    esp_err_t ret = esp_io_expander_set_level(io_expander, LR20XX_EXP_RST_OUTPUT_PIN, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "Set reset pin low failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ret = esp_io_expander_set_level(io_expander, LR20XX_EXP_RST_OUTPUT_PIN, 1);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "Set reset pin high failed");
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void i2c_bus_init(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_handle);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

static void i2c_dev_tca6424_init(void)
{
    esp_err_t ret = esp_io_expander_new_i2c_tca6424(i2c_handle, I2C_ADDRESS, &io_expander);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "tca6424 create returned error");
}

// static void i2c_bus_deinit(void)
// {
//     esp_err_t ret = i2c_del_master_bus(i2c_handle);
//     TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C uninstall returned error");
// }

// static void i2c_dev_tca6424_deinit(void)
// {
//     esp_err_t ret = esp_io_expander_del(io_expander);
//     TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "tca6424 delete returned error");
// }

void app_main(void)
{
    i2c_bus_init();
    i2c_dev_tca6424_init();
    // Initialize reset pin using IO expander
    esp_err_t ret;

    /* Test output level function */
    ret = esp_io_expander_set_dir(io_expander, LR20XX_EXP_RST_OUTPUT_PIN, IO_EXPANDER_OUTPUT);
    ret = esp_io_expander_set_level(io_expander, LR20XX_EXP_RST_OUTPUT_PIN, 1);

    lr20xx_init(&m_lr2021);

    radio_params_lora.pkt_params.pld_len_in_bytes     = m_buf_len;

    ral_reset(&(modem_radio.ral));

    ral_status_t status = ral_init(&(modem_radio.ral));

    status = ralf_setup_lora(&modem_radio, &radio_params_lora);
    status = ral_set_dio_irq_params(&(modem_radio.ral), RAL_IRQ_TX_DONE);
    
    while(1)
    {
        ral_set_pkt_payload(&(modem_radio.ral), m_tx_buf, m_buf_len);
        ral_set_tx(&(modem_radio.ral)); 
        ESP_LOGI(TAG, "LR20XX is transmitting...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
    }
}