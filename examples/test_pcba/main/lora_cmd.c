#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_console.h"
#include "esp_random.h"
#include "esp_spiffs.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_pm.h"
#include "nvs.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "meshpager_x2.h"

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
#include "lr20xx_radio_flrc_types.h"

static const char *TAG = "LR20XX";

static lr20xx_hal_context_t radio;
static const ralf_t modem_radio = RALF_LR20XX_INSTANTIATE(&radio);

static uint8_t __g_tx_done_flag = true;
static uint8_t __g_rx_buf[256] = {0};

static uint16_t __g_rx_ok_cnt = 0;
static uint16_t __g_rx_done_cnt = 0;
static uint16_t __g_rx_hdr_err_cnt = 0;
static uint16_t __g_rx_crc_err_cnt = 0;
static bool s_radio_dio_isr_ready = false;
static bool s_test_flrc_flag = false;


static struct {
    struct arg_int *freq;
    struct arg_int *power;
    struct arg_int *br_bw;
    struct arg_int *len;
    struct arg_int *cnt;
    struct arg_end *end;
} flrc_tx_args;

static const uint32_t mod_br_bps[LR20XX_RADIO_FLRC_BR_0_260_BW_0_307 + 1] = 
{
    2600000, 2080000, 1300000, 1040000, 650000, 520000, 325000, 260000
};
static const uint32_t mod_bw_hz[LR20XX_RADIO_FLRC_BR_0_260_BW_0_307 + 1] = 
{
    2666000, 2222000, 1333000, 1333000, 740000, 571000, 357000, 307000
};

// static const uint8_t default_syncword_1[4] = { 0x90, 0x56, 0x34, 0x12 };
static const uint8_t default_syncword_1[4] = { 0x55, 0x55, 0x55, 0x55 };
// static const uint8_t default_syncword_2[4] = { 0x00, 0x00, 0x00, 0x00 };
// static const uint8_t default_syncword_3[4] = { 0x00, 0x00, 0x00, 0x00 };
static int flrc_tx_test(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &flrc_tx_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, flrc_tx_args.end, argv[0]);
        return 1;
    }

    int      power  = 22;
    uint32_t freq   = 915000000;
    int      br_bw  = 0;
    uint32_t len    = 200;
    uint32_t cnt    = 100;
    uint32_t interval = 1000;  

    int8_t flrc_power = power;
    uint32_t flrc_freq = freq;
    uint32_t flrc_br_in_bps = mod_br_bps[0];
    uint32_t flrc_bw_dsb_in_hz = mod_bw_hz[0];
    uint8_t cr = RAL_FLRC_CR_3_4;
    uint8_t pulse_shape = RAL_FLRC_PULSE_SHAPE_BT_05;
    uint8_t crc = RAL_FLRC_CRC_2_BYTES;
    bool pld_is_fix = false;
    uint8_t match_sync_word = RAL_FLRC_RX_MATCH_SYNCWORD_1;
    uint8_t tx_syncword = RAL_FLRC_TX_SYNCWORD_1;
    uint8_t sync_word_len = RAL_FLRC_SYNCWORD_LENGTH_4_BYTES;
    uint16_t preamble_len_in_bits = RAL_FLRC_PREAMBLE_LENGTH_32_BITS;     // 

    char buf[512] = {0};
    for(uint8_t u8i = 0; u8i<len; u8i++)
    {
        buf[u8i] = u8i;
    }

    if (flrc_tx_args.power->count) {
        power = flrc_tx_args.power->ival[0];
        flrc_power = power;
    }
    if (flrc_tx_args.freq->count) {
        freq = flrc_tx_args.freq->ival[0];
        flrc_freq = freq;
    }
    if (flrc_tx_args.br_bw->count) {
        br_bw = flrc_tx_args.br_bw->ival[0];
        if (br_bw > 7) {
            br_bw = 7;
        }
    }
    if (flrc_tx_args.len->count) {
        len = flrc_tx_args.len->ival[0];
        if (len > 255) {
            len = 255;
        }
    }
    if (flrc_tx_args.cnt->count) {
        cnt = flrc_tx_args.cnt->ival[0];
        if (cnt > 200) {
            cnt = 200;
        }
    }


    uint8_t status = 0;
    ralf_params_flrc_t radio_params_flrc = { 
        .output_pwr_in_dbm  = flrc_power,
        .crc_polynomial  = 0x00000000,
        .crc_seed  = 0x00000000,
        .rf_freq_in_hz  = flrc_freq,
        .sync_word[0] = &default_syncword_1[0],
        .sync_word[1] = &default_syncword_1[0],
        .sync_word[2] = &default_syncword_1[0], 
        .mod_params.raw_bit_rate = 7 - br_bw,
        .mod_params.cr = cr,
        .mod_params.pulse_shape = pulse_shape,
        .pkt_params.crc_type = crc,
        .pkt_params.pld_len_in_bytes = len,
        .pkt_params.pld_is_fix = pld_is_fix,
        .pkt_params.match_sync_word = match_sync_word,
        .pkt_params.tx_syncword = tx_syncword,
        .pkt_params.sync_word_len = sync_word_len,
        .pkt_params.preamble_len = preamble_len_in_bits,
        .is_tx = true
    };
    flrc_br_in_bps = mod_br_bps[br_bw];
    flrc_bw_dsb_in_hz = mod_bw_hz[br_bw];
    ESP_LOGI(TAG,  "Power          = %d dB", flrc_power);
    ESP_LOGI(TAG,  "Frequency      = %u Hz", flrc_freq);
    ESP_LOGI(TAG, "TX bit rate        = %d", flrc_br_in_bps);
    ESP_LOGI(TAG, "TX bw dsb in hz    = %d", flrc_bw_dsb_in_hz);
    ESP_LOGI(TAG,  "pulse_shape    = %d ", radio_params_flrc.mod_params.pulse_shape);
    ESP_LOGI(TAG,  "cr             = %u ", radio_params_flrc.mod_params.cr);
    ESP_LOGI(TAG,  "crc_type       = %u ", radio_params_flrc.pkt_params.crc_type);
    ESP_LOGI(TAG,  "pld_is_fix       = %s ", radio_params_flrc.pkt_params.pld_is_fix == false ? "false":"true");    
    ESP_LOGI(TAG,  "sync_word_len  = %u ", radio_params_flrc.pkt_params.sync_word_len);
    ESP_LOGI(TAG,  "tx_syncword    = %u ", radio_params_flrc.pkt_params.tx_syncword);
    ESP_LOGI(TAG,  "preamble_len   = %u ", radio_params_flrc.pkt_params.preamble_len);
    ESP_LOGI(TAG,  "len            = %u ", len);
    ESP_LOGI(TAG,  "cnt            = %u ", cnt);
    
    ESP_LOGI(TAG,  "Start Tx flrc");

    status = ralf_setup_flrc(&modem_radio, &radio_params_flrc);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ralf_setup_lora error, %d", status);
    }

    status = ral_set_dio_irq_params(&(modem_radio.ral), RAL_IRQ_TX_DONE);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_dio_irq_params error, %d", status);
    }
    s_test_flrc_flag = true;
    uint8_t quit = 0;
    uint32_t num = 0;
    __g_tx_done_flag = true;
    uint32_t recv_len = 0;
    while(1) {
        recv_len = usb_serial_jtag_read_bytes(&quit, 1,  1 / portTICK_PERIOD_MS);
        if( recv_len > 0) {
            if( quit == 0x3) { //Ctrl+C
                break;
            }
        }

        while(__g_tx_done_flag == false) {
            vTaskDelay(20 / portTICK_PERIOD_MS);
            recv_len = usb_serial_jtag_read_bytes(&quit, 1,  1 / portTICK_PERIOD_MS);
            if( recv_len > 0) {
                if( quit == 0x3) { //Ctrl+C
                    break;
                }
            }
        };

        if( recv_len > 0) {
            if( quit==0x3) { //Ctrl+C
                break;
            }
        }

        __g_tx_done_flag = false;
        ral_set_pkt_payload(&(modem_radio.ral), (uint8_t *)buf, len);
        ral_set_tx(&(modem_radio.ral));
        ESP_LOGI(TAG, "send...%d", num ++);

        if (interval) {
            vTaskDelay( interval / portTICK_PERIOD_MS);
        }

        if( cnt !=0  && num >= cnt) {
            break;
        }
    }
    s_test_flrc_flag = false;
    return 0;
}

static void register_flrc_tx_test(void)
{
    flrc_tx_args.freq =
        arg_int0("f", "freq", "<f>", "Set the radio frequency in Hz, range: 415000000 ~ 940000000 Hz, default: 915000000");
    flrc_tx_args.power =
        arg_int0("p", "power", "", "Set the radio power, LPA range: -17 ~ +14 dB, HPA range: -9 ~ +22 dB, default: 22");
    flrc_tx_args.br_bw =
        arg_int0("b", "br_bw", "", "mod br&bps index, 0:BR_2_600_BW_2_666 1:BR_2_080_BW_2_222 2:BR_1_300_BW_1_333 3:BR_1_040_BW_1_333 4:BR_0_650_BW_0_740 5:BR_0_520_BW_0_571 6:BR_0_325_BW_0_357 7:BR_0_260_BW_0_307, default: 0");
    flrc_tx_args.len =
        arg_int0("l", "len", "", "Set tx data length, default: 200");
    flrc_tx_args.cnt =
        arg_int0("c", "cnt", "", "tx number, default: 100");

    flrc_tx_args.end = arg_end(5);

    const esp_console_cmd_t cmd = {
        .command = "subg_tx_flrc",
        .help = "Subg tx flrc test",
        .hint = NULL,
        .func = &flrc_tx_test,
        .argtable = &flrc_tx_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static struct {
    struct arg_int *freq;
    struct arg_int *br_bw;
    struct arg_end *end;
} flrc_rx_args;



static int flrc_rx_test(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &flrc_tx_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, flrc_tx_args.end, argv[0]);
        return 1;
    }

    int      power  = 22;
    uint32_t freq   = 915000000;
    int      br_bw  = 0;

    if (flrc_tx_args.freq->count) {
        freq = flrc_tx_args.freq->ival[0];
    }
    if (flrc_tx_args.br_bw->count) {
        br_bw = flrc_tx_args.br_bw->ival[0];
        if (br_bw > 7) {
            br_bw = 7;
        }
    }
    
    ESP_LOGI(TAG,  "Start rx flrc");

    uint8_t status = 0;
    ralf_params_flrc_t radio_params_flrc = { 
        .output_pwr_in_dbm  = power,
        .crc_polynomial  = 0x00000000,
        .crc_seed  = 0x00000000,
        .rf_freq_in_hz  = freq,
        .sync_word[0] = &default_syncword_1[0],
        .sync_word[1] = &default_syncword_1[0],
        .sync_word[2] = &default_syncword_1[0],
        .mod_params.raw_bit_rate = 7 - br_bw,
        .mod_params.cr = RAL_FLRC_CR_3_4,
        .mod_params.pulse_shape = RAL_FLRC_PULSE_SHAPE_BT_05,
        .pkt_params.crc_type = RAL_FLRC_CRC_2_BYTES,
        .pkt_params.pld_len_in_bytes = 256,
        .pkt_params.pld_is_fix = false,
        .pkt_params.match_sync_word = RAL_FLRC_RX_MATCH_SYNCWORD_1,
        .pkt_params.tx_syncword = RAL_FLRC_TX_SYNCWORD_1,
        .pkt_params.sync_word_len = RAL_FLRC_SYNCWORD_LENGTH_4_BYTES,
        .pkt_params.preamble_len = RAL_FLRC_PREAMBLE_LENGTH_32_BITS,
        .is_tx = false
    };

    ESP_LOGI(TAG,  "Power          = %d dB", power);
    ESP_LOGI(TAG,  "Frequency      = %u Hz", freq);
    ESP_LOGI(TAG,  "pulse_shape    = %d ", radio_params_flrc.mod_params.pulse_shape);
    ESP_LOGI(TAG,  "cr             = %u ", radio_params_flrc.mod_params.cr);
    ESP_LOGI(TAG,  "crc_type       = %u ", radio_params_flrc.pkt_params.crc_type);
    ESP_LOGI(TAG,  "pld_is_fix       = %s ", radio_params_flrc.pkt_params.pld_is_fix == false ? "false":"true");    
    ESP_LOGI(TAG,  "sync_word_len  = %u ", radio_params_flrc.pkt_params.sync_word_len);
    ESP_LOGI(TAG,  "tx_syncword    = %u ", radio_params_flrc.pkt_params.tx_syncword);
    ESP_LOGI(TAG,  "preamble_len   = %u ", radio_params_flrc.pkt_params.preamble_len);
    ESP_LOGI(TAG,  "Start rx flrc");

    status = ralf_setup_flrc(&modem_radio, &radio_params_flrc);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ralf_setup_lora error, %d", status);
    }

    status = ral_set_dio_irq_params(&(modem_radio.ral), RAL_IRQ_RX_DONE|RAL_IRQ_RX_TIMEOUT|RAL_IRQ_RX_HDR_ERROR|RAL_IRQ_RX_CRC_ERROR);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_dio_irq_params error, %d", status);
    }

    status = ral_clear_irq_status(&(modem_radio.ral), RAL_IRQ_ALL);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_clear_irq_status error, %d", status);
    }  

    status = ral_set_rx(&(modem_radio.ral), 0);
    if(status != RAL_STATUS_OK) {
        ESP_LOGE(TAG, "ral_set_rx error, %d", status);
    }
    s_test_flrc_flag = true;
    uint8_t quit = 0;
    int len = 0;
    __g_rx_ok_cnt = 0;
    __g_rx_hdr_err_cnt = 0;
    __g_rx_crc_err_cnt = 0;
    __g_rx_done_cnt = 0;
    while(1) {
        len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
        if( len > 0) {
            if(quit == 0x3) { //Ctrl+C
                break;
            }
        }
    }
    s_test_flrc_flag = false;
    status = ral_set_standby(&(modem_radio.ral), RAL_STANDBY_CFG_RC);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_standby error, %d", status);
    }

    printf("\r\n");
    ESP_LOGW(TAG, "rx_done %d", __g_rx_done_cnt);
    ESP_LOGW(TAG, "rx_ok_cnt %d", __g_rx_ok_cnt);
    ESP_LOGW(TAG, "rx_hdr_err %d", __g_rx_hdr_err_cnt);
    ESP_LOGW(TAG, "rx_crc_err %d\r\n", __g_rx_crc_err_cnt);
    
    ESP_LOGI(TAG, "End of receive!");

    return 0;
}

static void register_flrc_rx_test(void)
{
    flrc_rx_args.freq =
        arg_int0("f", "freq", "<f>", "Set the radio frequency in Hz, range: 415000000 ~ 940000000 Hz, default: 915000000");
    flrc_rx_args.br_bw =
        arg_int0("b", "br_bw", "", "mod br&bps index, 0:BR_2_600_BW_2_666 1:BR_2_080_BW_2_222 2:BR_1_300_BW_1_333 3:BR_1_040_BW_1_333 4:BR_0_650_BW_0_740 5:BR_0_520_BW_0_571 6:BR_0_325_BW_0_357 7:BR_0_260_BW_0_307, default: 0");

    flrc_rx_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "subg_rx_flrc",
        .help = "Subg rx flrc test",
        .hint = NULL,
        .func = &flrc_rx_test,
        .argtable = &flrc_rx_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


static struct {
    struct arg_int *freq;
    struct arg_int *power;
    struct arg_int *ocp;
    struct arg_end *end;
} lora_tx_cw_args;



static int lora_cw_test(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &lora_tx_cw_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, lora_tx_cw_args.end, argv[0]);
        return 1;
    }

    int      power  = 10;
    uint32_t freq   = 868000000;
    int      ocp    = 0;

    if (lora_tx_cw_args.power->count) {
        power = lora_tx_cw_args.power->ival[0];
    }
    if (lora_tx_cw_args.freq->count) {
        freq = lora_tx_cw_args.freq->ival[0];
    }
    if (lora_tx_cw_args.ocp->count) {
        ocp = lora_tx_cw_args.ocp->ival[0];
        if (ocp > 63) {
            ocp = 63;
        }
    }

    ESP_LOGI(TAG,  "Power          = %d dB", power);
    ESP_LOGI(TAG,  "Frequency      = %u Hz", freq);

    if (ocp > 0) {
        ESP_LOGI(TAG,  "OCP            = %.1f mA", (float)(ocp) * 2.5);
    }
    
    ESP_LOGI(TAG,  "Start Tx Continuous Wave");

    uint8_t status = 0;
    ralf_params_lora_t tx_lora_param = { 
        .sync_word                       = 0x12,
        .symb_nb_timeout                 = 0,
        .rf_freq_in_hz                   = freq,
        .output_pwr_in_dbm               = power,
        .mod_params.cr                   = RAL_LORA_CR_4_5,
        .mod_params.sf                   = RAL_LORA_SF7,
        .mod_params.bw                   = RAL_LORA_BW_125_KHZ,
        .mod_params.ldro                 = false,
        .pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT,
        .pkt_params.pld_len_in_bytes     = 4,
        .pkt_params.crc_is_on            = true,
        .pkt_params.invert_iq_is_on      = false,
        .pkt_params.preamble_len_in_symb = 8 };
    
    status = ralf_setup_lora(&modem_radio, &tx_lora_param);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ralf_setup_lora error, %d", status);
    }

    status = ral_set_tx_cw(&(modem_radio.ral));
    if(status != RAL_STATUS_OK) {
        ESP_LOGE(TAG, "ral_set_tx_cw error, %d", status);
    }
    int len = 0;
    uint8_t quit = 0;
    while(1) 
    {
        len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
        if( len > 0) {
            if(quit == 0x3) { //Ctrl+C
                break;
            }
        }
    }

    status = ral_set_standby(&(modem_radio.ral), RAL_STANDBY_CFG_RC);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_standby error, %d", status);
    }

    return 0;
}

static void register_lora_cw_test(void)
{
    lora_tx_cw_args.freq =
        arg_int0("f", "freq", "<f>", "Set the radio frequency in Hz, range: 415000000 ~ 940000000 Hz, default: 868000000");
    lora_tx_cw_args.power =
        arg_int0("p", "power", "", "Set the radio power, LPA range: -17 ~ +14 dB, HPA range: -9 ~ +22 dB, default: 10");
    lora_tx_cw_args.ocp =
        arg_int0("o", "ocp", "", "Set the radio power, step by 2.5mA, range: 0 ~ 63, default: 24");

    lora_tx_cw_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "lora_cw",
        .help = "Tx Continuous Wave",
        .hint = NULL,
        .func = &lora_cw_test,
        .argtable = &lora_tx_cw_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static struct {
    struct arg_int *freq;
    struct arg_int *sf;
    struct arg_int *bw;
    struct arg_int *cr;
    struct arg_int *power;
    struct arg_int *crc;
    struct arg_int *iq;
    struct arg_int *public;
    struct arg_int *interval;
    struct arg_str *txt;
    struct arg_int *cnt;
    struct arg_int *half_power;
    struct arg_int *duty_cycle;
    struct arg_int *lf_slices;    
    struct arg_end *end;
} lora_tx_args;

static int lora_tx_test(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &lora_tx_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, lora_tx_args.end, argv[0]);
        return 1;
    }

    int      power  = 10;
    uint32_t freq   = 868000000;
    uint32_t sf     = RAL_LORA_SF7;
    uint32_t bw     = RAL_LORA_BW_125_KHZ;
    uint32_t cr     = RAL_LORA_CR_4_5;
    uint32_t crc    = 1;
    uint32_t iq     = 0;
    uint32_t public = 1;
    uint32_t interval = 0;
    uint32_t cnt      = 10;
    int half_power = 0;
    uint32_t duty_cycle = 0;
    uint32_t lf_slices = 0;
    
    bool is_config_pa = false;

    char buf[255];
    uint32_t len = 20;

    strcpy( buf, "hello");
    len= strlen("hello");

    if (lora_tx_args.power->count) {
        power = lora_tx_args.power->ival[0];
    }
    if (lora_tx_args.freq->count) {
        freq = lora_tx_args.freq->ival[0];
    }
    if (lora_tx_args.sf->count) {
        sf = lora_tx_args.sf->ival[0];
        if(sf < 5 || sf > 12)
        {
            sf = 7;
        }
    }
    if (lora_tx_args.bw->count) {
        uint32_t bw_temp = lora_tx_args.bw->ival[0];
        if(bw_temp > 0x09)
        {
            bw_temp = 0x09;
        }
        bw = bw_temp + RAL_LORA_BW_031_KHZ;
    }
    if (lora_tx_args.cr->count) {
        cr = lora_tx_args.cr->ival[0];
    }
    if (lora_tx_args.crc->count) {
        crc = lora_tx_args.crc->ival[0];
    }
    if (lora_tx_args.iq->count) {
        iq = lora_tx_args.iq->ival[0];
    }
    if (lora_tx_args.public->count) {
        public = lora_tx_args.public->ival[0];
    }
    if (lora_tx_args.interval->count) {
        interval = lora_tx_args.interval->ival[0];
    }
    if (lora_tx_args.cnt->count) {
        cnt = lora_tx_args.cnt->ival[0];
    }
    if (lora_tx_args.txt->count) {
        strncpy( buf, lora_tx_args.txt->sval[0],  sizeof(buf));
        len = strlen(lora_tx_args.txt->sval[0]);
    }

    // PA Test
    if (lora_tx_args.half_power->count) {
        half_power = lora_tx_args.half_power->ival[0];
        is_config_pa = true;
    }
    if (lora_tx_args.duty_cycle->count) {
        duty_cycle = lora_tx_args.duty_cycle->ival[0];
        is_config_pa = true;
    }
    if (lora_tx_args.lf_slices->count) {
        lf_slices = lora_tx_args.lf_slices->ival[0];
        is_config_pa = true;
    }

    uint8_t status = 0;
    uint8_t sync_word = 0x34;
    uint8_t is_ldo = 0;

    if (public) {
        sync_word = 0x34;
    }
    else
    {
        sync_word = 0x12;        
    }

    if (bw == RAL_LORA_BW_125_KHZ && sf >= 11) {
        is_ldo = 1;
    } else if (bw == RAL_LORA_BW_250_KHZ && sf == 12) {
        is_ldo = 1;
    }
 
    ralf_params_lora_t tx_lora_param = { 
        .sync_word                       = sync_word,
        .symb_nb_timeout                 = 0,
        .rf_freq_in_hz                   = freq,
        .output_pwr_in_dbm               = power,
        .mod_params.cr                   = cr,
        .mod_params.sf                   = sf,
        .mod_params.bw                   = bw,
        .mod_params.ldro                 = is_ldo,
        .pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT,
        .pkt_params.pld_len_in_bytes     = len,
        .pkt_params.crc_is_on            = crc,
        .pkt_params.invert_iq_is_on      = iq,
        .pkt_params.preamble_len_in_symb = 8 
    };

    ESP_LOGI(TAG,  "Power          = %d dB", power);
    ESP_LOGI(TAG,  "Frequency      = %u Hz", freq);
    ESP_LOGI(TAG,  "SF             = %s", ral_lora_sf_to_str(sf));
    ESP_LOGI(TAG,  "BandWidth      = %s", ral_lora_bw_to_str(bw));
    ESP_LOGI(TAG,  "CodingRate     = %s", ral_lora_cr_to_str(cr));
    ESP_LOGI(TAG,  "CRC            = %d", crc);
    ESP_LOGI(TAG,  "IQ             = %d", iq);
    ESP_LOGI(TAG,  "SyncWord       = 0x%x", sync_word);
    ESP_LOGI(TAG,  "LDO            = %d", is_ldo);
    ESP_LOGI(TAG,  "Interval       = %d", interval);
    ESP_LOGI(TAG,  "Num            = %d", cnt);
    ESP_LOGI(TAG,  "half_power     = %d", half_power);
    ESP_LOGI(TAG,  "duty_cycle     = %d", duty_cycle);
    ESP_LOGI(TAG,  "lf_slices      = %d", lf_slices);
    ESP_LOGI(TAG,  "is_config_pa      = %d", is_config_pa);
    ESP_LOGI(TAG,  "tx len %d, payload: %s", len, buf);  

    status = ralf_setup_lora(&modem_radio, &tx_lora_param);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ralf_setup_lora error, %d", status);
    }
    if(is_config_pa) {
        status = usr_lr20xx_set_tx_cfg( &(modem_radio.ral), power, freq, duty_cycle, lf_slices, duty_cycle, half_power);
        if(status != RAL_STATUS_OK)
        {
            ESP_LOGE(TAG, "usr_lr20xx_set_tx_cfg error, %d", status);
        }
    }

    status = ral_set_dio_irq_params(&(modem_radio.ral), RAL_IRQ_TX_DONE);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_dio_irq_params error, %d", status);
    }

    uint8_t quit = 0;
    int recv_len = 0;
    uint32_t num = 0;
    __g_tx_done_flag = true;
    while(1) {
        
        recv_len = usb_serial_jtag_read_bytes(&quit, 1,  1 / portTICK_PERIOD_MS);
        if( recv_len > 0) {
            if( quit == 0x3) { //Ctrl+C
                break;
            }
        }

        while(__g_tx_done_flag == false) {
            vTaskDelay(20 / portTICK_PERIOD_MS);
            recv_len = usb_serial_jtag_read_bytes(&quit, 1,  1 / portTICK_PERIOD_MS);
            if( recv_len > 0) {
                if( quit == 0x3) { //Ctrl+C
                    break;
                }
            }
        };

        if( recv_len > 0) {
            if( quit==0x3) { //Ctrl+C
                break;
            }
        }

        __g_tx_done_flag = false;
        ral_set_pkt_payload(&(modem_radio.ral), (uint8_t *)buf, len);
        ral_set_tx(&(modem_radio.ral));
        ESP_LOGI(TAG, "send...%d", num ++);
        if (interval) {
            vTaskDelay( interval / portTICK_PERIOD_MS);
        }

        if( cnt !=0  && num >= cnt) {
            break;
        }
    }

    // wait last tx done
    while(__g_tx_done_flag == false) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        recv_len = usb_serial_jtag_read_bytes(&quit, 1,  1 / portTICK_PERIOD_MS);
        if( recv_len > 0) {
            if( quit==0x3) { //Ctrl+C
                break;
            }
        }
    };
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    status = ral_set_standby(&(modem_radio.ral), RAL_STANDBY_CFG_RC);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_standby error, %d", status);
    }

    ESP_LOGI(TAG, "End of send!");

    return 0;
}

static void register_lora_tx_test(void)
{
    lora_tx_args.freq =
        arg_int0("f", "freq", "<f>", "Set the radio frequency in Hz, range: 150000000 ~ 2500000000 Hz, default: 868000000");
    lora_tx_args.sf =
        arg_int0("s", "sf", "<6~12>", "Set Lora SF, range: 5 ~ 12, default: 7");
    lora_tx_args.bw =
        arg_int0("b", "bw", "<0~9>", "Set Lora Bandwidth, 0:31kHz  1:41KHz  2:62KHz  3:125KHz, 4:200kHz  5:250KHz  6:400KHz  7:500KHz  8:800KHz  9:1000KHz, default: 3");
    lora_tx_args.cr =
        arg_int0("c", "cr", "<1~7>", "Set Lora CodingRate, 1:CR_4_5  2:CR_4_6  3:CR_4_7  4:CR_4_8  5:CR_LI_4_5  6:CR_LI_4_6  7:CR_LI_4_8, default: 1");
    lora_tx_args.power =
        arg_int0("p", "power", "", "Set the radio power, LF range: -10 ~ +22 dB, HF range: -17 ~ +12 dB, default: 10");
    lora_tx_args.crc =
        arg_int0(NULL, "crc", "<0|1>", "Set Lora CRC, 0:DISABLE  1:ENABLE, default: 1");
    lora_tx_args.iq =
        arg_int0(NULL, "iq", "<0|1>", "Set Lora IQ mode, 0:STANDARD  1:INVERTED, default: 0");
    lora_tx_args.public =
        arg_int0(NULL, "net", "<0|1>", "Set Public Network, 0: Private Network(0x12), 1: Public Network(0x34), default: 1");
    lora_tx_args.interval =
        arg_int0("i", "interval", "<t>", "Set the tx interval ms, default: 0");
    lora_tx_args.txt =
        arg_str0("d", "txt", "<d>", "Set the text data to send, default: hello");
    lora_tx_args.cnt =
        arg_int0("n", "num", "<n>", "Set Number of packets sent, 0: Keep sending, default: 1");

    lora_tx_args.half_power =
        arg_int0("hp", "half_power", "", "Set the radio PA half power,0.5dB steps, range: -19 ~ 44 (LF) -39 ~ 24(HF) (see datasheet Table 7-15,7-16,7-17,7-18,7-20), default: 0");
    lora_tx_args.duty_cycle =
        arg_int0("dc", "duty_cycle", "", "Set the radio PA duty_cycle, range: 0 ~ 31, default: 0");
    lora_tx_args.lf_slices =
        arg_int0("lf", "lf_slices", "", "Set the radio PA lf_slices, range: 0 ~ 15, default: 0");


    lora_tx_args.end = arg_end(14);

    const esp_console_cmd_t cmd = {
        .command = "lora_tx",
        .help = "lora tx data",
        .hint = NULL,
        .func = &lora_tx_test,
        .argtable = &lora_tx_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

#define FHSS_125K_MODE  0
#define FHSS_500K_MODE  1

#define BW125K_FREQ_INTERVAL        200000
#define BW500K_FREQ_INTERVAL_1      1600000
#define BW500K_FREQ_INTERVAL_2      600000

#define CH1_FREQ                    902300000
#define CH65_FREQ                   903000000
#define CH73_FREQ                   923300000

static struct {
    struct arg_int *mode;
    struct arg_int *sf;
    struct arg_int *cr;
    struct arg_int *power;
    struct arg_int *crc;
    struct arg_int *iq;
    struct arg_int *public;
    struct arg_int *interval;
    struct arg_str *txt;
    struct arg_end *end;
} lora_tx_fhss_args;

static uint32_t rand_freq[64] = {0};

static void rand_generate(uint8_t* buf, uint8_t count, uint8_t fhss_mode)
{
    uint8_t randTmp = 0;

    for (int i = 0; i < count; i++) {
LOOP:
        if(fhss_mode == FHSS_125K_MODE)
        {
            randTmp = (uint8_t) (esp_random() % 64 + 1) ;
        }
        else
        {
            randTmp = (uint8_t)  (esp_random() % 16 + 65); // (65, 80)
        }

        for (int j = i; j >= 0; j--)
        {
            if (buf[j] == randTmp)
            {
                goto LOOP;
            }
        }

        buf[i] = randTmp;
        //printf("%d ", randTmp);
    }
}
static int freq_buf_generate(uint8_t  fhss_mode)
{
    uint8_t  rand_buff[64] = {0};
    uint32_t freq = 0;
    int freq_index = 0;
    int rand_index = 0;
    if( fhss_mode == FHSS_125K_MODE) {
        rand_generate(rand_buff, 64, fhss_mode);
        
        for( rand_index = 0; rand_index < 64; rand_index++) {
            freq = CH1_FREQ + (rand_buff[rand_index] - 1) * BW125K_FREQ_INTERVAL;
            rand_freq[freq_index++] = freq;
        }
        return freq_index;

    } else {
        rand_generate(rand_buff, 16, fhss_mode);

        for( rand_index = 0; rand_index < 16; rand_index++) {

            if(rand_buff[rand_index] < 73)
            {
                freq = CH65_FREQ + (rand_buff[rand_index] - 65 ) * BW500K_FREQ_INTERVAL_1;
            }
            else
            {
                freq = CH73_FREQ + (rand_buff[rand_index] - 73 ) * BW500K_FREQ_INTERVAL_2;
            }
            rand_freq[freq_index++] = freq;
        }
        return freq_index;
    }
}

static int lora_fcc_fhss_tx_test(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &lora_tx_fhss_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, lora_tx_fhss_args.end, argv[0]);
        return 1;
    }

    int      power  = 14;
    uint32_t freq   = CH1_FREQ;
    uint32_t sf     = 10;
    uint32_t mode   = FHSS_125K_MODE;
    uint32_t bw     = 0;
    uint32_t cr     = 1;
    uint32_t crc    = 1;
    uint32_t iq     = 0;
    uint32_t public = 1;
    uint32_t interval = 0;
    uint32_t cnt = 0;
    char buf[255];
    uint32_t len = 4;

    strcpy( buf, "hello seeed! 1234567");
    len= strlen("hello seeed! 1234567");

    if (lora_tx_fhss_args.power->count) {
        power = lora_tx_fhss_args.power->ival[0];
    }

    if (lora_tx_fhss_args.mode->count) {
        mode = lora_tx_fhss_args.mode->ival[0];
    }
    if (lora_tx_fhss_args.sf->count) {
        sf = lora_tx_fhss_args.sf->ival[0];
    }
    if (lora_tx_fhss_args.cr->count) {
        cr = lora_tx_fhss_args.cr->ival[0];
    }
    if (lora_tx_fhss_args.crc->count) {
        crc = lora_tx_fhss_args.crc->ival[0];
    }
    if (lora_tx_fhss_args.iq->count) {
        iq = lora_tx_fhss_args.iq->ival[0];
    }
    if (lora_tx_fhss_args.public->count) {
        public = lora_tx_fhss_args.public->ival[0];
    }
    if (lora_tx_fhss_args.interval->count) {
        interval = lora_tx_fhss_args.interval->ival[0];
    }
    if (lora_tx_fhss_args.txt->count) {
        strncpy( buf, lora_tx_fhss_args.txt->sval[0],  sizeof(buf));
        len = strlen(lora_tx_fhss_args.txt->sval[0]);
    }

    if( mode == FHSS_125K_MODE) {
        bw = 0;
    } else {
        bw = 2;
    }
    memset(rand_freq, 0, sizeof(rand_freq));
    cnt = freq_buf_generate(mode);

    ESP_LOGI(TAG,  "Frequency  count: %d", cnt);
    for(int i = 0; i < cnt; i++) {
        printf("%d ", rand_freq[i]);
    }
    printf("Hz\r\n");
    ESP_LOGI(TAG,  "Power          = %d dB", power);
    ESP_LOGI(TAG,  "SF             = %d", sf);
    ESP_LOGI(TAG,  "BandWidth      = %d", bw);
    ESP_LOGI(TAG,  "CodingRate     = %d", cr);
    ESP_LOGI(TAG,  "CRC            = %d", crc);
    ESP_LOGI(TAG,  "IQ             = %d", iq);
    ESP_LOGI(TAG,  "Public         = %d", public);
    ESP_LOGI(TAG,  "Interval       = %d", interval);
    ESP_LOGI(TAG,  "Num            = %d", cnt);
    ESP_LOGI(TAG,  "tx len %d, payload: %s", len, buf);
    
    uint8_t status = 0;
    uint8_t sync_word = 0x12;

    if (public) {
        sync_word = 0x34;
    }

    ralf_params_lora_t tx_lora_param = { 
        .sync_word                       = sync_word,
        .symb_nb_timeout                 = 0,
        .rf_freq_in_hz                   = freq,
        .output_pwr_in_dbm               = power,
        .mod_params.cr                   = cr,
        .mod_params.sf                   = sf,
        .mod_params.bw                   = RAL_LORA_BW_125_KHZ + bw,
        .mod_params.ldro                 = false,
        .pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT,
        .pkt_params.pld_len_in_bytes     = len,
        .pkt_params.crc_is_on            = crc,
        .pkt_params.invert_iq_is_on      = iq,
        .pkt_params.preamble_len_in_symb = 8 
    };
    
    status = ralf_setup_lora(&modem_radio, &tx_lora_param);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ralf_setup_lora error, %d", status);
    }

    status = ral_set_dio_irq_params(&(modem_radio.ral), RAL_IRQ_TX_DONE);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_dio_irq_params error, %d", status);
    }

    uint8_t quit = 0;
    int recv_len = 0;
    // uint32_t num = 0;
    __g_tx_done_flag = true;
    int index = 0;

    while(1) {
        recv_len = usb_serial_jtag_read_bytes(&quit, 1,  1 / portTICK_PERIOD_MS);
        if( recv_len > 0) {
            if(quit == 0x3) { //Ctrl+C
                break;
            }
        }

        while(__g_tx_done_flag == false) {
            vTaskDelay(20 / portTICK_PERIOD_MS);
            recv_len = usb_serial_jtag_read_bytes(&quit, 1,  1 / portTICK_PERIOD_MS);
            if( recv_len > 0) {
                if( quit==0x3) { //Ctrl+C
                    break;
                }
            }
        };

        if( recv_len > 0) {
            if(quit == 0x3) { //Ctrl+C
                break;
            }
        }

        if( index >= cnt) {
            break;
        }

        __g_tx_done_flag = false;
        freq = rand_freq[index];
        
        ESP_LOGI(TAG, "freq:%d, send...%d", freq, index);

        ral_set_rf_freq(&(modem_radio.ral), freq);
        ral_set_pkt_payload(&(modem_radio.ral), (uint8_t *)buf, len);
        ral_set_tx(&(modem_radio.ral));

        if (interval) {
            vTaskDelay(interval / portTICK_PERIOD_MS);
        }
        index++;
    }

    status = ral_set_standby(&(modem_radio.ral), RAL_STANDBY_CFG_RC);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_standby error, %d", status);
    }

    ESP_LOGI(TAG, "End of send!");

    return 0;
}

static void register_lora_fcc_fhss_test(void)
{
    lora_tx_fhss_args.mode =
        arg_int0("m", "mode", "<0|1>", "Set the fhss mode, 0: FHSS_125K_MODE, 1: FHSS_500K_MODE, default: 0");
    lora_tx_fhss_args.sf =
        arg_int0("s", "sf", "<6~12>", "Set Lora SF, range: 6 ~ 12, default: 10");
    lora_tx_fhss_args.cr =
        arg_int0("c", "cr", "<1|2|3|4>", "Set Lora CodingRate, 1:CR_4_5  2:CR_4_6  3:CR_4_7  4:CR_4_8 , default: 1");
    lora_tx_fhss_args.power =
        arg_int0("p", "power", "", "Set the radio power, LPA range: -17 ~ +14 dB, HPA range: -9 ~ +22 dB, default: 14");
    lora_tx_fhss_args.crc =
        arg_int0(NULL, "crc", "<0|1>", "Set Lora CRC, 0:DISABLE  1:ENABLE, default: 1");
    lora_tx_fhss_args.iq =
        arg_int0(NULL, "iq", "<0|1>", "Set Lora IQ mode, 0:STANDARD  1:INVERTED, default: 0");
    lora_tx_fhss_args.public =
        arg_int0(NULL, "net", "<0|1>", "Set Public Network, 0: Private Network, 1: Public Network, default: 0");
    lora_tx_fhss_args.interval =
        arg_int0("i", "interval", "<t>", "Set the tx interval ms, default: 0");
    lora_tx_fhss_args.txt =
        arg_str0("d", "txt", "<d>", "Set the text data to send, default: hello seeed! 1234567");
    lora_tx_fhss_args.end = arg_end(10);

    const esp_console_cmd_t cmd = {
        .command = "lora_fcc_fhss",
        .help = "lora fcc fhss tx test",
        .hint = NULL,
        .func = &lora_fcc_fhss_tx_test,
        .argtable = &lora_tx_fhss_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static struct {
    struct arg_int *freq;
    struct arg_int *sf;
    struct arg_int *bw;
    struct arg_int *cr;
    struct arg_int *crc;
    struct arg_int *iq;
    struct arg_int *public;
    struct arg_int *boosted;
    struct arg_end *end;
} lora_rx_args;


static int lora_rx_test(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &lora_rx_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, lora_rx_args.end, argv[0]);
        return 1;
    }

    uint32_t freq   = 868000000;
    uint32_t sf     = RAL_LORA_SF7;
    uint32_t bw     = RAL_LORA_BW_125_KHZ;
    uint32_t cr     = RAL_LORA_CR_4_5;
    uint32_t crc    = 1;
    uint32_t iq     = 0;
    uint32_t public = 1;
    // uint32_t boosted= 1;
    
    if (lora_rx_args.freq->count) {
        freq = lora_rx_args.freq->ival[0];
    }
    if (lora_rx_args.sf->count) {
        sf = lora_rx_args.sf->ival[0];
        if(sf > 12)
        {
            sf = 12;
        }
        else if(sf < 5)
        {
            sf = 5;
        }
        else{}
    }
    if (lora_rx_args.bw->count) {
        uint32_t bw_temp = lora_rx_args.bw->ival[0];
        if(bw_temp > 0x09)
        {
            bw_temp = 0x09;
        }
        bw = bw_temp + RAL_LORA_BW_031_KHZ;
    }
    if (lora_rx_args.cr->count) {
        cr = lora_rx_args.cr->ival[0];
    }
    if (lora_rx_args.crc->count) {
        crc = lora_rx_args.crc->ival[0];
    }
    if (lora_rx_args.iq->count) {
        iq = lora_rx_args.iq->ival[0];
    }
    if (lora_rx_args.public->count) {
        public = lora_rx_args.public->ival[0];
    }

    uint8_t status = 0;
    uint8_t sync_word = 0x34;
    uint8_t is_ldo = 0;

    if (public) 
    {
        sync_word = 0x34;
    }
    else
    {
       sync_word = 0x12;     
    }

    if (bw == RAL_LORA_BW_125_KHZ && sf >= 11) {
        is_ldo = 1;
    } else if (bw == RAL_LORA_BW_250_KHZ && sf == 12) {
        is_ldo = 1;
    }

    
    ralf_params_lora_t rx_lora_param = { 
        .sync_word                       = sync_word,
        .symb_nb_timeout                 = 0,
        .rf_freq_in_hz                   = freq,
        .mod_params.cr                   = cr,
        .mod_params.sf                   = sf,
        .mod_params.bw                   = bw,
        .mod_params.ldro                 = is_ldo,
        .pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT,
        .pkt_params.pld_len_in_bytes     = 255,
        .pkt_params.crc_is_on            = crc,
        .pkt_params.invert_iq_is_on      = iq,
        .pkt_params.preamble_len_in_symb = 8 
    };
    
    status = ralf_setup_lora(&modem_radio, &rx_lora_param);
    if(status != RAL_STATUS_OK) {
        ESP_LOGE(TAG, "ralf_setup_lora error, %d", status);
    }

    status = ral_set_dio_irq_params(&(modem_radio.ral), RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR | RAL_IRQ_RX_CRC_ERROR);
    if(status != RAL_STATUS_OK) {
        ESP_LOGE(TAG, "ral_set_dio_irq_params error, %d", status);
    }

    status = ral_clear_irq_status(&(modem_radio.ral), RAL_IRQ_ALL);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_clear_irq_status error, %d", status);
    }  

    ESP_LOGI(TAG,  "Frequency      = %u Hz", freq);
    ESP_LOGI(TAG,  "SF             = %s", ral_lora_sf_to_str(sf));
    ESP_LOGI(TAG,  "BandWidth      = %s", ral_lora_bw_to_str(bw));
    ESP_LOGI(TAG,  "CodingRate     = %s", ral_lora_cr_to_str(cr));
    ESP_LOGI(TAG,  "Header_type    = %s", ral_lora_pkt_len_modes_to_str(RAL_LORA_PKT_EXPLICIT));
    ESP_LOGI(TAG,  "CRC            = %d", crc);
    ESP_LOGI(TAG,  "IQ             = %d", iq);
    ESP_LOGI(TAG,  "SyncWord       = 0x%x", sync_word);
    ESP_LOGI(TAG,  "LDO            = %d", is_ldo);

    status = ral_set_rx(&(modem_radio.ral), 0);
    if(status != RAL_STATUS_OK) {
        ESP_LOGE(TAG, "ral_set_rx error, %d", status);
    }

    uint8_t quit = 0;
    int len = 0;

    __g_rx_ok_cnt = 0;
    __g_rx_hdr_err_cnt = 0;
    __g_rx_crc_err_cnt = 0;
    __g_rx_done_cnt = 0;
    while(1) {
        len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
        if( len > 0) {
            if(quit == 0x3) { //Ctrl+C
                break;
            }
        }
    }

    status = ral_set_standby(&(modem_radio.ral), RAL_STANDBY_CFG_RC);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_standby error, %d", status);
    }

    printf("\r\n");
    ESP_LOGW(TAG, "rx_done %d", __g_rx_done_cnt);
    ESP_LOGW(TAG, "rx_ok_cnt %d", __g_rx_ok_cnt);
    ESP_LOGW(TAG, "rx_hdr_err %d", __g_rx_hdr_err_cnt);
    ESP_LOGW(TAG, "rx_crc_err %d\r\n", __g_rx_crc_err_cnt);
    ESP_LOGI(TAG, "End of receive!");

    return 0;
}

static void register_lora_rx_test(void)
{
    lora_rx_args.freq =
        arg_int0("f", "freq", "<f>", "Set the radio frequency in Hz, range: 150000000 ~ 2500000000 Hz, default: 868000000");
    lora_rx_args.sf =
        arg_int0("s", "sf", "<5~12>", "Set Lora SF, range: 5 ~ 12, default: 7");
    lora_rx_args.bw =
        arg_int0("b", "bw", "<0~9>", "Set Lora Bandwidth, 0:31kHz  1:41KHz  2:62KHz  3:125KHz, 4:200kHz  5:250KHz  6:400KHz  7:500KHz  8:800KHz  9:1000KHz, default: 3");
    lora_rx_args.cr =
        arg_int0("c", "cr", "<1|2|3|4>", "Set Lora CodingRate, 1:CR_4_5  2:CR_4_6  3:CR_4_7  4:CR_4_8 , default: 1");
    lora_rx_args.crc =
        arg_int0(NULL, "crc", "<0|1>", "Set Lora CRC, 0:DISABLE  1:ENABLE, default: 1");
    lora_rx_args.iq =
        arg_int0(NULL, "iq", "<0|1>", "Set Lora IQ mode, 0:STANDARD  1:INVERTED, default: 0");
    lora_rx_args.public =
        arg_int0(NULL, "net", "<0|1>", "Set Public Network, 0: Private Network(0x12), 1: Public Network(0x34), default: 1");
    lora_rx_args.boosted =
        arg_int0(NULL, "boosted", "<0|1>", "1: Boosted RX, 0: Normal RX, default: 1");

    lora_rx_args.end = arg_end(8);

    const esp_console_cmd_t cmd = {
        .command = "lora_rx",
        .help = "lora rx data",
        .hint = NULL,
        .func = &lora_rx_test,
        .argtable = &lora_rx_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


static struct {
    struct arg_int *power;
    struct arg_int *freq;
    struct arg_int *bw;
    struct arg_int *cr;
    struct arg_int *grid;
    struct arg_int *hop;
    struct arg_int *interval;
    struct arg_str *txt;
    struct arg_int *cnt;
    struct arg_end *end;
} lr_fhss_tx_args;

static ralf_params_lr_fhss_t lr_fhss_param;
static uint8_t lr_fhss_state[RAL_LR_FHSS_STATE_MAXSIZE];
static const uint8_t lr_fhss_sync_word[4] = {0x2C, 0x0F, 0x79, 0x95};

static int lr_fhss_tx_test(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &lr_fhss_tx_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, lr_fhss_tx_args.end, argv[0]);
        return 1;
    }

    int power = 10;
    uint32_t freq = 868000000;
    uint32_t bw = LR_FHSS_V1_BW_1574219_HZ;
    uint32_t cr = LR_FHSS_V1_CR_5_6;
    uint32_t grid = LR_FHSS_V1_GRID_25391_HZ;
    uint32_t hop = true;
    uint32_t interval = 0;
    uint32_t cnt = 1;
    char buf[255];
    uint32_t len = 4;

    strcpy( buf, "hello");
    len= strlen("hello");

    if (lr_fhss_tx_args.power->count) {
        power = lr_fhss_tx_args.power->ival[0];
    }
    if (lr_fhss_tx_args.freq->count) {
        freq = lr_fhss_tx_args.freq->ival[0];
    }
    if (lr_fhss_tx_args.bw->count) {
        bw = lr_fhss_tx_args.bw->ival[0];
    }
    if (lr_fhss_tx_args.cr->count) {
        cr = lr_fhss_tx_args.cr->ival[0];
    }
    if (lr_fhss_tx_args.grid->count) {
        grid = lr_fhss_tx_args.grid->ival[0];
    }
    if (lr_fhss_tx_args.hop->count) {
        hop = lr_fhss_tx_args.hop->ival[0];
    }
    if (lr_fhss_tx_args.interval->count) {
        interval = lr_fhss_tx_args.interval->ival[0];
    }
    if (lr_fhss_tx_args.cnt->count) {
        cnt = lr_fhss_tx_args.cnt->ival[0];
    }
    if (lr_fhss_tx_args.txt->count) {
        strncpy( buf, lr_fhss_tx_args.txt->sval[0],  sizeof(buf));
        len = strlen(lr_fhss_tx_args.txt->sval[0]);
    }

    ESP_LOGI(TAG,  "Power          = %d dB", power);
    ESP_LOGI(TAG,  "Frequency      = %u Hz", freq);
    ESP_LOGI(TAG,  "BandWidth      = %d", bw);
    ESP_LOGI(TAG,  "CodingRate     = %d", cr);
    ESP_LOGI(TAG,  "Grid           = %d", grid);
    ESP_LOGI(TAG,  "Hop            = %d", hop);
    ESP_LOGI(TAG,  "Interval       = %d", interval);
    ESP_LOGI(TAG,  "Num            = %d", cnt);
    ESP_LOGI(TAG,  "tx len %d, payload: %s", len, buf);   

    uint8_t quit = 0;
    int recv_len = 0;
    uint32_t num = 0;
    ral_status_t status = 0;

    while(1) {
        ESP_LOGI(TAG, "send...%d", num ++);

        __g_tx_done_flag = false;

        // lr_fhss init on here
        memset(lr_fhss_state, 0, sizeof(lr_fhss_state));
        memset(&lr_fhss_param, 0, sizeof(ralf_params_lr_fhss_t));

        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.modulation_type = LR_FHSS_V1_MODULATION_TYPE_GMSK_488;
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.cr = cr;
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.grid = grid;
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.enable_hopping = hop;
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.bw = bw;
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.header_count = 2;
        lr_fhss_param.ral_lr_fhss_params.lr_fhss_params.sync_word = lr_fhss_sync_word;
        lr_fhss_param.ral_lr_fhss_params.center_frequency_in_hz = freq;
        lr_fhss_param.ral_lr_fhss_params.device_offset = 0;

        status = ral_lr_fhss_init(&(modem_radio.ral), &lr_fhss_param.ral_lr_fhss_params);
        if(status != RAL_STATUS_OK) {
            ESP_LOGE("TAG", "ral_lr_fhss_init error %d\r\n", status);
            return 0;
        }

        status = ral_set_tx_cfg(&(modem_radio.ral), power, freq);
        if(status != RAL_STATUS_OK) {
            ESP_LOGE("TAG", "ral_set_tx_cfg error %d\r\n",status);
            return 0;
        }

        status = ral_set_dio_irq_params(&(modem_radio.ral), RAL_IRQ_TX_DONE | RAL_IRQ_LR_FHSS_HOP);
        if(status != RAL_STATUS_OK) {
            ESP_LOGE("TAG", "ral_set_dio_irq_params error, %d", status);
            return 0;
        }

        unsigned int nb_max_hop_sequence = 0;
        status = ral_lr_fhss_get_hop_sequence_count(&(modem_radio.ral), &lr_fhss_param.ral_lr_fhss_params, &nb_max_hop_sequence);
        if(status != RAL_STATUS_OK) {
            ESP_LOGE("TAG", "ral_lr_fhss_get_hop_sequence_count error %d", status);
            return 0;
        }

        const uint16_t hop_sequence_id = esp_random( ) % nb_max_hop_sequence;
        status = ral_lr_fhss_build_frame(&(modem_radio.ral), &lr_fhss_param.ral_lr_fhss_params, lr_fhss_state, hop_sequence_id, (uint8_t *)buf, len);
        if(status != RAL_STATUS_OK) {
            ESP_LOGE("TAG", "ral_lr_fhss_build_frame error %d\r\n", status);
        }

        status = ral_set_tx(&(modem_radio.ral));
        if(status != RAL_STATUS_OK) {
            ESP_LOGE("TAG", "ral_set_tx error %d\r\n", status);
            return 0;
        }

        recv_len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
        if(recv_len > 0) {
            if(quit == 0x3) { //Ctrl+C
                break;
            }
        }

        while(__g_tx_done_flag == false) {
            vTaskDelay(20 / portTICK_PERIOD_MS);
            recv_len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
            if(recv_len > 0) {
                if(quit == 0x3) { //Ctrl+C
                    break;
                }
            }
        };

        ral_lr_fhss_handle_tx_done(&(modem_radio.ral), &lr_fhss_param.ral_lr_fhss_params, lr_fhss_state);

        if(recv_len > 0) {
            if(quit==0x3) { //Ctrl+C
                break;
            }
        }

        if (interval) {
            vTaskDelay(interval / portTICK_PERIOD_MS);
        }

        if(cnt !=0  && num >= cnt) {
            break;
        }
    }

    // wait last tx done
    while(__g_tx_done_flag == false) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        recv_len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
        if( recv_len > 0) {
            if( quit==0x3) { //Ctrl+C
                break;
            }
        }
    };
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    status = ral_set_standby(&(modem_radio.ral), RAL_STANDBY_CFG_RC);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_standby error, %d", status);
    }

    ESP_LOGI(TAG, "End of send!");

    return 0;
}

static void register_lr_fhss_tx_test(void)
{
    lr_fhss_tx_args.freq =
        arg_int0("f", "freq", "<f>", "Set the radio frequency in Hz, range: 415000000 ~ 940000000 Hz, default: 868000000");
    lr_fhss_tx_args.bw =
        arg_int0("b", "bw", "<0-9>", "Set LR_FHSS Bandwidth, 0:39063_HZ  1:85938_HZ  2:136719_HZ, 3:183594_HZ, 4:335938_HZ, 5:386719_HZ, 6:722656_HZ, 7:773438_HZ, 8:1523438_HZ, 9:1574219_HZ, default: 9");
    lr_fhss_tx_args.cr =
        arg_int0("c", "cr", "<0-3>", "Set LR_FHSS CodingRate, 0:CR_5_6, 1:CR_2_3, 2:CR_1_2, 3:CR_1_3, default: 0");
    lr_fhss_tx_args.grid =
        arg_int0("g", "grid", "<0|1>", "Set LR_FHSS Grid, 0:25391_HZ, 1:3906_HZ, default: 0");
    lr_fhss_tx_args.hop =
        arg_int0("h", "hop", "<0|1>", "Set LR_FHSS Hop, 0:disable, 1:enable, default: 1");
    lr_fhss_tx_args.power =
        arg_int0("p", "power", "", "Set the radio power, LPA range: -17 ~ +14 dB, HPA range: -9 ~ +22 dB, default: 10");
    lr_fhss_tx_args.interval =
        arg_int0("i", "interval", "<t>", "Set the tx interval ms, default: 0");
    lr_fhss_tx_args.txt =
        arg_str0("d", "txt", "<d>", "Set the text data to send, default: hello");
    lr_fhss_tx_args.cnt =
        arg_int0("n", "num", "<n>", "Set Number of packets sent, 0: Keep sending, default: 1");
    lr_fhss_tx_args.end = arg_end(9);

    const esp_console_cmd_t cmd = {
        .command = "lr_fhss_tx",
        .help = "lr_fhss tx data",
        .hint = NULL,
        .func = &lr_fhss_tx_test,
        .argtable = &lr_fhss_tx_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static struct {
    struct arg_int *power;
    struct arg_int *freq;
    struct arg_int *bitrate;
    struct arg_int *fdev_in_hz;
    struct arg_int *bw_dsb_in_hz;
    struct arg_int *interval;
    struct arg_str *txt;
    struct arg_int *cnt;
    struct arg_end *end;
} gfsk_tx_args;

static int gfsk_tx_test(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &gfsk_tx_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, gfsk_tx_args.end, argv[0]);
        return 1;
    }

    int power = 10;
    uint32_t freq = 868000000;
    uint32_t bitrate = 50000;
    uint32_t fdev_in_hz = 25000;
    uint32_t bw_dsb_in_hz = 100000;
    uint32_t interval = 0;
    uint32_t cnt = 1;
    char buf[255];
    uint32_t len = 4;

    strcpy(buf, "hello");
    len= strlen("hello");

    if (gfsk_tx_args.power->count) {
        power = gfsk_tx_args.power->ival[0];
    }
    if (gfsk_tx_args.freq->count) {
        freq = gfsk_tx_args.freq->ival[0];
    }
    if (gfsk_tx_args.bitrate->count) {
        bitrate = gfsk_tx_args.bitrate->ival[0];
    }
    if (gfsk_tx_args.fdev_in_hz->count) {
        fdev_in_hz = gfsk_tx_args.fdev_in_hz->ival[0];
    }
    if (gfsk_tx_args.bw_dsb_in_hz->count) {
        bw_dsb_in_hz = gfsk_tx_args.bw_dsb_in_hz->ival[0];
    }
    if (gfsk_tx_args.interval->count) {
        interval = gfsk_tx_args.interval->ival[0];
    }
    if (gfsk_tx_args.cnt->count) {
        cnt = gfsk_tx_args.cnt->ival[0];
    }
    if (gfsk_tx_args.txt->count) {
        strncpy( buf, gfsk_tx_args.txt->sval[0], sizeof(buf));
        len = strlen(gfsk_tx_args.txt->sval[0]);
    }

    ESP_LOGI(TAG,  "Power          = %d dB", power);
    ESP_LOGI(TAG,  "Frequency      = %u Hz", freq);
    ESP_LOGI(TAG,  "bitrate        = %u", bitrate);
    ESP_LOGI(TAG,  "fdev_in_hz     = %u", fdev_in_hz);
    ESP_LOGI(TAG,  "bw_dsb_in_hz   = %d", bw_dsb_in_hz);
    ESP_LOGI(TAG,  "Interval       = %d", interval);
    ESP_LOGI(TAG,  "Num            = %d", cnt);
    ESP_LOGI(TAG,  "tx len %d, payload: %s", len, buf);

    ral_status_t status = 0;
    ralf_params_gfsk_t radio_params_gfsk = {0};
    const uint8_t sync_word[4] = { 0x2C, 0x0F, 0x79, 0x95 };
    
    radio_params_gfsk.rf_freq_in_hz = freq;
    radio_params_gfsk.output_pwr_in_dbm = power;
    radio_params_gfsk.sync_word = sync_word;
    radio_params_gfsk.mod_params.br_in_bps = bitrate;
    radio_params_gfsk.mod_params.fdev_in_hz = fdev_in_hz;
    radio_params_gfsk.mod_params.bw_dsb_in_hz = bw_dsb_in_hz;
    radio_params_gfsk.mod_params.pulse_shape = RAL_GFSK_PULSE_SHAPE_OFF;
    radio_params_gfsk.pkt_params.preamble_len_in_bits = 40;
    radio_params_gfsk.pkt_params.preamble_detector = 0;
    radio_params_gfsk.pkt_params.sync_word_len_in_bits = 24;
    radio_params_gfsk.pkt_params.address_filtering = 0;
    radio_params_gfsk.pkt_params.header_type = RAL_GFSK_PKT_VAR_LEN;
    radio_params_gfsk.pkt_params.pld_len_in_bytes = len;
    radio_params_gfsk.pkt_params.crc_type = RAL_GFSK_CRC_OFF;
    radio_params_gfsk.pkt_params.dc_free = RAL_GFSK_DC_FREE_OFF;

    status = ralf_setup_gfsk(&modem_radio, &radio_params_gfsk);
    if(status != RAL_STATUS_OK) {
        ESP_LOGE("TAG", "ralf_setup_gfsk error, %d", status);
    }

    status = ral_set_dio_irq_params(&(modem_radio.ral), RAL_IRQ_TX_DONE);
    if(status != RAL_STATUS_OK) {
        ESP_LOGE("TAG", "ral_set_dio_irq_params error, %d", status);
    }

    uint8_t quit = 0;
    int recv_len = 0;
    uint32_t num = 0;
    __g_tx_done_flag = true;
    while(1) {
        recv_len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
        if(recv_len > 0) {
            if(quit == 0x3) { //Ctrl+C
                break;
            }
        }

        while(__g_tx_done_flag == false) {
            vTaskDelay(20 / portTICK_PERIOD_MS);
            recv_len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
            if(recv_len > 0) {
                if(quit == 0x3) { //Ctrl+C
                    break;
                }
            }
        };

        if(recv_len > 0) {
            if(quit==0x3) { //Ctrl+C
                break;
            }
        }

        __g_tx_done_flag = false;
        ral_set_pkt_payload(&(modem_radio.ral), (uint8_t *)buf, len);
        ral_set_tx(&(modem_radio.ral));
        ESP_LOGI(TAG, "send...%d", num ++);
        if (interval) {
            vTaskDelay(interval / portTICK_PERIOD_MS);
        }

        if(cnt !=0  && num >= cnt) {
            break;
        }
    }

    // wait last tx done
    while(__g_tx_done_flag == false) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        recv_len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
        if( recv_len > 0) {
            if( quit==0x3) { //Ctrl+C
                break;
            }
        }
    };
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    status = ral_set_standby(&(modem_radio.ral), RAL_STANDBY_CFG_RC);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_standby error, %d", status);
    }

    ESP_LOGI(TAG, "End of send!");

    return 0;
}

static void register_gfsk_tx_test(void)
{
    gfsk_tx_args.freq =
        arg_int0("f", "freq", "<f>", "Set the radio frequency in Hz, range: 415000000 ~ 940000000 Hz, default: 868000000");
    gfsk_tx_args.power =
        arg_int0("p", "power", "", "Set the radio power, LPA range: -17 ~ +14 dB, HPA range: -9 ~ +22 dB, default: 10");
    gfsk_tx_args.bitrate =
        arg_int0(NULL, "br", "", "Set gfsk bitrate, default: 50000");
    gfsk_tx_args.fdev_in_hz =
        arg_int0(NULL, "fdev", "", "Set gfsk fdev Hz, default: 25000");
    gfsk_tx_args.bw_dsb_in_hz =
        arg_int0(NULL, "bw", "", "Set gfsk bw dsb, default: 100000");
    gfsk_tx_args.interval =
        arg_int0("i", "interval", "<t>", "Set the tx interval ms, default: 0");
    gfsk_tx_args.txt =
        arg_str0("d", "txt", "<d>", "Set the text data to send, default: hello");
    gfsk_tx_args.cnt =
        arg_int0("n", "num", "<n>", "Set Number of packets sent, 0: Keep sending, default: 1");
    gfsk_tx_args.end = arg_end(8);

    const esp_console_cmd_t cmd = {
        .command = "gfsk_tx",
        .help = "gfsk tx data",
        .hint = NULL,
        .func = &gfsk_tx_test,
        .argtable = &gfsk_tx_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

#define CH1_FREQ_2              902500000
#define BW500K_FREQ_INTERVAL    800000
static uint32_t rand_freq_2[32] = {0};

static struct {
    struct arg_int *sf;
    struct arg_int *cr;
    struct arg_int *power;
    struct arg_int *crc;
    struct arg_int *iq;
    struct arg_int *public;
    struct arg_int *interval;
    struct arg_str *txt;
    struct arg_end *end;
} lora_tx_fhss_2_args;

static void rand_generate_2(uint8_t* buf, uint8_t count)
{
    uint8_t randTmp = 0;

    for (int i = 0; i < count; i++) {
LOOP:
        randTmp = (uint8_t) (esp_random() % 31 + 1) ;

        for (int j = i; j >= 0; j--)
        {
            if (buf[j] == randTmp)
            {
                goto LOOP;
            }
        }

        buf[i] = randTmp;
        //printf("%d ", randTmp);
    }
}

static int freq_buf_generate_2(void)
{
    uint8_t  rand_buff[32] = {0};
    uint32_t freq = 0;
    int freq_index = 0;
    int rand_index = 0;
    rand_generate_2(rand_buff, 31);
    for(rand_index = 0; rand_index < 31; rand_index++) {
        freq = CH1_FREQ_2 + (rand_buff[rand_index] - 1) * BW500K_FREQ_INTERVAL;
        rand_freq_2[freq_index++] = freq;
    }
    return freq_index;
}

static int lora_fcc_fhss_2_tx_test(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &lora_tx_fhss_2_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, lora_tx_fhss_2_args.end, argv[0]);
        return 1;
    }

    int      power  = 14;
    uint32_t freq   = CH1_FREQ;
    uint32_t sf     = 10;
    uint32_t cr     = 1;
    uint32_t crc    = 1;
    uint32_t iq     = 0;
    uint32_t public = 1;
    uint32_t interval = 0;
    uint32_t cnt = 0;
    char buf[255];
    uint32_t len = 4;

    strcpy( buf, "hello seeed! 1234567");
    len= strlen("hello seeed! 1234567");

    if (lora_tx_fhss_2_args.power->count) {
        power = lora_tx_fhss_2_args.power->ival[0];
    }
    if (lora_tx_fhss_2_args.sf->count) {
        sf = lora_tx_fhss_2_args.sf->ival[0];
    }
    if (lora_tx_fhss_2_args.cr->count) {
        cr = lora_tx_fhss_2_args.cr->ival[0];
    }
    if (lora_tx_fhss_2_args.crc->count) {
        crc = lora_tx_fhss_2_args.crc->ival[0];
    }
    if (lora_tx_fhss_2_args.iq->count) {
        iq = lora_tx_fhss_2_args.iq->ival[0];
    }
    if (lora_tx_fhss_2_args.public->count) {
        public = lora_tx_fhss_2_args.public->ival[0];
    }
    if (lora_tx_fhss_2_args.interval->count) {
        interval = lora_tx_fhss_2_args.interval->ival[0];
    }
    if (lora_tx_fhss_2_args.txt->count) {
        strncpy( buf, lora_tx_fhss_2_args.txt->sval[0],  sizeof(buf));
        len = strlen(lora_tx_fhss_2_args.txt->sval[0]);
    }

    memset(rand_freq_2, 0, sizeof(rand_freq_2));
    cnt = freq_buf_generate_2();

    ESP_LOGI(TAG,  "Frequency  count: %d", cnt);
    for(int i = 0; i < cnt; i++) {
        printf("%d ", rand_freq_2[i]);
    }
    printf("Hz\r\n");
    ESP_LOGI(TAG,  "Power          = %d dB", power);
    ESP_LOGI(TAG,  "SF             = %d", sf);
    ESP_LOGI(TAG,  "CodingRate     = %d", cr);
    ESP_LOGI(TAG,  "CRC            = %d", crc);
    ESP_LOGI(TAG,  "IQ             = %d", iq);
    ESP_LOGI(TAG,  "Public         = %d", public);
    ESP_LOGI(TAG,  "Interval       = %d", interval);
    ESP_LOGI(TAG,  "Num            = %d", cnt);
    ESP_LOGI(TAG,  "tx len %d, payload: %s", len, buf);
    
    uint8_t status = 0;
    uint8_t sync_word = 0x12;

    if (public) {
        sync_word = 0x34;
    }

    ralf_params_lora_t tx_lora_param = { 
        .sync_word                       = sync_word,
        .symb_nb_timeout                 = 0,
        .rf_freq_in_hz                   = freq,
        .output_pwr_in_dbm               = power,
        .mod_params.cr                   = cr,
        .mod_params.sf                   = sf,
        .mod_params.bw                   = RAL_LORA_BW_500_KHZ,
        .mod_params.ldro                 = false,
        .pkt_params.header_type          = RAL_LORA_PKT_EXPLICIT,
        .pkt_params.pld_len_in_bytes     = len,
        .pkt_params.crc_is_on            = crc,
        .pkt_params.invert_iq_is_on      = iq,
        .pkt_params.preamble_len_in_symb = 8 
    };
    
    status = ralf_setup_lora(&modem_radio, &tx_lora_param);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ralf_setup_lora error, %d", status);
    }

    status = ral_set_dio_irq_params(&(modem_radio.ral), RAL_IRQ_TX_DONE);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_dio_irq_params error, %d", status);
    }

    uint8_t quit = 0;
    int recv_len = 0;
    __g_tx_done_flag = true;
    int index = 0;

    while(1) {
        recv_len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
        if( recv_len > 0) {
            if(quit == 0x3) { //Ctrl+C
                break;
            }
        }

        while(__g_tx_done_flag == false) {
            vTaskDelay(20 / portTICK_PERIOD_MS);
            recv_len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
            if( recv_len > 0) {
                if( quit==0x3) { //Ctrl+C
                    break;
                }
            }
        };

        if( recv_len > 0) {
            if(quit == 0x3) { //Ctrl+C
                break;
            }
        }

        if( index >= cnt) {
            break;
        }

        __g_tx_done_flag = false;
        freq = rand_freq_2[index];
        
        ESP_LOGI(TAG, "freq:%d, send...%d", freq, index);

        ral_set_rf_freq(&(modem_radio.ral), freq);
        ral_set_pkt_payload(&(modem_radio.ral), (uint8_t *)buf, len);
        ral_set_tx(&(modem_radio.ral));

        if (interval) {
            vTaskDelay(interval / portTICK_PERIOD_MS);
        }
        index++;
    }

    status = ral_set_standby(&(modem_radio.ral), RAL_STANDBY_CFG_RC);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_standby error, %d", status);
    }

    ESP_LOGI(TAG, "End of send!");

    return 0;
}

static void register_lora_fcc_fhss_2_test(void)
{
    lora_tx_fhss_2_args.sf =
        arg_int0("s", "sf", "<6~12>", "Set Lora SF, range: 6 ~ 12, default: 10");
    lora_tx_fhss_2_args.cr =
        arg_int0("c", "cr", "<1|2|3|4>", "Set Lora CodingRate, 1:CR_4_5  2:CR_4_6  3:CR_4_7  4:CR_4_8 , default: 1");
    lora_tx_fhss_2_args.power =
        arg_int0("p", "power", "", "Set the radio power, LPA range: -17 ~ +14 dB, HPA range: -9 ~ +22 dB, default: 14");
    lora_tx_fhss_2_args.crc =
        arg_int0(NULL, "crc", "<0|1>", "Set Lora CRC, 0:DISABLE  1:ENABLE, default: 1");
    lora_tx_fhss_2_args.iq =
        arg_int0(NULL, "iq", "<0|1>", "Set Lora IQ mode, 0:STANDARD  1:INVERTED, default: 0");
    lora_tx_fhss_2_args.public =
        arg_int0(NULL, "net", "<0|1>", "Set Public Network, 0: Private Network, 1: Public Network, default: 0");
    lora_tx_fhss_2_args.interval =
        arg_int0("i", "interval", "<t>", "Set the tx interval ms, default: 0");
    lora_tx_fhss_2_args.txt =
        arg_str0("d", "txt", "<d>", "Set the text data to send, default: hello seeed! 1234567");
    lora_tx_fhss_2_args.end = arg_end(10);

    const esp_console_cmd_t cmd = {
        .command = "lora_fcc_fhss_2",
        .help = "lora fcc fhss 2 tx test",
        .hint = NULL,
        .func = &lora_fcc_fhss_2_tx_test,
        .argtable = &lora_tx_fhss_2_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

#define CH1_FREQ_3              902200000
#define BW100K_FREQ_INTERVAL    200000
static uint32_t rand_freq_3[132] = {0};

static void rand_generate_3(uint8_t* buf, uint8_t count)
{
    uint8_t randTmp = 0;

    for (int i = 0; i < count; i++) {
LOOP:
        randTmp = (uint8_t) (esp_random() % 129 + 1) ;

        for (int j = i; j >= 0; j--)
        {
            if (buf[j] == randTmp)
            {
                goto LOOP;
            }
        }

        buf[i] = randTmp;
        //printf("%d ", randTmp);
    }
}

static int freq_buf_generate_3(void)
{
    uint8_t  rand_buff[132] = {0};
    uint32_t freq = 0;
    int freq_index = 0;
    int rand_index = 0;
    rand_generate_3(rand_buff, 129);
    for(rand_index = 0; rand_index < 129; rand_index++) {
        freq = CH1_FREQ_3 + (rand_buff[rand_index] - 1) * BW100K_FREQ_INTERVAL;
        rand_freq_3[freq_index++] = freq;
    }
    return freq_index;
}

static int lora_fcc_fhss_3_tx_test(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &gfsk_tx_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, gfsk_tx_args.end, argv[0]);
        return 1;
    }

    int power = 10;
    uint32_t freq = 868000000;
    uint32_t bitrate = 50000;
    uint32_t fdev_in_hz = 25000;
    uint32_t bw_dsb_in_hz = 100000;
    uint32_t interval = 0;
    uint32_t cnt = 1;
    char buf[255];
    uint32_t len = 4;

    strcpy( buf, "hello");
    len= strlen("hello");

    if (gfsk_tx_args.power->count) {
        power = gfsk_tx_args.power->ival[0];
    }
    if (gfsk_tx_args.freq->count) {
        freq = gfsk_tx_args.freq->ival[0];
    }
    if (gfsk_tx_args.bitrate->count) {
        bitrate = gfsk_tx_args.bitrate->ival[0];
    }
    if (gfsk_tx_args.fdev_in_hz->count) {
        fdev_in_hz = gfsk_tx_args.fdev_in_hz->ival[0];
    }
    if (gfsk_tx_args.bw_dsb_in_hz->count) {
        bw_dsb_in_hz = gfsk_tx_args.bw_dsb_in_hz->ival[0];
    }
    if (gfsk_tx_args.interval->count) {
        interval = gfsk_tx_args.interval->ival[0];
    }
    if (gfsk_tx_args.cnt->count) {
        cnt = gfsk_tx_args.cnt->ival[0];
    }
    if (gfsk_tx_args.txt->count) {
        strncpy( buf, gfsk_tx_args.txt->sval[0],  sizeof(buf));
        len = strlen(gfsk_tx_args.txt->sval[0]);
    }

    ESP_LOGI(TAG,  "Power          = %d dB", power);
    ESP_LOGI(TAG,  "Frequency      = %u Hz", freq);
    ESP_LOGI(TAG,  "bitrate        = %u", bitrate);
    ESP_LOGI(TAG,  "fdev_in_hz     = %u", fdev_in_hz);
    ESP_LOGI(TAG,  "bw_dsb_in_hz   = %d", bw_dsb_in_hz);
    ESP_LOGI(TAG,  "Interval       = %d", interval);
    ESP_LOGI(TAG,  "Num            = %d", cnt);
    ESP_LOGI(TAG,  "tx len %d, payload: %s", len, buf);

    memset(rand_freq_3, 0, sizeof(rand_freq_3));
    cnt = freq_buf_generate_3();

    ESP_LOGI(TAG,  "Frequency  count: %d", cnt);
    for(int i = 0; i < cnt; i++) {
        printf("%d ", rand_freq_3[i]);
    }
    printf("Hz\r\n");

    ral_status_t status = 0;
    ralf_params_gfsk_t radio_params_gfsk = {0};
    const uint8_t sync_word[4] = { 0x2C, 0x0F, 0x79, 0x95 };

    radio_params_gfsk.rf_freq_in_hz = freq;
    radio_params_gfsk.output_pwr_in_dbm = power;
    radio_params_gfsk.sync_word = sync_word;
    radio_params_gfsk.mod_params.br_in_bps = bitrate;
    radio_params_gfsk.mod_params.fdev_in_hz = fdev_in_hz;
    radio_params_gfsk.mod_params.bw_dsb_in_hz = bw_dsb_in_hz;
    radio_params_gfsk.mod_params.pulse_shape = RAL_GFSK_PULSE_SHAPE_OFF;
    radio_params_gfsk.pkt_params.preamble_len_in_bits = 40;
    radio_params_gfsk.pkt_params.preamble_detector = 0;
    radio_params_gfsk.pkt_params.sync_word_len_in_bits = 24;
    radio_params_gfsk.pkt_params.address_filtering = 0;
    radio_params_gfsk.pkt_params.header_type = RAL_GFSK_PKT_VAR_LEN;
    radio_params_gfsk.pkt_params.pld_len_in_bytes = len;
    radio_params_gfsk.pkt_params.crc_type = RAL_GFSK_CRC_OFF;
    radio_params_gfsk.pkt_params.dc_free = RAL_GFSK_DC_FREE_OFF;
    
    status = ralf_setup_gfsk(&modem_radio, &radio_params_gfsk);
    if(status != RAL_STATUS_OK) {
        ESP_LOGE("TAG", "ralf_setup_gfsk error, %d", status);
    }

    status = ral_set_dio_irq_params(&(modem_radio.ral), RAL_IRQ_TX_DONE);
    if(status != RAL_STATUS_OK) {
        ESP_LOGE("TAG", "ral_set_dio_irq_params error, %d", status);
    }

    uint8_t quit = 0;
    int recv_len = 0;
    __g_tx_done_flag = true;
    int index = 0;

    while(1) {
        recv_len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
        if( recv_len > 0) {
            if(quit == 0x3) { //Ctrl+C
                break;
            }
        }

        while(__g_tx_done_flag == false) {
            vTaskDelay(20 / portTICK_PERIOD_MS);
            recv_len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
            if( recv_len > 0) {
                if( quit==0x3) { //Ctrl+C
                    break;
                }
            }
        };

        if( recv_len > 0) {
            if(quit == 0x3) { //Ctrl+C
                break;
            }
        }

        if( index >= cnt) {
            break;
        }

        __g_tx_done_flag = false;
        freq = rand_freq_3[index];
        
        ESP_LOGI(TAG, "freq:%d, send...%d", freq, index);

        ral_set_rf_freq(&(modem_radio.ral), freq);
        ral_set_pkt_payload(&(modem_radio.ral), (uint8_t *)buf, len);
        ral_set_tx(&(modem_radio.ral));

        if (interval) {
            vTaskDelay(interval / portTICK_PERIOD_MS);
        }
        index++;
    }

    // wait last tx done
    while(__g_tx_done_flag == false) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        recv_len = usb_serial_jtag_read_bytes(&quit, 1,  10 / portTICK_PERIOD_MS);
        if( recv_len > 0) {
            if( quit==0x3) { //Ctrl+C
                break;
            }
        }
    };
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    status = ral_set_standby(&(modem_radio.ral), RAL_STANDBY_CFG_RC);
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGE(TAG, "ral_set_standby error, %d", status);
    }

    ESP_LOGI(TAG, "End of send!");

    return 0;
}

static void register_gfsk_fcc_fhss_3_test(void)
{
    gfsk_tx_args.freq =
        arg_int0("f", "freq", "<f>", "Set the radio frequency in Hz, range: 415000000 ~ 940000000 Hz, default: 868000000");
    gfsk_tx_args.power =
        arg_int0("p", "power", "", "Set the radio power, LPA range: -17 ~ +14 dB, HPA range: -9 ~ +22 dB, default: 10");
    gfsk_tx_args.bitrate =
        arg_int0(NULL, "br", "", "Set gfsk bitrate, default: 50000");
    gfsk_tx_args.fdev_in_hz =
        arg_int0(NULL, "fdev", "", "Set gfsk fdev Hz, default: 25000");
    gfsk_tx_args.bw_dsb_in_hz =
        arg_int0(NULL, "bw", "", "Set gfsk bw dsb, default: 100000");
    gfsk_tx_args.interval =
        arg_int0("i", "interval", "<t>", "Set the tx interval ms, default: 0");
    gfsk_tx_args.txt =
        arg_str0("d", "txt", "<d>", "Set the text data to send, default: hello");
    gfsk_tx_args.cnt =
        arg_int0("n", "num", "<n>", "Set Number of packets sent, 0: Keep sending, default: 1");
    gfsk_tx_args.end = arg_end(8);

    const esp_console_cmd_t cmd = {
        .command = "gfsk_fcc_fhss_3",
        .help = "gfsk fcc fhss tx test",
        .hint = NULL,
        .func = &lora_fcc_fhss_3_tx_test,
        .argtable = &gfsk_tx_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


static TaskHandle_t radio_dio_task_handle;

static void radio_dio_isr_handler(void* arg)
{
    if (radio_dio_task_handle) {
        xTaskNotifyGive( radio_dio_task_handle );
    }
}

esp_err_t radio_dio_init(void)
{
    esp_err_t ret = ESP_OK;

    if (s_radio_dio_isr_ready) {
        return ESP_OK;
    }

    const gpio_config_t int_gpio_config = {
        .pull_up_en = 0,
        .pull_down_en = 1,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = 1ULL << CONFIG_RADIO_INT_GPIO,
    };
    ret = gpio_config(&int_gpio_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_isr_handler_add(CONFIG_RADIO_INT_GPIO, radio_dio_isr_handler, (void *)CONFIG_RADIO_INT_GPIO);
    if (ret != ESP_OK) {
        return ret;
    }

    s_radio_dio_isr_ready = true;
    return ret;
}

void radio_lr20xx_irq_task(void* arg)
{
    radio_dio_init(); // 
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ral_irq_t irq_regs;
        ral_get_irq_status(&(modem_radio.ral), &irq_regs);
        ral_clear_irq_status(&(modem_radio.ral), RAL_IRQ_ALL);
        // printf("\r\n");
        // ESP_LOGI(TAG, "irq_regs: 0x%x", irq_regs);

        if((irq_regs & RAL_IRQ_TX_DONE) == RAL_IRQ_TX_DONE) {
            ESP_LOGI(TAG, "LoRa tx done\r\n");
            __g_tx_done_flag = true;
        }
        else if((irq_regs & RAL_IRQ_RX_DONE) == RAL_IRQ_RX_DONE) {
            // ESP_LOGI(TAG, "LoRa rx done\r\n");

            uint16_t rx_len = 0;
            uint16_t rx_buf_max = sizeof(__g_rx_buf);
            ral_get_pkt_payload(&(modem_radio.ral), rx_buf_max, __g_rx_buf, &rx_len);

            if(s_test_flrc_flag)
            {
                ral_flrc_rx_pkt_status_t packet_status = { 0 };
                ral_get_flrc_rx_pkt_status(&(modem_radio.ral), &packet_status);
                printf("RX rssi = %d, len:%hu\r\n", packet_status.rssi_sync_in_dbm, rx_len);  
                // for( int i = 0; i < rx_len; i++) {
                //     printf("0x%x ", __g_rx_buf[i] );
                // }
                // printf("\n");              
            }
            else
            {
                ral_lora_rx_pkt_status_t pkt_status = { 0 };
                ral_get_lora_rx_pkt_status(&(modem_radio.ral), &pkt_status);
                
                // ESP_LOGI(TAG, "LoRa rx done, rssi:%d dBm, snr:%d dB, len:%hu", pkt_status.rssi_pkt_in_dbm, pkt_status.snr_pkt_in_db, rx_len);
                printf("LoRa rx done, rssi:%d dBm, snr:%d dB, len:%hu\r\n", pkt_status.rssi_pkt_in_dbm, pkt_status.snr_pkt_in_db, rx_len);
                // for( int i = 0; i < rx_len; i++) {
                //     printf("0x%x ", __g_rx_buf[i] );
                // }
                // printf("\n");
            }
            if(((irq_regs & RAL_IRQ_RX_CRC_ERROR) != RAL_IRQ_RX_CRC_ERROR)&& ((irq_regs & RAL_IRQ_RX_HDR_ERROR) != RAL_IRQ_RX_HDR_ERROR))
            {
                __g_rx_ok_cnt ++;
            }
            else
            {

            }
            __g_rx_done_cnt ++;
            ral_set_rx(&(modem_radio.ral), 0);
        }
        else if((irq_regs & RAL_IRQ_RX_TIMEOUT) == RAL_IRQ_RX_TIMEOUT) {
            // ESP_LOGI(TAG, "LoRa rx timeout\r\n");
            printf("LoRa rx timeout\r\n");
            ral_clear_irq_status(&(modem_radio.ral), RAL_IRQ_ALL);
            ral_set_rx(&(modem_radio.ral), 0);            
        }
        else if((irq_regs & RAL_IRQ_RX_HDR_ERROR) == RAL_IRQ_RX_HDR_ERROR) {
            // ESP_LOGI(TAG, "LoRa hdr error\r\n");
            printf("LoRa hdr error\r\n");
            __g_rx_hdr_err_cnt ++;
            ral_clear_irq_status(&(modem_radio.ral), RAL_IRQ_ALL);
            ral_set_rx(&(modem_radio.ral), 0);
        }
        else if((irq_regs & RAL_IRQ_RX_CRC_ERROR) == RAL_IRQ_RX_CRC_ERROR) {
            // ESP_LOGI(TAG, "LoRa crc error\r\n");
            printf("LoRa crc error\r\n");
            __g_rx_crc_err_cnt ++;
            ral_clear_irq_status(&(modem_radio.ral), RAL_IRQ_ALL);
            ral_set_rx(&(modem_radio.ral), 0);
        }
    }
}

void lora_cmd_register_all(void)
{
    uint8_t status = 0;
    lr20xx_init(&radio);
    ral_reset(&(modem_radio.ral));
    status = ral_init(&(modem_radio.ral));
    if(status != RAL_STATUS_OK)
    {
        ESP_LOGI(TAG, "ral_init error, %d", status);
    }

    register_lora_tx_test();
    register_lora_rx_test();
    register_lora_cw_test();
    register_lora_fcc_fhss_test();
    register_gfsk_tx_test();
    register_lr_fhss_tx_test();
    register_lora_fcc_fhss_2_test();
    register_gfsk_fcc_fhss_3_test();
    register_flrc_tx_test();
    register_flrc_rx_test();
    xTaskCreate(radio_lr20xx_irq_task, "dio", 4096, NULL, 5, &radio_dio_task_handle);
}
