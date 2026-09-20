#include "sdkconfig.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_ldo_regulator.h"
#include "esp_vfs_fat.h"
#include "driver/gpio.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "bsp_gnss_control.h"
#include "driver/uart.h"
#include <string.h>

static bool gnss_uart_init_flag = false;
static char g_GnssUartTxBuffer[GNSS_UART_TX_BUF_SIZE] = {0};

static uint8_t app_nmea_check_sum( char *buf );

/* Wake-from-backup timing. The GNSS module is left in RTC/backup mode by
 * bsp_gnss_scan_stop(); scan_start must wake it via the RTC_INT pulse.
 * These cover the two failure modes that caused "command entered but no
 * GNSS data": giving the core time to boot + bring up the UART before we
 * send keep-awive commands, and verifying (with retry) that NMEA is
 * actually flowing instead of assuming the wake succeeded. */
#define BSP_GNSS_WAKE_PULSE_MS      10      /* RTC_INT high width (was 3ms; I2C expander adds latency) */
#define BSP_GNSS_WAKE_SETTLE_MS     500     /* backup -> full-on boot + UART ready before sending commands */
#define BSP_GNSS_NMEA_WAIT_MS       2500    /* window to detect the first NMEA byte after wake */
#define BSP_GNSS_SCAN_START_RETRY   3       /* re-pulse RTC_INT if no NMEA appears */

void bsp_gnss_set_io_input_pullup(void)
{
    gpio_config_t input_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
    };

    input_cfg.pin_bit_mask = 1ULL << (BSP_GNSS_PPS0);
    gpio_config(&input_cfg);

    input_cfg.pin_bit_mask = 1ULL << (BSP_GNSS_TX);
    gpio_config(&input_cfg);

    input_cfg.pin_bit_mask = 1ULL << (BSP_GNSS_RX);
    gpio_config(&input_cfg);
}

void bsp_gnss_uart_init( uint32_t rx_pin, uint32_t tx_pin, uint32_t baud )
{
    if(gnss_uart_init_flag == false)
    {
        /* Configure parameters of an UART driver,
        * communication pins and install the driver */
        uart_config_t uart_config = {
            .baud_rate = baud,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
            .source_clk = UART_SCLK_DEFAULT,
#endif
        };
        int intr_alloc_flags = 0;

    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif
        ESP_ERROR_CHECK(uart_param_config(GNSS_UART_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(GNSS_UART_NUM, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_ERROR_CHECK(uart_driver_install(GNSS_UART_NUM, GNSS_UART_RX_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));

        gnss_uart_init_flag = true;
    }
}
void bsp_gnss_uart_deinit( )
{
    if(gnss_uart_init_flag)
    {
        ESP_ERROR_CHECK(uart_driver_delete(GNSS_UART_NUM));
        gnss_uart_init_flag = false;
    }
}

int gnss_uart_read_bytes(void *buf, uint32_t length, TickType_t ticks_to_wait)
{
    if(gnss_uart_init_flag)
    {
        return uart_read_bytes(GNSS_UART_NUM,buf, length, ticks_to_wait);
    }
    return 0;
}

bool gnss_uart_is_initialized(void)
{
    return gnss_uart_init_flag;
}

/* Bytes currently held in the driver RX ring buffer. Returns -1 if the driver
 * is not installed (so the caller can distinguish "no driver" from "empty"). */
int gnss_uart_buffered_len(void)
{
    if(!gnss_uart_init_flag)
    {
        return -1;
    }
    size_t buffered = 0;
    if(uart_get_buffered_data_len(GNSS_UART_NUM, &buffered) != ESP_OK)
    {
        return -1;
    }
    return (int)buffered;
}

/* Drain every byte from the RX path: read the driver ring buffer to empty AND
 * flush the hardware RX FIFO. Used at the entry and exit of a parse window so
 * stale data from a previous window (and the long unattended overflow that
 * builds up between windows in debug-idle mode) cannot leave the RX channel in
 * a full/overrun state. Safe to call when the driver is not installed. */
void gnss_uart_drain_rx(void)
{
    if(!gnss_uart_init_flag)
    {
        return;
    }
    size_t buffered = 0;
    uint8_t tmp[128];
    /* Drain the driver ring buffer first; uart_flush_input() below only clears
     * the hardware FIFO, not the ring buffer. */
    while (uart_get_buffered_data_len(GNSS_UART_NUM, &buffered) == ESP_OK &&
           buffered > 0)
    {
        int n = uart_read_bytes(GNSS_UART_NUM, tmp, sizeof(tmp), 0);
        if (n <= 0)
        {
            break;
        }
    }
    /* Then drop anything still in the hardware FIFO and clear any overrun flag. */
    uart_flush_input(GNSS_UART_NUM);
}

int gnss_uart_write_bytes(void *data, uint32_t len)
{
    if(gnss_uart_init_flag)
    {
        return uart_write_bytes(GNSS_UART_NUM,(const char *) data, len);
    }
    return -1;
}

static void gnss_uart_vprint(const char* fmt, va_list argp )
{
    if( 0 < vsnprintf( g_GnssUartTxBuffer, sizeof(g_GnssUartTxBuffer), fmt, argp ) )  // build string
    {
        gnss_uart_write_bytes(g_GnssUartTxBuffer, strlen( g_GnssUartTxBuffer ));
    }
}

void gnss_uart_trace_print( const char* fmt, ... )
{
    va_list argp;
    va_start( argp, fmt );
    gnss_uart_vprint(fmt, argp );
    va_end( argp );
}

void bsp_gnss_power_init( void )
{
    // GNSS
    bsp_exp_io_set_dir(BSP_GNSS_PWR_EN|BSP_GNSS_VRTC_EN|BSP_GNSS_RST|BSP_GNSS_SLEEP_INT|BSP_GNSS_RTC_INT, EXPANDER_IO_DIR_OUTPUT);
    bsp_exp_output_io_set_level( BSP_GNSS_PWR_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    bsp_exp_output_io_set_level( BSP_GNSS_VRTC_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    bsp_exp_output_io_set_level( BSP_GNSS_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    bsp_exp_output_io_set_level( BSP_GNSS_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    bsp_exp_output_io_set_level( BSP_GNSS_SLEEP_INT, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    bsp_exp_output_io_set_level( BSP_GNSS_RTC_INT, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    bsp_gnss_set_io_input_pullup();

}

void bsp_gnss_poweroff( void )
{
    /* power off gnss */
    bsp_exp_output_io_set_level( BSP_GNSS_PWR_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10)); 

    gpio_config_t input_cfg = {
        .mode = GPIO_MODE_INPUT,
    };


    input_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    input_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    input_cfg.pin_bit_mask = 1ULL << (BSP_GNSS_RX);
    gpio_config(&input_cfg);
    input_cfg.pin_bit_mask = 1ULL << (BSP_GNSS_TX);
    gpio_config(&input_cfg);
    input_cfg.pin_bit_mask = (1ULL << BSP_GNSS_PPS0);
    gpio_config(&input_cfg); 

    bsp_exp_output_io_set_level( BSP_GNSS_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    bsp_exp_output_io_set_level( BSP_GNSS_VRTC_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    bsp_exp_io_set_dir(BSP_GNSS_SLEEP_INT|BSP_GNSS_RTC_INT, EXPANDER_IO_DIR_INPUT);

}

void bsp_gnss_reset( void )
{
    bsp_exp_output_io_set_level( BSP_GNSS_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10)); 
    bsp_exp_output_io_set_level( BSP_GNSS_RST, 0);    
}


void bsp_limit_nmea_data( void )
{
    for( uint8_t i = 0; i < 2; i++ )
    {
        gnss_uart_trace_print( "$PAIR062,0,1*3F\r\n" ); // GGA ON
        vTaskDelay(pdMS_TO_TICKS(40));
        gnss_uart_trace_print( "$PAIR062,1,0*3F\r\n" ); // GLL OFF
        vTaskDelay(pdMS_TO_TICKS(40));
        gnss_uart_trace_print( "$PAIR062,2,0*3C\r\n" ); // GSA OFF
        vTaskDelay(pdMS_TO_TICKS(40));
        gnss_uart_trace_print( "$PAIR062,3,0*3D\r\n" ); // GSV OFF
        vTaskDelay(pdMS_TO_TICKS(40));
        gnss_uart_trace_print( "$PAIR062,4,1*3B\r\n" ); // RMC ON
        vTaskDelay(pdMS_TO_TICKS(40));
        gnss_uart_trace_print( "$PAIR062,5,0*3B\r\n" ); // VTG OFF
        vTaskDelay(pdMS_TO_TICKS(40));
        gnss_uart_trace_print( "$PAIR062,6,0*38\r\n" ); // ZDA ON
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    vTaskDelay(pdMS_TO_TICKS(250));
    gnss_uart_trace_print("$PAIR513*3D\r\n"); // save configuration
}



void bsp_gnss_scan_lock_sleep( void )
{
    char command[32] = { 0 };
    uint8_t check_sum = app_nmea_check_sum( "$PAIR382,1" );
    sprintf( command, "$PAIR382,1*%02X\r\n", check_sum );
    for( uint8_t i = 0; i < 3; i++ )
    {
        gnss_uart_trace_print( "%s", command );
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void bsp_gnss_scan_unlock_sleep( void )
{
    char command[32] = { 0 };
    uint8_t check_sum = app_nmea_check_sum( "$PAIR382,0" );
    sprintf( command, "$PAIR382,0*%02X\r\n", check_sum );
    for( uint8_t i = 0; i < 4; i++ )
    {
        gnss_uart_trace_print( "%s", command );
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}


void bsp_gnss_scan_enter_rtc_mode( void )
{
    for( uint8_t i = 0; i < 3; i++ )
    {
        gnss_uart_trace_print( "$PAIR650,0*25\r\n" ); // enter RTC mode
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void bsp_gnss_scan_set_utc( void )
{

}


static bool bsp_gnss_wait_for_nmea(uint32_t timeout_ms)
{
    /* Non-consuming check: poll the RX buffer level so we don't steal the
     * first sentence from the caller's read loop. Returns true as soon as
     * the module is observed transmitting (i.e. wake succeeded). */
    uint32_t steps = (timeout_ms + 19) / 20;
    for (uint32_t i = 0; i < steps; i++) {
        size_t buffered = 0;
        if (uart_get_buffered_data_len(GNSS_UART_NUM, &buffered) == ESP_OK &&
            buffered > 0) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return false;
}

bool bsp_gnss_scan_start_with_baud( uint32_t baud )
{
    /* Init uart (one-shot; guarded by gnss_uart_init_flag) */
    bsp_gnss_uart_init(BSP_UART1_RX, BSP_UART1_TX, baud);
    
    for (uint8_t attempt = 0; attempt < BSP_GNSS_SCAN_START_RETRY; attempt++) {
        /* Drop any stale bytes in the RX FIFO/buffer so the wake detection
         * below reflects only traffic from this wake attempt. */
        uart_flush_input(GNSS_UART_NUM);

        /* Wakeup gnss (power is already kept on to preserve warm start) */
        bsp_exp_output_io_set_level( BSP_GNSS_PWR_EN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));

        bsp_exp_io_set_dir(BSP_GNSS_SLEEP_INT|BSP_GNSS_RTC_INT, EXPANDER_IO_DIR_OUTPUT);

        /* RTC_INT wake pulse. Width bumped from 3ms to 10ms because each
         * level set is an I2C expander transaction; the wider pulse gives
         * reliable recognition of the wake edge. */
        bsp_exp_output_io_set_level( BSP_GNSS_RTC_INT, 1);
        vTaskDelay(pdMS_TO_TICKS(BSP_GNSS_WAKE_PULSE_MS));
        bsp_exp_output_io_set_level( BSP_GNSS_RTC_INT, 0);

        /* Let the module boot its core and bring the UART up before we send
         * keep-alive commands; otherwise $PAIR382,1 can be sent while the
         * module isn't listening yet and it falls back to sleep. */
        vTaskDelay(pdMS_TO_TICKS(BSP_GNSS_WAKE_SETTLE_MS));

        bsp_gnss_scan_lock_sleep( );
        bsp_gnss_scan_set_utc( );
        bsp_limit_nmea_data();
        /* Confirm the module is actually transmitting. If not, the wake
         * didn't take — retry the whole sequence. */
        if (bsp_gnss_wait_for_nmea(BSP_GNSS_NMEA_WAIT_MS)) {
            return true;
        }
    }

    return false;
}

bool bsp_gnss_scan_start( void )
{
    return bsp_gnss_scan_start_with_baud(115200);
}

void bsp_gnss_scan_stop( void )
{
    /* power on gnss */
    bsp_gnss_scan_unlock_sleep( );
    bsp_gnss_scan_enter_rtc_mode( );
    vTaskDelay( 50 );
    bsp_exp_output_io_set_level( BSP_GNSS_PWR_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Deinit uart */
    bsp_gnss_uart_deinit();
}

static uint8_t app_nmea_check_sum( char *buf )
{
    uint8_t i = 0;
    uint8_t chk = 0;
    uint8_t len = strlen( buf );

    for( chk=buf[1], i = 2; i < len; i++ )
    {
        chk ^= buf[i];
    }

    return chk;
}
