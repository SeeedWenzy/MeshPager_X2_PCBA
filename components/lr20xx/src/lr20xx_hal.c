/**
 * @file      lr20xx_hal.c
 *
 * @brief     HAL implementation for LR20xx radio chip.
 *
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stddef.h>

#include "lr20xx_hal.h"

#include "esp_rom_sys.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum
{
    RADIO_SLEEP,
    RADIO_AWAKE
} radio_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static volatile radio_mode_t radio_mode = RADIO_AWAKE;

static uint8_t hal_spi_in_out(const lr20xx_hal_context_t *context, const uint8_t out_data);

static void lr20xx_hal_wait_on_busy(int busy_pin);

static void lr20xx_hal_check_device_ready(const lr20xx_hal_context_t *context);

__attribute__((weak)) void expander_io_lr20xx_reset( void) 
{

}
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
static bool s_lora_spi_init = false;
void lora_spi_init(const void* context)
{
    if(s_lora_spi_init == false)
    {
        lr20xx_hal_context_t *p_lr20xx = (lr20xx_hal_context_t *)context;
        esp_err_t ret = ESP_OK;

        // SPI initialization is handled in lr20xx_init() to ensure it's done after GPIOs are configured
        spi_bus_config_t buscfg = {
            .miso_io_num = CONFIG_RADIO_MISO_GPIO,
            .mosi_io_num = CONFIG_RADIO_MOSI_GPIO,
            .sclk_io_num = CONFIG_RADIO_SCK_GPIO,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 256 * 8,
        };
        ret = spi_bus_initialize(CONFIG_RADIO_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
        ESP_ERROR_CHECK(ret);

        spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = 1000000,
            .spics_io_num = -1,
            .queue_size = 1,
            .command_bits = 0,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = 0
        };
        ESP_ERROR_CHECK(spi_bus_add_device(CONFIG_RADIO_SPI_HOST, &dev_cfg, &p_lr20xx->spi_handle));
        s_lora_spi_init = true;
    }
}
void lora_spi_deinit(const void* context)
{
    if(s_lora_spi_init)
    {
        lr20xx_hal_context_t *p_lr20xx = (lr20xx_hal_context_t *)context;
        // esp_err_t ret = ESP_OK;
        spi_bus_remove_device(p_lr20xx->spi_handle);
        spi_bus_free(CONFIG_RADIO_SPI_HOST);
        s_lora_spi_init = false;
    }
}

void lr20xx_init(const void* context)
{
    lr20xx_hal_context_t *p_lr20xx = (lr20xx_hal_context_t *)context;
    // esp_err_t ret = ESP_OK;

    p_lr20xx->nss = CONFIG_RADIO_CS_GPIO;
    p_lr20xx->busy = CONFIG_RADIO_BUSY_GPIO;
#if !defined(USE_EXPANDER_FOR_RESET_PIN)
    p_lr20xx->reset = CONFIG_RADIO_RST_GPIO;
#else

#endif
    p_lr20xx->dio = CONFIG_RADIO_INT_GPIO;

    gpio_config_t output_cfg = {
        .mode = GPIO_MODE_OUTPUT,
    };
    output_cfg.pin_bit_mask = 1ULL << (p_lr20xx->nss);
    gpio_config(&output_cfg);
    gpio_set_level(p_lr20xx->nss, 1);
#if !defined(USE_EXPANDER_FOR_RESET_PIN)
    output_cfg.pin_bit_mask = 1ULL << (p_lr20xx->reset);
    gpio_config(&output_cfg);
    gpio_set_level(p_lr20xx->reset, 1);
#else

#endif
    // Busy
    gpio_config_t input_cfg = {
        .mode = GPIO_MODE_INPUT,
    };
    input_cfg.pin_bit_mask = 1ULL << (p_lr20xx->busy);
    input_cfg.pull_up_en = 0;
    input_cfg.pull_down_en = 0;
    gpio_config(&input_cfg);

    // DIO
    input_cfg.pin_bit_mask = 1ULL << (p_lr20xx->dio);
    input_cfg.pull_up_en = 0;
    input_cfg.pull_down_en = 1;
    gpio_config(&input_cfg);

    lora_spi_init(p_lr20xx);
    // spi_bus_config_t buscfg = {
    //     .miso_io_num = CONFIG_RADIO_MISO_GPIO,
    //     .mosi_io_num = CONFIG_RADIO_MOSI_GPIO,
    //     .sclk_io_num = CONFIG_RADIO_SCK_GPIO,
    //     .quadwp_io_num = -1,
    //     .quadhd_io_num = -1,
    //     .max_transfer_sz = 256 * 8,
    // };
    // ret = spi_bus_initialize(CONFIG_RADIO_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    // ESP_ERROR_CHECK(ret);

    // spi_device_interface_config_t dev_cfg = {
    //     .clock_speed_hz = 1000000,
    //     .spics_io_num = -1,
    //     .queue_size = 1,
    //     .command_bits = 0,
    //     .address_bits = 0,
    //     .dummy_bits = 0,
    //     .mode = 0
    // };
    // ESP_ERROR_CHECK(spi_bus_add_device(CONFIG_RADIO_SPI_HOST, &dev_cfg, &p_lr20xx->spi_handle));
}

lr20xx_hal_status_t lr20xx_hal_reset( const void* context )
{
#if !defined(USE_EXPANDER_FOR_RESET_PIN)
    lr20xx_hal_context_t *p_lr20xx = (lr20xx_hal_context_t *)context;
    gpio_set_level(p_lr20xx->reset, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(p_lr20xx->reset, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
#else
    expander_io_lr20xx_reset();
#endif
    radio_mode = RADIO_AWAKE;
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_wakeup( const void* context )
{
    lr20xx_hal_context_t *p_lr20xx = (lr20xx_hal_context_t *)context;
    gpio_set_level(p_lr20xx->nss, 0);
    esp_rom_delay_us(1000);
    gpio_set_level(p_lr20xx->nss, 1);
    radio_mode = RADIO_AWAKE;
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_read( const void* context, const uint8_t* cbuffer, const uint16_t cbuffer_length,
                                     uint8_t* rbuffer, const uint16_t rbuffer_length )
{
    lr20xx_hal_context_t *p_lr20xx = (lr20xx_hal_context_t *)context;
    lr20xx_hal_check_device_ready( p_lr20xx );

    uint8_t dummy_bytes[2] = { 0x00, 0x00 };

    // Put NSS low to start spi transaction
    gpio_set_level( p_lr20xx->nss, 0 );

    for(uint16_t i = 0; i < cbuffer_length; i++) 
    {
        hal_spi_in_out(p_lr20xx, cbuffer[i]);
    }

    gpio_set_level( p_lr20xx->nss, 1 );

    if( rbuffer_length > 0 ) 
    {
        lr20xx_hal_wait_on_busy( p_lr20xx->busy);
        gpio_set_level( p_lr20xx->nss, 0 );
        // Send dummy bytes
        for( uint16_t i = 0; i < sizeof( dummy_bytes ); i++ )
        {
            hal_spi_in_out( p_lr20xx, dummy_bytes[i] );
        }
        for(uint16_t i = 0; i < rbuffer_length; i++) 
        {
            rbuffer[i] = hal_spi_in_out(p_lr20xx, LR20XX_NOP);
        }
        gpio_set_level( p_lr20xx->nss, 1 );
    }

    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_write( const void* context, const uint8_t* cbuffer, const uint16_t cbuffer_length,
                                      const uint8_t* cdata, const uint16_t cdata_length )
{
    lr20xx_hal_context_t *p_lr20xx = (lr20xx_hal_context_t *)context;
    lr20xx_hal_check_device_ready( p_lr20xx );

    // Put NSS low to start spi transaction
    gpio_set_level( p_lr20xx->nss, 0 );

    // Send command
    for( uint16_t i = 0; i < cbuffer_length; i++ )
    {
        hal_spi_in_out( p_lr20xx, cbuffer[i] );
    }

    // Send data
    for( uint16_t i = 0; i < cdata_length; i++ )
    {
        hal_spi_in_out( p_lr20xx, cdata[i] );
    }

    // Put NSS high as the spi transaction is finished
    gpio_set_level( p_lr20xx->nss, 1 );

    // Check if command sent is a sleep command LR20XX_SYSTEM_SET_SLEEP_MODE_OC = 0x0127 and save context
    if( ( cbuffer[0] == 0x01 ) && ( cbuffer[1] == 0x27 ) )
    {
        radio_mode = RADIO_SLEEP;
        esp_rom_delay_us(5000);
        lora_spi_deinit(p_lr20xx);
        // add a incompressible delay to prevent trying to wake the radio before it is full asleep
    }
    else
    {
        lr20xx_hal_check_device_ready(p_lr20xx);
    }
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{
    lr20xx_hal_context_t *p_lr20xx = (lr20xx_hal_context_t *)context;
    lr20xx_hal_check_device_ready(p_lr20xx);

    // Put NSS low to start spi transaction
    gpio_set_level( p_lr20xx->nss, 0 );

    for( uint16_t i = 0; i < data_length; i++ )
    {
        data[i] = hal_spi_in_out( p_lr20xx, 0 );
    }

    // Put NSS high as the spi transaction is finished
    gpio_set_level( p_lr20xx->nss, 1 );

    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read_fifo( const void* context, const uint8_t* command,
                                                 const uint16_t command_length, uint8_t* data,
                                                 const uint16_t data_length )
{
    lr20xx_hal_context_t *p_lr20xx = (lr20xx_hal_context_t *)context;
    lr20xx_hal_check_device_ready(p_lr20xx);

    // Put NSS low to start spi transaction
    gpio_set_level( p_lr20xx->nss, 0 );

    // Send command
    for( uint16_t i = 0; i < command_length; i++ )
    {
        hal_spi_in_out( p_lr20xx, command[i] );
    }

    // Send data
    for( uint16_t i = 0; i < data_length; i++ )
    {
        data[i] = hal_spi_in_out( p_lr20xx, 0 );
    }

    // Put NSS high as the spi transaction is finished
    gpio_set_level( p_lr20xx->nss, 1 );

    return LR20XX_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */
static void lr20xx_hal_wait_on_busy( int busy_pin )
{
    uint32_t cnt = 0;
    while(gpio_get_level(busy_pin)) {
        esp_rom_delay_us(1000);
        cnt ++;
        if( cnt >= 2000 ) {
            break;
        }
    }
}

static void lr20xx_hal_check_device_ready(const lr20xx_hal_context_t *context)
{
    if( radio_mode != RADIO_SLEEP )
    {
        lr20xx_hal_wait_on_busy( context->busy);
    }
    else
    {
        lora_spi_init(context);
        gpio_set_level(context->nss, 0);
        esp_rom_delay_us(1000);
        gpio_set_level(context->nss, 1);
        lr20xx_hal_wait_on_busy(context->busy);
        radio_mode = RADIO_AWAKE;
    }
}
static uint8_t hal_spi_in_out(const lr20xx_hal_context_t *context, const uint8_t out_data)
{
    uint8_t wdata = out_data, rdata = 0;
    spi_transaction_t t = {
        .rx_buffer = &rdata,
        .tx_buffer = &wdata,
        .rxlength = 8,
        .length = 8,
    };
    spi_device_polling_transmit(context->spi_handle, &t);
    return rdata;
}

/* --- EOF ------------------------------------------------------------------ */
