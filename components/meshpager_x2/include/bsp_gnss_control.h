#pragma once

#include "sdkconfig.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "driver/gpio.h"
#include "bsp_err_check.h"
// #include "esp_io_expander_tca6424.h"
#include "meshpager_x2.h"
#include <string.h>

/**************************************************************************************************
 *
 * GNSS UART Interface
 *
 * UART interface for communication with GNSS module.
 * Provides initialization, deinitialization, and data transmission functions.
 **************************************************************************************************/

/**
 * @brief Initialize GNSS UART driver
 *
 * @note This function can be called multiple times but will only initialize once
 * @param[in] rx_pin UART RX pin number
 * @param[in] tx_pin UART TX pin number
 * @param[in] baud UART baud rate (e.g., 115200)
 *
 * @return None
 */
void bsp_gnss_uart_init( uint32_t rx_pin, uint32_t tx_pin, uint32_t baud );

/**
 * @brief Deinitialize GNSS UART driver and free its resources
 *
 * @note Only executes if UART was previously initialized
 *
 * @return None
 */
void bsp_gnss_uart_deinit( );

/**
 * @brief Read bytes from GNSS UART
 *
 * @note Only reads if UART was previously initialized
 * @param[out] buf Pointer to buffer to store read data
 * @param[in] length Number of bytes to read
 * @param[in] ticks_to_wait Maximum time to wait for data (in RTOS ticks)
 *
 * @return
 *      - Number of bytes actually read
 *      - 0 if UART not initialized
 */
int gnss_uart_read_bytes(void *buf, uint32_t length, TickType_t ticks_to_wait);

/**
 * @brief Query whether the GNSS UART driver is currently installed
 *
 * @return true if bsp_gnss_uart_init() has run and bsp_gnss_uart_deinit() has not
 */
bool gnss_uart_is_initialized(void);

/**
 * @brief Get the number of bytes currently buffered in the GNSS UART RX ring buffer
 *
 * @note Only meaningful when the UART driver is installed
 *
 * @return
 *      - >= 0 number of buffered bytes
 *      - -1 if the UART driver is not installed or the query failed
 */
int gnss_uart_buffered_len(void);

/**
 * @brief Drain all bytes from the GNSS UART RX path (ring buffer + hardware FIFO)
 *
 * Reads the driver ring buffer to empty and flushes the hardware RX FIFO,
 * clearing any overrun state. Safe to call when the UART driver is not
 * installed (no-op). Use at the entry/exit of a parse window so a long
 * unattended overflow between windows cannot stall RX reception.
 *
 * @return None
 */
void gnss_uart_drain_rx(void);

/**
 * @brief Write bytes to GNSS UART
 *
 * @note Only writes if UART was previously initialized
 * @param[in] data Pointer to data buffer to write
 * @param[in] len Number of bytes to write
 *
 * @return
 *      - Number of bytes written on success
 *      - -1 if UART not initialized
 */
int gnss_uart_write_bytes(void *data, uint32_t len);

/**
 * @brief Print formatted string to GNSS UART
 *
 * @note This function uses variable arguments (printf-style)
 * @param[in] fmt Format string
 * @param[in] ... Variable arguments matching the format string
 *
 * @return None
 */
void gnss_uart_trace_print( const char* fmt, ... );

/**************************************************************************************************
 *
 * GNSS Power Control
 *
 * Functions for controlling GNSS module power state and reset sequence.
 **************************************************************************************************/

/**
 * @brief Configure GNSS RX/TX/PPS0 pins as input with pull-up
 *
 * This is used to leave the GNSS UART pins in a safe idle state,
 * for example before external firmware download/programming.
 *
 * @return None
 */
void bsp_gnss_set_io_input_pullup(void);

/**
 * @brief Initialize GNSS power control
 *
 * This function performs the complete GNSS module initialization sequence:
 * - Enables GNSS power (BSP_GNSS_PWR_EN)
 * - Enables VRTC power (BSP_GNSS_VRTC_EN)
 * - Performs hardware reset sequence (BSP_GNSS_RST)
 * - Configures sleep and RTC interrupt pins
 * - Configures GNSS PPS0, TX, and RX pins as inputs with pull-up
 *
 * @return None
 */
void bsp_gnss_power_init( void );

/**
 * @brief Power off GNSS module
 *
 * This function performs the complete GNSS module power-off sequence:
 * - Disables GNSS power
 * - Configures GNSS TX as output and sets low
 * - Configures GNSS RX as input with pull-down
 * - Resets GNSS control pins to low state
 *
 * @return None
 */
void bsp_gnss_poweroff( void );

/**
 * @brief Reset the gps for download
 * 
 */
void bsp_gnss_reset( void );

/**************************************************************************************************
 *
 * GNSS Sleep Control
 *
 * Functions for controlling GNSS module sleep mode and power states.
 **************************************************************************************************/

/**
 * @brief Lock GNSS module in active state (prevent sleep)
 *
 * Sends NMEA command "$PAIR382,1" multiple times to disable GNSS sleep mode
 *
 * @note Command is sent 25 times with 40ms delay between attempts
 *
 * @return None
 */
void bsp_gnss_scan_lock_sleep( void );

/**
 * @brief Unlock GNSS module sleep (allow sleep)
 *
 * Sends NMEA command "$PAIR382,0" multiple times to enable GNSS sleep mode
 *
 * @note Command is sent 4 times with 40ms delay between attempts
 *
 * @return None
 */
void bsp_gnss_scan_unlock_sleep( void );

/**************************************************************************************************
 *
 * GNSS RTC Mode
 *
 * Functions for controlling GNSS RTC (Real Time Clock) mode.
 **************************************************************************************************/

/**
 * @brief Enter GNSS RTC mode
 *
 * Sends NMEA command "$PAIR650,0" to enter RTC mode
 *
 * @note Command is sent 25 times with 40ms delay between attempts
 *
 * @return None
 */
void bsp_gnss_scan_enter_rtc_mode( void );

/**
 * @brief Set GNSS UTC time
 *
 * This function is reserved for setting UTC time on the GNSS module
 *
 * @note Currently empty implementation
 *
 * @return None
 */
void bsp_gnss_scan_set_utc( void );

/**************************************************************************************************
 *
 * GNSS Scan Control
 *
 * High-level functions for starting and stopping GNSS scanning operations.
 **************************************************************************************************/

/**
 * @brief Start GNSS scanning operation at the default baud (115200)
 *
 * Convenience wrapper around bsp_gnss_scan_start_with_baud(115200). Kept for
 * callers that don't need to override the UART baud rate.
 *
 * @return
 *      - true NMEA traffic detected (module is awake and transmitting)
 *      - false No NMEA detected after all retries (wake failed)
 */
bool bsp_gnss_scan_start( void );

/**
 * @brief Start GNSS scanning operation with a configurable UART baud rate
 *
 * This function performs the complete GNSS scan startup sequence:
 * - Initializes UART at the requested baud rate
 * - Powers on GNSS module
 * - Sends RTC interrupt pulse to wake GNSS
 * - Locks GNSS sleep mode to keep it active
 * - Sets UTC time
 * - Waits for the first NMEA byte to confirm the module actually woke up;
 *   retries the wake sequence (pulse + keep-alive) up to
 *   BSP_GNSS_SCAN_START_RETRY times if no NMEA appears.
 *
 * @note The UART driver is installed only if not already installed. To switch
 *       baud rates between runs, call bsp_gnss_scan_stop() (which deinitializes
 *       the UART) first — bsp_gnss_scan_stop() already does this, so two
 *       successive `gnss` invocations with different bauds re-init at the new
 *       rate.
 *
 * @param[in] baud UART baud rate (e.g. 115200, 921600)
 *
 * @return
 *      - true NMEA traffic detected (module is awake and transmitting)
 *      - false No NMEA detected after all retries (wake failed)
 */
bool bsp_gnss_scan_start_with_baud( uint32_t baud );

/**
 * @brief Stop GNSS scanning operation
 *
 * This function performs the complete GNSS scan shutdown sequence:
 * - Unlocks GNSS sleep mode
 * - Enters RTC mode
 * - Powers on GNSS (maintains VRTC)
 * - Deinitializes UART
 *
 * @return None
 */
void bsp_gnss_scan_stop( void );
