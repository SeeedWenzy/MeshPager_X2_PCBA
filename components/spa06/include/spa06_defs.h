/**
 * @file  spa06_defs.h
 * @brief SPA06-003 digital pressure sensor driver - definitions
 *
 * C port of the Adafruit SPA06_003 Arduino driver, restructured to follow the
 * same portable-component layout used by the Bosch BMM350 driver in this
 * project. The compensation math, register map, calibration-coefficient
 * parsing and scaling-factor table are taken from the SPA06-003 datasheet
 * (Goermicro, Ver2.0) and verified against it.
 *
 * The sensor is an I2C/SPI 24-bit pressure & temperature sensor. Only the I2C
 * interface is supported by this port (matching the ESP32-S3 use case).
 *
 * Original Arduino driver: Copyright (c) Adafruit Industries, MIT license.
 *
 * @license MIT
 */

#ifndef _SPA06_DEFS_H
#define _SPA06_DEFS_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/**************************** API Version *******************************/
#define SPA06_VER_MAJOR   1
#define SPA06_VER_MINOR   0
#define SPA06_VER_BUGFIX  0

/*************************** Header files *******************************/
#include <stdint.h>
#include <stddef.h>

/***************************** Common Macros ****************************/

#ifndef NULL
#define NULL ((void *)0)
#endif

/*! Maximum number of bytes read in a single burst (coef block is 21 bytes) */
#define SPA06_READ_BUFFER_LENGTH UINT8_C(21)

/************************* Interface return type *************************/

#ifndef SPA06_INTF_RET_TYPE
/*! Return type used by the platform read/write callbacks. */
#define SPA06_INTF_RET_TYPE int8_t
#endif

#ifndef SPA06_INTF_RET_SUCCESS
/*! Success value returned by the platform read/write callbacks. */
#define SPA06_INTF_RET_SUCCESS INT8_C(0)
#endif

/**************************** Error codes *******************************/

#define SPA06_OK                INT8_C(0)    /*!< Success */
#define SPA06_E_NULL_PTR        INT8_C(-1)   /*!< Null pointer passed */
#define SPA06_E_COM_FAIL        INT8_C(-2)   /*!< Communication failure */
#define SPA06_E_DEV_NOT_FOUND   INT8_C(-3)   /*!< Device not found (wrong chip id) */
#define SPA06_E_INVALID_CONFIG INT8_C(-4)   /*!< Invalid configuration */
#define SPA06_E_COEF_NOT_READY  INT8_C(-5)   /*!< Calibration coefficients not ready */
#define SPA06_E_TIMEOUT        INT8_C(-6)   /*!< Timed out waiting for readiness */
#define SPA06_E_INVALID_INPUT   INT8_C(-7)   /*!< Invalid input parameter */

/************************** I2C slave addresses **************************/

/*! Primary I2C address (SDO tied to GND) */
#define SPA06_I2C_ADDR_PRIM UINT8_C(0x77)
/*! Secondary I2C address (SDO tied to VDDIO) */
#define SPA06_I2C_ADDR_SEC  UINT8_C(0x76)
/*! Default I2C address used when none is provided */
#define SPA06_I2C_ADDR_DEFAULT SPA06_I2C_ADDR_PRIM

/*! Expected product/revision ID returned from the ID register */
#define SPA06_CHIP_ID UINT8_C(0x11)

/**************************** I2C bus speed *****************************/
/*! Default I2C clock for the standalone ESP-i2cdev bridge. */
#define SPA06_I2C_FREQ_HZ (400000U)

/******************************* Registers *******************************/

/*! Pressure data registers (24-bit, MSB first) */
#define SPA06_REG_PSR_B2 UINT8_C(0x00)  /*!< Pressure data byte 2 (MSB) */
#define SPA06_REG_PSR_B1 UINT8_C(0x01)  /*!< Pressure data byte 1 */
#define SPA06_REG_PSR_B0 UINT8_C(0x02)  /*!< Pressure data byte 0 (LSB) */

/*! Temperature data registers (24-bit, MSB first) */
#define SPA06_REG_TMP_B2 UINT8_C(0x03)  /*!< Temperature data byte 2 (MSB) */
#define SPA06_REG_TMP_B1 UINT8_C(0x04)  /*!< Temperature data byte 1 */
#define SPA06_REG_TMP_B0 UINT8_C(0x05)  /*!< Temperature data byte 0 (LSB) */

/*! Configuration registers */
#define SPA06_REG_PRS_CFG  UINT8_C(0x06) /*!< Pressure configuration */
#define SPA06_REG_TMP_CFG  UINT8_C(0x07) /*!< Temperature configuration */
#define SPA06_REG_MEAS_CFG UINT8_C(0x08) /*!< Measurement configuration / status */
#define SPA06_REG_CFG_REG  UINT8_C(0x09) /*!< Interrupt & FIFO configuration */

/*! Status registers */
#define SPA06_REG_INT_STS  UINT8_C(0x0A) /*!< Interrupt status */
#define SPA06_REG_FIFO_STS UINT8_C(0x0B) /*!< FIFO status */

/*! Control registers */
#define SPA06_REG_RESET UINT8_C(0x0C) /*!< Soft reset / FIFO flush */
#define SPA06_REG_ID    UINT8_C(0x0D) /*!< Product & revision ID */

/*! Calibration coefficient block (0x10 .. 0x24, 21 bytes) */
#define SPA06_REG_COEF UINT8_C(0x10)
/*! Number of coefficient bytes */
#define SPA06_COEF_LEN UINT8_C(21)

/********************* PRS_CFG / TMP_CFG field masks *********************/

/*! Measurement rate occupies bits [7:4] of PRS_CFG / TMP_CFG. */
#define SPA06_RATE_MASK   UINT8_C(0xF0)
#define SPA06_RATE_SHIFT  (0U)
#define SPA06_RATE_POS    (4U)  /*!< Field bit position (LSB index) */

/*! Oversampling occupies bits [3:0] of PRS_CFG / TMP_CFG. */
#define SPA06_PRC_MASK    UINT8_C(0x0F)
#define SPA06_PRC_SHIFT   (0U)
#define SPA06_PRC_POS     (0U)

/*! Oversampling value above which the result-shift bit must be set. */
#define SPA06_SHIFT_THRESHOLD_OSR UINT8_C(0x08)

/********************* MEAS_CFG field masks / bits ************************/

#define SPA06_MEAS_CFG_COEF_RDY_BIT  (7U)  /*!< Coefficients ready */
#define SPA06_MEAS_CFG_SENSOR_RDY_BIT (6U) /*!< Sensor ready */
#define SPA06_MEAS_CFG_TMP_RDY_BIT   (5U)  /*!< Temperature result ready */
#define SPA06_MEAS_CFG_PRS_RDY_BIT   (4U)  /*!< Pressure result ready */
#define SPA06_MEAS_CTRL_MASK         UINT8_C(0x07) /*!< Measurement mode [2:0] */
#define SPA06_MEAS_CTRL_POS          (0U)

/********************* CFG_REG field masks / bits ************************/

#define SPA06_CFG_INT_HL_BIT    (7U)  /*!< Interrupt polarity */
#define SPA06_CFG_INT_FIFO_BIT  (6U)  /*!< FIFO interrupt enable */
#define SPA06_CFG_INT_TMP_BIT   (5U)  /*!< Temperature-ready interrupt enable */
#define SPA06_CFG_INT_PRS_BIT   (4U)  /*!< Pressure-ready interrupt enable */
#define SPA06_CFG_T_SHIFT_BIT   (3U)  /*!< Temperature result shift */
#define SPA06_CFG_P_SHIFT_BIT   (2U)  /*!< Pressure result shift */
#define SPA06_CFG_FIFO_EN_BIT   (1U)  /*!< FIFO enable */

/********************* RESET field masks / bits **************************/

#define SPA06_RESET_FIFO_FLUSH_BIT (7U) /*!< FIFO flush */
#define SPA06_RESET_SOFT_RST_MASK  UINT8_C(0x0F) /*!< Soft reset [3:0] */
#define SPA06_RESET_SOFT_RST_VAL   UINT8_C(0x09) /*!< Value to trigger soft reset */

/********************* INT_STS / FIFO_STS bits ***************************/

#define SPA06_INT_FIFO_FULL UINT8_C(0x04) /*!< FIFO full flag */
#define SPA06_INT_TMP_RDY   UINT8_C(0x02) /*!< Temperature ready flag */
#define SPA06_INT_PRS_RDY   UINT8_C(0x01) /*!< Pressure ready flag */

#define SPA06_FIFO_STS_FULL_BIT  (1U) /*!< FIFO full */
#define SPA06_FIFO_STS_EMPTY_BIT (0U) /*!< FIFO empty */

/******************************* Enums **********************************/

/*!
 * @brief Measurement rate (pressure and temperature). Values match the
 *        PRS_CFG / TMP_CFG rate field encoding.
 */
typedef enum {
    SPA06_RATE_1 = 0x00,     /*!< 1 measurements per second */
    SPA06_RATE_2 = 0x01,     /*!< 2 measurements per second */
    SPA06_RATE_4 = 0x02,     /*!< 4 measurements per second */
    SPA06_RATE_8 = 0x03,     /*!< 8 measurements per second */
    SPA06_RATE_16 = 0x04,    /*!< 16 measurements per second */
    SPA06_RATE_32 = 0x05,    /*!< 32 measurements per second */
    SPA06_RATE_64 = 0x06,    /*!< 64 measurements per second */
    SPA06_RATE_128 = 0x07,   /*!< 128 measurements per second */
    SPA06_RATE_25_16 = 0x08, /*!< 25/16 samples per second */
    SPA06_RATE_25_8 = 0x09,  /*!< 25/8 samples per second */
    SPA06_RATE_25_4 = 0x0A,  /*!< 25/4 samples per second */
    SPA06_RATE_25_2 = 0x0B,  /*!< 25/2 samples per second */
    SPA06_RATE_25 = 0x0C,    /*!< 25 samples per second */
    SPA06_RATE_50 = 0x0D,    /*!< 50 samples per second */
    SPA06_RATE_100 = 0x0E,   /*!< 100 samples per second */
    SPA06_RATE_200 = 0x0F    /*!< 200 samples per second */
} spa06_rate_t;

/*!
 * @brief Oversampling rate (shared by pressure and temperature).
 */
typedef enum {
    SPA06_OVERSAMPLE_1 = 0x00,   /*!< Single */
    SPA06_OVERSAMPLE_2 = 0x01,   /*!< 2 times */
    SPA06_OVERSAMPLE_4 = 0x02,   /*!< 4 times */
    SPA06_OVERSAMPLE_8 = 0x03,   /*!< 8 times */
    SPA06_OVERSAMPLE_16 = 0x04,  /*!< 16 times */
    SPA06_OVERSAMPLE_32 = 0x05,  /*!< 32 times */
    SPA06_OVERSAMPLE_64 = 0x06,  /*!< 64 times */
    SPA06_OVERSAMPLE_128 = 0x07  /*!< 128 times */
} spa06_oversample_t;

/*!
 * @brief Measurement mode (MEAS_CTRL field of MEAS_CFG).
 */
typedef enum {
    SPA06_MEAS_IDLE = 0x00,               /*!< Idle / stop background measurement */
    SPA06_MEAS_PRESSURE = 0x01,           /*!< Pressure measurement (command mode) */
    SPA06_MEAS_TEMPERATURE = 0x02,        /*!< Temperature measurement (command mode) */
    SPA06_MEAS_CONTINUOUS_PRESSURE = 0x05,    /*!< Continuous pressure (background) */
    SPA06_MEAS_CONTINUOUS_TEMPERATURE = 0x06,/*!< Continuous temperature (background) */
    SPA06_MEAS_CONTINUOUS_BOTH = 0x07     /*!< Continuous pressure & temperature */
} spa06_meas_mode_t;

/*!
 * @brief Interrupt polarity.
 */
typedef enum {
    SPA06_INT_ACTIVE_LOW = 0x00,  /*!< Interrupt active low */
    SPA06_INT_ACTIVE_HIGH = 0x01  /*!< Interrupt active high */
} spa06_int_polarity_t;

/************************* Function pointer types ************************/

/* Pre-declaration */
struct spa06_dev;

/*!
 * @brief Bus read callback. Maps to the platform I2C read function.
 */
typedef SPA06_INTF_RET_TYPE (*spa06_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data,
                                                uint32_t len, void *intf_ptr);

/*!
 * @brief Bus write callback. Maps to the platform I2C write function.
 */
typedef SPA06_INTF_RET_TYPE (*spa06_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data,
                                                  uint32_t len, void *intf_ptr);

/*!
 * @brief Microsecond delay callback.
 */
typedef void (*spa06_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

/************************* Structure definitions *************************/

/*!
 * @brief Calibration coefficients parsed from the COEF register block.
 *
 * Bit widths per the datasheet:
 *  c0, c1           - 12-bit 2's complement
 *  c00, c10         - 20-bit 2's complement
 *  c01, c11, c20,
 *    c21, c30       - 16-bit 2's complement
 *  c31, c40         - 12-bit 2's complement
 */
struct spa06_calib_coef {
    int16_t c0;   /*!< 12-bit */
    int16_t c1;   /*!< 12-bit */
    int32_t c00;  /*!< 20-bit */
    int32_t c10;  /*!< 20-bit */
    int16_t c01;  /*!< 16-bit */
    int16_t c11;  /*!< 16-bit */
    int16_t c20;  /*!< 16-bit */
    int16_t c21;  /*!< 16-bit */
    int16_t c30;  /*!< 16-bit */
    int16_t c31;  /*!< 12-bit */
    int16_t c40;  /*!< 12-bit */
};

/*!
 * @brief Device structure. The caller allocates one instance and passes it to
 *        every API. The bus callbacks (read/write/delay_us) and intf_ptr are
 *        wired up by the platform bridge (common.c / spa06_bsp_i2c1).
 */
struct spa06_dev {
    /*!
     * Interface pointer - links the platform bus descriptor (e.g. an
     * i2c_dev_t * or a bsp context *) used by the read/write callbacks.
     */
    void *intf_ptr;

    /*! Chip ID read from the ID register (should be SPA06_CHIP_ID). */
    uint8_t chip_id;

    /*! I2C slave address in use. */
    uint8_t dev_addr;

    /*! Bus read function pointer. */
    spa06_read_fptr_t read;

    /*! Bus write function pointer. */
    spa06_write_fptr_t write;

    /*! Microsecond delay function pointer. */
    spa06_delay_us_fptr_t delay_us;

    /*! Stores the last interface return code for diagnostics. */
    SPA06_INTF_RET_TYPE intf_rslt;

    /*! Parsed calibration coefficients. */
    struct spa06_calib_coef coef;

    /*!
     * Cached pressure oversampling. Used by spa06_calculate_pressure() to
     * pick the scaling factor without an extra register read. Updated by
     * spa06_set_pressure_oversampling() and spa06_init().
     */
    spa06_oversample_t pressure_oversampling;

    /*! Cached temperature oversampling (see pressure_oversampling). */
    spa06_oversample_t temperature_oversampling;
};

#ifdef __cplusplus
} /* extern "C" */
#endif /* End of CPP guard */

#endif /* _SPA06_DEFS_H */
