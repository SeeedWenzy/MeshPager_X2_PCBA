/**
 * @file  spa06.c
 * @brief SPA06-003 digital pressure & temperature sensor driver - core
 *
 * Interface-agnostic driver core. The bus is accessed only through the
 * read/write/delay_us callbacks stored in #spa06_dev, so this file contains
 * no platform code and can be unit-tested or retargeted without changes.
 *
 * Compensation math, register map and coefficient parsing follow the
 * SPA06-003 datasheet (Goermicro Ver2.0). Ported from the Adafruit
 * SPA06_003 Arduino driver (MIT, Adafruit Industries).
 *
 * @license MIT
 */

#include "spa06.h"

#include <string.h>

/************************** Private helpers *******************************/

/*!
 * @brief Validate that the device and its bus callbacks are populated.
 */
static int8_t null_ptr_check(const struct spa06_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) ||
        (dev->delay_us == NULL)) {
        rslt = SPA06_E_NULL_PTR;
    } else {
        rslt = SPA06_OK;
    }

    return rslt;
}

/*!
 * @brief Read one register byte.
 */
static int8_t spa06_read_reg(struct spa06_dev *dev, uint8_t reg_addr, uint8_t *reg_data)
{
    int8_t rslt = null_ptr_check(dev);

    if ((rslt == SPA06_OK) && (reg_data != NULL)) {
        dev->intf_rslt = dev->read(reg_addr, reg_data, 1, dev->intf_ptr);

        if (dev->intf_rslt != SPA06_INTF_RET_SUCCESS) {
            rslt = SPA06_E_COM_FAIL;
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief Write one register byte.
 */
static int8_t spa06_write_reg(struct spa06_dev *dev, uint8_t reg_addr, uint8_t reg_data)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        dev->intf_rslt = dev->write(reg_addr, &reg_data, 1, dev->intf_ptr);

        if (dev->intf_rslt != SPA06_INTF_RET_SUCCESS) {
            rslt = SPA06_E_COM_FAIL;
        }
    }

    return rslt;
}

/*!
 * @brief Read a masked bit-field from a register.
 * @param[in]  dev   Device instance.
 * @param[in]  reg   Register address.
 * @param[in]  mask  Field mask (contiguous bits).
 * @param[in]  pos   Field LSB position.
 * @param[out] value Destination for the (right-justified) field value.
 * @return SPA06_OK on success, negative error code otherwise.
 */
static int8_t spa06_get_field(struct spa06_dev *dev, uint8_t reg, uint8_t mask,
                              uint8_t pos, uint8_t *value)
{
    int8_t rslt;
    uint8_t reg_val = 0;

    rslt = spa06_read_reg(dev, reg, &reg_val);
    if (rslt == SPA06_OK) {
        *value = (uint8_t)((reg_val & mask) >> pos);
    }

    return rslt;
}

/*!
 * @brief Write a masked bit-field into a register (read-modify-write).
 * @param[in] dev   Device instance.
 * @param[in] reg   Register address.
 * @param[in] mask  Field mask (contiguous bits).
 * @param[in] pos   Field LSB position.
 * @param[in] value Right-justified field value to write.
 * @return SPA06_OK on success, negative error code otherwise.
 */
static int8_t spa06_set_field(struct spa06_dev *dev, uint8_t reg, uint8_t mask,
                              uint8_t pos, uint8_t value)
{
    int8_t rslt;
    uint8_t reg_val = 0;

    rslt = spa06_read_reg(dev, reg, &reg_val);
    if (rslt == SPA06_OK) {
        reg_val = (uint8_t)((reg_val & (uint8_t)~mask) |
                            ((value << pos) & mask));
        rslt = spa06_write_reg(dev, reg, reg_val);
    }

    return rslt;
}

/************************** Generic register access **********************/

int8_t spa06_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
                      struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if ((rslt == SPA06_OK) && (reg_data != NULL) && (len > 0U)) {
        dev->intf_rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr);

        if (dev->intf_rslt != SPA06_INTF_RET_SUCCESS) {
            rslt = SPA06_E_COM_FAIL;
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint16_t len,
                      struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if ((rslt == SPA06_OK) && (reg_data != NULL) && (len > 0U)) {
        dev->intf_rslt = dev->write(reg_addr, reg_data, len, dev->intf_ptr);

        if (dev->intf_rslt != SPA06_INTF_RET_SUCCESS) {
            rslt = SPA06_E_COM_FAIL;
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_delay_us(uint32_t period_us, const struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        dev->delay_us(period_us, dev->intf_ptr);
    }

    return rslt;
}

/************************** Status / readiness ****************************/

int8_t spa06_get_meas_cfg(uint8_t *meas_cfg, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if ((rslt == SPA06_OK) && (meas_cfg != NULL)) {
        rslt = spa06_read_reg(dev, SPA06_REG_MEAS_CFG, meas_cfg);
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

static int8_t spa06_get_meas_cfg_bit(uint8_t bit, uint8_t *ready, struct spa06_dev *dev)
{
    int8_t rslt;
    uint8_t reg_val = 0;

    rslt = spa06_read_reg(dev, SPA06_REG_MEAS_CFG, &reg_val);
    if (rslt == SPA06_OK) {
        *ready = (uint8_t)((reg_val >> bit) & 0x01U);
    }

    return rslt;
}

int8_t spa06_is_coeff_ready(uint8_t *ready, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if ((rslt == SPA06_OK) && (ready != NULL)) {
        rslt = spa06_get_meas_cfg_bit(SPA06_MEAS_CFG_COEF_RDY_BIT, ready, dev);
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_is_sensor_ready(uint8_t *ready, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if ((rslt == SPA06_OK) && (ready != NULL)) {
        rslt = spa06_get_meas_cfg_bit(SPA06_MEAS_CFG_SENSOR_RDY_BIT, ready, dev);
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_is_temp_ready(uint8_t *ready, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if ((rslt == SPA06_OK) && (ready != NULL)) {
        rslt = spa06_get_meas_cfg_bit(SPA06_MEAS_CFG_TMP_RDY_BIT, ready, dev);
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_is_pressure_ready(uint8_t *ready, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if ((rslt == SPA06_OK) && (ready != NULL)) {
        rslt = spa06_get_meas_cfg_bit(SPA06_MEAS_CFG_PRS_RDY_BIT, ready, dev);
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

/************************** Configuration *********************************/

int8_t spa06_set_pressure_oversampling(spa06_oversample_t oversampling, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_PRS_CFG, SPA06_PRC_MASK,
                               SPA06_PRC_POS, (uint8_t)oversampling);
    }

    if (rslt == SPA06_OK) {
        /* Oversampling above 8x requires the pressure result shift bit. */
        rslt = spa06_set_pressure_shift((uint8_t)(oversampling > SPA06_OVERSAMPLE_8), dev);
    }

    if (rslt == SPA06_OK) {
        dev->pressure_oversampling = oversampling;
    }

    return rslt;
}

int8_t spa06_get_pressure_oversampling(spa06_oversample_t *oversampling, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t val = 0;

    if ((rslt == SPA06_OK) && (oversampling != NULL)) {
        rslt = spa06_get_field(dev, SPA06_REG_PRS_CFG, SPA06_PRC_MASK,
                               SPA06_PRC_POS, &val);
        if (rslt == SPA06_OK) {
            *oversampling = (spa06_oversample_t)val;
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_set_temperature_oversampling(spa06_oversample_t oversampling, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_TMP_CFG, SPA06_PRC_MASK,
                               SPA06_PRC_POS, (uint8_t)oversampling);
    }

    if (rslt == SPA06_OK) {
        rslt = spa06_set_temp_shift((uint8_t)(oversampling > SPA06_OVERSAMPLE_8), dev);
    }

    if (rslt == SPA06_OK) {
        dev->temperature_oversampling = oversampling;
    }

    return rslt;
}

int8_t spa06_get_temperature_oversampling(spa06_oversample_t *oversampling, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t val = 0;

    if ((rslt == SPA06_OK) && (oversampling != NULL)) {
        rslt = spa06_get_field(dev, SPA06_REG_TMP_CFG, SPA06_PRC_MASK,
                               SPA06_PRC_POS, &val);
        if (rslt == SPA06_OK) {
            *oversampling = (spa06_oversample_t)val;
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_set_pressure_rate(spa06_rate_t rate, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_PRS_CFG, SPA06_RATE_MASK,
                               SPA06_RATE_POS, (uint8_t)rate);
    }

    return rslt;
}

int8_t spa06_get_pressure_rate(spa06_rate_t *rate, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t val = 0;

    if ((rslt == SPA06_OK) && (rate != NULL)) {
        rslt = spa06_get_field(dev, SPA06_REG_PRS_CFG, SPA06_RATE_MASK,
                               SPA06_RATE_POS, &val);
        if (rslt == SPA06_OK) {
            *rate = (spa06_rate_t)val;
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_set_temperature_rate(spa06_rate_t rate, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_TMP_CFG, SPA06_RATE_MASK,
                               SPA06_RATE_POS, (uint8_t)rate);
    }

    return rslt;
}

int8_t spa06_get_temperature_rate(spa06_rate_t *rate, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t val = 0;

    if ((rslt == SPA06_OK) && (rate != NULL)) {
        rslt = spa06_get_field(dev, SPA06_REG_TMP_CFG, SPA06_RATE_MASK,
                               SPA06_RATE_POS, &val);
        if (rslt == SPA06_OK) {
            *rate = (spa06_rate_t)val;
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_set_measurement_mode(spa06_meas_mode_t mode, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_MEAS_CFG, SPA06_MEAS_CTRL_MASK,
                               SPA06_MEAS_CTRL_POS, (uint8_t)mode);
    }

    return rslt;
}

int8_t spa06_get_measurement_mode(spa06_meas_mode_t *mode, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t val = 0;

    if ((rslt == SPA06_OK) && (mode != NULL)) {
        rslt = spa06_get_field(dev, SPA06_REG_MEAS_CFG, SPA06_MEAS_CTRL_MASK,
                               SPA06_MEAS_CTRL_POS, &val);
        if (rslt == SPA06_OK) {
            *mode = (spa06_meas_mode_t)val;
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_set_interrupt_polarity(spa06_int_polarity_t polarity, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_CFG_REG,
                               (uint8_t)(1U << SPA06_CFG_INT_HL_BIT),
                               SPA06_CFG_INT_HL_BIT, (uint8_t)polarity);
    }

    return rslt;
}

int8_t spa06_set_interrupt_source(uint8_t fifo, uint8_t temp_ready,
                                   uint8_t pres_ready, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_CFG_REG,
                               (uint8_t)(1U << SPA06_CFG_INT_FIFO_BIT),
                               SPA06_CFG_INT_FIFO_BIT, fifo);
    }

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_CFG_REG,
                               (uint8_t)(1U << SPA06_CFG_INT_TMP_BIT),
                               SPA06_CFG_INT_TMP_BIT, temp_ready);
    }

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_CFG_REG,
                               (uint8_t)(1U << SPA06_CFG_INT_PRS_BIT),
                               SPA06_CFG_INT_PRS_BIT, pres_ready);
    }

    return rslt;
}

int8_t spa06_set_temp_shift(uint8_t enable, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_CFG_REG,
                               (uint8_t)(1U << SPA06_CFG_T_SHIFT_BIT),
                               SPA06_CFG_T_SHIFT_BIT, enable ? 1U : 0U);
    }

    return rslt;
}

int8_t spa06_set_pressure_shift(uint8_t enable, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_CFG_REG,
                               (uint8_t)(1U << SPA06_CFG_P_SHIFT_BIT),
                               SPA06_CFG_P_SHIFT_BIT, enable ? 1U : 0U);
    }

    return rslt;
}

/************************** FIFO / status *********************************/

int8_t spa06_enable_fifo(uint8_t enable, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_CFG_REG,
                               (uint8_t)(1U << SPA06_CFG_FIFO_EN_BIT),
                               SPA06_CFG_FIFO_EN_BIT, enable ? 1U : 0U);
    }

    return rslt;
}

int8_t spa06_is_fifo_enabled(uint8_t *enabled, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t reg_val = 0;

    if ((rslt == SPA06_OK) && (enabled != NULL)) {
        rslt = spa06_read_reg(dev, SPA06_REG_CFG_REG, &reg_val);
        if (rslt == SPA06_OK) {
            *enabled = (uint8_t)((reg_val >> SPA06_CFG_FIFO_EN_BIT) & 0x01U);
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_is_fifo_empty(uint8_t *empty, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t reg_val = 0;

    if ((rslt == SPA06_OK) && (empty != NULL)) {
        rslt = spa06_read_reg(dev, SPA06_REG_FIFO_STS, &reg_val);
        if (rslt == SPA06_OK) {
            *empty = (uint8_t)((reg_val >> SPA06_FIFO_STS_EMPTY_BIT) & 0x01U);
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_is_fifo_full(uint8_t *full, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t reg_val = 0;

    if ((rslt == SPA06_OK) && (full != NULL)) {
        rslt = spa06_read_reg(dev, SPA06_REG_FIFO_STS, &reg_val);
        if (rslt == SPA06_OK) {
            *full = (uint8_t)((reg_val >> SPA06_FIFO_STS_FULL_BIT) & 0x01U);
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_get_status(uint8_t *status, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t reg_val = 0;

    if ((rslt == SPA06_OK) && (status != NULL)) {
        rslt = spa06_read_reg(dev, SPA06_REG_INT_STS, &reg_val);
        if (rslt == SPA06_OK) {
            *status = (uint8_t)(reg_val & (SPA06_INT_FIFO_FULL |
                                           SPA06_INT_TMP_RDY |
                                           SPA06_INT_PRS_RDY));
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_flush_fifo(struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_RESET,
                               (uint8_t)(1U << SPA06_RESET_FIFO_FLUSH_BIT),
                               SPA06_RESET_FIFO_FLUSH_BIT, 1U);
    }

    return rslt;
}

/************************** Reset / chip id *******************************/

int8_t spa06_soft_reset(struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if (rslt == SPA06_OK) {
        rslt = spa06_set_field(dev, SPA06_REG_RESET, SPA06_RESET_SOFT_RST_MASK,
                               0U, SPA06_RESET_SOFT_RST_VAL);
    }

    return rslt;
}

int8_t spa06_get_chip_id(uint8_t *chip_id, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);

    if ((rslt == SPA06_OK) && (chip_id != NULL)) {
        rslt = spa06_read_reg(dev, SPA06_REG_ID, chip_id);
        if (rslt == SPA06_OK) {
            dev->chip_id = *chip_id;
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

/************************** Calibration coefficients **********************/

/*!
 * @brief Sign-extend a 12-bit value held in an int16_t.
 */
static int16_t spa06_sext12(int16_t value)
{
    if ((value & 0x0800U) != 0U) {
        value |= (int16_t)0xF000U;
    }
    return value;
}

/*!
 * @brief Sign-extend a 20-bit value held in an int32_t.
 */
static int32_t spa06_sext20(int32_t value)
{
    if (((uint32_t)value & 0x080000U) != 0U) {
        value |= (int32_t)0xFFF00000U;
    }
    return value;
}

int8_t spa06_read_coefficients(struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t coef_data[SPA06_COEF_LEN];

    if (rslt != SPA06_OK) {
        return rslt;
    }

    /* Coefficients must be ready before they can be read. */
    uint8_t ready = 0;
    rslt = spa06_is_coeff_ready(&ready, dev);
    if ((rslt != SPA06_OK) || (ready == 0U)) {
        return (rslt == SPA06_OK) ? SPA06_E_COEF_NOT_READY : rslt;
    }

    rslt = spa06_get_regs(SPA06_REG_COEF, coef_data, SPA06_COEF_LEN, dev);
    if (rslt != SPA06_OK) {
        return rslt;
    }

    /* c0: 12-bit (coef[0]<<4 | coef[1]>>4) */
    dev->coef.c0 = spa06_sext12((int16_t)(((uint16_t)coef_data[0] << 4) |
                                           (coef_data[1] >> 4)));

    /* c1: 12-bit (low nibble of coef[1] <<8 | coef[2]) */
    dev->coef.c1 = spa06_sext12((int16_t)(((uint16_t)(coef_data[1] & 0x0FU) << 8) |
                                           coef_data[2]));

    /* c00: 20-bit (coef[3]<<12 | coef[4]<<4 | coef[5]>>4) */
    {
        uint32_t tmp = (((uint32_t)coef_data[3] << 12) & 0xFF000U) |
                       (((uint32_t)coef_data[4] << 4) & 0x00FF0U) |
                       ((uint32_t)(coef_data[5] >> 4) & 0x0000FU);
        dev->coef.c00 = spa06_sext20((int32_t)tmp);
    }

    /* c10: 20-bit (low nibble of coef[5]<<16 | coef[6]<<8 | coef[7]) */
    {
        uint32_t tmp = (((uint32_t)(coef_data[5] & 0x0FU) << 16) & 0xF0000U) |
                       (((uint32_t)coef_data[6] << 8) & 0x0FF00U) |
                       ((uint32_t)coef_data[7] & 0x000FFU);
        dev->coef.c10 = spa06_sext20((int32_t)tmp);
    }

    /* c01: 16-bit (coef[8]<<8 | coef[9]) */
    dev->coef.c01 = (int16_t)(((uint16_t)coef_data[8] << 8) | coef_data[9]);

    /* c11: 16-bit (coef[10]<<8 | coef[11]) */
    dev->coef.c11 = (int16_t)(((uint16_t)coef_data[10] << 8) | coef_data[11]);

    /* c20: 16-bit (coef[12]<<8 | coef[13]) */
    dev->coef.c20 = (int16_t)(((uint16_t)coef_data[12] << 8) | coef_data[13]);

    /* c21: 16-bit (coef[14]<<8 | coef[15]) */
    dev->coef.c21 = (int16_t)(((uint16_t)coef_data[14] << 8) | coef_data[15]);

    /* c30: 16-bit (coef[16]<<8 | coef[17]) */
    dev->coef.c30 = (int16_t)(((uint16_t)coef_data[16] << 8) | coef_data[17]);

    /* c31: 12-bit (coef[18]<<4 | coef[19]>>4) */
    dev->coef.c31 = spa06_sext12((int16_t)(((uint16_t)(coef_data[18] << 4) & 0xFF0U) |
                                            ((uint16_t)(coef_data[19] >> 4) & 0x00FU)));

    /* c40: 12-bit (low nibble of coef[19]<<8 | coef[20]) */
    dev->coef.c40 = spa06_sext12((int16_t)(((uint16_t)(coef_data[19] & 0x0FU) << 8) |
                                            coef_data[20]));

    return rslt;
}

/************************** Compensation math *****************************/

float spa06_get_scaling_factor(spa06_oversample_t oversample)
{
    float factor;

    switch (oversample) {
    case SPA06_OVERSAMPLE_1:
        factor = 524288.0f;
        break;
    case SPA06_OVERSAMPLE_2:
        factor = 1572864.0f;
        break;
    case SPA06_OVERSAMPLE_4:
        factor = 3670016.0f;
        break;
    case SPA06_OVERSAMPLE_8:
        factor = 7864320.0f;
        break;
    case SPA06_OVERSAMPLE_16:
        factor = 253952.0f;
        break;
    case SPA06_OVERSAMPLE_32:
        factor = 516096.0f;
        break;
    case SPA06_OVERSAMPLE_64:
        factor = 1040384.0f;
        break;
    case SPA06_OVERSAMPLE_128:
        factor = 2088960.0f;
        break;
    default:
        factor = 524288.0f;
        break;
    }

    return factor;
}

float spa06_calculate_temperature(int32_t raw_temp, struct spa06_dev *dev)
{
    float kT;
    float temp_raw_sc;
    float temp_comp;

    kT = spa06_get_scaling_factor(dev->temperature_oversampling);
    temp_raw_sc = (float)raw_temp / kT;

    /* Tcomp(degC) = c0 * 0.5 + c1 * Traw_sc */
    temp_comp = ((float)dev->coef.c0 * 0.5f) + ((float)dev->coef.c1 * temp_raw_sc);

    return temp_comp;
}

float spa06_calculate_pressure(int32_t raw_pres, int32_t raw_temp,
                                struct spa06_dev *dev)
{
    float kP;
    float kT;
    float pres_raw_sc;
    float temp_raw_sc;
    float pres_raw_sc_2;
    float pres_raw_sc_3;
    float pres_raw_sc_4;
    float pres_comp;

    kP = spa06_get_scaling_factor(dev->pressure_oversampling);
    kT = spa06_get_scaling_factor(dev->temperature_oversampling);

    pres_raw_sc = (float)raw_pres / kP;
    temp_raw_sc = (float)raw_temp / kT;

    pres_raw_sc_2 = pres_raw_sc * pres_raw_sc;
    pres_raw_sc_3 = pres_raw_sc_2 * pres_raw_sc;
    pres_raw_sc_4 = pres_raw_sc_3 * pres_raw_sc;

    /* Pcomp(Pa) = c00 + c10*Praw_sc + c20*Praw_sc^2 + c30*Praw_sc^3 +
     *             c40*Praw_sc^4 +
     *             Traw_sc*(c01 + c11*Praw_sc + c21*Praw_sc^2 + c31*Praw_sc^3) */
    pres_comp = (float)dev->coef.c00 +
                (float)dev->coef.c10 * pres_raw_sc +
                (float)dev->coef.c20 * pres_raw_sc_2 +
                (float)dev->coef.c30 * pres_raw_sc_3 +
                (float)dev->coef.c40 * pres_raw_sc_4 +
                temp_raw_sc * ((float)dev->coef.c01 +
                               (float)dev->coef.c11 * pres_raw_sc +
                               (float)dev->coef.c21 * pres_raw_sc_2 +
                               (float)dev->coef.c31 * pres_raw_sc_3);

    /* Convert Pa -> hPa. */
    return pres_comp / 100.0f;
}

/************************** Measurement data ******************************/

int8_t spa06_get_pressure_data(int32_t *raw_pres, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t buf[3];

    if ((rslt == SPA06_OK) && (raw_pres != NULL)) {
        rslt = spa06_get_regs(SPA06_REG_PSR_B2, buf, 3, dev);
        if (rslt == SPA06_OK) {
            int32_t value = (int32_t)(((uint32_t)buf[0] << 16) |
                                      ((uint32_t)buf[1] << 8) |
                                      (uint32_t)buf[2]);
            /* 24-bit 2's complement sign extension. */
            if (((uint32_t)value & 0x800000U) != 0U) {
                value |= (int32_t)0xFF000000U;
            }
            *raw_pres = value;
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_get_temperature_data(int32_t *raw_temp, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t buf[3];

    if ((rslt == SPA06_OK) && (raw_temp != NULL)) {
        rslt = spa06_get_regs(SPA06_REG_TMP_B2, buf, 3, dev);
        if (rslt == SPA06_OK) {
            int32_t value = (int32_t)(((uint32_t)buf[0] << 16) |
                                      ((uint32_t)buf[1] << 8) |
                                      (uint32_t)buf[2]);
            if (((uint32_t)value & 0x800000U) != 0U) {
                value |= (int32_t)0xFF000000U;
            }
            *raw_temp = value;
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_read_temperature(float *temperature, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    int32_t raw_temp = 0;

    if ((rslt == SPA06_OK) && (temperature != NULL)) {
        rslt = spa06_get_temperature_data(&raw_temp, dev);
        if (rslt == SPA06_OK) {
            *temperature = spa06_calculate_temperature(raw_temp, dev);
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

int8_t spa06_read_pressure(float *pressure, struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    int32_t raw_pres = 0;
    int32_t raw_temp = 0;

    if ((rslt == SPA06_OK) && (pressure != NULL)) {
        rslt = spa06_get_pressure_data(&raw_pres, dev);
        if (rslt == SPA06_OK) {
            rslt = spa06_get_temperature_data(&raw_temp, dev);
        }
        if (rslt == SPA06_OK) {
            *pressure = spa06_calculate_pressure(raw_pres, raw_temp, dev);
        }
    } else {
        rslt = SPA06_E_NULL_PTR;
    }

    return rslt;
}

/************************** Initialization *******************************/

/*!
 * @brief Wait for both coefficient-ready and sensor-ready bits, polling every
 *        ~10 ms for up to SPA06_INIT_TIMEOUT_MS.
 */
#define SPA06_INIT_TIMEOUT_MS (1000U)

int8_t spa06_init(struct spa06_dev *dev)
{
    int8_t rslt = null_ptr_check(dev);
    uint8_t chip_id = 0;
    uint8_t ready = 0;
    uint32_t elapsed_ms;

    if (rslt != SPA06_OK) {
        return rslt;
    }

    /* Verify chip ID. */
    rslt = spa06_get_chip_id(&chip_id, dev);
    if (rslt != SPA06_OK) {
        return rslt;
    }
    if (chip_id != SPA06_CHIP_ID) {
        return SPA06_E_DEV_NOT_FOUND;
    }

    /* Soft reset, then give the part a moment to settle. */
    rslt = spa06_soft_reset(dev);
    if (rslt != SPA06_OK) {
        return rslt;
    }
    dev->delay_us(10000U, dev->intf_ptr); /* ~10 ms */

    /* Poll for coefficients + sensor ready (rough 10 ms tick). */
    elapsed_ms = 0U;
    do {
        rslt = spa06_is_coeff_ready(&ready, dev);
        if (rslt != SPA06_OK) {
            return rslt;
        }
        if (ready != 0U) {
            rslt = spa06_is_sensor_ready(&ready, dev);
            if (rslt != SPA06_OK) {
                return rslt;
            }
        }
        if (ready == 0U) {
            dev->delay_us(10000U, dev->intf_ptr); /* ~10 ms */
            elapsed_ms += 10U;
        }
    } while ((ready == 0U) && (elapsed_ms < SPA06_INIT_TIMEOUT_MS));

    if (ready == 0U) {
        return SPA06_E_TIMEOUT;
    }

    /* Read calibration coefficients. */
    rslt = spa06_read_coefficients(dev);
    if (rslt != SPA06_OK) {
        return rslt;
    }

    /* High-precision default: 128x oversampling, 200 Hz, continuous both. */
    rslt = spa06_set_pressure_oversampling(SPA06_OVERSAMPLE_128, dev);
    if (rslt != SPA06_OK) {
        return rslt;
    }
    rslt = spa06_set_pressure_rate(SPA06_RATE_200, dev);
    if (rslt != SPA06_OK) {
        return rslt;
    }
    rslt = spa06_set_temperature_oversampling(SPA06_OVERSAMPLE_128, dev);
    if (rslt != SPA06_OK) {
        return rslt;
    }
    rslt = spa06_set_temperature_rate(SPA06_RATE_200, dev);
    if (rslt != SPA06_OK) {
        return rslt;
    }

    /* Enable temperature + pressure ready interrupts (mirror upstream). */
    rslt = spa06_set_interrupt_source(0U, 1U, 1U, dev);
    if (rslt != SPA06_OK) {
        return rslt;
    }

    rslt = spa06_set_measurement_mode(SPA06_MEAS_CONTINUOUS_BOTH, dev);

    return rslt;
}
