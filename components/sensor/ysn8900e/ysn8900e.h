#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"


#ifdef __cplusplus
extern "C" {
#endif

#define YSN8900E_I2C_ADDRESS       (0x32)

#define YSN8900E_SEC_REG           (0x00)
#define YSN8900E_MIN_REG           (0x01)
#define YSN8900E_HR_REG            (0x02)
#define YSN8900E_WEEKDAY_REG       (0x03)
#define YSN8900E_DAY_REG           (0x04)
#define YSN8900E_MONTH_REG         (0x05)
#define YSN8900E_YEAR_REG          (0x06)
#define YSN8900E_ALRM_MIN_REG      (0x08)
#define YSN8900E_ALRM_HR_REG       (0x09)
#define YSN8900E_ALRM_WEEK_DAY_REG (0x0A)
#define YSN8900E_TIMER_COUNTER_0   (0x0B)
#define YSN8900E_TIMER_COUNTER_1   (0x0C)
#define YSN8900E_EXTENSION_REG     (0x0D)
#define YSN8900E_FLAG_REG          (0x0E)
#define YSN8900E_CONTROL_REG       (0x0F)
#define YSN8900E_DEVICE_ID_REG     (0x20)
#define YSN8900E_CONTROL_1_REG     (0x21)

#define YSN8900E_TIMER_DEFAULT_CNT (32U)
#define YSN8900E_I2C_CLK_HZ        (400000U)

typedef esp_err_t (*ysn8900e_read_fn)(void *context, uint8_t reg, uint8_t *buffer, size_t length);
typedef esp_err_t (*ysn8900e_write_fn)(void *context, uint8_t reg, const uint8_t *buffer, size_t length);

typedef struct {
	bool available;
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t week;
	uint8_t day;
	uint8_t month;
	uint16_t year;
} ysn8900e_datetime_t;

typedef struct {
	void *context;
	ysn8900e_read_fn read;
	ysn8900e_write_fn write;
	uint8_t i2c_address;
} ysn8900e_t;

typedef struct {
	void *dev_handle;
} ysn8900e_bsp_i2c0_context_t;

esp_err_t ysn8900e_init(ysn8900e_t *dev);
esp_err_t ysn8900e_init_bsp_i2c0(ysn8900e_t *dev, ysn8900e_bsp_i2c0_context_t *context);
esp_err_t ysn8900e_deinit_bsp_i2c0(ysn8900e_t *dev, ysn8900e_bsp_i2c0_context_t *context);
esp_err_t ysn8900e_set_datetime(ysn8900e_t *dev, const ysn8900e_datetime_t *datetime);
esp_err_t ysn8900e_get_datetime(ysn8900e_t *dev, ysn8900e_datetime_t *datetime);
esp_err_t ysn8900e_enable_timer(ysn8900e_t *dev);
esp_err_t ysn8900e_disable_timer(ysn8900e_t *dev);
esp_err_t ysn8900e_reset_timer(ysn8900e_t *dev);
esp_err_t ysn8900e_reload_timer(ysn8900e_t *dev);
esp_err_t ysn8900e_get_device_id(ysn8900e_t *dev, uint8_t *device_id);


#ifdef __cplusplus
}
#endif





