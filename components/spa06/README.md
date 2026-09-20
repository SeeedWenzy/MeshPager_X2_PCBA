# SPA06-003 Pressure & Temperature Sensor Driver (C, ESP-IDF)

C port of the Adafruit `Adafruit_SPA06_003` Arduino driver, restructured to
follow the same portable-component layout used by the Bosch **BMM350** driver
in this project (`components/bmm350`).

The compensation math, register map, calibration-coefficient parsing and the
compensation scale-factor table (Table 4) are taken from the SPA06-003
datasheet (Goermicro, Ver2.0) and verified against it. The chip ID (`0x11`),
soft-reset command (`0x09`), and all field bit positions match the datasheet.

Original Arduino driver: Copyright (c) Adafruit Industries — MIT license.

## Layout

```
components/spa06/
├── include/
│   ├── spa06.h        # public API
│   ├── spa06_defs.h   # registers, enums, INTF types, calib/dev structs
│   └── common.h       # ESP-i2cdev bridge prototypes
├── spa06.c            # interface-agnostic driver core (no platform code)
├── common.c           # ESP-IDF i2cdev bridge (standalone port/GPIO path)
├── CMakeLists.txt
└── idf_component.yml
```

The core (`spa06.c`) talks to the bus only through the `read`/`write`/`delay_us`
callbacks stored in a `struct spa06_dev`. Two ready-made bridges wire those
callbacks for ESP32-S3:

| Bridge | File | Use when |
|--------|------|----------|
| Project-shared I2C_1 bus | `components/sensor/spa06/spa06_bsp_i2c1.c` | You are inside this firmware (meshpager) and want the sensor on the shared `bsp_i2c_1` bus alongside SHT4x / BMM350 / LSM6DSOx. |
| Standalone i2cdev | `components/spa06/common.c` | You want a self-contained sensor on its own I2C port + GPIOs (no meshpager BSP). |

## Quick start — on the shared I2C_1 bus (recommended in this project)

Mirrors how BMM350 is used in `examples/test_pcba/main/sensor_cmd.c`:

```c
#include "spa06.h"
#include "spa06_bsp_i2c1.h"

static struct spa06_dev s_spa06_dev;
static spa06_bsp_i2c1_context_t s_spa06_ctx;

static esp_err_t spa06_init_device(void)
{
    esp_err_t ret;
    int8_t rslt;

    memset(&s_spa06_dev, 0, sizeof(s_spa06_dev));
    memset(&s_spa06_ctx, 0, sizeof(s_spa06_ctx));

    ret = spa06_init_bsp_i2c1(&s_spa06_dev, &s_spa06_ctx);   /* wire bus callbacks */
    if (ret != ESP_OK) {
        return ret;
    }

    rslt = spa06_init(&s_spa06_dev);                         /* id check, reset, coef, config */
    if (rslt != SPA06_OK) {
        spa06_deinit_bsp_i2c1(&s_spa06_dev, &s_spa06_ctx);
        return ESP_FAIL;
    }

    return ESP_OK;
}

void spa06_sample(float *pressure_hpa, float *temperature_c)
{
    spa06_read_pressure(&s_spa06_dev, pressure_hpa);     /* hPa */
    spa06_read_temperature(&s_spa06_dev, temperature_c); /* degC */
}
```

## Quick start — standalone i2cdev (own port/GPIOs)

```c
#include "spa06.h"
#include "common.h"

static struct spa06_dev dev;

void spa06_task(void)
{
    int8_t rslt = spa06_esp_init(&dev, I2C_NUM_0,
                                 GPIO_NUM_8, GPIO_NUM_9,   /* SDA, SCL */
                                 0 /* default addr 0x77 */);
    if (rslt != SPA06_OK) { spa06_error_codes_print_result("spa06_esp_init", rslt); return; }

    rslt = spa06_init(&dev);
    if (rslt != SPA06_OK) { spa06_error_codes_print_result("spa06_init", rslt); return; }

    float pressure, temperature;
    spa06_read_pressure(&pressure, &dev);
    spa06_read_temperature(&temperature, &dev);

    printf("SPA06: %.2f hPa, %.2f C\n", pressure, temperature);

    spa06_esp_deinit(&dev);
}
```

## Default configuration applied by `spa06_init()`

Same high-precision defaults as the upstream Adafruit driver:

- Pressure oversampling 128x, measurement rate 200 Hz
- Temperature oversampling 128x, measurement rate 200 Hz
- Pressure/temperature result shift bits set automatically (> 8x oversampling)
- Temperature + pressure ready interrupts enabled
- Continuous pressure & temperature background measurement

To change settings afterwards, use `spa06_set_pressure_oversampling()`,
`spa06_set_pressure_rate()`, `spa06_set_measurement_mode()`, etc. Note that
the oversampling getters cache the value in `spa06_dev` — always set
oversampling through the API (not raw register writes) so the compensation
scaling factor stays in sync.

## Wiring

| Sensor pin | ESP32-S3 |
|------------|----------|
| VDD        | 3.3 V    |
| GND        | GND      |
| SDA        | your SDA GPIO |
| SCL        | your SCL GPIO |
| SDO        | GND -> addr 0x77 ; VDDIO -> addr 0x76 |

## Error codes

| Code | Meaning |
|------|---------|
| `SPA06_OK` | Success |
| `SPA06_E_NULL_PTR` | NULL pointer / missing callback |
| `SPA06_E_COM_FAIL` | I2C communication failure |
| `SPA06_E_DEV_NOT_FOUND` | Chip ID not 0x11 |
| `SPA06_E_COEF_NOT_READY` | Calibration coefficients not ready |
| `SPA06_E_TIMEOUT` | Timed out waiting for readiness after reset |
| `SPA06_E_INVALID_CONFIG` / `SPA06_E_INVALID_INPUT` | Bad argument |

Use `spa06_error_codes_print_result("api", rslt)` to log them.
