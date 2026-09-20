#include <stdio.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "bmm350.h"
#include "bmm350_bsp_i2c1.h"
#include "spa06.h"
#include "spa06_bsp_i2c1.h"
#include "bsp_i2c_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/kode_lsm6dsox.h"
#include "meshpager_x2.h"
#include "sht4x_bsp_i2c1.h"
#include "ysn8900e.h"
#include "src/lsm6dsox_reg.h"

static const char *TAG = "SENSOR_TEST";

#define SENSOR_READY_TIMEOUT_MS 2000U
#define SENSOR_READY_POLL_MS    10U
#define SPA06_STARTUP_SETTLE_MS 300U
#define LSM6DSOX_GYRO_SETTLE_MS 100U

static ysn8900e_t s_sensor_dev;
static ysn8900e_bsp_i2c0_context_t s_sensor_ctx;
static bool s_sensor_ready;

static sht4x_bsp_i2c1_t s_sht4x_dev;
static sht4x_bsp_i2c1_context_t s_sht4x_ctx;
static bool s_sht4x_ready;

static struct bmm350_dev s_bmm350_dev;
static bmm350_bsp_i2c1_context_t s_bmm350_ctx;
static bool s_bmm350_ready;

static struct spa06_dev s_spa06_dev;
static spa06_bsp_i2c1_context_t s_spa06_ctx;
static bool s_spa06_ready;
static bool s_spa06_sample_warmed;

static kode_lsm6dsox_handle_t s_lsm6dsox_handle;
static bool s_lsm6dsox_ready;
static bool s_lsm6dsox_gyro_warmed;

static struct {
    struct arg_end *end;
} sensor_id_args;

static struct {
    struct arg_end *end;
} sensor_time_get_args;

static struct {
    struct arg_end *end;
} sht4x_serial_args;

static struct {
    struct arg_end *end;
} sht4x_read_args;

static struct {
    struct arg_end *end;
} bmm350_id_args;

static struct {
    struct arg_end *end;
} bmm350_read_args;

static struct {
    struct arg_end *end;
} bmm350_reset_args;

static struct {
    struct arg_end *end;
} spa06_id_args;

static struct {
    struct arg_end *end;
} spa06_read_args;

static struct {
    struct arg_end *end;
} lsm6dsox_id_args;

static struct {
    struct arg_end *end;
} lsm6dsox_read_args;

static struct {
    struct arg_end *end;
} battery_get_args;

static struct {
    struct arg_int *year;
    struct arg_int *month;
    struct arg_int *day;
    struct arg_int *week;
    struct arg_int *hour;
    struct arg_int *minute;
    struct arg_int *second;
    struct arg_end *end;
} sensor_time_set_args;

static esp_err_t sensor_init_device(void)
{
    esp_err_t ret;

    if (s_sensor_ready) {
        return ESP_OK;
    }

    memset(&s_sensor_dev, 0, sizeof(s_sensor_dev));
    memset(&s_sensor_ctx, 0, sizeof(s_sensor_ctx));

    ret = ysn8900e_init_bsp_i2c0(&s_sensor_dev, &s_sensor_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to bind YSN8900E to bsp_i2c_0: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ysn8900e_init(&s_sensor_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize YSN8900E: %s", esp_err_to_name(ret));
        ysn8900e_deinit_bsp_i2c0(&s_sensor_dev, &s_sensor_ctx);
        return ret;
    }

    s_sensor_ready = true;
    ESP_LOGI(TAG, "YSN8900E initialized");
    return ESP_OK;
}

static esp_err_t sensor_require_ready(void)
{
    esp_err_t ret = sensor_init_device();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor is not ready");
    }

    return ret;
}

static esp_err_t sht4x_init_device(void)
{
    esp_err_t ret;

    if (s_sht4x_ready) {
        return ESP_OK;
    }

    memset(&s_sht4x_dev, 0, sizeof(s_sht4x_dev));
    memset(&s_sht4x_ctx, 0, sizeof(s_sht4x_ctx));

    ret = sht4x_bsp_i2c1_init(&s_sht4x_dev, &s_sht4x_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SHT4X on bsp_i2c_1: %s", esp_err_to_name(ret));
        return ret;
    }

    s_sht4x_ready = true;
    ESP_LOGI(TAG, "SHT4X initialized");
    return ESP_OK;
}

static esp_err_t sht4x_require_ready(void)
{
    esp_err_t ret = sht4x_init_device();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT4X is not ready");
    }

    return ret;
}

/* Re-apply the BMM350 runtime configuration (power mode, ODR/performance,
 * enabled axes). Used both during initial setup and after a soft reset: a
 * reset restores the default register values (suspend mode, default ODR,
 * axes disabled), so without this the sensor returns no valid data. */
static esp_err_t bmm350_apply_config(void)
{
    int8_t rslt;

    rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &s_bmm350_dev);
    if (rslt != BMM350_OK) {
        ESP_LOGE(TAG, "Failed to set BMM350 power mode: %d", rslt);
        return ESP_FAIL;
    }

    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_LOWNOISE, &s_bmm350_dev);
    if (rslt != BMM350_OK) {
        ESP_LOGE(TAG, "Failed to configure BMM350 ODR/performance: %d", rslt);
        return ESP_FAIL;
    }

    rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &s_bmm350_dev);
    if (rslt != BMM350_OK) {
        ESP_LOGE(TAG, "Failed to enable BMM350 axes: %d", rslt);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t bmm350_init_device(void)
{
    esp_err_t ret;
    int8_t rslt;

    if (s_bmm350_ready) {
        return ESP_OK;
    }

    memset(&s_bmm350_dev, 0, sizeof(s_bmm350_dev));
    memset(&s_bmm350_ctx, 0, sizeof(s_bmm350_ctx));

    ret = bmm350_init_bsp_i2c1(&s_bmm350_dev, &s_bmm350_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to bind BMM350 to bsp_i2c_1: %s", esp_err_to_name(ret));
        return ret;
    }

    rslt = bmm350_init(&s_bmm350_dev);
    if (rslt != BMM350_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMM350: %d", rslt);
        bmm350_deinit_bsp_i2c1(&s_bmm350_dev, &s_bmm350_ctx);
        return ESP_FAIL;
    }

    ret = bmm350_apply_config();
    if (ret != ESP_OK) {
        bmm350_deinit_bsp_i2c1(&s_bmm350_dev, &s_bmm350_ctx);
        return ret;
    }

    s_bmm350_ready = true;
    ESP_LOGI(TAG, "BMM350 initialized");
    return ESP_OK;
}

static esp_err_t bmm350_require_ready(void)
{
    esp_err_t ret = bmm350_init_device();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMM350 is not ready");
    }

    return ret;
}

static esp_err_t spa06_wait_for_sample(void)
{
    float pressure;
    float temperature;

    if (s_spa06_sample_warmed) {
        return ESP_OK;
    }

    /* The SPA06 ready bits are not reliable for this continuous-mode setup.
     * Let the first high-oversampling conversion complete, then discard that
     * startup result before exposing a sample to the test operator. */
    vTaskDelay(pdMS_TO_TICKS(SPA06_STARTUP_SETTLE_MS));

    int8_t rslt = spa06_read_pressure(&pressure, &s_spa06_dev);
    if (rslt != SPA06_OK) {
        ESP_LOGE(TAG, "Failed to discard initial SPA06 pressure sample: %d", rslt);
        return ESP_FAIL;
    }

    rslt = spa06_read_temperature(&temperature, &s_spa06_dev);
    if (rslt != SPA06_OK) {
        ESP_LOGE(TAG, "Failed to discard initial SPA06 temperature sample: %d", rslt);
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(SPA06_STARTUP_SETTLE_MS));
    s_spa06_sample_warmed = true;
    return ESP_OK;
}

/* SPA06-003 pressure & temperature sensor shares I2C_1 with the other sensors. */
static esp_err_t spa06_init_device(void)
{
    esp_err_t ret;
    int8_t rslt;

    if (s_spa06_ready) {
        return ESP_OK;
    }

    memset(&s_spa06_dev, 0, sizeof(s_spa06_dev));
    memset(&s_spa06_ctx, 0, sizeof(s_spa06_ctx));
    s_spa06_sample_warmed = false;

    ret = spa06_init_bsp_i2c1(&s_spa06_dev, &s_spa06_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to bind SPA06 to bsp_i2c_1: %s", esp_err_to_name(ret));
        return ret;
    }

    rslt = spa06_init(&s_spa06_dev);
    if (rslt != SPA06_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPA06: %d", rslt);
        spa06_deinit_bsp_i2c1(&s_spa06_dev, &s_spa06_ctx);
        return ESP_FAIL;
    }

    s_spa06_ready = true;
    ESP_LOGI(TAG, "SPA06 initialized");
    return ESP_OK;
}

static esp_err_t spa06_require_ready(void)
{
    esp_err_t ret = spa06_init_device();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPA06 is not ready");
    }

    return ret;
}

/* LSM6DSOX (6-axis IMU) shares I2C_1 with BMM350 / SHT4X / audio codecs. */
static esp_err_t lsm6dsox_init_device(void)
{
    esp_err_t ret;
    /* The LSM6DSOX SDA/SA0 strap selects the address: 0x6A (SA0=0) or 0x6B
     * (SA0=1). Probe both so the command works regardless of board strapping. */
    static const uint8_t addr_candidates[] = { 0x6A, 0x6B };

    if (s_lsm6dsox_ready) {
        return ESP_OK;
    }

    s_lsm6dsox_gyro_warmed = false;
    memset(&s_lsm6dsox_handle, 0, sizeof(s_lsm6dsox_handle));

    i2c_master_bus_handle_t bus = bsp_i2c_1_get_handle();
    if (bus == NULL) {
        ESP_LOGE(TAG, "Failed to init LSM6DSOX: I2C_1 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    bool probed = false;
    for (size_t i = 0; i < sizeof(addr_candidates) / sizeof(addr_candidates[0]); i++) {
        uint8_t addr = addr_candidates[i];
        ret = kode_lsm6dsox_init_i2c(bus, addr, &s_lsm6dsox_handle);
        if (ret != ESP_OK) {
            kode_lsm6dsox_deinit_i2c(&s_lsm6dsox_handle);
            continue;
        }
        uint8_t id = 0;
        if (kode_lsm6dsox_check_id(&s_lsm6dsox_handle, &id) == ESP_OK && id == 0x6C) {
            probed = true;
            ESP_LOGI(TAG, "LSM6DSOX found on I2C_1 @0x%02X (id=0x%02X)", addr, id);
            break;
        }
    }

    if (!probed) {
        ESP_LOGE(TAG, "No LSM6DSOX (id 0x6C) found on I2C_1 @0x6A/0x6B");
        return ESP_ERR_NOT_FOUND;
    }

    // ret = kode_lsm6dsox_config_default(&s_lsm6dsox_handle);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to configure LSM6DSOX: %s", esp_err_to_name(ret));
    //     return ret;
    // }

    // Higher rate for more responsive interrupts
    ret = kode_lsm6dsox_accel_config(&s_lsm6dsox_handle, LSM6DSOX_XL_ODR_417Hz, LSM6DSOX_2g);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LSM6DSOX accelerometer: %s", esp_err_to_name(ret));
        kode_lsm6dsox_deinit_i2c(&s_lsm6dsox_handle);
        return ret;
    }

    ret = kode_lsm6dsox_gyro_config(&s_lsm6dsox_handle, LSM6DSOX_GY_ODR_417Hz, LSM6DSOX_2000dps);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LSM6DSOX gyroscope: %s", esp_err_to_name(ret));
        kode_lsm6dsox_deinit_i2c(&s_lsm6dsox_handle);
        return ret;
    }

    // Configure interrupts - active high, push-pull
    ret = kode_lsm6dsox_interrupt_pin_config(&s_lsm6dsox_handle, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LSM6DSOX interrupt pins: %s", esp_err_to_name(ret));
        kode_lsm6dsox_deinit_i2c(&s_lsm6dsox_handle);
        return ret;
    }

    // Configure INT1 for data ready interrupts (accelerometer)
    ret = kode_lsm6dsox_interrupt_enable_int1(&s_lsm6dsox_handle, KODE_LSM6DSOX_INT_DRDY_XL|KODE_LSM6DSOX_INT_DRDY_G);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LSM6DSOX data-ready interrupt: %s", esp_err_to_name(ret));
        kode_lsm6dsox_deinit_i2c(&s_lsm6dsox_handle);
        return ret;
    }

    // 
    // lsm6dsox_data_ready_mode_set(&s_lsm6dsox_handle.ctx, LSM6DSOX_DRDY_PULSED); // LSM6DSOX_DRDY_LATCHED

    s_lsm6dsox_ready = true;
    return ESP_OK;
}

static esp_err_t lsm6dsox_require_ready(void)
{
    esp_err_t ret = lsm6dsox_init_device();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LSM6DSOX is not ready");
    }

    return ret;
}

static esp_err_t lsm6dsox_wait_for_sample(void)
{
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < SENSOR_READY_TIMEOUT_MS) {
        uint8_t accel_ready = 0;
        uint8_t gyro_ready = 0;
        esp_err_t ret = kode_lsm6dsox_accel_data_ready(&s_lsm6dsox_handle, &accel_ready);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to check LSM6DSOX accelerometer readiness: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = kode_lsm6dsox_gyro_data_ready(&s_lsm6dsox_handle, &gyro_ready);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to check LSM6DSOX gyroscope readiness: %s", esp_err_to_name(ret));
            return ret;
        }

        if (accel_ready != 0U && gyro_ready != 0U) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READY_POLL_MS));
        elapsed_ms += SENSOR_READY_POLL_MS;
    }

    ESP_LOGE(TAG, "Timed out waiting for LSM6DSOX measurement");
    return ESP_ERR_TIMEOUT;
}

static esp_err_t lsm6dsox_wait_for_gyro_settle(void)
{
    uint32_t elapsed_ms = 0;

    while (elapsed_ms < LSM6DSOX_GYRO_SETTLE_MS) {
        uint8_t gyro_ready = 0;
        esp_err_t ret = kode_lsm6dsox_gyro_data_ready(&s_lsm6dsox_handle, &gyro_ready);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to check LSM6DSOX gyroscope readiness: %s", esp_err_to_name(ret));
            return ret;
        }

        if (gyro_ready != 0U) {
            float discard_x;
            float discard_y;
            float discard_z;
            ret = kode_lsm6dsox_read_gyro(&s_lsm6dsox_handle, &discard_x, &discard_y, &discard_z);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to discard settling LSM6DSOX gyro sample: %s", esp_err_to_name(ret));
                return ret;
            }
            vTaskDelay(pdMS_TO_TICKS(3));
            elapsed_ms += 3U;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READY_POLL_MS));
        elapsed_ms += SENSOR_READY_POLL_MS;
    }

    return ESP_OK;
}

static bool sensor_datetime_valid(const ysn8900e_datetime_t *datetime)
{
    if (datetime->year < 2000 || datetime->year > 2099) {
        return false;
    }

    if (datetime->month < 1 || datetime->month > 12) {
        return false;
    }

    if (datetime->day < 1 || datetime->day > 31) {
        return false;
    }

    if (datetime->week < 1 || datetime->week > 7) {
        return false;
    }

    if (datetime->hour > 23 || datetime->minute > 59 || datetime->second > 59) {
        return false;
    }

    return true;
}

static void sensor_log_datetime(const ysn8900e_datetime_t *datetime)
{
    ESP_LOGI(TAG,
        "Time: %04u-%02u-%02u week=%u %02u:%02u:%02u available=%s",
        datetime->year,
        datetime->month,
        datetime->day,
        datetime->week,
        datetime->hour,
        datetime->minute,
        datetime->second,
        datetime->available ? "true" : "false");  
}

static uint8_t battery_voltage_to_percent(uint16_t voltage_mv)
{
    const uint16_t empty_mv = 3300;
    const uint16_t full_mv = 4200;

    if (voltage_mv <= empty_mv) {
        return 0;
    }

    if (voltage_mv >= full_mv) {
        return 100;
    }

    return (uint8_t)(((uint32_t)(voltage_mv - empty_mv) * 100U) / (full_mv - empty_mv));
}

static uint16_t battery_read_with_enable_level(uint8_t level)
{
    bsp_exp_output_io_set_level(BSP_BAT_ADC_EN, level);
    vTaskDelay(pdMS_TO_TICKS(50));
    return bsp_battery_voltage_read();
}

static int sensor_id_cmd(int argc, char **argv)
{
    uint8_t device_id = 0;
    int nerrors = arg_parse(argc, argv, (void **)&sensor_id_args);
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, sensor_id_args.end, argv[0]);
        return 1;
    }

    ret = sensor_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = ysn8900e_get_device_id(&s_sensor_dev, &device_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device id: %s", esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG, "YSN8900E device id: 0x%02X", device_id);
    return 0;
}

static int sensor_time_get_cmd(int argc, char **argv)
{
    ysn8900e_datetime_t datetime = { 0 };
    int nerrors = arg_parse(argc, argv, (void **)&sensor_time_get_args);
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, sensor_time_get_args.end, argv[0]);
        return 1;
    }

    ret = sensor_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = ysn8900e_get_datetime(&s_sensor_dev, &datetime);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read time: %s", esp_err_to_name(ret));
        return 1;
    }
    uint32_t rtc_int_count = get_rtc_int_count();
    ESP_LOGI(TAG, "rtc_int_count: %u", rtc_int_count);
    
    sensor_log_datetime(&datetime);
    return 0;
}

static int sensor_time_set_cmd(int argc, char **argv)
{
    ysn8900e_datetime_t datetime = { 0 };
    int nerrors = arg_parse(argc, argv, (void **)&sensor_time_set_args);
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, sensor_time_set_args.end, argv[0]);
        return 1;
    }

    datetime.year = (uint16_t)sensor_time_set_args.year->ival[0];
    datetime.month = (uint8_t)sensor_time_set_args.month->ival[0];
    datetime.day = (uint8_t)sensor_time_set_args.day->ival[0];
    datetime.week = (uint8_t)sensor_time_set_args.week->ival[0];
    datetime.hour = (uint8_t)sensor_time_set_args.hour->ival[0];
    datetime.minute = (uint8_t)sensor_time_set_args.minute->ival[0];
    datetime.second = (uint8_t)sensor_time_set_args.second->ival[0];
    datetime.available = true;

    if (!sensor_datetime_valid(&datetime)) {
        ESP_LOGE(TAG, "Invalid datetime, expected year=2000-2099 month=1-12 day=1-31 week=1-7 hour=0-23 minute=0-59 second=0-59");
        return 1;
    }

    ret = sensor_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = ysn8900e_set_datetime(&s_sensor_dev, &datetime);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write time: %s", esp_err_to_name(ret));
        return 1;
    }

    sensor_log_datetime(&datetime);
    return 0;
}

static int sht4x_serial_cmd(int argc, char **argv)
{
    uint32_t serial = 0;
    int nerrors = arg_parse(argc, argv, (void **)&sht4x_serial_args);
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, sht4x_serial_args.end, argv[0]);
        return 1;
    }

    ret = sht4x_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = sht4x_bsp_i2c1_get_serial(&s_sht4x_dev, &serial);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SHT4X serial: %s", esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG, "SHT4X serial: 0x%08" PRIX32, serial);
    return 0;
}

static int sht4x_read_cmd(int argc, char **argv)
{
    float temperature = 0.0f;
    float humidity = 0.0f;
    int nerrors = arg_parse(argc, argv, (void **)&sht4x_read_args);
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, sht4x_read_args.end, argv[0]);
        return 1;
    }

    ret = sht4x_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = sht4x_bsp_i2c1_measure(&s_sht4x_dev, &temperature, &humidity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SHT4X: %s", esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG, "SHT4X temperature: %.2f C, humidity: %.2f %%", temperature, humidity);
    return 0;
}

static int bmm350_id_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&bmm350_id_args);
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, bmm350_id_args.end, argv[0]);
        return 1;
    }

    ret = bmm350_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ESP_LOGI(TAG, "BMM350 chip id: 0x%02X", s_bmm350_dev.chip_id);
    return 0;
}

static int bmm350_read_cmd(int argc, char **argv)
{
    struct bmm350_mag_temp_data mag_data = { 0 };
    int8_t rslt;
    int nerrors = arg_parse(argc, argv, (void **)&bmm350_read_args);
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, bmm350_read_args.end, argv[0]);
        return 1;
    }

    ret = bmm350_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_data, &s_bmm350_dev);
    if (rslt != BMM350_OK) {
        ESP_LOGE(TAG, "Failed to read BMM350: %d", rslt);
        return 1;
    }

    ESP_LOGI(TAG,
        "BMM350 x=%.2f uT y=%.2f uT z=%.2f uT temp=%.2f C",
        mag_data.x,
        mag_data.y,
        mag_data.z,
        mag_data.temperature);
    return 0;
}

static int bmm350_reset_cmd(int argc, char **argv)
{
    int8_t rslt;
    int nerrors = arg_parse(argc, argv, (void **)&bmm350_reset_args);
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, bmm350_reset_args.end, argv[0]);
        return 1;
    }

    ret = bmm350_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    rslt = bmm350_soft_reset(&s_bmm350_dev);
    if (rslt != BMM350_OK) {
        ESP_LOGE(TAG, "Failed to soft-reset BMM350: %d", rslt);
        return 1;
    }

    /* Soft reset restores default register values; re-apply the runtime
     * configuration so subsequent bmm350_id / bmm350_read keep working. */
    ret = bmm350_apply_config();
    if (ret != ESP_OK) {
        return 1;
    }

    ESP_LOGI(TAG, "BMM350 soft-reset and reconfigured");
    return 0;
}

static int spa06_id_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&spa06_id_args);
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, spa06_id_args.end, argv[0]);
        return 1;
    }

    ret = spa06_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ESP_LOGI(TAG, "SPA06 chip id: 0x%02X", s_spa06_dev.chip_id);
    return 0;
}

static int spa06_read_cmd(int argc, char **argv)
{
    float pressure = 0.0f;
    float temperature = 0.0f;
    int8_t rslt;
    int nerrors = arg_parse(argc, argv, (void **)&spa06_read_args);
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, spa06_read_args.end, argv[0]);
        return 1;
    }

    ret = spa06_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = spa06_wait_for_sample();
    if (ret != ESP_OK) {
        return 1;
    }

    rslt = spa06_read_pressure(&pressure, &s_spa06_dev);
    if (rslt != SPA06_OK) {
        ESP_LOGE(TAG, "Failed to read SPA06 pressure: %d", rslt);
        return 1;
    }

    rslt = spa06_read_temperature(&temperature, &s_spa06_dev);
    if (rslt != SPA06_OK) {
        ESP_LOGE(TAG, "Failed to read SPA06 temperature: %d", rslt);
        return 1;
    }

    ESP_LOGI(TAG, "SPA06 pressure=%.2f hPa temp=%.2f C", pressure, temperature);
    return 0;
}

static int battery_get_cmd(int argc, char **argv)
{
    uint16_t voltage_level0_mv;
    uint16_t voltage_level1_mv;
    uint16_t voltage_mv;
    int nerrors = arg_parse(argc, argv, (void **)&battery_get_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, battery_get_args.end, argv[0]);
        return 1;
    }

    voltage_level0_mv = battery_read_with_enable_level(0);
    voltage_level1_mv = battery_read_with_enable_level(1);
    bsp_bat_power_control(false);

    ESP_LOGI(TAG, "Battery probe: BAT_ADC_EN=0 -> %u mV, BAT_ADC_EN=1 -> %u mV", voltage_level0_mv, voltage_level1_mv);

    voltage_mv = (voltage_level0_mv > voltage_level1_mv) ? voltage_level0_mv : voltage_level1_mv;

    ESP_LOGI(TAG, "Battery voltage: %u mV, estimated level: %u%%", voltage_mv, battery_voltage_to_percent(voltage_mv));
    return 0;
}

static int lsm6dsox_id_cmd(int argc, char **argv)
{
    uint8_t id = 0;
    int nerrors = arg_parse(argc, argv, (void **)&lsm6dsox_id_args);
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, lsm6dsox_id_args.end, argv[0]);
        return 1;
    }

    ret = lsm6dsox_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = kode_lsm6dsox_check_id(&s_lsm6dsox_handle, &id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read LSM6DSOX id: %s", esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG, "LSM6DSOX device id: 0x%02X (expect 0x6C)", id);
    return 0;
}

static int lsm6dsox_read_cmd(int argc, char **argv)
{
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float temp_c = 0.0f;
    int nerrors = arg_parse(argc, argv, (void **)&lsm6dsox_read_args);
    esp_err_t ret;

    if (nerrors != 0) {
        arg_print_errors(stderr, lsm6dsox_read_args.end, argv[0]);
        return 1;
    }

    ret = lsm6dsox_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = lsm6dsox_wait_for_sample();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = kode_lsm6dsox_read_accel(&s_lsm6dsox_handle, &ax, &ay, &az);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read LSM6DSOX accel: %s", esp_err_to_name(ret));
        return 1;
    }

    ret = kode_lsm6dsox_read_gyro(&s_lsm6dsox_handle, &gx, &gy, &gz);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read LSM6DSOX gyro: %s", esp_err_to_name(ret));
        return 1;
    }

    if (!s_lsm6dsox_gyro_warmed) {
        /* The first gyro result after enabling the sensor can contain a
         * startup transient. Discard fresh samples for a bounded settling
         * interval before using the next result. */
        ret = lsm6dsox_wait_for_gyro_settle();
        if (ret != ESP_OK) {
            return 1;
        }
        s_lsm6dsox_gyro_warmed = true;

        ret = lsm6dsox_wait_for_sample();
        if (ret != ESP_OK) {
            return 1;
        }

        ret = kode_lsm6dsox_read_gyro(&s_lsm6dsox_handle, &gx, &gy, &gz);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read settled LSM6DSOX gyro: %s", esp_err_to_name(ret));
            return 1;
        }
    }

    ret = kode_lsm6dsox_read_temp(&s_lsm6dsox_handle, &temp_c);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read LSM6DSOX temperature: %s", esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG,
             "LSM6DSOX accel X=%.1f Y=%.1f Z=%.1f mg, gyro X=%.1f Y=%.1f Z=%.1f mdps, temp=%.2f C",
             ax, ay, az, gx, gy, gz, temp_c);
    return 0;
}

static void register_sensor_id_cmd(void)
{
    sensor_id_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "sensor_id",
        .help = "Read YSN8900E device id",
        .hint = NULL,
        .func = &sensor_id_cmd,
        .argtable = &sensor_id_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_sensor_time_get_cmd(void)
{
    sensor_time_get_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "sensor_time_get",
        .help = "Read YSN8900E datetime",
        .hint = NULL,
        .func = &sensor_time_get_cmd,
        .argtable = &sensor_time_get_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_sensor_time_set_cmd(void)
{
    sensor_time_set_args.year = arg_int1("y", "year", "<2000-2099>", "Year");
    sensor_time_set_args.month = arg_int1("M", "month", "<1-12>", "Month");
    sensor_time_set_args.day = arg_int1("d", "day", "<1-31>", "Day");
    sensor_time_set_args.week = arg_int1("w", "week", "<1-7>", "Week day");
    sensor_time_set_args.hour = arg_int1("H", "hour", "<0-23>", "Hour");
    sensor_time_set_args.minute = arg_int1("m", "minute", "<0-59>", "Minute");
    sensor_time_set_args.second = arg_int1("s", "second", "<0-59>", "Second");
    sensor_time_set_args.end = arg_end(7);

    const esp_console_cmd_t cmd = {
        .command = "sensor_time_set",
        .help = "Write YSN8900E datetime",
        .hint = NULL,
        .func = &sensor_time_set_cmd,
        .argtable = &sensor_time_set_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_sht4x_serial_cmd(void)
{
    sht4x_serial_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "sht4x_serial",
        .help = "Read SHT4X serial number",
        .hint = NULL,
        .func = &sht4x_serial_cmd,
        .argtable = &sht4x_serial_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_sht4x_read_cmd(void)
{
    sht4x_read_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "sht4x_read",
        .help = "Read SHT4X temperature and humidity",
        .hint = NULL,
        .func = &sht4x_read_cmd,
        .argtable = &sht4x_read_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_bmm350_id_cmd(void)
{
    bmm350_id_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "bmm350_id",
        .help = "Read BMM350 chip id",
        .hint = NULL,
        .func = &bmm350_id_cmd,
        .argtable = &bmm350_id_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_bmm350_read_cmd(void)
{
    bmm350_read_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "bmm350_read",
        .help = "Read BMM350 magnetic field and temperature",
        .hint = NULL,
        .func = &bmm350_read_cmd,
        .argtable = &bmm350_read_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_bmm350_reset_cmd(void)
{
    bmm350_reset_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "bmm350_reset",
        .help = "Soft-reset BMM350 and re-apply its configuration",
        .hint = NULL,
        .func = &bmm350_reset_cmd,
        .argtable = &bmm350_reset_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_spa06_id_cmd(void)
{
    spa06_id_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "spa06_id",
        .help = "Read SPA06-003 chip id (expect 0x11)",
        .hint = NULL,
        .func = &spa06_id_cmd,
        .argtable = &spa06_id_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_spa06_read_cmd(void)
{
    spa06_read_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "spa06_read",
        .help = "Read SPA06-003 pressure (hPa) and temperature (C)",
        .hint = NULL,
        .func = &spa06_read_cmd,
        .argtable = &spa06_read_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_battery_get_cmd(void)
{
    battery_get_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "battery_get",
        .help = "Read battery voltage and estimate battery level",
        .hint = NULL,
        .func = &battery_get_cmd,
        .argtable = &battery_get_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_lsm6dsox_id_cmd(void)
{
    lsm6dsox_id_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "lsm6dsox_id",
        .help = "Read LSM6DSOX (IMU) device id (expect 0x6C)",
        .hint = NULL,
        .func = &lsm6dsox_id_cmd,
        .argtable = &lsm6dsox_id_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_lsm6dsox_read_cmd(void)
{
    lsm6dsox_read_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "lsm6dsox_read",
        .help = "Read LSM6DSOX accel (mg) / gyro (mdps) / temperature",
        .hint = NULL,
        .func = &lsm6dsox_read_cmd,
        .argtable = &lsm6dsox_read_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void sensor_cmd_register_all(void)
{
    register_sensor_id_cmd();
    register_sensor_time_get_cmd();
    register_sensor_time_set_cmd();
    register_sht4x_serial_cmd();
    register_sht4x_read_cmd();
    register_bmm350_id_cmd();
    register_bmm350_read_cmd();
    register_bmm350_reset_cmd();
    register_spa06_id_cmd();
    register_spa06_read_cmd();
    register_battery_get_cmd();
    register_lsm6dsox_id_cmd();
    register_lsm6dsox_read_cmd();

    ESP_LOGI(TAG, "Sensor test commands registered:");
    ESP_LOGI(TAG, "  sensor_id");
    ESP_LOGI(TAG, "  sensor_time_get");
    ESP_LOGI(TAG, "  sensor_time_set -y <year> -M <month> -d <day> -w <week> -H <hour> -m <minute> -s <second>");
    ESP_LOGI(TAG, "  sht4x_serial");
    ESP_LOGI(TAG, "  sht4x_read");
    ESP_LOGI(TAG, "  bmm350_id");
    ESP_LOGI(TAG, "  bmm350_read");
    ESP_LOGI(TAG, "  bmm350_reset");
    ESP_LOGI(TAG, "  spa06_id");
    ESP_LOGI(TAG, "  spa06_read");
    ESP_LOGI(TAG, "  battery_get");
    ESP_LOGI(TAG, "  lsm6dsox_id");
    ESP_LOGI(TAG, "  lsm6dsox_read");
}