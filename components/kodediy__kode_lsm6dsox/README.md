# LSM6DSOX 6-AXIS IMU

[![Component Registry](https://components.espressif.com/components/kodediy/kode_lsm6dsox/badge.svg)](https://components.espressif.com/components/kodediy/kode_lsm6dsox)

The LSM6DSOX 6-axis IMU is a 3D accelerometer and 3D gyroscope sensor. This component is based on the STM32 driver and is compatible with ESP-IDF.

| IMU | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| LSM6DSOX         | I2C                     | kode_lsm6dsox      | [PDF](https://github.com/kodediy/kode_lsm6dsox/blob/main/LSM6DSOX_Datasheet_Rev4.pdf) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g.
```
    idf.py add-dependency kode_bq25896==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Features

- I2C communication interface (SPI to be added in future)
- Configurable accelerometer and gyroscope parameters (ODR, full-scale)
- Temperature sensor reading
- FIFO support with configurable watermark
- Complete interrupt handling for INT1 and INT2 pins
- Power management for different power modes
- Comprehensive examples demonstrating different features

## Usage

### Basic Usage

```c
#include "kode_lsm6dsox.h"

// Initialize I2C
i2c_master_bus_handle_t i2c_bus = NULL;
i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = CONFIG_I2C_SCL_PIN,
    .sda_io_num = CONFIG_I2C_SDA_PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));

// Initialize LSM6DSOX
kode_lsm6dsox_handle_t imu_handle;
ESP_ERROR_CHECK(kode_lsm6dsox_init_i2c(i2c_bus, 0x6A, &imu_handle));

// Reading sensor data
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float temperature;

ESP_ERROR_CHECK(kode_lsm6dsox_read_accel(&imu_handle, &accel_x, &accel_y, &accel_z));
ESP_ERROR_CHECK(kode_lsm6dsox_read_gyro(&imu_handle, &gyro_x, &gyro_y, &gyro_z));
ESP_ERROR_CHECK(kode_lsm6dsox_read_temp(&imu_handle, &temperature));

printf("Accel: X=%.2f mg, Y=%.2f mg, Z=%.2f mg\n", accel_x, accel_y, accel_z);
printf("Gyro: X=%.2f mdps, Y=%.2f mdps, Z=%.2f mdps\n", gyro_x, gyro_y, gyro_z);
printf("Temperature: %.2f °C\n", temperature);
```

### FIFO Usage

```c
// Configure FIFO in continuous mode with watermark of 32 samples
ESP_ERROR_CHECK(kode_lsm6dsox_fifo_config(&imu_handle, 
                                       KODE_LSM6DSOX_FIFO_CONTINUOUS_MODE, 
                                       32));

// Read FIFO data level
uint16_t samples;
ESP_ERROR_CHECK(kode_lsm6dsox_fifo_data_level(&imu_handle, &samples));

// Read accelerometer data from FIFO
float accel_data[32][3]; // 32 samples, each with x,y,z values
ESP_ERROR_CHECK(kode_lsm6dsox_fifo_read_accel(&imu_handle, accel_data, 32));

// Read gyroscope data from FIFO
float gyro_data[32][3]; // 32 samples, each with x,y,z values
ESP_ERROR_CHECK(kode_lsm6dsox_fifo_read_gyro(&imu_handle, gyro_data, 32));

// Flush FIFO if needed
ESP_ERROR_CHECK(kode_lsm6dsox_fifo_flush(&imu_handle));
```

### Interrupt Usage

Configure interrupt pins and enable specific interrupts:

```c
// Configure interrupt pins - active high, push-pull
ESP_ERROR_CHECK(kode_lsm6dsox_interrupt_pin_config(&imu_handle, 0, 0));

// Enable accelerometer data ready interrupt on INT1
ESP_ERROR_CHECK(kode_lsm6dsox_interrupt_enable_int1(&imu_handle, 
                                                 KODE_LSM6DSOX_INT_DRDY_XL));

// Enable FIFO threshold and gyroscope data ready interrupts on INT2
ESP_ERROR_CHECK(kode_lsm6dsox_interrupt_enable_int2(&imu_handle, 
                                                 KODE_LSM6DSOX_INT_FIFO_TH | 
                                                 KODE_LSM6DSOX_INT_DRDY_G));

// Read interrupt source after an interrupt occurs
lsm6dsox_all_sources_t int_src;
ESP_ERROR_CHECK(kode_lsm6dsox_interrupt_source_get(&imu_handle, &int_src));

// Check which interrupt occurred
if (int_src.drdy_xl) {
    printf("Accelerometer data ready\n");
}
if (int_src.drdy_g) {
    printf("Gyroscope data ready\n");
}
if (int_src.fifo_th) {
    printf("FIFO threshold reached\n");
}
```

### Power Management

The LSM6DSOX component supports different power modes to optimize power consumption based on your application needs:

```c
// Set power mode
ESP_ERROR_CHECK(kode_lsm6dsox_set_power_mode(&imu_handle, KODE_LSM6DSOX_POWER_LOW_NORMAL));

// Get current power mode
kode_lsm6dsox_power_mode_t current_mode;
ESP_ERROR_CHECK(kode_lsm6dsox_get_power_mode(&imu_handle, &current_mode));
```

Available power modes:

1. `KODE_LSM6DSOX_POWER_HIGH_PERFORMANCE`: Maximum performance mode
   - Typical current: 3.75 mA
   - Best for applications requiring high data rates and accuracy

2. `KODE_LSM6DSOX_POWER_LOW_NORMAL`: Balanced power and performance
   - Typical current: 3.45 mA
   - Suitable for most applications

3. `KODE_LSM6DSOX_POWER_ULTRA_LOW`: Minimum power consumption
   - Typical current: 0.03 mA
   - Best for battery-powered applications with infrequent measurements

4. `KODE_LSM6DSOX_POWER_SUSPEND`: Sensor in sleep mode
   - Minimal power consumption
   - Use when sensor readings are not needed

For a complete example of power management usage, see `example_power_modes.c`.

## Reference

### API Functions

For a complete list of available functions, please see `include/kode_lsm6dsox.h`.

### Examples

The component includes example applications that demonstrate various features:

1. `example_interrupt.c` - Shows how to use INT1 and INT2 pins to trigger on data ready events
2. `example_fifo_interrupt.c` - Demonstrates FIFO usage with interrupts for efficient data collection
3. `example_power_modes.c` - Shows how to use power management to optimize power consumption

## License

This component is licensed under Apache 2.0 license.
