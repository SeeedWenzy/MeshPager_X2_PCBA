# BMM350 Magnetometer Test Example

## Overview

This example demonstrates how to use the BMM350 magnetometer sensor with ESP-IDF. The BMM350 is a 3-axis magnetometer that measures magnetic field strength in microtesla (µT) along X, Y, and Z axes, and also provides temperature readings.

## Features

- **Sensor Initialization**: Complete setup of BMM350 sensor via I2C
- **Configuration**: 
  - Power mode management (Normal/Suspend/Forced modes)
  - Output Data Rate (ODR) configuration (1.5625 Hz to 400 Hz)
  - Performance/Averaging settings (Low power to Ultra low noise)
  - Axis enable/disable control
- **Data Reading**: Continuous reading of compensated magnetometer data (X, Y, Z) and temperature
- **Self-Test**: Built-in self-test functionality for X and Y axes
- **Error Handling**: Comprehensive error checking and logging

## Hardware Requirements

- ESP32 or ESP32-S3 development board
- BMM350 magnetometer sensor module
- I2C connection (default):
  - SDA: GPIO 4
  - SCL: GPIO 5
- Power supply: 3.3V

## Wiring

```
BMM350 Module    ESP32
------------     -----
VCC      ------>  3.3V
GND      ------>  GND
SDA      ------>  GPIO 4 (or your chosen SDA pin)
SCL      ------>  GPIO 5 (or your chosen SCL pin)
```

## Building and Flashing

### 1. Configure the Project

```bash
cd components/bmm350/examples/main
idf.py set-target esp32  # or esp32s3
```

### 2. Configure I2C Pins (Optional)

If you need to use different I2C pins, edit `main.c` and modify the `bmm350_esp_init()` call:

```c
// Change GPIO pins as needed
rslt = bmm350_esp_init(&dev, I2C_NUM_0, YOUR_SDA_GPIO, YOUR_SCL_GPIO);
```

### 3. Build and Flash

```bash
idf.py build
idf.py flash monitor
```

## Output Example

When running successfully, you should see output similar to:

```
I (1234) bmm350_test: ========================================
I (1235) bmm350_test: BMM350 Magnetometer Test Example
I (1236) bmm350_test: ========================================
I (1250) bmm350_test: BMM350 initialized successfully!
I (1251) bmm350_test: Chip ID: 0x33
I (1255) bmm350_test: BMM350 API Version: 1.10.0
I (1260) bmm350_test: Power mode set to NORMAL
I (1265) bmm350_test: ODR set to 25 Hz with LOW noise performance
I (1270) bmm350_test: All axes enabled (X, Y, Z)
I (1275) bmm350_test: Performing self-test...
I (3400) bmm350_test: Self-test completed successfully
I (3401) bmm350_test:   X-axis - High: 45.67 uT, Low: -45.23 uT
I (3402) bmm350_test:   Y-axis - High: 44.89 uT, Low: -44.91 uT
I (3403) bmm350_test:   X-axis result: 45.45 uT
I (3404) bmm350_test:   Y-axis result: 44.90 uT
I (3405) bmm350_test: ========================================
I (3406) bmm350_test: Starting continuous measurements...
I (3407) bmm350_test: ========================================
BMM350 - Mag X:    12.34 uT, Mag Y:   -5.67 uT, Mag Z:    42.18 uT, Temp:  25.42 °C
BMM350 - Mag X:    12.45 uT, Mag Y:   -5.59 uT, Mag Z:    42.21 uT, Temp:  25.43 °C
BMM350 - Mag X:    12.38 uT, Mag Y:   -5.63 uT, Mag Z:    42.19 uT, Temp:  25.42 °C
...
```

## Code Structure

### Main Functions

- `app_main()`: Entry point, initializes sensor and configuration
- `bmm350_measurement_task()`: FreeRTOS task that continuously reads sensor data
- `print_api_version()`: Displays driver version information
- `perform_self_test()`: Executes sensor self-test

### Key Configuration Parameters

#### Output Data Rate (ODR)
Available options:
- `BMM350_DATA_RATE_400HZ` - 400 Hz
- `BMM350_DATA_RATE_200HZ` - 200 Hz
- `BMM350_DATA_RATE_100HZ` - 100 Hz
- `BMM350_DATA_RATE_50HZ` - 50 Hz
- `BMM350_DATA_RATE_25HZ` - 25 Hz (default)
- `BMM350_DATA_RATE_12_5HZ` - 12.5 Hz
- `BMM350_DATA_RATE_6_25HZ` - 6.25 Hz
- `BMM350_DATA_RATE_3_125HZ` - 3.125 Hz
- `BMM350_DATA_RATE_1_5625HZ` - 1.5625 Hz

#### Performance/Averaging
Available options:
- `BMM350_LOWPOWER` - No averaging (highest noise, lowest power)
- `BMM350_REGULARPOWER` - 2x averaging
- `BMM350_LOWNOISE` - 4x averaging (default)
- `BMM350_ULTRALOWNOISE` - 8x averaging (lowest noise, highest power)

#### Power Modes
- `BMM350_SUSPEND_MODE` - Sleep mode
- `BMM350_NORMAL_MODE` - Continuous measurement
- `BMM350_FORCED_MODE` - Single measurement
- `BMM350_FORCED_MODE_FAST` - Fast single measurement

## Troubleshooting

### Sensor not detected

1. **Check wiring**: Verify I2C connections (SDA, SCL, VCC, GND)
2. **Check I2C address**: BMM350 default address is 0x10, but can be 0x11
3. **Check pull-up resistors**: Ensure SDA and SCL have 4.7kΩ pull-up resistors
4. **Use I2C scanner**: Run `idf.py menuconfig` → Component config → I2C to enable I2C scanner

### Compilation errors

Ensure required components are available:
- `esp-idf-lib/i2cdev`
- `bmm350` driver

### Incorrect readings

1. **Magnetic reset**: The sensor may need a magnetic reset if exposed to strong magnetic fields (>400mT)
2. **Calibration**: For best accuracy, perform factory calibration or implement soft-iron/hard-iron calibration
3. **Performance settings**: Higher averaging (ULTRALOWNOISE) provides more stable readings

## Advanced Usage

### Magnetic Reset

If the sensor was exposed to strong magnetic fields, perform a magnetic reset:

```c
rslt = bmm350_magnetic_reset_and_wait(&dev);
```

### Reading Raw Data

For applications requiring raw sensor data:

```c
struct bmm350_raw_mag_data raw_data;
rslt = bmm350_read_uncomp_mag_temp_data(&raw_data, &dev);
```

### Interrupt Configuration

Enable data ready interrupt for event-driven reading:

```c
bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, &dev);
bmm350_configure_interrupt(BMM350_LATCHED, BMM350_ACTIVE_HIGH, 
                           BMM350_INTR_PUSH_PULL, BMM350_MAP_TO_PIN, &dev);
```

## References

- [BMM350 Datasheet](https://www.bosch-sensortec.com/products/magnetometers/magnetometer-for-consumer-electronics/bmm350/)
- [BMM350 Driver Repository](https://github.com/BoschSensortec/BMM350-Sensor-API)

## License

This example uses the BMM350 driver, which is licensed under BSD-3-Clause by Bosch Sensortec GmbH.