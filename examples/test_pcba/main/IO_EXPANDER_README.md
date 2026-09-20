# IO Expander Test Code Implementation

## Overview
This implementation adds external IO expander test functionality to the PCBA test application, allowing testing of the TCA6424 IO expander chip via I2C bus.

## Files Created
1. **io_expander_cmd.h** - Header file with function declarations
2. **io_expander_cmd.c** - Implementation of IO expander test commands

## Files Modified
1. **main.c** - Added include for io_expander_cmd.h and registered IO expander commands
2. **CMakeLists.txt** - Added io_expander_cmd.c to source files and required dependencies

## Dependencies
- **esp_io_expander_tca6424** - Component at `components/esp_io_expander_tca6424`
- **esp_io_expander** - Base IO expander library (local component at `components/esp_io_expander`)
- **driver** - ESP-IDF driver library

**Note**: The esp_io_expander component was created as a local component from the managed component in esp_io_expander_pca9535 to resolve build dependencies.

## Available Commands

### 1. io_exp_init
Initialize the IO expander device.
```
io_exp_init
```
- Initializes I2C bus (SCL: GPIO 7, SDA: GPIO 6)
- Creates TCA6424 IO expander instance with address 0x22 (GND)
- Must be run before other IO expander commands

### 2. io_exp_config
Configure IO expander pin direction.
```
io_exp_config -i <io> -m <mode> [-p <pull>]
```
Parameters:
- `-i <io>`: IO pin number (0-23)
- `-m <mode>`: IO mode (0=input, 1=output)
- `-p <pull>`: Optional pull mode (0=none, 1=pullup, 2=pulldown)

Example:
```
io_exp_config -i 0 -m 0 -p 1  # Set IO0 as input with pullup
io_exp_config -i 1 -m 1        # Set IO1 as output
```

### 3. io_exp_set
Set IO expander pin level.
```
io_exp_set -i <io> -l <level>
```
Parameters:
- `-i <io>`: IO pin number (0-23)
- `-l <level>`: Level to set (0 or 1)

Example:
```
io_exp_set -i 1 -l 1  # Set IO1 to high level
```

### 4. io_exp_get
Get IO expander pin level.
```
io_exp_get -i <io>
```
Parameters:
- `-i <io>`: IO pin number (0-23)

Example:
```
io_exp_get -i 0  # Read IO0 level
```

### 5. io_exp_print
Print current state of all IO expander pins.
```
io_exp_print
```
- Displays direction, input level, and output level for all 24 IOs

## Usage Example
```
# Initialize IO expander
io_exp_init

# Configure IO0 as input with pullup
io_exp_config -i 0 -m 0 -p 1

# Configure IO1 as output
io_exp_config -i 1 -m 1

# Set IO1 to high
io_exp_set -i 1 -l 1

# Read IO0 level
io_exp_get -i 0

# Print all IO states
io_exp_print
```

## Hardware Configuration
- **I2C SCL**: GPIO 7
- **I2C SDA**: GPIO 6
- **I2C Frequency**: 400 kHz
- **IO Expander Address**: 0x22 (GND configuration)
- **IO Count**: 24 pins (IO0-IO23)

## Technical Details
- Uses TCA6424 IO expander chip via I2C communication
- Supports up to 24 GPIO pins (3 banks of 8 bits)
- Follows the same command structure as gpio_cmd.c for consistency
- Includes input/output direction configuration
- Supports pull-up/pull-down resistor configuration
- Provides real-time status display of all pins

## Error Handling
- Validates IO number range (0-23)
- Validates level values (0 or 1)
- Validates mode values (0=input, 1=output)
- Validates pull mode values (0=none, 1=pullup, 2=pulldown)
- Checks if IO expander is initialized before operations
- Reports ESP-IDF error codes for debugging

## Integration with Existing Code
- Registered in main.c alongside other test commands (gpio_cmd, lora_cmd, etc.)
- Uses same console command framework (esp_console + argtable3)
- Follows existing code style and conventions
- Maintains consistency with gpio_cmd.c structure

## Testing Recommendations
1. Initialize IO expander with `io_exp_init`
2. Configure a pin as output and verify with `io_exp_set`
3. Configure a pin as input and verify with `io_exp_get`
4. Use `io_exp_print` to monitor all pin states
5. Test pull-up/pull-down functionality
6. Verify I2C communication with physical connections