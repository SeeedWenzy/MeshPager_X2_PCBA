# meshpager_x2

Board Support Package (BSP) component for the **SenseCAP MeshPager X2** board, based on the
ESP32-S3. It abstracts the board's power path, IO expander (TCA6424), buttons, display
(ST7789P3 LCD), audio codecs (ES8311 / ES7243E), GNSS module, SD card, SPIFFS, battery ADC,
and the two I2C buses, so application code can drive the hardware through a small,
high-level C API instead of touching registers and pins directly.

- **Target:** ESP32-S3 (`targets: esp32s3`)
- **IDF version:** `>=5.3`
- **Hardware/Firmware version:** `BSP_HW_VERSION "1.0.0"` / `BSP_SW_VERSION "1.0.0"`
- **Dependencies:** `button`, `esp-sr`, `esp_codec_dev`, `esp_io_expander_tca6424`, `lr20xx`
  (LoRa), plus IDF components `driver`, `esp_adc`, `esp_lcd`, `nvs_flash`, `usb`, `spiffs`, `fatfs`.

## 1. What this component provides

| Subsystem | Public API (in `include/`) | Notes |
|-----------|----------------------------|-------|
| Board power-up / power hold | `bsp_power_up_init()` | Must be the **first** BSP call. Holds `BSP_PWR_HOLD` so the board stays powered when running from battery. Returns `ESP_ERR_INVALID_STATE` when the board is in the off state — callers should skip normal startup. |
| Buttons (via TCA6424) | `bsp_button_event_callback_register()` | Event-driven: callback receives a button mask + pressed flag. |
| Power-off on long press | `bsp_power_shutdown_callback_register()` | Callback fires before `BSP_PWR_HOLD` is dropped on a long power-button press. |
| IO expander | `bsp_exp_output_io_set_level()`, `bsp_exp_input_io_get_level()`, `bsp_exp_io_set_dir()`, `bsp_deinit_io_expander()` | Direct access to TCA6424 lines. |
| LCD (ST7789P3, SPI) | `bsp_display_init/deinit()`, `bsp_display_get_panel[_io]()`, `bsp_display_draw_bitmap()`, `bsp_display_brightness_set()` | 240×320, RGB565. Backlight via LEDC PWM. |
| LCD backlight | `lcd_bl_init()`, `lcd_bl_set()`, `lcd_bl_on()`, `lcd_bl_off()` | Brightness 0–255. |
| Audio | `bsp_audio_init()`, `bsp_audio_codec_speaker_init()`, `bsp_audio_codec_microphone_init()` | I2S + `esp_codec_dev`. ES8311 DAC (playback), ES7243E ADC (recording). |
| SD card (FAT VFS) | `bsp_sdcard_mount/unmount()` | Exposes `bsp_sdcard` and `BSP_SD_MOUNT_POINT`. |
| SPIFFS (VFS) | `bsp_spiffs_mount/unmount()` | `BSP_SPIFFS_MOUNT_POINT`. |
| Battery | `bsp_bat_power_control()`, `bsp_battery_voltage_read()` | Voltage in mV via ADC with a 2:1 divider. |
| I2C buses | see `bsp_i2c_control.h` | I2C0: TCA6424 + RTC YSN8900E. I2C1: SHT4x, LSM6DSO, BMM350, ES7243E, ES8311. |
| GNSS | see `bsp_gnss_control.h` | UART + power/sleep/RTC mode control. |
| Low-power test helpers | `bsp_power_hold_deep_sleep_test()`, `bsp_power_hold_gnss_test()` | For current/sleep measurements. |

## 2. Power-up model (read this first)

There are two ways to power and boot the board (see comment in
[`include/meshpager_x2.h`](include/meshpager_x2.h)):

1. **Battery + USB** without pressing the power button — the device activates automatically.
2. **Battery only** — press the `BSP_BUTTON_ONOFF` button to activate; the board then needs
   `BSP_PWR_HOLD` driven high to stay on.

`bsp_power_up_init()` is what asserts `BSP_PWR_HOLD` and reads the persisted power state from
NVS (`namespace="power"`, key=`"state"`). It must run before any other BSP call that touches
expander-powered peripherals (LCD, SD, GNSS, sensors, audio PA, etc.).

The power button (`BSP_BUTTON_ONOFF`, GPIO9) is polled by an internal task. A long press
longer than `BSP_POWER_BUTTON_LONG_PRESS_DURATION_MS` (3000 ms) triggers shutdown: the
registered shutdown callback is invoked, then `BSP_PWR_HOLD` is released and the board
cuts its own power.

## 3. Adding the component to a project

The component is registered through `idf_component.yml`:

```yaml
dependencies:
  espressif/meshpager_x2: "^0.0.1"
```

Or, for a local checkout, place this folder under `components/` (or point
`EXTRA_COMPONENT_DIRS` at it) — exactly what the bundled example does:

```cmake
# example/CMakeLists.txt
set(EXTRA_COMPONENT_DIRS ../../../components)
```

In your `main/CMakeLists.txt`, list the component in `REQUIRES`:

```cmake
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES meshpager_x2)
```

Then include the public header (which pulls in everything):

```c
#include "esp-bsp.h"   // == #include "meshpager_x2.h"
```

For GNSS or I2C helpers, include the dedicated headers:

```c
#include "bsp_gnss_control.h"
#include "bsp_i2c_control.h"
```

## 4. Kconfig options

The component adds a `Board Support Package(ESP32-S3)` menu (see [Kconfig](Kconfig)):

| Option | Default | Description |
|--------|---------|-------------|
| `BSP_ERROR_CHECK` | `y` | Assert on errors inside BSP instead of returning error codes (see `bsp_err_check.h`). |
| `BSP_SD_FORMAT_ON_MOUNT_FAIL` | `n` | Format the uSD card as FAT if mounting fails. |
| `BSP_SD_MOUNT_POINT` | `/sdcard` | VFS mount point for the uSD card. |
| `BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL` | `n` | Format SPIFFS if mounting fails. |
| `BSP_SPIFFS_MOUNT_POINT` | `/spiffs` | VFS mount point for SPIFFS. |
| `BSP_SPIFFS_PARTITION_LABEL` | `storage` | Partition label that holds SPIFFS. |
| `BSP_SPIFFS_MAX_FILES` | `5` | Max simultaneously open files on SPIFFS. |

## 5. Minimal usage example

The simplest program — power the board up and keep it alive (mirrors
[`example/main/main.c`](example/main/main.c)):

```c
#include "esp_log.h"
#include "esp-bsp.h"

static const char *TAG = "app";

void app_main(void)
{
    esp_err_t ret = bsp_power_up_init();
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Power state is off, skip example startup");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "MeshPager X2 board power up...");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
```

## 6. Button events

Register a single callback for all IO-expander buttons (UP/DOWN/LEFT/RIGHT/CONFIRM/RETURN/PWR,
plus IMU_INT). It is dispatched from an internal task whenever a button level changes:

```c
static void on_button(uint32_t mask, bool pressed, void *ctx)
{
    if (pressed && mask == BSP_BUTTON_CONFIRM) {
        ESP_LOGI("btn", "CONFIRM pressed");
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(bsp_power_up_init());
    bsp_button_event_callback_register(on_button, NULL);
}
```

The button masks are defined as bit masks over the TCA6424 pins (e.g. `BSP_BUTTON_UP`,
`BSP_BUTTON_CONFIRM`, `BSP_BUTTON_PWR`).

## 7. Display (ST7789P3)

Initialize **after** `bsp_power_up_init()` so the LCD power and reset expander lines are
available, then draw RGB565 bitmaps or hand the panel handle to LVGL:

```c
ESP_ERROR_CHECK(bsp_power_up_init());
ESP_ERROR_CHECK(bsp_display_init());
lcd_bl_init(100);                       // backlight to 100/255
bsp_display_brightness_set(200);        // 0..255

esp_lcd_panel_handle_t panel = bsp_display_get_panel();
/* feed `panel` to lvgl or call bsp_display_draw_bitmap() directly */
```

Backlight helpers: `lcd_bl_on()`, `lcd_bl_off()`, `lcd_bl_set(level)`.

## 8. Audio (ES8311 playback / ES7243E recording)

```c
bsp_audio_init(NULL, NULL);                          // default I2S: mono, 16-bit, 22050 Hz
esp_codec_dev_handle_t spk = bsp_audio_codec_speaker_init();
esp_codec_dev_handle_t mic = bsp_audio_codec_microphone_init();

esp_codec_dev_set_out_vol(spk, DEFAULT_VOLUME);
esp_codec_dev_open(spk, &fs);
esp_codec_dev_write(spk, wav_bytes, bytes_read);
esp_codec_dev_close(spk);
```

Resources can be released later with `i2s_del_channel()` (there is no dedicated deinit).

## 9. SD card and SPIFFS

```c
bsp_sdcard_mount();
FILE *f = fopen(BSP_SD_MOUNT_POINT "/log.txt", "w");
fprintf(f, "hello\n");
fclose(f);
/* ... */
bsp_sdcard_unmount();

bsp_spiffs_mount();
FILE *p = fopen(BSP_SPIFFS_MOUNT_POINT "/config.json", "r");
/* ... */
bsp_spiffs_unmount();
```

## 10. Battery voltage

```c
bsp_bat_power_control(true);              // enable the voltage divider circuit
uint16_t mv = bsp_battery_voltage_read(); // voltage in mV
bsp_bat_power_control(false);             // disable to save power
```

## 11. GNSS

```c
#include "bsp_gnss_control.h"

if (bsp_gnss_scan_start()) {              // powers GNSS, wakes via RTC int, locks sleep
    uint8_t buf[256];
    int n = gnss_uart_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(100));
    /* parse NMEA ... */
}
bsp_gnss_scan_stop();                    // unlock sleep, enter RTC mode, deinit UART
```

Other helpers: `bsp_gnss_uart_init/deinit()`, `gnss_uart_write_bytes()`,
`gnss_uart_trace_print()`, `bsp_gnss_power_init()`, `bsp_gnss_poweroff()`,
`bsp_gnss_reset()`, `bsp_gnss_scan_lock/unlock_sleep()`,
`bsp_gnss_scan_enter_rtc_mode()`.

## 12. I2C buses

```c
#include "bsp_i2c_control.h"

bsp_i2c_0_init();   // TCA6424 IO expander + YSN8900E RTC
bsp_i2c_1_init();   // SHT4x, LSM6DSO, BMM350, ES7243E, ES8311
i2c_master_bus_handle_t bus = bsp_i2c_1_get_handle();
bsp_i2c_0_scan();    // enumerate present slaves
```

## 13. Pin map (summary)

Defined in [`include/meshpager_x2.h`](include/meshpager_x2.h). GPIOs that live on the TCA6424
expander are given as `1ULL << EXP_PIN_NUM_n` (e.g. `BSP_PWR_HOLD`, `BSP_LCD_PWR_EN`,
`BSP_GNSS_PWR_EN`, `BSP_SD_PWR_EN`, `BSP_PA_PWR_EN`, …); the button inputs are likewise on
the expander. Direct ESP32-S3 GPIOs include the LCD SPI (`BSP_LCD_CS/SDA/SCK/A0/PWM`),
LoRa SPI (`BSP_LORA_SPI_*`), battery ADC (`BSP_VBAT_ADC`), `BSP_BUTTON_ONOFF` (GPIO9),
and the UART1 debug/GNSS pins.

## 14. Running the bundled example

```bash
cd components/meshpager_x2/example
idf.py set-target esp32s3
idf.py build flash monitor
```

`example/sdkconfig.defaults` enables 240 MHz CPU, disables the task watchdog, and sets
`FREERTOS_HZ=1000` — a sensible baseline for the board.
