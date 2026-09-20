/*
 * main.c — test_fullfeatured (light-sleep low-power loop)
 *
 * Boots the board, caches media from SD into PSRAM, initializes every
 * peripheral once, then loops forever: step every peripheral's test once,
 * cut the backlight + GPS, set WiFi low-power, hold the LoRa/LCD CS pins,
 * enter light sleep for 60 s, wake and restore — then repeat.
 */

#include <stdio.h>
#include <string.h>

#include "esp_console.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "meshpager_x2.h"

#include "periodic_runner.h"
#include "peripherals.h"

static const char *TAG = "MAIN";
#define APP_SW_VERSION "test_fullfeatured-2.0.2"

#define HEAP_DEBUG_CHECKPOINTS 1

#if HEAP_DEBUG_CHECKPOINTS
static void heap_debug_checkpoint(const char *stage)
{
    bool intact = heap_caps_check_integrity_all(true);
    ESP_LOGI(TAG,
             "HEAP %-28s intact=%d internal=%u/%u 8bit=%u/%u psram=%u/%u",
             stage,
             intact ? 1 : 0,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
}
#else
#define heap_debug_checkpoint(stage) do { (void)(stage); } while (0)
#endif

/* Kconfig bools are #define'd to 1 when selected and undefined otherwise.
 * Normalize so the job table compiles in both states. */
#ifdef CONFIG_RUNNER_LORA_RX_ENABLED_BY_DEFAULT
#define LORA_RX_DEFAULT_ENABLED 1
#else
#define LORA_RX_DEFAULT_ENABLED 0
#endif

/* ---- console REPL backend selector (from examples/test_pcba/main/main.c) ---- */

static void app_console_start(esp_console_repl_config_t *repl_config)
{
    esp_console_repl_t *repl = NULL;

#if CONFIG_ESP_CONSOLE_UART_DEFAULT || CONFIG_ESP_CONSOLE_UART_CUSTOM
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, repl_config, &repl));
#else
#error "Console REPL requires a supported CONFIG_ESP_CONSOLE_* backend"
#endif

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

/* ---- job definitions ---- */

static runner_job_t s_job_table[] = {
    {
        .name = "display",
        .init_fn = display_init, .step_fn = display_step, .suspend_fn = display_suspend,
        .default_period_ms = CONFIG_RUNNER_PERIOD_DISPLAY_MS,
        .enabled = true, .stack_bytes = 4096, .priority = 5,
    },
    {
        .name = "audio",
        .init_fn = audio_init, .step_fn = audio_step, .suspend_fn = audio_suspend,
        .default_period_ms = CONFIG_RUNNER_PERIOD_AUDIO_MS,
        .enabled = true, .stack_bytes = 8192, .priority = 5,
    },
    {
        .name = "wifi",
        .init_fn = wifi_init, .step_fn = wifi_step, .suspend_fn = wifi_suspend,
        .default_period_ms = CONFIG_RUNNER_PERIOD_WIFI_MS,
        .enabled = true, .stack_bytes = 6144, .priority = 5,
    },
    {
        .name = "ble",
        .init_fn = ble_init, .step_fn = ble_step, .suspend_fn = ble_suspend,
        .default_period_ms = CONFIG_RUNNER_PERIOD_BLE_MS,
        .enabled = true, .stack_bytes = 4096, .priority = 5,
    },
    {
        .name = "gnss",
        .init_fn = gnss_init, .step_fn = gnss_step, .suspend_fn = gnss_suspend,
        .default_period_ms = CONFIG_RUNNER_PERIOD_GNSS_MS,
        .enabled = true, .stack_bytes = 4096, .priority = 5,
    },
    {
        .name = "sensor",
        .init_fn = sensor_init, .step_fn = sensor_step, .suspend_fn = sensor_suspend,
        .default_period_ms = CONFIG_RUNNER_PERIOD_SENSOR_MS,
        .enabled = true, .stack_bytes = 6144, .priority = 6,
    },
    {
        .name = "imu",
        .init_fn = imu_init, .step_fn = imu_step, .suspend_fn = imu_suspend,
        .default_period_ms = CONFIG_RUNNER_PERIOD_IMU_MS,
        .enabled = true, .stack_bytes = 4096, .priority = 6,
    },
    {
        .name = "lora_tx",
        .init_fn = lora_tx_init, .step_fn = lora_tx_step, .suspend_fn = lora_tx_suspend,
        .default_period_ms = CONFIG_RUNNER_PERIOD_LORA_TX_MS,
        .enabled = true, .stack_bytes = 4096, .priority = 5,
    },
    {
        .name = "lora_rx",
        .init_fn = lora_rx_init, .step_fn = lora_rx_step, .suspend_fn = lora_rx_suspend,
        .default_period_ms = CONFIG_RUNNER_PERIOD_LORA_RX_MS,
        .enabled = LORA_RX_DEFAULT_ENABLED,
        .stack_bytes = 4096, .priority = 5,
    },
    {
        .name = "sdcard",
        .init_fn = sdcard_init, .step_fn = sdcard_step, .suspend_fn = sdcard_suspend,
        .default_period_ms = CONFIG_RUNNER_PERIOD_SDCARD_MS,
        .enabled = true, .stack_bytes = 4096, .priority = 5,
    },
    {
        .name = "battery",
        .init_fn = battery_init, .step_fn = battery_step, .suspend_fn = battery_suspend,
        .default_period_ms = CONFIG_RUNNER_PERIOD_BATTERY_MS,
        .enabled = true, .stack_bytes = 3072, .priority = 5,
    },
};

/* ------------------------------------------------------------------ */
/* Light-sleep low-power loop                                         */
/* ------------------------------------------------------------------ */

/* 60 s light-sleep window between test cycles. */
#define LIGHT_SLEEP_DURATION_US  (3*60ULL * 1000ULL * 1000ULL)

#define LIGHT_SLEEP_DEBUG_DURATION_US  (10ULL * 1000ULL * 1000ULL)

/* Hold ONLY the LoRa CS pin HIGH across light sleep. The LoRa module has no
 * rail we cut, so it stays powered and its CS must be held high to avoid a
 * falling CS while its SPI host is idle. The LCD CS is NOT held — its rail is
 * cut in teardown and isolate_cut_rail_signal_pins() releases it to high-Z, so
 * a driven-high CS can't back-feed the unpowered panel. */
static void hold_sleep_cs_high(void)
{
    gpio_set_direction(BSP_LORA_SPI_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(BSP_LORA_SPI_CS, 1);
    gpio_hold_en(BSP_LORA_SPI_CS);
}

/* After wake: drop the LoRa CS GPIO hold and return it to normal output-high. */
static void release_sleep_cs(void)
{
    gpio_hold_dis(BSP_LORA_SPI_CS);
    gpio_set_direction(BSP_LORA_SPI_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(BSP_LORA_SPI_CS, 1);
}

/* Release the signal pins of every peripheral whose power rail we just cut.
 * With VDD removed, any pin still driven HIGH back-powers the chip through its
 * input clamp diodes (parasitic current) — a major cause of unexpectedly high
 * sleep current. gpio_reset_pin() returns each to a high-Z input so nothing
 * leaks. Mirrors prepare_low_leakage_sleep() in examples/test_temp_lowpower.
 * The recover phase re-configures them via the *_init calls. */
static void isolate_cut_rail_signal_pins(void)
{
    /* LCD panel (BSP_LCD_PWR_EN cut) — CS included: don't hold it, the rail is
     * off and a held-high CS would back-feed the panel. */
    gpio_reset_pin(BSP_LCD_CS);
    gpio_reset_pin(BSP_LCD_SDA);
    gpio_reset_pin(BSP_LCD_SCK);
    gpio_reset_pin(BSP_LCD_A0);
    gpio_reset_pin(BSP_LCD_PWM);
    /* SD card (BSP_SD_PWR_EN cut) */
    gpio_reset_pin(BSP_SD_D0);
    gpio_reset_pin(BSP_SD_CLK);
    gpio_reset_pin(BSP_SD_CMD);
    /* NOTE: the audio I2S pins (MCLK/SCLK/LRLK/SDIN/SDOUT) are intentionally
     * NOT reset here. bsp_audio_init() configures them only once (guarded by
     * i2s_adc_data_if); a gpio_reset_pin() here detaches the GPIO matrix and
     * the post-wake codec re-init never reconnects them, so the ES7243E ends
     * up unclocked and recording is silent (raw_mic=0). audio_suspend() already
     * stops the I2S clocks, so the back-feed from these pins is small — accept
     * a little extra sleep current to keep recording working after a rail cut. */
    /* GNSS (powered off in gnss_suspend) */
    gpio_reset_pin(BSP_GNSS_RX);
    gpio_reset_pin(BSP_GNSS_TX);
    gpio_reset_pin(BSP_GNSS_PPS0);
    /* Note: BSP_VBAT_ADC left as-is — it's a high-Z analog input (negligible
     * back-feed) and battery_init doesn't re-configure the ADC pin. */
}

/* Debug switch for the low-power loop.
 *   1 = debug: steps (6) CS-hold and (7) light sleep are replaced by a plain
 *       vTaskDelay idle, so the USB-Serial-JTAG console stays connected during
 *       bring-up (the hardware USB peripheral drops off the bus under real
 *       light sleep). Steps (3)/(5)/(9) still run.
 *   0 = release: real CS-hold + 60 s light sleep. */
#define LOWPOWER_DEBUG_IDLE 1

/* Full-feature low-power loop. Exercises every peripheral once per cycle, but
 * ONE AT A TIME: each job is initialized, stepped, then suspended before the
 * next job starts, so only one peripheral is live during its own step. After the
 * active phase the whole board is already torn down; the loop then cuts the
 * LCD/SD/SEN_EN rails, isolates signal pins to stop back-feed, holds the LoRa
 * CS high, and enters light sleep. On wake it re-powers the rails — the next
 * cycle's per-job init re-initializes each peripheral. Recording still works
 * across the SEN_EN rail cut because audio_record_step re-inits the codec
 * before each take.
 *
 * Per cycle:
 *   (2) for each job: init -> step -> suspend (only that job is live)
 *   (3)+(5) drivers already suspended above; cut LCD/SD/SEN_EN rails;
 *           isolate their signal pins; hold LoRa CS high
 *   (7) enter light sleep
 *   (8) wake: release LoRa CS hold; re-power the cut rails
 *   (9) (no bulk re-init) loop back; each job re-inits before it steps
 *   (10) repeat from (2) */

/* Bring one job up, run a single bounded step, then tear it straight back down.
 * Suspending each job before the next starts keeps only one peripheral active at
 * a time, so the active phase draws ~baseline + one peripheral instead of
 * baseline + all of them (e.g. the WiFi/BLE modem is on only for the few ms of
 * its own step, not the whole active phase). init is idempotent (guarded by
 * per-driver ready flags), so this is safe whether or not the job was already
 * initialized. ctx is NULL here — shared state is file-scope in peripherals.c. */
static void run_job_lifecycle(const runner_job_t *job)
{
    if (job->init_fn != NULL) {
        job->init_fn(job->ctx);
    }
    job->step_fn(job->ctx);
#if !LOWPOWER_DEBUG_IDLE
    /* Debug mode skips teardown so peripherals stay live and the console stays
     * usable during bring-up (it also skips the rail cut + light sleep below). */
    if (job->suspend_fn != NULL) {
        job->suspend_fn(job->ctx);
    }
#endif
}

static void lowpower_loop_run(void)
{
    ESP_LOGI(TAG, "cold boot / first cycle");

    const size_t n = sizeof(s_job_table) / sizeof(s_job_table[0]);

    while (true) {
        /* (2) exercise every peripheral once, ONE AT A TIME: each job is
         * init'd, stepped, then suspended before the next starts, so only the
         * current job's hardware is live during its step. */
        for (size_t i = 0; i < n; i++) {
            run_job_lifecycle(&s_job_table[i]);
        }

#if LOWPOWER_DEBUG_IDLE
        /* DEBUG: no teardown / no rail cut / no light sleep — just idle so the
         * console stays usable and every peripheral keeps running. */
        ESP_LOGI(TAG, "debug idle %llu s (no light sleep)",
                 (unsigned long long)(LIGHT_SLEEP_DEBUG_DURATION_US / 1000000ULL));
        vTaskDelay(pdMS_TO_TICKS((uint32_t)(LIGHT_SLEEP_DEBUG_DURATION_US / 1000ULL)));
#else
        /* (3)+(5) every driver is already suspended — run_job_lifecycle tore
         * each one down right after it ran (RF modem off; codec/panel/gnss/
         * sensor/imu/lora in standby). Only the rails the *_suspend calls leave
         * energized remain; cut them now (biggest draws left). */
        bsp_exp_output_io_set_level(BSP_LCD_PWR_EN, 0);  /* LCD panel + BL boost */
        bsp_exp_output_io_set_level(BSP_LCD_RST, 0);
        bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 0);   /* SD card */
        bsp_exp_output_io_set_level(BSP_SEN_EN, 0);      /* codecs + sensors */

        /* (6) release the signal pins of every cut rail — pins still driven
         * HIGH back-feed the unpowered chips through their clamp diodes, a
         * major cause of unexpectedly high sleep current. Then hold only the
         * LoRa CS high (its module stays powered). */
        isolate_cut_rail_signal_pins();
        hold_sleep_cs_high();

        /* (7) arm the 60 s timer wake and enter light sleep (resumes here). */
        esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_DURATION_US);
        ESP_LOGI(TAG, "light sleep %llu s",
                 (unsigned long long)(LIGHT_SLEEP_DURATION_US / 1000000ULL));
        vTaskDelay(pdMS_TO_TICKS(50));   /* let the log flush before sleeping */
        esp_err_t sl = esp_light_sleep_start();
        if (sl != ESP_OK) {
            ESP_LOGW(TAG, "light sleep returned %s (woke early?)", esp_err_to_name(sl));
        }

        /* (8) wake: drop the LoRa CS hold, re-power the cut rails. Rail
         * order/settle mirrors bsp_power_startup_peripherals(). */
        release_sleep_cs();
        bsp_exp_output_io_set_level(BSP_SEN_EN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        bsp_exp_output_io_set_level(BSP_SD_PWR_EN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));   /* SD card needs longer to settle */
        bsp_exp_output_io_set_level(BSP_LCD_PWR_EN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        bsp_exp_output_io_set_level(BSP_LCD_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(10));

        /* (9) no bulk re-init: the next loop iteration's run_job_lifecycle
         * re-initializes each peripheral (init is idempotent) right before it
         * steps it, against the freshly re-powered rails. audio_record_step
         * still re-inits the codec before each take, so recording survives the
         * SEN_EN rail cut. */
#endif
        /* (10) loop back to (2). */
    }
}



/* Set to 1 to strip the app down to record -> save -> play-back ONLY (no other
 * peripherals, no low-power loop, no rail cycling). Used to debug the mic
 * capture path in isolation, matching test_pcba microphone_read's minimal
 * context. Set back to 0 for the full low-power feature loop. */
#define AUDIO_ONLY_TEST 0

/* Set to 1 to run a one-boot mic-capture bisection: enable one peripheral at a
 * time after the codec and probe raw_mic after each, to find which peripheral
 * collapses the ES7243E capture. Takes priority over AUDIO_ONLY_TEST. */
#define MIC_BISECT 0

void app_main(void)
{
    heap_debug_checkpoint("before power up");
    /* 1. Power up: powers all rails + I2C_0/I2C_1 + IO expander. */
    esp_err_t ret = bsp_power_up_init();
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Power state is off, skip app startup");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Application software version: SW=%s", APP_SW_VERSION);
    heap_debug_checkpoint("after power up");

    vTaskDelay(pdMS_TO_TICKS(5000));

    heap_debug_checkpoint("before shutdown cb");
    ESP_ERROR_CHECK(bsp_power_shutdown_callback_register(peripherals_cleanup, NULL));
    heap_debug_checkpoint("after shutdown cb");

    /* Mount SD once — recording writes test_recordN.wav to it. */
    heap_debug_checkpoint("before sd mount");
    (void)peripherals_sdcard_mount_boot();
    heap_debug_checkpoint("after sd mount");

    /* Resource mutexes — audio_step takes s_i2c1_lock. */
    heap_debug_checkpoint("before locks");
    ESP_ERROR_CHECK(peripherals_init_locks());
    heap_debug_checkpoint("after locks");

    /* Console + commands, started while heap is plentiful (the codec/REPL heap
     * ordering note below still applies). */
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "fullfeatured>";
    repl_config.max_cmdline_length = 1024;
    repl_config.task_stack_size = 8192;

    ESP_ERROR_CHECK(esp_console_register_help_command());
    audio_cmd_register();

    app_console_start(&repl_config);

#if MIC_BISECT
    /* One-boot bisection: find which peripheral init kills the mic capture.
     * Make continuous sound into the mic for ~15 s after boot. */
    peripherals_mic_bisect();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#elif AUDIO_ONLY_TEST
    /* ---- Minimal audio-only path ----
     * Init the codec once, then loop: record 5 s -> save test_recordN.wav ->
     * play it back, every 10 s. Every other peripheral and the light-sleep
     * loop are skipped so the mic capture can be tested with no concurrent
     * load and no rail cycling — same context as test_pcba microphone_read. */
    audio_init(NULL);
    ESP_LOGI(TAG, "AUDIO_ONLY_TEST: record 5s -> save -> play, every 10s");
    while (true) {
        audio_step(NULL);              /* RECORD mode: record + save + play-back */
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
#else
    /* 2. LoRa DIO IRQ task (created once; shared by TX/RX). */
    heap_debug_checkpoint("before lora irq task");
    peripherals_create_lora_irq_task();
    heap_debug_checkpoint("after lora irq task");

    /* Cache media into PSRAM (only the full runner's play path needs it). */
    heap_debug_checkpoint("before media load");
    peripherals_load_media();
    heap_debug_checkpoint("after media load");

    /* 6. Initialize every peripheral ONCE at power-on (idempotent; warn-and-
     *    continue on per-device failure). Done single-threaded here so the
     *    test steps below run against fully-initialized hardware. */
    heap_debug_checkpoint("before peripheral init");
    peripherals_init_all();
    heap_debug_checkpoint("after peripheral init");

    /* 7. Enter the light-sleep low-power loop: step every peripheral once,
     *    cut backlight + GPS, set WiFi low-power, hold the LoRa/LCD CS pins,
     *    sleep 60 s, wake and restore — then repeat forever. */
    ESP_LOGI(TAG, "Entering light-sleep low-power loop");
    lowpower_loop_run();
#endif
}

