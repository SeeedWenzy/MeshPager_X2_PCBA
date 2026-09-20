#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lr20xx_hal.h"
#include "lr20xx_init.h"
#include "lr20xx_radio_common.h"
#include "meshpager_x2.h"
#include "ral.h"
#include "ral_defs.h"
#include "ralf.h"
#include "ralf_defs.h"
#include "ralf_lr20xx.h"
#include "radio_utilities.h"

#include "lora_rx_test.h"

static const char *TAG = "COEX_LORA";

#define LORA_FREQUENCY_HZ       (915000000U)
#define LORA_RX_TIMEOUT_SYMBOLS (100U)
#define LORA_RX_WAIT_MS         (2500U)
#define LORA_POLL_AFTER_MS      (200U)

static lr20xx_hal_context_t s_radio;
static const ralf_t s_modem_radio = RALF_LR20XX_INSTANTIATE(&s_radio);
static TaskHandle_t s_irq_task;
static TaskHandle_t s_rx_task;
static bool s_irq_installed;
static bool s_ready;
static volatile bool s_rx_done;
static volatile bool s_rx_timeout;

static void lora_dio_isr(void *arg)
{
    (void)arg;
    if (s_irq_task != NULL) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_irq_task, &woken);
        if (woken) {
            portYIELD_FROM_ISR();
        }
    }
}

static void lora_irq_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ral_irq_t irq = 0;
        if (ral_get_and_clear_irq_status(&s_modem_radio.ral, &irq) != RAL_STATUS_OK) {
            ESP_LOGW(TAG, "read IRQ status failed");
            continue;
        }
        if (irq & RAL_IRQ_RX_DONE) {
            s_rx_done = true;
        }
        if (irq & RAL_IRQ_RX_TIMEOUT) {
            s_rx_timeout = true;
        }
    }
}

static esp_err_t lora_init_once(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    lr20xx_init(&s_radio);
    if (ral_reset(&s_modem_radio.ral) != RAL_STATUS_OK ||
        ral_init(&s_modem_radio.ral) != RAL_STATUS_OK) {
        return ESP_FAIL;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CONFIG_RADIO_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "LoRa DIO GPIO config failed");
    if (!s_irq_installed) {
        ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CONFIG_RADIO_INT_GPIO, lora_dio_isr, NULL),
                            TAG, "LoRa DIO ISR install failed");
        s_irq_installed = true;
    }
    s_ready = true;
    return ESP_OK;
}

static void lora_rx_task(void *arg)
{
    (void)arg;
    const ralf_params_lora_t params = {
        .sync_word = 0x12,
        .symb_nb_timeout = LORA_RX_TIMEOUT_SYMBOLS,
        .rf_freq_in_hz = LORA_FREQUENCY_HZ,
        .output_pwr_in_dbm = 0,
        .mod_params.cr = RAL_LORA_CR_4_5,
        .mod_params.sf = RAL_LORA_SF7,
        .mod_params.bw = RAL_LORA_BW_125_KHZ,
        .mod_params.ldro = false,
        .pkt_params.header_type = RAL_LORA_PKT_EXPLICIT,
        .pkt_params.pld_len_in_bytes = 255,
        .pkt_params.crc_is_on = true,
        .pkt_params.invert_iq_is_on = false,
        .pkt_params.preamble_len_in_symb = 8,
    };

    for (;;) {
        if (ralf_setup_lora(&s_modem_radio, &params) != RAL_STATUS_OK ||
            ral_set_dio_irq_params(&s_modem_radio.ral, RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT) != RAL_STATUS_OK ||
            ral_set_rx(&s_modem_radio.ral, LORA_RX_TIMEOUT_SYMBOLS) != RAL_STATUS_OK) {
            ESP_LOGE(TAG, "LoRa RX setup failed; retrying");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        s_rx_done = false;
        s_rx_timeout = false;
        const int64_t deadline = esp_timer_get_time() + (int64_t)LORA_RX_WAIT_MS * 1000;
        while (!s_rx_done && !s_rx_timeout && esp_timer_get_time() < deadline) {
            /* Keep the RX task schedulable while Wi-Fi/BLE coexistence tasks run. */
            vTaskDelay(pdMS_TO_TICKS(10));
            if (esp_timer_get_time() - (deadline - (int64_t)LORA_RX_WAIT_MS * 1000) >
                (int64_t)LORA_POLL_AFTER_MS * 1000) {
                ral_irq_t irq = 0;
                if (ral_get_and_clear_irq_status(&s_modem_radio.ral, &irq) == RAL_STATUS_OK) {
                    s_rx_done |= (irq & RAL_IRQ_RX_DONE) != 0;
                    s_rx_timeout |= (irq & RAL_IRQ_RX_TIMEOUT) != 0;
                }
            }
        }

        if (s_rx_done) {
            uint8_t payload[256];
            uint16_t length = 0;
            if (ral_get_pkt_payload(&s_modem_radio.ral, sizeof(payload), payload, &length) == RAL_STATUS_OK) {
                ESP_LOGI(TAG, "LoRa RX packet len=%u", (unsigned)length);
                if (length > 0 && length <= 32) {
                    ESP_LOG_BUFFER_HEXDUMP(TAG, payload, length, ESP_LOG_INFO);
                }
            }
        }
    }
}

esp_err_t lora_rx_test_start(void)
{
    if (s_rx_task != NULL) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(lora_init_once(), TAG, "LoRa init failed");
    if (s_irq_task == NULL) {
        if (xTaskCreate(lora_irq_task, "coex_lora_irq", 3072, NULL, 8, &s_irq_task) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (xTaskCreatePinnedToCore(lora_rx_task, "coex_lora_rx", 4096, NULL, 5, &s_rx_task, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "LoRa RX started: %u Hz, SF7, BW125, sync=0x12", LORA_FREQUENCY_HZ);
    return ESP_OK;
}
