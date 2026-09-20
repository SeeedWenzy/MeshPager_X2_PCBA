
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_console.h"
#include "esp_random.h"
#include "esp_spiffs.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_pm.h"
#include "nvs.h"
#include "driver/rtc_io.h"
#include "driver/usb_serial_jtag.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "nmea.h"
#include "gpgll.h"
#include "gpgga.h"
#include "gprmc.h"
#include "gpgsa.h"
#include "gpvtg.h"
#include "gptxt.h"
#include "gpgsv.h"
#include "meshpager_x2.h"
#include "bsp_gnss_control.h"

#define UART_RX_BUF_SIZE        GNSS_UART_RX_BUF_SIZE

static char s_buf[UART_RX_BUF_SIZE + 1];
static size_t s_total_bytes;
static char *s_last_buf_end;

static void nmea_read_line(char **out_line_buf, size_t *out_line_len, int timeout_ms)
{
    *out_line_buf = NULL;
    *out_line_len = 0;

    if (s_last_buf_end != NULL) {
        /* Data left at the end of the buffer after the last call;
         * copy it to the beginning.
         */
        size_t len_remaining = s_total_bytes - (s_last_buf_end - s_buf);
        memmove(s_buf, s_last_buf_end, len_remaining);
        s_last_buf_end = NULL;
        s_total_bytes = len_remaining;
    }

    /* Read data from the UART */
    int read_bytes = gnss_uart_read_bytes((uint8_t *) s_buf + s_total_bytes,
                                          UART_RX_BUF_SIZE - s_total_bytes,
                                          pdMS_TO_TICKS(timeout_ms));
    if (read_bytes <= 0) {
        return;
    }
    s_total_bytes += read_bytes;

    /* find start (a dollar sign) */
    char *start = memchr(s_buf, '$', s_total_bytes);
    if (start == NULL) {
        s_total_bytes = 0;
        return;
    }

    /* find end of line */
    char *end = memchr(start, '\r', s_total_bytes - (start - s_buf));
    if (end == NULL || *(++end) != '\n') {
        return;
    }
    end++;

    end[-2] = NMEA_END_CHAR_1;
    end[-1] = NMEA_END_CHAR_2;

    *out_line_buf = start;
    *out_line_len = end - start;
    if (end < s_buf + s_total_bytes) {
        /* some data left at the end of the buffer, record its position until the next call */
        s_last_buf_end = end;
    } else {
        s_total_bytes = 0;
    }
}

static void read_and_raw_nmea()
{
    char *start;
    size_t length = 0;
    nmea_read_line(&start, &length, 100 /* ms */);
    if (length) {
        /* start is NOT NUL-terminated; nmea_read_line only signals length
         * separately. Use %.*s with the reported length — %s would read
         * past the line and past s_buf until a random NUL byte, flooding
         * the USB-serial-JTAG TX and eventually tripping the Task Watchdog
         * or a LoadStoreError. The line already ends with \r\n. */
        printf("%.*s", (int) length, start);
        // usb_serial_jtag_write_bytes((const char *)start, length, pdMS_TO_TICKS(100));
        /* printf alone is enough: with CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG,
         * stdout already goes out over USB-Serial-JTAG. Calling
         * usb_serial_jtag_write_bytes() on top of it wrote the same bytes
         * to the same TX path a second time, so every sentence printed twice. */
    }
}

static void read_and_parse_nmea()
{
    char fmt_buf[32];
    nmea_s *data;

    char *start;
    size_t length;
    nmea_read_line(&start, &length, 100 /* ms */);
    if (length == 0) {
        return;
    }

    /* handle data */
    data = nmea_parse(start, length, 0);
    if (data == NULL) {
        printf("Failed to parse the sentence!\n");
        printf("  Type: %.5s (%d)\n", start + 1, nmea_get_type(start));
    } else {
        if (data->errors != 0) {
            printf("WARN: The sentence struct contains parse errors!\n");
        }

        if (NMEA_GPGGA == data->type) {
            printf("GPGGA sentence\n");
            nmea_gpgga_s *gpgga = (nmea_gpgga_s *) data;
            printf("Number of satellites: %d\n", gpgga->n_satellites);
            printf("Altitude: %f %c\n", gpgga->altitude,
                    gpgga->altitude_unit);
        }

        if (NMEA_GPGLL == data->type) {
            printf("GPGLL sentence\n");
            nmea_gpgll_s *pos = (nmea_gpgll_s *) data;
            printf("Longitude:\n");
            printf("  Degrees: %d\n", pos->longitude.degrees);
            printf("  Minutes: %f\n", pos->longitude.minutes);
            printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);
            printf("Latitude:\n");
            printf("  Degrees: %d\n", pos->latitude.degrees);
            printf("  Minutes: %f\n", pos->latitude.minutes);
            printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);
            strftime(fmt_buf, sizeof(fmt_buf), "%H:%M:%S", &pos->time);
            printf("Time: %s\n", fmt_buf);
        }

        if (NMEA_GPRMC == data->type) {
            printf("GPRMC sentence\n");
            nmea_gprmc_s *pos = (nmea_gprmc_s *) data;
            printf("Longitude:\n");
            printf("  Degrees: %d\n", pos->longitude.degrees);
            printf("  Minutes: %f\n", pos->longitude.minutes);
            printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);
            printf("Latitude:\n");
            printf("  Degrees: %d\n", pos->latitude.degrees);
            printf("  Minutes: %f\n", pos->latitude.minutes);
            printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);
            strftime(fmt_buf, sizeof(fmt_buf), "%d %b %T %Y", &pos->date_time);
            printf("Date & Time: %s\n", fmt_buf);
            printf("Speed, in Knots: %f\n", pos->gndspd_knots);
            printf("Track, in degrees: %f\n", pos->track_deg);
            printf("Magnetic Variation:\n");
            printf("  Degrees: %f\n", pos->magvar_deg);
            printf("  Cardinal: %c\n", (char) pos->magvar_cardinal);
            double adjusted_course = pos->track_deg;
            if (NMEA_CARDINAL_DIR_EAST == pos->magvar_cardinal) {
                adjusted_course -= pos->magvar_deg;
            } else if (NMEA_CARDINAL_DIR_WEST == pos->magvar_cardinal) {
                adjusted_course += pos->magvar_deg;
            } else {
                printf("Invalid Magnetic Variation Direction!\n");
            }

            printf("Adjusted Track (heading): %f\n", adjusted_course);
        }

        if (NMEA_GPGSA == data->type) {
            nmea_gpgsa_s *gpgsa = (nmea_gpgsa_s *) data;

            printf("GPGSA Sentence:\n");
            printf("  Mode: %c\n", gpgsa->mode);
            printf("  Fix:  %d\n", gpgsa->fixtype);
            printf("  PDOP: %.2lf\n", gpgsa->pdop);
            printf("  HDOP: %.2lf\n", gpgsa->hdop);
            printf("  VDOP: %.2lf\n", gpgsa->vdop);
        }

        if (NMEA_GPGSV == data->type) {
            nmea_gpgsv_s *gpgsv = (nmea_gpgsv_s *) data;

            printf("GPGSV Sentence:\n");
            printf("  Num: %d\n", gpgsv->sentences);
            printf("  ID:  %d\n", gpgsv->sentence_number);
            printf("  SV:  %d\n", gpgsv->satellites);
            printf("  #1:  %d %d %d %d\n", gpgsv->sat[0].prn, gpgsv->sat[0].elevation, gpgsv->sat[0].azimuth, gpgsv->sat[0].snr);
            printf("  #2:  %d %d %d %d\n", gpgsv->sat[1].prn, gpgsv->sat[1].elevation, gpgsv->sat[1].azimuth, gpgsv->sat[1].snr);
            printf("  #3:  %d %d %d %d\n", gpgsv->sat[2].prn, gpgsv->sat[2].elevation, gpgsv->sat[2].azimuth, gpgsv->sat[2].snr);
            printf("  #4:  %d %d %d %d\n", gpgsv->sat[3].prn, gpgsv->sat[3].elevation, gpgsv->sat[3].azimuth, gpgsv->sat[3].snr);
        }

        if (NMEA_GPTXT == data->type) {
            nmea_gptxt_s *gptxt = (nmea_gptxt_s *) data;

            printf("GPTXT Sentence:\n");
            printf("  ID: %d %d %d\n", gptxt->id_00, gptxt->id_01, gptxt->id_02);
            printf("  %s\n", gptxt->text);
        }

        if (NMEA_GPVTG == data->type) {
            nmea_gpvtg_s *gpvtg = (nmea_gpvtg_s *) data;

            printf("GPVTG Sentence:\n");
            printf("  Track [deg]:   %.2lf\n", gpvtg->track_deg);
            printf("  Speed [kmph]:  %.2lf\n", gpvtg->gndspd_kmph);
            printf("  Speed [knots]: %.2lf\n", gpvtg->gndspd_knots);
        }

        nmea_free(data);
    }
}

static struct {
    struct arg_int *mode;
    struct arg_int *baud;
    struct arg_end *end;
} app_gnss_parse_args;

static struct {
    struct arg_end *end;
} app_gnss_flash_args;

static int app_gnss_parse_test(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &app_gnss_parse_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, app_gnss_parse_args.end, argv[0]);
        return 1;
    }

    int mode = 0;
    if (app_gnss_parse_args.mode->count) {
        mode = app_gnss_parse_args.mode->ival[0];
        if (mode == 0) {
            ESP_LOGI("GNSS",  "gnss raw begin");
        } else if (mode == 1) {
            ESP_LOGI("GNSS",  "gnss parse begin");
        } else {
            ESP_LOGE("GNSS", "unsupported mode: %d", mode);
            return 1;
        }
    }

    uint32_t baud = 115200;
    if (app_gnss_parse_args.baud->count) {
        baud = (uint32_t) app_gnss_parse_args.baud->ival[0];
        if (baud == 0) {
            ESP_LOGE("GNSS", "invalid baud: %lu", (unsigned long) baud);
            return 1;
        }
        ESP_LOGI("GNSS", "gnss baud: %lu", (unsigned long) baud);
    }

    s_total_bytes = 0;
    s_last_buf_end = NULL;

    if (!bsp_gnss_scan_start_with_baud(baud)) {
        ESP_LOGE("GNSS", "failed to start gnss scan");
        return 1;
    }

    uint8_t quit = 0;
    int recv_len = 0;
    while(1) {
        recv_len = usb_serial_jtag_read_bytes(&quit, 1, pdMS_TO_TICKS(1));
        if( recv_len > 0) {
            if( quit == 0x3) { //Ctrl+C
                break;
            }
        }

        if (mode == 0) {
            read_and_raw_nmea();
        } else if (mode == 1) {
            read_and_parse_nmea();
        }

        /* Floor yield: when the GNSS RX is flooded with garbage (e.g. baud
         * mismatch), every blocking read above returns immediately and the
         * loop would spin without giving CPU0's IDLE task any time — which
         * trips the Task Watchdog. One tick here guarantees a yield per
         * iteration regardless of how fast the reads return. */
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    bsp_gnss_scan_stop();

    ESP_LOGI("GNSS",  "gnss test end");

    return 0;
}

static int app_gnss_flash_mode(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &app_gnss_flash_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, app_gnss_flash_args.end, argv[0]);
        return 1;
    }

    bsp_gnss_uart_deinit();
    bsp_gnss_set_io_input_pullup();
    
    bsp_gnss_reset();
    ESP_LOGI("GNSS", "gnss flash mode ready");

    return 0;
}

static void register_app_gnss_parse_test(void)
{
    app_gnss_parse_args.mode =
        arg_int0("m", "mode", "<0|1>", "0: raw gnss data, 1: parse gnss data, default: 0");
    app_gnss_parse_args.baud =
        arg_int0("b", "baud", "<baud>", "UART baud, e.g. 115200/921600, default: 115200");
    app_gnss_parse_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "gnss",
        .help = "gnss parse test. -m 0: raw, -m 1: parse; -b <baud> (default 115200)",
        .hint = NULL,
        .func = &app_gnss_parse_test,
        .argtable = &app_gnss_parse_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void register_app_gnss_flash_mode(void)
{
    app_gnss_flash_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "gnss_flash",
        .help = "prepare GNSS IO for firmware download",
        .hint = NULL,
        .func = &app_gnss_flash_mode,
        .argtable = &app_gnss_flash_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

void gnss_cmd_register_all(void)
{
    register_app_gnss_parse_test();
    register_app_gnss_flash_mode();
}
