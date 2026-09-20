#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>

#include "argtable3/argtable3.h"
#include "esp_check.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pngle.h"

#include "meshpager_x2.h"
#include "sd_lcd_cmd.h"
#include "app_storage.h"

static const char *TAG = "SD_LCD_TEST";

#define LCD_V_RES 320
#define LCD_H_RES 240

#define SD_MOUNT_POINT "/sdcard"
#define SD_IMAGE_DIR SD_MOUNT_POINT "/images"
#define SD_DEFAULT_IMAGE SD_MOUNT_POINT "/frame.rgb565"
#define SD_BENCH_FILE SD_MOUNT_POINT "/sd_bench.bin"
/* Dedicated directory for raw RGB565 frames used by the FPS test. Keeping the
 * bench frames out of SD_IMAGE_DIR lets /sdcard/images hold displayable PNGs
 * while lcd_fps_test pulls raw frames from here. */
#define SD_RGB565_DIR SD_MOUNT_POINT "/rgb565"

#define FRAME_WIDTH LCD_H_RES
#define FRAME_HEIGHT LCD_V_RES
#define FRAME_BUFFER_SIZE (FRAME_WIDTH * FRAME_HEIGHT * sizeof(uint16_t))

/* Resolve an SD-rooted asset path to the path that should actually be used.
 * When the SD card is available, the input is returned unchanged (points at
 * the static literal). When the SD card is unavailable, the "/sdcard" prefix
 * is rewritten to the SPIFFS asset mount point so the baked-in assets under
 * /spiffs/images and /spiffs/rgb565 are used instead. The result is written
 * into buf (when remapping) and returned; NULL on error. */
static const char *lcd_resolve_asset_path(const char *sd_path, char *buf, size_t buf_size)
{
    static const char sd_prefix[] = "/sdcard";
    const size_t sd_prefix_len = sizeof(sd_prefix) - 1;
    const char *tail;

    if (sd_path == NULL || buf == NULL || buf_size == 0) {
        return NULL;
    }

    if (app_storage_sd_available()) {
        return sd_path;
    }

    /* Rewrite "/sdcard<rest>" -> "<SPIFFS><rest>". tail keeps the leading '/'
     * (or is empty for the bare mount point) so the result is well-formed. */
    if (strncmp(sd_path, sd_prefix, sd_prefix_len) == 0) {
        tail = sd_path + sd_prefix_len;            /* "" or "/images" ... */
    } else if (sd_path[0] == '/') {
        tail = sd_path;                              /* other absolute path */
    } else {
        if (snprintf(buf, buf_size, "%s/%s", BSP_SPIFFS_MOUNT_POINT, sd_path) >= (int)buf_size) {
            return NULL;
        }
        return buf;
    }

    if (tail[0] == '\0') {
        if (strlcpy(buf, BSP_SPIFFS_MOUNT_POINT, buf_size) >= buf_size) {
            return NULL;
        }
    } else {
        if (snprintf(buf, buf_size, "%s%s", BSP_SPIFFS_MOUNT_POINT, tail) >= (int)buf_size) {
            return NULL;
        }
    }
    return buf;
}

#define SD_BENCH_TOTAL_BYTES (8 * 1024 * 1024)
#define SD_BENCH_CHUNK_BYTES (64 * 1024)
#define SD_BENCH_MIN_CHUNK_BYTES (4 * 1024)
#define SD_IMAGE_MAX_FILES 32
#define SD_IMAGE_PATH_MAX_LEN 160
#define SD_SCAN_MAX_DEPTH 3
#define SD_LIST_MAX_DEPTH 3
#define SD_LIST_MAX_ENTRIES 64
#define FPS_OVERLAY_X 4
#define FPS_OVERLAY_Y 4
#define FPS_OVERLAY_WIDTH 176
#define FPS_OVERLAY_HEIGHT 68
#define FPS_FONT_WIDTH 5
#define FPS_FONT_HEIGHT 7
#define FPS_TEXT_SCALE 2
#define FPS_CHUNK_LINES 16
#define FPS_UPDATE_PERIOD_US 250000
#define FPS_OVERLAY_BUFFER_SIZE (FPS_OVERLAY_WIDTH * FPS_OVERLAY_HEIGHT * sizeof(uint16_t))
#define PNG_READ_CHUNK_SIZE 1024

typedef enum {
    SD_IMAGE_TYPE_RGB565 = 0,
    SD_IMAGE_TYPE_PNG,
} sd_image_type_t;

typedef enum {
    FPS_MODE_NONE = 0,
    FPS_MODE_SD,
    FPS_MODE_PSRAM,
} fps_mode_t;

typedef struct {
    char path[SD_IMAGE_PATH_MAX_LEN];
    uint8_t *data;
    size_t size;
    bool preloaded;
    sd_image_type_t type;
} sd_image_entry_t;

typedef struct {
    uint16_t *line_buffer;
    uint16_t *frame_buffer;   /* full-frame RGB565 target (FRAME_WIDTH*FRAME_HEIGHT) */
    uint32_t image_width;
    uint32_t image_height;
    uint32_t display_width;
    uint32_t display_height;
    uint32_t current_row;
    int32_t x_offset;
    int32_t y_offset;
    bool row_dirty;
    bool decode_done;
    esp_err_t status;
} png_display_ctx_t;

typedef struct {
    struct arg_str *mode;
    struct arg_int *chunk_kb;
    struct arg_int *total_kb;
    struct arg_end *end;
} sd_bench_args_t;

typedef struct {
    struct arg_str *action;
    struct arg_str *mode;
    struct arg_int *interval_ms;
    struct arg_end *end;
} lcd_fps_args_t;

static sd_bench_args_t s_sd_bench_args;
static lcd_fps_args_t s_lcd_fps_args;

static SemaphoreHandle_t s_lvgl_mutex;
static SemaphoreHandle_t s_state_mutex;
static TaskHandle_t s_fps_task_handle;
static bool s_lvgl_ready;
static bool s_sd_scan_done;
static bool s_psram_preload_done;
static bool s_preload_attempted;
static bool s_fps_task_running;
static fps_mode_t s_active_mode = FPS_MODE_NONE;
static uint32_t s_switch_interval_ms = 0;
static uint32_t s_frames_displayed = 0;
static uint32_t s_last_fps_value = 0;
static uint32_t s_frames_since_update = 0;
static int64_t s_last_fps_update_us = 0;
static int64_t s_run_start_us = 0;
static size_t s_selected_image_index;
static sd_image_entry_t s_images[SD_IMAGE_MAX_FILES];
static size_t s_image_count;
/* Records which root directory the current s_images[] list was scanned from,
 * so switching between /sdcard/images (display) and /sdcard/rgb565 (FPS) can
 * invalidate the cache and rescan. Empty string means "not scanned yet". */
static char s_scan_root_used[SD_IMAGE_PATH_MAX_LEN];
static uint8_t *s_sd_frame_buffer;
static uint16_t *s_fps_compose_buffer;
static uint16_t *s_fps_overlay_buffer;
static uint8_t *s_bench_buffer;
static size_t s_bench_buffer_size;
static uint16_t *s_png_line_buffer;

static bool lock_take(SemaphoreHandle_t mutex, TickType_t timeout)
{
    return (mutex != NULL) && (xSemaphoreTake(mutex, timeout) == pdTRUE);
}

static void set_run_state(fps_mode_t mode, uint32_t interval_ms);

static esp_err_t show_image_locked(const uint8_t *buffer);

static const char *fps_mode_name(fps_mode_t mode)
{
    switch (mode) {
    case FPS_MODE_SD:
        return "sd";
    case FPS_MODE_PSRAM:
        return "psram";
    default:
        return "idle";
    }
}

static void make_indent(int depth, char *buffer, size_t buffer_size)
{
    size_t offset = 0;

    if (buffer_size == 0) {
        return;
    }

    for (int i = 0; i < depth && (offset + 2) < buffer_size; i++) {
        buffer[offset++] = ' ';
        buffer[offset++] = ' ';
    }
    buffer[offset] = '\0';
}

static void format_overlay_file_name(const char *file_name, char *buffer, size_t buffer_size)
{
    const char *base_name = file_name;

    if (buffer_size == 0) {
        return;
    }

    if (base_name != NULL) {
        const char *slash = strrchr(file_name, '/');
        if (slash != NULL) {
            base_name = slash + 1;
        }
    }

    if ((base_name == NULL) || (base_name[0] == '\0')) {
        snprintf(buffer, buffer_size, "-");
        return;
    }

    size_t name_len = strlen(base_name);
    if (name_len <= 18) {
        snprintf(buffer, buffer_size, "%s", base_name);
        return;
    }

    snprintf(buffer, buffer_size, "%.7s...%.8s", base_name, base_name + name_len - 8);
}

static const uint8_t *fps_get_glyph(char ch)
{
    static const uint8_t glyph_space[7] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t glyph_dash[7] = { 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00 };
    static const uint8_t glyph_colon[7] = { 0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00 };
    static const uint8_t glyph_dot[7] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C };
    static const uint8_t glyph_slash[7] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x00, 0x00 };
    static const uint8_t glyph_0[7] = { 0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E };
    static const uint8_t glyph_1[7] = { 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E };
    static const uint8_t glyph_2[7] = { 0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F };
    static const uint8_t glyph_3[7] = { 0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E };
    static const uint8_t glyph_4[7] = { 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02 };
    static const uint8_t glyph_5[7] = { 0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E };
    static const uint8_t glyph_6[7] = { 0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E };
    static const uint8_t glyph_7[7] = { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 };
    static const uint8_t glyph_8[7] = { 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E };
    static const uint8_t glyph_9[7] = { 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C };
    static const uint8_t glyph_A[7] = { 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 };
    static const uint8_t glyph_B[7] = { 0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E };
    static const uint8_t glyph_C[7] = { 0x0F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0F };
    static const uint8_t glyph_D[7] = { 0x1E, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1E };
    static const uint8_t glyph_E[7] = { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F };
    static const uint8_t glyph_F[7] = { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10 };
    static const uint8_t glyph_G[7] = { 0x0F, 0x10, 0x10, 0x13, 0x11, 0x11, 0x0F };
    static const uint8_t glyph_I[7] = { 0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E };
    static const uint8_t glyph_M[7] = { 0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11 };
    static const uint8_t glyph_O[7] = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E };
    static const uint8_t glyph_P[7] = { 0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10 };
    static const uint8_t glyph_R[7] = { 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11 };
    static const uint8_t glyph_S[7] = { 0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E };
    static const uint8_t glyph_T[7] = { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 };
    static const uint8_t glyph_U[7] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E };
    static const uint8_t glyph_V[7] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04 };
    static const uint8_t glyph_W[7] = { 0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A };

    switch (ch) {
    case ' ': return glyph_space;
    case '-': return glyph_dash;
    case ':': return glyph_colon;
    case '.': return glyph_dot;
    case '/': return glyph_slash;
    case '0': return glyph_0;
    case '1': return glyph_1;
    case '2': return glyph_2;
    case '3': return glyph_3;
    case '4': return glyph_4;
    case '5': return glyph_5;
    case '6': return glyph_6;
    case '7': return glyph_7;
    case '8': return glyph_8;
    case '9': return glyph_9;
    case 'A': case 'a': return glyph_A;
    case 'B': case 'b': return glyph_B;
    case 'C': case 'c': return glyph_C;
    case 'D': case 'd': return glyph_D;
    case 'E': case 'e': return glyph_E;
    case 'F': case 'f': return glyph_F;
    case 'G': case 'g': return glyph_G;
    case 'I': case 'i': return glyph_I;
    case 'M': case 'm': return glyph_M;
    case 'O': case 'o': return glyph_O;
    case 'P': case 'p': return glyph_P;
    case 'R': case 'r': return glyph_R;
    case 'S': case 's': return glyph_S;
    case 'T': case 't': return glyph_T;
    case 'U': case 'u': return glyph_U;
    case 'V': case 'v': return glyph_V;
    case 'W': case 'w': return glyph_W;
    default: return glyph_space;
    }
}

static uint16_t fps_dim_color(uint16_t color)
{
    return (uint16_t)(((color & 0xF7DEU) >> 1) & 0x7BEFU);
}

static void fps_overlay_draw_pixel(int x, int y, uint16_t color)
{
    if (s_fps_overlay_buffer == NULL) {
        return;
    }
    if (x < 0 || y < 0 || x >= FPS_OVERLAY_WIDTH || y >= FPS_OVERLAY_HEIGHT) {
        return;
    }
    s_fps_overlay_buffer[y * FPS_OVERLAY_WIDTH + x] = color;
}

static void fps_overlay_fill_rect(int x, int y, int width, int height, uint16_t color)
{
    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
            fps_overlay_draw_pixel(x + col, y + row, color);
        }
    }
}

static void fps_overlay_draw_char(int x, int y, char ch, uint16_t fg_color, uint16_t bg_color, int scale)
{
    const uint8_t *glyph = fps_get_glyph(ch);

    for (int row = 0; row < FPS_FONT_HEIGHT; row++) {
        for (int col = 0; col < FPS_FONT_WIDTH; col++) {
            bool pixel_on = (glyph[row] & (1U << (FPS_FONT_WIDTH - 1 - col))) != 0U;
            uint16_t color = pixel_on ? fg_color : bg_color;

            for (int dy = 0; dy < scale; dy++) {
                for (int dx = 0; dx < scale; dx++) {
                    fps_overlay_draw_pixel(x + col * scale + dx, y + row * scale + dy, color);
                }
            }
        }
    }
}

static void fps_overlay_draw_text(int x, int y, const char *text, uint16_t fg_color, uint16_t bg_color, int scale)
{
    int cursor_x = x;

    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        if (*text == '\n') {
            cursor_x = x;
            y += (FPS_FONT_HEIGHT + 2) * scale;
            text++;
            continue;
        }
        fps_overlay_draw_char(cursor_x, y, *text, fg_color, bg_color, scale);
        cursor_x += (FPS_FONT_WIDTH + 1) * scale;
        text++;
    }
}

static esp_err_t ensure_fps_overlay_buffer(void)
{
    if (s_fps_overlay_buffer != NULL) {
        return ESP_OK;
    }

    s_fps_overlay_buffer = (uint16_t *)heap_caps_malloc(FPS_OVERLAY_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_fps_overlay_buffer == NULL) {
        s_fps_overlay_buffer = (uint16_t *)heap_caps_malloc(FPS_OVERLAY_BUFFER_SIZE, MALLOC_CAP_8BIT);
    }

    return (s_fps_overlay_buffer != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t ensure_fps_compose_buffer(void)
{
    const size_t buffer_size = FRAME_BUFFER_SIZE;

    if (s_fps_compose_buffer != NULL) {
        return ESP_OK;
    }

    s_fps_compose_buffer = (uint16_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_fps_compose_buffer == NULL) {
        s_fps_compose_buffer = (uint16_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_8BIT);
    }

    return (s_fps_compose_buffer != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t ensure_png_line_buffer(void)
{
    if (s_png_line_buffer != NULL) {
        return ESP_OK;
    }

    s_png_line_buffer = (uint16_t *)heap_caps_malloc(FRAME_WIDTH * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_png_line_buffer == NULL) {
        s_png_line_buffer = (uint16_t *)heap_caps_malloc(FRAME_WIDTH * sizeof(uint16_t), MALLOC_CAP_8BIT);
    }

    return (s_png_line_buffer != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t draw_fps_overlay(const uint16_t *frame_data, uint32_t fps_value, const char *mode_name, const char *file_name)
{
    char short_name[24];
    char line1[24];
    char line2[24];
    char line3[24];
    const uint16_t bg_color = 0x0000;

    if (frame_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(ensure_fps_overlay_buffer(), TAG, "alloc fps overlay failed");

    for (int y = 0; y < FPS_OVERLAY_HEIGHT; y++) {
        const uint16_t *src = frame_data + (FPS_OVERLAY_Y + y) * FRAME_WIDTH + FPS_OVERLAY_X;
        uint16_t *dst = s_fps_overlay_buffer + y * FPS_OVERLAY_WIDTH;
        memcpy(dst, src, FPS_OVERLAY_WIDTH * sizeof(uint16_t));
        for (int x = 0; x < FPS_OVERLAY_WIDTH; x++) {
            dst[x] = fps_dim_color(fps_dim_color(dst[x]));
        }
    }

    fps_overlay_fill_rect(0, 0, FPS_OVERLAY_WIDTH, 2, 0x07FF);
    fps_overlay_fill_rect(0, FPS_OVERLAY_HEIGHT - 2, FPS_OVERLAY_WIDTH, 2, 0x07FF);
    fps_overlay_fill_rect(0, 0, 2, FPS_OVERLAY_HEIGHT, 0x07FF);
    fps_overlay_fill_rect(FPS_OVERLAY_WIDTH - 2, 0, 2, FPS_OVERLAY_HEIGHT, 0x07FF);

    format_overlay_file_name(file_name, short_name, sizeof(short_name));
    snprintf(line1, sizeof(line1), "FPS %3lu", (unsigned long)fps_value);
    snprintf(line2, sizeof(line2), "SRC %s %lums", (mode_name != NULL) ? mode_name : "idle", (unsigned long)s_switch_interval_ms);
    snprintf(line3, sizeof(line3), "IMG %s", short_name);

    fps_overlay_draw_text(8, 6, line1, 0xFFFF, bg_color, FPS_TEXT_SCALE);
    fps_overlay_draw_text(8, 24, line2, 0x07E0, bg_color, 1);
    fps_overlay_draw_text(8, 38, line3, 0xFFFF, bg_color, 1);

    return ESP_OK;
}

static esp_err_t prepare_fps_frame(const uint16_t *frame_data, uint32_t fps_value, const char *mode_name, const char *file_name)
{
    ESP_RETURN_ON_ERROR(ensure_fps_compose_buffer(), TAG, "alloc fps compose buffer failed");

    if (frame_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(s_fps_compose_buffer, frame_data, FRAME_BUFFER_SIZE);

    ESP_RETURN_ON_ERROR(draw_fps_overlay(frame_data, fps_value, mode_name, file_name), TAG, "prepare fps overlay failed");

    for (int row = 0; row < FPS_OVERLAY_HEIGHT; row++) {
        uint16_t *dst = s_fps_compose_buffer + (FPS_OVERLAY_Y + row) * FRAME_WIDTH + FPS_OVERLAY_X;
        const uint16_t *src = s_fps_overlay_buffer + row * FPS_OVERLAY_WIDTH;
        memcpy(dst, src, FPS_OVERLAY_WIDTH * sizeof(uint16_t));
    }

    return ESP_OK;
}

static void lock_give(SemaphoreHandle_t mutex)
{
    if (mutex != NULL) {
        xSemaphoreGive(mutex);
    }
}

static uint16_t rgba_to_rgb565_over_black(const uint8_t rgba[4])
{
    uint32_t alpha = rgba[3];
    uint8_t red = (uint8_t)((rgba[0] * alpha + 127U) / 255U);
    uint8_t green = (uint8_t)((rgba[1] * alpha + 127U) / 255U);
    uint8_t blue = (uint8_t)((rgba[2] * alpha + 127U) / 255U);

    return (uint16_t)(((red & 0xF8U) << 8) | ((green & 0xFCU) << 3) | (blue >> 3));
}

static esp_err_t flush_png_row_locked(png_display_ctx_t *ctx)
{
    if (ctx == NULL || ctx->line_buffer == NULL || ctx->frame_buffer == NULL ||
        !ctx->row_dirty || ctx->current_row >= ctx->image_height) {
        return ESP_OK;
    }

    uint32_t y_start = (ctx->current_row * ctx->display_height) / ctx->image_height;
    uint32_t y_end = ((ctx->current_row + 1U) * ctx->display_height) / ctx->image_height;
    if (y_end > ctx->display_height) {
        y_end = ctx->display_height;
    }
    if (y_start >= ctx->display_height) {
        y_start = (ctx->display_height > 0) ? (ctx->display_height - 1U) : 0U;
    }

    /* Commit the decoded source row into the full-frame buffer at the scaled
     * vertical range. No SPI traffic here -- the whole frame is blitted in
     * 16-line chunks after decode completes, matching the smooth RGB565 path. */
    size_t copy_bytes = (size_t)ctx->display_width * sizeof(uint16_t);
    for (uint32_t y = y_start; y < y_end; y++) {
        uint16_t *dst = ctx->frame_buffer +
                        (size_t)(ctx->y_offset + (int32_t)y) * FRAME_WIDTH +
                        (size_t)ctx->x_offset;
        memcpy(dst, ctx->line_buffer, copy_bytes);
    }

    return ESP_OK;
}

static void png_init_callback(pngle_t *pngle, uint32_t width, uint32_t height)
{
    png_display_ctx_t *ctx = (png_display_ctx_t *)pngle_get_user_data(pngle);
    pngle_ihdr_t *ihdr = pngle_get_ihdr(pngle);

    if (ctx == NULL || ctx->line_buffer == NULL || ctx->status != ESP_OK) {
        return;
    }

    if (ihdr != NULL && ihdr->interlace != 0) {
        ctx->status = ESP_ERR_NOT_SUPPORTED;
        return;
    }

    if (width == 0 || height == 0 || width > FRAME_WIDTH || height > FRAME_HEIGHT) {
        if ((uint64_t)FRAME_WIDTH * height <= (uint64_t)FRAME_HEIGHT * width) {
            ctx->display_width = FRAME_WIDTH;
            ctx->display_height = (uint32_t)(((uint64_t)height * FRAME_WIDTH) / width);
        } else {
            ctx->display_height = FRAME_HEIGHT;
            ctx->display_width = (uint32_t)(((uint64_t)width * FRAME_HEIGHT) / height);
        }

        if (ctx->display_width == 0) {
            ctx->display_width = 1;
        }
        if (ctx->display_height == 0) {
            ctx->display_height = 1;
        }
    } else {
        if ((uint64_t)FRAME_WIDTH * height <= (uint64_t)FRAME_HEIGHT * width) {
            ctx->display_width = FRAME_WIDTH;
            ctx->display_height = (uint32_t)(((uint64_t)height * FRAME_WIDTH) / width);
        } else {
            ctx->display_height = FRAME_HEIGHT;
            ctx->display_width = (uint32_t)(((uint64_t)width * FRAME_HEIGHT) / height);
        }

        if (ctx->display_width == 0) {
            ctx->display_width = 1;
        }
        if (ctx->display_height == 0) {
            ctx->display_height = 1;
        }
    }

    if (ctx->display_width > FRAME_WIDTH || ctx->display_height > FRAME_HEIGHT) {
        ctx->status = ESP_ERR_NOT_SUPPORTED;
        return;
    }

    ctx->image_width = width;
    ctx->image_height = height;
    ctx->current_row = UINT32_MAX;
    ctx->x_offset = (FRAME_WIDTH - (int32_t)ctx->display_width) / 2;
    ctx->y_offset = (FRAME_HEIGHT - (int32_t)ctx->display_height) / 2;
    ctx->row_dirty = false;
}

static void png_draw_callback(pngle_t *pngle, uint32_t x, uint32_t y, uint32_t width, uint32_t height, const uint8_t rgba[4])
{
    png_display_ctx_t *ctx = (png_display_ctx_t *)pngle_get_user_data(pngle);

    if (ctx == NULL || ctx->line_buffer == NULL || ctx->status != ESP_OK) {
        return;
    }

    if (width != 1 || height != 1 || x >= ctx->image_width || y >= ctx->image_height) {
        ctx->status = ESP_ERR_NOT_SUPPORTED;
        return;
    }

    if (ctx->current_row != y) {
        if (ctx->current_row != UINT32_MAX) {
            esp_err_t flush_ret = flush_png_row_locked(ctx);
            if (flush_ret != ESP_OK) {
                ctx->status = flush_ret;
                return;
            }
        }

        memset(ctx->line_buffer, 0, ctx->display_width * sizeof(uint16_t));
        ctx->current_row = y;
        ctx->row_dirty = false;
    }

    uint32_t x_start = (x * ctx->display_width) / ctx->image_width;
    uint32_t x_end = ((x + 1U) * ctx->display_width) / ctx->image_width;

    if (x_end > x_start) {
        uint16_t color = rgba_to_rgb565_over_black(rgba);
        for (uint32_t dst_x = x_start; dst_x < x_end; dst_x++) {
            ctx->line_buffer[dst_x] = color;
        }
        ctx->row_dirty = true;
    }
}

static void png_done_callback(pngle_t *pngle)
{
    png_display_ctx_t *ctx = (png_display_ctx_t *)pngle_get_user_data(pngle);

    if (ctx == NULL || ctx->status != ESP_OK) {
        return;
    }

    ctx->status = flush_png_row_locked(ctx);
    if (ctx->status == ESP_OK) {
        ctx->decode_done = true;
    }
}

static esp_err_t show_png_image_locked(const char *path)
{
    FILE *fp = NULL;
    pngle_t *png = NULL;
    uint8_t feed_buffer[PNG_READ_CHUNK_SIZE];
    png_display_ctx_t ctx = {
        .line_buffer = NULL,
        .frame_buffer = NULL,
        .image_width = 0,
        .image_height = 0,
        .display_width = 0,
        .display_height = 0,
        .current_row = UINT32_MAX,
        .x_offset = 0,
        .y_offset = 0,
        .row_dirty = false,
        .decode_done = false,
        .status = ESP_OK,
    };
    size_t carry = 0;
    esp_err_t ret = ESP_OK;

    if (path == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(ensure_png_line_buffer(), TAG, "alloc png line buffer failed");
    ctx.line_buffer = s_png_line_buffer;

    /* Decode into the shared full-frame compose buffer (PSRAM). Pre-fill it
     * black so letterbox borders are cleared in one memset instead of a
     * line-by-line screen wipe. The finished frame is blitted in 16-line
     * chunks below, same smooth path used for RGB565/fps rendering. */
    ESP_RETURN_ON_ERROR(ensure_fps_compose_buffer(), TAG, "alloc fps compose buffer failed");
    memset(s_fps_compose_buffer, 0, FRAME_BUFFER_SIZE);
    ctx.frame_buffer = s_fps_compose_buffer;

    fp = fopen(path, "rb");
    if (fp == NULL) {
        return ESP_FAIL;
    }

    png = pngle_new();
    if (png == NULL) {
        fclose(fp);
        return ESP_ERR_NO_MEM;
    }

    pngle_set_user_data(png, &ctx);
    pngle_set_init_callback(png, png_init_callback);
    pngle_set_draw_callback(png, png_draw_callback);
    pngle_set_done_callback(png, png_done_callback);

    while (true) {
        size_t read_len = fread(feed_buffer + carry, 1, sizeof(feed_buffer) - carry, fp);
        size_t total_len = carry + read_len;

        if (read_len == 0 && total_len == 0) {
            break;
        }

        int fed = pngle_feed(png, feed_buffer, total_len);
        if (fed < 0) {
            ESP_LOGE(TAG, "PNG decode failed for %s: %s", path, pngle_error(png));
            ret = ESP_FAIL;
            goto cleanup;
        }

        if ((size_t)fed > total_len) {
            ret = ESP_FAIL;
            goto cleanup;
        }

        if (ctx.status != ESP_OK) {
            ret = ctx.status;
            goto cleanup;
        }

        carry = total_len - (size_t)fed;
        if (carry > 0) {
            memmove(feed_buffer, feed_buffer + fed, carry);
        }

        if (read_len == 0 && fed == 0) {
            ret = ESP_FAIL;
            goto cleanup;
        }

        if (ctx.decode_done && carry == 0) {
            break;
        }
    }

    if (ferror(fp)) {
        ret = ESP_FAIL;
        goto cleanup;
    }

    if (ctx.status != ESP_OK) {
        ret = ctx.status;
        goto cleanup;
    }

    if (!ctx.decode_done) {
        ret = ESP_FAIL;
        goto cleanup;
    }

    /* Blit the fully decoded frame in 16-line chunks -- pipelined SPI, no
     * progressive top-to-bottom fill. */
    ret = show_image_locked((const uint8_t *)s_fps_compose_buffer);

cleanup:
    if (png != NULL) {
        pngle_destroy(png);
    }
    if (fp != NULL) {
        fclose(fp);
    }

    return ret;
}

static bool has_rgb565_suffix(const char *name)
{
    const char *ext = strrchr(name, '.');

    return (ext != NULL) && (strcasecmp(ext, ".rgb565") == 0);
}

static bool has_raw_frame_suffix(const char *name)
{
    const char *ext = strrchr(name, '.');

    if (ext == NULL) {
        return false;
    }

    return (strcasecmp(ext, ".raw") == 0)
        || (strcasecmp(ext, ".bin") == 0)
        || (strcasecmp(ext, ".rgb") == 0)
        || (strcasecmp(ext, ".565") == 0);
}

static bool has_png_suffix(const char *name)
{
    const char *ext = strrchr(name, '.');

    return (ext != NULL) && (strcasecmp(ext, ".png") == 0);
}

static bool is_rgb565_frame_candidate(const char *name, const struct stat *st)
{
    if (name == NULL || st == NULL || !S_ISREG(st->st_mode)) {
        return false;
    }

    if (st->st_size != FRAME_BUFFER_SIZE) {
        return false;
    }

    if (has_rgb565_suffix(name) || has_raw_frame_suffix(name)) {
        return true;
    }

    return strrchr(name, '.') == NULL;
}

static bool is_png_candidate(const char *name, const struct stat *st)
{
    return (name != NULL) && (st != NULL) && S_ISREG(st->st_mode) && has_png_suffix(name);
}

static bool image_list_supports_raw_frames_only(void)
{
    for (size_t i = 0; i < s_image_count; i++) {
        if (s_images[i].type != SD_IMAGE_TYPE_RGB565) {
            return false;
        }
    }

    return true;
}

static int path_compare(const void *lhs, const void *rhs)
{
    const sd_image_entry_t *left = (const sd_image_entry_t *)lhs;
    const sd_image_entry_t *right = (const sd_image_entry_t *)rhs;

    return strcasecmp(left->path, right->path);
}

static void log_sdcard_entries_recursive(const char *dir_path, int depth, size_t *entry_count)
{
    DIR *dir;
    struct dirent *entry;
    char indent[16];

    if (depth > SD_LIST_MAX_DEPTH || entry_count == NULL || *entry_count >= SD_LIST_MAX_ENTRIES) {
        return;
    }

    dir = opendir(dir_path);
    if (dir == NULL) {
        if (depth == 0) {
            ESP_LOGW(TAG, "Failed to open %s: errno=%d (%s)", dir_path, errno, strerror(errno));
        }
        return;
    }

    make_indent(depth, indent, sizeof(indent));
    while ((entry = readdir(dir)) != NULL) {
        char full_path[SD_IMAGE_PATH_MAX_LEN];
        struct stat st;

        if (*entry_count >= SD_LIST_MAX_ENTRIES) {
            break;
        }

        if ((strcmp(entry->d_name, ".") == 0) || (strcmp(entry->d_name, "..") == 0)) {
            continue;
        }

        if (snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, entry->d_name) >= (int)sizeof(full_path)) {
            ESP_LOGW(TAG, "Skip long path under %s", dir_path);
            continue;
        }

        if (stat(full_path, &st) != 0) {
            ESP_LOGW(TAG, "stat failed for %s: errno=%d (%s)", full_path, errno, strerror(errno));
            continue;
        }

        if (S_ISDIR(st.st_mode)) {
            ESP_LOGI(TAG, "SD %s[%s/]", indent, entry->d_name);
            (*entry_count)++;
            log_sdcard_entries_recursive(full_path, depth + 1, entry_count);
            continue;
        }

        if (S_ISREG(st.st_mode)) {
            ESP_LOGI(TAG, "SD %s%s (%ld bytes)", indent, entry->d_name, (long)st.st_size);
            (*entry_count)++;
        }
    }

    closedir(dir);
}

static void log_sdcard_file_list(void)
{
    size_t entry_count = 0;

    ESP_LOGI(TAG, "Listing SD card files under %s", SD_MOUNT_POINT);
    log_sdcard_entries_recursive(SD_MOUNT_POINT, 0, &entry_count);
    if (entry_count == 0) {
        ESP_LOGW(TAG, "No files found under %s", SD_MOUNT_POINT);
    } else if (entry_count >= SD_LIST_MAX_ENTRIES) {
        ESP_LOGW(TAG, "SD file list truncated at %u entries", SD_LIST_MAX_ENTRIES);
    }
}

static esp_err_t scan_image_dir_recursive(const char *dir_path, int depth)
{
    DIR *dir;
    struct dirent *entry;

    if (depth > SD_SCAN_MAX_DEPTH) {
        return ESP_OK;
    }

    dir = opendir(dir_path);
    if (dir == NULL) {
        if (depth == 0 && errno == ENOENT) {
            ESP_LOGW(TAG, "Image directory not found: %s", dir_path);
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGE(TAG, "Failed to open %s: errno=%d", dir_path, errno);
        return ESP_FAIL;
    }

    while ((entry = readdir(dir)) != NULL) {
        char full_path[SD_IMAGE_PATH_MAX_LEN];
        struct stat st;

        if ((strcmp(entry->d_name, ".") == 0) || (strcmp(entry->d_name, "..") == 0)) {
            continue;
        }

        if (snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, entry->d_name) >= (int)sizeof(full_path)) {
            ESP_LOGW(TAG, "Skip long path under %s", dir_path);
            continue;
        }

        if (stat(full_path, &st) != 0) {
            ESP_LOGW(TAG, "stat failed for %s", full_path);
            continue;
        }

        if (S_ISDIR(st.st_mode)) {
            if (s_image_count < SD_IMAGE_MAX_FILES) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(scan_image_dir_recursive(full_path, depth + 1));
            }
            continue;
        }

        if (!S_ISREG(st.st_mode)) {
            continue;
        }

        if (!is_rgb565_frame_candidate(entry->d_name, &st) && !is_png_candidate(entry->d_name, &st)) {
            if (has_rgb565_suffix(entry->d_name) || has_raw_frame_suffix(entry->d_name) || has_png_suffix(entry->d_name) || strrchr(entry->d_name, '.') == NULL) {
                ESP_LOGW(TAG, "Skip %s, size=%ld expect=%d", full_path, (long)st.st_size, FRAME_BUFFER_SIZE);
            }
            continue;
        }

        if (s_image_count >= SD_IMAGE_MAX_FILES) {
            ESP_LOGW(TAG, "Image list full, ignoring remaining files");
            break;
        }

        ESP_LOGI(TAG, "Accept image candidate: %s (%ld bytes)", full_path, (long)st.st_size);
        strlcpy(s_images[s_image_count].path, full_path, sizeof(s_images[s_image_count].path));
        s_images[s_image_count].size = (size_t)st.st_size;
        s_images[s_image_count].data = NULL;
        s_images[s_image_count].preloaded = false;
        s_images[s_image_count].type = is_png_candidate(entry->d_name, &st) ? SD_IMAGE_TYPE_PNG : SD_IMAGE_TYPE_RGB565;
        s_image_count++;
    }

    closedir(dir);
    return ESP_OK;
}

/* Forward declaration: ensure_image_list_ex() must release any previously
 * preloaded PSRAM buffers before rescanning into the shared s_images[] list. */
static void free_preloaded_images(void);

/* Scan displayable images into the shared s_images[] list.
 *
 * root == NULL  -> scan SD_IMAGE_DIR (with fallbacks to /sdcard recursive and
 *                  SD_DEFAULT_IMAGE), used by the picture-viewer commands.
 * root != NULL  -> scan exactly that directory with no fallback, used by the
 *                  FPS test which needs /sdcard/rgb565 to contain only raw
 *                  RGB565 frames.
 *
 * The scan result is cached per root directory: a request whose root differs
 * from s_scan_root_used releases the previous PSRAM preload and rescans, so
 * switching between the viewer and the FPS test always sees the right files. */
static esp_err_t ensure_image_list_ex(const char *root)
{
    esp_err_t ret;
    /* Resolve SD-rooted paths to the SPIFFS asset partition when the SD card
     * is unavailable, so image display / FPS tests work without an SD card.
     * The resolved buffers stay alive for the whole function. */
    char image_dir_buf[SD_IMAGE_PATH_MAX_LEN];
    char mount_buf[SD_IMAGE_PATH_MAX_LEN];
    char default_buf[SD_IMAGE_PATH_MAX_LEN];
    char root_buf[SD_IMAGE_PATH_MAX_LEN];
    const char *image_dir = lcd_resolve_asset_path(SD_IMAGE_DIR, image_dir_buf, sizeof(image_dir_buf));
    const char *mount     = lcd_resolve_asset_path(SD_MOUNT_POINT, mount_buf, sizeof(mount_buf));
    const char *default_img = lcd_resolve_asset_path(SD_DEFAULT_IMAGE, default_buf, sizeof(default_buf));
    const char *effective = (root != NULL) ? lcd_resolve_asset_path(root, root_buf, sizeof(root_buf)) : image_dir;
    const bool use_default_root = (root == NULL);

    if (image_dir == NULL || mount == NULL || default_img == NULL || effective == NULL) {
        ESP_LOGE(TAG, "Failed to resolve asset path");
        return ESP_ERR_INVALID_ARG;
    }

    if (s_sd_scan_done && s_image_count > 0 &&
        strcmp(s_scan_root_used, effective) == 0) {
        return ESP_OK;
    }

    /* Root changed (or first scan): drop any PSRAM-preloaded frames from the
     * previous list before overwriting s_images[], otherwise they'd leak. */
    free_preloaded_images();
    memset(s_images, 0, sizeof(s_images));
    s_image_count = 0;

    ret = scan_image_dir_recursive(effective, 0);
    if (use_default_root && s_image_count == 0) {
        if (ret == ESP_OK) {
            ESP_LOGW(TAG, "No displayable images under %s, fallback to recursive scan of %s", image_dir, mount);
        }
        ret = scan_image_dir_recursive(mount, 0);
    }

    if (use_default_root && s_image_count == 0) {
        struct stat st;

        if ((stat(default_img, &st) == 0) && S_ISREG(st.st_mode) && (st.st_size == FRAME_BUFFER_SIZE)) {
            strlcpy(s_images[0].path, default_img, sizeof(s_images[0].path));
            s_images[0].size = FRAME_BUFFER_SIZE;
            s_images[0].type = SD_IMAGE_TYPE_RGB565;
            s_image_count = 1;
            ret = ESP_OK;
        }
    }

    if (ret == ESP_OK && s_image_count > 0) {
        qsort(s_images, s_image_count, sizeof(s_images[0]), path_compare);
    }

    s_sd_scan_done = true;
    strlcpy(s_scan_root_used, effective, sizeof(s_scan_root_used));
    if (ret == ESP_OK && s_image_count > 0) {
        ESP_LOGI(TAG, "Found %u image(s) under %s", (unsigned)s_image_count, effective);
    }
    return (s_image_count > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static esp_err_t ensure_image_list(void)
{
    return ensure_image_list_ex(NULL);
}

static esp_err_t read_file_exact(const char *path, void *buffer, size_t size)
{
    FILE *fp = fopen(path, "rb");
    size_t read_bytes;

    if (fp == NULL) {
        ESP_LOGE(TAG, "Failed to open %s", path);
        return ESP_FAIL;
    }

    read_bytes = fread(buffer, 1, size, fp);
    fclose(fp);

    if (read_bytes != size) {
        ESP_LOGE(TAG, "Short read for %s: %u/%u", path, (unsigned)read_bytes, (unsigned)size);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void free_preloaded_images(void)
{
    for (size_t i = 0; i < s_image_count; i++) {
        if (s_images[i].data != NULL && s_images[i].preloaded) {
            free(s_images[i].data);
            s_images[i].data = NULL;
            s_images[i].preloaded = false;
        }
    }
    s_psram_preload_done = false;
}

static esp_err_t preload_images_to_psram_ex(const char *root)
{
    esp_err_t ret;

    /* Resolve the list first: switching roots frees the previous preload and
     * clears s_psram_preload_done, so the short-circuit below is per-directory. */
    ret = ensure_image_list_ex(root);
    if (ret != ESP_OK) {
        return ret;
    }

    if (s_psram_preload_done) {
        return ESP_OK;
    }

    for (size_t i = 0; i < s_image_count; i++) {
        if (s_images[i].type != SD_IMAGE_TYPE_RGB565) {
            continue;
        }

        if (s_images[i].preloaded && s_images[i].data != NULL) {
            continue;
        }

        uint8_t *buffer = (uint8_t *)heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (buffer == NULL) {
            ESP_LOGE(TAG, "PSRAM alloc failed for image %s", s_images[i].path);
            free_preloaded_images();
            return ESP_ERR_NO_MEM;
        }

        ret = read_file_exact(s_images[i].path, buffer, FRAME_BUFFER_SIZE);
        if (ret != ESP_OK) {
            free(buffer);
            free_preloaded_images();
            return ret;
        }

        s_images[i].data = buffer;
        s_images[i].size = FRAME_BUFFER_SIZE;
        s_images[i].preloaded = true;
    }

    s_psram_preload_done = image_list_supports_raw_frames_only();
    ESP_LOGI(TAG, "Preloaded raw RGB565 image(s) into PSRAM");
    return ESP_OK;
}

static esp_err_t preload_images_to_psram(void)
{
    return preload_images_to_psram_ex(NULL);
}

static esp_err_t ensure_sd_frame_buffer(void)
{
    if (s_sd_frame_buffer != NULL) {
        return ESP_OK;
    }

    s_sd_frame_buffer = (uint8_t *)heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_sd_frame_buffer == NULL) {
        s_sd_frame_buffer = (uint8_t *)heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_8BIT);
    }

    return (s_sd_frame_buffer != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t ensure_lvgl_ready(void)
{
    esp_err_t ret;

    if (s_lvgl_ready) {
        return ESP_OK;
    }

    ret = bsp_display_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    if (s_lvgl_mutex == NULL) {
        s_lvgl_mutex = xSemaphoreCreateMutex();
        if (s_lvgl_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_state_mutex == NULL) {
        s_state_mutex = xSemaphoreCreateMutex();
        if (s_state_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    ret = ensure_fps_overlay_buffer();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ensure_fps_compose_buffer();
    if (ret != ESP_OK) {
        return ret;
    }

    s_lvgl_ready = true;
    return ESP_OK;
}

static void update_fps_overlay_locked(uint32_t fps_value, const char *mode_name, const char *file_name)
{
    (void)fps_value;
    (void)mode_name;
    (void)file_name;
}

static esp_err_t show_image_locked(const uint8_t *buffer)
{
    const uint16_t *frame_data = (const uint16_t *)buffer;

    if (buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(ensure_fps_compose_buffer(), TAG, "alloc fps compose buffer failed");

    for (int y = 0; y < FRAME_HEIGHT; y += FPS_CHUNK_LINES) {
        int line_count = FPS_CHUNK_LINES;
        const uint16_t *chunk;

        if ((y + line_count) > FRAME_HEIGHT) {
            line_count = FRAME_HEIGHT - y;
        }

        chunk = frame_data + (y * FRAME_WIDTH);

        ESP_RETURN_ON_ERROR(
            bsp_display_draw_bitmap(0, y, FRAME_WIDTH, y + line_count, chunk),
            TAG,
            "draw fps frame chunk failed");
    }

    return ESP_OK;
}

static esp_err_t load_image_for_display(size_t index, const uint8_t **frame_data)
{
    esp_err_t ret;

    if (frame_data == NULL || index >= s_image_count) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_images[index].preloaded && s_images[index].data != NULL) {
        *frame_data = s_images[index].data;
        return ESP_OK;
    }

    ret = ensure_sd_frame_buffer();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = read_file_exact(s_images[index].path, s_sd_frame_buffer, FRAME_BUFFER_SIZE);
    if (ret != ESP_OK) {
        return ret;
    }

    *frame_data = s_sd_frame_buffer;
    return ESP_OK;
}

static esp_err_t show_selected_image(size_t index)
{
    const uint8_t *frame_data = NULL;
    esp_err_t ret;

    ret = ensure_image_list();
    if (ret != ESP_OK) {
        return ret;
    }

    if (s_image_count == 0 || index >= s_image_count) {
        return ESP_ERR_NOT_FOUND;
    }

    ret = ensure_lvgl_ready();
    if (ret != ESP_OK) {
        return ret;
    }

    set_run_state(FPS_MODE_NONE, 0);

    if (!lock_take(s_lvgl_mutex, pdMS_TO_TICKS(1000))) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_images[index].type == SD_IMAGE_TYPE_PNG) {
        ret = show_png_image_locked(s_images[index].path);
    } else {
        ret = load_image_for_display(index, &frame_data);
        if (ret == ESP_OK) {
            ret = show_image_locked(frame_data);
        }
    }

    lock_give(s_lvgl_mutex);
    if (ret != ESP_OK) {
        return ret;
    }

    lcd_bl_on();
    s_selected_image_index = index;
    ESP_LOGI(TAG, "Show image %u/%u: %s",
        (unsigned)(index + 1),
        (unsigned)s_image_count,
        s_images[index].path);
    return ESP_OK;
}

esp_err_t sd_lcd_show_first_image(void)
{
    esp_err_t ret = ensure_image_list();

    if (ret != ESP_OK) {
        return ret;
    }

    if (s_image_count == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    return show_selected_image(0);
}

esp_err_t sd_lcd_show_next_image(void)
{
    size_t index;
    esp_err_t ret = ensure_image_list();

    if (ret != ESP_OK) {
        return ret;
    }

    if (s_image_count == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    if (s_selected_image_index >= s_image_count) {
        s_selected_image_index = 0;
    }

    index = (s_selected_image_index + 1) % s_image_count;
    return show_selected_image(index);
}

esp_err_t sd_lcd_show_prev_image(void)
{
    size_t index;
    esp_err_t ret = ensure_image_list();

    if (ret != ESP_OK) {
        return ret;
    }

    if (s_image_count == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    if (s_selected_image_index >= s_image_count) {
        s_selected_image_index = 0;
    }

    index = (s_selected_image_index == 0) ? (s_image_count - 1) : (s_selected_image_index - 1);
    return show_selected_image(index);
}

static void set_run_state(fps_mode_t mode, uint32_t interval_ms)
{
    if (lock_take(s_state_mutex, portMAX_DELAY)) {
        s_active_mode = mode;
        s_switch_interval_ms = interval_ms;
        if (mode == FPS_MODE_NONE) {
            s_run_start_us = 0;
            s_frames_displayed = 0;
            s_last_fps_value = 0;
            s_frames_since_update = 0;
            s_last_fps_update_us = 0;
        }
        lock_give(s_state_mutex);
    }
}

static void fps_worker_task(void *arg)
{
    (void)arg;
    size_t index = 0;

    while (true) {
        fps_mode_t mode;
        uint32_t interval_ms;

        if (!lock_take(s_state_mutex, portMAX_DELAY)) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        mode = s_active_mode;
        interval_ms = s_switch_interval_ms;
        lock_give(s_state_mutex);

        if (mode == FPS_MODE_NONE) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (s_image_count == 0) {
            ESP_LOGW(TAG, "No raw RGB565 images available for FPS task");
            set_run_state(FPS_MODE_NONE, 0);
            continue;
        }

        if (s_images[index].type != SD_IMAGE_TYPE_RGB565) {
            ESP_LOGW(TAG, "FPS task skips non-RGB565 image: %s", s_images[index].path);
            set_run_state(FPS_MODE_NONE, 0);
            continue;
        }

        const uint8_t *frame_data = NULL;
        const char *frame_name = s_images[index].path;
        esp_err_t ret = ESP_OK;

        if (mode == FPS_MODE_SD) {
            ret = ensure_sd_frame_buffer();
            if (ret == ESP_OK) {
                ret = read_file_exact(s_images[index].path, s_sd_frame_buffer, FRAME_BUFFER_SIZE);
            }
            frame_data = s_sd_frame_buffer;
        } else {
            if (!s_images[index].preloaded || s_images[index].data == NULL) {
                ret = ESP_ERR_INVALID_STATE;
                ESP_LOGE(TAG, "Image not preloaded: %s", s_images[index].path);
            }
            frame_data = s_images[index].data;
        }

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "FPS frame load failed: %s", esp_err_to_name(ret));
            set_run_state(FPS_MODE_NONE, 0);
            continue;
        }

        if (lock_take(s_lvgl_mutex, portMAX_DELAY)) {
            int64_t now = esp_timer_get_time();
            const char *mode_name = (mode == FPS_MODE_SD) ? "sd" : "psram";
            uint32_t overlay_fps = s_last_fps_value;

            if (s_run_start_us == 0) {
                s_run_start_us = now;
                s_last_fps_update_us = now;
                s_frames_displayed = 0;
                s_frames_since_update = 0;
                update_fps_overlay_locked(0, mode_name, frame_name);
            }

            ret = prepare_fps_frame((const uint16_t *)frame_data, overlay_fps, mode_name, frame_name);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "FPS overlay prepare failed: %s", esp_err_to_name(ret));
                set_run_state(FPS_MODE_NONE, 0);
                lock_give(s_lvgl_mutex);
                continue;
            }

            ret = show_image_locked((const uint8_t *)s_fps_compose_buffer);
            if (ret == ESP_OK) {
                s_frames_displayed++;
                s_frames_since_update++;
                if ((now - s_last_fps_update_us) >= FPS_UPDATE_PERIOD_US) {
                    int64_t interval_elapsed = now - s_last_fps_update_us;
                    if (interval_elapsed > 0) {
                        s_last_fps_value = (uint32_t)((s_frames_since_update * 1000000ULL) / interval_elapsed);
                    }
                    update_fps_overlay_locked(s_last_fps_value, mode_name, frame_name);
                    s_last_fps_update_us = now;
                    s_frames_since_update = 0;
                }
            }
            lock_give(s_lvgl_mutex);
        }

        index = (index + 1) % s_image_count;
        if (interval_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
        } else {
            taskYIELD();
        }
    }
}

static esp_err_t ensure_fps_task(void)
{
    if (s_fps_task_running) {
        return ESP_OK;
    }

    if (xTaskCreate(fps_worker_task, "lcd_fps", 8192, NULL, 5, &s_fps_task_handle) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_fps_task_running = true;
    return ESP_OK;
}

static esp_err_t ensure_bench_buffer(size_t chunk_bytes)
{
    if (s_bench_buffer != NULL && s_bench_buffer_size >= chunk_bytes) {
        return ESP_OK;
    }

    if (s_bench_buffer != NULL) {
        free(s_bench_buffer);
        s_bench_buffer = NULL;
        s_bench_buffer_size = 0;
    }

    s_bench_buffer = (uint8_t *)heap_caps_malloc(chunk_bytes, MALLOC_CAP_8BIT);
    if (s_bench_buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_bench_buffer_size = chunk_bytes;

    for (size_t i = 0; i < chunk_bytes; i++) {
        s_bench_buffer[i] = (uint8_t)(i & 0xFFU);
    }
    return ESP_OK;
}

static esp_err_t ensure_sd_mount_ready(void)
{
    struct stat st = {0};

    if (stat(SD_MOUNT_POINT, &st) != 0) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!S_ISDIR(st.st_mode)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

static esp_err_t flush_bench_file(FILE *fp)
{
    if (fflush(fp) != 0) {
        return ESP_FAIL;
    }

    if (fsync(fileno(fp)) != 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t run_sd_write_bench(size_t total_bytes, size_t chunk_bytes, double *mbps)
{
    FILE *fp;
    size_t remaining = total_bytes;
    int64_t start_us;
    int64_t elapsed_us;

    ESP_RETURN_ON_ERROR(ensure_sd_mount_ready(), TAG, "sd card is not mounted");
    ESP_RETURN_ON_ERROR(ensure_bench_buffer(chunk_bytes), TAG, "alloc bench buffer failed");

    fp = fopen(SD_BENCH_FILE, "wb");
    if (fp == NULL) {
        return ESP_FAIL;
    }

    start_us = esp_timer_get_time();
    while (remaining > 0) {
        size_t to_write = (remaining > chunk_bytes) ? chunk_bytes : remaining;
        if (fwrite(s_bench_buffer, 1, to_write, fp) != to_write) {
            fclose(fp);
            return ESP_FAIL;
        }
        remaining -= to_write;
    }

    if (flush_bench_file(fp) != ESP_OK) {
        fclose(fp);
        return ESP_FAIL;
    }
    fclose(fp);
    elapsed_us = esp_timer_get_time() - start_us;
    *mbps = (elapsed_us > 0) ? ((double)total_bytes / (1024.0 * 1024.0)) / ((double)elapsed_us / 1000000.0) : 0.0;
    return ESP_OK;
}

static esp_err_t run_sd_read_bench(size_t total_bytes, size_t chunk_bytes, double *mbps)
{
    FILE *fp;
    size_t remaining = total_bytes;
    int64_t start_us;
    int64_t elapsed_us;

    ESP_RETURN_ON_ERROR(ensure_sd_mount_ready(), TAG, "sd card is not mounted");
    ESP_RETURN_ON_ERROR(ensure_bench_buffer(chunk_bytes), TAG, "alloc bench buffer failed");

    fp = fopen(SD_BENCH_FILE, "rb");
    if (fp == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    start_us = esp_timer_get_time();
    while (remaining > 0) {
        size_t to_read = (remaining > chunk_bytes) ? chunk_bytes : remaining;
        if (fread(s_bench_buffer, 1, to_read, fp) != to_read) {
            fclose(fp);
            return ESP_FAIL;
        }
        remaining -= to_read;
    }

    fclose(fp);
    elapsed_us = esp_timer_get_time() - start_us;
    *mbps = (elapsed_us > 0) ? ((double)total_bytes / (1024.0 * 1024.0)) / ((double)elapsed_us / 1000000.0) : 0.0;
    return ESP_OK;
}

static int sd_bench_cmd(int argc, char **argv)
{
    const char *mode;
    size_t chunk_bytes = SD_BENCH_CHUNK_BYTES;
    size_t total_bytes = SD_BENCH_TOTAL_BYTES;
    double write_mbps = 0.0;
    double read_mbps = 0.0;
    bool do_write = false;
    bool do_read = false;
    esp_err_t ret;
    int nerrors = arg_parse(argc, argv, (void **)&s_sd_bench_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, s_sd_bench_args.end, argv[0]);
        return 1;
    }

    mode = s_sd_bench_args.mode->sval[0];
    if (s_sd_bench_args.chunk_kb->count > 0) {
        chunk_bytes = (size_t)s_sd_bench_args.chunk_kb->ival[0] * 1024U;
    }
    if (s_sd_bench_args.total_kb->count > 0) {
        total_bytes = (size_t)s_sd_bench_args.total_kb->ival[0] * 1024U;
    }

    if (chunk_bytes < SD_BENCH_MIN_CHUNK_BYTES || total_bytes < chunk_bytes) {
        ESP_LOGE(TAG, "Invalid bench size: chunk=%u total=%u", (unsigned)chunk_bytes, (unsigned)total_bytes);
        return 1;
    }

    do_write = (strcasecmp(mode, "write") == 0) || (strcasecmp(mode, "rw") == 0);
    do_read = (strcasecmp(mode, "read") == 0) || (strcasecmp(mode, "rw") == 0);

    if (!do_write && !do_read) {
        ESP_LOGE(TAG, "Unsupported mode: %s", mode);
        return 1;
    }

    if (do_write) {
        ret = run_sd_write_bench(total_bytes, chunk_bytes, &write_mbps);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SD write bench failed: %s", esp_err_to_name(ret));
            return 1;
        }
        ESP_LOGI(TAG, "SD write %.2f MB/s, total=%u KB chunk=%u KB", write_mbps, (unsigned)(total_bytes / 1024U), (unsigned)(chunk_bytes / 1024U));
    }

    if (do_read) {
        ret = run_sd_read_bench(total_bytes, chunk_bytes, &read_mbps);
        if (ret != ESP_OK) {
            if (ret == ESP_ERR_NOT_FOUND) {
                ESP_LOGE(TAG, "SD read bench source file missing, run write or rw mode first");
            }
            ESP_LOGE(TAG, "SD read bench failed: %s", esp_err_to_name(ret));
            return 1;
        }
        ESP_LOGI(TAG, "SD read %.2f MB/s, total=%u KB chunk=%u KB", read_mbps, (unsigned)(total_bytes / 1024U), (unsigned)(chunk_bytes / 1024U));
    }

    return 0;
}

static int lcd_fps_cmd(int argc, char **argv)
{
    const char *action;
    const char *mode = "sd";
    uint32_t interval_ms = 0;
    esp_err_t ret;
    int nerrors = arg_parse(argc, argv, (void **)&s_lcd_fps_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, s_lcd_fps_args.end, argv[0]);
        return 1;
    }

    action = s_lcd_fps_args.action->sval[0];
    if (s_lcd_fps_args.mode->count > 0) {
        mode = s_lcd_fps_args.mode->sval[0];
    }
    if (s_lcd_fps_args.interval_ms->count > 0) {
        interval_ms = (uint32_t)s_lcd_fps_args.interval_ms->ival[0];
    }

    if (strcasecmp(action, "status") == 0) {
        if (lock_take(s_state_mutex, pdMS_TO_TICKS(100))) {
            ESP_LOGI(TAG, "fps task mode=%s interval=%lu ms frames=%lu current_fps=%lu images=%u preload=%s",
                fps_mode_name(s_active_mode),
                (unsigned long)s_switch_interval_ms,
                (unsigned long)s_frames_displayed,
                (unsigned long)s_last_fps_value,
                (unsigned)s_image_count,
                s_psram_preload_done ? "yes" : "no");
            lock_give(s_state_mutex);
        }
        return 0;
    }

    if (strcasecmp(action, "stop") == 0) {
        set_run_state(FPS_MODE_NONE, 0);
        if (lock_take(s_lvgl_mutex, pdMS_TO_TICKS(1000))) {
            update_fps_overlay_locked(0, "idle", NULL);
            lock_give(s_lvgl_mutex);
        }
        ESP_LOGI(TAG, "LCD FPS task stopped");
        return 0;
    }

    if (strcasecmp(action, "start") != 0) {
        ESP_LOGE(TAG, "Unsupported action: %s", action);
        return 1;
    }

    ret = ensure_image_list_ex(SD_RGB565_DIR);
    if (ret != ESP_OK) {
        char rgb565_dir_buf[SD_IMAGE_PATH_MAX_LEN];
        const char *rgb565_dir = lcd_resolve_asset_path(SD_RGB565_DIR, rgb565_dir_buf, sizeof(rgb565_dir_buf));
        ESP_LOGE(TAG, "No raw RGB565 frames found under %s (expect %dx%d RGB565, %d bytes each)",
                 rgb565_dir ? rgb565_dir : SD_RGB565_DIR, FRAME_WIDTH, FRAME_HEIGHT, FRAME_BUFFER_SIZE);
        return 1;
    }

    if (!image_list_supports_raw_frames_only()) {
        char rgb565_dir_buf[SD_IMAGE_PATH_MAX_LEN];
        const char *rgb565_dir = lcd_resolve_asset_path(SD_RGB565_DIR, rgb565_dir_buf, sizeof(rgb565_dir_buf));
        ESP_LOGE(TAG, "lcd_fps_test only supports raw RGB565 frames under %s", rgb565_dir ? rgb565_dir : SD_RGB565_DIR);
        return 1;
    }

    ret = ensure_lvgl_ready();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LVGL init failed: %s", esp_err_to_name(ret));
        return 1;
    }

    if (strcasecmp(mode, "psram") == 0) {
        ret = preload_images_to_psram_ex(SD_RGB565_DIR);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PSRAM preload failed: %s", esp_err_to_name(ret));
            return 1;
        }
        set_run_state(FPS_MODE_PSRAM, interval_ms);
    } else if (strcasecmp(mode, "sd") == 0) {
        ret = ensure_sd_frame_buffer();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Frame buffer alloc failed: %s", esp_err_to_name(ret));
            return 1;
        }
        set_run_state(FPS_MODE_SD, interval_ms);
    } else {
        ESP_LOGE(TAG, "Unsupported mode: %s", mode);
        return 1;
    }

    ret = ensure_fps_task();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start FPS task: %s", esp_err_to_name(ret));
        set_run_state(FPS_MODE_NONE, 0);
        return 1;
    }

    if (lock_take(s_lvgl_mutex, pdMS_TO_TICKS(1000))) {
        update_fps_overlay_locked(0, mode, s_images[0].path);
        lock_give(s_lvgl_mutex);
    }

    ESP_LOGI(TAG, "LCD FPS task started, mode=%s images=%u interval=%lu ms", mode, (unsigned)s_image_count, (unsigned long)interval_ms);
    return 0;
}

static void register_sd_bench_cmd(void)
{
    s_sd_bench_args.mode = arg_str1("m", "mode", "<write|read|rw>", "SD bench mode");
    s_sd_bench_args.chunk_kb = arg_int0("c", "chunk-kb", "<kb>", "Chunk size in KB, default 64");
    s_sd_bench_args.total_kb = arg_int0("t", "total-kb", "<kb>", "Total size in KB, default 8192");
    s_sd_bench_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "sd_rw_test",
        .help = "Benchmark SD card read/write throughput",
        .hint = NULL,
        .func = &sd_bench_cmd,
        .argtable = &s_sd_bench_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_lcd_fps_cmd(void)
{
    s_lcd_fps_args.action = arg_str1("a", "action", "<start|stop|status>", "Control LCD FPS task");
    s_lcd_fps_args.mode = arg_str0("m", "mode", "<sd|psram>", "Source mode for frame switching");
    s_lcd_fps_args.interval_ms = arg_int0("i", "interval-ms", "<ms>", "Delay between frame switches, default 0");
    s_lcd_fps_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "lcd_fps_test",
        .help = "Run LVGL LCD FPS test from SD or preloaded PSRAM images",
        .hint = NULL,
        .func = &lcd_fps_cmd,
        .argtable = &s_lcd_fps_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

esp_err_t sd_lcd_cmd_init(void)
{
    esp_err_t ret;

    if (s_preload_attempted) {
        return ESP_OK;
    }

    s_preload_attempted = true;
    log_sdcard_file_list();
    ret = ensure_image_list();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Skip PSRAM preload, no images available yet");
        return ESP_OK;
    }

    ret = preload_images_to_psram();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Power-on PSRAM preload failed: %s", esp_err_to_name(ret));
    }

    if (s_image_count > 0) {
        s_selected_image_index = 0;
        ret = sd_lcd_show_first_image();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Power-on first image display failed: %s", esp_err_to_name(ret));
        } else {
            lcd_bl_on();
            ESP_LOGI(TAG, "Power-on first image displayed, LCD backlight enabled");
        }
    }

    return ESP_OK;
}

void sd_lcd_cmd_register_all(void)
{
    register_sd_bench_cmd();
    register_lcd_fps_cmd();

    ESP_LOGI(TAG, "SD/LCD test commands registered:");
    ESP_LOGI(TAG, "  sd_rw_test -m rw");
    ESP_LOGI(TAG, "  sd_rw_test -m write -c 128 -t 4096");
    ESP_LOGI(TAG, "  lcd_fps_test -a start -m sd");
    ESP_LOGI(TAG, "  lcd_fps_test -a start -m psram -i 30");
    ESP_LOGI(TAG, "  lcd_fps_test -a stop");
    ESP_LOGI(TAG, "  lcd_fps_test -a status");
}