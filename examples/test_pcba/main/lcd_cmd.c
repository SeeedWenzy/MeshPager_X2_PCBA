#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "meshpager_x2.h"

static const char *TAG = "LCD_TEST";

#define LCD_FB_WIDTH                BSP_LCD_H_RES
#define LCD_FB_HEIGHT               BSP_LCD_V_RES
#define LCD_FB_PIXELS               (LCD_FB_WIDTH * LCD_FB_HEIGHT)
#define LCD_TEXT_SCALE              (3)
#define LCD_FONT_WIDTH              (5)
#define LCD_FONT_HEIGHT             (7)
#define LCD_LINE_BUFFER_LINES       (16)

static struct {
    struct arg_str *mode;
    struct arg_str *color;
    struct arg_int *brightness;
    struct arg_end *end;
} lcd_test_args;

static struct {
    struct arg_str *mode;
    struct arg_end *end;
} lcd_fb_demo_args;

static struct {
    struct arg_str *mode;
    struct arg_str *action;
    struct arg_end *end;
} lcd_ui_demo_args;

static struct {
    struct arg_str *mode;
    struct arg_end *end;
} lcd_line_demo_args;

static uint16_t *s_lcd_framebuffer;
static uint16_t *s_lcd_line_buffer;
static bool s_lcd_ready;
static TaskHandle_t s_lcd_dashboard_task;
static volatile bool s_lcd_dashboard_stop;

static esp_err_t lcd_require_ready(void)
{
    esp_err_t ret;

    if (s_lcd_ready) {
        return ESP_OK;
    }

    ret = bsp_display_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize display: %s", esp_err_to_name(ret));
        return ret;
    }

    s_lcd_ready = true;
    lcd_bl_on();
    return ESP_OK;
}

static esp_err_t lcd_require_framebuffer(void)
{
    if (s_lcd_framebuffer != NULL) {
        return ESP_OK;
    }

    s_lcd_framebuffer = (uint16_t *)calloc(LCD_FB_PIXELS, sizeof(uint16_t));
    if (s_lcd_framebuffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static esp_err_t lcd_require_line_buffer(void)
{
    if (s_lcd_line_buffer != NULL) {
        return ESP_OK;
    }

    s_lcd_line_buffer = (uint16_t *)calloc(LCD_FB_WIDTH * LCD_LINE_BUFFER_LINES, sizeof(uint16_t));
    if (s_lcd_line_buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static uint16_t lcd_color_from_name(const char *name)
{
    if (name == NULL) {
        return 0xFFFF;
    }

    if (strcasecmp(name, "black") == 0) return 0x0000;
    if (strcasecmp(name, "white") == 0) return 0xFFFF;
    if (strcasecmp(name, "red") == 0) return 0xF800;
    if (strcasecmp(name, "green") == 0) return 0x07E0;
    if (strcasecmp(name, "blue") == 0) return 0x001F;
    if (strcasecmp(name, "yellow") == 0) return 0xFFE0;
    if (strcasecmp(name, "cyan") == 0) return 0x07FF;
    if (strcasecmp(name, "magenta") == 0) return 0xF81F;
    if (strcasecmp(name, "gray") == 0 || strcasecmp(name, "grey") == 0) return 0x8410;
    if (strcasecmp(name, "orange") == 0) return 0xFD20;

    return 0xFFFF;
}

static const uint8_t *lcd_get_glyph(char ch)
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
    static const uint8_t glyph_H[7] = { 0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 };
    static const uint8_t glyph_I[7] = { 0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E };
    static const uint8_t glyph_J[7] = { 0x01, 0x01, 0x01, 0x01, 0x11, 0x11, 0x0E };
    static const uint8_t glyph_K[7] = { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 };
    static const uint8_t glyph_L[7] = { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F };
    static const uint8_t glyph_M[7] = { 0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11 };
    static const uint8_t glyph_N[7] = { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 };
    static const uint8_t glyph_O[7] = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E };
    static const uint8_t glyph_P[7] = { 0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10 };
    static const uint8_t glyph_Q[7] = { 0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D };
    static const uint8_t glyph_R[7] = { 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11 };
    static const uint8_t glyph_S[7] = { 0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E };
    static const uint8_t glyph_T[7] = { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 };
    static const uint8_t glyph_U[7] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E };
    static const uint8_t glyph_V[7] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04 };
    static const uint8_t glyph_W[7] = { 0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A };
    static const uint8_t glyph_X[7] = { 0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11 };
    static const uint8_t glyph_Y[7] = { 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04 };
    static const uint8_t glyph_Z[7] = { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F };

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
    case 'H': case 'h': return glyph_H;
    case 'I': case 'i': return glyph_I;
    case 'J': case 'j': return glyph_J;
    case 'K': case 'k': return glyph_K;
    case 'L': case 'l': return glyph_L;
    case 'M': case 'm': return glyph_M;
    case 'N': case 'n': return glyph_N;
    case 'O': case 'o': return glyph_O;
    case 'P': case 'p': return glyph_P;
    case 'Q': case 'q': return glyph_Q;
    case 'R': case 'r': return glyph_R;
    case 'S': case 's': return glyph_S;
    case 'T': case 't': return glyph_T;
    case 'U': case 'u': return glyph_U;
    case 'V': case 'v': return glyph_V;
    case 'W': case 'w': return glyph_W;
    case 'X': case 'x': return glyph_X;
    case 'Y': case 'y': return glyph_Y;
    case 'Z': case 'z': return glyph_Z;
    default: return glyph_space;
    }
}

static void lcd_fill_framebuffer(uint16_t color)
{
    for (size_t index = 0; index < LCD_FB_PIXELS; index++) {
        s_lcd_framebuffer[index] = color;
    }
}

static void lcd_draw_pixel_to_fb(int x, int y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= LCD_FB_WIDTH || y >= LCD_FB_HEIGHT) {
        return;
    }
    s_lcd_framebuffer[y * LCD_FB_WIDTH + x] = color;
}

static void lcd_fill_rect_to_fb(int x, int y, int width, int height, uint16_t color)
{
    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
            lcd_draw_pixel_to_fb(x + col, y + row, color);
        }
    }
}

static void lcd_draw_char_to_fb(int x, int y, char ch, uint16_t fg_color, uint16_t bg_color, int scale)
{
    const uint8_t *glyph = lcd_get_glyph(ch);

    for (int row = 0; row < LCD_FONT_HEIGHT; row++) {
        for (int col = 0; col < LCD_FONT_WIDTH; col++) {
            bool pixel_on = (glyph[row] & (1U << (LCD_FONT_WIDTH - 1 - col))) != 0U;
            uint16_t color = pixel_on ? fg_color : bg_color;

            for (int dy = 0; dy < scale; dy++) {
                for (int dx = 0; dx < scale; dx++) {
                    lcd_draw_pixel_to_fb(x + col * scale + dx, y + row * scale + dy, color);
                }
            }
        }
    }
}

static void lcd_draw_text_to_fb(int x, int y, const char *text, uint16_t fg_color, uint16_t bg_color, int scale)
{
    int cursor_x = x;

    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        if (*text == '\n') {
            cursor_x = x;
            y += (LCD_FONT_HEIGHT + 2) * scale;
            text++;
            continue;
        }
        lcd_draw_char_to_fb(cursor_x, y, *text, fg_color, bg_color, scale);
        cursor_x += (LCD_FONT_WIDTH + 1) * scale;
        text++;
    }
}

static void lcd_draw_panel_to_fb(int x, int y, int width, int height, uint16_t fill_color, uint16_t border_color)
{
    lcd_fill_rect_to_fb(x, y, width, height, fill_color);
    lcd_fill_rect_to_fb(x, y, width, 2, border_color);
    lcd_fill_rect_to_fb(x, y + height - 2, width, 2, border_color);
    lcd_fill_rect_to_fb(x, y, 2, height, border_color);
    lcd_fill_rect_to_fb(x + width - 2, y, 2, height, border_color);
}

static void lcd_draw_color_bars(void)
{
    static const uint16_t colors[] = {
        0xF800, 0x07E0, 0x001F, 0xFFE0, 0x07FF, 0xF81F, 0xFFFF, 0x0000,
    };
    const int bar_count = (int)(sizeof(colors) / sizeof(colors[0]));
    const int bar_width = LCD_FB_WIDTH / bar_count;

    for (int y = 0; y < LCD_FB_HEIGHT; y++) {
        for (int x = 0; x < LCD_FB_WIDTH; x++) {
            int bar = x / bar_width;
            if (bar >= bar_count) {
                bar = bar_count - 1;
            }
            s_lcd_framebuffer[y * LCD_FB_WIDTH + x] = colors[bar];
        }
    }
}

static void lcd_draw_checkerboard(void)
{
    const int cell = 20;

    for (int y = 0; y < LCD_FB_HEIGHT; y++) {
        for (int x = 0; x < LCD_FB_WIDTH; x++) {
            bool odd = (((x / cell) + (y / cell)) & 1) != 0;
            s_lcd_framebuffer[y * LCD_FB_WIDTH + x] = odd ? 0xFFFF : 0x0000;
        }
    }
}

static void lcd_draw_gradient(void)
{
    for (int y = 0; y < LCD_FB_HEIGHT; y++) {
        for (int x = 0; x < LCD_FB_WIDTH; x++) {
            uint16_t red = (uint16_t)((x * 31) / (LCD_FB_WIDTH - 1));
            uint16_t green = (uint16_t)((y * 63) / (LCD_FB_HEIGHT - 1));
            uint16_t blue = (uint16_t)(((x + y) * 31) / (LCD_FB_WIDTH + LCD_FB_HEIGHT - 2));
            s_lcd_framebuffer[y * LCD_FB_WIDTH + x] = (uint16_t)((red << 11) | (green << 5) | blue);
        }
    }
}

static void lcd_draw_framebuffer_border(void)
{
    const uint16_t border_color = 0xFFFF;

    for (int x = 0; x < LCD_FB_WIDTH; x++) {
        s_lcd_framebuffer[x] = border_color;
        s_lcd_framebuffer[(LCD_FB_HEIGHT - 1) * LCD_FB_WIDTH + x] = border_color;
    }

    for (int y = 0; y < LCD_FB_HEIGHT; y++) {
        s_lcd_framebuffer[y * LCD_FB_WIDTH] = border_color;
        s_lcd_framebuffer[y * LCD_FB_WIDTH + (LCD_FB_WIDTH - 1)] = border_color;
    }
}

/* --------------------------------------------------------------- battery -- */
/* Charger state derived from the board's two charger status pins:
 *   BSP_CHARGER_IN  high -> USB power is connected
 *     BSP_CHARGER_STAT low  -> still charging
 *     BSP_CHARGER_STAT high -> charge complete (full)
 *   BSP_CHARGER_IN  low  -> no USB power, not charging
 */
typedef enum {
    LCD_CHARGE_NONE,      /* No USB power connected */
    LCD_CHARGE_CHARGING,  /* USB present, battery still charging */
    LCD_CHARGE_FULL,      /* USB present, charge complete */
} lcd_charge_state_t;

/* Battery percentage mapping used by the test dashboard:
 *   >= 4.4 V  -> 100 %
 *   <= 3.3 V  ->   0 %
 *   3.3..4.4 V -> linear
 */
#define LCD_BAT_FULL_MV      (4400)
#define LCD_BAT_EMPTY_MV     (3300)

static int lcd_battery_percent_from_mv(uint16_t voltage_mv)
{
    if (voltage_mv >= LCD_BAT_FULL_MV) {
        return 100;
    }
    if (voltage_mv <= LCD_BAT_EMPTY_MV) {
        return 0;
    }
    return (int)(((uint32_t)(voltage_mv - LCD_BAT_EMPTY_MV) * 100) /
                 (LCD_BAT_FULL_MV - LCD_BAT_EMPTY_MV));
}

static lcd_charge_state_t lcd_read_charge_state(void)
{
    uint32_t in_mask = bsp_exp_input_io_get_level(BSP_CHARGER_IN);
    if ((in_mask & BSP_CHARGER_IN) == 0) {
        return LCD_CHARGE_NONE;
    }

    uint32_t stat_mask = bsp_exp_input_io_get_level(BSP_CHARGER_STAT);
    return (stat_mask & BSP_CHARGER_STAT) ? LCD_CHARGE_FULL : LCD_CHARGE_CHARGING;
}

/* Power the battery divider on, read the voltage, then power it back off.
 * bsp_battery_voltage_read() does its own multi-sample settling internally. */
static uint16_t lcd_read_battery_mv(void)
{
    bsp_bat_power_control(true);
    vTaskDelay(pdMS_TO_TICKS(10));
    uint16_t mv = bsp_battery_voltage_read();
    bsp_bat_power_control(false);
    return mv;
}

static void lcd_draw_ui_dashboard(void)
{
    /* Read live battery + charger state so the status panels reflect reality. */
    uint16_t bat_mv = lcd_read_battery_mv();
    int bat_pct = lcd_battery_percent_from_mv(bat_mv);
    lcd_charge_state_t charge = lcd_read_charge_state();

    const char *charge_text;
    uint16_t charge_color;
    switch (charge) {
    case LCD_CHARGE_CHARGING:
        charge_text = "CHARGING";
        charge_color = 0xFFE0;  /* yellow */
        break;
    case LCD_CHARGE_FULL:
        charge_text = "FULL";
        charge_color = 0x07E0;  /* green */
        break;
    default:
        charge_text = "NO USB";
        charge_color = 0xF800;  /* red */
        break;
    }

    char bat_pct_str[8];
    char bat_v_str[10];
    snprintf(bat_pct_str, sizeof(bat_pct_str), "%d%%", bat_pct);
    snprintf(bat_v_str, sizeof(bat_v_str), "%d.%02dV",
             bat_mv / 1000, (bat_mv % 1000) / 10);

    lcd_fill_framebuffer(0x0841);
    lcd_fill_rect_to_fb(0, 0, LCD_FB_WIDTH, 44, 0x001F);
    lcd_draw_text_to_fb(12, 8, "MESH PAGER", 0xFFFF, 0x001F, LCD_TEXT_SCALE);

    /* WIFI / GNSS */
    lcd_draw_panel_to_fb(12, 50, 104, 78, 0xFFFF, 0x07E0);
    lcd_draw_text_to_fb(24, 64, "WIFI", 0x001F, 0xFFFF, LCD_TEXT_SCALE);
    lcd_draw_text_to_fb(24, 100, "READY", 0x07E0, 0xFFFF, 2);

    lcd_draw_panel_to_fb(124, 50, 104, 78, 0xFFFF, 0xF800);
    lcd_draw_text_to_fb(136, 64, "GNSS", 0x001F, 0xFFFF, LCD_TEXT_SCALE);
    lcd_draw_text_to_fb(136, 100, "LOCK 3D", 0xF800, 0xFFFF, 2);

    /* AUDIO */
    lcd_draw_panel_to_fb(12, 134, 104, 84, 0xFFFF, 0xFFE0);
    lcd_draw_text_to_fb(24, 148, "AUDIO", 0x0000, 0xFFFF, LCD_TEXT_SCALE);
    lcd_draw_text_to_fb(24, 176, "SPK", 0x07E0, 0xFFFF, 2);
    lcd_draw_text_to_fb(24, 196, "MIC", 0x07E0, 0xFFFF, 2);

    /* BATTERY (percentage + voltage) */
    lcd_draw_panel_to_fb(124, 134, 104, 84, 0xFFFF, 0x07E0);
    lcd_draw_text_to_fb(136, 148, "BAT", 0x0000, 0xFFFF, LCD_TEXT_SCALE);
    lcd_draw_text_to_fb(136, 176, bat_pct_str, 0x001F, 0xFFFF, 2);
    lcd_draw_text_to_fb(136, 196, bat_v_str, 0x001F, 0xFFFF, 2);

    /* CHARGER status — full-width band, colored border reflects state */
    lcd_draw_panel_to_fb(12, 224, 216, 36, 0x0000, charge_color);
    int charge_text_w = (int)strlen(charge_text) * ((LCD_FONT_WIDTH + 1) * LCD_TEXT_SCALE);
    int charge_text_x = 12 + (216 - charge_text_w) / 2;
    lcd_draw_text_to_fb(charge_text_x, 232, charge_text, 0xFFFF, 0x0000, LCD_TEXT_SCALE);

    /* Soft keys */
    lcd_draw_panel_to_fb(12, 266, 66, 36, 0x07E0, 0xFFFF);
    lcd_draw_panel_to_fb(87, 266, 66, 36, 0xF800, 0xFFFF);
    lcd_draw_panel_to_fb(162, 266, 66, 36, 0x001F, 0xFFFF);
    lcd_draw_text_to_fb(24, 276, "YES", 0xFFFF, 0x07E0, 2);
    lcd_draw_text_to_fb(102, 276, "NO", 0xFFFF, 0xF800, 2);
    lcd_draw_text_to_fb(174, 276, "TEST", 0xFFFF, 0x001F, 2);
}

static esp_err_t lcd_push_framebuffer(void)
{
    for (int y = 0; y < LCD_FB_HEIGHT; y += LCD_LINE_BUFFER_LINES) {
        int line_count = LCD_LINE_BUFFER_LINES;
        const uint16_t *chunk;

        if (y + line_count > LCD_FB_HEIGHT) {
            line_count = LCD_FB_HEIGHT - y;
        }

        chunk = &s_lcd_framebuffer[y * LCD_FB_WIDTH];
        esp_err_t ret = bsp_display_draw_bitmap(0, y, LCD_FB_WIDTH, y + line_count, chunk);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

#define LCD_DASHBOARD_REFRESH_MS (5000)

static void lcd_dashboard_task(void *arg)
{
    (void)arg;

    while (!s_lcd_dashboard_stop) {
        lcd_draw_ui_dashboard();
        esp_err_t ret = lcd_push_framebuffer();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Dashboard framebuffer push failed: %s", esp_err_to_name(ret));
        }

        uint32_t notified = 0;
        if (xTaskNotifyWait(0, UINT32_MAX, &notified,
                            pdMS_TO_TICKS(LCD_DASHBOARD_REFRESH_MS)) == pdTRUE) {
            break;
        }
    }

    s_lcd_dashboard_task = NULL;
    vTaskDelete(NULL);
}

static esp_err_t lcd_start_dashboard_task(void)
{
    if (s_lcd_dashboard_task != NULL) {
        return ESP_OK;
    }

    s_lcd_dashboard_stop = false;
    if (xTaskCreate(lcd_dashboard_task, "lcd_dashboard", 4096, NULL, 5,
                    &s_lcd_dashboard_task) != pdPASS) {
        s_lcd_dashboard_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static void lcd_stop_dashboard_task(void)
{
    TaskHandle_t task = s_lcd_dashboard_task;
    if (task == NULL) {
        return;
    }

    s_lcd_dashboard_stop = true;
    xTaskNotifyGive(task);
    while (s_lcd_dashboard_task != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void lcd_fill_line_chunk_gradient(int y_start, int line_count)
{
    for (int row = 0; row < line_count; row++) {
        int y = y_start + row;
        for (int x = 0; x < LCD_FB_WIDTH; x++) {
            uint16_t red = (uint16_t)((x * 31) / (LCD_FB_WIDTH - 1));
            uint16_t green = (uint16_t)((y * 63) / (LCD_FB_HEIGHT - 1));
            uint16_t blue = (uint16_t)((y * 31) / (LCD_FB_HEIGHT - 1));
            s_lcd_line_buffer[row * LCD_FB_WIDTH + x] = (uint16_t)((red << 11) | (green << 5) | blue);
        }
    }
}

static void lcd_fill_line_chunk_bands(int y_start, int line_count)
{
    static const uint16_t band_colors[] = { 0xF800, 0xFD20, 0xFFE0, 0x07E0, 0x07FF, 0x001F, 0xF81F };
    const int band_count = (int)(sizeof(band_colors) / sizeof(band_colors[0]));
    const int band_height = LCD_FB_HEIGHT / band_count;

    for (int row = 0; row < line_count; row++) {
        int y = y_start + row;
        int band = y / band_height;
        uint16_t color;

        if (band >= band_count) {
            band = band_count - 1;
        }
        color = band_colors[band];
        for (int x = 0; x < LCD_FB_WIDTH; x++) {
            s_lcd_line_buffer[row * LCD_FB_WIDTH + x] = color;
        }
    }
}

static esp_err_t lcd_push_line_buffer_demo(const char *mode)
{
    esp_err_t ret;

    ret = lcd_require_ready();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = lcd_require_line_buffer();
    if (ret != ESP_OK) {
        return ret;
    }

    for (int y = 0; y < LCD_FB_HEIGHT; y += LCD_LINE_BUFFER_LINES) {
        int line_count = LCD_LINE_BUFFER_LINES;

        if (y + line_count > LCD_FB_HEIGHT) {
            line_count = LCD_FB_HEIGHT - y;
        }

        if (strcasecmp(mode, "gradient") == 0) {
            lcd_fill_line_chunk_gradient(y, line_count);
        } else {
            lcd_fill_line_chunk_bands(y, line_count);
        }

        ret = bsp_display_draw_bitmap(0, y, LCD_FB_WIDTH, y + line_count, s_lcd_line_buffer);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

static int lcd_test_cmd(int argc, char **argv)
{
    const char *mode;
    const char *color_name = NULL;
    esp_err_t ret;
    int nerrors = arg_parse(argc, argv, (void **)&lcd_test_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, lcd_test_args.end, argv[0]);
        return 1;
    }

    mode = lcd_test_args.mode->sval[0];
    if (lcd_test_args.color->count > 0) {
        color_name = lcd_test_args.color->sval[0];
    }

    ret = lcd_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    if (lcd_test_args.brightness->count > 0) {
        int brightness = lcd_test_args.brightness->ival[0];
        bsp_display_brightness_set((uint8_t)brightness);
        ESP_LOGI(TAG, "Backlight brightness set to %d%%", brightness);
    }

    ret = lcd_require_framebuffer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer: %s", esp_err_to_name(ret));
        return 1;
    }

    if (strcasecmp(mode, "solid") == 0) {
        lcd_fill_framebuffer(lcd_color_from_name(color_name));
    } else if (strcasecmp(mode, "bars") == 0) {
        lcd_draw_color_bars();
    } else if (strcasecmp(mode, "checker") == 0) {
        lcd_draw_checkerboard();
    } else {
        ESP_LOGE(TAG, "Unsupported mode: %s", mode);
        return 1;
    }

    ret = lcd_push_framebuffer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD draw failed: %s", esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG, "LCD test pattern displayed: %s", mode);
    return 0;
}

static int lcd_fb_demo_cmd(int argc, char **argv)
{
    const char *mode;
    esp_err_t ret;
    int nerrors = arg_parse(argc, argv, (void **)&lcd_fb_demo_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, lcd_fb_demo_args.end, argv[0]);
        return 1;
    }

    mode = lcd_fb_demo_args.mode->sval[0];

    ret = lcd_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = lcd_require_framebuffer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer: %s", esp_err_to_name(ret));
        return 1;
    }

    if (strcasecmp(mode, "gradient") == 0) {
        lcd_draw_gradient();
        lcd_draw_framebuffer_border();
    } else if (strcasecmp(mode, "ui") == 0) {
        lcd_draw_ui_dashboard();
    } else if (strcasecmp(mode, "clear") == 0) {
        lcd_fill_framebuffer(0x0000);
    } else {
        ESP_LOGE(TAG, "Unsupported framebuffer demo mode: %s", mode);
        return 1;
    }

    ret = lcd_push_framebuffer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Framebuffer push failed: %s", esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG, "LCD framebuffer demo displayed: %s", mode);
    return 0;
}

static int lcd_ui_demo_cmd(int argc, char **argv)
{
    const char *mode = NULL;
    const char *action = NULL;
    esp_err_t ret;
    int nerrors = arg_parse(argc, argv, (void **)&lcd_ui_demo_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, lcd_ui_demo_args.end, argv[0]);
        return 1;
    }

    if (lcd_ui_demo_args.mode->count > 0) {
        mode = lcd_ui_demo_args.mode->sval[0];
    }
    if (lcd_ui_demo_args.action->count > 0) {
        action = lcd_ui_demo_args.action->sval[0];
    }

    if (action != NULL) {
        if (strcasecmp(action, "stop") == 0) {
            lcd_stop_dashboard_task();
            ESP_LOGI(TAG, "LCD UI dashboard stopped");
            return 0;
        }

        if (strcasecmp(action, "status") == 0) {
            ESP_LOGI(TAG, "LCD UI dashboard is %s",
                     s_lcd_dashboard_task != NULL ? "running" : "stopped");
            return 0;
        }

        if (strcasecmp(action, "start") != 0) {
            ESP_LOGE(TAG, "Unsupported UI action: %s", action);
            return 1;
        }
    } else if (mode == NULL) {
        ESP_LOGE(TAG, "Specify --action start|stop|status or --mode dashboard|status");
        return 1;
    }

    if (action == NULL && mode != NULL &&
        strcasecmp(mode, "dashboard") != 0 && strcasecmp(mode, "status") != 0) {
        ESP_LOGE(TAG, "Unsupported UI mode: %s", mode);
        return 1;
    }

    if (action == NULL && s_lcd_dashboard_task != NULL) {
        ESP_LOGE(TAG, "LCD UI dashboard is running; use --action stop first");
        return 1;
    }

    ret = lcd_require_ready();
    if (ret != ESP_OK) {
        return 1;
    }

    ret = lcd_require_framebuffer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer: %s", esp_err_to_name(ret));
        return 1;
    }

    if (action == NULL) {
        lcd_draw_ui_dashboard();
        ret = lcd_push_framebuffer();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "UI framebuffer push failed: %s", esp_err_to_name(ret));
            return 1;
        }
        ESP_LOGI(TAG, "LCD UI demo displayed: %s", mode);
        return 0;
    }

    ret = lcd_start_dashboard_task();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start LCD UI dashboard: %s", esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG, "LCD UI dashboard started; refreshing every %d seconds",
             LCD_DASHBOARD_REFRESH_MS / 1000);
    return 0;
}

static int lcd_line_demo_cmd(int argc, char **argv)
{
    const char *mode;
    esp_err_t ret;
    int nerrors = arg_parse(argc, argv, (void **)&lcd_line_demo_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, lcd_line_demo_args.end, argv[0]);
        return 1;
    }

    mode = lcd_line_demo_args.mode->sval[0];
    if (strcasecmp(mode, "gradient") != 0 && strcasecmp(mode, "bands") != 0) {
        ESP_LOGE(TAG, "Unsupported line buffer mode: %s", mode);
        return 1;
    }

    ret = lcd_push_line_buffer_demo(mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Line buffer demo failed: %s", esp_err_to_name(ret));
        return 1;
    }

    ESP_LOGI(TAG, "LCD line buffer demo displayed: %s", mode);
    return 0;
}

static void register_lcd_test_cmd(void)
{
    lcd_test_args.mode = arg_str1("m", "mode", "<solid|bars|checker>", "LCD test pattern mode");
    lcd_test_args.color = arg_str0("c", "color", "<name>", "Solid color name, e.g. red/green/blue/white/black");
    lcd_test_args.brightness = arg_int0("b", "brightness", "<1-99>", "Backlight brightness percentage, clamped to 1-99 (default: unchanged)");
    lcd_test_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "lcd_test",
        .help = "Draw a basic LCD test pattern",
        .hint = NULL,
        .func = &lcd_test_cmd,
        .argtable = &lcd_test_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_lcd_fb_demo_cmd(void)
{
    lcd_fb_demo_args.mode = arg_str1("m", "mode", "<gradient|ui|clear>", "Simple framebuffer demo mode");
    lcd_fb_demo_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "lcd_fb_demo",
        .help = "Draw a simple framebuffer-based LCD demo",
        .hint = NULL,
        .func = &lcd_fb_demo_cmd,
        .argtable = &lcd_fb_demo_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_lcd_ui_demo_cmd(void)
{
    lcd_ui_demo_args.mode = arg_str0("m", "mode", "<dashboard|status>",
                                    "One-shot UI demo mode (legacy)");
    lcd_ui_demo_args.action = arg_str0("a", "action", "<start|stop|status>",
                                      "Control periodic dashboard refresh");
    lcd_ui_demo_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "lcd_ui_demo",
        .help = "Draw or control the LCD UI dashboard (refreshes every 5 seconds)",
        .hint = NULL,
        .func = &lcd_ui_demo_cmd,
        .argtable = &lcd_ui_demo_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_lcd_line_demo_cmd(void)
{
    lcd_line_demo_args.mode = arg_str1("m", "mode", "<gradient|bands>", "Line buffer refresh demo mode");
    lcd_line_demo_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "lcd_line_demo",
        .help = "Draw LCD content using a small line buffer and partial refresh",
        .hint = NULL,
        .func = &lcd_line_demo_cmd,
        .argtable = &lcd_line_demo_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void lcd_cmd_register_all(void)
{
    register_lcd_test_cmd();
    register_lcd_fb_demo_cmd();
    register_lcd_ui_demo_cmd();
    register_lcd_line_demo_cmd();

    ESP_LOGI(TAG, "LCD test commands registered:");
    ESP_LOGI(TAG, "  lcd_test -m solid -c red");
    ESP_LOGI(TAG, "  lcd_test -m bars");
    ESP_LOGI(TAG, "  lcd_test -m checker");
    ESP_LOGI(TAG, "  lcd_test -m solid -c white -b 50");
    ESP_LOGI(TAG, "  lcd_fb_demo -m gradient");
    ESP_LOGI(TAG, "  lcd_fb_demo -m ui");
    ESP_LOGI(TAG, "  lcd_fb_demo -m clear");
    ESP_LOGI(TAG, "  lcd_ui_demo -m dashboard");
    ESP_LOGI(TAG, "  lcd_line_demo -m gradient");
    ESP_LOGI(TAG, "  lcd_line_demo -m bands");
}