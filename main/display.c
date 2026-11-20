#include "display.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "zero_span.h"

const unsigned center_steps_mhz[CENTER_STEP_COUNT] = {1, 5, 10, 25, 50};
const unsigned edit_steps_khz[EDIT_STEP_COUNT] = {500, 1000, 5000, 10000, 25000, 50000, 100000};
const int gain_choices[GAIN_COUNT] = {-1, 15, 30, 45, 60};
const int scale_choices[SCALE_COUNT] = {0, -15, -30, -45};

static const char *const mode_names[MODE_COUNT] = {
    [MODE_SWEEP] = "SWEEP", [MODE_RT] = "RT", [MODE_ZERO_SPAN] = "ZERO SPAN",
};

static const settings_field_t rt_fields[] = {
    FIELD_CENTER_STEP, FIELD_SPAN, FIELD_GAIN, FIELD_DETECTOR, FIELD_SCALE, FIELD_PAUSE, FIELD_MAX_HOLD,
};
static const settings_field_t zero_span_fields[] = {
    FIELD_RBW, FIELD_TIME, FIELD_TRIGGER_MODE, FIELD_TRIGGER_LEVEL, FIELD_TRIGGER_EDGE,
    FIELD_CENTER_STEP, FIELD_GAIN, FIELD_DETECTOR, FIELD_SCALE, FIELD_PAUSE, FIELD_MAX_HOLD,
    FIELD_TRIGGER_ARM,
};
static const settings_field_t sweep_fields[] = {
    FIELD_START, FIELD_STOP, FIELD_EDIT_STEP, FIELD_GAIN, FIELD_DETECTOR, FIELD_SCALE,
};

settings_field_t settings_field(mode_tuner_t mode, unsigned row) {
    return (mode == MODE_ZERO_SPAN ? zero_span_fields : mode == MODE_RT ? rt_fields : sweep_fields)[row];
}

unsigned settings_rows(const ui_t *u) {
    return u->mode == MODE_ZERO_SPAN ? sizeof(zero_span_fields) / sizeof(*zero_span_fields) - (u->trigger_mode != ZERO_SINGLE)
         : u->mode == MODE_RT ? sizeof(rt_fields) / sizeof(*rt_fields)
                                  : sizeof(sweep_fields) / sizeof(*sweep_fields);
}

uint32_t settings_value(const ui_t *u, settings_field_t field) {
    const mode_settings_t *p = &u->settings[u->mode];
    switch (field) {
    case FIELD_CENTER_STEP:
    case FIELD_EDIT_STEP: return p->step_i;
    case FIELD_START: return u->sweep_band.low_hz;
    case FIELD_STOP: return u->sweep_band.high_hz;
    case FIELD_SPAN: return u->profile;
    case FIELD_RBW: return u->rbw_i;
    case FIELD_TIME: return u->time_i;
    case FIELD_TRIGGER_MODE: return u->trigger_mode;
    case FIELD_TRIGGER_LEVEL: return u->trigger_level;
    case FIELD_TRIGGER_EDGE: return u->trigger_falling;
    case FIELD_TRIGGER_ARM: return u->trigger_state == ZERO_HELD;
    case FIELD_GAIN: return p->gain_i;
    case FIELD_DETECTOR: return p->peak_detector;
    case FIELD_SCALE: return p->scale_i;
    case FIELD_PAUSE: return p->paused;
    case FIELD_MAX_HOLD: return p->max_hold;
    }
    return 0;
}

#define RGB(r,g,b) ((uint16_t)((((r) & 0xf8) << 8) | (((g) & 0xfc) << 3) | ((b) >> 3)))
#define BG       RGB(9,15,23)
#define PANEL    RGB(14,25,37)
#define TEXT     RGB(230,242,249)
#define MUTED    RGB(135,164,186)
#define GRID     RGB(37,55,70)
#define TEAL     RGB(84,241,202)
#define FILL     RGB(19,57,56)
#define GOLD     RGB(249,190,94)
#define ERROR    RGB(255,113,105)

/* A compact, upper-case 5x7 LCD font. Each row is stored in the low five bits. */
static const uint8_t digits[10][7] = {
    {14,17,19,21,25,17,14},{4,12,4,4,4,4,14},{14,17,1,2,4,8,31},
    {30,1,1,14,1,1,30},{2,6,10,18,31,2,2},{31,16,16,30,1,1,30},
    {14,16,16,30,17,17,14},{31,1,2,4,8,8,8},{14,17,17,14,17,17,14},{14,17,17,15,1,1,14},
};
static const uint8_t letters[26][7] = {
    {14,17,17,31,17,17,17},{30,17,17,30,17,17,30},{14,17,16,16,16,17,14},
    {30,17,17,17,17,17,30},{31,16,16,30,16,16,31},{31,16,16,30,16,16,16},
    {14,17,16,23,17,17,15},{17,17,17,31,17,17,17},{14,4,4,4,4,4,14},
    {7,2,2,2,2,18,12},{17,18,20,24,20,18,17},{16,16,16,16,16,16,31},
    {17,27,21,21,17,17,17},{17,25,21,19,17,17,17},{14,17,17,17,17,17,14},
    {30,17,17,30,16,16,16},{14,17,17,17,21,18,13},{30,17,17,30,20,18,17},
    {15,16,16,14,1,1,30},{31,4,4,4,4,4,4},{17,17,17,17,17,17,14},
    {17,17,17,17,17,10,4},{17,17,17,21,21,21,10},{17,17,10,4,10,17,17},
    {17,17,10,4,4,4,4},{31,1,2,4,8,16,31},
};
static const uint8_t *glyph(char c) {
    static const uint8_t blank[7] = {0}, dot[7] = {0,0,0,0,0,6,6};
    static const uint8_t dash[7] = {0,0,0,31,0,0,0}, plus[7] = {0,4,4,31,4,4,0};
    static const uint8_t arrow[7] = {0,8,4,2,4,8,0};
    if (c >= 'A' && c <= 'Z') return letters[c - 'A'];
    if (c >= '0' && c <= '9') return digits[c - '0'];
    if (c == '.') return dot;
    if (c == '-') return dash;
    if (c == '+') return plus;
    static const uint8_t slash[7] = {1,1,2,4,8,16,16};
    if (c == '>') return arrow;
    if (c == '/') return slash;
    return blank;
}

static DMA_ATTR uint16_t fb[LCD_W * LCD_STRIPE];
static int stripe_y;

static inline void px(int x, int y, uint16_t color) {
    if ((unsigned)x < LCD_W && (unsigned)(y - stripe_y) < LCD_STRIPE)
        fb[(y - stripe_y) * LCD_W + x] = color;
}

static void rect(int x, int y, int w, int h, uint16_t color) {
    int bottom = y + h, right = x + w;
    if (y < stripe_y) y = stripe_y;
    if (bottom > stripe_y + LCD_STRIPE) bottom = stripe_y + LCD_STRIPE;
    if (x < 0) x = 0;
    if (right > LCD_W) right = LCD_W;
    for (; y < bottom; ++y) {
        uint16_t *row = fb + (y - stripe_y) * LCD_W;
        for (int xx = x; xx < right; ++xx) row[xx] = color;
    }
}

static void text(int x, int y, const char *s, int scale, uint16_t color) {
    if (y >= stripe_y + LCD_STRIPE || y + 7 * scale <= stripe_y) return;
    for (; *s; ++s, x += 6 * scale) {
        const uint8_t *bits = glyph(*s);
        for (int r = 0; r < 7; ++r)
            for (int c = 0; c < 5; ++c)
                if (bits[r] & (16 >> c)) rect(x + c * scale, y + r * scale, scale, scale, color);
    }
}

static void right_text(int x_end, int y, const char *s, uint16_t c) {
    text(x_end + 1 - (int)strlen(s) * 6, y, s, 1, c);
}

void display_reset(ui_t *u) {
    for (int x = 0; x < TRACE_W; ++x) u->live[x] = u->mx[x] = UI_NO_LEVEL;
    memset(u->water, 0, sizeof(u->water));
    u->water_head = 0;
    u->peak_hz = 0;
}

static const int LEVEL_SPAN = 140; // half-dB units
void display_trace(ui_t *u, const trace_t *t) {
    if (t->peak < 0) return;
    u->peak_hz = t->peak > 0 ? t->peak_hz : 0;
    const mode_settings_t *p = &u->settings[u->mode];
    const int top = 2 * scale_choices[p->scale_i];
    u->water_head = (u->water_head + WATER_H - 1) % WATER_H;
    for (int x = 0; x < TRACE_W; ++x) {
        int level = t->cell[x] < 0 ? UI_NO_LEVEL : level_half_db(t->cell[x]);
        u->live[x] = level;
        if (p->max_hold && level > u->mx[x]) u->mx[x] = level;
        int col = (level - top + LEVEL_SPAN) * 62 / LEVEL_SPAN;
        u->water[u->water_head][x] = level == UI_NO_LEVEL ? 0 : col < 0 ? 1 : col > 62 ? 63 : col + 1;
    }
}

void display_time_trace(ui_t *u, const float power[TRACE_W]) {
    const bool max_hold = u->settings[u->mode].max_hold;
    for (unsigned x = 0; x < TRACE_W; ++x) {
        const int level = zero_span_level_half_db(power[x]);
        u->live[x] = level;
        if (max_hold && level > u->mx[x]) u->mx[x] = level;
    }
}

static uint16_t water_color(int k) {
    if (!k) return PANEL;
    if (k < 21) return RGB(10 + k / 3, 25 + k * 2, 42 + k * 4);
    if (k < 43) { k -= 21; return RGB(17 + k, 67 + k * 7, 126 + k * 2); }
    k -= 43;
    return RGB(40 + k * 10, 221 + k, 168 - k * 4);
}

enum { GX = 29, GY = 44, BOTTOM = 90, TIME_BOTTOM = 116, WX = GX, WY = 106 };
static int level_y(int level, int top, int bottom) {
    int d = top - level;
    if (d < 0) d = 0;
    if (d > LEVEL_SPAN) d = LEVEL_SPAN;
    return GY + d * (bottom - GY) / LEVEL_SPAN;
}

static void curve(const int16_t *levels, int top, int bottom, uint16_t color) {
    int prev = -1;
    for (int x = 0; x < TRACE_W; ++x) {
        if (levels[x] == UI_NO_LEVEL) { prev = -1; continue; }
        int y = level_y(levels[x], top, bottom);
        if (prev < 0) px(GX + x, y, color);
        else rect(GX + x, prev < y ? prev : y, 1, abs(prev - y) + 1, color);
        prev = y;
    }
}

static void freq_text(char *buf, size_t n, uint32_t hz, bool always_dec) {
    unsigned tenth = (hz + 50000u) / 100000u;
    if (always_dec || tenth % 10u) snprintf(buf, n, "%u.%u", tenth / 10u, tenth % 10u);
    else snprintf(buf, n, "%u", tenth / 10u);
}

static void rbw_text(char *buf, size_t n, unsigned hz) {
    if (hz >= 1000000u) snprintf(buf, n, "%u MHZ", hz / 1000000u);
    else snprintf(buf, n, "%u KHZ", hz / 1000u);
}

static void time_text(char *buf, size_t n, int us, bool ms, bool units) {
    const char *suffix = units ? (ms ? " MS" : " US") : "";
    if (!ms || us % 1000 == 0) {
        snprintf(buf, n, "%d%s", ms ? us / 1000 : us, suffix);
        return;
    }
    const unsigned mag = abs(us);
    unsigned fraction = mag % 1000u;
    int digits = 3;
    while (fraction % 10u == 0) { fraction /= 10u; --digits; }
    snprintf(buf, n, "%s%u.%0*u%s", us < 0 ? "-" : "", mag / 1000u, digits, fraction, suffix);
}

static int trigger_column(void) {
    return GX + TRACE_W / ZERO_PRETRIGGER_DIV;
}

static void trace_plot(const ui_t *u, int bottom) {
    const mode_settings_t *p = &u->settings[u->mode];
    const int top = 2 * scale_choices[p->scale_i], height = bottom - GY;
    if (stripe_y <= bottom && stripe_y + LCD_STRIPE > GY) {
        rect(GX, GY, TRACE_W, height + 1, PANEL);
        for (int x = 0; x < TRACE_W; ++x) {
            if (u->live[x] == UI_NO_LEVEL) continue;
            int y = level_y(u->live[x], top, bottom);
            rect(GX + x, y, 1, bottom - y + 1, FILL);
        }
        for (int i = 0; i <= 4; ++i) rect(GX + i * (TRACE_W - 1) / 4, GY, 1, height + 1, GRID);
        for (int i = 0; i <= 2; ++i) rect(GX, GY + i * height / 2, TRACE_W, 1, GRID);
        if (u->mode == MODE_ZERO_SPAN && u->trace_triggered)
            for (int y = GY; y < bottom; y += 4) rect(trigger_column(), y, 1, 2, GOLD);
        if (p->max_hold) curve(u->mx, top, bottom, GOLD);
        curve(u->live, top, bottom, TEAL);
    }
    for (int i = 0; i <= 2; ++i) {
        char s[12];
        snprintf(s, sizeof(s), "%d", (top - i * LEVEL_SPAN / 2) / 2);
        right_text(26, GY + i * height / 2 - (i == 2 ? 6 : 0), s, MUTED);
    }
}

static void spectrum_page(const ui_t *u) {
    char s[32], f[16];
    const char *mode = mode_names[u->mode];
    const mode_settings_t *p = &u->settings[u->mode];
    rect(8, 5, 7 + 6 * (int)strlen(mode), 11, FILL);
    text(12, 7, mode, 1, TEAL);
    text(94, 7, "HOLD B MODE", 1, MUTED);
    const bool held = p->paused || (u->mode == MODE_ZERO_SPAN && u->trigger_state == ZERO_HELD);
    const char *status = held ? "HOLD" : "RUN";
    if (!held && u->mode == MODE_ZERO_SPAN && u->trigger_mode != ZERO_FREE)
        status = u->trigger_state == ZERO_TRIGGERED ? "TRIG" : u->trigger_mode == ZERO_AUTO ? "AUTO" : "ARM";
    right_text(231, 7, u->rf_error ? "RF ERROR" : status, u->rf_error ? ERROR : held ? GOLD : MUTED);
    if (p->max_hold && !u->rf_error) text(170, 7, "MAX", 1, GOLD);
    if (u->mode != MODE_SWEEP) {
        snprintf(s, sizeof(s), "%u MHZ", p->center_mhz);
        text(8, 22, s, 2, TEXT);
        text(116, 26, "A+ B-", 1, TEAL);
    } else {
        char start[8], stop[8];
        freq_text(start, sizeof(start), u->band.low_hz, false);
        freq_text(stop, sizeof(stop), u->band.high_hz, false);
        snprintf(s, sizeof(s), "%s-%s MHZ", start, stop);
        text(8, 26, s, 1, TEXT);
    }
    if (u->mode == MODE_ZERO_SPAN) {
        rbw_text(f, sizeof(f), zero_rbws_hz[u->rbw_i]);
        snprintf(s, sizeof(s), "RBW %s", f);
        right_text(232, 26, s, TEAL);
    } else if (u->peak_hz) {
        freq_text(f, sizeof(f), u->peak_hz, true);
        snprintf(s, sizeof(s), "P %s", f);
        right_text(232, 26, s, TEAL);
    }
    rect(8, 39, 224, 1, GRID);
    if (u->mode == MODE_ZERO_SPAN) {
        trace_plot(u, TIME_BOTTOM);
        const int us = zero_times_us[u->time_i];
        const bool ms = us >= 1000;
        const int offset = u->trace_triggered ? us / ZERO_PRETRIGGER_DIV : 0;
        time_text(s, sizeof(s), -offset, ms, false);
        text(GX, 123, s, 1, MUTED);
        if (u->trace_triggered) {
            text(trigger_column() - 2, 123, "0", 1, GOLD);
        } else {
            time_text(s, sizeof(s), us / 2, ms, false);
            text(GX + TRACE_W / 2 - (int)strlen(s) * 3, 123, s, 1, MUTED);
        }
        time_text(s, sizeof(s), us - offset, ms, true);
        right_text(GX + TRACE_W, 123, s, MUTED);
        return;
    }
    trace_plot(u, BOTTOM);
    freq_text(f, sizeof(f), u->band.low_hz, false);
    text(GX, 95, f, 1, MUTED);
    freq_text(f, sizeof(f), u->band.low_hz + (u->band.high_hz - u->band.low_hz) / 2u, false);
    text(GX + TRACE_W / 2 - (int)strlen(f) * 3, 95, f, 1, MUTED);
    freq_text(f, sizeof(f), u->band.high_hz, false);
    right_text(GX + TRACE_W, 95, f, MUTED);
    for (int y = WY; y < WY + WATER_H; ++y) {
        if ((unsigned)(y - stripe_y) >= LCD_STRIPE) continue;
        const uint8_t *row = u->water[(u->water_head + y - WY) % WATER_H];
        for (int x = 0; x < TRACE_W; ++x) px(WX + x, y, water_color(row[x]));
    }
    if (u->sweep_n > 1) {
        rect(WX, 129, TRACE_W, 1, GRID);
        rect(WX, 129, TRACE_W * u->sweep_pos / u->sweep_n, 1, TEAL);
    }
}

static void field_value(settings_field_t field, uint32_t v, char *value, size_t n) {
    char f[12];
    switch (field) {
    case FIELD_CENTER_STEP: snprintf(value, n, "%u MHZ", center_steps_mhz[v]); break;
    case FIELD_SPAN: snprintf(value, n, "%u MHZ", live_profiles[v].span_mhz); break;
    case FIELD_RBW: rbw_text(value, n, zero_rbws_hz[v]); break;
    case FIELD_TIME: time_text(value, n, zero_times_us[v], zero_times_us[v] >= 1000, true); break;
    case FIELD_TRIGGER_MODE: {
        static const char *const names[] = {"FREE", "AUTO", "NORMAL", "SINGLE"};
        snprintf(value, n, "%s", names[v]);
        break;
    }
    case FIELD_TRIGGER_LEVEL: snprintf(value, n, "%d DBFS", zero_trigger_level_dbfs(v)); break;
    case FIELD_TRIGGER_EDGE: snprintf(value, n, "%s", v ? "FALL" : "RISE"); break;
    case FIELD_TRIGGER_ARM: snprintf(value, n, "%s", v ? "HOLD" : "ARM"); break;
    case FIELD_START:
    case FIELD_STOP:
    case FIELD_EDIT_STEP:
        freq_text(f, sizeof(f), field == FIELD_EDIT_STEP ? edit_steps_khz[v] * 1000u : v, false);
        snprintf(value, n, "%s MHZ", f);
        break;
    case FIELD_GAIN:
        if (gain_choices[v] < 0) snprintf(value, n, "AUTO");
        else snprintf(value, n, "%d", gain_choices[v]);
        break;
    case FIELD_DETECTOR: snprintf(value, n, "%s", v ? "PEAK" : "AVG"); break;
    case FIELD_SCALE: snprintf(value, n, "%d DBFS", scale_choices[v]); break;
    case FIELD_PAUSE:
    case FIELD_MAX_HOLD: snprintf(value, n, "%s", v ? "ON" : "OFF"); break;
    }
}

static void settings_page(const ui_t *u) {
    static const char *const labels[] = {
        [FIELD_CENTER_STEP] = "CENTER STEP", [FIELD_SPAN] = "SPAN",
        [FIELD_START] = "START", [FIELD_STOP] = "STOP", [FIELD_EDIT_STEP] = "EDIT STEP",
        [FIELD_GAIN] = "GAIN", [FIELD_DETECTOR] = "DETECTOR", [FIELD_SCALE] = "REF LEVEL",
        [FIELD_PAUSE] = "PAUSE", [FIELD_MAX_HOLD] = "MAX HOLD",
        [FIELD_RBW] = "RBW", [FIELD_TIME] = "TIME SPAN",
        [FIELD_TRIGGER_MODE] = "TRIGGER", [FIELD_TRIGGER_LEVEL] = "TRIG LEVEL",
        [FIELD_TRIGGER_EDGE] = "TRIG EDGE", [FIELD_TRIGGER_ARM] = "REARM",
    };
    char title[24];
    snprintf(title, sizeof(title), "%s SETTINGS", mode_names[u->mode]);
    text(9, 7, title, 1, TEAL);
    rect(8, 19, 224, 1, GRID);
    const unsigned rows = settings_rows(u);
    const unsigned visible = rows > 8u ? 8u : rows;
    unsigned first = 0;
    if (rows > visible) {
        first = u->setting_row > visible / 2 ? u->setting_row - visible / 2 : 0;
        if (first > rows - visible) first = rows - visible;
        char position[16];
        snprintf(position, sizeof(position), "%u/%u", u->setting_row + 1u, rows);
        right_text(231, 7, position, MUTED);
    }
    const int spacing = rows > 7 ? 12 : rows > 6 ? 13 : 15;
    for (unsigned i = first; i < first + visible; ++i) {
        const settings_field_t field = settings_field(u->mode, i);
        char value[20];
        int y = 23 + spacing * (i - first);
        const bool focused = i == u->setting_row;
        const bool selected = focused && u->editing;
        field_value(field, selected ? u->edit_value : settings_value(u, field), value, sizeof(value));
        if (selected) rect(8, y, 224, spacing - 1, FILL);
        if (focused) text(13, y + 2, ">", 1, selected ? TEAL : TEXT);
        text(24, y + 2, labels[field], 1, focused ? TEXT : MUTED);
        right_text(225, y + 2, value, selected ? TEAL : TEXT);
    }
    text(9, 123, u->editing ? "A+ B-" : "A DOWN B UP", 1, MUTED);
    const bool arm = settings_field(u->mode, u->setting_row) == FIELD_TRIGGER_ARM;
    right_text(231, 123, u->editing ? "HOLD A OK  B BACK" : arm ? "HOLD A ARM  B EXIT" : "HOLD A EDIT  B EXIT", MUTED);
}

esp_err_t display_draw(const ui_t *u) {
    for (stripe_y = 0; stripe_y < LCD_H; stripe_y += LCD_STRIPE) {
        rect(0, stripe_y, LCD_W, LCD_STRIPE, BG);
        if (u->menu) settings_page(u);
        else spectrum_page(u);
        ESP_RETURN_ON_ERROR(board_push_stripe(stripe_y, fb), "LCD", "update");
    }
    return ESP_OK;
}
