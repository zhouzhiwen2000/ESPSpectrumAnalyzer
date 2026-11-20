#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "spectrum_model.h"
#include "zero_span.h"

typedef struct {
    unsigned center_mhz, step_i, gain_i, scale_i;
    bool peak_detector, paused, max_hold;
} mode_settings_t;

typedef struct {
    mode_tuner_t mode;
    mode_settings_t settings[MODE_COUNT];
    band_t sweep_band;
    unsigned profile, rbw_i, time_i;
    zero_trigger_mode_t trigger_mode;
    zero_trigger_state_t trigger_state;
    unsigned trigger_level;
    bool trigger_falling, trace_triggered;
    bool menu, editing;
    unsigned setting_row;
    uint32_t edit_value;
    unsigned sweep_pos, sweep_n;
    bool rf_error;
    band_t band;
    uint32_t peak_hz;
    int16_t live[TRACE_W], mx[TRACE_W];
    uint8_t water[WATER_H][TRACE_W];
    unsigned water_head;
} ui_t;

enum { UI_NO_LEVEL = -32768 };
typedef enum { FIELD_CENTER_STEP, FIELD_SPAN, FIELD_START, FIELD_STOP,
               FIELD_EDIT_STEP, FIELD_GAIN, FIELD_DETECTOR, FIELD_SCALE,
               FIELD_PAUSE, FIELD_MAX_HOLD, FIELD_RBW, FIELD_TIME,
               FIELD_TRIGGER_MODE, FIELD_TRIGGER_LEVEL, FIELD_TRIGGER_EDGE, FIELD_TRIGGER_ARM } settings_field_t;
enum { CENTER_STEP_COUNT = 5, EDIT_STEP_COUNT = 7, GAIN_COUNT = 5, SCALE_COUNT = 4 };
extern const unsigned center_steps_mhz[CENTER_STEP_COUNT];
extern const unsigned edit_steps_khz[EDIT_STEP_COUNT];
extern const int gain_choices[GAIN_COUNT], scale_choices[SCALE_COUNT];
settings_field_t settings_field(mode_tuner_t mode, unsigned row);
unsigned settings_rows(const ui_t *u);
uint32_t settings_value(const ui_t *u, settings_field_t field);
void display_reset(ui_t *u);
void display_trace(ui_t *u, const trace_t *t);
void display_time_trace(ui_t *u, const float power[TRACE_W]);
esp_err_t display_draw(const ui_t *u);
