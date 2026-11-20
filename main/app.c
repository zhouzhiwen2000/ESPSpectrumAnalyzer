#include "board.h"
#include "display.h"
#include "radio.h"
#include "spectrum_model.h"
#include "zero_span.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "spectrum";
static ui_t ui = {
    .mode = MODE_SWEEP, .profile = 2, .rbw_i = 2, .time_i = 3,
    .trigger_level = ZERO_TRIGGER_DEFAULT_LEVEL,
    .settings = {
        [MODE_SWEEP] = {.step_i = 2, .scale_i = 1, .peak_detector = true},
        [MODE_RT] = {.center_mhz = 2442, .step_i = 1, .scale_i = 1, .peak_detector = true},
        [MODE_ZERO_SPAN] = {.center_mhz = 2442, .step_i = 1, .scale_i = 1, .peak_detector = true},
    },
};
static float iq_power[FFT_N];
static trace_t trace;
static sweep_t active_sweep;
static bool dirty;
static unsigned consecutive_errors;
static int64_t zero_last_trace;

static void reset_view(void) {
    ui.sweep_pos = 0;
    ui.rf_error = false;
    consecutive_errors = 0;
    if (ui.mode == MODE_ZERO_SPAN) {
        ui.trigger_state = ZERO_WAITING;
        ui.trace_triggered = false;
        zero_last_trace = esp_timer_get_time();
    }
    if (ui.mode != MODE_SWEEP) {
        if (ui.mode == MODE_RT)
            ui.band = live_band(ui.settings[MODE_RT].center_mhz, live_profiles[ui.profile].span_mhz);
        ui.sweep_n = 0;
    } else {
        active_sweep = sweep_for_band(ui.sweep_band);
        ui.band = active_sweep.band;
        ui.sweep_n = active_sweep.count;
    }
    trace_clear(&trace);
    display_reset(&ui);
    dirty = true;
}

static unsigned rotate(unsigned value, int dir, unsigned count) {
    return (unsigned)((int)value + (int)count + dir) % count;
}

static void tune_center(int dir) {
    mode_settings_t *p = &ui.settings[ui.mode];
    const unsigned step = center_steps_mhz[p->step_i];
    const unsigned next = center_adjust_mhz(p->center_mhz, dir * (int)step);
    if (next == p->center_mhz) return;
    p->center_mhz = next;
    p->paused = false;
    reset_view();
}

static void toggle_max_hold(mode_settings_t *p) {
    p->max_hold = !p->max_hold;
    for (int x = 0; x < TRACE_W; ++x) ui.mx[x] = p->max_hold ? ui.live[x] : UI_NO_LEVEL;
}

static void adjust(int dir) {
    const mode_settings_t *p = &ui.settings[ui.mode];
    const settings_field_t field = settings_field(ui.mode, ui.setting_row);
    uint32_t *v = &ui.edit_value;
    switch (field) {
    case FIELD_START:
    case FIELD_STOP: {
        const bool stop = field == FIELD_STOP;
        band_t current = ui.sweep_band;
        if (stop) current.high_hz = *v;
        else current.low_hz = *v;
        const band_t next = sweep_adjust_edge(current, stop,
                                              dir * (int)edit_steps_khz[p->step_i]);
        *v = stop ? next.high_hz : next.low_hz;
        break;
    }
    case FIELD_CENTER_STEP: *v = rotate(*v, dir, CENTER_STEP_COUNT); break;
    case FIELD_EDIT_STEP: *v = rotate(*v, dir, EDIT_STEP_COUNT); break;
    case FIELD_SPAN: *v = rotate(*v, dir, LIVE_PROFILES); break;
    case FIELD_RBW: *v = rotate(*v, dir, ZERO_RBW_COUNT); break;
    case FIELD_TIME: *v = rotate(*v, dir, ZERO_TIME_COUNT); break;
    case FIELD_GAIN: *v = rotate(*v, dir, GAIN_COUNT); break;
    case FIELD_SCALE: *v = rotate(*v, dir, SCALE_COUNT); break;
    case FIELD_TRIGGER_MODE: *v = rotate(*v, dir, ZERO_TRIGGER_MODES); break;
    case FIELD_TRIGGER_LEVEL:
        if (dir > 0 && *v + 1u < ZERO_TRIGGER_LEVELS) ++*v;
        if (dir < 0 && *v > 0) --*v;
        break;
    case FIELD_TRIGGER_ARM: break;
    case FIELD_TRIGGER_EDGE:
    case FIELD_DETECTOR:
    case FIELD_PAUSE:
    case FIELD_MAX_HOLD: *v = !*v; break;
    }
}

static void commit_setting(void) {
    const settings_field_t field = settings_field(ui.mode, ui.setting_row);
    const uint32_t v = ui.edit_value;
    if (v == settings_value(&ui, field)) return;
    mode_settings_t *p = &ui.settings[ui.mode];
    switch (field) {
    case FIELD_CENTER_STEP:
    case FIELD_EDIT_STEP: p->step_i = v; return;
    case FIELD_PAUSE: p->paused = v != 0; return;
    case FIELD_MAX_HOLD: toggle_max_hold(p); return;
    case FIELD_START: ui.sweep_band.low_hz = v; break;
    case FIELD_STOP: ui.sweep_band.high_hz = v; break;
    case FIELD_SPAN: ui.profile = v; break;
    case FIELD_RBW: ui.rbw_i = v; break;
    case FIELD_TIME: ui.time_i = v; break;
    case FIELD_TRIGGER_MODE: ui.trigger_mode = v; break;
    case FIELD_TRIGGER_LEVEL: ui.trigger_level = v; break;
    case FIELD_TRIGGER_EDGE: ui.trigger_falling = v != 0; break;
    case FIELD_TRIGGER_ARM: return;
    case FIELD_GAIN: p->gain_i = v; break;
    case FIELD_DETECTOR: p->peak_detector = v != 0; break;
    case FIELD_SCALE: p->scale_i = v; break;
    }
    reset_view();
}

static void handle_keys(keys_t keys) {
    if (!keys.a && !keys.b) return;
    dirty = true;
    if (ui.menu) {
        if (keys.b == KEY_LONG) {
            if (ui.editing) ui.editing = false;
            else {
                ui.menu = false;
                zero_last_trace = esp_timer_get_time();
            }
        } else if (keys.a == KEY_LONG) {
            if (settings_field(ui.mode, ui.setting_row) == FIELD_TRIGGER_ARM) {
                ui.settings[MODE_ZERO_SPAN].paused = false;
                ui.trigger_state = ZERO_WAITING;
                return;
            }
            if (ui.editing) commit_setting();
            else ui.edit_value = settings_value(&ui, settings_field(ui.mode, ui.setting_row));
            ui.editing = !ui.editing;
        } else if (ui.editing) {
            if (keys.a == KEY_TAP) adjust(1);
            if (keys.b == KEY_TAP) adjust(-1);
        } else {
            if (keys.a == KEY_TAP) ui.setting_row = rotate(ui.setting_row, 1, settings_rows(&ui));
            if (keys.b == KEY_TAP) ui.setting_row = rotate(ui.setting_row, -1, settings_rows(&ui));
        }
        return;
    }
    if (keys.b == KEY_LONG) {
        ui.mode = (ui.mode + 1) % MODE_COUNT;
        reset_view();
    } else if (keys.a == KEY_LONG) {
        ui.menu = true;
        ui.editing = false;
        ui.setting_row = 0;
    } else if (ui.mode != MODE_SWEEP) {
        if (keys.a == KEY_TAP) tune_center(1);
        if (keys.b == KEY_TAP) tune_center(-1);
    } else {
        mode_settings_t *p = &ui.settings[MODE_SWEEP];
        if (keys.a == KEY_TAP) p->paused = !p->paused;
        if (keys.b == KEY_TAP) toggle_max_hold(p);
    }
}

static bool measurement_ok(unsigned lo, rf_status_t r) {
    if (r.error) {
        if (++consecutive_errors == 1 || consecutive_errors % 30 == 0)
            ESP_LOGW(TAG, "%u MHz: RF failure %u (%u)", lo, r.error, r.detail);
        if (consecutive_errors == 3) { ui.rf_error = true; dirty = true; }
        return false;
    }
    if (ui.rf_error) { ui.rf_error = false; dirty = true; }
    consecutive_errors = 0;
    return true;
}

static bool measure(unsigned lo, unsigned rate, unsigned fs, unsigned filter, unsigned dwell,
                    unsigned skip, unsigned wing) {
    const mode_settings_t *p = &ui.settings[ui.mode];
    const rf_status_t r = radio_capture(lo, rate, filter, gain_choices[p->gain_i], dwell, p->peak_detector, iq_power);
    if (!measurement_ok(lo, r)) return false;
    trace_merge(&trace, ui.band, lo, fs, skip, wing, iq_power);
    return true;
}

static void measure_zero_span(void) {
    if (ui.trigger_state == ZERO_HELD) return;
    const mode_settings_t *p = &ui.settings[MODE_ZERO_SPAN];
    const zero_trigger_t trigger = {
        .enabled = ui.trigger_mode != ZERO_FREE,
        .falling = ui.trigger_falling,
        .fallback = ui.trigger_mode == ZERO_AUTO && esp_timer_get_time() - zero_last_trace >= ZERO_AUTO_TIMEOUT_US,
        .level_dbfs = zero_trigger_level_dbfs(ui.trigger_level),
    };
    zero_trace_result_t result;
    const rf_status_t r = radio_capture_time(p->center_mhz, zero_rbws_hz[ui.rbw_i], zero_times_us[ui.time_i],
                                            gain_choices[p->gain_i], p->peak_detector, trigger, &result, iq_power);
    if (!measurement_ok(p->center_mhz, r)) return;
    if (result == ZERO_NO_EDGE) {
        if (ui.trigger_state != ZERO_WAITING) dirty = true;
        ui.trigger_state = ZERO_WAITING;
        return;
    }
    const bool triggered = result == ZERO_EDGE;
    if (ui.trace_triggered != triggered) display_reset(&ui); // Different time origin for MAX HOLD.
    ui.trace_triggered = triggered;
    ui.trigger_state = !triggered ? ZERO_WAITING : ui.trigger_mode == ZERO_SINGLE ? ZERO_HELD : ZERO_TRIGGERED;
    zero_last_trace = esp_timer_get_time();
    display_time_trace(&ui, iq_power);
    dirty = true;
}

void app_main(void) {
    ESP_ERROR_CHECK(board_init());
    ui.sweep_band = sweep_default_band;
    reset_view();
    ESP_ERROR_CHECK(display_draw(&ui));
    ESP_ERROR_CHECK(board_backlight_on());
    if (radio_init() != ESP_OK) {
        ui.rf_error = true;
        ESP_ERROR_CHECK(display_draw(&ui));
        return;
    }
    int64_t last_paint = 0;
    for (;;) {
        keys_t keys;
        while ((keys = board_keys()).a || keys.b) handle_keys(keys);
        if (dirty) {
            ESP_ERROR_CHECK(display_draw(&ui));
            last_paint = esp_timer_get_time();
            dirty = false;
            handle_keys(board_keys());
        }
        const bool paused = ui.settings[ui.mode].paused;
        if (!ui.menu && !paused) {
            if (ui.mode == MODE_ZERO_SPAN) {
                measure_zero_span();
            } else if (ui.mode == MODE_RT) {
                const live_profile_t *p = &live_profiles[ui.profile];
                trace_clear(&trace);
                if (measure(ui.settings[MODE_RT].center_mhz, p->rate_code, p->fs_hz, p->filter_mhz, 42,
                            2u * p->fs_hz / FFT_N, p->span_mhz * 500000u)) {
                    display_trace(&ui, &trace);
                    dirty = true;
                }
            } else {
                measure(sweep_lo(&active_sweep, ui.sweep_pos), 1, SWEEP_FS_HZ, 30, 9,
                        SWEEP_DC_SKIP_HZ, SWEEP_WING_HZ);
                if (++ui.sweep_pos == ui.sweep_n) {
                    display_trace(&ui, &trace);
                    trace_clear(&trace);
                    ui.sweep_pos = 0;
                    dirty = true;
                } else if (esp_timer_get_time() - last_paint > 125000) dirty = true;
            }
        }
        // Feed the scheduler and let Wi-Fi housekeeping run between RF windows.
        const bool held = ui.mode == MODE_ZERO_SPAN && ui.trigger_state == ZERO_HELD;
        vTaskDelay(pdMS_TO_TICKS(ui.menu || paused || held ? 12 : 3));
    }
}
