#pragma once
#include <stdbool.h>
#include <stdint.h>

enum { FFT_N = 1024, TRACE_W = 202, WATER_H = 20 };
typedef enum { MODE_SWEEP, MODE_RT, MODE_ZERO_SPAN, MODE_COUNT } mode_tuner_t;
typedef struct { uint32_t low_hz, high_hz; } band_t;
typedef struct {
    unsigned low_mhz, high_mhz, step_mhz, count;
    band_t band;
} sweep_t;
typedef struct { unsigned span_mhz, rate_code, fs_hz, filter_mhz; } live_profile_t;
enum { LIVE_PROFILES = 3, LO_MIN_MHZ = 1850, LO_MAX_MHZ = 2750 };
enum { SWEEP_MIN_SPAN_HZ = 10000000, SWEEP_FS_HZ = 40000000,
       SWEEP_DC_SKIP_HZ = 1200000, SWEEP_WING_HZ = 14000000 };
extern const live_profile_t live_profiles[LIVE_PROFILES];
extern const band_t sweep_default_band;

typedef struct {
    float cell[TRACE_W]; // negative means unobserved, 0 means below quantization floor
    float peak;
    uint32_t peak_hz;
} trace_t;

band_t live_band(unsigned center_mhz, unsigned span_mhz);
unsigned center_adjust_mhz(unsigned center_mhz, int delta_mhz);
/* The input band and result lie within the sweep tuning limits. */
band_t sweep_adjust_edge(band_t band, bool stop, int delta_khz);
sweep_t sweep_for_band(band_t band);
unsigned sweep_lo(const sweep_t *s, unsigned step);
uint32_t bin_hz(unsigned center_mhz, unsigned fs_hz, unsigned k);
void trace_clear(trace_t *t);
void trace_merge(trace_t *t, band_t band, unsigned lo_mhz, unsigned fs_hz,
                 unsigned skip_hz, unsigned wing_hz, const float power[FFT_N]);
/* Relative to an ADC full-scale complex tone after the Hann window, 0.5 dB units. */
int level_half_db(float power);
