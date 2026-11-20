#include "spectrum_model.h"
#include <math.h>

const live_profile_t live_profiles[LIVE_PROFILES] = {
    {12, 6, 16000000, 13}, {32, 1, 40000000, 33}, {60, 0, 80000000, 0},
};
const band_t sweep_default_band = {2400000000u, 2483500000u};
enum { SWEEP_LO_STEP_MHZ = 10 };
static const sweep_t sweep_ism = {
    .low_mhz = 2408, .high_mhz = 2478, .step_mhz = SWEEP_LO_STEP_MHZ, .count = 8,
};
static const sweep_t sweep_wide = {
    .low_mhz = LO_MIN_MHZ, .high_mhz = LO_MAX_MHZ, .step_mhz = SWEEP_LO_STEP_MHZ,
    .count = (LO_MAX_MHZ - LO_MIN_MHZ) / SWEEP_LO_STEP_MHZ + 1,
};

band_t live_band(unsigned center_mhz, unsigned span_mhz) {
    return (band_t){(center_mhz * 2u - span_mhz) * 500000u,
                    (center_mhz * 2u + span_mhz) * 500000u};
}

unsigned center_adjust_mhz(unsigned center_mhz, int delta_mhz) {
    const int64_t next = (int64_t)center_mhz + delta_mhz;
    return next < LO_MIN_MHZ ? LO_MIN_MHZ : next > LO_MAX_MHZ ? LO_MAX_MHZ : (unsigned)next;
}

band_t sweep_adjust_edge(band_t band, bool stop, int delta_khz) {
    const uint32_t min = stop ? band.low_hz + SWEEP_MIN_SPAN_HZ : LO_MIN_MHZ * 1000000u;
    const uint32_t max = stop ? LO_MAX_MHZ * 1000000u : band.high_hz - SWEEP_MIN_SPAN_HZ;
    uint32_t *edge = stop ? &band.high_hz : &band.low_hz;
    const int64_t hz = (int64_t)*edge + (int64_t)delta_khz * 1000;
    *edge = hz < min ? min : hz > max ? max : (uint32_t)hz;
    return band;
}

sweep_t sweep_for_band(band_t band) {
    // Keep the shorter LO grid for ISM-only ranges; use the full grid otherwise.
    const sweep_t *preset = band.low_hz >= sweep_default_band.low_hz && band.high_hz <= sweep_default_band.high_hz
                          ? &sweep_ism : &sweep_wide;
    unsigned first = 0, last = preset->count - 1;
    while (first < last && sweep_lo(preset, first) * 1000000u + SWEEP_WING_HZ < band.low_hz)
        ++first;
    while (first < last && sweep_lo(preset, last) * 1000000u - SWEEP_WING_HZ > band.high_hz)
        --last;
    return (sweep_t){sweep_lo(preset, first), sweep_lo(preset, last), preset->step_mhz,
                    last - first + 1, band};
}

unsigned sweep_lo(const sweep_t *s, unsigned step) {
    unsigned lo = s->low_mhz + step * s->step_mhz;
    return lo <= s->high_mhz ? lo : s->high_mhz;
}

uint32_t bin_hz(unsigned center_mhz, unsigned fs_hz, unsigned k) {
    const int offset = k < FFT_N / 2 ? (int)k : (int)k - FFT_N;
    // ESP-SDR S3 dump convention: positive I+jQ frequency is below the LO.
    return (int64_t)center_mhz * 1000000 - (int64_t)offset * fs_hz / FFT_N;
}

void trace_clear(trace_t *t) {
    for (unsigned x = 0; x < TRACE_W; ++x) t->cell[x] = -1.0f;
    t->peak = -1.0f;
    t->peak_hz = 0;
}

void trace_merge(trace_t *t, band_t band, unsigned lo_mhz, unsigned fs_hz,
                 unsigned skip_hz, unsigned wing_hz, const float power[FFT_N]) {
    const int64_t width = (int64_t)band.high_hz - band.low_hz;
    if (width <= 0) return;
    for (unsigned k = 0; k < FFT_N; ++k) {
        const unsigned abs_bin = k <= FFT_N / 2 ? k : FFT_N - k;
        const unsigned delta = (uint64_t)abs_bin * fs_hz / FFT_N;
        if (delta <= skip_hz || delta > wing_hz || power[k] < 0 || !isfinite(power[k])) continue;
        const uint32_t hz = bin_hz(lo_mhz, fs_hz, k);
        const int64_t offset = (int64_t)hz - band.low_hz;
        if (offset < 0 || offset > width) continue;
        unsigned x = (unsigned)(offset * TRACE_W / width);
        if (x == TRACE_W) --x;
        if (power[k] > t->cell[x]) t->cell[x] = power[k];
        if (power[k] > t->peak) { t->peak = power[k]; t->peak_hz = hz; }
    }
}

int level_half_db(float power) {
    // Q15 scaling is 64 * ADC code; the Hann coherent gain is 1/2.
    const float adc_fs_power = (511.0f * 32.0f) * (511.0f * 32.0f);
    return power > 0 ? (int)lroundf(20.0f * log10f(power / adc_fs_power)) : -200;
}
