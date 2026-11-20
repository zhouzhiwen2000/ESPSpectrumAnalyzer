#include "spectrum_model.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static float power[FFT_N];

static void sweep_coverage(const sweep_t *s) {
    trace_t t;
    trace_clear(&t);
    for (unsigned k = 0; k < FFT_N; ++k) power[k] = 1;
    for (unsigned i = 0; i < s->count; ++i)
        trace_merge(&t, s->band, sweep_lo(s, i), SWEEP_FS_HZ, SWEEP_DC_SKIP_HZ, SWEEP_WING_HZ, power);
    for (unsigned x = 0; x < TRACE_W; ++x) assert(t.cell[x] == 1);
    assert(sweep_lo(s, s->count - 1) == s->high_mhz);
}

static void sweep_range(void) {
    const band_t initial = sweep_default_band;
    assert(initial.low_hz == 2400000000u && initial.high_hz == 2483500000u);
    const sweep_t ism = sweep_for_band(initial);
    assert(ism.count == 8 && ism.low_mhz == 2408 && ism.high_mhz == 2478);
    sweep_coverage(&ism);

    // Starting from the default SWEEP view, both edges can move beyond the ISM band.
    const band_t expanded_low = sweep_adjust_edge(initial, false, -1000000);
    const band_t limits = sweep_adjust_edge(expanded_low, true, 1000000);
    assert(limits.low_hz == 1850000000u && limits.high_hz == 2750000000u);
    const sweep_t full = sweep_for_band(limits);
    assert(full.count == 91 && full.low_mhz == 1850 && full.high_mhz == 2750);
    sweep_coverage(&full);
    const sweep_t just_below = sweep_for_band(sweep_adjust_edge(initial, false, -500));
    const sweep_t just_above = sweep_for_band(sweep_adjust_edge(initial, true, 500));
    sweep_coverage(&just_below);
    sweep_coverage(&just_above);

    const band_t lower = sweep_adjust_edge(limits, true, -1000000);
    assert(lower.low_hz == limits.low_hz && lower.high_hz == lower.low_hz + SWEEP_MIN_SPAN_HZ);
    const band_t upper = sweep_adjust_edge(limits, false, 1000000);
    assert(upper.high_hz == limits.high_hz && upper.low_hz == upper.high_hz - SWEEP_MIN_SPAN_HZ);
    assert(sweep_adjust_edge(lower, false, -500).low_hz == limits.low_hz);
    assert(sweep_adjust_edge(upper, true, 500).high_hz == limits.high_hz);
    assert(sweep_adjust_edge(lower, false, 500).low_hz == lower.low_hz);
    assert(sweep_adjust_edge(upper, true, -500).high_hz == upper.high_hz);
    const band_t shifted = sweep_adjust_edge(lower, true, 500);
    assert(shifted.high_hz == lower.high_hz + 500000 && shifted.low_hz == lower.low_hz);

    const sweep_t a = sweep_for_band(lower), b = sweep_for_band(upper);
    assert(a.count < full.count && b.count < full.count);
    sweep_coverage(&a);
    sweep_coverage(&b);
    for (uint32_t lo = limits.low_hz; lo + SWEEP_MIN_SPAN_HZ <= limits.high_hz; lo += 500000) {
        const sweep_t s = sweep_for_band((band_t){lo, lo + SWEEP_MIN_SPAN_HZ});
        sweep_coverage(&s);
    }
}

int main(void) {
    assert(center_adjust_mhz(2442, -5) == 2437);
    assert(center_adjust_mhz(2442, 5) == 2447);
    assert(center_adjust_mhz(1851, -5) == 1850);
    assert(center_adjust_mhz(2749, 5) == 2750);
    assert(bin_hz(2442, 80000000, 0) == 2442000000u);
    assert(bin_hz(2442, 80000000, 128) == 2432000000u);
    assert(bin_hz(2442, 80000000, 896) == 2452000000u);
    assert(bin_hz(2442, 80000000, 512) == 2482000000u);
    const band_t b = live_band(2442, 60);
    assert(b.low_hz == 2412000000u && b.high_hz == 2472000000u);
    trace_t t;
    trace_clear(&t);
    power[0] = 100000;
    power[128] = 200;
    power[896] = 100;
    power[512] = 900;
    trace_merge(&t, b, 2442, 80000000, 156250, 30000000, power);
    assert(t.peak_hz == 2432000000u && t.peak == 200);
    assert(t.cell[(unsigned)(20ull * TRACE_W / 60u)] == 200);
    assert(t.cell[(unsigned)(40ull * TRACE_W / 60u)] == 100);
    assert(level_half_db((511.f * 32.f) * (511.f * 32.f)) == 0);
    assert(level_half_db((511.f * 32.f) * (511.f * 32.f) / 100.f) == -40);
    assert(level_half_db(0) == -200);
    sweep_range();
    for (unsigned i = 0; i < LIVE_PROFILES; ++i) {
        const live_profile_t *p = &live_profiles[i];
        trace_clear(&t);
        trace_merge(&t, live_band(2442, p->span_mhz), 2442, p->fs_hz,
                    2 * p->fs_hz / FFT_N, p->span_mhz * 500000u, power);
        assert(t.cell[0] > 0 && t.cell[TRACE_W - 1] > 0);
    }
    puts("Spectrum model tests passed");
}
