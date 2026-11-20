#include "zero_span.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

enum { SHORT_PAIRS = (ZERO_PREROLL_US + ZERO_SHORT_TIME_US) * ZERO_PAIRS_PER_US };
static uint32_t iq[ZERO_MAX_RECORD_PAIRS];
static float out[TRACE_W], peak[TRACE_W];
static const unsigned short_span = ZERO_SHORT_TIME_US * ZERO_PAIRS_PER_US;
static const zero_trigger_t free_run = {0};

static uint32_t packed(int i, int q) {
    return ((uint32_t)i & 1023u) | (((uint32_t)q & 1023u) << 10);
}

static void tone(double hz, bool pulse) {
    for (unsigned k = 0; k < SHORT_PAIRS; ++k) {
        const double radians = 6.2831853071795864769 * hz * k / ZERO_FS_HZ;
        const int a = !pulse || (k >= 6000 && k < 10000) ? 180 : 0;
        iq[k] = packed(79 + (int)lround(a * cos(radians)), -37 + (int)lround(a * sin(radians)));
    }
}

static double mean_out(unsigned first, unsigned end) {
    double sum = 0;
    for (unsigned i = first; i < end; ++i) sum += out[i];
    return sum / (end - first);
}

static void pulse(unsigned pairs, unsigned start, unsigned stop) {
    static const int cosine[4] = {1, 0, -1, 0};
    static const int sine[4] = {0, 1, 0, -1};
    for (unsigned k = 0; k < pairs; ++k) {
        const int a = k >= start && k < stop ? 180 : 0;
        iq[k] = packed(79 + a * cosine[k % 4], -37 + a * sine[k % 4]);
    }
}

static void test_trigger(void) {
    const unsigned span = ZERO_MAX_TIME_US * ZERO_PAIRS_PER_US;
    const unsigned record = zero_span_record_pairs(ZERO_MAX_TIME_US, true);
    const unsigned warm = ZERO_PREROLL_US * ZERO_PAIRS_PER_US;
    const unsigned column = TRACE_W / ZERO_PRETRIGGER_DIV;
    zero_trigger_t trigger = {.enabled = true, .level_dbfs = -20};
    assert(zero_trigger_level_dbfs(ZERO_TRIGGER_DEFAULT_LEVEL) == -40);

    // First pulse is too early for the requested pretrigger; use the later edge.
    const unsigned edge = warm + span / ZERO_PRETRIGGER_DIV + 24000u;
    pulse(record, edge, record);
    for (unsigned k = warm + 400; k < warm + 800; ++k) {
        static const int cosine[4] = {1, 0, -1, 0}, sine[4] = {0, 1, 0, -1};
        iq[k] = packed(79 + 180 * cosine[k % 4], -37 + 180 * sine[k % 4]);
    }
    assert(zero_span_reduce(iq, record, span, true, 300000, false, trigger, out) == ZERO_EDGE);
    assert(out[column - 1] < 1 && out[column + 1] > 30000);

    trigger.falling = true;
    pulse(record, 0, edge);
    assert(zero_span_reduce(iq, record, span, true, 300000, false, trigger, out) == ZERO_EDGE);
    assert(out[column - 1] > 30000 && out[column + 1] < 1);

    trigger.falling = false;
    pulse(record, 0, record);
    out[0] = -1;
    assert(zero_span_reduce(iq, record, span, true, 300000, false, trigger, out) == ZERO_NO_EDGE);
    assert(out[0] == -1); // NORMAL/SINGLE keep the last displayed trace.
    pulse(record, 0, record);
    trigger.fallback = true;
    assert(zero_span_reduce(iq, record, span, true, 300000, false, trigger, out) == ZERO_UNTRIGGERED);
    assert(out[0] > 30000);

    // AUTO must still prefer a real edge when fallback is enabled.
    pulse(record, edge, record);
    assert(zero_span_reduce(iq, record, span, true, 300000, false, trigger, out) == ZERO_EDGE);
    pulse(record, record - 1000u, record);
    trigger.fallback = false;
    assert(zero_span_reduce(iq, record, span, true, 300000, false, trigger, out) == ZERO_NO_EDGE);
}

int main(void) {
    assert(zero_span_lo_mhz(1850) == 1854 && zero_span_lo_mhz(2442) == 2446);
    assert(zero_span_lo_mhz(2750) == 2746);
    assert(zero_span_level_half_db(511.f * 511.f) == 0);
    assert(zero_span_reduce(iq, 10, short_span, true, 300000, false, free_run, out) == ZERO_INVALID);
    for (unsigned i = 0; i < ZERO_RBW_COUNT; ++i) {
        const unsigned rbw = zero_rbws_hz[i];
        tone(4000000, false);
        assert(zero_span_reduce(iq, SHORT_PAIRS, short_span, true, rbw, false, free_run, out) == ZERO_UNTRIGGERED);
        const double on = mean_out(8, TRACE_W);
        assert(on > 0.97 * 180 * 180 && on < 1.03 * 180 * 180);
        for (int side = -1; side <= 1; side += 2) {
            tone(4000000 + side * (rbw / 2.0), false);
            assert(zero_span_reduce(iq, SHORT_PAIRS, short_span, true, rbw, false, free_run, out) == ZERO_UNTRIGGERED);
            const double ratio = mean_out(8, TRACE_W) / on;
            assert(ratio > 0.48 && ratio < 0.52);
        }
        tone(4000000 + rbw, false);
        assert(zero_span_reduce(iq, SHORT_PAIRS, short_span, true, rbw, false, free_run, out) == ZERO_UNTRIGGERED);
        assert(mean_out(8, TRACE_W) / on < 0.007);
        tone(-4000000, true);
        assert(zero_span_reduce(iq, SHORT_PAIRS, short_span, false, rbw, false, free_run, out) == ZERO_UNTRIGGERED);
        tone(-4000000, true);
        assert(zero_span_reduce(iq, SHORT_PAIRS, short_span, false, rbw, true, free_run, peak) == ZERO_UNTRIGGERED);
        assert(mean_out(0, 20) < 1 && mean_out(75, 100) > 0.97 * on && mean_out(150, 202) < 1);
        for (unsigned k = 0; k < TRACE_W; ++k) assert(peak[k] + 0.05f >= out[k]);
    }
    test_trigger();
    puts("Zero span DSP and trigger tests passed");
}
