#include "zero_span.h"
#include <math.h>
#include <string.h>

const unsigned zero_rbws_hz[ZERO_RBW_COUNT] = {30000, 100000, 300000, 1000000, 3000000};
const unsigned zero_times_us[ZERO_TIME_COUNT] = {
    100, 250, 500, ZERO_SHORT_TIME_US, 1000, 2000, 5000, 10000, ZERO_MAX_TIME_US,
};

unsigned zero_span_record_pairs(unsigned span_us, bool triggered) {
    unsigned extra_us = span_us / ZERO_TRIGGER_MARGIN_DIV;
    if (extra_us < ZERO_MIN_TRIGGER_MARGIN_US) extra_us = ZERO_MIN_TRIGGER_MARGIN_US;
    return (ZERO_PREROLL_US + span_us + (triggered ? extra_us : 0u)) * ZERO_PAIRS_PER_US;
}

int zero_trigger_level_dbfs(unsigned index) {
    return ZERO_TRIGGER_MIN_DBFS + (int)index * ZERO_TRIGGER_STEP_DB;
}

unsigned zero_span_lo_mhz(unsigned center_mhz) {
    // Low IF keeps RX DC outside the measured channel, including on-center CW.
    const unsigned offset = ZERO_FS_HZ / 4000000u;
    return center_mhz <= LO_MAX_MHZ - offset ? center_mhz + offset : center_mhz - offset;
}

typedef struct { float b0, b1, a1, a2; } lowpass_t;
typedef struct { float z1, z2; } state_t;

static lowpass_t lowpass(unsigned rbw_hz, double q) {
    // Digital fourth-order Butterworth, built from two sections with these Qs.
    // The complex-baseband cutoff is half the specified full RF -3 dB width.
    const double w = 3.14159265358979323846 * rbw_hz / ZERO_FS_HZ;
    const double c = cos(w), alpha = sin(w) / (2.0 * q);
    const double b0 = (1.0 - c) * 0.5 / (1.0 + alpha);
    return (lowpass_t){b0, 2.0 * b0, -2.0 * c / (1.0 + alpha), (1.0 - alpha) / (1.0 + alpha)};
}

static inline float filter(lowpass_t f, state_t *s, float x) {
    const float bx = f.b0 * x;
    const float y = bx + s->z1;
    s->z1 = f.b1 * x - f.a1 * y + s->z2;
    s->z2 = bx - f.a2 * y;
    return y;
}

static inline int signed10(uint32_t bits) {
    const int sample = bits & 1023u;
    return (sample ^ 512) - 512;
}

zero_trace_result_t zero_span_reduce(uint32_t *raw, unsigned pairs, unsigned span_pairs,
                                    bool positive_if, unsigned rbw_hz, bool peak,
                                    zero_trigger_t trigger, float power[TRACE_W]) {
    const unsigned warm = ZERO_PREROLL_US * ZERO_PAIRS_PER_US;
    if (!raw || !power || pairs > ZERO_MAX_RECORD_PAIRS || pairs <= warm || span_pairs > pairs - warm || span_pairs < TRACE_W ||
        rbw_hz < zero_rbws_hz[0] || rbw_hz > zero_rbws_hz[ZERO_RBW_COUNT - 1]) return ZERO_INVALID;

    const lowpass_t lo = lowpass(rbw_hz, 0.541196100146197), hi = lowpass(rbw_hz, 1.306562964876377);
    state_t i0 = {0}, i1 = {0}, q0 = {0}, q1 = {0};
    // Remove receiver DC before mixing. The selected signal is around +/- Fs/4 here.
    int sum_i = 0, sum_q = 0;
    for (unsigned k = 0; k < pairs; ++k) {
        const uint32_t w = raw[k];
        sum_i += signed10(w);
        sum_q += signed10(w >> 10);
    }
    const float dc_i = (float)sum_i / pairs, dc_q = (float)sum_q / pairs;
    const float level = trigger.enabled ? 511.f * 511.f * powf(10.f, trigger.level_dbfs / 10.f) : 0.f;
    const float arm_level = level * (trigger.falling ? 1.5848932f : 0.6309573f); // 2 dB hysteresis
    const unsigned pre = span_pairs / ZERO_PRETRIGGER_DIV;
    const unsigned latest = pairs - (span_pairs - pre);
    unsigned begin = warm;
    bool armed = false;
    zero_trace_result_t result = trigger.enabled ? ZERO_NO_EDGE : ZERO_UNTRIGGERED;
    unsigned end = trigger.enabled ? pairs : warm + span_pairs;
    for (unsigned k = 0; k < end; ++k) {
        const uint32_t w = raw[k];
        const float a = signed10(w) - dc_i, b = signed10(w >> 10) - dc_q;
        float i = a, q = b;
        // ESP-SDR I+jQ convention: RF below the LO maps to positive IF.
        switch ((positive_if ? k : 0u - k) & 3u) {
        case 1: i = b; q = -a; break;
        case 2: i = -a; q = -b; break;
        case 3: i = -b; q = a; break;
        }
        i = filter(hi, &i1, filter(lo, &i0, i));
        q = filter(hi, &q1, filter(lo, &q0, q));
        if (k < warm) continue;
        const float p = i * i + q * q;
        memcpy(&raw[k], &p, sizeof(p));
        if (result != ZERO_NO_EDGE || k > latest) continue;
        if (armed && (trigger.falling ? p <= level : p >= level)) {
            armed = false;
            if (k < warm + pre) continue; // This edge did not have enough pre-trigger data.
            begin = k - pre;
            end = begin + span_pairs;
            result = ZERO_EDGE;
        } else if (trigger.falling ? p >= arm_level : p <= arm_level) {
            armed = true;
        }
    }
    if (result == ZERO_NO_EDGE && !trigger.fallback) return result;
    unsigned from = begin;
    for (unsigned col = 1; col <= TRACE_W; ++col) {
        const unsigned to = begin + span_pairs * col / TRACE_W;
        float acc = 0.f;
        for (unsigned k = from; k < to; ++k) {
            float p;
            memcpy(&p, &raw[k], sizeof(p));
            acc = peak ? (p > acc ? p : acc) : acc + p;
        }
        power[col - 1] = peak ? acc : acc / (to - from);
        from = to;
    }
    return result == ZERO_NO_EDGE ? ZERO_UNTRIGGERED : result;
}

int zero_span_level_half_db(float power) {
    return power > 0 ? (int)lroundf(20.0f * log10f(power / (511.0f * 511.0f))) : -200;
}
