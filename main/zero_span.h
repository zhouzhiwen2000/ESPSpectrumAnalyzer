#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "spectrum_model.h"

enum { ZERO_FS_HZ = 16000000, ZERO_PAIRS_PER_US = ZERO_FS_HZ / 1000000,
       ZERO_PREROLL_US = 200, ZERO_SHORT_TIME_US = 800, ZERO_MAX_TIME_US = 20000,
       ZERO_RBW_COUNT = 5, ZERO_TIME_COUNT = 9, ZERO_PRETRIGGER_DIV = 5,
       ZERO_TRIGGER_MARGIN_DIV = 4, ZERO_MIN_TRIGGER_MARGIN_US = 400,
       ZERO_MAX_RECORD_PAIRS = (ZERO_PREROLL_US + ZERO_MAX_TIME_US
                                + ZERO_MAX_TIME_US / ZERO_TRIGGER_MARGIN_DIV) * ZERO_PAIRS_PER_US,
       ZERO_TRIGGER_MIN_DBFS = -100, ZERO_TRIGGER_STEP_DB = 2,
       ZERO_TRIGGER_LEVELS = 1 - ZERO_TRIGGER_MIN_DBFS / ZERO_TRIGGER_STEP_DB,
       ZERO_TRIGGER_DEFAULT_LEVEL = (-40 - ZERO_TRIGGER_MIN_DBFS) / ZERO_TRIGGER_STEP_DB,
       ZERO_AUTO_TIMEOUT_US = 250000 };
extern const unsigned zero_rbws_hz[ZERO_RBW_COUNT], zero_times_us[ZERO_TIME_COUNT];

typedef enum { ZERO_FREE, ZERO_AUTO, ZERO_NORMAL, ZERO_SINGLE, ZERO_TRIGGER_MODES } zero_trigger_mode_t;
typedef enum { ZERO_WAITING, ZERO_TRIGGERED, ZERO_HELD } zero_trigger_state_t;
typedef enum { ZERO_INVALID, ZERO_NO_EDGE, ZERO_UNTRIGGERED, ZERO_EDGE } zero_trace_result_t;
typedef struct {
    bool enabled, falling, fallback;
    int level_dbfs;
} zero_trigger_t;

unsigned zero_span_lo_mhz(unsigned center_mhz);
unsigned zero_span_record_pairs(unsigned span_us, bool triggered);
int zero_trigger_level_dbfs(unsigned index);
/* Filters the raw IQ10 in place and produces linear power per time column. */
zero_trace_result_t zero_span_reduce(uint32_t *raw, unsigned pairs, unsigned span_pairs,
                                    bool positive_if, unsigned rbw_hz, bool peak,
                                    zero_trigger_t trigger, float power[TRACE_W]);
int zero_span_level_half_db(float power);
