#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "spectrum_model.h"
#include "zero_span.h"

typedef struct { unsigned error, detail; } rf_status_t;
esp_err_t radio_init(void);
/* gain < 0 is hardware AGC; rate is the ESP-SDR 0/1/6 hardware code. */
rf_status_t radio_capture(unsigned mhz, unsigned rate, unsigned filter_mhz, int gain,
                          unsigned dwell_ms, bool peak, float power[FFT_N]);
rf_status_t radio_capture_time(unsigned mhz, unsigned rbw_hz, unsigned span_us, int gain,
                               bool peak, zero_trigger_t trigger, zero_trace_result_t *trace_result,
                               float power[TRACE_W]);
