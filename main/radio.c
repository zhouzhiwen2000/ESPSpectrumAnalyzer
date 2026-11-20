// ESP32-S3 PHY setup adapted from ESPARGOS/esp-sdr (GPL-3.0-or-later).
#include "radio.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "heap_memory_layout.h"
#include "ring_capture.h"
#include "rx_bandwidth.h"
#include "rx_lo.h"
#include "rx_recalibration.h"
#include "soc/soc.h"

#define TAG "radio"
SOC_RESERVE_MEMORY_REGION(RING_BANK_BASE, RING_BANK_END, s3_rf_dump);
_Static_assert(FFT_N <= RING_SPEC_NFFT_MAX, "FFT capacity");
_Static_assert((ZERO_PREROLL_US + ZERO_SHORT_TIME_US) * ZERO_PAIRS_PER_US <= RING_PAIRS - 4u,
               "Zero span burst capacity");
_Static_assert(ZERO_MAX_RECORD_PAIRS <= RING_RECORD_MAX_PAIRS, "Zero span GDMA record capacity");

extern void stop_tx_tone(unsigned);
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(unsigned);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_set_rxclk_en(unsigned);
extern void set_chanfreq(unsigned, unsigned);
extern void set_rf_freq_offset(unsigned, unsigned, int);
extern bool rx_gain_verified_apply(bool manual, unsigned index);
extern bool rx_gain_verified_check(unsigned index);
unsigned spectrum_dc_mode = 1;

/* Required by the SDK RF archive. No factory/serial command parser is exposed. */
int cmd_parse(char *cmd, char *name, int *argc, char **argv) {
    (void)cmd; (void)name; (void)argc; (void)argv;
    return -1;
}

esp_err_t radio_init(void) {
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = 0;
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "Wi-Fi PHY init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "Wi-Fi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_NULL), TAG, "RX-only mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "Wi-Fi power saving off");
    ESP_RETURN_ON_ERROR(esp_wifi_set_promiscuous(true), TAG, "Wi-Fi RX enable");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE), TAG, "initial channel");
    ring_capture_init();
    ESP_LOGI(TAG, "Spectrum worker: %s", ring_capture_core1_alive() ? "core 1" : "single core fallback");
    return ESP_OK;
}

static bool set_receiver(unsigned mhz, int gain) {
    static unsigned tuned;
    static int applied = -2;
    if (tuned != mhz || rx_recalibration_stale()) {
        rx_recalibrate(mhz);
        rx_lo_plan_t plan = rx_lo_plan(mhz);
        bool channel = (mhz >= 2412 && mhz <= 2472 && (mhz - 2412) % 5 == 0) || mhz == 2484;
        rx_lo_select(false);
        set_chanfreq(channel ? mhz : 2412, 0);
        if (!channel) set_rf_freq_offset(0, plan.mhz, plan.offset_khz);
        stop_tx_tone(1);
        rom_pbus_workmode();
        rom_pbus_xpd_tx_off();
        rom_pbus_xpd_rx_on(1);
        rom_set_rxclk_en(1);
        rx_lo_select(plan.alternate);
        esp_rom_delay_us(3000);
        tuned = mhz;
        applied = -2;
    }
    if (gain >= 0) {
        const unsigned gain_max = (REG_READ(0x6001c02cu) >> 8) & 127u;
        if (gain_max > 82 || (unsigned)gain > gain_max) return false;
    }
    if (applied == gain) return gain < 0 || rx_gain_verified_check((unsigned)gain);
    if (!rx_gain_verified_apply(gain >= 0, gain < 0 ? 40u : (unsigned)gain)) return false;
    applied = gain;
    return true;
}

static void filter_begin(unsigned filter_mhz, unsigned saved[2]) {
    const unsigned cap = rx_bandwidth_dcap(filter_mhz);
    for (unsigned j = 0; j < 2; ++j) {
        saved[j] = rom_chip_i2c_readReg(0x67, 0, 4 + j);
        rom_chip_i2c_writeReg(0x67, 0, 4 + j, (saved[j] & ~63u) | cap);
    }
}

static void filter_end(const unsigned saved[2]) {
    for (unsigned j = 0; j < 2; ++j) rom_chip_i2c_writeReg(0x67, 0, 4 + j, saved[j]);
}

rf_status_t radio_capture(unsigned mhz, unsigned rate, unsigned filter_mhz, int gain,
                          unsigned dwell_ms, bool peak, float power[FFT_N]) {
    if (!set_receiver(mhz, gain)) return (rf_status_t){.error = 100};
    unsigned saved[2];
    filter_begin(filter_mhz, saved);
    const ring_config_t cfg = {
        .rate = rate, .duration_ms = dwell_ms,
        .nfft = FFT_N, .max_hold = peak, .local_power = power,
    };
    ring_result_t result;
    ring_capture_run(&cfg, &result);
    filter_end(saved);
    if (result.status) return (rf_status_t){result.status, result.detail};
    if (!result.ffts) return (rf_status_t){.error = 101};
    return (rf_status_t){0};
}

rf_status_t radio_capture_time(unsigned mhz, unsigned rbw_hz, unsigned span_us, int gain,
                               bool peak, zero_trigger_t trigger, zero_trace_result_t *trace_result,
                               float power[TRACE_W]) {
    if (!span_us || span_us > ZERO_MAX_TIME_US) return (rf_status_t){.error = RING_FAIL_ARG};
    const unsigned pairs = zero_span_record_pairs(span_us, trigger.enabled);
    static uint32_t *record;
    if (pairs > RING_PAIRS - 4u && !record) {
        record = heap_caps_aligned_alloc(RING_RECORD_ALIGN, RING_RECORD_BYTES(ZERO_MAX_RECORD_PAIRS),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (!record) return (rf_status_t){.error = 102};
    }
    const unsigned lo_mhz = zero_span_lo_mhz(mhz);
    if (!set_receiver(lo_mhz, gain)) return (rf_status_t){.error = 100};
    unsigned saved[2];
    uint32_t *raw;
    rf_status_t status;
    filter_begin(13, saved);
    if (pairs <= RING_PAIRS - 4u) {
        status = (rf_status_t){.error = ring_capture_burst(6, pairs, &raw)};
    } else {
        const ring_config_t cfg = {.rate = 6, .record = record, .record_pairs = pairs};
        ring_result_t r;
        ring_capture_run(&cfg, &r);
        raw = r.record;
        status = (rf_status_t){r.status, r.detail};
    }
    filter_end(saved);
    if (status.error) return status;
    *trace_result = zero_span_reduce(raw, pairs, span_us * ZERO_PAIRS_PER_US, lo_mhz > mhz,
                                     rbw_hz, peak, trigger, power);
    return *trace_result == ZERO_INVALID ? (rf_status_t){.error = 101} : (rf_status_t){0};
}
