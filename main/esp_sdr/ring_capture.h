/* ESP-SDR bank rotation and on-chip spectrum, adapted for the StickS3. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "spectrum_dc.h"

#define RING_BANKS         3u
#define RING_BANK_BASE     0x3fcb0000u
#define RING_BANK_END      0x3fce0000u
#define RING_BANK_STRIDE   0x10000u
#define RING_PAIRS         16384u
#define RING_THRESHOLD     12288u
#define RING_SPEC_NFFT_MAX 1024u
#define RING_RECORD_ALIGN  64u
#define RING_RECORD_MAX_PAIRS 409600u
/* Round up for cache maintenance, including up to 15 leading words. */
#define RING_RECORD_BYTES(pairs) (((pairs) * 4u + 2u * RING_RECORD_ALIGN - 1u) & ~(RING_RECORD_ALIGN - 1u))

typedef enum {
    RING_OK = 0,
    RING_FAIL_ARG,
    RING_FAIL_LATE,
    RING_FAIL_AGE,
    RING_FAIL_START,
    RING_FAIL_END,
    RING_FAIL_LENGTH,
    RING_FAIL_COPY,
} ring_status_t;

typedef struct {
    unsigned rate;         /* code 0 = 80, 1 = 40, 6 = 16 MS/s */
    unsigned duration_ms;  /* bounded to 1–100 ms */
    unsigned nfft;
    bool max_hold;          /* peak power across windows; false for mean */
    float *local_power;     /* nfft linear power bins, natural FFT order */
    uint32_t *record;      /* optional IQ destination, RING_RECORD_ALIGN and RING_RECORD_BYTES */
    unsigned record_pairs; /* raw record at 16 MS/s, in place of the FFT */
} ring_config_t;

typedef struct {
    uint32_t status, detail, units;
    uint64_t pairs, elapsed_us;
    uint32_t late_max, work_max, abandoned, ffts;
    uint32_t *record;       /* first captured pair, within the aligned destination */
} ring_result_t;

void ring_capture_init(void);
bool ring_capture_core1_alive(void);
/* Caller prepares the RX path. Interrupts are masked during bank rotation. */
void ring_capture_run(const ring_config_t *config, ring_result_t *result);
/* One contiguous IQ10 snapshot; pointer valid until the next RF dump. */
ring_status_t ring_capture_burst(unsigned rate, unsigned count, uint32_t **samples);
