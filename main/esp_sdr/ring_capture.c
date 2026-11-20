/*
 * Continuous RF capture: ESP32-S3 bank rotation.
 *
 * The ADC dump engine writes IQ pairs at a ring index that advances
 * continuously (mod 16384) into whichever of the SRAM banks is selected.
 * Switching the selected bank while the engine runs therefore splits the
 * sample stream into gapless "units", one per bank visit. The scheme follows
 * h0m3us3r/eSpDR capture.c and ESPARGOS/esp-sdr:
 *
 *   - while the writer fills bank b, bank b+1 gets sentinel words where the
 *     next unit is predicted to start and to end;
 *   - once THRESHOLD pairs are in bank b, the writer is moved to bank b+1;
 *   - the exact first and last pair of the finished unit are located by
 *     binary search over the sentinel windows, and each unit's first pair
 *     must equal the previous unit's end, which proves the stream gapless.
 *
 * Raw records use GDMA to transfer completed units to PSRAM before bank reuse.
 * In spectrum mode, the CPU runs short work slices (individual FFT stages) on units
 * already finished, so the switch latency is bounded by one slice. When the
 * processing falls behind, the unfinished rest of the oldest unit is
 * abandoned instead of stalling the ring (counted in `abandoned`).
 *
 * Interrupts remain masked for a bounded capture (no RTOS, Wi-Fi ISR,
 * or display driver during capture). The completed spectrum stays in RAM.
 * These targets require CONFIG_ESP_INT_WDT=n.
 */
#include "ring_capture.h"
#include "sdkconfig.h"

#include <math.h>
#include <string.h>

/* esp-dsp aes3 FFT with the sum-branch bias corrected (s3_fft_rnd.S). */
static int16_t *fft_twiddles;
int s3_fft2r_sc16_rnd(int16_t *data, int N, int16_t *w);
int s3_fft2r_sc16_rnd_stage(int16_t *data, int N, int16_t *w, unsigned stage);
#define S3_FFT(buf, n)              s3_fft2r_sc16_rnd((buf), (int)(n), fft_twiddles)
#define S3_FFT_STAGE(buf, n, stage) s3_fft2r_sc16_rnd_stage((buf), (int)(n), fft_twiddles, (stage))
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_cache.h"
#include "esp_private/gdma.h"
#include "hal/dma_types.h"
#include "hal/gdma_ll.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc.h"
#include "esp_heap_caps.h"
#include "hal/cpu_utility_ll.h"
#include "xt_utils.h"
#include "esp32s3/rom/ets_sys.h"

/* S3 bank deadlines must not depend on flash-cache misses. */
#define RING_HOT IRAM_ATTR
#define RING_SWITCH IRAM_ATTR
#define RING_MASK (RING_PAIRS - 1u)
#define THRESHOLD RING_THRESHOLD
#define SENTINEL  0xA5C33C5Au
#define START_GUARD 3072u /* sentinels from a unit's earliest possible start */
#define END_GUARD   4096u /* sentinels from a unit's earliest possible end */
#define LATE_LIMIT  2000u /* switch lateness allowed: 2*LATE_LIMIT < END_GUARD */
#define UNIT0_END_GUARD 1024u /* unit 0: whole bank is sentinel, stay clear of its start */
#define MIN_PAIRS       THRESHOLD
#define MAX_PAIRS       (THRESHOLD + LATE_LIMIT + 64u)
#define NOT_FOUND       0xffffffffu

#define DUMP_CTRL_REG        0x60033d5cu
#define DUMP_WRITE_INDEX_REG 0x60033d60u
#define DUMP_CONFIG_REG      0x60033d90u
#define DUMP_BANK_SELECT_REG 0x600c101cu
#define DUMP_CTRL_RUN        0x80000000u
#define DUMP_CTRL_CIRCULAR   0x00024000u /* circular 16384-pair ring, IQ source 0 */
#define DUMP_CONFIG_IQ       0x000c2040u

static inline void select_banks(uint32_t saved, unsigned mask) {
    REG_WRITE(DUMP_BANK_SELECT_REG, (saved & ~15u) | mask);
}

_Static_assert(RING_BANK_BASE + RING_BANKS * RING_BANK_STRIDE == RING_BANK_END, "bank map");
_Static_assert(START_GUARD + THRESHOLD <= RING_PAIRS, "start window overlaps");
_Static_assert(END_GUARD + THRESHOLD <= RING_PAIRS, "end window overlaps unit start");
_Static_assert(2u * LATE_LIMIT + 64u < END_GUARD && LATE_LIMIT + 64u < START_GUARD, "guards");

static inline uint32_t *bank_ptr(unsigned b) {
    return (uint32_t *)(RING_BANK_BASE + b * RING_BANK_STRIDE);
}
static unsigned ring_capture_rate_hz(unsigned rate) {
    return rate == 6 ? 16000000u : rate == 1 ? 40000000u : rate == 0 ? 80000000u : 0u;
}
static uint32_t rate_bits(unsigned rate) {
    return rate == 6 ? (1u << 16) : rate == 1 ? (1u << 15) : 0;
}
static unsigned cycles_per_pair(unsigned rate) { /* 240 MHz CPU */
    return CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000u / ring_capture_rate_hz(rate);
}

/* ---- sentinel windows ------------------------------------------------- */

/* Fills ring indices [at, at+count) of bank b with sentinels, wrapping.
 * 128-bit PIE stores (from eSpDR capture.c, 0BSD): at 80 Msps the scalar
 * loop was too slow to prepare 7 k words before the next switch. */
RING_SWITCH static void fill_sentinels(unsigned b, unsigned at, unsigned count) {
    at &= RING_MASK;
    while (count) {
        unsigned n = RING_PAIRS - at;
        if (n > count) n = count;
        uint32_t *p = bank_ptr(b) + at;
        unsigned head = (-at) & 3u;
        if (head > n) head = n;
        for (unsigned i = 0; i < head; i++) *p++ = SENTINEL;
        unsigned vectors = 0;
        vectors = (n - head) / 4;
        if (vectors) {
            uint32_t value = SENTINEL;
            __asm__ volatile("ee.movi.32.q q6, %[v], 0\n"
                             "ee.movi.32.q q6, %[v], 1\n"
                             "ee.movi.32.q q6, %[v], 2\n"
                             "ee.movi.32.q q6, %[v], 3\n"
                             "loopnez %[n], 1f\n"
                             "ee.vst.128.ip q6, %[p], 16\n"
                             "1:"
                             : [p] "+a"(p)
                             : [v] "a"(value), [n] "a"(vectors)
                             : "memory");
        }
        for (unsigned i = head + vectors * 4; i < n; i++) *p++ = SENTINEL;
        count -= n;
        at = 0;
    }
    __asm__ volatile("memw" ::: "memory");
}

/* The ESP-SDR S3 linear dump uses the top bank and a hardware completion bit. */
ring_status_t ring_capture_burst(unsigned rate, unsigned count, uint32_t **samples) {
    if (!samples || !ring_capture_rate_hz(rate) || !count || count > RING_PAIRS - 4u)
        return RING_FAIL_ARG;
    *samples = NULL;
    const unsigned bank = RING_BANKS - 1;
    const uint32_t saved = REG_READ(DUMP_BANK_SELECT_REG);
    fill_sentinels(bank, 0, count);
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_CONFIG_REG, DUMP_CONFIG_IQ);
    select_banks(saved, 1u << bank);
    const uint32_t ctrl = DUMP_CTRL_RUN | rate_bits(rate) | count;
    const int64_t start = esp_timer_get_time();
    REG_WRITE(DUMP_CTRL_REG, ctrl);
    REG_WRITE(DUMP_CTRL_REG, ctrl | (1u << 19));
    REG_WRITE(DUMP_CTRL_REG, ctrl);
    while (!(REG_READ(DUMP_CTRL_REG) & (1u << 18)) && esp_timer_get_time() - start < 20000) {}
    const bool done = (REG_READ(DUMP_CTRL_REG) & (1u << 18)) != 0;
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_BANK_SELECT_REG, saved);
    if (!done) return RING_FAIL_END;
    uint32_t *p = bank_ptr(bank);
    for (unsigned i = 0; i < count; ++i)
        if (p[i] == SENTINEL) return RING_FAIL_LENGTH;
    *samples = p;
    return RING_OK;
}

/* First written pair in [origin, origin+guard), which holds sentinels up to
 * the unit start and data from there on. */
RING_SWITCH static uint32_t find_first(unsigned b, unsigned origin, unsigned guard) {
    const uint32_t *p = bank_ptr(b);
    if (p[origin & RING_MASK] != SENTINEL) return NOT_FOUND; /* started early */
    if (p[(origin + guard - 1) & RING_MASK] == SENTINEL) return NOT_FOUND;
    unsigned lo = 0, hi = guard - 1;
    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        if (p[(origin + mid) & RING_MASK] == SENTINEL)
            lo = mid + 1;
        else
            hi = mid;
    }
    return (origin + lo) & RING_MASK;
}

/* One past the last written pair in [origin, origin+guard), searching from
 * the write index seen before the switch. */
RING_SWITCH static uint32_t find_end(unsigned b, unsigned origin, unsigned write_index, unsigned guard) {
    const uint32_t *p = bank_ptr(b);
    unsigned lo = (write_index - origin) & RING_MASK;
    if (lo >= guard || p[(origin + guard - 1) & RING_MASK] != SENTINEL) return NOT_FOUND;
    unsigned hi = guard - 1;
    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        if (p[(origin + mid) & RING_MASK] == SENTINEL)
            hi = mid;
        else
            lo = mid + 1;
    }
    return (origin + lo) & RING_MASK;
}

/* ---- spectrum reduction ------------------------------------------------ */
#define CHUNK        128u /* bins per accumulation slice */
#define UNPACK_CHUNK 512u /* samples per unpack slice */

static int16_t window_q15[RING_SPEC_NFFT_MAX];
/* Window with each coefficient twice (I and Q lanes) for the PIE unpack. */
static int16_t win2[2 * RING_SPEC_NFFT_MAX] __attribute__((aligned(16)));
void s3_unpack_iq10_win(const uint32_t *src, int16_t *dst, const int16_t *win2, unsigned groups8);
static int16_t fft_buf[2 * RING_SPEC_NFFT_MAX] __attribute__((aligned(16)));
/* The active union member is selected once per capture. */
static union { float sum; uint32_t peak; } accum[RING_SPEC_NFFT_MAX];
static uint16_t bin_of[RING_SPEC_NFFT_MAX]; /* natural bin -> bit-reversed slot */
static unsigned spec_n, spec_log2; /* FFT size of the current run */

static void c1_start(void);
static bool dsp_ready;
void ring_capture_init(void) {
    if (dsp_ready) return;
    /* twiddles from the heap: BSS must end below the RF ring (sram_guard.ld) */
    fft_twiddles = heap_caps_aligned_alloc(16, RING_SPEC_NFFT_MAX * sizeof(int16_t),
                                          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!fft_twiddles) return;
    /* Same Q15, bit-reversed half-circle as ESP-DSP's sc16 DIF kernel. */
    for (unsigned j = 0; j < RING_SPEC_NFFT_MAX / 2; ++j) {
        unsigned r = 0;
        for (unsigned m = RING_SPEC_NFFT_MAX / 2, b = j; m > 1; m >>= 1, b >>= 1)
            r = (r << 1) | (b & 1u);
        float angle = (float)(2.0 * M_PI / RING_SPEC_NFFT_MAX) * r;
        fft_twiddles[2 * j] = (int16_t)(32767 * cosf(angle));
        fft_twiddles[2 * j + 1] = (int16_t)(32767 * sinf(angle));
    }
    dsp_ready = true;
    if (dsp_ready) c1_start();
}

static bool ring_capture_valid_nfft(unsigned n) {
    return n <= RING_SPEC_NFFT_MAX && (n >= 256 && !(n & (n - 1)));
}

/* Hann window and bit-reversal bin map for FFT size n; outside the timed run.
 * Scaling is independent of n: IQ10 << 6 in, the int16 FFT divides by n. */
static void spec_setup(unsigned n) {
    if (n == spec_n) return;
    unsigned log2n = 0;
    while ((1u << log2n) < n) log2n++;
    for (unsigned i = 0; i < n; i++) {
        window_q15[i] = (int16_t)lrintf(32767.0f * 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / n)));
        win2[2 * i] = win2[2 * i + 1] = window_q15[i];
        unsigned r = 0;
        for (unsigned bit = 0; bit < log2n; bit++) r |= ((i >> bit) & 1u) << (log2n - 1 - bit);
        bin_of[i] = (uint16_t)r;
    }
    spec_n = n;
    spec_log2 = log2n;
}

/* Block pipeline, one slice per call so the ring is polled in between:
 * unpack (UNPACK_CHUNK samples, needs the bank) -> FFT (one stage) ->
 * accumulate (CHUNK bins). Frame closure waits for a complete transform;
 * only unpacking reads the RF bank. */
IRAM_ATTR static void spec_unpack(const uint32_t *p, unsigned at, unsigned from, unsigned to) {
    unsigned first = (at + from) & RING_MASK, count = to - from;
    if (!(from & 7u) && !(count & 7u) && count && first + count + 4 <= RING_PAIRS) {
        /* PIE path (~3 cycles/sample instead of ~17); bit-identical output */
        s3_unpack_iq10_win(p + first, fft_buf + 2 * from, win2 + 2 * from, count / 8u);
        return;
    }
    for (unsigned i = from; i < to; i++) {
        uint32_t w = p[(at + i) & RING_MASK];
        int32_t I = (int32_t)(w << 22) >> 22, Q = (int32_t)(w << 12) >> 22;
        fft_buf[2 * i] = (int16_t)((I * window_q15[i]) >> 9); /* IQ10 << 6 */
        fft_buf[2 * i + 1] = (int16_t)((Q * window_q15[i]) >> 9);
    }
}

/* Updated only by the DSP owner (core 1 also processes assisted blocks). */
static spectrum_dc_t dc;
IRAM_ATTR static void dc_track(int16_t *x) {
    spectrum_dc_apply(&dc, x, spec_n);
}
IRAM_ATTR static void spec_remove_dc(void) {
    dc_track(fft_buf);
}

IRAM_ATTR static void spec_accumulate(bool max_hold, unsigned from, unsigned to) {
    const int16_t *x = fft_buf + 2 * from;
    if (max_hold) {
        for (unsigned k = from; k < to; k++, x += 2) {
            int32_t re = x[0], im = x[1];
            uint32_t power = (uint32_t)(re * re) + (uint32_t)(im * im); /* <= 2^31 */
            if (power > accum[k].peak) accum[k].peak = power;
        }
    } else {
        for (unsigned k = from; k < to; k++, x += 2) {
            int32_t re = x[0], im = x[1];
            accum[k].sum += (float)((uint32_t)(re * re) + (uint32_t)(im * im));
        }
    }
}

/* ---- run ---------------------------------------------------------------- */

typedef struct {
    bool pending;
    uint32_t first, next_block, blocks;
} work_t;

enum { BLK_IDLE, BLK_UNPACK, BLK_FFT, BLK_ACCUM };

static struct {
    const ring_config_t *cfg;
    ring_result_t *res;
    work_t work[RING_BANKS];
    unsigned fifo[RING_BANKS], fifo_len; /* banks with pending work, oldest first */
    /* block in flight */
    unsigned phase, pos, blk_bank, fft_stage;
    uint32_t blk_at;
} st;


/* Retire a bank before RF reuses it. Once unpacking is complete the FFT owns
 * its input copy, so it can finish after retirement. */
RING_HOT static void unit_done(void) {
    unsigned b = st.fifo[0];
    work_t *w = &st.work[b];
    if (w->next_block < w->blocks) {
        st.res->abandoned += w->blocks - w->next_block;
    }
    if (st.phase == BLK_UNPACK && st.blk_bank == b) {
        st.phase = BLK_IDLE;
        st.res->abandoned++;
    }
    w->pending = false;
    for (unsigned i = 1; i < st.fifo_len; i++) st.fifo[i - 1] = st.fifo[i];
    st.fifo_len--;
}

/* Bank b is about to be overwritten: drop whatever work it still holds. */
RING_HOT static void release_bank(unsigned b) {
    while (st.work[b].pending) unit_done(); /* FIFO order: older units retire first */
}

/* One work slice; returns false when there is nothing to do. */
RING_HOT static bool work_slice(void) {
    uint32_t t0 = esp_cpu_get_cycle_count();
    if (st.phase == BLK_FFT) {
        S3_FFT_STAGE(fft_buf, spec_n, st.fft_stage);
        if (++st.fft_stage == spec_log2) {
            spec_remove_dc();
            st.phase = BLK_ACCUM;
            st.pos = 0;
        }
    } else if (st.phase == BLK_ACCUM) {
        /* Commit the FFT count only once its full power vector is added. */
        unsigned end = st.pos + CHUNK < spec_n ? st.pos + CHUNK : spec_n;
        spec_accumulate(st.cfg->max_hold, st.pos, end);
        st.pos = end;
        if (end == spec_n) {
            st.phase = BLK_IDLE;
            st.res->ffts++;
        }
    } else if (st.phase == BLK_UNPACK) {
        unsigned end = st.pos + UNPACK_CHUNK < spec_n ? st.pos + UNPACK_CHUNK : spec_n;
        spec_unpack(bank_ptr(st.blk_bank), st.blk_at, st.pos, end);
        st.pos = end;
        if (end == spec_n) {
            st.phase = BLK_FFT;
            st.fft_stage = 0;
        }
    } else {
        if (!st.fifo_len) return false;
        unsigned b = st.fifo[0];
        work_t *w = &st.work[b];
        if (w->next_block >= w->blocks) {
            unit_done();
        } else {
            st.blk_bank = b;
            st.blk_at = w->first + (w->next_block << spec_log2);
            w->next_block++;
            st.phase = BLK_UNPACK;
            st.pos = 0;
        }
    }
    uint32_t dt = esp_cpu_get_cycle_count() - t0;
    if (dt > st.res->work_max) st.res->work_max = dt;
    return true;
}

/* ---- core 1 SPEC worker ------------------------------------------------- */
/* Core 0 owns the RF ring; core 1 owns the spectrum accumulator. Core 0
 * also assists with FFTs when there is time before the next bank switch.
 * Handshake through internal SRAM (no data cache):
 *   - core 0 posts each located unit with a sequence number and marks its
 *     bank valid (bank_seq[b] = seq); core 1 clears it when done with it;
 *   - before refilling a bank with sentinels, core 0 revokes it
 *     (bank_seq[b] = 0) and waits while core 1 unpacks from it (busy == b+1);
 *   - core 1 sets busy, then re-checks bank_seq before every unpack (Dekker,
 *     memw on both sides), so it never reads a bank that is being refilled.
 * Core 1 runs only IRAM code on DRAM data, interrupts masked, no flash. */
#define C1_QUEUE 8u
#define MEMW()   __asm__ volatile("memw" ::: "memory")
typedef struct {
    uint32_t seq, bank, first, blocks;
} c1_unit_t;
static volatile struct {
    uint32_t alive, run, posted, taken, end, done, busy, block_max;
    uint32_t bank_seq[RING_BANKS];
} c1;
static c1_unit_t c1q[C1_QUEUE];
static bool c1_ok;
extern void s3_core1_entry(void);

bool ring_capture_core1_alive(void) { return c1_ok; }

/* PIE loads one extra vector; keep that read inside CPU-owned RF SRAM.
 * Wrap and bank-end groups use scalar copies, preserving destination alignment. */
IRAM_ATTR static void unpack_seg(int16_t *dst, const uint32_t *p, unsigned at, unsigned from, unsigned to) {
    while (from < to) {
        unsigned first = (at + from) & RING_MASK;
        unsigned available = first + 4 < RING_PAIRS ? RING_PAIRS - first - 4 : 0;
        unsigned count = to - from < available ? to - from : available;
        unsigned bulk = count & ~7u;
        if (bulk) {
            s3_unpack_iq10_win(p + first, dst + 2 * from, win2 + 2 * from, bulk / 8);
            from += bulk;
        } else {
            unsigned end = from + 8 < to ? from + 8 : to;
            for (; from < end; from++) {
                uint32_t w = p[(at + from) & RING_MASK];
                int32_t i = (int32_t)(w << 22) >> 22, q = (int32_t)(w << 12) >> 22;
                dst[2 * from] = (int16_t)((i * window_q15[from]) >> 9);
                dst[2 * from + 1] = (int16_t)((q * window_q15[from]) >> 9);
            }
        }
    }
}

/* Buffer-parameter versions of the block stages (core 0 hand-off path). */
IRAM_ATTR static void unpack_to(int16_t *dst, const uint32_t *p, unsigned at) {
    unpack_seg(dst, p, at, 0, spec_n);
}
IRAM_ATTR static void accumulate_buf(const int16_t *x, bool max_hold) {
    if (max_hold) {
        for (unsigned k = 0; k < spec_n; k++, x += 2) {
            int32_t re = x[0], im = x[1];
            uint32_t power = (uint32_t)(re * re) + (uint32_t)(im * im);
            if (power > accum[k].peak) accum[k].peak = power;
        }
    } else {
        for (unsigned k = 0; k < spec_n; k++, x += 2) {
            int32_t re = x[0], im = x[1];
            accum[k].sum += (float)((uint32_t)(re * re) + (uint32_t)(im * im));
        }
    }
}

/* Shared block cursor of the unit core 1 is working on, and the core-0
 * hand-off buffer (one FFT output waiting for core 1's accumulator). */
static volatile struct {
    uint32_t seq; /* unit open for claims, 0 = none */
    uint32_t bank, first, blocks;
    uint32_t claim;   /* next block index (atomic) */
    uint32_t c0_seq;  /* core 0 is between claim and hand-off of this unit */
    uint32_t hb_full; /* hand-off buffer holds an FFT for core 1 */
    uint32_t hb_seq;
    uint32_t done;      /* blocks accumulated for the open unit */
} cl;
static int16_t *hbuf; /* internal heap, aligned for the SIMD FFT */

IRAM_ATTR static bool claim_block(uint32_t *out) {
    for (;;) {
        uint32_t b = cl.claim;
        if (b >= cl.blocks) return false;
        if (xt_utils_compare_and_set(&cl.claim, b, b + 1u)) {
            *out = b;
            return true;
        }
    }
}

IRAM_ATTR static void c1_take_handoff(void) {
    dc_track(hbuf);
    accumulate_buf(hbuf, st.cfg->max_hold);
    st.res->ffts++;
    cl.done++;
    MEMW();
    cl.hb_full = 0;
}

IRAM_ATTR static void c1_unit(const c1_unit_t *u) {
    const unsigned total = u->blocks;
    cl.bank = u->bank;
    cl.first = u->first;
    cl.blocks = u->blocks;
    cl.claim = 0;
    cl.done = 0;
    MEMW();
    cl.seq = u->seq; /* open for claims (core 0 too) */
    MEMW();
    bool revoked = false;
    for (;;) {
        if (cl.hb_full) c1_take_handoff();
        uint32_t blk;
        if (revoked || !claim_block(&blk)) break;
        uint32_t t0 = esp_cpu_get_cycle_count();
        const uint32_t *bp = bank_ptr(u->bank);
        unsigned at = u->first + (blk << spec_log2);
        for (unsigned s0 = 0; s0 < spec_n && !revoked; s0 += 256u) {
            c1.busy = u->bank + 1u;
            MEMW();
            if (c1.bank_seq[u->bank] != u->seq)
                revoked = true; /* core 0 needs the bank */
            else
                unpack_seg(fft_buf, bp, at, s0, s0 + 256u);
            MEMW();
            c1.busy = 0;
        }
        if (revoked) {
            cl.claim = cl.blocks; /* stop further claims; the partial block is dropped */
            continue;
        }
        S3_FFT(fft_buf, spec_n);
        spec_remove_dc();
        spec_accumulate(st.cfg->max_hold, 0, spec_n);
        st.res->ffts++;
        cl.done++;
        uint32_t dt = esp_cpu_get_cycle_count() - t0;
        if (dt > c1.block_max) c1.block_max = dt;
    }
    /* Close: no more claims; collect core 0's block still in flight. */
    cl.claim = cl.blocks;
    MEMW();
    while (cl.c0_seq == u->seq || (cl.hb_full && cl.hb_seq == u->seq)) {
        if (cl.hb_full) c1_take_handoff();
    }
    cl.seq = 0;
    MEMW();
    if (cl.done < total) st.res->abandoned += total - cl.done;
    if (c1.bank_seq[u->bank] == u->seq) c1.bank_seq[u->bank] = 0; /* bank free */
    MEMW();
}

/* Core 0: process one block of core 1's open unit into the hand-off buffer.
 * Called from the poll loop only when it fits before the next ring deadline. */
IRAM_ATTR static void c0_assist(void) {
    if (cl.hb_full) return;
    uint32_t seq = cl.seq;
    if (!seq) return;
    cl.c0_seq = seq;
    MEMW();
    uint32_t blk;
    if (cl.seq != seq || !claim_block(&blk)) {
        cl.c0_seq = 0;
        return;
    }
    unsigned bank = cl.bank;
    if (c1.bank_seq[bank] != seq) { /* revoked meanwhile (core 0 itself) */
        cl.c0_seq = 0;
        return;
    }
    unpack_to(hbuf, bank_ptr(bank), cl.first + (blk << spec_log2));
    S3_FFT(hbuf, spec_n);
    cl.hb_seq = seq;
    MEMW();
    cl.hb_full = 1;
    MEMW();
    cl.c0_seq = 0;
}

IRAM_ATTR void s3_core1_main(void) {
    uint32_t run_seen = c1.run;
    MEMW();
    c1.alive = 1;
    MEMW();
    for (;;) {
        while (c1.run == run_seen) {}
        run_seen = c1.run;
        MEMW();
        uint32_t taken = 0;
        for (;;) {
            MEMW();
            if (taken == c1.posted) {
                if (!c1.end) continue;
                MEMW();
                if (taken == c1.posted) break; /* end is written after the last post */
                continue;
            }
            c1_unit_t u = c1q[taken % C1_QUEUE];
            c1_unit(&u);
            taken++;
            c1.taken = taken;
        }
        MEMW();
        c1.done = 1;
    }
}

static inline void c1_revoke(unsigned b) {
    c1.bank_seq[b] = 0;
    MEMW();
    while (c1.busy == b + 1u) {}
}

static void c1_start(void) {
    hbuf = heap_caps_aligned_alloc(16, 2 * RING_SPEC_NFFT_MAX * sizeof(int16_t),
                                   MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!hbuf) return; /* no second core: single-core SPEC */
    cpu_utility_ll_unstall_cpu(1);
    cpu_utility_ll_enable_clock_and_reset_app_cpu();
    ets_set_appcpu_boot_addr((uint32_t)s3_core1_entry);
    int64_t t0 = esp_timer_get_time();
    while (!c1.alive && esp_timer_get_time() - t0 < 50000) {}
    c1_ok = c1.alive != 0;
}

RING_HOT static void fail(ring_result_t *r, ring_status_t code, uint32_t detail) {
    if (!r->status) {
        r->status = code;
        r->detail = detail;
    }
}

/* Reclaiming a bank cancels work that has not copied its input yet. */
static inline void reclaim_bank(bool dual, unsigned b) {
    if (dual)
        c1_revoke(b);
    else
        release_bank(b);
}
static inline bool bank_idle(bool dual, unsigned b) {
    return dual ? c1.bank_seq[b] == 0 : !st.work[b].pending;
}

/* Offload aligned bank copies to GDMA. Save the partial cache lines in
 * internal SRAM so CPU writes can never overlap a GDMA cache line. */
#define RECORD_ALIGN_PAIRS (RING_RECORD_ALIGN / sizeof(uint32_t))
#define RECORD_DESC_SIZE DMA_DESCRIPTOR_BUFFER_MAX_SIZE_64B_ALIGNED
#define RECORD_DESC_COUNT (MAX_PAIRS * sizeof(uint32_t) / RECORD_DESC_SIZE + 3u)
#define RECORD_EDGE_COUNT (RING_RECORD_MAX_PAIRS / MIN_PAIRS + 2u)
static dma_descriptor_t record_rx[RECORD_DESC_COUNT], record_tx[RECORD_DESC_COUNT];
static struct {
    uint32_t *dest;
    unsigned head, tail, count;
    uint32_t partial[2 * (RECORD_ALIGN_PAIRS - 1u)];
} record_edges[RECORD_EDGE_COUNT];
static unsigned record_edge_count;
static int dma_rx_id = -1, dma_tx_id = -1;

typedef struct {
    uint32_t *dest;
    unsigned first, remaining;
} record_work_t;

static bool record_dma_init(void) {
    if (dma_rx_id >= 0) return true;
    gdma_channel_handle_t tx = NULL, rx = NULL;
    const gdma_channel_alloc_config_t tx_cfg = {
        .direction = GDMA_CHANNEL_DIRECTION_TX, .flags.reserve_sibling = true,
    };
    if (gdma_new_ahb_channel(&tx_cfg, &tx) != ESP_OK) return false;
    const gdma_channel_alloc_config_t rx_cfg = {.direction = GDMA_CHANNEL_DIRECTION_RX, .sibling_chan = tx};
    uint32_t free_triggers = 0;
    if (gdma_new_ahb_channel(&rx_cfg, &rx) == ESP_OK &&
        gdma_get_free_m2m_trig_id_mask(tx, &free_triggers) == ESP_OK && free_triggers) {
        gdma_trigger_t trigger = GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_M2M, 0);
        trigger.instance_id = __builtin_ctz(free_triggers);
        const gdma_strategy_config_t strategy = {.owner_check = true, .eof_till_data_popped = true};
        const gdma_transfer_config_t transfer = {.max_data_burst_size = 64, .access_ext_mem = true};
        int rx_id, tx_id;
        if (gdma_connect(rx, trigger) == ESP_OK && gdma_connect(tx, trigger) == ESP_OK &&
            gdma_apply_strategy(rx, &strategy) == ESP_OK && gdma_apply_strategy(tx, &strategy) == ESP_OK &&
            gdma_config_transfer(rx, &transfer) == ESP_OK && gdma_config_transfer(tx, &transfer) == ESP_OK &&
            gdma_get_channel_id(rx, &rx_id) == ESP_OK && gdma_get_channel_id(tx, &tx_id) == ESP_OK) {
            dma_rx_id = rx_id;
            dma_tx_id = tx_id;
            return true;
        }
    }
    if (rx) {
        gdma_disconnect(rx);
        gdma_del_channel(rx);
    }
    gdma_disconnect(tx);
    gdma_del_channel(tx);
    return false;
}

static inline void record_dma_reset(void) {
    gdma_ll_rx_reset_channel(&GDMA, dma_rx_id);
    gdma_ll_tx_reset_channel(&GDMA, dma_tx_id);
    gdma_ll_rx_clear_interrupt_status(&GDMA, dma_rx_id, GDMA_LL_RX_EVENT_MASK);
    gdma_ll_tx_clear_interrupt_status(&GDMA, dma_tx_id, GDMA_LL_TX_EVENT_MASK);
}

RING_HOT static bool record_start(unsigned bank, const record_work_t *work) {
    const uint32_t *src = bank_ptr(bank);
    unsigned head = (-work->first) & (RECORD_ALIGN_PAIRS - 1u);
    if (head > work->remaining) head = work->remaining;
    const unsigned tail = (work->remaining - head) & (RECORD_ALIGN_PAIRS - 1u);
    const unsigned edge = record_edge_count++;
    record_edges[edge].dest = work->dest;
    record_edges[edge].count = work->remaining;
    record_edges[edge].head = head;
    record_edges[edge].tail = tail;
    for (unsigned i = 0; i < head; ++i)
        record_edges[edge].partial[i] = src[(work->first + i) & RING_MASK];
    for (unsigned i = 0; i < tail; ++i)
        record_edges[edge].partial[head + i] = src[(work->first + work->remaining - tail + i) & RING_MASK];

    unsigned left = work->remaining - head - tail;
    unsigned at = (work->first + head) & RING_MASK;
    uint32_t *dest = work->dest + head;
    unsigned desc = 0;
    while (left) {
        unsigned n = RING_PAIRS - at;
        if (n > left) n = left;
        if (n > RECORD_DESC_SIZE / sizeof(uint32_t)) n = RECORD_DESC_SIZE / sizeof(uint32_t);
        left -= n;
        const unsigned bytes = n * sizeof(uint32_t);
        record_tx[desc] = (dma_descriptor_t){
            .dw0 = {.size = bytes, .length = bytes, .suc_eof = !left, .owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA},
            .buffer = (void *)(src + at), .next = left ? &record_tx[desc + 1u] : NULL,
        };
        record_rx[desc] = (dma_descriptor_t){
            .dw0 = {.size = bytes, .owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA},
            .buffer = dest, .next = left ? &record_rx[desc + 1u] : NULL,
        };
        ++desc;
        at = (at + n) & RING_MASK;
        dest += n;
    }
    if (!desc) return false;
    record_dma_reset();
    MEMW();
    gdma_ll_rx_set_desc_addr(&GDMA, dma_rx_id, (uintptr_t)record_rx);
    gdma_ll_tx_set_desc_addr(&GDMA, dma_tx_id, (uintptr_t)record_tx);
    gdma_ll_rx_start(&GDMA, dma_rx_id);
    gdma_ll_tx_start(&GDMA, dma_tx_id);
    return true;
}

RING_HOT static void record_poll(record_work_t work[RING_BANKS], unsigned *bank, bool *active, ring_result_t *r) {
    if (*active) {
        const uint32_t rx = gdma_ll_rx_get_interrupt_status(&GDMA, dma_rx_id, true);
        const uint32_t tx = gdma_ll_tx_get_interrupt_status(&GDMA, dma_tx_id, true);
        if ((rx & (GDMA_LL_EVENT_RX_DESC_ERROR | GDMA_LL_EVENT_RX_ERR_EOF)) ||
            (tx & GDMA_LL_EVENT_TX_DESC_ERROR)) {
            fail(r, RING_FAIL_COPY, (tx << 16) | rx);
            return;
        }
        if (!(rx & GDMA_LL_EVENT_RX_SUC_EOF)) return;
        *active = false;
        work[*bank].remaining = 0;
        *bank = (*bank + 1u) % RING_BANKS;
    }
    if (!work[*bank].remaining) return;
    if (record_edge_count >= RECORD_EDGE_COUNT) {
        fail(r, RING_FAIL_COPY, record_edge_count);
        return;
    }
    *active = record_start(*bank, &work[*bank]);
    if (!*active) {
        work[*bank].remaining = 0;
        *bank = (*bank + 1u) % RING_BANKS;
    }
}

RING_HOT void ring_capture_run(const ring_config_t *cfg, ring_result_t *r) {
    memset(r, 0, sizeof(*r));
    memset(&st, 0, sizeof(st));
    st.cfg = cfg;
    st.res = r;
    const bool record = cfg->record != NULL;
    if (record ? (cfg->rate != 6 || !cfg->record_pairs || cfg->record_pairs > RING_RECORD_MAX_PAIRS ||
                  ((uintptr_t)cfg->record & (RING_RECORD_ALIGN - 1u)))
               : (!dsp_ready || !cfg->local_power || !cfg->duration_ms || cfg->duration_ms > 100u ||
                  !ring_capture_rate_hz(cfg->rate) || !ring_capture_valid_nfft(cfg->nfft))) {
        fail(r, RING_FAIL_ARG, 0);
        return;
    }
    if (record) {
        if (!record_dma_init() ||
            esp_cache_msync(cfg->record, RING_RECORD_BYTES(cfg->record_pairs),
                            ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_INVALIDATE) != ESP_OK) {
            fail(r, RING_FAIL_COPY, 0);
            return;
        }
        record_edge_count = 0;
    } else {
        spec_setup(cfg->nfft);
    }

    const unsigned cpp = cycles_per_pair(cfg->rate);
    const uint32_t max_age = (RING_PAIRS - 128u) * cpp;
    const uint32_t settle = 16u * cpp;
    const uint32_t ctrl = DUMP_CTRL_CIRCULAR | rate_bits(cfg->rate);
    const int64_t duration_us = (int64_t)cfg->duration_ms * 1000;
    const uint32_t bank_sel_saved = REG_READ(DUMP_BANK_SELECT_REG);
    uint32_t start_probe[RING_BANKS] = {0}, end_probe[RING_BANKS] = {0};

    for (unsigned b = 0; b < RING_BANKS; b++) fill_sentinels(b, 0, RING_PAIRS);

    uint32_t assist_cost = 0;
    if (!record) { /* Warm caches/tables before starting RF; bank 2 holds sentinels only. */
        uint32_t assist_start = esp_cpu_get_cycle_count();
        spec_unpack(bank_ptr(2), 0, 0, spec_n);
        uint32_t longest = 0;
        for (unsigned stage = 0; stage < spec_log2; stage++) {
            uint32_t start = esp_cpu_get_cycle_count();
            S3_FFT_STAGE(fft_buf, spec_n, stage);
            uint32_t elapsed = esp_cpu_get_cycle_count() - start;
            if (elapsed > longest) longest = elapsed;
        }
        assist_cost = esp_cpu_get_cycle_count() - assist_start;
        uint32_t start = esp_cpu_get_cycle_count();
        spec_accumulate(cfg->max_hold, 0, CHUNK);
        uint32_t elapsed = esp_cpu_get_cycle_count() - start;
        if (elapsed > longest) longest = elapsed;
        memset(accum, 0, sizeof(accum));
        /* Seed the slice budget with the longest processing stage of this size, so the
         * first slices are gated correctly (+15 % for cache/bus variation). */
        r->work_max = longest + longest / 7u;
    }
    const bool dual = !record && c1_ok;
    record_work_t copies[RING_BANKS] = {0};
    unsigned copy_bank = 0;
    bool copy_active = false;
    uint32_t c1_seq = 0, c0_lost = 0;
    uint32_t c0_cost = assist_cost + assist_cost / 3u; /* warm budget + margin */
    if (dual) {                                        /* hand the run to core 1 (idle since its last run) */
        c1.posted = c1.taken = c1.end = c1.done = c1.busy = c1.block_max = 0;
        cl.seq = cl.c0_seq = cl.hb_full = 0;
        for (unsigned i = 0; i < RING_BANKS; i++) c1.bank_seq[i] = 0;
        MEMW();
        c1.run = c1.run + 1u;
    }
    dc = (spectrum_dc_t){0};
    unsigned irq = portSET_INTERRUPT_MASK_FROM_ISR();
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_CONFIG_REG, DUMP_CONFIG_IQ);
    REG_WRITE(DUMP_CTRL_REG, ctrl);
    select_banks(bank_sel_saved, 1u);
    int64_t t_start = esp_timer_get_time();
    REG_WRITE(DUMP_CTRL_REG, ctrl | DUMP_CTRL_RUN);
    uint32_t epoch = esp_cpu_get_cycle_count();
    const uint32_t w0 = REG_READ(DUMP_WRITE_INDEX_REG) & RING_MASK;

    /* Unit 0 starts near w0; its exact start is found after the switch. */
    uint32_t expected = (w0 - 256u) & RING_MASK; /* poll reference for unit 0 */
    uint64_t index = 0;
    unsigned b = 0;
    bool prepared = false;

    for (unsigned unit = 0;; unit++) {
        const unsigned next = (b + 1) % RING_BANKS;

        /* 1. Poll, doing short FFT work slices between polls. The stop
         *    decision is made mid-unit so nothing but the index read sits
         *    between the threshold and the bank write. */
        uint32_t write_index, written;
        bool stop = false, stop_checked = false;
        for (;;) {
            write_index = REG_READ(DUMP_WRITE_INDEX_REG) & RING_MASK;
            written = (write_index - expected) & RING_MASK;
            uint32_t age = esp_cpu_get_cycle_count() - epoch;
            if (age > max_age) {
                fail(r, RING_FAIL_AGE, age);
                break;
            }
            if (written >= THRESHOLD) {
                if (!stop && !prepared) { /* a long slice ran past the deadline */
                    if (record && copies[next].remaining) {
                        fail(r, RING_FAIL_COPY, copies[next].remaining);
                        break;
                    }
                    if (!record) reclaim_bank(dual, next);
                    start_probe[next] = (expected + THRESHOLD) & RING_MASK;
                    end_probe[next] = (start_probe[next] + THRESHOLD) & RING_MASK;
                    fill_sentinels(next, start_probe[next], START_GUARD);
                    fill_sentinels(next, end_probe[next], END_GUARD);
                    prepared = true;
                    write_index = REG_READ(DUMP_WRITE_INDEX_REG) & RING_MASK;
                    written = (write_index - expected) & RING_MASK;
                }
                break;
            }
            if (!stop_checked && written >= THRESHOLD / 2) {
                stop_checked = true;
                stop = record ? index + MIN_PAIRS - (unit ? 0u : 1024u) >= cfg->record_pairs
                              : esp_timer_get_time() - t_start >= duration_us;
                continue;
            }
            /* Prepare the next bank as soon as its old unit is retired, and
             * unconditionally with one late-limit of margin left. */
            /* The forced path may also retire an unfinished unit. */
            const unsigned prep_cycles = !dual && r->work_max > 20000u ? r->work_max : 20000u;
            uint32_t prep_pairs = prep_cycles / cpp + 1024u;
            if (!stop && !prepared &&
                ((record ? !copies[next].remaining : bank_idle(dual, next)) ||
                 (!record && written + prep_pairs + LATE_LIMIT >= THRESHOLD))) {
                if (!record) reclaim_bank(dual, next);
                start_probe[next] = (expected + THRESHOLD) & RING_MASK;
                end_probe[next] = (start_probe[next] + THRESHOLD) & RING_MASK;
                fill_sentinels(next, start_probe[next], START_GUARD);
                fill_sentinels(next, end_probe[next], END_GUARD);
                prepared = true;
                continue;
            }
            /* Start a slice only if the longest one seen so far still fits
             * before the switch; at 40/80 Msps most blocks are skipped. */
            if (dual) {
                /* one core-0 block (unpack + FFT) must end before the bank
                 * preparation deadline, or before the switch if prepared */
                uint32_t c0_pairs = c0_cost / cpp + 256u;
                uint32_t limit =
                    prepared || stop ? THRESHOLD + LATE_LIMIT / 2 : THRESHOLD - prep_pairs - LATE_LIMIT;
                if (written + c0_pairs < limit) {
                    uint32_t t0 = esp_cpu_get_cycle_count();
                    c0_assist();
                    uint32_t dt = esp_cpu_get_cycle_count() - t0;
                    if (dt > c0_cost) c0_cost = dt;
                }
            }
            if (record) {
                if (written + 1024u < THRESHOLD) record_poll(copies, &copy_bank, &copy_active, r);
                if (r->status) break;
            } else if (!dual) {
                const unsigned minimum_slice = 8000;
                uint32_t slice_cycles = r->work_max > minimum_slice ? r->work_max : minimum_slice;
                uint32_t slice_pairs = slice_cycles / cpp + 128u;
                if (written + slice_pairs + 1024u < THRESHOLD + LATE_LIMIT / 2) {
                    (void)work_slice();
                }
            }
        }
        if (r->status) break;

        uint32_t late = written - THRESHOLD;
        if (late > r->late_max) r->late_max = late;
        if (late > LATE_LIMIT) {
            fail(r, RING_FAIL_LATE, written);
            break;
        }

        /* 2. Stop or switch. */
        const bool last = stop;
        if (!last && !prepared) {
            fail(r, RING_FAIL_ARG, 1);
            break;
        } /* cannot happen */
        if (last) {
            REG_WRITE(DUMP_CTRL_REG, ctrl);
            select_banks(bank_sel_saved, 0u);
        } else {
            select_banks(bank_sel_saved, 1u << next);
            epoch = esp_cpu_get_cycle_count();
        }
        uint32_t t = esp_cpu_get_cycle_count();
        while (esp_cpu_get_cycle_count() - t < settle) {}

        /* 3. Locate the finished unit exactly. */
        uint32_t first, end;
        if (unit == 0) {
            first = find_first(b, (w0 - 1024u) & RING_MASK, 3072u);
            end = find_end(b, write_index, write_index, UNIT0_END_GUARD);
        } else {
            first = find_first(b, start_probe[b], START_GUARD);
            end = find_end(b, end_probe[b], write_index, END_GUARD);
        }
        if (first == NOT_FOUND || (unit && first != expected)) {
            fail(r, RING_FAIL_START, (first & 0xffffu) | (expected << 16));
            break;
        }
        if (end == NOT_FOUND) {
            fail(r, RING_FAIL_END, write_index);
            break;
        }
        uint32_t count = (end - first) & RING_MASK;
        if (count < (unit ? MIN_PAIRS : MIN_PAIRS - 1024u) || count > MAX_PAIRS) {
            fail(r, RING_FAIL_LENGTH, count);
            break;
        }

        if (record) {
            if (unit == 0) r->record = cfg->record + (first & (RECORD_ALIGN_PAIRS - 1u));
            if (index < cfg->record_pairs) {
                unsigned remaining = cfg->record_pairs - (unsigned)index;
                copies[b] = (record_work_t){r->record + index, first, count < remaining ? count : remaining};
            }
        } else if (dual) {
            uint32_t i = c1.posted;
            if (i - c1.taken >= C1_QUEUE) { /* core 1 far behind: skip this unit */
                c0_lost += count >> spec_log2;
            } else {
                c1q[i % C1_QUEUE] = (c1_unit_t){.seq = ++c1_seq,
                                                .bank = b,
                                                .first = first,
                                                .blocks = count >> spec_log2};
                c1.bank_seq[b] = c1_seq;
                MEMW();
                c1.posted = i + 1u;
            }
        } else {
            work_t *w = &st.work[b];
            *w = (work_t){.pending = true,
                          .first = first,
                          .blocks = count >> spec_log2};
            st.fifo[st.fifo_len++] = b;
        }
        r->units++;
        index += count;
        expected = end;
        b = next;
        prepared = false;
        if (last) break;
    }

    /* Writer stopped (normally or by failure). Finish queued work. */
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_BANK_SELECT_REG, bank_sel_saved);
    r->elapsed_us = (uint64_t)(esp_timer_get_time() - t_start);
    r->pairs = index;
    if (record && index < cfg->record_pairs) fail(r, RING_FAIL_LENGTH, (uint32_t)index);
    if (dual) {
        MEMW();
        c1.end = 1;
        MEMW();
        while (!c1.done) {}
        MEMW();
        r->abandoned += c0_lost;
        r->work_max = c1.block_max; /* longest core-1 block (unpack..accumulate) */
    } else if (!record) {
        while (work_slice()) {}
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
    if (record) {
        const int64_t deadline = esp_timer_get_time() + 5000;
        while (!r->status && copies[copy_bank].remaining && esp_timer_get_time() < deadline)
            record_poll(copies, &copy_bank, &copy_active, r);
        if (copies[copy_bank].remaining) fail(r, RING_FAIL_COPY, copies[copy_bank].remaining);
        record_dma_reset();
        if (!r->status && esp_cache_msync(cfg->record, RING_RECORD_BYTES(cfg->record_pairs),
                                          ESP_CACHE_MSYNC_FLAG_DIR_M2C) != ESP_OK)
            fail(r, RING_FAIL_COPY, 0);
        if (!r->status) {
            for (unsigned i = 0; i < record_edge_count; ++i) {
                uint32_t *dest = record_edges[i].dest;
                const unsigned head = record_edges[i].head, tail = record_edges[i].tail;
                memcpy(dest, record_edges[i].partial, head * sizeof(*dest));
                memcpy(dest + record_edges[i].count - tail, record_edges[i].partial + head, tail * sizeof(*dest));
            }
        }
    }

    /* RF SRAM is back with the CPU; the DSP worker has published its result. */
    if (!r->status && r->ffts) {
        float inv = 1.0f / r->ffts;
        for (unsigned k = 0; k < spec_n; ++k) {
            unsigned slot = bin_of[k];
            cfg->local_power[k] = cfg->max_hold ? (float)accum[slot].peak : accum[slot].sum * inv;
        }
    }
}
