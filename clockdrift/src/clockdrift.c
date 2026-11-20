
/* clockdrift: sample-clock drift compensation.

   Two free-running audio clocks (capture/consumer) do not tick at exactly the
   same rate. Over a long session the cumulative frame counts diverge, so the
   consumer occasionally underruns or repeats a frame — the waveform "blips".

   This module keeps cumulative counts of frames consumed at the input and
   frames produced at the output. Their difference is the drift, in frames.
   When it reaches one frame the drift is corrected inside the current block by
   dropping an input frame (input clock runs fast) or repeating the previous
   frame (input clock runs slow), spread evenly across the block. Corrections
   are exact: the residual drift is always strictly below one frame.

   If the drift ever reaches a configurable limit the module reports a fault
   instead of correcting without bound; the fault latches until dr_reset().
   With correction disabled the output is copied through unchanged.

   All buffers are interleaved stereo frames. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

typedef struct {
    int enabled;        /* 0 = pure passthrough, no frames inserted/dropped */
    int64_t drift_limit;/* fault once |drift| reaches this many frames, must be > 0 */
} DriftCfg;

typedef struct {
    int64_t in_total;       /* cumulative frames consumed from the input side */
    int64_t out_total;      /* cumulative frames handed to the output side */
    double drift;           /* residual drift in frames, always |drift| < 1 */
    int64_t dropped;        /* total frames dropped (input fast) */
    int64_t inserted;       /* total frames inserted/repeated (input slow) */
    int fault;              /* latched: drift hit drift_limit */
} DriftStats;

typedef struct {
    DriftCfg cfg;
    DriftStats st;
} DriftComp;

static int dr_init(DriftComp* dc, DriftCfg cfg) {
    if (cfg.drift_limit <= 0) return -1;
    memset(dc, 0, sizeof(*dc));
    dc->cfg = cfg;
    return 0;
}

static void dr_reset(DriftComp* dc) {
    memset(&dc->st, 0, sizeof(dc->st));
}

static void dr_get_stats(const DriftComp* dc, DriftStats* out) {
    *out = dc->st;
}

/* Produce out_frames stereo frames from in_frames input frames.

   The frame-count difference is the raw drift added this block. Drops and
   inserts are mutually exclusive in one block and are spread evenly over it.
   Returns 0 on success, -1 when the drift limit has been reached: the fault
   latches and out is left untouched until dr_reset(). */
static int dr_process(DriftComp* dc, const float* in, int64_t in_frames,
                      float* out, int64_t out_frames) {
    if (dc->st.fault) return -1;

    double bal = dc->st.drift + (double)(out_frames - in_frames);
    if (fabs(bal) >= (double)dc->cfg.drift_limit) {
        dc->st.fault = 1;
        return -1;
    }

    int64_t drops = 0, inserts = 0;
    if (dc->cfg.enabled) {
        /* bal > 0: output needs more frames than the input supplied -> insert;
           bal < 0: excess input frames -> drop. */
        if (bal >= 1.0) {
            inserts = (int64_t)bal;
            bal -= (double)inserts;
        } else if (bal <= -1.0) {
            drops = (int64_t)-bal;
            bal += (double)drops;
        }
    }

    /* Spacing accumulators start half a step late so the first correction
       lands mid-block instead of clustering at the start. */
    const double d_step = drops ? (double)in_frames / (double)drops : 0.0;
    const double i_step = inserts ? (double)out_frames / (double)inserts : 0.0;
    double d_acc = d_step / 2.0, i_acc = i_step / 2.0;
    int64_t drops_left = drops, ins_left = inserts;

    int64_t i = 0, o = 0, have_prev = 0;
    float prev_l = 0.0f, prev_r = 0.0f;
    while (o < out_frames) {
        if (drops_left > 0 && i < in_frames) {
            d_acc -= 1.0;
            if (d_acc <= 0.0) {
                d_acc += d_step;
                drops_left--;
                i++;                    /* skip this input frame */
                continue;
            }
        }
        int insert_due = 0;
        if (ins_left > 0) {
            i_acc -= 1.0;
            if (i_acc <= 0.0) { i_acc += i_step; insert_due = 1; }
        }
        if (insert_due) {
            ins_left--;
            if (have_prev) { out[o * 2] = prev_l; out[o * 2 + 1] = prev_r; }
            else { out[o * 2] = 0.0f; out[o * 2 + 1] = 0.0f; }
            o++;
            continue;
        }
        if (i < in_frames) {
            out[o * 2] = in[i * 2];
            out[o * 2 + 1] = in[i * 2 + 1];
            prev_l = in[i * 2];
            prev_r = in[i * 2 + 1];
            have_prev = 1;
            i++;
            o++;
        } else {
            /* input exhausted before the next scheduled insert: fill now */
            if (ins_left > 0) ins_left--;
            if (have_prev) { out[o * 2] = prev_l; out[o * 2 + 1] = prev_r; }
            else { out[o * 2] = 0.0f; out[o * 2 + 1] = 0.0f; }
            o++;
        }
    }

    dc->st.in_total += in_frames;
    dc->st.out_total += out_frames;
    /* count what actually fired; a zero-length block cannot fire anything */
    dc->st.dropped += drops - drops_left;
    dc->st.inserted += inserts - ins_left;
    dc->st.drift = bal;
    return 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

#define BLOCK 128

int main(void) {
    DriftComp dc;
    DriftStats s;
    float in[(BLOCK + 2) * 2], out[(BLOCK + 2) * 2];
    for (int i = 0; i < (BLOCK + 2) * 2; i++) in[i] = (float)i * 0.001f;

    /* 1. configuration */
    check("init_rejects_bad_limit", dr_init(&dc, (DriftCfg){ 1, 0 }) == -1);
    check("init_ok", dr_init(&dc, (DriftCfg){ 0, 64 }) == 0);

    /* 2. disabled: output bit-identical to input, never corrected */
    check("disabled_passthrough",
          dr_process(&dc, in, BLOCK, out, BLOCK) == 0 &&
          memcmp(in, out, (size_t)BLOCK * 2 * sizeof(float)) == 0);
    dr_get_stats(&dc, &s);
    check("disabled_no_correction", s.dropped == 0 && s.inserted == 0);

    /* 3. one-shot drop: input one frame fast */
    dr_init(&dc, (DriftCfg){ 1, 4096 });
    check("drop_one", dr_process(&dc, in, BLOCK + 1, out, BLOCK) == 0);
    dr_get_stats(&dc, &s);
    check("drop_one_stats",
          s.dropped == 1 && s.inserted == 0 && fabs(s.drift) < 1.0);
    int strictly_increasing = 1;
    for (int o = 1; o < BLOCK; o++)
        if (out[o * 2] <= out[(o - 1) * 2]) strictly_increasing = 0;
    check("drop_keeps_order", strictly_increasing);

    /* 4. one-shot insert: input one frame short, previous frame repeated */
    dr_init(&dc, (DriftCfg){ 1, 4096 });
    check("insert_one", dr_process(&dc, in, BLOCK - 1, out, BLOCK) == 0);
    dr_get_stats(&dc, &s);
    check("insert_one_stats",
          s.inserted == 1 && s.dropped == 0 && fabs(s.drift) < 1.0);
    int repeated = -1;
    for (int o = 1; o < BLOCK; o++)
        if (out[o * 2] == out[(o - 1) * 2]) { repeated = o; break; }
    check("insert_repeats_prev",
          repeated > 0 && out[repeated * 2] == in[(repeated - 1) * 2]);

    /* 5. no drift: no corrections, exact copy */
    dr_init(&dc, (DriftCfg){ 1, 4096 });
    check("no_drift", dr_process(&dc, in, BLOCK, out, BLOCK) == 0);
    dr_get_stats(&dc, &s);
    check("no_drift_stats",
          s.dropped == 0 && s.inserted == 0 && s.drift == 0.0 &&
          memcmp(in, out, (size_t)BLOCK * 2 * sizeof(float)) == 0);

    /* 6. long two-clock simulation. A capture phase accumulator generates
          BLOCK*(1+ppm*1e-6) frames per consumer block (128 frames), so the
          per-block count is 127/128/129 with the fractional part carried.
          ~7.5h of audio at 48 kHz in two runs. */
    const int blocks = 200000;
    for (int tc = 0; tc < 2; tc++) {
        const double ppm = tc == 0 ? 50.0 : -120.0;   /* fast then slow */
        dr_init(&dc, (DriftCfg){ 1, 4096 });
        double phase = 0.0;
        int64_t gen_total = 0;
        int steady_ok = 1, order_ok = 1;

        for (int b = 0; b < blocks; b++) {
            phase += BLOCK * (1.0 + ppm * 1e-6);
            int64_t gen = (int64_t)phase;
            phase -= (double)gen;
            gen_total += gen;

            float blk[(BLOCK + 2) * 2];
            for (int64_t k = 0; k < gen; k++) {
                float v = (float)k * 0.001f;   /* monotonic within the block */
                blk[k * 2] = v;
                blk[k * 2 + 1] = -v;
            }
            if (dr_process(&dc, blk, gen, out, BLOCK) != 0) {
                steady_ok = 0; break;
            }
            dr_get_stats(&dc, &s);
            if (b > 100 && fabs(s.drift) >= 1.0) steady_ok = 0;
            for (int o = 1; o < BLOCK; o++) {
                if (ppm > 0.0) { if (out[o * 2] <= out[(o - 1) * 2]) order_ok = 0; }
                else { if (out[o * 2] < out[(o - 1) * 2]) order_ok = 0; }
            }
        }
        dr_get_stats(&dc, &s);
        char name[64];
        snprintf(name, sizeof(name), "steady_under_one_%s",
                 ppm > 0 ? "fast" : "slow");
        check(name, steady_ok && fabs(s.drift) < 1.0);
        snprintf(name, sizeof(name), "output_order_%s",
                 ppm > 0 ? "fast" : "slow");
        check(name, order_ok);

        /* real drift from the two independent clock models vs. corrections */
        int64_t real_drift = gen_total - s.out_total;
        int64_t corrected = s.dropped - s.inserted;
        snprintf(name, sizeof(name), "matches_real_drift_%s",
                 ppm > 0 ? "fast" : "slow");
        check(name, llabs(real_drift - corrected) < 1);
        if (ppm > 0)
            check("fast_clock_drops", s.dropped > 0 && s.inserted == 0);
        else
            check("slow_clock_inserts", s.inserted > 0 && s.dropped == 0);
        check("totals_accounted",
              s.in_total == gen_total &&
              s.out_total == (int64_t)blocks * BLOCK);
    }

    /* 7. drift beyond the limit must report and latch, not correct forever */
    dr_init(&dc, (DriftCfg){ 1, 64 });
    for (int i = 0; i < BLOCK * 2; i++) out[i] = -7.0f;
    check("fault_reported", dr_process(&dc, in, 0, out, BLOCK) == -1);
    dr_get_stats(&dc, &s);
    check("fault_latched",
          s.fault == 1 && dr_process(&dc, in, BLOCK, out, BLOCK) == -1);
    check("fault_leaves_out_untouched", out[0] == -7.0f);

    dr_reset(&dc);
    dr_get_stats(&dc, &s);
    check("reset_clears_fault",
          s.fault == 0 && s.drift == 0.0 &&
          dr_process(&dc, in, BLOCK, out, BLOCK) == 0);

    /* 8. disabled mode: drift accumulates uncorrected; the limit still guards
          it, and with headroom nothing is ever inserted or dropped */
    dr_init(&dc, (DriftCfg){ 0, 16 });
    check("disabled_still_guarded",
          dr_process(&dc, in, BLOCK, out, BLOCK + 16) == -1);

    dr_init(&dc, (DriftCfg){ 0, 1 << 30 });
    int rc = 0;
    for (int b = 0; b < 40; b++)
        rc |= dr_process(&dc, in, BLOCK, out, BLOCK + 2);
    dr_get_stats(&dc, &s);
    check("disabled_headroom_ok",
          rc == 0 && s.fault == 0 && s.dropped == 0 && s.inserted == 0 &&
          fabs(s.drift - 80.0) < 1e-9);

    /* 9. total input starvation: silence inserted, bounded by one frame */
    dr_init(&dc, (DriftCfg){ 1, 4096 });
    check("underrun_insert", dr_process(&dc, in, 0, out, BLOCK) == 0);
    dr_get_stats(&dc, &s);
    check("underrun_stats",
          s.inserted == BLOCK && fabs(s.drift) < 1.0 &&
          out[0] == 0.0f && out[(BLOCK - 1) * 2] == 0.0f);

    /* 10. alternating fast/slow blocks: the sign of the drift flips every
           block, phase must stay bounded and never cross a whole frame */
    dr_init(&dc, (DriftCfg){ 1, 4096 });
    int flip_ok = 1;
    for (int b = 0; b < 10000; b++) {
        int64_t gen = BLOCK + (b % 2 ? 1 : -1);
        if (dr_process(&dc, in, gen, out, BLOCK) != 0) { flip_ok = 0; break; }
        dr_get_stats(&dc, &s);
        if (fabs(s.drift) >= 1.0) flip_ok = 0;
    }
    dr_get_stats(&dc, &s);
    check("flip_stays_bounded", flip_ok && fabs(s.drift) < 1.0);

    /* 11. zero-length blocks must not corrupt the state */
    dr_init(&dc, (DriftCfg){ 1, 4096 });
    check("zero_block", dr_process(&dc, in, 0, out, 0) == 0);
    dr_get_stats(&dc, &s);
    check("zero_block_stats",
          s.in_total == 0 && s.out_total == 0 && s.dropped == 0 &&
          s.inserted == 0 && s.drift == 0.0 && s.fault == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
