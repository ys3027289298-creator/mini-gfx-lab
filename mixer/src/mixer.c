/* mixer: multi-bus audio mixing with per-bus gain, sidechain ducking, soft
   clipping, and a shared reverb with per-bus pre-delay and independent
   dry/wet gains. All buffers are interleaved stereo. */
#include <stdio.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#define FRAMES 128
#define MAX_BUSES 3

typedef struct {
    int bus;        /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
} Source;

typedef struct {
    float gain[MAX_BUSES];
    float duck_depth;    /* 0..1, applied to bus 0 when voice is active */
    int duck_enabled;
} Mixer;

/* ---- shared reverb ------------------------------------------------------
   One reverb instance serves every bus. Each bus has its own pre-delay line,
   but all lines are indexed by one absolute frame counter owned by the
   reverb, so every bus is delayed against the same time origin even when it
   starts feeding the reverb in a later block. */
#define REVERB_MAX_LEN 1024
#define PREDELAY_RING 2048
#define HISTORY_RING REVERB_MAX_LEN

_Static_assert((PREDELAY_RING & (PREDELAY_RING - 1)) == 0, "pre-delay ring must be a power of two");
_Static_assert(PREDELAY_RING > REVERB_MAX_LEN, "pre-delay ring must hold the longest pre-delay");
_Static_assert((HISTORY_RING & (HISTORY_RING - 1)) == 0, "history ring must be a power of two");
_Static_assert(HISTORY_RING >= REVERB_MAX_LEN, "history ring must hold the whole impulse response");

enum {
    REVERB_OK = 0,
    REVERB_ERR_BUS = -1,
    REVERB_ERR_PREDELAY_NEG = -2,
    REVERB_ERR_PREDELAY_LONG = -3,
    REVERB_ERR_SEND = -4,
    REVERB_ERR_INIT = -5,
    REVERB_ERR_ALIGN = -6
};

typedef struct {
    int predelay;       /* samples, 0..len */
    float dry;          /* independent dry gain for this bus */
    float send;         /* send level into the shared wet path */
} BusReverb;

typedef struct {
    int sr;                     /* common sample rate / time base for all buses */
    int len;                    /* reverb length in samples, 1..REVERB_MAX_LEN */
    float wet_gain;             /* global wet return gain */
    float ir[REVERB_MAX_LEN];   /* shared mono impulse response */
    BusReverb bus[MAX_BUSES];
    long long clock;            /* absolute frames processed since init */
    int lane_skew[MAX_BUSES];   /* frames by which a lane's writes are shifted;
                                   the engine keeps this 0 -- a non-zero value
                                   models a lane that started on its own clock */
    float pline[MAX_BUSES][PREDELAY_RING][2];
    float send_hist[HISTORY_RING][2];
    int valid;
} Reverb;

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

static void mix(const Mixer* mx, const Source* srcs, int n,
                const float* const* in, float* out, int frames) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    float bus[MAX_BUSES][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b >= MAX_BUSES) continue;
        for (int i = 0; i < frames * 2; i++)
            bus[b][i] += in[s][i] * srcs[s].gain;
    }

    float dg = 1.0f;
    if (mx->duck_enabled) {
        /* voice bus energy drives ducking of the sfx bus */
        float e = 0.0f;
        for (int i = 0; i < frames * 2; i++) e += fabsf(bus[2][i]);
        e /= (float)(frames * 2);
        if (e > 0.01f) dg = 1.0f - mx->duck_depth;
    }

    for (int i = 0; i < frames * 2; i++) {
        float v = bus[0][i] * mx->gain[0] * dg
                + bus[1][i] * mx->gain[1]
                + bus[2][i] * mx->gain[2];
        out[i] = soft_clip(v);
    }
}

/* ---- reverb ------------------------------------------------------------- */

static void build_ir(Reverb* rv, int len) {
    unsigned int s = 0x12345678u;
    float denom = len > 1 ? (float)(len - 1) : 1.0f;
    for (int k = 0; k < len; k++) {
        s = s * 1664525u + 1013904223u;
        float n = (float)((s >> 8) & 0xFFFFFFu) / 8388608.0f - 1.0f;
        rv->ir[k] = n * expf(-6.0f * (float)k / denom);
    }
    rv->ir[0] = 1.0f;       /* direct tap, keeps the wet onset measurable */
}

int reverb_init(Reverb* rv, int sr, int len) {
    if (!rv || sr <= 0 || len <= 0 || len > REVERB_MAX_LEN) return REVERB_ERR_INIT;
    memset(rv, 0, sizeof(*rv));
    rv->sr = sr;
    rv->len = len;
    rv->wet_gain = 1.0f;
    build_ir(rv, len);
    for (int b = 0; b < MAX_BUSES; b++) {
        rv->bus[b].predelay = 0;
        rv->bus[b].dry = 1.0f;
        rv->bus[b].send = 0.0f;
        rv->lane_skew[b] = 0;
    }
    rv->clock = 0;
    rv->valid = 1;
    return REVERB_OK;
}

int reverb_set_bus(Reverb* rv, int bus, int predelay, float dry, float send) {
    if (!rv || !rv->valid) return REVERB_ERR_INIT;
    if (bus < 0 || bus >= MAX_BUSES) return REVERB_ERR_BUS;
    if (predelay < 0) return REVERB_ERR_PREDELAY_NEG;
    if (predelay > rv->len) return REVERB_ERR_PREDELAY_LONG;
    rv->bus[bus].predelay = predelay;
    rv->bus[bus].dry = dry;
    rv->bus[bus].send = send;
    return REVERB_OK;
}

/* Pre-delay line is indexed by the reverb's absolute frame clock, so every
   bus reads and writes the same slot on a given frame regardless of when it
   joined or how its pre-delay was changed. */
static float dly_get(const Reverb* rv, int b, long long t, int c) {
    long long i = t - rv->lane_skew[b] - rv->bus[b].predelay;
    if (i < 0) return 0.0f;
    return rv->pline[b][(int)(i & (PREDELAY_RING - 1))][c];
}

static float send_get(const Reverb* rv, long long t, long long back, int c) {
    long long i = t - back;
    if (i < 0) return 0.0f;
    return rv->send_hist[(int)(i & (HISTORY_RING - 1))][c];
}

int reverb_process(Reverb* rv, const float* const* bus_in, float* out, int frames) {
    if (!rv || !rv->valid) return REVERB_ERR_INIT;
    if (frames < 0 || (frames > 0 && !out)) return REVERB_ERR_INIT;

    for (int n = 0; n < frames; n++) {
        long long t = rv->clock + n;
        int wi = (int)(t & (PREDELAY_RING - 1));

        /* record the dry buses first so a zero pre-delay reads this frame */
        for (int b = 0; b < MAX_BUSES; b++) {
            const float* in = bus_in ? bus_in[b] : NULL;
            for (int c = 0; c < 2; c++)
                rv->pline[b][wi][c] = in ? in[n * 2 + c] : 0.0f;
        }

        /* one shared wet send: each bus pre-delayed on the common clock */
        float send[2] = { 0.0f, 0.0f };
        for (int b = 0; b < MAX_BUSES; b++) {
            for (int c = 0; c < 2; c++)
                send[c] += rv->bus[b].send * dly_get(rv, b, t, c);
        }
        int si = (int)(t & (HISTORY_RING - 1));
        rv->send_hist[si][0] = send[0];
        rv->send_hist[si][1] = send[1];

        float dry[2] = { 0.0f, 0.0f };
        for (int b = 0; b < MAX_BUSES; b++) {
            const float* in = bus_in ? bus_in[b] : NULL;
            if (!in) continue;
            for (int c = 0; c < 2; c++)
                dry[c] += rv->bus[b].dry * in[n * 2 + c];
        }

        for (int c = 0; c < 2; c++) {
            float wet = 0.0f;
            for (int k = 0; k < rv->len; k++)
                wet += rv->ir[k] * send_get(rv, t, k, c);
            out[n * 2 + c] = soft_clip(dry[c] + rv->wet_gain * wet);
        }
    }
    rv->clock += frames;
    return REVERB_OK;
}

/* Run an impulse through a clean copy of the state and return the first frame
   where the wet return is non-zero. With the dry path muted this is the whole
   chain latency: lane skew + configured pre-delay. */
static int measure_bus_delay(const Reverb* rv, int bus) {
    Reverb tmp = *rv;
    memset(tmp.pline, 0, sizeof(tmp.pline));
    memset(tmp.send_hist, 0, sizeof(tmp.send_hist));
    tmp.clock = 0;
    tmp.wet_gain = 1.0f;
    for (int b = 0; b < MAX_BUSES; b++) {
        tmp.bus[b].dry = 0.0f;
        tmp.bus[b].send = 0.0f;
    }
    tmp.bus[bus].send = 1.0f;

    float zero[FRAMES * 2];
    memset(zero, 0, sizeof(zero));
    float imp[FRAMES * 2];
    memset(imp, 0, sizeof(imp));
    imp[0] = imp[1] = 1.0f;

    const float* in[MAX_BUSES];
    for (int b = 0; b < MAX_BUSES; b++) in[b] = zero;
    in[bus] = imp;

    float out[FRAMES * 2];
    /* only the tolerance band is measured: a lane that fires more than one
       sample early or late is reported as missing (INT_MAX) */
    int limit = tmp.bus[bus].predelay + 3;
    for (int base = 0; base < limit; base += FRAMES) {
        int frames = limit - base < FRAMES ? limit - base : FRAMES;
        reverb_process(&tmp, in, out, frames);
        for (int i = 0; i < frames * 2; i++)
            if (fabsf(out[i]) > 1e-6f) return base + i / 2;
        in[bus] = zero;     /* impulse only in the first block */
        imp[0] = imp[1] = 0.0f;
    }
    return INT_MAX;
}

/* Tolerance is one sample: anything larger means the bus is off the shared
   time base. Returns the index of the first offending bus, or -1. */
static int find_misaligned(const int* configured, const int* measured, int nbus) {
    for (int b = 0; b < nbus; b++) {
        int d = measured[b] - configured[b];
        if (d < 0) d = -d;
        if (d > 1) return b;
    }
    return -1;
}

int reverb_check_alignment(const Reverb* rv, int* bad_bus) {
    if (bad_bus) *bad_bus = -1;
    if (!rv || !rv->valid) return REVERB_ERR_INIT;

    int configured[MAX_BUSES], measured[MAX_BUSES];
    for (int b = 0; b < MAX_BUSES; b++) {
        configured[b] = rv->bus[b].predelay;
        measured[b] = measure_bus_delay(rv, b);
    }
    int bad = find_misaligned(configured, measured, MAX_BUSES);
    if (bad_bus) *bad_bus = bad;
    return bad < 0 ? REVERB_OK : REVERB_ERR_ALIGN;
}

/* ---- reference rendering for point-by-point comparison ------------------ */

/* Brute-force model of exactly what the reverb is specified to do:
     wet[t] = convolve_{b,k}( send[b] * x_b[t - predelay_b - k], ir[k] )
     out[t] = dry[t] + wet_gain * wet[t]
   applied to a whole signal that is known up front. No rings, no clock. */
static int render_reference(const Reverb* rv, const float* const* x, int frames,
                            float* out) {
    for (int t = 0; t < frames; t++) {
        float wet[2] = { 0.0f, 0.0f };
        for (int b = 0; b < MAX_BUSES; b++) {
            if (rv->bus[b].send == 0.0f) continue;
            for (int k = 0; k < rv->len; k++) {
                long long i = (long long)t - rv->lane_skew[b]
                              - rv->bus[b].predelay - k;
                if (i < 0 || i >= frames) continue;
                for (int c = 0; c < 2; c++)
                    wet[c] += rv->bus[b].send * x[b][i * 2 + c] * rv->ir[k];
            }
        }
        for (int c = 0; c < 2; c++) {
            float dry = 0.0f;
            for (int b = 0; b < MAX_BUSES; b++)
                dry += rv->bus[b].dry * x[b][t * 2 + c];
            out[t * 2 + c] = soft_clip(dry + rv->wet_gain * wet[c]);
        }
    }
    return REVERB_OK;
}

/* Render the same input in odd-sized blocks (so pre-delay windows straddle
   block boundaries) and return the largest absolute sample error against the
   hand-spliced reference. */
static float compare_with_reference(const Reverb* rv, const float* const* x,
                                    int frames, float* ref, float* got) {
    render_reference(rv, x, frames, ref);

    Reverb tmp = *rv;
    memset(tmp.pline, 0, sizeof(tmp.pline));
    memset(tmp.send_hist, 0, sizeof(tmp.send_hist));
    tmp.clock = 0;

    static const int BLOCKS[] = { 7, 13, 31, 64, 3 };
    int base = 0, bi = 0;
    while (base < frames) {
        int n = BLOCKS[bi++ % (int)(sizeof(BLOCKS) / sizeof(BLOCKS[0]))];
        if (n > frames - base) n = frames - base;
        const float* part[MAX_BUSES];
        for (int b = 0; b < MAX_BUSES; b++) part[b] = x[b] + base * 2;
        reverb_process(&tmp, part, got + base * 2, n);
        base += n;
    }

    float err = 0.0f;
    for (int i = 0; i < frames * 2; i++) {
        float d = fabsf(got[i] - ref[i]);
        if (d > err) err = d;
    }
    return err;
}

static float peak(const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(b[i]) > m) m = fabsf(b[i]);
    return m;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

#define SIG 512

static void fill_signal(float* buf, int frames, float freq, float amp) {
    for (int n = 0; n < frames; n++) {
        float v = amp * sinf(2.0f * 3.14159265f * freq * (float)n / 48000.0f);
        buf[n * 2] = buf[n * 2 + 1] = v;
    }
}

int main(void) {
    float sfx[FRAMES * 2], music[FRAMES * 2], voice[FRAMES * 2], out[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.0f;
    }
    const float* in[3] = { sfx, music, voice };
    Source srcs[3] = { {0, 1.0f}, {1, 1.0f}, {2, 1.0f} };
    Mixer mx = { {1.0f, 1.0f, 1.0f}, 0.5f, 1 };

    mix(&mx, srcs, 3, in, out, FRAMES);
    check("sum_ok", fabsf(out[0] - 0.75f) < 1e-5f);
    check("no_clip_when_low", peak(out, FRAMES * 2) < 0.99f);

    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.3f;
    mix(&mx, srcs, 3, in, out, FRAMES);
    float with_duck = out[0];

    mx.duck_enabled = 0;
    mix(&mx, srcs, 3, in, out, FRAMES);
    float without_duck = out[0];
    check("duck_lowers_sfx", with_duck < without_duck);

    mx.duck_enabled = 1;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.0f;
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("duck_idle_passthrough", fabsf(out[0] - 0.75f) < 1e-5f);

    for (int s = 0; s < 3; s++) srcs[s].gain = 4.0f;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.5f;
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("soft_clip_bounds", peak(out, FRAMES * 2) <= 1.0f + 1e-6f);

    srcs[0].bus = 7;   /* invalid bus must be ignored */
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("bad_bus_ignored", peak(out, FRAMES * 2) <= 1.0f + 1e-6f);

    /* ---- reverb: validation ------------------------------------------- */
    Reverb rv;
    check("init_ok", reverb_init(&rv, 48000, 96) == REVERB_OK);
    check("reject_negative_predelay",
          reverb_set_bus(&rv, 0, -1, 1.0f, 0.2f) == REVERB_ERR_PREDELAY_NEG);
    check("reject_predelay_past_len",
          reverb_set_bus(&rv, 0, 97, 1.0f, 0.2f) == REVERB_ERR_PREDELAY_LONG);
    check("accept_predelay_at_len",
          reverb_set_bus(&rv, 0, 96, 1.0f, 0.2f) == REVERB_OK);
    check("reject_bad_bus", reverb_set_bus(&rv, 3, 0, 1.0f, 0.2f) == REVERB_ERR_BUS);
    check("bad_set_leaves_state",
          rv.bus[0].predelay == 96);   /* the rejected -1 did not stick */

    /* reverb_set_bus also silences the other buses, so the dry/wet paths are
       measured one bus at a time */
    Reverb a, b;
    reverb_init(&a, 48000, 96);
    reverb_set_bus(&a, 0, 20, 1.0f, 0.0f);      /* dry only */
    reverb_init(&b, 48000, 96);
    reverb_set_bus(&b, 0, 20, 0.0f, 0.0f);      /* nothing */

    float x0[SIG * 2], ref[SIG * 2], got[SIG * 2];
    fill_signal(x0, SIG, 440.0f, 0.4f);
    const float* xin[MAX_BUSES] = { x0, NULL, NULL };
    float dry_out[SIG * 2], silent[SIG * 2];
    reverb_process(&a, xin, dry_out, SIG);
    reverb_process(&b, xin, silent, SIG);
    float dry_err = 0.0f, silent_peak = peak(silent, SIG * 2);
    for (int i = 0; i < SIG * 2; i++) {
        float d = fabsf(dry_out[i] - x0[i]);
        if (d > dry_err) dry_err = d;
    }
    check("dry_path_untouched", dry_err < 1e-5f);
    check("zero_dry_zero_send_is_silent", silent_peak < 1e-6f);

    /* direct sound is present from sample 0 while the wet onset is held back
       by the pre-delay: feed silence, then an impulse, and find the onset */
    reverb_init(&a, 48000, 96);
    reverb_set_bus(&a, 0, 40, 1.0f, 1.0f);      /* equal-power dry + wet */
    float imp[SIG * 2];
    memset(imp, 0, sizeof(imp));
    imp[0] = imp[1] = 0.5f;
    const float* imp_in[MAX_BUSES] = { imp, NULL, NULL };
    reverb_process(&a, imp_in, dry_out, SIG);
    int wet_onset = -1;
    for (int n = 1; n < SIG; n++)   /* skip the dry impulse at n=0 */
        if (fabsf(dry_out[n * 2]) > 1e-6f) { wet_onset = n; break; }
    check("predelay_shifts_wet_onset", wet_onset == 40);
    check("dry_direct_not_delayed", fabsf(dry_out[0] - 0.5f) < 1e-5f);

    /* ---- reverb: per-bus pre-delay on one shared time base ------------ */
    reverb_init(&a, 48000, 96);
    reverb_set_bus(&a, 0, 10, 0.0f, 0.5f);
    reverb_set_bus(&a, 1, 25, 0.0f, 0.5f);
    reverb_set_bus(&a, 2, 60, 0.0f, 0.5f);
    int bad = -123;
    check("alignment_ok", reverb_check_alignment(&a, &bad) == REVERB_OK);
    check("alignment_reports_none", bad == -1);

    /* a lane whose pre-delay line was started on a different clock is off the
       shared time base; the check must name that bus */
    a.lane_skew[1] = 2;
    check("misaligned_detected",
          reverb_check_alignment(&a, &bad) == REVERB_ERR_ALIGN);
    check("misaligned_names_bus", bad == 1);
    a.lane_skew[1] = 0;

    /* one sample of skew is inside tolerance, two is not */
    a.lane_skew[2] = 1;
    check("one_sample_skew_tolerated",
          reverb_check_alignment(&a, &bad) == REVERB_OK);
    a.lane_skew[2] = 2;
    check("two_sample_skew_rejected",
          reverb_check_alignment(&a, &bad) == REVERB_ERR_ALIGN);
    check("two_sample_skew_names_bus", bad == 2);
    a.lane_skew[2] = 0;

    /* cross-block: pre-delay windows straddle blocks, and the late bus only
       starts sending in a later block; it must still line up with the rest */
    float s0[SIG * 2], s1[SIG * 2], s2[SIG * 2];
    fill_signal(s0, SIG, 220.0f, 0.3f);
    fill_signal(s1, SIG, 330.0f, 0.2f);
    fill_signal(s2, SIG, 550.0f, 0.1f);
    const float* xs[MAX_BUSES] = { s0, s1, s2 };
    reverb_init(&a, 48000, 64);
    reverb_set_bus(&a, 0, 5, 1.0f, 0.4f);
    reverb_set_bus(&a, 1, 30, 1.0f, 0.3f);
    reverb_set_bus(&a, 2, 62, 1.0f, 1.0f);
    float err = compare_with_reference(&a, xs, SIG, ref, got);
    printf("manual_compare_max_err=%.9f\n", err);
    check("matches_manual_delay", err < 1e-5f);

    /* a deliberately mis-spliced bus (pre-delay off by one sample) must show
       up as a clear error, proving the comparison is not trivially passing */
    float shifted[SIG * 2];
    render_reference(&a, xs, SIG, ref);
    Reverb off = a;
    off.bus[2].predelay += 1;
    render_reference(&off, xs, SIG, shifted);
    float diff_err = 0.0f;
    for (int i = 0; i < SIG * 2; i++) {
        float d = fabsf(shifted[i] - ref[i]);
        if (d > diff_err) diff_err = d;
    }
    printf("one_sample_mismatch_err=%.9f\n", diff_err);
    check("comparison_detects_one_sample", diff_err > 1e-4f);

    /* per-bus dry gains are independent: mute one bus's dry and the other
       bus's direct sound is untouched */
    reverb_init(&a, 48000, 64);
    reverb_set_bus(&a, 0, 5, 0.0f, 0.0f);
    reverb_set_bus(&a, 1, 30, 1.0f, 0.0f);
    reverb_set_bus(&a, 2, 62, 0.0f, 0.0f);
    reverb_process(&a, xs, got, SIG);
    float dry_mix_err = 0.0f;
    for (int i = 0; i < SIG * 2; i++) {
        float d = fabsf(got[i] - s1[i]);
        if (d > dry_mix_err) dry_mix_err = d;
    }
    check("per_bus_dry_independent", dry_mix_err < 1e-5f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
