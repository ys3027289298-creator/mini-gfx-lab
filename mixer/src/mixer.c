
/* mixer: multi-bus audio mixing with per-bus gain, sidechain ducking, soft
   clipping and per-source polyphase sample-rate conversion. Sources may run
   at any sample rate; each is resampled to the bus rate independently.
   All buffers are interleaved stereo. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_SRC 8
#define FRAMES 128
#define BUS_COUNT 3

/* ---- polyphase resampler configuration ---- */
#define RS_HALF      16              /* kernel half length: 32 taps total */
#define RS_TAPS      (2 * RS_HALF)
#define RS_PHASES    128             /* polyphase coefficient phases */
#define RS_BETA      8.6             /* Kaiser beta: ~ -80 dB stopband */
#define RS_XFADE     32              /* rate-switch crossfade in frames */
#define RS_MAX_PLANS 12              /* distinct (in,out) rate pairs */
#define RS_DELAY_CAP (FRAMES * 8 + RS_TAPS + 8)

/* A Plan describes one in_rate -> out_rate ratio. The polyphase table holds
   RS_PHASES+1 fractional-delay snapshots of the prototype filter; runtime
   phases are resolved by linear interpolation between adjacent rows. */
typedef struct {
    int in_rate, out_rate;
    double ratio;                    /* input samples per output sample */
    double fc;                       /* cutoff, input-domain cycles/sample */
    double gain;                     /* kernel DC gain = 2*fc */
    int identity;                    /* in_rate == out_rate fast path */
    float table[RS_PHASES + 1][RS_TAPS];
} Plan;

typedef struct {
    Plan plans[RS_MAX_PLANS];
    int count;
} PlanCache;

typedef struct {
    const Plan* plan;                /* current filter/ratio */
    const Plan* prev_plan;           /* old plan during a switch crossfade */
    int xfade_left;
    double pos;                      /* read position in delay, input frames */
    int stored;                      /* valid frames in delay */
    float delay[RS_DELAY_CAP * 2];   /* history + unconsumed input, stereo */
} Resampler;

static long rs_tap_ops;              /* deterministic per-tap work counter */

static double bessel_i0(double x) {
    double sum = 1.0, term = 1.0, y = x * 0.5;
    for (int m = 1; m < 32; m++) {
        term *= (y / m) * (y / m);
        sum += term;
        if (term < 1e-16 * sum) break;
    }
    return sum;
}

static double rs_sinc(double x) {
    if (fabs(x) < 1e-12) return 1.0;
    double px = M_PI * x;
    return sin(px) / px;
}

static double kaiser(double d) {
    double u = d / RS_HALF;
    if (fabs(u) >= 1.0) return 0.0;
    double arg = RS_BETA * sqrt(1.0 - u * u);
    return bessel_i0(arg) / bessel_i0(RS_BETA);
}

/* prototype lowpass: bandlimited to the lower of the two Nyquist rates */
static double kernel_at(const Plan* p, double d) {
    return p->gain * rs_sinc(2.0 * p->fc * d) * kaiser(d);
}

static void plan_init(Plan* p, int in_rate, int out_rate) {
    p->in_rate = in_rate;
    p->out_rate = out_rate;
    p->ratio = (double)in_rate / (double)out_rate;
    p->identity = in_rate == out_rate;
    p->fc = 0.5 * (p->ratio < 1.0 ? 1.0 : 1.0 / p->ratio);
    p->gain = 2.0 * p->fc;
    for (int ph = 0; ph <= RS_PHASES; ph++) {
        double frac = (double)ph / RS_PHASES;
        for (int k = 0; k < RS_TAPS; k++) {
            double j = k - (RS_HALF - 1);
            p->table[ph][k] = (float)kernel_at(p, j - frac);
        }
    }
}

/* Plans are cached: every source at the same rate pair shares one table, so
   setup cost is paid per distinct ratio, not per source. */
static const Plan* cache_get(PlanCache* c, int in_rate, int out_rate) {
    for (int i = 0; i < c->count; i++) {
        if (c->plans[i].in_rate == in_rate && c->plans[i].out_rate == out_rate)
            return &c->plans[i];
    }
    if (c->count >= RS_MAX_PLANS) return NULL;
    Plan* p = &c->plans[c->count++];
    plan_init(p, in_rate, out_rate);
    return p;
}

static void resampler_init(Resampler* r, const Plan* p) {
    memset(r, 0, sizeof(*r));
    r->plan = p;
    r->stored = RS_HALF - 1;         /* zero prefill; delay memory is zeroed */
    r->pos = RS_HALF - 1;
}

/* Input frames the caller must supply next to produce out_frames output. */
static int resampler_input_needed(const Resampler* r, int out_frames) {
    /* while a crossfade runs both old and new filters are evaluated, so the
       polyphase lookahead requirement applies even when the new plan is
       identity-rate */
    int polyphase = !r->plan->identity ||
                    (r->xfade_left > 0 && r->prev_plan &&
                     !r->prev_plan->identity);
    int need;
    if (!polyphase) {
        need = (int)floor(r->pos) + out_frames - r->stored;
    } else {
        double last_pos = r->pos + (double)(out_frames - 1) * r->plan->ratio;
        need = (int)floor(last_pos) + RS_HALF + 1 - r->stored;
    }
    return need > 0 ? need : 0;
}

static void filter_frame(const Plan* p, const float* delay, int idx,
                         double frac, float* ol, float* or_) {
    if (p->identity) {
        *ol = delay[idx * 2];
        *or_ = delay[idx * 2 + 1];
        return;
    }
    double fp = frac * RS_PHASES;
    int p0 = (int)fp;
    if (p0 >= RS_PHASES) p0 = RS_PHASES - 1;
    float a = (float)(fp - p0);
    const float* c0 = p->table[p0];
    const float* c1 = p->table[p0 + 1];
    int base = idx - (RS_HALF - 1);
    float l = 0.0f, rr = 0.0f;
    for (int k = 0; k < RS_TAPS; k++) {
        float c = c0[k] + (c1[k] - c0[k]) * a;
        int di = (base + k) * 2;
        l += c * delay[di];
        rr += c * delay[di + 1];
    }
    rs_tap_ops += RS_TAPS;
    *ol = l;
    *or_ = rr;
}

/* Smooth rate change: history and phase are kept, the new filter is faded in
   over RS_XFADE frames so no step discontinuity reaches the bus. */
static void resampler_set_plan(Resampler* r, const Plan* p) {
    if (p == r->plan) return;
    r->prev_plan = r->plan;
    r->plan = p;
    r->xfade_left = RS_XFADE;
}

/* Streams one block. Returns output frames produced (short of out_frames only
   on input underrun; tail is zero-filled). State persists between calls, so
   phase is continuous across block boundaries. */
static int resampler_process(Resampler* r, const float* in, int in_frames,
                             float* out, int out_frames) {
    if (r->stored + in_frames > RS_DELAY_CAP) return -1;
    memcpy(r->delay + r->stored * 2, in, sizeof(float) * 2 * (size_t)in_frames);
    r->stored += in_frames;

    int produced = 0;
    for (int m = 0; m < out_frames; m++) {
        int idx = (int)floor(r->pos);
        int polyphase = !r->plan->identity ||
                        (r->xfade_left > 0 && r->prev_plan &&
                         !r->prev_plan->identity);
        int last_read = polyphase ? idx + RS_HALF : idx;
        if (last_read + 1 > r->stored) break;        /* underrun */
        double frac = r->pos - idx;
        float nl, nr;
        filter_frame(r->plan, r->delay, idx, frac, &nl, &nr);

        if (r->xfade_left > 0 && r->prev_plan) {
            float ol, orr;
            filter_frame(r->prev_plan, r->delay, idx, frac, &ol, &orr);
            float w_new = (float)(RS_XFADE - r->xfade_left + 1) / RS_XFADE;
            nl = ol + (nl - ol) * w_new;
            nr = orr + (nr - orr) * w_new;
            if (--r->xfade_left == 0) r->prev_plan = NULL;
        }
        out[m * 2] = nl;
        out[m * 2 + 1] = nr;
        r->pos += r->plan->ratio;
        produced++;
    }

    /* drop input samples no future output can reference */
    int keep_from = (int)floor(r->pos) - (RS_HALF - 1);
    if (keep_from > 0) {
        int remain = r->stored - keep_from;
        memmove(r->delay, r->delay + keep_from * 2,
                sizeof(float) * 2 * (size_t)remain);
        r->stored = remain;
        r->pos -= keep_from;
    }
    for (int m = produced; m < out_frames; m++) {
        out[m * 2] = 0.0f;
        out[m * 2 + 1] = 0.0f;
    }
    return produced;
}

/* ---------------- mixer ---------------- */

typedef struct {
    int bus;          /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
    int sample_rate;  /* source rate; converted to Mixer.bus_rate */
} Source;

typedef struct {
    float gain[BUS_COUNT];
    float duck_depth;    /* 0..1, applied to bus 0 when voice is active */
    int duck_enabled;
    int bus_rate;
    PlanCache cache;
    Resampler rs[MAX_SRC];
    int rs_used[MAX_SRC];
    int last_rate[MAX_SRC];
    float rs_buf[MAX_SRC][FRAMES * 2];
} Mixer;

static void mixer_init(Mixer* mx, int bus_rate) {
    memset(mx, 0, sizeof(*mx));
    for (int b = 0; b < BUS_COUNT; b++) mx->gain[b] = 1.0f;
    mx->duck_depth = 0.5f;
    mx->bus_rate = bus_rate;
}

/* Bus rate change: every source resampler switches filter without losing
   history, so the swap is click-free. */
static void mixer_set_bus_rate(Mixer* mx, int rate) {
    if (rate == mx->bus_rate) return;
    mx->bus_rate = rate;
    for (int s = 0; s < MAX_SRC; s++) {
        if (!mx->rs_used[s]) continue;
        const Plan* p = cache_get(&mx->cache, mx->last_rate[s], rate);
        if (p) resampler_set_plan(&mx->rs[s], p);
    }
}

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

static void mix(Mixer* mx, const Source* srcs, int n,
                const float* const* in, const int* in_frames,
                float* out, int frames) {
    float bus[BUS_COUNT][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b >= BUS_COUNT) continue;

        if (!mx->rs_used[s]) {
            const Plan* p = cache_get(&mx->cache, srcs[s].sample_rate,
                                      mx->bus_rate);
            if (!p) continue;               /* plan table full: skip source */
            resampler_init(&mx->rs[s], p);
            mx->rs_used[s] = 1;
            mx->last_rate[s] = srcs[s].sample_rate;
        } else if (mx->last_rate[s] != srcs[s].sample_rate) {
            const Plan* p = cache_get(&mx->cache, srcs[s].sample_rate,
                                      mx->bus_rate);
            if (!p) continue;
            resampler_set_plan(&mx->rs[s], p);
            mx->last_rate[s] = srcs[s].sample_rate;
        }

        int got = resampler_process(&mx->rs[s], in[s], in_frames[s],
                                    mx->rs_buf[s], frames);
        if (got < 0) continue;
        for (int i = 0; i < frames * 2; i++)
            bus[b][i] += mx->rs_buf[s][i] * srcs[s].gain;
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

static float peak(const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(b[i]) > m) m = fabsf(b[i]);
    return m;
}

/* ---------------- test harness ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void make_tone(float* buf, int frames, double fs,
                      const double* f, const double* amp, int nf,
                      double* phase) {
    for (int i = 0; i < frames; i++) {
        double v = 0.0;
        for (int t = 0; t < nf; t++) {
            v += amp[t] * sin(2.0 * M_PI * f[t] * (*phase) / fs);
        }
        (*phase)++;
        buf[i * 2] = (float)v;
        buf[i * 2 + 1] = (float)v;
    }
}

/* Reference resampler: same prototype kernel, evaluated directly in double
   with no polyphase table — the engine output is measured against this. */
static double ref_filter(const Plan* p, const double* delay,
                         int idx, double frac) {
    double acc = 0.0;
    for (int k = 0; k < RS_TAPS; k++) {
        double j = k - (RS_HALF - 1);
        acc += kernel_at(p, j - frac) * delay[idx + (int)j];
    }
    return acc;
}

static double db_error(const float* got, const double* ref, int n) {
    double rr = 0.0, re = 0.0;
    for (int i = 0; i < n; i++) {
        double e = ref[i] - got[i * 2];
        rr += ref[i] * ref[i];
        re += e * e;
    }
    return 20.0 * log10(sqrt(re / n) / sqrt(rr / n));
}

/* Full engine-vs-reference comparison for one rate pair. */
static double resampler_error_db(int in_rate, int out_rate) {
    enum { OUT_TARGET = 1024 };
    Plan plan, ref_plan;
    plan_init(&plan, in_rate, out_rate);
    plan_init(&ref_plan, in_rate, out_rate);

    int cap_in = OUT_TARGET * in_rate / out_rate + 4 * RS_TAPS;
    float* input = (float*)malloc(sizeof(float) * 2 * (size_t)cap_in);
    float* output = (float*)malloc(sizeof(float) * 2 * (size_t)OUT_TARGET);
    double* ref_delay = (double*)malloc(sizeof(double) *
                                        (size_t)(cap_in + 2 * RS_TAPS));
    double* ref_out = (double*)malloc(sizeof(double) * (size_t)OUT_TARGET);

    double freqs[] = { 1000.0, 3700.0 };
    double amps[] = { 0.6, 0.4 };
    double ph = 0.0;
    make_tone(input, cap_in, in_rate, freqs, amps, 2, &ph);

    Resampler r;
    resampler_init(&r, &plan);
    int produced = 0, consumed = 0;
    while (produced < OUT_TARGET) {
        int want = OUT_TARGET - produced;
        if (want > FRAMES) want = FRAMES;
        int need = resampler_input_needed(&r, want);
        int got = resampler_process(&r, input + consumed * 2, need,
                                    output + produced * 2, want);
        consumed += need;
        produced += got;
    }

    for (int i = 0; i < RS_HALF - 1; i++) ref_delay[i] = 0.0;
    for (int i = 0; i < cap_in; i++)
        ref_delay[RS_HALF - 1 + i] = input[i * 2];
    double pos = RS_HALF - 1;
    for (int m = 0; m < OUT_TARGET; m++) {
        int idx = (int)floor(pos);
        ref_out[m] = ref_filter(&ref_plan, ref_delay, idx, pos - idx);
        pos += ref_plan.ratio;
    }

    int lo = RS_TAPS, hi = OUT_TARGET - RS_TAPS;
    double db = db_error(output + lo * 2, ref_out + lo, hi - lo);

    free(input); free(output); free(ref_delay); free(ref_out);
    return db;
}

static float max_delta(const float* b, int frames) {
    float m = 0.0f;
    for (int i = 2; i < frames * 2; i++) {
        float d = fabsf(b[i] - b[i - 2]);
        if (d > m) m = d;
    }
    return m;
}

static float estimate_freq(const float* b, int frames, int fs) {
    int crossings = 0;
    for (int i = 1; i < frames; i++)
        if (b[(i - 1) * 2] <= 0.0f && b[i * 2] > 0.0f) crossings++;
    return (float)crossings * (float)fs / (float)frames;
}

int main(void) {
    /* ---- original mixer behavior, identity rate path ---- */
    float sfx[FRAMES * 2], music[FRAMES * 2], voice[FRAMES * 2], out[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.0f;
    }
    const float* in[3] = { sfx, music, voice };
    int in_frames[3] = { FRAMES, FRAMES, FRAMES };
    Source srcs[3] = { {0, 1.0f, 48000}, {1, 1.0f, 48000}, {2, 1.0f, 48000} };
    Mixer mx;
    mixer_init(&mx, 48000);
    mx.duck_enabled = 1;

    mix(&mx, srcs, 3, in, in_frames, out, FRAMES);
    check("sum_ok", fabsf(out[0] - 0.75f) < 1e-5f);
    check("no_clip_when_low", peak(out, FRAMES * 2) < 0.99f);

    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.3f;
    mix(&mx, srcs, 3, in, in_frames, out, FRAMES);
    float with_duck = out[0];

    mx.duck_enabled = 0;
    mix(&mx, srcs, 3, in, in_frames, out, FRAMES);
    float without_duck = out[0];
    check("duck_lowers_sfx", with_duck < without_duck);

    mx.duck_enabled = 1;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.0f;
    mix(&mx, srcs, 3, in, in_frames, out, FRAMES);
    check("duck_idle_passthrough", fabsf(out[0] - 0.75f) < 1e-5f);

    for (int s = 0; s < 3; s++) srcs[s].gain = 4.0f;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.5f;
    mix(&mx, srcs, 3, in, in_frames, out, FRAMES);
    check("soft_clip_bounds", peak(out, FRAMES * 2) <= 1.0f + 1e-6f);

    srcs[0].bus = 7;   /* invalid bus must be ignored */
    mix(&mx, srcs, 3, in, in_frames, out, FRAMES);
    check("bad_bus_ignored", peak(out, FRAMES * 2) <= 1.0f + 1e-6f);
    srcs[0].bus = 0;

    /* ---- identity rate is bit-exact passthrough ---- */
    Plan id_plan;
    plan_init(&id_plan, 48000, 48000);
    Resampler rr;
    resampler_init(&rr, &id_plan);
    float id_in[FRAMES * 2], id_out[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) id_in[i] = sinf(i * 0.1f) * 0.5f;
    int need = resampler_input_needed(&rr, FRAMES);
    resampler_process(&rr, id_in, need, id_out, FRAMES);
    int exact = 1;
    for (int i = 0; i < FRAMES * 2; i++)
        if (id_out[i] != id_in[i]) { exact = 0; break; }
    check("identity_bitexact", exact);

    /* ---- phase continuity: one 256-frame job == two 128-frame jobs ---- */
    Plan p44_48;
    plan_init(&p44_48, 44100, 48000);
    enum { BIG = 256 };
    int big_cap = BIG * 44100 / 48000 + 4 * RS_TAPS;
    float* big_in = (float*)malloc(sizeof(float) * 2 * (size_t)big_cap);
    float one_out[BIG * 2], two_out[BIG * 2];
    double ph1 = 0.0;
    double tf[] = { 1234.0 };
    double ta[] = { 0.7 };
    make_tone(big_in, big_cap, 44100, tf, ta, 1, &ph1);

    Resampler r_one;
    resampler_init(&r_one, &p44_48);
    int n1 = resampler_input_needed(&r_one, BIG);
    resampler_process(&r_one, big_in, n1, one_out, BIG);

    Resampler r_two;
    resampler_init(&r_two, &p44_48);
    int c = 0;
    for (int blk = 0; blk < 2; blk++) {
        int nn = resampler_input_needed(&r_two, FRAMES);
        resampler_process(&r_two, big_in + c * 2, nn,
                          two_out + blk * FRAMES * 2, FRAMES);
        c += nn;
    }
    check("phase_continuous", memcmp(one_out, two_out, sizeof(one_out)) == 0);
    check("split_consumes_same_input", c == n1);
    free(big_in);

    /* ---- accuracy: polyphase engine vs double-precision reference ---- */
    struct { int in_rate, out_rate; } pairs[] = {
        { 22050, 48000 }, { 48000, 44100 }, { 44100, 48000 },
        { 8000, 48000 },  { 48000, 8000 },  { 11025, 48000 },
        { 96000, 44100 }, { 32000, 48000 },
    };
    int err_ok = 1;
    for (size_t pi = 0; pi < sizeof(pairs) / sizeof(pairs[0]); pi++) {
        double db = resampler_error_db(pairs[pi].in_rate, pairs[pi].out_rate);
        printf("err_db %d->%d = %0.1f\n", pairs[pi].in_rate,
               pairs[pi].out_rate, db);
        err_ok &= db < -60.0;
    }
    check("error_below_60db", err_ok);

    /* ---- pitch preservation: a 1 kHz tone stays 1 kHz after conversion ---- */
    Plan p22_48;
    plan_init(&p22_48, 22050, 48000);
    enum { PITCH_OUT = 2048 };
    int pitch_cap = PITCH_OUT * 22050 / 48000 + 4 * RS_TAPS;
    float* pitch_in = (float*)malloc(sizeof(float) * 2 * (size_t)pitch_cap);
    float* pitch_out = (float*)malloc(sizeof(float) * 2 * (size_t)PITCH_OUT);
    double ph2 = 0.0, pf[] = { 1000.0 }, pa[] = { 0.8 };
    make_tone(pitch_in, pitch_cap, 22050, pf, pa, 1, &ph2);
    Resampler r_pitch;
    resampler_init(&r_pitch, &p22_48);
    int pc = 0, po = 0;
    while (po < PITCH_OUT) {
        int want = PITCH_OUT - po;
        if (want > FRAMES) want = FRAMES;
        int nn = resampler_input_needed(&r_pitch, want);
        po += resampler_process(&r_pitch, pitch_in + pc * 2, nn,
                                pitch_out + po * 2, want);
        pc += nn;
    }
    float fest = estimate_freq(pitch_out + RS_TAPS * 2,
                               PITCH_OUT - 2 * RS_TAPS, 48000);
    printf("pitch_est=%0.1f Hz\n", fest);
    check("pitch_preserved", fabsf(fest - 1000.0f) < 30.0f);
    free(pitch_in); free(pitch_out);

    /* ---- no pops on rate switch; contrast with a hard reset ---- */
    Plan sw_a, sw_b, sw_c;
    plan_init(&sw_a, 48000, 48000);
    plan_init(&sw_b, 48000, 24000);
    plan_init(&sw_c, 48000, 44100);
    enum { SW_BLOCKS = 8 };
    float* sw_out = (float*)malloc(sizeof(float) * 2 * FRAMES * SW_BLOCKS);
    float* sw_hard = (float*)malloc(sizeof(float) * 2 * FRAMES * SW_BLOCKS);
    int sw_cap = FRAMES * 8 * 2 + 256;
    float* sw_in = (float*)malloc(sizeof(float) * 2 * (size_t)sw_cap);
    double ph3 = 0.0, swf[] = { 1000.0 }, swa[] = { 0.8 };
    make_tone(sw_in, sw_cap, 48000, swf, swa, 1, &ph3);

    Resampler rsw, rhard;
    resampler_init(&rsw, &sw_a);
    resampler_init(&rhard, &sw_a);
    const Plan* sequence[SW_BLOCKS] = {
        &sw_a, &sw_a, &sw_b, &sw_b, &sw_c, &sw_c, &sw_a, &sw_a
    };
    int sic = 0, hic = 0, so = 0, ho = 0;
    for (int blk = 0; blk < SW_BLOCKS; blk++) {
        if (blk > 0 && sequence[blk] != sequence[blk - 1]) {
            resampler_set_plan(&rsw, sequence[blk]);
            resampler_init(&rhard, sequence[blk]);   /* the naive way */
        }
        int n_s = resampler_input_needed(&rsw, FRAMES);
        so += resampler_process(&rsw, sw_in + sic * 2, n_s,
                                sw_out + so * 2, FRAMES);
        sic += n_s;
        int n_h = resampler_input_needed(&rhard, FRAMES);
        ho += resampler_process(&rhard, sw_in + hic * 2, n_h,
                                sw_hard + ho * 2, FRAMES);
        hic += n_h;
    }
    float smooth_d = max_delta(sw_out, so);
    float hard_d = max_delta(sw_hard, ho);
    printf("switch_delta smooth=%0.4f hard_reset=%0.4f\n", smooth_d, hard_d);
    check("no_pop_on_switch", smooth_d < 0.4f);
    check("hard_reset_does_pop", hard_d > 0.5f);
    free(sw_out); free(sw_hard); free(sw_in);

    /* ---- cost linear in source count; plans shared per rate pair ---- */
    PlanCache lc;
    memset(&lc, 0, sizeof(lc));
    Resampler lin[8];
    const Plan* lp[8];
    for (int s = 0; s < 8; s++) {
        lp[s] = cache_get(&lc, 44100, 48000);
        resampler_init(&lin[s], lp[s]);
    }
    check("plan_shared", lc.count == 1 && lp[0] == lp[7]);
    int lin_cap = FRAMES * 2 + 4 * RS_TAPS;
    float* lin_in = (float*)malloc(sizeof(float) * 2 * (size_t)lin_cap);
    float* lin_out = (float*)malloc(sizeof(float) * 2 * FRAMES * 8);
    double ph4 = 0.0;
    make_tone(lin_in, lin_cap, 44100, tf, ta, 1, &ph4);

    rs_tap_ops = 0;
    for (int s = 0; s < 4; s++) {
        int nn = resampler_input_needed(&lin[s], FRAMES);
        resampler_process(&lin[s], lin_in, nn, lin_out + s * FRAMES * 2, FRAMES);
    }
    long ops4 = rs_tap_ops;
    rs_tap_ops = 0;
    for (int s = 0; s < 8; s++) {
        int nn = resampler_input_needed(&lin[s], FRAMES);
        resampler_process(&lin[s], lin_in, nn, lin_out + s * FRAMES * 2, FRAMES);
    }
    long ops8 = rs_tap_ops;
    printf("tap_ops 4src=%ld 8src=%ld\n", ops4, ops8);
    check("cost_linear", ops8 == 2 * ops4 && ops4 > 0);
    free(lin_in); free(lin_out);

    /* ---- mixed source rates on the bus stay bounded and fully produced ---- */
    Mixer mx2;
    mixer_init(&mx2, 48000);
    enum { RATES = 3 };
    int rates[RATES] = { 22050, 44100, 48000 };
    float* mi[RATES];
    int mi_frames[RATES];
    Source msrc[RATES];
    Resampler probe[RATES];
    for (int s = 0; s < RATES; s++) {
        const Plan* pp = cache_get(&mx2.cache, rates[s], 48000);
        resampler_init(&probe[s], pp);
        mi_frames[s] = resampler_input_needed(&probe[s], FRAMES);
        mi[s] = (float*)malloc(sizeof(float) * 2 * (size_t)mi_frames[s]);
        double phx = 0.0;
        make_tone(mi[s], mi_frames[s], rates[s], pf, pa, 1, &phx);
        msrc[s] = (Source){ s % BUS_COUNT, 0.5f, rates[s] };
    }
    mix(&mx2, msrc, RATES, (const float* const*)mi, mi_frames, out, FRAMES);
    /* all resampler buffers are finite and the mix stays in clip bounds */
    int finite_out = 1;
    for (int i = 0; i < FRAMES * 2; i++) finite_out &= isfinite(out[i]);
    check("mixed_rates_bounded",
          finite_out && peak(out, FRAMES * 2) <= 1.0f + 1e-6f);
    for (int s = 0; s < RATES; s++) free(mi[s]);

    /* ---- bus rate switch is click-free for every active source ---- */
    mixer_set_bus_rate(&mx2, 24000);
    check("bus_switch_xfade_armed", mx2.rs[0].xfade_left > 0);
    for (int s = 0; s < RATES; s++) {
        const Plan* pp = cache_get(&mx2.cache, rates[s], 24000);
        Resampler tmp;
        resampler_init(&tmp, pp);
        int nn = resampler_input_needed(&tmp, FRAMES);
        mi[s] = (float*)malloc(sizeof(float) * 2 * (size_t)nn);
        double phx = 0.0;
        make_tone(mi[s], nn, rates[s], pf, pa, 1, &phx);
        mi_frames[s] = nn;
    }
    float d_before_switch = max_delta(out, FRAMES);
    mix(&mx2, msrc, RATES, (const float* const*)mi, mi_frames, out, FRAMES);
    check("bus_switch_bounded",
          isfinite(out[0]) && peak(out, FRAMES * 2) <= 1.0f + 1e-6f);
    check("bus_switch_no_pop", max_delta(out, FRAMES) <= d_before_switch * 4.0f + 0.4f);
    for (int s = 0; s < RATES; s++) free(mi[s]);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
