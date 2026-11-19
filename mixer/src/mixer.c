
/* mixer: multi-bus audio mixing with per-bus gain, sidechain ducking,
   soft clipping and gain automation. All buffers are interleaved stereo.

   Automation: every bus gain can be driven by a breakpoint curve made of
   linear or exponential segments. Each segment precomputes its slope once
   when appended; during processing a block is entered at the exact curve
   value (seg_val) and then advanced one sample at a time with a single
   multiply-add per bus -- additive for linear segments, multiplicative in
   the log domain for exponential ones. Block/segment boundaries are always
   reasserted to the exact curve value, so per-sample output matches an
   explicit per-sample curve evaluation within float tolerance and no error
   accumulates across blocks. Replacing a curve at runtime starts the new
   curve at the current gain, so switching curves never produces a step. */
#define _POSIX_C_SOURCE 199309L
#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#endif
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define MAX_SRC 8
#define FRAMES 128
#define SR 48000
#define BUS 3
#define MAX_AUTO_SEG 32

typedef struct {
    int bus;        /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
} Source;

typedef enum { AUTO_LINEAR = 0, AUTO_EXP } AutoCurve;

typedef struct {
    double t0, t1;          /* seconds */
    float v0, v1;           /* gains; exponential segments require v0,v1 > 0 */
    AutoCurve curve;
    /* precomputed once at append time so the hot path takes no log():
       exp segments: g(t) = v0 * exp(log_slope * (t - t0))
       linear:       g(t) = v0 + lin_slope * (t - t0) */
    float log_slope;
    float lin_slope;
} AutoSeg;

typedef struct {
    AutoSeg segs[MAX_AUTO_SEG];
    int count;
    double t_end;           /* time of the last breakpoint */
    float v_end;            /* value of the last breakpoint / hold value */
    int hint;               /* segment cursor for block rendering */
} Auto;

typedef struct {
    float gain[BUS];        /* constant gains used while automation is off */
    Auto autom[BUS];
    unsigned char auto_on[BUS];
    float duck_depth;       /* 0..1, applied to bus 0 when voice is active */
    int duck_enabled;
} Mixer;

static void auto_init(Auto* a, float gain) {
    memset(a, 0, sizeof(*a));
    a->v_end = gain;
}

/* Append a segment running from the current end breakpoint to (t1, v1). */
static int auto_append(Auto* a, double t1, float v1, AutoCurve c) {
    if (a->count >= MAX_AUTO_SEG) return -1;
    if (!(t1 > a->t_end)) return -1;
    if (v1 < 0.0f || a->v_end < 0.0f) return -1;
    if (c == AUTO_EXP && (v1 <= 0.0f || a->v_end <= 0.0f)) return -1;
    AutoSeg* s = &a->segs[a->count++];
    s->t0 = a->t_end; s->t1 = t1;
    s->v0 = a->v_end; s->v1 = v1; s->curve = c;
    double span = t1 - a->t_end;
    if (c == AUTO_EXP) {
        s->log_slope = (float)(log(v1 / (double)a->v_end) / span);
        s->lin_slope = 0.0f;
    } else {
        s->log_slope = 0.0f;
        s->lin_slope = (float)((v1 - a->v_end) / span);
    }
    a->t_end = t1; a->v_end = v1;
    a->hint = 0;
    return 0;
}

/* Closed-form value inside a known segment; no segment scan. Exponential
   segments use the precomputed log slope, so this is one expf at most. */
static float seg_val(const AutoSeg* s, double t) {
    double dt = t - s->t0;
    if (dt <= 0.0) return s->v0;
    if (t >= s->t1) return s->v1;
    if (s->curve == AUTO_EXP)
        return s->v0 * expf(s->log_slope * (float)dt);
    return s->v0 + s->lin_slope * (float)dt;
}

/* Exact curve value at time t; holds the end values outside the range. */
static float auto_eval(const Auto* a, double t) {
    if (a->count == 0) return a->v_end;
    const AutoSeg* f = &a->segs[0];
    if (t <= f->t0) return f->v0;
    for (int i = 0; i < a->count; i++) {
        const AutoSeg* s = &a->segs[i];
        if (t < s->t1 || i == a->count - 1) return seg_val(s, t);
    }
    return a->v_end;
}

/* Replace the curve at time "now" with a ramp toward target, starting from
   whatever gain the old curve had at that instant: no step on switch. */
static void auto_ramp_from_now(Auto* a, double now, double dur,
                               float target, AutoCurve c) {
    float v0 = auto_eval(a, now);
    auto_init(a, v0);
    a->t_end = now;
    if (dur > 0.0) {
        if (auto_append(a, now + dur, target, c) != 0 && c == AUTO_EXP)
            auto_append(a, now + dur, target, AUTO_LINEAR);  /* v0 may be 0 */
    } else {
        a->v_end = target;
    }
}

static void emit_ramp(float* g, int n, float va, float vb, AutoCurve c) {
    if (n == 1) { g[0] = va; return; }
    if (c == AUTO_EXP) {
        /* geometric ramp in the log domain: one expf for the whole block;
           the last sample is pinned to vb so block/segment joins stay exact */
        float step = expf((logf(vb) - logf(va)) / (float)(n - 1));
        float v = va;
        for (int j = 0; j < n; j++) { g[j] = v; v *= step; }
    } else {
        float step = (vb - va) / (float)(n - 1);
        for (int j = 0; j < n; j++) g[j] = va + step * (float)j;
    }
    g[n - 1] = vb;   /* exact endpoint keeps segment joins step-free */
}

/* Sample the curve for a block starting at t0 and write one gain per sample
   frame. Sample times that fall outside every segment hold the end value. */
static void auto_render(Auto* a, double t0, int frames, int sr, float* g) {
    if (a->count == 0) {
        for (int k = 0; k < frames; k++) g[k] = a->v_end;
        return;
    }
    double dt = 1.0 / (double)sr;
    int last = a->count - 1;
    if (t0 >= a->segs[last].t1) {
        float v = a->segs[last].v1;
        for (int k = 0; k < frames; k++) g[k] = v;
        return;
    }
    int i = a->hint;
    if (i > last) i = last;
    while (i > 0 && t0 < a->segs[i].t0) i--;
    while (i < last && t0 >= a->segs[i].t1) i++;
    a->hint = i;

    int k = 0;
    if (t0 < a->segs[0].t0) {       /* leading hold */
        int n = 0;
        while (k + n < frames && t0 + (k + n) * dt < a->segs[0].t0) n++;
        for (int j = 0; j < n; j++) g[k + j] = a->segs[0].v0;
        k += n;
    }
    while (k < frames) {
        double tk = t0 + k * dt;
        while (i <= last && tk >= a->segs[i].t1) i++;
        if (i > last) {             /* trailing hold */
            float v = a->segs[last].v1;
            for (int j = k; j < frames; j++) g[j] = v;
            return;
        }
        const AutoSeg* s = &a->segs[i];
        int n = 1;                  /* samples whose time stays in this seg */
        while (k + n < frames && t0 + (k + n) * dt < s->t1) n++;
        /* the segment is already known: evaluate directly, no segment scan */
        float va = seg_val(s, tk);
        float vb = seg_val(s, tk + (n - 1) * dt);
        emit_ramp(g + k, n, va, vb, s->curve);
        k += n;
    }
}

/* Block-local ramp state. prepare() fills the first ramp; gain_at() walks
   the curve one sample at a time in the final output loop, so per-sample
   gain costs one multiply-add and never touches a materialized array. */
typedef struct {
    float v;            /* gain at the current sample */
    float step;         /* per-sample additive (linear) or multiplicative
                           (exp) delta, derived from the whole segment span */
    int remain;         /* samples left in the current ramp, incl. current */
    AutoCurve curve;
    int seg;            /* current segment index */
    double t0;          /* block start time */
    double dt;
} GainState;

/* Per-sample exponential ramp step. Over ordinary segments the per-sample
   log delta is far below 1, so a degree-4 Taylor polynomial is enough that
   the error accumulated across a block stays under 1e-6 relative; exact
   segment values are reasserted at each boundary anyway. If a segment is so
   short or steep that the delta is large (rare, pathological automation),
   fall back to the correctly-rounded libm expf rather than extrapolate. */
static inline float exp_step(float x) {
    if (fabsf(x) > 0.25f) return expf(x);
    float x2 = x * x;
    return 1.0f + x + x2 * 0.5f + x2 * x * (1.0f / 6.0f)
                 + x2 * x2 * (1.0f / 24.0f);
}

/* Per-sample advance for one segment, starting at the exact value va. The
   per-sample step comes straight from the segment's precomputed slope: no
   logf in the hot path, and exp_step handles the tiny per-sample log
   delta. Exact curve values are reasserted at every block/segment boundary
   via seg_val(), so drift cannot accumulate across blocks. */
static void ramp_between(GainState* gs, const AutoSeg* s, int n,
                         float va, double dt) {
    gs->curve = s->curve;
    gs->v = va;
    if (n <= 1) { gs->step = 0.0f; return; }
    if (s->curve == AUTO_EXP)
        gs->step = exp_step(s->log_slope * (float)dt);
    else
        gs->step = s->lin_slope * (float)dt;
}

/* Samples in [t0, s->t1): the sample at exactly s->t1 starts the next
   segment, the same ownership rule as auto_render(). */
static int samples_in_seg(const AutoSeg* s, double t0, double dt, int frames) {
    int n = 1;
    while (n < frames && t0 + n * dt < s->t1) n++;
    return n;
}

/* Begin the block and compute its first ramp; outside any segment the value
   just holds. Endpoints are evaluated exactly with seg_val(), so two ramps
   meeting on a sample share one value: no step at block/segment joins. */
static void auto_prepare(Auto* a, double t0, int frames, int sr,
                         GainState* gs) {
    gs->t0 = t0;
    gs->dt = 1.0 / (double)sr;
    gs->seg = 0;
    gs->curve = AUTO_LINEAR;
    gs->step = 0.0f;

    if (a->count == 0) {
        gs->v = a->v_end; gs->remain = frames; return;
    }
    int last = a->count - 1;
    if (t0 < a->segs[0].t0) {                  /* leading hold */
        int n = 0;
        while (n < frames && t0 + n * gs->dt < a->segs[0].t0) n++;
        if (n > 0) { gs->v = a->segs[0].v0; gs->remain = n; return; }
        /* n == 0: t0 is exactly the first segment start, fall through */
    }
    int i = a->hint;
    if (i > last) i = last;
    while (i > 0 && t0 < a->segs[i].t0) i--;
    while (i < last && t0 >= a->segs[i].t1) i++;
    a->hint = i;
    gs->seg = i;

    if (i > last) {                            /* trailing hold */
        gs->v = a->segs[last].v1; gs->remain = frames; return;
    }
    const AutoSeg* s = &a->segs[i];
    int n = samples_in_seg(s, t0, gs->dt, frames);
    float va = seg_val(s, t0);
    gs->remain = n;
    ramp_between(gs, s, n, va, gs->dt);
}

/* Gain for the current sample, then advance one sample. */
static inline float auto_gain_at(Auto* a, int frames, int frame_idx,
                                 GainState* gs) {
    float g = gs->v;
    if (frame_idx + 1 >= frames) return g;      /* nothing follows */
    if (gs->remain > 1) {
        gs->remain--;
        gs->curve == AUTO_EXP ? (gs->v *= gs->step)
                              : (gs->v += gs->step);
        return g;
    }
    if (a->count == 0) { gs->remain = frames - frame_idx - 1; return g; }
    /* current sample ended a ramp: start the next segment at its first sample */
    double tnxt = gs->t0 + (frame_idx + 1) * gs->dt;
    int last = a->count - 1;
    int i = gs->seg + 1;
    if (i > last) {
        gs->v = a->segs[last].v1;
        gs->step = 0.0f; gs->remain = frames - frame_idx - 1;
        gs->curve = AUTO_LINEAR; gs->seg = last;
        return g;
    }
    gs->seg = i;
    const AutoSeg* s = &a->segs[i];
    int n = samples_in_seg(s, tnxt, gs->dt, frames - frame_idx - 1);
    float va = seg_val(s, tnxt);
    gs->remain = n;
    ramp_between(gs, s, n, va, gs->dt);
    return g;
}

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

static void mixer_init(Mixer* mx) {
    memset(mx, 0, sizeof(*mx));
    for (int b = 0; b < BUS; b++) {
        mx->gain[b] = 1.0f;
        auto_init(&mx->autom[b], 1.0f);
    }
}

static void mix(Mixer* mx, const Source* srcs, int n,
                const float* const* in, float* out, int frames,
                double time, int sr) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    float bus[BUS][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b >= BUS) continue;
        for (int i = 0; i < frames * 2; i++)
            bus[b][i] += in[s][i] * srcs[s].gain;
    }

    GainState gs[BUS];
    for (int b = 0; b < BUS; b++) {
        if (mx->auto_on[b])
            auto_prepare(&mx->autom[b], time, frames, sr, &gs[b]);
        else {
            gs[b].v = mx->gain[b]; gs[b].step = 0.0f;
            gs[b].remain = frames; gs[b].curve = AUTO_LINEAR;
            gs[b].seg = 0; gs[b].t0 = time; gs[b].dt = 1.0 / (double)sr;
        }
    }

    float dg = 1.0f;
    if (mx->duck_enabled) {
        /* voice bus energy drives ducking of the sfx bus */
        float e = 0.0f;
        for (int i = 0; i < frames * 2; i++) e += fabsf(bus[2][i]);
        e /= (float)(frames * 2);
        if (e > 0.01f) dg = 1.0f - mx->duck_depth;
    }

    for (int i = 0; i < frames; i++) {
        float g0 = auto_gain_at(&mx->autom[0], frames, i, &gs[0]) * dg;
        float g1 = auto_gain_at(&mx->autom[1], frames, i, &gs[1]);
        float g2 = auto_gain_at(&mx->autom[2], frames, i, &gs[2]);
        out[i * 2]     = soft_clip(bus[0][i * 2] * g0
                                 + bus[1][i * 2] * g1
                                 + bus[2][i * 2] * g2);
        out[i * 2 + 1] = soft_clip(bus[0][i * 2 + 1] * g0
                                 + bus[1][i * 2 + 1] * g1
                                 + bus[2][i * 2 + 1] * g2);
    }
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

/* Render whole blocks over [t0, t0 + total) and compare every sample with
   an explicit per-sample evaluation of the same curve. */
static int curve_matches_eval(Auto* a, double t0, int total, int frames,
                              int sr, float rel_tol) {
    float g[FRAMES];
    int done = 0;
    while (done < total) {
        int n = total - done < frames ? total - done : frames;
        double bt = t0 + (double)done / sr;
        auto_render(a, bt, n, sr, g);
        for (int k = 0; k < n; k++) {
            float ref = auto_eval(a, bt + (double)k / sr);
            float tol = rel_tol * fmaxf(1.0f, fabsf(ref));
            if (fabsf(g[k] - ref) > tol) return 0;
        }
        done += n;
    }
    return 1;
}

/* Cost is measured in reference cycles, not wall time: a block is well
   under a microsecond, where scheduler and timer granularity dominate a
   clock_gettime reading. rdtscp is a serializing cycle read; on other
   architectures fall back to monotonic nanoseconds. */
static unsigned long long cycles(void) {
#if defined(__x86_64__) || defined(__i386__)
    unsigned aux;
    _mm_lfence();
    return (unsigned long long)__rdtscp(&aux);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)ts.tv_sec * 1000000000ULL
         + (unsigned long long)ts.tv_nsec;
#endif
}

int main(void) {
    const int sr = SR;
    float sfx[FRAMES * 2], music[FRAMES * 2], voice[FRAMES * 2], out[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.0f;
    }
    const float* in[3] = { sfx, music, voice };
    Source srcs[3] = { {0, 1.0f}, {1, 1.0f}, {2, 1.0f} };
    Mixer mx;
    mixer_init(&mx);
    mx.duck_depth = 0.5f;
    mx.duck_enabled = 1;

    mix(&mx, srcs, 3, in, out, FRAMES, 0.0, sr);
    check("sum_ok", fabsf(out[0] - 0.75f) < 1e-5f);
    check("no_clip_when_low", peak(out, FRAMES * 2) < 0.99f);

    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.3f;
    mix(&mx, srcs, 3, in, out, FRAMES, 0.0, sr);
    float with_duck = out[0];

    mx.duck_enabled = 0;
    mix(&mx, srcs, 3, in, out, FRAMES, 0.0, sr);
    float without_duck = out[0];
    check("duck_lowers_sfx", with_duck < without_duck);

    mx.duck_enabled = 1;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.0f;
    mix(&mx, srcs, 3, in, out, FRAMES, 0.0, sr);
    check("duck_idle_passthrough", fabsf(out[0] - 0.75f) < 1e-5f);

    for (int s = 0; s < 3; s++) srcs[s].gain = 4.0f;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.5f;
    mix(&mx, srcs, 3, in, out, FRAMES, 0.0, sr);
    check("soft_clip_bounds", peak(out, FRAMES * 2) <= 1.0f + 1e-6f);

    srcs[0].bus = 7;   /* invalid bus must be ignored */
    mix(&mx, srcs, 3, in, out, FRAMES, 0.0, sr);
    check("bad_bus_ignored", peak(out, FRAMES * 2) <= 1.0f + 1e-6f);
    srcs[0].bus = 0;
    for (int s = 0; s < 3; s++) srcs[s].gain = 1.0f;

    /* ---- automation curves -------------------------------------------- */

    Auto a;
    auto_init(&a, 0.25f);
    check("append_linear", auto_append(&a, 0.25, 0.9f, AUTO_LINEAR) == 0);
    check("append_exp", auto_append(&a, 0.3333, 0.3f, AUTO_EXP) == 0);
    check("append_linear2", auto_append(&a, 0.75, 1.2f, AUTO_LINEAR) == 0);
    check("append_exp2", auto_append(&a, 1.0, 0.6f, AUTO_EXP) == 0);
    check("exp_rejects_zero", auto_append(&a, 1.2, 0.0f, AUTO_EXP) == -1);
    check("exp_rejects_neg_start",
          (auto_init(&a, 0.0f), auto_append(&a, 0.1, 0.5f, AUTO_EXP)) == -1);
    check("reject_backwards_time",
          (auto_init(&a, 1.0f), auto_append(&a, 0.0, 0.5f, AUTO_LINEAR)) == -1);

    auto_init(&a, 0.25f);
    auto_append(&a, 0.25, 0.9f, AUTO_LINEAR);
    auto_append(&a, 0.3333, 0.3f, AUTO_EXP);
    auto_append(&a, 0.75, 1.2f, AUTO_LINEAR);
    auto_append(&a, 1.0, 0.6f, AUTO_EXP);

    /* block rendering matches per-sample curve evaluation, including the
       linear->exponential joins and clamp/hold regions at both ends */
    check("linear_exp_matches_curve_128",
          curve_matches_eval(&a, -0.1, (int)(1.3 * sr), FRAMES, sr, 2e-5f));
    check("linear_exp_matches_curve_37",
          curve_matches_eval(&a, -0.1, (int)(1.3 * sr), 37, sr, 2e-5f));

    /* pure exponential curve: geometric midpoint and per-sample match */
    Auto e;
    auto_init(&e, 0.5f);
    auto_append(&e, 1.0, 2.0f, AUTO_EXP);
    check("exp_geometric_midpoint", fabsf(auto_eval(&e, 0.5) - 1.0f) < 1e-6f);
    check("exp_matches_curve",
          curve_matches_eval(&e, 0.0, sr, FRAMES, sr, 2e-5f));

    /* pure linear curve is exact, not just close */
    Auto ln;
    auto_init(&ln, 0.2f);
    auto_append(&ln, 1.0, 0.8f, AUTO_LINEAR);
    check("linear_exact", curve_matches_eval(&ln, 0.0, sr, FRAMES, sr, 1e-7f));

    /* block size independence: 128-frame and 16-frame blockings agree */
    enum { total = SR };
    static float big[total], small[total];
    float g[FRAMES];
    for (int k = 0; k < total; k += FRAMES) {
        auto_render(&a, (double)k / sr, FRAMES, sr, g);
        for (int j = 0; j < FRAMES; j++) big[k + j] = g[j];
    }
    for (int k = 0; k < total; k += 16) {
        auto_render(&a, (double)k / sr, 16, sr, g);
        for (int j = 0; j < 16; j++) small[k + j] = g[j];
    }
    int independent = 1;
    for (int k = 0; k < total; k++) {
        float tol = 2e-5f * fmaxf(1.0f, fabsf(big[k]));
        if (fabsf(big[k] - small[k]) > tol) { independent = 0; break; }
    }
    check("block_size_independent", independent);

    /* hold outside the segment range */
    check("hold_before", fabsf(auto_eval(&a, -5.0) - 0.25f) < 1e-7f);
    check("hold_after", fabsf(auto_eval(&a, 5.0) - 0.6f) < 1e-7f);

    /* runtime curve switch starts at the current gain: no step */
    Auto sw;
    auto_init(&sw, 1.0f);
    auto_append(&sw, 1.0, 0.4f, AUTO_LINEAR);
    double tblock = FRAMES / (double)sr;
    float g1[FRAMES], g2[FRAMES];
    auto_render(&sw, 0.0, FRAMES, sr, g1);
    float v_at_switch = auto_eval(&sw, tblock);
    auto_ramp_from_now(&sw, tblock, 0.5, 1.6f, AUTO_EXP);
    auto_render(&sw, tblock, FRAMES, sr, g2);
    check("switch_no_step", g2[0] == v_at_switch);
    float normal_step = fabsf(g1[FRAMES - 1] - g1[FRAMES - 2]);
    check("switch_no_extra_jump",
          fabsf(g2[0] - g1[FRAMES - 1]) <= normal_step * 1.5f + 1e-7f);
    check("switch_reaches_target",
          fabsf(auto_eval(&sw, tblock + 0.5) - 1.6f) < 1e-6f);

    /* automated gain actually drives the mix output */
    Mixer m2;
    mixer_init(&m2);
    auto_init(&m2.autom[0], 1.0f);
    auto_append(&m2.autom[0], FRAMES / (double)sr, 0.0f, AUTO_LINEAR);
    m2.auto_on[0] = 1;
    Source one[1] = { {0, 1.0f} };
    const float* one_in[1] = { sfx };
    mix(&m2, one, 1, one_in, out, FRAMES, 0.0, sr);
    check("output_follows_gain_start", fabsf(out[0] - 0.5f) < 1e-6f);
    /* last sample of the block is at u = 127/128; the 0 endpoint lands on
       the first sample of the next block and then holds */
    check("output_follows_gain_within_block",
          fabsf(out[(FRAMES - 1) * 2] - 0.5f / FRAMES) < 2e-6f);
    mix(&m2, one, 1, one_in, out, FRAMES, FRAMES / (double)sr, sr);
    check("output_follows_gain_hold_end", fabsf(out[0]) < 1e-6f);

    /* fused mix path: every output sample must equal the exact curve value
       applied to a unit input, across many blocks and segment/block joins */
    Mixer m3;
    mixer_init(&m3);
    m3.duck_enabled = 0;
    m3.auto_on[0] = 1;
    {
        Auto* au = &m3.autom[0];
        auto_init(au, 0.4f);
        auto_append(au, 0.21, 0.95f, AUTO_EXP);
        auto_append(au, 0.47, 0.25f, AUTO_LINEAR);
        auto_append(au, 0.83, 0.7f, AUTO_EXP);
        auto_append(au, 1.15, 0.5f, AUTO_LINEAR);
    }
    /* 0.5 input: even at the largest gain below (1.6) the product is 0.8,
       under soft_clip's 0.9 threshold, so these tests exercise gain only */
    static float unit[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) unit[i] = 0.5f;
    const float* unit_in[1] = { unit };
    Source unit_src[1] = { {0, 1.0f} };
    int fused_ok = 1;
    for (int blk = 0; blk < 40 && fused_ok; blk++) {
        double bt = blk * FRAMES / (double)sr;
        mix(&m3, unit_src, 1, unit_in, out, FRAMES, bt, sr);
        for (int k = 0; k < FRAMES; k++) {
            float ref = 0.5f * auto_eval(&m3.autom[0], bt + (double)k / sr);
            if (fabsf(out[k * 2] - ref) > 3e-5f * fmaxf(1.0f, ref) ||
                fabsf(out[k * 2 + 1] - ref) > 3e-5f * fmaxf(1.0f, ref)) {
                fused_ok = 0; break;
            }
        }
    }
    check("fused_output_matches_curve", fused_ok);

    /* fused path, runtime curve switch: the sample at the switch must equal
       the old curve's value there (no step), then follow the new curve */
    Mixer m4;
    mixer_init(&m4);
    m4.duck_enabled = 0;
    m4.auto_on[0] = 1;
    auto_init(&m4.autom[0], 1.0f);
    auto_append(&m4.autom[0], 1.0, 0.4f, AUTO_LINEAR);
    double sw_t = 3 * FRAMES / (double)sr;
    for (int blk = 0; blk < 3; blk++)                       /* run up to switch */
        mix(&m4, unit_src, 1, unit_in, out, FRAMES,
            blk * FRAMES / (double)sr, sr);
    float old_here = auto_eval(&m4.autom[0], sw_t);
    auto_ramp_from_now(&m4.autom[0], sw_t, 0.5, 1.6f, AUTO_EXP);
    mix(&m4, unit_src, 1, unit_in, out, FRAMES, sw_t, sr);
    check("fused_switch_no_step", fabsf(out[0] - 0.5f * old_here) < 1e-6f);
    check("fused_switch_follows_new",
          fabsf(out[(FRAMES - 1) * 2]
                - 0.5f * auto_eval(&m4.autom[0],
                            sw_t + (FRAMES - 1) / (double)sr)) < 3e-5f);

    /* steep, very short exponential segments force the exp_step() libm
       fallback and put several boundaries inside single blocks; the fused
       path must still track the exact per-sample curve */
    Mixer m5;
    mixer_init(&m5);
    m5.duck_enabled = 0;
    m5.auto_on[0] = 1;
    {
        Auto* au = &m5.autom[0];
        auto_init(au, 0.01f);
        double t = 0.0;
        float v = 0.01f;
        for (int j = 0; j < 12; j++) {        /* ~1 ms segments, 32:1 swings */
            t += 0.0011;
            v = (j & 1) ? 0.32f : 0.01f;
            auto_append(au, t, v, AUTO_EXP);
        }
    }
    int steep_ok = 1;
    for (int blk = 0; blk < 6 && steep_ok; blk++) {
        double bt = blk * FRAMES / (double)sr;
        mix(&m5, unit_src, 1, unit_in, out, FRAMES, bt, sr);
        for (int k = 0; k < FRAMES; k++) {
            float ref = 0.5f * auto_eval(&m5.autom[0], bt + (double)k / sr);
            float tol = 1e-4f * fmaxf(1.0f, ref);
            if (fabsf(out[k * 2] - ref) > tol ||
                fabsf(out[k * 2 + 1] - ref) > tol) { steep_ok = 0; break; }
        }
    }
    check("steep_exp_matches_curve", steep_ok);

    /* ---- overhead budget: automation must cost <= 20% of plain mixing.
       Both configurations are built once outside timing and run over
       identical inputs at the same level, so soft-clip/ducking take the
       same path in both; only the gain logic differs. Cost is read in
       reference cycles (see cycles()), taking the minimum over trials so a
       single descheduled interval cannot inflate the result. */
    enum { REP = 100000, TRIALS = 9 };
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.0f;
    Mixer mp, ma;
    mixer_init(&mp);                       /* plain: constant per-bus gains */
    mixer_init(&ma);
    ma.duck_enabled = mp.duck_enabled = 0;
    for (int b = 0; b < BUS; b++) {
        float base = 0.9f + 0.02f * b;
        mp.gain[b] = base;
        ma.auto_on[b] = 1;
        Auto* au = &ma.autom[b];
        auto_init(au, base);
        /* alternating linear/exponential ramps; gains stay below 1 so both
           configurations remain in the same soft-clip branch */
        for (int j = 1; j <= 30; j++)
            auto_append(au, (double)j,
                        base - 0.05f * (j % 3),
                        (j & 1) ? AUTO_LINEAR : AUTO_EXP);
    }
    /* warm up: instruction cache, branch predictors and CPU frequency */
    for (int rep = 0; rep < 10000; rep++) {
        mix(&ma, srcs, 3, in, out, FRAMES, rep * tblock, sr);
        mix(&mp, srcs, 3, in, out, FRAMES, rep * tblock, sr);
    }
    unsigned long long best_p = ~0ULL, best_a = ~0ULL;
    for (int trial = 0; trial < TRIALS; trial++) {
        unsigned long long c0, c1, c2, c3;
        double tt;
        if (trial & 1) {                  /* alternate order each trial */
            tt = 0.0; c0 = cycles();
            for (int rep = 0; rep < REP; rep++) {
                mix(&ma, srcs, 3, in, out, FRAMES, tt, sr); tt += tblock;
            }
            c1 = cycles(); tt = 0.0; c2 = cycles();
            for (int rep = 0; rep < REP; rep++) {
                mix(&mp, srcs, 3, in, out, FRAMES, tt, sr); tt += tblock;
            }
            c3 = cycles();
            if (c1 - c0 < best_a) best_a = c1 - c0;
            if (c3 - c2 < best_p) best_p = c3 - c2;
        } else {
            tt = 0.0; c0 = cycles();
            for (int rep = 0; rep < REP; rep++) {
                mix(&mp, srcs, 3, in, out, FRAMES, tt, sr); tt += tblock;
            }
            c1 = cycles(); tt = 0.0; c2 = cycles();
            for (int rep = 0; rep < REP; rep++) {
                mix(&ma, srcs, 3, in, out, FRAMES, tt, sr); tt += tblock;
            }
            c3 = cycles();
            if (c1 - c0 < best_p) best_p = c1 - c0;
            if (c3 - c2 < best_a) best_a = c3 - c2;
        }
    }
    double plain_c = (double)best_p / REP;
    double auto_c = (double)best_a / REP;
    double overhead = (auto_c - plain_c) / plain_c;
    printf("auto_overhead=%.3f (plain=%.0f cyc/block, auto=%.0f cyc/block)\n",
           overhead, plain_c, auto_c);
    check("auto_overhead_budget", overhead <= 0.20);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
