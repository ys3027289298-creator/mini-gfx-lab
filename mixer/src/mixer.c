
/* mixer: multi-bus audio mixing with per-bus gain, ducking, soft clipping and
   curve-driven multi-bus automation. All buffers are interleaved stereo.
 *
 * Automation replaces the old one-parameter scheme.  A single curve carries
 * any number of targets; each target maps the same normalized curve value
 * onto one bus with its own transfer shape and gain/dB mapping.  All targets
 * are evaluated from the same curve value at the same instant, so several
 * bus faders move together.
 *
 * Switching curves snapshots the currently applied gains and smoothsteps
 * from the snapshot to the incoming curve, so the output never jumps.  Once
 * the transition finishes the driven gains are exactly the incoming curve's
 * mapped values - the same numbers a caller would set by hand.
 *
 * Curve values outside [0,1] are clamped before mapping and every clamped
 * evaluation is recorded (total counter plus a ring of recent events).
 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_SRC 8
#define FRAMES 128
#define BUSES 3

typedef struct {
    int bus;        /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
} Source;

typedef struct {
    float gain[BUSES];
    float duck_depth;    /* 0..1, applied to bus 0 when voice is active */
    int duck_enabled;
} Mixer;

/* ------------------------------------------------------- curve automation */

typedef enum {
    AUTO_LINEAR = 0,
    AUTO_SMOOTH,        /* smoothstep */
    AUTO_EASE_IN,
    AUTO_EASE_OUT,
    AUTO_STEP
} AutoEase;

typedef enum {
    AUTO_UNIT_LINEAR = 0,   /* mapping output is a linear gain */
    AUTO_UNIT_DB            /* mapping output is a gain in dBFS */
} AutoUnit;

typedef enum {
    AUTO_SHAPE_LINEAR = 0,
    AUTO_SHAPE_SQRT,
    AUTO_SHAPE_SQUARE,
    AUTO_SHAPE_COS,         /* cos(pi/2 * u), together with SIN gives equal power */
    AUTO_SHAPE_SIN
} AutoShape;

#define AUTO_MAX_KEYS 16
#define AUTO_MAX_TARGETS 8
#define AUTO_CLAMP_LOG 8
#define AUTO_HALF_PI 1.57079632679f

typedef struct {
    float t;
    float v;                /* normalized parameter, clamped by the automation */
    AutoEase ease;
} AutoKey;

typedef struct {
    AutoKey keys[AUTO_MAX_KEYS];
    int count;
    float duration;
    int loop;
} AutoCurve;

typedef struct {
    int bus;
    AutoUnit unit;
    AutoShape shape;
    float lo;               /* value at normalized curve 0 */
    float hi;               /* value at normalized curve 1 */
} AutoTarget;

typedef struct {
    float time;             /* automation time of the evaluation */
    float raw;              /* value returned by the curve */
    float clamped;          /* value after clamping to [0,1] */
    int targets;            /* targets that used the clamped value */
} AutoClampEvent;

typedef struct {
    AutoCurve curve;
    int has_curve;

    AutoTarget targets[AUTO_MAX_TARGETS];
    int target_count;

    float time;             /* time since the active curve was selected */
    float gain[BUSES];      /* last applied gain of each driven bus */

    float from[BUSES];      /* snapshot taken when a switch starts */
    float trans_elapsed;
    float trans_dur;
    int transitioning;

    unsigned long clamp_count;
    AutoClampEvent clamp_log[AUTO_CLAMP_LOG];
    int clamp_log_count;
    int clamp_log_head;     /* next slot to write */
} Automation;

static float auto_ease_apply(AutoEase e, float u) {
    switch (e) {
    case AUTO_SMOOTH: return u * u * (3.0f - 2.0f * u);
    case AUTO_EASE_IN: return u * u;
    case AUTO_EASE_OUT: return 1.0f - (1.0f - u) * (1.0f - u);
    case AUTO_STEP: return u < 1.0f ? 0.0f : 1.0f;
    case AUTO_LINEAR:
    default: return u;
    }
}

static void curve_init(AutoCurve* c, float duration, int loop) {
    memset(c, 0, sizeof(*c));
    c->duration = duration;
    c->loop = loop;
}

static int curve_add(AutoCurve* c, float t, float v, AutoEase e) {
    if (c->count >= AUTO_MAX_KEYS) return -1;
    if (t < 0.0f || (c->duration > 0.0f && t > c->duration)) return -1;
    if (c->count > 0 && t < c->keys[c->count - 1].t) return -1;
    AutoKey* k = &c->keys[c->count++];
    k->t = t;
    k->v = v;           /* deliberately unchecked: clamping is observable */
    k->ease = e;
    return 0;
}

static float curve_eval(const AutoCurve* c, float time) {
    if (c->count == 0) return 0.0f;
    if (c->duration > 0.0f) {
        if (c->loop) {
            time = fmodf(time, c->duration);
            if (time < 0.0f) time += c->duration;
        } else {
            if (time < 0.0f) time = 0.0f;
            if (time > c->duration) time = c->duration;
        }
    }
    if (time <= c->keys[0].t) return c->keys[0].v;
    for (int i = 1; i < c->count; i++) {
        const AutoKey* a = &c->keys[i - 1];
        const AutoKey* b = &c->keys[i];
        if (time <= b->t) {
            float span = b->t - a->t;
            float u = span > 0.0f ? (time - a->t) / span : 1.0f;
            return a->v + (b->v - a->v) * auto_ease_apply(a->ease, u);
        }
    }
    return c->keys[c->count - 1].v;
}

static float auto_shape_apply(AutoShape s, float u) {
    switch (s) {
    case AUTO_SHAPE_SQRT: return sqrtf(u);
    case AUTO_SHAPE_SQUARE: return u * u;
    case AUTO_SHAPE_COS: return cosf(u * AUTO_HALF_PI);
    case AUTO_SHAPE_SIN: return sinf(u * AUTO_HALF_PI);
    case AUTO_SHAPE_LINEAR:
    default: return u;
    }
}

/* Map a normalized curve value onto a target's bus.  The shape warps the
   normalized value first; lo/hi then place it in the target's domain. */
static float target_map(const AutoTarget* t, float u) {
    float f = auto_shape_apply(t->shape, u);
    float v = t->lo + (t->hi - t->lo) * f;
    if (t->unit == AUTO_UNIT_DB) return powf(10.0f, v / 20.0f);
    return v > 0.0f ? v : 0.0f;
}

static void auto_init(Automation* a) {
    memset(a, 0, sizeof(*a));
    for (int b = 0; b < BUSES; b++) {
        a->gain[b] = 1.0f;
        a->from[b] = 1.0f;
    }
}

static int auto_add_target(Automation* a, int bus, AutoUnit unit,
                           AutoShape shape, float lo, float hi) {
    if (a->target_count >= AUTO_MAX_TARGETS) return -1;
    if (bus < 0 || bus >= BUSES) return -1;
    for (int i = 0; i < a->target_count; i++)
        if (a->targets[i].bus == bus) return -1;   /* one mapping per bus */
    AutoTarget* t = &a->targets[a->target_count++];
    t->bus = bus;
    t->unit = unit;
    t->shape = shape;
    t->lo = lo;
    t->hi = hi;
    a->gain[bus] = 1.0f;
    a->from[bus] = 1.0f;
    return 0;
}

static void auto_log_clamp(Automation* a, float raw, float clamped) {
    a->clamp_count++;
    AutoClampEvent* e = &a->clamp_log[a->clamp_log_head];
    e->time = a->time;
    e->raw = raw;
    e->clamped = clamped;
    e->targets = a->target_count;
    a->clamp_log_head = (a->clamp_log_head + 1) % AUTO_CLAMP_LOG;
    if (a->clamp_log_count < AUTO_CLAMP_LOG) a->clamp_log_count++;
}

static const AutoClampEvent* auto_last_clamp(const Automation* a) {
    if (a->clamp_log_count == 0) return NULL;
    int idx = (a->clamp_log_head + AUTO_CLAMP_LOG - 1) % AUTO_CLAMP_LOG;
    return &a->clamp_log[idx];
}

static void auto_reset_clamp_log(Automation* a) {
    a->clamp_count = 0;
    a->clamp_log_count = 0;
    a->clamp_log_head = 0;
}

static int auto_targeted(const Automation* a, int bus) {
    for (int i = 0; i < a->target_count; i++)
        if (a->targets[i].bus == bus) return 1;
    return 0;
}

static float auto_gain(const Automation* a, int bus) {
    if (bus < 0 || bus >= BUSES) return 1.0f;
    return a->gain[bus];
}

/* Advance the clock, evaluate the active curve once, clamp it, map it onto
   every target and blend the result with the switch snapshot. */
static void auto_tick(Automation* a, float dt) {
    if (dt < 0.0f) dt = 0.0f;
    a->time += dt;

    float base[BUSES];
    for (int b = 0; b < BUSES; b++) base[b] = 1.0f;

    if (a->has_curve) {
        float raw = curve_eval(&a->curve, a->time);
        float u = raw;
        if (!(u >= 0.0f)) u = 0.0f;      /* also catches NaN */
        if (u > 1.0f) u = 1.0f;
        if (!(raw == u)) auto_log_clamp(a, raw, u);
        for (int i = 0; i < a->target_count; i++) {
            const AutoTarget* t = &a->targets[i];
            base[t->bus] = target_map(t, u);
        }
    }

    if (a->transitioning) {
        a->trans_elapsed += dt;
        float p = a->trans_dur > 0.0f ? a->trans_elapsed / a->trans_dur : 1.0f;
        int done = p >= 1.0f;
        if (done) p = 1.0f;
        float w = p * p * (3.0f - 2.0f * p);
        for (int b = 0; b < BUSES; b++) {
            if (!auto_targeted(a, b)) continue;
            /* Land exactly on the mapped value, with no 1-ulp residue, so the
               settled result matches a hand-set fader bit for bit. */
            a->gain[b] = done ? base[b] : a->from[b] + (base[b] - a->from[b]) * w;
        }
        if (done) a->transitioning = 0;
    } else {
        for (int b = 0; b < BUSES; b++)
            if (auto_targeted(a, b)) a->gain[b] = base[b];
    }
}

/* Select a curve.  The gains currently applied are snapshotted first, so a
   switch mid-transition continues from the blended value, then the new
   curve plays from time 0 and crossfades in over transition_s seconds. */
static void auto_set_curve(Automation* a, const AutoCurve* c, float transition_s) {
    float current[BUSES];
    for (int b = 0; b < BUSES; b++) current[b] = a->gain[b];

    a->curve = *c;
    a->has_curve = 1;
    a->time = 0.0f;
    for (int b = 0; b < BUSES; b++) a->from[b] = current[b];
    a->trans_elapsed = 0.0f;
    a->trans_dur = transition_s > 0.0f ? transition_s : 0.0f;
    a->transitioning = a->trans_dur > 0.0f;
    auto_tick(a, 0.0f);
}

/* Automation drives the bus faders: this is equivalent to setting each
   mixer gain by hand to the mapped value. */
static void auto_apply(const Automation* a, Mixer* mx) {
    for (int i = 0; i < a->target_count; i++) {
        int b = a->targets[i].bus;
        mx->gain[b] = a->gain[b];
    }
}

/* ------------------------------------------------------------- mixing core */

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

static void mix(Mixer* mx, const Source* srcs, int n,
                const float* const* in, float* out, int frames) {
    float bus[BUSES][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b >= BUSES) continue;
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

static float peak(const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(b[i]) > m) m = fabsf(b[i]);
    return m;
}

/* ------------------------------------------------------------------ tests */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void test_curve(void) {
    AutoCurve c;
    curve_init(&c, 2.0f, 0);
    check("curve_add_ok", curve_add(&c, 0.0f, 0.0f, AUTO_LINEAR) == 0
                       && curve_add(&c, 1.0f, 1.0f, AUTO_LINEAR) == 0
                       && curve_add(&c, 2.0f, 0.0f, AUTO_LINEAR) == 0);
    check("curve_mid_up", fabsf(curve_eval(&c, 0.5f) - 0.5f) < 1e-6f);
    check("curve_mid_down", fabsf(curve_eval(&c, 1.5f) - 0.5f) < 1e-6f);
    check("curve_before", fabsf(curve_eval(&c, -1.0f) - 0.0f) < 1e-6f);
    check("curve_after", fabsf(curve_eval(&c, 3.0f) - 0.0f) < 1e-6f);

    AutoCurve sm;
    curve_init(&sm, 1.0f, 0);
    curve_add(&sm, 0.0f, 0.0f, AUTO_SMOOTH);
    curve_add(&sm, 1.0f, 1.0f, AUTO_LINEAR);
    check("curve_smooth_quarter", fabsf(curve_eval(&sm, 0.25f) - 0.15625f) < 1e-6f);

    AutoCurve lp;
    curve_init(&lp, 1.0f, 1);
    curve_add(&lp, 0.0f, 0.0f, AUTO_LINEAR);
    curve_add(&lp, 1.0f, 1.0f, AUTO_LINEAR);
    check("curve_loop_wrap", fabsf(curve_eval(&lp, 1.5f) - 0.5f) < 1e-5f);
    check("curve_loop_negative", fabsf(curve_eval(&lp, -0.5f) - 0.5f) < 1e-5f);

    AutoCurve bad;
    curve_init(&bad, 1.0f, 0);
    check("curve_reject_unsorted",
          (curve_add(&bad, 0.5f, 1.0f, AUTO_LINEAR),
           curve_add(&bad, 0.2f, 2.0f, AUTO_LINEAR)) == -1);
    check("curve_reject_range", curve_add(&bad, 2.0f, 3.0f, AUTO_LINEAR) == -1);
}

static void test_mapping(void) {
    AutoTarget lin = { 0, AUTO_UNIT_LINEAR, AUTO_SHAPE_LINEAR, 1.0f, 0.0f };
    check("map_linear_start", fabsf(target_map(&lin, 0.0f) - 1.0f) < 1e-6f);
    check("map_linear_mid", fabsf(target_map(&lin, 0.5f) - 0.5f) < 1e-6f);
    check("map_linear_end", fabsf(target_map(&lin, 1.0f) - 0.0f) < 1e-6f);

    AutoTarget db = { 2, AUTO_UNIT_DB, AUTO_SHAPE_LINEAR, -24.0f, 0.0f };
    check("map_db_low", fabsf(target_map(&db, 0.0f) - 0.0630957f) < 1e-5f);
    check("map_db_high", fabsf(target_map(&db, 1.0f) - 1.0f) < 1e-6f);

    AutoTarget co = { 0, AUTO_UNIT_LINEAR, AUTO_SHAPE_COS, 0.0f, 1.0f };
    AutoTarget si = { 1, AUTO_UNIT_LINEAR, AUTO_SHAPE_SIN, 0.0f, 1.0f };
    float worst = 0.0f;
    for (int i = 0; i <= 10; i++) {
        float u = (float)i / 10.0f;
        float a = target_map(&co, u), b = target_map(&si, u);
        float err = fabsf(a * a + b * b - 1.0f);
        if (err > worst) worst = err;
    }
    printf("equal_power_worst=%.9f\n", worst);
    check("map_equal_power", worst < 1e-6f);
}

static void test_clamp_log(void) {
    Automation a;
    auto_init(&a);
    check("target_add", auto_add_target(&a, 0, AUTO_UNIT_LINEAR,
                                        AUTO_SHAPE_LINEAR, 0.0f, 1.0f) == 0);
    check("target_reject_dup", auto_add_target(&a, 0, AUTO_UNIT_LINEAR,
                                               AUTO_SHAPE_LINEAR, 0.0f, 1.0f) == -1);
    check("target_reject_bus", auto_add_target(&a, 7, AUTO_UNIT_LINEAR,
                                               AUTO_SHAPE_LINEAR, 0.0f, 1.0f) == -1);

    AutoCurve c;
    curve_init(&c, 1.0f, 0);
    curve_add(&c, 0.0f, -0.5f, AUTO_LINEAR);
    curve_add(&c, 1.0f, 1.5f, AUTO_LINEAR);
    auto_set_curve(&a, &c, 0.0f);          /* evaluates t = 0 immediately */

    const AutoClampEvent* e = auto_last_clamp(&a);
    check("clamp_count_first", a.clamp_count == 1);
    check("clamp_raw_neg", e && fabsf(e->raw + 0.5f) < 1e-6f
                             && fabsf(e->clamped) < 1e-6f && e->targets == 1);
    check("clamp_gain_floor", fabsf(auto_gain(&a, 0)) < 1e-6f);

    auto_tick(&a, 0.5f);
    check("clamp_normal_no_log", a.clamp_count == 1);
    check("clamp_normal_gain", fabsf(auto_gain(&a, 0) - 0.5f) < 1e-6f);

    auto_tick(&a, 0.5f);
    e = auto_last_clamp(&a);
    check("clamp_count_second", a.clamp_count == 2);
    check("clamp_raw_pos", e && fabsf(e->raw - 1.5f) < 1e-6f
                             && fabsf(e->clamped - 1.0f) < 1e-6f);
    check("clamp_gain_ceiling", fabsf(auto_gain(&a, 0) - 1.0f) < 1e-6f);
    check("clamp_event_time", e && fabsf(e->time - 1.0f) < 1e-6f);

    auto_reset_clamp_log(&a);
    check("clamp_reset", a.clamp_count == 0 && auto_last_clamp(&a) == NULL);
}

static void test_switch_transition(void) {
    Automation a;
    auto_init(&a);
    auto_add_target(&a, 0, AUTO_UNIT_LINEAR, AUTO_SHAPE_LINEAR, 1.0f, 0.0f);

    AutoCurve A, B;
    curve_init(&A, 0.0f, 0);
    curve_add(&A, 0.0f, 0.0f, AUTO_LINEAR);
    curve_init(&B, 0.0f, 0);
    curve_add(&B, 0.0f, 1.0f, AUTO_LINEAR);

    auto_set_curve(&a, &A, 0.0f);
    check("switch_start_gain", fabsf(auto_gain(&a, 0) - 1.0f) < 1e-6f);

    float before = auto_gain(&a, 0);
    auto_set_curve(&a, &B, 0.1f);
    float after = auto_gain(&a, 0);
    check("switch_continuity", fabsf(after - before) < 1e-6f);

    auto_tick(&a, 0.001f);
    float early = auto_gain(&a, 0);
    check("switch_early_moves", early < 0.9999f && early > 0.999f);
    check("switch_not_jump", early > 0.9f);

    auto_tick(&a, 0.099f);
    check("switch_end_gain", fabsf(auto_gain(&a, 0)) < 1e-6f);
    check("switch_end_done", a.transitioning == 0);

    /* switching again while the transition is half-way must continue from
       the blended gain, not from the outgoing curve or the destination */
    auto_set_curve(&a, &A, 0.0f);
    auto_set_curve(&a, &B, 0.1f);
    auto_tick(&a, 0.05f);
    float mid = auto_gain(&a, 0);
    check("switch_mid_half", fabsf(mid - 0.5f) < 1e-5f);
    auto_set_curve(&a, &A, 0.1f);
    check("switch_mid_continuity", fabsf(auto_gain(&a, 0) - mid) < 1e-6f);
    auto_tick(&a, 0.1f);
    check("switch_mid_end", fabsf(auto_gain(&a, 0) - 1.0f) < 1e-6f);
}

#define SR_F 48000.0f
#define PI_F 3.14159265358979f
#define BLOCKS 600

/* Automation must produce the same output as setting each fader by hand to
   the mapped value for every block. */
static void test_manual_equivalence(void) {
    Automation a;
    auto_init(&a);
    const float lo1 = 0.1f, hi1 = 0.9f;
    auto_add_target(&a, 0, AUTO_UNIT_LINEAR, AUTO_SHAPE_LINEAR, 1.0f, 0.0f);
    auto_add_target(&a, 1, AUTO_UNIT_LINEAR, AUTO_SHAPE_LINEAR, lo1, hi1);

    AutoCurve c;
    curve_init(&c, 1.0f, 0);
    curve_add(&c, 0.0f, 0.0f, AUTO_LINEAR);
    curve_add(&c, 1.0f, 1.0f, AUTO_LINEAR);
    auto_set_curve(&a, &c, 0.0f);

    Mixer auto_mx = { { 1.0f, 1.0f, 0.0f }, 0.5f, 0 };
    Mixer man_mx  = { { 1.0f, 1.0f, 0.0f }, 0.5f, 0 };
    Source srcs[2] = { { 0, 1.0f }, { 1, 1.0f } };

    float worst = 0.0f;
    for (int blk = 0; blk < BLOCKS; blk++) {
        float sfx[FRAMES * 2], music[FRAMES * 2], oa[FRAMES * 2], om[FRAMES * 2];
        for (int i = 0; i < FRAMES; i++) {
            int n = blk * FRAMES + i;
            float s0 = 0.20f * sinf(2.0f * PI_F * 220.0f * (float)n / SR_F);
            float s1 = 0.15f * sinf(2.0f * PI_F * 317.0f * (float)n / SR_F + 0.7f);
            sfx[2 * i + 0] = sfx[2 * i + 1] = s0;
            music[2 * i + 0] = music[2 * i + 1] = s1;
        }

        auto_tick(&a, (float)FRAMES / SR_F);
        auto_apply(&a, &auto_mx);

        float u = a.time < 1.0f ? a.time : 1.0f;
        man_mx.gain[0] = 1.0f + (0.0f - 1.0f) * u;
        man_mx.gain[1] = lo1 + (hi1 - lo1) * u;

        const float* in[2] = { sfx, music };
        mix(&auto_mx, srcs, 2, in, oa, FRAMES);
        mix(&man_mx, srcs, 2, in, om, FRAMES);
        for (int i = 0; i < FRAMES * 2; i++) {
            float d = fabsf(oa[i] - om[i]);
            if (d > worst) worst = d;
        }
    }
    printf("manual_equiv_worst=%.9f\n", worst);
    check("manual_equiv_ok", worst < 1e-7f);
}

/* One curve drives two buses as an equal-power crossfade.  Both bus signals
   carry equal power, so linked output energy stays flat while the curve
   sweeps; anything above 1 dB would mean the linkage is not complementary. */
static void test_equal_power_energy(void) {
    Automation a;
    auto_init(&a);
    auto_add_target(&a, 0, AUTO_UNIT_LINEAR, AUTO_SHAPE_COS, 0.0f, 1.0f);
    auto_add_target(&a, 1, AUTO_UNIT_LINEAR, AUTO_SHAPE_SIN, 0.0f, 1.0f);

    AutoCurve c;
    curve_init(&c, 2.0f, 0);
    curve_add(&c, 0.0f, 0.0f, AUTO_LINEAR);
    curve_add(&c, 2.0f, 1.0f, AUTO_LINEAR);
    auto_set_curve(&a, &c, 0.0f);

    Mixer mx = { { 1.0f, 1.0f, 0.0f }, 0.5f, 0 };
    Source srcs[2] = { { 0, 1.0f }, { 1, 1.0f } };

    enum { WIN_FRAMES = 4096, TOTAL_FRAMES = 48000 };
    double win_e = 0.0;
    int win_n = 0, windows = 0;
    float min_db = 1e9f, max_db = -1e9f;
    double sum_db = 0.0;

    for (int off = 0; off + FRAMES <= TOTAL_FRAMES; off += FRAMES) {
        float sfx[FRAMES * 2], music[FRAMES * 2], out[FRAMES * 2];
        for (int i = 0; i < FRAMES; i++) {
            int n = off + i;
            float s0 = 0.283f * sinf(2.0f * PI_F * 440.0f * (float)n / SR_F);
            float s1 = 0.283f * sinf(2.0f * PI_F * 613.0f * (float)n / SR_F + 1.0f);
            sfx[2 * i + 0] = sfx[2 * i + 1] = s0;
            music[2 * i + 0] = music[2 * i + 1] = s1;
        }

        auto_tick(&a, (float)FRAMES / SR_F);
        auto_apply(&a, &mx);
        const float* in[2] = { sfx, music };
        mix(&mx, srcs, 2, in, out, FRAMES);

        for (int i = 0; i < FRAMES; i++) {
            win_e += (double)out[2 * i] * out[2 * i]
                   + (double)out[2 * i + 1] * out[2 * i + 1];
            win_n += 2;
        }
        if (win_n >= WIN_FRAMES * 2) {
            float db = 20.0f * (float)log10(sqrt(win_e / (double)win_n));
            if (db < min_db) min_db = db;
            if (db > max_db) max_db = db;
            sum_db += db;
            windows++;
            win_e = 0.0;
            win_n = 0;
        }
    }
    float mean_db = windows > 0 ? (float)(sum_db / windows) : 0.0f;
    printf("energy_windows=%d min_db=%.3f max_db=%.3f mean_db=%.3f spread=%.3f\n",
           windows, min_db, max_db, mean_db, max_db - min_db);
    check("energy_windows_enough", windows >= 8);
    check("energy_flat", max_db - min_db < 1.0f);
    check("energy_absolute", fabsf(mean_db - (-13.979f)) < 0.5f);
}

int main(void) {
    /* legacy behaviour: summing and ducking still work with static faders */
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

    test_curve();
    test_mapping();
    test_clamp_log();
    test_switch_transition();
    test_manual_equivalence();
    test_equal_power_energy();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
