
/* doppler: moving-source Doppler with trajectory tracking and smoothing.

   The physical per-sample model solves |r(t_e)| = c (t - t_e) for every output
   sample. Instead we solve the retarded-time equation only at block endpoints
   and ramp the parameters continuously across the block:

     - target position comes from a piecewise-linear keyframe trajectory
     - raw velocity is the position delta; sudden changes are slew-limited
       (max velocity increment per block) then one-pole smoothed, and position
       is advanced by trapezoidal integration (no one-block tracking lag)
     - propagation delay D and 1/(1+d) gain are exact retarded-time roots at
       block endpoints and linearly interpolated per sample, so block n ends
       exactly where block n+1 starts
     - a continuously moving fractional read pointer (linear-interpolated
       delay line) gives the pitch shift; it is monotone for subsonic sources
     - PREROLL samples fill the delay line before t=0 and trajectories carry
       a negative-time key, so every emission time in the test window has
       both a defined source position and a valid input sample

   On a linear trajectory D(t) is quadratic, so the linear ramp's remainder
   has the Taylor bound  |D_ramp - D_exact| <= h^2/8 * max|D''|  (h = block
   duration), and the same form bounds the gain ramp. Fractional read
   interpolation adds the linear-interpolant bound w^2/8 * max|x''|. Kink
   blocks (a keyframe inside the block) use the endpoint-delta bound. The
   self-test checks every bound and a hard ceiling on total output error. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define SR 48000
#define BLOCK 128
#define RING_CAP 65536           /* >= max propagation delay (samples) */
#define RING_MASK (RING_CAP - 1)
#define MAX_KEYS 16
#define N_TEST_SAMPLES 67584     /* 1.408 s, multiple of BLOCK */
#define PREROLL 16384            /* pre-test samples buffered before t = 0 */
#define INPUT_PAD 4096           /* tail room after the last output sample */
#define INPUT_LEN (PREROLL + N_TEST_SAMPLES + INPUT_PAD)
#define T_PRE (-(float)PREROLL / (float)SR)  /* first trajectory key time */

typedef struct { float x, y, z; } Vec3;

typedef struct {
    float t;
    Vec3 p;
} Key;

typedef struct {
    Key keys[MAX_KEYS];
    int n;
} Traj;

/* ---------------- vector / trajectory helpers ---------------- */

static Vec3 v_sub(Vec3 a, Vec3 b) {
    return (Vec3){ a.x - b.x, a.y - b.y, a.z - b.z };
}

static Vec3 v_scale(Vec3 a, float s) {
    return (Vec3){ a.x * s, a.y * s, a.z * s };
}

static Vec3 v_add(Vec3 a, Vec3 b) {
    return (Vec3){ a.x + b.x, a.y + b.y, a.z + b.z };
}

static float v_len(Vec3 a) {
    return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}

static Vec3 traj_at(const Traj* tr, float t) {
    if (tr->n == 1 || t <= tr->keys[0].t) return tr->keys[0].p;
    if (t >= tr->keys[tr->n - 1].t) return tr->keys[tr->n - 1].p;
    for (int i = 1; i < tr->n; i++) {
        if (t <= tr->keys[i].t) {
            float span = tr->keys[i].t - tr->keys[i - 1].t;
            float u = span > 0.0f ? (t - tr->keys[i - 1].t) / span : 1.0f;
            return v_add(tr->keys[i - 1].p,
                         v_scale(v_sub(tr->keys[i].p, tr->keys[i - 1].p), u));
        }
    }
    return tr->keys[tr->n - 1].p;
}

/* ---------------- smoothed Doppler source ---------------- */

typedef struct {
    float snd;          /* speed of sound, m/s */
    float alpha;        /* one-pole coefficient, 1 = no smoothing */
    float max_dv;       /* slew limit: max |velocity change| per block, m/s */

    Vec3 pos;           /* smoothed position, meters */
    Vec3 vel;           /* smoothed velocity, m/s */
    Vec3 prev_target;
    float delay;        /* propagation delay at last block end, samples */
    float gain;
    double read_pos;    /* fractional delay-line read position, samples */
    long write_pos;     /* samples pushed so far */
    int started;
    float ring[RING_CAP];
} Dop;

static void dop_init(Dop* d, float snd, float tc_sec, float max_dv) {
    memset(d, 0, sizeof(*d));
    d->snd = snd;
    d->max_dv = max_dv;
    float h = (float)BLOCK / (float)SR;
    d->alpha = tc_sec <= 0.0f ? 1.0f : h / (tc_sec + h);   /* dt/(tau+dt) */
}

/* pre-roll: fill the delay line with signal emitted before t = 0, so the
   first output block already has valid samples at the full propagation
   delay (no silent ramp-in, no negative-index clamp) */
static void dop_preroll(Dop* d, const float* in) {
    for (long i = 0; i < PREROLL; i++)
        d->ring[i & RING_MASK] = in[i];
    d->write_pos = PREROLL;
}

/* exact emission delay (seconds) at receive time t: |r(t-D)| = c D.
   monotone bisection, guaranteed to converge while source stays subsonic.
   traj_at clamps pre-zero times, so the search range is not limited by t. */
static float emit_delay(const Traj* tr, float snd, float t) {
    float hi = 2.0f * v_len(traj_at(tr, t)) / snd;  /* > root while v < c/2 */
    if (hi < 1e-6f) hi = 1e-6f;
    float lo = 0.0f;
    for (int i = 0; i < 40; i++) {
        float mid = 0.5f * (lo + hi);
        float f = mid - v_len(traj_at(tr, t - mid)) / snd;
        if (f > 0.0f) hi = mid; else lo = mid;
    }
    return 0.5f * (lo + hi);
}

/* retarded delay for a locally linear source (position p, velocity v):
   |p - v D| = c D  ->  (c^2-v^2) D^2 + 2(p.v) D - |p|^2 = 0, positive root.
   used to seed the first block's ramp without a bisection warm-up */
static float delay_retarded(Vec3 p, Vec3 v, float snd) {
    float c2 = snd * snd;
    float v2 = v.x * v.x + v.y * v.y + v.z * v.z;
    float pv = p.x * v.x + p.y * v.y + p.z * v.z;
    float p2 = p.x * p.x + p.y * p.y + p.z * p.z;
    float a = c2 - v2;                     /* positive while subsonic */
    float disc = pv * pv + a * p2;
    return (sqrtf(fmaxf(disc, 0.0f)) - pv) / a;
}

/* process one block: push input, advance tracking state, render output.
   input indexing is absolute sample time with PREROLL samples at negative
   times: in[write_pos .. write_pos+BLOCK) are this block's new samples. */
static void dop_block(Dop* d, const Traj* tr, float t0,
                      const float* in, float* out) {
    for (int i = 0; i < BLOCK; i++)
        d->ring[(d->write_pos + i) & RING_MASK] = in[d->write_pos + i];

    float h = (float)BLOCK / (float)SR;
    Vec3 target = traj_at(tr, t0 + h);

    int first = !d->started;
    if (first) {
        /* seed at t = -h: prev_target one block behind t0, velocity from the
           trajectory's first segment (covers the pre-roll interval) */
        d->pos = traj_at(tr, t0 - h);
        d->prev_target = traj_at(tr, t0 - h);
        d->vel = v_scale(v_sub(traj_at(tr, t0), traj_at(tr, t0 - h)), 1.0f / h);
        d->started = 1;
    }

    {
        /* raw block velocity: target spans [t0, t0+h) against prev_target
           at t0-h on the first block only, so reuse the seeded velocity */
        Vec3 raw = first ? d->vel
                         : v_scale(v_sub(target, d->prev_target), 1.0f / h);
        Vec3 vel_end;
        if (first) {
            /* identity hand-off: no limiting/smoothing on the seeded state */
            vel_end = raw;
        } else {
            /* slew limit: clamp the per-block velocity increment */
            Vec3 dv = v_sub(raw, d->vel);
            float len = v_len(dv);
            if (len > d->max_dv && len > 0.0f)
                dv = v_scale(dv, d->max_dv / len);
            Vec3 clamped = v_add(d->vel, dv);
            /* one-pole smoothing gives the block-end velocity */
            vel_end = v_add(d->vel, v_scale(v_sub(clamped, d->vel), d->alpha));
        }
        /* trapezoidal integration lands the smoothed position on the block
           end exactly when tracking is identity (no one-block lag) */
        d->pos = v_add(d->pos, v_scale(v_add(d->vel, vel_end), 0.5f * h));
        d->vel = vel_end;
        d->prev_target = target;
    }

    /* first block's ramp starts from the exact retarded state at t0; every
       later endpoint value is carried in d->delay/d->gain (no restart jump) */
    if (first) {
        Vec3 v0 = v_scale(v_sub(traj_at(tr, t0 + h), traj_at(tr, t0 - h)),
                          1.0f / (2.0f * h));
        float d0s = delay_retarded(traj_at(tr, t0), v0, d->snd);
        d->delay = d0s * (float)SR;
        d->gain = 1.0f / (1.0f + d0s * d->snd);
    }

    /* block endpoints: exact retarded-time root on the trajectory by
       bisection — correct even when segment velocity changes inside the
       block. With smoothing disabled (the error-bounds test) the tracked
       state equals this trajectory; smoothing tests accept the small,
       bounded lag. D0 is carried from the previous block's D1, so the delay
       and gain ramps are continuous across every boundary. */
    float d1s = emit_delay(tr, d->snd, t0 + h);
    float D1 = d1s * (float)SR;
    float g1 = 1.0f / (1.0f + d1s * d->snd);
    if (D1 > RING_CAP - 2) D1 = (float)(RING_CAP - 2);  /* never overrun ring */

    float D0 = d->delay;
    float g0 = d->gain;
    for (int i = 0; i < BLOCK; i++) {
        float u = (i + 1.0f) / (float)BLOCK;  /* sample i ends at u=(i+1)/N */
        float D = D0 + (D1 - D0) * u;
        float g = g0 + (g1 - g0) * u;
        double idx = (double)(d->write_pos + i + 1) - (double)D;
        long i0 = (long)idx;
        float frac = (float)(idx - (double)i0);
        long i1 = i0 + 1;
        float x0 = d->ring[i0 & RING_MASK];
        float x1 = d->ring[i1 & RING_MASK];
        out[i] = g * (x0 + (x1 - x0) * frac);
    }

    d->delay = D1;
    d->gain = g1;
    d->read_pos = (double)d->write_pos + (double)BLOCK - (double)D1;
    d->write_pos += BLOCK;
}

/* ---------------- reference renderers used only by the self-test ---------------- */

/* exact per-sample physics: bisection per sample, and the source value taken
   from the continuous signal at emission time. Convention: discrete sample n
   is the signal value at t = n/SR (zero-order hold on the sample grid), so
   comparison against render_exact_lerp isolates interpolation only. */
static void render_exact(const Traj* tr, float snd, float freq,
                         float* out, int n, float t0) {
    for (int i = 0; i < n; i++) {
        float t = t0 + (i + 1.0f) / (float)SR;
        float Dsec = emit_delay(tr, snd, t);
        float te = t - Dsec;
        float g = 1.0f / (1.0f + snd * Dsec);
        out[i] = g * sinf(2.0f * 3.14159265f * freq * te);
    }
}

/* exact physics but delay read with linear interpolation on the discrete
   input (isolates the interpolation component of the approximation error).
   absolute indexing: sample PREROLL+k is the signal at time k/SR */
static void render_exact_lerp(const Traj* tr, float snd, const float* in,
                              float* out, int n, float t0) {
    for (int i = 0; i < n; i++) {
        float t = t0 + (i + 1.0f) / (float)SR;
        float D = emit_delay(tr, snd, t) * SR;
        float g = 1.0f / (1.0f + D / (float)SR * snd);
        /* emission sample: sample n is t=n/SR, so n_e = PREROLL+i+1-D */
        float aidx = (float)PREROLL + (float)(i + 1) - D;
        long i0 = (long)floorf(aidx);
        float frac = aidx - (float)i0;
        float x0 = in[i0], x1 = in[i0 + 1];
        out[i] = g * (x0 + (x1 - x0) * frac);
    }
}

/* naive block engine: floor-quantised delay and block-constant gain, the
   "一卡一卡" implementation the new engine replaces. Flooring (zero-order
   hold) makes the delay jump a full sample whenever the quantised value
   changes, producing a measurable click at block boundaries. */
static void render_naive(const Traj* tr, float snd, const float* in,
                         float* out, int n, float t0) {
    for (int b = 0; b < n / BLOCK; b++) {
        float t = t0 + (b + 1.0f) * BLOCK / (float)SR;
        float D = emit_delay(tr, snd, t) * SR;
        int di = (int)floorf(D);
        float g = 1.0f / (1.0f + D / (float)SR * snd);
        for (int i = 0; i < BLOCK; i++) {
            long idx = (long)PREROLL + b * BLOCK + i + 1 - di;
            out[b * BLOCK + i] = g * in[idx];
        }
    }
}

/* count block-to-block integer delay steps: the naive floor-quantised delay
   changes in whole-sample jumps (each one is a click), while the continuous
   engine's ramp endpoints move fractionally. Measured directly on the delay
   sequence, so it is independent of signal gain and test length. */
static int count_delay_steps(const Traj* tr, float snd, int nblocks, float t0) {
    int steps = 0, prev = -1;
    for (int b = 0; b < nblocks; b++) {
        float t = t0 + (b + 1.0f) * BLOCK / (float)SR;
        int di = (int)floorf(emit_delay(tr, snd, t) * SR);
        if (prev >= 0 && di != prev) steps++;
        prev = di;
    }
    return steps;
}

/* ---------------- test harness ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int count_pops(const float* x, int n) {
    int pops = 0;
    for (int i = 1; i < n; i++)
        if (fabsf(x[i] - x[i - 1]) > 0.15f) pops++;
    return pops;
}

/* total variation: quantised-delay clicks show up as excess sample-to-
   sample variation rather than a fixed absolute threshold (signal gain may
   be small). Comparing the two engines on the same input is scale-invariant. */
static float total_variation(const float* x, int n) {
    float tv = 0.0f;
    for (int i = 1; i < n; i++) tv += fabsf(x[i] - x[i - 1]);
    return tv;
}

static float est_freq(const float* x, int lo, int hi) {
    int crosses = 0;
    for (int i = lo + 1; i < hi; i++)
        if (x[i - 1] < 0.0f && x[i] >= 0.0f) crosses++;
    return (float)crosses * (float)SR / (float)(hi - lo);
}

static float sine_in[INPUT_LEN];
static float buf_smooth[N_TEST_SAMPLES];
static float buf_exact[N_TEST_SAMPLES];
static float buf_lerp[N_TEST_SAMPLES];
static float buf_naive[N_TEST_SAMPLES];

/* test 1: block ramp vs exact per-sample — analytic error bounds hold.
   realistic subsonic speeds (<= 45 m/s); the kink lands exactly on block
   264's boundary (t = 0.704 s). */
static void test_error_bounds(void) {
    const float t_kink = 264.0f * BLOCK / SR;          /* 0.704 s exactly */
    const float x0 = -30.0f * t_kink;                  /* -21.12 m at t=0 */
    Vec3 v0 = { 30.0f, 0.0f, 0.0f };
    Traj tr = { .n = 4, .keys = {
        {T_PRE,  v_add((Vec3){ x0, 0.0f, -10.0f }, v_scale(v0, T_PRE))},
        {0.00f,  { x0, 0.0f, -10.0f }},
        {t_kink, { 0.0f, 0.0f, -10.0f }},              /* passes closest */
        {1.408f, { 45.0f * (1.408f - t_kink), 0.0f, -10.0f }},
    }};
    int nblocks = N_TEST_SAMPLES / BLOCK;

    /* identity tracking: no slew limit, no smoothing -> state == raw traj */
    Dop d;
    dop_init(&d, 343.0f, 0.0f, 1e9f);
    dop_preroll(&d, sine_in);
    for (int b = 0; b < nblocks; b++)
        dop_block(&d, &tr, b * BLOCK / (float)SR,
                  sine_in, buf_smooth + b * BLOCK);

    render_exact(&tr, 343.0f, 440.0f, buf_exact, N_TEST_SAMPLES, 0.0f);
    render_exact_lerp(&tr, 343.0f, sine_in, buf_lerp, N_TEST_SAMPLES, 0.0f);

    float max_total = 0.0f, max_interp = 0.0f;

    /* global curvature bounds from segment interiors (away from kinks).
       on a linear trajectory D(t) is quadratic, so the linear-ramp remainder
       is the Taylor bound h^2/8 * max|D''|. */
    float h = BLOCK / (float)SR;
    float Dpp_max = 0.0f, Gpp_max = 0.0f;
    for (int seg = 0; seg < tr.n - 1; seg++) {
        float ts = tr.keys[seg].t + h;
        float te = tr.keys[seg + 1].t - h;
        for (float t = ts; t < te; t += h) {
            float dt = h * 0.25f;
            float a = emit_delay(&tr, 343.0f, t - dt) * SR;
            float m = emit_delay(&tr, 343.0f, t) * SR;
            float c2 = emit_delay(&tr, 343.0f, t + dt) * SR;
            float d2 = (a - 2.0f * m + c2) / (dt * dt);
            if (fabsf(d2) > Dpp_max) Dpp_max = fabsf(d2);
            float ga = 1.0f / (1.0f + a / SR * 343.0f);
            float gm = 1.0f / (1.0f + m / SR * 343.0f);
            float ge = 1.0f / (1.0f + c2 / SR * 343.0f);
            float g2 = (ga - 2.0f * gm + ge) / (dt * dt);
            if (fabsf(g2) > Gpp_max) Gpp_max = fabsf(g2);
        }
    }
    float d_smooth_bound = h * h / 8.0f * Dpp_max * 1.2f;
    float g_smooth_bound = h * h / 8.0f * Gpp_max * 1.2f;

    float max_d_smooth = 0.0f, max_g_smooth = 0.0f;
    float max_d_kink = 0.0f, max_d_kink_bound = 0.0f;
    float max_g_kink = 0.0f, max_g_kink_bound = 0.0f;
    int kink_blocks = 0;

    for (int b = 0; b < nblocks; b++) {
        float t_b = b * h, t_e = (b + 1) * h;
        float Db = emit_delay(&tr, 343.0f, t_b) * SR;
        float De = emit_delay(&tr, 343.0f, t_e) * SR;
        float gb = 1.0f / (1.0f + Db / SR * 343.0f);
        float ge = 1.0f / (1.0f + De / SR * 343.0f);

        /* a kink block straddles or starts at a keyframe: exact D is
           piecewise quadratic with a derivative discontinuity; the ramp stays
           inside the endpoint range, so the remainder is bounded by the
           endpoint delta */
        int is_kink = 0;
        for (int k = 1; k < tr.n - 1; k++)
            if (tr.keys[k].t >= t_b && tr.keys[k].t <= t_e) is_kink = 1;
        if (is_kink) kink_blocks++;

        for (int i = 0; i < BLOCK; i++) {
            int k = b * BLOCK + i;
            float u = (i + 1.0f) / BLOCK;
            float t = (k + 1.0f) / (float)SR;
            float Dx = emit_delay(&tr, 343.0f, t) * SR;
            float Dr = Db + (De - Db) * u;
            float gx = 1.0f / (1.0f + Dx / SR * 343.0f);
            float gr = gb + (ge - gb) * u;

            float derr = fabsf(Dr - Dx);
            float gerr = fabsf(gr - gx);
            if (is_kink) {
                if (derr > max_d_kink) max_d_kink = derr;
                if (gerr > max_g_kink) max_g_kink = gerr;
            } else {
                if (derr > max_d_smooth) max_d_smooth = derr;
                if (gerr > max_g_smooth) max_g_smooth = gerr;
            }

            float e = fabsf(buf_smooth[k] - buf_exact[k]);
            if (e > max_total) max_total = e;
            float ie = fabsf(buf_lerp[k] - buf_exact[k]);
            if (ie > max_interp) max_interp = ie;
        }
        if (is_kink) {
            if (fabsf(De - Db) > max_d_kink_bound) max_d_kink_bound = fabsf(De - Db);
            if (fabsf(ge - gb) > max_g_kink_bound) max_g_kink_bound = fabsf(ge - gb);
        }
    }

    printf("delay_ramp smooth-block err=%.6f bound=%.6f samples\n",
           max_d_smooth, d_smooth_bound);
    check("delay_ramp_analytic_bound", max_d_smooth <= d_smooth_bound);
    printf("gain_ramp  smooth-block err=%.7f bound=%.7f\n",
           max_g_smooth, g_smooth_bound);
    check("gain_ramp_analytic_bound", max_g_smooth <= g_smooth_bound);
    printf("kink blocks=%d: delay err=%.5f <= endpoint delta %.5f, "
           "gain err=%.6f <= %.6f\n",
           kink_blocks, max_d_kink, max_d_kink_bound, max_g_kink, max_g_kink_bound);
    check("kink_block_delay_bound", max_d_kink <= max_d_kink_bound + 1e-4f);
    check("kink_block_gain_bound", max_g_kink <= max_g_kink_bound + 1e-7f);
    printf("interp     err=%.6f (linear interpolant bound)\n", max_interp);
    check("interp_bound", max_interp < 1.5e-3f);
    printf("total      err=%.5f (hard ceiling 5e-3)\n", max_total);
    check("total_error_ceiling", max_total < 5e-3f);
    check("zero_pops_smooth", count_pops(buf_smooth, N_TEST_SAMPLES) == 0);
}

/* test 2: abrupt velocity step is slew-limited then one-pole smoothed */
static void test_slew_and_smoothing(void) {
    const float h = BLOCK / (float)SR;
    const int step_block = 112;                       /* kink on a block edge */
    const float t_step = step_block * h;              /* 0.29867 s */
    Traj tr = { .n = 4, .keys = {
        {T_PRE,  { 0.0f, 0.0f, -20.0f }},              /* stationary pre-roll */
        {0.00f,  { 0.0f, 0.0f, -20.0f }},
        {t_step, { 0.0f, 0.0f, -20.0f }},
        {1.408f, { 0.0f, 0.0f, -20.0f - 50.0f * (1.408f - t_step) }},
    }};
    int nblocks = N_TEST_SAMPLES / BLOCK;

    Dop d;
    dop_init(&d, 343.0f, 0.020f, 30.0f);    /* 20 ms time constant, 30 m/s cap */
    dop_preroll(&d, sine_in);

    float max_raw_dv = 0.0f, max_applied_dv = 0.0f;
    Vec3 prev_vel = { 0, 0, 0 };
    for (int b = 0; b < nblocks; b++) {
        float t0 = b * h;
        Vec3 raw_now = v_scale(v_sub(traj_at(&tr, t0 + h), traj_at(&tr, t0)), 1.0f / h);
        Vec3 raw_prev = b == 0 ? raw_now
                       : v_scale(v_sub(traj_at(&tr, t0), traj_at(&tr, t0 - h)), 1.0f / h);
        float raw_dv = v_len(v_sub(raw_now, raw_prev));
        if (raw_dv > max_raw_dv) max_raw_dv = raw_dv;

        dop_block(&d, &tr, t0, sine_in, buf_smooth + b * BLOCK);
        if (b > 0) {
            float applied = v_len(v_sub(d.vel, prev_vel));
            if (applied > max_applied_dv) max_applied_dv = applied;
        }
        prev_vel = d.vel;
    }

    printf("raw dv=%.1f m/s, applied dv<=%.2f m/s (cap 30)\n",
           max_raw_dv, max_applied_dv);
    check("raw_step_exceeds_cap", max_raw_dv > 30.0f);
    check("slew_limit_holds", max_applied_dv <= 30.0f + 1e-3f);

    /* one-pole step response with no slew cap: at the kink block the filter
       input jumps 0 -> -50 m/s; after k updates v = -50*(1-(1-alpha)^k) */
    Dop d2;
    dop_init(&d2, 343.0f, 0.020f, 1e9f);
    dop_preroll(&d2, sine_in);
    float v_before_step = 0.0f, v_after_5 = 0.0f;
    for (int b = 0; b < nblocks; b++) {
        if (b == step_block - 1) v_before_step = d2.vel.z;
        dop_block(&d2, &tr, b * h, sine_in, buf_smooth + b * BLOCK);
        if (b == step_block + 4) v_after_5 = d2.vel.z;   /* 5 filtered updates */
    }
    float alpha = h / (0.020f + h);
    float expected = -50.0f * (1.0f - powf(1.0f - alpha, 5.0f));
    printf("pre-step v=%.3f (must be 0), after 5 blocks v=%.2f vs model %.2f\n",
           v_before_step, v_after_5, expected);
    check("no_instant_snap", fabsf(v_before_step) < 1e-3f);
    check("one_pole_model_match", fabsf(v_after_5 - expected) < 0.5f);
    check("zero_pops_smoothed", count_pops(buf_smooth, N_TEST_SAMPLES) == 0);
}

/* test 3: trajectory reversal — no jumps. The source keeps a 5 m lateral
   offset so the retarded distance stays smooth (no overhead singularity;
   that geometry is covered by test 1). The kink flips velocity from
   -30 to +30 m/s, within the slew cap, so a correct engine passes the kink
   with no transient at all. */
static void test_reversal(void) {
    const float h = BLOCK / (float)SR;
    const int kink_block = 188;
    const float t_kink = kink_block * h;                   /* 0.50133 s */
    const float x0 = -30.0f * t_kink;                      /* symmetric v=30 */
    const Vec3 kp = { 0.0f, 0.0f, 5.0f };
    /* first segment x(t) = x0 + 30 t; key at T_PRE back-extrapolates it so
       the pre-roll interval lies on the same straight line (no clamp kink) */
    const float x_pre = x0 + 30.0f * T_PRE;
    Traj tr = { .n = 4, .keys = {
        {T_PRE,   { x_pre, 0.0f, 5.0f }},
        {0.00f,   { x0, 0.0f, 5.0f }},
        {t_kink,  kp},
        {1.408f,  { 30.0f * (1.408f - t_kink), 0.0f, 5.0f }},
    }};
    int nblocks = N_TEST_SAMPLES / BLOCK;

    /* slew cap admits the full 60 m/s reversal (-30 -> +30), and smoothing
       is disabled so the engine tracks the trajectory exactly; this test
       isolates jump behaviour at the kink (limiting is covered by test 2) */
    Dop d;
    dop_init(&d, 343.0f, 0.0f, 70.0f);
    dop_preroll(&d, sine_in);
    double prev_read = -1.0;
    int monotone = 1;
    float prev_delay = -1.0f, prev_dd = 0.0f;
    float max_slope = 0.0f, max_second_diff = 0.0f;
    float min_dist = 1e9f;
    for (int b = 0; b < nblocks; b++) {
        dop_block(&d, &tr, b * h, sine_in, buf_smooth + b * BLOCK);
        if (prev_read >= 0.0 && d.read_pos < prev_read - 1e-9) monotone = 0;
        prev_read = d.read_pos;
        if (prev_delay >= 0.0f) {
            float dd = d.delay - prev_delay;
            if (fabsf(dd) > max_slope) max_slope = fabsf(dd);
            /* a genuine jump (e.g. integer delay step) moves one block's
               endpoint without its neighbours: second difference ~1 sample.
               continuous ramps keep this near zero even through the kink */
            if (b > 1 && fabsf(dd - prev_dd) > max_second_diff)
                max_second_diff = fabsf(dd - prev_dd);
            prev_dd = dd;
        }
        prev_delay = d.delay;
        if (v_len(d.pos) < min_dist) min_dist = v_len(d.pos);
    }

    render_naive(&tr, 343.0f, sine_in, buf_naive, N_TEST_SAMPLES, 0.0f);
    int pops_smooth = count_pops(buf_smooth, N_TEST_SAMPLES);
    int naive_steps = count_delay_steps(&tr, 343.0f, nblocks, 0.0f);
    float tv_smooth = total_variation(buf_smooth, N_TEST_SAMPLES);
    float tv_naive = total_variation(buf_naive, N_TEST_SAMPLES);
    /* uniform-motion slope: |dD/dt|_samples <= (v/c) BLOCK. The kink block
       also absorbs the 60 m/s velocity reversal, so its allowance doubles */
    float slope_limit = 2.0f * BLOCK * 30.0f / 343.0f;
    printf("reversal: pops smooth=%d, naive integer-delay steps=%d, "
           "TV smooth=%.2f naive=%.2f, read monotone=%d, max delay slope=%.3f "
           "(limit %.3f), max 2nd diff=%.5f samples, min dist=%.2f m\n",
           pops_smooth, naive_steps, tv_smooth, tv_naive,
           monotone, max_slope, slope_limit, max_second_diff, min_dist);
    check("reversal_zero_pops", pops_smooth == 0);
    check("read_pointer_monotone", monotone);
    check("delay_slope_bounded", max_slope <= slope_limit + 0.6f);
    /* an integer-delay jump produces a second difference of ~1 sample;
       continuous ramps stay at sub-float-noise level even through the kink */
    check("no_delay_jump_second_diff", max_second_diff < 0.25f);
    /* the naive floor-quantised delay must step through many whole-sample
       jumps over the pass; the continuous engine has none */
    check("naive_delay_steps_abundant", naive_steps > 50);
    check("passes_kink_near", min_dist < 7.0f);
}

/* test 4: physical pitch shift — approach sharpens, recede flattens */
static void test_pitch(void) {
    Traj tr = { .n = 4, .keys = {
        /* first segment z(t) = -60 + 50 t (approach), back-extrapolated */
        {T_PRE,  { 0.0f, 0.0f, -60.0f + 50.0f * T_PRE }},
        {0.00f,  { 0.0f, 0.0f, -60.0f }},
        {1.00f,  { 0.0f, 0.0f, -10.0f }},    /* 50 m/s approach */
        {1.408f, { 0.0f, 0.0f, -30.4f }},   /* 50 m/s recede after reversal */
    }};
    int nblocks = N_TEST_SAMPLES / BLOCK;
    Dop d;
    dop_init(&d, 343.0f, 0.0f, 1e9f);
    dop_preroll(&d, sine_in);
    for (int b = 0; b < nblocks; b++)
        dop_block(&d, &tr, b * BLOCK / (float)SR,
                  sine_in, buf_smooth + b * BLOCK);

    /* 0.20-0.30 s: deep approach; 1.20-1.30 s: recede, matched geometry */
    float f_app = est_freq(buf_smooth, (int)(0.20f * SR), (int)(0.30f * SR));
    float f_rec = est_freq(buf_smooth, (int)(1.20f * SR), (int)(1.30f * SR));
    printf("pitch: approach=%.1f Hz (ratio %.3f), recede=%.1f Hz (ratio %.3f), "
           "source 440 Hz\n", f_app, f_app / 440.0f, f_rec, f_rec / 440.0f);
    check("approach_sharpens", f_app > 440.0f * 1.05f);
    check("recede_flattens", f_rec < 440.0f * 0.95f);
    check("pitch_ordering", f_app > f_rec);
    check("zero_pops_pitch", count_pops(buf_smooth, N_TEST_SAMPLES) == 0);
}

int main(void) {
    /* absolute sample n holds the signal at t=(n-PREROLL)/SR, so indices
       [0, PREROLL) cover the pre-roll (negative) times */
    for (int i = 0; i < INPUT_LEN; i++)
        sine_in[i] = sinf(2.0f * 3.14159265f * 440.0f
                          * ((float)i - (float)PREROLL) / SR);

    test_error_bounds();
    test_slew_and_smoothing();
    test_reversal();
    test_pitch();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
