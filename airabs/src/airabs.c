
/* airabs: ISO 9613-1 atmospheric air absorption for distant sources.
   Per-octave-band attenuation driven by distance, temperature and humidity,
   combined with inverse-distance geometric attenuation. The heavy ISO physics
   runs once per climate in airabs_climate_set; the per-frame path
   (airabs_update) is one geometric divide plus eight branchless polynomial
   fast-exp evaluations, and steady-state frames (distance within 1 cm) are
   served from a cache at near-zero cost. An optional one-pole air shelf
   (airabs_apply) gives the high-frequency roll-off a real filter path, its
   coefficients solved analytically and lazily only when a block is filtered.

   Coefficient table comparison: airabs_table_dev() checks the computed
   attenuation against the ISO 9613-1 Table 1 reference row and reports the
   signed relative deviation so the physics can be regression-tested. */
#define _GNU_SOURCE
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define NBANDS 8

/* return codes */
#define AIRABS_OK 0
#define AIRABS_E_PARAM (-1)   /* out-of-range or non-finite input */
#define AIRABS_E_STATE (-2)   /* struct used before valid init */

/* octave-band center frequencies, Hz */
static const float BAND_HZ[NBANDS] = {
    63.0f, 125.0f, 250.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f, 8000.0f
};

/* ISO 9613-1 Table 1, attenuation dB/km, 20 C, atmospheric pressure 101.325 kPa */
typedef struct { float rh; float a[NBANDS]; } TableRow;
static const TableRow TABLE_20C[3] = {
    { 30.0f, { 0.192f, 0.615f, 1.42f, 2.52f, 5.01f, 14.1f, 48.5f, 166.0f } },
    { 50.0f, { 0.123f, 0.445f, 1.32f, 2.73f, 4.66f, 9.86f, 29.4f, 104.0f } },
    { 70.0f, { 0.0897f, 0.339f, 1.13f, 2.80f, 4.98f, 9.02f, 22.9f, 76.6f } },
};

/* validated input envelope: wide enough to cover Table 1 conditions and
   normal game-world weather, tight enough to keep the ISO formulas sane. */
#define T_MIN (-20.0f)   /* deg C */
#define T_MAX (50.0f)
#define RH_MIN (0.0f)    /* percent */
#define RH_MAX (100.0f)
#define P_MIN (50.0f)    /* kPa */
#define P_MAX (200.0f)
#define D_MAX (10000.0f) /* meters */

typedef struct {
    float t_c, rh, p_kpa;
    float alpha[NBANDS];          /* dB/km, ISO 9613-1 eq. 1 (x1000) */
    float nepers[NBANDS];         /* nepers per metre: dB/m * ln(10)/20 */
    int valid;
} AirClimate;

typedef struct {
    float distance;               /* meters */
    float last_distance;          /* update cache: skip recompute when ~same */
    float nepers[NBANDS];         /* cached per-metre snapshot of the climate */
    float air_db[NBANDS];         /* air-only attenuation over distance, dB */
    float gain[NBANDS];           /* combined linear gain per band */
    float shelf_c, shelf_g;       /* one-pole air shelf state parameters */
    float z1_l, z1_r;             /* shelf filter memory */
    int shelf_dirty;              /* shelf redesign deferred to airabs_apply */
    int valid;
} AirModel;

static int is_finite(float v) { return isfinite(v); }

static int range_ok(float v, float lo, float hi) {
    return is_finite(v) && v >= lo && v <= hi;
}

/* expf replacement for the per-frame path. The Taylor reciprocal is only
   accurate for small arguments, so the input is scaled by a fixed 1/8 (the
   audible domain x <= 10 lands in [0,1.25]), the 5th-order reciprocal is
   evaluated there, and three squarings restore exp(-x). Branchless, every
   term positive and well-conditioned in float, no integer conversion or
   transcendental. Relative error stays under 1.5% over [0,10] (~-87 dB;
   beyond that it clamps to silence) - self-checked in main. */
static float fast_exp_small(float x) {
    float p = 1.0f + x * (1.0f + x * (0.5f +
            x * (0.16666667f + x * (0.04166667f + x * 0.00833333f))));
    return 1.0f / p;
}
static float fast_exp_neg(float x) {
    if (x <= 0.0f) return 1.0f;
    if (x > 10.0f) return 0.0f;
    float q = fast_exp_small(x * 0.125f);   /* exp(-x/8) */
    q *= q;                                  /* exp(-x/4) */
    q *= q;                                  /* exp(-x/2) */
    q *= q;                                  /* exp(-x)   */
    return q;
}

/* sin(pi * 4000 / 48000): shelf geometry depends only on sample rate, so it
   is computed once, never on the per-frame path. */
#define SHELF_S 0.25881904510252074f   /* sin(pi/12) */

/* ISO 9613-1:1993 equation (1) returns dB per metre. Table 1 values are
   quoted in dB/km, so callers store alpha_dbm * 1000 for table comparison.
   t_c: Celsius, rh: percent relative humidity, p_kpa: kilopascal. */
static float iso9613_alpha_dbm(float f_hz, float t_c, float rh, float p_kpa) {
    const float t_ref = 293.15f;            /* 20 C reference temperature, K */
    float t_kel = t_c + 273.15f;
    float T = t_kel / t_ref;
    float pr = p_kpa / 101.325f;            /* pressure ratio vs reference */

    /* molar concentration of water vapour, percent */
    float p_sat_kpa = 0.61078f * expf(17.27f * t_c / (t_c + 237.3f));
    float h = rh * p_sat_kpa / p_kpa;

    /* oxygen and nitrogen relaxation frequencies, Hz (eqs. 3 and 4) */
    float fr_o = pr * (24.0f + 4.04e4f * h * (0.02f + h) / (0.391f + h));
    float fr_n = (pr / sqrtf(T)) *
                 (9.0f + 280.0f * h * expf(-4.17f * (powf(T, -1.0f / 3.0f) - 1.0f)));

    float f2 = f_hz * f_hz;
    float term_n = 0.1068f * expf(-3352.0f / t_kel) /
                   (fr_n + f2 / fr_n);
    float term_o = 0.01275f * expf(-2239.1f / t_kel) /
                   (fr_o + f2 / fr_o);
    float cl = 1.84e-11f * (1.0f / pr) * sqrtf(T);   /* classical loss term */

    return 8.686f * f2 * (cl + term_n + term_o);
}

/* compute and cache the eight-band climate coefficients.
   Repeated calls with an unchanged climate are nearly free. */
int airabs_climate_set(AirClimate* c, float t_c, float rh, float p_kpa) {
    if (!c) return AIRABS_E_PARAM;
    if (!range_ok(t_c, T_MIN, T_MAX) ||
        !range_ok(rh, RH_MIN, RH_MAX) ||
        !range_ok(p_kpa, P_MIN, P_MAX))
        return AIRABS_E_PARAM;

    if (c->valid &&
        fabsf(c->t_c - t_c) < 1e-4f &&
        fabsf(c->rh - rh) < 1e-4f &&
        fabsf(c->p_kpa - p_kpa) < 1e-4f)
        return AIRABS_OK;

    c->t_c = t_c; c->rh = rh; c->p_kpa = p_kpa;
    for (int b = 0; b < NBANDS; b++) {
        float dbm = iso9613_alpha_dbm(BAND_HZ[b], t_c, rh, p_kpa);
        c->alpha[b] = dbm * 1000.0f;            /* dB/km, Table 1 units */
        c->nepers[b] = dbm * 0.1151292546f;     /* ln(10)/20 */
    }
    c->valid = 1;
    return AIRABS_OK;
}

/* build per-band air attenuation and combine it with inverse-distance
   geometric attenuation (1/(1+d), matching the spatial module). Distance is
   validated first, so a bad distance is never reported as a state error.
   Distance changes below DIST_EPS reuse the cached gains: sub-centimetre
   motion is inaudible, and the hysteresis bounds the worst-case recompute. */
#define DIST_EPS 0.01f
void airabs_shelf_design(AirModel* m);
int airabs_update(AirModel* m, const AirClimate* c, float distance) {
    if (!range_ok(distance, 0.0f, D_MAX)) return AIRABS_E_PARAM;
    if (!m || !c || !c->valid) return AIRABS_E_STATE;
    if (m->valid && fabsf(m->last_distance - distance) < DIST_EPS)
        return AIRABS_OK;

    m->distance = distance;
    m->last_distance = distance;
    memcpy(m->nepers, (void*)c->nepers, sizeof(m->nepers));
    /* hot path: one geometric divide, eight multiplies and eight fast_exp
       calls (pure float polynomial, no transcendental). Steady-state frames
       hit the distance cache above and skip all of this. The shelf redesign
       is deferred to airabs_apply. */
    float geo_gain = 1.0f / (1.0f + distance);
    for (int b = 0; b < NBANDS; b++) {
        float xb = c->nepers[b] * distance;             /* nepers over path */
        m->air_db[b] = xb * 8.685889638f;               /* nepers -> dB */
        m->gain[b] = geo_gain * fast_exp_neg(xb);
    }
    m->shelf_dirty = 1;
    m->valid = 1;
    return AIRABS_OK;
}

/* inverse-distance geometric attenuation in dB, computed only when asked. */
float airabs_geo_db(const AirModel* m) {
    if (!m || !m->valid) return 0.0f;
    return 20.0f * log10f(1.0f + m->distance);
}

/* design the one-pole air shelf analytically from the band air losses.
   Runs lazily, only when a block is filtered and the distance has changed,
   so it is off the gain[] hot path. Cost is one sqrtf (plus two fast exps).
   One-pole LP power response at w is c^2 / (c^2 + 4(1-c)^2 sin^2(w/2));
   setting it equal to (g_hi/g_lo)^2 at 4 kHz gives, with
   k = 4 sin^2(w/2) r^2 / (1-r^2), the cancellation-safe rationalised form
   c = k / (k + sqrt(k^2 + k)) (equivalent to sqrt(k^2+k) - k). */
void airabs_shelf_design(AirModel* m) {
    float g_hi = fast_exp_neg(m->nepers[6] * m->distance);   /* 4 kHz */
    float g_lo = fast_exp_neg(m->nepers[0] * m->distance);   /* 63 Hz */
    m->shelf_g = g_lo;
    float r2 = (g_hi * g_hi) / (g_lo * g_lo);
    if (r2 >= 1.0f - 1e-7f) { m->shelf_c = 1.0f; return; }   /* d ~ 0: open */
    float k = 4.0f * SHELF_S * SHELF_S * r2 / (1.0f - r2);
    m->shelf_c = k / (k + sqrtf(k * k + k));
}

/* apply the air shelf to an interleaved stereo block in place. The shelf is
   redesigned here, lazily, so callers that only consume gain[] never pay it. */
int airabs_apply(AirModel* m, float* buf, int frames) {
    if (!m || !m->valid || !buf) return AIRABS_E_STATE;
    if (frames <= 0) return AIRABS_E_PARAM;
    if (m->shelf_dirty) { airabs_shelf_design(m); m->shelf_dirty = 0; }
    float c = m->shelf_c, g = m->shelf_g;
    float zl = m->z1_l, zr = m->z1_r;
    for (int i = 0; i < frames; i++) {
        float xl = buf[2 * i];
        zl += c * (xl - zl);
        buf[2 * i] = g * (xl + (zl - xl));
        float xr = buf[2 * i + 1];
        zr += c * (xr - zr);
        buf[2 * i + 1] = g * (xr + (zr - xr));
    }
    m->z1_l = zl; m->z1_r = zr;
    return AIRABS_OK;
}

/* signed relative deviation vs an ISO 9613-1 Table 1 row, per band:
   (computed - table) / table. Table conditions: 20 C, 101.325 kPa. */
int airabs_table_dev(const AirClimate* c, float rh, float dev_out[NBANDS]) {
    if (!c || !c->valid || !dev_out) return AIRABS_E_STATE;
    if (!range_ok(rh, RH_MIN, RH_MAX)) return AIRABS_E_PARAM;
    const TableRow* row = 0;
    for (int i = 0; i < 3; i++)
        if (fabsf(TABLE_20C[i].rh - rh) < 0.01f) row = &TABLE_20C[i];
    if (!row) return AIRABS_E_PARAM;
    for (int b = 0; b < NBANDS; b++)
        dev_out[b] = (c->alpha[b] - row->a[b]) / row->a[b];
    return AIRABS_OK;
}

/* ---------------- self-test ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static float max_abs_dev(float rh) {
    AirClimate c;
    if (airabs_climate_set(&c, 20.0f, rh, 101.325f) != AIRABS_OK) return 1e9f;
    float dev[NBANDS];
    if (airabs_table_dev(&c, rh, dev) != AIRABS_OK) return 1e9f;
    const TableRow* row = 0;
    for (int i = 0; i < 3; i++)
        if (fabsf(TABLE_20C[i].rh - rh) < 0.01f) row = &TABLE_20C[i];
    float m = 0.0f;
    for (int b = 0; b < NBANDS; b++) {
        printf("  table %.0f%% %5g Hz: iso=%.4g tab=%.4g dev=%+.2f%%\n",
               (double)rh, (double)BAND_HZ[b], c.alpha[b], row->a[b],
               100.0 * dev[b]);
        if (fabsf(dev[b]) > m) m = fabsf(dev[b]);
    }
    return m;
}

/* baseline frame cost to budget against: the spatial module's render of
   eight mono sources into interleaved stereo, including the per-sample
   inverse-distance gain and soft-clip pass. */
static float soft_clip(float v) {
    const float k = 0.8f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

#define ITERS 50000
#define FRAMES 256
#define NSRC 8
#define ROUNDS 7

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(void) {
    AirClimate c;
    AirModel m;
    memset(&c, 0, sizeof c);
    memset(&m, 0, sizeof m);

    /* --- parameter validation: out-of-range inputs must be rejected --- */
    check("reject_bad_temp", airabs_climate_set(&c, 60.0f, 50.0f, 101.325f) == AIRABS_E_PARAM);
    check("reject_bad_rh",   airabs_climate_set(&c, 20.0f, 150.0f, 101.325f) == AIRABS_E_PARAM);
    check("reject_bad_press",airabs_climate_set(&c, 20.0f, 50.0f, 10.0f) == AIRABS_E_PARAM);
    check("reject_nan",      airabs_climate_set(&c, NAN, 50.0f, 101.325f) == AIRABS_E_PARAM);
    check("reject_neg_dist", airabs_update(&m, &c, -5.0f) == AIRABS_E_PARAM);
    check("reject_huge_dist",airabs_update(&m, &c, D_MAX + 1.0f) == AIRABS_E_PARAM);
    AirModel fresh;
    memset(&fresh, 0, sizeof fresh);
    AirClimate empty;
    memset(&empty, 0, sizeof empty);
    check("reject_uninit",   airabs_update(&fresh, &empty, 10.0f) == AIRABS_E_STATE);

    /* --- happy path + geometric/air superposition identity --- */
    int rc = airabs_climate_set(&c, 20.0f, 50.0f, 101.325f);
    check("climate_ok", rc == AIRABS_OK);
    rc = airabs_update(&m, &c, 0.0f);
    check("zero_dist_unity", rc == AIRABS_OK &&
          fabsf(m.gain[0] - 1.0f) < 1e-6f && fabsf(m.air_db[0]) < 1e-6f);

    airabs_update(&m, &c, 100.0f);
    float geo_g = 1.0f / (1.0f + 100.0f);
    float air_4k = expf(-(c.nepers[6] * 100.0f));
    check("superposition", fabsf(m.gain[6] - geo_g * air_4k) < 0.01f * geo_g);
    check("geo_matches_spatial", fabsf(exp10f(-airabs_geo_db(&m) / 20.0f) - geo_g) < 1e-6f);
    printf("  gains @100m: 63Hz=%.2e 4kHz=%.2e 8kHz=%.2e (air 4k=%.4f)\n",
           (double)m.gain[0], (double)m.gain[6], (double)m.gain[7], (double)air_4k);

    /* --- per-band air gains track the exact exponential over the range
       where the band is still audible (nepers <= 10, ~-87 dB). Beyond that
       fast_exp clamps to silence, which is the intended behaviour. --- */
    float max_gain_err = 0.0f;
    for (float d = 0.5f; d < 1000.0f; d += 1.37f) {
        airabs_update(&m, &c, d);
        float geo_here = 1.0f / (1.0f + d);
        for (int b = 0; b < NBANDS; b++) {
            float xb = c.nepers[b] * d;
            if (xb > 10.0f || xb < 1e-4f) continue;
            float exact = expf(-xb);
            float got = m.gain[b] / geo_here;       /* strip geometric term */
            float err = fabsf(got / exact - 1.0f);
            if (err > max_gain_err) max_gain_err = err;
        }
    }
    printf("  air gain max rel err vs expf=%.3f%%\n", 100.0 * max_gain_err);
    check("air_gain_accurate", max_gain_err < 0.015f);

    /* --- fast exp stays accurate on the audible loss range x in [0,10] --- */
    float max_exp_err = 0.0f;
    for (float x = 0.0f; x <= 10.0f; x += 0.01f) {
        float err = fabsf(fast_exp_neg(x) / expf(-x) - 1.0f);
        if (err > max_exp_err) max_exp_err = err;
    }
    printf("  fast_exp max rel err=%.3f%%\n", 100.0 * max_exp_err);
    check("fast_exp_accurate", max_exp_err < 0.015f);

    /* --- high frequencies must attenuate harder than low frequencies --- */
    int monotone = 1;
    for (int b = 1; b < NBANDS; b++)
        if (m.gain[b] > m.gain[b - 1] + 1e-9f) monotone = 0;
    check("hf_rolls_off", monotone);

    /* far source: air term must become a substantial part of the loss */
    airabs_update(&m, &c, 1000.0f);
    check("far_air_significant", m.air_db[7] > 20.0f && m.air_db[0] < 1.0f);

    /* --- climate caching: unchanged inputs keep identical coefficients --- */
    float snap[NBANDS];
    memcpy(snap, c.alpha, sizeof snap);
    airabs_climate_set(&c, 20.0f, 50.0f, 101.325f);
    int same = 1;
    for (int b = 0; b < NBANDS; b++) if (c.alpha[b] != snap[b]) same = 0;
    check("climate_cached", same);

    /* --- temperature/humidity correction moves the coefficients --- */
    AirClimate hot, cold, dry, humid;
    airabs_climate_set(&hot, 35.0f, 50.0f, 101.325f);
    airabs_climate_set(&cold, -10.0f, 50.0f, 101.325f);
    airabs_climate_set(&dry, 20.0f, 20.0f, 101.325f);
    airabs_climate_set(&humid, 20.0f, 90.0f, 101.325f);
    check("temp_corrects", hot.alpha[6] != cold.alpha[6]);
    check("humidity_corrects", dry.alpha[6] != humid.alpha[6]);
    printf("  alpha 4kHz: -10C=%.3g 35C=%.3g | RH20=%.3g RH90=%.3g dB/km\n",
           (double)cold.alpha[6], (double)hot.alpha[6],
           (double)dry.alpha[6], (double)humid.alpha[6]);

    /* --- shelf filter really attenuates a high-frequency tone, and in the
       steady state its response at 4 kHz matches the analytic design --- */
    AirModel ms;
    memset(&ms, 0, sizeof ms);
    airabs_climate_set(&c, 20.0f, 50.0f, 101.325f);
    airabs_update(&ms, &c, 500.0f);
    float tone[FRAMES * 2];
    for (int i = 0; i < FRAMES; i++) {
        tone[2 * i] = sinf(2.0f * (float)M_PI * 4000.0f * i / 48000.0f);
        tone[2 * i + 1] = tone[2 * i];
    }
    airabs_apply(&ms, tone, FRAMES);
    float amp_after = 0.0f;
    for (int i = 128; i < FRAMES; i++) amp_after += tone[2 * i] * tone[2 * i];
    float ratio = sqrtf(2.0f * amp_after / (FRAMES - 128));   /* steady RMS */
    check("shelf_cuts_4k", ratio < 0.75f);
    check("shelf_matches_design", fabsf(ratio - ms.shelf_g *
          sqrtf(ms.shelf_c * ms.shelf_c /
                (ms.shelf_c * ms.shelf_c +
                 4.0f * (1.0f - ms.shelf_c) * (1.0f - ms.shelf_c) *
                 sinf((float)M_PI * 4000.0f / 48000.0f) *
                 sinf((float)M_PI * 4000.0f / 48000.0f)))) < 0.05f);
    printf("  shelf 4kHz steady RMS @500m=%.3f (design target %.3f)\n",
           (double)ratio, (double)fast_exp_neg(ms.nepers[6] * 500.0f));

    /* --- ISO table comparison: deviation report and tolerance --- */
    printf("table_comparison (ISO 9613-1 Table 1, 20C):\n");
    float d30 = max_abs_dev(30.0f);
    float d50 = max_abs_dev(50.0f);
    float d70 = max_abs_dev(70.0f);
    printf("max_abs_dev: RH30=%.2f%% RH50=%.2f%% RH70=%.2f%%\n",
           100.0 * d30, 100.0 * d50, 100.0 * d70);
    /* ISO states the formula reproduces the table within ~10%; we hold 6% */
    check("within_table_tolerance", d30 < 0.06f && d50 < 0.06f && d70 < 0.06f);

    /* --- per-frame overhead budget: <=10% of the existing frame work.
       Baseline mirrors spatial.c's real frame: per-source azimuth trig
       (cos/sin/atan2), ITD sin, IID and distance sqrts, then 8 mono sources
       rendered into a 256-frame interleaved stereo buffer and a full
       soft-clip pass. Each workload is timed ROUNDS times and the minimum is
       kept, so scheduler jitter cannot inflate the measured overhead. Two
       air paths are measured:
         steady - distance unchanged: gains are reused, essentially free;
         moving - every source moves each frame: full recompute path. --- */
    static float mono[FRAMES];
    static float frame[FRAMES * 2];
    for (int i = 0; i < FRAMES; i++) mono[i] = 0.5f * sinf(0.03f * i);
    AirModel pool[NSRC];
    for (int s = 0; s < NSRC; s++) airabs_update(&pool[s], &c, 10.0f + s);

    double base_best = 1e9, steady_best = 1e9, move_best = 1e9;
    volatile float sink = 0.0f;
    for (int round = 0; round < ROUNDS; round++) {
        double t0 = now_sec();
        for (int it = 0; it < ITERS; it++) {
            memset(frame, 0, sizeof frame);
            float yaw = 0.001f * it;
            for (int s = 0; s < NSRC; s++) {
                /* spatial.c per-source geometry work: az, ITD, IID, distance */
                float x = 5.0f + 0.1f * (it + s), z = -3.0f;
                float xr = cosf(yaw) * x - sinf(yaw) * z;
                float zr = sinf(yaw) * x + cosf(yaw) * z;
                float az = atan2f(xr, -zr);
                int delay = (int)lroundf(sinf(az) * 28.8f); /* 0.6 ms * 48k */
                float t = (sinf(az) + 1.0f) * 0.5f;
                float gl = sqrtf(1.0f - t), gr = sqrtf(t);
                float g = 1.0f / (1.0f + sqrtf(xr * xr + zr * zr));
                for (int i = 0; i < FRAMES; i++) {
                    int j = i - delay;
                    float sm = (j >= 0 && j < FRAMES) ? mono[j] : 0.0f;
                    frame[2 * i] += sm * g * gl;
                    frame[2 * i + 1] += sm * g * gr;
                }
            }
            float acc = 0.0f;
            for (int i = 0; i < FRAMES * 2; i++) acc += soft_clip(frame[i]);
            sink += acc;
        }
        double dt = now_sec() - t0;
        if (dt < base_best) base_best = dt;

        t0 = now_sec();
        for (int it = 0; it < ITERS; it++) {
            float acc = 0.0f;
            for (int s = 0; s < NSRC; s++) {
                airabs_update(&pool[s], &c, pool[s].distance); /* cache hit */
                for (int b = 0; b < NBANDS; b++) acc += pool[s].gain[b];
            }
            sink += acc;
        }
        dt = now_sec() - t0;
        if (dt < steady_best) steady_best = dt;

        t0 = now_sec();
        for (int it = 0; it < ITERS; it++) {
            float acc = 0.0f;
            for (int s = 0; s < NSRC; s++) {
                float d = 5.0f + 0.1f * (it + s);
                airabs_update(&pool[s], &c, d);              /* cache miss */
                for (int b = 0; b < NBANDS; b++) acc += pool[s].gain[b];
            }
            sink += acc;
        }
        dt = now_sec() - t0;
        if (dt < move_best) move_best = dt;
    }

    double steady_over = steady_best / base_best;
    double move_over = move_best / base_best;
    printf("timing per frame (best of %d rounds): baseline=%.4f ms "
           "steady=%.4f ms (%.2f%%) moving=%.4f ms (%.2f%%)\n",
           ROUNDS, base_best / ITERS * 1e3,
           steady_best / ITERS * 1e3, 100.0 * steady_over,
           move_best / ITERS * 1e3, 100.0 * move_over);
    /* both budgets bind: steady frames are the common case, moving frames the
       worst case; neither may exceed 10% of the existing frame work. */
    check("overhead_steady_under_10pct", steady_over <= 0.10);
    check("overhead_moving_under_10pct", move_over <= 0.10);
    (void)sink;

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
