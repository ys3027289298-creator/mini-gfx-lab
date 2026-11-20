/* dyneq: per-band dynamic equalizer for content whose low/high balance
   drifts from scene to scene.

   A fixed EQ cannot track that drift, so every band is measured and corrected:

     split -> band power -> smoothed level (dB) -> deviation from anchor
           -> target gain = -ratio * deviation (clamped to the band range)
           -> attack/release smoothing -> per-frame slew limit -> apply

   Bands come from complementary one-pole splits, so the band signals sum back
   to the input and a gain of 0 dB in every band is an exact passthrough.

   Contract checked by main():
     - overlapping / gapped / malformed bands and out-of-range gains are
       rejected at configure time
     - steady state matches a hand-tuned static-gain reference per sample,
       with an explicit error bound
     - gain motion is monotone (no overshoot) and does not follow short-term
       program envelope (no pumping); release is slower than attack
     - bypass is bit-identical to the input
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EQ_MAX_BANDS 8
#define EQ_CHANNELS 2
#define EQ_FRAME_MAX 256
#define EQ_GAIN_LIMIT_DB 24.0f   /* hard ceiling on any configured gain */
#define EQ_FLOOR_DB (-120.0f)
#define EQ_POWER_EPS 1e-20f

typedef enum {
    EQ_OK = 0,
    EQ_ERR_ARG = -1,          /* bad argument */
    EQ_ERR_LAYOUT = -2,       /* overlapping, gapped or malformed bands */
    EQ_ERR_TUNING = -3,       /* gain range / ratio / time constant invalid */
    EQ_ERR_UNCONFIGURED = -4
} EqResult;

typedef struct {
    float lo_hz;        /* 0 on the first band only */
    float hi_hz;        /* split point; 0 = open top on the last band only */
    float anchor_db;    /* reference-scene band level, dB (set by calibrate) */
    float range_db;     /* symmetric gain limit for this band, <= 24 dB */
    float ratio;        /* fraction of the deviation to correct, 0..1 */
    float attack_ms;    /* time constant while cutting gain */
    float release_ms;   /* time constant while raising gain, >= attack_ms */
    float hold_ms;      /* freeze release this long after a cut (anti-pumping) */
} EqBand;

typedef struct {
    float sample_rate;
    int frame_size;      /* samples per processing frame, 1..EQ_FRAME_MAX */
    float energy_ms;     /* band power detector time constant */
    float max_step_db;   /* gain slew limit per frame, > 0 */
    float min_gain_db;   /* global clamp, >= -24 dB */
    float max_gain_db;   /* global clamp, <= +24 dB */
} EqCfg;

typedef struct {
    float a;                 /* one-pole coefficient */
    float lp[EQ_CHANNELS];   /* split filter state per channel */
} EqSplit;

typedef struct {
    float env;               /* smoothed mean-square level */
    float level_db;
    double gain_db;
    double lin_gain;
    double hold_left_s;      /* remaining release hold, seconds */
    int primed;
} EqBandState;

typedef struct {
    EqCfg cfg;
    EqBand band[EQ_MAX_BANDS];
    int band_count;
    EqSplit split[EQ_MAX_BANDS - 1];
    EqBandState st[EQ_MAX_BANDS];
    float scratch[EQ_MAX_BANDS][EQ_CHANNELS][EQ_FRAME_MAX];
    int enabled;
    int configured;
} DynEq;

static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* one-pole low-pass coefficient for a corner at hz */
static float split_coef(float hz, float sr) {
    return 1.0f - expf(-2.0f * (float)M_PI * hz / sr);
}

static EqResult validate(const EqCfg* cfg, const EqBand* bands, int n) {
    if (!cfg || !bands || n < 1 || n > EQ_MAX_BANDS) return EQ_ERR_ARG;
    if (!(cfg->sample_rate > 0.0f) || !isfinite(cfg->sample_rate)) return EQ_ERR_ARG;
    if (cfg->frame_size < 1 || cfg->frame_size > EQ_FRAME_MAX) return EQ_ERR_ARG;
    if (!(cfg->energy_ms > 0.0f) || !isfinite(cfg->energy_ms)) return EQ_ERR_TUNING;
    if (!(cfg->max_step_db > 0.0f) || !isfinite(cfg->max_step_db)) return EQ_ERR_TUNING;
    if (!isfinite(cfg->min_gain_db) || !isfinite(cfg->max_gain_db)) return EQ_ERR_TUNING;
    if (cfg->min_gain_db > cfg->max_gain_db) return EQ_ERR_TUNING;
    if (cfg->min_gain_db < -EQ_GAIN_LIMIT_DB || cfg->max_gain_db > EQ_GAIN_LIMIT_DB)
        return EQ_ERR_TUNING;

    for (int i = 0; i < n; i++) {
        const EqBand* b = &bands[i];
        if (!isfinite(b->anchor_db) || b->anchor_db < EQ_FLOOR_DB ||
            b->anchor_db > EQ_GAIN_LIMIT_DB)
            return EQ_ERR_TUNING;
        if (!(b->range_db >= 0.0f) || b->range_db > EQ_GAIN_LIMIT_DB)
            return EQ_ERR_TUNING;                       /* gain out of range */
        if (!(b->ratio >= 0.0f) || b->ratio > 1.0f)
            return EQ_ERR_TUNING;
        if (!(b->attack_ms > 0.0f) || !(b->release_ms >= b->attack_ms))
            return EQ_ERR_TUNING;                       /* pumping guard */
        if (!(b->hold_ms >= 0.0f) || !isfinite(b->hold_ms))
            return EQ_ERR_TUNING;

        /* Edges form a contiguous chain from DC to the open top. This is
           stricter than "no overlap": a gap would leave that range with no
           band, so reconstruction would no longer sum to the input. */
        float want_lo = (i == 0) ? 0.0f : bands[i - 1].hi_hz;
        if (b->lo_hz != want_lo) return EQ_ERR_LAYOUT;  /* overlap or gap */
        if (i == n - 1) {
            if (b->hi_hz != 0.0f) return EQ_ERR_LAYOUT; /* top must be open */
        } else {
            if (!(b->hi_hz > b->lo_hz)) return EQ_ERR_LAYOUT;
            if (b->hi_hz >= cfg->sample_rate * 0.5f) return EQ_ERR_LAYOUT;
        }
    }

    /* per-band range must be reachable under the global clamp */
    for (int i = 0; i < n; i++) {
        if (-bands[i].range_db < cfg->min_gain_db) return EQ_ERR_TUNING;
        if (bands[i].range_db > cfg->max_gain_db) return EQ_ERR_TUNING;
    }
    return EQ_OK;
}

static void reset_state(DynEq* eq) {
    for (int k = 0; k < EQ_MAX_BANDS - 1; k++) {
        eq->split[k].lp[0] = eq->split[k].lp[1] = 0.0f;
    }
    for (int i = 0; i < eq->band_count; i++) {
        eq->st[i].env = 0.0f;
        eq->st[i].level_db = EQ_FLOOR_DB;
        eq->st[i].gain_db = 0.0;
        eq->st[i].lin_gain = 1.0;
        eq->st[i].hold_left_s = 0.0;
        eq->st[i].primed = 0;
    }
}

EqResult eq_configure(DynEq* eq, const EqCfg* cfg, const EqBand* bands, int n) {
    EqResult r = validate(cfg, bands, n);
    if (r != EQ_OK) return r;
    if (!eq) return EQ_ERR_ARG;

    eq->cfg = *cfg;
    eq->band_count = n;
    memcpy(eq->band, bands, sizeof(EqBand) * (size_t)n);
    eq->enabled = 1;
    eq->configured = 1;
    reset_state(eq);
    for (int k = 0; k + 1 < n; k++)
        eq->split[k].a = split_coef(bands[k].hi_hz, cfg->sample_rate);
    return EQ_OK;
}

void eq_reset(DynEq* eq) {
    if (eq) reset_state(eq);
}

void eq_set_enabled(DynEq* eq, int on) {
    if (eq) eq->enabled = on ? 1 : 0;
}

int eq_is_enabled(const DynEq* eq) {
    return eq && eq->enabled;
}

double eq_band_gain_db(const DynEq* eq, int band) {
    if (!eq || !eq->configured || band < 0 || band >= eq->band_count) return 0.0;
    return eq->st[band].gain_db;
}

float eq_band_level_db(const DynEq* eq, int band) {
    if (!eq || !eq->configured || band < 0 || band >= eq->band_count) return EQ_FLOOR_DB;
    return eq->st[band].level_db;
}

/* Split the frame into band signals and accumulate power. Bands are numbered
   low to high, matching the configuration order:
       band 0        = LP(edge 0)
       band j (mid)  = LP(edge j) - LP(edge j-1)
       band last     = x - LP(edge last-1)
   so the bands sum exactly back to x. Band samples are kept in scratch so
   gains can be applied in a single pass afterwards. */
static void split_frame(DynEq* eq, const float* in, int n, float* power) {
    int nb = eq->band_count;
    for (int i = 0; i < n; i++) {
        for (int c = 0; c < EQ_CHANNELS; c++) {
            float x = in[2 * i + c];
            float prev = 0.0f;
            for (int k = 0; k + 1 < nb; k++) {
                EqSplit* sp = &eq->split[k];
                sp->lp[c] += sp->a * (x - sp->lp[c]);
                eq->scratch[k][c][i] = (k == 0) ? sp->lp[c] : sp->lp[c] - prev;
                prev = sp->lp[c];
            }
            eq->scratch[nb - 1][c][i] = x - prev;
            for (int b = 0; b < nb; b++)
                power[b] += eq->scratch[b][c][i] * eq->scratch[b][c][i];
        }
    }
}

/* Advance detector, target gain, asymmetry and per-frame slew limit. */
static void update_gains(DynEq* eq, const float* power, int n) {
    float dt = (float)n / eq->cfg.sample_rate;
    float tau_s = eq->cfg.energy_ms * 0.001f;

    /* Symmetric power detector: its steady state is the mean-square level, so
       anchors and the hand-tuned reference are directly comparable. Anti-
       pumping is handled on the gain side (release hold + slow release). */
    float bcoef = 1.0f - expf(-dt / tau_s);

    float total = (float)(n * EQ_CHANNELS);
    for (int b = 0; b < eq->band_count; b++) {
        EqBandState* s = &eq->st[b];
        const EqBand* q = &eq->band[b];
        float p = power[b] / total;
        if (!s->primed) { s->env = p; s->primed = 1; }
        else s->env += bcoef * (p - s->env);
        s->level_db = 10.0f * log10f(s->env > EQ_POWER_EPS ? s->env : EQ_POWER_EPS);

        double target = -(double)q->ratio * ((double)s->level_db - (double)q->anchor_db);
        target = clampd(target, -(double)q->range_db, (double)q->range_db);
        target = clampd(target, (double)eq->cfg.min_gain_db, (double)eq->cfg.max_gain_db);

        /* fast when cutting, slow (and held) when releasing: no pumping */
        int cutting = target < s->gain_db;
        if (cutting) s->hold_left_s = q->hold_ms * 0.001;
        else if (s->hold_left_s > 0.0) s->hold_left_s -= (double)dt;

        double next = s->gain_db;
        if (cutting || s->hold_left_s <= 0.0) {
            double tau = (cutting ? q->attack_ms : q->release_ms) * 0.001;
            double alpha = tau > 0.0 ? 1.0 - exp(-(double)dt / tau) : 1.0;
            next = s->gain_db + alpha * (target - s->gain_db);
        }

        double step = clampd(next - s->gain_db,
                             -(double)eq->cfg.max_step_db,
                             (double)eq->cfg.max_step_db);
        s->gain_db = clampd(s->gain_db + step,
                            (double)eq->cfg.min_gain_db,
                            (double)eq->cfg.max_gain_db);
        s->lin_gain = pow(10.0, s->gain_db / 20.0);
    }
}

EqResult eq_process(DynEq* eq, const float* in, float* out, int frames) {
    if (!eq || !in || !out || frames < 0) return EQ_ERR_ARG;
    if (!eq->configured) return EQ_ERR_UNCONFIGURED;

    if (!eq->enabled) {   /* bypass must be bit-exact */
        if (out != in) memcpy(out, in, sizeof(float) * (size_t)frames * EQ_CHANNELS);
        return EQ_OK;
    }

    int done = 0;
    while (done < frames) {
        int n = eq->cfg.frame_size;
        if (n > frames - done) n = frames - done;

        float power[EQ_MAX_BANDS] = { 0.0f };
        split_frame(eq, in + (size_t)done * EQ_CHANNELS, n, power);
        update_gains(eq, power, n);

        for (int i = 0; i < n; i++) {
            for (int c = 0; c < EQ_CHANNELS; c++) {
                double acc = 0.0;
                for (int b = 0; b < eq->band_count; b++)
                    acc += (double)eq->scratch[b][c][i] * eq->st[b].lin_gain;
                out[(size_t)(done + i) * EQ_CHANNELS + c] = (float)acc;
            }
        }
        done += n;
    }
    return EQ_OK;
}

/* Measure each band's steady-state level on a reference signal and store it
   as that band's anchor. Leaves the filter reset. */
EqResult eq_calibrate(DynEq* eq, const float* in, int frames) {
    if (!eq || !in || frames <= 0) return EQ_ERR_ARG;
    if (!eq->configured) return EQ_ERR_UNCONFIGURED;

    reset_state(eq);
    int done = 0;
    while (done < frames) {
        int n = eq->cfg.frame_size;
        if (n > frames - done) n = frames - done;
        float power[EQ_MAX_BANDS] = { 0.0f };
        split_frame(eq, in + (size_t)done * EQ_CHANNELS, n, power);
        update_gains(eq, power, n);
        done += n;
    }
    for (int b = 0; b < eq->band_count; b++)
        eq->band[b].anchor_db = eq->st[b].level_db;
    reset_state(eq);
    return EQ_OK;
}

/* ------------------------------------------------------------------ */
/* self-test                                                           */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

#define SR 48000.0f
#define FRAME 128
#define NSEC 3

static void fill_scene(float* buf, int frames, const float amp[NSEC],
                       const float hz[NSEC], const float phase0[NSEC]) {
    for (int i = 0; i < frames; i++) {
        float t = (float)i / SR;
        float v = 0.0f;
        for (int k = 0; k < NSEC; k++)
            v += amp[k] * sinf(2.0f * (float)M_PI * hz[k] * t + phase0[k]);
        for (int c = 0; c < EQ_CHANNELS; c++) buf[2 * i + c] = v;
    }
}

/* Independent static reference: same complementary split, gains fixed from
   the first sample on. This is what hand-tuning the EQ for the scene yields. */
typedef struct {
    float a[EQ_MAX_BANDS - 1];
    float lp[EQ_MAX_BANDS - 1][EQ_CHANNELS];
} RefFilter;

static void ref_run(RefFilter* rf, const float* in, float* out, int frames,
                    const float* gains_db, int nb) {
    for (int i = 0; i < frames; i++) {
        for (int c = 0; c < EQ_CHANNELS; c++) {
            float x = in[2 * i + c];
            float prev = 0.0f, band[EQ_MAX_BANDS];
            for (int k = 0; k + 1 < nb; k++) {
                float* s = &rf->lp[k][c];
                *s += rf->a[k] * (x - *s);
                band[k] = (k == 0) ? *s : *s - prev;
                prev = *s;
            }
            band[nb - 1] = x - prev;
            float acc = 0.0f;
            for (int b = 0; b < nb; b++)
                acc += band[b] * (float)pow(10.0, (double)gains_db[b] / 20.0);
            out[2 * i + c] = acc;
        }
    }
}

/* Steady-state band levels of a signal under the same split. */
static void measure_bands(const float* in, int frames, float out_db[EQ_MAX_BANDS],
                          int nb, const float* edges) {
    float lp[EQ_MAX_BANDS - 1][EQ_CHANNELS];
    float acc[EQ_MAX_BANDS] = { 0.0f };
    memset(lp, 0, sizeof(lp));
    /* same low-to-high band order as split_frame */
    for (int i = 0; i < frames; i++) {
        for (int c = 0; c < EQ_CHANNELS; c++) {
            float x = in[2 * i + c];
            float prev = 0.0f, band;
            for (int k = 0; k + 1 < nb; k++) {
                lp[k][c] += split_coef(edges[k], SR) * (x - lp[k][c]);
                band = (k == 0) ? lp[k][c] : lp[k][c] - prev;
                acc[k] += band * band;
                prev = lp[k][c];
            }
            band = x - prev;
            acc[nb - 1] += band * band;
        }
    }
    for (int b = 0; b < nb; b++) {
        float p = acc[b] / (float)(frames * EQ_CHANNELS);
        out_db[b] = 10.0f * log10f(p > EQ_POWER_EPS ? p : EQ_POWER_EPS);
    }
}

static float peak_diff(const float* a, const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = fabsf(a[i] - b[i]);
        if (d > m) m = d;
    }
    return m;
}

int main(void) {
    const int nb = 3;
    const float edges[EQ_MAX_BANDS - 1] = { 500.0f, 2000.0f };

    EqCfg cfg = { SR, FRAME, 60.0f, 0.05f, -18.0f, 18.0f };
    EqBand bands[EQ_MAX_BANDS] = {
        { 0.0f,    500.0f,  0.0f, 12.0f, 0.6f, 15.0f, 300.0f, 200.0f },
        { 500.0f,  2000.0f, 0.0f, 12.0f, 0.4f, 15.0f, 300.0f, 200.0f },
        { 2000.0f, 0.0f,    0.0f, 12.0f, 0.6f, 15.0f, 300.0f, 200.0f },
    };
    DynEq eq;
    memset(&eq, 0, sizeof(eq));
    check("configure_ok", eq_configure(&eq, &cfg, bands, nb) == EQ_OK);

    /* --- rejection cases (each must leave the working config intact) --- */
    EqBand t[EQ_MAX_BANDS];

    memcpy(t, bands, sizeof(t));
    t[1].lo_hz = 400.0f;                             /* overlaps band 0 */
    check("reject_overlap", eq_configure(&eq, &cfg, t, nb) == EQ_ERR_LAYOUT);

    memcpy(t, bands, sizeof(t));
    t[1].lo_hz = 600.0f;                             /* 500..600 unassigned */
    check("reject_gap", eq_configure(&eq, &cfg, t, nb) == EQ_ERR_LAYOUT);

    memcpy(t, bands, sizeof(t));
    t[0].hi_hz = 0.0f;                               /* mid band left open */
    check("reject_malformed", eq_configure(&eq, &cfg, t, nb) == EQ_ERR_LAYOUT);

    memcpy(t, bands, sizeof(t));
    t[0].range_db = 30.0f;                           /* beyond 24 dB limit */
    check("reject_gain_range", eq_configure(&eq, &cfg, t, nb) == EQ_ERR_TUNING);

    memcpy(t, bands, sizeof(t));
    t[2].release_ms = 1.0f;                          /* release faster than attack */
    check("reject_fast_release", eq_configure(&eq, &cfg, t, nb) == EQ_ERR_TUNING);

    memcpy(t, bands, sizeof(t));
    t[1].ratio = 1.5f;
    check("reject_ratio", eq_configure(&eq, &cfg, t, nb) == EQ_ERR_TUNING);

    check("survives_reject", eq_configure(&eq, &cfg, bands, nb) == EQ_OK);

    /* --- calibrate on scene A, process the differently balanced scene B --- */
    const float hz[NSEC] = { 120.0f, 1000.0f, 5000.0f };
    const float ph[NSEC] = { 0.0f, 0.3f, 0.7f };
    const float amp_a[NSEC] = { 0.40f, 0.15f, 0.05f };  /* low-heavy */
    const float amp_b[NSEC] = { 0.05f, 0.15f, 0.35f };  /* high-heavy */

    int n_a = (int)(SR * 1.0f);
    float* a = malloc(sizeof(float) * (size_t)n_a * EQ_CHANNELS);
    fill_scene(a, n_a, amp_a, hz, ph);
    check("calibrate_ok", eq_calibrate(&eq, a, n_a) == EQ_OK);
    printf("anchors_db=%.2f,%.2f,%.2f\n",
           eq.band[0].anchor_db, eq.band[1].anchor_db, eq.band[2].anchor_db);

    int n_b = (int)(SR * 3.0f);
    float* b = malloc(sizeof(float) * (size_t)n_b * EQ_CHANNELS);
    fill_scene(b, n_b, amp_b, hz, ph);

    float lv_b[EQ_MAX_BANDS];
    measure_bands(b, n_b, lv_b, nb, edges);
    float ref_gain[EQ_MAX_BANDS];
    /* anchors live in the eq (calibrate writes them there), not in `bands` */
    for (int k = 0; k < nb; k++) {
        double g = -(double)eq.band[k].ratio *
                   ((double)lv_b[k] - (double)eq.band[k].anchor_db);
        ref_gain[k] = (float)clampd(g, -eq.band[k].range_db, eq.band[k].range_db);
    }
    printf("ref_gain_db=%.3f,%.3f,%.3f\n", ref_gain[0], ref_gain[1], ref_gain[2]);

    float* out_dyn = malloc(sizeof(float) * (size_t)n_b * EQ_CHANNELS);
    float* out_ref = malloc(sizeof(float) * (size_t)n_b * EQ_CHANNELS);

    RefFilter rf;
    memset(&rf, 0, sizeof(rf));
    for (int k = 0; k + 1 < nb; k++)
        rf.a[k] = split_coef(edges[k], SR);
    ref_run(&rf, b, out_ref, n_b, ref_gain, nb);

    eq_set_enabled(&eq, 1);
    check("process_ok", eq_process(&eq, b, out_dyn, n_b) == EQ_OK);
    printf("gain_after_b=%.3f,%.3f,%.3f\n",
           eq_band_gain_db(&eq, 0), eq_band_gain_db(&eq, 1), eq_band_gain_db(&eq, 2));

    int tail = n_b / 3;                          /* last 1 s of a 3 s scene */
    const float steady_bound = 2e-3f;
    float serr = peak_diff(out_dyn + (size_t)(n_b - tail) * EQ_CHANNELS,
                           out_ref + (size_t)(n_b - tail) * EQ_CHANNELS,
                           tail * EQ_CHANNELS);
    printf("steady_err=%.6g bound=%.1g\n", serr, steady_bound);
    check("steady_within_bound", serr < steady_bound);
    check("gain_matches_ref",
          fabsf((float)eq_band_gain_db(&eq, 0) - ref_gain[0]) < 0.05f &&
          fabsf((float)eq_band_gain_db(&eq, 1) - ref_gain[1]) < 0.05f &&
          fabsf((float)eq_band_gain_db(&eq, 2) - ref_gain[2]) < 0.05f);

    /* --- adjustment path: monotone, no overshoot --- */
    eq_reset(&eq);
    int frames_b = n_b / FRAME;
    float prev[EQ_MAX_BANDS] = { 0.0f, 0.0f, 0.0f };
    int dir[EQ_MAX_BANDS] = { 0, 0, 0 };
    int reversals = 0, overshoot = 0;
    for (int f = 0; f < frames_b; f++) {
        eq_process(&eq, b + (size_t)f * FRAME * EQ_CHANNELS,
                   out_dyn + (size_t)f * FRAME * EQ_CHANNELS, FRAME);
        for (int k = 0; k < nb; k++) {
            float g = (float)eq_band_gain_db(&eq, k);
            float d = g - prev[k];
            int s = d > 1e-4f ? 1 : (d < -1e-4f ? -1 : 0);
            int far = fabsf(g - ref_gain[k]) > 0.25f;   /* still settling */
            if (s != 0 && dir[k] != 0 && s != dir[k] && far) reversals++;
            if (s != 0) dir[k] = s;
            prev[k] = g;
            if (ref_gain[k] < -0.05f && g < ref_gain[k] - 0.02f) overshoot++;
            if (ref_gain[k] >  0.05f && g > ref_gain[k] + 0.02f) overshoot++;
        }
    }
    printf("gain_reversals=%d overshoot_frames=%d\n", reversals, overshoot);
    check("no_gain_oscillation", reversals == 0);
    check("no_overshoot", overshoot == 0);

    /* --- attack vs release: 90% settling time, measured on band 0 --- */
    float* loud = malloc(sizeof(float) * (size_t)n_b * EQ_CHANNELS);
    const float amp_loud[NSEC] = { 0.80f, 0.0f, 0.0f };
    fill_scene(loud, n_b, amp_loud, hz, ph);

    int attack_frames = -1;
    eq_reset(&eq);
    for (int f = 0; f < frames_b; f++)
        eq_process(&eq, loud + (size_t)f * FRAME * EQ_CHANNELS,
                   out_dyn + (size_t)f * FRAME * EQ_CHANNELS, FRAME);
    float g_cut = (float)eq_band_gain_db(&eq, 0);

    eq_reset(&eq);
    for (int f = 0; f < frames_b; f++) {
        eq_process(&eq, loud + (size_t)f * FRAME * EQ_CHANNELS,
                   out_dyn + (size_t)f * FRAME * EQ_CHANNELS, FRAME);
        float g = (float)eq_band_gain_db(&eq, 0);
        if (fabsf(g - g_cut) <= 0.1f * fabsf(g_cut)) { attack_frames = f; break; }
    }

    int release_frames = -1;
    memset(out_dyn, 0, sizeof(float) * (size_t)n_b * EQ_CHANNELS);
    float* silence = out_ref;
    memset(silence, 0, sizeof(float) * (size_t)n_b * EQ_CHANNELS);
    int frames_hold = (int)(0.5f * SR) / FRAME;
    for (int f = 0; f < frames_hold; f++)   /* settle the cut first */
        eq_process(&eq, loud + (size_t)f * FRAME * EQ_CHANNELS,
                   out_dyn + (size_t)f * FRAME * EQ_CHANNELS, FRAME);
    float g_cut2 = (float)eq_band_gain_db(&eq, 0);
    int n_release = (int)(SR * 3.0f) / FRAME;
    float* sil = malloc(sizeof(float) * (size_t)n_release * FRAME * EQ_CHANNELS);
    memset(sil, 0, sizeof(float) * (size_t)n_release * FRAME * EQ_CHANNELS);
    float* out_sil = malloc(sizeof(float) * (size_t)n_release * FRAME * EQ_CHANNELS);
    /* run silence to convergence to learn the release target */
    for (int f = 0; f < n_release; f++)
        eq_process(&eq, sil + (size_t)f * FRAME * EQ_CHANNELS,
                   out_sil + (size_t)f * FRAME * EQ_CHANNELS, FRAME);
    float g_rest = (float)eq_band_gain_db(&eq, 0);

    eq_reset(&eq);
    for (int f = 0; f < frames_hold; f++)
        eq_process(&eq, loud + (size_t)f * FRAME * EQ_CHANNELS,
                   out_dyn + (size_t)f * FRAME * EQ_CHANNELS, FRAME);
    for (int f = 0; f < n_release; f++) {
        eq_process(&eq, sil + (size_t)f * FRAME * EQ_CHANNELS,
                   out_sil + (size_t)f * FRAME * EQ_CHANNELS, FRAME);
        float g = (float)eq_band_gain_db(&eq, 0);
        if (fabsf(g - g_rest) <= 0.1f * fabsf(g_rest - g_cut2)) { release_frames = f; break; }
    }
    printf("attack_frames=%d release_frames=%d cut=%.2f rest=%.2f\n",
           attack_frames, release_frames, g_cut, g_rest);
    check("attack_settles", attack_frames > 0);
    check("release_settles", release_frames > 0);
    check("release_slower_than_attack",
          release_frames > 2 * attack_frames);

    /* --- per-frame slew limit is never exceeded (the "限幅" guarantee) --- */
    eq_reset(&eq);
    float max_step = 0.0f;
    for (int f = 0; f < frames_b; f++) {
        float before = (float)eq_band_gain_db(&eq, 0);
        eq_process(&eq, b + (size_t)f * FRAME * EQ_CHANNELS,
                   out_dyn + (size_t)f * FRAME * EQ_CHANNELS, FRAME);
        float d = fabsf((float)eq_band_gain_db(&eq, 0) - before);
        if (d > max_step) max_step = d;
    }
    printf("max_step_db=%.4f limit=%.2f\n", max_step, cfg.max_step_db);
    check("slew_limited", max_step <= cfg.max_step_db + 1e-4f);

    /* --- per-band range clamp holds under a gross imbalance --- */
    float* extreme = malloc(sizeof(float) * (size_t)n_b * EQ_CHANNELS);
    const float amp_extreme[NSEC] = { 0.0f, 0.0f, 0.90f };  /* only band 2 */
    fill_scene(extreme, n_b, amp_extreme, hz, ph);
    eq_reset(&eq);
    for (int f = 0; f < frames_b; f++)
        eq_process(&eq, extreme + (size_t)f * FRAME * EQ_CHANNELS,
                   out_dyn + (size_t)f * FRAME * EQ_CHANNELS, FRAME);
    float g2 = (float)eq_band_gain_db(&eq, 2);
    printf("clamped_gain_db=%.3f range=%.2f\n", g2, eq.band[2].range_db);
    check("gain_clamped_to_range", fabsf(g2) <= eq.band[2].range_db + 1e-3f);

    /* --- no pumping: 4 Hz level tremolo must not modulate the gain --- */
    /* +-3 dB, 4 Hz level drift on the low band: a static scene with a slow
       envelope, which must NOT be tracked as a gain change */
    float* trem = malloc(sizeof(float) * (size_t)n_b * EQ_CHANNELS);
    for (int i = 0; i < n_b; i++) {
        float t = (float)i / SR;
        float drift_db = 3.0f * sinf(2.0f * (float)M_PI * 4.0f * t);
        float env = powf(10.0f, drift_db / 20.0f);
        float v = 0.3f * env * sinf(2.0f * (float)M_PI * 120.0f * t)
                + 0.1f * sinf(2.0f * (float)M_PI * 3000.0f * t);
        for (int c = 0; c < EQ_CHANNELS; c++) trem[2 * i + c] = v;
    }
    eq_reset(&eq);
    float gmin = 1e9f, gmax = -1e9f;
    for (int f = 0; f < frames_b; f++) {
        eq_process(&eq, trem + (size_t)f * FRAME * EQ_CHANNELS,
                   out_dyn + (size_t)f * FRAME * EQ_CHANNELS, FRAME);
        if (f > frames_b / 3) {                 /* ignore warm-up */
            float g = (float)eq_band_gain_db(&eq, 0);
            if (g < gmin) gmin = g;
            if (g > gmax) gmax = g;
        }
    }
    printf("tremolo_gain_pp=%.4f dB\n", gmax - gmin);
    check("no_pumping", (gmax - gmin) < 0.5f);

    /* --- bypass is bit-identical --- */
    for (int i = 0; i < 64 * EQ_CHANNELS; i++) out_dyn[i] = 123.0f;
    eq_set_enabled(&eq, 0);
    eq_process(&eq, b, out_dyn, 64);
    check("bypass_bit_exact", memcmp(out_dyn, b, sizeof(float) * 64 * EQ_CHANNELS) == 0);

    /* --- all-zero correction reconstructs the input --- */
    EqBand flat[EQ_MAX_BANDS];
    memcpy(flat, bands, sizeof(flat));
    for (int k = 0; k < nb; k++) flat[k].ratio = 0.0f;
    DynEq eq2;
    memset(&eq2, 0, sizeof(eq2));
    check("flat_configure", eq_configure(&eq2, &cfg, flat, nb) == EQ_OK);
    eq_set_enabled(&eq2, 1);
    eq_process(&eq2, b, out_dyn, 8192);
    printf("passthrough_err=%.3g\n", peak_diff(out_dyn, b, 8192 * EQ_CHANNELS));
    check("zero_gain_passthrough", peak_diff(out_dyn, b, 8192 * EQ_CHANNELS) < 1e-5f);

    printf("all_ok=%d\n", failures == 0);
    free(a); free(b); free(loud); free(trem); free(sil); free(out_sil);
    free(extreme);
    free(out_dyn); free(out_ref);
    return failures == 0 ? 0 : 1;
}
