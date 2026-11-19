
/* mixer: multi-bus audio mixing with per-bus gain, sidechain ducking and
   soft clipping. All buffers are interleaved stereo.
   Also includes a 3-band dynamics processor (LR4 crossover tree, per-band
   peak compressors, automatic makeup gain), see mb_* below. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <complex.h>

#define MAX_SRC 8
#define FRAMES 128
#define MB_BANDS 3
#define PI 3.14159265358979323846

typedef struct {
    int bus;        /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
} Source;

typedef struct {
    float gain[3];
    float duck_depth;    /* 0..1, applied to bus 0 when voice is active */
    int duck_enabled;
} Mixer;

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
    float bus[3][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b > 2) continue;
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

/* ===================================================================== */
/* Multiband dynamics                                                     */
/*                                                                         */
/* Crossover: Linkwitz-Riley 4th order = two cascaded 2nd-order Butter-   */
/* worth biquads per arm. An LR4 low/high pair sums to a 2nd-order all-   */
/* pass (unit magnitude everywhere, identical phase at the crossover),    */
/* so the tree below reconstructs with zero magnitude dip:                */
/*   low  = LP4(fc1) * AP2(fc2)                                           */
/*   mid  = HP4(fc1) * LP2(fc2)                                           */
/*   high = HP4(fc1) * HP4(fc2)                                           */
/*   sum  = AP2(fc1) * AP2(fc2)   (magnitude exactly 1.0)                 */
/* Everything is causal IIR with no look-ahead, so algorithmic latency    */
/* is 0 samples; only filter group delay contributes (measured in test).  */
/* ===================================================================== */

typedef struct {
    float b0, b1, b2, a1, a2;
    float s1, s2;                 /* transposed direct-form II state */
} Biquad;

typedef struct {
    float threshold_db;
    float ratio;                  /* e.g. 4.0 = 4:1 */
    float attack_ms;
    float release_ms;
    float makeup_db;              /* manual trim, added on top of auto */
    int   auto_makeup;
} BandCfg;

typedef struct {
    /* [channel][cascade stage] */
    Biquad lp1[2][2], hp1[2][2];  /* LR4 at fc1 */
    Biquad lp2[2][2], hp2[2][2];  /* LR4 at fc2, fed from the high arm */
    Biquad ap2[2];                /* AP2 at fc2, delay match for low band */
    BandCfg band[MB_BANDS];
    float env[MB_BANDS];          /* peak envelope, linear, stereo linked */
    float atk[MB_BANDS], rel[MB_BANDS];
    float auto_gain[MB_BANDS];    /* linear, from mb_calibrate */
    float global_makeup_db;
    float fs, fc1, fc2;
    int enabled;
} Multiband;

static float bq_process(Biquad* f, float x) {
    float y = f->b0 * x + f->s1;
    f->s1 = f->b1 * x - f->a1 * y + f->s2;
    f->s2 = f->b2 * x - f->a2 * y;
    return y;
}

static void bq_reset(Biquad* f) { f->s1 = f->s2 = 0.0f; }

/* RBJ cookbook designs, Q = 1/sqrt(2) (Butterworth 2nd order). */
static void bq_design(Biquad* f, int type, double fc, double fs) {
    double w0 = 2.0 * PI * fc / fs;
    double cw = cos(w0), sw = sin(w0);
    double alpha = sw / (2.0 * 0.7071067811865476);
    double a0 = 1.0 + alpha;
    double b0, b1, b2;
    if (type == 0) {             /* low pass */
        b0 = (1.0 - cw) / 2.0; b1 = 1.0 - cw; b2 = (1.0 - cw) / 2.0;
    } else if (type == 1) {      /* high pass */
        b0 = (1.0 + cw) / 2.0; b1 = -(1.0 + cw); b2 = (1.0 + cw) / 2.0;
    } else {                     /* 2nd-order all pass */
        b0 = 1.0 - alpha; b1 = -2.0 * cw; b2 = 1.0 + alpha;
    }
    f->b0 = (float)(b0 / a0);
    f->b1 = (float)(b1 / a0);
    f->b2 = (float)(b2 / a0);
    f->a1 = (float)(-2.0 * cw / a0);
    f->a2 = (float)((1.0 - alpha) / a0);
    f->s1 = f->s2 = 0.0f;
}

static void mb_init(Multiband* mb, float fs, float fc1, float fc2,
                    const BandCfg cfg[MB_BANDS]) {
    memset(mb, 0, sizeof(*mb));
    mb->fs = fs; mb->fc1 = fc1; mb->fc2 = fc2;
    mb->enabled = 1;
    mb->global_makeup_db = 0.0f;
    for (int ch = 0; ch < 2; ch++) {
        for (int k = 0; k < 2; k++) {
            bq_design(&mb->lp1[ch][k], 0, fc1, fs);
            bq_design(&mb->hp1[ch][k], 1, fc1, fs);
            bq_design(&mb->lp2[ch][k], 0, fc2, fs);
            bq_design(&mb->hp2[ch][k], 1, fc2, fs);
        }
        bq_design(&mb->ap2[ch], 2, fc2, fs);
    }
    for (int b = 0; b < MB_BANDS; b++) {
        mb->band[b] = cfg[b];
        mb->env[b] = 0.0f;
        mb->atk[b] = (float)(1.0 - exp(-1.0 / (fs * cfg[b].attack_ms / 1000.0)));
        mb->rel[b] = (float)(1.0 - exp(-1.0 / (fs * cfg[b].release_ms / 1000.0)));
        mb->auto_gain[b] = 1.0f;
    }
}

static void mb_reset(Multiband* mb) {
    for (int ch = 0; ch < 2; ch++) {
        for (int k = 0; k < 2; k++) {
            bq_reset(&mb->lp1[ch][k]); bq_reset(&mb->hp1[ch][k]);
            bq_reset(&mb->lp2[ch][k]); bq_reset(&mb->hp2[ch][k]);
        }
        bq_reset(&mb->ap2[ch]);
    }
    for (int b = 0; b < MB_BANDS; b++) mb->env[b] = 0.0f;
}

/* Static compressor curve: gain in dB for a given peak level in dBFS.
   6 dB soft knee around the threshold. */
static float comp_curve_db(const BandCfg* c, float level_db) {
    const float kw = 6.0f;
    float over = level_db - c->threshold_db;
    if (over <= -kw / 2.0f) return 0.0f;
    if (over < kw / 2.0f) {
        float t = over + kw / 2.0f;
        return -(1.0f - 1.0f / c->ratio) * t * t / (2.0f * kw);
    }
    return -(1.0f - 1.0f / c->ratio) * over;
}

/* Split one stereo sample into the three phase-matched bands. */
static void mb_split(Multiband* mb, float x0, float x1,
                     float bands[MB_BANDS][2]) {
    for (int ch = 0; ch < 2; ch++) {
        float x = ch == 0 ? x0 : x1;
        float l1 = bq_process(&mb->lp1[ch][1],
                   bq_process(&mb->lp1[ch][0], x));
        float h1 = bq_process(&mb->hp1[ch][1],
                   bq_process(&mb->hp1[ch][0], x));
        float m  = bq_process(&mb->lp2[ch][1],
                   bq_process(&mb->lp2[ch][0], h1));
        float hi = bq_process(&mb->hp2[ch][1],
                   bq_process(&mb->hp2[ch][0], h1));
        float lo = bq_process(&mb->ap2[ch], l1);
        bands[0][ch] = lo; bands[1][ch] = m; bands[2][ch] = hi;
    }
}

/* Core runner. When acc non-null, accumulates per-band pre/post RMS sums
   (used by calibration and spectral analysis). */
static void mb_run(Multiband* mb, const float* in, float* out, int frames,
                   double acc_pre[MB_BANDS], double acc_post[MB_BANDS]) {
    if (acc_pre) memset(acc_pre, 0, sizeof(double) * MB_BANDS);
    if (acc_post) memset(acc_post, 0, sizeof(double) * MB_BANDS);
    float global = powf(10.0f, mb->global_makeup_db / 20.0f);

    for (int i = 0; i < frames; i++) {
        float bands[MB_BANDS][2];
        mb_split(mb, in[i * 2], in[i * 2 + 1], bands);

        float gain_lin[MB_BANDS];
        for (int b = 0; b < MB_BANDS; b++) {
            float p = fabsf(bands[b][0]) > fabsf(bands[b][1])
                    ? fabsf(bands[b][0]) : fabsf(bands[b][1]);
            float c = p > mb->env[b] ? mb->atk[b] : mb->rel[b];
            mb->env[b] += c * (p - mb->env[b]);

            float level = 20.0f * log10f(mb->env[b] + 1e-10f);
            float gdb = mb->enabled ? comp_curve_db(&mb->band[b], level) : 0.0f;
            float makeup = mb->band[b].makeup_db;
            if (mb->band[b].auto_makeup)
                makeup += 20.0f * log10f(mb->auto_gain[b] + 1e-10f);
            gain_lin[b] = powf(10.0f, (gdb + makeup) / 20.0f);

            if (acc_pre)
                acc_pre[b] += (double)bands[b][0] * bands[b][0]
                            + (double)bands[b][1] * bands[b][1];
        }

        for (int ch = 0; ch < 2; ch++) {
            float v = 0.0f;
            for (int b = 0; b < MB_BANDS; b++) {
                float y = bands[b][ch] * gain_lin[b];
                v += y;
                if (acc_post) acc_post[b] += (double)y * y;
            }
            if (out) out[i * 2 + ch] = v * global;
        }
    }
}

static void mb_process(Multiband* mb, const float* in, float* out, int frames) {
    mb_run(mb, in, out, frames, NULL, NULL);
}

/* One calibration pass: per-band auto makeup restores measured band RMS. */
static void mb_calibrate(Multiband* mb, const float* in, int frames) {
    double pre[MB_BANDS], post[MB_BANDS];
    mb_reset(mb);
    mb_run(mb, in, NULL, frames, pre, post);   /* out unused when acc given */
    for (int b = 0; b < MB_BANDS; b++) {
        double rms_in = sqrt(pre[b] / (2.0 * frames));
        double rms_out = sqrt(post[b] / (2.0 * frames));
        mb->auto_gain[b] = rms_out > 1e-6 ? (float)(rms_in / rms_out) : 1.0f;
    }
    mb_reset(mb);
}

/* Single-band reference compressor: same curve, one broadband gain. */
static void single_band(const BandCfg* c, float fs,
                        const float* in, float* out, int frames) {
    float atk = (float)(1.0 - exp(-1.0 / (fs * c->attack_ms / 1000.0)));
    float rel = (float)(1.0 - exp(-1.0 / (fs * c->release_ms / 1000.0)));
    float env = 0.0f;
    for (int i = 0; i < frames; i++) {
        float p = fabsf(in[i * 2]);
        if (fabsf(in[i * 2 + 1]) > p) p = fabsf(in[i * 2 + 1]);
        env += (p > env ? atk : rel) * (p - env);
        float level = 20.0f * log10f(env + 1e-10f);
        float g = powf(10.0f, comp_curve_db(c, level) / 20.0f);
        out[i * 2] = in[i * 2] * g;
        out[i * 2 + 1] = in[i * 2 + 1] * g;
    }
}

/* ---- analytic response of the crossover (double precision) ---- */

static complex double bq_response(const Biquad* f, double w) {
    complex double z1 = cexp(-I * w);
    complex double z2 = z1 * z1;
    complex double num = f->b0 + f->b1 * z1 + f->b2 * z2;
    complex double den = 1.0 + f->a1 * z1 + f->a2 * z2;
    return num / den;
}

static complex double p4(const Biquad f[2], double w) {
    complex double h = bq_response(&f[0], w);
    return h * bq_response(&f[1], w);
}

/* full reconstruction response of the 3-band tree, channel 0 coefficients */
static complex double mb_response(Multiband* mb, double w) {
    complex double low = p4(mb->lp1[0], w) * bq_response(&mb->ap2[0], w);
    complex double h1_lo = p4(mb->hp1[0], w);
    complex double mid = h1_lo * p4(mb->lp2[0], w);
    complex double high = h1_lo * p4(mb->hp2[0], w);
    return low + mid + high;
}

static float db_of(double mag) { return (float)(20.0 * log10(mag)); }

/* ===================================================================== */
/* Self tests                                                             */
/* ===================================================================== */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    /* ---------- original mixer tests ---------- */
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

    /* ---------- multiband dynamics tests ---------- */
    const float FS = 48000.0f;
    const float FC1 = 350.0f, FC2 = 3000.0f;
    BandCfg cfg[MB_BANDS] = {
        { -14.0f, 4.0f, 2.0f, 80.0f, 0.0f, 1 },   /* low  */
        { -16.0f, 2.5f, 2.0f, 80.0f, 0.0f, 1 },   /* mid  */
        { -22.0f, 2.0f, 2.0f, 80.0f, 0.0f, 1 },   /* high */
    };
    Multiband mb;
    mb_init(&mb, FS, FC1, FC2, cfg);

    /* static compressor curve: unity below knee, ratio above it */
    int curve_ok = 1;
    curve_ok &= fabsf(comp_curve_db(&cfg[0], -40.0f)) < 1e-6f;
    /* 10 dB above threshold with 4:1 -> 7.5 dB reduction (past 6 dB knee) */
    float gr = -comp_curve_db(&cfg[0], cfg[0].threshold_db + 10.0f);
    curve_ok &= fabsf(gr - 7.5f) < 0.01f;
    /* 2:1 band, 12 dB over -> 6 dB reduction */
    float gr2 = -comp_curve_db(&cfg[2], cfg[2].threshold_db + 12.0f);
    curve_ok &= fabsf(gr2 - 6.0f) < 0.01f;
    /* monotonic non-increasing gain across the knee */
    float prev_g = 0.0f;
    for (float lv = cfg[0].threshold_db - 4.0f;
         lv <= cfg[0].threshold_db + 4.0f; lv += 0.25f) {
        float g = comp_curve_db(&cfg[0], lv);
        if (g > prev_g + 1e-6f) curve_ok = 0;
        prev_g = g;
    }
    check("comp_curve_threshold_ratio_knee", curve_ok);

    /* identity: LR4 low+high pair equals the AP2 allpass exactly (holds to
       ~1e-14 in double; float-stored coefficients widen this to ~1e-5) */
    Biquad ap_fc1, ap_fc2;
    bq_design(&ap_fc1, 2, FC1, FS);
    bq_design(&ap_fc2, 2, FC2, FS);
    int identity_ok = 1;
    for (int k = 1; k <= 20; k++) {
        double f = 20.0 * pow(1000.0, (k - 1) / 19.0);     /* 20 Hz .. 20 kHz */
        double w = 2.0 * PI * f / FS;
        complex double pair1 = p4(mb.lp1[0], w) + p4(mb.hp1[0], w);
        complex double pair2 = p4(mb.lp2[0], w) + p4(mb.hp2[0], w);
        if (cabs(pair1 - bq_response(&ap_fc1, w)) > 1e-4) identity_ok = 0;
        if (cabs(pair2 - bq_response(&ap_fc2, w)) > 1e-4) identity_ok = 0;
        if (fabs(cabs(bq_response(&ap_fc1, w)) - 1.0) > 1e-6) identity_ok = 0;
    }
    check("lr4_pair_is_allpass", identity_ok);

    /* magnitude flatness of the reconstructed response */
    const int NPT = 2000;
    float worst_db = 0.0f;
    for (int k = 0; k < NPT; k++) {
        double f = 20.0 * pow(1000.0, (double)k / (NPT - 1));
        double w = 2.0 * PI * f / FS;
        float d = fabsf(db_of(cabs(mb_response(&mb, w))));
        if (d > worst_db) worst_db = d;
    }
    printf("crossover_worst_dip_db=%.6f\n", worst_db);
    check("crossover_dip_under_1db", worst_db < 1.0f);

    /* phase continuity at the crossover: LP4 and HP4 in phase */
    int phase_ok = 1;
    for (int xi = 0; xi < 2; xi++) {
        double fc = xi == 0 ? FC1 : FC2;
        double w = 2.0 * PI * fc / FS;
        complex double lp = p4(xi == 0 ? mb.lp1[0] : mb.lp2[0], w);
        complex double hp = p4(xi == 0 ? mb.hp1[0] : mb.hp2[0], w);
        double d = carg(lp) - carg(hp);
        d = fmod(d + PI, 2.0 * PI) - PI;
        if (fabs(d) > 1e-3) phase_ok = 0;
    }
    check("crossover_phase_continuous", phase_ok);

    /* no phase jump anywhere: unwrapped reconstruction phase stays smooth */
    int smooth_ok = 1;
    double prev = carg(mb_response(&mb, 2.0 * PI * 20.0 / FS));
    double prev_slope = 0.0;
    for (int k = 1; k < NPT; k++) {
        double f = 20.0 * pow(1000.0, (double)k / (NPT - 1));
        double w = 2.0 * PI * f / FS;
        double ph = carg(mb_response(&mb, w));
        double d = ph - prev;
        while (d > PI) d -= 2 * PI;
        while (d < -PI) d += 2 * PI;
        if (fabs(d) > PI * 0.99) smooth_ok = 0;          /* unwrap failure */
        if (k > 1 && fabs(d - prev_slope) > 0.05) smooth_ok = 0;
        prev_slope = d;
        prev = ph;
    }
    check("phase_no_jump", smooth_ok);

    /* group delay of the reconstructed path, and zero-sample latency.
       The reconstruction response is AP2(fc1)*AP2(fc2), so its delay is
       pure filter group delay (no look-ahead anywhere). */
    float gd_max_ms = 0.0f;
    float gd_at[2] = { 0.0f, 0.0f };
    double peak_f = 0.0;
    for (int k = 0; k < NPT; k++) {
        double f = 20.0 * pow(1000.0, (double)k / (NPT - 1));
        double w = 2.0 * PI * f / FS;
        double dw = w * 1e-4 + 1e-9;
        double p1 = carg(mb_response(&mb, w - dw));
        double p2 = carg(mb_response(&mb, w + dw));
        double u = p2 - p1;
        while (u > PI) u -= 2 * PI;
        while (u < -PI) u += 2 * PI;
        float gd = (float)(-u / (2.0 * dw) * 1000.0 / FS);
        if (gd > gd_max_ms) { gd_max_ms = gd; peak_f = f; }
        if (fabs(f - FC1) < 3.0) gd_at[0] = gd;
        if (fabs(f - FC2) < 3.0) gd_at[1] = gd;
    }
    /* exact evaluations at the two crossover frequencies */
    for (int xi = 0; xi < 2; xi++) {
        double fc = xi == 0 ? FC1 : FC2;
        double w = 2.0 * PI * fc / FS, dw = w * 1e-4;
        double u = carg(mb_response(&mb, w + dw))
                 - carg(mb_response(&mb, w - dw));
        while (u > PI) u -= 2 * PI;
        while (u < -PI) u += 2 * PI;
        gd_at[xi] = (float)(-u / (2.0 * dw) * 1000.0 / FS);
    }
    printf("group_delay_at_fc1_ms=%.4f\n", gd_at[0]);
    printf("group_delay_at_fc2_ms=%.4f\n", gd_at[1]);
    printf("group_delay_max_ms=%.4f (at %.0f Hz)\n", gd_max_ms, peak_f);
    check("latency_under_2ms", gd_max_ms < 2.0f);

    float imp[2] = { 1.0f, 1.0f }, imp_out[2];
    mb_reset(&mb);
    mb.enabled = 0;
    mb_process(&mb, imp, imp_out, 1);
    mb.enabled = 1;
    check("zero_sample_latency", imp_out[0] != 0.0f && imp_out[1] != 0.0f);

    /* bypass path: broadband noise keeps its level and stays finite */
    const int SIG = 48000;
    float* dry = malloc(sizeof(float) * SIG * 2);
    float* buf = malloc(sizeof(float) * SIG * 2);
    unsigned seed = 12345;
    double e_in = 0, e_out = 0;
    int finite_ok = 1;
    mb_reset(&mb); mb.enabled = 0;
    for (int i = 0; i < SIG * 2; i++) {
        seed = seed * 1103515245u + 12345u;
        dry[i] = (float)((int)(seed >> 16) % 2001 - 1000) / 1000.0f;
    }
    mb_process(&mb, dry, buf, SIG);
    for (int i = 0; i < SIG * 2; i++) {
        e_in += (double)dry[i] * dry[i];
        e_out += (double)buf[i] * buf[i];
        if (!isfinite(buf[i])) finite_ok = 0;
    }
    float bypass_db = db_of(sqrt(e_out / e_in));
    printf("bypass_broadband_db=%.4f\n", bypass_db);
    check("bypass_flat", fabsf(bypass_db) < 0.05f);
    check("output_finite", finite_ok);
    mb.enabled = 1;

    /* ---------- spectral comparison: single-band vs multiband ---------- */
    const float tone_f[MB_BANDS] = { 80.0f, 1000.0f, 8000.0f };
    const float tone_a[MB_BANDS] = { 0.50f, 0.25f, 0.12f };
    for (int i = 0; i < SIG; i++) {
        for (int ch = 0; ch < 2; ch++) {
            float v = 0;
            for (int b = 0; b < MB_BANDS; b++)
                v += tone_a[b] * sinf(2.0f * PI * tone_f[b] * i / FS);
            dry[i * 2 + ch] = v;
        }
    }

    /* per-band dry RMS via a fresh splitter */
    Multiband an;
    mb_init(&an, FS, FC1, FC2, cfg);
    an.enabled = 0;
    double pre[MB_BANDS];
    mb_run(&an, dry, buf, SIG, pre, NULL);
    float dry_rms[MB_BANDS];
    for (int b = 0; b < MB_BANDS; b++)
        dry_rms[b] = db_of(sqrt(pre[b] / (2.0 * SIG)));

    /* single-band reference: low-band settings applied to the whole signal */
    BandCfg single = { -14.0f, 4.0f, 2.0f, 80.0f, 0.0f, 0 };
    float* sb = malloc(sizeof(float) * SIG * 2);
    single_band(&single, FS, dry, sb, SIG);
    mb_reset(&an);
    mb_run(&an, sb, buf, SIG, pre, NULL);
    float sb_rms[MB_BANDS];
    for (int b = 0; b < MB_BANDS; b++)
        sb_rms[b] = db_of(sqrt(pre[b] / (2.0 * SIG)));

    /* multiband: calibrate auto makeup, then process */
    mb_reset(&mb);
    mb_calibrate(&mb, dry, SIG);
    float* mbout = malloc(sizeof(float) * SIG * 2);
    mb_process(&mb, dry, mbout, SIG);
    mb_reset(&an);
    mb_run(&an, mbout, buf, SIG, pre, NULL);
    float mb_rms[MB_BANDS];
    for (int b = 0; b < MB_BANDS; b++)
        mb_rms[b] = db_of(sqrt(pre[b] / (2.0 * SIG)));

    printf("spectral_compare_db:\n");
    printf("  band  dry     single  multi   single_gr  multi_gr\n");
    int single_flattens = 1, multi_preserves = 1, independent = 1;
    for (int b = 0; b < MB_BANDS; b++) {
        float gr_s = dry_rms[b] - sb_rms[b];
        float gr_m = dry_rms[b] - mb_rms[b];
        printf("  %-4d  %6.2f  %6.2f  %6.2f  %9.2f  %8.2f\n",
               b, dry_rms[b], sb_rms[b], mb_rms[b], gr_s, gr_m);
        if (gr_s < 6.0f) single_flattens = 0;          /* every band crushed */
        if (fabsf(gr_m) > 1.5f) multi_preserves = 0;   /* makeup restores */
    }
    check("single_band_flattens_spectrum", single_flattens);
    check("multiband_preserves_balance", multi_preserves);
    check("multiband_beats_single_on_mid_high",
          (dry_rms[1] - sb_rms[1]) - (dry_rms[1] - mb_rms[1]) > 4.0f &&
          (dry_rms[2] - sb_rms[2]) - (dry_rms[2] - mb_rms[2]) > 4.0f);

    /* band independence: only the low band is driven, mid/high must stay
       at their (near-zero) dry level regardless of low-band compression */
    for (int i = 0; i < SIG; i++)
        for (int ch = 0; ch < 2; ch++)
            dry[i * 2 + ch] = 0.8f * sinf(2.0f * PI * 80.0f * i / FS);
    mb_reset(&mb);
    mb_calibrate(&mb, dry, SIG);
    mb_process(&mb, dry, mbout, SIG);
    mb_reset(&an);
    mb_run(&an, mbout, buf, SIG, pre, NULL);
    double leak_mid = sqrt(pre[1] / (2.0 * SIG));
    double leak_high = sqrt(pre[2] / (2.0 * SIG));
    printf("band_leak_mid=%.2f_dB band_leak_high=%.2f_dB\n",
           db_of(leak_mid), db_of(leak_high));
    if (db_of(leak_mid) > -40.0f || db_of(leak_high) > -40.0f) independent = 0;
    check("band_independence", independent);

    /* end-to-end chain: mix bus -> multiband -> soft clip stays bounded */
    for (int i = 0; i < FRAMES * 2; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.0f;
    }
    for (int s = 0; s < 3; s++) srcs[s].gain = 1.0f;
    srcs[0].bus = 0;
    float chain[FRAMES * 2];
    float busmix[FRAMES * 2];
    mix(&mx, srcs, 3, in, busmix, FRAMES);
    mb_reset(&mb);
    mb_process(&mb, busmix, chain, FRAMES);
    for (int i = 0; i < FRAMES * 2; i++) chain[i] = soft_clip(chain[i]);
    check("chain_bounded", peak(chain, FRAMES * 2) <= 1.0f + 1e-6f);

    free(dry); free(buf); free(sb); free(mbout);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
