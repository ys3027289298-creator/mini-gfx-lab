
/* mixer: multi-bus audio mixing with per-bus gain, sidechain ducking and
   soft clipping. All mix buffers are interleaved stereo.

   Also contains optional 5.1 bass management: every non-LFE channel is
   high-passed at the crossover, its low band is summed into LFE, and the
   LFE input passes through untouched. Crossovers are Linkwitz-Riley 2nd
   (12 dB/oct) or 4th (24 dB/oct) order. Disabled by default; the disabled
   path is a bit-exact copy. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_SRC 8
#define FRAMES 128

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

/* ------------------------------------------------------------------ */
/* 5.1 bass management                                                */
/* ------------------------------------------------------------------ */

#define CH_FL  0
#define CH_FR  1
#define CH_FC  2
#define CH_LFE 3
#define CH_SL  4
#define CH_SR  5
#define NCH    6

#define BM_PI    3.14159265358979323846
#define BM_SQRT2 1.41421356237309504880

/* A biquad; first-order sections use b2 = a2 = 0.
   Coefficients and states are double: at low crossovers the denominator
   terms nearly cancel (1 + a1 + a2 ~ 1e-4 at 80 Hz), and float rounding
   there would shift the response by ~1e-3. */
typedef struct {
    double b0, b1, b2, a1, a2;
} Biquad;

/* Two cascaded sections. LR2 = two 1st-order sections (Q = 0.5),
   LR4 = two 2nd-order Butterworth sections (Q = 1/sqrt(2)).
   Both give -6 dB per branch at the crossover; LR2 branches are 180
   degrees apart when summed, LR4 branches sum in phase to an allpass. */
typedef struct {
    Biquad sec[2];
    int nsec;
    double z1[2], z2[2];    /* transposed direct-form II states */
} Filter;

typedef struct {
    int enabled;
    int configured;
    float fs;
    float xover_hz;
    int slope_db_oct;       /* 12 (LR2) or 24 (LR4) */
    int route[NCH];         /* 1 = this channel's low band goes to LFE */
    float lfe_gain;
    Filter hp[NCH], lp[NCH];
} BassManager;

static Biquad bq_first_order(float fs, float fc, int highpass) {
    double k = tan(BM_PI * (double)fc / (double)fs);
    double a0 = 1.0 + k;
    Biquad b;
    b.b0 = highpass ? 1.0 / a0 : k / a0;
    b.b1 = highpass ? -1.0 / a0 : k / a0;
    b.b2 = 0.0;
    b.a1 = (k - 1.0) / a0;
    b.a2 = 0.0;
    return b;
}

static Biquad bq_butterworth2(float fs, float fc, int highpass) {
    double k = tan(BM_PI * (double)fc / (double)fs);
    double k2 = k * k;
    double a0 = 1.0 + BM_SQRT2 * k + k2;
    Biquad b;
    if (highpass) {
        b.b0 = 1.0 / a0;
        b.b1 = -2.0 / a0;
        b.b2 = 1.0 / a0;
    } else {
        b.b0 = k2 / a0;
        b.b1 = 2.0 * k2 / a0;
        b.b2 = k2 / a0;
    }
    b.a1 = 2.0 * (k2 - 1.0) / a0;
    b.a2 = (1.0 - BM_SQRT2 * k + k2) / a0;
    return b;
}

static void filter_clear(Filter* f) {
    memset(f->z1, 0, sizeof(f->z1));
    memset(f->z2, 0, sizeof(f->z2));
}

static void filter_build(Filter* f, float fs, float fc, int highpass,
                         int slope_db_oct) {
    memset(f, 0, sizeof(*f));
    Biquad s = (slope_db_oct == 12) ? bq_first_order(fs, fc, highpass)
                                    : bq_butterworth2(fs, fc, highpass);
    f->sec[0] = s;
    f->sec[1] = s;
    f->nsec = 2;
}

static double biquad_run(const Biquad* b, double* z1, double* z2, double x) {
    double y = b->b0 * x + *z1;
    *z1 = b->b1 * x - b->a1 * y + *z2;
    *z2 = b->b2 * x - b->a2 * y;
    return y;
}

static double filter_run(Filter* f, double x) {
    for (int i = 0; i < f->nsec; i++)
        x = biquad_run(&f->sec[i], &f->z1[i], &f->z2[i], x);
    return x;
}

/* Complex response of a section at w = 2*pi*f/fs. Double precision matters:
   at 80 Hz the denominator terms nearly cancel, and float rounding would
   show up as millidecibels of error. */
static void biquad_resp(const Biquad* b, double w, double* re, double* im) {
    double c1 = cos(w), s1 = -sin(w);
    double c2 = cos(2.0 * w), s2 = -sin(2.0 * w);
    double nr = b->b0 + b->b1 * c1 + b->b2 * c2;
    double ni = b->b1 * s1 + b->b2 * s2;
    double dr = 1.0 + b->a1 * c1 + b->a2 * c2;
    double di = b->a1 * s1 + b->a2 * s2;
    double d = dr * dr + di * di;
    *re = (nr * dr + ni * di) / d;
    *im = (ni * dr - nr * di) / d;
}

static void filter_resp(const Filter* f, float fs, float hz,
                        double* re, double* im) {
    double w = 2.0 * BM_PI * (double)hz / (double)fs;
    *re = 1.0;
    *im = 0.0;
    for (int i = 0; i < f->nsec; i++) {
        double sr, si;
        biquad_resp(&f->sec[i], w, &sr, &si);
        double r = *re * sr - *im * si;
        double m = *re * si + *im * sr;
        *re = r;
        *im = m;
    }
}

/* Reject an unusable configuration: fs must be positive, crossover must be
   strictly below Nyquist, slope must be 12 or 24 dB/oct. On failure the
   manager stays disabled and configured = 0. */
static int bm_init(BassManager* bm, float fs, float xover_hz, int slope_db_oct) {
    memset(bm, 0, sizeof(*bm));
    bm->lfe_gain = 1.0f;
    for (int c = 0; c < NCH; c++)
        bm->route[c] = (c == CH_LFE) ? 0 : 1;
    if (!(fs > 0.0f) || !(xover_hz > 0.0f) || xover_hz >= fs * 0.5f)
        return -1;
    if (slope_db_oct != 12 && slope_db_oct != 24)
        return -1;
    bm->fs = fs;
    bm->xover_hz = xover_hz;
    bm->slope_db_oct = slope_db_oct;
    for (int c = 0; c < NCH; c++) {
        filter_build(&bm->hp[c], fs, xover_hz, 1, slope_db_oct);
        filter_build(&bm->lp[c], fs, xover_hz, 0, slope_db_oct);
    }
    bm->configured = 1;
    return 0;
}

static void bm_reset(BassManager* bm) {
    for (int c = 0; c < NCH; c++) {
        filter_clear(&bm->hp[c]);
        filter_clear(&bm->lp[c]);
    }
}

static void bm_set_enabled(BassManager* bm, int on) {
    if (!bm->configured) { bm->enabled = 0; return; }
    if (on && !bm->enabled) bm_reset(bm);
    bm->enabled = on ? 1 : 0;
}

static void bm_set_route(BassManager* bm, int ch, int on) {
    if (ch >= 0 && ch < NCH && ch != CH_LFE)
        bm->route[ch] = on ? 1 : 0;
}

static void copy_or_zero(float* dst, const float* src, int n) {
    if (!dst) return;
    if (src) memcpy(dst, src, sizeof(float) * (size_t)n);
    else memset(dst, 0, sizeof(float) * (size_t)n);
}

/* in/out have NCH channels of `frames` samples each; lfe_out is a mono bus.
   out[CH_LFE] and lfe_out carry the LFE input plus the routed low bands.
   Non-routed channels are still high-passed, as in a normal bass manager. */
static void bm_process(BassManager* bm, const float* const* in,
                       float* const* out, float* lfe_out, int frames) {
    if (!bm->enabled) {
        for (int c = 0; c < NCH; c++)
            copy_or_zero(out[c], in[c], frames);
        copy_or_zero(lfe_out, in[CH_LFE], frames);
        return;
    }
    for (int i = 0; i < frames; i++) {
        double low = in[CH_LFE] ? (double)in[CH_LFE][i] : 0.0;
        for (int c = 0; c < NCH; c++) {
            if (c == CH_LFE) {          /* raw LFE passes through */
                if (out[c]) out[c][i] = (float)low;
                continue;
            }
            double x = in[c] ? (double)in[c][i] : 0.0;
            double h = filter_run(&bm->hp[c], x);
            if (out[c]) out[c][i] = (float)h;
            if (bm->route[c])
                low += (double)bm->lfe_gain * filter_run(&bm->lp[c], x);
        }
        if (lfe_out) lfe_out[i] = (float)low;
    }
}

/* Magnitude (dB) and relative phase of LP vs HP at the crossover. */
static void bm_crossover_report(const BassManager* bm, int ch,
                                float* lp_db, float* hp_db, float* phase_deg) {
    double lr, li, hr, hi;
    filter_resp(&bm->lp[ch], bm->fs, bm->xover_hz, &lr, &li);
    filter_resp(&bm->hp[ch], bm->fs, bm->xover_hz, &hr, &hi);
    *lp_db = (float)(20.0 * log10(hypot(lr, li)));
    *hp_db = (float)(20.0 * log10(hypot(hr, hi)));
    double re = lr * hr + li * hi;      /* Re(LP * conj(HP)) */
    double im = li * hr - lr * hi;      /* Im(LP * conj(HP)) */
    *phase_deg = (float)(atan2(im, re) * 180.0 / BM_PI);
}

/* Reference implementation for the self-test: independent filter topology
   (direct form I) and independent coefficient derivation (RBJ cookbook).
   Used to check the bass manager sample by sample. */
typedef struct { double x1, x2, y1, y2; } Df1State;

typedef struct {
    Biquad sec[2];
    int nsec;
    Df1State st[2];
} RefFilter;

static double biquad_df1(const Biquad* b, Df1State* s, double x) {
    double y = (double)b->b0 * x + (double)b->b1 * s->x1 + (double)b->b2 * s->x2
             - (double)b->a1 * s->y1 - (double)b->a2 * s->y2;
    s->x2 = s->x1; s->x1 = x;
    s->y2 = s->y1; s->y1 = y;
    return y;
}

static double ref_run(RefFilter* f, double x) {
    for (int i = 0; i < f->nsec; i++)
        x = biquad_df1(&f->sec[i], &f->st[i], x);
    return x;
}

static void ref_build(RefFilter* f, float fs, float fc, int highpass, int slope) {
    memset(f, 0, sizeof(*f));
    f->nsec = 2;
    Biquad b;
    memset(&b, 0, sizeof(b));
    if (slope == 12) {
        double k = tan(BM_PI * (double)fc / (double)fs);
        double a0 = 1.0 + k;
        b.b0 = highpass ? 1.0 / a0 : k / a0;
        b.b1 = highpass ? -1.0 / a0 : k / a0;
        b.a1 = (k - 1.0) / a0;
    } else {
        double w0 = 2.0 * BM_PI * (double)fc / (double)fs;
        double cw = cos(w0), sw = sin(w0);
        double alpha = sw / (2.0 * (1.0 / BM_SQRT2));
        double a0 = 1.0 + alpha;
        if (highpass) {
            b.b0 = (1.0 + cw) * 0.5 / a0;
            b.b1 = -(1.0 + cw) / a0;
            b.b2 = b.b0;
        } else {
            b.b0 = (1.0 - cw) * 0.5 / a0;
            b.b1 = (1.0 - cw) / a0;
            b.b2 = b.b0;
        }
        b.a1 = -2.0 * cw / a0;
        b.a2 = (1.0 - alpha) / a0;
    }
    f->sec[0] = b;
    f->sec[1] = b;
}

static void manual_bass(const float* const* in, int frames,
                        float fs, float xover, int slope, float lfe_gain,
                        const int* route, float* lfe_out, float* const* out) {
    RefFilter hp[NCH], lp[NCH];
    for (int c = 0; c < NCH; c++) {
        ref_build(&hp[c], fs, xover, 1, slope);
        ref_build(&lp[c], fs, xover, 0, slope);
    }
    for (int i = 0; i < frames; i++) {
        double low = in[CH_LFE] ? (double)in[CH_LFE][i] : 0.0;
        for (int c = 0; c < NCH; c++) {
            if (c == CH_LFE) { if (out[c]) out[c][i] = (float)low; continue; }
            double x = in[c] ? (double)in[c][i] : 0.0;
            if (out[c]) out[c][i] = (float)ref_run(&hp[c], x);
            if (route[c]) low += (double)lfe_gain * ref_run(&lp[c], x);
        }
        if (lfe_out) lfe_out[i] = (float)low;
    }
}

/* ------------------------------------------------------------------ */
/* self-test                                                          */
/* ------------------------------------------------------------------ */

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

static double rms_from(const float* x, int from, int to) {
    double e = 0.0;
    for (int i = from; i < to; i++) e += (double)x[i] * (double)x[i];
    return sqrt(e / (double)(to - from));
}

#define BM_N 9600
static float bm_in[NCH][BM_N];
static float bm_out[NCH][BM_N];
static float bm_lfe[BM_N];

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

    /* ---------------- bass management ---------------- */

    BassManager bm;
    check("xover_at_nyquist_rejected",
          bm_init(&bm, 48000.0f, 24000.0f, 24) != 0);
    check("xover_above_nyquist_rejected",
          bm_init(&bm, 48000.0f, 30000.0f, 24) != 0);
    check("xover_just_below_nyquist_ok",
          bm_init(&bm, 48000.0f, 23999.0f, 24) == 0);
    check("bad_slope_rejected",
          bm_init(&bm, 48000.0f, 80.0f, 16) != 0);
    check("bm_init_ok", bm_init(&bm, 48000.0f, 80.0f, 24) == 0);

    /* disabled: every channel is a bit-exact copy of the input */
    for (int i = 0; i < BM_N; i++) {
        for (int c = 0; c < NCH; c++)
            bm_in[c][i] = (float)((c + 1) * 0.01 * sin(0.01 * i + c));
    }
    const float* bin[NCH];
    float* bout[NCH];
    for (int c = 0; c < NCH; c++) { bin[c] = bm_in[c]; bout[c] = bm_out[c]; }

    bm_set_enabled(&bm, 0);
    bm_process(&bm, bin, bout, bm_lfe, 512);
    int same = 1;
    for (int c = 0; c < NCH; c++)
        if (memcmp(bm_out[c], bm_in[c], sizeof(float) * 512) != 0) same = 0;
    if (memcmp(bm_lfe, bm_in[CH_LFE], sizeof(float) * 512) != 0) same = 0;
    check("disabled_bit_exact", same);

    /* per-band manual split, sample by sample */
    for (int i = 0; i < BM_N; i++) {
        double t = (double)i / 48000.0;
        double ph = 2.0 * 3.14159265358979323846 * t;
        bm_in[CH_FL][i] = (float)(0.5 * sin(ph * 40.0) + 0.2 * sin(ph * 1000.0));
        bm_in[CH_FR][i] = (float)(0.5 * sin(ph * 55.0) + 0.2 * sin(ph * 5000.0));
        bm_in[CH_FC][i] = (float)(0.4 * sin(ph * 80.0));
        bm_in[CH_LFE][i] = (float)(0.3 * sin(ph * 30.0));
        bm_in[CH_SL][i] = (float)(0.4 * sin(ph * 120.0));
        bm_in[CH_SR][i] = (float)(0.4 * sin(ph * 35.0) + 0.1 * sin(ph * 3000.0));
    }

    static float ref_out[NCH][512];
    static float ref_lfe[512];
    float* rout[NCH];
    for (int c = 0; c < NCH; c++) { bout[c] = bm_out[c]; rout[c] = ref_out[c]; }

    bm_set_enabled(&bm, 1);
    bm_reset(&bm);
    bm_process(&bm, bin, bout, bm_lfe, 512);
    manual_bass(bin, 512, bm.fs, bm.xover_hz, bm.slope_db_oct,
                bm.lfe_gain, bm.route, ref_lfe, rout);

    float maxd = 0.0f;
    for (int c = 0; c < NCH; c++)
        for (int i = 0; i < 512; i++) {
            float d = fabsf(bm_out[c][i] - ref_out[c][i]);
            if (d > maxd) maxd = d;
        }
    for (int i = 0; i < 512; i++) {
        float d = fabsf(bm_lfe[i] - ref_lfe[i]);
        if (d > maxd) maxd = d;
    }
    printf("manual_split_maxdiff=%.3e\n", (double)maxd);
    check("manual_split_sample_exact", maxd < 1e-5f);

    /* same comparison at a different slope and crossover */
    BassManager bm2;
    check("bm_init_slope12_ok", bm_init(&bm2, 48000.0f, 120.0f, 12) == 0);
    bm_set_enabled(&bm2, 1);
    bm_reset(&bm2);
    bm_process(&bm2, bin, bout, bm_lfe, 512);
    manual_bass(bin, 512, bm2.fs, bm2.xover_hz, bm2.slope_db_oct,
                bm2.lfe_gain, bm2.route, ref_lfe, rout);
    maxd = 0.0f;
    for (int c = 0; c < NCH; c++)
        for (int i = 0; i < 512; i++) {
            float d = fabsf(bm_out[c][i] - ref_out[c][i]);
            if (d > maxd) maxd = d;
        }
    for (int i = 0; i < 512; i++) {
        float d = fabsf(bm_lfe[i] - ref_lfe[i]);
        if (d > maxd) maxd = d;
    }
    printf("manual_split_maxdiff_slope12=%.3e\n", (double)maxd);
    check("manual_split_sample_exact_slope12", maxd < 1e-5f);

    /* crossover magnitude and relative phase */
    float lp_db, hp_db, phase;
    bm_crossover_report(&bm, CH_FL, &lp_db, &hp_db, &phase);
    printf("lr4_fc=80 lp=%.2fdB hp=%.2fdB phase_diff=%.2fdeg\n",
           (double)lp_db, (double)hp_db, (double)phase);
    check("lr4_minus6db_at_fc",
          fabsf(lp_db + 6.0206f) < 0.05f && fabsf(hp_db + 6.0206f) < 0.05f);
    check("lr4_branches_in_phase", fabsf(phase) < 0.5f);

    bm_crossover_report(&bm2, CH_FL, &lp_db, &hp_db, &phase);
    printf("lr2_fc=120 lp=%.2fdB hp=%.2fdB phase_diff=%.2fdeg\n",
           (double)lp_db, (double)hp_db, (double)phase);
    check("lr2_minus6db_at_fc",
          fabsf(lp_db + 6.0206f) < 0.05f && fabsf(hp_db + 6.0206f) < 0.05f);
    check("lr2_phase_diff_180", fabsf(fabsf(phase) - 180.0f) < 0.5f);

    /* LP + HP is an allpass for LR4: flat magnitude everywhere */
    double worst = 0.0;
    float probes[8] = { 20.0f, 40.0f, 80.0f, 160.0f, 320.0f, 1000.0f, 5000.0f, 15000.0f };
    for (int i = 0; i < 8; i++) {
        double lre, lim, hre, him;
        filter_resp(&bm.lp[CH_FL], bm.fs, probes[i], &lre, &lim);
        filter_resp(&bm.hp[CH_FL], bm.fs, probes[i], &hre, &him);
        double mag = hypot(lre + hre, lim + him);
        double err = fabs(mag - 1.0);
        if (err > worst) worst = err;
    }
    printf("lr4_allpass_max_err=%.3e\n", worst);
    check("lr4_sum_allpass_flat", worst < 1e-4);

    /* configurable slope: 12 dB/oct leaks more just above the crossover */
    double l4r, l4i, l2r, l2i;
    filter_resp(&bm.lp[CH_FL], bm.fs, 160.0f, &l4r, &l4i);
    filter_resp(&bm2.lp[CH_FL], bm2.fs, 240.0f, &l2r, &l2i);
    double l4db = 20.0 * log10(hypot(l4r, l4i));
    double l2db = 20.0 * log10(hypot(l2r, l2i));
    printf("lp_at_2fc lr4=%.2fdB lr2=%.2fdB\n", l4db, l2db);
    check("slope_steeper_when_lr4", l4db < l2db - 5.0);

    /* LFE extraction and high-passing of a 40 Hz tone */
    for (int i = 0; i < BM_N; i++) {
        for (int c = 0; c < NCH; c++) bm_in[c][i] = 0.0f;
        bm_in[CH_FL][i] = (float)(0.5 * sin(2.0 * 3.14159265358979323846 *
                                            40.0 * (double)i / 48000.0));
    }
    BassManager bm3;
    bm_init(&bm3, 48000.0f, 80.0f, 24);
    bm_set_enabled(&bm3, 1);
    bm_reset(&bm3);
    bm_process(&bm3, bin, bout, bm_lfe, BM_N);
    double in_rms = rms_from(bm_in[CH_FL], 1200, BM_N);
    double hp_rms = rms_from(bm_out[CH_FL], 1200, BM_N);
    double lfe_rms = rms_from(bm_lfe, 1200, BM_N);
    printf("tone40 hp_ratio=%.4f lfe_ratio=%.4f\n",
           hp_rms / in_rms, lfe_rms / in_rms);
    check("low_tone_highpassed", hp_rms / in_rms < 0.1);
    check("low_tone_into_lfe", lfe_rms / in_rms > 0.8);

    /* routing: center is not sent unless routed */
    for (int i = 0; i < BM_N; i++) {
        bm_in[CH_FL][i] = 0.0f;
        bm_in[CH_FC][i] = bm_in[CH_LFE][i] = 0.0f;
        bm_in[CH_FC][i] = (float)(0.5 * sin(2.0 * 3.14159265358979323846 *
                                            40.0 * (double)i / 48000.0));
    }
    for (int c = 0; c < NCH; c++) bm_set_route(&bm3, c, 0);
    bm_reset(&bm3);
    bm_process(&bm3, bin, bout, bm_lfe, BM_N);
    double un_routed = rms_from(bm_lfe, 1200, BM_N);
    bm_set_route(&bm3, CH_FC, 1);
    bm_reset(&bm3);
    bm_process(&bm3, bin, bout, bm_lfe, BM_N);
    double routed = rms_from(bm_lfe, 1200, BM_N);
    printf("route_center lfe_off=%.6f lfe_on=%.6f\n", un_routed, routed);
    check("unrouted_center_not_in_lfe", un_routed < 1e-6);
    check("routed_center_into_lfe", routed > 0.3);

    /* LFE input passes through untouched (the other channels are silent) */
    for (int i = 0; i < BM_N; i++) {
        for (int c = 0; c < NCH; c++) bm_in[c][i] = 0.0f;
        bm_in[CH_LFE][i] = (float)(0.1 * cos(0.03 * i) + 0.05 * sin(0.11 * i));
    }
    for (int c = 0; c < NCH; c++) bm_set_route(&bm3, c, 1);
    bm_reset(&bm3);
    bm_process(&bm3, bin, bout, bm_lfe, BM_N);
    check("lfe_input_passthrough_exact",
          memcmp(bm_lfe, bm_in[CH_LFE], sizeof(float) * BM_N) == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
