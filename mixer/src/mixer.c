
/* mixer: multi-bus audio mixing with per-bus gain, sidechain ducking,
   soft clipping and a 3-band (low/mid/high) master equalizer.
   All buffers are interleaved stereo.

   The EQ splits the master signal with two 4th-order Linkwitz-Riley
   crossovers (each half is a pair of cascaded 2nd-order Butterworth
   biquads, Q = 1/sqrt(2)).  For an LR4 split |L|^2 + |H|^2 = 1 and the
   two halves sum to an all-pass, so at unity band gains the three bands
   reconstruct the input and no band ever flips polarity -- no section
   negates its signal, unlike the 2nd-order LR2 split which needs one
   side inverted. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define MAX_SRC 8
#define FRAMES 128
#define EQ_BANDS 3
#define EQ_STAGES 8            /* 4 biquads per crossover: 2x LP + 2x HP */
#define EQ_DEFAULT_SR 44100.0
#define EQ_DEFAULT_FLOW 250.0
#define EQ_DEFAULT_FHIGH 4000.0

typedef struct {
    int bus;        /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
} Source;

/* One RBJ-cookbook biquad.  Cascading two identical Butterworth stages
   gives the Linkwitz-Riley 4th-order response. */
typedef struct {
    float b0, b1, b2, a1, a2;
} Biquad;

typedef struct {
    float z1, z2;      /* transposed direct-form II state */
} BqState;

typedef struct {
    float gain_db[EQ_BANDS];   /* 0 dB = unity */
    float flow, fhigh;         /* crossover frequencies, Hz */
    float sr;
    Biquad bq[EQ_STAGES];      /* shared by both channels */
    BqState st[2][EQ_STAGES];  /* per-channel filter state */
} Eq3;

typedef struct {
    float gain[3];
    float duck_depth;    /* 0..1, applied to bus 0 when voice is active */
    int duck_enabled;
    Eq3 eq;
} Mixer;

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

/* Coefficients are designed in double precision so the realised response
   stays tight against its analytic value, then stored as float for the
   per-sample path. */
static Biquad make_biquad(int highpass, double freq, double sr) {
    double w0 = 2.0 * 3.14159265358979323846 * freq / sr;
    double cw = cos(w0), sw = sin(w0);
    double alpha = sw / (2.0 * 0.70710678118654752440);  /* Q = 1/sqrt(2) */
    double b0, b1, b2, a0, a1, a2;
    if (highpass) {
        b0 = (1.0 + cw) / 2.0; b1 = -(1.0 + cw); b2 = (1.0 + cw) / 2.0;
    } else {
        b0 = (1.0 - cw) / 2.0; b1 = 1.0 - cw;     b2 = (1.0 - cw) / 2.0;
    }
    a0 = 1.0 + alpha; a1 = -2.0 * cw; a2 = 1.0 - alpha;
    Biquad q;
    q.b0 = (float)(b0 / a0); q.b1 = (float)(b1 / a0); q.b2 = (float)(b2 / a0);
    q.a1 = (float)(a1 / a0); q.a2 = (float)(a2 / a0);
    return q;
}

static int eq_set_crossovers(Eq3* eq, float flow, float fhigh, float sr) {
    if (!(sr > 0.0f) || !(flow > 0.0f && fhigh > flow) ||
        !(fhigh < sr * 0.5f))
        return 0;
    eq->sr = sr;
    eq->flow = flow;
    eq->fhigh = fhigh;
    /* low crossover: LP4 -> low band, HP4 -> mid/high path */
    eq->bq[0] = eq->bq[1] = make_biquad(0, flow, sr);
    eq->bq[2] = eq->bq[3] = make_biquad(1, flow, sr);
    /* high crossover: LP4 -> mid band, HP4 -> high band */
    eq->bq[4] = eq->bq[5] = make_biquad(0, fhigh, sr);
    eq->bq[6] = eq->bq[7] = make_biquad(1, fhigh, sr);
    memset(eq->st, 0, sizeof(eq->st));
    return 1;
}

static void eq_init(Eq3* eq, float sr) {
    memset(eq, 0, sizeof(*eq));
    if (!eq_set_crossovers(eq, EQ_DEFAULT_FLOW, EQ_DEFAULT_FHIGH,
                           sr > 0.0f ? sr : EQ_DEFAULT_SR))
        eq_set_crossovers(eq, EQ_DEFAULT_FLOW, EQ_DEFAULT_FHIGH,
                          EQ_DEFAULT_SR);
}

static inline float bq_process(const Biquad* q, BqState* s, float x) {
    float y = q->b0 * x + s->z1;
    s->z1 = q->b1 * x - q->a1 * y + s->z2;
    s->z2 = q->b2 * x - q->a2 * y;
    return y;
}

/* Filter every sample through the band split.  The unity bypass lives in
   eq_process so the production 0 dB path never touches filter arithmetic. */
static void eq_run(Eq3* eq, const float* in, float* out, int frames,
                   int bypass_at_unity) {
    int unity = eq->gain_db[0] == 0.0f && eq->gain_db[1] == 0.0f
             && eq->gain_db[2] == 0.0f;
    float g[EQ_BANDS];
    for (int b = 0; b < EQ_BANDS; b++)
        g[b] = unity ? 1.0f : powf(10.0f, eq->gain_db[b] / 20.0f);

    for (int n = 0; n < frames; n++) {
        for (int ch = 0; ch < 2; ch++) {
            float x = in[n * 2 + ch];
            BqState* s = eq->st[ch];
            float low  = bq_process(&eq->bq[1], &s[1],
                                    bq_process(&eq->bq[0], &s[0], x));
            float high = bq_process(&eq->bq[3], &s[3],
                                    bq_process(&eq->bq[2], &s[2], x));
            float mid  = bq_process(&eq->bq[5], &s[5],
                                    bq_process(&eq->bq[4], &s[4], high));
            float hi   = bq_process(&eq->bq[7], &s[7],
                                    bq_process(&eq->bq[6], &s[6], high));
            /* no stage negates a signal: band polarity matches the input */
            float y = low * g[0] + mid * g[1] + hi * g[2];
            out[n * 2 + ch] = (unity && bypass_at_unity) ? x : y;
        }
    }
}

/* Process `frames` interleaved-stereo frames.  The two LR4 crossovers are
   cascaded: the low crossover splits the input into low and high, the high
   crossover splits that high path into mid and high.  When every band gain
   is 0 dB the input is copied through untouched (no filter arithmetic at
   all), so the output matches the pre-EQ mix path sample for sample. */
static void eq_process(Eq3* eq, const float* in, float* out, int frames) {
    eq_run(eq, in, out, frames, 1);
}

static void mix(Mixer* mx, const Source* srcs, int n,
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

    float pre[FRAMES * 2] = {0};
    for (int i = 0; i < frames * 2; i++) {
        pre[i] = bus[0][i] * mx->gain[0] * dg
               + bus[1][i] * mx->gain[1]
               + bus[2][i] * mx->gain[2];
    }
    eq_process(&mx->eq, pre, out, frames);
    for (int i = 0; i < frames * 2; i++)
        out[i] = soft_clip(out[i]);
}

static float peak(const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(b[i]) > m) m = fabsf(b[i]);
    return m;
}

/* ---------------- self test ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Original mix path, kept as the bit-exact reference for the 0 dB case. */
static void mix_reference(const Mixer* mx, const Source* srcs, int n,
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

/* Complex frequency response of one biquad, evaluated in double precision
   from the stored (float) coefficients. */
static void bq_response(const Biquad* q, double w, double* re, double* im) {
    double c1 = cos(w), s1 = sin(w);
    double c2 = c1 * c1 - s1 * s1, s2 = 2.0 * c1 * s1;
    double num_re = q->b0 + q->b1 * c1 + q->b2 * c2;
    double num_im =       - q->b1 * s1 - q->b2 * s2;
    double den_re = 1.0 + q->a1 * c1 + q->a2 * c2;
    double den_im =       - q->a1 * s1 - q->a2 * s2;
    double d2 = den_re * den_re + den_im * den_im;
    *re = (num_re * den_re + num_im * den_im) / d2;
    *im = (num_im * den_re - num_re * den_im) / d2;
}

static void cmul(double ar, double ai, double br, double bi,
                 double* rr, double* ri) {
    *rr = ar * br - ai * bi;
    *ri = ar * bi + ai * br;
}

/* Analytic response of the whole EQ:
   g0*LP4low + g1*LP4high*HP4low + g2*HP4high*HP4low.
   The two crossovers are cascaded, matching eq_process.  Evaluated from
   the actual stored filter coefficients. */
static void eq_analytic(const Eq3* eq, double f,
                        double* re, double* im) {
    double w = 2.0 * 3.14159265358979323846 * f / eq->sr;
    double h_re, h_im, t_re, t_im, acc_re = 0.0, acc_im = 0.0;
    double g[EQ_BANDS];
    for (int b = 0; b < EQ_BANDS; b++)
        g[b] = pow(10.0, eq->gain_db[b] / 20.0);

    bq_response(&eq->bq[0], w, &h_re, &h_im);
    cmul(h_re, h_im, h_re, h_im, &t_re, &t_im);       /* LP4 low xo */
    acc_re += g[0] * t_re; acc_im += g[0] * t_im;

    bq_response(&eq->bq[2], w, &h_re, &h_im);
    cmul(h_re, h_im, h_re, h_im, &t_re, &t_im);       /* HP4 low xo */
    double hp_re = t_re, hp_im = t_im;

    bq_response(&eq->bq[4], w, &h_re, &h_im);
    cmul(h_re, h_im, h_re, h_im, &t_re, &t_im);       /* LP4 high xo */
    cmul(hp_re, hp_im, t_re, t_im, &t_re, &t_im);
    acc_re += g[1] * t_re; acc_im += g[1] * t_im;

    bq_response(&eq->bq[6], w, &h_re, &h_im);
    cmul(h_re, h_im, h_re, h_im, &t_re, &t_im);       /* HP4 high xo */
    cmul(hp_re, hp_im, t_re, t_im, &t_re, &t_im);
    acc_re += g[2] * t_re; acc_im += g[2] * t_im;

    *re = acc_re; *im = acc_im;
}

/* Steady-state measured response at the exact DFT bin f = sr/N (so no
   grid-mismatch leakage): drive an integer-period cosine through the
   filter, discard the settling region, correlate one full period.  With
   x[n] = cos(w n) the DFT bin gives H = C - jS. */
static void eq_measure(Eq3* eq, int period, double* re, double* im) {
    int discard = 4000;
    int total = discard + period;
    float* mem = (float*)calloc((size_t)total * 4, sizeof(float));
    if (!mem) { *re = *im = 0.0; return; }
    float* buf = mem;
    float* out = mem + (size_t)total * 2;
    double f = eq->sr / period;
    for (int n = 0; n < total; n++) {
        float v = (float)cos(2.0 * 3.14159265358979323846 * f * n / eq->sr);
        buf[n * 2] = v;
        buf[n * 2 + 1] = v;
    }
    eq_run(eq, buf, out, total, 0);
    double c = 0.0, s = 0.0;
    for (int n = discard; n < total; n++) {
        double ang = 2.0 * 3.14159265358979323846 * f * n / eq->sr;
        c += out[n * 2] * cos(ang);
        s += out[n * 2] * sin(ang);
    }
    c = 2.0 * c / period;
    s = 2.0 * s / period;
    *re = c;
    *im = -s;
    free(mem);
}

static double to_db(double mag) { return 20.0 * log10(mag); }
static double wrap_pi(double a) {
    while (a > 3.14159265358979323846) a -= 2.0 * 3.14159265358979323846;
    while (a < -3.14159265358979323846) a += 2.0 * 3.14159265358979323846;
    return a;
}

int main(void) {
    float sfx[FRAMES * 2], music[FRAMES * 2], voice[FRAMES * 2],
          out[FRAMES * 2], ref[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.0f;
    }
    const float* in[3] = { sfx, music, voice };
    Source srcs[3] = { {0, 1.0f}, {1, 1.0f}, {2, 1.0f} };
    Mixer mx;
    memset(&mx, 0, sizeof(mx));
    mx.gain[0] = mx.gain[1] = mx.gain[2] = 1.0f;
    mx.duck_depth = 0.5f;
    mx.duck_enabled = 1;
    eq_init(&mx.eq, EQ_DEFAULT_SR);

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

    /* --- all bands 0 dB: output must equal the original path sample
           for sample, including across blocks and with ducking active --- */
    srcs[0].bus = 0;
    for (int s = 0; s < 3; s++) srcs[s].gain = 1.0f;
    unsigned int seed = 12345u;
    int bitexact = 1;
    for (int trial = 0; trial < 4 && bitexact; trial++) {
        mx.duck_enabled = trial & 1;
        for (int i = 0; i < FRAMES * 2; i++) {
            seed = seed * 1103515245u + 12345u;
            sfx[i] = (float)((int)(seed >> 16) - 32768) / 32768.0f * 0.6f;
            seed = seed * 1103515245u + 12345u;
            music[i] = (float)((int)(seed >> 16) - 32768) / 32768.0f * 0.4f;
            seed = seed * 1103515245u + 12345u;
            voice[i] = (float)((int)(seed >> 16) - 32768) / 32768.0f * 0.3f;
        }
        for (int blk = 0; blk < 4; blk++) {
            mix(&mx, srcs, 3, in, out, FRAMES);
            mix_reference(&mx, srcs, 3, in, ref, FRAMES);
            if (memcmp(out, ref, sizeof(out)) != 0) { bitexact = 0; break; }
        }
    }
    check("unity_bit_exact", bitexact);

    /* --- frequency response: four point groups, measured against the
           analytic transfer function at several gain settings.  Points are
           exact DFT bins (period N samples, f = sr/N) spanning the low
           band, both crossover neighbourhoods, the mid band, the high
           band and the near-Nyquist edge. --- */
    static const int groups[4][3] = {
        { 1102,  551, 294 },  /* 40 / 80 / 150 Hz: low band interior */
        {  176,   11,  44 },  /* 250 (low xo) / 4010 (high xo) / 1002 Hz */
        {    7,    5,   4 },  /* 6300 / 8820 / 11025 Hz: high band */
        { 2205,  176,   3 }   /* 20 Hz edge, low xo, 14700 Hz near Nyquist */
    };
    static const double gain_sets[][EQ_BANDS] = {
        { 0, 0, 0 }, { 6, -6, 3 }, { -12, 9, -3 }, { 3, 3, 3 }
    };
    double worst_db = 0.0, worst_phase = 0.0;
    int resp_ok = 1, polarity_ok = 1;
    for (int gs = 0; gs < 4; gs++) {
        for (int b = 0; b < EQ_BANDS; b++)
            mx.eq.gain_db[b] = (float)gain_sets[gs][b];
        for (int grp = 0; grp < 4; grp++) {
            for (int k = 0; k < 3; k++) {
                int period = groups[grp][k];
                double f = mx.eq.sr / period;
                double mre, mim, are, aim;
                eq_measure(&mx.eq, period, &mre, &mim);
                eq_analytic(&mx.eq, f, &are, &aim);
                double dm = to_db(hypot(mre, mim)) - to_db(hypot(are, aim));
                double dp = fabs(wrap_pi(atan2(mim, mre) - atan2(aim, are)));
                if (fabs(dm) > worst_db) worst_db = fabs(dm);
                if (dp > worst_phase) worst_phase = dp;
                if (fabs(dm) > 0.5 || dp > 0.1) resp_ok = 0;
            }
        }
    }
    printf("resp_max_db_err=%.4f\n", worst_db);
    printf("resp_max_phase_err=%.5f\n", worst_phase);
    check("response_within_0p5db", resp_ok);

    /* --- polarity: inside each pass band the measured response must be
           in phase with the input (no +/-180 degree inversion) --- */
    static const int band_probe[EQ_BANDS] = { 551, 44, 5 };  /* 80/1002/8820 Hz */
    for (int b = 0; b < EQ_BANDS; b++) {
        for (int x = 0; x < EQ_BANDS; x++) mx.eq.gain_db[x] = 0.0f;
        mx.eq.gain_db[b] = 6.0f;
        double mre, mim, are, aim;
        eq_measure(&mx.eq, band_probe[b], &mre, &mim);
        eq_analytic(&mx.eq, mx.eq.sr / band_probe[b], &are, &aim);
        double pm = atan2(mim, mre), pa = atan2(aim, are);
        if (fabs(wrap_pi(pm)) > 3.14159265358979323846 / 2.0 ||
            fabs(wrap_pi(pa)) > 3.14159265358979323846 / 2.0 ||
            mre <= 0.0 || are <= 0.0)
            polarity_ok = 0;
    }
    check("no_polarity_flip", polarity_ok);

    /* unity gains reconstruct the input magnitude at every probe point */
    for (int b = 0; b < EQ_BANDS; b++) mx.eq.gain_db[b] = 0.0f;
    int flat_ok = 1;
    for (int grp = 0; grp < 4 && flat_ok; grp++)
        for (int k = 0; k < 3; k++) {
            double mre, mim;
            eq_measure(&mx.eq, groups[grp][k], &mre, &mim);
            if (fabs(to_db(hypot(mre, mim))) > 0.5) { flat_ok = 0; break; }
        }
    check("unity_sum_flat", flat_ok);

    /* crossover reconfiguration: valid ranges only, state resets cleanly */
    check("xo_valid", eq_set_crossovers(&mx.eq, 500.0f, 2000.0f, 48000.0f));
    check("xo_reject_bad_order",
          !eq_set_crossovers(&mx.eq, 4000.0f, 250.0f, 44100.0f));
    check("xo_reject_above_nyquist",
          !eq_set_crossovers(&mx.eq, 250.0f, 30000.0f, 44100.0f));
    check("xo_reject_bad_sr",
          !eq_set_crossovers(&mx.eq, 250.0f, 4000.0f, 0.0f));
    eq_set_crossovers(&mx.eq, EQ_DEFAULT_FLOW, EQ_DEFAULT_FHIGH,
                      EQ_DEFAULT_SR);
    double mre, mim;
    eq_measure(&mx.eq, 44, &mre, &mim);   /* ~1002 Hz mid-band bin */
    check("xo_reset_stable", fabs(to_db(hypot(mre, mim))) < 0.5);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
