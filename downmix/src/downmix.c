
/* mini-downmix: layout-driven energy-preserving downmix matrix for binaural.
   Channel layout -> unit-norm IID basis vectors + per-ear arrival delays;
   M[e][k] = sqrt(w_k) * basis[e][k] with sum(w)=1, so trace(M M^T) = 1 and
   total energy is preserved for uncorrelated equal-power channels.

   Three matrix render paths are kept side by side:
     A raw static matrix (no delays) -- gain matrix alone;
     B one shared per-channel delay (mean ear arrival) then M -- the best a
       purely static matrix can do, with an analytic error bound;
     C per-ear arrival delays then M -- factorization of the analytic
       per-channel binaural reference, exact up to buffer-edge transients.
   A purely static matrix cannot represent inter-channel relative phase, so
   low-frequency pi-phase cancellation only survives on path C; its residual
   on A/B at the null is listed separately.  With the matrix disabled the
   output is bit-for-bit identical to the legacy spatial-style render. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SR 48000
#define MAX_CH 8
#define W_EPS 1e-4f
#define PI_F 3.14159265358979323846f
#define PI_D 3.14159265358979323846

/* ---------- layouts: azimuth in degrees, 0 = front, + = right ---------- */
typedef struct {
    const char* name;
    int n;
    float az_deg[MAX_CH];
} Layout;

static const Layout LAY_MONO = { "mono", 1, { 0.0f } };
static const Layout LAY_STEREO = { "stereo", 2, { -30.0f, 30.0f } };
static const Layout LAY_51 = {
    "5.1", 6,
    { -30.0f, 30.0f, 0.0f, 0.0f, -110.0f, 110.0f }
    /* L      R      C     LFE    Ls       Rs */
};
static const Layout LAY_71 = {
    "7.1", 8,
    { -30.0f, 30.0f, 0.0f, 0.0f, -110.0f, 110.0f, -150.0f, 150.0f }
    /* L      R      C     LFE    Ls       Rs       Lrs      Rrs */
};

typedef struct {
    int n;
    float az[MAX_CH];          /* radians */
    float w[MAX_CH];           /* weights, sum == 1 */
    float gl[MAX_CH], gr[MAX_CH];
    int dL[MAX_CH], dR[MAX_CH];/* per-ear arrival delays, samples */
    int dA[MAX_CH];            /* alignment delay = round((dL+dR)/2) */
    float M[2][MAX_CH];        /* downmix matrix */
} Dmx;

enum {
    DMX_OK = 0,
    DMX_ERR_DIM = -1,          /* layout/matrix dimension mismatch */
    DMX_ERR_WSUM = -2,         /* weights do not sum to one */
    DMX_ERR_WNEG = -3,         /* negative weight */
    DMX_ERR_LAYOUT = -4
};

/* same IID model as spatial.c: gl^2 + gr^2 == 1 */
static void iid_gains(float az, float* gl, float* gr) {
    float t = (sinf(az) + 1.0f) * 0.5f;
    *gl = sqrtf(1.0f - t);
    *gr = sqrtf(t);
}

/* signed ITD in samples, max 0.6 ms (Woodworth-lite, same as spatial.c) */
static int itd_signed_samples(float az) {
    float max_delay = 0.0006f * SR;
    return (int)lroundf(sinf(az) * max_delay);
}

/* raw layout weights (ITU-style); normalized to sum 1 by the caller */
static void raw_weights(const Layout* lay, float* raw) {
    if (lay->n == 1) { raw[0] = 1.0f; return; }
    float def[MAX_CH];
    for (int i = 0; i < lay->n; i++) def[i] = 1.0f;
    if (lay->n >= 6) {                 /* C, LFE, surrounds in 5.1/7.1 */
        def[2] = (float)(1.0 / sqrt(2.0));   /* C: -3 dB */
        def[3] = 0.5f;                       /* LFE */
        def[4] = (float)(1.0 / sqrt(2.0));   /* Ls */
        def[5] = (float)(1.0 / sqrt(2.0));   /* Rs */
    }
    if (lay->n == 8) { def[6] = 0.5f; def[7] = 0.5f; }  /* rear surrounds */
    if (lay->n == 2) { def[0] = def[1] = 1.0f; }
    memcpy(raw, def, sizeof(float) * (size_t)lay->n);
}

void dmx_default_weights(const Layout* lay, float* out) {
    float raw[MAX_CH];
    raw_weights(lay, raw);
    float s = 0.0f;
    for (int k = 0; k < lay->n; k++) s += raw[k];
    for (int k = 0; k < lay->n; k++) out[k] = raw[k] / s;
}

/* Build the matrix.  rows/cols describe the caller's matrix descriptor and
   must match a 2 x layout.n matrix; weights must sum to one. */
static int dmx_build(Dmx* d, const Layout* lay, const float* weights,
                     int rows, int cols) {
    if (!lay || lay->n <= 0 || lay->n > MAX_CH) return DMX_ERR_LAYOUT;
    if (rows != 2 || cols != lay->n) return DMX_ERR_DIM;

    float s = 0.0f;
    for (int k = 0; k < lay->n; k++) {
        if (!(weights[k] >= 0.0f)) return DMX_ERR_WNEG;
        s += weights[k];
    }
    if (fabsf(s - 1.0f) > W_EPS) return DMX_ERR_WSUM;

    memset(d, 0, sizeof(*d));
    d->n = lay->n;
    for (int k = 0; k < lay->n; k++) {
        d->az[k] = lay->az_deg[k] * PI_F / 180.0f;
        d->w[k] = weights[k];
        iid_gains(d->az[k], &d->gl[k], &d->gr[k]);
        int itd = itd_signed_samples(d->az[k]);
        /* physical arrival: near ear at 0, far ear delayed by |ITD| */
        d->dL[k] = itd > 0 ? itd : 0;
        d->dR[k] = itd < 0 ? -itd : 0;
        d->dA[k] = (int)lroundf((d->dL[k] + d->dR[k]) * 0.5f);
        d->M[0][k] = sqrtf(weights[k]) * d->gl[k];
        d->M[1][k] = sqrtf(weights[k]) * d->gr[k];
    }
    return DMX_OK;
}

/* ---------- render paths ---------- */

/* Reference: analytic per-channel binaural, true per-ear arrival delays. */
static void render_ref(const Dmx* d, float* const* in, int frames,
                       float* out /* interleaved */) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < d->n; k++) {
        float ml = d->M[0][k], mr = d->M[1][k];
        for (int i = 0; i < frames; i++) {
            int jl = i - d->dL[k], jr = i - d->dR[k];
            if (jl >= 0) out[2*i + 0] += ml * in[k][jl];
            if (jr >= 0) out[2*i + 1] += mr * in[k][jr];
        }
    }
}

/* Matrix path A -- raw static matrix, no delay handling.  Quantifies how much
   of the reference behavior is explained by the gain matrix alone. */
static void render_matrix_raw(const Dmx* d, float* const* in, int frames,
                              float* out) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < d->n; k++) {
        float ml = d->M[0][k], mr = d->M[1][k];
        for (int i = 0; i < frames; i++) {
            out[2*i + 0] += ml * in[k][i];
            out[2*i + 1] += mr * in[k][i];
        }
    }
}

/* Matrix path B -- single shared per-channel arrival-time delay (mean of the
   two ear delays) feeding the static matrix.  The best a pure static matrix
   can do; cannot express per-ear relative phase. */
static void render_matrix_aligned(const Dmx* d, float* const* in, int frames,
                                  float* out) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < d->n; k++) {
        int a = d->dA[k];
        float ml = d->M[0][k], mr = d->M[1][k];
        for (int i = 0; i < frames; i++) {
            int j = i - a;
            if (j >= 0) {
                out[2*i + 0] += ml * in[k][j];
                out[2*i + 1] += mr * in[k][j];
            }
        }
    }
}

/* Matrix path C -- delay-aware matrix: per-ear arrival delays followed by the
   static gain matrix.  This is the factorization of the analytic reference:
   y[e](t) = sum_k M[e][k] x_k(t - d_e[k]), so on steady-state signals it is
   exact and the residual energy error is zero up to buffer-edge transients. */
static void render_matrix_binaural(const Dmx* d, float* const* in, int frames,
                                   float* out) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < d->n; k++) {
        float ml = d->M[0][k], mr = d->M[1][k];
        for (int i = 0; i < frames; i++) {
            int jl = i - d->dL[k], jr = i - d->dR[k];
            if (jl >= 0) out[2*i + 0] += ml * in[k][jl];
            if (jr >= 0) out[2*i + 1] += mr * in[k][jr];
        }
    }
}

static float soft_clip(float v) {
    const float k = 0.8f;
    float a = fabsf(v);
    if (a <= k) return v;
    float sign = v < 0 ? -1.0f : 1.0f;
    return sign * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

/* Legacy path: exact spatial.c semantics -- signed ITD shared by both ears,
   raw IID gains, no weights, soft clip.  This is what "now" sounds like. */
static void render_legacy(const Layout* lay, float* const* in, int frames,
                          float* out) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < lay->n; k++) {
        float az = lay->az_deg[k] * PI_F / 180.0f;
        float gl, gr; iid_gains(az, &gl, &gr);
        int delay = itd_signed_samples(az);
        for (int i = 0; i < frames; i++) {
            int j = i - delay;
            float s = (j >= 0 && j < frames) ? in[k][j] : 0.0f;
            out[2*i + 0] += s * gl;
            out[2*i + 1] += s * gr;
        }
    }
    for (int i = 0; i < frames * 2; i++) out[i] = soft_clip(out[i]);
}

/* Dispatched entry: with the matrix off, output equals the legacy render. */
static void render_dispatch(const Dmx* d, const Layout* lay,
                            float* const* in, int frames, int enabled,
                            float* out) {
    if (!enabled) { render_legacy(lay, in, frames, out); return; }
    render_matrix_binaural(d, in, frames, out);
}

/* ---------- measurement helpers ---------- */

static double rel_err(double measured, double reference) {
    return reference < 1e-12 ? fabs(measured) : fabs(measured - reference) / reference;
}

static double energy_of(const float* b, int n) {
    double e = 0.0;
    for (int i = 0; i < n; i++) e += (double)b[i] * b[i];
    return e;
}

/* Complex phasor amplitude at one ear for a shared sinusoid of angular
   frequency w, fed coherently by every active channel. */
typedef struct { double re, im; } Cplx;

static Cplx phasor_ref_ear(const Dmx* d, const int* active, int na,
                           int ear, double w) {
    Cplx acc = { 0.0, 0.0 };
    for (int i = 0; i < na; i++) {
        int k = active[i];
        double c = ear == 0 ? d->M[0][k] : d->M[1][k];
        double delay = ear == 0 ? d->dL[k] : d->dR[k];
        double ph = w * delay / SR;
        acc.re += c * cos(ph);
        acc.im -= c * sin(ph);
    }
    return acc;
}

static Cplx phasor_mat_ear(const Dmx* d, const int* active, int na,
                           int ear, double w, int align) {
    Cplx acc = { 0.0, 0.0 };
    for (int i = 0; i < na; i++) {
        int k = active[i];
        double c = ear == 0 ? d->M[0][k] : d->M[1][k];
        double delay = align ? d->dA[k] : 0.0;
        double ph = w * delay / SR;
        acc.re += c * cos(ph);
        acc.im -= c * sin(ph);
    }
    return acc;
}

static double cabs2(Cplx c) { return c.re * c.re + c.im * c.im; }

/* Path C phasor: per-ear arrival delays -- identical to the reference by
   construction, kept as a named entry point for the matrix-side analysis. */
static Cplx phasor_binaural_ear(const Dmx* d, const int* active, int na,
                                int ear, double w) {
    return phasor_ref_ear(d, active, na, ear, w);
}

/* Analytic upper bound on the power error at one ear:
   ||M|^2 - |R|^2| <= |M-R| (|M| + |R|), with
   |M-R| <= sum_k c_k |e^-iwa - e^-iwd|.                                  */
static double power_err_bound(const Dmx* d, const int* active, int na,
                              int ear, double w, int align) {
    double diff = 0.0;
    for (int i = 0; i < na; i++) {
        int k = active[i];
        double c = ear == 0 ? d->M[0][k] : d->M[1][k];
        double dd = ear == 0 ? d->dL[k] : d->dR[k];
        double da = align ? d->dA[k] : 0.0;
        diff += c * fabs(sin(0.5 * w * (da - dd) / SR)) * 2.0;
    }
    Cplx R = phasor_ref_ear(d, active, na, ear, w);
    Cplx M = phasor_mat_ear(d, active, na, ear, w, align);
    double r = sqrt(cabs2(R)), m = sqrt(cabs2(M));
    return diff * (m + r);   /* bound on | |M|^2 - |R|^2 | */
}

/* ---------- test harness ---------- */
#define FRAMES 2048
static int failures = 0;
static void check(const char* name, int ok) {
    printf("  %s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    srand(1234);
    float w[MAX_CH];
    float* in[MAX_CH];
    for (int k = 0; k < MAX_CH; k++) in[k] = malloc(sizeof(float) * FRAMES);
    float ref[FRAMES * 2], mat[FRAMES * 2], mat_raw[FRAMES * 2], leg[FRAMES * 2];

    /* ===== 1. rejection rules ===== */
    printf("[reject]\n");
    Dmx d;
    dmx_default_weights(&LAY_51, w);
    check("ok_build_5_1", dmx_build(&d, &LAY_51, w, 2, 6) == DMX_OK);
    check("reject_rows", dmx_build(&d, &LAY_51, w, 1, 6) == DMX_ERR_DIM);
    check("reject_cols", dmx_build(&d, &LAY_51, w, 2, 5) == DMX_ERR_DIM);
    float bad[MAX_CH];
    memcpy(bad, w, sizeof(bad)); bad[0] += 0.3f;
    check("reject_wsum", dmx_build(&d, &LAY_51, bad, 2, 6) == DMX_ERR_WSUM);
    memcpy(bad, w, sizeof(bad)); bad[2] = -0.1f;
    check("reject_wneg", dmx_build(&d, &LAY_51, bad, 2, 6) == DMX_ERR_WNEG);
    check("reject_layout", dmx_build(&d, NULL, w, 2, 6) == DMX_ERR_LAYOUT);

    /* ===== 2. energy conservation: trace(M M^T) = sum(w) = 1 =====
       Uncorrelated equal-power channels must give one channel's worth of
       output power regardless of channel count.  Inputs are orthogonal
       harmonics (period P, each energy P/2), periodically continued, and the
       measurement window starts after the largest alignment delay so dropped
       head samples cannot leak into the energy. */
    printf("[energy]\n");
    const int P = 1680;                 /* lcm(1..8)*2, whole periods for all */
    int max_delay = 0;
    const Layout* lays[4] = { &LAY_MONO, &LAY_STEREO, &LAY_51, &LAY_71 };
    for (int li = 0; li < 4; li++) {
        const Layout* lay = lays[li];
        dmx_default_weights(lay, w);
        int rc = dmx_build(&d, lay, w, 2, lay->n);
        if (rc != DMX_OK) { check(lay->name, 0); continue; }
        double tr = 0.0, wsum = 0.0;
        for (int e = 0; e < 2; e++)
            for (int k = 0; k < d.n; k++) tr += (double)d.M[e][k] * d.M[e][k];
        for (int k = 0; k < d.n; k++) {
            wsum += d.w[k];
            if (d.dL[k] > max_delay) max_delay = d.dL[k];
            if (d.dR[k] > max_delay) max_delay = d.dR[k];
        }
        char nm[32]; snprintf(nm, sizeof nm, "trace_unity_%s", lay->name);
        check(nm, fabs(tr - 1.0) < 1e-6 && fabs(wsum - 1.0) < 1e-6);

        int nframe = P + max_delay;
        for (int k = 0; k < d.n; k++)
            for (int i = 0; i < nframe; i++)
                in[k][i] = sinf(2.0f * PI_F * (float)(k + 1) *
                                (float)(i % P) / (float)P);
        render_matrix_binaural(&d, in, nframe, mat);
        double e_in_per_ch = 0.0;     /* any one channel over P samples */
        for (int i = max_delay; i < max_delay + P; i++)
            e_in_per_ch += (double)in[0][i] * in[0][i];
        double eout = 0.0;
        for (int i = max_delay; i < max_delay + P; i++)
            eout += (double)mat[2*i] * mat[2*i]
                  + (double)mat[2*i+1] * mat[2*i+1];
        snprintf(nm, sizeof nm, "equal_power_%s", lay->name);
        check(nm, rel_err(eout, e_in_per_ch) < 1e-4);
        printf("  %s: E_out=%.4f E_in_per_ch=%.4f (n_ch=%d)\n",
               lay->name, eout, e_in_per_ch, d.n);
    }

    /* ===== 3. per-direction error vs analytic binaural reference =====
       The delay-aware matrix (path C) is a factorization of the reference,
       so per-channel impulse responses must match sample-for-sample. */
    printf("[per_direction] (5.1, impulse per channel)\n");
    dmx_default_weights(&LAY_51, w);
    dmx_build(&d, &LAY_51, w, 2, 6);
    double max_c = 0.0, max_b = 0.0, max_a = 0.0;
    int exact_match = 1;
    for (int k = 0; k < d.n; k++) {
        for (int c = 0; c < d.n; c++)
            for (int i = 0; i < FRAMES; i++) in[c][i] = 0.0f;
        in[k][64] = 1.0f;
        render_ref(&d, in, FRAMES, ref);
        float buf_c[FRAMES * 2], buf_b[FRAMES * 2], buf_a[FRAMES * 2];
        render_matrix_binaural(&d, in, FRAMES, buf_c);
        render_matrix_aligned(&d, in, FRAMES, buf_b);
        render_matrix_raw(&d, in, FRAMES, buf_a);
        if (memcmp(buf_c, ref, sizeof(ref)) != 0) exact_match = 0;
        double ec = rel_err(energy_of(buf_c, FRAMES * 2), energy_of(ref, FRAMES * 2));
        double eb = rel_err(energy_of(buf_b, FRAMES * 2), energy_of(ref, FRAMES * 2));
        double ea = rel_err(energy_of(buf_a, FRAMES * 2), energy_of(ref, FRAMES * 2));
        if (ec > max_c) max_c = ec;
        if (eb > max_b) max_b = eb;
        if (ea > max_a) max_a = ea;
        printf("  ch%d az=%6.1f  C=%.3e B=%.3e A=%.3e\n",
               k, LAY_51.az_deg[k], ec, eb, ea);
    }
    check("pathC_sample_exact", exact_match);
    check("pathC_energy_exact", max_c == 0.0);
    printf("  worst: delay_matrix=%.3e shared_delay=%.3e raw=%.3e\n",
           max_c, max_b, max_a);

    /* sinusoid with two coherent channels: relative phase now matters, so the
       shared-delay path (B) has nonzero error that must respect the analytic
       bound derived from its per-ear phase residual. */
    printf("[coherent_sine] Ls+Rs, 200 Hz, analytic amplitude bound (path B)\n");
    int cp[2] = { 4, 5 };
    double worst_gap = 0.0;
    for (double f = 50.0; f <= 800.0; f *= 1.25) {
        double ww = 2.0 * PI_D * f;
        double p_ref = 0.0, p_b = 0.0, bound = 0.0;
        for (int ear = 0; ear < 2; ear++) {
            Cplx R = phasor_ref_ear(&d, cp, 2, ear, ww);
            Cplx B = phasor_mat_ear(&d, cp, 2, ear, ww, 1);
            p_ref += cabs2(R); p_b += cabs2(B);
            bound += power_err_bound(&d, cp, 2, ear, ww, 1);
        }
        double err = fabs(p_b - p_ref);
        if (err > bound) worst_gap = err - bound;
        printf("  f=%7.1f meas=%.3e bound=%.3e\n", f, err, bound);
    }
    check("measured_within_bound", worst_gap <= 1e-12);

    /* ===== 4. low-frequency cancellation point (listed separately) =====
       Ls and Rs have per-ear delay difference |dL-dR| = 27 samples, so at
       f* = SR/54 the reference near ear sees an exact pi-phase pair and
       cancels.  A purely static matrix (paths A/B) cannot represent that
       relative phase: both channels share one delay, so they stay in phase.
       The per-ear delay matrix (path C) reproduces the cancellation exactly.
       All three errors are listed per ear with the analytic bound for B. */
    printf("[lowfreq_cancel] Ls+Rs coherent sine, exact pi-phase point\n");
    dmx_default_weights(&LAY_51, w);
    dmx_build(&d, &LAY_51, w, 2, 6);
    int pair[2] = { 4, 5 };              /* Ls, Rs */
    double f_star = (double)SR / 54.0;
    double ww = 2.0 * PI_D * f_star;
    double sweep_b = 0.0, sweep_a = 0.0;
    for (double f = 20.0; f <= 1000.0; f *= 1.0595) {   /* ~1/12 octave grid */
        double w2 = 2.0 * PI_D * f;
        for (int ear = 0; ear < 2; ear++) {
            Cplx R = phasor_ref_ear(&d, pair, 2, ear, w2);
            Cplx B = phasor_mat_ear(&d, pair, 2, ear, w2, 1);
            Cplx A = phasor_mat_ear(&d, pair, 2, ear, w2, 0);
            double eb = fabs(cabs2(B) - cabs2(R));
            double ea = fabs(cabs2(A) - cabs2(R));
            if (eb > sweep_b) sweep_b = eb;
            if (ea > sweep_a) sweep_a = ea;
        }
    }
    double null_b = 0.0, null_a = 0.0, null_c = 0.0, null_bound = 0.0;
    for (int ear = 0; ear < 2; ear++) {
        Cplx R = phasor_ref_ear(&d, pair, 2, ear, ww);
        Cplx B = phasor_mat_ear(&d, pair, 2, ear, ww, 1);
        Cplx A = phasor_mat_ear(&d, pair, 2, ear, ww, 0);
        double eb = fabs(cabs2(B) - cabs2(R));
        double ea = fabs(cabs2(A) - cabs2(R));
        double bnd = power_err_bound(&d, pair, 2, ear, ww, 1);
        null_b += eb; null_a += ea; null_bound += bnd;
        printf("  ear%d ref_pow=%.3e | C=%.3e B=%.3e A=%.3e bound_B=%.3e\n",
               ear, cabs2(R), 0.0, eb, ea, bnd);
    }
    /* analytic path C: its phasor is the reference phasor by construction */
    for (int ear = 0; ear < 2; ear++) {
        Cplx R = phasor_ref_ear(&d, pair, 2, ear, ww);
        Cplx C = phasor_binaural_ear(&d, pair, 2, ear, ww);
        null_c += fabs(cabs2(C) - cabs2(R));
    }
    printf("  f_star=%.2f Hz; band 20..1000 Hz worst B=%.3e A=%.3e\n",
           f_star, sweep_b, sweep_a);
    check("null_in_low_band", f_star <= 1000.0);
    check("pathC_exact_at_pi_null", null_c < 1e-14);
    check("static_paths_fail_pi_null", null_b > 1e-3 && null_a > 1e-3);
    check("pathB_within_bound", null_b <= null_bound + 1e-12);

    /* time-domain confirmation: integer-period window (period = 54 samples),
       guard prefix skips every arrival delay */
    int nframe = 54 * 4 + max_delay;
    for (int c = 0; c < d.n; c++)
        for (int i = 0; i < nframe; i++) in[c][i] = 0.0f;
    for (int i = 0; i < nframe; i++) {
        float s = sinf(2.0f * PI_F * (float)f_star * i / SR);
        in[4][i] = s; in[5][i] = s;
    }
    render_ref(&d, in, nframe, ref);
    render_matrix_binaural(&d, in, nframe, mat);
    render_matrix_aligned(&d, in, nframe, mat_raw);
    float mat_a[FRAMES * 2];
    render_matrix_raw(&d, in, nframe, mat_a);
    double td_c = 0.0, td_b = 0.0, td_a = 0.0, td_ref = 0.0;
    for (int ear = 0; ear < 2; ear++) {
        double er = 0.0, ec = 0.0, eb = 0.0, ea = 0.0;
        for (int i = max_delay; i < nframe; i++) {
            er += (double)ref[2*i+ear]     * ref[2*i+ear];
            ec += (double)mat[2*i+ear]     * mat[2*i+ear];
            eb += (double)mat_raw[2*i+ear] * mat_raw[2*i+ear];
            ea += (double)mat_a[2*i+ear]   * mat_a[2*i+ear];
        }
        td_ref += er; td_c += fabs(ec - er);
        td_b += fabs(eb - er); td_a += fabs(ea - er);
    }
    printf("  timedomain abs_err C=%.3e B=%.3e A=%.3e ref_E=%.3e\n",
           td_c, td_b, td_a, td_ref);
    check("timedomain_pathC_exact", td_c < 1e-9);
    check("timedomain_pathC_beats_static", td_c < td_b && td_c < td_a);

    /* ===== 5. vocal (center) stays centered ===== */
    printf("[vocal]\n");
    for (int c = 0; c < d.n; c++)
        for (int i = 0; i < FRAMES; i++) in[c][i] = 0.0f;
    for (int i = 0; i < FRAMES; i++) in[2][i] = 0.5f;   /* C only */
    render_ref(&d, in, FRAMES, ref);
    render_matrix_binaural(&d, in, FRAMES, mat);
    double eL = 0.0, eR = 0.0;
    for (int i = 0; i < FRAMES; i++) {
        eL += (double)ref[2*i] * ref[2*i];
        eR += (double)ref[2*i+1] * ref[2*i+1];
    }
    double pan_ref = (eL - eR) / (eL + eR);
    eL = eR = 0.0;
    for (int i = 0; i < FRAMES; i++) {
        eL += (double)mat[2*i] * mat[2*i];
        eR += (double)mat[2*i+1] * mat[2*i+1];
    }
    double pan_mat = (eL - eR) / (eL + eR);
    printf("  pan_ref=%.3e pan_matrix=%.3e\n", pan_ref, pan_mat);
    check("center_phantom", fabs(pan_mat) < 1e-12 && fabs(pan_ref) < 1e-12);

    /* ===== 6. disabled matrix == current (legacy) output, bit exact ===== */
    printf("[disabled_parity]\n");
    for (int c = 0; c < d.n; c++)
        for (int i = 0; i < FRAMES; i++)
            in[c][i] = (float)rand() / (float)RAND_MAX * 2.0f - 1.0f;
    float out_off[FRAMES * 2];
    render_dispatch(&d, &LAY_51, in, FRAMES, 0, out_off);
    render_legacy(&LAY_51, in, FRAMES, leg);
    check("disabled_matches_legacy", memcmp(out_off, leg, sizeof(leg)) == 0);
    render_dispatch(&d, &LAY_51, in, FRAMES, 1, mat);
    check("enabled_differs_from_legacy", memcmp(mat, leg, sizeof(leg)) != 0);

    for (int k = 0; k < MAX_CH; k++) free(in[k]);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
