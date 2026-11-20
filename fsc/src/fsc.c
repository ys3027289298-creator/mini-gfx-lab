/* fsc: frequency-selective sidechain compression.
   A plain sidechain ducks the whole signal, so a low-frequency trigger pulls
   the bass down with it. Here target and key are split into the same bands by
   Linkwitz-Riley crossovers; every band runs its own envelope follower on the
   key and is gain-reduced only while its own envelope is over threshold, then
   the bands are summed back. Crossover frequencies are part of the config.

   Three independent implementations of the same algorithm live in this file:
     - Fsc / fsc_process:  production path, sample-major streaming state
     - ref_process_f:      per-band manual processing on full float buffers
     - gt_process_d:       same manual structure in double precision, which
                           also records the trace used for the analytical
                           upper bound on float-rounding error
   The two float paths use independently written code but identical operation
   order, so they must match bit for bit; the double path quantifies how far
   float rounding can move the output, checked against a derived bound. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define FSC_MAX_BANDS 4
#define FSC_MAX_STAGES 4          /* LR4 middle band: HP x2 then LP x2 */
#define FSC_MAX_N 4096

/* ---------- shared band topology (indexing only, no DSP math) ---------- */

typedef struct { int cross; int hp; } StageSpec;   /* hp: 0 = lowpass */

/* Each LR2 stage is listed twice: two cascaded identical LR2 biquads make a
   4th-order Linkwitz-Riley section. Band 0 is low, the last band is high,
   middle bands are high(cross[b-1]) then low(cross[b]). */
static int band_stages(int bands, int b, StageSpec* out) {
    int n = 0;
    if (b == 0) {
        out[n++] = (StageSpec){0, 0};
        out[n++] = (StageSpec){0, 0};
    } else if (b == bands - 1) {
        out[n++] = (StageSpec){b - 1, 1};
        out[n++] = (StageSpec){b - 1, 1};
    } else {
        out[n++] = (StageSpec){b - 1, 1};
        out[n++] = (StageSpec){b - 1, 1};
        out[n++] = (StageSpec){b, 0};
        out[n++] = (StageSpec){b, 0};
    }
    return n;
}

/* =================== production path: streaming float =================== */

typedef struct {
    float b0, b1, b2, a1, a2, z1, z2;
} FscBq;

typedef struct {
    FscBq t[FSC_MAX_STAGES];     /* target (ducked) signal path */
    FscBq k[FSC_MAX_STAGES];     /* key (trigger) signal path   */
    int nst;
    float env;
} FscBand;

typedef struct {
    int bands;
    float fs, thresh, ratio, kexp;
    float atk_s, rel_s, atk, rel;
    float cross[FSC_MAX_BANDS - 1];
    FscBand band[FSC_MAX_BANDS];
} Fsc;

static void lr2_coeffs_f(float f0, float fs, int hp, FscBq* q) {
    const float PI = 3.14159265358979323846f;
    float w0 = 2.0f * PI * f0 / fs;
    float cs = cosf(w0);
    float sn = sinf(w0);
    float alpha = sn;                 /* LR2 = Butterworth: Q = 0.5 */
    float b0, b1, b2;
    if (!hp) {
        b0 = (1.0f - cs) * 0.5f; b1 = 1.0f - cs; b2 = (1.0f - cs) * 0.5f;
    } else {
        b0 = (1.0f + cs) * 0.5f; b1 = -(1.0f + cs); b2 = (1.0f + cs) * 0.5f;
    }
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cs;
    float a2 = 1.0f - alpha;
    q->b0 = b0 / a0; q->b1 = b1 / a0; q->b2 = b2 / a0;
    q->a1 = a1 / a0; q->a2 = a2 / a0;
    q->z1 = q->z2 = 0.0f;
}

/* transposed direct-form II; this exact operation order is mirrored in the
   reference and ground-truth steps so the float paths stay bit identical */
static float bq_step_f(FscBq* q, float x) {
    float y = q->b0 * x + q->z1;
    q->z1 = q->b1 * x - q->a1 * y + q->z2;
    q->z2 = q->b2 * x - q->a2 * y;
    return y;
}

int fsc_init(Fsc* f, int bands, const float* cross, float fs,
             float thresh, float ratio, float atk_s, float rel_s) {
    if (!f || !cross || bands < 2 || bands > FSC_MAX_BANDS) return -1;
    if (!(fs > 0.0f) || !(thresh > 0.0f) || ratio < 1.0f ||
        !(atk_s > 0.0f) || !(rel_s > 0.0f)) return -1;
    for (int i = 0; i < bands - 1; i++) {
        if (!(cross[i] > 0.0f && cross[i] < fs * 0.5f)) return -1;
        if (i > 0 && !(cross[i] > cross[i - 1])) return -1;
    }
    memset(f, 0, sizeof(*f));
    f->bands = bands;
    f->fs = fs;
    f->thresh = thresh;
    f->ratio = ratio;
    f->kexp = 1.0f - 1.0f / ratio;
    f->atk_s = atk_s;
    f->rel_s = rel_s;
    f->atk = 1.0f - expf(-1.0f / (fs * atk_s));
    f->rel = 1.0f - expf(-1.0f / (fs * rel_s));
    memcpy(f->cross, cross, sizeof(float) * (size_t)(bands - 1));
    for (int b = 0; b < bands; b++) {
        StageSpec st[FSC_MAX_STAGES];
        int nst = band_stages(bands, b, st);
        f->band[b].nst = nst;
        for (int s = 0; s < nst; s++) {
            lr2_coeffs_f(cross[st[s].cross], fs, st[s].hp, &f->band[b].t[s]);
            lr2_coeffs_f(cross[st[s].cross], fs, st[s].hp, &f->band[b].k[s]);
        }
    }
    return 0;
}

/* target and key are read-only; only out is written, one sample at a time,
   with no lookahead and no resampling, so the input time base is preserved */
void fsc_process(Fsc* f, const float* target, const float* key,
                 float* out, int n) {
    for (int i = 0; i < n; i++) {
        float acc = 0.0f;
        for (int b = 0; b < f->bands; b++) {
            FscBand* bd = &f->band[b];
            float yt = target[i];
            for (int s = 0; s < bd->nst; s++) yt = bq_step_f(&bd->t[s], yt);
            float yk = key[i];
            for (int s = 0; s < bd->nst; s++) yk = bq_step_f(&bd->k[s], yk);
            float ax = fabsf(yk);
            float c = ax > bd->env ? f->atk : f->rel;
            bd->env = bd->env + c * (ax - bd->env);
            float g = 1.0f;                    /* untouched band = unity */
            if (bd->env > f->thresh)
                g = powf(f->thresh / bd->env, f->kexp);
            float gy = g * yt;
            acc = b == 0 ? gy : acc + gy;
        }
        out[i] = acc;
    }
}

/* ---------- interleaved stereo convenience wrapper (two mono states) ---- */

#define FSC_ST_CHUNK 256

typedef struct {
    Fsc ch[2];
    float tl[FSC_ST_CHUNK], tr[FSC_ST_CHUNK];
    float kl[FSC_ST_CHUNK], kr[FSC_ST_CHUNK];
    float ol[FSC_ST_CHUNK], or_[FSC_ST_CHUNK];
} FscStereo;

static int fsc_stereo_init(FscStereo* s, int bands, const float* cross, float fs,
                           float thresh, float ratio, float atk_s, float rel_s) {
    if (!s) return -1;
    int r = fsc_init(&s->ch[0], bands, cross, fs, thresh, ratio, atk_s, rel_s);
    if (r != 0) return r;
    return fsc_init(&s->ch[1], bands, cross, fs, thresh, ratio, atk_s, rel_s);
}

static void fsc_stereo_process(FscStereo* s, const float* target,
                               const float* key, float* out, int frames) {
    for (int base = 0; base < frames; base += FSC_ST_CHUNK) {
        int m = frames - base;
        if (m > FSC_ST_CHUNK) m = FSC_ST_CHUNK;
        for (int i = 0; i < m; i++) {
            s->tl[i] = target[(base + i) * 2];
            s->tr[i] = target[(base + i) * 2 + 1];
            s->kl[i] = key[(base + i) * 2];
            s->kr[i] = key[(base + i) * 2 + 1];
        }
        fsc_process(&s->ch[0], s->tl, s->kl, s->ol, m);
        fsc_process(&s->ch[1], s->tr, s->kr, s->or_, m);
        for (int i = 0; i < m; i++) {
            out[(base + i) * 2] = s->ol[i];
            out[(base + i) * 2 + 1] = s->or_[i];
        }
    }
}

/* =============== reference: per-band manual processing, float =========== */
/* Independently written: own coefficient and biquad code, band-major loop,
   full materialized band buffers. Arithmetic order matches fsc_process. */

typedef struct {
    float b0, b1, b2, a1, a2, z1, z2;
} RefBq;

static void ref_lr2_f(float f0, float fs, int hp, RefBq* q) {
    float w = 6.28318530717958647692f * f0 / fs;
    float c = cosf(w), s = sinf(w);
    float al = s;
    float num0, num1, num2;
    if (hp) {
        num0 = 0.5f * (1.0f + c); num1 = -1.0f - c; num2 = num0;
    } else {
        num0 = 0.5f * (1.0f - c); num1 = 1.0f - c; num2 = num0;
    }
    float den0 = 1.0f + al, den1 = -2.0f * c, den2 = 1.0f - al;
    q->b0 = num0 / den0; q->b1 = num1 / den0; q->b2 = num2 / den0;
    q->a1 = den1 / den0; q->a2 = den2 / den0;
    q->z1 = 0.0f; q->z2 = 0.0f;
}

static float ref_bq_f(RefBq* q, float x) {
    float y = q->b0 * x + q->z1;
    q->z1 = q->b1 * x - q->a1 * y + q->z2;
    q->z2 = q->b2 * x - q->a2 * y;
    return y;
}

typedef struct {
    int bands;
    const float* cross;
    float fs, thresh, kexp, atk, rel;
} RefCfg;

static void ref_process_f(const RefCfg* c, const float* target,
                          const float* key, float* out, int n) {
    float yb[FSC_MAX_BANDS][FSC_MAX_N];
    float gb[FSC_MAX_BANDS][FSC_MAX_N];
    for (int b = 0; b < c->bands; b++) {
        StageSpec st[FSC_MAX_STAGES];
        int nst = band_stages(c->bands, b, st);
        RefBq tq[FSC_MAX_STAGES], kq[FSC_MAX_STAGES];
        for (int s = 0; s < nst; s++) {
            ref_lr2_f(c->cross[st[s].cross], c->fs, st[s].hp, &tq[s]);
            ref_lr2_f(c->cross[st[s].cross], c->fs, st[s].hp, &kq[s]);
        }
        float env = 0.0f;
        for (int i = 0; i < n; i++) {
            float yt = target[i];
            for (int s = 0; s < nst; s++) yt = ref_bq_f(&tq[s], yt);
            yb[b][i] = yt;
            float yk = key[i];
            for (int s = 0; s < nst; s++) yk = ref_bq_f(&kq[s], yk);
            float mag = fabsf(yk);
            float rate = mag > env ? c->atk : c->rel;
            env = env + rate * (mag - env);
            float g = 1.0f;
            if (env > c->thresh) g = powf(c->thresh / env, c->kexp);
            gb[b][i] = g;
        }
    }
    for (int i = 0; i < n; i++) {
        float sum = gb[0][i] * yb[0][i];
        for (int b = 1; b < c->bands; b++) sum = sum + gb[b][i] * yb[b][i];
        out[i] = sum;
    }
}

/* ============ ground truth: same manual structure, double =============== */

typedef struct {
    double b0, b1, b2, a1, a2, z1, z2;
} GtBq;

typedef struct {
    /* per-sample trace of every intermediate magnitude, used to propagate a
       sample-level worst-case float error instead of a loose steady-state
       bound. Storage is ~1.7 MB; the module is single threaded. */
    double mt[FSC_MAX_BANDS][FSC_MAX_STAGES][FSC_MAX_N]; /* target stage |in| */
    double mk[FSC_MAX_BANDS][FSC_MAX_STAGES][FSC_MAX_N]; /* key stage |in| */
    double yt[FSC_MAX_BANDS][FSC_MAX_N];    /* target band signal */
    double mag[FSC_MAX_BANDS][FSC_MAX_N];   /* |key band signal| */
    double env[FSC_MAX_BANDS][FSC_MAX_N];   /* envelope after update */
    double rate[FSC_MAX_BANDS][FSC_MAX_N];  /* attack/release coeff applied */
    double gain[FSC_MAX_BANDS][FSC_MAX_N];
    double pout;
} Trace;

static void gt_lr2_d(double f0, double fs, int hp, GtBq* q) {
    const double PI = 3.14159265358979323846;
    double w = 2.0 * PI * f0 / fs;
    double c = cos(w), s = sin(w);
    double al = s;                              /* Q = 0.5 */
    double b0, b1, b2;
    if (hp) { b0 = (1 + c) / 2; b1 = -(1 + c); b2 = (1 + c) / 2; }
    else    { b0 = (1 - c) / 2; b1 = 1 - c;    b2 = (1 - c) / 2; }
    double a0 = 1 + al, a1 = -2 * c, a2 = 1 - al;
    q->b0 = b0 / a0; q->b1 = b1 / a0; q->b2 = b2 / a0;
    q->a1 = a1 / a0; q->a2 = a2 / a0;
    q->z1 = q->z2 = 0.0;
}

static double gt_bq_d(GtBq* q, double x) {
    double y = q->b0 * x + q->z1;
    q->z1 = q->b1 * x - q->a1 * y + q->z2;
    q->z2 = q->b2 * x - q->a2 * y;
    return y;
}

typedef struct {
    int bands;
    const float* cross;
    double fs, thresh, ratio, atk_s, rel_s;
} GtCfg;

static void gt_process_d(const GtCfg* c, const float* target,
                         const float* key, double* out, int n, Trace* tr) {
    memset(tr, 0, sizeof(*tr));
    double yb[FSC_MAX_BANDS][FSC_MAX_N];
    double gb[FSC_MAX_BANDS][FSC_MAX_N];
    double atk = 1.0 - exp(-1.0 / (c->fs * c->atk_s));
    double rel = 1.0 - exp(-1.0 / (c->fs * c->rel_s));
    double kexp = 1.0 - 1.0 / c->ratio;
    for (int b = 0; b < c->bands; b++) {
        StageSpec st[FSC_MAX_STAGES];
        int nst = band_stages(c->bands, b, st);
        GtBq tq[FSC_MAX_STAGES], kq[FSC_MAX_STAGES];
        for (int s = 0; s < nst; s++) {
            gt_lr2_d(c->cross[st[s].cross], c->fs, st[s].hp, &tq[s]);
            gt_lr2_d(c->cross[st[s].cross], c->fs, st[s].hp, &kq[s]);
        }
        double env = 0.0;
        for (int i = 0; i < n; i++) {
            double yt = (double)target[i];
            for (int s = 0; s < nst; s++) {
                tr->mt[b][s][i] = fabs(yt);
                yt = gt_bq_d(&tq[s], yt);
            }
            tr->yt[b][i] = yt;
            yb[b][i] = yt;
            double yk = (double)key[i];
            for (int s = 0; s < nst; s++) {
                tr->mk[b][s][i] = fabs(yk);
                yk = gt_bq_d(&kq[s], yk);
            }
            double mag = fabs(yk);
            double rate = mag > env ? atk : rel;
            env = env + rate * (mag - env);
            double g = 1.0;
            if (env > c->thresh) g = pow(c->thresh / env, kexp);
            gb[b][i] = g;
            tr->mag[b][i] = mag;
            tr->env[b][i] = env;
            tr->rate[b][i] = rate;
            tr->gain[b][i] = g;
        }
    }
    for (int i = 0; i < n; i++) {
        double sum = gb[0][i] * yb[0][i];
        for (int b = 1; b < c->bands; b++) sum = sum + gb[b][i] * yb[b][i];
        out[i] = sum;
        if (fabs(sum) > tr->pout) tr->pout = fabs(sum);
    }
}

/* ---- analytical upper bound on |float output - double ground truth| ---- */
/* Sample-level worst-case propagation along the double-precision trace.
   Every DF2-transposed biquad step obeys |e[n]| <= rho|e[n-1]| + 16*u*M[n],
   where the 16u covers 8 roundings and 5 stored coefficients and M[n] is the
   trace magnitude actually presented at that stage. We iterate the recurrence
   instead of taking the steady-state peak bound M/(1-rho), so slow poles no
   longer inflate the bound with a worst case the signal never reaches. The
   envelope one-pole and the threshold gain are propagated the same way using
   the trace's per-sample rate, envelope and gain; a band below threshold keeps
   the literal gain 1.0 and contributes no gain error. */

static float pole_radius_f(float a1, float a2) {
    float disc = a1 * a1 - 4.0f * a2;
    if (disc >= 0.0f) {
        float r1 = fabsf((-a1 + sqrtf(disc)) * 0.5f);
        float r2 = fabsf((-a1 - sqrtf(disc)) * 0.5f);
        return r1 > r2 ? r1 : r2;
    }
    return sqrtf(a2);                    /* complex pair, modulus sqrt(a2) */
}

static double error_bound(const Fsc* f, const Trace* tr, int n) {
    const double u = ldexp(1.0, -24);   /* float unit roundoff, 2^-24 */
    /* per-band output error trajectory; the output is the sum of the bands,
       so a valid bound must sum band errors per sample, not take their max */
    double contrib[FSC_MAX_BANDS][FSC_MAX_N];
    for (int b = 0; b < f->bands; b++) {
        StageSpec st[FSC_MAX_STAGES];
        int nst = band_stages(f->bands, b, st);
        float rho[FSC_MAX_STAGES];
        for (int s = 0; s < nst; s++) {
            FscBq q;
            lr2_coeffs_f(f->cross[st[s].cross], f->fs, st[s].hp, &q);
            rho[s] = pole_radius_f(q.a1, q.a2);
        }
        /* per-stage biquad error trajectories, key and target paths */
        double ek[FSC_MAX_STAGES] = {0}, et[FSC_MAX_STAGES] = {0};
        double e_env = 0.0;
        for (int i = 0; i < n; i++) {
            double pk = 0.0, pt = 0.0;
            for (int s = 0; s < nst; s++) {
                ek[s] = (double)rho[s] * ek[s] + 16.0 * u * tr->mk[b][s][i];
                et[s] = (double)rho[s] * et[s] + 16.0 * u * tr->mt[b][s][i];
                pk += ek[s];
                pt += et[s];
            }

            /* envelope one-pole: env' = env + rate*(mag - env) */
            double c = tr->rate[b][i];
            double r_env = u * (tr->env[b][i] + 2.0 * tr->mag[b][i]
                                + fabs(tr->mag[b][i] - tr->env[b][i]));
            e_env = (1.0 - c) * e_env + c * pk + r_env;

            /* gain branch: literal 1.0 below threshold is exact. Widen the
               branch by e_env so a float/double envelope sitting on opposite
               sides of the threshold is still modelled conservatively. */
            double e_g = 0.0;
            if (tr->env[b][i] + e_env > (double)f->thresh) {
                double lip = (double)f->kexp
                           / fmax(tr->env[b][i], (double)f->thresh);
                e_g = lip * e_env + 3.0 * u;
            }
            /* |delta(g*yt)| <= g*|delta yt| + |yt|*|delta g| + one rounding */
            contrib[b][i] = tr->gain[b][i] * pt
                          + fabs(tr->yt[b][i]) * e_g
                          + u * fabs(tr->gain[b][i] * tr->yt[b][i]);
        }
    }
    double bound = 0.0;
    for (int i = 0; i < n; i++) {
        double total = u * (double)(f->bands - 1) * tr->pout;  /* B-1 adds */
        for (int b = 0; b < f->bands; b++) total += contrib[b][i];
        if (total > bound) bound = total;
    }
    return bound;
}

/* processing variant used by the tests: same arithmetic as fsc_process, but
   also records every band signal and gain trajectory so selectivity can be
   asserted directly instead of reconstructed by subtracting two runs whose
   filter states would differ */
static void fsc_process_traced(Fsc* f, const float* target, const float* key,
                               float* out, int n,
                               float band_sig[FSC_MAX_BANDS][FSC_MAX_N],
                               float gain[FSC_MAX_BANDS][FSC_MAX_N],
                               float envp[FSC_MAX_BANDS][FSC_MAX_N]) {
    for (int i = 0; i < n; i++) {
        float acc = 0.0f;
        for (int b = 0; b < f->bands; b++) {
            FscBand* bd = &f->band[b];
            float yt = target[i];
            for (int s = 0; s < bd->nst; s++) yt = bq_step_f(&bd->t[s], yt);
            float yk = key[i];
            for (int s = 0; s < bd->nst; s++) yk = bq_step_f(&bd->k[s], yk);
            float ax = fabsf(yk);
            float c = ax > bd->env ? f->atk : f->rel;
            bd->env = bd->env + c * (ax - bd->env);
            float g = 1.0f;
            if (bd->env > f->thresh) g = powf(f->thresh / bd->env, f->kexp);
            band_sig[b][i] = yt;
            gain[b][i] = g;
            envp[b][i] = bd->env;
            float gy = g * yt;
            acc = b == 0 ? gy : acc + gy;
        }
        out[i] = acc;
    }
}

/* =============================== tests ================================= */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static float max_abs_diff_f(const float* a, const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = fabsf(a[i] - b[i]);
        if (d > m) m = d;
    }
    return m;
}

static double max_abs_diff_fd(const float* a, const double* b, int n) {
    double m = 0.0;
    for (int i = 0; i < n; i++) {
        double d = fabs((double)a[i] - b[i]);
        if (d > m) m = d;
    }
    return m;
}

static double checksum_abs(const float* a, int n) {
    double s = 0.0;
    for (int i = 0; i < n; i++) s += fabs((double)a[i]);
    return s;
}

static double checksum_weighted(const float* a, int n) {
    double s = 0.0;
    for (int i = 0; i < n; i++) s += (double)a[i] * (double)(i % 7 + 1);
    return s;
}

int main(void) {
    const int N = FSC_MAX_N;
    const float fs = 48000.0f;
    const double PI = 3.14159265358979323846;

    float T[N], K[N], K0[N], out_dut[N], out_ref[N], out_blk[N], out_idle[N];
    double out_gt[N];
    for (int i = 0; i < N; i++) {
        double t = (double)i / (double)fs;
        /* three tones, one per band of the {500, 2000} Hz crossover; each
           sits >= ~2.4x away from a crossover so LR4 transition-band leakage
           stays below 1e-3 */
        T[i] = (float)(0.45 * sin(2 * PI * 120.0 * t)
                     + 0.30 * sin(2 * PI * 1000.0 * t)
                     + 0.20 * sin(2 * PI * 5000.0 * t));
        /* low-band trigger, gated to the middle third of the sequence */
        double gate = (i >= 1024 && i < 3072) ? 1.0 : 0.0;
        K[i] = (float)(gate * 0.5 * sin(2 * PI * 150.0 * t));
        K0[i] = 0.0f;
    }
    double csT_abs = checksum_abs(T, N), csT_w = checksum_weighted(T, N);
    double csK_abs = checksum_abs(K, N), csK_w = checksum_weighted(K, N);

    float cross3[2] = {500.0f, 2000.0f};
    Fsc f;
    int ok_init = fsc_init(&f, 3, cross3, fs, 0.05f, 4.0f, 0.002f, 0.05f) == 0;

    fsc_process(&f, T, K, out_dut, N);

    RefCfg rc = {3, cross3, fs, 0.05f, 1.0f - 1.0f / 4.0f, f.atk, f.rel};
    ref_process_f(&rc, T, K, out_ref, N);
    float d_manual = max_abs_diff_f(out_dut, out_ref, N);
    check("init_ok", ok_init);
    check("manual_matches_streaming", d_manual == 0.0f);

    /* same stream, delivered in arbitrary block sizes, must be identical */
    Fsc fb;
    fsc_init(&fb, 3, cross3, fs, 0.05f, 4.0f, 0.002f, 0.05f);
    fsc_process(&fb, T, K, out_blk, 1024);
    fsc_process(&fb, T + 1024, K + 1024, out_blk + 1024, 1024);
    fsc_process(&fb, T + 2048, K + 2048, out_blk + 2048, N - 2048);
    check("block_split_matches", max_abs_diff_f(out_dut, out_blk, N) == 0.0f);

    /* double ground truth plus analytical error bound */
    GtCfg gc = {3, cross3, (double)fs, 0.05, 4.0, 0.002, 0.05};
    Trace tr;
    gt_process_d(&gc, T, K, out_gt, N, &tr);
    double observed = max_abs_diff_fd(out_dut, out_gt, N);
    double bound = error_bound(&f, &tr, N);
    printf("max_float_double_diff=%.3e\n", observed);
    printf("analytic_bound=%.3e\n", bound);
    printf("bound_headroom=%.1fx\n", bound / observed);
    check("error_within_bound", observed <= bound);

    /* zero key: gains stay exactly 1.0, output still equals manual path */
    Fsc fi;
    fsc_init(&fi, 3, cross3, fs, 0.05f, 4.0f, 0.002f, 0.05f);
    fsc_process(&fi, T, K0, out_idle, N);
    RefCfg rc0 = {3, cross3, fs, 0.05f, 1.0f - 1.0f / 4.0f, fi.atk, fi.rel};
    float ref_idle[N];
    ref_process_f(&rc0, T, K0, ref_idle, N);
    check("idle_matches_manual", max_abs_diff_f(out_idle, ref_idle, N) == 0.0f);

    /* selectivity: low-band trigger ducks only the low band. Asserts on the
       per-band gain trajectory and envelope of a single run, so filter
       transient states are shared rather than differenced away. The window
       starts well after attack and crossover settling. */
    Fsc fsel;
    fsc_init(&fsel, 3, cross3, fs, 0.05f, 4.0f, 0.002f, 0.05f);
    float out_sel[N], bs[FSC_MAX_BANDS][FSC_MAX_N];
    float gg[FSC_MAX_BANDS][FSC_MAX_N], ee[FSC_MAX_BANDS][FSC_MAX_N];
    fsc_process_traced(&fsel, T, K, out_sel, N, bs, gg, ee);
    float gmin_lo[3], gmax_lo[3], emax_lo[3];
    for (int b = 0; b < 3; b++) {
        gmin_lo[b] = 1.0f; gmax_lo[b] = 0.0f; emax_lo[b] = 0.0f;
        for (int i = 1536; i < 2560; i++) {
            if (gg[b][i] < gmin_lo[b]) gmin_lo[b] = gg[b][i];
            if (gg[b][i] > gmax_lo[b]) gmax_lo[b] = gg[b][i];
            if (ee[b][i] > emax_lo[b]) emax_lo[b] = ee[b][i];
        }
    }
    printf("low_key gmin=(%.4f %.4f %.4f) emax=(%.4f %.4f %.4f)\n",
           gmin_lo[0], gmin_lo[1], gmin_lo[2],
           emax_lo[0], emax_lo[1], emax_lo[2]);
    check("low_key_ducks_low_only",
          gmin_lo[0] < 0.5f && gmax_lo[1] == 1.0f && gmax_lo[2] == 1.0f &&
          emax_lo[1] < 0.05f && emax_lo[2] < 0.05f);
    check("traced_run_matches_streaming",
          max_abs_diff_f(out_sel, out_dut, N) == 0.0f);

    /* high-band trigger: same construction, key moved into the high band */
    float Kh[N];
    for (int i = 0; i < N; i++) {
        double t = (double)i / (double)fs;
        double gate = (i >= 1024 && i < 3072) ? 1.0 : 0.0;
        Kh[i] = (float)(gate * 0.5 * sin(2 * PI * 5000.0 * t));
    }
    Fsc fh;
    fsc_init(&fh, 3, cross3, fs, 0.05f, 4.0f, 0.002f, 0.05f);
    float out_h[N], bsh[FSC_MAX_BANDS][FSC_MAX_N];
    float ggh[FSC_MAX_BANDS][FSC_MAX_N], eeh[FSC_MAX_BANDS][FSC_MAX_N];
    fsc_process_traced(&fh, T, Kh, out_h, N, bsh, ggh, eeh);
    float gmin_hi[3], gmax_hi[3], emax_hi[3];
    for (int b = 0; b < 3; b++) {
        gmin_hi[b] = 1.0f; gmax_hi[b] = 0.0f; emax_hi[b] = 0.0f;
        for (int i = 1536; i < 2560; i++) {
            if (ggh[b][i] < gmin_hi[b]) gmin_hi[b] = ggh[b][i];
            if (ggh[b][i] > gmax_hi[b]) gmax_hi[b] = ggh[b][i];
            if (eeh[b][i] > emax_hi[b]) emax_hi[b] = eeh[b][i];
        }
    }
    printf("high_key gmin=(%.4f %.4f %.4f) emax=(%.4f %.4f %.4f)\n",
           gmin_hi[0], gmin_hi[1], gmin_hi[2],
           emax_hi[0], emax_hi[1], emax_hi[2]);
    check("high_key_ducks_high_only",
          gmin_hi[2] < 0.5f && gmax_hi[0] == 1.0f && gmax_hi[1] == 1.0f &&
          emax_hi[0] < 0.05f && emax_hi[1] < 0.05f);

    /* reconstruction identity: with no trigger every gain is exactly 1.0, so
       the crossover band signals must sum back to the output bit for bit */
    Fsc fsum;
    fsc_init(&fsum, 3, cross3, fs, 0.05f, 4.0f, 0.002f, 0.05f);
    float out_sum[N], bss[FSC_MAX_BANDS][FSC_MAX_N];
    float ggs[FSC_MAX_BANDS][FSC_MAX_N], ees[FSC_MAX_BANDS][FSC_MAX_N];
    fsc_process_traced(&fsum, T, K0, out_sum, N, bss, ggs, ees);
    int sum_ok = 1;
    for (int i = 0; i < N; i++) {
        float s = bss[0][i];
        for (int b = 1; b < 3; b++) s += bss[b][i];
        if (s != out_sum[i]) { sum_ok = 0; break; }
    }
    check("bands_sum_to_output", sum_ok);

    /* input sequences must be byte-for-byte unchanged */
    check("inputs_untouched",
          checksum_abs(T, N) == csT_abs && checksum_weighted(T, N) == csT_w &&
          checksum_abs(K, N) == csK_abs && checksum_weighted(K, N) == csK_w);

    /* two-band config: single crossover, same equivalence and bound checks */
    float cross2[1] = {1000.0f};
    Fsc f2;
    fsc_init(&f2, 2, cross2, fs, 0.05f, 4.0f, 0.002f, 0.05f);
    float out2[N], ref2[N];
    double gt2[N];
    fsc_process(&f2, T, K, out2, N);
    RefCfg rc2 = {2, cross2, fs, 0.05f, 1.0f - 1.0f / 4.0f, f2.atk, f2.rel};
    ref_process_f(&rc2, T, K, ref2, N);
    GtCfg gc2 = {2, cross2, (double)fs, 0.05, 4.0, 0.002, 0.05};
    Trace tr2;
    gt_process_d(&gc2, T, K, gt2, N, &tr2);
    double obs2 = max_abs_diff_fd(out2, gt2, N);
    double bnd2 = error_bound(&f2, &tr2, N);
    printf("twoband_diff=%.3e twoband_bound=%.3e\n", obs2, bnd2);
    check("two_band_matches_manual", max_abs_diff_f(out2, ref2, N) == 0.0f);
    check("two_band_within_bound", obs2 <= bnd2);

    /* config validation */
    float good[2] = {500.0f, 2000.0f};
    float bad_order[2] = {2000.0f, 500.0f};
    float bad_range[1] = {30000.0f};
    Fsc fx;
    int rejected =
        fsc_init(&fx, 1, good, fs, 0.05f, 4.0f, 0.002f, 0.05f) == -1 &&
        fsc_init(&fx, FSC_MAX_BANDS + 1, good, fs, 0.05f, 4.0f, 0.002f, 0.05f) == -1 &&
        fsc_init(&fx, 3, bad_order, fs, 0.05f, 4.0f, 0.002f, 0.05f) == -1 &&
        fsc_init(&fx, 2, bad_range, fs, 0.05f, 4.0f, 0.002f, 0.05f) == -1 &&
        fsc_init(&fx, 3, good, fs, 0.05f, 0.5f, 0.002f, 0.05f) == -1 &&
        fsc_init(&fx, 3, good, fs, 0.05f, 4.0f, -1.0f, 0.05f) == -1 &&
        fsc_init(0, 3, good, fs, 0.05f, 4.0f, 0.002f, 0.05f) == -1;
    check("bad_config_rejected", rejected);

    /* stereo wrapper equals two independent mono runs, sample for sample */
    FscStereo st;
    fsc_stereo_init(&st, 3, cross3, fs, 0.05f, 4.0f, 0.002f, 0.05f);
    float Ti[2 * N], Ki[2 * N], Si[2 * N];
    float mono_t[N], mono_k[N], mono_o[N];
    for (int i = 0; i < N; i++) {
        double t = (double)i / (double)fs;
        float tr_sig = (float)(0.4 * sin(2 * PI * 333.0 * t));
        float kr_sig = (float)((i >= 512) ? 0.4 * sin(2 * PI * 250.0 * t) : 0.0);
        Ti[i * 2] = T[i]; Ti[i * 2 + 1] = tr_sig;
        Ki[i * 2] = K[i]; Ki[i * 2 + 1] = kr_sig;
        mono_t[i] = tr_sig; mono_k[i] = kr_sig;
    }
    fsc_stereo_process(&st, Ti, Ki, Si, N);
    Fsc fm;
    fsc_init(&fm, 3, cross3, fs, 0.05f, 4.0f, 0.002f, 0.05f);
    fsc_process(&fm, mono_t, mono_k, mono_o, N);
    int stereo_ok = 1;
    for (int i = 0; i < N; i++)
        if (Si[i * 2 + 1] != mono_o[i]) { stereo_ok = 0; break; }
    check("stereo_matches_mono", stereo_ok);

    /* deterministic stress sweep: several crossover layouts x signal shapes,
       every case must satisfy both bit-identical float equivalence and the
       analytical bound against the double ground truth */
    struct { int bands; float cross[FSC_MAX_BANDS - 1]; } layouts[3] = {
        {2, {800.0f, 0.0f, 0.0f}},
        {3, {300.0f, 3000.0f, 0.0f}},
        {4, {200.0f, 1000.0f, 5000.0f}}
    };
    unsigned long rng = 0x12345678UL;
    int stress_ok = 1;
    double worst_headroom = 1e300;
    for (int lay = 0; lay < 3 && stress_ok; lay++) {
        for (int shape = 0; shape < 4 && stress_ok; shape++) {
            float Ts[N], Ks[N], os[N], rs[N];
            double gs[N];
            for (int i = 0; i < N; i++) {
                double tt = (double)i / (double)fs;
                rng = rng * 1103515245UL + 12345UL;
                float nz = (float)((long)(rng >> 16) % 2001 - 1000) / 1000.0f;
                float imp = (i % 997 == 0) ? 0.9f : 0.0f;
                switch (shape) {
                case 0:  /* broadband noise */
                    Ts[i] = 0.6f * nz;
                    Ks[i] = 0.5f * nz;
                    break;
                case 1:  /* impulses */
                    Ts[i] = imp;
                    Ks[i] = (i % 509 == 0) ? 0.8f : 0.0f;
                    break;
                case 2:  /* gated tones across the spectrum */
                    Ts[i] = (float)(0.4 * sin(2 * PI * 80 * tt)
                                  + 0.3 * sin(2 * PI * 1400 * tt)
                                  + 0.2 * sin(2 * PI * 7000 * tt));
                    Ks[i] = (i >= 700 && i < 3300)
                          ? (float)(0.5 * sin(2 * PI * 120 * tt)) : 0.0f;
                    break;
                default: /* noise + tone mixture */
                    Ts[i] = 0.4f * nz + (float)(0.3 * sin(2 * PI * 420 * tt));
                    Ks[i] = 0.3f * nz + (float)(0.4 * sin(2 * PI * 6000 * tt));
                    break;
                }
            }
            int B = layouts[lay].bands;
            Fsc ff;
            if (fsc_init(&ff, B, layouts[lay].cross, fs, 0.04f, 6.0f,
                         0.001f, 0.03f) != 0) { stress_ok = 0; break; }
            fsc_process(&ff, Ts, Ks, os, N);
            RefCfg rcx = {B, layouts[lay].cross, fs, 0.04f,
                          1.0f - 1.0f / 6.0f, ff.atk, ff.rel};
            ref_process_f(&rcx, Ts, Ks, rs, N);
            GtCfg gcx = {B, layouts[lay].cross, (double)fs,
                         0.04, 6.0, 0.001, 0.03};
            Trace trx;
            gt_process_d(&gcx, Ts, Ks, gs, N, &trx);
            double bnd = error_bound(&ff, &trx, N);
            double obs = max_abs_diff_fd(os, gs, N);
            double hd = bnd / obs;
            if (hd < worst_headroom) worst_headroom = hd;
            if (max_abs_diff_f(os, rs, N) != 0.0f || obs > bnd) stress_ok = 0;
        }
    }
    printf("stress_worst_headroom=%.1fx\n", worst_headroom);
    check("stress_all_bounded", stress_ok);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
