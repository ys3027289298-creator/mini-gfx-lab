
/* animcomp: three-stage animation compression pipeline.
 *
 *   stage 1  decimate : RDP keyframe thinning, max error <= EPS_DEC vs source
 *   stage 2  fit      : per-segment curve fitting — Catmull-Rom where the
 *                       spline stays within EPS_FIT of the stage-1 output,
 *                       linear fallback where it does not — plus greedy key
 *                       removal under the same EPS_FIT constraint
 *   stage 3  cache    : quantized-time direct-mapped eval cache;
 *                       a hit returns the exact bits of a direct eval
 *
 * Stages 1+2 compose: total error <= EPS_DEC + EPS_FIT (triangle inequality).
 * The bound is verified by dense sampling, and size / eval cost are measured
 * against the raw track and printed as a table.
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#define MAX_SAMPLES 4096
#define MAX_KEYS    1024

#define SAMPLE_HZ   240.0f
#define DURATION    2.0f
#define EPS_DEC     0.01f   /* stage-1 threshold */
#define EPS_FIT     0.01f   /* stage-2 threshold */

#define PROBE_STEP  (1.0f / (SAMPLE_HZ * 8.0f))  /* dense error probe grid */

typedef struct { float t, v; } Key;

/* ---- source track: dense uniform samples of an analytic curve ---------- */

static float g_src[MAX_SAMPLES];
static int   g_n;

static float analytic(float t) {
    const float PI2 = 6.28318530718f;
    return sinf(PI2 * 1.5f * t)
         + 0.35f * sinf(PI2 * 4.3f * t + 0.7f)
         + 0.12f * sinf(PI2 * 11.0f * t + 2.1f);
}

static void build_source(void) {
    g_n = (int)(DURATION * SAMPLE_HZ) + 1;
    for (int i = 0; i < g_n; i++)
        g_src[i] = analytic(i / SAMPLE_HZ);
}

/* ground truth: linear interpolation of the dense source samples */
static float source_eval(float t) {
    if (t <= 0.0f) return g_src[0];
    if (t >= DURATION) return g_src[g_n - 1];
    float x = t * SAMPLE_HZ;
    int i = (int)x;
    float u = x - (float)i;
    return g_src[i] + (g_src[i + 1] - g_src[i]) * u;
}

/* ---- stage 1: RDP decimation ------------------------------------------- */

static int decimate(float eps, Key* out, int maxout) {
    unsigned char keep[MAX_SAMPLES] = {0};
    int stack[MAX_SAMPLES], sp = 0;
    keep[0] = keep[g_n - 1] = 1;
    stack[sp++] = 0; stack[sp++] = g_n - 1;
    while (sp > 0) {
        int b = stack[--sp], a = stack[--sp];
        float ta = a / SAMPLE_HZ, tb = b / SAMPLE_HZ;
        float va = g_src[a], vb = g_src[b];
        float maxd = 0.0f; int maxi = -1;
        for (int i = a + 1; i < b; i++) {
            float u = (i / SAMPLE_HZ - ta) / (tb - ta);
            float d = fabsf(g_src[i] - (va + (vb - va) * u));
            if (d > maxd) { maxd = d; maxi = i; }
        }
        if (maxd > eps && maxi > 0) {
            keep[maxi] = 1;
            stack[sp++] = a;    stack[sp++] = maxi;
            stack[sp++] = maxi; stack[sp++] = b;
        }
    }
    int m = 0;
    for (int i = 0; i < g_n && m < maxout; i++)
        if (keep[i]) { out[m].t = i / SAMPLE_HZ; out[m].v = g_src[i]; m++; }
    return m;
}

/* ---- shared evaluators -------------------------------------------------- */

static int find_seg(const Key* k, int n, float t) {
    int lo = 0, hi = n - 1;             /* k[lo].t <= t < k[hi].t */
    while (hi - lo > 1) {
        int mid = (lo + hi) >> 1;
        if (k[mid].t <= t) lo = mid; else hi = mid;
    }
    return lo;
}

static float lin_eval(const Key* k, int n, float t) {
    if (t <= k[0].t) return k[0].v;
    if (t >= k[n - 1].t) return k[n - 1].v;
    int i = find_seg(k, n, t);
    float span = k[i + 1].t - k[i].t;
    float u = span > 0.0f ? (t - k[i].t) / span : 1.0f;
    return k[i].v + (k[i + 1].v - k[i].v) * u;
}

/* one segment of the fitted track: Catmull-Rom or linear */
static float seg_eval(const Key* k, int n, int j, int use_cr, float t) {
    float span = k[j + 1].t - k[j].t;
    float u = span > 0.0f ? (t - k[j].t) / span : 1.0f;
    if (!use_cr)
        return k[j].v + (k[j + 1].v - k[j].v) * u;
    float p0 = k[j > 0 ? j - 1 : 0].v;
    float p1 = k[j].v;
    float p2 = k[j + 1].v;
    float p3 = k[j + 2 < n ? j + 2 : n - 1].v;
    float u2 = u * u, u3 = u2 * u;
    return 0.5f * ((2.0f * p1)
                 + (-p0 + p2) * u
                 + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * u2
                 + (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * u3);
}

static float fit_eval(const Key* k, const unsigned char* mode, int n, float t) {
    if (t <= k[0].t) return k[0].v;
    if (t >= k[n - 1].t) return k[n - 1].v;
    int i = find_seg(k, n, t);
    return seg_eval(k, n, i, mode[i], t);
}

/* ---- stage 2: per-segment curve fit + greedy key removal ---------------- */
/* A CR segment through the same keys is NOT the piecewise-linear curve: at  */
/* sharp corners the spline rounds the corner and can drift past EPS_FIT.   */
/* So each segment independently keeps whichever shape stays within eps of  */
/* the stage-1 output, and a key may be dropped only when every affected    */
/* segment still has a valid shape afterwards.                              */

static int seg_within(const Key* k, int n, int j, int use_cr,
                      const Key* ref, int rn, float eps) {
    float t0 = k[j].t, t1 = k[j + 1].t;
    for (float t = t0; t < t1; t += PROBE_STEP)
        if (fabsf(seg_eval(k, n, j, use_cr, t) - lin_eval(ref, rn, t)) > eps)
            return 0;
    return fabsf(seg_eval(k, n, j, use_cr, t1) - lin_eval(ref, rn, t1)) <= eps;
}

static int fit_curve(const Key* in, int n, float eps,
                     Key* out, unsigned char* mode_out) {
    Key buf[MAX_KEYS];
    memcpy(buf, in, sizeof(Key) * n);
    int m = n, changed = 1;
    while (changed) {
        changed = 0;
        for (int i = 1; i < m - 1; i++) {
            Key tmp[MAX_KEYS];
            int mc = m - 1;
            memcpy(tmp, buf, sizeof(Key) * i);
            memcpy(tmp + i, buf + i + 1, sizeof(Key) * (m - i - 1));
            /* dropping key i re-shapes segments i-2..i+1 (CR support radius) */
            int lo = i - 2 > 0 ? i - 2 : 0;
            int hi = i + 1 < mc - 1 ? i + 1 : mc - 2;
            int ok = 1;
            for (int j = lo; j <= hi && ok; j++)
                ok = seg_within(tmp, mc, j, 1, in, n, eps)
                  || seg_within(tmp, mc, j, 0, in, n, eps);
            if (ok) {
                memmove(buf + i, buf + i + 1, sizeof(Key) * (m - i - 1));
                m--; changed = 1; i--;
            }
        }
    }
    for (int j = 0; j < m - 1; j++)
        mode_out[j] = seg_within(buf, m, j, 1, in, n, eps) ? 1 : 0;
    memcpy(out, buf, sizeof(Key) * m);
    return m;
}

/* ---- stage 3: quantized-time eval cache --------------------------------- */

#define CACHE_BITS 9
#define CACHE_SIZE (1 << CACHE_BITS)
#define CACHE_Q    240.0f     /* time quantum: 1/240 s */

typedef struct { long key; float v; int valid; } Slot;

static Slot g_cache[CACHE_SIZE];
static Key  g_final[MAX_KEYS];
static int  g_final_n;
static unsigned char g_mode[MAX_KEYS];

static float pipeline_eval(float t) { return fit_eval(g_final, g_mode, g_final_n, t); }

static float cached_eval(float t, int* hit) {
    long q = lroundf(t * CACHE_Q);
    Slot* s = &g_cache[(unsigned long)q & (CACHE_SIZE - 1)];
    if (s->valid && s->key == q) { *hit = 1; return s->v; }
    *hit = 0;
    float v = pipeline_eval((float)q / CACHE_Q);
    s->key = q; s->v = v; s->valid = 1;
    return v;
}

/* ---- measurement helpers ------------------------------------------------ */

static float max_error(float (*fn)(float), float step) {
    float e = 0.0f;
    for (float t = 0.0f; t <= DURATION + 1e-6f; t += step) {
        float d = fabsf(fn(t) - source_eval(t));
        if (d > e) e = d;
    }
    return e;
}

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static volatile float g_sink;

/* deterministic LCG so timing runs are reproducible */
static uint64_t g_rng = 0x9E3779B97F4A7C15ull;
static float frand(float lo, float hi) {
    g_rng = g_rng * 6364136223846793005ull + 1442695040888963407ull;
    float u = (float)(g_rng >> 33) / 2147483648.0f;
    return lo + (hi - lo) * u;
}

#define TIMING_N 2000000

static double time_eval(float (*fn)(float)) {
    double t0 = now_s();
    float acc = 0.0f;
    for (int i = 0; i < TIMING_N; i++)
        acc += fn(frand(0.0f, DURATION));
    g_sink = acc;
    return (now_s() - t0) * 1e9 / TIMING_N;
}

static double time_cache_hits(void) {
    memset(g_cache, 0, sizeof(g_cache));
    int hit;
    for (int i = 0; i < 64; i++)            /* warm */
        cached_eval(frand(0.0f, DURATION), &hit);
    double t0 = now_s();
    float acc = 0.0f;
    for (int i = 0; i < TIMING_N; i++) {
        float t = floorf(frand(0.0f, DURATION) * CACHE_Q + 0.5f) / CACHE_Q;
        acc += cached_eval(t, &hit);
    }
    g_sink = acc;
    return (now_s() - t0) * 1e9 / TIMING_N;
}

/* ---- tests + report ----------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static Key g_dec[MAX_KEYS];
static int g_dec_n;

static float eval_stage1(float t) { return lin_eval(g_dec, g_dec_n, t); }
static float eval_final(float t)  { return fit_eval(g_final, g_mode, g_final_n, t); }

int main(void) {
    build_source();

    g_dec_n = decimate(EPS_DEC, g_dec, MAX_KEYS);
    g_final_n = fit_curve(g_dec, g_dec_n, EPS_FIT, g_final, g_mode);

    int cr_segs = 0;
    for (int j = 0; j < g_final_n - 1; j++)
        cr_segs += g_mode[j];

    /* error bounds, measured on a 8x-oversampled grid */
    float err_dec = max_error(eval_stage1, PROBE_STEP);
    float err_all = max_error(eval_final, PROBE_STEP);
    float err_fit = 0.0f;                       /* stage2 vs stage1 */
    for (float t = 0.0f; t <= DURATION + 1e-6f; t += PROBE_STEP) {
        float d = fabsf(eval_final(t) - eval_stage1(t));
        if (d > err_fit) err_fit = d;
    }
    float bound = EPS_DEC + EPS_FIT;

    check("decimate_within_eps", err_dec <= EPS_DEC + 1e-4f);
    check("fit_within_eps", err_fit <= EPS_FIT + 1e-4f);
    check("pipeline_within_sum", err_all <= bound + 1e-3f);
    check("fit_reduces_keys", g_final_n < g_dec_n);

    /* cache: hit must return the exact bits of a direct eval */
    memset(g_cache, 0, sizeof(g_cache));
    int consistent = 1, hits = 0, probes = 20000;
    for (int i = 0; i < probes; i++) {
        float t = frand(0.0f, DURATION);
        long q = lroundf(t * CACHE_Q);
        int h1, h2;
        float v1 = cached_eval(t, &h1);
        float v2 = cached_eval(t, &h2);
        float vd = pipeline_eval((float)q / CACHE_Q);
        if (!h2) consistent = 0;                          /* second call must hit */
        if (memcmp(&v1, &v2, sizeof(float)) != 0) consistent = 0;
        if (memcmp(&v1, &vd, sizeof(float)) != 0) consistent = 0;
        if (h1) hits++;
    }
    check("cache_hit_matches_direct", consistent);

    /* sizes: source = 4 B/sample (uniform t); compressed = 8 B/key (t,v)
     * plus 1 bit/segment for the shape mode */
    int bytes_src = g_n * 4;
    int bytes_dec = g_dec_n * 8;
    int bytes_fit = g_final_n * 8 + (g_final_n - 1 + 7) / 8;

    double ns_src = time_eval(source_eval);
    double ns_dec = time_eval(eval_stage1);
    double ns_fit = time_eval(eval_final);
    double ns_hit = time_cache_hits();

    printf("\nerror report (thresholds: eps_dec=%.4f eps_fit=%.4f)\n", EPS_DEC, EPS_FIT);
    printf("  stage1 max error vs source : %.6f (<= %.4f)\n", err_dec, EPS_DEC);
    printf("  stage2 max error vs stage1 : %.6f (<= %.4f)\n", err_fit, EPS_FIT);
    printf("  full   max error vs source : %.6f (<= %.4f = eps_dec+eps_fit)\n", err_all, bound);
    printf("  fitted segments: %d Catmull-Rom + %d linear\n",
           cr_segs, g_final_n - 1 - cr_segs);
    printf("  cache hit rate over %d random probes: %.1f%%\n",
           probes, 100.0 * hits / probes);

    printf("\n| stage            | keys/samples | bytes | vs raw  | ns/eval | speedup vs raw |\n");
    printf("|------------------|--------------|-------|---------|---------|----------------|\n");
    printf("| raw samples      | %12d | %5d | 100.0%% | %7.2f |           1.00x |\n",
           g_n, bytes_src, ns_src);
    printf("| decimated        | %12d | %5d | %5.1f%% | %7.2f | %13.2fx |\n",
           g_dec_n, bytes_dec, 100.0 * bytes_dec / bytes_src, ns_dec, ns_src / ns_dec);
    printf("| decimated+fitted | %12d | %5d | %5.1f%% | %7.2f | %13.2fx |\n",
           g_final_n, bytes_fit, 100.0 * bytes_fit / bytes_src, ns_fit, ns_src / ns_fit);
    printf("| cache hit        |            - |     - |       - | %7.2f | %13.2fx |\n",
           ns_hit, ns_src / ns_hit);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
