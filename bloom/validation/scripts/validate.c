/* validate.c — two-stage validation of the soft-knee floor inversion.
 *
 * Stage A (synthetic): 512x512 pure-black image + one uniform 240-gray block.
 * Stage B (real photo): PPM with large dark areas, 10 hand-picked dark samples.
 *
 * Per stage we compare three numbers at each of 10 dark samples:
 *   e_analytic : independent 2-D neighborhood sum of kernel weights * W(L)
 *                (separate code path from the separable pipeline blur)
 *   e_measured : elevation produced by the actual bloom pipeline
 *   e_ideal    : what an untruncated continuous Gaussian kernel would predict
 * Truncation is isolated by solving the floor once with the real truncated
 * kernel (t0_trunc) and once with the ideal kernel (t0_ideal), then running
 * BOTH through the real pipeline: only the truncated-kernel floor can meet
 * the tolerance; the ideal-kernel floor's error is purely the missing tail.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "bloom.c"  /* BLOOM_NO_MAIN defined on the compile line */

#define SIGMA 20.0f
#define RADIUS 60            /* 3 sigma: deliberate tail truncation */
#define GAIN 1.0f
#define EPS 2.0f             /* tolerable dark lift, gray levels 0..255 */
#define DELTA 16.0f          /* soft knee width t1 - t0 */
#define SZ 512
#define NS 10

typedef struct { int x, y; } Pt;

static float luma_at(const Img* a, int x, int y) { return a->p[y * a->w + x]; }

static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* untruncated continuous 2-D Gaussian weight (normalizes to 1 over the plane) */
static float ideal_w(int dx, int dy) {
    float s2 = SIGMA * SIGMA;
    return expf(-(float)(dx*dx + dy*dy) / (2.0f * s2)) / (2.0f * (float)M_PI * s2);
}

/* truncated, per-axis renormalized 1-D kernel, matching blur() exactly */
static float* trunc_kernel(void) { return make_kernel(SIGMA, RADIUS); }

/* independent analytic elevation at one sample; ideal=1 uses untruncated weights */
static float analytic_e(const Img* src, Pt p, float t0, int ideal) {
    float* k = ideal ? NULL : trunc_kernel();
    float acc = 0.0f;
    for (int dy = -RADIUS; dy <= RADIUS; dy++) {
        int yy = clampi(p.y + dy, 0, src->h - 1);
        for (int dx = -RADIUS; dx <= RADIUS; dx++) {
            int xx = clampi(p.x + dx, 0, src->w - 1);
            float w = ideal ? ideal_w(dx, dy) : k[dx + RADIUS] * k[dy + RADIUS];
            acc += w * brightpass(BP_SOFT, t0, t0 + DELTA, luma_at(src, xx, yy));
        }
    }
    free(k);
    return GAIN * acc;
}

static float analytic_max(const Img* src, const Pt* pts, float t0, int ideal) {
    float e = 0.0f;
    for (int i = 0; i < NS; i++) {
        float v = analytic_e(src, pts[i], t0, ideal);
        if (v > e) e = v;
    }
    return e;
}

/* floor inversion against the ideal (untruncated) kernel */
static float solve_floor_ideal(const Img* src, const Pt* pts) {
    float lo = 0.0f, hi = 255.0f;
    for (int it = 0; it < 40; it++) {
        float mid = 0.5f * (lo + hi);
        if (analytic_max(src, pts, mid, 1) > EPS) lo = mid; else hi = mid;
    }
    return hi;
}

/* probability mass of the ideal Gaussian lying outside the truncation window */
static float tail_mass(void) {
    float kept = 0.0f;
    for (int dy = -RADIUS; dy <= RADIUS; dy++)
        for (int dx = -RADIUS; dx <= RADIUS; dx++)
            kept += ideal_w(dx, dy);
    return 1.0f - kept;
}

/* fraction of the truncated kernel weight landing on pixels brighter than t */
static float px_trunc(const Img* src, Pt p, float t) {
    float* k = trunc_kernel();
    float s = 0.0f;
    for (int dy = -RADIUS; dy <= RADIUS; dy++) {
        int yy = clampi(p.y + dy, 0, src->h - 1);
        for (int dx = -RADIUS; dx <= RADIUS; dx++) {
            int xx = clampi(p.x + dx, 0, src->w - 1);
            if (luma_at(src, xx, yy) > t) s += k[dx + RADIUS] * k[dy + RADIUS];
        }
    }
    free(k);
    return s;
}

static float px_ideal(const Img* src, Pt p, float t) {
    float s = 0.0f;
    for (int dy = -RADIUS; dy <= RADIUS; dy++) {
        int yy = clampi(p.y + dy, 0, src->h - 1);
        for (int dx = -RADIUS; dx <= RADIUS; dx++) {
            int xx = clampi(p.x + dx, 0, src->w - 1);
            if (luma_at(src, xx, yy) > t) s += ideal_w(dx, dy);
        }
    }
    return s;
}

static Img make_synthetic(void) {
    Img img = { SZ, SZ, calloc((size_t)SZ * SZ, sizeof(float)) };
    for (int y = 200; y < 312; y++)
        for (int x = 200; x < 312; x++)
            img.p[y * SZ + x] = 240.0f;   /* single uniform highlight, rest pure black */
    return img;
}

static void run_stage(const char* name, const Img* src, const Pt* pts,
                      const char* out_none, const char* out_hard, const char* out_soft) {
    printf("\n================ STAGE: %s ================\n", name);
    printf("params: sigma=%.1f radius=%d (%.1fsigma truncation) gain=%.2f eps=%.2f delta=%.1f\n",
           SIGMA, RADIUS, RADIUS / SIGMA, GAIN, EPS, DELTA);
    printf("ideal-Gaussian tail mass outside window: %.6f (%.3f%%)\n", tail_mass(), 100.0 * tail_mass());

    int sx[NS], sy[NS];
    for (int i = 0; i < NS; i++) { sx[i] = pts[i].x; sy[i] = pts[i].y; }

    /* floor from the real truncated kernel (the production inversion) */
    float t0_trunc = solve_floor(src, sx, sy, NS, DELTA, SIGMA, RADIUS, GAIN, EPS, 40);
    /* floor an untruncated-kernel model would have chosen */
    float t0_ideal = solve_floor_ideal(src, pts);
    printf("floor t0 (truncated kernel, bisection) = %.4f\n", t0_trunc);
    printf("floor t0 (ideal untruncated kernel)    = %.4f   delta=%+.4f\n",
           t0_ideal, t0_ideal - t0_trunc);

    /* baseline: the original bug — blur result added with no threshold at all */
    Img buggy = run_pipeline(src, BP_NONE, 0.0f, DELTA, SIGMA, RADIUS, GAIN, NULL, NULL);
    Img hard  = run_pipeline(src, BP_HARD, t0_trunc, t0_trunc + DELTA, SIGMA, RADIUS, GAIN, NULL, NULL);
    Img soft  = run_pipeline(src, BP_SOFT, t0_trunc, t0_trunc + DELTA, SIGMA, RADIUS, GAIN, NULL, NULL);
    pgm_save(out_none, &buggy);
    pgm_save(out_hard, &hard);
    pgm_save(out_soft, &soft);

    printf("\n%-3s %5s %5s %7s | %8s %8s %8s | %6s %6s %5s | %8s %8s\n",
           "idx", "x", "y", "L", "p_trunc", "p_ideal", "e_anltc", "g0", "g1", "dg", "e_meas", "e_none");
    float max_an = 0, max_me = 0, max_none = 0, max_hard = 0;
    for (int i = 0; i < NS; i++) {
        Pt p = pts[i];
        float L = luma_at(src, p.x, p.y);
        float pt = px_trunc(src, p, t0_trunc);
        float pi = px_ideal(src, p, t0_trunc);
        float ea = analytic_e(src, p, t0_trunc, 0);
        float em = soft.p[p.y * SZ + p.x] - L;                       /* pipeline elevation */
        float en = buggy.p[p.y * SZ + p.x] - L;                      /* no-threshold repro */
        float eh = hard.p[p.y * SZ + p.x] - L;
        int g0 = (int)lrintf(L), g1 = (int)lrintf(soft.p[p.y * SZ + p.x]);
        if (ea > max_an) max_an = ea;
        if (em > max_me) max_me = em;
        if (en > max_none) max_none = en;
        if (eh > max_hard) max_hard = eh;
        printf("%-3d %5d %5d %7.2f | %8.5f %8.5f %8.4f | %6d %6d %5d | %8.4f %8.4f\n",
               i, p.x, p.y, L, pt, pi, ea, g0, g1, g1 - g0, em, en);
    }
    printf("\nsummary: max analytic=%.4f  max measured=%.4f  eps=%.2f  "
           "|analytic-measured|_max check below\n", max_an, max_me, EPS);
    float mismatch = 0.0f;
    for (int i = 0; i < NS; i++) {
        float ea = analytic_e(src, pts[i], t0_trunc, 0);
        float em = soft.p[pts[i].y * SZ + pts[i].x] - luma_at(src, pts[i].x, pts[i].y);
        if (fabsf(ea - em) > mismatch) mismatch = fabsf(ea - em);
    }
    printf("max |analytic - measured| over 10 points = %.6f gray levels\n", mismatch);
    printf("no-threshold baseline max dark lift = %.4f (the reported bug)\n", max_none);
    printf("hard threshold @same floor max lift  = %.4f\n", max_hard);

    /* truncation diagnostic: run the ideal-kernel floor through the REAL pipeline */
    float e_ideal_floor_on_real = 0.0f;
    Img soft_id = run_pipeline(src, BP_SOFT, t0_ideal, t0_ideal + DELTA, SIGMA, RADIUS, GAIN, NULL, NULL);
    for (int i = 0; i < NS; i++) {
        float v = soft_id.p[pts[i].y * SZ + pts[i].x] - luma_at(src, pts[i].x, pts[i].y);
        if (v > e_ideal_floor_on_real) e_ideal_floor_on_real = v;
    }
    printf("ideal-kernel floor %.3f run through truncated pipeline -> max lift %.4f vs eps %.2f "
           "(excess %.4f attributable to tail truncation)\n",
           t0_ideal, e_ideal_floor_on_real, EPS, e_ideal_floor_on_real - EPS);
    printf("PASS_inversion=%d (measured max %.4f <= eps %.2f)\n", max_me <= EPS + 1e-3f, max_me, EPS);

    /* stage A closed-form cross-check of the bisection.
       Binary image: block pixels are all L=A, dark pixels 0, so only the
       fraction p* of kernel weight on the block matters:
         e(t0) = gain * p* * W_A(t0)
       Linear limb (A >= t0+delta):  t0 = A - eps/(gain*p*)
       Quadratic limb (t0 < A < t0+delta): t0 = A - sqrt(delta*eps/(gain*p*)) */
    if (!strcmp(name, "synthetic")) {
        float A = 240.0f;
        Pt nearest = pts[3]; /* 2 px outside the uniform block: binding sample */
        float p = px_trunc(src, nearest, t0_trunc);
        float t0_linear = A - EPS / (GAIN * p);
        float t0_quad = A - sqrtf(DELTA * EPS / (GAIN * p));
        const char* branch = (t0_trunc > A - DELTA) ? "quadratic (t0<A<t1)" : "linear (L>=t1)";
        float t0_closed = (t0_trunc > A - DELTA) ? t0_quad : t0_linear;
        printf("closed form (uniform block): p*=%.5f at binding sample (%d,%d)\n",
               p, nearest.x, nearest.y);
        printf("  linear limb    t0 = A - eps/(gain*p*)        = %.4f\n", t0_linear);
        printf("  quadratic limb t0 = A - sqrt(delta*eps/gain/p) = %.4f\n", t0_quad);
        printf("  bisection=%.4f branch=%s |closed-bisect|=%.6f\n",
               t0_trunc, branch, fabsf(t0_closed - t0_trunc));
    }
    free(buggy.p); free(hard.p); free(soft.p); free(soft_id.p);
}

int main(void) {
    Pt synth[NS] = {
        {10, 10}, {100, 256}, {180, 256}, {198, 256}, {256, 100},
        {400, 256}, {256, 450}, {500, 500}, {60, 60}, {350, 100}
    };
    Pt photo[NS] = {
        {24, 42}, {30, 82}, {276, 94}, {414, 126}, {162, 156},
        {254, 182}, {412, 240}, {480, 276}, {318, 406}, {126, 510}
    };
    Img s = make_synthetic();
    pgm_save("out/synthetic_input.pgm", &s);
    run_stage("synthetic", &s, synth,
              "out/synthetic_none.pgm", "out/synthetic_hard.pgm", "out/synthetic_soft.pgm");
    free(s.p);

    Img ph;
    if (ppm_load("input/astronaut.ppm", &ph)) return 1;
    pgm_save("out/photo_input.pgm", &ph);
    run_stage("real photo (astronaut, luma)", &ph, photo,
              "out/photo_none.pgm", "out/photo_hard.pgm", "out/photo_soft.pgm");
    free(ph.p);
    return 0;
}
