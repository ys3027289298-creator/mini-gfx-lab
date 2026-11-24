/* bloom: threshold-protected glow with an analytic soft-knee floor.
 *
 * Pipeline (single-channel, gray levels 0..255):
 *   1. bright-pass W(L): none / hard / soft-knee
 *   2. separable Gaussian blur B = K * W(L), kernel normalized to sum 1
 *   3. additive composite: out = clamp(L + gain * B)
 *
 * Soft knee (C0 continuous, C1 at t0):
 *   W(L) = 0                       L <= t0
 *        = (L-t0)^2 / (t1-t0)      t0 < L < t1
 *        = L - t0                  L >= t1
 *
 * Pointwise W(L) <= (L-t0)+ holds, so after blurring with any normalized
 * non-negative kernel k (truncated ones included):
 *   B(x) = sum_j k_j W(L_j) <= sum_j k_j (L_j-t0)+
 * The right side is a neighborhood sum, not a function of M = K*L alone:
 * (M-t0)+ is only a lower bound when M < t0, never an upper one. The exact
 * elevation e(x;t0) = gain*B(x) is a continuous, monotone, piecewise quadratic
 * function of the floor t0 (see solve_floor), inverted by bisection; the
 * uniform-highlight case has the linear closed form e = gain*(A-t0)*p_x.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef struct {
    int w, h;
    float* p;
} Img;

/* ---------------- PPM/PGM I/O (P5 gray, P6 rgb, 8-bit) ---------------- */

static int read_token(FILE* f, char* buf, int n) {
    int c;
    do {
        c = fgetc(f);
        if (c == '#') { while (c != '\n' && c != EOF) c = fgetc(f); }
    } while (c != EOF && (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '#'));
    int i = 0;
    while (c != EOF && c != ' ' && c != '\t' && c != '\n' && c != '\r' && i < n - 1) {
        buf[i++] = (char)c;
        c = fgetc(f);
    }
    buf[i] = '\0';
    return i;
}

static int ppm_load(const char* path, Img* img) {
    FILE* f = fopen(path, "rb");
    if (!f) { perror(path); return -1; }
    char tok[32];
    read_token(f, tok, sizeof tok);
    int rgb;
    if (!strcmp(tok, "P5")) rgb = 0;
    else if (!strcmp(tok, "P6")) rgb = 1;
    else { fprintf(stderr, "%s: unsupported magic %s\n", path, tok); fclose(f); return -1; }
    read_token(f, tok, sizeof tok); img->w = atoi(tok);
    read_token(f, tok, sizeof tok); img->h = atoi(tok);
    read_token(f, tok, sizeof tok);
    if (atoi(tok) != 255) { fprintf(stderr, "%s: only maxval 255 supported\n", path); fclose(f); return -1; }
    int c;
    do { c = fgetc(f); } while (c != EOF && (c == ' ' || c == '\t' || c == '\n' || c == '\r'));
    if (c != EOF) ungetc(c, f); /* first data byte back on the stream */
    size_t n = (size_t)img->w * img->h;
    img->p = malloc(n * sizeof(float));
    if (!img->p) { fclose(f); return -1; }
    unsigned char* raw = malloc(n * (rgb ? 3u : 1u));
    if (!raw) { free(img->p); fclose(f); return -1; }
    if (fread(raw, rgb ? 3u : 1u, n, f) != n) { fprintf(stderr, "%s: short read\n", path); free(raw); free(img->p); fclose(f); return -1; }
    for (size_t i = 0; i < n; i++) {
        if (rgb) {
            float r = raw[3*i], g = raw[3*i+1], b = raw[3*i+2];
            img->p[i] = 0.299f*r + 0.587f*g + 0.114f*b; /* Rec. 601 luma */
        } else {
            img->p[i] = (float)raw[i];
        }
    }
    free(raw);
    fclose(f);
    return 0;
}

static int pgm_save(const char* path, const Img* img) {
    FILE* f = fopen(path, "wb");
    if (!f) { perror(path); return -1; }
    fprintf(f, "P5\n%d %d\n255\n", img->w, img->h);
    size_t n = (size_t)img->w * img->h;
    unsigned char* raw = malloc(n);
    if (!raw) { fclose(f); return -1; }
    for (size_t i = 0; i < n; i++) {
        float v = img->p[i];
        if (v < 0.0f) v = 0.0f; else if (v > 255.0f) v = 255.0f;
        raw[i] = (unsigned char)lrintf(v);
    }
    int ok = fwrite(raw, 1, n, f) == n;
    free(raw);
    fclose(f);
    return ok ? 0 : -1;
}

/* ---------------- bright pass ---------------- */

typedef enum { BP_NONE, BP_HARD, BP_SOFT } BpMode;

static float brightpass(BpMode mode, float t0, float t1, float L) {
    switch (mode) {
    case BP_NONE: return L;
    case BP_HARD: return L > t0 ? L - t0 : 0.0f;
    case BP_SOFT:
        if (L <= t0) return 0.0f;
        if (L >= t1) return L - t0;
        float d = L - t0;
        return d * d / (t1 - t0);
    }
    return 0.0f;
}

/* ---------------- separable Gaussian blur ---------------- */

static float* make_kernel(float sigma, int radius) {
    float* k = malloc((2u * radius + 1u) * sizeof(float));
    if (!k) return NULL;
    float s = 0.0f;
    float two_s2 = 2.0f * sigma * sigma;
    for (int i = -radius; i <= radius; i++) {
        float v = expf(-(float)(i*i) / two_s2);
        k[i + radius] = v;
        s += v;
    }
    for (int i = 0; i <= 2 * radius; i++) k[i] /= s; /* renormalize: truncation keeps sum 1 */
    return k;
}

static void blur(const Img* in, float sigma, int radius, Img* out) {
    int w = in->w, h = in->h;
    out->w = w; out->h = h;
    out->p = malloc((size_t)w * h * sizeof(float));
    float* tmp = malloc((size_t)w * h * sizeof(float));
    float* k = make_kernel(sigma, radius);
    if (!out->p || !tmp || !k) { free(out->p); free(tmp); free(k); exit(2); }
    for (int y = 0; y < h; y++) { /* horizontal */
        for (int x = 0; x < w; x++) {
            float acc = 0.0f;
            for (int i = -radius; i <= radius; i++) {
                int xx = x + i;
                if (xx < 0) xx = 0; else if (xx >= w) xx = w - 1; /* clamp edges */
                acc += k[i + radius] * in->p[y*w + xx];
            }
            tmp[y*w + x] = acc;
        }
    }
    for (int y = 0; y < h; y++) { /* vertical */
        for (int x = 0; x < w; x++) {
            float acc = 0.0f;
            for (int i = -radius; i <= radius; i++) {
                int yy = y + i;
                if (yy < 0) yy = 0; else if (yy >= h) yy = h - 1;
                acc += k[i + radius] * tmp[yy*w + x];
            }
            out->p[y*w + x] = acc;
        }
    }
    free(tmp);
    free(k);
}

/* ---------------- pipeline ---------------- */

static Img run_pipeline(const Img* src, BpMode mode, float t0, float t1,
                        float sigma, int radius, float gain,
                        const char* dump_pre_path, const char* dump_blur_path) {
    size_t n = (size_t)src->w * src->h;
    Img pre = { src->w, src->h, malloc(n * sizeof(float)) };
    for (size_t i = 0; i < n; i++) pre.p[i] = brightpass(mode, t0, t1, src->p[i]);
    if (dump_pre_path) pgm_save(dump_pre_path, &pre);
    Img blr;
    blur(&pre, sigma, radius, &blr);
    if (dump_blur_path) pgm_save(dump_blur_path, &blr);
    Img dst = { src->w, src->h, malloc(n * sizeof(float)) };
    for (size_t i = 0; i < n; i++) {
        float v = src->p[i] + gain * blr.p[i];
        if (v < 0.0f) v = 0.0f; else if (v > 255.0f) v = 255.0f;
        dst.p[i] = v;
    }
    free(pre.p);
    free(blr.p);
    return dst;
}

/* ---------------- floor inversion ----------------
 *
 * Analytic relation between the soft-knee floor t0 (t1 = t0 + delta) and the
 * elevation at a protected sample x. Writing k_j for the (truncated,
 * renormalized) 2-D kernel weights and S for its support:
 *
 *   e(x;t0) = gain * sum_j k_j W_t0(L_j)
 *           = gain * [ sum_{L_j >= t0+delta} k_j (L_j-t0)
 *                    + sum_{t0 < L_j < t0+delta} k_j (L_j-t0)^2 / delta ]
 *
 * Piecewise quadratic in t0, continuous, and monotone non-increasing, so the
 * floor that meets a tolerance eps on a set of dark samples is found by
 * bisection. The uniform-highlight special case collapses to the linear closed
 * form e = gain*(A-t0)*p_x used by the synthetic validation stage.
 */

static float max_elevation(const Img* src, const int* sx, const int* sy, int ns,
                           float t0, float delta, float sigma, int radius, float gain) {
    size_t n = (size_t)src->w * src->h;
    Img pre = { src->w, src->h, malloc(n * sizeof(float)) };
    for (size_t i = 0; i < n; i++) pre.p[i] = brightpass(BP_SOFT, t0, t0 + delta, src->p[i]);
    Img b;
    blur(&pre, sigma, radius, &b);
    float e = 0.0f;
    for (int i = 0; i < ns; i++) {
        float v = gain * b.p[sy[i] * src->w + sx[i]];
        if (v > e) e = v;
    }
    free(pre.p);
    free(b.p);
    return e;
}

static float solve_floor(const Img* src, const int* sx, const int* sy, int ns,
                         float delta, float sigma, int radius, float gain,
                         float eps, int iters) {
    float lo = 0.0f, hi = 255.0f; /* e(lo) >= 0; e(hi) = 0 since W = 0 everywhere */
    for (int i = 0; i < iters; i++) {
        float mid = 0.5f * (lo + hi);
        if (max_elevation(src, sx, sy, ns, mid, delta, sigma, radius, gain) > eps)
            lo = mid;   /* floor too low: darks lift beyond tolerance */
        else
            hi = mid;
    }
    return hi;
}

#ifndef BLOOM_NO_MAIN
/* ---------------- self test ---------------- */

static int selftest(void) {
    int fails = 0;
    /* kernel normalization for a few truncation radii */
    for (float sigma = 5.0f; sigma <= 25.0f; sigma += 10.0f) {
        for (int r = 15; r <= 80; r += 65) {
            float* k = make_kernel(sigma, r);
            float s = 0.0f;
            for (int i = 0; i <= 2*r; i++) s += k[i];
            if (fabsf(s - 1.0f) > 1e-6f) { printf("FAIL kernel sum sigma=%.1f r=%d s=%g\n", sigma, r, s); fails++; }
            free(k);
        }
    }
    /* soft-knee continuity and the W <= (L-t0)+ bound */
    float t0 = 40.0f, t1 = 56.0f;
    for (int li = 0; li <= 255; li++) {
        float L = (float)li;
        float w = brightpass(BP_SOFT, t0, t1, L);
        float hard = brightpass(BP_HARD, t0, 0.0f, L);
        if (w < 0.0f || w > hard + 1e-6f) { printf("FAIL bound L=%d w=%g hard=%g\n", li, w, hard); fails++; break; }
    }
    /* C0 at both knee knots; inside the quadratic limb |dW/dL| <= 2 by construction */
    if (brightpass(BP_SOFT, t0, t1, t0) != 0.0f) { printf("FAIL knee at t0\n"); fails++; }
    if (fabsf(brightpass(BP_SOFT, t0, t1, t1) - (t1 - t0)) > 1e-6f) { printf("FAIL knee at t1\n"); fails++; }
    if (brightpass(BP_SOFT, t0, t1, t0 - 0.01f) != 0.0f ||
        fabsf(brightpass(BP_SOFT, t0, t1, t1 + 0.01f) - (t1 + 0.01f - t0)) > 1e-6f) {
        printf("FAIL knee neighborhood\n"); fails++;
    }
    /* end-to-end guarantee on a random image: floor solved from the analytic
       bisection must keep every protected dark sample at or below eps */
    srand(1234);
    int w = 64, h = 64;
    Img img = { w, h, malloc((size_t)w*h*sizeof(float)) };
    for (int i = 0; i < w*h; i++) img.p[i] = (float)(rand() % 256);
    float sigma = 6.0f, gain = 1.5f, eps = 2.0f; int radius = 18;
    int sx[16], sy[16], ns = 0;
    for (int y = 0; y < h && ns < 16; y++)
        for (int x = 0; x < w && ns < 16; x++)
            if (img.p[y*w+x] < 30.0f && (x + y) % 7 == 0) { sx[ns] = x; sy[ns] = y; ns++; }
    float floor0 = solve_floor(&img, sx, sy, ns, 16.0f, sigma, radius, gain, eps, 40);
    float e_max = max_elevation(&img, sx, sy, ns, floor0, 16.0f, sigma, radius, gain);
    if (e_max > eps + 1e-3f) { printf("FAIL inversion e_max=%g eps=%g t0=%g\n", e_max, eps, floor0); fails++; }
    /* monotonicity: a smaller floor must not lift less */
    float e_lo = max_elevation(&img, sx, sy, ns, floor0 * 0.5f, 16.0f, sigma, radius, gain);
    if (e_lo + 1e-6f < e_max) { printf("FAIL monotonicity %g < %g\n", e_lo, e_max); fails++; }
    free(img.p);
    printf("selftest_fails=%d\n", fails);
    return fails ? 1 : 0;
}

/* ---------------- CLI ---------------- */

static void usage(void) {
    fprintf(stderr,
        "usage:\n"
        "  bloom selftest\n"
        "  bloom process IN.pgm|IN.ppm OUT.pgm -mode none|hard|soft\n"
        "        [-t0 F] [-t1 F] [-sigma F] [-radius N] [-gain F]\n"
        "        [-dumpblur P.pgm] [-dumppre P.pgm]\n"
        "  bloom solve IN.pgm|IN.ppm PTS.txt -eps F [-delta F]\n"
        "        [-sigma F] [-radius N] [-gain F]   (prints inverted floor t0)\n"
        "  PTS.txt: one 'x y' per line, the protected dark samples\n");
}

int main(int argc, char** argv) {
    if (argc >= 2 && !strcmp(argv[1], "selftest")) return selftest();
    if (argc >= 2 && !strcmp(argv[1], "solve")) {
        if (argc < 5) { usage(); return 2; }
        const char* in_path = argv[2];
        const char* pts_path = argv[3];
        float eps = 2.0f, delta = 16.0f, sigma = 20.0f, gain = 1.0f;
        int radius = 60;
        for (int i = 4; i < argc; i++) {
            if (!strcmp(argv[i], "-eps") && i+1 < argc) eps = (float)atof(argv[++i]);
            else if (!strcmp(argv[i], "-delta") && i+1 < argc) delta = (float)atof(argv[++i]);
            else if (!strcmp(argv[i], "-sigma") && i+1 < argc) sigma = (float)atof(argv[++i]);
            else if (!strcmp(argv[i], "-radius") && i+1 < argc) radius = atoi(argv[++i]);
            else if (!strcmp(argv[i], "-gain") && i+1 < argc) gain = (float)atof(argv[++i]);
            else { usage(); return 2; }
        }
        Img src;
        if (ppm_load(in_path, &src)) return 1;
        FILE* pf = fopen(pts_path, "r");
        if (!pf) { perror(pts_path); return 1; }
        int cap = 1024, ns = 0;
        int *sx = malloc(cap * sizeof(int)), *sy = malloc(cap * sizeof(int));
        while (fscanf(pf, "%d %d", &sx[ns], &sy[ns]) == 2) {
            if (++ns == cap) { cap *= 2; sx = realloc(sx, cap * sizeof(int)); sy = realloc(sy, cap * sizeof(int)); }
        }
        fclose(pf);
        float t0 = solve_floor(&src, sx, sy, ns, delta, sigma, radius, gain, eps, 40);
        float e = max_elevation(&src, sx, sy, ns, t0, delta, sigma, radius, gain);
        printf("t0=%.6f eps=%.4f achieved_max_elev=%.6f samples=%d\n", t0, eps, e, ns);
        free(sx); free(sy); free(src.p);
        return 0;
    }
    if (argc < 5 || strcmp(argv[1], "process")) { usage(); return 2; }
    const char* in_path = argv[2];
    const char* out_path = argv[3];
    BpMode mode = BP_SOFT;
    float t0 = 0.0f, t1 = 16.0f, sigma = 20.0f, gain = 1.0f;
    int radius = 60;
    const char* dump_blur = NULL;
    const char* dump_pre = NULL;
    for (int i = 4; i < argc; i++) {
        if (!strcmp(argv[i], "-mode") && i+1 < argc) {
            const char* m = argv[++i];
            if (!strcmp(m, "none")) mode = BP_NONE;
            else if (!strcmp(m, "hard")) mode = BP_HARD;
            else if (!strcmp(m, "soft")) mode = BP_SOFT;
            else { usage(); return 2; }
        } else if (!strcmp(argv[i], "-t0") && i+1 < argc) t0 = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "-t1") && i+1 < argc) t1 = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "-sigma") && i+1 < argc) sigma = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "-radius") && i+1 < argc) radius = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-gain") && i+1 < argc) gain = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "-dumpblur") && i+1 < argc) dump_blur = argv[++i];
        else if (!strcmp(argv[i], "-dumppre") && i+1 < argc) dump_pre = argv[++i];
        else { usage(); return 2; }
    }
    if (mode == BP_SOFT && t1 <= t0) { fprintf(stderr, "soft mode requires t1 > t0\n"); return 2; }
    Img src;
    if (ppm_load(in_path, &src)) return 1;
    Img dst = run_pipeline(&src, mode, t0, t1, sigma, radius, gain, dump_pre, dump_blur);
    if (pgm_save(out_path, &dst)) { fprintf(stderr, "write failed: %s\n", out_path); return 1; }
    free(src.p); free(dst.p);
    return 0;
}
#endif /* BLOOM_NO_MAIN */
