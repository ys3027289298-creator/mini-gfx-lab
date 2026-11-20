/* posterize: perceptually uniform colour quantization for poster/print output.

   Why per-channel posterization bands badly: quantizing R, G and B
   independently makes each hue's band edges land at different places, so the
   "steps" of a gradient jump around and the boundaries look arbitrary.  This
   module quantizes a perceptual lattice instead:

     - The source is converted to CIELAB (D65, 8-bit sRGB input).  `levels`
       lattice points are placed at equal distances in L* (centres at
       (k + 1/2) * 100 / levels) with each pixel's own hue and chroma.  So
       the lattice is equidistant in the perceptually uniform lightness
       axis, and hue is never quantized.

     - Each pixel is assigned the lattice point that is *perceptually*
       nearest, measured with CIE76 in CIELAB.  For neutral colours this is
       plain nearest-L* rounding; for saturated colours it matters, because
       moving L* can push a colour out of sRGB and chroma then has to give
       way.  Whenever that happens the chroma is reduced along the same hue
       (only as far as needed), so hue is preserved; the nearest-point
       search makes sure a pixel never jumps to a much more desaturated
       band just because that band's L* is closer.

     - Ordered dithering blends the two perceptually nearest lattice
       points.  The Bayer threshold is weighted per pixel by human contrast
       sensitivity for that tone and for the kernel's spatial frequency:

           p = f * amplitude * w(L, n, ppd)      (f = d0 / (d0 + d1))

       w() is the Mannos-Sakrison luminance CSF times a luminance-adaptation
       term, normalised to 1.0 at the reference condition (L*=50, 4x4 kernel,
       the configured `viewing_ppd`).  The eye adapts away from luminance
       detail in dark tones, so those get more dither; a kernel whose
       fundamental sits further from the CSF peak gets less.  Thresholding
       happens through this probability, so dither only selects between
       lattice points and never invents a colour outside the lattice.

     - `enabled = 0` is the identity: the very same bytes come out.

   Reference: every pixel is independently mapped to the perceptually
   nearest lattice point by an exhaustive CIE76 search, with no dithering.
   That is the ideal quantization; the dithered output is compared against
   it pixel by pixel, and the maximum and RMS CIE76 differences are
   returned to the caller and printed by the self-test.  The reference's own
   distance to the source is reported as well, so the two error sources --
   lattice spacing and dither -- can be told apart. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_LEVELS 16
#define MAX_KERNEL  8
#define PI_F 3.14159265358979323846f
#define GAMUT_SCAN_STEPS 128

/* ------------------------------------------------------------------ */
/* sRGB <-> CIELAB (D65)                                              */

typedef struct { float L, a, b; } Lab;
typedef struct { float r, g, b; } LinearRGB;

static float srgb_to_linear(float c) {
    c /= 255.0f;
    return c <= 0.04045f ? c / 12.92f : powf((c + 0.055f) / 1.055f, 2.4f);
}

static float linear_to_srgb(float c) {
    if (c <= 0.0f) return 0.0f;
    if (c >= 1.0f) return 255.0f;
    c = c <= 0.0031308f ? 12.92f * c : 1.055f * powf(c, 1.0f / 2.4f) - 0.055f;
    return c * 255.0f;
}

static float lab_f(float t) {
    const float e = 216.0f / 24389.0f;    /* (6/29)^3 */
    const float k = 24389.0f / 27.0f;     /* (29/3)^3 */
    return t > e ? cbrtf(t) : (k * t + 16.0f) / 116.0f;
}

static float lab_finv(float t) {
    const float e = 6.0f / 29.0f;
    float t3 = t * t * t;
    return t3 > e * e * e ? t3 : (116.0f * t - 16.0f) / (24389.0f / 27.0f);
}

static Lab lab_from_linear(LinearRGB c) {
    /* linear sRGB -> XYZ (D65), normalised so that white is Y = 1 */
    float X = (0.4124564f * c.r + 0.3575761f * c.g + 0.1804375f * c.b) / 0.95047f;
    float Y =  0.2126729f * c.r + 0.7151522f * c.g + 0.0721750f * c.b;
    float Z = (0.0193339f * c.r + 0.1191920f * c.g + 0.9503041f * c.b) / 1.08883f;
    float fx = lab_f(X), fy = lab_f(Y), fz = lab_f(Z);
    Lab out;
    out.L = 116.0f * fy - 16.0f;
    out.a = 500.0f * (fx - fy);
    out.b = 200.0f * (fy - fz);
    return out;
}

static LinearRGB linear_from_lab(Lab c) {
    float fy = (c.L + 16.0f) / 116.0f;
    float fx = fy + c.a / 500.0f;
    float fz = fy - c.b / 200.0f;
    float X = 0.95047f * lab_finv(fx);
    float Y = lab_finv(fy);
    float Z = 1.08883f * lab_finv(fz);
    LinearRGB out;
    out.r =  3.2404542f * X - 1.5371385f * Y - 0.4985314f * Z;
    out.g = -0.9692660f * X + 1.8760108f * Y + 0.0415560f * Z;
    out.b =  0.0556434f * X - 0.2040259f * Y + 1.0572252f * Z;
    return out;
}

static Lab rgb_to_lab(int r, int g, int b) {
    LinearRGB c = { srgb_to_linear((float)r), srgb_to_linear((float)g),
                    srgb_to_linear((float)b) };
    return lab_from_linear(c);
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* cheap in-gamut test; a little slack absorbs float noise at the boundary */
static int linear_in_gamut(LinearRGB c) {
    const float e = 1e-4f;
    return c.r >= -e && c.r <= 1.0f + e &&
           c.g >= -e && c.g <= 1.0f + e &&
           c.b >= -e && c.b <= 1.0f + e;
}

/* keep hue, give up chroma only as far as required to fit sRGB.  A pure
   bisection is not enough: for strongly out-of-gamut hues the sRGB solid is
   not star-shaped about the grey axis, so the in-gamut chroma can be
   non-contiguous along a hue line.  Scan outward and keep the last chroma
   that still fits. */
static Lab gamut_map_hue_preserving(Lab c) {
    if (linear_in_gamut(linear_from_lab(c))) return c;
    Lab best = c;
    best.a = 0.0f;
    best.b = 0.0f;
    for (int i = 1; i <= GAMUT_SCAN_STEPS; i++) {
        float t = (float)i / (float)GAMUT_SCAN_STEPS;
        Lab trial = c;
        trial.a *= t;
        trial.b *= t;
        if (!linear_in_gamut(linear_from_lab(trial))) break;
        best = trial;
    }
    return best;
}

static void lab_to_rgb(Lab c, int* r, int* g, int* b) {
    LinearRGB lin = linear_from_lab(c);
    int rr = (int)lroundf(linear_to_srgb(lin.r));
    int gg = (int)lroundf(linear_to_srgb(lin.g));
    int bb = (int)lroundf(linear_to_srgb(lin.b));
    *r = rr < 0 ? 0 : (rr > 255 ? 255 : rr);
    *g = gg < 0 ? 0 : (gg > 255 ? 255 : gg);
    *b = bb < 0 ? 0 : (bb > 255 ? 255 : bb);
}

static float cielab_dist(Lab x, Lab y) {
    float dl = x.L - y.L, da = x.a - y.a, db = x.b - y.b;
    return sqrtf(dl * dl + da * da + db * db);
}

/* ------------------------------------------------------------------ */
/* configuration                                                      */

typedef struct {
    int enabled;        /* 0 = pass through unchanged */
    int levels;         /* 2..MAX_LEVELS, lattice points at equal L* steps */
    int kernel;         /* ordered dither matrix: 2, 4 or 8 */
    float amplitude;    /* dither strength, 0..1 */
    float viewing_ppd;  /* display pixels per degree of visual angle */
} PosterizeConfig;

static int config_valid(const PosterizeConfig* cfg) {
    if (!cfg) return 0;
    if (!cfg->enabled) return 1;
    if (cfg->levels < 2 || cfg->levels > MAX_LEVELS) return 0;
    if (cfg->kernel != 2 && cfg->kernel != 4 && cfg->kernel != 8) return 0;
    if (!(cfg->amplitude >= 0.0f && cfg->amplitude <= 1.0f)) return 0;
    if (!(cfg->viewing_ppd > 0.0f) || !(cfg->viewing_ppd < 1.0e5f)) return 0;
    return 1;
}

/* ------------------------------------------------------------------ */
/* ordered dither kernel                                              */

typedef struct {
    int n;
    float w[MAX_KERNEL][MAX_KERNEL];   /* zero mean, unit RMS */
    float u[MAX_KERNEL][MAX_KERNEL];   /* uniform thresholds in (0,1) */
} DitherKernel;

static int kernel_build(int n, DitherKernel* k) {
    if (n != 2 && n != 4 && n != 8) return -1;
    k->n = n;

    /* 2x2 Bayer seed grown by M2n = [4Mn+0, 4Mn+2; 4Mn+3, 4Mn+1] */
    static const int seed[2][2] = { {0, 2}, {3, 1} };
    int idx[MAX_KERNEL][MAX_KERNEL];
    int m = 2;
    for (int y = 0; y < m; y++)
        for (int x = 0; x < m; x++) idx[y][x] = seed[y][x];
    while (m < n) {
        int next[MAX_KERNEL][MAX_KERNEL] = { {0} };
        for (int y = 0; y < m; y++) {
            for (int x = 0; x < m; x++) {
                int v = 4 * idx[y][x];
                next[y][x] = v + 0;
                next[y][x + m] = v + 2;
                next[y + m][x] = v + 3;
                next[y + m][x + m] = v + 1;
            }
        }
        memcpy(idx, next, sizeof(next));
        m *= 2;
    }

    float mean = 0.0f;
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++) {
            k->u[y][x] = ((float)idx[y][x] + 0.5f) / (float)(n * n);
            k->w[y][x] = k->u[y][x] - 0.5f;
            mean += k->w[y][x];
        }
    mean /= (float)(n * n);
    float rms = 0.0f;
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++) {
            k->w[y][x] -= mean;
            rms += k->w[y][x] * k->w[y][x];
        }
    rms = sqrtf(rms / (float)(n * n));
    if (rms <= 1e-9f) return -1;
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++) k->w[y][x] /= rms;
    return 0;
}

/* ------------------------------------------------------------------ */
/* contrast-sensitivity weighting                                     */

/* Mannos-Sakrison luminance CSF, f in cycles per degree */
static float csf_spatial(float f) {
    if (f < 0.0f) f = 0.0f;
    return 2.6f * (0.0192f + 0.114f * f) * expf(-powf(0.114f * f, 1.1f));
}

/* adaptation: the eye trades luminance-detail sensitivity for range in
   dark tones, so an equal L* step is less visible there */
static float csf_adaptation(float L) {
    float t = clampf((L + 16.0f) / 116.0f, 0.0f, 1.0f);
    float Y = t * t * t;                       /* relative luminance */
    return 0.55f + 0.45f * clampf(Y / 0.18f, 0.0f, 1.0f);
}

/* kernel size -> fundamental spatial frequency of its dither pattern */
static float kernel_frequency(int n, float ppd) {
    return ppd / (2.0f * (float)n);
}

/* normalised so the reference condition (L*=50, 4x4, given ppd) is 1.0.
   The lower bound keeps the weighting from exploding at very fine pitches,
   where the CSF model is no longer meaningful. */
static float dither_weight(float L, int n, float ppd) {
    const float L_ref = 50.0f;
    float sens = csf_spatial(kernel_frequency(n, ppd)) * csf_adaptation(L);
    float ref = csf_spatial(kernel_frequency(4, ppd)) * csf_adaptation(L_ref);
    float w = ref / sens;
    return w < 0.25f ? 0.25f : w;
}

/* ------------------------------------------------------------------ */
/* quantization lattice and nearest-point search                      */

/* lattice point k for a source colour: same hue and chroma, L* moved to the
   k-th equal step; chroma reduced only if that leaves sRGB */
static Lab lattice_colour(Lab src, int k, int levels) {
    float step = 100.0f / (float)levels;
    Lab q = src;
    q.L = ((float)k + 0.5f) * step;
    return gamut_map_hue_preserving(q);
}

/* the two perceptually nearest lattice points; f is the mixing fraction of
   the second one that best sits between them */
typedef struct {
    Lab c0, c1;
    float d0, d1;
    float f;
} NearestPair;

static void nearest_pair(Lab src, int levels, NearestPair* np) {
    np->c0 = lattice_colour(src, 0, levels);
    np->d0 = cielab_dist(src, np->c0);
    np->c1 = np->c0;
    np->d1 = np->d0;
    for (int k = 1; k < levels; k++) {
        Lab c = lattice_colour(src, k, levels);
        float d = cielab_dist(src, c);
        if (d < np->d0) {
            np->c1 = np->c0; np->d1 = np->d0;
            np->c0 = c;      np->d0 = d;
        } else if (d < np->d1) {
            np->c1 = c;      np->d1 = d;
        }
    }
    np->f = (np->d0 + np->d1 > 1e-6f) ? np->d0 / (np->d0 + np->d1) : 0.0f;
}

/* dithered path: pick one of the two nearest lattice points */
static int posterize_rgb(const unsigned char* src, unsigned char* dst,
                         int w, int h, const PosterizeConfig* cfg) {
    if (!src || !dst || w <= 0 || h <= 0) return -1;
    if (!config_valid(cfg)) return -1;

    size_t bytes = (size_t)w * (size_t)h * 3u;
    if (!cfg->enabled) {
        if (src != dst) memcpy(dst, src, bytes);
        return 0;
    }

    DitherKernel k;
    if (kernel_build(cfg->kernel, &k) != 0) return -1;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            size_t p = ((size_t)y * (size_t)w + (size_t)x) * 3u;
            Lab c = rgb_to_lab(src[p], src[p + 1], src[p + 2]);

            NearestPair np;
            nearest_pair(c, cfg->levels, &np);

            /* probability weight of the second-nearest lattice point */
            float g = cfg->amplitude * dither_weight(c.L, cfg->kernel, cfg->viewing_ppd);
            g = clampf(g, 0.0f, 1.0f);
            float prob = clampf(np.f * g, 0.0f, 1.0f);

            Lab out = (k.u[y % k.n][x % k.n] < prob) ? np.c1 : np.c0;

            int r, gg, b;
            lab_to_rgb(out, &r, &gg, &b);
            dst[p] = (unsigned char)r;
            dst[p + 1] = (unsigned char)gg;
            dst[p + 2] = (unsigned char)b;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* per-pixel perceptual reference and error metric                    */

/* every pixel independently takes its perceptually nearest lattice point
   (exhaustive CIE76), with no dithering: the ideal quantization */
static void reference_quantize(const unsigned char* src, unsigned char* dst,
                               int w, int h, int levels) {
    for (size_t i = 0; i < (size_t)w * (size_t)h; i++) {
        size_t p = i * 3u;
        Lab c = rgb_to_lab(src[p], src[p + 1], src[p + 2]);
        NearestPair np;
        nearest_pair(c, levels, &np);
        int r, g, b;
        lab_to_rgb(np.c0, &r, &g, &b);
        dst[p] = (unsigned char)r;
        dst[p + 1] = (unsigned char)g;
        dst[p + 2] = (unsigned char)b;
    }
}

typedef struct {
    float max_de;       /* worst per-pixel CIE76 difference */
    float rms_de;
    int   max_x, max_y;
} DeltaE;

static DeltaE delta_e_image(const unsigned char* a, const unsigned char* b,
                            int w, int h) {
    DeltaE res = { 0.0f, 0.0f, -1, -1 };
    double sum_sq = 0.0;
    size_t n = (size_t)w * (size_t)h;
    for (size_t i = 0; i < n; i++) {
        size_t p = i * 3u;
        Lab ca = rgb_to_lab(a[p], a[p + 1], a[p + 2]);
        Lab cb = rgb_to_lab(b[p], b[p + 1], b[p + 2]);
        float d = cielab_dist(ca, cb);
        sum_sq += (double)d * (double)d;
        if (d > res.max_de) {
            res.max_de = d;
            res.max_x = (int)(i % (size_t)w);
            res.max_y = (int)(i / (size_t)w);
        }
    }
    res.rms_de = (float)sqrt(sum_sq / (double)(n ? n : 1));
    return res;
}

static float delta_e_to_source(const unsigned char* img, const unsigned char* src,
                               int w, int h, float* out_max) {
    double sum_sq = 0.0;
    float worst = 0.0f;
    size_t n = (size_t)w * (size_t)h;
    for (size_t i = 0; i < n; i++) {
        size_t p = i * 3u;
        float d = cielab_dist(rgb_to_lab(img[p], img[p + 1], img[p + 2]),
                              rgb_to_lab(src[p], src[p + 1], src[p + 2]));
        sum_sq += (double)d * (double)d;
        if (d > worst) worst = d;
    }
    if (out_max) *out_max = worst;
    return (float)sqrt(sum_sq / (double)(n ? n : 1));
}

/* ------------------------------------------------------------------ */
/* test harness                                                        */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

#define W 64
#define H 64

static unsigned char src[W * H * 3];
static unsigned char out[W * H * 3];
static unsigned char flat[W * H * 3];
static unsigned char ref[W * H * 3];

/* grey ramp on top, a hue-swept chroma ramp on the bottom: the bottom half
   is what exposes per-channel posterization, since equal RGB steps of
   different hues are unequal perceptual steps. */
static void make_gradient(void) {
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            size_t p = ((size_t)y * W + x) * 3u;
            int g = x * 255 / (W - 1);
            src[p + 0] = (unsigned char)g;
            src[p + 1] = (unsigned char)(y < H / 2 ? g : 255 - g);
            src[p + 2] = (unsigned char)(y < H / 2 ? g : g / 3 + 2 * y / 3);
        }
}

static int same_bytes(const unsigned char* a, const unsigned char* b, size_t n) {
    return memcmp(a, b, n) == 0;
}

/* distinct CIELAB L* values present in the image.  The 0.75 tolerance
   absorbs the 8-bit sRGB round-trip, which is coarsest in the shadows
   (~0.5 * L* per code near black); bands are >= 100/MAX_LEVELS apart. */
static int count_lstar_levels(const unsigned char* img) {
    float vals[MAX_LEVELS + 8];
    int nv = 0;
    for (size_t i = 0; i < (size_t)W * H; i++) {
        size_t p = i * 3u;
        float L = rgb_to_lab(img[p], img[p + 1], img[p + 2]).L;
        int found = 0;
        for (int j = 0; j < nv; j++)
            if (fabsf(vals[j] - L) < 0.75f) { found = 1; break; }
        if (!found) {
            if (nv >= (int)(sizeof(vals) / sizeof(vals[0]))) return -1;
            vals[nv++] = L;
        }
    }
    return nv;
}

/* longest run of one L* value along a row: the visible "band" length */
static int longest_lstar_run(const unsigned char* img, int row) {
    int best = 0, run = 1;
    float prev = rgb_to_lab(img[((size_t)row * W) * 3],
                            img[((size_t)row * W) * 3 + 1],
                            img[((size_t)row * W) * 3 + 2]).L;
    for (int x = 1; x < W; x++) {
        size_t p = ((size_t)row * W + x) * 3u;
        float L = rgb_to_lab(img[p], img[p + 1], img[p + 2]).L;
        if (fabsf(L - prev) < 0.05f) run++;
        else { if (run > best) best = run; run = 1; prev = L; }
    }
    return run > best ? run : best;
}

int main(void) {
    make_gradient();
    size_t bytes = (size_t)W * H * 3u;

    PosterizeConfig cfg = { 1, 6, 4, 1.0f, 60.0f };

    /* --- invalid configs are rejected --- */
    PosterizeConfig bad = cfg; bad.levels = 1;
    check("reject_levels_low", posterize_rgb(src, out, W, H, &bad) == -1);
    bad.levels = MAX_LEVELS + 1;
    check("reject_levels_high", posterize_rgb(src, out, W, H, &bad) == -1);
    bad.levels = 0;
    check("reject_levels_zero", posterize_rgb(src, out, W, H, &bad) == -1);
    bad.levels = -3;
    check("reject_levels_negative", posterize_rgb(src, out, W, H, &bad) == -1);
    bad = cfg; bad.kernel = 3;
    check("reject_kernel_3", posterize_rgb(src, out, W, H, &bad) == -1);
    bad.kernel = 16;
    check("reject_kernel_16", posterize_rgb(src, out, W, H, &bad) == -1);
    bad.kernel = 0;
    check("reject_kernel_0", posterize_rgb(src, out, W, H, &bad) == -1);
    bad = cfg; bad.amplitude = 1.5f;
    check("reject_amplitude", posterize_rgb(src, out, W, H, &bad) == -1);
    bad = cfg; bad.viewing_ppd = 0.0f;
    check("reject_ppd", posterize_rgb(src, out, W, H, &bad) == -1);
    check("reject_null_buffer", posterize_rgb(NULL, out, W, H, &cfg) == -1);
    check("reject_zero_size", posterize_rgb(src, out, 0, H, &cfg) == -1);
    check("reject_null_config", posterize_rgb(src, out, W, H, NULL) == -1);

    /* --- disabled is a bit-exact pass-through, even with garbage fields --- */
    PosterizeConfig off = { 0, 0, 0, 0.0f, 0.0f };
    memset(out, 0xAB, bytes);
    check("disabled_ok", posterize_rgb(src, out, W, H, &off) == 0);
    check("disabled_identity", same_bytes(src, out, bytes));
    check("disabled_ignores_garbage_cfg",
          posterize_rgb(src, out, W, H, &(PosterizeConfig){ 0, -5, 7, 99.0f, -1.0f }) == 0 &&
          same_bytes(src, out, bytes));

    /* --- kernels: zero mean, unit RMS, uniform thresholds, all distinct --- */
    {
        int ok = 1;
        int sizes[3] = { 2, 4, 8 };
        for (int s = 0; s < 3; s++) {
            DitherKernel k;
            if (kernel_build(sizes[s], &k) != 0 || k.n != sizes[s]) { ok = 0; break; }
            float mean = 0.0f, rms = 0.0f;
            for (int y = 0; y < k.n; y++)
                for (int x = 0; x < k.n; x++) {
                    mean += k.w[y][x];
                    rms += k.w[y][x] * k.w[y][x];
                }
            mean /= (float)(k.n * k.n);
            rms = sqrtf(rms / (float)(k.n * k.n));
            if (fabsf(mean) > 1e-5f || fabsf(rms - 1.0f) > 1e-5f) { ok = 0; break; }
            int distinct = 0;
            for (int a = 0; a < k.n * k.n; a++) {
                int dy = a / k.n, dx = a % k.n, dup = 0;
                for (int b = 0; b < a; b++) {
                    int ey = b / k.n, ex = b % k.n;
                    if (fabsf(k.w[dy][dx] - k.w[ey][ex]) < 1e-6f) { dup = 1; break; }
                }
                if (!dup) distinct++;
            }
            if (distinct != k.n * k.n) { ok = 0; break; }
            /* thresholds must tile (0,1): midpoint samples of a uniform ramp,
               so mean 0.5 and mean square ~1/3 (exact up to the midpoint
               sampling error of the 2x2 case) */
            float um = 0.0f, u2 = 0.0f;
            for (int y = 0; y < k.n; y++)
                for (int x = 0; x < k.n; x++) {
                    um += k.u[y][x];
                    u2 += k.u[y][x] * k.u[y][x];
                }
            um /= (float)(k.n * k.n);
            u2 /= (float)(k.n * k.n);
            if (fabsf(um - 0.5f) > 1e-5f || fabsf(u2 - 1.0f / 3.0f) > 1e-2f) { ok = 0; break; }
        }
        check("kernel_normalised", ok);
        DitherKernel k;
        check("kernel_reject_3", kernel_build(3, &k) == -1);
    }

    /* --- CSF: peak near 8 cyc/deg, high frequencies attenuated, shadows
           get more dither, reference condition is exactly 1 --- */
    {
        float peak_f = 0.0f, peak_v = -1.0f;
        for (float f = 0.5f; f <= 60.0f; f += 0.1f) {
            float v = csf_spatial(f);
            if (v > peak_v) { peak_v = v; peak_f = f; }
        }
        check("csf_peak_reasonable", peak_f > 4.0f && peak_f < 12.0f);
        check("csf_falls_off_high", csf_spatial(40.0f) < csf_spatial(8.0f));
        check("csf_dark_less_sensitive", csf_adaptation(5.0f) < csf_adaptation(50.0f));
        check("weight_ref_is_one", fabsf(dither_weight(50.0f, 4, 60.0f) - 1.0f) < 1e-4f);
        check("weight_dark_higher",
              dither_weight(5.0f, 4, 60.0f) > dither_weight(95.0f, 4, 60.0f));
        /* on a retina-class display the coarser patterns (2x2) fall further
           from the CSF peak and get proportionally more dither */
        check("weight_kernel_monotone_at_high_ppd",
              dither_weight(50.0f, 8, 120.0f) < dither_weight(50.0f, 4, 120.0f) &&
              dither_weight(50.0f, 4, 120.0f) < dither_weight(50.0f, 2, 120.0f));
    }

    /* --- equal L* lattice: exactly `levels` distinct L* values, at the
           equal-step centres, independent of hue --- */
    {
        PosterizeConfig nc = cfg; nc.amplitude = 0.0f;
        check("flat_ok", posterize_rgb(src, flat, W, H, &nc) == 0);
        check("band_count_equals_levels", count_lstar_levels(flat) == cfg.levels);
        check("dithered_band_count_equals_levels",
              posterize_rgb(src, out, W, H, &cfg) == 0 &&
              count_lstar_levels(out) == cfg.levels);

        float step = 100.0f / (float)cfg.levels;
        float worst = 0.0f;
        for (int k = 0; k < cfg.levels; k++) {
            float want = ((float)k + 0.5f) * step;
            float best = 1e9f;
            for (size_t i = 0; i < (size_t)W * H; i++) {
                size_t p = i * 3u;
                float L = rgb_to_lab(flat[p], flat[p + 1], flat[p + 2]).L;
                if (fabsf(L - want) < best) best = fabsf(L - want);
            }
            if (best > worst) worst = best;
        }
        check("bands_at_equal_lstar_centres", worst < 0.5f);
        printf("worst_centre_offset=%.4f step=%.4f\n", worst, step);
    }

    /* --- hue is never quantized: flat mode keeps each pixel's hue angle;
           chroma is only lost where sRGB forces it --- */
    {
        float worst_hue = 0.0f, worst_chroma_drop = 0.0f;
        for (size_t i = 0; i < (size_t)W * H; i++) {
            size_t p = i * 3u;
            Lab a = rgb_to_lab(src[p], src[p + 1], src[p + 2]);
            Lab b = rgb_to_lab(flat[p], flat[p + 1], flat[p + 2]);
            float ca = sqrtf(a.a * a.a + a.b * a.b);
            float cb = sqrtf(b.a * b.a + b.b * b.b);
            if (ca > 20.0f) {
                float d = fabsf(atan2f(a.b, a.a) - atan2f(b.b, b.a));
                if (d > PI_F) d = 2.0f * PI_F - d;
                if (d > worst_hue) worst_hue = d;
            }
            if (ca - cb > worst_chroma_drop) worst_chroma_drop = ca - cb;
        }
        check("hue_preserved", worst_hue < 2.0f * PI_F / 180.0f);
        check("chroma_only_gamut_limited", worst_chroma_drop < 20.0f);
        printf("max_hue_shift_deg=%.3f max_chroma_drop=%.3f\n",
               worst_hue * 180.0f / PI_F, worst_chroma_drop);
    }

    /* --- dither breaks up the bands: longest constant-L* run shrinks --- */
    {
        int row = H / 4;                     /* grey ramp */
        int run_flat = longest_lstar_run(flat, row);
        int run_dith = longest_lstar_run(out, row);
        check("dither_breaks_bands", run_dith < run_flat);
        printf("longest_run_flat=%d longest_run_dither=%d\n", run_flat, run_dith);
    }

    /* --- reference: exhaustive per-pixel CIE76 nearest lattice point.
           With dither off the quantizer must match it exactly. --- */
    {
        reference_quantize(src, ref, W, H, cfg.levels);
        check("undithered_equals_reference", same_bytes(flat, ref, bytes));

        float ref_max = 0.0f, flat_max = 0.0f;
        float ref_rms = delta_e_to_source(ref, src, W, H, &ref_max);
        float flat_rms = delta_e_to_source(flat, src, W, H, &flat_max);
        check("quantizer_is_optimal", flat_rms <= ref_rms + 1e-3f && flat_max <= ref_max + 1e-3f);
        /* a lattice with half a step of residual error, plus the chroma
           price paid by saturated colours at their nearest band */
        float step = 100.0f / (float)cfg.levels;
        check("reference_error_bounded", ref_max <= step + 5.0f);
        printf("reference_vs_source: max=%.4f rms=%.4f\n", ref_max, ref_rms);

        /* the reported comparison: dithered output vs per-pixel reference */
        check("dither_ok", posterize_rgb(src, out, W, H, &cfg) == 0);
        check("dither_changes_output", !same_bytes(out, ref, bytes));
        DeltaE de = delta_e_image(out, ref, W, H);
        check("delta_e_rms_le_max", de.rms_de <= de.max_de + 1e-4f);

        /* dither can only switch to the second-nearest band, so the error
           it adds is bounded by the distance between that band pair */
        float pair_bound = 0.0f;
        for (size_t i = 0; i < (size_t)W * H; i++) {
            size_t p = i * 3u;
            Lab c = rgb_to_lab(src[p], src[p + 1], src[p + 2]);
            NearestPair np;
            nearest_pair(c, cfg.levels, &np);
            float d = cielab_dist(np.c0, np.c1);
            if (d > pair_bound) pair_bound = d;
        }
        check("delta_e_within_band_pair", de.max_de <= pair_bound + 1.0f);
        printf("max_delta_e=%.4f rms_delta_e=%.4f at=(%d,%d) band_pair_bound=%.4f\n",
               de.max_de, de.rms_de, de.max_x, de.max_y, pair_bound);
    }

    /* --- levels are configurable: finer lattice reduces the reference
           error, and more distinct bands come out --- */
    {
        float e3_max = 0.0f, e10_max = 0.0f;
        reference_quantize(src, ref, W, H, 3);
        float e3 = delta_e_to_source(ref, src, W, H, &e3_max);
        check("reference_three_bands", count_lstar_levels(ref) == 3);
        reference_quantize(src, ref, W, H, 10);
        float e10 = delta_e_to_source(ref, src, W, H, &e10_max);
        check("reference_ten_bands", count_lstar_levels(ref) == 10);
        check("configurable_levels", e10 < e3);
        printf("reference_rms_levels3=%.4f reference_rms_levels10=%.4f\n", e3, e10);
    }

    /* --- kernel size changes the dither pattern, not the band grid --- */
    {
        unsigned char small[W * H * 3], large[W * H * 3];
        PosterizeConfig c2 = cfg; c2.kernel = 2;
        PosterizeConfig c8 = cfg; c8.kernel = 8;
        check("kernel2_ok", posterize_rgb(src, small, W, H, &c2) == 0);
        check("kernel8_ok", posterize_rgb(src, large, W, H, &c8) == 0);
        check("kernel_size_changes_output", !same_bytes(small, large, bytes));
        check("kernel_size_keeps_bands",
              count_lstar_levels(small) == cfg.levels &&
              count_lstar_levels(large) == cfg.levels);
    }

    /* --- amplitude 0 is exactly the reference; higher amplitude moves
           more pixels off it --- */
    {
        PosterizeConfig a0 = cfg; a0.amplitude = 0.0f;
        PosterizeConfig a1 = cfg; a1.amplitude = 1.0f;
        check("amplitude0_ok", posterize_rgb(src, flat, W, H, &a0) == 0);
        reference_quantize(src, ref, W, H, cfg.levels);
        check("amplitude0_is_reference", same_bytes(flat, ref, bytes));
        posterize_rgb(src, out, W, H, &a1);
        DeltaE de1 = delta_e_image(out, ref, W, H);
        PosterizeConfig ah = cfg; ah.amplitude = 0.5f;
        posterize_rgb(src, out, W, H, &ah);
        DeltaE de_h = delta_e_image(out, ref, W, H);
        check("amplitude_monotone", de1.rms_de >= de_h.rms_de);
    }

    /* --- identity holds for a saturated image too --- */
    {
        for (size_t i = 0; i < bytes; i += 3) {
            src[i] = 255; src[i + 1] = 0; src[i + 2] = 0;
        }
        memset(out, 0, bytes);
        check("disabled_identity_saturated",
              posterize_rgb(src, out, W, H, &off) == 0 && same_bytes(src, out, bytes));
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
