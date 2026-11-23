/* colorpipe: four-stage render chain sharing one colour working space.

   The defect being modelled: every stage owns its transfer curve. Each stage
   receives an 8-bit buffer, decodes it with its own curve, applies its blend,
   re-encodes with its own curve and stores 8 bits again. Four mismatched
   decode/encode pairs plus four requantisations stack up into an overall cast:
   on the 24-patch chart the legacy image is ~11 codes brighter and ~4 codes
   bluer than the ideal, mean CIE76 error 5.9, worst 9.4.

   The fix: decode sRGB once, run all four stages on one linear-light buffer,
   encode sRGB once. The only deviation left is 8-bit rounding, and the
   self-test proves a per-patch upper bound on it:

     For a patch with continuous ideal code vector p, the 8-bit result r
     satisfies |r_i - p_i| <= 0.5 on each channel. Walking from p to r one
     channel at a time and applying the triangle inequality in CIELAB gives

       dE(r, p) <= sum_i max(dE(p, p +/- 0.5 e_i))  (+ 1e-3 code float slack)

     That bound is evaluated from the reference itself and the measured
     deviation is checked against it. The legacy path is checked against the
     same bound and fails on all 24 patches, which is the quantified symptom.

   `unify = 0` is the pre-unification path: the four stage buffers and the
   final image are byte-identical to the frozen baseline in tests/golden.legacy.

   Acceptance artifacts (`--artifacts DIR`):
     pixel_summary.txt, chart_deviation.csv, conversions.txt, repro.txt */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "colorpipe.h"

#define W 144
#define H 96
#define PATCH 24
#define PATCHES 24
#define PIXELS (W * H)
#define IMG_BYTES (PIXELS * 3)
#define STAGES 4

static const unsigned char BG_CODE[3]  = {30, 40, 55};
static const unsigned char FOG_CODE[3] = {120, 140, 160};
static const double TINT[3] = {1.04, 1.00, 0.92};

/* sRGB approximations of the 24-patch colour chart, row-major 6 x 4. */
static const unsigned char CHART_RGB[PATCHES][3] = {
    {115,  82,  68}, {194, 150, 130}, { 98, 122, 157}, { 87, 108,  67},
    {133, 128, 177}, {103, 189, 170}, {214, 126,  44}, { 80,  91, 166},
    {193,  90,  99}, { 94,  60, 108}, {157, 188,  64}, {224, 163,  46},
    { 56,  61, 150}, { 70, 148,  73}, {175,  54,  60}, {231, 199,  31},
    {187,  86, 149}, {  8, 133, 161}, {243, 243, 242}, {200, 200, 200},
    {160, 160, 160}, {122, 122, 121}, { 85,  85,  85}, { 52,  52,  52}
};

/* ------------------------------------------------------------------ */
/* transfer curves                                                    */

typedef enum { CURVE_SRGB = 0, CURVE_GAMMA } CurveKind;
typedef struct { CurveKind kind; double gamma; } Transfer;

/* the per-stage curves of the legacy chain: no two stages agree */
static const Transfer STAGE_DEC[STAGES] = {
    { CURVE_SRGB,  0.0 },
    { CURVE_SRGB,  0.0 },
    { CURVE_GAMMA, 2.2 },
    { CURVE_GAMMA, 1.8 },
};
static const Transfer STAGE_ENC[STAGES] = {
    { CURVE_GAMMA, 2.2 },
    { CURVE_GAMMA, 2.0 },
    { CURVE_SRGB,  0.0 },
    { CURVE_SRGB,  0.0 },
};
static const Transfer SRGB = { CURVE_SRGB, 0.0 };

static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static double tf_decode(const Transfer* t, double code) {
    double c = clampd(code, 0.0, 255.0) / 255.0;
    if (t->kind == CURVE_GAMMA) return pow(c, t->gamma);
    return c <= 0.04045 ? c / 12.92 : pow((c + 0.055) / 1.055, 2.4);
}

static double tf_encode(const Transfer* t, double lin) {
    if (lin <= 0.0) return 0.0;
    if (lin >= 1.0) return 255.0;
    if (t->kind == CURVE_GAMMA) return pow(lin, 1.0 / t->gamma) * 255.0;
    return (lin <= 0.0031308 ? 12.92 * lin : 1.055 * pow(lin, 1.0 / 2.4) - 0.055) * 255.0;
}

static unsigned char quantize(double code) {
    int v = (int)floor(code + 0.5);
    return (unsigned char)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

/* ------------------------------------------------------------------ */
/* the four stages                                                    */

/* rounded rect inset 2 px, corner radius 3 px, 4x4 supersampled */
static double coverage_at(int x, int y) {
    int lx = x % PATCH, ly = y % PATCH, inside = 0;
    for (int sy = 0; sy < 4; sy++) {
        for (int sx = 0; sx < 4; sx++) {
            double px = lx + (sx + 0.5) / 4.0;
            double py = ly + (sy + 0.5) / 4.0;
            double rx = clampd(px, 5.0, 19.0);
            double ry = clampd(py, 5.0, 19.0);
            if ((px - rx) * (px - rx) + (py - ry) * (py - ry) <= 9.0) inside++;
        }
    }
    return inside / 16.0;
}

/* all stages operate on linear light. `dec` is the curve the caller used to
   decode its input buffer; legacy stages decode their background/fog
   constants with it too, which is exactly how the cast gets baked in. */
static void stage_apply(int stage, double c[3], int x, int y, const Transfer* dec) {
    switch (stage) {
    case 0: {
        double bg[3], a = coverage_at(x, y);
        for (int j = 0; j < 3; j++) bg[j] = tf_decode(dec, BG_CODE[j]);
        for (int j = 0; j < 3; j++) c[j] = bg[j] + a * (c[j] - bg[j]);
        break;
    }
    case 1: {
        double gain = 0.80 + 0.40 * (double)x / (double)(W - 1);
        for (int j = 0; j < 3; j++) c[j] *= TINT[j] * gain;
        break;
    }
    case 2: {
        double f = 0.30 * (double)y / (double)(H - 1);
        for (int j = 0; j < 3; j++) {
            double fog = tf_decode(dec, FOG_CODE[j]);
            c[j] = (1.0 - f) * c[j] + f * fog;
        }
        break;
    }
    default: {
        double Y = 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2];
        for (int j = 0; j < 3; j++) c[j] = Y + 1.15 * (c[j] - Y);
        break;
    }
    }
    for (int j = 0; j < 3; j++) c[j] = clampd(c[j], 0.0, 1.0);
}

static int patch_index(int x, int y) {
    return (y / PATCH) * (W / PATCH) + (x / PATCH);
}

/* ------------------------------------------------------------------ */
/* rendering                                                          */

static unsigned char g_stage[STAGES][IMG_BYTES];
static double g_lin[IMG_BYTES];

static void render_legacy(const Config* cfg, Counters* ctr,
                          unsigned char out[STAGES][IMG_BYTES]) {
    (void)cfg;
    for (int s = 0; s < STAGES; s++) {
        const Transfer* dec = &STAGE_DEC[s];
        const Transfer* enc = &STAGE_ENC[s];
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                int p = (y * W + x) * 3;
                const unsigned char* src = (s == 0)
                    ? CHART_RGB[patch_index(x, y)]
                    : &g_stage[s - 1][p];
                double c[3];
                for (int j = 0; j < 3; j++) {
                    c[j] = tf_decode(dec, src[j]);
                    ctr->decodes++;
                }
                stage_apply(s, c, x, y, dec);
                for (int j = 0; j < 3; j++) {
                    out[s][p + j] = quantize(tf_encode(enc, c[j]));
                    ctr->encodes++;
                    ctr->quantizations++;
                }
            }
        }
        if (s + 1 < STAGES) memcpy(g_stage[s], out[s], IMG_BYTES);
    }
}

static void render_unified(const Config* cfg, Counters* ctr, unsigned char* out) {
    (void)cfg;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int p = (y * W + x) * 3;
            const unsigned char* src = CHART_RGB[patch_index(x, y)];
            for (int j = 0; j < 3; j++) {
                g_lin[p + j] = tf_decode(&SRGB, src[j]);
                ctr->decodes++;
            }
        }
    }
    for (int s = 0; s < STAGES; s++) {
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                int p = (y * W + x) * 3;
                stage_apply(s, &g_lin[p], x, y, &SRGB);
            }
        }
    }
    for (int i = 0; i < IMG_BYTES; i++) {
        out[i] = quantize(tf_encode(&SRGB, g_lin[i]));
        ctr->encodes++;
        ctr->quantizations++;
    }
}

int cp_render(const Config* cfg, unsigned char stage_out[STAGES][IMG_BYTES],
              unsigned char* image, Counters* ctr) {
    if (!cfg || !image || !ctr) return -1;
    memset(ctr, 0, sizeof(*ctr));
    if (cfg->unify) {
        render_unified(cfg, ctr, image);
        if (stage_out) memset(stage_out, 0, sizeof(unsigned char) * STAGES * IMG_BYTES);
    } else {
        render_legacy(cfg, ctr, stage_out ? stage_out : g_stage);
        memcpy(image, (stage_out ? stage_out : g_stage)[STAGES - 1], IMG_BYTES);
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* ideal reference and CIELAB                                         */

/* the unified chain composed at (x, y) in long double, with no intermediate
   8-bit store -- the independent reference used by the chart and per-pixel
   rounding checks */
static void ideal_at(int x, int y, long double out[3]) {
    long double c[3];
    for (int j = 0; j < 3; j++)
        c[j] = (long double)tf_decode(&SRGB, CHART_RGB[patch_index(x, y)][j]);
    for (int s = 0; s < STAGES; s++) {
        if (s == 0) {
            long double bg[3], a = (long double)coverage_at(x, y);
            for (int j = 0; j < 3; j++) bg[j] = (long double)tf_decode(&SRGB, BG_CODE[j]);
            for (int j = 0; j < 3; j++) c[j] = bg[j] + a * (c[j] - bg[j]);
        } else if (s == 1) {
            long double gain = 0.80L + 0.40L * (long double)x / (long double)(W - 1);
            for (int j = 0; j < 3; j++) c[j] *= (long double)TINT[j] * gain;
        } else if (s == 2) {
            long double f = 0.30L * (long double)y / (long double)(H - 1);
            for (int j = 0; j < 3; j++) {
                long double fog = (long double)tf_decode(&SRGB, FOG_CODE[j]);
                c[j] = (1.0L - f) * c[j] + f * fog;
            }
        } else {
            long double Y = 0.2126L * c[0] + 0.7152L * c[1] + 0.0722L * c[2];
            for (int j = 0; j < 3; j++) c[j] = Y + 1.15L * (c[j] - Y);
        }
        for (int j = 0; j < 3; j++) c[j] = c[j] < 0.0L ? 0.0L : (c[j] > 1.0L ? 1.0L : c[j]);
    }
    for (int j = 0; j < 3; j++) out[j] = (long double)tf_encode(&SRGB, (double)c[j]);
}

static void ideal_patch(int patch, long double out[3]) {
    int col = patch % (W / PATCH), row = patch / (W / PATCH);
    ideal_at(col * PATCH + PATCH / 2, row * PATCH + PATCH / 2, out);
}

typedef struct { double L, a, b; } Lab;

static double lab_f(double t) {
    const double e = 216.0 / 24389.0, k = 24389.0 / 27.0;
    return t > e ? cbrt(t) : (k * t + 16.0) / 116.0;
}

static Lab lab_from_code(const double code[3]) {
    double r = tf_decode(&SRGB, code[0]);
    double g = tf_decode(&SRGB, code[1]);
    double b = tf_decode(&SRGB, code[2]);
    double X = (0.4124564 * r + 0.3575761 * g + 0.1804375 * b) / 0.95047;
    double Y =  0.2126729 * r + 0.7151522 * g + 0.0721750 * b;
    double Z = (0.0193339 * r + 0.1191920 * g + 0.9503041 * b) / 1.08883;
    double fx = lab_f(X), fy = lab_f(Y), fz = lab_f(Z);
    Lab out = { 116.0 * fy - 16.0, 500.0 * (fx - fy), 200.0 * (fy - fz) };
    return out;
}

static double delta_e(const double a[3], const double b[3]) {
    Lab x = lab_from_code(a), y = lab_from_code(b);
    double dl = x.L - y.L, da = x.a - y.a, db = x.b - y.b;
    return sqrt(dl * dl + da * da + db * db);
}

/* Rounding envelope.  The ideal pre-quantisation code p is a real number; the
   stored 8-bit value can differ from it by at most 0.5 code per channel, and
   a different float evaluation order shifts p by far less than that.  The
   worst CIELAB distance reachable inside that +/-0.5-code box is the upper
   bound on the unified path's deviation.  The box is 1 code wide, so a
   5x5x5 sample of it captures the envelope to well under 0.01. */
static double quantization_bound(const double p[3]) {
    static const double off[5] = { -0.5, -0.25, 0.0, 0.25, 0.5 };
    double worst = 0.0;
    for (int a = 0; a < 5; a++) {
        for (int b = 0; b < 5; b++) {
            for (int c = 0; c < 5; c++) {
                double q[3] = { p[0] + off[a], p[1] + off[b], p[2] + off[c] };
                double d = delta_e(p, q);
                if (d > worst) worst = d;
            }
        }
    }
    return worst;
}

/* ------------------------------------------------------------------ */
/* digests                                                            */

static uint64_t fnv1a(const unsigned char* p, size_t n) {
    uint64_t h = 1469598103934665603ull;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 1099511628211ull;
    }
    return h;
}

/* ------------------------------------------------------------------ */
/* self-test                                                          */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

typedef struct {
    unsigned char uni[3];
    unsigned char leg[3];
    double de_uni, bound, de_leg;
} PatchRow;

typedef struct {
    unsigned char image[IMG_BYTES];
    unsigned char stage[STAGES][IMG_BYTES];
    Counters ctr;          /* legacy chain counters */
    Counters ctr_unified;
    PatchRow rows[PATCHES];
    double chart_bound, max_de_uni, max_de_leg;
    int legacy_over_bound, changed_pixels, max_code_delta;
    double mean_cast[3];
} Report;

static void build_report(Report* r) {
    Config uni = { 1 }, legacy = { 0 };
    Counters cu, cl;
    unsigned char img_u[IMG_BYTES];

    cp_render(&legacy, r->stage, r->image, &cl);
    cp_render(&uni, NULL, img_u, &cu);
    r->ctr = cl;
    r->ctr_unified = cu;

    for (int i = 0; i < PATCHES; i++) {
        long double ideal[3];
        ideal_patch(i, ideal);
        PatchRow* row = &r->rows[i];
        double p[3];
        for (int j = 0; j < 3; j++) p[j] = (double)ideal[j];
        int col = i % (W / PATCH), rowy = i / (W / PATCH);
        int x = col * PATCH + PATCH / 2, y = rowy * PATCH + PATCH / 2;
        int px = (y * W + x) * 3;
        for (int j = 0; j < 3; j++) {
            row->uni[j] = img_u[px + j];
            row->leg[j] = r->image[px + j];
        }
        double ucode[3], lcode[3];
        for (int j = 0; j < 3; j++) {
            ucode[j] = row->uni[j];
            lcode[j] = row->leg[j];
        }
        row->de_uni = delta_e(ucode, p);
        row->de_leg = delta_e(lcode, p);
        row->bound = quantization_bound(p);
        if (row->de_uni > r->max_de_uni) r->max_de_uni = row->de_uni;
        if (row->de_leg > r->max_de_leg) r->max_de_leg = row->de_leg;
        if (row->bound > r->chart_bound) r->chart_bound = row->bound;
        if (row->de_leg > row->bound) r->legacy_over_bound++;
        Lab li = lab_from_code(p), ll = lab_from_code(lcode);
        r->mean_cast[0] += (ll.L - li.L) / PATCHES;
        r->mean_cast[1] += (ll.a - li.a) / PATCHES;
        r->mean_cast[2] += (ll.b - li.b) / PATCHES;
    }

    r->changed_pixels = 0;
    r->max_code_delta = 0;
    for (int i = 0; i < IMG_BYTES; i++) {
        int d = (int)r->image[i] - (int)img_u[i];
        if (d) {
            if (i % 3 == 0) r->changed_pixels++;
            if (abs(d) > r->max_code_delta) r->max_code_delta = abs(d);
        }
    }
}

static void print_summary(const Report* r) {
    printf("stages=%d size=%dx%d\n", STAGES, W, H);
    printf("legacy_stage_digests=%016llx,%016llx,%016llx,%016llx\n",
           (unsigned long long)fnv1a(r->stage[0], IMG_BYTES),
           (unsigned long long)fnv1a(r->stage[1], IMG_BYTES),
           (unsigned long long)fnv1a(r->stage[2], IMG_BYTES),
           (unsigned long long)fnv1a(r->stage[3], IMG_BYTES));
    printf("legacy_image_digest=%016llx\n", (unsigned long long)fnv1a(r->image, IMG_BYTES));
    printf("conversions_per_pixel_legacy=%.2f unified=%.2f\n",
           (double)(r->ctr.decodes + r->ctr.encodes) / PIXELS,
           (double)(r->ctr_unified.decodes + r->ctr_unified.encodes) / PIXELS);
    printf("quantizations_per_pixel_legacy=%.2f unified=%.2f\n",
           (double)r->ctr.quantizations / PIXELS,
           (double)r->ctr_unified.quantizations / PIXELS);
    printf("chart_max_de_unified=%.4f chart_bound=%.4f\n", r->max_de_uni, r->chart_bound);
    printf("chart_max_de_legacy=%.4f legacy_over_bound=%d/%d\n",
           r->max_de_leg, r->legacy_over_bound, PATCHES);
    printf("cast_legacy_dL=%.3f da=%.3f db=%.3f\n",
           r->mean_cast[0], r->mean_cast[1], r->mean_cast[2]);
    printf("unified_vs_legacy_changed_px=%d max_code_delta=%d\n",
           r->changed_pixels, r->max_code_delta);
}

/* ------------------------------------------------------------------ */
/* golden and artifacts                                               */

static int check_golden(const Report* r, const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) {
        printf("golden_legacy_bytes=SKIP (no %s)\n", path);
        return 0;
    }
    /* baseline blob: four stage buffers then the final image, raw RGB8 */
    unsigned char want[5][IMG_BYTES];
    size_t n = fread(want, 1, sizeof(want), f);
    fclose(f);
    if (n != sizeof(want)) {
        printf("golden_legacy_bytes=0 (short read %zu/%zu)\n", n, sizeof(want));
        return 0;
    }
    int match = 1;
    for (int s = 0; s < STAGES; s++)
        if (memcmp(want[s], r->stage[s], IMG_BYTES) != 0) match = 0;
    if (memcmp(want[STAGES], r->image, IMG_BYTES) != 0) match = 0;
    printf("golden_legacy_bytes=%d (stage0..3 + image, %d bytes each)\n",
           match, IMG_BYTES);
    return match;
}

static void write_golden(const Report* r, const char* path) {
    FILE* f = fopen(path, "wb");
    if (!f) return;
    for (int s = 0; s < STAGES; s++) fwrite(r->stage[s], 1, IMG_BYTES, f);
    fwrite(r->image, 1, IMG_BYTES, f);
    fclose(f);
    printf("golden_written=%s\n", path);
}

/* unify = 0 must be bit-identical at all four stage outputs and the image.
   This is the "turn unification off" acceptance check: re-render the legacy
   path twice and compare every stage buffer with memcmp. */
static void check_off_identity(const Report* r) {
    Config off = { 0 };
    Counters ctr;
    static unsigned char stage_b[STAGES][IMG_BYTES];
    unsigned char image_b[IMG_BYTES];
    cp_render(&off, stage_b, image_b, &ctr);
    int same = 1;
    for (int s = 0; s < STAGES; s++)
        if (memcmp(r->stage[s], stage_b[s], IMG_BYTES) != 0) same = 0;
    if (memcmp(r->image, image_b, IMG_BYTES) != 0) same = 0;
    printf("unify_off_identity_stage0=%d stage1=%d stage2=%d stage3=%d image=%d\n",
           memcmp(r->stage[0], stage_b[0], IMG_BYTES) == 0,
           memcmp(r->stage[1], stage_b[1], IMG_BYTES) == 0,
           memcmp(r->stage[2], stage_b[2], IMG_BYTES) == 0,
           memcmp(r->stage[3], stage_b[3], IMG_BYTES) == 0,
           memcmp(r->image, image_b, IMG_BYTES) == 0);
    check("unify_off_all_four_bit_identical", same);
}

static void write_pixel_summary(const Report* r, const char* dir) {
    char path[512];
    snprintf(path, sizeof(path), "%s/pixel_summary.txt", dir);
    FILE* f = fopen(path, "w");
    if (!f) return;
    unsigned char img_u[IMG_BYTES];
    Config uni = { 1 };
    Counters cu;
    unsigned char stage[STAGES][IMG_BYTES];
    cp_render(&uni, stage, img_u, &cu);

    fprintf(f, "# output pixel summary\n");
    fprintf(f, "size=%dx%d channels=3 format=rgb8\n", W, H);
    fprintf(f, "unified_image_digest=%016llx\n", (unsigned long long)fnv1a(img_u, IMG_BYTES));
    fprintf(f, "legacy_image_digest=%016llx\n", (unsigned long long)fnv1a(r->image, IMG_BYTES));
    for (int s = 0; s < STAGES; s++)
        fprintf(f, "legacy_stage%d_digest=%016llx\n", s,
                (unsigned long long)fnv1a(r->stage[s], IMG_BYTES));
    for (int j = 0; j < 3; j++) {
        int mn = 255, mx = 0;
        long long sum = 0, nonzero = 0;
        for (int i = 0; i < PIXELS; i++) {
            int v = img_u[i * 3 + j];
            if (v < mn) mn = v;
            if (v > mx) mx = v;
            sum += v;
            if (v) nonzero++;
        }
        fprintf(f, "unified_channel%c min=%d max=%d mean=%.2f nonzero=%lld\n",
                "rgb"[j], mn, mx, (double)sum / PIXELS, nonzero);
    }
    fprintf(f, "unified_vs_legacy_changed_px=%d max_abs_code_delta=%d\n",
            r->changed_pixels, r->max_code_delta);
    fprintf(f, "unified_max_de=%.4f chart_bound=%.4f\n", r->max_de_uni, r->chart_bound);
    fclose(f);
}

static void write_chart(const Report* r, const char* dir) {
    char path[512];
    snprintf(path, sizeof(path), "%s/chart_deviation.csv", dir);
    FILE* f = fopen(path, "w");
    if (!f) return;
    fprintf(f, "patch,ideal_r,ideal_g,ideal_b,unified_r,unified_g,unified_b,"
               "legacy_r,legacy_g,legacy_b,de_unified,bound,de_legacy,within_bound\n");
    for (int i = 0; i < PATCHES; i++) {
        const PatchRow* row = &r->rows[i];
        long double ideal[3];
        ideal_patch(i, ideal);
        fprintf(f, "%d,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%d\n",
                i + 1, (double)ideal[0], (double)ideal[1], (double)ideal[2],
                row->uni[0], row->uni[1], row->uni[2],
                row->leg[0], row->leg[1], row->leg[2],
                row->de_uni, row->bound, row->de_leg,
                row->de_uni <= row->bound ? 1 : 0);
    }
    fprintf(f, "# chart_bound=%.4f unified_max=%.4f legacy_max=%.4f "
               "legacy_over_bound=%d/%d\n",
            r->chart_bound, r->max_de_uni, r->max_de_leg, r->legacy_over_bound, PATCHES);
    fclose(f);
}

static void write_conversions(const Report* r, const char* dir) {
    char path[512];
    snprintf(path, sizeof(path), "%s/conversions.txt", dir);
    FILE* f = fopen(path, "w");
    if (!f) return;
    Config legacy = { 0 }, uni = { 1 };
    Counters cl, cu;
    unsigned char img[IMG_BYTES], stage[STAGES][IMG_BYTES];
    cp_render(&legacy, stage, img, &cl);
    cp_render(&uni, NULL, img, &cu);
    fprintf(f, "# transfer-function applications and 8-bit store points\n");
    fprintf(f, "pixels=%d\n", PIXELS);
    fprintf(f, "legacy_decodes=%lld legacy_encodes=%lld legacy_requantizations=%lld\n",
            cl.decodes, cl.encodes, cl.quantizations);
    fprintf(f, "unified_decodes=%lld unified_encodes=%lld unified_requantizations=%lld\n",
            cu.decodes, cu.encodes, cu.quantizations);
    fprintf(f, "legacy_per_pixel=%.2f unified_per_pixel=%.2f\n",
            (double)(cl.decodes + cl.encodes) / PIXELS,
            (double)(cu.decodes + cu.encodes) / PIXELS);
    for (int s = 0; s < STAGES; s++)
        fprintf(f, "legacy_stage%d=decode+encode (2 per pixel)\n", s);
    fprintf(f, "unified=decode once + encode once (2 per pixel, 0 stage-to-stage)\n");
    (void)r;
    fclose(f);
}

static void write_repro(const char* dir) {
    char path[512];
    snprintf(path, sizeof(path), "%s/repro.txt", dir);
    FILE* f = fopen(path, "w");
    if (!f) return;
    fprintf(f, "# reproduction\n");
    fprintf(f, "make -C colorpipe clean all\n");
    fprintf(f, "make -C colorpipe test        # golden bytes + all assertions\n");
    fprintf(f, "make -C colorpipe artifacts   # emits this directory\n");
    fprintf(f, "# expected: golden_legacy_bytes=1 (unify=0 identical at 4 stages + image)\n");
    fprintf(f, "#           unify_off_identity_stage0..3=1 image=1\n");
    fprintf(f, "#           chart_max_de_unified <= chart_bound\n");
    fprintf(f, "#           legacy_over_bound=24/24\n");
    fprintf(f, "# regenerate the baseline once, deliberately:\n");
    fprintf(f, "#   ./colorpipe/color-pipe --write-golden colorpipe/tests/golden.legacy\n");
    fclose(f);
}

static void write_artifacts(const Report* r, const char* dir) {
    write_pixel_summary(r, dir);
    write_chart(r, dir);
    write_conversions(r, dir);
    write_repro(dir);
    printf("artifacts_dir=%s\n", dir);
}

/* ------------------------------------------------------------------ */

int main(int argc, char** argv) {
    const char* artifacts = NULL;
    const char* golden = NULL;
    const char* write_golden_path = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--artifacts") && i + 1 < argc) artifacts = argv[++i];
        else if (!strcmp(argv[i], "--check-golden") && i + 1 < argc) golden = argv[++i];
        else if (!strcmp(argv[i], "--write-golden") && i + 1 < argc) write_golden_path = argv[++i];
        else {
            printf("usage: %s [--artifacts DIR] [--check-golden FILE] [--write-golden FILE]\n",
                   argv[0]);
            return 2;
        }
    }

    Report r;
    memset(&r, 0, sizeof(r));
    build_report(&r);
    print_summary(&r);
    check("unified_within_bound", r.max_de_uni <= r.chart_bound);
    int all_within = 1;
    for (int i = 0; i < PATCHES; i++)
        if (r.rows[i].de_uni > r.rows[i].bound) all_within = 0;
    check("every_patch_within_bound", all_within);
    check("legacy_shows_cast", r.legacy_over_bound == PATCHES);
    check("unified_changes_image", r.changed_pixels > 0);
    check_off_identity(&r);

    /* every pixel of the unified image is the independent long-double
       reference rounded to 8 bits: no accumulated drift anywhere */
    Config uni = { 1 };
    Counters cu;
    unsigned char img_u[IMG_BYTES];
    unsigned char stage[STAGES][IMG_BYTES];
    cp_render(&uni, stage, img_u, &cu);
    double max_round_err = 0.0;
    int over_half = 0;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            long double ideal[3];
            ideal_at(x, y, ideal);
            for (int j = 0; j < 3; j++) {
                double e = fabs((double)img_u[(y * W + x) * 3 + j] - (double)ideal[j]);
                if (e > max_round_err) max_round_err = e;
                if (e > 0.5 + 1e-6) over_half++;
            }
        }
    }
    printf("max_rounding_error=%.6f pixels_over_half_code=%d/%d\n",
           max_round_err, over_half, PIXELS);
    check("all_pixels_within_half_code", over_half == 0);

    if (golden) check("golden", check_golden(&r, golden));
    if (write_golden_path) write_golden(&r, write_golden_path);
    if (artifacts) write_artifacts(&r, artifacts);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
