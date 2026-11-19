
/* fontatlas: shelf packing of glyph bitmaps into a fixed atlas, plus kerning. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ATLAS_W 256
#define ATLAS_H 256
#define MAX_GLYPHS 64
#define MAX_KERN 32

typedef struct { int w, h; } GlyphSize;
typedef struct { int x, y, w, h; int placed; } GlyphRect;
typedef struct { char left, right; int delta; } KernPair;

typedef struct {
    int used_h;        /* current shelf top */
    int shelf_x;       /* next x on the current shelf */
    int shelf_h;       /* height of the current shelf */
    GlyphRect rects[MAX_GLYPHS];
    int count;
} Atlas;

static void atlas_init(Atlas* a) {
    memset(a, 0, sizeof(*a));
}

/* shelf packing: place to the right on the same shelf, or open a new shelf */
static int atlas_add(Atlas* a, GlyphSize g, int* out_index) {
    if (a->count >= MAX_GLYPHS) return -1;
    if (g.w <= 0 || g.h <= 0 || g.w > ATLAS_W || g.h > ATLAS_H) return -1;
    if (a->shelf_x + g.w > ATLAS_W) {
        a->used_h += a->shelf_h;
        a->shelf_x = 0;
        a->shelf_h = 0;
    }
    if (a->used_h + g.h > ATLAS_H) return -1;

    GlyphRect* r = &a->rects[a->count];
    r->x = a->shelf_x;
    r->y = a->used_h;
    r->w = g.w;
    r->h = g.h;
    r->placed = 1;
    a->shelf_x += g.w;
    if (g.h > a->shelf_h) a->shelf_h = g.h;
    if (out_index) *out_index = a->count;
    a->count++;
    return 0;
}

static int atlas_overlap(const Atlas* a) {
    for (int i = 0; i < a->count; i++) {
        const GlyphRect* p = &a->rects[i];
        for (int j = i + 1; j < a->count; j++) {
            const GlyphRect* q = &a->rects[j];
            int sep = p->x + p->w <= q->x || q->x + q->w <= p->x ||
                      p->y + p->h <= q->y || q->y + q->h <= p->y;
            if (!sep) return 1;
        }
    }
    return 0;
}

static int kerning_apply(const KernPair* pairs, int n, const char* text) {
    int total = 0;
    for (const char* p = text; *p; p++) {
        total += 1;   /* placeholder advance per glyph */
        if (p[1]) {
            for (int i = 0; i < n; i++)
                if (pairs[i].left == *p && pairs[i].right == p[1]) total += pairs[i].delta;
        }
    }
    return total;
}

/* --- gamma correction -----------------------------------------------------
 * Coverage in the atlas is a linear-light fraction, but renderers typically
 * blend it in encoded (sRGB-ish) space, which darkens mid coverage and makes
 * edges look gray on dark backgrounds. Two tools:
 *   1. coverage_gamma(): remap coverage c -> c^(1/gamma) at atlas build time,
 *      so the renderer's naive encoded-space lerp approximates the linear
 *      result. Purely a per-pixel value remap: glyph geometry is untouched.
 *   2. blend_linear(): the linear-light reference (decode, lerp, encode),
 *      i.e. what manual correction under linear lighting produces.
 * GammaConfig.enabled toggles the remap off to get the original curve back.
 */
#define FONTATLAS_GAMMA 2.2

typedef struct { double gamma; int enabled; } GammaConfig;

static uint8_t clamp8(double v) {
    if (v < 0.0) return 0;
    if (v > 255.0) return 255;
    return (uint8_t)v;
}

static uint8_t coverage_gamma(uint8_t c, double gamma) {
    if (c == 0 || c == 255) return c;   /* fixed points: shape preserved */
    return clamp8(pow(c / 255.0, 1.0 / gamma) * 255.0 + 0.5);
}

static void bitmap_gamma(uint8_t* px, int n, const GammaConfig* cfg) {
    if (!cfg->enabled) return;   /* original curve */
    for (int i = 0; i < n; i++) px[i] = coverage_gamma(px[i], cfg->gamma);
}

static double to_linear(uint8_t v, double gamma) { return pow(v / 255.0, gamma); }
static uint8_t to_encoded(double l, double gamma) {
    return clamp8(pow(l, 1.0 / gamma) * 255.0 + 0.5);
}

/* what the renderer does today: lerp in encoded space */
static uint8_t blend_naive(uint8_t fg, uint8_t bg, uint8_t cov) {
    return (uint8_t)((fg * cov + bg * (255 - cov) + 127) / 255);
}

/* linear-light reference: decode both endpoints, lerp, re-encode */
static uint8_t blend_linear(uint8_t fg, uint8_t bg, uint8_t cov, double gamma) {
    double a = cov / 255.0;
    double l = a * to_linear(fg, gamma) + (1.0 - a) * to_linear(bg, gamma);
    return to_encoded(l, gamma);
}

/* WCAG-style contrast ratio between two linear luminances */
static double contrast_ratio(double l1, double l2) {
    if (l1 < l2) { double t = l1; l1 = l2; l2 = t; }
    return (l1 + 0.05) / (l2 + 0.05);
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Atlas a;
    atlas_init(&a);
    int idx = -1;

    check("add_small", atlas_add(&a, (GlyphSize){16, 16}, &idx) == 0 && idx == 0);
    check("add_more", atlas_add(&a, (GlyphSize){32, 32}, NULL) == 0);
    check("no_overlap_two", atlas_overlap(&a) == 0);
    check("rect_in_bounds", a.rects[1].x + a.rects[1].w <= ATLAS_W && a.rects[1].y + a.rects[1].h <= ATLAS_H);

    /* fill the shelf so it must wrap */
    while (atlas_add(&a, (GlyphSize){40, 20}, NULL) == 0) { if (a.count >= 20) break; }
    check("shelf_wrap_used_h", a.used_h > 0);
    check("no_overlap_after_wrap", atlas_overlap(&a) == 0);

    Atlas full;
    atlas_init(&full);
    int rc = 0, n = 0;
    while ((rc = atlas_add(&full, (GlyphSize){64, 64}, NULL)) == 0) { if (++n > MAX_GLYPHS) break; }
    check("atlas_fills_then_fails", rc == -1 && n > 0);
    check("full_no_overlap", atlas_overlap(&full) == 0);

    Atlas bad;
    atlas_init(&bad);
    check("reject_zero", atlas_add(&bad, (GlyphSize){0, 10}, NULL) == -1);
    check("reject_huge", atlas_add(&bad, (GlyphSize){ATLAS_W + 1, 10}, NULL) == -1);

    KernPair pairs[] = { {'A', 'V', -1}, {'T', 'o', -1} };
    check("kern_no_pairs", kerning_apply(pairs, 2, "A") == 1);
    check("kern_AV", kerning_apply(pairs, 2, "AV") == 1);
    check("kern_To", kerning_apply(pairs, 2, "To") == 1);
    check("kern_plain", kerning_apply(pairs, 2, "AB") == 2);

    /* --- gamma correction --- */
    const double g = FONTATLAS_GAMMA;
    GammaConfig gamma_off = { g, 0 };
    GammaConfig gamma_on  = { g, 1 };

    /* fake glyph edge: full range of coverages */
    uint8_t glyph[64], orig[64];
    for (int i = 0; i < 64; i++) glyph[i] = orig[i] = (uint8_t)(i * 4); /* 0..252 */

    /* toggling off must return the original curve byte-for-byte */
    bitmap_gamma(glyph, 64, &gamma_off);
    check("toggle_off_byte_identical", memcmp(glyph, orig, 64) == 0);

    /* toggling on remaps values but not shape: silhouette and solid core stay */
    bitmap_gamma(glyph, 64, &gamma_on);
    int shape_same = 1;
    for (int i = 0; i < 64; i++) {
        if ((orig[i] > 0) != (glyph[i] > 0)) shape_same = 0;       /* silhouette */
        if (orig[i] == 255 && glyph[i] != 255) shape_same = 0;     /* solid core */
        if (orig[i] == 0 && glyph[i] != 0) shape_same = 0;
    }
    check("toggle_on_shape_unchanged", shape_same);

    int mono = 1;
    for (int c = 1; c <= 255; c++)
        if (coverage_gamma((uint8_t)c, g) < coverage_gamma((uint8_t)(c - 1), g)) mono = 0;
    check("remap_monotonic", mono);
    check("remap_endpoints_fixed", coverage_gamma(0, g) == 0 && coverage_gamma(255, g) == 255);

    /* consistency: remapped coverage + naive blend ~= linear-light reference.
     * Exact (rounding only) on a pure black background... */
    int maxerr_black = 0;
    for (int c = 0; c <= 255; c++) {
        int got = blend_naive(255, 0, coverage_gamma((uint8_t)c, g));
        int ref = blend_linear(255, 0, (uint8_t)c, g);
        int err = got - ref; if (err < 0) err = -err;
        if (err > maxerr_black) maxerr_black = err;
    }
    printf("maxerr_vs_linear_bg0=%d\n", maxerr_black);
    check("remap_matches_linear_black_bg", maxerr_black <= 1);

    /* ...and small on a near-black background (the remap is exact only when
     * bg=0; a nonzero bg leaves a bounded residual, still far below the
     * uncorrected pipeline's error) */
    int maxerr_dark = 0, maxerr_dark_raw = 0;
    for (int c = 0; c <= 255; c++) {
        int ref = blend_linear(255, 16, (uint8_t)c, g);
        int got = blend_naive(255, 16, coverage_gamma((uint8_t)c, g));
        int raw = blend_naive(255, 16, (uint8_t)c);
        int err = got - ref; if (err < 0) err = -err;
        int rerr = raw - ref; if (rerr < 0) rerr = -rerr;
        if (err > maxerr_dark) maxerr_dark = err;
        if (rerr > maxerr_dark_raw) maxerr_dark_raw = rerr;
    }
    printf("maxerr_vs_linear_bg16=%d maxerr_uncorrected_bg16=%d\n", maxerr_dark, maxerr_dark_raw);
    check("remap_matches_linear_dark_bg", maxerr_dark <= 16);
    check("remap_much_closer_than_raw", maxerr_dark * 3 <= maxerr_dark_raw);

    /* linear reference sanity: endpoints exact */
    check("linear_blend_endpoints",
          blend_linear(255, 16, 0, g) == 16 && blend_linear(255, 16, 255, g) == 255);

    /* contrast data: edge pixel at 50% coverage, naive vs gamma-corrected.
     * Dark bg: white text on bg=16. Light bg: black text on bg=240. */
    uint8_t c50 = coverage_gamma(128, g);
    uint8_t edge_dark_naive = blend_naive(255, 16, 128);
    uint8_t edge_dark_corr  = blend_naive(255, 16, c50);
    uint8_t edge_lite_naive = blend_naive(0, 240, 128);
    uint8_t edge_lite_corr  = blend_naive(0, 240, c50);

    double cr_dark_naive = contrast_ratio(to_linear(edge_dark_naive, g), to_linear(16, g));
    double cr_dark_corr  = contrast_ratio(to_linear(edge_dark_corr, g),  to_linear(16, g));
    double cr_lite_naive = contrast_ratio(to_linear(edge_lite_naive, g), to_linear(240, g));
    double cr_lite_corr  = contrast_ratio(to_linear(edge_lite_corr, g),  to_linear(240, g));

    printf("edge_dark_naive=%d edge_dark_corr=%d\n", edge_dark_naive, edge_dark_corr);
    printf("edge_lite_naive=%d edge_lite_corr=%d\n", edge_lite_naive, edge_lite_corr);
    printf("contrast_dark_naive=%.2f contrast_dark_corr=%.2f\n", cr_dark_naive, cr_dark_corr);
    printf("contrast_lite_naive=%.2f contrast_lite_corr=%.2f\n", cr_lite_naive, cr_lite_corr);
    check("contrast_improves_dark_bg", cr_dark_corr > cr_dark_naive);
    check("contrast_improves_lite_bg", cr_lite_corr > cr_lite_naive);

    /* corrected blend must track the linear reference on the dark bg */
    check("corrected_edge_near_linear",
          abs(edge_dark_corr - blend_linear(255, 16, 128, g)) <= 8);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
