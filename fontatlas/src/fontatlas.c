
/* fontatlas: shelf packing of glyph bitmaps into a fixed atlas, plus kerning,
   and synthetic bold / italic variants for fonts that ship a single face.

   Fake bold offsets the glyph outline along its stroke normals: a signed
   distance field is built from a supersampled coverage mask, then every
   contour is pushed out by the requested radius.  Fake italic shears rows
   horizontally about a baseline; each row is resampled with exact area
   weights into a wider canvas, so no ink is clipped and no edge is lost. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define ATLAS_W 256
#define ATLAS_H 256
#define MAX_GLYPHS 64
#define MAX_KERN 32

/* largest glyph box the style synthesizer produces (after padding) */
#define GLYPH_MAX 72
#define STYLE_SS 4                          /* supersampling for the distance field */
#define STYLE_HI (GLYPH_MAX * STYLE_SS)
#define STYLE_INF (1 << 20)
#define STYLE_SCALE 1024                    /* fixed point for the shear weights */
#define BOLD_MAX_RADIUS 2.0f
#define ITALIC_MAX_SLANT 1.0f

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

typedef struct {
    int w, h;
    int origin_x, origin_y;   /* bitmap (0,0) relative to the original glyph origin */
    unsigned char a[GLYPH_MAX][GLYPH_MAX];   /* coverage, [y][x] */
} GlyphBitmap;

typedef struct {
    float bold;        /* outline radius in pixels, 0 = off */
    float slant;       /* horizontal shear per pixel of height, 0 = off */
    float baseline;    /* y coordinate the slant pivots around */
} GlyphStyle;

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

/* ---------------- glyph bitmaps ---------------- */

static int bitmap_init(GlyphBitmap* g, int w, int h) {
    if (w <= 0 || h <= 0 || w > GLYPH_MAX || h > GLYPH_MAX) return -1;
    memset(g, 0, sizeof(*g));
    g->w = w;
    g->h = h;
    return 0;
}

static unsigned bitmap_at(const GlyphBitmap* g, int x, int y) {
    if (x < 0 || y < 0 || x >= g->w || y >= g->h) return 0;
    return g->a[y][x];
}

/* bilinear coverage sample in bitmap coordinates, 0..1 */
static float bitmap_sample(const GlyphBitmap* g, float x, float y) {
    int x0 = (int)floorf(x), y0 = (int)floorf(y);
    float fx = x - x0, fy = y - y0, v = 0.0f;
    for (int oy = 0; oy < 2; oy++)
        for (int ox = 0; ox < 2; ox++)
            v += bitmap_at(g, x0 + ox, y0 + oy) * (ox ? fx : 1.0f - fx) * (oy ? fy : 1.0f - fy);
    return v / 255.0f;
}

/* ---------------- signed distance field ---------------- */

/* Felzenszwalb-Huttenlocher squared Euclidean distance transform, 1D pass */
static void edt_1d(const int* f, int n, int* d) {
    static int v[STYLE_HI + 1];
    static double z[STYLE_HI + 1];
    int k = 0;
    v[0] = 0;
    z[0] = -1e30;
    z[1] = 1e30;
    for (int q = 1; q < n; q++) {
        double s;
        for (;;) {
            int p = v[k];
            s = ((double)(f[q] + q * q) - (double)(f[p] + p * p)) / (2.0 * q - 2.0 * p);
            if (s <= z[k]) { k--; continue; }
            break;
        }
        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = 1e30;
    }
    k = 0;
    for (int q = 0; q < n; q++) {
        while (z[k + 1] < (double)q) k++;
        int dp = q - v[k];
        d[q] = dp * dp + f[v[k]];
    }
}

/* distance from every pixel to the nearest seed pixel, squared */
static void edt_2d(const unsigned char* seed, int w, int h, int* out) {
    static int f[STYLE_HI * STYLE_HI];
    static int col[STYLE_HI], cold[STYLE_HI];
    for (int i = 0; i < w * h; i++) f[i] = seed[i] ? 0 : STYLE_INF;
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) col[y] = f[y * w + x];
        edt_1d(col, h, cold);
        for (int y = 0; y < h; y++) f[y * w + x] = cold[y];
    }
    for (int y = 0; y < h; y++) edt_1d(f + y * w, w, out + y * w);
}

/* ---------------- fake bold ----------------
   The outline is displaced outward by `radius` pixels.  Working from a signed
   distance field means every stroke widens by 2*radius along its own normal,
   including diagonals, curves and stroke terminals, which matches a real bold
   face up to the raster grid. */
static int bold_synthesize(const GlyphBitmap* src, float radius, GlyphBitmap* dst) {
    if (radius < 0.0f || radius > BOLD_MAX_RADIUS) return -1;
    if (radius == 0.0f) { *dst = *src; return 0; }

    int pad = (int)ceilf(radius) + 1;   /* room for the growth plus its antialias */
    int w = src->w + 2 * pad, h = src->h + 2 * pad;
    if (w > GLYPH_MAX || h > GLYPH_MAX) return -1;

    GlyphBitmap in;
    bitmap_init(&in, w, h);
    in.origin_x = src->origin_x - pad;
    in.origin_y = src->origin_y - pad;
    for (int y = 0; y < src->h; y++)
        for (int x = 0; x < src->w; x++)
            in.a[y + pad][x + pad] = src->a[y][x];

    static float hi[STYLE_HI * STYLE_HI];
    static float out[STYLE_HI * STYLE_HI];
    static unsigned char ink[STYLE_HI * STYLE_HI];
    static unsigned char bg[STYLE_HI * STYLE_HI];
    static int d_ink[STYLE_HI * STYLE_HI];   /* nearest ink, used outside */
    static int d_bg[STYLE_HI * STYLE_HI];    /* nearest background, used inside */

    int W = w * STYLE_SS, H = h * STYLE_SS;
    int has_ink = 0, has_bg = 0;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float fx = (x + 0.5f) / STYLE_SS - 0.5f;
            float fy = (y + 0.5f) / STYLE_SS - 0.5f;
            int x0 = (int)floorf(fx), y0 = (int)floorf(fy);
            float ax = fx - x0, ay = fy - y0, v = 0.0f;
            for (int oy = 0; oy < 2; oy++)
                for (int ox = 0; ox < 2; ox++) {
                    int qx = x0 + ox, qy = y0 + oy;
                    if (qx < 0 || qy < 0 || qx >= w || qy >= h) continue;
                    v += in.a[qy][qx] * (ox ? ax : 1.0f - ax) * (oy ? ay : 1.0f - ay);
                }
            hi[y * W + x] = v / 255.0f;
            if (hi[y * W + x] >= 0.5f) has_ink = 1;
            else has_bg = 1;
        }
    }
    if (!has_ink || !has_bg) { *dst = in; return 0; }

    for (int i = 0; i < W * H; i++) {
        ink[i] = hi[i] >= 0.5f;
        bg[i] = (unsigned char)!ink[i];
    }
    edt_2d(ink, W, H, d_ink);
    edt_2d(bg, W, H, d_bg);

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int i = y * W + x;
            /* signed distance in high-res pixels, positive inside the 0.5 contour;
               a straight edge sits half a high-res pixel from the last ink center */
            float sdf = ink[i] ? (sqrtf((float)d_bg[i]) - 0.5f)
                               : -(sqrtf((float)d_ink[i]) - 0.5f);
            float v = 0.5f + sdf / STYLE_SS + radius;
            out[i] = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
        }
    }

    bitmap_init(dst, w, h);
    dst->origin_x = in.origin_x;
    dst->origin_y = in.origin_y;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float s = 0.0f;
            for (int sy = 0; sy < STYLE_SS; sy++)
                for (int sx = 0; sx < STYLE_SS; sx++)
                    s += out[(y * STYLE_SS + sy) * W + x * STYLE_SS + sx];
            float v = s * (255.0f / (STYLE_SS * STYLE_SS));
            dst->a[y][x] = (unsigned char)(v + 0.5f);
        }
    }
    return 0;
}

static int floor_div(int a, int b) {
    int q = a / b;
    if ((a % b) != 0 && ((a < 0) != (b < 0))) q--;
    return q;
}

static int ceil_div(int a, int b) {
    int q = a / b;
    if ((a % b) != 0 && ((a < 0) == (b < 0))) q++;
    return q;
}

/* ---------------- fake italic ----------------
   x' = x + slant * (baseline - y).  Rows are translated, never scaled, so each
   row keeps its ink exactly: the row is resampled with exact fixed-point area
   weights into an expanded canvas, and the glyph origin follows the shift. */
static int italic_synthesize(const GlyphBitmap* src, float slant, float baseline, GlyphBitmap* dst) {
    if (slant < -ITALIC_MAX_SLANT || slant > ITALIC_MAX_SLANT) return -1;
    if (slant == 0.0f) { *dst = *src; return 0; }

    int lo = 0, hi = 0;                  /* sheared ink bounds, fixed point */
    int have_ink = 0;
    for (int y = 0; y < src->h; y++) {
        int off = (int)lroundf(slant * (baseline - (float)y) * STYLE_SCALE);
        for (int x = 0; x < src->w; x++) {
            if (!src->a[y][x]) continue;
            int a = x * STYLE_SCALE + off;
            int b = (x + 1) * STYLE_SCALE + off;
            if (!have_ink) { lo = a; hi = b; have_ink = 1; }
            else {
                if (a < lo) lo = a;
                if (b > hi) hi = b;
            }
        }
    }
    if (!have_ink) { *dst = *src; return 0; }   /* empty glyph */

    /* the canvas is exactly the sheared ink box: nothing is clipped */
    int left = floor_div(lo, STYLE_SCALE);
    int right = ceil_div(hi, STYLE_SCALE);
    int w = right - left;
    if (w <= 0 || w > GLYPH_MAX || src->h > GLYPH_MAX) return -1;

    bitmap_init(dst, w, src->h);
    dst->origin_x = src->origin_x + left;
    dst->origin_y = src->origin_y;

    for (int y = 0; y < src->h; y++) {
        int off = (int)lroundf(slant * (baseline - (float)y) * STYLE_SCALE);
        for (int x = 0; x < src->w; x++) {
            unsigned a = src->a[y][x];
            if (!a) continue;
            int p = x * STYLE_SCALE + off - left * STYLE_SCALE;   /* start of [p, p+S) */
            int i0 = p / STYLE_SCALE;
            int frac = p % STYLE_SCALE;
            unsigned first = (unsigned)(((long)a * (STYLE_SCALE - frac) + STYLE_SCALE / 2) / STYLE_SCALE);
            unsigned second = a - first;
            if (i0 >= 0 && i0 < dst->w) {
                unsigned s = (unsigned)dst->a[y][i0] + first;
                dst->a[y][i0] = (unsigned char)(s > 255 ? 255 : s);
            }
            if (second && i0 + 1 < dst->w) {
                unsigned s = (unsigned)dst->a[y][i0 + 1] + second;
                dst->a[y][i0 + 1] = (unsigned char)(s > 255 ? 255 : s);
            }
        }
    }
    return 0;
}

/* compose styles: bold first (isotropic thickening), then the slant */
static int glyph_apply_style(const GlyphBitmap* src, const GlyphStyle* st, GlyphBitmap* dst) {
    GlyphBitmap bold, slanted;
    const GlyphBitmap* cur = src;
    if (st->bold > 0.0f) {
        if (bold_synthesize(cur, st->bold, &bold) != 0) return -1;
        cur = &bold;
    }
    if (st->slant != 0.0f) {
        if (italic_synthesize(cur, st->slant, st->baseline, &slanted) != 0) return -1;
        cur = &slanted;
    }
    *dst = *cur;
    return 0;
}

/* ---------------- self checks ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* test-only rasterizer: capsule strokes and rings in a 48x48 box, 4x4 samples */
#define TEST_SIZE 48
#define TEST_SS 4

typedef struct { int ring; float x0, y0, x1, y1, r; } TestPrim;
typedef struct { TestPrim p[10]; int n; } TestGlyph;

static void tg_seg(TestGlyph* g, float x0, float y0, float x1, float y1, float r) {
    g->p[g->n++] = (TestPrim){0, x0, y0, x1, y1, r};
}

static void tg_ring(TestGlyph* g, float cx, float cy, float radius, float r) {
    g->p[g->n++] = (TestPrim){1, cx, cy, radius, 0.0f, r};
}

static float seg_dist(float px, float py, float x0, float y0, float x1, float y1) {
    float dx = x1 - x0, dy = y1 - y0, L2 = dx * dx + dy * dy;
    float t = L2 > 0.0f ? ((px - x0) * dx + (py - y0) * dy) / L2 : 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return hypotf(px - (x0 + t * dx), py - (y0 + t * dy));
}

static void test_glyph_make(int id, float r, TestGlyph* g) {
    g->n = 0;
    switch (id) {
    case 0:   /* I */
        tg_seg(g, 24, 12, 24, 40, r);
        tg_seg(g, 16, 12, 32, 12, r);
        tg_seg(g, 16, 40, 32, 40, r);
        break;
    case 1:   /* L */
        tg_seg(g, 16, 12, 16, 40, r);
        tg_seg(g, 16, 40, 36, 40, r);
        break;
    case 2:   /* V */
        tg_seg(g, 13, 12, 26, 40, r);
        tg_seg(g, 39, 12, 26, 40, r);
        break;
    case 3:   /* T */
        tg_seg(g, 13, 12, 39, 12, r);
        tg_seg(g, 26, 12, 26, 40, r);
        break;
    case 4:   /* O */
        tg_ring(g, 26, 26, 12, r);
        break;
    case 5:   /* A */
        tg_seg(g, 13, 40, 26, 12, r);
        tg_seg(g, 39, 40, 26, 12, r);
        tg_seg(g, 19, 29, 33, 29, r);
        break;
    default:  /* N */
        tg_seg(g, 15, 40, 15, 12, r);
        tg_seg(g, 15, 12, 37, 40, r);
        tg_seg(g, 37, 40, 37, 12, r);
        break;
    }
}

static void test_render(int id, float r, GlyphBitmap* out) {
    TestGlyph g;
    test_glyph_make(id, r, &g);
    bitmap_init(out, TEST_SIZE, TEST_SIZE);
    for (int y = 0; y < TEST_SIZE; y++) {
        for (int x = 0; x < TEST_SIZE; x++) {
            int hits = 0;
            for (int sy = 0; sy < TEST_SS; sy++) {
                for (int sx = 0; sx < TEST_SS; sx++) {
                    float px = x + (sx + 0.5f) / TEST_SS;
                    float py = y + (sy + 0.5f) / TEST_SS;
                    for (int i = 0; i < g.n; i++) {
                        float d;
                        if (g.p[i].ring)
                            d = fabsf(hypotf(px - g.p[i].x0, py - g.p[i].y0) - g.p[i].x1);
                        else
                            d = seg_dist(px, py, g.p[i].x0, g.p[i].y0, g.p[i].x1, g.p[i].y1);
                        if (d <= g.p[i].r) { hits++; break; }
                    }
                }
            }
            out->a[y][x] = (unsigned char)((255 * hits + TEST_SS * TEST_SS / 2) / (TEST_SS * TEST_SS));
        }
    }
}

/* width of the stroke crossed by a line, measured at the 0.5 coverage contour */
static float probe_width(const GlyphBitmap* g, float px, float py, float dx, float dy, float len) {
    const float step = 0.02f;
    int n = (int)(len / step);
    float inside = 0.0f, prev = 0.0f;
    for (int i = 0; i <= n; i++) {
        float t = -0.5f * len + (float)i * step;
        float v = bitmap_sample(g, px + dx * t, py + dy * t);
        if (i > 0) {
            if ((prev - 0.5f) * (v - 0.5f) < 0.0f) inside += step * (0.5f - prev) / (v - prev);
            else if (v >= 0.5f) inside += step;
        }
        prev = v;
    }
    return inside;
}

typedef struct { int glyph; float x, y, dx, dy; } Probe;

static const Probe probes[] = {
    {0, 24, 26, 1, 0}, {0, 24, 12, 0, 1},
    {1, 16, 26, 1, 0}, {1, 26, 40, 0, 1},
    {2, 19.5f, 25, -0.918f, 0.397f},
    {3, 26, 26, 1, 0}, {3, 26, 12, 0, 1},
    {4, 26, 14, 0, 1}, {4, 14, 26, 1, 0}, {4, 26, 38, 0, 1}, {4, 38, 26, 1, 0},
    {5, 19.5f, 25, 0.918f, 0.397f}, {5, 26, 29, 0, 1},
    {6, 15, 26, 1, 0}, {6, 26, 26, -0.786f, 0.618f}, {6, 37, 26, 1, 0},
};
#define NPROBES ((int)(sizeof(probes) / sizeof(probes[0])))

/* a probe lives in the original (unstyled) glyph frame; the styled bitmap has
   its own origin, so shift the coordinates into it */
static float probe_styled(const GlyphBitmap* g, const Probe* p) {
    float x = p->x - (float)g->origin_x;
    float y = p->y - (float)g->origin_y;
    return probe_width(g, x, y, p->dx, p->dy, 20.0f);
}

static int bitmap_border_ink(const GlyphBitmap* g) {
    for (int x = 0; x < g->w; x++)
        if (g->a[0][x] || g->a[g->h - 1][x]) return 1;
    for (int y = 0; y < g->h; y++)
        if (g->a[y][0] || g->a[y][g->w - 1]) return 1;
    return 0;
}

/* ink runs of one row, weighted by coverage */
static int row_runs(const GlyphBitmap* g, int y, float* runs, int max) {
    int n = 0, in = 0;
    float acc = 0.0f;
    for (int x = 0; x < g->w; x++) {
        unsigned a = g->a[y][x];
        if (a) { acc += a / 255.0f; in = 1; }
        else if (in) { if (n < max) runs[n++] = acc; acc = 0.0f; in = 0; }
    }
    if (in && n < max) runs[n++] = acc;
    return n;
}

static float row_centroid(const GlyphBitmap* g, int y) {
    float sw = 0.0f, sx = 0.0f;
    for (int x = 0; x < g->w; x++) {
        sw += g->a[y][x];
        sx += g->a[y][x] * (x + 0.5f);
    }
    return sw > 0.0f ? sx / sw : 0.0f;
}

static int row_sum(const GlyphBitmap* g, int y) {
    int s = 0;
    for (int x = 0; x < g->w; x++) s += g->a[y][x];
    return s;
}

/* every ink pixel with real coverage must still be present after the shear */
static int italic_pixels_present(const GlyphBitmap* src, float slant, float baseline, const GlyphBitmap* dst) {
    int missing = 0;
    for (int y = 0; y < src->h; y++) {
        float off = slant * (baseline - (float)y) + (float)src->origin_x - (float)dst->origin_x;
        for (int x = 0; x < src->w; x++) {
            if (src->a[y][x] < 8) continue;
            float v = bitmap_sample(dst, (float)x + 0.5f + off, (float)y + 0.5f);
            if (v < 0.004f) missing++;
        }
    }
    return missing;
}

/* the smallest box holding every sheared ink pixel, in the source bitmap frame;
   fixed point so it lines up exactly with what italic_synthesize computes */
static void italic_expected_bounds(const GlyphBitmap* src, float slant, float baseline,
                                   int* left, int* right) {
    int lo = 0, hi = 0, have = 0;
    for (int y = 0; y < src->h; y++) {
        int off = (int)lroundf(slant * (baseline - (float)y) * STYLE_SCALE);
        for (int x = 0; x < src->w; x++) {
            if (!src->a[y][x]) continue;
            int a = x * STYLE_SCALE + off, b = (x + 1) * STYLE_SCALE + off;
            if (!have) { lo = a; hi = b; have = 1; }
            else {
                if (a < lo) lo = a;
                if (b > hi) hi = b;
            }
        }
    }
    *left = floor_div(lo, STYLE_SCALE);
    *right = ceil_div(hi, STYLE_SCALE);
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

    /* ---- fake bold: stroke width against a genuinely bolder face ---- */
    {
        const float radii[] = { 0.5f, 0.75f, 1.0f };
        float worst = 0.0f;
        int probes_checked = 0, missing_glyph = 0;
        for (int gi = 0; gi < 7; gi++) {
            GlyphBitmap regular, real, fake;
            test_render(gi, 1.5f, &regular);
            for (int ri = 0; ri < 3; ri++) {
                test_render(gi, 1.5f + radii[ri], &real);
                if (bold_synthesize(&regular, radii[ri], &fake) != 0) { missing_glyph++; continue; }
                for (int pi = 0; pi < NPROBES; pi++) {
                    if (probes[pi].glyph != gi) continue;
                    float wf = probe_styled(&fake, &probes[pi]);
                    float wr = probe_styled(&real, &probes[pi]);
                    float d = fabsf(wf - wr);
                    if (d > worst) worst = d;
                    probes_checked++;
                }
            }
        }
        printf("bold_worst_width_error=%.3f probes=%d\n", worst, probes_checked);
        check("bold_all_synthesized", missing_glyph == 0);
        check("bold_width_under_1px", probes_checked > 40 && worst < 1.0f);
    }

    /* bold must not clip its own growth */
    {
        GlyphBitmap regular, fake, fake2;
        int clips = 0;
        test_render(0, 1.5f, &regular);
        if (bold_synthesize(&regular, 1.0f, &fake) == 0) {
            if (bitmap_border_ink(&fake)) clips++;
            if (fake.w != regular.w + 4 || fake.h != regular.h + 4) clips++;
            if (fake.origin_x != regular.origin_x - 2 || fake.origin_y != regular.origin_y - 2) clips++;
        } else clips++;
        test_render(4, 1.5f, &regular);   /* curves too */
        if (bold_synthesize(&regular, 2.0f, &fake2) == 0) {
            if (bitmap_border_ink(&fake2)) clips++;
            if (fake2.w != regular.w + 6 || fake2.h != regular.h + 6) clips++;
        } else clips++;
        check("bold_pads_without_clipping", clips == 0);
    }

    /* radius 0 is a plain copy; oversized input is rejected */
    {
        GlyphBitmap regular, same;
        test_render(3, 1.5f, &regular);
        int ok = bold_synthesize(&regular, 0.0f, &same) == 0 &&
                 same.w == regular.w && same.h == regular.h &&
                 memcmp(same.a, regular.a, sizeof(regular.a)) == 0;
        check("bold_zero_is_copy", ok);

        GlyphBitmap big;
        bitmap_init(&big, GLYPH_MAX - 2, GLYPH_MAX - 2);
        check("bold_rejects_overflow", bold_synthesize(&big, 2.0f, &same) == -1);
        check("bold_rejects_radius", bold_synthesize(&regular, 3.0f, &same) == -1);
    }

    /* ---- fake italic: bounds, conserved ink, correct slant ---- */
    {
        const float slants[] = { 0.25f, -0.25f, 0.5f };
        int clip = 0, rows_mismatch = 0, pixels_lost = 0, geom_bad = 0;
        unsigned total_src = 0, total_dst = 0;
        for (int gi = 0; gi < 7; gi++) {
            GlyphBitmap regular;
            test_render(gi, 1.5f, &regular);
            float baseline = (float)(regular.h - 1);
            for (int si = 0; si < 3; si++) {
                GlyphBitmap tilted;
                if (italic_synthesize(&regular, slants[si], baseline, &tilted) != 0) { clip++; continue; }
                if (tilted.h != regular.h) clip++;
                /* rows are translated, so ink mass and geometry must survive */
                for (int y = 0; y < regular.h; y++) {
                    int s0 = row_sum(&regular, y), s1 = row_sum(&tilted, y);
                    total_src += (unsigned)s0;
                    total_dst += (unsigned)s1;
                    if (s0 != s1) rows_mismatch++;
                    if (s0 == 0) continue;
                    float expect = row_centroid(&regular, y) + slants[si] * (baseline - (float)y)
                                   - (float)tilted.origin_x;
                    if (fabsf(row_centroid(&tilted, y) - expect) > 0.02f) geom_bad++;
                }
                pixels_lost += italic_pixels_present(&regular, slants[si], baseline, &tilted);
                /* the canvas exactly covers the sheared ink; the origin carries the shift */
                int left, right;
                italic_expected_bounds(&regular, slants[si], baseline, &left, &right);
                if (tilted.w != right - left) clip++;
                if (tilted.origin_x != regular.origin_x + left) clip++;
            }
        }
        check("italic_expands_not_clips", clip == 0);
        check("italic_rows_conserve_ink", rows_mismatch == 0 && total_dst == total_src);
        check("italic_no_pixel_lost", pixels_lost == 0);
        check("italic_shear_geometry", geom_bad == 0);
    }

    /* zero slant copies, out-of-range slant is rejected */
    {
        GlyphBitmap regular, same, tilted;
        test_render(1, 1.5f, &regular);
        int ok = italic_synthesize(&regular, 0.0f, 47.0f, &same) == 0 &&
                 same.w == regular.w && same.h == regular.h &&
                 memcmp(same.a, regular.a, sizeof(regular.a)) == 0;
        check("italic_zero_is_copy", ok);
        check("italic_rejects_slant", italic_synthesize(&regular, 1.5f, 47.0f, &tilted) == -1);
    }

    /* ---- stacked bold + italic ---- */
    {
        GlyphBitmap regular, bold_only, combo;
        test_render(2, 1.5f, &regular);
        GlyphStyle bs = { 1.0f, 0.0f, 0.0f };
        GlyphStyle both = { 1.0f, 0.3f, (float)(regular.h - 1) };
        int ok = glyph_apply_style(&regular, &bs, &bold_only) == 0 &&
                 glyph_apply_style(&regular, &both, &combo) == 0;
        check("stack_applies", ok);

        /* the shear only translates rows, so every horizontal ink run keeps its width */
        int run_mismatch = 0, rows = 0;
        for (int y = 0; y < bold_only.h; y++) {
            float rb[16], rc[16];
            int nb = row_runs(&bold_only, y, rb, 16);
            int nc = row_runs(&combo, y, rc, 16);
            if (nb != nc) { run_mismatch++; continue; }
            for (int i = 0; i < nb; i++) {
                if (fabsf(rb[i] - rc[i]) > 0.02f) run_mismatch++;
            }
            rows += nb;
        }
        check("stack_keeps_stroke_widths", run_mismatch == 0 && rows > 50);
        {
            /* the slant really is applied on top of the bold bitmap */
            int bad = 0;
            for (int y = 0; y < bold_only.h; y++) {
                if (row_sum(&bold_only, y) == 0) continue;
                float cb = row_centroid(&bold_only, y);
                float cc = row_centroid(&combo, y);
                float expect = cb + both.slant * (both.baseline - (float)y)
                               + (float)bold_only.origin_x - (float)combo.origin_x;
                if (fabsf(cc - expect) > 0.02f) bad++;
            }
            check("stack_shear_geometry", bad == 0);
        }
        {
            int left, right;
            italic_expected_bounds(&bold_only, both.slant, both.baseline, &left, &right);
            check("stack_canvas_matches_bounds",
                  combo.w == right - left && combo.origin_x == bold_only.origin_x + left);
        }
        check("stack_no_clipping", italic_pixels_present(&bold_only, both.slant, both.baseline, &combo) == 0);

        /* styled glyphs pack into the atlas at their new extents */
        Atlas styled;
        atlas_init(&styled);
        int packed = 0;
        for (int gi = 0; gi < 7; gi++) {
            GlyphBitmap g0, g1;
            test_render(gi, 1.5f, &g0);
            GlyphStyle st = { 1.0f, 0.25f, (float)(g0.h - 1) };
            if (glyph_apply_style(&g0, &st, &g1) == 0 &&
                atlas_add(&styled, (GlyphSize){g1.w, g1.h}, NULL) == 0) packed++;
        }
        check("styled_glyphs_pack", packed >= 5);
        check("styled_atlas_no_overlap", atlas_overlap(&styled) == 0);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
