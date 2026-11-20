
/* fontatlas: shelf packing of glyph bitmaps into a fixed atlas, plus kerning
 * and baseline-driven pixel snapping with small-size stroke hinting. */
#include <stdio.h>
#include <string.h>
#include <math.h>

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

/* ------------------------------------------------------------------ */
/* pixel snapping: baseline grid snap, small-size horizontal stroke   */
/* hinting, bounded offsets, and a width-change guard.                */
/*                                                                    */
/* Coordinate convention: y grows downward. A glyph bitmap of h rows  */
/* occupies [base_y - h, base_y); base_y is the baseline.            */

#define SNAP_MAX_EDGES 8
#define RASTER_MAX 32

typedef struct {
    float x, y;          /* fractional pen position / baseline (y = baseline) */
    float w, h;          /* fractional ink-rect size relative to the pen      */
    int   n_hedges;      /* horizontal stroke edges (y of stroke tops,        */
    float hedges[SNAP_MAX_EDGES]; /* relative to the baseline), small sizes   */
} GlyphLayout;

typedef struct {
    float max_offset;    /* hard cap on every adjustment, in pixels          */
    float small_px;      /* sizes at or below this get stroke hinting        */
    float max_width_delta; /* allowed width change, in whole pixel cells     */
} SnapConfig;

typedef enum {
    SNAP_OK = 0,
    SNAP_E_PARAM = 1,
    SNAP_E_CAP = 2,       /* nearest grid snap exceeds the offset cap        */
    SNAP_E_WIDTH = 3      /* forced snapping would change width > one cell   */
} SnapCode;

typedef struct {
    int   code;
    float dx, dy;         /* integer pixel-grid adjustment of the pen        */
    float hy;             /* sub-pixel outline hint applied at small sizes   */
    int   hinted;
    float rx, ry, rw, rh; /* final fractional ink rect fed to the rasterizer */
    int   width_cells;    /* integer width after snapping                    */
    float width_delta;    /* |forced cell span - snapped width|              */
} SnapResult;

static SnapConfig snap_config_default(void) {
    SnapConfig c = { 0.5f, 12.0f, 1.0f };
    return c;
}

/* distance of an edge to its nearest integer pixel boundary */
static float edge_frac(float v) {
    float f = v - floorf(v);
    if (f > 0.5f) f = 1.0f - f;
    return f;
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static int feq(float a, float b) {
    float d = a - b; if (d < 0.f) d = -d;
    return d < 1e-5f;
}

/*
 * Snap one glyph.
 *
 * The pen is rounded to the pixel grid on both axes; rounding is rejected
 * (SNAP_E_CAP) instead of clamped when it would move farther than
 * cfg.max_offset, since clamping would leave the glyph on a half pixel and
 * defeat the purpose.
 *
 * At small sizes the outline may additionally shift by a sub-pixel amount hy
 * so that horizontal stroke edges land on pixel boundaries. An integer cell
 * shift alone cannot do this: it translates every edge by the same whole
 * number of pixels and leaves their fractional parts untouched. hy is chosen
 * from candidates that put an edge exactly on a boundary, scored by total
 * edge distance, then clamped to the same offset cap.
 *
 * Width guard: blindly covering every pixel cell the fractional ink rect
 * touches can grow the glyph by nearly two cells. If that forced span differs
 * from the snapped integer width by more than cfg.max_width_delta (normally
 * one cell), SNAP_E_WIDTH is returned rather than forcing the snap.
 */
static SnapResult glyph_snap(const GlyphLayout* g, const SnapConfig* cfg) {
    SnapResult r;
    memset(&r, 0, sizeof(r));
    if (!g || !cfg || g->w <= 0.f || g->h <= 0.f ||
        g->n_hedges < 0 || g->n_hedges > SNAP_MAX_EDGES ||
        cfg->max_offset < 0.f || cfg->max_width_delta < 0.f) {
        r.code = SNAP_E_PARAM;
        return r;
    }

    float sx = roundf(g->x);
    float sy = roundf(g->y);
    r.dx = sx - g->x;
    r.dy = sy - g->y;
    if (fabsf(r.dx) > cfg->max_offset || fabsf(r.dy) > cfg->max_offset) {
        r.code = SNAP_E_CAP;
        return r;
    }

    float left   = g->x + r.dx;
    float right  = left + g->w;
    float top    = g->y + r.dy - g->h;   /* y grows downward */

    /* forced span: every pixel cell the fractional rect touches */
    int forced_cells = (int)floorf(right) - (int)floorf(left);
    r.width_cells = (int)lroundf(g->w);
    r.width_delta = (float)forced_cells - (float)r.width_cells;
    if (r.width_delta < 0.f) r.width_delta = -r.width_delta;
    if (r.width_delta > cfg->max_width_delta) {
        r.code = SNAP_E_WIDTH;
        return r;
    }

    r.hy = 0.f;
    r.hinted = 0;
    if (g->h <= cfg->small_px && g->n_hedges > 0) {
        /* candidate outline shifts: each puts one edge on a grid line */
        float best = 0.f, best_score = 0.f, have = 0.f;
        for (int i = 0; i < g->n_hedges; i++) {
            float edge_abs = g->y + r.dy + g->hedges[i];
            float cand = -edge_frac(edge_abs);          /* shift toward floor */
            if (edge_abs - floorf(edge_abs) > 0.5f) cand = -cand;
            cand = clampf(cand, -cfg->max_offset, cfg->max_offset);
            float score = 0.f;
            for (int j = 0; j < g->n_hedges; j++)
                score += edge_frac(g->y + r.dy + g->hedges[j] + cand);
            if (!have || score < best_score) {
                have = 1.f; best_score = score; best = cand;
            }
        }
        r.hy = best;
        r.hinted = 1;
    }

    /* positive hy moves the outline down; top and bottom move together */
    r.rx = left;
    r.ry = top + r.hy;
    r.rw = g->w;
    r.rh = g->h;
    r.code = SNAP_OK;
    return r;
}

/*
 * Rasterize one axis-aligned fractional rect into an 8-bit coverage buffer
 * by exact rectangle/cell overlap area. A rect on integer boundaries yields
 * only 0/255 coverage (hard edges); a half-pixel rect yields 128-wide
 * fringes (blurry edges).
 */
static void raster_rect(unsigned char* buf, int bw, int bh,
                        float x, float y, float w, float h) {
    memset(buf, 0, (size_t)bw * bh);
    float x0 = x, x1 = x + w, y0 = y, y1 = y + h;
    int cx0 = (int)floorf(x0), cx1 = (int)floorf(x1);
    int cy0 = (int)floorf(y0), cy1 = (int)floorf(y1);
    for (int cy = cy0; cy <= cy1; cy++) {
        for (int cx = cx0; cx <= cx1; cx++) {
            if (cx < 0 || cy < 0 || cx >= bw || cy >= bh) continue;
            float ox = fminf(x1, (float)(cx + 1)) - fmaxf(x0, (float)cx);
            float oy = fminf(y1, (float)(cy + 1)) - fmaxf(y0, (float)cy);
            float cover = fmaxf(0.f, ox) * fmaxf(0.f, oy);
            int v = (int)lroundf(cover * 255.f);
            if (v > 255) v = 255;
            buf[cy * bw + cx] = (unsigned char)v;
        }
    }
}

/*
 * Edge sharpness of a coverage buffer:
 *  mean_grad   - mean |gradient| across the four-connected interior
 *                (reference only: an L1 gradient is conserved, so spreading
 *                 one hard 255 jump into two half-strength jumps leaves this
 *                 unchanged)
 *  hard_ratio  - fraction of neighboring pairs differing by >= 200/255,
 *                i.e. fully on/off edges with no anti-alias fringe. This is
 *                the discriminating measure: grid-aligned glyphs keep every
 *                edge on one boundary, half-pixel glyphs have none.
 */
static void edge_sharpness(const unsigned char* buf, int bw, int bh,
                           double* mean_grad, double* hard_ratio) {
    long sum = 0, pairs = 0, hard = 0;
    for (int y = 0; y < bh; y++) {
        for (int x = 0; x < bw; x++) {
            int v = buf[y * bw + x];
            if (x + 1 < bw) {
                int d = buf[y * bw + x + 1] - v; if (d < 0) d = -d;
                sum += d; pairs++; if (d >= 200) hard++;
            }
            if (y + 1 < bh) {
                int d = buf[(y + 1) * bw + x] - v; if (d < 0) d = -d;
                sum += d; pairs++; if (d >= 200) hard++;
            }
        }
    }
    *mean_grad = pairs ? (double)sum / pairs : 0.0;
    *hard_ratio = pairs ? (double)hard / pairs : 0.0;
}

static int buffers_equal(const unsigned char* a, const unsigned char* b, int n) {
    for (int i = 0; i < n; i++) if (a[i] != b[i]) return 0;
    return 1;
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

    /* ---- pixel snapping -------------------------------------------- */
    SnapConfig cfg = snap_config_default();
    SnapResult s;

    /* integer pen: no movement at all */
    GlyphLayout g0 = { 10.0f, 20.0f, 5.0f, 8.0f, 0, {0} };
    s = glyph_snap(&g0, &cfg);
    check("snap_integer_noop", s.code == SNAP_OK &&
          feq(s.dx, 0.f) && feq(s.dy, 0.f) && feq(s.rx, 10.f) && feq(s.ry, 12.f));

    /* half-pixel pen rounds to the grid, within the default 0.5 cap */
    GlyphLayout g1 = { 10.5f, 20.5f, 5.0f, 8.0f, 0, {0} };
    s = glyph_snap(&g1, &cfg);
    check("snap_half_pen", s.code == SNAP_OK &&
          feq(s.dx, 0.5f) && feq(s.dy, 0.5f) &&
          feq(s.rx, 11.f) && feq(s.ry, 13.f) && s.width_delta == 0.f);

    /* offset cap: a tighter cap rejects the snap instead of clamping */
    SnapConfig tight = { 0.25f, 12.0f, 1.0f };
    s = glyph_snap(&g1, &tight);
    check("snap_cap_rejects", s.code == SNAP_E_CAP);

    /* small size: sub-pixel hint pulls horizontal edges onto grid lines.
     * three off-grid edges (fracs .4/.4/.1) make one candidate a strict
     * winner; with two edges the two rounding directions always tie.      */
    GlyphLayout g2 = { 10.5f, 20.5f, 4.0f, 8.0f, 3, { -7.6f, -3.6f, -1.1f } };
    s = glyph_snap(&g2, &cfg);
    check("snap_hint_small", s.code == SNAP_OK && s.hinted == 1 &&
          feq(s.hy, -0.4f) && fabsf(s.hy) <= cfg.max_offset);

    /* hint amount itself is clamped to the cap */
    GlyphLayout g3 = { 10.0f, 20.0f, 3.0f, 8.0f, 1, { -7.5f } };
    s = glyph_snap(&g3, &tight);
    check("snap_hint_clamped", s.code == SNAP_OK && s.hinted == 1 &&
          feq(s.hy, -0.25f));

    /* large glyph: no outline hinting, only the integer grid snap */
    GlyphLayout g4 = { 10.5f, 20.5f, 6.0f, 16.0f, 2, { -12.0f, -4.0f } };
    s = glyph_snap(&g4, &cfg);
    check("snap_large_no_hint", s.code == SNAP_OK &&
          s.hinted == 0 && feq(s.hy, 0.f));

    /* width guard: 4.8-wide ink spans only 4 cells from an integer pen but
     * rounds to a 5-cell advance. Allowed at one-cell tolerance, rejected
     * at zero tolerance -- never force-snapped.                            */
    GlyphLayout g5 = { 10.3f, 20.0f, 4.8f, 8.0f, 0, {0} };
    s = glyph_snap(&g5, &cfg);
    check("snap_width_one_cell_ok", s.code == SNAP_OK &&
          s.width_cells == 5 && feq(s.width_delta, 1.f));
    SnapConfig strict_w = { 0.5f, 12.0f, 0.0f };
    s = glyph_snap(&g5, &strict_w);
    check("snap_width_guard_errors", s.code == SNAP_E_WIDTH);

    /* bad layout parameters are rejected */
    GlyphLayout gbad = { 1.0f, 2.0f, 0.0f, 8.0f, 0, {0} };
    check("snap_reject_param", glyph_snap(&gbad, &cfg).code == SNAP_E_PARAM);

    /* pixel-by-pixel cross-check against manually chosen integer coords,
     * plus edge-sharpness numbers for blurry vs snapped rendering.        */
    unsigned char blurry[RASTER_MAX * RASTER_MAX];
    unsigned char snapped[RASTER_MAX * RASTER_MAX];
    unsigned char manual[RASTER_MAX * RASTER_MAX];
    raster_rect(blurry, RASTER_MAX, RASTER_MAX, 10.5f, 14.5f, 4.0f, 6.0f);
    GlyphLayout g6 = { 10.5f, 20.5f, 4.0f, 6.0f, 0, {0} };
    s = glyph_snap(&g6, &cfg);
    raster_rect(snapped, RASTER_MAX, RASTER_MAX, s.rx, s.ry, s.rw, s.rh);
    raster_rect(manual, RASTER_MAX, RASTER_MAX, 11.0f, 15.0f, 4.0f, 6.0f);
    check("snap_matches_manual_px",
          buffers_equal(snapped, manual, RASTER_MAX * RASTER_MAX));
    check("unsnapped_differs_px",
          !buffers_equal(blurry, manual, RASTER_MAX * RASTER_MAX));

    double bm_grad, bm_hard, sn_grad, sn_hard;
    edge_sharpness(blurry, RASTER_MAX, RASTER_MAX, &bm_grad, &bm_hard);
    edge_sharpness(snapped, RASTER_MAX, RASTER_MAX, &sn_grad, &sn_hard);
    printf("sharpness_blurry  mean_grad=%.2f hard_ratio=%.4f\n", bm_grad, bm_hard);
    printf("sharpness_snapped mean_grad=%.2f hard_ratio=%.4f\n", sn_grad, sn_hard);
    /* hard_ratio is the sharp/soft discriminator: the half-pixel glyph
     * spreads every edge across two cells, so it has zero hard pairs;  */
    check("snap_sharper_edges", bm_hard == 0.0 && sn_hard > 0.0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
