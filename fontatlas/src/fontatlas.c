
/* fontatlas: four-stage glyph pipeline.
   stage 1: SDF generation from cell-pattern outlines (exact signed distance)
   stage 2: shelf packing of glyph SDF bitmaps into a fixed atlas
   stage 3: fixed-point (1/64 px) text layout with kerning
   stage 4: subpixel (quarter-pixel) positioning of glyph origins
   Each stage is validated before the next consumes its output;
   per-stage timings and intermediate sizes are reported at the end. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#define ATLAS_W 256
#define ATLAS_H 256
#define MAX_GLYPHS 64
#define MAX_KERN 32

/* pipeline tuning */
#define GRID_W 5     /* glyph pattern columns */
#define GRID_H 7     /* glyph pattern rows */
#define CELL 8       /* SDF pixels per font cell */
#define PAD 4        /* SDF spread = atlas padding, SDF pixels */
#define GAP 1        /* trailing advance gap, cells */
#define MAG 2        /* magnification factor for the edge-error check */
#define SPACE_ADV 3  /* space advance, cells */

#define MAX_SEGS (GRID_W * GRID_H * 4)
#define MAX_BMP_W (GRID_W * CELL + 2 * PAD)
#define MAX_BMP_H (GRID_H * CELL + 2 * PAD)
#define MAX_RUN 64

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

/* ---------------- glyph outlines: 5x7 cell patterns ---------------- */

typedef struct { char ch; const char* row[GRID_H]; } GlyphDef;

static const GlyphDef DEFS[] = {
    { 'A', { ".###.", "#...#", "#...#", "#####", "#...#", "#...#", "#...#" } },
    { 'B', { "####.", "#...#", "#...#", "####.", "#...#", "#...#", "####." } },
    { 'T', { "#####", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.." } },
    { 'V', { "#...#", "#...#", "#...#", "#...#", "#...#", ".#.#.", "..#.." } },
    { 'o', { ".###.", "#...#", "#...#", "#...#", "#...#", "#...#", ".###." } },
    { 'l', { "..#..", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.." } },
    { ' ', { ".....", ".....", ".....", ".....", ".....", ".....", "....." } },
};
#define NDEFS (sizeof(DEFS) / sizeof(DEFS[0]))

static const GlyphDef* def_for(char ch) {
    for (size_t i = 0; i < NDEFS; i++)
        if (DEFS[i].ch == ch) return &DEFS[i];
    return NULL;
}

static int def_filled(const GlyphDef* d, int r, int c) {
    if (r < 0 || r >= GRID_H || c < 0 || c >= GRID_W) return 0;
    return d->row[r][c] == '#';
}

/* ---------------- stage 1: SDF generation ---------------- */

typedef struct { float x0, y0, x1, y1; } Seg;

/* boundary segments of the cell union: an edge is boundary iff the
   cell across it is empty, so the segment set is exactly the outline */
static int def_segments(const GlyphDef* d, Seg* out) {
    int n = 0;
    for (int r = 0; r < GRID_H; r++)
        for (int c = 0; c < GRID_W; c++) {
            if (!def_filled(d, r, c)) continue;
            float x0 = c * CELL, y0 = r * CELL, x1 = x0 + CELL, y1 = y0 + CELL;
            if (!def_filled(d, r - 1, c)) out[n++] = (Seg){ x0, y0, x1, y0 };
            if (!def_filled(d, r + 1, c)) out[n++] = (Seg){ x0, y1, x1, y1 };
            if (!def_filled(d, r, c - 1)) out[n++] = (Seg){ x0, y0, x0, y1 };
            if (!def_filled(d, r, c + 1)) out[n++] = (Seg){ x1, y0, x1, y1 };
        }
    return n;
}

static float seg_dist(const Seg* s, float x, float y) {
    float dx = s->x1 - s->x0, dy = s->y1 - s->y0;
    float len2 = dx * dx + dy * dy;
    float t = len2 > 0.0f ? ((x - s->x0) * dx + (y - s->y0) * dy) / len2 : 0.0f;
    t = t < 0.0f ? 0.0f : t > 1.0f ? 1.0f : t;
    float px = s->x0 + t * dx - x, py = s->y0 + t * dy - y;
    return sqrtf(px * px + py * py);
}

/* exact signed distance to the outline, SDF pixels, negative inside */
static float def_sd(const GlyphDef* d, const Seg* segs, int nseg, float x, float y) {
    float best = 1e30f;
    for (int i = 0; i < nseg; i++) best = fminf(best, seg_dist(&segs[i], x, y));
    int inside = def_filled(d, (int)floorf(y / CELL), (int)floorf(x / CELL));
    return inside ? -best : best;
}

typedef struct {
    const GlyphDef* def;
    Seg segs[MAX_SEGS];
    int nseg;
    int min_col, max_col, ink_w_cells;
    int min_row, max_row, ink_h_cells;
    float advance_cells;
    int bmp_w, bmp_h;     /* 0x0 when the glyph has no ink (space) */
    float x0, y0;         /* glyph-local coords of bitmap origin, SDF px */
    long adv_q6;          /* advance at display resolution, 1/64 px */
    long bearing_q6;      /* bitmap left edge from pen, 1/64 px */
    int rect;             /* atlas rect index, -1 if not packed */
    uint8_t sdf[MAX_BMP_W * MAX_BMP_H];
} Glyph;

static void stage1_sdf(Glyph* g) {
    const GlyphDef* d = g->def;
    g->min_col = GRID_W; g->max_col = -1;
    g->min_row = GRID_H; g->max_row = -1;
    for (int r = 0; r < GRID_H; r++)
        for (int c = 0; c < GRID_W; c++)
            if (def_filled(d, r, c)) {
                if (c < g->min_col) g->min_col = c;
                if (c > g->max_col) g->max_col = c;
                if (r < g->min_row) g->min_row = r;
                if (r > g->max_row) g->max_row = r;
            }
    g->ink_w_cells = g->max_col >= 0 ? g->max_col - g->min_col + 1 : 0;
    g->ink_h_cells = g->max_row >= 0 ? g->max_row - g->min_row + 1 : 0;
    g->advance_cells = g->ink_w_cells > 0 ? (float)(g->ink_w_cells + GAP) : (float)SPACE_ADV;
    g->nseg = def_segments(d, g->segs);
    g->bmp_w = 0;
    g->bmp_h = 0;
    g->rect = -1;
    if (g->ink_w_cells > 0) {
        g->bmp_w = g->ink_w_cells * CELL + 2 * PAD;
        g->bmp_h = GRID_H * CELL + 2 * PAD;
        g->x0 = g->min_col * CELL - PAD;
        g->y0 = -(float)PAD;
        for (int j = 0; j < g->bmp_h; j++)
            for (int i = 0; i < g->bmp_w; i++) {
                float dd = def_sd(d, g->segs, g->nseg,
                                  g->x0 + i + 0.5f, g->y0 + j + 0.5f);
                float v = 0.5f + dd / (2.0f * PAD);
                v = v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v;
                g->sdf[j * g->bmp_w + i] = (uint8_t)(v * 255.0f + 0.5f);
            }
    }
    g->adv_q6 = lroundf(g->advance_cells * CELL * MAG * 64.0f);
    g->bearing_q6 = g->ink_w_cells > 0
        ? (long)(g->min_col * CELL - PAD) * MAG * 64 : 0;
}

/* bilinear SDF sample at bitmap-local coords (SDF px), returns distance */
static float sdf_sample(const Glyph* g, float fx, float fy) {
    float u = fx - 0.5f, v = fy - 0.5f;
    int x = (int)floorf(u), y = (int)floorf(v);
    float tx = u - x, ty = v - y;
    if (x < 0) x = 0;
    if (x > g->bmp_w - 2) x = g->bmp_w - 2;
    if (y < 0) y = 0;
    if (y > g->bmp_h - 2) y = g->bmp_h - 2;
    const uint8_t* b = g->sdf;
    int w = g->bmp_w;
    float s00 = b[y * w + x],     s10 = b[y * w + x + 1];
    float s01 = b[(y + 1) * w + x], s11 = b[(y + 1) * w + x + 1];
    float val = (s00 * (1 - tx) + s10 * tx) * (1 - ty)
              + (s01 * (1 - tx) + s11 * tx) * ty;
    return (val / 255.0f - 0.5f) * 2.0f * PAD;
}

/* magnify the SDF MAGx and compare against the analytic outline:
   max |d_recon - d_true| over display pixels within 1 px of the true edge;
   since |grad d| == 1 on the true field, this bounds contour displacement */
static float edge_error_px(const Glyph* g) {
    float worst = 0.0f;
    for (int j = 0; j < g->bmp_h * MAG; j++)
        for (int i = 0; i < g->bmp_w * MAG; i++) {
            float fx = (i + 0.5f) / MAG, fy = (j + 0.5f) / MAG;
            float d_true = def_sd(g->def, g->segs, g->nseg,
                                  g->x0 + fx, g->y0 + fy) * MAG;
            if (fabsf(d_true) > 1.0f) continue;
            float d_rec = sdf_sample(g, fx, fy) * MAG;
            float e = fabsf(d_rec - d_true);
            if (e > worst) worst = e;
        }
    return worst;
}

/* measured ink extent of the baked bitmap (pixels with value < 128) */
static void sdf_ink_extent(const Glyph* g, int* out_w, int* out_h) {
    int x0 = g->bmp_w, x1 = -1, y0 = g->bmp_h, y1 = -1;
    for (int j = 0; j < g->bmp_h; j++)
        for (int i = 0; i < g->bmp_w; i++)
            if (g->sdf[j * g->bmp_w + i] < 128) {
                if (i < x0) x0 = i;
                if (i > x1) x1 = i;
                if (j < y0) y0 = j;
                if (j > y1) y1 = j;
            }
    *out_w = x1 >= 0 ? x1 - x0 + 1 : 0;
    *out_h = y1 >= 0 ? y1 - y0 + 1 : 0;
}

/* ---------------- stage 3: fixed-point layout ---------------- */

typedef struct { char left, right; float delta_cells; } PipeKern;
static const PipeKern PKERN[] = { { 'A', 'V', -0.5f }, { 'T', 'o', -0.25f } };
#define NPKERN (sizeof(PKERN) / sizeof(PKERN[0]))

typedef struct {
    int glyph;            /* index into the glyph set, -1 = unmapped */
    long x_q6;            /* pen position before this glyph, 1/64 px */
    long kern_q6;
    long adv_q6;
    long origin_q6;       /* stage 4 input: bitmap origin */
    long snapped_q6;      /* stage 4 output: origin snapped to 1/4 px */
} RunGlyph;

static long pkern_q6(char left, char right) {
    for (size_t i = 0; i < NPKERN; i++)
        if (PKERN[i].left == left && PKERN[i].right == right)
            return lroundf(PKERN[i].delta_cells * CELL * MAG * 64.0f);
    return 0;
}

static long stage3_layout(const char* text, const Glyph* gs, int ng,
                          const int* char_map, RunGlyph* run) {
    long pen = 0;
    int n = 0;
    for (const char* p = text; *p && n < MAX_RUN; p++, n++) {
        RunGlyph* rg = &run[n];
        rg->glyph = char_map[(unsigned char)*p];
        rg->x_q6 = pen;
        rg->kern_q6 = p[1] ? pkern_q6(p[0], p[1]) : 0;
        rg->adv_q6 = rg->glyph >= 0 ? gs[rg->glyph].adv_q6 : 0;
        rg->origin_q6 = 0;
        rg->snapped_q6 = 0;
        pen += rg->adv_q6 + rg->kern_q6;
    }
    (void)ng;
    return pen;
}

/* ---------------- stage 4: subpixel positioning ---------------- */

static long snap_quarter(long q6) {
    /* round to nearest multiple of 16 q6 (= 1/4 px), ties away from zero */
    if (q6 >= 0) return ((q6 + 8) / 16) * 16;
    return -(((-q6 + 8) / 16) * 16);
}

static void stage4_snap(const Glyph* gs, RunGlyph* run, int n) {
    for (int i = 0; i < n; i++) {
        if (run[i].glyph < 0 || gs[run[i].glyph].rect < 0) continue;
        run[i].origin_q6 = run[i].x_q6 + gs[run[i].glyph].bearing_q6;
        run[i].snapped_q6 = snap_quarter(run[i].origin_q6);
    }
}

/* ---------------- self-checks ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

int main(void) {
    /* --- legacy packing/kerning unit checks --- */
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

    /* --- the four-stage pipeline --- */
    static Glyph gs[NDEFS];
    static RunGlyph run[MAX_RUN];
    const char* text = "AV Toll";
    int char_map[256];
    for (int i = 0; i < 256; i++) char_map[i] = -1;
    int ng = 0;
    for (const char* p = text; *p; p++) {
        if (char_map[(unsigned char)*p] >= 0) continue;
        const GlyphDef* d = def_for(*p);
        if (!d) continue;
        char_map[(unsigned char)*p] = ng;
        gs[ng].def = d;
        ng++;
    }

    /* stage 1: SDF generation */
    double t0 = now_ms();
    long bytes1 = 0;
    for (int i = 0; i < ng; i++) {
        stage1_sdf(&gs[i]);
        bytes1 += (long)gs[i].bmp_w * gs[i].bmp_h;
    }
    double t1 = now_ms();

    /* V1: bitmap dims match metrics, baked ink matches outline ink,
           magnified edge error below half a display pixel */
    int v1_dims = 1, v1_ink = 1;
    float edge_max = 0.0f;
    for (int i = 0; i < ng; i++) {
        const Glyph* g = &gs[i];
        if (g->ink_w_cells == 0) continue;
        if (g->bmp_w != g->ink_w_cells * CELL + 2 * PAD ||
            g->bmp_h != GRID_H * CELL + 2 * PAD) v1_dims = 0;
        int iw, ih;
        sdf_ink_extent(g, &iw, &ih);
        if (iw != g->ink_w_cells * CELL || ih != g->ink_h_cells * CELL) v1_ink = 0;
        float e = edge_error_px(g);
        if (e > edge_max) edge_max = e;
    }
    check("v1_dims_match_metrics", v1_dims);
    check("v1_ink_matches_outline", v1_ink);
    check("v1_edge_err_lt_half_px", edge_max < 0.5f);
    printf("v1_edge_err_max_px=%.3f\n", edge_max);
    double t1v = now_ms();

    /* stage 2: atlas packing */
    Atlas pack;
    atlas_init(&pack);
    for (int i = 0; i < ng; i++)
        if (gs[i].bmp_w > 0)
            atlas_add(&pack, (GlyphSize){gs[i].bmp_w, gs[i].bmp_h}, &gs[i].rect);
    double t2 = now_ms();
    long bytes2 = (long)ATLAS_W * ATLAS_H + (long)pack.count * (long)sizeof(GlyphRect);

    /* V2: every inked glyph placed exactly once, no overlap, in bounds */
    int v2_placed = 1, v2_bounds = 1, inked = 0;
    for (int i = 0; i < ng; i++) {
        if (gs[i].bmp_w == 0) continue;
        inked++;
        if (gs[i].rect < 0) v2_placed = 0;
        else {
            const GlyphRect* r = &pack.rects[gs[i].rect];
            if (r->x + r->w > ATLAS_W || r->y + r->h > ATLAS_H) v2_bounds = 0;
        }
    }
    check("v2_all_placed", v2_placed && pack.count == inked);
    check("v2_no_overlap", atlas_overlap(&pack) == 0);
    check("v2_in_bounds", v2_bounds);
    double t2v = now_ms();

    /* stage 3: layout */
    long total_q6 = stage3_layout(text, gs, ng, char_map, run);
    int nrun = (int)strlen(text);
    double t3 = now_ms();
    long bytes3 = (long)nrun * (long)sizeof(RunGlyph);

    /* V3: per-glyph layout advance identical to the bitmap-derived advance */
    int v3_mapped = 1, v3_adv = 1, v3_bitmap = 1, v3_mono = 1;
    long sum_q6 = 0;
    for (int i = 0; i < nrun; i++) {
        RunGlyph* rg = &run[i];
        if (rg->glyph < 0) { v3_mapped = 0; continue; }
        const Glyph* g = &gs[rg->glyph];
        if (rg->adv_q6 != g->adv_q6) v3_adv = 0;
        if (g->rect >= 0) {
            long from_bitmap = (long)(pack.rects[g->rect].w - 2 * PAD + GAP * CELL) * MAG * 64;
            if (from_bitmap != rg->adv_q6) v3_bitmap = 0;
        }
        if (i > 0 && rg->x_q6 < run[i - 1].x_q6) v3_mono = 0;
        sum_q6 += rg->adv_q6 + rg->kern_q6;
    }
    check("v3_all_mapped", v3_mapped);
    check("v3_adv_per_glyph", v3_adv);
    check("v3_adv_matches_bitmap", v3_bitmap);
    check("v3_total_width", sum_q6 == total_q6);
    check("v3_monotonic_pen", v3_mono);
    double t3v = now_ms();

    /* stage 4: subpixel positioning */
    stage4_snap(gs, run, nrun);
    double t4 = now_ms();
    long bytes4 = (long)nrun * (long)sizeof(long);

    /* V4: snap error within half a quarter-pixel step, order preserved */
    int v4_bound = 1, v4_order = 1;
    long prev = -1;
    for (int i = 0; i < nrun; i++) {
        if (run[i].glyph < 0 || gs[run[i].glyph].rect < 0) continue;
        long err = run[i].snapped_q6 - run[i].origin_q6;
        if (err < 0) err = -err;
        if (err > 8) v4_bound = 0;   /* 8/64 px = 1/8 px = half a 1/4-px step */
        if (prev >= 0 && run[i].snapped_q6 < prev) v4_order = 0;
        prev = run[i].snapped_q6;
    }
    check("v4_snap_within_half_step", v4_bound);
    check("v4_order_preserved", v4_order);
    double t4v = now_ms();

    /* per-stage timing (compute only) and intermediate product sizes */
    printf("stage1_sdf time_ms=%.3f bytes=%ld glyphs=%d\n", t1 - t0, bytes1, ng);
    printf("stage2_pack time_ms=%.3f bytes=%ld rects=%d\n", t2 - t1v, bytes2, pack.count);
    printf("stage3_layout time_ms=%.3f bytes=%ld run=%d\n", t3 - t2v, bytes3, nrun);
    printf("stage4_subpixel time_ms=%.3f bytes=%ld\n", t4 - t3v, bytes4);
    printf("validate time_ms=%.3f\n",
           (t1v - t1) + (t2v - t2) + (t3v - t3) + (t4v - t4));
    printf("layout_width_px=%.2f\n", total_q6 / 64.0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
