
/* fontatlas: shelf packing of glyph bitmaps into a fixed atlas, plus kerning,
   plus quarter-pixel subpixel glyph positioning. */
#include <math.h>
#include <stdio.h>
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

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* ---- subpixel positioning: quarter-pixel phase cache ----
   Glyphs are resampled at PHASES subpixel x-offsets and every variant is
   cached in the atlas. At layout time the fractional pen x picks the nearest
   cached phase, so the rendered origin is quantized to 1/4 px (max error
   1/8 px) instead of a whole pixel (max error 1/2 px). */

#define PHASES 4              /* quarter-pixel steps */
#define GLYPH_W 10
#define GLYPH_H 12
#define SUB_W (GLYPH_W + 1)   /* shifted bitmap may spill one column */

typedef struct { int w, h; unsigned char px[SUB_W * GLYPH_H]; } Bitmap;

/* bilinear sample, zero outside the bitmap */
static double bmp_sample(const Bitmap* b, double x, double y) {
    int x0 = (int)floor(x), y0 = (int)floor(y);
    double fx = x - x0, fy = y - y0, s = 0.0;
    for (int j = 0; j <= 1; j++)
        for (int i = 0; i <= 1; i++) {
            int xx = x0 + i, yy = y0 + j;
            double w = (i ? fx : 1.0 - fx) * (j ? fy : 1.0 - fy);
            if (xx >= 0 && xx < b->w && yy >= 0 && yy < b->h)
                s += w * b->px[yy * b->w + xx];
        }
    return s;
}

/* resample src shifted right by dx fractional pixels */
static void bmp_shift(const Bitmap* src, double dx, Bitmap* dst) {
    dst->w = SUB_W;
    dst->h = src->h;
    for (int y = 0; y < dst->h; y++)
        for (int x = 0; x < dst->w; x++)
            dst->px[y * dst->w + x] = (unsigned char)(bmp_sample(src, x - dx, y) + 0.5);
}

static double bmp_ink(const Bitmap* b) {
    double s = 0.0;
    for (int i = 0; i < b->w * b->h; i++) s += b->px[i];
    return s;
}

static double bmp_centroid_x(const Bitmap* b) {
    double sw = 0.0, sx = 0.0;
    for (int y = 0; y < b->h; y++)
        for (int x = 0; x < b->w; x++) {
            sw += b->px[y * b->w + x];
            sx += x * b->px[y * b->w + x];
        }
    return sw > 0.0 ? sx / sw : 0.0;
}

/* per-glyph cache: one resampled bitmap per quarter-pixel phase */
typedef struct {
    Bitmap phase[PHASES];     /* phase k = glyph shifted right by k/4 px */
    int atlas_index[PHASES];
    int built;
} SubpixelGlyph;

static void subpixel_build(SubpixelGlyph* g, const Bitmap* src, Atlas* atlas) {
    memset(g, 0, sizeof(*g));
    for (int k = 0; k < PHASES; k++) {
        bmp_shift(src, k / (double)PHASES, &g->phase[k]);
        int idx = -1;
        if (atlas_add(atlas, (GlyphSize){g->phase[k].w, g->phase[k].h}, &idx) != 0) return;
        g->atlas_index[k] = idx;
    }
    g->built = 1;
}

/* nearest cached phase for a fractional x; caller adds floor(x) */
static int subpixel_pick(double x) {
    double frac = x - floor(x);
    int k = (int)(frac * PHASES + 0.5);
    return k == PHASES ? 0 : k;   /* rounded up to the next whole pixel */
}

/* snap to the nearest cached phase position: max error 1/(2*PHASES) px */
static double snap_quarter(double x) {
    int ip = (int)floor(x);
    double frac = x - ip;
    int k = (int)(frac * PHASES + 0.5);
    if (k == PHASES) { ip++; k = 0; }
    return ip + k / (double)PHASES;
}

/* classic integer-pixel positioning: max error 1/2 px */
static double snap_integer(double x) { return floor(x + 0.5); }

/* deterministic test glyph: vertical stem + diagonal bar, margins all round */
static void synth_glyph(Bitmap* b) {
    b->w = GLYPH_W;
    b->h = GLYPH_H;
    for (int y = 0; y < b->h; y++)
        for (int x = 0; x < b->w; x++) {
            int stem = (x >= 1 && x <= 2 && y >= 1 && y <= 10);
            double cx = 4.0 + 4.0 * (y - 1) / 9.0;
            int diag = (y >= 1 && y <= 10 && fabs(x - cx) <= 1.0);
            b->px[y * b->w + x] = (stem || diag) ? 255 : 0;
        }
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

    /* ---- subpixel positioning ---- */
    Bitmap glyph;
    synth_glyph(&glyph);
    Atlas sub_atlas;
    atlas_init(&sub_atlas);
    SubpixelGlyph sub;
    subpixel_build(&sub, &glyph, &sub_atlas);
    check("subpixel_built", sub.built);
    check("subpixel_cached_in_atlas", sub_atlas.count == PHASES && atlas_overlap(&sub_atlas) == 0);

    /* phase 0 is the unshifted glyph (one column wider, right column empty) */
    int phase0_same = 1;
    for (int y = 0; y < GLYPH_H && phase0_same; y++)
        for (int x = 0; x < GLYPH_W && phase0_same; x++)
            if (sub.phase[0].px[y * SUB_W + x] != glyph.px[y * GLYPH_W + x]) phase0_same = 0;
    check("phase0_matches_source", phase0_same);

    /* resampling moves the ink centroid by exactly k/4 px and conserves ink */
    double c0 = bmp_centroid_x(&sub.phase[0]);
    double ink0 = bmp_ink(&sub.phase[0]);
    int centroid_ok = 1, ink_ok = 1;
    for (int k = 1; k < PHASES; k++) {
        if (fabs(bmp_centroid_x(&sub.phase[k]) - c0 - k / (double)PHASES) > 0.02) centroid_ok = 0;
        if (fabs(bmp_ink(&sub.phase[k]) - ink0) > 0.02 * ink0) ink_ok = 0;
    }
    check("phase_centroid_shifts_quarter_px", centroid_ok);
    check("phase_ink_conserved", ink_ok);

    /* nearest-phase cache lookup */
    check("pick_nearest_phase", subpixel_pick(0.13) == 1 && subpixel_pick(0.40) == 2 &&
          subpixel_pick(0.63) == 3 && subpixel_pick(0.90) == 0 && subpixel_pick(0.0) == 0);

    /* alignment error: integer vs quarter-pixel, sweeping the fractional offset */
    double max_i = 0, max_q = 0, sum_i = 0, sum_q = 0;
    for (int i = 0; i < 16; i++) {
        double frac = i / 16.0, x = 3.0 + frac;
        double ei = fabs(snap_integer(x) - x);
        double eq = fabs(snap_quarter(x) - x);
        printf("align_err frac=%.4f int=%.4f quarter=%.4f\n", frac, ei, eq);
        if (ei > max_i) max_i = ei;
        if (eq > max_q) max_q = eq;
        sum_i += ei; sum_q += eq;
    }
    printf("align_err_max int=%.4f quarter=%.4f\n", max_i, max_q);
    printf("align_err_mean int=%.4f quarter=%.4f\n", sum_i / 16, sum_q / 16);
    check("align_err_quarter_le_eighth_px", max_q <= 0.125 + 1e-9);
    check("align_err_int_up_to_half_px", max_i >= 0.49);
    check("align_err_quarter_3x_better", sum_i > 3.0 * sum_q);

    /* slanted layout: 8 glyphs, 10 px advance on a 10-degree baseline.
       Sweep a global x offset; rendered origins must move monotonically and
       adjacent spacing must drift gradually, not jump by whole pixels. */
    const int N = 8;
    const double dx = 10.0 * cos(10.0 * 3.14159265358979323846 / 180.0);
    for (int pass = 0; pass < 2; pass++) {
        double (*snap)(double) = pass == 0 ? snap_integer : snap_quarter;
        const char* tag = pass == 0 ? "int" : "quarter";
        double dev_max = 0, step_max = 0, prev_pos[N], prev_sp[N - 1];
        int mono = 1;
        for (int s = 0; s < 16; s++) {
            double t = s / 16.0, pos[N];
            for (int g = 0; g < N; g++) pos[g] = snap(t + g * dx);
            if (s > 0)
                for (int g = 0; g < N; g++)
                    if (pos[g] < prev_pos[g] - 1e-9) mono = 0;
            for (int g = 0; g < N - 1; g++) {
                double sp = pos[g + 1] - pos[g];
                double dev = fabs(sp - dx);
                if (dev > dev_max) dev_max = dev;
                if (s > 0) {
                    double step = fabs(sp - prev_sp[g]);
                    if (step > step_max) step_max = step;
                }
                prev_sp[g] = sp;
            }
            memcpy(prev_pos, pos, sizeof(pos));
        }
        printf("spacing_%s dev_max=%.4f step_max=%.4f monotonic=%d\n",
               tag, dev_max, step_max, mono);
        if (pass == 0) {
            check("spacing_int_jumps", step_max >= 0.9 && dev_max > 0.4);
        } else {
            check("spacing_quarter_monotonic", mono);
            check("spacing_quarter_dev_le_quarter_px", dev_max <= 0.25 + 1e-9);
            check("spacing_quarter_no_jumps", step_max <= 0.25 + 1e-9);
        }
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
