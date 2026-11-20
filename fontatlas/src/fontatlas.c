
/* fontatlas: shelf packing of glyph bitmaps into a fixed atlas, plus kerning. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* ---- coverage mip pyramid ----------------------------------------------
   Glyph coverage is stored once at full resolution; smaller levels are
   built by successive 2x2 box filtering of the previous level. Sampling
   picks a continuous LOD from the texel footprint and blends the two
   adjacent levels, so a level boundary never shows up as a seam. */

#define GLYPH_SIZE 64
#define GLYPH_MAX_LEVELS 8
#define ANISO_MAX_TAPS 8
#define REF_SS 16          /* supersampling of the analytic reference */
#define SRC_SS 8           /* supersampling used to rasterize the one bitmap */

typedef struct { int w, h; unsigned char px[GLYPH_SIZE * GLYPH_SIZE]; } MipLevel;
typedef struct { MipLevel lv[GLYPH_MAX_LEVELS]; int count; } MipChain;

typedef struct {
    float cx, cy, r, half_t;        /* ring */
    float x0, y0, x1, y1, half_w;   /* diagonal bar */
} GlyphShape;

typedef struct { float ox, oy, dux, duy, dvx, dvy; int nx, ny; } Mapping;
typedef struct { float dux, duy, dvx, dvy; } Footprint;

static void glyph_default(GlyphShape* g) {
    g->cx = 0.5f; g->cy = 0.46f; g->r = 0.30f; g->half_t = 0.05f;
    g->x0 = 0.22f; g->y0 = 0.80f; g->x1 = 0.78f; g->y1 = 0.18f; g->half_w = 0.045f;
}

static float glyph_sdf(const GlyphShape* g, float x, float y) {
    float dx = x - g->cx, dy = y - g->cy;
    float d_ring = fabsf(sqrtf(dx * dx + dy * dy) - g->r) - g->half_t;

    float ex = g->x1 - g->x0, ey = g->y1 - g->y0;
    float t = ((x - g->x0) * ex + (y - g->y0) * ey) / (ex * ex + ey * ey);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float px = g->x0 + t * ex, py = g->y0 + t * ey;
    float bx = x - px, by = y - py;
    float d_bar = sqrtf(bx * bx + by * by) - g->half_w;
    return d_ring < d_bar ? d_ring : d_bar;
}

/* average coverage of one target pixel described by its uv origin and
   the uv vectors spanned by the pixel in each axis */
static float glyph_cover(const GlyphShape* g, float u, float v,
                         float ux, float uy, float vx, float vy, int ss) {
    int inside = 0;
    for (int j = 0; j < ss; j++) {
        for (int i = 0; i < ss; i++) {
            float uu = u + ux * ((float)i + 0.5f) / (float)ss + vx * ((float)j + 0.5f) / (float)ss;
            float vv = v + uy * ((float)i + 0.5f) / (float)ss + vy * ((float)j + 0.5f) / (float)ss;
            if (glyph_sdf(g, uu, vv) < 0.0f) inside++;
        }
    }
    return (float)inside / (float)(ss * ss);
}

static void mip_build(const GlyphShape* g, MipChain* c) {
    memset(c, 0, sizeof(*c));
    MipLevel* l0 = &c->lv[0];
    l0->w = GLYPH_SIZE;
    l0->h = GLYPH_SIZE;
    for (int y = 0; y < l0->h; y++) {
        for (int x = 0; x < l0->w; x++) {
            float cov = glyph_cover(g, (float)x / GLYPH_SIZE, (float)y / GLYPH_SIZE,
                                    1.0f / GLYPH_SIZE, 0.0f, 0.0f, 1.0f / GLYPH_SIZE, SRC_SS);
            l0->px[y * l0->w + x] = (unsigned char)(cov * 255.0f + 0.5f);
        }
    }
    c->count = 1;

    /* each level is derived from the previous one, halving both axes */
    while (c->count < GLYPH_MAX_LEVELS) {
        const MipLevel* s = &c->lv[c->count - 1];
        if (s->w == 1 && s->h == 1) break;
        MipLevel* d = &c->lv[c->count];
        d->w = (s->w + 1) / 2;
        d->h = (s->h + 1) / 2;
        for (int y = 0; y < d->h; y++) {
            for (int x = 0; x < d->w; x++) {
                int sum = 0, n = 0;
                for (int oy = 0; oy < 2; oy++) {
                    for (int ox = 0; ox < 2; ox++) {
                        int sx = 2 * x + ox; if (sx > s->w - 1) sx = s->w - 1;
                        int sy = 2 * y + oy; if (sy > s->h - 1) sy = s->h - 1;
                        sum += s->px[sy * s->w + sx];
                        n++;
                    }
                }
                d->px[y * d->w + x] = (unsigned char)((sum + n / 2) / n);
            }
        }
        c->count++;
    }
}

static size_t mip_bytes(const MipChain* c) {
    size_t bytes = 0;
    for (int i = 0; i < c->count; i++) bytes += (size_t)c->lv[i].w * (size_t)c->lv[i].h;
    return bytes;
}

static float level_fetch(const MipLevel* L, float u, float v) {
    float x = u * (float)L->w - 0.5f;
    float y = v * (float)L->h - 0.5f;
    int xi = (int)floorf(x), yi = (int)floorf(y);
    float fx = x - (float)xi, fy = y - (float)yi;
    int xa = xi, xb = xi + 1, ya = yi, yb = yi + 1;
    if (xa < 0) xa = 0;
    if (xa > L->w - 1) xa = L->w - 1;
    if (xb < 0) xb = 0;
    if (xb > L->w - 1) xb = L->w - 1;
    if (ya < 0) ya = 0;
    if (ya > L->h - 1) ya = L->h - 1;
    if (yb < 0) yb = 0;
    if (yb > L->h - 1) yb = L->h - 1;
    float top = L->px[ya * L->w + xa] * (1.0f - fx) + L->px[ya * L->w + xb] * fx;
    float bot = L->px[yb * L->w + xa] * (1.0f - fx) + L->px[yb * L->w + xb] * fx;
    return (top * (1.0f - fy) + bot * fy) / 255.0f;
}

/* continuous LOD: bilinear within each level, linear blend between the
   two levels bracketing the LOD, so level changes cannot produce a step */
static float mip_sample(const MipChain* c, float u, float v, float lod) {
    float last = (float)(c->count - 1);
    if (lod < 0.0f) lod = 0.0f;
    if (lod > last) lod = last;
    int l0 = (int)floorf(lod);
    int l1 = l0 + 1 < c->count ? l0 + 1 : c->count - 1;
    float f = lod - (float)l0;
    float a = level_fetch(&c->lv[l0], u, v);
    if (f <= 0.0f) return a;
    float b = level_fetch(&c->lv[l1], u, v);
    return a + (b - a) * f;
}

/* Isotropic path uses the major footprint axis for the LOD. The
   anisotropic path takes several taps spread along the major axis and
   derives the LOD from the minor axis, so a stretched footprint is
   filtered without the over-blur of a single major-axis LOD. */
static float aniso_sample(const MipChain* c, float u, float v, Footprint f,
                          int compensate, float* out_lod) {
    float a = f.dux, b = f.dvx, cc = f.duy, d = f.dvy;
    float S = a * a + b * b + cc * cc + d * d;
    float det = a * d - b * cc;
    float disc = S * S - 4.0f * det * det;
    if (disc < 0.0f) disc = 0.0f;
    float s1 = sqrtf(fmaxf((S + sqrtf(disc)) * 0.5f, 0.0f));
    float s2 = sqrtf(fmaxf((S - sqrtf(disc)) * 0.5f, 0.0f));
    float major = s1 * GLYPH_SIZE, minor = s2 * GLYPH_SIZE;
    if (major < 1e-4f) major = 1e-4f;
    if (minor < 1e-4f) minor = 1e-4f;

    float lod;
    int taps = 1;
    if (compensate) {
        float ratio = major / minor;
        taps = (int)ceilf(ratio);
        if (taps < 1) taps = 1;
        if (taps > ANISO_MAX_TAPS) taps = ANISO_MAX_TAPS;
        float lod_minor = log2f(minor);
        float lod_cap = log2f(major) - log2f((float)ANISO_MAX_TAPS);
        lod = lod_minor > lod_cap ? lod_minor : lod_cap;
    } else {
        lod = log2f(major);
    }
    if (lod < 0.0f) lod = 0.0f;
    if (out_lod) *out_lod = lod;

    if (taps <= 1) return mip_sample(c, u, v, lod);

    /* taps are spread in uv space, so the major-axis direction comes from
       the uv-space singular vector: a v1 = s1^2 u1. Taking it from the
       screen-space eigenvector instead only happens to match when the
       mapping is axis aligned. */
    float bxx = a * a + b * b, byy = cc * cc + d * d, bxy = a * cc + b * d;
    float ang = 0.5f * atan2f(2.0f * bxy, bxx - byy);
    float dx = cosf(ang), dy = sinf(ang);
    float acc = 0.0f;
    for (int i = 0; i < taps; i++) {
        float t = ((float)i + 0.5f) / (float)taps - 0.5f;
        float o = t * s1;   /* spread across the major footprint, in uv */
        acc += mip_sample(c, u + dx * o, v + dy * o, lod);
    }
    return acc / (float)taps;
}

static void mapping_axis(Mapping* m, int nx, int ny) {
    m->ox = 0.0f; m->oy = 0.0f;
    m->dux = 1.0f / (float)nx; m->duy = 0.0f;
    m->dvx = 0.0f; m->dvy = 1.0f / (float)ny;
    m->nx = nx; m->ny = ny;
}

/* rotated anisotropic footprint: axis vectors of unequal length that are
   turned by theta, so the tap direction cannot be read off the axes */
static void mapping_rotated(Mapping* m, int nx, int ny, float theta) {
    float c = cosf(theta), s = sinf(theta);
    m->ox = 0.0f; m->oy = 0.0f;
    m->dux = c / (float)nx;  m->duy = s / (float)nx;
    m->dvx = -s / (float)ny; m->dvy = c / (float)ny;
    m->nx = nx; m->ny = ny;
}

static Footprint mapping_footprint(const Mapping* m) {
    Footprint f = { m->dux, m->duy, m->dvx, m->dvy };
    return f;
}

static void mapping_uv(const Mapping* m, int x, int y, float* u, float* v) {
    *u = m->ox + ((float)x + 0.5f) * m->dux + ((float)y + 0.5f) * m->dvx;
    *v = m->oy + ((float)x + 0.5f) * m->duy + ((float)y + 0.5f) * m->dvy;
}

/* lower-left corner of the pixel, which is what glyph_cover expects */
static void mapping_corner(const Mapping* m, int x, int y, float* u, float* v) {
    *u = m->ox + (float)x * m->dux + (float)y * m->dvx;
    *v = m->oy + (float)x * m->duy + (float)y * m->dvy;
}

/* edge-only error of both paths against a supersampled analytic reference;
   errors are reported in 0..255 coverage units */
static void eval_errors(const MipChain* c, const GlyphShape* g, const Mapping* m,
                        float* base_rmse, float* iso_rmse, float* aniso_rmse,
                        float* base_max, float* aniso_max, int* edge_n) {
    double sb = 0.0, si = 0.0, sa = 0.0;
    float mb = 0.0f, ma = 0.0f;
    int n = 0;
    Footprint f = mapping_footprint(m);
    for (int y = 0; y < m->ny; y++) {
        for (int x = 0; x < m->nx; x++) {
            float u, v, cu, cv;
            mapping_uv(m, x, y, &u, &v);
            mapping_corner(m, x, y, &cu, &cv);
            float ref = glyph_cover(g, cu, cv, f.dux, f.duy, f.dvx, f.dvy, REF_SS);
            if (ref <= 0.02f || ref >= 0.98f) continue;
            float b = level_fetch(&c->lv[0], u, v);
            float io = aniso_sample(c, u, v, f, 0, NULL);
            float an = aniso_sample(c, u, v, f, 1, NULL);
            float db = fabsf(b - ref), dio = fabsf(io - ref), da = fabsf(an - ref);
            sb += (double)db * db;
            si += (double)dio * dio;
            sa += (double)da * da;
            if (db > mb) mb = db;
            if (da > ma) ma = da;
            n++;
        }
    }
    double scale = 255.0 / sqrt((double)(n > 0 ? n : 1));
    *base_rmse = (float)(sqrt(sb) * scale);
    *iso_rmse = (float)(sqrt(si) * scale);
    *aniso_rmse = (float)(sqrt(sa) * scale);
    *base_max = mb * 255.0f;
    *aniso_max = ma * 255.0f;
    *edge_n = n;
}

/* sweep the scale continuously across every level boundary. Returns the
   largest change of the sampled value between adjacent scales (a seam
   would show up here) and checks that LOD visits consecutive levels. */
static int seam_sweep(const MipChain* c, float* out_worst_step, float* out_lod_span,
                      int* out_levels_used) {
    enum { GRID = 24, STEPS_LIMIT = 9000 };
    float prev[GRID * GRID];
    int have_prev = 0, worst_floor_step = 0;
    int used[GLYPH_MAX_LEVELS] = {0};
    float worst = 0.0f, lod_min = 1e9f, lod_max = -1e9f;
    float prev_lod = -1.0f;
    int prev_floor = -1, monotone = 1;

    /* 1x down to 1/64x: covers every level of a 64px chain */
    for (float zoom = 1.0f, step = 0; step < STEPS_LIMIT && zoom >= 0.015f; zoom *= 0.997f, step++) {
        float p = 1.0f / zoom;             /* footprint in source texels */
        Footprint f = { p / GLYPH_SIZE, 0.0f, 0.0f, p / GLYPH_SIZE };
        float lod = 0.0f;
        int idx = 0;
        for (int y = 0; y < GRID; y++) {
            for (int x = 0; x < GRID; x++) {
                float u = ((float)x + 0.5f) / GRID;
                float v = ((float)y + 0.5f) / GRID;
                float s = aniso_sample(c, u, v, f, 1, &lod);
                if (have_prev && fabsf(s - prev[idx]) > worst) worst = fabsf(s - prev[idx]);
                prev[idx++] = s;
            }
        }
        have_prev = 1;
        if (lod < lod_min) lod_min = lod;
        if (lod > lod_max) lod_max = lod;
        int fl = (int)floorf(lod);
        if (fl < 0) fl = 0;
        if (fl > c->count - 1) fl = c->count - 1;
        used[fl] = 1;
        if (prev_floor >= 0 && abs(fl - prev_floor) > worst_floor_step)
            worst_floor_step = abs(fl - prev_floor);
        if (prev_lod >= 0.0f && lod < prev_lod - 1e-4f) monotone = 0;
        prev_lod = lod;
        prev_floor = fl;
    }

    *out_worst_step = worst;
    *out_lod_span = lod_max - lod_min;
    *out_levels_used = 0;
    for (int i = 0; i < c->count; i++) *out_levels_used += used[i];
    return worst_floor_step <= 1 && monotone;
}

/* At every level boundary compare a hard (nearest-level) switch with the
   trilinear result: the hard switch jumps, the blended one does not. */
static void boundary_seam(const MipChain* c, float* hard, float* soft) {
    enum { GRID = 48 };
    float h = 0.0f, s = 0.0f;
    for (int li = 1; li < c->count; li++) {
        for (int y = 0; y < GRID; y++) {
            for (int x = 0; x < GRID; x++) {
                float u = ((float)x + 0.5f) / GRID, v = ((float)y + 0.5f) / GRID;
                float a = level_fetch(&c->lv[li - 1], u, v);
                float b = level_fetch(&c->lv[li], u, v);
                /* keep to pixels where the two levels genuinely disagree,
                   i.e. edge pixels rather than flat interior/exterior */
                if (a <= 0.02f || b <= 0.02f || a >= 0.98f || b >= 0.98f) continue;
                if (fabsf(a - b) > h) h = fabsf(a - b);
                float se = mip_sample(c, u, v, (float)li + 1e-3f) -
                           mip_sample(c, u, v, (float)li - 1e-3f);
                if (fabsf(se) > s) s = fabsf(se);
            }
        }
    }
    *hard = h * 255.0f;
    *soft = s * 255.0f;
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

    /* ---- coverage mip pyramid ---- */
    GlyphShape glyph;
    glyph_default(&glyph);
    MipChain chain;
    mip_build(&glyph, &chain);

    check("mip_levels", chain.count == 7 && chain.lv[6].w == 1 && chain.lv[6].h == 1);
    int halving_ok = 1;
    for (int i = 1; i < chain.count; i++)
        if (chain.lv[i].w != (chain.lv[i - 1].w + 1) / 2 ||
            chain.lv[i].h != (chain.lv[i - 1].h + 1) / 2) halving_ok = 0;
    check("mip_halves", halving_ok);

    /* mean coverage must survive downsampling (a box filter is conservative) */
    float mean0 = 0.0f, max_mean_dev = 0.0f;
    for (int i = 0; i < chain.count; i++) {
        double sum = 0.0;
        const MipLevel* L = &chain.lv[i];
        for (int p = 0; p < L->w * L->h; p++) sum += L->px[p];
        float mean = (float)(sum / (double)(L->w * L->h));
        if (i == 0) { mean0 = mean; continue; }
        if (fabsf(mean - mean0) > max_mean_dev) max_mean_dev = fabsf(mean - mean0);
    }
    printf("mip_mean_dev=%.3f\n", max_mean_dev);
    check("mip_coverage_conserved", max_mean_dev < 3.0f);

    size_t base_bytes = (size_t)GLYPH_SIZE * GLYPH_SIZE;
    size_t chain_bytes = mip_bytes(&chain);
    int bytes_ok = chain_bytes > base_bytes && chain_bytes < base_bytes * 2;
    check("mip_memory_bounded", bytes_ok);
    /* payload is what a packed upload would cost; sizeof is this
       implementation's static storage, which over-reserves each level */
    printf("mem_base=%zu mem_chain_payload=%zu mem_chain_struct=%zu\n",
           base_bytes, chain_bytes, sizeof(chain));

    /* five zoom steps: 1x, 1/2, 1/3, 1/4 and a 4:1 horizontally
       stretched footprint, comparing baseline, isotropic and anisotropic */
    Mapping cases[5];
    const int sizes[5][2] = { {64, 64}, {32, 32}, {21, 21}, {16, 16}, {64, 16} };
    for (int i = 0; i < 5; i++) mapping_axis(&cases[i], sizes[i][0], sizes[i][1]);
    const char* names[5] = { "1x", "1/2", "1/3", "1/4", "aniso4" };

    float base_r[5], iso_r[5], ani_r[5], base_max[5], ani_max[5];
    int edge_n[5];
    for (int i = 0; i < 5; i++)
        eval_errors(&chain, &glyph, &cases[i], &base_r[i], &iso_r[i], &ani_r[i],
                    &base_max[i], &ani_max[i], &edge_n[i]);

    printf("scale,base_rmse,aniso_rmse,iso_rmse,base_max,aniso_max,edge_px\n");
    for (int i = 0; i < 5; i++)
        printf("%s,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n", names[i], base_r[i], ani_r[i],
               iso_r[i], base_max[i], ani_max[i], edge_n[i]);

    /* minification must not lose to the raw bitmap at any scale. At 1/2 the
       baseline bilinear tap already lands on a whole 2x2 texel block, so a
       tie there is the best possible result; below that the mip chain wins. */
    int no_loss = 1;
    for (int i = 0; i < 5; i++) if (ani_r[i] > base_r[i] + 1.0f) no_loss = 0;
    check("mip_no_worse_than_base", no_loss);
    check("mip_beats_base_at_third", ani_r[2] < base_r[2] * 0.95f);
    check("mip_beats_base_at_quarter", ani_r[3] < base_r[3] * 0.25f);
    /* the stretched footprint is where anisotropic compensation has to
       beat the isotropic major-axis choice */
    check("aniso_beats_iso_when_stretched", ani_r[4] < iso_r[4] * 0.25f);

    /* rotated & sheared 4:1 footprint: the major axis is no longer aligned
       with u, so this only passes if the tap direction is derived correctly */
    Mapping rot;
    mapping_rotated(&rot, 48, 12, 0.6f);
    float r_base, r_iso, r_ani, r_bmax, r_amax;
    int r_edges;
    eval_errors(&chain, &glyph, &rot, &r_base, &r_iso, &r_ani, &r_bmax, &r_amax, &r_edges);
    printf("rotated base=%.2f aniso=%.2f iso=%.2f edges=%d\n", r_base, r_ani, r_iso, r_edges);
    check("aniso_beats_iso_when_rotated", r_ani < r_iso * 0.5f);

    float worst_step, lod_span;
    int levels_used;
    int sweep_ok = seam_sweep(&chain, &worst_step, &lod_span, &levels_used);
    printf("seam_worst_step=%.2f lod_span=%.2f levels_used=%d\n",
           worst_step * 255.0f, lod_span, levels_used);
    check("lod_selects_all_levels", levels_used == chain.count);
    check("lod_no_level_skip", sweep_ok);
    /* sampled value must change smoothly as the scale sweeps across every
       level boundary; a hard level switch shows up as a large step here */
    check("seam_free_across_levels", worst_step * 255.0f < 2.0f);

    float hard_jump, soft_step;
    boundary_seam(&chain, &hard_jump, &soft_step);
    printf("boundary_hard_jump=%.2f boundary_trilinear_step=%.2f\n", hard_jump, soft_step);
    check("trilinear_removes_level_seam", soft_step < 2.0f && soft_step < hard_jump * 0.25f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
