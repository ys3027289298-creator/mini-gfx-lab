#define _POSIX_C_SOURCE 200809L
/* tilemap: layered tile map with 4-neighbour autotiling, per-layer culling
   and per-layer blend modes (normal, multiply, overlay, screen) with
   adjustable opacity.

   Compositing walks the layers bottom-up in place: every output pixel depends
   only on its own backdrop, so no second buffer is needed. Non-opaque layers
   are applied through a per-layer lookup table built once when the layer's
   blend mode/opacity is configured, which makes the per-pixel blend cost a
   table load plus a store. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define W 32
#define H 32
#define LAYERS 3
#define TILE 8
#define IMG_W (W * TILE)
#define IMG_H (H * TILE)
#define FRAME_BYTES (IMG_W * IMG_H * 4)

typedef enum { BLEND_NORMAL = 0, BLEND_MULTIPLY, BLEND_OVERLAY, BLEND_SCREEN, BLEND_MODE_COUNT } BlendMode;

typedef struct {
    unsigned char base[LAYERS][H][W];   /* 0 = empty, else tile id */
    int layer_visible[LAYERS];
    BlendMode layer_blend[LAYERS];      /* how this layer composites onto the result below it */
    float layer_opacity[LAYERS];        /* 0..1 */
    unsigned char colors[256][3];       /* tile id -> RGB */
} TileMap;

static void map_set_color(TileMap* m, unsigned char id, unsigned char r,
                          unsigned char g, unsigned char b) {
    m->colors[id][0] = r; m->colors[id][1] = g; m->colors[id][2] = b;
}

static void map_init(TileMap* m) {
    memset(m, 0, sizeof(*m));
    for (int l = 0; l < LAYERS; l++) {
        m->layer_visible[l] = 1;
        m->layer_blend[l] = BLEND_NORMAL;
        m->layer_opacity[l] = 1.0f;
    }
    /* varied default palette so blends always have data to work on */
    for (int id = 1; id < 256; id++) {
        m->colors[id][0] = (unsigned char)(37 * id + 11);
        m->colors[id][1] = (unsigned char)(91 * id + 53);
        m->colors[id][2] = (unsigned char)(151 * id + 97);
    }
}

static void map_set(TileMap* m, int layer, int x, int y, unsigned char id) {
    if (layer < 0 || layer >= LAYERS || x < 0 || x >= W || y < 0 || y >= H) return;
    m->base[layer][y][x] = id;
}

static unsigned char map_get(const TileMap* m, int layer, int x, int y) {
    if (layer < 0 || layer >= LAYERS || x < 0 || x >= W || y < 0 || y >= H) return 0;
    return m->base[layer][y][x];
}

/* per-layer blend configuration; invalid input leaves the layer untouched */
static int map_set_blend(TileMap* m, int layer, BlendMode mode, float opacity) {
    if (layer < 0 || layer >= LAYERS) return -1;
    if (mode < 0 || mode >= BLEND_MODE_COUNT) return -1;
    if (!(opacity >= 0.0f && opacity <= 1.0f)) return -1;   /* also rejects NaN */
    m->layer_blend[layer] = mode;
    m->layer_opacity[layer] = opacity;
    return 0;
}

/* 4-bit mask: 1 = neighbour filled (N,E,S,W) */
static int autotile_mask(const TileMap* m, int layer, int x, int y) {
    int mask = 0;
    if (map_get(m, layer, x, y - 1)) mask |= 1;
    if (map_get(m, layer, x + 1, y)) mask |= 2;
    if (map_get(m, layer, x, y + 1)) mask |= 4;
    if (map_get(m, layer, x - 1, y)) mask |= 8;
    return mask;
}

/* autotile variant id for a filled cell: mask plus a base offset */
static int autotile_variant(const TileMap* m, int layer, int x, int y) {
    if (!map_get(m, layer, x, y)) return -1;
    return autotile_mask(m, layer, x, y);
}

/* count visible tiles inside a rect across visible layers */
static int cull_count(const TileMap* m, int x0, int y0, int x1, int y1) {
    int n = 0;
    for (int l = 0; l < LAYERS; l++) {
        if (!m->layer_visible[l]) continue;
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++)
                if (map_get(m, l, x, y)) n++;
    }
    return n;
}

/* ---- blend formulas ---------------------------------------------------
   All channels are linear [0,1]. The separable blend functions follow the
   W3C compositing spec:

     multiply: B(cb, cs) = cb * cs
     screen:   B(cb, cs) = cb + cs - cb * cs
     overlay:  B(cb, cs) = hard-light(cs, cb)
                         = 2*cs*cb             if cb <= 0.5
                           1 - 2*(1-cs)*(1-cb) otherwise

   and the layer is composited source-over with opacity alpha:

     Co = (1 - alpha) * Cb + alpha * B(Cb, Cs)
     Ao = alpha + (1 - alpha) * Ab

   The framebuffer is RGBA8, so each layer pass reads 8-bit backdrop bytes
   and writes rounded 8-bit results; the formulas are evaluated per pixel
   per pass. blend_px8() is the canonical definition of one such pass and
   the lookup table built below must reproduce it byte for byte. */

static unsigned char clamp255(long v) {
    return (unsigned char)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

static float blend_channel(BlendMode mode, float cb, float cs) {
    switch (mode) {
    case BLEND_MULTIPLY: return cb * cs;
    case BLEND_SCREEN:   return cb + cs - cb * cs;
    case BLEND_OVERLAY:
        return cb <= 0.5f ? 2.0f * cs * cb : 1.0f - 2.0f * (1.0f - cs) * (1.0f - cb);
    case BLEND_NORMAL:
    default: return cs;
    }
}

static unsigned char blend_px8(BlendMode mode, unsigned char cb8,
                               unsigned char cs8, float alpha) {
    float cb = cb8 / 255.0f, cs = cs8 / 255.0f;
    float co = (1.0f - alpha) * cb + alpha * blend_channel(mode, cb, cs);
    return clamp255(lroundf(co * 255.0f));
}

static unsigned char alpha_over8(float alpha, unsigned char ab8) {
    float ab = ab8 / 255.0f;
    return clamp255(lroundf((alpha + (1.0f - alpha) * ab) * 255.0f));
}

/* ---- reference compositor: per pixel, straight from the formulas ---- */

static unsigned char* pixel_at(unsigned char* img, int x, int y) {
    return img + ((size_t)y * IMG_W + (size_t)x) * 4;
}

static void composite_ref(const TileMap* m, unsigned char* out) {
    memset(out, 0, FRAME_BYTES);
    for (int l = 0; l < LAYERS; l++) {
        if (!m->layer_visible[l]) continue;
        float alpha = m->layer_opacity[l];
        if (alpha <= 0.0f) continue;
        BlendMode mode = m->layer_blend[l];
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                unsigned char id = map_get(m, l, x, y);
                if (!id) continue;   /* transparent: backdrop shows through */
                const unsigned char* sc = m->colors[id];
                for (int ty = 0; ty < TILE; ty++) {
                    for (int tx = 0; tx < TILE; tx++) {
                        unsigned char* p = pixel_at(out, x * TILE + tx, y * TILE + ty);
                        for (int ch = 0; ch < 3; ch++)
                            p[ch] = blend_px8(mode, p[ch], sc[ch], alpha);
                        p[3] = alpha_over8(alpha, p[3]);
                    }
                }
            }
        }
    }
}

/* ---- blend lookup table -----------------------------------------------
   The blend functions are separable, so one table serves all three colour
   channels: rgb[cs][cb] is the exact blend_px8() result for a fully
   specified (source, backdrop) byte pair at the layer's opacity, and a[ab]
   is the matching alpha result. A row is contiguous so a tile's three
   colours stay hot in cache while its pixels are walked.

   The table is rebuilt only when the layer's blend configuration changes,
   i.e. at configuration time, never per frame. */

typedef struct {
    int valid;
    BlendMode mode;
    float alpha;
    unsigned char rgb[256][256];   /* [source byte][backdrop byte] */
    unsigned char a[256];          /* [backdrop alpha] */
} BlendTable;

typedef struct {
    unsigned char frame[FRAME_BYTES];
    BlendTable tables[LAYERS];
    long table_rebuilds;
    long blend_lookups;
} Compositor;

/* Exact integer form of the fully opaque pass, used to build the table so
   its contents do not depend on the host's rounding mode or float
   behaviour. Verified against blend_px8(..., 1.0f) for every input pair. */
static unsigned char blend_px8_opaque(BlendMode mode, unsigned cb, unsigned cs) {
    switch (mode) {
    case BLEND_MULTIPLY: return (unsigned char)((2u * cb * cs + 255u) / 510u);
    case BLEND_SCREEN:   return (unsigned char)(cb + cs - (2u * cb * cs + 255u) / 510u);
    case BLEND_OVERLAY:
        return cb <= 127u ? (unsigned char)((4u * cb * cs + 255u) / 510u)
                          : (unsigned char)(255u - (4u * (255u - cb) * (255u - cs) + 255u) / 510u);
    case BLEND_NORMAL:
    default: return (unsigned char)cs;
    }
}

static void blend_table_build(BlendTable* t, BlendMode mode, float alpha) {
    if (alpha >= 1.0f) {
        for (int cs = 0; cs < 256; cs++)
            for (int cb = 0; cb < 256; cb++)
                t->rgb[cs][cb] = blend_px8_opaque(mode, (unsigned)cb, (unsigned)cs);
        for (int ab = 0; ab < 256; ab++) t->a[ab] = 255;
    } else {
        for (int cs = 0; cs < 256; cs++)
            for (int cb = 0; cb < 256; cb++)
                t->rgb[cs][cb] = blend_px8(mode, (unsigned char)cb, (unsigned char)cs, alpha);
        for (int ab = 0; ab < 256; ab++)
            t->a[ab] = alpha_over8(alpha, (unsigned char)ab);
    }
    t->mode = mode;
    t->alpha = alpha;
    t->valid = 1;
}

static const BlendTable* blend_table_for(Compositor* c, int layer,
                                         BlendMode mode, float alpha) {
    BlendTable* t = &c->tables[layer];
    if (!t->valid || t->mode != mode ||
        memcmp(&t->alpha, &alpha, sizeof(float)) != 0) {
        blend_table_build(t, mode, alpha);
        c->table_rebuilds++;
    }
    return t;
}

static void comp_init(Compositor* c) {
    memset(c, 0, sizeof(*c));
}

static void fill_cell(unsigned char* out, int x, int y, const unsigned char color[3]) {
    for (int ty = 0; ty < TILE; ty++) {
        unsigned char* row = out + ((size_t)(y * TILE + ty) * IMG_W + (size_t)x * TILE) * 4;
        for (int tx = 0; tx < TILE; tx++) {
            row[tx * 4 + 0] = color[0];
            row[tx * 4 + 1] = color[1];
            row[tx * 4 + 2] = color[2];
            row[tx * 4 + 3] = 255;
        }
    }
}

static void blend_cell(const BlendTable* t, const unsigned char color[3],
                       unsigned char* out, int x, int y) {
    const unsigned char* r = t->rgb[color[0]];
    const unsigned char* g = t->rgb[color[1]];
    const unsigned char* b = t->rgb[color[2]];
    const unsigned char* a = t->a;
    for (int ty = 0; ty < TILE; ty++) {
        unsigned char* p = out + ((size_t)(y * TILE + ty) * IMG_W + (size_t)x * TILE) * 4;
        for (int tx = 0; tx < TILE; tx++, p += 4) {
            p[0] = r[p[0]];
            p[1] = g[p[1]];
            p[2] = b[p[2]];
            p[3] = a[p[3]];
        }
    }
}

static void composite_frame(Compositor* c, const TileMap* m) {
    c->blend_lookups = 0;
    memset(c->frame, 0, FRAME_BYTES);

    for (int l = 0; l < LAYERS; l++) {
        if (!m->layer_visible[l]) continue;
        float alpha = m->layer_opacity[l];
        if (alpha <= 0.0f) continue;          /* no contribution at all */
        BlendMode mode = m->layer_blend[l];
        int plain = (mode == BLEND_NORMAL && alpha >= 1.0f);
        const BlendTable* t = plain ? NULL : blend_table_for(c, l, mode, alpha);

        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                unsigned char id = map_get(m, l, x, y);
                if (!id) continue;            /* keep the backdrop untouched */
                if (plain) fill_cell(c->frame, x, y, m->colors[id]);
                else blend_cell(t, m->colors[id], c->frame, x, y);
            }
        }
        if (!plain) c->blend_lookups += (long)IMG_W * IMG_H;
    }
}

/* per-pixel additive baseline with the same traversal, used to benchmark
   the blend overhead */
static void composite_additive(const TileMap* m, unsigned char* out) {
    memset(out, 0, FRAME_BYTES);
    for (int l = 0; l < LAYERS; l++) {
        if (!m->layer_visible[l]) continue;
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                unsigned char id = map_get(m, l, x, y);
                if (!id) continue;
                const unsigned char* sc = m->colors[id];
                for (int ty = 0; ty < TILE; ty++) {
                    unsigned char* p = out + ((size_t)(y * TILE + ty) * IMG_W +
                                              (size_t)x * TILE) * 4;
                    for (int tx = 0; tx < TILE; tx++, p += 4) {
                        unsigned r = p[0] + sc[0];
                        unsigned g = p[1] + sc[1];
                        unsigned b = p[2] + sc[2];
                        p[0] = (unsigned char)(r > 255 ? 255 : r);
                        p[1] = (unsigned char)(g > 255 ? 255 : g);
                        p[2] = (unsigned char)(b > 255 ? 255 : b);
                        p[3] = 255;
                    }
                }
            }
        }
    }
}

/* ---- self tests ------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void fill_layer(TileMap* m, int l, unsigned char id) {
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            map_set(m, l, x, y, id);
}

static int frame_px_is(const unsigned char* frame, int x, int y,
                       unsigned char r, unsigned char g, unsigned char b) {
    const unsigned char* p = frame + ((size_t)y * IMG_W + (size_t)x) * 4;
    return p[0] == r && p[1] == g && p[2] == b && p[3] == 255;
}

static double now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

static volatile unsigned bench_sink;

#define BENCH_ITERS 600
#define BENCH_TRIALS 5

static double bench_blend(Compositor* c, const TileMap* m) {
    double best = 1e30;
    for (int trial = 0; trial < BENCH_TRIALS; trial++) {
        unsigned h = 0;
        double t0 = now_ns();
        for (int i = 0; i < BENCH_ITERS; i++) {
            composite_frame(c, m);
            h += c->frame[(unsigned)(i * 97u) % FRAME_BYTES];
        }
        double dt = now_ns() - t0;
        bench_sink ^= h;
        if (dt < best) best = dt;
    }
    return best / (BENCH_ITERS * (double)(IMG_W * IMG_H));
}

static double bench_additive(const TileMap* m, unsigned char* frame) {
    double best = 1e30;
    for (int trial = 0; trial < BENCH_TRIALS; trial++) {
        unsigned h = 0;
        double t0 = now_ns();
        for (int i = 0; i < BENCH_ITERS; i++) {
            composite_additive(m, frame);
            h += frame[(unsigned)(i * 97u) % FRAME_BYTES];
        }
        double dt = now_ns() - t0;
        bench_sink ^= h;
        if (dt < best) best = dt;
    }
    return best / (BENCH_ITERS * (double)(IMG_W * IMG_H));
}

int main(void) {
    TileMap m;
    map_init(&m);
    for (int x = 5; x < 10; x++) map_set(&m, 0, x, 5, 1);
    map_set(&m, 1, 5, 5, 2);

    check("single_mask", autotile_mask(&m, 0, 5, 5) == 2);          /* only E */
    check("middle_mask", autotile_mask(&m, 0, 7, 5) == (2 | 8));    /* E and W */
    check("end_mask", autotile_mask(&m, 0, 9, 5) == 8);
    check("isolated_mask", autotile_mask(&m, 0, 20, 20) == 0);
    check("empty_variant", autotile_variant(&m, 0, 20, 20) == -1);
    check("filled_variant", autotile_variant(&m, 0, 7, 5) == (2 | 8));

    check("cull_all", cull_count(&m, 0, 0, W - 1, H - 1) == 6);
    check("cull_sub", cull_count(&m, 5, 5, 9, 5) == 6);
    check("cull_narrow", cull_count(&m, 6, 5, 8, 5) == 3);

    m.layer_visible[0] = 0;
    check("cull_hidden_layer", cull_count(&m, 0, 0, W - 1, H - 1) == 1);
    m.layer_visible[0] = 1;

    check("out_of_range_set", (map_set(&m, 0, -1, 0, 1), map_get(&m, 0, -1, 0)) == 0);
    check("bad_layer", map_get(&m, 9, 0, 0) == 0);

    /* blend formulas against hand-computed values */
    check("normal_formula", blend_px8(BLEND_NORMAL, 40, 200, 1.0f) == 200);
    check("multiply_formula", blend_px8(BLEND_MULTIPLY, 128, 128, 1.0f) == 64);
    check("screen_formula", blend_px8(BLEND_SCREEN, 128, 128, 1.0f) == 192);
    check("overlay_dark_formula", blend_px8(BLEND_OVERLAY, 64, 192, 1.0f) == 96);
    check("overlay_light_formula", blend_px8(BLEND_OVERLAY, 192, 64, 1.0f) == 161);
    check("opacity_zero_identity", blend_px8(BLEND_MULTIPLY, 77, 200, 0.0f) == 77);

    /* the integer builder used for opaque tables is exactly the formula */
    int builder_exact = 1;
    for (int m = 0; m < BLEND_MODE_COUNT; m++)
        for (int cb = 0; cb < 256; cb++)
            for (int cs = 0; cs < 256; cs++)
                if (blend_px8_opaque((BlendMode)m, (unsigned)cb, (unsigned)cs) !=
                    blend_px8((BlendMode)m, (unsigned char)cb, (unsigned char)cs, 1.0f))
                    builder_exact = 0;
    check("opaque_builder_exhaustive_exact", builder_exact);

    /* identities the W3C formulas must satisfy for every byte pair:
       multiply/screen are duals under inversion, screen never darkens,
       multiply never brightens, and both are commutative in cb/cs. */
    int identities = 1;
    for (int cb = 0; cb < 256; cb++)
        for (int cs = 0; cs < 256; cs++) {
            int b = cb, s = cs;
            int mul = blend_px8(BLEND_MULTIPLY, (unsigned char)b, (unsigned char)s, 1.0f);
            int scr = blend_px8(BLEND_SCREEN, (unsigned char)b, (unsigned char)s, 1.0f);
            int mul_dual = 255 - blend_px8(BLEND_MULTIPLY, (unsigned char)(255 - b),
                                           (unsigned char)(255 - s), 1.0f);
            if (scr != mul_dual) identities = 0;
            if (mul > (b < s ? b : s) || mul > 255) identities = 0;
            if (scr < (b > s ? b : s) || scr > 255) identities = 0;
            if (mul != blend_px8(BLEND_MULTIPLY, (unsigned char)s, (unsigned char)b, 1.0f))
                identities = 0;
            if (scr != blend_px8(BLEND_SCREEN, (unsigned char)s, (unsigned char)b, 1.0f))
                identities = 0;
        }
    check("multiply_screen_identities", identities);
    /* overlay is hard-light with the operands swapped: it leaves the
       backdrop alone where the source is neutral (cs = 128 pushes toward
       cb), forces 255 at cs = 255, and follows the dark/light branch at
       cb = 0 / cb = 255 */
    check("overlay_identity_black_white", blend_px8(BLEND_OVERLAY, 200, 255, 1.0f) == 255 &&
                                          blend_px8(BLEND_OVERLAY, 0, 123, 1.0f) == 0 &&
                                          blend_px8(BLEND_OVERLAY, 255, 123, 1.0f) == 255 &&
                                          blend_px8(BLEND_OVERLAY, 37, 128, 1.0f) == 37);
    check("multiply_identity_white", blend_px8(BLEND_MULTIPLY, 91, 255, 1.0f) == 91);
    check("screen_identity_black", blend_px8(BLEND_SCREEN, 91, 0, 1.0f) == 91);

    /* per-layer configuration is validated and stored */
    check("blend_config_ok", map_set_blend(&m, 1, BLEND_SCREEN, 0.5f) == 0);
    check("blend_bad_layer", map_set_blend(&m, 9, BLEND_SCREEN, 0.5f) == -1);
    check("blend_bad_mode", map_set_blend(&m, 1, (BlendMode)99, 0.5f) == -1);
    check("blend_bad_opacity_hi", map_set_blend(&m, 1, BLEND_SCREEN, 1.5f) == -1);
    check("blend_bad_opacity_nan", map_set_blend(&m, 1, BLEND_SCREEN, NAN) == -1);
    check("blend_config_stored", m.layer_blend[1] == BLEND_SCREEN &&
                                 m.layer_opacity[1] == 0.5f);
    map_set_blend(&m, 1, BLEND_NORMAL, 1.0f);

    static Compositor comp;
    comp_init(&comp);
    static unsigned char ref[FRAME_BYTES];
    static unsigned char add_frame[FRAME_BYTES];

    /* two opaque layers, one blending layer on top */
    map_set_color(&m, 1, 200, 100, 50);
    map_set_color(&m, 2, 80, 160, 240);
    fill_layer(&m, 0, 1);
    fill_layer(&m, 1, 2);
    fill_layer(&m, 2, 0);

    const BlendMode modes[3] = { BLEND_MULTIPLY, BLEND_OVERLAY, BLEND_SCREEN };
    for (int i = 0; i < 3; i++) {
        map_set_blend(&m, 1, modes[i], 1.0f);
        composite_frame(&comp, &m);
        composite_ref(&m, ref);
        check(i == 0 ? "multiply_layer" : i == 1 ? "overlay_layer" : "screen_layer",
              memcmp(comp.frame, ref, FRAME_BYTES) == 0);
    }

    for (int i = 0; i < 3; i++) {
        map_set_blend(&m, 1, modes[i], 0.5f);
        composite_frame(&comp, &m);
        composite_ref(&m, ref);
        check(i == 0 ? "multiply_opacity" : i == 1 ? "overlay_opacity" : "screen_opacity",
              memcmp(comp.frame, ref, FRAME_BYTES) == 0);
    }

    /* the table path and the formula path must agree on every single byte
       for a spread of opacities, over a cluttered map */
    map_init(&m);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            map_set(&m, 0, x, y, (unsigned char)(1 + ((x * 7 + y * 13) % 40)));
            if ((x + y) % 3 != 0) map_set(&m, 1, x, y, (unsigned char)(41 + ((x * 5 + y) % 40)));
            if ((x * 3 + y) % 4 != 0) map_set(&m, 2, x, y, (unsigned char)(81 + ((x + y * 9) % 40)));
        }
    static const float opacities[] = { 0.0f, 0.1f, 0.25f, 0.5f, 0.75f, 0.9f, 1.0f };
    int exact = 1;
    for (int oi = 0; oi < (int)(sizeof(opacities) / sizeof(opacities[0])); oi++) {
        for (int mi = 0; mi < 3; mi++) {
            map_set_blend(&m, 0, BLEND_NORMAL, 1.0f);
            map_set_blend(&m, 1, modes[mi], opacities[oi]);
            map_set_blend(&m, 2, modes[(mi + 1) % 3], opacities[(oi + 3) % 7]);
            composite_frame(&comp, &m);
            composite_ref(&m, ref);
            if (memcmp(comp.frame, ref, FRAME_BYTES) != 0) exact = 0;
        }
    }
    check("table_matches_formula_all_opacities", exact);

    /* hidden layers are skipped and leave the frame untouched */
    map_set_blend(&m, 0, BLEND_NORMAL, 1.0f);   /* opaque normal: no table */
    map_set_blend(&m, 1, BLEND_MULTIPLY, 0.6f);
    map_set_blend(&m, 2, BLEND_OVERLAY, 0.6f);
    composite_frame(&comp, &m);
    memcpy(add_frame, comp.frame, FRAME_BYTES);
    m.layer_visible[1] = 0;
    composite_frame(&comp, &m);
    composite_ref(&m, ref);
    check("hidden_layer_exact", memcmp(comp.frame, ref, FRAME_BYTES) == 0);
    check("hidden_layer_skips_table", comp.blend_lookups == 1L * IMG_W * IMG_H);
    m.layer_visible[1] = 1;
    composite_frame(&comp, &m);
    check("visible_layer_uses_table", comp.blend_lookups == 2L * IMG_W * IMG_H);

    /* zero opacity is a no-op */
    map_set_blend(&m, 2, BLEND_SCREEN, 0.0f);
    composite_frame(&comp, &m);
    memcpy(add_frame, comp.frame, FRAME_BYTES);
    map_set_blend(&m, 2, BLEND_NORMAL, 1.0f);
    m.layer_visible[2] = 0;
    composite_frame(&comp, &m);
    check("opacity_zero_noop", memcmp(comp.frame, add_frame, FRAME_BYTES) == 0);
    m.layer_visible[2] = 1;

    /* fully opaque normal coverage reproduces the source image exactly */
    map_init(&m);
    map_set_color(&m, 1, 200, 100, 50);
    fill_layer(&m, 0, 1);
    fill_layer(&m, 1, 0);
    fill_layer(&m, 2, 0);
    composite_frame(&comp, &m);
    composite_ref(&m, ref);
    check("opaque_normal_matches_ref", memcmp(comp.frame, ref, FRAME_BYTES) == 0);
    check("opaque_normal_is_source", frame_px_is(comp.frame, 16, 16, 200, 100, 50) &&
                                      frame_px_is(comp.frame, 0, 0, 200, 100, 50));
    check("opaque_alpha_filled", comp.frame[(16 * IMG_W + 16) * 4 + 3] == 255);

    /* "fully opaque coverage leaves the image unchanged": a normal layer at
       opacity 1 must reproduce the source pixels exactly, with no second
       blend pass, no darkening and alpha = 255. */
    {
        TileMap src;
        map_init(&src);
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                map_set(&src, 0, x, y, (unsigned char)(1 + ((x * 5 + y * 11) % 200)));

        map_init(&m);
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                map_set(&m, 0, x, y, map_get(&src, 0, x, y));
        composite_frame(&comp, &m);
        int identical = 1;
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                unsigned char id = map_get(&src, 0, x, y);
                const unsigned char* sc = src.colors[id];
                const unsigned char* p = comp.frame +
                    ((size_t)(y * TILE) * IMG_W + (size_t)(x * TILE)) * 4;
                for (int ty = 0; ty < TILE; ty++)
                    for (int tx = 0; tx < TILE; tx++) {
                        const unsigned char* q = p + ((size_t)ty * IMG_W + tx) * 4;
                        if (q[0] != sc[0] || q[1] != sc[1] || q[2] != sc[2] || q[3] != 255)
                            identical = 0;
                    }
            }
        check("opaque_normal_reproduces_source", identical);

        /* re-compositing the same frame must be idempotent: nothing was
           pre-blended into the source data */
        memcpy(ref, comp.frame, FRAME_BYTES);
        composite_frame(&comp, &m);
        check("opaque_normal_composite_idempotent",
              memcmp(ref, comp.frame, FRAME_BYTES) == 0);
    }

    /* every mode at full opacity over full coverage, checked per channel
       against the formula with the literal backdrop underneath */
    for (int mi = 0; mi < 3; mi++) {
        map_init(&m);
        fill_layer(&m, 0, 3);
        fill_layer(&m, 1, 4);
        fill_layer(&m, 2, 5);
        map_set_blend(&m, 1, BLEND_NORMAL, 1.0f);
        map_set_blend(&m, 2, modes[mi], 1.0f);
        composite_frame(&comp, &m);
        composite_ref(&m, ref);
        const unsigned char* bc = m.colors[4];   /* opaque layer 1 underneath */
        const unsigned char* sc = m.colors[5];
        int ok = memcmp(comp.frame, ref, FRAME_BYTES) == 0;
        for (int ch = 0; ch < 3; ch++)
            ok = ok && comp.frame[(100 * IMG_W + 100) * 4 + ch] ==
                       blend_px8(modes[mi], bc[ch], sc[ch], 1.0f);
        check(mi == 0 ? "multiply_full_cover" : mi == 1 ? "overlay_full_cover"
                                                        : "screen_full_cover", ok);
    }

    /* the table is built at configuration time and reused across frames */
    map_init(&m);
    fill_layer(&m, 0, 3);
    map_set_blend(&m, 0, BLEND_OVERLAY, 0.7f);
    composite_frame(&comp, &m);
    long rebuilds = comp.table_rebuilds;
    for (int i = 0; i < 5; i++) composite_frame(&comp, &m);
    check("table_reused_across_frames", comp.table_rebuilds == rebuilds);
    map_set_blend(&m, 0, BLEND_OVERLAY, 0.8f);
    composite_frame(&comp, &m);
    check("table_rebuilt_on_config_change", comp.table_rebuilds == rebuilds + 1);

    /* blend cost against a per-pixel additive baseline on the same map */
    map_init(&m);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            map_set(&m, 0, x, y, (unsigned char)(1 + ((x + y) % 60)));
            map_set(&m, 1, x, y, (unsigned char)(61 + ((x * 3 + y) % 60)));
            map_set(&m, 2, x, y, (unsigned char)(121 + ((x + y * 5) % 60)));
        }
    map_set_blend(&m, 0, BLEND_SCREEN, 1.0f);
    map_set_blend(&m, 1, BLEND_MULTIPLY, 1.0f);
    map_set_blend(&m, 2, BLEND_OVERLAY, 1.0f);
    composite_frame(&comp, &m);              /* build tables / warm caches */
    composite_additive(&m, add_frame);

    double blend_ns = bench_blend(&comp, &m);
    double add_ns = bench_additive(&m, add_frame);
    printf("blend_ns_px=%.4f additive_ns_px=%.4f ratio=%.3f\n",
           blend_ns, add_ns, blend_ns / add_ns);
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
    /* instrumentation reorders the cost model; report, do not assert */
    printf("blend_cost_le_additive=skipped(sanitizer)\n");
#else
    check("blend_cost_le_additive", blend_ns <= add_ns);
#endif

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
