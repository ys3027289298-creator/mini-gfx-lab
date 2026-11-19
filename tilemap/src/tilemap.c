
/* tilemap: layered tile map with 4-neighbour autotiling, per-layer parallax
   (independent offset + scale) and viewport culling.

   Culling model: each layer maps tile (tx, ty) to the screen rect
   [ox + tx*step, ox + (tx+1)*step) x [oy + ty*step, oy + (ty+1)*step),
   step = TILE_PX * scale. visible_range() solves for the tile-index interval
   intersecting the viewport and clamps it to map bounds, so the culled render
   path indexes base[][][] directly and can never read out of bounds. Its loop
   visits exactly the visible-rect cells, i.e. cost linear in visible tiles. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#define W 64
#define H 64
#define LAYERS 3
#define TILE_PX 8              /* base tile edge in pixels at scale 1.0 */
#define VIEW_MAX 96            /* largest viewport edge used by the tests */

typedef struct {
    unsigned char base[LAYERS][H][W];   /* 0 = empty, else tile id */
    int layer_visible[LAYERS];
    float par_ox[LAYERS];               /* screen-space parallax offset, px */
    float par_oy[LAYERS];
    float par_scale[LAYERS];            /* 1.0 = TILE_PX screen px per tile */
} TileMap;

static void map_init(TileMap* m) {
    memset(m, 0, sizeof(*m));
    for (int l = 0; l < LAYERS; l++) {
        m->layer_visible[l] = 1;
        m->par_scale[l] = 1.0f;
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

/* per-layer parallax: independent offset and uniform scale */
static void layer_set_view(TileMap* m, int layer, float ox, float oy, float scale) {
    if (layer < 0 || layer >= LAYERS) return;
    m->par_ox[layer] = ox;
    m->par_oy[layer] = oy;
    m->par_scale[layer] = scale > 1e-6f ? scale : 1e-6f;
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

/* count visible tiles inside a map-space rect across visible layers */
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

/* float -> int floor/ceil with a safe window so huge/inf/NaN offsets and
   scales cannot trigger undefined float-to-int behaviour */
static int safe_ifloor(float v) {
    if (!(v > -1e6f)) v = -1e6f;
    if (v > 1e6f) v = 1e6f;
    return (int)floorf(v);
}
static int safe_iceil(float v) {
    if (!(v > -1e6f)) v = -1e6f;
    if (v > 1e6f) v = 1e6f;
    return (int)ceilf(v);
}

/* Visible tile-index interval for one layer, clamped to [0,W) x [0,H).
   Tile tx covers [ox+tx*step, ox+(tx+1)*step); it is visible iff that
   half-open interval intersects [0, view_w). Returns 0 (empty interval,
   x0 > x1) when the layer is hidden or fully offscreen. */
static int visible_range(const TileMap* m, int layer, int view_w, int view_h,
                         int* x0, int* y0, int* x1, int* y1) {
    *x0 = *y0 = 0;
    *x1 = *y1 = -1;
    if (layer < 0 || layer >= LAYERS || !m->layer_visible[layer]) return 0;
    float step = TILE_PX * m->par_scale[layer];
    if (!(step > 0.0f)) return 0;
    float ox = m->par_ox[layer];
    float oy = m->par_oy[layer];

    int rx0 = safe_ifloor(-ox / step);
    int rx1 = safe_iceil((view_w - ox) / step) - 1;
    int ry0 = safe_ifloor(-oy / step);
    int ry1 = safe_iceil((view_h - oy) / step) - 1;

    /* clamp instead of letting a scrolled-off map index out of bounds */
    if (rx0 < 0) rx0 = 0;
    if (ry0 < 0) ry0 = 0;
    if (rx1 > W - 1) rx1 = W - 1;
    if (ry1 > H - 1) ry1 = H - 1;
    if (rx0 > rx1 || ry0 > ry1) return 0;

    *x0 = rx0; *y0 = ry0; *x1 = rx1; *y1 = ry1;
    return 1;
}

typedef struct {
    int w, h;
    unsigned char* px;       /* one byte per pixel, stores tile id */
} Framebuf;

static void fb_clear(Framebuf* fb) {
    memset(fb->px, 0, (size_t)fb->w * fb->h);
}

/* fill the pixels covered by one tile rect, clipped to the framebuffer */
static void tile_blit(Framebuf* fb, unsigned char id, float sx, float sy, float step) {
    if (id == 0) return;
    int x0 = safe_ifloor(sx);
    int y0 = safe_ifloor(sy);
    int x1 = safe_ifloor(sx + step);
    int y1 = safe_ifloor(sy + step);
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > fb->w) x1 = fb->w;
    if (y1 > fb->h) y1 = fb->h;
    for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++)
            fb->px[y * fb->w + x] = id;
}

/* reference path: walk every map cell; blitting is clipped per tile */
static void render_full(const TileMap* m, Framebuf* fb) {
    fb_clear(fb);
    for (int l = 0; l < LAYERS; l++) {
        if (!m->layer_visible[l]) continue;
        float step = TILE_PX * m->par_scale[l];
        for (int ty = 0; ty < H; ty++) {
            for (int tx = 0; tx < W; tx++) {
                unsigned char id = m->base[l][ty][tx];
                if (!id) continue;
                tile_blit(fb, id,
                          m->par_ox[l] + tx * step,
                          m->par_oy[l] + ty * step, step);
            }
        }
    }
}

/* culled path: only the clamped visible rect per layer is visited. Cells are
   indexed directly because visible_range() guarantees in-bounds indices.
   *examined receives the number of visited cells (= sum of visible rects). */
static void render_culled(const TileMap* m, Framebuf* fb, long* examined) {
    fb_clear(fb);
    long n = 0;
    for (int l = 0; l < LAYERS; l++) {
        int x0, y0, x1, y1;
        if (!visible_range(m, l, fb->w, fb->h, &x0, &y0, &x1, &y1)) continue;
        float step = TILE_PX * m->par_scale[l];
        for (int ty = y0; ty <= y1; ty++) {
            for (int tx = x0; tx <= x1; tx++) {
                n++;
                unsigned char id = m->base[l][ty][tx];   /* clamped: safe */
                if (!id) continue;
                tile_blit(fb, id,
                          m->par_ox[l] + tx * step,
                          m->par_oy[l] + ty * step, step);
            }
        }
    }
    if (examined) *examined = n;
}

static int frames_equal(const Framebuf* a, const Framebuf* b) {
    return a->w == b->w && a->h == b->h &&
           memcmp(a->px, b->px, (size_t)a->w * a->h) == 0;
}

static int fb_all_zero(const Framebuf* fb) {
    for (int i = 0; i < fb->w * fb->h; i++)
        if (fb->px[i]) return 0;
    return 1;
}

static unsigned rng_state = 0x12345678u;
static float frand(float lo, float hi) {
    rng_state = rng_state * 1103515245u + 12345u;
    float t = (float)(rng_state >> 8) / 16777216.0f;
    return lo + t * (hi - lo);
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned char buf_a[VIEW_MAX * VIEW_MAX];
static unsigned char buf_b[VIEW_MAX * VIEW_MAX];

int main(void) {
    TileMap m;
    map_init(&m);
    for (int x = 5; x < 10; x++) map_set(&m, 0, x, 5, 1);
    map_set(&m, 1, 5, 5, 2);

    /* ---- autotiling (unchanged behaviour) ---- */
    check("single_mask", autotile_mask(&m, 0, 5, 5) == 2);          /* only E */
    check("middle_mask", autotile_mask(&m, 0, 7, 5) == (2 | 8));    /* E and W */
    check("end_mask", autotile_mask(&m, 0, 9, 5) == 8);
    check("isolated_mask", autotile_mask(&m, 0, 20, 20) == 0);
    check("empty_variant", autotile_variant(&m, 0, 20, 20) == -1);
    check("filled_variant", autotile_variant(&m, 0, 7, 5) == (2 | 8));

    /* ---- map-space region culling (unchanged behaviour) ---- */
    check("cull_all", cull_count(&m, 0, 0, W - 1, H - 1) == 6);
    check("cull_sub", cull_count(&m, 5, 5, 9, 5) == 6);
    check("cull_narrow", cull_count(&m, 6, 5, 8, 5) == 3);

    m.layer_visible[0] = 0;
    check("cull_hidden_layer", cull_count(&m, 0, 0, W - 1, H - 1) == 1);
    m.layer_visible[0] = 1;

    check("out_of_range_set", (map_set(&m, 0, -1, 0, 1), map_get(&m, 0, -1, 0)) == 0);
    check("bad_layer", map_get(&m, 9, 0, 0) == 0);

    /* ---- fill a deterministic multi-layer pattern for render tests ---- */
    for (int ty = 0; ty < H; ty++) {
        for (int tx = 0; tx < W; tx++) {
            map_set(&m, 0, tx, ty, (unsigned char)(((tx * 7 + ty * 13) % 17) + 1));
            if ((tx ^ ty) % 3 == 0)
                map_set(&m, 1, tx, ty, (unsigned char)(((tx + ty) % 9) + 20));
            if (tx == 0 || ty == 0 || tx == W - 1 || ty == H - 1)
                map_set(&m, 2, tx, ty, 77);
        }
    }

    /* ---- visible-range math: offsets and scales ---- */
    int x0, y0, x1, y1;
    layer_set_view(&m, 0, 0.0f, 0.0f, 1.0f);            /* step = 8 px */
    check("range_basic", visible_range(&m, 0, 40, 24, &x0, &y0, &x1, &y1) &&
          x0 == 0 && y0 == 0 && x1 == 4 && y1 == 2);    /* 5 x 3 tiles */

    layer_set_view(&m, 0, -16.0f, 0.0f, 1.0f);          /* map pans left */
    check("range_scrolled", visible_range(&m, 0, 40, 24, &x0, &y0, &x1, &y1) &&
          x0 == 2 && x1 == 6);

    layer_set_view(&m, 0, 20.0f, 0.0f, 1.0f);           /* raw x0 = -3 -> clamp 0 */
    check("range_clamp_near", visible_range(&m, 0, 40, 24, &x0, &y0, &x1, &y1) &&
          x0 == 0 && x1 == 2);

    layer_set_view(&m, 0, 0.0f, 0.0f, 2.0f);            /* step = 16 px */
    check("range_zoom_in", visible_range(&m, 0, 40, 24, &x0, &y0, &x1, &y1) &&
          x1 == 2 && y1 == 1);                          /* 3 x 2 tiles */

    layer_set_view(&m, 0, 0.0f, 0.0f, 0.5f);            /* step = 4 px */
    check("range_zoom_out", visible_range(&m, 0, 40, 24, &x0, &y0, &x1, &y1) &&
          x1 == 9 && y1 == 5);                          /* 10 x 6 tiles */

    /* ---- parallax: camera pans 16 px, factors 1.0 / 0.5 / 0.0 ---- */
    layer_set_view(&m, 0, -16.0f, 0.0f, 1.0f);
    layer_set_view(&m, 1, -8.0f, 0.0f, 1.0f);
    layer_set_view(&m, 2, 0.0f, 0.0f, 1.0f);
    int px0[LAYERS], px1[LAYERS];
    for (int l = 0; l < LAYERS; l++)
        visible_range(&m, l, 40, 24, &px0[l], &y0, &px1[l], &y1);
    check("parallax_factors", px0[0] == 2 && px0[1] == 1 && px0[2] == 0);

    /* ---- fully offscreen: empty range, nothing examined or drawn ---- */
    for (int l = 0; l < LAYERS; l++)
        layer_set_view(&m, l, 10000.0f, 10000.0f, 1.0f);
    long examined = -1;
    Framebuf fb = {40, 24, buf_a};
    render_culled(&m, &fb, &examined);
    check("offscreen_empty", !visible_range(&m, 0, 40, 24, &x0, &y0, &x1, &y1) &&
          examined == 0 && fb_all_zero(&fb));

    for (int l = 0; l < LAYERS; l++)
        layer_set_view(&m, l, -10000.0f, -10000.0f, 1.0f);
    render_culled(&m, &fb, &examined);
    check("offscreen_negative_empty", examined == 0 && fb_all_zero(&fb));

    /* ---- fuzz: ranges must always be clamped, never carry OOB indices ---- */
    static const float scales[] = {0.25f, 0.5f, 1.0f, 1.5f, 2.0f, 4.0f};
    int clamp_bad = 0, empties = 0;
    for (int i = 0; i < 2000; i++) {
        int vw = 1 + (int)frand(0.0f, (float)VIEW_MAX);
        int vh = 1 + (int)frand(0.0f, (float)VIEW_MAX);
        for (int l = 0; l < LAYERS; l++) {
            layer_set_view(&m, l, frand(-2000.0f, 2000.0f),
                           frand(-2000.0f, 2000.0f),
                           scales[(int)frand(0.0f, 5.999f)]);
            if (!visible_range(&m, l, vw, vh, &x0, &y0, &x1, &y1)) {
                empties++;
                continue;
            }
            if (x0 < 0 || y0 < 0 || x1 >= W || y1 >= H || x0 > x1 || y0 > y1)
                clamp_bad = 1;
        }
    }
    check("fuzz_range_clamped", clamp_bad == 0);
    check("fuzz_saw_empty", empties > 0);

    /* ---- pixel-identical rendering: culled vs full, fixed scenarios ---- */
    static const struct {
        float ox, oy, sc;
        int vw, vh;
    } scenes[] = {
        {    0.0f,   0.0f, 1.0f,  40, 24},
        {  -12.0f,  -7.0f, 1.0f,  40, 24},
        {   20.0f,  13.0f, 1.0f,  40, 24},
        {    0.0f,   0.0f, 0.5f,  40, 24},
        {    0.0f,   0.0f, 2.0f,  40, 24},
        {  -30.0f,  18.0f, 1.5f,  57, 33},
        { 3.25f, -5.75f, 0.75f, 31, 19},
        {10000.0f,   0.0f, 1.0f,  40, 24},
        {-10000.0f,-10000.0f,1.0f, 40, 24},
    };
    int pixel_bad = 0;
    for (unsigned i = 0; i < sizeof(scenes) / sizeof(scenes[0]); i++) {
        for (int l = 0; l < LAYERS; l++)
            layer_set_view(&m, l, scenes[i].ox, scenes[i].oy, scenes[i].sc);
        Framebuf fa = {scenes[i].vw, scenes[i].vh, buf_a};
        Framebuf fb2 = {scenes[i].vw, scenes[i].vh, buf_b};
        render_full(&m, &fa);
        render_culled(&m, &fb2, NULL);
        if (!frames_equal(&fa, &fb2)) pixel_bad = 1;
    }
    check("pixel_match_scenarios", pixel_bad == 0);

    /* per-layer independent offsets/scales, plus a hidden layer */
    for (int l = 0; l < LAYERS; l++)
        layer_set_view(&m, l, frand(-60.0f, 60.0f), frand(-40.0f, 40.0f),
                       scales[l % 6]);
    m.layer_visible[1] = 0;
    Framebuf fa = {48, 32, buf_a};
    Framebuf fb2 = {48, 32, buf_b};
    render_full(&m, &fa);
    render_culled(&m, &fb2, NULL);
    check("pixel_match_independent_layers", frames_equal(&fa, &fb2));
    check("hidden_layer_skipped",
          !visible_range(&m, 1, 48, 32, &x0, &y0, &x1, &y1));
    m.layer_visible[1] = 1;

    /* randomised pixel comparison */
    int rnd_bad = 0;
    for (int i = 0; i < 64; i++) {
        int vw = 1 + (int)frand(1.0f, (float)VIEW_MAX);
        int vh = 1 + (int)frand(1.0f, (float)VIEW_MAX);
        for (int l = 0; l < LAYERS; l++)
            layer_set_view(&m, l, frand(-500.0f, 500.0f),
                           frand(-500.0f, 500.0f),
                           scales[(int)frand(0.0f, 5.999f)]);
        Framebuf ra = {vw, vh, buf_a};
        Framebuf rb = {vw, vh, buf_b};
        render_full(&m, &ra);
        render_culled(&m, &rb, NULL);
        if (!frames_equal(&ra, &rb)) rnd_bad = 1;
    }
    check("pixel_match_random", rnd_bad == 0);

    /* ---- culling cost is linear in visible cells, not map size ---- */
    for (int l = 0; l < LAYERS; l++)
        layer_set_view(&m, l, 0.0f, 0.0f, 1.0f);

    Framebuf small = {16, 16, buf_a};
    Framebuf large = {48, 40, buf_b};
    long ex_small = 0, ex_large = 0;
    render_culled(&m, &small, &ex_small);
    render_culled(&m, &large, &ex_large);
    check("linear_small_viewport", ex_small == LAYERS * 2 * 2);       /* 2x2/layer */
    check("linear_large_viewport", ex_large == LAYERS * 6 * 5);       /* 6x5/layer */
    check("linear_beats_full_scan", ex_small * 100 < W * H &&
          ex_large < (long)W * H * LAYERS);

    /* examined must equal the visible-rect area summed over layers */
    long expected = 0;
    for (int l = 0; l < LAYERS; l++) {
        if (visible_range(&m, l, large.w, large.h, &x0, &y0, &x1, &y1))
            expected += (long)(x1 - x0 + 1) * (y1 - y0 + 1);
    }
    check("linear_examined_is_visible_area", ex_large == expected);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
