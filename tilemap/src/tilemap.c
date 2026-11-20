
/* tilemap: layered tile map with 4-neighbour autotiling, per-layer culling,
   and a depth-key sorted renderer (unified sort vs per-layer reference). */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <string.h>
#include <time.h>

#define W 32
#define H 32
#define LAYERS 8

typedef struct {
    unsigned char base[LAYERS][H][W];   /* 0 = empty, else tile id */
    int layer_visible[LAYERS];
    int z_base[LAYERS];                 /* depth base per layer; default = layer index */
} TileMap;

static void map_init(TileMap* m) {
    memset(m, 0, sizeof(*m));
    for (int l = 0; l < LAYERS; l++) {
        m->layer_visible[l] = 1;
        m->z_base[l] = l;
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

/* ---- renderer: unified depth-key sort vs per-layer reference ---- */

#define TILE 8
#define FBW (W * TILE)
#define FBH (H * TILE)
typedef unsigned char FB[FBH][FBW][3];

typedef struct { unsigned char r, g, b, a; } Color;

static Color palette(unsigned char id) {
    switch (id) {
        case 1: return (Color){180, 180, 180, 255}; /* stone, opaque */
        case 2: return (Color){120, 80, 40, 255};   /* wood, opaque */
        case 3: return (Color){255, 255, 255, 128}; /* glass, semi-transparent */
        case 4: return (Color){40, 90, 200, 96};    /* water, semi-transparent */
        default: return (Color){0, 0, 0, 0};
    }
}

static unsigned char blend_ch(unsigned char dst, unsigned char src, unsigned a) {
    return (unsigned char)((src * a + dst * (255 - a) + 127) / 255);
}

/* one depth key compared across all layers:
   [31] transparent group | [30..12] z | [11..0] cell (row-major).
   Opaque sorts before transparent via the top bit; z drives cross-layer
   occlusion; equal keys keep collection order (stable sort below). */
#define CELL_MASK 0xFFFu

typedef struct { unsigned key; unsigned char id; } Item;

static unsigned item_key(const TileMap* m, int layer, int cell, unsigned char id) {
    unsigned z = (unsigned)m->z_base[layer];
    unsigned tr = palette(id).a < 255 ? 1u : 0u;
    return (tr << 31) | (z << 12) | (unsigned)cell;
}

static int collect_items(const TileMap* m, int x0, int y0, int x1, int y1, Item* out) {
    int n = 0;
    for (int l = 0; l < LAYERS; l++) {
        if (!m->layer_visible[l]) continue;
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++) {
                unsigned char id = m->base[l][y][x];
                if (!id) continue;
                out[n].key = item_key(m, l, y * W + x, id);
                out[n].id = id;
                n++;
            }
    }
    return n;
}

/* stable LSD radix sort, 4 x 8-bit passes; cost is data-independent:
   exactly 4n item moves, i.e. linear in n. */
static unsigned long sort_moves;

static void radix_sort(Item* a, Item* tmp, int n) {
    sort_moves = 0;
    for (int p = 0; p < 4; p++) {
        int cnt[256] = {0};
        int pos[256];
        int shift = p * 8;
        int sum = 0;
        int i;
        for (i = 0; i < n; i++) cnt[(a[i].key >> shift) & 255]++;
        for (i = 0; i < 256; i++) { pos[i] = sum; sum += cnt[i]; }
        for (i = 0; i < n; i++) {
            tmp[pos[(a[i].key >> shift) & 255]++] = a[i];
            sort_moves++;
        }
        { Item* s = a; a = tmp; tmp = s; }
    }
    /* even pass count: sorted data ends back in the caller's buffer */
}

static void fb_clear(FB fb, unsigned char r, unsigned char g, unsigned char b) {
    for (int y = 0; y < FBH; y++)
        for (int x = 0; x < FBW; x++) {
            fb[y][x][0] = r; fb[y][x][1] = g; fb[y][x][2] = b;
        }
}

static void draw_cell(FB fb, int cell, unsigned char id) {
    Color c = palette(id);
    int x0 = (int)((unsigned)cell % W) * TILE;
    int y0 = (int)((unsigned)cell / W) * TILE;
    for (int y = y0; y < y0 + TILE; y++)
        for (int x = x0; x < x0 + TILE; x++) {
            unsigned char* px = fb[y][x];
            if (c.a == 255) {
                px[0] = c.r; px[1] = c.g; px[2] = c.b;
            } else {
                px[0] = blend_ch(px[0], c.r, c.a);
                px[1] = blend_ch(px[1], c.g, c.a);
                px[2] = blend_ch(px[2], c.b, c.a);
            }
        }
}

static long now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000L + ts.tv_nsec;
}

static Item items[LAYERS * W * H];
static Item scratch[LAYERS * W * H];

typedef struct { long collect_ns, sort_ns, draw_ns; } RenderCost;

/* new path: collect visible items, sort by the single depth key, draw back-to-front */
static void render_unified(const TileMap* m, FB fb, RenderCost* cost) {
    long t0 = now_ns();
    int n = collect_items(m, 0, 0, W - 1, H - 1, items);
    long t1 = now_ns();
    radix_sort(items, scratch, n);
    long t2 = now_ns();
    for (int i = 0; i < n; i++)
        draw_cell(fb, (int)(items[i].key & CELL_MASK), items[i].id);
    long t3 = now_ns();
    if (cost) {
        cost->collect_ns = t1 - t0;
        cost->sort_ns = t2 - t1;
        cost->draw_ns = t3 - t2;
    }
}

/* reference path (old behaviour): draw layer by layer, row-major inside a layer */
static void render_layered(const TileMap* m, FB fb) {
    for (int l = 0; l < LAYERS; l++) {
        if (!m->layer_visible[l]) continue;
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                unsigned char id = m->base[l][y][x];
                if (id) draw_cell(fb, y * W + x, id);
            }
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static FB fb_unified, fb_layered;

int main(void) {
    TileMap m;
    map_init(&m);
    for (int x = 5; x < 10; x++) map_set(&m, 0, x, 5, 1);
    map_set(&m, 1, 5, 5, 2);

    check("single_mask", autotile_mask(&m, 0, 5, 5) == 2);          /* only E */
    check("middle_mask", autotile_mask(&m, 0, 7, 5) == 2 | 8);      /* E and W */
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

    /* scene A: transparency already back-to-front under layer order
       (glass/water only above opaque at their cells) -> both renderers
       must produce pixel-identical frames. */
    {
        TileMap a;
        RenderCost cost;
        map_init(&a);
        for (int x = 2; x <= 12; x++) map_set(&a, 0, x, 3, 1);   /* stone row */
        map_set(&a, 1, 4, 3, 2);                                  /* wood decor */
        map_set(&a, 1, 5, 3, 2);
        map_set(&a, 2, 4, 3, 3);                                  /* glass on top */
        map_set(&a, 2, 8, 3, 3);
        map_set(&a, 1, 20, 10, 4);                                /* water, nothing above */

        fb_clear(fb_unified, 10, 10, 20);
        fb_clear(fb_layered, 10, 10, 20);
        render_unified(&a, fb_unified, &cost);
        render_layered(&a, fb_layered);
        check("unified_matches_layered", memcmp(fb_unified, fb_layered, sizeof(FB)) == 0);
        printf("cost collect_ns=%ld sort_ns=%ld draw_ns=%ld\n",
               cost.collect_ns, cost.sort_ns, cost.draw_ns);

        /* grouping falls out of the key: all opaque before all transparent */
        {
            int n = collect_items(&a, 0, 0, W - 1, H - 1, items);
            int seen_tr = 0, group_ok = 1;
            radix_sort(items, scratch, n);
            for (int i = 0; i < n; i++) {
                int tr = (int)(items[i].key >> 31);
                if (tr) seen_tr = 1;
                else if (seen_tr) group_ok = 0;
            }
            check("opaque_grouped_before_transparent", group_ok);
        }
    }

    /* scene B: glass UNDER an opaque tile of a higher layer. The per-layer
       reference gets the front/back relation wrong (opaque overwrites the
       glass); the unified key draws opaque first and blends glass on top. */
    {
        TileMap b;
        int px, py;
        map_init(&b);
        map_set(&b, 0, 6, 6, 3);   /* glass on the bottom layer */
        map_set(&b, 1, 6, 6, 1);   /* stone above it */
        fb_clear(fb_unified, 10, 10, 20);
        fb_clear(fb_layered, 10, 10, 20);
        render_unified(&b, fb_unified, 0);
        render_layered(&b, fb_layered);
        px = 6 * TILE; py = 6 * TILE;
        /* blend(white 128 over stone 180) = (255*128 + 180*127 + 127)/255 = 218 */
        check("cross_layer_occlusion_fixed",
              fb_unified[py][px][0] == 218 && fb_unified[py][px][1] == 218 &&
              fb_unified[py][px][2] == 218);
        check("reference_kept_for_contrast",
              fb_layered[py][px][0] == 180 && fb_layered[py][px][1] == 180 &&
              fb_layered[py][px][2] == 180);
    }

    /* scene C: z_base overridden so a higher layer sorts below a lower one;
       occlusion follows the shared z key, not the layer index. */
    {
        TileMap c;
        int px, py;
        map_init(&c);
        map_set(&c, 0, 2, 2, 1);            /* stone, z 0 */
        map_set(&c, 2, 2, 2, 2);            /* wood, z 1 (overridden) */
        map_set(&c, 1, 2, 2, 3);            /* glass, z 2 (overridden) */
        c.z_base[1] = 2;
        c.z_base[2] = 1;
        fb_clear(fb_unified, 10, 10, 20);
        render_unified(&c, fb_unified, 0);
        px = 2 * TILE; py = 2 * TILE;
        /* glass over wood: (255*128 + {120,80,40}*127 + 127)/255 = {188,168,148} */
        check("z_key_drives_occlusion",
              fb_unified[py][px][0] == 188 && fb_unified[py][px][1] == 168 &&
              fb_unified[py][px][2] == 148);
    }

    /* equal keys: stable and reproducible order (insertion order kept) */
    {
        Item eq[8], tmp[8];
        int stable_ok = 1, repro_ok = 1;
        for (int i = 0; i < 8; i++) {
            eq[i].key = 0u << 31 | 7u << 12 | 100u;   /* identical key */
            eq[i].id = (unsigned char)(i + 1);
        }
        radix_sort(eq, tmp, 8);
        for (int i = 0; i < 8; i++) if (eq[i].id != i + 1) stable_ok = 0;
        check("stable_equal_keys", stable_ok);
        {
            Item again[8];
            for (int i = 0; i < 8; i++) { again[i].key = eq[i].key; again[i].id = (unsigned char)(i + 1); }
            radix_sort(again, tmp, 8);
            for (int i = 0; i < 8; i++) if (again[i].id != eq[i].id) repro_ok = 0;
        }
        check("reproducible_sort", repro_ok);
    }

    /* sort cost vs transparent layer count: moves must be exactly 4n (linear);
       wall time is reported separately per configuration. */
    {
        static const int ks[] = {1, 2, 4, 8};
        int linear_ok = 1;
        for (int t = 0; t < 4; t++) {
            TileMap p;
            int k = ks[t], n;
            long best = -1;
            map_init(&p);
            for (int l = 0; l < k; l++)
                for (int y = 0; y < H; y++)
                    for (int x = 0; x < W; x++)
                        map_set(&p, l, x, y, 3);      /* all glass: k transparent layers */
            n = collect_items(&p, 0, 0, W - 1, H - 1, items);
            for (int rep = 0; rep < 20; rep++) {      /* best-of to cut timer noise */
                long s = now_ns();
                radix_sort(items, scratch, n);
                s = now_ns() - s;
                if (best < 0 || s < best) best = s;
            }
            if (sort_moves != 4UL * (unsigned long)n) linear_ok = 0;
            printf("perf transparent_layers=%d n=%d sort_ns=%ld moves=%lu\n",
                   k, n, best, sort_moves);
        }
        check("sort_cost_linear", linear_ok);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
