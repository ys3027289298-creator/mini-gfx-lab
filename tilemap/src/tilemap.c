
/* tilemap: layered tile map with 8-neighbour autotiling (47 blob variants,
   4-neighbour fallback for 16-variant tilesets) and per-layer culling. */
#include <stdio.h>
#include <string.h>

#define W 32
#define H 32
#define LAYERS 3

/* Neighbour bits. The cardinals join edges; the diagonals only carry
   inner/outer-corner detail once both adjacent cardinals are filled. */
enum {
    ADJ_N  = 1 << 0,
    ADJ_E  = 1 << 1,
    ADJ_S  = 1 << 2,
    ADJ_W  = 1 << 3,
    ADJ_NE = 1 << 4,
    ADJ_SE = 1 << 5,
    ADJ_SW = 1 << 6,
    ADJ_NW = 1 << 7,
};

#define TILE_VARIANTS_4 16   /* 4-neighbour tileset: cardinal masks only */
#define TILE_VARIANTS_8 47   /* blob tileset: 16 edge cases + 31 corner cases */

typedef struct {
    unsigned char base[LAYERS][H][W];   /* 0 = empty, else tile id */
    int layer_visible[LAYERS];
} TileMap;

static void map_init(TileMap* m) {
    memset(m, 0, sizeof(*m));
    for (int l = 0; l < LAYERS; l++) m->layer_visible[l] = 1;
}

static void map_set(TileMap* m, int layer, int x, int y, unsigned char id) {
    if (layer < 0 || layer >= LAYERS || x < 0 || x >= W || y < 0 || y >= H) return;
    m->base[layer][y][x] = id;
}

static unsigned char map_get(const TileMap* m, int layer, int x, int y) {
    if (layer < 0 || layer >= LAYERS || x < 0 || x >= W || y < 0 || y >= H) return 0;
    return m->base[layer][y][x];
}

/* 8-bit neighbour mask (N,E,S,W,NE,SE,SW,NW). A diagonal only counts when
   both cardinals beside it are filled: a lone diagonal neighbour does not
   touch this cell's edge, so it must not pull the artwork toward a corner
   that has no continuous border. */
static int autotile_mask(const TileMap* m, int layer, int x, int y) {
    int mask = 0;
    if (map_get(m, layer, x, y - 1)) mask |= ADJ_N;
    if (map_get(m, layer, x + 1, y)) mask |= ADJ_E;
    if (map_get(m, layer, x, y + 1)) mask |= ADJ_S;
    if (map_get(m, layer, x - 1, y)) mask |= ADJ_W;
    if (map_get(m, layer, x + 1, y - 1)) mask |= ADJ_NE;
    if (map_get(m, layer, x + 1, y + 1)) mask |= ADJ_SE;
    if (map_get(m, layer, x - 1, y + 1)) mask |= ADJ_SW;
    if (map_get(m, layer, x - 1, y - 1)) mask |= ADJ_NW;
    if ((mask & (ADJ_N | ADJ_E)) != (ADJ_N | ADJ_E)) mask &= ~ADJ_NE;
    if ((mask & (ADJ_S | ADJ_E)) != (ADJ_S | ADJ_E)) mask &= ~ADJ_SE;
    if ((mask & (ADJ_S | ADJ_W)) != (ADJ_S | ADJ_W)) mask &= ~ADJ_SW;
    if ((mask & (ADJ_N | ADJ_W)) != (ADJ_N | ADJ_W)) mask &= ~ADJ_NW;
    return mask;
}

/* Non-cardinal canonical masks, sorted. Index i here is variant 16 + i;
   the 16 cardinal-only masks map to variants 0..15, so a 4-neighbour
   tileset can use the same mask value as a variant id. */
static const unsigned char blob_corner_masks[31] = {
    19, 23, 27, 31, 38, 39, 46, 47, 55, 63, 76, 77, 78, 79, 95, 110,
    111, 127, 137, 139, 141, 143, 155, 159, 175, 191, 205, 207, 223, 239, 255,
};

/* Map a canonical 8-bit mask to its 0..46 blob variant, or -1. */
static int blob_index(int mask) {
    if (mask < TILE_VARIANTS_4) return mask;
    for (int i = 0; i < (int)(sizeof(blob_corner_masks) / sizeof(blob_corner_masks[0])); i++)
        if (blob_corner_masks[i] == mask) return TILE_VARIANTS_4 + i;
    return -1;
}

/* Autotile variant for a filled cell. A full 8-neighbour tileset has 47
   variants; a 4-neighbour tileset has no corner artwork, so diagonal
   information is dropped and the cardinal mask (0..15) is returned. Cells
   whose mask is already cardinal-only get the same id under both modes,
   which keeps existing 4-neighbour maps looking unchanged. */
static int autotile_variant_mode(const TileMap* m, int layer, int x, int y, int variants) {
    if (!map_get(m, layer, x, y)) return -1;
    int mask = autotile_mask(m, layer, x, y);
    if (variants == TILE_VARIANTS_8) {
        int v = blob_index(mask);
        if (v >= 0) return v;
    }
    return mask & 15;
}

/* Convenience wrappers for the two tileset modes. */
static int autotile_variant(const TileMap* m, int layer, int x, int y) {
    return autotile_variant_mode(m, layer, x, y, TILE_VARIANTS_8);
}

static int autotile_variant_4(const TileMap* m, int layer, int x, int y) {
    return autotile_variant_mode(m, layer, x, y, TILE_VARIANTS_4);
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

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Build a 3x3 patch around (cx,cy) from an 8-bit neighbour mask plus the
   centre, using the same bit order as autotile_mask. */
static void paint_patch(TileMap* m, int layer, int cx, int cy, int mask) {
    static const int NX[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    static const int NY[8] = {-1, 0, 1, 0, -1, 1, 1, -1};
    for (int i = 0; i < 8; i++)
        if (mask & (1 << i)) map_set(m, layer, cx + NX[i], cy + NY[i], 1);
    map_set(m, layer, cx, cy, 1);
}

int main(void) {
    TileMap m;
    map_init(&m);
    for (int x = 5; x < 10; x++) map_set(&m, 0, x, 5, 1);
    map_set(&m, 1, 5, 5, 2);

    /* The strip's edge tiles are N-free, so the diagonal above/on the strip
       is not adjacent to them; the mask stays cardinal-only. */
    check("single_mask", autotile_mask(&m, 0, 5, 5) == 2);          /* only E */
    check("middle_mask", autotile_mask(&m, 0, 7, 5) == (2 | 8));    /* E and W */
    check("end_mask", autotile_mask(&m, 0, 9, 5) == 8);
    check("isolated_mask", autotile_mask(&m, 0, 20, 20) == 0);
    check("empty_variant", autotile_variant(&m, 0, 20, 20) == -1);
    check("filled_variant", autotile_variant(&m, 0, 7, 5) == (2 | 8));

    /* The strip's two end tiles have a filled diagonal (NE/SE of the end
       tile) but only one cardinal, so the diagonal is not counted: an end
       cap must not turn into a corner. */
    check("end_mask_ignores_single_diag", autotile_mask(&m, 0, 5, 5) == ADJ_E);
    check("end_variant", autotile_variant(&m, 0, 9, 5) == ADJ_W);
    {
        TileMap c;
        map_init(&c);
        map_set(&c, 0, 5, 3, 1);   /* NE of the centre only */
        map_set(&c, 0, 4, 4, 1);
        check("lone_diag_mask", autotile_mask(&c, 0, 4, 4) == 0);
    }

    /* 4-neighbour fallback: 16 ids, corners degrade to the cardinal mask */
    check("fallback_strip_variant", autotile_variant_4(&m, 0, 7, 5) == (2 | 8));
    check("fallback_edge_variant", autotile_variant_4(&m, 0, 5, 5) == ADJ_E);

    /* Eight hand-labelled edge/corner cases. Expected variants come from the
       47-entry blob table: the 16 cardinal-only masks keep ids 0..15, the 31
       canonical corner masks follow at 16..46. A "corner" here is outer when
       the diagonal is filled (terrain wraps around an empty cell) and inner
       when both cardinals are filled but the diagonal is empty (a notch). */
    struct { const char* name; int mask; int variant; } cases[] = {
        {"case_isolated",      0,                                         0  },
        {"case_edge_end_e",    ADJ_E,                                     2  },
        {"case_straight_ew",   ADJ_E | ADJ_W,                             10 },
        {"case_inner_corner_ne",  ADJ_N | ADJ_E,                          3  },
        {"case_outer_corner_ne",  ADJ_N | ADJ_E | ADJ_NE,                 16 },
        {"case_outer_corner_sw",  ADJ_S | ADJ_W | ADJ_SW,                 26 },
        {"case_interior",      ADJ_N | ADJ_E | ADJ_S | ADJ_W,             15 },
        {"case_lone_diag",     ADJ_NE,                                    0  },
    };
    for (int i = 0; i < (int)(sizeof(cases) / sizeof(cases[0])); i++) {
        TileMap c;
        map_init(&c);
        paint_patch(&c, 0, 4, 4, cases[i].mask);
        check(cases[i].name, autotile_variant(&c, 0, 4, 4) == cases[i].variant);
    }

    /* A 4-neighbour tileset has no corner artwork; corners fall back to the
       cardinal mask, so cells that were already correct keep their id. */
    {
        TileMap c;
        map_init(&c);
        paint_patch(&c, 0, 4, 4, ADJ_N | ADJ_E | ADJ_NE);
        check("fallback_corner_ne", autotile_variant_4(&c, 0, 4, 4) == (ADJ_N | ADJ_E));
        map_init(&c);
        paint_patch(&c, 0, 4, 4, ADJ_N | ADJ_E | ADJ_S | ADJ_W | ADJ_NE);
        check("fallback_full", autotile_variant_4(&c, 0, 4, 4) == (ADJ_N | ADJ_E | ADJ_S | ADJ_W));
    }

    check("cull_all", cull_count(&m, 0, 0, W - 1, H - 1) == 6);
    check("cull_sub", cull_count(&m, 5, 5, 9, 5) == 6);
    check("cull_narrow", cull_count(&m, 6, 5, 8, 5) == 3);

    m.layer_visible[0] = 0;
    check("cull_hidden_layer", cull_count(&m, 0, 0, W - 1, H - 1) == 1);
    m.layer_visible[0] = 1;

    check("out_of_range_set", (map_set(&m, 0, -1, 0, 1), map_get(&m, 0, -1, 0)) == 0);
    check("bad_layer", map_get(&m, 9, 0, 0) == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
