
/* tilemap: layered tile map with 4-neighbour autotiling and per-layer culling.

   Failure encoding: invalid geometry/layers report through gfault block
   0x20 (tilemap). map_get keeps its original 0-on-bad-input contract so
   autotiling math is unchanged; map_set now reports instead of silently
   ignoring out-of-range writes. The success path is the original logic. */
#include <stdio.h>
#include <string.h>
#include "../../gfault/include/gfault.h"

#define W 32
#define H 32
#define LAYERS 3

typedef struct {
    unsigned char base[LAYERS][H][W];   /* 0 = empty, else tile id */
    int layer_visible[LAYERS];
} TileMap;

static void map_init(TileMap* m) {
    memset(m, 0, sizeof(*m));
    for (int l = 0; l < LAYERS; l++) m->layer_visible[l] = 1;
}

static int map_set(TileMap* m, int layer, int x, int y, unsigned char id) {
    if (layer < 0 || layer >= LAYERS) {
        gfault_report(GFAULT_SEG_TILEMAP, GFAULT_T_BAD_LAYER, __LINE__);
        return gfault_wrap(GFAULT_T_BAD_LAYER);
    }
    if (x < 0 || x >= W || y < 0 || y >= H) {
        gfault_report(GFAULT_SEG_TILEMAP, GFAULT_T_OUT_OF_RANGE, __LINE__);
        return gfault_wrap(GFAULT_T_OUT_OF_RANGE);
    }
    /* id == 0 keeps its original meaning: clear the cell. GFAULT_T_BAD_TILE
       stays reserved for a future tile-id validation hook. */
    m->base[layer][y][x] = id;
    return 0;
}

static unsigned char map_get(const TileMap* m, int layer, int x, int y) {
    /* Read-side out-of-range is a benign query used by autotile border
       logic, not a render failure: keep the original 0-sentinel contract.
       Hard failures are reported on the write side (map_set). */
    if (layer < 0 || layer >= LAYERS || x < 0 || x >= W || y < 0 || y >= H)
        return 0;
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

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

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

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
