
/* tilemap: layered tile map with 4-neighbour autotiling and per-layer culling. */
#include <stdio.h>
#include <string.h>

#include "diag.h"
#include "canvas.h"

#define MOD "tilemap"

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
    if (layer < 0 || layer >= LAYERS || x < 0 || x >= W || y < 0 || y >= H ||
        DIAG_FAULT(MOD, "map_set_out_of_range"))
        DIAG_FAIL_RETURN(MOD, "map_set_out_of_range", DIAG_E_OUT_OF_RANGE);
    m->base[layer][y][x] = id;
    return DIAG_OK;
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
    if (layer < 0 || layer >= LAYERS || DIAG_FAULT(MOD, "autotile_bad_layer"))
        DIAG_FAIL_RETURN(MOD, "autotile_bad_layer", DIAG_E_INVALID_ARG);
    if (!map_get(m, layer, x, y)) return -1;
    return autotile_mask(m, layer, x, y);
}

/* count visible tiles inside a rect across visible layers */
static int cull_count(const TileMap* m, int x0, int y0, int x1, int y1) {
    if (x0 > x1 || y0 > y1 || DIAG_FAULT(MOD, "cull_invalid_rect"))
        DIAG_FAIL_RETURN(MOD, "cull_invalid_rect", DIAG_E_INVALID_ARG);
    int n = 0;
    for (int l = 0; l < LAYERS; l++) {
        if (!m->layer_visible[l]) continue;
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++)
                if (map_get(m, l, x, y)) n++;
    }
    return n;
}

/* ---------------------------------------------------------------- stage --- */

int tilemap_stage(Canvas* c, int y0, int rows) {
    Canvas s;
    canvas_init(&s);
    TileMap m;
    map_init(&m);
    if (!c || rows <= 0 || rows > CANVAS_H || y0 < 0 || y0 + rows > CANVAS_H ||
        DIAG_FAULT(MOD, "output_overflow"))
        DIAG_FAIL_RETURN(MOD, "output_overflow", DIAG_E_CAPACITY);
    for (int x = 5; x < 10; x++) {
        int rc = map_set(&m, 0, x, 5, 1);
        if (rc < 0) return rc;
    }
    int rc = map_set(&m, 1, 5, 5, 2);
    if (rc < 0) return rc;

    /* draw a window of both layers; '#' solid, '+' layer-1 prop */
    int draw_rows = rows - 2;
    if (draw_rows < 1) draw_rows = 1;
    for (int y = 0; y < draw_rows; y++)
        for (int x = 0; x < 26; x++) {
            int mx = x + 3, my = y + 3;
            if (map_get(&m, 1, mx, my))
                canvas_put(&s, 2 + x, 2 + y, '+');
            else if (map_get(&m, 0, mx, my))
                canvas_put(&s, 2 + x, 2 + y, '#');
        }
    int vis = cull_count(&m, 0, 0, W - 1, H - 1);
    if (vis < 0) return vis;
    int variant = autotile_variant(&m, 0, 7, 5);
    if (variant < 0 && variant != -1) return variant;
    /* stage invariant: the middle of the 5..9 run must stitch E|W */
    if (variant != (2 | 8) || DIAG_FAULT(MOD, "selftest_failed"))
        DIAG_FAIL_RETURN(MOD, "selftest_failed", DIAG_E_INVARIANT);
    char head[CANVAS_W + 1];
    snprintf(head, sizeof head, "tilemap visible=%d layers=%d", vis, LAYERS);
    canvas_text(&s, 1, 0, head);

    canvas_blit_band(c, &s, y0, rows);
    return DIAG_OK;
}

/* ----------------------------------------------------------------- demo --- */
#ifndef TILEMAP_NO_MAIN
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    diag_boot();
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

    /* deliberate rejection probe: suppress the diagnostic it would record */
    int was = diag_suspend();
    int oor_rc = map_set(&m, 0, -1, 0, 1);
    diag_resume(was);
    check("out_of_range_set", (oor_rc < 0 && map_get(&m, 0, -1, 0)) == 0);
    check("bad_layer", map_get(&m, 9, 0, 0) == 0);

    if (failures) diag_fail_point(MOD, "selftest_failed", DIAG_E_INVARIANT);
    printf("all_ok=%d\n", failures == 0);
    diag_flush(stderr);
    return failures == 0 ? 0 : 1;
}
#endif
