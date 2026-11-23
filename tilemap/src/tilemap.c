
/* tilemap: layered tile map with 4-neighbour autotiling and per-layer culling. */
#include <stdio.h>
#include <string.h>
#include "diag.h"

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

static int failures = 0;
static Diag g_diag;

#if DIAG
/* evidence material for the run log: one record per check */
#define LOG_CAP 16
static const char* log_names[LOG_CAP];
static int log_ok[LOG_CAP];
static int log_n;

static void log_record(const char* name, int ok) {
    if (log_n < LOG_CAP) { log_names[log_n] = name; log_ok[log_n] = ok; log_n++; }
}

static void emit_run_log(void) {
    fprintf(stderr, "[tilemap] run log (%d checks):\n", log_n);
    for (int i = 0; i < log_n; i++)
        fprintf(stderr, "[tilemap] %02d %-20s %s\n", i, log_names[i], log_ok[i] ? "ok" : "FAIL");
    fprintf(stderr, "[tilemap] summary: %d/%d failed\n", g_diag.failed, g_diag.checks);
}
#endif

static void check(const char* name, int ok) {
#if DIAG
    diag_check(&g_diag, ok);
    log_record(name, ok);
    DIAG_DRAW("%s=%d\n", name, ok ? 1 : 0);
#else
    printf("%s=%d\n", name, ok ? 1 : 0);
#endif
    if (!ok) failures++;
}

int main(void) {
    diag_begin(&g_diag, "tilemap");
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

#if DIAG
    diag_check(&g_diag, !diag_forced(&g_diag));
    log_record("no_injected_fault", !g_diag.forced);
    DIAG_DRAW("all_ok=%d\n", failures == 0);
    if (diag_ok(&g_diag)) { diag_draw_flush(); return 0; }
    diag_drop_notice(&g_diag);
    emit_run_log();
    return DIAG_EXIT_DROP;
#else
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
#endif
}
