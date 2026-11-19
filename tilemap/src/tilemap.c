
/* tilemap: layered tile map with 4-neighbour autotiling, per-layer culling,
   and a per-cell autotile variant cache. A variant depends only on the cell
   itself and its 4 neighbours, so an edit invalidates at most 5 cells; cache
   hits are served directly, misses recompute just that cell. */
#include <stdio.h>
#include <string.h>

#define W 32
#define H 32
#define LAYERS 3
#define CELLS (LAYERS * H * W)

typedef struct {
    unsigned char base[LAYERS][H][W];    /* 0 = empty, else tile id */
    signed char variant[LAYERS][H][W];   /* cached variant: -1 empty, 0..15 = N/E/S/W mask */
    unsigned char vvalid[LAYERS][H][W];  /* 1 = variant entry is cache-valid */
    int layer_visible[LAYERS];
    long cache_rebuilds;                 /* cell recomputes served via the cache path */
    long full_rebuilds;                  /* cell recomputes served via the full path */
    long cache_invalidations;            /* valid->dirty transitions (deduped) */
} TileMap;

static void map_init(TileMap* m) {
    memset(m, 0, sizeof(*m));
    for (int l = 0; l < LAYERS; l++) m->layer_visible[l] = 1;
}

static void map_set(TileMap* m, int layer, int x, int y, unsigned char id);

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

/* pure variant computation straight from base tiles, no cache */
static int compute_variant(const TileMap* m, int layer, int x, int y) {
    if (!map_get(m, layer, x, y)) return -1;
    return autotile_mask(m, layer, x, y);
}

/* mark one cell dirty; out-of-range and already-dirty cells cost nothing */
static void invalidate_cell(TileMap* m, int layer, int x, int y) {
    if (layer < 0 || layer >= LAYERS || x < 0 || x >= W || y < 0 || y >= H) return;
    if (!m->vvalid[layer][y][x]) return;
    m->vvalid[layer][y][x] = 0;
    m->cache_invalidations++;
}

/* a variant reads only the cell and its 4 neighbours, so an edit dirties
   exactly that dependency set: at most 5 cells, fewer at the map border */
static void invalidate_around(TileMap* m, int layer, int x, int y) {
    invalidate_cell(m, layer, x, y);
    invalidate_cell(m, layer, x, y - 1);
    invalidate_cell(m, layer, x + 1, y);
    invalidate_cell(m, layer, x, y + 1);
    invalidate_cell(m, layer, x - 1, y);
}

static void map_set(TileMap* m, int layer, int x, int y, unsigned char id) {
    if (layer < 0 || layer >= LAYERS || x < 0 || x >= W || y < 0 || y >= H) return;
    int was_filled = m->base[layer][y][x] != 0;
    m->base[layer][y][x] = id;
    /* masks only test filled/not-filled: same fill state changes no variant */
    if (was_filled != (id != 0)) invalidate_around(m, layer, x, y);
}

/* cached variant accessor: hit returns the stored value, miss recomputes
   only this one cell */
static int autotile_variant(TileMap* m, int layer, int x, int y) {
    if (layer < 0 || layer >= LAYERS || x < 0 || x >= W || y < 0 || y >= H) return -1;
    if (!m->vvalid[layer][y][x]) {
        m->variant[layer][y][x] = (signed char)compute_variant(m, layer, x, y);
        m->vvalid[layer][y][x] = 1;
        m->cache_rebuilds++;
    }
    return m->variant[layer][y][x];
}

/* read every variant through the cache; returns cells recomputed (misses) */
static int sweep_variants(TileMap* m) {
    long start = m->cache_rebuilds;
    for (int l = 0; l < LAYERS; l++)
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                (void)autotile_variant(m, l, x, y);
    return (int)(m->cache_rebuilds - start);
}

/* full recompute path: every cell rebuilt, no cache lookup */
static int rebuild_all(TileMap* m) {
    int n = 0;
    for (int l = 0; l < LAYERS; l++)
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                m->variant[l][y][x] = (signed char)compute_variant(m, l, x, y);
                m->vvalid[l][y][x] = 1;
                m->full_rebuilds++;
                n++;
            }
    return n;
}

/* independent check: each cached entry equals a fresh from-base recomputation */
static int cache_consistent(const TileMap* m) {
    for (int l = 0; l < LAYERS; l++)
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                if (!m->vvalid[l][y][x]) return 0;
                if (m->variant[l][y][x] != (signed char)compute_variant(m, l, x, y))
                    return 0;
            }
    return 1;
}

/* per-cell variant equality of two maps */
static int variants_identical(const TileMap* a, const TileMap* b) {
    return memcmp(a->variant, b->variant, sizeof(a->variant)) == 0;
}

static int count_valid(const TileMap* m, int layer) {
    int n = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            n += m->vvalid[layer][y][x];
    return n;
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

/* deterministic LCG so the fuzz sequence is reproducible */
static unsigned long rng_state = 0x12345678u;
static unsigned long rng_next(void) {
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
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

    /* ---- original behaviour (variant now served via cache) ---- */
    check("single_mask", autotile_mask(&m, 0, 5, 5) == 2);          /* only E */
    check("middle_mask", autotile_mask(&m, 0, 7, 5) == (2 | 8));      /* E and W */
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

    /* ---- cache hit / miss ---- */
    map_set(&m, 0, 10, 10, 1);
    long r0 = m.cache_rebuilds;
    int v_first = autotile_variant(&m, 0, 10, 10);   /* cold entry: miss */
    long r1 = m.cache_rebuilds;
    int v_again = autotile_variant(&m, 0, 10, 10);   /* warm entry: hit */
    long r2 = m.cache_rebuilds;
    check("cache_miss_rebuilds_one", r1 - r0 == 1);
    check("cache_hit_serves_cached", r2 == r1 && v_again == v_first);

    /* cold start fills the whole cache; a warm sweep rebuilds nothing */
    TileMap cold;
    map_init(&cold);
    check("cold_start_fills_cache", sweep_variants(&cold) == CELLS);
    check("warm_sweep_no_rebuild", sweep_variants(&cold) == 0);

    /* warm m fully so later invalidation counts are exact */
    sweep_variants(&m);

    /* ---- precise invalidation: interior edit dirties self + 4 neighbours ---- */
    long i0 = m.cache_invalidations;
    map_set(&m, 0, 10, 10, 0);                          /* filled -> empty */
    check("invalidate_interior_5", m.cache_invalidations - i0 == 5);
    long r3 = m.cache_rebuilds;
    int single_rebuilt = sweep_variants(&m);
    check("sweep_rebuilds_only_dirty", single_rebuilt == 5 && m.cache_rebuilds - r3 == 5);
    check("dirty_cells_match_reference", cache_consistent(&m));

    /* corner cell: self + only 2 in-bounds neighbours */
    long i1 = m.cache_invalidations;
    map_set(&m, 0, 0, 0, 1);
    check("invalidate_corner_3", m.cache_invalidations - i1 == 3);

    /* no-op write, and an id swap with unchanged fill state, dirty nothing */
    long i2 = m.cache_invalidations;
    map_set(&m, 0, 25, 25, 0);                         /* empty stays empty */
    check("noop_write_no_invalidation", m.cache_invalidations == i2);
    map_set(&m, 0, 7, 7, 3);
    long i3 = m.cache_invalidations;
    map_set(&m, 0, 7, 7, 4);
    check("same_fill_swap_no_invalidation", m.cache_invalidations == i3);

    /* layer isolation: editing layer 1 must not touch layer 0 entries */
    int valid_l0_before = count_valid(&m, 0);
    map_set(&m, 1, 10, 10, 1);
    check("layer_isolation", count_valid(&m, 0) == valid_l0_before);

    /* ---- cached map vs full-rebuild map over a random edit sequence ---- */
    TileMap mc, mf;
    map_init(&mc);
    map_init(&mf);

    rng_state = 0x12345678u;
    for (int i = 0; i < 80; i++) {
        int l = (int)(rng_next() % LAYERS);
        int x = (int)(rng_next() % W);
        int y = (int)(rng_next() % H);
        unsigned char id = (unsigned char)(1 + rng_next() % 7);
        map_set(&mc, l, x, y, id);
        map_set(&mf, l, x, y, id);
    }
    sweep_variants(&mc);
    int per_full = rebuild_all(&mf);
    check("initial_variants_match", variants_identical(&mc, &mf) && cache_consistent(&mc));

    long cached_cells = 0, full_cells = 0;
    int mismatches = 0;
    enum { STEPS = 200 };
    for (int s = 0; s < STEPS; s++) {
        int l = (int)(rng_next() % LAYERS);
        int x = (int)(rng_next() % W);
        int y = (int)(rng_next() % H);
        unsigned char id = (rng_next() & 1u) ? (unsigned char)(1 + rng_next() % 7) : 0;

        map_set(&mc, l, x, y, id);
        map_set(&mf, l, x, y, id);

        cached_cells += sweep_variants(&mc);    /* cache path: only dirty cells recompute */
        full_cells += rebuild_all(&mf);         /* full path: every cell recomputes */

        if (!cache_consistent(&mc) || !variants_identical(&mc, &mf))
            mismatches++;
    }

    check("per_cell_equivalence_all_steps", mismatches == 0);
    check("final_matches_reference", cache_consistent(&mc) && variants_identical(&mc, &mf));

    /* ---- rebuilt-cell comparison ---- */
    printf("edit_steps=%d\n", STEPS);
    printf("cells_per_full_rebuild=%d\n", per_full);
    printf("single_edit_rebuild_cached=%d\n", single_rebuilt);
    printf("single_edit_rebuild_full=%d\n", per_full);
    printf("rebuild_cached=%ld\n", cached_cells);
    printf("rebuild_full=%ld\n", full_cells);
    printf("rebuild_ratio_pct=%ld\n", cached_cells * 100 / full_cells);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
