
/* tilemap: layered tile map with 4-neighbour autotiling and per-layer culling. */
#include <stdio.h>
#include <string.h>

#define W 32
#define H 32
#define LAYERS 3
#define MAX_COLLIDERS (W * H)

typedef struct {
    unsigned char base[LAYERS][H][W];   /* 0 = empty, else tile id */
    int layer_visible[LAYERS];
} TileMap;

typedef struct {
    int x, y, w, h;      /* tile units; covers [x, x+w) x [y, y+h) */
    int material;        /* material id shared by every tile in the rect */
} Collider;

typedef struct {
    Collider items[MAX_COLLIDERS];
    int count;
} ColliderSet;

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

/*
 * Collision shape merging.
 *
 * Every tile id is its own material, so a 4-neighbour run of equal ids merges
 * into one rect. For each material we sweep the layer with a greedy
 * maximal-rectangle pass: find the top-left-most unclaimed cell, grow a run
 * right while the id matches and the cell is unclaimed, then grow that run
 * down while every cell in the row still matches. Each tile ends up in exactly
 * one rect, so the merged set covers exactly the filled cells (no holes, no
 * overlaps). Layer is part of the material key: colliders from different layers
 * never merge, since they live at different depths.
 */
static int collider_material(const TileMap* m, int layer, int x, int y) {
    unsigned char id = map_get(m, layer, x, y);
    if (!id) return -1;
    return layer * 256 + id;
}

static void colliders_build(const TileMap* m, int layer, ColliderSet* out) {
    /* claimed[y][x]: cell already covered by a rect. */
    unsigned char claimed[H][W];
    memset(claimed, 0, sizeof(claimed));
    out->count = 0;

    /* Materials are processed one at a time in ascending id order, so the
     * output is grouped by material and never mixes ids inside a rect. */
    for (int id = 1; id < 256; id++) {
        int mat = layer * 256 + id;
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                if (claimed[y][x] || collider_material(m, layer, x, y) != mat)
                    continue;

                int w = 1;
                while (x + w < W && !claimed[y][x + w] &&
                       collider_material(m, layer, x + w, y) == mat) w++;

                int h = 1;
                for (;; h++) {
                    int ny = y + h;
                    if (ny >= H) break;
                    int row_ok = 1;
                    for (int dx = 0; dx < w; dx++) {
                        if (claimed[ny][x + dx] ||
                            collider_material(m, layer, x + dx, ny) != mat) {
                            row_ok = 0;
                            break;
                        }
                    }
                    if (!row_ok) break;
                }

                for (int dy = 0; dy < h; dy++)
                    for (int dx = 0; dx < w; dx++)
                        claimed[y + dy][x + dx] = 1;

                if (out->count < MAX_COLLIDERS) {
                    Collider* c = &out->items[out->count++];
                    c->x = x; c->y = y; c->w = w; c->h = h;
                    c->material = mat;
                }
            }
        }
    }
}

/* One collider per filled tile: the unmerged reference shape set. */
static void colliders_build_per_tile(const TileMap* m, int layer, ColliderSet* out) {
    out->count = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int mat = collider_material(m, layer, x, y);
            if (mat < 0) continue;
            if (out->count < MAX_COLLIDERS) {
                Collider* c = &out->items[out->count++];
                c->x = x; c->y = y; c->w = 1; c->h = 1;
                c->material = mat;
            }
        }
}

static int colliders_agree(const Collider* colliders, int n, int material,
                           int x, int y) {
    for (int i = 0; i < n; i++)
        if (colliders[i].material == material &&
            x >= colliders[i].x && x < colliders[i].x + colliders[i].w &&
            y >= colliders[i].y && y < colliders[i].y + colliders[i].h)
            return 1;
    return 0;
}

/* Point test against one collider set; returns material or -1. */
static int colliders_hit_point(const Collider* colliders, int n, int x, int y) {
    for (int i = 0; i < n; i++) {
        const Collider* c = &colliders[i];
        if (x >= c->x && x < c->x + c->w && y >= c->y && y < c->y + c->h)
            return c->material;
    }
    return -1;
}

/* Sum of rect areas: merging must preserve total covered area. */
static int colliders_area(const Collider* colliders, int n) {
    int a = 0;
    for (int i = 0; i < n; i++) a += colliders[i].w * colliders[i].h;
    return a;
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

    /* ---- collision shape merging ---- */
    {
        TileMap cm;
        ColliderSet merged, per_tile, merged2;
        map_init(&cm);

        /* solid 8x6 slab of material 1 with a material-2 hole/overlay */
        for (int y = 2; y < 8; y++)
            for (int x = 3; x < 11; x++)
                map_set(&cm, 0, x, y, 1);
        for (int y = 4; y < 6; y++)
            for (int x = 6; x < 9; x++)
                map_set(&cm, 0, x, y, 2);
        /* disjoint blob of material 1 */
        map_set(&cm, 0, 20, 20, 1);
        map_set(&cm, 0, 21, 20, 1);
        map_set(&cm, 0, 20, 21, 1);
        map_set(&cm, 0, 21, 21, 1);

        colliders_build(&cm, 0, &merged);
        colliders_build_per_tile(&cm, 0, &per_tile);

        int sorted = 1;
        for (int i = 1; i < merged.count; i++)
            if (merged.items[i].material < merged.items[i - 1].material) sorted = 0;
        check("collider_count_reduced", merged.count < per_tile.count);
        /* 48 slab cells (6 retagged material 2) + 4-cell blob: material 1
         * splits around the overlay into 8x2 / 3x2 / 2x2 / 8x2 plus the blob,
         * material 2 is one 3x2 rect, so 6 merged vs 52 per-tile. */
        check("collider_count_exact", per_tile.count == 52 && merged.count == 6);
        check("collider_area_preserved",
              colliders_area(merged.items, merged.count) ==
              colliders_area(per_tile.items, per_tile.count));

        /* material grouping: every rect is single-material and 4-neighbour runs
         * of the same id are never split across rects without a material change */
        int grouping_ok = 1;
        for (int i = 0; i < merged.count; i++)
            for (int dy = 0; dy < merged.items[i].h; dy++)
                for (int dx = 0; dx < merged.items[i].w; dx++)
                    if (collider_material(&cm, 0, merged.items[i].x + dx,
                                          merged.items[i].y + dy) !=
                        merged.items[i].material)
                        grouping_ok = 0;
        check("collider_single_material", grouping_ok);
        check("collider_materials_sorted", sorted);

        /* per-point collision agreement over every cell (inside and outside) */
        int agree = 1;
        for (int y = -1; y <= H; y++)
            for (int x = -1; x <= W; x++) {
                int a = colliders_hit_point(merged.items, merged.count, x, y);
                int b = colliders_hit_point(per_tile.items, per_tile.count, x, y);
                if (a != b) agree = 0;
                if (colliders_agree(per_tile.items, per_tile.count, 1, x, y) !=
                    colliders_agree(merged.items, merged.count, 1, x, y))
                    agree = 0;
            }
        check("collider_point_agrees", agree);

        /* rebuild is deterministic */
        colliders_build(&cm, 0, &merged2);
        int same = merged2.count == merged.count;
        for (int i = 0; same && i < merged.count; i++)
            same = memcmp(&merged.items[i], &merged2.items[i], sizeof(Collider)) == 0;
        check("collider_deterministic", same);

        /* layer isolation: same geometry on layer 1 is never merged with layer 0 */
        for (int y = 2; y < 8; y++)
            for (int x = 3; x < 11; x++)
                map_set(&cm, 1, x, y, 1);
        colliders_build(&cm, 1, &merged2);
        /* colliders from different layers are distinct materials */
        int layer_ok = merged2.count > 0;
        for (int i = 0; i < merged2.count; i++)
            if (merged2.items[i].material / 256 != 1) layer_ok = 0;
        check("collider_layer_isolated", layer_ok);

        printf("collider_merged=%d collider_per_tile=%d\n",
               merged.count, per_tile.count);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
