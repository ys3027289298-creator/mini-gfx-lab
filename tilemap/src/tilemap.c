/* tilemap: layered tile map with 4-neighbour autotiling and per-layer culling,
   plus baked per-tile ambient occlusion (AO).

   AO model
   --------
   A filled cell's raw AO is built from its four orthogonal neighbours: each
   filled side contributes 0.25. The raw value is scaled by the tile material's
   ao_scale and later multiplied into the tile colour.

   Bake
   ----
   The AO field is baked into 8x8-tile chunks, each padded by one tile on every
   side. Runtime sampling is bilinear between cell centres; because every chunk
   stores identical values in its pad ring, the interpolated field is continuous
   across chunk boundaries (zero jump pixels).

   Validation
   ----------
   The baked field is compared against per-pixel 2D ray-sampled AO. Mean
   absolute error is reported per tile type. */
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#define W 32
#define H 32
#define LAYERS 3

/* AO atlas: CHUNKS_X * CHUNKS_Y chunks of CHUNK x CHUNK tiles, with PAD tiles
   of duplicated neighbour data on each side of every chunk. */
#define CHUNK 8
#define PAD 1
#define CHUNKS_X ((W + CHUNK - 1) / CHUNK)
#define CHUNKS_Y ((H + CHUNK - 1) / CHUNK)
#define CHUNK_STRIDE (CHUNK + 2 * PAD)

#define MAX_MATERIALS 8
#define PIX_PER_TILE 8
#define RAY_COUNT 128
#define RAY_MAX_DIST 3.0f

typedef struct {
    unsigned char base[LAYERS][H][W];   /* 0 = empty, else tile id */
    int layer_visible[LAYERS];
} TileMap;

typedef struct {
    int id;
    float ao_scale;   /* material weight applied to the raw AO term */
} Material;

typedef struct {
    Material mats[MAX_MATERIALS];
    int count;
} MaterialTable;

/* Baked AO field: one float per padded chunk texel. */
typedef struct {
    float tex[LAYERS][CHUNKS_Y][CHUNKS_X][CHUNK_STRIDE][CHUNK_STRIDE];
} AOChunkMap;

/* ------------------------------------------------------------------ */
/* map basics                                                          */
/* ------------------------------------------------------------------ */

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

/* ------------------------------------------------------------------ */
/* materials                                                           */
/* ------------------------------------------------------------------ */

static void materials_init(MaterialTable* mt) {
    memset(mt, 0, sizeof(*mt));
}

static const Material* material_try_find(const MaterialTable* mt, int id) {
    for (int i = 0; i < mt->count; i++)
        if (mt->mats[i].id == id) return &mt->mats[i];
    return NULL;
}

/* Register a material. Negative AO strength is a configuration error and
   aborts immediately; so is a duplicate id or a full table. */
static int material_add(MaterialTable* mt, int id, float ao_scale) {
    if (ao_scale < 0.0f) {
        fprintf(stderr, "material_add: negative ao_scale %g for material %d\n",
                (double)ao_scale, id);
        abort();
    }
    if (mt->count >= MAX_MATERIALS) {
        fprintf(stderr, "material_add: material table full (%d)\n", MAX_MATERIALS);
        abort();
    }
    if (material_try_find(mt, id)) {
        fprintf(stderr, "material_add: duplicate material id %d\n", id);
        abort();
    }
    Material* mat = &mt->mats[mt->count++];
    mat->id = id;
    mat->ao_scale = ao_scale;
    return id;
}

/* Look up a material at bake time. A tile whose id has no material is a
   configuration error and aborts immediately. */
static const Material* material_get(const MaterialTable* mt, int id) {
    const Material* mat = material_try_find(mt, id);
    if (!mat) {
        fprintf(stderr, "material_get: unknown material id %d\n", id);
        abort();
    }
    return mat;
}

/* ------------------------------------------------------------------ */
/* AO strength                                                         */
/* ------------------------------------------------------------------ */

/* Raw AO in [0,1]: each of the four occupied sides contributes 0.25, so a
   cell with all four neighbours filled reaches 1.0. */
static float ao_raw(int mask) {
    int sides = 0;
    for (int bit = 0; bit < 4; bit++) sides += (mask >> bit) & 1;
    return 0.25f * (float)sides;
}

/* AO strength for a cell: raw occupancy weighted by its material. Empty cells
   are unoccluded; filled cells must name a registered material. */
static float ao_strength(const TileMap* m, const MaterialTable* mt,
                         int layer, int x, int y) {
    unsigned char id = map_get(m, layer, x, y);
    if (!id) return 0.0f;
    const Material* mat = material_get(mt, id);
    return ao_raw(autotile_mask(m, layer, x, y)) * mat->ao_scale;
}

/* Multiply baked AO into an 8-bit colour; higher AO means a darker tile. */
static void color_apply_ao(float ao, unsigned char rgb[3]) {
    float k = 1.0f - ao;
    for (int i = 0; i < 3; i++) {
        float v = (float)rgb[i] * k;
        rgb[i] = (unsigned char)(v < 0.0f ? 0.0f : (v > 255.0f ? 255.0f : v + 0.5f));
    }
}

/* ------------------------------------------------------------------ */
/* chunked bake and sampling                                           */
/* ------------------------------------------------------------------ */

/* Bake every layer into padded chunks. Each texel holds the AO strength of
   the map cell at that world position, pads included, so adjacent chunks
   agree exactly on their shared pad ring. */
static void ao_bake(const TileMap* m, const MaterialTable* mt, AOChunkMap* out) {
    memset(out, 0, sizeof(*out));
    for (int l = 0; l < LAYERS; l++) {
        for (int cy = 0; cy < CHUNKS_Y; cy++) {
            for (int cx = 0; cx < CHUNKS_X; cx++) {
                for (int py = 0; py < CHUNK_STRIDE; py++) {
                    for (int px = 0; px < CHUNK_STRIDE; px++) {
                        int wx = cx * CHUNK - PAD + px;
                        int wy = cy * CHUNK - PAD + py;
                        /* Cells outside the map contribute no occlusion. */
                        out->tex[l][cy][cx][py][px] =
                            (wx >= 0 && wx < W && wy >= 0 && wy < H)
                                ? ao_strength(m, mt, l, wx, wy)
                                : 0.0f;
                    }
                }
            }
        }
    }
}

/* Fetch one texel inside chunk (cx,cy) at padded local coordinates. */
static float ao_chunk_texel(const AOChunkMap* aom, int layer, int cx, int cy,
                            int px, int py) {
    return aom->tex[layer][cy][cx][py][px];
}

/* Bilinear sample at a fractional world position (tile units), per chunk.
   Texel values live at cell centres: cell c is centred at c + 0.5.

   The owning chunk is chosen from the continuous position, then the sample is
   mapped into that chunk's padded grid. Near a chunk edge the tap pair spills
   into the pad ring, which holds copies of the neighbouring cells, so a sample
   just left of a boundary and one just right of it interpolate between the same
   two cell values and the field is continuous. */
static float ao_sample(const AOChunkMap* aom, int layer, float x, float y) {
    int cx = (int)floorf(x / (float)CHUNK);
    int cy = (int)floorf(y / (float)CHUNK);
    if (cx < 0) cx = 0;
    if (cx >= CHUNKS_X) cx = CHUNKS_X - 1;
    if (cy < 0) cy = 0;
    if (cy >= CHUNKS_Y) cy = CHUNKS_Y - 1;

    float ux = x - (float)(cx * CHUNK) - 0.5f + (float)PAD;
    float uy = y - (float)(cy * CHUNK) - 0.5f + (float)PAD;
    float x0f = floorf(ux), y0f = floorf(uy);
    float tx = ux - x0f, ty = uy - y0f;
    int px = (int)x0f, py = (int)y0f;
    int px1 = px + 1, py1 = py + 1;
    if (px < 0) px = 0;
    if (py < 0) py = 0;
    if (px1 >= CHUNK_STRIDE) px1 = CHUNK_STRIDE - 1;
    if (py1 >= CHUNK_STRIDE) py1 = CHUNK_STRIDE - 1;

    float c00 = ao_chunk_texel(aom, layer, cx, cy, px, py);
    float c10 = ao_chunk_texel(aom, layer, cx, cy, px1, py);
    float c01 = ao_chunk_texel(aom, layer, cx, cy, px, py1);
    float c11 = ao_chunk_texel(aom, layer, cx, cy, px1, py1);
    float a = c00 * (1.0f - tx) + c10 * tx;
    float b = c01 * (1.0f - tx) + c11 * tx;
    return a * (1.0f - ty) + b * ty;
}

/* ------------------------------------------------------------------ */
/* ray-sampled ground truth                                            */
/* ------------------------------------------------------------------ */

/* March a unit-length direction from (px,py) across the tile grid, skipping
   the starting cell. Any filled cell within maxd blocks the ray. */
static int ray_blocked(const TileMap* m, int layer, float px, float py,
                       float dx, float dy, int sx, int sy, float maxd) {
    int cx = sx, cy = sy;
    int step_x = dx > 0.0f ? 1 : -1;
    int step_y = dy > 0.0f ? 1 : -1;
    float tmax_x, tmax_y, tdelta_x, tdelta_y;

    if (fabsf(dx) < 1e-6f) {
        tmax_x = INFINITY; tdelta_x = INFINITY;
    } else {
        float next = dx > 0.0f ? (float)(cx + 1) : (float)cx;
        tmax_x = (next - px) / dx;
        tdelta_x = fabsf(1.0f / dx);
    }
    if (fabsf(dy) < 1e-6f) {
        tmax_y = INFINITY; tdelta_y = INFINITY;
    } else {
        float next = dy > 0.0f ? (float)(cy + 1) : (float)cy;
        tmax_y = (next - py) / dy;
        tdelta_y = fabsf(1.0f / dy);
    }

    float t = 0.0f;
    while (t <= maxd) {
        if (tmax_x < tmax_y) {
            cx += step_x; t = tmax_x; tmax_x += tdelta_x;
        } else {
            cy += step_y; t = tmax_y; tmax_y += tdelta_y;
        }
        if (t > maxd) break;
        if (cx < 0 || cx >= W || cy < 0 || cy >= H) return 0;  /* left the map */
        if (map_get(m, layer, cx, cy)) return 1;
    }
    return 0;
}

/* Fraction of uniformly distributed 2D directions blocked by filled cells. */
static float ray_ao(const TileMap* m, int layer, float px, float py) {
    int sx = (int)floorf(px), sy = (int)floorf(py);
    int blocked = 0;
    for (int k = 0; k < RAY_COUNT; k++) {
        float a = 6.28318530718f * (float)k / (float)RAY_COUNT;
        if (ray_blocked(m, layer, px, py, cosf(a), sinf(a), sx, sy, RAY_MAX_DIST))
            blocked++;
    }
    return (float)blocked / (float)RAY_COUNT;
}

/* ------------------------------------------------------------------ */
/* tests                                                               */
/* ------------------------------------------------------------------ */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int feq(float a, float b, float eps) {
    return fabsf(a - b) <= eps;
}

/* A child that aborts is the expected "fail immediately" behaviour. */
static int expect_abort(void (*fn)(void)) {
    fflush(NULL);
    pid_t pid = fork();
    if (pid < 0) return 0;
    if (pid == 0) {
        struct rlimit no_core = {0, 0};
        setrlimit(RLIMIT_CORE, &no_core);
        if (!freopen("/dev/null", "w", stderr)) { /* stderr noise is cosmetic */ }
        fn();
        _exit(0);   /* reached only when no failure occurred */
    }
    int status = 0;
    if (waitpid(pid, &status, 0) != pid) return 0;
    return WIFSIGNALED(status) && WTERMSIG(status) == SIGABRT;
}

static void child_negative_scale(void) {
    MaterialTable mt;
    materials_init(&mt);
    material_add(&mt, 1, -0.1f);
}

static void child_missing_material(void) {
    TileMap m;
    MaterialTable mt;
    map_init(&m);
    materials_init(&mt);
    material_add(&mt, 1, 0.5f);
    map_set(&m, 0, 3, 3, 9);   /* tile id 9 has no material */
    AOChunkMap aom;
    ao_bake(&m, &mt, &aom);
}

/* Fixture with solid regions, holes, notches and diagonal-only contacts so
   that all four neighbour directions and all tile types are exercised. */
static void build_fixture(TileMap* m, MaterialTable* mt) {
    map_init(m);
    materials_init(mt);
    material_add(mt, 1, 1.00f);   /* stone  */
    material_add(mt, 2, 0.75f);   /* grass  */
    material_add(mt, 3, 0.50f);   /* wood   */

    /* stone plateau with a notch from the top and scattered holes */
    for (int y = 2; y <= 12; y++)
        for (int x = 2; x <= 12; x++) map_set(m, 0, x, y, 1);
    for (int y = 2; y <= 5; y++)
        for (int x = 7; x <= 9; x++) map_set(m, 0, x, y, 0);
    map_set(m, 0, 4, 9, 0);
    map_set(m, 0, 10, 7, 0);
    map_set(m, 0, 6, 11, 0);

    /* grass block with an inner courtyard: four inner corners */
    for (int y = 16; y <= 27; y++)
        for (int x = 3; x <= 14; x++) map_set(m, 0, x, y, 2);
    for (int y = 20; y <= 23; y++)
        for (int x = 7; x <= 10; x++) map_set(m, 0, x, y, 0);

    /* wood lattice: diagonal-only contacts plus isolated cells */
    for (int y = 17; y <= 26; y++)
        for (int x = 18; x <= 28; x++)
            if (((x - 18) + (y - 17)) % 3 != 0) map_set(m, 0, x, y, 3);
    map_set(m, 0, 26, 5, 3);
    map_set(m, 0, 29, 8, 2);
    map_set(m, 0, 20, 8, 1);
    map_set(m, 0, 21, 9, 1);
    map_set(m, 0, 25, 11, 2);
    map_set(m, 0, 26, 12, 2);

    /* upper layers: AO is baked per layer; only layer 0 is ray-compared */
    for (int y = 4; y <= 20; y++)
        for (int x = 4; x <= 9; x++) map_set(m, 1, x, y, 2);
    map_set(m, 1, 15, 15, 1);
    map_set(m, 2, 8, 8, 3);
    map_set(m, 2, 24, 24, 1);
}

/* Count pixels whose one-sided limits straddle a chunk boundary with a
   visible step. The one-sided test uses a tiny epsilon, so ordinary
   bilinear gradients do not count as jumps. */
static int count_chunk_jumps(const AOChunkMap* aom) {
    const float eps = 1e-4f;
    const float jump = 1e-3f;
    int jumps = 0;
    for (int l = 0; l < LAYERS; l++) {
        for (int cx = 1; cx < CHUNKS_X; cx++) {
            float xb = (float)(cx * CHUNK);
            for (int y = 0; y < H * PIX_PER_TILE; y++) {
                float py = ((float)y + 0.5f) / (float)PIX_PER_TILE;
                float lo = ao_sample(aom, l, xb - eps, py);
                float hi = ao_sample(aom, l, xb + eps, py);
                if (fabsf(hi - lo) > jump) jumps++;
            }
        }
        for (int cy = 1; cy < CHUNKS_Y; cy++) {
            float yb = (float)(cy * CHUNK);
            for (int x = 0; x < W * PIX_PER_TILE; x++) {
                float px = ((float)x + 0.5f) / (float)PIX_PER_TILE;
                float lo = ao_sample(aom, l, px, yb - eps);
                float hi = ao_sample(aom, l, px, yb + eps);
                if (fabsf(hi - lo) > jump) jumps++;
            }
        }
    }
    return jumps;
}

/* Every chunk's pad ring must equal the neighbouring chunk's interior, which
   is what makes the bilinear field continuous. Returns max mismatch. */
static float chunk_pad_mismatch(const AOChunkMap* aom) {
    float worst = 0.0f;
    for (int l = 0; l < LAYERS; l++) {
        for (int cy = 0; cy < CHUNKS_Y; cy++) {
            for (int cx = 0; cx < CHUNKS_X; cx++) {
                for (int k = 0; k < CHUNK; k++) {
                    /* West pad of this chunk vs the same world cell inside
                       the chunk to the west (local column CHUNK - 1 + PAD). */
                    if (cx > 0) {
                        float a = aom->tex[l][cy][cx][k + PAD][0];
                        float b = aom->tex[l][cy][cx - 1][k + PAD][CHUNK - 1 + PAD];
                        if (fabsf(a - b) > worst) worst = fabsf(a - b);
                    }
                    if (cx + 1 < CHUNKS_X) {
                        float a = aom->tex[l][cy][cx][k + PAD][CHUNK_STRIDE - 1];
                        float b = aom->tex[l][cy][cx + 1][k + PAD][PAD];
                        if (fabsf(a - b) > worst) worst = fabsf(a - b);
                    }
                    if (cy > 0) {
                        float a = aom->tex[l][cy][cx][0][k + PAD];
                        float b = aom->tex[l][cy - 1][cx][CHUNK - 1 + PAD][k + PAD];
                        if (fabsf(a - b) > worst) worst = fabsf(a - b);
                    }
                    if (cy + 1 < CHUNKS_Y) {
                        float a = aom->tex[l][cy][cx][CHUNK_STRIDE - 1][k + PAD];
                        float b = aom->tex[l][cy + 1][cx][PAD][k + PAD];
                        if (fabsf(a - b) > worst) worst = fabsf(a - b);
                    }
                }
            }
        }
    }
    return worst;
}

int main(void) {
    /* --- original autotile / culling checks ------------------------- */
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

    /* --- AO strength from the four directions ----------------------- */
    TileMap am;
    MaterialTable mt;
    map_init(&am);
    materials_init(&mt);
    material_add(&mt, 1, 1.0f);
    material_add(&mt, 2, 0.5f);
    map_set(&am, 0, 10, 10, 1);
    check("ao_isolated", feq(ao_strength(&am, &mt, 0, 10, 10), 0.0f, 1e-6f));
    map_set(&am, 0, 11, 10, 1);
    check("ao_one_side", feq(ao_strength(&am, &mt, 0, 10, 10), 0.25f, 1e-6f));
    map_set(&am, 0, 10, 9, 1);
    check("ao_two_sides", feq(ao_strength(&am, &mt, 0, 10, 10), 0.50f, 1e-6f));
    map_set(&am, 0, 9, 10, 1);
    check("ao_three_sides", feq(ao_strength(&am, &mt, 0, 10, 10), 0.75f, 1e-6f));
    map_set(&am, 0, 10, 11, 1);
    check("ao_four_sides", feq(ao_strength(&am, &mt, 0, 10, 10), 1.0f, 1e-6f));
    check("ao_empty_cell", feq(ao_strength(&am, &mt, 0, 20, 20), 0.0f, 1e-6f));

    /* same fully enclosed cell, half-strength material -> 1.0 * 0.5 */
    map_set(&am, 0, 10, 10, 2);
    check("ao_material_scale",
          feq(ao_strength(&am, &mt, 0, 10, 10), 1.0f * 0.5f, 1e-6f));

    /* --- AO applied to colour --------------------------------------- */
    unsigned char rgb[3] = {200, 120, 40};
    color_apply_ao(0.5f, rgb);
    check("color_ao", rgb[0] == 100 && rgb[1] == 60 && rgb[2] == 20);
    rgb[0] = 200; rgb[1] = 120; rgb[2] = 40;
    color_apply_ao(0.0f, rgb);
    check("color_no_ao", rgb[0] == 200 && rgb[1] == 120 && rgb[2] == 40);

    /* --- hard failures ---------------------------------------------- */
    check("fail_negative_scale", expect_abort(child_negative_scale));
    check("fail_missing_material", expect_abort(child_missing_material));

    /* --- bake the fixture ------------------------------------------- */
    TileMap fm;
    MaterialTable fmt;
    build_fixture(&fm, &fmt);
    AOChunkMap aom;
    ao_bake(&fm, &fmt, &aom);

    /* baked tile-centre value must equal the analytic strength */
    int centre_ok = 1;
    for (int y = 0; y < H && centre_ok; y++)
        for (int x = 0; x < W; x++)
            for (int l = 0; l < LAYERS; l++) {
                float want = ao_strength(&fm, &fmt, l, x, y);
                float got = ao_sample(&aom, l, (float)x + 0.5f, (float)y + 0.5f);
                if (!feq(want, got, 1e-6f)) { centre_ok = 0; break; }
            }
    check("ao_bake_centres", centre_ok);

    /* sampled AO modifies the tile colour consistently */
    unsigned char tile_rgb[3] = {180, 180, 180};
    color_apply_ao(ao_sample(&aom, 0, 3.5f, 3.5f), tile_rgb);
    unsigned char want_rgb[3] = {180, 180, 180};
    color_apply_ao(ao_strength(&fm, &fmt, 0, 3, 3), want_rgb);
    check("ao_color_matches_bake",
          tile_rgb[0] == want_rgb[0] && tile_rgb[1] == want_rgb[1] &&
          tile_rgb[2] == want_rgb[2]);

    /* --- chunk continuity ------------------------------------------- */
    float pad_mismatch = chunk_pad_mismatch(&aom);
    check("ao_chunk_pads_match", feq(pad_mismatch, 0.0f, 1e-6f));
    int jumps = count_chunk_jumps(&aom);
    printf("ao_chunk_jump_pixels=%d\n", jumps);
    check("ao_chunk_jump_zero", jumps == 0);

    /* --- per-pixel ray comparison, reported per tile type ----------- */
    /* Mean absolute error between the bilinear baked field and per-pixel
       2D ray AO, grouped by the tile id under each pixel. The ray result is
       in raw units, so it is multiplied by the tile's material scale before
       the comparison, exactly like the baked term. */
    double err_sum[MAX_MATERIALS + 1] = {0};
    long samples[MAX_MATERIALS + 1] = {0};
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int id = map_get(&fm, 0, x, y);
            if (!id) continue;
            float scale = material_get(&fmt, id)->ao_scale;
            for (int py = 0; py < PIX_PER_TILE; py++) {
                for (int px = 0; px < PIX_PER_TILE; px++) {
                    float sx = (float)x + ((float)px + 0.5f) / (float)PIX_PER_TILE;
                    float sy = (float)y + ((float)py + 0.5f) / (float)PIX_PER_TILE;
                    float baked = ao_sample(&aom, 0, sx, sy);
                    float truth = ray_ao(&fm, 0, sx, sy) * scale;
                    err_sum[id] += fabs((double)baked - (double)truth);
                    samples[id]++;
                }
            }
        }
    }
    double worst_mean = 0.0;
    for (int id = 1; id <= MAX_MATERIALS; id++) {
        if (!samples[id]) continue;
        double mean_err = err_sum[id] / (double)samples[id];
        printf("ao_ray_err tile=%d mean=%.4f samples=%ld\n",
               id, mean_err, samples[id]);
        if (mean_err > worst_mean) worst_mean = mean_err;
    }
    printf("ao_ray_err_worst=%.4f\n", worst_mean);
    check("ao_ray_all_types", samples[1] > 0 && samples[2] > 0 && samples[3] > 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
