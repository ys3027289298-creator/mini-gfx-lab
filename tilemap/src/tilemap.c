/* tilemap: layered tile map with 4-neighbour autotiling, per-layer culling,
 * and chunked dirty-region vertex uploads.

 * Geometry lives in one shared vertex buffer split into fixed per-chunk slot
 * ranges. A chunk owns cells [cx*CHUNK, cx*CHUNK+CHUNK) x [cy*CHUNK, ...) and
 * always writes its cells to the same offsets, in the same order, so the index
 * buffer is static and a dirty rebuild never shifts another chunk's geometry.

 * A cell edit dirties its own chunk; if the cell sits on a chunk border the
 * adjacent chunks are dirtied too, because the 4-neighbour autotile mask of a
 * tile across the border changes with it. */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define W 32
#define H 32
#define LAYERS 3

#define CHUNK 8
#define CHUNKS_X (W / CHUNK)
#define CHUNKS_Y (H / CHUNK)
#define NCHUNKS (CHUNKS_X * CHUNKS_Y)
#define CELLS_PER_CHUNK (CHUNK * CHUNK)
#define SLOTS_PER_CHUNK (CELLS_PER_CHUNK * LAYERS)

#define VERTS_PER_CELL 4
#define IDX_PER_CELL 6
#define IDX_PER_CHUNK (SLOTS_PER_CHUNK * IDX_PER_CELL)

typedef struct {
    int16_t x, y;          /* top-left corner in pixels (8 px per tile) */
    uint8_t tile;          /* tile id */
    uint8_t variant;       /* 4-bit autotile mask, 0..15 */
    uint8_t layer;         /* owning layer */
    uint8_t w;             /* 1 = occupied, 0 = empty slot */
} Vertex;

_Static_assert(sizeof(Vertex) == 8, "vertex must be 8 bytes");
#define CELL_VERT_BYTES ((int)(sizeof(Vertex) * VERTS_PER_CELL))
#define CHUNK_VB_BYTES (SLOTS_PER_CHUNK * CELL_VERT_BYTES)
#define VB_BYTES (NCHUNKS * CHUNK_VB_BYTES)
#define IB_BYTES (NCHUNKS * IDX_PER_CHUNK * (int)sizeof(unsigned short))

#define FB_W (W * 8)
#define FB_H (H * 8)

typedef struct {
    unsigned char base[LAYERS][H][W];   /* 0 = empty, else tile id */
    int layer_visible[LAYERS];

    /* CPU mirror of what was last uploaded, one blob per chunk. */
    unsigned char chunk_vb[NCHUNKS][CHUNK_VB_BYTES];

    /* "GPU" buffers. */
    unsigned char gpu_vb[VB_BYTES];
    unsigned short gpu_ib[NCHUNKS * IDX_PER_CHUNK];

    unsigned char dirty[NCHUNKS];

    /* upload statistics, reset per measurement */
    long long uploaded_vb_bytes;
    long long uploaded_ib_bytes;
    int rebuilt_chunks;
} TileMap;

typedef struct {
    unsigned char rgb[FB_H][FB_W][3];
} Framebuffer;

static int g_inject_seam = 0;   /* deliberately forget cross-border marks */
static int g_no_dirty = 0;      /* disable the incremental path */

static void idx_fill(TileMap* m) {
    for (int c = 0; c < NCHUNKS; c++) {
        unsigned short* ib = &m->gpu_ib[c * IDX_PER_CHUNK];
        unsigned int vbase = (unsigned int)c * SLOTS_PER_CHUNK * VERTS_PER_CELL;
        for (int s = 0; s < SLOTS_PER_CHUNK; s++) {
            unsigned int v = vbase + (unsigned int)s * VERTS_PER_CELL;
            unsigned short* q = &ib[s * IDX_PER_CELL];
            q[0] = (unsigned short)(v + 0);
            q[1] = (unsigned short)(v + 1);
            q[2] = (unsigned short)(v + 2);
            q[3] = (unsigned short)(v + 0);
            q[4] = (unsigned short)(v + 2);
            q[5] = (unsigned short)(v + 3);
        }
    }
}

static void chunk_mark(TileMap* m, int cx, int cy) {
    if (cx < 0 || cx >= CHUNKS_X || cy < 0 || cy >= CHUNKS_Y) return;
    m->dirty[cy * CHUNKS_X + cx] = 1;
}

/* Mark the chunk owning (x,y). A cell on a chunk border can change the
 * autotile mask of a neighbour in the next chunk, so mark that chunk too. */
static void mark_cell(TileMap* m, int layer, int x, int y) {
    (void)layer;   /* culling is per chunk, not per layer */
    int cx = x / CHUNK, cy = y / CHUNK;
    int lx = x % CHUNK, ly = y % CHUNK;
    chunk_mark(m, cx, cy);
    if (g_inject_seam) return;
    if (lx == 0) chunk_mark(m, cx - 1, cy);
    if (lx == CHUNK - 1) chunk_mark(m, cx + 1, cy);
    if (ly == 0) chunk_mark(m, cx, cy - 1);
    if (ly == CHUNK - 1) chunk_mark(m, cx, cy + 1);
}

static void mark_all(TileMap* m) {
    for (int i = 0; i < NCHUNKS; i++) m->dirty[i] = 1;
}

static void map_init(TileMap* m) {
    memset(m, 0, sizeof(*m));
    for (int l = 0; l < LAYERS; l++) m->layer_visible[l] = 1;
    idx_fill(m);
    m->uploaded_ib_bytes = IB_BYTES;   /* index buffer uploaded once */
    mark_all(m);
}

static void map_set(TileMap* m, int layer, int x, int y, unsigned char id) {
    if (layer < 0 || layer >= LAYERS || x < 0 || x >= W || y < 0 || y >= H) return;
    m->base[layer][y][x] = id;
    mark_cell(m, layer, x, y);
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
/* geometry build and upload                                           */

static const int CORNER[4][2] = { {0, 0}, {8, 0}, {8, 8}, {0, 8} };

static void put_cell(unsigned char* vb, int slot, int x, int y,
                     int tile, int variant, int layer) {
    Vertex* v = (Vertex*)(void*)(vb + (size_t)slot * CELL_VERT_BYTES);
    for (int k = 0; k < VERTS_PER_CELL; k++) {
        v[k].x = (int16_t)(x * 8 + CORNER[k][0]);
        v[k].y = (int16_t)(y * 8 + CORNER[k][1]);
        v[k].tile = (uint8_t)tile;
        v[k].variant = (uint8_t)variant;
        v[k].layer = (uint8_t)layer;
        v[k].w = 1;
    }
}

/* Fill one chunk's vertex range. Slot layout is fixed: layer-major, then row,
 * then column, so the produced bytes depend only on the map contents. */
static void build_chunk(const TileMap* m, int cx, int cy, unsigned char* vb) {
    memset(vb, 0, (size_t)CHUNK_VB_BYTES);
    int x0 = cx * CHUNK, y0 = cy * CHUNK;
    for (int l = 0; l < LAYERS; l++) {
        for (int ly = 0; ly < CHUNK; ly++) {
            for (int lx = 0; lx < CHUNK; lx++) {
                int x = x0 + lx, y = y0 + ly;
                int id = map_get(m, l, x, y);
                if (!id) continue;
                int variant = autotile_variant(m, l, x, y);
                int slot = l * CELLS_PER_CHUNK + ly * CHUNK + lx;
                put_cell(vb, slot, x, y, id, variant, l);
            }
        }
    }
}

/* Upload only byte runs that differ from the previous upload. */
static void upload_chunk_diff(TileMap* m, int c, const unsigned char* tmp,
                              long long* bytes) {
    unsigned char* shadow = m->chunk_vb[c];
    unsigned char* gpu = m->gpu_vb + (size_t)c * CHUNK_VB_BYTES;
    int i = 0;
    while (i < CHUNK_VB_BYTES) {
        if (tmp[i] == shadow[i]) { i++; continue; }
        int j = i;
        while (j < CHUNK_VB_BYTES && tmp[j] != shadow[j]) j++;
        memcpy(shadow + i, tmp + i, (size_t)(j - i));
        memcpy(gpu + i, tmp + i, (size_t)(j - i));
        *bytes += j - i;
        i = j;
    }
}

static void upload_dirty(TileMap* m) {
    m->uploaded_vb_bytes = 0;
    m->uploaded_ib_bytes = 0;
    m->rebuilt_chunks = 0;
    if (g_no_dirty) {
        /* full rebuild and full re-upload of both buffers */
        for (int c = 0; c < NCHUNKS; c++) {
            unsigned char tmp[CHUNK_VB_BYTES];
            build_chunk(m, c % CHUNKS_X, c / CHUNKS_X, tmp);
            memcpy(m->chunk_vb[c], tmp, sizeof(tmp));
            memcpy(m->gpu_vb + (size_t)c * CHUNK_VB_BYTES, tmp, sizeof(tmp));
        }
        m->uploaded_vb_bytes = VB_BYTES;
        idx_fill(m);
        m->uploaded_ib_bytes = IB_BYTES;
        m->rebuilt_chunks = NCHUNKS;
        memset(m->dirty, 0, sizeof(m->dirty));
        return;
    }
    /* index buffer is static after idx_fill(); only vertex ranges move */
    for (int c = 0; c < NCHUNKS; c++) {
        if (!m->dirty[c]) continue;
        unsigned char tmp[CHUNK_VB_BYTES];
        build_chunk(m, c % CHUNKS_X, c / CHUNKS_X, tmp);
        upload_chunk_diff(m, c, tmp, &m->uploaded_vb_bytes);
        m->rebuilt_chunks++;
        m->dirty[c] = 0;
    }
}

static void stats_reset(TileMap* m) {
    m->uploaded_vb_bytes = 0;
    m->uploaded_ib_bytes = 0;
    m->rebuilt_chunks = 0;
}

/* ------------------------------------------------------------------ */
/* software rasterizer, consumes gpu_vb / gpu_ib                        */

static void cell_color(int tile, int variant, int layer,
                       unsigned char* r, unsigned char* g, unsigned char* b) {
    *r = (unsigned char)(tile * 37 + variant * 11 + layer * 3 + 7);
    *g = (unsigned char)(tile * 17 + variant * 53 + layer * 80 + 23);
    *b = (unsigned char)(tile * 3 + variant * 97 + layer * 31 + 41);
}

static long long edge(int ax, int ay, int bx, int by, int px, int py) {
    return (long long)(bx - ax) * (py - ay) - (long long)(by - ay) * (px - ax);
}

static void fb_tri(Framebuffer* fb, const Vertex* a, const Vertex* b,
                   const Vertex* c, int r, int g, int bl) {
    int ax = a->x * 2, ay = a->y * 2;
    int bx = b->x * 2, by = b->y * 2;
    int cx = c->x * 2, cy = c->y * 2;
    int x0 = (ax < bx ? (ax < cx ? ax : cx) : (bx < cx ? bx : cx)) / 2;
    int x1 = (ax > bx ? (ax > cx ? ax : cx) : (bx > cx ? bx : cx)) / 2;
    int y0 = (ay < by ? (ay < cy ? ay : cy) : (by < cy ? by : cy)) / 2;
    int y1 = (ay > by ? (ay > cy ? ay : cy) : (by > cy ? by : cy)) / 2;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > FB_W - 1) x1 = FB_W - 1;
    if (y1 > FB_H - 1) y1 = FB_H - 1;
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            int px = 2 * x + 1, py = 2 * y + 1;
            long long e0 = edge(ax, ay, bx, by, px, py);
            long long e1 = edge(bx, by, cx, cy, px, py);
            long long e2 = edge(cx, cy, ax, ay, px, py);
            if ((e0 >= 0 && e1 >= 0 && e2 >= 0) ||
                (e0 <= 0 && e1 <= 0 && e2 <= 0)) {
                fb->rgb[y][x][0] = (unsigned char)r;
                fb->rgb[y][x][1] = (unsigned char)g;
                fb->rgb[y][x][2] = (unsigned char)bl;
            }
        }
    }
}

static Vertex vb_vertex(const TileMap* m, unsigned int idx) {
    Vertex v;
    memcpy(&v, m->gpu_vb + (size_t)idx * sizeof(Vertex), sizeof(v));
    return v;
}

static void render(const TileMap* m, Framebuffer* fb) {
    memset(fb, 0, sizeof(*fb));
    for (int l = 0; l < LAYERS; l++) {
        if (!m->layer_visible[l]) continue;
        for (int c = 0; c < NCHUNKS; c++) {
            const unsigned short* ib = &m->gpu_ib[c * IDX_PER_CHUNK +
                                                   l * CELLS_PER_CHUNK * IDX_PER_CELL];
            for (int cell = 0; cell < CELLS_PER_CHUNK; cell++) {
                int slot = l * CELLS_PER_CHUNK + cell;
                unsigned int vbase = (unsigned int)c * SLOTS_PER_CHUNK * VERTS_PER_CELL +
                                     (unsigned int)slot * VERTS_PER_CELL;
                Vertex v0 = vb_vertex(m, vbase);
                if (!v0.w) continue;
                unsigned char r, g, b;
                cell_color(v0.tile, v0.variant, v0.layer, &r, &g, &b);
                for (int t = 0; t < 2; t++) {
                    Vertex a = vb_vertex(m, ib[cell * IDX_PER_CELL + t * 3 + 0]);
                    Vertex bb = vb_vertex(m, ib[cell * IDX_PER_CELL + t * 3 + 1]);
                    Vertex cc = vb_vertex(m, ib[cell * IDX_PER_CELL + t * 3 + 2]);
                    fb_tri(fb, &a, &bb, &cc, r, g, b);
                }
            }
        }
    }
}

/* Compare every GPU cell slot against what the map says it should hold.
 * A missed border mark shows up as a stale variant on the far side of the
 * chunk seam; those cells are counted separately. */
static void seam_scan(const TileMap* m, int* mismatch, int* seams) {
    int bad = 0, seam = 0;
    for (int c = 0; c < NCHUNKS; c++) {
        int cx = c % CHUNKS_X, cy = c / CHUNKS_X;
        for (int s = 0; s < SLOTS_PER_CHUNK; s++) {
            int l = s / CELLS_PER_CHUNK;
            int cell = s % CELLS_PER_CHUNK;
            int lx = cell % CHUNK, ly = cell / CHUNK;
            int x = cx * CHUNK + lx, y = cy * CHUNK + ly;
            int id = map_get(m, l, x, y);
            int want_w = id ? 1 : 0;
            int variant = id ? autotile_variant(m, l, x, y) : 0;
            int cell_bad = 0;
            for (int k = 0; k < VERTS_PER_CELL; k++) {
                Vertex v = vb_vertex(m,
                    (unsigned int)c * SLOTS_PER_CHUNK * VERTS_PER_CELL +
                    (unsigned int)s * VERTS_PER_CELL + (unsigned int)k);
                if (v.w != want_w) { cell_bad = 1; break; }
                if (!id) continue;
                if (v.x != x * 8 + CORNER[k][0] || v.y != y * 8 + CORNER[k][1] ||
                    v.tile != id || v.variant != variant || v.layer != l) {
                    cell_bad = 1;
                    break;
                }
            }
            if (cell_bad) {
                bad++;
                if (lx == 0 || lx == CHUNK - 1 || ly == 0 || ly == CHUNK - 1) seam++;
            }
        }
    }
    *mismatch = bad;
    *seams = seam;
}

/* ------------------------------------------------------------------ */
/* original self-checks (kept byte-identical)                          */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void base_checks(void) {
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
}

/* ------------------------------------------------------------------ */
/* incremental tests                                                   */

static void copy_base(TileMap* dst, const TileMap* src) {
    memcpy(dst->base, src->base, sizeof(dst->base));
    memcpy(dst->layer_visible, src->layer_visible, sizeof(dst->layer_visible));
    mark_all(dst);
}

static Framebuffer* fb_new(void) {
    Framebuffer* fb = (Framebuffer*)malloc(sizeof(Framebuffer));
    if (!fb) { fprintf(stderr, "oom\n"); exit(2); }
    return fb;
}

/* Repeatedly set the same neighbours, upload, then flip the centre cell.  With
 * seam injection the two chunks holding the neighbours are never dirtied, so
 * their autotile variants go stale. */
static void build_seam_case(TileMap* m) {
    map_init(m);
    map_set(m, 0, 15, 8, 1);
    map_set(m, 0, 17, 8, 1);
    map_set(m, 0, 16, 7, 1);
    map_set(m, 0, 16, 9, 1);
    upload_dirty(m);
    stats_reset(m);
    map_set(m, 0, 16, 8, 1);
    upload_dirty(m);
}

static void pixel_compare(const TileMap* dirty, Framebuffer* a,
                          Framebuffer* b, int* mismatch) {
    TileMap* ref = (TileMap*)malloc(sizeof(TileMap));
    if (!ref) { fprintf(stderr, "oom\n"); exit(2); }
    map_init(ref);
    memcpy(ref->base, dirty->base, sizeof(ref->base));
    memcpy(ref->layer_visible, dirty->layer_visible, sizeof(ref->layer_visible));
    mark_all(ref);
    stats_reset(ref);
    upload_dirty(ref);
    render(dirty, a);
    render(ref, b);
    *mismatch = memcmp(a, b, sizeof(Framebuffer)) != 0;
    free(ref);
}

static void test_interior_and_border(void) {
    int fb_mismatch = 0, mism = 0, seams = 0;
    Framebuffer* a = fb_new();
    Framebuffer* b = fb_new();

    /* interior edit: inside chunk (2,2), one neighbour in the same chunk */
    TileMap* mi = (TileMap*)malloc(sizeof(TileMap));
    map_init(mi);
    for (int y = 19; y <= 21; y++)
        for (int x = 19; x <= 21; x++) map_set(mi, 0, x, y, 1);
    upload_dirty(mi);
    stats_reset(mi);
    map_set(mi, 0, 22, 20, 1);
    upload_dirty(mi);
    printf("interior_edit_vb_bytes=%lld\n", mi->uploaded_vb_bytes);
    printf("interior_edit_chunks=%d\n", mi->rebuilt_chunks);
    seam_scan(mi, &mism, &seams);
    pixel_compare(mi, a, b, &fb_mismatch);
    check("interior_pixels_match", fb_mismatch == 0);
    check("interior_seams_clean", seams == 0 && mism == 0);

    /* border edit: tile at (16,8) touches four chunks' autotile masks */
    TileMap* mb = (TileMap*)malloc(sizeof(TileMap));
    map_init(mb);
    map_set(mb, 0, 15, 8, 1);
    map_set(mb, 0, 17, 8, 1);
    map_set(mb, 0, 16, 7, 1);
    map_set(mb, 0, 16, 9, 1);
    upload_dirty(mb);
    stats_reset(mb);
    map_set(mb, 0, 16, 8, 1);
    upload_dirty(mb);
    printf("border_edit_vb_bytes=%lld\n", mb->uploaded_vb_bytes);
    printf("border_edit_chunks=%d\n", mb->rebuilt_chunks);
    seam_scan(mb, &mism, &seams);
    pixel_compare(mb, a, b, &fb_mismatch);
    check("border_pixels_match", fb_mismatch == 0);
    check("border_seams_clean", seams == 0 && mism == 0);

    free(a); free(b); free(mi); free(mb);
}

/* Full-rebuild the reference map in place (same map contents), then compare
 * its buffers and pixels against the incrementally updated map. */
static void compare_against_full(TileMap* dirty, TileMap* ref,
                                 Framebuffer* a, Framebuffer* b,
                                 int* bytes_ok, int* pixels_ok, int* seams_ok) {
    int saved = g_no_dirty;
    g_no_dirty = 1;
    mark_all(ref);
    upload_dirty(ref);
    g_no_dirty = saved;
    if (memcmp(dirty->gpu_vb, ref->gpu_vb, VB_BYTES) != 0) *bytes_ok = 0;
    if (memcmp(dirty->gpu_ib, ref->gpu_ib, IB_BYTES) != 0) *bytes_ok = 0;
    int mism = 0, seams = 0;
    seam_scan(dirty, &mism, &seams);
    if (mism || seams) *seams_ok = 0;
    render(dirty, a);
    render(ref, b);
    if (memcmp(a, b, sizeof(Framebuffer)) != 0) *pixels_ok = 0;
}

static void test_walk(void) {
    Framebuffer* a = fb_new();
    Framebuffer* b = fb_new();
    TileMap* dirty = (TileMap*)malloc(sizeof(TileMap));
    TileMap* ref = (TileMap*)malloc(sizeof(TileMap));
    if (!dirty || !ref) { fprintf(stderr, "oom\n"); exit(2); }
    map_init(dirty);
    map_init(ref);

    /* Two full rows: the walk below runs along y = 8, the top row of the
     * second chunk band, so every step touches a horizontal chunk seam. */
    for (int x = 0; x < W; x++) {
        map_set(dirty, 0, x, 8, 1);
        map_set(dirty, 0, x, 9, 1);
    }
    upload_dirty(dirty);
    copy_base(ref, dirty);
    stats_reset(ref);
    upload_dirty(ref);

    unsigned short ib_before[NCHUNKS * IDX_PER_CHUNK];
    memcpy(ib_before, dirty->gpu_ib, sizeof(ib_before));

    long long walk_bytes = 0;
    int walk_frames = 0, all_pixels = 1, all_seams = 1, all_bytes = 1;
    long long step_min = -1, step_max = 0;
    long long walk_chunks = 0;

    for (int step = 0; step < W; step++) {
        if (step > 0) {
            map_set(dirty, 0, step - 1, 8, 1);
            map_set(ref, 0, step - 1, 8, 1);
        }
        map_set(dirty, 0, step, 8, 2);   /* the character marker */
        map_set(ref, 0, step, 8, 2);
        upload_dirty(dirty);
        long long step_bytes = dirty->uploaded_vb_bytes;
        walk_bytes += step_bytes;
        walk_chunks += dirty->rebuilt_chunks;
        walk_frames++;
        if (step_min < 0 || step_bytes < step_min) step_min = step_bytes;
        if (step_bytes > step_max) step_max = step_bytes;

        compare_against_full(dirty, ref, a, b, &all_bytes, &all_pixels, &all_seams);
    }

    long long full_total = ((long long)VB_BYTES + IB_BYTES) * walk_frames;
    printf("walk_frames=%d\n", walk_frames);
    printf("walk_dirty_vb_bytes=%lld\n", walk_bytes);
    printf("walk_dirty_bytes=%lld\n", walk_bytes);   /* index buffer never re-sent */
    printf("walk_bytes_per_frame_min=%lld\n", step_min);
    printf("walk_bytes_per_frame_max=%lld\n", step_max);
    printf("walk_rebuilt_chunks_total=%lld\n", walk_chunks);
    printf("walk_full_bytes=%lld\n", full_total);
    printf("walk_saved_pct=%.4f\n",
           100.0 * (1.0 - (double)walk_bytes / (double)full_total));

    check("walk_pixels_match", all_pixels);
    check("walk_buffers_match", all_bytes);
    check("walk_seams_clean", all_seams);
    check("index_buffer_static",
          memcmp(ib_before, dirty->gpu_ib, sizeof(ib_before)) == 0);
    check("walk_uploads_something", walk_bytes > 0);

    free(a); free(b); free(dirty); free(ref);
}

/* A chunk can be marked dirty and rebuilt while its bytes end up unchanged
 * (revert within one frame, or an edit that does not affect autotile masks).
 * Such a rebuild must upload nothing. */
static void test_revert_no_upload(void) {
    TileMap* m = (TileMap*)malloc(sizeof(TileMap));
    if (!m) { fprintf(stderr, "oom\n"); exit(2); }
    map_init(m);
    for (int x = 16; x <= 20; x++) map_set(m, 0, x, 16, 3);
    upload_dirty(m);
    stats_reset(m);
    map_set(m, 0, 18, 16, 9);
    map_set(m, 0, 18, 16, 3);   /* back to the uploaded state */
    upload_dirty(m);
    printf("revert_rebuilt_chunks=%d\n", m->rebuilt_chunks);
    printf("revert_vb_bytes=%lld\n", m->uploaded_vb_bytes);
    check("revert_uploads_zero", m->uploaded_vb_bytes == 0);
    free(m);
}

/* Scripted pseudo-random edit storm: every frame must stay equal to a full
 * rebuild, and the total uploaded bytes must stay far below the full path. */
static void test_random_edits(void) {
    Framebuffer* a = fb_new();
    Framebuffer* b = fb_new();
    TileMap* dirty = (TileMap*)malloc(sizeof(TileMap));
    TileMap* ref = (TileMap*)malloc(sizeof(TileMap));
    if (!dirty || !ref) { fprintf(stderr, "oom\n"); exit(2); }
    map_init(dirty);
    map_init(ref);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if (((x * 7 + y * 13) % 5) == 0) map_set(dirty, 0, x, y, 1);
    upload_dirty(dirty);
    copy_base(ref, dirty);

    unsigned int seed = 12345u;
    long long total = 0;
    int frames = 400, all_pixels = 1, all_bytes = 1, all_seams = 1;
    for (int f = 0; f < frames; f++) {
        seed = seed * 1103515245u + 12345u;
        int x = (int)((seed >> 16) % W);
        seed = seed * 1103515245u + 12345u;
        int y = (int)((seed >> 16) % H);
        seed = seed * 1103515245u + 12345u;
        int id = (int)((seed >> 16) % 4);   /* 0 removes the tile */
        int layer = (int)((seed >> 8) % LAYERS);
        map_set(dirty, layer, x, y, (unsigned char)id);
        map_set(ref, layer, x, y, (unsigned char)id);
        upload_dirty(dirty);
        total += dirty->uploaded_vb_bytes;
        compare_against_full(dirty, ref, a, b, &all_bytes, &all_pixels, &all_seams);
    }
    long long full_total = ((long long)VB_BYTES + IB_BYTES) * frames;
    printf("random_edits=%d\n", frames);
    printf("random_dirty_vb_bytes=%lld\n", total);
    printf("random_full_bytes=%lld\n", full_total);
    printf("random_saved_pct=%.4f\n",
           100.0 * (1.0 - (double)total / (double)full_total));
    check("random_pixels_match", all_pixels);
    check("random_buffers_match", all_bytes);
    check("random_seams_clean", all_seams);
    free(a); free(b); free(dirty); free(ref);
}

static void test_no_dirty_path(void) {
    Framebuffer* a = fb_new();
    Framebuffer* b = fb_new();
    TileMap* dirty = (TileMap*)malloc(sizeof(TileMap));
    TileMap* ref = (TileMap*)malloc(sizeof(TileMap));
    if (!dirty || !ref) { fprintf(stderr, "oom\n"); exit(2); }
    map_init(dirty);
    map_init(ref);
    for (int x = 0; x < W; x++) map_set(dirty, 0, x, 8, 1);
    upload_dirty(dirty);
    map_init(ref);
    memcpy(ref->base, dirty->base, sizeof(ref->base));
    mark_all(ref);
    g_no_dirty = 1;
    upload_dirty(ref);
    g_no_dirty = 0;

    int ok = 1;
    for (int step = 0; step < W; step++) {
        map_set(dirty, 0, step, 8, 2);
        map_set(ref, 0, step, 8, 2);
        upload_dirty(dirty);            /* incremental */
        g_no_dirty = 1;
        upload_dirty(ref);              /* full rebuild of the same map */
        g_no_dirty = 0;
        if (memcmp(dirty->gpu_vb, ref->gpu_vb, VB_BYTES) != 0) ok = 0;
        if (memcmp(dirty->gpu_ib, ref->gpu_ib, IB_BYTES) != 0) ok = 0;
        int mism = 0, seams = 0;
        seam_scan(dirty, &mism, &seams);
        int fb_mismatch = 0;
        pixel_compare(dirty, a, b, &fb_mismatch);
        if (mism || seams || fb_mismatch) ok = 0;
    }
    check("no_dirty_byte_identical", ok);
    free(a); free(b); free(dirty); free(ref);
}

/* --seam-scan mode: exit non-zero if the seam checker finds stale geometry. */
static int seam_scan_mode(void) {
    TileMap* m = (TileMap*)malloc(sizeof(TileMap));
    TileMap* ref = (TileMap*)malloc(sizeof(TileMap));
    Framebuffer* a = fb_new();
    Framebuffer* b = fb_new();
    build_seam_case(m);
    int mism = 0, seams = 0;
    seam_scan(m, &mism, &seams);

    map_init(ref);
    memcpy(ref->base, m->base, sizeof(ref->base));
    mark_all(ref);
    upload_dirty(ref);
    render(m, a);
    render(ref, b);
    int pixel_mismatch = memcmp(a, b, sizeof(Framebuffer)) != 0;

    printf("seam_mismatches=%d\n", seams);
    printf("geometry_mismatches=%d\n", mism);
    printf("pixel_mismatch=%d\n", pixel_mismatch);
    printf("inject_seam=%d\n", g_inject_seam);
    return (seams || mism) ? 1 : 0;
}

int main(int argc, char** argv) {
    int legacy = 0, scan = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--legacy") == 0) legacy = 1;
        else if (strcmp(argv[i], "--inject-seam") == 0) g_inject_seam = 1;
        else if (strcmp(argv[i], "--no-dirty") == 0) g_no_dirty = 1;
        else if (strcmp(argv[i], "--seam-scan") == 0) scan = 1;
    }
    if (scan) return seam_scan_mode();
    const char* env = getenv("NO_DIRTY");
    if (env && env[0] && env[0] != '0') g_no_dirty = 1;

    /* With the incremental path disabled the program produces exactly the
     * output of the pre-chunked full-rebuild version. */
    if (g_no_dirty) legacy = 1;

    base_checks();
    if (!legacy) {
        long long full_per_frame = (long long)VB_BYTES + IB_BYTES;
        printf("full_rebuild_vb_bytes=%d\n", VB_BYTES);
        printf("full_rebuild_ib_bytes=%d\n", IB_BYTES);
        printf("full_rebuild_bytes_per_frame=%lld\n", full_per_frame);

        test_interior_and_border();
        test_walk();
        test_revert_no_upload();
        test_random_edits();
        test_no_dirty_path();

        /* injected bug must be visible to the seam checker */
        g_inject_seam = 1;
        TileMap* m = (TileMap*)malloc(sizeof(TileMap));
        build_seam_case(m);
        int mism = 0, seams = 0;
        seam_scan(m, &mism, &seams);
        g_inject_seam = 0;
        printf("injected_seam_cells=%d\n", seams);
        check("injected_seam_detected", seams > 0);
        free(m);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
