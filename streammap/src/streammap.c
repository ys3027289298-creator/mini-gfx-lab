/* streammap: out-of-core chunked tile world.

   The map lives in a tile store, not in memory. Only a small LRU cache of
   chunks is resident; every query goes through an accessor that clips to the
   world and loads the owning chunk on demand. Because reads cross chunk
   borders in absolute world coordinates, a query result never depends on
   which chunks happen to be resident: autotiled seam cells, rect counts and
   per-cell values match a whole-map load at any cache size, including zero.

   Peak memory is the cache plus the descriptor: O(resident chunks), with no
   term for world size. The store stays on disk. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define T        16      /* chunk edge, cells */
#define LAYERS   2       /* tile layers per chunk */
#define CAP_MAX  64      /* upper bound for the test cache */

typedef struct { int cx, cy; } ChunkRef;

/* Tile store ---------------------------------------------------------- */

typedef struct {
    uint32_t magic;      /* "TMP1" */
    int32_t w, h, layers;
} StoreHeader;

typedef struct {
    int w, h;            /* world size, cells */
    int cw, ch;          /* world size, chunks */
    long data_off;
    FILE* f;             /* tile store, one [layer][y][x] plane per layer */
} World;

/* Deterministic content: about a third of the cells are filled, ids 1..7. */
static unsigned char gen_tile(int layer, int x, int y) {
    uint32_t v = (uint32_t)x * 73856093u ^ (uint32_t)y * 19349663u ^ (uint32_t)layer * 83492791u;
    v ^= v >> 13; v *= 0x5bd1e995u; v ^= v >> 15;
    return (v % 100u) < 35u ? (unsigned char)(1u + (v >> 8) % 7u) : 0;
}

/* Generate the store in fixed-size runs: building the map must not itself
   allocate anything proportional to the map. */
static int world_create(World* w, int W, int H) {
    if (W <= 0 || H <= 0) return -1;
    FILE* f = tmpfile();
    if (!f) return -1;
    StoreHeader hd = { 0x31504d54u, W, H, LAYERS };
    if (fwrite(&hd, sizeof hd, 1, f) != 1) { fclose(f); return -1; }
    unsigned char buf[256];
    for (int l = 0; l < LAYERS; l++)
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; ) {
                int n = W - x < (int)sizeof buf ? W - x : (int)sizeof buf;
                for (int i = 0; i < n; i++) buf[i] = gen_tile(l, x + i, y);
                if (fwrite(buf, 1, (size_t)n, f) != (size_t)n) {
                    fclose(f); return -1;
                }
                x += n;
            }
    fflush(f);
    w->w = W; w->h = H;
    w->cw = (W + T - 1) / T;
    w->ch = (H + T - 1) / T;
    w->data_off = (long)sizeof hd;
    w->f = f;
    return 0;
}

static void world_destroy(World* w) {
    if (w->f) fclose(w->f);
    w->f = NULL;
}

static int in_world(const World* w, int x, int y) {
    return x >= 0 && y >= 0 && x < w->w && y < w->h;
}

/* Single-cell store read. Used when no chunk can be held (cache unusable),
   so results stay exact even then; normally chunk_read is the read path. */
static unsigned char world_tile(const World* w, int layer, int x, int y) {
    if (!w->f || layer < 0 || layer >= LAYERS || !in_world(w, x, y)) return 0;
    long off = w->data_off + ((long)layer * w->h + y) * w->w + x;
    if (fseek(w->f, off, SEEK_SET) != 0) return 0;
    int c = fgetc(w->f);
    return c == EOF ? 0 : (unsigned char)c;
}

/* One fread per row: chunk rows are contiguous in the store. Edge chunks are
   partial; the rest of the slot is zeroed, though border reads never reach it
   because every accessor clips to the world before mapping a cell to a chunk. */
static int chunk_read(const World* w, ChunkRef r, unsigned char out[LAYERS][T][T]) {
    int x0 = r.cx * T, y0 = r.cy * T;
    int cols = w->w - x0, rows = w->h - y0;
    if (cols > T) cols = T;
    if (rows > T) rows = T;
    if (cols <= 0 || rows <= 0) return -1;
    memset(out, 0, sizeof(unsigned char) * LAYERS * T * T);
    for (int l = 0; l < LAYERS; l++)
        for (int y = 0; y < rows; y++) {
            long off = w->data_off + ((long)l * w->h + (y0 + y)) * w->w + x0;
            if (fseek(w->f, off, SEEK_SET) != 0) return -1;
            if (fread(out[l][y], 1, (size_t)cols, w->f) != (size_t)cols) return -1;
        }
    return 0;
}

/* Residency cache ----------------------------------------------------- */

typedef struct {
    unsigned char base[LAYERS][T][T];
    ChunkRef ref;
    int loaded;
    uint64_t age;        /* LRU stamp; bumped on every touch */
} Chunk;

typedef struct {
    const World* world;
    Chunk* slots;        /* exactly cap entries: memory is chunk-count shaped */
    int cap, resident;
    long loads, evicts;
    uint64_t clock;
} Cache;

static size_t cache_bytes(const Cache* c) {
    return sizeof(Cache) + (size_t)c->cap * sizeof(Chunk);
}

static int same_ref(ChunkRef a, ChunkRef b) { return a.cx == b.cx && a.cy == b.cy; }

static int cache_open(Cache* c, const World* w, int cap) {
    memset(c, 0, sizeof *c);
    c->world = w;
    if (cap < 0) cap = 0;
    if (cap > CAP_MAX) cap = CAP_MAX;
    c->cap = cap;
    if (cap > 0) {
        c->slots = (Chunk*)calloc((size_t)cap, sizeof(Chunk));
        if (!c->slots) { c->cap = 0; return -1; }
    }
    return 0;
}

static void cache_close(Cache* c) {
    free(c->slots);
    c->slots = NULL;
    c->cap = c->resident = 0;
}

static int cache_slot(const Cache* c, ChunkRef ref) {
    for (int i = 0; i < c->cap; i++)
        if (c->slots[i].loaded && same_ref(c->slots[i].ref, ref)) return i;
    return -1;
}

static void cache_drop(Cache* c, int slot) {
    Chunk* s = &c->slots[slot];
    if (!s->loaded) return;
    s->loaded = 0;
    c->resident--;
    c->evicts++;
}

/* Load by chunk index if absent, evicting the coldest chunk when full. */
static Chunk* cache_get(Cache* c, ChunkRef ref) {
    if (c->cap <= 0) return NULL;
    int i = cache_slot(c, ref);
    if (i >= 0) {
        c->slots[i].age = ++c->clock;
        return &c->slots[i];
    }
    if (c->resident < c->cap) {
        for (i = 0; i < c->cap && c->slots[i].loaded; i++) {}
    } else {
        i = 0;
        for (int j = 1; j < c->cap; j++)
            if (c->slots[j].age < c->slots[i].age) i = j;
        if (!c->slots[i].loaded) return NULL;
        cache_drop(c, i);
    }
    if (i < 0 || i >= c->cap) return NULL;
    Chunk* s = &c->slots[i];
    if (chunk_read(c->world, ref, s->base) != 0) return NULL;
    s->ref = ref;
    s->loaded = 1;
    s->age = ++c->clock;
    c->resident++;
    c->loads++;
    return s;
}

/* Explicit index-based unload, e.g. when a region leaves view. */
static int cache_unload(Cache* c, ChunkRef ref) {
    int i = cache_slot(c, ref);
    if (i < 0) return 0;
    cache_drop(c, i);
    return 1;
}

static int cache_resident(const Cache* c, ChunkRef ref) {
    return cache_slot(c, ref) >= 0;
}

/* Accessors -----------------------------------------------------------
   One signature for both backends, so the query code below is literally the
   same for a streamed cache and a whole-map buffer. */

typedef unsigned char (*TileFn)(void* ctx, int layer, int x, int y);

typedef struct { const World* w; Cache* cache; } Stream;

static unsigned char stream_tile(void* ctx, int layer, int x, int y) {
    Stream* s = (Stream*)ctx;
    const World* w = s->w;
    if (layer < 0 || layer >= LAYERS || !in_world(w, x, y)) return 0;
    if (!s->cache || s->cache->cap <= 0) return world_tile(w, layer, x, y);
    ChunkRef ref = { x / T, y / T };        /* in-world: floor division is a shift */
    Chunk* ch = cache_get(s->cache, ref);
    if (!ch) return world_tile(w, layer, x, y);     /* stay exact without a cache */
    return ch->base[layer][y - ref.cy * T][x - ref.cx * T];
}

typedef struct { const unsigned char* data; const World* w; } Whole;

static unsigned char whole_tile(void* ctx, int layer, int x, int y) {
    Whole* h = (Whole*)ctx;
    if (layer < 0 || layer >= LAYERS || !in_world(h->w, x, y)) return 0;
    return h->data[((size_t)layer * h->w->h + y) * h->w->w + x];
}

/* Queries ------------------------------------------------------------- */

/* 4-bit mask: 1 = neighbour filled (N,E,S,W). Neighbours are absolute world
   cells, so a tile on a chunk border sees the other chunk's content and a
   tile on the world border sees empty space, exactly like the whole map. */
static int neighbor_mask(TileFn get, void* ctx, int layer, int x, int y) {
    int m = 0;
    if (get(ctx, layer, x, y - 1)) m |= 1;
    if (get(ctx, layer, x + 1, y)) m |= 2;
    if (get(ctx, layer, x, y + 1)) m |= 4;
    if (get(ctx, layer, x - 1, y)) m |= 8;
    return m;
}

static int variant_at(TileFn get, void* ctx, int layer, int x, int y) {
    if (!get(ctx, layer, x, y)) return -1;
    return neighbor_mask(get, ctx, layer, x, y);
}

static int clip_rect(const World* w, int* x0, int* y0, int* x1, int* y1) {
    if (*x0 > *x1) { int t = *x0; *x0 = *x1; *x1 = t; }
    if (*y0 > *y1) { int t = *y0; *y0 = *y1; *y1 = t; }
    if (*x0 < 0) *x0 = 0;
    if (*y0 < 0) *y0 = 0;
    if (*x1 >= w->w) *x1 = w->w - 1;
    if (*y1 >= w->h) *y1 = w->h - 1;
    return *x0 <= *x1 && *y0 <= *y1;
}

/* Clipped scan through any accessor. */
static int count_rect(TileFn get, void* ctx, const World* w, int layer,
                      int x0, int y0, int x1, int y1) {
    if (!clip_rect(w, &x0, &y0, &x1, &y1)) return 0;
    int n = 0;
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            if (get(ctx, layer, x, y)) n++;
    return n;
}

/* Cross-chunk query: clip once, then count per clipped chunk and sum. No cell
   is counted twice and no chunk outside the world is ever opened, even when
   the requested rect extends past the map. */
static int count_rect_stream(Stream* s, int layer, int x0, int y0, int x1, int y1) {
    const World* w = s->w;
    if (!clip_rect(w, &x0, &y0, &x1, &y1)) return 0;
    int n = 0;
    for (int cy = y0 / T; cy <= y1 / T; cy++)
        for (int cx = x0 / T; cx <= x1 / T; cx++) {
            int ax = cx * T > x0 ? cx * T : x0;
            int ay = cy * T > y0 ? cy * T : y0;
            int bx = cx * T + T - 1 < x1 ? cx * T + T - 1 : x1;
            int by = cy * T + T - 1 < y1 ? cy * T + T - 1 : y1;
            n += count_rect(stream_tile, s, w, layer, ax, ay, bx, by);
        }
    return n;
}

/* Whole-image backend, the reference the stream must match. */
static int whole_load(Whole* h, const World* w) {
    size_t n = (size_t)w->w * w->h * LAYERS;
    h->w = w;
    h->data = (const unsigned char*)malloc(n);
    if (!h->data) return -1;
    if (fseek(w->f, w->data_off, SEEK_SET) != 0) return -1;
    return fread((void*)h->data, 1, n, w->f) == n ? 0 : -1;
}

static void whole_free(Whole* h) { free((void*)h->data); h->data = NULL; }

static size_t whole_bytes(const World* w) {
    return sizeof(World) + sizeof(Whole) + (size_t)w->w * w->h * LAYERS;
}

/* Visit every cell once in chunk order, comparing two backends. Chunk order is
   the natural streaming access pattern: a chunk's cells are consumed before
   the cache has to hold anything else. */
typedef struct {
    long cells;
    long tile_bad, variant_bad;
    uint64_t hash;
} Sweep;

static Sweep sweep_compare(TileFn ga, void* ca, TileFn gb, void* cb, const World* w) {
    Sweep r = { 0, 0, 0, 1469598103934665603ull };
    for (int cy = 0; cy < w->ch; cy++)
        for (int cx = 0; cx < w->cw; cx++) {
            int x0 = cx * T, y0 = cy * T;
            int x1 = x0 + T < w->w ? x0 + T : w->w;
            int y1 = y0 + T < w->h ? y0 + T : w->h;
            for (int l = 0; l < LAYERS; l++)
                for (int y = y0; y < y1; y++)
                    for (int x = x0; x < x1; x++) {
                        unsigned char ta = ga(ca, l, x, y), tb = gb(cb, l, x, y);
                        int va = variant_at(ga, ca, l, x, y);
                        int vb = variant_at(gb, cb, l, x, y);
                        if (ta != tb) r.tile_bad++;
                        if (va != vb) r.variant_bad++;
                        r.cells++;
                        r.hash = (r.hash ^ (uint64_t)(va + 1)) * 1099511628211ull;
                    }
        }
    return r;
}

/* Tests --------------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    World w;
    check("create_world", world_create(&w, 53, 41) == 0);   /* 4x3 chunks, partial edges */
    Whole whole;
    check("load_whole", whole_load(&whole, &w) == 0);

    Cache c9;
    check("open_cache", cache_open(&c9, &w, 9) == 0);
    Stream st9 = { &w, &c9 };

    Sweep sw = sweep_compare(whole_tile, &whole, stream_tile, &st9, &w);
    check("cells_visited", sw.cells == (long)w.w * w.h * LAYERS);
    check("tile_values_exact", sw.tile_bad == 0);
    check("variant_cells_exact", sw.variant_bad == 0);

    /* Seam cells, named explicitly: last column/row of a full chunk, first
       column/row of the next one, and the partial edge chunk's border. */
    check("seam_east",
          variant_at(whole_tile, &whole, 0, T - 1, 5) ==
          variant_at(stream_tile, &st9, 0, T - 1, 5));
    check("seam_corner",
          variant_at(whole_tile, &whole, 0, T, T) ==
          variant_at(stream_tile, &st9, 0, T, T));
    check("seam_partial_edge",
          variant_at(whole_tile, &whole, 1, w.w - 1, w.h - 1) ==
          variant_at(stream_tile, &st9, 1, w.w - 1, w.h - 1));

    /* Cross-chunk rect queries, including rects that run off the map. */
    check("rect_cross_seam",
          count_rect(whole_tile, &whole, &w, 0, T - 3, T - 3, T + 3, T + 3) ==
          count_rect_stream(&st9, 0, T - 3, T - 3, T + 3, T + 3));
    check("rect_off_map",
          count_rect(whole_tile, &whole, &w, 0, -20, -20, 90, 90) ==
          count_rect_stream(&st9, 0, -20, -20, 90, 90));
    check("rect_inverted",
          count_rect(whole_tile, &whole, &w, 0, 30, 30, 10, 10) ==
          count_rect_stream(&st9, 0, 30, 30, 10, 10));
    check("rect_full_map",
          count_rect(whole_tile, &whole, &w, 1, 0, 0, w.w - 1, w.h - 1) ==
          count_rect_stream(&st9, 1, 0, 0, w.w - 1, w.h - 1));

    check("outside_cell_empty",
          stream_tile(&st9, 0, -1, -1) == 0 && stream_tile(&st9, 0, w.w, 0) == 0 &&
          stream_tile(&st9, 0, 0, w.h) == 0);
    check("bad_layer_empty", stream_tile(&st9, 9, 0, 0) == 0);

    /* Results must not depend on how many chunks are resident. */
    Cache c1, c64, c0;
    check("open_zero_cache", cache_open(&c0, &w, 0) == 0);
    cache_open(&c1, &w, 1);
    cache_open(&c64, &w, 64);
    Stream st1 = { &w, &c1 }, st64 = { &w, &c64 }, st0 = { &w, &c0 };
    Sweep s1 = sweep_compare(whole_tile, &whole, stream_tile, &st1, &w);
    Sweep s64 = sweep_compare(whole_tile, &whole, stream_tile, &st64, &w);
    Sweep s0 = sweep_compare(whole_tile, &whole, stream_tile, &st0, &w);
    check("budget0_exact", s0.tile_bad == 0 && s0.variant_bad == 0);
    check("budget1_exact", s1.tile_bad == 0 && s1.variant_bad == 0);
    check("budget_all_exact", s64.tile_bad == 0 && s64.variant_bad == 0);
    check("budget_independent_hash",
          s0.hash == sw.hash && s1.hash == sw.hash && s64.hash == sw.hash);
    check("budget0_holds_nothing", c0.loads == 0 && c0.resident == 0);

    /* Index-based load/unload: dropping a resident chunk must not change a
       later query, and the query must bring it back. */
    Cache cu;
    cache_open(&cu, &w, 4);
    Stream stu = { &w, &cu };
    ChunkRef probe = { 1, 1 };
    int before = variant_at(stream_tile, &stu, 0, probe.cx * T + 3, probe.cy * T + 3);
    check("probe_resident", cache_resident(&cu, probe) == 1);
    check("explicit_unload", cache_unload(&cu, probe) == 1);
    check("unload_dropped", cache_resident(&cu, probe) == 0);
    int after = variant_at(stream_tile, &stu, 0, probe.cx * T + 3, probe.cy * T + 3);
    check("reload_same_value", after == before &&
          after == variant_at(whole_tile, &whole, 0, probe.cx * T + 3, probe.cy * T + 3));
    check("reload_resident", cache_resident(&cu, probe) == 1);
    cache_close(&cu);

    /* Residency stats: a chunk-row-major sweep visits chunks in the order a
       camera would, and two chunk rows are what its seams need (the north
       neighbour of row r+1 is in row r). With that much room every chunk
       loads once; a smaller budget stays exact, just reloads (budget1_exact). */
    Cache cs;
    cache_open(&cs, &w, 2 * w.cw);
    Stream sts = { &w, &cs };
    Sweep ss = sweep_compare(whole_tile, &whole, stream_tile, &sts, &w);
    printf("chunks=%d loads=%ld evicts=%ld resident=%d cap=%d\n",
           w.cw * w.ch, cs.loads, cs.evicts, cs.resident, cs.cap);
    check("sweep_exact", ss.tile_bad == 0 && ss.variant_bad == 0);
    check("resident_within_budget", cs.resident <= cs.cap);
    check("tight_budget_bounded", cs.loads <= 2 * w.cw * w.ch);
    check("evicted_some", cs.evicts > 0);
    check("load_minus_evict", cs.loads - cs.evicts == cs.resident);
    cache_close(&cs);

    /* A budget that holds the whole map is the degenerate case: one load per
       chunk and no eviction. Same code path, same answers. */
    Cache call;
    cache_open(&call, &w, w.cw * w.ch);
    Stream stall = { &w, &call };
    sweep_compare(whole_tile, &whole, stream_tile, &stall, &w);
    printf("all_resident loads=%ld evicts=%ld\n", call.loads, call.evicts);
    check("all_resident_one_load", call.loads == w.cw * w.ch && call.evicts == 0);
    cache_close(&call);

    /* Peak memory: the streamed path is sized by the budget alone, the whole
       path by the map. Same budget, different map -> identical cache bytes. */
    size_t st9_bytes = sizeof(World) + cache_bytes(&c9) + sizeof(Stream);
    printf("small_map_bytes=%zu stream_bytes_cap9=%zu chunk_bytes=%zu\n",
           whole_bytes(&w), st9_bytes, sizeof(Chunk));

    Cache c3, c12;
    cache_open(&c3, &w, 3);
    cache_open(&c12, &w, 12);
    printf("cache_bytes_cap3=%zu cap9=%zu cap12=%zu\n",
           cache_bytes(&c3), cache_bytes(&c9), cache_bytes(&c12));
    check("memory_linear_in_resident",
          cache_bytes(&c12) - cache_bytes(&c3) == 9 * sizeof(Chunk) &&
          cache_bytes(&c12) > cache_bytes(&c9) && cache_bytes(&c9) > cache_bytes(&c3));
    cache_close(&c3);
    cache_close(&c12);

    cache_close(&c0);
    cache_close(&c1);
    cache_close(&c64);
    cache_close(&c9);
    whole_free(&whole);
    world_destroy(&w);

    /* Big map: same code, same cache size, whole image no longer fits
       comfortably in the budget. */
    World big;
    check("create_big", world_create(&big, 512, 512) == 0);
    Whole whole_big;
    check("load_whole_big", whole_load(&whole_big, &big) == 0);
    Cache bigc;
    cache_open(&bigc, &big, 9);
    Stream bst = { &big, &bigc };
    Sweep bs = sweep_compare(whole_tile, &whole_big, stream_tile, &bst, &big);
    check("big_cells_exact", bs.tile_bad == 0 && bs.variant_bad == 0);
    check("big_cells_visited", bs.cells == (long)big.w * big.h * LAYERS);
    check("big_seam",
          variant_at(whole_tile, &whole_big, 0, 2 * T - 1, 3 * T - 1) ==
          variant_at(stream_tile, &bst, 0, 2 * T - 1, 3 * T - 1));

    size_t stream_big = sizeof(World) + cache_bytes(&bigc) + sizeof(Stream);
    size_t whole_big_bytes = whole_bytes(&big);
    printf("big_chunks=%d whole_bytes=%zu stream_bytes_cap9=%zu loads=%ld evicts=%ld\n",
           big.cw * big.ch, whole_big_bytes, stream_big, bigc.loads, bigc.evicts);
    check("big_stream_much_smaller", whole_big_bytes > 40 * stream_big);
    check("peak_independent_of_map", stream_big == st9_bytes);

    cache_close(&bigc);
    whole_free(&whole_big);
    world_destroy(&big);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
