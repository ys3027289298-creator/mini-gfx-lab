/* tilerender: a tile render chain in three stages.

   1. stitch - the map is cut into edge blocks of EB x EB tiles.  A block's
      variant is a stitched EB*TILE square image: every cell in it is painted
      from that cell's own 4-neighbour mask (inside the block from the
      occupancy bitmap, on the border from the side-context bits).  The block
      content key is the exact occupancy bitmap, the side context and the
      per-cell tile ids.

   2. cache - variants are content addressed by that exact key, so an entry can
      never go stale: the same key always paints the same image.  An entry is
      invalidated when the last live block stops referencing it.  A cell edit
      re-stitches only the block it is in plus a block across a block edge whose
      side context reads that cell; unchanged content is a no-op.

   3. sort - every run of horizontally adjacent cells in one block becomes one
      draw command, and every command from every layer carries the single depth
      key (layer, y, x).  The whole list is sorted once before rasterising.

   Verification compares the chain against a reference that recomputes and
   draws every tile on its own in layer/row/column order; the two must agree
   pixel for pixel.  Drawing the sorted list backwards must not agree, because
   layers composite with alpha.
*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define MW 96
#define MH 64
#define ML 4

#define EB 8                      /* block size in tiles */
#define WB (MW / EB)
#define HB (MH / EB)
#define NBLK (ML * WB * HB)

#define TILE 8
#define BPX (EB * TILE)           /* pixels per variant side */
#define PIXW (MW * TILE)
#define PIXH (MH * TILE)

#define MAXQ NBLK
#define MAXCMD (MW * MH * ML)
/* One live key per populated block, plus the previous key of every block that
   changes in the same frame; this bound means an allocation can always find a
   slot no live block references. */
#define NSLOT (2 * NBLK + 16)

/* mask bits */
#define N 1
#define E 2
#define S 4
#define W 8

typedef struct {
    unsigned char cell[ML][MH][MW];   /* 0 = empty, else tile id */
    int visible[ML];
} TileMap;

static int map_get(const TileMap* m, int layer, int x, int y) {
    if (layer < 0 || layer >= ML || x < 0 || x >= MW || y < 0 || y >= MH) return 0;
    return m->cell[layer][y][x];
}

/* ---------------------------------------------------------------- blocks */

typedef struct {
    uint64_t occ;                 /* 8x8 occupancy, bit (j*8+i) */
    uint32_t sides;               /* outside presence: L | R<<8 | T<<16 | B<<24 */
    unsigned char id[EB][EB];
    int slot;                     /* cache entry, or -1 */
    int need;                     /* content changed, resolve cache */
    int queued;
} Block;

typedef struct {
    uint64_t occ;
    uint32_t sides;
    unsigned char id[EB][EB];
    unsigned char img[BPX][BPX];
    int used;
    int refs;                     /* live blocks whose current key is this */
    unsigned long stamp;
} Slot;

typedef struct {
    int layer, x, y, len, src_i, slot;
    uint64_t depth;
} Cmd;

typedef struct {
    long rebuilt, queries, hits, misses, gens, invalidated, evicted;
    long bailed, cmds, tiles, live;
} FrameStats;

typedef struct {
    long queries, hits, misses, gens, invalidated, evicted;
} CacheStats;

static TileMap g_map;
static Block g_blk[NBLK];
static Slot g_slots[NSLOT];
static Cmd g_cmds[MAXCMD];
static unsigned char g_canvas[PIXH][PIXW];

typedef struct {
    int q[MAXQ], qh, qn;
    unsigned long stamp;
    int ncmds;
    long slot_mismatch;
    long evictions;
    long bailed;   /* allocation failed (all slots live); never expected */
    unsigned char alpha[ML];
    FrameStats fstat;
    CacheStats cstat;
} Renderer;

static int blk_index(int layer, int bx, int by) {
    return (layer * HB + by) * WB + bx;
}

static void blk_enqueue(Renderer* r, int bi) {
    if (bi < 0 || bi >= NBLK || g_blk[bi].queued) return;
    g_blk[bi].queued = 1;
    r->q[r->qn++] = bi;
}

/* A cell edit can only change the art of the block it sits in, plus a block
   across a block edge whose edge-adjacent cell is occupied (that cell's mask
   includes the edited one).  An empty block has no art to update. */
static void mark_cell(Renderer* r, int layer, int x, int y) {
    int bx = x / EB, by = y / EB;
    blk_enqueue(r, blk_index(layer, bx, by));
    if (map_get(&g_map, layer, x - 1, y) && x % EB == 0 && bx > 0)
        blk_enqueue(r, blk_index(layer, bx - 1, by));
    if (map_get(&g_map, layer, x + 1, y) && x % EB == EB - 1 && bx + 1 < WB)
        blk_enqueue(r, blk_index(layer, bx + 1, by));
    if (map_get(&g_map, layer, x, y - 1) && y % EB == 0 && by > 0)
        blk_enqueue(r, blk_index(layer, bx, by - 1));
    if (map_get(&g_map, layer, x, y + 1) && y % EB == EB - 1 && by + 1 < HB)
        blk_enqueue(r, blk_index(layer, bx, by + 1));
}

static void map_set(Renderer* r, int layer, int x, int y, int id) {
    if (layer < 0 || layer >= ML || x < 0 || x >= MW || y < 0 || y >= MH) return;
    if (g_map.cell[layer][y][x] == (unsigned char)id) return;
    g_map.cell[layer][y][x] = (unsigned char)id;
    mark_cell(r, layer, x, y);
}

/* visibility only gates draw commands; content and cache are untouched */
static void map_visibility(Renderer* r, int layer, int on) {
    (void)r;
    if (layer < 0 || layer >= ML) return;
    g_map.visible[layer] = on;
}

/* ------------------------------------------------------- stitched pixels */

/* Art of one tile: shade by tile id, texture by position, exposed sides
   darkened.  The mask has bits set for neighbours that are present. */
static unsigned char tile_px(int id, int mask, int x, int y) {
    unsigned base = 60u + 60u * (unsigned)(id % 4);
    uint64_t h = (uint64_t)(unsigned)id * 131u + (uint64_t)(unsigned)mask * 17u
               + (uint64_t)x * 7u + (uint64_t)y * 13u;
    h ^= h >> 3;
    h *= 0x9E3779B1u;
    h ^= h >> 11;
    unsigned v = base + (unsigned)(h % 20u);
    int exposed = (!(mask & N) && y == 0) || (!(mask & S) && y == TILE - 1) ||
                  (!(mask & W) && x == 0) || (!(mask & E) && x == TILE - 1);
    if (exposed) v = v > 70u ? v - 60u : v / 2u;
    return (unsigned char)v;
}

static int blk_cell_present(uint64_t occ, int i, int j) {
    return (int)((occ >> (j * EB + i)) & 1ull);
}

/* side bit accessors: L row j, R row j, T column i, B column i */
static int side_present(uint32_t sides, int which, int k) {
    return (int)((sides >> (which * EB + k)) & 1u);
}

/* Rebuild a variant image.  Every tile inside it is painted from that tile's
   own mask, exactly as a per-tile pass would paint it. */
static void variant_gen(Slot* s) {
    for (int j = 0; j < EB; j++) {
        for (int i = 0; i < EB; i++) {
            if (!blk_cell_present(s->occ, i, j)) {
                for (int py = 0; py < TILE; py++)
                    memset(&s->img[j * TILE + py][i * TILE], 0, TILE);
                continue;
            }
            int mask = 0;
            if (j > 0 ? blk_cell_present(s->occ, i, j - 1) : side_present(s->sides, 2, i))
                mask |= N;
            if (i < EB - 1 ? blk_cell_present(s->occ, i + 1, j) : side_present(s->sides, 1, j))
                mask |= E;
            if (j < EB - 1 ? blk_cell_present(s->occ, i, j + 1) : side_present(s->sides, 3, i))
                mask |= S;
            if (i > 0 ? blk_cell_present(s->occ, i - 1, j) : side_present(s->sides, 0, j))
                mask |= W;
            for (int py = 0; py < TILE; py++)
                for (int px = 0; px < TILE; px++)
                    s->img[j * TILE + py][i * TILE + px] =
                        tile_px(s->id[i][j], mask, px, py);
        }
    }
}

/* ---------------------------------------------------------------- cache */

static int slot_matches(const Slot* s, const Block* b) {
    return s->used && s->occ == b->occ && s->sides == b->sides &&
           memcmp(s->id, b->id, sizeof(s->id)) == 0;
}

static int cache_find(const Block* b) {
    for (int i = 0; i < NSLOT; i++)
        if (slot_matches(&g_slots[i], b)) return i;
    return -1;
}

/* never evicts a slot a live block still points at */
static int cache_alloc(Renderer* r) {
    for (int i = 0; i < NSLOT; i++)
        if (!g_slots[i].used) return i;

    int best = -1;
    unsigned long oldest = (unsigned long)-1;
    for (int i = 0; i < NSLOT; i++) {
        const Slot* s = &g_slots[i];
        if (s->refs == 0 && s->stamp < oldest) { oldest = s->stamp; best = i; }
    }
    if (best < 0) {   /* unreachable: NSLOT exceeds the number of live keys */
        r->bailed++;
        return -1;
    }
    r->evictions++;
    r->fstat.evicted++;
    g_slots[best].used = 0;
    return best;
}

static void resolve_block(Renderer* r, int bi) {
    Block* b = &g_blk[bi];
    if (!b->need) return;
    b->need = 0;

    r->fstat.queries++;
    int idx = cache_find(b);
    if (idx >= 0) {
        r->fstat.hits++;
    } else {
        r->fstat.misses++;
        idx = cache_alloc(r);
        if (idx < 0) { b->slot = -1; return; }
        Slot* s = &g_slots[idx];
        s->used = 0;
        memset(s, 0, sizeof(*s));
        s->used = 1;
        s->occ = b->occ;
        s->sides = b->sides;
        memcpy(s->id, b->id, sizeof(s->id));
        s->stamp = ++r->stamp;
        variant_gen(s);
        r->fstat.gens++;
    }
    g_slots[idx].stamp = ++r->stamp;
    g_slots[idx].refs++;
    b->slot = idx;
}

/* Count references for every settled block before any allocation, so the
   allocator can never take a slot a live block still points at. */
static void cache_begin(Renderer* r) {
    for (int i = 0; i < NSLOT; i++) g_slots[i].refs = 0;

    for (int bi = 0; bi < NBLK; bi++) {
        Block* b = &g_blk[bi];
        if (b->need || !b->occ) continue;
        if (b->slot < 0) { b->need = 1; r->slot_mismatch++; continue; }
        Slot* s = &g_slots[b->slot];
        if (!slot_matches(s, b)) {
            b->need = 1;
            r->slot_mismatch++;
            continue;
        }
        s->refs++;
    }
}

/* An entry lives exactly as long as some live block's current key maps to it. */
static void cache_end_frame(Renderer* r) {
    r->fstat.live = 0;
    for (int i = 0; i < NSLOT; i++) {
        Slot* s = &g_slots[i];
        if (!s->used) continue;
        if (s->refs == 0) {
            s->used = 0;
            r->fstat.invalidated++;
        } else {
            r->fstat.live++;
        }
    }
}

/* ---------------------------------------------------------------- stitch */

static void stitch_block(Renderer* r, int bi) {
    int layer = bi / (HB * WB);
    int rem = bi % (HB * WB);
    int by = rem / WB, bx = rem % WB;
    const unsigned char (*cells)[MW] = g_map.cell[layer];

    uint64_t occ = 0;
    for (int j = 0; j < EB; j++)
        for (int i = 0; i < EB; i++)
            if (cells[by * EB + j][bx * EB + i])
                occ |= 1ull << (j * EB + i);

    uint32_t sides = 0;
    for (int k = 0; k < EB; k++) {
        /* only an occupied edge cell can expose the outside neighbour, so the
           key is art-precise: an edit outside an empty edge cell leaves it */
        int edge;

        edge = (int)((occ >> (k * EB + 0)) & 1ull);
        if (edge && map_get(&g_map, layer, bx * EB - 1, by * EB + k)) sides |= 1u << k;

        edge = (int)((occ >> (k * EB + EB - 1)) & 1ull);
        if (edge && map_get(&g_map, layer, bx * EB + EB, by * EB + k))
            sides |= 1u << (8 + k);

        edge = (int)((occ >> (0 * EB + k)) & 1ull);
        if (edge && map_get(&g_map, layer, bx * EB + k, by * EB - 1))
            sides |= 1u << (16 + k);

        edge = (int)((occ >> ((EB - 1) * EB + k)) & 1ull);
        if (edge && map_get(&g_map, layer, bx * EB + k, by * EB + EB))
            sides |= 1u << (24 + k);
    }

    Block* b = &g_blk[bi];
    b->occ = occ;
    b->sides = sides;
    for (int j = 0; j < EB; j++)
        for (int i = 0; i < EB; i++)
            b->id[i][j] = cells[by * EB + j][bx * EB + i];

    if (!occ) {
        b->slot = -1;
        b->need = 0;
    } else if (b->slot >= 0 && slot_matches(&g_slots[b->slot], b)) {
        b->need = 0;     /* stitched back to the same content: nothing to do */
    } else {
        b->need = 1;
    }
    r->fstat.rebuilt++;
}

/* ----------------------------------------------------------------- sort */

static uint64_t depth_key(int layer, int x, int y) {
    return ((uint64_t)layer * MH + (uint64_t)y) * MW + (uint64_t)x;
}

static int cmd_compare(const void* pa, const void* pb) {
    uint64_t a = ((const Cmd*)pa)->depth;
    uint64_t b = ((const Cmd*)pb)->depth;
    return a < b ? -1 : (a > b ? 1 : 0);
}

/* One draw command per horizontal run of cells inside a block.  The run is a
   contiguous strip of the variant image, so drawing it as one command is
   pixel-equivalent to drawing its cells one by one. */
static void build_cmds(Renderer* r, int x0, int y0, int x1, int y1) {
    r->ncmds = 0;
    r->fstat.tiles = 0;
    for (int layer = 0; layer < ML; layer++) {
        if (!g_map.visible[layer]) continue;
        for (int y = y0; y <= y1; y++) {
            int by = y / EB, j = y % EB;
            int x = x0;
            while (x <= x1) {
                int bx = x / EB;
                const Block* b = &g_blk[blk_index(layer, bx, by)];
                if (!blk_cell_present(b->occ, x % EB, j) || b->slot < 0) {
                    x++;
                    continue;
                }
                int stop = (bx + 1) * EB - 1;
                if (stop > x1) stop = x1;
                int start = x;
                while (x + 1 <= stop && blk_cell_present(b->occ, (x + 1) % EB, j)) x++;
                if (r->ncmds >= MAXCMD) return;
                Cmd* c = &g_cmds[r->ncmds++];
                c->layer = layer;
                c->x = start;
                c->y = y;
                c->len = x - start + 1;
                c->src_i = start % EB;
                c->slot = b->slot;
                c->depth = depth_key(layer, start, y);
                r->fstat.tiles += c->len;
                x = start + c->len;
            }
        }
    }
    r->fstat.cmds = r->ncmds;
}

/* Layers composite, so the sort order changes pixels wherever layers overlap. */
static unsigned char blend(unsigned char dst, unsigned char src, unsigned a) {
    return (unsigned char)((src * a + dst * (255u - a) + 127u) / 255u);
}

static void draw_cmds(Renderer* r, unsigned char out[PIXH][PIXW], int reverse) {
    for (int k = 0; k < r->ncmds; k++) {
        const Cmd* c = &g_cmds[reverse ? r->ncmds - 1 - k : k];
        const Slot* s = &g_slots[c->slot];
        unsigned a = r->alpha[c->layer];
        int w = c->len * TILE;
        int src0 = c->src_i * TILE;
        for (int py = 0; py < TILE; py++) {
            const unsigned char* src = &s->img[(c->y % EB) * TILE + py][src0];
            unsigned char* dst = &out[c->y * TILE + py][c->x * TILE];
            for (int px = 0; px < w; px++)
                dst[px] = blend(dst[px], src[px], a);
        }
    }
}

/* ------------------------------------------------------------- pipeline */

static void render_frame(Renderer* r, int x0, int y0, int x1, int y1) {
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= MW) x1 = MW - 1;
    if (y1 >= MH) y1 = MH - 1;

    memset(&r->fstat, 0, sizeof(r->fstat));
    memset(g_canvas, 0, sizeof(g_canvas));

    /* stage 1: stitch every dirty block */
    while (r->qh < r->qn) {
        int bi = r->q[r->qh++];
        g_blk[bi].queued = 0;
        stitch_block(r, bi);
    }
    r->qh = r->qn = 0;

    /* stage 2: resolve the cache for changed blocks */
    cache_begin(r);
    for (int bi = 0; bi < NBLK; bi++)
        if (g_blk[bi].need) resolve_block(r, bi);

    /* stage 3: one depth key for every run, one sort, one draw pass */
    build_cmds(r, x0, y0, x1, y1);
    qsort(g_cmds, (size_t)r->ncmds, sizeof(g_cmds[0]), cmd_compare);
    draw_cmds(r, g_canvas, 0);

    cache_end_frame(r);

    r->cstat.queries += r->fstat.queries;
    r->cstat.hits += r->fstat.hits;
    r->cstat.misses += r->fstat.misses;
    r->cstat.gens += r->fstat.gens;
    r->cstat.invalidated += r->fstat.invalidated;
    r->cstat.evicted += r->fstat.evicted;
}

static void render_init(Renderer* r) {
    memset(r, 0, sizeof(*r));
    memset(g_map.cell, 0, sizeof(g_map.cell));
    memset(g_slots, 0, sizeof(g_slots));
    memset(g_blk, 0, sizeof(g_blk));
    for (int l = 0; l < ML; l++) {
        g_map.visible[l] = 1;
        /* layer 0 is the opaque floor, the rest composite over it */
        r->alpha[l] = l == 0 ? 255 : (unsigned char)(220 - 40 * l);
    }
    for (int bi = 0; bi < NBLK; bi++) {
        g_blk[bi].slot = -1;
        blk_enqueue(r, bi);
    }
}

/* ----------------------------------------- reference: draw every tile fresh */

static void render_reference(const Renderer* r, unsigned char out[PIXH][PIXW]) {
    memset(out, 0, (size_t)PIXH * PIXW);
    for (int layer = 0; layer < ML; layer++) {
        if (!g_map.visible[layer]) continue;
        unsigned a = r->alpha[layer];
        for (int y = 0; y < MH; y++) {
            for (int x = 0; x < MW; x++) {
                int id = map_get(&g_map, layer, x, y);
                if (!id) continue;
                int mask = 0;
                if (map_get(&g_map, layer, x, y - 1)) mask |= N;
                if (map_get(&g_map, layer, x + 1, y)) mask |= E;
                if (map_get(&g_map, layer, x, y + 1)) mask |= S;
                if (map_get(&g_map, layer, x - 1, y)) mask |= W;
                for (int py = 0; py < TILE; py++) {
                    unsigned char* row = &out[y * TILE + py][x * TILE];
                    for (int px = 0; px < TILE; px++)
                        row[px] = blend(row[px], tile_px(id, mask, px, py), a);
                }
            }
        }
    }
}

static long compare_pixels(const unsigned char a[PIXH][PIXW],
                           const unsigned char b[PIXH][PIXW],
                           int x0, int y0, int x1, int y1) {
    long bad = 0;
    for (int y = y0 * TILE; y <= y1 * TILE + TILE - 1; y++)
        for (int x = x0 * TILE; x <= x1 * TILE + TILE - 1; x++)
            if (a[y][x] != b[y][x]) bad++;
    return bad;
}

static long expected_tiles(int x0, int y0, int x1, int y1) {
    long n = 0;
    for (int l = 0; l < ML; l++) {
        if (!g_map.visible[l]) continue;
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++)
                if (map_get(&g_map, l, x, y)) n++;
    }
    return n;
}

static int depth_sorted(const Renderer* r) {
    for (int i = 1; i < r->ncmds; i++)
        if (g_cmds[i - 1].depth >= g_cmds[i].depth) return 0;
    return 1;
}

/* ----------------------------------------------------------------- tests */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static uint64_t rng_state = 0x123456789abcdefull;
static unsigned rnd(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return (unsigned)(rng_state >> 16);
}

static void fill_random(Renderer* r) {
    for (int layer = 0; layer < 3; layer++) {
        int pct = layer == 0 ? 34 : (layer == 1 ? 16 : 6);
        for (int y = 0; y < MH; y++)
            for (int x = 0; x < MW; x++)
                if ((int)(rnd() % 100) < pct)
                    map_set(r, layer, x, y, 1 + (int)(rnd() % 3));
    }
}

/* Repeated terrain patch: interior blocks repeat, so the cache should serve
   them after the first generation. */
static void fill_terrain(Renderer* r) {
    static const unsigned char patch[EB][EB] = {
        { 1, 2, 1, 1, 3, 1, 2, 1 },
        { 1, 0, 0, 1, 0, 0, 1, 2 },
        { 2, 0, 3, 1, 0, 1, 0, 1 },
        { 1, 1, 1, 0, 1, 1, 0, 1 },
        { 1, 0, 0, 2, 0, 1, 1, 3 },
        { 3, 0, 1, 1, 1, 0, 0, 1 },
        { 1, 2, 0, 0, 1, 1, 2, 1 },
        { 1, 1, 3, 1, 1, 2, 1, 1 },
    };
    for (int y = 0; y < MH; y++)
        for (int x = 0; x < MW; x++)
            if (patch[y % EB][x % EB])
                map_set(r, 0, x, y, patch[y % EB][x % EB]);
}

static long run_random_frames(Renderer* r, unsigned char ref[PIXH][PIXW]) {
    long bad = 0;
    for (int f = 0; f < 30; f++) {
        unsigned op = rnd() % 10;
        for (int k = 0; k < (op < 6 ? 3 : 1); k++) {
            if (op < 6) {
                int l = (int)(rnd() % 3);
                int x = (int)(rnd() % MW), y = (int)(rnd() % MH);
                map_set(r, l, x, y, map_get(&g_map, l, x, y) ? 0 : 1 + (int)(rnd() % 3));
            } else if (op < 8) {
                /* id-only edit: masks and neighbour context are unchanged */
                int l = (int)(rnd() % 3);
                int x = (int)(rnd() % MW), y = (int)(rnd() % MH);
                int cur = map_get(&g_map, l, x, y);
                if (cur) map_set(r, l, x, y, cur % 3 + 1);
            } else {
                int l = 1 + (int)(rnd() % 2);
                map_visibility(r, l, !g_map.visible[l]);
            }
        }

        render_frame(r, 0, 0, MW - 1, MH - 1);
        render_reference(r, ref);
        long diff = compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1);
        bad += diff;
        if (f < 6 || diff) {
            printf("frame=%d rebuild=%ld query=%ld hit=%ld miss=%ld gen=%ld "
                   "inval=%ld live=%ld cmds=%ld tiles=%ld\n",
                   f, r->fstat.rebuilt, r->fstat.queries, r->fstat.hits,
                   r->fstat.misses, r->fstat.gens, r->fstat.invalidated,
                   r->fstat.live, r->fstat.cmds, r->fstat.tiles);
        }
        if (!depth_sorted(r)) {
            printf("depth_order_broken frame=%d\n", f);
            bad++;
        }
    }
    return bad;
}

int main(void) {
    static unsigned char ref[PIXH][PIXW];
    static unsigned char alt[PIXH][PIXW];
    static Renderer r;

    /* 1. chain equivalence under random edits and visibility changes */
    render_init(&r);
    fill_random(&r);
    long bad = run_random_frames(&r, ref);
    check("random_pixels_identical", bad == 0);
    check("slot_pointers_consistent", r.slot_mismatch == 0);
    check("cache_no_capacity_evict", r.cstat.evicted == 0 && r.evictions == 0 &&
                                     r.bailed == 0);
    long q = r.cstat.queries;
    double hit_rate = q ? (double)r.cstat.hits / (double)q : 1.0;
    printf("random hit_rate=%.4f queries=%ld hits=%ld misses=%ld gens=%ld\n",
           hit_rate, q, r.cstat.hits, r.cstat.misses, r.cstat.gens);

    /* 2. steady frame: no stitching, no lookups, no invalidation */
    long cmds_before = r.fstat.cmds;
    long tiles_before = r.fstat.tiles;
    render_frame(&r, 0, 0, MW - 1, MH - 1);
    check("steady_no_rebuild", r.fstat.rebuilt == 0);
    check("steady_no_queries", r.fstat.queries == 0);
    check("steady_no_invalidation", r.fstat.invalidated == 0);
    check("steady_same_cmds", r.fstat.cmds == cmds_before &&
                              r.fstat.tiles == tiles_before);
    render_reference(&r, ref);
    check("steady_pixels", compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1) == 0);

    /* 3. camera window: only the window is drawn, and it is exact */
    render_frame(&r, 20, 12, 59, 41);
    long want = expected_tiles(20, 12, 59, 41);
    printf("cull_window tiles=%ld cmds=%ld expected_tiles=%ld\n",
           r.fstat.tiles, r.fstat.cmds, want);
    check("cull_tile_count", r.fstat.tiles == want);
    check("cull_cmd_under_tiles", r.fstat.cmds <= r.fstat.tiles);
    render_reference(&r, ref);
    check("cull_pixels_identical", compare_pixels(g_canvas, ref, 20, 12, 59, 41) == 0);
    check("cull_depth_sorted", depth_sorted(&r));

    /* 4. depth order is load bearing: reverse the sorted list and diff */
    {
        static Renderer s;
        render_init(&s);
        for (int y = 16; y < 40; y++)
            for (int x = 16; x < 40; x++) map_set(&s, 0, x, y, 1);
        for (int y = 20; y < 36; y++)
            for (int x = 20; x < 36; x++) map_set(&s, 1, x, y, 2);
        for (int y = 24; y < 32; y++)
            for (int x = 24; x < 32; x++) map_set(&s, 2, x, y, 3);
        render_frame(&s, 0, 0, MW - 1, MH - 1);
        render_reference(&s, ref);
        memset(alt, 0, sizeof(alt));
        draw_cmds(&s, alt, 1);
        long fwd_bad = compare_pixels(g_canvas, ref, 16, 16, 39, 39);
        long rev_bad = compare_pixels(alt, g_canvas, 16, 16, 39, 39);
        printf("sorted_vs_reference=%ld reversed_vs_sorted=%ld\n", fwd_bad, rev_bad);
        check("forward_matches_reference", fwd_bad == 0);
        check("reverse_differs", rev_bad > 0);
    }

    /* 5. two identical blocks share one variant, then one of them changes */
    {
        static Renderer b;
        render_init(&b);
        for (int y = 8; y < 16; y++)
            for (int x = 8; x < 16; x++) map_set(&b, 0, x, y, 1);       /* block (1,1) */
        for (int y = 8; y < 16; y++)
            for (int x = 24; x < 32; x++) map_set(&b, 0, x, y, 1);      /* block (3,1) */
        render_frame(&b, 0, 0, MW - 1, MH - 1);
        render_reference(&b, ref);
        printf("shared_variant query=%ld hit=%ld miss=%ld gen=%ld inval=%ld\n",
               b.fstat.queries, b.fstat.hits, b.fstat.misses, b.fstat.gens,
               b.fstat.invalidated);
        check("shared_variant_pixels", compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1) == 0);
        check("shared_variant_dedup", b.cstat.queries == 2 && b.cstat.gens == 1);

        /* an id edit changes the art: one block re-stitches, new variant
           generated, the old one survives because block (3,1) still uses it */
        map_set(&b, 0, 11, 11, 2);
        render_frame(&b, 0, 0, MW - 1, MH - 1);
        render_reference(&b, ref);
        printf("edit_id rebuild=%ld query=%ld miss=%ld gen=%ld inval=%ld\n",
               b.fstat.rebuilt, b.fstat.queries, b.fstat.misses, b.fstat.gens,
               b.fstat.invalidated);
        check("edit_id_rebuild_one", b.fstat.rebuilt == 1);
        check("edit_id_one_miss", b.fstat.queries == 1 && b.fstat.misses == 1);
        check("edit_id_old_variant_kept", b.fstat.invalidated == 0);
        check("edit_id_pixels", compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1) == 0);

        /* reverting is a cache hit: the original variant is still live */
        long gens_before = b.cstat.gens;
        map_set(&b, 0, 11, 11, 1);
        render_frame(&b, 0, 0, MW - 1, MH - 1);
        render_reference(&b, ref);
        printf("revert rebuild=%ld hit=%ld miss=%ld gen_delta=%ld\n",
               b.fstat.rebuilt, b.fstat.hits, b.fstat.misses,
               b.cstat.gens - gens_before);
        check("revert_rebuild_one", b.fstat.rebuilt == 1);
        check("revert_cache_hit", b.fstat.queries == 1 && b.fstat.hits == 1);
        check("revert_no_regeneration", b.cstat.gens == gens_before);
        check("revert_pixels", compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1) == 0);

        /* the abandoned variant is reclaimed once nothing references it */
        map_set(&b, 0, 11, 11, 2);
        render_frame(&b, 0, 0, MW - 1, MH - 1);
        long gen_after = b.cstat.gens;
        map_set(&b, 0, 27, 11, 2);   /* make block (3,1) stop using the old key */
        render_frame(&b, 0, 0, MW - 1, MH - 1);
        render_reference(&b, ref);
        printf("reclaim inval=%ld gen_delta=%ld live=%ld\n",
               b.fstat.invalidated, b.cstat.gens - gen_after, b.fstat.live);
        check("reclaim_invalidates", b.fstat.invalidated == 1);
        check("reclaim_pixels", compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1) == 0);
    }

    /* 6. block-edge edit: the neighbouring block's edge tile art changes too */
    {
        static Renderer e;
        render_init(&e);
        map_set(&e, 0, 15, 17, 1);   /* right edge cell of block (1,2) */
        render_frame(&e, 0, 0, MW - 1, MH - 1);
        map_set(&e, 0, 16, 17, 1);   /* left edge of block (2,2) */
        render_frame(&e, 0, 0, MW - 1, MH - 1);
        render_reference(&e, ref);
        printf("edge_add rebuild=%ld query=%ld gen=%ld\n",
               e.fstat.rebuilt, e.fstat.queries, e.fstat.gens);
        check("edge_rebuild_two", e.fstat.rebuilt == 2);
        check("edge_rebuild_both_generate", e.fstat.queries == 2 && e.fstat.gens == 2);
        check("edge_pixels", compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1) == 0);

        /* id-only change on the edge: both re-stitch, but the neighbour's art
           only depends on presence, so it is not re-queried */
        map_set(&e, 0, 16, 17, 2);
        render_frame(&e, 0, 0, MW - 1, MH - 1);
        render_reference(&e, ref);
        printf("edge_id rebuild=%ld query=%ld miss=%ld\n",
               e.fstat.rebuilt, e.fstat.queries, e.fstat.misses);
        check("edge_id_rebuild_two", e.fstat.rebuilt == 2);
        check("edge_id_one_query", e.fstat.queries == 1);
        check("edge_id_pixels", compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1) == 0);

        map_set(&e, 0, 16, 17, 0);
        render_frame(&e, 0, 0, MW - 1, MH - 1);
        render_reference(&e, ref);
        printf("edge_remove rebuild=%ld inval=%ld\n", e.fstat.rebuilt, e.fstat.invalidated);
        check("edge_remove_rebuild_two", e.fstat.rebuilt == 2);
        check("edge_remove_invalidates", e.fstat.invalidated == 2);
        check("edge_remove_pixels", compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1) == 0);
    }

    /* 7. block-corner edit touches self, left and top — not the diagonal */
    {
        static Renderer k;
        render_init(&k);
        map_set(&k, 0, 14, 22, 1);   /* a live cell in the diagonal block (1,2) */
        map_set(&k, 0, 15, 24, 1);   /* right edge cell of block (1,3) */
        map_set(&k, 0, 16, 23, 1);   /* bottom edge cell of block (2,2) */
        render_frame(&k, 0, 0, MW - 1, MH - 1);
        map_set(&k, 0, 16, 24, 1);   /* corner (0,0) of block (2,3) */
        render_frame(&k, 0, 0, MW - 1, MH - 1);
        render_reference(&k, ref);
        printf("corner_add rebuild=%ld gen=%ld\n", k.fstat.rebuilt, k.fstat.gens);
        check("corner_rebuild_three", k.fstat.rebuilt == 3);
        check("corner_three_generate", k.fstat.gens == 3);
        check("corner_pixels", compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1) == 0);

        /* the diagonal block (1,2) is never touched by either edit */
        check("corner_diagonal_untouched", g_blk[blk_index(0, 1, 2)].slot >= 0 &&
                                           g_blk[blk_index(0, 1, 2)].need == 0);

        map_set(&k, 0, 16, 24, 0);
        render_frame(&k, 0, 0, MW - 1, MH - 1);
        render_reference(&k, ref);
        check("corner_remove_rebuild_three", k.fstat.rebuilt == 3);
        check("corner_remove_pixels", compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1) == 0);
    }

    /* 8. tiled terrain: repeated blocks reuse variants, runs collapse to fewer
          draw commands than tiles */
    {
        static Renderer t;
        render_init(&t);
        fill_terrain(&t);
        render_frame(&t, 0, 0, MW - 1, MH - 1);
        render_reference(&t, ref);
        check("terrain_pixels", compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1) == 0);
        long tq = t.cstat.queries;
        printf("terrain blocks=%ld variants=%ld hit_rate=%.4f\n",
               tq, t.cstat.gens, tq ? (double)t.cstat.hits / (double)tq : 1.0);
        printf("terrain draw: tiles=%ld cmds=%ld tiles_per_cmd=%.2f\n",
               t.fstat.tiles, t.fstat.cmds,
               (double)t.fstat.tiles / (double)t.fstat.cmds);
        check("terrain_variants_deduped", t.cstat.gens < tq);
        check("terrain_cmds_fewer_than_tiles", t.fstat.cmds < t.fstat.tiles);

        long gens_before_t = t.cstat.gens;
        map_set(&t, 0, 22, 22, 0);   /* one interior cell, patch has a tile here */
        render_frame(&t, 0, 0, MW - 1, MH - 1);
        render_reference(&t, ref);
        printf("terrain edit rebuild=%ld gen_delta=%ld invalidated=%ld\n",
               t.fstat.rebuilt, t.cstat.gens - gens_before_t, t.fstat.invalidated);
        check("terrain_edit_rebuild_one", t.fstat.rebuilt == 1);
        check("terrain_edit_pixels", compare_pixels(g_canvas, ref, 0, 0, MW - 1, MH - 1) == 0);

        /* a steady frame after the edit does no work at all */
        render_frame(&t, 0, 0, MW - 1, MH - 1);
        check("terrain_settles", t.fstat.rebuilt == 0 && t.fstat.queries == 0 &&
                                 t.fstat.invalidated == 0);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
