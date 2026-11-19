
/* mapdelta: map variants stored as base map + region-merged delta layer.
 *
 * A variant (season swap, destruction) keeps the base map untouched and
 * records per-cell edits in a Delta. Dirty 8x8 chunks are merged into
 * horizontal region runs on save, so serialized size grows with the number
 * of changed cells, not the map size. Loading overlays the delta on the
 * base; a base-version mismatch is rejected before anything is applied.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define W 64
#define H 64
#define LAYERS 2
#define CHUNK 8
#define CW (W / CHUNK)
#define CH (H / CHUNK)

#define FULL_MAGIC  0x4D465531u /* 'MFU1' */
#define DELTA_MAGIC 0x4D444C31u /* 'MDL1' */

typedef struct {
    uint8_t cells[LAYERS][H][W];
    uint32_t version;
} Map;

static void map_init(Map* m, uint32_t version) {
    memset(m, 0, sizeof(*m));
    m->version = version;
}

static int in_bounds(int l, int x, int y) {
    return l >= 0 && l < LAYERS && x >= 0 && x < W && y >= 0 && y < H;
}

static void map_set(Map* m, int l, int x, int y, uint8_t id) {
    if (!in_bounds(l, x, y)) return;
    m->cells[l][y][x] = id;
}

static uint8_t map_get(const Map* m, int l, int x, int y) {
    if (!in_bounds(l, x, y)) return 0;
    return m->cells[l][y][x];
}

/* ---- delta recording: sparse per-chunk edit lists ---- */

typedef struct { uint8_t x, y, id; } CellEdit;
typedef struct { CellEdit* v; int n, cap; } ChunkDelta;

typedef struct {
    uint32_t base_version;
    ChunkDelta chunk[LAYERS][CH][CW];
    int edits; /* net changed cells */
} Delta;

static void delta_init(Delta* d, uint32_t base_version) {
    memset(d, 0, sizeof(*d));
    d->base_version = base_version;
}

static void delta_free(Delta* d) {
    for (int l = 0; l < LAYERS; l++)
        for (int cy = 0; cy < CH; cy++)
            for (int cx = 0; cx < CW; cx++)
                free(d->chunk[l][cy][cx].v);
    memset(d, 0, sizeof(*d));
}

static int chunk_find(const ChunkDelta* c, int x, int y) {
    for (int i = 0; i < c->n; i++)
        if (c->v[i].x == x && c->v[i].y == y) return i;
    return -1;
}

/* Record one edit against the base. Editing back to the base value
 * removes the entry, so size tracks net differences. */
static void delta_record(Delta* d, const Map* base, int l, int x, int y, uint8_t id) {
    if (!in_bounds(l, x, y)) return;
    ChunkDelta* c = &d->chunk[l][y / CHUNK][x / CHUNK];
    int idx = chunk_find(c, x, y);
    if (id == map_get(base, l, x, y)) {          /* reverted to base */
        if (idx >= 0) { c->v[idx] = c->v[--c->n]; d->edits--; }
        return;
    }
    if (idx >= 0) { c->v[idx].id = id; return; } /* re-edit: latest wins */
    if (c->n == c->cap) {
        c->cap = c->cap ? c->cap * 2 : 4;
        c->v = realloc(c->v, (size_t)c->cap * sizeof(CellEdit));
        if (!c->v) { fprintf(stderr, "oom\n"); exit(1); }
    }
    c->v[c->n++] = (CellEdit){ (uint8_t)x, (uint8_t)y, id };
    d->edits++;
}

/* ---- region merge: runs of dirty chunks, same layer and chunk row ---- */

static int delta_regions(const Delta* d) {
    int n = 0;
    for (int l = 0; l < LAYERS; l++)
        for (int cy = 0; cy < CH; cy++)
            for (int cx = 0; cx < CW; cx++) {
                if (!d->chunk[l][cy][cx].n) continue;
                n++;
                while (cx < CW && d->chunk[l][cy][cx].n) cx++;
            }
    return n;
}

/* ---- byte IO ---- */

typedef struct { uint8_t* p; size_t cap, len; } Writer;
static void w8 (Writer* w, uint8_t  v) { if (w->len < w->cap) w->p[w->len] = (uint8_t)v; w->len++; }
static void w16(Writer* w, uint16_t v) { w8(w, (uint8_t)v); w8(w, (uint8_t)(v >> 8)); }
static void w32(Writer* w, uint32_t v) { w16(w, (uint16_t)v); w16(w, (uint16_t)(v >> 16)); }

typedef struct { const uint8_t* p; size_t len, pos; int err; } Reader;
static uint8_t r8(Reader* r) {
    if (r->pos >= r->len) { r->err = 1; return 0; }
    return r->p[r->pos++];
}
static uint16_t r16(Reader* r) { uint16_t lo = r8(r); return (uint16_t)(lo | ((uint16_t)r8(r) << 8)); }
static uint32_t r32(Reader* r) { uint32_t lo = r16(r); return lo | ((uint32_t)r16(r) << 16); }

/* delta layout: magic u32, base_version u32, n_regions u16, then per region:
 * layer u8, cy u8, cx0 u8, cx1 u8, count u16, count x {x u8, y u8, id u8}.
 * Total = 10 + 6*regions + 3*changed_cells. */
static size_t delta_save(const Delta* d, uint8_t* buf, size_t cap) {
    Writer w = { buf, cap, 0 };
    w32(&w, DELTA_MAGIC);
    w32(&w, d->base_version);
    w16(&w, (uint16_t)delta_regions(d));
    for (int l = 0; l < LAYERS; l++)
        for (int cy = 0; cy < CH; cy++)
            for (int cx = 0; cx < CW; cx++) {
                if (!d->chunk[l][cy][cx].n) continue;
                int cx0 = cx, n = 0;
                while (cx < CW && d->chunk[l][cy][cx].n) { n += d->chunk[l][cy][cx].n; cx++; }
                w8(&w, (uint8_t)l); w8(&w, (uint8_t)cy);
                w8(&w, (uint8_t)cx0); w8(&w, (uint8_t)(cx - 1));
                w16(&w, (uint16_t)n);
                for (int c = cx0; c < cx; c++) {
                    const ChunkDelta* cd = &d->chunk[l][cy][c];
                    for (int i = 0; i < cd->n; i++) {
                        w8(&w, cd->v[i].x); w8(&w, cd->v[i].y); w8(&w, cd->v[i].id);
                    }
                }
            }
    return w.len;
}

/* full-map layout: magic u32, version u32, raw cells */
static size_t map_save_full(const Map* m, uint8_t* buf, size_t cap) {
    Writer w = { buf, cap, 0 };
    w32(&w, FULL_MAGIC);
    w32(&w, m->version);
    for (size_t i = 0; i < sizeof(m->cells); i++) w8(&w, ((const uint8_t*)m->cells)[i]);
    return w.len;
}

static int map_load_full(const uint8_t* buf, size_t len, Map* out) {
    Reader r = { buf, len, 0, 0 };
    if (r32(&r) != FULL_MAGIC) return -1;
    uint32_t version = r32(&r);
    for (size_t i = 0; i < sizeof(out->cells); i++) ((uint8_t*)out->cells)[i] = r8(&r);
    if (r.err || r.pos != r.len) return -1;
    out->version = version;
    return 0;
}

/* Overlay a serialized delta on base into out.
 * Returns 0 ok, 1 version mismatch (out untouched), -1 corrupt data. */
static int map_restore(const Map* base, const uint8_t* buf, size_t len, Map* out) {
    Reader r = { buf, len, 0, 0 };
    if (r32(&r) != DELTA_MAGIC) return -1;
    uint32_t version = r32(&r);
    int n_regions = r16(&r);
    if (r.err) return -1;
    if (version != base->version) return 1;

    size_t body = r.pos;
    for (int pass = 0; pass < 2; pass++) {   /* pass 0 validate, pass 1 apply */
        r.pos = body; r.err = 0;
        if (pass == 1) {
            memcpy(out->cells, base->cells, sizeof(out->cells));
            out->version = base->version;
        }
        for (int rg = 0; rg < n_regions; rg++) {
            int l = r8(&r), cy = r8(&r), cx0 = r8(&r), cx1 = r8(&r);
            int n = r16(&r);
            if (l >= LAYERS || cy >= CH || cx0 > cx1 || cx1 >= CW) return -1;
            for (int i = 0; i < n; i++) {
                int x = r8(&r), y = r8(&r), id = r8(&r);
                if (r.err) return -1;
                if (x < cx0 * CHUNK || x >= (cx1 + 1) * CHUNK ||
                    y < cy * CHUNK || y >= (cy + 1) * CHUNK) return -1;
                if (pass == 1) out->cells[l][y][x] = (uint8_t)id;
            }
        }
        if (r.err) return -1;
    }
    return r.pos == r.len ? 0 : -1;
}

/* ---- self test ---- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static uint8_t buf[1 << 16];

int main(void) {
    Map base, truth;
    map_init(&base, 1);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            base.cells[0][y][x] = (uint8_t)((x / 8 + y / 8) % 3 + 1); /* terrain 1..3 */
    for (int i = 0; i < 40; i++) map_set(&base, 1, (i * 13) % W, (i * 29) % H, 7); /* decor */

    truth = base;
    Delta d; delta_init(&d, base.version);

    /* season change: terrain 1 -> snow 5; destruction: crater at (40,40) r6 */
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if (map_get(&base, 0, x, y) == 1) { map_set(&truth, 0, x, y, 5); delta_record(&d, &base, 0, x, y, 5); }
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if ((x - 40) * (x - 40) + (y - 40) * (y - 40) <= 36)
                for (int l = 0; l < LAYERS; l++) { map_set(&truth, l, x, y, 0); delta_record(&d, &base, l, x, y, 0); }

    size_t full_size = map_save_full(&truth, buf, sizeof(buf));
    Map rt;
    check("full_roundtrip", map_load_full(buf, full_size, &rt) == 0 &&
          rt.version == truth.version && memcmp(rt.cells, truth.cells, sizeof(rt.cells)) == 0);

    size_t dsize = delta_save(&d, buf, sizeof(buf));
    check("delta_exact_size", dsize == 10 + 6 * (size_t)delta_regions(&d) + 3 * (size_t)d.edits);
    check("delta_smaller_than_full", dsize < full_size);

    Map out;
    check("restore_ok", map_restore(&base, buf, dsize, &out) == 0);
    check("restore_matches_full", memcmp(out.cells, truth.cells, sizeof(out.cells)) == 0);
    check("restore_keeps_version", out.version == base.version);

    /* size proportional to changed cells */
    Delta d1, d2; delta_init(&d1, 1); delta_init(&d2, 1);
    delta_record(&d1, &base, 0, 3, 3, 9);
    for (int i = 0; i < 120; i++) delta_record(&d2, &base, 0, (i * 37) % W, (i * 53) % H, 9);
    size_t s1 = delta_save(&d1, buf, sizeof(buf));
    size_t s2 = delta_save(&d2, buf, sizeof(buf));
    check("one_edit_tiny", s1 == 10 + 6 + 3);
    check("size_linear_in_edits", s2 > s1 && s2 <= 120 * s1);
    delta_free(&d1); delta_free(&d2);

    /* region merge: adjacent dirty chunks collapse into one region */
    Delta dm; delta_init(&dm, 1);
    for (int cx = 0; cx < 3; cx++) delta_record(&dm, &base, 0, cx * CHUNK, 0, 9); /* run of 3 */
    delta_record(&dm, &base, 0, 4 * CHUNK, 0, 9);                                  /* gap, then 1 */
    delta_record(&dm, &base, 1, 0, 0, 9);                                          /* other layer */
    check("region_merge", delta_regions(&dm) == 3);
    delta_free(&dm);

    /* dedupe and revert-to-base */
    Delta dd; delta_init(&dd, 1);
    delta_record(&dd, &base, 0, 5, 5, 8);
    delta_record(&dd, &base, 0, 5, 5, 9);
    delta_record(&dd, &base, 0, 5, 5, 10);
    check("reedit_latest_wins", dd.edits == 1 && dd.chunk[0][0][0].v[0].id == 10);
    delta_record(&dd, &base, 0, 5, 5, map_get(&base, 0, 5, 5));
    check("revert_removes", dd.edits == 0 && delta_regions(&dd) == 0);
    delta_free(&dd);

    /* version mismatch rejected, output untouched */
    Map base2 = base; base2.version = 2;
    memset(&out, 0xAA, sizeof(out));
    check("version_mismatch_rejected", map_restore(&base2, buf, 0, &out) != 0); /* sanity: corrupt too */
    size_t dsize2 = delta_save(&d, buf, sizeof(buf));
    check("version_mismatch_rc", map_restore(&base2, buf, dsize2, &out) == 1);
    check("version_mismatch_untouched", out.cells[0][0][0] == 0xAA);

    /* empty delta restores the base itself */
    Delta de; delta_init(&de, 1);
    size_t esz = delta_save(&de, buf, sizeof(buf));
    check("empty_delta_restore", map_restore(&base, buf, esz, &out) == 0 &&
          memcmp(out.cells, base.cells, sizeof(out.cells)) == 0);
    delta_free(&de);

    /* corrupt data rejected */
    check("truncated_rejected", map_restore(&base, buf, dsize2 - 1, &out) == -1);
    buf[0] ^= 0xFF;
    check("bad_magic_rejected", map_restore(&base, buf, dsize2, &out) == -1);

    delta_free(&d);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
