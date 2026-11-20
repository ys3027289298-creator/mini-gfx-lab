
/* atlasstream: chunked atlas streaming with mip-level fallback.
 *
 * The atlas is too large to load at once, so the fine mip levels are split
 * into fixed-size chunks that stream in by region.  While a chunk is
 * missing the sampler falls back to the parent level (coarser chunk, then
 * the always-resident base pyramid).  A chunk is decoded into a private
 * buffer and only published at a frame-boundary commit, fully decoded, so
 * the substitute is replaced atomically: no frame ever samples partial
 * data, and per-region detail never regresses (no flicker).
 *
 * The "disk" here is a procedural image so any region/level can be read
 * independently; chunk decode and the base pyramid therefore agree
 * bit-exactly, and the converged picture is pixel-identical to loading
 * the whole atlas up front.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ATLAS_W 2048
#define ATLAS_H 2048
#define LEVELS 12               /* 2048, 1024, ..., 1 */
#define CHUNKED_LEVELS 2        /* levels 0,1 are chunked; >=2 fully resident */
#define BPP 4
#define DECODE_ROWS_PER_TICK 32 /* simulated async decode speed */
#define MAX_CHUNKS 2048         /* request queue capacity (> total chunks) */

/* ---- tracked allocator: measures current/peak atlas memory ---- */
static size_t mem_current, mem_peak;
static void* amalloc(size_t n) {
    void* p = malloc(n);
    if (p) {
        mem_current += n;
        if (mem_current > mem_peak) mem_peak = mem_current;
    }
    return p;
}
static void afree(void* p, size_t n) {
    free(p);
    mem_current -= n;
}

/* ---- "disk" image: deterministic, any texel of any level on demand ---- */
static unsigned hash2(unsigned x, unsigned y) {
    unsigned h = x * 0x8da6b343u ^ y * 0xd8163841u;
    h ^= h >> 15; h *= 0x2c1b3c6du;
    h ^= h >> 12; h *= 0x297a2d39u;
    h ^= h >> 15;
    return h;
}

static void disk_pixel_l0(int x, int y, unsigned char out[4]) {
    unsigned b = hash2((unsigned)(x / 8), (unsigned)(y / 8)); /* coarse blocks */
    unsigned d = hash2((unsigned)x, (unsigned)y);             /* fine detail */
    out[0] = (unsigned char)((b & 0xff) / 2 + (d & 0xff) / 2);
    out[1] = (unsigned char)(((b >> 8) & 0xff) / 2 + ((d >> 8) & 0xff) / 2);
    out[2] = (unsigned char)(((b >> 16) & 0xff) / 2 + ((d >> 16) & 0xff) / 2);
    out[3] = 255;
}

/* mip texel = exact 2^L x 2^L box average of level 0 */
static void disk_pixel(int level, int x, int y, unsigned char out[4]) {
    if (level == 0) { disk_pixel_l0(x, y, out); return; }
    int s = 1 << level;
    unsigned acc[4] = {0, 0, 0, 0};
    unsigned char px[4];
    for (int dy = 0; dy < s; dy++)
        for (int dx = 0; dx < s; dx++) {
            disk_pixel_l0(x * s + dx, y * s + dy, px);
            for (int c = 0; c < 4; c++) acc[c] += px[c];
        }
    unsigned n = (unsigned)(s * s);
    for (int c = 0; c < 4; c++) out[c] = (unsigned char)((acc[c] + n / 2) / n);
}

/* ---- chunk / level / atlas ---- */
enum { CH_MISSING, CH_QUEUED, CH_LOADING, CH_DECODED, CH_READY };

typedef struct {
    int state;
    unsigned char* px;      /* cs*cs*BPP while resident */
    int rows_done;          /* decode progress; invisible until commit */
    unsigned long lru;
    int pinned;             /* used by the frame in flight: not evictable */
} Chunk;

typedef struct {
    int w, h;               /* texels at this level */
    int gw, gh;             /* chunk grid (chunked levels) */
    Chunk* chunks;
    unsigned char* full;    /* resident base pyramid levels */
} Level;

typedef struct {
    int cs;                 /* chunk edge in texels */
    int cache_chunks;       /* max resident chunks */
    Level lv[LEVELS];
    int resident;           /* chunks with px allocated */
    unsigned long clock;    /* LRU clock */
    int* q;                 /* request queue, pairs of (level, idx) */
    int qh, qt, qc;
    long samples, fallback_samples;
} Atlas;

static size_t chunk_bytes(const Atlas* a) {
    return (size_t)a->cs * a->cs * BPP;
}

/* base pyramid is identical for every atlas instance: build once, memcpy */
static unsigned char* g_base[LEVELS];
static void build_base_once(void) {
    for (int L = CHUNKED_LEVELS; L < LEVELS; L++) {
        int w = ATLAS_W >> L, h = ATLAS_H >> L;
        g_base[L] = malloc((size_t)w * h * BPP);
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++)
                disk_pixel(L, x, y, g_base[L] + ((size_t)y * w + x) * BPP);
    }
}

static void atlas_init(Atlas* a, int cs, int cache_chunks) {
    memset(a, 0, sizeof(*a));
    a->cs = cs;
    a->cache_chunks = cache_chunks;
    for (int L = 0; L < LEVELS; L++) {
        a->lv[L].w = ATLAS_W >> L;
        a->lv[L].h = ATLAS_H >> L;
        if (L < CHUNKED_LEVELS) {
            a->lv[L].gw = a->lv[L].w / cs;
            a->lv[L].gh = a->lv[L].h / cs;
            a->lv[L].chunks = calloc((size_t)a->lv[L].gw * a->lv[L].gh, sizeof(Chunk));
        } else {
            size_t n = (size_t)a->lv[L].w * a->lv[L].h * BPP;
            a->lv[L].full = amalloc(n);
            memcpy(a->lv[L].full, g_base[L], n);
        }
    }
    a->qc = MAX_CHUNKS;
    a->q = calloc((size_t)a->qc * 2, sizeof(int));
}

static void atlas_free(Atlas* a) {
    for (int L = 0; L < LEVELS; L++) {
        if (a->lv[L].chunks) {
            int n = a->lv[L].gw * a->lv[L].gh;
            for (int i = 0; i < n; i++)
                if (a->lv[L].chunks[i].px)
                    afree(a->lv[L].chunks[i].px, chunk_bytes(a));
            free(a->lv[L].chunks);
        }
        if (a->lv[L].full)
            afree(a->lv[L].full, (size_t)a->lv[L].w * a->lv[L].h * BPP);
    }
    free(a->q);
}

static void atlas_clear_pins(Atlas* a) {
    for (int L = 0; L < CHUNKED_LEVELS; L++) {
        int n = a->lv[L].gw * a->lv[L].gh;
        for (int i = 0; i < n; i++) a->lv[L].chunks[i].pinned = 0;
    }
}

/* evict the least recently used ready, unpinned chunk; 0 if none evictable */
static int evict_one(Atlas* a) {
    Chunk* best = NULL;
    for (int L = 0; L < CHUNKED_LEVELS; L++) {
        int n = a->lv[L].gw * a->lv[L].gh;
        for (int i = 0; i < n; i++) {
            Chunk* c = &a->lv[L].chunks[i];
            if (c->state == CH_READY && !c->pinned && (!best || c->lru < best->lru))
                best = c;
        }
    }
    if (!best) return 0;
    afree(best->px, chunk_bytes(a));
    best->px = NULL;
    best->state = CH_MISSING;
    best->rows_done = 0;
    a->resident--;
    return 1;
}

static void atlas_request(Atlas* a, int level, int idx) {
    Chunk* c = &a->lv[level].chunks[idx];
    if (c->state != CH_MISSING) return;
    if (a->resident >= a->cache_chunks && !evict_one(a))
        return; /* cache full of pinned/loading chunks: retried next frame */
    c->px = amalloc(chunk_bytes(a));
    a->resident++;
    c->state = CH_QUEUED;
    c->rows_done = 0;
    /* total chunks <= 1280 < qc, and states dedup requests: never full */
    a->q[a->qt * 2] = level;
    a->q[a->qt * 2 + 1] = idx;
    a->qt = (a->qt + 1) % a->qc;
}

/* loader: decode up to DECODE_ROWS_PER_TICK rows of "disk" data per tick */
static void atlas_tick(Atlas* a) {
    int budget = DECODE_ROWS_PER_TICK;
    while (budget > 0 && a->qh != a->qt) {
        int level = a->q[a->qh * 2], idx = a->q[a->qh * 2 + 1];
        Chunk* c = &a->lv[level].chunks[idx];
        int cs = a->cs;
        int rows = cs - c->rows_done;
        if (rows > budget) rows = budget;
        int gx = idx % a->lv[level].gw, gy = idx / a->lv[level].gw;
        c->state = CH_LOADING;
        for (int r = 0; r < rows; r++) {
            int y = c->rows_done + r;
            for (int x = 0; x < cs; x++) {
                unsigned char px[BPP];
                disk_pixel(level, gx * cs + x, gy * cs + y, px);
                memcpy(c->px + ((size_t)y * cs + x) * BPP, px, BPP);
            }
        }
        c->rows_done += rows;
        budget -= rows;
        if (c->rows_done == cs) {
            c->state = CH_DECODED; /* published at the next commit */
            a->qh = (a->qh + 1) % a->qc;
        }
    }
}

/* frame-boundary commit: fully decoded chunks become visible atomically */
static void atlas_commit(Atlas* a) {
    for (int L = 0; L < CHUNKED_LEVELS; L++) {
        int n = a->lv[L].gw * a->lv[L].gh;
        for (int i = 0; i < n; i++)
            if (a->lv[L].chunks[i].state == CH_DECODED)
                a->lv[L].chunks[i].state = CH_READY;
    }
}

/* sample atlas texel (x,y); returns the level actually used (0 = full) */
static int atlas_sample(Atlas* a, int x, int y, unsigned char out[4]) {
    int miss_lvl[CHUNKED_LEVELS], miss_idx[CHUNKED_LEVELS], nm = 0;
    int level = CHUNKED_LEVELS;
    for (int L = 0; L < CHUNKED_LEVELS; L++) {
        int t = x >> L, u = y >> L;
        int cx = t / a->cs, cy = u / a->cs;
        int idx = cy * a->lv[L].gw + cx;
        Chunk* c = &a->lv[L].chunks[idx];
        if (c->state == CH_READY) {
            int lx = t - cx * a->cs, ly = u - cy * a->cs;
            memcpy(out, c->px + ((size_t)ly * a->cs + lx) * BPP, BPP);
            c->lru = ++a->clock;
            c->pinned = 1;
            level = L;
            break;
        }
        miss_lvl[nm] = L;
        miss_idx[nm] = idx;
        nm++;
    }
    if (level == CHUNKED_LEVELS) {
        /* base pyramid is always resident: the fallback chain never fails */
        int L = CHUNKED_LEVELS;
        memcpy(out, a->lv[L].full + ((size_t)(y >> L) * a->lv[L].w + (x >> L)) * BPP, BPP);
        a->fallback_samples++;
    }
    /* request missing chunks coarsest-first so the substitute arrives early */
    for (int i = nm - 1; i >= 0; i--) atlas_request(a, miss_lvl[i], miss_idx[i]);
    a->samples++;
    return level;
}

static void render(Atlas* a, unsigned char* fb, int fw, int fh, int vx, int vy,
                   int* chunk_level, int* miss_count) {
    for (int j = 0; j < fh; j++)
        for (int i = 0; i < fw; i++) {
            int x = vx + i, y = vy + j;
            if (x < 0) x = 0;
            if (x >= ATLAS_W) x = ATLAS_W - 1;
            if (y < 0) y = 0;
            if (y >= ATLAS_H) y = ATLAS_H - 1;
            unsigned char px[BPP];
            int L = atlas_sample(a, x, y, px);
            memcpy(fb + ((size_t)j * fw + i) * BPP, px, BPP);
            if (L > 0 && miss_count) (*miss_count)++;
            if (chunk_level) {
                int g = ATLAS_W / a->cs;
                chunk_level[(y / a->cs) * g + x / a->cs] = L;
            }
        }
}

/* ---- reference: whole atlas loaded up front ---- */
static unsigned char* g_ref;
static void build_ref_once(void) {
    g_ref = malloc((size_t)ATLAS_W * ATLAS_H * BPP);
    for (int y = 0; y < ATLAS_H; y++)
        for (int x = 0; x < ATLAS_W; x++)
            disk_pixel_l0(x, y, g_ref + ((size_t)y * ATLAS_W + x) * BPP);
}

static void render_ref(unsigned char* fb, int fw, int fh, int vx, int vy) {
    for (int j = 0; j < fh; j++)
        for (int i = 0; i < fw; i++) {
            int x = vx + i, y = vy + j;
            if (x < 0) x = 0;
            if (x >= ATLAS_W) x = ATLAS_W - 1;
            if (y < 0) y = 0;
            if (y >= ATLAS_H) y = ATLAS_H - 1;
            memcpy(fb + ((size_t)j * fw + i) * BPP,
                   g_ref + ((size_t)y * ATLAS_W + x) * BPP, BPP);
        }
}

/* ---- self test ---- */
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* stream a viewport until no fallback pixels remain; 0 on timeout */
static int converge(Atlas* a, unsigned char* fb, int fw, int fh, int vx, int vy) {
    for (int it = 0; it < 100000; it++) {
        int miss = 0;
        atlas_clear_pins(a);
        render(a, fb, fw, fh, vx, vy, NULL, &miss);
        if (miss == 0) return 1;
        atlas_tick(a);
        atlas_commit(a);
    }
    return 0;
}

static void test_fallback_chain(void) {
    Atlas a;
    unsigned char px[4], want[4];
    int L;
    atlas_init(&a, 128, 400);

    L = atlas_sample(&a, 1000, 700, px);
    disk_pixel(2, 250, 175, want);
    check("fallback_to_base", L == 2 && memcmp(px, want, BPP) == 0);

    atlas_tick(&a); atlas_tick(&a); atlas_commit(&a);
    L = atlas_sample(&a, 1000, 700, px);
    check("no_partial_during_decode", L == 2); /* half-decoded L1 invisible */

    atlas_tick(&a); atlas_tick(&a); atlas_commit(&a);
    L = atlas_sample(&a, 1000, 700, px);
    disk_pixel(1, 500, 350, want);
    check("parent_substitute_exact", L == 1 && memcmp(px, want, BPP) == 0);

    for (int i = 0; i < 4; i++) atlas_tick(&a);
    atlas_commit(&a);
    L = atlas_sample(&a, 1000, 700, px);
    disk_pixel_l0(1000, 700, want);
    check("refine_to_full_exact", L == 0 && memcmp(px, want, BPP) == 0);

    L = atlas_sample(&a, 100, 100, px);
    check("unrelated_region_still_coarse", L == 2);
    atlas_free(&a);
}

static void test_pixel_identical(void) {
    static unsigned char fb[160 * 120 * BPP], rf[160 * 120 * BPP];
    static const int views[3][2] = {{0, 0}, {700, 300}, {1900, 1900}};
    Atlas a;
    atlas_init(&a, 128, 400);
    for (int v = 0; v < 3; v++) {
        char name[64];
        int ok = converge(&a, fb, 160, 120, views[v][0], views[v][1]);
        render_ref(rf, 160, 120, views[v][0], views[v][1]);
        snprintf(name, sizeof(name), "converged_pixel_identical_%d", v);
        check(name, ok && memcmp(fb, rf, sizeof(fb)) == 0);
    }
    atlas_free(&a);
}

static void test_no_flicker(void) {
    static unsigned char fb[160 * 120 * BPP];
    static int finest[256], cur[256];
    Atlas a;
    int mono_ok = 1, first_fb = -1, last_fb = -1;
    atlas_init(&a, 128, 400); /* cache fits everything: no eviction here */
    for (int i = 0; i < 256; i++) finest[i] = 99;
    for (int f = 0; f < 80; f++) {
        int vx = f * 8;
        int miss = 0;
        if (vx > 472) vx = 472; /* stop scrolling, let the loader catch up */
        atlas_clear_pins(&a);
        for (int i = 0; i < 256; i++) cur[i] = -1;
        render(&a, fb, 160, 120, vx, 100, cur, &miss);
        if (first_fb < 0) first_fb = miss;
        last_fb = miss;
        for (int i = 0; i < 256; i++)
            if (cur[i] >= 0) {
                if (cur[i] > finest[i]) mono_ok = 0; /* detail regressed: flicker */
                if (cur[i] < finest[i]) finest[i] = cur[i];
            }
        atlas_tick(&a);
        atlas_commit(&a);
    }
    check("monotonic_no_flicker", mono_ok);
    check("final_frame_full_detail", last_fb == 0);
    printf("fallback_pixels first_frame=%d last_frame=%d\n", first_fb, last_fb);
    atlas_free(&a);
}

static void test_memory_report(void) {
    static unsigned char fb[64 * 64 * BPP];
    static const int css[] = {64, 128, 256};
    static const int caps[] = {16, 32, 64};
    size_t full = 0;
    int bound_ok = 1, linear_ok = 1;
    for (int L = 0; L < LEVELS; L++)
        full += (size_t)(ATLAS_W >> L) * (ATLAS_H >> L) * BPP;
    printf("-- memory report: peak vs resident chunk count --\n");
    printf("full_load_bytes=%zu (all %d levels, %.1f MiB)\n", full, LEVELS,
           (double)full / 1048576.0);
    printf("chunk_size cache_chunks total_chunks peak_bytes predicted bound_ok pct_of_full\n");
    for (int ci = 0; ci < 3; ci++)
        for (int ki = 0; ki < 3; ki++) {
            int cs = css[ci], cap = caps[ki];
            Atlas a;
            size_t base, predicted;
            int total = 0;
            mem_current = 0;
            mem_peak = 0;
            atlas_init(&a, cs, cap);
            base = mem_current;
            for (int L = 0; L < CHUNKED_LEVELS; L++)
                total += a.lv[L].gw * a.lv[L].gh;
            for (int f = 0; f < 60; f++) {
                atlas_clear_pins(&a);
                render(&a, fb, 64, 64, f * 32, 500, NULL, NULL);
                atlas_tick(&a);
                atlas_commit(&a);
            }
            predicted = base + (size_t)cap * cs * cs * BPP;
            if (mem_peak > predicted) bound_ok = 0;
            /* where the working set exceeds the cache, peak is exactly linear */
            if ((cs == 64 || (cs == 128 && cap <= 32) || (cs == 256 && cap == 16))
                && mem_peak != predicted)
                linear_ok = 0;
            printf("%9d %12d %12d %10zu %10zu %8d %11.2f\n", cs, cap, total,
                   mem_peak, predicted, mem_peak <= predicted,
                   100.0 * (double)mem_peak / (double)full);
            atlas_free(&a);
        }
    printf("relation: peak = base_pyramid + resident_chunks * chunk_bytes"
           " (exact while working_set >= cache_chunks)\n");
    check("memory_within_bound", bound_ok);
    check("memory_linear_in_chunks", linear_ok);
}

int main(void) {
    build_base_once();
    build_ref_once();
    test_fallback_chain();
    test_pixel_identical();
    test_no_flicker();
    test_memory_report();
    free(g_ref);
    for (int L = CHUNKED_LEVELS; L < LEVELS; L++) free(g_base[L]);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
