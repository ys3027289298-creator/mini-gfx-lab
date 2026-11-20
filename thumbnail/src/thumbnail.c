/* thumbnail-cache: resolution ladder + capped LRU cache for thumbnails.

   A request names the display width it needs. The source keeps a ladder of
   power-of-two-ish widths built by ceil-halving (so odd sizes still line up).
   The nearest ladder level is chosen; a cache hit returns the stored pixels,
   a miss synthesises the level by halving the nearest cached finer level (or
   the source when nothing finer is cached) and evicts the least recently used
   entry when the cache is full. Because every level is a deterministic box
   filter of the finest level, a cached thumbnail is pixel-identical to one
   generated from scratch. Hit rate and generation counts are tracked, and a
   capacity of zero is a safe no-op. */
#include <stdio.h>
#include <string.h>

#define MAX_W 256
#define MAX_H 256
#define MAX_LEVELS 16
#define MAX_ENTRIES 32

typedef struct { unsigned char r, g, b, a; } RGBA;

typedef struct {
    const char* name;
    int src_w, src_h;
    RGBA src[MAX_H][MAX_W];
    int nlev;
    int lev_w[MAX_LEVELS], lev_h[MAX_LEVELS];   /* ascending, finest last */
} Source;

typedef struct {
    const Source* src;
    int lev;
    int valid;
    unsigned long long tick;
    RGBA pix[MAX_H][MAX_W];
} Entry;

typedef struct {
    Entry e[MAX_ENTRIES];
    int cap;
    unsigned long long clock;
    long hits, misses;   /* requests served / not served from the cache */
    long gen;            /* level buffers synthesised by downsampling */
} Cache;

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* ---------- source / ladder --------------------------------------------- */

static int ceil_half(int v) { return v > 1 ? (v + 1) / 2 : 1; }

static void src_init(Source* s, const char* name, int w, int h, int levels) {
    memset(s, 0, sizeof(*s));
    s->name = name;
    s->src_w = w;
    s->src_h = h;
    s->nlev = levels;
    s->lev_w[levels - 1] = w;
    s->lev_h[levels - 1] = h;
    for (int i = levels - 2; i >= 0; i--) {
        s->lev_w[i] = ceil_half(s->lev_w[i + 1]);
        s->lev_h[i] = ceil_half(s->lev_h[i + 1]);
    }
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            RGBA* p = &s->src[y][x];
            p->r = (unsigned char)((x * 7 + y * 3) & 0xFF);
            p->g = (unsigned char)((x * 5 + y * 11 + 17) & 0xFF);
            p->b = (unsigned char)((x ^ y) & 0xFF);
            p->a = (x + y) % 13 == 0 ? 0 : 255;
        }
}

/* closest ladder level to the requested display width; ties go to the
   coarser level, which needs less memory for the same visible result */
static int src_choose(const Source* s, int display_w) {
    int best = 0;
    int best_d = s->lev_w[0] > display_w ? s->lev_w[0] - display_w : display_w - s->lev_w[0];
    for (int i = 1; i < s->nlev; i++) {
        int d = s->lev_w[i] > display_w ? s->lev_w[i] - display_w : display_w - s->lev_w[i];
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

/* 2x2 box halving; an odd parent reuses its extra row/column */
static void downsample(const RGBA (*src)[MAX_W], int sw, int sh,
                       RGBA (*dst)[MAX_W], int dw, int dh) {
    for (int y = 0; y < dh; y++)
        for (int x = 0; x < dw; x++) {
            int x0 = 2 * x, x1 = 2 * x + 1;
            int y0 = 2 * y, y1 = 2 * y + 1;
            if (x1 >= sw) x1 = x0;
            if (y1 >= sh) y1 = y0;
            const RGBA* a = &src[y0][x0];
            const RGBA* b = &src[y0][x1];
            const RGBA* c = &src[y1][x0];
            const RGBA* d = &src[y1][x1];
            dst[y][x].r = (unsigned char)((a->r + b->r + c->r + d->r + 2) >> 2);
            dst[y][x].g = (unsigned char)((a->g + b->g + c->g + d->g + 2) >> 2);
            dst[y][x].b = (unsigned char)((a->b + b->b + c->b + d->b + 2) >> 2);
            dst[y][x].a = (unsigned char)((a->a + b->a + c->a + d->a + 2) >> 2);
        }
}

/* ---------- cache ------------------------------------------------------- */

static void cache_init(Cache* c, int cap) {
    memset(c, 0, sizeof(*c));
    c->cap = cap;
}

static int cache_find(const Cache* c, const Source* s, int lev) {
    for (int i = 0; i < c->cap; i++)
        if (c->e[i].valid && c->e[i].src == s && c->e[i].lev == lev) return i;
    return -1;
}

/* finest cached level strictly finer than lev, or -1 */
static int cache_finer(const Cache* c, const Source* s, int lev) {
    for (int p = lev + 1; p < s->nlev; p++)
        if (cache_find(c, s, p) >= 0) return p;
    return -1;
}

/* free slot, else the entry with the oldest tick (LRU); -1 when disabled */
static int cache_slot(Cache* c) {
    if (c->cap <= 0) return -1;
    for (int i = 0; i < c->cap; i++) if (!c->e[i].valid) return i;
    int lru = 0;
    for (int i = 1; i < c->cap; i++) if (c->e[i].tick < c->e[lru].tick) lru = i;
    return lru;
}

/* Produce level lev into dst. Starts from the nearest cached finer level so
   only the missing rungs are generated; with no cache at all this walks the
   full chain down from the source. Returns the number of halving steps. */
static int level_build(Source* s, const Cache* c, int lev, RGBA (*dst)[MAX_W]) {
    static RGBA buf0[MAX_H][MAX_W], buf1[MAX_H][MAX_W];
    RGBA (*cur)[MAX_W] = buf0;
    RGBA (*nxt)[MAX_W] = buf1;

    int from = cache_finer(c, s, lev);
    if (from < 0) from = s->nlev - 1;
    if (from == s->nlev - 1) {
        memcpy(cur, s->src, sizeof(s->src));
    } else {
        int idx = cache_find(c, s, from);
        memcpy(cur, c->e[idx].pix, sizeof(buf0));
    }

    int steps = 0;
    for (int k = from - 1; k >= lev; k--) {
        downsample((const RGBA(*)[MAX_W])cur, s->lev_w[k + 1], s->lev_h[k + 1],
                   nxt, s->lev_w[k], s->lev_h[k]);
        RGBA (*t)[MAX_W] = cur; cur = nxt; nxt = t;
        steps++;
    }
    memcpy(dst, cur, sizeof(buf0));
    return steps;
}

/* Resolve display_width to a ladder level and hand back the pixels, counting
   the request as a hit or a miss. A hit promotes the entry to most-recently
   used; a miss stores the synthesised level, evicting LRU if full. */
static const RGBA (*cache_lookup(Cache* c, Source* s, int display_width, int* out_lev))[MAX_W] {
    int lev = src_choose(s, display_width);
    *out_lev = lev;
    int idx = cache_find(c, s, lev);
    if (idx >= 0) {
        c->hits++;
        c->e[idx].tick = ++c->clock;
        return (const RGBA(*)[MAX_W])c->e[idx].pix;
    }
    c->misses++;
    int slot = cache_slot(c);
    static RGBA built[MAX_H][MAX_W];
    c->gen += level_build(s, c, lev, built);
    if (slot >= 0) {
        Entry* e = &c->e[slot];
        e->src = s;
        e->lev = lev;
        e->valid = 1;
        e->tick = ++c->clock;
        memcpy(e->pix, built, sizeof(built));
    }
    return (const RGBA(*)[MAX_W])built;
}

/* uncached path: no cache, no counters, just generate the chosen level */
static const RGBA (*fresh_render(Source* s, int display_width, int* out_lev))[MAX_W] {
    static Cache none;
    static RGBA built[MAX_H][MAX_W];
    int lev = src_choose(s, display_width);
    *out_lev = lev;
    level_build(s, &none, lev, built);   /* cap 0, so always from the source */
    return (const RGBA(*)[MAX_W])built;
}

static int pix_equal(const RGBA(*a)[MAX_W], const RGBA(*b)[MAX_W], int w, int h) {
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (a[y][x].r != b[y][x].r || a[y][x].g != b[y][x].g ||
                a[y][x].b != b[y][x].b || a[y][x].a != b[y][x].a) return 0;
    return 1;
}

static int valid_entries(const Cache* c) {
    int n = 0;
    for (int i = 0; i < c->cap; i++) n += c->e[i].valid;
    return n;
}

int main(void) {
    /* pixel buffers are hundreds of kilobytes; keep them off the stack */
    static Source a, b;
    static Cache c;

    /* odd dimensions exercise the ceil-halving path */
    src_init(&a, "alpha", 200, 160, 5);   /* ascending 13 25 50 100 200 */
    src_init(&b, "beta", 255, 170, 6);    /* ascending 8 16 32 64 128 255 */

    /* ladder */
    check("ladder_ascending", a.lev_w[0] < a.lev_w[1] && a.lev_w[4] == 200);
    check("ladder_halves", a.lev_w[3] == 100 && a.lev_w[2] == 50 && a.lev_w[1] == 25);
    check("ladder_odd_ceil", b.lev_w[4] == 128 && b.lev_w[3] == 64 && b.lev_w[0] == 8);
    check("ladder_finest_is_src", a.lev_w[a.nlev - 1] == a.src_w && b.lev_w[b.nlev - 1] == b.src_w);
    check("ladder_heights", a.lev_h[2] == 40 && a.lev_h[4] == 160 &&
          b.lev_h[3] == 43 && b.lev_h[0] == 6);

    /* nearest-level selection */
    check("pick_exact", src_choose(&a, 100) == 3);
    check("pick_nearest_below", src_choose(&a, 90) == 3);
    check("pick_nearest_above", src_choose(&a, 60) == 2);
    check("pick_oversize_clamps", src_choose(&a, 1000) == 4);
    check("pick_tiny_clamps", src_choose(&a, 1) == 0);
    check("pick_tie_goes_coarse", src_choose(&a, 75) == 2); /* 50/100 tie -> 50 */

    /* uncached generation matches the declared ladder exactly */
    int lev, gen_lev;
    int sizes_ok = 1, levels_differ = 1;
    for (lev = 0; lev < a.nlev; lev++) {
        const RGBA(*p)[MAX_W] = fresh_render(&a, a.lev_w[lev], &gen_lev);
        if (gen_lev != lev) sizes_ok = 0;
        (void)p;
    }
    check("fresh_hits_each_level", sizes_ok);
    /* fresh_render reuses an internal buffer, so copy before holding one */
    static RGBA box_buf[MAX_H][MAX_W], l2_buf[MAX_H][MAX_W], again_buf[MAX_H][MAX_W];
    memcpy(box_buf, fresh_render(&a, 100, &gen_lev), sizeof(box_buf)); /* level 3 */
    const RGBA(*box)[MAX_W] = (const RGBA(*)[MAX_W])box_buf;
    RGBA got = box[0][0];
    check("box_average_pixel",
          got.r == (unsigned char)((a.src[0][0].r + a.src[0][1].r + a.src[1][0].r + a.src[1][1].r + 2) >> 2) &&
          got.g == (unsigned char)((a.src[0][0].g + a.src[0][1].g + a.src[1][0].g + a.src[1][1].g + 2) >> 2) &&
          got.b == (unsigned char)((a.src[0][0].b + a.src[0][1].b + a.src[1][0].b + a.src[1][1].b + 2) >> 2) &&
          got.a == (unsigned char)((a.src[0][0].a + a.src[0][1].a + a.src[1][0].a + a.src[1][1].a + 2) >> 2));
    /* odd-size level from the 255-wide source keeps its full extent */
    const RGBA(*odd)[MAX_W] = fresh_render(&b, 128, &gen_lev);
    (void)odd;
    check("odd_level_dimensions", gen_lev == 4 && b.lev_w[4] == 128 && b.lev_h[4] == 85);
    /* neighbouring levels are actually distinct images */
    memcpy(l2_buf, fresh_render(&a, 50, &gen_lev), sizeof(l2_buf));
    const RGBA(*l2)[MAX_W] = (const RGBA(*)[MAX_W])l2_buf;
    levels_differ = !pix_equal(box, l2, 50, 40);
    check("levels_differ", levels_differ);
    /* generation is deterministic */
    memcpy(again_buf, fresh_render(&a, 100, &gen_lev), sizeof(again_buf));
    const RGBA(*again)[MAX_W] = (const RGBA(*)[MAX_W])again_buf;
    check("generation_deterministic", pix_equal(box, again, 100, 80));

    /* hit / miss accounting: repeated requests reuse the level */
    cache_init(&c, 8);
    for (int i = 0; i < 5; i++) cache_lookup(&c, &a, 100, &lev);
    check("counters_hits", c.hits == 4 && c.misses == 1);
    check("counters_gen", c.gen == 1);          /* one 200->100 halving */
    check("hit_rate_roundtrip", c.hits * 100 / (c.hits + c.misses) == 80);

    /* cached pixels must equal a fresh render, pixel for pixel */
    int cache_lev, fresh_lev;
    const RGBA(*cached)[MAX_W] = cache_lookup(&c, &a, 40, &cache_lev);
    const RGBA(*fresh)[MAX_W] = fresh_render(&a, 40, &fresh_lev);
    check("hit_and_fresh_same_level", cache_lev == fresh_lev);
    check("pixel_equal_small", pix_equal(cached, fresh, a.lev_w[cache_lev], a.lev_h[cache_lev]));

    /* every display size agrees with the uncached path, hits and misses alike */
    int all_same = 1;
    for (int dw = 1; dw <= 256; dw += 7) {
        const RGBA(*x)[MAX_W] = cache_lookup(&c, &a, dw, &cache_lev);
        const RGBA(*y)[MAX_W] = fresh_render(&a, dw, &fresh_lev);
        if (cache_lev != fresh_lev ||
            !pix_equal(x, y, a.lev_w[cache_lev], a.lev_h[cache_lev])) all_same = 0;
    }
    check("pixel_equal_all_sizes", all_same);
    check("sizes_respect_capacity", valid_entries(&c) <= 8);
    long hit_rate = c.hits * 100 / (c.hits + c.misses);
    printf("cache hits=%ld misses=%ld gen=%ld hit_rate=%ld%% entries=%d/%d\n",
           c.hits, c.misses, c.gen, hit_rate, valid_entries(&c), c.cap);

    /* capacity: only cap entries live, and eviction is least-recently-used */
    cache_init(&c, 3);
    cache_lookup(&c, &a, 100, &lev);  /* M3, built from the source */
    cache_lookup(&c, &a, 40,  &lev);  /* M2, built from cached M3 */
    cache_lookup(&c, &a, 20,  &lev);  /* M1, built from cached M2 */
    check("cap_respected", valid_entries(&c) == 3);
    check("gen_incremental_from_cache", c.gen == 3); /* 2 + 1 + 1 halvings */
    cache_lookup(&c, &a, 100, &lev);  /* touch M3: M2 becomes LRU */
    cache_lookup(&c, &a, 13,  &lev);  /* M0 evicts M2 */
    check("evict_lru", cache_find(&c, &a, 2) < 0);
    check("evict_kept_recent", cache_find(&c, &a, 3) >= 0 &&
          cache_find(&c, &a, 1) >= 0 && cache_find(&c, &a, 0) >= 0);
    check("cap_respected_after_evict", valid_entries(&c) == 3);
    /* an evicted level is a miss again and is re-synthesised, not resurrected */
    static RGBA re_buf[MAX_H][MAX_W], want_buf[MAX_H][MAX_W];
    long misses_before = c.misses, gen_before = c.gen;
    memcpy(re_buf, cache_lookup(&c, &a, 40, &lev), sizeof(re_buf));
    const RGBA(*re)[MAX_W] = (const RGBA(*)[MAX_W])re_buf;
    check("evicted_is_miss", c.misses == misses_before + 1);
    check("evicted_regenerates", c.gen > gen_before);
    memcpy(want_buf, fresh_render(&a, 40, &fresh_lev), sizeof(want_buf));
    const RGBA(*want)[MAX_W] = (const RGBA(*)[MAX_W])want_buf;
    check("evicted_same_pixels", pix_equal(re, want, a.lev_w[lev], a.lev_h[lev]));

    /* capacity one: strict thrash, every alternating request is a miss */
    cache_init(&c, 1);
    cache_lookup(&c, &a, 100, &lev);
    cache_lookup(&c, &a, 40, &lev);
    cache_lookup(&c, &a, 100, &lev);
    cache_lookup(&c, &a, 40, &lev);
    check("cap1_misses", c.misses == 4 && c.hits == 0);
    check("cap1_one_entry", valid_entries(&c) == 1);

    /* display sizes between levels collapse onto the same neighbour, so
       distinct requests still score hits even with a tiny cache */
    cache_init(&c, 2);
    cache_lookup(&c, &a, 95, &lev);
    cache_lookup(&c, &a, 98, &lev);
    cache_lookup(&c, &a, 103, &lev);
    check("neighbour_display_hits", c.hits == 2 && c.misses == 1);
    check("neighbour_cap_respected", valid_entries(&c) == 1);

    /* zero capacity: safe no-op, every request a miss, nothing stored */
    cache_init(&c, 0);
    static RGBA z_buf[MAX_H][MAX_W], z_want_buf[MAX_H][MAX_W];
    memcpy(z_buf, cache_lookup(&c, &a, 100, &lev), sizeof(z_buf));
    const RGBA(*z)[MAX_W] = (const RGBA(*)[MAX_W])z_buf;
    check("cap0_no_write", c.misses == 1 && c.hits == 0 && valid_entries(&c) == 0);
    memcpy(z_want_buf, fresh_render(&a, 100, &fresh_lev), sizeof(z_want_buf));
    const RGBA(*z_want)[MAX_W] = (const RGBA(*)[MAX_W])z_want_buf;
    check("cap0_still_correct", pix_equal(z, z_want, a.lev_w[lev], a.lev_h[lev]));

    /* miss count equals unique level requests; generation only synthesises
       the rungs not already present at a finer level */
    cache_init(&c, 16);
    int reqs[] = {64, 64, 32, 128, 64, 32};
    for (unsigned i = 0; i < sizeof(reqs) / sizeof(reqs[0]); i++)
        cache_lookup(&c, &b, reqs[i], &lev);
    check("miss_equals_unique_levels", c.misses == 3);
    check("gen_equals_ladder_pull", c.gen == 4); /* M5->M3 (2), M3->M2 (1), M5->M4 (1) */
    check("hits_equals_rest", c.hits == 3);
    check("all_requests_counted", c.hits + c.misses == 6);
    printf("src_b levels=%d widest=%d\n", b.nlev, b.lev_w[b.nlev - 1]);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
