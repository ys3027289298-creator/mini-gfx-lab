
/* tilemap: layered tile map with 4-neighbour autotiling and per-layer culling,
 * plus a multi-page tile atlas: frequency-based page allocation, cross-page
 * reference resolution and cost-based page replacement. */
#include <stdio.h>
#include <string.h>

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

/* =====================================================================
 * Multi-page tile atlas
 *
 * A page is a fixed grid of slots.  The single-page implementation is one
 * 8x8 page that holds every tile id (and can only be rebuilt wholesale once
 * full); the multi-page manager holds several 4x4 pages and replaces only
 * the page that actually misses.
 * ===================================================================== */
#define TILE_PX 8
#define PAGE_COLS 4
#define PAGE_ROWS 4
#define PAGE_SLOTS (PAGE_COLS * PAGE_ROWS)          /* 16 slots per page */
#define ATLAS_MAX_SLOTS 64                          /* single-page grid */
#define MAX_PAGES 4
#define N_TILE_IDS 40                               /* tile ids 1..40 */

#define ATL_MAP_W 24
#define ATL_MAP_H 16
#define VIEW_TW 10                                  /* viewport width in tiles */
#define VIEW_Y0 2
#define VIEW_Y1 13
#define FB_W (ATL_MAP_W * TILE_PX)
#define FB_H (ATL_MAP_H * TILE_PX)
#define N_FRAMES (ATL_MAP_W - VIEW_TW + 2)          /* sliding views + full map */

typedef struct {
    int cols, rows;
    unsigned char slot_id[ATLAS_MAX_SLOTS];         /* 0 = free, else tile id */
} AtlasPage;

static void page_init(AtlasPage* p, int cols, int rows) {
    memset(p, 0, sizeof(*p));
    p->cols = cols;
    p->rows = rows;
}

static int page_find(const AtlasPage* p, unsigned id) {
    for (int s = 0; s < p->cols * p->rows; s++)
        if (p->slot_id[s] == id) return s;
    return -1;
}

static int page_first_free(const AtlasPage* p) {
    for (int s = 0; s < p->cols * p->rows; s++)
        if (p->slot_id[s] == 0) return s;
    return -1;
}

/* deterministic 8x8 pixel pattern for a tile id: both atlas paths must
 * produce exactly these bytes when they blit */
static unsigned char tile_pixel(unsigned id, int lx, int ly) {
    unsigned v = id * 73u + (unsigned)lx * 37u + (unsigned)ly * 131u
               + (id ^ (unsigned)(lx * 7) ^ (unsigned)(ly * 11));
    return (unsigned char)(v & 0xffu);
}

static void blit_tile(unsigned id, int tx, int ty, unsigned char* fb) {
    int ox = tx * TILE_PX, oy = ty * TILE_PX;
    for (int ly = 0; ly < TILE_PX; ly++)
        for (int lx = 0; lx < TILE_PX; lx++)
            fb[(oy + ly) * FB_W + (ox + lx)] = tile_pixel(id, lx, ly);
}

/* single-page reference: one big page assumed to contain every used id */
static void render_rect_single(const TileMap* m, int layer,
                               int x0, int y0, int x1, int y1,
                               const AtlasPage* ref, unsigned char* fb) {
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            unsigned id = map_get(m, layer, x, y);
            if (!id) continue;
            if (page_find(ref, id) < 0) { fprintf(stderr, "single: id %u missing\n", id); return; }
            blit_tile(id, x, y, fb);
        }
    }
}

#define POLICY_COST  0   /* evict min(freq, LRU age) victim */
#define POLICY_NAIVE 1   /* frequency-blind random victim */

typedef struct {
    AtlasPage pages[MAX_PAGES];
    int npages;
    int policy;
    int freq[N_TILE_IDS + 1];       /* usage counts on the map */
    int last_used[N_TILE_IDS + 1];  /* resolve clock at last access */
    int cur_page;                   /* page bound for rendering */
    int switches;                   /* page binding changes */
    int rebuilds;                   /* page content mutations after prefill */
    int initial_builds;             /* pages populated during prefill */
    unsigned prng;                  /* naive-policy victim RNG state */
    int clock;
    int max_pages_per_frame;        /* most pages touched in one frame */
} MultiAtlas;

static void ma_init(MultiAtlas* a, int npages, int policy) {
    memset(a, 0, sizeof(*a));
    a->npages = npages;
    a->policy = policy;
    a->cur_page = -1;
    a->prng = 0xabcdef01u;
    for (int p = 0; p < npages; p++) page_init(&a->pages[p], PAGE_COLS, PAGE_ROWS);
}

/* allocate ids to pages by usage frequency: hottest ids fill page 0 first */
static void ma_prefill(MultiAtlas* a, const int freq[N_TILE_IDS + 1]) {
    int order[N_TILE_IDS];
    int n = 0;
    for (int id = 1; id <= N_TILE_IDS; id++) {
        if (freq[id] > 0) { a->freq[id] = freq[id]; order[n++] = id; }
    }
    for (int i = 0; i < n; i++) {              /* insertion sort: freq desc, id asc */
        int key = order[i], j = i - 1;
        while (j >= 0 && (a->freq[order[j]] < a->freq[key] ||
                          (a->freq[order[j]] == a->freq[key] && order[j] > key))) {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = key;
    }
    int page = 0, slot = 0;
    for (int i = 0; i < n; i++) {
        if (page >= a->npages) break;         /* coldest ids start unresident */
        a->pages[page].slot_id[slot] = (unsigned char)order[i];
        a->last_used[order[i]] = 0;
        if (++slot == PAGE_SLOTS) { page++; slot = 0; }
    }
    a->initial_builds = page + (n > 0 && slot > 0 && page < a->npages ? 1 : 0);
}

/* resolve an id to (page, slot), loading or replacing it on a miss.
 * returns the page index; the slot is written through out_slot. */
static int ma_resolve(MultiAtlas* a, unsigned id, int* out_slot) {
    a->clock++;
    for (int p = 0; p < a->npages; p++) {
        int s = page_find(&a->pages[p], id);
        if (s >= 0) {                                            /* resident hit */
            if (a->cur_page >= 0 && a->cur_page != p) { a->switches++; a->cur_page = p; }
            else if (a->cur_page < 0) a->cur_page = p;          /* first binding */
            a->last_used[id] = a->clock;
            *out_slot = s;
            return p;
        }
    }

    int vp, vs;
    int free_p = -1, free_s = -1;                                /* prefer a free slot */
    for (int p = 0; p < a->npages && free_p < 0; p++) {
        int s = page_first_free(&a->pages[p]);
        if (s >= 0) { free_p = p; free_s = s; }
    }
    if (free_p >= 0) {
        vp = free_p; vs = free_s;
    } else if (a->policy == POLICY_NAIVE) {
        unsigned x = a->prng;                       /* random victim, cost-blind */
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;
        a->prng = x;
        int idx = (int)(x % (unsigned)(a->npages * PAGE_SLOTS));
        vp = idx / PAGE_SLOTS;
        vs = idx % PAGE_SLOTS;
    } else {
        /* cost = freq then LRU.  First pick the page that owns the globally
         * cheapest slot, then evict within that page: all churn concentrates
         * on the coldest page and the hot page stays resident. */
        vp = 0; vs = 0;
        for (int p = 0; p < a->npages; p++) {
            for (int s = 0; s < PAGE_SLOTS; s++) {
                unsigned cur = a->pages[p].slot_id[s];
                unsigned best = a->pages[vp].slot_id[vs];
                int worse = a->freq[cur] < a->freq[best] ||
                    (a->freq[cur] == a->freq[best] && a->last_used[cur] < a->last_used[best]);
                if (worse) { vp = p; vs = s; }
            }
        }
        for (int s = 0; s < PAGE_SLOTS; s++) {
            unsigned cur = a->pages[vp].slot_id[s];
            unsigned best = a->pages[vp].slot_id[vs];
            int worse = a->freq[cur] < a->freq[best] ||
                (a->freq[cur] == a->freq[best] && a->last_used[cur] < a->last_used[best]);
            if (worse) vs = s;
        }
    }

    a->rebuilds++;                                              /* this page is rebuilt */
    a->pages[vp].slot_id[vs] = (unsigned char)id;
    if (a->cur_page >= 0 && a->cur_page != vp) { a->switches++; a->cur_page = vp; }
    else if (a->cur_page < 0) a->cur_page = vp;
    a->last_used[id] = a->clock;
    *out_slot = vs;
    return vp;
}

static void render_rect_multi(const TileMap* m, int layer,
                              int x0, int y0, int x1, int y1,
                              MultiAtlas* a, unsigned char* fb, int* pages_touched) {
    int touched = 0;
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            unsigned id = map_get(m, layer, x, y);
            if (!id) continue;
            int slot;
            int p = ma_resolve(a, id, &slot);
            touched |= 1 << p;
            blit_tile(id, x, y, fb);
        }
    }
    {
        int cnt = 0;
        for (int p = 0; p < a->npages; p++) if (touched & (1 << p)) cnt++;
        if (cnt > a->max_pages_per_frame) a->max_pages_per_frame = cnt;
        if (pages_touched) *pages_touched = touched;
    }
}

/* deterministic scenario: skewed tile frequencies (hot pool + long tail) */
static unsigned rng_state;
static unsigned rng_next(void) {
    unsigned x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}

static void build_scenario(TileMap* m, int freq[N_TILE_IDS + 1]) {
    map_init(m);
    rng_state = 0x12345678u;
    for (int y = 0; y < ATL_MAP_H; y++) {
        for (int x = 0; x < ATL_MAP_W; x++) {
            unsigned id = (rng_next() % 100 < 80) ? 1 + rng_next() % 8
                                                  : 1 + rng_next() % N_TILE_IDS;
            map_set(m, 0, x, y, (unsigned char)id);
            if (rng_next() % 4 == 0) {                          /* sparse overlay */
                unsigned id2 = (rng_next() % 100 < 70) ? 1 + rng_next() % 8
                                                       : 1 + rng_next() % N_TILE_IDS;
                map_set(m, 1, x, y, (unsigned char)id2);
            }
        }
    }
    map_set(m, 0, ATL_MAP_W - 1, VIEW_Y1, 40);                 /* cold anchor in view path */
    map_set(m, 0, 0, 0, 39);
    memset(freq, 0, (N_TILE_IDS + 1) * sizeof(int));
    for (int l = 0; l < LAYERS; l++)
        for (int y = 0; y < ATL_MAP_H; y++)
            for (int x = 0; x < ATL_MAP_W; x++) {
                unsigned id = map_get(m, l, x, y);
                if (id) freq[id]++;
            }
}

/* render the same frame sequence: sliding layer-0 viewports, then a full
 * two-layer frame.  returns byte-identical framebuffers for any resolver. */
static void run_frames_single(const TileMap* m, const AtlasPage* ref,
                              unsigned char fbs[N_FRAMES][FB_W * FB_H]) {
    for (int f = 0; f < N_FRAMES; f++) {
        memset(fbs[f], 0, FB_W * FB_H);
        if (f < N_FRAMES - 1) {
            render_rect_single(m, 0, f, VIEW_Y0, f + VIEW_TW - 1, VIEW_Y1, ref, fbs[f]);
        } else {
            for (int l = 0; l < LAYERS; l++)
                if (m->layer_visible[l])
                    render_rect_single(m, l, 0, 0, ATL_MAP_W - 1, ATL_MAP_H - 1, ref, fbs[f]);
        }
    }
}

static void run_frames_multi(const TileMap* m, MultiAtlas* a,
                             unsigned char fbs[N_FRAMES][FB_W * FB_H]) {
    for (int f = 0; f < N_FRAMES; f++) {
        memset(fbs[f], 0, FB_W * FB_H);
        if (f < N_FRAMES - 1) {
            render_rect_multi(m, 0, f, VIEW_Y0, f + VIEW_TW - 1, VIEW_Y1, a, fbs[f], NULL);
        } else {
            for (int l = 0; l < LAYERS; l++)
                if (m->layer_visible[l])
                    render_rect_multi(m, l, 0, 0, ATL_MAP_W - 1, ATL_MAP_H - 1, a, fbs[f], NULL);
        }
    }
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

    /* ---- multi-page atlas ---- */
    static unsigned char ref_fb[N_FRAMES][FB_W * FB_H];
    static unsigned char mul_fb[N_FRAMES][FB_W * FB_H];
    int freq[N_TILE_IDS + 1];
    build_scenario(&m, freq);

    /* single-page reference atlas: 8x8 grid, every id resident at slot id-1 */
    AtlasPage ref;
    page_init(&ref, 8, 8);
    for (int id = 1; id <= N_TILE_IDS; id++) ref.slot_id[id - 1] = (unsigned char)id;
    run_frames_single(&m, &ref, ref_fb);

    struct { const char* name; int pages; int policy; } cfgs[] = {
        {"single16",     1, POLICY_COST },   /* one small page: wholesale rebuilds */
        {"multi2_cost",  2, POLICY_COST },
        {"multi3_cost",  3, POLICY_COST },
        {"multi4_cost",  4, POLICY_COST },
        {"multi2_rand",  2, POLICY_NAIVE},
    };
    int sw[5], rb[5], ib[5], mp[5];
    int pixel_ok = 1;
    for (int c = 0; c < 5; c++) {
        MultiAtlas a;
        ma_init(&a, cfgs[c].pages, cfgs[c].policy);
        ma_prefill(&a, freq);
        run_frames_multi(&m, &a, mul_fb);
        for (int f = 0; f < N_FRAMES; f++)
            if (memcmp(ref_fb[f], mul_fb[f], FB_W * FB_H) != 0) pixel_ok = 0;
        sw[c] = a.switches; rb[c] = a.rebuilds; ib[c] = a.initial_builds; mp[c] = a.max_pages_per_frame;
        printf("metrics cfg=%s pages=%d initial_builds=%d switches=%d rebuilds=%d max_pages_per_frame=%d\n",
               cfgs[c].name, cfgs[c].pages, ib[c], sw[c], rb[c], mp[c]);
    }

    /* determinism: a second cost-policy run must give identical counters */
    MultiAtlas d1, d2;
    ma_init(&d1, 2, POLICY_COST); ma_prefill(&d1, freq); run_frames_multi(&m, &d1, mul_fb);
    ma_init(&d2, 2, POLICY_COST); ma_prefill(&d2, freq); run_frames_multi(&m, &d2, mul_fb);
    check("deterministic_counters", d1.switches == d2.switches && d1.rebuilds == d2.rebuilds);

    /* frequency allocation: page 0 holds the hottest id; pages are filled in
     * non-increasing frequency order, and nothing overlaps two slots */
    MultiAtlas hot;
    ma_init(&hot, 3, POLICY_COST);
    ma_prefill(&hot, freq);
    int hottest = 1;
    for (int id = 2; id <= N_TILE_IDS; id++) if (freq[id] > freq[hottest]) hottest = id;
    check("prefill_hot_on_page0", page_find(&hot.pages[0], (unsigned)hottest) >= 0);
    int order_ok = 1, overlap = 0;
    int prev_min = 1 << 30;
    for (int p = 0; p < hot.npages; p++) {
        int pmin = 1 << 30, pmax = -1;
        for (int s = 0; s < PAGE_SLOTS; s++) {
            unsigned id = hot.pages[p].slot_id[s];
            if (!id) continue;
            if (freq[id] < pmin) pmin = freq[id];
            if (freq[id] > pmax) pmax = freq[id];
            for (int q = p; q < hot.npages; q++)
                for (int t = (q == p ? s + 1 : 0); t < PAGE_SLOTS; t++)
                    if (hot.pages[q].slot_id[t] == id) overlap++;
        }
        if (pmax >= 0) { if (pmin > prev_min) order_ok = 0; prev_min = pmin; }
    }
    check("prefill_freq_ordered", order_ok);
    check("prefill_no_duplicate", overlap == 0);
    check("prefill_cold_evicted", hot.initial_builds == 3);  /* 40 ids, 48 slots: 3 pages */

    /* cross-page references resolve: a frame must span pages yet match pixels */
    check("cross_page_touched", mp[1] >= 2);                 /* multi2_cost */
    check("pixel_identical_all", pixel_ok);

    /* cost-based replacement beats the naive cyclic policy on both counters */
    check("cost_beats_naive_switches", sw[1] <= sw[4]);
    check("cost_beats_naive_rebuilds", rb[1] <= rb[4]);

    /* the trade-off the table is meant to show: one page never switches but
     * rebuilds constantly; more pages switch sometimes but rebuild far less */
    check("single_never_switches", sw[0] == 0);
    check("multi_fewer_rebuilds", rb[1] < rb[0]);
    check("enough_pages_no_rebuilds", rb[3] == 0);           /* 4 pages = 64 > 40 ids */

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
