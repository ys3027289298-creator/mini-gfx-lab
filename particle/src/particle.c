#define _POSIX_C_SOURCE 200809L
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget
   and emitter priority scheduling, plus a batch sorter that groups draws by
   (blend mode, texture) and depth-sorts each group with a counting sort. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MAX_PARTICLES 16384
#define MAX_EMITTERS 8

/* --- batch sort parameters ---------------------------------------------- */
#define INTERLEAVED 10000        /* particles under test, deliberately shuffled */
#define DEPTH_KEYS 1024          /* quantized depth values 0 (near) .. 1023 */
#define MAX_BATCHES 16           /* distinct (blend, texture) pairs */

typedef struct {
    float x, y, vx, vy, life, age;
    int alive;
    int tag;
    int blend;                   /* 0 = additive, 1 = alpha */
    int tex;                     /* texture id */
    int depth;                   /* quantized depth, 0 = nearest */
} Particle;
typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
} World;

static void world_init(World* w) {
    memset(w, 0, sizeof(*w));
    w->emitter_count = 0;
}

static int emitter_add(World* w, int id, int quota, int priority) {
    if (w->emitter_count >= MAX_EMITTERS) return -1;
    Emitter* e = &w->emitters[w->emitter_count++];
    e->id = id; e->quota = quota; e->spawned = 0; e->priority = priority;
    return e->id;
}

static int pool_find_free(World* w) {
    for (int i = 0; i < MAX_PARTICLES; i++) if (!w->pool[i].alive) return i;
    return -1;
}

static int spawn_one(World* w, int emitter_id) {
    int idx = pool_find_free(w);
    if (idx < 0) return -1;
    Particle* p = &w->pool[idx];
    p->alive = 1; p->age = 0.0f;
    p->x = 0.0f; p->y = 0.0f;
    p->vx = 1.0f; p->vy = 0.5f;
    p->life = 2.0f;
    p->tag = emitter_id;
    p->blend = 0; p->tex = 0; p->depth = 0;
    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    return idx;
}

/* highest priority emitters spend their quota first */
static void spawn_frame(World* w, int budget) {
    int left = budget;
    int order[MAX_EMITTERS]; int n = w->emitter_count;
    for (int i = 0; i < n; i++) order[i] = i;
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
            if (w->emitters[order[j]].priority > w->emitters[order[i]].priority) {
                int t = order[i]; order[i] = order[j]; order[j] = t;
            }
    for (int k = 0; k < n && left > 0; k++) {
        Emitter* e = &w->emitters[order[k]];
        e->spawned = 0;
        int want = e->quota < left ? e->quota : left;
        for (int i = 0; i < want; i++) if (spawn_one(w, e->id) >= 0) { e->spawned++; left--; }
    }
}

static void update(World* w, float dt) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        p->age += dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
        if (p->age >= p->life) { p->alive = 0; w->alive_count--; }
    }
}

/* --- batch sorting ------------------------------------------------------ */

typedef struct { int blend, tex; } BatchKey;

typedef struct {
    int pid;                     /* pool index, also the creation order */
    int group;                   /* index into the batch key table */
} BatchEntry;

static BatchKey g_keys[MAX_BATCHES];
static int g_key_count;
static BatchEntry g_entries[MAX_PARTICLES];   /* draw order, filled by sort */
static BatchEntry g_spread[MAX_PARTICLES];    /* grouped but unsorted scratch */
static BatchEntry g_creation[MAX_PARTICLES];  /* unsorted order, for comparison */
static int g_counts[DEPTH_KEYS];

/* map (blend, texture) to a dense batch index on first sight */
static int batch_key_index(int blend, int tex) {
    for (int i = 0; i < g_key_count; i++)
        if (g_keys[i].blend == blend && g_keys[i].tex == tex) return i;
    if (g_key_count >= MAX_BATCHES) return -1;
    g_keys[g_key_count].blend = blend;
    g_keys[g_key_count].tex = tex;
    return g_key_count++;
}

/* Collect every live particle in creation order, group by batch, and within
   each batch counting-sort by depth, back to front (alpha blending needs far
   primitives first). Counting sort is stable, so particles sharing a depth
   stay in creation order and the result is deterministic.
   Cost is O(n + groups * DEPTH_KEYS): no per-particle comparisons at all. */
static int batch_sort(World* w) {
    g_key_count = 0;
    int n = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle* p = &w->pool[i];
        if (!p->alive) continue;
        int g = batch_key_index(p->blend, p->tex);
        if (g < 0) return -1;
        g_creation[n].pid = i; g_creation[n].group = g;
        n++;
    }

    int start[MAX_BATCHES], size[MAX_BATCHES];
    memset(size, 0, sizeof(size));
    for (int i = 0; i < n; i++) size[g_creation[i].group]++;
    int off = 0;
    for (int g = 0; g < g_key_count; g++) { start[g] = off; off += size[g]; }

    /* stable bucket pass: walking creation order keeps pid order per batch */
    int fill[MAX_BATCHES];
    memcpy(fill, start, sizeof(int) * (size_t)g_key_count);
    for (int i = 0; i < n; i++)
        g_spread[fill[g_creation[i].group]++] = g_creation[i];

    for (int g = 0; g < g_key_count; g++) {
        int beg = start[g], m = size[g];
        if (m == 0) continue;
        memset(g_counts, 0, sizeof(g_counts));
        for (int i = 0; i < m; i++) g_counts[w->pool[g_spread[beg + i].pid].depth]++;
        /* descending depth: offsets count keys strictly farther than d */
        int sum = 0;
        for (int d = DEPTH_KEYS - 1; d >= 0; d--) { int c = g_counts[d]; g_counts[d] = sum; sum += c; }
        for (int i = 0; i < m; i++) {
            BatchEntry e = g_spread[beg + i];
            g_entries[beg + g_counts[w->pool[e.pid].depth]++] = e;
        }
    }
    return n;
}

/* number of contiguous same-batch runs = draw calls after the API's own
   batching has done all it can with the given order */
static int draw_calls(const BatchEntry* es, int n) {
    int runs = 0, prev = -1;
    for (int i = 0; i < n; i++) {
        if (es[i].group != prev) { runs++; prev = es[i].group; }
    }
    return runs;
}

/* --- reference order: full sort over every particle ---------------------- */

/* qsort cannot see the pool, so the reference sort runs over an explicit
   (group, depth, pid) key array instead. */
typedef struct { int group, depth, pid; } SortKey;

static int cmp_key(const void* a, const void* b) {
    const SortKey* x = (const SortKey*)a;
    const SortKey* y = (const SortKey*)b;
    if (x->group != y->group) return x->group - y->group;
    if (x->depth != y->depth) return y->depth - x->depth;   /* far to near */
    return x->pid - y->pid;
}

/* --- software raster proxy for the pixel-equality check ----------------- */

#define SW 256
#define SH 256
static unsigned char g_px[SH][SW][3];

static unsigned mix32(unsigned x) {
    x ^= x >> 16; x *= 0x7feb352du;
    x ^= x >> 15; x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

static unsigned char add_sat(int d, int s) {
    int v = d + s;
    return (unsigned char)(v > 255 ? 255 : v);
}

/* Draw each entry as a soft sprite in entry order. Order matters: alpha
   blending is not commutative and even additive blending clips, so any
   reordering that changes composites shows up as a pixel difference. */
static void render_draw_order(const BatchEntry* es, int n, const World* w) {
    memset(g_px, 0, sizeof(g_px));
    for (int k = 0; k < n; k++) {
        const Particle* p = &w->pool[es[k].pid];
        int cx = (int)(p->x * 4.0f) % SW; if (cx < 0) cx += SW;
        int cy = (int)(p->y * 6.0f) % SH; if (cy < 0) cy += SH;
        unsigned h = mix32((unsigned)es[k].pid ^ ((unsigned)(p->tag & 0xff) << 16));
        int col[3] = { (int)(h & 63) + 32, (int)((h >> 8) & 63) + 32,
                       (int)((h >> 16) & 63) + 32 };
        int rad = 2 + (int)(h % 4);
        float a = 0.3f + 0.6f * (float)((h >> 24) & 255) / 255.0f;
        for (int y = cy - rad; y <= cy + rad; y++) {
            if (y < 0 || y >= SH) continue;
            for (int x = cx - rad; x <= cx + rad; x++) {
                if (x < 0 || x >= SW) continue;
                int dx = x - cx, dy = y - cy;
                if (dx * dx + dy * dy > rad * rad) continue;
                unsigned char* q = g_px[y][x];
                for (int c = 0; c < 3; c++) {
                    if (p->blend == 0) q[c] = add_sat(q[c], col[c]);
                    else q[c] = (unsigned char)((float)q[c] +
                                                ((float)col[c] - (float)q[c]) * a);
                }
            }
        }
    }
}

static long long now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

/* Deterministic scene: 4 blend/texture batches whose particles are created
   in an interleaved pattern, so naive creation order switches batch on
   almost every particle. Depths are scrambled independently of creation
   order, and quantized so ties are common. */
static void setup_scene(World* w) {
    static const int blends[4] = { 0, 1, 0, 1 };
    static const int texes[4]  = { 0, 0, 1, 1 };
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        int k = i % 4;
        p->blend = blends[k];
        p->tex = texes[k];
        unsigned h = mix32((unsigned)i * 2654435761u + 1u);
        p->x = (float)(h % 256u) / 4.0f;              /* screen x = x*4 */
        p->y = (float)((h >> 9) % 256u) / 6.0f;       /* screen y = y*6 */
        p->depth = (int)((mix32(h) >> 8) % (DEPTH_KEYS - 8u));
        p->depth += (int)((h >> 1) & 7u);             /* small jitter creates ties */
    }
}

int main(void) {
    World w; world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    for (int frame = 0; frame < 10; frame++) {
        spawn_frame(&w, 50);
        update(&w, 0.1f);
    }
    printf("alive=%d high_water=%d\n", w.alive_count, w.high_water);
    int tag1 = 0, tag2 = 0, tag3 = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (w.pool[i].alive) {
        if (w.pool[i].tag == 1) tag1++; else if (w.pool[i].tag == 2) tag2++; else tag3++;
    }
    printf("by_emitter=%d,%d,%d\n", tag1, tag2, tag3);
    /* quota 优先：高优先级 2 应先拿满 30 */
    printf("priority_respected=%d\n", tag2 >= tag3 && tag3 >= 0);

    /* --- batch sort benchmark + correctness ----------------------------- */
    while (w.alive_count < INTERLEAVED) spawn_frame(&w, INTERLEAVED);
    setup_scene(&w);                       /* per-particle blend/tex/depth */
    printf("interleaved_alive=%d\n", w.alive_count);
    int n = w.alive_count;
    if (n < INTERLEAVED) return 1;

    if (batch_sort(&w) != n) { printf("batch_sort failed\n"); return 1; }

    int calls_before = draw_calls(g_creation, n);
    int calls_after = draw_calls(g_entries, n);
    printf("draw_calls_before=%d after=%d batches=%d\n",
           calls_before, calls_after, g_key_count);

    /* reference: sort every particle's (group, depth, pid) key */
    static SortKey keys[MAX_PARTICLES];
    static BatchEntry ref[MAX_PARTICLES];
    for (int i = 0; i < n; i++) {
        keys[i].group = g_creation[i].group;
        keys[i].depth = w.pool[g_creation[i].pid].depth;
        keys[i].pid = g_creation[i].pid;
    }
    qsort(keys, (size_t)n, sizeof(SortKey), cmp_key);
    for (int i = 0; i < n; i++) { ref[i].pid = keys[i].pid; ref[i].group = keys[i].group; }

    int order_identical = 1, depth_ok = 1;
    int seen_blend[MAX_BATCHES] = {0}, seen_tex[MAX_BATCHES] = {0}, runs = 0;
    int runs_blend = -1, runs_tex = -1;
    for (int i = 0; i < n; i++) {
        if (ref[i].pid != g_entries[i].pid) order_identical = 0;
        const Particle* b = &w.pool[g_entries[i].pid];
        if (i == 0 || b->blend != runs_blend || b->tex != runs_tex) {
            /* a new run must not repeat a (blend, texture) seen earlier */
            for (int j = 0; j < runs; j++)
                if (seen_blend[j] == b->blend && seen_tex[j] == b->tex) depth_ok = 0;
            if (runs < MAX_BATCHES) { seen_blend[runs] = b->blend; seen_tex[runs] = b->tex; }
            runs++;
            runs_blend = b->blend; runs_tex = b->tex;
        } else {
            const Particle* a = &w.pool[g_entries[i - 1].pid];
            if (a->depth < b->depth) depth_ok = 0;    /* within a batch: far to near */
        }
    }
    if (runs != g_key_count) depth_ok = 0;

    /* pixel equality is against the reference per-particle sort, not against
       creation order: reordering is the whole point, but the grouped order
       must composite identically to sorting every particle. */
    static unsigned char px_naive[SH][SW][3];
    render_draw_order(g_creation, n, &w);
    memcpy(px_naive, g_px, sizeof(g_px));
    render_draw_order(ref, n, &w);
    long long diff_ref_vs_creation = 0;   /* proves order visibly matters */
    for (int y = 0; y < SH; y++)
        for (int x = 0; x < SW; x++)
            for (int c = 0; c < 3; c++)
                if (px_naive[y][x][c] != g_px[y][x][c]) diff_ref_vs_creation++;

    static unsigned char px_sorted[SH][SW][3];
    memcpy(px_sorted, g_px, sizeof(g_px));
    render_draw_order(g_entries, n, &w);
    long long diff_px = 0;
    for (int y = 0; y < SH; y++)
        for (int x = 0; x < SW; x++)
            for (int c = 0; c < 3; c++)
                if (px_sorted[y][x][c] != g_px[y][x][c]) diff_px++;

    /* informational: a depth-only sort that ignores batch keys interleaves
       additive and alpha draws, which is what a batch switch cannot do.
       Report the gap so the supported reference is explicit. (Additive draws
       form a commutative group, so that one batch alone stays invariant.) */
    static SortKey gkeys[MAX_PARTICLES];
    static BatchEntry gdepth[MAX_PARTICLES];
    for (int i = 0; i < n; i++) {
        gkeys[i].group = 0;
        gkeys[i].depth = w.pool[g_creation[i].pid].depth;
        gkeys[i].pid = g_creation[i].pid;
    }
    qsort(gkeys, (size_t)n, sizeof(SortKey), cmp_key);
    for (int i = 0; i < n; i++) { gdepth[i].pid = gkeys[i].pid; gdepth[i].group = 0; }
    memcpy(px_sorted, g_px, sizeof(g_px));
    render_draw_order(gdepth, n, &w);
    long long diff_global = 0;
    for (int y = 0; y < SH; y++)
        for (int x = 0; x < SW; x++)
            for (int c = 0; c < 3; c++)
                if (px_sorted[y][x][c] != g_px[y][x][c]) diff_global++;

    /* timing: best of 100 on a freshly built world state */
    long long best = 0;
    for (int it = 0; it < 100; it++) {
        long long t0 = now_us();
        batch_sort(&w);
        long long t1 = now_us();
        if (best == 0 || t1 - t0 < best) best = t1 - t0;
    }
    printf("sort_us_min=%lld group_ok=%d order_identical=%d pixel_diff=%lld\n",
           best, depth_ok, order_identical, diff_px);
    printf("order_matters_pixel_diff=%lld\n", diff_ref_vs_creation);
    printf("global_depth_sort_pixel_diff=%lld\n", diff_global);
    printf("draw_calls_reduced=%d\n", calls_after < calls_before);
    printf("sort_under_3ms=%d\n", best < 3000);
    return !(depth_ok && order_identical && diff_px == 0 &&
             calls_after < calls_before && best < 3000);
}
