
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget
   and emitter priority scheduling.

   Lock-free parallel update: the pool is split into contiguous slot
   ranges, one per thread. A thread writes only inside its own range
   (cross-range reads would be legal, nothing needs them); the only
   cross-thread traffic is a generation counter and a done counter, both
   plain atomics — no mutex anywhere. Per-particle randomness is hashed
   from (slot index, frame), never from shared state, so a run is
   bit-identical for any thread count. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdatomic.h>
#include <pthread.h>
#include <time.h>

#define MAX_PARTICLES 65536
#define MAX_EMITTERS 8
#define MAX_THREADS 8

typedef struct { float x, y, vx, vy, life, age; int alive; int tag; } Particle;
typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
    uint64_t frame; /* drives per-particle RNG; bumped once per update */
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

static uint32_t hash_u32(uint32_t x) {
    x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16;
    return x;
}

/* deterministic noise in [0,1): a pure function of (slot, frame, salt),
   so it needs no shared RNG state and is scheduling-independent */
static float particle_rand(int idx, uint64_t frame, uint32_t salt) {
    uint32_t key = (uint32_t)idx
                 ^ (uint32_t)frame * 0x9e3779b9u
                 ^ (uint32_t)(frame >> 32) * 0x85ebca6bu
                 ^ salt;
    return (float)(hash_u32(key) >> 8) * (1.0f / 16777216.0f);
}

/* integrates particles [begin,end); all writes stay inside the range,
   w->frame is read-only shared state */
static void update_range(const World* w, int begin, int end, float dt, int* killed_out) {
    Particle* pool = (Particle*)w->pool; /* caller guarantees disjoint ranges */
    uint64_t frame = w->frame;
    int killed = 0;
    for (int i = begin; i < end; i++) {
        Particle* p = &pool[i];
        if (!p->alive) continue;
        float jx = particle_rand(i, frame, 0x68bc21ebu) - 0.5f;
        float jy = particle_rand(i, frame, 0x02e5be93u) - 0.5f;
        p->age += dt;
        p->x += (p->vx + jx * 0.4f) * dt;
        p->y += (p->vy + jy * 0.4f) * dt;
        if (p->age >= p->life) { p->alive = 0; killed++; }
    }
    *killed_out = killed;
}

static void update(World* w, float dt) {
    int killed = 0;
    update_range(w, 0, MAX_PARTICLES, dt, &killed);
    w->alive_count -= killed; /* integer reduction: order-independent */
    w->frame++;
}

/* ---- lock-free worker pool -------------------------------------------- */

typedef struct { int v; char pad[64 - sizeof(int)]; } PaddedInt;
typedef struct { atomic_uint v; char pad[64 - sizeof(atomic_uint)]; } PaddedAtomic;

typedef struct ParPool ParPool;
struct ParPool {
    World* w;
    float dt;
    int nthreads;                       /* slot 0 runs on the calling thread */
    int begin[MAX_THREADS], end[MAX_THREADS];
    PaddedInt killed[MAX_THREADS];      /* per-slot, no false sharing */
    PaddedAtomic gen;                   /* main -> workers: new frame ready */
    PaddedAtomic done;                  /* workers -> main: slot finished */
    atomic_int stop;
    pthread_t tid[MAX_THREADS];
    struct { ParPool* pp; int slot; } arg[MAX_THREADS];
};

static void cpu_relax(void) {
#if defined(__x86_64__) || defined(__i386__)
    __asm__ volatile("pause" ::: "memory");
#else
    __asm__ volatile("" ::: "memory");
#endif
}

static void* worker_main(void* p) {
    ParPool* pp = ((struct { ParPool* pp; int slot; }*)p)->pp;
    int slot = ((struct { ParPool* pp; int slot; }*)p)->slot;
    unsigned seen = 0;
    for (;;) {
        if (atomic_load_explicit(&pp->stop, memory_order_acquire)) return NULL;
        unsigned g = atomic_load_explicit(&pp->gen.v, memory_order_acquire);
        if (g == seen) { cpu_relax(); continue; }
        seen = g; /* acquire pairs with the release bump: w/dt/ranges visible */
        update_range(pp->w, pp->begin[slot], pp->end[slot], pp->dt, &pp->killed[slot].v);
        atomic_fetch_add_explicit(&pp->done.v, 1u, memory_order_release);
    }
}

static int pool_init(ParPool* pp, int nthreads) {
    if (nthreads < 2) nthreads = 2;
    if (nthreads > MAX_THREADS) nthreads = MAX_THREADS;
    memset(pp, 0, sizeof(*pp));
    pp->nthreads = nthreads;
    atomic_store(&pp->gen.v, 0);
    atomic_store(&pp->done.v, 0);
    atomic_store(&pp->stop, 0);
    for (int s = 1; s < nthreads; s++) {
        pp->arg[s].pp = pp; pp->arg[s].slot = s;
        if (pthread_create(&pp->tid[s], NULL, worker_main, &pp->arg[s]) != 0) return -1;
    }
    return 0;
}

static void pool_destroy(ParPool* pp) {
    atomic_store_explicit(&pp->stop, 1, memory_order_release);
    for (int s = 1; s < pp->nthreads; s++) pthread_join(pp->tid[s], NULL);
}

static void update_parallel(ParPool* pp, World* w, float dt) {
    pp->w = w; pp->dt = dt;
    int n = pp->nthreads, base = MAX_PARTICLES / n, rem = MAX_PARTICLES % n, at = 0;
    for (int s = 0; s < n; s++) {           /* contiguous slot ranges */
        int len = base + (s < rem ? 1 : 0);
        pp->begin[s] = at; pp->end[s] = at + len; at += len;
    }
    atomic_store_explicit(&pp->done.v, 0, memory_order_relaxed);
    atomic_fetch_add_explicit(&pp->gen.v, 1u, memory_order_release); /* publish */
    update_range(w, pp->begin[0], pp->end[0], dt, &pp->killed[0].v); /* slot 0 */
    unsigned need = (unsigned)(n - 1);
    while (atomic_load_explicit(&pp->done.v, memory_order_acquire) != need) cpu_relax();
    int killed = 0;
    for (int s = 0; s < n; s++) killed += pp->killed[s].v;
    w->alive_count -= killed;
    w->frame++;
}

/* ---- verification + benchmark ----------------------------------------- */

static double now_ns(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e9 + (double)ts.tv_nsec;
}

/* Particle is 6 floats + 2 ints = 32 bytes, no padding; raw bytes are
   a faithful snapshot of the full simulation state */
static uint64_t world_hash(const World* w) {
    uint64_t h = 1469598103934665603ULL;
    const uint64_t* p = (const uint64_t*)w->pool;
    for (size_t i = 0; i < sizeof(w->pool) / (sizeof(uint64_t)); i++) {
        h ^= p[i]; h *= 1099511628211ULL;
    }
    h ^= (uint64_t)w->alive_count; h *= 1099511628211ULL;
    return h;
}

/* runs the same deterministic scenario; returns a hash folded over every
   frame's full state. If frame_ns is non-NULL it must hold `frames`
   doubles and receives the time blocked inside update for each frame.
   Budget 400/frame keeps ~8000 particles alive at equilibrium, so update
   is compute-bound rather than a memory-bandwidth-bound scan */
static uint64_t sim_run(int nthreads, int frames, double* frame_ns) {
    World w; world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    ParPool pp; ParPool* ppp = NULL;
    if (nthreads > 1) { pool_init(&pp, nthreads); ppp = &pp; }
    uint64_t h = 1469598103934665603ULL;
    for (int f = 0; f < frames; f++) {
        spawn_frame(&w, 400);
        double t0 = now_ns();
        if (ppp) update_parallel(ppp, &w, 0.1f); else update(&w, 0.1f);
        if (frame_ns) frame_ns[f] = now_ns() - t0;
        h ^= world_hash(&w); h *= 1099511628211ULL;
    }
    if (ppp) pool_destroy(&pp);
    return h;
}

static int cmp_double(const void* a, const void* b) {
    double d = *(const double*)a - *(const double*)b;
    return (d > 0) - (d < 0);
}

static double median(double* v, int n) {
    qsort(v, (size_t)n, sizeof(*v), cmp_double);
    return v[n / 2];
}

int main(void) {
    /* original demo, single-threaded */
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

    /* determinism: every thread count must reproduce the 1-thread run
       bit-for-bit, per particle per frame */
    uint64_t ref = sim_run(1, 120, NULL);
    int all = 1;
    int counts[] = { 1, 2, 3, 4, 8 };
    for (unsigned i = 0; i < sizeof(counts) / sizeof(counts[0]); i++) {
        int m = sim_run(counts[i], 120, NULL) == ref;
        all &= m;
        printf("threads=%d match=%d\n", counts[i], m);
    }
    printf("all_match=%d\n", all);

    /* benchmark: per-frame time blocked in update, 1 thread vs 4 threads;
       medians over 600 frames so scheduler jitter can't flip the result */
    enum { BENCH_FRAMES = 600 };
    double* f1 = malloc(BENCH_FRAMES * sizeof(double));
    double* f4 = malloc(BENCH_FRAMES * sizeof(double));
    uint64_t h1 = sim_run(1, BENCH_FRAMES, f1);
    uint64_t h4 = sim_run(4, BENCH_FRAMES, f4);
    double m1 = median(f1, BENCH_FRAMES), m4 = median(f4, BENCH_FRAMES);
    printf("bench_match=%d\n", h1 == h4);
    printf("single_us=%.1f parallel4_us=%.1f\n", m1 / 1e3, m4 / 1e3);
    printf("speedup_ok=%d\n", m4 <= 0.7 * m1);
    free(f1); free(f4);
    return 0;
}
