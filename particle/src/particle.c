
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget
   and emitter priority scheduling.

   Two storage layouts run in lockstep:
     - AoS (baseline): Particle pool[MAX]           -- kept as the reference
     - SoA:            one array per field
   Every frame both versions are compared field-by-field, particle-by-particle.
   The field list below is the single source of truth: the AoS struct, the SoA
   struct and the comparator are all generated from it, so adding/removing a
   field updates the checker automatically -- and forgetting to update one of
   the two update() implementations for that field is caught the same frame.
   (`./particle-pool desync-selftest` demonstrates that detection.)

   A set-associative cache model (32KB, 8-way, 64B lines, LRU) instruments the
   update loop of both layouts and reports simulated hit rates. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 8

/* ---- particle fields: single source of truth for both layouts + checker ---- */
#define PARTICLE_FIELDS(X) \
    X(float, x) \
    X(float, y) \
    X(float, vx) \
    X(float, vy) \
    X(float, life) \
    X(float, age) \
    X(int, alive) \
    X(int, tag)

/* ---- AoS baseline layout ---- */
#define X(T, name) T name;
typedef struct { PARTICLE_FIELDS(X) } Particle;
#undef X

/* ---- SoA layout: one array per field ---- */
#define X(T, name) T name[MAX_PARTICLES];
typedef struct { PARTICLE_FIELDS(X) } ParticleSoA;
#undef X

typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
} WorldAos;

typedef struct {
    ParticleSoA f;
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
} WorldSoA;

/* ---- cache simulator: set-associative LRU ---- */
typedef struct {
    int sets, assoc, line_shift;
    unsigned long* tags;   /* sets*assoc, tag+1 (0 = empty) */
    unsigned long* stamp;  /* sets*assoc, last-use tick */
    unsigned long tick;
    unsigned long hits, misses;
} CacheSim;

static void cache_init(CacheSim* c, int bytes, int assoc, int line_bytes) {
    int lines = bytes / line_bytes;
    c->sets = lines / assoc;
    c->assoc = assoc;
    c->line_shift = 6; /* 64B lines */
    c->tags = calloc((size_t)c->sets * assoc, sizeof(unsigned long));
    c->stamp = calloc((size_t)c->sets * assoc, sizeof(unsigned long));
    c->tick = c->hits = c->misses = 0;
}

static void cache_access(CacheSim* c, const void* p) {
    unsigned long line = (unsigned long)(uintptr_t)p >> c->line_shift;
    unsigned long set = line % (unsigned long)c->sets;
    unsigned long tag = line / (unsigned long)c->sets + 1;
    unsigned long* tags = &c->tags[set * (unsigned long)c->assoc];
    unsigned long* stamp = &c->stamp[set * (unsigned long)c->assoc];
    c->tick++;
    for (int i = 0; i < c->assoc; i++)
        if (tags[i] == tag) { c->hits++; stamp[i] = c->tick; return; }
    c->misses++;
    int victim = 0;
    for (int i = 0; i < c->assoc; i++)
        if (tags[i] == 0 || stamp[i] < stamp[victim]) victim = i;
    tags[victim] = tag;
    stamp[victim] = c->tick;
}

static void cache_free(CacheSim* c) { free(c->tags); free(c->stamp); }

/* record one read / one read+write of a field (NULL cs = no instrumentation) */
#define ACCR(cs, ptr)  do { if (cs) cache_access(cs, (ptr)); } while (0)
#define ACCRW(cs, ptr) do { if (cs) { cache_access(cs, (ptr)); cache_access(cs, (ptr)); } } while (0)

/* ================= AoS baseline (reference implementation) ================= */

static void aos_init(WorldAos* w) { memset(w, 0, sizeof(*w)); }

static int aos_emitter_add(WorldAos* w, int id, int quota, int priority) {
    if (w->emitter_count >= MAX_EMITTERS) return -1;
    Emitter* e = &w->emitters[w->emitter_count++];
    e->id = id; e->quota = quota; e->spawned = 0; e->priority = priority;
    return e->id;
}

static int aos_spawn_one(WorldAos* w, int emitter_id) {
    int idx = -1;
    for (int i = 0; i < MAX_PARTICLES; i++) if (!w->pool[i].alive) { idx = i; break; }
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
static void aos_spawn_frame(WorldAos* w, int budget) {
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
        for (int i = 0; i < want; i++) if (aos_spawn_one(w, e->id) >= 0) { e->spawned++; left--; }
    }
}

static void aos_update(WorldAos* w, float dt, CacheSim* cs) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        ACCR(cs, &p->alive);
        if (!p->alive) continue;
        ACCRW(cs, &p->age); p->age += dt;
        ACCRW(cs, &p->x); ACCR(cs, &p->vx); p->x += p->vx * dt;
        ACCRW(cs, &p->y); ACCR(cs, &p->vy); p->y += p->vy * dt;
        ACCR(cs, &p->life);
        if (p->age >= p->life) { ACCRW(cs, &p->alive); p->alive = 0; w->alive_count--; }
    }
}

/* ================= SoA layout ================= */

static void soa_init(WorldSoA* w) { memset(w, 0, sizeof(*w)); }

static int soa_emitter_add(WorldSoA* w, int id, int quota, int priority) {
    if (w->emitter_count >= MAX_EMITTERS) return -1;
    Emitter* e = &w->emitters[w->emitter_count++];
    e->id = id; e->quota = quota; e->spawned = 0; e->priority = priority;
    return e->id;
}

static int soa_spawn_one(WorldSoA* w, int emitter_id) {
    int idx = -1;
    for (int i = 0; i < MAX_PARTICLES; i++) if (!w->f.alive[i]) { idx = i; break; }
    if (idx < 0) return -1;
    w->f.alive[idx] = 1; w->f.age[idx] = 0.0f;
    w->f.x[idx] = 0.0f; w->f.y[idx] = 0.0f;
    w->f.vx[idx] = 1.0f; w->f.vy[idx] = 0.5f;
    w->f.life[idx] = 2.0f;
    w->f.tag[idx] = emitter_id;
    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    return idx;
}

static void soa_spawn_frame(WorldSoA* w, int budget) {
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
        for (int i = 0; i < want; i++) if (soa_spawn_one(w, e->id) >= 0) { e->spawned++; left--; }
    }
}

/* skip_field: test hook simulating "field added, update site forgotten" */
static void soa_update(WorldSoA* w, float dt, CacheSim* cs, const char* skip_field) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        ACCR(cs, &w->f.alive[i]);
        if (!w->f.alive[i]) continue;
        ACCRW(cs, &w->f.age[i]); w->f.age[i] += dt;
        ACCRW(cs, &w->f.x[i]); ACCR(cs, &w->f.vx[i]); w->f.x[i] += w->f.vx[i] * dt;
        if (!skip_field || strcmp(skip_field, "y") != 0) {
            ACCRW(cs, &w->f.y[i]); ACCR(cs, &w->f.vy[i]); w->f.y[i] += w->f.vy[i] * dt;
        }
        ACCR(cs, &w->f.life[i]);
        if (w->f.age[i] >= w->f.life[i]) { ACCRW(cs, &w->f.alive[i]); w->f.alive[i] = 0; w->alive_count--; }
    }
}

/* ================= lockstep field-by-field comparison ================= */

static int compare_worlds(const WorldAos* a, const WorldSoA* b, int frame) {
    int bad = 0;
    if (a->alive_count != b->alive_count) {
        printf("MISMATCH frame=%d alive_count aos=%d soa=%d\n", frame, a->alive_count, b->alive_count);
        bad++;
    }
    for (int i = 0; i < MAX_PARTICLES; i++) {
#define X(T, name) \
        if (a->pool[i].name != b->f.name[i]) { \
            if (bad < 8) printf("MISMATCH frame=%d slot=%d field=%s aos=%g soa=%g\n", \
                                frame, i, #name, (double)a->pool[i].name, (double)b->f.name[i]); \
            bad++; \
        }
        PARTICLE_FIELDS(X)
#undef X
    }
    return bad;
}

/* ================= main ================= */

int main(int argc, char** argv) {
    int selftest = argc > 1 && strcmp(argv[1], "desync-selftest") == 0;

    static WorldAos a; static WorldSoA b;
    aos_init(&a); soa_init(&b);
    aos_emitter_add(&a, 1, 40, 2); soa_emitter_add(&b, 1, 40, 2);
    aos_emitter_add(&a, 2, 30, 9); soa_emitter_add(&b, 2, 30, 9);
    aos_emitter_add(&a, 3, 20, 5); soa_emitter_add(&b, 3, 20, 5);

    CacheSim cs_aos, cs_soa;
    cache_init(&cs_aos, 32 * 1024, 8, 64);
    cache_init(&cs_soa, 32 * 1024, 8, 64);

    int mismatches = 0;
    for (int frame = 0; frame < 10; frame++) {
        aos_spawn_frame(&a, 50);
        soa_spawn_frame(&b, 50);
        aos_update(&a, 0.1f, &cs_aos);
        soa_update(&b, 0.1f, &cs_soa, selftest ? "y" : NULL);
        mismatches += compare_worlds(&a, &b, frame);
    }

    /* original baseline report (from the AoS reference) */
    printf("alive=%d high_water=%d\n", a.alive_count, a.high_water);
    int tag1 = 0, tag2 = 0, tag3 = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (a.pool[i].alive) {
        if (a.pool[i].tag == 1) tag1++; else if (a.pool[i].tag == 2) tag2++; else tag3++;
    }
    printf("by_emitter=%d,%d,%d\n", tag1, tag2, tag3);
    /* quota 优先：高优先级 2 应先拿满 30 */
    printf("priority_respected=%d\n", tag2 >= tag3 && tag3 >= 0);

    printf("lockstep: frames=10 field_mismatches=%d\n", mismatches);

    printf("cache_sim (update loop, 32KB 8-way 64B-line LRU):\n");
    printf("  aos: accesses=%lu hits=%lu misses=%lu hit_rate=%.1f%%\n",
           cs_aos.hits + cs_aos.misses, cs_aos.hits, cs_aos.misses,
           100.0 * (double)cs_aos.hits / (double)(cs_aos.hits + cs_aos.misses));
    printf("  soa: accesses=%lu hits=%lu misses=%lu hit_rate=%.1f%%\n",
           cs_soa.hits + cs_soa.misses, cs_soa.hits, cs_soa.misses,
           100.0 * (double)cs_soa.hits / (double)(cs_soa.hits + cs_soa.misses));

    cache_free(&cs_aos); cache_free(&cs_soa);

    if (selftest) {
        /* the desync MUST be caught; exit 0 only if the checker fired */
        if (mismatches > 0) { printf("desync-selftest: detected (ok)\n"); return 0; }
        printf("desync-selftest: NOT detected (checker broken)\n");
        return 1;
    }
    return mismatches ? 1 : 0;
}
