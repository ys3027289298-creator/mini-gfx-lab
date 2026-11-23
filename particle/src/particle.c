
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget
   and emitter priority scheduling. */
#include <stdio.h>
#include <string.h>
#include "diag.h"

#define FRAMES 10

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 8

typedef struct { float x, y, vx, vy, life, age; int alive; int tag; } Particle;
typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
} World;

#if DIAG
/* evidence material for the numeric table, captured during the run */
static int tbl_spawned[FRAMES], tbl_alive[FRAMES], tbl_hw[FRAMES];

static void emit_numeric_table(const Diag* d, int tag1, int tag2, int tag3, const World* w) {
    fprintf(stderr, "[particle] numeric evidence (per-frame):\n");
    fprintf(stderr, "  frame  spawned  alive  high_water\n");
    for (int i = 0; i < FRAMES; i++)
        fprintf(stderr, "  %5d  %7d  %5d  %10d\n", i, tbl_spawned[i], tbl_alive[i], tbl_hw[i]);
    fprintf(stderr, "[particle] final: alive=%d high_water=%d by_emitter=%d,%d,%d\n",
            w->alive_count, w->high_water, tag1, tag2, tag3);
    fprintf(stderr, "[particle] checks failed: %d/%d\n", d->failed, d->checks);
}
#endif

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

static void update(World* w, float dt) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        p->age += dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
        if (p->age >= p->life) { p->alive = 0; w->alive_count--; }
    }
}

int main(void) {
    Diag d; diag_begin(&d, "particle");
    World w; world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    for (int frame = 0; frame < FRAMES; frame++) {
        spawn_frame(&w, 50);
#if DIAG
        int sp = 0;
        for (int i = 0; i < w.emitter_count; i++) sp += w.emitters[i].spawned;
#endif
        update(&w, 0.1f);
#if DIAG
        tbl_spawned[frame] = sp; tbl_alive[frame] = w.alive_count; tbl_hw[frame] = w.high_water;
#endif
    }
    int tag1 = 0, tag2 = 0, tag3 = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (w.pool[i].alive) {
        if (w.pool[i].tag == 1) tag1++; else if (w.pool[i].tag == 2) tag2++; else tag3++;
    }
    /* acceptance: golden values from the reference run */
    diag_check(&d, w.alive_count == 500);
    diag_check(&d, w.high_water == 500);
    diag_check(&d, tag1 == 0 && tag2 == 300 && tag3 == 200);
    /* quota 优先：高优先级 2 应先拿满 30 */
    diag_check(&d, tag2 >= tag3);
    diag_check(&d, !diag_forced(&d));

    DIAG_DRAW("alive=%d high_water=%d\n", w.alive_count, w.high_water);
    DIAG_DRAW("by_emitter=%d,%d,%d\n", tag1, tag2, tag3);
    DIAG_DRAW("priority_respected=%d\n", tag2 >= tag3 && tag3 >= 0);
#if DIAG
    if (diag_ok(&d)) { diag_draw_flush(); return 0; }
    diag_drop_notice(&d);
    emit_numeric_table(&d, tag1, tag2, tag3, &w);
    return DIAG_EXIT_DROP;
#else
    return 0;
#endif
}
