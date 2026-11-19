
/* oldbench.c — the pre-change pool policy (linear scan for a free slot),
   kept only as a baseline so frame-time jitter can be compared.
   Same workload and timing method as bench.c. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <string.h>
#include <time.h>

#ifndef MAX_PARTICLES
#define MAX_PARTICLES 10000
#endif
#define MAX_EMITTERS 8

typedef struct { float x, y, vx, vy, life, age; int alive; int tag; } Particle;
typedef struct { int id; int quota; int spawned; int priority; } Emitter;
typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
} World;

static void world_init(World* w) { memset(w, 0, sizeof(*w)); }
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
    p->x = 0.0f; p->y = 0.0f; p->vx = 1.0f; p->vy = 0.5f;
    p->life = 2.0f; p->tag = emitter_id;
    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    return idx;
}
static void spawn_frame(World* w, int budget) {
    int left = budget, n = w->emitter_count;
    int order[MAX_EMITTERS];
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

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static World world;
static double frame_ms[1024];

int main(void) {
    const int frames = 600;
    printf("OLD-policy baseline: capacity=%d (linear scan, spawn fails when full)\n",
           MAX_PARTICLES);
    world_init(&world);
    emitter_add(&world, 1, 2500, 9);
    emitter_add(&world, 2, 2000, 5);
    emitter_add(&world, 3, 1500, 2);
    double t_start = now_ms();
    for (int f = 0; f < frames; f++) {
        double t0 = now_ms();
        spawn_frame(&world, f == 0 ? MAX_PARTICLES : 2000);
        update(&world, 0.1f);
        frame_ms[f] = now_ms() - t0;
    }
    double run = now_ms() - t_start;
    double sorted[1024];
    for (int i = 0; i < frames; i++) sorted[i] = frame_ms[i];
    for (int i = 1; i < frames; i++) {
        double key = sorted[i]; int j = i - 1;
        while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
        sorted[j + 1] = key;
    }
    double avg = run / frames, sq = 0.0;
    for (int i = 0; i < frames; i++) sq += (frame_ms[i] - avg) * (frame_ms[i] - avg);
    double sd = sq / frames, g = sd > 0 ? sd : 1e-12;
    for (int it = 0; it < 24; it++) g = 0.5 * (g + sd / g);
    printf("max=%.4f p99=%.4f avg=%.4f stddev=%.4f alive=%d (spawns after full: dropped)\n",
           sorted[frames - 1], sorted[(frames * 99) / 100], avg, g, world.alive_count);
    return 0;
}
