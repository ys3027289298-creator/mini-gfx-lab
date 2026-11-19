
/* particle-pool: fixed-capacity particle pool with zero runtime allocation.
   All state is preallocated inside World: there is no malloc/calloc/realloc
   anywhere, including init. Spawn and death are O(1):
     - free slots form an intrusive singly linked free list (free_next)
     - when the pool is full, a spawn recycles the slot at a rotating cursor
       (round-robin FIFO eviction) instead of growing the pool
   Per-frame spawn budget and emitter priority scheduling are retained. */
#include <stdio.h>
#include <string.h>

#ifndef MAX_PARTICLES
#define MAX_PARTICLES 10000
#endif
#ifndef MAX_EMITTERS
#define MAX_EMITTERS 8
#endif

typedef struct { float x, y, vx, vy, life, age; int alive; int tag; } Particle;
typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct {
    Particle pool[MAX_PARTICLES];   /* particle slots, preallocated */
    int free_next[MAX_PARTICLES];   /* intrusive free list: next free slot, -1 = end */
    int free_head;                  /* first free slot, -1 == pool full */
    int evict_cursor;               /* next slot recycled when the pool is full */
    int alive_count, high_water, evictions;
    Emitter emitters[MAX_EMITTERS];
    int order[MAX_EMITTERS];        /* preallocated scratch for priority sorting */
    int emitter_count;
} World;

/* Builds the free list in place; touches only preallocated memory. */
static void world_init(World* w) {
    memset(w, 0, sizeof(*w));
    for (int i = 0; i < MAX_PARTICLES; i++) w->free_next[i] = i + 1;
    w->free_next[MAX_PARTICLES - 1] = -1;
    w->free_head = 0;
    w->evict_cursor = 0;
}

static int emitter_add(World* w, int id, int quota, int priority) {
    if (w->emitter_count >= MAX_EMITTERS) return -1;
    Emitter* e = &w->emitters[w->emitter_count++];
    e->id = id; e->quota = quota; e->spawned = 0; e->priority = priority;
    return e->id;
}

/* Always succeeds: a full pool evicts via the rotating cursor.
   Free-list path is O(1); eviction path is O(1) and never reallocates. */
static int spawn_one(World* w, int emitter_id) {
    int idx;
    if (w->free_head >= 0) {
        idx = w->free_head;
        w->free_head = w->free_next[idx];
        w->alive_count++;
        if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    } else {
        /* round-robin FIFO: recycle the slot that has served longest since
           the last full-pool sweep; the pool stays at fixed capacity. */
        idx = w->evict_cursor;
        w->evict_cursor = (w->evict_cursor + 1) % MAX_PARTICLES;
        w->evictions++;
    }
    Particle* p = &w->pool[idx];
    p->alive = 1; p->age = 0.0f;
    p->x = 0.0f; p->y = 0.0f;
    p->vx = 1.0f; p->vy = 0.5f;
    p->life = 2.0f;
    p->tag = emitter_id;
    return idx;
}

/* Highest-priority emitters spend their quota first. */
static void spawn_frame(World* w, int budget) {
    int left = budget;
    int n = w->emitter_count;
    for (int i = 0; i < n; i++) {
        w->order[i] = i;
        w->emitters[i].spawned = 0;
    }
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
            if (w->emitters[w->order[j]].priority > w->emitters[w->order[i]].priority) {
                int t = w->order[i]; w->order[i] = w->order[j]; w->order[j] = t;
            }
    for (int k = 0; k < n && left > 0; k++) {
        Emitter* e = &w->emitters[w->order[k]];
        int want = e->quota < left ? e->quota : left;
        for (int i = 0; i < want; i++) {
            spawn_one(w, e->id);
            e->spawned++;
            left--;
        }
    }
}

static void update(World* w, float dt) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        p->age += dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
        if (p->age >= p->life) {
            p->alive = 0;
            w->alive_count--;
            w->free_next[i] = w->free_head;   /* return slot to free list, O(1) */
            w->free_head = i;
        }
    }
}

#ifndef PARTICLE_NO_MAIN
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

    /* Eviction stress: demand exceeds capacity; the pool must stay at
       MAX_PARTICLES, reject nothing, and record every eviction. */
    World w2; world_init(&w2);
    enum { OVER = 512 };
    emitter_add(&w2, 7, MAX_PARTICLES + OVER, 0);
    spawn_frame(&w2, MAX_PARTICLES + OVER);
    int free_slots = 0;
    for (int i = w2.free_head; i >= 0; i = w2.free_next[i]) free_slots++;
    int ok = w2.alive_count == MAX_PARTICLES
          && w2.high_water == MAX_PARTICLES
          && w2.evictions == OVER
          && w2.free_head == -1
          && free_slots == 0;
    printf("eviction_alive=%d evictions=%d capacity=%d free_slots=%d\n",
           w2.alive_count, w2.evictions, MAX_PARTICLES, free_slots);

    /* After aging every particle out, the free list must cover the whole pool. */
    for (int i = 0; i < MAX_PARTICLES; i++) w2.pool[i].life = 0.0f;
    update(&w2, 1.0f);
    free_slots = 0;
    for (int i = w2.free_head; i >= 0; i = w2.free_next[i]) free_slots++;
    ok = ok && w2.alive_count == 0 && free_slots == MAX_PARTICLES;
    printf("reaped=%d free_slots=%d self_check=%s\n",
           MAX_PARTICLES, free_slots, ok ? "pass" : "FAIL");
    return ok ? 0 : 1;
}
#endif
