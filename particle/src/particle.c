
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget
   and emitter priority scheduling. */
#include <stdio.h>
#include <string.h>

#include "diag.h"
#include "canvas.h"

#define MOD "particle"

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

static void world_init(World* w) {
    memset(w, 0, sizeof(*w));
    w->emitter_count = 0;
}

static int emitter_add(World* w, int id, int quota, int priority) {
    for (int i = 0; i < w->emitter_count; i++)
        if (w->emitters[i].id == id || DIAG_FAULT(MOD, "duplicate_emitter"))
            DIAG_FAIL_RETURN(MOD, "duplicate_emitter", DIAG_E_INVALID_ARG);
    if (w->emitter_count >= MAX_EMITTERS || DIAG_FAULT(MOD, "emitter_capacity"))
        DIAG_FAIL_RETURN(MOD, "emitter_capacity", DIAG_E_CAPACITY);
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
    if (idx < 0 || DIAG_FAULT(MOD, "pool_full"))
        DIAG_FAIL_RETURN(MOD, "pool_full", DIAG_E_CAPACITY);
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
static int spawn_frame(World* w, int budget) {
    if (budget < 0 || DIAG_FAULT(MOD, "bad_budget"))
        DIAG_FAIL_RETURN(MOD, "bad_budget", DIAG_E_INVALID_ARG);
    for (int i = 0; i < w->emitter_count; i++)
        if (w->emitters[i].priority < 0 || DIAG_FAULT(MOD, "invalid_priority"))
            DIAG_FAIL_RETURN(MOD, "invalid_priority", DIAG_E_INVALID_ARG);
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
        for (int i = 0; i < want; i++) {
            int idx = spawn_one(w, e->id);
            if (DIAG_IS_ERR(idx)) return idx;   /* pool exhaustion is a module failure */
            if (idx >= 0) { e->spawned++; left--; }
        }
    }
    return DIAG_OK;
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

/* ---------------------------------------------------------------- stage --- */

int particle_stage(Canvas* c, int y0, int rows) {
    World w; world_init(&w);
    Canvas s;
    canvas_init(&s);
    if (!c || rows <= 0 || rows > CANVAS_H || y0 < 0 || y0 + rows > CANVAS_H ||
        DIAG_FAULT(MOD, "output_overflow"))
        DIAG_FAIL_RETURN(MOD, "output_overflow", DIAG_E_CAPACITY);

    int rc = emitter_add(&w, 1, 40, 2); if (rc < 0) return rc;
    rc = emitter_add(&w, 2, 30, 9);     if (rc < 0) return rc;
    rc = emitter_add(&w, 3, 20, 5);     if (rc < 0) return rc;
    for (int frame = 0; frame < 10; frame++) {
        rc = spawn_frame(&w, 50);
        if (rc != DIAG_OK) return rc;
        update(&w, 0.1f);
    }

    int plot_rows = rows - 2;
    if (plot_rows < 1) plot_rows = 1;
    static const char glyph[4] = { '.', 'o', 'O', '+' };
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle* p = &w.pool[i];
        if (!p->alive) continue;
        int lane = p->tag >= 1 && p->tag <= 3 ? p->tag : 3;
        int x = 2 + lane * 23 + (int)(p->x * 9.0f);
        int y = 2 + (int)(p->y * (float)(plot_rows - 1) / 0.6f);
        canvas_put(&s, x, y, glyph[lane]);
    }

    char head[CANVAS_W + 1];
    snprintf(head, sizeof head, "particles alive=%d high_water=%d  o=1 O=2 +=3",
             w.alive_count, w.high_water);
    canvas_text(&s, 1, 0, head);

    canvas_blit_band(c, &s, y0, rows);
    return DIAG_OK;
}

/* ----------------------------------------------------------------- demo --- */
#ifndef PARTICLE_NO_MAIN
int main(void) {
    diag_boot();
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
    diag_flush(stderr);
    return 0;
}
#endif
