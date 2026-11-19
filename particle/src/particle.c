
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget,
   emitter priority scheduling and batched collision events.

   Collision-event guarantees
   - an event carries a stable particle id and the impact position; ids are
     assigned at spawn and never reused.
   - a particle reports its first contact once (`hit` latches), so the number
     of events equals the number of collisions instead of one per frame.
   - contacts are collected while update() integrates, then the whole batch is
     handed to subscribers once per frame as a read-only array.
   - delivery copies nothing and allocates nothing: every subscriber receives
     a pointer to the same fixed batch stored in World.
   - callbacks must not mutate the pool; particle_spawn/spawn_frame/update are
     rejected during dispatch and counted in dispatch_rejections.
   - a callback returning non-zero is a callback failure: it is counted and
     the remaining subscribers and later frames still run.
   - a host that unwinds a callback non-locally (longjmp / C++ exception) must
     call world_end_dispatch() after catching it.
   - contacts are point-in-shape tests on post-integration positions; fast
     particles can tunnel through thin colliders. */
#include <stdio.h>
#include <string.h>
#include <setjmp.h>   /* test only: proves recovery after a non-local unwind */

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 8
#define MAX_COLLIDERS 8
#define MAX_SUBS 4

/* A particle emits at most one event per frame, so a batch sized to the pool
   can never overflow and no event is ever dropped. */
#define MAX_EVENTS MAX_PARTICLES

typedef struct {
    float x, y, vx, vy, life, age;
    int alive;
    int tag;
    int hit;
    unsigned id;
} Particle;

typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef enum { COLLIDER_BOX = 0, COLLIDER_CIRCLE = 1 } ColliderKind;

typedef struct {
    int id;
    ColliderKind kind;
    float x, y;
    float hw, hh;    /* box half extents */
    float r;         /* circle radius */
} Collider;

typedef struct {
    unsigned particle_id;   /* stable id assigned at spawn */
    int slot;               /* pool slot, valid for the emitting frame only */
    float x, y;             /* impact position */
    int collider_id;
} ParticleEvent;

/* Return 0 to keep receiving events, non-zero to signal a callback failure. */
typedef int (*ParticleEventListener)(const ParticleEvent* events, int count, void* ctx);

typedef struct {
    ParticleEventListener cb;
    void* ctx;
    int id;
    int active;
} Subscriber;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    unsigned next_id;

    Emitter emitters[MAX_EMITTERS];
    int emitter_count;

    Collider colliders[MAX_COLLIDERS];
    int collider_count;
    int next_collider_id;

    /* batch produced by the last update() */
    ParticleEvent events[MAX_EVENTS];
    int event_count;
    unsigned events_total;

    Subscriber subs[MAX_SUBS];
    int next_sub_id;
    int in_delivery;
    int dispatch_rejections;
    int callback_errors;
} World;

static void world_init(World* w) {
    memset(w, 0, sizeof(*w));
    w->next_id = 1;
    w->next_collider_id = 1;
    w->next_sub_id = 1;
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

/* pool mutation; callers are responsible for the dispatch guard */
static int pool_alloc(World* w, int tag, float x, float y,
                      float vx, float vy, float life) {
    if (!(life > 0.0f)) return -1;
    int idx = pool_find_free(w);
    if (idx < 0) return -1;
    Particle* p = &w->pool[idx];
    memset(p, 0, sizeof(*p));
    p->alive = 1;
    p->x = x; p->y = y;
    p->vx = vx; p->vy = vy;
    p->life = life;
    p->tag = tag;
    if (w->next_id == 0) w->next_id = 1;   /* keep 0 = invalid */
    p->id = w->next_id++;
    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    return idx;
}

/* returns the new particle id, 0 on failure */
static unsigned particle_spawn(World* w, int tag, float x, float y,
                               float vx, float vy, float life) {
    if (w->in_delivery) { w->dispatch_rejections++; return 0; }
    int idx = pool_alloc(w, tag, x, y, vx, vy, life);
    return idx >= 0 ? w->pool[idx].id : 0;
}

/* returns the pool slot, -1 when full */
static int spawn_one(World* w, int emitter_id) {
    return pool_alloc(w, emitter_id, 0.0f, 0.0f, 1.0f, 0.5f, 2.0f);
}

/* highest priority emitters spend their quota first; returns spawned count */
static int spawn_frame(World* w, int budget) {
    if (w->in_delivery) { w->dispatch_rejections++; return -1; }
    int left = budget, total = 0;
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
        for (int i = 0; i < want; i++)
            if (spawn_one(w, e->id) >= 0) { e->spawned++; left--; total++; }
    }
    return total;
}

static int collider_add_box(World* w, float x, float y, float hw, float hh) {
    if (w->collider_count >= MAX_COLLIDERS) return -1;
    if (!(hw > 0.0f) || !(hh > 0.0f)) return -1;
    Collider* c = &w->colliders[w->collider_count++];
    c->kind = COLLIDER_BOX;
    c->x = x; c->y = y; c->hw = hw; c->hh = hh; c->r = 0.0f;
    c->id = w->next_collider_id++;
    return c->id;
}

static int collider_add_circle(World* w, float x, float y, float r) {
    if (w->collider_count >= MAX_COLLIDERS) return -1;
    if (!(r > 0.0f)) return -1;
    Collider* c = &w->colliders[w->collider_count++];
    c->kind = COLLIDER_CIRCLE;
    c->x = x; c->y = y; c->hw = 0.0f; c->hh = 0.0f; c->r = r;
    c->id = w->next_collider_id++;
    return c->id;
}

static int collider_contains(const Collider* c, float x, float y) {
    if (c->kind == COLLIDER_BOX)
        return x >= c->x - c->hw && x <= c->x + c->hw &&
               y >= c->y - c->hh && y <= c->y + c->hh;
    float dx = x - c->x, dy = y - c->y;
    return dx * dx + dy * dy <= c->r * c->r;
}

/* event_count <= number of live particles <= MAX_EVENTS: no overflow check
   is needed and no event can be dropped. */
static void emit_collision(World* w, int slot, const Collider* c) {
    Particle* p = &w->pool[slot];
    ParticleEvent* e = &w->events[w->event_count++];
    e->particle_id = p->id;
    e->slot = slot;
    e->x = p->x;
    e->y = p->y;
    e->collider_id = c->id;
}

static int world_subscribe(World* w, ParticleEventListener cb, void* ctx) {
    if (!cb) return -1;
    int slot = -1;
    for (int i = 0; i < MAX_SUBS; i++) if (!w->subs[i].active) { slot = i; break; }
    if (slot < 0) return -1;
    Subscriber* s = &w->subs[slot];
    s->cb = cb;
    s->ctx = ctx;
    s->active = 1;
    s->id = w->next_sub_id++;
    if (w->next_sub_id <= 0) w->next_sub_id = 1;
    return s->id;
}

static int world_unsubscribe(World* w, int sub_id) {
    for (int i = 0; i < MAX_SUBS; i++) {
        if (w->subs[i].active && w->subs[i].id == sub_id) {
            w->subs[i].active = 0;
            w->subs[i].cb = NULL;
            w->subs[i].ctx = NULL;
            return 0;
        }
    }
    return -1;
}

/* Called by hosts that caught a non-local unwinding out of a callback.
   Without it the next update() would look re-entrant and be rejected. */
static void world_end_dispatch(World* w) {
    w->in_delivery = 0;
}

/* Snapshot the subscriber list on the stack, then hand every subscriber the
   same batch pointer. Adding/removing subscribers from a callback only takes
   effect for the next frame. */
static void deliver_events(World* w) {
    ParticleEventListener cbs[MAX_SUBS];
    void* ctxs[MAX_SUBS];
    int n = 0;
    for (int i = 0; i < MAX_SUBS; i++) {
        if (w->subs[i].active) {
            cbs[n] = w->subs[i].cb;
            ctxs[n] = w->subs[i].ctx;
            n++;
        }
    }
    w->in_delivery = 1;
    for (int i = 0; i < n; i++) {
        if (cbs[i](w->events, w->event_count, ctxs[i]) != 0)
            w->callback_errors++;
    }
    w->in_delivery = 0;
}

/* integrate, collect contacts for this frame, then deliver the batch once.
   returns the number of events produced, -1 when called from a callback. */
static int update(World* w, float dt) {
    if (w->in_delivery) { w->dispatch_rejections++; return -1; }

    w->event_count = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        p->age += dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
        /* contacts are tested before the lifetime cull so an impact on a
           particle's final frame is still reported */
        if (!p->hit) {
            for (int k = 0; k < w->collider_count; k++) {
                if (collider_contains(&w->colliders[k], p->x, p->y)) {
                    p->hit = 1;
                    emit_collision(w, i, &w->colliders[k]);
                    break;   /* first contact only */
                }
            }
        }
        if (p->age >= p->life) { p->alive = 0; w->alive_count--; }
    }

    int produced = w->event_count;
    w->events_total += (unsigned)produced;
    if (produced > 0) deliver_events(w);
    return produced;
}

/* ------------------------------------------------------------------ */
/* tests                                                              */

#define CAP_MAX 64

typedef struct {
    unsigned ids[CAP_MAX];
    float xs[CAP_MAX], ys[CAP_MAX];
    int cids[CAP_MAX];
    int n;
    int calls;
    int max_count;
    int batch_diff;
    const ParticleEvent* batch_ptr;
} Capture;

typedef struct {
    World* w;
    int blocked;
    int alive_before, alive_after;
} Naughty;

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int capture_cb(const ParticleEvent* ev, int count, void* ctx) {
    Capture* c = (Capture*)ctx;
    c->calls++;
    if (c->calls == 1) c->batch_ptr = ev;
    else if (ev != c->batch_ptr) c->batch_diff = 1;
    if (count > c->max_count) c->max_count = count;
    for (int i = 0; i < count && c->n < CAP_MAX; i++) {
        c->ids[c->n] = ev[i].particle_id;
        c->xs[c->n] = ev[i].x;
        c->ys[c->n] = ev[i].y;
        c->cids[c->n] = ev[i].collider_id;
        c->n++;
    }
    return 0;
}

static int fail_cb(const ParticleEvent* ev, int count, void* ctx) {
    (void)ev; (void)count; (void)ctx;
    return -1;
}

static int naughty_cb(const ParticleEvent* ev, int count, void* ctx) {
    (void)ev; (void)count;
    Naughty* n = (Naughty*)ctx;
    int before = n->w->alive_count;
    unsigned id = particle_spawn(n->w, 99, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f);
    int spawned = spawn_frame(n->w, 10);
    int rc = update(n->w, 0.1f);
    int after = n->w->alive_count;
    n->blocked = (id == 0 && spawned < 0 && rc < 0 && before == after);
    n->alive_before = before;
    n->alive_after = after;
    return 0;
}

static int has_duplicate_ids(const Capture* c) {
    for (int i = 0; i < c->n; i++)
        for (int j = i + 1; j < c->n; j++)
            if (c->ids[i] == c->ids[j]) return 1;
    return 0;
}

static int contains_id(const unsigned* ids, int n, unsigned id) {
    for (int i = 0; i < n; i++) if (ids[i] == id) return 1;
    return 0;
}

/* longjmp out of a callback needs storage with static duration so the
   objects the callback touched are not indeterminate after the jump. */
static World g_throw_world;
static Capture g_throw_cap;
static jmp_buf g_throw_jmp;
static int throw_cb(const ParticleEvent* ev, int count, void* ctx) {
    (void)ev; (void)count; (void)ctx;
    longjmp(g_throw_jmp, 1);
    return 0;   /* not reached */
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

    /* ---- collision events: one per particle, batched per frame ---- */
    World c; world_init(&c);
    int box = collider_add_box(&c, 4.5f, 0.0f, 0.5f, 1.0f);
    int circle = collider_add_circle(&c, 50.0f, 50.0f, 1.0f);
    check("collider_ids", box == 1 && circle == 2);
    check("collider_reject_degenerate", collider_add_box(&c, 0.0f, 0.0f, 0.0f, 1.0f) == -1);

    Capture cap_a, cap_b;
    memset(&cap_a, 0, sizeof(cap_a));
    memset(&cap_b, 0, sizeof(cap_b));
    int sub_a = world_subscribe(&c, capture_cb, &cap_a);
    int sub_b = world_subscribe(&c, capture_cb, &cap_b);
    check("subscribe_ids", sub_a > 0 && sub_b > 0 && sub_a != sub_b);
    check("subscribe_reject_null", world_subscribe(&c, NULL, NULL) == -1);

    const float sx[8] = { 3.0f, 3.2f, 3.4f, 3.5f, 3.5f, 3.5f, 3.6f, 3.8f };
    unsigned sid[8];
    int spawned_ok = 1;
    for (int i = 0; i < 8; i++) {
        sid[i] = particle_spawn(&c, 42, sx[i], 0.0f, 1.0f, 0.0f, 5.0f);
        if (!sid[i]) spawned_ok = 0;
    }
    unsigned miss_a = particle_spawn(&c, 42, 0.0f, 3.0f, 1.0f, 0.0f, 5.0f);
    unsigned miss_b = particle_spawn(&c, 42, 0.0f, 0.0f, -1.0f, 0.0f, 5.0f);
    check("spawn_api", spawned_ok && miss_a && miss_b);

    int ev_frames = 0, ev_sum = 0;
    for (int f = 0; f < 15; f++) {
        int n = update(&c, 0.1f);
        if (n > 0) { ev_frames++; ev_sum += n; }
    }
    check("event_count_matches_collisions",
          c.events_total == 8 && ev_sum == 8 && c.event_count == 0);
    check("event_frames", ev_frames == 6);
    check("batch_max_count", cap_a.max_count == 3);
    check("batch_delivered_each", cap_a.n == 8 && cap_b.n == 8);
    check("batch_shared_pointer",
          cap_a.batch_ptr == cap_b.batch_ptr && cap_a.batch_ptr == &c.events[0]);
    check("batch_no_per_subscriber_copy", cap_a.batch_diff == 0 && cap_b.batch_diff == 0);
    check("event_ids_unique", !has_duplicate_ids(&cap_a));
    int ids_known = 1, colliders_known = 1, pos_ok = 1;
    for (int i = 0; i < cap_a.n; i++) {
        if (!contains_id(sid, 8, cap_a.ids[i])) ids_known = 0;
        if (cap_a.cids[i] != box) colliders_known = 0;
        if (!(cap_a.xs[i] >= 4.0f - 1e-3f && cap_a.xs[i] <= 5.0f + 1e-3f &&
              cap_a.ys[i] >= -1.0f && cap_a.ys[i] <= 1.0f)) pos_ok = 0;
    }
    check("event_ids_match_spawn", ids_known);
    check("event_collider_ids", colliders_known);
    check("event_positions", pos_ok);

    for (int f = 0; f < 10; f++) update(&c, 0.1f);
    check("no_repeat_events", c.events_total == 8 && cap_a.n == 8);

    check("unsubscribe_ok", world_unsubscribe(&c, sub_b) == 0);
    check("unsubscribe_twice", world_unsubscribe(&c, sub_b) == -1);
    check("unsubscribe_bad_id", world_unsubscribe(&c, 999) == -1);

    unsigned late = particle_spawn(&c, 42, 3.9f, 0.0f, 1.0f, 0.0f, 5.0f);
    for (int f = 0; f < 5; f++) update(&c, 0.1f);
    check("unsubscribe_effect", late != 0 && cap_a.n == 9 && cap_b.n == 8);
    check("events_total_after_late", c.events_total == 9);

    /* ---- callbacks may not touch the pool ---- */
    World g; world_init(&g);
    collider_add_box(&g, 0.6f, 0.0f, 0.2f, 0.5f);
    Naughty ng = { &g, 0, 0, 0 };
    world_subscribe(&g, naughty_cb, &ng);
    particle_spawn(&g, 1, 0.0f, 0.0f, 1.0f, 0.0f, 5.0f);
    for (int f = 0; f < 8 && g.events_total == 0; f++) update(&g, 0.1f);
    check("callback_mutation_blocked", ng.blocked == 1);
    check("callback_mutation_counted", g.dispatch_rejections == 3);
    check("callback_pool_unchanged", ng.alive_before == ng.alive_after);

    /* ---- a failing callback does not stop the frame or later frames ---- */
    World ex; world_init(&ex);
    collider_add_box(&ex, 0.6f, 0.0f, 0.2f, 0.5f);
    Capture cap_e;
    memset(&cap_e, 0, sizeof(cap_e));
    world_subscribe(&ex, fail_cb, NULL);
    world_subscribe(&ex, capture_cb, &cap_e);
    particle_spawn(&ex, 1, 0.0f, 0.0f, 1.0f, 0.0f, 5.0f);
    for (int f = 0; f < 8; f++) update(&ex, 0.1f);
    check("error_callback_counted", ex.callback_errors == 1);
    check("error_callback_isolated", cap_e.n == 1);
    particle_spawn(&ex, 1, 0.0f, 0.0f, 1.0f, 0.0f, 5.0f);
    for (int f = 0; f < 8; f++) update(&ex, 0.1f);
    check("error_next_frame_ok", cap_e.n == 2 && ex.callback_errors == 2);

    /* ---- non-local unwind: host recovers, next frames keep working ---- */
    world_init(&g_throw_world);
    memset(&g_throw_cap, 0, sizeof(g_throw_cap));
    collider_add_box(&g_throw_world, 0.6f, 0.0f, 0.2f, 0.5f);
    world_subscribe(&g_throw_world, capture_cb, &g_throw_cap);
    int throw_sub = world_subscribe(&g_throw_world, throw_cb, NULL);
    particle_spawn(&g_throw_world, 1, 0.0f, 0.0f, 1.0f, 0.0f, 5.0f);
    volatile int caught = 0;
    for (volatile int f = 0; f < 8; f++) {
        if (setjmp(g_throw_jmp) == 0) {
            update(&g_throw_world, 0.1f);
        } else {
            world_end_dispatch(&g_throw_world);
            caught = 1;
        }
    }
    check("exception_caught", caught == 1 && g_throw_cap.n == 1);
    world_unsubscribe(&g_throw_world, throw_sub);
    unsigned after_throw = particle_spawn(&g_throw_world, 1, 0.0f, 0.0f, 1.0f, 0.0f, 5.0f);
    for (int f = 0; f < 8; f++) update(&g_throw_world, 0.1f);
    check("exception_next_frame_ok", after_throw != 0 && g_throw_cap.n == 2);

    /* ---- ids stay unique across slot reuse ---- */
    World t; world_init(&t);
    unsigned id_a = particle_spawn(&t, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.05f);
    update(&t, 0.1f);
    unsigned id_b = particle_spawn(&t, 1, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    check("ids_not_reused", id_a > 0 && id_b > id_a);
    check("spawn_reject_life", particle_spawn(&t, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f) == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
