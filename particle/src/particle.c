
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget,
   emitter priority scheduling and a parented emitter hierarchy.

   Emitters carry a local rigid transform (translation + rotation) relative to
   their parent; particles live in their emitter's local frame and are mapped
   to world space on demand, so a moving/rotating parent carries every child
   emitter and its particles without per-frame manual coordinate edits. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 16
#define MAX_HIERARCHY_DEPTH 8   /* hard upper bound on parent-chain length */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 2D rigid transform: world = rotate(local, a) + t */
typedef struct { float x, y, angle; } Transform;

typedef struct { float x, y, vx, vy, life, age; int alive; int tag; } Particle;
typedef struct {
    int id, quota, spawned, priority;
    int parent;             /* emitter index, -1 = root */
    float lx, ly, langle;   /* local transform relative to parent */
    Transform world;        /* cached world transform */
} Emitter;

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

/* ---- rigid transform math ---------------------------------------------- */

static Transform tr_compose(const Transform* parent, const Transform* local) {
    Transform out;
    float c = cosf(parent->angle), s = sinf(parent->angle);
    out.x = parent->x + c * local->x - s * local->y;
    out.y = parent->y + s * local->x + c * local->y;
    out.angle = atan2f(sinf(parent->angle + local->angle),
                       cosf(parent->angle + local->angle));
    return out;
}

static Transform tr_inverse(const Transform* t) {
    Transform inv;
    float c = cosf(t->angle), s = sinf(t->angle);
    inv.x = -(c * t->x + s * t->y);
    inv.y = -(-s * t->x + c * t->y);
    inv.angle = -t->angle;
    return inv;
}

/* point: affine (rotation + translation); vector: rotation only */
static void tr_point(const Transform* t, float lx, float ly, float* wx, float* wy) {
    float c = cosf(t->angle), s = sinf(t->angle);
    *wx = t->x + c * lx - s * ly;
    *wy = t->y + s * lx + c * ly;
}

static void tr_vector(const Transform* t, float lx, float ly, float* wx, float* wy) {
    float c = cosf(t->angle), s = sinf(t->angle);
    *wx = c * lx - s * ly;
    *wy = s * lx + c * ly;
}

/* ---- emitter hierarchy ------------------------------------------------- */

static int emitter_add(World* w, int quota, int priority) {
    if (w->emitter_count >= MAX_EMITTERS) return -1;
    Emitter* e = &w->emitters[w->emitter_count];
    e->id = w->emitter_count;
    e->quota = quota; e->spawned = 0; e->priority = priority;
    e->parent = -1;
    e->lx = e->ly = e->langle = 0.0f;
    e->world = (Transform){ 0.0f, 0.0f, 0.0f };
    return w->emitter_count++;
}

/* parent < 0 detaches to root. Rejects unknown nodes, self-parenting,
   cycles and chains deeper than MAX_HIERARCHY_DEPTH. */
static int emitter_set_parent(World* w, int child, int parent) {
    if (child < 0 || child >= w->emitter_count) return -1;
    if (parent >= w->emitter_count) return -1;
    if (parent == child) return -1;
    if (parent >= 0) {
        int depth = 1, cur = parent;
        while (cur != -1) {
            if (cur == child) return -1;               /* cycle */
            if (depth >= MAX_HIERARCHY_DEPTH) return -1;
            cur = w->emitters[cur].parent;
            depth++;
        }
    }
    w->emitters[child].parent = parent;
    return 0;
}

static void emitter_set_local(World* w, int idx, float x, float y, float angle) {
    Emitter* e = &w->emitters[idx];
    e->lx = x; e->ly = y; e->langle = angle;
}

/* world transform = local transforms composed from the root down */
static Transform emitter_world_transform(const World* w, int idx) {
    Transform chain[MAX_HIERARCHY_DEPTH];
    int n = 0, cur = idx;
    while (cur != -1 && n < MAX_HIERARCHY_DEPTH) {
        const Emitter* e = &w->emitters[cur];
        chain[n++] = (Transform){ e->lx, e->ly, e->langle };
        cur = e->parent;
    }
    Transform acc = { 0.0f, 0.0f, 0.0f };
    for (int i = n - 1; i >= 0; i--) acc = tr_compose(&acc, &chain[i]);
    return acc;
}

static void emitter_update_world(World* w) {
    for (int i = 0; i < w->emitter_count; i++)
        w->emitters[i].world = emitter_world_transform(w, i);
}

/* local <-> world coordinate conversion, points and direction vectors */
static void emitter_local_to_world(const Emitter* e, float lx, float ly, float* wx, float* wy) {
    tr_point(&e->world, lx, ly, wx, wy);
}

static void emitter_world_to_local(const Emitter* e, float wx, float wy, float* lx, float* ly) {
    Transform inv = tr_inverse(&e->world);
    tr_point(&inv, wx, wy, lx, ly);
}

static void emitter_vec_local_to_world(const Emitter* e, float lx, float ly, float* wx, float* wy) {
    tr_vector(&e->world, lx, ly, wx, wy);
}

static void emitter_vec_world_to_local(const Emitter* e, float wx, float wy, float* lx, float* ly) {
    Transform inv = tr_inverse(&e->world);
    tr_vector(&inv, wx, wy, lx, ly);
}

/* ---- particle pool ----------------------------------------------------- */

static int pool_find_free(World* w) {
    for (int i = 0; i < MAX_PARTICLES; i++) if (!w->pool[i].alive) return i;
    return -1;
}

/* spawns in the emitter's local frame; local velocity rotates with it */
static int spawn_one(World* w, int emitter_idx, float lvx, float lvy, float life) {
    int idx = pool_find_free(w);
    if (idx < 0) return -1;
    Particle* p = &w->pool[idx];
    p->alive = 1; p->age = 0.0f;
    p->x = 0.0f; p->y = 0.0f;
    p->vx = lvx; p->vy = lvy;
    p->life = life;
    p->tag = emitter_idx;
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
        for (int i = 0; i < want; i++)
            if (spawn_one(w, e->id, 1.0f, 0.5f, 2.0f) >= 0) { e->spawned++; left--; }
    }
}

/* integration happens in local space; hierarchy motion is applied via transforms */
static void update(World* w, float dt) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        p->age += dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
        if (p->age >= p->life) { p->alive = 0; w->alive_count--; }
    }
}

static void particle_world_pos(const World* w, const Particle* p, float* wx, float* wy) {
    tr_point(&w->emitters[p->tag].world, p->x, p->y, wx, wy);
}

static void particle_world_vel(const World* w, const Particle* p, float* wx, float* wy) {
    tr_vector(&w->emitters[p->tag].world, p->vx, p->vy, wx, wy);
}

/* ---- test harness ------------------------------------------------------ */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* smallest absolute difference between two angles, wrapped to [-pi, pi] */
static float angle_diff(float a, float b) {
    float d = fmodf(a - b, 2.0f * M_PI);
    if (d > M_PI) d -= 2.0f * M_PI;
    if (d < -M_PI) d += 2.0f * M_PI;
    return d;
}

int main(void) {
    /* quota + priority scheduling still works */
    World w; world_init(&w);
    int e1 = emitter_add(&w, 40, 2);
    int e2 = emitter_add(&w, 30, 9);
    int e3 = emitter_add(&w, 20, 5);
    for (int frame = 0; frame < 10; frame++) {
        spawn_frame(&w, 50);
        update(&w, 0.1f);
    }
    int tag1 = 0, tag2 = 0, tag3 = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (w.pool[i].alive) {
        if (w.pool[i].tag == e1) tag1++;
        else if (w.pool[i].tag == e2) tag2++;
        else if (w.pool[i].tag == e3) tag3++;
    }
    check("priority_quota", w.emitters[e2].spawned == 30 && tag2 >= tag3);

    /* parent translation carries child emitter and its particles */
    World h; world_init(&h);
    int p = emitter_add(&h, 0, 0);
    int c = emitter_add(&h, 0, 0);
    check("parent_attach", emitter_set_parent(&h, c, p) == 0);
    emitter_set_local(&h, p, 5.0f, 3.0f, 0.0f);
    emitter_set_local(&h, c, 10.0f, 0.0f, 0.0f);
    emitter_update_world(&h);
    float wx, wy;
    emitter_local_to_world(&h.emitters[c], 0.0f, 0.0f, &wx, &wy);
    check("child_world_pos", fabsf(wx - 15.0f) < 1e-5f && fabsf(wy - 3.0f) < 1e-5f);

    int pi = spawn_one(&h, c, 1.0f, 0.0f, 10.0f);
    update(&h, 0.5f);
    particle_world_pos(&h, &h.pool[pi], &wx, &wy);
    check("particle_carried", fabsf(wx - 15.5f) < 1e-5f && fabsf(wy - 3.0f) < 1e-5f);

    /* nested rotation: angles accumulate, points and vectors rotate together */
    World r; world_init(&r);
    int rp = emitter_add(&r, 0, 0);
    int rc = emitter_add(&r, 0, 0);
    emitter_set_parent(&r, rc, rp);
    emitter_set_local(&r, rp, 1.0f, 2.0f, M_PI / 2.0f);
    emitter_set_local(&r, rc, 1.0f, 0.0f, M_PI / 2.0f);
    emitter_update_world(&r);
    check("nested_angle", fabsf(angle_diff(r.emitters[rc].world.angle, M_PI)) < 1e-4f);
    emitter_local_to_world(&r.emitters[rc], 0.0f, 0.0f, &wx, &wy);
    check("nested_point", fabsf(wx - 1.0f) < 1e-5f && fabsf(wy - 3.0f) < 1e-5f);
    float vx, vy;
    emitter_vec_local_to_world(&r.emitters[rc], 1.0f, 0.0f, &vx, &vy);
    check("vector_rotates", fabsf(vx + 1.0f) < 1e-5f && fabsf(vy) < 1e-5f);

    /* rotating the emitter rotates existing particles' velocity direction */
    World v; world_init(&v);
    int ve = emitter_add(&v, 0, 0);
    int vi = spawn_one(&v, ve, 1.0f, 0.0f, 10.0f);
    update(&v, 0.1f);
    emitter_set_local(&v, ve, 0.0f, 0.0f, M_PI / 2.0f);
    emitter_update_world(&v);
    particle_world_vel(&v, &v.pool[vi], &vx, &vy);
    check("vel_rotates_with_emitter", fabsf(vx) < 1e-5f && fabsf(vy - 1.0f) < 1e-5f);
    check("local_vel_unchanged", fabsf(v.pool[vi].vx - 1.0f) < 1e-6f && v.pool[vi].vy == 0.0f);
    update(&v, 0.1f);
    particle_world_pos(&v, &v.pool[vi], &wx, &wy);
    check("vel_rotate_motion", fabsf(wx) < 1e-5f && fabsf(wy - 0.2f) < 1e-5f);

    /* matches per-frame manual integration through the same transform math */
    World m; world_init(&m);
    int me = emitter_add(&m, 0, 0);
    int mi = spawn_one(&m, me, 1.0f, 0.0f, 10.0f);
    emitter_set_local(&m, me, 4.0f, -2.0f, M_PI / 4.0f);
    emitter_update_world(&m);
    update(&m, 0.25f);
    float man_x, man_y;
    tr_point(&m.emitters[me].world, m.pool[mi].x, m.pool[mi].y, &man_x, &man_y);
    particle_world_pos(&m, &m.pool[mi], &wx, &wy);
    check("matches_manual", fabsf(wx - man_x) < 1e-6f && fabsf(wy - man_y) < 1e-6f);

    /* local <-> world round trips */
    emitter_world_to_local(&r.emitters[rc], 1.0f, 3.0f, &wx, &wy);
    check("roundtrip_point", fabsf(wx) < 1e-5f && fabsf(wy) < 1e-5f);
    emitter_vec_world_to_local(&r.emitters[rc], -1.0f, 0.0f, &vx, &vy);
    check("roundtrip_vector", fabsf(vx - 1.0f) < 1e-5f && fabsf(vy) < 1e-5f);

    /* cycles and self-parenting are rejected, state untouched */
    World cy; world_init(&cy);
    int n0 = emitter_add(&cy, 0, 0);
    int n1 = emitter_add(&cy, 0, 0);
    int n2 = emitter_add(&cy, 0, 0);
    int n3 = emitter_add(&cy, 0, 0);
    int n4 = emitter_add(&cy, 0, 0);
    check("reject_self_parent", emitter_set_parent(&cy, n1, n1) == -1);
    check("chain_builds", emitter_set_parent(&cy, n1, n0) == 0
                       && emitter_set_parent(&cy, n2, n1) == 0
                       && emitter_set_parent(&cy, n3, n2) == 0
                       && emitter_set_parent(&cy, n4, n3) == 0);
    check("reject_cycle", emitter_set_parent(&cy, n1, n4) == -1);
    check("state_unchanged", cy.emitters[n1].parent == n0);

    /* hierarchy depth has a hard upper bound */
    World d; world_init(&d);
    int prev = emitter_add(&d, 0, 0);
    int depth_ok = 1;
    for (int i = 1; i < MAX_HIERARCHY_DEPTH; i++) {
        int node = emitter_add(&d, 0, 0);
        if (emitter_set_parent(&d, node, prev) != 0) depth_ok = 0;
        prev = node;
    }
    check("depth_bound_accepted", depth_ok && d.emitter_count == MAX_HIERARCHY_DEPTH);
    int extra = emitter_add(&d, 0, 0);
    check("depth_bound_rejects", emitter_set_parent(&d, extra, prev) == -1);
    check("detach_to_root", emitter_set_parent(&d, extra, -1) == 0 && d.emitters[extra].parent == -1);

    printf("alive=%d high_water=%d\n", w.alive_count, w.high_water);
    printf("by_emitter=%d,%d,%d\n", tag1, tag2, tag3);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
