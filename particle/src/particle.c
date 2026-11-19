
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget,
   emitter priority scheduling, and frustum visibility culling.

   Culling model
   -------------
   Every particle owns a bounding circle (x, y, radius).  The view is a 2-D
   frustum made of four half-planes (near, far, left, right).  Particles are
   grouped into fixed-size batches; a batch is rejected when the circle that
   encloses every member circle lies completely outside one plane.  Only the
   members of surviving batches are tested one by one.  Because the batch
   circle contains the particle circles, a rejected batch cannot hide a
   visible particle, so the resulting visible set is exactly the per-particle
   test's result.

   Culled particles are frozen: update_visible() integrates only the visible
   list, so age and position are untouched while the particle is outside the
   frustum.  When it re-enters, integration resumes from the frozen state,
   which is the same trajectory a per-particle culler produces.
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 8
#define BATCH_SIZE 32

#define TEST_PARTICLES 960
#define TEST_FRAMES 300
#define TEST_DT 0.05f

typedef struct {
    float x, y, vx, vy, life, age, radius;
    int alive, tag;
    unsigned char visible;       /* inside the frustum on the last cull */
    unsigned char frozen;        /* culled: integration is suspended */
    unsigned frozen_frames;      /* consecutive frames spent culled */
} Particle;

typedef struct { int id; int quota; int spawned; int priority; } Emitter;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;

    int slot_high;               /* one past the highest slot ever occupied */
    int visible_idx[MAX_PARTICLES];
    int visible_count;
    int hidden_count;            /* particles culled this frame */
    int resumed_total;           /* frozen -> visible transitions */
    unsigned rng;
} World;

typedef struct { float cx, cy, yaw, half_angle, near_d, far_d; } Camera;

/* inside: nx*x + ny*y + d >= 0 */
typedef struct { float nx, ny, d; } Plane;

typedef struct { Plane p[4]; } Frustum;

typedef struct {
    int bound_probes;            /* particle visits while building bounds */
    int exact_tests;             /* particles tested individually */
    int plane_tests;             /* frustum-plane evaluations */
    int rejected_batches;
} CullStats;

static void world_init(World* w) {
    memset(w, 0, sizeof(*w));
    w->emitter_count = 0;
    w->rng = 0x2545F491u;
    w->slot_high = 0;
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

static void note_slot(World* w, int idx) {
    if (idx + 1 > w->slot_high) w->slot_high = idx + 1;
}

static int spawn_one(World* w, int emitter_id) {
    int idx = pool_find_free(w);
    if (idx < 0) return -1;
    Particle* p = &w->pool[idx];
    p->alive = 1; p->age = 0.0f;
    p->x = 0.0f; p->y = 0.0f;
    p->vx = 1.0f; p->vy = 0.5f;
    p->life = 2.0f;
    p->radius = 0.1f;
    p->tag = emitter_id;
    p->visible = 0; p->frozen = 0; p->frozen_frames = 0;
    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    note_slot(w, idx);
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

/* Legacy path used by the spawn/scheduling scenario, which has no camera:
   advances every live particle.  Camera-driven frames use cull_batch() +
   update_visible() below so culled particles stay frozen. */
static void update_all(World* w, float dt) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        p->age += dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
        if (p->age >= p->life) { p->alive = 0; w->alive_count--; }
    }
}

/* ---------------- frustum ---------------- */

static Frustum frustum_build(const Camera* c) {
    float cf = cosf(c->yaw), sf = sinf(c->yaw);
    float fx = cf, fy = sf;              /* forward */
    float rx = -sf, ry = cf;             /* right */
    float t = tanf(c->half_angle);
    Frustum fr;

    /* near: dot(f, p - cam) - near >= 0 */
    fr.p[0].nx = fx; fr.p[0].ny = fy;
    fr.p[0].d  = -(fx * c->cx + fy * c->cy) - c->near_d;

    /* far: far - dot(f, p - cam) >= 0 */
    fr.p[1].nx = -fx; fr.p[1].ny = -fy;
    fr.p[1].d  =  (fx * c->cx + fy * c->cy) + c->far_d;

    /* right side: u <= t*v  <=>  dot(t*f - r, p - cam) >= 0 */
    {
        float nx = t * fx - rx, ny = t * fy - ry;
        float inv = 1.0f / sqrtf(nx * nx + ny * ny);
        fr.p[2].nx = nx * inv; fr.p[2].ny = ny * inv;
        fr.p[2].d  = -(fr.p[2].nx * c->cx + fr.p[2].ny * c->cy);
    }
    /* left side: u >= -t*v  <=>  dot(t*f + r, p - cam) >= 0 */
    {
        float nx = t * fx + rx, ny = t * fy + ry;
        float inv = 1.0f / sqrtf(nx * nx + ny * ny);
        fr.p[3].nx = nx * inv; fr.p[3].ny = ny * inv;
        fr.p[3].d  = -(fr.p[3].nx * c->cx + fr.p[3].ny * c->cy);
    }
    return fr;
}

static int circle_in_frustum(float x, float y, float r, const Frustum* fr) {
    for (int i = 0; i < 4; i++) {
        float dist = fr->p[i].nx * x + fr->p[i].ny * y + fr->p[i].d;
        if (dist < -r) return 0;
    }
    return 1;
}

/* ---------------- visibility culling ---------------- */

static void mark_visible(World* w, int idx) {
    Particle* p = &w->pool[idx];
    if (p->frozen) {
        p->frozen = 0;
        p->frozen_frames = 0;
        w->resumed_total++;
    }
    p->visible = 1;
    w->visible_idx[w->visible_count++] = idx;
}

static void mark_culled(World* w, int idx) {
    Particle* p = &w->pool[idx];
    if (!p->frozen) { p->frozen = 1; p->frozen_frames = 0; }
    p->visible = 0;
    p->frozen_frames++;
    w->hidden_count++;
}

/* Batched cull: reject whole batches with a conservative bounding circle,
   then test survivors exactly.  Work is linear in the number of particles. */
static void cull_batch(World* w, const Frustum* fr, CullStats* st) {
    CullStats sink;
    if (!st) st = &sink;
    memset(st, 0, sizeof(*st));
    w->visible_count = 0;
    w->hidden_count = 0;

    int span = w->slot_high;
    for (int lo = 0; lo < span; lo += BATCH_SIZE) {
        int hi = lo + BATCH_SIZE;
        if (hi > span) hi = span;

        float minx = 0.0f, miny = 0.0f, maxx = 0.0f, maxy = 0.0f;
        int any = 0;
        for (int i = lo; i < hi; i++) {
            const Particle* p = &w->pool[i];
            if (!p->alive) continue;
            st->bound_probes++;
            if (!any) {
                minx = maxx = p->x; miny = maxy = p->y; any = 1;
            } else {
                if (p->x < minx) minx = p->x;
                if (p->x > maxx) maxx = p->x;
                if (p->y < miny) miny = p->y;
                if (p->y > maxy) maxy = p->y;
            }
        }
        if (!any) continue;

        float cx = (minx + maxx) * 0.5f;
        float cy = (miny + maxy) * 0.5f;
        float rb = 0.0f;
        for (int i = lo; i < hi; i++) {
            const Particle* p = &w->pool[i];
            if (!p->alive) continue;
            st->bound_probes++;
            float d = hypotf(p->x - cx, p->y - cy) + p->radius;
            if (d > rb) rb = d;
        }
        /* Must contain every member circle.  The margin absorbs rounding and
           can only make the batch test more permissive (never hide a visible
           particle); the exact tests below keep the result unchanged. */
        rb = rb * 1.0001f + 1e-4f;

        st->plane_tests += 4;
        if (!circle_in_frustum(cx, cy, rb, fr)) {
            st->rejected_batches++;
            for (int i = lo; i < hi; i++) if (w->pool[i].alive) mark_culled(w, i);
            continue;
        }
        for (int i = lo; i < hi; i++) {
            Particle* p = &w->pool[i];
            if (!p->alive) continue;
            st->exact_tests++;
            st->plane_tests += 4;
            if (circle_in_frustum(p->x, p->y, p->radius, fr)) mark_visible(w, i);
            else mark_culled(w, i);
        }
    }
}

/* Ground truth: every particle is tested individually. */
static void cull_reference(World* w, const Frustum* fr, CullStats* st) {
    CullStats sink;
    if (!st) st = &sink;
    memset(st, 0, sizeof(*st));
    w->visible_count = 0;
    w->hidden_count = 0;

    for (int i = 0; i < w->slot_high; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        st->exact_tests++;
        st->plane_tests += 4;
        if (circle_in_frustum(p->x, p->y, p->radius, fr)) mark_visible(w, i);
        else mark_culled(w, i);
    }
}

/* Only visible particles advance; culled particles stay frozen. */
static void update_visible(World* w, float dt) {
    for (int k = 0; k < w->visible_count; k++) {
        Particle* p = &w->pool[w->visible_idx[k]];
        p->age += dt;
        p->x += p->vx * dt;
        p->y += p->vy * dt;
        if (p->age >= p->life) {
            p->alive = 0;
            p->visible = 0;
            p->frozen = 0;
            w->alive_count--;
        }
    }
}

/* ---------------- deterministic random helpers ---------------- */

static unsigned rng_next(unsigned* s) {
    unsigned x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return x;
}

static float rng_unit(unsigned* s) {
    return (float)(rng_next(s) >> 8) * (1.0f / 16777216.0f);
}

/* Spawns one particle in a cluster keyed by its slot, so a batch stays
   spatially coherent and batch rejection is actually exercised. */
static int spawn_clustered(World* w, int emitter_id) {
    int idx = pool_find_free(w);
    if (idx < 0) return -1;
    Particle* p = &w->pool[idx];
    int cluster = idx / BATCH_SIZE;
    float ca = (float)cluster * 2.39996323f;
    float cr = 1.0f + 0.7f * (float)(cluster % 9);

    p->x = cosf(ca) * cr + (rng_unit(&w->rng) - 0.5f) * 0.4f;
    p->y = sinf(ca) * cr + (rng_unit(&w->rng) - 0.5f) * 0.4f;
    float va = ca + 1.57079633f + (rng_unit(&w->rng) - 0.5f) * 0.3f;
    /* slow tangential drift: particles cross the rotating frustum repeatedly
       instead of racing past the far plane and never returning */
    float spd = 0.10f + 0.25f * (float)((cluster * 37) % 11) / 10.0f;
    spd += (rng_unit(&w->rng) - 0.5f) * 0.04f;
    p->vx = cosf(va) * spd;
    p->vy = sinf(va) * spd;
    p->life = 1e6f; p->age = 0.0f;
    p->radius = 0.05f + rng_unit(&w->rng) * 0.1f;
    p->alive = 1; p->tag = emitter_id;
    p->visible = 0; p->frozen = 0; p->frozen_frames = 0;

    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    note_slot(w, idx);
    return idx;
}

/* ---------------- tests ---------------- */

/* Batch culling and per-particle culling must agree every frame, and the two
   simulations (including freeze/resume) must stay bit-identical. */
static int test_cull_lockstep(void) {
    World a, b;
    world_init(&a); world_init(&b);
    a.rng = 0xA341316Cu; b.rng = a.rng;

    for (int i = 0; i < TEST_PARTICLES; i++) {
        int sa = spawn_clustered(&a, 1 + (i % 3));
        int sb = spawn_clustered(&b, 1 + (i % 3));
        if (sa != sb) { printf("lockstep: spawn diverged\n"); return 0; }
    }

    int vis_mismatch = 0, state_mismatch = 0, frozen_drift = 0;
    int frames_hidden = 0, frames_rejected = 0, resumed = 0;
    long exact_total = 0, rejected_total = 0, probes_total = 0;
    static float sx[MAX_PARTICLES], sy[MAX_PARTICLES], sage[MAX_PARTICLES];

    for (int frame = 0; frame < TEST_FRAMES; frame++) {
        float t = (float)frame * TEST_DT;
        Camera cam;
        cam.cx = 0.4f * cosf(0.37f * t);
        cam.cy = 0.3f * sinf(0.29f * t);
        cam.yaw = 0.9f * t;          /* sweeps over frozen clusters repeatedly */
        cam.half_angle = 0.5f;
        cam.near_d = 0.15f;
        cam.far_d = 10.0f;
        Frustum fr = frustum_build(&cam);

        CullStats sa, sb;
        int before = a.resumed_total;
        cull_batch(&a, &fr, &sa);
        resumed += a.resumed_total - before;
        cull_reference(&b, &fr, &sb);

        for (int i = 0; i < a.slot_high; i++) {
            if (a.pool[i].visible != b.pool[i].visible) vis_mismatch++;
        }
        if (a.visible_count != b.visible_count) vis_mismatch++;

        if (a.hidden_count > 0) frames_hidden++;
        if (sa.rejected_batches > 0) frames_rejected++;
        exact_total += sa.exact_tests;
        rejected_total += sa.rejected_batches;
        probes_total += sa.bound_probes;

        for (int i = 0; i < a.slot_high; i++) {
            sx[i] = a.pool[i].x; sy[i] = a.pool[i].y; sage[i] = a.pool[i].age;
        }
        update_visible(&a, TEST_DT);
        update_visible(&b, TEST_DT);


        for (int i = 0; i < a.slot_high; i++) {
            const Particle* pa = &a.pool[i];
            const Particle* pb = &b.pool[i];
            if (pa->alive && !pa->visible) {
                if (pa->x != sx[i] || pa->y != sy[i] || pa->age != sage[i]) frozen_drift++;
            }
            if (pa->alive != pb->alive || pa->x != pb->x ||
                pa->y != pb->y || pa->age != pb->age) state_mismatch++;
        }
    }

    int ok = vis_mismatch == 0 && state_mismatch == 0 && frozen_drift == 0 &&
             resumed > 0 && frames_hidden > 0 && frames_rejected > 0;
    printf("cull_lockstep particles=%d frames=%d visible=%d resumed=%d\n",
           TEST_PARTICLES, TEST_FRAMES, a.visible_count, resumed);
    printf("cull_lockstep batches_rejected=%ld exact_tests=%ld bound_probes=%ld\n",
           rejected_total, exact_total, probes_total);
    printf("cull_lockstep hidden_frames=%d rejected_frames=%d vis_mismatch=%d "
           "frozen_drift=%d state_mismatch=%d\n",
           frames_hidden, frames_rejected, vis_mismatch, frozen_drift, state_mismatch);
    printf("cull_lockstep_ok=%d\n", ok);
    return ok;
}

/* Randomized per-particle equivalence, including batches fully outside.
   Fields are scattered so only a few batches are wholly off-screen each
   case; the random camera orientation then puts many contiguous batches
   outside the four planes. */
static int test_cull_fuzz(void) {
    enum { N = 256, CASES = 500 };
    World a, b;
    world_init(&a); world_init(&b);
    a.rng = 0xDEADBEEFu;

    int mismatches = 0;
    long rejected = 0, exact = 0;
    for (int it = 0; it < CASES; it++) {
        for (int i = 0; i < N; i++) {
            Particle p;
            memset(&p, 0, sizeof(p));
            p.alive = (rng_next(&a.rng) % 8) != 0;
            p.x = ((float)(i % 16) - 7.5f) * 3.0f + rng_unit(&a.rng) * 6.0f;
            p.y = ((float)(i / 16) - 7.5f) * 3.0f + rng_unit(&a.rng) * 6.0f;
            p.radius = rng_unit(&a.rng) * 1.5f;
            a.pool[i] = p;
            b.pool[i] = p;
        }
        a.slot_high = b.slot_high = N;

        Camera cam;
        cam.cx = (rng_unit(&a.rng) - 0.5f) * 10.0f;
        cam.cy = (rng_unit(&a.rng) - 0.5f) * 10.0f;
        cam.yaw = rng_unit(&a.rng) * 6.2831853f;
        cam.half_angle = 0.10f + rng_unit(&a.rng) * 0.7f;
        cam.near_d = 0.0f;
        cam.far_d = 1.0f + rng_unit(&a.rng) * 12.0f;
        Frustum fr = frustum_build(&cam);

        CullStats sa, sb;
        cull_batch(&a, &fr, &sa);
        cull_reference(&b, &fr, &sb);
        for (int i = 0; i < N; i++) {
            if (a.pool[i].visible != b.pool[i].visible) mismatches++;
        }
        rejected += sa.rejected_batches;
        exact += sa.exact_tests;
    }

    int ok = mismatches == 0 && rejected > 0;
    printf("cull_fuzz cases=%d particles=%d rejected_batches=%ld exact_tests=%ld "
           "mismatches=%d\n", CASES, N, rejected, exact, mismatches);
    printf("cull_fuzz_ok=%d\n", ok);
    return ok;
}

/* Cull work and wall time must grow linearly with the particle count. */
static int test_cull_linear(void) {
    static const int sizes[] = { 256, 512, 1024, 2048, 4096 };
    enum { NSIZES = 5, TRIALS = 5 };
    static World w;
    double per_particle[NSIZES];
    int work_ok = 1;

    printf("cull_scale");
    for (int k = 0; k < NSIZES; k++) {
        int n = sizes[k];
        world_init(&w);
        w.rng = 0x9E3779B9u;
        for (int i = 0; i < n; i++) {
            Particle* p = &w.pool[i];
            memset(p, 0, sizeof(*p));
            p->alive = 1;
            p->x = 5.0f + rng_unit(&w.rng) * 15.0f;
            p->y = (rng_unit(&w.rng) - 0.5f) * 20.0f;
            p->radius = 0.05f + rng_unit(&w.rng) * 0.15f;
        }
        w.slot_high = n;

        /* Wide frustum: every particle is individually tested (worst case). */
        Camera cam;
        cam.cx = 0.0f; cam.cy = 0.0f; cam.yaw = 0.0f;
        cam.half_angle = 1.3f; cam.near_d = 0.1f; cam.far_d = 1e6f;
        Frustum fr = frustum_build(&cam);

        int reps = 400000 / n;
        if (reps < 16) reps = 16;
        CullStats st;
        clock_t best = 0;
        for (int trial = 0; trial < TRIALS; trial++) {
            clock_t t0 = clock();
            for (int r = 0; r < reps; r++) cull_batch(&w, &fr, &st);
            clock_t t1 = clock();
            if (trial == 0 || t1 - t0 < best) best = t1 - t0;
        }

        int nbatches = (n + BATCH_SIZE - 1) / BATCH_SIZE;
        if (st.bound_probes != 2 * n) work_ok = 0;
        if (st.exact_tests != n) work_ok = 0;
        if (st.plane_tests != 4 * nbatches + 4 * n) work_ok = 0;
        if (st.rejected_batches != 0) work_ok = 0;
        if (w.visible_count != n) work_ok = 0;

        per_particle[k] = 1e9 * (double)best /
                          ((double)CLOCKS_PER_SEC * (double)reps * (double)n);
        printf(" n=%d ns/particle=%.1f probes=%d exact=%d plane=%d",
               n, per_particle[k], st.bound_probes, st.exact_tests, st.plane_tests);
    }
    printf("\n");

    double lo = per_particle[0], hi = per_particle[0];
    for (int k = 1; k < NSIZES; k++) {
        if (per_particle[k] < lo) lo = per_particle[k];
        if (per_particle[k] > hi) hi = per_particle[k];
    }
    int time_ok = lo > 0.0 && hi / lo <= 5.0;
    int ok = work_ok && time_ok;
    printf("cull_scale_work_linear=%d time_spread=%.2fx\n", work_ok, lo > 0.0 ? hi / lo : 0.0);
    printf("cull_scale_ok=%d\n", ok);
    return ok;
}

int main(void) {
    int ok = 1;

    /* --- fixed pool + per-frame quota + emitter priority (unchanged) --- */
    World w; world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    for (int frame = 0; frame < 10; frame++) {
        spawn_frame(&w, 50);
        update_all(&w, 0.1f);
    }
    printf("alive=%d high_water=%d\n", w.alive_count, w.high_water);
    int tag1 = 0, tag2 = 0, tag3 = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) if (w.pool[i].alive) {
        if (w.pool[i].tag == 1) tag1++; else if (w.pool[i].tag == 2) tag2++; else tag3++;
    }
    printf("by_emitter=%d,%d,%d\n", tag1, tag2, tag3);
    /* quota 优先：高优先级 2 应先拿满 30 */
    printf("priority_respected=%d\n", tag2 >= tag3 && tag3 >= 0);
    ok &= (tag2 >= tag3 && tag3 >= 0);

    /* --- frustum visibility culling --- */
    ok &= test_cull_lockstep();
    ok &= test_cull_fuzz();
    ok &= test_cull_linear();

    printf("all_ok=%d\n", ok);
    return ok ? 0 : 1;
}
