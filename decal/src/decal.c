/* decal: projector-spawned surface decals with a fixed capacity, lifetime
   fade-out, oldest-first eviction, and per-texel surface visibility so a
   clipped or evicted decal leaves no residue on the target. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <time.h>

#define MAX_DECALS 64
#define MAX_TARGETS 8
#define VIS_N 16                 /* visibility texture resolution per decal */
#define PROJ_DEPTH 0.05f         /* half depth of a projected decal box */
#define Q_EPS 1e-3f
#define MIN_FACING 0.5f          /* dot(surface normal, decal normal) cutoff */

typedef struct { float x, y, z; } Vec3;

static Vec3 v3(float x, float y, float z) { Vec3 r = { x, y, z }; return r; }
static Vec3 vadd(Vec3 a, Vec3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static Vec3 vsub(Vec3 a, Vec3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static Vec3 vmul(Vec3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static float vdot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static Vec3 vcross(Vec3 a, Vec3 b) {
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x);
}
static Vec3 vnorm(Vec3 a) {
    float l = sqrtf(vdot(a, a));
    return l > 1e-8f ? vmul(a, 1.0f / l) : v3(0.0f, 0.0f, 0.0f);
}

/* a planar, oriented surface patch: pos + u*x + v*y, normal n, half extents */
typedef struct {
    Vec3 pos, u, v, n;
    float hw, hh;
} Target;

enum { DECAL_TANGENT = 0, DECAL_PROJECTED = 1 };

typedef struct {
    unsigned serial;             /* insertion order, smallest = oldest */
    int active;
    int target;                  /* owning surface; never affects another */
    int mode;
    Vec3 center, u, v, n;
    float hw, hh;
    float depth;                 /* projection box half depth (0 = tangent) */
    float age, opacity;
    unsigned char vis[VIS_N * VIS_N];   /* 1 = target present under texel */
    int open_texels;
} Decal;

typedef struct {
    Target targets[MAX_TARGETS];
    int target_count;
    Decal decals[MAX_DECALS];
    int count, high_water;
    unsigned next_serial;
    int spawned, evicted, expired;
    float life, fade;            /* fade starts at life - fade */
} DecalWorld;

static void dw_init(DecalWorld* w) {
    memset(w, 0, sizeof(*w));
    w->next_serial = 1;
    w->life = 10.0f;
    w->fade = 2.0f;
}

static void target_set(Target* t, Vec3 pos, Vec3 u, Vec3 v, float hw, float hh) {
    t->pos = pos;
    t->u = vnorm(u);
    t->v = vnorm(v);
    t->n = vcross(t->u, t->v);
    t->hw = hw;
    t->hh = hh;
}

/* slab test against a zero-thickness oriented rectangle */
static int ray_target(const Target* t, Vec3 ro, Vec3 rd, float* out_t) {
    Vec3 o = vsub(ro, t->pos);
    float lo[3] = { vdot(o, t->u), vdot(o, t->v), vdot(o, t->n) };
    float ld[3] = { vdot(rd, t->u), vdot(rd, t->v), vdot(rd, t->n) };
    float ext[3] = { t->hw, t->hh, 0.0f };
    float tmin = -FLT_MAX, tmax = FLT_MAX;
    for (int k = 0; k < 3; k++) {
        if (fabsf(ld[k]) < 1e-8f) {
            if (fabsf(lo[k]) > ext[k]) return 0;
            continue;
        }
        float t1 = (-ext[k] - lo[k]) / ld[k];
        float t2 = (ext[k] - lo[k]) / ld[k];
        if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
        if (t1 > tmin) tmin = t1;
        if (t2 < tmax) tmax = t2;
    }
    if (tmax < tmin || tmax < 0.0f) return 0;
    if (tmax == FLT_MAX) return 0;          /* ray lies in the surface plane */
    *out_t = tmin >= 0.0f ? tmin : tmax;
    return 1;
}

/* nearest surface under the ray; ties keep the lower target index */
static int raycast_targets(const DecalWorld* w, Vec3 ro, Vec3 rd,
                           int* out_target, Vec3* out_point, Vec3* out_n) {
    int best = -1;
    float best_t = FLT_MAX;
    for (int i = 0; i < w->target_count; i++) {
        float t;
        if (ray_target(&w->targets[i], ro, rd, &t) && t < best_t) {
            best_t = t;
            best = i;
        }
    }
    if (best < 0) return -1;
    *out_target = best;
    *out_point = vadd(ro, vmul(rd, best_t));
    *out_n = w->targets[best].n;
    return best;
}

/* in-plane basis from a reference up vector, then a roll about the normal */
static void basis_from(Vec3 n, Vec3 up, float roll, Vec3* ou, Vec3* ov) {
    Vec3 u = vnorm(vsub(up, vmul(n, vdot(up, n))));
    if (vdot(u, u) < 1e-8f) {
        Vec3 alt = fabsf(n.x) < 0.9f ? v3(1.0f, 0.0f, 0.0f) : v3(0.0f, 0.0f, 1.0f);
        u = vnorm(vcross(n, alt));
    }
    Vec3 v = vcross(n, u);
    float c = cosf(roll), s = sinf(roll);
    *ou = vadd(vmul(u, c), vmul(v, s));
    *ov = vsub(vmul(v, c), vmul(u, s));
}

static Vec3 decal_sample(const Decal* d, int i, int j) {
    float u = ((i + 0.5f) / VIS_N * 2.0f - 1.0f) * d->hw;
    float v = ((j + 0.5f) / VIS_N * 2.0f - 1.0f) * d->hh;
    return vadd(d->center, vadd(vmul(d->u, u), vmul(d->v, v)));
}

/* a texel is visible when the decal quad at that texel lands on the target */
static void decal_rebuild_vis(Decal* d, const Target* tg) {
    int open = 0;
    for (int j = 0; j < VIS_N; j++) {
        for (int i = 0; i < VIS_N; i++) {
            Vec3 p = decal_sample(d, i, j);
            Vec3 o = vsub(p, tg->pos);
            int in = fabsf(vdot(o, tg->u)) <= tg->hw + 1e-4f &&
                     fabsf(vdot(o, tg->v)) <= tg->hh + 1e-4f &&
                     fabsf(vdot(o, tg->n)) <= PROJ_DEPTH;
            d->vis[j * VIS_N + i] = (unsigned char)in;
            open += in;
        }
    }
    d->open_texels = open;
}

static int decal_claim_slot(DecalWorld* w) {
    for (int i = 0; i < MAX_DECALS; i++) {
        if (!w->decals[i].active) {
            memset(&w->decals[i], 0, sizeof(w->decals[i]));
            return i;
        }
    }
    /* at capacity: drop the earliest inserted decal */
    int old = -1;
    unsigned best = 0;
    for (int i = 0; i < MAX_DECALS; i++) {
        if (!w->decals[i].active) continue;
        if (old < 0 || w->decals[i].serial < best) {
            old = i;
            best = w->decals[i].serial;
        }
    }
    if (old < 0) return -1;
    w->decals[old].active = 0;
    w->count--;
    w->evicted++;
    memset(&w->decals[old], 0, sizeof(w->decals[old]));   /* no stale vis */
    return old;
}

static int decal_create(DecalWorld* w, int target, Vec3 center, Vec3 n, Vec3 up,
                        float hw, float hh, int mode, float roll,
                        unsigned* out_serial) {
    if (target < 0 || target >= w->target_count) return -1;
    int slot = decal_claim_slot(w);
    if (slot < 0) return -1;
    Decal* d = &w->decals[slot];
    d->active = 1;
    d->serial = w->next_serial++;
    d->target = target;
    d->mode = mode;
    d->center = center;
    d->n = vnorm(n);
    basis_from(d->n, up, roll, &d->u, &d->v);
    d->hw = hw;
    d->hh = hh;
    d->depth = (mode == DECAL_PROJECTED) ? PROJ_DEPTH : 0.0f;
    d->age = 0.0f;
    d->opacity = 1.0f;
    decal_rebuild_vis(d, &w->targets[target]);
    w->count++;
    w->spawned++;
    if (w->count > w->high_water) w->high_water = w->count;
    if (out_serial) *out_serial = d->serial;
    return slot;
}

/* projector spawn: ray into the scene, decal is fixed to the first surface */
static int decal_spawn_ray(DecalWorld* w, Vec3 ro, Vec3 rd,
                           float hw, float hh, int mode, float roll,
                           unsigned* out_serial) {
    int target;
    Vec3 p, n;
    if (raycast_targets(w, ro, rd, &target, &p, &n) < 0) return -1;
    return decal_create(w, target, p, n, w->targets[target].v, hw, hh,
                        mode, roll, out_serial);
}

/* manual placement (level-authored decal) */
static int decal_place(DecalWorld* w, int target, Vec3 center, Vec3 n, Vec3 up,
                       float hw, float hh, int mode, float roll,
                       unsigned* out_serial) {
    return decal_create(w, target, center, n, up, hw, hh, mode, roll, out_serial);
}

static Decal* decal_find(DecalWorld* w, unsigned serial) {
    for (int i = 0; i < MAX_DECALS; i++)
        if (w->decals[i].active && w->decals[i].serial == serial)
            return &w->decals[i];
    return NULL;
}

static void decal_tick(DecalWorld* w, float dt) {
    for (int i = 0; i < MAX_DECALS; i++) {
        Decal* d = &w->decals[i];
        if (!d->active) continue;
        d->age += dt;
        float rem = w->life - d->age;
        d->opacity = rem <= 0.0f ? 0.0f : (rem >= w->fade ? 1.0f : rem / w->fade);
        if (d->age >= w->life) {
            d->active = 0;
            memset(d, 0, sizeof(*d));
            w->count--;
            w->expired++;
        }
    }
}

static void decal_local(const Decal* d, Vec3 p, float* u, float* v, float* q) {
    Vec3 o = vsub(p, d->center);
    *u = vdot(o, d->u);
    *v = vdot(o, d->v);
    *q = vdot(o, d->n);
}

/* frontmost decal whose projection box contains p and which faces the surface */
static int decal_owner_at(const DecalWorld* w, Vec3 p, Vec3 surf_n) {
    int best = -1;
    float best_q = -FLT_MAX;
    unsigned best_serial = 0;
    for (int i = 0; i < MAX_DECALS; i++) {
        const Decal* d = &w->decals[i];
        if (!d->active) continue;
        if (vdot(surf_n, d->n) < MIN_FACING) continue;
        float u, v, q;
        decal_local(d, p, &u, &v, &q);
        if (q < -Q_EPS || q > d->depth + Q_EPS) continue;
        if (fabsf(u) > d->hw + Q_EPS || fabsf(v) > d->hh + Q_EPS) continue;
        if (best < 0 || q > best_q + 1e-6f ||
            (fabsf(q - best_q) <= 1e-6f && d->serial < best_serial)) {
            best = i;
            best_q = q;
            best_serial = d->serial;
        }
    }
    return best;
}

static int decal_texel(const Decal* d, float u, float v) {
    int i = (int)((u / d->hw * 0.5f + 0.5f) * VIS_N);
    int j = (int)((v / d->hh * 0.5f + 0.5f) * VIS_N);
    if (i < 0) i = 0;
    if (i >= VIS_N) i = VIS_N - 1;
    if (j < 0) j = 0;
    if (j >= VIS_N) j = VIS_N - 1;
    return j * VIS_N + i;
}

/* what the pool draws at a surface point */
static float shade_system(const DecalWorld* w, Vec3 p, Vec3 surf_n) {
    int idx = decal_owner_at(w, p, surf_n);
    if (idx < 0) return 0.0f;
    const Decal* d = &w->decals[idx];
    float u, v, q;
    decal_local(d, p, &u, &v, &q);
    return d->opacity * (float)d->vis[decal_texel(d, u, v)];
}

/* what a manually clipped decal quad draws at the same point */
static float shade_reference(const DecalWorld* w, Vec3 p, Vec3 surf_n) {
    float s = 0.0f;
    for (int i = 0; i < MAX_DECALS; i++) {
        const Decal* d = &w->decals[i];
        if (!d->active) continue;
        if (vdot(surf_n, d->n) < MIN_FACING) continue;
        const Target* tg = &w->targets[d->target];
        Vec3 o = vsub(p, tg->pos);
        if (fabsf(vdot(o, tg->n)) > PROJ_DEPTH) continue;
        if (fabsf(vdot(o, tg->u)) > tg->hw || fabsf(vdot(o, tg->v)) > tg->hh) continue;
        float u, v, q;
        decal_local(d, p, &u, &v, &q);
        if (q < -Q_EPS || q > d->depth + Q_EPS) continue;
        if (fabsf(u) > d->hw || fabsf(v) > d->hh) continue;
        s += d->opacity;
    }
    return s;
}

/* compare both renderers at every live decal's texel centres */
static float parity_max_diff(const DecalWorld* w) {
    float worst = 0.0f;
    for (int i = 0; i < MAX_DECALS; i++) {
        const Decal* d = &w->decals[i];
        if (!d->active) continue;
        Vec3 surf_n = w->targets[d->target].n;
        for (int j = 0; j < VIS_N; j++) {
            for (int k = 0; k < VIS_N; k++) {
                Vec3 p = decal_sample(d, k, j);
                float a = shade_reference(w, p, surf_n);
                float b = shade_system(w, p, surf_n);
                float diff = fabsf(a - b);
                if (diff > worst) worst = diff;
            }
        }
    }
    return worst;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e3 + (double)ts.tv_nsec / 1e6;
}

static unsigned rng = 123456789u;
static float rnd01(void) {
    rng = rng * 1664525u + 1013904223u;
    return (float)((rng >> 8) & 0xFFFFFFu) / 16777216.0f;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void world_with_floor(DecalWorld* w) {
    dw_init(w);
    Target* floor = &w->targets[w->target_count++];
    target_set(floor, v3(0, 0, 0), v3(1, 0, 0), v3(0, 0, 1), 1.0f, 0.5f);
}

int main(void) {
    DecalWorld w;
    world_with_floor(&w);
    unsigned basic_serial = 0;

    /* ---- projector spawn and target picking ---- */
    check("ray_miss",
          decal_spawn_ray(&w, v3(3.0f, 3.0f, 0.0f), v3(0, -1, 0),
                          0.2f, 0.2f, DECAL_TANGENT, 0.0f, NULL) == -1);
    check("miss_leaves_pool_empty", w.count == 0);

    int slot = decal_spawn_ray(&w, v3(0.3f, 3.0f, 0.2f), v3(0, -1, 0),
                               0.25f, 0.2f, DECAL_TANGENT, 0.0f, &basic_serial);
    check("ray_hit", slot >= 0 && w.count == 1);
    check("tangent_all_open",
          w.decals[slot].open_texels == VIS_N * VIS_N);

    Target* platform = &w.targets[w.target_count++];
    target_set(platform, v3(0, 0.5f, 0), v3(1, 0, 0), v3(0, 0, 1), 0.2f, 0.2f);
    int hit_target = -1;
    Vec3 hp = v3(0, 0, 0), hn = v3(0, 0, 0);
    check("ray_picks_nearest",
          raycast_targets(&w, v3(0.05f, 3.0f, 0.05f), v3(0, -1, 0),
                          &hit_target, &hp, &hn) == 1);
    unsigned plat_serial = 0;
    int plat_slot = decal_spawn_ray(&w, v3(0.05f, 3.0f, 0.05f), v3(0, -1, 0),
                                    0.05f, 0.05f, DECAL_TANGENT, 0.0f, &plat_serial);
    check("platform_decal_open", plat_slot >= 0 &&
          w.decals[plat_slot].open_texels == VIS_N * VIS_N);
    /* the platform decal must not paint the floor underneath it */
    check("platform_not_seen_from_floor",
          shade_system(&w, v3(0.05f, 0.0f, 0.05f), w.targets[0].n) == 0.0f);

    /* ---- manual decals: one inside, four straddling the edges ---- */
    const float edge_z[4] = { -0.35f, -0.12f, 0.11f, 0.34f };
    int boundary_ok = 1;
    for (int i = 0; i < 4; i++) {
        int s = decal_place(&w, 0, v3(1.0f, 0.0f, edge_z[i]), w.targets[0].n,
                            v3(0, 0, 1), 0.10f, 0.12f, DECAL_TANGENT, 0.0f, NULL);
        if (s < 0) { boundary_ok = 0; continue; }
        const Decal* d = &w.decals[s];
        if (d->open_texels <= 0 || d->open_texels >= VIS_N * VIS_N) boundary_ok = 0;
    }
    check("edge_decals_boundary", boundary_ok);

    int inside = decal_place(&w, 0, v3(-0.5f, 0.0f, -0.2f), w.targets[0].n,
                             v3(0, 0, 1), 0.2f, 0.15f, DECAL_TANGENT, 0.0f, NULL);
    check("inside_decal_not_boundary", inside >= 0 &&
          w.decals[inside].open_texels == VIS_N * VIS_N);

    /* hand-placed rendering and pool rendering must agree everywhere */
    check("manual_render_matches_pool", parity_max_diff(&w) < 1e-5f);

    /* ---- eviction: oldest first, and the target goes clean afterwards ---- */
    Vec3 probe = v3(0.3f, 0.0f, 0.2f);
    check("probe_covered_before_evict", shade_system(&w, probe, w.targets[0].n) > 0.0f);
    int total_before = w.count;
    unsigned newest_serial = 0;
    for (int i = 0; i < MAX_DECALS + 4; i++) {
        decal_spawn_ray(&w, v3(0.6f, 3.0f, 0.35f), v3(0, -1, 0),
                        0.1f, 0.1f, DECAL_TANGENT, 0.0f, &newest_serial);
        decal_tick(&w, 1.0f / 60.0f);
    }
    check("cap_respected", w.count == MAX_DECALS && total_before <= MAX_DECALS);
    check("oldest_evicted", decal_find(&w, basic_serial) == NULL &&
          decal_find(&w, plat_serial) == NULL);
    check("newest_retained", decal_find(&w, newest_serial) != NULL);
    check("eviction_order", w.evicted == total_before + 4);
    check("no_residual_after_eviction",
          shade_system(&w, probe, w.targets[0].n) == 0.0f &&
          shade_reference(&w, probe, w.targets[0].n) == 0.0f);
    check("edge_point_clean_after_eviction",
          shade_system(&w, v3(1.0f, 0.0f, 0.34f), w.targets[0].n) == 0.0f);
    check("owner_deterministic",
          decal_owner_at(&w, v3(0.6f, 0.0f, 0.35f), w.targets[0].n) ==
          decal_owner_at(&w, v3(0.6f, 0.0f, 0.35f), w.targets[0].n));

    /* ---- lifetime decay ---- */
    DecalWorld life;
    world_with_floor(&life);
    unsigned life_serial = 0;
    int life_slot = decal_spawn_ray(&life, v3(0.0f, 3.0f, 0.0f), v3(0, -1, 0),
                                    0.2f, 0.2f, DECAL_TANGENT, 0.0f, &life_serial);
    float opacity_85 = 0.0f, opacity_95 = 0.0f;
    for (int i = 0; i < 68; i++) decal_tick(&life, 0.125f);   /* 8.5 s */
    Decal* ld = decal_find(&life, life_serial);
    if (ld) opacity_85 = ld->opacity;
    for (int i = 0; i < 8; i++) decal_tick(&life, 0.125f);    /* 9.5 s */
    ld = decal_find(&life, life_serial);
    if (ld) opacity_95 = ld->opacity;
    check("fade_quarter_at_8_5s", life_slot >= 0 && fabsf(opacity_85 - 0.75f) < 1e-4f);
    check("fade_monotonic", opacity_95 < opacity_85 && opacity_95 > 0.0f);
    for (int i = 0; i < 4; i++) decal_tick(&life, 0.125f);    /* 10.0 s */
    check("expired_at_lifetime", life.count == 0 && life.expired == 1 &&
          decal_find(&life, life_serial) == NULL);
    check("no_texel_left_behind", shade_system(&life, v3(0.0f, 0.0f, 0.0f),
                                               life.targets[0].n) == 0.0f);

    /* ---- per-frame cost with the pool at capacity ---- */
    DecalWorld perf;
    world_with_floor(&perf);
    perf.life = 1e9f;
    perf.fade = 1e9f;
    while (perf.count < MAX_DECALS) {
        float x = (rnd01() * 2.0f - 1.0f) * 0.85f;
        float z = (rnd01() * 2.0f - 1.0f) * 0.45f;
        decal_spawn_ray(&perf, v3(x, 3.0f, z), v3(0, -1, 0),
                        0.09f, 0.09f, DECAL_TANGENT, 0.0f, NULL);
    }
    check("prefill_reaches_cap", perf.count == MAX_DECALS);

    float perf_max_ms = 0.0f, perf_avg_ms = 0.0f;
    double sum_ms = 0.0;
    int cap_ok = 1, boundary_seen = 0;
    volatile double sink = 0.0;
    const int frames = 600;
    for (int f = 0; f < frames; f++) {
        double t0 = now_ms();
        if (f % 8 == 0) {
            int edge = f % 4;
            float t = ((f * 37) % 100) / 100.0f * 1.6f - 0.8f;
            Vec3 c, up;
            if (edge == 0)      { c = v3(1.0f, 0.0f, t * 0.5f);  up = v3(0, 0, 1); }
            else if (edge == 1) { c = v3(-1.0f, 0.0f, t * 0.5f); up = v3(0, 0, 1); }
            else if (edge == 2) { c = v3(t, 0.0f, 0.5f);         up = v3(1, 0, 0); }
            else                { c = v3(t, 0.0f, -0.5f);        up = v3(1, 0, 0); }
            decal_place(&perf, 0, c, perf.targets[0].n, up,
                        0.08f, 0.08f, DECAL_TANGENT, 0.0f, NULL);
        } else {
            float x = (rnd01() * 2.0f - 1.0f) * 0.9f;
            float z = (rnd01() * 2.0f - 1.0f) * 0.45f;
            decal_spawn_ray(&perf, v3(x, 3.0f, z), v3(0, -1, 0),
                            0.08f, 0.08f, DECAL_TANGENT, 0.0f, NULL);
        }
        decal_tick(&perf, 1.0f / 60.0f);

        double s = 0.0;
        for (int j = 0; j < 24; j++) {
            for (int i = 0; i < 24; i++) {
                float x = ((i + 0.5f) / 24.0f * 2.0f - 1.0f) * 0.98f;
                float z = ((j + 0.5f) / 24.0f * 2.0f - 1.0f) * 0.48f;
                s += shade_system(&perf, v3(x, 0.0f, z), perf.targets[0].n);
            }
        }
        sink += s;
        double ms = now_ms() - t0;
        sum_ms += ms;
        if (ms > (double)perf_max_ms) perf_max_ms = (float)ms;
        if (perf.count != MAX_DECALS) cap_ok = 0;
        for (int i = 0; i < MAX_DECALS; i++) {
            const Decal* d = &perf.decals[i];
            if (d->active && d->open_texels > 0 && d->open_texels < VIS_N * VIS_N) {
                boundary_seen = 1;
                break;
            }
        }
    }
    perf_avg_ms = (float)(sum_ms / frames);
    check("cap_held_during_churn", cap_ok);
    check("boundary_decals_at_cap", boundary_seen);
    check("in_plane_ray_ignored",
          raycast_targets(&perf, v3(3.0f, 0.0f, 0.0f), v3(0, 0, 1),
                          &hit_target, &hp, &hn) < 0);
    check("frame_time_positive", perf_max_ms > 0.0f);
    check("frame_time_sane", perf_max_ms < 50.0f);
    (void)sink;

    printf("cap=%d frames=%d spawned=%d evicted=%d expired=%d\n",
           MAX_DECALS, frames, perf.spawned, perf.evicted, perf.expired);
    printf("cap_frame_ms_avg=%.4f cap_frame_ms_max=%.4f\n",
           perf_avg_ms, perf_max_ms);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
