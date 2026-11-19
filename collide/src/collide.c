
/* tri-collide: particle vs triangle-mesh collision.
   - swept segment intersection along the motion (no tunneling)
   - BVH broadphase; results identical to per-triangle brute force
   - bounce keeps tangential velocity, restitution on the normal part
   - rest threshold + sleep so settled ("hanging") particles don't jitter
   self-test: brute-force consistency, rest stability, perf budget
   (1000 particles vs 50k tris, < 5 ms/frame). */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* ---------------- vec3 ---------------- */
typedef struct { float x, y, z; } Vec3;
static Vec3 v3(float x, float y, float z) { Vec3 v = {x, y, z}; return v; }
static Vec3 vadd(Vec3 a, Vec3 b) { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static Vec3 vsub(Vec3 a, Vec3 b) { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static Vec3 vscale(Vec3 a, float s) { return v3(a.x*s, a.y*s, a.z*s); }
static float vdot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static Vec3 vcross(Vec3 a, Vec3 b) {
    return v3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
static float vlen(Vec3 a) { return sqrtf(vdot(a, a)); }
static Vec3 vnorm(Vec3 a) { float l = vlen(a); return l > 0 ? vscale(a, 1.0f/l) : v3(0,0,0); }

/* ---------------- mesh ---------------- */
typedef struct {
    Vec3* verts; int nv;
    int*  tris;  int nt;     /* 3 indices per triangle */
    Vec3* nrm;               /* per-triangle unit normal */
    float* bmin;             /* per-triangle aabb, 3 floats */
    float* bmax;
} Mesh;

static void mesh_finalize(Mesh* m) {
    m->nrm  = malloc(sizeof(Vec3) * (size_t)m->nt);
    m->bmin = malloc(sizeof(float) * 3 * (size_t)m->nt);
    m->bmax = malloc(sizeof(float) * 3 * (size_t)m->nt);
    for (int i = 0; i < m->nt; i++) {
        Vec3 a = m->verts[m->tris[3*i+0]];
        Vec3 b = m->verts[m->tris[3*i+1]];
        Vec3 c = m->verts[m->tris[3*i+2]];
        m->nrm[i] = vnorm(vcross(vsub(b, a), vsub(c, a)));
        float* lo = m->bmin + 3*i; float* hi = m->bmax + 3*i;
        const float* v[3] = { &a.x, &b.x, &c.x };
        lo[0] = hi[0] = a.x; lo[1] = hi[1] = a.y; lo[2] = hi[2] = a.z;
        for (int j = 1; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                float val = v[j][k];
                if (val < lo[k]) lo[k] = val;
                if (val > hi[k]) hi[k] = val;
            }
        }
    }
}

static void mesh_free(Mesh* m) {
    free(m->verts); free(m->tris); free(m->nrm); free(m->bmin); free(m->bmax);
    memset(m, 0, sizeof(*m));
}

/* ---------------- segment x triangle (Möller–Trumbore, t in [0, tmax]) ---------------- */
static int seg_tri(Vec3 o, Vec3 d, Vec3 a, Vec3 b, Vec3 c, float tmax, float* t_out) {
    Vec3 e1 = vsub(b, a), e2 = vsub(c, a);
    Vec3 p = vcross(d, e2);
    float det = vdot(e1, p);
    if (fabsf(det) < 1e-9f) return 0;              /* parallel to plane */
    float inv = 1.0f / det;
    Vec3 s = vsub(o, a);
    float u = vdot(s, p) * inv;
    if (u < -1e-6f || u > 1.000001f) return 0;
    Vec3 q = vcross(s, e1);
    float v = vdot(d, q) * inv;
    if (v < -1e-6f || u + v > 1.000001f) return 0;
    float t = vdot(e2, q) * inv;
    if (t < 0.0f || t > tmax) return 0;
    *t_out = t;
    return 1;
}

typedef struct { float t; int tri; } Hit;          /* tri < 0: no hit */

/* reference: test every triangle, keep earliest */
static Hit brute_sweep(const Mesh* m, Vec3 o, Vec3 d) {
    Hit h = { 0.0f, -1 };
    float best = 1.0f;
    for (int i = 0; i < m->nt; i++) {
        float t;
        if (seg_tri(o, d, m->verts[m->tris[3*i+0]], m->verts[m->tris[3*i+1]],
                    m->verts[m->tris[3*i+2]], best, &t) && (h.tri < 0 || t < best)) {
            best = t; h.t = t; h.tri = i;
        }
    }
    return h;
}

/* ---------------- BVH ---------------- */
#define BVH_LEAF 4
typedef struct {
    float bmin[3], bmax[3];
    int left, right;          /* < 0 for leaf */
    int start, count;         /* leaf: range in order[] */
} BvhNode;

typedef struct { BvhNode* nodes; int nnodes; int* order; } Bvh;

static const Mesh* g_sort_mesh;
static int g_sort_axis;
static float tri_centroid_axis(const Mesh* m, int tri, int axis) {
    return (m->bmin[3*tri+axis] + m->bmax[3*tri+axis]) * 0.5f;
}
static int cmp_tri_axis(const void* pa, const void* pb) {
    float ca = tri_centroid_axis(g_sort_mesh, *(const int*)pa, g_sort_axis);
    float cb = tri_centroid_axis(g_sort_mesh, *(const int*)pb, g_sort_axis);
    return (ca > cb) - (ca < cb);
}

static int bvh_build_node(Bvh* b, int* order, int start, int count) {
    int idx = b->nnodes++;
    BvhNode* n = &b->nodes[idx];
    for (int k = 0; k < 3; k++) { n->bmin[k] = 1e30f; n->bmax[k] = -1e30f; }
    for (int i = start; i < start + count; i++) {
        int t = order[i];
        for (int k = 0; k < 3; k++) {
            float lo = g_sort_mesh->bmin[3*t+k], hi = g_sort_mesh->bmax[3*t+k];
            if (lo < n->bmin[k]) n->bmin[k] = lo;
            if (hi > n->bmax[k]) n->bmax[k] = hi;
        }
    }
    if (count <= BVH_LEAF) {
        n->left = n->right = -1; n->start = start; n->count = count;
        return idx;
    }
    int axis = 0;
    float ext0 = n->bmax[0] - n->bmin[0];
    if (n->bmax[1] - n->bmin[1] > ext0) axis = 1;
    if (n->bmax[2] - n->bmin[2] > n->bmax[axis] - n->bmin[axis]) axis = 2;
    g_sort_axis = axis;
    qsort(order + start, (size_t)count, sizeof(int), cmp_tri_axis);
    int half = count / 2;
    n->count = 0;
    n->left  = bvh_build_node(b, order, start, half);
    n->right = bvh_build_node(b, order, start + half, count - half);
    return idx;
}

static void bvh_build(Bvh* b, const Mesh* m) {
    b->nodes = malloc(sizeof(BvhNode) * (size_t)(2 * m->nt));
    b->order = malloc(sizeof(int) * (size_t)m->nt);
    b->nnodes = 0;
    for (int i = 0; i < m->nt; i++) b->order[i] = i;
    g_sort_mesh = m;
    bvh_build_node(b, b->order, 0, m->nt);
}

static void bvh_free(Bvh* b) { free(b->nodes); free(b->order); memset(b, 0, sizeof(*b)); }

static float safe_inv(float d) {
    return fabsf(d) > 1e-12f ? 1.0f / d : (d < 0.0f ? -1e12f : 1e12f);
}

static int seg_box(Vec3 o, Vec3 invd, float tmax, const float* bmin, const float* bmax) {
    float t0 = 0.0f, t1 = tmax;
    const float* op = &o.x; const float* ip = &invd.x;
    for (int k = 0; k < 3; k++) {
        float ta = (bmin[k] - op[k]) * ip[k];
        float tb = (bmax[k] - op[k]) * ip[k];
        float lo = ta < tb ? ta : tb;
        float hi = ta > tb ? ta : tb;
        if (lo > t0) t0 = lo;
        if (hi < t1) t1 = hi;
        if (t0 > t1) return 0;
    }
    return 1;
}

/* accelerated sweep: same earliest-hit semantics as brute_sweep */
static Hit bvh_sweep(const Mesh* m, const Bvh* b, Vec3 o, Vec3 d) {
    Hit h = { 0.0f, -1 };
    float best = 1.0f;
    Vec3 invd = v3(safe_inv(d.x), safe_inv(d.y), safe_inv(d.z));
    int stack[128]; int sp = 0;
    stack[sp++] = 0;
    while (sp > 0) {
        const BvhNode* n = &b->nodes[stack[--sp]];
        if (!seg_box(o, invd, best, n->bmin, n->bmax)) continue;
        if (n->left < 0) {
            for (int i = n->start; i < n->start + n->count; i++) {
                int t = b->order[i];
                float tt;
                if (seg_tri(o, d, m->verts[m->tris[3*t+0]], m->verts[m->tris[3*t+1]],
                            m->verts[m->tris[3*t+2]], best, &tt) && (h.tri < 0 || tt < best)) {
                    best = tt; h.t = tt; h.tri = t;
                }
            }
        } else {
            stack[sp++] = n->left;
            stack[sp++] = n->right;
        }
    }
    return h;
}

/* ---------------- particle response ---------------- */
#define SKIN         1e-3f    /* contact offset along the normal */
#define REST_SPEED   0.10f    /* base rest threshold, m/s */
#define RESTITUTION  0.25f
#define TANGENT_KEEP 0.92f    /* tangential damping per contact (friction) */
#define MAX_ITERS    4

typedef struct { Vec3 pos, vel; int sleeping; } P;

/* One integration step: sweep the motion segment, bounce/slide on hits,
   go to sleep when slow and supported (kills resting-contact jitter). */
static void particle_step(const Mesh* m, const Bvh* b, P* p, Vec3 g, float dt) {
    Vec3 up = vscale(vnorm(g), -1.0f);
    if (p->sleeping) {
        /* cheap support probe: still resting on something? */
        Hit h = bvh_sweep(m, b, p->pos, vscale(vnorm(g), SKIN * 4.0f));
        if (h.tri < 0) p->sleeping = 0;    /* support gone, wake up */
        else return;
    }
    p->vel = vadd(p->vel, vscale(g, dt));
    /* rest threshold must exceed one frame of gravity gain, otherwise a
       settled particle re-bounces every frame -> jitter */
    float rest_speed = fmaxf(REST_SPEED, 1.2f * vlen(g) * dt);
    float remaining = 1.0f;
    for (int it = 0; it < MAX_ITERS && remaining > 1e-4f; it++) {
        Vec3 seg = vscale(p->vel, dt * remaining);
        if (vlen(seg) < 1e-7f) break;
        Hit h = bvh_sweep(m, b, p->pos, seg);
        if (h.tri < 0) { p->pos = vadd(p->pos, seg); break; }
        Vec3 n = m->nrm[h.tri];
        if (vdot(n, p->vel) > 0.0f) n = vscale(n, -1.0f);   /* face the particle */
        p->pos = vadd(p->pos, vadd(vscale(seg, h.t), vscale(n, SKIN)));
        float vn = vdot(p->vel, n);
        Vec3 vt = vsub(p->vel, vscale(n, vn));               /* tangential part */
        /* rest when the *outgoing* bounce speed is below threshold, not the
           incoming one: a bounce too weak to escape one frame of gravity
           re-collides next frame and feeds a bounce limit cycle (jitter) */
        if (-vn * RESTITUTION < rest_speed) {
            /* resting contact: drop normal component, keep (damped) tangent */
            p->vel = vscale(vt, TANGENT_KEEP);
            if (vlen(p->vel) < rest_speed && vdot(n, up) > 0.5f) {
                p->vel = v3(0, 0, 0);
                p->sleeping = 1;
                return;
            }
        } else {
            /* bounce: tangential preserved, normal scaled by restitution */
            p->vel = vsub(vscale(vt, TANGENT_KEEP), vscale(n, vn * RESTITUTION));
        }
        remaining *= 1.0f - h.t;
    }
}

/* ---------------- test helpers ---------------- */
static unsigned rng_s = 12345u;
static float frand(void) {   /* deterministic xorshift, [0,1) */
    rng_s ^= rng_s << 13; rng_s ^= rng_s >> 17; rng_s ^= rng_s << 5;
    return (rng_s >> 8) * (1.0f / 16777216.0f);
}
static float frange(float lo, float hi) { return lo + (hi - lo) * frand(); }

static Mesh make_random_mesh(int nt, float extent) {
    Mesh m; memset(&m, 0, sizeof(m));
    m.nv = nt * 3; m.nt = nt;
    m.verts = malloc(sizeof(Vec3) * (size_t)m.nv);
    m.tris  = malloc(sizeof(int) * 3 * (size_t)nt);
    for (int i = 0; i < m.nv; i++)
        m.verts[i] = v3(frange(-extent, extent), frange(-extent, extent), frange(-extent, extent));
    for (int i = 0; i < 3 * nt; i++) m.tris[i] = i;
    mesh_finalize(&m);
    return m;
}

static Mesh make_floor(float y, float half) {
    Mesh m; memset(&m, 0, sizeof(m));
    m.nv = 4; m.nt = 2;
    m.verts = malloc(sizeof(Vec3) * 4);
    m.tris  = malloc(sizeof(int) * 6);
    m.verts[0] = v3(-half, y, -half); m.verts[1] = v3( half, y, -half);
    m.verts[2] = v3( half, y,  half); m.verts[3] = v3(-half, y,  half);
    m.tris[0] = 0; m.tris[1] = 1; m.tris[2] = 2;
    m.tris[3] = 0; m.tris[4] = 2; m.tris[5] = 3;
    mesh_finalize(&m);
    return m;
}

static float terrain_h(float x, float z) {
    return 0.9f*sinf(x*0.35f)*cosf(z*0.28f) + 0.25f*sinf(x*1.3f + z*0.7f);
}

static Mesh make_terrain(int n, float half) {
    Mesh m; memset(&m, 0, sizeof(m));
    m.nv = (n+1)*(n+1); m.nt = n*n*2;
    m.verts = malloc(sizeof(Vec3) * (size_t)m.nv);
    m.tris  = malloc(sizeof(int) * 3 * (size_t)m.nt);
    for (int j = 0; j <= n; j++)
        for (int i = 0; i <= n; i++) {
            float x = -half + 2.0f*half*i/n, z = -half + 2.0f*half*j/n;
            m.verts[j*(n+1)+i] = v3(x, terrain_h(x, z), z);
        }
    int t = 0;
    for (int j = 0; j < n; j++)
        for (int i = 0; i < n; i++) {
            int a = j*(n+1)+i, b = a+1, c = a+(n+1), d = c+1;
            m.tris[3*t+0]=a; m.tris[3*t+1]=c; m.tris[3*t+2]=b; t++;
            m.tris[3*t+0]=b; m.tris[3*t+1]=c; m.tris[3*t+2]=d; t++;
        }
    mesh_finalize(&m);
    return m;
}

static double now_ms(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

int main(void) {
    Vec3 gravity = v3(0, -9.8f, 0);
    int ok = 1;

    /* 1. BVH sweep must match per-triangle brute force */
    {
        rng_s = 99u;
        Mesh m = make_random_mesh(4096, 10.0f);
        Bvh b; bvh_build(&b, &m);
        int segments = 20000, mism = 0, hits = 0;
        for (int i = 0; i < segments; i++) {
            Vec3 o = v3(frange(-12,12), frange(-12,12), frange(-12,12));
            Vec3 d = v3(frange(-3,3), frange(-3,3), frange(-3,3));
            Hit hb = brute_sweep(&m, o, d);
            Hit hv = bvh_sweep(&m, &b, o, d);
            if ((hb.tri < 0) != (hv.tri < 0)) { mism++; continue; }
            if (hb.tri >= 0) {
                hits++;
                if (fabsf(hb.t - hv.t) > 1e-5f) mism++;
            }
        }
        printf("consistency: segments=%d hits=%d mismatches=%d\n", segments, hits, mism);
        if (mism) ok = 0;
        bvh_free(&b); mesh_free(&m);
    }

    /* 2. rest on a floor: settle, sleep, zero jitter, no sink-through */
    {
        rng_s = 7u;
        Mesh m = make_floor(0.0f, 30.0f);
        Bvh b; bvh_build(&b, &m);
        enum { NP = 100, FRAMES = 600 };
        P* ps = calloc(NP, sizeof(P));
        for (int i = 0; i < NP; i++)
            ps[i].pos = v3(frange(-5,5), frange(1,5), frange(-5,5));
        float dt = 1.0f/60.0f;
        int all_asleep_frame = -1;
        float max_jitter = 0.0f, min_y = 1e30f;
        for (int f = 0; f < FRAMES; f++) {
            int asleep = 0;
            for (int i = 0; i < NP; i++) {
                Vec3 before = ps[i].pos;
                particle_step(&m, &b, &ps[i], gravity, dt);
                if (f >= FRAMES - 120) {
                    float moved = vlen(vsub(ps[i].pos, before));
                    if (moved > max_jitter) max_jitter = moved;
                }
                if (ps[i].sleeping) asleep++;
                if (ps[i].pos.y < min_y) min_y = ps[i].pos.y;
            }
            if (asleep == NP && all_asleep_frame < 0) all_asleep_frame = f;
        }
        int asleep = 0;
        for (int i = 0; i < NP; i++) if (ps[i].sleeping) asleep++;
        printf("rest: asleep=%d/%d all_asleep_frame=%d max_jitter=%.2e min_y=%.5f\n",
               asleep, NP, all_asleep_frame, max_jitter, min_y);
        if (asleep != NP || max_jitter > 1e-4f || min_y < -1e-3f) ok = 0;
        free(ps); bvh_free(&b); mesh_free(&m);
    }

    /* 3. perf: 1000 particles vs ~50k triangles, budget 5 ms/frame */
    {
        rng_s = 2026u;
        Mesh m = make_terrain(158, 20.0f);           /* 158^2*2 = 49928 tris */
        Bvh b; bvh_build(&b, &m);
        enum { NP = 1000, FRAMES = 240 };
        P* ps = calloc(NP, sizeof(P));
        for (int i = 0; i < NP; i++)
            ps[i].pos = v3(frange(-18,18), frange(3,9), frange(-18,18));
        float dt = 1.0f/60.0f;
        double t0 = now_ms();
        for (int f = 0; f < FRAMES; f++)
            for (int i = 0; i < NP; i++)
                particle_step(&m, &b, &ps[i], gravity, dt);
        double avg = (now_ms() - t0) / FRAMES;
        int sane = 1;
        for (int i = 0; i < NP; i++)
            if (!isfinite(ps[i].pos.x + ps[i].pos.y + ps[i].pos.z)) sane = 0;
        printf("perf: tris=%d particles=%d frames=%d avg_ms=%.3f budget_ms=5.0 sane=%d\n",
               m.nt, NP, FRAMES, avg, sane);
        if (avg >= 5.0 || !sane) ok = 0;
        free(ps); bvh_free(&b); mesh_free(&m);
    }

    printf("result=%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
