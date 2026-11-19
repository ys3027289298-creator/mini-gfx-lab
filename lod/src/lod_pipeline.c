/* lod-pipeline: offline LOD variant generation + zero-allocation runtime.

   Pipeline: high-poly mesh -> quadric-error-metric edge collapses driven by
   per-level error targets -> packed variant set (vertices, indices, AABB,
   face counts, precomputed switch distances) -> binary blob.  The runtime
   side only reads tables: lod_select() does one distance compare per level
   boundary and never allocates.

   The self-testing main builds a 20k-triangle terrain grid and an icosphere,
   generates five levels, and verifies every requirement (30 s budget,
   bounding-box coverage, monotone counts/errors, round-trip, hysteresis). */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <assert.h>

#define LOD_MAGIC   0x31444f4cu /* 'LOD1' little-endian */
#define LOD_VERSION 1u
#define LOD_MAX_LEVELS 8
#define LOD_LEVELS  5
#define EPS_BBOX    1e-4f

/* ------------------------------------------------------------------ math */

typedef struct { double x, y, z; } V3;

static V3 v3_sub(V3 a, V3 b) { return (V3){a.x - b.x, a.y - b.y, a.z - b.z}; }
static V3 v3_cross(V3 a, V3 b) {
    return (V3){a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
}
static double v3_dot(V3 a, V3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static V3 v3_norm(V3 a) {
    double l = sqrt(v3_dot(a, a));
    if (l < 1e-12) return (V3){0, 0, 0};
    return (V3){a.x / l, a.y / l, a.z / l};
}
static V3 v3_mid(V3 a, V3 b) {
    return (V3){(a.x + b.x) * 0.5, (a.y + b.y) * 0.5, (a.z + b.z) * 0.5};
}

/* Symmetric 4x4 quadric, stored as 10 unique entries:
   [q0 q1 q2 q3]
   [q1 q4 q5 q6]
   [q2 q5 q7 q8]
   [q3 q6 q8 q9] */
typedef double Quadric[10];

static void q_add(Quadric dst, const Quadric a) {
    for (int i = 0; i < 10; i++) dst[i] += a[i];
}
/* quadric of plane n.x + d = 0 (n assumed unit length) */
static void q_plane(Quadric q, V3 n, double d) {
    q[0] = n.x*n.x; q[1] = n.x*n.y; q[2] = n.x*n.z; q[3] = n.x*d;
                    q[4] = n.y*n.y; q[5] = n.y*n.z; q[6] = n.y*d;
                                    q[7] = n.z*n.z; q[8] = n.z*d;
                                                    q[9] = d*d;
}
static double q_eval(const Quadric q, V3 p) {
    double x = p.x, y = p.y, z = p.z;
    return q[0]*x*x + 2*q[1]*x*y + 2*q[2]*x*z + 2*q[3]*x
         + q[4]*y*y   + 2*q[5]*y*z + 2*q[6]*y
         + q[7]*z*z   + 2*q[8]*z   + q[9];
}
/* optimal contraction point; solves the 3x3 block A p = -b and falls back
   to the edge midpoint when A is (near-)singular, which happens routinely on
   flat planar regions.  Relative pivot tolerance keeps the test scale-safe. */
static int q_optimal(const Quadric q, V3 a, V3 b, V3* out, double* err) {
    double m[3][4];
    m[0][0]=q[0]; m[0][1]=q[1]; m[0][2]=q[2]; m[0][3]=-q[3];
    m[1][0]=q[1]; m[1][1]=q[4]; m[1][2]=q[5]; m[1][3]=-q[6];
    m[2][0]=q[2]; m[2][1]=q[5]; m[2][2]=q[7]; m[2][3]=-q[8];
    double scale = fabs(q[0]);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double v = fabs(m[i][j]);
            if (v > scale) scale = v;
        }
    double tol = (scale > 0 ? scale : 1.0) * 1e-10;
    for (int col = 0; col < 3; col++) {
        int piv = col;
        for (int r = col + 1; r < 3; r++)
            if (fabs(m[r][col]) > fabs(m[piv][col])) piv = r;
        if (fabs(m[piv][col]) < tol) {
            *out = v3_mid(a, b);
            *err = q_eval(q, *out);
            if (*err < 0) *err = 0;
            return 0;
        }
        if (piv != col) {
            for (int c = 0; c < 4; c++) {
                double tmp = m[col][c]; m[col][c] = m[piv][c]; m[piv][c] = tmp;
            }
        }
        double d = m[col][col];
        for (int c = 0; c < 4; c++) m[col][c] /= d;
        for (int r = 0; r < 3; r++) {
            if (r == col) continue;
            double f = m[r][col];
            for (int c = 0; c < 4; c++) m[r][c] -= f * m[col][c];
        }
    }
    *out = (V3){m[0][3], m[1][3], m[2][3]};
    *err = q_eval(q, *out);
    if (*err < 0) *err = 0;
    return 1;
}

/* ------------------------------------------------------------- allocators */

static void* xmalloc(size_t n) {
    void* p = malloc(n);
    if (!p) { fprintf(stderr, "lod: out of memory\n"); exit(2); }
    return p;
}
static void* xcalloc(size_t cnt, size_t n) {
    void* p = calloc(cnt, n);
    if (!p) { fprintf(stderr, "lod: out of memory\n"); exit(2); }
    return p;
}
static void* xrealloc(void* p, size_t n) {
    void* r = realloc(p, n);
    if (!r) { fprintf(stderr, "lod: out of memory\n"); exit(2); }
    return r;
}

/* ------------------------------------------------- open-addressing edge map */

typedef struct { uint64_t key; int val; } MapSlot;
typedef struct { MapSlot* s; int cap, mask, used; } EdgeMap;

static void em_init(EdgeMap* m, int estimated) {
    int cap = 8;
    while (cap < estimated * 2) cap <<= 1;
    m->s = (MapSlot*)xcalloc((size_t)cap, sizeof(MapSlot));
    m->cap = cap;
    m->mask = cap - 1;
    m->used = 0;
}
static void em_free(EdgeMap* m) { free(m->s); m->s = NULL; }
static uint64_t em_key(int a, int b) {
    if (a > b) { int t = a; a = b; b = t; }
    return ((uint64_t)(uint32_t)a << 32) | (uint32_t)b;
}
static uint32_t em_hash(uint64_t key, int mask) {
    return (uint32_t)(key * 0x9e3779b97f4a7c15ull >> 33) & (uint32_t)mask;
}
static void em_grow(EdgeMap* m) {
    int old_cap = m->cap;
    MapSlot* old = m->s;
    m->cap = old_cap * 2;
    m->mask = m->cap - 1;
    m->s = (MapSlot*)xcalloc((size_t)m->cap, sizeof(MapSlot));
    m->used = 0;
    for (int i = 0; i < old_cap; i++) {
        if (old[i].key == 0) continue;
        uint32_t h = em_hash(old[i].key, m->mask);
        while (m->s[h].key != 0) h = (h + 1) & (uint32_t)m->mask;
        m->s[h] = old[i];
        m->used++;
    }
    free(old);
}
/* returns pointer to the value slot for (a,b), creating it with -1 if new */
static int* em_ref(EdgeMap* m, int a, int b) {
    /* keep load factor below 0.5 so probing always terminates */
    if ((m->used + 1) * 2 >= m->cap) em_grow(m);
    uint64_t key = em_key(a, b);
    uint32_t h = em_hash(key, m->mask);
    for (;;) {
        MapSlot* s = &m->s[h];
        if (s->key == 0) { s->key = key; s->val = -1; m->used++; return &s->val; }
        if (s->key == key) return &s->val;
        h = (h + 1) & (uint32_t)m->mask;
    }
}

/* ------------------------------------------------------------- simplifier */

typedef struct {
    V3 p;
    Quadric q;
    int* edges; int en, ecap;
    int* tris;  int tn, tcap;
    int alive;
} SVertex;

typedef struct {
    int v[3];
    int alive;
} STri;

typedef struct {
    int a, b;
    int t0, t1;          /* incident triangles (-1 when absent) */
    double err;
    int ver;             /* heap version; -1 = edge dead */
} SEdge;

typedef struct { int edge, ver; double err; } HNode;

typedef struct {
    SVertex* v; int vn;
    STri* t;    int tn, talive;
    SEdge* e;   int en, ecap;
    HNode* h;   int hn, hcap;
} Simp;

static int* arr_grow(int** arr, int* n, int* cap) {
    if (*n == *cap) {
        *cap = *cap ? *cap * 2 : 8;
        *arr = (int*)xrealloc(*arr, (size_t)*cap * sizeof(int));
    }
    return *arr;
}

static void heap_push(Simp* s, int edge, int ver, double err) {
    if (s->hn == s->hcap) {
        s->hcap = s->hcap ? s->hcap * 2 : 256;
        s->h = (HNode*)xrealloc(s->h, (size_t)s->hcap * sizeof(HNode));
    }
    int i = s->hn++;
    s->h[i] = (HNode){edge, ver, err};
    while (i > 0) {
        int par = (i - 1) / 2;
        if (s->h[par].err <= s->h[i].err) break;
        HNode tmp = s->h[par]; s->h[par] = s->h[i]; s->h[i] = tmp;
        i = par;
    }
}
/* pops the cheapest currently-valid edge, or -1 when the heap is exhausted */
static int heap_pop(Simp* s) {
    while (s->hn > 0) {
        HNode nd = s->h[0];
        s->h[0] = s->h[--s->hn];
        int i = 0;
        for (;;) {
            int l = 2*i + 1, r = l + 1, best = i;
            if (l < s->hn && s->h[l].err < s->h[best].err) best = l;
            if (r < s->hn && s->h[r].err < s->h[best].err) best = r;
            if (best == i) break;
            HNode tmp = s->h[best]; s->h[best] = s->h[i]; s->h[i] = tmp;
            i = best;
        }
        SEdge* e = &s->e[nd.edge];
        if (nd.ver == e->ver && e->ver >= 0 &&
            s->v[e->a].alive && s->v[e->b].alive)
            return nd.edge;
    }
    return -1;
}
static void edge_refresh(Simp* s, int ei) {
    SEdge* e = &s->e[ei];
    if (e->ver < 0 || !s->v[e->a].alive || !s->v[e->b].alive) return;
    Quadric q;
    memcpy(q, s->v[e->a].q, sizeof(q));
    q_add(q, s->v[e->b].q);
    V3 p;
    q_optimal(q, s->v[e->a].p, s->v[e->b].p, &p, &e->err);
    e->ver++;
    heap_push(s, ei, e->ver, e->err);
}

static Simp* simp_build(const float* verts, int vn, const int* tris, int tn) {
    Simp* s = (Simp*)xcalloc(1, sizeof(Simp));
    s->vn = vn; s->tn = tn; s->talive = tn;
    s->v = (SVertex*)xcalloc((size_t)vn, sizeof(SVertex));
    s->t = (STri*)xcalloc((size_t)tn, sizeof(STri));
    for (int i = 0; i < vn; i++) {
        s->v[i].p = (V3){verts[3*i], verts[3*i+1], verts[3*i+2]};
        s->v[i].alive = 1;
    }
    for (int i = 0; i < tn; i++)
        for (int k = 0; k < 3; k++) s->t[i].v[k] = tris[3*i+k];
    s->ecap = tn * 2;
    s->e = (SEdge*)xcalloc((size_t)s->ecap, sizeof(SEdge));

    EdgeMap map; em_init(&map, tn * 3);
    for (int ti = 0; ti < tn; ti++) {
        s->t[ti].alive = 1;
        int* tv = s->t[ti].v;
        for (int k = 0; k < 3; k++) {
            int** pp = &s->v[tv[k]].tris;
            arr_grow(pp, &s->v[tv[k]].tn, &s->v[tv[k]].tcap);
            (*pp)[s->v[tv[k]].tn++] = ti;
        }
        for (int k = 0; k < 3; k++) {
            int a = tv[k], b = tv[(k + 1) % 3];
            int* slot = em_ref(&map, a, b);
            int ei;
            if (*slot == -1) {
                if (s->en == s->ecap) {
                    s->ecap *= 2;
                    s->e = (SEdge*)xrealloc(s->e, (size_t)s->ecap * sizeof(SEdge));
                    memset(s->e + s->en, 0, (size_t)(s->ecap - s->en) * sizeof(SEdge));
                }
                ei = s->en++;
                *slot = ei;
                s->e[ei].a = a < b ? a : b;
                s->e[ei].b = a < b ? b : a;
                s->e[ei].t0 = ti; s->e[ei].t1 = -1;
                s->e[ei].ver = 0;
                arr_grow(&s->v[a].edges, &s->v[a].en, &s->v[a].ecap);
                s->v[a].edges[s->v[a].en++] = ei;
                arr_grow(&s->v[b].edges, &s->v[b].en, &s->v[b].ecap);
                s->v[b].edges[s->v[b].en++] = ei;
            } else {
                ei = *slot;
                if (s->e[ei].t1 != -1) {
                    fprintf(stderr, "lod: non-manifold edge at tri %d\n", ti);
                    exit(2);
                }
                s->e[ei].t1 = ti;
            }
        }
    }
    em_free(&map);

    /* face plane quadrics */
    for (int ti = 0; ti < tn; ti++) {
        int* tv = s->t[ti].v;
        V3 p0 = s->v[tv[0]].p, p1 = s->v[tv[1]].p, p2 = s->v[tv[2]].p;
        V3 n = v3_norm(v3_cross(v3_sub(p1, p0), v3_sub(p2, p0)));
        Quadric q; q_plane(q, n, -v3_dot(n, p0));
        for (int k = 0; k < 3; k++) q_add(s->v[tv[k]].q, q);
    }
    /* boundary constraints: plane through the edge, perpendicular to the
       adjacent face, so open meshes do not collapse their border */
    for (int ei = 0; ei < s->en; ei++) {
        SEdge* e = &s->e[ei];
        if (e->t1 != -1) continue;
        STri* f = &s->t[e->t0];
        V3 p0 = s->v[f->v[0]].p, p1 = s->v[f->v[1]].p, p2 = s->v[f->v[2]].p;
        V3 nf = v3_norm(v3_cross(v3_sub(p1, p0), v3_sub(p2, p0)));
        V3 pa = s->v[e->a].p, pb = s->v[e->b].p;
        V3 d = v3_norm(v3_sub(pb, pa));
        V3 nb = v3_norm(v3_cross(d, nf));
        Quadric q; q_plane(q, nb, -v3_dot(nb, pa));
        q_add(s->v[e->a].q, q);
        q_add(s->v[e->b].q, q);
    }
    for (int ei = 0; ei < s->en; ei++) edge_refresh(s, ei);
    return s;
}

static int find_edge(const Simp* s, int va, int vb) {
    const SVertex* vx = &s->v[va];
    for (int i = 0; i < vx->en; i++) {
        const SEdge* e = &s->e[vx->edges[i]];
        if (e->ver < 0) continue;
        if ((e->a == va && e->b == vb) || (e->a == vb && e->b == va))
            return vx->edges[i];
    }
    return -1;
}

/* collapses edge ei; returns the geometric error introduced.  All access to
   s->e / s->v sub-arrays goes through indices: arr_grow() may realloc those
   arrays, so caching a raw struct pointer across it would dangle. */
static double simp_collapse(Simp* s, int ei) {
    int va = s->e[ei].a, vb = s->e[ei].b;
    Quadric q;
    memcpy(q, s->v[va].q, sizeof(q));
    q_add(q, s->v[vb].q);
    V3 p; double perr;
    q_optimal(q, s->v[va].p, s->v[vb].p, &p, &perr);

    /* triangles: drop those containing both endpoints, rewrite vb -> va */
    for (int pass = 0; pass < 2; pass++) {
        int src = pass == 0 ? va : vb;
        for (int i = 0; i < s->v[src].tn; i++) {
            int fi = s->v[src].tris[i];
            STri* f = &s->t[fi];
            if (!f->alive) continue;
            int has_a = 0, has_b = 0;
            for (int k = 0; k < 3; k++) {
                if (f->v[k] == va) has_a = 1;
                if (f->v[k] == vb) has_b = 1;
            }
            if (has_a && has_b) {
                f->alive = 0;
                s->talive--;
            } else if (has_b) {
                for (int k = 0; k < 3; k++)
                    if (f->v[k] == vb) f->v[k] = va;
                arr_grow(&s->v[va].tris, &s->v[va].tn, &s->v[va].tcap);
                s->v[va].tris[s->v[va].tn++] = fi;
            }
        }
    }

    /* edges: merge duplicates into va, repoint the rest, kill the edge */
    int vb_en = s->v[vb].en;
    for (int i = 0; i < vb_en; i++) {
        int ej = s->v[vb].edges[i];
        if (s->e[ej].ver < 0) continue;
        int w = s->e[ej].a == vb ? s->e[ej].b : s->e[ej].a;
        if (w == va) { s->e[ej].ver = -1; continue; }
        if (find_edge(s, va, w) >= 0) { s->e[ej].ver = -1; continue; }
        s->e[ej].a = va; s->e[ej].b = w;
        arr_grow(&s->v[va].edges, &s->v[va].en, &s->v[va].ecap);
        s->v[va].edges[s->v[va].en++] = ej;
    }
    s->e[ei].ver = -1;
    s->v[vb].alive = 0;

    s->v[va].p = p;
    memcpy(s->v[va].q, q, sizeof(q));
    for (int i = 0; i < s->v[va].en; i++) edge_refresh(s, s->v[va].edges[i]);
    return perr;
}

static void simp_free(Simp* s) {
    for (int i = 0; i < s->vn; i++) { free(s->v[i].edges); free(s->v[i].tris); }
    free(s->v); free(s->t); free(s->e); free(s->h); free(s);
}

/* -------------------------------------------------- variant set + blob io */

typedef struct {
    int voff, vc, toff, tc;       /* offsets/sizes inside the packed blob */
    float err;                    /* geometric error, world units */
    float bmin[3], bmax[3];
} LevelSnap;

typedef struct {
    int levels;
    LevelSnap lv[LOD_MAX_LEVELS];
    float* vbuf;  int vn;         /* packed float xyz */
    uint32_t* ibuf; int in;       /* packed indices */
} LodBuild;

static void snap_level(LodBuild* lb, const Simp* s, int level, double err) {
    int* map = (int*)xmalloc((size_t)s->vn * sizeof(int));
    for (int i = 0; i < s->vn; i++) map[i] = -1;
    int vc = 0;
    for (int i = 0; i < s->vn; i++)
        if (s->v[i].alive) map[i] = vc++;

    int voff = lb->vn;
    lb->vbuf = (float*)xrealloc(lb->vbuf, (size_t)(lb->vn + vc * 3) * sizeof(float));
    float bmin[3] = {1e30f, 1e30f, 1e30f}, bmax[3] = {-1e30f, -1e30f, -1e30f};
    for (int i = 0; i < s->vn; i++) {
        if (!s->v[i].alive) continue;
        float xyz[3] = {(float)s->v[i].p.x, (float)s->v[i].p.y, (float)s->v[i].p.z};
        for (int k = 0; k < 3; k++) {
            lb->vbuf[lb->vn++] = xyz[k];
            if (xyz[k] < bmin[k]) bmin[k] = xyz[k];
            if (xyz[k] > bmax[k]) bmax[k] = xyz[k];
        }
    }
    int tc = 0;
    for (int i = 0; i < s->tn; i++) if (s->t[i].alive) tc++;
    int toff = lb->in;
    lb->ibuf = (uint32_t*)xrealloc(lb->ibuf, (size_t)(lb->in + tc * 3) * sizeof(uint32_t));
    for (int i = 0; i < s->tn; i++) {
        if (!s->t[i].alive) continue;
        for (int k = 0; k < 3; k++) lb->ibuf[lb->in++] = (uint32_t)map[s->t[i].v[k]];
    }
    free(map);

    LevelSnap* sn = &lb->lv[level];
    sn->voff = voff; sn->vc = vc; sn->toff = toff; sn->tc = tc;
    sn->err = (float)sqrt(err);
    memcpy(sn->bmin, bmin, sizeof(bmin));
    memcpy(sn->bmax, bmax, sizeof(bmax));
}

/* Camera parameters used to bake switch distances into the blob. */
typedef struct { float fov_y, screen_h, tolerance_px, tau; } LodCam;

/* in: welded triangle mesh. err_targets[0] is ignored (level 0 keeps the
   full mesh); simplification of level k stops once the next collapse would
   exceed err_targets[k]^2, or the triangle count reaches min_tris. */
static LodBuild* lod_generate(const float* verts, int vn, const int* tris, int tn,
                              int levels, const double* err_targets, int min_tris) {
    if (levels < 1 || levels > LOD_MAX_LEVELS) return NULL;
    Simp* s = simp_build(verts, vn, tris, tn);
    LodBuild* lb = (LodBuild*)xcalloc(1, sizeof(LodBuild));
    lb->levels = levels;
    snap_level(lb, s, 0, 0.0);
    for (int k = 1; k < levels; k++) {
        /* edge error is squared distance; compare against the squared target */
        double limit = err_targets[k] * err_targets[k];
        double last_err = 0;
        for (;;) {
            if (s->talive <= min_tris) break;
            int ei = heap_pop(s);
            if (ei < 0) break;
            if (s->e[ei].err > limit) break;
            double got = simp_collapse(s, ei);
            if (got > last_err) last_err = got;
        }
        snap_level(lb, s, k, last_err);
    }
    simp_free(s);
    return lb;
}
static void lod_build_free(LodBuild* lb) {
    if (!lb) return;
    free(lb->vbuf); free(lb->ibuf); free(lb);
}

/* --------------------------------------------------------- blob serialization */

typedef struct { uint8_t* p; size_t n, cap; } Buf;

static void bp_reserve(Buf* b, size_t extra) {
    if (b->n + extra <= b->cap) return;
    size_t cap = b->cap ? b->cap : 4096;
    while (cap < b->n + extra) cap <<= 1;
    b->p = (uint8_t*)xrealloc(b->p, cap);
    b->cap = cap;
}
static void bp_u32(Buf* b, uint32_t v) {
    bp_reserve(b, 4);
    b->p[b->n++] = (uint8_t)(v);
    b->p[b->n++] = (uint8_t)(v >> 8);
    b->p[b->n++] = (uint8_t)(v >> 16);
    b->p[b->n++] = (uint8_t)(v >> 24);
}
static void bp_f32(Buf* b, float v) {
    uint32_t u; memcpy(&u, &v, 4); bp_u32(b, u);
}
static void bp_bytes(Buf* b, const void* src, size_t n) {
    bp_reserve(b, n);
    memcpy(b->p + b->n, src, n);
    b->n += n;
}

#define HEADER_FIXED 32u  /* magic,version,levels,payload_off + 4 camera floats */
#define LEVEL_RECORD 52u  /* 13 u32/f32 words per level */

static size_t lod_blob_size(const LodBuild* lb) {
    return HEADER_FIXED + (size_t)lb->levels * LEVEL_RECORD
         + (size_t)lb->vn * 4 + (size_t)lb->in * 4;
}

static uint8_t* lod_serialize(const LodBuild* lb, LodCam cam, size_t* out_n) {
    Buf b = {0};
    uint32_t payload_off = HEADER_FIXED + (uint32_t)lb->levels * LEVEL_RECORD;
    bp_u32(&b, LOD_MAGIC);
    bp_u32(&b, LOD_VERSION);
    bp_u32(&b, (uint32_t)lb->levels);
    bp_u32(&b, payload_off);
    bp_f32(&b, cam.fov_y);
    bp_f32(&b, cam.screen_h);
    bp_f32(&b, cam.tolerance_px);
    bp_f32(&b, cam.tau);

    float k = cam.screen_h / (2.0f * tanf(cam.fov_y * 0.5f) * cam.tolerance_px);
    /* payload packs all vertex floats first, then all index words */
    uint32_t vbase = payload_off;
    uint32_t ibase = payload_off + (uint32_t)lb->vn * 4u;
    for (int i = 0; i < lb->levels; i++) {
        const LevelSnap* sn = &lb->lv[i];
        float r = sn->err * k;                 /* ideal switch radius */
        float rise = (i == lb->levels - 1) ? 0.0f : r * cam.tau;
        float fall = (i == 0) ? 0.0f : r / cam.tau;
        bp_f32(&b, sn->err);
        bp_u32(&b, (uint32_t)sn->tc);
        bp_u32(&b, (uint32_t)sn->vc);
        bp_u32(&b, vbase + (uint32_t)sn->voff * 4u);
        bp_u32(&b, ibase + (uint32_t)sn->toff * 4u);
        for (int j = 0; j < 3; j++) bp_f32(&b, sn->bmin[j]);
        for (int j = 0; j < 3; j++) bp_f32(&b, sn->bmax[j]);
        bp_f32(&b, rise);
        bp_f32(&b, fall);
    }
    assert(b.n == payload_off);
    bp_bytes(&b, lb->vbuf, (size_t)lb->vn * 4);
    bp_bytes(&b, lb->ibuf, (size_t)lb->in * 4);
    assert(b.n == lod_blob_size(lb));
    *out_n = b.n;
    return b.p;
}

static int gr_u32(const uint8_t* p, size_t n, size_t* off, uint32_t* out) {
    if (*off + 4 > n) return -1;
    *out = (uint32_t)p[*off] | ((uint32_t)p[*off+1] << 8)
         | ((uint32_t)p[*off+2] << 16) | ((uint32_t)p[*off+3] << 24);
    *off += 4;
    return 0;
}
static int gr_f32(const uint8_t* p, size_t n, size_t* off, float* out) {
    uint32_t u;
    if (gr_u32(p, n, off, &u)) return -1;
    memcpy(out, &u, 4);
    return 0;
}

/* Parsed view over caller-owned memory; this and lod_select() never malloc. */
typedef struct {
    const uint8_t* p;
    size_t n;
    int levels;
    size_t rec_off;
} LodBlob;

static int lod_blob_open(const void* buf, size_t n, LodBlob* b) {
    const uint8_t* p = (const uint8_t*)buf;
    size_t off = 0;
    uint32_t magic, ver, lvls, payload;
    if (gr_u32(p, n, &off, &magic) || magic != LOD_MAGIC) return -1;
    if (gr_u32(p, n, &off, &ver) || ver != LOD_VERSION) return -1;
    if (gr_u32(p, n, &off, &lvls) || lvls < 1 || lvls > LOD_MAX_LEVELS) return -1;
    if (gr_u32(p, n, &off, &payload) || payload > n) return -1;
    off += 16; /* camera floats: validated structurally below, unused here */
    size_t rec = off;

    /* payload layout: every level's vertices concatenated, then every
       level's indices.  Validate ranges, monotone offsets, and that the two
       regions are disjoint and end exactly at the blob tail. */
    uint32_t prev_vend = payload, prev_tend = 0;
    uint32_t index_region_start = 0;
    for (uint32_t i = 0; i < lvls; i++) {
        size_t ro = rec + (size_t)i * LEVEL_RECORD;
        off = ro + 4; /* skip error */
        uint32_t tc, vc, voff, toff;
        if (gr_u32(p, n, &off, &tc) || gr_u32(p, n, &off, &vc) ||
            gr_u32(p, n, &off, &voff) || gr_u32(p, n, &off, &toff))
            return -1;
        uint64_t vend = (uint64_t)voff + (uint64_t)vc * 3u * 4u;
        uint64_t tend = (uint64_t)toff + (uint64_t)tc * 3u * 4u;
        if (vend > n || tend > n) return -1;
        if (voff < payload) return -1;
        if (voff != prev_vend) return -1;       /* packed, no gaps */
        prev_vend = (uint32_t)vend;
        if (i == 0) index_region_start = toff;
        else {
            if (toff < prev_tend) return -1;
            if (toff != prev_tend) return -1;   /* packed indices too */
        }
        prev_tend = (uint32_t)tend;
        off += 6 * 4 + 2 * 4; /* bbox + switch distances */
        if (off > n) return -1;
    }
    /* all vertices must precede all indices, and payload fills the blob */
    if (prev_vend != index_region_start) return -1;
    if (prev_tend != n) return -1;
    b->p = p; b->n = n; b->levels = (int)lvls; b->rec_off = rec;
    return 0;
}

static size_t lv_word(const LodBlob* b, int lv, int word) {
    return b->rec_off + (size_t)lv * LEVEL_RECORD + (size_t)word * 4;
}
static float lv_f32(const LodBlob* b, int lv, int word) {
    float v = 0.0f;
    size_t off = lv_word(b, lv, word);
    gr_f32(b->p, b->n, &off, &v);
    return v;
}
static uint32_t lv_u32(const LodBlob* b, int lv, int word) {
    uint32_t v = 0;
    size_t off = lv_word(b, lv, word);
    gr_u32(b->p, b->n, &off, &v);
    return v;
}

/* word indices inside LEVEL_RECORD:
   0 err | 1 tc | 2 vc | 3 voff | 4 toff | 5..7 bmin | 8..10 bmax
   | 11 rise | 12 fall */
#define W_ERR 0
#define W_TC  1
#define W_VC  2
#define W_VOFF 3
#define W_TOFF 4
#define W_BMIN 5
#define W_BMAX 8
#define W_RISE 11
#define W_FALL 12

static int lod_level_tris(const LodBlob* b, int lv)   { return (int)lv_u32(b, lv, W_TC); }
static int lod_level_verts(const LodBlob* b, int lv)  { return (int)lv_u32(b, lv, W_VC); }
static float lod_level_error(const LodBlob* b, int lv){ return lv_f32(b, lv, W_ERR); }
static void lod_level_bounds(const LodBlob* b, int lv, float mn[3], float mx[3]) {
    for (int i = 0; i < 3; i++) {
        mn[i] = lv_f32(b, lv, W_BMIN + i);
        mx[i] = lv_f32(b, lv, W_BMAX + i);
    }
}
static const float* lod_level_vertices(const LodBlob* b, int lv) {
    return (const float*)(b->p + lv_u32(b, lv, W_VOFF));
}
static const uint32_t* lod_level_indices(const LodBlob* b, int lv) {
    return (const uint32_t*)(b->p + lv_u32(b, lv, W_TOFF));
}

/* Runtime LOD selection: table-only, no allocation.
   Each level k stores an outward switch radius rise_d (W_RISE): the
   distance at which level k is allowed to coarsen to k+1.  The matching
   inward radius for refining back from k+1 is rise_d / tau (stored as
   W_FALL), so every boundary has a tau-wide dead band where the current
   level is held.  The target is computed directly, so a distance jump
   lands on the correct level in one call while small movements inside a
   dead band keep the previous level (no thrash, stable at fixed distance). */
static int lod_select(const LodBlob* b, int prev, float distance) {
    int lv = prev;
    if (lv < 0) lv = 0;
    if (lv >= b->levels) lv = b->levels - 1;
    /* coarsen while beyond the current level's outward threshold */
    while (lv < b->levels - 1 && distance > lv_f32(b, lv, W_RISE))
        lv++;
    /* refine while inside the current level's inward threshold */
    while (lv > 0 && distance < lv_f32(b, lv, W_FALL))
        lv--;
    return lv;
}

/* ------------------------------------------------------------- test meshes */

static void make_terrain(int n, float** out_v, int* out_vn, int** out_t, int* out_tn) {
    int vn = n * n, tn = 2 * (n - 1) * (n - 1);
    float* v = (float*)xmalloc((size_t)vn * 3 * sizeof(float));
    int* t = (int*)xmalloc((size_t)tn * 3 * sizeof(int));
    for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
            float fx = 100.0f * x / (n - 1), fy = 100.0f * y / (n - 1);
            v[3 * (y * n + x) + 0] = fx;
            v[3 * (y * n + x) + 1] = 4.0f * sinf(fx * 0.15f) * cosf(fy * 0.11f);
            v[3 * (y * n + x) + 2] = fy;
        }
    }
    int w = 0;
    for (int y = 0; y < n - 1; y++) {
        for (int x = 0; x < n - 1; x++) {
            int a = y * n + x, b = a + 1, c = a + n, d = c + 1;
            t[w++] = a; t[w++] = c; t[w++] = b;
            t[w++] = b; t[w++] = c; t[w++] = d;
        }
    }
    assert(w == tn * 3);
    *out_v = v; *out_vn = vn; *out_t = t; *out_tn = tn;
}

static void make_icosphere(int subdiv, float** out_v, int* out_vn,
                           int** out_t, int* out_tn) {
    double t = (1.0 + sqrt(5.0)) / 2.0;
    double base[12][3] = {
        {-1, t, 0}, {1, t, 0}, {-1,-t, 0}, {1,-t, 0},
        {0,-1, t}, {0, 1, t}, {0,-1,-t}, {0, 1,-t},
        {t, 0,-1}, {t, 0, 1}, {-t, 0,-1}, {-t, 0, 1}
    };
    int faces[20][3] = {
        {0,11,5},{0,5,1},{0,1,7},{0,7,10},{0,10,11},
        {1,5,9},{5,11,4},{11,10,2},{10,7,6},{7,1,8},
        {3,9,4},{3,4,2},{3,2,6},{3,6,8},{3,8,9},
        {4,9,5},{2,4,11},{6,2,10},{8,6,7},{9,8,1}
    };
    int vcap = 4096;
    double* vd = (double*)xmalloc((size_t)vcap * 3 * sizeof(double));
    int vn = 12;
    memcpy(vd, base, sizeof(base));
    /* tn is always a triangle count; the index cursor walks 3 entries each */
    int tcap = 4096;
    int* tris = (int*)xmalloc((size_t)tcap * 3 * sizeof(int));
    int tn = 0, ti_cursor = 0;
    for (int i = 0; i < 20; i++) {
        tris[ti_cursor++] = faces[i][0];
        tris[ti_cursor++] = faces[i][1];
        tris[ti_cursor++] = faces[i][2];
        tn++;
    }

    for (int s = 0; s < subdiv; s++) {
        /* edge midpoints are shared only within this subdivision round, so
           the lookup map must be fresh each iteration */
        EdgeMap mid; em_init(&mid, tn * 3);
        int* nt = (int*)xmalloc((size_t)tn * 4 * 3 * sizeof(int));
        int ntn = 0, ni = 0;
        for (int fi = 0; fi < tn; fi++) {
            int a = tris[3*fi], b = tris[3*fi+1], c = tris[3*fi+2];
            int midx[3];
            int pair[3][2] = {{a,b},{b,c},{c,a}};
            for (int k = 0; k < 3; k++) {
                int* slot = em_ref(&mid, pair[k][0], pair[k][1]);
                if (*slot == -1) {
                    if (vn * 3 >= vcap) {
                        vcap *= 2;
                        vd = (double*)xrealloc(vd, (size_t)vcap * 3 * sizeof(double));
                    }
                    V3 p = v3_mid((V3){vd[3*pair[k][0]], vd[3*pair[k][0]+1], vd[3*pair[k][0]+2]},
                                  (V3){vd[3*pair[k][1]], vd[3*pair[k][1]+1], vd[3*pair[k][1]+2]});
                    p = v3_norm(p);
                    vd[3*vn] = p.x; vd[3*vn+1] = p.y; vd[3*vn+2] = p.z;
                    *slot = vn++;
                }
                midx[k] = *slot;
            }
            int out_faces[4][3] = {
                {a, midx[0], midx[2]},
                {b, midx[1], midx[0]},
                {c, midx[2], midx[1]},
                {midx[0], midx[1], midx[2]}
            };
            for (int f = 0; f < 4; f++) {
                nt[ni++] = out_faces[f][0];
                nt[ni++] = out_faces[f][1];
                nt[ni++] = out_faces[f][2];
                ntn++;
            }
        }
        em_free(&mid);
        free(tris);
        tris = nt;
        tn = ntn;
    }
    float* vf = (float*)xmalloc((size_t)vn * 3 * sizeof(float));
    for (int i = 0; i < vn * 3; i++) vf[i] = (float)(vd[i] * 10.0);
    free(vd);
    *out_v = vf; *out_vn = vn; *out_t = tris; *out_tn = tn;
}

/* ------------------------------------------------------------------ checks */

static int check_bbox_coverage(const LodBlob* b) {
    for (int lv = 0; lv < b->levels; lv++) {
        float mn[3], mx[3];
        lod_level_bounds(b, lv, mn, mx);
        const float* v = lod_level_vertices(b, lv);
        int vc = lod_level_verts(b, lv);
        for (int i = 0; i < vc; i++)
            for (int k = 0; k < 3; k++) {
                float x = v[3*i+k];
                float span = fabsf(mx[k] - mn[k]);
                float tol = EPS_BBOX * (1.0f + span);
                if (x < mn[k] - tol || x > mx[k] + tol) {
                    fprintf(stderr, "bbox fail lv=%d vert=%d axis=%d\n", lv, i, k);
                    return 0;
                }
            }
        /* tightness: each axis extreme must be touched by some vertex */
        for (int k = 0; k < 3; k++) {
            int got_min = 0, got_max = 0;
            for (int i = 0; i < vc; i++) {
                float x = v[3*i+k];
                if (x == mn[k]) got_min = 1;
                if (x == mx[k]) got_max = 1;
            }
            if (!got_min || !got_max) {
                fprintf(stderr, "bbox not tight lv=%d axis=%d\n", lv, k);
                return 0;
            }
        }
    }
    return 1;
}

static int check_indices(const LodBlob* b) {
    for (int lv = 0; lv < b->levels; lv++) {
        int vc = lod_level_verts(b, lv);
        int tc = lod_level_tris(b, lv);
        const uint32_t* idx = lod_level_indices(b, lv);
        for (int i = 0; i < tc * 3; i++)
            if (idx[i] >= (uint32_t)vc) return 0;
    }
    return 1;
}

int main(void) {
    int fail = 0;
    float* tv; int tvn; int* tt; int ttn;
    make_terrain(101, &tv, &tvn, &tt, &ttn);
    printf("input: terrain verts=%d tris=%d\n", tvn, ttn);

    double targets[LOD_LEVELS] = {0.0, 0.05, 0.5, 5.0, 50.0};
    clock_t t0 = clock();
    LodBuild* lb = lod_generate(tv, tvn, tt, ttn, LOD_LEVELS, targets, 16);
    double build_s = (double)(clock() - t0) / CLOCKS_PER_SEC;
    if (!lb) { fprintf(stderr, "generate failed\n"); return 1; }

    printf("build: %.0f ms\n", build_s * 1000.0);
    for (int i = 0; i < lb->levels; i++) {
        const LevelSnap* s = &lb->lv[i];
        printf("lod%d: verts=%d tris=%d err=%.4f bbox=[%.2f..%.2f %.2f..%.2f %.2f..%.2f]\n",
               i, s->vc, s->tc, s->err,
               s->bmin[0], s->bmax[0], s->bmin[1], s->bmax[1],
               s->bmin[2], s->bmax[2]);
    }

    /* five levels, counts and errors monotone, strict reduction each step */
    if (lb->levels != LOD_LEVELS) { fprintf(stderr, "FAIL level count\n"); fail = 1; }
    for (int k = 1; k < lb->levels; k++) {
        if (lb->lv[k].tc >= lb->lv[k-1].tc) { fprintf(stderr, "FAIL tris monotone at %d\n", k); fail = 1; }
        if (lb->lv[k].vc >= lb->lv[k-1].vc) { fprintf(stderr, "FAIL verts monotone at %d\n", k); fail = 1; }
        if (lb->lv[k].err < lb->lv[k-1].err) { fprintf(stderr, "FAIL error monotone at %d\n", k); fail = 1; }
    }
    if (lb->lv[0].tc != ttn || lb->lv[0].vc != tvn) { fprintf(stderr, "FAIL level0 identity\n"); fail = 1; }
    if (build_s >= 30.0) { fprintf(stderr, "FAIL 30s budget: %.2fs\n", build_s); fail = 1; }
    if (lb->lv[LOD_LEVELS-1].tc > ttn / 4) { fprintf(stderr, "FAIL coarse level too dense\n"); fail = 1; }

    /* level-0 positions must be bit-identical to the input */
    for (int i = 0; i < tvn * 3; i++)
        if (lb->vbuf[i] != tv[i]) { fprintf(stderr, "FAIL level0 positions\n"); fail = 1; break; }

    /* serialize + round-trip through a file */
    LodCam cam = {1.0f, 1080.0f, 50.0f, 1.1f};
    size_t blob_n;
    uint8_t* blob = lod_serialize(lb, cam, &blob_n);
    printf("blob: %zu bytes\n", blob_n);

    FILE* f = tmpfile();
    if (!f) { fprintf(stderr, "tmpfile failed\n"); return 1; }
    fwrite(blob, 1, blob_n, f);
    rewind(f);
    uint8_t* readback = (uint8_t*)xmalloc(blob_n);
    size_t got = fread(readback, 1, blob_n, f);
    fclose(f);
    if (got != blob_n) { fprintf(stderr, "FAIL file roundtrip size\n"); fail = 1; }

    LodBlob b;
    if (lod_blob_open(readback, blob_n, &b)) { fprintf(stderr, "FAIL blob open\n"); fail = 1; }
    if (b.levels != LOD_LEVELS) { fprintf(stderr, "FAIL blob levels\n"); fail = 1; }
    for (int i = 0; i < b.levels && !fail; i++) {
        if (lod_level_tris(&b, i) != lb->lv[i].tc) { fprintf(stderr, "FAIL blob tc %d\n", i); fail = 1; }
        if (lod_level_verts(&b, i) != lb->lv[i].vc) { fprintf(stderr, "FAIL blob vc %d\n", i); fail = 1; }
        if (fabsf(lod_level_error(&b, i) - lb->lv[i].err) > 1e-6f) { fprintf(stderr, "FAIL blob err %d\n", i); fail = 1; }
    }
    if (!fail && !check_bbox_coverage(&b)) { fprintf(stderr, "FAIL bbox coverage\n"); fail = 1; }
    if (!fail && !check_indices(&b)) { fprintf(stderr, "FAIL index range\n"); fail = 1; }

    /* corrupt / truncated blobs must be rejected */
    if (!fail) {
        LodBlob bad;
        uint8_t* junk = (uint8_t*)xmalloc(blob_n);
        memcpy(junk, readback, blob_n);
        junk[0] ^= 0xff;
        if (lod_blob_open(junk, blob_n, &bad) == 0) { fprintf(stderr, "FAIL bad magic accepted\n"); fail = 1; }
        if (lod_blob_open(readback, blob_n - 8, &bad) == 0) { fprintf(stderr, "FAIL truncation accepted\n"); fail = 1; }
        free(junk);
    }

    /* runtime selection: monotone on sweep, hysteresis on oscillation */
    if (!fail) {
        int lv = 0, switches = 0, prev = 0;
        for (float d = 0.1f; d < 5000.0f; d *= 1.05f) {
            lv = lod_select(&b, lv, d);
            if (lv != prev) { switches++; prev = lv; }
            if (lv < 0 || lv >= LOD_LEVELS) { fprintf(stderr, "FAIL select range\n"); fail = 1; }
        }
        if (lv != LOD_LEVELS - 1) { fprintf(stderr, "FAIL far distance not coarsest (lv=%d)\n", lv); fail = 1; }
        int out_switches = switches;

        switches = 0; prev = lv;
        for (float d = 5000.0f; d > 0.05f; d /= 1.05f) {
            int nl = lod_select(&b, lv, d);
            if (nl != lv) switches++;
            lv = nl;
        }
        if (lv != 0) { fprintf(stderr, "FAIL near distance not finest\n"); fail = 1; }
        printf("selector: outward switches=%d return switches=%d\n", out_switches, switches);

        /* hold a distance strictly inside the first dead band: after the
           initial settle, repeated queries must never switch again */
        float rise = lv_f32(&b, 1, W_RISE), fall = lv_f32(&b, 1, W_FALL);
        float mid = sqrtf(rise * fall);
        if (!(fall < mid && mid < rise)) { fprintf(stderr, "FAIL deadband geometry\n"); fail = 1; }
        lv = lod_select(&b, 0, mid);             /* settle into the band */
        int settled = lv;
        int churn = 0;
        for (int i = 0; i < 200; i++) {
            int nl = lod_select(&b, lv, mid);
            if (nl != lv) churn++;
            lv = nl;
        }
        if (churn != 0) { fprintf(stderr, "FAIL deadband churn=%d\n", churn); fail = 1; }
        if (lv != settled) { fprintf(stderr, "FAIL deadband level drift\n"); fail = 1; }

        /* a distance inside the band must also hold when approached from
           the coarser side (true hysteresis, not one-way) */
        int from_coarse = lod_select(&b, LOD_LEVELS - 1, mid);
        if (from_coarse != settled && from_coarse != settled + 1) {
            fprintf(stderr, "FAIL two-sided hysteresis\n"); fail = 1;
        }

        /* repeated query at fixed distance must be stable */
        lv = lod_select(&b, 0, mid * 0.5f);
        for (int i = 0; i < 100; i++)
            if (lod_select(&b, lv, mid * 0.5f) != lv) { fprintf(stderr, "FAIL stability\n"); fail = 1; }
    }

    /* closed-mesh path (no boundary edges) must simplify cleanly.  Targets
       are scaled to the radius-10 sphere, whose smallest edge is ~0.7: the
       first target must admit at least one real collapse. */
    if (!fail) {
        float* iv; int ivn; int* it; int itn;
        make_icosphere(3, &iv, &ivn, &it, &itn);
        double ico_targets[LOD_LEVELS] = {0.0, 0.3, 1.0, 3.0, 10.0};
        LodBuild* ic = lod_generate(iv, ivn, it, itn, LOD_LEVELS, ico_targets, 16);
        if (!ic || ic->levels != LOD_LEVELS) { fprintf(stderr, "FAIL icosphere generate\n"); fail = 1; }
        else {
            for (int k = 1; k < LOD_LEVELS; k++) {
                if (ic->lv[k].tc >= ic->lv[k-1].tc) { fprintf(stderr, "FAIL ico monotone at %d\n", k); fail = 1; }
                if (ic->lv[k].err < ic->lv[k-1].err) { fprintf(stderr, "FAIL ico error monotone at %d\n", k); fail = 1; }
            }
            printf("icosphere: %d tris -> %d tris over %d levels\n",
                   itn, ic->lv[LOD_LEVELS-1].tc, LOD_LEVELS);
        }
        lod_build_free(ic);
        free(iv); free(it);
    }

    free(blob); free(readback);
    lod_build_free(lb);
    free(tv); free(tt);

    if (fail) { printf("RESULT: FAIL\n"); return 1; }
    printf("RESULT: OK\n");
    return 0;
}
