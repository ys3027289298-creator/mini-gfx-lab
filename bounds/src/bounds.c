
/* bounds: incremental AABB updates over a scene tree.

   A changed transform dirties its own subtree (world matrices must be
   rebuilt) and marks ancestors child-dirty (they only re-merge).  Nothing
   recomputes until bounds are queried, and a query only descends into
   dirty subtrees.  A parent's box is always rebuilt from its children's
   current boxes -- never unioned with its own previous box -- so boxes
   shrink again when children move inward.  detect_stale() compares the
   cache against a from-scratch recompute and flags nodes whose cached box
   strictly contains the true box: the "grow-only" failure mode. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAXN 256
#define MAXK 8
#define FRAMES 40

typedef struct { float min[3], max[3]; } AABB;

typedef struct {
    int parent;
    int child[MAXK];
    int nchild;
    float local[16];          /* row-major 4x4 */
    float world[16];
    AABB mesh;                /* local-space mesh box; empty when no geometry */
    AABB bounds;              /* cached world-space subtree box */
    unsigned char dirty_self; /* own transform changed: rebuild world matrix */
    unsigned char dirty_child;/* a descendant changed: re-merge only */
} Node;

typedef struct {
    Node n[MAXN];
    int count;
    int grow_only;     /* buggy merge mode, exercises the stale detector */
    long recomputed;   /* nodes recomputed by the last update */
} Scene;

/* ---- AABB helpers ---- */

static AABB aabb_empty(void) {
    AABB b;
    for (int k = 0; k < 3; k++) { b.min[k] = HUGE_VALF; b.max[k] = -HUGE_VALF; }
    return b;
}

static int aabb_is_empty(AABB b) { return b.min[0] > b.max[0]; }

static AABB aabb_union(AABB a, AABB b) {
    AABB r;
    for (int k = 0; k < 3; k++) {
        r.min[k] = a.min[k] < b.min[k] ? a.min[k] : b.min[k];
        r.max[k] = a.max[k] > b.max[k] ? a.max[k] : b.max[k];
    }
    return r;
}

static int aabb_equal(AABB a, AABB b) {
    for (int k = 0; k < 3; k++)
        if (a.min[k] != b.min[k] || a.max[k] != b.max[k]) return 0;
    return 1;
}

static int aabb_contains(AABB outer, AABB inner) { /* outer superset of inner */
    for (int k = 0; k < 3; k++)
        if (outer.min[k] > inner.min[k] || outer.max[k] < inner.max[k]) return 0;
    return 1;
}

static AABB aabb_transform(const float m[16], AABB b) {
    if (aabb_is_empty(b)) return b;
    AABB r = aabb_empty();
    for (int i = 0; i < 8; i++) {
        float p[3] = { (i & 1) ? b.max[0] : b.min[0],
                       (i & 2) ? b.max[1] : b.min[1],
                       (i & 4) ? b.max[2] : b.min[2] };
        for (int k = 0; k < 3; k++) {
            float v = m[k*4+0] * p[0] + m[k*4+1] * p[1] + m[k*4+2] * p[2] + m[k*4+3];
            if (v < r.min[k]) r.min[k] = v;
            if (v > r.max[k]) r.max[k] = v;
        }
    }
    return r;
}

/* ---- matrices ---- */

static void mat_identity(float m[16]) {
    memset(m, 0, 16 * sizeof(float));
    m[0] = m[5] = m[10] = m[15] = 1.0f;
}

static void mat_mul(float out[16], const float a[16], const float b[16]) {
    float t[16];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            t[i*4+j] = a[i*4+0]*b[0*4+j] + a[i*4+1]*b[1*4+j]
                     + a[i*4+2]*b[2*4+j] + a[i*4+3]*b[3*4+j];
    memcpy(out, t, sizeof t);
}

static void mat_trs(float m[16], float tx, float ty, float tz, float ry, float s) {
    float c = cosf(ry) * s, sn = sinf(ry) * s;
    float r[16] = { c, 0, -sn, tx,
                    0, s, 0,   ty,
                    sn, 0, c,  tz,
                    0, 0, 0,   1 };
    memcpy(m, r, sizeof r);
}

/* ---- scene ---- */

static int add_node(Scene* sc, int parent) {
    int id = sc->count++;
    Node* n = &sc->n[id];
    memset(n, 0, sizeof *n);
    n->parent = parent;
    mat_identity(n->local);
    n->mesh = aabb_empty();
    n->bounds = aabb_empty();
    n->dirty_self = 1;
    if (parent >= 0) {
        Node* p = &sc->n[parent];
        p->child[p->nchild++] = id;
    }
    return id;
}

/* invariant: dirty_self implies the whole subtree is dirty_self */
static void mark_subtree(Scene* sc, int id) {
    Node* n = &sc->n[id];
    if (n->dirty_self) return;
    n->dirty_self = 1;
    for (int i = 0; i < n->nchild; i++) mark_subtree(sc, n->child[i]);
}

static void set_transform(Scene* sc, int id, float tx, float ty, float tz,
                          float ry, float s) {
    mat_trs(sc->n[id].local, tx, ty, tz, ry, s);
    mark_subtree(sc, id);
    /* invariant: dirty_child implies all ancestors are dirty_child */
    for (int p = sc->n[id].parent; p >= 0 && !sc->n[p].dirty_child;
         p = sc->n[p].parent)
        sc->n[p].dirty_child = 1;
}

/* pull-based update: recompute only dirty nodes, top-down from the query */
static void update_node(Scene* sc, int id) {
    Node* n = &sc->n[id];
    if (!n->dirty_self && !n->dirty_child) return;
    sc->recomputed++;
    if (n->dirty_self) {
        if (n->parent < 0) memcpy(n->world, n->local, sizeof n->world);
        else mat_mul(n->world, sc->n[n->parent].world, n->local);
    }
    AABB b = aabb_transform(n->world, n->mesh);
    for (int i = 0; i < n->nchild; i++) {
        update_node(sc, n->child[i]);
        b = aabb_union(b, sc->n[n->child[i]].bounds);
    }
    if (sc->grow_only) b = aabb_union(b, n->bounds); /* the bug: never shrinks */
    n->bounds = b;
    n->dirty_self = n->dirty_child = 0;
}

static AABB query_bounds(Scene* sc, int root) {
    sc->recomputed = 0;
    update_node(sc, root);
    return sc->n[root].bounds;
}

/* from-scratch recompute of every node; returns nodes touched */
static long full_recompute(Scene* sc) {
    for (int i = 0; i < sc->count; i++) sc->n[i].dirty_self = 1;
    sc->recomputed = 0;
    update_node(sc, 0);
    return sc->recomputed;
}

/* count nodes whose cached box strictly contains the true box (grow-only) */
static int detect_stale(const Scene* sc) {
    static Scene fresh;
    fresh = *sc;
    fresh.grow_only = 0;
    full_recompute(&fresh);
    int stale = 0;
    for (int i = 0; i < sc->count; i++)
        if (aabb_contains(sc->n[i].bounds, fresh.n[i].bounds) &&
            !aabb_equal(sc->n[i].bounds, fresh.n[i].bounds))
            stale++;
    return stale;
}

/* ---- test scene ---- */

static void build(Scene* sc, int* dyn, int* ndyn) {
    memset(sc, 0, sizeof *sc);
    int root = add_node(sc, -1);
    set_transform(sc, root, 0, 0, 0, 0, 1);
    int objects[36], parts[144], no = 0, np = 0;
    for (int g = 0; g < 6; g++) {
        int grp = add_node(sc, root);
        float a = (float)g * 1.0472f;
        set_transform(sc, grp, 8.0f * cosf(a), 0, 8.0f * sinf(a), a, 1);
        for (int o = 0; o < 6; o++) {
            int obj = add_node(sc, grp);
            objects[no++] = obj;
            set_transform(sc, obj, 2.0f * o - 5.0f, 0.5f * o, 0, 0.3f * o, 1);
            for (int p = 0; p < 4; p++) {
                int part = add_node(sc, obj);
                parts[np++] = part;
                set_transform(sc, part, 0.4f * p, 0, 0.3f * p, 0, 1);
                sc->n[part].mesh = (AABB){ {-0.5f, -0.5f, -0.5f},
                                           { 0.5f,  0.5f,  0.5f} };
            }
        }
    }
    /* mix of interior nodes (whole subtree moves) and leaves */
    dyn[0] = objects[8];  dyn[1] = objects[29];
    dyn[2] = parts[1];    dyn[3] = parts[70];  dyn[4] = parts[140];
    *ndyn = 5;
}

static void move_dynamic(Scene* sc, const int* dyn, int ndyn, int f) {
    for (int i = 0; i < ndyn; i++) {
        float t = 0.25f * f;
        float amp = 1.5f + 0.5f * i;
        /* oscillation makes boxes grow AND shrink over the run */
        set_transform(sc, dyn[i], amp * sinf(t + (float)i), 0.2f * i,
                      amp * cosf(0.7f * t), 0.1f * f, 1.0f);
    }
}

int main(void) {
    static Scene sc;
    int dyn[8], ndyn;
    build(&sc, dyn, &ndyn);
    printf("nodes=%d frames=%d dynamic=%d\n", sc.count, FRAMES, ndyn);

    /* lazy: dirty marks alone must not recompute anything */
    move_dynamic(&sc, dyn, ndyn, 0);
    int lazy_ok = (sc.recomputed == 0);
    query_bounds(&sc, 0);

    long inc_total = 0, full_total = 0;
    int mismatch = 0;
    for (int f = 0; f < FRAMES; f++) {
        move_dynamic(&sc, dyn, ndyn, f);
        query_bounds(&sc, 0);
        inc_total += sc.recomputed;
        static Scene shadow;
        shadow = sc;
        full_total += full_recompute(&shadow);
        for (int i = 0; i < sc.count; i++)
            if (!aabb_equal(sc.n[i].bounds, shadow.n[i].bounds)) mismatch++;
        if (f < 6)
            printf("frame=%d updated=%ld full=%d\n", f, sc.recomputed, sc.count);
    }
    printf("avg_updated=%.1f avg_full=%.1f\n",
           (double)inc_total / FRAMES, (double)full_total / FRAMES);
    printf("match_full=%d\n", mismatch == 0);
    printf("lazy_ok=%d\n", lazy_ok);
    printf("stale_correct=%d\n", detect_stale(&sc));

    /* same run on a grow-only cache: boxes only ever expand */
    static Scene buggy;
    build(&buggy, dyn, &ndyn);
    buggy.grow_only = 1;
    query_bounds(&buggy, 0);
    for (int f = 0; f < FRAMES; f++) {
        move_dynamic(&buggy, dyn, ndyn, f);
        query_bounds(&buggy, 0);
    }
    int stale = detect_stale(&buggy);
    printf("growonly_stale=%d detected=%d\n", stale, stale > 0);

    int ok = mismatch == 0 && lazy_ok && detect_stale(&sc) == 0 && stale > 0;
    printf("ok=%d\n", ok);
    return !ok;
}
