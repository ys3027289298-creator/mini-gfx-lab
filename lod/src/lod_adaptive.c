/* lod-adaptive: screen-space vertex-density LOD for a closed mesh.

   Simplification strength comes from the projected vertex spacing: an edge
   shorter than TARGET_PX on screen is a collapse candidate, so a distant mesh
   is coarsened much more aggressively than a near one.

   Silhouette edges of the current view are found from the front/back facing
   transition.  Their vertices and the surrounding one-ring are locked, which
   leaves the silhouette polyline bit-identical through simplification.  A
   collapse is additionally rejected if it would push any triangle across the
   view silhouette plane or rotate its normal too far: such slivers flip their
   facing back and forth and carve phantom edges inside the surface.

   A job spreads edge collapses over frames with a fixed per-frame budget and
   writes into a back buffer.  The front buffer keeps being drawn until the job
   completes and is swapped at a frame boundary. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>

#define MAX_V 4096
#define MAX_T 8192
#define MAX_E (MAX_T * 3)
#define MAX_SIL 8192
#define MAX_MID 8192
#define VT_CAP 24

#define TARGET_PX 8.0f          /* desired projected vertex spacing */
#define STREAM_BUDGET 24        /* collapse attempts per frame */
#define MAX_FRAMES 2000
#define MIN_QUALITY 0.12f       /* |2*area| >= q * longest_edge^2 */
#define MIN_NORMAL_DOT 0.2f     /* reject normals that rotate past ~78 deg */

typedef struct { float x, y, z; } Vec3;
typedef struct { float x, y, z; int ok; } Proj;
typedef struct { int a, b; } Edge;
typedef struct { int a, b; float len; } Cand;
typedef struct { float x0, y0, x1, y1; } Seg;

typedef struct {
    Vec3 pos[MAX_V];
    int v_alive[MAX_V];
    int tri[MAX_T][3];
    int t_alive[MAX_T];
    int nv, nt;
    int vtn[MAX_V];             /* triangles incident to each vertex */
    int vt[MAX_V][VT_CAP];
    unsigned char v_overflow[MAX_V];   /* more incident tris than VT_CAP */
} Mesh;

typedef struct {
    Vec3 eye;                   /* camera axes are fixed: looking down -Z */
    float tan_half;
    int w, h;
} Camera;

static void mesh_adjacency(Mesh* m);

typedef struct {
    Mesh front;                 /* what the renderer draws */
    Mesh back;                  /* the job writes here */
    int running, done;
    int frames;
    int last_frame_collapses;
    int max_frame_collapses;
    int total_collapses;
    float target_px;
    int lock_silhouette;
} LodSystem;

/* scratch buffers, single-threaded demo */
static Edge g_edge_buf[MAX_E];
static Cand g_cand_buf[MAX_E];
static Seg g_ref_seg[MAX_SIL];
static Seg g_cur_seg[MAX_SIL];

static Vec3 v3(float x, float y, float z) { Vec3 r = { x, y, z }; return r; }
static Vec3 vadd(Vec3 a, Vec3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static Vec3 vsub(Vec3 a, Vec3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static Vec3 vmul(Vec3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static float vdot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static Vec3 vcross(Vec3 a, Vec3 b) {
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static float vlen(Vec3 a) { return sqrtf(vdot(a, a)); }
static Vec3 vnorm(Vec3 a) {
    float l = vlen(a);
    return l > 1e-9f ? vmul(a, 1.0f / l) : a;
}

/* ---------- mesh ---------- */

static void mesh_init(Mesh* m) {
    memset(m, 0, sizeof(*m));
}

static int mesh_add_vertex(Mesh* m, Vec3 p) {
    if (m->nv >= MAX_V) return -1;
    m->pos[m->nv] = p;
    m->v_alive[m->nv] = 1;
    return m->nv++;
}

static int mesh_add_tri(Mesh* m, int a, int b, int c) {
    if (m->nt >= MAX_T) return -1;
    m->tri[m->nt][0] = a;
    m->tri[m->nt][1] = b;
    m->tri[m->nt][2] = c;
    m->t_alive[m->nt] = 1;
    return m->nt++;
}

typedef struct { int a, b, idx; } Mid;

static int midpoint_index(Mesh* m, Mid* cache, int* ncache, int a, int b) {
    int lo = a < b ? a : b, hi = a < b ? b : a;
    for (int i = 0; i < *ncache; i++)
        if (cache[i].a == lo && cache[i].b == hi) return cache[i].idx;
    Vec3 p = vnorm(vmul(vadd(m->pos[lo], m->pos[hi]), 0.5f));
    int idx = mesh_add_vertex(m, p);
    if (idx < 0) return -1;
    if (*ncache < MAX_MID) cache[(*ncache)++] = (Mid){ lo, hi, idx };
    return idx;
}

static void mesh_icosphere(Mesh* m, int subdiv) {
    static const int base_tri[20][3] = {
        {0,11,5},{0,5,1},{0,1,7},{0,7,10},{0,10,11},
        {1,5,9},{5,11,4},{11,10,2},{10,7,6},{7,1,8},
        {3,9,4},{3,4,2},{3,2,6},{3,6,8},{3,8,9},
        {4,9,5},{2,4,11},{6,2,10},{8,6,7},{9,8,1}
    };
    static Mid cache[MAX_MID];
    int ncache = 0;
    mesh_init(m);
    float t = (1.0f + sqrtf(5.0f)) * 0.5f;
    Vec3 base[12] = {
        v3(-1, t, 0), v3(1, t, 0), v3(-1, -t, 0), v3(1, -t, 0),
        v3(0, -1, t), v3(0, 1, t), v3(0, -1, -t), v3(0, 1, -t),
        v3(t, 0, -1), v3(t, 0, 1), v3(-t, 0, -1), v3(-t, 0, 1)
    };
    for (int i = 0; i < 12; i++) mesh_add_vertex(m, vnorm(base[i]));
    for (int i = 0; i < 20; i++) mesh_add_tri(m, base_tri[i][0], base_tri[i][1], base_tri[i][2]);

    for (int level = 0; level < subdiv; level++) {
        int old_nt = m->nt;
        int (*old)[3] = (int (*)[3])malloc(sizeof(int) * 3 * (size_t)old_nt);
        if (!old) return;
        memcpy(old, m->tri, sizeof(int) * 3 * (size_t)old_nt);
        m->nt = 0;
        for (int i = 0; i < old_nt; i++) {
            int a = old[i][0], b = old[i][1], c = old[i][2];
            int ab = midpoint_index(m, cache, &ncache, a, b);
            int bc = midpoint_index(m, cache, &ncache, b, c);
            int ca = midpoint_index(m, cache, &ncache, c, a);
            mesh_add_tri(m, a, ab, ca);
            mesh_add_tri(m, b, bc, ab);
            mesh_add_tri(m, c, ca, bc);
            mesh_add_tri(m, ab, bc, ca);
        }
        free(old);
    }
}

/* adjacency is derived data; rebuild after any topological change.  A vertex
   with more than VT_CAP incident triangles cannot be validated by the link
   condition, so it is flagged and every collapse touching it is refused. */
static void mesh_adjacency(Mesh* m) {
    memset(m->vtn, 0, sizeof(int) * (size_t)m->nv);
    memset(m->v_overflow, 0, sizeof(m->v_overflow));
    for (int t = 0; t < m->nt; t++) {
        if (!m->t_alive[t]) continue;
        for (int k = 0; k < 3; k++) {
            int v = m->tri[t][k];
            if (m->vtn[v] < VT_CAP) m->vt[v][m->vtn[v]++] = t;
            else m->v_overflow[v] = 1;
        }
    }
}

static int tri_has(const int* tri, int v) {
    return tri[0] == v || tri[1] == v || tri[2] == v;
}

static int ring_centroid(const Mesh* m, int v, Vec3* out) {
    Vec3 s = v3(0, 0, 0);
    int n = 0;
    for (int i = 0; i < m->vtn[v]; i++) {
        int t = m->vt[v][i];
        for (int k = 0; k < 3; k++) {
            int c = m->tri[t][k];
            if (c != v) { s = vadd(s, m->pos[c]); n++; }
        }
    }
    if (n == 0) return 0;
    *out = vmul(s, 1.0f / (float)n);
    return 1;
}

/* a degenerate / near-degenerate triangle has an unstable normal, which would
   invent silhouette edges all over the surface */
static int tri_quality_ok(Vec3 p0, Vec3 p1, Vec3 p2) {
    Vec3 n = vcross(vsub(p1, p0), vsub(p2, p0));
    float longest = 0.0f;
    float e[3] = { vlen(vsub(p1, p0)), vlen(vsub(p2, p1)), vlen(vsub(p0, p2)) };
    for (int k = 0; k < 3; k++) if (e[k] > longest) longest = e[k];
    if (longest < 1e-9f) return 0;
    return vlen(n) >= MIN_QUALITY * longest * longest;
}

/* side of the triangle relative to the camera: +1 front, -1 back */
static int tri_facing(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 eye) {
    Vec3 n = vcross(vsub(p1, p0), vsub(p2, p0));
    Vec3 mid = vmul(vadd(vadd(p0, p1), p2), 1.0f / 3.0f);
    return vdot(n, vsub(eye, mid)) > 0.0f ? 1 : -1;
}

/* Edge collapse with the link condition: the only vertices shared by the two
   one-rings must be the vertices opposite the edge, otherwise the collapse
   would weld two separate sheets or create duplicate triangles.  Subset
   placement keeps the surviving endpoint at its old position, so no triangle
   can flip. */
static int try_collapse(Mesh* m, int a, int b, const Camera* cam) {
    if (a == b || !m->v_alive[a] || !m->v_alive[b]) return 0;
    if (m->v_overflow[a] || m->v_overflow[b]) return 0;

    int opp = 0;
    for (int i = 0; i < m->vtn[a]; i++) {
        int t = m->vt[a][i];
        if (m->t_alive[t] && tri_has(m->tri[t], b)) opp++;
    }
    if (opp < 1 || opp > 2) return 0;

    unsigned char mark[MAX_V];
    unsigned char seen[MAX_V];
    memset(mark, 0, sizeof(mark));
    memset(seen, 0, sizeof(seen));
    for (int i = 0; i < m->vtn[a]; i++) {
        int t = m->vt[a][i];
        for (int k = 0; k < 3; k++) mark[m->tri[t][k]] = 1;
    }
    int common = 0;
    for (int i = 0; i < m->vtn[b]; i++) {
        int t = m->vt[b][i];
        for (int k = 0; k < 3; k++) {
            int c = m->tri[t][k];
            if (c != a && c != b && mark[c] && !seen[c]) { seen[c] = 1; common++; }
        }
    }
    if (common != opp) return 0;

    int new_val = m->vtn[a] + m->vtn[b] - 2 * opp;
    if (new_val > VT_CAP) return 0;

    /* keep the endpoint closest to its own one-ring centroid */
    int keep = a, drop = b;
    Vec3 ca, cb;
    int has_a = ring_centroid(m, a, &ca);
    int has_b = ring_centroid(m, b, &cb);
    if (has_a && has_b) {
        float da = vlen(vsub(m->pos[a], ca));
        float db = vlen(vsub(m->pos[b], cb));
        if (db < da) { keep = b; drop = a; }
    }

    /* A triangle that contains the dropped vertex but not the kept one changes
       shape; reject the collapse if any of them would flip, rotate too far or
       collapse to a sliver.  Triangles holding both simply die, the rest are
       untouched.  The dot-product bound keeps nearly edge-on triangles from
       ending up with a normal pointing almost sideways, which would show up
       as a phantom silhouette edge inside the surface. */
    for (int t = 0; t < m->nt; t++) {
        if (!m->t_alive[t]) continue;
        const int* tr = m->tri[t];
        if (!tri_has(tr, drop) || tri_has(tr, keep)) continue;
        Vec3 q[3];
        for (int k = 0; k < 3; k++)
            q[k] = m->pos[tr[k] == drop ? keep : tr[k]];
        if (!tri_quality_ok(q[0], q[1], q[2])) return 0;
        int before = tri_facing(m->pos[tr[0]], m->pos[tr[1]], m->pos[tr[2]], cam->eye);
        int after = tri_facing(q[0], q[1], q[2], cam->eye);
        if (before != after) return 0;      /* crossed the silhouette plane */
        Vec3 old_n = vnorm(vcross(vsub(m->pos[tr[1]], m->pos[tr[0]]),
                                  vsub(m->pos[tr[2]], m->pos[tr[0]])));
        Vec3 new_n = vnorm(vcross(vsub(q[1], q[0]), vsub(q[2], q[0])));
        if (vdot(old_n, new_n) < MIN_NORMAL_DOT) return 0;
    }

    for (int t = 0; t < m->nt; t++) {
        if (!m->t_alive[t]) continue;
        int* tr = m->tri[t];
        for (int k = 0; k < 3; k++) if (tr[k] == drop) tr[k] = keep;
        if (tr[0] == tr[1] || tr[1] == tr[2] || tr[0] == tr[2]) m->t_alive[t] = 0;
    }
    m->v_alive[drop] = 0;
    mesh_adjacency(m);
    return 1;
}

/* ---------- camera, projection, silhouette ---------- */

static Camera make_camera(float dist, int w, int h, float fov_y) {
    Camera c;
    c.eye = v3(0.0f, 0.0f, dist);
    c.tan_half = tanf(fov_y * 0.5f);
    c.w = w; c.h = h;
    return c;
}

static Proj project(const Camera* c, Vec3 p) {
    Proj r = { 0.0f, 0.0f, 0.0f, 0 };
    Vec3 d = vsub(p, c->eye);
    float z = -d.z;                       /* forward = (0,0,-1) */
    if (z <= 1e-4f) return r;
    float f = (float)c->h * 0.5f / c->tan_half;
    r.x = (float)c->w * 0.5f + d.x / z * f;
    r.y = (float)c->h * 0.5f - d.y / z * f;
    r.z = z;
    r.ok = 1;
    return r;
}

static int cmp_edge(const void* A, const void* B) {
    const Edge* x = (const Edge*)A;
    const Edge* y = (const Edge*)B;
    if (x->a != y->a) return x->a < y->a ? -1 : 1;
    if (x->b != y->b) return x->b < y->b ? -1 : 1;
    return 0;
}

static int cmp_cand(const void* A, const void* B) {
    const Cand* x = (const Cand*)A;
    const Cand* y = (const Cand*)B;
    if (x->len < y->len) return -1;
    if (x->len > y->len) return 1;
    return 0;
}

/* A view silhouette edge separates a front-facing and a back-facing triangle
   (or borders the mesh).  Lock the endpoints and their one-ring: no triangle
   touching them can change, so the silhouette polyline is preserved exactly. */
static int silhouette_segments(const Mesh* m, const Camera* cam, const Proj* pj,
                               unsigned char* locked, Seg* segs, int cap) {
    unsigned char facing[MAX_T];
    for (int t = 0; t < m->nt; t++) {
        facing[t] = 0;
        if (!m->t_alive[t]) continue;
        Vec3 a = m->pos[m->tri[t][0]];
        Vec3 b = m->pos[m->tri[t][1]];
        Vec3 c = m->pos[m->tri[t][2]];
        Vec3 n = vcross(vsub(b, a), vsub(c, a));
        Vec3 mid = vmul(vadd(vadd(a, b), c), 1.0f / 3.0f);
        facing[t] = vdot(n, vsub(cam->eye, mid)) > 0.0f;
    }

    int ne = 0;
    for (int t = 0; t < m->nt; t++) {
        if (!m->t_alive[t]) continue;
        for (int k = 0; k < 3; k++) {
            int i = m->tri[t][k];
            int j = m->tri[t][(k + 1) % 3];
            int other = -1;
            for (int q = 0; q < m->vtn[i]; q++) {
                int u = m->vt[i][q];
                if (u != t && m->t_alive[u] && tri_has(m->tri[u], j)) { other = u; break; }
            }
            if (other >= 0 && facing[other] == facing[t]) continue;
            if (ne < MAX_E) g_edge_buf[ne].a = i < j ? i : j;
            if (ne < MAX_E) g_edge_buf[ne].b = i < j ? j : i;
            ne++;
        }
    }
    if (ne > MAX_E) ne = MAX_E;
    qsort(g_edge_buf, (size_t)ne, sizeof(Edge), cmp_edge);

    int nseg = 0;
    for (int i = 0; i < ne; i++) {
        if (i > 0 && g_edge_buf[i].a == g_edge_buf[i - 1].a &&
            g_edge_buf[i].b == g_edge_buf[i - 1].b) continue;
        int a = g_edge_buf[i].a, b = g_edge_buf[i].b;
        if (locked) {
            for (int q = 0; q < m->vtn[a]; q++) {
                int t = m->vt[a][q];
                for (int k = 0; k < 3; k++) locked[m->tri[t][k]] = 1;
            }
            for (int q = 0; q < m->vtn[b]; q++) {
                int t = m->vt[b][q];
                for (int k = 0; k < 3; k++) locked[m->tri[t][k]] = 1;
            }
        }
        if (segs && nseg < cap && pj[a].ok && pj[b].ok) {
            segs[nseg].x0 = pj[a].x; segs[nseg].y0 = pj[a].y;
            segs[nseg].x1 = pj[b].x; segs[nseg].y1 = pj[b].y;
            nseg++;
        }
    }
    return nseg;
}

/* ---------- screen-space density ---------- */

static int gather_candidates(const Mesh* m, const Proj* pj, float target_px,
                             const unsigned char* locked, Cand* out, int cap) {
    int ne = 0;
    for (int t = 0; t < m->nt; t++) {
        if (!m->t_alive[t]) continue;
        for (int k = 0; k < 3; k++) {
            int i = m->tri[t][k];
            int j = m->tri[t][(k + 1) % 3];
            if (ne < MAX_E) {
                g_edge_buf[ne].a = i < j ? i : j;
                g_edge_buf[ne].b = i < j ? j : i;
            }
            ne++;
        }
    }
    if (ne > MAX_E) ne = MAX_E;
    qsort(g_edge_buf, (size_t)ne, sizeof(Edge), cmp_edge);

    int nc = 0;
    for (int i = 0; i < ne; i++) {
        if (i > 0 && g_edge_buf[i].a == g_edge_buf[i - 1].a &&
            g_edge_buf[i].b == g_edge_buf[i - 1].b) continue;
        int a = g_edge_buf[i].a, b = g_edge_buf[i].b;
        if (!m->v_alive[a] || !m->v_alive[b]) continue;
        if (locked[a] || locked[b]) continue;
        if (!pj[a].ok || !pj[b].ok) continue;
        float dx = pj[a].x - pj[b].x, dy = pj[a].y - pj[b].y;
        float len = sqrtf(dx * dx + dy * dy);
        if (len >= target_px) continue;
        if (nc >= cap) break;
        out[nc].a = a; out[nc].b = b; out[nc].len = len;
        nc++;
    }
    return nc;
}

static int mesh_alive_verts(const Mesh* m) {
    int n = 0;
    for (int i = 0; i < m->nv; i++) n += m->v_alive[i];
    return n;
}

static int mesh_alive_tris(const Mesh* m) {
    int n = 0;
    for (int i = 0; i < m->nt; i++) n += m->t_alive[i];
    return n;
}

/* average / max projected edge length over the alive mesh */
static void edge_screen_stats(const Mesh* m, const Proj* pj, float* avg, float* maxv) {
    int ne = 0;
    for (int t = 0; t < m->nt; t++) {
        if (!m->t_alive[t]) continue;
        for (int k = 0; k < 3; k++) {
            int i = m->tri[t][k];
            int j = m->tri[t][(k + 1) % 3];
            if (ne < MAX_E) {
                g_edge_buf[ne].a = i < j ? i : j;
                g_edge_buf[ne].b = i < j ? j : i;
            }
            ne++;
        }
    }
    if (ne > MAX_E) ne = MAX_E;
    qsort(g_edge_buf, (size_t)ne, sizeof(Edge), cmp_edge);
    double sum = 0.0;
    float mx = 0.0f;
    int cnt = 0;
    for (int i = 0; i < ne; i++) {
        if (i > 0 && g_edge_buf[i].a == g_edge_buf[i - 1].a &&
            g_edge_buf[i].b == g_edge_buf[i - 1].b) continue;
        int a = g_edge_buf[i].a, b = g_edge_buf[i].b;
        if (!pj[a].ok || !pj[b].ok) continue;
        float dx = pj[a].x - pj[b].x, dy = pj[a].y - pj[b].y;
        float len = sqrtf(dx * dx + dy * dy);
        sum += len;
        if (len > mx) mx = len;
        cnt++;
    }
    *avg = cnt > 0 ? (float)(sum / cnt) : 0.0f;
    *maxv = mx;
}

/* ---------- time-sliced job ---------- */

static void lod_begin(LodSystem* s) {
    s->back = s->front;
    mesh_adjacency(&s->back);
    s->running = 1;
    s->done = 0;
    s->frames = 0;
    s->last_frame_collapses = 0;
    s->max_frame_collapses = 0;
    s->total_collapses = 0;
}

/* do at most `budget` collapse attempts, all in the back buffer */
static int lod_step(LodSystem* s, const Camera* cam, int budget) {
    if (!s->running) return 0;
    Mesh* m = &s->back;

    Proj pj[MAX_V];
    for (int v = 0; v < m->nv; v++) pj[v] = project(cam, m->pos[v]);

    unsigned char locked[MAX_V];
    memset(locked, 0, sizeof(locked));
    if (s->lock_silhouette) {
        mesh_adjacency(m);
        silhouette_segments(m, cam, pj, locked, NULL, 0);
    }

    int nc = gather_candidates(m, pj, s->target_px, locked, g_cand_buf, MAX_E);
    qsort(g_cand_buf, (size_t)nc, sizeof(Cand), cmp_cand);

    int attempts = 0, collapses = 0;
    for (int i = 0; i < nc && collapses < budget; i++) {
        int a = g_cand_buf[i].a, b = g_cand_buf[i].b;
        if (!m->v_alive[a] || !m->v_alive[b]) continue;
        if (locked[a] || locked[b]) continue;
        attempts++;
        if (try_collapse(m, a, b, cam)) collapses++;
    }

    s->frames++;
    s->last_frame_collapses = collapses;
    s->total_collapses += collapses;
    if (collapses > s->max_frame_collapses) s->max_frame_collapses = collapses;
    /* The job is done when a pass over every candidate produced nothing: the
       short edges that remain are all blocked by the link condition, the
       quality floor or the silhouette rule.  The pass is bounded because the
       loop only stops early once `budget` collapses succeeded. */
    if (collapses == 0) s->done = 1;
    return collapses;
}

/* frame boundary: publish the finished back buffer, otherwise keep drawing
   the old front buffer */
static int lod_commit(LodSystem* s) {
    if (!s->running || !s->done) return 0;
    s->front = s->back;
    mesh_adjacency(&s->front);
    s->running = 0;
    return 1;
}

/* ---------- silhouette error ---------- */

static float point_seg_dist(float px, float py, const Seg* s) {
    float vx = s->x1 - s->x0, vy = s->y1 - s->y0;
    float wx = px - s->x0, wy = py - s->y0;
    float vv = vx * vx + vy * vy;
    float t = vv > 1e-12f ? (wx * vx + wy * vy) / vv : 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float dx = px - (s->x0 + vx * t);
    float dy = py - (s->y0 + vy * t);
    return sqrtf(dx * dx + dy * dy);
}

/* max distance from samples on A to the polyline B, in pixels */
static float one_way_deviation(const Seg* A, int na, const Seg* B, int nb) {
    if (na == 0 || nb == 0) return FLT_MAX;
    const int SAMPLES = 32;
    float worst = 0.0f;
    for (int i = 0; i < na; i++) {
        for (int k = 0; k <= SAMPLES; k++) {
            float u = (float)k / (float)SAMPLES;
            float px = A[i].x0 + (A[i].x1 - A[i].x0) * u;
            float py = A[i].y0 + (A[i].y1 - A[i].y0) * u;
            float best = FLT_MAX;
            for (int j = 0; j < nb; j++) {
                float d = point_seg_dist(px, py, &B[j]);
                if (d < best) best = d;
            }
            if (best > worst) worst = best;
        }
    }
    return worst;
}

static unsigned long long mesh_checksum(const Mesh* m) {
    unsigned long long h = 1469598103934665603ULL;
    for (int v = 0; v < m->nv; v++) {
        if (!m->v_alive[v]) continue;
        long long q[3] = {
            (long long)lroundf(m->pos[v].x * 4096.0f),
            (long long)lroundf(m->pos[v].y * 4096.0f),
            (long long)lroundf(m->pos[v].z * 4096.0f)
        };
        for (int k = 0; k < 3; k++) { h ^= (unsigned long long)q[k]; h *= 1099511628211ULL; }
    }
    for (int t = 0; t < m->nt; t++) {
        if (!m->t_alive[t]) continue;
        for (int k = 0; k < 3; k++) { h ^= (unsigned long long)m->tri[t][k]; h *= 1099511628211ULL; }
    }
    return h;
}

/* ---------- tests ---------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

typedef struct {
    int v0, t0, v1, t1;
    int frames, collapses, max_frame;
    int front_stable, swapped_once;
    float sil_err_px;
    float avg_before, max_before, avg_after, max_after;
} CaseResult;

static void run_case(const Mesh* orig, float dist, float target_px, int lock_sil,
                     CaseResult* r) {
    static LodSystem sys;
    Camera cam = make_camera(dist, 1280, 720, 1.0471976f);   /* 60 deg */
    memset(r, 0, sizeof(*r));

    sys.front = *orig;
    mesh_adjacency(&sys.front);
    sys.target_px = target_px;
    sys.lock_silhouette = lock_sil;

    Proj pj[MAX_V];
    for (int v = 0; v < sys.front.nv; v++) pj[v] = project(&cam, sys.front.pos[v]);
    edge_screen_stats(&sys.front, pj, &r->avg_before, &r->max_before);

    unsigned char dummy[MAX_V];
    memset(dummy, 0, sizeof(dummy));
    int nref = silhouette_segments(&sys.front, &cam, pj, dummy, g_ref_seg, MAX_SIL);

    r->v0 = mesh_alive_verts(&sys.front);
    r->t0 = mesh_alive_tris(&sys.front);
    unsigned long long before = mesh_checksum(&sys.front);

    lod_begin(&sys);
    r->front_stable = 1;
    int frames = 0;
    for (; frames < MAX_FRAMES && sys.running; frames++) {
        /* this frame draws sys.front; the job must not touch it */
        if (mesh_checksum(&sys.front) != before) r->front_stable = 0;
        int c = lod_step(&sys, &cam, STREAM_BUDGET);
        if (mesh_checksum(&sys.front) != before) r->front_stable = 0;
        if (c > r->max_frame) r->max_frame = c;
        if (lod_commit(&sys)) r->swapped_once = 1;
        if (sys.running) continue;
    }
    r->frames = sys.frames;
    r->collapses = sys.total_collapses;
    r->v1 = mesh_alive_verts(&sys.front);
    r->t1 = mesh_alive_tris(&sys.front);

    if (sys.total_collapses > 0 && mesh_checksum(&sys.front) == before)
        r->front_stable = 0;               /* committed result must be visible */

    for (int v = 0; v < sys.front.nv; v++) pj[v] = project(&cam, sys.front.pos[v]);
    edge_screen_stats(&sys.front, pj, &r->avg_after, &r->max_after);
    int ncur = silhouette_segments(&sys.front, &cam, pj, NULL, g_cur_seg, MAX_SIL);

    float d1 = one_way_deviation(g_ref_seg, nref, g_cur_seg, ncur);
    float d2 = one_way_deviation(g_cur_seg, ncur, g_ref_seg, nref);
    r->sil_err_px = d1 > d2 ? d1 : d2;
}

int main(void) {
    static Mesh orig;
    mesh_icosphere(&orig, 4);              /* 2562 verts, 5120 tris */

    printf("mesh=%d verts %d tris target_px=%.1f budget=%d/frame\n",
           mesh_alive_verts(&orig), mesh_alive_tris(&orig), TARGET_PX, STREAM_BUDGET);

    const float dists[] = { 4.0f, 8.0f, 16.0f, 32.0f, 64.0f };
    const int ncases = (int)(sizeof(dists) / sizeof(dists[0]));
    CaseResult res[5];
    int all_err_ok = 1, all_stable = 1, all_budget_ok = 1;
    for (int i = 0; i < ncases; i++) {
        run_case(&orig, dists[i], TARGET_PX, 1, &res[i]);
        CaseResult* r = &res[i];
        float pct = r->v0 > 0 ? 100.0f * (float)(r->v0 - r->v1) / (float)r->v0 : 0.0f;
        printf("d=%5.1f verts %4d->%4d (%5.1f%% fewer) tris %4d->%4d "
               "frames=%2d collapses=%4d max/frame=%2d edge_px %.2f->%.2f sil_err=%.3fpx\n",
               dists[i], r->v0, r->v1, pct, r->t0, r->t1,
               r->frames, r->collapses, r->max_frame,
               r->avg_before, r->avg_after, r->sil_err_px);
        if (!(r->sil_err_px < 1.0f)) all_err_ok = 0;
        if (!r->front_stable) all_stable = 0;
        if (r->max_frame > STREAM_BUDGET) all_budget_ok = 0;
    }

    CaseResult far, near;
    far = res[ncases - 1];
    near = res[0];

    check("silhouette_err_under_1px", all_err_ok);
    check("front_buffer_untouched", all_stable);
    check("budget_respected", all_budget_ok);
    check("far_more_simplified_than_near", far.v1 < near.v1);
    check("far_reduction_over_40pct", far.v1 * 10 <= far.v0 * 6);
    check("far_used_multiple_frames", far.frames > 1);
    check("near_keeps_more_than_far", near.v1 > far.v1);
    check("near_reduction_below_far", (float)(near.v0 - near.v1) < (float)(far.v0 - far.v1));
    check("job_committed_once", far.swapped_once == 1);

    /* contrast: without silhouette locking the same target cuts the contour */
    CaseResult lock_on, lock_off;
    run_case(&orig, 20.0f, 24.0f, 1, &lock_on);
    run_case(&orig, 20.0f, 24.0f, 0, &lock_off);
    printf("contrast target_px=24 d=20 lock_on  verts %d->%d sil_err=%.3fpx\n",
           lock_on.v0, lock_on.v1, lock_on.sil_err_px);
    printf("contrast target_px=24 d=20 lock_off verts %d->%d sil_err=%.3fpx\n",
           lock_off.v0, lock_off.v1, lock_off.sil_err_px);
    check("lock_on_under_1px", lock_on.sil_err_px < 1.0f);
    check("lock_off_worse", lock_off.sil_err_px > lock_on.sil_err_px);
    check("lock_off_over_1px", lock_off.sil_err_px > 1.0f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
