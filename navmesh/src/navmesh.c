
/* mini-navmesh: walkable cells merged into convex rects (navmesh), A* over
   the rect adjacency graph, funnel algorithm to straighten the corridor.
   Grid A* (octile, no corner cutting) kept as baseline: navmesh path length
   must not be worse than the grid version. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

typedef struct { int x, y; } Pt;
typedef struct { double x, y; } V2;

typedef struct { int w, h; unsigned char* blocked; } Grid;

static int walkable(const Grid* g, int x, int y) {
    return x >= 0 && y >= 0 && x < g->w && y < g->h && !g->blocked[y * g->w + x];
}

/* ---------------- binary min-heap (id keyed by double) ---------------- */
typedef struct { int* id; double* key; int n, cap; } Heap;

static void hinit(Heap* h, int cap) {
    h->id = (int*)malloc(sizeof(int) * (size_t)cap);
    h->key = (double*)malloc(sizeof(double) * (size_t)cap);
    h->n = 0; h->cap = cap;
}
static void hfree(Heap* h) { free(h->id); free(h->key); }
static void hpush(Heap* h, int id, double k) {
    if (h->n == h->cap) {
        h->cap *= 2;
        h->id = (int*)realloc(h->id, sizeof(int) * (size_t)h->cap);
        h->key = (double*)realloc(h->key, sizeof(double) * (size_t)h->cap);
    }
    int i = h->n++;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (h->key[p] <= k) break;
        h->id[i] = h->id[p]; h->key[i] = h->key[p]; i = p;
    }
    h->id[i] = id; h->key[i] = k;
}
static int hpop(Heap* h, double* k) {
    int top = h->id[0]; *k = h->key[0];
    h->n--;
    if (h->n > 0) {
        int id = h->id[h->n]; double kk = h->key[h->n];
        int i = 0;
        for (;;) {
            int c = 2 * i + 1;
            if (c >= h->n) break;
            if (c + 1 < h->n && h->key[c + 1] < h->key[c]) c++;
            if (h->key[c] >= kk) break;
            h->id[i] = h->id[c]; h->key[i] = h->key[c]; i = c;
        }
        h->id[i] = id; h->key[i] = kk;
    }
    return top;
}

/* ---------------- grid A* baseline (8-dir, no corner cutting) ---------------- */
static const int DIRS[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

typedef struct { double g, f; int parent; unsigned char open, closed; } GNode;

/* diagonal step p->q is legal only if both orthogonal neighbours are free */
static int step_ok(const Grid* g, Pt p, Pt q) {
    int dx = q.x - p.x, dy = q.y - p.y;
    if (!walkable(g, q.x, q.y)) return 0;
    if (dx != 0 && dy != 0) {
        if (g->blocked[p.y * g->w + q.x] || g->blocked[q.y * g->w + p.x]) return 0;
    }
    return 1;
}

static double octile(int dx, int dy) {
    dx = abs(dx); dy = abs(dy);
    int mn = dx < dy ? dx : dy, mx = dx > dy ? dx : dy;
    return mx + 0.41421356237309503 * mn;
}

static int grid_astar(const Grid* g, Pt s, Pt t, Pt* out, int max_out) {
    if (!walkable(g, s.x, s.y) || !walkable(g, t.x, t.y)) return -1;
    int n = g->w * g->h;
    GNode* nd = (GNode*)calloc((size_t)n, sizeof(GNode));
    Heap h; hinit(&h, 256);
    int si = s.y * g->w + s.x, ti = t.y * g->w + t.x;
    nd[si].g = 0.0; nd[si].f = octile(t.x - s.x, t.y - s.y);
    nd[si].parent = -1; nd[si].open = 1;
    hpush(&h, si, nd[si].f);
    int found = 0;
    while (h.n > 0) {
        double k; int cur = hpop(&h, &k);
        if (nd[cur].closed) continue;
        if (k > nd[cur].f) continue; /* stale heap entry */
        nd[cur].closed = 1;
        if (cur == ti) { found = 1; break; }
        Pt cp = { cur % g->w, cur / g->w };
        for (int d = 0; d < 8; d++) {
            Pt np = { cp.x + DIRS[d][0], cp.y + DIRS[d][1] };
            if (!step_ok(g, cp, np)) continue;
            int ni = np.y * g->w + np.x;
            if (nd[ni].closed) continue;
            double step = (DIRS[d][0] && DIRS[d][1]) ? 1.4142135623730951 : 1.0;
            double ng = nd[cur].g + step;
            if (!nd[ni].open || ng < nd[ni].g) {
                nd[ni].g = ng;
                nd[ni].f = ng + octile(t.x - np.x, t.y - np.y);
                nd[ni].parent = cur;
                nd[ni].open = 1;
                hpush(&h, ni, nd[ni].f);
            }
        }
    }
    int rc = -1;
    if (found) {
        int m = 0, cur = ti;
        while (cur != -1) m++, cur = nd[cur].parent;
        if (m <= max_out) {
            cur = ti;
            for (int i = m - 1; i >= 0; i--) {
                out[i].x = cur % g->w; out[i].y = cur / g->w;
                cur = nd[cur].parent;
            }
            rc = m;
        }
    }
    hfree(&h); free(nd);
    return rc;
}

static double pts_length(const Pt* p, int n) {
    double s = 0.0;
    for (int i = 1; i < n; i++) s += hypot((double)(p[i].x - p[i-1].x), (double)(p[i].y - p[i-1].y));
    return s;
}

/* ---------------- navmesh: rect decomposition + adjacency ---------------- */
typedef struct { int x0, y0, x1, y1; } Rect; /* inclusive cell coords; convex */

typedef struct {
    int w, h;
    Rect* rects; int nrects;
    int* cell_rect;          /* per cell -> rect id, -1 if blocked */
    int* adj_off;            /* CSR offsets, nrects+1 */
    int* adj_to;             /* neighbour rect id */
    double *adj_px0, *adj_px1, *adj_py; /* shared portal segment (continuous coords) */
    int nadj;
    int walkable_cells;
} NavMesh;

static int navmesh_build(NavMesh* nm, const Grid* g) {
    memset(nm, 0, sizeof(*nm));
    nm->w = g->w; nm->h = g->h;
    int cap = 1024;
    nm->rects = (Rect*)malloc(sizeof(Rect) * (size_t)cap);
    nm->cell_rect = (int*)malloc(sizeof(int) * (size_t)g->w * g->h);
    int* prev = (int*)malloc(sizeof(int) * (size_t)g->w); /* rect covering column x in row y-1 */
    for (int x = 0; x < g->w; x++) prev[x] = -1;
    nm->walkable_cells = 0;

    /* horizontal strips, merged vertically when the run [x0,x1] repeats */
    for (int y = 0; y < g->h; y++) {
        int x = 0;
        while (x < g->w) {
            if (g->blocked[y * g->w + x]) {
                nm->cell_rect[y * g->w + x] = -1;
                prev[x] = -1;
                x++;
                continue;
            }
            int x0 = x;
            while (x < g->w && !g->blocked[y * g->w + x]) x++;
            int x1 = x - 1;
            int id = -1, cand = prev[x0];
            if (cand >= 0 && nm->rects[cand].x0 == x0 && nm->rects[cand].x1 == x1
                && nm->rects[cand].y1 == y - 1) {
                nm->rects[cand].y1 = y;
                id = cand;
            } else {
                if (nm->nrects == cap) {
                    cap *= 2;
                    nm->rects = (Rect*)realloc(nm->rects, sizeof(Rect) * (size_t)cap);
                }
                id = nm->nrects++;
                nm->rects[id].x0 = x0; nm->rects[id].y0 = y;
                nm->rects[id].x1 = x1; nm->rects[id].y1 = y;
            }
            nm->walkable_cells += x1 - x0 + 1;
            for (int cx = x0; cx <= x1; cx++) { nm->cell_rect[y * g->w + cx] = id; prev[cx] = id; }
        }
    }
    free(prev);

    /* adjacency: strips are x-maximal, so rects only touch across row bands.
       bucket rects by y1, then match each rect against the band above its y0. */
    int* cnt = (int*)calloc((size_t)g->h, sizeof(int));
    for (int r = 0; r < nm->nrects; r++) cnt[nm->rects[r].y1]++;
    int* off = (int*)malloc(sizeof(int) * ((size_t)g->h + 1));
    off[0] = 0;
    for (int y = 0; y < g->h; y++) off[y + 1] = off[y] + cnt[y];
    int* order = (int*)malloc(sizeof(int) * (size_t)(nm->nrects > 0 ? nm->nrects : 1));
    int* cur = (int*)malloc(sizeof(int) * (size_t)g->h);
    memcpy(cur, off, sizeof(int) * (size_t)g->h);
    for (int r = 0; r < nm->nrects; r++) order[cur[nm->rects[r].y1]++] = r;

    int* deg = (int*)calloc((size_t)(nm->nrects > 0 ? nm->nrects : 1), sizeof(int));
    nm->nadj = 0;
    for (int b = 0; b < nm->nrects; b++) {
        int y = nm->rects[b].y0 - 1;
        if (y < 0) continue;
        for (int j = off[y]; j < off[y + 1]; j++) {
            int a = order[j];
            int lo = nm->rects[a].x0 > nm->rects[b].x0 ? nm->rects[a].x0 : nm->rects[b].x0;
            int hi = nm->rects[a].x1 < nm->rects[b].x1 ? nm->rects[a].x1 : nm->rects[b].x1;
            if (lo <= hi) { deg[a]++; deg[b]++; nm->nadj += 2; }
        }
    }
    nm->adj_off = (int*)malloc(sizeof(int) * ((size_t)nm->nrects + 1));
    nm->adj_off[0] = 0;
    for (int r = 0; r < nm->nrects; r++) nm->adj_off[r + 1] = nm->adj_off[r] + deg[r];
    nm->adj_to = (int*)malloc(sizeof(int) * (size_t)(nm->nadj > 0 ? nm->nadj : 1));
    nm->adj_px0 = (double*)malloc(sizeof(double) * (size_t)(nm->nadj > 0 ? nm->nadj : 1));
    nm->adj_px1 = (double*)malloc(sizeof(double) * (size_t)(nm->nadj > 0 ? nm->nadj : 1));
    nm->adj_py = (double*)malloc(sizeof(double) * (size_t)(nm->nadj > 0 ? nm->nadj : 1));
    int* pos = (int*)calloc((size_t)(nm->nrects > 0 ? nm->nrects : 1), sizeof(int));
    for (int b = 0; b < nm->nrects; b++) {
        int y = nm->rects[b].y0 - 1;
        if (y < 0) continue;
        for (int j = off[y]; j < off[y + 1]; j++) {
            int a = order[j];
            int lo = nm->rects[a].x0 > nm->rects[b].x0 ? nm->rects[a].x0 : nm->rects[b].x0;
            int hi = nm->rects[a].x1 < nm->rects[b].x1 ? nm->rects[a].x1 : nm->rects[b].x1;
            if (lo > hi) continue;
            double py = (double)nm->rects[b].y0; /* continuous boundary y */
            double px0 = (double)lo, px1 = (double)hi + 1.0;
            int e;
            e = nm->adj_off[a] + pos[a]++;
            nm->adj_to[e] = b; nm->adj_px0[e] = px0; nm->adj_px1[e] = px1; nm->adj_py[e] = py;
            e = nm->adj_off[b] + pos[b]++;
            nm->adj_to[e] = a; nm->adj_px0[e] = px0; nm->adj_px1[e] = px1; nm->adj_py[e] = py;
        }
    }
    free(pos); free(cur); free(order); free(off); free(cnt); free(deg);
    return 0;
}

static void navmesh_free(NavMesh* nm) {
    free(nm->rects); free(nm->cell_rect);
    free(nm->adj_off); free(nm->adj_to);
    free(nm->adj_px0); free(nm->adj_px1); free(nm->adj_py);
    memset(nm, 0, sizeof(*nm));
}

/* ---------------- A* over rects (costs via portal midpoints) ---------------- */
typedef struct {
    double g, f, ex, ey;      /* entry point (ex,ey) where this rect was entered */
    int parent;
    unsigned char open;
} MNode;

static int mesh_astar(const NavMesh* nm, int s, int t, V2 sp, V2 gp,
                      int* path, int max_path) {
    MNode* nd = (MNode*)calloc((size_t)nm->nrects, sizeof(MNode));
    Heap h; hinit(&h, 256);
    nd[s].g = 0.0; nd[s].f = hypot(gp.x - sp.x, gp.y - sp.y);
    nd[s].ex = sp.x; nd[s].ey = sp.y; nd[s].parent = -1; nd[s].open = 1;
    hpush(&h, s, nd[s].f);
    int found = 0;
    while (h.n > 0) {
        double k; int c = hpop(&h, &k);
        if (k > nd[c].f) continue; /* stale */
        if (c == t) { found = 1; break; }
        for (int e = nm->adj_off[c]; e < nm->adj_off[c + 1]; e++) {
            int to = nm->adj_to[e];
            double mx = (nm->adj_px0[e] + nm->adj_px1[e]) * 0.5, my = nm->adj_py[e];
            double ng = nd[c].g + hypot(mx - nd[c].ex, my - nd[c].ey);
            if (!nd[to].open || ng < nd[to].g - 1e-12) {
                nd[to].g = ng;
                nd[to].f = ng + hypot(gp.x - mx, gp.y - my); /* admissible */
                nd[to].ex = mx; nd[to].ey = my;
                nd[to].parent = c;
                nd[to].open = 1;
                hpush(&h, to, nd[to].f);
            }
        }
    }
    int rc = -1;
    if (found) {
        int m = 0, c = t;
        while (c != -1) m++, c = nd[c].parent;
        if (m <= max_path) {
            c = t;
            for (int i = m - 1; i >= 0; i--) { path[i] = c; c = nd[c].parent; }
            rc = m;
        }
    }
    hfree(&h); free(nd);
    return rc;
}

/* ---------------- funnel (simple stupid funnel, Mononen) ---------------- */
static double triarea2(V2 a, V2 b, V2 c) {
    return (c.x - a.x) * (b.y - a.y) - (b.x - a.x) * (c.y - a.y);
}

static int funnel(V2 sp, V2 gp, const double* px0, const double* px1, const double* py,
                  const int* dir_down, int np, V2* out, int max_out) {
    int m = np + 2;
    V2* L = (V2*)malloc(sizeof(V2) * (size_t)m);
    V2* R = (V2*)malloc(sizeof(V2) * (size_t)m);
    L[0] = sp; R[0] = sp;
    for (int i = 0; i < np; i++) {
        V2 a = { px0[i], py[i] }, b = { px1[i], py[i] };
        if (dir_down[i]) { L[i + 1] = a; R[i + 1] = b; } /* crossing downward: left = smaller x */
        else             { L[i + 1] = b; R[i + 1] = a; }
    }
    L[m - 1] = gp; R[m - 1] = gp;

    int n = 0;
    V2 apex = sp, left = sp, right = sp;
    int apex_i = 0, left_i = 0, right_i = 0;
    out[n++] = sp;
    for (int i = 1; i < m; i++) {
        /* update right */
        if (triarea2(apex, right, R[i]) <= 0.0) {
            if ((apex.x == right.x && apex.y == right.y) || triarea2(apex, left, R[i]) > 0.0) {
                right = R[i]; right_i = i;
            } else {
                if (n >= max_out) { free(L); free(R); return -1; }
                out[n++] = left;
                apex = left; apex_i = left_i;
                left = apex; right = apex; left_i = apex_i; right_i = apex_i;
                i = apex_i;
                continue;
            }
        }
        /* update left */
        if (triarea2(apex, left, L[i]) >= 0.0) {
            if ((apex.x == left.x && apex.y == left.y) || triarea2(apex, right, L[i]) < 0.0) {
                left = L[i]; left_i = i;
            } else {
                if (n >= max_out) { free(L); free(R); return -1; }
                out[n++] = right;
                apex = right; apex_i = right_i;
                left = apex; right = apex; left_i = apex_i; right_i = apex_i;
                i = apex_i;
                continue;
            }
        }
    }
    if (n >= max_out) { free(L); free(R); return -1; }
    if (out[n-1].x != gp.x || out[n-1].y != gp.y) out[n++] = gp;
    free(L); free(R);
    return n;
}

/* portal shared by two adjacent rects */
static int find_portal(const NavMesh* nm, int a, int b,
                       double* px0, double* px1, double* py) {
    for (int e = nm->adj_off[a]; e < nm->adj_off[a + 1]; e++) {
        if (nm->adj_to[e] == b) {
            *px0 = nm->adj_px0[e]; *px1 = nm->adj_px1[e]; *py = nm->adj_py[e];
            return 1;
        }
    }
    return 0;
}

/* funnel over an explicit rect corridor */
static int funnel_corridor(const NavMesh* nm, const int* crects, int cn,
                           V2 sp, V2 gp, V2* out, int max_out) {
    if (cn <= 0) return -1;
    int np = cn - 1;
    double* px0 = (double*)malloc(sizeof(double) * (size_t)(np > 0 ? np : 1));
    double* px1 = (double*)malloc(sizeof(double) * (size_t)(np > 0 ? np : 1));
    double* py = (double*)malloc(sizeof(double) * (size_t)(np > 0 ? np : 1));
    int* dir = (int*)malloc(sizeof(int) * (size_t)(np > 0 ? np : 1));
    int ok = 1;
    for (int k = 0; k < np; k++) {
        if (!find_portal(nm, crects[k], crects[k + 1], &px0[k], &px1[k], &py[k])) { ok = 0; break; }
        dir[k] = (nm->rects[crects[k + 1]].y0 == (int)py[k]);
    }
    int n = ok ? funnel(sp, gp, px0, px1, py, dir, np, out, max_out) : -1;
    free(px0); free(px1); free(py); free(dir);
    return n;
}

/* map a grid path to a rect corridor: horizontal neighbours share a strip,
   diagonal steps land in a rect adjacent through the row below/above, so
   consecutive distinct rects are always portal-adjacent */
static int grid_corridor(const NavMesh* nm, const Pt* pts, int n, int* crects, int max) {
    int m = 0, prev = -1;
    for (int i = 0; i < n; i++) {
        int r = nm->cell_rect[pts[i].y * nm->w + pts[i].x];
        if (r < 0) return -1;
        if (r != prev) {
            if (m >= max) return -1;
            crects[m++] = r;
            prev = r;
        }
    }
    return m;
}

static double v2_length(const V2* p, int n) {
    double s = 0.0;
    for (int i = 1; i < n; i++) s += hypot(p[i].x - p[i-1].x, p[i].y - p[i-1].y);
    return s;
}

/* full query: cell endpoints -> funnel waypoints. Runs the funnel on both the
   mesh-A* corridor and the corridor seeded by the grid A* path, and keeps the
   shorter one — the grid-seeded corridor contains the grid path itself, so the
   result is never worse than the grid version. `out`/`corridor` must have room
   for g->w * g->h + 8 entries (worst case: corridor as long as the grid path). */
static int navmesh_path(const NavMesh* nm, const Grid* g, Pt s, Pt t,
                        V2* out, int max_out, int* corridor, int* clen) {
    int si = nm->cell_rect[s.y * nm->w + s.x];
    int ti = nm->cell_rect[t.y * nm->w + t.x];
    if (si < 0 || ti < 0) return -1;
    V2 sp = { s.x + 0.5, s.y + 0.5 }, gp = { t.x + 0.5, t.y + 0.5 };
    int cap = g->w * g->h;
    int* cr = (int*)malloc(sizeof(int) * (size_t)cap);
    V2* cand = (V2*)malloc(sizeof(V2) * (size_t)max_out);
    int best = -1, best_cn = 0;
    double best_len = 0.0;

    for (int src = 0; src < 2; src++) {
        int cn;
        if (src == 0) {
            cn = mesh_astar(nm, si, ti, sp, gp, cr, cap);
        } else {
            Pt* gpts = (Pt*)malloc(sizeof(Pt) * (size_t)cap);
            int gn = grid_astar(g, s, t, gpts, cap);
            cn = gn > 0 ? grid_corridor(nm, gpts, gn, cr, cap) : -1;
            free(gpts);
        }
        if (cn <= 0) continue;
        int n = funnel_corridor(nm, cr, cn, sp, gp, cand, max_out);
        if (n <= 0) continue;
        double len = v2_length(cand, n);
        if (best < 0 || len < best_len - 1e-12) {
            memcpy(out, cand, sizeof(V2) * (size_t)n);
            memcpy(corridor, cr, sizeof(int) * (size_t)cn);
            best = n; best_cn = cn; best_len = len;
        }
    }
    free(cr); free(cand);
    if (clen) *clen = best_cn;
    return best;
}

/* every sampled point of every segment must lie inside some corridor rect */
static int path_in_corridor(const NavMesh* nm, const int* corridor, int clen,
                            const V2* path, int plen) {
    const double EPS = 1e-6;
    for (int i = 1; i < plen; i++) {
        for (int s = 0; s <= 64; s++) {
            double t = (double)s / 64.0;
            double x = path[i-1].x + (path[i].x - path[i-1].x) * t;
            double y = path[i-1].y + (path[i].y - path[i-1].y) * t;
            int inside = 0;
            for (int k = 0; k < clen && !inside; k++) {
                const Rect* r = &nm->rects[corridor[k]];
                if (x >= r->x0 - EPS && x <= r->x1 + 1.0 + EPS &&
                    y >= r->y0 - EPS && y <= r->y1 + 1.0 + EPS) inside = 1;
            }
            if (!inside) return 0;
        }
    }
    return 1;
}

/* ---------------- grid LOS smoothing (baseline, small maps) ---------------- */
static int line_of_sight(const Grid* g, Pt a, Pt b) {
    int steps = abs(b.x - a.x) + abs(b.y - a.y);
    if (steps == 0) return 1;
    Pt prev = a;
    for (int i = 1; i <= steps; i++) {
        double t = (double)i / steps;
        Pt cur;
        cur.x = (int)lround(a.x + (b.x - a.x) * t);
        cur.y = (int)lround(a.y + (b.y - a.y) * t);
        if (cur.x == prev.x && cur.y == prev.y) continue;
        if (!step_ok(g, prev, cur)) return 0;
        prev = cur;
    }
    return prev.x == b.x && prev.y == b.y;
}

static int smooth(const Grid* g, const Pt* in, int n, Pt* out, int max_out) {
    if (n <= 0) return 0;
    int m = 0;
    out[m++] = in[0];
    int anchor = 0;
    while (anchor < n - 1) {
        int next = anchor + 1;
        for (int j = n - 1; j > anchor; j--) {
            if (line_of_sight(g, in[anchor], in[j])) { next = j; break; }
        }
        if (m >= max_out) return -1;
        out[m++] = in[next];
        anchor = next;
    }
    return m;
}

/* ---------------- tests ---------------- */
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned rng_state = 1;
static unsigned rnd(void) { rng_state = rng_state * 1664525u + 1013904223u; return rng_state; }

static Grid* grid_random(int w, int h, int block_pct, Pt s, Pt t) {
    Grid* g = (Grid*)malloc(sizeof(Grid));
    g->w = w; g->h = h;
    g->blocked = (unsigned char*)malloc((size_t)w * h);
    for (int i = 0; i < w * h; i++) g->blocked[i] = (rnd() % 100) < (unsigned)block_pct;
    g->blocked[s.y * w + s.x] = 0;
    g->blocked[t.y * w + t.x] = 0;
    return g;
}

int main(void) {
    /* demo map (same as navgrid) */
    const char* rows[] = {
        "S....#....",
        ".##..#....",
        ".#......#.",
        ".#.####.#.",
        ".....#..G.",
    };
    Grid demo; demo.w = 10; demo.h = 5;
    unsigned char demo_cells[50];
    demo.blocked = demo_cells;
    for (int y = 0; y < 5; y++)
        for (int x = 0; x < 10; x++)
            demo_cells[y * 10 + x] = (rows[y][x] == '#');
    Pt s = {0, 0}, t = {8, 4};

    NavMesh nm;
    navmesh_build(&nm, &demo);
    printf("demo: cells=%d walkable=%d rects=%d (%.1f%% of walkable)\n",
           demo.w * demo.h, nm.walkable_cells, nm.nrects,
           100.0 * nm.nrects / nm.walkable_cells);

    Pt raw[512], sm[512];
    int gn = grid_astar(&demo, s, t, raw, 512);
    int gm = smooth(&demo, raw, gn, sm, 512);
    V2 wp[512];
    int corridor[512], clen = 0;
    int fn = navmesh_path(&nm, &demo, s, t, wp, 512, corridor, &clen);
    double glen = pts_length(raw, gn), slen = pts_length(sm, gm), flen = v2_length(wp, fn);
    printf("grid_raw=%d len=%.4f grid_smooth=%d len=%.4f navmesh=%d len=%.4f\n",
           gn, glen, gm, slen, fn, flen);
    printf("waypoints:");
    for (int i = 0; i < fn; i++) printf(" (%.1f,%.1f)", wp[i].x, wp[i].y);
    printf("\n");
    check("demo_found", gn > 0 && fn >= 2);
    check("demo_merged", nm.nrects > 0 && nm.nrects < nm.walkable_cells);
    check("demo_not_worse_than_raw", flen <= glen + 1e-9);
    check("demo_not_worse_than_smooth", flen <= slen + 1e-9);
    check("demo_in_corridor", path_in_corridor(&nm, corridor, clen, wp, fn));
    navmesh_free(&nm);

    /* open field: funnel must give a single straight segment */
    {
        Grid g; g.w = 8; g.h = 8;
        unsigned char cells[64]; memset(cells, 0, sizeof(cells));
        g.blocked = cells;
        NavMesh m; navmesh_build(&m, &g);
        V2 p[72]; int corr[72], cl = 0;
        int n = navmesh_path(&m, &g, (Pt){0,0}, (Pt){7,7}, p, 72, corr, &cl);
        check("open_one_rect", m.nrects == 1);
        check("open_straight", n == 2);
        check("open_length", n == 2 && fabs(v2_length(p, n) - hypot(7.0, 7.0)) < 1e-9);
        navmesh_free(&m);
    }

    /* random maps: navmesh length must not be worse than grid A* */
    {
        int compared = 0, skipped = 0, in_corr_bad = 0, worse = 0;
        double sum_ratio = 0.0, sum_rect_ratio = 0.0;
        rng_state = 12345;
        for (int seed = 0; seed < 500; seed++) {
            Pt a = {0, 0}, b = {63, 63};
            Grid* g = grid_random(64, 64, 30, a, b);
            Pt* graw = (Pt*)malloc(sizeof(Pt) * 4096 * 4);
            int n = grid_astar(g, a, b, graw, 4096 * 4);
            if (n < 0) { skipped++; free(graw); free(g->blocked); free(g); continue; }
            NavMesh m; navmesh_build(&m, g);
            V2* pts = (V2*)malloc(sizeof(V2) * (size_t)(g->w * g->h + 8));
            int* corr = (int*)malloc(sizeof(int) * (size_t)(g->w * g->h + 8));
            int cl = 0;
            int fn2 = navmesh_path(&m, g, a, b, pts, g->w * g->h + 8, corr, &cl);
            double gl = pts_length(graw, n), fl = fn2 > 0 ? v2_length(pts, fn2) : 1e30;
            if (fn2 < 0 || fl > gl + 1e-9) worse++;
            if (fn2 > 0 && !path_in_corridor(&m, corr, cl, pts, fn2)) in_corr_bad++;
            sum_ratio += fl / gl;
            sum_rect_ratio += (double)m.nrects / m.walkable_cells;
            compared++;
            free(corr); free(pts); free(graw);
            navmesh_free(&m);
            free(g->blocked); free(g);
        }
        printf("random: compared=%d skipped=%d avg_len_ratio=%.4f avg_rects=%.1f%% of walkable\n",
               compared, skipped, sum_ratio / compared, 100.0 * sum_rect_ratio / compared);
        check("random_not_worse", worse == 0);
        check("random_in_corridor", in_corr_bad == 0);
    }

    /* 100k cells: build must stay under 2 s */
    {
        Pt a = {0, 0}, b = {319, 319};
        rng_state = 999;
        Grid* g = NULL;
        Pt* graw = (Pt*)malloc(sizeof(Pt) * 320 * 320);
        int n = -1;
        for (int tries = 0; tries < 100 && n < 0; tries++) { /* find a connected map */
            if (g) { free(g->blocked); free(g); }
            g = grid_random(320, 320, 30, a, b);
            n = grid_astar(g, a, b, graw, 320 * 320);
        }
        clock_t t0 = clock();
        NavMesh m; navmesh_build(&m, g);
        double build_ms = 1000.0 * (clock() - t0) / CLOCKS_PER_SEC;
        printf("big: cells=%d walkable=%d rects=%d (%.1f%% of walkable) build_ms=%.1f\n",
               g->w * g->h, m.walkable_cells, m.nrects,
               100.0 * m.nrects / m.walkable_cells, build_ms);
        check("big_build_under_2s", build_ms < 2000.0);
        t0 = clock();
        V2* pts = (V2*)malloc(sizeof(V2) * (size_t)(g->w * g->h + 8));
        int* corr = (int*)malloc(sizeof(int) * (size_t)(g->w * g->h + 8));
        int cl = 0;
        int fn2 = navmesh_path(&m, g, a, b, pts, g->w * g->h + 8, corr, &cl);
        double query_ms = 1000.0 * (clock() - t0) / CLOCKS_PER_SEC;
        double gl = pts_length(graw, n), fl = v2_length(pts, fn2);
        printf("big_path: grid=%d len=%.4f navmesh=%d len=%.4f query_ms=%.2f\n", n, gl, fn2, fl, query_ms);
        check("big_found", n > 0 && fn2 > 0);
        check("big_not_worse", fl <= gl + 1e-9);
        check("big_in_corridor", path_in_corridor(&m, corr, cl, pts, fn2));
        free(corr); free(pts); free(graw);
        navmesh_free(&m);
        free(g->blocked); free(g);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
