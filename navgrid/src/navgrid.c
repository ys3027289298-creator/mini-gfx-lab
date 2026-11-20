
/* mini-navgrid: 8-direction A* on an ASCII grid, with corner-cut prevention,
   path smoothing by line-of-sight, and hierarchical pathfinding (HPA*).

   Two planners share the same binary-heap A* core:
   - astar_rect(): flat full-map A* (baseline), or a cluster-bounded local A*
   - hpa_build()/hpa_query(): coarse abstract graph over rectangular clusters,
     border entrances as connection points, abstract A* then per-edge refinement. */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <time.h>

#define MAX_W 1024
#define MAX_H 1024
#define MAX_NODES (MAX_W * MAX_H)
#define INF 1e18

typedef struct { int w, h; unsigned char blocked[MAX_H][MAX_W]; } Grid;

typedef struct { int x, y; } Pt;

typedef struct { int x0, y0, x1, y1; } Rect;

static int grid_load(Grid* g, const char* const* rows, int h) {
    int w = (int)strlen(rows[0]);
    if (w <= 0 || w > MAX_W || h <= 0 || h > MAX_H) return -1;
    g->w = w; g->h = h;
    for (int y = 0; y < h; y++) {
        if ((int)strlen(rows[y]) != w) return -1;
        for (int x = 0; x < w; x++) g->blocked[y][x] = (rows[y][x] == '#');
    }
    return 0;
}

static int in_bounds(const Grid* g, int x, int y) {
    return x >= 0 && y >= 0 && x < g->w && y < g->h;
}
static int walkable(const Grid* g, int x, int y) {
    return in_bounds(g, x, y) && !g->blocked[y][x];
}

/* diagonal step p->q is legal only if both orthogonal neighbours are free */
static int step_ok(const Grid* g, Pt p, Pt q) {
    int dx = q.x - p.x, dy = q.y - p.y;
    if (q.x < 0 || q.y < 0 || q.x >= g->w || q.y >= g->h) return 0;
    if (g->blocked[q.y][q.x]) return 0;
    if (dx != 0 && dy != 0) {
        if (g->blocked[p.y][q.x] || g->blocked[q.y][p.x]) return 0;
    }
    return 1;
}

static const int DIRS[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

/* ---------------- shared A* core with a binary heap ---------------- */

typedef struct { double g, f; int parent; unsigned char closed; } SNode;

static void heap_push(int* heap, int* hn, const SNode* nd, int id) {
    int i = (*hn)++;
    heap[i] = id;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (nd[heap[p]].f <= nd[heap[i]].f) break;
        int tmp = heap[p]; heap[p] = heap[i]; heap[i] = tmp;
        i = p;
    }
}

static int heap_pop(int* heap, int* hn, const SNode* nd) {
    int top = heap[0];
    heap[0] = heap[--(*hn)];
    int i = 0;
    for (;;) {
        int l = 2 * i + 1, r = l + 1, best = i;
        if (l < *hn && nd[heap[l]].f < nd[heap[best]].f) best = l;
        if (r < *hn && nd[heap[r]].f < nd[heap[best]].f) best = r;
        if (best == i) break;
        int tmp = heap[best]; heap[best] = heap[i]; heap[i] = tmp;
        i = best;
    }
    return top;
}

/* A* restricted to rectangle r (the whole grid when r covers it). Buffers are
   supplied by the caller so repeated local searches do not reallocate. */
static int astar_rect(const Grid* g, Pt s, Pt t, Rect r,
                      SNode* nd, int* heap, Pt* out, int max_out, int* expanded) {
    if (!walkable(g, s.x, s.y) || !walkable(g, t.x, t.y)) return -1;
    if (s.x < r.x0 || s.x > r.x1 || s.y < r.y0 || s.y > r.y1) return -1;
    if (t.x < r.x0 || t.x > r.x1 || t.y < r.y0 || t.y > r.y1) return -1;
    for (int y = r.y0; y <= r.y1; y++)
        for (int x = r.x0; x <= r.x1; x++) {
            int i = y * g->w + x;
            nd[i].g = nd[i].f = INF; nd[i].parent = -1; nd[i].closed = 0;
        }
    int hn = 0, ncells = g->w * g->h;
    int si = s.y * g->w + s.x, ti = t.y * g->w + t.x;
    nd[si].g = 0.0;
    nd[si].f = hypot((double)(t.x - s.x), (double)(t.y - s.y));
    heap_push(heap, &hn, nd, si);
    int found = 0, pops = 0;
    while (hn > 0) {
        int cur = heap_pop(heap, &hn, nd);
        if (nd[cur].closed) continue;
        nd[cur].closed = 1;
        pops++;
        if (cur == ti) { found = 1; break; }
        Pt cp = { cur % g->w, cur / g->w };
        for (int d = 0; d < 8; d++) {
            Pt np = { cp.x + DIRS[d][0], cp.y + DIRS[d][1] };
            if (np.x < r.x0 || np.x > r.x1 || np.y < r.y0 || np.y > r.y1) continue;
            if (!step_ok(g, cp, np)) continue;
            int ni = np.y * g->w + np.x;
            if (nd[ni].closed) continue;
            double step = (DIRS[d][0] && DIRS[d][1]) ? 1.4142135623730951 : 1.0;
            double ng = nd[cur].g + step;
            if (ng < nd[ni].g) {
                nd[ni].g = ng;
                nd[ni].f = ng + hypot((double)(t.x - np.x), (double)(t.y - np.y));
                nd[ni].parent = cur;
                heap_push(heap, &hn, nd, ni);
            }
        }
    }
    int rc = -1;
    if (found) {
        int n = 0, cur = ti;
        while (cur != -1) n++, cur = nd[cur].parent;
        if (n <= max_out) {
            cur = ti;
            for (int i = n - 1; i >= 0; i--) {
                out[i].x = cur % g->w; out[i].y = cur / g->w;
                cur = nd[cur].parent;
            }
            rc = n;
        }
    }
    if (expanded) *expanded += pops;
    (void)ncells;
    return rc;
}

/* Grid supercover by integer DDA: every cell the segment a->b crosses is
   visited (Bresenham variant that takes both axes when the line crosses a
   grid corner). A diagonal step is accepted only when both orthogonal
   neighbours are open — the exact rule step_ok applies to A* moves, so a
   LOS shortcut can never clip a wall corner. */
static int line_of_sight(const Grid* g, Pt a, Pt b) {
    int x = a.x, y = a.y;
    int adx = abs(b.x - a.x), ady = abs(b.y - a.y);
    int sx = b.x > a.x ? 1 : (b.x < a.x ? -1 : 0);
    int sy = b.y > a.y ? 1 : (b.y < a.y ? -1 : 0);
    if (adx == 0 && ady == 0) return walkable(g, x, y);
    int err = adx - ady;   /* Bresenham accumulator */
    while (x != b.x || y != b.y) {
        int nx = x, ny = y;
        int e2 = 2 * err;
        int take_x = adx != 0 && e2 >= -adx;   /* line has reached next x line */
        int take_y = ady != 0 && e2 <= adx;    /* line has reached next y line */
        if (take_x) { nx += sx; err -= ady; }
        if (take_y) { ny += sy; err += adx; }
        if (!in_bounds(g, nx, ny) || g->blocked[ny][nx]) return 0;
        /* diagonal DDA step: both orthogonal neighbours must be open,
           exactly the corner-cut rule step_ok applies to A* moves */
        if (nx != x && ny != y && (g->blocked[y][nx] || g->blocked[ny][x])) return 0;
        x = nx; y = ny;
    }
    return 1;
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

static double path_length(const Pt* p, int n) {
    double s = 0.0;
    for (int i = 1; i < n; i++) s += hypot((double)(p[i].x - p[i-1].x), (double)(p[i].y - p[i-1].y));
    return s;
}

/* every cell walkable and every consecutive move legal. Cell-to-cell paths
   (raw A* output) are checked with the single-step rule step_ok; waypoint
   paths (smoothed output, whose segments may span many cells) are checked by
   the same rule through the supercover line_of_sight. */
static int path_valid(const Grid* g, Pt s, Pt t, const Pt* p, int n, int smoothed) {
    if (n < 2 || p[0].x != s.x || p[0].y != s.y || p[n-1].x != t.x || p[n-1].y != t.y) return 0;
    for (int i = 0; i < n; i++)
        if (!walkable(g, p[i].x, p[i].y)) return 0;
    for (int i = 1; i < n; i++) {
        if (smoothed) { if (!line_of_sight(g, p[i-1], p[i])) return 0; }
        else if (!step_ok(g, p[i-1], p[i])) return 0;
    }
    return 1;
}

/* ---------------- hierarchical planner: abstract graph ---------------- */

typedef struct { Pt p; int cluster; } AbsNode;
typedef struct { int to; double w; } AEdge;
typedef struct { AEdge* e; int n, cap; } EdgeList;
typedef struct { int* id; int n, cap; } IdList;

typedef struct {
    int w, h, cs, nclx, ncly;
    AbsNode* nodes; int nn, capn;
    EdgeList* adj;
    IdList* clusters;          /* abstract node ids per cluster */
    int* cell2abs;             /* grid cell -> abstract node id, -1 if none */
    SNode* scratch_nd;         /* w*h reusable local-A* buffers */
    int* scratch_heap;
    Pt* scratch_path;
} AbsGraph;

static void* xrealloc(void* p, size_t n) {
    void* q = realloc(p, n);
    if (!q) { perror("realloc"); exit(1); }
    return q;
}

static void id_push(IdList* l, int v) {
    if (l->n == l->cap) { l->cap = l->cap ? l->cap * 2 : 8; l->id = xrealloc(l->id, sizeof(int) * (size_t)l->cap); }
    l->id[l->n++] = v;
}

static void edge_push(EdgeList* l, int to, double w) {
    if (l->n == l->cap) { l->cap = l->cap ? l->cap * 2 : 8; l->e = xrealloc(l->e, sizeof(AEdge) * (size_t)l->cap); }
    l->e[l->n++] = (AEdge){ to, w };
}

static Rect cluster_rect(const AbsGraph* G, int c) {
    int cx = c % G->nclx, cy = c / G->nclx;
    Rect r = { cx * G->cs, cy * G->cs,
               (cx + 1) * G->cs - 1 < G->w ? (cx + 1) * G->cs - 1 : G->w - 1,
               (cy + 1) * G->cs - 1 < G->h ? (cy + 1) * G->cs - 1 : G->h - 1 };
    return r;
}

static int abs_get_or_create(AbsGraph* G, Pt p) {
    int cell = p.y * G->w + p.x;
    if (G->cell2abs[cell] >= 0) return G->cell2abs[cell];
    if (G->nn == G->capn) {
        G->capn = G->capn ? G->capn * 2 : 256;
        G->nodes = xrealloc(G->nodes, sizeof(AbsNode) * (size_t)G->capn);
        G->adj = xrealloc(G->adj, sizeof(EdgeList) * (size_t)G->capn);
        memset(&G->adj[G->nn], 0, sizeof(EdgeList) * (size_t)(G->capn - G->nn));
    }
    int id = G->nn++;
    int cx = p.x / G->cs, cy = p.y / G->cs, c = cy * G->nclx + cx;
    G->nodes[id] = (AbsNode){ p, c };
    G->cell2abs[cell] = id;
    id_push(&G->clusters[c], id);
    return id;
}

/* one maximal walkable run straddling a cluster border: abstract nodes at both
   ends (one per side), linked by a cost-1 transition edge */
static void add_entrance(AbsGraph* G, Pt a, Pt b) {
    int u = abs_get_or_create(G, a), v = abs_get_or_create(G, b);
    edge_push(&G->adj[u], v, 1.0);
    edge_push(&G->adj[v], u, 1.0);
}

/* Single-source, multi-target Dijkstra restricted to one cluster. Explores
   the whole reachable rectangle once; the settled g-values are exact shortest
   distances from src to every cell, hence to every other entrance. Adds one
   undirected abstract edge per reachable entrance (only the src->other half
   is added here; callers invoke this for every entrance in turn). */
static int astar_multi(const Grid* g, Pt src, Rect r, AbsGraph* G,
                       const int* ids, int nids, int src_slot,
                       SNode* nd, int* heap) {
    for (int y = r.y0; y <= r.y1; y++)
        for (int x = r.x0; x <= r.x1; x++) {
            int i = y * g->w + x;
            nd[i].g = nd[i].f = INF; nd[i].parent = -1; nd[i].closed = 0;
        }
    int hn = 0;
    int si = src.y * g->w + src.x;
    nd[si].g = 0.0; nd[si].f = 0.0;
    heap_push(heap, &hn, nd, si);
    while (hn > 0) {
        int cur = heap_pop(heap, &hn, nd);
        if (nd[cur].closed) continue;
        nd[cur].closed = 1;
        Pt cp = { cur % g->w, cur / g->w };
        for (int d = 0; d < 8; d++) {
            Pt np = { cp.x + DIRS[d][0], cp.y + DIRS[d][1] };
            if (np.x < r.x0 || np.x > r.x1 || np.y < r.y0 || np.y > r.y1) continue;
            if (!step_ok(g, cp, np)) continue;
            int ni = np.y * g->w + np.x;
            if (nd[ni].closed) continue;
            double step = (DIRS[d][0] && DIRS[d][1]) ? 1.4142135623730951 : 1.0;
            double ng = nd[cur].g + step;
            if (ng < nd[ni].g) {
                nd[ni].g = ng; nd[ni].f = ng;   /* uniform Dijkstra */
                nd[ni].parent = cur;
                heap_push(heap, &hn, nd, ni);
            }
        }
    }
    int u = ids[src_slot];
    for (int k = 0; k < nids; k++) {
        if (k == src_slot) continue;
        int v = ids[k];
        double d = nd[G->nodes[v].p.y * g->w + G->nodes[v].p.x].g;
        if (d < INF) edge_push(&G->adj[u], v, d);
    }
    return 0;
}

static int hpa_build(AbsGraph* G, const Grid* g, int cs) {
    memset(G, 0, sizeof(*G));
    G->w = g->w; G->h = g->h; G->cs = cs;
    G->nclx = (g->w + cs - 1) / cs;
    G->ncly = (g->h + cs - 1) / cs;
    int ncells = g->w * g->h;
    G->cell2abs = xrealloc(NULL, sizeof(int) * (size_t)ncells);
    for (int i = 0; i < ncells; i++) G->cell2abs[i] = -1;
    int nc = G->nclx * G->ncly;
    G->clusters = xrealloc(NULL, sizeof(IdList) * (size_t)nc);
    memset(G->clusters, 0, sizeof(IdList) * (size_t)nc);
    G->scratch_nd = xrealloc(NULL, sizeof(SNode) * (size_t)ncells);
    G->scratch_heap = xrealloc(NULL, sizeof(int) * (size_t)ncells);
    G->scratch_path = xrealloc(NULL, sizeof(Pt) * (size_t)ncells);

    /* vertical borders: between cluster cx and cx+1, for each cluster row cy */
    for (int cx = 0; cx < G->nclx - 1; cx++) {
        int xL = (cx + 1) * cs - 1, xR = xL + 1;
        for (int cy = 0; cy < G->ncly; cy++) {
            int y0 = cy * cs, y1 = (cy + 1) * cs - 1 < g->h ? (cy + 1) * cs - 1 : g->h - 1;
            int y = y0;
            while (y <= y1) {
                if (walkable(g, xL, y) && walkable(g, xR, y)) {
                    int a = y;
                    while (y + 1 <= y1 && walkable(g, xL, y + 1) && walkable(g, xR, y + 1)) y++;
                    int b = y;
                    add_entrance(G, (Pt){ xL, a }, (Pt){ xR, a });
                    if (b > a) add_entrance(G, (Pt){ xL, b }, (Pt){ xR, b });
                }
                y++;
            }
        }
    }
    /* horizontal borders: between cluster cy and cy+1 */
    for (int cy = 0; cy < G->ncly - 1; cy++) {
        int yT = (cy + 1) * cs - 1, yB = yT + 1;
        for (int cx = 0; cx < G->nclx; cx++) {
            int x0 = cx * cs, x1 = (cx + 1) * cs - 1 < g->w ? (cx + 1) * cs - 1 : g->w - 1;
            int x = x0;
            while (x <= x1) {
                if (walkable(g, x, yT) && walkable(g, x, yB)) {
                    int a = x;
                    while (x + 1 <= x1 && walkable(g, x + 1, yT) && walkable(g, x + 1, yB)) x++;
                    int b = x;
                    add_entrance(G, (Pt){ a, yT }, (Pt){ a, yB });
                    if (b > a) add_entrance(G, (Pt){ b, yT }, (Pt){ b, yB });
                }
                x++;
            }
        }
    }

    /* intra-cluster edges: one multi-target A* per entrance yields its exact
       distance to every other entrance in the cluster — O(k) searches instead
       of O(k^2) pairwise ones, over the same rectangle. */
    for (int c = 0; c < nc; c++) {
        Rect r = cluster_rect(G, c);
        IdList ids = G->clusters[c];
        for (int i = 0; i < ids.n; i++) {
            int u = ids.id[i];
            if (astar_multi(g, G->nodes[u].p, r, G, ids.id, ids.n, i,
                            G->scratch_nd, G->scratch_heap) != 0) return -1;
        }
    }
    return 0;
}

static void hpa_free(AbsGraph* G) {
    for (int i = 0; i < G->nn; i++) free(G->adj[i].e);
    free(G->adj);
    free(G->nodes);
    for (int i = 0; i < G->nclx * G->ncly; i++) free(G->clusters[i].id);
    free(G->clusters);
    free(G->cell2abs);
    free(G->scratch_nd);
    free(G->scratch_heap);
    free(G->scratch_path);
    memset(G, 0, sizeof(*G));
}

typedef struct { int exp_abs; int exp_local; } HpaStats;

/* min-heap keyed on a parallel key array; grows on demand because lazy
   deletion may insert the same node more than once */
static void aheap_push(int** heap, int* hn, int* cap, const double* key, int id) {
    if (*hn == *cap) {
        *cap = *cap ? *cap * 2 : 16;
        *heap = xrealloc(*heap, sizeof(int) * (size_t)*cap);
    }
    int i = (*hn)++;
    (*heap)[i] = id;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (key[(*heap)[p]] <= key[(*heap)[i]]) break;
        int tmp = (*heap)[p]; (*heap)[p] = (*heap)[i]; (*heap)[i] = tmp;
        i = p;
    }
}

static int aheap_pop(int* heap, int* hn, const double* key) {
    int top = heap[0];
    heap[0] = heap[--(*hn)];
    int i = 0;
    for (;;) {
        int l = 2 * i + 1, r = l + 1, best = i;
        if (l < *hn && key[heap[l]] < key[heap[best]]) best = l;
        if (r < *hn && key[heap[r]] < key[heap[best]]) best = r;
        if (best == i) break;
        int tmp = heap[best]; heap[best] = heap[i]; heap[i] = tmp;
        i = best;
    }
    return top;
}

/* abstract A* over the graph plus two query-only virtual nodes (start, goal) */
static int hpa_query(const AbsGraph* G, const Grid* g, Pt s, Pt t,
                     Pt* out, int max_out, HpaStats* st) {
    if (st) { st->exp_abs = 0; st->exp_local = 0; }
    if (!walkable(g, s.x, s.y) || !walkable(g, t.x, t.y)) return -1;
    int nn = G->nn, VS = nn, VT = nn + 1, total = nn + 2;
    int csx = s.x / G->cs, csy = s.y / G->cs, c_s = csy * G->nclx + csx;
    int ctx = t.x / G->cs, cty = t.y / G->cs, c_t = cty * G->nclx + ctx;
    Rect rs = cluster_rect(G, c_s), rt = cluster_rect(G, c_t);
    int ncells = G->w * G->h;

    /* virtual edges: local A* from s/t to every entrance of its own cluster.
       s-edges are relaxed out of VS; t-edges are reversed (entrance->VT) so
       the goal node is only ever reached through relaxation, never expanded. */
    IdList sn = G->clusters[c_s], tn = G->clusters[c_t];
    AEdge* ves = xrealloc(NULL, sizeof(AEdge) * (size_t)(sn.n + 1));
    AEdge* vet = xrealloc(NULL, sizeof(AEdge) * (size_t)(tn.n + 1));
    int nves = 0, nvet = 0;
    for (int i = 0; i < sn.n; i++) {
        int v = sn.id[i];
        int n = astar_rect(g, s, G->nodes[v].p, rs, G->scratch_nd,
                           G->scratch_heap, G->scratch_path, ncells, st ? &st->exp_local : NULL);
        if (n > 0) ves[nves++] = (AEdge){ v, path_length(G->scratch_path, n) };
    }
    for (int i = 0; i < tn.n; i++) {
        int v = tn.id[i];
        int n = astar_rect(g, t, G->nodes[v].p, rt, G->scratch_nd,
                           G->scratch_heap, G->scratch_path, ncells, st ? &st->exp_local : NULL);
        if (n > 0) vet[nvet++] = (AEdge){ v, path_length(G->scratch_path, n) };
    }
    double direct_w = INF;   /* s->t inside one cluster, if reachable */
    if (c_s == c_t) {
        int n = astar_rect(g, s, t, rs, G->scratch_nd, G->scratch_heap,
                           G->scratch_path, ncells, st ? &st->exp_local : NULL);
        if (n > 0) direct_w = path_length(G->scratch_path, n);
    }

    double* ag = xrealloc(NULL, sizeof(double) * (size_t)total);
    double* af = xrealloc(NULL, sizeof(double) * (size_t)total);
    int* apar = xrealloc(NULL, sizeof(int) * (size_t)total);
    unsigned char* aclosed = xrealloc(NULL, (size_t)total);
    int* aheap = NULL; int aheap_cap = 0;
    for (int i = 0; i < total; i++) { ag[i] = af[i] = INF; apar[i] = -1; aclosed[i] = 0; }
    int ahn = 0;
    ag[VS] = 0.0; af[VS] = hypot((double)(t.x - s.x), (double)(t.y - s.y));
    aheap_push(&aheap, &ahn, &aheap_cap, af, VS);
    int found = 0;
    while (ahn > 0) {
        int cur = aheap_pop(aheap, &ahn, af);
        if (aclosed[cur]) continue;
        aclosed[cur] = 1;
        if (st) st->exp_abs++;
        if (cur == VT) { found = 1; break; }
        if (cur == VS) {
            for (int k = 0; k < nves; k++) {
                int to = ves[k].to;
                double ng = ag[VS] + ves[k].w;
                if (!aclosed[to] && ng < ag[to]) {
                    ag[to] = ng;
                    af[to] = ng + hypot((double)(t.x - G->nodes[to].p.x), (double)(t.y - G->nodes[to].p.y));
                    apar[to] = VS;
                    aheap_push(&aheap, &ahn, &aheap_cap, af, to);
                }
            }
            if (direct_w < INF && ag[VS] + direct_w < ag[VT]) {
                ag[VT] = ag[VS] + direct_w;
                af[VT] = ag[VT];
                apar[VT] = VS;
                aheap_push(&aheap, &ahn, &aheap_cap, af, VT);
            }
            continue;
        }
        /* real abstract node: graph edges, then reverse t-edges into VT */
        for (int k = 0; k < G->adj[cur].n; k++) {
            int to = G->adj[cur].e[k].to;
            if (aclosed[to]) continue;
            Pt pn = G->nodes[to].p;
            double ng = ag[cur] + G->adj[cur].e[k].w;
            if (ng < ag[to]) {
                ag[to] = ng;
                af[to] = ng + hypot((double)(t.x - pn.x), (double)(t.y - pn.y));
                apar[to] = cur;
                aheap_push(&aheap, &ahn, &aheap_cap, af, to);
            }
        }
        if (G->nodes[cur].cluster == c_t) {
            for (int k = 0; k < nvet; k++) {
                if (vet[k].to != cur) continue;
                double ng = ag[cur] + vet[k].w;
                if (ng < ag[VT]) {
                    ag[VT] = ng;
                    af[VT] = ng;
                    apar[VT] = cur;
                    aheap_push(&aheap, &ahn, &aheap_cap, af, VT);
                }
            }
        }
    }
    int rc = -1;
    if (found) {
        int chain_n = 0, cur = VT;
        while (cur != -1) { chain_n++; cur = apar[cur]; }
        int* chain = xrealloc(NULL, sizeof(int) * (size_t)chain_n);
        cur = VT;
        for (int i = chain_n - 1; i >= 0; i--) { chain[i] = cur; cur = apar[cur]; }

        /* refine: re-run bounded local A* for each abstract edge */
        int m = 0;
        if (m < max_out) out[m++] = s;
        for (int i = 0; i < chain_n - 1 && m > 0; i++) {
            int a = chain[i], b = chain[i + 1];
            Pt pa = a == VS ? s : (a == VT ? t : G->nodes[a].p);
            Pt pb = b == VS ? s : (b == VT ? t : G->nodes[b].p);
            int ca = a == VS ? c_s : (a == VT ? c_t : G->nodes[a].cluster);
            int cb = b == VS ? c_s : (b == VT ? c_t : G->nodes[b].cluster);
            if (a != VS && a != VT && b != VS && b != VT && ca != cb) {
                /* entrance transition: adjacent cells, one legal step */
                if (m < max_out) out[m++] = pb;
                continue;
            }
            Rect r = cluster_rect(G, ca);
            int n = astar_rect(g, pa, pb, r, G->scratch_nd, G->scratch_heap,
                               G->scratch_path, ncells, st ? &st->exp_local : NULL);
            if (n <= 0) { m = -1; break; }
            for (int k = 1; k < n; k++) {
                if (m >= max_out) { m = -1; break; }
                out[m++] = G->scratch_path[k];
            }
        }
        rc = m;
        free(chain);
    }
    free(ves); free(vet); free(ag); free(af); free(apar); free(aclosed); free(aheap);
    return rc;
}

/* ---------------- benchmark harness ---------------- */

static uint64_t rng_state = 0x123456789abcdef0ULL;
static uint64_t xs_next(void) {
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 7; rng_state ^= rng_state << 17;
    return rng_state;
}
static int rnd(int n) { return (int)(xs_next() % (uint64_t)n); }

static void gen_random(Grid* g, int w, int h, double density) {
    g->w = w; g->h = h;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            g->blocked[y][x] = (xs_next() % 1000) < (uint64_t)(density * 1000.0);
}

/* rooms carved out of solid rock plus L-shaped corridors: guaranteed connected */
static void gen_rooms(Grid* g, int w, int h, int rooms) {
    g->w = w; g->h = h;
    memset(g->blocked, 1, sizeof(g->blocked));
    Pt centers[64];
    for (int i = 0; i < rooms; i++) {
        int rw = 8 + rnd(20), rh = 8 + rnd(20);
        int x0 = rnd(w - rw - 1) + 1, y0 = rnd(h - rh - 1) + 1;
        for (int y = y0; y < y0 + rh && y < h - 1; y++)
            for (int x = x0; x < x0 + rw && x < w - 1; x++)
                g->blocked[y][x] = 0;
        centers[i] = (Pt){ x0 + rw / 2, y0 + rh / 2 };
    }
    for (int i = 1; i < rooms; i++) {
        Pt a = centers[i - 1], b = centers[i];
        int x0 = a.x < b.x ? a.x : b.x, x1 = a.x > b.x ? a.x : b.x;
        int y0 = a.y < b.y ? a.y : b.y, y1 = a.y > b.y ? a.y : b.y;
        for (int x = x0; x <= x1; x++) { g->blocked[a.y][x] = 0; g->blocked[b.y][x] = 0; }
        for (int y = y0; y <= y1; y++) { g->blocked[y][a.x] = 0; g->blocked[y][b.x] = 0; }
    }
}

static Pt random_walkable(const Grid* g) {
    for (;;) {
        Pt p = { rnd(g->w), rnd(g->h) };
        if (walkable(g, p.x, p.y)) return p;
    }
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

#define Q 20

typedef struct { int found; double raw_len, smooth_len, ms; int exp; } BaseResult;

int main(void) {
    /* ---- original small-map sanity demo ---- */
    const char* rows[] = {
        "S....#....",
        ".##..#....",
        ".#......#.",
        ".#.####.#.",
        ".....#..G.",
    };
    static Grid small;
    if (grid_load(&small, rows, 5) != 0) { printf("load fail\n"); return 1; }
    Pt s0 = {0,0}, t0 = {8,4};
    static SNode nd0[MAX_NODES];
    static int heap0[MAX_NODES];
    static Pt raw0[MAX_NODES], sm0[MAX_NODES];
    Rect full0 = { 0, 0, small.w - 1, small.h - 1 };
    int n0 = astar_rect(&small, s0, t0, full0, nd0, heap0, raw0, MAX_NODES, NULL);
    if (n0 < 0) { printf("no path\n"); return 1; }
    int m0 = smooth(&small, raw0, n0, sm0, MAX_NODES);
    printf("== sanity demo (5x10) ==\n");
    printf("raw=%d len=%.4f smooth=%d len=%.4f\n", n0, path_length(raw0, n0), m0, path_length(sm0, m0));
    for (int i = 0; i < m0; i++) printf("(%d,%d)%s", sm0[i].x, sm0[i].y, i+1==m0 ? "\n" : " ");
    int bad = 0;
    for (int i = 1; i < m0; i++) if (!line_of_sight(&small, sm0[i-1], sm0[i])) bad++;
    printf("bad_segments=%d\n", bad);
    printf("corner_cut=%d\n\n", !step_ok(&small, (Pt){1,0}, (Pt){2,1}));

    /* ---- large-map benchmark ---- */
    const int W = 512, H = 512;
    const int TIERS[3] = { 16, 32, 64 };       /* cluster size            */
    Grid* map = xrealloc(NULL, sizeof(Grid));  /* 1 MB blocked array      */
    const char* map_names[2] = { "random p=0.25", "rooms+corridors" };

    SNode* nd = xrealloc(NULL, sizeof(SNode) * (size_t)W * H);
    int* hp = xrealloc(NULL, sizeof(int) * (size_t)W * H);
    Pt* raw = xrealloc(NULL, sizeof(Pt) * (size_t)W * H);
    Pt* sm = xrealloc(NULL, sizeof(Pt) * (size_t)W * H);
    Pt* hraw = xrealloc(NULL, sizeof(Pt) * (size_t)W * H);
    Pt* hsm = xrealloc(NULL, sizeof(Pt) * (size_t)W * H);

    for (int mi = 0; mi < 2; mi++) {
        rng_state = 0x123456789abcdef0ULL;
        if (mi == 0) gen_random(map, W, H, 0.25);
        else gen_rooms(map, W, H, 40);
        Rect full = { 0, 0, W - 1, H - 1 };

        Pt ss[Q], tt[Q];
        for (int q = 0; q < Q; q++) { ss[q] = random_walkable(map); tt[q] = random_walkable(map); }

        BaseResult base[Q];
        memset(base, 0, sizeof(base));
        double t_start = now_ms();
        for (int q = 0; q < Q; q++) {
            double a = now_ms();
            int n = astar_rect(map, ss[q], tt[q], full, nd, hp, raw, W * H, &base[q].exp);
            double b = now_ms();
            base[q].ms = b - a;
            base[q].found = n > 0;
            if (n > 0) {
                base[q].raw_len = path_length(raw, n);
                int m = smooth(map, raw, n, sm, W * H);
                base[q].smooth_len = path_length(sm, m);
            } else base[q].raw_len = base[q].smooth_len = 0.0;
        }
        double base_total = now_ms() - t_start;
        int reachable = 0;
        double base_len_avg = 0.0, exp_avg = 0.0;
        for (int q = 0; q < Q; q++)
            if (base[q].found) { reachable++; base_len_avg += base[q].smooth_len; exp_avg += base[q].exp; }
        base_len_avg /= reachable ? reachable : 1;
        exp_avg /= reachable ? reachable : 1;

        printf("== map: %s %dx%d, queries=%d (reachable=%d) ==\n", map_names[mi], W, H, Q, reachable);
        printf("baseline full A* : total %7.2f ms | avg %6.3f ms | exp_avg %8.1f | smooth_len avg %.2f\n",
               base_total, base_total / Q, exp_avg, base_len_avg);

        for (int ti = 0; ti < 3; ti++) {
            int cs = TIERS[ti];
            AbsGraph G;
            double tb0 = now_ms();
            hpa_build(&G, map, cs);
            double build_ms = now_ms() - tb0;

            int clip = 0, clip_raw = 0, mismatch = 0, valid_q = 0;
            double q_ms = 0.0, len_sum = 0.0, delta_sum = 0.0, raw_delta_sum = 0.0;
            double max_delta = 0.0, max_raw_delta = 0.0;
            long exp_abs_sum = 0, exp_loc_sum = 0;
            for (int q = 0; q < Q; q++) {
                HpaStats st;
                double a = now_ms();
                int hn = hpa_query(&G, map, ss[q], tt[q], hraw, W * H, &st);
                int hm = 0;
                if (hn > 0) hm = smooth(map, hraw, hn, hsm, W * H);
                double b = now_ms();
                if ((hn > 0) != base[q].found) { mismatch++; continue; }
                if (hn <= 0) continue;
                q_ms += b - a;
                valid_q++;
                exp_abs_sum += st.exp_abs;
                exp_loc_sum += st.exp_local;
                /* raw refined path obeys single-step rules; smoothed path is
                   a waypoint chain checked through supercover LOS */
                if (!path_valid(map, ss[q], tt[q], hraw, hn, 0)) clip_raw++;
                if (!path_valid(map, ss[q], tt[q], hsm, hm, 1)) clip++;
                double hl = path_length(hsm, hm), hrawl = path_length(hraw, hn);
                len_sum += hl;
                double d = (hl - base[q].smooth_len) / base[q].smooth_len * 100.0;
                double dr = (hrawl - base[q].raw_len) / base[q].raw_len * 100.0;
                delta_sum += d; raw_delta_sum += dr;
                if (d > max_delta) max_delta = d;
                if (dr > max_raw_delta) max_raw_delta = dr;
            }
            int regions = G.nclx * G.ncly;
            int pass = (mismatch == 0 && clip == 0 && clip_raw == 0 && max_delta <= 5.0);
            printf("HPA cs=%2d regions=%4d : build %7.2f ms | query avg %6.3f ms (%.2fx) | "
                   "exp_abs %6.1f exp_loc %8.1f | len avg %.2f | dsmooth avg %+.2f%% max %+.2f%% | "
                   "draw avg %+.2f%% max %+.2f%% | clip raw=%d smooth=%d %s\n",
                   cs, regions, build_ms,
                   q_ms / (valid_q ? valid_q : 1),
                   base_total / Q / (q_ms / (valid_q ? valid_q : 1)),
                   (double)exp_abs_sum / (valid_q ? valid_q : 1),
                   (double)exp_loc_sum / (valid_q ? valid_q : 1),
                   len_sum / (valid_q ? valid_q : 1),
                   delta_sum / (valid_q ? valid_q : 1), max_delta,
                   raw_delta_sum / (valid_q ? valid_q : 1), max_raw_delta,
                   clip_raw, clip, pass ? "PASS" : "FAIL");
            if (mismatch) printf("  WARNING reachability mismatch on %d queries\n", mismatch);
            hpa_free(&G);
        }
        printf("\n");
    }
    free(map); free(nd); free(hp); free(raw); free(sm); free(hraw); free(hsm);
    return 0;
}
