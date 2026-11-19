#define _POSIX_C_SOURCE 199309L

/* mini-navgrid: 8-direction A* on an ASCII grid, with corner-cut prevention,
   path smoothing by line-of-sight, and a topology check that reports
   connected components, cut-off regions, and the cells to unblock.

   Topology notes
   --------------
   With "no corner cutting", a diagonal step p->q is legal only when both
   orthogonal cells beside it are free. That means every legal diagonal edge
   already implies an orthogonal path p -> corner -> q. So walkability is
   exactly 4-connectivity, and component counting can use 4-neighbour BFS
   without changing the answer. The self-test cross-checks this against a
   union-find built from step_ok() edges. */

#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct { int w, h; unsigned char* blocked; } Grid;

typedef struct { int x, y; } Pt;

static int grid_init(Grid* g, int w, int h) {
    if (w <= 0 || h <= 0) return -1;
    if ((size_t)w > (size_t)INT_MAX / (size_t)h) return -1;
    g->w = w; g->h = h;
    g->blocked = (unsigned char*)calloc((size_t)w * (size_t)h, 1);
    if (!g->blocked) { g->w = g->h = 0; return -1; }
    return 0;
}

static void grid_free(Grid* g) {
    free(g->blocked);
    g->blocked = NULL;
    g->w = g->h = 0;
}

static int grid_load(Grid* g, const char* const* rows, int h) {
    if (h <= 0) return -1;
    int w = (int)strlen(rows[0]);
    if (w <= 0) return -1;
    for (int y = 0; y < h; y++)
        if ((int)strlen(rows[y]) != w) return -1;
    if (grid_init(g, w, h) != 0) return -1;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            g->blocked[y * w + x] = (rows[y][x] == '#');
    return 0;
}

static int in_bounds(const Grid* g, int x, int y) {
    return x >= 0 && y >= 0 && x < g->w && y < g->h;
}
static int walkable(const Grid* g, int x, int y) {
    return in_bounds(g, x, y) && !g->blocked[y * g->w + x];
}

/* diagonal step p->q is legal only if both orthogonal neighbours are free */
static int step_ok(const Grid* g, Pt p, Pt q) {
    int dx = q.x - p.x, dy = q.y - p.y;
    if (q.x < 0 || q.y < 0 || q.x >= g->w || q.y >= g->h) return 0;
    if (g->blocked[q.y * g->w + q.x]) return 0;
    if (dx != 0 && dy != 0) {
        if (g->blocked[p.y * g->w + q.x] || g->blocked[q.y * g->w + p.x]) return 0;
    }
    return 1;
}

static const int DIRS[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
static const int D4[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};

typedef struct { double g, f; int parent; unsigned char open, closed; } Node;

static int astar(const Grid* g, Pt s, Pt t, Pt* out, int max_out) {
    if (!walkable(g, s.x, s.y) || !walkable(g, t.x, t.y)) return -1;
    int n = g->w * g->h;
    Node* nodes = (Node*)calloc((size_t)n, sizeof(Node));
    int* heap = (int*)malloc(sizeof(int) * (size_t)n);
    if (!nodes || !heap) { free(nodes); free(heap); return -1; }
    int hn = 0;
    int si = s.y * g->w + s.x, ti = t.y * g->w + t.x;
    nodes[si].g = 0.0;
    nodes[si].f = hypot((double)(t.x - s.x), (double)(t.y - s.y));
    nodes[si].parent = -1;
    nodes[si].open = 1;
    heap[hn++] = si;
    int found = 0;
    while (hn > 0) {
        int best = 0;
        for (int i = 1; i < hn; i++) if (nodes[heap[i]].f < nodes[heap[best]].f) best = i;
        int cur = heap[best]; heap[best] = heap[--hn];
        if (nodes[cur].closed) continue;
        nodes[cur].closed = 1;
        if (cur == ti) { found = 1; break; }
        Pt cp = { cur % g->w, cur / g->w };
        for (int d = 0; d < 8; d++) {
            Pt np = { cp.x + DIRS[d][0], cp.y + DIRS[d][1] };
            if (!step_ok(g, cp, np)) continue;
            int ni = np.y * g->w + np.x;
            if (nodes[ni].closed) continue;
            double step = (DIRS[d][0] && DIRS[d][1]) ? 1.4142135623730951 : 1.0;
            double ng = nodes[cur].g + step;
            if (!nodes[ni].open || ng < nodes[ni].g) {
                nodes[ni].g = ng;
                nodes[ni].f = ng + hypot((double)(t.x - np.x), (double)(t.y - np.y));
                nodes[ni].parent = cur;
                nodes[ni].open = 1;
                heap[hn++] = ni;
            }
        }
    }
    int rc = -1;
    if (found) {
        int n = 0, cur = ti;
        while (cur != -1) n++, cur = nodes[cur].parent;
        if (n <= max_out) {
            cur = ti;
            for (int i = n - 1; i >= 0; i--) {
                out[i].x = cur % g->w; out[i].y = cur / g->w;
                cur = nodes[cur].parent;
            }
            rc = n;
        }
    }
    free(heap); free(nodes);
    return rc;
}

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

static double path_length(const Pt* p, int n) {
    double s = 0.0;
    for (int i = 1; i < n; i++) s += hypot((double)(p[i].x - p[i-1].x), (double)(p[i].y - p[i-1].y));
    return s;
}

/* ------------------------------------------------------------------ */
/* topology check                                                      */
/* ------------------------------------------------------------------ */

/* Label every walkable cell with a 4-connected component id (-1 = blocked).
   `sizes` is allocated and receives one entry per component. */
static int grid_components(const Grid* g, int* label, int** sizes_out, int* ncomp_out) {
    int n = g->w * g->h;
    int* queue = (int*)malloc(sizeof(int) * (size_t)n);
    int* sizes = NULL;
    int cap = 8, ncomp = 0;
    if (!queue) return -1;
    sizes = (int*)malloc(sizeof(int) * (size_t)cap);
    if (!sizes) { free(queue); return -1; }
    for (int i = 0; i < n; i++) label[i] = -1;
    for (int start = 0; start < n; start++) {
        if (g->blocked[start] || label[start] >= 0) continue;
        if (ncomp == cap) {
            int nc = cap * 2;
            int* ns = (int*)realloc(sizes, sizeof(int) * (size_t)nc);
            if (!ns) { free(sizes); free(queue); return -1; }
            sizes = ns; cap = nc;
        }
        int id = ncomp++;
        int head = 0, tail = 0, sz = 0;
        label[start] = id;
        queue[tail++] = start;
        while (head < tail) {
            int c = queue[head++];
            sz++;
            int x = c % g->w, y = c / g->w;
            for (int d = 0; d < 4; d++) {
                int nx = x + D4[d][0], ny = y + D4[d][1];
                if (!in_bounds(g, nx, ny)) continue;
                int ni = ny * g->w + nx;
                if (g->blocked[ni] || label[ni] >= 0) continue;
                label[ni] = id;
                queue[tail++] = ni;
            }
        }
        sizes[id] = sz;
    }
    free(queue);
    *sizes_out = sizes;
    *ncomp_out = ncomp;
    return 0;
}

typedef struct {
    int comp;
    int size;
    int seed;              /* linear index of a representative cell */
    int x0, y0, x1, y1;    /* bounding box */
} CutRegion;

typedef struct {
    int components;        /* total number of walkable components        */
    int main_comp;         /* component treated as reachable (-1 if none) */
    int main_size;
    int cut_regions;       /* components other than main_comp             */
    int cut_cells;         /* walkable cells inside those regions         */
    CutRegion* cut;        /* cut_regions entries, largest first          */
    int patch_cells;       /* unique blocked cells to open                */
    Pt* patch;             /* the suggested cells                         */
    int repair_ok;         /* suggestion makes the whole grid connected   */
} TopoReport;

static void topo_report_free(TopoReport* r) {
    free(r->cut);
    free(r->patch);
    memset(r, 0, sizeof(*r));
    r->main_comp = -1;
}

typedef struct { int cost, a, b, ca, cb; } Bridge;  /* cb = -1: direct */

static int bridge_push(Bridge** v, int* n, int* cap, int cost, int a, int b, int ca, int cb) {
    if (*n == *cap) {
        if (*cap > INT_MAX / 2) return -1;
        int nc = *cap ? *cap * 2 : 64;
        Bridge* nv = (Bridge*)realloc(*v, sizeof(Bridge) * (size_t)nc);
        if (!nv) return -1;
        *v = nv; *cap = nc;
    }
    (*v)[(*n)++] = (Bridge){ cost, a, b, ca, cb };
    return 0;
}

static int cmp_bridge(const void* pa, const void* pb) {
    const Bridge* a = (const Bridge*)pa;
    const Bridge* b = (const Bridge*)pb;
    if (a->cost != b->cost) return a->cost < b->cost ? -1 : 1;
    return 0;
}

static int cmp_cut(const void* pa, const void* pb) {
    const CutRegion* a = (const CutRegion*)pa;
    const CutRegion* b = (const CutRegion*)pb;
    if (a->size != b->size) return b->size - a->size;  /* largest first */
    return a->seed - b->seed;
}

static int dsu_find(int* p, int x) {
    while (p[x] != x) { p[x] = p[p[x]]; x = p[x]; }
    return x;
}

/* append a blocked-cell chain to the patch list, skipping cells already listed */
static int mark_chain(const int* parent, int cell, const Grid* g,
                      unsigned char* is_patch, TopoReport* r) {
    while (cell >= 0) {
        if (!is_patch[cell]) {
            if (r->patch_cells >= g->w * g->h) return -1;
            is_patch[cell] = 1;
            r->patch[r->patch_cells].x = cell % g->w;
            r->patch[r->patch_cells].y = cell / g->w;
            r->patch_cells++;
        }
        cell = parent[cell];
    }
    return 0;
}

/* Analyze walkability topology.

   `spawn` selects the reachable region: if it is a walkable cell, its
   component is "main"; pass {-1,-1} to use the largest component.
   Every other component is reported as cut off.

   Repair suggestion: multi-source BFS over blocked cells from all
   components gives each blocked cell the nearest component and the number
   of blocked cells needed to reach it. Adjacent blocked cells with
   different origins, and blocked cells touching another component, become
   bridge candidates. A minimum spanning tree over the component graph then
   picks the cheapest set of cells to open.

   Why that is minimal for the common case: BFS exactness means the two
   components nearest a blocked cell are at distance dist[i] and the chain
   to each is stored, so a candidate bridge is a real opening, and the
   cheapest candidate between a pair is the minimum cut separating them.
   With exactly two components the MST is that single pair, hence optimal.
   With three or more, connecting all of them is a Steiner-tree problem and
   the MST is an upper bound, except when the tree has no shared cells; the
   report's repair_ok field confirms the suggestion does reconnect, but it
   does not claim global optimality beyond the two-component case. */
static int topo_check(const Grid* g, Pt spawn, TopoReport* r) {
    memset(r, 0, sizeof(*r));
    r->main_comp = -1;

    int n = g->w * g->h;
    int* label = NULL;
    int* sizes = NULL;
    CutRegion* tmp_cut = NULL;
    unsigned char* cut_seen = NULL;
    int* origin = NULL;
    int* dist = NULL;
    int* parent = NULL;
    int* queue = NULL;
    int* dsu = NULL;
    unsigned char* is_patch = NULL;
    Bridge* br = NULL;
    int nbr = 0, brcap = 0;
    int ncomp = 0;
    int rc = -1;

    label = (int*)malloc(sizeof(int) * (size_t)n);
    if (!label || grid_components(g, label, &sizes, &ncomp) != 0) goto done;

    r->components = ncomp;
    if (ncomp == 0) { r->repair_ok = 1; rc = 0; goto done; }

    {
        int main_comp;
        if (in_bounds(g, spawn.x, spawn.y) && walkable(g, spawn.x, spawn.y))
            main_comp = label[spawn.y * g->w + spawn.x];
        else {
            main_comp = 0;
            for (int c = 1; c < ncomp; c++)
                if (sizes[c] > sizes[main_comp]) main_comp = c;
        }
        r->main_comp = main_comp;
        r->main_size = sizes[main_comp];
    }

    /* collect cut-region details */
    tmp_cut = (CutRegion*)calloc((size_t)ncomp, sizeof(CutRegion));
    cut_seen = (unsigned char*)calloc((size_t)ncomp, 1);
    if (!tmp_cut || !cut_seen) goto done;
    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < g->w; x++) {
            int i = y * g->w + x;
            if (g->blocked[i]) continue;
            int c = label[i];
            if (c == r->main_comp) continue;
            if (!cut_seen[c]) {
                cut_seen[c] = 1;
                tmp_cut[c].comp = c;
                tmp_cut[c].seed = i;
                tmp_cut[c].x0 = tmp_cut[c].x1 = x;
                tmp_cut[c].y0 = tmp_cut[c].y1 = y;
            } else {
                if (x < tmp_cut[c].x0) tmp_cut[c].x0 = x;
                if (x > tmp_cut[c].x1) tmp_cut[c].x1 = x;
                if (y < tmp_cut[c].y0) tmp_cut[c].y0 = y;
                if (y > tmp_cut[c].y1) tmp_cut[c].y1 = y;
            }
            tmp_cut[c].size++;
        }
    }
    r->cut_regions = ncomp - 1;
    r->cut = (CutRegion*)malloc(sizeof(CutRegion) * (size_t)r->cut_regions);
    if (!r->cut) { r->cut_regions = 0; goto done; }
    {
        int k = 0;
        for (int c = 0; c < ncomp; c++) {
            if (c == r->main_comp) continue;
            r->cut[k++] = tmp_cut[c];
            r->cut_cells += tmp_cut[c].size;
        }
        qsort(r->cut, (size_t)r->cut_regions, sizeof(CutRegion), cmp_cut);
    }

    if (ncomp == 1) { r->repair_ok = 1; rc = 0; goto done; }

    /* wavefront over blocked cells: nearest component and its distance */
    origin = (int*)malloc(sizeof(int) * (size_t)n);
    dist = (int*)malloc(sizeof(int) * (size_t)n);
    parent = (int*)malloc(sizeof(int) * (size_t)n);
    queue = (int*)malloc(sizeof(int) * (size_t)n);
    if (!origin || !dist || !parent || !queue) goto done;
    for (int i = 0; i < n; i++) { origin[i] = -1; dist[i] = 0; parent[i] = -1; }

    int head = 0, tail = 0;
    for (int i = 0; i < n; i++) {
        if (g->blocked[i]) continue;
        int x = i % g->w, y = i / g->w;
        for (int d = 0; d < 4; d++) {
            int nx = x + D4[d][0], ny = y + D4[d][1];
            if (!in_bounds(g, nx, ny)) continue;
            int ni = ny * g->w + nx;
            if (!g->blocked[ni] || origin[ni] >= 0) continue;
            origin[ni] = label[i];
            dist[ni] = 1;
            queue[tail++] = ni;
        }
    }
    while (head < tail) {
        int c = queue[head++];
        int x = c % g->w, y = c / g->w;
        for (int d = 0; d < 4; d++) {
            int nx = x + D4[d][0], ny = y + D4[d][1];
            if (!in_bounds(g, nx, ny)) continue;
            int ni = ny * g->w + nx;
            if (!g->blocked[ni] || origin[ni] >= 0) continue;
            origin[ni] = origin[c];
            dist[ni] = dist[c] + 1;
            parent[ni] = c;
            queue[tail++] = ni;
        }
    }

    /* bridge candidates */
    for (int i = 0; i < n; i++) {
        if (!g->blocked[i] || origin[i] < 0) continue;
        int x = i % g->w, y = i / g->w;
        for (int d = 0; d < 4; d++) {
            int nx = x + D4[d][0], ny = y + D4[d][1];
            if (!in_bounds(g, nx, ny)) continue;
            int ni = ny * g->w + nx;
            if (!g->blocked[ni]) {
                int other = label[ni];
                if (other != origin[i] &&
                    bridge_push(&br, &nbr, &brcap, dist[i], origin[i], other, i, -1) != 0)
                    goto done;
            } else if (origin[ni] >= 0 && origin[ni] != origin[i]) {
                if (bridge_push(&br, &nbr, &brcap, dist[i] + dist[ni],
                                origin[i], origin[ni], i, ni) != 0)
                    goto done;
            }
        }
    }
    qsort(br, (size_t)nbr, sizeof(Bridge), cmp_bridge);

    /* Kruskal over the component graph, materialising the chosen cells */
    dsu = (int*)malloc(sizeof(int) * (size_t)ncomp);
    r->patch = (Pt*)malloc(sizeof(Pt) * (size_t)n);
    is_patch = (unsigned char*)calloc((size_t)n, 1);
    if (!dsu || !r->patch || !is_patch) goto done;
    for (int c = 0; c < ncomp; c++) dsu[c] = c;

    int accepted = 0;
    for (int i = 0; i < nbr && accepted < ncomp - 1; i++) {
        int ra = dsu_find(dsu, br[i].a);
        int rb = dsu_find(dsu, br[i].b);
        if (ra == rb) continue;
        dsu[ra] = rb;
        if (mark_chain(parent, br[i].ca, g, is_patch, r) != 0) goto done;
        if (br[i].cb >= 0 && mark_chain(parent, br[i].cb, g, is_patch, r) != 0) goto done;
        accepted++;
    }
    r->repair_ok = (accepted == ncomp - 1);
    rc = 0;

done:
    free(label); free(sizes); free(tmp_cut); free(cut_seen);
    free(origin); free(dist); free(parent); free(queue);
    free(dsu); free(is_patch); free(br);
    if (rc != 0) topo_report_free(r);
    return rc;
}

static void topo_print(const TopoReport* r) {
    printf("topo components=%d main=%d main_size=%d cut_regions=%d cut_cells=%d "
           "patch_cells=%d repair_ok=%d\n",
           r->components, r->main_comp, r->main_size, r->cut_regions,
           r->cut_cells, r->patch_cells, r->repair_ok);
    for (int i = 0; i < r->cut_regions && i < 8; i++) {
        const CutRegion* c = &r->cut[i];
        if (c->seed < 0) continue;
        printf("  cut comp=%d size=%d bbox=(%d,%d)-(%d,%d)\n",
               c->comp, c->size, c->x0, c->y0, c->x1, c->y1);
    }
}

/* ------------------------------------------------------------------ */
/* self-test                                                            */
/* ------------------------------------------------------------------ */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int dsu_root(int* p, int x) { return dsu_find(p, x); }

/* open the suggested cells on a scratch copy and count components again */
static int patch_reconnects(const Grid* g, const TopoReport* r) {
    int n = g->w * g->h;
    Grid c = *g;
    int* label = (int*)malloc(sizeof(int) * (size_t)n);
    int* sizes = NULL;
    int ncomp = -1, ok;
    c.blocked = (unsigned char*)malloc((size_t)n);
    if (!c.blocked || !label) { free(c.blocked); free(label); return 0; }
    memcpy(c.blocked, g->blocked, (size_t)n);
    for (int i = 0; i < r->patch_cells; i++)
        c.blocked[r->patch[i].y * c.w + r->patch[i].x] = 0;
    ok = grid_components(&c, label, &sizes, &ncomp) == 0 && ncomp == 1;
    free(sizes); free(label); free(c.blocked);
    return ok;
}

/* exhaustive minimum number of blocked cells to open so the grid becomes
   one component; only for tiny maps. Returns -1 if max_k is too small. */
#define BRUTE_MAX_K 16
static int brute_min_patch(const Grid* g, int max_k) {
    int n = g->w * g->h;
    int* label = (int*)malloc(sizeof(int) * (size_t)n);
    int* cells = (int*)malloc(sizeof(int) * (size_t)n);
    unsigned char* scratch = (unsigned char*)malloc((size_t)n);
    int* sizes = NULL;
    int ncomp = 0, nblocked = 0, best = -1;
    if (!label || !cells || !scratch) {
        free(label); free(cells); free(scratch);
        return -1;
    }
    if (grid_components(g, label, &sizes, &ncomp) != 0) {
        free(label); free(cells); free(scratch); free(sizes);
        return -1;
    }
    free(sizes);
    for (int i = 0; i < n; i++) if (g->blocked[i]) cells[nblocked++] = i;
    if (ncomp <= 1) { free(label); free(cells); free(scratch); return 0; }
    if (max_k > BRUTE_MAX_K) max_k = BRUTE_MAX_K;
    if (max_k > nblocked) max_k = nblocked;

    for (int k = 1; k <= max_k && best < 0; k++) {
        int idx[BRUTE_MAX_K];
        for (int i = 0; i < k; i++) idx[i] = i;
        for (;;) {
            memcpy(scratch, g->blocked, (size_t)n);
            for (int i = 0; i < k; i++) scratch[cells[idx[i]]] = 0;
            Grid c = *g;
            c.blocked = scratch;
            int nc = 0, *s2 = NULL;
            if (grid_components(&c, label, &s2, &nc) == 0 && nc == 1) best = k;
            free(s2);
            if (best >= 0) break;
            int i = k - 1;
            while (i >= 0 && idx[i] == nblocked - k + i) i--;
            if (i < 0) break;
            idx[i]++;
            for (int j = i + 1; j < k; j++) idx[j] = idx[j - 1] + 1;
        }
    }
    free(label); free(cells); free(scratch);
    return best;
}

/* build a union-find over legal 8-dir steps and compare its partition with
   the 4-neighbour component labeling used by topo_check */
static int check_components_vs_dsu(const Grid* g) {
    int n = g->w * g->h;
    int* p = (int*)malloc(sizeof(int) * (size_t)n);
    int* label = (int*)malloc(sizeof(int) * (size_t)n);
    int* comp_root = NULL;
    int* root_comp = NULL;
    int* sizes = NULL;
    int ncomp = 0;
    int ok = 1;
    if (!p || !label) { free(p); free(label); return 0; }

    for (int i = 0; i < n; i++) p[i] = i;
    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < g->w; x++) {
            Pt a = { x, y };
            if (!walkable(g, x, y)) continue;
            for (int d = 0; d < 8; d++) {
                Pt b = { x + DIRS[d][0], y + DIRS[d][1] };
                if (!step_ok(g, a, b)) continue;
                int ra = dsu_root(p, y * g->w + x);
                int rb = dsu_root(p, b.y * g->w + b.x);
                if (ra != rb) p[ra] = rb;
            }
        }
    }
    if (grid_components(g, label, &sizes, &ncomp) != 0) ok = 0;

    if (ok) {
        int roots = 0;
        for (int i = 0; i < n; i++)
            if (!g->blocked[i] && dsu_root(p, i) == i) roots++;
        if (roots != ncomp) ok = 0;
    }
    if (ok) {
        comp_root = (int*)malloc(sizeof(int) * (size_t)ncomp);
        root_comp = (int*)malloc(sizeof(int) * (size_t)n);
        if (!comp_root || !root_comp) ok = 0;
        else {
            for (int c = 0; c < ncomp; c++) comp_root[c] = -1;
            for (int i = 0; i < n; i++) root_comp[i] = -1;
            for (int i = 0; i < n && ok; i++) {
                if (g->blocked[i]) continue;
                int c = label[i];
                int rt = dsu_root(p, i);
                if (comp_root[c] < 0) {
                    if (root_comp[rt] >= 0 && root_comp[rt] != c) ok = 0;
                    comp_root[c] = rt;
                    root_comp[rt] = c;
                } else if (comp_root[c] != rt) ok = 0;
            }
        }
    }
    free(p); free(label); free(sizes); free(comp_root); free(root_comp);
    return ok;
}

static unsigned rng_state = 12345u;
static unsigned rng_next(void) {
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state;
}

static void fill_random(Grid* g, unsigned density_pct) {
    for (int i = 0; i < g->w * g->h; i++)
        g->blocked[i] = (rng_next() % 100u) < density_pct;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

int main(int argc, char** argv) {
    if (argc > 1 && strcmp(argv[1], "--bench") == 0) {
        const char* names[] = {"sparse10", "dense45", "dense65", "checker", "thickwall", "maze_stripes"};
        for (int mode = 0; mode < 6; mode++) {
            Grid bg;
            if (grid_init(&bg, 400, 250) != 0) return 1;
            if (mode == 0) fill_random(&bg, 10);
            else if (mode == 1) fill_random(&bg, 45);
            else if (mode == 2) fill_random(&bg, 65);
            else if (mode == 3) {
                for (int y = 0; y < bg.h; y++)
                    for (int x = 0; x < bg.w; x++)
                        bg.blocked[y * bg.w + x] = ((x + y) & 1) ? 0 : 1;
            } else if (mode == 4) {
                for (int x = 0; x < bg.w; x += 9)
                    for (int y = 0; y < bg.h; y++) bg.blocked[y * bg.w + x] = 1;
                for (int y = 0; y < bg.h; y += 9)
                    for (int x = 0; x < bg.w; x++) bg.blocked[y * bg.w + x] = 1;
            } else {
                for (int y = 0; y < bg.h; y++)
                    for (int x = 0; x < bg.w; x++)
                        bg.blocked[y * bg.w + x] = ((x * 37 + y * 11) % 23 < 7);
            }
            TopoReport r;
            double best = 1e9;
            int rc = -1, comps = 0, pc = 0;
            for (int i = 0; i < 3; i++) {
                double t0 = now_ms();
                rc = topo_check(&bg, (Pt){-1,-1}, &r);
                double dt = now_ms() - t0;
                if (dt < best) best = dt;
                comps = r.components; pc = r.patch_cells;
                topo_report_free(&r);
            }
            printf("bench %-13s rc=%d comps=%d patch=%d ms=%.2f\n",
                   names[mode], rc, comps, pc, best);
            grid_free(&bg);
        }
        return 0;
    }

    /* --- original pathfinding demo --- */
    const char* rows[] = {
        "S....#....",
        ".##..#....",
        ".#......#.",
        ".#.####.#.",
        ".....#..G.",
    };
    Grid g;
    if (grid_load(&g, rows, 5) != 0) { printf("load fail\n"); return 1; }
    Pt s = {0,0}, t = {8,4};
    int cap = g.w * g.h;
    Pt* raw = (Pt*)malloc(sizeof(Pt) * (size_t)cap);
    Pt* sm = (Pt*)malloc(sizeof(Pt) * (size_t)cap);
    if (!raw || !sm) { printf("alloc fail\n"); grid_free(&g); return 1; }
    int n = astar(&g, s, t, raw, cap);
    if (n < 0) { printf("no path\n"); return 1; }
    int m = smooth(&g, raw, n, sm, cap);
    printf("raw=%d len=%.4f smooth=%d len=%.4f\n", n, path_length(raw,n), m, path_length(sm,m));
    for (int i = 0; i < m; i++) printf("(%d,%d)%s", sm[i].x, sm[i].y, i+1==m ? "\n" : " ");
    int bad = 0;
    for (int i = 1; i < m; i++) if (!line_of_sight(&g, sm[i-1], sm[i])) bad++;
    printf("bad_segments=%d\n", bad);
    printf("corner_cut=%d\n", !step_ok(&g, (Pt){1,0}, (Pt){2,1}));
    check("astar_found", n > 0);
    check("smooth_valid", m > 0 && bad == 0);
    free(raw); free(sm); grid_free(&g);

    /* --- split map: two 2-wide full-height walls, three equal rooms --- */
    {
        const char* split[] = {
            "..##..##..",
            "..##..##..",
            "..##..##..",
            "..##..##..",
            "..##..##..",
        };
        Grid sg;
        TopoReport r;
        if (grid_load(&sg, split, 5) != 0) { printf("load fail\n"); return 1; }
        check("split_load", 1);
        check("split_topology", topo_check(&sg, (Pt){-1,-1}, &r) == 0);
        topo_print(&r);
        check("split_components", r.components == 3);
        check("split_cut_regions", r.cut_regions == 2);
        check("split_cut_cells", r.cut_cells == 20);
        check("split_patch_four", r.patch_cells == 4);
        check("split_repair_ok", r.repair_ok == 1);
        check("split_patch_reconnects", patch_reconnects(&sg, &r));
        int all_wall = 1;
        for (int i = 0; i < r.patch_cells; i++) {
            Pt p = r.patch[i];
            if (!sg.blocked[p.y * sg.w + p.x]) all_wall = 0;
        }
        check("split_patch_all_wall", all_wall);
        check("split_brute_min", brute_min_patch(&sg, 4) == r.patch_cells);
        topo_report_free(&r);

        /* spawn on the right selects it as main; both other rooms are cut */
        check("split_spawn", topo_check(&sg, (Pt){9,0}, &r) == 0);
        check("split_spawn_main_right", r.main_comp != -1 && r.main_size == 10);
        check("split_spawn_cut_cells", r.cut_cells == 20);
        topo_report_free(&r);

        check("split_dsu_match", check_components_vs_dsu(&sg));
        grid_free(&sg);
    }

    /* --- three rooms separated by 2-wide walls: four cells needed --- */
    {
        const char* three[] = {
            "...##...##...",
            "...##...##...",
            "...##...##...",
            "...##...##...",
            "...##...##...",
        };
        Grid tg;
        TopoReport r;
        if (grid_load(&tg, three, 5) != 0) { printf("load fail\n"); return 1; }
        check("three_topology", topo_check(&tg, (Pt){-1,-1}, &r) == 0);
        topo_print(&r);
        check("three_components", r.components == 3);
        check("three_cut_regions", r.cut_regions == 2);
        check("three_patch_four", r.patch_cells == 4);
        check("three_repair_ok", r.repair_ok == 1);
        check("three_patch_reconnects", patch_reconnects(&tg, &r));
        check("three_brute_min", brute_min_patch(&tg, 5) == r.patch_cells);
        topo_report_free(&r);
        check("three_dsu_match", check_components_vs_dsu(&tg));
        grid_free(&tg);
    }

    /* --- enclosed room inside an open field --- */
    {
        Grid eg;
        if (grid_init(&eg, 20, 20) != 0) { printf("load fail\n"); return 1; }
        for (int y = 6; y <= 12; y++)
            for (int x = 6; x <= 12; x++)
                if (y == 6 || y == 12 || x == 6 || x == 12)
                    eg.blocked[y * eg.w + x] = 1;
        TopoReport r;
        check("enclosed_topology", topo_check(&eg, (Pt){-1,-1}, &r) == 0);
        topo_print(&r);
        check("enclosed_components", r.components == 2);
        check("enclosed_cut_cells", r.cut_cells == 25);
        check("enclosed_patch_one", r.patch_cells == 1);
        check("enclosed_repair_ok", r.repair_ok == 1);
        check("enclosed_patch_reconnects", patch_reconnects(&eg, &r));
        check("enclosed_brute_min", brute_min_patch(&eg, 3) == r.patch_cells);
        topo_report_free(&r);
        check("enclosed_dsu_match", check_components_vs_dsu(&eg));
        grid_free(&eg);
    }

    /* --- random maps: topology must agree with union-find --- */
    {
        Grid rg;
        int all_ok = 1, all_repair = 1;
        if (grid_init(&rg, 64, 64) != 0) { printf("load fail\n"); return 1; }
        for (unsigned dens = 10; dens <= 50; dens += 10) {
            fill_random(&rg, dens);
            if (!check_components_vs_dsu(&rg)) all_ok = 0;
            TopoReport r;
            if (topo_check(&rg, (Pt){-1,-1}, &r) != 0 ||
                r.components <= 0 ||
                r.cut_regions != r.components - 1 ||
                !r.repair_ok) all_repair = 0;
            topo_report_free(&r);
        }
        check("random_dsu_match", all_ok);
        check("random_repair_ok", all_repair);
        grid_free(&rg);
    }

    /* --- diagonal-only islands: corner-cut rule still means 4-connectivity --- */
    {
        Grid cg;
        if (grid_init(&cg, 33, 33) != 0) { printf("load fail\n"); return 1; }
        for (int y = 0; y < cg.h; y++)
            for (int x = 0; x < cg.w; x++)
                cg.blocked[y * cg.w + x] = ((x + y) & 1) ? 0 : 1;
        TopoReport r;
        check("checker_dsu_match", check_components_vs_dsu(&cg));
        check("checker_topology", topo_check(&cg, (Pt){-1,-1}, &r) == 0);
        printf("checker components=%d patch_cells=%d repair_ok=%d\n",
               r.components, r.patch_cells, r.repair_ok);
        check("checker_many_components", r.components > 100);
        check("checker_repair_ok", r.repair_ok == 1);
        topo_report_free(&r);
        grid_free(&cg);
    }

    /* --- 100k cells, topology check under 50 ms --- */
    {
        Grid bg;
        if (grid_init(&bg, 400, 250) != 0) { printf("load fail\n"); return 1; }
        for (int y = 0; y < bg.h; y++) bg.blocked[y * bg.w + 100] = 1;   /* full vertical wall */
        for (int x = 0; x <= 100; x++) bg.blocked[120 * bg.w + x] = 1;   /* splits the left half */
        for (int y = 60; y <= 70; y++)                                   /* enclosed room */
            for (int x = 250; x <= 260; x++)
                if (y == 60 || y == 70 || x == 250 || x == 260)
                    bg.blocked[y * bg.w + x] = 1;

        TopoReport r;
        double best_ms = 1e9;
        int rc = -1;
        for (int i = 0; i < 3; i++) {
            double t0 = now_ms();
            rc = topo_check(&bg, (Pt){-1,-1}, &r);
            double dt = now_ms() - t0;
            if (dt < best_ms) best_ms = dt;
            topo_report_free(&r);
        }
        check("big_topology", rc == 0);
        rc = topo_check(&bg, (Pt){-1,-1}, &r);
        printf("big cells=%d components=%d cut_regions=%d cut_cells=%d "
               "patch_cells=%d repair_ok=%d topo_ms=%.3f\n",
               bg.w * bg.h, r.components, r.cut_regions, r.cut_cells,
               r.patch_cells, r.repair_ok, best_ms);
        check("big_components", r.components == 4);
        check("big_patch_three", r.patch_cells == 3);
        check("big_repair_ok", r.repair_ok == 1);
        check("big_under_50ms", best_ms < 50.0);
        topo_report_free(&r);
        grid_free(&bg);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
