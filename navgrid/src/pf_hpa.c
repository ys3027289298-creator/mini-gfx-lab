/* pf_hpa.c - HPA* (hierarchical path A*) over fixed 8x8 clusters.
 *
 * Abstract graph
 *   A cell on a cluster border (it has a walkable 4-neighbour in another
 *   cluster) is an abstract node. Within each cluster every pair of border
 *   nodes is joined by an intra-edge whose cost is the exact A* distance
 *   restricted to that cluster; across a cluster border, neighbouring border
 *   cells get a boundary edge.
 *
 * Query
 *   Two temporary nodes S/G connect the start/goal cells to the border nodes of
 *   their clusters. An abstract A* runs S -> G, then the abstract edges are
 *   stitched back into grid cells, re-running the intra-cluster searches on
 *   demand. The result is a legal grid path whose cost is an upper bound on A*
 *   (it is exact whenever the optimal path crosses cluster borders the same
 *   way the abstraction does); the sufficiency test checks that gap.
 *
 * Same-cluster queries fall back to plain A*: a cluster-restricted path may
 * leave and re-enter a cluster, so restricting it would not be comparable.
 *
 * Expanded-node counts include the cells expanded by intra-cluster searches,
 * so the benchmark table compares like with like.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "pf_internal.h"

#define HPA_CHAIN_MAX (HPA_CLUSTER_CELLS + 1)
#define HPA_ID_NONE (-1)

typedef struct {
    double g;
    int parent;
    unsigned char closed;
} HNode;

static void hpa_clear(PfHpa* h) {
    if (h->adj) {
        for (int c = 0; c < h->adj_count; c++) free(h->adj[c].e);
        free(h->adj);
    }
    free(h->cell_node);
    free(h->nx);
    free(h->ny);
    free(h->ncl);
    free(h->cl_start);
    free(h->cl_len);
    free(h->cl_nodes);
    memset(h, 0, sizeof(*h));
}

void pf__hpa_free(PfSolver* s) {
    hpa_clear(&s->hpa);
}

void pf__hpa_invalidate(PfSolver* s) {
    hpa_clear(&s->hpa);
}

/* In the abstract graph a border node is a cell that can step out of its
 * cluster, which is exactly what makes the abstraction connected. */
static int is_border(const PfGrid* g, int x, int y) {
    if (!pf_walkable(g, x, y)) return 0;
    int cl = pf__cluster_of(g, x, y);
    for (int d = 0; d < PF_DIR_COUNT; d++) {
        int nx = x + PF_DX[d], ny = y + PF_DY[d];
        if (!pf_walkable(g, nx, ny)) continue;
        if (pf__cluster_of(g, nx, ny) != cl) return 1;
    }
    return 0;
}

static void adj_push(PfHAdj* a, int to, double cost) {
    if (a->n == a->cap) {
        int cap = a->cap ? a->cap * 2 : 8;
        PfHEdge* e = (PfHEdge*)realloc(a->e, sizeof(*e) * (size_t)cap);
        if (!e) return;
        a->e = e;
        a->cap = cap;
    }
    a->e[a->n].to = to;
    a->e[a->n].cost = (float)cost;
    a->n++;
}

static int adj_find(const PfHAdj* a, int to) {
    for (int i = 0; i < a->n; i++) if (a->e[i].to == to) return i;
    return -1;
}

static double hnode_key(PfPoint p, PfPoint goal) {
    return pf__octile(p, goal);
}

/* A* restricted to cluster cl. chain (optional) receives the cell indices of
 * the path, start inclusive. Returns 0 on success. */
static int cluster_search(const PfGrid* g, int cl, int sx, int sy, int tx, int ty,
                          int* chain, int* chain_len, double* cost, long* expanded) {
    int cols = (g->w + HPA_CLUSTER - 1) / HPA_CLUSTER;
    int x0 = (cl % cols) * HPA_CLUSTER, y0 = (cl / cols) * HPA_CLUSTER;
    int x1 = x0 + HPA_CLUSTER, y1 = y0 + HPA_CLUSTER;
    if (x1 > g->w) x1 = g->w;
    if (y1 > g->h) y1 = g->h;
    if (sx < x0 || sy < y0 || sx >= x1 || sy >= y1) return -1;
    if (tx < x0 || ty < y0 || tx >= x1 || ty >= y1) return -1;
    int w = g->w, cw = x1 - x0, ch = y1 - y0, cn = cw * ch;
    HNode* nodes = (HNode*)malloc(sizeof(HNode) * (size_t)cn);
    if (!nodes) return -1;
    for (int i = 0; i < cn; i++) {
        nodes[i].g = PF_INF;
        nodes[i].parent = -1;
        nodes[i].closed = 0;
    }
    PfHeap heap;
    pf_heap_init(&heap);
    int ls = (sy - y0) * cw + (sx - x0);
    int lt = (ty - y0) * cw + (tx - x0);
    PfPoint tp = { tx, ty };
    nodes[ls].g = 0.0;
    int found = (ls == lt);
    if (!found) {
        pf_heap_push(&heap, (PfHeapNode){ pf__octile((PfPoint){ sx, sy }, tp), 0.0, ls, 0 });
        PfHeapNode e;
        while (pf_heap_pop(&heap, &e) == 0) {
            int u = e.idx;
            if (nodes[u].closed) continue;
            nodes[u].closed = 1;
            if (expanded) (*expanded)++;
            if (u == lt) { found = 1; break; }
            PfPoint up = { x0 + u % cw, y0 + u / cw };
            for (int d = 0; d < PF_DIR_COUNT; d++) {
                PfPoint np = { up.x + PF_DX[d], up.y + PF_DY[d] };
                if (np.x < x0 || np.y < y0 || np.x >= x1 || np.y >= y1) continue;
                if (!pf_step_ok(g, up, np)) continue;
                int ni = (np.y - y0) * cw + (np.x - x0);
                if (nodes[ni].closed) continue;
                double ng = nodes[u].g + ((PF_DX[d] && PF_DY[d]) ? 1.4142135623730951 : 1.0);
                if (ng < nodes[ni].g) {
                    nodes[ni].g = ng;
                    nodes[ni].parent = u;
                    pf_heap_push(&heap, (PfHeapNode){ ng + pf__octile(np, tp), ng, ni, 0 });
                }
            }
        }
    }
    int rc = -1;
    if (found) {
        if (chain) {
            int n = 0, c = lt;
            while (c != -1) { n++; c = nodes[c].parent; }
            if (n > HPA_CHAIN_MAX) {
                pf_heap_free(&heap);
                free(nodes);
                return -1;
            }
            c = lt;
            for (int i = n - 1; i >= 0; i--) {
                chain[i] = (y0 + c / cw) * w + (x0 + c % cw);
                c = nodes[c].parent;
            }
            *chain_len = n;
        }
        *cost = nodes[lt].g;
        rc = 0;
    }
    pf_heap_free(&heap);
    free(nodes);
    return rc;
}

/* One Dijkstra inside cluster cl from a single source. `local` maps every
 * cell of the cluster (cluster-local index) to an abstract-node id, or -1; the
 * distances to those nodes are written into `cost[id]`, so the scratch buffer
 * is always one cluster long no matter how many border nodes exist per map.
 * Stops as soon as every target is settled: only border-to-border distances
 * are ever needed. Scratch buffers are supplied by the caller so a build does
 * not malloc per source. */
static void cluster_dijkstra(const PfGrid* g, int cl, int sx, int sy,
                             const int* local, int targets,
                             double* cost, double* dist, char* closed) {
    int cols = (g->w + HPA_CLUSTER - 1) / HPA_CLUSTER;
    int x0 = (cl % cols) * HPA_CLUSTER, y0 = (cl / cols) * HPA_CLUSTER;
    int x1 = x0 + HPA_CLUSTER, y1 = y0 + HPA_CLUSTER;
    if (x1 > g->w) x1 = g->w;
    if (y1 > g->h) y1 = g->h;
    int cw = x1 - x0, ch = y1 - y0, cn = cw * ch;
    for (int i = 0; i < cn; i++) { dist[i] = PF_INF; closed[i] = 0; }
    PfHeap heap;
    pf_heap_init(&heap);
    int ls = (sy - y0) * cw + (sx - x0);
    dist[ls] = 0.0;
    pf_heap_push(&heap, (PfHeapNode){ 0.0, 0.0, ls, 0 });
    int found = 0;
    PfHeapNode e;
    while (pf_heap_pop(&heap, &e) == 0) {
        int u = e.idx;
        if (closed[u]) continue;
        closed[u] = 1;
        if (local[u] >= 0) {
            cost[local[u]] = dist[u];
            if (++found == targets) break;
        }
        PfPoint up = { x0 + u % cw, y0 + u / cw };
        for (int d = 0; d < PF_DIR_COUNT; d++) {
            PfPoint np = { up.x + PF_DX[d], up.y + PF_DY[d] };
            if (np.x < x0 || np.y < y0 || np.x >= x1 || np.y >= y1) continue;
            if (!pf_step_ok(g, up, np)) continue;
            int ni = (np.y - y0) * cw + (np.x - x0);
            if (closed[ni]) continue;
            double ng = dist[u] + pf__step_cost(PF_DX[d], PF_DY[d]);
            if (ng < dist[ni]) {
                dist[ni] = ng;
                pf_heap_push(&heap, (PfHeapNode){ ng, ng, ni, 0 });
            }
        }
    }
    pf_heap_free(&heap);
}

static void hpa_build(PfSolver* s) {
    const PfGrid* g = &s->grid;
    PfHpa* h = &s->hpa;
    hpa_clear(h);
    int w = g->w, n = w * g->h;
    h->cluster_cols = (g->w + HPA_CLUSTER - 1) / HPA_CLUSTER;
    h->cluster_rows = (g->h + HPA_CLUSTER - 1) / HPA_CLUSTER;
    h->cluster_count = h->cluster_cols * h->cluster_rows;

    int cap = 64, nn = 0;
    int* nxs = (int*)malloc(sizeof(int) * (size_t)cap);
    int* nys = (int*)malloc(sizeof(int) * (size_t)cap);
    int* ncl = (int*)malloc(sizeof(int) * (size_t)cap);
    h->cell_node = (int*)malloc(sizeof(int) * (size_t)n);
    if (!nxs || !nys || !ncl || !h->cell_node) { free(nxs); free(nys); free(ncl); hpa_clear(h); return; }
    for (int i = 0; i < n; i++) h->cell_node[i] = HPA_ID_NONE;
    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < w; x++) {
            if (!is_border(g, x, y)) continue;
            if (nn == cap) {
                cap *= 2;
                int* a = (int*)realloc(nxs, sizeof(int) * (size_t)cap);
                if (a) nxs = a;
                int* b = (int*)realloc(nys, sizeof(int) * (size_t)cap);
                if (b) nys = b;
                int* c = (int*)realloc(ncl, sizeof(int) * (size_t)cap);
                if (c) ncl = c;
                if (!a || !b || !c) { free(nxs); free(nys); free(ncl); hpa_clear(h); return; }
            }
            nxs[nn] = x; nys[nn] = y; ncl[nn] = pf__cluster_of(g, x, y);
            h->cell_node[y * w + x] = nn;
            nn++;
        }
    }
    h->border_count = nn;
    h->nx = nxs;
    h->ny = nys;
    h->ncl = ncl;
    h->adj = (PfHAdj*)calloc((size_t)(nn ? nn : 1), sizeof(PfHAdj));
    h->adj_count = nn;
    h->cl_start = (int*)malloc(sizeof(int) * (size_t)h->cluster_count);
    h->cl_len = (int*)calloc((size_t)h->cluster_count, sizeof(int));
    h->cl_nodes = (int*)malloc(sizeof(int) * (size_t)(nn ? nn : 1));
    int* fill = (int*)calloc((size_t)h->cluster_count, sizeof(int));
    if (!h->adj || !h->cl_start || !h->cl_len || !h->cl_nodes || !fill) {
        free(fill);
        hpa_clear(h);
        return;
    }
    for (int i = 0; i < nn; i++) h->cl_len[ncl[i]]++;
    int run = 0;
    for (int c = 0; c < h->cluster_count; c++) { h->cl_start[c] = run; run += h->cl_len[c]; }
    for (int i = 0; i < nn; i++) h->cl_nodes[h->cl_start[ncl[i]] + fill[ncl[i]]++] = i;
    free(fill);

    /* intra-cluster edges: one Dijkstra per border node covers all of its
     * pairs with the other border nodes in the same cluster */
    int cols = h->cluster_cols;
    int* local = (int*)malloc(sizeof(int) * HPA_CLUSTER_CELLS);
    double* nodecost = (double*)malloc(sizeof(double) * (size_t)(nn ? nn : 1));
    double* dist = (double*)malloc(sizeof(double) * HPA_CLUSTER_CELLS);
    char* closed = (char*)malloc(HPA_CLUSTER_CELLS);
    if (!local || !nodecost || !dist || !closed) {
        free(local); free(nodecost); free(dist); free(closed);
        hpa_clear(h);
        return;
    }
    for (int c = 0; c < h->cluster_count; c++) {
        int base = h->cl_start[c], cnt = h->cl_len[c];
        if (cnt < 2) continue;
        int cx0 = (c % cols) * HPA_CLUSTER, cy0 = (c / cols) * HPA_CLUSTER;
        int cw = HPA_CLUSTER, chh = HPA_CLUSTER;
        if (cx0 + cw > g->w) cw = g->w - cx0;
        if (cy0 + chh > g->h) chh = g->h - cy0;
        for (int i = 0; i < cw * chh; i++) local[i] = -1;
        for (int i = 0; i < cnt; i++) {
            int id = h->cl_nodes[base + i];
            local[(h->ny[id] - cy0) * cw + (h->nx[id] - cx0)] = id;
        }
        for (int i = 0; i < cnt; i++) {
            int a = h->cl_nodes[base + i];
            /* nodecost is reused across sources; unreachable targets must not
             * keep an older source's value. */
            for (int j = 0; j < cnt; j++) nodecost[h->cl_nodes[base + j]] = PF_INF;
            cluster_dijkstra(g, c, h->nx[a], h->ny[a], local, cnt - 1,
                             nodecost, dist, closed);
            for (int j = i + 1; j < cnt; j++) {
                int b = h->cl_nodes[base + j];
                double cost = nodecost[b];
                if (cost >= PF_INF) continue;
                adj_push(&h->adj[a], b, cost);
                adj_push(&h->adj[b], a, cost);
            }
        }
    }
    free(local);
    free(nodecost);
    free(dist);
    free(closed);
    /* boundary edges across cluster seams */
    for (int i = 0; i < nn; i++) {
        int x = h->nx[i], y = h->ny[i];
        for (int d = 0; d < PF_DIR_COUNT; d++) {
            int nx = x + PF_DX[d], ny = y + PF_DY[d];
            if (!pf_walkable(g, nx, ny)) continue;
            if (!pf_step_ok(g, (PfPoint){ x, y }, (PfPoint){ nx, ny })) continue;
            int j = h->cell_node[ny * w + nx];
            if (j < 0 || h->ncl[j] == h->ncl[i]) continue;
            if (adj_find(&h->adj[i], j) >= 0) continue;
            double cost = (PF_DX[d] && PF_DY[d]) ? 1.4142135623730951 : 1.0;
            adj_push(&h->adj[i], j, cost);
        }
    }
    h->valid = 1;
}

/* Append the cells of path a..b (excluding a). A legal single step (which can
 * follow a cluster seam) is appended directly; everything else goes through a
 * cluster-restricted search. */
static int append_segment(const PfGrid* g, int a, int b, PfPoint* pts, int* n,
                          int cap, long* expansions) {
    if (a == b) return 0;
    PfPoint pa = { a % g->w, a / g->w };
    PfPoint pb = { b % g->w, b / g->w };
    if (pf_step_ok(g, pa, pb)) {
        if (*n >= cap) return -1;
        pts[(*n)++] = pb;
        return 0;
    }
    int chain[HPA_CHAIN_MAX];
    int len = 0;
    double cost = 0.0;
    if (cluster_search(g, pf__cluster_of(g, pa.x, pa.y), pa.x, pa.y, pb.x, pb.y,
                       chain, &len, &cost, expansions) != 0) return -1;
    for (int i = 1; i < len; i++) {
        if (*n >= cap) return -1;
        pts[(*n)++] = (PfPoint){ chain[i] % g->w, chain[i] / g->w };
    }
    return 0;
}

int pf__hpa_solve(PfSolver* s, PfPoint start, PfPoint goal, PfPath* out) {
    const PfGrid* g = &s->grid;
    PfHpa* h = &s->hpa;
    int w = g->w;
    out->n = 0;
    out->expansions = 0;
    if (!pf_walkable(g, start.x, start.y) || !pf_walkable(g, goal.x, goal.y))
        return PF_ENOPATH;
    if (!h->valid) hpa_build(s);
    if (!h->valid) return pf__astar(g, start, goal, 0, out, &out->expansions);

    int sc = pf__cluster_of(g, start.x, start.y);
    int gc = pf__cluster_of(g, goal.x, goal.y);
    if (sc == gc || h->cl_len[sc] == 0 || h->cl_len[gc] == 0)
        return pf__astar(g, start, goal, 0, out, &out->expansions);

    int an = h->border_count;
    int S = an, G = an + 1, vn = an + 2;
    HNode* nodes = (HNode*)malloc(sizeof(HNode) * (size_t)vn);
    if (!nodes) return pf__astar(g, start, goal, 0, out, &out->expansions);
    for (int i = 0; i < vn; i++) {
        nodes[i].g = PF_INF;
        nodes[i].parent = -1;
        nodes[i].closed = 0;
    }
    PfHeap heap;
    pf_heap_init(&heap);
    nodes[S].g = 0.0;
    pf_heap_push(&heap, (PfHeapNode){ hnode_key(start, goal), 0.0, S, 0 });

    int found = 0;
    PfHeapNode e;
    while (pf_heap_pop(&heap, &e) == 0) {
        int u = e.idx;
        if (nodes[u].closed) continue;
        nodes[u].closed = 1;
        out->expansions++;
        if (u == G) { found = 1; break; }

        if (u == S) {
            int base = h->cl_start[sc], cnt = h->cl_len[sc];
            for (int i = 0; i < cnt; i++) {
                int id = h->cl_nodes[base + i];
                double cost = 0.0;
                if (cluster_search(g, sc, start.x, start.y, h->nx[id], h->ny[id],
                                   NULL, NULL, &cost, &out->expansions) != 0) continue;
                if (nodes[id].closed) continue;
                if (cost < nodes[id].g) {
                    nodes[id].g = cost;
                    nodes[id].parent = S;
                    pf_heap_push(&heap, (PfHeapNode){
                        cost + hnode_key((PfPoint){ h->nx[id], h->ny[id] }, goal), cost, id, 0 });
                }
            }
            continue;
        }
        if (u == G) continue;

        for (int i = 0; i < h->adj[u].n; i++) {
            int v = h->adj[u].e[i].to;
            if (nodes[v].closed) continue;
            double ng = nodes[u].g + h->adj[u].e[i].cost;
            if (ng < nodes[v].g) {
                nodes[v].g = ng;
                nodes[v].parent = u;
                pf_heap_push(&heap, (PfHeapNode){
                    ng + hnode_key((PfPoint){ h->nx[v], h->ny[v] }, goal), ng, v, 0 });
            }
        }
        /* connect to the goal through this cluster */
        if (h->ncl[u] == gc) {
            double cost = 0.0;
            if (cluster_search(g, gc, h->nx[u], h->ny[u], goal.x, goal.y,
                               NULL, NULL, &cost, &out->expansions) == 0) {
                double ng = nodes[u].g + cost;
                if (ng < nodes[G].g) {
                    nodes[G].g = ng;
                    nodes[G].parent = u;
                    pf_heap_push(&heap, (PfHeapNode){ ng, ng, G, 0 });
                }
            }
        }
    }

    int rc = PF_ENOPATH;
    if (found) {
        int ids[PF_MAX_DIM * PF_MAX_DIM];
        int cnt = 0, c = nodes[G].parent;
        while (c != -1 && c != S && cnt < PF_MAX_DIM * PF_MAX_DIM) {
            ids[cnt++] = c;
            c = nodes[c].parent;
        }
        int pn = 0;
        out->pts[pn++] = start;
        int cur = start.y * w + start.x;
        int ok = 1;
        for (int i = cnt - 1; i >= 0 && ok; i--) {
            int nxt = h->ny[ids[i]] * w + h->nx[ids[i]];
            if (append_segment(g, cur, nxt, out->pts, &pn, out->cap, &out->expansions) != 0) ok = 0;
            cur = nxt;
        }
        if (ok) {
            int gn = goal.y * w + goal.x;
            if (append_segment(g, cur, gn, out->pts, &pn, out->cap, &out->expansions) != 0) ok = 0;
        }
        if (ok) {
            int m = 0;
            for (int i = 0; i < pn; i++) {
                if (m > 0 && out->pts[i].x == out->pts[m - 1].x && out->pts[i].y == out->pts[m - 1].y)
                    continue;
                out->pts[m++] = out->pts[i];
            }
            out->n = m;
            out->cost = pf_path_length(out->pts, out->n);
            rc = (m > 0) ? PF_OK : PF_ENOPATH;
        }
    }
    pf_heap_free(&heap);
    free(nodes);
    if (rc == PF_OK) return rc;
    /* Disconnected abstraction or stitching mismatch: grid search is truth. */
    return pf__astar(g, start, goal, 0, out, &out->expansions);
}
