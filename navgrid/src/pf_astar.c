/* pf_astar.c - grid A* and its any-angle variant (Theta*).
 *
 * A*: 8-directional, corner cutting forbidden (pf_step_ok), octile heuristic.
 *
 * Theta*: A* plus line-of-sight parent relaxation. When the line from a node's
 * parent to a neighbour is clear, the neighbour inherits the parent, so path
 * segments may span many cells and leave the 8 compass directions. The
 * heuristic is Euclidean instead of octile because octile over-estimates
 * straight-line movement. Both heuristics are consistent with their own edge
 * sets, so no node is ever reopened.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "pf_internal.h"

typedef struct {
    double g, f;
    int parent;
    unsigned char closed;
} ANode;

static double step_len(int dx, int dy) {
    return pf__step_cost(dx, dy);
}

static int emit_chain(const ANode* n, int ti, int w, PfPath* out) {
    int len = 0;
    for (int c = ti; c != -1; c = n[c].parent) {
        len++;
        if (len > out->cap) return PF_EPATHBUF;
    }
    int c = ti;
    for (int i = len - 1; i >= 0; i--) {
        out->pts[i].x = c % w;
        out->pts[i].y = c / w;
        c = n[c].parent;
    }
    out->n = len;
    return PF_OK;
}

/* A* keeps its g values inside node structs; hand them to the shared canonical
 * reconstruction so A* and LPA* emit identical paths. */
static double astar_g(const void* ctx, int idx) {
    return ((const ANode*)ctx)[idx].g;
}

static int emit_optimal(const PfGrid* g, const ANode* n, int si, int ti, PfPath* out) {
    int len = pf__descend_optimal(g, astar_g, n, si, ti, out->pts, out->cap);
    if (len == -2) return PF_EPATHBUF;
    if (len < 0) return emit_chain(n, ti, g->w, out);
    out->n = len;
    return PF_OK;
}

int pf__astar(const PfGrid* g, PfPoint s, PfPoint t, int theta,
              PfPath* out, long* expansions) {
    if (!pf_walkable(g, s.x, s.y) || !pf_walkable(g, t.x, t.y)) return PF_EBADARG;
    int w = g->w, h = g->h, n = w * h;
    int si = s.y * w + s.x, ti = t.y * w + t.x;

    ANode* nodes = (ANode*)malloc(sizeof(ANode) * (size_t)n);
    if (!nodes) return PF_EBADARG;
    for (int i = 0; i < n; i++) { nodes[i].g = PF_INF; nodes[i].parent = -1; nodes[i].closed = 0; }

    PfHeap heap;
    pf_heap_init(&heap);
    double h0 = theta ? hypot((double)(t.x - s.x), (double)(t.y - s.y))
                      : pf__octile(s, t);
    nodes[si].g = 0.0;
    nodes[si].f = h0;
    pf_heap_push(&heap, (PfHeapNode){ h0, 0.0, si, 0 });

    int found = 0;
    PfHeapNode e;
    while (pf_heap_pop(&heap, &e) == 0) {
        int u = e.idx;
        if (nodes[u].closed) continue;
        PfPoint cp = { u % w, u / w };
        double hu = theta ? hypot((double)(t.x - cp.x), (double)(t.y - cp.y))
                          : pf__octile(cp, t);
        if (e.k1 > nodes[u].g + hu + 1e-12) continue;   /* stale entry */
        nodes[u].closed = 1;
        if (expansions) (*expansions)++;
        if (u == ti) { found = 1; break; }

        int gp = nodes[u].parent;
        for (int d = 0; d < PF_DIR_COUNT; d++) {
            PfPoint np = { cp.x + PF_DX[d], cp.y + PF_DY[d] };
            if (!pf_step_ok(g, cp, np)) continue;
            int ni = np.y * w + np.x;
            if (nodes[ni].closed) continue;
            double ng;
            int par;
            if (theta && gp >= 0) {
                PfPoint p = { gp % w, gp / w };
                if (pf_line_clear(g, p, np)) {
                    ng = nodes[gp].g + hypot((double)(np.x - p.x), (double)(np.y - p.y));
                    par = gp;
                } else {
                    ng = nodes[u].g + step_len(PF_DX[d], PF_DY[d]);
                    par = u;
                }
            } else {
                ng = nodes[u].g + step_len(PF_DX[d], PF_DY[d]);
                par = u;
            }
            if (ng < nodes[ni].g) {
                nodes[ni].g = ng;
                double hn = theta ? hypot((double)(t.x - np.x), (double)(t.y - np.y))
                                  : pf__octile(np, t);
                nodes[ni].f = ng + hn;
                nodes[ni].parent = par;
                pf_heap_push(&heap, (PfHeapNode){ nodes[ni].f, ng, ni, 0 });
            }
        }
    }

    int rc = PF_ENOPATH;
    if (found) {
        out->n = 0;
        out->cost = nodes[ti].g;
        if (theta) {
            rc = emit_chain(nodes, ti, w, out);
            if (rc == PF_OK) out->cost = pf_path_length(out->pts, out->n);
        } else {
            rc = emit_optimal(g, nodes, si, ti, out);
        }
    }
    pf_heap_free(&heap);
    free(nodes);
    return rc;
}
