/* pf_lpa.c - LPA* (lifelong planning A*) with lazy queue deletion.
 *
 * The solver keeps g/rhs values and the priority queue between calls. A grid
 * edit only dirties the edited cell and its neighbours; the next query
 * recomputes locally inconsistent cells instead of searching the whole map.
 * That is the incremental planner: query 1 is a normal A* search, later
 * queries after edits touch a fraction of the nodes.
 *
 * Paths are rebuilt from the final g values (greedy predecessor descent), not
 * from cached parent pointers, because parents may be recomputed several times
 * during one propagation and only the settled g values are authoritative.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "pf_internal.h"

static double step_cost(int dx, int dy) {
    return pf__step_cost(dx, dy);
}

/* key(u) = [min(g,rhs) + h(u,goal), min(g,rhs)]; INF keys never pop first. */
static void key_of(const PfLpa* l, int u, int w, PfPoint goal, PfHeapNode* out) {
    double m = l->g[u] < l->rhs[u] ? l->g[u] : l->rhs[u];
    out->idx = u;
    out->k1 = PF_INF;
    out->k2 = PF_INF;
    if (m < PF_INF) {
        PfPoint p = { u % w, u / w };
        out->k1 = m + pf__octile(p, goal);
        out->k2 = m;
    }
}

static void full_reset(PfLpa* l, int n) {
    for (int i = 0; i < n; i++) {
        l->g[i] = PF_INF;
        l->rhs[i] = PF_INF;
        l->ver[i] = 0;
    }
    pf_heap_clear(&l->heap);
}

/* Recompute rhs(u) from its walkable predecessors and requeue if inconsistent.
 * A blocked cell has rhs = INF, which is how walls propagate into the search. */
static void update_vertex(PfSolver* s, int u) {
    PfLpa* l = &s->lpa;
    const PfGrid* g = &s->grid;
    int w = g->w;
    PfPoint goal = { l->goal % w, l->goal / w };
    double best = PF_INF;
    if (u == l->start) {
        best = 0.0;
    } else if (pf_walkable(g, u % w, u / w)) {
        PfPoint up = { u % w, u / w };
        for (int d = 0; d < PF_DIR_COUNT; d++) {
            PfPoint p = { up.x + PF_DX[d], up.y + PF_DY[d] };
            if (!pf_walkable(g, p.x, p.y)) continue;
            if (!pf_step_ok(g, p, up)) continue;
            int pi = p.y * w + p.x;
            if (l->g[pi] >= PF_INF) continue;
            double c = l->g[pi] + step_cost(PF_DX[d], PF_DY[d]);
            if (c < best) best = c;
        }
    }
    l->rhs[u] = best;
    l->ver[u]++;
    if (l->g[u] != l->rhs[u]) {
        PfHeapNode k;
        key_of(l, u, w, goal, &k);
        k.ver = l->ver[u];
        pf_heap_push(&l->heap, k);
    }
}

/* Lowering g[u] can only lower its successors' rhs, and the new rhs is simply
 * min(old rhs, g[u] + step). Skipping the full predecessor rescan here is what
 * keeps LPA*'s per-expansion cost close to A*'s; the raise path still rescans. */
static void lower_successor(PfSolver* s, int v, int u, double step) {
    PfLpa* l = &s->lpa;
    int w = s->grid.w;
    if (l->g[u] >= PF_INF) return;
    double cand = l->g[u] + step;
    if (cand >= l->rhs[v]) return;
    l->rhs[v] = cand;
    l->ver[v]++;
    if (l->g[v] != l->rhs[v]) {
        PfHeapNode k;
        PfPoint goal = { l->goal % w, l->goal / w };
        key_of(l, v, w, goal, &k);
        k.ver = l->ver[v];
        pf_heap_push(&l->heap, k);
    }
}

void pf__lpa_init(PfSolver* s) {
    int n = s->grid.w * s->grid.h;
    PfLpa* l = &s->lpa;
    memset(l, 0, sizeof(*l));
    l->g = (double*)malloc(sizeof(double) * (size_t)n);
    l->rhs = (double*)malloc(sizeof(double) * (size_t)n);
    l->ver = (unsigned*)malloc(sizeof(unsigned) * (size_t)n);
    l->n = n;
    pf_heap_init(&l->heap);
    if (l->g && l->rhs && l->ver) full_reset(l, n);
}

void pf__lpa_free(PfSolver* s) {
    PfLpa* l = &s->lpa;
    free(l->g);
    free(l->rhs);
    free(l->ver);
    pf_heap_free(&l->heap);
    memset(l, 0, sizeof(*l));
}

/* An edit changes the rhs of the cell and of every neighbour that may step
 * into it. With corner cutting forbidden a cell also affects the diagonal
 * steps *between* its neighbours (a diagonal needs both corner cells free),
 * so the refresh radius is 2: the full 5x5 block around the edit. Missing
 * that second ring leaves stale rhs values that no queue entry refers to,
 * which shows up as a lost path after an opening. */
static void apply_edit(PfSolver* s, int x, int y) {
    const PfGrid* g = &s->grid;
    int w = g->w;
    for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= g->w || ny >= g->h) continue;
            if (!pf_walkable(g, nx, ny)) continue;
            update_vertex(s, ny * w + nx);
        }
    }
}

static void compute_shortest_path(PfSolver* s, long* expansions) {
    PfLpa* l = &s->lpa;
    const PfGrid* g = &s->grid;
    int w = g->w;
    int ti = l->goal;
    PfPoint goal_pt = { ti % w, ti / w };
    PfHeapNode e;
    while (pf_heap_peek(&l->heap, &e) == 0) {
        PfHeapNode goal_key;
        key_of(l, ti, w, goal_pt, &goal_key);
        /* Standard LPA* termination: the goal must be consistent AND its key
         * no worse than the best queued key. Comparing keys alone is not
         * enough: a goal that settled at INF is "consistent" but still needs
         * lowering, and its INF key would otherwise pass the test. */
        if (l->g[ti] == l->rhs[ti] &&
            !(e.k1 < goal_key.k1 || (e.k1 == goal_key.k1 && e.k2 < goal_key.k2)))
            break;
        if (pf_heap_pop(&l->heap, &e) != 0) break;
        int u = e.idx;
        if (e.ver != l->ver[u]) continue;          /* lazily invalidated */
        (*expansions)++;
        /* g > rhs means the cell's value belongs lower (over-consistent), so
         * lowering it can only improve successors — no rescan needed. g <= rhs
         * means raising it, which may destroy the cheapest predecessor of any
         * successor, so those need a full rhs recompute; the cell itself needs
         * one too (the Succ(s) ∪ {s} step of the pseudocode), otherwise it can
         * settle at INF while still inconsistent with nothing left to fix it. */
        int raising = l->g[u] <= l->rhs[u];
        l->g[u] = raising ? PF_INF : l->rhs[u];
        PfPoint up = { u % w, u / w };
        for (int d = 0; d < PF_DIR_COUNT; d++) {
            PfPoint np = { up.x + PF_DX[d], up.y + PF_DY[d] };
            if (!pf_walkable(g, np.x, np.y)) continue;
            if (!pf_step_ok(g, up, np)) continue;
            if (raising) update_vertex(s, np.y * w + np.x);
            else lower_successor(s, np.y * w + np.x, u, step_cost(PF_DX[d], PF_DY[d]));
        }
        if (raising) update_vertex(s, u);
    }
    /* Queue entries whose version has moved on are dead; drop them now so the
     * heap does not grow by a full iteration per re-plan. */
    pf_heap_compact(&l->heap, l->ver);
}

/* Paths come from the shared canonical descent over the settled g values, so
 * an incremental query returns exactly the A* path on the same map. Returns
 * cell count, -1 for a broken descent, -2 when the caller's buffer is full. */
static double lpa_g(const void* ctx, int idx) {
    return ((const PfLpa*)ctx)->g[idx];
}

static int rebuild_path(const PfLpa* l, const PfGrid* g, PfPoint* pts, int cap) {
    return pf__descend_optimal(g, lpa_g, l, l->start, l->goal, pts, cap);
}

int pf__lpa_solve(PfSolver* s, PfPoint start, PfPoint goal, PfPath* out) {
    PfLpa* l = &s->lpa;
    const PfGrid* g = &s->grid;
    int w = g->w;
    out->n = 0;
    out->expansions = 0;
    if (!pf_walkable(g, start.x, start.y) || !pf_walkable(g, goal.x, goal.y))
        return PF_ENOPATH;
    if (!l->g || !l->rhs || !l->ver) return PF_EBADARG;

    int si = start.y * w + start.x, ti = goal.y * w + goal.x;
    int same_query = l->valid && l->start == si && l->goal == ti;
    if (!same_query) {
        full_reset(l, l->n);
        l->start = si;
        l->goal = ti;
        l->valid = 1;
        s->ndirty = 0;
        update_vertex(s, si);
    }
    /* Edits are applied on every query, including the first one after a reset:
     * cells edited before the very first plan must still shape the map, they
     * just have no cached values to invalidate. */
    if (s->ndirty > 0) {
        for (int i = 0; i < s->ndirty; i++)
            apply_edit(s, s->dirty[i] % w, s->dirty[i] / w);
        s->ndirty = 0;
    }

    compute_shortest_path(s, &out->expansions);

    if (l->g[ti] >= PF_INF) return PF_ENOPATH;

    int n = rebuild_path(l, g, out->pts, out->cap);
    if (n == -2) return PF_EPATHBUF;
    if (n < 0) {
        /* Cached state is stale; rebuild from scratch once so callers never
         * receive a path that disagrees with A*. */
        full_reset(l, l->n);
        l->start = si;
        l->goal = ti;
        s->ndirty = 0;
        update_vertex(s, si);
        compute_shortest_path(s, &out->expansions);
        if (l->g[ti] >= PF_INF) return PF_ENOPATH;
        n = rebuild_path(l, g, out->pts, out->cap);
        if (n == -2) return PF_EPATHBUF;
        if (n < 0) return PF_ENOPATH;
    }
    out->n = n;
    out->cost = l->g[ti];
    return PF_OK;
}
