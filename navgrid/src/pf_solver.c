/* pf_solver.c - solver lifecycle, map edits and the four-way dispatch. */
#include <stdlib.h>
#include <string.h>

#include "pf_internal.h"

PfSolver* pf_solver_new(const PfGrid* g) {
    if (!g || !g->blocked || g->w <= 0 || g->h <= 0) return NULL;
    PfSolver* s = (PfSolver*)calloc(1, sizeof(*s));
    if (!s) return NULL;
    pf__grid_copy(&s->grid, g);
    if (!s->grid.blocked) { free(s); return NULL; }
    s->dirty_cap = 64;
    s->dirty = (int*)malloc(sizeof(int) * (size_t)s->dirty_cap);
    if (!s->dirty) { pf_grid_free(&s->grid); free(s); return NULL; }
    pf__lpa_init(s);
    return s;
}

void pf_solver_free(PfSolver* s) {
    if (!s) return;
    pf__lpa_free(s);
    pf__hpa_free(s);
    pf_grid_free(&s->grid);
    free(s->dirty);
    free(s);
}

const PfGrid* pf_solver_grid(const PfSolver* s) {
    return s ? &s->grid : NULL;
}

void pf_solver_set_changes_expected(PfSolver* s, int yes) {
    if (!s) return;
    s->changes_expected = yes != 0;
    s->profile_valid = 0;
}

static int dirty_has(const PfSolver* s, int idx) {
    for (int i = 0; i < s->ndirty; i++) if (s->dirty[i] == idx) return 1;
    return 0;
}

static void mark_dirty(PfSolver* s, int x, int y) {
    int idx = y * s->grid.w + x;
    if (dirty_has(s, idx)) return;
    if (s->ndirty == s->dirty_cap) {
        int cap = s->dirty_cap * 2;
        int* d = (int*)realloc(s->dirty, sizeof(int) * (size_t)cap);
        if (!d) return;
        s->dirty = d;
        s->dirty_cap = cap;
    }
    s->dirty[s->ndirty++] = idx;
}

/* Edits are recorded, not applied to LPA* state here: apply_edit() needs the
 * current start/goal. HPA* has no cheap local update, so any edit drops its
 * abstraction and it rebuilds lazily on the next query. */
void pf_solver_set_blocked(PfSolver* s, int x, int y, int blocked) {
    if (!s) return;
    if (x < 0 || y < 0 || x >= s->grid.w || y >= s->grid.h) return;
    int was = pf_grid_blocked(&s->grid, x, y);
    int now = blocked != 0;
    if (was == now) return;
    pf_grid_set(&s->grid, x, y, now);
    mark_dirty(s, x, y);
    s->changes++;
    s->profile_valid = 0;
    pf__hpa_invalidate(s);
}

static PfAlgo resolve_auto(PfSolver* s, PfPoint start, PfPoint goal, PfPath* out) {
    if (!s->profile_valid) {
        s->auto_algo = pf_select_algo(&s->grid, start, goal, s->changes_expected, &s->profile);
        s->profile_valid = 1;
    }
    out->algo = s->auto_algo;
    return s->auto_algo;
}

int pf_solve(PfSolver* s, PfPoint start, PfPoint goal, PfAlgo algo, PfPath* out) {
    if (!s || !out || !out->pts || out->cap <= 0) return PF_EBADARG;
    out->n = 0;
    out->cost = 0.0;
    out->expansions = 0;
    out->algo = algo;
    if (!pf_walkable(&s->grid, start.x, start.y) || !pf_walkable(&s->grid, goal.x, goal.y))
        return PF_EBADARG;

    if (algo == PF_AUTO) algo = resolve_auto(s, start, goal, out);

    int rc;
    switch (algo) {
    case PF_ASTAR:
        out->algo = PF_ASTAR;
        rc = pf__astar(&s->grid, start, goal, 0, out, &out->expansions);
        break;
    case PF_ANYANGLE:
        out->algo = PF_ANYANGLE;
        rc = pf__astar(&s->grid, start, goal, 1, out, &out->expansions);
        break;
    case PF_INCREMENTAL:
        out->algo = PF_INCREMENTAL;
        rc = pf__lpa_solve(s, start, goal, out);
        break;
    case PF_HIERARCHICAL:
        out->algo = PF_HIERARCHICAL;
        rc = pf__hpa_solve(s, start, goal, out);
        break;
    default:
        return PF_EBADARG;
    }
    return rc;
}
