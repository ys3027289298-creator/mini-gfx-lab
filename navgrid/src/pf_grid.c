/* pf_grid.c - grid container, geometric predicates and map profiling. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pf_internal.h"

const int PF_DX[PF_DIR_COUNT] = { 1, -1, 0, 0, 1, 1, -1, -1 };
const int PF_DY[PF_DIR_COUNT] = { 0, 0, 1, -1, 1, -1, 1, -1 };

const char* pf_algo_name(PfAlgo a) {
    switch (a) {
    case PF_ASTAR: return "astar";
    case PF_ANYANGLE: return "anyangle";
    case PF_INCREMENTAL: return "incremental";
    case PF_HIERARCHICAL: return "hierarchical";
    case PF_AUTO: return "auto";
    default: return "?";
    }
}

int pf_grid_init(PfGrid* g, int w, int h) {
    if (!g || w <= 0 || h <= 0 || w > PF_MAX_DIM || h > PF_MAX_DIM) return -1;
    memset(g, 0, sizeof(*g));
    g->blocked = (unsigned char*)calloc((size_t)w * (size_t)h, 1);
    if (!g->blocked) return -1;
    g->w = w;
    g->h = h;
    return 0;
}

void pf_grid_free(PfGrid* g) {
    if (!g) return;
    free(g->blocked);
    g->blocked = NULL;
    g->w = g->h = 0;
}

int pf_grid_load(PfGrid* g, const char* const* rows, int h) {
    if (!g || !rows || h <= 0) return -1;
    int w = (int)strlen(rows[0]);
    if (pf_grid_init(g, w, h) != 0) return -1;
    for (int y = 0; y < h; y++) {
        if ((int)strlen(rows[y]) != w) { pf_grid_free(g); return -1; }
        for (int x = 0; x < w; x++)
            g->blocked[y * w + x] = (unsigned char)(rows[y][x] == '#');
    }
    return 0;
}

void pf_grid_set(PfGrid* g, int x, int y, int blocked) {
    if (!g || x < 0 || y < 0 || x >= g->w || y >= g->h) return;
    g->blocked[y * g->w + x] = (unsigned char)(blocked != 0);
}

int pf_grid_blocked(const PfGrid* g, int x, int y) {
    return g->blocked[y * g->w + x] != 0;
}

int pf_walkable(const PfGrid* g, int x, int y) {
    if (!g || x < 0 || y < 0 || x >= g->w || y >= g->h) return 0;
    return !g->blocked[y * g->w + x];
}

/* A diagonal step is legal only when both orthogonal neighbours are free, so
 * no path can squeeze between two touching walls. */
int pf_step_ok(const PfGrid* g, PfPoint a, PfPoint b) {
    int dx = b.x - a.x, dy = b.y - a.y;
    if (abs(dx) > 1 || abs(dy) > 1) return 0;
    if (!pf_walkable(g, b.x, b.y)) return 0;
    if (dx != 0 && dy != 0)
        if (!pf_walkable(g, a.x + dx, a.y) || !pf_walkable(g, a.x, a.y + dy)) return 0;
    return 1;
}

/* Supercover-free sampling: enough that a walk of |dx|+|dy| cells never skips
 * a cell, which is all the visibility checks need. */
int pf_line_clear(const PfGrid* g, PfPoint a, PfPoint b) {
    int steps = abs(b.x - a.x) + abs(b.y - a.y);
    if (steps == 0) return 1;
    PfPoint prev = a;
    for (int i = 1; i <= steps; i++) {
        double t = (double)i / (double)steps;
        PfPoint cur;
        cur.x = (int)lround(a.x + (b.x - a.x) * t);
        cur.y = (int)lround(a.y + (b.y - a.y) * t);
        if (cur.x == prev.x && cur.y == prev.y) continue;
        if (!pf_step_ok(g, prev, cur)) return 0;
        prev = cur;
    }
    return prev.x == b.x && prev.y == b.y;
}

double pf_path_length(const PfPoint* pts, int n) {
    double s = 0.0;
    for (int i = 1; i < n; i++)
        s += hypot((double)(pts[i].x - pts[i - 1].x), (double)(pts[i].y - pts[i - 1].y));
    return s;
}

int pf_path_valid(const PfGrid* g, const PfPath* p, int any_angle) {
    if (!p || !p->pts || p->n <= 0) return 0;
    for (int i = 0; i < p->n; i++)
        if (!pf_walkable(g, p->pts[i].x, p->pts[i].y)) return 0;
    for (int i = 1; i < p->n; i++) {
        int ok = any_angle ? pf_line_clear(g, p->pts[i - 1], p->pts[i])
                           : pf_step_ok(g, p->pts[i - 1], p->pts[i]);
        if (!ok) return 0;
    }
    return 1;
}

void pf_path_init(PfPath* p, PfPoint* buf, int cap) {
    if (!p) return;
    p->pts = buf;
    p->n = 0;
    p->cap = buf ? cap : 0;
    p->cost = 0.0;
    p->expansions = 0;
}

/* ---------------------------------------------------------------- profiling */

void pf_profile(const PfGrid* g, PfProfile* out) {
    memset(out, 0, sizeof(*out));
    int total = g->w * g->h;
    int blocked = 0, walk = 0, corridor = 0, junction = 0;
    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < g->w; x++) {
            if (pf_grid_blocked(g, x, y)) { blocked++; continue; }
            walk++;
            int n = (pf_walkable(g, x + 1, y) ? 1 : 0) + (pf_walkable(g, x - 1, y) ? 1 : 0) +
                    (pf_walkable(g, x, y + 1) ? 1 : 0) + (pf_walkable(g, x, y - 1) ? 1 : 0);
            if (n <= 2) corridor++;
            else if (n >= 3) junction++;
        }
    }
    out->blocked_ratio = total ? (double)blocked / (double)total : 0.0;
    out->corridor_ratio = walk ? (double)corridor / (double)walk : 0.0;
    out->junction_ratio = walk ? (double)junction / (double)walk : 0.0;
    out->clusters = ((g->w + HPA_CLUSTER - 1) / HPA_CLUSTER) *
                    ((g->h + HPA_CLUSTER - 1) / HPA_CLUSTER);
    out->large = (g->w >= 96 && g->h >= 96);
}

/* Pick by map shape rather than by hand:
 *   map edited mid-run         -> LPA* (only the edit stales the plan)
 *   mostly-corridor maps       -> A* (any-angle buys nothing in 1-wide halls,
 *                                 and clusters only add overhead in a maze)
 *   large maps with real walls -> HPA* (clusters keep expansion counts down)
 *   open/junction-heavy maps   -> Theta* (fewer waypoints, shorter paths)
 * A large but nearly empty map stays any-angle: clusters would buy nothing. */
PfAlgo pf_select_algo(const PfGrid* g, PfPoint s, PfPoint t,
                      int changes_expected, PfProfile* out) {
    (void)s;
    (void)t;
    PfProfile local;
    if (!out) out = &local;
    pf_profile(g, out);
    if (changes_expected) return PF_INCREMENTAL;
    if (out->corridor_ratio >= 0.55) return PF_ASTAR;
    if (out->large && out->blocked_ratio >= 0.05) return PF_HIERARCHICAL;
    return PF_ANYANGLE;
}

/* ------------------------------------------------------------------- solver */

void pf__grid_copy(PfGrid* dst, const PfGrid* src) {
    dst->w = src->w;
    dst->h = src->h;
    dst->blocked = (unsigned char*)malloc((size_t)src->w * (size_t)src->h);
    if (dst->blocked) memcpy(dst->blocked, src->blocked, (size_t)src->w * (size_t)src->h);
}

double pf__octile(PfPoint a, PfPoint b) {
    int dx = abs(a.x - b.x), dy = abs(a.y - b.y);
    int lo = dx < dy ? dx : dy;
    return (double)lo * 1.4142135623730951 + (double)(dx + dy - 2 * lo);
}

int pf__cluster_of(const PfGrid* g, int x, int y) {
    int cols = (g->w + HPA_CLUSTER - 1) / HPA_CLUSTER;
    return (y / HPA_CLUSTER) * cols + (x / HPA_CLUSTER);
}

double pf__step_cost(int dx, int dy) {
    return (dx != 0 && dy != 0) ? 1.4142135623730951 : 1.0;
}

/* Walk backwards from the goal, always to the cheapest predecessor that
 * closes the cost identity within tolerance. Equal-cost predecessors are
 * resolved in the fixed PF_DX/PF_DY order, which is what makes the result
 * reproducible across planners. Written straight into `out` and reversed.
 * Returns the cell count, or -1 if no descent exists, or -2 if `cap` is too
 * small to hold the path. */
int pf__descend_optimal(const PfGrid* g, PfGvalFn get, const void* ctx,
                        int start, int goal, PfPoint* out, int cap) {
    int w = g->w;
    int n = 0, cur = goal;
    int guard = w * g->h + 1;
    if (cap <= 0) return -2;
    out[n++] = (PfPoint){ cur % w, cur / w };
    while (cur != start) {
        if (--guard < 0) return -1;
        PfPoint up = { cur % w, cur / w };
        double gcur = get(ctx, cur);
        int bp = -1;
        double bd = PF_INF;
        for (int d = 0; d < PF_DIR_COUNT; d++) {
            PfPoint p = { up.x + PF_DX[d], up.y + PF_DY[d] };
            if (!pf_walkable(g, p.x, p.y)) continue;
            if (!pf_step_ok(g, p, up)) continue;
            int pi = p.y * w + p.x;
            double gp = get(ctx, pi);
            if (gp >= PF_INF) continue;
            double c = gp + pf__step_cost(PF_DX[d], PF_DY[d]);
            if (c <= gcur + 1e-9 && c < bd) { bd = c; bp = pi; }
        }
        if (bp < 0) return -1;
        if (n >= cap) return -2;
        cur = bp;
        out[n++] = (PfPoint){ cur % w, cur / w };
    }
    for (int i = 0, j = n - 1; i < j; i++, j--) {
        PfPoint t = out[i]; out[i] = out[j]; out[j] = t;
    }
    return n;
}
