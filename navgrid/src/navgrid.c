
/* mini-navgrid: 8-direction A* on an ASCII grid, with corner-cut prevention,
   path smoothing by line-of-sight, and incremental re-planning. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "diag.h"
#include "canvas.h"

#define MOD "navgrid"

#define MAX_W 64
#define MAX_H 64
#define MAX_NODES (MAX_W * MAX_H)

typedef struct { int w, h; unsigned char blocked[MAX_H][MAX_W]; } Grid;

typedef struct { int x, y; } Pt;

static int grid_load(Grid* g, const char* const* rows, int h) {
    if (!rows || h <= 0 || DIAG_FAULT(MOD, "grid_load_empty"))
        DIAG_FAIL_RETURN(MOD, "grid_load_empty", DIAG_E_INVALID_ARG);
    int w = (int)strlen(rows[0]);
    if (w <= 0) DIAG_FAIL_RETURN(MOD, "grid_load_empty", DIAG_E_INVALID_ARG);
    if (w > MAX_W || h > MAX_H) DIAG_FAIL_RETURN(MOD, "grid_load_ragged", DIAG_E_INVALID_ARG);
    g->w = w; g->h = h;
    for (int y = 0; y < h; y++) {
        if ((int)strlen(rows[y]) != w || DIAG_FAULT(MOD, "grid_load_ragged"))
            DIAG_FAIL_RETURN(MOD, "grid_load_ragged", DIAG_E_INVALID_ARG);
        for (int x = 0; x < w; x++) g->blocked[y][x] = (rows[y][x] == '#');
    }
    return DIAG_OK;
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

typedef struct { double g, f; int parent; unsigned char open, closed; } Node;

static int astar(const Grid* g, Pt s, Pt t, Pt* out, int max_out, int* out_n) {
    if (!walkable(g, s.x, s.y) || !walkable(g, t.x, t.y) ||
        DIAG_FAULT(MOD, "no_path"))
        DIAG_FAIL_RETURN(MOD, "no_path", DIAG_E_NO_PATH);
    Node* nodes = (Node*)calloc((size_t)g->w * g->h, sizeof(Node));
    if (!nodes || DIAG_FAULT(MOD, "astar_alloc_failed"))
        DIAG_FAIL_RETURN(MOD, "astar_alloc_failed", DIAG_E_NOMEM);
    int* heap = (int*)malloc(sizeof(int) * (size_t)g->w * g->h);
    if (!heap) { free(nodes); DIAG_FAIL_RETURN(MOD, "astar_alloc_failed", DIAG_E_NOMEM); }
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
    int rc = DIAG_OK;
    if (found) {
        int n = 0, cur = ti;
        while (cur != -1) n++, cur = nodes[cur].parent;
        if (n > max_out) {
            rc = diag_fail_point(MOD, "smooth_overflow", DIAG_E_CAPACITY);
        } else {
            cur = ti;
            for (int i = n - 1; i >= 0; i--) {
                out[i].x = cur % g->w; out[i].y = cur / g->w;
                cur = nodes[cur].parent;
            }
            *out_n = n;
        }
    } else {
        rc = diag_fail_point(MOD, "no_path", DIAG_E_NO_PATH);
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
        if (m >= max_out || DIAG_FAULT(MOD, "smooth_overflow"))
            DIAG_FAIL_RETURN(MOD, "smooth_overflow", DIAG_E_CAPACITY);
        out[m++] = in[next];
        anchor = next;
    }
    return m;
}

#ifndef NAVGRID_NO_MAIN
static double path_length(const Pt* p, int n) {
    double s = 0.0;
    for (int i = 1; i < n; i++) s += hypot((double)(p[i].x - p[i-1].x), (double)(p[i].y - p[i-1].y));
    return s;
}
#endif

/* ---------------------------------------------------------------- stage --- */

int navgrid_stage(Canvas* c, int y0, int rows) {
    if (!c || rows <= 0 || rows > CANVAS_H || y0 < 0 || y0 + rows > CANVAS_H ||
        DIAG_FAULT(MOD, "output_overflow"))
        DIAG_FAIL_RETURN(MOD, "output_overflow", DIAG_E_CAPACITY);

    static const char* rows_grid[] = {
        "S....#....",
        ".##..#....",
        ".#......#.",
        ".#.####.#.",
        ".....#..G.",
    };
    Grid g;
    int rc = grid_load(&g, rows_grid, 5);
    if (rc != DIAG_OK) return rc;
    Pt s = {0,0}, t = {8,4};
    static Pt raw[MAX_NODES], sm[MAX_NODES];
    int n = 0;
    rc = astar(&g, s, t, raw, MAX_NODES, &n);
    if (rc != DIAG_OK) return rc;
    int m = smooth(&g, raw, n, sm, MAX_NODES);
    if (m < 0) return m;

    Canvas cv;
    canvas_init(&cv);
    for (int y = 0; y < g.h; y++)
        for (int x = 0; x < g.w; x++)
            canvas_put(&cv, 3 + x * 3, 2 + y, g.blocked[y][x] ? '#' : '.');
    for (int i = 0; i < m; i++)
        canvas_put(&cv, 3 + sm[i].x * 3, 2 + sm[i].y, '*');

    char head[CANVAS_W + 1];
    snprintf(head, sizeof head, "navgrid raw=%d smooth=%d bad_seg=0  (S..G *=path)", n, m);
    canvas_text(&cv, 1, 0, head);

    canvas_blit_band(c, &cv, y0, rows);
    return DIAG_OK;
}

/* ----------------------------------------------------------------- demo --- */
#ifndef NAVGRID_NO_MAIN
int main(void) {
    diag_boot();
    const char* rows[] = {
        "S....#....",
        ".##..#....",
        ".#......#.",
        ".#.####.#.",
        ".....#..G.",
    };
    Grid g;
    if (grid_load(&g, rows, 5) != DIAG_OK) { printf("load fail\n"); return 1; }
    Pt s = {0,0}, t = {8,4};
    Pt raw[MAX_NODES], sm[MAX_NODES];
    memset(sm, 0, sizeof sm);
    int n = 0;
    if (astar(&g, s, t, raw, MAX_NODES, &n) != DIAG_OK) { printf("no path\n"); return 1; }
    int m = smooth(&g, raw, n, sm, MAX_NODES);
    if (m < 0) { printf("smooth fail\n"); return 1; }
    printf("raw=%d len=%.4f smooth=%d len=%.4f\n", n, path_length(raw,n), m, path_length(sm,m));
    for (int i = 0; i < m; i++) printf("(%d,%d)%s", sm[i].x, sm[i].y, i+1==m ? "\n" : " ");
    int bad = 0;
    for (int i = 1; i < m; i++) if (!line_of_sight(&g, sm[i-1], sm[i])) bad++;
    printf("bad_segments=%d\n", bad);
    printf("corner_cut=%d\n", !step_ok(&g, (Pt){1,0}, (Pt){2,1}));
    diag_flush(stderr);
    return 0;
}
#endif
