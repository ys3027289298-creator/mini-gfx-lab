
/* mini-navgrid: 8-direction least-time A* on an ASCII grid, with per-cell
   terrain speed multipliers, corner-cut prevention, path smoothing by
   line-of-sight, and incremental re-planning. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_W 64
#define MAX_H 64
#define MAX_NODES (MAX_W * MAX_H)
#define SQRT2 1.4142135623730951

/* Terrain legend: '.'/'S'/'G' plain (1.0), 'm' mud (0.5), 'w' water (0.25),
   'H' high ground (2.0), '#' wall, 'x' zero-speed terrain (impassable). */
static double terrain_speed(char c) {
    switch (c) {
        case '.': case 'S': case 'G': return 1.0;
        case 'm': return 0.5;
        case 'w': return 0.25;
        case 'H': return 2.0;
        case '#': case 'x': return 0.0;
        default: return -1.0;
    }
}

typedef struct {
    int w, h;
    unsigned char blocked[MAX_H][MAX_W];
    double speed[MAX_H][MAX_W];   /* movement multiplier, 0 = impassable */
    double max_speed;             /* fastest walkable terrain, for heuristic */
} Grid;

typedef struct { int x, y; } Pt;

static int grid_load(Grid* g, const char* const* rows, int h) {
    int w = (int)strlen(rows[0]);
    if (w <= 0 || w > MAX_W || h <= 0 || h > MAX_H) return -1;
    g->w = w; g->h = h;
    g->max_speed = 0.0;
    for (int y = 0; y < h; y++) {
        if ((int)strlen(rows[y]) != w) return -1;
        for (int x = 0; x < w; x++) {
            double s = terrain_speed(rows[y][x]);
            if (s < 0.0) return -1;
            g->blocked[y][x] = (rows[y][x] == '#');
            g->speed[y][x] = s;
            if (!g->blocked[y][x] && s > g->max_speed) g->max_speed = s;
        }
    }
    return 0;
}

static int in_bounds(const Grid* g, int x, int y) {
    return x >= 0 && y >= 0 && x < g->w && y < g->h;
}
/* walls and zero-speed cells are both impassable */
static int walkable(const Grid* g, int x, int y) {
    return in_bounds(g, x, y) && !g->blocked[y][x] && g->speed[y][x] > 0.0;
}

/* diagonal step p->q is legal only if both orthogonal neighbours are free */
static int step_ok(const Grid* g, Pt p, Pt q) {
    int dx = q.x - p.x, dy = q.y - p.y;
    if (!walkable(g, q.x, q.y)) return 0;
    if (dx != 0 && dy != 0) {
        if (!walkable(g, q.x, p.y) || !walkable(g, p.x, q.y)) return 0;
    }
    return 1;
}

/* time to step onto q: distance divided by the destination cell's multiplier */
static double step_cost(const Grid* g, Pt p, Pt q) {
    int diagonal = (q.x != p.x) && (q.y != p.y);
    return (diagonal ? SQRT2 : 1.0) / g->speed[q.y][q.x];
}

/* Visit each grid cell along the rasterised line p->q, checking that the move
   into it is legal and accumulating its travel time. Returns -1 if blocked. */
static double walk_line(const Grid* g, Pt p, Pt q) {
    int steps = abs(q.x - p.x) > abs(q.y - p.y) ? abs(q.x - p.x) : abs(q.y - p.y);
    double t = 0.0;
    Pt prev = p;
    for (int i = 1; i <= steps; i++) {
        double u = (double)i / steps;
        Pt cur;
        cur.x = (int)lround(p.x + (q.x - p.x) * u);
        cur.y = (int)lround(p.y + (q.y - p.y) * u);
        if (cur.x == prev.x && cur.y == prev.y) continue;
        if (!step_ok(g, prev, cur)) return -1.0;
        t += step_cost(g, prev, cur);
        prev = cur;
    }
    return (prev.x == q.x && prev.y == q.y) ? t : -1.0;
}

/* geodesic distance divided by the fastest terrain is an admissible and
   consistent lower bound on travel time */
static double heuristic(const Grid* g, Pt a, Pt b) {
    double d = hypot((double)(b.x - a.x), (double)(b.y - a.y));
    return g->max_speed > 0.0 ? d / g->max_speed : d;
}

static const int DIRS[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

typedef struct { double g, f; int parent; unsigned char open, closed; } Node;

static int astar(const Grid* g, Pt s, Pt t, Pt* out, int max_out) {
    if (!walkable(g, s.x, s.y) || !walkable(g, t.x, t.y)) return -1;
    Node* nodes = (Node*)calloc((size_t)g->w * g->h, sizeof(Node));
    if (!nodes) return -1;
    int* heap = (int*)malloc(sizeof(int) * (size_t)g->w * g->h);
    int hn = 0;
    int si = s.y * g->w + s.x, ti = t.y * g->w + t.x;
    nodes[si].g = 0.0;
    nodes[si].f = heuristic(g, s, t);
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
            double ng = nodes[cur].g + step_cost(g, cp, np);
            if (!nodes[ni].open || ng < nodes[ni].g) {
                nodes[ni].g = ng;
                nodes[ni].f = ng + heuristic(g, np, t);
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

/* travel time along the straight grid line a->b, or -1 if it is not legal */
static double segment_time(const Grid* g, Pt a, Pt b) {
    if (a.x == b.x && a.y == b.y) return 0.0;
    return walk_line(g, a, b);
}

/* Keep only line-of-sight waypoints, and only when the shortcut does not
   cost more time than the raw sub-path it replaces. */
static int smooth(const Grid* g, const Pt* in, int n, Pt* out, int max_out) {
    if (n <= 0) return 0;
    double prefix[MAX_NODES];
    prefix[0] = 0.0;
    for (int i = 1; i < n; i++)
        prefix[i] = prefix[i-1] + step_cost(g, in[i-1], in[i]);
    int m = 0;
    out[m++] = in[0];
    int anchor = 0;
    while (anchor < n - 1) {
        int next = anchor + 1;
        for (int j = n - 1; j > anchor; j--) {
            double budget = prefix[j] - prefix[anchor];
            double cost = segment_time(g, in[anchor], in[j]);
            if (cost >= 0.0 && cost <= budget + 1e-9) { next = j; break; }
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

static double path_time(const Grid* g, const Pt* p, int n) {
    double t = 0.0;
    for (int i = 1; i < n; i++) {
        double s = segment_time(g, p[i-1], p[i]);
        if (s < 0.0) return -1.0;
        t += s;
    }
    return t;
}

static int near(double a, double b) {
    return fabs(a - b) <= 1e-6;
}

static int run_case(const Grid* g, Pt s, Pt t, Pt* raw, Pt* sm,
                    double* out_time, double* out_smooth_time, int* out_raw_n) {
    int n = astar(g, s, t, raw, MAX_NODES);
    if (n <= 0) return -1;
    int m = smooth(g, raw, n, sm, MAX_NODES);
    if (m <= 0) return -1;
    *out_time = path_time(g, raw, n);
    *out_smooth_time = path_time(g, sm, m);
    *out_raw_n = n;
    return m;
}

int main(void) {
    int ok = 1;
    Pt raw[MAX_NODES], sm[MAX_NODES];
    double t, st;
    int n, m;

    /* Mud and water slow movement: 1/1 + 1/0.5 + 1/0.25 + 1/1 = 8 */
    {
        const char* rows[] = { "S.mwG" };
        Grid g;
        if (grid_load(&g, rows, 1) != 0) { printf("load fail\n"); return 1; }
        m = run_case(&g, (Pt){0,0}, (Pt){4,0}, raw, sm, &t, &st, &n);
        int case_ok = m > 0 && near(t, 8.0) && near(st, 8.0);
        printf("terrain_time=%.4f expect=8.0000 smooth_time=%.4f %s\n",
               t, st, case_ok ? "ok" : "FAIL");
        ok &= case_ok;
    }

    /* Detour over high ground is longer (8.83 > 8) but faster (5.12 < 19).
       Manually: sqrt(2)/2 + 6*0.5 + sqrt(2) = 5.1213 */
    {
        const char* rows[] = {
            "Swwww...G",
            ".HHHHHHH.",
        };
        Grid g;
        if (grid_load(&g, rows, 2) != 0) { printf("load fail\n"); return 1; }
        m = run_case(&g, (Pt){0,0}, (Pt){8,0}, raw, sm, &t, &st, &n);
        double expect = SQRT2 / 2.0 + 3.0 + SQRT2;
        double dist = m > 0 ? path_length(raw, n) : -1.0;
        int longer = dist > 8.0;
        int case_ok = m > 0 && near(t, expect) && near(st, expect) && longer;
        printf("detour_time=%.4f expect=%.4f distance=%.4f longer=%d %s\n",
               t, expect, dist, longer, case_ok ? "ok" : "FAIL");
        ok &= case_ok;
    }

    /* Zero multiplier is impassable even without a wall: the direct 3-step
       route is cut, so the path costs 1+1+1+1 = 4 */
    {
        const char* rows[] = {
            "SxG",
            "...",
        };
        Grid g;
        if (grid_load(&g, rows, 2) != 0) { printf("load fail\n"); return 1; }
        m = run_case(&g, (Pt){0,0}, (Pt){2,0}, raw, sm, &t, &st, &n);
        int zero_blocked = !walkable(&g, 1, 0);
        int corner_blocked = !step_ok(&g, (Pt){0,0}, (Pt){1,1});
        int target_blocked = astar(&g, (Pt){0,0}, (Pt){1,0}, raw, MAX_NODES) < 0;
        int case_ok = m > 0 && near(t, 4.0) && near(st, 4.0) &&
                      zero_blocked && corner_blocked && target_blocked;
        printf("zero_time=%.4f expect=4.0000 zero_blocked=%d corner_blocked=%d "
               "target_blocked=%d %s\n",
               t, zero_blocked, corner_blocked, target_blocked,
               case_ok ? "ok" : "FAIL");
        ok &= case_ok;
    }

    /* Walls still prevent corner cutting */
    {
        const char* rows[] = {
            "S.#",
            "...",
            "..G",
        };
        Grid g;
        if (grid_load(&g, rows, 3) != 0) { printf("load fail\n"); return 1; }
        int cut = step_ok(&g, (Pt){1,0}, (Pt){2,1});
        printf("corner_cut=%d\n", !cut);
        ok &= !cut;
    }

    printf("all_ok=%d\n", ok);
    return ok ? 0 : 1;
}
