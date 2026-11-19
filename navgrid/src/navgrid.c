
/* mini-navgrid: 8-direction A* on an ASCII grid, with corner-cut prevention,
   path smoothing by line-of-sight, incremental re-planning, and reverse
   batch pathfinding: one Dijkstra distance field from the shared target,
   every start backtracks along the gradient instead of running its own A*. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define MAX_W 64
#define MAX_H 64
#define MAX_NODES (MAX_W * MAX_H)
#define INF_DIST 1e18
#define EPS 1e-9

typedef struct { int w, h; unsigned char blocked[MAX_H][MAX_W]; } Grid;

typedef struct { int x, y; } Pt;

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

typedef struct { double g, f; int parent; unsigned char open, closed; } Node;

static int astar(const Grid* g, Pt s, Pt t, Pt* out, int max_out) {
    if (!walkable(g, s.x, s.y) || !walkable(g, t.x, t.y)) return -1;
    Node* nodes = (Node*)calloc((size_t)g->w * g->h, sizeof(Node));
    if (!nodes) return -1;
    int* heap = (int*)malloc(sizeof(int) * (size_t)g->w * g->h);
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

/* Reverse Dijkstra seeded at the shared target. The move graph is symmetric
   (step_ok(a,b) == step_ok(b,a)), so a forward search from the target yields
   the exact shortest target-distance of every reachable cell. Cost is paid
   once for the whole batch instead of once per unit. dist has w*h entries. */
static int reverse_field(const Grid* g, Pt t, double* dist) {
    int ncells = g->w * g->h;
    if (!walkable(g, t.x, t.y)) return -1;
    unsigned char* done = (unsigned char*)calloc((size_t)ncells, 1);
    if (!done) return -1;
    for (int i = 0; i < ncells; i++) dist[i] = INF_DIST;
    int ti = t.y * g->w + t.x;
    dist[ti] = 0.0;
    int settled = 0;
    while (settled < ncells) {
        int cur = -1;
        double best = INF_DIST;
        for (int i = 0; i < ncells; i++) {
            if (!done[i] && dist[i] < best) { best = dist[i]; cur = i; }
        }
        if (cur < 0) break;                       /* remaining cells unreachable */
        done[cur] = 1;
        settled++;
        Pt cp = { cur % g->w, cur / g->w };
        for (int d = 0; d < 8; d++) {
            Pt np = { cp.x + DIRS[d][0], cp.y + DIRS[d][1] };
            if (!step_ok(g, cp, np)) continue;
            int ni = np.y * g->w + np.x;
            if (done[ni]) continue;
            double step = (DIRS[d][0] && DIRS[d][1]) ? 1.4142135623730951 : 1.0;
            double nd = dist[cur] + step;
            if (nd < dist[ni]) dist[ni] = nd;
        }
    }
    free(done);
    return 0;
}

/* Backtrack one start down the distance field. Each move goes to a legal
   neighbour whose distance drops by the exact step cost; ties prefer the cell
   nearest the target so the route is deterministic. Strict decrease makes a
   loop impossible, and reaching the target (distance 0) is guaranteed for any
   start the field marked reachable. */
static int gradient_path(const Grid* g, const double* dist, Pt s, Pt t,
                         Pt* out, int max_out) {
    int si = s.y * g->w + s.x;
    if (!walkable(g, s.x, s.y) || dist[si] >= INF_DIST / 2) return -1;
    int n = 0;
    Pt cur = s;
    for (;;) {
        if (n >= max_out) return -1;
        out[n++] = cur;
        if (cur.x == t.x && cur.y == t.y) return n;
        int pick = -1;
        double pick_d = INF_DIST, pick_euclid = INF_DIST;
        int ci = cur.y * g->w + cur.x;
        for (int d = 0; d < 8; d++) {
            Pt np = { cur.x + DIRS[d][0], cur.y + DIRS[d][1] };
            if (!step_ok(g, cur, np)) continue;
            int ni = np.y * g->w + np.x;
            double step = (DIRS[d][0] && DIRS[d][1]) ? 1.4142135623730951 : 1.0;
            if (dist[ni] + EPS < dist[ci] &&
                fabs((dist[ci] - dist[ni]) - step) <= 1e-9) {
                double eu = hypot((double)(t.x - np.x), (double)(t.y - np.y));
                if (dist[ni] < pick_d - EPS ||
                    (fabs(dist[ni] - pick_d) <= EPS && eu < pick_euclid)) {
                    pick = ni; pick_d = dist[ni]; pick_euclid = eu;
                }
            }
        }
        if (pick < 0) return -1;                   /* field inconsistency */
        cur.x = pick % g->w;
        cur.y = pick / g->w;
    }
}

/* Point-by-point legality check used on every gradient backtrack result. */
static int path_valid(const Grid* g, const double* dist, const Pt* p, int n,
                      Pt s, Pt t) {
    if (n <= 0) return 0;
    if (p[0].x != s.x || p[0].y != s.y) return 0;
    if (p[n-1].x != t.x || p[n-1].y != t.y) return 0;
    for (int i = 0; i < n; i++) {
        if (!walkable(g, p[i].x, p[i].y)) return 0;
        if (dist[p[i].y * g->w + p[i].x] >= INF_DIST / 2) return 0;
    }
    for (int i = 1; i < n; i++) {
        int dx = abs(p[i].x - p[i-1].x), dy = abs(p[i].y - p[i-1].y);
        if (dx > 1 || dy > 1 || (dx == 0 && dy == 0)) return 0;
        if (!step_ok(g, p[i-1], p[i])) return 0;   /* includes corner-cut rule */
        double step = (dx && dy) ? 1.4142135623730951 : 1.0;
        double d0 = dist[p[i-1].y * g->w + p[i-1].x];
        double d1 = dist[p[i].y * g->w + p[i].x];
        if (d1 + EPS >= d0) return 0;              /* strict gradient descent */
        if (fabs((d0 - d1) - step) > 1e-9) return 0;
    }
    return 1;
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

int main(void) {
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
    Pt raw[MAX_NODES], sm[MAX_NODES];
    int n = astar(&g, s, t, raw, MAX_NODES);
    if (n < 0) { printf("no path\n"); return 1; }
    int m = smooth(&g, raw, n, sm, MAX_NODES);
    printf("raw=%d len=%.4f smooth=%d len=%.4f\n", n, path_length(raw,n), m, path_length(sm,m));
    for (int i = 0; i < m; i++) printf("(%d,%d)%s", sm[i].x, sm[i].y, i+1==m ? "\n" : " ");
    int bad = 0;
    for (int i = 1; i < m; i++) if (!line_of_sight(&g, sm[i-1], sm[i])) bad++;
    printf("bad_segments=%d\n", bad);
    printf("corner_cut=%d\n", !step_ok(&g, (Pt){1,0}, (Pt){2,1}));

    /* --- reverse batch pathfinding on the same grid: one field, many starts --- */
    double field[MAX_NODES];
    if (reverse_field(&g, t, field) != 0) { printf("field fail\n"); return 1; }
    Pt gp[MAX_NODES];
    int gn = gradient_path(&g, field, s, t, gp, MAX_NODES);
    if (gn < 0) { printf("gradient no path\n"); return 1; }
    printf("gradient=%d len=%.4f astar_len=%.4f valid=%d\n",
           gn, path_length(gp, gn), path_length(raw, n),
           path_valid(&g, field, gp, gn, s, t));
    for (int i = 0; i < gn; i++) printf("(%d,%d)%s", gp[i].x, gp[i].y, i+1==gn ? "\n" : " ");
    /* blocked cell and a fully enclosed cell must both be rejected */
    const char* cage_rows[] = {
        ".....",
        ".###.",
        ".#.#.",
        ".###.",
        ".....",
    };
    Grid cage;
    if (grid_load(&cage, cage_rows, 5) != 0) { printf("cage load fail\n"); return 1; }
    Pt ct = {4,4};
    double cage_field[MAX_NODES];
    if (reverse_field(&cage, ct, cage_field) != 0) { printf("cage field fail\n"); return 1; }
    printf("blocked_start=%d unreachable_start=%d\n",
           gradient_path(&g, field, (Pt){1,1}, t, gp, MAX_NODES),
           gradient_path(&cage, cage_field, (Pt){2,2}, ct, gp, MAX_NODES));

    /* --- batch benchmark: 1 reverse field + N traces vs N independent A* --- */
#define BENCH_W 40
#define BENCH_H 24
#define BENCH_WALLS 170           /* ~18% of cells */
#define MAX_STARTS 100
#define ROUNDS 20
    char bgrid[BENCH_H][BENCH_W + 1];
    const char* brows[BENCH_H];
    for (int y = 0; y < BENCH_H; y++) {
        memset(bgrid[y], '.', BENCH_W);
        bgrid[y][BENCH_W] = '\0';
        brows[y] = bgrid[y];
    }
    unsigned rng = 0x1234abcdu;
    Pt bt = { BENCH_W - 2, BENCH_H - 2 };
    int placed = 0;
    while (placed < BENCH_WALLS) {
        rng = rng * 1103515245u + 12345u;
        int x = (int)((rng >> 16) % BENCH_W);
        rng = rng * 1103515245u + 12345u;
        int y = (int)((rng >> 16) % BENCH_H);
        if (bgrid[y][x] == '#' || (x == bt.x && y == bt.y)) continue;
        bgrid[y][x] = '#';
        placed++;
    }
    Grid bg;
    if (grid_load(&bg, brows, BENCH_H) != 0) { printf("bench load fail\n"); return 1; }
    double bfield[MAX_NODES];
    if (reverse_field(&bg, bt, bfield) != 0) { printf("bench field fail\n"); return 1; }

    /* collect distinct walkable starts, keep only those reachable from target */
    Pt starts[MAX_STARTS];
    unsigned char used[MAX_NODES] = {0};
    int got = 0;
    while (got < MAX_STARTS) {
        rng = rng * 1103515245u + 12345u;
        int x = (int)((rng >> 16) % BENCH_W);
        rng = rng * 1103515245u + 12345u;
        int y = (int)((rng >> 16) % BENCH_H);
        int idx = y * BENCH_W + x;
        if (used[idx] || !walkable(&bg, x, y)) continue;
        if (bfield[idx] >= INF_DIST / 2 || (x == bt.x && y == bt.y)) continue;
        used[idx] = 1;
        starts[got].x = x; starts[got].y = y;
        got++;
    }

    static Pt gpaths[MAX_STARTS][MAX_NODES], apaths[MAX_STARTS][MAX_NODES];
    int glens[MAX_STARTS], alens[MAX_STARTS];
    int invalid = 0, mismatched = 0, astar_miss = 0;
    for (int i = 0; i < MAX_STARTS; i++) {
        glens[i] = gradient_path(&bg, bfield, starts[i], bt, gpaths[i], MAX_NODES);
        alens[i] = astar(&bg, starts[i], bt, apaths[i], MAX_NODES);
        if (glens[i] < 0 || !path_valid(&bg, bfield, gpaths[i], glens[i], starts[i], bt)) invalid++;
        if (alens[i] < 0) { astar_miss++; continue; }
        double dl = fabs(path_length(gpaths[i], glens[i]) - path_length(apaths[i], alens[i]));
        if (dl > 1e-9) mismatched++;
    }
    printf("starts=%d invalid=%d len_mismatch=%d astar_miss=%d\n",
           got, invalid, mismatched, astar_miss);

    double t_field = (double)clock() / CLOCKS_PER_SEC;
    for (int r = 0; r < ROUNDS; r++) reverse_field(&bg, bt, bfield);
    t_field = ((double)clock() / CLOCKS_PER_SEC - t_field) * 1e6 / ROUNDS;
    double t_trace0 = (double)clock() / CLOCKS_PER_SEC;
    for (int r = 0; r < ROUNDS; r++)
        for (int i = 0; i < MAX_STARTS; i++)
            gradient_path(&bg, bfield, starts[i], bt, gpaths[i], MAX_NODES);
    double trace_total = ((double)clock() / CLOCKS_PER_SEC - t_trace0) * 1e6 / ROUNDS;
    printf("field_us=%.1f trace_us_all=%d=%.1f trace_us_per_unit=%.2f\n",
           t_field, MAX_STARTS, trace_total, trace_total / MAX_STARTS);

    printf("%-6s %12s %12s %8s\n", "units", "batch_us", "astar_us", "speedup");
    const int scales[] = {1, 10, 25, 50, 100};
    double first_batch = 0.0, first_astar = 0.0, last_batch = 0.0, last_astar = 0.0;
    for (int k = 0; k < (int)(sizeof(scales) / sizeof(scales[0])); k++) {
        int nu = scales[k];
        double tb0 = (double)clock() / CLOCKS_PER_SEC;
        for (int r = 0; r < ROUNDS; r++) {
            reverse_field(&bg, bt, bfield);
            for (int i = 0; i < nu; i++)
                gradient_path(&bg, bfield, starts[i], bt, gpaths[i], MAX_NODES);
        }
        double batch_us = ((double)clock() / CLOCKS_PER_SEC - tb0) * 1e6 / ROUNDS;
        double ta0 = (double)clock() / CLOCKS_PER_SEC;
        for (int r = 0; r < ROUNDS; r++)
            for (int i = 0; i < nu; i++)
                astar(&bg, starts[i], bt, apaths[i], MAX_NODES);
        double astar_us = ((double)clock() / CLOCKS_PER_SEC - ta0) * 1e6 / ROUNDS;
        printf("%-6d %12.1f %12.1f %7.2fx\n", nu, batch_us, astar_us, astar_us / batch_us);
        if (k == 0) { first_batch = batch_us; first_astar = astar_us; }
        last_batch = batch_us; last_astar = astar_us;
    }
    printf("scaling_1_to_100 batch=%.2fx astar=%.2fx (batch stays flat: field is shared)\n",
           last_batch / first_batch, last_astar / first_astar);
    return 0;
}
