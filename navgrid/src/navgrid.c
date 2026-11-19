
/* mini-navgrid: 8-direction A* on an ASCII grid, with corner-cut prevention,
   path smoothing by line-of-sight, and incremental re-planning. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_W 64
#define MAX_H 64
#define MAX_NODES (MAX_W * MAX_H)

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

/* --- boundary clearance: inset walkable area by the unit radius ---
   radius is in cell units, measured cell-center to cell-center. */

static double clearance_at(const Grid* g, int x, int y) {
    double best = 1e30;
    for (int yy = 0; yy < g->h; yy++)
        for (int xx = 0; xx < g->w; xx++)
            if (g->blocked[yy][xx]) {
                double d = hypot((double)(xx - x), (double)(yy - y));
                if (d < best) best = d;
            }
    return best;
}

/* a removed cell counts as a "narrow gap" cell only if the original
   obstacles sit within reach on BOTH sides along some axis — i.e. the
   inset seals a slit instead of just trimming a wall face */
static int gap_cell(const Grid* g, int x, int y, double radius) {
    static const int AXES[4][2] = {{1,0},{0,1},{1,1},{1,-1}};
    int reach = (int)ceil(2.0 * radius) + 1;
    for (int a = 0; a < 4; a++) {
        int dx = AXES[a][0], dy = AXES[a][1], both = 1;
        for (int sign = -1; sign <= 1; sign += 2) {
            int hit = 0;
            for (int s = 1; s <= reach && !hit; s++) {
                int nx = x + dx * s * sign, ny = y + dy * s * sign;
                if (!in_bounds(g, nx, ny)) break;
                if (g->blocked[ny][nx]) hit = 1;
            }
            if (!hit) { both = 0; break; }
        }
        if (both) return 1;
    }
    return 0;
}

/* block every free cell closer than radius to an obstacle; log the
   narrow gaps this removes; returns number of cells removed */
static int grid_inset(Grid* g, double radius) {
    unsigned char rem[MAX_H][MAX_W], gap[MAX_H][MAX_W], seen[MAX_H][MAX_W];
    memset(rem, 0, sizeof(rem));
    memset(gap, 0, sizeof(gap));
    memset(seen, 0, sizeof(seen));
    int removed = 0;
    for (int y = 0; y < g->h; y++)
        for (int x = 0; x < g->w; x++)
            if (!g->blocked[y][x] && clearance_at(g, x, y) < radius - 1e-9) {
                rem[y][x] = 1;
                removed++;
            }
    for (int y = 0; y < g->h; y++)
        for (int x = 0; x < g->w; x++)
            if (rem[y][x]) gap[y][x] = (unsigned char)gap_cell(g, x, y, radius);
    int comps = 0;
    for (int y = 0; y < g->h; y++)
        for (int x = 0; x < g->w; x++) {
            if (!gap[y][x] || seen[y][x]) continue;
            Pt stack[MAX_NODES]; int sn = 0;
            int n = 0, x0 = x, x1 = x, y0 = y, y1 = y;
            stack[sn++] = (Pt){ x, y };
            seen[y][x] = 1;
            while (sn > 0) {
                Pt c = stack[--sn];
                n++;
                if (c.x < x0) x0 = c.x;
                if (c.x > x1) x1 = c.x;
                if (c.y < y0) y0 = c.y;
                if (c.y > y1) y1 = c.y;
                for (int d = 0; d < 8; d++) {
                    int nx = c.x + DIRS[d][0], ny = c.y + DIRS[d][1];
                    if (in_bounds(g, nx, ny) && gap[ny][nx] && !seen[ny][nx]) {
                        seen[ny][nx] = 1;
                        stack[sn++] = (Pt){ nx, ny };
                    }
                }
            }
            comps++;
            printf("clearance: narrow gap closed: cells=%d bbox=(%d,%d)-(%d,%d)\n",
                   n, x0, y0, x1, y1);
        }
    if (!comps) printf("clearance: no narrow gaps\n");
    for (int y = 0; y < g->h; y++)
        for (int x = 0; x < g->w; x++)
            if (rem[y][x]) g->blocked[y][x] = 1;
    printf("clearance: radius=%.2f removed=%d walkable=%d\n",
           radius, removed, g->w * g->h - removed);
    return removed;
}

/* smallest distance from any walkable cell to the ORIGINAL obstacles;
   must be >= radius after grid_inset — the inset guarantee, checked
   not assumed. `obs` is a snapshot of the grid before inset. */
static double min_clearance(const Grid* g, const Grid* obs) {
    double best = 1e30;
    for (int y = 0; y < g->h; y++)
        for (int x = 0; x < g->w; x++)
            if (!g->blocked[y][x]) {
                double d = clearance_at(obs, x, y);
                if (d < best) best = d;
            }
    return best;
}

static void grid_dump(const Grid* g, const char* tag) {
    printf("%s\n", tag);
    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < g->w; x++) putchar(g->blocked[y][x] ? '#' : '.');
        putchar('\n');
    }
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

    /* --- boundary clearance demo: a 1-cell slit in a wall; the unit
       (radius 1.5) no longer fits, so the slit must seal and the path
       must detour around the wall's bottom edge --- */
    const char* rows2[] = {
        ".....#......",
        ".....#......",
        "S.........G.",
        ".....#......",
        ".....#......",
        "............",
        "............",
        "............",
    };
    Grid g2;
    if (grid_load(&g2, rows2, 8) != 0) { printf("load fail\n"); return 1; }
    Pt s2 = {0, 2}, t2 = {10, 2};
    double radius = 1.5;
    int n0 = astar(&g2, s2, t2, raw, MAX_NODES);
    if (n0 < 0) { printf("no path before inset\n"); return 1; }
    printf("before_inset: path=%d len=%.4f (straight through the slit)\n",
           n0, path_length(raw, n0));
    Grid orig = g2; /* snapshot of real obstacles for the clearance check */
    grid_inset(&g2, radius);
    grid_dump(&g2, "cleared grid:");
    int n1 = astar(&g2, s2, t2, raw, MAX_NODES);
    if (n1 < 0) { printf("FAIL: no path after clearance\n"); return 1; }
    int m1 = smooth(&g2, raw, n1, sm, MAX_NODES);
    double mc = min_clearance(&g2, &orig);
    printf("after_inset: path=%d len=%.4f smooth=%d min_clearance=%.4f radius=%.2f\n",
           n1, path_length(raw, n1), m1, mc, radius);
    for (int i = 0; i < m1; i++) printf("(%d,%d)%s", sm[i].x, sm[i].y, i+1==m1 ? "\n" : " ");
    printf("clearance_ok=%d reachable=%d\n", mc >= radius - 1e-9, n1 > 0);
    return 0;
}
