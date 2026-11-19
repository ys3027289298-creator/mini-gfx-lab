/* mini-flowfield: one reverse Dijkstra distance field from the target, turned
   into a per-cell direction field; any number of units then just integrate
   along the field. Compared against per-unit A* (with LOS smoothing) on
   arrival time and total planning cost. Rebuild cost depends only on grid
   size, never on unit count. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define GW 128
#define GH 96
#define NCELLS (GW * GH)
#define INF 1e30
#define SQRT2 1.4142135623730951

static unsigned char blocked[NCELLS];
static double distf[NCELLS];          /* distance field: cost to target */
static float dirx[NCELLS], diry[NCELLS]; /* direction field: unit vector toward target */

static const int DIRS[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

/* diagonal step is legal only if both orthogonal neighbours are free */
static int step_ok(int x, int y, int nx, int ny) {
    if (nx < 0 || ny < 0 || nx >= GW || ny >= GH) return 0;
    if (blocked[ny * GW + nx]) return 0;
    int dx = nx - x, dy = ny - y;
    if (dx != 0 && dy != 0) {
        if (blocked[y * GW + nx] || blocked[ny * GW + x]) return 0;
    }
    return 1;
}

static int cell_blocked(double x, double y) {
    int ix = (int)x, iy = (int)y;
    if (ix < 0) ix = 0;
    if (ix >= GW) ix = GW - 1;
    if (iy < 0) iy = 0;
    if (iy >= GH) iy = GH - 1;
    return blocked[iy * GW + ix];
}

/* deterministic xorshift so runs are reproducible */
static unsigned long long rng_state = 0x9e3779b97f4a7c15ull;
static unsigned long long rng_next(void) {
    unsigned long long x = rng_state;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    rng_state = x; return x;
}
static double rng_double(void) { return (rng_next() >> 11) * (1.0 / 9007199254740992.0); }

static void gen_map(void) {
    for (int y = 0; y < GH; y++)
        for (int x = 0; x < GW; x++) {
            int b = (x == 0 || y == 0 || x == GW - 1 || y == GH - 1);
            if (!b && rng_double() < 0.10) b = 1;
            blocked[y * GW + x] = (unsigned char)b;
        }
    /* long walls with 2-cell gaps to force real detours */
    for (int w = 0; w < 6; w++) {
        int vert = (int)(rng_next() & 1);
        int pos = 8 + (int)(rng_next() % (vert ? GW - 16 : GH - 16));
        int len = (vert ? GH : GW) * 2 / 3;
        int start = 4 + (int)(rng_next() % 8);
        int gap = start + 1 + (int)(rng_next() % (len - 2)); /* gap inside the wall */
        for (int i = start; i < start + len; i++) {
            if (i == gap || i == gap + 1) continue;
            int x = vert ? pos : i, y = vert ? i : pos;
            if (x > 0 && y > 0 && x < GW - 1 && y < GH - 1) blocked[y * GW + x] = 1;
        }
    }
}

/* binary min-heap of (cell, key) pairs; the key is frozen at push time so
   later distance updates cannot corrupt heap order (lazy deletion) */
#define HEAP_CAP (NCELLS * 16)
static int heap_idx[HEAP_CAP];
static double heap_key[HEAP_CAP];
static int hn;

static void hpush(int i, double k) {
    if (hn >= HEAP_CAP) { fprintf(stderr, "heap overflow\n"); exit(1); }
    int c = hn++;
    while (c > 0) {
        int p = (c - 1) / 2;
        if (heap_key[p] <= k) break;
        heap_idx[c] = heap_idx[p]; heap_key[c] = heap_key[p]; c = p;
    }
    heap_idx[c] = i; heap_key[c] = k;
}
static int hpop(double* k) {
    int top = heap_idx[0]; *k = heap_key[0];
    hn--;
    if (hn > 0) {
        int li = heap_idx[hn], c = 0;
        double lk = heap_key[hn];
        for (;;) { /* sift the relocated last element down: compare against lk */
            int l = 2 * c + 1, r = l + 1, m = -1;
            double best = lk;
            if (l < hn && heap_key[l] < best) { m = l; best = heap_key[l]; }
            if (r < hn && heap_key[r] < best) { m = r; best = heap_key[r]; }
            if (m < 0) break;
            heap_idx[c] = heap_idx[m]; heap_key[c] = heap_key[m]; c = m;
        }
        heap_idx[c] = li; heap_key[c] = lk;
    }
    return top;
}

/* reverse Dijkstra from target + direction extraction; this is the only
   planning step, and its cost is O(cells log cells) regardless of units */
static void flow_build(int tx, int ty) {
    for (int i = 0; i < NCELLS; i++) distf[i] = INF;
    hn = 0;
    int ti = ty * GW + tx;
    distf[ti] = 0.0; hpush(ti, 0.0);
    while (hn > 0) {
        double k;
        int c = hpop(&k);
        if (k > distf[c]) continue; /* stale heap entry */
        int cx = c % GW, cy = c / GW;
        for (int d = 0; d < 8; d++) {
            int nx = cx + DIRS[d][0], ny = cy + DIRS[d][1];
            if (!step_ok(cx, cy, nx, ny)) continue;
            double step = (DIRS[d][0] && DIRS[d][1]) ? SQRT2 : 1.0;
            double nd = distf[c] + step;
            int ni = ny * GW + nx;
            if (nd < distf[ni]) {
                distf[ni] = nd; hpush(ni, nd);
            }
        }
    }
    for (int y = 0; y < GH; y++)
        for (int x = 0; x < GW; x++) {
            int c = y * GW + x;
            dirx[c] = 0.0f; diry[c] = 0.0f;
            if (blocked[c] || distf[c] >= INF) continue;
            double bd = distf[c];
            int bx = 0, by = 0, found = 0;
            for (int d = 0; d < 8; d++) {
                int nx = x + DIRS[d][0], ny = y + DIRS[d][1];
                if (!step_ok(x, y, nx, ny)) continue;
                double nd = distf[ny * GW + nx];
                if (nd < bd - 1e-9) { bd = nd; bx = DIRS[d][0]; by = DIRS[d][1]; found = 1; }
            }
            if (found) {
                double inv = 1.0 / hypot((double)bx, (double)by);
                dirx[c] = (float)(bx * inv); diry[c] = (float)(by * inv);
            }
        }
}

/* bilinear blend of the surrounding cell directions. Cells with a zero
   vector (blocked, unreachable, or the target itself) are SKIPPED and the
   remaining weights renormalized: a zero contributor would otherwise eat
   the blended component as the unit approaches that cell's boundary, and
   the unit would crawl toward the edge asymptotically without ever
   crossing (Zeno stall). */
static void sample_dir(double x, double y, double* dx, double* dy) {
    int x0 = (int)floor(x), y0 = (int)floor(y);
    if (x0 < 0) x0 = 0;
    if (x0 >= GW - 1) x0 = GW - 2;
    if (y0 < 0) y0 = 0;
    if (y0 >= GH - 1) y0 = GH - 2;
    double fx = x - x0, fy = y - y0;
    if (fx < 0) fx = 0;
    if (fx > 1) fx = 1;
    if (fy < 0) fy = 0;
    if (fy > 1) fy = 1;
    double vx = 0, vy = 0, wsum = 0;
    for (int j = 0; j < 2; j++)
        for (int i = 0; i < 2; i++) {
            double w = (i ? fx : 1 - fx) * (j ? fy : 1 - fy);
            int c = (y0 + j) * GW + (x0 + i);
            if (dirx[c] == 0 && diry[c] == 0) continue;
            vx += w * dirx[c]; vy += w * diry[c]; wsum += w;
        }
    if (wsum > 1e-9) { vx /= wsum; vy /= wsum; }
    double len = hypot(vx, vy);
    if (len < 1e-6) { /* degenerate blend: fall back to the current cell */
        int c = (int)y * GW + (int)x;
        if (c >= 0 && c < NCELLS) { vx = dirx[c]; vy = diry[c]; len = hypot(vx, vy); }
    }
    if (len < 1e-6) { *dx = 0; *dy = 0; return; }
    *dx = vx / len; *dy = vy / len;
}

/* integrate along the field until within 0.7 cells of the target;
   returns arrival time, or -1 if stuck/unreachable */
static double flow_arrival(double sx, double sy, double tx, double ty,
                           double speed, double dt, double tmax) {
    double x = sx, y = sy, t = 0;
    int itx = (int)tx, ity = (int)ty;
    while (t < tmax) {
        double rx = tx - x, ry = ty - y;
        /* 0.75 > sqrt(0.5): covers the cell-corner saddle where the blended
           field cancels in an epsilon neighbourhood of the goal corner */
        if (rx * rx + ry * ry < 0.5625) return t;
        if ((int)x == itx && (int)y == ity) return t; /* inside the goal cell */
        double dx, dy;
        sample_dir(x, y, &dx, &dy);
        if (dx == 0 && dy == 0) return -1;
        double step = speed * dt;
        double nx = x + dx * step, ny = y + dy * step;
        int legal = !cell_blocked(nx, ny);
        if (legal && (int)nx != (int)x && (int)ny != (int)y) {
            /* crossing a cell corner diagonally: same no-corner-cut rule as
               the grid, or the unit could slip into a pocket the field
               (correctly) never routes through */
            if (cell_blocked(nx, y) || cell_blocked(x, ny)) legal = 0;
        }
        if (!legal) {
            /* The blend can be poisoned by cells this unit cannot legally
               flow toward (blocked, corner-cut forbidden, or the target's
               own zero vector). Fall back to this cell's own grid
               direction: it points to a step_ok-legal neighbour by
               construction, so this move always follows the true field. */
            int c = (int)y * GW + (int)x;
            dx = dirx[c]; dy = diry[c];
            if (dx == 0 && dy == 0) return -1;
            nx = x + dx * step; ny = y + dy * step;
            if (cell_blocked(nx, ny)) return -1;
        }
        x = nx; y = ny;
        t += dt;
    }
    return -1;
}

/* ---- per-unit A* baseline (binary heap, octile heuristic) ---- */
static double ag[NCELLS];
static int aparent[NCELLS];
static unsigned char astate[NCELLS];

static double heur(int x, int y, int tx, int ty) {
    double dx = fabs((double)(x - tx)), dy = fabs((double)(y - ty));
    double mn = dx < dy ? dx : dy, mx = dx > dy ? dx : dy;
    return mx + (SQRT2 - 1.0) * mn;
}

static int astar(int sx, int sy, int tx, int ty, int* out, int max_out) {
    if (blocked[sy * GW + sx] || blocked[ty * GW + tx]) return -1;
    memset(astate, 0, NCELLS);
    hn = 0;
    int si = sy * GW + sx, ti = ty * GW + tx;
    ag[si] = 0.0; aparent[si] = -1; astate[si] = 1;
    hpush(si, heur(sx, sy, tx, ty));
    int found = 0;
    while (hn > 0) {
        double k;
        int c = hpop(&k);
        if (astate[c] == 2) continue;
        astate[c] = 2;
        if (c == ti) { found = 1; break; }
        int cx = c % GW, cy = c / GW;
        for (int d = 0; d < 8; d++) {
            int nx = cx + DIRS[d][0], ny = cy + DIRS[d][1];
            if (!step_ok(cx, cy, nx, ny)) continue;
            int ni = ny * GW + nx;
            if (astate[ni] == 2) continue;
            double step = (DIRS[d][0] && DIRS[d][1]) ? SQRT2 : 1.0;
            double ng = ag[c] + step;
            if (astate[ni] == 0 || ng < ag[ni]) {
                ag[ni] = ng; aparent[ni] = c; astate[ni] = 1;
                hpush(ni, ng + heur(nx, ny, tx, ty));
            }
        }
    }
    if (!found) return -1;
    int n = 0, c = ti;
    while (c != -1) { n++; c = aparent[c]; }
    if (n > max_out) return -1;
    c = ti;
    for (int i = n - 1; i >= 0; i--) { out[i] = c; c = aparent[c]; }
    return n;
}

static int los(int x0, int y0, int x1, int y1) {
    int steps = abs(x1 - x0) + abs(y1 - y0);
    if (steps == 0) return 1;
    int px = x0, py = y0;
    for (int i = 1; i <= steps; i++) {
        double t = (double)i / steps;
        int cx = (int)lround(x0 + (x1 - x0) * t);
        int cy = (int)lround(y0 + (y1 - y0) * t);
        if (cx == px && cy == py) continue;
        if (!step_ok(px, py, cx, cy)) return 0;
        px = cx; py = cy;
    }
    return px == x1 && py == y1;
}

static int smooth_path(const int* in, int n, int* out, int max_out) {
    if (n <= 0) return 0;
    int m = 0, anchor = 0;
    out[m++] = in[0];
    while (anchor < n - 1) {
        int next = anchor + 1;
        for (int j = n - 1; j > anchor; j--) {
            if (los(in[anchor] % GW, in[anchor] / GW, in[j] % GW, in[j] / GW)) { next = j; break; }
        }
        if (m >= max_out) return -1;
        out[m++] = in[next];
        anchor = next;
    }
    return m;
}

static double path_length(const int* p, int n) {
    double s = 0.0;
    for (int i = 1; i < n; i++)
        s += hypot((double)(p[i] % GW - p[i-1] % GW), (double)(p[i] / GW - p[i-1] / GW));
    return s;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

#define NUNITS 500
#define SPEED 6.0
#define DT (1.0 / 60.0)

int main(void) {
    gen_map();
    int tx = GW - 12, ty = GH - 12;
    for (int y = ty - 2; y <= ty + 2; y++)          /* keep the goal area open */
        for (int x = tx - 2; x <= tx + 2; x++)
            blocked[y * GW + x] = 0;

    int nblocked = 0;
    for (int i = 0; i < NCELLS; i++) nblocked += blocked[i];
    printf("map=%dx%d blocked=%d target=(%d,%d) units=%d speed=%.1f cells/s\n",
           GW, GH, nblocked, tx, ty, NUNITS, SPEED);

    /* rebuild benchmark: cost is a function of the grid only */
    double sum = 0, worst = 0;
    for (int r = 0; r < 50; r++) {
        double t0 = now_ms();
        flow_build(tx, ty);
        double e = now_ms() - t0;
        sum += e; if (e > worst) worst = e;
    }
    double rebuild_ms = sum / 50;
    printf("flow rebuild: avg=%.2fms worst=%.2fms over 50 runs (budget 50ms)\n",
           rebuild_ms, worst);

    /* spawn units on random walkable cells */
    static double ux[NUNITS], uy[NUNITS];
    for (int i = 0; i < NUNITS; i++) {
        int x, y;
        do {
            x = 1 + (int)(rng_next() % (GW - 2));
            y = 1 + (int)(rng_next() % (GH - 2));
        } while (blocked[y * GW + x] || (x == tx && y == ty));
        ux[i] = x + 0.5; uy[i] = y + 0.5;
    }

    /* flow field: one rebuild already done, units only integrate */
    int flow_ok = 0, flow_stuck = 0;
    double flow_tsum = 0;
    for (int i = 0; i < NUNITS; i++) {
        double t = flow_arrival(ux[i], uy[i], tx + 0.5, ty + 0.5, SPEED, DT, 300.0);
        if (t >= 0) { flow_ok++; flow_tsum += t; } else flow_stuck++;
    }

    /* per-unit A*: every unit pays its own query, then follows smoothed path */
    static int raw[NCELLS], sm[NCELLS];
    int astar_ok = 0;
    double astar_tsum = 0, astar_plan_ms = 0;
    for (int i = 0; i < NUNITS; i++) {
        double t0 = now_ms();
        int n = astar((int)ux[i], (int)uy[i], tx, ty, raw, NCELLS);
        int m = n > 0 ? smooth_path(raw, n, sm, NCELLS) : -1;
        astar_plan_ms += now_ms() - t0;
        if (m > 0) { astar_ok++; astar_tsum += path_length(sm, m) / SPEED; }
    }

    double flow_mean = flow_ok ? flow_tsum / flow_ok : 0;
    double astar_mean = astar_ok ? astar_tsum / astar_ok : 0;
    double gap = astar_mean > 0 ? (flow_mean - astar_mean) / astar_mean * 100.0 : 0;

    printf("flowfield: arrived=%d stuck=%d mean_t=%.2fs planning_total=%.2fms (1 rebuild)\n",
           flow_ok, flow_stuck, flow_mean, rebuild_ms);
    printf("astar:     arrived=%d mean_t=%.2fs planning_total=%.2fms (%.2fms/query x %d)\n",
           astar_ok, astar_mean, astar_plan_ms,
           NUNITS ? astar_plan_ms / NUNITS : 0, NUNITS);
    printf("arrival gap: %+.1f%% (flow vs astar)\n", gap);
    if (astar_plan_ms > 0)
        printf("planning cost: astar spends %.0fx the flow rebuild for %d units; "
               "flow rebuild is independent of unit count\n",
               astar_plan_ms / rebuild_ms, NUNITS);

    int ok = 1;
    if (rebuild_ms >= 50.0) { printf("FAIL: rebuild over 50ms budget\n"); ok = 0; }
    if (flow_ok != astar_ok) { printf("FAIL: reachability mismatch\n"); ok = 0; }
    if (gap > 25.0) { printf("FAIL: flow arrivals too slow\n"); ok = 0; }
    printf(ok ? "OK\n" : "FAILED\n");
    return ok ? 0 : 1;
}
