
/* mini-navgrid: 8-direction A* on an ASCII grid, with corner-cut prevention,
   path smoothing by line-of-sight, and incremental re-planning. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <time.h>

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
    /* Canonical direction: always sample from the lower-numbered endpoint.
       lround() tie-breaking is direction-dependent, so without this LOS(a,b)
       could differ from LOS(b,a) — which would break symmetric storage. */
    if (a.y * g->w + a.x > b.y * g->w + b.x) { Pt t = a; a = b; b = t; }
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

/* ---- precomputed visibility matrix: symmetric, bit-packed upper triangle ----
   A full byte matrix costs n*n bytes. LOS is symmetric (see line_of_sight),
   so only pairs i<j are stored, one bit each: n*(n-1)/2 bits ~= n^2/16 bytes,
   i.e. 1/16 of the full matrix (<= 1/8 including index arrays). Row i holds
   bits for j = i+1..n-1 starting at bit offset i*(2n-i-1)/2. A point query is
   a single bit test; a whole row can be decompressed to bytes on demand. */
typedef struct {
    int n;              /* number of walkable cells indexed */
    int gw, gh;         /* underlying grid dims */
    int* id;            /* gw*gh: cell -> compact id, -1 if blocked */
    Pt* pt;             /* compact id -> point */
    uint64_t* bits;     /* packed upper triangle */
    size_t words;       /* allocated words in bits */
} VisMat;

static size_t vm_row_start(int i, int n) { return (size_t)i * (2*(size_t)n - (size_t)i - 1) / 2; }

static void vm_free(VisMat* vm) { free(vm->id); free(vm->pt); free(vm->bits); }

static size_t vm_bytes(const VisMat* vm) {
    return vm->words * sizeof(uint64_t)
         + (size_t)vm->gw * vm->gh * sizeof(int)
         + (size_t)vm->n * sizeof(Pt);
}

static int vm_build(const Grid* g, VisMat* vm) {
    vm->gw = g->w; vm->gh = g->h;
    vm->id = (int*)malloc(sizeof(int) * (size_t)g->w * g->h);
    vm->pt = (Pt*)malloc(sizeof(Pt) * (size_t)g->w * g->h);
    if (!vm->id || !vm->pt) return -1;
    int n = 0;
    for (int y = 0; y < g->h; y++)
        for (int x = 0; x < g->w; x++) {
            int idx = y * g->w + x;
            if (g->blocked[y][x]) { vm->id[idx] = -1; continue; }
            vm->id[idx] = n;
            vm->pt[n] = (Pt){ x, y };
            n++;
        }
    vm->n = n;
    size_t total = (size_t)n * (n - 1) / 2;
    vm->words = (total + 63) / 64;
    vm->bits = (uint64_t*)calloc(vm->words, sizeof(uint64_t));
    if (!vm->bits) return -1;
    for (int i = 0; i < n; i++) {
        size_t base = vm_row_start(i, n);
        for (int j = i + 1; j < n; j++) {
            if (line_of_sight(g, vm->pt[i], vm->pt[j])) {
                size_t k = base + (size_t)(j - i - 1);
                vm->bits[k >> 6] |= 1ull << (k & 63);
            }
        }
    }
    return 0;
}

static int vm_visible_id(const VisMat* vm, int i, int j) {
    if (i == j) return 1;
    /* branchless min/max (cmov): a 50%-mispredicted swap branch would
       dominate the query cost on random pairs */
    int lo = i < j ? i : j;
    int hi = i < j ? j : i;
    size_t k = vm_row_start(lo, vm->n) + (size_t)(hi - lo - 1);
    return (int)((vm->bits[k >> 6] >> (k & 63)) & 1ull);
}

/* 0 if either endpoint is not walkable */
static int vm_visible(const VisMat* vm, Pt a, Pt b) {
    int i = vm->id[a.y * vm->gw + a.x], j = vm->id[b.y * vm->gw + b.x];
    if (i < 0 || j < 0) return 0;
    return vm_visible_id(vm, i, j);
}

/* batch point query: k pairs given as compact ids */
static void vm_visible_batch(const VisMat* vm, const int* ai, const int* bi, int k, unsigned char* out) {
    for (int q = 0; q < k; q++) out[q] = (unsigned char)vm_visible_id(vm, ai[q], bi[q]);
}

/* on-demand row decompression: out[j] = visible(i, j) for all j */
static void vm_row_bytes(const VisMat* vm, int i, unsigned char* out) {
    for (int j = 0; j < vm->n; j++) out[j] = (unsigned char)vm_visible_id(vm, i, j);
}

static uint64_t rng_s;
static uint64_t xr(void) { rng_s ^= rng_s << 13; rng_s ^= rng_s >> 7; rng_s ^= rng_s << 17; return rng_s; }

static double now_s(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
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

    /* ---- visibility matrix compression experiment (64x64, ~16% obstacles) ---- */
    enum { VW = 64, VH = 64 };
    Grid g2; g2.w = VW; g2.h = VH;
    rng_s = 0x9e3779b97f4a7c15ull;
    for (int y = 0; y < VH; y++)
        for (int x = 0; x < VW; x++)
            g2.blocked[y][x] = (x == 0 || y == 0 || x == VW - 1 || y == VH - 1) || (xr() % 1000 < 160);

    VisMat vm;
    double t0 = now_s();
    if (vm_build(&g2, &vm) != 0) { printf("vis build fail\n"); return 1; }
    double t_build = now_s() - t0;
    int vn = vm.n;
    int fails = 0;

    /* 1) memory: compressed vs full byte matrix, actual and projected to 100k cells */
    size_t full_bytes = (size_t)vn * vn;
    size_t comp_bytes = vm_bytes(&vm);
    size_t proj_n = 100000;
    size_t proj_full = proj_n * proj_n;
    size_t proj_comp = proj_n * (proj_n - 1) / 2 / 8 + proj_n * (sizeof(int) + sizeof(Pt));
    printf("vis: cells=%d build=%.2fs full=%zuB compressed=%zuB (1/%.1f) | 100k cells: %zuB -> %zuB (1/%.1f)\n",
           vn, t_build, full_bytes, comp_bytes, (double)full_bytes / comp_bytes,
           proj_full, proj_comp, (double)proj_full / proj_comp);
    if (comp_bytes * 8 > full_bytes) { printf("FAIL: memory ratio above 1/8\n"); fails++; }

    /* 2) correctness: matrix query must equal direct line_of_sight traversal */
    long mism = 0;
    for (long q = 0; q < 1000000; q++) {
        int i = (int)(xr() % vn), j = (int)(xr() % vn);
        if (vm_visible_id(&vm, i, j) != line_of_sight(&g2, vm.pt[i], vm.pt[j])) mism++;
    }
    /* on-demand row decompression agrees too */
    unsigned char* row = (unsigned char*)malloc((size_t)vn);
    for (int r = 0; r < 8; r++) {
        int i = (int)(xr() % vn);
        vm_row_bytes(&vm, i, row);
        for (int j = 0; j < vn; j++)
            if (row[j] != line_of_sight(&g2, vm.pt[i], vm.pt[j])) mism++;
    }
    free(row);
    /* blocked endpoints read as not visible */
    Pt bp = {-1, -1};
    for (int y = 0; y < VH && bp.x < 0; y++)
        for (int x = 0; x < VW; x++) if (g2.blocked[y][x]) { bp = (Pt){x, y}; break; }
    if (bp.x >= 0 && vm_visible(&vm, bp, vm.pt[0]) != 0) mism++;
    printf("vis: correctness mismatches=%ld\n", mism);
    if (mism) { printf("FAIL: query mismatch vs direct traversal\n"); fails++; }

    /* 3) batch throughput: compressed bit query vs full byte matrix query.
       Both sides run the same inline query-and-accumulate loop so the
       measurement isolates per-query cost. K is large so each rep runs
       ~0.1s and timer/scheduler noise averages out. */
    enum { K = 20000000 };
    int* ai = (int*)malloc(sizeof(int) * K);
    int* bi = (int*)malloc(sizeof(int) * K);
    unsigned char* out = (unsigned char*)malloc(K);
    for (int q = 0; q < K; q++) { ai[q] = (int)(xr() % vn); bi[q] = (int)(xr() % vn); }
    unsigned char* fullmat = (unsigned char*)malloc((size_t)vn * vn);
    for (int i = 0; i < vn; i++)
        for (int j = 0; j < vn; j++) fullmat[(size_t)i * vn + j] = (unsigned char)vm_visible_id(&vm, i, j);
    /* batch API agrees with point queries */
    vm_visible_batch(&vm, ai, bi, K, out);
    for (int q = 0; q < K; q++)
        if (out[q] != vm_visible_id(&vm, ai[q], bi[q])) mism++;
    if (mism) { printf("FAIL: batch API mismatch\n"); fails++; }
    volatile unsigned long sink = 0;
    double t_byte = 1e30, t_comp = 1e30;
    for (int rep = 0; rep < 5; rep++) {
        t0 = now_s();
        for (int q = 0; q < K; q++) sink += fullmat[(size_t)ai[q] * vn + bi[q]];
        double d = now_s() - t0; if (d < t_byte) t_byte = d;
        t0 = now_s();
        for (int q = 0; q < K; q++) sink += vm_visible_id(&vm, ai[q], bi[q]);
        d = now_s() - t0; if (d < t_comp) t_comp = d;
    }
    printf("vis: batch %d queries byte=%.3fs bit=%.3fs ratio=%.3f (sink=%lu)\n",
           K, t_byte, t_comp, t_comp / t_byte, sink);
    if (t_comp > t_byte * 1.2) { printf("FAIL: batch throughput drop over 20%%\n"); fails++; }

    free(ai); free(bi); free(out); free(fullmat);
    vm_free(&vm);
    printf("vis: %s\n", fails ? "FAIL" : "OK");
    return fails ? 1 : 0;
}
