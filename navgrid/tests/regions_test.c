/* regions_test: exercises src/regions.c.

   Checks, in order:
     1. full partition of 102400 cells vs a union-find reference;
     2. incremental toggles, each compared cell-for-cell with a fresh
        chunk-constrained union-find partition;
     3. locality: after a toggle, only cells of the affected region(s) change;
     4. build cost, budget 20 ms for 100k cells;
     5. region count / render batch table across chunk sizes. */
#define _POSIX_C_SOURCE 200809L
#include "regions.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define N_MAIN 102400            /* 320 x 320 */
#define MAIN_W 320
#define MAIN_H 320
#define BUDGET_MS 20.0

static int failures = 0;
static void check(int cond, const char *what) {
    if (!cond) { printf("FAIL %s\n", what); failures++; }
}

/* ---------------------------------------------------------------- rng */
static unsigned long long rng_state = 0x9E3779B97F4A7C15ull;
static unsigned rng_next(void) {
    rng_state ^= rng_state >> 12;
    rng_state ^= rng_state << 25;
    rng_state ^= rng_state >> 27;
    return (unsigned)((rng_state * 0x2545F4914F6CDD1Dull) >> 32);
}
static double rng_unit(void) { return (rng_next() & 0xFFFFFF) / 16777216.0; }

/* -------------------------------------------------------------- timing */
static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}
static int cmp_double(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

/* ------------------------------------------------------------ reference */
/* Chunk-constrained union-find partition: 4-neighbour unions only between
   cells in the same render chunk.  Ids are canonical (min linear index+1). */
static int *ref_partition(const unsigned char *blocked, int w, int h, int chunk) {
    int n = w * h;
    int *parent = (int *)malloc((size_t)n * sizeof(int));
    int *sz = (int *)malloc((size_t)n * sizeof(int));
    int *minidx = (int *)malloc((size_t)n * sizeof(int));
    int *label = (int *)malloc((size_t)n * sizeof(int));
    if (!parent || !sz || !minidx || !label) { free(parent); free(sz); free(minidx); free(label); return NULL; }
    for (int i = 0; i < n; i++) { parent[i] = i; sz[i] = 1; minidx[i] = i; label[i] = 0; }

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int i = y * w + x;
            if (blocked[i]) continue;
            if (x + 1 < w && (x + 1) / chunk == x / chunk) {
                int j = i + 1;
                if (!blocked[j]) {
                    int a = i, b = j;
                    while (parent[a] != a) a = parent[a];
                    while (parent[b] != b) b = parent[b];
                    if (a != b) {
                        if (sz[a] < sz[b]) { int t = a; a = b; b = t; }
                        parent[b] = a; sz[a] += sz[b];
                        if (minidx[b] < minidx[a]) minidx[a] = minidx[b];
                    }
                }
            }
            if (y + 1 < h && (y + 1) / chunk == y / chunk) {
                int j = i + w;
                if (!blocked[j]) {
                    int a = i, b = j;
                    while (parent[a] != a) a = parent[a];
                    while (parent[b] != b) b = parent[b];
                    if (a != b) {
                        if (sz[a] < sz[b]) { int t = a; a = b; b = t; }
                        parent[b] = a; sz[a] += sz[b];
                        if (minidx[b] < minidx[a]) minidx[a] = minidx[b];
                    }
                }
            }
        }
    }
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int i = y * w + x;
            if (blocked[i]) continue;
            int r = i;
            while (parent[r] != r) r = parent[r];
            int m = minidx[r];
            int rr = i;
            while (parent[rr] != rr) { int next = parent[rr]; parent[rr] = r; rr = next; }
            label[i] = m + 1;
        }
    }
    free(parent); free(sz); free(minidx);
    return label;
}

static int labels_match(const int *a, const int *b, int n, const char *tag) {
    for (int i = 0; i < n; i++) {
        if (a[i] != b[i]) {
            printf("FAIL %s mismatch at idx %d: inc=%d ref=%d\n", tag, i, a[i], b[i]);
            failures++;
            return 0;
        }
    }
    return 1;
}

/* ------------------------------------------------------------- maps */
static void gen_open(unsigned char *b, int w, int h, double density) {
    for (int i = 0; i < w * h; i++) b[i] = rng_unit() < density;
}
/* rooms separated by walls, with 1-cell doors cut at random spans */
static void gen_rooms(unsigned char *b, int w, int h, int room) {
    memset(b, 0, (size_t)w * h);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if ((x % room == 0) || (y % room == 0)) b[y * w + x] = 1;
    for (int y = room; y < h; y += room)
        for (int x = room; x < w; x += room) {
            b[y * w + (x - 1 - (int)(rng_next() % 3))] = 0;  /* door in the wall */
            b[(y - 1 - (int)(rng_next() % 3)) * w + x] = 0;
        }
}
/* winding corridors: every other column is a wall with a gap */
static void gen_maze(unsigned char *b, int w, int h) {
    memset(b, 0, (size_t)w * h);
    for (int x = 1; x < w; x += 2)
        for (int y = 0; y < h; y++) b[y * w + x] = 1;
    for (int x = 1; x < w; x += 2) {
        int gap = (int)(rng_next() % (unsigned)h);
        b[gap * w + x] = 0;
        if (gap > 0) b[(gap - 1) * w + x] = 0;
    }
}

/* -------------------------------------------------- locality measurement */
/* After a toggle, count changed cells and verify each changed cell belonged
   (before the toggle) to one of `allowed_ids` or is the toggled cell. */
static int locality_ok(const int *before, const int *after, int n,
                       int idx, const int *allowed_ids, int n_allowed) {
    for (int i = 0; i < n; i++) {
        if (before[i] == after[i]) continue;
        if (i == idx) continue;
        int old = before[i];
        int ok = 0;
        for (int k = 0; k < n_allowed; k++) if (allowed_ids[k] == old) { ok = 1; break; }
        if (!ok) return 0;
    }
    return 1;
}

/* ids of walkable in-chunk 4-neighbours, deduplicated */
static int neighbour_ids(const int *label, int w, int h, int chunk, int idx, int out[4]) {
    int x = idx % w, y = idx / w;
    int x0 = (x / chunk) * chunk, y0 = (y / chunk) * chunk;
    int x1 = x0 + chunk; if (x1 > w) x1 = w;
    int y1 = y0 + chunk; if (y1 > h) y1 = h;
    int nb[4], nn = 0, m = 0;
    if (x > x0) nb[nn++] = idx - 1;
    if (x + 1 < x1) nb[nn++] = idx + 1;
    if (y > y0) nb[nn++] = idx - w;
    if (y + 1 < y1) nb[nn++] = idx + w;
    for (int i = 0; i < nn; i++) {
        int id = label[nb[i]];
        if (id <= 0) continue;
        int dup = 0;
        for (int j = 0; j < m; j++) if (out[j] == id) { dup = 1; break; }
        if (!dup) out[m++] = id;
    }
    return m;
}

/* -------------------------------------------------- incremental grid run */
/* Toggle cells one at a time, forcing a roughly even split of opens and
   closes so both relabel paths get exercised, and compare every step with a
   fresh union-find partition.  Returns 1 when exact, local and valid. */
static int incremental_run(int shape, int chunk, int toggles,
                           int *out_opens, int *out_closes,
                           double *out_median_ms, double *out_p99_ms,
                           int *out_worst_cells) {
    int n = MAIN_W * MAIN_H;
    RegionGrid g;
    unsigned char *base = (unsigned char *)malloc((size_t)n);
    int *before = (int *)malloc((size_t)n * sizeof(int));
    if (!base || !before) { free(base); free(before); return 0; }
    rng_state = 0xC0FFEE + (unsigned)shape * 1013u + (unsigned)chunk;
    if (shape == 0) gen_open(base, MAIN_W, MAIN_H, 0.05);
    else if (shape == 1) gen_rooms(base, MAIN_W, MAIN_H, 20);
    else gen_maze(base, MAIN_W, MAIN_H);

    if (rg_init(&g, MAIN_W, MAIN_H, chunk) != 0) { free(base); free(before); return 0; }
    memcpy(g.blocked, base, (size_t)n);
    int ok = rg_build(&g) >= 0 && rg_validate(&g);

    int exact = 1, local = 1, val = 1, opens = 0, closes = 0;
    int worst_cells = 0;
    double times[512];
    int n_times = 0;
    for (int t = 0; ok && t < toggles; t++) {
        /* pick a cell of the desired parity: blocked cells are opens */
        int want_open = (t & 1) == 0;
        int x = -1, y = -1;
        for (int tries = 0; tries < 64; tries++) {
            int cx = (int)(rng_next() % MAIN_W);
            int cy = (int)(rng_next() % MAIN_H);
            int blocked = g.blocked[cy * MAIN_W + cx] != 0;
            if (blocked == want_open) { x = cx; y = cy; break; }
        }
        if (x < 0) continue;
        int idx = y * MAIN_W + x;
        memcpy(before, g.label, (size_t)n * sizeof(int));

        int allowed[5], n_allowed = 0;
        if (g.blocked[idx]) {
            n_allowed = neighbour_ids(before, MAIN_W, MAIN_H, chunk, idx, allowed);
            opens++;
        } else {
            allowed[0] = before[idx];
            n_allowed = 1;
            closes++;
        }

        double t0 = now_ms();
        int nr = rg_toggle(&g, x, y);
        if (n_times < (int)(sizeof(times) / sizeof(times[0]))) times[n_times++] = now_ms() - t0;
        if (nr < 0) { exact = 0; break; }

        if (!locality_ok(before, g.label, n, idx, allowed, n_allowed)) local = 0;
        int changed = 0;
        for (int i = 0; i < n; i++) if (before[i] != g.label[i]) changed++;
        if (changed > worst_cells) worst_cells = changed;

        int *ref = ref_partition(g.blocked, MAIN_W, MAIN_H, chunk);
        if (!ref || !labels_match(g.label, ref, n, "incremental")) exact = 0;
        free(ref);
        if (!rg_validate(&g)) { val = 0; break; }
    }
    if (out_opens) *out_opens = opens;
    if (out_closes) *out_closes = closes;
    if (n_times > 0) {
        qsort(times, (size_t)n_times, sizeof(double), cmp_double);
        if (out_median_ms) *out_median_ms = times[n_times / 2];
        int p99 = (n_times * 99) / 100; if (p99 >= n_times) p99 = n_times - 1;
        if (out_p99_ms) *out_p99_ms = times[p99];
    } else {
        if (out_median_ms) *out_median_ms = 0.0;
        if (out_p99_ms) *out_p99_ms = 0.0;
    }
    if (out_worst_cells) *out_worst_cells = worst_cells;
    rg_free(&g);
    free(base); free(before);
    return ok && exact && local && val;
}

int main(void) {
    int n = MAIN_W * MAIN_H;
    printf("regions: cells=%d\n", n);

    /* ---------------- edge cases: tiny / clipped / degenerate grids ------- */
    {
        int edge_ok = 1;
        struct { int w, h, chunk; } shapes[] = {
            {1, 1, 1}, {1, 1, 7}, {7, 5, 16}, {17, 13, 5}, {32, 32, 1}, {3, 9, 2}
        };
        for (unsigned s = 0; s < sizeof(shapes) / sizeof(shapes[0]); s++) {
            RegionGrid g;
            int w = shapes[s].w, h = shapes[s].h, chunk = shapes[s].chunk;
            if (rg_init(&g, w, h, chunk) != 0) { edge_ok = 0; continue; }
            /* all blocked */
            for (int i = 0; i < w * h; i++) g.blocked[i] = 1;
            if (rg_build(&g) != 0 || !rg_validate(&g)) edge_ok = 0;
            /* open everything: one region per render chunk */
            for (int i = 0; i < w * h; i++) g.blocked[i] = 0;
            int expect = ((w + chunk - 1) / chunk) * ((h + chunk - 1) / chunk);
            if (rg_build(&g) != expect) edge_ok = 0;
            if (!rg_validate(&g)) edge_ok = 0;
            int *ref = ref_partition(g.blocked, w, h, chunk);
            if (!ref || !labels_match(g.label, ref, w * h, "edge-build")) edge_ok = 0;
            free(ref);

            /* every cell toggled twice, comparing with the reference each time */
            for (int y = 0; y < h && edge_ok; y++) {
                for (int x = 0; x < w && edge_ok; x++) {
                    if (rg_toggle(&g, x, y) < 0) { edge_ok = 0; break; }
                    int *r1 = ref_partition(g.blocked, w, h, chunk);
                    if (!r1 || !labels_match(g.label, r1, w * h, "edge-toggle-close")) edge_ok = 0;
                    free(r1);
                    if (!rg_validate(&g)) edge_ok = 0;
                    if (rg_toggle(&g, x, y) < 0) { edge_ok = 0; break; }
                    int *r2 = ref_partition(g.blocked, w, h, chunk);
                    if (!r2 || !labels_match(g.label, r2, w * h, "edge-toggle-open")) edge_ok = 0;
                    free(r2);
                    if (!rg_validate(&g)) edge_ok = 0;
                }
            }
            /* out-of-range toggle must be rejected */
            if (rg_toggle(&g, -1, 0) != -1 || rg_toggle(&g, w, 0) != -1) edge_ok = 0;
            rg_free(&g);
        }
        printf("edge_cases=%d\n", edge_ok);
        check(edge_ok, "edge cases");
    }

    /* ---------------- main map: three shapes, several chunk sizes -------- */
    const char *names[3] = {"open5%", "rooms", "maze"};
    printf("\n%-8s %-6s %-7s %-8s %-9s %-9s %-8s %-9s %-9s %-8s\n",
           "map", "chunk", "chunks", "regions", "reg/chunk", "max_reg/ch",
           "max_reg", "avg_reg", "batches", "cells/bt");
    int table_ok = 1;
    for (int shape = 0; shape < 3; shape++) {
        unsigned char *base = (unsigned char *)malloc((size_t)n);
        rng_state = 0x9E3779B97F4A7C15ull + (unsigned long long)shape * 7919;
        if (shape == 0) gen_open(base, MAIN_W, MAIN_H, 0.05);
        else if (shape == 1) gen_rooms(base, MAIN_W, MAIN_H, 20);
        else gen_maze(base, MAIN_W, MAIN_H);

        for (int chunk = 8; chunk <= 128; chunk *= 2) {
            RegionGrid g;
            if (rg_init(&g, MAIN_W, MAIN_H, chunk) != 0) { check(0, "init"); continue; }
            memcpy(g.blocked, base, (size_t)n);
            int regions = rg_build(&g);
            if (regions < 0 || !rg_validate(&g)) { check(0, "build/validate"); rg_free(&g); continue; }
            int *ref = ref_partition(g.blocked, MAIN_W, MAIN_H, chunk);
            if (!ref || !labels_match(g.label, ref, n, "build")) check(0, "build vs reference");
            free(ref);

            /* chunk query must agree with an independent scan of that chunk */
            int *ids = (int *)malloc((size_t)chunk * chunk * sizeof(int));
            for (int cy = 0; cy < g.ch && table_ok; cy++) {
                for (int cx = 0; cx < g.cw && table_ok; cx++) {
                    int got = rg_chunk_regions(&g, cx, cy, ids, chunk * chunk);
                    int want = 0;
                    int x0 = cx * chunk, y0 = cy * chunk;
                    int x1 = x0 + chunk; if (x1 > MAIN_W) x1 = MAIN_W;
                    int y1 = y0 + chunk; if (y1 > MAIN_H) y1 = MAIN_H;
                    for (int y = y0; y < y1; y++)
                        for (int x = x0; x < x1; x++) {
                            int id = g.label[y * MAIN_W + x];
                            if (id <= 0) continue;
                            int seen = 0;
                            for (int k = 0; k < want; k++) if (ids[k] == id) { seen = 1; break; }
                            if (!seen) ids[want++] = id;
                        }
                    if (got != want) { table_ok = 0; printf("FAIL chunk_regions %d,%d got=%d want=%d\n", cx, cy, got, want); }
                }
            }
            free(ids);

            RgStats st;
            rg_stats(&g, &st);
            int batches = st.regions;              /* one draw per region */
            double per_batch = batches ? (double)st.region_cells / batches : 0.0;
            printf("%-8s %-6d %-7d %-8d %-9.2f %-9d %-8d %-9.1f %-9d %-8.1f\n",
                   names[shape], chunk, st.chunks, st.regions,
                   st.chunks ? (double)st.regions / st.chunks : 0.0,
                   st.max_regions_per_chunk, st.max_region_cells, per_batch,
                   batches, per_batch);
            if (st.regions != regions) table_ok = 0;
            rg_free(&g);
        }
        free(base);
    }
    printf("table_ok=%d\n", table_ok);
    check(table_ok, "region/batch table");

    /* ---------------- build timing: 100k cells, 20 ms budget ------------- */
    {
        unsigned char *base = (unsigned char *)malloc((size_t)n);
        rng_state = 1234567;
        gen_open(base, MAIN_W, MAIN_H, 0.05);
        int timed_ok = 1;
        printf("\nbuild_timing cells=%d budget=%.0fms\n", n, BUDGET_MS);
        for (int chunk = 8; chunk <= 128; chunk *= 2) {
            double t[9];
            int regions = -1;
            RegionGrid g;
            if (rg_init(&g, MAIN_W, MAIN_H, chunk) != 0) { timed_ok = 0; break; }
            memcpy(g.blocked, base, (size_t)n);
            for (int r = 0; r < 9; r++) {
                /* clear before each timed build so every run is cold-state */
                memset(g.label, 0, (size_t)n * sizeof(int));
                double t0 = now_ms();
                regions = rg_build(&g);
                t[r] = now_ms() - t0;
            }
            qsort(t, 9, sizeof(double), cmp_double);
            int ok = regions >= 0 && t[0] < BUDGET_MS;
            printf("chunk=%-4d regions=%-6d best=%.3fms median=%.3fms ok=%d\n",
                   chunk, regions, t[0], t[4], ok);
            timed_ok &= ok;
            rg_free(&g);
        }
        free(base);
        printf("build_budget_ok=%d\n", timed_ok);
        check(timed_ok, "build under 20ms");
    }

    /* ---------------- incremental toggles vs union-find ------------------ */
    {
        const char *shape_names[3] = {"open5%", "rooms", "maze"};
        printf("\nincremental toggles (each step vs union-find)\n");
        printf("%-8s %-6s %-7s %-7s %-7s %-11s %-11s %-13s %-6s\n",
               "map", "chunk", "opens", "closes", "exact", "med_ms", "p99_ms",
               "worst_cells", "ok");
        int all_ok = 1;
        for (int shape = 0; shape < 3; shape++) {
            for (int chunk = 8; chunk <= 64; chunk *= 2) {
                int opens = 0, closes = 0, worst_cells = 0;
                double median_ms = 0.0, p99_ms = 0.0;
                int ok = incremental_run(shape, chunk, 200, &opens, &closes,
                                         &median_ms, &p99_ms, &worst_cells);
                printf("%-8s %-6d %-7d %-7d %-7d %-11.4f %-11.4f %-13d %-6d\n",
                       shape_names[shape], chunk, opens, closes, ok,
                       median_ms, p99_ms, worst_cells, ok);
                if (!ok) {
                    char msg[96];
                    snprintf(msg, sizeof(msg), "incremental %s chunk=%d",
                             shape_names[shape], chunk);
                    check(0, msg);
                }
                all_ok &= ok;
            }
        }
        printf("incremental_ok=%d\n", all_ok);
    }

    printf("\n%s failures=%d\n", failures ? "RESULT FAIL" : "RESULT OK", failures);
    return failures ? 1 : 0;
}
