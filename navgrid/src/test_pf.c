/* test_pf.c - self-checks and the cross-algorithm benchmark table.
 *
 * Sections
 *   1. geometry / path-validity primitives
 *   2. per-algorithm sanity on the hand-written map
 *   3. cross-validation on three generated map classes
 *   4. incremental re-planning: edited maps must agree with A*
 *   5. auto-selection returns the planner the map profile implies
 *   6. benchmark table: time + expansions per algorithm per map class
 *
 * Run `test-pf bench` (or `make bench`) for section 6 only.
 */
#define _POSIX_C_SOURCE 199309L
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pf.h"

#define MAXW 160
#define MAXH 160

static int failures = 0;

static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* ------------------------------------------------------------- map making */

typedef struct {
    PfGrid grid;
    int w, h;
} Map;

static int map_from_rows(Map* m, const char* const* rows, int h) {
    if (pf_grid_load(&m->grid, rows, h) != 0) return -1;
    m->w = m->grid.w;
    m->h = m->grid.h;
    return 0;
}

static int map_blank(Map* m, int w, int h) {
    if (pf_grid_init(&m->grid, w, h) != 0) return -1;
    m->w = w;
    m->h = h;
    return 0;
}

static void map_free(Map* m) {
    pf_grid_free(&m->grid);
    m->w = m->h = 0;
}

static unsigned rng_state = 0x9e3779b9u;
static unsigned rng_next(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

/* Seeded generators so every section and the benchmark see the same maps. */
static void rng_seed(unsigned v) { rng_state = v ? v : 1u; }

/* Map class 1: open room with a few pillars (any-angle's home turf). */
static int make_open(Map* m, int w, int h) {
    if (map_blank(m, w, h) != 0) return -1;
    for (int i = 0; i < (w * h) / 120; i++) {
        int x = 2 + (int)(rng_next() % (unsigned)(w - 4));
        int y = 2 + (int)(rng_next() % (unsigned)(h - 4));
        pf_grid_set(&m->grid, x, y, 1);
    }
    return 0;
}

/* Map class 2: DFS-carved maze on a 2-cell pitch (A*'s home turf). */
static int make_maze(Map* m, int w, int h) {
    if (w % 2 == 0) w--;
    if (h % 2 == 0) h--;
    if (map_blank(m, w, h) != 0) return -1;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) pf_grid_set(&m->grid, x, y, 1);
    int* stack = (int*)malloc(sizeof(int) * (size_t)w * (size_t)h);
    if (!stack) return -1;
    int sp = 0;
    stack[sp++] = 1 * w + 1;
    pf_grid_set(&m->grid, 1, 1, 0);
    while (sp > 0) {
        int cur = stack[sp - 1];
        int cx = cur % w, cy = cur / w;
        int cand[4][2], nc = 0;
        static const int ddx[4] = { 2, -2, 0, 0 }, ddy[4] = { 0, 0, 2, -2 };
        for (int d = 0; d < 4; d++) {
            int nx = cx + ddx[d], ny = cy + ddy[d];
            if (nx <= 0 || ny <= 0 || nx >= w - 1 || ny >= h - 1) continue;
            if (!pf_grid_blocked(&m->grid, nx, ny)) continue;
            cand[nc][0] = nx; cand[nc][1] = ny; nc++;
        }
        if (nc == 0) { sp--; continue; }
        int pick = (int)(rng_next() % (unsigned)nc);
        int nx = cand[pick][0], ny = cand[pick][1];
        pf_grid_set(&m->grid, (cx + nx) / 2, (cy + ny) / 2, 0);
        pf_grid_set(&m->grid, nx, ny, 0);
        stack[sp++] = ny * w + nx;
    }
    free(stack);
    return 0;
}

/* Map class 3: rooms joined by doorways (HPA*'s home turf). */
static int make_rooms(Map* m, int w, int h) {
    if (map_blank(m, w, h) != 0) return -1;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (x == 0 || y == 0 || x == w - 1 || y == h - 1) pf_grid_set(&m->grid, x, y, 1);
    for (int y = 12; y < h - 1; y += 13) {
        for (int x = 1; x < w - 1; x++)
            if (x % 11 != 3) pf_grid_set(&m->grid, x, y, 1);
    }
    for (int x = 15; x < w - 1; x += 16) {
        for (int y = 1; y < h - 1; y++)
            if (y % 9 != 4) pf_grid_set(&m->grid, x, y, 1);
    }
    return 0;
}

typedef enum { MAP_OPEN, MAP_MAZE, MAP_ROOMS, MAP_CLASSES } MapClass;

static const char* class_name(MapClass c) {
    switch (c) {
    case MAP_OPEN: return "open";
    case MAP_MAZE: return "maze";
    case MAP_ROOMS: return "rooms";
    default: return "?";
    }
}

/* First walkable cell in reading order / last walkable cell. */
static PfPoint map_first_walkable(const Map* m) {
    for (int y = 0; y < m->h; y++)
        for (int x = 0; x < m->w; x++)
            if (pf_walkable(&m->grid, x, y)) return (PfPoint){ x, y };
    return (PfPoint){ -1, -1 };
}

static PfPoint map_last_walkable(const Map* m) {
    for (int y = m->h - 1; y >= 0; y--)
        for (int x = m->w - 1; x >= 0; x--)
            if (pf_walkable(&m->grid, x, y)) return (PfPoint){ x, y };
    return (PfPoint){ -1, -1 };
}

static const PfAlgo QUERY_ALGOS[4] = { PF_ASTAR, PF_ANYANGLE, PF_INCREMENTAL, PF_HIERARCHICAL };

/* ----------------------------------------------------------------- checks */

/* Four scratch path buffers, so section 3 can hold one result per algorithm
 * alive at the same time while comparing them. */
#define PF_PTS_CAP (MAXW * MAXH * 4)
static PfPoint* g_pts[4] = { NULL, NULL, NULL, NULL };

typedef struct {
    PfPath p;
    int rc;
} Result;

/* buf selects which retained scratch buffer receives the points. */
static Result run_buf(const PfSolver* s, PfPoint a, PfPoint b, PfAlgo algo, int buf) {
    Result r;
    pf_path_init(&r.p, g_pts[buf], PF_PTS_CAP);
    r.p.algo = algo;
    r.rc = pf_solve((PfSolver*)s, a, b, algo, &r.p);
    return r;
}

static Result run(const PfSolver* s, PfPoint a, PfPoint b, PfAlgo algo) {
    return run_buf(s, a, b, algo, 0);
}

static int every_point_walkable(const PfGrid* g, const PfPath* p) {
    for (int i = 0; i < p->n; i++)
        if (!pf_walkable(g, p->pts[i].x, p->pts[i].y)) return 0;
    return 1;
}

/* Cross-algorithm agreement, per map class:
 *   astar == lpa        point for point (same move rule, both optimal)
 *   anyangle cost       <= astar cost            (any-angle can only shorten)
 *   hpa cost            <= astar * (1 + slack)   (bounded abstraction error)
 * Every algorithm must produce a legal path under its own move rule. */

int main(int argc, char** argv) {
    for (int i = 0; i < 4; i++) {
        g_pts[i] = (PfPoint*)malloc(sizeof(PfPoint) * PF_PTS_CAP);
        if (!g_pts[i]) return 1;
    }

    /* ------------------------------------------------ 1. geometry */
    {
        const char* rows[] = {
            "S....#....",
            ".##..#....",
            ".#......#.",
            ".#.####.#.",
            ".....#..G.",
        };
        Map m;
        check("load_map", map_from_rows(&m, rows, 5) == 0);
        check("step_diag_ok", pf_step_ok(&m.grid, (PfPoint){ 3, 0 }, (PfPoint){ 4, 1 }) == 1);
        check("step_blocks_wall", pf_step_ok(&m.grid, (PfPoint){ 0, 0 }, (PfPoint){ 1, 0 }) == 1);
        check("step_blocks_corner", pf_step_ok(&m.grid, (PfPoint){ 1, 0 }, (PfPoint){ 2, 1 }) == 0);
        check("step_rejects_jump", pf_step_ok(&m.grid, (PfPoint){ 0, 0 }, (PfPoint){ 2, 0 }) == 0);
        check("los_clear", pf_line_clear(&m.grid, (PfPoint){ 0, 0 }, (PfPoint){ 0, 2 }) == 1);
        check("los_blocked", pf_line_clear(&m.grid, (PfPoint){ 1, 0 }, (PfPoint){ 1, 3 }) == 0);

        PfProfile prof;
        pf_profile(&m.grid, &prof);
        check("profile_walkable", prof.blocked_ratio > 0.0 && prof.blocked_ratio < 1.0);
        check("profile_sums", fabs(prof.corridor_ratio + prof.junction_ratio +
                                   prof.blocked_ratio - 1.0) < 1e-9 ||
                              prof.corridor_ratio + prof.junction_ratio <= 1.0);
        check("select_small_maze_is_astar", pf_select_algo(&m.grid, (PfPoint){ 0, 0 },
                                                            (PfPoint){ 8, 4 }, 0, NULL) == PF_ASTAR);
        map_free(&m);
    }

    /* ------------------------------------------------ 2. per-algorithm sanity */
    {
        const char* rows[] = {
            "S....#....",
            ".##..#....",
            ".#......#.",
            ".#.####.#.",
            ".....#..G.",
        };
        Map m;
        map_from_rows(&m, rows, 5);
        PfSolver* s = pf_solver_new(&m.grid);
        PfPoint a = { 0, 0 }, b = { 8, 4 };
        for (int i = 0; i < 4; i++) {
            Result r = run(s, a, b, QUERY_ALGOS[i]);
            char name[64];
            snprintf(name, sizeof(name), "%s_finds_path", pf_algo_name(QUERY_ALGOS[i]));
            check(name, r.rc == PF_OK && r.p.n >= 2);
            snprintf(name, sizeof(name), "%s_starts_at_start", pf_algo_name(QUERY_ALGOS[i]));
            check(name, r.rc == PF_OK && r.p.pts[0].x == a.x && r.p.pts[0].y == a.y);
            snprintf(name, sizeof(name), "%s_ends_at_goal", pf_algo_name(QUERY_ALGOS[i]));
            check(name, r.rc == PF_OK && r.p.pts[r.p.n - 1].x == b.x && r.p.pts[r.p.n - 1].y == b.y);
            snprintf(name, sizeof(name), "%s_path_valid", pf_algo_name(QUERY_ALGOS[i]));
            check(name, r.rc == PF_OK &&
                        pf_path_valid(&m.grid, &r.p, QUERY_ALGOS[i] == PF_ANYANGLE) == 1);
            snprintf(name, sizeof(name), "%s_points_walkable", pf_algo_name(QUERY_ALGOS[i]));
            check(name, r.rc == PF_OK && every_point_walkable(&m.grid, &r.p));
            snprintf(name, sizeof(name), "%s_has_cost", pf_algo_name(QUERY_ALGOS[i]));
            check(name, r.rc == PF_OK && fabs(r.p.cost - pf_path_length(r.p.pts, r.p.n)) < 1e-6);
        }
        Result ast = run(s, a, b, PF_ASTAR);
        Result lpa = run(s, a, b, PF_INCREMENTAL);
        int same = ast.p.n == lpa.p.n;
        for (int i = 0; same && i < ast.p.n; i++)
            if (ast.p.pts[i].x != lpa.p.pts[i].x || ast.p.pts[i].y != lpa.p.pts[i].y) same = 0;
        check("astar_matches_lpa_pointwise", same);
        check("lpa_cost_matches_astar", fabs(ast.p.cost - lpa.p.cost) < 1e-9);
        Result th = run(s, a, b, PF_ANYANGLE);
        check("anyangle_no_longer_than_astar", th.p.cost <= ast.p.cost + 1e-9);
        Result hpa = run(s, a, b, PF_HIERARCHICAL);
        check("hpa_no_shorter_than_astar", hpa.p.cost >= ast.p.cost - 1e-9);

        pf_solver_free(s);
        map_free(&m);
    }

    /* ------------------------------------------------ 3. cross-validation, 3 map classes */
    {
        const char* names[MAP_CLASSES] = { "open", "maze", "rooms" };
        for (int c = 0; c < MAP_CLASSES; c++) {
            Map m;
            int ok = 0;
            rng_seed(0x1234u + (unsigned)c);
            if (c == MAP_OPEN) ok = make_open(&m, 96, 96);
            else if (c == MAP_MAZE) ok = make_maze(&m, 97, 97);
            else ok = make_rooms(&m, 96, 96);
            check("map_build", ok == 0);
            PfSolver* s = pf_solver_new(&m.grid);
            PfPoint a = map_first_walkable(&m), b = map_last_walkable(&m);
            Result ref[4];
            for (int i = 0; i < 4; i++) ref[i] = run_buf(s, a, b, QUERY_ALGOS[i], i);
            char name[96];
            for (int i = 0; i < 4; i++) {
                const Result* r = &ref[i];
                snprintf(name, sizeof(name), "%s_%s_path", names[c], pf_algo_name(QUERY_ALGOS[i]));
                check(name, r->rc == PF_OK && pf_path_valid(&m.grid, &r->p,
                                                            QUERY_ALGOS[i] == PF_ANYANGLE));
                snprintf(name, sizeof(name), "%s_%s_cost_exact", names[c],
                         pf_algo_name(QUERY_ALGOS[i]));
                check(name, r->rc == PF_OK &&
                            fabs(r->p.cost - pf_path_length(r->p.pts, r->p.n)) < 1e-6);
            }
            double cost[4];
            for (int i = 0; i < 4; i++) cost[i] = ref[i].p.cost;

            /* Cross-validation among the planners that share the 8-connected
             * move rule: either identical point sequences, or costs within the
             * stated bound. any-angle uses a different move rule, so it gets
             * its own one-sided bound against A* below. */
            struct { int a, b; const char* tag; double bound_pct; } pairs[] = {
                { 0, 2, "astar_lpa", 0.0 },
                { 0, 3, "astar_hpa", 5.0 },
                { 2, 3, "lpa_hpa", 5.0 },
            };
            for (unsigned p = 0; p < sizeof(pairs) / sizeof(pairs[0]); p++) {
                const Result* ra = &ref[pairs[p].a];
                const Result* rb = &ref[pairs[p].b];
                snprintf(name, sizeof(name), "%s_%s_agree", names[c], pairs[p].tag);
                int same_points = ra->rc == PF_OK && rb->rc == PF_OK && ra->p.n == rb->p.n;
                for (int i = 0; same_points && i < ra->p.n; i++)
                    if (ra->p.pts[i].x != rb->p.pts[i].x ||
                        ra->p.pts[i].y != rb->p.pts[i].y) same_points = 0;
                double bound = cost[pairs[p].a] * pairs[p].bound_pct / 100.0 + 1e-9;
                int cost_ok = ra->rc == PF_OK && rb->rc == PF_OK &&
                              fabs(cost[pairs[p].a] - cost[pairs[p].b]) <= bound;
                check(name, same_points || cost_ok);
            }
            /* any-angle is never worse than A* (it has strictly more moves) */
            snprintf(name, sizeof(name), "%s_anyangle_within_astar", names[c]);
            check(name, ref[1].rc == PF_OK && ref[0].rc == PF_OK &&
                        cost[1] <= cost[0] + 1e-9);
            /* HPA* is an upper bound: never below A*, at most 5% above */
            snprintf(name, sizeof(name), "%s_hpa_within_bound", names[c]);
            check(name, ref[3].rc == PF_OK && ref[0].rc == PF_OK &&
                        cost[3] >= cost[0] - 1e-9 && cost[3] <= cost[0] * 1.05 + 1e-9);
            pf_solver_free(s);
            map_free(&m);
        }
    }

    /* ------------------------------------------------ 4. incremental re-planning */
    {
        const char* rows[] = {
            "S....#....",
            ".##..#....",
            ".#......#.",
            ".#.####.#.",
            ".....#..G.",
        };
        Map m;
        map_from_rows(&m, rows, 5);
        PfSolver* s = pf_solver_new(&m.grid);
        PfPoint a = { 0, 0 }, b = { 8, 4 };
        Result before = run(s, a, b, PF_INCREMENTAL);
        check("incr_initial", before.rc == PF_OK);
        /* Open a shortcut on the long way around (7,2) and confirm the
         * incremental planner picks it up exactly like A* does. */
        pf_solver_set_blocked(s, 4, 4, 0);
        Result after = run(s, a, b, PF_INCREMENTAL);
        Result ref = run(s, a, b, PF_ASTAR);
        check("incr_replans_after_edit", after.rc == PF_OK);
        check("incr_matches_astar_after_edit", fabs(after.p.cost - ref.p.cost) < 1e-9);
        int same = after.p.n == ref.p.n;
        for (int i = 0; same && i < after.p.n; i++)
            if (after.p.pts[i].x != ref.p.pts[i].x || after.p.pts[i].y != ref.p.pts[i].y) same = 0;
        check("incr_path_identical_after_edit", same);
        check("incr_reuses_fewer_expansions", after.p.expansions <= ref.p.expansions);
        pf_solver_set_blocked(s, 8, 0, 1);          /* wall off the goal column */
        Result walled = run(s, a, b, PF_INCREMENTAL);
        Result walled_ref = run(s, a, b, PF_ASTAR);
        check("incr_wall_matches_astar", walled.rc == walled_ref.rc &&
                                         (walled.rc != PF_OK ||
                                          fabs(walled.p.cost - walled_ref.p.cost) < 1e-9));
        pf_solver_set_blocked(s, 8, 0, 0);
        pf_solver_set_blocked(s, 4, 4, 1);          /* restore the closed cell */
        Result reopened = run(s, a, b, PF_INCREMENTAL);
        Result ref2 = run(s, a, b, PF_ASTAR);
        check("incr_reopen_matches_astar", reopened.rc == PF_OK &&
                                           fabs(reopened.p.cost - ref2.p.cost) < 1e-9);
        Result hpa_after = run(s, a, b, PF_HIERARCHICAL);
        check("hpa_after_edit_valid", hpa_after.rc == PF_OK &&
                                      pf_path_valid(&m.grid, &hpa_after.p, 0));
        pf_solver_free(s);
        map_free(&m);
    }

    /* ------------------------------------------------ 5. auto-selection */
    {
        Map m;
        rng_seed(0x51u);
        make_open(&m, 128, 128);
        PfSolver* s = pf_solver_new(&m.grid);
        PfProfile prof;
        PfAlgo pick = pf_select_algo(&m.grid, (PfPoint){ 1, 1 },
                                     (PfPoint){ 126, 126 }, 0, &prof);
        check("auto_open_is_anyangle", pick == PF_ANYANGLE);
        PfPoint a = map_first_walkable(&m), b = map_last_walkable(&m);
        Result r = run(s, a, b, PF_AUTO);
        check("auto_reports_chosen_algo", r.p.algo == PF_ANYANGLE && r.rc == PF_OK);
        pf_solver_set_changes_expected(s, 1);
        Result r2 = run(s, a, b, PF_AUTO);
        check("auto_edits_is_incremental", r2.p.algo == PF_INCREMENTAL);
        pf_solver_set_changes_expected(s, 0);
        pf_solver_free(s);
        map_free(&m);

        make_maze(&m, 65, 65);
        check("auto_maze_is_astar", pf_select_algo(&m.grid, (PfPoint){ 1, 1 },
                                                   (PfPoint){ 63, 63 }, 0, NULL) == PF_ASTAR);
        map_free(&m);

        make_rooms(&m, 128, 128);
        check("auto_rooms_is_hierarchical", pf_select_algo(&m.grid, (PfPoint){ 1, 1 },
                                                           (PfPoint){ 126, 126 }, 0, NULL) ==
                                            PF_HIERARCHICAL);
        map_free(&m);
    }

    /* ------------------------------------------------ 6. benchmark table */
    if (argc > 1 && strcmp(argv[1], "bench") == 0) {
        /* One row per (map, algorithm).
         *   cold_ms / cold_exp : fresh solver, first query (HPA* pays build,
         *                        LPA* pays its initial search)
         *   warm_ms / warm_exp : 6th query on the same solver and map, so HPA*
         *                        reuses its abstraction and LPA* reuses its
         *                        values; A* and any-angle have no state and are
         *                        unchanged by construction
         * Times are the best of five. */
        printf("\nmap,algo,cold_ms,warm_ms,cold_exp,warm_exp,path_len,cost\n");
        for (int c = 0; c < MAP_CLASSES; c++) {
            Map m;
            rng_seed(0x1234u + (unsigned)c);
            if (c == MAP_OPEN) make_open(&m, 128, 128);
            else if (c == MAP_MAZE) make_maze(&m, 129, 129);
            else make_rooms(&m, 128, 128);
            PfPoint a = map_first_walkable(&m), b = map_last_walkable(&m);
            double base_cost = 0.0, gap_pct = 0.0;
            for (int i = 0; i < 4; i++) {
                double cold_ms = 1e9, warm_ms = 1e9, cost = 0.0;
                long cold_exp = 0, warm_exp = 0;
                int plen = 0;
                int reps = 5;
                for (int rep = 0; rep < reps; rep++) {
                    PfSolver* s = pf_solver_new(&m.grid);   /* cold every rep */
                    struct timespec t0, t1;
                    clock_gettime(CLOCK_MONOTONIC, &t0);
                    Result r = run(s, a, b, QUERY_ALGOS[i]);
                    clock_gettime(CLOCK_MONOTONIC, &t1);
                    double ms = (double)(t1.tv_sec - t0.tv_sec) * 1e3 +
                                (double)(t1.tv_nsec - t0.tv_nsec) / 1e6;
                    if (r.rc != PF_OK) { pf_solver_free(s); continue; }
                    if (ms < cold_ms) {
                        cold_ms = ms;
                        cold_exp = r.p.expansions;
                        cost = r.p.cost;
                        plen = r.p.n;
                    }
                    /* warm: same solver, same map, repeated query */
                    for (int q = 0; q < 5; q++) run(s, a, b, QUERY_ALGOS[i]);
                    clock_gettime(CLOCK_MONOTONIC, &t0);
                    Result wr = run(s, a, b, QUERY_ALGOS[i]);
                    clock_gettime(CLOCK_MONOTONIC, &t1);
                    double wms = (double)(t1.tv_sec - t0.tv_sec) * 1e3 +
                                 (double)(t1.tv_nsec - t0.tv_nsec) / 1e6;
                    if (wr.rc == PF_OK && wms < warm_ms) {
                        warm_ms = wms;
                        warm_exp = wr.p.expansions;
                    }
                    pf_solver_free(s);
                }
                printf("%s,%s,%.3f,%.3f,%ld,%ld,%d,%.4f\n", class_name((MapClass)c),
                       pf_algo_name(QUERY_ALGOS[i]), cold_ms, warm_ms,
                       cold_exp, warm_exp, plen, cost);
                if (QUERY_ALGOS[i] == PF_ASTAR) base_cost = cost;
                else if (QUERY_ALGOS[i] == PF_HIERARCHICAL && base_cost > 0)
                    gap_pct = 100.0 * (cost / base_cost - 1.0);
            }
            printf("  (%s: hpa gap vs astar %.2f%%)\n", class_name((MapClass)c), gap_pct);
            map_free(&m);
        }

        /* Re-planning table. Each sample edits the map, then re-plans with:
         *   astar       - cold search from scratch
         *   incremental - LPA* reusing the state built by the previous query
         * Edits sweep along the current optimal route. The typical edit is a
         * door opening (a wall beside the route); where there is no adjacent
         * wall, a route cell is closed and samples that disconnect the map are
         * skipped. Only samples where both planners agree on the cost count. */
        printf("\nreplan,algo,ms,expansions\n");
        for (int c = 0; c < MAP_CLASSES; c++) {
            Map m;
            rng_seed(0x1234u + (unsigned)c);
            if (c == MAP_OPEN) make_open(&m, 128, 128);
            else if (c == MAP_MAZE) make_maze(&m, 129, 129);
            else make_rooms(&m, 128, 128);
            PfSolver* s = pf_solver_new(&m.grid);
            PfPoint a = map_first_walkable(&m), b = map_last_walkable(&m);
            run_buf(s, a, b, PF_INCREMENTAL, 2);            /* warm LPA* state */
            Result base = run_buf(s, a, b, PF_ASTAR, 3);
            double astar_ms = 0, lpa_ms = 0;
            long astar_exp = 0, lpa_exp = 0;
            int samples = 0;
            for (int step = 1; step <= 8; step++) {
                int k = base.p.n * step / 9;
                if (k < 1 || k >= base.p.n) continue;
                PfPoint cell = base.p.pts[k];
                static const int D4X[4] = { 1, -1, 0, 0 };
                static const int D4Y[4] = { 0, 0, 1, -1 };
                int ex = -1, ey = -1;
                for (int d = 0; d < 4 && ex < 0; d++) {
                    int wx = cell.x + D4X[d], wy = cell.y + D4Y[d];
                    if (wx <= 0 || wy <= 0 || wx >= m.w - 1 || wy >= m.h - 1) continue;
                    if (pf_walkable(&m.grid, wx, wy)) continue;
                    ex = wx; ey = wy;
                }
                int opening = ex >= 0;
                if (!opening) { ex = cell.x; ey = cell.y; }
                pf_solver_set_blocked(s, ex, ey, opening ? 0 : 1);
                struct timespec t0, t1, t2;
                clock_gettime(CLOCK_MONOTONIC, &t0);
                Result r0 = run(s, a, b, PF_ASTAR);
                clock_gettime(CLOCK_MONOTONIC, &t1);
                Result r1 = run_buf(s, a, b, PF_INCREMENTAL, 2);
                clock_gettime(CLOCK_MONOTONIC, &t2);
                if (r0.rc == PF_OK && r1.rc == PF_OK && fabs(r0.p.cost - r1.p.cost) < 1e-9) {
                    astar_ms += (double)(t1.tv_sec - t0.tv_sec) * 1e3 +
                                (double)(t1.tv_nsec - t0.tv_nsec) / 1e6;
                    lpa_ms += (double)(t2.tv_sec - t1.tv_sec) * 1e3 +
                              (double)(t2.tv_nsec - t1.tv_nsec) / 1e6;
                    astar_exp += r0.p.expansions;
                    lpa_exp += r1.p.expansions;
                    samples++;
                }
                pf_solver_set_blocked(s, ex, ey, opening ? 1 : 0);
            }
            if (samples > 0) {
                printf("%s,astar,%.3f,%ld\n", class_name((MapClass)c),
                       astar_ms / samples, astar_exp / samples);
                printf("%s,incremental,%.3f,%ld\n", class_name((MapClass)c),
                       lpa_ms / samples, lpa_exp / samples);
            }
            pf_solver_free(s);
            map_free(&m);
        }
    }

    printf("all_ok=%d\n", failures == 0);
    for (int i = 0; i < 4; i++) free(g_pts[i]);
    return failures == 0 ? 0 : 1;
}
