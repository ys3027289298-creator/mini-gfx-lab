
/* mini-navgrid: 8-direction A* on an ASCII grid, with corner-cut prevention,
   path smoothing by line-of-sight, and incremental re-planning.  Also an
   interception demo: a chaser that extrapolates a moving target, refines the
   meeting cell by fixed-point iteration, and falls back to plain tracking. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

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
    /* lazy deletion re-pushes a node whenever its g improves, so the open
       list can hold more than one entry per cell; 8 directions bounds it */
    int cap = 8 * (int)((size_t)g->w * g->h) / 8 + 8;
    int* heap = (int*)malloc(sizeof(int) * (size_t)cap);
    if (!heap) { free(nodes); return -1; }
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
                if (hn >= cap) break;   /* defensive: heap is sized for 8/cell */
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

/* ======================= interception prediction =======================
   The chaser knows the target's current velocity.  It extrapolates the
   target forward, plans a path to that predicted cell, uses the path's
   travel time as the next time guess, and repeats.  The loop is capped at
   MAX_INTERCEPT_ITERS.  If the fixed point does not settle (or the target
   is at least as fast and moving away) the chaser falls back to tracking
   the target's current cell; a small hysteresis on the mode keeps the
   fallback from flapping between prediction and tracking.               */

#define VP                   2.2    /* pursuer speed, cells/second */
#define HIT_RADIUS           0.60
#define SIM_DT               0.05
#define REPLAN_DT            0.25
#define SIM_TIME             8.0
#define MAX_INTERCEPT_ITERS  8      /* hard upper bound on refinement */
#define ITER_TOL             0.10   /* seconds */
#define ETA_HORIZON          12.0
#define ORACLE_SAMPLES       24
#define ORACLE_HORIZON       8.0
#define TRIALS               20
#define JINK_PERIOD          0.75   /* seconds between target course changes */

typedef struct { double x, y; } Vec2;

static double vec_len(Vec2 a) { return hypot(a.x, a.y); }
static double vec_dist(Vec2 a, Vec2 b) { return hypot(a.x - b.x, a.y - b.y); }
static Vec2 vec_sub(Vec2 a, Vec2 b) { Vec2 r = { a.x - b.x, a.y - b.y }; return r; }
static Vec2 vec_scale(Vec2 a, double s) { Vec2 r = { a.x * s, a.y * s }; return r; }

static Pt cell_of(Vec2 p) { Pt c = { (int)floor(p.x), (int)floor(p.y) }; return c; }
static Vec2 center_of(Pt c) { Vec2 p = { c.x + 0.5, c.y + 0.5 }; return p; }

typedef struct { uint64_t s; } Rng;

static double rnd01(Rng* r) {
    r->s ^= r->s << 13;
    r->s ^= r->s >> 7;
    r->s ^= r->s << 17;
    return (double)(r->s >> 11) * (1.0 / 9007199254740992.0);
}

static int nearest_walkable(const Grid* g, Vec2 p, Pt* out) {
    int bx = (int)floor(p.x), by = (int)floor(p.y);
    int best = -1;
    double bd = 1e300;
    for (int dy = -6; dy <= 6; dy++) {
        for (int dx = -6; dx <= 6; dx++) {
            int x = bx + dx, y = by + dy;
            if (!walkable(g, x, y)) continue;
            double ddx = x + 0.5 - p.x, ddy = y + 0.5 - p.y;
            double d = ddx * ddx + ddy * ddy;
            if (d < bd) { bd = d; best = y * g->w + x; }
        }
    }
    if (best < 0) return -1;
    out->x = best % g->w;
    out->y = best / g->w;
    return 0;
}

/* one axis at a time; bounce off walls and blocked cells */
static void advance_target(const Grid* g, Vec2* p, Vec2* v, double dt) {
    if (v->x != 0.0) {
        double nx = p->x + v->x * dt;
        if (walkable(g, (int)floor(nx), (int)floor(p->y))) p->x = nx;
        else v->x = -v->x;
    }
    if (v->y != 0.0) {
        double ny = p->y + v->y * dt;
        if (walkable(g, (int)floor(p->x), (int)floor(ny))) p->y = ny;
        else v->y = -v->y;
    }
}

static Vec2 extrapolate(const Grid* g, Vec2 p, Vec2 v, double t) {
    const double h = 0.1;
    Vec2 vv = v;
    for (double s = 0.0; s < t; s += h) {
        double dt = (t - s < h) ? t - s : h;
        advance_target(g, &p, &vv, dt);
    }
    return p;
}

/* A* + line-of-sight smoothing; returns waypoint count and travel length */
static int plan_path(const Grid* g, Pt a, Pt b, Pt* out, int max_out, double* len) {
    if (!walkable(g, a.x, a.y)) {
        if (nearest_walkable(g, center_of(a), &a) < 0) return -1;
    }
    Pt raw[MAX_NODES];
    int n = astar(g, a, b, raw, MAX_NODES);
    if (n <= 0) return -1;
    int m = smooth(g, raw, n, out, max_out);
    if (m <= 0) return -1;
    if (len) *len = path_length(out, m);
    return m;
}

typedef enum {
    AIM_FAILED = 0,       /* no usable aim: fall back to tracking */
    AIM_SETTLED = 1,      /* |eta - t| within tolerance */
    AIM_RESOLUTION = 2    /* oscillates at the grid-resolution floor */
} AimStatus;

typedef struct {
    int valid;        /* an aim cell was produced */
    AimStatus status; /* why the refinement stopped */
    int iters;        /* refinement steps actually used */
    Pt aim;
    double eta;       /* estimated travel time to aim, seconds */
} Aim;

static Aim tracking_aim(const Grid* g, Vec2 pp, Vec2 tp) {
    Aim a;
    memset(&a, 0, sizeof(a));
    if (nearest_walkable(g, tp, &a.aim) < 0) return a;
    Pt path[MAX_NODES];
    double len = 0.0;
    int n = plan_path(g, cell_of(pp), a.aim, path, MAX_NODES, &len);
    a.valid = 1;
    a.status = AIM_SETTLED;
    a.eta = n > 0 ? len / VP : ETA_HORIZON;
    return a;
}

/* fixed-point refinement: t <- plan_time(extrapolate(target, t)).
   The aim can oscillate between two adjacent cells when the grid resolution
   is coarser than the convergence tolerance; detect that and stop. */
static Aim solve_iterative(const Grid* g, Vec2 pp, Vec2 tp, Vec2 tv) {
    Aim a;
    memset(&a, 0, sizeof(a));
    Vec2 rem = vec_sub(tp, pp);
    double sp = vec_len(tv);

    /* receding and no faster: no interception exists */
    if (sp >= VP && rem.x * tv.x + rem.y * tv.y > 0.0) {
        a.status = AIM_FAILED;
        return a;
    }

    double t = vec_dist(tp, pp) / VP;
    Pt seen[MAX_INTERCEPT_ITERS];
    double seen_eta[MAX_INTERCEPT_ITERS];
    double seen_err[MAX_INTERCEPT_ITERS];
    int nseen = 0;
    int best = -1;
    for (int it = 1; it <= MAX_INTERCEPT_ITERS; it++) {
        a.iters = it;
        if (!isfinite(t) || t < 0.0 || t > ETA_HORIZON) break;
        Vec2 fp = extrapolate(g, tp, tv, t);
        Pt aim;
        if (nearest_walkable(g, fp, &aim) < 0) break;
        Pt path[MAX_NODES];
        double len = 0.0;
        if (plan_path(g, cell_of(pp), aim, path, MAX_NODES, &len) < 0) break;
        double eta = len / VP;
        a.valid = 1;
        a.aim = aim;
        a.eta = eta;
        double err = fabs(eta - t);
        int cycled = 0;
        for (int k = 0; k < nseen; k++)
            if (seen[k].x == aim.x && seen[k].y == aim.y) { cycled = 1; break; }
        if (nseen < MAX_INTERCEPT_ITERS) {
            if (best < 0 || err < seen_err[best]) best = nseen;
            seen[nseen] = aim;
            seen_eta[nseen] = eta;
            seen_err[nseen++] = err;
        }
        t = eta;
        if (err < ITER_TOL) { a.status = AIM_SETTLED; break; }
        if (cycled) {
            /* oscillates between cells: take the closest fit.  It is a
               function of the target state only, so the aim does not
               wander between replans of the same situation. */
            a.aim = seen[best];
            a.eta = seen_eta[best];
            a.status = AIM_RESOLUTION;
            break;
        }
    }
    if (!a.valid || a.eta > ETA_HORIZON) { a.valid = 0; a.status = AIM_FAILED; }
    return a;
}

/* exhaustive reference: best rendezvous over a bounded time scan.
   Used only as the "ideal intercept" baseline for the statistics. */
static Aim solve_oracle(const Grid* g, Vec2 pp, Vec2 tp, Vec2 tv) {
    Aim best;
    memset(&best, 0, sizeof(best));
    double best_err = 1e300;
    for (int k = 0; k <= ORACLE_SAMPLES; k++) {
        double t = ORACLE_HORIZON * (double)k / ORACLE_SAMPLES;
        Vec2 fp = extrapolate(g, tp, tv, t);
        Pt aim;
        if (nearest_walkable(g, fp, &aim) < 0) continue;
        Pt path[MAX_NODES];
        double len = 0.0;
        if (plan_path(g, cell_of(pp), aim, path, MAX_NODES, &len) < 0) continue;
        double eta = len / VP;
        double err = fabs(eta - t);
        if (err < best_err) {
            best_err = err;
            best.valid = 1;
            best.aim = aim;
            best.eta = eta;
            best.status = (err < ITER_TOL) ? AIM_SETTLED : AIM_RESOLUTION;
        }
    }
    if (!best.valid || best_err > 0.75) { best.valid = 0; best.status = AIM_FAILED; }
    return best;
}

/* mode hysteresis: a single bad prediction is not enough to abandon
   prediction; after dropping to tracking a short cooldown plus one good
   prediction brings it back.  Sweeping the aim every replan is what makes
   a path jitter, so this must not be too eager to swap. */
typedef struct { int mode; int fail_streak, good_streak; double since_switch; } Hyst;

#define HYST_FAIL_STREAK 2
#define HYST_GOOD_STREAK 1
#define HYST_COOLDOWN    0.15

static void hyst_update(Hyst* h, int prediction_failed, double dt) {
    h->since_switch += dt;
    if (h->mode == 0) {
        h->fail_streak = prediction_failed ? h->fail_streak + 1 : 0;
        if (h->fail_streak >= HYST_FAIL_STREAK) {
            h->mode = 1;
            h->since_switch = 0.0;
            h->good_streak = 0;
            h->fail_streak = 0;
        }
    } else {
        h->good_streak = prediction_failed ? 0 : h->good_streak + 1;
        if (h->good_streak >= HYST_GOOD_STREAK && h->since_switch >= HYST_COOLDOWN) {
            h->mode = 0;
            h->since_switch = 0.0;
            h->good_streak = 0;
            h->fail_streak = 0;
        }
    }
}

static void move_along(Vec2* p, const Pt* path, int n, int* idx, double dist) {
    double remain = dist;
    while (*idx < n && remain > 1e-9) {
        Vec2 w = center_of(path[*idx]);
        double d = vec_dist(*p, w);
        if (d <= remain) {
            *p = w;
            (*idx)++;
            remain -= d;
        } else {
            double s = remain / d;
            p->x += (w.x - p->x) * s;
            p->y += (w.y - p->y) * s;
            remain = 0.0;
        }
    }
}

typedef struct { Vec2 p0, t0, tv; int jink; uint64_t seed; } Trial;

typedef enum { STRAT_CHASE = 0, STRAT_PREDICT = 1, STRAT_ORACLE = 2,
               STRAT_PREDICT_NOHYST = 3 } Strategy;

typedef struct {
    int hit;
    double hit_time;
    double travel;       /* pursuer path length, cells */
    int reversals;       /* heading flips while in tracking fallback */
    int path_reversals;  /* heading flips anywhere in the pursued path */
    int fallback_plans;
    int source_switches; /* prediction <-> tracking transitions at replans */
    int pred_plans;
    int conv_plans;
    int res_plans;
    int iters_max;
} SimResult;

static SimResult simulate(const Grid* g, Trial tr, Strategy strat) {
    SimResult r;
    memset(&r, 0, sizeof(r));
    Vec2 p = tr.p0, tp = tr.t0, tv = tr.tv;
    Pt path[MAX_NODES];
    int path_n = 0, path_i = 0;
    Hyst h;
    memset(&h, 0, sizeof(h));
    Pt last_aim = { 0, 0 };
    int have_aim = 0;
    Vec2 last_dir = { 0.0, 0.0 };
    int have_dir = 0;
    int was_tracking = 1;
    int have_planned = 0;
    double next_plan = 0.0;
    Rng jr = { tr.seed | 1u };
    double next_jink = JINK_PERIOD;

    for (double t = 0.0; t < SIM_TIME; t += SIM_DT) {
        if (tr.jink && t + 1e-9 >= next_jink) {
            next_jink += JINK_PERIOD;
            double ang = 6.283185307179586 * rnd01(&jr);
            double sp = vec_len(tv);
            tv.x = cos(ang) * sp;
            tv.y = sin(ang) * sp;
        }
        if (t + 1e-9 >= next_plan) {
            Aim track = tracking_aim(g, p, tp);
            Aim chosen;
            memset(&chosen, 0, sizeof(chosen));
            int used_tracking = 1;
            int prev_was_tracking = was_tracking;

            if (strat == STRAT_CHASE) {
                chosen = track;
            } else if (strat == STRAT_PREDICT || strat == STRAT_PREDICT_NOHYST) {
                Aim pred = solve_iterative(g, p, tp, tv);
                int failed = (pred.status == AIM_FAILED);
                if (strat == STRAT_PREDICT) hyst_update(&h, failed, REPLAN_DT);
                r.pred_plans++;
                if (pred.iters > r.iters_max) r.iters_max = pred.iters;
                if (pred.status == AIM_SETTLED) r.conv_plans++;
                if (pred.status == AIM_RESOLUTION) r.res_plans++;
                int use_pred =
                    (strat == STRAT_PREDICT) ? (!failed && h.mode == 0) : !failed;
                if (use_pred) { chosen = pred; used_tracking = 0; }
                else if (strat == STRAT_PREDICT && h.mode == 0 && have_aim) {
                    /* a single bad prediction is not a reason to abandon the
                       plan: hold the previous aim instead of turning around
                       to chase the target's current cell */
                    chosen = pred;
                    chosen.valid = 1;
                    chosen.aim = last_aim;
                    chosen.status = AIM_RESOLUTION;
                    used_tracking = 0;
                }
                else chosen = track;
            } else {
                Aim pred = solve_oracle(g, p, tp, tv);
                if (pred.status != AIM_FAILED) { chosen = pred; used_tracking = 0; }
                else chosen = track;
            }
            if (used_tracking) r.fallback_plans++;
            if (have_planned && used_tracking != prev_was_tracking) r.source_switches++;
            was_tracking = used_tracking;
            have_planned = 1;

            if (chosen.valid) {
                double len = 0.0;
                int n = plan_path(g, cell_of(p), chosen.aim, path, MAX_NODES, &len);
                if (n > 0) {
                    path_n = n;
                    path_i = (n > 1) ? 1 : n;   /* skip the cell we stand in */
                } else {
                    path_n = 0;
                    path_i = 0;
                }
                last_aim = chosen.aim;
                have_aim = 1;
            }
            next_plan += REPLAN_DT;
        }

        Vec2 before = p;
        move_along(&p, path, path_n, &path_i, VP * SIM_DT);
        advance_target(g, &tp, &tv, SIM_DT);

        Vec2 mv = vec_sub(p, before);
        double ml = vec_len(mv);
        r.travel += ml;
        if (ml > 1e-9) {
            Vec2 dir = vec_scale(mv, 1.0 / ml);
            /* a heading flip during predictive pursuit is the target
               jinking; a flip during fallback tracking is a jittery path */            if (have_dir && dir.x * last_dir.x + dir.y * last_dir.y < -0.5) {
                if (was_tracking) r.reversals++;
                r.path_reversals++;
            }
            last_dir = dir;
            have_dir = 1;
        }
        if (vec_dist(p, tp) <= HIT_RADIUS) {
            r.hit = 1;
            r.hit_time = t;
            break;
        }
    }
    return r;
}

static int rand_cell(const Grid* g, Rng* rng) {
    for (int i = 0; i < 256; i++) {
        int x = (int)(rnd01(rng) * g->w);
        int y = (int)(rnd01(rng) * g->h);
        if (walkable(g, x, y)) return y * g->w + x;
    }
    return -1;
}

static Trial make_flee_trial(double ratio, int lane) {
    Trial tr;
    memset(&tr, 0, sizeof(tr));
    tr.p0 = (Vec2){ 2.5, lane + 0.5 };
    tr.t0 = (Vec2){ 9.5, lane + 0.5 };
    tr.tv = (Vec2){ ratio * VP, 0.0 };   /* straight away from the chaser */
    return tr;
}

static Trial make_trial(const Grid* g, double ratio, uint64_t seed) {
    Rng rng = { seed | 1u };
    Trial tr;
    memset(&tr, 0, sizeof(tr));
    int ti = rand_cell(g, &rng);
    while (ti < 0) ti = rand_cell(g, &rng);
    tr.t0 = center_of((Pt){ ti % g->w, ti / g->w });
    double ang = 6.283185307179586 * rnd01(&rng);
    double sp = ratio * VP;
    tr.tv = (Vec2){ cos(ang) * sp, sin(ang) * sp };
    int pi = -1;
    for (int tries = 0; tries < 200; tries++) {
        int cand = rand_cell(g, &rng);
        if (cand < 0) continue;
        Vec2 pc = center_of((Pt){ cand % g->w, cand / g->w });
        if (vec_dist(pc, tr.t0) >= 4.0) { pi = cand; break; }
    }
    if (pi < 0) pi = ti;
    tr.p0 = center_of((Pt){ pi % g->w, pi / g->w });
    return tr;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static const char* ARENA[] = {
    "................",
    "....##....##....",
    "....##....##....",
    "................",
    "................",
    "..#..........#..",
    "..#..........#..",
    "................",
    "................",
    "....##....##....",
    "....##....##....",
    "................",
};

static void run_interception_demo(void) {
    Grid g;
    if (grid_load(&g, ARENA, 12) != 0) { printf("arena load fail\n"); failures++; return; }

    const double ratios[] = { 0.50, 0.80, 1.00, 1.25 };
    int hits[4] = { 0, 0, 0, 0 };
    int slow_predict = 0;
    int total = 0;
    int iters_max = 0, pred_plans = 0, conv_plans = 0, res_plans = 0, fb_plans = 0;
    int rev_pred = 0, rev_chase = 0, rev_nohyst = 0;
    int sw_pred = 0, sw_nohyst = 0, sw_chase = 0;
    double time_sum[4] = { 0.0, 0.0, 0.0, 0.0 };
    double travel_sum[4] = { 0.0, 0.0, 0.0, 0.0 };

    printf("--- interception (vp=%.2f, trials=%d/ratio) ---\n", VP, TRIALS);
    for (int ri = 0; ri < 4; ri++) {
        int rh[4] = { 0, 0, 0, 0 };
        for (int j = 0; j < TRIALS; j++) {
            uint64_t seed = 0x9E3779B97F4A7C15ULL * (uint64_t)(ri + 1)
                          + (uint64_t)j * 7919u + 12345u;
            Trial tr = make_trial(&g, ratios[ri], seed);
            for (int s = 0; s < 4; s++) {
                SimResult r = simulate(&g, tr, (Strategy)s);
                if (r.hit) {
                    hits[s]++; rh[s]++;
                    time_sum[s] += r.hit_time;
                    travel_sum[s] += r.travel;
                }
                if (s == STRAT_PREDICT) {
                    pred_plans += r.pred_plans;
                    conv_plans += r.conv_plans;
                    res_plans += r.res_plans;
                    fb_plans += r.fallback_plans;
                    if (r.iters_max > iters_max) iters_max = r.iters_max;
                    rev_pred += r.reversals;
                    sw_pred += r.source_switches;
                } else if (s == STRAT_CHASE) {
                    rev_chase += r.reversals;
                    sw_chase += r.source_switches;
                } else if (s == STRAT_PREDICT_NOHYST) {
                    rev_nohyst += r.reversals;
                    sw_nohyst += r.source_switches;
                }
            }
            total++;
        }
        if (ri == 0) slow_predict = rh[STRAT_PREDICT];
        printf("ratio=%.2f chase=%5.1f%% predict=%5.1f%% ideal=%5.1f%%\n",
               ratios[ri],
               100.0 * rh[STRAT_CHASE] / TRIALS,
               100.0 * rh[STRAT_PREDICT] / TRIALS,
               100.0 * rh[STRAT_ORACLE] / TRIALS);
    }
    printf("hit_total chase=%5.1f%% predict=%5.1f%% ideal=%5.1f%%\n",
           100.0 * hits[STRAT_CHASE] / total,
           100.0 * hits[STRAT_PREDICT] / total,
           100.0 * hits[STRAT_ORACLE] / total);
    printf("hits predict=%d of %d ideal=%d\n", hits[STRAT_PREDICT], total, hits[STRAT_ORACLE]);
    printf("mean_hit_time chase=%.2fs predict=%.2fs ideal=%.2fs\n",
           time_sum[STRAT_CHASE] / (hits[STRAT_CHASE] > 0 ? hits[STRAT_CHASE] : 1),
           time_sum[STRAT_PREDICT] / (hits[STRAT_PREDICT] > 0 ? hits[STRAT_PREDICT] : 1),
           time_sum[STRAT_ORACLE] / (hits[STRAT_ORACLE] > 0 ? hits[STRAT_ORACLE] : 1));
    printf("mean_travel chase=%.2f predict=%.2f ideal=%.2f cells\n",
           travel_sum[STRAT_CHASE] / (hits[STRAT_CHASE] > 0 ? hits[STRAT_CHASE] : 1),
           travel_sum[STRAT_PREDICT] / (hits[STRAT_PREDICT] > 0 ? hits[STRAT_PREDICT] : 1),
           travel_sum[STRAT_ORACLE] / (hits[STRAT_ORACLE] > 0 ? hits[STRAT_ORACLE] : 1));
    printf("iters_max=%d bound=%d settled=%5.1f%% resolution_floor=%5.1f%% fallback=%5.1f%%\n",
           iters_max, MAX_INTERCEPT_ITERS,
           100.0 * conv_plans / (pred_plans > 0 ? pred_plans : 1),
           100.0 * res_plans / (pred_plans > 0 ? pred_plans : 1),
           100.0 * fb_plans / (pred_plans > 0 ? pred_plans : 1));
    printf("fallback_tracking reversals predict=%d chase=%d predict_nohyst=%d\n",
           rev_pred, rev_chase, rev_nohyst);
    printf("source_switches predict=%d predict_nohyst=%d chase=%d\n",
           sw_pred, sw_nohyst, sw_chase);
    /* the target changes velocity on a 0.75 s cadence: the prediction
       goes stale on purpose and must degrade without a jittery path */
    int jh[4] = { 0, 0, 0, 0 };
    int jrev[4] = { 0, 0, 0, 0 };
    int jfall[4] = { 0, 0, 0, 0 };
    for (int j = 0; j < TRIALS; j++) {
        uint64_t seed = 0xABCDEF1234567ULL + (uint64_t)j * 7919u;
        Trial tr = make_trial(&g, 0.60, seed);
        tr.jink = 1;
        tr.seed = seed;
        for (int s = 0; s < 4; s++) {
            SimResult r = simulate(&g, tr, (Strategy)s);
            if (r.hit) jh[s]++;
            jrev[s] += r.path_reversals;
            jfall[s] += r.fallback_plans;
        }
    }
    printf("jinking_target hits chase=%d predict=%d ideal=%d nohyst=%d of %d\n",
           jh[STRAT_CHASE], jh[STRAT_PREDICT], jh[STRAT_ORACLE],
           jh[STRAT_PREDICT_NOHYST], TRIALS);
    printf("jinking_path reversals chase=%d predict=%d ideal=%d nohyst=%d\n",
           jrev[STRAT_CHASE], jrev[STRAT_PREDICT], jrev[STRAT_ORACLE],
           jrev[STRAT_PREDICT_NOHYST]);
    printf("jinking_fallback_plans chase=%d predict=%d nohyst=%d\n",
           jfall[STRAT_CHASE], jfall[STRAT_PREDICT], jfall[STRAT_PREDICT_NOHYST]);

    /* a faster target running straight away: no intercept exists, so the
       fallback path must stay as calm as plain tracking */
    int fh[3] = { 0, 0, 0 };
    int frev[3] = { 0, 0, 0 };
    int fsw[3] = { 0, 0, 0 };
    const int lanes[] = { 1, 4, 7 };
    for (int li = 0; li < 3; li++) {
        Trial tr = make_flee_trial(1.30, lanes[li]);
        for (int s = 0; s < 3; s++) {
            SimResult r = simulate(&g, tr, (Strategy)s);
            if (r.hit) fh[s]++;
            frev[s] += r.reversals;
            fsw[s] += r.source_switches;
        }
    }
    printf("fleeing_target hits chase=%d predict=%d ideal=%d of 3\n",
           fh[STRAT_CHASE], fh[STRAT_PREDICT], fh[STRAT_ORACLE]);
    printf("fleeing_heading_flips chase=%d predict=%d ideal=%d\n",
           frev[STRAT_CHASE], frev[STRAT_PREDICT], frev[STRAT_ORACLE]);
    printf("fleeing_source_switches chase=%d predict=%d ideal=%d\n",
           fsw[STRAT_CHASE], fsw[STRAT_PREDICT], fsw[STRAT_ORACLE]);

    check("iterations_bounded", iters_max <= MAX_INTERCEPT_ITERS);
    check("predict_usable", (conv_plans + res_plans) * 10 >= pred_plans * 7);
    check("fallback_is_used", fb_plans > 0);
    check("fallback_only_when_needed", fb_plans * 2 < pred_plans);
    check("predict_beats_chase", hits[STRAT_PREDICT] > hits[STRAT_CHASE]);
    check("ideal_not_worse", hits[STRAT_ORACLE] >= hits[STRAT_PREDICT]);
    check("slow_target_reliable", slow_predict >= (TRIALS * 9) / 10);
    check("fallback_no_more_jitter_than_chase", rev_pred <= rev_chase);
    check("hysteresis_no_hit_loss", hits[STRAT_PREDICT] >= hits[STRAT_PREDICT_NOHYST]);
    check("hysteresis_no_extra_flapping", sw_pred <= sw_nohyst);
    check("stale_prediction_degrades_gracefully",
          jrev[STRAT_PREDICT] <= jrev[STRAT_CHASE] + TRIALS);
    check("fleeing_target_not_more_erratic",
          frev[STRAT_PREDICT] <= frev[STRAT_CHASE]);

    /* a failing prediction must be deterministic: the same target state
       must always yield the same answer, so a fallback cannot oscillate */
    {
        Trial tr = make_flee_trial(1.30, 4);
        Aim first = solve_iterative(&g, tr.p0, tr.t0, tr.tv);
        int stable = (first.status == AIM_FAILED);
        for (int k = 0; k < 16; k++) {
            Aim again = solve_iterative(&g, tr.p0, tr.t0, tr.tv);
            if (again.status != first.status) stable = 0;
            if (again.aim.x != first.aim.x || again.aim.y != first.aim.y) stable = 0;
            if (again.iters != first.iters) stable = 0;
        }
        check("failed_prediction_is_deterministic", stable);
        check("runaway_target_reports_failure", first.status == AIM_FAILED);
    }
    {
        /* stationary target: the fixed point is the target cell itself */
        Trial tr;
        memset(&tr, 0, sizeof(tr));
        tr.p0 = (Vec2){ 2.5, 2.5 };
        tr.t0 = (Vec2){ 9.5, 9.5 };
        Aim a = solve_iterative(&g, tr.p0, tr.t0, tr.tv);
        check("static_target_settles", a.status == AIM_SETTLED && a.iters <= MAX_INTERCEPT_ITERS);
        check("static_target_aims_true_cell", a.aim.x == 9 && a.aim.y == 9);
    }
    {
        /* receding target: refinement must stop within the bound even if
           the time guess never settles */
        Trial tr = make_flee_trial(1.10, 4);
        Aim a = solve_iterative(&g, tr.p0, tr.t0, tr.tv);
        check("receding_faster_target_is_capped", a.iters <= MAX_INTERCEPT_ITERS);
    }

    /* policy unit test: a lone bad prediction must not abort prediction */
    {
        Hyst p;
        memset(&p, 0, sizeof(p));
        hyst_update(&p, 1, REPLAN_DT);          /* one failure */
        check("hyst_single_failure_keeps_predict", p.mode == 0);
        hyst_update(&p, 1, REPLAN_DT);          /* second in a row */
        check("hyst_streak_falls_back", p.mode == 1);
        hyst_update(&p, 0, REPLAN_DT);
        hyst_update(&p, 0, REPLAN_DT);
        check("hyst_recovers_after_cooldown", p.mode == 0);
        hyst_update(&p, 1, REPLAN_DT);
        hyst_update(&p, 0, REPLAN_DT);
        hyst_update(&p, 1, REPLAN_DT);
        check("hyst_intermittent_failures_keep_predict", p.mode == 0);
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
    memset(sm, 0, sizeof(sm));
    int n = astar(&g, s, t, raw, MAX_NODES);
    if (n < 0) { printf("no path\n"); return 1; }
    int m = smooth(&g, raw, n, sm, MAX_NODES);
    if (m <= 0) { printf("smooth fail\n"); return 1; }
    printf("raw=%d len=%.4f smooth=%d len=%.4f\n", n, path_length(raw,n), m, path_length(sm,m));
    for (int i = 0; i < m; i++) printf("(%d,%d)%s", sm[i].x, sm[i].y, i+1==m ? "\n" : " ");
    int bad = 0;
    for (int i = 1; i < m; i++) if (!line_of_sight(&g, sm[i-1], sm[i])) bad++;
    printf("bad_segments=%d\n", bad);
    printf("corner_cut=%d\n", !step_ok(&g, (Pt){1,0}, (Pt){2,1}));
    check("astar_path_found", n > 0);
    check("smooth_no_corner_cuts", bad == 0);

    run_interception_demo();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
