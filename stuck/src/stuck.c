
/* stuck: stuck detection and unstuck maneuvers for grid-navigated units.

   A continuous-time simulation on top of a tile grid: units follow caravan
   waypoints with circle-vs-tile collision, a zero-false-positive stuck
   detector (rolling displacement/intent window), an escalating recovery
   state machine (strafe -> other strafe -> back-pedal -> replan flag), and
   formation bookkeeping (ordering, overlaps, slot deviation). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ----------------------------- parameters ------------------------------ */

#define DT             0.05    /* fixed tick, seconds */
#define MAX_W          40
#define MAX_H          24
#define MAX_AGENTS     8
#define MAX_PATH       96
#define WINDOW         24      /* stuck detector window: 24 ticks = 1.2 s */
#define INTENT_RATIO   0.40    /* arm only while intended speed > 0.4 * vmax */
#define PROGRESS_RATIO 0.25    /* actual / intended travel below this = stuck */
#define MIN_INTENT     0.60    /* min intended travel (cells) in the window */
#define GRACE_TICKS    10      /* detector disarmed 0.5 s after a recovery */
#define MAN_STRAFE_BUDGET 14   /* 0.7 s */
#define MAN_BACK_BUDGET   18   /* 0.9 s */
#define MAN_SPEED      1.0     /* gentle, local recovery speed (cells/s) */
#define MAN_OK_DIST    0.40    /* maneuver success displacement */
#define REPEAT_TICKS   60      /* re-stuck this soon at the same spot... */
#define REPEAT_DIST    0.55    /* ...=> skip strafes, escalate faster */
#define REPLAN_RETRY   12
#define REPLAN_CAP     300
#define MAX_EPISODES   64
#define VMAX           2.0
#define RADIUS         0.22
#define ARRIVE_DIST    0.18

/* ----------------------------- basic types ----------------------------- */

typedef struct { int w, h; unsigned char blocked[MAX_H][MAX_W]; } Grid;
typedef struct { double x, y; } V2;

static V2   v(double x, double y) { V2 r = { x, y }; return r; }
static V2   vadd(V2 a, V2 b) { return v(a.x + b.x, a.y + b.y); }
static V2   vsub(V2 a, V2 b) { return v(a.x - b.x, a.y - b.y); }
static V2   vscale(V2 a, double s) { return v(a.x * s, a.y * s); }
static double vdot(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }
static double vlen(V2 a) { return hypot(a.x, a.y); }
static V2   vnorm(V2 a) { double l = vlen(a); return l > 1e-12 ? vscale(a, 1.0 / l) : v(0, 0); }
static double vdist(V2 a, V2 b) { return vlen(vsub(a, b)); }
static V2   cell_center(int cx, int cy) { return v(cx + 0.5, cy + 0.5); }

/* deterministic LCG so every reported number is reproducible */
static unsigned long rng_state = 0x12345678ul;
static double randf(void) {
    rng_state = rng_state * 1103515245ul + 12345ul;
    return (double)((rng_state >> 16) & 0x7fff) / 32768.0;
}

static int trace_on;   /* STUCK_TRACE=1: per-tick trace of agent 0 */

static int grid_load(Grid* g, const char* const* rows, int h) {
    int w = (int)strlen(rows[0]);
    if (w <= 0 || w > MAX_W || h <= 0 || h > MAX_H) return -1;
    g->w = w; g->h = h;
    memset(g->blocked, 0, sizeof(g->blocked));
    for (int y = 0; y < h; y++) {
        if ((int)strlen(rows[y]) != w) return -1;
        for (int x = 0; x < w; x++) g->blocked[y][x] = (rows[y][x] == '#');
    }
    return 0;
}

static int walkable(const Grid* g, int x, int y) {
    return x >= 0 && y >= 0 && x < g->w && y < g->h && !g->blocked[y][x];
}

/* ----------------------------- collision ------------------------------- */

/* distance from circle centre c to the nearest blocked tile surface */
static double wall_dist(const Grid* g, V2 c, double r) {
    int x0 = (int)floor(c.x - r) - 1, x1 = (int)floor(c.x + r) + 1;
    int y0 = (int)floor(c.y - r) - 1, y1 = (int)floor(c.y + r) + 1;
    double best = 1e9;
    for (int ty = y0; ty <= y1; ty++) {
        for (int tx = x0; tx <= x1; tx++) {
            if (walkable(g, tx, ty)) continue;
            double nx = fmin(fmax(c.x, (double)tx), tx + 1.0);
            double ny = fmin(fmax(c.y, (double)ty), ty + 1.0);
            double d = vdist(c, v(nx, ny));
            if (d < best) best = d;
        }
    }
    return best;
}

static int circle_hits(const Grid* g, V2 c, double r) {
    return wall_dist(g, c, r) < r - 1e-12;
}

static V2 resolve_walls(const Grid* g, V2 c, double r);

/* move by velocity with per-axis blocking: a blocked axis loses its normal
   velocity (classic wall slide) instead of being clamped afterwards. Clamp
   afterwards only corrects residual corner penetration, which at a convex
   inside corner blocks both axes and pins the unit (the real jam). */
static V2 move_ball(const Grid* g, V2 c, double r, V2 vel, double dt) {
    V2 nx = c; nx.x += vel.x * dt;
    if (!circle_hits(g, nx, r)) c = nx;
    V2 ny = c; ny.y += vel.y * dt;
    if (!circle_hits(g, ny, r)) c = ny;
    return resolve_walls(g, c, r);
}

/* push a circle out of every overlapping tile; residual corner correction
   only (per-axis blocking above handles the primary collision response) */
static V2 resolve_walls(const Grid* g, V2 c, double r) {
    for (int pass = 0; pass < 2; pass++) {
        int x0 = (int)floor(c.x - r) - 1, x1 = (int)floor(c.x + r) + 1;
        int y0 = (int)floor(c.y - r) - 1, y1 = (int)floor(c.y + r) + 1;
        for (int ty = y0; ty <= y1; ty++) {
            for (int tx = x0; tx <= x1; tx++) {
                if (walkable(g, tx, ty)) continue;
                double nx = fmin(fmax(c.x, (double)tx), tx + 1.0);
                double ny = fmin(fmax(c.y, (double)ty), ty + 1.0);
                V2 d = vsub(c, v(nx, ny));
                double l = vlen(d);
                if (l >= r) continue;
                if (l > 1e-9) {
                    c = vadd(v(nx, ny), vscale(vnorm(d), r + 1e-9));
                } else {
                    double oxL = c.x - tx, oxR = tx + 1.0 - c.x;
                    double oyT = c.y - ty, oyB = ty + 1.0 - c.y;
                    double m = fmin(fmin(oxL, oxR), fmin(oyT, oyB));
                    if (m == oxL) c.x = tx - r - 1e-9;
                    else if (m == oxR) c.x = tx + 1.0 + r + 1e-9;
                    else if (m == oyT) c.y = ty - r - 1e-9;
                    else c.y = ty + 1.0 + r + 1e-9;
                }
            }
        }
    }
    return c;
}

/* -------------------------------- A* ----------------------------------- */

typedef struct { int x, y; } Pt;
static const int DIRS[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

static int step_ok(const Grid* g, Pt p, Pt q) {
    int dx = q.x - p.x, dy = q.y - p.y;
    if (!walkable(g, q.x, q.y)) return 0;
    if (dx != 0 && dy != 0 && (g->blocked[p.y][q.x] || g->blocked[q.y][p.x])) return 0;
    return 1;
}

static int astar(const Grid* g, Pt s, Pt t, Pt* out, int max_out) {
    if (!walkable(g, s.x, s.y) || !walkable(g, t.x, t.y)) return -1;
    int ncells = g->w * g->h;
    double* gf = (double*)malloc(sizeof(double) * (size_t)ncells);
    double* ff = (double*)malloc(sizeof(double) * (size_t)ncells);
    int* par = (int*)malloc(sizeof(int) * (size_t)ncells);
    unsigned char* op = (unsigned char*)calloc((size_t)ncells, 1);
    unsigned char* cl = (unsigned char*)calloc((size_t)ncells, 1);
    int* heap = (int*)malloc(sizeof(int) * (size_t)ncells);
    if (!gf || !ff || !par || !op || !cl || !heap) exit(2);
    for (int i = 0; i < ncells; i++) { gf[i] = 1e18; par[i] = -1; }
    int hn = 0, si = s.y * g->w + s.x, ti = t.y * g->w + t.x;
    gf[si] = 0.0; ff[si] = hypot((double)(t.x - s.x), (double)(t.y - s.y)); op[si] = 1;
    heap[hn++] = si;
    int found = 0;
    while (hn > 0) {
        int best = 0;
        for (int i = 1; i < hn; i++) if (ff[heap[i]] < ff[heap[best]]) best = i;
        int cur = heap[best]; heap[best] = heap[--hn];
        if (cl[cur]) continue;
        cl[cur] = 1;
        if (cur == ti) { found = 1; break; }
        Pt cp = { cur % g->w, cur / g->w };
        for (int d = 0; d < 8; d++) {
            Pt np = { cp.x + DIRS[d][0], cp.y + DIRS[d][1] };
            if (!step_ok(g, cp, np)) continue;
            int ni = np.y * g->w + np.x;
            if (cl[ni]) continue;
            double step = (DIRS[d][0] && DIRS[d][1]) ? 1.4142135623730951 : 1.0;
            double ng = gf[cur] + step;
            if (!op[ni] || ng < gf[ni]) {
                gf[ni] = ng;
                ff[ni] = ng + hypot((double)(t.x - np.x), (double)(t.y - np.y));
                par[ni] = cur; op[ni] = 1; heap[hn++] = ni;
            }
        }
    }
    int rc = -1;
    if (found) {
        Pt rev[MAX_PATH]; int n = 0, cur = ti;
        while (cur != -1 && n < MAX_PATH) { rev[n++] = (Pt){ cur % g->w, cur / g->w }; cur = par[cur]; }
        if (n <= max_out) {
            for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
            rc = n;
        }
    }
    free(heap); free(cl); free(op); free(par); free(ff); free(gf);
    return rc;
}

static int line_of_sight(const Grid* g, Pt a, Pt b) {
    int steps = abs(b.x - a.x) + abs(b.y - a.y);
    if (steps == 0) return 1;
    Pt prev = a;
    for (int i = 1; i <= steps; i++) {
        double t = (double)i / steps;
        Pt cur = { (int)lround(a.x + (b.x - a.x) * t), (int)lround(a.y + (b.y - a.y) * t) };
        if (cur.x == prev.x && cur.y == prev.y) continue;
        if (!step_ok(g, prev, cur)) return 0;
        prev = cur;
    }
    return prev.x == b.x && prev.y == b.y;
}

static int smooth_path(const Grid* g, const Pt* in, int n, Pt* out, int max_out) {
    if (n <= 0) return 0;
    int m = 0; out[m++] = in[0];
    int anchor = 0;
    while (anchor < n - 1) {
        int next = anchor + 1;
        for (int j = n - 1; j > anchor; j--)
            if (line_of_sight(g, in[anchor], in[j])) { next = j; break; }
        if (m >= max_out) return -1;
        out[m++] = in[next];
        anchor = next;
    }
    return m;
}

/* polyline length and point at a given arc distance */
static double poly_len(const V2* p, int n) {
    double s = 0;
    for (int i = 1; i < n; i++) s += vdist(p[i - 1], p[i]);
    return s;
}
static V2 poly_at(const V2* p, int n, double s) {
    if (n == 1) return p[0];
    double acc = s;
    for (int i = 1; i < n; i++) {
        double seg = vdist(p[i - 1], p[i]);
        if (acc <= seg || i == n - 1)
            return vadd(p[i - 1], vscale(vnorm(vsub(p[i], p[i - 1])), fmin(acc, seg)));
        acc -= seg;
    }
    return p[n - 1];
}
/* arc distance of the nearest projection of c onto the polyline */
static double poly_project(const V2* p, int n, V2 c) {
    double acc = 0, best = 0, bestd = 1e18;
    for (int i = 1; i < n; i++) {
        V2 ab = vsub(p[i], p[i - 1]);
        double l2 = vdot(ab, ab);
        double u = l2 > 1e-12 ? fmin(fmax(vdot(vsub(c, p[i - 1]), ab) / l2, 0.0), 1.0) : 0.0;
        V2 q = vadd(p[i - 1], vscale(ab, u));
        double d = vdist(c, q);
        if (d < bestd) { bestd = d; best = acc + u * sqrt(l2); }
        acc += sqrt(l2);
    }
    return best;
}

/* --------------------------- stuck detector ---------------------------- */

typedef struct {
    double intent[WINDOW];
    double actual[WINDOW];
    int    head, run;          /* run = consecutive armed ticks */
    int    grace;
} Detector;

static void det_init(Detector* d) { memset(d, 0, sizeof(*d)); }

/* returns 1 when a sustained high-intent / near-zero-progress jam is seen.
   Zero-intent ticks (formation halt, arrival, teammate yield) reset the run,
   so ordinary walking can never accumulate a full window. */
static int det_tick(Detector* d, double intent_step, double actual_step) {
    if (d->grace > 0) { d->grace--; d->run = 0; return 0; }
    if (intent_step <= INTENT_RATIO * VMAX * DT) { d->run = 0; return 0; }
    d->intent[d->head] = intent_step;
    d->actual[d->head] = actual_step;
    d->head = (d->head + 1) % WINDOW;
    if (d->run < WINDOW) d->run++;
    if (d->run < WINDOW) return 0;
    double is = 0, as = 0;
    for (int i = 0; i < WINDOW; i++) { is += d->intent[i]; as += d->actual[i]; }
    return is >= MIN_INTENT && as / is < PROGRESS_RATIO;
}

/* ------------------------------ agents --------------------------------- */

typedef enum { RC_NORMAL = 0, RC_STRAFE_A, RC_STRAFE_B, RC_BACK, RC_REPLAN } RecState;
typedef enum { OUT_NONE = 0, OUT_STRAFE, OUT_BACK, OUT_REPLAN, OUT_FAIL } Outcome;

typedef struct {
    V2 pos;
    int goal_cell_x, goal_cell_y;
    double lag;                /* caravan slot distance behind leader */
    Detector det;
    RecState rstate;
    int man_ticks, man_budget, attempts;
    V2 man_start, man_dir;
    int need_replan, replans, replan_requests, replan_ticks;
    V2 own_path[MAX_PATH]; int own_n, own_i;
    V2 last_jam_pos; int last_jam_tick; int jam_repeats;
    int arrived;
    int events, ok_strafe, ok_back, ok_replan, failed, mate_block_ticks;
    int dur_sum, dur_min, dur_max;
} Agent;

typedef struct Sim {
    const char* name;
    Grid grid;
    Agent ag[MAX_AGENTS]; int n;
    V2 route[MAX_PATH]; int route_n;   /* shared caravan polyline */
    double progress;                    /* leader arc distance */
    int solo;                           /* 1: agents steer at own goal cell */
    int tick;
    void (*hook)(struct Sim* sim, int tick);
    struct { int agent; int start; int dur; Outcome out; int near_wall; } ep[MAX_EPISODES];
    int ep_n;
    int collisions, swaps;
    int prev_order[MAX_AGENTS];          /* edge-triggered swap detection */
    unsigned char prev_touch[MAX_AGENTS * MAX_AGENTS];
    double max_dev_normal, max_dev_recovery;
} Sim;

static void request_replan(Sim* s, int idx);
static void note_recovery_exit(Sim* s, int idx);

static int in_recovery(const Agent* a) { return a->rstate != RC_NORMAL; }

static int pos_free_for(Sim* s, int self, V2 p, double r) {
    if (circle_hits(&s->grid, p, r)) return 0;
    for (int i = 0; i < s->n; i++) {
        if (i == self) continue;
        if (vdist(p, s->ag[i].pos) < 2.0 * r + 0.05) return 0;
    }
    return 1;
}

static V2 agent_target(Sim* s, Agent* a) {
    if (a->own_n > 0) {
        V2 wp = a->own_path[a->own_i];
        int cx = (int)floor(wp.x), cy = (int)floor(wp.y);
        int wp_in_wall = !walkable(&s->grid, cx, cy);
        /* a waypoint inside geometry can never be "reached": keep steering at
           it so a genuine jam forms and recovery runs, instead of skipping on
           to the next leg while still trapped */
        if (!wp_in_wall && vdist(a->pos, wp) < 0.20) {
            if (a->own_i + 1 < a->own_n) a->own_i++;
            else a->own_n = 0;
        }
        if (a->own_n > 0) return a->own_path[a->own_i];
    }
    if (s->solo) return cell_center(a->goal_cell_x, a->goal_cell_y);
    double total = poly_len(s->route, s->route_n);
    /* while the caravan moves, follow the paced slot; once the pace reaches
       the end, steer at the member's own parked goal cell */
    if (s->progress >= total - 1e-9)
        return cell_center(a->goal_cell_x, a->goal_cell_y);
    double slot = fmax(0.0, s->progress - a->lag);
    return poly_at(s->route, s->route_n, slot);
}

static void start_maneuver(Sim* s, int idx, RecState st) {
    Agent* a = &s->ag[idx];
    a->rstate = st;
    a->man_start = a->pos;
    a->man_ticks = 0;
    a->man_budget = (st == RC_BACK) ? MAN_BACK_BUDGET : MAN_STRAFE_BUDGET;
    a->man_dir = v(0, 0);
}

static void begin_episode(Sim* s, int idx) {
    Agent* a = &s->ag[idx];
    a->events++;
    if (s->ep_n < MAX_EPISODES) {
        s->ep[s->ep_n].agent = idx;
        s->ep[s->ep_n].start = s->tick;
        s->ep[s->ep_n].dur = 0;
        s->ep[s->ep_n].out = OUT_NONE;
        s->ep[s->ep_n].near_wall = wall_dist(&s->grid, a->pos, RADIUS) < RADIUS + 0.15;
        s->ep_n++;
    }
    /* repeated jam at the same spot: local wiggles already failed there,
       skip straight to back-pedal; a third jam there is unrecoverable
       locally, so flag for re-planning immediately */
    int repeat = s->tick - a->last_jam_tick <= REPEAT_TICKS &&
                 vdist(a->pos, a->last_jam_pos) < REPEAT_DIST;
    if (repeat) {
        a->jam_repeats++;
        if (a->jam_repeats >= 2) {
            a->last_jam_pos = a->pos;
            a->last_jam_tick = s->tick;
            request_replan(s, idx);
            return;
        }
    } else {
        a->jam_repeats = 0;
    }
    a->last_jam_pos = a->pos;
    a->last_jam_tick = s->tick;
    start_maneuver(s, idx, repeat ? RC_BACK : RC_STRAFE_A);
}

static void finish_episode(Sim* s, int idx, Outcome o) {
    Agent* a = &s->ag[idx];
    if (s->ep_n > 0 && s->ep[s->ep_n - 1].agent == idx && s->ep[s->ep_n - 1].out == OUT_NONE) {
        int dur = s->tick - s->ep[s->ep_n - 1].start;
        s->ep[s->ep_n - 1].dur = dur;
        s->ep[s->ep_n - 1].out = o;
        a->dur_sum += dur;
        if (a->dur_min == 0 || dur < a->dur_min) a->dur_min = dur;
        if (dur > a->dur_max) a->dur_max = dur;
    }
    if (o == OUT_FAIL) a->failed++;
    else if (o == OUT_STRAFE) a->ok_strafe++;
    else if (o == OUT_BACK) a->ok_back++;
    else if (o == OUT_REPLAN) a->ok_replan++;
    /* a successful escape means the unreachable in-wall waypoint is moot: drop
       it so the unit resumes on the next reachable leg toward its goal */
    if (o != OUT_FAIL && a->own_n > 0) {
        V2 wp = a->own_path[a->own_i];
        if (!walkable(&s->grid, (int)floor(wp.x), (int)floor(wp.y))) {
            if (a->own_i + 1 < a->own_n) a->own_i++;
            else a->own_n = 0;
        }
    }
    note_recovery_exit(s, idx);
    a->rstate = RC_NORMAL;
    a->man_ticks = 0;
    det_init(&a->det);
    a->det.grace = GRACE_TICKS;   /* avoid re-triggering the instant we resume */
}

/* successful local recoveries still leave the unit near the same hazard;
   clear the repeat counter only once it has genuinely moved on */
static void note_recovery_exit(Sim* s, int idx) {
    Agent* a = &s->ag[idx];
    if (vdist(a->pos, a->last_jam_pos) > REPEAT_DIST) a->jam_repeats = 0;
}

static void request_replan(Sim* s, int idx) {
    Agent* a = &s->ag[idx];
    a->need_replan = 1;
    a->replan_requests++;
    a->rstate = RC_REPLAN;
    a->replan_ticks = 0;
}

/* build a fresh A* route; temporarily block the cell the unit was pushing
   into so the new route actually differs from the failing one */
static int try_replan(Sim* s, int idx) {
    Agent* a = &s->ag[idx];
    Pt sx = { (int)floor(a->pos.x), (int)floor(a->pos.y) };
    Pt tx = { a->goal_cell_x, a->goal_cell_y };
    V2 intend = vnorm(vsub(agent_target(s, a), a->pos));
    Pt blk = { (int)floor(a->pos.x + intend.x * 0.9), (int)floor(a->pos.y + intend.y * 0.9) };
    int have_blk = walkable(&s->grid, blk.x, blk.y) &&
                   !(blk.x == sx.x && blk.y == sx.y) && !(blk.x == tx.x && blk.y == tx.y);
    if (have_blk) s->grid.blocked[blk.y][blk.x] = 1;
    Pt cells[MAX_PATH];
    int n = astar(&s->grid, sx, tx, cells, MAX_PATH);
    if (have_blk) s->grid.blocked[blk.y][blk.x] = 0;
    if (n <= 0) return 0;
    a->own_n = n;
    for (int i = 0; i < n; i++) a->own_path[i] = cell_center(cells[i].x, cells[i].y);
    a->own_i = 0;
    a->replans++;
    a->need_replan = 0;
    return 1;
}

static void escalate(Sim* s, int idx) {
    Agent* a = &s->ag[idx];
    a->attempts++;
    if (a->rstate == RC_STRAFE_A) { start_maneuver(s, idx, RC_STRAFE_B); return; }
    if (a->rstate == RC_STRAFE_B) { start_maneuver(s, idx, RC_BACK); return; }
    request_replan(s, idx);       /* three local maneuvers failed */
}

/* strafe direction perpendicular to intent, preferring the open side */
static V2 strafe_dir(Sim* s, int idx, int side) {
    Agent* a = &s->ag[idx];
    V2 intend = vnorm(vsub(agent_target(s, a), a->pos));
    V2 d1 = v(-intend.y, intend.x), d2 = vscale(d1, -1.0);
    V2 pick = side == 0 ? d1 : d2, other = side == 0 ? d2 : d1;
    if (pos_free_for(s, idx, vadd(a->pos, vscale(pick, 0.6)), RADIUS)) return pick;
    if (pos_free_for(s, idx, vadd(a->pos, vscale(other, 0.6)), RADIUS)) return other;
    return v(0, 0);
}

static void recovery_tick(Sim* s, int idx, V2* desired) {
    Agent* a = &s->ag[idx];
    if (a->rstate == RC_REPLAN) {
        *desired = v(0, 0);
        if (a->replan_ticks == 0 || a->replan_ticks % REPLAN_RETRY == 0) {
            if (try_replan(s, idx)) { finish_episode(s, idx, OUT_REPLAN); return; }
        }
        if (++a->replan_ticks > REPLAN_CAP) finish_episode(s, idx, OUT_FAIL);
        return;
    }
    if (a->man_ticks == 0) {
        V2 d = v(0, 0);
        if (a->rstate == RC_BACK) {
            V2 back = vscale(vnorm(vsub(agent_target(s, a), a->pos)), -1.0);
            if (pos_free_for(s, idx, vadd(a->pos, vscale(back, 0.6)), RADIUS)) d = back;
        } else {
            d = strafe_dir(s, idx, a->rstate == RC_STRAFE_A ? 0 : 1);
        }
        if (d.x == 0.0 && d.y == 0.0) { escalate(s, idx); return; }
        a->man_dir = d;
    }
    a->man_ticks++;
    *desired = vscale(a->man_dir, MAN_SPEED);
    int moved_enough = vdist(a->pos, a->man_start) >= MAN_OK_DIST &&
                       !circle_hits(&s->grid, a->pos, RADIUS);
    if (moved_enough) {
        finish_episode(s, idx, a->rstate == RC_BACK ? OUT_BACK : OUT_STRAFE);
        return;
    }
    if (a->man_ticks >= a->man_budget) escalate(s, idx);
}

/* ----------------------------- simulation ------------------------------ */

static void sim_init(Sim* s, const char* name, const char* const* rows, int h,
                     const int* starts, int n, const double* lags, Pt goal,
                     int solo, void (*hook)(struct Sim*, int)) {
    memset(s, 0, sizeof(*s));
    s->name = name;
    if (grid_load(&s->grid, rows, h) != 0) { printf("grid load fail\n"); exit(2); }
    s->n = n; s->solo = solo; s->hook = hook;
    for (int i = 0; i < n; i++) {
        Agent* a = &s->ag[i];
        a->pos = cell_center(starts[2 * i], starts[2 * i + 1]);
        a->goal_cell_x = goal.x; a->goal_cell_y = goal.y;
        a->lag = lags ? lags[i] : 0.0;
        det_init(&a->det);
    }
}

/* shared A* caravan route from the first agent to the goal */
static void build_route(Sim* s) {
    Pt s0 = { (int)floor(s->ag[0].pos.x), (int)floor(s->ag[0].pos.y) };
    Pt t0 = { s->ag[0].goal_cell_x, s->ag[0].goal_cell_y };
    Pt raw[MAX_PATH], sm[MAX_PATH];
    int n = astar(&s->grid, s0, t0, raw, MAX_PATH);
    if (n < 0) { printf("no route\n"); exit(2); }
    int m = smooth_path(&s->grid, raw, n, sm, MAX_PATH);
    s->route_n = m;
    for (int i = 0; i < m; i++) s->route[i] = cell_center(sm[i].x, sm[i].y);
}

static void inject_waypoint(Sim* s, V2 wp) {
    V2 old[MAX_PATH];
    int on = s->route_n;
    memcpy(old, s->route, sizeof(V2) * (size_t)on);
    s->route[0] = old[0];
    s->route[1] = wp;
    memmove(s->route + 2, old + 1, sizeof(V2) * (size_t)(on - 1));
    s->route_n = on + 1;
}

static int any_recovery(Sim* s) {
    for (int i = 0; i < s->n; i++) if (in_recovery(&s->ag[i])) return 1;
    return 0;
}

/* a teammate directly ahead blocking the lane: yields are not jams */
static int mate_blocked(Sim* s, int idx, V2 intend) {
    Agent* a = &s->ag[idx];
    for (int i = 0; i < s->n; i++) {
        if (i == idx) continue;
        V2 d = vsub(s->ag[i].pos, a->pos);
        double gap = vlen(d);
        if (gap < 2.0 * RADIUS + 0.18 && vdot(vnorm(d), intend) > 0.0) return 1;
        /* caravan followers must keep their spacing behind the unit ahead */
        if (!s->solo && gap < 2.0 * RADIUS + 0.30 && vdot(vnorm(d), intend) > 0.5) return 1;
    }
    return 0;
}

/* a genuine collision is an overlap that survives into the next tick; the
   resolver's same-tick correction of momentary contact is formation keeping,
   not a breach */
static void count_overlaps(Sim* s) {
    for (int i = 0; i < s->n; i++) {
        for (int j = i + 1; j < s->n; j++) {
            int touch = vdist(s->ag[i].pos, s->ag[j].pos) < 2.0 * RADIUS - 1e-3;
            int key = i * MAX_AGENTS + j;
            if (touch && s->prev_touch[key]) s->collisions++;
            s->prev_touch[key] = touch;
        }
    }
}

static void separate_pairs(Sim* s) {
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < s->n; i++) {
            for (int j = i + 1; j < s->n; j++) {
                int ia = s->ag[i].arrived, ja = s->ag[j].arrived;
                if (ia && ja) continue;              /* settled pair stays put */
                V2 d = vsub(s->ag[i].pos, s->ag[j].pos);
                double l = vlen(d);
                double want = 2.0 * RADIUS;
                if (l >= want || l < 1e-12) continue;
                if (ia || ja) {
                    /* push only the moving one, the settled unit is anchored */
                    int k = ia ? j : i;
                    V2 away = vscale(vnorm(ia ? vsub(s->ag[k].pos, s->ag[i].pos)
                                              : vsub(s->ag[k].pos, s->ag[j].pos)),
                                     want - l);
                    s->ag[k].pos = vadd(s->ag[k].pos, away);
                } else {
                    V2 push = vscale(vnorm(d), (want - l) * 0.5);
                    s->ag[i].pos = vadd(s->ag[i].pos, push);
                    s->ag[j].pos = vsub(s->ag[j].pos, push);
                }
            }
        }
    }
}

static void sim_step(Sim* s) {
    if (s->hook) s->hook(s, s->tick);
    double route_total = poly_len(s->route, s->route_n);

    for (int i = 0; i < s->n; i++) {
        Agent* a = &s->ag[i];
        V2 before = a->pos;
        if (a->arrived) continue;          /* settled: no steering, no pushing */
        V2 target = agent_target(s, a);
        V2 to = vsub(target, a->pos);
        double dist = vlen(to);
        V2 desired = v(0, 0);
        if (in_recovery(a)) {
            recovery_tick(s, i, &desired);
        } else if (dist > ARRIVE_DIST) {
            V2 dir = vnorm(to);
            /* small steering jitter, present in every scenario; it scales with
               forward speed so a near-stationary unit never drifts sideways */
            double speed = fmin(VMAX, dist / DT);
            double ang = (randf() - 0.5) * 0.10 * (speed / VMAX);
            V2 jdir = v(dir.x * cos(ang) - dir.y * sin(ang),
                         dir.x * sin(ang) + dir.y * cos(ang));
            desired = vscale(jdir, speed);
        }
        double intent_step = vlen(desired) * DT;
        int mb = (!in_recovery(a) && intent_step > 0 && mate_blocked(s, i, vnorm(desired)));
        if (mb) { a->mate_block_ticks++; intent_step = 0.0; }

        /* caravan car-following: a follower never closes inside the spacing
           toward the member immediately ahead of it */
        if (!s->solo && i > 0 && !in_recovery(a) && intent_step > 0.0) {
            Agent* ahead = &s->ag[i - 1];
            V2 gapv = vsub(ahead->pos, a->pos);
            double gap = vlen(gapv);
            double follow = 2.0 * RADIUS + 0.55;
            V2 fwd = vnorm(desired);
            if (gap < follow && vdot(vnorm(gapv), fwd) > 0.3) {
                double scale = fmax(0.0, (gap - 2.0 * RADIUS) / (follow - 2.0 * RADIUS));
                desired = vscale(desired, scale);
                intent_step = vlen(desired) * DT;
            }
        }

        /* recovery maneuvers also yield to teammates: cancel the component of
           the recovery velocity that would drive into a nearby member, so a
           strafe never breaks the formation to escape a wall */
        if (in_recovery(a) && intent_step > 0.0) {
            for (int k = 0; k < s->n; k++) {
                if (k == i || s->ag[k].arrived) continue;
                V2 d = vsub(s->ag[k].pos, a->pos);
                double gap = vlen(d);
                double sense = 2.0 * RADIUS + 0.35;
                if (gap < sense && vdot(vnorm(d), vnorm(desired)) > 0.2) {
                    V2 away = v(-vnorm(d).y, vnorm(d).x);   /* perpendicular */
                    desired = vadd(vscale(desired, 0.3), vscale(away, MAN_SPEED * 0.7));
                    intent_step = vlen(desired) * DT;
                }
            }
        }

        /* hard spacing guard: regardless of intent (even when a follower's
           slot is clamped at the route head), never let two members overlap;
           steer the rear one back along the line joining them */
        if (!in_recovery(a)) {
            for (int k = 0; k < s->n; k++) {
                if (k == i || s->ag[k].arrived) continue;
                V2 d = vsub(a->pos, s->ag[k].pos);
                double gap = vlen(d);
                double hard = 2.0 * RADIUS + 0.12;
                if (gap < hard && gap > 1e-9) {
                    V2 away = vscale(vnorm(d), (hard - gap));
                    desired = vadd(desired, vscale(away, 1.0 / DT));
                    intent_step = vlen(desired) * DT;
                }
            }
        }

        a->pos = move_ball(&s->grid, a->pos, RADIUS, desired, DT);
        separate_pairs(s);
        for (int j = 0; j < s->n; j++)
            s->ag[j].pos = resolve_walls(&s->grid, s->ag[j].pos, RADIUS);

        double actual_step = vdist(a->pos, before);
        if (trace_on)
            fprintf(stderr, "t=%d a=%d pos=(%.3f,%.3f) tgt=(%.3f,%.3f) goal=(%.1f,%.1f) intent=%.4f actual=%.4f rec=%d arr=%d\n",
                    s->tick, i, a->pos.x, a->pos.y, target.x, target.y,
                    a->goal_cell_x + 0.5, a->goal_cell_y + 0.5,
                    intent_step, actual_step, a->rstate, a->arrived);
        if (!in_recovery(a) && det_tick(&a->det, intent_step, actual_step))
            begin_episode(s, i);
        /* settle on the own goal cell once close enough */
        V2 goal = cell_center(a->goal_cell_x, a->goal_cell_y);
        if (vdist(a->pos, goal) < 0.30) a->arrived = 1;
    }

    /* caravan pace tracks the leader's real arc position, capped by its max
       speed so it can never teleport ahead; followers inherit that pace */
    if (!s->solo && !any_recovery(s) && s->progress < route_total - 1e-9) {
        double lead = poly_project(s->route, s->route_n, s->ag[0].pos);
        double want = fmin(route_total, s->progress + VMAX * DT);
        s->progress = fmin(want, lead + 0.35);
    }

    /* formation bookkeeping */
    count_overlaps(s);
    int rec_now = any_recovery(s);
    double prog[MAX_AGENTS];
    for (int i = 0; i < s->n; i++) {
        Agent* a = &s->ag[i];
        /* the ideal position must match agent_target: paced slot while moving,
           the member's own goal cell once the caravan reaches the end */
        V2 ideal = agent_target(s, a);
        double dev = vdist(a->pos, ideal);
        if (rec_now && dev > s->max_dev_recovery) s->max_dev_recovery = dev;
        if (!rec_now && dev > s->max_dev_normal) s->max_dev_normal = dev;
        prog[i] = s->solo ? 0.0 : poly_project(s->route, s->route_n, a->pos);
    }
    /* edge-triggered: count a swap only when the arc-length order actually
       inverts between adjacent members by a meaningful margin */
    if (!s->solo) {
        for (int i = 1; i < s->n; i++) {
            int inverted = prog[i - 1] < prog[i] - 0.45;
            if (inverted && !s->prev_order[i]) s->swaps++;
            s->prev_order[i] = inverted;
        }
    }
    s->tick++;
}

static int all_arrived(Sim* s) {
    for (int i = 0; i < s->n; i++)
        if (!s->ag[i].arrived) return 0;
    return 1;
}

static void sim_run(Sim* s, int max_ticks) {
    while (s->tick < max_ticks && !all_arrived(s)) sim_step(s);
}

/* ------------------------------- metrics ------------------------------- */

typedef struct {
    int scenarios, fp_scenarios;
    int events, strafe, back, replan, fail, replan_flags, near_wall_ok;
    int dur_min, dur_sum, dur_max, dur_n;
    int collisions, swaps;
    double max_dev_rec, max_dev_normal;
    int checks_failed;
} Report;

static void report_scenario(Report* r, Sim* s, int fp_scenario, int expect_jam) {
    int ev = 0, st = 0, bk = 0, rp = 0, fl = 0, flags = 0;
    int dmin = 0, dsum = 0, dmax = 0, dn = 0;
    for (int i = 0; i < s->n; i++) {
        Agent* a = &s->ag[i];
        ev += a->events; st += a->ok_strafe; bk += a->ok_back;
        rp += a->ok_replan; fl += a->failed; flags += a->replan_requests;
        dsum += a->dur_sum; dmax = fmax(dmax, a->dur_max);
        if (a->dur_min > 0) dmin = dmin == 0 ? a->dur_min : fmin(dmin, a->dur_min);
        dn += a->ok_strafe + a->ok_back + a->ok_replan + a->failed;
    }
    int wallok = 1, near_wall_events = 0;
    for (int e = 0; e < s->ep_n; e++) {
        if (s->ep[e].near_wall) near_wall_events++;
        else wallok = 0;
    }
    int arrived_n = 0;
    for (int i = 0; i < s->n; i++) if (s->ag[i].arrived) arrived_n++;
    int arrived = arrived_n == s->n;
    printf("== %s ==\n", s->name);
    printf("ticks=%d (%.1fs) arrived=%d/%d\n", s->tick, s->tick * DT, arrived_n, s->n);
    printf("stuck=%d resolved=%d failed=%d outcomes strafe=%d back=%d replan=%d\n",
           ev, st + bk + rp, fl, st, bk, rp);
    if (dn > 0)
        printf("recover_ticks min/mean/max=%d/%.1f/%d  seconds min/mean/max=%.2f/%.2f/%.2f\n",
               dmin, (double)dsum / dn, dmax, dmin * DT, dsum * DT / dn, dmax * DT);
    printf("replan_flags=%d trigger_near_wall=%d\n", flags, wallok);
    printf("formation collisions=%d swaps=%d max_dev_normal=%.3f max_dev_recovery=%.3f\n",
           s->collisions, s->swaps, s->max_dev_normal, s->max_dev_recovery);

    int ok = 1;
    if (fp_scenario) {
        ok &= (ev == 0);
        printf("check_no_false_positive=%d\n", ev == 0);
    }
    if (expect_jam) {
        ok &= (ev > 0);
        printf("check_stuck_detected=%d\n", ev > 0);
        ok &= (fl == 0 && arrived);
        printf("check_all_recovered_and_arrived=%d\n", fl == 0 && arrived);
        ok &= wallok;
        printf("check_every_trigger_at_obstacle=%d\n", wallok);
    } else {
        ok &= arrived;
        printf("check_all_arrived=%d\n", arrived);
    }
    if (s->n > 1) {
        int form = s->collisions == 0 && s->swaps == 0 && s->max_dev_recovery < 1.3;
        ok &= form;
        printf("check_formation_intact=%d\n", form);
    }
    if (!ok) r->checks_failed++;

    r->scenarios++;
    if (fp_scenario) r->fp_scenarios++;
    r->events += ev; r->strafe += st; r->back += bk; r->replan += rp; r->fail += fl;
    r->replan_flags += flags; r->near_wall_ok += near_wall_events;
    r->dur_sum += dsum; r->dur_n += dn; r->dur_max = fmax(r->dur_max, dmax);
    if (dmin > 0) r->dur_min = r->dur_min == 0 ? dmin : fmin(r->dur_min, dmin);
    r->collisions += s->collisions; r->swaps += s->swaps;
    r->max_dev_rec = fmax(r->max_dev_rec, s->max_dev_recovery);
    r->max_dev_normal = fmax(r->max_dev_normal, s->max_dev_normal);
}

/* ------------------------------ scenarios ------------------------------ */

/* give each caravan member its own goal cell along the route tail, spaced by
   its lag, so the formation parks in a column instead of stacking on one cell */
static void assign_column_goals(Sim* s) {
    double total = poly_len(s->route, s->route_n);
    for (int i = 0; i < s->n; i++) {
        double arc = fmax(0.0, total - s->ag[i].lag);
        V2 p = poly_at(s->route, s->route_n, arc);
        s->ag[i].goal_cell_x = (int)floor(p.x);
        s->ag[i].goal_cell_y = (int)floor(p.y);
    }
}

/* single units follow the polyline as explicit waypoints instead of the
   caravan pace, which would deadlock when the leader is the only member */
static void follow_route_solo(Sim* s) {
    for (int i = 0; i < s->n && i < MAX_PATH; i++) {
        s->ag[i].own_n = s->route_n < MAX_PATH ? s->route_n : MAX_PATH;
        for (int k = 0; k < s->ag[i].own_n; k++) s->ag[i].own_path[k] = s->route[k];
        s->ag[i].own_i = 0;
    }
}

/* S6: timed gate sealing an alcove once the unit is inside. It reopens after
   the unit has exhausted local maneuvers and asked for a re-plan, modelling an
   obstacle that clears, so the recovery pipeline ends in a successful replan */
static int gate_x, gate_y;
static void hook_gate(Sim* s, int tick) {
    Agent* a = &s->ag[0];
    /* the gate spans the corridor mouth: close once the unit is below it */
    if (!s->grid.blocked[gate_y][gate_x] &&
        a->pos.y > gate_y + 0.75 &&
        fabs(a->pos.x - (gate_x + 0.5)) < 0.6)
        s->grid.blocked[gate_y][gate_x] = 1;
    if (s->grid.blocked[gate_y][gate_x] &&
        (a->replan_requests > 0 || tick == 320))
        s->grid.blocked[gate_y][gate_x] = 0;
}

int main(void) {
    Report R; memset(&R, 0, sizeof(R));
    trace_on = getenv("STUCK_TRACE") != 0;

    /* S1 open-field walk, 3-unit column, steering noise on */
    {
        const char* rows[] = {
            "################",
            "#..............#",
            "#..............#",
            "#..............#",
            "#..............#",
            "################",
        };
        int starts[] = {1, 3, 1, 3, 1, 3};
        double lags[] = {0.0, 1.1, 2.2};
        Sim s;
        sim_init(&s, "S1_open_walk", rows, 6, starts, 3, lags, (Pt){14, 3}, 0, 0);
        build_route(&s);
        assign_column_goals(&s);
        /* leader (i=0) at the front, followers stacked behind it */
        for (int i = 0; i < s.n; i++) s.ag[i].pos = v(1.5 + (s.n - 1 - i) * 1.1, 3.5);
        /* pace starts at the rearmost lag so every follower slot maps to a
           real, non-clamped arc position matching its physical spot */
        s.progress = s.ag[s.n - 1].lag;
        sim_run(&s, 400);
        report_scenario(&R, &s, 1, 0);
    }

    /* S2 sustained wall slide: real contact, good tangential progress */
    {
        const char* rows[] = {
            "################",
            "#..............#",
            "#..............#",
            "#..............#",
            "##############.#",
            "################",
        };
        int starts[] = {1, 1};
        Sim s;
        sim_init(&s, "S2_wall_slide", rows, 6, starts, 1, 0, (Pt){14, 3}, 1, 0);
        s.solo = 0; s.route_n = 3;
        s.route[0] = v(1.5, 1.5);
        s.route[1] = v(10.5, 3.72);   /* skims the wall top at y=4 */
        s.route[2] = cell_center(14, 3);
        sim_run(&s, 400);
        report_scenario(&R, &s, 1, 0);
    }

    /* S3 formation halted externally for 2 s: zero intent, must not fire */
    {
        const char* rows[] = {
            "################",
            "#..............#",
            "#..............#",
            "#..............#",
            "#..............#",
            "################",
        };
        int starts[] = {1, 3, 1, 3, 1, 3};
        double lags[] = {0.0, 1.1, 2.2};
        Sim s;
        sim_init(&s, "S3_formation_halt", rows, 6, starts, 3, lags, (Pt){14, 3}, 0, 0);
        build_route(&s);
        assign_column_goals(&s);
        for (int i = 0; i < s.n; i++) s.ag[i].pos = v(1.5 + (s.n - 1 - i) * 1.1, 3.5);
        /* pace starts at the rearmost lag so every follower slot maps to a
           real, non-clamped arc position matching its physical spot */
        s.progress = s.ag[s.n - 1].lag;
        double frozen = -1;
        while (s.tick < 400 && !all_arrived(&s)) {
            sim_step(&s);
            if (s.tick >= 80 && s.tick < 120) {
                if (frozen < 0) frozen = s.progress;
                s.progress = frozen;
                if (s.tick == 119) frozen = -1;
            }
        }
        report_scenario(&R, &s, 1, 0);
    }

    /* S4 two solo agents on crossing paths in open space: brief yield, no jam */
    {
        const char* rows[] = {
            "##############",
            "#............#",
            "#............#",
            "#............#",
            "#............#",
            "#............#",
            "#............#",
            "##############",
        };
        int starts[] = {1, 2, 1, 5};
        Sim s;
        sim_init(&s, "S4_crossing_yield", rows, 8, starts, 2, 0, (Pt){11, 2}, 1, 0);
        s.ag[0].pos = cell_center(1, 2);
        s.ag[1].pos = cell_center(1, 5);
        s.ag[1].goal_cell_x = 11; s.ag[1].goal_cell_y = 5;
        sim_run(&s, 400);
        report_scenario(&R, &s, 1, 0);
    }

    /* S5 U-shaped niche: the unit walks into a three-walled pocket whose mouth
       is behind it. Front and both sideways directions are blocked, so both
       strafes fail and back-pedal toward the mouth is the only local escape */
    {
        const char* rows[] = {
            "#############",
            "#...........#",
            "#...........#",
            "###.#########",
            "###.#########",
            "###.#########",
            "#############",
            "#############",
            "#############",
        };
        int starts[] = {3, 2};
        Sim s;
        sim_init(&s, "S5_deadend_corner", rows, 9, starts, 1, 0, (Pt){11, 2}, 0, 0);
        s.route_n = 3;
        s.route[0] = cell_center(3, 2);
        s.route[1] = v(3.5, 6.30);          /* cell (3,6) is the sealed back wall */
        s.route[2] = cell_center(11, 2);
        follow_route_solo(&s);
        sim_run(&s, 700);
        report_scenario(&R, &s, 0, 1);
    }

    /* S6 sealed alcove with timed gate: straight corridor in, the gate
       closes behind the unit; both strafes and the back-pedal hit walls,
       so repeated jams raise the replan flag */
    {
        const char* rows[] = {
            "###########",
            "#.........#",
            "#######.###",
            "#######.###",
            "#######.###",
            "#######.###",
            "#######.###",
            "###########",
        };
        gate_x = 7; gate_y = 2;
        int starts[] = {7, 1};
        Sim s;
        sim_init(&s, "S6_alcove_gate", rows, 8, starts, 1, 0, (Pt){1, 1}, 1, hook_gate);
        s.route_n = 3;
        s.route[0] = cell_center(7, 1);
        s.route[1] = cell_center(7, 5);   /* bottom of the dead-end shaft */
        s.route[2] = cell_center(1, 1);
        follow_route_solo(&s);
        sim_run(&s, 900);
        report_scenario(&R, &s, 0, 1);
    }

    /* S7 five-unit column grazing inside corners: formation must hold */
    {
        /* wide (3-cell) corridors with inside corners that can pin a unit,
           enough room for the column to pass without stacking */
        const char* rows[] = {
            "####################",
            "#..................#",
            "#..#########.......#",
            "#..#.......#.......#",
            "#..#.......#.......#",
            "#..#.......#########",
            "#..#...............#",
            "#..#################",
            "#..................#",
            "####################",
        };
        int starts[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
        double lags[] = {0.0, 1.1, 2.2, 3.3, 4.4};
        Sim s;
        sim_init(&s, "S7_squad_gauntlet", rows, 10, starts, 5, lags, (Pt){18, 8}, 0, 0);
        build_route(&s);
        inject_waypoint(&s, cell_center(3, 5));   /* graze the first inside corner */
        assign_column_goals(&s);
        for (int i = 0; i < s.n; i++) s.ag[i].pos = v(1.5 + (s.n - 1 - i) * 1.1, 1.5);
        /* pace starts at the rearmost lag so every follower slot maps to a
           real, non-clamped arc position matching its physical spot */
        s.progress = s.ag[s.n - 1].lag;
        sim_run(&s, 1400);
        report_scenario(&R, &s, 0, 1);
    }

    printf("== summary ==\n");
    printf("scenarios=%d false_positive_scenarios_clean=%d/%d\n",
           R.scenarios, R.fp_scenarios, R.fp_scenarios);
    printf("stuck_events=%d recovered=%d failed=%d success_rate=%.1f%%\n",
           R.events, R.strafe + R.back + R.replan, R.fail,
           R.events ? 100.0 * (R.strafe + R.back + R.replan) / R.events : 100.0);
    printf("outcomes strafe=%d back=%d replan=%d replan_flags=%d triggers_at_obstacle=%d/%d\n",
           R.strafe, R.back, R.replan, R.replan_flags, R.near_wall_ok, R.events);
    if (R.dur_n > 0)
        printf("recover_seconds min/mean/max=%.2f/%.2f/%.2f\n",
               R.dur_min * DT, R.dur_sum * DT / R.dur_n, R.dur_max * DT);
    printf("formation collisions=%d swaps=%d max_dev_normal=%.3f max_dev_recovery=%.3f\n",
           R.collisions, R.swaps, R.max_dev_normal, R.max_dev_rec);
    printf("ALL_CHECKS=%d\n", R.checks_failed == 0);
    return R.checks_failed ? 1 : 0;
}
