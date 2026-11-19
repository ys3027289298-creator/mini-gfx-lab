
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

/* ------------------------------------------------------------------ */
/* Patrol routes: looping waypoints with per-point dwell, knockback
   recovery, frame-exact timing and a path cache that survives route
   edits.  All timing is integer ticks; one tick is one frame. */

#define MAX_WAYPOINTS 16
#define PATROL_PATH_POINTS 512
#define PATH_CACHE_SLOTS 32

typedef struct { Pt pts[PATROL_PATH_POINTS]; int n; } Path;

typedef struct {
    Pt from, to;
    Path path;
    int valid;
    unsigned long stamp;
} CacheSlot;

typedef struct {
    const Grid* grid;
    CacheSlot slots[PATH_CACHE_SLOTS];
    unsigned long clock;
    int plan_calls;          /* telemetry: number of A* runs */
} PathCache;

static void cache_init(PathCache* c, const Grid* g) {
    memset(c, 0, sizeof(*c));
    c->grid = g;
}

static void cache_clear(PathCache* c) {
    memset(c->slots, 0, sizeof(c->slots));
    c->clock = 0;
    c->plan_calls = 0;
}

/* Look up a smoothed path between two cells, planning only on a miss.
   Paths are keyed by their endpoints, so editing an unrelated part of a
   route never invalidates the legs that are still in use. */
static const Path* cache_path(PathCache* c, Pt from, Pt to) {
    CacheSlot* free_slot = NULL;
    for (int i = 0; i < PATH_CACHE_SLOTS; i++) {
        CacheSlot* s = &c->slots[i];
        if (!s->valid) {
            if (!free_slot) free_slot = s;
            continue;
        }
        if (s->from.x == from.x && s->from.y == from.y &&
            s->to.x == to.x && s->to.y == to.y) {
            s->stamp = ++c->clock;
            return &s->path;
        }
    }
    if (!free_slot) {
        free_slot = &c->slots[0];
        for (int i = 1; i < PATH_CACHE_SLOTS; i++)
            if (c->slots[i].stamp < free_slot->stamp) free_slot = &c->slots[i];
    }

    Pt raw[MAX_NODES];
    int n = astar(c->grid, from, to, raw, MAX_NODES);
    c->plan_calls++;
    if (n < 0) return NULL;

    int m = smooth(c->grid, raw, n, free_slot->path.pts, PATROL_PATH_POINTS);
    if (m < 0) {
        /* A very twisty route can smooth into more points than we store. */
        if (n > PATROL_PATH_POINTS) return NULL;
        memcpy(free_slot->path.pts, raw, sizeof(Pt) * (size_t)n);
        m = n;
    }
    free_slot->path.n = m;
    free_slot->from = from;
    free_slot->to = to;
    free_slot->valid = 1;
    free_slot->stamp = ++c->clock;
    return &free_slot->path;
}

typedef struct {
    Pt wp[MAX_WAYPOINTS];
    int dwell[MAX_WAYPOINTS];   /* frames spent at each waypoint */
    int n;
} Route;

typedef enum { PATROL_STOPPED = 0, PATROL_DWELL, PATROL_TRAVEL } PatrolState;

typedef struct {
    PathCache* cache;
    Route route;
    int cell_frames;            /* frames to cross one grid cell */

    PatrolState state;
    int cur_wp;                 /* last waypoint reached */
    int target_wp;              /* waypoint currently travelled to */
    int dwell_left;
    int laps;
    int started;                /* left waypoint 0 at least once */
    int recovering;             /* travelling back after a knockback */

    Pt cell;                    /* last grid point reached exactly */
    double x, y;                /* interpolated position, render only */

    Path leg;                   /* active leg, copied from the cache */
    int leg_i;                  /* index of the point being headed to */
    Pt seg_a, seg_b;            /* current rendered segment */
    int seg_frames, move_left;  /* both integer: no drift accumulates */
} Patrol;

static int route_valid(const Grid* g, const Route* r) {
    if (r->n < 1 || r->n > MAX_WAYPOINTS) return 0;
    for (int i = 0; i < r->n; i++) {
        if (!walkable(g, r->wp[i].x, r->wp[i].y)) return 0;
        if (r->dwell[i] < 0) return 0;
    }
    return 1;
}

static int route_set(Route* r, const Grid* g, const Pt* wp, const int* dwell, int n) {
    Route tmp;
    if (n < 1 || n > MAX_WAYPOINTS) return -1;
    memcpy(tmp.wp, wp, sizeof(Pt) * (size_t)n);
    memcpy(tmp.dwell, dwell, sizeof(int) * (size_t)n);
    tmp.n = n;
    if (!route_valid(g, &tmp)) return -1;
    *r = tmp;
    return 0;
}

static void patrol_init(Patrol* p, PathCache* cache, const Route* route, int cell_frames) {
    memset(p, 0, sizeof(*p));
    p->cache = cache;
    p->route = *route;
    p->cell_frames = cell_frames > 0 ? cell_frames : 1;
    p->state = PATROL_STOPPED;
}

static int patrol_start(Patrol* p, int wp_index) {
    if (wp_index < 0 || wp_index >= p->route.n) return -1;
    p->cur_wp = p->target_wp = wp_index;
    p->cell = p->route.wp[wp_index];
    p->x = (double)p->cell.x;
    p->y = (double)p->cell.y;
    p->state = PATROL_DWELL;
    p->dwell_left = p->route.dwell[wp_index];
    p->laps = 0;
    p->started = 0;
    p->recovering = 0;
    p->leg.n = 0;
    p->leg_i = 0;
    p->move_left = 0;
    p->seg_frames = 0;
    return 0;
}

/* BFS distance makes "nearest waypoint after a knockback" follow walkable
   space instead of straight-line distance through walls. */
static void bfs_dist(const Grid* g, Pt src, int* dist) {
    int n = g->w * g->h;
    for (int i = 0; i < n; i++) dist[i] = -1;
    int* queue = (int*)malloc(sizeof(int) * (size_t)n);
    if (!queue) return;
    int head = 0, tail = 0;
    int si = src.y * g->w + src.x;
    dist[si] = 0;
    queue[tail++] = si;
    while (head < tail) {
        int cur = queue[head++];
        Pt cp = { cur % g->w, cur / g->w };
        for (int d = 0; d < 8; d++) {
            Pt np = { cp.x + DIRS[d][0], cp.y + DIRS[d][1] };
            if (!step_ok(g, cp, np)) continue;
            int ni = np.y * g->w + np.x;
            if (dist[ni] >= 0) continue;
            dist[ni] = dist[cur] + 1;
            queue[tail++] = ni;
        }
    }
    free(queue);
}

static int patrol_nearest_wp(const Patrol* p, Pt from) {
    int dist[MAX_NODES];
    bfs_dist(p->cache->grid, from, dist);
    int best = -1, best_d = 0;
    for (int i = 0; i < p->route.n; i++) {
        Pt w = p->route.wp[i];
        int d = dist[w.y * p->cache->grid->w + w.x];
        if (d < 0) continue;
        if (best < 0 || d < best_d) { best = i; best_d = d; }
    }
    return best;
}

static void patrol_arrive(Patrol* p) {
    p->cell = p->route.wp[p->target_wp];
    p->x = (double)p->cell.x;
    p->y = (double)p->cell.y;
    p->cur_wp = p->target_wp;
    if (p->target_wp == 0 && p->started && !p->recovering) p->laps++;
    p->recovering = 0;
    p->state = PATROL_DWELL;
    p->dwell_left = p->route.dwell[p->cur_wp];
    p->move_left = 0;
    p->leg_i = 0;
}

/* Start the next straight segment of the active leg from points[idx]. */
static void patrol_next_segment(Patrol* p, int idx) {
    Pt a = p->leg.pts[idx];
    for (int j = idx + 1; j < p->leg.n; j++) {
        int dx = p->leg.pts[j].x - a.x;
        int dy = p->leg.pts[j].y - a.y;
        int adx = abs(dx), ady = abs(dy);
        int steps = adx > ady ? adx : ady;
        int dur = steps * p->cell_frames;
        if (dur <= 0) continue;             /* duplicate point */
        p->leg_i = j;
        p->seg_a = a;
        p->seg_b = p->leg.pts[j];
        p->seg_frames = dur;
        p->move_left = dur;
        return;
    }
    patrol_arrive(p);
}

static void patrol_begin_leg(Patrol* p) {
    int to = (p->cur_wp + 1) % p->route.n;
    const Path* path = cache_path(p->cache, p->route.wp[p->cur_wp], p->route.wp[to]);
    if (!path) { p->state = PATROL_STOPPED; return; }
    p->leg = *path;
    p->target_wp = to;
    p->recovering = 0;
    if (p->cur_wp == 0) p->started = 1;
    p->state = PATROL_TRAVEL;
    p->leg_i = 0;
    if (p->leg.n <= 1) { patrol_arrive(p); return; }
    patrol_next_segment(p, 0);
}

static void patrol_travel_tick(Patrol* p) {
    if (p->move_left <= 0) { p->state = PATROL_STOPPED; return; }
    p->move_left--;
    if (p->move_left > 0) {
        /* Position is recomputed from integer endpoints and counters every
           frame, never integrated, so repeated laps cannot drift. */
        double u = (double)(p->seg_frames - p->move_left) / (double)p->seg_frames;
        p->x = p->seg_a.x + (p->seg_b.x - p->seg_a.x) * u;
        p->y = p->seg_a.y + (p->seg_b.y - p->seg_a.y) * u;
        return;
    }
    p->cell = p->seg_b;
    p->x = (double)p->seg_b.x;
    p->y = (double)p->seg_b.y;
    if (p->leg_i + 1 >= p->leg.n) { patrol_arrive(p); return; }
    patrol_next_segment(p, p->leg_i);
}

/* One tick == one frame.  Dwell is a whole-frame countdown: a tick that
   spends a frame standing still decrements it, and the tick *after* the
   last stationary frame starts moving.  So "dwell = N" is exactly N frames
   parked and the leg departs on frame N+1, matching the frame counters the
   travel code uses. */
static void patrol_tick(Patrol* p) {
    if (p->state == PATROL_DWELL) {
        if (p->dwell_left > 0) { p->dwell_left--; return; }
        patrol_begin_leg(p);      /* only reached when dwell_left == 0 */
        /* A one-point leg or a failed plan leaves us parked/stopped, not moving. */
        if (p->state == PATROL_TRAVEL) patrol_travel_tick(p);
    } else if (p->state == PATROL_TRAVEL) {
        patrol_travel_tick(p);
    }
}

/* Cell a recovery leg should start from.  Mid-segment the rendered position
   is between two cells, so round it instead of rewinding to the last cell
   reached; fall back to that cell if rounding lands on a wall. */
static Pt patrol_resume_origin(const Patrol* p) {
    Pt c = { (int)lround(p->x), (int)lround(p->y) };
    if (walkable(p->cache->grid, c.x, c.y)) return c;
    return p->cell;
}

/* Head back to the closest reachable waypoint and resume the loop there. */
static int patrol_resume_from_cell(Patrol* p) {
    Pt from = patrol_resume_origin(p);
    p->state = PATROL_STOPPED;
    p->move_left = 0;
    p->cell = from;
    p->x = (double)from.x;
    p->y = (double)from.y;

    int nearest = patrol_nearest_wp(p, from);
    if (nearest < 0) return -1;
    const Path* path = cache_path(p->cache, from, p->route.wp[nearest]);
    if (!path) return -1;

    p->leg = *path;
    p->target_wp = nearest;
    p->recovering = 1;          /* resuming, not completing a lap */
    p->state = PATROL_TRAVEL;
    p->leg_i = 0;
    if (p->leg.n <= 1) { patrol_arrive(p); return 0; }
    patrol_next_segment(p, 0);
    return 0;
}

static int patrol_knockback(Patrol* p, Pt to) {
    if (!walkable(p->cache->grid, to.x, to.y)) return -1;
    p->x = (double)to.x;
    p->y = (double)to.y;
    p->cell = to;
    return patrol_resume_from_cell(p);
}

/* Swap the route without throwing the cache away.  Legs whose endpoints are
   unchanged keep their planned path; the new ones are planned lazily when
   the patrol actually gets to them.  An in-flight leg survives if its
   target waypoint still exists somewhere in the new route. */
static int patrol_set_route(Patrol* p, const Route* r) {
    if (!route_valid(p->cache->grid, r)) return -1;

    Pt target_cell = {0, 0};
    Pt origin = patrol_resume_origin(p);
    int had_target = 0;
    if (p->state == PATROL_TRAVEL) {
        target_cell = p->route.wp[p->target_wp];
        had_target = 1;
    }

    p->route = *r;

    /* A stopped patrol (earlier leg failed or the previous route was
       broken) gets another chance once a valid route is installed. */
    if (p->state == PATROL_STOPPED) {
        p->cell = origin;
        p->x = (double)origin.x;
        p->y = (double)origin.y;
        return patrol_resume_from_cell(p);
    }
    if (p->state == PATROL_DWELL) {
        for (int i = 0; i < r->n; i++) {
            if (p->route.wp[i].x == p->cell.x && p->route.wp[i].y == p->cell.y) {
                p->cur_wp = p->target_wp = i;
                if (p->dwell_left > r->dwell[i]) p->dwell_left = r->dwell[i];
                return 0;
            }
        }
        return patrol_resume_from_cell(p);
    }
    if (p->state == PATROL_TRAVEL && had_target) {
        for (int i = 0; i < r->n; i++) {
            if (p->route.wp[i].x == target_cell.x && p->route.wp[i].y == target_cell.y) {
                p->target_wp = i;
                return 0;   /* same destination, same cached leg */
            }
        }
        return patrol_resume_from_cell(p);
    }
    return 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int run_until_laps(Patrol* p, int laps, int max_ticks) {    for (int i = 0; i < max_ticks && p->laps < laps; i++) patrol_tick(p);
    return p->laps >= laps;
}

static int run_until_wp(Patrol* p, int wp, int max_ticks) {
    for (int i = 0; i < max_ticks; i++) {
        if (p->state == PATROL_DWELL && p->cur_wp == wp) return 1;
        patrol_tick(p);
    }
    return 0;
}

/* Run one full lap at wp0 and record the position after every tick.
   Returns the lap length in ticks, or -1 if it did not complete. */
static int record_lap(Patrol* p, double* xs, double* ys, int cap) {
    int laps0 = p->laps, i = 0;
    while (p->laps == laps0) {
        if (i >= cap) return -1;
        patrol_tick(p);
        xs[i] = p->x;
        ys[i] = p->y;
        i++;
    }
    return i;
}

/* Any point on a smoothed segment is an exact linear blend of the two
   integer endpoints, evaluated with integer frame counters.  Equality
   (not a tolerance) is the actual guarantee we want to assert. */
static int lap_matches(const double* xs, const double* ys, const double* xs2,
                       const double* ys2, int n) {
    for (int i = 0; i < n; i++)
        if (xs[i] != xs2[i] || ys[i] != ys2[i]) return 0;
    return 1;
}

/* Independent hop-count BFS used by the tests as an oracle for the
   walkable-distance ranking that patrol_nearest_wp must reproduce. */
static int oracle_hops(const Grid* g, Pt s, Pt t) {
    int n = g->w * g->h;
    int* d = (int*)malloc(sizeof(int) * (size_t)n);
    int* q = (int*)malloc(sizeof(int) * (size_t)n);
    if (!d || !q) { free(d); free(q); return -1; }
    for (int i = 0; i < n; i++) d[i] = -1;
    int head = 0, tail = 0, ti = t.y * g->w + t.x;
    d[s.y * g->w + s.x] = 0;
    q[tail++] = s.y * g->w + s.x;
    while (head < tail) {
        int cur = q[head++];
        if (cur == ti) break;
        Pt cp = { cur % g->w, cur / g->w };
        for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) {
                if (!dx && !dy) continue;
                Pt np = { cp.x + dx, cp.y + dy };
                if (!step_ok(g, cp, np)) continue;
                int ni = np.y * g->w + np.x;
                if (d[ni] >= 0) continue;
                d[ni] = d[cur] + 1;
                q[tail++] = ni;
            }
    }
    int rc = d[ti];
    free(d); free(q);
    return rc;
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
    Pt raw[MAX_NODES] = {{0,0}}, sm[MAX_NODES] = {{0,0}};
    int n = astar(&g, s, t, raw, MAX_NODES);
    if (n < 0) { printf("no path\n"); return 1; }
    int m = smooth(&g, raw, n, sm, MAX_NODES);
    printf("raw=%d len=%.4f smooth=%d len=%.4f\n", n, path_length(raw,n), m, path_length(sm,m));
    for (int i = 0; i < m; i++) printf("(%d,%d)%s", sm[i].x, sm[i].y, i+1==m ? "\n" : " ");
    int bad = 0;
    for (int i = 1; i < m; i++) if (!line_of_sight(&g, sm[i-1], sm[i])) bad++;
    printf("bad_segments=%d\n", bad);
    printf("corner_cut=%d\n", !step_ok(&g, (Pt){1,0}, (Pt){2,1}));

    /* ---- patrol routes -------------------------------------------- */
    PathCache cache;
    cache_init(&cache, &g);
    Pt wps[] = {{0,0}, {8,0}, {8,4}, {4,4}, {4,2}};
    int dwells[] = {2, 3, 2, 4, 1};
    Route route;
    check("route_reject_empty", route_set(&route, &g, wps, dwells, 0) == -1);
    check("route_set", route_set(&route, &g, wps, dwells, 5) == 0);
    Pt blocked_wp[] = {{0,0}, {5,0}};   /* (5,0) is a wall */
    check("route_reject_blocked", route_set(&route, &g, blocked_wp, dwells, 2) == -1);

    Patrol p;
    patrol_init(&p, &cache, &route, 4);
    check("patrol_start", patrol_start(&p, 0) == 0);

    /* Dwell is frame-exact: 2 frames at wp0, then movement starts. */
    check("dwell_state", p.state == PATROL_DWELL && p.dwell_left == 2);
    patrol_tick(&p);
    check("dwell_frame1", p.state == PATROL_DWELL && p.dwell_left == 1 && p.x == 0.0 && p.y == 0.0);
    patrol_tick(&p);
    check("dwell_frame2", p.state == PATROL_DWELL && p.dwell_left == 0 && p.x == 0.0 && p.y == 0.0);
    patrol_tick(&p);
    check("travel_after_dwell", p.state == PATROL_TRAVEL && p.cell_frames == 4);
    /* Departure tick is frame 1 of its segment, computed from the integer
       endpoints and the integer frame count, not accumulated. */
    Pt seg_a = p.seg_a, seg_b = p.seg_b;
    int seg_frames = p.seg_frames;
    double u1 = 1.0 / (double)seg_frames;
    check("travel_frame1",
          p.x == (double)seg_a.x + (seg_b.x - seg_a.x) * u1 &&
          p.y == (double)seg_a.y + (seg_b.y - seg_a.y) * u1);
    check("travel_mid_leg", p.x >= 0.0 && p.x <= 8.0 && p.y == 0.0);

    /* A one-cell leg at cell_frames=4 takes exactly 4 moving frames. */
    Pt pair[] = {{0,0}, {1,0}};
    int pair_dwell[] = {0, 0};
    Route pair_route;
    check("pair_route", route_set(&pair_route, &g, pair, pair_dwell, 2) == 0);
    patrol_init(&p, &cache, &pair_route, 4);
    patrol_start(&p, 0);
    patrol_tick(&p);   /* dwell 0: this frame departs and advances */
    check("pair_frame1", p.x == 0.25 && p.y == 0.0);
    patrol_tick(&p);
    patrol_tick(&p);
    check("pair_frame3", p.x == 0.75);
    patrol_tick(&p);
    check("pair_frame4_arrives", p.cell.x == 1 && p.cell.y == 0 && p.x == 1.0 && p.y == 0.0);

    /* Legs are cached: one lap should plan only the distinct legs. */
    cache_clear(&cache);
    patrol_init(&p, &cache, &route, 4);
    patrol_start(&p, 0);
    check("one_lap", run_until_laps(&p, 1, 20000));
    check("one_lap_plans", cache.plan_calls == 5);

    /* Re-walking the loop must not replan anything. */
    int plans_after_first = cache.plan_calls;
    check("ten_laps", run_until_laps(&p, 10, 200000));
    check("ten_laps_no_replan", cache.plan_calls == plans_after_first);

    /* Drift: every frame of lap 10 must land on the exact same coordinates
       as the corresponding frame of lap 2. */
    static double xs1[8192], ys1[8192], xs2[8192], ys2[8192];
    patrol_init(&p, &cache, &route, 4);
    patrol_start(&p, 0);
    check("drift_to_lap2", run_until_laps(&p, 1, 200000));
    int lap_len = record_lap(&p, xs1, ys1, 8192);   /* records lap 2 */
    check("drift_lap_recorded", lap_len > 0);
    check("drift_to_lap10", run_until_laps(&p, 9, 200000));
    int lap_len2 = record_lap(&p, xs2, ys2, 8192);  /* records lap 10 */
    check("drift_same_length", lap_len2 == lap_len);
    check("drift_no_accumulation",
          lap_len > 0 && lap_len2 == lap_len &&
          lap_matches(xs1, ys1, xs2, ys2, lap_len));

    cache_clear(&cache);
    patrol_init(&p, &cache, &route, 3);   /* cell_frames that does not divide */
    patrol_start(&p, 0);
    int ok_laps = run_until_laps(&p, 10, 400000);
    check("ten_laps_oddframes", ok_laps);
    check("ten_laps_on_waypoint",
          p.cell.x == 0 && p.cell.y == 0 && p.x == 0.0 && p.y == 0.0 &&
          p.state == PATROL_DWELL);

    /* Knockback: land two cells off, recover to the nearest waypoint by
       path distance, then keep looping without counting a false lap. */
    patrol_init(&p, &cache, &route, 4);
    patrol_start(&p, 0);
    run_until_laps(&p, 2, 200000);
    int laps_before = p.laps;
    int plans_before_recovery = cache.plan_calls;
    check("knockback_ok", patrol_knockback(&p, (Pt){2,2}) == 0);
    check("knockback_recovering", p.recovering == 1 && p.state == PATROL_TRAVEL);
    check("knockback_target_near",
          p.target_wp >= 0 && p.target_wp < route.n);
    check("knockback_arrives", run_until_wp(&p, p.target_wp, 2000));
    check("knockback_no_extra_lap", p.laps == laps_before);
    check("knockback_resumes", run_until_laps(&p, laps_before + 2, 200000));
    /* One plan for the recovery leg, plus at most one new leg when the
       loop resumes from a waypoint the warm cache had not covered yet. */
    check("knockback_replans_bounded", cache.plan_calls - plans_before_recovery <= 2);
    check("knockback_wall_rejected", patrol_knockback(&p, (Pt){5,0}) == -1);

    /* Route edits reuse cached legs and never replan the ones still valid. */
    cache_clear(&cache);
    patrol_init(&p, &cache, &route, 4);
    patrol_start(&p, 0);
    run_until_laps(&p, 1, 200000);       /* warm every leg of the old route */
    check("edit_warmup", run_until_wp(&p, 1, 200000));
    int plans_before_edit = cache.plan_calls;
    Pt wps2[] = {{0,0}, {8,0}, {8,4}, {4,4}, {4,1}};   /* only last point moved */
    int dwells2[] = {2, 3, 2, 4, 1};
    Route route2;
    check("edit_route_ok", route_set(&route2, &g, wps2, dwells2, 5) == 0);
    check("edit_applied", patrol_set_route(&p, &route2) == 0);
    check("edit_same_target", p.target_wp == 1);
    check("edit_no_immediate_replan", cache.plan_calls == plans_before_edit);
    check("edit_continue", run_until_wp(&p, 1, 2000));
    int laps_after_edit = p.laps;
    check("edit_reaches_changed_wp", run_until_laps(&p, laps_after_edit + 1, 200000));
    /* Only the two legs touching the moved waypoint are new: wp3->wp4 and
       the closing wp4->wp0.  Everything else comes from the cache. */
    check("edit_plans_only_new_legs", cache.plan_calls - plans_before_edit == 2);

    /* Nearest-waypoint recovery uses walkable distance, not crow-flight. */
    Pt nearest_probe = {2, 2};
    int nearest_idx = patrol_nearest_wp(&p, nearest_probe);
    int oracle = -1, oracle_d = 0;
    for (int i = 0; i < route2.n; i++) {
        int d = oracle_hops(&g, nearest_probe, route2.wp[i]);
        if (d < 0) continue;
        if (oracle < 0 || d < oracle_d) { oracle = i; oracle_d = d; }
    }
    check("nearest_matches_shortest", nearest_idx == oracle);

    /* A waypoint that is close as the crow flies but sits behind a wall
       must lose to a farther one that is actually easier to walk to. */
    const char* detour_rows[] = {
        ".#...",
        ".#...",
        ".....",
        ".....",
    };
    Grid gd;
    check("detour_grid_load", grid_load(&gd, detour_rows, 4) == 0);
    PathCache cached;
    cache_init(&cached, &gd);
    Pt detour_wps[] = {{2,0}, {3,3}};
    int detour_dwell[] = {0, 0};
    Route rd;
    check("detour_route", route_set(&rd, &gd, detour_wps, detour_dwell, 2) == 0);
    Patrol pd;
    patrol_init(&pd, &cached, &rd, 1);
    patrol_start(&pd, 0);
    check("detour_oracle_agrees",
          oracle_hops(&gd, (Pt){0,0}, detour_wps[0]) >
          oracle_hops(&gd, (Pt){0,0}, detour_wps[1]));
    check("detour_uses_path_distance",
          patrol_nearest_wp(&pd, (Pt){0,0}) == 1);

    /* A blocked route parks the patrol; installing a good one revives it. */
    const char* gap_rows[] = {
        "..#.",
        "#.#.",
        "..#.",
    };
    Grid gw;
    check("wall_grid_load", grid_load(&gw, gap_rows, 3) == 0);
    PathCache cachew;
    cache_init(&cachew, &gw);
    Pt unreachable[] = {{0,0}, {3,1}};   /* right-hand region is sealed off */
    int dw[] = {0, 0};
    Route rw;
    check("unreachable_route", route_set(&rw, &gw, unreachable, dw, 2) == 0);
    check("unreachable_oracle", oracle_hops(&gw, (Pt){0,0}, (Pt){3,1}) < 0);
    Patrol pw;
    patrol_init(&pw, &cachew, &rw, 1);
    patrol_start(&pw, 0);
    for (int i = 0; i < 100; i++) patrol_tick(&pw);
    check("unreachable_halts", pw.state == PATROL_STOPPED);

    Pt reachable[] = {{0,0}, {1,0}, {1,2}};
    int dw2[] = {0, 0, 0};
    Route rw2;
    check("recover_route_set", route_set(&rw2, &gw, reachable, dw2, 3) == 0);
    check("recover_set_route", patrol_set_route(&pw, &rw2) == 0);
    check("recover_resumes", run_until_laps(&pw, 1, 5000));
    check("recover_cell_valid",
          walkable(&gw, pw.cell.x, pw.cell.y) && pw.cell.x >= 0 && pw.cell.y >= 0);

    /* Mid-segment route edits resume from the nearest rounded cell rather
       than rewinding to the cell last touched. */
    cache_clear(&cache);
    patrol_init(&p, &cache, &route, 4);
    patrol_start(&p, 0);
    while (p.cell.x < 3) patrol_tick(&p);   /* well into the first leg */
    Pt wps3[] = {{8,4}, {0,0}};             /* last leg's destination is gone */
    int dwells3[] = {0, 0};
    Route route3;
    check("midleg_route_set", route_set(&route3, &g, wps3, dwells3, 2) == 0);
    check("midleg_edit", patrol_set_route(&p, &route3) == 0);
    check("midleg_near_position",
          abs(p.cell.x - (int)lround(p.x)) <= 1 && abs(p.cell.y - (int)lround(p.y)) <= 1);
    check("midleg_continues", run_until_laps(&p, 2, 5000));

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
