
/* mini-navgrid: 8-direction A* on an ASCII grid, with corner-cut prevention,
   path smoothing by line-of-sight, incremental re-planning, and a formation
   movement experiment.

   Formation model:
   - one clearance-aware route is planned for the whole group (the group
     detours together instead of every unit finding its own way);
   - members hold fixed offsets around the route anchor;
   - when the formation cannot fit (a narrow mouth), slots collapse to a
     single-file queue along the route, and the transition is logged;
   - a member more than LAG_GAP cells from its slot makes the anchor wait and
     may take a catch-up step, so stragglers rejoin;
   - the independent baseline plans one plain A* per unit for cost comparison. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_W 64
#define MAX_H 64
#define MAX_NODES (MAX_W * MAX_H)

#define MAX_UNITS 12
#define MAX_EVENTS 64
#define MAX_TICKS 4096
#define LAG_GAP 3          /* chebyshev distance that counts as falling behind */

typedef struct { int w, h; unsigned char blocked[MAX_H][MAX_W]; } Grid;

typedef struct { int x, y; } Pt;

static int idx(int x, int y) { return y * MAX_W + x; }
static int pt_eq(Pt a, Pt b) { return a.x == b.x && a.y == b.y; }
static int pt_cheb(Pt a, Pt b) {
    int dx = abs(a.x - b.x), dy = abs(a.y - b.y);
    return dx > dy ? dx : dy;
}
static double pt_dist2(Pt a, Pt b) {
    double dx = (double)(a.x - b.x), dy = (double)(a.y - b.y);
    return dx * dx + dy * dy;
}

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

/* mask is an optional per-cell extra wall layer (used for formation clearance) */
static int cell_blocked(const Grid* g, const unsigned char* mask, int x, int y) {
    if (!walkable(g, x, y)) return 1;
    if (mask && mask[idx(x, y)]) return 1;
    return 0;
}

/* diagonal step p->q is legal only if both orthogonal neighbours are free */
static int step_ok(const Grid* g, const unsigned char* mask, Pt p, Pt q) {
    int dx = q.x - p.x, dy = q.y - p.y;
    if (cell_blocked(g, mask, q.x, q.y)) return 0;
    if (dx != 0 && dy != 0) {
        if (cell_blocked(g, mask, p.y, q.x) || cell_blocked(g, mask, q.y, p.x)) return 0;
    }
    return 1;
}

static const int DIRS[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

typedef struct { double g, f; int parent; unsigned char open, closed; } Node;

static int astar(const Grid* g, const unsigned char* mask, Pt s, Pt t, Pt* out, int max_out) {
    if (cell_blocked(g, mask, s.x, s.y) || cell_blocked(g, mask, t.x, t.y)) return -1;
    Node* nodes = (Node*)calloc((size_t)g->w * g->h, sizeof(Node));
    if (!nodes) return -1;
    int* heap = (int*)malloc(sizeof(int) * (size_t)g->w * g->h);
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
            if (!step_ok(g, mask, cp, np)) continue;
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

static int line_of_sight(const Grid* g, const unsigned char* mask, Pt a, Pt b) {
    int steps = abs(b.x - a.x) + abs(b.y - a.y);
    if (steps == 0) return 1;
    Pt prev = a;
    for (int i = 1; i <= steps; i++) {
        double t = (double)i / steps;
        Pt cur;
        cur.x = (int)lround(a.x + (b.x - a.x) * t);
        cur.y = (int)lround(a.y + (b.y - a.y) * t);
        if (cur.x == prev.x && cur.y == prev.y) continue;
        if (!step_ok(g, mask, prev, cur)) return 0;
        prev = cur;
    }
    return prev.x == b.x && prev.y == b.y;
}

static int smooth(const Grid* g, const unsigned char* mask, const Pt* in, int n,
                  Pt* out, int max_out) {
    if (n <= 0) return 0;
    int m = 0;
    out[m++] = in[0];
    int anchor = 0;
    while (anchor < n - 1) {
        int next = anchor + 1;
        for (int j = n - 1; j > anchor; j--) {
            if (line_of_sight(g, mask, in[anchor], in[j])) { next = j; break; }
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

/* ---------------- formation ---------------- */

typedef struct { Pt off[MAX_UNITS]; int n; } Formation;

typedef enum { MODE_FORMATION = 0, MODE_FILE = 1 } SlotMode;

typedef enum { EV_COMPRESS = 1, EV_EXPAND = 2, EV_REGROUP = 3 } EventKind;

typedef struct {
    int kind, tick, unit, x, y, aux;
} Event;

typedef struct {
    int ticks, steps, waits, compress, expand, regroup, replans;
    int arrived, wide_route, route_n, pinch_uses;
    double rel_err, route_len, spread;
} SimResult;

/* 2 rows x 3 columns, front edge toward +x; unit 0 is the route anchor */
static void formation_init(Formation* f) {
    static const Pt o[] = {{-1,-1},{0,-1},{1,-1},{-1,0},{0,0},{1,0}};
    f->n = (int)(sizeof(o) / sizeof(o[0]));
    for (int i = 0; i < f->n; i++) f->off[i] = o[i];
}

/* single-file order: +x first, then -y, so the column keeps its leading edge */
static int unit_rank(const Formation* f, int i) {
    int r = 0;
    for (int j = 0; j < f->n; j++) {
        if (j == i) continue;
        if (f->off[j].x > f->off[i].x) r++;
        else if (f->off[j].x == f->off[i].x && f->off[j].y < f->off[i].y) r++;
    }
    return r;
}

/* wall cells that no formation anchor may occupy: any member's offset would clip */
static void build_clearance(const Grid* g, const Formation* f, unsigned char* mask) {
    memset(mask, 0, (size_t)MAX_H * MAX_W);
    for (int y = 0; y < g->h; y++)
        for (int x = 0; x < g->w; x++) {
            if (!walkable(g, x, y)) continue;
            for (int i = 0; i < f->n; i++) {
                if (!walkable(g, x + f->off[i].x, y + f->off[i].y)) {
                    mask[idx(x, y)] = 1;
                    break;
                }
            }
        }
}

/* slots for the formation mode at a given route anchor cell */
static int slots_formation(const Grid* g, const Formation* f, Pt anchor, Pt* out) {
    for (int i = 0; i < f->n; i++) {
        out[i].x = anchor.x + f->off[i].x;
        out[i].y = anchor.y + f->off[i].y;
        if (!walkable(g, out[i].x, out[i].y)) return 0;
    }
    return 1;
}

/* single-file slots: rank r sits r cells behind the anchor along the route */
static int slots_file(const Formation* f, const Pt* route, int route_n, int ai, Pt* out) {
    (void)route_n;
    for (int i = 0; i < f->n; i++) {
        int back = ai - unit_rank(f, i);
        if (back < 0) return 0;
        out[i] = route[back];
    }
    return 1;
}

static int slots_build(const Grid* g, const Formation* f, const Pt* route, int route_n,
                       int ai, SlotMode mode, Pt* out) {
    if (mode == MODE_FORMATION) return slots_formation(g, f, route[ai], out);
    return slots_file(f, route, route_n, ai, out);
}

/* last route cell where the open formation cannot be placed at all */
static int last_blocked_anchor(const Grid* g, const Formation* f, const Pt* route, int route_n) {
    Pt slots[MAX_UNITS];
    int last = -1;
    for (int p = 0; p < route_n; p++) {
        if (!slots_build(g, f, route, route_n, p, MODE_FORMATION, slots)) last = p;
    }
    return last;
}

/* is the open formation placeable for every anchor in the n-cell lookahead? */
static int formation_fits_ahead(const Grid* g, const Formation* f, const Pt* route,
                                int route_n, int ai) {
    Pt slots[MAX_UNITS];
    int last = ai + f->n - 1;
    if (last >= route_n) last = route_n - 1;
    for (int p = ai; p <= last; p++)
        if (!slots_build(g, f, route, route_n, p, MODE_FORMATION, slots)) return 0;
    return 1;
}

/* Compress as soon as the open formation no longer fits the next n anchors
   (the mouth is coming). Once compressed, stay in file order until the rear
   rank has cleared the pinch: expanding earlier lets the front rank's offsets
   re-block the mouth and strands everyone behind. */
static SlotMode mode_at(const Grid* g, const Formation* f, const Pt* route, int route_n,
                        int ai, int last_blocked, SlotMode current) {
    Pt slots[MAX_UNITS];
    if (current == MODE_FILE) {
        int cleared = ai > last_blocked + f->n - 1;
        if (!cleared || !formation_fits_ahead(g, f, route, route_n, ai)) return MODE_FILE;
        return MODE_FORMATION;
    }
    if (formation_fits_ahead(g, f, route, route_n, ai)) return MODE_FORMATION;
    if (!slots_build(g, f, route, route_n, ai, MODE_FILE, slots)) return MODE_FORMATION;
    return MODE_FILE;
}

static void log_event(Event* ev, int* n, int kind, int tick, int unit, Pt p, int aux) {
    if (*n >= MAX_EVENTS) return;
    ev[*n].kind = kind; ev[*n].tick = tick; ev[*n].unit = unit;
    ev[*n].x = p.x; ev[*n].y = p.y; ev[*n].aux = aux;
    (*n)++;
}

/* move one unit one cell toward its slot, strictly decreasing (chebyshev, dist2).
   Diagonal moves also require both orthogonal cells to be free of other units,
   so members never slip through each other and break the formation. */
static int greedy_step(const Grid* g, unsigned char* occ, Pt* pos, Pt slot) {
    Pt best = *pos;
    int best_cheb = pt_cheb(*pos, slot);
    double best_d2 = pt_dist2(*pos, slot);
    int moved = 0;
    occ[idx(pos->x, pos->y)] = 0;
    for (int d = 0; d < 8; d++) {
        Pt q = { pos->x + DIRS[d][0], pos->y + DIRS[d][1] };
        if (!walkable(g, q.x, q.y) || occ[idx(q.x, q.y)]) continue;
        if (!step_ok(g, NULL, *pos, q)) continue;
        if (q.x != pos->x && q.y != pos->y) {
            if (occ[idx(q.x, pos->y)] || occ[idx(pos->x, q.y)]) continue;
        }
        int c = pt_cheb(q, slot);
        double d2 = pt_dist2(q, slot);
        if (c < best_cheb || (c == best_cheb && d2 < best_d2)) {
            best = q; best_cheb = c; best_d2 = d2; moved = 1;
        }
    }
    if (moved) {
        *pos = best;
        occ[idx(pos->x, pos->y)] = 1;
    } else {
        occ[idx(pos->x, pos->y)] = 1;
    }
    return moved;
}

/* A straggler cannot always close in a straight line: its slot may be behind
   other members. When the greedy step stalls, plan around the occupied cells
   and take the first step of that path. */
static int regroup_step(const Grid* g, unsigned char* occ, Pt* pos, Pt slot) {
    Pt path[MAX_NODES];
    int own = idx(pos->x, pos->y);
    occ[own] = 0;
    int n = astar(g, occ, *pos, slot, path, MAX_NODES);
    occ[own] = 1;
    if (n < 2) return 0;
    Pt q = path[1];
    if (!walkable(g, q.x, q.y) || occ[idx(q.x, q.y)]) return 0;
    occ[own] = 0;
    *pos = q;
    occ[idx(q.x, q.y)] = 1;
    return 1;
}

/* spawn_override, when non-NULL, replaces the ideal start slot per unit (used to
   inject a unit that is already out of place); pinch/has_pinch lets the caller
   ask whether the chosen route crosses a given cell. */
static void simulate_formation(const Grid* g, const Formation* f, Pt start, Pt goal,
                               const Pt* spawn_override, Pt pinch, int has_pinch,
                               SimResult* res) {
    memset(res, 0, sizeof(*res));
    res->arrived = 0;
    Pt route[MAX_NODES];
    int rn = -1;

    unsigned char mask[MAX_H * MAX_W];
    build_clearance(g, f, mask);
    rn = astar(g, mask, start, goal, route, MAX_NODES);
    if (rn > 0) res->wide_route = 1;
    if (rn < 0) {
        rn = astar(g, NULL, start, goal, route, MAX_NODES);
        res->replans++;          /* formation route failed, fell back to a trail */
    }
    if (rn < 0) return;
    res->route_n = rn;
    res->route_len = 0.0;
    for (int i = 1; i < rn; i++) {
        double dx = (double)(route[i].x - route[i-1].x);
        double dy = (double)(route[i].y - route[i-1].y);
        res->route_len += sqrt(dx * dx + dy * dy);
    }
    if (has_pinch) {
        for (int i = 0; i < rn; i++) if (pt_eq(route[i], pinch)) res->pinch_uses++;
    }

    Pt pos[MAX_UNITS], slots[MAX_UNITS], final[MAX_UNITS];
    unsigned char occ[MAX_H * MAX_W];
    memset(occ, 0, sizeof(occ));
    for (int i = 0; i < f->n; i++) {
        if (spawn_override) {
            pos[i] = spawn_override[i];
        } else {
            pos[i].x = start.x + f->off[i].x;
            pos[i].y = start.y + f->off[i].y;
        }
        if (!walkable(g, pos[i].x, pos[i].y) || occ[idx(pos[i].x, pos[i].y)]) return;
        occ[idx(pos[i].x, pos[i].y)] = 1;
    }

    Event ev[MAX_EVENTS];
    int ne = 0;
    int lag[MAX_UNITS] = {0};
    int last_blocked = last_blocked_anchor(g, f, route, rn);
    int ai = 0;
    SlotMode mode = MODE_FORMATION;
    int tick = 0;

    int idle_run = 0;
    for (; tick < MAX_TICKS; tick++) {
        SlotMode nm = mode_at(g, f, route, rn, ai, last_blocked, mode);
        if (nm != mode) {
            if (nm == MODE_FILE) {
                res->compress++;
                log_event(ev, &ne, EV_COMPRESS, tick, -1, route[ai], 0);
            } else {
                res->expand++;
                log_event(ev, &ne, EV_EXPAND, tick, -1, route[ai], 0);
            }
            mode = nm;
        }
        if (!slots_build(g, f, route, rn, ai, mode, slots)) {
            /* current layout no longer fits here: try the other one before
               giving up (happens when the start sits right at a pinch) */
            SlotMode alt = (mode == MODE_FORMATION) ? MODE_FILE : MODE_FORMATION;
            if (!slots_build(g, f, route, rn, ai, alt, slots)) break;
            if (alt == MODE_FILE) {
                res->compress++;
                log_event(ev, &ne, EV_COMPRESS, tick, -1, route[ai], 0);
            } else {
                res->expand++;
                log_event(ev, &ne, EV_EXPAND, tick, -1, route[ai], 0);
            }
            mode = alt;
        }

        int straggler = 0;
        for (int i = 0; i < f->n; i++) {
            int d = pt_cheb(pos[i], slots[i]);
            if (d > LAG_GAP && !lag[i]) {
                lag[i] = 1;
                res->regroup++;
                log_event(ev, &ne, EV_REGROUP, tick, i, pos[i], d);
            } else if (d <= 1) {
                lag[i] = 0;
            }
            if (d > LAG_GAP) straggler = 1;
        }

        int all_on_slot = 1;
        for (int i = 0; i < f->n; i++) if (!pt_eq(pos[i], slots[i])) all_on_slot = 0;
        if (ai == rn - 1 && all_on_slot) break;

        /* the anchor is a virtual front marker: it only waits for stragglers */
        if (ai < rn - 1) {
            if (straggler) res->waits++;
            else ai++;
        }
        if (!slots_build(g, f, route, rn, ai, mode, slots)) {
            SlotMode alt = (mode == MODE_FORMATION) ? MODE_FILE : MODE_FORMATION;
            if (!slots_build(g, f, route, rn, ai, alt, slots)) break;
            if (alt == MODE_FILE) {
                res->compress++;
                log_event(ev, &ne, EV_COMPRESS, tick, -1, route[ai], 0);
            } else {
                res->expand++;
                log_event(ev, &ne, EV_EXPAND, tick, -1, route[ai], 0);
            }
            mode = alt;
        }

        /* lagging members move first and may take a catch-up step */
        int steps_before = res->steps;
        int order[MAX_UNITS];
        for (int i = 0; i < f->n; i++) order[i] = i;
        for (int i = 0; i < f->n; i++)
            for (int j = i + 1; j < f->n; j++)
                if (pt_cheb(pos[order[j]], slots[order[j]]) >
                    pt_cheb(pos[order[i]], slots[order[i]])) {
                    int t = order[i]; order[i] = order[j]; order[j] = t;
                }
        for (int k = 0; k < f->n; k++) {
            int i = order[k];
            int lagging = pt_cheb(pos[i], slots[i]) > LAG_GAP;
            int moves = lagging ? 2 : 1;
            for (int m = 0; m < moves; m++) {
                if (pt_eq(pos[i], slots[i])) break;
                int moved = greedy_step(g, occ, &pos[i], slots[i]);
                if (!moved && lagging) moved = regroup_step(g, occ, &pos[i], slots[i]);
                if (!moved) break;
                res->steps++;
            }
        }

        /* watchdog: the anchor is at the end and nobody could move, so the
           group is wedged; stop instead of spinning to MAX_TICKS */
        if (ai == rn - 1 && res->steps == steps_before) {
            if (++idle_run > (int)f->n) break;
        } else {
            idle_run = 0;
        }
    }

    /* arrival contract: every member within one cell of its ideal slot.
       Prefer open-formation slots; fall back to the column if the goal itself
       is still a pinch (the goal area was too tight to expand). */
    int have_final = 0;
    if (ai == rn - 1) {
        if (slots_formation(g, f, route[rn-1], final)) have_final = 1;
        else if (slots_file(f, route, rn, rn - 1, final)) have_final = 1;
    }
    if (have_final) {
        double err = 0.0;
        for (int i = 0; i < f->n; i++) {
            double e = sqrt(pt_dist2(pos[i], final[i]));
            if (e > err) err = e;
        }
        res->rel_err = err;
        res->arrived = err < 1.0;
    }
    /* max pairwise chebyshev spread among members at the end */
    res->spread = 0;
    for (int i = 0; i < f->n; i++)
        for (int j = i + 1; j < f->n; j++) {
            int d = pt_cheb(pos[i], pos[j]);
            if (d > res->spread) res->spread = d;
        }
    res->ticks = tick;

    printf("  events:");
    if (ne == 0) printf(" none");
    for (int i = 0; i < ne; i++) {
        if (ev[i].kind == EV_COMPRESS)
            printf(" compress@t%d(%d,%d)", ev[i].tick, ev[i].x, ev[i].y);
        else if (ev[i].kind == EV_EXPAND)
            printf(" expand@t%d(%d,%d)", ev[i].tick, ev[i].x, ev[i].y);
        else
            printf(" regroup@t%d(unit=%d dist=%d)", ev[i].tick, ev[i].unit, ev[i].aux);
    }
    printf("\n");
}

/* independent baseline: every unit plans its own plain A* path.
   pinch is an optional cell whose crossing marks "used the shortcut";
   split counts how many units disagreed with the majority route choice. */
static void plan_independent(const Grid* g, const Formation* f, Pt start, Pt goal,
                             Pt pinch, int has_pinch, SimResult* res) {
    memset(res, 0, sizeof(*res));
    res->arrived = 1;
    res->route_len = 0.0;
    int via_pinch = 0;
    for (int i = 0; i < f->n; i++) {
        Pt s = { start.x + f->off[i].x, start.y + f->off[i].y };
        Pt t = { goal.x + f->off[i].x, goal.y + f->off[i].y };
        Pt p[MAX_NODES];
        int n = astar(g, NULL, s, t, p, MAX_NODES);
        if (n < 0) { res->arrived = 0; continue; }
        res->steps += n - 1;
        if (n - 1 > res->ticks) res->ticks = n - 1;
        double len = 0.0;
        for (int k = 1; k < n; k++) {
            double dx = (double)(p[k].x - p[k-1].x), dy = (double)(p[k].y - p[k-1].y);
            len += sqrt(dx * dx + dy * dy);
        }
        if (len > res->route_len) res->route_len = len;
        int used = 0;
        for (int k = 0; k < n; k++) if (pt_eq(p[k], pinch)) used = 1;
        via_pinch += used;
    }
    if (has_pinch) res->pinch_uses = via_pinch;
}

/* ---------------- tests ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void report(const char* tag, const SimResult* f, const SimResult* b) {
    double pct = b->steps > 0 ? 100.0 * (double)(f->steps - b->steps) / (double)b->steps : 0.0;
    printf("%s formation: ticks=%d steps=%d max_spread=%.0f rel_err=%.3f "
           "compress=%d expand=%d regroup=%d waits=%d replans=%d route=%.2f"
           "%s\n",
           tag, f->ticks, f->steps, f->spread, f->rel_err, f->compress, f->expand,
           f->regroup, f->waits, f->replans, f->route_len,
           f->arrived ? "" : " (not arrived)");
    printf("%s independent: ticks=%d steps=%d route=%.2f pinch_uses=%d\n",
           tag, b->ticks, b->steps, b->route_len, b->pinch_uses);
    printf("%s step_delta=%+d (%+.1f%%)\n", tag, f->steps - b->steps, pct);
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
    int n = astar(&g, NULL, s, t, raw, MAX_NODES);
    if (n < 0) { printf("no path\n"); return 1; }
    int m = smooth(&g, NULL, raw, n, sm, MAX_NODES);
    printf("raw=%d len=%.4f smooth=%d len=%.4f\n", n, path_length(raw,n), m, path_length(sm,m));
    for (int i = 0; i < m; i++) printf("(%d,%d)%s", sm[i].x, sm[i].y, i+1==m ? "\n" : " ");
    int bad = 0;
    for (int i = 1; i < m; i++) if (!line_of_sight(&g, NULL, sm[i-1], sm[i])) bad++;
    check("bad_segments", bad == 0);
    check("corner_cut", !step_ok(&g, NULL, (Pt){1,0}, (Pt){2,1}));

    /* two 22x10 fields sharing a wall at x=10.
       detour: single-cell shortcut at y=4, but a 2-row-wide formation can
               still cross at y=7..9 (rows 6..9 are open on both sides).
       narrow: only the 1-wide mouth at y=4; no 2-row footprint fits anywhere. */
    static const char* narrow_rows[] = {
        "......................",
        "..........#...........",
        "..........#...........",
        "..........#...........",
        "......................",
        "..........#...........",
        "..........#...........",
        "..........#...........",
        "..........#...........",
        "......................",
    };
    /* narrow shortcut on row 4; rows 6..9 stay open for the group detour */
    static const char* detour_rows[] = {
        "......................",
        "..........#...........",
        "..........#...........",
        "..........#...........",
        "......................",
        "..........#...........",
        "......................",
        "......................",
        "......................",
        "......................",
    };
    Formation f;
    formation_init(&f);
    Pt start = {2, 4}, goal = {19, 4};

    Grid gd, gn;
    if (grid_load(&gd, detour_rows, 10) != 0) { printf("load fail\n"); return 1; }
    if (grid_load(&gn, narrow_rows, 10) != 0) { printf("load fail\n"); return 1; }

    SimResult fd, bd, fn, bn;
    printf("detour (narrow shortcut y=4, group can still cross at y=6..9):\n");
    simulate_formation(&gd, &f, start, goal, NULL, (Pt){10,4}, 1, &fd);
    plan_independent(&gd, &f, start, goal, (Pt){10,4}, 1, &bd);
    report("detour", &fd, &bd);

    printf("narrow (only a 1-wide mouth at y=4):\n");
    simulate_formation(&gn, &f, start, goal, NULL, (Pt){10,4}, 1, &fn);
    plan_independent(&gn, &f, start, goal, (Pt){10,4}, 1, &bn);
    report("narrow", &fn, &bn);

    /* lag scene: open field, one member spawned 4 cells behind its slot */
    static const char* open_rows[] = {
        "......................",
        "......................",
        "......................",
        "......................",
        "......................",
        "......................",
        "......................",
        "......................",
        "......................",
        "......................",
    };
    Grid go;
    if (grid_load(&go, open_rows, 10) != 0) { printf("load fail\n"); return 1; }
    Pt s2 = {6, 5}, t2 = {19, 5};
    Pt spawn[MAX_UNITS];
    for (int i = 0; i < f.n; i++) {
        spawn[i].x = s2.x + f.off[i].x;
        spawn[i].y = s2.y + f.off[i].y;
    }
    spawn[5].x -= 4;              /* unit 5 starts four cells behind */
    SimResult fl, bl;
    printf("lag (unit 5 spawns 4 cells behind its slot):\n");
    simulate_formation(&go, &f, s2, t2, spawn, (Pt){-1,-1}, 0, &fl);
    plan_independent(&go, &f, s2, t2, (Pt){-1,-1}, 0, &bl);
    report("lag", &fl, &bl);

    int total_f = fd.steps + fn.steps + fl.steps, total_b = bd.steps + bn.steps + bl.steps;
    printf("total (3 scenes) formation_steps=%d independent_steps=%d delta=%+d (%+.1f%%)\n",
           total_f, total_b, total_f - total_b,
           total_b > 0 ? 100.0 * (double)(total_f - total_b) / (double)total_b : 0.0);

    check("detour_group_rerouted", fd.wide_route == 1);
    check("detour_no_compaction", fd.compress == 0);
    check("detour_arrived", fd.arrived && fd.rel_err < 1.0);
    check("detour_costs_more_than_solo", fd.steps > bd.steps);
    /* solo units all squeeze the one-cell mouth one at a time; the group detours */
    check("detour_group_avoids_pinch", fd.wide_route == 1 && fd.pinch_uses == 0);
    check("detour_solo_all_squeeze", bd.pinch_uses == f.n);
    check("narrow_takes_direct_route", fn.wide_route == 0 && fn.pinch_uses > 0);
    check("narrow_compacted", fn.compress >= 1);
    check("narrow_expanded", fn.expand >= 1);
    check("narrow_arrived", fn.arrived && fn.rel_err < 1.0);
    check("narrow_stayed_cohesive", fn.waits > 0);
    check("narrow_solo_squeeze", bn.pinch_uses == f.n);
    check("narrow_rejoined", fn.spread <= (int)(2 * LAG_GAP));
    check("independent_baseline", bd.arrived && bn.arrived && bd.steps > 0 && bn.steps > 0);
    check("lag_detected", fl.regroup >= 1);
    check("lag_leader_waited", fl.waits > 0);
    check("lag_reformed", fl.arrived && fl.rel_err < 1.0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
