
/* tilemap: layered tile map with 4-neighbour autotiling, per-layer culling,
   and priority-resolved rectangular trigger zones. */
#include <stdio.h>
#include <string.h>

#define W 32
#define H 32
#define LAYERS 3

typedef struct {
    unsigned char base[LAYERS][H][W];   /* 0 = empty, else tile id */
    int layer_visible[LAYERS];
} TileMap;

static void map_init(TileMap* m) {
    memset(m, 0, sizeof(*m));
    for (int l = 0; l < LAYERS; l++) m->layer_visible[l] = 1;
}

static void map_set(TileMap* m, int layer, int x, int y, unsigned char id) {
    if (layer < 0 || layer >= LAYERS || x < 0 || x >= W || y < 0 || y >= H) return;
    m->base[layer][y][x] = id;
}

static unsigned char map_get(const TileMap* m, int layer, int x, int y) {
    if (layer < 0 || layer >= LAYERS || x < 0 || x >= W || y < 0 || y >= H) return 0;
    return m->base[layer][y][x];
}

/* 4-bit mask: 1 = neighbour filled (N,E,S,W) */
static int autotile_mask(const TileMap* m, int layer, int x, int y) {
    int mask = 0;
    if (map_get(m, layer, x, y - 1)) mask |= 1;
    if (map_get(m, layer, x + 1, y)) mask |= 2;
    if (map_get(m, layer, x, y + 1)) mask |= 4;
    if (map_get(m, layer, x - 1, y)) mask |= 8;
    return mask;
}

/* autotile variant id for a filled cell: mask plus a base offset */
static int autotile_variant(const TileMap* m, int layer, int x, int y) {
    if (!map_get(m, layer, x, y)) return -1;
    return autotile_mask(m, layer, x, y);
}

/* count visible tiles inside a rect across visible layers */
static int cull_count(const TileMap* m, int x0, int y0, int x1, int y1) {
    int n = 0;
    for (int l = 0; l < LAYERS; l++) {
        if (!m->layer_visible[l]) continue;
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++)
                if (map_get(m, l, x, y)) n++;
    }
    return n;
}

/* ---- trigger zones ----
 * Rectangular zones firing enter/exit events as an entity crosses their
 * effective boundary. Effective region of a zone = its rect minus the
 * rects of all higher-priority zones: the highest-priority zone containing
 * the entity owns it and is the only one that can fire, so overlapping
 * regions never double-trigger and every boundary crossing produces
 * exactly one exit/enter event pair. TRIG_ONCE zones fire their first
 * enter+exit pair, then stay inert. trig_place() is the spawn point and
 * fires an enter if it lands inside a zone; trig_move() re-evaluates
 * ownership per call, so teleporting clear across a zone fires nothing.
 * A config is unsolvable when two overlapping zones share a priority (no
 * deterministic owner) or a rect is malformed; trig_validate() counts
 * those problems, 0 = solvable.
 */
#define MAXZ 16
#define MAXEV 512

enum { TRIG_ONCE = 0, TRIG_REPEAT = 1 };
enum { EV_ENTER = 0, EV_EXIT = 1 };

typedef struct {
    int x0, y0, x1, y1;   /* inclusive cell rect */
    int priority;         /* higher wins on overlap */
    int mode;             /* TRIG_ONCE or TRIG_REPEAT */
    int fired;            /* TRIG_ONCE: enter already emitted */
    int active;           /* current stay emitted an enter, exit still owed */
} TriggerZone;

typedef struct {
    TriggerZone z[MAXZ];
    int n;
    int owner;            /* zone currently owning the entity, -1 = none */
    int px, py;           /* entity cell */
    int ev_type[MAXEV];
    int ev_zone[MAXEV];
    int ev_n;
} TriggerSys;

static void trig_init(TriggerSys* s) {
    memset(s, 0, sizeof(*s));
    s->owner = -1;
}

static int trig_add(TriggerSys* s, int x0, int y0, int x1, int y1, int priority, int mode) {
    if (s->n >= MAXZ) return -1;
    TriggerZone* z = &s->z[s->n];
    z->x0 = x0; z->y0 = y0; z->x1 = x1; z->y1 = y1;
    z->priority = priority; z->mode = mode;
    z->fired = 0; z->active = 0;
    return s->n++;
}

static int rect_contains(const TriggerZone* z, int x, int y) {
    return x >= z->x0 && x <= z->x1 && y >= z->y0 && y <= z->y1;
}

static int rects_overlap(const TriggerZone* a, const TriggerZone* b) {
    return a->x0 <= b->x1 && b->x0 <= a->x1 && a->y0 <= b->y1 && b->y0 <= a->y1;
}

/* 0 = solvable; else number of problems (equal-priority overlaps, bad rects) */
static int trig_validate(const TriggerSys* s) {
    int bad = 0;
    for (int i = 0; i < s->n; i++) {
        if (s->z[i].x1 < s->z[i].x0 || s->z[i].y1 < s->z[i].y0) bad++;
        for (int j = i + 1; j < s->n; j++)
            if (s->z[i].priority == s->z[j].priority &&
                rects_overlap(&s->z[i], &s->z[j]))
                bad++;
    }
    return bad;
}

/* highest-priority zone containing (x,y), -1 if none; lower id wins ties */
static int trig_owner_at(const TriggerSys* s, int x, int y) {
    int best = -1;
    for (int i = 0; i < s->n; i++)
        if (rect_contains(&s->z[i], x, y) &&
            (best < 0 || s->z[i].priority > s->z[best].priority))
            best = i;
    return best;
}

static void trig_emit(TriggerSys* s, int type, int zone) {
    if (s->ev_n < MAXEV) {
        s->ev_type[s->ev_n] = type;
        s->ev_zone[s->ev_n] = zone;
        s->ev_n++;
    }
}

/* move the entity to a cell, firing exit/enter on ownership change */
static void trig_move(TriggerSys* s, int x, int y) {
    int next = trig_owner_at(s, x, y);
    if (next != s->owner) {
        if (s->owner >= 0) {
            TriggerZone* o = &s->z[s->owner];
            if (o->active) { trig_emit(s, EV_EXIT, s->owner); o->active = 0; }
        }
        if (next >= 0) {
            TriggerZone* z = &s->z[next];
            if (z->mode == TRIG_REPEAT || !z->fired) {
                trig_emit(s, EV_ENTER, next);
                z->fired = 1;
                z->active = 1;
            }
        }
        s->owner = next;
    }
    s->px = x; s->py = y;
}

/* spawn: same as a move in from outside every zone */
static void trig_place(TriggerSys* s, int x, int y) {
    s->owner = -1;
    trig_move(s, x, y);
}

static int trig_count(const TriggerSys* s, int zone, int type) {
    int n = 0;
    for (int i = 0; i < s->ev_n; i++)
        if (s->ev_zone[i] == zone && s->ev_type[i] == type) n++;
    return n;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    TileMap m;
    map_init(&m);
    for (int x = 5; x < 10; x++) map_set(&m, 0, x, 5, 1);
    map_set(&m, 1, 5, 5, 2);

    check("single_mask", autotile_mask(&m, 0, 5, 5) == 2);          /* only E */
    check("middle_mask", autotile_mask(&m, 0, 7, 5) == 2 | 8);      /* E and W */
    check("end_mask", autotile_mask(&m, 0, 9, 5) == 8);
    check("isolated_mask", autotile_mask(&m, 0, 20, 20) == 0);
    check("empty_variant", autotile_variant(&m, 0, 20, 20) == -1);
    check("filled_variant", autotile_variant(&m, 0, 7, 5) == (2 | 8));

    check("cull_all", cull_count(&m, 0, 0, W - 1, H - 1) == 6);
    check("cull_sub", cull_count(&m, 5, 5, 9, 5) == 6);
    check("cull_narrow", cull_count(&m, 6, 5, 8, 5) == 3);

    m.layer_visible[0] = 0;
    check("cull_hidden_layer", cull_count(&m, 0, 0, W - 1, H - 1) == 1);
    m.layer_visible[0] = 1;

    check("out_of_range_set", (map_set(&m, 0, -1, 0, 1), map_get(&m, 0, -1, 0)) == 0);
    check("bad_layer", map_get(&m, 9, 0, 0) == 0);

    /* trigger zones: repeat zone, events pair up with boundary crossings */
    TriggerSys ts;
    trig_init(&ts);
    int zr = trig_add(&ts, 2, 2, 4, 4, 1, TRIG_REPEAT);
    trig_place(&ts, 0, 0);
    check("trig_spawn_outside_quiet", ts.ev_n == 0);
    trig_move(&ts, 2, 2);                    /* cross in */
    trig_move(&ts, 3, 3);                    /* move inside: no event */
    trig_move(&ts, 5, 5);                    /* cross out */
    trig_move(&ts, 4, 4);                    /* cross in again */
    trig_move(&ts, 0, 0);                    /* cross out again */
    check("trig_repeat_enter", trig_count(&ts, zr, EV_ENTER) == 2);
    check("trig_repeat_exit", trig_count(&ts, zr, EV_EXIT) == 2);
    check("trig_repeat_matches_crossings", ts.ev_n == 4);   /* 4 crossings */

    /* once zone: only the first enter+exit pair fires */
    trig_init(&ts);
    int zo = trig_add(&ts, 2, 2, 4, 4, 1, TRIG_ONCE);
    trig_place(&ts, 0, 0);
    trig_move(&ts, 3, 3);
    trig_move(&ts, 0, 0);
    trig_move(&ts, 3, 3);                    /* second stay: inert */
    trig_move(&ts, 0, 0);
    check("trig_once_enter", trig_count(&ts, zo, EV_ENTER) == 1);
    check("trig_once_exit", trig_count(&ts, zo, EV_EXIT) == 1);
    check("trig_once_total", ts.ev_n == 2);

    /* nested zones: higher priority owns the overlap, no double firing */
    trig_init(&ts);
    int za = trig_add(&ts, 1, 1, 8, 8, 1, TRIG_REPEAT);
    int zb = trig_add(&ts, 3, 3, 5, 5, 2, TRIG_REPEAT);
    trig_place(&ts, 0, 0);
    trig_move(&ts, 2, 2);                    /* enter A */
    trig_move(&ts, 4, 4);                    /* into B: exit A + enter B only */
    trig_move(&ts, 4, 3);                    /* still inside B: no event */
    trig_move(&ts, 7, 7);                    /* back to A-only: exit B + enter A */
    trig_move(&ts, 0, 0);                    /* exit A */
    check("trig_nest_a_enter", trig_count(&ts, za, EV_ENTER) == 2);
    check("trig_nest_a_exit", trig_count(&ts, za, EV_EXIT) == 2);
    check("trig_nest_b_enter", trig_count(&ts, zb, EV_ENTER) == 1);
    check("trig_nest_b_exit", trig_count(&ts, zb, EV_EXIT) == 1);
    check("trig_nest_no_double", ts.ev_n == 6);

    /* partial overlap: entering the overlap fires only the higher zone */
    trig_init(&ts);
    int zc = trig_add(&ts, 0, 0, 4, 4, 1, TRIG_REPEAT);
    int zd = trig_add(&ts, 2, 0, 6, 4, 2, TRIG_REPEAT);
    trig_place(&ts, 1, 2);                   /* spawn inside C: enter fires */
    trig_move(&ts, 3, 2);                    /* overlap: exit C, enter D */
    trig_move(&ts, 5, 2);                    /* D-only: no event */
    trig_move(&ts, 8, 8);                    /* exit D */
    check("trig_overlap_c", trig_count(&ts, zc, EV_ENTER) == 1 &&
                            trig_count(&ts, zc, EV_EXIT) == 1);
    check("trig_overlap_d", trig_count(&ts, zd, EV_ENTER) == 1 &&
                            trig_count(&ts, zd, EV_EXIT) == 1);
    check("trig_overlap_total", ts.ev_n == 4);

    /* spawn inside a zone fires its enter, and the exit still pairs up */
    trig_init(&ts);
    int zs = trig_add(&ts, 2, 2, 4, 4, 1, TRIG_REPEAT);
    trig_place(&ts, 3, 3);
    check("trig_spawn_enter", trig_count(&ts, zs, EV_ENTER) == 1);
    trig_move(&ts, 9, 9);
    check("trig_spawn_exit_pair", trig_count(&ts, zs, EV_EXIT) == 1);

    /* unsolvable configs are reported, solvable ones are not */
    trig_init(&ts);
    trig_add(&ts, 0, 0, 3, 3, 1, TRIG_REPEAT);
    trig_add(&ts, 2, 2, 5, 5, 1, TRIG_REPEAT);       /* same prio, overlaps */
    check("trig_validate_conflict", trig_validate(&ts) == 1);
    trig_init(&ts);
    trig_add(&ts, 0, 0, 3, 3, 1, TRIG_REPEAT);
    trig_add(&ts, 2, 2, 5, 5, 2, TRIG_REPEAT);       /* overlap, ranked */
    trig_add(&ts, 4, 4, 6, 6, 1, TRIG_REPEAT);       /* same prio, disjoint */
    check("trig_validate_ok", trig_validate(&ts) == 0);
    trig_init(&ts);
    trig_add(&ts, 5, 5, 2, 2, 1, TRIG_REPEAT);       /* malformed rect */
    check("trig_validate_bad_rect", trig_validate(&ts) == 1);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
