
/* decor: procedural decoration placement on a tile map.
   Per-tile-type density rules scatter decorations with a seeded RNG:
   walkable cells and existing decorations are avoided, placements keep
   a minimum spacing (no overlap), and the same seed reproduces the
   exact same layout. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_W 32
#define MAX_H 32
#define MAX_RULES 8

typedef struct {
    int w, h;
    unsigned char tile[MAX_H][MAX_W];   /* tile type id */
    unsigned char walk[MAX_H][MAX_W];   /* 1 = walkable, never decorated */
    unsigned char decor[MAX_H][MAX_W];  /* 0 = none, else decoration id */
} Map;

typedef struct {
    unsigned char tile_type; /* only cells of this type are eligible */
    unsigned char decor_id;  /* id written into the decor layer */
    double density;          /* target fraction of eligible cells, 0..1 */
    int spacing;             /* min Chebyshev distance between decorations */
} Rule;

typedef struct { int eligible, target, placed; } RuleStat;

/* splitmix64: deterministic, seedable, platform-independent */
static unsigned long long rng_state;
static void rng_seed(unsigned long long s) { rng_state = s; }
static unsigned long long rng_next(void) {
    unsigned long long z = (rng_state += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

/* no decoration within Chebyshev distance < spacing of (x,y)? */
static int spacing_ok(const Map* m, int x, int y, int spacing) {
    for (int dy = -(spacing - 1); dy <= spacing - 1; dy++)
        for (int dx = -(spacing - 1); dx <= spacing - 1; dx++) {
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= m->w || ny >= m->h) continue;
            if (m->decor[ny][nx]) return 0;
        }
    return 1;
}

/* Scatter decorations rule by rule. Eligible cells are collected in scan
   order, shuffled (Fisher-Yates) with the seeded RNG, then filled greedily
   up to round(eligible * density) while respecting spacing. Fixed visit
   order plus a seeded RNG makes the whole run reproducible. */
static void scatter(Map* m, const Rule* rules, int nrules,
                    unsigned long long seed, RuleStat* stats) {
    int cand[MAX_W * MAX_H];
    rng_seed(seed);
    for (int r = 0; r < nrules; r++) {
        const Rule* rule = &rules[r];
        int nc = 0;
        for (int y = 0; y < m->h; y++)
            for (int x = 0; x < m->w; x++)
                if (m->tile[y][x] == rule->tile_type && !m->walk[y][x]
                    && !m->decor[y][x] && spacing_ok(m, x, y, rule->spacing))
                    cand[nc++] = y * m->w + x;
        int target = (int)(nc * rule->density + 0.5);
        for (int i = nc - 1; i > 0; i--) {
            int j = (int)(rng_next() % (unsigned long long)(i + 1));
            int t = cand[i]; cand[i] = cand[j]; cand[j] = t;
        }
        int placed = 0;
        for (int i = 0; i < nc && placed < target; i++) {
            int x = cand[i] % m->w, y = cand[i] / m->w;
            if (!spacing_ok(m, x, y, rule->spacing)) continue;
            m->decor[y][x] = rule->decor_id;
            placed++;
        }
        if (stats) {
            stats[r].eligible = nc;
            stats[r].target = target;
            stats[r].placed = placed;
        }
    }
}

/* 4-neighbour flood fill over walkable cells, for the path-clear check */
static int reachable_count(const Map* m, int sx, int sy) {
    static const int D[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
    unsigned char seen[MAX_H][MAX_W];
    int q[MAX_W * MAX_H], qh = 0, qt = 0, n = 0;
    memset(seen, 0, sizeof(seen));
    if (!m->walk[sy][sx]) return 0;
    seen[sy][sx] = 1;
    q[qt++] = sy * m->w + sx;
    while (qh < qt) {
        int cur = q[qh++], x = cur % m->w, y = cur / m->w;
        n++;
        for (int d = 0; d < 4; d++) {
            int nx = x + D[d][0], ny = y + D[d][1];
            if (nx < 0 || ny < 0 || nx >= m->w || ny >= m->h) continue;
            if (seen[ny][nx] || !m->walk[ny][nx]) continue;
            seen[ny][nx] = 1;
            q[qt++] = ny * m->w + nx;
        }
    }
    return n;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

enum { T_GRASS = 1, T_MEADOW = 2, T_WATER = 3, T_ROAD = 4 };
enum { D_TUFT = 1, D_ROCK = 2, D_EXISTING = 9 };

static void build_map(Map* m) {
    memset(m, 0, sizeof(*m));
    m->w = 32; m->h = 24;
    for (int y = 0; y < m->h; y++)
        for (int x = 0; x < m->w; x++)
            m->tile[y][x] = T_GRASS;
    for (int y = 10; y < 20; y++)          /* meadow patch */
        for (int x = 16; x < 28; x++)
            m->tile[y][x] = T_MEADOW;
    for (int y = 2; y < 6; y++)            /* pond: no rule, stays bare */
        for (int x = 20; x < 26; x++)
            m->tile[y][x] = T_WATER;
    for (int x = 0; x < m->w; x++) {       /* horizontal road */
        m->tile[12][x] = T_ROAD;
        m->walk[12][x] = 1;
    }
    for (int y = 0; y < m->h; y++) {       /* vertical road */
        m->tile[y][8] = T_ROAD;
        m->walk[y][8] = 1;
    }
    m->decor[3][3] = D_EXISTING;           /* hand-placed, must survive */
    m->decor[5][14] = D_EXISTING;
}

static int rule_spacing(const Rule* rules, int nrules, int decor_id) {
    for (int i = 0; i < nrules; i++)
        if (rules[i].decor_id == decor_id) return rules[i].spacing;
    return 0;
}

int main(void) {
    Rule rules[] = {
        { T_GRASS, D_TUFT, 0.20, 1 },
        { T_MEADOW, D_ROCK, 0.15, 2 },
    };
    int nrules = 2;
    RuleStat sa[MAX_RULES], sb[MAX_RULES];

    Map a; build_map(&a);
    int reach_before = reachable_count(&a, 0, 12);
    scatter(&a, rules, nrules, 42, sa);

    Map b; build_map(&b);
    scatter(&b, rules, nrules, 42, sb);
    check("reproducible", memcmp(a.decor, b.decor, sizeof(a.decor)) == 0);

    Map c; build_map(&c);
    scatter(&c, rules, nrules, 7, 0);
    check("seed_varies", memcmp(a.decor, c.decor, sizeof(a.decor)) != 0);

    int walk_clear = 1, water_clear = 1, type_ok = 1, nexist = 0;
    for (int y = 0; y < a.h; y++)
        for (int x = 0; x < a.w; x++) {
            int d = a.decor[y][x];
            if (!d) continue;
            if (a.walk[y][x]) walk_clear = 0;
            if (a.tile[y][x] == T_WATER) water_clear = 0;
            if (d == D_TUFT && a.tile[y][x] != T_GRASS) type_ok = 0;
            if (d == D_ROCK && a.tile[y][x] != T_MEADOW) type_ok = 0;
            if (d == D_EXISTING) nexist++;
        }
    check("avoids_walkable", walk_clear);
    check("avoids_unruled_tiles", water_clear && type_ok);
    check("existing_preserved",
          nexist == 2 && a.decor[3][3] == D_EXISTING && a.decor[5][14] == D_EXISTING);

    /* no overlap: each new decor keeps its rule's spacing from every other */
    int spacing_ok_all = 1;
    for (int y = 0; y < a.h && spacing_ok_all; y++)
        for (int x = 0; x < a.w && spacing_ok_all; x++) {
            int d = a.decor[y][x];
            if (d == 0 || d == D_EXISTING) continue;
            int sp = rule_spacing(rules, nrules, d);
            for (int qy = 0; qy < a.h; qy++)
                for (int qx = 0; qx < a.w; qx++) {
                    if ((qx != x || qy != y) && a.decor[qy][qx]) {
                        int dist = abs(qx - x) > abs(qy - y) ? abs(qx - x) : abs(qy - y);
                        if (dist < sp) spacing_ok_all = 0;
                    }
                }
        }
    check("no_overlap", spacing_ok_all);

    for (int r = 0; r < nrules; r++) {
        double expected = sa[r].eligible * rules[r].density;
        double dev = fabs(sa[r].placed - expected) / expected;
        char name[64];
        snprintf(name, sizeof(name), "target_hit_rule%d", r);
        check(name, sa[r].placed == sa[r].target);
        snprintf(name, sizeof(name), "density_within_5pct_rule%d", r);
        check(name, dev < 0.05);
        printf("rule%d eligible=%d target=%d placed=%d dev=%.4f\n",
               r, sa[r].eligible, sa[r].target, sa[r].placed, dev);
    }

    check("path_clear", reach_before > 0 && reachable_count(&a, 0, 12) == reach_before);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
