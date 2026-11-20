
/* shadow-pipeline: multi-light shadow casting over a fixed shadow-map pool.

   Per frame:
     1. candidate selection - rank shadow-casting lights by receiver coverage,
                              keep top MAX_SHADOW_MAPS (hard cap)
     2. occluder collection - per selected light, gather occluders that can
                              block receivers inside the light radius. Exact:
                              an occluder whose near point is beyond the radius
                              cannot intersect any light->receiver segment.
     3. map alloc / reuse   - fixed pool; a map is re-rendered only when its
                              light or a collected occluder moved (content hash).

   A shadow map is a depth-sorted list of angular occluder intervals used as a
   broad phase; visibility is confirmed with an exact segment-circle test, so
   the image is identical to casting every occluder per light. The map only
   prunes. Pool size and per-map capacity are compile-time bounds. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_SHADOW_MAPS 4       /* hard cap on shadow maps in the pool */
#define MAX_INTERVALS   64      /* hard cap on intervals per map */
#define MAX_LIGHTS      32
#define MAX_OCCLUDERS   64
#define SIDE            16
#define N_RECEIVERS     (SIDE * SIDE)
#define WORLD           40.0f
#define EPS             1e-3f
#define PI              3.14159265358979f

typedef struct { float x, y, radius, intensity; } Light;
typedef struct { float x, y, r; } Occluder;
typedef struct { float a0, a1, depth; int occ; } Interval;

typedef struct {
    Interval iv[MAX_INTERVALS];
    int      count;
    int      owner;             /* light id, -1 = free */
    unsigned hash;              /* content hash: light + collected occluders */
    int      drops;             /* intervals dropped at capacity (breaks exactness) */
} ShadowMap;

typedef struct {
    long collect_tests;         /* light-occluder distance tests */
    long renders;               /* shadow map (re)renders */
    long reuses;                /* frames a map was kept as-is */
    long queries;               /* receiver visibility queries */
    long confirms;              /* exact segment-circle tests from queries */
    long intervals;             /* intervals written into maps */
    long ref_tests;             /* exact tests the per-light full cast would do */
} Stats;

static ShadowMap pool[MAX_SHADOW_MAPS];

/* ---------- exact visibility (ground truth) ---------- */

static int ray_blocked(float x0, float y0, float x1, float y1, Occluder o) {
    float dx = x1 - x0, dy = y1 - y0;
    float len2 = dx * dx + dy * dy;
    float t = 0.0f;
    if (len2 > 0.0f) {
        t = ((o.x - x0) * dx + (o.y - y0) * dy) / len2;
        if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
    }
    float cx = x0 + t * dx - o.x, cy = y0 + t * dy - o.y;
    return cx * cx + cy * cy < o.r * o.r;
}

/* 逐光源全量投射: every light x every receiver x every occluder */
static int reference_lit(const Light* L, const Occluder* o, int n,
                         float rx, float ry, Stats* st) {
    for (int i = 0; i < n; i++) {
        st->ref_tests++;
        if (ray_blocked(L->x, L->y, rx, ry, o[i])) return 0;
    }
    return 1;
}

/* ---------- shadow map ---------- */

static void map_insert(ShadowMap* m, float a0, float a1, float depth, int occ, Stats* st) {
    int pos;
    if (m->count == MAX_INTERVALS) { m->drops++; pos = MAX_INTERVALS - 1; }
    else pos = m->count++;
    while (pos > 0 && m->iv[pos - 1].depth > depth) { m->iv[pos] = m->iv[pos - 1]; pos--; }
    m->iv[pos] = (Interval){ a0, a1, depth, occ };
    st->intervals++;
}

/* add angular span, splitting at the +-pi wrap */
static void map_add(ShadowMap* m, float a0, float a1, float depth, int occ, Stats* st) {
    if (a1 - a0 >= 2.0f * PI) { map_insert(m, -PI, PI, depth, occ, st); return; }
    if (a0 < -PI) {
        map_insert(m, a0 + 2.0f * PI, PI, depth, occ, st);
        map_insert(m, -PI, a1, depth, occ, st);
    } else if (a1 > PI) {
        map_insert(m, a0, PI, depth, occ, st);
        map_insert(m, -PI, a1 - 2.0f * PI, depth, occ, st);
    } else {
        map_insert(m, a0, a1, depth, occ, st);
    }
}

/* map query: broad phase by interval, exact confirm per candidate occluder */
static int query_lit(const ShadowMap* m, const Light* L, const Occluder* occs,
                     float rx, float ry, Stats* st) {
    float dx = rx - L->x, dy = ry - L->y;
    float t = sqrtf(dx * dx + dy * dy);
    float ang = atan2f(dy, dx);
    for (int i = 0; i < m->count; i++) {
        const Interval* iv = &m->iv[i];
        if (iv->depth > t + EPS) break;     /* depth-sorted: nothing nearer remains */
        if (ang < iv->a0 || ang > iv->a1) continue;
        st->confirms++;
        if (ray_blocked(L->x, L->y, rx, ry, occs[iv->occ])) return 0;
    }
    return 1;
}

/* ---------- pipeline stages ---------- */

static unsigned mixf(unsigned h, float f) {
    unsigned u; memcpy(&u, &f, 4);
    return (h ^ u) * 16777619u;
}

/* stage 2: collect occluders that can block a receiver inside the light
   radius; hash the collection so stage 3 can detect unchanged maps */
static unsigned collect_occluders(const Light* L, const Occluder* o, int n,
                                  int* ids, int* count, Stats* st) {
    unsigned h = 2166136261u;
    h = mixf(mixf(mixf(h, L->x), L->y), L->radius);
    int c = 0;
    for (int i = 0; i < n; i++) {
        st->collect_tests++;
        float dx = o[i].x - L->x, dy = o[i].y - L->y;
        float d = sqrtf(dx * dx + dy * dy);
        if (d - o[i].r > L->radius + EPS) continue;   /* provably cannot block */
        ids[c++] = i;
        h = (h ^ (unsigned)i) * 16777619u;
        h = mixf(mixf(mixf(h, o[i].x), o[i].y), o[i].r);
    }
    *count = c;
    return h;
}

static void render_map(ShadowMap* m, int owner, const Light* L, const Occluder* o,
                       const int* ids, int n, unsigned hash, Stats* st) {
    m->count = 0; m->drops = 0; m->owner = owner; m->hash = hash;
    for (int k = 0; k < n; k++) {
        Occluder oc = o[ids[k]];
        float dx = oc.x - L->x, dy = oc.y - L->y;
        float d = sqrtf(dx * dx + dy * dy);
        if (d <= oc.r) { map_add(m, -PI, PI, 0.0f, ids[k], st); continue; }
        float ang = atan2f(dy, dx);
        float half = asinf(oc.r / d);
        map_add(m, ang - half, ang + half, d - oc.r, ids[k], st);
    }
    st->renders++;
}

/* stage 1: rank casting lights by intensity * receivers in range, keep top
   MAX_SHADOW_MAPS. Returns selected light ids, worst-first eviction order. */
static int select_lights(const Light* L, const int* casting, int n,
                         const float* rx, const float* ry, int* out) {
    float score[MAX_LIGHTS];
    int ids[MAX_LIGHTS], m = 0;
    for (int i = 0; i < n; i++) {
        const Light* li = &L[casting[i]];
        long cover = 0;
        for (int r = 0; r < N_RECEIVERS; r++) {
            float dx = rx[r] - li->x, dy = ry[r] - li->y;
            if (dx * dx + dy * dy <= li->radius * li->radius) cover++;
        }
        score[m] = li->intensity * (float)cover;
        ids[m] = casting[i];
        m++;
    }
    for (int i = 1; i < m; i++) {           /* stable insertion sort, desc */
        float s = score[i]; int id = ids[i], j = i - 1;
        while (j >= 0 && score[j] < s) { score[j + 1] = score[j]; ids[j + 1] = ids[j]; j--; }
        score[j + 1] = s; ids[j + 1] = id;
    }
    int keep = m < MAX_SHADOW_MAPS ? m : MAX_SHADOW_MAPS;
    for (int i = 0; i < keep; i++) out[i] = ids[i];
    return keep;
}

static int slot_of(int light_id) {
    for (int i = 0; i < MAX_SHADOW_MAPS; i++)
        if (pool[i].owner == light_id) return i;
    return -1;
}

/* run stages 1-3; fills light_slot[light_id] for selected lights */
static int pipeline_frame(const Light* lights, const int* casting, int ncasting,
                          const Occluder* occs, int noccs,
                          const float* rx, const float* ry,
                          int* selected, int* light_slot, Stats* st) {
    int nsel = select_lights(lights, casting, ncasting, rx, ry, selected);
    for (int i = 0; i < nsel; i++) {
        int lid = selected[i];
        int ids[MAX_OCCLUDERS], n = 0;
        unsigned h = collect_occluders(&lights[lid], occs, noccs, ids, &n, st);
        int s = slot_of(lid);
        if (s >= 0 && pool[s].hash == h) { st->reuses++; }
        else {
            if (s < 0) {                    /* allocate: free slot, else evict unselected */
                s = slot_of(-1);
                if (s < 0) {
                    for (int k = 0; k < MAX_SHADOW_MAPS; k++) {
                        int owned = pool[k].owner, still = 0;
                        for (int j = 0; j < nsel; j++) if (selected[j] == owned) still = 1;
                        if (!still) { s = k; break; }
                    }
                }
            }
            render_map(&pool[s], lid, &lights[lid], occs, ids, n, h, st);
        }
        light_slot[lid] = s;
    }
    return nsel;
}

/* ---------- scenes & reports ---------- */

static void make_receivers(float* rx, float* ry) {
    for (int y = 0; y < SIDE; y++)
        for (int x = 0; x < SIDE; x++) {
            rx[y * SIDE + x] = ((float)x + 0.5f) * (WORLD / SIDE);
            ry[y * SIDE + x] = ((float)y + 0.5f) * (WORLD / SIDE);
        }
}

static float attenuation(const Light* L, float rx, float ry) {
    float dx = rx - L->x, dy = ry - L->y;
    float d = sqrtf(dx * dx + dy * dy);
    float a = 1.0f - d / L->radius;
    return a > 0.0f ? L->intensity * a : 0.0f;
}

int main(void) {
    float rx[N_RECEIVERS], ry[N_RECEIVERS];
    make_receivers(rx, ry);
    for (int i = 0; i < MAX_SHADOW_MAPS; i++) pool[i].owner = -1;

    printf("shadow_mem_bound=%zu bytes (%d maps x %d intervals x %zu B)\n",
           sizeof(pool), MAX_SHADOW_MAPS, MAX_INTERVALS, sizeof(Interval));

    Occluder occs[] = {
        {15,10,1.5f},{10,15,1.2f},{25,10,1.0f},{30,18,1.5f},{18,25,1.8f},
        {24,30,1.2f},{12,28,1.0f},{33,30,1.5f},{20,18,1.0f},{27,22,0.8f},
    };
    int noccs = (int)(sizeof(occs) / sizeof(occs[0]));

    /* --- consistency: 3 casting lights <= pool, pipeline vs full cast --- */
    Light lights[] = {
        {10,10,18,1.0f}, {30,12,15,0.8f}, {20,30,20,1.2f},
    };
    int casting[] = {0, 1, 2};
    Stats st = {0};
    int mismatches = 0, drops = 0;
    for (int frame = 0; frame < 3; frame++) {
        if (frame == 1) { occs[5].x += 2.0f; }                     /* move an occluder */
        if (frame == 2) { lights[1].x -= 4.0f; }                   /* move a light */
        int selected[MAX_SHADOW_MAPS], light_slot[MAX_LIGHTS];
        for (int i = 0; i < MAX_LIGHTS; i++) light_slot[i] = -1;
        int nsel = pipeline_frame(lights, casting, 3, occs, noccs, rx, ry,
                                  selected, light_slot, &st);
        for (int i = 0; i < nsel; i++) {
            int lid = selected[i];
            ShadowMap* m = &pool[light_slot[lid]];
            drops += m->drops;
            for (int r = 0; r < N_RECEIVERS; r++) {
                if (attenuation(&lights[lid], rx[r], ry[r]) <= 0.0f) continue;
                st.queries++;
                int pipe = query_lit(m, &lights[lid], occs, rx[r], ry[r], &st);
                int ref  = reference_lit(&lights[lid], occs, noccs, rx[r], ry[r], &st);
                if (pipe != ref) mismatches++;
            }
        }
        printf("frame=%d renders=%ld reuses=%ld\n", frame, st.renders, st.reuses);
    }
    printf("consistency: mismatches=%d interval_drops=%d queries=%ld confirms=%ld ref_tests=%ld\n",
           mismatches, drops, st.queries, st.confirms, st.ref_tests);
    printf("consistent=%d\n", mismatches == 0 && drops == 0);

    /* --- budget: 6 casting lights > 4 maps, report what is lost --- */
    Light many[] = {
        {10,10,18,1.0f},{30,12,15,0.8f},{20,30,20,1.2f},
        {32,32,14,0.9f},{8,30,12,0.7f},{20,8,12,0.6f},
    };
    int casting6[] = {0,1,2,3,4,5};
    for (int i = 0; i < MAX_SHADOW_MAPS; i++) pool[i].owner = -1;
    Stats bs = {0};
    int selected[MAX_SHADOW_MAPS], light_slot[MAX_LIGHTS];
    for (int i = 0; i < MAX_LIGHTS; i++) light_slot[i] = -1;
    int nsel = pipeline_frame(many, casting6, 6, occs, noccs, rx, ry, selected, light_slot, &bs);
    double ideal = 0.0, got = 0.0;
    for (int r = 0; r < N_RECEIVERS; r++) {
        for (int l = 0; l < 6; l++) {
            float a = attenuation(&many[l], rx[r], ry[r]);
            if (a <= 0.0f) continue;
            ideal += a * reference_lit(&many[l], occs, noccs, rx[r], ry[r], &bs);
            int s = light_slot[l];
            if (s >= 0) got += a * query_lit(&pool[s], &many[l], occs, rx[r], ry[r], &bs);
            else        got += a;           /* over budget: light without shadow */
        }
    }
    printf("budget: lights=6 selected=%d dropped=", nsel);
    for (int l = 0; l < 6; l++) {
        int found = 0;
        for (int i = 0; i < nsel; i++) if (selected[i] == l) found = 1;
        if (!found) printf("%d ", l);
    }
    printf("\nbudget: ideal_energy=%.1f pipeline_energy=%.1f unshadowed_excess=%.2f%%\n",
           ideal, got, ideal > 0.0 ? 100.0 * (got - ideal) / ideal : 0.0);

    /* --- scaling: cost vs light count, memory bound stays fixed --- */
    printf("scaling: lights selected collect_tests renders queries confirms ref_tests\n");
    for (int nl = 1; nl <= 32; nl *= 2) {
        Light ls[MAX_LIGHTS];
        int cast[MAX_LIGHTS];
        for (int i = 0; i < nl; i++) {
            float a = 2.0f * PI * (float)i / (float)nl;
            ls[i] = (Light){ 20 + 12.0f * cosf(a), 20 + 12.0f * sinf(a), 14, 1.0f };
            cast[i] = i;
        }
        for (int i = 0; i < MAX_SHADOW_MAPS; i++) pool[i].owner = -1;
        Stats ss = {0};
        int sel[MAX_SHADOW_MAPS], slot[MAX_LIGHTS];
        for (int i = 0; i < MAX_LIGHTS; i++) slot[i] = -1;
        int n = pipeline_frame(ls, cast, nl, occs, noccs, rx, ry, sel, slot, &ss);
        for (int i = 0; i < n; i++)
            for (int r = 0; r < N_RECEIVERS; r++) {
                ss.queries++;
                query_lit(&pool[slot[sel[i]]], &ls[sel[i]], occs, rx[r], ry[r], &ss);
            }
        for (int l = 0; l < nl; l++)
            for (int r = 0; r < N_RECEIVERS; r++)
                reference_lit(&ls[l], occs, noccs, rx[r], ry[r], &ss);
        printf("scaling: %6d %8d %13ld %7ld %7ld %8ld %9ld\n",
               nl, n, ss.collect_tests, ss.renders, ss.queries, ss.confirms, ss.ref_tests);
    }
    return 0;
}
