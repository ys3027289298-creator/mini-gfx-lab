
/* ccd: two-level particle-vs-obstacle collision.
   Broad phase is a sorted sweep-and-prune over swept AABBs; every candidate
   carries the particle's swept box and the obstacle's box into the narrow
   phase. Narrow phase is continuous (segment vs circle), so fast or thin
   movers cannot tunnel. Each frame is also solved by brute-force all-pairs
   narrow phase, and the two contact sets must match bit for bit. */
#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define DT (1.0 / 60.0)
#define PI 3.14159265358979323846
#define BROAD_EPS 1e-9   /* conservative inflation so rounding never culls a hit */
#define BENCH_PARTICLES 10000
#define BENCH_OBSTACLES 1000

typedef struct { double x, y; } Vec2;
typedef struct { double minx, miny, maxx, maxy; } Box;

typedef struct {
    Vec2 a, b;          /* circle center at frame start / frame end */
    double r;
} Particle;

typedef struct { Vec2 c; double r; } Obstacle;

typedef struct {
    Particle* particles; int particle_count;
    Obstacle* obstacles; int obstacle_count;
} World;

/* A candidate pair plus the bounding volumes handed from broad to narrow. */
typedef struct {
    int particle, obstacle;
    Box swept;          /* particle swept AABB, own radius included */
    Box obstacle_box;   /* obstacle AABB, own radius included */
} Candidate;

typedef struct { int particle, obstacle; double toi; } Contact;

typedef struct { Candidate* items; int count, cap; } CandidateList;
typedef struct { Contact* items; int count, cap; } ContactList;

/* Sweep event. type 0 = particle, 1 = obstacle. */
typedef struct { double lo, hi; int obj, type; } Edge;

typedef struct {
    Edge* edges; int edge_cap;
    int* active; int active_cap;
    Box* pboxes; int pbox_cap;
    Box* oboxes; int obox_cap;
    CandidateList candidates;
} Broad;

typedef struct {
    long long pairs, candidates, contacts;
    long long mismatch, broad_missed, discrete_missed, box_reject;
} Stats;

static void* xrealloc(void* p, size_t n) {
    void* q = realloc(p, n);
    if (!q) { fprintf(stderr, "out of memory\n"); exit(1); }
    return q;
}

/* --- geometry ---------------------------------------------------------- */

static Box box_of_circle(Vec2 c, double r) {
    Box b;
    b.minx = c.x - r - BROAD_EPS; b.maxx = c.x + r + BROAD_EPS;
    b.miny = c.y - r - BROAD_EPS; b.maxy = c.y + r + BROAD_EPS;
    return b;
}

static Box box_swept(Vec2 a, Vec2 b, double r) {
    Box s;
    s.minx = fmin(a.x, b.x) - r - BROAD_EPS;
    s.maxx = fmax(a.x, b.x) + r + BROAD_EPS;
    s.miny = fmin(a.y, b.y) - r - BROAD_EPS;
    s.maxy = fmax(a.y, b.y) + r + BROAD_EPS;
    return s;
}

static int box_overlap(const Box* a, const Box* b) {
    return a->minx <= b->maxx && b->minx <= a->maxx &&
           a->miny <= b->maxy && b->miny <= a->maxy;
}

/* Continuous narrow phase: does the circle of radius r swept along a->b
   intersect the static circle at c? Also returns the entry time in [0,1].
   The hit test is a closest-point-on-segment test; the time is the smaller
   root of |a + t(b-a) - c|^2 = r^2. */
static int sweep_circle_hit(Vec2 a, Vec2 b, Vec2 c, double r, double* toi) {
    double dx = b.x - a.x, dy = b.y - a.y;
    double fx = a.x - c.x, fy = a.y - c.y;
    double r2 = r * r;
    double len2 = dx * dx + dy * dy;
    double t, qx, qy, bq, cq, disc;

    if (fx * fx + fy * fy <= r2) {          /* already overlapping at t = 0 */
        *toi = 0.0;
        return 1;
    }
    if (len2 == 0.0) return 0;              /* stationary and outside */

    t = -(fx * dx + fy * dy) / len2;
    if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
    qx = fx + dx * t;
    qy = fy + dy * t;
    /* tolerance keeps an exact tangency a hit without admitting a graze */
    if (qx * qx + qy * qy > r2 * (1.0 + 1e-12)) return 0;

    bq = fx * dx + fy * dy;
    cq = fx * fx + fy * fy - r2;            /* > 0: start is outside */
    disc = bq * bq - len2 * cq;
    if (disc < 0.0) disc = 0.0;             /* tangency rounded below zero */
    t = (-bq - sqrt(disc)) / len2;          /* entry root; = tangency point */
    if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
    *toi = t;
    return 1;
}

/* --- broad phase: sorted sweep-and-prune ------------------------------- */

static int edge_cmp(const void* va, const void* vb) {
    const Edge* a = (const Edge*)va;
    const Edge* b = (const Edge*)vb;
    if (a->lo < b->lo) return -1;
    if (a->lo > b->lo) return 1;
    if (a->hi < b->hi) return -1;
    if (a->hi > b->hi) return 1;
    if (a->type != b->type) return a->type - b->type;
    return a->obj - b->obj;
}

static int candidate_cmp(const void* va, const void* vb) {
    const Candidate* a = (const Candidate*)va;
    const Candidate* b = (const Candidate*)vb;
    if (a->particle != b->particle) return a->particle - b->particle;
    return a->obstacle - b->obstacle;
}

static void candidates_push(CandidateList* l, Candidate c) {
    if (l->count == l->cap) {
        l->cap = l->cap ? l->cap * 2 : 256;
        l->items = (Candidate*)xrealloc(l->items, sizeof(Candidate) * (size_t)l->cap);
    }
    l->items[l->count++] = c;
}

static void broad_reserve(Broad* bd, int np, int no) {
    int n = np + no;
    if (bd->edge_cap < n) {
        bd->edge_cap = n;
        bd->edges = (Edge*)xrealloc(bd->edges, sizeof(Edge) * (size_t)n);
        bd->active = (int*)xrealloc(bd->active, sizeof(int) * (size_t)n);
    }
    if (bd->pbox_cap < np) {
        bd->pbox_cap = np;
        bd->pboxes = (Box*)xrealloc(bd->pboxes, sizeof(Box) * (size_t)np);
    }
    if (bd->obox_cap < no) {
        bd->obox_cap = no;
        bd->oboxes = (Box*)xrealloc(bd->oboxes, sizeof(Box) * (size_t)no);
    }
}

/* Sort all swept boxes by their left edge, sweep, and emit particle/obstacle
   pairs whose boxes overlap on both axes. Output is sorted by pair key. */
static void broad_phase(Broad* bd, const Particle* ps, int np,
                        const Obstacle* obs, int no) {
    int n = np + no, active_n = 0;

    broad_reserve(bd, np, no);
    bd->candidates.count = 0;

    for (int i = 0; i < np; i++) {
        bd->pboxes[i] = box_swept(ps[i].a, ps[i].b, ps[i].r);
        bd->edges[i] = (Edge){ bd->pboxes[i].minx, bd->pboxes[i].maxx, i, 0 };
    }
    for (int j = 0; j < no; j++) {
        bd->oboxes[j] = box_of_circle(obs[j].c, obs[j].r);
        bd->edges[np + j] = (Edge){ bd->oboxes[j].minx, bd->oboxes[j].maxx, j, 1 };
    }
    if (n > 1) qsort(bd->edges, (size_t)n, sizeof(Edge), edge_cmp);

    for (int k = 0; k < n; k++) {
        const Edge* cur = &bd->edges[k];
        int w = 0;

        /* drop intervals that ended before this one starts */
        for (int i = 0; i < active_n; i++)
            if (bd->edges[bd->active[i]].hi >= cur->lo) bd->active[w++] = bd->active[i];
        active_n = w;

        for (int i = 0; i < active_n; i++) {
            const Edge* prev = &bd->edges[bd->active[i]];
            int pi, oi;
            if (prev->type == cur->type) continue;   /* only cross pairs */
            pi = (cur->type == 0) ? cur->obj : prev->obj;
            oi = (cur->type == 1) ? cur->obj : prev->obj;
            if (!box_overlap(&bd->pboxes[pi], &bd->oboxes[oi])) continue;
            candidates_push(&bd->candidates,
                            (Candidate){ pi, oi, bd->pboxes[pi], bd->oboxes[oi] });
        }
        bd->active[active_n++] = k;
    }

    if (bd->candidates.count > 1)
        qsort(bd->candidates.items, (size_t)bd->candidates.count,
              sizeof(Candidate), candidate_cmp);
}

/* --- narrow phase ------------------------------------------------------ */

static void contacts_push(ContactList* l, Contact c) {
    if (l->count == l->cap) {
        l->cap = l->cap ? l->cap * 2 : 256;
        l->items = (Contact*)xrealloc(l->items, sizeof(Contact) * (size_t)l->cap);
    }
    l->items[l->count++] = c;
}

/* Consume the bounding volumes handed over by the broad phase, then run the
   exact continuous test. Returns the number of candidates rejected by the
   transferred boxes; it must be zero, or broad and narrow disagree. */
static long long narrow_phase(const World* w, const CandidateList* cands,
                              ContactList* out) {
    long long box_reject = 0;
    out->count = 0;
    for (int i = 0; i < cands->count; i++) {
        const Candidate* c = &cands->items[i];
        const Particle* p = &w->particles[c->particle];
        const Obstacle* o = &w->obstacles[c->obstacle];
        double toi;
        if (!box_overlap(&c->swept, &c->obstacle_box)) { box_reject++; continue; }
        if (sweep_circle_hit(p->a, p->b, o->c, p->r + o->r, &toi))
            contacts_push(out, (Contact){ c->particle, c->obstacle, toi });
    }
    return box_reject;
}

/* Reference path: the same narrow phase over every particle/obstacle pair. */
static void brute_force(const World* w, ContactList* out) {
    out->count = 0;
    for (int i = 0; i < w->particle_count; i++) {
        const Particle* p = &w->particles[i];
        for (int j = 0; j < w->obstacle_count; j++) {
            const Obstacle* o = &w->obstacles[j];
            double toi;
            if (sweep_circle_hit(p->a, p->b, o->c, p->r + o->r, &toi))
                contacts_push(out, (Contact){ i, j, toi });
        }
    }
}

static int contact_cmp(const void* va, const void* vb) {
    const Contact* a = (const Contact*)va;
    const Contact* b = (const Contact*)vb;
    if (a->particle != b->particle) return a->particle - b->particle;
    return a->obstacle - b->obstacle;
}

static void contacts_sort(ContactList* l) {
    if (l->count > 1)
        qsort(l->items, (size_t)l->count, sizeof(Contact), contact_cmp);
}

static int contacts_equal(const ContactList* a, const ContactList* b) {
    if (a->count != b->count) return 0;
    for (int i = 0; i < a->count; i++) {
        if (a->items[i].particle != b->items[i].particle) return 0;
        if (a->items[i].obstacle != b->items[i].obstacle) return 0;
        if (a->items[i].toi != b->items[i].toi) return 0;   /* bit-exact */
    }
    return 1;
}

/* Binary search in the broad-phase output (sorted by pair key). */
static int candidates_contain(const CandidateList* cl, int pi, int oi) {
    int lo = 0, hi = cl->count - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        const Candidate* c = &cl->items[mid];
        if (c->particle < pi || (c->particle == pi && c->obstacle < oi)) lo = mid + 1;
        else if (c->particle == pi && c->obstacle == oi) return 1;
        else hi = mid - 1;
    }
    return 0;
}

static long long count_broad_misses(const CandidateList* cl, const ContactList* brute) {
    long long missed = 0;
    for (int i = 0; i < brute->count; i++)
        if (!candidates_contain(cl, brute->items[i].particle, brute->items[i].obstacle))
            missed++;
    return missed;
}

/* Continuous hits whose end-of-frame position is no longer overlapping:
   exactly the contacts a discrete per-frame overlap test would tunnel through. */
static long long count_discrete_misses(const World* w, const ContactList* brute) {
    long long missed = 0;
    for (int i = 0; i < brute->count; i++) {
        const Particle* p = &w->particles[brute->items[i].particle];
        const Obstacle* o = &w->obstacles[brute->items[i].obstacle];
        double rr = p->r + o->r;
        double dx = p->b.x - o->c.x, dy = p->b.y - o->c.y;
        if (dx * dx + dy * dy > rr * rr) missed++;
    }
    return missed;
}

static void verify(const World* w, Broad* bd, ContactList* pipe, ContactList* brute,
                   Stats* st, long long box_reject) {
    contacts_sort(pipe);
    contacts_sort(brute);
    st->pairs += (long long)w->particle_count * w->obstacle_count;
    st->candidates += bd->candidates.count;
    st->contacts += pipe->count;
    st->box_reject += box_reject;
    if (!contacts_equal(pipe, brute)) st->mismatch++;
    st->broad_missed += count_broad_misses(&bd->candidates, brute);
    st->discrete_missed += count_discrete_misses(w, brute);
}

/* --- worlds ------------------------------------------------------------ */

typedef struct { uint64_t s; } Rng;

static uint64_t rng_u64(Rng* r) {
    uint64_t x = r->s;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    r->s = x;
    return x;
}

static double rng_unit(Rng* r) {
    return (double)(rng_u64(r) >> 11) * (1.0 / 9007199254740992.0);
}

static double rng_range(Rng* r, double lo, double hi) {
    return lo + (hi - lo) * rng_unit(r);
}

static World world_random(uint64_t seed, int np, int no, double field,
                          double speed_lo, double speed_hi) {
    World w;
    Rng rng = { seed ? seed : 1u };
    w.particle_count = np;
    w.obstacle_count = no;
    w.particles = (Particle*)xrealloc(NULL, sizeof(Particle) * (size_t)(np ? np : 1));
    w.obstacles = (Obstacle*)xrealloc(NULL, sizeof(Obstacle) * (size_t)(no ? no : 1));

    for (int i = 0; i < np; i++) {
        double ang = rng_range(&rng, 0.0, 2.0 * PI);
        double spd = rng_range(&rng, speed_lo, speed_hi);
        w.particles[i].a.x = rng_range(&rng, 0.0, field);
        w.particles[i].a.y = rng_range(&rng, 0.0, field);
        w.particles[i].b.x = w.particles[i].a.x + cos(ang) * spd * DT;
        w.particles[i].b.y = w.particles[i].a.y + sin(ang) * spd * DT;
        w.particles[i].r = rng_range(&rng, 0.10, 0.35);
    }
    for (int j = 0; j < no; j++) {
        w.obstacles[j].c.x = rng_range(&rng, 0.0, field);
        w.obstacles[j].c.y = rng_range(&rng, 0.0, field);
        w.obstacles[j].r = rng_range(&rng, 0.4, 2.0);
    }
    return w;
}

static void world_free(World* w) {
    free(w->particles);
    free(w->obstacles);
    w->particles = NULL;
    w->obstacles = NULL;
}

/* --- timing ------------------------------------------------------------ */

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e3 + (double)ts.tv_nsec / 1e6;
}

static int double_cmp(const void* va, const void* vb) {
    double a = *(const double*)va, b = *(const double*)vb;
    return (a > b) - (a < b);
}

static double min_of(const double* v, int n) {
    double m = v[0];
    for (int i = 1; i < n; i++) if (v[i] < m) m = v[i];
    return m;
}

static double median_of(const double* v, int n) {
    double* t = (double*)xrealloc(NULL, sizeof(double) * (size_t)n);
    memcpy(t, v, sizeof(double) * (size_t)n);
    qsort(t, (size_t)n, sizeof(double), double_cmp);
    double m = (n & 1) ? t[n / 2] : 0.5 * (t[n / 2 - 1] + t[n / 2]);
    free(t);
    return m;
}

/* --- tests ------------------------------------------------------------- */

static int micro_ok = 1;

static void micro_case(const char* name, Vec2 a, Vec2 b, Vec2 c, double r,
                       int expect_hit, double expect_toi) {
    double toi = -1.0;
    int hit = sweep_circle_hit(a, b, c, r, &toi);
    int pass = (hit == expect_hit) && (!hit || fabs(toi - expect_toi) < 1e-12);
    printf("case=%-14s hit=%d toi=%.6f%s\n", name, hit, hit ? toi : 0.0,
           pass ? "" : "  <-- FAIL");
    if (!pass) micro_ok = 0;
}

static void micro_tests(void) {
    micro_case("head_on",     (Vec2){0, 0},   (Vec2){2, 0},   (Vec2){1.5, 0}, 0.75, 1, 0.375);
    micro_case("tunnel",      (Vec2){0, 0},   (Vec2){10, 0},  (Vec2){5, 0},   0.15, 1, 0.485);
    micro_case("tangent",     (Vec2){-2, 1},  (Vec2){2, 1},   (Vec2){0, 0},   1.00, 1, 0.500);
    micro_case("graze_miss",  (Vec2){-2, 1.0000001}, (Vec2){2, 1.0000001}, (Vec2){0, 0}, 1.00, 0, 0);
    micro_case("start_inside",(Vec2){0.1, 0}, (Vec2){5, 0},   (Vec2){0, 0},   0.70, 1, 0.000);
    micro_case("end_inside",  (Vec2){0, 0},   (Vec2){2, 0},   (Vec2){2.5, 0}, 0.75, 1, 0.875);
    micro_case("static_miss", (Vec2){0, 0},   (Vec2){0, 0},   (Vec2){5, 0},   1.00, 0, 0);
    micro_case("static_inside",(Vec2){0, 0},  (Vec2){0, 0},   (Vec2){0.5, 0}, 1.00, 1, 0.000);
}

static void broad_free(Broad* bd) {
    free(bd->edges); free(bd->active);
    free(bd->pboxes); free(bd->oboxes);
    free(bd->candidates.items);
    memset(bd, 0, sizeof(*bd));
}

/* Run one frame through both paths. Returns 1 when they agree. */
static int solve_frame(const World* w, Broad* bd, ContactList* pipe,
                       ContactList* brute, Stats* st) {
    long long box_reject;
    broad_phase(bd, w->particles, w->particle_count,
                w->obstacles, w->obstacle_count);
    box_reject = narrow_phase(w, &bd->candidates, pipe);
    brute_force(w, brute);
    verify(w, bd, pipe, brute, st, box_reject);
    return contacts_equal(pipe, brute) && box_reject == 0;
}

static int benchmark(int np, int no, int reps) {
    World w = world_random(0x00C0FFEE12345678ull, np, no, 400.0, 10.0, 120.0);
    Broad bd;
    ContactList pipe = {0}, brute = {0};
    Stats st;
    double t_broad[64], t_narrow[64], t_pipe[64], t_brute[64];
    int ok;

    memset(&bd, 0, sizeof(bd));
    memset(&st, 0, sizeof(st));
    if (reps > 64) reps = 64;

    ok = solve_frame(&w, &bd, &pipe, &brute, &st);
    st.pairs = (long long)np * no;
    st.candidates = bd.candidates.count;
    st.contacts = pipe.count;
    st.mismatch = ok ? 0 : 1;
    st.broad_missed = count_broad_misses(&bd.candidates, &brute);
    st.discrete_missed = count_discrete_misses(&w, &brute);

    broad_phase(&bd, w.particles, np, w.obstacles, no);   /* warm allocations */
    for (int i = 0; i < reps; i++) {
        double t0, t1, t2, t3;
        t0 = now_ms();
        broad_phase(&bd, w.particles, np, w.obstacles, no);
        t1 = now_ms();
        narrow_phase(&w, &bd.candidates, &pipe);
        t2 = now_ms();
        brute_force(&w, &brute);
        t3 = now_ms();
        t_broad[i] = t1 - t0;
        t_narrow[i] = t2 - t1;
        t_pipe[i] = t2 - t0;
        t_brute[i] = t3 - t2;
    }

    printf("bench_particles=%d\n", np);
    printf("bench_obstacles=%d\n", no);
    printf("bench_all_pairs=%lld\n", st.pairs);
    printf("bench_broad_candidates=%lld\n", st.candidates);
    printf("bench_contacts=%lld\n", st.contacts);
    printf("bench_broad_ms=%.4f\n", min_of(t_broad, reps));
    printf("bench_narrow_ms=%.4f\n", min_of(t_narrow, reps));
    printf("bench_pipeline_ms=%.4f\n", min_of(t_pipe, reps));
    printf("bench_pipeline_ms_median=%.4f\n", median_of(t_pipe, reps));
    printf("bench_bruteforce_ms=%.4f\n", min_of(t_brute, reps));
    printf("bench_bruteforce_ms_median=%.4f\n", median_of(t_brute, reps));
    printf("bench_speedup=%.1f\n", min_of(t_brute, reps) / min_of(t_pipe, reps));
    printf("bench_contacts_mismatch=%lld\n", st.mismatch);
    printf("bench_broad_missed=%lld\n", st.broad_missed);
    printf("bench_discrete_missed=%lld\n", st.discrete_missed);
    printf("bench_box_reject=%lld\n", st.box_reject);

    world_free(&w);
    broad_free(&bd);
    free(pipe.items);
    free(brute.items);
    return ok;
}

static int stress(int frames) {
    Broad bd;
    ContactList pipe = {0}, brute = {0};
    Stats st;
    int bad = 0;
    memset(&bd, 0, sizeof(bd));
    memset(&st, 0, sizeof(st));

    for (int f = 0; f < frames; f++) {
        int np = 120 + (f * 37) % 380;
        int no = 20 + (f * 13) % 80;
        double field = 40.0 + (double)((f * 7) % 160);   /* dense and sparse */
        uint64_t seed = 0x123456789ABCDEF0ull + (uint64_t)f * 0x9E3779B97F4A7C15ull;
        World w = world_random(seed, np, no, field, 5.0, 220.0);
        if (!solve_frame(&w, &bd, &pipe, &brute, &st)) bad++;
        world_free(&w);
    }

    printf("stress_frames=%d\n", frames);
    printf("stress_all_pairs=%lld\n", st.pairs);
    printf("stress_broad_candidates=%lld\n", st.candidates);
    printf("stress_contacts=%lld\n", st.contacts);
    printf("stress_contacts_mismatch=%lld\n", st.mismatch);
    printf("stress_broad_missed=%lld\n", st.broad_missed);
    printf("stress_discrete_missed=%lld\n", st.discrete_missed);
    printf("stress_box_reject=%lld\n", st.box_reject);

    broad_free(&bd);
    free(pipe.items);
    free(brute.items);
    return bad == 0 && st.mismatch == 0 && st.broad_missed == 0 &&
           st.box_reject == 0;
}

int main(void) {
    int bench_ok, stress_ok;

    micro_tests();
    printf("micro_ok=%d\n", micro_ok);

    bench_ok = benchmark(BENCH_PARTICLES, BENCH_OBSTACLES, 5);
    printf("bench_ok=%d\n", bench_ok);

    stress_ok = stress(40);
    printf("stress_ok=%d\n", stress_ok);

    printf("all_ok=%d\n", micro_ok && bench_ok && stress_ok);
    return (micro_ok && bench_ok && stress_ok) ? 0 : 1;
}
