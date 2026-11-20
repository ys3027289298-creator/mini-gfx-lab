
/* tri-budget: three-view (main/shadow/reflection) triangle budget arbitration.
   Each frame a greedy marginal-utility pass plans per-view quality tiers
   (best weighted-error-reduction per triangle wins the budget); planned
   downgrades apply instantly while upgrades must persist for a dwell time,
   so transitions don't pop; a safety clamp coarsens lowest-priority views
   if the cap would be exceeded. Quality is compared against a fixed
   60/25/15 split. The arbiter itself is O(VIEWS*TIERS) per frame. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <math.h>
#include <time.h>

#define VIEWS 3
#define TIERS 5
#define FRAMES 240
#define CAP 1000000L
#define UPGRADE_DELAY 6  /* frames a finer plan must persist before applying */

enum { V_MAIN, V_SHADOW, V_REFL };

typedef struct {
    const char* name;
    float weight;        /* view importance */
    int   tris[TIERS];   /* triangle cost per tier, finest -> coarsest */
    float err[TIERS];    /* screen-space error per tier, px */
} View;

static const View V[VIEWS] = {
    { "main",   1.00f, {600000, 300000, 150000, 60000, 20000}, {0.5f, 1.0f, 2.0f, 4.0f,  8.0f} },
    { "shadow", 0.50f, {400000, 200000,  80000, 30000, 10000}, {1.0f, 2.0f, 4.0f, 8.0f, 16.0f} },
    { "refl",   0.35f, {300000, 150000,  60000, 25000,  8000}, {1.0f, 2.0f, 4.0f, 8.0f, 16.0f} },
};

/* animated scene complexity per view (visible casters / reflectees change);
   ranges dip below the static shares so redistribution actually matters */
static float demand_mult(int v, int f) {
    switch (v) {
        case V_MAIN:   return 0.9f + 0.7f * sinf(f * 0.050f);
        case V_SHADOW: return 1.0f + 0.9f * sinf(f * 0.037f + 2.0f);
        default:       return 0.7f + 0.6f * sinf(f * 0.061f + 4.0f);
    }
}

typedef struct {
    int   tier[VIEWS];
    int   dwell[VIEWS];  /* frames the plan has continuously permitted an upgrade */
    int   switches;
    int   fallbacks;     /* frames where the safety clamp engaged */
    long  ops;           /* arbitration work counter */
} Arbiter;

/* safety clamp: never exceed the cap, coarsen lowest priority first */
static void enforce_cap(Arbiter* a, const float m[VIEWS]) {
    static const int order[VIEWS] = { V_REFL, V_SHADOW, V_MAIN };
    long spent = 0;
    for (int v = 0; v < VIEWS; v++) spent += (long)(V[v].tris[a->tier[v]] * m[v]);
    int fb = 0;
    while (spent > CAP) {
        int done = 1;
        for (int i = 0; i < VIEWS; i++) {
            int v = order[i];
            if (a->tier[v] < TIERS - 1) {
                spent -= (long)(V[v].tris[a->tier[v]] * m[v]);
                a->tier[v]++;
                a->switches++;
                spent += (long)(V[v].tris[a->tier[v]] * m[v]);
                a->ops += 3;
                done = 0;
                fb = 1;
                break;
            }
        }
        if (done) break; /* all coarsest: cap unreachable, give up */
    }
    if (fb) a->fallbacks++;
}

static void arbitrate(Arbiter* a, const float m[VIEWS]) {
    float c[VIEWS][TIERS];
    for (int v = 0; v < VIEWS; v++) {
        for (int t = 0; t < TIERS; t++) c[v][t] = V[v].tris[t] * m[v];
        a->ops += TIERS;
    }
    /* greedy marginal-utility plan: start all views coarsest, then repeatedly
       take the upgrade with the best weighted-error-reduction per triangle
       that still fits the cap */
    int p[VIEWS] = { TIERS - 1, TIERS - 1, TIERS - 1 };
    float used = c[0][TIERS - 1] + c[1][TIERS - 1] + c[2][TIERS - 1];
    for (;;) {
        int best = -1;
        float best_pri = 0.0f, best_cost = 0.0f;
        for (int v = 0; v < VIEWS; v++) {
            a->ops += 2;
            if (p[v] == 0) continue;
            float dtris = c[v][p[v] - 1] - c[v][p[v]];
            float gain = V[v].weight * (V[v].err[p[v]] - V[v].err[p[v] - 1]);
            float pri = gain / dtris;
            if (used + dtris <= (float)CAP && pri > best_pri) {
                best_pri = pri;
                best_cost = dtris;
                best = v;
            }
        }
        if (best < 0) break;
        used += best_cost;
        p[best]--;
    }
    /* apply plan: downgrades are immediate (budget safety), upgrades must
       persist for UPGRADE_DELAY frames so quality doesn't pop */
    for (int v = 0; v < VIEWS; v++) {
        a->ops += 2;
        if (p[v] >= a->tier[v]) {
            a->dwell[v] = 0;
            if (p[v] > a->tier[v]) { a->tier[v] = p[v]; a->switches++; }
        } else if (++a->dwell[v] >= UPGRADE_DELAY) {
            a->tier[v] = p[v];
            a->switches++;
            a->dwell[v] = 0;
        }
    }
    enforce_cap(a, m);
}

/* baseline: fixed 60/25/15 split, no redistribution, no smoothing */
static const float FIXED[VIEWS] = { 0.60f, 0.25f, 0.15f };

typedef struct { int tier[VIEWS]; int switches; } Fixed;

static void fixed_step(Fixed* fx, const float m[VIEWS]) {
    for (int v = 0; v < VIEWS; v++) {
        float share = (float)CAP * FIXED[v];
        int t = TIERS - 1;
        for (int k = 0; k < TIERS; k++)
            if (V[v].tris[k] * m[v] <= share) { t = k; break; }
        if (t != fx->tier[v]) { fx->switches++; fx->tier[v] = t; }
    }
}

int main(void) {
    Arbiter a = {0};
    Fixed fx = {0};
    for (int v = 0; v < VIEWS; v++) { a.tier[v] = TIERS - 1; fx.tier[v] = TIERS - 1; }

    double err_a = 0.0, err_f = 0.0;
    long max_a = 0, max_f = 0;
    for (int f = 0; f < FRAMES; f++) {
        float m[VIEWS];
        for (int v = 0; v < VIEWS; v++) m[v] = demand_mult(v, f);
        arbitrate(&a, m);
        fixed_step(&fx, m);
        long ta = 0, tf = 0;
        for (int v = 0; v < VIEWS; v++) {
            ta += (long)(V[v].tris[a.tier[v]] * m[v]);
            tf += (long)(V[v].tris[fx.tier[v]] * m[v]);
            err_a += V[v].weight * V[v].err[a.tier[v]];
            err_f += V[v].weight * V[v].err[fx.tier[v]];
        }
        if (ta > max_a) max_a = ta;
        if (tf > max_f) max_f = tf;
    }
    err_a /= FRAMES;
    err_f /= FRAMES;

    /* clamp unit check: an over-budget state must be coarsened under the cap */
    Arbiter c = {0}; /* all tiers finest */
    float big[VIEWS] = { 2.0f, 2.0f, 2.0f };
    enforce_cap(&c, big);
    long spent = 0;
    for (int v = 0; v < VIEWS; v++) spent += (long)(V[v].tris[c.tier[v]] * big[v]);
    int clamp_ok = spent <= CAP && c.fallbacks == 1;

    /* overhead: time the arbiter in isolation */
    Arbiter b = {0};
    for (int v = 0; v < VIEWS; v++) b.tier[v] = TIERS - 1;
    float m[VIEWS] = { 1.3f, 1.8f, 1.1f };
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    const int iters = 200000;
    for (int i = 0; i < iters; i++) {
        m[0] = 1.0f + 0.5f * sinf(i * 0.01f);
        arbitrate(&b, m);
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double ns = ((double)(t1.tv_sec - t0.tv_sec) * 1e9 +
                 (double)(t1.tv_nsec - t0.tv_nsec)) / iters;

    printf("arbiter: werr=%.3f switches=%d fallbacks=%d max_tris=%ld\n",
           err_a, a.switches, a.fallbacks, max_a);
    printf("fixed:   werr=%.3f switches=%d max_tris=%ld\n",
           err_f, fx.switches, max_f);
    printf("overhead: %.0f ns/call, %ld ops/frame\n", ns, a.ops / FRAMES);

    int cap_ok = max_a <= CAP;
    int quality_better = err_a < err_f;
    int smoother = a.switches < fx.switches;
    int overhead_ok = ns < 1000.0; /* << 1us vs a 16.6ms frame */
    printf("cap_ok=%d clamp_ok=%d quality_better=%d smoother=%d overhead_ok=%d\n",
           cap_ok, clamp_ok, quality_better, smoother, overhead_ok);
    return !(cap_ok && clamp_ok && quality_better && smoother && overhead_ok);
}
