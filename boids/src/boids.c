#define _POSIX_C_SOURCE 199309L

/* mini-boids: Reynolds flocking on a toroidal field.
   Three rules — separation / alignment / cohesion — with tunable weights,
   spatial-hash neighbor lookup (cell size = perception radius, 3x3 scan),
   brute-force parity checks, a parameter sweep over three emergent regimes
   (dispersed gas / cohesive milling / polarized stream), and a per-frame
   time budget. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define N            1000
#define WORLD        750.0f
#define COLS         30
#define ROWS         30
#define CELLS        (COLS * ROWS)
#define CELL         (WORLD / (float)COLS)   /* 25.0 == perception radius */
#define R_NEIGH      CELL
#define STEPS        2400                    /* 40 s at 60 fps */
#define BENCH_FRAMES 300
#define WINDOW       90                      /* last 1.5 s: settled-state metrics */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { float x, y, vx, vy; } Boid;

typedef struct {
    float sep_w;                 /* separation weight (u/s^2) */
    float ali_w;                 /* alignment velocity-match rate (1/s) */
    float coh_w;                 /* cohesion weight (u/s^2) */
    float sep_r;                 /* separation radius (<= R_NEIGH) */
    float coh_sat;               /* distance where cohesion force saturates */
    float min_speed, max_speed;  /* u/s */
    float max_force;             /* total acceleration clamp, u/s^2 */
    float noise;                 /* random heading jitter, u/s^2; 0 = off */
} Params;

typedef struct {
    int count[CELLS];
    int start[CELLS + 1];
    int cursor[CELLS];
    int order[N];
    long checks;                 /* candidate distance tests accumulated */
    long frame;                  /* integration step, seeds deterministic noise */
    /* per-boid force accumulators, sized once so a step stays allocation-free */
    int nn[N], ns[N];
    float svx[N], svy[N], sdx[N], sdy[N], sepx[N], sepy[N];
} Grid;

/* ---------- deterministic rng, toroidal topology ---------- */

static uint32_t rng_u32(uint32_t* s) {
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return x;
}

static float rng_f(uint32_t* s) {          /* [0, 1) */
    return (rng_u32(s) >> 8) * (1.0f / 16777216.0f);
}

/* deterministic per-boid, per-frame noise angle in [0, 2pi): a pure function
   of (index, frame, world seed), so hash and brute-force paths draw identical
   jitter, while the jitter stays independent of the initial configuration.
   The seed is set once via noise_seed() before a run. */
static uint32_t g_noise_seed = 0;

static void noise_seed(uint32_t s) { g_noise_seed = s ? s : 1u; }

static float noise_angle(int idx, long frame) {
    uint32_t h = (uint32_t)idx * 0x9e3779b9u
               + (uint32_t)frame * 0x85ebca6bu
               + g_noise_seed * 0xc2b2ae35u;
    h ^= h >> 16; h *= 0x7feb352du; h ^= h >> 15;
    return (h >> 8) * (1.0f / 16777216.0f) * 2.0f * (float)M_PI;
}

static float wrapf(float v) {
    v = fmodf(v, WORLD);
    if (v < 0.0f) v += WORLD;
    return v;
}

static float dwrap(float d) {              /* minimum-image delta */
    if (d >  WORLD * 0.5f) d -= WORLD;
    if (d < -WORLD * 0.5f) d += WORLD;
    return d;
}

static void world_seed(Boid* b, uint32_t seed) {
    noise_seed(seed);
    for (int i = 0; i < N; i++) {
        b[i].x = rng_f(&seed) * WORLD;
        b[i].y = rng_f(&seed) * WORLD;
        float a = rng_f(&seed) * 2.0f * (float)M_PI;
        float sp = 30.0f;
        b[i].vx = cosf(a) * sp;
        b[i].vy = sinf(a) * sp;
    }
}

/* ---------- spatial hash: counting sort into grid cells ---------- */

static int cell_of(const Boid* b) {
    int cx = (int)(b->x / CELL);
    int cy = (int)(b->y / CELL);
    if (cx >= COLS) cx = COLS - 1;
    if (cy >= ROWS) cy = ROWS - 1;
    return cy * COLS + cx;
}

static void grid_build(Grid* g, const Boid* b) {
    memset(g->count, 0, sizeof(g->count));
    for (int i = 0; i < N; i++) g->count[cell_of(&b[i])]++;
    g->start[0] = 0;
    for (int c = 0; c < CELLS; c++) g->start[c + 1] = g->start[c] + g->count[c];
    memcpy(g->cursor, g->start, sizeof(g->cursor));
    for (int i = 0; i < N; i++) {
        int c = cell_of(&b[i]);
        g->order[g->cursor[c]++] = i;
    }
}

/* ---------- shared rule math ----------
   Hybrid Reynolds steerings:
     alignment  = ali_w * (mean neighbor velocity - own velocity), a velocity
                  difference so it matches both heading and speed (1/s gain);
     cohesion   = coh_w * vector toward neighbor centroid (a spring, 1/s^2;
                  grows with distance so it keeps compressing a forming cluster);
     separation = sep_w * mean inverse-distance push direction (u/s^2).
   The summed acceleration is clamped to max_force. */
static void steer_and_integrate(Boid* b, const Params* p, float dt, int idx,
                                long frame,
                                int nn, float svx, float svy,
                                float sdx, float sdy,
                                int ns, float sepx, float sepy) {
    float ax = 0.0f, ay = 0.0f;
    if (nn > 0) {
        ax += p->ali_w * (svx / nn - b->vx);
        ay += p->ali_w * (svy / nn - b->vy);
        /* saturated spring toward the neighbor centroid: force magnitude grows
           with distance up to coh_sat, then caps — long-range it pulls the
           flock together, near the core it only steers without crushing the
           orbital motion that produces milling */
        float acx = sdx / nn, acy = sdy / nn;
        float dist = sqrtf(acx * acx + acy * acy);
        if (dist > 0.0f) {
            float k = p->coh_w * fminf(dist, p->coh_sat) / dist;
            ax += k * acx; ay += k * acy;
        }
    }
    if (ns > 0) {
        float ux = sepx / ns, uy = sepy / ns;
        float ls = sqrtf(ux * ux + uy * uy);
        if (ls > 0.0f) { ax += p->sep_w * ux / ls; ay += p->sep_w * uy / ls; }
    }
    if (p->noise > 0.0f) {
        float na = noise_angle(idx, frame);
        ax += p->noise * cosf(na);
        ay += p->noise * sinf(na);
    }
    float f = sqrtf(ax * ax + ay * ay);
    if (f > p->max_force && f > 0.0f) {
        ax *= p->max_force / f;
        ay *= p->max_force / f;
    }
    b->vx += ax * dt;
    b->vy += ay * dt;
    float sp = sqrtf(b->vx * b->vx + b->vy * b->vy);
    if (sp > p->max_speed) {
        b->vx *= p->max_speed / sp; b->vy *= p->max_speed / sp;
    } else if (sp < p->min_speed && sp > 0.0f) {
        b->vx *= p->min_speed / sp; b->vy *= p->min_speed / sp;
    }
    b->x = wrapf(b->x + b->vx * dt);
    b->y = wrapf(b->y + b->vy * dt);
}

#define ACCUMULATE(j, i) do {                                              \
    float dx = dwrap(b[j].x - b[i].x);                                     \
    float dy = dwrap(b[j].y - b[i].y);                                     \
    float d2 = dx * dx + dy * dy;                                          \
    if (d2 < r2) {                                                         \
        nn++; svx += b[j].vx; svy += b[j].vy; sdx += dx; sdy += dy;       \
        if (d2 < sr2) {                                                    \
            float inv = 1.0f / (sqrtf(d2) + 1e-6f);                        \
            ns++; sepx -= dx * inv; sepy -= dy * inv;                      \
        }                                                                  \
    }                                                                      \
} while (0)

/* Jacobi update: build the hash from current positions, accumulate every
   boid's steering from that frozen snapshot, then integrate all boids. This
   matches the brute-force path exactly, so the two stay bit-comparable. */
static void step_hash(Boid* b, Grid* g, const Params* p, float dt, int* neigh_per) {
    grid_build(g, b);
    float r2 = R_NEIGH * R_NEIGH;
    float sr2 = p->sep_r * p->sep_r;
    for (int i = 0; i < N; i++) {
        int cx = (int)(b[i].x / CELL); if (cx >= COLS) cx = COLS - 1;
        int cy = (int)(b[i].y / CELL); if (cy >= ROWS) cy = ROWS - 1;
        int nn = 0, ns = 0;
        float svx = 0, svy = 0, sdx = 0, sdy = 0, sepx = 0, sepy = 0;
        for (int oy = -1; oy <= 1; oy++) {
            int ccy = (cy + oy + ROWS) % ROWS;
            for (int ox = -1; ox <= 1; ox++) {
                int ccx = (cx + ox + COLS) % COLS;
                int c = ccy * COLS + ccx;
                for (int k = g->start[c]; k < g->start[c + 1]; k++) {
                    int j = g->order[k];
                    g->checks++;
                    if (j == i) continue;
                    ACCUMULATE(j, i);
                }
            }
        }
        g->nn[i] = nn; g->ns[i] = ns;
        g->svx[i] = svx; g->svy[i] = svy; g->sdx[i] = sdx; g->sdy[i] = sdy;
        g->sepx[i] = sepx; g->sepy[i] = sepy;
        if (neigh_per) neigh_per[i] = nn;
    }
    for (int i = 0; i < N; i++)
        steer_and_integrate(&b[i], p, dt, i, g->frame,
                            g->nn[i], g->svx[i], g->svy[i], g->sdx[i], g->sdy[i],
                            g->ns[i], g->sepx[i], g->sepy[i]);
    g->frame++;
}

static long step_brute(Boid* b, const Params* p, float dt, long frame) {
    long checks = 0;
    float r2 = R_NEIGH * R_NEIGH;
    float sr2 = p->sep_r * p->sep_r;
    static int bnn[N], bns[N];
    static float bsvx[N], bsvy[N], bsdx[N], bsdy[N], bsepx[N], bsepy[N];
    for (int i = 0; i < N; i++) {
        int nn = 0, ns = 0;
        float svx = 0, svy = 0, sdx = 0, sdy = 0, sepx = 0, sepy = 0;
        for (int j = 0; j < N; j++) {
            if (j == i) continue;
            checks++;
            ACCUMULATE(j, i);
        }
        bnn[i] = nn; bns[i] = ns;
        bsvx[i] = svx; bsvy[i] = svy; bsdx[i] = sdx; bsdy[i] = sdy;
        bsepx[i] = sepx; bsepy[i] = sepy;
    }
    for (int i = 0; i < N; i++)
        steer_and_integrate(&b[i], p, dt, i, frame,
                            bnn[i], bsvx[i], bsvy[i], bsdx[i], bsdy[i],
                            bns[i], bsepx[i], bsepy[i]);
    return checks;
}

/* ---------- emergent-behavior metrics ---------- */

typedef struct { double pol, dens, mill, radius; } Metrics;

/* polarization: normalized mean velocity (0 = disordered, 1 = fully aligned).
   mill: mean local angular momentum, each boid orbiting its own neighbor
   centroid (robust on the torus and under cluster translation).
   radius: mean distance to the population centroid, torus-corrected. */
static Metrics measure(const Boid* b, const int* neigh_per, const Grid* g) {
    double vx = 0, vy = 0;
    for (int i = 0; i < N; i++) {
        float sp = sqrtf(b[i].vx * b[i].vx + b[i].vy * b[i].vy);
        vx += b[i].vx / sp; vy += b[i].vy / sp;
    }
    /* torus centroid via circular means */
    double cx = 0, sx = 0, cy = 0, sy = 0;
    for (int i = 0; i < N; i++) {
        cx += cosf(2.0f * (float)M_PI * b[i].x / WORLD);
        sx += sinf(2.0f * (float)M_PI * b[i].x / WORLD);
        cy += cosf(2.0f * (float)M_PI * b[i].y / WORLD);
        sy += sinf(2.0f * (float)M_PI * b[i].y / WORLD);
    }
    float cent_x = atan2f((float)sx, (float)cx) * WORLD / (2.0f * (float)M_PI);
    if (cent_x < 0.0f) cent_x += WORLD;
    float cent_y = atan2f((float)sy, (float)cy) * WORLD / (2.0f * (float)M_PI);
    if (cent_y < 0.0f) cent_y += WORLD;

    double L_sum = 0, L_cnt = 0, rmean = 0, dens = 0;
    float r2 = R_NEIGH * R_NEIGH;
    for (int i = 0; i < N; i++) {
        int cxg = (int)(b[i].x / CELL); if (cxg >= COLS) cxg = COLS - 1;
        int cyg = (int)(b[i].y / CELL); if (cyg >= ROWS) cyg = ROWS - 1;
        double mx = 0, my = 0, L = 0; int nn = 0;
        for (int oy = -1; oy <= 1; oy++) {
            int ccy = (cyg + oy + ROWS) % ROWS;
            for (int ox = -1; ox <= 1; ox++) {
                int ccx = (cxg + ox + COLS) % COLS;
                int c = ccy * COLS + ccx;
                for (int k = g->start[c]; k < g->start[c + 1]; k++) {
                    int j = g->order[k];
                    if (j == i) continue;
                    float dx = dwrap(b[j].x - b[i].x);
                    float dy = dwrap(b[j].y - b[i].y);
                    if (dx * dx + dy * dy >= r2) continue;
                    mx += dx; my += dy; nn++;
                }
            }
        }
        if (nn >= 6) {
            mx /= nn; my /= nn;            /* neighbor centroid relative to i */
            for (int oy = -1; oy <= 1; oy++) {
                int ccy = (cyg + oy + ROWS) % ROWS;
                for (int ox = -1; ox <= 1; ox++) {
                    int ccx = (cxg + ox + COLS) % COLS;
                    int c = ccy * COLS + ccx;
                    for (int k = g->start[c]; k < g->start[c + 1]; k++) {
                        int j = g->order[k];
                        if (j == i) continue;
                        float dx = dwrap(b[j].x - b[i].x);
                        float dy = dwrap(b[j].y - b[i].y);
                        if (dx * dx + dy * dy >= r2) continue;
                        float rx = dx - (float)mx, ry = dy - (float)my;
                        L += (double)rx * b[j].vy - (double)ry * b[j].vx;
                    }
                }
            }
            /* normalize by mean radius * mean speed, summed over neighbors:
               0 = no rotation, larger = coherent local orbiting */
            double rc = 0, vs = 0;
            for (int oy = -1; oy <= 1; oy++) {
                int ccy = (cyg + oy + ROWS) % ROWS;
                for (int ox = -1; ox <= 1; ox++) {
                    int ccx = (cxg + ox + COLS) % COLS;
                    int c = ccy * COLS + ccx;
                    for (int k = g->start[c]; k < g->start[c + 1]; k++) {
                        int j = g->order[k];
                        if (j == i) continue;
                        float dx = dwrap(b[j].x - b[i].x);
                        float dy = dwrap(b[j].y - b[i].y);
                        if (dx * dx + dy * dy >= r2) continue;
                        float rx = dx - (float)mx, ry = dy - (float)my;
                        rc += sqrtf(rx * rx + ry * ry);
                        vs += sqrtf(b[j].vx * b[j].vx + b[j].vy * b[j].vy);
                    }
                }
            }
            L_sum += fabs(L) * nn / (rc * vs + 1e-12);
            L_cnt++;
        }
        float rx = dwrap(b[i].x - cent_x);
        float ry = dwrap(b[i].y - cent_y);
        rmean += sqrtf(rx * rx + ry * ry);
        dens += neigh_per[i];
    }
    Metrics m;
    m.pol = sqrt(vx * vx + vy * vy) / N;
    m.dens = dens / N;
    m.radius = (rmean / N) / WORLD;
    m.mill = L_cnt > 0 ? L_sum / L_cnt : 0.0;
    return m;
}

/* ---------- tests ---------- */

/* baseline parameters used by the parity and performance tests */
static const Params BASE = { 40.0f, 3.0f, 30.0f, 12.0f, 10.0f, 15.0f, 50.0f, 400.0f, 0.0f };

static int test_neighbor_lookup(const Boid* ref, Grid* g) {
    Boid b[N]; memcpy(b, ref, sizeof(b));
    grid_build(g, b);
    float r2 = R_NEIGH * R_NEIGH;
    int bad = 0;
    for (int i = 0; i < N; i++) {
        int cx = (int)(b[i].x / CELL); if (cx >= COLS) cx = COLS - 1;
        int cy = (int)(b[i].y / CELL); if (cy >= ROWS) cy = ROWS - 1;
        int hash_n = 0;
        for (int oy = -1; oy <= 1; oy++) {
            int ccy = (cy + oy + ROWS) % ROWS;
            for (int ox = -1; ox <= 1; ox++) {
                int ccx = (cx + ox + COLS) % COLS;
                int c = ccy * COLS + ccx;
                for (int k = g->start[c]; k < g->start[c + 1]; k++) {
                    int j = g->order[k];
                    if (j == i) continue;
                    float dx = dwrap(b[j].x - b[i].x);
                    float dy = dwrap(b[j].y - b[i].y);
                    if (dx * dx + dy * dy < r2) hash_n++;
                }
            }
        }
        int brute_n = 0;
        for (int j = 0; j < N; j++) {
            if (j == i) continue;
            float dx = dwrap(b[j].x - b[i].x);
            float dy = dwrap(b[j].y - b[i].y);
            if (dx * dx + dy * dy < r2) brute_n++;
        }
        if (hash_n != brute_n) bad++;
    }
    printf("[1] neighbor lookup: hash vs brute counts mismatch=%d (all %d boids)\n",
           bad, N);
    return bad == 0;
}

static float parity_run(const Boid* ref, Grid* g, const Params* p) {
    Boid a[N], c[N]; memcpy(a, ref, sizeof(a)); memcpy(c, ref, sizeof(c));
    g->frame = 0;
    step_hash(a, g, p, 1.0f / 60.0f, NULL);
    step_brute(c, p, 1.0f / 60.0f, 0);
    float worst = 0.0f;
    for (int i = 0; i < N; i++) {
        float d = fabsf(a[i].x - c[i].x); if (d > worst) worst = d;
        d = fabsf(a[i].y - c[i].y); if (d > worst) worst = d;
        d = fabsf(a[i].vx - c[i].vx); if (d > worst) worst = d;
        d = fabsf(a[i].vy - c[i].vy); if (d > worst) worst = d;
    }
    return worst;
}

static int test_integrator_parity(const Boid* ref, Grid* g) {
    float w0 = parity_run(ref, g, &BASE);
    Params noisy = BASE; noisy.noise = 160.0f;   /* exercise deterministic jitter */
    float w1 = parity_run(ref, g, &noisy);
    printf("[2] integrator parity: max |hash - brute| after 1 frame "
           "(no noise %.2e, noise %.2e)\n", w0, w1);
    return w0 < 1e-3f && w1 < 1e-3f;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static int test_performance(const Boid* ref, Grid* g, double* ratio_out) {
    Boid b[N]; memcpy(b, ref, sizeof(b));
    int neigh[N];
    for (int k = 0; k < 30; k++) step_hash(b, g, &BASE, 1.0f / 60.0f, neigh); /* warmup */
    g->checks = 0;
    double sum = 0, worst = 0;
    for (int k = 0; k < BENCH_FRAMES; k++) {
        double t0 = now_ms();
        step_hash(b, g, &BASE, 1.0f / 60.0f, neigh);
        double dt = now_ms() - t0;
        sum += dt;
        if (dt > worst) worst = dt;
    }
    long hash_per_frame = g->checks / BENCH_FRAMES;
    long brute_per_frame = (long)N * (N - 1);
    double ratio = (double)brute_per_frame / (double)hash_per_frame;
    *ratio_out = ratio;
    printf("[3] perf %d boids x %d frames: mean %.3f ms, max %.3f ms (budget 3 ms)\n",
           N, BENCH_FRAMES, sum / BENCH_FRAMES, worst);
    printf("    neighbor checks/frame: hash=%ld brute=%ld speedup=%.1fx (need >=10x)\n",
           hash_per_frame, brute_per_frame, ratio);
    return worst < 3.0 && ratio >= 10.0;
}

typedef struct { const char* name; Params p; } Preset;

static int test_sweep(const Boid* ref, Grid* g, double* min_ratio) {
    const Preset presets[3] = {
        /*                 sep    ali   coh  sep_r csat vmin vmax fmax noise */
        { "gas   (separation dom)", { 200.0f, 0.10f, 5.0f, 12.0f, 10.0f, 15, 50, 400, 160.0f } },
        { "mill  (cohesion dom)  ", {   6.0f, 0.10f, 4.0f, 12.0f, 10.0f, 15, 50, 400,   0.0f } },
        { "flock (alignment dom) ", {  90.0f, 9.00f, 0.3f, 12.0f,  8.0f, 15, 50, 500, 110.0f } },
    };
    Metrics got[3];
    *min_ratio = 1e300;
    printf("[4] parameter sweep: %d frames, metrics averaged over last %d\n",
           STEPS, WINDOW);
    printf("    %-24s %6s %6s %6s %7s %9s\n",
           "preset", "pol", "dens", "mill", "radius", "hash x");
    for (int q = 0; q < 3; q++) {
        Boid b[N]; memcpy(b, ref, sizeof(b));
        int neigh[N];
        double pol = 0, dens = 0, mill = 0, radius = 0;
        /* each preset starts from frame 0 so its deterministic noise phase is
           independent of how many presets ran before it */
        g->checks = 0;
        g->frame = 0;
        for (int k = 0; k < STEPS; k++) {
            step_hash(b, g, &presets[q].p, 1.0f / 60.0f, neigh);
            if (k >= STEPS - WINDOW) {
                Metrics m = measure(b, neigh, g);
                pol += m.pol; dens += m.dens; mill += m.mill; radius += m.radius;
            }
        }
        got[q].pol = pol / WINDOW;
        got[q].dens = dens / WINDOW;
        got[q].mill = mill / WINDOW;
        got[q].radius = radius / WINDOW;
        double ratio = (double)N * (N - 1) / (double)(g->checks / STEPS);
        if (ratio < *min_ratio) *min_ratio = ratio;
        printf("    %-24s %6.3f %6.2f %6.3f %7.3f %8.1fx\n",
               presets[q].name, got[q].pol, got[q].dens, got[q].mill,
               got[q].radius, ratio);
    }
    int ok = 1;
    /* 三状态预期（阈值取自五种子扫描的最差值并留裕量）：
       flock：高极化且方向一致，不打旋；mill：低极化、局域密度远高于弥散基线、
       远高于弥散基线、涡旋显著高于定向流；gas：保持弥散、低极化 */
    int c1 = got[2].pol > 0.60 && got[2].pol > 3.0 * got[0].pol
             && got[2].pol > 2.0 * got[1].pol;
    int c2 = got[1].dens > 4.0 * got[0].dens && got[1].pol < 0.40;
    int c3 = got[1].mill > 3.0 * got[2].mill;
    printf("    polarized stream (flock pol>0.6, dominates)      : %s\n", c1 ? "PASS" : "FAIL");
    printf("    cohesive cluster   (mill dens>4x gas, pol<0.40) : %s\n", c2 ? "PASS" : "FAIL");
    printf("    rotating swarm     (mill spin >3x flock)         : %s\n", c3 ? "PASS" : "FAIL");
    ok = c1 && c2 && c3;
    return ok;
}

static int tune(int argc, char** argv, Grid* g, Boid* ref) {
    (void)argc;
    Params p = BASE;
    p.sep_w = (float)atof(argv[1]);
    p.ali_w = (float)atof(argv[2]);
    p.coh_w = (float)atof(argv[3]);
    p.max_force = (float)atof(argv[4]);
    int an = 5;
    if (argc > an && argv[an][0] != 's') { p.noise = (float)atof(argv[an]); an++; }
    if (argc > an && argv[an][0] != 's') { p.coh_sat = (float)atof(argv[an]); an++; }
    int series = argv[argc - 1][0] == 's';
    uint32_t tseed = 42;
    const char* tenv = getenv("BOIDS_SEED");
    if (tenv) tseed = (uint32_t)strtoul(tenv, NULL, 10);
    world_seed(ref, tseed);
    Boid b[N]; memcpy(b, ref, sizeof(b));
    int neigh[N];
    double pol = 0, dens = 0, mill = 0, radius = 0;
    for (int k = 0; k < STEPS; k++) {
        step_hash(b, g, &p, 1.0f / 60.0f, neigh);
        if (series && (k % 60 == 59)) {
            Metrics m = measure(b, neigh, g);
            printf("  t=%4.1fs pol=%.3f dens=%6.2f mill=%.3f radius=%.3f\n",
                   (k + 1) / 60.0, m.pol, m.dens, m.mill, m.radius);
        }
        if (k >= STEPS - WINDOW) {
            Metrics m = measure(b, neigh, g);
            pol += m.pol; dens += m.dens; mill += m.mill; radius += m.radius;
        }
    }
    printf("sep=%6.1f ali=%6.1f coh=%6.1f fmax=%5.0f | pol=%.3f dens=%6.2f mill=%.3f radius=%.3f\n",
           p.sep_w, p.ali_w, p.coh_w, p.max_force,
           pol / WINDOW, dens / WINDOW, mill / WINDOW, radius / WINDOW);
    return 0;
}

int main(int argc, char** argv) {
    Grid* g = calloc(1, sizeof(Grid));
    if (!g) { printf("alloc failed\n"); return 1; }
    Boid* ref = malloc(sizeof(Boid) * N);
    if (!ref) { printf("alloc failed\n"); return 1; }
    world_seed(ref, 42);
    if (argc >= 5 && argc <= 8) return tune(argc, argv, g, ref);

    uint32_t seed = 42;
    const char* env = getenv("BOIDS_SEED");
    if (env) seed = (uint32_t)strtoul(env, NULL, 10);
    world_seed(ref, seed);

    int failures = 0;
    failures += !test_neighbor_lookup(ref, g);
    failures += !test_integrator_parity(ref, g);
    double bench_ratio;
    failures += !test_performance(ref, g, &bench_ratio);
    double sweep_ratio;
    failures += !test_sweep(ref, g, &sweep_ratio);
    if (sweep_ratio < 10.0) {
        printf("    sweep worst-case hash speedup %.1fx < 10x: FAIL\n", sweep_ratio);
        failures++;
    }
    printf("summary: %s (bench ratio %.1fx, sweep worst ratio %.1fx)\n",
           failures ? "FAIL" : "ALL PASS", bench_ratio, sweep_ratio);
    free(g); free(ref);
    return failures ? 1 : 0;
}
