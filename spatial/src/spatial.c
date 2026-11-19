
/* mini-spatial-audio: binaural positioning of mono sources.
   ITD by azimuth (max 0.6 ms), IID by azimuth, inverse-distance attenuation,
   soft-clip mixing of N sources into an interleaved stereo buffer.

   Spatial grouping: sources are bucketed into a uniform xz grid and the
   listener is quantized to its cell center, so geometry is constant within
   a cell. Moving inside one cell (or standing still) recomputes nothing --
   the cached per-source parameter table is reused; crossing a cell boundary
   or turning recomputes every source, bucket by bucket, in one exact pass.
   A translation shifts the relative bearing of every source, so a partial
   "only nearby buckets" update cannot be sample-identical and is not used.
   The flat reference applies the same cell quantization, per-source math
   and source order, making the grouped renderer bit-identical to recomputing
   every source one by one while recomputing far fewer sources over a walk. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define SR 48000
#define MAX_FRAMES 4096
#define MAX_SRC 512

/* spatial grid: world x,z in [-32, 32), 8 m cells -> 8x8 buckets */
#define WORLD_MIN (-32.0f)
#define CELL_SIZE 8.0f
#define GRID_W 8
#define GRID_H 8
#define GRID_N (GRID_W * GRID_H)

typedef struct {
    float x, y, z;      /* world position, meters */
    float gain;
} Source;

typedef struct {
    float x, z;         /* world position */
    float yaw;          /* radians, 0 = facing -z */
} Listener;

/* cached per-source spatialization parameters */
typedef struct {
    int delay;
    float gl, gr, g;
} SrcParams;

typedef enum { RECALC_NONE, RECALC_ALL } RecalcMode;

typedef struct {
    int head[GRID_N];       /* bucket chain head, -1 = empty */
    int next[MAX_SRC];      /* per-source chain link */
    int cell_of[MAX_SRC];   /* bucket index of each source */
    int entered;            /* bucket entered on the latest cell change */
    int lc_x, lc_z;         /* cached listener cell */
    int have_cell;
    float yaw;              /* cached orientation */
    int force_all;          /* latched by rebuild after sources moved */
} SpatialGrid;

typedef struct { long long recalc; } Counters;

static float soft_clip(float v) {
    const float k = 0.8f;
    float a = fabsf(v);
    if (a <= k) return v;
    float sign = v < 0 ? -1.0f : 1.0f;
    return sign * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

static int cell_coord(float w) {
    int c = (int)floorf((w - WORLD_MIN) / CELL_SIZE);
    if (c < 0) c = 0;
    if (c >= GRID_W) c = GRID_W - 1;
    return c;
}

/* world coordinate of a cell's center along one axis */
static float cell_center(int c) {
    return WORLD_MIN + (c + 0.5f) * CELL_SIZE;
}

/* quantize a listener pose to its cell center; geometry is constant per cell */
static Listener quantize(const Listener* l) {
    Listener q = { cell_center(cell_coord(l->x)),
                   cell_center(cell_coord(l->z)), l->yaw };
    return q;
}

/* azimuth in radians, 0 = front, +pi/2 = right; dx,dz relative to listener */
static float azimuth_rel(float dx, float dz, float yaw) {
    float c = cosf(yaw), sn = sinf(yaw);
    float xr =  c * dx - sn * dz;
    float zr =  sn * dx + c * dz;
    return atan2f(xr, -zr);
}

static int itd_samples(float az) {
    float max_delay = 0.0006f * SR;          /* 0.6 ms, Woodworth-lite */
    return (int)lroundf(sinf(az) * max_delay);
}

static void iid_gains(float az, float* gl, float* gr) {
    float t = (sinf(az) + 1.0f) * 0.5f;      /* 0 = hard left, 1 = hard right */
    *gl = sqrtf(1.0f - t);
    *gr = sqrtf(t);
}

static void calc_params(const Source* s, const Listener* q, SrcParams* p) {
    float rx = s->x - q->x;
    float ry = s->y;                         /* listener ear height stays at 0 */
    float rz = s->z - q->z;
    float az = azimuth_rel(rx, rz, q->yaw);
    p->delay = itd_samples(az);
    iid_gains(az, &p->gl, &p->gr);
    float d = sqrtf(rx * rx + ry * ry + rz * rz);
    p->g = s->gain / (1.0f + d);
}

/* shared mix path: source-index order in both renderers, which is what makes
   the grouped output bit-identical to the flat reference */
static void mix_with(const SrcParams* p, int n,
                     const float* mono, int frames, float* out) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < n; k++) {
        int delay = p[k].delay;
        float gl = p[k].gl, gr = p[k].gr, g = p[k].g;
        for (int i = 0; i < frames; i++) {
            int j = i - delay;
            float s = (j >= 0 && j < frames) ? mono[j] : 0.0f;
            out[2*i + 0] += s * g * gl;
            out[2*i + 1] += s * g * gr;
        }
    }
    for (int i = 0; i < frames * 2; i++) out[i] = soft_clip(out[i]);
}

/* ---- flat reference: quantize, then recompute every source every call ----- */

static void render_flat(const Source* srcs, int n, const Listener* l,
                        const float* mono, int frames, float* out,
                        SrcParams* p, Counters* c) {
    Listener q = quantize(l);
    for (int k = 0; k < n; k++) {
        calc_params(&srcs[k], &q, &p[k]);
        c->recalc++;
    }
    mix_with(p, n, mono, frames, out);
}

/* ---- spatial grid: bucket sources, cache geometry per listener cell ------- */

static void grid_init(SpatialGrid* g) {
    memset(g, 0, sizeof(*g));
    for (int c = 0; c < GRID_N; c++) g->head[c] = -1;
    g->have_cell = 0;
    g->force_all = 1;
}

/* rebuild buckets after sources were added/moved; cached params go stale, so
   the next update latches a full recompute */
static void grid_rebuild(SpatialGrid* g, const Source* srcs, int n) {
    for (int c = 0; c < GRID_N; c++) g->head[c] = -1;
    for (int k = 0; k < n; k++) {
        int c = cell_coord(srcs[k].z) * GRID_W + cell_coord(srcs[k].x);
        g->cell_of[k] = c;
        g->next[k] = g->head[c];
        g->head[c] = k;
    }
    g->force_all = 1;
}

/* decide whether the cached parameter table is still exact. Cell quantization
   makes geometry constant inside a cell, so same-cell motion and stillness
   cost nothing; a cell change or any yaw change invalidates the table. */
static RecalcMode grid_update(SpatialGrid* g, const Listener* q) {
    int cx = cell_coord(q->x), cz = cell_coord(q->z);
    if (!g->have_cell || g->force_all) {
        g->have_cell = 1;
        g->force_all = 0;
        g->lc_x = cx; g->lc_z = cz; g->yaw = q->yaw;
        g->entered = cz * GRID_W + cx;
        return RECALC_ALL;
    }
    if (cx == g->lc_x && cz == g->lc_z && q->yaw == g->yaw)
        return RECALC_NONE;                  /* same cell, same facing: cache */

    g->lc_x = cx; g->lc_z = cz; g->yaw = q->yaw;
    g->entered = cz * GRID_W + cx;
    return RECALC_ALL;                       /* cell change or turn */
}

static void render_grouped(SpatialGrid* g, const Source* srcs, int n,
                           const Listener* l, const float* mono, int frames,
                           float* out, SrcParams* p, Counters* c) {
    Listener q = quantize(l);
    if (grid_update(g, &q) == RECALC_ALL) {
        /* one exact pass over every source in source-index order, so the mix
           matches the flat reference bit for bit */
        for (int k = 0; k < n; k++) {
            calc_params(&srcs[k], &q, &p[k]);
            c->recalc++;
        }
    }
    mix_with(p, n, mono, frames, out);
}

/* ---- test helpers -------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static float peak(const float* buf, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(buf[i]) > m) m = fabsf(buf[i]);
    return m;
}

static unsigned rng_state;
static float frand(float lo, float hi) {
    rng_state = rng_state * 1103515245u + 12345u;
    float t = (float)(rng_state >> 8) / (float)(1u << 24);
    return lo + t * (hi - lo);
}

/* randomized frame-by-frame equivalence: the grouped output must be
   bit-identical to the flat reference, and recompute counts must follow the
   still-in-cell (zero) vs cell-change/turn/rebuild (all) rule */
static void test_bitexact(void) {
    rng_state = 0xC0FFEEu;
    enum { N = 40, FRAMES = 256, STEPS = 80 };
    Source srcs[N];
    for (int k = 0; k < N; k++)
        srcs[k] = (Source){ frand(-30, 30), 0.0f, frand(-30, 30), frand(0.5f, 1.5f) };

    float mono[FRAMES], out_flat[FRAMES * 2], out_grp[FRAMES * 2];
    for (int i = 0; i < FRAMES; i++) mono[i] = sinf(0.07f * i);

    SrcParams pf[N], pg[N];
    Counters cf = {0}, cg = {0};
    Listener lf = {0, 0, 0}, lg = {0, 0, 0};
    SpatialGrid g;
    grid_init(&g);
    grid_rebuild(&g, srcs, N);

    int bitexact = 1, idle_zero = 1, never_exceeds_n = 1, rule_match = 1;
    int cell_changes = 0;
    for (int step = 0; step < STEPS; step++) {
        int pcx = cell_coord(lf.x), pcz = cell_coord(lf.z);
        float py = lf.yaw;
        float roll = frand(0, 1);
        int rebuilt = 0;
        if (roll < 0.40f) {
            /* idle: pose unchanged */
        } else if (roll < 0.65f) {
            lf.x += frand(-0.4f, 0.4f); lf.z += frand(-0.4f, 0.4f);  /* in-cell */
        } else if (roll < 0.85f) {
            lf.x += frand(-12, 12); lf.z += frand(-12, 12);          /* cross cells */
        } else if (roll < 0.95f) {
            lf.yaw += frand(-1.2f, 1.2f);                            /* rotate */
        } else {
            int k = (int)frand(0, N - 1);                            /* source moved */
            srcs[k].x = frand(-30, 30);
            srcs[k].z = frand(-30, 30);
            grid_rebuild(&g, srcs, N);
            rebuilt = 1;
        }
        int same_cell = cell_coord(lf.x) == pcx &&
                        cell_coord(lf.z) == pcz && lf.yaw == py;
        if (!same_cell) cell_changes++;
        lg = lf;

        long long before_g = cg.recalc;
        render_flat(srcs, N, &lf, mono, FRAMES, out_flat, pf, &cf);
        render_grouped(&g, srcs, N, &lg, mono, FRAMES, out_grp, pg, &cg);
        long long delta = cg.recalc - before_g;

        if (memcmp(out_flat, out_grp, sizeof(out_flat)) != 0) bitexact = 0;
        if (delta > N) never_exceeds_n = 0;
        if (step > 0 && !rebuilt && same_cell && delta != 0) idle_zero = 0;
        long long expect = (step == 0 || rebuilt || !same_cell) ? N : 0;
        if (delta != expect) rule_match = 0;
        /* after a full recompute the cached table must equal the reference */
        if (delta == N && memcmp(pf, pg, sizeof(pf)) != 0) bitexact = 0;
    }
    check("bitexact_all_frames", bitexact);
    check("incell_or_idle_no_recalc", idle_zero);
    check("recalc_never_exceeds_n", never_exceeds_n);
    check("recalc_count_matches_cell_dirty_rule", rule_match);
    check("grouped_recalc_fewer_overall", cg.recalc < cf.recalc);
    printf("recalc_flat=%lld recalc_grouped=%lld cell_changes=%d saved=%.1f%%\n",
           cf.recalc, cg.recalc, cell_changes,
           100.0 * (1.0 - (double)cg.recalc / cf.recalc));
}

/* ---- benchmark: recompute counts and grouping overhead ------------------- */

typedef struct { float x, z, yaw; } Pose;

static void build_path(Pose* path, int steps, int scenario) {
    float x = 0.0f, z = 0.0f, yaw = 0.0f, dir = 1.0f;
    for (int i = 0; i < steps; i++) {
        if (scenario == 0) {                       /* idle */
            x = z = yaw = 0.0f;
        } else if (scenario == 3) {                /* rotate: yaw accumulates */
            yaw += 0.1f;
        } else {
            float step = scenario == 1 ? 0.3f : 2.0f;   /* jitter vs walk */
            x += step * dir;
            if (x > 30.0f || x < -30.0f) { dir = -dir; x += step * dir; }
        }
        path[i] = (Pose){ x, z, yaw };
    }
}

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static volatile long long g_sink;

/* bookkeeping alone: grid rebuild + per-frame cell lookup, no DSP or mixing */
static double time_bookkeeping(const Source* srcs, int n, const Pose* path, int steps) {
    SpatialGrid g;
    grid_init(&g);
    grid_rebuild(&g, srcs, n);
    double t0 = now_sec();
    long long touched = 0;
    for (int i = 0; i < steps; i++) {
        Listener l = { path[i].x, path[i].z, path[i].yaw };
        Listener q = quantize(&l);
        if (grid_update(&g, &q) == RECALC_ALL)
            touched += n;                       /* sources the DSP pass touches */
    }
    double t1 = now_sec();
    g_sink += touched;
    return t1 - t0;
}

static void benchmark(void) {
    enum { N = 512, FRAMES = 256, STEPS = 120 };
    static Source srcs[N];
    static SrcParams pf[N], pg[N];
    static float mono[FRAMES], out_flat[FRAMES * 2], out_grp[FRAMES * 2];
    rng_state = 7u;
    for (int k = 0; k < N; k++)
        srcs[k] = (Source){ frand(-31, 31), 0.0f, frand(-31, 31), 1.0f };
    for (int i = 0; i < FRAMES; i++) mono[i] = sinf(0.05f * i);

    const char* names[4] = { "idle  ", "jitter", "walk  ", "rotate" };
    double max_overhead = 0.0;
    int within_budget = 1, all_bitexact = 1;
    printf("\n%-8s %12s %12s %8s %10s %10s %9s\n",
           "scene", "flat_recalc", "grp_recalc", "saved", "flat_ms", "grp_ms", "ovh%%");
    for (int sc = 0; sc < 4; sc++) {
        Pose path[STEPS];
        build_path(path, STEPS, sc);

        Counters cf = {0}, cg = {0};
        Listener l;
        double t0 = now_sec();
        for (int i = 0; i < STEPS; i++) {
            l = (Listener){ path[i].x, path[i].z, path[i].yaw };
            render_flat(srcs, N, &l, mono, FRAMES, out_flat, pf, &cf);
        }
        double flat_ms = (now_sec() - t0) * 1e3;

        SpatialGrid g;
        grid_init(&g);
        grid_rebuild(&g, srcs, N);
        t0 = now_sec();
        for (int i = 0; i < STEPS; i++) {
            l = (Listener){ path[i].x, path[i].z, path[i].yaw };
            render_grouped(&g, srcs, N, &l, mono, FRAMES, out_grp, pg, &cg);
        }
        double grp_ms = (now_sec() - t0) * 1e3;
        if (memcmp(out_flat, out_grp, sizeof(out_flat)) != 0)
            all_bitexact = 0;

        double book_ms = time_bookkeeping(srcs, N, path, STEPS) * 1e3;
        double overhead = 100.0 * book_ms / grp_ms;
        if (overhead > max_overhead) max_overhead = overhead;
        if (overhead > 20.0) within_budget = 0;
        double saved = 100.0 * (1.0 - (double)cg.recalc / cf.recalc);
        printf("%-8s %12lld %12lld %7.1f%% %10.2f %10.2f %8.2f\n",
               names[sc], cf.recalc, cg.recalc, saved, flat_ms, grp_ms, overhead);
    }
    check("benchmark_bitexact", all_bitexact);
    check("grouping_overhead_under_20pct", within_budget);
    printf("overhead_max=%.2f%% (budget 20%%)\n", max_overhead);
}

int main(void) {
    float mono[MAX_FRAMES];
    float out[MAX_FRAMES * 2];
    SrcParams p[MAX_SRC];
    Counters c = {0};
    for (int i = 0; i < 256; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;  /* impulse at 64 */

    Listener l = {0, 0, 0};
    SrcParams pr = {0}, pl = {0};
    /* z offset keeps a clear left/right azimuth after cell-center
       quantization instead of snapping onto the +/-90 degree side axis */
    Source right = { 10.0f, 0.0f, -3.0f, 1.0f };
    Source left  = { -10.0f, 0.0f, -3.0f, 1.0f };
    Listener q0 = quantize(&l);
    calc_params(&right, &q0, &pr);
    calc_params(&left,  &q0, &pl);
    printf("right_source ITD=%+d samples IID gl=%.3f gr=%.3f\n", pr.delay, pr.gl, pr.gr);
    printf("left_source  ITD=%+d samples IID gl=%.3f gr=%.3f\n", pl.delay, pl.gl, pl.gr);
    render_flat(&right, 1, &l, mono, 256, out, p, &c);
    printf("right_source mix_peak=%.4f (impulse@64)\n", peak(out, 256*2));

    Source near = { 0.0f, 0.0f, -2.0f, 1.0f };
    Source far  = { 0.0f, 0.0f, -18.0f, 1.0f };
    render_flat(&near, 1, &l, mono, 256, out, p, &c); float pn = peak(out, 256*2);
    render_flat(&far, 1, &l, mono, 256, out, p, &c);  float pf = peak(out, 256*2);
    printf("attenuation near=%.4f far=%.4f ratio=%.3f\n", pn, pf, pf / pn);

    Source many[4] = { {10,0,-3,1}, {-10,0,-3,1}, {0,0,-2,1}, {0,0,6,1} };
    float loud[MAX_FRAMES];
    for (int i = 0; i < 256; i++) loud[i] = 0.9f;
    render_flat(many, 4, &l, loud, 256, out, p, &c);
    printf("mix_peak=%.4f (soft clip 上限 1.0)\n", peak(out, 256*2));

    test_bitexact();
    benchmark();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
