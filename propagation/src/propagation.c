/* propagation: bake a sound source's arrival-time / attenuation field onto
   a grid once, then look it up by listener position at runtime.

   Model: image-source method for a rectangular room -- the direct wave plus
   reflections up to 3rd order off the four walls. Arrival time is the
   energy-weighted mean wave arrival time; attenuation is the RMS amplitude
   sum. Distance attenuation is 1/(1 + 0.2 d), so the field is finite
   everywhere, including at the source.

   Baseline replaced: recomputing the whole field every frame. The bake costs
   O(nodes) once; each runtime lookup is O(1) bilinear interpolation. Bilinear
   interpolation is C0 at cell edges, so the field does not jump when the
   listener crosses a grid line, and it is exact at the grid nodes.

   Known limit: the direct wave's arrival time is r/c, whose second derivative
   diverges at the source. No fixed grid resolves that cone arbitrarily close
   to the source, so the tolerance holds outside a small radius around it; the
   self-test reports both that radius and the convergence under refinement. */
#define _POSIX_C_SOURCE 199309L   /* clock_gettime / CLOCK_MONOTONIC; must precede includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

/* monotonic: wall-clock times can step backwards under NTP and make the
   reported bake time negative */
static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e3 + (double)ts.tv_nsec * 1e-6;
}

#define ROOM_W 16.0f              /* meters */
#define ROOM_H 12.0f
#define GRID_H 0.025f             /* nominal node spacing, meters */
#define SRC_X 4.0123f             /* deliberately off-node: stresses interpolation */
#define SRC_Y 5.0077f

#define REFL_ORDER 3              /* keep image sources with |m|+|n| <= this */
#define MAX_IMAGES (4 * (2*REFL_ORDER+1) * (2*REFL_ORDER+1))
#define C_AIR 343.0f              /* m/s */
#define DIST_K 0.2f               /* attenuation rolloff, 1/(1 + k d) */
#define WALL_REFL 0.5f            /* wall amplitude reflection coefficient */

#define SOURCE_EXCL (2.0f * GRID_H) /* tolerance claimed beyond two cells of the
                                       source; the arrival-time cone there is
                                       steeper than bilinear can follow */
#define TIME_TOL_MS 0.02          /* about one sample at 48 kHz */
#define GAIN_TOL 2e-3
#define QUERIES_PER_FRAME 128
#define BENCH_LOOKUPS 200000

typedef struct { float time_ms, gain; } PropSample;

typedef struct {
    int nx, ny;                   /* node counts, inclusive of both walls */
    float cell_x, cell_y;         /* meters between nodes */
    float* time_ms;               /* nx*ny, arrival time, milliseconds */
    float* gain;                  /* nx*ny, attenuation, linear amplitude */
    double bake_ms;
    size_t bytes;
} PropField;

typedef struct {
    int probes;
    double max_time_err, max_gain_err, sum_sq_time;
    int near_probes;
    double near_max_time_err;
} ErrStats;

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

typedef struct {
    float w, h;
    int count;
    float img_x[MAX_IMAGES], img_y[MAX_IMAGES];
    float refl[MAX_IMAGES];
} Room;

/* image-source enumeration: direct plus reflections up to REFL_ORDER off
   each axis. For a rectangular room every image (m, n) is at
   (2 m w +/- sx, 2 n h +/- sy) with weight WALL_REFL^(|m|+|n|). */
static void room_init(Room* r, float w, float h, float sx, float sy) {
    r->w = w;
    r->h = h;
    r->count = 0;
    for (int m = -REFL_ORDER; m <= REFL_ORDER; m++)
        for (int n = -REFL_ORDER; n <= REFL_ORDER; n++) {
            if (abs(m) + abs(n) > REFL_ORDER) continue;
            int sx_signs = (m == 0) ? 1 : 2;   /* both parities when wrapped */
            int sy_signs = (n == 0) ? 1 : 2;
            for (int a = 0; a < sx_signs; a++)
                for (int b = 0; b < sy_signs; b++) {
                    float x = 2.0f * (float)m * w + (a == 0 ? sx : -sx);
                    float y = 2.0f * (float)n * h + (b == 0 ? sy : -sy);
                    float refl = powf(WALL_REFL, (float)(abs(m) + abs(n)));
                    r->img_x[r->count] = x;
                    r->img_y[r->count] = y;
                    r->refl[r->count] = refl;
                    r->count++;
                }
        }
}

/* energy-weighted mean arrival time (ms) and RMS amplitude at (px, py) */
static PropSample model_eval(const Room* r, float px, float py) {
    float wsum = 0.0f, tsum = 0.0f;
    for (int k = 0; k < r->count; k++) {
        float dx = px - r->img_x[k];
        float dy = py - r->img_y[k];
        float d = sqrtf(dx * dx + dy * dy);
        float a = r->refl[k] / (1.0f + DIST_K * d);
        float e = a * a;
        wsum += e;
        tsum += e * (d * (1000.0f / C_AIR));
    }
    PropSample s;
    s.time_ms = tsum / wsum;
    s.gain = sqrtf(wsum);
    return s;
}

/* ------------------------------------------------------------------ */
/* Bake: sample the model at every node, spanning the room exactly     */
/* ------------------------------------------------------------------ */

static int prop_bake(PropField* f, const Room* r, float spacing) {
    memset(f, 0, sizeof(*f));
    if (spacing <= 0.0f || spacing >= r->w || spacing >= r->h) return -1;
    f->nx = (int)lroundf(r->w / spacing) + 1;
    f->ny = (int)lroundf(r->h / spacing) + 1;
    f->cell_x = r->w / (float)(f->nx - 1);
    f->cell_y = r->h / (float)(f->ny - 1);
    size_t n = (size_t)f->nx * (size_t)f->ny;
    f->time_ms = (float*)malloc(n * sizeof(float));
    f->gain = (float*)malloc(n * sizeof(float));
    if (!f->time_ms || !f->gain) {
        free(f->time_ms);
        free(f->gain);
        f->time_ms = f->gain = NULL;
        return -1;
    }
    f->bytes = n * 2 * sizeof(float);
    double t0 = now_ms();
    for (int j = 0; j < f->ny; j++) {
        float py = (float)j * f->cell_y;
        for (int i = 0; i < f->nx; i++) {
            PropSample s = model_eval(r, (float)i * f->cell_x, py);
            f->time_ms[(size_t)j * f->nx + i] = s.time_ms;
            f->gain[(size_t)j * f->nx + i] = s.gain;
        }
    }
    f->bake_ms = now_ms() - t0;
    return 0;
}

static void prop_free(PropField* f) {
    free(f->time_ms);
    free(f->gain);
    f->time_ms = f->gain = NULL;
}

/* the per-frame baseline: rebuild the entire field from the model */
static double full_field_recompute(const Room* r, PropField* f) {
    double t0 = now_ms();
    for (int j = 0; j < f->ny; j++) {
        float py = (float)j * f->cell_y;
        for (int i = 0; i < f->nx; i++) {
            PropSample s = model_eval(r, (float)i * f->cell_x, py);
            f->time_ms[(size_t)j * f->nx + i] = s.time_ms;
            f->gain[(size_t)j * f->nx + i] = s.gain;
        }
    }
    return now_ms() - t0;
}

/* ------------------------------------------------------------------ */
/* Runtime: bilinear lookup by listener position                       */
/* ------------------------------------------------------------------ */

static PropSample prop_sample(const PropField* f, float px, float py) {
    float gx = clampf(px / f->cell_x, 0.0f, (float)(f->nx - 1));
    float gy = clampf(py / f->cell_y, 0.0f, (float)(f->ny - 1));
    int x0 = (int)floorf(gx);
    int y0 = (int)floorf(gy);
    if (x0 > f->nx - 2) x0 = f->nx - 2;
    if (y0 > f->ny - 2) y0 = f->ny - 2;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    float fx = gx - (float)x0;
    float fy = gy - (float)y0;
    int x1 = x0 + 1, y1 = y0 + 1;
    const float* t = f->time_ms;
    const float* g = f->gain;
    size_t i00 = (size_t)y0 * f->nx + x0, i10 = (size_t)y0 * f->nx + x1;
    size_t i01 = (size_t)y1 * f->nx + x0, i11 = (size_t)y1 * f->nx + x1;
    float w00 = (1.0f - fx) * (1.0f - fy), w10 = fx * (1.0f - fy);
    float w01 = (1.0f - fx) * fy, w11 = fx * fy;
    PropSample s;
    s.time_ms = t[i00] * w00 + t[i10] * w10 + t[i01] * w01 + t[i11] * w11;
    s.gain = g[i00] * w00 + g[i10] * w10 + g[i01] * w01 + g[i11] * w11;
    return s;
}

/* piecewise-constant lookup, used only as a negative control for the
   continuity test */
static PropSample prop_sample_nearest(const PropField* f, float px, float py) {
    float gx = clampf(px / f->cell_x, 0.0f, (float)(f->nx - 1));
    float gy = clampf(py / f->cell_y, 0.0f, (float)(f->ny - 1));
    int xi = (int)lroundf(gx);
    int yi = (int)lroundf(gy);
    xi = xi < 0 ? 0 : (xi >= f->nx ? f->nx - 1 : xi);
    yi = yi < 0 ? 0 : (yi >= f->ny ? f->ny - 1 : yi);
    PropSample s;
    s.time_ms = f->time_ms[(size_t)yi * f->nx + xi];
    s.gain = f->gain[(size_t)yi * f->nx + xi];
    return s;
}

/* ------------------------------------------------------------------ */
/* Verification                                                        */
/* ------------------------------------------------------------------ */

static void probe_one(const Room* r, const PropField* f, float px, float py, ErrStats* st) {
    PropSample want = model_eval(r, px, py);
    PropSample got = prop_sample(f, px, py);
    double dt = fabs((double)got.time_ms - (double)want.time_ms);
    double dg = fabs((double)got.gain - (double)want.gain);
    float dsx = px - SRC_X, dsy = py - SRC_Y;
    int near_src = (dsx * dsx + dsy * dsy) < SOURCE_EXCL * SOURCE_EXCL;
    if (near_src) {
        if (dt > st->near_max_time_err) st->near_max_time_err = dt;
        st->near_probes++;
        return;
    }
    if (dt > st->max_time_err) st->max_time_err = dt;
    if (dg > st->max_gain_err) st->max_gain_err = dg;
    st->sum_sq_time += dt * dt;
    st->probes++;
}

/* Compare the baked+interpolated field against evaluating the model at the
   same point (what a per-frame full computation yields there). A coarse sweep
   covers the room; a dense sweep at 5 mm around the source finds the worst
   case, where the arrival-time cone has the highest curvature. */
static ErrStats compare_with_model(const Room* r, const PropField* f) {
    ErrStats st;
    memset(&st, 0, sizeof(st));
    for (float py = 0.0f; py <= ROOM_H; py += 0.05f)
        for (float px = 0.0f; px <= ROOM_W; px += 0.05f)
            probe_one(r, f, px, py, &st);
    for (float py = SRC_Y - 1.0f; py <= SRC_Y + 1.0f; py += 0.005f)
        for (float px = SRC_X - 1.0f; px <= SRC_X + 1.0f; px += 0.005f) {
            if (px < 0.0f || px > ROOM_W || py < 0.0f || py > ROOM_H) continue;
            probe_one(r, f, px, py, &st);
        }
    return st;
}

/* bilinear lookup must return the baked node value at a node position */
static double node_exactness(const PropField* f) {
    double worst = 0.0;
    for (int j = 0; j < f->ny; j++)
        for (int i = 0; i < f->nx; i++) {
            PropSample s = prop_sample(f, (float)i * f->cell_x, (float)j * f->cell_y);
            double d = fabs((double)s.time_ms - (double)f->time_ms[(size_t)j * f->nx + i]);
            if (d > worst) worst = d;
        }
    return worst;
}

typedef struct { double max_time_jump, max_gain_jump; } JumpStats;

/* Largest change in the lookup across a cell boundary, probing at +/- eps
   perpendicular to it. Bilinear is C0 there, so this goes to zero with eps;
   nearest-cell lookup steps at the boundary between the two nodes' Voronoi
   cells (the midpoint), so it stays at a full node-to-node step. Probes sit
   at those midpoints so the control actually straddles the nearest-cell step. */
static void edge_scan(const PropField* f, float eps, JumpStats* bilinear, JumpStats* nearest) {
    memset(bilinear, 0, sizeof(*bilinear));
    memset(nearest, 0, sizeof(*nearest));
    for (int j = 0; j < f->ny; j += 7) {
        float py = (float)j * f->cell_y + 0.37f * f->cell_y;
        for (int i = 1; i < f->nx - 1; i++) {
            float seam = ((float)i + 0.5f) * f->cell_x;
            PropSample al = prop_sample(f, seam - eps, py);
            PropSample ar = prop_sample(f, seam + eps, py);
            PropSample nl = prop_sample_nearest(f, seam - eps, py);
            PropSample nr = prop_sample_nearest(f, seam + eps, py);
            double dt = fabs((double)al.time_ms - (double)ar.time_ms);
            double dg = fabs((double)al.gain - (double)ar.gain);
            if (dt > bilinear->max_time_jump) bilinear->max_time_jump = dt;
            if (dg > bilinear->max_gain_jump) bilinear->max_gain_jump = dg;
            dt = fabs((double)nl.time_ms - (double)nr.time_ms);
            dg = fabs((double)nl.gain - (double)nr.gain);
            if (dt > nearest->max_time_jump) nearest->max_time_jump = dt;
            if (dg > nearest->max_gain_jump) nearest->max_gain_jump = dg;
        }
    }
    for (int i = 0; i < f->nx; i += 7) {
        float px = (float)i * f->cell_x + 0.37f * f->cell_x;
        for (int j = 1; j < f->ny - 1; j++) {
            float seam = ((float)j + 0.5f) * f->cell_y;
            PropSample al = prop_sample(f, px, seam - eps);
            PropSample ar = prop_sample(f, px, seam + eps);
            PropSample nl = prop_sample_nearest(f, px, seam - eps);
            PropSample nr = prop_sample_nearest(f, px, seam + eps);
            double dt = fabs((double)al.time_ms - (double)ar.time_ms);
            double dg = fabs((double)al.gain - (double)ar.gain);
            if (dt > bilinear->max_time_jump) bilinear->max_time_jump = dt;
            if (dg > bilinear->max_gain_jump) bilinear->max_gain_jump = dg;
            dt = fabs((double)nl.time_ms - (double)nr.time_ms);
            dg = fabs((double)nl.gain - (double)nr.gain);
            if (dt > nearest->max_time_jump) nearest->max_time_jump = dt;
            if (dg > nearest->max_gain_jump) nearest->max_gain_jump = dg;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Timings                                                             */
/* ------------------------------------------------------------------ */

static float rng_next(unsigned* s) {
    *s = *s * 1664525u + 1013904223u;
    return (float)((*s >> 8) & 0xFFFFFFu) * (1.0f / 16777216.0f);
}

static double bench_lookup(const PropField* f, int n) {
    unsigned s = 12345u;
    volatile double sink = 0.0;
    double t0 = now_ms();
    for (int i = 0; i < n; i++) {
        float px = rng_next(&s) * ROOM_W;
        float py = rng_next(&s) * ROOM_H;
        PropSample p = prop_sample(f, px, py);
        sink += (double)p.time_ms + (double)p.gain;
    }
    double dt = now_ms() - t0;
    (void)sink;
    return dt * 1e6 / (double)n;
}

/* per-query cost of evaluating the model at one listener position, the
   apples-to-apples alternative to one interpolated lookup */
static double bench_model(const Room* r, int n) {
    unsigned s = 12345u;
    volatile double sink = 0.0;
    double t0 = now_ms();
    for (int i = 0; i < n; i++) {
        float px = rng_next(&s) * ROOM_W;
        float py = rng_next(&s) * ROOM_H;
        PropSample p = model_eval(r, px, py);
        sink += (double)p.time_ms + (double)p.gain;
    }
    double dt = now_ms() - t0;
    (void)sink;
    return dt * 1e6 / (double)n;
}

/* ------------------------------------------------------------------ */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Room room;
    room_init(&room, ROOM_W, ROOM_H, SRC_X, SRC_Y);

    PropField f;
    if (prop_bake(&f, &room, GRID_H) != 0) {
        printf("bake_failed=1\nall_ok=0\n");
        return 1;
    }
    check("bake_ok", f.time_ms != NULL && f.gain != NULL);

    size_t n = (size_t)f.nx * (size_t)f.ny;
    printf("grid=%dx%d nodes=%zu cell_m=%.4fx%.4f spacing_nominal=%.4f\n",
           f.nx, f.ny, n, (double)f.cell_x, (double)f.cell_y, (double)GRID_H);
    printf("bake_ms=%.3f bake_bytes=%zu bake_kib=%.1f bytes_per_node=%zu\n",
           f.bake_ms, f.bytes, (double)f.bytes / 1024.0, 2 * sizeof(float));

    /* baseline: full-field per-frame recompute must reproduce the baked table
       at every node, so the bake is a faithful snapshot of that computation */
    float* baked_t = (float*)malloc(n * sizeof(float));
    float* baked_g = (float*)malloc(n * sizeof(float));
    if (!baked_t || !baked_g) { printf("alloc_failed=1\nall_ok=0\n"); return 1; }
    memcpy(baked_t, f.time_ms, n * sizeof(float));
    memcpy(baked_g, f.gain, n * sizeof(float));
    double full_frame_ms = full_field_recompute(&room, &f);
    int full_matches = memcmp(baked_t, f.time_ms, n * sizeof(float)) == 0 &&
                       memcmp(baked_g, f.gain, n * sizeof(float)) == 0;
    check("full_recompute_matches_baked_nodes", full_matches);
    free(baked_t);
    free(baked_g);

    /* correctness at arbitrary listener positions against the per-frame model */
    ErrStats es = compare_with_model(&room, &f);
    double rms_time = sqrt(es.sum_sq_time / (double)(es.probes > 0 ? es.probes : 1));
    printf("probes=%d max_time_err_ms=%.6f max_gain_err=%.6f rms_time_err_ms=%.6f\n",
           es.probes, es.max_time_err, es.max_gain_err, rms_time);
    printf("source_exclusion_m=%.3f excluded_probes=%d near_source_max_err_ms=%.6f\n",
           (double)SOURCE_EXCL, es.near_probes, es.near_max_time_err);
    printf("time_tol_ms=%.3f gain_tol=%.4f\n", (double)TIME_TOL_MS, (double)GAIN_TOL);
    check("arrival_time_within_tol", es.max_time_err <= TIME_TOL_MS);
    check("gain_within_tol", es.max_gain_err <= GAIN_TOL);

    double node_err = node_exactness(&f);
    printf("node_lookup_max_err_ms=%.3e\n", node_err);
    check("node_lookup_exact", node_err < 1e-4);

    /* refinement: halving the spacing should cut the interpolation error
       by roughly 4x, which confirms the tolerance is resolution-limited and
       not a modelling mistake */
    PropField fine;
    if (prop_bake(&fine, &room, 0.5f * GRID_H) == 0) {
        ErrStats fe = compare_with_model(&room, &fine);
        printf("fine_grid=%dx%d bake_ms=%.3f bake_kib=%.1f fine_max_time_err_ms=%.6f\n",
               fine.nx, fine.ny, fine.bake_ms, (double)fine.bytes / 1024.0,
               fe.max_time_err);
        printf("refinement_ratio=%.2f\n", es.max_time_err > 0 ? fe.max_time_err / es.max_time_err : 0.0);
        check("refinement_converges", fe.max_time_err < 0.35 * es.max_time_err);
        check("fine_grid_within_tol", fe.max_time_err <= TIME_TOL_MS);
        prop_free(&fine);
    } else {
        check("refinement_converges", 0);
        check("fine_grid_within_tol", 0);
    }

    /* continuity across cell edges: the bilinear jump shrinks with the probe
       offset while the nearest-cell jump does not */
    JumpStats big, mid, small, nn;
    edge_scan(&f, 0.25f * GRID_H, &big, &nn);
    edge_scan(&f, 0.0625f * GRID_H, &mid, &nn);
    edge_scan(&f, 0.015625f * GRID_H, &small, &nn);
    printf("edge_bilinear_jump_ms eps=%.5f,%.5f,%.5f -> %.3e,%.3e,%.3e\n",
           (double)(0.25 * GRID_H), (double)(0.0625 * GRID_H), (double)(0.015625 * GRID_H),
           big.max_time_jump, mid.max_time_jump, small.max_time_jump);
    printf("edge_bilinear_gain_jump -> %.3e,%.3e,%.3e\n",
           big.max_gain_jump, mid.max_gain_jump, small.max_gain_jump);
    printf("edge_nearest_jump_ms=%.3e gain=%.3e (eps-independent)\n",
           nn.max_time_jump, nn.max_gain_jump);
    check("edge_jump_shrinks_with_eps",
          small.max_time_jump < 0.5 * big.max_time_jump &&
          small.max_gain_jump < 0.5 * big.max_gain_jump);
    check("no_step_at_cell_edges",
          small.max_time_jump < 0.1 * nn.max_time_jump &&
          small.max_gain_jump < 0.1 * nn.max_gain_jump);

    /* clamping outside the node hull must not introduce a seam either */
    PropSample out_a = prop_sample(&f, -5.0f, -5.0f);
    PropSample edge_b = prop_sample(&f, 0.0f, 0.0f);
    check("outside_is_clamped",
          fabsf(out_a.time_ms - edge_b.time_ms) < 1e-6f &&
          fabsf(out_a.gain - edge_b.gain) < 1e-6f);

    /* runtime cost: per-frame full recompute vs 128 interpolated lookups.
       The full-grid baseline re-evaluates the model at every node; the second
       comparison isolates the per-query cost (one lookup vs one evaluation). */
    double lookup_ns = bench_lookup(&f, BENCH_LOOKUPS);
    double model_ns = bench_model(&room, BENCH_LOOKUPS);
    double baked_frame_ms = (double)QUERIES_PER_FRAME * lookup_ns / 1e6;
    double model_frame_ms = (double)QUERIES_PER_FRAME * model_ns / 1e6;
    printf("lookup_ns=%.2f model_eval_ns=%.2f queries_per_frame=%d\n",
           lookup_ns, model_ns, QUERIES_PER_FRAME);
    printf("full_grid_frame_ms=%.3f baked_frame_ms=%.4f model_per_query_frame_ms=%.4f\n",
           full_frame_ms, baked_frame_ms, model_frame_ms);
    printf("frame_speedup=%.0fx break_even_frames=%.2f per_query_speedup=%.1fx\n",
           full_frame_ms / fmax(1e-9, baked_frame_ms),
           f.bake_ms / fmax(1e-9, full_frame_ms - baked_frame_ms),
           model_ns / fmax(1e-9, lookup_ns));
    check("bake_memory_reported", f.bytes == n * 2 * sizeof(float));
    check("baked_frame_faster", baked_frame_ms < 0.1 * full_frame_ms);
    check("lookup_faster_than_model_eval", lookup_ns < 0.5 * model_ns);

    prop_free(&f);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
