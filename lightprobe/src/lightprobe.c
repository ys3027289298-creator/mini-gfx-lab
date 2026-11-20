
/* lightprobe: indirect-light probe grid with trilinear reconstruction.

   Indirect light is baked at a sparse set of probes and, at shading time,
   reconstructed from the eight corners of the cell around the query point:
   the three trilinear passes (four lerps on x, two on y, one on z).  Each
   corner's trilinear basis weight is additionally scaled by a distance
   falloff, so a probe contributes less the farther the shading point is
   from it; the weights are renormalised afterwards.

   Outside the grid the lookup clamps to the nearest probe: the cell index
   stops at the edge cell and the fractional part is pinned, so the
   reconstruction is exactly the edge probe.  That value is also returned
   just inside the outer face, so the reconstruction is continuous right up
   to the boundary (no seam at the edge).

   Disabling interpolation returns the pre-existing behaviour -- the nearest
   probe to the shading point -- bit for bit, so toggling the feature cannot
   change an already-shipped frame.

   The reference is a per-pixel hemisphere integral of the scene radiance,
   so interpolation error can be reported per probe cell (region).
*/
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_DIM 16
#define NSAMP 128                 /* reference quadrature directions */
#define ERR_MAX 16
#define ERR_PRINT_MAX 8
#define FALLOFF_R 2.0f            /* distance falloff radius, in probe cells */
#define PI_F 3.14159265358979f

#define IMG_W 64
#define IMG_H 64
#define PLANE 2.5f                /* rendered patch spans [-PLANE, PLANE] */
#define GRID_N 6                  /* probes per horizontal axis */
#define GRID_SPACING 1.0f

typedef struct { float x, y, z; } Vec3;
typedef struct { float r, g, b; } Rgb;

typedef enum { MODE_NEAREST = 0, MODE_TRILINEAR = 1 } SampleMode;

#define ZERO_RGB ((Rgb){ 0.0f, 0.0f, 0.0f })

static Rgb rgb_scale(Rgb a, float s) { return (Rgb){ a.r * s, a.g * s, a.b * s }; }
static Rgb rgb_add(Rgb a, Rgb b)     { return (Rgb){ a.r + b.r, a.g + b.g, a.b + b.b }; }
static float rgb_maxdiff(Rgb a, Rgb b) {
    float d = fabsf(a.r - b.r);
    if (fabsf(a.g - b.g) > d) d = fabsf(a.g - b.g);
    if (fabsf(a.b - b.b) > d) d = fabsf(a.b - b.b);
    return d;
}

/* ---- error log ---------------------------------------------------------- */

typedef struct { char code[24]; int ix, iy, iz; } ProbeErr;
typedef struct { ProbeErr item[ERR_MAX]; int count; int dropped; } ErrLog;

static void err_clear(ErrLog* log) { memset(log, 0, sizeof(*log)); }

/* ---- scene and reference integrator ------------------------------------- */

typedef struct { Vec3 pos; Rgb color; float power; } SoftLight;
typedef struct { Rgb sky; SoftLight light[2]; } Scene;

static Vec3 g_dir[NSAMP];

static void dirs_init(void) {
    /* Fibonacci hemisphere around +y: uniform in y, hence uniform solid
       angle; deterministic so probe bakes and pixel references agree. */
    const float golden = 2.39996323f;
    for (int i = 0; i < NSAMP; i++) {
        float y = ((float)i + 0.5f) / (float)NSAMP;
        float r = sqrtf(fmaxf(0.0f, 1.0f - y * y));
        float phi = (float)i * golden;
        g_dir[i] = (Vec3){ r * cosf(phi), y, r * sinf(phi) };
    }
}

/* irradiance at p for a surface facing +y, by summing scene radiance over
   the hemisphere samples (the "per-pixel integral" reference) */
static Rgb integrate(const Scene* sc, Vec3 p) {
    const float dw = 2.0f * PI_F / (float)NSAMP;
    Rgb e = ZERO_RGB;
    for (int i = 0; i < NSAMP; i++) {
        Vec3 w = g_dir[i];
        Rgb L = sc->sky;
        for (int l = 0; l < 2; l++) {
            Vec3 d = { sc->light[l].pos.x - p.x,
                       sc->light[l].pos.y - p.y,
                       sc->light[l].pos.z - p.z };
            float dist2 = d.x * d.x + d.y * d.y + d.z * d.z;
            float inv = 1.0f / sqrtf(dist2);
            float c = (w.x * d.x + w.y * d.y + w.z * d.z) * inv;
            if (c > 0.0f) {
                float lobe = c * c * c * c * c * c;   /* c^6, soft lobe */
                float att = sc->light[l].power / (1.0f + dist2);
                L = rgb_add(L, rgb_scale(sc->light[l].color, lobe * att));
            }
        }
        e = rgb_add(e, rgb_scale(L, w.y * dw));
    }
    return e;
}

/* ---- probe grid --------------------------------------------------------- */

typedef struct {
    Vec3 origin;
    float spacing;
    int dim[3];
    Rgb value[MAX_DIM][MAX_DIM][MAX_DIM];          /* [x][y][z] */
    unsigned char present[MAX_DIM][MAX_DIM][MAX_DIM];
} ProbeGrid;

static void err_report(ErrLog* log, const ProbeGrid* g, const char* code,
                       int ix, int iy, int iz) {
    for (int i = 0; i < log->count; i++)
        if (log->item[i].ix == ix && log->item[i].iy == iy &&
            log->item[i].iz == iz && strcmp(log->item[i].code, code) == 0)
            return;
    int stored = 0;
    if (log->count < ERR_MAX) {
        ProbeErr* e = &log->item[log->count++];
        snprintf(e->code, sizeof(e->code), "%s", code);
        e->ix = ix; e->iy = iy; e->iz = iz;
        stored = 1;
    } else {
        log->dropped++;
    }
    if (stored && log->count <= ERR_PRINT_MAX) {
        float wx = g ? g->origin.x : 0.0f;
        float wy = g ? g->origin.y : 0.0f;
        float wz = g ? g->origin.z : 0.0f;
        if (g && isfinite(g->spacing) && g->spacing > 0.0f) {
            wx += (float)ix * g->spacing;
            wy += (float)iy * g->spacing;
            wz += (float)iz * g->spacing;
        }
        printf("error=%s probe=(%d,%d,%d) world=(%.3f,%.3f,%.3f)\n",
               code, ix, iy, iz, wx, wy, wz);
    }
}

static int err_has(const ErrLog* log, const char* code, int ix, int iy, int iz) {
    for (int i = 0; i < log->count; i++)
        if (log->item[i].ix == ix && log->item[i].iy == iy &&
            log->item[i].iz == iz && strcmp(log->item[i].code, code) == 0)
            return 1;
    return 0;
}

static Vec3 probe_world(const ProbeGrid* g, int ix, int iy, int iz) {
    return (Vec3){ g->origin.x + (float)ix * g->spacing,
                   g->origin.y + (float)iy * g->spacing,
                   g->origin.z + (float)iz * g->spacing };
}

/* Zero spacing and zero dimensions are configuration mistakes: reject the
   grid before any probe is written, naming the offending index so the
   caller can point at the coordinate in the scene. */
static int grid_init(ProbeGrid* g, ErrLog* log, Vec3 origin, float spacing,
                     int nx, int ny, int nz) {
    memset(g, 0, sizeof(*g));
    g->origin = origin;
    g->spacing = spacing;

    if (!isfinite(spacing) || spacing <= 0.0f) {
        err_report(log, g, "grid_spacing_zero", 0, 0, 0);
        return -1;
    }
    int d[3] = { nx, ny, nz };
    static const char* zcode[3] = { "grid_size_zero_x", "grid_size_zero_y", "grid_size_zero_z" };
    static const char* lcode[3] = { "grid_too_large_x", "grid_too_large_y", "grid_too_large_z" };
    for (int a = 0; a < 3; a++) {
        if (d[a] <= 0) {
            err_report(log, g, zcode[a], 0, 0, 0);
            return -1;
        }
        if (d[a] > MAX_DIM) {
            int idx[3] = { 0, 0, 0 };
            idx[a] = MAX_DIM;
            err_report(log, g, lcode[a], idx[0], idx[1], idx[2]);
            return -1;
        }
    }

    memcpy(g->dim, d, sizeof(d));
    return 0;
}

static void grid_set(ProbeGrid* g, int ix, int iy, int iz, Rgb v) {
    if (ix < 0 || ix >= g->dim[0] || iy < 0 || iy >= g->dim[1] ||
        iz < 0 || iz >= g->dim[2]) return;
    g->value[ix][iy][iz] = v;
    g->present[ix][iy][iz] = 1;
}

static void grid_mark_missing(ProbeGrid* g, int ix, int iy, int iz) {
    if (ix < 0 || ix >= g->dim[0] || iy < 0 || iy >= g->dim[1] ||
        iz < 0 || iz >= g->dim[2]) return;
    g->present[ix][iy][iz] = 0;
}

static void grid_bake(ProbeGrid* g, const Scene* sc) {
    for (int x = 0; x < g->dim[0]; x++)
        for (int y = 0; y < g->dim[1]; y++)
            for (int z = 0; z < g->dim[2]; z++)
                grid_set(g, x, y, z, integrate(sc, probe_world(g, x, y, z)));
}

/* ---- reconstruction ----------------------------------------------------- */

/* cell index + fractional position; outside the grid both are pinned so the
   result is the nearest edge probe */
/* fractional world coordinate along axis a */
static float axis_coord(Vec3 p, int a) {
    return a == 0 ? p.x : (a == 1 ? p.y : p.z);
}

/* cell index + fractional position; outside the grid the index is clamped to
   the edge cell and the fraction pinned, so the result is the edge probe.
   With the fraction at 0 (low edge) or 1 (high edge) the trilinear basis is
   exactly one corner, i.e. nearest-probe clamping. */
static void cell_coords(const ProbeGrid* g, Vec3 p, int c[3], float f[3]) {
    for (int a = 0; a < 3; a++) {
        float cc = (axis_coord(p, a) - axis_coord(g->origin, a)) / g->spacing;
        if (g->dim[a] < 2) { c[a] = 0; f[a] = 0.0f; continue; }
        if (cc <= 0.0f) {
            c[a] = 0;
            f[a] = 0.0f;
        } else if (cc >= (float)(g->dim[a] - 1)) {
            c[a] = g->dim[a] - 2;
            f[a] = 1.0f;
        } else {
            c[a] = (int)floorf(cc);
            f[a] = cc - (float)c[a];
        }
    }
}

/* trilinear basis for the eight corners, indexed i + 2j + 4k */
static void trilinear_basis(const float f[3], float t[8]) {
    for (int k = 0; k < 2; k++)
        for (int j = 0; j < 2; j++)
            for (int i = 0; i < 2; i++) {
                float a = i ? f[0] : 1.0f - f[0];
                float b = j ? f[1] : 1.0f - f[1];
                float c = k ? f[2] : 1.0f - f[2];
                t[i + 2 * j + 4 * k] = a * b * c;
            }
}

/* explicit three-pass reconstruction over the same eight corners; equals the
   weighted sum when the falloff is off */
static Rgb trilerp_lerps(const Rgb v[8], const float f[3]) {
    Rgb x[4];
    for (int k = 0; k < 2; k++)
        for (int j = 0; j < 2; j++) {
            Rgb a = v[0 + 2 * j + 4 * k], b = v[1 + 2 * j + 4 * k];
            x[j + 2 * k] = rgb_add(a, rgb_scale((Rgb){ b.r - a.r, b.g - a.g, b.b - a.b }, f[0]));
        }
    Rgb y[2];
    for (int k = 0; k < 2; k++) {
        Rgb a = x[0 + 2 * k], b = x[1 + 2 * k];
        y[k] = rgb_add(a, rgb_scale((Rgb){ b.r - a.r, b.g - a.g, b.b - a.b }, f[1]));
    }
    return rgb_add(y[0], rgb_scale((Rgb){ y[1].r - y[0].r, y[1].g - y[0].g, y[1].b - y[0].b }, f[2]));
}

/* distance falloff applied to each corner weight, d in probe cells */
static float falloff_kernel(float d) {
    return 1.0f / (1.0f + (d / FALLOFF_R) * (d / FALLOFF_R));
}

/* raw (unnormalised) corner weights: trilinear basis times distance falloff.
   Missing probes still get a weight here so the caller can report them. */
static float corner_raw_weights(const ProbeGrid* g, Vec3 p, int decay,
                                int c[3], float f[3], float w[8]) {
    float t[8], sum = 0.0f;
    cell_coords(g, p, c, f);
    trilinear_basis(f, t);
    for (int k = 0; k < 2; k++)
        for (int j = 0; j < 2; j++)
            for (int i = 0; i < 2; i++) {
                int idx = i + 2 * j + 4 * k;
                float d = sqrtf((f[0] - (float)i) * (f[0] - (float)i) +
                                (f[1] - (float)j) * (f[1] - (float)j) +
                                (f[2] - (float)k) * (f[2] - (float)k));
                w[idx] = t[idx] * (decay ? falloff_kernel(d) : 1.0f);
                sum += w[idx];
            }
    return sum;
}

/* normalised weights over valid corners only; for tests and callers that
   want to inspect the falloff */
static void corner_weights(const ProbeGrid* g, Vec3 p, int decay, float w[8]) {
    int c[3]; float f[3];
    corner_raw_weights(g, p, decay, c, f, w);
    float sum = 0.0f;
    for (int k = 0; k < 2; k++)
        for (int j = 0; j < 2; j++)
            for (int i = 0; i < 2; i++) {
                int idx = i + 2 * j + 4 * k;
                /* a one-probe axis keeps c at 0; its far corner is out of
                   range and carries weight 0 anyway */
                int px = c[0] + i, py = c[1] + j, pz = c[2] + k;
                if (px >= g->dim[0] || py >= g->dim[1] || pz >= g->dim[2] ||
                    !g->present[px][py][pz]) w[idx] = 0.0f;
                sum += w[idx];
            }
    if (sum > 0.0f) for (int i = 0; i < 8; i++) w[i] /= sum;
}

static Rgb resolve(const ProbeGrid* g, ErrLog* log, Vec3 p,
                   SampleMode mode, int decay) {
    if (mode == MODE_NEAREST) {
        int idx[3];
        for (int a = 0; a < 3; a++) {
            long i = (long)floorf((axis_coord(p, a) - axis_coord(g->origin, a))
                                  / g->spacing + 0.5f);
            if (i < 0) i = 0;
            if (i > g->dim[a] - 1) i = g->dim[a] - 1;
            idx[a] = (int)i;
        }
        if (!g->present[idx[0]][idx[1]][idx[2]]) {
            err_report(log, g, "probe_missing", idx[0], idx[1], idx[2]);
            return ZERO_RGB;
        }
        return g->value[idx[0]][idx[1]][idx[2]];
    }

    int c[3]; float f[3], w[8];
    float raw = corner_raw_weights(g, p, decay, c, f, w);

    Rgb acc = ZERO_RGB;
    float used = 0.0f;
    for (int k = 0; k < 2; k++)
        for (int j = 0; j < 2; j++)
            for (int i = 0; i < 2; i++) {
                int idx = i + 2 * j + 4 * k;
                if (w[idx] <= 0.0f) continue;
                int px = c[0] + i, py = c[1] + j, pz = c[2] + k;
                if (px >= g->dim[0] || py >= g->dim[1] || pz >= g->dim[2])
                    continue;
                if (!g->present[px][py][pz]) {
                    err_report(log, g, "probe_missing", px, py, pz);
                    continue;
                }
                acc = rgb_add(acc, rgb_scale(g->value[px][py][pz], w[idx]));
                used += w[idx];
            }
    if (raw <= 0.0f || used <= 0.0f) return ZERO_RGB;
    /* renormalise over the corners that exist; at a cell wall a missing
       corner's weight tends to zero, so continuity survives */
    return rgb_scale(acc, 1.0f / used);
}

/* the lookup the renderer used before interpolation existed: nearest probe,
   written independently of resolve() so the comparison is meaningful */
static Rgb legacy_nearest(const ProbeGrid* g, Vec3 p) {
    int idx[3];
    for (int a = 0; a < 3; a++) {
        float cc = (axis_coord(p, a) - axis_coord(g->origin, a)) / g->spacing;
        int i = (int)floorf(cc + 0.5f);
        if (i < 0) i = 0;
        if (i > g->dim[a] - 1) i = g->dim[a] - 1;
        idx[a] = i;
    }
    return g->value[idx[0]][idx[1]][idx[2]];
}

/* ---- image + error report ----------------------------------------------- */

static Vec3 pixel_pos(int x, int y) {
    float u = ((float)x + 0.5f) / (float)IMG_W;
    float v = ((float)y + 0.5f) / (float)IMG_H;
    return (Vec3){ -PLANE + 2.0f * PLANE * u, 0.0f, -PLANE + 2.0f * PLANE * v };
}

static void render_image(const ProbeGrid* g, ErrLog* log, SampleMode mode,
                         int decay, Rgb out[IMG_W * IMG_H]) {
    for (int y = 0; y < IMG_H; y++)
        for (int x = 0; x < IMG_W; x++)
            out[y * IMG_W + x] = resolve(g, log, pixel_pos(x, y), mode, decay);
}

typedef struct {
    int n;
    double sse_nearest, sse_trilinear, sse_interp;
    float max_interp;
} RegionStat;

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    dirs_init();

    Scene sc = { { 0.05f, 0.06f, 0.08f }, {
        { { -1.5f, 1.3f, 0.9f }, { 1.0f, 0.85f, 0.7f }, 1.0f },
        { {  1.7f, 1.0f, -1.2f }, { 0.6f, 0.75f, 1.0f }, 0.8f },
    } };

    ErrLog log;
    err_clear(&log);

    ProbeGrid g;
    Vec3 origin = { -PLANE, -0.5f, -PLANE };
    if (grid_init(&g, &log, origin, GRID_SPACING, GRID_N, 2, GRID_N) != 0) {
        printf("grid_init_failed\n");
        return 1;
    }
    grid_bake(&g, &sc);

    /* ---- images ------------------------------------------------------- */
    static Rgb img_ref[IMG_W * IMG_H];
    static Rgb img_legacy[IMG_W * IMG_H];   /* renderer before this change */
    static Rgb img_off[IMG_W * IMG_H];      /* interpolation disabled */
    static Rgb img_trilin[IMG_W * IMG_H];   /* pure trilinear */
    static Rgb img_interp[IMG_W * IMG_H];   /* trilinear + distance falloff */

    for (int y = 0; y < IMG_H; y++)
        for (int x = 0; x < IMG_W; x++) {
            Vec3 p = pixel_pos(x, y);
            img_ref[y * IMG_W + x] = integrate(&sc, p);
            img_legacy[y * IMG_W + x] = legacy_nearest(&g, p);
        }

    render_image(&g, &log, MODE_NEAREST, 0, img_off);
    render_image(&g, &log, MODE_TRILINEAR, 0, img_trilin);
    render_image(&g, &log, MODE_TRILINEAR, 1, img_interp);

    check("render_no_probe_errors", log.count == 0);

    /* disabled interpolation must reproduce the old nearest-probe image
       exactly, value for value (not just within a tolerance) */
    int legacy_diff = 0;
    for (int i = 0; i < IMG_W * IMG_H; i++)
        if (img_off[i].r != img_legacy[i].r || img_off[i].g != img_legacy[i].g ||
            img_off[i].b != img_legacy[i].b) legacy_diff++;
    check("interp_off_matches_previous", legacy_diff == 0);
    check("interp_off_is_nearest", rgb_maxdiff(img_off[0], legacy_nearest(&g, pixel_pos(0, 0))) == 0.0f);

    /* ---- per-region error --------------------------------------------- */
    RegionStat region[GRID_N - 1][GRID_N - 1];
    memset(region, 0, sizeof(region));
    double sse_n = 0.0, sse_t = 0.0, sse_d = 0.0;
    for (int y = 0; y < IMG_H; y++)
        for (int x = 0; x < IMG_W; x++) {
            int i = y * IMG_W + x;
            Vec3 p = pixel_pos(x, y);
            int rx = (int)floorf((p.x - origin.x) / GRID_SPACING);
            int rz = (int)floorf((p.z - origin.z) / GRID_SPACING);
            if (rx < 0) rx = 0;
            if (rx > GRID_N - 2) rx = GRID_N - 2;
            if (rz < 0) rz = 0;
            if (rz > GRID_N - 2) rz = GRID_N - 2;
            RegionStat* rs = &region[rx][rz];
            float en = rgb_maxdiff(img_off[i], img_ref[i]);
            float et = rgb_maxdiff(img_trilin[i], img_ref[i]);
            float ed = rgb_maxdiff(img_interp[i], img_ref[i]);
            rs->n++;
            rs->sse_nearest += (double)en * en;
            rs->sse_trilinear += (double)et * et;
            rs->sse_interp += (double)ed * ed;
            if (ed > rs->max_interp) rs->max_interp = ed;
            sse_n += (double)en * en;
            sse_t += (double)et * et;
            sse_d += (double)ed * ed;
        }

    int regions_ok = 1;
    for (int rz = 0; rz < GRID_N - 1; rz++)
        for (int rx = 0; rx < GRID_N - 1; rx++) {
            RegionStat* rs = &region[rx][rz];
            double rn = sqrt(rs->sse_nearest / (double)rs->n);
            double rt = sqrt(rs->sse_trilinear / (double)rs->n);
            double rd = sqrt(rs->sse_interp / (double)rs->n);
            printf("region=(%d,%d) px=%d rmse_nearest=%.5f rmse_trilinear=%.5f "
                   "rmse_interp=%.5f max_interp=%.5f\n",
                   rx, rz, rs->n, rn, rt, rd, rs->max_interp);
            if (rd > rn + 1e-6) regions_ok = 0;
        }
    int total = IMG_W * IMG_H;
    double rn = sqrt(sse_n / total), rt = sqrt(sse_t / total), rd = sqrt(sse_d / total);
    printf("overall rmse_nearest=%.5f rmse_trilinear=%.5f rmse_interp=%.5f "
           "reduction=%.1f%%\n", rn, rt, rd, 100.0 * (1.0 - rd / rn));

    check("interp_beats_nearest_overall", rd < rn * 0.75);
    check("no_region_worse_than_nearest", regions_ok);
    check("trilinear_beats_nearest_overall", rt < rn * 0.75);

    /* every interior wall of the reconstruction cells sits at a probe index.
       A discontinuity would leave the jump across the wall constant as the
       two sample points close in; a continuous reconstruction's jump shrinks
       with the separation.  Test both sides at delta and delta/32 and require
       the jump to fall off proportionally. */
    float wall_jump_big = 0.0f, wall_jump_small = 0.0f;
    int wall_cases = 0;
    const float delta = 1e-3f;
    for (int axis = 0; axis < 2; axis++) {
        for (int k = 1; k < GRID_N - 1; k++) {
            float face = origin.x + (float)k * GRID_SPACING;
            for (int i = 0; i < 7; i++) {
                float other = -2.2f + 0.73f * (float)i;
                float y = -0.2f + 0.04f * (float)i;
                int axis_idx = axis;
                float d[2] = { delta, delta / 32.0f };
                for (int pass = 0; pass < 2; pass++) {
                    float dd = d[pass];
                    Vec3 a = axis_idx ? (Vec3){ other, y, face - dd }
                                      : (Vec3){ face - dd, y, other };
                    Vec3 b = axis_idx ? (Vec3){ other, y, face + dd }
                                      : (Vec3){ face + dd, y, other };
                    float jump = rgb_maxdiff(resolve(&g, &log, a, MODE_TRILINEAR, 1),
                                             resolve(&g, &log, b, MODE_TRILINEAR, 1));
                    if (pass == 0) { if (jump > wall_jump_big) wall_jump_big = jump; }
                    else           { if (jump > wall_jump_small) wall_jump_small = jump; }
                }
                wall_cases++;
            }
        }
    }
    printf("wall_cases=%d interp_wall_jump_step1e-3=%.8f step3e-5=%.8f\n",
           wall_cases, wall_jump_big, wall_jump_small);
    check("wall_cases_tested", wall_cases > 20);
    /* a continuous field's jump is proportional to the separation, so closing
       the window 32x must shrink it by roughly that factor; a real seam keeps
       the same jump regardless of how close the samples get.  Both the shrink
       and the absolute small-window jump are required. */
    check("no_seam_inside_grid",
          wall_jump_small < wall_jump_big * (1.0f / 8.0f) && wall_jump_small < 1e-4f);

    /* the un-interpolated lookup really does step: it switches probe at the
       cell midpoints.  This is the artefact the reconstruction removes. */
    float nearest_step = 0.0f;
    int step_cases = 0;
    for (int k = 1; k < GRID_N - 1; k++) {
        float mid = origin.x + ((float)k + 0.5f) * GRID_SPACING;
        for (int i = 0; i < 7; i++) {
            float other = -2.2f + 0.73f * (float)i;
            Vec3 a = { mid - delta, 0.0f, other };
            Vec3 b = { mid + delta, 0.0f, other };
            float d = rgb_maxdiff(legacy_nearest(&g, a), legacy_nearest(&g, b));
            if (d > nearest_step) nearest_step = d;
            if (d > 1e-4f) step_cases++;
        }
    }
    printf("nearest_midpoint_steps=%d nearest_largest_step=%.5f\n",
           step_cases, nearest_step);
    check("nearest_lookup_steps_between_probes", nearest_step > 1e-2f);

    /* outer face: inside the last cell and just outside must agree */
    {
        Vec3 inside  = { PLANE - 1e-6f, 0.1f, 0.3f };
        Vec3 outside = { PLANE + 1e-6f, 0.1f, 0.3f };
        float d = rgb_maxdiff(resolve(&g, &log, inside, MODE_TRILINEAR, 1),
                              resolve(&g, &log, outside, MODE_TRILINEAR, 1));
        check("boundary_clamped_continuous", d < 1e-4f);
    }

    /* far outside the grid the clamp is exactly the corner probe */
    {
        Vec3 far_out = { -10.0f, -10.0f, -10.0f };
        Rgb v = resolve(&g, &log, far_out, MODE_TRILINEAR, 1);
        check("boundary_clamps_to_nearest_probe",
              rgb_maxdiff(v, g.value[0][0][0]) < 1e-7f);
    }

    /* ---- at a probe the reconstruction returns that probe ------------- */
    {
        int px = 3, py = 1, pz = 2;
        Vec3 at = probe_world(&g, px, py, pz);
        Rgb v = resolve(&g, &log, at, MODE_TRILINEAR, 1);
        check("probe_value_exact", rgb_maxdiff(v, g.value[px][py][pz]) < 1e-6f);
    }

    /* ---- weight decay with distance ----------------------------------- */
    {
        float w0[8], w1[8];
        Vec3 p = pixel_pos(10, 10);
        corner_weights(&g, p, 0, w0);
        corner_weights(&g, p, 1, w1);
        /* pixel (10,10) sits nearest corner 7 of its cell; the falloff has to
           move weight from the opposite corner 0 towards it */
        const float before_near = w0[7], before_far = w0[0];
        const float after_near = w1[7], after_far = w1[0];
        printf("falloff_corner_weight near %.4f->%.4f far %.4f->%.4f\n",
               before_near, after_near, before_far, after_far);
        check("falloff_shifts_weight_to_near",
              after_near > before_near && after_far < before_far);
        check("falloff_monotonic",
              falloff_kernel(0.0f) > falloff_kernel(1.0f) &&
              falloff_kernel(1.0f) > falloff_kernel(2.0f) &&
              falloff_kernel(2.0f) > falloff_kernel(3.0f));
    }
    {
        float w[8];
        Vec3 p = { -2.0f, 0.25f, -2.1f };
        corner_weights(&g, p, 1, w);
        float sum = 0.0f;
        for (int i = 0; i < 8; i++) sum += w[i];
        check("weights_normalised", fabsf(sum - 1.0f) < 1e-5f);
    }

    /* weighted sum with falloff off equals the explicit three-pass lerp */
    {
        Vec3 p = { -1.73f, 0.27f, 0.41f };
        int c[3]; float f[3], w[8];
        corner_raw_weights(&g, p, 0, c, f, w);
        Rgb corners[8];
        for (int k = 0; k < 2; k++)
            for (int j = 0; j < 2; j++)
                for (int i = 0; i < 2; i++)
                    corners[i + 2 * j + 4 * k] = g.value[c[0] + i][c[1] + j][c[2] + k];
        Rgb lerped = trilerp_lerps(corners, f);
        Rgb weighted = resolve(&g, &log, p, MODE_TRILINEAR, 0);
        check("trilinear_matches_three_lerps", rgb_maxdiff(lerped, weighted) < 1e-6f);
    }

    /* ---- configuration errors ----------------------------------------- */
    {
        ErrLog el; err_clear(&el);
        ProbeGrid bad;
        int rc = grid_init(&bad, &el, (Vec3){ 1.0f, 2.0f, 3.0f }, 0.0f, 4, 4, 4);
        check("reject_zero_spacing", rc == -1 && err_has(&el, "grid_spacing_zero", 0, 0, 0));

        err_clear(&el);
        rc = grid_init(&bad, &el, (Vec3){ 1.0f, 2.0f, 3.0f }, 1.0f, 0, 4, 4);
        check("reject_zero_dim_x", rc == -1 && err_has(&el, "grid_size_zero_x", 0, 0, 0));

        err_clear(&el);
        rc = grid_init(&bad, &el, (Vec3){ 1.0f, 2.0f, 3.0f }, 1.0f, 4, 0, 4);
        check("reject_zero_dim_y", rc == -1 && err_has(&el, "grid_size_zero_y", 0, 0, 0));

        err_clear(&el);
        rc = grid_init(&bad, &el, (Vec3){ 1.0f, 2.0f, 3.0f }, 1.0f, 4, 4, 0);
        check("reject_zero_dim_z", rc == -1 && err_has(&el, "grid_size_zero_z", 0, 0, 0));
    }

    /* ---- missing probe: reported with its coordinates ----------------- */
    {
        ErrLog el; err_clear(&el);
        grid_mark_missing(&g, 2, 1, 2);
        Vec3 at = probe_world(&g, 2, 1, 2);
        Rgb v = resolve(&g, &el, at, MODE_TRILINEAR, 1);
        check("missing_probe_reported", err_has(&el, "probe_missing", 2, 1, 2));
        check("missing_probe_value_zero", rgb_maxdiff(v, ZERO_RGB) < 1e-7f);

        /* a neighbouring cell that uses the probe also reports it */
        ErrLog el2; err_clear(&el2);
        Vec3 near = { at.x + 0.1f, at.y - 0.1f, at.z + 0.1f };
        v = resolve(&g, &el2, near, MODE_TRILINEAR, 1);
        check("missing_probe_reported_from_cell", err_has(&el2, "probe_missing", 2, 1, 2));
        check("missing_probe_renormalises", isfinite(v.r) && isfinite(v.g) && isfinite(v.b));

        /* the log does not repeat the same coordinate */
        resolve(&g, &el2, near, MODE_TRILINEAR, 1);
        check("missing_probe_deduplicated", el2.count == 1);

        grid_set(&g, 2, 1, 2, integrate(&sc, at));
        err_clear(&el);
        check("missing_probe_restored",
              resolve(&g, &el, at, MODE_TRILINEAR, 1).r > 0.0f && el.count == 0);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
