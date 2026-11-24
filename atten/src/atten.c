/* atten: top-down entity lighting, screen-space falloff vs world-space falloff.
 *
 * The shipped renderer attenuated entities by their distance from the viewport
 * centre in pixels (a radial mask in view space).  That makes brightness depend
 * on camera pan and zoom, and it clamps to zero at the viewport corner.  The
 * physically correct model is an inverse-square falloff in world space:
 *
 *     A_world(d) = 1 / (1 + (d / d0)^2)
 *
 * Screen model (what the renderer did):
 *
 *     A_screen(d) = 1 - smoothstep(d / R),   R = half the viewport diagonal
 *
 * Calibration.  Near the camera both curves start at 1.  Expanding about d=0,
 *
 *     A_screen ~ 1 - 3 (d/R)^2 ,    A_world ~ 1 - (d/d0)^2
 *
 * so matching the curvature at the nadir gives
 *
 *     d0 = R / sqrt(3).
 *
 * With that d0 the two curves differ by less than 0.36 % out to 20 m, and
 * since their difference is exactly
 *
 *     A_screen / A_world - 1 = t^3 (2 - 9t + 6t^2),   t = d/R,
 *
 * they cross exactly once (besides d=0) at t = (9 - sqrt(33)) / 12, i.e.
 * d = 0.2713 R.  Past the crossing the screen model is dimmer, reaches -5 %
 * at d = 0.4148 R, and goes to zero at the viewport corner, while the world
 * model keeps its 1/d^2 tail forever.
 *
 * Any other conversion (e.g. d0 = R or d0 = R/2) misses the near-field
 * calibration and is rejected by the 5 % near-field check in main().
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define IMG_W 3840
#define IMG_H 2160
#define PX_PER_M 30.0

#define GROUND_ALBEDO 0.30
#define POLE_ALBEDO 0.95
#define POLE_RADIUS_M 0.45

#define NEAR_LIMIT_M 20.0       /* near-field agreement window */
#define NEAR_LIMIT_PCT 5.0      /* acceptance limit */
#define PROFILE_MAX_M 120.0
#define PROFILE_STEP_M 0.25

#define RACK_COUNT 3
#define POLES_PER_RACK 5
#define POLE_SPACING_M 2.0
#define RACK_CROP_HALF_X_M 6.0
#define RACK_CROP_HALF_Y_M 7.0

typedef enum { MODEL_SCREEN = 0, MODEL_WORLD = 1, MODEL_COUNT } Model;

typedef struct { double x, y; } Vec2;

static const double RACK_X[RACK_COUNT] = { 12.0, 32.0, 52.0 };
static const char* const MODEL_NAME[MODEL_COUNT] = { "screen", "world" };

/* ---- attenuation models -------------------------------------------------- */

static double viewport_radius(void)
{
    double diag_px = hypot((double)IMG_W, (double)IMG_H);
    return 0.5 * diag_px / PX_PER_M;
}

static double screen_gain_at(double d, double px_per_m)
{
    double R = 0.5 * hypot((double)IMG_W, (double)IMG_H) / px_per_m;
    double t = d / R;
    if (t <= 0.0) return 1.0;
    if (t >= 1.0) return 0.0;
    return 1.0 - t * t * (3.0 - 2.0 * t);
}

static double screen_gain(double d)
{
    return screen_gain_at(d, PX_PER_M);
}

static double world_d0(void)
{
    return viewport_radius() / sqrt(3.0);
}

static double world_gain(double d)
{
    double q = d / world_d0();
    return 1.0 / (1.0 + q * q);
}

static double gain(Model m, double d)
{
    return m == MODEL_SCREEN ? screen_gain(d) : world_gain(d);
}

/* ---- scene --------------------------------------------------------------- */

static Vec2 pole_pos(int rack, int idx)
{
    Vec2 p;
    p.x = RACK_X[rack];
    p.y = ((double)idx - 0.5 * (POLES_PER_RACK - 1)) * POLE_SPACING_M;
    return p;
}

static double pole_radius_px(void)
{
    return POLE_RADIUS_M * PX_PER_M;
}

/* linear light -> 8-bit display value (simple 2.2 encode) */
static unsigned char encode(double linear)
{
    double v = pow(linear, 1.0 / 2.2);
    long iv = lround(v * 255.0);
    if (iv < 0) iv = 0;
    if (iv > 255) iv = 255;
    return (unsigned char)iv;
}

/* Coverage of a pole disc at a pixel, with a 2x2 sub-sample anti-aliased edge.
   Returns the fraction of the pixel that lies inside the disc. */
static double pole_coverage(double px, double py, Vec2 c)
{
    static const double SUB[4][2] = { {0.25,0.25}, {0.75,0.25}, {0.25,0.75}, {0.75,0.75} };
    double cx = 0.5 * IMG_W + c.x * PX_PER_M;
    double cy = 0.5 * IMG_H - c.y * PX_PER_M;   /* world +y points up */
    double r = pole_radius_px();
    int hits = 0;
    for (int i = 0; i < 4; i++) {
        double dx = px + SUB[i][0] - cx;
        double dy = py + SUB[i][1] - cy;
        if (dx * dx + dy * dy <= r * r) hits++;
    }
    return hits / 4.0;
}

static void render_scene(Model m, unsigned char* gray)
{
    /* ground plane: one sample per pixel is enough, the falloff is smooth */
    for (int py = 0; py < IMG_H; py++) {
        for (int px = 0; px < IMG_W; px++) {
            double wx = ((double)px + 0.5 - 0.5 * IMG_W) / PX_PER_M;
            double wy = (0.5 * IMG_H - ((double)py + 0.5)) / PX_PER_M;
            double d = hypot(wx, wy);
            gray[(size_t)py * IMG_W + px] = encode(GROUND_ALBEDO * gain(m, d));
        }
    }

    /* marker poles, composited in any order (they never overlap) */
    for (int r = 0; r < RACK_COUNT; r++) {
        for (int k = 0; k < POLES_PER_RACK; k++) {
            Vec2 c = pole_pos(r, k);
            double d = hypot(c.x, c.y);
            unsigned char v = encode(POLE_ALBEDO * gain(m, d));
            int x0 = (int)floor(0.5 * IMG_W + (c.x - POLE_RADIUS_M) * PX_PER_M) - 1;
            int x1 = (int)ceil(0.5 * IMG_W + (c.x + POLE_RADIUS_M) * PX_PER_M) + 1;
            int y0 = (int)floor(0.5 * IMG_H - (c.y + POLE_RADIUS_M) * PX_PER_M) - 1;
            int y1 = (int)ceil(0.5 * IMG_H - (c.y - POLE_RADIUS_M) * PX_PER_M) + 1;
            if (x0 < 0) x0 = 0;
            if (y0 < 0) y0 = 0;
            if (x1 > IMG_W) x1 = IMG_W;
            if (y1 > IMG_H) y1 = IMG_H;
            for (int py = y0; py < y1; py++) {
                for (int px = x0; px < x1; px++) {
                    double cov = pole_coverage(px, py, c);
                    if (cov <= 0.0) continue;
                    size_t i = (size_t)py * IMG_W + px;
                    double mix = cov * v + (1.0 - cov) * gray[i];
                    long iv = lround(mix);
                    gray[i] = (unsigned char)(iv < 0 ? 0 : (iv > 255 ? 255 : iv));
                }
            }
        }
    }

    /* small camera crosshair at the viewport centre (the screen model's origin) */
    double cx = 0.5 * IMG_W, cy = 0.5 * IMG_H;
    for (int k = -18; k <= 18; k++) {
        if (k > -5 && k < 5) continue;
        int xs[2] = { (int)cx + k, (int)cx };
        int ys[2] = { (int)cy, (int)cy + k };
        for (int j = 0; j < 2; j++) {
            if (xs[j] >= 0 && xs[j] < IMG_W && ys[j] >= 0 && ys[j] < IMG_H)
                gray[(size_t)ys[j] * IMG_W + xs[j]] = 240;
        }
    }
}

static int write_pgm(const char* path, const unsigned char* gray, int w, int h)
{
    FILE* f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "atten: cannot open %s\n", path);
        return -1;
    }
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    size_t n = fwrite(gray, 1, (size_t)w * h, f);
    fclose(f);
    return n == (size_t)w * h ? 0 : -1;
}

static int write_crop(const char* path, const unsigned char* gray, int rack)
{
    int half_w = (int)lround(RACK_CROP_HALF_X_M * PX_PER_M);
    int half_h = (int)lround(RACK_CROP_HALF_Y_M * PX_PER_M);
    int cx = (int)lround(0.5 * IMG_W + RACK_X[rack] * PX_PER_M);
    int cy = IMG_H / 2;
    int x0 = cx - half_w, y0 = cy - half_h;
    int w = 2 * half_w, h = 2 * half_h;
    unsigned char* crop = malloc((size_t)w * h);
    if (!crop) return -1;
    for (int y = 0; y < h; y++)
        memcpy(crop + (size_t)y * w, gray + (size_t)(y0 + y) * IMG_W + x0, (size_t)w);
    int rc = write_pgm(path, crop, w, h);
    free(crop);
    return rc;
}

/* ---- measurements -------------------------------------------------------- */

static double max_near_error(double d0)
{
    double worst = 0.0;
    for (double d = 0.0; d <= NEAR_LIMIT_M; d += 0.01) {
        double a = screen_gain(d);
        double b = 1.0 / (1.0 + (d / d0) * (d / d0));
        double e = fabs(a - b) / b;
        if (e > worst) worst = e;
    }
    return worst;
}

/* d > 0 where the two gains are equal, found by bisection on (1, R) */
static double crossing_distance(void)
{
    double lo = 1e-6, hi = viewport_radius() * (1.0 - 1e-9);
    double f_lo = screen_gain(lo) - world_gain(lo);
    for (int i = 0; i < 200; i++) {
        double mid = 0.5 * (lo + hi);
        double f_mid = screen_gain(mid) - world_gain(mid);
        if (f_mid == 0.0) return mid;
        if (f_lo * f_mid < 0.0) hi = mid; else { lo = mid; f_lo = f_mid; }
    }
    return 0.5 * (lo + hi);
}

/* first d past the crossing where the screen model is 5 % below the world one */
static double five_percent_distance(void)
{
    double lo = crossing_distance();
    double hi = viewport_radius() * (1.0 - 1e-9);
    for (int i = 0; i < 200; i++) {
        double mid = 0.5 * (lo + hi);
        double rel = (screen_gain(mid) - world_gain(mid)) / world_gain(mid);
        if (fabs(rel) < NEAR_LIMIT_PCT / 100.0) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

static int failures = 0;
static void check(const char* name, int ok, const char* detail)
{
    printf("  %-22s %s   %s\n", name, ok ? "PASS" : "FAIL", detail);
    if (!ok) failures++;
}

int main(void)
{
    double R = viewport_radius();
    double d0 = world_d0();
    double d_cross = crossing_distance();
    double a_cross = screen_gain(d_cross);
    double near_err = max_near_error(d0);
    double d5 = five_percent_distance();
    double naive_full = max_near_error(R);
    double naive_half = max_near_error(0.5 * R);

    printf("atten: screen-space vs world-space falloff\n");
    printf("  viewport        %d x %d @ %.0f px/m  (world span %.1f x %.1f m)\n",
           IMG_W, IMG_H, PX_PER_M, IMG_W / PX_PER_M, IMG_H / PX_PER_M);
    printf("  R (half diagonal) %.6f m\n", R);
    printf("  d0 = R/sqrt(3)    %.6f m  (d0/R = %.9f)\n", d0, d0 / R);

    /* 1. the two gains agree at the camera */
    check("gain_at_camera", fabs(screen_gain(0.0) - 1.0) < 1e-12 &&
                            fabs(world_gain(0.0) - 1.0) < 1e-12, "both = 1.000000");

    /* 2. near field: the acceptance criterion */
    char buf[160];
    snprintf(buf, sizeof(buf), "max |rel err| = %.4f %% over 0..%.0f m (limit %.1f %%)",
             100.0 * near_err, NEAR_LIMIT_M, NEAR_LIMIT_PCT);
    check("near_field_<=5%", near_err <= NEAR_LIMIT_PCT / 100.0, buf);

    /* 3. exactly one crossing, at the analytically predicted distance */
    double t_star = (9.0 - sqrt(33.0)) / 12.0;
    snprintf(buf, sizeof(buf), "d = %.4f m, gain = %.6f (analytic %.4f m)",
             d_cross, a_cross, t_star * R);
    check("single_crossing", fabs(d_cross - t_star * R) < 1e-6, buf);
    check("crossing_in_near", d_cross <= NEAR_LIMIT_M, "inside the near-field window");

    /* 4. departure past the crossing */
    snprintf(buf, sizeof(buf), "5 %% at d = %.4f m (%.4f R), screen = %.4f, world = %.4f",
             d5, d5 / R, screen_gain(d5), world_gain(d5));
    check("diverge_past_near", d5 > NEAR_LIMIT_M && d5 < R, buf);

    /* 5. world tail is 1/d^2 forever; screen dies at the viewport corner */
    double tail = world_gain(80.0 * d0) / world_gain(40.0 * d0);
    snprintf(buf, sizeof(buf), "world A(80 d0)/A(40 d0) = %.6f (ideal 0.25)", tail);
    check("world_inverse_square", fabs(tail - 0.25) < 0.005, buf);
    check("screen_dies_at_corner", screen_gain(R) == 0.0 && world_gain(R) > 0.2,
          "screen = 0 at R, world still 0.23");

    /* 6. view dependence: panning moves the screen gain, not the world gain */
    double p0 = screen_gain(52.0);
    double p1 = screen_gain(52.0 - 30.0);   /* camera panned 30 m toward it */
    snprintf(buf, sizeof(buf), "object at 52 m: centred %.4f, panned %.4f (+%.1f %%)",
             p0, p1, 100.0 * (p1 / p0 - 1.0));
    check("screen_is_view_dependent", p1 / p0 > 1.5, buf);

    /* 7. zoom dependence: same object, different pixels per metre */
    double z_wide = screen_gain_at(32.0, 0.5 * PX_PER_M);    /* zoomed out */
    double z_norm = screen_gain_at(32.0, PX_PER_M);
    double z_tele = screen_gain_at(32.0, 2.0 * PX_PER_M);    /* zoomed in */
    snprintf(buf, sizeof(buf), "object at 32 m: zoom-out %.4f, 1x %.4f, zoom-in %.4f",
             z_wide, z_norm, z_tele);
    check("screen_is_zoom_dependent", fabs(z_wide - z_tele) > 0.2, buf);

    /* 8. the 5 % test would catch a wrong conversion constant */
    snprintf(buf, sizeof(buf), "d0=R -> %.2f %%, d0=R/2 -> %.2f %% (both > 5 %%)",
             100.0 * naive_full, 100.0 * naive_half);
    check("bad_conversion_rejected",
          naive_full > NEAR_LIMIT_PCT / 100.0 && naive_half > NEAR_LIMIT_PCT / 100.0, buf);

    printf("\n  %-10s %-8s %-8s %-8s %-8s\n", "d (m)", "screen", "world", "screen/world", "sRGB");
    for (int d = 0; d <= 60; d += 4) {
        double a = screen_gain((double)d), w = world_gain((double)d);
        printf("  %-10d %-8.4f %-8.4f %-8.4f %3u / %3u\n",
               d, a, w, a / w, encode(POLE_ALBEDO * a), encode(POLE_ALBEDO * w));
    }

    /* render both full scenes and the six per-group crops */
    unsigned char* gray = malloc((size_t)IMG_W * IMG_H);
    if (!gray) return 1;
    for (int m = 0; m < MODEL_COUNT; m++) {
        render_scene((Model)m, gray);
        char path[256];
        snprintf(path, sizeof(path), "out/scene_%s.pgm", MODEL_NAME[m]);
        if (write_pgm(path, gray, IMG_W, IMG_H) != 0) return 1;
        for (int r = 0; r < RACK_COUNT; r++) {
            snprintf(path, sizeof(path), "out/group%d_%s.pgm", r + 1, MODEL_NAME[m]);
            if (write_crop(path, gray, r) != 0) return 1;
        }
    }
    free(gray);

    /* profile along y = 0, the horizontal cut through all three racks */
    FILE* csv = fopen("out/profile.csv", "w");
    if (!csv) return 1;
    fprintf(csv, "d_m,screen_gain,world_gain,screen_over_world,rel_error_pct\n");
    for (double d = 0.0; d <= PROFILE_MAX_M + 1e-9; d += PROFILE_STEP_M) {
        double a = screen_gain(d), w = world_gain(d);
        fprintf(csv, "%.2f,%.9f,%.9f,%.9f,%.6f\n",
                d, a, w, w > 0.0 ? a / w : 0.0, w > 0.0 ? 100.0 * (a / w - 1.0) : -100.0);
    }
    fclose(csv);

    /* summary for the chart compositor (single source of truth for the markers) */
    FILE* sum = fopen("out/summary.txt", "w");
    if (!sum) return 1;
    fprintf(sum, "viewport_w_px=%d\n", IMG_W);
    fprintf(sum, "viewport_h_px=%d\n", IMG_H);
    fprintf(sum, "px_per_m=%.6f\n", PX_PER_M);
    fprintf(sum, "R_m=%.9f\n", R);
    fprintf(sum, "d0_m=%.9f\n", d0);
    fprintf(sum, "cross_m=%.9f\n", d_cross);
    fprintf(sum, "cross_gain=%.9f\n", a_cross);
    fprintf(sum, "five_pct_m=%.9f\n", d5);
    fprintf(sum, "near_limit_m=%.6f\n", NEAR_LIMIT_M);
    fprintf(sum, "near_max_err_pct=%.9f\n", 100.0 * near_err);
    fclose(sum);

    printf("\n  crossing %.4f m, 5%% at %.4f m, near max %.4f%%\n",
           d_cross, d5, 100.0 * near_err);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 2;
}
