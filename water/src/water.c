
/* water: 2D water reflection. Rows above the waterline are mirrored into
   the water region, sampled with a configurable wave distortion, and
   blended toward a deep-water tint by a fresnel-style weight that grows
   toward grazing angles (deeper pixels reflect more).
   Two paths are provided: a per-pixel reference (water_reflect_ref) and
   an optimised LUT/fixed-point path (water_reflect_opt); the tests pin
   them to within one LSB of each other. */
#define _POSIX_C_SOURCE 199309L
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define IMG_W 320
#define IMG_H 180

typedef struct { unsigned char r, g, b; } Px;

typedef struct {
    int   waterline_y; /* first water row; rows above are reflected */
    float amplitude;   /* wave distortion amplitude, pixels */
    float wavelength;  /* wave period along y, pixels */
    float speed;       /* wave phase speed, cycles/sec */
    float f0;          /* reflectance looking straight down, 0..1 */
    Px    tint;        /* deep water colour */
} WaterCfg;

static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* per-row wave offsets and fresnel weight, shared by both paths */
typedef struct { int ox, oy; float fres; } RowPar;

static void row_params(const WaterCfg* c, int h, int y, float t, RowPar* out) {
    float phase = 2.f * (float)M_PI * ((float)y / c->wavelength + t * c->speed);
    out->ox = (int)lroundf(c->amplitude * sinf(phase));
    out->oy = (int)lroundf(0.5f * c->amplitude * cosf(0.7f * phase));
    float v = (float)(y - c->waterline_y) / (float)(h - c->waterline_y);
    float v5 = v * v * v * v * v; /* Schlick-style grazing falloff */
    out->fres = c->f0 + (1.f - c->f0) * v5;
}

/* clamped mirror-sample coordinate for dst pixel (x, y); exposed so the
   tests can verify distortion never reads outside the scene */
static void sample_coord(const WaterCfg* c, int w, int h, int x, int y,
                         const RowPar* rp, int* sx, int* sy) {
    int my = 2 * c->waterline_y - y;
    *sx = clampi(x + rp->ox, 0, w - 1);
    *sy = clampi(my + rp->oy, 0, h - 1);
}

/* reference: naive per-pixel float mirror sampling */
static void water_reflect_ref(Px* dst, const Px* scene, int w, int h,
                              const WaterCfg* c, float t) {
    for (int y = c->waterline_y; y < h; y++) {
        RowPar rp;
        row_params(c, h, y, t, &rp);
        for (int x = 0; x < w; x++) {
            int sx, sy;
            sample_coord(c, w, h, x, y, &rp, &sx, &sy);
            const Px* s = &scene[sy * w + sx];
            Px* d = &dst[y * w + x];
            float f = rp.fres, g = 1.f - f;
            d->r = (unsigned char)lroundf(s->r * f + c->tint.r * g);
            d->g = (unsigned char)lroundf(s->g * f + c->tint.g * g);
            d->b = (unsigned char)lroundf(s->b * f + c->tint.b * g);
        }
    }
}

/* optimised: per-row LUT + fixed-point blend, one sample per water pixel */
static void water_reflect_opt(Px* dst, const Px* scene, int w, int h,
                              const WaterCfg* c, float t) {
    RowPar rows[IMG_H];
    int fres8[IMG_H];
    for (int y = c->waterline_y; y < h; y++) {
        row_params(c, h, y, t, &rows[y]);
        fres8[y] = (int)(rows[y].fres * 256.f + 0.5f);
    }
    for (int y = c->waterline_y; y < h; y++) {
        int fi = fres8[y], gi = 256 - fi;
        int tr = c->tint.r * gi, tg = c->tint.g * gi, tb = c->tint.b * gi;
        for (int x = 0; x < w; x++) {
            int sx, sy;
            sample_coord(c, w, h, x, y, &rows[y], &sx, &sy);
            const Px* s = &scene[sy * w + sx];
            Px* d = &dst[y * w + x];
            d->r = (unsigned char)((s->r * fi + tr + 128) >> 8);
            d->g = (unsigned char)((s->g * fi + tg + 128) >> 8);
            d->b = (unsigned char)((s->b * fi + tb + 128) >> 8);
        }
    }
}

/* stand-in for a real scene draw: sky gradient + two composited layers */
static void scene_render(Px* buf, int w, int h, float t) {
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int r = 40 + (y * 120) / h;
            int g = 80 + (x * 60) / w;
            int b = 160 - (y * 60) / h;
            /* far hills */
            int hill = h / 3 + (int)(20.f * sinf(x * 0.05f + t));
            if (y > hill) { r = r / 2 + 30; g = g / 2 + 60; b = b / 2 + 20; }
            /* near ridge with translucent blend */
            int ridge = h / 2 + (int)(30.f * sinf(x * 0.013f + 1.7f));
            if (y > ridge) {
                r = (r * 2 + 90) / 3;
                g = (g * 2 + 70) / 3;
                b = (b * 2 + 40) / 3;
            }
            Px* p = &buf[y * w + x];
            p->r = (unsigned char)r;
            p->g = (unsigned char)g;
            p->b = (unsigned char)b;
        }
    }
}

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int max_diff(const Px* a, const Px* b, int w, int h, int y0) {
    int m = 0;
    for (int y = y0; y < h; y++)
        for (int x = 0; x < w; x++) {
            const Px* p = &a[y * w + x];
            const Px* q = &b[y * w + x];
            int dr = abs(p->r - q->r), dg = abs(p->g - q->g), db = abs(p->b - q->b);
            if (dr > m) m = dr;
            if (dg > m) m = dg;
            if (db > m) m = db;
        }
    return m;
}

int main(void) {
    static Px scene[IMG_H * IMG_W], ref[IMG_H * IMG_W], opt[IMG_H * IMG_W];
    WaterCfg cfg = {
        .waterline_y = 100,
        .amplitude = 3.f,
        .wavelength = 48.f,
        .speed = 0.5f,
        .f0 = 0.35f,
        .tint = { 12, 40, 70 },
    };
    scene_render(scene, IMG_W, IMG_H, 0.f);

    /* amplitude 0, f0 1: pure mirror, both paths bit-exact vs scene */
    WaterCfg flat = cfg;
    flat.amplitude = 0.f;
    flat.f0 = 1.f;
    water_reflect_ref(ref, scene, IMG_W, IMG_H, &flat, 0.f);
    water_reflect_opt(opt, scene, IMG_W, IMG_H, &flat, 0.f);
    int mirror_ok = 1;
    for (int y = cfg.waterline_y; y < IMG_H && mirror_ok; y++)
        for (int x = 0; x < IMG_W; x++) {
            const Px* s = &scene[(2 * cfg.waterline_y - y) * IMG_W + x];
            const Px* d = &ref[y * IMG_W + x];
            if (s->r != d->r || s->g != d->g || s->b != d->b) { mirror_ok = 0; break; }
        }
    check("mirror_exact", mirror_ok);
    check("mirror_opt_exact", max_diff(ref, opt, IMG_W, IMG_H, cfg.waterline_y) == 0);

    /* optimised path matches per-pixel manual sampling within 1 LSB */
    water_reflect_ref(ref, scene, IMG_W, IMG_H, &cfg, 0.7f);
    water_reflect_opt(opt, scene, IMG_W, IMG_H, &cfg, 0.7f);
    check("opt_matches_ref", max_diff(ref, opt, IMG_W, IMG_H, cfg.waterline_y) <= 1);

    /* amplitude is configurable: 0 vs 6 px must differ, 0 stays a clean mirror */
    WaterCfg big = cfg;
    big.amplitude = 6.f;
    static Px amp0[IMG_H * IMG_W], amp6[IMG_H * IMG_W];
    WaterCfg zero = cfg;
    zero.amplitude = 0.f;
    water_reflect_opt(amp0, scene, IMG_W, IMG_H, &zero, 0.7f);
    water_reflect_opt(amp6, scene, IMG_W, IMG_H, &big, 0.7f);
    check("amplitude_changes_output", max_diff(amp0, amp6, IMG_W, IMG_H, cfg.waterline_y) > 0);
    /* amplitude 0: output is time-invariant (no residual wave phase) */
    static Px amp0_later[IMG_H * IMG_W];
    water_reflect_opt(amp0_later, scene, IMG_W, IMG_H, &zero, 2.9f);
    check("amplitude_zero_no_distortion", max_diff(amp0, amp0_later, IMG_W, IMG_H, cfg.waterline_y) == 0);

    /* extreme amplitude: sampling stays in bounds (clamped, never wraps)
       and opt still tracks the reference */
    WaterCfg wild = cfg;
    wild.amplitude = 60.f;
    int in_bounds = 1;
    for (int y = cfg.waterline_y; y < IMG_H && in_bounds; y++) {
        RowPar rp;
        row_params(&wild, IMG_H, y, 1.3f, &rp);
        for (int x = 0; x < IMG_W; x++) {
            int sx, sy;
            sample_coord(&wild, IMG_W, IMG_H, x, y, &rp, &sx, &sy);
            if (sx < 0 || sx >= IMG_W || sy < 0 || sy >= IMG_H) { in_bounds = 0; break; }
        }
    }
    check("extreme_amplitude_in_bounds", in_bounds);
    water_reflect_ref(ref, scene, IMG_W, IMG_H, &wild, 1.3f);
    water_reflect_opt(opt, scene, IMG_W, IMG_H, &wild, 1.3f);
    check("extreme_amplitude_matches_ref", max_diff(ref, opt, IMG_W, IMG_H, cfg.waterline_y) <= 1);

    /* fresnel: surface weight equals f0, grows monotonically to 1 at depth */
    RowPar r0, r1, r2;
    row_params(&cfg, IMG_H, cfg.waterline_y, 0.f, &r0);
    row_params(&cfg, IMG_H, (cfg.waterline_y + IMG_H) / 2, 0.f, &r1);
    row_params(&cfg, IMG_H, IMG_H - 1, 0.f, &r2);
    check("fresnel_surface_f0", fabsf(r0.fres - cfg.f0) < 1e-4f);
    check("fresnel_monotonic", r0.fres < r1.fres && r1.fres < r2.fres);
    check("fresnel_grazing_one", r2.fres > 0.95f);

    /* f0 0: the surface row itself is pure tint */
    WaterCfg glass = cfg;
    glass.f0 = 0.f;
    glass.amplitude = 0.f;
    water_reflect_opt(opt, scene, IMG_W, IMG_H, &glass, 0.f);
    const Px* top = &opt[cfg.waterline_y * IMG_W + 10];
    check("surface_pure_tint", top->r == cfg.tint.r && top->g == cfg.tint.g && top->b == cfg.tint.b);

    /* reflection pass must cost no more than 30% of the scene draw */
    const int iters = 400;
    double t0 = now_s();
    for (int i = 0; i < iters; i++) scene_render(scene, IMG_W, IMG_H, (float)i * 0.01f);
    double scene_t = now_s() - t0;
    t0 = now_s();
    for (int i = 0; i < iters; i++) water_reflect_opt(opt, scene, IMG_W, IMG_H, &cfg, (float)i * 0.01f);
    double water_t = now_s() - t0;
    double ratio = water_t / scene_t;
    printf("cost_ratio=%.3f\n", ratio);
    check("cost_under_30_percent", ratio <= 0.30);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
