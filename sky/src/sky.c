/* sky-dome: gradient sky sphere plus an analytic per-pixel sun/halo layer
   and a hard horizon mask.

   The original renderer is a vertex-coloured gradient sphere: base colour
   is evaluated on a latitude/longitude grid and linearly interpolated.
   The new layer adds, per pixel:
     - a sun disk placed by azimuth/elevation (elevation validated);
     - a halo whose strength follows the sun elevation;
     - a hard mask that replaces every sub-horizon sample with a ground
       sentinel, so no sun light leaks below the horizon.
   When the layer is switched off the renderer reduces to the original
   gradient sphere (checked in the self-test).

   Two evaluation paths are kept on purpose: sky_analytic() is the exact
   per-pixel formula, sky_dome() is what the sphere renderer produces.
   The self-test compares them region by region. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#define PI      3.14159265358979323846f
#define HALF_PI (PI * 0.5f)
#define TWO_PI (2.0f * PI)

typedef struct {
    float c[3];
} Rgb;

typedef struct {
    Rgb   zenith;     /* gradient colour overhead */
    Rgb   horizon;    /* gradient colour at the horizon */
    Rgb   ground;     /* mask sentinel below the horizon */
    int   sun_on;     /* layer toggle: 0 == legacy gradient sphere */
    float az;         /* sun azimuth, rad, normalised to [0, 2pi) */
    float el;         /* sun elevation, rad, must be in [0, pi/2] */
    float disk_r;     /* sun disk angular radius, rad */
    float disk_soft;  /* disk soft-edge width, rad */
    float sun_power;  /* disk intensity */
    float halo_exp;   /* halo falloff exponent on cos(angular distance) */
    float halo_lo;    /* halo strength with the sun on the horizon */
    float halo_hi;    /* halo strength with the sun overhead */
    int   dome_segs;  /* sphere grid: longitude segments */
    int   dome_rings; /* sphere grid: latitude rings */
} Sky;

/* ---------- small math helpers ---------- */

static float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static float mixf(float a, float b, float t) { return a + (b - a) * t; }

static float smoothstep(float e0, float e1, float x) {
    float t = clampf((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static void rgb_mix(const Rgb* a, const Rgb* b, float t, Rgb* o) {
    for (int k = 0; k < 3; k++) o->c[k] = mixf(a->c[k], b->c[k], t);
}

static void rgb_add_clamp(Rgb* o, const Rgb* a, float s) {
    for (int k = 0; k < 3; k++) o->c[k] = clampf(o->c[k] + a->c[k] * s, 0.0f, 1.0f);
}

/* ---------- configuration ---------- */

static void sky_default(Sky* s) {
    s->zenith     = (Rgb){{0.16f, 0.31f, 0.55f}};
    s->horizon    = (Rgb){{0.72f, 0.80f, 0.90f}};
    s->ground     = (Rgb){{0.10f, 0.09f, 0.08f}};
    s->sun_on     = 0;
    s->az         = 0.0f;
    s->el         = 0.0f;
    s->disk_r     = 0.045f;
    s->disk_soft  = 0.008f;
    s->sun_power  = 1.15f;
    s->halo_exp   = 7.0f;
    s->halo_lo    = 0.55f;
    s->halo_hi    = 0.12f;
    s->dome_segs  = 72;
    s->dome_rings = 48;
}

/* Place the sun by azimuth/elevation. Elevation must stay inside the
   horizon range [0, pi/2]; anything else is rejected and the config is
   left untouched. Azimuth is wrapped, so it is never rejected. */
static int sky_set_sun(Sky* s, float az, float el) {
    if (isnan(az) || isinf(az) || isnan(el) || isinf(el)) return -1;
    if (el < 0.0f || el > HALF_PI) return -1;
    az = fmodf(az, TWO_PI);
    if (az < 0.0f) az += TWO_PI;
    s->az = az;
    s->el = el;
    return 0;
}

/* ---------- analytic sky model ---------- */

/* Legacy gradient: zenith->horizon by view elevation, clamped below the
   horizon so the old sphere shows horizon colour underneath. */
static void sky_base(const Sky* s, float el, Rgb* o) {
    float t = clampf(el / HALF_PI, 0.0f, 1.0f);
    t = powf(t, 0.55f);
    rgb_mix(&s->horizon, &s->zenith, t, o);
}

static void sun_direction(const Sky* s, float d[3]) {
    float ce = cosf(s->el);
    d[0] = ce * sinf(s->az);
    d[1] = sinf(s->el);
    d[2] = ce * cosf(s->az);
}

/* Halo scalar only; kept separate so the elevation dependency can be
   tested at a fixed view direction. */
static float halo_term(const Sky* s, float cosg) {
    float h = s->el / HALF_PI;
    float strength = mixf(s->halo_lo, s->halo_hi, h);
    return strength * powf(fmaxf(cosg, 0.0f), s->halo_exp);
}

/* Sun-only contribution (disk + elevation-driven halo), no mask. */
static void sky_sun(const Sky* s, const float v[3], Rgb* add) {
    float sd[3];
    sun_direction(s, sd);
    float cosg = clampf(v[0] * sd[0] + v[1] * sd[1] + v[2] * sd[2], -1.0f, 1.0f);
    float gamma = acosf(cosg);

    /* disk: 1 at the centre, soft edge around disk_r */
    float disk = 1.0f - smoothstep(s->disk_r - s->disk_soft,
                                   s->disk_r + s->disk_soft, gamma);

    /* colour temperature: warm at the horizon, white overhead */
    float h = s->el / HALF_PI;
    Rgb warm  = {{1.00f, 0.52f, 0.22f}};
    Rgb white = {{1.00f, 0.97f, 0.92f}};
    Rgb sun_col;
    rgb_mix(&warm, &white, h, &sun_col);

    float halo = halo_term(s, cosg);
    for (int k = 0; k < 3; k++)
        add->c[k] = sun_col.c[k] * (s->sun_power * disk + halo);
}

/* Full per-pixel analytic formula, including the horizon mask. With
   sun_on == 0 it is exactly sky_base(), same argument derivation. */
static void sky_analytic(const Sky* s, const float v[3], Rgb* o) {
    if (s->sun_on && v[1] < 0.0f) {
        *o = s->ground;
        return;
    }
    float el = asinf(clampf(v[1], -1.0f, 1.0f));
    sky_base(s, el, o);
    if (s->sun_on) {
        Rgb add;
        sky_sun(s, v, &add);
        rgb_add_clamp(o, &add, 1.0f);
    }
}

/* ---------- gradient-sphere renderer ---------- */

static void direction(float az, float el, float v[3]) {
    float ce = cosf(el);
    v[0] = ce * sinf(az);
    v[1] = sinf(el);
    v[2] = ce * cosf(az);
}

/* What the engine draws: base colour interpolated from sphere vertices,
   the sun evaluated analytically per pixel (never baked into vertices),
   and the mask applied as a hard step. With sun_on == 0 this is exactly
   the pre-change gradient sphere. The base gradient is azimuth-free, so
   the longitude grid only wraps the seam; interpolation is vertical. */
static void sky_dome(const Sky* s, const float v[3], Rgb* o) {
    if (s->sun_on && v[1] < 0.0f) {
        *o = s->ground;
        return;
    }

    float el = asinf(clampf(v[1], -1.0f, 1.0f));
    int rings = s->dome_rings;
    float fy = (el + HALF_PI) / PI * rings;
    int j0 = (int)floorf(fy);
    float ty = fy - floorf(fy);
    if (j0 < 0) j0 = 0;
    if (j0 > rings - 1) j0 = rings - 1;
    int j1 = j0 + 1;
    if (j1 > rings) j1 = rings;

    Rgb c0, c1;
    sky_base(s, -HALF_PI + PI * j0 / rings, &c0);
    sky_base(s, -HALF_PI + PI * j1 / rings, &c1);
    rgb_mix(&c0, &c1, ty, o);

    if (s->sun_on) {
        Rgb add;
        sky_sun(s, v, &add);
        rgb_add_clamp(o, &add, 1.0f);
    }
}

/* ---------- self-test ---------- */

#define NA 240
#define NE 120
#define NP (NA * NE)

/* Bands are classified by view elevation first, so interpolation error
   and sun accuracy never share a bucket. */
enum { R_BELOW = 0, R_HORIZON, R_UPPER, R_ZENITH, R_HALO, R_DISK, R_N };

static const char* REGION_NAME[R_N] = {
    "below", "horizon", "upper", "zenith", "halo", "sun_disk"
};

typedef struct {
    long n;
    double sum;
    float maxv;
} RegionStat;

static int region_of(const Sky* s, const float v[3], float gamma, float el) {
    if (v[1] < 0.0f) return R_BELOW;
    if (s->sun_on && gamma <= s->disk_r + s->disk_soft) return R_DISK;
    if (s->sun_on && gamma < 0.6f && el >= HALF_PI / 6.0f) return R_HALO;
    if (el >= HALF_PI / 3.0f) return R_ZENITH;       /* >= 60 deg */
    if (el >= HALF_PI / 6.0f) return R_UPPER;        /* >= 30 deg */
    return R_HORIZON;
}

static float luminance(const Rgb* c) {
    return 0.2126f * c->c[0] + 0.7152f * c->c[1] + 0.0722f * c->c[2];
}

int main(void) {
    int fails = 0;

    /* 1. elevation outside the horizon range is rejected */
    Sky cfg;
    sky_default(&cfg);
    float bad_el[] = { -0.01f, HALF_PI + 0.01f, -1.0f, 2.0f, NAN };
    int reject_ok = 1;
    for (size_t i = 0; i < sizeof(bad_el) / sizeof(bad_el[0]); i++)
        if (sky_set_sun(&cfg, 0.5f, bad_el[i]) != -1) reject_ok = 0;
    if (sky_set_sun(&cfg, 12.3f, 0.0f) != 0) reject_ok = 0;      /* boundary */
    if (sky_set_sun(&cfg, 12.3f, HALF_PI) != 0) reject_ok = 0;   /* boundary */
    if (sky_set_sun(&cfg, NAN, 0.5f) != -1) reject_ok = 0;

    /* rejected call must not mutate the config */
    Sky probe, saved;
    sky_default(&probe);
    sky_set_sun(&probe, 0.785f, 0.524f);
    saved = probe;
    sky_set_sun(&probe, 1.0f, -0.5f);
    int unchanged_ok = memcmp(&probe, &saved, sizeof probe) == 0;

    /* 2. render frames through both paths */
    Rgb* dome_on   = malloc(sizeof(Rgb) * NP);
    Rgb* ref_on    = malloc(sizeof(Rgb) * NP);
    Rgb* dome_off  = malloc(sizeof(Rgb) * NP);
    Rgb* ref_off   = malloc(sizeof(Rgb) * NP);
    Rgb* dome_off2 = malloc(sizeof(Rgb) * NP);
    if (!dome_on || !ref_on || !dome_off || !ref_off || !dome_off2) return 2;

    Sky on = cfg, off = cfg, off2 = cfg;
    sky_set_sun(&on, PI / 4.0f, HALF_PI / 3.0f);   /* az 45, el 30 */
    on.sun_on = 1;
    off.sun_on = 0;
    off2.sun_on = 0;
    sky_set_sun(&off2, 5.0f, 0.07f);               /* different sun, still off */

    RegionStat st[R_N] = {0};
    int mask_ok = 1, bitexact_ok = 1, place_ok = 0;
    float best_lum = -1.0f;
    int best_idx = -1;
    float sun_sd[3];
    sun_direction(&on, sun_sd);

    for (int p = 0; p < NP; p++) {
        int ia = p % NA;
        int ie = p / NA;
        float az = TWO_PI * ia / NA;
        float el = -HALF_PI + PI * (ie + 0.5f) / NE;
        float v[3];
        direction(az, el, v);
        float vel = asinf(clampf(v[1], -1.0f, 1.0f));

        sky_dome(&on, v, &dome_on[p]);
        sky_analytic(&on, v, &ref_on[p]);
        sky_dome(&off, v, &dome_off[p]);
        sky_dome(&off2, v, &dome_off2[p]);
        sky_analytic(&off, v, &ref_off[p]);

        /* layer off: the analytic path must equal the legacy base
           gradient evaluated with the same v-derived elevation */
        Rgb base_direct;
        sky_base(&off, vel, &base_direct);
        if (memcmp(&ref_off[p], &base_direct, sizeof(Rgb)) != 0) bitexact_ok = 0;

        /* mask: sub-horizon samples are the ground sentinel, both paths */
        if (v[1] < 0.0f) {
            if (memcmp(&dome_on[p], &on.ground, sizeof(Rgb)) != 0 ||
                memcmp(&ref_on[p],  &on.ground, sizeof(Rgb)) != 0)
                mask_ok = 0;
        }

        /* region-wise error: sphere renderer vs analytic reference */
        float cosg = clampf(v[0]*sun_sd[0] + v[1]*sun_sd[1] + v[2]*sun_sd[2],
                            -1.0f, 1.0f);
        float gamma = acosf(cosg);
        int r = region_of(&on, v, gamma, vel);
        for (int k = 0; k < 3; k++) {
            float d = fabsf(dome_on[p].c[k] - ref_on[p].c[k]);
            st[r].sum += d;
            if (d > st[r].maxv) st[r].maxv = d;
        }
        st[r].n++;

        float lum = luminance(&ref_on[p]);
        if (v[1] >= 0.0f && lum > best_lum) {
            best_lum = lum;
            best_idx = p;
        }
    }

    /* 3. toggle invariance: sun fields must not matter while layer is off */
    int toggle_ok = memcmp(dome_off, dome_off2, sizeof(Rgb) * NP) == 0;

    /* 4. brightest pixel must sit at the requested sun position */
    if (best_idx >= 0) {
        float baz = TWO_PI * (best_idx % NA) / NA;
        float bel = -HALF_PI + PI * (best_idx / NA + 0.5f) / NE;
        float bv[3];
        direction(baz, bel, bv);
        float cosd = clampf(bv[0]*sun_sd[0] + bv[1]*sun_sd[1] + bv[2]*sun_sd[2],
                            -1.0f, 1.0f);
        place_ok = acosf(cosd) < (2.0f * TWO_PI / NA) + on.disk_r;
    }

    /* 5. halo gets stronger as the sun drops toward the horizon.
       Compare the halo scalar at one fixed angular offset, so the disk
       and view direction cannot contaminate the result. */
    Sky low = on, high = on;
    sky_set_sun(&low, on.az, 0.07f);
    sky_set_sun(&high, on.az, 1.4f);
    float fixed_cosg = cosf(0.25f);              /* 14.3 deg off the sun */
    float halo_lo_v = halo_term(&low, fixed_cosg);
    float halo_hi_v = halo_term(&high, fixed_cosg);
    int halo_ok = halo_lo_v > halo_hi_v * 2.0f;

    /* 6. region report, errors scaled to 8-bit channel units */
    printf("sky: %d samples, dome %dx%d, sun az=%.1fdeg el=%.1fdeg\n",
           NP, on.dome_segs, on.dome_rings,
           on.az * 180.0f / PI, on.el * 180.0f / PI);
    printf("%-9s %8s %12s %12s\n", "region", "n", "mean(/255)", "max(/255)");
    int region_ok = 1;
    for (int r = 0; r < R_N; r++) {
        double mean = st[r].n ? st[r].sum / (3.0 * st[r].n) * 255.0 : 0.0;
        float maxv = st[r].maxv * 255.0f;
        printf("%-9s %8ld %12.4f %12.4f\n", REGION_NAME[r], st[r].n, mean, maxv);
        if (r == R_BELOW) {
            if (st[r].maxv != 0.0f) region_ok = 0;            /* mask is exact */
        } else if (r == R_DISK || r == R_HALO) {
            if (maxv >= 1.5f) region_ok = 0;                  /* per-pixel sun */
        } else {
            if (mean >= 1.0f || maxv >= 6.0f) region_ok = 0;  /* gradient interp */
        }
    }
    printf("layer-off: analytic==base bit-exact=%d; dome independent of sun "
           "fields=%d\n", bitexact_ok, toggle_ok);
    printf("halo scalar low-sun=%.4f high-sun=%.4f (same angular offset)\n",
           halo_lo_v, halo_hi_v);
    printf("reject_ok=%d unchanged_ok=%d toggle_ok=%d bitexact_ok=%d "
           "mask_ok=%d place_ok=%d halo_ok=%d region_ok=%d\n",
           reject_ok, unchanged_ok, toggle_ok, bitexact_ok, mask_ok,
           place_ok, halo_ok, region_ok);

    if (!reject_ok || !unchanged_ok || !toggle_ok || !bitexact_ok ||
        !mask_ok || !place_ok || !halo_ok || !region_ok)
        fails = 1;

    free(dome_on); free(ref_on); free(dome_off);
    free(ref_off); free(dome_off2);
    printf("%s\n", fails ? "FAIL" : "ALL OK");
    return fails;
}
