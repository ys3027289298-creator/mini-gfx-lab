/* softshadow: contact-hardening soft shadows.

   The blur radius is driven by the occluder-to-receiver distance: sharp at
   contact, wider farther away.  The radius curve is validated up front; bad
   values are rejected with a specific error instead of being clamped or
   otherwise rounded into a value that merely looks plausible.

   A hand-tuned per-band radius table produces the reference image.  The
   curve-driven render is compared with it pixel by pixel, and edge gradients
   are reported per distance band. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define IMG_W 192
#define IMG_H 64
#define STRIPE_PERIOD 32       /* hard shadow pattern: alternating stripes */
#define D_MAX 8.0f             /* world distance at the far edge of the scene */
#define MAX_BLUR_RADIUS 10.0f  /* hard limit; values above are an error */
#define CONTACT_RADIUS_MAX 1.0f
#define FAR_RADIUS_MIN 4.0f
#define CURVE_MAX_POINTS 12

#define REF_BANDS 16           /* hand-tuned reference: one radius per band */
#define MARGIN 10              /* ignore border samples in edge analysis */
#define EDGE_GRAD_MIN 0.01f
#define DIFF_OVER 0.10f

typedef enum {
    CURVE_OK = 0,
    CURVE_ERR_TOO_FEW,
    CURVE_ERR_TOO_MANY,
    CURVE_ERR_NONFINITE,
    CURVE_ERR_NEGATIVE_DISTANCE,
    CURVE_ERR_NOT_AT_CONTACT,
    CURVE_ERR_UNSORTED,
    CURVE_ERR_NEGATIVE_RADIUS,
    CURVE_ERR_RADIUS_TOO_LARGE,
    CURVE_ERR_FLAT,
    CURVE_ERR_DECREASING,
    CURVE_ERR_CONTACT_TOO_SOFT,
    CURVE_ERR_FAR_TOO_TIGHT
} CurveError;

typedef struct { float dist; float radius; } CurvePoint;

typedef struct {
    CurvePoint pts[CURVE_MAX_POINTS];
    int count;
    int valid;
} RadiusCurve;

typedef struct { float px[IMG_W * IMG_H]; } Image;

static const char* curve_error_str(CurveError e) {
    switch (e) {
    case CURVE_OK: return "ok";
    case CURVE_ERR_TOO_FEW: return "too_few_points";
    case CURVE_ERR_TOO_MANY: return "too_many_points";
    case CURVE_ERR_NONFINITE: return "nonfinite";
    case CURVE_ERR_NEGATIVE_DISTANCE: return "negative_distance";
    case CURVE_ERR_NOT_AT_CONTACT: return "not_at_contact";
    case CURVE_ERR_UNSORTED: return "unsorted";
    case CURVE_ERR_NEGATIVE_RADIUS: return "negative_radius";
    case CURVE_ERR_RADIUS_TOO_LARGE: return "radius_too_large";
    case CURVE_ERR_FLAT: return "flat";
    case CURVE_ERR_DECREASING: return "decreasing";
    case CURVE_ERR_CONTACT_TOO_SOFT: return "contact_too_soft";
    case CURVE_ERR_FAR_TOO_TIGHT: return "far_too_tight";
    }
    return "unknown";
}

/* Validate without touching the curve.  Every failure is a hard error: the
   caller never gets a repaired radius. */
static CurveError curve_validate(const RadiusCurve* c) {
    if (c->count < 2) return CURVE_ERR_TOO_FEW;
    if (c->count > CURVE_MAX_POINTS) return CURVE_ERR_TOO_MANY;

    for (int i = 0; i < c->count; i++) {
        if (!isfinite(c->pts[i].dist) || !isfinite(c->pts[i].radius))
            return CURVE_ERR_NONFINITE;
        if (c->pts[i].dist < 0.0f) return CURVE_ERR_NEGATIVE_DISTANCE;
        if (c->pts[i].radius < 0.0f) return CURVE_ERR_NEGATIVE_RADIUS;
        if (c->pts[i].radius > MAX_BLUR_RADIUS) return CURVE_ERR_RADIUS_TOO_LARGE;
    }
    if (c->pts[0].dist != 0.0f) return CURVE_ERR_NOT_AT_CONTACT;
    for (int i = 1; i < c->count; i++) {
        if (c->pts[i].dist <= c->pts[i - 1].dist) return CURVE_ERR_UNSORTED;
        if (c->pts[i].radius < c->pts[i - 1].radius) return CURVE_ERR_DECREASING;
        if (c->pts[i].radius == c->pts[i - 1].radius) return CURVE_ERR_FLAT;
    }
    if (c->pts[0].radius > CONTACT_RADIUS_MAX) return CURVE_ERR_CONTACT_TOO_SOFT;
    if (c->pts[c->count - 1].radius < FAR_RADIUS_MIN) return CURVE_ERR_FAR_TOO_TIGHT;
    return CURVE_OK;
}

/* Copy the points only if the whole curve validates.  On any error the struct
   is left invalid and empty -- no partial or clamped curve survives. */
static CurveError curve_init(RadiusCurve* c, const CurvePoint* pts, int n) {
    memset(c, 0, sizeof(*c));
    c->count = n;
    if (n > 0 && n <= CURVE_MAX_POINTS) memcpy(c->pts, pts, sizeof(*pts) * (size_t)n);
    CurveError e = curve_validate(c);
    if (e != CURVE_OK) {
        memset(c, 0, sizeof(*c));
        return e;
    }
    c->valid = 1;
    return CURVE_OK;
}

/* Evaluate inside the curve domain.  Out-of-domain samples are an error, not
   an extrapolation or a silent clamp. */
static int curve_eval(const RadiusCurve* c, float d, float* out) {
    if (!c->valid || c->count < 2) return -1;
    const float first = c->pts[0].dist;
    const float last = c->pts[c->count - 1].dist;
    if (d < first - 1e-5f || d > last + 1e-5f) return -1;
    if (d < first) d = first;
    if (d > last) d = last;
    for (int i = 1; i < c->count; i++) {
        if (d <= c->pts[i].dist) {
            const CurvePoint* a = &c->pts[i - 1];
            const CurvePoint* b = &c->pts[i];
            float span = b->dist - a->dist;
            float u = span > 0.0f ? (d - a->dist) / span : 0.0f;
            *out = a->radius + (b->radius - a->radius) * u;
            return 0;
        }
    }
    *out = c->pts[c->count - 1].radius;
    return 0;
}

/* Scene: a receiver plane receding left to right, so occluder distance grows
   with x.  The hard shadow is a stripe pattern whose edges sit at several
   distances, which makes the hardening measurable per band. */
static void build_distance_field(float* d) {
    for (int y = 0; y < IMG_H; y++)
        for (int x = 0; x < IMG_W; x++)
            d[y * IMG_W + x] = D_MAX * (float)x / (float)(IMG_W - 1);
}

static float hard_shadow(int x) {
    return ((x / STRIPE_PERIOD) & 1) ? 1.0f : 0.0f;   /* 1 = lit */
}

/* Gaussian blur whose sigma is half the requested radius; the image is
   constant in y, so the 2D kernel reduces to this 1D pass.  Radius is in
   pixels.  Any radius outside the supported range is a hard error. */
static int blur(const float* radius, Image* out) {
    for (int i = 0; i < IMG_W * IMG_H; i++) {
        float r = radius[i];
        if (!isfinite(r) || r < 0.0f || r > MAX_BLUR_RADIUS) return -1;
    }
    for (int y = 0; y < IMG_H; y++) {
        for (int x = 0; x < IMG_W; x++) {
            float r = radius[y * IMG_W + x];
            float sigma = 0.5f * r;
            if (sigma < 1e-4f) {
                out->px[y * IMG_W + x] = hard_shadow(x);
                continue;
            }
            int ir = (int)ceilf(3.0f * sigma);
            float sum = 0.0f, wsum = 0.0f;
            for (int dx = -ir; dx <= ir; dx++) {
                int sx = x + dx;
                if (sx < 0) sx = 0;
                if (sx >= IMG_W) sx = IMG_W - 1;
                float w = expf(-0.5f * (float)(dx * dx) / (sigma * sigma));
                sum += w * hard_shadow(sx);
                wsum += w;
            }
            out->px[y * IMG_W + x] = sum / wsum;
        }
    }
    return 0;
}

static int render_curved(const RadiusCurve* c, const float* dist, Image* out) {
    if (!c->valid || curve_validate(c) != CURVE_OK) return -1;
    float* radius = (float*)malloc(sizeof(float) * IMG_W * IMG_H);
    if (!radius) return -1;
    int ok = 1;
    for (int i = 0; i < IMG_W * IMG_H && ok; i++) {
        float r;
        if (curve_eval(c, dist[i], &r) != 0) ok = 0;
        else if (!isfinite(r) || r < 0.0f || r > MAX_BLUR_RADIUS) ok = 0;
        else radius[i] = r;
    }
    /* nothing is written to out unless every sample validated */
    if (ok) ok = blur(radius, out) == 0;
    free(radius);
    return ok ? 0 : -1;
}

/* Hand-tuned radius at the centre of each reference band, dialled in by hand
   at 0.05 px granularity.  These are authored independently of the curve --
   they are not sampled from it -- so the pixel comparison tests the curve
   against an artist's target rather than against itself. */
static const float kRefRadii[REF_BANDS] = {
    0.55f, 0.70f, 0.90f, 1.20f, 1.50f, 1.80f, 2.20f, 2.60f,
    3.05f, 3.50f, 3.95f, 4.45f, 5.00f, 5.55f, 6.10f, 6.70f
};

static int render_reference(const float* dist, Image* out) {
    float* radius = (float*)malloc(sizeof(float) * IMG_W * IMG_H);
    if (!radius) return -1;
    const float band_w = D_MAX / (float)REF_BANDS;
    for (int i = 0; i < IMG_W * IMG_H; i++) {
        int b = (int)(dist[i] / band_w);
        if (b < 0) b = 0;
        if (b >= REF_BANDS) b = REF_BANDS - 1;
        radius[i] = kRefRadii[b];
    }
    int ok = blur(radius, out);
    free(radius);
    return ok;
}

typedef struct { float max_abs, mean_abs; int over; } DiffStats;

static DiffStats compare(const Image* a, const Image* b) {
    DiffStats s = { 0.0f, 0.0f, 0 };
    double sum = 0.0;
    for (int i = 0; i < IMG_W * IMG_H; i++) {
        float d = fabsf(a->px[i] - b->px[i]);
        sum += d;
        if (d > s.max_abs) s.max_abs = d;
        if (d > DIFF_OVER) s.over++;
    }
    s.mean_abs = (float)(sum / (double)(IMG_W * IMG_H));
    return s;
}

typedef struct {
    int samples;
    float grad_max;
    float grad_mean;
    float penumbra_px;
} BandStat;

static int band_of_x(int x) {
    if (x < IMG_W / 3) return 0;
    if (x < 2 * IMG_W / 3) return 1;
    return 2;
}

/* Edge gradient magnitude on the centre row, gathered per distance band. */
static BandStat band_stat(const Image* img, int band) {
    int y = IMG_H / 2;
    double sum = 0.0;
    int n = 0;
    float mx = 0.0f;
    for (int x = MARGIN; x <= IMG_W - 1 - MARGIN; x++) {
        if (band_of_x(x) != band) continue;
        float g = fabsf(img->px[y * IMG_W + x + 1] - img->px[y * IMG_W + x - 1]) * 0.5f;
        if (g < EDGE_GRAD_MIN) continue;
        sum += g;
        n++;
        if (g > mx) mx = g;
    }
    BandStat s;
    s.samples = n;
    s.grad_max = mx;
    s.grad_mean = n > 0 ? (float)(sum / n) : 0.0f;
    s.penumbra_px = mx > 0.0f ? 1.0f / mx : 0.0f;
    return s;
}

static const char* kBandName[3] = { "near", "mid", "far" };

static void print_band(const char* label, int band, const BandStat* s) {
    float lo = D_MAX * (float)(band * IMG_W / 3) / (float)(IMG_W - 1);
    float hi = D_MAX * (float)((band + 1) * IMG_W / 3) / (float)(IMG_W - 1);
    printf("band=%s:%s dist=[%0.2f,%0.2f) edge_samples=%d "
           "edge_grad_mean=%0.4f edge_grad_max=%0.4f penumbra_px=%0.2f\n",
           label, kBandName[band], lo, hi, s->samples,
           s->grad_mean, s->grad_max, s->penumbra_px);
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void expect_reject(const char* name, const CurvePoint* pts, int n, CurveError want) {
    RadiusCurve c;
    CurveError got = curve_init(&c, pts, n);
    char buf[80];
    snprintf(buf, sizeof(buf), "reject_%s", name);
    check(buf, got == want);
    if (got != want)
        printf("  want=%s got=%s\n", curve_error_str(want), curve_error_str(got));
    snprintf(buf, sizeof(buf), "no_curve_after_reject_%s", name);
    check(buf, c.count == 0 && c.valid == 0);
}

int main(void) {
    /* Piecewise-linear curve snapped to the target look: 0.5 px at contact,
       7 px at the far edge of the scene. */
    static const CurvePoint kCurvePts[] = {
        { 0.0f, 0.500f }, { 1.0f, 0.787f }, { 2.0f, 1.312f },
        { 3.0f, 1.993f }, { 4.0f, 2.798f }, { 5.0f, 3.711f },
        { 6.0f, 4.722f }, { 7.0f, 5.820f }, { 8.0f, 7.000f }
    };
    RadiusCurve curve;
    CurveError ce = curve_init(&curve, kCurvePts,
                                (int)(sizeof(kCurvePts) / sizeof(kCurvePts[0])));
    check("curve_valid", ce == CURVE_OK);
    if (ce != CURVE_OK) {
        printf("curve_error=%s\nall_ok=0\n", curve_error_str(ce));
        return 1;
    }

    float* dist = (float*)malloc(sizeof(float) * IMG_W * IMG_H);
    if (!dist) return 1;
    build_distance_field(dist);

    Image curve_img, ref_img, uniform_img;
    int rc_curve = render_curved(&curve, dist, &curve_img);
    int rc_ref = render_reference(dist, &ref_img);
    check("render_curve_ok", rc_curve == 0);
    check("render_reference_ok", rc_ref == 0);
    if (rc_curve != 0 || rc_ref != 0) {
        free(dist);
        printf("all_ok=0\n");
        return 1;
    }

    DiffStats ds = compare(&curve_img, &ref_img);
    printf("pixel_match=max_abs=%0.4f mean_abs=%0.5f pixels_over_%0.2f=%d\n",
           ds.max_abs, ds.mean_abs, DIFF_OVER, ds.over);
    check("pixel_match_reference", ds.max_abs <= 0.05f && ds.mean_abs <= 0.002f && ds.over == 0);

    /* Negative control: a curve can pass every range check and still be the
       wrong shape.  A linear falloff is valid input, so only the pixel
       comparison can catch it -- and it must. */
    static const CurvePoint kLinearPts[] = { { 0.0f, 0.500f }, { 8.0f, 5.000f } };
    RadiusCurve linear;
    check("wrong_curve_passes_validation", curve_init(&linear, kLinearPts, 2) == CURVE_OK);
    Image wrong_img;
    DiffStats wd = { 0 };
    if (linear.valid && render_curved(&linear, dist, &wrong_img) == 0) wd = compare(&wrong_img, &ref_img);
    printf("wrong_curve_vs_reference=max_abs=%0.4f mean_abs=%0.5f pixels_over_%0.2f=%d\n",
           wd.max_abs, wd.mean_abs, DIFF_OVER, wd.over);
    check("pixel_match_rejects_wrong_curve", wd.max_abs > 0.05f && wd.over > 0);

    BandStat cs[3];
    for (int b = 0; b < 3; b++) {
        cs[b] = band_stat(&curve_img, b);
        print_band("curve", b, &cs[b]);
    }
    BandStat rs[3];
    for (int b = 0; b < 3; b++) {
        rs[b] = band_stat(&ref_img, b);
        print_band("reference", b, &rs[b]);
    }

    check("near_sharper_than_mid", cs[0].grad_max > cs[1].grad_max * 1.3f);
    check("mid_sharper_than_far", cs[1].grad_max > cs[2].grad_max * 1.3f);
    check("penumbra_grows_with_distance",
          cs[0].penumbra_px < cs[1].penumbra_px && cs[1].penumbra_px < cs[2].penumbra_px);
    check("reference_shows_same_hardening",
          rs[0].grad_max > rs[1].grad_max && rs[1].grad_max > rs[2].grad_max);

    /* Contrast with the old behaviour: one radius everywhere. */
    float* uniform_radius = (float*)malloc(sizeof(float) * IMG_W * IMG_H);
    if (!uniform_radius) return 1;
    for (int i = 0; i < IMG_W * IMG_H; i++) uniform_radius[i] = 3.0f;
    int rc_uni = blur(uniform_radius, &uniform_img);
    free(uniform_radius);
    check("render_uniform_ok", rc_uni == 0);
    if (rc_uni == 0) {
        BandStat us[3];
        for (int b = 0; b < 3; b++) {
            us[b] = band_stat(&uniform_img, b);
            print_band("uniform", b, &us[b]);
        }
        check("uniform_near_too_soft", us[0].grad_max < cs[0].grad_max);
        check("uniform_far_too_sharp", us[2].grad_max > cs[2].grad_max);
    }

    /* --- radius curve rejection: wrong values are refused, never repaired --- */
    static const CurvePoint too_few[] = { { 0.0f, 0.5f } };
    static const CurvePoint nan_radius[] = {
        { 0.0f, 0.5f }, { 1.0f, NAN }, { 8.0f, 7.0f }
    };
    static const CurvePoint negative_dist[] = { { -1.0f, 0.5f }, { 8.0f, 7.0f } };
    static const CurvePoint not_contact[] = { { 1.0f, 0.5f }, { 8.0f, 7.0f } };
    static const CurvePoint unsorted[] = {
        { 0.0f, 0.5f }, { 6.0f, 5.0f }, { 5.0f, 6.5f }
    };
    static const CurvePoint negative_radius[] = {
        { 0.0f, 0.5f }, { 1.0f, -0.2f }, { 8.0f, 7.0f }
    };
    static const CurvePoint radius_too_large[] = {
        { 0.0f, 0.5f }, { 1.0f, 10.5f }, { 8.0f, 7.0f }
    };
    static const CurvePoint flat[] = {
        { 0.0f, 0.5f }, { 4.0f, 0.5f }, { 8.0f, 7.0f }
    };
    static const CurvePoint decreasing[] = {
        { 0.0f, 0.5f }, { 4.0f, 3.0f }, { 8.0f, 2.0f }
    };
    static const CurvePoint contact_soft[] = { { 0.0f, 3.0f }, { 8.0f, 7.0f } };
    static const CurvePoint far_tight[] = { { 0.0f, 0.5f }, { 8.0f, 3.0f } };
    static const CurvePoint too_many[] = {
        { 0.0f, 0.5f }, { 1.0f, 0.7f }, { 2.0f, 1.0f }, { 3.0f, 1.4f },
        { 4.0f, 1.8f }, { 5.0f, 2.3f }, { 6.0f, 2.9f }, { 7.0f, 3.5f },
        { 8.0f, 4.2f }, { 9.0f, 5.0f }, { 10.0f, 5.8f }, { 11.0f, 6.6f },
        { 12.0f, 7.0f }
    };

    expect_reject("too_few", too_few, 1, CURVE_ERR_TOO_FEW);
    expect_reject("too_many", too_many, 13, CURVE_ERR_TOO_MANY);
    expect_reject("nan_radius", nan_radius, 3, CURVE_ERR_NONFINITE);
    expect_reject("negative_distance", negative_dist, 2, CURVE_ERR_NEGATIVE_DISTANCE);
    expect_reject("not_at_contact", not_contact, 2, CURVE_ERR_NOT_AT_CONTACT);
    expect_reject("unsorted", unsorted, 3, CURVE_ERR_UNSORTED);
    /* Specific error codes prove the bad value was rejected, not clamped. */
    expect_reject("negative_radius_not_clamped", negative_radius, 3, CURVE_ERR_NEGATIVE_RADIUS);
    expect_reject("oversized_radius_not_clamped", radius_too_large, 3, CURVE_ERR_RADIUS_TOO_LARGE);
    expect_reject("flat_radius", flat, 3, CURVE_ERR_FLAT);
    expect_reject("decreasing_radius", decreasing, 3, CURVE_ERR_DECREASING);
    expect_reject("contact_too_soft", contact_soft, 2, CURVE_ERR_CONTACT_TOO_SOFT);
    expect_reject("far_too_tight", far_tight, 2, CURVE_ERR_FAR_TOO_TIGHT);

    /* A valid-but-short curve must not be stretched to cover the scene. */
    static const CurvePoint short_curve[] = { { 0.0f, 0.5f }, { 5.0f, 5.0f } };
    RadiusCurve sc;
    check("short_curve_valid", curve_init(&sc, short_curve, 2) == CURVE_OK);
    Image untouched;
    for (int i = 0; i < IMG_W * IMG_H; i++) untouched.px[i] = -1.0f;
    check("render_rejects_domain_gap", render_curved(&sc, dist, &untouched) == -1);
    check("render_does_not_clamp_domain", untouched.px[0] == -1.0f && untouched.px[IMG_W * IMG_H - 1] == -1.0f);

    /* Mutation after validation is caught by the render-time re-check. */
    RadiusCurve mutated = curve;
    mutated.pts[4].radius = NAN;
    check("render_rejects_mutated_curve", render_curved(&mutated, dist, &untouched) == -1);

    RadiusCurve zeroed;
    memset(&zeroed, 0, sizeof(zeroed));
    check("render_rejects_unvalidated_curve", render_curved(&zeroed, dist, &untouched) == -1);

    /* Boundary acceptance: the limit itself is legal, one step over is not. */
    static const CurvePoint at_limit[] = { { 0.0f, 0.5f }, { 8.0f, MAX_BLUR_RADIUS } };
    RadiusCurve lim;
    check("accepts_radius_at_limit", curve_init(&lim, at_limit, 2) == CURVE_OK);

    free(dist);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
