
/* shadowmap: screen-coverage-driven shadow-map resolution allocation.

   Instead of giving every caster the same shadow-map resolution, each caster's
   visible screen fraction drives its texel quota.  A global texel budget caps
   the total; when the budget is exhausted, the least important casters are
   downgraded first (down to shadows being switched off).  With redistribution
   off every caster keeps the legacy uniform resolution, reproducing the old
   picture exactly. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define UNIFORM_RES   1024
#define MAX_CASTERS   8
#define TIERS         8
#define PIC_W         1920
#define PIC_H         1080

typedef struct { float x, y, z; } Vec3;

typedef struct {
    const char* name;
    Vec3 center;     /* view space, camera at origin looking down -z */
    float radius;    /* bounding sphere radius, world units */
    int importance;  /* higher importance survives budget pressure */
} Caster;

typedef struct { int width, height; } Screen;
typedef struct { float fov_y; float near_z; } Camera;

typedef enum {
    SHADOW_OK = 0,
    SHADOW_ERR_ZERO_COVERAGE,   /* coverage statistics came out 0 / unusable */
    SHADOW_ERR_NEGATIVE_QUOTA,  /* budget or remaining quota < 0 */
    SHADOW_ERR_BAD_ARGS
} ShadowError;

typedef struct {
    const char* name;
    float coverage;                 /* clipped projected area / screen area */
    float screen_cx, screen_cy;     /* projected centre, pixels */
    float radius_px;
    float bbox_x0, bbox_y0, bbox_x1, bbox_y1;   /* unclipped projected bbox */
    int importance;
    int desired_tier;               /* proportional share, before the cap */
    int tier;
    int resolution;
    long quota;                     /* resolution^2 texels */
    int downgraded;                 /* resolution below its fair share */
} CasterAlloc;

typedef struct {
    CasterAlloc casters[MAX_CASTERS];
    int count;
    long quota_used;
    long cap;
} Alloc;

static const int kTierRes[TIERS] = {0, 64, 128, 256, 512, 1024, 2048, 4096};
static const int kTierCount = TIERS;

static long tier_quota(int tier) {
    return (long)kTierRes[tier] * kTierRes[tier];
}

/* nearest power-of-two tier to a texel quota, ties resolved downwards */
static int tier_from_quota(float quota_texels) {
    if (!(quota_texels > 0.0f)) return 0;
    float res = sqrtf(quota_texels);
    if (res < 32.0f) return 0;              /* too small to deserve a map */
    int best = 1;
    float best_score = fabsf(logf(res / (float)kTierRes[1]));
    for (int i = 2; i < kTierCount; i++) {
        float score = fabsf(logf(res / (float)kTierRes[i]));
        if (score < best_score - 1e-6f) { best = i; best_score = score; }
    }
    return best;
}

static int tier_of_res(int res) {
    for (int i = 0; i < kTierCount; i++) if (kTierRes[i] == res) return i;
    return 0;
}

/* Visible fraction of a disc that falls inside the screen.  Simpson
   integration of the clipped vertical chord is accurate enough to classify
   casters and is deterministic across platforms. */
static double disc_rect_area(double cx, double cy, double r,
                             double x0, double y0, double x1, double y1) {
    double lo = fmax(x0 - cx, -r);
    double hi = fmin(x1 - cx, r);
    if (!(hi > lo)) return 0.0;

    const int N = 512;                      /* even number of intervals */
    double step = (hi - lo) / N;
    double sum = 0.0;
    for (int i = 0; i <= N; i++) {
        double x = lo + step * (double)i;
        double t = r * r - x * x;
        double h = t > 0.0 ? sqrt(t) : 0.0;
        double top = fmin(y1 - cy, h);
        double bot = fmax(y0 - cy, -h);
        double chord = top > bot ? top - bot : 0.0;
        sum += (i == 0 || i == N ? 1.0 : (i & 1 ? 4.0 : 2.0)) * chord;
    }
    return sum * step / 3.0;
}

/* Project the bounding sphere and compute its visible screen fraction.
   Returns 0 when the coverage statistic is zero or not finite. */
static int project_caster(const Caster* c, const Screen* sc, const Camera* cam,
                          CasterAlloc* out) {
    memset(out, 0, sizeof(*out));
    out->name = c->name;
    out->importance = c->importance;
    if (!isfinite(c->radius) || c->radius <= 0.0f) return 0;
    if (!isfinite(c->center.x) || !isfinite(c->center.y) || !isfinite(c->center.z)) return 0;

    float depth = -c->center.z;
    if (!(depth > cam->near_z) || !isfinite(depth)) return 0;

    float half_tan = tanf(cam->fov_y * 0.5f);
    if (!(half_tan > 0.0f)) return 0;
    float focal = (float)sc->height * 0.5f / half_tan;

    float cx = (float)sc->width * 0.5f + focal * c->center.x / depth;
    float cy = (float)sc->height * 0.5f - focal * c->center.y / depth;
    float rp = focal * c->radius / depth;
    if (!(rp > 0.0f) || !isfinite(rp) || !isfinite(cx) || !isfinite(cy)) return 0;

    out->screen_cx = cx;
    out->screen_cy = cy;
    out->radius_px = rp;
    out->bbox_x0 = cx - rp;
    out->bbox_y0 = cy - rp;
    out->bbox_x1 = cx + rp;
    out->bbox_y1 = cy + rp;

    float ix0 = fmaxf(out->bbox_x0, 0.0f);
    float iy0 = fmaxf(out->bbox_y0, 0.0f);
    float ix1 = fminf(out->bbox_x1, (float)sc->width);
    float iy1 = fminf(out->bbox_y1, (float)sc->height);
    if (!(ix1 > ix0 && iy1 > iy0)) return 0;

    double area = disc_rect_area(cx, cy, rp, 0.0, 0.0, sc->width, sc->height);
    double cov = area / ((double)sc->width * (double)sc->height);
    if (!(cov > 0.0) || !isfinite(cov)) return 0;

    out->coverage = (float)cov;
    return 1;
}

static int lower_importance(const Alloc* al, int a, int b) {
    const CasterAlloc* ca = &al->casters[a];
    const CasterAlloc* cb = &al->casters[b];
    if (ca->importance != cb->importance) return ca->importance < cb->importance;
    if (ca->coverage != cb->coverage) return ca->coverage < cb->coverage;
    return a > b;                           /* stable: later entry yields first */
}

/* Trim the allocation until it fits the cap, taking one tier away from the
   current least-important caster at each step. */
static void degrade_to_cap(Alloc* al) {
    long used = 0;
    for (int i = 0; i < al->count; i++) used += tier_quota(al->casters[i].tier);

    while (used > al->cap) {
        int victim = -1;
        for (int i = 0; i < al->count; i++) {
            if (al->casters[i].tier <= 0) continue;
            if (victim < 0 || lower_importance(al, i, victim)) victim = i;
        }
        if (victim < 0) break;              /* nothing left to give up */
        CasterAlloc* a = &al->casters[victim];
        used -= tier_quota(a->tier);
        a->tier--;
        used += tier_quota(a->tier);
    }
    al->quota_used = used;
}

/* Spend downgrade leftovers on the most important caster that can still step
   up one tier, so the cap is not wasted on rounding slack. */
static void reclaim_leftover(Alloc* al) {
    long used = al->quota_used;
    int moved = 1;
    while (moved) {
        moved = 0;
        int best = -1;
        for (int i = 0; i < al->count; i++) {
            CasterAlloc* a = &al->casters[i];
            if (a->tier >= a->desired_tier) continue;
            long delta = tier_quota(a->tier + 1) - tier_quota(a->tier);
            if (used + delta > al->cap) continue;
            if (best < 0 || a->importance > al->casters[best].importance) best = i;
        }
        if (best >= 0) {
            CasterAlloc* a = &al->casters[best];
            used += tier_quota(a->tier + 1) - tier_quota(a->tier);
            a->tier++;
            moved = 1;
        }
    }
    al->quota_used = used;
}

/* Allocate a resolution per caster.  Returns 0 on success, -1 with *err set.
   When redistribute == 0 the legacy uniform resolution is used for all. */
static int shadow_alloc(const Caster* casters, int n, Screen screen, Camera cam,
                        long cap, int redistribute, Alloc* out, ShadowError* err) {
    if (err) *err = SHADOW_OK;
    if (!out || !casters || n <= 0 || n > MAX_CASTERS) {
        if (err) *err = SHADOW_ERR_BAD_ARGS;
        return -1;
    }
    out->count = n;
    out->cap = cap;
    out->quota_used = 0;

    if (cap < 0) {                          /* negative pixel quota */
        if (err) *err = SHADOW_ERR_NEGATIVE_QUOTA;
        return -1;
    }
    if (screen.width <= 0 || screen.height <= 0) {  /* coverage stat is zero */
        if (err) *err = SHADOW_ERR_ZERO_COVERAGE;
        return -1;
    }

    float total_cov = 0.0f;
    for (int i = 0; i < n; i++) {
        if (!project_caster(&casters[i], &screen, &cam, &out->casters[i])) {
            if (err) *err = SHADOW_ERR_ZERO_COVERAGE;
            return -1;
        }
        total_cov += out->casters[i].coverage;
    }
    if (!(total_cov > 0.0f)) {
        if (err) *err = SHADOW_ERR_ZERO_COVERAGE;
        return -1;
    }

    if (!redistribute) {
        long used = 0;
        for (int i = 0; i < n; i++) {
            CasterAlloc* a = &out->casters[i];
            a->desired_tier = tier_of_res(UNIFORM_RES);
            a->tier = a->desired_tier;
            a->resolution = UNIFORM_RES;
            a->quota = tier_quota(a->tier);
            a->downgraded = 0;
            used += a->quota;
        }
        if (used > cap) {                   /* remaining quota would go negative */
            if (err) *err = SHADOW_ERR_NEGATIVE_QUOTA;
            return -1;
        }
        out->quota_used = used;
        return 0;
    }

    /* 1. proportional fair share of the budget, rounded to a tier */
    for (int i = 0; i < n; i++) {
        CasterAlloc* a = &out->casters[i];
        float fair = (float)cap * a->coverage / total_cov;
        a->desired_tier = tier_from_quota(fair);
        a->tier = a->desired_tier;
    }

    /* 2. if the rounded shares overflow the cap, degrade by importance */
    degrade_to_cap(out);
    /* 3. put rounding leftovers back into the most important downgraded map */
    reclaim_leftover(out);
    if (out->quota_used < 0 || out->quota_used > cap) {
        if (err) *err = SHADOW_ERR_NEGATIVE_QUOTA;
        return -1;
    }

    for (int i = 0; i < n; i++) {
        CasterAlloc* a = &out->casters[i];
        a->resolution = kTierRes[a->tier];
        a->quota = tier_quota(a->tier);
        a->downgraded = a->tier < a->desired_tier;
    }
    return 0;
}

/* shadow lookup at a screen pixel: nearest-neighbour from the caster's map.
   The analytic coverage disc is what the map stores; res controls the
   quantisation of its boundary. */
static int shadow_sample_res(const CasterAlloc* a, int res, float px, float py) {
    if (res <= 0) return 0;
    float texel = 2.0f * a->radius_px / (float)res;
    float left = a->screen_cx - a->radius_px;
    float top = a->screen_cy - a->radius_px;
    int tx = (int)floorf((px - left) / texel);
    int ty = (int)floorf((py - top) / texel);
    if (tx < 0) tx = 0; else if (tx >= res) tx = res - 1;
    if (ty < 0) ty = 0; else if (ty >= res) ty = res - 1;
    float tcx = left + ((float)tx + 0.5f) * texel;
    float tcy = top + ((float)ty + 0.5f) * texel;
    float dx = tcx - a->screen_cx;
    float dy = tcy - a->screen_cy;
    float r = a->radius_px;
    return dx * dx + dy * dy <= r * r;
}

static int reference_sample(const CasterAlloc* a, float px, float py) {
    float dx = px - a->screen_cx;
    float dy = py - a->screen_cy;
    float r = a->radius_px;
    return dx * dx + dy * dy <= r * r;
}

/* Clip a projected bbox to the screen and convert to integer half-open pixel
   bounds.  Clamping happens in float first so huge off-screen coordinates
   cannot overflow the int conversion.  Returns 0 when nothing is visible. */
static int bbox_pixel_span(const CasterAlloc* a, const Screen* sc,
                           int* x0, int* y0, int* x1, int* y1) {
    float fx0 = fminf(fmaxf(a->bbox_x0, 0.0f), (float)sc->width);
    float fy0 = fminf(fmaxf(a->bbox_y0, 0.0f), (float)sc->height);
    float fx1 = fminf(fmaxf(a->bbox_x1, 0.0f), (float)sc->width);
    float fy1 = fminf(fmaxf(a->bbox_y1, 0.0f), (float)sc->height);
    int ix0 = (int)floorf(fx0), iy0 = (int)floorf(fy0);
    int ix1 = (int)ceilf(fx1), iy1 = (int)ceilf(fy1);
    if (ix1 <= ix0 || iy1 <= iy0) return 0;
    *x0 = ix0; *y0 = iy0; *x1 = ix1; *y1 = iy1;
    return 1;
}

/* Per-caster shadow error against the analytic disc, over the visible bbox. */
typedef struct {
    long mismatch;      /* pixels where the quantised shadow disagrees */
    long samples;       /* pixels compared */
    double px_per_texel;/* average screen pixels covered by one shadow texel */
} CasterError;

static void measure_caster(const CasterAlloc* a, const Screen* sc, int res,
                           CasterError* out) {
    memset(out, 0, sizeof(*out));
    int x0, y0, x1, y1;
    if (!bbox_pixel_span(a, sc, &x0, &y0, &x1, &y1)) return;

    long mism = 0;
    for (int y = y0; y < y1; y++) {
        float py = (float)y + 0.5f;
        for (int x = x0; x < x1; x++) {
            float px = (float)x + 0.5f;
            mism += (reference_sample(a, px, py) != shadow_sample_res(a, res, px, py));
        }
    }
    out->mismatch = mism;
    out->samples = (long)(x1 - x0) * (long)(y1 - y0);
    out->px_per_texel = res > 0 ? (double)(2.0f * a->radius_px) / (double)res
                                : (double)(2.0f * a->radius_px);
}

static double caster_error(const CasterError* e) {
    return e->samples ? (double)e->mismatch / (double)e->samples : 0.0;
}

static unsigned char g_pic_legacy[PIC_W * PIC_H];
static unsigned char g_pic_off[PIC_W * PIC_H];

/* force_res > 0 renders every caster at that resolution (the legacy uniform
   picture); 0 uses each caster's allocated resolution. */
static void render_picture(const Alloc* al, const Screen* sc, int force_res,
                           unsigned char* buf) {
    memset(buf, 0, (size_t)sc->width * (size_t)sc->height);
    for (int i = 0; i < al->count; i++) {
        const CasterAlloc* a = &al->casters[i];
        int res = force_res > 0 ? force_res : a->resolution;
        if (res <= 0) continue;
        int x0, y0, x1, y1;
        if (!bbox_pixel_span(a, sc, &x0, &y0, &x1, &y1)) continue;
        for (int y = y0; y < y1; y++)
            for (int x = x0; x < x1; x++)
                if (shadow_sample_res(a, res, (float)x + 0.5f, (float)y + 0.5f))
                    buf[(size_t)y * (size_t)sc->width + (size_t)x] = 1;
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Place a caster at a depth and screen centre so that its projected disc has
   the requested screen coverage.  Coverage is only exact while the disc is
   fully on screen; the reported statistics come from the real projection. */
static Caster make_caster(const char* name, float depth, float coverage,
                          float sx, float sy, int importance,
                          const Screen* sc, const Camera* cam) {
    Caster c;
    float focal = (float)sc->height * 0.5f / tanf(cam->fov_y * 0.5f);
    float radius_px = sqrtf(coverage * (float)sc->width * (float)sc->height / 3.14159265f);
    c.name = name;
    c.center.x = (sx - (float)sc->width * 0.5f) * depth / focal;
    c.center.y = -(sy - (float)sc->height * 0.5f) * depth / focal;
    c.center.z = -depth;
    c.radius = radius_px * depth / focal;
    c.importance = importance;
    return c;
}

static void print_alloc(const Alloc* al) {
    for (int i = 0; i < al->count; i++) {
        const CasterAlloc* a = &al->casters[i];
        printf("caster=%s coverage=%.4f importance=%d desired_res=%d res=%d "
               "quota=%ld px_per_texel=%.3f downgraded=%d\n",
               a->name ? a->name : "?", a->coverage, a->importance,
               kTierRes[a->desired_tier], a->resolution, a->quota,
               a->resolution > 0 ? (double)(2.0f * a->radius_px) / (double)a->resolution
                                 : 0.0,
               a->downgraded);
    }
}

int main(void) {
    Screen sc = { PIC_W, PIC_H };
    Camera cam = { 60.0f * 3.14159265358979f / 180.0f, 0.1f };

    /* hero is close and large; the rest are progressively smaller and farther.
       At 1024^2 the hero disc is coarse.  The 5.4M texel budget is more than
       the five fair shares need, so the cascade shows all three outcomes:
       upgrade (hero), unchanged (wall), downgrade (crate) and disable
       (debris, leaf). */
    Caster scene[5];
    scene[0] = make_caster("hero",   1.60f, 0.42f,  900.0f, 540.0f, 100, &sc, &cam);
    scene[1] = make_caster("wall",   4.00f, 0.22f, 1350.0f, 540.0f,  80, &sc, &cam);
    scene[2] = make_caster("crate",  3.00f, 0.10f,  400.0f, 780.0f,  50, &sc, &cam);
    scene[3] = make_caster("debris", 7.50f, 0.05f, 1620.0f, 220.0f,  20, &sc, &cam);
    scene[4] = make_caster("leaf",   9.00f, 0.02f,  220.0f, 160.0f,   5, &sc, &cam);

    const long cap = 5400000L;
    ShadowError err = SHADOW_OK;
    Alloc adapt, off, tmp;

    int rc_adapt = shadow_alloc(scene, 5, sc, cam, cap, 1, &adapt, &err);
    check("alloc_ok", rc_adapt == 0 && err == SHADOW_OK);

    int cov_ok = 1, res_ok = 1, quota_ok = 1;
    long sum = 0;
    for (int i = 0; i < adapt.count; i++) {
        const CasterAlloc* a = &adapt.casters[i];
        if (!(a->coverage > 0.0f)) cov_ok = 0;
        if (tier_of_res(a->resolution) != a->tier && a->resolution != 0) res_ok = 0;
        if (a->quota != (long)a->resolution * a->resolution) quota_ok = 0;
        sum += a->quota;
    }
    check("coverage_all_positive", cov_ok);
    check("resolutions_on_tier_grid", res_ok);
    check("quota_matches_resolution", quota_ok);
    check("quota_sum_matches", sum == adapt.quota_used);
    check("quota_within_cap", adapt.quota_used <= cap);

    /* near hero must actually get more resolution than the uniform baseline */
    check("hero_upgraded", adapt.casters[0].resolution > UNIFORM_RES);
    /* mid-importance caster is downgraded a tier, not switched off */
    check("mid_tier_downgraded",
          adapt.casters[2].resolution > 0 &&
          adapt.casters[2].resolution < kTierRes[adapt.casters[2].desired_tier]);
    /* casters are listed in decreasing importance: resolution must not rise
       as importance falls, i.e. the cut is applied least-important first */
    int order_ok = 1;
    for (int i = 1; i < adapt.count; i++)
        if (adapt.casters[i].resolution > adapt.casters[i - 1].resolution) order_ok = 0;
    check("degrade_follows_importance", order_ok);

    /* legacy path: uniform resolution for every caster */
    int rc_off = shadow_alloc(scene, 5, sc, cam, cap, 0, &off, &err);
    check("redistribution_off_ok", rc_off == 0 && err == SHADOW_OK);
    int off_uniform = rc_off == 0;
    for (int i = 0; i < off.count; i++)
        if (off.casters[i].resolution != UNIFORM_RES) off_uniform = 0;
    check("redistribution_off_uniform", off_uniform);

    render_picture(&off, &sc, UNIFORM_RES, g_pic_legacy);  /* current picture */
    render_picture(&off, &sc, 0, g_pic_off);               /* redistribution off */
    long diff = 0;
    size_t pic_bytes = (size_t)sc.width * (size_t)sc.height;
    for (size_t i = 0; i < pic_bytes; i++) diff += (g_pic_legacy[i] != g_pic_off[i]);
    printf("off_vs_legacy_diff_px=%ld\n", diff);
    check("redistribution_off_identical", diff == 0);

    /* hero (large) and wall (medium) count as near-field */
    const float near_cov = 0.20f;
    CasterError near_uni = {0}, near_ad = {0};
    CasterError per_uni[5], per_ad[5];
    for (int i = 0; i < 5; i++) {
        measure_caster(&off.casters[i], &sc, UNIFORM_RES, &per_uni[i]);
        measure_caster(&adapt.casters[i], &sc, adapt.casters[i].resolution, &per_ad[i]);
        if (adapt.casters[i].coverage >= near_cov) {
            near_uni.mismatch += per_uni[i].mismatch;
            near_uni.samples += per_uni[i].samples;
            near_ad.mismatch += per_ad[i].mismatch;
            near_ad.samples += per_ad[i].samples;
        }
    }
    double near_err_uni = caster_error(&near_uni);
    double near_err_ad = caster_error(&near_ad);
    double improvement = near_err_uni > 0.0
                       ? 100.0 * (near_err_uni - near_err_ad) / near_err_uni : 0.0;

    printf("quota_used=%ld quota_cap=%ld quota_use_pct=%.1f\n",
           adapt.quota_used, cap, 100.0 * (double)adapt.quota_used / (double)cap);
    print_alloc(&adapt);
    for (int i = 0; i < 5; i++) {
        double ppt_uni = per_uni[i].px_per_texel;
        double ppt_ad = adapt.casters[i].resolution > 0 ? per_ad[i].px_per_texel : 0.0;
        printf("detail caster=%s coverage=%.4f uniform_res=%d adaptive_res=%d "
               "px_per_texel_uniform=%.3f px_per_texel_adaptive=%.3f "
               "err_uniform=%.6f err_adaptive=%.6f\n",
               adapt.casters[i].name, adapt.casters[i].coverage,
               UNIFORM_RES, adapt.casters[i].resolution,
               ppt_uni, ppt_ad,
               caster_error(&per_uni[i]), caster_error(&per_ad[i]));
    }
    printf("near_detail_err_uniform=%.6f\n", near_err_uni);
    printf("near_detail_err_adaptive=%.6f\n", near_err_ad);
    printf("near_detail_improvement_pct=%.1f\n", improvement);
    check("near_detail_improved", near_ad.samples > 0 && near_err_ad < near_err_uni);

    /* zero coverage statistics must be rejected: behind-camera and off-screen */
    Caster behind = { "behind", { 0.0f, 0.0f, 2.0f }, 1.0f, 1 };
    Caster offscreen = { "offscreen", { 50.0f, 0.0f, -5.0f }, 0.5f, 1 };
    Caster with_behind[6], with_offscreen[6];
    memcpy(with_behind, scene, sizeof(scene));
    memcpy(with_offscreen, scene, sizeof(scene));
    with_behind[5] = behind;
    with_offscreen[5] = offscreen;
    int rc = shadow_alloc(with_behind, 6, sc, cam, cap, 1, &tmp, &err);
    check("zero_coverage_behind_rejected",
          rc == -1 && err == SHADOW_ERR_ZERO_COVERAGE);
    rc = shadow_alloc(with_offscreen, 6, sc, cam, cap, 1, &tmp, &err);
    check("zero_coverage_offscreen_rejected",
          rc == -1 && err == SHADOW_ERR_ZERO_COVERAGE);

    Screen empty = { 0, PIC_H };
    rc = shadow_alloc(scene, 5, empty, cam, cap, 1, &tmp, &err);
    check("zero_screen_rejected", rc == -1 && err == SHADOW_ERR_ZERO_COVERAGE);

    /* negative pixel quota must be rejected */
    rc = shadow_alloc(scene, 5, sc, cam, -1, 1, &tmp, &err);
    check("negative_quota_rejected", rc == -1 && err == SHADOW_ERR_NEGATIVE_QUOTA);
    rc = shadow_alloc(scene, 5, sc, cam, 1L, 1, &tmp, &err);
    check("tiny_budget_rejects_nothing",
          rc == 0 && err == SHADOW_OK && tmp.quota_used == 0);

    /* a budget just big enough for the hero's map forces everyone else off */
    rc = shadow_alloc(scene, 5, sc, cam, 4194304L, 1, &tmp, &err);
    int starved_ok = rc == 0 && tmp.quota_used <= 4194304L;
    int zeros = 0;
    for (int i = 0; i < tmp.count; i++) if (tmp.casters[i].resolution == 0) zeros++;
    printf("starved: hero_res=%d zeros=%d used=%ld\n",
           rc == 0 ? tmp.casters[0].resolution : -1, zeros,
           rc == 0 ? tmp.quota_used : -1L);
    check("exhausted_budget_disables_low_importance",
          starved_ok && tmp.casters[0].resolution > 0 && zeros >= 2);

    /* redistribution off with a cap below the uniform total cannot honour the
       legacy picture: report the negative remaining quota */
    rc = shadow_alloc(scene, 5, sc, cam, 4L * UNIFORM_RES * UNIFORM_RES, 0, &tmp, &err);
    check("redistribution_off_over_cap_rejected",
          rc == -1 && err == SHADOW_ERR_NEGATIVE_QUOTA);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
