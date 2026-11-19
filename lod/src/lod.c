
/* lod-selector: screen-space-error LOD selection with hysteresis to avoid
   level thrashing when the camera hovers near a threshold.

   宽高比校正：投影误差按纵向、横向分别计算（横向 fov 由纵向 fov 与视口
   宽高比推得），取较大者驱动选级。只用纵轴时，宽屏下估计会与真实像素
    footprint 偏差最多 ~aspect 倍（例如横向锁定 fov 的 21:9 相机）。 */
#include <stdio.h>
#include <math.h>

#define LEVELS 4

/* 宽高比钳制范围：极端比例下投影不能失真（fov_x 必须保持合法透视） */
#define MIN_ASPECT (1.0f / 16.0f)
#define MAX_ASPECT 16.0f

typedef struct {
    float error[LEVELS];   /* geometric error per level, world units */
    float radius;          /* bounding radius */
    float tau;             /* hysteresis ratio, e.g. 1.1 */
} MeshLod;

typedef struct {
    float fov_y;           /* vertical field of view, radians */
    int w, h;              /* viewport size, pixels */
} View;

typedef struct {
    int level;
    int switches;
    int thrash;            /* switches within one unit of distance change */
} State;

static float clamp_aspect(float a) {
    if (a < MIN_ASPECT) return MIN_ASPECT;
    if (a > MAX_ASPECT) return MAX_ASPECT;
    return a;
}

/* horizontal fov implied by the vertical fov and the viewport aspect */
static float fov_x_of(const View* v) {
    float aspect = clamp_aspect((float)v->w / (float)v->h);
    return 2.0f * atanf(tanf(v->fov_y * 0.5f) * aspect);
}

/* build a view from a locked horizontal fov (Hor+, common on widescreen) */
static View view_from_fov_x(float fov_x, int w, int h) {
    View v;
    v.w = w; v.h = h;
    v.fov_y = 2.0f * atanf(tanf(fov_x * 0.5f) * (float)h / (float)w);
    return v;
}

/* projected size of a world-space error on one axis, in pixels */
static float axis_error(float geom_error, float distance, float fov, int pixels) {
    if (distance <= 0.0f) return 1e9f;
    return geom_error * (float)pixels / (2.0f * distance * tanf(fov * 0.5f));
}

/* aspect-corrected screen-space error: project on both axes, take the max */
static float screen_error(const View* v, float geom_error, float distance) {
    float ey = axis_error(geom_error, distance, v->fov_y, v->h);
    float ex = axis_error(geom_error, distance, fov_x_of(v), v->w);
    return fmaxf(ex, ey);
}

/* ground truth: project the silhouette of a bounding sphere of radius
   geom_error/2 at the given depth, measure its pixel footprint per axis */
static float measured_error(const View* v, float geom_error, float distance) {
    float r = geom_error * 0.5f;
    if (distance <= r) return 1e9f;
    float half_angle = asinf(r / distance);
    float my = tanf(half_angle) / tanf(v->fov_y * 0.5f) * (float)v->h;
    float mx = tanf(half_angle) / tanf(fov_x_of(v) * 0.5f) * (float)v->w;
    return fmaxf(mx, my);
}

/* pick coarsest level whose projected error is below tolerance (no hysteresis) */
static int pick_level(const MeshLod* m, const View* v, float distance, float tolerance_px) {
    for (int i = LEVELS - 1; i >= 0; i--)
        if (screen_error(v, m->error[i], distance) <= tolerance_px) return i;
    return 0;
}

static int select_level(const MeshLod* m, const View* v, float distance,
                        float tolerance_px, State* st) {
    int want = pick_level(m, v, distance, tolerance_px);
    int prev = st->level;
    if (want != prev) {
        /* hysteresis: require the candidate to beat the threshold by tau */
        float e_new = screen_error(v, m->error[want], distance);
        float e_old = screen_error(v, m->error[prev], distance);
        if (want > prev && e_new > tolerance_px * m->tau) return prev;
        if (want < prev && e_old < tolerance_px / m->tau) return prev;
        st->switches++;
        st->level = want;
    }
    return st->level;
}

/* 校正公式 vs 逐像素实测：各级别、距离、宽高比下偏差须在 10% 内 */
static int measured_matches(const MeshLod* m) {
    int dims[][2] = { {1024, 1024}, {3440, 1440}, {1440, 3440}, {7680, 1080} };
    for (int a = 0; a < 4; a++) {
        View v = { 1.0f, dims[a][0], dims[a][1] };
        for (int i = 1; i < LEVELS; i++)
            for (float d = 3.0f * m->error[i]; d < 600.0f; d *= 1.7f) {
                float formula = screen_error(&v, m->error[i], d);
                float measured = measured_error(&v, m->error[i], d);
                if (fabsf(formula - measured) / measured > 0.10f) return 0;
            }
    }
    return 1;
}

/* 距离拉远时选级必须单调不降 */
static int monotonic(const MeshLod* m, const View* v, float tolerance_px) {
    int prev = 0;
    for (float d = 2.0f; d <= 2000.0f; d *= 1.05f) {
        int lv = pick_level(m, v, d, tolerance_px);
        if (lv < prev) return 0;
        prev = lv;
    }
    return 1;
}

/* 极端宽高比（含超出钳制范围的）：误差有限、透视合法、选级仍单调 */
static int extreme_aspect_ok(const MeshLod* m, float tolerance_px) {
    int dims[][2] = { {160, 2560}, {2560, 160}, {32, 2048}, {2048, 32} };
    for (int a = 0; a < 4; a++) {
        View v = { 1.0f, dims[a][0], dims[a][1] };
        float fx = fov_x_of(&v);
        if (!(fx > 0.0f && fx < 3.1416f)) return 0;
        for (float d = 2.0f; d <= 2000.0f; d *= 1.5f)
            if (!isfinite(screen_error(&v, m->error[LEVELS - 1], d))) return 0;
        if (!monotonic(m, &v, tolerance_px)) return 0;
    }
    return 1;
}

int main(void) {
    MeshLod m = { {0.0f, 1.0f, 4.0f, 16.0f}, 2.0f, 1.1f };
    State st = {0, 0, 0};
    View v = { 1.0f, 1920, 1080 };
    int hist[LEVELS] = {0};
    for (int i = 0; i < 40; i++) {
        float d = 5.0f + 5.0f * i;             /* receding */
        int lv = select_level(&m, &v, d, 50.0f, &st);
        hist[lv]++;
    }
    printf("levels=%d,%d,%d,%d switches=%d\n", hist[0], hist[1], hist[2], hist[3], st.switches);
    /* 迟滞应当让切换次数远小于采样次数 */
    printf("hysteresis_ok=%d\n", st.switches <= 8);

    /* 宽屏：横向锁定 fov=1.0 的 21:9 相机。旧代码把该 fov 当纵轴、只看
       纵向，误差低估 ~2.4 倍；校正后与逐像素实测一致。 */
    View wide = view_from_fov_x(1.0f, 3440, 1440);
    float legacy = axis_error(4.0f, 100.0f, 1.0f, wide.h);   /* 旧：纵轴单算 */
    float fixed  = screen_error(&wide, 4.0f, 100.0f);        /* 新：双轴取大 */
    printf("wide@21:9 legacy=%.1f corrected=%.1f measured=%.1f\n",
           legacy, fixed, measured_error(&wide, 4.0f, 100.0f));

    printf("measured_within_10pct=%d\n", measured_matches(&m));
    printf("monotonic=%d\n", monotonic(&m, &v, 50.0f) && monotonic(&m, &wide, 50.0f));
    printf("extreme_aspect_ok=%d\n", extreme_aspect_ok(&m, 50.0f));
    return 0;
}
