
/* lod-selector: 投影误差自适应 LOD 选择。
   阈值不写死：容差以视口高度比例给出，随分辨率缩放；
   每级允许误差由视口尺寸与视场角换算到世界空间，按几何误差选级；
   选级为 O(LEVELS) 线性扫描，迟滞带防止边界抖动。 */
#include <stdio.h>
#include <math.h>

#define LEVELS 4

typedef struct {
    float error[LEVELS];   /* 每级几何误差，世界单位 */
    float radius;          /* 包围半径 */
    float tau;             /* 迟滞比，如 1.1 */
} MeshLod;

typedef struct {
    int width, height;     /* 视口像素 */
    float fov_y;           /* 垂直视场角，弧度 */
} LodView;

typedef struct {
    int level;
    int switches;
    int max_evals;         /* 单次选级误差评估次数峰值，用于验证线性时间 */
} State;

/* 由水平视场角与宽高比推垂直视场角；宽屏下 vfov 收窄，投影误差被放大，
   这正是固守像素阈值在宽屏上失效的根源之一 */
static LodView view_from_hfov(int w, int h, float fov_x) {
    LodView v = { w, h, 2.0f * atanf(tanf(fov_x * 0.5f) * (float)h / (float)w) };
    return v;
}

/* 投影误差（像素）：几何误差经透视投影到屏幕上的尺寸 */
static float screen_error(float geom_error, float distance, const LodView* v) {
    if (distance <= 0.0f) return 1e9f;
    return geom_error * (float)v->height / (2.0f * distance * tanf(v->fov_y * 0.5f));
}

/* 每级允许误差（世界单位）：由视口高度、视场角与像素容差换算。
   自适应用法下 tol_px = rel_tol * height，约去 height 后允许误差只随
   距离与视场角变化——分辨率改变时画质（误差占屏比）保持恒定 */
static float allowed_error(float distance, const LodView* v, float tol_px) {
    if (distance <= 0.0f) return 0.0f;
    return tol_px * 2.0f * distance * tanf(v->fov_y * 0.5f) / (float)v->height;
}

/* 选满足误差预算的最粗级：O(LEVELS) 线性扫描 + 迟滞带 */
static int select_level(const MeshLod* m, const LodView* v, float distance,
                        float tol_px, State* st) {
    float allow = allowed_error(distance, v, tol_px);
    int want = 0, evals = 0;
    for (int i = LEVELS - 1; i >= 0; i--) {
        evals++;
        if (m->error[i] <= allow) { want = i; break; }
    }
    if (evals > st->max_evals) st->max_evals = evals;
    int prev = st->level;
    if (want != prev) {
        /* 迟滞：精化需当前级明显超预算（> allow*tau），
           粗化需候选级明显低于预算（< allow/tau），否则维持现状 */
        if (want < prev && m->error[prev] <= allow * m->tau) return prev;
        if (want > prev && m->error[want] >= allow / m->tau) return prev;
        st->switches++;
        st->level = want;
    }
    return st->level;
}

#define SAMPLES 80   /* 远去 40 采样 + 折返 40 采样 */

int main(void) {
    MeshLod m = { {0.0f, 1.0f, 4.0f, 16.0f}, 2.0f, 1.1f };
    const float rel_tol = 0.02f;     /* 自适应：视口高度的 2% */
    const float fixed_px = 50.0f;    /* 固守：写死 50px */
    const float hfov = 1.2f;

    struct { const char* name; int w, h; } views[] = {
        { "720p",   1280,  720 },
        { "1080p",  1920, 1080 },
        { "uw1080", 2560, 1080 },   /* 21:9 */
        { "uw1440", 3440, 1440 },   /* 21:9 宽屏 */
        { "4k",     3840, 2160 },
    };

    printf("%-9s %-7s %9s %9s %9s %8s %7s %8s\n",
           "policy", "view", "tol_px", "err_max%", "err_avg%", "over2%", "avg_lvl", "switches");

    int max_evals_all = 0, max_switches = 0;
    float adaptive_worst = 0.0f;

    for (int vi = 0; vi < 5; vi++) {
        LodView v = view_from_hfov(views[vi].w, views[vi].h, hfov);
        for (int policy = 0; policy < 2; policy++) {
            float tol_px = policy ? rel_tol * (float)v.height : fixed_px;
            State st = { 0, 0, 0 };
            float max_pct = 0.0f, sum_pct = 0.0f;
            int over = 0, lvl_sum = 0;
            for (int s = 0; s < SAMPLES; s++) {
                float d = (s < 40) ? 5.0f + 5.0f * s : 200.0f - 5.0f * (s - 40);
                int lv = select_level(&m, &v, d, tol_px, &st);
                float pct = screen_error(m.error[lv], d, &v) / (float)v.height * 100.0f;
                if (pct > max_pct) max_pct = pct;
                sum_pct += pct;
                if (pct > rel_tol * 100.0f) over++;
                lvl_sum += lv;
            }
            printf("%-9s %-7s %9.1f %8.2f%% %8.2f%% %7.1f%% %6.2f %8d\n",
                   policy ? "adaptive" : "fixed", views[vi].name, tol_px,
                   max_pct, sum_pct / SAMPLES, 100.0f * over / SAMPLES,
                   (float)lvl_sum / SAMPLES, st.switches);
            if (st.max_evals > max_evals_all) max_evals_all = st.max_evals;
            if (st.switches > max_switches) max_switches = st.switches;
            if (policy && max_pct > adaptive_worst) adaptive_worst = max_pct;
        }
    }

    /* 选级每次最多评估 LEVELS 个误差，线性时间 */
    printf("linear_ok=%d (max_evals=%d <= levels=%d)\n",
           max_evals_all <= LEVELS, max_evals_all, LEVELS);
    /* 迟滞：80 采样里切换次数应远小于采样数 */
    printf("hysteresis_ok=%d (max_switches=%d <= %d)\n",
           max_switches <= 2 * (LEVELS - 1), max_switches, 2 * (LEVELS - 1));
    /* 自适应：任意视口下误差占屏比不超过 rel_tol*tau（迟滞带上沿） */
    printf("adaptive_ok=%d (worst_err=%.2f%% <= %.2f%% at every view)\n",
           adaptive_worst <= rel_tol * 100.0f * m.tau + 1e-3f,
           adaptive_worst, rel_tol * 100.0f * m.tau);
    return 0;
}
