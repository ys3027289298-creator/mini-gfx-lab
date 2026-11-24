
/* normal-correct: 低分辨率光带会让切线空间法线图的均值整体偏移，光照随之
   偏移。这里用邻近置信度（邻域法线与全局稳健均值的一致程度）加权求局部
   均值，把低置信度纹素拉向该均值。校正半径与贴图尺度成约束：
   r_texels * texel_size == WORLD_RADIUS，低分辨率纹素的半径（以纹素计）
   按比例缩小。最后圈出偏差最大的区域，并在半径极限 R_MAX 处重跑一遍，
   确认校正不会引入更大的角度偏差。 */
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define N 64
#define P_CONF 16           /* 置信度锐度 */
#define CONF_MIN 0.5f       /* 低于此置信度才校正 */
#define WORLD_RADIUS 8.0f   /* 半径约束常数（世界单位） */
#define R_MIN 1
#define R_MAX 8             /* 校正半径极限 */

typedef struct { float x, y, z; } Vec3;

static Vec3 vadd(Vec3 a, Vec3 b) { return (Vec3){a.x + b.x, a.y + b.y, a.z + b.z}; }
static Vec3 vscale(Vec3 a, float s) { return (Vec3){a.x * s, a.y * s, a.z * s}; }
static float vdot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static Vec3 vnorm(Vec3 a) {
    float l = sqrtf(vdot(a, a));
    return l > 1e-9f ? vscale(a, 1.0f / l) : (Vec3){0.0f, 0.0f, 1.0f};
}
static float angle_deg(Vec3 a, Vec3 b) {
    float d = vdot(vnorm(a), vnorm(b));
    if (d > 1.0f) d = 1.0f;
    if (d < -1.0f) d = -1.0f;
    return acosf(d) * 180.0f / (float)M_PI;
}

typedef struct {
    Vec3 n[N][N];        /* 切线空间法线 */
    float texel[N][N];   /* 每纹素的世界尺寸；>1 表示低分辨率区 */
} NormalMap;

/* 真值：平滑起伏的法线场，单位纹素 */
static void gen_truth(NormalMap* m) {
    const float pi = (float)M_PI;
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) {
            float u = (float)x / N, v = (float)y / N;
            float nx = 0.15f * sinf(u * 2.0f * pi * 3.0f) * cosf(v * 2.0f * pi * 2.0f);
            float ny = 0.15f * cosf(u * 2.0f * pi * 2.0f) * sinf(v * 2.0f * pi * 3.0f);
            m->n[y][x] = vnorm((Vec3){nx, ny, 1.0f});
            m->texel[y][x] = 1.0f;
        }
}

/* 低分辨率光带：2x 纹素、法线整体倾斜，导致光照均值偏移 */
static void add_light_band(NormalMap* m, int y0, int y1, float tilt) {
    for (int y = y0; y < y1; y++)
        for (int x = 0; x < N; x++) {
            m->n[y][x] = vnorm((Vec3){tilt, 0.0f, 1.0f});
            m->texel[y][x] = 2.0f;
        }
}

/* 约束关系：半径（纹素计）随贴图尺度反比变化，r * texel_size 恒定 */
static int radius_for(float texel_size) {
    int r = (int)(WORLD_RADIUS / texel_size + 0.5f);
    if (r < R_MIN) r = R_MIN;
    if (r > R_MAX) r = R_MAX;
    return r;
}

static Vec3 global_mean(const NormalMap* m) {
    Vec3 s = {0.0f, 0.0f, 0.0f};
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) s = vadd(s, m->n[y][x]);
    return vnorm(s);   /* 光带是少数区域，全局和仍接近真值方向 */
}

static void confidence_map(const NormalMap* m, float conf[N][N]) {
    Vec3 g = global_mean(m);
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) {
            float d = vdot(m->n[y][x], g);
            if (d < 0.0f) d = 0.0f;
            conf[y][x] = powf(d, P_CONF);
        }
}

/* 单遍校正；radius_override > 0 时强制所有纹素用该半径（极限复跑用） */
static void correct(const NormalMap* src, NormalMap* dst,
                    const float conf[N][N], int radius_override) {
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) {
            dst->texel[y][x] = src->texel[y][x];
            if (conf[y][x] >= CONF_MIN) {          /* 可信纹素不动 */
                dst->n[y][x] = src->n[y][x];
                continue;
            }
            int r = radius_override > 0 ? radius_override
                                        : radius_for(src->texel[y][x]);
            Vec3 s = {0.0f, 0.0f, 0.0f};
            float wsum = 0.0f;
            for (int dy = -r; dy <= r; dy++)
                for (int dx = -r; dx <= r; dx++) {
                    if (dx * dx + dy * dy > r * r || (dx == 0 && dy == 0)) continue;
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= N || ny >= N) continue;
                    float w = conf[ny][nx];
                    s = vadd(s, vscale(src->n[ny][nx], w));
                    wsum += w;
                }
            Vec3 mean = wsum > 1e-6f ? vscale(s, 1.0f / wsum)
                                     : (Vec3){0.0f, 0.0f, 1.0f};
            float lambda = 1.0f - conf[y][x];      /* 越不可信越跟随邻域均值 */
            dst->n[y][x] = vnorm(vadd(vscale(src->n[y][x], 1.0f - lambda),
                                      vscale(mean, lambda)));
        }
}

static void err_stats(const NormalMap* a, const NormalMap* truth,
                      float* mean_deg, float* max_deg) {
    double sum = 0.0;
    float mx = 0.0f;
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) {
            float d = angle_deg(a->n[y][x], truth->n[y][x]);
            sum += d;
            if (d > mx) mx = d;
        }
    *mean_deg = (float)(sum / (N * N));
    *max_deg = mx;
}

/* 从偏差最大的纹素出发，洪水填充偏差超过峰值一半的区域，输出包围盒 */
static void worst_region(const NormalMap* biased, const NormalMap* fixed,
                         int* x0, int* y0, int* x1, int* y1, int* count) {
    float dev[N][N], mx = 0.0f;
    int sx = 0, sy = 0;
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) {
            dev[y][x] = angle_deg(biased->n[y][x], fixed->n[y][x]);
            if (dev[y][x] > mx) { mx = dev[y][x]; sx = x; sy = y; }
        }
    float thr = 0.5f * mx;
    unsigned char seen[N][N] = {{0}};
    int stack[N * N][2], sp = 0;
    seen[sy][sx] = 1;
    stack[sp][0] = sx; stack[sp][1] = sy; sp++;
    *x0 = *x1 = sx; *y0 = *y1 = sy; *count = 0;
    while (sp > 0) {
        sp--;
        int x = stack[sp][0], y = stack[sp][1];
        (*count)++;
        if (x < *x0) *x0 = x;
        if (x > *x1) *x1 = x;
        if (y < *y0) *y0 = y;
        if (y > *y1) *y1 = y;
        const int d[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
        for (int k = 0; k < 4; k++) {
            int nx = x + d[k][0], ny = y + d[k][1];
            if (nx < 0 || ny < 0 || nx >= N || ny >= N || seen[ny][nx]) continue;
            if (dev[ny][nx] < thr) continue;
            seen[ny][nx] = 1;
            stack[sp][0] = nx; stack[sp][1] = ny; sp++;
        }
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    enum { Y0 = 30, Y1 = 34 };   /* 光带行范围 [Y0, Y1) */
    static NormalMap truth, biased, fixed, extreme;
    static float conf[N][N];

    gen_truth(&truth);
    biased = truth;
    add_light_band(&biased, Y0, Y1, 0.8f);

    confidence_map(&biased, conf);
    correct(&biased, &fixed, conf, 0);

    float mean_b, max_b, mean_a, max_a;
    err_stats(&biased, &truth, &mean_b, &max_b);
    err_stats(&fixed, &truth, &mean_a, &max_a);
    printf("mean_err_before_deg=%.3f mean_err_after_deg=%.3f\n", mean_b, mean_a);
    printf("max_err_before_deg=%.3f max_err_after_deg=%.3f\n", max_b, max_a);
    check("mean_err_reduced", mean_a < mean_b);
    check("max_err_not_worse", max_a <= max_b + 1e-3f);

    /* 半径与贴图尺度的约束关系：r * texel_size 恒定，且被极限夹住 */
    int r1 = radius_for(1.0f), r2 = radius_for(2.0f);
    printf("radius_texel1=%d radius_texel2=%d\n", r1, r2);
    check("radius_scale_constraint",
          r1 * 1.0f == WORLD_RADIUS && r2 * 2.0f == WORLD_RADIUS);
    check("radius_clamped", radius_for(0.5f) == R_MAX && radius_for(64.0f) == R_MIN);

    /* 偏差最大的区域单独圈出 */
    int x0, y0, x1, y1, count;
    worst_region(&biased, &fixed, &x0, &y0, &x1, &y1, &count);
    printf("worst_region=[%d,%d]-[%d,%d] texels=%d\n", x0, y0, x1, y1, count);
    check("worst_region_is_band",
          x0 == 0 && x1 == N - 1 && y0 == Y0 && y1 == Y1 - 1 && count == (Y1 - Y0) * N);

    /* 在校正半径极限处取极端值重跑，确认不会引入更大的角度偏差 */
    correct(&biased, &extreme, conf, R_MAX);
    float mean_e, max_e;
    err_stats(&extreme, &truth, &mean_e, &max_e);
    printf("extreme_radius=%d extreme_mean_err_deg=%.3f extreme_max_err_deg=%.3f\n",
           R_MAX, mean_e, max_e);
    check("extreme_radius_not_worse",
          max_e <= max_b + 1e-3f && mean_e <= mean_b + 1e-3f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
