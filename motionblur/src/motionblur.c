
/* motionblur: per-pixel velocity-field motion blur.
   每个输出像素沿自身速度场做快门时间线积分，核长 L = |v| * T 随速率变化，
   采样权重归一保持能量（DC 增益 = 1）。含解析积分参考图与误差分解测试。 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MB_MAX_DIM 256
#define MB_MAX_SAMPLES 256

typedef struct {
    int enabled;        /* 0 = 直通，输出与输入逐位相同 */
    float shutter;      /* 快门时间 T，核长 L = |v| * T */
    int samples;        /* 每像素时间采样数 N（中点求积） */
    float max_speed;    /* >0 时速率上限(px/T)，越界拒绝；0 表示不限 */
    float max_kernel;   /* >0 时核长上限(px)，越界拒绝；0 表示不限 */
} MBConfig;

enum {
    MB_OK = 0,
    MB_ERR_DIM = 1,
    MB_ERR_CONFIG = 2,
    MB_ERR_VALUE = 3,    /* 速度含 NaN/inf */
    MB_ERR_SPEED = 4,    /* 速率越界 */
    MB_ERR_KERNEL = 5    /* 核长越界 */
};

/* 预检：尺寸、配置与全场速度/核长范围。不修改任何像素。 */
static int mb_validate(const MBConfig* c, int w, int h,
                       const float* vx, const float* vy) {
    if (w <= 0 || h <= 0 || w > MB_MAX_DIM || h > MB_MAX_DIM) return MB_ERR_DIM;
    if (!c || !vx || !vy) return MB_ERR_CONFIG;
    if (!c->enabled) return MB_OK;
    if (!(c->shutter >= 0.0f) || c->samples < 1 || c->samples > MB_MAX_SAMPLES)
        return MB_ERR_CONFIG;
    if (c->max_speed < 0.0f || c->max_kernel < 0.0f) return MB_ERR_CONFIG;
    int n = w * h;
    for (int i = 0; i < n; i++) {
        if (!isfinite(vx[i]) || !isfinite(vy[i])) return MB_ERR_VALUE;
        float speed = hypotf(vx[i], vy[i]);
        if (c->max_speed > 0.0f && speed > c->max_speed) return MB_ERR_SPEED;
        float L = speed * c->shutter;
        if (c->max_kernel > 0.0f && L > c->max_kernel) return MB_ERR_KERNEL;
    }
    return MB_OK;
}

static inline int clampi(int v, int hi) {
    return v < 0 ? 0 : (v > hi ? hi : v);
}

/* 双线性采样，图外按边缘像素钳制 */
static float sample2d(const float* img, int w, int h, float px, float py) {
    int x0 = (int)floorf(px);
    int y0 = (int)floorf(py);
    float fx = px - x0, fy = py - y0;
    int x1 = clampi(x0 + 1, w - 1), y1 = clampi(y0 + 1, h - 1);
    x0 = clampi(x0, w - 1);
    y0 = clampi(y0, h - 1);
    float a = img[y0 * w + x0] * (1.0f - fx) + img[y0 * w + x1] * fx;
    float b = img[y1 * w + x0] * (1.0f - fx) + img[y1 * w + x1] * fx;
    return a * (1.0f - fy) + b * fy;
}

/* 逐像素速度场模糊。in 与 out 不可别名。失败时不写 out。 */
static int mb_blur(const MBConfig* c, const float* in, float* out,
                   int w, int h, const float* vx, const float* vy) {
    int rc = mb_validate(c, w, h, vx, vy);
    if (rc != MB_OK) return rc;
    int n = w * h;
    if (!in || !out || in == out) return MB_ERR_CONFIG;
    if (!c->enabled) {
        memcpy(out, in, (size_t)n * sizeof(float));   /* 关闭时逐位相同 */
        return MB_OK;
    }
    for (int i = 0; i < n; i++) {
        float speed = hypotf(vx[i], vy[i]);
        float L = speed * c->shutter;
        if (speed == 0.0f || L <= 0.0f) {
            out[i] = in[i];                            /* 静止像素不被糊 */
            continue;
        }
        float acc = 0.0f, wsum = 0.0f;
        for (int k = 0; k < c->samples; k++) {
            /* 中点求积：t 均匀落在 [-T/2, T/2] */
            float t = -0.5f * c->shutter
                    + ((float)k + 0.5f) * c->shutter / (float)c->samples;
            float px = (float)(i % w) - vx[i] * t;
            float py = (float)(i / w) - vy[i] * t;
            float wt = 1.0f;                           /* 均匀快门响应 */
            acc += wt * sample2d(in, w, h, px, py);
            wsum += wt;
        }
        out[i] = acc / wsum;                           /* 能量归一 */
    }
    return MB_OK;
}

/* ---------------- 解析参考：1D 阶跃边 I(q) = [q >= edge] ---------------- */

/* 冻结速度模型：对称核 [x-L/2, x+L/2] 对理想阶跃的解析积分 */
static float ref_frozen_step(float x, float L, float edge) {
    if (L <= 0.0f) return x >= edge ? 1.0f : 0.0f;
    float f = (x + 0.5f * L - edge) / L;
    return f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
}

/* 光栅化阶跃经双线性重建后的分段线性信号 g(q) 的原函数 G(q) */
static float antideriv_recon(float q, float e0) {
    if (q <= e0) return 0.0f;
    if (q <= e0 + 1.0f) {
        float d = q - e0;
        return 0.5f * d * d;
    }
    return 0.5f + (q - (e0 + 1.0f));
}

/* 双线性重建信号在核区间上的解析均值（求积误差的对照基准） */
static float ref_recon_step(float x, float L, float edge) {
    if (L <= 0.0f) {
        float e0 = floorf(edge);
        if (x <= e0) return 0.0f;
        if (x >= e0 + 1.0f) return 1.0f;
        return x - e0;
    }
    float a = x - 0.5f * L, b = x + 0.5f * L;
    return (antideriv_recon(b, floorf(edge)) - antideriv_recon(a, floorf(edge))) / L;
}

/* 变速场 v(x) = v0 + k*x 的真实曝光：沿轨迹 dX/dt = v(X) 积分。
   X(-t;x) = (x+c)e^(-kt) - c, c = v0/k，求穿越边缘的 t* 得解析占比。 */
static float ref_trajectory_step(float x, float v0, float k,
                                 float T, float edge) {
    float c = v0 / k;
    float tstar = logf((x + c) / (edge + c)) / k;
    float f = (tstar + 0.5f * T) / T;
    return f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
}

/* ---------------- 解析参考：2D 正弦场斜向匀速模糊 ---------------- */

static float sincf(float z) { return z == 0.0f ? 1.0f : sinf(z) / z; }

/* I = .5 + .25 sin(wx)sin(wy)，速度 (vx,vy)，对称快门的解析线积分 */
static float ref_sinusoid(float x, float y, float vx, float vy,
                          float T, float omega) {
    float z1 = omega * (vx - vy) * 0.5f * T;
    float z2 = omega * (vx + vy) * 0.5f * T;
    float s = 0.5f * (cosf(omega * (x - y)) * sincf(z1)
                   - cosf(omega * (x + y)) * sincf(z2));
    return 0.5f + 0.25f * s;
}

/* ---------------- 测试工具 ---------------- */

#define W 64
#define H 16

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void metrics(const float* a, const float* b, int n,
                    float* maxe, float* mean) {
    float m = 0.0f, sum = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = fabsf(a[i] - b[i]);
        if (d > m) m = d;
        sum += d;
    }
    *maxe = m;
    *mean = sum / (float)n;
}

static float max_abs(const float* a, const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = fabsf(a[i] - b[i]);
        if (d > m) m = d;
    }
    return m;
}

static void build_step(float* img, float edge) {
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            img[y * W + x] = (float)x >= edge ? 1.0f : 0.0f;
}

static void fill_field(float* vx, float* vy, float fx, float fy) {
    for (int i = 0; i < W * H; i++) { vx[i] = fx; vy[i] = fy; }
}

int main(void) {
    static float in[W * H], out[W * H], vx[W * H], vy[W * H];
    static float ra[W * H], rb[W * H];
    const int N = W * H;
    const float T = 1.0f;

    /* A. 匀速阶跃边：离散核 vs 理想阶跃解析积分，并拆分两类偏差 */
    const float edge_u = 20.3f;
    build_step(in, edge_u);
    fill_field(vx, vy, 4.0f, 0.0f);
    for (int i = 0; i < N; i++) {
        ra[i] = ref_frozen_step((float)(i % W), 4.0f, edge_u);
        rb[i] = ref_recon_step((float)(i % W), 4.0f, edge_u);
    }
    MBConfig cfg8 =  {1, T, 8,  0.0f, 0.0f};
    MBConfig cfg64 = {1, T, 64, 0.0f, 0.0f};
    float tm, ta, q8, q64, recon_m;
    mb_blur(&cfg8, in, out, W, H, vx, vy);
    metrics(out, ra, N, &tm, &ta);
    q8 = max_abs(out, rb, N);
    recon_m = max_abs(rb, ra, N);
    printf("uniform_total_max=%.5f uniform_total_mean=%.5f\n", tm, ta);
    printf("uniform_recon_max=%.5f uniform_quadr8_max=%.5f\n", recon_m, q8);
    mb_blur(&cfg64, in, out, W, H, vx, vy);
    q64 = max_abs(out, rb, N);
    printf("uniform_quadr64_max=%.7f\n", q64);
    check("uniform_total_bounded", tm < 0.10f);
    check("recon_floor_exists", recon_m > 1e-4f);
    /* 重建信号分段线性，中点求积对线性函数精确：求积误差仅余浮点舍入 */
    check("midpoint_exact_on_ramp", q8 < 1e-6f && q64 < 1e-6f);
    check("total_equals_recon_floor", fabsf(tm - recon_m) < 1e-6f);

    /* B. 变速场 v(x)=v0+kx：核长逐像素变化，对照真实轨迹解析解 */
    const float v0 = 2.0f, k = 0.08f, edge_v = 44.3f;
    build_step(in, edge_v);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            vx[y * W + x] = v0 + k * (float)x;
            vy[y * W + x] = 0.0f;
        }
    MBConfig cfgv = {1, T, 32, 16.0f, 16.0f};
    mb_blur(&cfgv, in, out, W, H, vx, vy);
    float total_m = 0.0f, model_m = 0.0f, rec_m = 0.0f, quad_m = 0.0f;
    for (int y = 0; y < H; y++) {
        for (int x = 32; x < W; x++) {           /* 变速段 band */
            int i = y * W + x;
            float L = vx[i] * T;
            float E = ref_trajectory_step((float)x, v0, k, T, edge_v);
            float F = ref_frozen_step((float)x, L, edge_v);
            float G = ref_recon_step((float)x, L, edge_v);
            float d1 = fabsf(out[i] - E);
            float d2 = fabsf(F - E);
            float d3 = fabsf(G - F);
            float d4 = fabsf(out[i] - G);
            if (d1 > total_m) total_m = d1;
            if (d2 > model_m) model_m = d2;
            if (d3 > rec_m) rec_m = d3;
            if (d4 > quad_m) quad_m = d4;
        }
    }
    printf("vary_total_max=%.5f vary_model_max=%.5f\n", total_m, model_m);
    printf("vary_recon_max=%.5f vary_quadr_max=%.5f\n", rec_m, quad_m);
    check("vary_model_bias_exists", model_m > 1e-4f);   /* 匀速假设露馅处 */
    check("vary_total_bounded", total_m < 0.15f);
    check("vary_quadrature_small", quad_m < 0.02f);

    /* C. 2D 正弦场 + 斜向速度：验证 vx/vy 两个方向的核积分。
       正弦非分段线性，中点求积误差随 N 收敛，在这里展示求积阶数。 */
    const float omega = 2.0f * 3.14159265f / 16.0f;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            in[y * W + x] = 0.5f + 0.25f * sinf(omega * x) * sinf(omega * y);
    fill_field(vx, vy, 3.0f, 2.0f);
    MBConfig cfgs = {1, T, 32, 0.0f, 0.0f};
    mb_blur(&cfgs, in, out, W, H, vx, vy);
    static float ref[W * H];
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            ref[y * W + x] = ref_sinusoid((float)x, (float)y, 3.0f, 2.0f, T, omega);
    float sin_m = 0.0f;
    for (int y = 4; y < H - 4; y++)
        for (int x = 4; x < W - 4; x++) {
            float d = fabsf(out[y * W + x] - ref[y * W + x]);
            if (d > sin_m) sin_m = d;
        }
    printf("sinusoid_max=%.5f\n", sin_m);
    check("sinusoid_analytic_match", sin_m < 0.02f);

    MBConfig cfgs1 = {1, T, 1, 0.0f, 0.0f};
    mb_blur(&cfgs1, in, out, W, H, vx, vy);
    float sin1_m = 0.0f;
    for (int y = 4; y < H - 4; y++)
        for (int x = 4; x < W - 4; x++) {
            float d = fabsf(out[y * W + x] - ref[y * W + x]);
            if (d > sin1_m) sin1_m = d;
        }
    printf("sinusoid_n1_max=%.5f\n", sin1_m);
    /* N=1 为单点矩形规则（对曲率有 O(T²) 误差），N=32 应显著更准 */
    check("quadrature_converges", sin_m < 0.5f * sin1_m);

    /* D. 静止像素逐位保留；混速场中静止区不被糊 */
    build_step(in, edge_u);
    for (int i = 0; i < N; i++) { vx[i] = (i % W) < 10 ? 0.0f : 4.0f; vy[i] = 0.0f; }
    MBConfig cfgm = {1, T, 16, 0.0f, 0.0f};
    mb_blur(&cfgm, in, out, W, H, vx, vy);
    int static_exact = 1, blurred_changed = 0;
    for (int i = 0; i < N; i++) {
        if (vx[i] == 0.0f && out[i] != in[i]) static_exact = 0;
        if (vx[i] > 0.0f && out[i] != in[i]) blurred_changed = 1;
    }
    check("zero_velocity_bit_exact", static_exact);
    check("moving_pixels_actually_blur", blurred_changed);

    /* E. 关闭时整图逐位相同，即使速度场越界也不检查、不处理 */
    fill_field(vx, vy, 1e6f, 1e6f);
    MBConfig off = {0, T, 8, 1.0f, 1.0f};
    int rc = mb_blur(&off, in, out, W, H, vx, vy);
    check("disabled_passthrough", rc == MB_OK && memcmp(in, out, sizeof(in)) == 0);

    /* F. 越界拒绝：速率、核长、NaN、坏配置，且失败时 out 保持原样 */
    for (int i = 0; i < N; i++) out[i] = -777.0f;
    fill_field(vx, vy, 11.0f, 0.0f);
    MBConfig lim_speed = {1, T, 8, 10.0f, 0.0f};
    rc = mb_blur(&lim_speed, in, out, W, H, vx, vy);
    check("reject_speed", rc == MB_ERR_SPEED && out[0] == -777.0f);

    fill_field(vx, vy, 4.0f, 0.0f);
    MBConfig lim_kernel = {1, T, 8, 0.0f, 3.0f};   /* L=4 > 3 */
    rc = mb_blur(&lim_kernel, in, out, W, H, vx, vy);
    check("reject_kernel", rc == MB_ERR_KERNEL && out[0] == -777.0f);

    fill_field(vx, vy, 1.0f, 0.0f);
    vx[0] = NAN;
    rc = mb_blur(&cfg8, in, out, W, H, vx, vy);
    check("reject_nan", rc == MB_ERR_VALUE && out[0] == -777.0f);

    MBConfig bad = {1, T, 0, 0.0f, 0.0f};
    fill_field(vx, vy, 1.0f, 0.0f);
    rc = mb_blur(&bad, in, out, W, H, vx, vy);
    check("reject_bad_config", rc == MB_ERR_CONFIG && out[0] == -777.0f);

    /* G. 能量归一：平坦场加长核（含跨边界像素），DC 增益必须为 1 */
    for (int i = 0; i < N; i++) in[i] = 0.3f;
    fill_field(vx, vy, 8.0f, 0.0f);
    MBConfig cfge = {1, T, 32, 0.0f, 0.0f};
    mb_blur(&cfge, in, out, W, H, vx, vy);
    float energy_m = max_abs(in, out, N);
    printf("energy_max_dev=%.7f\n", energy_m);
    check("energy_normalized", energy_m < 1e-6f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
