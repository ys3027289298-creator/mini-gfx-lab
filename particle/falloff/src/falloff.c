/* falloff — 粒子贴图 alpha 径向衰减的指数标定实验
 *
 * 剖面: a(u) = (1 - u)^k,  u = r/R ∈ [0,1]
 *
 * 关系 1 (指数 ↔ 单粒子边缘宽度): alpha 从 0.9 降到 0.1 的径向带宽
 *   w(k) = R·(0.9^(1/k) − 0.1^(1/k)) ≈ ln(9)·R/k ≈ 2.197·R/k   (k ≳ 5)
 *
 * 关系 2 (指数 ↔ 簇叠加中心亮度): N 个粒子均匀散布在半径 S 的簇内, 加法混合
 *   E[B] = (2NR²/S²)·∫₀^q (1−t)^k·t dt,  q = min(1, S/R)
 *        = 2NR² / ((k+1)(k+2)·S²)                            (R ≤ S)
 *   → 要钉住 B 必须 k ∝ R; 若 k 恒定(只按单粒子相对柔和度优化)则 B ∝ R²,
 *     大直径簇中心必过曝。
 *
 * 定指数规则: k*(D) = 满足 E[B] ≤ B_TARGET 的最小 k (最软但不过曝),
 *             并要求 w(k*) ≥ W_MIN_PX, 否则该直径下两个目标不可兼得。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/stat.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define B_MAX     1.0   /* 加法混合 LDR 过曝上限 */
#define B_TARGET  0.80  /* 定指数用的目标中心亮度 (留 20% 余量) */
#define W_MIN_PX  1.5   /* 单粒子边缘带宽下限(px), 低于此视为硬边 */
#define CLUSTER_N 256   /* 一簇的粒子数 */
#define CLUSTER_S 32.0  /* 簇散布半径(px), 由发射器物理决定, 与粒子直径无关 */
#define TRIALS    101   /* 每档重复喷簇次数, 取中心亮度均值 */
#define ZOOM      8     /* 单粒子截图放大倍数 */

static unsigned rng_state = 12345u;
static double frand(void) { /* xorshift32 */
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5;
    return (rng_state >> 8) * (1.0 / 16777216.0);
}

static double profile(double u, double k) { return u >= 1.0 ? 0.0 : pow(1.0 - u, k); }

/* 关系 1: 指数 -> 边缘宽度(px) */
static double edge_width_px(double R, double k) {
    return R * (pow(0.9, 1.0 / k) - pow(0.1, 1.0 / k));
}

/* 关系 2: 指数 -> 簇中心亮度期望 */
static double cluster_center_mean(double R, double k) {
    double q = CLUSTER_S < R ? CLUSTER_S / R : 1.0;
    double b = 1.0 - q;
    double I = 1.0 / ((k + 1.0) * (k + 2.0))
             - pow(b, k + 1.0) / (k + 1.0)
             + pow(b, k + 2.0) / (k + 2.0);
    return CLUSTER_N * (2.0 * R * R / (CLUSTER_S * CLUSTER_S)) * I;
}

/* 满足 E[B] <= target 的最小 k (最软但不过曝) */
static double solve_k_brightness(double R, double target) {
    double lo = 0.05, hi = 4096.0;
    for (int it = 0; it < 200; it++) {
        double mid = 0.5 * (lo + hi);
        if (cluster_center_mean(R, mid) > target) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

/* 达到目标边缘宽度的 k (对照组用) */
static double solve_k_edge(double R, double w_target) {
    double lo = 0.05, hi = 4096.0;
    for (int it = 0; it < 200; it++) {
        double mid = 0.5 * (lo + hi);
        if (edge_width_px(R, mid) > w_target) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

static void write_pgm(const char* path, const double* img, int W) {
    FILE* f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P5\n%d %d\n255\n", W, W);
    for (int i = 0; i < W * W; i++) {
        double v = img[i] > 1.0 ? 1.0 : img[i]; /* >1 饱和, 直观看过曝区 */
        unsigned char b = (unsigned char)(v * 255.0 + 0.5);
        fputc(b, f);
    }
    fclose(f);
}

/* 喷一个粒子, 放大 ZOOM 倍渲染, 沿 +x 扫描 alpha 0.9->0.1 带宽, 返回屏幕 px */
static double measure_edge(double R, double k, const char* pgm) {
    int half = (int)ceil(R * ZOOM) + 2, W = 2 * half + 1, c = half;
    double* img = calloc((size_t)W * W, sizeof(double));
    for (int y = 0; y < W; y++)
        for (int x = 0; x < W; x++) {
            double dx = (x - c) / (double)ZOOM, dy = (y - c) / (double)ZOOM;
            img[y * W + x] = profile(hypot(dx, dy) / R, k);
        }
    double r09 = -1.0, r01 = -1.0;
    for (int x = c; x < W - 1; x++) {
        double a0 = img[c * W + x], a1 = img[c * W + x + 1];
        if (r09 < 0.0 && a0 >= 0.9 && a1 < 0.9) r09 = (x - c + (a0 - 0.9) / (a0 - a1)) / ZOOM;
        if (r01 < 0.0 && a0 >= 0.1 && a1 < 0.1) r01 = (x - c + (a0 - 0.1) / (a0 - a1)) / ZOOM;
    }
    if (pgm) write_pgm(pgm, img, W);
    free(img);
    return r01 - r09;
}

/* 喷一簇: N 个粒子均匀散布在半径 CLUSTER_S 的盘内, 加法混合溅射进帧缓冲,
 * 重复 TRIALS 次, 返回簇中心像素亮度的均值 */
static double measure_cluster(double R, double k, const char* pgm, double* bmin, double* bmax,
                              int* over) {
    int half = (int)ceil(CLUSTER_S + R) + 4, W = 2 * half + 1, c = half;
    double* img = malloc((size_t)W * W * sizeof(double));
    double sum = 0.0; *bmin = 1e9; *bmax = -1e9; *over = 0;
    for (int t = 0; t < TRIALS; t++) {
        memset(img, 0, (size_t)W * W * sizeof(double));
        for (int i = 0; i < CLUSTER_N; i++) {
            double ang = 2.0 * M_PI * frand(), rr = CLUSTER_S * sqrt(frand());
            double px = c + rr * cos(ang), py = c + rr * sin(ang);
            int x0 = (int)floor(px - R) - 1, x1 = (int)ceil(px + R) + 1;
            int y0 = (int)floor(py - R) - 1, y1 = (int)ceil(py + R) + 1;
            for (int y = y0; y <= y1; y++)
                for (int x = x0; x <= x1; x++) {
                    if (x < 0 || y < 0 || x >= W || y >= W) continue;
                    img[y * W + x] += profile(hypot(x - px, y - py) / R, k);
                }
        }
        double B = img[c * W + c];
        sum += B;
        if (B < *bmin) *bmin = B;
        if (B > *bmax) *bmax = B;
        if (B > B_MAX) (*over)++;
        if (pgm && t == TRIALS - 1) write_pgm(pgm, img, W);
    }
    free(img);
    return sum / TRIALS;
}

int main(void) {
    mkdir("out", 0755);
    static const double D[3] = {16, 48, 128};

    printf("profile: a(u) = (1-u)^k, u = r/R\n");
    printf("rel1 edge:   w(k) = R*(0.9^(1/k) - 0.1^(1/k)) ~ 2.197*R/k\n");
    printf("rel2 cluster: E[B] = (2*N*R^2/S^2)*I(k,q), N=%d S=%.0f; hold B => k~R; const k => B~R^2\n\n",
           CLUSTER_N, CLUSTER_S);

    /* 按两条关系定指数 */
    double k[3];
    printf("[k* from both relations] B_target=%.2f B_max=%.2f w_min=%.1fpx\n",
           B_TARGET, B_MAX, W_MIN_PX);
    for (int i = 0; i < 3; i++) {
        double R = D[i] / 2;
        k[i] = solve_k_brightness(R, B_TARGET);
        double w = edge_width_px(R, k[i]);
        printf("  D=%3.0f R=%5.1f  k*=%7.2f  w_model=%.2fpx  B_model=%.3f  %s\n",
               D[i], R, k[i], w, cluster_center_mean(R, k[i]),
               w >= W_MIN_PX ? "feasible" : "INFEASIBLE: softness/brightness conflict");
    }

    /* 三组六个数: 单粒子放大量边缘宽度, 喷一簇量中心亮度 */
    printf("\n[measurements] single zoomed x%d; cluster %d trials avg\n", ZOOM, TRIALS);
    for (int i = 0; i < 3; i++) {
        double R = D[i] / 2, lo, hi;
        int over;
        char path[64];
        snprintf(path, sizeof path, "out/single_d%03d.pgm", (int)D[i]);
        double w = measure_edge(R, k[i], path);
        snprintf(path, sizeof path, "out/cluster_d%03d.pgm", (int)D[i]);
        double B = measure_cluster(R, k[i], path, &lo, &hi, &over);
        printf("  D=%3.0f: edge=%.2f px  B_center=%.3f (range %.3f..%.3f, over %d/%d) %s\n",
               D[i], w, B, lo, hi, over, TRIALS, B > B_MAX ? "OVER B_max" : "ok");
    }

    /* 对照: 只按单粒子优化 — 相对柔和度恒定 w=0.10R, 即 k 恒定 */
    printf("\n[control: single-particle-only rule, w = 0.10*R]\n");
    double kc[3], Bc[3], lo, hi;
    int over;
    for (int i = 0; i < 3; i++) {
        kc[i] = solve_k_edge(D[i] / 2, 0.10 * D[i] / 2);
        Bc[i] = measure_cluster(D[i] / 2, kc[i], NULL, &lo, &hi, &over);
    }
    printf("  D      k       edge_px   B_center\n");
    for (int i = 0; i < 3; i++)
        printf("  %3.0f  %7.2f  %7.2f  %8.3f %s\n", D[i], kc[i],
               edge_width_px(D[i] / 2, kc[i]), Bc[i],
               Bc[i] > B_MAX ? "  > B_max OVER" : "");

    if (Bc[2] > B_MAX)
        printf("\nverdict: 大直径簇中心亮度超上限 —— 只按单粒子优化的指数(k 恒定)没算叠加;\n"
               "         必须按关系 2 让 k 随屏幕直径增长 (k ~ D)。\n");
    return 0;
}
