
/* ssaa-render: 软件子像素超采样抗锯齿（无硬件 AA 依赖）。
   每个像素取 n 个可配置子样本，按落在形状内的样本权重之和得到覆盖率，
   再以覆盖率为权重混合前景/背景色。解析参考用半平面对像素正方形做
   Sutherland-Hodgman 裁剪求精确覆盖面积，用于界定采样误差。 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <math.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { float x, y; } Vec2;
typedef struct { float nx, ny, d; } Plane;    /* 内侧: nx*x + ny*y + d >= 0 */
typedef struct { float u, v, w; } Sample;     /* 单位正方形内的子样本及其权重 */

#define MAX_PLANES 8
#define MAX_VERTS  16

typedef struct {
    Plane e[MAX_PLANES];
    int ne;
} ConvexPoly;

/* 由 CCW 顶点构造凸多边形（每条有向边取左法向指向内部） */
static void poly_from_verts(ConvexPoly* p, const Vec2* v, int nv) {
    p->ne = nv;
    for (int i = 0; i < nv; i++) {
        Vec2 a = v[i], b = v[(i + 1) % nv];
        float nx = -(b.y - a.y), ny = b.x - a.x;
        float len = sqrtf(nx * nx + ny * ny);
        nx /= len; ny /= len;
        p->e[i].nx = nx; p->e[i].ny = ny;
        p->e[i].d = -(nx * a.x + ny * a.y);
    }
}

static void make_rot_square(ConvexPoly* p, float cx, float cy, float half_diag, float deg) {
    Vec2 v[4];
    float a0 = deg * (float)M_PI / 180.0f;
    for (int i = 0; i < 4; i++) {
        float a = a0 + (45.0f + 90.0f * i) * (float)M_PI / 180.0f;
        v[i].x = cx + half_diag * cosf(a);
        v[i].y = cy + half_diag * sinf(a);
    }
    poly_from_verts(p, v, 4);
}

static int inside_plane(const Plane* e, float x, float y) {
    return e->nx * x + e->ny * y + e->d >= 0.0f;
}

static int point_inside(const ConvexPoly* p, float x, float y) {
    for (int i = 0; i < p->ne; i++)
        if (!inside_plane(&p->e[i], x, y)) return 0;
    return 1;
}

/* ---------- 解析覆盖率：像素正方形被所有半平面裁剪后的交面积 ---------- */

static Vec2 seg_hit_plane(Vec2 s, Vec2 e, const Plane* pl) {
    float fs = pl->nx * s.x + pl->ny * s.y + pl->d;
    float fe = pl->nx * e.x + pl->ny * e.y + pl->d;
    float t = fs / (fs - fe);
    Vec2 hit = { s.x + t * (e.x - s.x), s.y + t * (e.y - s.y) };
    return hit;
}

static int clip_poly(const Vec2* in, int n, const Plane* pl, Vec2* out) {
    int m = 0;
    if (n == 0) return 0;
    Vec2 S = in[n - 1];
    int Sin = inside_plane(pl, S.x, S.y);
    for (int i = 0; i < n; i++) {
        Vec2 E = in[i];
        int Ein = inside_plane(pl, E.x, E.y);
        if (Ein) {
            if (!Sin) out[m++] = seg_hit_plane(S, E, pl);
            out[m++] = E;
        } else if (Sin) {
            out[m++] = seg_hit_plane(S, E, pl);
        }
        S = E; Sin = Ein;
    }
    return m;
}

static float poly_area(const Vec2* v, int n) {
    float a = 0.0f;
    for (int i = 0; i < n; i++) {
        Vec2 p = v[i], q = v[(i + 1) % n];
        a += p.x * q.y - q.x * p.y;
    }
    return fabsf(a) * 0.5f;
}

/* 像素 (px,py) 覆盖 [px,px+1] x [py,py+1]，面积为 1，交面积即覆盖率 */
static float analytic_coverage(const ConvexPoly* p, int px, int py) {
    Vec2 buf[2][MAX_VERTS];
    int cur = 0, n = 4;
    buf[cur][0] = (Vec2){ px,     py     };
    buf[cur][1] = (Vec2){ px + 1, py     };
    buf[cur][2] = (Vec2){ px + 1, py + 1 };
    buf[cur][3] = (Vec2){ px,     py + 1 };
    for (int i = 0; i < p->ne && n >= 3; i++) {
        n = clip_poly(buf[cur], n, &p->e[i], buf[cur ^ 1]);
        cur ^= 1;
    }
    return n >= 3 ? poly_area(buf[cur], n) : 0.0f;
}

/* ---------- 子样本模式（样本数 n 可配） ---------- */

#define PATTERN_GRID   0   /* 规则 k*k 网格，要求 n 为平方数 */
#define PATTERN_HALTON 1   /* Halton(2,3) 低差异序列，任意 n */

static float halton(int idx, int base) {
    float f = 1.0f, r = 0.0f;
    while (idx > 0) {
        f /= (float)base;
        r += f * (float)(idx % base);
        idx /= base;
    }
    return r;
}

static unsigned hash2(int x, int y, int k) {
    unsigned h = (unsigned)x * 374761393u + (unsigned)y * 668265263u
               + (unsigned)k * 1442695041u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h;
}

static void wrap01(float* t) { if (*t >= 1.0f) *t -= 1.0f; }

static void pattern_build(Sample* s, int n, int mode) {
    if (mode == PATTERN_GRID) {
        int k = (int)(sqrtf((float)n) + 0.5f);
        for (int j = 0; j < k; j++)
            for (int i = 0; i < k; i++) {
                Sample* q = &s[j * k + i];
                q->u = (i + 0.5f) / k;
                q->v = (j + 0.5f) / k;
                q->w = 1.0f / n;
            }
    } else {
        for (int i = 0; i < n; i++) {
            s[i].u = halton(i + 1, 2);
            s[i].v = halton(i + 1, 3);
            s[i].w = 1.0f / n;
        }
    }
}

/* 按覆盖权重合成：命中样本的权重之和即覆盖率，再混合前景/背景 */
static float coverage_at(const ConvexPoly* p, int px, int py,
                         const Sample* base, int n, int mode, long* evals) {
    float ou = (hash2(px, py, 0) >> 8) / (float)(1u << 24);
    float ov = (hash2(px, py, 1) >> 8) / (float)(1u << 24);
    float cov = 0.0f;
    for (int i = 0; i < n; i++) {
        float u = base[i].u, v = base[i].v;
        if (mode == PATTERN_HALTON) { u += ou; v += ov; wrap01(&u); wrap01(&v); }
        if (evals) (*evals)++;
        if (point_inside(p, px + u, py + v)) cov += base[i].w;
    }
    return cov;
}

static long render(const ConvexPoly* p, int W, int H,
                   const Sample* s, int n, int mode, float* img) {
    long evals = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            float cov = coverage_at(p, x, y, s, n, mode, &evals);
            img[y * W + x] = cov;   /* 黑白场景：颜色即覆盖率；通用式 fg*cov+bg*(1-cov) */
        }
    return evals;
}

/* ---------- 自测 ---------- */

static int approx(float a, float b, float eps) { return fabsf(a - b) <= eps; }

int main(void) {
    int fails = 0;
    ConvexPoly sq;
    make_rot_square(&sq, 12.0f, 12.0f, 7.0f, 30.0f);

    /* 1) 解析参考本身的已知值（顶点按 CCW 给出，内部在每条边的左侧） */
    ConvexPoly half;
    Vec2 hv[4] = { {1,0},{4,0},{4,4},{1,4} };   /* CCW 方块，左法向给出内部 1<=x<=4 */
    poly_from_verts(&half, hv, 4);
    float c_full_in  = analytic_coverage(&half, 2, 2);   /* 整块在内 */
    float c_full_out = analytic_coverage(&half, 0, 2);   /* 整块在外（x<=1） */
    ConvexPoly diag;
    Vec2 dv[3] = { {0,0},{1,0},{0,1} };                  /* CCW 三角形，斜边 x+y=1，内部 x+y<=1 */
    poly_from_verts(&diag, dv, 3);
    float c_half = analytic_coverage(&diag, 0, 0);       /* 角部三角形，面积 0.5 */
    float c_zero = analytic_coverage(&diag, 2, 2);       /* x+y>=4，在外 */
    printf("analytic: full_in=%.4f full_out=%.4f corner_tri=%.4f outside=%.4f\n",
           c_full_in, c_full_out, c_half, c_zero);
    int analytic_ok = approx(c_full_in, 1.0f, 1e-5f)
                   && approx(c_full_out, 0.0f, 1e-5f)
                   && approx(c_half, 0.5f, 1e-5f)
                   && approx(c_zero, 0.0f, 1e-5f);
    printf("analytic_ok=%d\n", analytic_ok);
    if (!analytic_ok) fails++;

    /* 2) 边缘像素：采样覆盖率 vs 解析覆盖率，随 n 收敛 */
    const int ns[] = {1, 4, 16, 64};
    float rms[4], emax[4];
    int edge_px = 0;
    for (int k = 0; k < 4; k++) {
        Sample s[64];
        pattern_build(s, ns[k], PATTERN_HALTON);
        double acc = 0.0; float mx = 0.0f; int cnt = 0;
        for (int y = 2; y < 22; y++)
            for (int x = 2; x < 22; x++) {
                float ref = analytic_coverage(&sq, x, y);
                if (ref <= 0.001f || ref >= 0.999f) continue;
                float got = coverage_at(&sq, x, y, s, ns[k], PATTERN_HALTON, 0);
                float d = got - ref;
                acc += (double)d * d;
                if (fabsf(d) > mx) mx = fabsf(d);
                cnt++;
            }
        edge_px = cnt;
        rms[k] = (float)sqrt(acc / cnt);
        emax[k] = mx;
        printf("coverage n=%-3d rms=%.4f max_err=%.4f\n", ns[k], rms[k], emax[k]);
    }
    /* 任意样本数（非平方数）也必须可配 */
    Sample s12[12];
    pattern_build(s12, 12, PATTERN_HALTON);
    double acc12 = 0.0; int cnt12 = 0;
    for (int y = 2; y < 22; y++)
        for (int x = 2; x < 22; x++) {
            float ref = analytic_coverage(&sq, x, y);
            if (ref <= 0.001f || ref >= 0.999f) continue;
            float got = coverage_at(&sq, x, y, s12, 12, PATTERN_HALTON, 0);
            float d = got - ref; acc12 += (double)d * d; cnt12++;
        }
    float rms12 = (float)sqrt(acc12 / cnt12);
    printf("coverage n=12  rms=%.4f (arbitrary count)\n", rms12);

    int grid_check = 1;
    Sample sg[16];
    pattern_build(sg, 9, PATTERN_GRID);   /* 平方数走网格模式 */
    if (!approx(sg[0].u, 1.0f / 6.0f, 1e-6f) || !approx(sg[4].v, 0.5f, 1e-6f)) grid_check = 0;

    int conv_ok = rms[0] > rms[1] && rms[1] > rms[2] && rms[2] > rms[3]
               && emax[3] < 0.15f && rms[3] < 0.05f && rms12 < 0.12f && grid_check;
    printf("convergence_ok=%d (edge_pixels=%d)\n", conv_ok, edge_px);
    if (!conv_ok) fails++;

    /* 3) 边缘扫描线梯度对比（多行，给原始数据与汇总） */
    int x0 = 0, x1 = 24, span = x1 - x0;
    Sample sa[1], s4[4], s16[16];
    pattern_build(sa, 1, PATTERN_GRID);
    pattern_build(s4, 4, PATTERN_GRID);
    pattern_build(s16, 16, PATTERN_GRID);
    double rms_ali = 0, rms_4 = 0, rms_16 = 0;
    double gsum_ali = 0, gsum_4 = 0, gsum_16 = 0, gsum_ref = 0;
    float gmax_ali = 0, gmax_4 = 0, gmax_16 = 0, gmax_ref = 0;
    int cells = 0;
    printf("\nedge pixel table, row=12 (coverage values):\n");
    printf(" px  analytic  alias(1)  ssaa(4)  ssaa(16)\n");
    for (int x = x0; x <= x1; x++) {
        float rf = analytic_coverage(&sq, x, 12);
        float af = coverage_at(&sq, x, 12, sa, 1, PATTERN_GRID, 0);
        float f4 = coverage_at(&sq, x, 12, s4, 4, PATTERN_GRID, 0);
        float f16 = coverage_at(&sq, x, 12, s16, 16, PATTERN_GRID, 0);
        if (rf > 0.001f && rf < 0.999f)
            printf("%3d  %.4f    %.4f    %.4f   %.4f\n", x, rf, af, f4, f16);
    }
    printf("\nedge scanlines (per-row max |gradient|, RMS measured on partial-coverage pixels):\n");
    printf(" row  gmax_alias gmax_ssaa4 gmax_ssaa16 gmax_analytic\n");
    for (int row = 9; row <= 15; row++) {
        float ref_row[32], ali_row[32], a4_row[32], a16_row[32];
        for (int x = x0; x <= x1; x++) {
            int i = x - x0;
            ref_row[i] = analytic_coverage(&sq, x, row);
            ali_row[i] = coverage_at(&sq, x, row, sa, 1, PATTERN_GRID, 0);
            a4_row[i]  = coverage_at(&sq, x, row, s4, 4, PATTERN_GRID, 0);
            a16_row[i] = coverage_at(&sq, x, row, s16, 16, PATTERN_GRID, 0);
        }
        float g[4] = {0,0,0,0};
        float* rows[4] = { ali_row, a4_row, a16_row, ref_row };
        for (int k = 0; k < 4; k++)
            for (int i = 0; i < span; i++) {
                float d = fabsf(rows[k][i + 1] - rows[k][i]);
                if (d > g[k]) g[k] = d;
            }
        printf("%4d  %.4f    %.4f    %.4f     %.4f\n",
               row, g[0], g[1], g[2], g[3]);
        gsum_ali += g[0]; gsum_4 += g[1]; gsum_16 += g[2]; gsum_ref += g[3];
        if (g[0] > gmax_ali) gmax_ali = g[0];
        if (g[1] > gmax_4)   gmax_4   = g[1];
        if (g[2] > gmax_16)  gmax_16  = g[2];
        if (g[3] > gmax_ref) gmax_ref = g[3];
        for (int i = 0; i <= span; i++) {
            if (ref_row[i] <= 0.001f || ref_row[i] >= 0.999f) continue;
            float ea = ali_row[i] - ref_row[i];
            float e4 = a4_row[i] - ref_row[i];
            float e16 = a16_row[i] - ref_row[i];
            rms_ali += ea * ea; rms_4 += e4 * e4; rms_16 += e16 * e16;
            cells++;
        }
    }
    rms_ali = sqrt(rms_ali / cells); rms_4 = sqrt(rms_4 / cells); rms_16 = sqrt(rms_16 / cells);
    printf("summary: gmax  alias=%.4f ssaa4=%.4f ssaa16=%.4f analytic=%.4f\n",
           gmax_ali, gmax_4, gmax_16, gmax_ref);
    printf("summary: gmean alias=%.4f ssaa4=%.4f ssaa16=%.4f analytic=%.4f (7 rows)\n",
           gsum_ali / 7, gsum_4 / 7, gsum_16 / 7, gsum_ref / 7);
    printf("summary: edge-pixel RMS vs analytic: alias=%.4f ssaa4=%.4f ssaa16=%.4f\n",
           rms_ali, rms_4, rms_16);
    int grad_ok = gmax_ali >= 0.999f && gmax_16 < gmax_ali
               && gsum_16 < gsum_ali && gsum_4 < gsum_ali
               && rms_16 < rms_4 && rms_4 < rms_ali
               && rms_16 < 0.05f;
    printf("gradient_ok=%d\n", grad_ok);
    if (!grad_ok) fails++;

    /* 4) 样本数翻倍，耗时线性增长 */
    enum { IMG = 512, REPS = 5 };
    static float img[IMG * IMG];
    const int tn[] = {4, 8, 16, 32, 64};
    double ms[5]; long ev[5];
    for (int k = 0; k < 5; k++) {
        Sample s[64];
        pattern_build(s, tn[k], PATTERN_HALTON);
        render(&sq, IMG, IMG, s, tn[k], PATTERN_HALTON, img); /* warmup */
        double t[REPS];
        for (int rep = 0; rep < REPS; rep++) {
            struct timespec t0, t1;
            clock_gettime(CLOCK_MONOTONIC, &t0);
            ev[k] = render(&sq, IMG, IMG, s, tn[k], PATTERN_HALTON, img);
            clock_gettime(CLOCK_MONOTONIC, &t1);
            t[rep] = (t1.tv_sec - t0.tv_sec) * 1e3 + (t1.tv_nsec - t0.tv_nsec) / 1e6;
        }
        for (int a = 0; a < REPS - 1; a++)                 /* 中位数，抗调度抖动 */
            for (int b = a + 1; b < REPS; b++)
                if (t[b] < t[a]) { double tmp = t[a]; t[a] = t[b]; t[b] = tmp; }
        ms[k] = t[REPS / 2];
        printf("timing n=%-3d %.2f ms evals=%ld\n", tn[k], ms[k], ev[k]);
    }
    for (int k = 1; k < 5; k++) {
        float ratio = (float)(ms[k] / ms[k - 1]);
        printf("time_ratio n %d->%d = %.3f (per-sample t/n: %.4f -> %.4f ms)\n",
               tn[k - 1], tn[k], ratio, ms[k - 1] / tn[k - 1], ms[k] / tn[k]);
    }
    /* 线性度判据：t(n) 对 n 线性回归的 R^2；工作量恒等由 evals==W*H*n 校验 */
    double sx = 0, sy = 0, sxy = 0, sxx = 0, syy = 0;
    for (int k = 0; k < 5; k++) {
        double x = tn[k], y = ms[k];
        sx += x; sy += y; sxy += x * y; sxx += x * x; syy += y * y;
    }
    double meanx = sx / 5, meany = sy / 5;
    double num = sxy - 5 * meanx * meany;
    double den = sqrt((sxx - 5 * meanx * meanx) * (syy - 5 * meany * meany));
    double r2 = (num / den) * (num / den);
    double slope = num / (sxx - 5 * meanx * meanx);
    double intercept = meany - slope * meanx;
    double pmin = 1e9, pmax = 0.0;
    for (int k = 2; k < 5; k++) {
        double p = ms[k] / tn[k];
        if (p < pmin) pmin = p;
        if (p > pmax) pmax = p;
    }
    printf("linear fit: t(n) = %.4f*n + %.4f ms, R^2 = %.5f\n", slope, intercept, r2);
    printf("per-sample cost n>=16: min=%.4f max=%.4f ms spread=%.3f (info only;\n",
           pmin, pmax, pmax / pmin);
    printf("  wall-clock 在共享主机上有调度噪声，工作量线性由 evals==W*H*n 恒等保证)\n");
    int time_ok = r2 >= 0.98;
    for (int k = 0; k < 5; k++)
        if (ev[k] != (long)IMG * IMG * tn[k]) time_ok = 0;
    printf("linearity_ok=%d\n", time_ok);
    if (!time_ok) fails++;

    printf("\nALL_TESTS_%s\n", fails ? "FAILED" : "PASSED");
    return fails ? 1 : 0;
}
