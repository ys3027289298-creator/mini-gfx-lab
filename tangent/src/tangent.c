
/* tangent-basis: 法线贴图切线基的退化防护与旋转自测。

   症状：角色绕垂直轴转身时贴图闪一下白，转过某个角度后恢复。
   原因：切线由 T = normalize(R - N*(N·R)) 构造，N 扫过主参考向量 R 附近时
   投影长度趋零，归一化结果由浮点舍入决定，帧间乱跳，着色法线瞬间偏转。

   修复：|N·R1| 超过门限时切换到备用参考 R2（要求 R1⊥R2）。
   门限由夹角推导，非经验值：
     R1⊥R2 且 |N|=1  ⇒  |N·R1|² + |N·R2|² ≤ 1        （Bessel 不等式）
     ⇒  min(|N·R1|, |N·R2|) ≤ √2/2，最坏情形在 45° 取等。
   故取门限 cos45° = √2/2：可保证所选参考与 N 的夹角 ≥ 45°，
   |N×R| ≥ sin45° = √2/2，有确定下界；且对双正交参考而言该门限是
   极小极大最优——任何更低门限都可能选到比 √2/2 更差的参考。 */
#include <stdio.h>
#include <math.h>

#define PI         3.14159265358979323846
#define SWITCH_COS 0.70710678118654752   /* cos(45°) = 1/√2，推导见上 */
#define DEV_GATE   1e-3                  /* rad，浮点舍入容差（测试判定用，非切换门限） */
#define FRAMES     24                    /* 绕垂直轴一整圈，每 15° 一帧 */
#define STEP_DEG   15

#define N_EQ  12                 /* 赤道环法线（四肢/轮廓） */
#define N_MID 8                  /* 60° 极角环（躯干） */
#define N_ALL (N_EQ + N_MID + 2) /* 另加头顶/足底 */

static float NM[3]; /* 法线贴图采样（切线空间），main 中归一化 */

static void rot_y_f(const float in[3], float yaw, float out[3]) {
    float c = cosf(yaw), s = sinf(yaw);
    out[0] = in[0] * c + in[2] * s;
    out[1] = in[1];
    out[2] = -in[0] * s + in[2] * c;
}

static void rot_y_d(const double in[3], double yaw, double out[3]) {
    double c = cos(yaw), s = sin(yaw);
    out[0] = in[0] * c + in[2] * s;
    out[1] = in[1];
    out[2] = -in[0] * s + in[2] * c;
}

/* 角色表面法线集合：float 路径模拟渲染管线的浮点旋转 */
static void gen_frame_f(float yaw, float out[][3]) {
    int i = 0;
    for (int j = 0; j < N_EQ; j++, i++) {
        float phi = (float)(j * 30) * (float)PI / 180.0f;
        float n[3] = { cosf(phi), 0.0f, sinf(phi) };
        rot_y_f(n, yaw, out[i]);
    }
    for (int j = 0; j < N_MID; j++, i++) {
        float phi = (float)(j * 45) * (float)PI / 180.0f;
        float th = 60.0f * (float)PI / 180.0f;
        float n[3] = { sinf(th) * cosf(phi), cosf(th), sinf(th) * sinf(phi) };
        rot_y_f(n, yaw, out[i]);
    }
    { float top[3] = {0, 1, 0};  rot_y_f(top, yaw, out[i++]); }
    { float bot[3] = {0, -1, 0}; rot_y_f(bot, yaw, out[i++]); }
}

/* 同一批法线的双精度路径，作为偏离度基准 */
static void gen_frame_d(double yaw, double out[][3]) {
    int i = 0;
    for (int j = 0; j < N_EQ; j++, i++) {
        double phi = (double)(j * 30) * PI / 180.0;
        double n[3] = { cos(phi), 0.0, sin(phi) };
        rot_y_d(n, yaw, out[i]);
    }
    for (int j = 0; j < N_MID; j++, i++) {
        double phi = (double)(j * 45) * PI / 180.0;
        double th = 60.0 * PI / 180.0;
        double n[3] = { sin(th) * cos(phi), cos(th), sin(th) * sin(phi) };
        rot_y_d(n, yaw, out[i]);
    }
    { double top[3] = {0, 1, 0};  rot_y_d(top, yaw, out[i++]); }
    { double bot[3] = {0, -1, 0}; rot_y_d(bot, yaw, out[i++]); }
}

/* T = normalize(R - N*(N·R)), B = N × T；use_fallback=0 时退化为原实现。
   返回所选参考的下标（0=R1, 1=R2），供双精度基准复用同一参考。 */
static int basis_f(const float n[3], const float r1[3], const float r2[3],
                   int use_fallback, float t[3], float b[3]) {
    const float* r = r1;
    int ref = 0;
    float d1 = fabsf(n[0] * r1[0] + n[1] * r1[1] + n[2] * r1[2]);
    if (use_fallback && d1 > (float)SWITCH_COS) { r = r2; ref = 1; }
    float d = n[0] * r[0] + n[1] * r[1] + n[2] * r[2];
    float v[3] = { r[0] - n[0] * d, r[1] - n[1] * d, r[2] - n[2] * d };
    float l = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    t[0] = v[0] / l; t[1] = v[1] / l; t[2] = v[2] / l;
    b[0] = n[1] * t[2] - n[2] * t[1];
    b[1] = n[2] * t[0] - n[0] * t[2];
    b[2] = n[0] * t[1] - n[1] * t[0];
    return ref;
}

/* 双精度基准：强制使用与 float 路径相同的参考（ref），
   否则切换边界两侧各选一个合法参考，度量到的就是参考切换本身而非退化。 */
static void basis_d(const double n[3], const double r1[3], const double r2[3],
                    int ref, double t[3], double b[3]) {
    const double* r = ref ? r2 : r1;
    double d = n[0] * r[0] + n[1] * r[1] + n[2] * r[2];
    double v[3] = { r[0] - n[0] * d, r[1] - n[1] * d, r[2] - n[2] * d };
    double l = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    t[0] = v[0] / l; t[1] = v[1] / l; t[2] = v[2] / l;
    b[0] = n[1] * t[2] - n[2] * t[1];
    b[1] = n[2] * t[0] - n[0] * t[2];
    b[2] = n[0] * t[1] - n[1] * t[0];
}

/* 着色法线 = normalize(nm.x*T + nm.y*B + nm.z*N) */
static void shade_f(const float t[3], const float b[3], const float n[3], float out[3]) {
    float x = NM[0] * t[0] + NM[1] * b[0] + NM[2] * n[0];
    float y = NM[0] * t[1] + NM[1] * b[1] + NM[2] * n[1];
    float z = NM[0] * t[2] + NM[1] * b[2] + NM[2] * n[2];
    float l = sqrtf(x * x + y * y + z * z);
    out[0] = x / l; out[1] = y / l; out[2] = z / l;
}

static void shade_d(const double t[3], const double b[3], const double n[3], double out[3]) {
    double x = (double)NM[0] * t[0] + (double)NM[1] * b[0] + (double)NM[2] * n[0];
    double y = (double)NM[0] * t[1] + (double)NM[1] * b[1] + (double)NM[2] * n[1];
    double z = (double)NM[0] * t[2] + (double)NM[1] * b[2] + (double)NM[2] * n[2];
    double l = sqrt(x * x + y * y + z * z);
    out[0] = x / l; out[1] = y / l; out[2] = z / l;
}

/* 两条着色法线的夹角（rad）；出现 NaN/零向量按最大偏离处理 */
static double deviation(const float a[3], const double b[3]) {
    double na = sqrt((double)a[0] * a[0] + (double)a[1] * a[1] + (double)a[2] * a[2]);
    double nb = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    if (na == 0.0 || nb == 0.0 || isnan(na) || isnan(nb)) return PI;
    double d = ((double)a[0] * b[0] + (double)a[1] * b[1] + (double)a[2] * b[2]) / (na * nb);
    if (isnan(d)) return PI;
    if (d > 1.0) d = 1.0;
    if (d < -1.0) d = -1.0;
    return acos(d);
}

/* 绕垂直轴转一整圈，每 15° 一帧，逐帧记录法线偏离度；返回超门限帧数 */
static int run_pass(const char* name, const float r1[3], const float r2[3],
                    int use_fallback) {
    double r1d[3] = { r1[0], r1[1], r1[2] };
    double r2d[3] = { r2[0], r2[1], r2[2] };
    int exceed = 0;
    printf("[%s]\n", name);
    for (int k = 0; k < FRAMES; k++) {
        float yawf = (float)(k * STEP_DEG) * (float)PI / 180.0f;
        double yawd = (double)(k * STEP_DEG) * PI / 180.0;
        float nf[N_ALL][3];
        double nd[N_ALL][3];
        gen_frame_f(yawf, nf);
        gen_frame_d(yawd, nd);
        double maxdev = 0.0;
        for (int i = 0; i < N_ALL; i++) {
            float t[3], b[3], ns[3];
            double td[3], bd[3], nsd[3];
            int ref = basis_f(nf[i], r1, r2, use_fallback, t, b);
            shade_f(t, b, nf[i], ns);
            basis_d(nd[i], r1d, r2d, ref, td, bd);
            shade_d(td, bd, nd[i], nsd);
            double dev = deviation(ns, nsd);
            if (dev > maxdev) maxdev = dev;
        }
        int bad = maxdev > DEV_GATE;
        exceed += bad;
        printf("frame %2d  yaw=%3d  maxdev=%10.6f deg  %s\n",
               k, k * STEP_DEG, maxdev * 180.0 / PI, bad ? "EXCEED" : "");
    }
    printf("exceed=%d/%d\n", exceed, FRAMES);
    if (exceed > 1) {
        /* 超门限的帧不止一处：备用参考很可能与主参考共线，列出夹角 */
        double d = r1d[0] * r2d[0] + r1d[1] * r2d[1] + r1d[2] * r2d[2];
        if (d > 1.0) d = 1.0;
        if (d < -1.0) d = -1.0;
        printf("exceed>1: fallback reference collinear with primary? "
               "angle(r1,r2)=%.2f deg\n", acos(d) * 180.0 / PI);
    }
    return exceed;
}

int main(void) {
    float nm[3] = { 0.3f, -0.2f, 1.0f };
    float l = sqrtf(nm[0] * nm[0] + nm[1] * nm[1] + nm[2] * nm[2]);
    NM[0] = nm[0] / l; NM[1] = nm[1] / l; NM[2] = nm[2] / l;

    printf("switch threshold: |N·R1| > cos(45°) = %.6f  (derived: min(|N·R1|,|N·R2|) <= 1/sqrt(2))\n",
           SWITCH_COS);
    printf("deviation gate: %.1e rad (float roundoff tolerance)\n", DEV_GATE);

    float r1[3] = { 0, 0, 1 };   /* 主参考：世界前向 */
    float r2[3] = { 1, 0, 0 };   /* 备用参考：与主参考正交 */

    /* 对照组：原实现，无切换 —— 应在法线扫过 R1 的帧出现闪白 */
    int e_buggy = run_pass("buggy: no fallback", r1, r2, 0);
    /* 修复后：|N·R1| > √2/2 切换到 R2 —— 应全程无超门限帧 */
    int e_fixed = run_pass("fixed: fallback at |N·R1| > 1/sqrt(2)", r1, r2, 1);
    /* 自检诊断：故意把备用参考选成与主参考共线，诊断应触发并列出夹角 */
    float r2_collinear[3] = { 0, 0, 1 };
    int e_col = run_pass("self-check: collinear fallback", r1, r2_collinear, 1);

    printf("buggy_flashes=%d\n", e_buggy > 1);
    printf("fixed_ok=%d\n", e_fixed == 0);
    printf("diagnostic_ok=%d\n", e_col > 1);
    return !(e_fixed == 0 && e_col > 1);
}
