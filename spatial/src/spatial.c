
/* mini-spatial-audio: binaural positioning of mono sources + 5.1 downmix.
   双耳部分：ITD by azimuth (max 0.6 ms), IID by azimuth, 反距离衰减,
   soft-clip mixing of N sources into an interleaved stereo buffer.
   5.1 部分：按方位角的余弦解析权重分配 L/R/C/SL/SR（能量归一化），
   80 Hz 一阶互补分频把低频单独合并进 LFE，六声道交织输出。 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define SR 48000
#define MAX_FRAMES 4096
#define MAX_SRC 8

/* 5.1 声道顺序（SMPTE/ITU 常用交织顺序） */
enum { CH_L = 0, CH_R, CH_C, CH_LFE, CH_SL, CH_SR, NCH_51 };
#define NCH_MAIN 5                 /* 五个全频主声道，LFE 不参与方位声像 */

#define LFE_CUTOFF 80.0f           /* Hz，影院低频管理常用分频点 */
#define PI_F 3.14159265358979323846f

typedef struct {
    float x, y, z;      /* world position, meters; listener at origin facing -z */
    float gain;
} Source;

typedef struct {
    float yaw;          /* radians, 0 = facing -z */
} Listener;

/* azimuth in radians, 0 = front, +pi/2 = right */
static float azimuth_of(const Source* s, const Listener* l) {
    float dx = s->x, dz = s->z;
    float c = cosf(l->yaw), sn = sinf(l->yaw);
    float xr =  c * dx - sn * dz;
    float zr =  sn * dx + c * dz;
    return atan2f(xr, -zr);
}

static int itd_samples(float az) {
    float max_delay = 0.0006f * SR;          /* 0.6 ms, Woodworth-lite */
    return (int)lroundf(sinf(az) * max_delay);
}

static void iid_gains(float az, float* gl, float* gr) {
    float t = (sinf(az) + 1.0f) * 0.5f;      /* 0 = hard left, 1 = hard right */
    *gl = sqrtf(1.0f - t);
    *gr = sqrtf(t);
}

static float distance_gain(const Source* s) {
    float d = sqrtf(s->x * s->x + s->y * s->y + s->z * s->z);
    return 1.0f / (1.0f + d);
}

static float soft_clip(float v) {
    const float k = 0.8f;
    float a = fabsf(v);
    if (a <= k) return v;
    float sign = v < 0 ? -1.0f : 1.0f;
    return sign * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

static void render(const Source* srcs, int n, const Listener* l,
                   const float* mono, int frames, float* out /* interleaved */) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < n; k++) {
        float az = azimuth_of(&srcs[k], l);
        int delay = itd_samples(az);
        float gl, gr; iid_gains(az, &gl, &gr);
        float g = srcs[k].gain * distance_gain(&srcs[k]);
        for (int i = 0; i < frames; i++) {
            int j = i - delay;
            float s = (j >= 0 && j < frames) ? mono[j] : 0.0f;
            out[2*i + 0] += s * g * gl;
            out[2*i + 1] += s * g * gr;
        }
    }
    for (int i = 0; i < frames * 2; i++) out[i] = soft_clip(out[i]);
}

static float peak(const float* buf, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(buf[i]) > m) m = fabsf(buf[i]);
    return m;
}

/* ------------------------------------------------------------------ */
/* 5.1 下混                                                            */
/* ------------------------------------------------------------------ */

/* 五个全频扬声器方位角（ITU-R BS.775）：C=0, L/R=±30°, SL/SR=±110°。
   数组下标是主声道编号，SPK_CH 映射到六声道缓冲的声道下标。 */
static const float SPK_AZ[NCH_MAIN] = {
    -30.0f * PI_F / 180.0f,   /* [0] L  */
     30.0f * PI_F / 180.0f,   /* [1] R  */
      0.0f,                   /* [2] C  */
   -110.0f * PI_F / 180.0f,   /* [3] SL */
    110.0f * PI_F / 180.0f    /* [4] SR */
};
static const int SPK_CH[NCH_MAIN] = { CH_L, CH_R, CH_C, CH_SL, CH_SR };

static float wrap_pi(float a) {
    while (a >  PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

/* 解析声像权重（也是测试对照的解析公式）：
   原始权重 w_i = max(0, cos(az - alpha_i))，半空间之外为 0；
   再做能量归一化 g_i = w_i / sqrt(sum w_j^2)，故恒有 sum(g_i^2) = 1。
   中心扬声器 alpha=0，w_C 即前后权重里的前方权重 max(0, cos az)：
   侧方 (|az|>=90°) 与后方声源的中心声道权重严格为 0。 */
static void pan_weights(float az, float g[NCH_MAIN]) {
    float raw[NCH_MAIN];
    float norm = 0.0f;
    for (int i = 0; i < NCH_MAIN; i++) {
        float d = fabsf(wrap_pi(az - SPK_AZ[i]));
        raw[i] = (d < PI_F / 2.0f) ? cosf(d) : 0.0f;
        norm += raw[i] * raw[i];
    }
    norm = sqrtf(norm);
    for (int i = 0; i < NCH_MAIN; i++) g[i] = raw[i] / norm;
}

typedef struct {
    float lpf_y[MAX_SRC];   /* 每声源一阶低通状态，跨帧保持 */
} Downmix51;

static void downmix51_init(Downmix51* c) {
    memset(c, 0, sizeof(*c));
}

/* 单帧处理：mono 为 N 个声源共享的单声道输入（与双耳 render 一致），
   out 为 frames*NCH_51 的交织缓冲。
   互补分频：low = LPF(x), high = x - low，low+high 恒等于 x；
   low 乘距离增益后合并进 LFE，high 乘解析权重进五个主声道。 */
static void downmix51_process(Downmix51* c, const Source* srcs, int n,
                              const Listener* l, const float* mono,
                              int frames, float* out) {
    const float a = 2.0f * PI_F * LFE_CUTOFF / (float)SR;  /* 一阶 LPF 系数 */
    memset(out, 0, sizeof(float) * (size_t)frames * NCH_51);

    for (int k = 0; k < n; k++) {
        float g[NCH_MAIN];
        pan_weights(azimuth_of(&srcs[k], l), g);
        float dg = srcs[k].gain * distance_gain(&srcs[k]);
        float yl = c->lpf_y[k];

        for (int i = 0; i < frames; i++) {
            float x = mono[i];
            yl += a * (x - yl);              /* 一阶低通 */
            float low = yl;
            float high = x - low;            /* 互补高通，low+high=x */
            float* o = out + (size_t)i * NCH_51;
            for (int j = 0; j < NCH_MAIN; j++)
                o[SPK_CH[j]] += high * dg * g[j];
            o[CH_LFE] += low * dg;           /* 低频单独合并，不带方位 */
        }
        c->lpf_y[k] = yl;
    }
    for (size_t i = 0; i < (size_t)frames * NCH_51; i++)
        out[i] = soft_clip(out[i]);
}

/* ---------------- 5.1 测试工具 ---------------- */

static double channel_energy(const float* buf, int frames, int ch) {
    double e = 0.0;
    for (int i = 0; i < frames; i++) {
        double v = buf[(size_t)i * NCH_51 + ch];
        e += v * v;
    }
    return e;
}

/* 半径 r、方位 az 度的声源位置（az=0 前方 -z，+90 右方 +x） */
static Source source_at(float r, float az_deg, float gain) {
    float az = az_deg * PI_F / 180.0f;
    Source s = { r * sinf(az), 0.0f, -r * cosf(az), gain };
    return s;
}

static int check(int cond, const char* what) {
    printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what);
    return cond ? 0 : 1;
}

int main(void) {
    int fails = 0;
    float mono[MAX_FRAMES];
    float out[MAX_FRAMES * 2];
    for (int i = 0; i < 256; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;  /* impulse at 64 */

    Listener l = { 0.0f };
    Source right = { 3.0f, 0.0f, 0.0f, 1.0f };
    render(&right, 1, &l, mono, 256, out);
    int best_l = 0, best_r = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*best_l+0])) best_l = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*best_r+1])) best_r = i;
    }
    printf("right_source peakL=%d peakR=%d (impulse@64)\n", best_l, best_r);

    Source left = { -3.0f, 0.0f, 0.0f, 1.0f };
    render(&left, 1, &l, mono, 256, out);
    int bl = 0, br = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*best_l+0])) bl = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*best_r+1])) br = i;
    }
    printf("left_source peakL=%d peakR=%d (impulse@64)\n", bl, br);

    Source near = { 0.0f, 0.0f, -1.0f, 1.0f };
    Source far  = { 0.0f, 0.0f, -9.0f, 1.0f };
    render(&near, 1, &l, mono, 256, out); float pn = peak(out, 256*2);
    render(&far, 1, &l, mono, 256, out);  float pf = peak(out, 256*2);
    printf("attenuation near=%.4f far=%.4f ratio=%.3f\n", pn, pf, pf / pn);

    Source many[4] = { {1,0,0,1}, {-1,0,0,1}, {0,0,-1,1}, {0,0,1,1} };
    float loud[MAX_FRAMES];
    for (int i = 0; i < 256; i++) loud[i] = 0.9f;
    render(many, 4, &l, loud, 256, out);
    printf("mix_peak=%.4f (soft clip 上限 1.0)\n", peak(out, 256*2));

    /* ============ 5.1 下混测试 ============ */
    float out6[MAX_FRAMES * NCH_51];
    Downmix51 dm;

    /* --- 1) 六个朝向：实测主声道能量分布 vs 硬编码解析权重（5% 内） --- */
    printf("\n[5.1] 六朝向能量分布（份额 %%，括号为解析值）\n");
    static const float AZ6[6] = { 0.0f, 60.0f, 120.0f, 180.0f, -120.0f, -60.0f };
    /* 解析份额 = g_i^2，按 pan_weights 公式离线算出的独立基准：
       120° -> SR=1; -120° -> SL=1; 180° -> SL/SR 各 .5;
       0° -> L/R 各 .3、C .4（±30° 扬声器余弦 .866，C 仍为最大单声道）;
       ±60° -> 同侧主/环绕/中心 0.5307/0.2924/0.1769。 */
    static const float EXP_SHARE[6][NCH_MAIN] = {
        /* L,R,C,SL,SR（主声道下标顺序） */
        { 0.3f,    0.3f,    0.4f,    0.0f,    0.0f    }, /*   0° */
        { 0.0f,    0.5307f, 0.1769f, 0.0f,    0.2924f }, /*  60° */
        { 0.0f,    0.0f,    0.0f,    0.0f,    1.0f    }, /* 120° */
        { 0.0f,    0.0f,    0.0f,    0.5f,    0.5f    }, /* 180° */
        { 0.0f,    0.0f,    0.0f,    1.0f,    0.0f    }, /*-120° */
        { 0.5307f, 0.0f,    0.1769f, 0.2924f, 0.0f    }, /* -60° */
    };
    static const char* MNAME[NCH_MAIN] = { "L", "R", "C", "SL", "SR" };
    const int FR = 512;
    for (int i = 0; i < FR; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;
    float worst_rel = 0.0f, worst_abs = 0.0f;

    for (int q = 0; q < 6; q++) {
        downmix51_init(&dm);
        Source s = source_at(2.0f, AZ6[q], 1.0f);
        downmix51_process(&dm, &s, 1, &l, mono, FR, out6);

        double e[NCH_MAIN], esum = 0.0;
        for (int j = 0; j < NCH_MAIN; j++) {
            e[j] = channel_energy(out6, FR, SPK_CH[j]);
            esum += e[j];
        }
        printf("  az=%6.0f°:", AZ6[q]);
        for (int j = 0; j < NCH_MAIN; j++) {
            float share = (float)(e[j] / esum);
            float expv = EXP_SHARE[q][j];
            float err;
            if (expv > 1e-4f) {
                err = fabsf(share - expv) / expv;
                if (err > worst_rel) worst_rel = err;
            } else {
                err = fabsf(share - expv);
                if (err > worst_abs) worst_abs = err;
            }
            printf(" %s=%.4f(%.4f)", MNAME[j], share, expv);
        }
        printf("\n");
    }
    printf("  最大相对误差=%.3f%% 零权重声道最大绝对份额=%.5f\n",
           worst_rel * 100.0f, worst_abs);
    fails += check(worst_rel < 0.05f && worst_abs < 0.02f,
                   "六朝向能量分布与解析权重差 < 5%");

    /* --- 2) 中心声道按前后权重生成：w_C = max(0, cos az)，侧/后为 0、前向单调 --- */
    static const float SWEEP_AZ[6] = { 0.0f, 30.0f, 60.0f, 90.0f, 135.0f, 180.0f };
    double c_share_prev = 2.0;            /* 份额随方位后移应单调不增 */
    int mono_ok = 1, back_zero_ok = 1;
    printf("\n[5.1] 中心声道前后权重：");
    for (int q = 0; q < 6; q++) {
        downmix51_init(&dm);
        Source s = source_at(2.0f, SWEEP_AZ[q], 1.0f);
        downmix51_process(&dm, &s, 1, &l, mono, FR, out6);
        double ec = channel_energy(out6, FR, CH_C);
        double em = 0.0;
        for (int j = 0; j < NCH_MAIN; j++) em += channel_energy(out6, FR, SPK_CH[j]);
        double share = ec / em;

        float g[NCH_MAIN];
        pan_weights(SWEEP_AZ[q] * PI_F / 180.0f, g);
        float exp_share = g[2] * g[2];     /* 解析中心权重（主声道下标 2 = C） */
        if (fabs((float)share - exp_share) > 0.01f) mono_ok = 0;
        if (SWEEP_AZ[q] >= 90.0f && ec != 0.0) back_zero_ok = 0;
        if (share > c_share_prev + 1e-12) mono_ok = 0;
        c_share_prev = share;
        printf(" %.0f°=%.4f", SWEEP_AZ[q], share);
    }
    printf("\n");
    fails += check(mono_ok, "C 份额跟随解析前后权重 max(0,cos az) 且前向单调");
    fails += check(back_zero_ok, "侧方(90°)与后方声源 C 严格为 0");

    /* --- 3) LFE 低频单独合并：40 Hz 进 LFE，2 kHz 留主声道 --- */
    struct { float freq; int frames; float min_lfe, max_lfe; } lfe_cases[2] = {
        { 40.0f,  SR / 40,  0.70f, 1.01f },  /* 一个完整周期，LFE 占绝大多数 */
        { 2000.0f, SR / 200, 0.0f,  0.02f }, /* 十个周期，LFE 几乎为 0 */
    };
    for (int q = 0; q < 2; q++) {
        int nfr = lfe_cases[q].frames;
        for (int i = 0; i < nfr; i++)
            mono[i] = sinf(2.0f * PI_F * lfe_cases[q].freq * i / SR);
        downmix51_init(&dm);
        Source s = source_at(2.0f, 30.0f, 1.0f);
        downmix51_process(&dm, &s, 1, &l, mono, nfr, out6);
        double elfe = channel_energy(out6, nfr, CH_LFE);
        double emain = 0.0;
        for (int j = 0; j < NCH_MAIN; j++) emain += channel_energy(out6, nfr, SPK_CH[j]);
        float lfe_share = (float)(elfe / (elfe + emain));
        int ok = lfe_share >= lfe_cases[q].min_lfe && lfe_share <= lfe_cases[q].max_lfe;
        printf("\n[5.1] %.0f Hz 信号 LFE 能量占比=%.4f\n", lfe_cases[q].freq, lfe_share);
        fails += check(ok, q == 0 ? "低频(40Hz)主要进入 LFE" : "高频(2kHz)不进 LFE");
    }

    /* --- 4) 总能量守恒：六声道总能量 == 距离增益^2 * 输入能量（5% 内） --- */
    for (int i = 0; i < FR; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;
    float worst_ratio = 1.0f;
    for (int q = 0; q < 6; q++) {
        downmix51_init(&dm);
        Source s = source_at(2.0f, AZ6[q], 1.0f);
        downmix51_process(&dm, &s, 1, &l, mono, FR, out6);
        double etot = 0.0;
        for (int ch = 0; ch < NCH_51; ch++) etot += channel_energy(out6, FR, ch);
        float dg = distance_gain(&s);
        float ratio = (float)etot / (dg * dg);   /* 输入冲激能量=1 */
        if (fabsf(ratio - 1.0f) > fabsf(worst_ratio - 1.0f)) worst_ratio = ratio;
    }
    printf("\n[5.1] 能量守恒最差比率=%.4f（1.0 为完全守恒）\n", worst_ratio);
    fails += check(worst_ratio > 0.95f && worst_ratio < 1.05f, "总能量守恒偏差 < 5%");

    /* --- 5) 单帧耗时 < 3 ms：8 声源、512 帧，取 200 次平均 --- */
    Source src8[MAX_SRC];
    for (int k = 0; k < MAX_SRC; k++)
        src8[k] = source_at(3.0f, -150.0f + (float)k * 45.0f, 1.0f);
    for (int i = 0; i < FR; i++) mono[i] = sinf(2.0f * PI_F * 220.0f * i / SR);
    downmix51_init(&dm);

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (int rep = 0; rep < 200; rep++)
        downmix51_process(&dm, src8, MAX_SRC, &l, mono, FR, out6);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double us = ((t1.tv_sec - t0.tv_sec) * 1e6 + (t1.tv_nsec - t0.tv_nsec) / 1e3) / 200.0;
    printf("\n[5.1] 单帧耗时 %.1f us/frame（%d 声源 × %d 帧，预算 3000 us）\n",
           us, MAX_SRC, FR);
    fails += check(us < 3000.0, "单帧处理 < 3 ms");

    if (fails) printf("\n%d 项检查失败\n", fails);
    else printf("\n全部 5.1 检查通过\n");
    return fails ? 1 : 0;
}
