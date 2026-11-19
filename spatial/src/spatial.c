
/* mini-spatial-audio: binaural positioning of mono sources.
   ITD by azimuth (max 0.6 ms) as a fractional delay (48-tap windowed
   sinc), IID by azimuth, inverse-distance attenuation, soft-clip mixing
   of N sources into an interleaved stereo buffer.

   分数延迟相位响应测量（make test 输出数据, 任一不达标退出码非零）:
     M1 亚采样延迟误差 — 低频群延迟 vs 指令延迟, 要求 < 1/20 采样
     M2 通带相位偏差 — vs 解析全通 H(e^jw)=e^{-jwD}, 0..21.6 kHz 内 < 3°
     M3 连续变参爆音 — 延迟参数连续扫描时相邻采样跳变不得越界 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SR 48000
#define MAX_FRAMES 4096
#define MAX_SRC 8
#define PI 3.14159265358979323846

/* ---- 分数延迟线: 加窗 sinc 插值 ----
   FIR 无反馈, 系数是 frac 的连续函数, 连续变参不会产生状态瞬变。
   固定群延迟 FDL_CENTER = 23.5 采样, frac ∈ [-0.5, 0.5]。 */
#define FDL_TAPS 48
#define FDL_CENTER ((FDL_TAPS - 1) * 0.5)
#define FDL_PASSBAND_HZ (0.9 * (SR / 2))   /* 通带: 0 .. 21.6 kHz */

static float sincf_(float x) {
    if (fabsf(x) < 1e-8f) return 1.0f;
    float p = (float)PI * x;
    return sinf(p) / p;
}

/* frac ∈ [-0.5, 0.5]; 总延迟 = FDL_CENTER + frac 采样 */
static void fdl_coeffs(float frac, float* h) {
    for (int k = 0; k < FDL_TAPS; k++) {
        float w = 0.42f - 0.5f * cosf(2.0f * (float)PI * k / (FDL_TAPS - 1))
                        + 0.08f * cosf(4.0f * (float)PI * k / (FDL_TAPS - 1));
        h[k] = sincf_((float)k - (float)FDL_CENTER - frac) * w;   /* Blackman */
    }
}

/* y[i] = sum_k h[k] * x[i-k], x[j]=0 for j<0 */
static void fdl_run(const float* h, const float* x, float* y, int n) {
    for (int i = 0; i < n; i++) {
        float acc = 0.0f;
        for (int k = 0; k < FDL_TAPS && k <= i; k++) acc += h[k] * x[i - k];
        y[i] = acc;
    }
}

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

static float itd_samples(float az) {
    float max_delay = 0.0006f * SR;          /* 0.6 ms = 28.8 采样, Woodworth-lite */
    return sinf(az) * max_delay;             /* 分数值, 亚采样部分由 FDL 实现 */
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

#define ITD_BULK 32.0f   /* 固定整体延迟, 保证负 ITD 也可因果; 双耳一致 */

static void render(const Source* srcs, int n, const Listener* l,
                   const float* mono, int frames, float* out /* interleaved */) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    float delayed[MAX_FRAMES];
    for (int k = 0; k < n; k++) {
        float az = azimuth_of(&srcs[k], l);
        float total = itd_samples(az) + ITD_BULK;   /* 总延迟 = total + FDL_CENTER */
        int base = (int)lroundf(total);
        float frac = total - (float)base;           /* [-0.5, 0.5] */
        float h[FDL_TAPS];
        fdl_coeffs(frac, h);
        for (int i = 0; i < frames; i++) {
            float acc = 0.0f;
            for (int m = 0; m < FDL_TAPS; m++) {
                int j = i - base - m;
                if (j >= 0 && j < frames) acc += h[m] * mono[j];
            }
            delayed[i] = acc;
        }
        float gl, gr; iid_gains(az, &gl, &gr);
        float g = srcs[k].gain * distance_gain(&srcs[k]);
        for (int i = 0; i < frames; i++) {
            out[2*i + 0] += delayed[i] * g * gl;
            out[2*i + 1] += delayed[i] * g * gr;
        }
    }
    for (int i = 0; i < frames * 2; i++) out[i] = soft_clip(out[i]);
}

static float peak(const float* buf, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(buf[i]) > m) m = fabsf(buf[i]);
    return m;
}

/* ---- 分数延迟测量 ---- */

static const float FRACS[] = { -0.45f, -0.30f, -0.15f, 0.0f, 0.15f, 0.30f, 0.45f };
#define NFRAC 7

/* 最小二乘拟合 y[i] ≈ C cos(w*(off+i)) + S sin(w*(off+i)),
   相位 = atan2(-S, C), 幅度 = hypot(C, S) */
static void fit_phasor(const float* y, int n, int off, double w,
                       double* phase, double* mag) {
    double cc = 0, ss = 0, cs = 0, yc = 0, ys = 0;
    for (int i = 0; i < n; i++) {
        double c = cos(w * (off + i)), s = sin(w * (off + i));
        cc += c * c; ss += s * s; cs += c * s;
        yc += y[i] * c; ys += y[i] * s;
    }
    double det = cc * ss - cs * cs;
    double C = (yc * ss - ys * cs) / det;
    double S = (ys * cc - yc * cs) / det;
    *phase = atan2(-S, C);
    *mag = hypot(C, S);
}

#define MEAS_N 4352
#define MEAS_SETTLE 256   /* FIR 瞬态仅 FDL_TAPS-1 个采样 */

/* 测 FDL(frac) 在 f_hz 处的复频响 (输入为单位余弦) */
static void measure_H(float frac, double f_hz, double* phase, double* mag) {
    static float x[MEAS_N], y[MEAS_N];
    float h[FDL_TAPS];
    double w = 2.0 * PI * f_hz / SR;
    fdl_coeffs(frac, h);
    for (int i = 0; i < MEAS_N; i++) x[i] = (float)cos(w * i);
    fdl_run(h, x, y, MEAS_N);
    fit_phasor(y + MEAS_SETTLE, MEAS_N - MEAS_SETTLE, MEAS_SETTLE, w, phase, mag);
}

/* M1: 亚采样延迟误差 < 1/20 采样 */
static int check_delay_error(void) {
    static const double tones[] = { 200, 400, 600, 800, 1000 };
    double worst = 0.0;
    printf("[M1] 亚采样延迟误差 (200-1000 Hz 相位斜率拟合群延迟)\n");
    for (int k = 0; k < NFRAC; k++) {
        double phi[5], w[5], mag;
        for (int t = 0; t < 5; t++) {
            measure_H(FRACS[k], tones[t], &phi[t], &mag);
            w[t] = 2.0 * PI * tones[t] / SR;
            if (t > 0) {   /* 解卷绕 */
                while (phi[t] - phi[t-1] >  PI) phi[t] -= 2.0 * PI;
                while (phi[t] - phi[t-1] < -PI) phi[t] += 2.0 * PI;
            }
        }
        double wm = 0, pm = 0;
        for (int t = 0; t < 5; t++) { wm += w[t]; pm += phi[t]; }
        wm /= 5; pm /= 5;
        double num = 0, den = 0;
        for (int t = 0; t < 5; t++) {
            num += (w[t] - wm) * (phi[t] - pm);
            den += (w[t] - wm) * (w[t] - wm);
        }
        double tau = -num / den;                /* 实测群延迟, 采样 */
        double cmd = FDL_CENTER + FRACS[k];     /* 指令延迟 */
        double err = tau - cmd;
        if (fabs(err) > worst) worst = fabs(err);
        printf("  frac=%+.2f  指令=%.2f  实测=%.5f  误差=%+.2e 采样\n",
               (double)FRACS[k], cmd, tau, err);
    }
    int ok = worst < 0.05;
    printf("  最大误差 = %.2e 采样 (要求 < 1/20 = 0.05) -> %s\n\n",
           worst, ok ? "PASS" : "FAIL");
    return !ok;
}

/* M2: 全通带相位偏差 vs 解析全通 e^{-jwD} < 3° */
#define M2_FREQS 64

static int check_phase_response(void) {
    double worst_dev = 0.0;
    printf("[M2] 通带相位偏差 vs 解析全通 e^{-jwD} (50 Hz .. %.0f Hz, %d 频点)\n",
           FDL_PASSBAND_HZ, M2_FREQS);
    for (int k = 0; k < NFRAC; k++) {
        double D = FDL_CENTER + FRACS[k];
        double max_dev = 0, max_db = 0, max_tau = 0;
        for (int m = 0; m < M2_FREQS; m++) {
            double f_hz = 50.0 + (FDL_PASSBAND_HZ - 50.0) * m / (M2_FREQS - 1);
            double phase, mag;
            measure_H(FRACS[k], f_hz, &phase, &mag);
            double w = 2.0 * PI * f_hz / SR;
            double dev = remainder(phase + w * D, 2.0 * PI);
            double dev_deg = fabs(dev) * 180.0 / PI;
            double db = fabs(20.0 * log10(mag));
            double tau_err = fabs(dev) / w;     /* 等效延迟误差, 采样 */
            if (dev_deg > max_dev) max_dev = dev_deg;
            if (db > max_db) max_db = db;
            if (tau_err > max_tau) max_tau = tau_err;
        }
        printf("  frac=%+.2f  最大相位偏差=%.4f°  等效延迟误差=%.5f 采样  幅度平坦度=%.4f dB\n",
               (double)FRACS[k], max_dev, max_tau, max_db);
        if (max_dev > worst_dev) worst_dev = max_dev;
    }
    int ok = worst_dev < 3.0;
    printf("  全通带最大相位偏差 = %.4f° (要求 < 3°) -> %s\n\n",
           worst_dev, ok ? "PASS" : "FAIL");
    return !ok;
}

/* M3: 延迟参数连续变化不产生爆音 */
static int check_click_free(void) {
    enum { N = 48000 };
    static float x[N], y[N];
    static const struct { const char* name; int ramp; double tone; } scen[] = {
        { "静态 f=+0.30 (参考)",      0, 1000 },
        { "线性扫描 -0.45..+0.45 / 1.0 s", 48000, 1000 },
        { "线性扫描 -0.45..+0.45 / 0.1 s",  4800, 1000 },
        { "线性扫描 -0.45..+0.45 / 10 ms",   480, 1000 },
        { "线性扫描 -0.45..+0.45 / 10 ms",   480, 8000 },
        { "正弦调制 2 Hz, 幅度 0.45",        -1, 1000 },
    };
    const double A = 0.5;
    int fails = 0;
    printf("[M3] 连续变参爆音检查 (1 s 正弦, 幅度 %.1f, 逐采样更新系数)\n", A);
    for (unsigned s = 0; s < sizeof(scen) / sizeof(scen[0]); s++) {
        double w = 2.0 * PI * scen[s].tone / SR;
        double rate;   /* |d frac / d i| 上限 */
        for (int i = 0; i < N; i++) x[i] = (float)(A * sin(w * i));
        for (int i = 0; i < N; i++) {
            float frac;
            if (scen[s].ramp > 0) {
                double t = (double)(i - 2048) / scen[s].ramp;
                t = t < 0 ? 0 : (t > 1 ? 1 : t);
                frac = (float)(-0.45 + 0.9 * t);
                rate = 0.9 / scen[s].ramp;
            } else if (scen[s].ramp == 0) {
                frac = 0.30f;
                rate = 0.0;
            } else {
                frac = (float)(0.45 * sin(2.0 * PI * 2.0 * i / SR));
                rate = 0.45 * 2.0 * PI * 2.0 / SR;
            }
            float h[FDL_TAPS];
            fdl_coeffs(frac, h);
            float acc = 0.0f;
            for (int m = 0; m < FDL_TAPS && m <= i; m++) acc += h[m] * x[i - m];
            y[i] = acc;
        }
        /* 输出瞬时频率被延迟调制缩放 (1+rate), 跳变上限随之放宽 */
        double bound = A * w * (1.0 + rate);
        double max_step = 0;
        int clicks = 0;
        for (int i = 2048; i < N; i++) {
            double step = fabs((double)y[i] - y[i - 1]);
            if (step > max_step) max_step = step;
            if (step > 1.5 * bound) clicks++;
        }
        double ratio = max_step / bound;
        int ok = ratio < 1.3 && clicks == 0;
        if (!ok) fails++;
        printf("  %-32s 最大跳变=%.5f  上限=%.5f  比值=%.3f  越界点=%d  %s\n",
               scen[s].name, max_step, bound, ratio, clicks, ok ? "OK" : "CLICK");
    }
    int ok = fails == 0;
    printf("  全部场景比值 < 1.3 且无越界点 -> %s\n\n", ok ? "PASS" : "FAIL");
    return !ok;
}

int main(void) {
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
    printf("right_source peakL=%d peakR=%d (impulse@64, 含固定延迟 %.1f 采样)\n",
           best_l, best_r, (double)(ITD_BULK + FDL_CENTER));

    Source left = { -3.0f, 0.0f, 0.0f, 1.0f };
    render(&left, 1, &l, mono, 256, out);
    int bl = 0, br = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*bl+0])) bl = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*br+1])) br = i;
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
    printf("mix_peak=%.4f (soft clip 上限 1.0)\n\n", peak(out, 256*2));

    int fails = 0;
    fails += check_delay_error();
    fails += check_phase_response();
    fails += check_click_free();
    printf(fails ? "FAIL: %d 项测量未达标\n" : "三项分数延迟测量全部达标\n", fails);
    return fails ? 1 : 0;
}
