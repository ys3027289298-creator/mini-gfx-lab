
/* mini-spatial-audio: binaural positioning of mono sources.
   ITD by azimuth — spherical-head (Woodworth) model, legacy sine model
   selectable via config — ILD ear gains, inverse-distance attenuation,
   soft-clip mixing of N sources into an interleaved stereo buffer. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SR 48000
#define MAX_FRAMES 4096
#define MAX_SRC 8

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

typedef enum {
    HEAD_MODEL_SINE,    /* 原模型: itd = max_itd * sin(az) */
    HEAD_MODEL_SPHERE   /* 球头模型 (Woodworth): 由头半径与声速推导 */
} HeadModel;

typedef struct {
    HeadModel model;
    float head_radius;   /* 头半径, m (SPHERE) */
    float sound_speed;   /* 声速, m/s (SPHERE) */
    float max_itd;       /* 最大耳间延迟, s (SINE) */
    float max_ild_db;    /* 90° 时远耳衰减, dB (SPHERE) */
} SpatialConfig;

static SpatialConfig default_config(void) {
    SpatialConfig c = {
        HEAD_MODEL_SPHERE,
        0.0875f,    /* 平均人头半径 ~8.75 cm */
        343.0f,     /* 20°C 空气声速 */
        0.0006f,    /* 原模型 0.6 ms */
        20.0f       /* 高频段典型头影衰减 ~20 dB */
    };
    return c;
}

typedef struct {
    float x, y, z;      /* world position, meters; listener at origin facing -z */
    float gain;
} Source;

typedef struct {
    float yaw;          /* radians, 0 = facing -z */
} Listener;

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* azimuth in radians, 0 = front, +pi/2 = right */
static float azimuth_of(const Source* s, const Listener* l) {
    float dx = s->x, dz = s->z;
    float c = cosf(l->yaw), sn = sinf(l->yaw);
    float xr =  c * dx - sn * dz;
    float zr =  sn * dx + c * dz;
    return atan2f(xr, -zr);
}

/* Woodworth 球头 ITD 曲线(归一化前的路径差因子):
   |az| <= pi/2: sin(a) + a  (直射 + 绕球面爬行)
   |az| >  pi/2: pi - a + sin(a) */
static float woodworth_factor(float a) {
    a = clampf(fabsf(a), 0.0f, (float)M_PI);
    if (a <= (float)M_PI_2) return sinf(a) + a;
    return (float)M_PI - a + sinf(a);
}

/* 方位角 -> 耳间时差(秒)。返回值 = t_left - t_right:
   >0 表示声源在右,左耳滞后;<0 表示声源在左,右耳滞后。 */
static float itd_seconds(const SpatialConfig* cfg, float az) {
    float itd;
    if (cfg->model == HEAD_MODEL_SINE) {
        itd = cfg->max_itd * sinf(fabsf(az));
    } else {
        itd = (cfg->head_radius / cfg->sound_speed) * woodworth_factor(az);
    }
    return az >= 0.0f ? itd : -itd;
}

/* 左右耳增益。SPHERE: 按归一化 Woodworth 曲线插值远耳衰减,
   再做等功率归一;SINE: 原 sqrt 声像分配。 */
static void ear_gains(const SpatialConfig* cfg, float az, float* gl, float* gr) {
    if (cfg->model == HEAD_MODEL_SINE) {
        float t = (sinf(az) + 1.0f) * 0.5f;  /* 0 = hard left, 1 = hard right */
        *gl = sqrtf(1.0f - t);
        *gr = sqrtf(t);
        return;
    }
    float w = woodworth_factor(az) / (1.0f + (float)M_PI_2);  /* 0..1, 90° 处为 1 */
    float far  = powf(10.0f, -cfg->max_ild_db * w / 20.0f);
    float near = 1.0f;
    float n = sqrtf((near * near + far * far) * 0.5f);        /* 等功率归一 */
    near /= n; far /= n;
    if (az >= 0.0f) { *gr = near; *gl = far; }
    else            { *gl = near; *gr = far; }
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

static void render(const SpatialConfig* cfg, const Source* srcs, int n,
                   const Listener* l, const float* mono, int frames,
                   float* out /* interleaved */) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < n; k++) {
        float az = azimuth_of(&srcs[k], l);
        float itd = itd_seconds(cfg, az);
        int dl = itd > 0.0f ? (int)lroundf(itd * SR) : 0;   /* 远耳延迟 */
        int dr = itd < 0.0f ? (int)lroundf(-itd * SR) : 0;
        float gl, gr; ear_gains(cfg, az, &gl, &gr);
        float g = srcs[k].gain * distance_gain(&srcs[k]);
        for (int i = 0; i < frames; i++) {
            int jl = i - dl, jr = i - dr;
            float sl = (jl >= 0 && jl < frames) ? mono[jl] : 0.0f;
            float sr = (jr >= 0 && jr < frames) ? mono[jr] : 0.0f;
            out[2*i + 0] += sl * g * gl;
            out[2*i + 1] += sr * g * gr;
        }
    }
    for (int i = 0; i < frames * 2; i++) out[i] = soft_clip(out[i]);
}

static float peak(const float* buf, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(buf[i]) > m) m = fabsf(buf[i]);
    return m;
}

/* KEMAR 假人水平面实测 ITD 近似值(综合 Kuhn 1977 / CIPIC 数据库均值的
   文献量级,仅作 sanity check 参考,非精确数据集)。单位: μs。 */
static const struct { float az_deg; float itd_us; } KEMAR_ITD[] = {
    {   0,   0 }, {  15, 140 }, {  30, 265 }, {  45, 375 },
    {  60, 480 }, {  75, 570 }, {  90, 660 }, { 105, 600 },
    { 120, 505 }, { 135, 390 }, { 150, 270 }, { 165, 140 },
    { 180,   0 },
};

static void compare_with_measured(const SpatialConfig* cfg, const char* name) {
    float se = 0.0f, maxe = 0.0f;
    int n = (int)(sizeof(KEMAR_ITD) / sizeof(KEMAR_ITD[0]));
    printf("--- %s: ITD vs KEMAR 实测 ---\n", name);
    printf("  az(deg)  model(us)  kemar(us)  err(us)\n");
    for (int i = 0; i < n; i++) {
        float az = KEMAR_ITD[i].az_deg * (float)M_PI / 180.0f;
        float model = fabsf(itd_seconds(cfg, az)) * 1e6f;
        float err = model - KEMAR_ITD[i].itd_us;
        se += err * err;
        if (fabsf(err) > maxe) maxe = fabsf(err);
        printf("  %7.0f  %9.1f  %10.1f  %7.1f\n",
               (double)KEMAR_ITD[i].az_deg, (double)model,
               (double)KEMAR_ITD[i].itd_us, (double)err);
    }
    printf("  RMS err = %.1f us, max err = %.1f us\n\n",
           (double)sqrtf(se / n), (double)maxe);
}

static void print_ear_gains(const SpatialConfig* cfg) {
    static const float azs[] = { 0, 30, 45, 60, 90, 135, 180 };
    printf("--- %s: 左右耳增益 ---\n",
           cfg->model == HEAD_MODEL_SPHERE ? "SPHERE" : "SINE");
    printf("  az(deg)   gL      gR      ILD(dB)\n");
    for (int i = 0; i < 7; i++) {
        float az = azs[i] * (float)M_PI / 180.0f;
        float gl, gr; ear_gains(cfg, az, &gl, &gr);
        printf("  %7.0f  %.4f  %.4f  %+7.2f\n", (double)azs[i],
               (double)gl, (double)gr,
               (double)(20.0f * log10f(gr / gl)));
    }
    printf("\n");
}

int main(void) {
    float mono[MAX_FRAMES];
    float out[MAX_FRAMES * 2];
    for (int i = 0; i < 256; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;  /* impulse at 64 */

    SpatialConfig cfg = default_config();
    Listener l = { 0.0f };

    Source right = { 3.0f, 0.0f, 0.0f, 1.0f };
    render(&cfg, &right, 1, &l, mono, 256, out);
    int best_l = 0, best_r = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*best_l+0])) best_l = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*best_r+1])) best_r = i;
    }
    printf("right_source peakL=%d peakR=%d (impulse@64, ITD=%d samples)\n",
           best_l, best_r, best_l - best_r);

    Source left = { -3.0f, 0.0f, 0.0f, 1.0f };
    render(&cfg, &left, 1, &l, mono, 256, out);
    int bl = 0, br = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*bl+0])) bl = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*br+0])) br = i;
    }
    printf("left_source peakL=%d peakR=%d (impulse@64, ITD=%d samples)\n",
           bl, br, br - bl);

    Source near = { 0.0f, 0.0f, -1.0f, 1.0f };
    Source far  = { 0.0f, 0.0f, -9.0f, 1.0f };
    render(&cfg, &near, 1, &l, mono, 256, out); float pn = peak(out, 256*2);
    render(&cfg, &far, 1, &l, mono, 256, out);  float pf = peak(out, 256*2);
    printf("attenuation near=%.4f far=%.4f ratio=%.3f\n", pn, pf, pf / pn);

    Source many[4] = { {1,0,0,1}, {-1,0,0,1}, {0,0,-1,1}, {0,0,1,1} };
    float loud[MAX_FRAMES];
    for (int i = 0; i < 256; i++) loud[i] = 0.9f;
    render(&cfg, many, 4, &l, loud, 256, out);
    printf("mix_peak=%.4f (soft clip 上限 1.0)\n\n", peak(out, 256*2));

    /* 球头模型: 与实测数据对比 + 耳增益 */
    compare_with_measured(&cfg, "SPHERE r=8.75cm c=343m/s");
    print_ear_gains(&cfg);

    /* 退化到原正弦模型,同样给出误差 */
    SpatialConfig sine = default_config();
    sine.model = HEAD_MODEL_SINE;
    compare_with_measured(&sine, "SINE max_itd=0.6ms (原模型)");
    print_ear_gains(&sine);
    return 0;
}
