/* agc: 带前瞻的自动增益，和固定静态增益作对照。

   静态增益：在开头 200 ms 上标定一次，之后不再变化。人声忽大忽小的时候，
   标定段偏小就会在后面的响段削顶，标定段偏大则安静段听不见。

   前瞻 AGC：
     - 前瞻窗口 1 ms（48 帧 @48 kHz），在窗口内取预测峰值；
     - 增益按 dB 逐样本限速（attack/release 两档），不会出现瞬时跳变；
     - 延迟正好是前瞻长度，低于 2 ms；
     - 低于噪声门时增益只降不升，且绝不超过 0 dB——静音段不抬底噪。

   信号流：
     in --> delay(48) ---------------------> * gain --> out
       \--> 窗口峰值 --> 目标增益 --> 限速 --> gain
   输出 out[i] 用 x[i-48] 乘当前增益，而当前增益已经看到 x[i] 及之前 48 个
   输入样本，所以渐变在瞬态到达之前就完成了。 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define SR                 48000
#define LOOKAHEAD_MS       1
#define LOOKAHEAD_FRAMES   (SR * LOOKAHEAD_MS / 1000)   /* 48 帧 */
#define MAX_SAMPLES        (2 * SR)
#define AGC_PI             3.14159265358979323846f

/* ---- 通用小工具 -------------------------------------------------------- */

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float lin_to_db(float g) {
    return 20.0f * log10f(g < 1e-9f ? 1e-9f : g);
}

static float db_to_lin(float db) {
    return powf(10.0f, db / 20.0f);
}

/* ---- 静态增益（对照组） ------------------------------------------------ */

typedef struct {
    float gain;
    int   calibrated;
} StaticGain;

/* 用开头 calib_n 个样本的峰值标定一次，之后固定。 */
static void static_calibrate(StaticGain* s, const float* x, int calib_n) {
    float p = 0.0f;
    for (int i = 0; i < calib_n; i++) {
        float a = fabsf(x[i]);
        if (a > p) p = a;
    }
    if (p < 1e-6f) p = 1e-6f;
    s->gain = clampf(0.9f / p, 0.05f, 10.0f);
    s->calibrated = 1;
}

/* 固定增益 + 硬削顶，模拟定点总线上的静态前级。 */
static void static_process(const StaticGain* s, const float* x, int n,
                           float* y) {
    float g = s->calibrated ? s->gain : 1.0f;
    for (int i = 0; i < n; i++) y[i] = clampf(x[i] * g, -1.0f, 1.0f);
}

/* ---- 前瞻 AGC ---------------------------------------------------------- */

typedef struct {
    float target_db;       /* 目标峰值，例如 -3 dBFS */
    float gate_db;         /* 预测峰值低于此值视为静音 */
    float max_boost_db;    /* 最大提升量，避免把安静段硬拽上来 */
    float attack_db;       /* 每样本最大下降，dB */
    float release_db;      /* 每样本最大上升，dB */
    float hold_ms;         /* 峰值过后暂停释放的时间 */
} AgcConfig;

typedef struct {
    AgcConfig cfg;
    float delay[LOOKAHEAD_FRAMES];   /* 环形延迟线，保存 x[i-48..i-1] */
    int   delay_pos;
    float gain_db;                   /* 当前增益，dB */
    int   hold_left;                 /* 释放保持剩余样本数 */
    int   lookahead;
    int   hold_samples;
    float last_gain_db;              /* 诊断用 */
    float last_peak;                 /* 诊断用：窗口预测峰值 */
} Agc;

static void agc_init(Agc* a, const AgcConfig* cfg) {
    memset(a, 0, sizeof(*a));
    a->cfg = *cfg;
    a->lookahead = LOOKAHEAD_FRAMES;
    a->hold_samples = (int)lroundf(cfg->hold_ms * 0.001f * SR);
    a->gain_db = 0.0f;
    a->last_gain_db = 0.0f;
}

/* 升/降都要经过同一套限速；hold 只在下降时重新计时，避免持续人声把
   释放永久锁死。静音门内只允许向下走，且目标不超过 0 dB。 */
static void agc_advance(Agc* a, float desired_db, int gated) {
    float g = a->gain_db;
    if (desired_db < g) {
        g = fmaxf(desired_db, g - a->cfg.attack_db);
        a->hold_left = a->hold_samples;
    } else if (desired_db > g && !gated) {
        if (a->hold_left == 0) g = fminf(desired_db, g + a->cfg.release_db);
    }
    a->gain_db = g;
    if (a->hold_left > 0) a->hold_left--;
}

static void agc_process(Agc* a, const float* in, int n, float* out) {
    const int L = a->lookahead;

    for (int i = 0; i < n; i++) {
        /* 延迟线上最老的样本，就是这一帧要输出的 x[i-L] */
        float xd = a->delay[a->delay_pos];

        /* 前瞻窗口 [i-L, i] 的峰值 */
        float p = fabsf(in[i]);
        for (int k = 0; k < L; k++) {
            float v = fabsf(a->delay[k]);
            if (v > p) p = v;
        }
        a->last_peak = p;

        /* 静音门：预测峰值低于门限时冻结增益。不提升（底噪不动），
           也不释放（避免底噪随增益释放被抬起来）。 */
        int gated = (p <= db_to_lin(a->cfg.gate_db));

        float desired;
        if (gated) {
            desired = 0.0f;    /* 绝不越过 0 dB，因此底噪不会被抬起来 */
        } else {
            desired = a->cfg.target_db - lin_to_db(p);
            desired = fminf(desired, a->cfg.max_boost_db);
            desired = fmaxf(desired, -60.0f);
        }

        agc_advance(a, desired, gated);
        a->last_gain_db = a->gain_db;

        out[i] = clampf(xd * db_to_lin(a->gain_db), -1.0f, 1.0f);

        a->delay[a->delay_pos] = in[i];
        a->delay_pos = (a->delay_pos + 1) % L;
    }
}

/* ---- 峰值 / 包络测量 ---------------------------------------------------- */

typedef struct {
    float peak_lin;
    float rms_lin;
} Levels;

static Levels measure(const float* x, int n) {
    Levels lv = { 0.0f, 0.0f };
    double acc = 0.0;
    for (int i = 0; i < n; i++) {
        float a = fabsf(x[i]);
        if (a > lv.peak_lin) lv.peak_lin = a;
        acc += (double)x[i] * x[i];
    }
    lv.rms_lin = (float)sqrt(acc / (double)(n > 0 ? n : 1));
    return lv;
}

/* 包络跟随：1 ms attack / 50 ms release，只用于观测，不参与处理。 */
typedef struct {
    float env;
    float at_coef;
    float rel_coef;
} EnvFollower;

static void env_init(EnvFollower* e, float at_ms, float rel_ms) {
    e->env = 0.0f;
    e->at_coef = expf(-1.0f / (at_ms * 0.001f * SR));
    e->rel_coef = expf(-1.0f / (rel_ms * 0.001f * SR));
}

static float env_step(EnvFollower* e, float x) {
    float a = fabsf(x);
    float coef = (a > e->env) ? e->at_coef : e->rel_coef;
    e->env = coef * e->env + (1.0f - coef) * a;
    return e->env;
}

/* ---- 测试信号 ---------------------------------------------------------- */

typedef struct {
    float level;
    int   ms;
} Seg;

/* 忽大忽小：响段与近静音交替，末尾再给一段很低的底噪。 */
static const Seg SCRIPT[] = {
    { 0.12f, 120 }, { 0.0f, 120 },
    { 0.90f,  60 }, { 0.0f, 120 },
    { 0.50f,  60 }, { 0.0f,  80 },
    { 0.06f,  60 }, { 0.0f, 120 },
    { 0.95f,  40 }, { 0.0f, 160 },
    { 0.30f,  60 }, { 0.0f, 200 },
};

static void put_tone(float* buf, int start, int len, float level) {
    for (int k = 0; k < len; k++) {
        float t = (float)k / SR;
        float am = 0.75f + 0.25f * sinf(2.0f * AGC_PI * 4.0f * t);
        buf[start + k] = level * am * sinf(2.0f * AGC_PI * 220.0f * t);
    }
}

static int build_script(float* buf) {
    int pos = 0;
    for (size_t s = 0; s < sizeof(SCRIPT) / sizeof(SCRIPT[0]); s++) {
        int len = SCRIPT[s].ms * SR / 1000;
        if (SCRIPT[s].level > 0.0f)
            put_tone(buf, pos, len, SCRIPT[s].level);
        else
            memset(buf + pos, 0, sizeof(float) * (size_t)len);
        pos += len;
    }
    return pos;
}

static float noise_sample(uint32_t* state) {
    *state = *state * 1664525u + 1013904223u;
    return ((float)(*state >> 8) / 16777216.0f) * 2.0f - 1.0f;
}

/* 伪随机噪声：确定性，且不依赖 libm 的 rand 实现。 */
static void fill_noise(float* buf, int n, float level, uint32_t* state) {
    for (int i = 0; i < n; i++) buf[i] = level * noise_sample(state);
}

/* 低底噪 -> 响段 -> 低底噪：检查静音段是否被抬起来。
   底噪用幅值 2e-4（-74 dBFS）的确定性噪声，返回尾段起点。 */
#define FLOOR_AMP  2.0e-4f

static int build_floor_test(float* buf, int* noise_start, int* tone_start,
                            int* noise2_start) {
    uint32_t rng = 0x9e3779b9u;
    int pos = 0;
    int n1 = 300 * SR / 1000;   /* 比 release 回到 0 dB 的时间更长 */
    *noise_start = pos;
    fill_noise(buf + pos, n1, FLOOR_AMP, &rng);
    pos += n1;
    int ns = 60 * SR / 1000;
    *tone_start = pos;
    put_tone(buf, pos, ns, 0.9f);
    pos += ns;
    int n2 = 400 * SR / 1000;   /* 比 release 的 0.01 dB/样本 更长 */
    *noise2_start = pos;
    fill_noise(buf + pos, n2, FLOOR_AMP, &rng);
    pos += n2;
    return pos;
}

/* ---- 输出 -------------------------------------------------------------- */

static int failures = 0;

static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void print_levels(const char* tag, const Levels* lv, float env_max) {
    printf("%-7s peak=%.4f (%+.2f dBFS)  rms=%.4f (%+.2f dBFS)  env_max=%.2f dBFS\n",
           tag, lv->peak_lin, lin_to_db(lv->peak_lin),
           lv->rms_lin, lin_to_db(lv->rms_lin), lin_to_db(env_max));
}

int main(void) {
    static float raw[MAX_SAMPLES];
    static float stc[MAX_SAMPLES];
    static float dyn[MAX_SAMPLES];
    static float env_in[MAX_SAMPLES];
    static float env_stc[MAX_SAMPLES];
    static float env_dyn[MAX_SAMPLES];

    int n = build_script(raw);
    if (n > MAX_SAMPLES) n = MAX_SAMPLES;

    StaticGain sg = { 1.0f, 0 };
    /* 只在前 200 ms 标定：这一段只有 -18 dBFS 的轻声，响段在后面 */
    static_calibrate(&sg, raw, 200 * SR / 1000);
    static_process(&sg, raw, n, stc);

    AgcConfig cfg = {
        .target_db    = -3.0f,
        .gate_db      = -60.0f,
        .max_boost_db = 12.0f,
        .attack_db    = 0.4f,     /* 0.4 dB/样本 = 19200 dB/s */
        .release_db   = 0.005f,   /* 0.005 dB/样本 = 240 dB/s */
        .hold_ms      = 15.0f,
    };
    Agc agc;
    agc_init(&agc, &cfg);
    agc_process(&agc, raw, n, dyn);

    Levels l_in  = measure(raw, n);
    Levels l_stc = measure(stc, n);
    Levels l_dyn = measure(dyn, n);

    EnvFollower e_in, e_stc, e_dyn;
    env_init(&e_in, 1.0f, 50.0f);
    env_init(&e_stc, 1.0f, 50.0f);
    env_init(&e_dyn, 1.0f, 50.0f);
    float emax_in = 0.0f, emax_stc = 0.0f, emax_dyn = 0.0f;
    for (int i = 0; i < n; i++) {
        env_in[i]  = env_step(&e_in, raw[i]);
        env_stc[i] = env_step(&e_stc, stc[i]);
        env_dyn[i] = env_step(&e_dyn, dyn[i]);
        if (env_in[i]  > emax_in)  emax_in  = env_in[i];
        if (env_stc[i] > emax_stc) emax_stc = env_stc[i];
        if (env_dyn[i] > emax_dyn) emax_dyn = env_dyn[i];
    }

    printf("lookahead=%d frames (%.2f ms)\n",
           agc.lookahead, 1000.0f * (float)agc.lookahead / (float)SR);
    printf("static_gain=%.3f (%.2f dB, 标定段=200 ms)\n",
           sg.gain, lin_to_db(sg.gain));
    printf("rate_limit attack=%.3f dB/样本 (%.0f dB/s) release=%.3f dB/样本 (%.0f dB/s)\n",
           cfg.attack_db, cfg.attack_db * SR, cfg.release_db, cfg.release_db * SR);
    printf("attack_window=%.2f dB  max_boost=%.1f dB  ->  最大回落需求 %.2f dB\n",
           cfg.attack_db * LOOKAHEAD_FRAMES, cfg.max_boost_db,
           cfg.max_boost_db - cfg.target_db);
    printf("\n");
    print_levels("input",   &l_in,  emax_in);
    print_levels("static",  &l_stc, emax_stc);
    print_levels("dynamic", &l_dyn, emax_dyn);

    printf("\n包络（每 100 ms，dBFS）\n");
    printf("%8s %10s %12s %12s\n", "t_ms", "in_peak", "static_env", "dynamic_env");
    int block = 100 * SR / 1000;
    for (int s = 0; s < n; s += block) {
        int len = (s + block <= n) ? block : n - s;
        Levels lp = measure(raw + s, len);
        printf("%8d %10.2f %12.2f %12.2f\n",
               s * 1000 / SR,
               lin_to_db(lp.peak_lin),
               lin_to_db(env_stc[s + len - 1]),
               lin_to_db(env_dyn[s + len - 1]));
    }

    printf("\n静态增益削顶 / 前瞻 AGC 未削顶的样本占比：");
    {
        int stc_clip = 0, dyn_clip = 0;
        for (int i = 0; i < n; i++) {
            if (fabsf(stc[i]) >= 0.999f) stc_clip++;
            if (fabsf(dyn[i])  >= 0.999f) dyn_clip++;
        }
        printf("static=%d%% dynamic=%d%%\n",
               stc_clip * 100 / n, dyn_clip * 100 / n);
    }

    /* ---- 自检 ---------------------------------------------------------- */

    /* 1. 固定增益在响段削顶 */
    check("static_peak_clips", l_stc.peak_lin >= 0.999f);

    /* 2. 前瞻版峰值压在目标附近（-3 dBFS = 0.708） */
    check("dynamic_peak_bounded", l_dyn.peak_lin <= 0.85f);

    /* 3. 动态包络整体低于静态包络 */
    check("dynamic_env_lower", emax_dyn <= emax_stc);

    /* 4. 延迟就是前瞻长度，且 < 2 ms */
    {
        static float impulse[MAX_SAMPLES];
        static float imp_out[256];
        memset(impulse, 0, sizeof(float) * 256);
        memset(imp_out, 0, sizeof(imp_out));
        impulse[0] = 0.5f;
        Agc a2;
        agc_init(&a2, &cfg);
        agc_process(&a2, impulse, 256, imp_out);
        int first = -1;
        for (int i = 0; i < 256; i++)
            if (fabsf(imp_out[i]) > 1e-6f) { first = i; break; }
        printf("latency first_output_at=%d frames (%.3f ms)\n",
               first, (float)first * 1000.0f / (float)SR);
        check("latency_under_2ms",
              first == LOOKAHEAD_FRAMES && (float)first * 1000.0f / (float)SR < 2.0f);
    }

    /* 5. 增益变化速率受限（逐样本记录实际增益） */
    {
        Agc a3;
        agc_init(&a3, &cfg);
        float prev = a3.last_gain_db;
        float max_step = 0.0f;
        float one;
        for (int i = 0; i < n; i++) {
            agc_process(&a3, raw + i, 1, &one);
            float d = fabsf(a3.last_gain_db - prev);
            if (d > max_step) max_step = d;
            prev = a3.last_gain_db;
        }
        printf("max_gain_step=%.4f dB/样本 (attack 上限 %.3f)\n",
               max_step, cfg.attack_db);
        check("gain_rate_limited", max_step <= cfg.attack_db + 1e-4f);
    }

    /* 6. 静音段不抬底噪 */
    {
        static float floor_raw[MAX_SAMPLES];
        static float floor_stc[MAX_SAMPLES];
        static float floor_dyn[MAX_SAMPLES];
        int noise_start, tone_start, noise2_start;
        int fn = build_floor_test(floor_raw, &noise_start, &tone_start,
                                  &noise2_start);

        /* 固定增益在前 300 ms 的底噪上标定，于是把底噪放大 ~13 dB */
        StaticGain fsg = { 1.0f, 0 };
        static_calibrate(&fsg, floor_raw, noise_start + 300 * SR / 1000);
        static_process(&fsg, floor_raw, fn, floor_stc);

        Agc fagc;
        agc_init(&fagc, &cfg);
        agc_process(&fagc, floor_raw, fn, floor_dyn);

        int tail = 200 * SR / 1000;
        int tail_at = fn - tail;
        Levels in_tail  = measure(floor_raw + tail_at, tail);
        Levels stc_tail = measure(floor_stc + tail_at, tail);
        Levels dyn_tail = measure(floor_dyn + tail_at, tail);
        printf("floor_tail input=%.6f static=%.6f dynamic=%.6f gain=%.2f dB\n",
               in_tail.peak_lin, stc_tail.peak_lin, dyn_tail.peak_lin,
               fagc.last_gain_db);
        check("silence_not_lifted",
              dyn_tail.peak_lin <= in_tail.peak_lin * 1.001f);
        check("no_boost_in_silence", fagc.last_gain_db <= 1e-6f + 1e-6f);
        check("static_lifts_floor", stc_tail.peak_lin > in_tail.peak_lin * 2.0f);

        /* 逐样本检查整个底噪段：增益始终 <= 0 dB，输出从不高于输入。 */
        Agc chk;
        agc_init(&chk, &cfg);
        float max_db = -1e9f, max_delta = 0.0f;
        for (int i = 0; i < fn; i++) {
            float y;
            agc_process(&chk, floor_raw + i, 1, &y);
            if (i >= noise2_start) {
                if (chk.last_gain_db > max_db) max_db = chk.last_gain_db;
                /* 比较的是同一时间点的信号：y 是 x[i-L] 经增益后的结果 */
                int j = i - chk.lookahead;
                if (j >= 0) {
                    float d = fabsf(y) - fabsf(floor_raw[j]);
                    if (d > max_delta) max_delta = d;
                }
            }
        }
        printf("floor_noise2 max_gain=%.3f dB max_out_minus_in=%.2e\n",
               max_db, max_delta);
        check("gain_never_positive_in_noise", max_db <= 1e-6f);
        check("noise_never_amplified", max_delta <= 1e-6f);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
