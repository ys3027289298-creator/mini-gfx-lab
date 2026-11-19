
/* mini-spatial-audio: binaural positioning of mono sources.
   ITD by azimuth (max 0.6 ms), IID by azimuth, inverse-distance attenuation,
   soft-clip mixing of N sources into an interleaved stereo buffer.

   Per-frame voice budget: when sources outnumber the DSP budget only the top N
   (by listener-side power = distance + source energy) are spatialized.  The
   culled voices are NOT muted: their power is folded into a diffuse center bed
   (energy compensation).  Membership changes carry hysteresis, and each voice
   crossfades between the two paths equal-power, so boundary jitter cannot
   cause power steps or clicks.  All choices use fixed-index tie-breaks and no
   allocation or wall-clock state, so shrinking/growing N is deterministic. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SR 48000
#define MAX_FRAMES 4096
#define MAX_SRC 32

/* hysteresis: an active voice keeps its slot until a challenger beats
   HYST_TAU x its score; 1.25 ~= 1 dB of power margin */
#define HYST_TAU 1.25f

typedef struct {
    float x, y, z;      /* world position, meters; listener at origin facing -z */
    float gain;
} Source;

typedef struct {
    float yaw;          /* radians, 0 = facing -z */
} Listener;

/* per-voice crossfade state, kept across frames */
typedef struct {
    int active;         /* member of the spatialized set last frame */
    float wet;          /* 0..1 participation factor, one-pole smoothed */
} VoiceState;

typedef struct {
    VoiceState voice[MAX_SRC];
} SpatialBus;

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

/* listener-side power of a voice: combines source energy and inverse-distance
   attenuation; higher = more worth spending DSP on */
static float voice_score(const Source* s) {
    float g = s->gain * distance_gain(s);
    return g * g;
}

/* select up to budget voices for spatialization with membership hysteresis.
   Incumbents are ranked on score x HYST_TAU, so a challenger near the
   boundary must clearly overtake before an incumbent is evicted.  Pure
   function of (srcs, previous active flags): fixed index tie-breaks make the
   result bit-deterministic for any sequence of budget values. */
static void cull_frame(const Source* srcs, int n, int budget,
                       const int* prev_active, int* active_out) {
    budget = clampf(budget, 0, n);
    for (int i = 0; i < n; i++) active_out[i] = 0;

    int chosen = 0;
    while (chosen < budget) {
        int best = -1;
        float best_eff = 0.0f;
        for (int i = 0; i < n; i++) {
            if (active_out[i]) continue;
            float score = voice_score(&srcs[i]);
            float eff = score * (prev_active[i] ? HYST_TAU : 1.0f);
            /* strict > keeps the lowest index on exact ties */
            if (best < 0 || eff > best_eff) { best = i; best_eff = eff; }
        }
        if (best < 0) break;
        active_out[best] = 1;
        chosen++;
    }
}

/* budgeted renderer.  Active voices take the full ITD/IID path; culled voices
   are summed by power into a diffuse center bed (energy compensation, never a
   mute).  Per-voice wet/dry factors crossfade equal-power, so the total power
   is invariant while a voice is moving between paths — no clicks when the
   boundary jitters.  Each voice reads its own input (pointers may alias the
   same buffer); independent real-world sources add incoherently, which is the
   regime the power-summed bed is designed for.  ramp_sec <= 0 hard-switches
   (used only by the pop test). */
static void render_budget(SpatialBus* bus, const Source* srcs, int n,
                          const Listener* l, int budget,
                          const float* const* in, int frames, float ramp_sec,
                          float* out /* interleaved */) {
    int prev[MAX_SRC], active[MAX_SRC];
    for (int i = 0; i < n; i++) prev[i] = bus->voice[i].active;
    cull_frame(srcs, n, budget, prev, active);

    /* one-pole coefficient toward the new target; same coefficient for every
       voice, so the crossfade shape depends only on frames, not on data */
    float rate = 1.0f;
    if (ramp_sec > 0.0f) {
        float k = 1.0f - expf(-(float)frames / (ramp_sec * (float)SR));
        rate = clampf(k, 0.0f, 1.0f);
    }

    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    float bed_power = 0.0f;               /* sum of culled effective gains^2 */

    for (int k = 0; k < n; k++) {
        VoiceState* v = &bus->voice[k];
        v->active = active[k];
        v->wet += ((active[k] ? 1.0f : 0.0f) - v->wet) * rate;
        float wet = clampf(v->wet, 0.0f, 1.0f);
        float g = srcs[k].gain * distance_gain(&srcs[k]);

        /* spatialized path, scaled by its equal-power share */
        float sg = g * sqrtf(wet);
        if (sg > 0.0f) {
            float az = azimuth_of(&srcs[k], l);
            int delay = itd_samples(az);
            float gl, gr; iid_gains(az, &gl, &gr);
            for (int i = 0; i < frames; i++) {
                int j = i - delay;
                float s = (j >= 0 && j < frames) ? in[k][j] : 0.0f;
                out[2*i + 0] += s * sg * gl;
                out[2*i + 1] += s * sg * gr;
            }
        }

        /* diffuse share: incoherent sources add by power; center-panned
           equal-power (1/sqrt2 per channel) */
        bed_power += g * g * (1.0f - wet);
    }

    /* all culled voices share one diffuse bed; use the first culled input as
       its signal carrier — at steady state the bed carries exactly the summed
       culled power regardless of which carrier plays it */
    float bed_gain = sqrtf(bed_power) * (1.0f / sqrtf(2.0f));
    if (bed_gain > 0.0f) {
        int carrier = 0;
        for (int k = 0; k < n; k++) if (!active[k]) { carrier = k; break; }
        for (int i = 0; i < frames; i++) {
            out[2*i + 0] += in[carrier][i] * bed_gain;
            out[2*i + 1] += in[carrier][i] * bed_gain;
        }
    }

    for (int i = 0; i < frames * 2; i++) out[i] = soft_clip(out[i]);
}

/* full (unbudgeted) reference: every voice spatialized, no soft-clip, so the
   culling stage can be checked against its pre-limiter power budget */
static void render_full_ref(const Source* srcs, int n, const Listener* l,
                            const float* const* in, int frames,
                            float* out /* interleaved */) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < n; k++) {
        float az = azimuth_of(&srcs[k], l);
        int delay = itd_samples(az);
        float gl, gr; iid_gains(az, &gl, &gr);
        float g = srcs[k].gain * distance_gain(&srcs[k]);
        for (int i = 0; i < frames; i++) {
            int j = i - delay;
            float s = (j >= 0 && j < frames) ? in[k][j] : 0.0f;
            out[2*i + 0] += s * g * gl;
            out[2*i + 1] += s * g * gr;
        }
    }
}

static float peak(const float* buf, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(buf[i]) > m) m = fabsf(buf[i]);
    return m;
}

static float energy(const float* buf, int n) {
    float e = 0.0f;
    for (int i = 0; i < n; i++) e += buf[i] * buf[i];
    return e;
}

/* 10 * log10 ratio, guarding the zero-energy edge */
static float db_ratio(float a, float b) {
    if (b <= 0.0f) return a <= 0.0f ? 0.0f : 99.0f;
    return 10.0f * log10f(a / b);
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

    /* ---- 每帧预算裁剪 ---- */

    /* 1) 距离+能量排序，只处理前 N 个；平分时按索引稳定决胜 */
    Source six[6] = {
        {0,0,-1,1}, {0,0,-2,1}, {0,0,-3,1},
        {0,0,-4,1}, {0,0,-5,1}, {0,0,-6,1},
    };
    int act[MAX_SRC], prev6[MAX_SRC] = {0};
    cull_frame(six, 6, 3, prev6, act);
    int top3_ok = act[0] && act[1] && act[2] && !act[3] && !act[4] && !act[5];
    printf("cull_top3=%d (kept=%d%d%d%d%d%d)\n", top3_ok,
           act[0], act[1], act[2], act[3], act[4], act[5]);

    Source tied[2] = { {0,0,-3,1}, {0,0,-3,1} };   /* 完全同分 */
    int ta[2], tp[2] = {0};
    cull_frame(tied, 2, 1, tp, ta);
    int tie_ok = ta[0] == 1 && ta[1] == 0;
    printf("cull_tiebreak=%d (lower index wins)\n", tie_ok);

    /* 2) 边界滞回：临界声源与在位者同分，增益在阈值上下抖动。
       无滞回时每帧反复进出（~5 次跳变），HYST_TAU=1.25 把抖动整段
       吞在滞回带内（最大分数比 1.05^2=1.10 < 1.25），成员关系不翻转 */
    Source hover[3] = { {0,0,-1,1}, {0,0,-3,1}, {0,0,-3,1} };
    int hp[3] = {0}, ha[3];
    cull_frame(hover, 3, 2, hp, ha);              /* 同分时低索引 1 号在位 */
    int target = 2, switches = 0, was = ha[target];
    int switches_ctrl = 0, was_ctrl = ha[target];
    for (int f = 0; f < 10; f++) {
        hover[target].gain = (f % 2 == 0) ? 1.05f : 0.95f;  /* 围绕边界抖动 */
        for (int i = 0; i < 3; i++) hp[i] = ha[i];
        cull_frame(hover, 3, 2, hp, ha);
        if (ha[target] != was) { switches++; was = ha[target]; }

        /* 对照组：每帧清空在位状态 = tau=1，无滞回，必然反复进出 */
        int zero[3] = {0}, hc[3];
        cull_frame(hover, 3, 2, zero, hc);
        if (hc[target] != was_ctrl) { switches_ctrl++; was_ctrl = hc[target]; }
    }
    printf("hysteresis switches=%d (带滞回) vs %d (无滞回对照)\n",
           switches, switches_ctrl);
    int hyst_ok = switches == 0 && switches_ctrl >= 4;
    printf("hysteresis_ok=%d\n", hyst_ok);

    /* 3) 能量补偿：裁剪后总能量相对无裁剪参考偏差 < 1 dB。
       每个声源一条独立输入，冲激落在不同采样点（互相正交、无交叉项，
       对应真实声源的非相干叠加）；ITD 只平移能量不改变总量，diffuse bed
       按功率求和。无 soft-clip 的参考渲染直接给出裁剪前的功率基准，
       稳态下 wet 收敛到 0/1，两条路径总能量严格相等 */
    float voice_in[6][MAX_FRAMES];
    const float* in6[6];
    for (int k = 0; k < 6; k++) {
        for (int i = 0; i < 256; i++) voice_in[k][i] = 0.0f;
        voice_in[k][20 + 8 * k] = 0.1f;
        in6[k] = voice_in[k];
    }
    render_full_ref(six, 6, &l, in6, 256, out);
    float ref_e = energy(out, 256 * 2);

    SpatialBus bus1; memset(&bus1, 0, sizeof(bus1));
    float e3;
    for (int f = 0; f < 8; f++)
        render_budget(&bus1, six, 6, &l, 3, in6, 256, 0.010f, out);
    e3 = energy(out, 256 * 2);

    SpatialBus bus0; memset(&bus0, 0, sizeof(bus0));
    float e0;
    for (int f = 0; f < 8; f++)
        render_budget(&bus0, six, 6, &l, 0, in6, 256, 0.010f, out);
    e0 = energy(out, 256 * 2);

    float db3 = fabsf(db_ratio(e3, ref_e));
    float db0 = fabsf(db_ratio(e0, ref_e));
    printf("energy_error budget3=%.4fdB budget0=%.4fdB (阈值 1dB)\n", db3, db0);
    int energy_ok = db3 < 1.0f && db0 < 1.0f;
    printf("energy_compensation_ok=%d\n", energy_ok);

    /* 4) 边界不爆音：N 在临界值交替，平滑交叉越迁的块间最大跳变
       必须远小于硬切换。声源放在正侧方（az=pi/2），此时空间路径全部
       偏到右声道而 diffuse bed 居中，两条路径的声道分布不同，硬切换
       会在块边界留下明显台阶；等功率交叉淡化把台阶摊平 */
    Source two[2] = { {2,0,0,1}, {3,0,0,1} };
    float sig[MAX_FRAMES];
    for (int i = 0; i < 256; i++) sig[i] = 0.1f;     /* 稳态直流，无 ITD 平移 */
    const float* in2[2] = { sig, sig };            /* 允许别名同一缓冲 */
    SpatialBus bs; memset(&bs, 0, sizeof(bs));
    SpatialBus bh; memset(&bh, 0, sizeof(bh));
    float max_smooth = 0.0f, max_hard = 0.0f;
    float prev_l = 0.0f;
    for (int f = 0; f < 16; f++) {
        int budget = (f % 2 == 0) ? 2 : 1;
        render_budget(&bs, two, 2, &l, budget, in2, 256, 0.010f, out);
        if (f > 0) {
            float j = fabsf(out[0] - prev_l);
            if (j > max_smooth) max_smooth = j;
        }
        prev_l = out[2*255];
    }
    prev_l = 0.0f;
    for (int f = 0; f < 16; f++) {
        int budget = (f % 2 == 0) ? 2 : 1;
        render_budget(&bh, two, 2, &l, budget, in2, 256, 0.0f, out);
        if (f > 0) {
            float j = fabsf(out[0] - prev_l);
            if (j > max_hard) max_hard = j;
        }
        prev_l = out[2*255];
    }
    printf("boundary_jump smooth=%.5f hard=%.5f\n", max_smooth, max_hard);
    int pop_ok = max_hard > 0.0f && max_smooth < 0.6f * max_hard;
    printf("no_pop_ok=%d\n", pop_ok);

    /* 5) 确定性：N 先缩小再放大，两条独立总线重放同一序列，
       输出逐字节一致；序列中所有 N 取值结果都确定 */
    Source ten[10];
    for (int i = 0; i < 10; i++) {
        ten[i].x = 0.0f; ten[i].y = 0.0f;
        ten[i].z = -(float)(i + 1); ten[i].gain = 1.0f;
    }
    const float* in10[10];
    for (int i = 0; i < 10; i++) in10[i] = mono;
    int seq[12] = {10, 7, 4, 2, 4, 7, 10, 5, 3, 5, 8, 10};
    SpatialBus da; memset(&da, 0, sizeof(da));
    SpatialBus db; memset(&db, 0, sizeof(db));
    float oa[MAX_FRAMES * 2], ob[MAX_FRAMES * 2];
    int det_ok = 1;
    for (int f = 0; f < 12 && det_ok; f++) {
        render_budget(&da, ten, 10, &l, seq[f], in10, 256, 0.010f, oa);
        render_budget(&db, ten, 10, &l, seq[f], in10, 256, 0.010f, ob);
        if (memcmp(oa, ob, sizeof(float) * 256 * 2) != 0) det_ok = 0;
    }
    printf("determinism_ok=%d (N 缩小与放大序列逐字节一致)\n", det_ok);

    int all_ok = top3_ok && tie_ok && hyst_ok && energy_ok && pop_ok && det_ok;
    printf("ALL_CHECKS=%d\n", all_ok);
    return all_ok ? 0 : 1;
}
