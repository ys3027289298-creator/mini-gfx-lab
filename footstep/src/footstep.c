/* footstep: 脚步声事件调度 + 44.1kHz -> 48kHz 线性重采样。

   现状（align=0）：事件进混音队列后按 480 帧（10ms）块边界派发，
   重采样相位从块起点归零起步 —— 实际发声位置比事件时间晚最多一个块，
   听起来"慢半拍"，重采样相位与事件时间也对不齐。

   修复（align=1）：事件对齐到精确输出采样位置（floor + 小数相位），
   重采样器带上亚采样偏移，落点偏差 <= 1 个采样。

   align=0 路径保持原样，输出与修复前逐字节相同（自检内嵌黄金哈希）。 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SRC_SR 44100
#define OUT_SR 48000
#define BLOCK 480              /* 10ms 音频回调块 */
#define SRC_FRAMES 7938        /* 脚步采样时长 0.18s @44.1k */
#define OUT_FRAMES 57600       /* 渲染 1.2s @48k */
#define N_EV 7                 /* 事件 t = k/7 s，落点覆盖 0..6/7 的小数相位 */

/* align=0 输出的黄金哈希（修复前"现状"的逐字节指纹，见 README 复现步骤） */
#define GOLDEN_OFF_HASH 0xae95f32f8610dacfULL

static float src[SRC_FRAMES];
static float out_off[OUT_FRAMES];
static float out_on[OUT_FRAMES];

typedef struct {
    long n0;          /* 事件实际起始输出采样 */
    double phase0;    /* 重采样起始小数相位（输出采样单位） */
    long written;     /* 该事件写出的输出采样数 */
    double last_st;   /* 最后一个写出采样对应的源采样位置 */
} EvStat;

static EvStat st_off[N_EV], st_on[N_EV];

/* 确定性脚步声：起始瞬态点击 + 90Hz 衰减正弦 + 少量噪声（固定种子） */
static void synth_footstep(void) {
    unsigned rng = 12345u;
    for (int i = 0; i < SRC_FRAMES; i++) {
        rng = rng * 1664525u + 1013904223u;
        float n = ((rng >> 8) & 0xFFFF) / 65535.0f - 0.5f;
        float t = (float)i / SRC_SR;
        float env = expf(-t * 30.0f);
        float v = 0.9f * env * sinf(2.0f * 3.14159265f * 90.0f * t) + 0.25f * env * n;
        if (i == 0) v = 1.0f;      /* 起始点击，作为对齐测量基准 */
        src[i] = v;
    }
}

static void render(const double* ev_t, int n_ev, int align, float* out, EvStat* stats) {
    const double ratio = (double)SRC_SR / OUT_SR;   /* 0.91875 源采样/输出采样 */
    memset(out, 0, sizeof(float) * OUT_FRAMES);
    for (int e = 0; e < n_ev; e++) {
        double pos = ev_t[e] * OUT_SR;              /* 精确落点（输出采样，含小数） */
        long n0;
        double base;
        if (align) {
            n0 = (long)floor(pos);                  /* 精确采样位置 */
            base = pos;                             /* 相位带上亚采样偏移 */
        } else {
            n0 = ((long)(pos / BLOCK) + 1) * BLOCK; /* 现状：派发到下一块边界 */
            base = (double)n0;                      /* 相位从块起点归零 */
        }
        long written = 0;
        double last_st = 0.0;
        for (long n = n0; n < OUT_FRAMES; n++) {
            double st = ((double)n - base) * ratio;
            if (st >= SRC_FRAMES - 1) break;
            float s = 0.0f;
            if (st >= 0.0) {
                long i = (long)st;
                double f = st - (double)i;
                s = (float)(src[i] + (src[i + 1] - src[i]) * f);
            }
            out[n] += s;
            written++;
            last_st = st;
        }
        if (stats) {
            stats[e].n0 = n0;
            stats[e].phase0 = pos - (double)n0;
            stats[e].written = written;
            stats[e].last_st = last_st;
        }
    }
}

/* 起点检测：窗口内第一个超过阈值的采样 */
static long find_onset(const float* out, double pos) {
    long from = (long)pos - 16;
    long to = (long)pos + BLOCK + 64;
    if (from < 0) from = 0;
    if (to > OUT_FRAMES) to = OUT_FRAMES;
    for (long n = from; n < to; n++)
        if (fabsf(out[n]) > 0.05f) return n;
    return -1;
}

static unsigned long long fnv1a64(const void* p, size_t n) {
    const unsigned char* b = (const unsigned char*)p;
    unsigned long long h = 1469598103934665603ULL;
    for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 1099511628211ULL; }
    return h;
}

static double peak_of(const float* b, long* pos) {
    double m = 0.0; long p = 0;
    for (long i = 0; i < OUT_FRAMES; i++)
        if (fabsf(b[i]) > m) { m = fabsf(b[i]); p = i; }
    if (pos) *pos = p;
    return m;
}

static double rms_of(const float* b) {
    double acc = 0.0;
    for (long i = 0; i < OUT_FRAMES; i++) acc += (double)b[i] * b[i];
    return sqrt(acc / OUT_FRAMES);
}

static int write_report(const double* ev_t,
                        const long* onset_off, const double* dev_off,
                        const long* onset_on, const double* dev_on,
                        unsigned long long h_off, unsigned long long h_on) {
    FILE* f;

    f = fopen("out/mix_align_off.f32", "wb");
    if (!f) return -1;
    fwrite(out_off, sizeof(float), OUT_FRAMES, f); fclose(f);
    f = fopen("out/mix_align_on.f32", "wb");
    if (!f) return -1;
    fwrite(out_on, sizeof(float), OUT_FRAMES, f); fclose(f);

    f = fopen("out/waveform_summary.txt", "w");
    if (!f) return -1;
    long pp;
    double pk = peak_of(out_off, &pp);
    fprintf(f, "# 波形摘要（float32 小端，%d Hz，%d 帧 = %.3fs）\n", OUT_SR, OUT_FRAMES,
            (double)OUT_FRAMES / OUT_SR);
    fprintf(f, "mode=off frames=%d peak=%.6f peak_pos=%ld rms=%.6f fnv1a64=%016llx\n",
            OUT_FRAMES, pk, pp, rms_of(out_off), h_off);
    pk = peak_of(out_on, &pp);
    fprintf(f, "mode=on  frames=%d peak=%.6f peak_pos=%ld rms=%.6f fnv1a64=%016llx\n",
            OUT_FRAMES, pk, pp, rms_of(out_on), h_on);
    fprintf(f, "note: mode=off 与修复前现状逐字节相同（黄金哈希 %016llx）\n",
            (unsigned long long)GOLDEN_OFF_HASH);
    fclose(f);

    f = fopen("out/event_deviation_table.csv", "w");
    if (!f) return -1;
    fprintf(f, "event,t_sec,exact_pos,onset_off,dev_off,onset_on,dev_on\n");
    for (int e = 0; e < N_EV; e++)
        fprintf(f, "%d,%.6f,%.4f,%ld,%.4f,%ld,%.4f\n",
                e, ev_t[e], ev_t[e] * OUT_SR,
                onset_off[e], dev_off[e], onset_on[e], dev_on[e]);
    fclose(f);

    f = fopen("out/resample_log.txt", "w");
    if (!f) return -1;
    fprintf(f, "# 重采样记录\n");
    fprintf(f, "src_sr=%d dst_sr=%d ratio=%.6f src_frames=%d dst_frames_per_event_ideal=%.4f\n",
            SRC_SR, OUT_SR, (double)SRC_SR / OUT_SR, SRC_FRAMES,
            (SRC_FRAMES - 1) * (double)OUT_SR / SRC_SR);
    fprintf(f, "mode=off: 事件派发到下一个 480 帧块边界，重采样相位从块起点归零\n");
    fprintf(f, "mode=on : 事件对齐精确采样位置，重采样相位携带亚采样偏移\n");
    fprintf(f, "mode,event,t_sec,exact_pos,n0,phase0,dst_written,src_consumed\n");
    for (int e = 0; e < N_EV; e++)
        fprintf(f, "off,%d,%.6f,%.4f,%ld,%.4f,%ld,%.4f\n",
                e, ev_t[e], ev_t[e] * OUT_SR,
                st_off[e].n0, st_off[e].phase0, st_off[e].written, st_off[e].last_st);
    for (int e = 0; e < N_EV; e++)
        fprintf(f, "on,%d,%.6f,%.4f,%ld,%.4f,%ld,%.4f\n",
                e, ev_t[e], ev_t[e] * OUT_SR,
                st_on[e].n0, st_on[e].phase0, st_on[e].written, st_on[e].last_st);
    fclose(f);
    return 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(int argc, char** argv) {
    double ev_t[N_EV];
    long onset_off[N_EV], onset_on[N_EV];
    double dev_off[N_EV], dev_on[N_EV];

    synth_footstep();
    for (int e = 0; e < N_EV; e++) ev_t[e] = e / 7.0;

    render(ev_t, N_EV, 0, out_off, st_off);
    render(ev_t, N_EV, 1, out_on, st_on);

    for (int e = 0; e < N_EV; e++) {
        double pos = ev_t[e] * OUT_SR;
        onset_off[e] = find_onset(out_off, pos);
        onset_on[e] = find_onset(out_on, pos);
        dev_off[e] = onset_off[e] - pos;
        dev_on[e] = onset_on[e] - pos;
    }

    unsigned long long h_off = fnv1a64(out_off, sizeof(out_off));
    unsigned long long h_on = fnv1a64(out_on, sizeof(out_on));

    if (argc > 1 && strcmp(argv[1], "--report") == 0) {
        if (write_report(ev_t, onset_off, dev_off, onset_on, dev_on, h_off, h_on) != 0) {
            fprintf(stderr, "write_report failed (先 mkdir -p out)\n");
            return 1;
        }
        printf("report written to out/\n");
        return 0;
    }

    double max_dev_on = 0.0, max_dev_off = 0.0;
    for (int e = 0; e < N_EV; e++) {
        if (fabs(dev_on[e]) > max_dev_on) max_dev_on = fabs(dev_on[e]);
        if (fabs(dev_off[e]) > max_dev_off) max_dev_off = fabs(dev_off[e]);
    }
    printf("max_dev_off=%.4f max_dev_on=%.4f (samples)\n", max_dev_off, max_dev_on);
    check("aligned_max_dev_le_1_sample", max_dev_on <= 1.0 + 1e-9);
    check("unaligned_actually_late", max_dev_off > 1.0);
    check("off_mode_byte_identical", h_off == (unsigned long long)GOLDEN_OFF_HASH);
    check("fix_changes_output", h_on != h_off);
    check("resample_len_exact", st_on[0].written == 8639 && st_off[0].written == 8639);
    check("onsets_found", onset_off[N_EV - 1] > 0 && onset_on[N_EV - 1] > 0);
    long pp; double pk = peak_of(out_on, &pp);
    check("peak_sane", pk > 0.5 && pk < 1.2);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
