
/* pipeline.c — four-stage offline export chain for the mixer module.
 *
 *   stage 1  source synthesis        (legacy rate  8000 Hz)
 *   stage 2  resample to mix-input   (legacy rate 44100 Hz)
 *   stage 3  per-bus mix + ducking   (legacy rate 22050 Hz)
 *   stage 4  resample + soft clip + WAV export (48000 Hz)
 *
 * Legacy build (MIXER_UNIFY_RATE unset or 0): every stage runs at its own
 * sample rate and every block resampler restarts its read position at each
 * 2 s block boundary. Non-integer rate ratios then make the first sample of
 * a block jump relative to the last sample of the previous block — one
 * click every two seconds.
 *
 * Unified build (MIXER_UNIFY_RATE=1, or --unify at runtime): all four
 * stages run at PIPELINE_RATE, both resamplers collapse to 1:1 copies and
 * block boundaries carry no phase reset, so the discontinuity disappears.
 *
 * The legacy code path is compiled unchanged by the unified switch; the
 * switch only selects the rate table, therefore a unified-off build is
 * byte-identical to a build without the feature.
 *
 * Usage: pipeline <out_dir> [--unify]
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#ifndef MIXER_UNIFY_RATE
#define MIXER_UNIFY_RATE 0
#endif

#define PIPELINE_RATE 48000
#define DURATION_SEC  8.0
#define BLOCK_SEC     2.0
#define NBLOCKS       ((int)(DURATION_SEC / BLOCK_SEC))
#define N_BUS         3
#define TWO_PI_F      (2.0f * (float)acosf(-1.0f))

/* Legacy per-stage rates: four stages, four different rates. */
static const int legacy_rate[4] = { 8000, 44100, 22050, 48000 };

typedef struct { int bus; float gain; } Source;

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

/* Nearest-neighbour resample of ONE block. The read position is computed
 * relative to the start of the block, so every block call restarts the
 * phase — the exact mechanism behind the 2 s clicks when in/out rates do
 * not divide evenly. Equal rates collapse to a straight memcpy. */
static void resample_block(const float* in, int in_frames,
                           float* out, int out_frames) {
    if (in_frames == out_frames) {
        memcpy(out, in, sizeof(float) * (size_t)in_frames * 2);
        return;
    }
    for (int j = 0; j < out_frames; j++) {
        long idx = (long)((double)j * in_frames / out_frames);
        if (idx >= in_frames) idx = in_frames - 1;
        out[j * 2]     = in[idx * 2];
        out[j * 2 + 1] = in[idx * 2 + 1];
    }
}

static void* xmalloc(size_t n) {
    void* p = malloc(n);
    if (!p) { fprintf(stderr, "out of memory\n"); exit(2); }
    return p;
}

static void write_f32(const char* path, const float* b, size_t frames) {
    FILE* f = fopen(path, "wb");
    if (!f) { perror(path); exit(2); }
    if (fwrite(b, sizeof(float), frames * 2, f) != frames * 2) {
        fprintf(stderr, "short write: %s\n", path); exit(2);
    }
    fclose(f);
}

static void write_wav(const char* path, const float* b, size_t frames, int rate) {
    FILE* f = fopen(path, "wb");
    if (!f) { perror(path); exit(2); }
    uint32_t data_bytes = (uint32_t)(frames * 2 * sizeof(int16_t));
    uint32_t riff_size = 36u + data_bytes;
    uint32_t byte_rate = (uint32_t)(rate * 2 * sizeof(int16_t));
    uint16_t block_align = 2 * sizeof(int16_t);
    uint32_t srate = (uint32_t)rate;

#define PUT32(v) do { uint32_t _x = (v); fwrite(&_x, 4, 1, f); } while (0)
#define PUT16(v) do { uint16_t _x = (v); fwrite(&_x, 2, 1, f); } while (0)
    fwrite("RIFF", 1, 4, f); PUT32(riff_size); fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f); PUT32(16); PUT16(1); PUT16(2);
    PUT32(srate); PUT32(byte_rate); PUT16(block_align); PUT16(16);
    fwrite("data", 1, 4, f); PUT32(data_bytes);
#undef PUT32
#undef PUT16

    for (size_t i = 0; i < frames * 2; i++) {
        float v = soft_clip(b[i]);
        if (v > 1.0f) v = 1.0f;
        if (v < -1.0f) v = -1.0f;
        int16_t s = (int16_t)lrintf(v * 32767.0f);
        fwrite(&s, sizeof(int16_t), 1, f);
    }
    fclose(f);
}

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s <out_dir> [--unify]\n", argv[0]); return 2; }
    const char* out_dir = argv[1];
    int unify = MIXER_UNIFY_RATE;
    for (int i = 2; i < argc; i++)
        if (strcmp(argv[i], "--unify") == 0) unify = 1;

    int rate[4];
    for (int s = 0; s < 4; s++)
        rate[s] = unify ? PIPELINE_RATE : legacy_rate[s];
    size_t frames[4];
    for (int s = 0; s < 4; s++)
        frames[s] = (size_t)(rate[s] * DURATION_SEC);
    int bf[4];
    for (int s = 0; s < 4; s++)
        bf[s] = (int)(rate[s] * BLOCK_SEC);

    /* one stereo stream per logical source */
    float* src1[3];
    for (int q = 0; q < 3; q++)
        src1[q] = xmalloc(frames[0] * 2 * sizeof(float));
    float* src2[3];
    for (int q = 0; q < 3; q++)
        src2[q] = xmalloc(frames[1] * 2 * sizeof(float));
    float* src3[3];
    for (int q = 0; q < 3; q++)
        src3[q] = xmalloc(frames[2] * 2 * sizeof(float));
    float* st3 = xmalloc(frames[2] * 2 * sizeof(float));
    float* st4 = xmalloc(frames[3] * 2 * sizeof(float));

    /* ---- stage 1: synthesis, phase continuous across block boundaries ---- */
    const float sfx_f = 880.0f, mus_f1 = 220.0f, mus_f2 = 440.0f, voc_f = 1000.0f;
    size_t idx = 0;
    for (int blk = 0; blk < NBLOCKS; blk++) {
        for (int n = 0; n < bf[0]; n++, idx++) {
            float t = (float)idx / (float)rate[0];
            float sfx = 0.6f * sinf(TWO_PI_F * sfx_f * t);
            float mus = 0.4f * 0.5f * (sinf(TWO_PI_F * mus_f1 * t)
                                     + sinf(TWO_PI_F * mus_f2 * t));
            float voc = 0.15f * sinf(TWO_PI_F * voc_f * t);
            float m[3] = { sfx, mus, voc };
            for (int q = 0; q < 3; q++) {
                src1[q][idx * 2]     = m[q];
                src1[q][idx * 2 + 1] = 0.8f * m[q];
            }
        }
    }

    /* ---- stage 2: per-source block-wise resample, phase reset per block ---- */
    for (int q = 0; q < 3; q++)
        for (int blk = 0; blk < NBLOCKS; blk++)
            resample_block(src1[q] + (size_t)blk * bf[0] * 2, bf[0],
                           src2[q] + (size_t)blk * bf[1] * 2, bf[1]);

    /* ---- stage 3: downsample to mix rate, bus mix, sidechain duck ---- */
    for (int q = 0; q < 3; q++)
        for (int blk = 0; blk < NBLOCKS; blk++)
            resample_block(src2[q] + (size_t)blk * bf[1] * 2, bf[1],
                           src3[q] + (size_t)blk * bf[2] * 2, bf[2]);

    const Source srcs[3] = { {0, 1.5f}, {1, 1.0f}, {2, 1.0f} };
    const float bus_gain[3] = { 1.0f, 1.0f, 1.0f };
    const float duck_depth = 0.5f;
    for (int blk = 0; blk < NBLOCKS; blk++) {
        float energy = 0.0f;
        for (int n = 0; n < bf[2] * 2; n++)
            energy += fabsf(src3[2][(size_t)blk * bf[2] * 2 + n]);
        energy /= (float)(bf[2] * 2);
        /* voice is continuous, so duck state is constant — block clicks can
         * only originate from the resampler phase resets, not the gate */
        float dg = (energy > 0.01f) ? 1.0f - duck_depth : 1.0f;
        for (int n = 0; n < bf[2]; n++) {
            size_t p = (size_t)(blk * bf[2] + n) * 2;
            for (int ch = 0; ch < 2; ch++) {
                float v = src3[0][p + ch] * srcs[0].gain * bus_gain[0] * dg
                        + src3[1][p + ch] * srcs[1].gain * bus_gain[1]
                        + src3[2][p + ch] * srcs[2].gain * bus_gain[2];
                st3[p + ch] = v;
            }
        }
    }

    /* ---- stage 4: block-wise resample to export rate (phase reset) ---- */
    for (int blk = 0; blk < NBLOCKS; blk++)
        resample_block(st3 + (size_t)blk * bf[2] * 2, bf[2],
                       st4 + (size_t)blk * bf[3] * 2, bf[3]);

    /* ---- dumps ---- */
    const char* stage_name[4] = { "synth", "resample_in", "mix_duck", "export" };
    char path[512];
    /* stage 1/2 dumps carry the sfx source as the representative stream */
    snprintf(path, sizeof(path), "%s/stage1_%s.f32", out_dir, stage_name[0]);
    write_f32(path, src1[0], frames[0]);
    snprintf(path, sizeof(path), "%s/stage2_%s.f32", out_dir, stage_name[1]);
    write_f32(path, src2[0], frames[1]);
    snprintf(path, sizeof(path), "%s/stage3_%s.f32", out_dir, stage_name[2]);
    write_f32(path, st3, frames[2]);
    snprintf(path, sizeof(path), "%s/stage4_%s.f32", out_dir, stage_name[3]);
    write_f32(path, st4, frames[3]);
    char wav_path[512];
    snprintf(wav_path, sizeof(wav_path), "%s/export_master.wav", out_dir);
    write_wav(wav_path, st4, frames[3], rate[3]);

    snprintf(path, sizeof(path), "%s/meta.txt", out_dir);
    FILE* m = fopen(path, "w");
    if (!m) { perror(path); return 2; }
    fprintf(m, "mode=%s\n", unify ? "unified" : "legacy");
    fprintf(m, "duration_sec=%.1f\nblock_sec=%.1f\nnblocks=%d\nstage_count=4\n",
            DURATION_SEC, BLOCK_SEC, NBLOCKS);
    for (int s = 0; s < 4; s++)
        fprintf(m, "stage%d_name=%s\nstage%d_rate=%d\nstage%d_frames=%zu\n",
                s + 1, stage_name[s], s + 1, rate[s], s + 1, frames[s]);
    fclose(m);

    printf("mode=%s rates=[%d,%d,%d,%d] wav=%s\n",
           unify ? "unified" : "legacy", rate[0], rate[1], rate[2], rate[3], wav_path);

    for (int q = 0; q < 3; q++) { free(src1[q]); free(src2[q]); free(src3[q]); }
    free(st3); free(st4);
    return 0;
}
