
/* mini-spatial-audio: binaural positioning of mono sources.
   ITD by azimuth (max 0.6 ms), IID by azimuth, inverse-distance attenuation,
   soft-clip mixing of N sources into an interleaved stereo buffer. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "diag.h"
#include "canvas.h"

#define MOD "spatial"

#define SR 48000
#define MAX_FRAMES 4096
#define MAX_SRC 8

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

static int render(const Source* srcs, int n, const Listener* l,
                  const float* mono, int frames, float* out /* interleaved */) {
    if (!srcs || !l || !mono || !out || frames <= 0 || frames > MAX_FRAMES ||
        DIAG_FAULT(MOD, "bad_frame_count"))
        DIAG_FAIL_RETURN(MOD, "bad_frame_count", DIAG_E_INVALID_ARG);
    if (n < 0 || n > MAX_SRC || DIAG_FAULT(MOD, "too_many_sources"))
        DIAG_FAIL_RETURN(MOD, "too_many_sources", DIAG_E_CAPACITY);
    for (int k = 0; k < n; k++) {
        if (!isfinite(srcs[k].gain) || !isfinite(srcs[k].x) ||
            !isfinite(srcs[k].y) || !isfinite(srcs[k].z) ||
            DIAG_FAULT(MOD, "nonfinite_gain"))
            DIAG_FAIL_RETURN(MOD, "nonfinite_gain", DIAG_E_INVALID_ARG);
    }
    for (int i = 0; i < frames; i++)
        if (!isfinite(mono[i]) || DIAG_FAULT(MOD, "nonfinite_sample"))
            DIAG_FAIL_RETURN(MOD, "nonfinite_sample", DIAG_E_INVALID_ARG);

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
    return DIAG_OK;
}

static float peak(const float* buf, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(buf[i]) > m) m = fabsf(buf[i]);
    return m;
}

/* ---------------------------------------------------------------- stage --- */

int spatial_stage(Canvas* c, int y0, int rows) {
    if (!c || rows <= 0 || rows > CANVAS_H || y0 < 0 || y0 + rows > CANVAS_H ||
        DIAG_FAULT(MOD, "output_overflow"))
        DIAG_FAIL_RETURN(MOD, "output_overflow", DIAG_E_CAPACITY);

    static float mono[MAX_FRAMES];
    static float out[MAX_FRAMES * 2];
    memset(mono, 0, sizeof mono);
    for (int i = 0; i < 256; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;  /* impulse at 64 */

    Listener l = { 0.0f };
    Source pairs[2][2] = {
        { {3.0f, 0.0f, 0.0f, 1.0f}, {-3.0f, 0.0f, 0.0f, 1.0f} },   /* R / L */
        { {0.0f, 0.0f, -1.0f, 1.0f}, {0.0f, 0.0f, -9.0f, 1.0f} },   /* near / far */
    };
    char lines[2][CANVAS_W + 1];
    for (int p = 0; p < 2; p++) {
        int rc = render(&pairs[p][0], 1, &l, mono, 256, out);
        if (rc != DIAG_OK) return rc;
        int pl = 0, pr = 0;
        for (int i = 1; i < 256; i++) {
            if (fabsf(out[2*i+0]) > fabsf(out[2*pl+0])) pl = i;
            if (fabsf(out[2*i+1]) > fabsf(out[2*pr+1])) pr = i;
        }
        if (p == 0)
            snprintf(lines[p], sizeof lines[p], "peak at 0dB: L src peakR@%d  R src peakL@%d", pr, pl);
        else {
            float pn = peak(out, 256 * 2);
            rc = render(&pairs[p][1], 1, &l, mono, 256, out);
            if (rc != DIAG_OK) return rc;
            float pf = peak(out, 256 * 2);
            snprintf(lines[p], sizeof lines[p], "attenuation near/far = %.3f", pf / pn);
        }
    }

    static float loud[MAX_FRAMES];
    for (int i = 0; i < 256; i++) loud[i] = 0.9f;
    Source many[4] = { {1,0,0,1}, {-1,0,0,1}, {0,0,-1,1}, {0,0,1,1} };
    int rc = render(many, 4, &l, loud, 256, out);
    if (rc != DIAG_OK) return rc;
    float mix_peak = peak(out, 256 * 2);
    if (mix_peak > 1.0f + 1e-6f || DIAG_FAULT(MOD, "clip_violation"))
        DIAG_FAIL_RETURN(MOD, "clip_violation", DIAG_E_INVARIANT);

    Canvas cv;
    canvas_init(&cv);
    canvas_text(&cv, 1, 0, "spatial audio (binaural)");
    canvas_text(&cv, 1, 2, lines[0]);
    canvas_text(&cv, 1, 3, lines[1]);
    char tail[CANVAS_W + 1];
    snprintf(tail, sizeof tail, "mix_peak=%.4f  soft-clip<=1.0", mix_peak);
    canvas_text(&cv, 1, 4, tail);

    canvas_blit_band(c, &cv, y0, rows);
    return DIAG_OK;
}

/* ----------------------------------------------------------------- demo --- */
#ifndef SPATIAL_NO_MAIN
int main(void) {
    diag_boot();
    float mono[MAX_FRAMES];
    static float out[MAX_FRAMES * 2];
    for (int i = 0; i < MAX_FRAMES; i++) mono[i] = 0.0f;
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
    diag_flush(stderr);
    return 0;
}
#endif
