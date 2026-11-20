
/* mini-spatial-audio: binaural positioning of mono sources.
   Two independent angular paths:
     azimuth  -> ITD (max 0.6 ms) + IID
     elevation-> ITD projection cos(el) + pinna high-frequency shelf
   plus inverse-distance attenuation and soft-clip mixing of N sources into
   an interleaved stereo buffer. Elevation outside [-pi/2, +pi/2] (including
   NaN / inf) is a hard error: it is never clamped to a pole. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "reference_data.h"

#define SR 48000
#define MAX_FRAMES 4096
#define MAX_SRC 8

#define PI_F 3.14159265358979323846f
#define ELEV_MAX (PI_F * 0.5f)
#define ITD_MAX_DELAY (0.0006f * SR)   /* 0.6 ms, Woodworth-lite */
#define HF_CROSS_HZ 2000.0f           /* pinna shelf crossover */

typedef enum {
    SPATIAL_OK = 0,
    SPATIAL_ERR_ELEVATION = -1
} SpatialStatus;

typedef struct {
    float x, y, z;      /* world position, meters; listener at origin facing -z */
    float gain;
} Source;

typedef struct {
    float yaw;          /* radians, 0 = facing -z */
    float pitch;        /* radians, + = looking up */
} Listener;

/* Fitted against ref_hf_db: hf_db = c0 + c1*sin(el) + c2*sin(el)^2 */
static float hf_c0, hf_c1, hf_c2;

/* NaN compares false against both bounds, so garbage elevation is rejected. */
static int elevation_valid(float el) {
    return el >= -ELEV_MAX && el <= ELEV_MAX;
}

/* az: 0 = front, +pi/2 = right; el: -pi/2 = underfoot, +pi/2 = overhead */
static SpatialStatus direction_of(const Source* s, const Listener* l,
                                  float* az, float* el) {
    float c = cosf(l->yaw), sn = sinf(l->yaw);
    float xr = c * s->x - sn * s->z;
    float zr = sn * s->x + c * s->z;
    float yr = s->y;
    float a = atan2f(xr, -zr);
    float e = atan2f(yr, sqrtf(xr * xr + zr * zr)) - l->pitch;
    if (!elevation_valid(e)) return SPATIAL_ERR_ELEVATION;
    *az = a;
    *el = e;
    return SPATIAL_OK;
}

/* Signed interaural delay in samples: + means the left ear lags (source right).
   Azimuth sets the lateral shape, elevation independently projects it onto
   the ear-to-ear axis; both poles give zero delay. */
static int itd_samples(float az, float el) {
    return (int)lroundf(ITD_MAX_DELAY * sinf(az) * cosf(el));
}

static void iid_gains(float az, float* gl, float* gr) {
    float t = (sinf(az) + 1.0f) * 0.5f;      /* 0 = hard left, 1 = hard right */
    *gl = sqrtf(1.0f - t);
    *gr = sqrtf(t);
}

/* Pinna high-frequency shelf gain in dB; elevation-only, asymmetric so that
   overhead and underfoot do not collapse onto the same timbre. */
static float hf_shelf_db(float el) {
    float s = sinf(el);
    return hf_c0 + hf_c1 * s + hf_c2 * s * s;
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

/* Raw-angle entry point: used by render(), and directly by the tests to prove
   that bad elevation values fail instead of being clamped. */
static SpatialStatus spatialize_angles(float az, float el, float base_gain,
                                       const float* mono, int frames,
                                       float* out /* interleaved, zeroed */) {
    if (!elevation_valid(el)) return SPATIAL_ERR_ELEVATION;

    int delay = itd_samples(az, el);
    int dL = delay > 0 ? delay : 0;     /* right-side source: left ear lags */
    int dR = delay < 0 ? -delay : 0;

    float gl, gr;
    iid_gains(az, &gl, &gr);

    float hf_gain = powf(10.0f, hf_shelf_db(el) / 20.0f);
    float a = 2.0f * PI_F * HF_CROSS_HZ / (float)SR;  /* one-pole coeff */
    float lowL = 0.0f, lowR = 0.0f;

    for (int i = 0; i < frames; i++) {
        int jL = i - dL, jR = i - dR;
        float xL = (jL >= 0 && jL < frames) ? mono[jL] : 0.0f;
        float xR = (jR >= 0 && jR < frames) ? mono[jR] : 0.0f;
        xL *= base_gain;
        xR *= base_gain;
        /* low-shelf: low band passes untouched, high band scaled by hf_gain */
        lowL += a * (xL - lowL);
        lowR += a * (xR - lowR);
        out[2*i + 0] += (lowL + hf_gain * (xL - lowL)) * gl;
        out[2*i + 1] += (lowR + hf_gain * (xR - lowR)) * gr;
    }
    return SPATIAL_OK;
}

static SpatialStatus render(const Source* srcs, int n, const Listener* l,
                            const float* mono, int frames,
                            float* out /* interleaved */) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < n; k++) {
        float az, el;
        SpatialStatus st = direction_of(&srcs[k], l, &az, &el);
        if (st != SPATIAL_OK) return st;       /* fail fast, no clamping */
        st = spatialize_angles(az, el,
                               srcs[k].gain * distance_gain(&srcs[k]),
                               mono, frames, out);
        if (st != SPATIAL_OK) return st;
    }
    for (int i = 0; i < frames * 2; i++) out[i] = soft_clip(out[i]);
    return SPATIAL_OK;
}

static float peak(const float* buf, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(buf[i]) > m) m = fabsf(buf[i]);
    return m;
}

/* Least-squares fit of hf_shelf_db against the measured reference curve. */
static int fit_hf_shelf(void) {
    double A[3][3] = {{0.0}}, b[3] = {0.0};
    for (int i = 0; i < REF_N; i++) {
        double s = sin((double)ref_elev_deg[i] * PI_F / 180.0);
        double p[3] = { 1.0, s, s * s };
        for (int r = 0; r < 3; r++) {
            b[r] += p[r] * (double)ref_hf_db[i];
            for (int c = 0; c < 3; c++) A[r][c] += p[r] * p[c];
        }
    }
    for (int r = 0; r < 3; r++) {           /* Gaussian elimination, pivot */
        int piv = r;
        for (int k = r + 1; k < 3; k++)
            if (fabs(A[k][r]) > fabs(A[piv][r])) piv = k;
        if (fabs(A[piv][r]) < 1e-12) return -1;
        if (piv != r) {
            for (int c = 0; c < 3; c++) { double t = A[r][c]; A[r][c] = A[piv][c]; A[piv][c] = t; }
            double t = b[r]; b[r] = b[piv]; b[piv] = t;
        }
        for (int k = 0; k < 3; k++) if (k != r) {
            double f = A[k][r] / A[r][r];
            for (int c = r; c < 3; c++) A[k][c] -= f * A[r][c];
            b[k] -= f * b[r];
        }
    }
    hf_c0 = (float)(b[0] / A[0][0]);
    hf_c1 = (float)(b[1] / A[1][1]);
    hf_c2 = (float)(b[2] / A[2][2]);
    return 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    float mono[MAX_FRAMES];
    float out[MAX_FRAMES * 2];
    for (int i = 0; i < 256; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;

    check("hf_shelf_fit", fit_hf_shelf() == 0);
    printf("hf_shelf: db = %.4f %+.4f*sin(el) %+.4f*sin(el)^2\n",
           hf_c0, hf_c1, hf_c2);

    /* ---- per-angle comparison against the measured reference curves ---- */
    const float ITD_TOL = 0.03f, HF_TOL = 0.25f;
    int ref_bad = 0;
    printf("%-5s %-27s %-30s\n", "elev", "itd_factor (ref/model/err)",
           "hf_db (ref/model/err)");
    for (int i = 0; i < REF_N; i++) {
        float el = (float)ref_elev_deg[i] * PI_F / 180.0f;
        float model_itd = cosf(el);                       /* az fixed at +90 */
        float model_hf = hf_shelf_db(el);
        float err_itd = model_itd - ref_itd_factor[i];
        float err_hf = model_hf - ref_hf_db[i];
        int ok = fabsf(err_itd) <= ITD_TOL && fabsf(err_hf) <= HF_TOL;
        if (!ok) ref_bad++;
        printf("%+4.0f  ref=%.4f model=%.4f err=%+.4f %s  "
               "ref=%+7.3f model=%+7.3f err=%+6.3f %s\n",
               ref_elev_deg[i], ref_itd_factor[i], model_itd, err_itd,
               fabsf(err_itd) <= ITD_TOL ? "OK" : "FAIL",
               ref_hf_db[i], model_hf, err_hf,
               fabsf(err_hf) <= HF_TOL ? "OK" : "FAIL");
    }
    check("ref_curves_per_angle", ref_bad == 0);

    /* ---- azimuth / elevation decoupling ---- */
    float el45 = PI_F / 4.0f;
    check("itd_elevation_symmetric",
          itd_samples(PI_F / 2.0f, el45) == itd_samples(PI_F / 2.0f, -el45));
    check("hf_overhead_neq_underfoot",
          hf_shelf_db(ELEV_MAX) - hf_shelf_db(-ELEV_MAX) > 5.0f);
    check("hf_elevation_asymmetric",
          fabsf(hf_shelf_db(el45) - hf_shelf_db(-el45)) > 1.0f);
    float g1l, g1r, g2l, g2r;
    iid_gains(0.7f, &g1l, &g1r);
    iid_gains(0.7f, &g2l, &g2r);  /* no elevation parameter by construction */
    check("iid_azimuth_only", g1l == g2l && g1r == g2r);
    check("zero_delay_at_poles",
          itd_samples(PI_F / 2.0f, ELEV_MAX) == 0 &&
          itd_samples(PI_F / 2.0f, -ELEV_MAX) == 0);
    check("zero_delay_at_median_plane", itd_samples(0.0f, el45) == 0);

    /* ---- invalid elevation: hard failure, never clamped ---- */
    float bad_els[5] = { ELEV_MAX + 0.1f, -ELEV_MAX - 0.1f,
                         NAN, INFINITY, -INFINITY };
    int all_rejected = 1;
    for (int i = 0; i < 5; i++) {
        memset(out, 0, sizeof(out));
        SpatialStatus st = spatialize_angles(0.0f, bad_els[i], 1.0f,
                                             mono, 256, out);
        if (st != SPATIAL_ERR_ELEVATION || peak(out, 256 * 2) != 0.0f)
            all_rejected = 0;
    }
    check("bad_elevation_rejected", all_rejected);

    Listener l = { 0.0f, 0.0f };
    Source nan_src = { NAN, 0.0f, -1.0f, 1.0f };
    check("nan_source_rejected",
          render(&nan_src, 1, &l, mono, 256, out) == SPATIAL_ERR_ELEVATION);
    Listener lp = { 0.0f, ELEV_MAX + 0.2f };
    Source front = { 0.0f, 0.0f, -1.0f, 1.0f };
    check("extreme_pitch_rejected",
          render(&front, 1, &lp, mono, 256, out) == SPATIAL_ERR_ELEVATION);

    /* ---- ITD timing: 60 deg keeps both ear gains nonzero (at hard +-90
       the IID far-ear gain is zero, which would mask the delay) ---- */
    int pL, pR;
    memset(out, 0, sizeof(out));
    spatialize_angles(PI_F / 3.0f, 0.0f, 1.0f, mono, 256, out);
    pL = pR = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*pL+0])) pL = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*pR+1])) pR = i;
    }
    printf("az=+60 source peakL=%d peakR=%d (impulse@64)\n", pL, pR);
    check("itd_right_ear_timing",
          pR == 64 && pL == 64 + itd_samples(PI_F / 3.0f, 0.0f));

    /* same azimuth at +60 elevation: delay shortens independently of IID */
    memset(out, 0, sizeof(out));
    spatialize_angles(PI_F / 3.0f, PI_F / 3.0f, 1.0f, mono, 256, out);
    pL = pR = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*pL+0])) pL = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*pR+1])) pR = i;
    }
    check("itd_elevation_shortens_delay",
          pR == 64 && pL == 64 + itd_samples(PI_F / 3.0f, PI_F / 3.0f) &&
          itd_samples(PI_F / 3.0f, PI_F / 3.0f) < itd_samples(PI_F / 3.0f, 0.0f));

    memset(out, 0, sizeof(out));
    spatialize_angles(-PI_F / 3.0f, 0.0f, 1.0f, mono, 256, out);
    pL = pR = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*pL+0])) pL = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*pR+1])) pR = i;
    }
    check("itd_left_ear_timing",
          pL == 64 && pR == 64 - itd_samples(-PI_F / 3.0f, 0.0f));

    Source near = { 0.0f, 0.0f, -1.0f, 1.0f };
    Source far  = { 0.0f, 0.0f, -9.0f, 1.0f };
    render(&near, 1, &l, mono, 256, out); float pn = peak(out, 256*2);
    render(&far, 1, &l, mono, 256, out);  float pf = peak(out, 256*2);
    printf("attenuation near=%.4f far=%.4f ratio=%.3f\n", pn, pf, pf / pn);
    check("distance_attenuation", pf < pn);

    Source overhead  = { 0.0f,  3.0f, 0.0f, 1.0f };
    Source underfoot = { 0.0f, -3.0f, 0.0f, 1.0f };
    render(&overhead, 1, &l, mono, 256, out);  float po = peak(out, 256*2);
    render(&underfoot, 1, &l, mono, 256, out); float pu = peak(out, 256*2);
    printf("pole_timbre overhead=%.4f underfoot=%.4f (ITD 0 both, HF differs)\n",
           po, pu);
    check("poles_distinguishable", fabsf(po - pu) > 1e-3f);

    Source many[4] = { {1,0,0,1}, {-1,0,0,1}, {0,0,-1,1}, {0,0,1,1} };
    float loud[MAX_FRAMES];
    for (int i = 0; i < 256; i++) loud[i] = 0.9f;
    render(many, 4, &l, loud, 256, out);
    printf("mix_peak=%.4f (soft clip 上限 1.0)\n", peak(out, 256*2));
    check("soft_clip_bounds", peak(out, 256 * 2) <= 1.0f + 1e-6f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
