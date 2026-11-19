
/* mini-spatial-audio: binaural positioning of mono sources.
   ITD by azimuth (max 0.6 ms), IID by azimuth, inverse-distance attenuation,
   soft-clip mixing of N sources into an interleaved stereo buffer.

   Also renders a 5.1 bus (ITU-R BS.775 speaker ring) and checks the downmix:
     [A] folded energy matches the binaural render within 0.5 dB
     [B] per-channel weights match the closed-form pan law
     [C] none of the six channels clips
     [D] the weight distribution at front/back/left/right/rear-left/rear-right */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SR 48000
#define MAX_FRAMES 4096
#define MAX_SRC 8
#define PI_F 3.14159265358979f

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

/* ---------------- 5.1 bus ---------------- */

enum { CH_L, CH_R, CH_C, CH_LFE, CH_LS, CH_RS, CH_COUNT };

static const char* const CH_NAME[CH_COUNT] = { "L", "R", "C", "LFE", "Ls", "Rs" };

/* 5.0 ring, clockwise seen from above:
   C @ 0, R @ 30, Rs @ 110, Ls @ 250, L @ 330 (degrees, 0 = front).
   Every azimuth falls inside one adjacent pair; those two speakers get
   equal-power weights cos/sin of the in-between fraction, i.e.
   w_a^2 + w_b^2 = 1, so nothing is added or lost by panning. */
static void pan_weights(float az, float* w) {
    static const float spk_deg[5] = { 0.0f, 30.0f, 110.0f, 250.0f, 330.0f };
    static const int   spk_ch[5]  = { CH_C, CH_R, CH_RS, CH_LS, CH_L };

    for (int i = 0; i < CH_COUNT; i++) w[i] = 0.0f;

    float deg = fmodf(az * (180.0f / PI_F), 360.0f);
    if (deg < 0.0f) deg += 360.0f;

    int k = 0;
    for (int i = 1; i < 5; i++) if (deg >= spk_deg[i]) k = i;
    float a0 = spk_deg[k];
    float a1 = (k == 4) ? 360.0f : spk_deg[k + 1];
    float t  = clampf((deg - a0) / (a1 - a0), 0.0f, 1.0f);
    float cw = cosf(t * PI_F * 0.5f);
    float sw = sinf(t * PI_F * 0.5f);

    w[spk_ch[k]]           += cw;
    w[spk_ch[(k + 1) % 5]] += sw;
    for (int i = 0; i < CH_COUNT; i++) if (w[i] < 1e-7f) w[i] = 0.0f;  /* kill -0 */
}

/* Raw ITU-R BS.775 fold of one pan distribution:
   L = FL + 0.707*FC + 0.707*Ls, R = FR + 0.707*FC + 0.707*Rs.
   LFE is not folded into the stereo bus. */
static void fold_itu(const float* w, float* pl, float* pr) {
    const float k = 0.70710678f;             /* -3 dB */
    *pl = w[CH_L] + k * w[CH_C] + k * w[CH_LS];
    *pr = w[CH_R] + k * w[CH_C] + k * w[CH_RS];
}

/* Per-source trim on the folded pair.
   The binaural pair has gl^2 + gr^2 = 1 at every azimuth, so "same loudness
   as the binaural version" means |fold| = 1. Raw BS.775 does not give that
   (front 0 dB, side +0.31 dB, rear -3.0 dB), so the fold is normalised by
   its own magnitude. This is a pure scalar per source: the six-channel
   weights and peaks are untouched, only the stereo fold is levelled. */
static float fold_trim(const float* w) {
    float pl, pr;
    fold_itu(w, &pl, &pr);
    float mag = sqrtf(pl * pl + pr * pr);
    return (mag > 1e-12f) ? 1.0f / mag : 1.0f;
}

/* Render N sources into the 6-channel bus (channel-major) and into the
   stereo downmix at the same time.  normalize=1 applies fold_trim per source,
   normalize=0 leaves the raw BS.775 levels for comparison. */
static void render_51(const Source* srcs, int n, const Listener* l,
                      const float* mono, int frames,
                      float* out6, float* st, int normalize) {
    memset(out6, 0, sizeof(float) * (size_t)frames * CH_COUNT);
    memset(st,   0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < n; k++) {
        float w[CH_COUNT];
        pan_weights(azimuth_of(&srcs[k], l), w);
        float g = srcs[k].gain * distance_gain(&srcs[k]);
        float pl, pr;
        fold_itu(w, &pl, &pr);
        float t = normalize ? fold_trim(w) : 1.0f;
        for (int i = 0; i < frames; i++) {
            float s = mono[i] * g;
            for (int ch = 0; ch < CH_COUNT; ch++)
                out6[(size_t)ch * frames + i] += s * w[ch];
            st[2*i + 0] += s * pl * t;
            st[2*i + 1] += s * pr * t;
        }
    }
}

/* ---------------- harness helpers ---------------- */

static float energy(const float* buf, int n) {
    double e = 0.0;
    for (int i = 0; i < n; i++) e += (double)buf[i] * (double)buf[i];
    return (float)e;
}

static int argmax_abs(const float* buf, int step, int off, int n) {
    int best = 0;
    for (int i = 1; i < n; i++)
        if (fabsf(buf[(size_t)i * step + off]) > fabsf(buf[(size_t)best * step + off]))
            best = i;
    return best;
}

/* One listener-relative probe; expect[] holds the closed-form pan law. */
typedef struct {
    const char* name;
    float x, z;
    float expect[CH_COUNT];
} Probe;

int main(void) {
    int fails = 0;

    float mono[MAX_FRAMES];
    float out[MAX_FRAMES * 2];
    for (int i = 0; i < 256; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;  /* impulse at 64 */

    Listener l = { 0.0f };
    Source right = { 3.0f, 0.0f, 0.0f, 1.0f };
    render(&right, 1, &l, mono, 256, out);
    int best_l = argmax_abs(out, 2, 0, 256);
    int best_r = argmax_abs(out, 2, 1, 256);
    printf("right_source peakL=%d peakR=%d (impulse@64)\n", best_l, best_r);
    if (!(best_r > best_l)) { printf("  FAIL: ITD/IID did not favour right\n"); fails++; }

    Source left = { -3.0f, 0.0f, 0.0f, 1.0f };
    render(&left, 1, &l, mono, 256, out);
    int bl = argmax_abs(out, 2, 0, 256);
    int br = argmax_abs(out, 2, 1, 256);
    printf("left_source peakL=%d peakR=%d (impulse@64)\n", bl, br);
    if (!(bl > br)) { printf("  FAIL: ITD/IID did not favour left\n"); fails++; }

    Source near = { 0.0f, 0.0f, -1.0f, 1.0f };
    Source far  = { 0.0f, 0.0f, -9.0f, 1.0f };
    render(&near, 1, &l, mono, 256, out); float pn = peak(out, 256*2);
    render(&far, 1, &l, mono, 256, out);  float pf = peak(out, 256*2);
    printf("attenuation near=%.4f far=%.4f ratio=%.3f\n", pn, pf, pf / pn);
    if (fabsf(pf / pn - 0.2f) > 0.01f) { printf("  FAIL: distance law drifted\n"); fails++; }

    Source many[4] = { {1,0,0,1}, {-1,0,0,1}, {0,0,-1,1}, {0,0,1,1} };
    float loud[MAX_FRAMES];
    for (int i = 0; i < 256; i++) loud[i] = 0.9f;
    render(many, 4, &l, loud, 256, out);
    printf("mix_peak=%.4f (soft clip 上限 1.0)\n", peak(out, 256*2));
    if (peak(out, 256*2) > 1.0f) { printf("  FAIL: mixer clipped\n"); fails++; }

    /* --- 5.1 downmix: four binaural probes --- */
    printf("\n5.1 downmix (ITU-R BS.775 ring, LFE unused, per-source fold trim)\n");

    const float SQRT_HALF = 0.70710678f;
    const float COS675 = 0.38268343f, SIN675 = 0.92387953f;    /* cos/sin 67.5 deg   */
    const float COS739 = 0.27683551f, SIN739 = 0.96091732f;    /* cos/sin 73.9286    */
    Probe probes[6] = {
        { "front",      0.0f, -3.0f, { 0, 0, 1, 0, 0, 0 } },
        { "back",       0.0f,  3.0f, { 0, 0, 0, 0, SQRT_HALF, SQRT_HALF } },
        { "left",      -3.0f,  0.0f, { COS675, 0, 0, 0, SIN675, 0 } },
        { "right",      3.0f,  0.0f, { 0, COS675, 0, 0, 0, SIN675 } },
        { "rear-left", -3.0f,  3.0f, { 0, 0, 0, 0, SIN739, COS739 } },
        { "rear-right", 3.0f,  3.0f, { 0, 0, 0, 0, COS739, SIN739 } },
    };
    const int NP = 6;

    float bus[MAX_FRAMES * CH_COUNT];
    float st[MAX_FRAMES * 2];
    float st_raw[MAX_FRAMES * 2];
    float w_of[6][CH_COUNT];
    float dev_norm[6], dev_raw[6], trim_db[6];

    printf("\n  [A] 总能量: E(下混) vs E(双耳), 同一源位置\n");
    printf("      %-11s %-9s %-9s %-9s %-9s %-9s\n",
           "probe", "E_bin", "E_raw", "raw dB", "trim dB", "E_norm");
    for (int p = 0; p < NP; p++) {
        Source src = { probes[p].x, 0.0f, probes[p].z, 1.0f };
        render(&src, 1, &l, mono, 256, out);
        float e_bin = energy(out, 256 * 2);

        render_51(&src, 1, &l, mono, 256, bus, st_raw, 0);
        render_51(&src, 1, &l, mono, 256, bus, st, 1);

        float e_raw  = energy(st_raw, 256 * 2);
        float e_norm = energy(st, 256 * 2);
        dev_raw[p]  = 10.0f * log10f(e_raw / e_bin);
        dev_norm[p] = 10.0f * log10f(e_norm / e_bin);

        pan_weights(azimuth_of(&src, &l), w_of[p]);
        trim_db[p] = 20.0f * log10f(fold_trim(w_of[p]));
        printf("      %-11s %-9.6f %-9.6f %+8.3f %+8.3f %-9.6f\n",
               probes[p].name, e_bin, e_raw, dev_raw[p], trim_db[p], e_norm);
        if (fabsf(dev_norm[p]) >= 0.5f) {
            printf("        FAIL: normalised fold deviates >= 0.5 dB\n"); fails++;
        }
    }

    printf("\n  [B] 各声道权重 vs 解析式 (容差 1e-6)\n");
    for (int p = 0; p < NP; p++) {
        float worst = 0.0f; int wch = 0;
        for (int ch = 0; ch < CH_COUNT; ch++) {
            float d = fabsf(w_of[p][ch] - probes[p].expect[ch]);
            if (d > worst) { worst = d; wch = ch; }
        }
        printf("      %-11s 解析式:", probes[p].name);
        int first = 1;
        for (int ch = 0; ch < CH_COUNT; ch++) {
            if (probes[p].expect[ch] == 0.0f) continue;
            printf("%s%s=%.8f", first ? " " : " ", CH_NAME[ch], probes[p].expect[ch]);
            first = 0;
        }
        printf("\n                 实测:  ");
        for (int ch = 0; ch < CH_COUNT; ch++) printf(" %s=%.8f", CH_NAME[ch], w_of[p][ch]);
        printf("\n                 max|err|=%.2e (%s)\n", worst, CH_NAME[wch]);
        if (worst > 1e-6f) { printf("        FAIL: pan law mismatch\n"); fails++; }
    }

    printf("\n  [C] 削顶检查: 满幅 ±1, 源距 0.05 m (g=%.3f)\n",
           distance_gain(&(Source){0.0f, 0.0f, -0.05f, 1.0f}));
    float full[MAX_FRAMES];
    for (int i = 0; i < 256; i++) full[i] = (i & 1) ? -1.0f : 1.0f;
    for (int p = 0; p < NP; p++) {
        Source src = { probes[p].x, 0.0f, probes[p].z, 1.0f };
        src.x *= 0.05f / 3.0f; src.z *= 0.05f / 3.0f;   /* pull in to 0.05 m */
        render_51(&src, 1, &l, full, 256, bus, st, 1);
        printf("      %-11s", probes[p].name);
        float worst = 0.0f;
        for (int ch = 0; ch < CH_COUNT; ch++) {
            float pk = peak(bus + (size_t)ch * 256, 256);
            printf(" %s=%.3f", CH_NAME[ch], pk);
            if (pk > worst) worst = pk;
        }
        printf("  | 6ch max=%.3f  fold max=%.3f", worst, peak(st, 256 * 2));
        if (worst > 1.0f) { printf("  FAIL"); fails++; }
        printf("\n");
    }

    /* Structural bound: unit input and g <= 1, so |channel| = max weight <= 1. */
    float worst_w = 0.0f, worst_az = 0.0f, worst_e = 0.0f;
    for (int i = 0; i < 720; i++) {
        float az = ((float)i * 0.5f - 180.0f) * (PI_F / 180.0f);
        float w[CH_COUNT];
        pan_weights(az, w);
        float mw = 0.0f, ss = 0.0f;
        for (int ch = 0; ch < CH_COUNT; ch++) {
            if (w[ch] > mw) mw = w[ch];
            ss += w[ch] * w[ch];
        }
        if (mw > worst_w) { worst_w = mw; worst_az = az * (180.0f / PI_F); }
        if (fabsf(ss - 1.0f) > worst_e) worst_e = fabsf(ss - 1.0f);

        float pl, pr; fold_itu(w, &pl, &pr);
        float tr = fold_trim(w);
        float e = (pl * tr) * (pl * tr) + (pr * tr) * (pr * tr);
        if (fabsf(e - 1.0f) > worst_e) worst_e = fabsf(e - 1.0f);
    }
    printf("      worst max weight over 1 deg sweep: %.8f @ az=%+.1f deg\n",
           worst_w, worst_az);
    printf("      max |sum(w^2) - 1| and max |E_fold - 1|: %.2e\n", worst_e);
    if (worst_w > 1.0f) { printf("        FAIL: a pan weight exceeds unity\n"); fails++; }
    if (worst_e > 1e-6f) { printf("        FAIL: pan law / fold not energy preserving\n"); fails++; }

    printf("\n  [D] 权重分布 (四个方位 + 左右对称侧后)\n");
    printf("      %-11s %8s %8s %8s %8s %8s %8s | %7s\n",
           "probe", "L", "R", "C", "LFE", "Ls", "Rs", "E_5.1");
    for (int p = 0; p < NP; p++) {
        printf("      %-11s", probes[p].name);
        float e5 = 0.0f;
        for (int ch = 0; ch < CH_COUNT; ch++) {
            printf(" %8.4f", w_of[p][ch]);
            e5 += w_of[p][ch] * w_of[p][ch];
        }
        printf(" | %7.4f\n", e5);
    }
    printf("      per-channel over probes:");
    for (int ch = 0; ch < CH_COUNT; ch++) {
        float mn = w_of[0][ch], mx = w_of[0][ch];
        for (int p = 1; p < NP; p++) {
            if (w_of[p][ch] < mn) mn = w_of[p][ch];
            if (w_of[p][ch] > mx) mx = w_of[p][ch];
        }
        printf(" %s[%.3f..%.3f]", CH_NAME[ch], mn, mx);
    }
    printf("\n");
    {
        float lo = dev_norm[0], hi = dev_norm[0];
        for (int p = 1; p < NP; p++) {
            if (dev_norm[p] < lo) lo = dev_norm[p];
            if (dev_norm[p] > hi) hi = dev_norm[p];
        }
        printf("      energy dev (normalised): %+.4f .. %+.4f dB, spread %.4f dB\n",
               lo, hi, hi - lo);
    }
    {
        float lo = dev_raw[0], hi = dev_raw[0];
        for (int p = 1; p < NP; p++) {
            if (dev_raw[p] < lo) lo = dev_raw[p];
            if (dev_raw[p] > hi) hi = dev_raw[p];
        }
        printf("      energy dev (raw BS.775): %+.4f .. %+.4f dB, spread %.4f dB\n",
               lo, hi, hi - lo);
    }

    printf("\n%s (%d failure%s)\n", fails ? "FAIL" : "PASS", fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
