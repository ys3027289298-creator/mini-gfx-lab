
/* mini-spatial-audio: binaural positioning of mono sources.
   ITD by azimuth (max 0.6 ms, differential per ear), IID by azimuth,
   inverse-distance attenuation, soft-clip mixing of N sources into an
   interleaved stereo buffer.
   Azimuth continuity: per-source tracker unwraps the raw atan2 azimuth
   (shortest-path delta, no jump across +/-pi) and ramps the smoothed
   value to the target within one block, so smoothing lag <= 1 frame.
   Occlusion switches crossfade (~10 ms) instead of jumping. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SR 48000
#define MAX_FRAMES 4096
#define MAX_SRC 8
#define PI_F 3.14159265358979323846f
#define TWO_PI_F (2.0f * PI_F)

typedef struct {
    float x, y, z;      /* world position, meters; listener at origin facing -z */
    float gain;
    int   occluded;     /* 1 = behind an obstacle: muffled + quieter */
} Source;

typedef struct {
    float yaw;          /* radians, 0 = facing -z */
} Listener;

/* --- azimuth continuity tracker (one per source, persists across blocks) --- */
typedef struct {
    float prev_raw;     /* last raw azimuth, wrapped to [-pi, pi] */
    float target;       /* unwrapped continuous azimuth, end-of-block target */
    float value;        /* smoothed azimuth at end of previous block */
    int   started;
} AzTracker;

typedef struct {
    AzTracker az;
    float occ;          /* smoothed occlusion amount 0..1 */
    float lp_l, lp_r;   /* occlusion lowpass state, per ear */
} SourceState;

static float wrap_pi(float a) {
    while (a >  PI_F) a -= TWO_PI_F;
    while (a < -PI_F) a += TWO_PI_F;
    return a;
}

static void az_tracker_reset(AzTracker* t) { memset(t, 0, sizeof *t); }

static void src_state_reset(SourceState* s) { memset(s, 0, sizeof *s); }

/* Feed one raw (wrapped) azimuth for this frame; returns the unwrapped
   end-of-block target. The delta is taken along the shortest arc, so a
   source crossing the +/-pi boundary moves by ~0, not by ~2*pi. */
static float az_tracker_push(AzTracker* t, float raw) {
    if (!t->started) {
        t->started = 1;
        t->prev_raw = raw;
        t->target = raw;
        t->value = raw;
        return t->target;
    }
    float d = wrap_pi(raw - t->prev_raw);
    t->prev_raw = raw;
    t->target += d;
    return t->target;
}

/* azimuth in radians, 0 = front, +pi/2 = right */
static float azimuth_of(const Source* s, const Listener* l) {
    float dx = s->x, dz = s->z;
    float c = cosf(l->yaw), sn = sinf(l->yaw);
    float xr =  c * dx - sn * dz;
    float zr =  sn * dx + c * dz;
    return atan2f(xr, -zr);
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

/* linear-interpolated read of mono at fractional position pos */
static float read_frac(const float* mono, int frames, float pos) {
    int j = (int)floorf(pos);
    float f = pos - (float)j;
    float s = 0.0f;
    if (j     >= 0 && j     < frames) s += mono[j] * (1.0f - f);
    if (j + 1 >= 0 && j + 1 < frames) s += mono[j + 1] * f;
    return s;
}

static void render(const Source* srcs, SourceState* st, int n, const Listener* l,
                   const float* mono, int frames, float* out /* interleaved */) {
    const float max_delay = 0.0006f * SR;    /* 0.6 ms, Woodworth-lite */
    const float occ_step = 1.0f / (0.010f * SR); /* occlusion ramp: 10 ms */
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int k = 0; k < n; k++) {
        float az1 = az_tracker_push(&st[k].az, azimuth_of(&srcs[k], l));
        float az0 = st[k].az.value;          /* ramp start = last block's end */
        float occ_target = srcs[k].occluded ? 1.0f : 0.0f;
        float occ = st[k].occ;
        float g_base = srcs[k].gain * distance_gain(&srcs[k]);
        float lp_l = st[k].lp_l, lp_r = st[k].lp_r;
        for (int i = 0; i < frames; i++) {
            /* one-block ramp: reaches the target exactly at block end,
               so the smoothing lag never exceeds one frame */
            float t = (float)(i + 1) / (float)frames;
            float az = az0 + (az1 - az0) * t;
            /* occlusion crossfade */
            if (occ < occ_target) occ = fminf(occ + occ_step, occ_target);
            else if (occ > occ_target) occ = fmaxf(occ - occ_step, occ_target);
            float gl, gr; iid_gains(az, &gl, &gr);
            float g = g_base * (1.0f - 0.7f * occ);
            /* differential ITD: ipsilateral ear leads */
            float d = sinf(az) * max_delay * 0.5f;
            float s_l = read_frac(mono, frames, (float)i - d);
            float s_r = read_frac(mono, frames, (float)i + d);
            /* occlusion lowpass: 18 kHz open -> 1.2 kHz fully occluded */
            float cutoff = 18000.0f + (1200.0f - 18000.0f) * occ;
            float a = 1.0f - expf(-TWO_PI_F * cutoff / SR);
            lp_l += a * (s_l * g * gl - lp_l);
            lp_r += a * (s_r * g * gr - lp_r);
            out[2*i + 0] += lp_l;
            out[2*i + 1] += lp_r;
        }
        st[k].az.value = az1;
        st[k].occ = occ;
        st[k].lp_l = lp_l;
        st[k].lp_r = lp_r;
    }
    for (int i = 0; i < frames * 2; i++) out[i] = soft_clip(out[i]);
}

static float peak(const float* buf, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(buf[i]) > m) m = fabsf(buf[i]);
    return m;
}

static int fails = 0;
#define CHECK(cond, ...) do { \
    if (!(cond)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } \
} while (0)

int main(void) {
    float mono[MAX_FRAMES];
    float out[MAX_FRAMES * 2];
    SourceState states[MAX_SRC];
    for (int i = 0; i < 256; i++) mono[i] = (i == 64) ? 1.0f : 0.0f;  /* impulse at 64 */

    Listener l = { 0.0f };
    /* 45-degree sources: both ears get signal, ITD shows as a peak offset */
    Source right = { 3.0f, 0.0f, -3.0f, 1.0f, 0 };
    src_state_reset(&states[0]);
    render(&right, &states[0], 1, &l, mono, 256, out);
    int best_l = 0, best_r = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*best_l+0])) best_l = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*best_r+1])) best_r = i;
    }
    printf("right_source peakL=%d peakR=%d (impulse@64, ITD 领先耳=%s)\n",
           best_l, best_r, best_r < best_l ? "R" : "L");
    CHECK(best_r < best_l, "right source should reach the right ear first");

    Source left = { -3.0f, 0.0f, -3.0f, 1.0f, 0 };
    src_state_reset(&states[0]);
    render(&left, &states[0], 1, &l, mono, 256, out);
    int bl = 0, br = 0;
    for (int i = 1; i < 256; i++) {
        if (fabsf(out[2*i+0]) > fabsf(out[2*bl+0])) bl = i;
        if (fabsf(out[2*i+1]) > fabsf(out[2*br+1])) br = i;
    }
    printf("left_source peakL=%d peakR=%d (impulse@64)\n", bl, br);
    CHECK(bl < br, "left source should reach the left ear first");

    Source near_s = { 0.0f, 0.0f, -1.0f, 1.0f, 0 };
    Source far_s  = { 0.0f, 0.0f, -9.0f, 1.0f, 0 };
    src_state_reset(&states[0]);
    render(&near_s, &states[0], 1, &l, mono, 256, out); float pn = peak(out, 256*2);
    src_state_reset(&states[0]);
    render(&far_s, &states[0], 1, &l, mono, 256, out);  float pf = peak(out, 256*2);
    printf("attenuation near=%.4f far=%.4f ratio=%.3f\n", pn, pf, pf / pn);

    Source many[4] = { {1,0,0,1,0}, {-1,0,0,1,0}, {0,0,-1,1,0}, {0,0,1,1,0} };
    for (int k = 0; k < 4; k++) src_state_reset(&states[k]);
    float loud[MAX_FRAMES];
    for (int i = 0; i < 256; i++) loud[i] = 0.9f;
    render(many, states, 4, &l, loud, 256, out);
    printf("mix_peak=%.4f (soft clip 上限 1.0)\n", peak(out, 256*2));

    /* --- azimuth continuity: noisy source orbiting the head --- */
    {
        AzTracker tr; az_tracker_reset(&tr);
        const int blocks = 4000;
        const float step = 0.023f;           /* rad/frame, ~14.6 revolutions */
        unsigned rng = 12345;
        float prev_raw = 0.0f, prev_sm = 0.0f;
        int jumps_raw = 0, jumps_sm = 0;
        float max_motion = 0.0f, max_mid_lag = 0.0f;
        for (int b = 0; b < blocks; b++) {
            float true_az = -PI_F + fmodf((float)b * step, TWO_PI_F);
            rng = rng * 1664525u + 1013904223u;
            float jitter = ((float)(rng >> 8) / 8388608.0f - 1.0f) * 0.15f;
            float raw = wrap_pi(true_az + jitter);   /* noisy per-frame estimate */
            float sm = az_tracker_push(&tr, raw);
            if (b > 0) {
                if (fabsf(raw - prev_raw) > 1.0f) jumps_raw++;
                if (fabsf(sm - prev_sm) > 1.0f) jumps_sm++;
                float motion = fabsf(sm - prev_sm);
                if (motion > max_motion) max_motion = motion;
                /* mid-block the ramp sits halfway to the target: lag = motion/2 */
                float mid_lag = 0.5f * motion;
                if (mid_lag > max_mid_lag) max_mid_lag = mid_lag;
            }
            prev_raw = raw; prev_sm = sm;
        }
        float expected = -PI_F + (float)blocks * step;   /* unwrapped true azimuth */
        /* azimuth is circular: compare mod 2pi (the tracker's anchor may sit
           one revolution off when the first noisy sample wraps) */
        float drift = fabsf(wrap_pi(prev_sm - expected));
        printf("orbit: jumps raw=%d tracked=%d, unwrap drift=%.3f rad, "
               "max lag=%.2f frame (limit 1)\n",
               jumps_raw, jumps_sm, drift,
               max_motion > 0.0f ? max_mid_lag / max_motion : 0.0f);
        CHECK(jumps_raw >= 10, "baseline should actually jump (%d)", jumps_raw);
        CHECK(jumps_sm * 10 <= jumps_raw,
              "tracked jumps (%d) not an order of magnitude below raw (%d)",
              jumps_sm, jumps_raw);
        CHECK(drift <= 0.2f, "unwrapped azimuth drifted %.3f rad", drift);
        CHECK(max_mid_lag <= max_motion + 1e-6f,
              "smoothing lag exceeds one frame of motion");
    }

    /* --- occlusion switch: crossfade, not a click --- */
    {
        float sine[MAX_FRAMES];
        for (int i = 0; i < 512; i++)
            sine[i] = 0.5f * sinf(TWO_PI_F * 440.0f * (float)i / SR);
        Source src = { 0.0f, 0.0f, -3.0f, 1.0f, 0 };

        /* smoothed switch */
        SourceState st; src_state_reset(&st);
        render(&src, &st, 1, &l, sine, 256, out);        /* settle, unoccluded */
        render(&src, &st, 1, &l, sine, 256, out);        /* steady-state block */
        float steady_maxd = 0.0f;
        for (int i = 1; i < 256; i++) {
            float d = fabsf(out[2*i] - out[2*(i-1)]);
            if (d > steady_maxd) steady_maxd = d;
        }
        src.occluded = 1;
        render(&src, &st, 1, &l, sine, 256, out);        /* switch block */
        float smooth_maxd = 0.0f;
        for (int i = 1; i < 256; i++) {
            float d = fabsf(out[2*i] - out[2*(i-1)]);
            if (d > smooth_maxd) smooth_maxd = d;
        }
        float occ_mid = st.occ;
        render(&src, &st, 1, &l, sine, 256, out);
        float occ_end = st.occ;

        /* hard switch reference: occlusion applied instantly */
        SourceState sh; src_state_reset(&sh);
        src.occluded = 0;
        render(&src, &sh, 1, &l, sine, 256, out);
        src.occluded = 1;
        sh.occ = 1.0f;                                    /* skip the ramp */
        render(&src, &sh, 1, &l, sine, 256, out);
        float hard_maxd = 0.0f;
        for (int i = 1; i < 256; i++) {
            float d = fabsf(out[2*i] - out[2*(i-1)]);
            if (d > hard_maxd) hard_maxd = d;
        }

        printf("occlusion: occ after 1 block=%.2f, after 2=%.2f, max sample step "
               "steady=%.4f smooth=%.4f hard=%.4f\n",
               occ_mid, occ_end, steady_maxd, smooth_maxd, hard_maxd);
        CHECK(occ_mid > 0.2f && occ_mid < 0.9f,
              "occlusion should ramp, not switch (occ=%.2f)", occ_mid);
        CHECK(occ_end == 1.0f, "occlusion ramp should finish within 2 blocks");
        CHECK(smooth_maxd < steady_maxd * 1.5f,
              "smoothed switch clicks (%.4f vs steady %.4f)", smooth_maxd, steady_maxd);
        CHECK(hard_maxd > smooth_maxd * 1.5f,
              "hard switch should be visibly worse than smoothed (%.4f vs %.4f)",
              hard_maxd, smooth_maxd);
    }

    if (fails) printf("FAILED (%d)\n", fails);
    else       printf("ALL OK\n");
    return fails != 0;
}
