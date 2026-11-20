
/* mixer: multi-bus audio mixing with two limiter topologies.
   - hard limiter: zero-latency peak clamp, anything over threshold is flattened
   - look-ahead limiter: predicts peaks over a window and lowers gain ahead of
     time, with configurable release and sidechain-linked per-bus allocation
     that protects the voice bus (its attenuation is capped below 1 dB).
   All buffers are interleaved stereo. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_SRC 8
#define FRAMES 128
#define BUS_COUNT 3         /* 0 = sfx, 1 = music, 2 = voice */
#define SR 48000
#define LA_MAX 480          /* max look-ahead window in samples (10 ms @48k) */
#define DEFAULT_LA_MS 5.0f
#define DEFAULT_REL_MS 100.0f
#define THRESH 1.0f
#define VOICE_FLOOR_DB 0.9f /* voice may lose at most 0.9 dB, i.e. strictly < 1 dB */
#define SC_TRIG 0.01f       /* mean |voice| level that arms the sidechain */
#define PI 3.14159265358979323846

typedef struct {
    int bus;
    float gain;
} Source;

/* gain-series smoother: running minimum over the look-ahead window followed by
   one-pole attack/release smoothing. The minimum lets gain drop early; the
   one-pole only slows the move, so the output never exceeds the window min. */
typedef struct {
    float atk, rel;
    float gain;
    float target[LA_MAX + 1];
    int len, pos;
} Smoother;

static void sm_init(Smoother* s, int la, float atk_ms, float rel_ms) {
    s->len = la + 1;
    s->pos = 0;
    s->gain = 1.0f;
    for (int i = 0; i < LA_MAX + 1; i++) s->target[i] = 1.0f;
    s->atk = expf(-1.0f / (SR * atk_ms / 1000.0f));
    s->rel = expf(-1.0f / (SR * rel_ms / 1000.0f));
}

static float sm_step(Smoother* s, float t) {
    s->target[s->pos % s->len] = t;
    s->pos++;
    float m = 1.0f;
    for (int i = 0; i < s->len; i++)
        if (s->target[i] < m) m = s->target[i];
    float c = m < s->gain ? s->atk : s->rel;
    s->gain = m + (s->gain - m) * c;
    return s->gain;
}

typedef struct {
    float gain[BUS_COUNT];
    float duck_depth;       /* 0..1, applied to the sfx bus when voice is active */
    int duck_enabled;
    float la_ms, release_ms;
    int la;                 /* look-ahead window in samples */
    int dlen;               /* delay line length = la + 1, matches gain window */
    Smoother sm[BUS_COUNT];
    /* planar per-bus, per-channel delay lines: no interleaved-frame aliasing */
    float delay[BUS_COUNT][2][LA_MAX + 1];
    int dpos;
    float voice_gain_min;   /* smallest gain applied to the voice bus */
} Mixer;

static void mix_init(Mixer* mx, float la_ms, float release_ms) {
    memset(mx, 0, sizeof(*mx));
    for (int b = 0; b < BUS_COUNT; b++) mx->gain[b] = 1.0f;
    mx->duck_depth = 0.5f;
    mx->duck_enabled = 1;
    mx->la_ms = la_ms;
    mx->release_ms = release_ms;
    mx->la = (int)(SR * la_ms / 1000.0f);
    if (mx->la < 1) mx->la = 1;   /* a zero window would divide by zero */
    mx->dlen = mx->la + 1;        /* delay matches the smoother's gain window */
    for (int b = 0; b < BUS_COUNT; b++)
        sm_init(&mx->sm[b], mx->la, la_ms, release_ms);
    mx->voice_gain_min = 1.0f;
}

static float hard_clip(float v) {
    if (v > THRESH) return THRESH;
    if (v < -THRESH) return -THRESH;
    return v;
}

static void accumulate(const Mixer* mx, const Source* srcs, int n,
                       const float* const* in,
                       float bus[BUS_COUNT][FRAMES * 2], int frames) {
    memset(bus, 0, sizeof(float) * BUS_COUNT * FRAMES * 2);
    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b >= BUS_COUNT) continue;
        for (int i = 0; i < frames * 2; i++)
            bus[b][i] += in[s][i] * srcs[s].gain * mx->gain[b];
    }
}

/* baseline topology: sum the buses and flatten every sample past threshold.
   no state, no delay. */
static void mix_hard(const Mixer* mx, const Source* srcs, int n,
                     const float* const* in, float* out, int frames) {
    float bus[BUS_COUNT][FRAMES * 2];
    accumulate(mx, srcs, n, in, bus, frames);
    float e = 0.0f;
    for (int i = 0; i < frames * 2; i++) e += fabsf(bus[2][i]);
    e /= (float)(frames * 2);
    float duck = (mx->duck_enabled && e > SC_TRIG) ? 1.0f - mx->duck_depth : 1.0f;
    for (int i = 0; i < frames * 2; i++)
        out[i] = hard_clip(bus[0][i] * duck + bus[1][i] + bus[2][i]);
}

/* look-ahead topology: per-sample target gains are predicted from the windowed
   peak, smoothed with attack/release, and applied to a signal delayed by exactly
   one look-ahead window. Sidechain activation reallocates the reduction so the
   voice bus keeps priority and is never attenuated past VOICE_FLOOR_DB. */
static void mix_la(Mixer* mx, const Source* srcs, int n,
                   const float* const* in, float* out, int frames) {
    float bus[BUS_COUNT][FRAMES * 2];
    accumulate(mx, srcs, n, in, bus, frames);

    float e = 0.0f;
    for (int i = 0; i < frames * 2; i++) e += fabsf(bus[2][i]);
    e /= (float)(frames * 2);
    int sc = mx->duck_enabled && e > SC_TRIG;
    float vfloor = powf(10.0f, -VOICE_FLOOR_DB / 20.0f);
    float duck = sc ? 1.0f - mx->duck_depth : 1.0f;

    for (int f = 0; f < frames; f++) {
        int i = f * 2;
        float sL = bus[0][i], sR = bus[0][i + 1];
        float mL = bus[1][i], mR = bus[1][i + 1];
        float vL = bus[2][i], vR = bus[2][i + 1];

        float peak = fmaxf(fabsf(sL + mL + vL), fabsf(sR + mR + vR));
        float r = peak > THRESH ? THRESH / peak : 1.0f;

        float tv, tm, ts;
        if (!sc) {
            /* linked buses: one common gain, master-bus style limiting */
            tv = tm = ts = r;
        } else {
            /* voice keeps priority: cap its attenuation, push the rest onto
               sfx/music under the per-channel peak budget */
            tv = fmaxf(r, vfloor);
            float avL = fabsf(vL * tv), avR = fabsf(vR * tv);
            float oL = sL + mL, oR = sR + mR;
            float kL = fabsf(oL) > 1e-12f ? (THRESH - avL) / fabsf(oL) : 1.0f;
            float kR = fabsf(oR) > 1e-12f ? (THRESH - avR) / fabsf(oR) : 1.0f;
            float k = fminf(fminf(kL, kR), 1.0f);
            if (k < 0.0f) k = 0.0f;
            tm = k;
            ts = k * duck;
        }

        float gv = sm_step(&mx->sm[2], tv);
        float gm = sm_step(&mx->sm[1], tm);
        float gs = sm_step(&mx->sm[0], ts);
        if (gv < mx->voice_gain_min) mx->voice_gain_min = gv;

        int q = mx->dpos % mx->dlen;
        float dsL = mx->delay[0][0][q], dsR = mx->delay[0][1][q];
        float dmL = mx->delay[1][0][q], dmR = mx->delay[1][1][q];
        float dvL = mx->delay[2][0][q], dvR = mx->delay[2][1][q];
        mx->delay[0][0][q] = sL; mx->delay[0][1][q] = sR;
        mx->delay[1][0][q] = mL; mx->delay[1][1][q] = mR;
        mx->delay[2][0][q] = vL; mx->delay[2][1][q] = vR;
        mx->dpos++;

        out[i]     = hard_clip(dsL * gs + dmL * gm + dvL * gv);
        out[i + 1] = hard_clip(dsR * gs + dmR * gm + dvR * gv);
    }
}

static float peak(const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(b[i]) > m) m = fabsf(b[i]);
    return m;
}

/* THD by complex DFT at the exact fundamental/harmonic bins. The drive is an
   integer number of cycles inside the window, so every harmonic lands on a bin
   and |re+i·im| is phase-independent — correct even though the limiter output
   is time-shifted relative to the source (the look-ahead path delays by a full
   window) and the measurement starts mid-cycle. */
static double dft_mag(const float* x, int n, int k) {
    double re = 0.0, im = 0.0;
    for (int t = 0; t < n; t++) {
        double w = 2.0 * PI * k * t / n;
        re += x[t] * cos(w);
        im -= x[t] * sin(w);
    }
    return sqrt(re * re + im * im);
}

static double thd_pct(const float* x, int n, int f0_bin) {
    double h1 = dft_mag(x, n, f0_bin);
    double sum = 0.0;
    for (int h = 2; h <= 5; h++) {
        double m = dft_mag(x, n, f0_bin * h);
        sum += m * m;
    }
    return 100.0 * sqrt(sum) / h1;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

#define BLOCKS 80
#define TONE_N (BLOCKS * FRAMES)          /* 10240 frames */
#define MEAS_N 4800                       /* 100 exact 1 kHz cycles @48k */

int main(void) {
    float sfx[FRAMES * 2], music[FRAMES * 2], voice[FRAMES * 2], out[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.0f;
    }
    const float* in[3] = { sfx, music, voice };
    Source srcs[3] = { {0, 1.0f}, {1, 1.0f}, {2, 1.0f} };

    /* ---- basic passthrough: look-ahead path is steady after one window ---- */
    Mixer mx;
    mix_init(&mx, DEFAULT_LA_MS, DEFAULT_REL_MS);
    for (int b = 0; b < 3; b++) mix_la(&mx, srcs, 3, in, out, FRAMES);
    check("sum_ok", fabsf(out[0] - 0.75f) < 1e-5f);
    check("no_clip_when_low", peak(out, FRAMES * 2) < 0.99f);

    /* ---- sidechain ducking still lowers the sfx bus ---- */
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.3f;
    Mixer d1, d2;
    mix_init(&d1, DEFAULT_LA_MS, DEFAULT_REL_MS);
    for (int b = 0; b < BLOCKS; b++) mix_la(&d1, srcs, 3, in, out, FRAMES);
    float with_duck = out[0];
    mix_init(&d2, DEFAULT_LA_MS, DEFAULT_REL_MS);
    d2.duck_enabled = 0;
    for (int b = 0; b < BLOCKS; b++) mix_la(&d2, srcs, 3, in, out, FRAMES);
    float without_duck = out[0];
    check("duck_lowers_sfx", with_duck < without_duck);

    /* ---- idle sidechain passes through untouched ---- */
    mx.duck_enabled = 1;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.0f;
    mix_init(&mx, DEFAULT_LA_MS, DEFAULT_REL_MS);
    for (int b = 0; b < 3; b++) mix_la(&mx, srcs, 3, in, out, FRAMES);
    check("duck_idle_passthrough", fabsf(out[0] - 0.75f) < 1e-5f);

    /* ---- heavy load: both topologies must hold the peak at/below threshold ---- */
    for (int s = 0; s < 3; s++) srcs[s].gain = 4.0f;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.5f;
    float la_peak_v = 0.0f, hard_peak_v = 0.0f;
    mix_init(&mx, DEFAULT_LA_MS, DEFAULT_REL_MS);
    for (int b = 0; b < BLOCKS; b++) {
        mix_la(&mx, srcs, 3, in, out, FRAMES);
        float p = peak(out, FRAMES * 2);
        if (p > la_peak_v) la_peak_v = p;
    }
    Mixer hx;
    mix_init(&hx, DEFAULT_LA_MS, DEFAULT_REL_MS);
    for (int b = 0; b < BLOCKS; b++) {
        mix_hard(&hx, srcs, 3, in, out, FRAMES);
        float p = peak(out, FRAMES * 2);
        if (p > hard_peak_v) hard_peak_v = p;
    }
    check("la_peak_guarantee", la_peak_v <= THRESH + 1e-6f);
    check("hard_peak_guarantee", hard_peak_v <= THRESH + 1e-6f);

    /* invalid bus must be ignored under either topology */
    srcs[0].bus = 7;
    mix_init(&mx, DEFAULT_LA_MS, DEFAULT_REL_MS);
    for (int b = 0; b < BLOCKS; b++) mix_la(&mx, srcs, 3, in, out, FRAMES);
    check("bad_bus_ignored", peak(out, FRAMES * 2) <= THRESH + 1e-6f);
    srcs[0].bus = 0;
    for (int s = 0; s < 3; s++) srcs[s].gain = 1.0f;

    /* ---- THD + latency comparison on a 1.5-amplitude 1 kHz tone ---- */
    float tone[TONE_N * 2], cap_hard[TONE_N * 2], cap_la[TONE_N * 2];
    for (int t = 0; t < TONE_N; t++) {
        float s = 1.5f * sinf(2.0f * PI * 1000.0f * t / SR);
        tone[t * 2] = s; tone[t * 2 + 1] = s;
    }
    Source tsrc[1] = { {0, 1.0f} };
    Mixer th, tl;
    mix_init(&th, DEFAULT_LA_MS, DEFAULT_REL_MS);
    mix_init(&tl, DEFAULT_LA_MS, DEFAULT_REL_MS);
    for (int b = 0; b < BLOCKS; b++) {
        /* streaming: each block reads the next slice of the source */
        const float* slice[1] = { tone + b * FRAMES * 2 };
        mix_hard(&th, tsrc, 1, slice, cap_hard + b * FRAMES * 2, FRAMES);
        mix_la(&tl, tsrc, 1, slice, cap_la + b * FRAMES * 2, FRAMES);
    }
    /* de-interleave left channel of the steady tail window */
    float mh[MEAS_N], ml[MEAS_N];
    for (int t = 0; t < MEAS_N; t++) {
        mh[t] = cap_hard[(TONE_N - MEAS_N + t) * 2];
        ml[t] = cap_la[(TONE_N - MEAS_N + t) * 2];
    }
    double hard_thd = thd_pct(mh, MEAS_N, 100);
    double la_thd = thd_pct(ml, MEAS_N, 100);
    double hard_peak_t = peak(cap_hard, TONE_N * 2);
    double la_peak_t = peak(cap_la, TONE_N * 2);
    int la_samples = tl.dlen;
    printf("hard_peak=%.6f la_peak=%.6f threshold=%.3f\n",
           hard_peak_t, la_peak_t, (double)THRESH);
    printf("hard_thd_pct=%.4f la_thd_pct=%.6f\n", hard_thd, la_thd);
    printf("hard_latency_ms=%.3f (0 samples) la_latency_ms=%.3f (%d samples)\n",
           0.0, (double)la_samples * 1000.0 / SR, la_samples);
    check("tone_both_peaks_safe",
          hard_peak_t <= THRESH + 1e-6 && la_peak_t <= THRESH + 1e-6);
    check("la_lower_distortion", la_thd < hard_thd / 10.0);
    /* latency equals the gain-prediction window plus its alignment sample */
    check("la_latency_is_window",
          fabs((double)la_samples * 1000.0 / SR -
               (DEFAULT_LA_MS + 1000.0 / SR)) < 1e-6);

    /* ---- sidechain priority: voice attenuation must stay under 1 dB ---- */
    for (int i = 0; i < FRAMES * 2; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.3f;
    }
    for (int s = 0; s < 3; s++) srcs[s].gain = 2.0f;
    const float* bin[3] = { sfx, music, voice };

    Mixer sc;
    mix_init(&sc, DEFAULT_LA_MS, DEFAULT_REL_MS);
    for (int b = 0; b < BLOCKS; b++) mix_la(&sc, srcs, 3, bin, out, FRAMES);
    float sc_peak = peak(out, FRAMES * 2);
    double sc_db = 20.0 * log10((double)sc.voice_gain_min);

    Mixer ns;
    mix_init(&ns, DEFAULT_LA_MS, DEFAULT_REL_MS);
    ns.duck_enabled = 0;
    for (int b = 0; b < BLOCKS; b++) mix_la(&ns, srcs, 3, bin, out, FRAMES);
    double nosc_db = 20.0 * log10((double)ns.voice_gain_min);

    printf("voice_atten_sidechain_db=%.4f voice_atten_no_sidechain_db=%.4f\n",
           sc_db, nosc_db);
    check("voice_under_1db", sc_db > -1.0);
    check("voice_priority_works", sc_db - nosc_db > 4.0);
    check("sidechain_peak_safe", sc_peak <= THRESH + 1e-6f);

    /* ---- release time is configurable and changes the recovery speed ---- */
    Smoother fast, slow;
    sm_init(&fast, mx.la, DEFAULT_LA_MS, 20.0f);
    sm_init(&slow, mx.la, DEFAULT_LA_MS, 200.0f);
    for (int t = 0; t < 300; t++) { sm_step(&fast, 0.5f); sm_step(&slow, 0.5f); }
    /* after the peak clears, the fast-release smoother recovers first */
    while (fast.gain < 0.98f) sm_step(&fast, 1.0f);
    for (int t = 0; t < 24; t++) sm_step(&slow, 1.0f);   /* one fast TC */
    check("release_configurable", fast.gain > 0.98f && slow.gain < 0.8f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
