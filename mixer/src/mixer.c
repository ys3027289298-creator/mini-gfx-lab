/* mixer: multi-bus audio mixing with per-bus gain, sidechain ducking,
   transient shaping and soft clipping. All buffers are interleaved stereo. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_SRC 8
#define FRAMES 128
#define N_BUS 3

typedef struct {
    int bus;        /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
} Source;

/* Transient shaper: a fast envelope tracks attack, a slow envelope tracks
   body/sustain. The positive difference between them splits each sample into
   an attack share (w) and a sustain share (1 - w), which receive independent
   gains. Detection is a one-pole follower with configurable attack/release,
   in peak (|x|) or RMS (x^2) domain. No lookahead: sample n is shaped using
   only samples <= n, so added latency is 0 samples. */
typedef struct {
    int enabled;
    int detector;                 /* 0 = peak, 1 = rms */
    float sample_rate;
    float attack_ms, release_ms;          /* fast envelope */
    float sustain_attack_ms, sustain_release_ms; /* slow envelope */
    float attack_gain, sustain_gain;      /* linear */
} TransientShaper;

typedef struct {
    float gain[N_BUS];
    float duck_depth;    /* 0..1, applied to bus 0 when voice is active */
    int duck_enabled;
    TransientShaper shaper[N_BUS];
    float fast_env[N_BUS];   /* per-bus shaper state, start zeroed */
    float slow_env[N_BUS];
} Mixer;

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

/* one-pole coefficient for a time constant in milliseconds */
static float env_coef(float ms, float sr) {
    if (ms <= 0.0f) return 0.0f;   /* instant */
    return expf(-1.0f / (ms * 0.001f * sr));
}

static float env_step(float env, float target, float coef_att, float coef_rel) {
    float c = target > env ? coef_att : coef_rel;
    return c * env + (1.0f - c) * target;
}

/* In-place stereo shaping. The detector is linked across channels (max of
   |L|, |R|) so the stereo image does not shift when the two channels differ. */
static void transient_process(const TransientShaper* ts, float* fast, float* slow,
                              float* buf, int frames) {
    float sr = ts->sample_rate > 0.0f ? ts->sample_rate : 48000.0f;
    float fatt = env_coef(ts->attack_ms, sr);
    float frel = env_coef(ts->release_ms, sr);
    float satt = env_coef(ts->sustain_attack_ms, sr);
    float srel = env_coef(ts->sustain_release_ms, sr);
    int rms = ts->detector == 1;

    for (int i = 0; i < frames; i++) {
        float l = buf[2 * i], r = buf[2 * i + 1];
        float d = fmaxf(fabsf(l), fabsf(r));
        if (rms) d *= d;                       /* follower works on energy */

        *fast = env_step(*fast, d, fatt, frel);
        *slow = env_step(*slow, d, satt, srel);

        float fv = rms ? sqrtf(*fast) : *fast;
        float sv = rms ? sqrtf(*slow) : *slow;
        float w = (fv - sv) / fmaxf(sv, 1e-6f);
        w = clampf(w, 0.0f, 1.0f);

        float g = ts->sustain_gain + (ts->attack_gain - ts->sustain_gain) * w;
        buf[2 * i] = l * g;
        buf[2 * i + 1] = r * g;
    }
}

static void mix(Mixer* mx, const Source* srcs, int n,
                const float* const* in, float* out, int frames) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    float bus[N_BUS][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b >= N_BUS) continue;
        for (int i = 0; i < frames * 2; i++)
            bus[b][i] += in[s][i] * srcs[s].gain;
    }

    for (int b = 0; b < N_BUS; b++) {
        if (!mx->shaper[b].enabled) continue;
        transient_process(&mx->shaper[b], &mx->fast_env[b], &mx->slow_env[b],
                          bus[b], frames);
    }

    float dg = 1.0f;
    if (mx->duck_enabled) {
        /* voice bus energy drives ducking of the sfx bus */
        float e = 0.0f;
        for (int i = 0; i < frames * 2; i++) e += fabsf(bus[2][i]);
        e /= (float)(frames * 2);
        if (e > 0.01f) dg = 1.0f - mx->duck_depth;
    }

    for (int i = 0; i < frames * 2; i++) {
        float v = bus[0][i] * mx->gain[0] * dg
                + bus[1][i] * mx->gain[1]
                + bus[2][i] * mx->gain[2];
        out[i] = soft_clip(v);
    }
}

static float peak(const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(b[i]) > m) m = fabsf(b[i]);
    return m;
}

static float rms(const float* b, int n) {
    float e = 0.0f;
    for (int i = 0; i < n; i++) e += b[i] * b[i];
    return sqrtf(e / (float)n);
}

/* peak-to-average ratio in dB; rises when transients get punchier */
static float crest_db(const float* b, int n) {
    float r = rms(b, n);
    return r > 0.0f ? 20.0f * log10f(peak(b, n) / r) : 0.0f;
}

/* ---- tests ---------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* reference implementation of the shaper, written independently so the
   mixer's per-sample envelopes and gains can be compared against it */
static void hand_shape(const TransientShaper* ts, const float* in, float* out,
                       int frames, float* fast_out, float* slow_out) {
    float sr = ts->sample_rate;
    float fa = expf(-1.0f / (ts->attack_ms * 0.001f * sr));
    float fr = expf(-1.0f / (ts->release_ms * 0.001f * sr));
    float sa = expf(-1.0f / (ts->sustain_attack_ms * 0.001f * sr));
    float sr_ = expf(-1.0f / (ts->sustain_release_ms * 0.001f * sr));
    int rms = ts->detector == 1;
    float fast = 0.0f, slow = 0.0f;

    for (int i = 0; i < frames; i++) {
        float l = in[2 * i], r = in[2 * i + 1];
        float d = fmaxf(fabsf(l), fabsf(r));
        if (rms) d *= d;

        float fc = d > fast ? fa : fr;
        fast = fc * fast + (1.0f - fc) * d;
        float sc = d > slow ? sa : sr_;
        slow = sc * slow + (1.0f - sc) * d;

        float fv = rms ? sqrtf(fast) : fast;
        float sv = rms ? sqrtf(slow) : slow;
        float w = (fv - sv) / fmaxf(sv, 1e-6f);
        if (w < 0.0f) w = 0.0f;
        if (w > 1.0f) w = 1.0f;
        float g = ts->sustain_gain + (ts->attack_gain - ts->sustain_gain) * w;
        out[2 * i] = l * g;
        out[2 * i + 1] = r * g;
    }
    *fast_out = fast;
    *slow_out = slow;
}

static void shaper_config(TransientShaper* ts, int detector) {
    memset(ts, 0, sizeof(*ts));
    ts->enabled = 1;
    ts->detector = detector;
    ts->sample_rate = 48000.0f;
    ts->attack_ms = 0.1f;
    ts->release_ms = 30.0f;
    ts->sustain_attack_ms = 30.0f;
    ts->sustain_release_ms = 300.0f;
    ts->attack_gain = 2.5f;
    ts->sustain_gain = 0.5f;
}

/* feed one block through mix() with only bus 0 active and unity bus gain, so
   the output is the shaped signal itself (levels stay below soft clip) */
static void run_shaper(Mixer* mx, const float* sfx, float* out, int frames) {
    const float* in[N_BUS] = { sfx, NULL, NULL };
    Source src = { 0, 1.0f };
    mix(mx, &src, 1, in, out, frames);
}

static int env_matches_hand(int detector, const char* name_peak, const char* name_slow) {
    Mixer mx;
    memset(&mx, 0, sizeof(mx));
    for (int b = 0; b < N_BUS; b++) mx.gain[b] = 1.0f;
    shaper_config(&mx.shaper[0], detector);

    const int n = 64;
    float sfx[FRAMES * 2], out[FRAMES * 2], ref[FRAMES * 2];
    for (int i = 0; i < n; i++) {
        /* keep shaped peaks below the 0.9 soft-clip knee so the reference,
           which covers only the shaper, is comparable */
        float v = (i % 17 == 0) ? 0.30f : 0.06f;   /* spikes + body */
        if (i > 40) v *= (float)(n - i) / (float)(n - 40);
        sfx[2 * i] = v;
        sfx[2 * i + 1] = v * 0.5f;                /* differ per channel */
    }

    run_shaper(&mx, sfx, out, n);

    float hf = 0.0f, hs = 0.0f;
    hand_shape(&mx.shaper[0], sfx, ref, n, &hf, &hs);

    float max_err = 0.0f;
    for (int i = 0; i < n * 2; i++) {
        float e = fabsf(out[i] - ref[i]);
        if (e > max_err) max_err = e;
    }
    printf("  detector=%s max_sample_err=%.3g\n", detector ? "rms" : "peak", max_err);
    check(name_peak, max_err < 1e-6f);
    check(name_slow, fabsf(mx.fast_env[0] - hf) < 1e-6f &&
                     fabsf(mx.slow_env[0] - hs) < 1e-6f);
    return mx.fast_env[0] != 0.0f;
}

int main(void) {
    float sfx[FRAMES * 2], music[FRAMES * 2], voice[FRAMES * 2], out[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.0f;
    }
    const float* in[3] = { sfx, music, voice };
    Source srcs[3] = { {0, 1.0f}, {1, 1.0f}, {2, 1.0f} };
    Mixer mx = { .gain = {1.0f, 1.0f, 1.0f}, .duck_depth = 0.5f, .duck_enabled = 1 };

    mix(&mx, srcs, 3, in, out, FRAMES);
    check("sum_ok", fabsf(out[0] - 0.75f) < 1e-5f);
    check("no_clip_when_low", peak(out, FRAMES * 2) < 0.99f);

    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.3f;
    mix(&mx, srcs, 3, in, out, FRAMES);
    float with_duck = out[0];

    mx.duck_enabled = 0;
    mix(&mx, srcs, 3, in, out, FRAMES);
    float without_duck = out[0];
    check("duck_lowers_sfx", with_duck < without_duck);

    mx.duck_enabled = 1;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.0f;
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("duck_idle_passthrough", fabsf(out[0] - 0.75f) < 1e-5f);

    for (int s = 0; s < 3; s++) srcs[s].gain = 4.0f;
    for (int i = 0; i < FRAMES * 2; i++) voice[i] = 0.5f;
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("soft_clip_bounds", peak(out, FRAMES * 2) <= 1.0f + 1e-6f);

    srcs[0].bus = 7;   /* invalid bus must be ignored */
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("bad_bus_ignored", peak(out, FRAMES * 2) <= 1.0f + 1e-6f);

    /* ---- transient shaper ---- */
    env_matches_hand(0, "env_matches_hand_peak", "env_state_matches_hand_peak");
    env_matches_hand(1, "env_matches_hand_rms", "env_state_matches_hand_rms");

    /* latency: the shaped onset must land on the input onset (0 added) */
    {
        Mixer smx;
        memset(&smx, 0, sizeof(smx));
        for (int b = 0; b < N_BUS; b++) smx.gain[b] = 1.0f;
        shaper_config(&smx.shaper[0], 0);
        memset(sfx, 0, sizeof(sfx));
        const int onset = 12;
        sfx[2 * onset] = 0.2f;
        run_shaper(&smx, sfx, out, FRAMES);
        int first = -1;
        for (int i = 0; i < FRAMES; i++)
            if (fabsf(out[2 * i]) > 1e-7f) { first = i; break; }
        int latency = first - onset;
        printf("  onset_in=%d onset_out=%d added_latency=%d\n", onset, first, latency);
        check("latency_le_one_sample", first >= 0 && latency >= 0 && latency <= 1);
    }

    /* crest factor on a drum-like pattern: 4 hits/s, ~60 ms decay tails so
       the fast envelope sees the attack and the slow one sees the body */
    {
        enum { BLOCKS = 375, LONG = FRAMES * BLOCKS };   /* 48000 samples = 1 s */
        static float drums[LONG * 2], shaped_out[LONG * 2], plain[LONG * 2];
        float* shaped = shaped_out;
        float a = 0.0f;
        for (int i = 0; i < LONG; i++) {
            if (i % 12000 == 0) a = 0.30f;
            else a *= 0.999667f;
            drums[2 * i] = a;
            drums[2 * i + 1] = a;
        }

        Mixer smx;
        memset(&smx, 0, sizeof(smx));
        for (int b = 0; b < N_BUS; b++) smx.gain[b] = 1.0f;
        shaper_config(&smx.shaper[0], 0);

        for (int blk = 0; blk < BLOCKS; blk++) {
            run_shaper(&smx, &drums[blk * FRAMES * 2], &shaped[blk * FRAMES * 2], FRAMES);
        }

        Mixer pmx;
        memset(&pmx, 0, sizeof(pmx));
        for (int b = 0; b < N_BUS; b++) pmx.gain[b] = 1.0f;
        for (int blk = 0; blk < BLOCKS; blk++) {
            run_shaper(&pmx, &drums[blk * FRAMES * 2], &plain[blk * FRAMES * 2], FRAMES);
        }

        float cb = crest_db(plain, LONG * 2);
        float ca = crest_db(shaped, LONG * 2);
        printf("  crest_before=%.2f dB crest_after=%.2f dB improvement=%.2f dB\n",
               cb, ca, ca - cb);
        check("shaper_changes_signal", fabsf(ca - cb) > 0.01f);
        check("crest_improves", ca > cb + 1.0f);
        check("shaped_below_clip", peak(shaped, LONG * 2) < 0.9f);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
