
/* mixer: multi-bus audio mixing with per-bus gain, a voice-keyed sidechain
   compressor on the music bus and soft clipping. Buffers are interleaved
   stereo at 48 kHz. The compressor is fully causal: the gain applied to a
   music sample is derived from the voice sample at the same index, so it
   adds zero samples of latency. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_SRC 8
#define FRAMES 128
#define BUSES 3
#define SR 48000.0f
#define KEY_BUS 2          /* voice drives the sidechain */
#define DB_EPS 1e-9f

typedef struct {
    int bus;        /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
} Source;

typedef struct {
    /* configuration */
    float threshold_db;  /* detector level, dBFS (full scale = 1.0), where GR starts */
    float ratio;         /* >= 1, e.g. 4 means 4:1 */
    float attack_ms;
    float release_ms;
    int target_bus;      /* bus being compressed, default music (1) */
    int enabled;

    /* state */
    float env;           /* linear peak envelope of the key (voice) bus */
    float gr_db;         /* gain reduction at the last processed sample, positive dB */
    float gr_max_db;     /* max gain reduction over the last processed block, positive dB */
} Sidechain;

typedef struct {
    float gain[BUSES];
    Sidechain sc;
} Mixer;

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

static void mixer_init(Mixer* mx) {
    for (int b = 0; b < BUSES; b++) mx->gain[b] = 1.0f;
    Sidechain* sc = &mx->sc;
    sc->threshold_db = -24.0f;
    sc->ratio = 4.0f;
    sc->attack_ms = 5.0f;
    sc->release_ms = 120.0f;
    sc->target_bus = 1;         /* music */
    sc->enabled = 1;
    sc->env = 0.0f;
    sc->gr_db = 0.0f;
    sc->gr_max_db = 0.0f;
}

static void sidechain_config(Mixer* mx, float threshold_db, float ratio,
                             float attack_ms, float release_ms) {
    Sidechain* sc = &mx->sc;
    sc->threshold_db = threshold_db;
    sc->ratio = ratio < 1.0f ? 1.0f : ratio;
    sc->attack_ms = attack_ms;
    sc->release_ms = release_ms;
}

static void sidechain_reset(Sidechain* sc) {
    sc->env = 0.0f;
    sc->gr_db = 0.0f;
    sc->gr_max_db = 0.0f;
}

static float mixer_gain_reduction_db(const Mixer* mx) { return mx->sc.gr_db; }
static float mixer_gain_reduction_max_db(const Mixer* mx) { return mx->sc.gr_max_db; }

/* one-pole coefficient for a time constant in ms; 0 ms = instant */
static float env_coef(float ms) {
    if (ms <= 0.0f) return 0.0f;
    return expf(-1.0f / (ms * 0.001f * SR));
}

static float gr_to_gain(float gr_db) {
    return powf(10.0f, -gr_db * 0.05f);
}

/* Static compressor curve (hard knee, dB domain): returns positive gain
   reduction in dB for a detector level above threshold at the given ratio. */
static float static_curve_gr(float level_db, float threshold_db, float ratio) {
    if (ratio < 1.0f) ratio = 1.0f;
    if (level_db <= threshold_db) return 0.0f;
    return (level_db - threshold_db) * (1.0f - 1.0f / ratio);
}

static void mix(Mixer* mx, const Source* srcs, int n,
                const float* const* in, float* out, int frames) {
    float bus[BUSES][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b >= BUSES) continue;
        for (int i = 0; i < frames * 2; i++)
            bus[b][i] += in[s][i] * srcs[s].gain;
    }

    Sidechain* sc = &mx->sc;
    sc->gr_max_db = 0.0f;
    if (sc->enabled) {
        float a_att = env_coef(sc->attack_ms);
        float a_rel = env_coef(sc->release_ms);
        int tb = sc->target_bus;
        if (tb < 0 || tb >= BUSES) tb = 1;

        for (int f = 0; f < frames; f++) {
            /* Key is the voice bus after source gains, before its fader.
               Detector and application use the same sample index: no lookahead. */
            float vl = bus[KEY_BUS][2 * f + 0];
            float vr = bus[KEY_BUS][2 * f + 1];
            float v = fmaxf(fabsf(vl), fabsf(vr));

            float a = v > sc->env ? a_att : a_rel;
            sc->env = a * sc->env + (1.0f - a) * v;

            float level_db = 20.0f * log10f(sc->env + DB_EPS);
            float gr = static_curve_gr(level_db, sc->threshold_db, sc->ratio);
            float g = gr_to_gain(gr);

            bus[tb][2 * f + 0] *= g;
            bus[tb][2 * f + 1] *= g;

            sc->gr_db = gr;
            if (gr > sc->gr_max_db) sc->gr_max_db = gr;
        }
    } else {
        sc->gr_db = 0.0f;
    }

    for (int i = 0; i < frames * 2; i++) {
        float v = bus[0][i] * mx->gain[0]
                + bus[1][i] * mx->gain[1]
                + bus[2][i] * mx->gain[2];
        out[i] = soft_clip(v);
    }
}

/* ------------------------------------------------------------------ tests */

#define MAXLEN 16384
static float g_music[MAXLEN * 2];
static float g_voice[MAXLEN * 2];
static float g_out[MAXLEN * 2];

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void fill(float* buf, int frames, float v) {
    for (int i = 0; i < frames * 2; i++) buf[i] = v;
}

/* Run music + voice through the mixer in <=FRAMES blocks so sidechain state
   carries across blocks, as it would in a real callback. Voice fader is
   assumed to be muted by the caller when only the music gain matters. */
static void run(Mixer* mx, const float* music, const float* voice,
                int len, float* out) {
    Source srcs[2] = { { 1, 1.0f }, { 2, 1.0f } };
    for (int off = 0; off < len; off += FRAMES) {
        int n = len - off;
        if (n > FRAMES) n = FRAMES;
        const float* inb[2] = { music + 2 * off, voice + 2 * off };
        mix(mx, srcs, 2, inb, out + 2 * off, n);
    }
}

static int test_static_curve(void) {
    const float thr[3] = { -24.0f, -12.0f, -30.0f };
    const float rat[3] = { 4.0f, 2.0f, 8.0f };
    const float lvl[] = { -36.0f, -30.0f, -24.0f, -18.0f, -12.0f, -6.0f, -3.0f };
    const int nl = (int)(sizeof(lvl) / sizeof(lvl[0]));
    float worst = 0.0f;

    for (int c = 0; c < 3; c++) {
        for (int k = 0; k < nl; k++) {
            Mixer mx; mixer_init(&mx);
            sidechain_config(&mx, thr[c], rat[c], 0.5f, 50.0f);
            mx.gain[2] = 0.0f;  /* keep the key out of the measured output */
            float amp = powf(10.0f, lvl[k] / 20.0f);
            fill(g_music, 4096, 0.2f);
            fill(g_voice, 4096, amp);
            for (int rep = 0; rep < 4; rep++) run(&mx, g_music, g_voice, 4096, g_out);
            float g = g_out[(4096 - 1) * 2] / 0.2f;
            float meas = -20.0f * log10f(g);
            float ref = static_curve_gr(lvl[k], thr[c], rat[c]);
            float err = fabsf(meas - ref);
            if (err > worst) worst = err;
        }
    }
    printf("static_curve_worst_err_db=%.4f\n", worst);
    check("static_curve_ok", worst < 1.0f);
    return worst < 1.0f;
}

static int test_static_curve_sine(void) {
    Mixer mx; mixer_init(&mx);
    sidechain_config(&mx, -24.0f, 4.0f, 1.0f, 100.0f);
    mx.gain[2] = 0.0f;
    const float amp = 0.5f;
    for (int i = 0; i < 8192; i++) {
        float s = amp * sinf(6.28318530718f * 1000.0f * (float)i / SR);
        g_music[2 * i + 0] = g_music[2 * i + 1] = s;
        g_voice[2 * i + 0] = g_voice[2 * i + 1] = s;
    }
    run(&mx, g_music, g_voice, 8192, g_out);

    /* gain at the sine peak of the settled tail */
    int best = 8192 - 2048;
    for (int i = best + 1; i < 8192; i++)
        if (fabsf(g_music[2 * i]) > fabsf(g_music[2 * best])) best = i;
    float g = g_out[2 * best] / g_music[2 * best];
    float meas = -20.0f * log10f(fabsf(g));
    float ref = static_curve_gr(20.0f * log10f(amp), -24.0f, 4.0f);
    printf("sine_gr_meas_db=%.4f ref_db=%.4f\n", meas, ref);
    check("static_curve_sine_ok", fabsf(meas - ref) < 1.0f);
    return fabsf(meas - ref) < 1.0f;
}

static int test_attack_monotonic(void) {
    /* Longer attack must never produce more gain reduction than a shorter
       one within the same window. The window is short (8 ms) so that the
       slowest attacks are still converging and the ordering is observable. */
    const float att[] = { 1.0f, 2.0f, 5.0f, 10.0f, 20.0f };
    const int n = (int)(sizeof(att) / sizeof(att[0]));
    float prev = 1e9f;
    int mono = 1;
    float first = 0.0f, last = 0.0f;

    for (int k = 0; k < n; k++) {
        Mixer mx; mixer_init(&mx);
        sidechain_config(&mx, -24.0f, 4.0f, att[k], 100.0f);
        mx.gain[2] = 0.0f;
        sidechain_reset(&mx.sc);
        fill(g_music, 384, 0.2f);
        fill(g_voice, 384, 0.5f);
        run(&mx, g_music, g_voice, 384, g_out);   /* 8 ms of voice */
        float gr = mixer_gain_reduction_db(&mx);
        if (k == 0) first = gr;
        if (k == n - 1) last = gr;
        if (!(gr <= prev + 1e-3f)) mono = 0;
        prev = gr;
    }
    printf("attack_gr_1ms=%.3f 20ms=%.3f\n", first, last);
    int ok = mono && first - last > 1.0f;
    check("attack_monotonic", ok);
    return ok;
}

static int test_release_monotonic(void) {
    const float rel[] = { 20.0f, 50.0f, 100.0f, 200.0f, 500.0f };
    const int n = (int)(sizeof(rel) / sizeof(rel[0]));
    float prev = -1.0f;
    int mono = 1;
    float first = 0.0f, last = 0.0f;

    for (int k = 0; k < n; k++) {
        Mixer mx; mixer_init(&mx);
        sidechain_config(&mx, -24.0f, 4.0f, 1.0f, rel[k]);
        mx.gain[2] = 0.0f;
        sidechain_reset(&mx.sc);

        fill(g_music, 4800, 0.2f);
        fill(g_voice, 4800, 0.5f);
        run(&mx, g_music, g_voice, 4800, g_out);       /* 100 ms active */

        fill(g_music, 960, 0.2f);
        fill(g_voice, 960, 0.0f);
        run(&mx, g_music, g_voice, 960, g_out);        /* 20 ms recovery */

        float gr = mixer_gain_reduction_db(&mx);
        if (k == 0) first = gr;
        if (k == n - 1) last = gr;
        if (!(gr > prev + 0.01f)) mono = 0;
        prev = gr;
    }
    printf("release_gr_20ms=%.3f 500ms=%.3f\n", first, last);
    check("release_monotonic", mono);
    return mono;
}

static int test_gain_reduction_meter(void) {
    Mixer mx; mixer_init(&mx);
    sidechain_config(&mx, -24.0f, 4.0f, 1.0f, 100.0f);
    mx.gain[2] = 0.0f;
    sidechain_reset(&mx.sc);
    fill(g_music, 4800, 0.2f);
    fill(g_voice, 4800, 0.5f);
    run(&mx, g_music, g_voice, 4800, g_out);

    float g = g_out[(4800 - 1) * 2] / 0.2f;
    float measured = -20.0f * log10f(g);
    float reported = mixer_gain_reduction_db(&mx);
    float reported_max = mixer_gain_reduction_max_db(&mx);
    printf("gr_measured_db=%.4f reported_db=%.4f max_db=%.4f\n",
           measured, reported, reported_max);
    int ok = fabsf(measured - reported) < 0.1f
          && fabsf(reported_max - reported) < 0.1f
          && reported > 0.0f;
    check("gr_meter_ok", ok);
    return ok;
}

static int test_zero_latency(void) {
    Mixer mx; mixer_init(&mx);
    sidechain_config(&mx, -24.0f, 4.0f, 0.0f, 100.0f);  /* instant attack */
    mx.gain[2] = 0.0f;
    sidechain_reset(&mx.sc);
    fill(g_music, 256, 0.2f);
    fill(g_voice, 256, 0.0f);
    g_voice[64 * 2 + 0] = g_voice[64 * 2 + 1] = 0.5f;
    run(&mx, g_music, g_voice, 256, g_out);

    int first_changed = -1;
    for (int i = 0; i < 256; i++) {
        if (fabsf(g_out[2 * i] - 0.2f) > 1e-6f) { first_changed = i; break; }
    }
    float at_onset = g_out[64 * 2];
    printf("first_changed_sample=%d (voice onset 64) music_at_onset=%.5f\n",
           first_changed, at_onset);
    int ok = first_changed == 64 && at_onset < 0.2f - 1e-3f;
    check("zero_latency_ok", ok);
    return ok;
}

int main(void) {
    /* basic summing, no voice: sidechain stays idle */
    float sfx[FRAMES * 2], music[FRAMES * 2], voice[FRAMES * 2], out[FRAMES * 2];
    Mixer mx; mixer_init(&mx);

    fill(sfx, FRAMES, 0.3f);
    fill(music, FRAMES, 0.2f);
    fill(voice, FRAMES, 0.0f);
    const float* in[3] = { sfx, music, voice };
    Source srcs[3] = { { 0, 1.0f }, { 1, 1.0f }, { 2, 1.0f } };

    mix(&mx, srcs, 3, in, out, FRAMES);
    check("sum_ok", fabsf(out[0] - 0.5f) < 1e-5f);
    check("no_clip_when_low", mixer_gain_reduction_db(&mx) == 0.0f && out[0] < 0.99f);

    /* voice now drives gain reduction on the music bus, not on sfx */
    fill(voice, FRAMES, 0.3f);
    sidechain_reset(&mx.sc);
    mix(&mx, srcs, 3, in, out, FRAMES);
    float with_sc = out[(FRAMES - 1) * 2];
    float gr_when_active = mixer_gain_reduction_db(&mx);

    mx.sc.enabled = 0;
    mix(&mx, srcs, 3, in, out, FRAMES);
    float without_sc = out[(FRAMES - 1) * 2];
    check("sidechain_lowers_music", with_sc < without_sc - 1e-3f);
    check("sidechain_gr_visible", gr_when_active > 0.5f);

    /* sfx and voice are untouched: mute music and compare enabled vs disabled */
    fill(music, FRAMES, 0.0f);
    mx.sc.enabled = 1;
    sidechain_reset(&mx.sc);
    mix(&mx, srcs, 3, in, out, FRAMES);
    float sfx_on = out[(FRAMES - 1) * 2];
    mx.sc.enabled = 0;
    mix(&mx, srcs, 3, in, out, FRAMES);
    float sfx_off = out[(FRAMES - 1) * 2];
    check("sfx_voice_unaffected", fabsf(sfx_on - sfx_off) < 1e-6f);

    /* idle passthrough */
    fill(music, FRAMES, 0.2f);
    fill(voice, FRAMES, 0.0f);
    mx.sc.enabled = 1;
    sidechain_reset(&mx.sc);
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("sidechain_idle_passthrough", fabsf(out[0] - 0.5f) < 1e-5f);

    /* compressor behaviour */
    test_static_curve();
    test_static_curve_sine();
    test_attack_monotonic();
    test_release_monotonic();
    test_gain_reduction_meter();
    test_zero_latency();

    /* soft clipping bound with a hot sum */
    Mixer hot; mixer_init(&hot);
    hot.sc.enabled = 0;
    for (int s = 0; s < 3; s++) srcs[s].gain = 4.0f;
    fill(sfx, FRAMES, 0.5f);
    fill(music, FRAMES, 0.25f);
    fill(voice, FRAMES, 0.5f);
    mix(&hot, srcs, 3, in, out, FRAMES);
    float peak = 0.0f;
    for (int i = 0; i < FRAMES * 2; i++) if (fabsf(out[i]) > peak) peak = fabsf(out[i]);
    check("soft_clip_bounds", peak <= 1.0f + 1e-6f);

    srcs[0].bus = 7;   /* invalid bus must be ignored */
    mix(&hot, srcs, 3, in, out, FRAMES);
    peak = 0.0f;
    for (int i = 0; i < FRAMES * 2; i++) if (fabsf(out[i]) > peak) peak = fabsf(out[i]);
    check("bad_bus_ignored", peak <= 1.0f + 1e-6f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
