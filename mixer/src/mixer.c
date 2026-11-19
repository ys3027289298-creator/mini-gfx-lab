
/* mixer: multi-bus audio mixing with per-bus gain, sidechain ducking,
   aux send/return and soft clipping. All buffers are interleaved stereo.

   Aux send/return: every source has a post-gain send level. The sends are
   summed into one aux bus, run through a feedback-comb effect and returned
   into the master sum *before* soft clipping. The return never re-enters the
   send, so the graph contains no loop; the comb feedback is clamped below 1,
   so even an excessive send only yields a bounded, decaying tail. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_SRC 8
#define FRAMES 128
#define NBUS 3
#define AUX_DELAY 97        /* samples, ~2 ms at 48 kHz */
#define AUX_FB_MAX 0.98f    /* strictly below 1: the comb tail always decays */

typedef struct {
    int bus;        /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
    float aux_send; /* post-gain level sent to the aux bus, 0 = no send */
} Source;

typedef struct {
    float line[2][AUX_DELAY];
    int pos;
} AuxState;

typedef struct {
    float gain[NBUS];
    float duck_depth;    /* 0..1, applied to bus 0 when voice is active */
    int duck_enabled;
    float aux_feedback;  /* comb feedback, clamped to [0, AUX_FB_MAX] */
    float aux_return;    /* level of the processed aux bus in the master sum */
    AuxState aux;
} Mixer;

static Mixer mixer_default(void) {
    Mixer m;
    memset(&m, 0, sizeof(m));
    for (int b = 0; b < NBUS; b++) m.gain[b] = 1.0f;
    return m;
}

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

/* Master sum without the aux return, before soft clip. */
static void render_dry(const Mixer* mx, const Source* srcs, int n,
                       const float* const* in, float* out, int frames) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    float bus[NBUS][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b >= NBUS) continue;
        for (int i = 0; i < frames * 2; i++)
            bus[b][i] += in[s][i] * srcs[s].gain;
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
        out[i] = bus[0][i] * mx->gain[0] * dg
               + bus[1][i] * mx->gain[1]
               + bus[2][i] * mx->gain[2];
    }
}

/* Aux bus input: post-source-gain, pre-bus-gain sends. Sources on an invalid
   bus are ignored entirely, sends included. */
static void render_aux_send(const Source* srcs, int n,
                            const float* const* in, float* out, int frames) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    for (int s = 0; s < n; s++) {
        if (srcs[s].bus < 0 || srcs[s].bus >= NBUS) continue;
        float g = srcs[s].gain * srcs[s].aux_send;
        if (g == 0.0f) continue;
        for (int i = 0; i < frames * 2; i++)
            out[i] += in[s][i] * g;
    }
}

/* Feedback comb used as the aux effect. Output is the delayed tap, so the
   return joins the dry mix one delay period later. State carries across
   blocks. Invalid feedback (negative, NaN, >= 1) is coerced into a stable
   range, which is what keeps a hot send from ever self-oscillating. */
static void aux_process(AuxState* a, float feedback, const float* in,
                        float* out, int frames) {
    if (!(feedback >= 0.0f)) feedback = 0.0f;   /* also catches NaN */
    if (feedback > AUX_FB_MAX) feedback = AUX_FB_MAX;

    for (int i = 0; i < frames; i++) {
        int p = a->pos;
        for (int c = 0; c < 2; c++) {
            float d = a->line[c][p];
            out[2 * i + c] = d;
            a->line[c][p] = in[2 * i + c] + d * feedback;
        }
        a->pos = (p + 1 == AUX_DELAY) ? 0 : p + 1;
    }
}

static void mix(Mixer* mx, const Source* srcs, int n,
                const float* const* in, float* out, int frames) {
    float dry[FRAMES * 2], send[FRAMES * 2], wet[FRAMES * 2];

    render_dry(mx, srcs, n, in, dry, frames);
    render_aux_send(srcs, n, in, send, frames);
    /* Process even when the return is muted so the effect tail stays
       continuous when the return is brought back up. */
    aux_process(&mx->aux, mx->aux_feedback, send, wet, frames);

    for (int i = 0; i < frames * 2; i++)
        out[i] = soft_clip(dry[i] + mx->aux_return * wet[i]);
}

static float peak(const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(b[i]) > m) m = fabsf(b[i]);
    return m;
}

static float max_diff(const float* a, const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = fabsf(a[i] - b[i]);
        if (d > m) m = d;
    }
    return m;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    float sfx[FRAMES * 2], music[FRAMES * 2], voice[FRAMES * 2], out[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.0f;
    }
    const float* in[3] = { sfx, music, voice };
    Source srcs[3] = { {0, 1.0f, 0.0f}, {1, 1.0f, 0.0f}, {2, 1.0f, 0.0f} };
    Mixer mx = mixer_default();
    mx.duck_depth = 0.5f;
    mx.duck_enabled = 1;

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

    /* --- aux send/return ---------------------------------------------- */
    float sa[FRAMES * 2], sb[FRAMES * 2], sc[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) {
        sa[i] = 0.17f * sinf(0.031f * (float)i);
        sb[i] = 0.11f * sinf(0.017f * (float)i + 1.0f);
        sc[i] = 0.09f * sinf(0.007f * (float)i + 2.0f);
    }
    const float* aux_in[3] = { sa, sb, sc };
    Source aux_srcs[3] = {
        { 0, 0.8f, 0.7f },
        { 1, 0.5f, 0.3f },
        { 2, 1.0f, 0.1f },
    };

    Mixer mx_auto = mixer_default();
    mx_auto.gain[1] = 0.8f;
    mx_auto.duck_depth = 0.4f;
    mx_auto.duck_enabled = 1;
    mx_auto.aux_feedback = 0.6f;
    mx_auto.aux_return = 0.5f;
    Mixer mx_man = mx_auto;     /* same config, independent copy of the state */

    float dry[FRAMES * 2], send[FRAMES * 2], wet[FRAMES * 2];
    float ref[FRAMES * 2], got[FRAMES * 2];
    int match = 1, audible = 0;
    for (int blk = 0; blk < 4; blk++) {
        /* Manual path: dry master + independently processed aux track,
           summed back and soft clipped exactly like the mixer does. */
        render_dry(&mx_man, aux_srcs, 3, aux_in, dry, FRAMES);
        render_aux_send(aux_srcs, 3, aux_in, send, FRAMES);
        aux_process(&mx_man.aux, mx_man.aux_feedback, send, wet, FRAMES);
        for (int i = 0; i < FRAMES * 2; i++)
            ref[i] = soft_clip(dry[i] + mx_man.aux_return * wet[i]);

        mix(&mx_auto, aux_srcs, 3, aux_in, got, FRAMES);

        if (max_diff(ref, got, FRAMES * 2) != 0.0f) match = 0;
        for (int i = 0; i < FRAMES * 2; i++)
            if (fabsf(ref[i] - soft_clip(dry[i])) > 1e-3f) audible = 1;
    }
    check("aux_manual_match", match);
    check("aux_return_audible", audible);

    /* Overload: a send far above unity must not turn the comb into an
       oscillator. Feedback above 1 is clamped, and the open return path
       means the send amount cannot change the loop gain at all. */
    float burst[FRAMES * 2], zero[FRAMES * 2], wet2[FRAMES * 2];
    memset(burst, 0, sizeof(burst));
    memset(zero, 0, sizeof(zero));
    for (int i = 0; i < 16; i++) burst[i] = 1000.0f;

    AuxState s1, s2;
    float w1[FRAMES * 2], w2[FRAMES * 2];
    memset(&s1, 0, sizeof(s1));
    memset(&s2, 0, sizeof(s2));
    aux_process(&s1, 5.0f, burst, w1, FRAMES);          /* way above 1 */
    aux_process(&s2, AUX_FB_MAX, burst, w2, FRAMES);
    check("aux_feedback_clamped", max_diff(w1, w2, FRAMES * 2) == 0.0f);

    AuxState st;
    memset(&st, 0, sizeof(st));
    aux_process(&st, 5.0f, burst, wet2, FRAMES);
    float prev = peak(wet2, FRAMES * 2);
    int sinks = 1;
    for (int blk = 0; blk < 400; blk++) {
        aux_process(&st, 5.0f, zero, wet2, FRAMES);
        float p = peak(wet2, FRAMES * 2);
        if (p > prev + 1e-3f) sinks = 0;
        prev = p;
    }
    check("aux_no_self_oscillation", sinks && prev < 1.0f);

    /* Same overload, but through the full graph: a send of 1e6 on every
       source must stay finite and bounded across many blocks. */
    Source hot[3] = {
        { 0, 600.0f, 1e6f },
        { 1, 600.0f, 1e6f },
        { 2, 600.0f, 1e6f },
    };
    Mixer mx_hot = mixer_default();
    mx_hot.aux_feedback = 2.0f;     /* requested above the stable range */
    mx_hot.aux_return = 4.0f;
    int bounded = 1;
    for (int blk = 0; blk < 200; blk++) {
        mix(&mx_hot, hot, 3, aux_in, out, FRAMES);
        for (int i = 0; i < FRAMES * 2; i++)
            if (!isfinite(out[i]) || fabsf(out[i]) > 1.0f + 1e-6f) bounded = 0;
    }
    check("aux_hot_send_bounded", bounded);

    memset(&st, 0, sizeof(st));
    aux_process(&st, NAN, burst, wet2, FRAMES);
    aux_process(&st, NAN, zero, wet2, FRAMES);
    int finite = 1;
    for (int i = 0; i < FRAMES * 2; i++)
        if (!isfinite(wet2[i])) finite = 0;
    for (int c = 0; c < 2; c++)
        for (int i = 0; i < AUX_DELAY; i++)
            if (!isfinite(st.line[c][i])) finite = 0;
    check("aux_bad_feedback_finite", finite);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
