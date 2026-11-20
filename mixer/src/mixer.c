
/* mixer: multi-bus audio mixing with per-bus gain, sidechain ducking,
   soft clipping and azimuth-based multichannel panning.
   Source inputs are mono; each source carries an azimuth and is
   assigned to the output channels by constant-power pairwise panning,
   with per-block cross-channel fades so moving sources never click.
   Output is interleaved with Mixer.channels samples per frame. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_SRC 8
#define MAX_CH 8
#define FRAMES 128

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    int bus;        /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
    float azimuth;  /* degrees: 0 = front, + = left, wraps at +-180 */
} Source;

typedef struct {
    float gain[3];
    float duck_depth;    /* 0..1, applied to bus 0 when voice is active */
    int duck_enabled;
    int channels;                /* 1..MAX_CH output channels */
    float pan[MAX_SRC][MAX_CH];  /* smoothed per-source channel gains */
    int pan_warm[MAX_SRC];       /* pan[] holds a valid ramp start */
    int pan_channels;            /* channel count the pan state belongs to */
} Mixer;

static Mixer mixer_new(int channels) {
    Mixer m;
    memset(&m, 0, sizeof(m));
    m.gain[0] = m.gain[1] = m.gain[2] = 1.0f;
    m.channels = channels;
    return m;
}

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

/* Speaker azimuths: mono is front-only, stereo is +-30, wider layouts
   are symmetric rings offset by a half step so they straddle the front. */
static void layout_angles(int channels, float* ang) {
    if (channels == 1) { ang[0] = 0.0f; return; }
    if (channels == 2) { ang[0] = 30.0f; ang[1] = -30.0f; return; }
    float step = 360.0f / (float)channels;
    for (int i = 0; i < channels; i++) {
        float a = step * 0.5f + step * (float)i;
        if (a > 180.0f) a -= 360.0f;
        ang[i] = a;
    }
}

/* Constant-power pairwise panning: the source plays through the two
   speakers whose azimuths bracket it, with cos/sin gains so the total
   energy (sum of squared gains) is 1 at every azimuth. All gains are
   non-negative, so a mono downmix can never cancel. */
static void pan_gains(float azimuth, int channels, float* out) {
    float ang[MAX_CH];
    int idx[MAX_CH];
    layout_angles(channels, ang);
    for (int i = 0; i < channels; i++) { out[i] = 0.0f; idx[i] = i; }
    if (channels == 1) { out[0] = 1.0f; return; }

    while (azimuth > 180.0f) azimuth -= 360.0f;
    while (azimuth <= -180.0f) azimuth += 360.0f;

    for (int i = 1; i < channels; i++) {  /* insertion sort by angle */
        int k = idx[i], j = i - 1;
        while (j >= 0 && ang[idx[j]] > ang[k]) { idx[j + 1] = idx[j]; j--; }
        idx[j + 1] = k;
    }

    for (int i = 0; i < channels; i++) {
        int a = idx[i], b = idx[(i + 1) % channels];
        float lo = ang[a];
        float hi = ang[b] + (i == channels - 1 ? 360.0f : 0.0f);
        float az = azimuth;
        if (i == channels - 1 && az < lo) az += 360.0f;
        if (az < lo || az > hi) continue;
        float t = (az - lo) / (hi - lo);
        /* clamp: float pi/2 is slightly high, so cosf can go a hair
           negative at t == 1 and break the no-cancellation guarantee */
        float ca = cosf(t * (float)M_PI * 0.5f);
        float sa = sinf(t * (float)M_PI * 0.5f);
        out[a] = ca < 0.0f ? 0.0f : ca;
        out[b] = sa < 0.0f ? 0.0f : sa;
        return;
    }
}

static void mix(Mixer* mx, const Source* srcs, int n,
                const float* const* in, float* out, int frames) {
    int ch = mx->channels;
    if (ch < 1) ch = 1;
    if (ch > MAX_CH) ch = MAX_CH;
    if (n > MAX_SRC) n = MAX_SRC;
    if (frames > FRAMES) frames = FRAMES;
    memset(out, 0, sizeof(float) * (size_t)frames * (size_t)ch);
    float bus[3][FRAMES * MAX_CH];
    memset(bus, 0, sizeof(bus));

    if (mx->pan_channels != ch) {  /* channel count changed: re-seed ramps */
        memset(mx->pan_warm, 0, sizeof(mx->pan_warm));
        mx->pan_channels = ch;
    }

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b > 2) continue;
        float tgt[MAX_CH];
        pan_gains(srcs[s].azimuth, ch, tgt);
        if (!mx->pan_warm[s]) {  /* first block: jump, don't fade in */
            memcpy(mx->pan[s], tgt, sizeof(tgt));
            mx->pan_warm[s] = 1;
        }
        /* cross-channel fade: ramp each channel gain linearly across the
           block so a moving or reassigned source never clicks */
        for (int f = 0; f < frames; f++) {
            float r = (float)(f + 1) / (float)frames;
            float v = in[s][f] * srcs[s].gain;
            for (int c = 0; c < ch; c++) {
                float g = mx->pan[s][c] + (tgt[c] - mx->pan[s][c]) * r;
                bus[b][f * ch + c] += v * g;
            }
        }
        memcpy(mx->pan[s], tgt, sizeof(tgt));
    }

    float dg = 1.0f;
    if (mx->duck_enabled) {
        /* voice bus energy drives ducking of the sfx bus */
        float e = 0.0f;
        for (int i = 0; i < frames * ch; i++) e += fabsf(bus[2][i]);
        e /= (float)(frames * ch);
        if (e > 0.01f) dg = 1.0f - mx->duck_depth;
    }

    for (int i = 0; i < frames * ch; i++) {
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

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    float sfx[FRAMES], music[FRAMES], voice[FRAMES], unit[FRAMES];
    float out[FRAMES * MAX_CH];
    for (int i = 0; i < FRAMES; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.0f; unit[i] = 1.0f;
    }
    const float* in[3] = { sfx, music, voice };
    Source srcs[3] = { {0, 1.0f, 0.0f}, {1, 1.0f, 0.0f}, {2, 1.0f, 0.0f} };
    const float c45 = 0.70710678f;  /* cos(pi/4): center of a speaker pair */

    Mixer mx = mixer_new(2);
    mx.duck_depth = 0.5f;
    mx.duck_enabled = 1;

    mix(&mx, srcs, 3, in, out, FRAMES);
    check("sum_ok", fabsf(out[0] - 0.75f * c45) < 1e-5f);
    check("no_clip_when_low", peak(out, FRAMES * 2) < 0.99f);

    for (int i = 0; i < FRAMES; i++) voice[i] = 0.3f;
    mix(&mx, srcs, 3, in, out, FRAMES);
    float with_duck = out[0];

    mx.duck_enabled = 0;
    mix(&mx, srcs, 3, in, out, FRAMES);
    float without_duck = out[0];
    check("duck_lowers_sfx", with_duck < without_duck);

    mx.duck_enabled = 1;
    for (int i = 0; i < FRAMES; i++) voice[i] = 0.0f;
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("duck_idle_passthrough", fabsf(out[0] - 0.75f * c45) < 1e-5f);

    for (int s = 0; s < 3; s++) srcs[s].gain = 4.0f;
    for (int i = 0; i < FRAMES; i++) voice[i] = 0.5f;
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("soft_clip_bounds", peak(out, FRAMES * 2) <= 1.0f + 1e-6f);

    srcs[0].bus = 7;   /* invalid bus must be ignored */
    mix(&mx, srcs, 3, in, out, FRAMES);
    check("bad_bus_ignored", peak(out, FRAMES * 2) <= 1.0f + 1e-6f);

    /* auto panning must match mixing each channel by hand with the
       constant-power gains */
    const float* uin[1] = { unit };
    /* keep the source gain below the soft-clip knee so the panned
       values reach the output unchanged */
    Source one = { 0, 0.5f, 0.0f };
    Mixer m2 = mixer_new(2);
    mix(&m2, &one, 1, uin, out, FRAMES);
    check("pan_center_matches_manual",
          fabsf(out[0] - 0.5f * c45) < 1e-5f &&
          fabsf(out[1] - 0.5f * c45) < 1e-5f);

    one.azimuth = 30.0f;
    m2 = mixer_new(2);
    mix(&m2, &one, 1, uin, out, FRAMES);
    check("pan_hard_left_matches_manual",
          fabsf(out[0] - 0.5f) < 1e-5f && fabsf(out[1]) < 1e-5f);

    one.azimuth = -30.0f;
    m2 = mixer_new(2);
    mix(&m2, &one, 1, uin, out, FRAMES);
    check("pan_hard_right_matches_manual",
          fabsf(out[0]) < 1e-5f && fabsf(out[1] - 0.5f) < 1e-5f);

    one.azimuth = 90.0f;  /* quad: halfway between FL(45) and RL(135) */
    Mixer m4 = mixer_new(4);
    mix(&m4, &one, 1, uin, out, FRAMES);
    check("pan_quad_side_matches_manual",
          fabsf(out[0] - 0.5f * c45) < 1e-5f &&
          fabsf(out[1] - 0.5f * c45) < 1e-5f &&
          fabsf(out[2]) < 1e-5f && fabsf(out[3]) < 1e-5f);

    /* energy continuity: sweep the azimuth and require constant total
       energy and no gain jumps, comparable across channel counts */
    const int counts[] = { 1, 2, 4, 5, 6, 8 };
    int energy_ok = 1, steps_ok = 1;
    for (int k = 0; k < 6; k++) {
        int ch = counts[k];
        float prev[MAX_CH] = { 0.0f };
        float max_dev = 0.0f, max_step = 0.0f;
        int have_prev = 0;
        for (float az = -180.0f; az <= 180.01f; az += 0.5f) {
            float g[MAX_CH];
            pan_gains(az, ch, g);
            float e = 0.0f;
            for (int c = 0; c < ch; c++) {
                e += g[c] * g[c];
                float d = fabsf(g[c] - prev[c]);
                if (have_prev && d > max_step) max_step = d;
            }
            float dev = fabsf(e - 1.0f);
            if (dev > max_dev) max_dev = dev;
            memcpy(prev, g, sizeof(g));
            have_prev = 1;
        }
        printf("pan_ch%d_energy_dev=%.6f max_gain_step=%.6f\n",
               ch, max_dev, max_step);
        if (max_dev > 1e-4f) energy_ok = 0;
        if (max_step > 0.02f) steps_ok = 0;
    }
    check("energy_continuous", energy_ok);
    check("gain_steps_small", steps_ok);

    /* mono compatibility: gains never go negative (no downmix
       cancellation) and the stereo downmix stays within 3 dB of the
       direct mono render at every azimuth */
    int mono_ok = 1;
    for (float az = -180.0f; az <= 180.01f; az += 0.5f) {
        float g[MAX_CH];
        pan_gains(az, 2, g);
        if (g[0] < 0.0f || g[1] < 0.0f) mono_ok = 0;
        float dn = (g[0] + g[1]) / sqrtf(2.0f);
        if (dn < 0.7f || dn > 1.001f) mono_ok = 0;
    }
    check("mono_downmix_compatible", mono_ok);

    /* cross-channel fade: a jumping source ramps across the block
       instead of stepping */
    Mixer mf = mixer_new(2);
    Source mv = { 0, 0.5f, 30.0f };  /* below the soft-clip knee */
    mix(&mf, &mv, 1, uin, out, FRAMES);  /* settle at hard left */
    mv.azimuth = -30.0f;
    mix(&mf, &mv, 1, uin, out, FRAMES);  /* ramp left -> right */
    int smooth = 1;
    float prev_l = 0.5f;
    for (int f = 0; f < FRAMES; f++) {
        float gl = out[f * 2];  /* unit input: sample value == gain * 0.5 */
        if (gl > prev_l + 1e-6f) smooth = 0;
        if (prev_l - gl > 1.0f / (float)FRAMES + 1e-6f) smooth = 0;
        prev_l = gl;
    }
    check("crossfade_smooth", smooth);
    check("crossfade_lands_on_target",
          fabsf(out[(FRAMES - 1) * 2]) < 1e-5f &&
          fabsf(out[(FRAMES - 1) * 2 + 1] - 0.5f) < 1e-5f);
    mix(&mf, &mv, 1, uin, out, FRAMES);
    check("crossfade_settles",
          fabsf(out[0]) < 1e-5f && fabsf(out[1] - 0.5f) < 1e-5f);

    /* moving source: render the same rotating source at each channel
       count and compare block-to-block energy continuity */
    int move_ok = 1;
    for (int k = 0; k < 6; k++) {
        int ch = counts[k];
        Mixer mm = mixer_new(ch);
        Source rot = { 0, 1.0f, 0.0f };
        float prev_e = -1.0f, worst_jump = 0.0f;
        for (int blk = 0; blk < 12; blk++) {
            rot.azimuth = -180.0f + 30.0f * (float)blk;
            mix(&mm, &rot, 1, uin, out, FRAMES);
            float e = 0.0f;
            for (int i = 0; i < FRAMES * ch; i++) e += out[i] * out[i];
            e /= (float)FRAMES;
            if (e < 0.75f || e > 1.05f) move_ok = 0;
            if (prev_e > 0.0f) {
                float j = fabsf(e - prev_e) / prev_e;
                if (j > worst_jump) worst_jump = j;
            }
            prev_e = e;
        }
        printf("move_ch%d_worst_block_jump=%.4f\n", ch, worst_jump);
        if (worst_jump > 0.15f) move_ok = 0;
    }
    check("moving_source_energy_continuous", move_ok);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
