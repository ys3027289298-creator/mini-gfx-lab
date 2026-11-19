
/* mixer: multi-bus audio mixing with per-bus gain, equal-power pan,
   mid/side stereo width, sidechain ducking and soft clipping.
   All buffers are interleaved stereo. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_SRC 8
#define FRAMES 128
#define BUS_COUNT 3
#define PI_F 3.14159265358979323846f

typedef struct {
    int bus;        /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
} Source;

typedef struct {
    float gain[BUS_COUNT];
    float pan[BUS_COUNT];    /* -1 = hard left .. 0 = center .. 1 = hard right */
    float width[BUS_COUNT];  /* 0 = mono .. 1 = original stereo width */
    float duck_depth;        /* 0..1, applied to bus 0 when voice is active */
    int duck_enabled;
} Mixer;

static void mix_init(Mixer* mx) {
    memset(mx, 0, sizeof(*mx));
    for (int b = 0; b < BUS_COUNT; b++) {
        mx->gain[b] = 1.0f;
        mx->width[b] = 1.0f;   /* width defaults to identity, not to mono */
    }
    mx->duck_depth = 0.5f;
    mx->duck_enabled = 1;
}

static float soft_clip(float v) {
    const float k = 0.9f;
    float a = fabsf(v);
    if (a <= k) return v;
    float s = v < 0 ? -1.0f : 1.0f;
    return s * (k + (1.0f - k) * tanhf((a - k) / (1.0f - k)));
}

/* Per-bus pan + width. Mid/side decomposition followed by an orthogonal
   rotation in the (m, s) plane:

     m = (L + R) / 2,  s = (L - R) / 2,  s *= width,  alpha = pan * pi/4
     L' = m*(cos a - sin a) + s*(sin a + cos a)
     R' = m*(cos a + sin a) + s*(sin a - cos a)

   The rotation preserves m^2 + s^2 and therefore total stereo energy
   L^2 + R^2. At pan = 0 the matrix is the identity, and the mono fold
   (L' + R') / 2 equals the original mid for any width, so width synthesis
   introduces no mono fold-down loss. */
static void spatialize(float* buf, int frames, float pan, float width) {
    if (pan == 0.0f && width == 1.0f) return;   /* bit-exact passthrough */
    if (pan < -1.0f) pan = -1.0f;
    else if (pan > 1.0f) pan = 1.0f;
    if (width < 0.0f) width = 0.0f;
    else if (width > 1.0f) width = 1.0f;

    float a = pan * PI_F * 0.25f;
    float ca = cosf(a), sa = sinf(a);
    for (int f = 0; f < frames; f++) {
        float l = buf[f * 2];
        float r = buf[f * 2 + 1];
        float m = (l + r) * 0.5f;
        float s = (l - r) * 0.5f * width;
        buf[f * 2]     = m * (ca - sa) + s * (sa + ca);
        buf[f * 2 + 1] = m * (ca + sa) + s * (sa - ca);
    }
}

static void mix(const Mixer* mx, const Source* srcs, int n,
                const float* const* in, float* out, int frames) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    float bus[BUS_COUNT][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b >= BUS_COUNT) continue;
        for (int i = 0; i < frames * 2; i++)
            bus[b][i] += in[s][i] * srcs[s].gain;
    }

    for (int b = 0; b < BUS_COUNT; b++)
        spatialize(bus[b], frames, mx->pan[b], mx->width[b]);

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

/* mean square of one channel (even = left, odd = right) */
static float channel_energy(const float* b, int frames, int ch) {
    double e = 0.0;
    for (int i = 0; i < frames; i++) e += (double)b[i * 2 + ch] * b[i * 2 + ch];
    return (float)(e / frames);
}

/* Pearson correlation between the two channels */
static float channel_corr(const float* b, int frames) {
    double ml = 0.0, mr = 0.0;
    for (int i = 0; i < frames; i++) {
        ml += b[i * 2];
        mr += b[i * 2 + 1];
    }
    ml /= frames;
    mr /= frames;
    double cov = 0.0, vl = 0.0, vr = 0.0;
    for (int i = 0; i < frames; i++) {
        double dl = b[i * 2] - ml;
        double dr = b[i * 2 + 1] - mr;
        cov += dl * dr;
        vl += dl * dl;
        vr += dr * dr;
    }
    return (float)(cov / sqrt(vl * vr));
}

static float fold_energy(const float* b, int frames) {
    double e = 0.0;
    for (int i = 0; i < frames; i++) {
        float fold = (b[i * 2] + b[i * 2 + 1]) * 0.5f;
        e += (double)fold * fold;
    }
    return (float)(e / frames);
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
    Source srcs[3] = { {0, 1.0f}, {1, 1.0f}, {2, 1.0f} };
    Mixer mx;
    mix_init(&mx);

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
    srcs[0].bus = 0;
    for (int s = 0; s < 3; s++) srcs[s].gain = 1.0f;

    /* ---- pan: mono bus, four positions vs analytic equal-power gains ---- */
    float mono[FRAMES * 2], blank[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) { mono[i] = 0.5f; blank[i] = 0.0f; }
    const float* pin[3] = { mono, blank, blank };

    const float pans[4] = { -1.0f, -1.0f / 3.0f, 1.0f / 3.0f, 1.0f };
    const char* pnames[4] = { "hard_left", "left_third", "right_third", "hard_right" };
    for (int k = 0; k < 4; k++) {
        Mixer pm;
        mix_init(&pm);
        pm.duck_enabled = 0;
        pm.pan[0] = pans[k];
        mix(&pm, srcs, 1, pin, out, FRAMES);

        float alpha = pans[k] * PI_F * 0.25f;
        float ca = cosf(alpha), sa = sinf(alpha);
        float gl = 0.5f * (ca - sa);          /* analytic gains, m = 0.5 */
        float gr = 0.5f * (ca + sa);
        float el = 0.25f * (1.0f - sinf(2.0f * alpha));
        float er = 0.25f * (1.0f + sinf(2.0f * alpha));

        char name[64];
        snprintf(name, sizeof name, "pan_%s_gain", pnames[k]);
        check(name, fabsf(out[0] - gl) < 1e-5f && fabsf(out[1] - gr) < 1e-5f);
        snprintf(name, sizeof name, "pan_%s_energy", pnames[k]);
        check(name, fabsf(channel_energy(out, FRAMES, 0) - el) < 1e-5f
                 && fabsf(channel_energy(out, FRAMES, 1) - er) < 1e-5f);
        snprintf(name, sizeof name, "pan_%s_total_power", pnames[k]);
        check(name, fabsf(channel_energy(out, FRAMES, 0)
                        + channel_energy(out, FRAMES, 1) - 0.5f) < 1e-5f);

        /* exact rational energies at the +-1/3 positions: 3/8 and 1/8 */
        if (pans[k] > -1.0f && pans[k] < 0.0f)
            check("pan_left_third_fractions",
                  fabsf(channel_energy(out, FRAMES, 0) - 0.375f) < 1e-5f &&
                  fabsf(channel_energy(out, FRAMES, 1) - 0.125f) < 1e-5f);
        else if (pans[k] > 0.0f && pans[k] < 1.0f)
            check("pan_right_third_fractions",
                  fabsf(channel_energy(out, FRAMES, 0) - 0.125f) < 1e-5f &&
                  fabsf(channel_energy(out, FRAMES, 1) - 0.375f) < 1e-5f);
    }

    /* ---- pan center is identity and rotation keeps stereo energy ---- */
    float st[FRAMES * 2];
    for (int i = 0; i < FRAMES; i++) {
        float phase = 2.0f * PI_F * (float)i / (float)FRAMES;
        st[i * 2]     = 0.5f * sinf(phase);
        st[i * 2 + 1] = 0.5f * cosf(phase);   /* orthogonal to L */
    }
    const float* sin_[3] = { st, blank, blank };
    float e_before = channel_energy(st, FRAMES, 0) + channel_energy(st, FRAMES, 1);

    Mixer cm;
    mix_init(&cm);
    cm.duck_enabled = 0;
    mix(&cm, srcs, 1, sin_, out, FRAMES);
    int identity = 1;
    for (int i = 0; i < FRAMES * 2; i++)
        identity &= fabsf(out[i] - st[i]) < 1e-6f;
    check("pan_center_identity", identity);

    cm.pan[0] = 1.0f / 3.0f;
    mix(&cm, srcs, 1, sin_, out, FRAMES);
    float e_after = channel_energy(out, FRAMES, 0) + channel_energy(out, FRAMES, 1);
    check("pan_preserves_stereo_energy", fabsf(e_after - e_before) < 1e-5f);

    /* ---- width drives the L/R correlation coefficient ----
       For equal-power uncorrelated L, R: r(width) = (1 - w^2) / (1 + w^2) */
    const float widths[3] = { 0.0f, 0.5f, 1.0f };
    const float want_r[3] = { 1.0f, 0.6f, 0.0f };
    const char* wnames[3] = { "mono", "half", "full" };
    for (int k = 0; k < 3; k++) {
        Mixer wm;
        mix_init(&wm);
        wm.duck_enabled = 0;
        wm.width[0] = widths[k];
        mix(&wm, srcs, 1, sin_, out, FRAMES);
        char name[64];
        snprintf(name, sizeof name, "width_corr_%s", wnames[k]);
        check(name, fabsf(channel_corr(out, FRAMES) - want_r[k]) < 1e-4f);
    }
    check("width_monotonic_decorrelates",
          channel_corr(out, FRAMES) < 0.001f);  /* width 1 from last run */

    /* ---- mono compatibility: fold loss across width synthesis <= 3 dB ---- */
    Mixer wm;
    mix_init(&wm);
    wm.duck_enabled = 0;
    wm.width[0] = 1.0f;
    mix(&wm, srcs, 1, sin_, out, FRAMES);
    float ref_fold = fold_energy(out, FRAMES);
    float worst_db = 0.0f;
    int fold_matches_mid = 1;
    for (int k = 0; k <= 10; k++) {
        float w = (float)k / 10.0f;
        wm.width[0] = w;
        mix(&wm, srcs, 1, sin_, out, FRAMES);
        float fe = fold_energy(out, FRAMES);
        float loss_db = 10.0f * log10f(ref_fold / fe);
        if (loss_db > worst_db) worst_db = loss_db;
        /* fold must equal the original mid (L + R) / 2 at every width */
        for (int i = 0; i < FRAMES; i++) {
            float mid = (st[i * 2] + st[i * 2 + 1]) * 0.5f;
            float fold = (out[i * 2] + out[i * 2 + 1]) * 0.5f;
            if (fabsf(fold - mid) > 1e-6f) fold_matches_mid = 0;
        }
    }
    printf("width_fold_loss_worst_db=%.4f\n", worst_db);
    check("width_fold_loss_le_3db", worst_db <= 3.0f);
    check("fold_equals_mid_any_width", fold_matches_mid);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
