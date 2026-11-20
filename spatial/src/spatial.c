/* mini-spatial-audio: binaural positioning of mono sources plus a multi-tap
   early-reflection diffuser.

   Direct path: azimuth -> per-ear ITD (<= 0.6 ms) and IID, inverse-distance
   attenuation, soft-clip mix into an interleaved stereo buffer.

   Diffusion: tap count and per-tap amplitude decay are configurable. Taps are
   placed on a shell around the listener by a deterministic low-discrepancy
   sequence, so every tap owns its own path delay, arrival azimuth and per-ear
   delay; left and right ears are accumulated independently.

   The tap bank's energy is sum_k decay^(2k). For 0 <= decay < 1 that is bounded
   by 1/(1-decay^2) for every tap count, so adding taps converges instead of
   diverging. Zero taps, negative decay and decay >= 1 (unbounded bank) are
   rejected. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>

#define SR 48000
#define MAX_FRAMES 4096
#define MAX_SRC 8
#define MAX_TAPS 64
#define SPEED_SOUND 343.0f
#define MAX_ITD_SEC 0.0006f
#define PI_F 3.14159265358979323846f

typedef struct {
    float x, y, z;      /* world position, meters; listener at origin facing -z */
    float gain;
} Source;

typedef struct {
    float yaw;          /* radians, 0 = facing -z */
} Listener;

/* A reflection point on the tap shell around the listener. */
typedef struct {
    float x, y, z;
    float amp;          /* amplitude of this tap (decay^k) */
} Tap;

typedef struct {
    int taps;           /* 1..MAX_TAPS */
    float decay;        /* per-tap amplitude ratio, 0 <= decay < 1 */
    float radius;       /* nominal shell radius, meters */
    float spread;       /* 0 = straight ahead, 1 = full circle; clamped */
} DiffusionConfig;

typedef struct {
    int delay_l, delay_r;   /* samples after the direct arrival */
    float gl, gr;           /* per-ear IID gain */
    float amp;
} TapPath;

enum {
    DIFF_OK = 0,
    DIFF_ERR_NULL = -1,
    DIFF_ERR_TAPS = -2,
    DIFF_ERR_DECAY_NEG = -3,
    DIFF_ERR_DECAY_UNBOUNDED = -4,
    DIFF_ERR_RADIUS = -5
};

static const char* diffusion_strerror(int st) {
    switch (st) {
    case DIFF_OK:                return "ok";
    case DIFF_ERR_NULL:          return "null argument";
    case DIFF_ERR_TAPS:          return "tap count out of range";
    case DIFF_ERR_DECAY_NEG:     return "negative decay";
    case DIFF_ERR_DECAY_UNBOUNDED: return "decay >= 1 (unbounded energy)";
    case DIFF_ERR_RADIUS:        return "non-positive radius";
    default:                     return "unknown";
    }
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float fractf(float v) {
    return v - floorf(v);
}

/* azimuth in radians, 0 = front, +pi/2 = right */
static float azimuth_from(float x, float z, const Listener* l) {
    float c = cosf(l->yaw), sn = sinf(l->yaw);
    float xr =  c * x - sn * z;
    float zr =  sn * x + c * z;
    return atan2f(xr, -zr);
}

static float azimuth_of(const Source* s, const Listener* l) {
    return azimuth_from(s->x, s->z, l);
}

/* Interaural time difference as two non-negative per-ear delays: a source on
   the right leads the right ear by up to 0.6 ms. */
static void ear_delays(float az, int* dl, int* dr) {
    float itd = sinf(az) * (MAX_ITD_SEC * SR);
    *dl = (int)lroundf(itd > 0.0f ? itd : 0.0f);
    *dr = (int)lroundf(itd < 0.0f ? -itd : 0.0f);
}

/* power-complementary IID: gl^2 + gr^2 == 1 */
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

static void soft_clip_buffer(float* buf, int n) {
    for (int i = 0; i < n; i++) buf[i] = soft_clip(buf[i]);
}

/* ------------------------------------------------------------------ taps */

static int diffusion_generate(const DiffusionConfig* cfg, Tap* out) {
    if (!cfg || !out) return DIFF_ERR_NULL;
    if (cfg->taps <= 0 || cfg->taps > MAX_TAPS) return DIFF_ERR_TAPS;
    if (!(cfg->decay >= 0.0f)) return DIFF_ERR_DECAY_NEG;      /* also catches NaN */
    if (!(cfg->decay < 1.0f)) return DIFF_ERR_DECAY_UNBOUNDED;
    if (!(cfg->radius > 0.0f)) return DIFF_ERR_RADIUS;        /* also catches NaN */

    float spread = clampf(cfg->spread, 0.0f, 1.0f);
    float amp = 1.0f;
    for (int k = 0; k < cfg->taps; k++) {
        float u = ((float)k + 0.5f) / (float)cfg->taps;
        /* even azimuth sweep, plus an irrational-step elevation/radius scatter
           so tap positions (and therefore path delays) do not collapse. */
        float az = (2.0f * u - 1.0f) * PI_F * spread;
        float el = (fractf((float)(k + 1) * 0.61803398875f) - 0.5f)
                 * (PI_F / 6.0f) * spread;
        float r  = cfg->radius
                 * (0.6f + 0.8f * fractf((float)(k + 1) * 0.7548776662f));
        out[k].x = r * cosf(el) * sinf(az);
        out[k].y = r * sinf(el);
        out[k].z = -r * cosf(el) * cosf(az);
        out[k].amp = amp;
        amp *= cfg->decay;
    }
    return DIFF_OK;
}

/* delay = excess length of source -> tap -> listener over the direct path. */
static void tap_path_of(const Tap* t, const Source* s, const Listener* l,
                        TapPath* p) {
    float dx = t->x - s->x, dy = t->y - s->y, dz = t->z - s->z;
    float leg1 = sqrtf(dx * dx + dy * dy + dz * dz);
    float leg2 = sqrtf(t->x * t->x + t->y * t->y + t->z * t->z);
    float direct = sqrtf(s->x * s->x + s->y * s->y + s->z * s->z);
    float excess = (leg1 + leg2 - direct) * (SR / SPEED_SOUND);
    int base = (int)lroundf(excess);
    if (base < 0) base = 0;

    float az = azimuth_from(t->x, t->z, l);
    int dl, dr;
    ear_delays(az, &dl, &dr);
    iid_gains(az, &p->gl, &p->gr);
    p->delay_l = base + dl;
    p->delay_r = base + dr;
    p->amp = t->amp;
}

static double tap_energy(const Tap* taps, int n) {
    double e = 0.0;
    for (int k = 0; k < n; k++) e += (double)taps[k].amp * (double)taps[k].amp;
    return e;
}

/* Adds the per-ear tap superposition into an already-zeroed stereo buffer. */
static void render_taps(const Tap* taps, int n, const Source* s, const Listener* l,
                        const float* mono, int frames, float* out) {
    for (int k = 0; k < n; k++) {
        TapPath p;
        tap_path_of(&taps[k], s, l, &p);
        float gl = s->gain * p.amp * p.gl;
        float gr = s->gain * p.amp * p.gr;
        for (int i = 0; i < frames; i++) {
            int jl = i - p.delay_l;
            int jr = i - p.delay_r;
            if (jl >= 0) out[2 * i + 0] += mono[jl] * gl;
            if (jr >= 0) out[2 * i + 1] += mono[jr] * gr;
        }
    }
}

/* Analytic reference: each tap's path is re-derived here in double precision
   straight from the geometry (source -> tap -> listener excess length, then
   the tap's own arrival azimuth for the per-ear ITD/IID), independent of the
   renderer's tap_path_of. The self-test compares this against render_taps
   sample by sample. */
static void taps_reference(const Tap* taps, int n, const Source* s, const Listener* l,
                           const float* mono, int frames, double* out) {
    memset(out, 0, sizeof(double) * (size_t)frames * 2);
    double sr_over_c = (double)SR / (double)SPEED_SOUND;
    double sx = (double)s->x, sy = (double)s->y, sz = (double)s->z;
    for (int k = 0; k < n; k++) {
        const Tap* t = &taps[k];
        double tx = (double)t->x, ty = (double)t->y, tz = (double)t->z;
        double l1x = tx - sx, l1y = ty - sy, l1z = tz - sz;
        double leg1 = sqrt(l1x * l1x + l1y * l1y + l1z * l1z);
        double leg2 = sqrt(tx * tx + ty * ty + tz * tz);
        double direct = sqrt(sx * sx + sy * sy + sz * sz);
        double excess = (leg1 + leg2 - direct) * sr_over_c;
        int base = (int)llround(excess);
        if (base < 0) base = 0;

        double c = cos((double)l->yaw), sn = sin((double)l->yaw);
        double xr = c * tx - sn * tz;
        double zr = sn * tx + c * tz;
        double az = atan2(xr, -zr);
        double itd = sin(az) * ((double)MAX_ITD_SEC * (double)SR);
        int dl = (int)llround(itd > 0.0 ? itd : 0.0);
        int dr = (int)llround(itd < 0.0 ? -itd : 0.0);
        double tl = (sin(az) + 1.0) * 0.5;
        double gl = sqrt(1.0 - tl), gr = sqrt(tl);

        double a = (double)s->gain * (double)t->amp;
        for (int i = 0; i < frames; i++) {
            int jl = i - (base + dl);
            int jr = i - (base + dr);
            if (jl >= 0) out[2 * i + 0] += (double)mono[jl] * a * gl;
            if (jr >= 0) out[2 * i + 1] += (double)mono[jr] * a * gr;
        }
    }
}

/* --------------------------------------------------------------- rendering */

static void render_direct(const Source* srcs, int n, const Listener* l,
                          const float* mono, int frames, float* out) {
    for (int k = 0; k < n; k++) {
        float az = azimuth_of(&srcs[k], l);
        int dl, dr;
        ear_delays(az, &dl, &dr);
        float gl, gr;
        iid_gains(az, &gl, &gr);
        float g = srcs[k].gain * distance_gain(&srcs[k]);
        float gl_g = g * gl, gr_g = g * gr;
        for (int i = 0; i < frames; i++) {
            int jl = i - dl;
            int jr = i - dr;
            if (jl >= 0) out[2 * i + 0] += mono[jl] * gl_g;
            if (jr >= 0) out[2 * i + 1] += mono[jr] * gr_g;
        }
    }
}

/* Direct sources + optional diffusion, soft-clipped. The diffuser is applied
   per source, each with its own tap bank path. */
static int render_scene(const Source* srcs, int n, const DiffusionConfig* dc,
                        const Listener* l, const float* mono, int frames,
                        float* out) {
    memset(out, 0, sizeof(float) * (size_t)frames * 2);
    render_direct(srcs, n, l, mono, frames, out);
    if (dc) {
        Tap taps[MAX_TAPS];
        int st = diffusion_generate(dc, taps);
        if (st != DIFF_OK) return st;
        for (int k = 0; k < n; k++)
            render_taps(taps, dc->taps, &srcs[k], l, mono, frames, out);
    }
    soft_clip_buffer(out, frames * 2);
    return DIFF_OK;
}

/* ------------------------------------------------------------------- test */

static int failures = 0;

static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int peak_index(const float* buf, int frames, int ch) {
    int best = 0;
    for (int i = 1; i < frames; i++)
        if (fabsf(buf[2 * i + ch]) > fabsf(buf[2 * best + ch])) best = i;
    return best;
}

static float peak(const float* buf, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(buf[i]) > m) m = fabsf(buf[i]);
    return m;
}

static double stereo_energy(const float* buf, int frames) {
    double e = 0.0;
    for (int i = 0; i < frames; i++) {
        double a = (double)buf[2 * i], b = (double)buf[2 * i + 1];
        e += a * a + b * b;
    }
    return e;
}

int main(void) {
    static float mono[MAX_FRAMES];
    static float out[MAX_FRAMES * 2];
    static float raw[MAX_FRAMES * 2];
    static double ref[MAX_FRAMES * 2];
    Tap taps[MAX_TAPS];

    Listener l = { 0.0f };
    /* --- direct path: per-ear ITD leads on the source side ----------------- */
    for (int i = 0; i < MAX_FRAMES; i++) mono[i] = 0.0f;
    mono[64] = 1.0f;

    Source right45 = { 3.0f, 0.0f, -3.0f, 1.0f };   /* azimuth +pi/4 */
    memset(out, 0, sizeof(out));
    render_direct(&right45, 1, &l, mono, 256, out);
    int pl = peak_index(out, 256, 0), pr = peak_index(out, 256, 1);
    printf("right45_peakL=%d right45_peakR=%d\n", pl, pr);
    check("itd_right_ear_leads", pr == 64 && pl == 84);

    Source left45 = { -3.0f, 0.0f, -3.0f, 1.0f };   /* azimuth -pi/4 */
    memset(out, 0, sizeof(out));
    render_direct(&left45, 1, &l, mono, 256, out);
    pl = peak_index(out, 256, 0); pr = peak_index(out, 256, 1);
    printf("left45_peakL=%d left45_peakR=%d\n", pl, pr);
    check("itd_left_ear_leads", pl == 64 && pr == 84);

    Source near = { 0.0f, 0.0f, -1.0f, 1.0f };
    Source far  = { 0.0f, 0.0f, -9.0f, 1.0f };
    memset(out, 0, sizeof(out));
    render_direct(&near, 1, &l, mono, 256, out);
    float pn = peak(out, 512);
    memset(out, 0, sizeof(out));
    render_direct(&far, 1, &l, mono, 256, out);
    float pf = peak(out, 512);
    printf("attenuation near=%.4f far=%.4f ratio=%.3f\n",
           (double)pn, (double)pf, (double)(pf / pn));
    check("distance_attenuation", fabsf(pf / pn - 0.2f) < 1e-3f);

    /* --- diffusion config rejection --------------------------------------- */
    DiffusionConfig dc = { 0, 0.5f, 2.0f, 0.8f };
    int st = diffusion_generate(&dc, taps);
    printf("zero_taps_error=%s\n", diffusion_strerror(st));
    check("reject_zero_taps", st == DIFF_ERR_TAPS);

    dc.taps = 4; dc.decay = -0.25f;
    st = diffusion_generate(&dc, taps);
    printf("negative_decay_error=%s\n", diffusion_strerror(st));
    check("reject_negative_decay", st == DIFF_ERR_DECAY_NEG);

    dc.decay = 1.0f;
    st = diffusion_generate(&dc, taps);
    check("reject_unbounded_decay", st == DIFF_ERR_DECAY_UNBOUNDED);

    dc.decay = 0.5f; dc.radius = 0.0f;
    st = diffusion_generate(&dc, taps);
    check("reject_bad_radius", st == DIFF_ERR_RADIUS);

    dc.radius = 2.0f; dc.decay = 0.0f;
    st = diffusion_generate(&dc, taps);
    check("accept_zero_decay", st == DIFF_OK && taps[0].amp == 1.0f
                            && taps[1].amp == 0.0f && taps[2].amp == 0.0f);

    /* --- tap placement: positions and path delays spread out --------------- */
    DiffusionConfig spread_cfg = { 16, 0.7f, 2.0f, 1.0f };
    st = diffusion_generate(&spread_cfg, taps);
    check("valid_diffusion_config", st == DIFF_OK);

    float min_sep = 1e9f;
    for (int a = 0; a < spread_cfg.taps; a++)
        for (int b = a + 1; b < spread_cfg.taps; b++) {
            float dx = taps[a].x - taps[b].x, dy = taps[a].y - taps[b].y,
                  dz = taps[a].z - taps[b].z;
            float d = sqrtf(dx * dx + dy * dy + dz * dz);
            if (d < min_sep) min_sep = d;
        }

    Source front = { 0.0f, 0.0f, -4.0f, 1.0f };
    int dmin = 1 << 20, dmax = 0;
    for (int k = 0; k < spread_cfg.taps; k++) {
        TapPath p;
        tap_path_of(&taps[k], &front, &l, &p);
        int d = p.delay_l < p.delay_r ? p.delay_l : p.delay_r;
        if (d < dmin) dmin = d;
        if (d > dmax) dmax = d;
    }
    printf("tap_min_separation=%.4fm delay_span=%d samples\n",
           (double)min_sep, dmax - dmin);
    check("tap_positions_spread", min_sep > 0.05f);
    check("early_reflections_spread", dmax - dmin > 100);

    /* --- binaural: each ear gets its own delay and gain -------------------- */
    DiffusionConfig two = { 2, 0.8f, 2.0f, 1.0f };
    st = diffusion_generate(&two, taps);
    TapPath left_tap, right_tap;
    tap_path_of(&taps[0], &front, &l, &left_tap);    /* azimuth -pi/2 */
    tap_path_of(&taps[1], &front, &l, &right_tap);   /* azimuth +pi/2 */
    printf("tap_left dl=%d dr=%d, tap_right dl=%d dr=%d\n",
           left_tap.delay_l, left_tap.delay_r,
           right_tap.delay_l, right_tap.delay_r);
    check("tap_binaural_itd",
          right_tap.delay_l > right_tap.delay_r &&
          left_tap.delay_r > left_tap.delay_l);

    /* A reflection sitting inside the direct path (pathological geometry)
       must clamp to zero rather than index the buffer backwards. */
    Tap ahead = { 0.0f, 0.0f, -1.0f, 1.0f };
    Source behind = { 0.0f, 0.0f, -2.0f, 1.0f };   /* listener->source passes the tap */
    TapPath clamped;
    tap_path_of(&ahead, &behind, &l, &clamped);
    printf("clamped_tap dl=%d dr=%d\n", clamped.delay_l, clamped.delay_r);
    check("tap_delay_non_negative", clamped.delay_l >= 0 && clamped.delay_r >= 0);

    for (int i = 0; i < 512; i++) mono[i] = sinf(0.11f * (float)i) * 0.5f;
    memset(raw, 0, sizeof(raw));
    render_taps(taps, two.taps, &front, &l, mono, 512, raw);
    double ear_diff = 0.0;
    for (int i = 0; i < 512; i++) {
        double d = fabs((double)raw[2 * i] - (double)raw[2 * i + 1]);
        if (d > ear_diff) ear_diff = d;
    }
    printf("binaural_max_LR_diff=%.6f\n", ear_diff);
    check("ears_processed_separately", ear_diff > 1e-3);

    /* --- sample-by-sample comparison with the analytic tap superposition --- */
    Source off = { 2.0f, 0.5f, -3.0f, 1.0f };
    Listener yawed = { 0.4f };
    int frames = 256;
    for (int i = 0; i < frames; i++)
        mono[i] = 0.6f * sinf(0.23f * (float)i) + (i == 90 ? 0.4f : 0.0f);

    memset(raw, 0, sizeof(float) * (size_t)frames * 2);
    render_taps(taps, spread_cfg.taps, &off, &yawed, mono, frames, raw);
    taps_reference(taps, spread_cfg.taps, &off, &yawed, mono, frames, ref);

    double err_max = 0.0;
    for (int i = 0; i < frames * 2; i++) {
        double d = fabs((double)raw[i] - ref[i]);
        if (d > err_max) err_max = d;
    }

    /* Conservative float-accumulation bound: gamma_n * |mono|max * sum|coeff|,
       with a factor for the extra rounded products. */
    int nterms = spread_cfg.taps;
    double eps = (double)FLT_EPSILON;
    double gamma = ((double)nterms * eps) / (1.0 - (double)nterms * eps);
    double mono_max = 0.0, sum_l = 0.0, sum_r = 0.0;
    for (int i = 0; i < frames; i++)
        if (fabs((double)mono[i]) > mono_max) mono_max = fabs((double)mono[i]);
    for (int k = 0; k < nterms; k++) {
        TapPath p;
        tap_path_of(&taps[k], &off, &yawed, &p);
        sum_l += fabs((double)p.amp * (double)p.gl);
        sum_r += fabs((double)p.amp * (double)p.gr);
    }
    double coeff_sum = (sum_l > sum_r ? sum_l : sum_r) * fabs((double)off.gain);
    double err_bound = 8.0 * gamma * mono_max * coeff_sum;
     printf("ref_err_max=%.3e ref_err_bound=%.3e\n", err_max, err_bound);
    check("analytic_tap_superposition", err_max <= err_bound);

    /* The renderer must be silent where the analytic superposition is silent:
       compare the supports sample by sample too, so a mis-indexed delay can
       never hide inside a tolerance. */
    int support_mismatch = 0;
    for (int i = 0; i < frames * 2; i++) {
        int live = (raw[i] != 0.0f) != (ref[i] != 0.0);
        if (live) support_mismatch = 1;
    }
    check("tap_superposition_support", !support_mismatch);

    /* --- energy converges as the tap count grows -------------------------- */
    double decay = 0.8;
    double limit = 1.0 / (1.0 - decay * decay);
    double prev_e = -1.0, prev_gap = 1e300;
    int bounded = 1, monotone = 1, converging = 1;
    for (int n = 1; n <= MAX_TAPS; n *= 2) {
        DiffusionConfig c = { n, (float)decay, 2.0f, 1.0f };
        st = diffusion_generate(&c, taps);
        if (st != DIFF_OK) { bounded = 0; break; }
        double e = tap_energy(taps, n);
        double gap = limit - e;
        printf("energy taps=%d E=%.6f limit=%.6f gap=%.3e\n", n, e, limit, gap);
        /* amp is accumulated in float, so allow one relative float ulp band
           on top of the exact 1/(1-decay^2) bound. */
        if (e > limit * (1.0 + 1e-6)) bounded = 0;
        if (prev_e >= 0.0 && e < prev_e) monotone = 0;
        if (n > 1) {
            if (gap > prev_gap) converging = 0;
            if (gap >= prev_gap) converging = 0;
        }
        prev_e = e;
        prev_gap = gap;
    }
    check("tap_energy_bounded", bounded);
    check("tap_energy_monotone", monotone);
    check("tap_energy_converges", converging);
    check("tap_energy_tail_small", prev_gap < 1e-3 * limit);

    /* Rendered impulse energy must stay under the coherent upper bound. */
    DiffusionConfig big = { MAX_TAPS, (float)decay, 2.0f, 1.0f };
    st = diffusion_generate(&big, taps);
    memset(mono, 0, sizeof(float) * (size_t)MAX_FRAMES);
    mono[0] = 1.0f;
    memset(raw, 0, sizeof(raw));
    render_taps(taps, big.taps, &front, &l, mono, MAX_FRAMES, raw);
    double amp_sum = 0.0;
    for (int k = 0; k < big.taps; k++) amp_sum += (double)taps[k].amp;
    double coherent = (double)front.gain * (double)front.gain * amp_sum * amp_sum;
    double e_rendered = stereo_energy(raw, MAX_FRAMES);
    printf("rendered_energy=%.6f coherent_bound=%.6f\n", e_rendered, coherent);
    check("rendered_energy_bounded", e_rendered <= coherent + 1e-6);

    /* --- tap count / decay are honoured ----------------------------------- */
    int counts_ok = 1;
    for (int n = 1; n <= 16; n += 5) {
        DiffusionConfig c = { n, 0.5f, 2.0f, 1.0f };
        st = diffusion_generate(&c, taps);
        if (st != DIFF_OK) { counts_ok = 0; continue; }
        for (int k = 0; k < n; k++) {
            float want = powf(0.5f, (float)k);
            if (fabsf(taps[k].amp - want) > 1e-6f) counts_ok = 0;
        }
    }
    check("tap_count_and_decay_configurable", counts_ok);

    /* --- stereo mix + soft clip ------------------------------------------- */
    Source many[4] = { { 1, 0, 0, 1 }, { -1, 0, 0, 1 }, { 0, 0, -1, 1 }, { 0, 0, 1, 1 } };
    static float loud[MAX_FRAMES];
    for (int i = 0; i < 256; i++) loud[i] = 0.9f;
    st = render_scene(many, 4, NULL, &l, loud, 256, out);
    check("mix_ok", st == DIFF_OK);
    printf("mix_peak=%.4f (soft clip bound 1.0)\n", (double)peak(out, 512));
    check("soft_clip_bounds", peak(out, 512) <= 1.0f + 1e-6f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
