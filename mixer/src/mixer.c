
/* mixer: multi-bus audio mixing with per-bus gain, sidechain ducking and
   soft clipping. All buffers are interleaved stereo.

   Determinism contract:
   - Summation order is fixed: sources are accumulated in ascending source
     index, samples in ascending sample index, and the final bus combine is
     a fixed-order sum (bus 0, then 1, then 2).
   - Accumulation uses double precision so the result matches a
     high-precision reference to well under 1e-7.
   - No compiler-dependent reordering: build with -ffp-contract=off so
     mul+add cannot be fused into FMA (which changes rounding), and never
     enable -ffast-math (see Makefile). Given the same inputs, the output
     is bit-identical across runs, compilers and optimization levels. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_SRC 8
#define FRAMES 128

typedef struct {
    int bus;        /* 0 = sfx, 1 = music, 2 = voice */
    float gain;
} Source;

typedef struct {
    float gain[3];
    float duck_depth;    /* 0..1, applied to bus 0 when voice is active */
    int duck_enabled;
} Mixer;

static float soft_clip(double v) {
    const double k = 0.9;
    double a = fabs(v);
    if (a <= k) return (float)v;
    double s = v < 0 ? -1.0 : 1.0;
    return (float)(s * (k + (1.0 - k) * tanh((a - k) / (1.0 - k))));
}

static void mix(const Mixer* mx, const Source* srcs, int n,
                const float* const* in, float* out, int frames) {
    /* fixed order: source index 0..n-1, sample index 0..frames*2-1 */
    double bus[3][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b > 2) continue;
        double g = (double)srcs[s].gain;
        for (int i = 0; i < frames * 2; i++)
            bus[b][i] += (double)in[s][i] * g;
    }

    double dg = 1.0;
    if (mx->duck_enabled) {
        /* voice bus energy drives ducking of the sfx bus */
        double e = 0.0;
        for (int i = 0; i < frames * 2; i++) e += fabs(bus[2][i]);
        e /= (double)(frames * 2);
        if (e > 0.01) dg = 1.0 - (double)mx->duck_depth;
    }

    /* fixed order: bus 0, then 1, then 2 */
    for (int i = 0; i < frames * 2; i++) {
        double v = bus[0][i] * (double)mx->gain[0] * dg
                 + bus[1][i] * (double)mx->gain[1]
                 + bus[2][i] * (double)mx->gain[2];
        out[i] = soft_clip(v);
    }
}

/* High-precision reference: identical structure to mix(), but every
   operation in long double. Used only by the self-test to bound the
   summation error of the float pipeline. */
static void mix_ref(const Mixer* mx, const Source* srcs, int n,
                    const float* const* in, long double* out, int frames) {
    long double bus[3][FRAMES * 2];
    memset(bus, 0, sizeof(bus));

    for (int s = 0; s < n; s++) {
        int b = srcs[s].bus;
        if (b < 0 || b > 2) continue;
        long double g = (long double)srcs[s].gain;
        for (int i = 0; i < frames * 2; i++)
            bus[b][i] += (long double)in[s][i] * g;
    }

    long double dg = 1.0L;
    if (mx->duck_enabled) {
        long double e = 0.0L;
        for (int i = 0; i < frames * 2; i++) e += fabsl(bus[2][i]);
        e /= (long double)(frames * 2);
        if (e > 0.01L) dg = 1.0L - (long double)mx->duck_depth;
    }

    for (int i = 0; i < frames * 2; i++) {
        long double v = bus[0][i] * (long double)mx->gain[0] * dg
                      + bus[1][i] * (long double)mx->gain[1]
                      + bus[2][i] * (long double)mx->gain[2];
        long double a = fabsl(v);
        if (a <= 0.9L) {
            out[i] = v;
        } else {
            long double s = v < 0 ? -1.0L : 1.0L;
            out[i] = s * (0.9L + 0.1L * tanhl((a - 0.9L) / 0.1L));
        }
    }
}

static float peak(const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; i++) if (fabsf(b[i]) > m) m = fabsf(b[i]);
    return m;
}

static unsigned long long fnv1a(const float* b, int n) {
    const unsigned char* p = (const unsigned char*)b;
    unsigned long long h = 1469598103934665603ULL;
    for (size_t i = 0; i < sizeof(float) * (size_t)n; i++) {
        h ^= p[i];
        h *= 1099511628211ULL;
    }
    return h;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* Determinism evidence: pseudo-random inputs (fixed seed), mixed twice per
   source count; the two runs must be bit-identical sample by sample, the
   result must track a long-double reference within 1e-7, and appending a
   silent source must not change a single bit. */
static void test_determinism(void) {
    enum { NS = 5 };
    static float buf[NS][FRAMES * 2];
    static float zeros[FRAMES * 2];
    const float* in[NS + 1];
    Source srcs[NS + 1];
    unsigned rng = 12345u;
    for (int s = 0; s < NS; s++) {
        for (int i = 0; i < FRAMES * 2; i++) {
            rng = rng * 1664525u + 1013904223u;
            buf[s][i] = (float)(rng >> 8) / 8388608.0f - 1.0f; /* [-1,1) */
        }
        in[s] = buf[s];
        srcs[s].bus = s % 3;
        srcs[s].gain = 0.5f;
    }
    Mixer mx = { {0.9f, 0.8f, 1.0f}, 0.5f, 1 };

    float a[FRAMES * 2], b[FRAMES * 2];

    /* same input mixed twice, for each source count 1..NS */
    int det_ok = 1;
    for (int n = 1; n <= NS; n++) {
        mix(&mx, srcs, n, in, a, FRAMES);
        mix(&mx, srcs, n, in, b, FRAMES);
        int mism = 0;
        for (int i = 0; i < FRAMES * 2; i++)
            if (memcmp(&a[i], &b[i], sizeof(float)) != 0) mism++;
        printf("n=%d run1=%016llx run2=%016llx mismatch_samples=%d\n",
               n, fnv1a(a, FRAMES * 2), fnv1a(b, FRAMES * 2), mism);
        if (mism) det_ok = 0;
    }
    check("deterministic_rerun", det_ok);

    /* error vs high-precision reference summation */
    long double ref[FRAMES * 2];
    mix_ref(&mx, srcs, NS, in, ref, FRAMES);
    mix(&mx, srcs, NS, in, a, FRAMES);
    double max_err = 0.0;
    for (int i = 0; i < FRAMES * 2; i++) {
        double d = fabs((double)a[i] - (double)ref[i]);
        if (d > max_err) max_err = d;
    }
    printf("ref_max_err=%.3g\n", max_err);
    check("ref_err_lt_1e-7", max_err < 1e-7);

    /* appending a silent source must not change any output bit */
    in[NS] = zeros;
    srcs[NS].bus = 1;
    srcs[NS].gain = 0.7f;
    mix(&mx, srcs, NS, in, a, FRAMES);
    mix(&mx, srcs, NS + 1, in, b, FRAMES);
    printf("hash_n%d=%016llx hash_n%d=%016llx\n",
           NS, fnv1a(a, FRAMES * 2), NS + 1, fnv1a(b, FRAMES * 2));
    check("silent_source_invariant", memcmp(a, b, sizeof(a)) == 0);
}

int main(void) {
    float sfx[FRAMES * 2], music[FRAMES * 2], voice[FRAMES * 2], out[FRAMES * 2];
    for (int i = 0; i < FRAMES * 2; i++) {
        sfx[i] = 0.5f; music[i] = 0.25f; voice[i] = 0.0f;
    }
    const float* in[3] = { sfx, music, voice };
    Source srcs[3] = { {0, 1.0f}, {1, 1.0f}, {2, 1.0f} };
    Mixer mx = { {1.0f, 1.0f, 1.0f}, 0.5f, 1 };

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

    test_determinism();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
