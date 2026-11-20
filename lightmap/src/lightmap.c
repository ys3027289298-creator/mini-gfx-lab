/* lightmap: bake a directional radiance probe into an SH-L2 irradiance map and
   sample it at runtime by normal. The self-test compares that runtime lookup
   against a per-pixel direct hemisphere integral, reports bake cost, and checks
   that sampling does not jump across the map seam.

   Why SH-L2 rather than a baked hemisphere raster: a raster sampled by normal
   is just the direct integral again (no approximation to measure, only seam
   aliasing). SH is the cheap path that actually approximates the cosine
   integral, so it is the one whose error is worth bounding.

   Bake model: per texel, project incoming radiance L(w) into real SH-L2 by
   uniform integration over the full sphere. One texel therefore answers any
   runtime normal; the Lambertian clamp is folded into the per-band weights
   SH_COS. The reference "per-pixel direct integration" does the hemisphere
   integral explicitly with cosine-weighted samples. Both report irradiance/pi
   (the diffuse colour), in [0, ~1] for this probe.

   Storage: 3 channels x 9 bands as IEEE half floats, wrapped in a W x H grid.
   Runtime does bilinear filtering on a periodic grid, so neighbouring chunks
   that share a texel edge agree and the value is C0 across the seam. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define LM_PI 3.14159265358979323846f
#define SH_COEFFS 9
#define CHANNELS 3

/* ---- directional radiance probe -------------------------------------------
   Fixed light directions, colours varying smoothly and periodically with
   position so the map is not a constant. No occlusion: the comparison then
   isolates SH truncation + filtering instead of hiding them behind shadowing
   noise. `probe_hard` is the known bad case for SH-L2 (hard light -> ringing)
   and is reported, not asserted away. */

typedef struct { float dir[3]; float rgb[3]; } LightDir;
typedef struct { LightDir lights[4]; int count; float ambient[3]; } Probe;

static void probe_at(float u, float v, Probe* p) {
    memset(p, 0, sizeof(*p));
    float s = 0.65f + 0.35f * sinf(2.0f * LM_PI * u) * cosf(2.0f * LM_PI * v);
    float t = 0.65f + 0.35f * cosf(2.0f * LM_PI * u) * sinf(2.0f * LM_PI * v);
    p->lights[0] = (LightDir){{0.35f, 0.90f, 0.26f}, {1.00f * s, 0.86f * s, 0.62f * s}};
    p->lights[1] = (LightDir){{-0.80f, 0.20f, -0.56f}, {0.18f * t, 0.24f * t, 0.34f * t}};
    p->count = 2;
    p->ambient[0] = 0.05f; p->ambient[1] = 0.05f; p->ambient[2] = 0.06f;
}

static void probe_flat(Probe* p) {
    memset(p, 0, sizeof(*p));
    p->ambient[0] = 1.0f; p->ambient[1] = 1.0f; p->ambient[2] = 1.0f;
}

/* Pure directional (delta) light in direction `dir`, radiance rgb. The direct
   integral for this probe is the closed form  max(0, n . dir) * rgb, so the
   hard case can be checked against exact maths instead of Monte Carlo: a
   near-delta cone is exactly where MC reference noise would masquerade as SH
   ringing, and vice versa. */
typedef struct { float dir[3]; float rgb[3]; } DeltaLight;

/* Radiance arriving from direction d (unit). */
static void probe_radiance(const Probe* p, const float d[3], float out[CHANNELS]) {
    for (int c = 0; c < CHANNELS; c++) out[c] = p->ambient[c];
    for (int i = 0; i < p->count; i++) {
        float dot = d[0] * p->lights[i].dir[0] + d[1] * p->lights[i].dir[1] + d[2] * p->lights[i].dir[2];
        if (dot <= 0.0f) continue;
        float w = dot * dot;                 /* broad Lambertian lobe */
        for (int c = 0; c < CHANNELS; c++) out[c] += p->lights[i].rgb[c] * w;
    }
}

/* ---- SH-L2 basis (real basis, orthonormal over the sphere) ---------------- */
static void sh_basis(const float d[3], float y[SH_COEFFS]) {
    float x = d[0], yv = d[1], z = d[2];
    y[0] = 0.282095f;
    y[1] = 0.488603f * yv;
    y[2] = 0.488603f * z;
    y[3] = 0.488603f * x;
    y[4] = 1.092548f * x * yv;
    y[5] = 1.092548f * yv * z;
    y[6] = 0.315392f * (3.0f * z * z - 1.0f);
    y[7] = 1.092548f * x * z;
    y[8] = 0.546274f * (x * x - yv * yv);
}

/* Lambertian band weights A_k / pi. Evaluating
   sum_k SH_COS[k] * L_k * Y_k(n) gives irradiance/pi for normal n. */
static const float SH_COS[SH_COEFFS] = {
    1.0f, 2.0f / 3.0f, 2.0f / 3.0f, 2.0f / 3.0f,
    1.0f / 4.0f, 1.0f / 4.0f, 1.0f / 4.0f, 1.0f / 4.0f, 1.0f / 4.0f
};

/* ---- samplers --------------------------------------------------------------
   The bake projects L over the full sphere (uniform, area-weighted); the
   direct reference integrates L*cos over the hemisphere. Both use stratified
   low-discrepancy samples: with an LCG, 64 spp on the two-light probe varied
   the DC estimate by +-20% between seeds, which is larger than the SH error
   being measured. Stratified sampling makes bake error a function of sample
   count, not of the seed, so the reported numbers are reproducible. */

/* Stratified cosine-weighted hemisphere sample: equal-area z bands, golden-
   ratio longitude. `seed` only rotates the longitude offset. */
static void sample_cosine(int s, int n, unsigned seed, float d[3]) {
    float u1 = ((float)s + 0.5f) / (float)n;
    float jitter = (float)(seed & 0xFFFFu) / 65536.0f;
    float u2 = fmodf((float)s * 0.61803398875f + jitter, 1.0f);
    float r = sqrtf(u1), phi = 2.0f * LM_PI * u2;
    d[0] = r * cosf(phi);
    d[1] = r * sinf(phi);
    d[2] = sqrtf(fmaxf(0.0f, 1.0f - u1));
}

/* Stratified sphere sample: equal-area latitude bands, golden-ratio longitude
   offset, one stratified point per band. A plain LCG gave the same key light
   different DC estimates at 64 spp (0.66..1.00 across seeds), which swamped the
   SH error under test; stratified samples make the estimator's error depend on
   the sample count, not the seed. `seed` only rotates the longitude offset. */
static void sample_sphere_strat(int s, int n, unsigned seed, float d[3]) {
    float z = 2.0f * ((float)s + 0.5f) / (float)n - 1.0f;
    float jitter = (float)(seed & 0xFFFFu) / 65536.0f;
    float phi = 2.0f * LM_PI * (fmodf((float)s * 0.61803398875f + jitter, 1.0f));
    float r = sqrtf(fmaxf(0.0f, 1.0f - z * z));
    d[0] = r * cosf(phi);
    d[1] = r * sinf(phi);
    d[2] = z;
}

/* ---- texture -------------------------------------------------------------- */
#define LM_W 16
#define LM_H 16
#define LM_TEXELS (LM_W * LM_H)

typedef struct {
    unsigned short tex[LM_TEXELS][SH_COEFFS][CHANNELS];   /* fp16 */
    int baked;
} Lightmap;

/* IEEE-754 binary32 -> binary16. Truncates the mantissa (relative error
   < 2^-11, well under the SH error we measure); flushes subnormals to zero. */
static unsigned short f32_to_f16(float f) {
    union { float f; unsigned u; } v;
    v.f = f;
    unsigned sign = (v.u >> 16) & 0x8000u;
    int exp = (int)((v.u >> 23) & 0xFFu) - 127;
    unsigned mant = v.u & 0x7FFFFFu;
    if (exp > 15) return (unsigned short)(sign | 0x7C00u);
    if (exp < -14) return (unsigned short)sign;
    return (unsigned short)(sign | ((unsigned)(exp + 15) << 10) | (mant >> 13));
}

static float f16_to_f32(unsigned short h) {
    unsigned sign = (h & 0x8000u) ? 0x80000000u : 0u;
    unsigned exp = (h >> 10) & 0x1Fu;
    unsigned mant = h & 0x3FFu;
    unsigned u;
    if (exp == 0) u = sign;
    else if (exp == 31) u = sign | 0x7F800000u | (mant << 13);
    else u = sign | ((exp - 15u + 127u) << 23) | (mant << 13);
    union { unsigned u; float f; } v;
    v.u = u;
    return v.f;
}

/* Project radiance from the probe at position (u,v) into SH-L2. */
static void project(const Probe* p, float u, float v, int samples, unsigned seed,
                    float sh[SH_COEFFS][CHANNELS]) {
    (void)u; (void)v;
    memset(sh, 0, sizeof(float) * SH_COEFFS * CHANNELS);
    for (int s = 0; s < samples; s++) {
        float d[3], rad[CHANNELS], y[SH_COEFFS];
        sample_sphere_strat(s, samples, seed, d);
        probe_radiance(p, d, rad);
        sh_basis(d, y);
        for (int k = 0; k < SH_COEFFS; k++)
            for (int c = 0; c < CHANNELS; c++)
                sh[k][c] += rad[c] * y[k];
    }
    float scale = 4.0f * LM_PI / (float)samples;   /* uniform sphere pdf = 1/4pi */
    for (int k = 0; k < SH_COEFFS; k++)
        for (int c = 0; c < CHANNELS; c++)
            sh[k][c] *= scale;
}

static void pack_texel(Lightmap* lm, int x, int y, const float sh[SH_COEFFS][CHANNELS]) {
    int t = y * LM_W + x;
    for (int k = 0; k < SH_COEFFS; k++)
        for (int c = 0; c < CHANNELS; c++)
            lm->tex[t][k][c] = f32_to_f16(sh[k][c]);
}

static void unpack_texel(const Lightmap* lm, int x, int y, float sh[SH_COEFFS][CHANNELS]) {
    int t = ((y % LM_H) + LM_H) % LM_H * LM_W + ((x % LM_W) + LM_W) % LM_W;
    for (int k = 0; k < SH_COEFFS; k++)
        for (int c = 0; c < CHANNELS; c++)
            sh[k][c] = f16_to_f32(lm->tex[t][k][c]);
}

/* Bake a pure directional light by projecting its radiance into SH-L2 in
   closed form rather than by sampling: a delta has radiance rgb wherever
   d == dir and 0 elsewhere, so its SH coefficient is rgb * Y_k(dir) * 4pi,
   and the 1/4pi from the projection cancels, leaving rgb * Y_k(dir). Sampling
   would need a huge sample count to find the delta at all. */
static void bake_delta(Lightmap* lm, const float dir[3], const float rgb[CHANNELS]) {
    float y[SH_COEFFS];
    sh_basis(dir, y);
    float sh[SH_COEFFS][CHANNELS];
    for (int k = 0; k < SH_COEFFS; k++)
        for (int c = 0; c < CHANNELS; c++) sh[k][c] = y[k] * rgb[c];
    for (int t = 0; t < LM_TEXELS; t++) pack_texel(lm, t % LM_W, t / LM_W, sh);
    lm->baked = 1;
}

/* Bake: each texel averages box*box sub-positions across its footprint, so a
   texel straddling a lighting gradient is a footprint average, not a point
   sample (that is what keeps the seam from aliasing). */
static void bake(Lightmap* lm, int samples, int box) {
    memset(lm, 0, sizeof(*lm));
    for (int y = 0; y < LM_H; y++) {
        for (int x = 0; x < LM_W; x++) {
            float acc[SH_COEFFS][CHANNELS];
            memset(acc, 0, sizeof(acc));
            for (int oy = 0; oy < box; oy++) {
                for (int ox = 0; ox < box; ox++) {
                    float u = ((float)x + ((float)ox + 0.5f) / (float)box) / (float)LM_W;
                    float v = ((float)y + ((float)oy + 0.5f) / (float)box) / (float)LM_H;
                    Probe p;
                    probe_at(u, v, &p);
                    unsigned seed = (unsigned)(y * 131 + oy) * 9781u ^ (unsigned)(x * 131 + ox) * 6271u;
                    float sh[SH_COEFFS][CHANNELS];
                    project(&p, u, v, samples, seed, sh);
                    for (int k = 0; k < SH_COEFFS; k++)
                        for (int c = 0; c < CHANNELS; c++) acc[k][c] += sh[k][c];
                }
            }
            float inv = 1.0f / (float)(box * box);
            for (int k = 0; k < SH_COEFFS; k++)
                for (int c = 0; c < CHANNELS; c++) acc[k][c] *= inv;
            pack_texel(lm, x, y, acc);
        }
    }
    lm->baked = 1;
}

/* Runtime lookup: bilinear on the periodic grid, then SH evaluation. */
static void sample(const Lightmap* lm, float u, float v, const float n[3], float out[CHANNELS]) {
    float fx = u * (float)LM_W - 0.5f;
    float fy = v * (float)LM_H - 0.5f;
    int x0 = (int)floorf(fx), y0 = (int)floorf(fy);
    float tx = fx - (float)x0, ty = fy - (float)y0;
    int x1 = x0 + 1, y1 = y0 + 1;

    float a[SH_COEFFS][CHANNELS], b[SH_COEFFS][CHANNELS];
    float c[SH_COEFFS][CHANNELS], d[SH_COEFFS][CHANNELS];
    unpack_texel(lm, x0, y0, a);
    unpack_texel(lm, x1, y0, b);
    unpack_texel(lm, x0, y1, c);
    unpack_texel(lm, x1, y1, d);

    float y[SH_COEFFS];
    sh_basis(n, y);
    for (int ch = 0; ch < CHANNELS; ch++) {
        float e = 0.0f;
        for (int k = 0; k < SH_COEFFS; k++) {
            float top = a[k][ch] + (b[k][ch] - a[k][ch]) * tx;
            float bot = c[k][ch] + (d[k][ch] - c[k][ch]) * tx;
            float coef = top + (bot - top) * ty;
            e += coef * y[k] * SH_COS[k];
        }
        out[ch] = e;
    }
}

/* Per-pixel direct integration at a position, for the probe the map was baked
   with: cosine-weighted hemisphere samples around n; mean(radiance) is
   irradiance/pi. Pass the same probe the bake used, otherwise the comparison
   is between two different lighting environments. */
static void reference(const Probe* p, const float n[3], int samples, float out[CHANNELS]) {
    float up[3] = {0, 0, 1};
    if (fabsf(n[2]) > 0.9f) { up[0] = 1; up[1] = 0; up[2] = 0; }
    float t[3] = {
        up[1] * n[2] - up[2] * n[1],
        up[2] * n[0] - up[0] * n[2],
        up[0] * n[1] - up[1] * n[0]
    };
    float tl = sqrtf(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
    t[0] /= tl; t[1] /= tl; t[2] /= tl;
    float b[3] = {
        n[1] * t[2] - n[2] * t[1],
        n[2] * t[0] - n[0] * t[2],
        n[0] * t[1] - n[1] * t[0]
    };

    float acc[CHANNELS] = {0};
    for (int s = 0; s < samples; s++) {
        float d[3], w[3], rad[CHANNELS];
        sample_cosine(s, samples, 0x9E3779B9u, d);
        w[0] = t[0] * d[0] + b[0] * d[1] + n[0] * d[2];
        w[1] = t[1] * d[0] + b[1] * d[1] + n[1] * d[2];
        w[2] = t[2] * d[0] + b[2] * d[1] + n[2] * d[2];
        probe_radiance(p, w, rad);
        for (int c = 0; c < CHANNELS; c++) acc[c] += rad[c];
    }
    for (int c = 0; c < CHANNELS; c++) out[c] = acc[c] / (float)samples;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    check("fp16_one", fabsf(f16_to_f32(f32_to_f16(1.0f)) - 1.0f) < 1e-6f);
    check("fp16_third", fabsf(f16_to_f32(f32_to_f16(1.0f / 3.0f)) - 1.0f / 3.0f) < 1e-3f);
    check("fp16_negative", fabsf(f16_to_f32(f32_to_f16(-0.25f)) + 0.25f) < 1e-6f);

    float y[SH_COEFFS];
    sh_basis((float[3]){0, 0, 1}, y);
    check("sh_dc", fabsf(y[0] - 0.282095f) < 1e-5f);
    check("sh_linear_z", fabsf(y[2] - 0.488603f) < 1e-5f);

    const int samples = 64;
    const int box = 2;
    long long total_samples = (long long)LM_TEXELS * box * box * samples;

    Lightmap lm;
    clock_t t0 = clock();
    bake(&lm, samples, box);
    clock_t t1 = clock();
    double ms = 1000.0 * (double)(t1 - t0) / (double)CLOCKS_PER_SEC;
    printf("bake_texels=%d box=%d samples_per_texel=%d total_samples=%lld bake_ms=%.3f\n",
           LM_TEXELS, box, samples, total_samples, ms);
    check("map_baked", lm.baked == 1);

    /* Normalisation: uniform radiance 1 everywhere must read back as
       irradiance/pi = 1 for every normal. This catches a wrong pi or pdf. */
    Lightmap flat;
    memset(&flat, 0, sizeof(flat));
    {
        float sh[SH_COEFFS][CHANNELS];
        Probe p;
        probe_flat(&p);
        project(&p, 0.0f, 0.0f, 4096, 12345u, sh);
        for (int t = 0; t < LM_TEXELS; t++) {
            for (int k = 0; k < SH_COEFFS; k++)
                for (int c = 0; c < CHANNELS; c++) flat.tex[t][k][c] = 0;
            pack_texel(&flat, t % LM_W, t / LM_W, sh);
        }
        flat.baked = 1;
    }
    {
        float got[CHANNELS];
        sample(&flat, 0.3f, 0.7f, (float[3]){0.3f, 0.7f, 0.64f}, got);
        check("uniform_ambient_one", fabsf(got[0] - 1.0f) < 5e-3f && fabsf(got[1] - 1.0f) < 5e-3f);
    }

    /* Accuracy against direct integration at texel centres (isolates SH
       truncation; bilinear error is measured separately below). */
    static const float normals[6][3] = {
        {0, 0, 1}, {0, 1, 0}, {1, 0, 0}, {-1, 0, 0},
        {0.57735f, 0.57735f, 0.57735f}, {0.0f, 0.70711f, 0.70711f}
    };
    float max_err = 0.0f, mean_err = 0.0f;
    int n_cmp = 0;
    for (int ty = 2; ty < LM_H - 2; ty += 5) {
        for (int tx = 2; tx < LM_W - 2; tx += 5) {
            float u = ((float)tx + 0.5f) / (float)LM_W;
            float v = ((float)ty + 0.5f) / (float)LM_H;
            Probe p;
            probe_at(u, v, &p);
            for (int i = 0; i < 6; i++) {
                float got[CHANNELS], want[CHANNELS];
                sample(&lm, u, v, normals[i], got);
                reference(&p, normals[i], 8192, want);
                for (int c = 0; c < CHANNELS; c++) {
                    float e = fabsf(got[c] - want[c]);
                    if (e > max_err) max_err = e;
                    mean_err += e;
                    n_cmp++;
                }
            }
        }
    }
    mean_err /= (float)n_cmp;
    printf("texel_err_max=%.4f texel_err_mean=%.4f\n", max_err, mean_err);
    check("texel_error_within_threshold", max_err < 0.06f);

    /* Accuracy including bilinear filtering, on a grid of arbitrary positions. */
    float filt_err = 0.0f;
    for (int i = 0; i < 64; i++) {
        float u = 0.013f + 0.97f * (float)i / 63.0f;
        float v = 0.017f + 0.96f * (float)((i * 37) % 64) / 63.0f;
        Probe p;
        probe_at(u, v, &p);
        for (int j = 0; j < 6; j++) {
            float got[CHANNELS], want[CHANNELS];
            sample(&lm, u, v, normals[j], got);
            reference(&p, normals[j], 8192, want);
            for (int c = 0; c < CHANNELS; c++) {
                float e = fabsf(got[c] - want[c]);
                if (e > filt_err) filt_err = e;
            }
        }
    }
    printf("filter_err_max=%.4f\n", filt_err);
    check("filter_error_within_threshold", filt_err < 0.09f);

    /* Seam: walk u across the wrap at fixed v. With bilinear filtering on a
       periodic grid the step between adjacent samples must equal the local
       slope that exists anywhere else on the same row; a jump means the wrap
       is sampling the wrong endpoints (or clamping instead of wrapping).
       Compare the largest step in the seam window against the largest step in
       the interior window, not against the mean, so a smooth but steep area
       does not mask a discontinuity. */
    float seam_step = 0.0f, interior_step = 0.0f, prev[CHANNELS] = {0};
    int have_prev = 0;
    const int steps = 400;
    for (int i = 0; i <= steps; i++) {
        float u = (float)i / (float)steps;
        float got[CHANNELS];
        sample(&lm, u, 0.313f, (float[3]){0.3f, 0.7f, 0.64f}, got);
        if (have_prev) {
            for (int c = 0; c < CHANNELS; c++) {
                float d = fabsf(got[c] - prev[c]);
                /* window width matches the seam window so the comparison is
                   slope-to-slope, not peak-to-mean */
                if (u > 0.97f || u < 0.03f) { if (d > seam_step) seam_step = d; }
                else if (d > interior_step) interior_step = d;
            }
        }
        memcpy(prev, got, sizeof(prev));
        have_prev = 1;
    }
    printf("seam_step=%.6f interior_step=%.6f\n", seam_step, interior_step);
    check("seam_no_jump", seam_step <= interior_step * 1.25f + 1e-5f);

    /* Negative control: a sampler that clamps instead of wrapping must fail
       the same equality at the seam (u -> 1 and u = 0 are one physical point).
       If the constant-direction map has no texel-0 vs texel-(W-1) contrast the
       control is meaningless, so assert on a map that does. */
    {
        Lightmap ramp;
        memset(&ramp, 0, sizeof(ramp));
        for (int y = 0; y < LM_H; y++) {
            for (int x = 0; x < LM_W; x++) {
                float sh[SH_COEFFS][CHANNELS];
                memset(sh, 0, sizeof(sh));
                sh[0][0] = 0.5f + 0.4f * (float)x / (float)(LM_W - 1);
                sh[0][1] = sh[0][0];
                sh[0][2] = sh[0][0];
                pack_texel(&ramp, x, y, sh);
            }
        }
        float a[CHANNELS], b[CHANNELS];
        sample(&ramp, 0.0f, 0.5f, (float[3]){0, 0, 1}, a);        /* wraps */
        sample(&ramp, 0.99999f, 0.5f, (float[3]){0, 0, 1}, b);
        float wrapped_delta = fabsf(a[0] - b[0]);

        /* Same thing with x1 clamped to the last texel instead of wrapping. */
        float fx = 0.99999f * (float)LM_W - 0.5f;
        int x0 = (int)floorf(fx), x1 = x0 + 1;
        if (x1 >= LM_W) x1 = LM_W - 1;
        float t = fx - floorf(fx);
        float c0[SH_COEFFS][CHANNELS], c1[SH_COEFFS][CHANNELS];
        unpack_texel(&ramp, x0, LM_H / 2, c0);
        unpack_texel(&ramp, x1, LM_H / 2, c1);
        float clamped = (c0[0][0] + (c1[0][0] - c0[0][0]) * t) * 0.282095f;
        float clamp_delta = fabsf(a[0] - clamped);
        printf("ramp_wrapped_delta=%.6f ramp_clamped_delta=%.6f\n",
               wrapped_delta, clamp_delta);
        check("seam_test_is_sensitive", clamp_delta > 0.05f && wrapped_delta < 1e-4f);
    }

    /* Explicit wrap equality: u just under 1 and u = 0 are the same point. */
    {
        float a[CHANNELS], b[CHANNELS];
        sample(&lm, 0.999999f, 0.313f, (float[3]){0.3f, 0.7f, 0.64f}, a);
        sample(&lm, 0.0f, 0.313f, (float[3]){0.3f, 0.7f, 0.64f}, b);
        float d = fabsf(a[0] - b[0]) + fabsf(a[1] - b[1]) + fabsf(a[2] - b[2]);
        printf("wrap_delta=%.7f\n", d);
        check("wrap_continuous", d < 1e-4f);
    }

    /* Known hard case: a pure directional (delta) light, compared against the
       closed form max(0, n . dir) / pi. SH-L2 reproduces the delta's radiance
       exactly; the only error is in reconstructing the clamped cosine from
       L2 bands, which is the classic ringing: normals facing away read back
       as a small positive value (~3% of peak, ~0.03 for unit radiance) instead
       of 0. That error is bounded and non-negative, so this case asserts a
       0.05 bound rather than 0: the SH-L2 clamp error is a known constant of
       the method, not a bug in the bake. */
    Lightmap hm;
    bake_delta(&hm, (float[3]){0.0f, 1.0f, 0.0f}, (float[3]){1.0f, 1.0f, 1.0f});
    float hard_err = 0.0f, hard_min = 1e9f;
    for (int i = 0; i < 6; i++) {
        float got[CHANNELS];
        sample(&hm, 0.5f, 0.5f, normals[i], got);
        float ndl = normals[i][1] > 0.0f ? normals[i][1] : 0.0f;
        float want = ndl / LM_PI;           /* irradiance/pi for a unit delta */
        float e = fabsf(got[0] - want);
        if (e > hard_err) hard_err = e;
        if (got[0] < hard_min) hard_min = got[0];
    }
    printf("delta_light_err_max=%.4f delta_light_min=%.4f (vs max(0,n.dir)/pi)\n",
           hard_err, hard_min);
    check("delta_light_low_error", hard_err < 0.05f);
    check("delta_light_non_negative", hard_min >= -1e-3f);
    /* The soft map is what the thresholds above certify; assert it never goes
       negative so a caller can skip the clamp for well-behaved probes. */
    float soft_min = 1e9f;
    for (int ty = 0; ty < LM_H; ty++) {
        for (int tx = 0; tx < LM_W; tx++) {
            float u = ((float)tx + 0.5f) / (float)LM_W;
            float v = ((float)ty + 0.5f) / (float)LM_H;
            for (int i = 0; i < 6; i++) {
                float got[CHANNELS];
                sample(&lm, u, v, normals[i], got);
                for (int c = 0; c < CHANNELS; c++) if (got[c] < soft_min) soft_min = got[c];
            }
        }
    }
    printf("soft_map_min=%.4f\n", soft_min);
    check("soft_map_non_negative", soft_min >= -1e-3f);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
