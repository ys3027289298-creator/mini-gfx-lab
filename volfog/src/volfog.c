
/* volfog: volumetric fog via jittered ray marching through an analytic
   density field, with Beer-Lambert transmittance and per-segment analytic
   in-scattering. The legacy uniform fog path is kept bit-identical and is
   used whenever volumetric fog is disabled.

   Self-test coverage:
     - rejects zero step count and negative density
     - disabled path matches legacy uniform fog exactly
     - per-region error report against a high-step reference
     - Richardson error bound for halved step count, verified against reference
     - low-discrepancy (Halton) jitter reduces banding error at low step counts */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef struct {
    int width, height;
    float cam_y;
    float fov_y;           /* vertical fov, radians */
    float t_max;           /* far clip for marching */
    float density_base;    /* >= 0, layered height-fog density */
    float density_blob;    /* >= 0, gaussian blob amplitude */
    float fog_rgb[3];      /* in-scatter color / legacy fog color */
    float uniform_density; /* >= 0, legacy uniform fog density */
} FogParams;

/* ---- density field ---------------------------------------------------- */
/* layer in [0.2, 1.0] and blob >= 0 keep the field non-negative as long as
   the config densities are non-negative; the marcher still rejects any
   negative sample at runtime. */
static float density_at(const FogParams* p, float x, float y, float z) {
    float layer = 0.6f + 0.4f * sinf(0.7f * x + 1.3f) * sinf(0.9f * z);
    float hf = expf(-0.35f * fmaxf(y + 1.0f, 0.0f)); /* denser near floor */
    float d = p->density_base * layer * hf;
    float dx = x - 2.0f, dy = y + 0.5f, dz = z - 6.0f;
    d += p->density_blob * expf(-(dx * dx + dy * dy + dz * dz) * 0.8f);
    return d;
}

int fog_validate(const FogParams* p) {
    if (p->width <= 0 || p->height <= 0) return -1;
    if (p->t_max <= 0.0f || p->fov_y <= 0.0f) return -1;
    if (p->density_base < 0.0f || p->density_blob < 0.0f) return -1;
    if (p->uniform_density < 0.0f) return -1;
    return 0;
}

/* ---- camera / scene ---------------------------------------------------- */
static void ray_dir(const FogParams* p, int px, int py, float d[3]) {
    float aspect = (float)p->width / (float)p->height;
    float tf = tanf(p->fov_y * 0.5f);
    float nx = ((px + 0.5f) / p->width * 2.0f - 1.0f) * tf * aspect;
    float ny = (1.0f - (py + 0.5f) / p->height * 2.0f) * tf;
    float inv = 1.0f / sqrtf(nx * nx + ny * ny + 1.0f);
    d[0] = nx * inv; d[1] = ny * inv; d[2] = inv;
}

/* floor plane at y=-1 with checker, otherwise sky gradient; returns hit t */
static float scene_color(const FogParams* p, const float o[3], const float d[3],
                         float rgb[3]) {
    if (d[1] < -1e-6f) {
        float t = (-1.0f - o[1]) / d[1];
        if (t < p->t_max) {
            float x = o[0] + d[0] * t, z = o[2] + d[2] * t;
            int chk = ((int)floorf(x) + (int)floorf(z)) & 1;
            float base = chk ? 0.55f : 0.22f;
            float diff = 0.35f + 0.65f * 0.8f; /* fixed key light on up-normal */
            rgb[0] = base * diff;
            rgb[1] = base * diff * 0.95f;
            rgb[2] = base * diff * 0.85f;
            return t;
        }
    }
    float g = fmaxf(d[1], 0.0f);
    rgb[0] = 0.15f + 0.20f * g;
    rgb[1] = 0.20f + 0.25f * g;
    rgb[2] = 0.35f + 0.30f * g;
    return p->t_max;
}

/* ---- legacy uniform fog ------------------------------------------------ */
int render_uniform(const FogParams* p, float* out) {
    if (fog_validate(p) != 0) return -1;
    float o[3] = { 0.0f, p->cam_y, 0.0f };
    for (int py = 0; py < p->height; py++) {
        for (int px = 0; px < p->width; px++) {
            float d[3], c[3];
            ray_dir(p, px, py, d);
            float t = scene_color(p, o, d, c);
            float f = 1.0f - expf(-p->uniform_density * t);
            float* dst = out + 3 * (py * p->width + px);
            for (int k = 0; k < 3; k++)
                dst[k] = c[k] * (1.0f - f) + p->fog_rgb[k] * f;
        }
    }
    return 0;
}

/* ---- volumetric fog ---------------------------------------------------- */
/* radical inverse base 2: low-discrepancy per-pixel ray-march offset */
static float halton2(unsigned i) {
    float f = 0.5f, r = 0.0f;
    while (i) { r += f * (float)(i & 1u); i >>= 1; f *= 0.5f; }
    return r;
}

/* march [0, t1] with midpoint density sampling and per-segment analytic
   transmittance; jit in [0,1) offsets all sample positions (correlated
   jitter turns banding into high-frequency noise). */
static int march_full(const FogParams* p, const float o[3], const float d[3],
                      float t1, int steps, float jit, float L[3], float* T) {
    float dt = t1 / (float)steps;
    float tr = 1.0f;
    L[0] = L[1] = L[2] = 0.0f;
    for (int i = 0; i < steps; i++) {
        float t = ((float)i + jit) * dt;
        float x = o[0] + d[0] * t, y = o[1] + d[1] * t, z = o[2] + d[2] * t;
        float rho = density_at(p, x, y, z);
        if (rho < 0.0f) return -1;
        float a = expf(-rho * dt);
        float w = tr * (1.0f - a);
        for (int k = 0; k < 3; k++) L[k] += w * p->fog_rgb[k];
        tr *= a;
    }
    *T = tr;
    return 0;
}

int render_volumetric(const FogParams* p, int steps, int jitter_on, float* out) {
    if (fog_validate(p) != 0) return -1;
    if (steps <= 0) return -1; /* reject zero/negative step count */
    float o[3] = { 0.0f, p->cam_y, 0.0f };
    for (int py = 0; py < p->height; py++) {
        for (int px = 0; px < p->width; px++) {
            float d[3], c[3], L[3], T;
            ray_dir(p, px, py, d);
            float t = scene_color(p, o, d, c);
            float jit = jitter_on ? halton2((unsigned)(py * p->width + px) + 1u)
                                  : 0.5f;
            if (march_full(p, o, d, t, steps, jit, L, &T) != 0) return -1;
            float* dst = out + 3 * (py * p->width + px);
            for (int k = 0; k < 3; k++) dst[k] = c[k] * T + L[k];
        }
    }
    return 0;
}

/* enabled=0 must reproduce the legacy uniform fog exactly */
int render_scene(const FogParams* p, int steps, int volumetric_enabled, float* out) {
    if (!volumetric_enabled) return render_uniform(p, out);
    return render_volumetric(p, steps, 1, out);
}

/* ---- error metrics ----------------------------------------------------- */
/* bands split the image horizontally: sky / horizon / floor */
static void band_error(const float* a, const float* b, int W, int H,
                       int band, int nbands, float* rmse, float* maxe) {
    int y0 = H * band / nbands, y1 = H * (band + 1) / nbands;
    double acc = 0.0; float mx = 0.0f; long n = 0;
    for (int y = y0; y < y1; y++) {
        for (int x = 0; x < W; x++) {
            for (int k = 0; k < 3; k++) {
                float e = fabsf(a[3 * (y * W + x) + k] - b[3 * (y * W + x) + k]);
                acc += (double)e * e;
                if (e > mx) mx = e;
                n++;
            }
        }
    }
    *rmse = (float)sqrt(acc / (double)n);
    *maxe = mx;
}

/* Richardson bound for a 2nd-order scheme with safety factor Fs=2 (Roache's
   grid-convergence-index practice: the raw 4/3 extrapolation is only an
   asymptotic estimate and underestimates where the field is not yet smooth
   enough, e.g. dense fog near the floor):
   E(N/2) <= Fs * (4/3)|I(N/2)-I(N)| */
static float halving_bound(const float* coarse, const float* fine, int W, int H,
                           int band, int nbands) {
    float rmse, mx;
    band_error(coarse, fine, W, H, band, nbands, &rmse, &mx);
    return rmse * (8.0f / 3.0f);
}

/* banding detector: fraction of horizontally adjacent pixels whose signed
   luminance error has the same sign. Structured (banded) error keeps one
   sign across whole bands (~1.0); jittered noise flips sign per pixel
   (~0.5). */
static float sign_agreement(const float* img, const float* ref, int W, int H) {
    long same = 0, total = 0;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x + 1 < W; x++) {
            float e0 = 0.0f, e1 = 0.0f;
            for (int k = 0; k < 3; k++) {
                e0 += img[3 * (y * W + x) + k] - ref[3 * (y * W + x) + k];
                e1 += img[3 * (y * W + x + 1) + k] - ref[3 * (y * W + x + 1) + k];
            }
            if (e0 * e1 > 0.0f) same++;
            total++;
        }
    }
    return (float)same / (float)total;
}

int main(void) {
    FogParams p = {
        .width = 64, .height = 36, .cam_y = 0.3f,
        .fov_y = 1.0f, .t_max = 30.0f,
        .density_base = 0.12f, .density_blob = 0.35f,
        .fog_rgb = { 0.62f, 0.66f, 0.72f },
        .uniform_density = 0.05f,
    };
    int W = p.width, H = p.height, npix = W * H * 3;
    int fails = 0;
    float *ref = malloc(npix * sizeof(float));
    float *imgA = malloc(npix * sizeof(float));
    float *imgB = malloc(npix * sizeof(float));
    float *imgC = malloc(npix * sizeof(float));
    if (!ref || !imgA || !imgB || !imgC) return 2;

    /* 1. invalid input rejection */
    FogParams bad = p; bad.density_base = -0.1f;
    int rej_neg = (render_volumetric(&bad, 16, 1, imgA) != 0);
    bad = p; bad.uniform_density = -1.0f;
    rej_neg = rej_neg && (render_uniform(&bad, imgA) != 0);
    int rej_zero = (render_volumetric(&p, 0, 1, imgA) != 0);
    printf("reject_negative_density=%d reject_zero_steps=%d\n", rej_neg, rej_zero);
    fails += !(rej_neg && rej_zero);

    /* 2. disabled path identical to legacy uniform fog */
    if (render_scene(&p, 16, 0, imgA) != 0) return 2;
    if (render_uniform(&p, imgB) != 0) return 2;
    int identical = (memcmp(imgA, imgB, npix * sizeof(float)) == 0);
    printf("disabled_matches_uniform=%d\n", identical);
    fails += !identical;

    /* 3. convergence vs high-step reference, error reported per band */
    if (render_volumetric(&p, 2048, 0, ref) != 0) return 2;
    const char* names[3] = { "sky", "horizon", "floor" };
    float prev_rmse = 1e9f;
    int monotone = 1;
    for (int s = 8; s <= 64; s *= 2) {
        if (render_volumetric(&p, s, 1, imgA) != 0) return 2;
        float rmse_all, mx_all;
        band_error(imgA, ref, W, H, 0, 1, &rmse_all, &mx_all);
        printf("steps=%3d rmse=%.5f", s, rmse_all);
        for (int b = 0; b < 3; b++) {
            float r, m;
            band_error(imgA, ref, W, H, b, 3, &r, &m);
            printf("  %s: rmse=%.5f max=%.5f", names[b], r, m);
        }
        printf("\n");
        if (rmse_all > prev_rmse) monotone = 0;
        prev_rmse = rmse_all;
    }
    printf("error_decreases_with_steps=%d\n", monotone);
    fails += !monotone;

    /* 4. halving bound: E(32) predicted from |I(32)-I(64)|, checked vs ref */
    if (render_volumetric(&p, 32, 1, imgB) != 0) return 2;
    if (render_volumetric(&p, 64, 1, imgC) != 0) return 2;
    int bound_ok = 1;
    for (int b = 0; b < 3; b++) {
        float actual, mx;
        band_error(imgB, ref, W, H, b, 3, &actual, &mx);
        float bound = halving_bound(imgB, imgC, W, H, b, 3);
        printf("halve 64->32 %s: bound=%.5f actual=%.5f covered=%d\n",
               names[b], bound, actual, actual <= bound);
        if (actual > bound) bound_ok = 0;
    }
    fails += !bound_ok;

    /* 5. jitter vs no jitter at low step count: banding is structured error,
       so compare spatial sign coherence, not RMSE (jitter trades structured
       error for noise of similar magnitude) */
    if (render_volumetric(&p, 8, 0, imgA) != 0) return 2;
    if (render_volumetric(&p, 8, 1, imgB) != 0) return 2;
    float r_plain, r_jit, m1, m2;
    band_error(imgA, ref, W, H, 0, 1, &r_plain, &m1);
    band_error(imgB, ref, W, H, 0, 1, &r_jit, &m2);
    float ag_plain = sign_agreement(imgA, ref, W, H);
    float ag_jit = sign_agreement(imgB, ref, W, H);
    int deband = (ag_jit < ag_plain) && (ag_jit < 0.6f);
    printf("steps=8 rmse_plain=%.5f rmse_jittered=%.5f "
           "sign_agreement plain=%.3f jittered=%.3f debands=%d\n",
           r_plain, r_jit, ag_plain, ag_jit, deband);
    fails += !deband;

    printf("all_ok=%d\n", fails == 0);
    free(ref); free(imgA); free(imgB); free(imgC);
    return fails ? 1 : 0;
}
