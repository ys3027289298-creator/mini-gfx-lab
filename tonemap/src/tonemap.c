
/* tonemap: HDR -> displayable LDR chain assembled from three swappable
   stages. The problem being addressed is that truncating HDR straight to
   [0,1] turns every highlight into the same white; here exposure, a curve
   and dither are run in a configured order, the staged loop is checked
   against hand-unrolled per-pixel formulas, and the self-test reports how
   much clipping survives, how the curve round-trips, whether the dither
   pattern leaves bands of its own, and how swapping stages changes output.

   Stage domains (kept explicit so that any order stays defined):
     exposure : [0,inf) -> [0,inf)   linear scale; may exceed 1 on purpose
     curve    : [0,inf) -> [0,1)     monotonic shoulder, exactly invertible
     dither   : [0,1]   -> [0,1]     zero-mean threshold, display referred
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define W 96
#define H 64
#define N (W * H)

/* ---- stage: exposure ------------------------------------------------- */
/* out = in * 2^stops, deliberately unclamped: the curve is what folds the
   extended range back down, so exposure must leave highlights above 1 for
   the shoulder to work on. */
static float stage_exposure(float x, float stops) {
    return x * exp2f(stops);
}

/* ---- stage: curve ---------------------------------------------------- */
/* Reinhard shoulder with a knee: f(x) = k*x / (k*x + 1).
   Monotonic, f(0)=0, f(inf)->1, and closed-form invertible:
   x = y / (k * (1 - y)). It approaches 1 slowly, so a hot pixel keeps a
   distinguishable code instead of snapping to pure white the way a fast
   exponential shoulder would. */
typedef struct { float k; } Curve;

static float curve_fwd(const Curve* c, float x) {
    if (x <= 0.0f) return 0.0f;
    float kx = c->k * x;
    return kx / (kx + 1.0f);
}

static float curve_inv(const Curve* c, float y) {
    if (y <= 0.0f) return 0.0f;
    if (y >= 1.0f) return 1e9f;
    return y / (c->k * (1.0f - y));
}

/* ---- stage: dither --------------------------------------------------- */
/* Interleaved gradient noise (Jimenez 2014): a deterministic, resolution
   independent threshold built from the pixel coordinates. Chosen over a
   4x4 Bayer matrix because Bayer rows/columns do not have equal means, so
   on a flat field it leaves a 4-pixel periodic stripe; IGN's row and column
   means are near equal (measured below). Amplitude is one quantisation
   step, centred on zero, so it carries no DC. */
static float fractf(float v) { return v - floorf(v); }

static float ign_threshold(int px, int py) {
    float g = fractf(0.06711056f * (float)px + 0.00583715f * (float)py);
    return fractf(52.9829189f * g) - 0.5f;      /* (-0.5, 0.5) */
}

/* 4x4 Bayer kept only as a comparison pattern for the test: it is the
   textbook choice, but its row and column means are unequal, which the
   stripe metric below exposes. */
static const float BAYER4[16] = {
     0.0f/16,  8.0f/16,  2.0f/16, 10.0f/16,
    12.0f/16,  4.0f/16, 14.0f/16,  6.0f/16,
     3.0f/16, 11.0f/16,  1.0f/16,  9.0f/16,
    15.0f/16,  7.0f/16, 13.0f/16,  5.0f/16
};

static float dither_threshold(int mode, int px, int py) {
    if (mode == 1) return BAYER4[(py & 3) * 4 + (px & 3)] - 0.5f;
    return ign_threshold(px, py);
}

static float stage_dither_mode(float x, int px, int py, int levels, int mode) {
    if (levels <= 1) return x;
    float step = 1.0f / (float)(levels - 1);
    float y = x + dither_threshold(mode, px, py) * step;
    return y < 0.0f ? 0.0f : (y > 1.0f ? 1.0f : y);
}

/* ---- the chain ------------------------------------------------------- */
typedef enum { ST_EXPOSURE = 0, ST_CURVE = 1, ST_DITHER = 2 } StageKind;

typedef struct {
    StageKind order[3];     /* permutation of the three stages */
    float stops;
    int curve_on;
    Curve curve;
    int dither_on;
    int dither_mode;        /* 0 = interleaved gradient noise, 1 = Bayer */
    int levels;             /* display quantisation levels, e.g. 64 or 256 */
} Chain;

static float chain_pixel(const Chain* ch, float x, int px, int py) {
    for (int i = 0; i < 3; i++) {
        switch (ch->order[i]) {
        case ST_EXPOSURE: x = stage_exposure(x, ch->stops); break;
        case ST_CURVE:    if (ch->curve_on) x = curve_fwd(&ch->curve, x); break;
        case ST_DITHER:   if (ch->dither_on)
                              x = stage_dither_mode(x, px, py, ch->levels, ch->dither_mode);
                          break;
        }
    }
    return x;
}

static int quantise(float x, int levels) {
    if (levels <= 1) return x > 0.5f;
    int q = (int)lroundf(x * (float)(levels - 1));
    return q < 0 ? 0 : (q > levels - 1 ? levels - 1 : q);
}

/* ---- hand-unrolled references --------------------------------------- */
/* Independent re-derivations of chain_pixel for the three orders the tests
   exercise. These do not call chain_pixel; that is what makes the equality
   check meaningful. The dither term is repeated here rather than shared. */
static float manual_ecd(const Chain* ch, float x, int px, int py) {
    float e = x * exp2f(ch->stops);
    float c = ch->curve_on ? (ch->curve.k * e) / (ch->curve.k * e + 1.0f) : e;
    if (!ch->dither_on) return c;
    float step = 1.0f / (float)(ch->levels - 1);
    float g = fractf(0.06711056f * (float)px + 0.00583715f * (float)py);
    float t = fractf(52.9829189f * g) - 0.5f;
    float d = c + t * step;
    return d < 0.0f ? 0.0f : (d > 1.0f ? 1.0f : d);
}

static float manual_ced(const Chain* ch, float x, int px, int py) {
    float c = ch->curve_on ? (ch->curve.k * x) / (ch->curve.k * x + 1.0f) : x;
    float e = c * exp2f(ch->stops);
    if (!ch->dither_on) return e;
    float step = 1.0f / (float)(ch->levels - 1);
    float g = fractf(0.06711056f * (float)px + 0.00583715f * (float)py);
    float t = fractf(52.9829189f * g) - 0.5f;
    float d = e + t * step;
    return d < 0.0f ? 0.0f : (d > 1.0f ? 1.0f : d);
}

static float manual_dec(const Chain* ch, float x, int px, int py) {
    float v = x;
    if (ch->dither_on) {
        float step = 1.0f / (float)(ch->levels - 1);
        float g = fractf(0.06711056f * (float)px + 0.00583715f * (float)py);
        float t = fractf(52.9829189f * g) - 0.5f;
        v += t * step;
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
    }
    float e = v * exp2f(ch->stops);
    return ch->curve_on ? (ch->curve.k * e) / (ch->curve.k * e + 1.0f) : e;
}

/* Compare against the hand-unrolled reference for the exact permutation in
   use. Returns -1 if the order has no reference, so a future stage order
   cannot silently be checked against the wrong formula. */
static int matches_manual(const Chain* ch, const float* hdr, float tol) {
    float (*ref_fn)(const Chain*, float, int, int);
    if (ch->order[0] == ST_EXPOSURE && ch->order[1] == ST_CURVE && ch->order[2] == ST_DITHER)
        ref_fn = manual_ecd;
    else if (ch->order[0] == ST_CURVE && ch->order[1] == ST_EXPOSURE && ch->order[2] == ST_DITHER)
        ref_fn = manual_ced;
    else if (ch->order[0] == ST_DITHER && ch->order[1] == ST_EXPOSURE && ch->order[2] == ST_CURVE)
        ref_fn = manual_dec;
    else
        return -1;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            float in = hdr[(size_t)y * W + x];
            if (fabsf(chain_pixel(ch, in, x, y) - ref_fn(ch, in, x, y)) > tol) return 0;
        }
    return 1;
}

/* ---- scene ----------------------------------------------------------- */
/* Smooth sky, a bright disc and a tiny specular spike: the sky is what a
   dither has to keep band-free, the spike is what exposure and the curve
   have to bring back into range without collapsing it to white. */
static void make_scene(float* hdr) {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float sky = 0.02f + 0.30f * (float)y / (float)(H - 1);
            float dx = (float)x - 30.0f, dy = (float)y - 22.0f;
            float light = 24.0f * expf(-(dx * dx + dy * dy) / 90.0f);
            float spike = 200.0f * expf(-((dx - 6.0f) * (dx - 6.0f) +
                                         (dy + 4.0f) * (dy + 4.0f)) / 6.0f);
            hdr[(size_t)y * W + x] = sky + light + spike;
        }
    }
}

/* ---- clipping -------------------------------------------------------- */
/* Pixels the old pipeline blows out: anything at or above 1 is a hard white
   in a straight truncation. */
static int count_white_truncated(const float* hdr) {
    int n = 0;
    for (int i = 0; i < N; i++) if (hdr[i] >= 1.0f) n++;
    return n;
}

/* Pixels that still end on the top display code after a chain. */
static int count_white_chain(const Chain* ch, const float* hdr) {
    int n = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            float v = chain_pixel(ch, hdr[(size_t)y * W + x], x, y);
            if (quantise(v, ch->levels) >= ch->levels - 1) n++;
        }
    return n;
}

/* ---- banding metrics ------------------------------------------------- */
/* Distinct output codes over a ramp whose total span is less than one
   quantisation step. With no dither the whole ramp snaps to a single code
   (one flat band); a working dither has to produce more than one. */
static int codes_over_subcode_ramp(const Chain* ch, float lo, float hi) {
    unsigned char seen[256];
    memset(seen, 0, sizeof(seen));
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            float in = lo + (hi - lo) * (float)x / (float)(W - 1);
            seen[quantise(chain_pixel(ch, in, x, y), ch->levels)] = 1;
        }
    int n = 0;
    for (int i = 0; i < ch->levels && i < 256; i++) n += seen[i];
    return n;
}

/* Worst deviation of a row or column mean from the ideal code on flat
   fields. This is the stripe signature: a pattern whose rows or columns
   carry unequal means (as a plain 4x4 Bayer matrix does) shows up here as
   a periodic band even when its overall DC is right. The ideal is taken
   from the same chain with dither off, so the metric isolates the dither
   pattern and is unaffected by exposure and curve. */
static float stripe_deviation(const Chain* ch, const Chain* ref,
                              const float* vals, int nv) {
    float worst = 0.0f;
    for (int v = 0; v < nv; v++) {
        float ideal = chain_pixel(ref, vals[v], 0, 0) * (float)(ch->levels - 1);
        double cols[W];
        for (int x = 0; x < W; x++) cols[x] = 0.0;
        for (int y = 0; y < H; y++) {
            double row = 0.0;
            for (int x = 0; x < W; x++) {
                int q = quantise(chain_pixel(ch, vals[v], x, y), ch->levels);
                row += q;
                cols[x] += q;
            }
            float d = fabsf((float)(row / W) - ideal);
            if (d > worst) worst = d;
        }
        for (int x = 0; x < W; x++) {
            float d = fabsf((float)(cols[x] / H) - ideal);
            if (d > worst) worst = d;
        }
    }
    return worst;
}

/* Mean output code over a flat patch, to show dither carries no DC. */
static float flat_mean(const Chain* ch, float v) {
    double sum = 0.0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            sum += quantise(chain_pixel(ch, v, x, y), ch->levels);
    return (float)(sum / (double)N);
}

/* Spread of output codes over the bright region of the scene. A chain that
   clips early collapses the highlight to one code and this goes to zero. */
static float highlight_std(const Chain* ch, const float* hdr) {
    double sum = 0.0, sum2 = 0.0;
    int n = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            size_t i = (size_t)y * W + x;
            if (hdr[i] < 4.0f) continue;
            double q = quantise(chain_pixel(ch, hdr[i], x, y), ch->levels);
            sum += q; sum2 += q * q; n++;
        }
    if (n < 2) return 0.0f;
    double mean = sum / n;
    double var = sum2 / n - mean * mean;
    return var > 0.0 ? (float)sqrt(var) : 0.0f;
}

static Chain make_chain(StageKind a, StageKind b, StageKind c) {
    Chain ch;
    memset(&ch, 0, sizeof(ch));
    ch.order[0] = a; ch.order[1] = b; ch.order[2] = c;
    ch.stops = -1.0f;          /* pull the scene into range */
    ch.curve_on = 1;
    ch.curve.k = 2.0f;
    ch.dither_on = 1;
    ch.dither_mode = 0;
    ch.levels = 64;            /* coarse on purpose so banding is measurable */
    return ch;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    static float hdr[N];
    make_scene(hdr);

    Chain good = make_chain(ST_EXPOSURE, ST_CURVE, ST_DITHER);

    /* 1. the symptom with data: straight truncation whites out a big area,
       the chain leaves far fewer top-code pixels */
    int clip_trunc = count_white_truncated(hdr);
    int clip_chain = count_white_chain(&good, hdr);
    printf("clipped_truncated=%d\n", clip_trunc);
    printf("clipped_chain=%d\n", clip_chain);
    printf("pixels_total=%d\n", N);
    printf("clipping_reduction_pct=%.2f\n",
           100.0 * (double)(clip_trunc - clip_chain) / (double)clip_trunc);
    check("tonemap_reduces_clipping", clip_chain < clip_trunc);
    check("tonemap_clipping_small", clip_chain * 10 < clip_trunc);

    /* 2. staged loop equals the hand-unrolled per-pixel formula */
    check("matches_manual_ecd", matches_manual(&good, hdr, 1e-6f) == 1);

    /* 3. curve: invertible, monotonic, bounded, optionally off */
    float worst_rt = 0.0f;
    for (int i = 0; i <= 512; i++) {
        float x = 8.0f * (float)i / 512.0f;
        float rt = curve_inv(&good.curve, curve_fwd(&good.curve, x));
        float er = fabsf(rt - x);
        if (er > worst_rt) worst_rt = er;
    }
    printf("curve_roundtrip_err=%.3g\n", worst_rt);
    check("curve_invertible", worst_rt < 1e-4f);
    check("curve_zero_fixed", curve_fwd(&good.curve, 0.0f) == 0.0f);
    check("curve_monotonic", curve_fwd(&good.curve, 0.5f) < curve_fwd(&good.curve, 2.0f));
    check("curve_bounded", curve_fwd(&good.curve, 1e4f) <= 1.0f);

    Chain noc = good; noc.curve_on = 0;
    check("curve_optional", matches_manual(&noc, hdr, 1e-6f) == 1);

    /* 4. dither breaks a sub-code ramp into more than one code, carries no
       DC, and its own pattern leaves no periodic stripe */
    Chain nod = good; nod.dither_on = 0;
    int codes_dither = codes_over_subcode_ramp(&good, 0.600f, 0.610f);
    int codes_plain = codes_over_subcode_ramp(&nod, 0.600f, 0.610f);
    printf("subcode_ramp_codes_dither=%d\n", codes_dither);
    printf("subcode_ramp_codes_undithered=%d\n", codes_plain);
    check("dither_breaks_bands", codes_dither > codes_plain && codes_dither >= 2);

    float m_on = flat_mean(&good, 0.6f);
    float m_off = flat_mean(&nod, 0.6f);
    printf("flat_mean_dither=%.3f flat_mean_undithered=%.3f\n", m_on, m_off);
    check("dither_no_dc_shift", fabsf(m_on - m_off) < 0.5f);

    float vals[3] = { 0.20f, 0.50f, 0.75f };
    float stripe_dither = stripe_deviation(&good, &nod, vals, 3);
    float stripe_plain = stripe_deviation(&nod, &nod, vals, 3);
    printf("stripe_dev_dither=%.3f\n", stripe_dither);
    printf("stripe_dev_undithered=%.3f\n", stripe_plain);
    check("dither_no_periodic_stripe", stripe_dither < 0.1f);
    check("stripe_not_worse_than_plain", stripe_dither < stripe_plain);

    /* Bayer for comparison: its unequal row means are the 4-pixel periodic
       stripe the metric above is built to catch. */
    Chain bayer = good; bayer.dither_mode = 1;
    float stripe_bayer = stripe_deviation(&bayer, &nod, vals, 3);
    printf("stripe_dev_bayer=%.3f\n", stripe_bayer);
    check("bayer_stripes_more_than_ign", stripe_bayer > stripe_dither);

    /* 5. order matters and is comparable: exposure before vs after curve.
       Report the mean absolute code difference so the two orders can be
       judged, not just told apart. */
    Chain swapped = make_chain(ST_CURVE, ST_EXPOSURE, ST_DITHER);
    double diff = 0.0;
    int changed = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            float in = hdr[(size_t)y * W + x];
            int a = quantise(chain_pixel(&good, in, x, y), good.levels);
            int b = quantise(chain_pixel(&swapped, in, x, y), swapped.levels);
            diff += abs(a - b);
            if (a != b) changed++;
        }
    printf("order_changed_px=%d\n", changed);
    printf("order_mean_abs_code_diff=%.2f\n", diff / (double)N);
    check("order_changes_result", changed > 0);
    check("order_difference_meaningful", diff / (double)N > 0.5);
    check("swapped_order_valid", matches_manual(&swapped, hdr, 1e-6f) == 1);

    /* 6. dither first is a worse order: its clamp destroys every pixel above
       1 before exposure and curve run, so highlight structure collapses */
    Chain dither_first = make_chain(ST_DITHER, ST_EXPOSURE, ST_CURVE);
    float hi_good = highlight_std(&good, hdr);
    float hi_first = highlight_std(&dither_first, hdr);
    printf("highlight_std_good=%.2f\n", hi_good);
    printf("highlight_std_dither_first=%.2f\n", hi_first);
    check("dither_first_degrades", hi_first < hi_good);
    check("dither_first_order_valid", matches_manual(&dither_first, hdr, 1e-6f) == 1);

    /* 7. deterministic and in range */
    static int a[N], b[N];
    for (int pass = 0; pass < 2; pass++) {
        int* dst = pass == 0 ? a : b;
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                dst[(size_t)y * W + x] =
                    quantise(chain_pixel(&good, hdr[(size_t)y * W + x], x, y), good.levels);
    }
    check("deterministic", memcmp(a, b, sizeof(a)) == 0);
    int bad = 0;
    for (int i = 0; i < N; i++) if (a[i] < 0 || a[i] >= good.levels) bad++;
    check("in_range", bad == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
