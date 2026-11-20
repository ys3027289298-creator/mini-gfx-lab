/* keymatte: chroma-key matte extraction for sprites shot on a flat backing
   colour. The distance from each pixel's chroma to the backing chroma is
   turned into coverage alpha (luma is ignored, so shading on the sprite
   survives), and the pixel colour is divided through by that coverage to
   remove the backing contribution ("decontamination"), which is what kills
   the coloured rim. The matte is then shrunk and feathered. Every stage is
   a single pass over the pixels and the arithmetic is integer fixed point,
   so the cost is linear in the pixel count. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

/* BT.601 Cb/Cr, scaled by 1e6. Each coordinate stays below 2^31, so squared
   distances fit comfortably in 64 bits. Both are linear in RGB, so a pixel
   that is a blend of ink and backing sits on the segment between their two
   chroma points. */
static void chroma_of(int r, int g, int b, int64_t* cb, int64_t* cr) {
    *cb =  500000LL * b - 168736LL * r - 331264LL * g;
    *cr =  500000LL * r - 418688LL * g -  81312LL * b;
}

/* floor(sqrt(v)) for unsigned 64-bit, no floating point. */
static uint64_t isqrt_u64(uint64_t v) {
    uint64_t r = 0, bit = (uint64_t)1 << 62;
    while (bit > v) bit >>= 2;
    while (bit) {
        if (v >= r + bit) {
            v -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}

static uint64_t chroma_dist2(int64_t cb, int64_t cr, int64_t kb, int64_t kr) {
    return (uint64_t)((cb - kb) * (cb - kb) + (cr - kr) * (cr - kr));
}

/* round(num / den) for signed num with den > 0, without truncating toward
   zero (which would bias the unmixed colour dark). */
static int sdiv_round(int64_t num, int den) {
    if (num >= 0) return (int)((num + den / 2) / den);
    return -(int)((-num + den / 2) / den);
}

static int clamp_u8(int v) { return v < 0 ? 0 : v > 255 ? 255 : v; }

typedef struct {
    int key[3];
    int64_t key_cb, key_cr;
    uint64_t lim;      /* chroma distance at which coverage reaches 255 */
} Keyer;

/* tol_permille is the full-ink distance as a fraction of the backing/ink
   distance: 1000 keys only the pure ink colour at full alpha (default),
   lower values key semi-ink colours too. Deriving the threshold from the
   two colours keeps it independent of the palette; no magic constant. */
static int keyer_init(Keyer* k, const int key[3], const int ink[3], int tol_permille) {
    int64_t kb, kr, ib, ir;
    if (tol_permille <= 0 || tol_permille > 1000) return -1;
    chroma_of(key[0], key[1], key[2], &kb, &kr);
    chroma_of(ink[0], ink[1], ink[2], &ib, &ir);
    uint64_t d = isqrt_u64(chroma_dist2(ib, ir, kb, kr));
    if (d == 0) return -1;      /* backing and ink are the same colour */
    k->key[0] = key[0]; k->key[1] = key[1]; k->key[2] = key[2];
    k->key_cb = kb; k->key_cr = kr;
    k->lim = d * (uint64_t)tol_permille / 1000;
    return k->lim == 0 ? -1 : 0;
}

typedef struct { unsigned char r, g, b, a; } RGBA;

typedef struct {
    int w, h;
    RGBA* px;
} Image;

static int image_alloc(Image* im, int w, int h) {
    if (w <= 0 || h <= 0) return -1;
    im->w = w; im->h = h;
    im->px = (RGBA*)calloc((size_t)w * (size_t)h, sizeof(RGBA));
    return im->px ? 0 : -1;
}

static void image_free(Image* im) {
    free(im->px);
    im->px = NULL;
    im->w = im->h = 0;
}

/* Key one pixel. src_a is the coverage the pixel already had (255 for
   opaque art, lower for semi-transparent art); it is multiplied in and
   never raised, so genuinely semi-transparent pixels stay semi-transparent.
   Returns the pre-existing-alpha-scaled coverage in out->a. */
static void key_pixel(const Keyer* k, RGBA in, RGBA* out) {
    int64_t cb, cr;
    int ch[3] = { in.r, in.g, in.b };

    chroma_of(in.r, in.g, in.b, &cb, &cr);
    uint64_t d2 = chroma_dist2(cb, cr, k->key_cb, k->key_cr);

    /* Linear ramp: backing side of the threshold -> 0, pure ink -> 255. */
    uint64_t d = isqrt_u64(d2);
    int ka = d >= k->lim ? 255 : (int)((d * 255 + k->lim / 2) / k->lim);

    out->a = (unsigned char)((ka * in.a + 127) / 255);
    if (out->a == 0) {
        out->r = out->g = out->b = 0;
        return;
    }
    /* Unmix: observed = ka/lim * ink + (1 - ka/lim) * key, per channel.
       Scaling the offset from the key by 255/ka removes the backing tint
       that the antialiased edge carried, leaving straight ink colour. */
    for (int i = 0; i < 3; i++)
        ch[i] = clamp_u8(k->key[i] + sdiv_round((int64_t)(ch[i] - k->key[i]) * 255, ka));
    out->r = (unsigned char)ch[0];
    out->g = (unsigned char)ch[1];
    out->b = (unsigned char)ch[2];
}

/* Pass 1: key every pixel into straight colour plus coverage. */
static void matte_key(const Keyer* k, const Image* src, Image* dst) {
    size_t n = (size_t)src->w * (size_t)src->h;
    for (size_t i = 0; i < n; i++) {
        RGBA out;
        key_pixel(k, src->px[i], &out);
        dst->px[i] = out;
    }
}

/* Separable min filter (square window of side 2r+1, zero outside, so the
   border erodes). Used as the shrink stage: full-alpha pixels next to a
   background pixel lose coverage, cutting away the backing-tinted rim that
   sits inside the silhouette. */
static void min_filter_h(const Image* src, Image* dst, int radius) {
    for (int y = 0; y < src->h; y++) {
        for (int x = 0; x < src->w; x++) {
            int m = 255;
            for (int i = -radius; i <= radius; i++) {
                int xx = x + i;
                if (xx < 0 || xx >= src->w) { m = 0; break; }
                int v = src->px[y * src->w + xx].a;
                if (v < m) m = v;
            }
            RGBA p = src->px[y * src->w + x];
            p.a = (unsigned char)m;
            dst->px[y * src->w + x] = p;
        }
    }
}

static void min_filter_v(const Image* src, Image* dst, int radius) {
    for (int y = 0; y < src->h; y++) {
        for (int x = 0; x < src->w; x++) {
            int m = 255;
            for (int i = -radius; i <= radius; i++) {
                int yy = y + i;
                if (yy < 0 || yy >= src->h) { m = 0; break; }
                int v = src->px[yy * src->w + x].a;
                if (v < m) m = v;
            }
            RGBA p = src->px[y * src->w + x];
            p.a = (unsigned char)m;
            dst->px[y * src->w + x] = p;
        }
    }
}

static int matte_shrink(Image* m, int radius) {
    Image tmp;
    if (radius <= 0) return 0;
    /* An erosion wider than the image would erase everything; the result is
       the same as running with the full size, so just clamp. */
    int cap = m->w > m->h ? m->w : m->h;
    if (radius > cap) radius = cap;
    if (image_alloc(&tmp, m->w, m->h) != 0) return -1;
    min_filter_h(m, &tmp, radius);
    min_filter_v(&tmp, m, radius);
    image_free(&tmp);
    return 0;
}

/* Feather: mean of the coverage over a square window (zero outside), then
   the minimum with the current coverage. It only ever lowers alpha, so it
   cannot invent coverage outside the silhouette; it turns the cut into a
   ramp. The block sums are gathered from an untouched snapshot of the
   coverage, otherwise pixels already feathered would roll into the next
   window. */
static int matte_feather(Image* m, int radius) {
    if (radius <= 0) return 0;
    /* A window wider than the image would overflow the area product and
       adds nothing: the mean is already taken over the whole image. */
    int cap = m->w > m->h ? m->w : m->h;
    if (radius > cap) radius = cap;
    const int64_t area = (int64_t)(2 * radius + 1) * (2 * radius + 1);
    const size_t n = (size_t)m->w * (size_t)m->h;
    int32_t* colsum = (int32_t*)malloc(n * sizeof(int32_t));
    if (!colsum) return -1;

    /* Vertical sliding sums of the original coverage. */
    for (int x = 0; x < m->w; x++) {
        int64_t s = 0;
        for (int y = 0; y <= radius && y < m->h; y++) s += m->px[y * m->w + x].a;
        for (int y = 0; y < m->h; y++) {
            colsum[y * m->w + x] = (int32_t)s;
            int add = y + 1 + radius, drop = y - radius;
            if (add < m->h) s += m->px[add * m->w + x].a;
            if (drop >= 0) s -= m->px[drop * m->w + x].a;
        }
    }
    /* Horizontal sliding sums over colsum -> full block sums. */
    for (int y = 0; y < m->h; y++) {
        int64_t row = 0;
        for (int x = 0; x <= radius && x < m->w; x++) row += colsum[y * m->w + x];
        for (int x = 0; x < m->w; x++) {
            int mean = (int)((row + area / 2) / area);
            RGBA p = m->px[y * m->w + x];
            if (mean < p.a) p.a = (unsigned char)mean;
            m->px[y * m->w + x] = p;
            int add = x + 1 + radius, drop = x - radius;
            if (add < m->w) row += colsum[y * m->w + add];
            if (drop >= 0) row -= colsum[y * m->w + drop];
        }
    }
    free(colsum);
    return 0;
}

/* Full pipeline. src alpha is respected; out holds straight colour + matte. */
static int matte_extract(const Keyer* k, const Image* src, Image* out,
                         int shrink_radius, int feather_radius) {
    if (image_alloc(out, src->w, src->h) != 0) return -1;
    matte_key(k, src, out);
    if (matte_shrink(out, shrink_radius) != 0) {
        image_free(out);
        return -1;
    }
    if (matte_feather(out, feather_radius) != 0) {
        image_free(out);
        return -1;
    }
    return 0;
}

/* Composite a matte over a flat colour, as the sprite would land in frame. */
static void composite_over(const Image* m, const int bg[3], Image* out) {
    size_t n = (size_t)m->w * (size_t)m->h;
    for (size_t i = 0; i < n; i++) {
        int a = m->px[i].a;
        out->px[i].a = 255;
        out->px[i].r = (unsigned char)clamp_u8((a * m->px[i].r + (255 - a) * bg[0] + 127) / 255);
        out->px[i].g = (unsigned char)clamp_u8((a * m->px[i].g + (255 - a) * bg[1] + 127) / 255);
        out->px[i].b = (unsigned char)clamp_u8((a * m->px[i].b + (255 - a) * bg[2] + 127) / 255);
    }
}

/* ----------------------------------------------------------------------- *
 * Self-test
 * ----------------------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

#define INK_R 220
#define INK_G 120
#define INK_B 40
#define KEY_R 0
#define KEY_G 255
#define KEY_B 0

static int max_abs(int a, int b) { return a < b ? b - a : a - b; }

/* A resolved disc rendered on the backing colour, sampled ss x ss times per
   pixel. ref receives the analytic coverage: what a careful manual cut-out
   would produce for the same edge, so the keyer can be compared to it. */
static void render_disc(Image* img, unsigned char* ref, int w, int h, int ss) {
    const double cx = w * 0.5 + 0.13, cy = h * 0.5 - 0.07;
    const double r = (w < h ? w : h) * 0.33;
    const int n = ss * ss;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int q = 0;
            for (int sy = 0; sy < ss; sy++) {
                for (int sx = 0; sx < ss; sx++) {
                    double px = x + (sx + 0.5) / ss, py = y + (sy + 0.5) / ss;
                    double dx = px - cx, dy = py - cy;
                    if (dx * dx + dy * dy <= r * r) q++;
                }
            }
            ref[y * w + x] = (unsigned char)((q * 255 + n / 2) / n);
            RGBA* p = &img->px[y * w + x];
            p->r = (unsigned char)((q * INK_R + (n - q) * KEY_R + n / 2) / n);
            p->g = (unsigned char)((q * INK_G + (n - q) * KEY_G + n / 2) / n);
            p->b = (unsigned char)((q * INK_B + (n - q) * KEY_B + n / 2) / n);
            p->a = 255;
        }
    }
}

/* Backing contamination of a straight colour: solve c = ink + beta *
   (key - ink). beta is the fraction of backing still mixed in, so a
   colour fringe shows up as beta > 0 and a clean ink colour as beta = 0. */
static double backing_beta(RGBA c) {
    const double v[3] = { c.r - INK_R, c.g - INK_G, c.b - INK_B };
    const double k[3] = { KEY_R - INK_R, KEY_G - INK_G, KEY_B - INK_B };
    double vk = 0, kk = 0;
    for (int i = 0; i < 3; i++) { vk += v[i] * k[i]; kk += k[i] * k[i]; }
    return vk / kk;
}

static int same_color(RGBA a, RGBA b) {
    return a.r == b.r && a.g == b.g && a.b == b.b;
}

/* True when every pixel in the (2r+1)^2 window around (x,y) is opaque, so
   an edge operation with that reach cannot touch the centre pixel. */
static int window_opaque(const Image* m, int x, int y, int r) {
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++) {
            int yy = y + dy, xx = x + dx;
            if (yy < 0 || yy >= m->h || xx < 0 || xx >= m->w
                || m->px[yy * m->w + xx].a != 255)
                return 0;
        }
    return 1;
}

int main(void) {
    enum { W = 96, H = 96 };
    static Image src, comp;
    static unsigned char ref[W * H];
    const int key[3] = { KEY_R, KEY_G, KEY_B };
    const int ink[3] = { INK_R, INK_G, INK_B };
    const int bgs[3][3] = { { 200, 40, 240 }, { 30, 30, 30 }, { 250, 250, 250 } };

    if (image_alloc(&src, W, H) != 0 || image_alloc(&comp, W, H) != 0) {
        printf("alloc_failed=1\nall_ok=0\n");
        return 1;
    }
    Keyer ky;
    check("keyer_init_ok", keyer_init(&ky, key, ink, 1000) == 0);
    check("keyer_rejects_same_color", keyer_init(&ky, key, key, 1000) == -1);
    check("keyer_rejects_bad_tol", keyer_init(&ky, key, ink, 0) == -1);
    keyer_init(&ky, key, ink, 1000);

    /* --- 1. backing is fully removed, pure ink is untouched ------------ */
    {
        RGBA in = { KEY_R, KEY_G, KEY_B, 255 }, out;
        key_pixel(&ky, in, &out);
        check("pure_bg_alpha_zero", out.a == 0);
        in.r = INK_R; in.g = INK_G; in.b = INK_B;
        key_pixel(&ky, in, &out);
        check("pure_ink_alpha_full", out.a == 255);
        check("pure_ink_color_exact", out.r == INK_R && out.g == INK_G && out.b == INK_B);
    }

    /* --- 2. analytic coverage ramp: alpha tracks it within 1 LSB ------- */
    {
        RGBA out;
        int worst = 0, semi = 0;
        for (int q = 0; q <= 64; q++) {
            RGBA in;
            in.r = (unsigned char)((q * INK_R + (64 - q) * KEY_R + 32) / 64);
            in.g = (unsigned char)((q * INK_G + (64 - q) * KEY_G + 32) / 64);
            in.b = (unsigned char)((q * INK_B + (64 - q) * KEY_B + 32) / 64);
            in.a = 255;
            key_pixel(&ky, in, &out);
            int expect = (q * 255 + 32) / 64;
            int err = max_abs(out.a, expect);
            if (err > worst) worst = err;
            if (q > 0 && q < 64 && out.a > 0 && out.a < 255) semi++;
        }
        check("ramp_alpha_error_le_1", worst <= 1);
        check("ramp_keeps_intermediates", semi >= 10);
    }

    /* --- 3. shrink is an exact erosion and only touches alpha ---------- */
    {
        Image base, shrunk;
        render_disc(&src, ref, W, H, 8);
        matte_extract(&ky, &src, &base, 0, 0);
        matte_extract(&ky, &src, &shrunk, 1, 0);

        int exact = 1, colors_ok = 1, lowered = 0;
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                int m = 255;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int yy = y + dy, xx = x + dx;
                        int v = (yy < 0 || yy >= H || xx < 0 || xx >= W)
                              ? 0 : base.px[yy * W + xx].a;
                        if (v < m) m = v;
                    }
                }
                RGBA s = shrunk.px[y * W + x], b = base.px[y * W + x];
                if (s.a != m) exact = 0;
                if (s.a > b.a) lowered = 1;
                if (!same_color(s, b)) colors_ok = 0;
            }
        }
        check("shrink_matches_exact_erosion", exact);
        check("shrink_never_raises", !lowered);
        check("shrink_leaves_colors", colors_ok);
        image_free(&base);
        image_free(&shrunk);
    }

    /* --- 4. feather is the exact mean, capped at the current alpha ----- */
    {
        Image base, soft;
        render_disc(&src, ref, W, H, 8);
        matte_extract(&ky, &src, &base, 0, 0);
        matte_extract(&ky, &src, &soft, 0, 2);

        const int r = 2, side = 2 * r + 1, area = side * side;
        int exact = 1, colors_ok = 1, raised = 0, added_semi = 0;
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                int64_t sum = 0;
                for (int dy = -r; dy <= r; dy++)
                    for (int dx = -r; dx <= r; dx++) {
                        int yy = y + dy, xx = x + dx;
                        if (yy >= 0 && yy < H && xx >= 0 && xx < W)
                            sum += base.px[yy * W + xx].a;
                    }
                int mean = (int)((sum + area / 2) / area);
                int expect = mean < base.px[y * W + x].a ? mean : base.px[y * W + x].a;
                RGBA s = soft.px[y * W + x], b = base.px[y * W + x];
                if (s.a != expect) exact = 0;
                if (s.a > b.a) raised = 1;
                if (!same_color(s, b)) colors_ok = 0;
                if (s.a > 0 && s.a < 255 && b.a == 255) added_semi++;
            }
        }
        check("feather_matches_exact_mean", exact);
        check("feather_never_raises", !raised);
        check("feather_leaves_colors", colors_ok);
        check("feather_softens_edge", added_semi > 0);
        image_free(&base);
        image_free(&soft);
    }

    /* --- 5. pixel-by-pixel against the manual matte -------------------- */
    {
        Image m;
        int worst_a = 0, solid_mismatch = 0, worst_c = 0, worst_inner = 0;
        int worst_comp = 0;
        render_disc(&src, ref, W, H, 8);
        matte_extract(&ky, &src, &m, 0, 0);

        for (int i = 0; i < W * H; i++) {
            int err = max_abs(m.px[i].a, ref[i]);
            if (err > worst_a) worst_a = err;
            if ((m.px[i].a == 0) != (ref[i] == 0)) solid_mismatch++;
            if (m.px[i].a >= 16 && ref[i] >= 16) {
                int e = max_abs(m.px[i].r, INK_R);
                if (max_abs(m.px[i].g, INK_G) > e) e = max_abs(m.px[i].g, INK_G);
                if (max_abs(m.px[i].b, INK_B) > e) e = max_abs(m.px[i].b, INK_B);
                if (e > worst_c) worst_c = e;
                if (m.px[i].a >= 64 && e > worst_inner) worst_inner = e;
            }
        }
        check("manual_alpha_error_le_1", worst_a <= 1);
        check("manual_solid_coverage_agrees", solid_mismatch == 0);
        check("manual_color_error_le_8", worst_c <= 8);
        check("manual_color_inner_le_2", worst_inner <= 2);

        /* Compose over several backgrounds and compare every pixel with
           the manual composite of the reference matte: the backing colour
           must not leak back in whatever sits behind the sprite. */
        for (int bi = 0; bi < 3; bi++) {
            composite_over(&m, bgs[bi], &comp);
            for (int i = 0; i < W * H; i++) {
                int ea = ref[i];
                int want[3] = {
                    (ea * INK_R + (255 - ea) * bgs[bi][0] + 127) / 255,
                    (ea * INK_G + (255 - ea) * bgs[bi][1] + 127) / 255,
                    (ea * INK_B + (255 - ea) * bgs[bi][2] + 127) / 255,
                };
                int got[3] = { comp.px[i].r, comp.px[i].g, comp.px[i].b };
                for (int c = 0; c < 3; c++) {
                    int e = max_abs(got[c], want[c]);
                    if (e > worst_comp) worst_comp = e;
                }
            }
        }
        check("composite_matches_manual_le_3", worst_comp <= 3);
        image_free(&m);
    }

    /* --- 6. colour fringe count is zero ------------------------------- */
    {
        Image m;
        int fringe_before = 0, fringe_after = 0;
        double worst_beta = 0.0;
        render_disc(&src, ref, W, H, 8);
        matte_extract(&ky, &src, &m, 0, 0);
        for (int i = 0; i < W * H; i++) {
            if (ref[i] >= 16 && backing_beta(src.px[i]) > 0.02) fringe_before++;
            if (m.px[i].a >= 16) {
                double beta = backing_beta(m.px[i]);
                if (beta > worst_beta) worst_beta = beta;
                if (beta > 0.02) fringe_after++;
            }
        }
        check("fringe_present_in_source", fringe_before > 0);
        check("fringe_after_keying_zero", fringe_after == 0);
        check("fringe_residual_below_2pct", worst_beta < 0.02);
        image_free(&m);
    }

    /* --- 7. shrink + feather together: no fringe, interior intact ------ */
    {
        Image base, shrunk, proc;
        int changed = 0, fringe_after = 0, shrink_interior = 1;

        render_disc(&src, ref, W, H, 8);
        matte_extract(&ky, &src, &base, 0, 0);
        matte_extract(&ky, &src, &shrunk, 1, 0);
        matte_extract(&ky, &src, &proc, 1, 2);
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                int i = y * W + x;
                if (window_opaque(&base, x, y, 1)
                    && (shrunk.px[i].a != 255 || !same_color(shrunk.px[i], base.px[i])))
                    shrink_interior = 0;
                if (proc.px[i].a >= 16 && backing_beta(proc.px[i]) > 0.02) fringe_after++;
                if (window_opaque(&shrunk, x, y, 2)
                    && (proc.px[i].a != 255 || !same_color(proc.px[i], base.px[i])))
                    changed++;
            }
        }
        check("edge_pipeline_no_fringe", fringe_after == 0);
        check("shrink_interior_intact", shrink_interior);
        check("feather_interior_intact", changed == 0);
        image_free(&base);
        image_free(&shrunk);
        image_free(&proc);
    }

    /* --- 8. finer input quantisation keeps the matte tight ------------ */
    {
        Image m;
        int worst = 0, mismatched = 0;
        render_disc(&src, ref, W, H, 16);
        matte_extract(&ky, &src, &m, 0, 0);
        for (int i = 0; i < W * H; i++) {
            int err = max_abs(m.px[i].a, ref[i]);
            if (err > worst) worst = err;
            if ((m.px[i].a == 0) != (ref[i] == 0)) mismatched++;
        }
        check("fine_input_alpha_error_le_1", worst <= 1);
        check("fine_input_coverage_agrees", mismatched == 0);
        image_free(&m);
    }

    /* --- 9. genuinely semi-transparent source pixels survive ---------- */
    {
        RGBA in = { INK_R, INK_G, INK_B, 128 }, out;
        key_pixel(&ky, in, &out);
        check("src_semi_alpha_kept", out.a == 128);
        /* exact half ink / half backing, already at 50% source coverage */
        in.r = INK_R / 2; in.g = (INK_G + KEY_G) / 2; in.b = INK_B / 2; in.a = 128;
        key_pixel(&ky, in, &out);
        check("src_semi_alpha_multiplied", out.a == 64);
        check("src_semi_still_semi", out.a > 0 && out.a < 255);
    }

    /* --- 10. runtime is linear in pixel count ------------------------- */
    {
        /* The same scene at three sizes; the per-pixel cost must stay in
           the same ballpark, i.e. total time grows linearly with pixels. */
        static const int sizes[3] = { 128, 256, 512 };
        double per_px[3] = { 0.0, 0.0, 0.0 };
        for (int s = 0; s < 3; s++) {
            int w = sizes[s], h = sizes[s];
            Image a, out;
            unsigned char* r = (unsigned char*)malloc((size_t)w * (size_t)h);
            if (image_alloc(&a, w, h) != 0 || !r) {
                printf("runtime_linear=0\nalloc_failed=1\nall_ok=0\n");
                return 1;
            }
            render_disc(&a, r, w, h, 4);
            free(r);
            /* warm up allocator and caches before timing */
            if (matte_extract(&ky, &a, &out, 1, 2) != 0) { printf("alloc_failed=1\n"); return 1; }
            image_free(&out);
            const int reps = 8;
            clock_t t0 = clock();
            for (int i = 0; i < reps; i++) {
                if (matte_extract(&ky, &a, &out, 1, 2) != 0) { printf("alloc_failed=1\n"); return 1; }
                image_free(&out);
            }
            per_px[s] = (double)(clock() - t0) / CLOCKS_PER_SEC
                      / ((double)reps * (double)w * (double)h);
            image_free(&a);
        }
        double lo = per_px[0], hi = per_px[0];
        for (int s = 1; s < 3; s++) {
            if (per_px[s] < lo) lo = per_px[s];
            if (per_px[s] > hi) hi = per_px[s];
        }
        check("runtime_linear", lo > 0.0 && hi <= lo * 3.0);
        printf("per_pixel_us=%.4f\n", per_px[2] * 1e6);
    }

    image_free(&src);
    image_free(&comp);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
