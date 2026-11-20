/* iconexport: multi-resolution icon export.
 *
 * One icon, many sizes. Baseline ("current") behavior resamples the
 * original directly per size. The optional ladder export downsamples
 * step by step (mip-style), applies unsharp-mask compensation per rung,
 * and packs all sizes into one bundle. Edge sharpness is reported per
 * size tier against a natively rendered reference. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SRC_SIZE 256
#define MAX_RUNGS 16
#define MAX_EXPORT 8
#define BUNDLE_MAGIC "ICXB"
#define UNSHARP_AMOUNT 0.15

typedef struct { int w, h; unsigned char* px; } Image; /* RGBA8, row-major */

typedef struct {
    int ladder_enabled; /* 0 keeps current behavior: direct resample per size */
    int levels;         /* ladder rungs; rung i has side src/2^i              */
    int sharpen;        /* unsharp-mask compensation after each downsample    */
} ExportOpts;

typedef struct { unsigned char* data; size_t len; } Bundle;

static Image img_new(int w, int h) {
    Image im;
    im.w = w;
    im.h = h;
    im.px = calloc((size_t)w * h * 4, 1);
    return im;
}

static void img_free(Image* im) { free(im->px); im->px = NULL; }

static Image img_copy(const Image* src) {
    Image im = img_new(src->w, src->h);
    memcpy(im.px, src->px, (size_t)src->w * src->h * 4);
    return im;
}

static int img_equal(const Image* a, const Image* b) {
    return a->w == b->w && a->h == b->h &&
           memcmp(a->px, b->px, (size_t)a->w * a->h * 4) == 0;
}

/* the icon, defined procedurally so a native-resolution reference exists */
static double icon_sample(double u, double v) {
    double dx = u - 0.5, dy = v - 0.5;
    if (u > 0.12 && u < 0.38 && v > 0.62 && v < 0.88) return 250.0; /* bright tab */
    if (dx * dx + dy * dy < 0.35 * 0.35) return 30.0;               /* dark disc */
    return 220.0;                                                   /* background */
}

static void render_icon(Image* img, int ss) {
    for (int y = 0; y < img->h; y++)
        for (int x = 0; x < img->w; x++) {
            double acc = 0;
            for (int sy = 0; sy < ss; sy++)
                for (int sx = 0; sx < ss; sx++)
                    acc += icon_sample((x + (sx + 0.5) / ss) / img->w,
                                       (y + (sy + 0.5) / ss) / img->h);
            unsigned char g = (unsigned char)(acc / (ss * ss) + 0.5);
            unsigned char* p = img->px + ((size_t)y * img->w + x) * 4;
            p[0] = p[1] = p[2] = g;
            p[3] = 255;
        }
}

/* fractional box filter, downsample only */
static void resample_area(const Image* src, Image* dst) {
    double sx = (double)src->w / dst->w;
    double sy = (double)src->h / dst->h;
    for (int y = 0; y < dst->h; y++) {
        double y0 = y * sy, y1 = y0 + sy;
        for (int x = 0; x < dst->w; x++) {
            double x0 = x * sx, x1 = x0 + sx;
            double acc[4] = {0, 0, 0, 0}, area = 0;
            for (int j = (int)y0; j < (int)ceil(y1); j++) {
                double wy = fmin(j + 1.0, y1) - fmax((double)j, y0);
                if (wy <= 0 || j >= src->h) continue;
                for (int i = (int)x0; i < (int)ceil(x1); i++) {
                    double wx = fmin(i + 1.0, x1) - fmax((double)i, x0);
                    if (wx <= 0 || i >= src->w) continue;
                    double w = wx * wy;
                    const unsigned char* p = src->px + ((size_t)j * src->w + i) * 4;
                    for (int c = 0; c < 4; c++) acc[c] += p[c] * w;
                    area += w;
                }
            }
            unsigned char* q = dst->px + ((size_t)y * dst->w + x) * 4;
            for (int c = 0; c < 4; c++)
                q[c] = (unsigned char)fmin(255.0, fmax(0.0, acc[c] / area + 0.5));
        }
    }
}

static void downsample_half(const Image* src, Image* dst) {
    for (int y = 0; y < dst->h; y++)
        for (int x = 0; x < dst->w; x++) {
            const unsigned char* p = src->px + ((size_t)(2 * y) * src->w + 2 * x) * 4;
            unsigned char* q = dst->px + ((size_t)y * dst->w + x) * 4;
            for (int c = 0; c < 4; c++) {
                int s = p[c] + p[4 + c] + p[src->w * 4 + c] + p[src->w * 4 + 4 + c];
                q[c] = (unsigned char)((s + 2) / 4);
            }
        }
}

/* boost = orig + amount * (orig - 3x3 box blur) */
static void unsharp(Image* img, double amount) {
    int w = img->w, h = img->h;
    size_t bytes = (size_t)w * h * 4;
    unsigned char* blur = malloc(bytes);
    unsigned char* orig = malloc(bytes);
    memcpy(orig, img->px, bytes);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            for (int c = 0; c < 4; c++) {
                int s = 0, n = 0;
                for (int dy = -1; dy <= 1; dy++)
                    for (int dx = -1; dx <= 1; dx++) {
                        int j = y + dy, i = x + dx;
                        if (i < 0 || j < 0 || i >= w || j >= h) continue;
                        s += orig[((size_t)j * w + i) * 4 + c];
                        n++;
                    }
                blur[((size_t)y * w + x) * 4 + c] = (unsigned char)(s / n);
            }
    for (size_t i = 0; i < bytes; i++) {
        int v = (int)(orig[i] + amount * ((int)orig[i] - (int)blur[i]) + 0.5);
        img->px[i] = (unsigned char)(v < 0 ? 0 : v > 255 ? 255 : v);
    }
    free(blur);
    free(orig);
}

/* mip-style ladder: rung 0 is the source, each next rung halves the size */
static int ladder_build(const Image* src, int levels, int sharpen, Image* rungs) {
    int mind = src->w < src->h ? src->w : src->h;
    if (levels < 1 || levels > MAX_RUNGS) return -1;
    if (levels > mind) return -1;               /* more rungs than source pixels */
    if ((mind >> (levels - 1)) < 1) return -1;  /* smallest rung would be empty  */
    if (levels > 1 && ((src->w & 1) || (src->h & 1))) return -1;
    rungs[0] = img_copy(src);
    for (int i = 1; i < levels; i++) {
        rungs[i] = img_new(rungs[i - 1].w / 2, rungs[i - 1].h / 2);
        downsample_half(&rungs[i - 1], &rungs[i]);
        if (sharpen) unsharp(&rungs[i], UNSHARP_AMOUNT);
    }
    return 0;
}

static int export_sizes(const Image* src, const int* sizes, int n,
                        const ExportOpts* opts, Image* out) {
    if (!src->px || n < 1 || n > MAX_EXPORT) return -1;
    for (int i = 0; i < n; i++)
        if (sizes[i] < 1 || sizes[i] > src->w || sizes[i] > src->h) return -1;
    if (!opts->ladder_enabled) {
        for (int i = 0; i < n; i++) {
            out[i] = img_new(sizes[i], sizes[i]);
            resample_area(src, &out[i]);
        }
        return 0;
    }
    Image rungs[MAX_RUNGS];
    memset(rungs, 0, sizeof(rungs));
    if (ladder_build(src, opts->levels, opts->sharpen, rungs) != 0) return -1;
    for (int i = 0; i < n; i++) {
        int hit = -1;
        for (int k = 0; k < opts->levels; k++)
            if (rungs[k].w == sizes[i] && rungs[k].h == sizes[i]) { hit = k; break; }
        if (hit >= 0) {
            out[i] = img_copy(&rungs[hit]);
        } else { /* size not on the ladder: direct resample fallback */
            out[i] = img_new(sizes[i], sizes[i]);
            resample_area(src, &out[i]);
        }
    }
    for (int k = 0; k < opts->levels; k++) img_free(&rungs[k]);
    return 0;
}

static void put_u32(unsigned char* p, unsigned v) {
    p[0] = v & 0xff;
    p[1] = (v >> 8) & 0xff;
    p[2] = (v >> 16) & 0xff;
    p[3] = (v >> 24) & 0xff;
}

static unsigned get_u32(const unsigned char* p) {
    return p[0] | ((unsigned)p[1] << 8) | ((unsigned)p[2] << 16) | ((unsigned)p[3] << 24);
}

static unsigned fnv1a(const unsigned char* p, size_t n) {
    unsigned h = 2166136261u;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

/* one blob: magic, count, per-image header + pixels, trailing checksum */
static int bundle_pack(const Image* imgs, int n, Bundle* out) {
    size_t len = 8 + 4;
    for (int i = 0; i < n; i++) len += 8 + (size_t)imgs[i].w * imgs[i].h * 4;
    unsigned char* d = malloc(len);
    unsigned char* p = d;
    memcpy(p, BUNDLE_MAGIC, 4);
    p += 4;
    put_u32(p, (unsigned)n);
    p += 4;
    for (int i = 0; i < n; i++) {
        size_t bytes = (size_t)imgs[i].w * imgs[i].h * 4;
        put_u32(p, (unsigned)imgs[i].w);
        p += 4;
        put_u32(p, (unsigned)imgs[i].h);
        p += 4;
        memcpy(p, imgs[i].px, bytes);
        p += bytes;
    }
    put_u32(p, fnv1a(d, len - 4));
    out->data = d;
    out->len = len;
    return 0;
}

static int bundle_unpack(const unsigned char* d, size_t len, Image* out, int cap) {
    if (len < 12 || memcmp(d, BUNDLE_MAGIC, 4) != 0) return -1;
    if (fnv1a(d, len - 4) != get_u32(d + len - 4)) return -1;
    unsigned n = get_u32(d + 4);
    if (n > (unsigned)cap) return -1;
    const unsigned char* p = d + 8;
    const unsigned char* end = d + len - 4;
    for (unsigned i = 0; i < n; i++) {
        if (p + 8 > end) return -1;
        unsigned w = get_u32(p), h = get_u32(p + 4);
        p += 8;
        if (w == 0 || h == 0 || w > 4096 || h > 4096) return -1;
        size_t bytes = (size_t)w * h * 4;
        if (p + bytes > end) return -1;
        out[i] = img_new((int)w, (int)h);
        memcpy(out[i].px, p, bytes);
        p += bytes;
    }
    if (p != end) return -1;
    return (int)n;
}

static int cmp_double(const void* a, const void* b) {
    double x = *(const double*)a, y = *(const double*)b;
    return (x > y) - (x < y);
}

/* mean of the top 1% luminance gradients: high = crisp edge, low = blurry */
static double edge_sharpness(const Image* img) {
    int w = img->w, h = img->h;
    size_t n = (size_t)w * h;
    double* g = malloc(n * sizeof(double));
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            const unsigned char* p = img->px + ((size_t)y * w + x) * 4;
            double gx = 0, gy = 0;
            if (x + 1 < w) gx = fabs((double)p[0] - (double)p[4]);
            if (y + 1 < h) gy = fabs((double)p[0] - (double)p[w * 4]);
            g[(size_t)y * w + x] = gx > gy ? gx : gy;
        }
    qsort(g, n, sizeof(double), cmp_double);
    size_t k = n / 100;
    if (k < 1) k = 1;
    double s = 0;
    for (size_t i = n - k; i < n; i++) s += g[i];
    free(g);
    return s / k;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Image src = img_new(SRC_SIZE, SRC_SIZE);
    render_icon(&src, 4);

    int sizes[] = {128, 64, 32, 16};
    int n = 4;

    /* current behavior: direct resample per size */
    Image direct[MAX_EXPORT] = {0};
    for (int i = 0; i < n; i++) {
        direct[i] = img_new(sizes[i], sizes[i]);
        resample_area(&src, &direct[i]);
    }

    /* export disabled: output must match current behavior exactly */
    ExportOpts off = {0, 0, 0};
    Image base[MAX_EXPORT] = {0};
    check("disabled_export_ok", export_sizes(&src, sizes, n, &off, base) == 0);
    int same = 1;
    for (int i = 0; i < n; i++) same &= img_equal(&base[i], &direct[i]);
    check("disabled_matches_current", same);

    /* ladder validation */
    ExportOpts zero_lv = {1, 0, 1};
    ExportOpts huge_lv = {1, SRC_SIZE + 1, 1};
    ExportOpts empty_rung = {1, 10, 1}; /* 256 >> 9 == 0 */
    Image tmp[MAX_EXPORT] = {0};
    check("reject_zero_levels", export_sizes(&src, sizes, n, &zero_lv, tmp) == -1);
    check("reject_levels_over_size", export_sizes(&src, sizes, n, &huge_lv, tmp) == -1);
    check("reject_empty_rung", export_sizes(&src, sizes, n, &empty_rung, tmp) == -1);
    int bad_size[] = {0};
    ExportOpts on = {1, 5, 1};
    check("reject_bad_size", export_sizes(&src, bad_size, 1, &on, tmp) == -1);

    /* ladder export with sharpening compensation */
    Image lad[MAX_EXPORT] = {0};
    check("ladder_export_ok", export_sizes(&src, sizes, n, &on, lad) == 0);
    check("ladder_path_active", !img_equal(&lad[n - 1], &direct[n - 1]));

    /* size not on the ladder falls back to direct resample */
    int odd[] = {100};
    Image fb[1] = {0}, fb_ref = img_new(100, 100);
    resample_area(&src, &fb_ref);
    check("non_rung_fallback",
          export_sizes(&src, odd, 1, &on, fb) == 0 && img_equal(&fb[0], &fb_ref));

    /* bundle: pack all tiers into one blob, unpack, compare */
    Bundle b = {0};
    check("pack_ok", bundle_pack(lad, n, &b) == 0 && b.len > 0);
    Image back[MAX_EXPORT] = {0};
    check("unpack_count", bundle_unpack(b.data, b.len, back, MAX_EXPORT) == n);
    int rt = 1;
    for (int i = 0; i < n; i++) rt &= img_equal(&lad[i], &back[i]);
    check("bundle_roundtrip", rt);
    check("reject_bad_magic", bundle_unpack(direct[0].px, 32, tmp, MAX_EXPORT) == -1);
    b.data[10] ^= 0xff;
    check("reject_corrupt", bundle_unpack(b.data, b.len, tmp, MAX_EXPORT) == -1);
    b.data[10] ^= 0xff;

    /* edge sharpness per size tier vs native-resolution reference */
    Image lad_raw[MAX_EXPORT] = {0};
    ExportOpts no_sharp = {1, 5, 0};
    export_sizes(&src, sizes, n, &no_sharp, lad_raw);
    int small_ok = 1, sharpen_helps = 1;
    for (int i = 0; i < n; i++) {
        Image ref = img_new(sizes[i], sizes[i]);
        render_icon(&ref, 8);
        double sl = edge_sharpness(&lad[i]);
        double sr = edge_sharpness(&ref);
        double s0 = edge_sharpness(&lad_raw[i]);
        printf("sharpness size=%d ladder=%.1f ref=%.1f ratio=%.3f\n",
               sizes[i], sl, sr, sr > 0 ? sl / sr : 0);
        if (sizes[i] <= 32 && sl < sr * 0.80) small_ok = 0; /* no blurry small tiers */
        if (sl <= s0) sharpen_helps = 0;
        img_free(&ref);
    }
    check("small_tiers_no_blur", small_ok);
    check("sharpen_compensates", sharpen_helps);

    for (int i = 0; i < n; i++) {
        img_free(&direct[i]);
        img_free(&base[i]);
        img_free(&lad[i]);
        img_free(&lad_raw[i]);
        img_free(&back[i]);
    }
    img_free(&fb[0]);
    img_free(&fb_ref);
    img_free(&src);
    free(b.data);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
