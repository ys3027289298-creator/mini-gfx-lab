
/* texcomp: BC1-style 4x4 block texture compression simulation.
   Three block modes (solid / opaque 4-colour / 3-colour + alpha),
   per-block-type PSNR reporting, boundary banding check.
   Endpoint quantisation overflow is an error, never truncated.
   Compression disabled -> output is bit-identical to input. */
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEX_W 32
#define TEX_H 32
#define BS 4            /* block edge */
#define SOLID_SPREAD 4  /* max channel spread for MODE_SOLID */

typedef struct { unsigned char r, g, b, a; } Rgba;

typedef enum { MODE_SOLID = 0, MODE_OPAQUE = 1, MODE_ALPHA = 2 } BlockMode;
typedef enum { BT_FLAT = 0, BT_GRADIENT = 1, BT_DETAIL = 2, BT_COUNT } BlockType;

typedef struct {
    unsigned short c0, c1; /* RGB565 endpoints */
    unsigned int indices;  /* 16 x 2-bit, row-major */
} Block;                   /* 8 bytes -> 4 bpp */

/* mode is derived from endpoint ordering, BC1-style:
   c0 > c1 opaque, c0 < c1 alpha, c0 == c1 solid (uniform) */
static BlockMode block_mode(const Block* b) {
    if (b->c0 < b->c1) return MODE_ALPHA;
    if (b->c0 == b->c1) return MODE_SOLID;
    return MODE_OPAQUE;
}

typedef struct {
    int enabled;           /* 0: passthrough, output identical to input */
    int force_mode;        /* -1 = auto per block, else BlockMode */
} CompConfig;

/* pack one channel into `bits`; overflow is an error, never truncated */
static int quant_channel(int v, int bits, unsigned* out) {
    if (v < 0 || v > (1 << bits) - 1) return -1;
    *out = (unsigned)v;
    return 0;
}

static int pack_565(int r5, int g6, int b5, unsigned short* out) {
    unsigned r, g, b;
    if (quant_channel(r5, 5, &r) || quant_channel(g6, 6, &g) ||
        quant_channel(b5, 5, &b))
        return -1;
    *out = (unsigned short)((r << 11) | (g << 5) | b);
    return 0;
}

static int to5(int v8) { return (v8 * 31 + 127) / 255; }
static int to6(int v8) { return (v8 * 63 + 127) / 255; }
static unsigned char from5(int v) { return (unsigned char)((v << 3) | (v >> 2)); }
static unsigned char from6(int v) { return (unsigned char)((v << 2) | (v >> 4)); }

static int rgba_to_565(Rgba c, unsigned short* out) {
    return pack_565(to5(c.r), to6(c.g), to5(c.b), out);
}

static Rgba rgba_from_565(unsigned short v) {
    Rgba c = { from5(v >> 11), from6((v >> 5) & 63), from5(v & 31), 255 };
    return c;
}

/* endpoints from luminance extremes, inset 1/16 toward the middle so
   smooth ramps decode without visible steps at block boundaries */
static void pick_endpoints(const Rgba px[16], Rgba* hi, Rgba* lo) {
    int imn = 0, imx = 0, mn = INT_MAX, mx = INT_MIN;
    for (int i = 0; i < 16; i++) {
        int v = px[i].r + px[i].g + px[i].b;
        if (v < mn) { mn = v; imn = i; }
        if (v > mx) { mx = v; imx = i; }
    }
    Rgba a = px[imx], b = px[imn];
    a.r = (unsigned char)(a.r - ((int)a.r - b.r) / 16);
    a.g = (unsigned char)(a.g - ((int)a.g - b.g) / 16);
    a.b = (unsigned char)(a.b - ((int)a.b - b.b) / 16);
    b.r = (unsigned char)(b.r + ((int)a.r - b.r) / 16);
    b.g = (unsigned char)(b.g + ((int)a.g - b.g) / 16);
    b.b = (unsigned char)(b.b + ((int)a.b - b.b) / 16);
    *hi = a;
    *lo = b;
}

static Rgba mixc(Rgba x, Rgba y, int wx, int wy, int div) {
    Rgba c = { (unsigned char)((x.r * wx + y.r * wy) / div),
               (unsigned char)((x.g * wx + y.g * wy) / div),
               (unsigned char)((x.b * wx + y.b * wy) / div), 255 };
    return c;
}

static void build_palette(BlockMode mode, Rgba e0, Rgba e1, Rgba pal[4]) {
    pal[0] = e0;
    pal[1] = e1;
    if (mode == MODE_ALPHA) {
        pal[2] = mixc(e0, e1, 1, 1, 2);
        pal[3] = (Rgba){ 0, 0, 0, 0 };
    } else {
        pal[2] = mixc(e0, e1, 2, 1, 3);
        pal[3] = mixc(e0, e1, 1, 2, 3);
    }
}

static long cdist(Rgba a, Rgba b) {
    long dr = (long)a.r - b.r, dg = (long)a.g - b.g, db = (long)a.b - b.b;
    return dr * dr + dg * dg + db * db;
}

static int encode_block(const Rgba px[16], BlockMode mode, Block* out) {
    Rgba e0, e1, pal[4];
    unsigned short c0, c1;
    unsigned idx = 0;
    if (mode == MODE_SOLID) {
        int r = 0, g = 0, b = 0;
        for (int i = 0; i < 16; i++) { r += px[i].r; g += px[i].g; b += px[i].b; }
        e0 = e1 = (Rgba){ (unsigned char)((r + 8) / 16),
                          (unsigned char)((g + 8) / 16),
                          (unsigned char)((b + 8) / 16), 255 };
    } else {
        pick_endpoints(px, &e0, &e1);
    }
    if (rgba_to_565(e0, &c0) || rgba_to_565(e1, &c1)) return -1;
    if (mode == MODE_OPAQUE && c0 < c1) { unsigned short t = c0; c0 = c1; c1 = t; }
    if (mode == MODE_ALPHA) {
        if (c0 > c1) { unsigned short t = c0; c0 = c1; c1 = t; }
        if (c0 == c1) {           /* keep c0 < c1 so the block stays alpha */
            if (c1 < 0xFFFF) c1++;
            else c0--;
        }
    }
    if (mode == MODE_SOLID) c1 = c0;
    build_palette(mode, rgba_from_565(c0), rgba_from_565(c1), pal);
    for (int i = 0; i < 16; i++) {
        int best = 0, n = mode == MODE_ALPHA ? 3 : mode == MODE_SOLID ? 1 : 4;
        long bd = LONG_MAX;
        if (mode == MODE_ALPHA && px[i].a < 128) {
            best = 3;
        } else {
            for (int p = 0; p < n; p++) {
                long d = cdist(px[i], pal[p]);
                if (d < bd) { bd = d; best = p; }
            }
        }
        idx |= (unsigned)best << (2 * i);
    }
    out->c0 = c0;
    out->c1 = c1;
    out->indices = idx;
    return 0;
}

static void decode_block(const Block* blk, Rgba px[16]) {
    Rgba pal[4];
    BlockMode mode = block_mode(blk);
    build_palette(mode, rgba_from_565(blk->c0), rgba_from_565(blk->c1), pal);
    for (int i = 0; i < 16; i++)
        px[i] = pal[(blk->indices >> (2 * i)) & 3];
}

static int block_spread(const Rgba px[16]) {
    int spread = 0;
    for (int c = 0; c < 3; c++) {
        int mn = 255, mx = 0;
        for (int i = 0; i < 16; i++) {
            int v = c == 0 ? px[i].r : c == 1 ? px[i].g : px[i].b;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        if (mx - mn > spread) spread = mx - mn;
    }
    return spread;
}

static BlockMode auto_mode(const Rgba px[16]) {
    for (int i = 0; i < 16; i++)
        if (px[i].a < 128) return MODE_ALPHA;
    return block_spread(px) <= SOLID_SPREAD ? MODE_SOLID : MODE_OPAQUE;
}

static BlockType classify(const Rgba px[16]) {
    int s = block_spread(px);
    if (s < 8) return BT_FLAT;
    if (s > 48) return BT_DETAIL;
    return BT_GRADIENT;
}

static int compress_texture(const Rgba* src, int w, int h, const CompConfig* cfg,
                            Block* blocks) {
    int n = 0;
    if (w % BS || h % BS) return -1;
    for (int by = 0; by < h; by += BS)
        for (int bx = 0; bx < w; bx += BS) {
            Rgba px[16];
            for (int y = 0; y < BS; y++)
                for (int x = 0; x < BS; x++)
                    px[y * BS + x] = src[(by + y) * w + bx + x];
            BlockMode mode = cfg->force_mode >= 0 ? (BlockMode)cfg->force_mode
                                                  : auto_mode(px);
            if (encode_block(px, mode, &blocks[n++])) return -1;
        }
    return n;
}

static void decompress_texture(const Block* blocks, int w, int h, Rgba* dst) {
    int n = 0;
    for (int by = 0; by < h; by += BS)
        for (int bx = 0; bx < w; bx += BS) {
            Rgba px[16];
            decode_block(&blocks[n++], px);
            for (int y = 0; y < BS; y++)
                for (int x = 0; x < BS; x++)
                    dst[(by + y) * w + bx + x] = px[y * BS + x];
        }
}

/* full pipeline; disabled -> dst is a bit-identical copy of src */
static int process(const Rgba* src, int w, int h, const CompConfig* cfg,
                   Rgba* dst, Block* blocks) {
    if (!cfg->enabled) {
        memcpy(dst, src, (size_t)w * h * sizeof(Rgba));
        return 0;
    }
    int n = compress_texture(src, w, h, cfg, blocks);
    if (n < 0) return -1;
    decompress_texture(blocks, w, h, dst);
    return n;
}

/* largest channel jump between horizontally adjacent pixels,
   split into block-interior vs block-boundary crossings */
static void boundary_stats(const Rgba* img, int w, int h, int* interior_max,
                           int* boundary_max) {
    *interior_max = *boundary_max = 0;
    for (int y = 0; y < h; y++)
        for (int x = 1; x < w; x++) {
            Rgba a = img[y * w + x - 1], b = img[y * w + x];
            int d = 0;
            int ch[3][2] = { { a.r, b.r }, { a.g, b.g }, { a.b, b.b } };
            for (int c = 0; c < 3; c++) {
                int dd = ch[c][0] - ch[c][1];
                if (dd < 0) dd = -dd;
                if (dd > d) d = dd;
            }
            if (x % BS == 0) {
                if (d > *boundary_max) *boundary_max = d;
            } else {
                if (d > *interior_max) *interior_max = d;
            }
        }
}

static double psnr(double mse) {
    return mse <= 0.0 ? INFINITY : 10.0 * log10(255.0 * 255.0 / mse);
}

/* per-block-type PSNR of dst vs src, plus overall and max pixel error */
static void report_psnr(const Rgba* src, const Rgba* dst, int w, int h) {
    double sse[BT_COUNT] = { 0 }, total_sse = 0;
    long cnt[BT_COUNT] = { 0 };
    int maxerr = 0;
    const char* names[BT_COUNT] = { "flat", "gradient", "detail" };
    for (int by = 0; by < h; by += BS)
        for (int bx = 0; bx < w; bx += BS) {
            Rgba px[16];
            for (int y = 0; y < BS; y++)
                for (int x = 0; x < BS; x++)
                    px[y * BS + x] = src[(by + y) * w + bx + x];
            BlockType t = classify(px);
            for (int y = 0; y < BS; y++)
                for (int x = 0; x < BS; x++) {
                    Rgba a = src[(by + y) * w + bx + x];
                    Rgba b = dst[(by + y) * w + bx + x];
                    int ch[3][2] = { { a.r, b.r }, { a.g, b.g }, { a.b, b.b } };
                    for (int c = 0; c < 3; c++) {
                        int d = ch[c][0] - ch[c][1];
                        sse[t] += (double)d * d;
                        total_sse += (double)d * d;
                        cnt[t]++;
                        if (d < 0) d = -d;
                        if (d > maxerr) maxerr = d;
                    }
                }
        }
    for (int t = 0; t < BT_COUNT; t++)
        if (cnt[t])
            printf("psnr_%s=%.2f\n", names[t], psnr(sse[t] / cnt[t]));
    printf("psnr_all=%.2f\n", psnr(total_sse / (3.0 * w * h)));
    printf("max_err=%d\n", maxerr);
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned lcg(unsigned* s) {
    *s = *s * 1664525u + 1013904223u;
    return *s >> 16;
}

int main(void) {
    static Rgba src[TEX_W * TEX_H], dst[TEX_W * TEX_H];
    static Block blocks[(TEX_W / BS) * (TEX_H / BS)];
    unsigned short pk;

    /* endpoint quantisation overflow: error, never truncation */
    check("quant_r_overflow", pack_565(32, 0, 0, &pk) == -1);
    check("quant_g_overflow", pack_565(0, 64, 0, &pk) == -1);
    check("quant_negative", pack_565(-1, 0, 0, &pk) == -1);
    check("quant_max_ok", pack_565(31, 63, 31, &pk) == 0 && pk == 0xFFFF);

    /* test texture: flat / smooth gradient / noise regions */
    unsigned seed = 7;
    for (int y = 0; y < TEX_H; y++)
        for (int x = 0; x < TEX_W; x++) {
            Rgba c = { 0, 0, 0, 255 };
            if (x < 16 && y < 16)
                c = (Rgba){ 200, 120, 40, 255 };                       /* flat */
            else if (x >= 16 && y < 16)
                c.r = c.g = c.b = (unsigned char)((x - 16) * 8);       /* ramp */
            else
                c.r = (unsigned char)lcg(&seed),
                c.g = (unsigned char)lcg(&seed),
                c.b = (unsigned char)lcg(&seed);                       /* noise */
            src[y * TEX_W + x] = c;
        }

    /* compression disabled: output identical to input */
    CompConfig off = { 0, -1 };
    process(src, TEX_W, TEX_H, &off, dst, blocks);
    check("passthrough_identical", memcmp(src, dst, sizeof(src)) == 0);

    /* auto mode selection */
    CompConfig on = { 1, -1 };
    int n = process(src, TEX_W, TEX_H, &on, dst, blocks);
    check("compress_ok", n == (TEX_W / BS) * (TEX_H / BS));
    check("mode_auto_solid", block_mode(&blocks[0]) == MODE_SOLID);
    check("mode_auto_opaque", block_mode(&blocks[4]) == MODE_OPAQUE);
    check("size_4bpp", sizeof(blocks) == (size_t)TEX_W * TEX_H / 2);

    /* forced modes */
    CompConfig fsolid = { 1, MODE_SOLID };
    process(src, TEX_W, TEX_H, &fsolid, dst, blocks);
    check("mode_forced_solid", block_mode(&blocks[4]) == MODE_SOLID);

    /* alpha mode: transparent pixels stay transparent */
    Rgba apx[16];
    for (int i = 0; i < 16; i++) apx[i] = (Rgba){ 255, 0, 0, 255 };
    apx[3].a = apx[7].a = 0;
    Block ab;
    Rgba apx_out[16];
    check("encode_alpha", encode_block(apx, MODE_ALPHA, &ab) == 0 &&
                          block_mode(&ab) == MODE_ALPHA);
    decode_block(&ab, apx_out);
    check("alpha_punchthrough", apx_out[3].a == 0 && apx_out[7].a == 0 &&
                                apx_out[0].a == 255);

    /* smooth ramp: no banding step at block boundaries */
    static Rgba ramp[TEX_W * TEX_H], ramp_out[TEX_W * TEX_H];
    for (int y = 0; y < TEX_H; y++)
        for (int x = 0; x < TEX_W; x++)
            ramp[y * TEX_W + x] =
                (Rgba){ (unsigned char)(x * 3), (unsigned char)(x * 3),
                        (unsigned char)(x * 3), 255 };
    process(ramp, TEX_W, TEX_H, &on, ramp_out, blocks);
    int interior = 0, boundary = 0;
    boundary_stats(ramp_out, TEX_W, TEX_H, &interior, &boundary);
    printf("ramp_interior_step=%d\n", interior);
    printf("ramp_boundary_step=%d\n", boundary);
    check("no_boundary_banding", boundary <= interior + 2);

    /* per-block-type PSNR against the original texture */
    process(src, TEX_W, TEX_H, &on, dst, blocks);
    report_psnr(src, dst, TEX_W, TEX_H);

    printf("failures=%d\n", failures);
    return failures ? 1 : 0;
}
