
/* blockpack: 4x4 block compression for an RGBA8 icon atlas.
 *
 * The source atlas keeps its raw RGBA8 pixels. Packing works on 4x4 blocks:
 *   - BP_TAG_BC1: two RGB565 endpoints + sixteen 2-bit palette indices (8 B),
 *     a DXT1/BC1-style four-color lerp, alpha implied opaque;
 *   - BP_TAG_RAW: the original sixteen RGBA8 texels lossless (64 B).
 * A block is stored BC1 only when its mean squared error against the raw
 * texels is at or below the caller's quality threshold; otherwise it falls
 * back to RAW. Blocks past the atlas edge are padded by edge-clamp
 * replication (no foreign colors, so no visible seam), and only real texels
 * are ever written back on decode. Every stream write/read goes through a
 * bounds-checked cursor, so a short buffer fails with -1 and never overruns.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#define BP_BLOCK 4
#define BP_HEADER_BYTES 16u
#define BP_TAG_RAW 0u
#define BP_TAG_BC1 1u
#define BP_BC1_BYTES 8u
#define BP_RAW_BYTES 64u
#define BP_MAX_DIM 512

typedef struct { uint8_t r, g, b, a; } RGBA8;

/* Forward-only, bounds-checked writer. On overflow it flags and refuses all
 * further writes; pb->p is never touched past pb->cap. */
typedef struct {
    uint8_t* p;
    size_t cap;
    size_t pos;
    int overflow;
} PackBuf;

typedef struct {
    const uint8_t* p;
    size_t cap;
    size_t pos;
    int err;
} ReadBuf;

typedef struct {
    int blocks;
    int bc1_blocks;
    int raw_blocks;
} BlockCounts;

typedef struct {
    size_t raw_bytes;       /* w * h * 4, the uncompressed atlas */
    size_t packed_bytes;    /* full stream, header included */
    size_t header_bytes;
    double ratio;           /* packed / raw */
    double bits_per_pixel;
    double max_diff;        /* largest single-channel absolute error */
    double mae;             /* mean absolute error per channel */
    double mse;             /* mean squared error per channel */
    double psnr_db;         /* INFINITY when lossless */
} PackMetrics;

/* ------------------------------------------------------------------ */
/* checked cursors                                                     */
/* ------------------------------------------------------------------ */

static int pb_write(PackBuf* pb, const void* src, size_t n) {
    if (pb->overflow) return -1;
    if (n > pb->cap - pb->pos) { pb->overflow = 1; return -1; }
    memcpy(pb->p + pb->pos, src, n);
    pb->pos += n;
    return 0;
}

static int pb_put8(PackBuf* pb, uint8_t v) { return pb_write(pb, &v, 1); }

static int pb_put16le(PackBuf* pb, uint16_t v) {
    uint8_t b[2] = { (uint8_t)(v & 0xff), (uint8_t)(v >> 8) };
    return pb_write(pb, b, 2);
}

static int pb_put32le(PackBuf* pb, uint32_t v) {
    uint8_t b[4] = {
        (uint8_t)(v & 0xff), (uint8_t)((v >> 8) & 0xff),
        (uint8_t)((v >> 16) & 0xff), (uint8_t)((v >> 24) & 0xff)
    };
    return pb_write(pb, b, 4);
}

static int rd_read(ReadBuf* rb, void* dst, size_t n) {
    if (rb->err) return -1;
    if (n > rb->cap - rb->pos) { rb->err = 1; return -1; }
    memcpy(dst, rb->p + rb->pos, n);
    rb->pos += n;
    return 0;
}

static int rd_u8(ReadBuf* rb, uint8_t* v) { return rd_read(rb, v, 1); }

static int rd_u16le(ReadBuf* rb, uint16_t* v) {
    uint8_t b[2];
    if (rd_read(rb, b, 2) != 0) return -1;
    *v = (uint16_t)(b[0] | ((uint16_t)b[1] << 8));
    return 0;
}

static int rd_u32le(ReadBuf* rb, uint32_t* v) {
    uint8_t b[4];
    if (rd_read(rb, b, 4) != 0) return -1;
    *v = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
         ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    return 0;
}

/* ------------------------------------------------------------------ */
/* RGB565 endpoints and the four BC1 palette colors                    */
/* ------------------------------------------------------------------ */

static uint16_t rgb_to_565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

static RGBA8 rgb565_to_rgba(uint16_t v) {
    uint8_t r5 = (uint8_t)((v >> 11) & 0x1f);
    uint8_t g6 = (uint8_t)((v >> 5) & 0x3f);
    uint8_t b5 = (uint8_t)(v & 0x1f);
    RGBA8 c = {
        (uint8_t)((r5 << 3) | (r5 >> 2)),
        (uint8_t)((g6 << 2) | (g6 >> 4)),
        (uint8_t)((b5 << 3) | (b5 >> 2)),
        255
    };
    return c;
}

static void bp_decode_palette(uint16_t e0, uint16_t e1, RGBA8 pal[4]) {
    pal[0] = rgb565_to_rgba(e0);
    pal[1] = rgb565_to_rgba(e1);
    for (int i = 0; i < 3; i++) {
        const uint8_t* c0 = &pal[0].r + i;
        const uint8_t* c1 = &pal[1].r + i;
        uint8_t* p2 = &pal[2].r + i;
        uint8_t* p3 = &pal[3].r + i;
        *p2 = (uint8_t)((2 * *c0 + *c1) / 3);
        *p3 = (uint8_t)((*c0 + 2 * *c1) / 3);
    }
    pal[2].a = pal[3].a = 255;
}

/* ------------------------------------------------------------------ */
/* block gathering: out-of-atlas texels replicate the nearest edge     */
/* ------------------------------------------------------------------ */

static RGBA8 block_texel(const RGBA8* src, int w, int h,
                         int bx, int by, int lx, int ly) {
    int x = bx * BP_BLOCK + lx;
    int y = by * BP_BLOCK + ly;
    if (x >= w) x = w - 1;
    if (y >= h) y = h - 1;
    return src[y * w + x];
}

static int color_dist2(RGBA8 a, RGBA8 b) {
    int dr = (int)a.r - (int)b.r;
    int dg = (int)a.g - (int)b.g;
    int db = (int)a.b - (int)b.b;
    int da = (int)a.a - (int)b.a;
    return dr * dr + dg * dg + db * db + da * da;
}

/* Encode one block. Returns the tag chosen, or -1 on stream failure. */
static int encode_block(const RGBA8* src, int w, int h,
                        int bx, int by, double mse_threshold,
                        BlockCounts* counts, PackBuf* pb) {
    RGBA8 tex[16];
    int rw = w - bx * BP_BLOCK; if (rw > BP_BLOCK) rw = BP_BLOCK;
    int rh = h - by * BP_BLOCK; if (rh > BP_BLOCK) rh = BP_BLOCK;
    int real = rw * rh;

    uint8_t mn[3] = { 255, 255, 255 };
    uint8_t mx[3] = { 0, 0, 0 };
    for (int ly = 0; ly < BP_BLOCK; ly++) {
        for (int lx = 0; lx < BP_BLOCK; lx++) {
            RGBA8 c = block_texel(src, w, h, bx, by, lx, ly);
            tex[ly * BP_BLOCK + lx] = c;
            if (c.r < mn[0]) mn[0] = c.r;
            if (c.g < mn[1]) mn[1] = c.g;
            if (c.b < mn[2]) mn[2] = c.b;
            if (c.r > mx[0]) mx[0] = c.r;
            if (c.g > mx[1]) mx[1] = c.g;
            if (c.b > mx[2]) mx[2] = c.b;
        }
    }

    uint16_t e0 = rgb_to_565(mx[0], mx[1], mx[2]);
    uint16_t e1 = rgb_to_565(mn[0], mn[1], mn[2]);
    RGBA8 pal[4];
    bp_decode_palette(e0, e1, pal);

    uint32_t indices = 0;
    long sse = 0;
    for (int ly = 0; ly < BP_BLOCK; ly++) {
        for (int lx = 0; lx < BP_BLOCK; lx++) {
            RGBA8 c = tex[ly * BP_BLOCK + lx];
            int best = 0;
            int best_d = color_dist2(c, pal[0]);
            for (int j = 1; j < 4; j++) {
                int d = color_dist2(c, pal[j]);
                if (d < best_d) { best_d = d; best = j; }
            }
            indices |= (uint32_t)best << (2 * (ly * BP_BLOCK + lx));
            if (lx < rw && ly < rh) {
                RGBA8 q = pal[best];
                int dr = (int)c.r - (int)q.r;
                int dg = (int)c.g - (int)q.g;
                int db = (int)c.b - (int)q.b;
                int da = (int)c.a - (int)q.a;
                sse += (long)(dr * dr + dg * dg + db * db + da * da);
            }
        }
    }

    /* Error is measured over real texels only, so edge blocks are not
     * credited or penalized for their replicated padding. */
    double mse = (double)sse / (double)(real * 4);
    int tag = (mse <= mse_threshold) ? BP_TAG_BC1 : BP_TAG_RAW;

    if (pb_put8(pb, (uint8_t)tag) != 0) return -1;
    if (tag == BP_TAG_BC1) {
        if (pb_put16le(pb, e0) != 0 || pb_put16le(pb, e1) != 0 ||
            pb_put32le(pb, indices) != 0) return -1;
        if (counts) counts->bc1_blocks++;
    } else {
        for (int i = 0; i < 16; i++) {
            if (pb_write(pb, &tex[i], sizeof(RGBA8)) != 0) return -1;
        }
        if (counts) counts->raw_blocks++;
    }
    if (counts) counts->blocks++;
    return tag;
}

/* Decode one block, writing only texels that fall inside w x h. */
static int decode_block(ReadBuf* rb, RGBA8* dst, int w, int h,
                        int bx, int by, BlockCounts* counts) {
    uint8_t tag;
    if (rd_u8(rb, &tag) != 0) return -1;

    RGBA8 tex[16];
    if (tag == BP_TAG_BC1) {
        uint16_t e0, e1;
        uint32_t indices;
        if (rd_u16le(rb, &e0) != 0 || rd_u16le(rb, &e1) != 0 ||
            rd_u32le(rb, &indices) != 0) return -1;
        RGBA8 pal[4];
        bp_decode_palette(e0, e1, pal);
        for (int ly = 0; ly < BP_BLOCK; ly++)
            for (int lx = 0; lx < BP_BLOCK; lx++) {
                int j = (indices >> (2 * (ly * BP_BLOCK + lx))) & 3;
                tex[ly * BP_BLOCK + lx] = pal[j];
            }
        if (counts) counts->bc1_blocks++;
    } else if (tag == BP_TAG_RAW) {
        if (rd_read(rb, tex, sizeof(tex)) != 0) return -1;
        if (counts) counts->raw_blocks++;
    } else {
        return -1;
    }
    if (counts) counts->blocks++;

    for (int ly = 0; ly < BP_BLOCK; ly++) {
        for (int lx = 0; lx < BP_BLOCK; lx++) {
            int x = bx * BP_BLOCK + lx;
            int y = by * BP_BLOCK + ly;
            if (x < w && y < h)
                dst[y * w + x] = tex[ly * BP_BLOCK + lx];
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* public pack / unpack                                                */
/* ------------------------------------------------------------------ */

/* Pack raw RGBA8 (w*h texels) into pb. mse_threshold <= 0 forces RAW.
 * Returns 0 on success, -1 on invalid arguments or buffer overflow. */
static int bp_pack(const RGBA8* src, int w, int h,
                   double mse_threshold, PackBuf* pb) {
    if (!src || !pb || !pb->p || w <= 0 || h <= 0 ||
        w > BP_MAX_DIM || h > BP_MAX_DIM)
        return -1;

    int gw = (w + BP_BLOCK - 1) / BP_BLOCK;
    int gh = (h + BP_BLOCK - 1) / BP_BLOCK;
    static const uint8_t magic[4] = { 'A', 'T', 'L', 'P' };
    if (pb_write(pb, magic, 4) != 0 ||
        pb_put16le(pb, (uint16_t)w) != 0 ||
        pb_put16le(pb, (uint16_t)h) != 0 ||
        pb_put16le(pb, (uint16_t)gw) != 0 ||
        pb_put16le(pb, (uint16_t)gh) != 0 ||
        pb_put32le(pb, 0) != 0)            /* total length, patched at the end */
        return -1;

    BlockCounts counts = { 0, 0, 0 };
    for (int by = 0; by < gh; by++)
        for (int bx = 0; bx < gw; bx++)
            /* encode_block returns the chosen tag (RAW=0, BC1=1);
             * only -1 signals a stream failure. */
            if (encode_block(src, w, h, bx, by, mse_threshold,
                             &counts, pb) < 0)
                return -1;

    if (pb->overflow) return -1;
    uint32_t total = (uint32_t)pb->pos;
    pb->p[12] = (uint8_t)(total & 0xff);
    pb->p[13] = (uint8_t)((total >> 8) & 0xff);
    pb->p[14] = (uint8_t)((total >> 16) & 0xff);
    pb->p[15] = (uint8_t)((total >> 24) & 0xff);
    return 0;
}

/* Unpack a stream into dst (capacity dst_cap bytes). Returns 0 on success,
 * -1 on any truncation, bad header, unknown tag or short destination. */
static int bp_unpack(const uint8_t* data, size_t len,
                     RGBA8* dst, size_t dst_cap, BlockCounts* out) {
    if (!data || !dst || len < BP_HEADER_BYTES) return -1;
    ReadBuf rb = { data, len, 0, 0 };
    uint8_t magic[4];
    uint16_t w, h, gw, gh;
    uint32_t total;
    if (rd_read(&rb, magic, 4) != 0 || rd_u16le(&rb, &w) != 0 ||
        rd_u16le(&rb, &h) != 0 || rd_u16le(&rb, &gw) != 0 ||
        rd_u16le(&rb, &gh) != 0 || rd_u32le(&rb, &total) != 0)
        return -1;
    static const uint8_t want[4] = { 'A', 'T', 'L', 'P' };
    if (memcmp(magic, want, 4) != 0 || w < 1 || h < 1 ||
        w > BP_MAX_DIM || h > BP_MAX_DIM)
        return -1;
    if (gw != (w + BP_BLOCK - 1) / BP_BLOCK ||
        gh != (h + BP_BLOCK - 1) / BP_BLOCK || total > len)
        return -1;
    if ((size_t)w * (size_t)h * 4u > dst_cap)
        return -1;

    BlockCounts counts = { 0, 0, 0 };
    for (int by = 0; by < gh; by++)
        for (int bx = 0; bx < gw; bx++)
            if (decode_block(&rb, dst, w, h, bx, by, &counts) != 0)
                return -1;
    if (rb.pos != total) return -1;
    if (out) *out = counts;
    return 0;
}

/* MSE that a PSNR target (dB) corresponds to, for picking a threshold. */
static double bp_mse_from_psnr(double psnr_db) {
    return 65025.0 / pow(10.0, psnr_db / 10.0);
}

static void bp_compute_metrics(const RGBA8* src, const RGBA8* dst,
                               int w, int h, size_t packed_bytes,
                               PackMetrics* m) {
    long sse = 0, sae = 0;
    int max_diff = 0;
    int n = w * h;
    for (int i = 0; i < n; i++) {
        const RGBA8* a = &src[i];
        const RGBA8* b = &dst[i];
        int d[4] = {
            (int)a->r - (int)b->r, (int)a->g - (int)b->g,
            (int)a->b - (int)b->b, (int)a->a - (int)b->a
        };
        for (int k = 0; k < 4; k++) {
            int ad = d[k] < 0 ? -d[k] : d[k];
            if (ad > max_diff) max_diff = ad;
            sae += ad;
            sse += (long)d[k] * d[k];
        }
    }
    int samples = n * 4;
    m->raw_bytes = (size_t)n * 4u;
    m->packed_bytes = packed_bytes;
    m->header_bytes = BP_HEADER_BYTES;
    m->ratio = (double)packed_bytes / (double)m->raw_bytes;
    m->bits_per_pixel = (double)packed_bytes * 8.0 / (double)n;
    m->max_diff = (double)max_diff;
    m->mae = (double)sae / (double)samples;
    m->mse = (double)sse / (double)samples;
    m->psnr_db = m->mse == 0.0 ? INFINITY
                              : 10.0 * log10(65025.0 / m->mse);
}

/* ------------------------------------------------------------------ */
/* self-test                                                           */
/* ------------------------------------------------------------------ */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static uint32_t rng_state = 0x12345678u;
static uint32_t rnd(void) {
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state;
}

static RGBA8* make_gradient(int w, int h) {
    RGBA8* a = (RGBA8*)malloc((size_t)w * h * sizeof(RGBA8));
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            /* smooth ramp: within a 4x4 block adjacent texels differ by
             * only a few levels, so every block stays inside BC1's MSE
             * budget under the self-test threshold */
            RGBA8 c = {
                (uint8_t)(x * 4),
                (uint8_t)(y * 4),
                (uint8_t)((x + y) * 2),
                255
            };
            a[y * w + x] = c;
        }
    return a;
}

static RGBA8* make_noise(int w, int h) {
    RGBA8* a = (RGBA8*)malloc((size_t)w * h * sizeof(RGBA8));
    for (int i = 0; i < w * h; i++)
        a[i] = (RGBA8){ (uint8_t)(rnd() & 0xff), (uint8_t)(rnd() & 0xff),
                        (uint8_t)(rnd() & 0xff), 255 };
    return a;
}

static void report(const char* name, int w, int h,
                   const BlockCounts* c, const PackMetrics* m) {
    printf("atlas=%s size=%dx%d blocks=%d bc1=%d raw=%d "
           "raw_bytes=%zu packed_bytes=%zu header_bytes=%zu "
           "ratio=%.2f%% bpp=%.2f maxdiff=%.0f mae=%.3f mse=%.3f "
           "psnr=%.2fdB\n",
           name, w, h, c->blocks, c->bc1_blocks, c->raw_blocks,
           m->raw_bytes, m->packed_bytes, m->header_bytes,
           m->ratio * 100.0, m->bits_per_pixel,
           m->max_diff, m->mae, m->mse, m->psnr_db);
}

/* Full pack/unpack round trip; returns 0 and fills outputs on success. */
static int roundtrip(const RGBA8* src, int w, int h, double threshold,
                     size_t cap, RGBA8** out_buf, size_t* out_len,
                     BlockCounts* counts) {
    uint8_t* packed = (uint8_t*)malloc(cap);
    if (!packed) return -1;
    PackBuf pb = { packed, cap, 0, 0 };
    if (bp_pack(src, w, h, threshold, &pb) != 0) { free(packed); return -1; }
    RGBA8* decoded = (RGBA8*)malloc((size_t)w * h * sizeof(RGBA8));
    if (!decoded) { free(packed); return -1; }
    BlockCounts local = { 0, 0, 0 };
    if (bp_unpack(packed, pb.pos, decoded, (size_t)w * h * 4u,
                  &local) != 0) {
        free(packed); free(decoded); return -1;
    }
    if (counts) *counts = local;
    *out_buf = decoded;
    *out_len = pb.pos;
    free(packed);
    return 0;
}

int main(void) {
    const double threshold = 64.0;   /* max allowed per-block MSE */

    /* --- smooth gradient: every block should fit BC1 ---------------- */
    int gw = 16, gh = 16;
    RGBA8* grad = make_gradient(gw, gh);
    RGBA8* dec = NULL; size_t plen = 0; BlockCounts cnt = { 0, 0, 0 };
    check("gradient_pack",
          roundtrip(grad, gw, gh, threshold, 4096, &dec, &plen, &cnt) == 0);
    check("gradient_all_bc1", cnt.blocks == 16 && cnt.raw_blocks == 0 &&
                              cnt.bc1_blocks == 16);
    PackMetrics mm;
    bp_compute_metrics(grad, dec, gw, gh, plen, &mm);
    check("gradient_shrinks", plen < (size_t)gw * gh * 4u);
    check("gradient_within_bound", mm.max_diff <= 8.0 * sqrt(threshold) + 0.5);
    report("gradient", gw, gh, &cnt, &mm);
    free(dec);

    /* --- hard noise: most blocks must fall back to RAW --------------- */
    rng_state = 0xdeadbeefu;
    int nw = 32, nh = 32;
    RGBA8* noise = make_noise(nw, nh);
    dec = NULL; cnt = (BlockCounts){ 0, 0, 0 };
    check("noise_pack",
          roundtrip(noise, nw, nh, threshold, 65536, &dec, &plen, &cnt) == 0);
    check("noise_mostly_raw", cnt.raw_blocks > cnt.bc1_blocks);
    bp_compute_metrics(noise, dec, nw, nh, plen, &mm);
    check("noise_within_bound", mm.max_diff <= 8.0 * sqrt(threshold) + 0.5);
    report("noise", nw, nh, &cnt, &mm);
    free(dec);

    /* --- threshold 0: only exactly representable blocks use BC1, ----- */
    /* so the whole atlas must round-trip losslessly.                   */
    dec = NULL; cnt = (BlockCounts){ 0, 0, 0 };
    check("zero_threshold_pack",
          roundtrip(noise, nw, nh, 0.0, 65536, &dec, &plen, &cnt) == 0);
    bp_compute_metrics(noise, dec, nw, nh, plen, &mm);
    check("zero_threshold_lossless", mm.mse == 0.0 && mm.max_diff == 0.0);
    check("zero_threshold_psnr_inf", isinf(mm.psnr_db));
    report("noise_lossless", nw, nh, &cnt, &mm);
    free(dec);

    /* --- alpha variation cannot be represented by BC1 -> RAW --------- */
    /* Each block spans both a wide alpha ramp and a wide RGB ramp, so the
     * BC1 palette cannot fit and every block deterministically falls back
     * to RAW; the full RGBA8 texels then round-trip exactly. */
    RGBA8* alpha = (RGBA8*)malloc(64 * sizeof(RGBA8));
    for (int i = 0; i < 64; i++)
        alpha[i] = (RGBA8){ (uint8_t)((i * 37) & 0xff),
                            (uint8_t)((i * 73) & 0xff),
                            (uint8_t)((i * 131) & 0xff),
                            (uint8_t)((i * 7) & 0xff) };
    dec = NULL; cnt = (BlockCounts){ 0, 0, 0 };
    check("alpha_pack",
          roundtrip(alpha, 8, 8, threshold, 4096, &dec, &plen, &cnt) == 0);
    check("alpha_blocks_raw", cnt.raw_blocks == 4 && cnt.bc1_blocks == 0);
    int alpha_exact = 1;
    for (int i = 0; i < 64; i++)
        alpha_exact = alpha_exact &&
                      memcmp(&alpha[i], &dec[i], sizeof(RGBA8)) == 0;
    check("alpha_roundtrip_exact", alpha_exact);
    free(dec); free(alpha);

    /* --- non-multiple-of-4 atlas: padding and seam behavior ---------- */
    int ew = 18, eh = 13;
    RGBA8* edge = make_gradient(ew, eh);
    dec = NULL; cnt = (BlockCounts){ 0, 0, 0 };
    check("edge_pack",
          roundtrip(edge, ew, eh, threshold, 4096, &dec, &plen, &cnt) == 0);
    check("edge_grid_5x4", cnt.blocks == 20);
    bp_compute_metrics(edge, dec, ew, eh, plen, &mm);
    check("edge_within_bound", mm.max_diff <= 8.0 * sqrt(threshold) + 0.5);

    /* Border texels are covered by padded blocks: their error must obey
     * the same per-channel bound as interior texels (no seam penalty). */
    int border_max = 0, interior_max = 0;
    for (int y = 0; y < eh; y++)
        for (int x = 0; x < ew; x++) {
            const RGBA8* a = &edge[y * ew + x];
            const RGBA8* b = &dec[y * ew + x];
            int d[4] = { (int)a->r - (int)b->r, (int)a->g - (int)b->g,
                         (int)a->b - (int)b->b, (int)a->a - (int)b->a };
            int local = 0;
            for (int k = 0; k < 4; k++) {
                int ad = d[k] < 0 ? -d[k] : d[k];
                if (ad > local) local = ad;
            }
            if (x == 0 || y == 0 || x == ew - 1 || y == eh - 1) {
                if (local > border_max) border_max = local;
            } else if (local > interior_max) {
                interior_max = local;
            }
        }
    check("edge_no_seam", border_max <= interior_max + 1 &&
                          border_max <= 8.0 * sqrt(threshold) + 0.5);
    report("edge_padded", ew, eh, &cnt, &mm);
    printf("seam_border_maxdiff=%d seam_interior_maxdiff=%d\n",
           border_max, interior_max);

    /* Exact-sized destination: ASan/UBSan catches any padding write. */
    RGBA8* exact = (RGBA8*)malloc((size_t)ew * eh * sizeof(RGBA8));
    uint8_t* ebuf = (uint8_t*)malloc(4096);
    PackBuf epb = { ebuf, 4096, 0, 0 };
    check("edge_exact_pack", bp_pack(edge, ew, eh, threshold, &epb) == 0);
    check("edge_exact_unpack",
          bp_unpack(ebuf, epb.pos, exact, (size_t)ew * eh * 4u, NULL) == 0);
    int exact_match = 1;
    for (int i = 0; i < ew * eh; i++)
        exact_match = exact_match &&
                      (memcmp(&exact[i], &dec[i], sizeof(RGBA8)) == 0);
    check("edge_exact_dst_matches", exact_match);
    free(exact); free(ebuf); free(dec);

    /* --- 1x1 atlas: all sixteen slots replicate the lone texel ------- */
    /* Pick a color exactly representable in RGB565 (it survives the
     * 5/6-bit quantization unchanged) so the BC1 block is byte-exact. */
    RGBA8 one[1] = { { 41, 44, 49, 255 } };
    RGBA8 dst2[2] = { { 0xCD, 0xCD, 0xCD, 0xCD },
                      { 0xCD, 0xCD, 0xCD, 0xCD } };
    uint8_t sbuf[64];
    PackBuf spb = { sbuf, sizeof sbuf, 0, 0 };
    check("1x1_pack", bp_pack(one, 1, 1, threshold, &spb) == 0);
    check("1x1_is_bc1_25_bytes", spb.pos == BP_HEADER_BYTES + 1 + BP_BC1_BYTES);
    check("1x1_unpack",
          bp_unpack(sbuf, spb.pos, dst2, sizeof(RGBA8), NULL) == 0);
    check("1x1_texel_matches", memcmp(dst2, one, sizeof(RGBA8)) == 0);
    check("1x1_guard_untouched",
          memcmp(&dst2[1], (RGBA8[1]){ { 0xCD, 0xCD, 0xCD, 0xCD } },
                 sizeof(RGBA8)) == 0);

    /* --- writer must never escape a short buffer --------------------- */
    uint8_t small[64];
    memset(small, 0xAB, sizeof small);
    PackBuf bad = { small, 20, 0, 0 };
    check("short_pack_fails", bp_pack(grad, gw, gh, threshold, &bad) == -1);
    check("short_pack_flagged", bad.overflow == 1);
    int canary = 1;
    for (size_t i = 20; i < sizeof small; i++) canary = canary && small[i] == 0xAB;
    check("short_pack_no_overrun", canary);

    /* Exact-capacity pack succeeds, one byte less fails. */
    size_t need = spb.pos;   /* reuse the 1x1 size: 25 bytes */
    uint8_t exact_cap[25];
    PackBuf ok = { exact_cap, need, 0, 0 };
    check("exact_cap_pack", bp_pack(one, 1, 1, threshold, &ok) == 0 &&
                            ok.pos == need);
    PackBuf tight = { exact_cap, need - 1, 0, 0 };
    check("one_byte_short_fails",
          bp_pack(one, 1, 1, threshold, &tight) == -1);

    /* --- reader rejects truncation, bad magic, short destination ----- */
    check("unpack_truncated",
          bp_unpack(sbuf, spb.pos / 2, dst2, sizeof dst2, NULL) == -1);
    uint8_t corrupt[64];
    memcpy(corrupt, sbuf, spb.pos);
    corrupt[0] = 'X';
    check("unpack_bad_magic",
          bp_unpack(corrupt, spb.pos, dst2, sizeof dst2, NULL) == -1);
    check("unpack_short_dst",
          bp_unpack(sbuf, spb.pos, dst2, 1, NULL) == -1);
    check("unpack_null_args",
          bp_unpack(NULL, spb.pos, dst2, sizeof dst2, NULL) == -1);
    check("pack_rejects_bad_dims",
          bp_pack(grad, 0, gh, threshold, &ok) == -1 &&
          bp_pack(grad, gw, BP_MAX_DIM + 1, threshold, &ok) == -1);

    /* --- threshold selection helper ---------------------------------- */
    double mse40 = bp_mse_from_psnr(40.0);
    printf("psnr_target=40dB -> mse_threshold=%.4f\n", mse40);
    check("psnr_helper", fabs(mse40 - 65025.0 / 10000.0) < 1e-9);

    free(grad);
    free(noise);
    free(edge);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
