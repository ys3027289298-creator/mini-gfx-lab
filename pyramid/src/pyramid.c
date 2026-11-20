
/* pyramid: lazily-grown image sum pyramid with exact 2x2 block reduction.
 *
 * Level k cell (i,j) stores the sum of the base pixels inside the base-space
 * rectangle [i*2^k,(i+1)*2^k) x [j*2^k,(j+1)*2^k), clipped to the image.
 * The cells of every level partition the base image, so each level can be
 * derived from the previous one by 2x2 summation and the result is identical
 * cell-for-cell to recomputing that level straight from the base image --
 * including images with odd dimensions, where edge cells cover 1x2/2x1/1x1
 * fine cells instead of a full 2x2 block.
 *
 * Levels are allocated one at a time on first request: a fresh pyramid only
 * pays for the base image, and coarser levels grow memory on demand.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

typedef struct {
    int W, H;                 /* base dimensions */
    int max_levels;           /* levels until a single cell covers the image */
    int levels_built;         /* levels currently allocated (0..built-1) */
    int *lw, *lh;             /* cell dimensions of each level */
    long long **data;         /* data[k] == NULL until level k is built */
    size_t bytes;             /* pixel memory currently held, in bytes */
    long long query_ops;      /* coarse cells read by the most recent query */
} Pyramid;

static int ilog2_ceil(int v) {
    int k = 0;
    while ((1 << k) < v) k++;
    return k;
}

static void pyramid_init(Pyramid *p, const unsigned char *base, int W, int H) {
    p->W = W;
    p->H = H;
    p->max_levels = ilog2_ceil(W > H ? W : H) + 1;
    p->levels_built = 0;
    p->lw = calloc((size_t)p->max_levels, sizeof(int));
    p->lh = calloc((size_t)p->max_levels, sizeof(int));
    p->data = calloc((size_t)p->max_levels, sizeof(long long *));
    p->bytes = 0;
    p->query_ops = 0;
    /* level 0 is the base image itself; it is the only level paid for now */
    p->lw[0] = W;
    p->lh[0] = H;
    p->data[0] = malloc(sizeof(long long) * (size_t)W * (size_t)H);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            p->data[0][y * W + x] = base[y * W + x];
    p->levels_built = 1;
    p->bytes = sizeof(long long) * (size_t)W * (size_t)H;
}

static void pyramid_free(Pyramid *p) {
    for (int k = 0; k < p->levels_built; k++) free(p->data[k]);
    free(p->data);
    free(p->lw);
    free(p->lh);
    memset(p, 0, sizeof(*p));
}

/* build exactly one new level from the previous one: each coarse cell is the
 * sum of the up-to-four fine cells whose footprints partition its rectangle */
static void pyramid_build_next(Pyramid *p) {
    int k = p->levels_built;
    if (k >= p->max_levels) return;
    int fw = p->lw[k - 1], fh = p->lh[k - 1];
    int cw = (fw + 1) / 2, ch = (fh + 1) / 2;
    const long long *fine = p->data[k - 1];
    long long *coarse = malloc(sizeof(long long) * (size_t)cw * (size_t)ch);
    for (int j = 0; j < ch; j++) {
        for (int i = 0; i < cw; i++) {
            long long acc = 0;
            for (int dy = 0; dy < 2; dy++) {
                int sy = 2 * j + dy;
                if (sy >= fh) continue;
                for (int dx = 0; dx < 2; dx++) {
                    int sx = 2 * i + dx;
                    if (sx >= fw) continue;
                    acc += fine[sy * fw + sx];
                }
            }
            coarse[j * cw + i] = acc;
        }
    }
    p->lw[k] = cw;
    p->lh[k] = ch;
    p->data[k] = coarse;
    p->levels_built = k + 1;
    p->bytes += sizeof(long long) * (size_t)cw * (size_t)ch;
}

/* 逐级生成: ensure levels 0..k exist, building them in order, on demand */
static void pyramid_ensure_level(Pyramid *p, int k) {
    if (k >= p->max_levels) k = p->max_levels - 1;
    while (p->levels_built <= k) pyramid_build_next(p);
}

/* coarsest level at which a region of size rw x rh still yields at least
 * `target` cells on its shorter side */
static int pyramid_pick_level(const Pyramid *p, int rw, int rh, int target) {
    if (target < 1) target = 1;
    int r = rw < rh ? rw : rh;
    int k = 0;
    while (k + 1 < p->max_levels) {
        int s = 1 << (k + 1);
        if ((r + s - 1) / s < target) break;
        k++;
    }
    return k;
}

/* expand a base-space rect to the level-k cell grid, clamped to the image;
 * returns 0 for an empty rect */
static int snap_rect(int W, int H, int k, int x0, int y0, int x1, int y1,
                     int *ix0, int *iy0, int *ix1, int *iy1) {
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > W) x1 = W;
    if (y1 > H) y1 = H;
    if (x0 >= x1 || y0 >= y1) return 0;
    int s = 1 << k;
    int lw = (W + s - 1) / s, lh = (H + s - 1) / s;
    *ix0 = x0 / s;
    *iy0 = y0 / s;
    *ix1 = (x1 + s - 1) / s;
    *iy1 = (y1 + s - 1) / s;
    if (*ix1 > lw) *ix1 = lw;
    if (*iy1 > lh) *iy1 = lh;
    return 1;
}

/* region query in base-image coordinates: the rect is expanded to level-k
 * cell boundaries and the covering coarse cells are copied out packed.
 * Cost is one cell read per output cell -- independent of full image size. */
static int pyramid_query(Pyramid *p, int k, int x0, int y0, int x1, int y1,
                         long long *out, int *out_w, int *out_h) {
    pyramid_ensure_level(p, k);
    if (k >= p->max_levels) k = p->max_levels - 1;
    int ix0, iy0, ix1, iy1;
    if (!snap_rect(p->W, p->H, k, x0, y0, x1, y1, &ix0, &iy0, &ix1, &iy1)) {
        *out_w = *out_h = 0;
        p->query_ops = 0;
        return 0;
    }
    int w = ix1 - ix0, h = iy1 - iy0;
    const long long *plane = p->data[k];
    long long ops = 0;
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++) {
            out[j * w + i] = plane[(iy0 + j) * p->lw[k] + (ix0 + i)];
            ops++;
        }
    p->query_ops = ops;
    *out_w = w;
    *out_h = h;
    return w * h;
}

/* reference: build one level directly from the base image, summing every
 * pixel of each cell's footprint -- the "full-image processing" baseline */
static long long *reference_level(const unsigned char *base, int W, int H, int k,
                                  int *out_w, int *out_h) {
    int s = 1 << k;
    int w = (W + s - 1) / s, h = (H + s - 1) / s;
    long long *plane = malloc(sizeof(long long) * (size_t)w * (size_t)h);
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            long long acc = 0;
            int ys = j * s, ye = ys + s;
            int xs = i * s, xe = xs + s;
            if (ye > H) ye = H;
            if (xe > W) xe = W;
            for (int y = ys; y < ye; y++)
                for (int x = xs; x < xe; x++)
                    acc += base[y * W + x];
            plane[j * w + i] = acc;
        }
    }
    *out_w = w;
    *out_h = h;
    return plane;
}

/* naive baseline for "any tile forces a full-image pass": the whole level-k
 * reduction is rebuilt from the entire base image before the region is cut */
static int naive_query(const unsigned char *base, int W, int H, int k,
                       int x0, int y0, int x1, int y1,
                       long long *out, int *ow, int *oh, long long *ops) {
    int rw, rh;
    long long *full = reference_level(base, W, H, k, &rw, &rh);
    long long n = 0;
    for (int j = 0; j < rh; j++)
        for (int i = 0; i < rw; i++) {
            (void)full[j * rw + i];
            n++;
        }
    int ix0, iy0, ix1, iy1;
    if (!snap_rect(W, H, k, x0, y0, x1, y1, &ix0, &iy0, &ix1, &iy1)) {
        *ow = *oh = 0;
        *ops = (long long)W * H;
        free(full);
        return 0;
    }
    int w = ix1 - ix0, h = iy1 - iy0;
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++)
            out[j * w + i] = full[(iy0 + j) * rw + (ix0 + i)];
    /* reference_level reads each base pixel exactly once: a full-image pass */
    *ops = (long long)W * H + n;
    *ow = w;
    *oh = h;
    free(full);
    return w * h;
}

static unsigned int rng_state = 0x12345678u;
static unsigned int rng_next(void) {
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

static int failures = 0;
static void check(const char *name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned char *make_image(int W, int H) {
    unsigned char *img = malloc((size_t)W * (size_t)H);
    for (int i = 0; i < W * H; i++) img[i] = (unsigned char)(rng_next() & 0xFFu);
    return img;
}

/* exactness: incremental 2x2 pyramid vs levels rebuilt from the base image,
 * on odd dimensions so edge-cell clipping is exercised */
static void test_exactness(void) {
    const int W = 53, H = 37;
    unsigned char *img = make_image(W, H);
    Pyramid p;
    pyramid_init(&p, img, W, H);
    check("init_base_only", p.levels_built == 1 &&
          p.bytes == sizeof(long long) * (size_t)W * (size_t)H);

    size_t expect_bytes = sizeof(long long) * (size_t)W * (size_t)H;
    int all_match = 1;
    for (int k = 0; k < p.max_levels; k++) {
        pyramid_ensure_level(&p, k);
        if (p.levels_built != k + 1) all_match = 0;      /* grew one level */
        if (k > 0)
            expect_bytes += sizeof(long long) *
                            (size_t)p.lw[k] * (size_t)p.lh[k];
        if (p.bytes != expect_bytes) all_match = 0;      /* accounting */
        int rw, rh;
        long long *ref = reference_level(img, W, H, k, &rw, &rh);
        if (rw != p.lw[k] || rh != p.lh[k] ||
            memcmp(ref, p.data[k],
                   sizeof(long long) * (size_t)rw * (size_t)rh) != 0)
            all_match = 0;
        free(ref);
    }
    check("level_by_level_exact", all_match);
    check("all_levels_grown_on_demand", p.levels_built == p.max_levels);

    /* region queries at random levels/rects must match base-image processing */
    int queries_match = 1, ops_exact = 1;
    long long out[4096], ref_out[4096];
    for (int t = 0; t < 60; t++) {
        int k = (int)(rng_next() % (unsigned)p.max_levels);
        int x0 = (int)(rng_next() % (unsigned)W);
        int x1 = (int)(rng_next() % (unsigned)(W + 12)) - 4; /* may go OOR */
        int y0 = (int)(rng_next() % (unsigned)H);
        int y1 = (int)(rng_next() % (unsigned)(H + 12)) - 4;
        int ow, oh, rw, rh;
        int n = pyramid_query(&p, k, x0, y0, x1, y1, out, &ow, &oh);
        int rn = naive_query(img, W, H, k, x0, y0, x1, y1,
                             ref_out, &rw, &rh, &(long long){0});
        if (n != rn || ow != rw || oh != rh) { queries_match = 0; continue; }
        if (n == 0) continue;
        if (memcmp(out, ref_out, sizeof(long long) * (size_t)n) != 0)
            queries_match = 0;
        if (p.query_ops != (long long)ow * oh) ops_exact = 0;
    }
    check("region_query_exact", queries_match);
    check("query_ops_eq_output_cells", ops_exact);

    int ow, oh;
    check("empty_query", pyramid_query(&p, 2, 10, 10, 10, 20,
                                       out, &ow, &oh) == 0);
    check("clamped_query", pyramid_query(&p, 1, -5, -5, W + 99, H + 99,
                                         out, &ow, &oh) == p.lw[1] * p.lh[1]);
    pyramid_free(&p);
    free(img);
}

/* query cost tracks output pixel count, not full-image size; contrast with
 * the naive full-image pass whose cost is the whole base image */
static void test_proportional_cost(void) {
    const int k = 3;
    const int sizes[] = {256, 512, 1024, 2048};
    long long prev_pyr = 0;
    int constant_on_growing_image = 1;
    long long buf[65536];
    for (size_t t = 0; t < sizeof(sizes) / sizeof(sizes[0]); t++) {
        int N = sizes[t];
        unsigned char *img = make_image(N, N);
        Pyramid p;
        pyramid_init(&p, img, N, N);
        int ow, oh;
        pyramid_query(&p, k, 10, 20, 10 + 64, 20 + 64, buf, &ow, &oh);
        long long nops;
        naive_query(img, N, N, k, 10, 20, 10 + 64, 20 + 64,
                    buf, &ow, &oh, &nops);
        if (t > 0 && p.query_ops != prev_pyr) constant_on_growing_image = 0;
        long long plane_cells = (long long)(N >> k) * (N >> k);
        if (nops != (long long)N * N + plane_cells)
            constant_on_growing_image = 0;
        if (p.levels_built != k + 1) constant_on_growing_image = 0;
        printf("cost N=%d: pyramid_ops=%lld naive_ops=%lld levels_built=%d\n",
               N, p.query_ops, nops, p.levels_built);
        prev_pyr = p.query_ops;
        pyramid_free(&p);
        free(img);
    }
    check("pyramid_cost_independent_of_image_size", constant_on_growing_image);

    /* doubling each region side quadruples output cells and query ops */
    unsigned char *img = make_image(1024, 1024);
    Pyramid p;
    pyramid_init(&p, img, 1024, 1024);
    int w1, h1, w2, h2;
    pyramid_query(&p, 2, 0, 0, 64, 64, buf, &w1, &h1);
    long long ops_small = p.query_ops;
    pyramid_query(&p, 2, 0, 0, 128, 128, buf, &w2, &h2);
    long long ops_large = p.query_ops;
    check("query_cost_linear_in_pixels",
          ops_large == 4 * ops_small && w2 == 2 * w1 && h2 == 2 * h1);
    pyramid_free(&p);
    free(img);
}

static void test_pick_level(void) {
    unsigned char *img = make_image(64, 64);
    Pyramid p;
    pyramid_init(&p, img, 64, 64);            /* max_levels = 7 */
    check("pick_target1_top", pyramid_pick_level(&p, 256, 256, 1) == 6);
    check("pick_target4", pyramid_pick_level(&p, 256, 256, 4) == 6);
    /* 128px short side, target 4 cells: level 5 spans 32px -> exactly 4;
     * level 6 would span 64px and give only 2 */
    check("pick_short_side", pyramid_pick_level(&p, 256, 128, 4) == 5);
    /* target 2 on a 3px-wide region: level 1 gives 2 cells, level 2 only 1 */
    check("pick_tiny_region", pyramid_pick_level(&p, 3, 3, 2) == 1);
    pyramid_free(&p);
    free(img);

    /* randomized invariant on an image large enough to contain every region:
     * target never exceeds the base-level cell count on the short side */
    unsigned char *big = make_image(512, 512);
    Pyramid q;
    pyramid_init(&q, big, 512, 512);            /* max_levels = 10 */
    int property = 1;
    for (int t = 0; t < 100; t++) {
        int rw = 1 + (int)(rng_next() % 512u);
        int rh = 1 + (int)(rng_next() % 512u);
        int r = rw < rh ? rw : rh;
        int target = 1 + (int)(rng_next() % (unsigned)(r < 8 ? r : 8));
        int k = pyramid_pick_level(&q, rw, rh, target);
        int cells_k = (r + (1 << k) - 1) >> k;
        if (cells_k < target) property = 0;                 /* satisfies */
        if (k + 1 < q.max_levels) {
            int cells_next = (r + (1 << (k + 1)) - 1) >> (k + 1);
            if (cells_next >= target) property = 0;         /* and is coarsest */
        }
    }
    check("pick_level_invariant", property);
    pyramid_free(&q);
    free(big);
}

/* memory upper bound as a function of the number of levels */
static void test_memory_bound(void) {
    const int W = 53, H = 37;
    unsigned char *img = make_image(W, H);
    Pyramid p;
    pyramid_init(&p, img, W, H);
    size_t measured = 0;
    printf("memory growth for %dx%d (8 bytes/cell), max_levels=%d:\n",
           W, H, p.max_levels);
    for (int k = 0; k < p.max_levels; k++) {
        pyramid_ensure_level(&p, k);
        measured += (size_t)p.lw[k] * (size_t)p.lh[k] * sizeof(long long);
        if (measured != p.bytes) failures++;
        double s4 = (4.0 / 3.0) * (1.0 - 1.0 / (double)(1ull << (2 * k + 2)));
        double s2 = 2.0 * (1.0 - 1.0 / (double)(1 << (k + 1)));
        double bound = 8.0 * ((double)W * H * s4 +
                              (double)(W + H) * s2 + (k + 1));
        printf("  L=%d dims=%dx%d bytes=%zu bound=%.0f ok=%d\n",
               k + 1, p.lw[k], p.lh[k], p.bytes, bound,
               (double)p.bytes <= bound);
        if (!((double)p.bytes <= bound)) failures++;
    }

    /* power-of-two image: rounding terms vanish, the geometric-series bound
     * is the exact cell count */
    unsigned char *sq = make_image(64, 64);
    Pyramid q;
    pyramid_init(&q, sq, 64, 64);
    pyramid_ensure_level(&q, q.max_levels - 1);
    long long cells = 0;
    for (int k = 0; k < q.max_levels; k++)
        cells += (long long)q.lw[k] * q.lh[k];
    long long exact_series = 4096 + 1024 + 256 + 64 + 16 + 4 + 1;
    check("pow2_geometric_series", cells == exact_series &&
          (size_t)(cells * 8) == q.bytes);
    pyramid_free(&q);
    free(sq);
    pyramid_free(&p);
    free(img);

    printf("memory_bound formula:\n");
    printf("  exact:    M(L) = 8 * sum_{k=0}^{L-1} ceil(W/2^k)*ceil(H/2^k)\n");
    printf("  pow2:     M(L) = 8*W*H*(4/3)*(1-4^-L) < (32/3)*W*H bytes\n");
    printf("  general:  M(L) <= 8*(W*H*(4/3)(1-4^-L) + (W+H)*2(1-2^-L) + L)\n");
    printf("  limit:    L -> inf costs < 4/3 of base memory (pow2 dims)\n");
    double big = 4096.0 * 4096.0;
    printf("  4096x4096: base=%.0f KiB, pyramid upper bound=%.0f KiB, "
           "max_levels=%d\n",
           8.0 * big / 1024.0, (32.0 / 3.0) * big / 1024.0, 13);
}

int main(void) {
    test_exactness();
    test_proportional_cost();
    test_pick_level();
    test_memory_bound();
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
