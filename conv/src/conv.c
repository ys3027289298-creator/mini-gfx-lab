/* conv: long impulse-response convolution in the frequency domain.
 *
 * See conv.h for the algorithm. The filter spectra and the twiddle table are
 * built once; each block then costs one forward FFT, P complex multiply-adds
 * and one inverse FFT.
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "conv.h"

/* ------------------------------------------------------------------ */
/* iterative radix-2 FFT, twiddles exp(-2*pi*i*k/n), unscaled          */

static void fft_run(double* re, double* im, int n, const double* tw,
                    int inverse) {
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            double t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    const double* twr = tw;
    const double* twi = tw + n / 2;
    /* tw holds exp(-2*pi*i*k/n): forward uses it as is, inverse conjugates */
    const double sgn = inverse ? -1.0 : 1.0;
    for (int len = 2; len <= n; len <<= 1) {
        int half = len >> 1, step = n / len;
        for (int i = 0; i < n; i += len) {
            for (int k = 0; k < half; k++) {
                double wr = twr[k * step];
                double wi = sgn * twi[k * step];
                int a = i + k, b = a + half;
                double br = re[b], bi = im[b];
                double tr = br * wr - bi * wi;
                double ti = br * wi + bi * wr;
                re[b] = re[a] - tr; im[b] = im[a] - ti;
                re[a] += tr;        im[a] += ti;
            }
        }
    }
}

/* smallest power of two >= n; n must be positive and <= 1 << 30 */
static int next_pow2(int n) {
    int p = 1;
    while (p < n) p <<= 1;
    return p;
}

/* ------------------------------------------------------------------ */

size_t conv_bytes(const Conv* c) {
    if (!c) return 0;
    size_t d = sizeof(double);
    size_t n = (size_t)c->partitions * 2 * (size_t)c->nfft;
    return d * ((size_t)c->block_len            /* previous block */
                + 4 * (size_t)c->nfft           /* frame + acc */
                + (size_t)c->nfft               /* twiddles */
                + 2 * n);                       /* hspec + delay line */
}

Conv* conv_create(const double* h, int filter_len, int block_len) {
    if (!h || filter_len <= 0 || block_len <= 0) return NULL;
    /* next_pow2(2 * block_len) must stay representable */
    if (block_len > (1 << 28)) return NULL;

    Conv* c = calloc(1, sizeof(*c));
    if (!c) return NULL;
    c->block_len = block_len;
    c->filter_len = filter_len;
    c->partitions = (filter_len + block_len - 1) / block_len;
    c->nfft = next_pow2(2 * block_len);

    size_t nfft = (size_t)c->nfft;
    size_t spec = (size_t)c->partitions * 2 * nfft;

    /* history before block 0 is silence; without this every early output
     * would be contaminated by whatever malloc returned */
    c->hist  = calloc((size_t)block_len, sizeof(double));
    c->frame = malloc(sizeof(double) * 2 * nfft);
    c->acc   = malloc(sizeof(double) * 2 * nfft);
    c->tw    = malloc(sizeof(double) * nfft);
    c->hspec = malloc(sizeof(double) * spec);
    c->fdl   = calloc(spec, sizeof(double));  /* past input blocks start silent */
    if (!c->hist || !c->frame || !c->acc || !c->tw ||
        !c->hspec || !c->fdl) {
        conv_destroy(c);
        return NULL;
    }

    /* twiddles for exp(-2*pi*i*k/nfft), k < nfft/2 */
    for (int k = 0; k < c->nfft / 2; k++) {
        double a = -6.283185307179586476925286766559 * (double)k / (double)c->nfft;
        c->tw[k] = cos(a);
        c->tw[c->nfft / 2 + k] = sin(a);
    }

    /* transform each filter partition once */
    for (int p = 0; p < c->partitions; p++) {
        double* re = c->hspec + (size_t)p * 2 * nfft;
        double* im = re + nfft;
        int off = p * block_len;
        int len = filter_len - off;
        if (len > block_len) len = block_len;
        /* zero the padding and the imaginary half: only `len` taps are real */
        memset(re, 0, sizeof(double) * 2 * nfft);
        for (int i = 0; i < len; i++) re[i] = h[off + i];
        fft_run(re, im, c->nfft, c->tw, 0);
    }

    return c;
}

void conv_destroy(Conv* c) {
    if (!c) return;
    free(c->hist);
    free(c->frame);
    free(c->acc);
    free(c->tw);
    free(c->hspec);
    free(c->fdl);
    free(c);
}

void conv_process_block(Conv* c, const double* x, double* y) {
    const int n = c->block_len, N = c->nfft, P = c->partitions;
    double* re = c->frame;
    double* im = c->frame + N;
    size_t spec = (size_t)2 * (size_t)N;

    /* overlap-save frame: [previous block | this block], zero-padded to nfft */
    memcpy(re, c->hist, sizeof(double) * (size_t)n);
    memcpy(re + n, x, sizeof(double) * (size_t)n);
    if (2 * n < N) memset(re + 2 * n, 0, sizeof(double) * (size_t)(N - 2 * n));
    memset(im, 0, sizeof(double) * (size_t)N);
    fft_run(re, im, N, c->tw, 0);

    /* Filter partition p convolves the input block from p blocks ago, so the
     * spectrum it multiplies is the one stored p slots back in the delay line
     * (zeros before the first block, which is exactly the silent history). */
    memset(c->acc, 0, sizeof(double) * 2 * (size_t)N);
    for (int p = 0; p < P; p++) {
        const double* xr;
        const double* xi;
        if (p == 0) {
            xr = re;
            xi = im;
        } else {
            size_t slot = ((size_t)c->head + (size_t)P - (size_t)p) % (size_t)P;
            xr = c->fdl + slot * spec;
            xi = xr + N;
        }
        const double* hr = c->hspec + (size_t)p * spec;
        const double* hi = hr + N;
        double* ar = c->acc;
        double* ai = c->acc + N;
        for (int k = 0; k < N; k++) {
            double xrk = xr[k], xik = xi[k];
            ar[k] += xrk * hr[k] - xik * hi[k];
            ai[k] += xrk * hi[k] + xik * hr[k];
        }
    }

    /* Samples [n, 2n) of the circular result are the overlap-save output: the
     * new block sits there, and for u in [n, 2n) every u - j (0 <= j < n) is
     * in [1, 2n), so no index wraps and no padding leaks in. */
    double* ar = c->acc;
    double* ai = c->acc + N;
    fft_run(ar, ai, N, c->tw, 1);
    double scale = 1.0 / (double)N;
    for (int i = 0; i < n; i++) y[i] = ar[n + i] * scale;

    /* retain this block's spectrum for the later filter partitions */
    memcpy(c->fdl + (size_t)c->head * spec, re, sizeof(double) * spec);
    c->head = (c->head + 1) % P;

    /* roll the time history for the next frame */
    memcpy(c->hist, x, sizeof(double) * (size_t)n);
}

int conv_process(Conv* c, const double* x, double* y, int n) {
    if (!c || !x || !y || n <= 0 || n % c->block_len != 0) return -1;
    for (int off = 0; off < n; off += c->block_len)
        conv_process_block(c, x + off, y + off);
    return 0;
}

int conv_run(const double* x, int N, const double* h, int M,
             int block_len, double* y) {
    if (!x || !h || !y || N <= 0 || M <= 0) return -1;
    if (block_len <= 0) block_len = 1024;

    Conv* c = conv_create(h, M, block_len);
    if (!c) return -1;

    int n = c->block_len;
    int out_len = N + M - 1;
    int out_blocks = (out_len + n - 1) / n;
    double* pad = malloc(sizeof(double) * (size_t)n);
    double* tail = malloc(sizeof(double) * (size_t)n);
    if (!pad || !tail) {
        free(pad);
        free(tail);
        conv_destroy(c);
        return -1;
    }

    for (int b = 0; b < out_blocks; b++) {
        int off = b * n;
        const double* src;
        if (off + n <= N) {
            src = x + off;
        } else {
            memset(pad, 0, sizeof(double) * (size_t)n);
            int left = N - off;
            if (left > 0) memcpy(pad, x + off, sizeof(double) * (size_t)left);
            src = pad;
        }
        conv_process_block(c, src, tail);
        int left = out_len - off;
        int take = left < n ? left : n;
        memcpy(y + off, tail, sizeof(double) * (size_t)take);
    }

    free(pad);
    free(tail);
    conv_destroy(c);
    return 0;
}

/* ------------------------------------------------------------------ */
/* self-test                                                          */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned long long rng_state = 0x2545F4914F6CDD1DULL;
static double rnd(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return (double)(rng_state >> 11) / (double)(1ULL << 53) * 2.0 - 1.0;
}

static void direct_conv(const double* x, int N, const double* h, int M,
                        double* y) {
    for (int n = 0; n < N + M - 1; n++) {
        double s = 0.0;
        int k0 = n - M + 1; if (k0 < 0) k0 = 0;
        int k1 = n < N - 1 ? n : N - 1;
        for (int k = k0; k <= k1; k++) s += x[k] * h[n - k];
        y[n] = s;
    }
}

static double max_diff(const double* a, const double* b, int n) {
    double m = 0.0;
    for (int i = 0; i < n; i++) {
        double d = fabs(a[i] - b[i]);
        if (d > m) m = d;
    }
    return m;
}

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(void) {
    enum { N = 1500, M = 701, LOUT = N + M - 1 };
    static double x[N], h[M], ref[LOUT], got[LOUT];
    static double hbig[16384];
    for (int i = 0; i < N; i++) x[i] = rnd();
    for (int i = 0; i < M; i++) h[i] = rnd();
    direct_conv(x, N, h, M, ref);

    /* accuracy against direct convolution for arbitrary block lengths */
    const int blocks[] = { 1, 3, 16, 100, 333, 1024 };
    const char* names[] = { "match_blk1", "match_blk3", "match_blk16",
                            "match_blk100", "match_blk333", "match_blk1024" };
    for (int t = 0; t < 6; t++) {
        memset(got, 0, sizeof(got));
        int rc = conv_run(x, N, h, M, blocks[t], got);
        check(names[t], rc == 0 && max_diff(ref, got, LOUT) < 1e-9);
    }

    /* impulse in -> impulse response out */
    {
        static double y2[2 * M], imp[N];
        memset(imp, 0, sizeof(imp));
        imp[0] = 1.0;
        conv_run(imp, M, h, M, 64, y2);
        check("impulse_out", max_diff(y2, h, M) < 1e-12);
    }

    /* streaming conv_process_block equals one-shot conv_run */
    {
        enum { BLK = 256, NB = (LOUT + BLK - 1) / BLK };
        Conv* c = conv_create(h, M, BLK);
        static double ys[NB * BLK], pad[NB * BLK];
        memset(ys, 0, sizeof(ys));
        memset(pad, 0, sizeof(pad));
        memcpy(pad, x, sizeof(double) * (size_t)N);
        int ok = c != NULL;
        for (int b = 0; ok && b < NB; b++)
            conv_process_block(c, pad + b * BLK, ys + b * BLK);
        check("stream_matches", ok && max_diff(ref, ys, LOUT) < 1e-9);
        check("bad_len_rejected",
              conv_process(c, x, ys, BLK + 1) == -1);
        conv_destroy(c);
    }

    /* memory: linear in block length, independent of input streaming length */
    {
        for (int i = 0; i < 16384; i++) hbig[i] = rnd();
        Conv* a = conv_create(hbig, 16384, 1024);
        Conv* b = conv_create(hbig, 16384, 4096);
        size_t ba = conv_bytes(a), bb = conv_bytes(b);
        check("mem_present", ba > 0 && bb > 0);
        check("mem_linear_bound",
              ba <= (size_t)300 * (16384 + 1024) &&
              bb <= (size_t)300 * (16384 + 4096));
        check("mem_block_independent", bb <= 2 * ba);
        conv_destroy(a);
        conv_destroy(b);
    }

    /* randomized sweep: every block length in 1..64 must match direct */
    {
        int worst_ok = 1;
        double worst = 0.0;
        for (int blk = 1; blk <= 64; blk++) {
            memset(got, 0, sizeof(got));
            int rc = conv_run(x, N, h, M, blk, got);
            double d = max_diff(ref, got, LOUT);
            if (d > worst) worst = d;
            if (rc != 0 || d >= 1e-9) worst_ok = 0;
        }
        printf("sweep_worst_err=%.2e\n", worst);
        check("blk_sweep_1_64", worst_ok);
    }

    /* odd filter lengths against odd block lengths */
    {
        int ok = 1;
        for (int m = 1; m <= 37; m += 4) {
            for (int blk = 1; blk <= 11; blk += 2) {
                static double hr[64], yr[200];
                for (int i = 0; i < m; i++) hr[i] = rnd();
                if (conv_run(x, 100, hr, m, blk, yr) != 0) { ok = 0; continue; }
                static double rr[200];
                direct_conv(x, 100, hr, m, rr);
                if (max_diff(rr, yr, 100 + m - 1) >= 1e-9) ok = 0;
            }
        }
        check("shape_sweep", ok);
    }

    check("reject_bad_args",
          conv_create(NULL, 10, 16) == NULL &&
          conv_create(h, 0, 16) == NULL &&
          conv_create(h, 10, 0) == NULL && conv_run(NULL, 1, h, 1, 16, got) != 0);

    /* speed and memory: block length 1024 vs direct convolution */
    {
        enum { SN = 32768, SM = 16384, SL = SN + SM - 1 };
        static double sx[SN], sh[SM], sy[SL], sd[SL];
        for (int i = 0; i < SN; i++) sx[i] = rnd();
        for (int i = 0; i < SM; i++) sh[i] = rnd();

        double t0 = now_sec();
        direct_conv(sx, SN, sh, SM, sd);
        double t_direct = now_sec() - t0;

        Conv* c = conv_create(sh, SM, 1024);
        size_t bytes = conv_bytes(c);
        t0 = now_sec();
        int rc = c != NULL && conv_run(sx, SN, sh, SM, 1024, sy) == 0;
        double t_fft = now_sec() - t0;
        conv_destroy(c);

        double err = max_diff(sd, sy, SL);
        double speedup = t_fft > 0 ? t_direct / t_fft : 0.0;
        printf("direct_ms=%.3f fft_ms=%.3f speedup=%.1fx err=%.2e "
               "mem_kb=%zu\n",
               t_direct * 1e3, t_fft * 1e3, speedup, err, bytes / 1024);
        check("speed_err", rc && err < 1e-9);
        check("speedup_5x", speedup >= 5.0);
        check("mem_linear_bound",
              bytes <= (size_t)80 * (size_t)(SM + 1024));
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
