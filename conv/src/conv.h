/* conv: long impulse-response convolution in the frequency domain.
 *
 * Uniformly partitioned overlap-save:
 *   - the impulse response is split into `block_len`-tap partitions, each
 *     transformed once at startup and kept as a spectrum;
 *   - each input block is framed as [previous block | new block] and
 *     transformed once;
 *   - filter partition p convolves the input block from p blocks ago, so the
 *     output block is one inverse transform of sum_p X_{m-p} * H_p; taking
 *     samples [n, 2n) of that result avoids all circular wrap.
 * With P partitions a block costs two FFTs (one forward, one inverse) and P
 * complex multiply-adds of length nfft, instead of block_len * P multiply-adds
 * for direct convolution. Filter spectra are computed once at creation.
 *
 * Any positive block length is supported: nfft is the next power of two
 * >= 2 * block_len. All arithmetic is double precision.
 */
#ifndef CONV_H
#define CONV_H

#include <stddef.h>

typedef struct {
    int block_len;      /* input/output samples per block */
    int filter_len;     /* taps in the impulse response */
    int nfft;           /* power of two >= 2 * block_len */
    int partitions;     /* ceil(filter_len / block_len) */
    int head;           /* next slot in the input-spectrum delay line */
    double* hist;       /* block_len: previous input block, zeroed at start */
    double* frame;      /* 2 * nfft: framed input spectrum (re, im) */
    double* acc;        /* 2 * nfft: spectrum accumulator (re, im) */
    double* hspec;      /* partitions * 2 * nfft: filter spectra */
    double* fdl;        /* partitions * 2 * nfft: past input spectra */
    double* tw;         /* nfft: twiddles, nfft/2 complex numbers */
} Conv;

/* Create a convolver for the filter_len-tap impulse response h[0..M-1].
 * block_len is the number of input/output samples processed per block; any
 * positive value is accepted. Returns NULL on invalid arguments or allocation
 * failure. */
Conv* conv_create(const double* h, int filter_len, int block_len);

void conv_destroy(Conv* c);

/* Bytes allocated by conv_create, filter spectra included. Linear in
 * filter_len + block_len; nothing grows with the number of samples processed. */
size_t conv_bytes(const Conv* c);

/* Process exactly block_len samples. y receives the matching block_len
 * output samples, advancing the convolution from where it last stopped. Call
 * the function again with block_len zeros (up to ceil(filter_len/block_len)
 * times) to flush the filter tail. */
void conv_process_block(Conv* c, const double* x, double* y);

/* Process n samples; n must be a positive multiple of block_len. Returns 0
 * on success, -1 on a bad length. */
int conv_process(Conv* c, const double* x, double* y, int n);

/* One-shot linear convolution of x[0..N-1] with h[0..M-1] into
 * y[0..N+M-2], including the filter tail beyond the last input sample.
 * A block_len <= 0 picks a default block size. Returns 0 on success, -1 on
 * invalid arguments or allocation failure. */
int conv_run(const double* x, int N, const double* h, int M,
             int block_len, double* y);

#endif /* CONV_H */
