/* fontatlas image pipeline: pack glyph bitmaps from a source sheet into an
   atlas image (reverse order, so the output is a real transform of the input).
   shared=1 reads the sheet through the shared buffer (zero-copy); shared=0
   first makes a private copy, the old behaviour. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "imgbuf.h"

#define SHEET 64   /* input: 4x4 grid of 16px glyph bitmaps */
#define GLYPH 16
#define ATLAS 64   /* output atlas */
#define TIMING_ITERS 1000

static const unsigned char* g_last_src = NULL;

/* pack the 16 glyphs into the atlas in reverse id order */
static void pack(const unsigned char* sheet, unsigned char* atlas) {
    for (int i = 0; i < 16; i++) {
        int id = 15 - i;
        int sx = (id % (SHEET / GLYPH)) * GLYPH;
        int sy = (id / (SHEET / GLYPH)) * GLYPH;
        int dx = (i % (ATLAS / GLYPH)) * GLYPH;
        int dy = (i / (ATLAS / GLYPH)) * GLYPH;
        for (int y = 0; y < GLYPH; y++)
            memcpy(atlas + ((size_t)(dy + y) * ATLAS + (size_t)dx) * 4,
                   sheet + ((size_t)(sy + y) * SHEET + (size_t)sx) * 4, GLYPH * 4);
    }
}

static int run_pipeline(ImgBuf* in, ImgBuf* out, int shared) {
    if (!in || !out) { fprintf(stderr, "fontatlas pipeline: null buffer\n"); return -1; }
    ImgBuf* hold;
    if (shared) {
        hold = imgbuf_share(in);   /* zero-copy: same pixels */
    } else {
        hold = imgbuf_create(imgbuf_width(in), imgbuf_height(in));
        if (hold && imgbuf_copy_into(hold, in) != 0) { imgbuf_release(hold); hold = NULL; }
    }
    if (!hold) { fprintf(stderr, "fontatlas pipeline: %s\n", imgbuf_last_error()); return -1; }
    g_last_src = imgbuf_pixels_ro(hold);
    pack(g_last_src, imgbuf_pixels(out));
    imgbuf_release(hold);
    return 0;
}

static unsigned long now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long)ts.tv_sec * 1000000ul + (unsigned long)ts.tv_nsec / 1000ul;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    printf("module=fontatlas\n");
    ImgBuf* in = imgbuf_create(SHEET, SHEET);
    if (!in) { printf("all_ok=0\n"); return 1; }
    unsigned char* px = imgbuf_pixels(in);
    for (size_t i = 0; i < imgbuf_bytes(in); i++) px[i] = (unsigned char)(i * 7u + 3u);
    size_t input_bytes = imgbuf_bytes(in);

    /* invalid interface usage must error */
    check("reject_null_in", run_pipeline(NULL, NULL, 1) == -1);
    check("reject_bad_create", imgbuf_create(0, 8) == NULL);
    check("reject_null_share", imgbuf_share(NULL) == NULL);
    check("error_reported", strlen(imgbuf_last_error()) > 0);

    /* integration off vs on: outputs must be pixel-identical */
    ImgBuf* out_p = imgbuf_create(ATLAS, ATLAS);
    ImgBuf* out_s = imgbuf_create(ATLAS, ATLAS);
    check("run_private", run_pipeline(in, out_p, 0) == 0);
    check("run_shared", run_pipeline(in, out_s, 1) == 0);
    check("identical", memcmp(imgbuf_pixels(out_p), imgbuf_pixels(out_s),
                              imgbuf_bytes(out_s)) == 0);

    /* content: atlas slot 0 holds glyph id 15 (bottom-right of the sheet) */
    const unsigned char* glyph15 = px + ((size_t)48 * SHEET + 48) * 4;
    check("slot0_is_glyph15", memcmp(imgbuf_pixels(out_s), glyph15, GLYPH * 4) == 0);
    imgbuf_release(out_p);
    imgbuf_release(out_s);

    /* shared-mode bench: peak <= 2x input, no second copy */
    imgbuf_reset_peak();
    unsigned long c0 = imgbuf_copy_count();
    ImgBuf* tmp = imgbuf_create(ATLAS, ATLAS);
    run_pipeline(in, tmp, 1);
    unsigned long shared_peak = imgbuf_peak_bytes();
    unsigned long shared_copies = imgbuf_copy_count() - c0;
    int same_ptr = g_last_src == imgbuf_pixels_ro(in);
    imgbuf_release(tmp);
    check("peak_ok", shared_peak <= 2 * input_bytes);
    check("no_second_copy", shared_copies == 0 && same_ptr);

    /* private-mode bench (integration off): the old copy shows in the peak */
    imgbuf_reset_peak();
    c0 = imgbuf_copy_count();
    tmp = imgbuf_create(ATLAS, ATLAS);
    run_pipeline(in, tmp, 0);
    unsigned long priv_peak = imgbuf_peak_bytes();
    unsigned long priv_copies = imgbuf_copy_count() - c0;
    imgbuf_release(tmp);
    check("private_over_2x", priv_peak > 2 * input_bytes && priv_copies == 1);

    /* timing: total elapsed over TIMING_ITERS pipeline runs */
    ImgBuf* tout = imgbuf_create(ATLAS, ATLAS);
    unsigned long t0 = now_us();
    for (int i = 0; i < TIMING_ITERS; i++) run_pipeline(in, tout, 1);
    unsigned long shared_us = now_us() - t0;
    t0 = now_us();
    for (int i = 0; i < TIMING_ITERS; i++) run_pipeline(in, tout, 0);
    unsigned long priv_us = now_us() - t0;
    imgbuf_release(tout);

    printf("mode=shared peak_bytes=%lu input_bytes=%lu copies=%lu same_ptr=%d elapsed_us=%lu iters=%d\n",
           shared_peak, (unsigned long)input_bytes, shared_copies, same_ptr,
           shared_us, TIMING_ITERS);
    printf("mode=private peak_bytes=%lu input_bytes=%lu copies=%lu elapsed_us=%lu iters=%d\n",
           priv_peak, (unsigned long)input_bytes, priv_copies, priv_us, TIMING_ITERS);

    imgbuf_release(in);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
