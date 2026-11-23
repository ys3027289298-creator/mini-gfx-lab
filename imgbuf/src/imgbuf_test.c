/* imgbuf self-test: refcounting, zero-copy sharing, byte accounting,
   and error reporting on invalid interface usage. */
#include <stdio.h>
#include <string.h>
#include "imgbuf.h"

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    ImgBuf* a = imgbuf_create(8, 8);
    check("create_ok", a != NULL);
    check("bytes_8x8", imgbuf_bytes(a) == 8 * 8 * 4);
    check("dims", imgbuf_width(a) == 8 && imgbuf_height(a) == 8);
    check("live_after_create", imgbuf_live_bytes() == 256);

    unsigned char* px = imgbuf_pixels(a);
    int zeroed = 1;
    for (int i = 0; i < 256; i++) if (px[i]) { zeroed = 0; break; }
    check("zeroed", zeroed);
    px[0] = 42;

    /* share: same buffer, same pixels, no copy counted */
    unsigned long c0 = imgbuf_copy_count();
    ImgBuf* b = imgbuf_share(a);
    check("share_same_buf", b == a);
    check("share_same_pixels", imgbuf_pixels(b) == px);
    check("share_no_copy", imgbuf_copy_count() == c0);
    check("share_sees_data", imgbuf_pixels_ro(b)[0] == 42);

    /* copy_into: the only counted copy path */
    ImgBuf* d = imgbuf_create(8, 8);
    check("copy_ok", imgbuf_copy_into(d, a) == 0);
    check("copy_counted", imgbuf_copy_count() == c0 + 1);
    check("copy_data", imgbuf_pixels(d)[0] == 42);
    check("copy_distinct", imgbuf_pixels(d) != px);

    /* live/peak accounting */
    check("live_two_bufs", imgbuf_live_bytes() == 512);
    check("peak_two_bufs", imgbuf_peak_bytes() == 512);
    imgbuf_release(d);
    check("live_after_free", imgbuf_live_bytes() == 256);
    check("peak_holds", imgbuf_peak_bytes() == 512);
    imgbuf_reset_peak();
    check("peak_reset_to_live", imgbuf_peak_bytes() == 256);

    /* refcount: buffer survives until the last release */
    imgbuf_release(b);   /* refs 2 -> 1 */
    check("refcount_holds", imgbuf_live_bytes() == 256 && imgbuf_pixels(a)[0] == 42);
    imgbuf_release(a);   /* refs 1 -> 0, freed */
    check("live_after_last", imgbuf_live_bytes() == 0);

    /* invalid interface usage must fail and report an error */
    check("reject_zero_w", imgbuf_create(0, 8) == NULL);
    check("reject_neg_h", imgbuf_create(8, -1) == NULL);
    check("reject_null_share", imgbuf_share(NULL) == NULL);
    check("reject_null_pixels", imgbuf_pixels(NULL) == NULL);
    check("reject_null_bytes", imgbuf_bytes(NULL) == 0);
    ImgBuf* x = imgbuf_create(8, 8);
    ImgBuf* y = imgbuf_create(4, 4);
    check("reject_mismatch", imgbuf_copy_into(x, y) == -1);
    check("reject_null_copy", imgbuf_copy_into(x, NULL) == -1);
    imgbuf_release(NULL);
    check("error_string", strlen(imgbuf_last_error()) > 0);
    imgbuf_release(x);
    imgbuf_release(y);
    check("live_drained", imgbuf_live_bytes() == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
