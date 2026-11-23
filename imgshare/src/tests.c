/* imgshare_tests: unit checks for sharing, copy-on-write, interning and the
   illegal-interface contract.  Every public entry point that can be called
   wrongly is exercised here; each wrong call must report an error. */
#include <stdio.h>
#include <string.h>

#include "imgshare.h"

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    ImgsImage img;
    ImgsRef a, b, w;
    uint64_t ha, hb;

    imgs_stats_reset();
    check("image_init", imgs_image_init(&img) == IMGS_OK);

    /* create is one allocation, one ref, not writable-shared yet */
    a = imgs_create(&img, 64, 64, IMGS_FMT_RGBA8, 100, "tex");
    check("create_ok", a != NULL);
    check("create_refs_1", imgs_refs(a) == 1);
    check("create_bytes", imgs_bytes(a) == 64u * 64u * 4u);
    check("create_counted", imgs_stats()->creates == 1);
    check("create_peak_is_input", imgs_stats()->peak_bytes <= 64u * 64u * 4u + 128u);
    check("fill_ok", imgs_pixel_fill(a, 7) == IMGS_OK);
    ha = imgs_pixel_hash(a);

    /* open by key returns the same buffer, no copy */
    b = NULL;
    check("open_ok", imgs_open(&img, 100, &b) == IMGS_OK);
    check("open_same_buffer", b == a);
    check("open_refs_2", imgs_refs(a) == 2);
    check("open_no_copy", imgs_stats()->copies == 0);
    check("open_avoided_copy", imgs_stats()->avoided_copies >= 1);
    check("open_peak_unchanged",
          imgs_stats()->peak_bytes <= 64u * 64u * 4u + 128u);

    /* hashes agree because it is the same pixels */
    hb = imgs_pixel_hash(b);
    check("hash_same", ha == hb);

    /* shared handle is not writable in place; COW gives a private copy */
    check("shared_busy", imgs_pixel_fill(b, 9) == IMGS_EBUSY);
    w = imgs_make_writable(&img, a);
    check("cow_returns_new", w != NULL && w != a);
    check("cow_refs_1", imgs_refs(w) == 1);
    check("cow_counts_copy", imgs_stats()->writable_copies == 1);
    check("cow_copy_bytes", imgs_stats()->copy_bytes == 64u * 64u * 4u);
    check("cow_same_hash_initially", imgs_pixel_hash(w) == ha);
    check("cow_writable", imgs_pixel_fill(w, 11) == IMGS_OK);
    check("cow_diverged", imgs_pixel_hash(w) != ha);
    check("canonical_untouched", imgs_pixel_hash(a) == ha);
    ImgsRef probe = NULL;
    check("cow_not_interned", imgs_open(&img, 100, &probe) == IMGS_OK && probe == a);
    check("probe_release", imgs_release(&img, probe) == IMGS_OK);

    /* make_writable on a sole writable ref is a no-op */
    ImgsRef solo = imgs_create(&img, 8, 8, IMGS_FMT_GRAY8, 0, "solo");
    check("solo_create", solo != NULL);
    check("solo_no_copy", imgs_make_writable(&img, solo) == solo);

    /* release semantics */
    check("release_shared", imgs_release(&img, b) == IMGS_OK && imgs_refs(a) == 1);    check("release_cow", imgs_release(&img, w) == IMGS_OK);
    check("release_solo", imgs_release(&img, solo) == IMGS_OK);

    /* illegal interface: every one of these must be rejected */
    check("bad_init_null", imgs_image_init(NULL) == IMGS_EINVAL);
    check("bad_create_null_img",
          imgs_create(NULL, 4, 4, IMGS_FMT_RGBA8, 0, NULL) == NULL);
    check("bad_create_w0", imgs_create(&img, 0, 4, IMGS_FMT_RGBA8, 0, NULL) == NULL);
    check("bad_create_h0", imgs_create(&img, 4, 0, IMGS_FMT_RGBA8, 0, NULL) == NULL);
    check("bad_create_neg", imgs_create(&img, -4, 4, IMGS_FMT_RGBA8, 0, NULL) == NULL);
    check("bad_create_fmt",
          imgs_create(&img, 4, 4, (ImgsFormat)0, 0, NULL) == NULL);
    check("bad_create_dupkey",
          imgs_create(&img, 4, 4, IMGS_FMT_RGBA8, 100, NULL) == NULL);
    check("bad_acquire_null", imgs_acquire(&img, NULL) == NULL);
    check("bad_acquire_null_img", imgs_acquire(NULL, a) == NULL);
    check("bad_open_null_out", imgs_open(&img, 100, NULL) != IMGS_OK);
    check("bad_open_zero", imgs_open(&img, 0, &b) == IMGS_EINVAL);
    check("bad_open_missing", imgs_open(&img, 424242, &b) == IMGS_ENOENT);
    check("bad_release_null", imgs_release(&img, NULL) == IMGS_EINVAL);
    check("bad_writable_null", imgs_make_writable(&img, NULL) == NULL);
    check("bad_hash_null", imgs_pixel_hash(NULL) == 0);
    check("bad_bytes_null", imgs_bytes(NULL) == 0);
    check("bad_refs_null", imgs_refs(NULL) == -1);
    check("bad_fill_null", imgs_pixel_fill(NULL, 0) == IMGS_EINVAL);
    check("illegal_counted", imgs_stats()->illegal_calls >= 17);

    /* a handle that does not belong to this image is rejected */
    ImgsImage other;
    imgs_image_init(&other);
    check("bad_foreign_acquire", imgs_acquire(&other, a) == NULL);
    check("bad_foreign_release", imgs_release(&other, a) == IMGS_EINVAL);
    imgs_image_destroy(&other);

    /* capacity: filling the intern table must fail cleanly, not corrupt */
    ImgsImage full;
    imgs_image_init(&full);
    int made = 0;
    while (imgs_create(&full, 2, 2, IMGS_FMT_GRAY8,
                       (uint64_t)(0x5000 + made), NULL) != NULL) {
        made++;
    }
    check("intern_table_caps", made == IMGS_MAX_BUFFERS);
    imgs_image_destroy(&full);

    imgs_release(&img, a);
    imgs_image_destroy(&img);
    check("destroy_frees_all", imgs_stats()->live_bytes == 0);
    check("destroy_no_buffers", imgs_stats()->live_buffers == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
