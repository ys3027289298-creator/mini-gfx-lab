/* imgshare_demo: four consumers that each need the same image, wired either
   onto one shared buffer (--share) or kept on the legacy copy-per-consumer
   path (--noshare).  The four consumers are the ones the task asks to wire:
     1. memtable   - the peak-memory table
     2. copylog    - the copy-count log
     3. pixelhash  - the per-consumer pixel hash
     4. repro      - the reproduce command
   Both modes must produce identical pixel hashes; only the memory/copy
   evidence is allowed to differ, and in --share the peak must stay under
   2x the input image. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "imgshare.h"

#define IMG_W 640
#define IMG_H 480
#define IMG_FMT IMGS_FMT_RGBA8
#define IMG_SEED 0x51ce
#define SRC_KEY 0x1a2b3c4d5e6f7788ull
#define N_CONSUMERS 4

static const char* const CONSUMER_NAMES[N_CONSUMERS] = {
    "memtable", "copylog", "pixelhash", "repro"
};

static int failures = 0;
static ImgsImage img;
static ImgsRef shared_img;
static ImgsRef held[N_CONSUMERS];
static uint64_t consumer_hash[N_CONSUMERS];
static uint64_t copies_run;      /* real memcpys of pixel data */
static uint64_t shares_taken;    /* shared handles handed out */

static void fail(const char* what) {
    printf("FAIL=%s\n", what);
    failures++;
}

/* ------------------------------------------------------- legacy copy path */

/* What every consumer used to do: allocate its own buffer and memcpy the
   whole image.  Uses the same allocator so the memory evidence is measured
   against one source of truth. */
static ImgsRef legacy_copy(int who) {
    ImgsRef c = imgs_create(&img, IMG_W, IMG_H, IMG_FMT, 0, "private-copy");
    if (c == NULL) return NULL;
    memcpy(c->pixels, shared_img->pixels, imgs_bytes(shared_img));
    copies_run++;
    printf("copylog: copy consumer=%s bytes=%zu\n",
           CONSUMER_NAMES[who], imgs_bytes(c));
    return c;
}

/* -------------------------------------------------------------- consumers */

static ImgsRef take(int who, int share_mode) {
    if (share_mode) {
        shares_taken++;
        /* memtable opens by key; the others acquire their existing handle.
           Both are the same shared buffer. */
        if (who == 0) {
            ImgsRef r = NULL;
            int rc = imgs_open(&img, SRC_KEY, &r);
            if (rc != IMGS_OK) return NULL;
            return r;
        }
        return imgs_acquire(&img, shared_img);
    }
    return legacy_copy(who);
}

/* consumer 1: peak-memory table (printed after everyone has taken a handle) */
static void emit_memtable(int share_mode, size_t input_bytes) {
    const ImgsStats* st = imgs_stats();
    double ratio = input_bytes ? (double)st->peak_bytes / (double)input_bytes : 0.0;
    int ok = ratio <= 2.0;
    printf("memtable:\n");
    printf("  mode=%s input_bytes=%zu peak_bytes=%zu ratio=%.4f limit=2.0000 ok=%d\n",
           share_mode ? "share" : "noshare", input_bytes, st->peak_bytes, ratio, ok);
    printf("peak_bytes=%zu\n", st->peak_bytes);
    printf("peak_ratio=%.6f\n", ratio);
    printf("peak_limit=2.000000\n");
    printf("peak_ok=%d\n", ok);
    /* the enabled design is required to pass; the baseline is expected to
       show the blow-up that motivated the change. */
    if (share_mode && !ok) fail("peak_over_2x_input");
}

/* consumer 2: copy-count log summary */
static void emit_copylog(int share_mode) {
    printf("copylog:\n");
    printf("  copies=%llu shares=%llu\n",
           (unsigned long long)copies_run, (unsigned long long)shares_taken);
    printf("copies=%llu\n", (unsigned long long)copies_run);
    printf("shares=%llu\n", (unsigned long long)shares_taken);
    if (share_mode && copies_run != 0) fail("share_mode_still_copies");
    if (!share_mode && copies_run != N_CONSUMERS) fail("baseline_copy_count");
}

/* consumer 3: per-consumer pixel hash; each consumer sees the same pixels */
static void emit_pixelhash(void) {
    int all_equal = 1, match_src = 1;
    uint64_t src = imgs_pixel_hash(shared_img);
    for (int i = 0; i < N_CONSUMERS; i++) {
        consumer_hash[i] = imgs_pixel_hash(held[i]);
        if (consumer_hash[i] != consumer_hash[0]) all_equal = 0;
        if (consumer_hash[i] != src) match_src = 0;
        printf("consumer.%s.hash=%016llx\n", CONSUMER_NAMES[i],
               (unsigned long long)consumer_hash[i]);
    }
    printf("input_hash=%016llx\n", (unsigned long long)src);
    printf("all_consumer_hashes_equal=%d\n", all_equal);
    printf("all_hashes_match_input=%d\n", match_src);
    if (!all_equal) fail("consumer_hashes_differ");
    if (!match_src) fail("consumer_hash_vs_input");
}

/* consumer 4: the reproduce command */
static void emit_repro(const char* argv0, int share_mode) {
    printf("repro=%s --%s\n", argv0, share_mode ? "share" : "noshare");
}

/* ----------------------------------------------------- illegal-interface probe */

/* Every illegal call below must return an error and be counted; silently
   succeeding is the failure this probe is looking for. */
static int probe_illegal(void) {
    int bad = 0;
    ImgsImage tmp;
    ImgsImage* nil = NULL;
    ImgsRef r = NULL;

    imgs_stats_reset();
    if (imgs_image_init(&tmp) != IMGS_OK) { printf("probe.init:unexpected_fail\n"); bad++; }

    r = imgs_create(nil, 8, 8, IMGS_FMT_RGBA8, 1, "x");
    printf("probe.create_null_image=%s\n", r == NULL ? "error" : "ACCEPTED");
    if (r != NULL) bad++;

    r = imgs_create(&tmp, 0, 8, IMGS_FMT_RGBA8, 1, "x");
    printf("probe.create_zero_w=%s\n", r == NULL ? "error" : "ACCEPTED");
    if (r != NULL) bad++;

    r = imgs_create(&tmp, 8, -3, IMGS_FMT_RGBA8, 1, "x");
    printf("probe.create_neg_h=%s\n", r == NULL ? "error" : "ACCEPTED");
    if (r != NULL) bad++;

    r = imgs_create(&tmp, 8, 8, (ImgsFormat)99, 1, "x");
    printf("probe.create_bad_fmt=%s\n", r == NULL ? "error" : "ACCEPTED");
    if (r != NULL) bad++;

    r = imgs_create(&tmp, IMGS_MAX_DIM + 1, 8, IMGS_FMT_RGBA8, 1, "x");
    printf("probe.create_oversize=%s\n", r == NULL ? "error" : "ACCEPTED");
    if (r != NULL) bad++;

    ImgsRef a = imgs_create(&tmp, 8, 8, IMGS_FMT_RGBA8, 77, "a");
    if (a == NULL) { printf("probe.setup_fail\n"); bad++; }
    r = imgs_create(&tmp, 8, 8, IMGS_FMT_RGBA8, 77, "dup");
    printf("probe.create_duplicate_key=%s\n", r == NULL ? "error" : "ACCEPTED");
    if (r != NULL) bad++;

    r = imgs_acquire(&tmp, NULL);
    printf("probe.acquire_null=%s\n", r == NULL ? "error" : "ACCEPTED");
    if (r != NULL) bad++;

    r = imgs_acquire(nil, a);
    printf("probe.acquire_null_image=%s\n", r == NULL ? "error" : "ACCEPTED");
    if (r != NULL) bad++;

    ImgsImage other;
    imgs_image_init(&other);
    r = imgs_acquire(&other, a);   /* handle does not belong to this image */
    printf("probe.acquire_foreign_handle=%s\n", r == NULL ? "error" : "ACCEPTED");
    if (r != NULL) bad++;

    int rc = imgs_release(&other, a);
    printf("probe.release_foreign_handle=%s\n", rc != IMGS_OK ? "error" : "ACCEPTED");
    if (rc == IMGS_OK) bad++;

    rc = imgs_open(&tmp, 0, &r);
    printf("probe.open_zero_key=%s\n", rc != IMGS_OK ? "error" : "ACCEPTED");
    if (rc == IMGS_OK) bad++;

    rc = imgs_open(&tmp, 0xdeadbeefull, &r);
    printf("probe.open_missing_key=%s\n", rc == IMGS_ENOENT ? "error" : "ACCEPTED");
    if (rc != IMGS_ENOENT) bad++;

    rc = imgs_open(nil, 77, &r);
    printf("probe.open_null_image=%s\n", rc != IMGS_OK ? "error" : "ACCEPTED");
    if (rc == IMGS_OK) bad++;

    r = imgs_make_writable(nil, a);
    printf("probe.writable_null_image=%s\n", r == NULL ? "error" : "ACCEPTED");
    if (r != NULL) bad++;

    uint64_t h = imgs_pixel_hash((ImgsRef)NULL);
    printf("probe.hash_null=%s\n", h == 0 ? "error" : "ACCEPTED");
    if (h != 0) bad++;

    printf("probe.illegal_calls=%llu\n",
           (unsigned long long)imgs_stats()->illegal_calls);
    if (imgs_stats()->illegal_calls == 0) bad++;

    imgs_image_destroy(&other);
    imgs_image_destroy(&tmp);
    printf("probe_bad=%d\n", bad);
    return bad;
}

/* -------------------------------------------------------------------- main */

int main(int argc, char** argv) {
    int share_mode = 1;
    if (argc > 1 && strcmp(argv[1], "--noshare") == 0) share_mode = 0;
    else if (argc > 1 && strcmp(argv[1], "--share") == 0) share_mode = 1;
    else if (argc > 1 && strcmp(argv[1], "--probe-illegal") == 0) {
        int bad = probe_illegal();
        return bad == 0 ? 0 : 1;
    }

    imgs_stats_reset();
    if (imgs_image_init(&img) != IMGS_OK) { printf("init fail\n"); return 1; }

    /* the input image: allocated once, through the same accounting path */
    shared_img = imgs_create(&img, IMG_W, IMG_H, IMG_FMT, SRC_KEY, "source");
    if (shared_img == NULL) { printf("source create fail\n"); return 1; }
    if (imgs_pixel_fill(shared_img, IMG_SEED) != IMGS_OK) {
        printf("source fill fail\n");
        return 1;
    }
    size_t input_bytes = imgs_bytes(shared_img);

    printf("mode=%s\n", share_mode ? "share" : "noshare");
    printf("input_bytes=%zu\n", input_bytes);

    for (int i = 0; i < N_CONSUMERS; i++) {
        held[i] = take(i, share_mode);
        if (held[i] == NULL) { fail("consumer_take"); return 1; }
    }

    emit_pixelhash();
    emit_copylog(share_mode);
    emit_memtable(share_mode, input_bytes);
    emit_repro(argv[0], share_mode);

    printf("all_ok=%d\n", failures == 0);

    /* release in reverse; shared handles all point at one buffer */
    for (int i = N_CONSUMERS - 1; i >= 0; i--)
        if (held[i] != NULL) imgs_release(&img, held[i]);
    imgs_image_destroy(&img);
    if (imgs_stats()->live_bytes != 0) fail("leak_after_destroy");

    return failures == 0 ? 0 : 1;
}
