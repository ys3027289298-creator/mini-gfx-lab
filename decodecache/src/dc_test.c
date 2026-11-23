/* decodecache self-test: codec round-trip, the zero-redecode cache contract,
   cache-off pixel identity, corruption rejection and capacity eviction. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dc_codec.h"

#define W 64
#define H 48
#define IMG_ID 0x3141592653589793ull
#define N_CONSUMERS 4

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned char* make_blob(size_t* len_out) {
    size_t cap = (size_t)W * H * DC_CHANNELS * 2 + 64;
    unsigned char* blob = (unsigned char*)malloc(cap);
    if (!blob) return NULL;
    size_t n = dc_build_blob(0x51ce, W, H, blob, cap);
    if (n == 0) { free(blob); return NULL; }
    *len_out = n;
    return blob;
}

int main(void) {
    size_t blob_len = 0;
    unsigned char* blob = make_blob(&blob_len);
    check("blob_built", blob != NULL && blob_len > 0);
    if (!blob) { printf("all_ok=0\n"); return 1; }

        /* deterministic build: same seed/size -> identical bytes */
    size_t len2 = 0;
    unsigned char* blob2 = make_blob(&len2);
    check("blob_deterministic", blob2 && len2 == blob_len &&
          memcmp(blob, blob2, blob_len) == 0);

    /* independent round-trip: caller-known pixels must come back byte-for-byte.
       The reference pixels are constructed here, never by the encoder, so an
       encoder/decoder pair that shared a bias could not pass. */
    {
        const int rw = 37, rh = 29;
        size_t rb = (size_t)rw * rh * DC_CHANNELS;
        unsigned char* ref = (unsigned char*)malloc(rb);
        unsigned char* ebuf = (unsigned char*)malloc(rb * 2 + 64);
        for (size_t i = 0; i < rb; i++)
            ref[i] = (unsigned char)((i * 31 + (i >> 3) * 7 + 5) & 0xff);
        size_t elen = dc_encode_rgba(rw, rh, ref, ebuf, rb * 2 + 64);
        check("encode_rgba_ok", elen > 0);
        DcCache* rt = dc_cache_create(0);
        const DcImage* got = dc_get(rt, 0x77, ebuf, elen);
        check("roundtrip_dims", got && got->w == rw && got->h == rh && got->bytes == rb);
        check("roundtrip_byte_identical",
              got && memcmp(got->pixels, ref, rb) == 0);
        dc_cache_destroy(rt);
        free(ref); free(ebuf);
    }

    /* ---- cache ON: four consumers decode the same image ------------------ */
    DcCache* on = dc_cache_create(1);
    check("cache_created", on != NULL && dc_cache_enabled(on));
    const DcImage* first = dc_get(on, IMG_ID, blob, blob_len);
    check("first_get_ok", first != NULL && first->w == W && first->h == H);
    check("first_is_miss", dc_misses(on) == 1 && dc_decodes(on) == 1);

    uint64_t on_hash = first->pixel_hash;
    const DcImage* views[N_CONSUMERS];
    views[0] = first;
    for (int i = 1; i < N_CONSUMERS; i++)
        views[i] = dc_get(on, IMG_ID, blob, blob_len);

    int all_hit = 1, same_pixels = 1;
    for (int i = 0; i < N_CONSUMERS; i++) {
        if (!views[i]) all_hit = 0;
        else if (views[i]->pixel_hash != on_hash) same_pixels = 0;
    }
    check("all_consumers_served", all_hit);
    check("all_hashes_equal", same_pixels);
    /* the core contract: after the first decode, re-decode count is zero */
    check("hits_recorded", dc_hits(on) == N_CONSUMERS - 1);
    check("decodes_exactly_one", dc_decodes(on) == 1);
    check("zero_redecode", dc_decodes(on) == 1 && dc_hits(on) == N_CONSUMERS - 1);
    printf("hitlog: %s\n", dc_last_log(on));

    /* a second distinct asset decodes once on its own */
    unsigned char* other = (unsigned char*)malloc((size_t)W * H * 4 * 2 + 64);
    size_t on2 = dc_build_blob(0x7ab1, W, H, other, (size_t)W * H * 4 * 2 + 64);
    check("second_blob_built", on2 > 0);
    const DcImage* img2 = dc_get(on, IMG_ID + 1, other, on2);
    check("second_asset_decodes", img2 != NULL && dc_decodes(on) == 2);
    const DcImage* img2b = dc_get(on, IMG_ID + 1, other, on2);
    check("second_asset_hits", img2b == img2 && dc_decodes(on) == 2);
    check("distinct_assets_distinct_hash", on_hash != img2->pixel_hash);

    /* ---- cache OFF: every call re-decodes, pixels stay identical --------- */
    DcCache* off = dc_cache_create(0);
    check("off_cache_disabled", !dc_cache_enabled(off));
    uint64_t off_hashes[N_CONSUMERS];
    const DcImage* off_views[N_CONSUMERS];
    for (int i = 0; i < N_CONSUMERS; i++) {
        off_views[i] = dc_get(off, IMG_ID, blob, blob_len);
        off_hashes[i] = off_views[i] ? off_views[i]->pixel_hash : 0;
    }
    check("off_decodes_every_time", dc_decodes(off) == N_CONSUMERS);
    check("off_no_hits", dc_hits(off) == 0);
    int off_same = 1;
    for (int i = 0; i < N_CONSUMERS; i++)
        if (off_hashes[i] != on_hash) off_same = 0;
    check("off_pixels_match_cache_on", off_same);
    /* byte-for-byte, not just same hash */
    check("off_byte_identical",
          memcmp(off_views[0]->pixels, first->pixels, first->bytes) == 0);
    printf("hitlog: %s\n", dc_last_log(off));

    /* ---- corruption is rejected, never silently decoded ----------------- */
    unsigned char* bad = (unsigned char*)malloc(blob_len);
    memcpy(bad, blob, blob_len);
    bad[blob_len / 2] ^= 0xff;                       /* flips inside payload */
    check("corrupt_payload_rejected", dc_get(on, IMG_ID + 99, bad, blob_len) == NULL);
    memcpy(bad, blob, blob_len);
    bad[0] = 'X';                                    /* breaks magic */
    check("bad_magic_rejected", dc_get(on, IMG_ID + 99, bad, blob_len) == NULL);
    check("truncated_rejected", dc_get(on, IMG_ID + 99, blob, 7) == NULL);
    check("null_args_rejected", dc_get(NULL, 1, blob, blob_len) == NULL &&
          dc_get(on, 1, NULL, blob_len) == NULL &&
          dc_get(on, 1, blob, 0) == NULL);
    free(bad);

    /* ---- fixed capacity: more distinct assets than slots evict cleanly -- */
    DcCache* cap = dc_cache_create(1);
    unsigned char* tmp = (unsigned char*)malloc((size_t)W * H * 4 * 2 + 64);
    int evict_ok = 1;
    for (uint32_t s = 1; s <= DC_CACHE_CAP + 3; s++) {
        size_t l = dc_build_blob(s, W, H, tmp, (size_t)W * H * 4 * 2 + 64);
        if (dc_get(cap, 1000 + s, tmp, l) == NULL) evict_ok = 0;
    }
    check("eviction_survives_overfill", evict_ok);
    /* an evicted id now misses and re-decodes rather than returning stale */
    size_t l0 = dc_build_blob(1, W, H, tmp, (size_t)W * H * 4 * 2 + 64);
    unsigned long before = dc_decodes(cap);
    const DcImage* back = dc_get(cap, 1001, tmp, l0);
    check("evicted_entry_redecodes", back != NULL && dc_decodes(cap) == before + 1);
    free(tmp);

    dc_cache_destroy(cap);
    dc_cache_destroy(off);
    dc_cache_destroy(on);
    free(other);
    free(blob2);
    free(blob);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
