/* lod decode pipeline: each frame the LOD selector builds the next mip level
   (2x2 box downsample) of the source image it just decoded.  cache=1 decodes
   the source once and later frames hit; cache=0 re-decodes every frame.  The
   mip pixels must be byte-identical between modes. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dc_codec.h"

#define SRC 64          /* input image */
#define MIP (SRC / 2)   /* output mip level */
#define FRAMES 4
#define SRC_SEED 0x10d5
#define SRC_ID 0x4d5e6f778899aabbull

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void downsample(const unsigned char* src, unsigned char* dst) {
    for (int y = 0; y < MIP; y++)
        for (int x = 0; x < MIP; x++)
            for (int c = 0; c < 4; c++) {
                unsigned s = src[((size_t)(2 * y) * SRC + 2 * x) * 4 + c] +
                             src[((size_t)(2 * y) * SRC + 2 * x + 1) * 4 + c] +
                             src[((size_t)(2 * y + 1) * SRC + 2 * x) * 4 + c] +
                             src[((size_t)(2 * y + 1) * SRC + 2 * x + 1) * 4 + c];
                dst[((size_t)y * MIP + (size_t)x) * 4 + c] = (unsigned char)(s / 4);
            }
}

typedef struct {
    unsigned char mip[MIP * MIP * 4];
    uint64_t input_hash, output_hash;
    unsigned long decodes, hits;
} RunResult;

static int run(int use_cache, const unsigned char* blob, size_t blob_len,
               RunResult* r) {
    memset(r, 0, sizeof(*r));
    DcCache* cache = dc_cache_create(use_cache);
    if (!cache) return -1;

    printf("decode_time: module=lod mode=%s frame kind us\n",
           use_cache ? "cache_on" : "cache_off");
    const DcImage* img = NULL;
    for (int f = 0; f < FRAMES; f++) {
        unsigned long t0 = dc_now_us();
        img = dc_get(cache, SRC_ID, blob, blob_len);
        unsigned long us = dc_now_us() - t0;
        if (!img) { dc_cache_destroy(cache); return -1; }
        const char* kind = use_cache ? (f == 0 ? "miss" : "hit") : "decode";
        printf("decode_time: frame=%d kind=%s us=%lu\n", f, kind, us);
        printf("hitlog: %s\n", dc_last_log(cache));
        downsample(img->pixels, r->mip);
    }
    r->input_hash = img->pixel_hash;
    r->output_hash = dc_image_pixel_hash(MIP, MIP, DC_CHANNELS, r->mip);
    r->decodes = dc_decodes(cache);
    r->hits = dc_hits(cache);
    dc_cache_destroy(cache);
    return 0;
}

int main(void) {
    printf("module=lod\n");
    size_t cap = (size_t)SRC * SRC * 4 * 2 + 64;
    unsigned char* blob = (unsigned char*)malloc(cap);
    if (!blob) { printf("all_ok=0\n"); return 1; }
    size_t blob_len = dc_build_blob(SRC_SEED, SRC, SRC, blob, cap);
    check("blob_built", blob_len > 0);
    if (!blob_len) { printf("all_ok=0\n"); return 1; }

    RunResult on, off;
    check("run_cache_on", run(1, blob, blob_len, &on) == 0);
    check("run_cache_off", run(0, blob, blob_len, &off) == 0);

    check("on_decodes_once", on.decodes == 1);
    check("on_zero_redecode", on.decodes == 1 && on.hits == FRAMES - 1);
    check("off_decodes_every_frame", off.decodes == FRAMES && off.hits == 0);
    check("input_hash_matches", on.input_hash == off.input_hash && on.input_hash != 0);
    check("output_hash_matches", on.output_hash == off.output_hash);
    check("identical", memcmp(on.mip, off.mip, sizeof(on.mip)) == 0);

    printf("pixel_hash: input=%016llx output_on=%016llx output_off=%016llx\n",
           (unsigned long long)on.input_hash,
           (unsigned long long)on.output_hash,
           (unsigned long long)off.output_hash);
    printf("counts: cache_on decodes=%lu hits=%lu | cache_off decodes=%lu hits=%lu\n",
           on.decodes, on.hits, off.decodes, off.hits);
    printf("repro=make -C lod bench\n");
    free(blob);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
