/* fontatlas decode pipeline: every frame the glyph packer decodes the same
   source glyph sheet before laying glyphs into the atlas.  cache=1 decodes
   the sheet once and serves later frames from the cache; cache=0 re-decodes
   every frame.  The packed atlas must be byte-identical in both modes. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dc_codec.h"

#define SHEET 64          /* 4x4 grid of 16px glyph bitmaps */
#define GLYPH 16
#define ATLAS 64
#define FRAMES 4
#define SHEET_SEED 0x6a11
#define SHEET_ID 0x2b3c4d5e6f778899ull

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

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

typedef struct {
    unsigned char atlas[ATLAS * ATLAS * 4];
    uint64_t input_hash, output_hash;
    unsigned long decodes, hits;
} RunResult;

static int run(int use_cache, const unsigned char* blob, size_t blob_len,
               RunResult* r) {
    memset(r, 0, sizeof(*r));
    DcCache* cache = dc_cache_create(use_cache);
    if (!cache) return -1;

    printf("decode_time: module=fontatlas mode=%s frame kind us\n",
           use_cache ? "cache_on" : "cache_off");
    const DcImage* img = NULL;
    for (int f = 0; f < FRAMES; f++) {
        unsigned long t0 = dc_now_us();
        img = dc_get(cache, SHEET_ID, blob, blob_len);
        unsigned long us = dc_now_us() - t0;
        if (!img) { dc_cache_destroy(cache); return -1; }
        const char* kind = use_cache ? (f == 0 ? "miss" : "hit") : "decode";
        printf("decode_time: frame=%d kind=%s us=%lu\n", f, kind, us);
        printf("hitlog: %s\n", dc_last_log(cache));
        pack(img->pixels, r->atlas);
    }
    r->input_hash = img->pixel_hash;
    r->output_hash = dc_image_pixel_hash(ATLAS, ATLAS, DC_CHANNELS, r->atlas);
    r->decodes = dc_decodes(cache);
    r->hits = dc_hits(cache);
    dc_cache_destroy(cache);
    return 0;
}

int main(void) {
    printf("module=fontatlas\n");
    size_t cap = (size_t)SHEET * SHEET * 4 * 2 + 64;
    unsigned char* blob = (unsigned char*)malloc(cap);
    if (!blob) { printf("all_ok=0\n"); return 1; }
    size_t blob_len = dc_build_blob(SHEET_SEED, SHEET, SHEET, blob, cap);
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
    check("identical", memcmp(on.atlas, off.atlas, sizeof(on.atlas)) == 0);

    printf("pixel_hash: input=%016llx output_on=%016llx output_off=%016llx\n",
           (unsigned long long)on.input_hash,
           (unsigned long long)on.output_hash,
           (unsigned long long)off.output_hash);
    printf("counts: cache_on decodes=%lu hits=%lu | cache_off decodes=%lu hits=%lu\n",
           on.decodes, on.hits, off.decodes, off.hits);
    printf("repro=make -C fontatlas bench\n");
    free(blob);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
