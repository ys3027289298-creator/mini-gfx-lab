/* tilemap decode pipeline: every frame the tilemap stage decodes the same
   tile-sheet image before stamping its canvas.  cache=1 goes through the
   decode cache (first frame decodes, later frames hit); cache=0 keeps the
   legacy path and re-decodes the sheet every frame.  The stamped canvas must
   be byte-identical between the two modes. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dc_codec.h"

#define SHEET 64          /* decoded sheet: 4x4 grid of 16px tiles */
#define TILE 16
#define CANVAS 64
#define FRAMES 4          /* frames that each used to re-decode the sheet */
#define SHEET_SEED 0x51ce
#define SHEET_ID 0x1a2b3c4d5e6f7788ull

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned long now_us(void) { return dc_now_us(); }

/* deterministic tile-stamp transform shared by both modes */
static void compose(const unsigned char* sheet, unsigned char* canvas) {
    for (int cy = 0; cy < CANVAS / TILE; cy++)
        for (int cx = 0; cx < CANVAS / TILE; cx++) {
            int id = (cx * 7 + cy * 13) % 16;
            int tx = (id % (SHEET / TILE)) * TILE;
            int ty = (id / (SHEET / TILE)) * TILE;
            for (int y = 0; y < TILE; y++)
                memcpy(canvas + ((size_t)(cy * TILE + y) * CANVAS + (size_t)cx * TILE) * 4,
                       sheet + ((size_t)(ty + y) * SHEET + (size_t)tx) * 4, TILE * 4);
        }
}

typedef struct {
    unsigned char canvas[CANVAS * CANVAS * 4];
    uint64_t input_hash, output_hash;
    unsigned long decodes, hits;
} RunResult;

/* run the FRAMES-frame pipeline in one cache mode and collect evidence */
static int run(int use_cache, const unsigned char* blob, size_t blob_len,
               RunResult* r) {
    memset(r, 0, sizeof(*r));
    DcCache* cache = dc_cache_create(use_cache);
    if (!cache) { printf("cache create fail: %s\n", dc_last_error()); return -1; }

    printf("decode_time: module=tilemap mode=%s frame kind us\n",
           use_cache ? "cache_on" : "cache_off");
    const DcImage* img = NULL;
    for (int f = 0; f < FRAMES; f++) {
        unsigned long t0 = now_us();
        img = dc_get(cache, SHEET_ID, blob, blob_len);
        unsigned long us = now_us() - t0;
        if (!img) { printf("decode fail: %s\n", dc_last_error()); dc_cache_destroy(cache); return -1; }
        const char* kind = use_cache ? (f == 0 ? "miss" : "hit") : "decode";
        printf("decode_time: frame=%d kind=%s us=%lu\n", f, kind, us);
        printf("hitlog: %s\n", dc_last_log(cache));
        compose(img->pixels, r->canvas);
    }
    r->input_hash = img->pixel_hash;
    r->output_hash = dc_image_pixel_hash(CANVAS, CANVAS, DC_CHANNELS, r->canvas);
    r->decodes = dc_decodes(cache);
    r->hits = dc_hits(cache);
    dc_cache_destroy(cache);
    return 0;
}

int main(void) {
    printf("module=tilemap\n");
    size_t cap = (size_t)SHEET * SHEET * 4 * 2 + 64;
    unsigned char* blob = (unsigned char*)malloc(cap);
    if (!blob) { printf("all_ok=0\n"); return 1; }
    size_t blob_len = dc_build_blob(SHEET_SEED, SHEET, SHEET, blob, cap);
    check("blob_built", blob_len > 0);
    if (!blob_len) { printf("all_ok=0\n"); return 1; }

    RunResult on, off;
    check("run_cache_on", run(1, blob, blob_len, &on) == 0);
    check("run_cache_off", run(0, blob, blob_len, &off) == 0);

    /* core contract: with the cache, only the first frame decodes */
    check("on_decodes_once", on.decodes == 1);
    check("on_zero_redecode", on.decodes == 1 && on.hits == FRAMES - 1);
    /* legacy path still decodes every frame, and never logs a hit */
    check("off_decodes_every_frame", off.decodes == FRAMES && off.hits == 0);

    /* cache off must reproduce the cache-on pixels exactly */
    check("input_hash_matches", on.input_hash == off.input_hash && on.input_hash != 0);
    check("output_hash_matches", on.output_hash == off.output_hash);
    check("identical", memcmp(on.canvas, off.canvas, sizeof(on.canvas)) == 0);

    printf("pixel_hash: input=%016llx output_on=%016llx output_off=%016llx\n",
           (unsigned long long)on.input_hash,
           (unsigned long long)on.output_hash,
           (unsigned long long)off.output_hash);
    printf("counts: cache_on decodes=%lu hits=%lu | cache_off decodes=%lu hits=%lu\n",
           on.decodes, on.hits, off.decodes, off.hits);
    printf("repro=make -C tilemap bench\n");
    free(blob);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
