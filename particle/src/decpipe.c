/* particle decode pipeline: each frame the emitter stamps its sprite image
   additively into the framebuffer, after decoding that sprite.  cache=1
   decodes the sprite once and the later frames hit; cache=0 re-decodes every
   frame.  The framebuffer must be byte-identical between modes. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dc_codec.h"

#define SIZE 64
#define FRAMES 4
#define SPRITE_SEED 0x9a27
#define SPRITE_ID 0x3c4d5e6f778899aaull

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* additive saturating stamp at three fixed emitter offsets (clipped) */
static void stamp(const unsigned char* sprite, unsigned char* fb) {
    static const int off[3][2] = { {0, 0}, {16, 8}, {40, 32} };
    for (int s = 0; s < 3; s++)
        for (int y = 0; y < SIZE; y++)
            for (int x = 0; x < SIZE; x++) {
                int dx = off[s][0] + x, dy = off[s][1] + y;
                if (dx >= SIZE || dy >= SIZE) continue;
                for (int c = 0; c < 4; c++) {
                    int a = fb[((size_t)dy * SIZE + (size_t)dx) * 4 + c] +
                            sprite[((size_t)y * SIZE + (size_t)x) * 4 + c];
                    fb[((size_t)dy * SIZE + (size_t)dx) * 4 + c] =
                        (unsigned char)(a > 255 ? 255 : a);
                }
            }
}

typedef struct {
    unsigned char fb[SIZE * SIZE * 4];
    uint64_t input_hash, output_hash;
    unsigned long decodes, hits;
} RunResult;

static int run(int use_cache, const unsigned char* blob, size_t blob_len,
               RunResult* r) {
    memset(r, 0, sizeof(*r));
    DcCache* cache = dc_cache_create(use_cache);
    if (!cache) return -1;

    printf("decode_time: module=particle mode=%s frame kind us\n",
           use_cache ? "cache_on" : "cache_off");
    const DcImage* img = NULL;
    for (int f = 0; f < FRAMES; f++) {
        unsigned long t0 = dc_now_us();
        img = dc_get(cache, SPRITE_ID, blob, blob_len);
        unsigned long us = dc_now_us() - t0;
        if (!img) { dc_cache_destroy(cache); return -1; }
        const char* kind = use_cache ? (f == 0 ? "miss" : "hit") : "decode";
        printf("decode_time: frame=%d kind=%s us=%lu\n", f, kind, us);
        printf("hitlog: %s\n", dc_last_log(cache));
        /* each frame starts from a cleared buffer so the result is stable */
        memset(r->fb, 0, sizeof(r->fb));
        stamp(img->pixels, r->fb);
    }
    r->input_hash = img->pixel_hash;
    r->output_hash = dc_image_pixel_hash(SIZE, SIZE, DC_CHANNELS, r->fb);
    r->decodes = dc_decodes(cache);
    r->hits = dc_hits(cache);
    dc_cache_destroy(cache);
    return 0;
}

int main(void) {
    printf("module=particle\n");
    size_t cap = (size_t)SIZE * SIZE * 4 * 2 + 64;
    unsigned char* blob = (unsigned char*)malloc(cap);
    if (!blob) { printf("all_ok=0\n"); return 1; }
    size_t blob_len = dc_build_blob(SPRITE_SEED, SIZE, SIZE, blob, cap);
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
    check("identical", memcmp(on.fb, off.fb, sizeof(on.fb)) == 0);

    printf("pixel_hash: input=%016llx output_on=%016llx output_off=%016llx\n",
           (unsigned long long)on.input_hash,
           (unsigned long long)on.output_hash,
           (unsigned long long)off.output_hash);
    printf("counts: cache_on decodes=%lu hits=%lu | cache_off decodes=%lu hits=%lu\n",
           on.decodes, on.hits, off.decodes, off.hits);
    printf("repro=make -C particle bench\n");
    free(blob);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
