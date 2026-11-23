/* imgbuf: shared image buffer with refcounting and copy/byte accounting. */
#include "imgbuf.h"

#include <stdlib.h>
#include <string.h>

#define IMGBUF_BPP 4   /* RGBA8 */

struct ImgBuf {
    int w, h;
    size_t bytes;
    int refs;
    unsigned char* px;
};

static unsigned long g_copies = 0;
static unsigned long g_live = 0;
static unsigned long g_peak = 0;
static char g_err[128] = "no error";

static void set_err(const char* msg) {
    strncpy(g_err, msg, sizeof(g_err) - 1);
    g_err[sizeof(g_err) - 1] = '\0';
}

const char* imgbuf_last_error(void) { return g_err; }

ImgBuf* imgbuf_create(int w, int h) {
    if (w <= 0 || h <= 0) { set_err("imgbuf_create: width/height must be positive"); return NULL; }
    size_t bytes = (size_t)w * (size_t)h * IMGBUF_BPP;
    if (bytes / IMGBUF_BPP / (size_t)w != (size_t)h) {
        set_err("imgbuf_create: size overflow");
        return NULL;
    }
    ImgBuf* b = (ImgBuf*)malloc(sizeof(*b));
    if (!b) { set_err("imgbuf_create: out of memory"); return NULL; }
    b->px = (unsigned char*)calloc(bytes, 1);
    if (!b->px) { free(b); set_err("imgbuf_create: out of memory"); return NULL; }
    b->w = w;
    b->h = h;
    b->bytes = bytes;
    b->refs = 1;
    g_live += bytes;
    if (g_live > g_peak) g_peak = g_live;
    return b;
}

ImgBuf* imgbuf_share(ImgBuf* b) {
    if (!b) { set_err("imgbuf_share: null buffer"); return NULL; }
    b->refs++;
    return b;
}

void imgbuf_release(ImgBuf* b) {
    if (!b) { set_err("imgbuf_release: null buffer"); return; }
    if (--b->refs > 0) return;
    g_live -= b->bytes;
    free(b->px);
    free(b);
}

unsigned char* imgbuf_pixels(ImgBuf* b) {
    if (!b) { set_err("imgbuf_pixels: null buffer"); return NULL; }
    return b->px;
}

const unsigned char* imgbuf_pixels_ro(const ImgBuf* b) {
    if (!b) { set_err("imgbuf_pixels_ro: null buffer"); return NULL; }
    return b->px;
}

int imgbuf_width(const ImgBuf* b) {
    if (!b) { set_err("imgbuf_width: null buffer"); return 0; }
    return b->w;
}

int imgbuf_height(const ImgBuf* b) {
    if (!b) { set_err("imgbuf_height: null buffer"); return 0; }
    return b->h;
}

size_t imgbuf_bytes(const ImgBuf* b) {
    if (!b) { set_err("imgbuf_bytes: null buffer"); return 0; }
    return b->bytes;
}

int imgbuf_copy_into(ImgBuf* dst, const ImgBuf* src) {
    if (!dst || !src) { set_err("imgbuf_copy_into: null buffer"); return -1; }
    if (dst->w != src->w || dst->h != src->h) {
        set_err("imgbuf_copy_into: dimension mismatch");
        return -1;
    }
    memcpy(dst->px, src->px, dst->bytes);
    g_copies++;
    return 0;
}

unsigned long imgbuf_copy_count(void) { return g_copies; }
unsigned long imgbuf_live_bytes(void) { return g_live; }
unsigned long imgbuf_peak_bytes(void) { return g_peak; }
void imgbuf_reset_peak(void) { g_peak = g_live; }
