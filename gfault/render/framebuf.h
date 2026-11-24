/*
 * framebuf: fixed-size RGBA8888 software framebuffer used to composite the
 * five render segments into one image. Everything is caller-owned memory;
 * the module performs no heap allocation and never aborts on bad input
 * (out-of-range geometry is clipped or reported via gfault).
 */
#ifndef FRAMEBUF_H
#define FRAMEBUF_H

#include "gfault.h"

#define FB_W 320
#define FB_H 240

typedef struct { unsigned char r, g, b, a; } Rgba;

typedef struct {
    Rgba px[FB_H][FB_W];
} FrameBuf;

void fb_clear(FrameBuf* fb, Rgba color);
void fb_pixel(FrameBuf* fb, int x, int y, Rgba color);            /* clipped */
void fb_rect(FrameBuf* fb, int x0, int y0, int x1, int y1, Rgba color);
void fb_hline(FrameBuf* fb, int x0, int x1, int y, Rgba color);
void fb_vline(FrameBuf* fb, int x, int y0, int y1, Rgba color);

/* Write the framebuffer as a binary PPM (P6) plus a companion .sha256 of
   its raw RGBA bytes. Returns 0 on success, a negative gfault code on
   failure. No allocation: writes in fixed-size blocks via stdio. */
int fb_write_ppm(const FrameBuf* fb, const char* path);
int fb_hash_rgba(const FrameBuf* fb, unsigned char out32[32]);

/* Per-segment panel geometry, shared by the harness and the bridges. */
typedef struct { int x, y, w, h; } FbRegion;
const FbRegion* fb_region(GfaultSeg segment);

#endif /* FRAMEBUF_H */
