/*
 * palette: fixed 256-entry RGBA lookup table, index translation and swatch
 * rendering. This is the fifth render segment named in the failure
 * unification work. All storage is caller-owned; no heap allocation.
 *
 * Every failure returns a negative code from the gfault palette block
 * (0x50..0x5F) and is reported through the shared diagnostic sink, so the
 * compositor can skip the palette panel without taking the other four
 * segments down with it.
 */
#ifndef PALETTE_H
#define PALETTE_H

#include "gfault.h"
#include "framebuf.h"

#define PALETTE_SIZE 256

typedef struct {
    Rgba lut[PALETTE_SIZE];
    int used[PALETTE_SIZE];   /* slot holds a registered color */
    int count;
} Palette;

void palette_init(Palette* pal);

/* Register a color at an explicit index. Negative gfault code on error. */
int palette_set(Palette* pal, int index, Rgba color);

/* Allocate the next free slot; returns the index or a negative code. */
int palette_add(Palette* pal, Rgba color);

/* Look up an index. Out-of-range/unregistered slots are reported, not
   silently rendered. On failure *out is left untouched. */
int palette_get(const Palette* pal, int index, Rgba* out);

/* Remap a logical index through the LUT into an Rgba pixel. Failures
   return gfault codes; the caller decides whether to skip drawing. */
int palette_resolve(const Palette* pal, int index, Rgba* out);

/* Render the swatch strip for indices [x0..x1] into the palette panel of
   the framebuffer. Returns the number of swatches drawn, or a negative
   gfault code on the first failure; already-drawn swatches are kept. */
int palette_render_swaths(const Palette* pal, FrameBuf* fb, int x0, int x1);

#endif /* PALETTE_H */
