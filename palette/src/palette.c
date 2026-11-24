
/* palette implementation: LUT management and swatch rendering.
   Failure points all flow through gfault block 0x50. */
#include "palette.h"
#include <string.h>

void palette_init(Palette* pal) {
    memset(pal, 0, sizeof(*pal));
}

static int color_valid(Rgba c) {
    (void)c; /* every byte value 0..255 is representable; kept as a hook
                so malformed HDR/out-of-gamut inputs get a code later */
    return 1;
}

int palette_set(Palette* pal, int index, Rgba color) {
    if (pal == 0)
        return gfault_report(GFAULT_SEG_PALETTE, GFAULT_PAL_BAD_ARG, __LINE__);
    if (index < 0 || index >= PALETTE_SIZE)
        return gfault_report(GFAULT_SEG_PALETTE, GFAULT_PAL_INDEX_RANGE, __LINE__);
    if (!color_valid(color))
        return gfault_report(GFAULT_SEG_PALETTE, GFAULT_PAL_BAD_COLOR, __LINE__);
    if (!pal->used[index]) pal->count++;
    pal->lut[index] = color;
    pal->used[index] = 1;
    return index;
}

int palette_add(Palette* pal, Rgba color) {
    if (pal == 0)
        return gfault_report(GFAULT_SEG_PALETTE, GFAULT_PAL_BAD_ARG, __LINE__);
    if (!color_valid(color))
        return gfault_report(GFAULT_SEG_PALETTE, GFAULT_PAL_BAD_COLOR, __LINE__);
    for (int i = 0; i < PALETTE_SIZE; i++) {
        if (!pal->used[i]) return palette_set(pal, i, color);
    }
    return gfault_report(GFAULT_SEG_PALETTE, GFAULT_PAL_LUT_FULL, __LINE__);
}

int palette_get(const Palette* pal, int index, Rgba* out) {
    if (pal == 0 || out == 0)
        return gfault_report(GFAULT_SEG_PALETTE, GFAULT_PAL_BAD_ARG, __LINE__);
    if (index < 0 || index >= PALETTE_SIZE)
        return gfault_report(GFAULT_SEG_PALETTE, GFAULT_PAL_INDEX_RANGE, __LINE__);
    if (!pal->used[index])
        return gfault_report(GFAULT_SEG_PALETTE, GFAULT_PAL_INDEX_RANGE, __LINE__);
    *out = pal->lut[index];
    return index;
}

int palette_resolve(const Palette* pal, int index, Rgba* out) {
    return palette_get(pal, index, out);
}

int palette_render_swaths(const Palette* pal, FrameBuf* fb, int x0, int x1) {
    if (pal == 0 || fb == 0)
        return gfault_report(GFAULT_SEG_PALETTE, GFAULT_PAL_BAD_ARG, __LINE__);
    if (x0 < 0) x0 = 0;
    if (x1 >= PALETTE_SIZE) x1 = PALETTE_SIZE - 1;
    if (x0 > x1)
        return gfault_report(GFAULT_SEG_PALETTE, GFAULT_PAL_BAD_ARG, __LINE__);

    const FbRegion* reg = fb_region(GFAULT_SEG_PALETTE);
    int drawn = 0;
    int slots = x1 - x0 + 1;
    int sw = reg->w / slots;              /* swatch width, >= 1 for <=160 */
    if (sw < 1)
        return gfault_report(GFAULT_SEG_PALETTE, GFAULT_PAL_BAD_ARG, __LINE__);

    for (int i = 0; i < slots; i++) {
        int idx = x0 + i;
        Rgba color;
        int rc = palette_resolve(pal, idx, &color);
        if (rc < 0) return rc;            /* segment latch happens in get() */
        int sx0 = reg->x + i * sw;
        int sx1 = (i + 1 == slots) ? reg->x + reg->w - 1 : sx0 + sw - 1;
        fb_rect(fb, sx0, reg->y, sx1, reg->y + reg->h - 1, color);
        drawn++;
    }
    return drawn;
}
