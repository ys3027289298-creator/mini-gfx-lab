
/* Bridge: palette segment -> framebuffer panel. The palette module is new
   in this change, so there is no pre-change source variant; this bridge
   links the current palette code directly. */
#include "palette.h"

int bridge_palette_render(FrameBuf* fb) {
    static Palette pal;
    palette_init(&pal);
    static const Rgba ramp[12] = {
        { 20,  20,  40, 255}, { 40,  50,  90, 255}, { 60,  90, 150, 255},
        { 80, 140, 200, 255}, {110, 190, 220, 255}, {160, 225, 220, 255},
        {220, 240, 200, 255}, {245, 230, 150, 255}, {235, 180,  90, 255},
        {220, 120,  60, 255}, {180,  60,  50, 255}, {110,  30,  40, 255},
    };
    for (int i = 0; i < 12; i++) {
        if (palette_set(&pal, i, ramp[i]) < 0)
            return gfault_wrap(GFAULT_PAL_BAD_COLOR);
    }
    int drawn = palette_render_swaths(&pal, fb, 0, 11);
    if (drawn < 0) return drawn;
    return drawn == 12 ? 0 : gfault_wrap(GFAULT_PAL_INDEX_RANGE);
}
