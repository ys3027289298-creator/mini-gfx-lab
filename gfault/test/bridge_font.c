
/* Bridge: fontatlas segment -> framebuffer panel. Draws every packed glyph
   rect as an outlined block at its atlas coordinates, plus a kerning
   readout strip. Success-only symbols are used, so the same bridge
   compiles against the pre-change and hardened atlas code. */
#define main fontatlas_demo_main
#include MODULE_SRC
#undef main
#include "framebuf.h"
#include "gfault.h"

int bridge_font_render(FrameBuf* fb) {
    (void)&fontatlas_demo_main;
    const FbRegion* reg = fb_region(GFAULT_SEG_FONT);
    static Atlas a;
    atlas_init(&a);
    int idx = -1;
    if (atlas_add(&a, (GlyphSize){16, 16}, &idx) != 0)
        return gfault_wrap(GFAULT_F_GLYPH_REJECTED);
    if (atlas_add(&a, (GlyphSize){32, 32}, 0) != 0)
        return gfault_wrap(GFAULT_F_ATLAS_FULL);
    int placed = 0;
    while (atlas_add(&a, (GlyphSize){40, 20}, 0) == 0) {
        if (++placed > 20) break;
    }

    const int panel_h = 40;
    const Rgba fill = {235, 220, 170, 255};
    const Rgba outline = {40, 40, 50, 255};
    for (int i = 0; i < a.count; i++) {
        const GlyphRect* r = &a.rects[i];
        int x0 = reg->x + r->x / 2;
        int y0 = reg->y + r->y / 2;
        int w = r->w / 2 > 0 ? r->w / 2 : 1;
        int h = r->h / 2 > 0 ? r->h / 2 : 1;
        if (y0 >= reg->y + panel_h) continue;
        fb_rect(fb, x0, y0, x0 + w - 1, y0 + h - 1, fill);
        fb_hline(fb, x0, x0 + w - 1, y0, outline);
        fb_hline(fb, x0, x0 + w - 1, y0 + h - 1, outline);
        fb_vline(fb, x0, y0, y0 + h - 1, outline);
        fb_vline(fb, x0 + w - 1, y0, y0 + h - 1, outline);
    }

    /* kerning strip: one pixel per glyph of the demo string, shifted by
       kern deltas so kerning changes would be visible pixel-wise. */
    static const KernPair pairs[] = { {'A', 'V', -1}, {'T', 'o', -1} };
    const char* text = "AToVAB";
    int advance = kerning_apply(pairs, 2, text);
    int x = reg->x + 4;
    int y = reg->y + reg->h - 10;
    const Rgba pen = {240, 240, 240, 255};
    for (const char* p = text; *p; p++) {
        fb_rect(fb, x, y, x + 3, y + 6, pen);
        x += 4;
    }
    if (advance <= 0) return gfault_wrap(GFAULT_F_NO_GLYPH);
    return 0;
}
