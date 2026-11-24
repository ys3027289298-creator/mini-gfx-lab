
/* palette selftest: LUT contracts plus the gfault 0x50 failure codes. */
#include "palette.h"
#include <stdio.h>

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Palette pal;
    palette_init(&pal);

    int idx = palette_set(&pal, 5, (Rgba){10, 20, 30, 255});
    check("set_ok", idx == 5);
    Rgba c = {0};
    check("get_ok", palette_get(&pal, 5, &c) == 5 &&
                    c.r == 10 && c.g == 20 && c.b == 30 && c.a == 255);

    int a1 = palette_add(&pal, (Rgba){1, 2, 3, 255});
    int a2 = palette_add(&pal, (Rgba){4, 5, 6, 255});
    check("add_uses_free_slots", a1 == 0 && a2 == 1);
    check("count_tracked", pal.count == 3);

    /* failure paths: every one must land in the 0x50 palette block */
    int rc = palette_get(&pal, 300, &c);
    check("index_range_code", rc < 0 && gfault_unwrap(rc) == GFAULT_PAL_INDEX_RANGE);
    rc = palette_get(&pal, 7, &c);                 /* unused slot */
    check("unused_slot_code", rc < 0 && gfault_unwrap(rc) == GFAULT_PAL_INDEX_RANGE);
    rc = palette_set(&pal, -4, (Rgba){0,0,0,0});
    check("set_range_code", rc < 0 && gfault_unwrap(rc) == GFAULT_PAL_INDEX_RANGE);
    rc = palette_get(0, 0, &c);
    check("null_arg_code", rc < 0 && gfault_unwrap(rc) == GFAULT_PAL_BAD_ARG);

    /* LUT-full path: fill every remaining slot, then one more add */
    for (int i = 0; i < PALETTE_SIZE; i++)
        if (!pal.used[i]) palette_set(&pal, i, (Rgba){255,255,255,255});
    rc = palette_add(&pal, (Rgba){9,9,9,255});
    check("lut_full_code", rc < 0 && gfault_unwrap(rc) == GFAULT_PAL_LUT_FULL);

    /* rendering: eight registered swatches must draw without failure */
    FrameBuf fb;
    fb_clear(&fb, (Rgba){0, 0, 0, 255});
    int drawn = palette_render_swaths(&pal, &fb, 0, 7);
    check("swaths_drawn", drawn == 8);
    check("swath_pixels_written",
          fb.px[fb_region(GFAULT_SEG_PALETTE)->y + 5][fb_region(GFAULT_SEG_PALETTE)->x + 5].r ==
          pal.lut[0].r);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
