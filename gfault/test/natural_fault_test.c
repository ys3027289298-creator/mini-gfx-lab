/*
 * natural_fault_test: drives the modules' REAL failure paths (no fault
 * injection) and asserts every failure lands in the module's own gfault
 * block and is visible through the diagnostic sink.
 */
#include "gfault.h"
#include "framebuf.h"
#include "palette.h"
#include <stdio.h>

#define main particle_demo_main
#define failures particle_failures
#define check particle_check
#include MODULE_PARTICLE
#undef check
#undef failures
#undef main

#define main tilemap_demo_main
#define failures tilemap_failures
#define check tilemap_check
#include MODULE_TILEMAP
#undef check
#undef failures
#undef main

#define main navgrid_demo_main
#include MODULE_NAVGRID
#undef main

#define main fontatlas_demo_main
#define failures font_failures
#define check font_check
#include MODULE_FONT
#undef check
#undef failures
#undef main

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int saw_code(int code) {
    GfaultRecord r[64];
    int n = gfault_pull(r, 64, 0);
    for (int i = 0; i < n; i++)
        if (r[i].code == code) return 1;
    return 0;
}

int main(void) {
    gfault_reset();

    /* particle: fill the emitter table past MAX_EMITTERS */
    World w; world_init(&w);
    int last = 0;
    for (int i = 0; i < MAX_EMITTERS + 3; i++) last = emitter_add(&w, 100 + i, 1, 0);
    check("particle_emitter_full_rc", last < 0 &&
          gfault_unwrap(last) == GFAULT_P_EMITTER_FULL);
    check("particle_block_0x10", saw_code(GFAULT_P_EMITTER_FULL));

    /* particle: exhaust the pool by spawning beyond capacity */
    gfault_reset();
    World w2; world_init(&w2);
    emitter_add(&w2, 1, MAX_PARTICLES + 10, 0);
    spawn_frame(&w2, MAX_PARTICLES + 10);
    check("particle_pool_exhausted_0x11", saw_code(GFAULT_P_POOL_EXHAUSTED));

    /* tilemap: bad layer and out-of-range coordinate (write-side faults;
       map_get's out-of-range sentinel is a benign autotile query) */
    gfault_reset();
    TileMap m; map_init(&m);
    int trc = map_set(&m, 9, 0, 0, 1);
    check("tilemap_bad_layer_rc", trc < 0 &&
          gfault_unwrap(trc) == GFAULT_T_BAD_LAYER);
    int trc2 = map_set(&m, 0, -5, 0, 1);
    check("tilemap_block_0x20", trc2 < 0 &&
          gfault_unwrap(trc2) == GFAULT_T_OUT_OF_RANGE);

    /* navgrid: unwalkable endpoint */
    gfault_reset();
    static const char* rows[] = { "##", "##" };
    Grid g;
    grid_load(&g, rows, 2);
    Pt raw[4], s = {0,0}, t = {1,1};
    int nrc = astar(&g, s, t, raw, 4);
    check("navgrid_unwalkable_rc", nrc < 0 &&
          gfault_unwrap(nrc) == GFAULT_N_UNWALKABLE);
    check("navgrid_block_0x30", saw_code(GFAULT_N_UNWALKABLE));

    /* fontatlas: rejected glyph and overfilled atlas */
    gfault_reset();
    Atlas a; atlas_init(&a);
    int frc = atlas_add(&a, (GlyphSize){0, 10}, 0);
    check("font_reject_rc", frc < 0 &&
          gfault_unwrap(frc) == GFAULT_F_GLYPH_REJECTED);
    Atlas full; atlas_init(&full);
    while (atlas_add(&full, (GlyphSize){64, 64}, 0) == 0) {}
    check("font_block_0x40", saw_code(GFAULT_F_ATLAS_FULL));

    /* palette: index range and LUT full */
    gfault_reset();
    Palette pal; palette_init(&pal);
    Rgba c;
    int prc = palette_get(&pal, 999, &c);
    check("palette_range_rc", prc < 0 &&
          gfault_unwrap(prc) == GFAULT_PAL_INDEX_RANGE);
    for (int i = 0; i < PALETTE_SIZE; i++)
        palette_set(&pal, i, (Rgba){1,2,3,255});
    prc = palette_add(&pal, (Rgba){4,5,6,255});
    check("palette_block_0x50", prc < 0 &&
          gfault_unwrap(prc) == GFAULT_PAL_LUT_FULL);

    /* every recorded code must resolve to a non-"unknown" message */
    gfault_reset();
    (void)palette_get(&pal, -1, &c);
    GfaultRecord rec[64];
    int n = gfault_pull(rec, 64, 0);
    int named = 0;
    for (int i = 0; i < n; i++) {
        const char* msg = gfault_strerror(rec[i].code);
        if (msg[0] != 'u' || msg[1] != 'n') named++;   /* not "unknown" */
    }
    check("no_unknown_strings", named == n && n > 0);

    /* ring overflow: more reports than the 64-slot ring must drop the
       oldest records, keep running, and count what was dropped. */
    gfault_reset();
    for (int i = 0; i < 200; i++)
        gfault_report(GFAULT_SEG_PARTICLE, GFAULT_P_BAD_ARG, 0);
    GfaultRecord storm[64];
    int dropped = 0;
    int got = gfault_pull(storm, 64, &dropped);
    check("ring_overflow_kept_cap", got == 64);
    check("ring_overflow_dropped_counted", dropped == 200 - 64 &&
          gfault_total_dropped() == 200 - 64);
    check("ring_overflow_latest_kept", storm[63].code == GFAULT_P_BAD_ARG);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
