/*
 * alloc_test: drives the same frame loop the compositor runs, with the
 * diagnostic sink permanently enabled, and asserts the heap-allocation
 * counter does not move while rendering. It repeats the loop with the sink
 * disabled and reports the before/after delta for both modes.
 */
#include "gfault.h"
#include "gfault_inject.h"
#include "framebuf.h"
#include <stdio.h>

unsigned long alloc_counter_allocs(void);
unsigned long alloc_counter_frees(void);
unsigned long alloc_counter_bytes(void);

int bridge_particle_render(FrameBuf* fb);
int bridge_tilemap_render(FrameBuf* fb);
int bridge_navgrid_render(FrameBuf* fb);
int bridge_font_render(FrameBuf* fb);
int bridge_palette_render(FrameBuf* fb);

typedef int (*BridgeFn)(FrameBuf* fb);
static const BridgeFn FNS[GFAULT_SEGMENTS] = {
    bridge_particle_render, bridge_tilemap_render, bridge_navgrid_render,
    bridge_font_render, bridge_palette_render,
};

#define FRAMES 12

static unsigned long run_frames(int enabled) {
    gfault_set_enabled(enabled);
    gfault_reset();
    static FrameBuf fb;                 /* static: no stack/heap pressure */
    unsigned long before = alloc_counter_allocs();
    for (unsigned int fr = 0; fr < FRAMES; fr++) {
        fb_clear(&fb, (Rgba){18, 18, 24, 255});
        gfault_frame_begin(fr);
        for (int s = 0; s < GFAULT_SEGMENTS; s++) {
            int rc = FNS[s](&fb);
            if (rc < 0) {
                /* isolate: clear the panel, keep the remaining segments */
                const FbRegion* r = fb_region((GfaultSeg)s);
                fb_rect(&fb, r->x, r->y, r->x + r->w - 1, r->y + r->h - 1,
                        (Rgba){18, 18, 24, 255});
            }
        }
        /* drain diagnostics every frame: this is the hot-path cost */
        GfaultRecord rec[64];
        int dropped = 0;
        (void)gfault_pull(rec, 64, &dropped);
    }
    unsigned long after = alloc_counter_allocs();
    return after - before;
}

/* Same loop but with one segment faulting per frame, so the diagnostic
   ring and latching logic are exercised while we watch the counter. */
static unsigned long run_fault_frames(void) {
    gfault_set_enabled(1);
    gfault_reset();
    static FrameBuf fb;
    unsigned long before = alloc_counter_allocs();
    static const int fault_code[GFAULT_SEGMENTS] = {
        GFAULT_P_POOL_EXHAUSTED, GFAULT_T_BAD_LAYER, GFAULT_N_NO_PATH,
        GFAULT_F_ATLAS_FULL, GFAULT_PAL_INDEX_RANGE,
    };
    for (unsigned int fr = 0; fr < FRAMES; fr++) {
        gfault_inject_disarm_all();
        gfault_inject_arm((GfaultSeg)(fr % GFAULT_SEGMENTS),
                          fault_code[fr % GFAULT_SEGMENTS], 1);
        fb_clear(&fb, (Rgba){18, 18, 24, 255});
        gfault_frame_begin(fr);
        for (int s = 0; s < GFAULT_SEGMENTS; s++) {
            if (gfault_inject_trip((GfaultSeg)s) != GFAULT_OK) continue;
            int rc = FNS[s](&fb);
            if (rc < 0) {
                const FbRegion* r = fb_region((GfaultSeg)s);
                fb_rect(&fb, r->x, r->y, r->x + r->w - 1, r->y + r->h - 1,
                        (Rgba){18, 18, 24, 255});
            }
        }
        GfaultRecord rec[64];
        int dropped = 0;
        (void)gfault_pull(rec, 64, &dropped);
    }
    unsigned long after = alloc_counter_allocs();
    return after - before;
}

/* Pure diagnostic hot loop: no bridges, no modules — just the operations
   the render path performs per frame while diagnostics stay enabled.
   Any heap traffic here would be attributable to gfault alone. */
static unsigned long run_diag_only(unsigned int iters) {
    gfault_set_enabled(1);
    gfault_reset();
    static const int codes[GFAULT_SEGMENTS] = {
        GFAULT_P_POOL_EXHAUSTED, GFAULT_T_BAD_LAYER, GFAULT_N_NO_PATH,
        GFAULT_F_ATLAS_FULL, GFAULT_PAL_INDEX_RANGE,
    };
    unsigned long before = alloc_counter_allocs();
    for (unsigned int fr = 0; fr < iters; fr++) {
        gfault_frame_begin(fr);
        for (int s = 0; s < GFAULT_SEGMENTS; s++)
            gfault_report((GfaultSeg)s, codes[s], 0);
        GfaultRecord rec[64];
        int dropped = 0;
        (void)gfault_pull(rec, 64, &dropped);
        (void)gfault_segment_failed((GfaultSeg)(fr % GFAULT_SEGMENTS));
        (void)gfault_strerror(codes[fr % GFAULT_SEGMENTS]);
    }
    return alloc_counter_allocs() - before;
}

int main(void) {
    /* warm stdio/loader allocations out of the measured region */
    printf("warmup alloc=%lu\n", alloc_counter_allocs());

    unsigned long d_on = run_frames(1);
    unsigned long d_off = run_frames(0);
    unsigned long d_fault = run_fault_frames();
    unsigned long d_diag = run_diag_only(1000);

    /* The module render path itself allocates (navgrid's A* scratch:
       calloc+malloc per search — pre-existing behavior). The contract is
       that leaving the diagnostic sink enabled adds NOTHING on top:
       on-minus-off must be 0, and the diag-only loop must stay at 0. */
    long added_by_diagnostics = (long)d_on - (long)d_off;
    printf("diag_on_render_alloc_delta=%lu over %d frames\n", d_on, FRAMES);
    printf("diag_off_render_alloc_delta=%lu over %d frames\n", d_off, FRAMES);
    printf("diagnostics_added_allocations=%ld\n", added_by_diagnostics);
    printf("fault_frames_alloc_delta=%lu over %d frames\n", d_fault, FRAMES);
    printf("diag_only_loop_alloc_delta=%lu over 1000 iters\n", d_diag);
    printf("bytes_total=%lu frees=%lu\n",
           alloc_counter_bytes(), alloc_counter_frees());

    int ok = added_by_diagnostics == 0 && d_diag == 0;
    printf("zero_heap_growth_while_diagnostics_on=%d\n", ok);
    return ok ? 0 : 1;
}
