/*
 * Bridge: particle segment -> framebuffer panel.
 *
 * The bridge #includes the module source directly so its internal static
 * API is reachable. MODULE_SRC is set on the compiler command line, which
 * lets the identical bridge compile against either the pre-change source
 * (git worktree at the base commit) or the hardened source. Only symbols
 * present in both versions are used.
 */
#define main particle_demo_main
#include MODULE_SRC
#undef main
#include "framebuf.h"
#include "gfault.h"

static Rgba bridge_p_tag_color(int tag) {
    switch (tag) {
    case 1:  return (Rgba){230,  70,  70, 255};
    case 2:  return (Rgba){ 70, 200,  90, 255};
    default: return (Rgba){ 90, 140, 240, 255};
    }
}

/* Runs the same fixed simulation as the module demo and plots alive
   particles into the particle panel. Returns 0 or a negative gfault code. */
int bridge_particle_render(FrameBuf* fb) {
    (void)&particle_demo_main;   /* keep the renamed demo entry referenced */
    const FbRegion* reg = fb_region(GFAULT_SEG_PARTICLE);
    static World w;
    world_init(&w);
    if (emitter_add(&w, 1, 40, 2) < 0) return gfault_wrap(GFAULT_P_BAD_EMITTER);
    if (emitter_add(&w, 2, 30, 9) < 0) return gfault_wrap(GFAULT_P_BAD_EMITTER);
    if (emitter_add(&w, 3, 20, 5) < 0) return gfault_wrap(GFAULT_P_BAD_EMITTER);
    for (int frame = 0; frame < 10; frame++) {
        spawn_frame(&w, 50);
        update(&w, 0.1f);
    }

    const int scale = 48;
    int plotted = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle* p = &w.pool[i];
        if (!p->alive) continue;
        int px = reg->x + 8 + (int)(p->x * scale);
        int py = reg->y + 8 + (int)(p->y * scale);
        Rgba col = bridge_p_tag_color(p->tag);
        fb_pixel(fb, px,     py,     col);
        fb_pixel(fb, px + 1, py,     col);
        fb_pixel(fb, px,     py + 1, col);
        fb_pixel(fb, px + 1, py + 1, col);
        plotted++;
    }
    return plotted == w.alive_count ? 0 : gfault_wrap(GFAULT_P_POOL_EXHAUSTED);
}
