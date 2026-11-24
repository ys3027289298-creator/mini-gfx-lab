
/* Bridge: tilemap segment -> framebuffer panel. See bridge_particle.c for
   the MODULE_SRC mechanism and old/new source compatibility rules. */
#define main tilemap_demo_main
#include MODULE_SRC
#undef main
#include "framebuf.h"
#include "gfault.h"

static Rgba bridge_t_layer_color(int layer) {
    switch (layer) {
    case 0:  return (Rgba){210, 160,  70, 255};
    case 1:  return (Rgba){ 80, 180, 220, 255};
    default: return (Rgba){190,  90, 200, 255};
    }
}

/* Builds the same map as the module demo and draws filled cells; edge
   cells get a dark outline derived from the 4-neighbour autotile mask. */
int bridge_tilemap_render(FrameBuf* fb) {
    (void)&tilemap_demo_main;
    const FbRegion* reg = fb_region(GFAULT_SEG_TILEMAP);
    static TileMap m;
    map_init(&m);
    for (int x = 5; x < 10; x++) map_set(&m, 0, x, 5, 1);
    map_set(&m, 1, 5, 5, 2);

    const int tile = 12;
    const Rgba edge = {30, 30, 30, 255};
    for (int l = 0; l < LAYERS; l++) {
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                if (!map_get(&m, l, x, y)) continue;
                int bx = reg->x + x * tile;
                int by = reg->y + y * tile;
                if (bx >= reg->x + reg->w || by >= reg->y + reg->h) continue;
                Rgba col = bridge_t_layer_color(l);
                fb_rect(fb, bx, by, bx + tile - 1, by + tile - 1, col);
                if (autotile_mask(&m, l, x, y) & 1) fb_hline(fb, bx, bx + tile - 1, by, edge);
                if (autotile_mask(&m, l, x, y) & 4) fb_hline(fb, bx, bx + tile - 1, by + tile - 1, edge);
                if (autotile_mask(&m, l, x, y) & 2) fb_vline(fb, bx + tile - 1, by, by + tile - 1, edge);
                if (autotile_mask(&m, l, x, y) & 8) fb_vline(fb, bx, by, by + tile - 1, edge);
            }
        }
    }
    return 0;
}
