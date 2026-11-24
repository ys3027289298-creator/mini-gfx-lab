
/* Bridge: navgrid segment -> framebuffer panel. Old/new source compatible:
   the bridge only checks success/failure of each call, never the specific
   negative code. */
#define main navgrid_demo_main
#include MODULE_SRC
#undef main
#include "framebuf.h"
#include "gfault.h"

/* Renders the demo grid: blocked cells, the raw A* path and the smoothed
   path with distinct colors. Returns 0 or a negative gfault code. */
int bridge_navgrid_render(FrameBuf* fb) {
    (void)&navgrid_demo_main;
    const FbRegion* reg = fb_region(GFAULT_SEG_NAVGRID);
    static const char* rows[] = {
        "S....#....",
        ".##..#....",
        ".#......#.",
        ".#.####.#.",
        ".....#..G.",
    };
    static Grid g;
    if (grid_load(&g, rows, 5) != 0) return gfault_wrap(GFAULT_N_BAD_GRID);

    static Pt raw[MAX_NODES], sm[MAX_NODES];
    Pt s = {0, 0}, t = {8, 4};
    int n = astar(&g, s, t, raw, MAX_NODES);
    if (n < 0) return gfault_wrap(GFAULT_N_NO_PATH);
    int m = smooth(&g, raw, n, sm, MAX_NODES);
    if (m < 0) return gfault_wrap(GFAULT_N_SMOOTH_CAP);

    const int cell = 14;
    const Rgba block = {70, 70, 80, 255};
    const Rgba raw_col = {230, 180, 70, 255};
    const Rgba smooth_col = {90, 220, 130, 255};

    for (int y = 0; y < g.h; y++)
        for (int x = 0; x < g.w; x++)
            if (g.blocked[y][x])
                fb_rect(fb, reg->x + x * cell, reg->y + y * cell,
                        reg->x + x * cell + cell - 2,
                        reg->y + y * cell + cell - 2, block);

    for (int i = 0; i < n; i++)
        fb_pixel(fb, reg->x + raw[i].x * cell + cell / 2,
                     reg->y + raw[i].y * cell + cell / 2, raw_col);
    for (int i = 0; i < m; i++) {
        int cx = reg->x + sm[i].x * cell + cell / 2;
        int cy = reg->y + sm[i].y * cell + cell / 2;
        fb_rect(fb, cx - 2, cy - 2, cx + 2, cy + 2, smooth_col);
    }
    return 0;
}
