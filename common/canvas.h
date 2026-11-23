/* canvas: small fixed-size character framebuffer used by the render chain.
   Static storage only; no allocation. */
#ifndef MINI_GFX_CANVAS_H
#define MINI_GFX_CANVAS_H

#include <stddef.h>

#define CANVAS_W 72
#define CANVAS_H 32

typedef struct {
    int w, h;
    char px[CANVAS_H][CANVAS_W + 1];
} Canvas;

void canvas_init(Canvas* c);
void canvas_clear(Canvas* c);
void canvas_fill(Canvas* c, char ch);
void canvas_put(Canvas* c, int x, int y, char ch);
void canvas_text(Canvas* c, int x, int y, const char* s);
void canvas_clear_band(Canvas* c, int y0, int rows);
/* Copy rows [0, rows) of `src` into `dst` starting at row y0.  Used by stages
   to commit a band only after the whole module succeeded. */
void canvas_blit_band(Canvas* dst, const Canvas* src, int y0, int rows);
/* Print the canvas inside a border, followed by a newline. */
void canvas_dump(const Canvas* c, void* file);

#endif /* MINI_GFX_CANVAS_H */
