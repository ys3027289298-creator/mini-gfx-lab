#ifndef MINI_GFX_CHAIN_H
#define MINI_GFX_CHAIN_H

#include "canvas.h"

#define CHAIN_BANDS 4
#define CHAIN_BAND_ROWS (CANVAS_H / CHAIN_BANDS)

/* Render all four stages into `frame`.  Returns the number of degraded
   stages; when `failed` is non-NULL it receives a bitmask of failed stages. */
int chain_render(Canvas* frame, unsigned* failed);

#endif
