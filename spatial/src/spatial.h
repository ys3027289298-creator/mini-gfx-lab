#ifndef MINI_GFX_SPATIAL_H
#define MINI_GFX_SPATIAL_H

#include "canvas.h"

/* Render this module into rows [y0, y0+rows) of `c`.
   Returns 0 on success or a negative DIAG_ERR(DiagCode) on failure; failures
   are recorded through the shared diagnostics layer. */
int spatial_stage(Canvas* c, int y0, int rows);

#endif
