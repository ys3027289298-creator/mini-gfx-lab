
/* chain: renders all four sub-projects into one frame.  A failing stage only
   loses its own band; the rest of the frame is still drawn.  Diagnostics are
   recorded through the shared diag layer and only printed when enabled. */
#include <stdio.h>
#include <string.h>

#include "diag.h"
#include "canvas.h"
#include "failpoints.h"
#include "particle.h"
#include "tilemap.h"
#include "navgrid.h"
#include "spatial.h"

#define BANDS 4
#define BAND_ROWS (CANVAS_H / BANDS)

typedef int (*StageFn)(Canvas*, int, int);

typedef struct {
    const char* module;
    StageFn     fn;
} Stage;

static const Stage k_stages[BANDS] = {
    { "particle", particle_stage },
    { "tilemap",  tilemap_stage  },
    { "navgrid",  navgrid_stage  },
    { "spatial",  spatial_stage  },
};

/* Renders one full frame.  Returns the number of degraded stages; `failed`
   (optional) receives a bitmask, bit i = stage i failed. */
int chain_render(Canvas* frame, unsigned* failed) {
    canvas_init(frame);
    int degraded = 0;
    unsigned mask = 0;
    for (int i = 0; i < BANDS; i++) {
        int y0 = i * BAND_ROWS;
        Canvas band;
        canvas_init(&band);
        int rc = k_stages[i].fn(&band, 0, BAND_ROWS);
        if (rc == DIAG_OK) {
            canvas_blit_band(frame, &band, y0, BAND_ROWS);
        } else {
            /* Only this module's drawing is dropped. */
            degraded++;
            mask |= 1u << i;
            char note[CANVAS_W + 1];
            snprintf(note, sizeof note, "!! %s degraded (%s) - band skipped",
                     k_stages[i].module, diag_code_name(DIAG_CODE_OF(rc)));
            canvas_clear_band(frame, y0, BAND_ROWS);
            canvas_text(frame, 1, y0 + BAND_ROWS / 2, note);
            if (diag_is_enabled()) {
                const char* m = NULL, *p = NULL;
                DiagCode code = DIAG_OK;
                if (diag_last_failure(&m, &p, &code))
                    diag_emitf(m, p, code, "error", "chain: stage %s failed rc=%d",
                               k_stages[i].module, rc);
                else
                    diag_emitf(k_stages[i].module, "output_overflow", DIAG_CODE_OF(rc),
                               "error", "chain: stage %s failed rc=%d",
                               k_stages[i].module, rc);
            }
        }
    }
    if (failed) *failed = mask;
    return degraded;
}

#ifndef CHAIN_NO_MAIN
int main(void) {
    diag_boot();
    Canvas frame;
    unsigned failed = 0;
    int degraded = chain_render(&frame, &failed);
    canvas_dump(&frame, stdout);
    diag_flush(stderr);
    printf("degraded=%d\n", degraded);
    return degraded > 0 ? 1 : 0;
}
#endif
