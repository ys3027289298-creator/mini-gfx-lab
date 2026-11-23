#ifndef MINI_GFX_FAILPOINTS_H
#define MINI_GFX_FAILPOINTS_H

#include "diag.h"

#define FAILPOINT_LIST(X) \
    X(particle, emitter_capacity,  DIAG_E_CAPACITY,    "emitter table full") \
    X(particle, duplicate_emitter, DIAG_E_INVALID_ARG, "duplicate emitter id") \
    X(particle, bad_budget,        DIAG_E_INVALID_ARG, "negative spawn budget") \
    X(particle, pool_full,         DIAG_E_CAPACITY,    "particle pool exhausted during spawn") \
    X(particle, invalid_priority,  DIAG_E_INVALID_ARG, "emitter priority is negative") \
    X(particle, output_overflow,   DIAG_E_CAPACITY,    "particle output buffer overflow") \
    X(tilemap, map_set_out_of_range, DIAG_E_OUT_OF_RANGE, "map_set coordinates outside grid") \
    X(tilemap, autotile_bad_layer,   DIAG_E_INVALID_ARG,  "autotile layer outside range") \
    X(tilemap, cull_invalid_rect,    DIAG_E_INVALID_ARG,  "cull rectangle is inverted") \
    X(tilemap, selftest_failed,      DIAG_E_INVARIANT,    "tilemap self-check failed") \
    X(tilemap, output_overflow,      DIAG_E_CAPACITY,     "tilemap output buffer overflow") \
    X(navgrid, grid_load_empty,    DIAG_E_INVALID_ARG, "grid has no cells") \
    X(navgrid, grid_load_ragged,   DIAG_E_INVALID_ARG, "grid rows have unequal width") \
    X(navgrid, no_path,            DIAG_E_NO_PATH,     "no path between start and goal") \
    X(navgrid, astar_alloc_failed, DIAG_E_NOMEM,       "A* scratch allocation failed") \
    X(navgrid, smooth_overflow,    DIAG_E_CAPACITY,    "smoothed path exceeds output capacity") \
    X(navgrid, output_overflow,    DIAG_E_CAPACITY,    "navgrid output buffer overflow") \
    X(spatial, bad_frame_count,   DIAG_E_INVALID_ARG, "frame count outside valid range") \
    X(spatial, too_many_sources,  DIAG_E_CAPACITY,    "source count outside valid range") \
    X(spatial, nonfinite_gain,    DIAG_E_INVALID_ARG, "source gain or position is not finite") \
    X(spatial, nonfinite_sample,  DIAG_E_INVALID_ARG, "input sample is not finite") \
    X(spatial, clip_violation,    DIAG_E_INVARIANT,   "mix output exceeds soft-clip bound") \
    X(spatial, output_overflow,   DIAG_E_CAPACITY,    "spatial output buffer overflow")

typedef struct {
    const char* module;
    const char* point;
    DiagCode    code;
    const char* message;
} FailPointDef;

const FailPointDef* failpoint_lookup(const char* module, const char* point);
int  failpoint_count(void);
const FailPointDef* failpoint_at(int index);
/* Print "module:point:CODE" for one module (module == NULL prints all). */
int  failpoint_print(const char* module);

#endif /* MINI_GFX_FAILPOINTS_H */
