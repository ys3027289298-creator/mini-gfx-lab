/* lodrt: runtime LOD lookup and budgeted selection.
   The runtime never simplifies anything.  It opens the pack produced by the
   baker, looks meshes up by id, and picks one level per visible instance such
   that the summed triangle count stays within a caller-supplied budget and
   the screen-space error stays as low as possible. */
#ifndef LODRT_H
#define LODRT_H

#include <stdint.h>

#include "lodfmt.h"

#define LODRT_MAX_INSTANCES 256u

typedef struct {
    uint32_t mesh_id;
    float distance;         /* camera to instance bounds, world units */
    uint32_t min_level;     /* clamped to [0, level_count) */
    int prev_level;         /* last level chosen, -1 when there is none */
} LodInstance;

typedef struct {
    uint32_t mesh_id;
    uint32_t level;
    uint32_t tri_count;
    float screen_error;
    int in_budget;
} LodChoice;

typedef struct {
    float fov_y;            /* radians */
    int screen_h;
    float tolerance_px;     /* desired max projected error */
    float hysteresis;       /* tau, >1 avoids level thrashing */
    uint32_t tri_budget;    /* total triangles across all instances */
} LodContext;

typedef struct {
    uint64_t tri_total;
    uint64_t tri_over;
    uint32_t instances_over;
    uint32_t instances_clamped;
    uint32_t instances_missing;
    uint32_t invalid_budget;
} LodReport;

/* Look up mesh_id, return the chosen level.  If the budget leaves room for a
   finer level the choice may be refined to the highest level fitting the
   remaining budget. */
LodChoice lod_select(const LodPack* pack, const LodContext* ctx,
                     const LodInstance* inst, LodReport* rep);

/* Select for all instances.  Returns LOD_OK, or LOD_ERR_COUNTS when the
   budget is below the coarsest possible total (every instance at level 0). */
LodError lod_select_batch(const LodPack* pack, const LodContext* ctx,
                          const LodInstance* insts, uint32_t count,
                          LodChoice* out, LodReport* rep);

#endif
