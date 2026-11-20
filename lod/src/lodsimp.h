/* lodsimp: offline mesh simplification used by the baker.
   Vertex clustering on a uniform grid: every vertex is snapped to a cell
   centre, duplicates are welded, and triangles that collapse are dropped.
   The grid is re-scaled per level until the triangle count reaches the
   requested target, and the geometric error is the measured one-sided
   Hausdorff distance from the simplified surface back to the original. */
#ifndef LODSIMP_H
#define LODSIMP_H

#include <stdint.h>

#include "lodfmt.h"

typedef struct {
    uint32_t vertex_count;
    uint32_t tri_count;
    float geom_error;
    LodBBox bounds;          /* bounds of the simplified mesh, unioned with source */
} LodSimpStats;

/* Build a simplified mesh from src.  The returned arrays are malloc'd and
   owned by the caller.  On success *out_verts has 3*vertex_count floats and
   *out_indices has 3*tri_count uint32. */
int lod_simplify(const float* src_verts, uint32_t src_vert_count,
                 const uint32_t* src_indices, uint32_t src_tri_count,
                 uint32_t target_tri_count,
                 float** out_verts, uint32_t* out_vert_count,
                 uint32_t** out_indices, uint32_t* out_tri_count,
                 LodSimpStats* stats);

#endif
