/* lodsimp: vertex-clustering simplification for the offline LOD bake. */
#include "lodsimp.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define SIMP_MAX_CELLS 96

typedef struct { float x, y, z; } Vec3;

static Vec3 v3_sub(Vec3 a, Vec3 b) { return (Vec3){a.x - b.x, a.y - b.y, a.z - b.z}; }
static float v3_dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static float v3_len2(Vec3 a) { return v3_dot(a, a); }

/* squared distance from p to triangle (a,b,c) */
static float point_tri_dist2(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
    Vec3 ab = v3_sub(b, a), ac = v3_sub(c, a), ap = v3_sub(p, a);
    float d1 = v3_dot(ab, ap), d2 = v3_dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return v3_len2(ap);
    Vec3 bp = v3_sub(p, b);
    float d3 = v3_dot(ab, bp), d4 = v3_dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return v3_len2(bp);
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        Vec3 q = {a.x + ab.x * v, a.y + ab.y * v, a.z + ab.z * v};
        return v3_len2(v3_sub(p, q));
    }
    Vec3 cp = v3_sub(p, c);
    float d5 = v3_dot(ab, cp), d6 = v3_dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return v3_len2(cp);
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        Vec3 q = {a.x + ac.x * w, a.y + ac.y * w, a.z + ac.z * w};
        return v3_len2(v3_sub(p, q));
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        Vec3 q = {b.x + (c.x - b.x) * w, b.y + (c.y - b.y) * w, b.z + (c.z - b.z) * w};
        return v3_len2(v3_sub(p, q));
    }
    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom, w = vc * denom;
    Vec3 q = {a.x + ab.x * v + ac.x * w, a.y + ab.y * v + ac.y * w,
              a.z + ab.z * v + ac.z * w};
    return v3_len2(v3_sub(p, q));
}

static Vec3 get_v(const float* v, uint32_t i) {
    return (Vec3){v[3u * i + 0], v[3u * i + 1], v[3u * i + 2]};
}

/* one-sided Hausdorff estimate, both directions: every vertex of A measured
   against the surface of B, and every vertex of B against the surface of A.
   Both directions catch shrink and expansion. */
static float hausdorff(const float* av, uint32_t an, const uint32_t* ai, uint32_t at,
                       const float* bv, uint32_t bn, const uint32_t* bi, uint32_t bt) {
    float worst2 = 0.0f;
    for (uint32_t v = 0; v < an; v++) {
        Vec3 p = get_v(av, v);
        float best = FLT_MAX;
        for (uint32_t t = 0; t < bt; t++) {
            Vec3 a = get_v(bv, bi[3u * t + 0]);
            Vec3 b = get_v(bv, bi[3u * t + 1]);
            Vec3 c = get_v(bv, bi[3u * t + 2]);
            float d2 = point_tri_dist2(p, a, b, c);
            if (d2 < best) best = d2;
        }
        if (best > worst2) worst2 = best;
    }
    for (uint32_t v = 0; v < bn; v++) {
        Vec3 p = get_v(bv, v);
        float best = FLT_MAX;
        for (uint32_t t = 0; t < at; t++) {
            Vec3 a = get_v(av, ai[3u * t + 0]);
            Vec3 b = get_v(av, ai[3u * t + 1]);
            Vec3 c = get_v(av, ai[3u * t + 2]);
            float d2 = point_tri_dist2(p, a, b, c);
            if (d2 < best) best = d2;
        }
        if (best > worst2) worst2 = best;
    }
    return sqrtf(worst2);
}

static LodBBox compute_bounds(const float* v, uint32_t n) {
    LodBBox b;
    lod_bbox_init(&b);
    for (uint32_t i = 0; i < n; i++) lod_bbox_add(&b, v[3u * i], v[3u * i + 1], v[3u * i + 2]);
    return b;
}

static size_t cell_index(const float* p, const LodBBox* sb, float scale, int cells) {
    int cx = (int)((p[0] - sb->min[0]) / scale * (float)cells);
    int cy = (int)((p[1] - sb->min[1]) / scale * (float)cells);
    int cz = (int)((p[2] - sb->min[2]) / scale * (float)cells);
    if (cx < 0) cx = 0;
    if (cx >= cells) cx = cells - 1;
    if (cy < 0) cy = 0;
    if (cy >= cells) cy = cells - 1;
    if (cz < 0) cz = 0;
    if (cz >= cells) cz = cells - 1;
    return ((size_t)cz * (size_t)cells + (size_t)cy) * (size_t)cells + (size_t)cx;
}

/* Cluster at a given grid resolution.  Returns malloc'd vertex/index arrays;
   *out_tris may be 0 when the grid is too coarse to keep any triangle. */
static int cluster(const float* src, uint32_t nv, const uint32_t* idx, uint32_t nt,
                   const LodBBox* sb, int cells,
                   float** ov, uint32_t* onv, uint32_t** oi, uint32_t* ont) {
    size_t cell_count = (size_t)cells * (size_t)cells * (size_t)cells;
    Vec3* sum = (Vec3*)calloc(cell_count, sizeof *sum);
    uint32_t* cnt = (uint32_t*)calloc(cell_count, sizeof *cnt);
    uint32_t* cell_new = (uint32_t*)calloc(cell_count, sizeof *cell_new);
    if (!sum || !cnt || !cell_new) {
        free(sum); free(cnt); free(cell_new);
        return -1;
    }

    Vec3 ext;
    ext.x = sb->max[0] - sb->min[0];
    ext.y = sb->max[1] - sb->min[1];
    ext.z = sb->max[2] - sb->min[2];
    float scale = fmaxf(ext.x, fmaxf(ext.y, ext.z));
    if (!(scale > 0.0f)) scale = 1.0f;

    for (uint32_t v = 0; v < nv; v++) {
        size_t ci = cell_index(src + 3u * v, sb, scale, cells);
        sum[ci].x += src[3u * v + 0];
        sum[ci].y += src[3u * v + 1];
        sum[ci].z += src[3u * v + 2];
        cnt[ci]++;
    }

    /* compact representatives in source-vertex order for determinism */
    uint32_t* remap = (uint32_t*)malloc((size_t)nv * sizeof *remap);
    Vec3* rv = (Vec3*)malloc((size_t)nv * sizeof *rv);
    if (!remap || !rv) {
        free(sum); free(cnt); free(cell_new); free(remap); free(rv);
        return -1;
    }
    uint32_t new_n = 0;
    for (uint32_t v = 0; v < nv; v++) {
        size_t ci = cell_index(src + 3u * v, sb, scale, cells);
        if (cell_new[ci] == 0) {
            Vec3 c;
            c.x = sum[ci].x / (float)cnt[ci];
            c.y = sum[ci].y / (float)cnt[ci];
            c.z = sum[ci].z / (float)cnt[ci];
            /* keep representatives inside the original bounds */
            if (c.x < sb->min[0]) c.x = sb->min[0];
            if (c.x > sb->max[0]) c.x = sb->max[0];
            if (c.y < sb->min[1]) c.y = sb->min[1];
            if (c.y > sb->max[1]) c.y = sb->max[1];
            if (c.z < sb->min[2]) c.z = sb->min[2];
            if (c.z > sb->max[2]) c.z = sb->max[2];
            cell_new[ci] = new_n + 1; /* 1-based so 0 means unset */
            rv[new_n] = c;
            new_n++;
        }
        remap[v] = cell_new[ci] - 1;
    }

    uint32_t* ni = (uint32_t*)malloc((size_t)nt * 3u * sizeof *ni);
    if (!ni) {
        free(sum); free(cnt); free(cell_new); free(remap); free(rv);
        return -1;
    }
    uint32_t out_t = 0;
    for (uint32_t t = 0; t < nt; t++) {
        uint32_t a = remap[idx[3u * t + 0]];
        uint32_t b = remap[idx[3u * t + 1]];
        uint32_t c = remap[idx[3u * t + 2]];
        if (a == b || b == c || a == c) continue; /* collapsed */
        ni[3u * out_t + 0] = a;
        ni[3u * out_t + 1] = b;
        ni[3u * out_t + 2] = c;
        out_t++;
    }

    free(sum); free(cnt); free(cell_new);
    free(remap);

    *ov = (float*)malloc((size_t)new_n * 3u * sizeof(float));
    if (!*ov) { free(rv); free(ni); return -1; }
    for (uint32_t i = 0; i < new_n; i++) {
        (*ov)[3u * i + 0] = rv[i].x;
        (*ov)[3u * i + 1] = rv[i].y;
        (*ov)[3u * i + 2] = rv[i].z;
    }
    free(rv);
    *oi = ni;
    *onv = new_n;
    *ont = out_t;
    if (out_t == 0) {
        free(*ov);
        free(ni);
        *oi = NULL;
        *ov = NULL;
        return 1; /* no usable triangle at this resolution */
    }
    /* drop representatives that no surviving triangle references, and
       renumber, so vertex_count is tight and indices cover [0, count) */
    uint32_t* used = (uint32_t*)calloc(new_n, sizeof *used);
    uint32_t* remap2 = (uint32_t*)malloc((size_t)new_n * sizeof *remap2);
    if (!used || !remap2) {
        free(used); free(remap2); free(*ov); free(ni);
        *oi = NULL;
        *ov = NULL;
        return -1;
    }
    for (uint32_t k = 0; k < out_t * 3u; k++) used[ni[k]] = 1;
    uint32_t tight = 0;
    for (uint32_t i = 0; i < new_n; i++)
        remap2[i] = used[i] ? tight++ : UINT32_MAX;
    for (uint32_t k = 0; k < out_t * 3u; k++) ni[k] = remap2[ni[k]];
    float* tv = (float*)malloc((size_t)tight * 3u * sizeof(float));
    if (!tv) {
        free(used); free(remap2); free(*ov); free(ni);
        *oi = NULL;
        *ov = NULL;
        return -1;
    }
    for (uint32_t i = 0; i < new_n; i++) {
        if (remap2[i] == UINT32_MAX) continue;
        tv[3u * remap2[i] + 0] = (*ov)[3u * i + 0];
        tv[3u * remap2[i] + 1] = (*ov)[3u * i + 1];
        tv[3u * remap2[i] + 2] = (*ov)[3u * i + 2];
    }
    free(used); free(remap2);
    free(*ov);
    *ov = tv;
    *onv = tight;
    return 0;
}

int lod_simplify(const float* src_verts, uint32_t src_vert_count,
                 const uint32_t* src_indices, uint32_t src_tri_count,
                 uint32_t target_tri_count,
                 float** out_verts, uint32_t* out_vert_count,
                 uint32_t** out_indices, uint32_t* out_tri_count,
                 LodSimpStats* stats) {
    if (!src_verts || !src_indices || src_vert_count == 0 || src_tri_count == 0 ||
        !out_verts || !out_indices || !stats)
        return -1;
    if (target_tri_count < 1) target_tri_count = 1;

    LodBBox sb = compute_bounds(src_verts, src_vert_count);
    if (!lod_bbox_is_valid(&sb)) return -1;

    float* best_v = NULL;
    uint32_t* best_i = NULL;
    uint32_t best_nv = 0, best_nt = 0;

    /* try progressively finer grids.  Take the first result that reaches the
       target; if the target is unreachable keep the finest usable result. */
    for (int cells = 2; cells <= SIMP_MAX_CELLS; cells++) {
        float* cv = NULL; uint32_t* ci = NULL; uint32_t cnv = 0, cnt = 0;
        int rc = cluster(src_verts, src_vert_count, src_indices, src_tri_count, &sb,
                         cells, &cv, &cnv, &ci, &cnt);
        if (rc < 0) {
            free(best_v); free(best_i);
            return -1;
        }
        if (rc == 1) continue; /* collapsed away entirely */
        free(best_v); free(best_i);
        best_v = cv; best_i = ci; best_nv = cnv; best_nt = cnt;
        if (cnt >= target_tri_count) break;
    }
    if (!best_v) return -1;

    stats->vertex_count = best_nv;
    stats->tri_count = best_nt;
    LodBBox simp_bounds = compute_bounds(best_v, best_nv);
    stats->bounds = lod_bbox_union(&simp_bounds, &sb);
    stats->geom_error = hausdorff(src_verts, src_vert_count, src_indices, src_tri_count,
                                  best_v, best_nv, best_i, best_nt);

    *out_verts = best_v;
    *out_indices = best_i;
    *out_vert_count = best_nv;
    *out_tri_count = best_nt;
    return 0;
}
