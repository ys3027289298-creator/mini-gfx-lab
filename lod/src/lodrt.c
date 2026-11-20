/* lodrt: runtime LOD lookup, screen-error selection with hysteresis, and a
   greedy triangle-budget allocator. */
#include "lodrt.h"

#include <math.h>
#include <string.h>

static float screen_error(float geom_error, float distance, float fov_y, int screen_h) {
    if (distance <= 0.0f) return 1e9f;
    return geom_error * (float)screen_h /
           (2.0f * distance * tanf(fov_y * 0.5f));
}

static int clamp_level(int level, uint32_t level_count) {
    if (level < 0) return 0;
    if ((uint32_t)level >= level_count) return (int)level_count - 1;
    return level;
}

static int level_tris(const LodPack* pack, int mesh_index, int level, float* err,
                      LodBBox* bounds) {
    LodLevelView lv;
    if (lod_pack_level(pack, (uint32_t)mesh_index, (uint32_t)level, &lv) != 0) return -1;
    if (err) *err = lv.geom_error;
    if (bounds) *bounds = lv.bounds;
    return (int)lv.tri_count;
}

/* Coarsest level whose projected error is within tolerance, on level 0..N-1
   with error strictly increasing.  Hysteresis requires a switch to beat the
   threshold by tau in whichever direction it moves. */
static int desired_level(const LodPack* pack, int mesh_index, const LodContext* ctx,
                         float distance, int prev_level) {
    LodMeshView mv;
    if (lod_pack_mesh(pack, (uint32_t)mesh_index, &mv) != 0) return 0;
    int want = 0;
    for (int l = (int)mv.level_count - 1; l >= 0; l--) {
        float e;
        if (level_tris(pack, mesh_index, l, &e, NULL) < 0) return 0;
        if (screen_error(e, distance, ctx->fov_y, ctx->screen_h) <= ctx->tolerance_px) {
            want = l;
            break;
        }
    }
    if (prev_level < 0 || prev_level == want) return want;
    int prev = clamp_level(prev_level, mv.level_count);
    float e_new, e_old;
    if (level_tris(pack, mesh_index, want, &e_new, NULL) < 0) return want;
    if (level_tris(pack, mesh_index, prev, &e_old, NULL) < 0) return prev;
    float s_new = screen_error(e_new, distance, ctx->fov_y, ctx->screen_h);
    float s_old = screen_error(e_old, distance, ctx->fov_y, ctx->screen_h);
    if (want > prev && s_new > ctx->tolerance_px * ctx->hysteresis) return prev;
    if (want < prev && s_old < ctx->tolerance_px / ctx->hysteresis) return prev;
    return want;
}LodError lod_select_batch(const LodPack* pack, const LodContext* ctx,
                          const LodInstance* insts, uint32_t count,
                          LodChoice* out, LodReport* rep) {
    LodReport local;
    memset(&local, 0, sizeof local);
    if (rep) *rep = local;
    if (!pack || !pack->buf || !ctx || count > LODRT_MAX_INSTANCES) return LOD_ERR_COUNTS;
    if (count > 0 && (!insts || !out)) return LOD_ERR_COUNTS;
    if (count == 0) return LOD_OK;
    if (!(ctx->screen_h > 0) || !(ctx->fov_y > 0.0f) ||
        !(ctx->tolerance_px > 0.0f) || !(ctx->hysteresis >= 1.0f)) {
        local.invalid_budget = 1;
        if (rep) *rep = local;
        return LOD_ERR_COUNTS;
    }

    struct { int mesh_index; int level; int tris; float err; } cur[LODRT_MAX_INSTANCES];
    uint64_t total = 0;

    for (uint32_t i = 0; i < count; i++) {
        int mi = lod_pack_find_mesh(pack, insts[i].mesh_id);
        if (mi < 0) {
            local.instances_missing++;
            out[i].mesh_id = insts[i].mesh_id;
            out[i].level = 0;
            out[i].tri_count = 0;
            out[i].screen_error = 0.0f;
            out[i].in_budget = 0;
            cur[i].mesh_index = -1;
            cur[i].level = 0; cur[i].tris = 0; cur[i].err = 0.0f;
            continue;
        }
        LodMeshView mv;
        lod_pack_mesh(pack, (uint32_t)mi, &mv);
        float d = insts[i].distance;
        int want = desired_level(pack, mi, ctx, d, insts[i].prev_level);
        int lo = clamp_level((int)insts[i].min_level, mv.level_count);
        if (want < lo) { want = lo; local.instances_clamped++; }
        float e;
        int tris = level_tris(pack, mi, want, &e, NULL);
        if (tris < 0) return LOD_ERR_LEVEL;
        cur[i].mesh_index = mi;
        cur[i].level = want;
        cur[i].tris = tris;
        cur[i].err = screen_error(e, d, ctx->fov_y, ctx->screen_h);
        total += (uint64_t)tris;
    }

    /* Over budget: coarsen with the smallest error increase per triangle
       freed until the total fits.  Level N-1 is always the cheapest. */
    uint32_t guard = 0;
    while (total > ctx->tri_budget && guard < count * LOD_MAX_LEVELS_PER_MESH) {
        guard++;
        int best = -1;
        double best_ratio = 0.0;
        int best_saved = 0;
        float best_err = 0.0f;
        for (uint32_t i = 0; i < count; i++) {
            if (cur[i].mesh_index < 0) continue;
            LodMeshView mv;
            lod_pack_mesh(pack, (uint32_t)cur[i].mesh_index, &mv);
            int next = cur[i].level + 1;
            if ((uint32_t)next >= mv.level_count) continue;
            float e_next;
            int tris_next = level_tris(pack, cur[i].mesh_index, next, &e_next, NULL);
            if (tris_next < 0) return LOD_ERR_LEVEL;
            int saved = cur[i].tris - tris_next;
            if (saved <= 0) continue;
            float s_next = screen_error(e_next, insts[i].distance, ctx->fov_y, ctx->screen_h);
            double ratio = (s_next - cur[i].err) / (double)saved;
            if (best < 0 || ratio < best_ratio) {
                best = (int)i;
                best_ratio = ratio;
                best_saved = saved;
                best_err = s_next;
            }
        }
        if (best < 0) break; /* everything already at the coarsest level */
        cur[best].level++;
        cur[best].tris -= best_saved;
        cur[best].err = best_err;
        total -= (uint64_t)best_saved;
    }

    for (uint32_t i = 0; i < count; i++) {
        out[i].mesh_id = insts[i].mesh_id;
        out[i].level = (uint32_t)cur[i].level;
        out[i].tri_count = (uint32_t)cur[i].tris;
        out[i].screen_error = cur[i].err;
        out[i].in_budget = cur[i].mesh_index >= 0;
    }
    local.tri_total = total;
    if (total > ctx->tri_budget) {
        local.tri_over = total - ctx->tri_budget;
        local.instances_over = count;
        if (rep) *rep = local;
        return LOD_ERR_COUNTS;
    }
    if (rep) *rep = local;
    return LOD_OK;
}

LodChoice lod_select(const LodPack* pack, const LodContext* ctx,
                     const LodInstance* inst, LodReport* rep) {
    LodChoice c;
    memset(&c, 0, sizeof c);
    LodReport local;
    lod_select_batch(pack, ctx, inst, 1, &c, &local);
    if (rep) *rep = local;
    return c;
}
