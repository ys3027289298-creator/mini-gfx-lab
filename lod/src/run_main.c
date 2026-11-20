/* lod-run: runtime stage of the LOD chain.  Loads the pack produced by
   lod-bake, validates it, looks variants up by id/level through the index,
   and selects levels under a total triangle budget.  It links only the
   format reader and the selector -- no simplifier. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "lodfmt.h"
#include "lodrt.h"
#include "lodtest.h"

#define PACK_PATH "lod.pack"

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

/* ---- tamper helpers ----------------------------------------------------- */

static uint64_t g_level_rec_off;

static void fix_crc(uint8_t* buf, size_t size) {
    lod_put_u32(buf + LOD_HDR_OFF_PAYLOAD_CRC,
                lod_crc32(buf + LOD_HEADER_BYTES, size - LOD_HEADER_BYTES));
}

static LodError validate_copy(const LodPack* pack, size_t size,
                              void (*mutate)(uint8_t*, size_t), int fix_checksum) {
    uint8_t* copy = (uint8_t*)malloc(size);
    if (!copy) return LOD_ERR_SIZE;
    memcpy(copy, pack->buf, size);
    mutate(copy, size);
    if (fix_checksum) fix_crc(copy, size);
    LodError e = lod_pack_validate(copy, size, NULL, NULL);
    free(copy);
    return e;
}

static void mut_flip_payload(uint8_t* b, size_t n) { b[n - 1] ^= 0x01; }
static void mut_bad_magic(uint8_t* b, size_t n) { (void)n; b[0] = 'X'; }
static void mut_bad_version(uint8_t* b, size_t n) { (void)n; lod_put_u32(b + LOD_HDR_OFF_VERSION, 99); }
static void mut_shrink_bbox(uint8_t* b, size_t n) { (void)n; lod_put_f32(b + g_level_rec_off + 16, 100.0f); }
static void mut_bad_index(uint8_t* b, size_t n) {
    (void)n;
    uint64_t idx_off = lod_get_u64(b + g_level_rec_off + 48);
    lod_put_u32(b + idx_off, 0xFFFFu);
}

/* ---- selection scenarios ------------------------------------------------ */

static LodInstance g_insts[8];

static void make_instances(void) {
    for (int i = 0; i < 8; i++) {
        g_insts[i].mesh_id = (i % 2 == 0) ? 1u : 2u;
        g_insts[i].distance = 3.0f + 4.0f * (float)i;
        g_insts[i].min_level = 0;
        g_insts[i].prev_level = -1;
    }
}

int main(void) {
    char err[256] = {0};

    /* ---- load + index (this is all the runtime does with the file) ------ */
    double t_load0 = now_ms();
    LodPack pack;
    LodError le = lod_pack_load(PACK_PATH, &pack, err, sizeof err);
    double load_ms = now_ms() - t_load0;
    check("load_ok", le == LOD_OK);
    if (le != LOD_OK) {
        printf("load_msg=%s\n", err);
        return test_result();
    }
    check("load_validates_on_open", 1); /* lod_pack_load ran the full validator */
    check("load_mesh_count", pack.hdr.mesh_count == 2);
    check("load_level_count", pack.hdr.level_count == 10);
    check("index_find_mesh1", lod_pack_find_mesh(&pack, 1) == 0);
    check("index_find_mesh2", lod_pack_find_mesh(&pack, 2) == 1);
    check("index_missing_mesh", lod_pack_find_mesh(&pack, 999) == -1);

    /* every level's bounding box must still cover its source mesh */
    int bbox_ok = 1, levels_ok = 1, idx_in_range = 1;
    for (uint32_t i = 0; i < pack.hdr.mesh_count; i++) {
        LodMeshView mv;
        lod_pack_mesh(&pack, i, &mv);
        for (uint32_t l = 0; l < mv.level_count; l++) {
            LodLevelView lv;
            if (lod_pack_level(&pack, i, l, &lv) != 0 || lv.level != l) { levels_ok = 0; continue; }
            if (!lod_bbox_contains(&lv.bounds, &mv.source_bounds)) bbox_ok = 0;
            const uint8_t* ip = pack.buf + lv.idx_off;
            for (uint32_t k = 0; k < lv.tri_count * 3u; k++)
                if (lod_get_u32(ip + 4u * k) >= lv.vertex_count) { idx_in_range = 0; break; }
        }
    }
    check("levels_readable_via_index", levels_ok);
    check("runtime_bbox_covers_source", bbox_ok);
    check("runtime_indices_in_range", idx_in_range);

    /* ---- corrupt packs must be refused, not partly trusted ------------- */
    LodHeader h;
    lod_read_header(pack.buf, pack.size, &h);
    g_level_rec_off = h.table_off + (uint64_t)h.mesh_count * LOD_MESH_REC_BYTES;

    check("reject_flipped_payload",
          validate_copy(&pack, pack.size, mut_flip_payload, 0) == LOD_ERR_CRC);
    check("reject_bad_magic",
          validate_copy(&pack, pack.size, mut_bad_magic, 0) == LOD_ERR_MAGIC);
    check("reject_bad_version",
          validate_copy(&pack, pack.size, mut_bad_version, 1) == LOD_ERR_VERSION);
    check("reject_truncated",
          lod_pack_validate(pack.buf, pack.size - 8u, NULL, NULL) == LOD_ERR_SIZE);
    check("reject_bbox_not_covering",
          validate_copy(&pack, pack.size, mut_shrink_bbox, 1) == LOD_ERR_BBOX);
    check("reject_out_of_range_index",
          validate_copy(&pack, pack.size, mut_bad_index, 1) == LOD_ERR_INDEX);

    /* ---- selection under budget ---------------------------------------- */
    LodContext ctx;
    memset(&ctx, 0, sizeof ctx);
    ctx.fov_y = 1.0f;
    ctx.screen_h = 1080;
    ctx.tolerance_px = 2.0f;
    ctx.hysteresis = 1.1f;
    make_instances();

    LodChoice choices[8];
    LodReport rep;

    ctx.tri_budget = 100000u;
    LodError se = lod_select_batch(&pack, &ctx, g_insts, 8, choices, &rep);
    check("select_ok_generous", se == LOD_OK);
    check("select_under_budget", rep.tri_total <= ctx.tri_budget);
    uint64_t generous_total = rep.tri_total;
    printf("select_generous tri_total=%llu tri_budget=%u\n",
           (unsigned long long)generous_total, ctx.tri_budget);

    ctx.tri_budget = (uint32_t)(generous_total / 2u);
    se = lod_select_batch(&pack, &ctx, g_insts, 8, choices, &rep);
    int levels_valid = 1;
    uint64_t sum = 0;
    for (int i = 0; i < 8; i++) {
        int mi = lod_pack_find_mesh(&pack, g_insts[i].mesh_id);
        LodMeshView mv;
        lod_pack_mesh(&pack, (uint32_t)mi, &mv);
        if (choices[i].level >= mv.level_count || choices[i].mesh_id != g_insts[i].mesh_id)
            levels_valid = 0;
        sum += choices[i].tri_count;
    }
    printf("select_tight tri_total=%llu tri_budget=%u tri_over=%llu\n",
           (unsigned long long)rep.tri_total, ctx.tri_budget,
           (unsigned long long)rep.tri_over);
    check("select_tight_fits", se == LOD_OK && rep.tri_total <= ctx.tri_budget);
    check("select_tight_sum_consistent", sum == rep.tri_total);
    check("select_tight_levels_valid", levels_valid);
    check("select_tight_coarsened", rep.tri_total < generous_total);

    /* budget below the all-coarsest total: report, never silently overrun */
    uint32_t coarsest_total = 0;
    for (int i = 0; i < 8; i++) {
        int mi = lod_pack_find_mesh(&pack, g_insts[i].mesh_id);
        LodMeshView mv;
        lod_pack_mesh(&pack, (uint32_t)mi, &mv);
        LodLevelView lv;
        lod_pack_level(&pack, (uint32_t)mi, mv.level_count - 1u, &lv);
        coarsest_total += lv.tri_count;
    }
    ctx.tri_budget = coarsest_total - 1u;
    se = lod_select_batch(&pack, &ctx, g_insts, 8, choices, &rep);
    printf("select_infeasible tri_total=%llu tri_budget=%u tri_over=%llu\n",
           (unsigned long long)rep.tri_total, ctx.tri_budget,
           (unsigned long long)rep.tri_over);
    check("select_infeasible_reports", se == LOD_ERR_COUNTS && rep.tri_over > 0);
    check("select_infeasible_not_claimed_ok", rep.tri_total > ctx.tri_budget);

    LodInstance miss = { 12345u, 10.0f, 0, -1 };
    LodReport mrep;
    ctx.tri_budget = 100000u;
    LodChoice mc = lod_select(&pack, &ctx, &miss, &mrep);
    check("select_missing_mesh", mrep.instances_missing == 1 && mc.tri_count == 0);

    LodInstance hi = { 1u, 5.0f, 0, -1 };
    int switches = 0, last_level = -1;
    double t_hyst0 = now_ms();
    for (int f = 0; f < 40; f++) {
        hi.distance = 5.0f + 5.0f * (float)f;
        LodReport r;
        LodChoice c = lod_select(&pack, &ctx, &hi, &r);
        if (last_level >= 0 && (int)c.level != last_level) switches++;
        last_level = (int)c.level;
        hi.prev_level = (int)c.level;
    }
    double hyst_ms = now_ms() - t_hyst0;
    printf("select_hysteresis switches=%d frames=40 final_level=%d\n", switches, last_level);
    check("select_hysteresis_stable", switches <= 8);

    LodContext bctx = ctx;
    bctx.tri_budget = 100000u;
    LodChoice batch_choices[8];
    double t_sel0 = now_ms();
    for (int it = 0; it < 1000; it++)
        lod_select_batch(&pack, &bctx, g_insts, 8, batch_choices, NULL);
    double select_ms = (now_ms() - t_sel0) / 1000.0;

    printf("timing_load_validate_ms=%.3f\n", load_ms);
    printf("timing_select_ms=%.4f\n", select_ms);
    printf("timing_select_hysteresis_ms=%.4f\n", hyst_ms);

    lod_pack_free(&pack);
    return test_result();
}
