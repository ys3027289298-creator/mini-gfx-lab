/* renderdiag 自测: 映射表完整性 + 失败降级帧合成器。 */
#include <stdio.h>
#include <string.h>
#include "renderdiag.h"

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* 帧合成器: 逐模块执行渲染步骤; 某模块返回错误码时上报并摘掉该模块,
   本帧用其余模块继续渲染, 不再整帧丢弃。 */
typedef int (*RdStep)(void* ctx);
typedef struct { const char* name; RdStep step; } RdModule;

static int rd_run_frame(const RdModule* mods, int n, unsigned* done_mask) {
    int degraded = 0;
    *done_mask = 0;
    for (int i = 0; i < n; i++) {
        int rc = mods[i].step(0);
        if (rc == RD_OK) { *done_mask |= 1u << i; continue; }
        RD_REPORT(rc);   /* 降级: 帧继续, 只少这个模块 */
        degraded++;
    }
    return degraded;
}

static int step_ok(void* ctx) { (void)ctx; return RD_OK; }
static int step_spatial_fail(void* ctx) { (void)ctx; return RD_SPATIAL_BAD_FRAMES; }

int main(void) {
    /* 1. 每个已定义错误码都映射到非 unknown 的名字与消息 */
    static const int all_codes[] = {
        RD_PARTICLE_POOL_FULL, RD_PARTICLE_EMITTER_FULL,
        RD_TILEMAP_SET_OUT_OF_RANGE,
        RD_NAVGRID_LOAD_BOUNDS, RD_NAVGRID_LOAD_RAGGED,
        RD_NAVGRID_ENDPOINT_BLOCKED, RD_NAVGRID_ALLOC, RD_NAVGRID_NO_PATH,
        RD_NAVGRID_PATH_TOO_LONG, RD_NAVGRID_SMOOTH_OVERFLOW,
        RD_SPATIAL_BAD_FRAMES, RD_SPATIAL_TOO_MANY_SOURCES,
    };
    int all_mapped = 1;
    for (unsigned i = 0; i < sizeof(all_codes) / sizeof(all_codes[0]); i++)
        if (!strcmp(rd_code_str(all_codes[i]), "unknown") ||
            !strcmp(rd_message(all_codes[i]), "unknown")) all_mapped = 0;
    check("all_codes_mapped", all_mapped);

    /* 2. 表内码与名均唯一 */
    int uniq = 1;
    for (unsigned i = 0; i < RD_TABLE_LEN; i++)
        for (unsigned j = i + 1; j < RD_TABLE_LEN; j++)
            if (rd_table[i].code == rd_table[j].code ||
                !strcmp(rd_table[i].name, rd_table[j].name)) uniq = 0;
    check("table_unique", uniq);

    /* 3. 模块分段: 码的段号与名字前缀一致 */
    static const char* seg_names[] = { "", "particle", "tilemap", "navgrid", "spatial" };
    int seg_ok = 1;
    for (unsigned i = 0; i < RD_TABLE_LEN; i++) {
        int seg = rd_module_of(rd_table[i].code);
        if (seg < 1 || seg > 4 ||
            strncmp(rd_table[i].name, seg_names[seg], strlen(seg_names[seg])) != 0)
            seg_ok = 0;
    }
    check("module_segments", seg_ok);

    /* 4. 只有表外码才回落 unknown (旧行为只留给真正未映射的码) */
    check("unknown_fallback", !strcmp(rd_code_str(0x7f7f), "unknown"));

    /* 5. 降级: 声学失败时帧不丢, 只摘掉声学模块 */
    RdModule frame[] = {
        { "particle", step_ok }, { "tilemap", step_ok },
        { "navgrid", step_ok }, { "spatial", step_spatial_fail },
    };
    unsigned done = 0;
    int degraded = rd_run_frame(frame, 4, &done);
    check("frame_survives", degraded == 1);   /* 整帧没丢 */
    check("module_dropped", done == 0x7u);    /* 只摘了位 3 的声学 */
    check("frame_not_black", done != 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
