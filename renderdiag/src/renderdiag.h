/* renderdiag: 渲染链统一诊断。
   粒子/图块/网格/声学四个模块的失败点归一到同一套错误码:
   位 8-15 为模块段, 位 0-7 为模块内码。rd_table 覆盖现有全部失败点,
   renderdiag/check_coverage.sh 会列出未被覆盖的失败点。
   约定: 模块步骤返回 RD_OK(0) 表示成功; 返回非零错误码时, 帧合成器
   只摘掉该模块、本帧继续渲染, 不再整帧丢弃。
   定义 RD_DIAG 后 RD_REPORT 向 stderr 打印映射后的诊断行 (stdout 不变);
   未定义时 RD_REPORT 编译为空语句, 无额外分配, 画面逐像素一致。 */
#ifndef RENDERDIAG_H
#define RENDERDIAG_H

#include <stdio.h>

#define RD_OK 0

/* 模块段 (错误码位 8-15) */
#define RD_SEG_PARTICLE 0x01
#define RD_SEG_TILEMAP  0x02
#define RD_SEG_NAVGRID  0x03
#define RD_SEG_SPATIAL  0x04

#define RD_CODE(seg, id) (((seg) << 8) | (id))

/* particle: 粒子 */
#define RD_PARTICLE_POOL_FULL        RD_CODE(RD_SEG_PARTICLE, 0x01)
#define RD_PARTICLE_EMITTER_FULL     RD_CODE(RD_SEG_PARTICLE, 0x02)
/* tilemap: 图块 */
#define RD_TILEMAP_SET_OUT_OF_RANGE  RD_CODE(RD_SEG_TILEMAP, 0x01)
/* navgrid: 网格 */
#define RD_NAVGRID_LOAD_BOUNDS       RD_CODE(RD_SEG_NAVGRID, 0x01)
#define RD_NAVGRID_LOAD_RAGGED       RD_CODE(RD_SEG_NAVGRID, 0x02)
#define RD_NAVGRID_ENDPOINT_BLOCKED  RD_CODE(RD_SEG_NAVGRID, 0x03)
#define RD_NAVGRID_ALLOC             RD_CODE(RD_SEG_NAVGRID, 0x04)
#define RD_NAVGRID_NO_PATH           RD_CODE(RD_SEG_NAVGRID, 0x05)
#define RD_NAVGRID_PATH_TOO_LONG     RD_CODE(RD_SEG_NAVGRID, 0x06)
#define RD_NAVGRID_SMOOTH_OVERFLOW   RD_CODE(RD_SEG_NAVGRID, 0x07)
/* spatial: 声学 */
#define RD_SPATIAL_BAD_FRAMES        RD_CODE(RD_SEG_SPATIAL, 0x01)
#define RD_SPATIAL_TOO_MANY_SOURCES  RD_CODE(RD_SEG_SPATIAL, 0x02)

typedef struct { int code; const char* name; const char* msg; } RdEntry;

static const RdEntry rd_table[] = {
    { RD_PARTICLE_POOL_FULL,       "particle.pool_full",       "粒子池已满, 本帧该发射器不再产出" },
    { RD_PARTICLE_EMITTER_FULL,    "particle.emitter_full",    "发射器表已满, 新发射器被丢弃" },
    { RD_TILEMAP_SET_OUT_OF_RANGE, "tilemap.set_out_of_range", "越界写瓦片被丢弃" },
    { RD_NAVGRID_LOAD_BOUNDS,      "navgrid.load_bounds",      "网格尺寸超出限制" },
    { RD_NAVGRID_LOAD_RAGGED,      "navgrid.load_ragged",      "网格行宽不一致" },
    { RD_NAVGRID_ENDPOINT_BLOCKED, "navgrid.endpoint_blocked", "起点或终点不可行走" },
    { RD_NAVGRID_ALLOC,            "navgrid.alloc",            "寻路工作区分配失败" },
    { RD_NAVGRID_NO_PATH,          "navgrid.no_path",          "终点不可达" },
    { RD_NAVGRID_PATH_TOO_LONG,    "navgrid.path_too_long",    "路径超出输出容量" },
    { RD_NAVGRID_SMOOTH_OVERFLOW,  "navgrid.smooth_overflow",  "平滑路径超出输出容量" },
    { RD_SPATIAL_BAD_FRAMES,       "spatial.bad_frames",       "帧数超出范围, 本帧声学跳过" },
    { RD_SPATIAL_TOO_MANY_SOURCES, "spatial.too_many_sources", "声源数超出范围, 本帧声学跳过" },
};
#define RD_TABLE_LEN (sizeof(rd_table) / sizeof(rd_table[0]))

/* 模块段号, 用于按模块分段统计 */
static inline int rd_module_of(int code) { return (code >> 8) & 0xff; }

/* 映射: 码 -> "module.code" 名; 表外码才回落 "unknown" */
static inline const char* rd_code_str(int code) {
    for (unsigned i = 0; i < RD_TABLE_LEN; i++)
        if (rd_table[i].code == code) return rd_table[i].name;
    return "unknown";
}

/* 映射: 码 -> 人类可读消息; 表外码才回落 "unknown" */
static inline const char* rd_message(int code) {
    for (unsigned i = 0; i < RD_TABLE_LEN; i++)
        if (rd_table[i].code == code) return rd_table[i].msg;
    return "unknown";
}

#ifdef RD_DIAG
#define RD_REPORT(code) \
    fprintf(stderr, "[renderdiag] %s: %s\n", rd_code_str(code), rd_message(code))
#else
#define RD_REPORT(code) ((void)0)
#endif

#endif
