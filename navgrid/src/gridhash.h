#ifndef GRIDHASH_H
#define GRIDHASH_H

/* 格子图内容哈希：用于联机对局双方校验地图一致性。
 *
 * 设计要点：
 * - 稳定：哈希只由 (w, h, 每格坐标与取值) 决定，不读取结构体原始字节，
 *   因此与内存布局（padding、行 stride）和主机字节序无关。
 * - 增量：每格独立计算 64 位 token，全图以模 2^64 加法折叠。
 *   加法满足交换律/结合律，单格改动只需 O(1)：acc += token(new) - token(old)。
 * - 可序列化：摘要为 64 位整数，规范形式是 16 字符小写十六进制文本。
 */
#include <stdint.h>
#include <stddef.h>

/* 单格 token：仅依赖坐标与取值，纯移位/乘法混合，跨平台逐位一致。 */
uint64_t gh_cell_token(uint32_t x, uint32_t y, uint32_t v);

/* 全量哈希。cells 为行主序，行 stride 可大于 w（兼容带 padding 的布局）。 */
uint64_t gh_hash_full_strided(uint32_t w, uint32_t h,
                              const unsigned char* cells, size_t stride);
uint64_t gh_hash_full(uint32_t w, uint32_t h, const unsigned char* cells);

/* 增量状态：初始化后随单格改动 O(1) 维护，digest 恒等于全量重算。 */
typedef struct { uint64_t acc; uint32_t w, h; } GridHash;

void gh_init(GridHash* gh, uint32_t w, uint32_t h, const unsigned char* cells);
void gh_on_cell_change(GridHash* gh, uint32_t x, uint32_t y,
                       unsigned char old_v, unsigned char new_v);
uint64_t gh_digest(const GridHash* gh);

/* 序列化：16 字符小写十六进制（数值的规范大端序文本，与主机字节序无关）。 */
void gh_to_hex(uint64_t digest, char out[17]);
int gh_from_hex(const char hex[16], uint64_t* out); /* 0 ok, -1 格式错误 */

#endif
