# decodecache

自包含 RGBA8 图像编解码（DCI1：RLE + PNG 风格逐行滤波 + FNV-1a 校验）与
**解码缓存**。同一张编码图被多处反复解码时，缓存开启后只解码一次，后续全部命中。

## 契约

- 缓存开启：同一 `image_id` + 同一编码字节（长度与 FNV-1a 源哈希都匹配）命中，
  **命中路径不再调用解码器**；首帧 miss 解码一次，之后 `decodes` 恒为 1。
- 缓存关闭：走旧路径，每次调用都完整解码（`decodes == 调用次数`、`hits == 0`）。
- 两种模式返回的像素**逐字节一致**（`memcmp` 与像素哈希双重校验）。
- 损坏输入（魔数/长度/校验和/RLE/滤波非法）一律报错返回 `NULL`，绝不静默出图。
- 固定容量 intern 表（`DC_CACHE_CAP`），超出后 FIFO 淘汰；被淘汰项再次访问会
  重新解码而不是返回陈旧数据。

## 构建与自检

```sh
make test      # 模块自检：往返、零再解码、关闭一致性、损坏拒绝、淘汰
```

## 四样证据（缺一不算完成）

`make evidence`（仓库根目录）收集并硬校验：

| # | 证据 | 文件 |
|---|---|---|
| 1 | 解码耗时表（每帧 miss/hit/decode 的 us） | `evidence/decode_time.txt` |
| 2 | 缓存命中日志（cache_miss/cache_hit/cache_off，含 decodes 计数） | `evidence/hit_log.txt` |
| 3 | 像素哈希（输入与 cache-on/off 输出，FNV-1a） | `evidence/pixel_hash.txt` |
| 4 | 复现命令 | `evidence/repro.txt` |

已接入 `tilemap`、`fontatlas`、`particle`、`lod` 四处图像管线，各自
`make -C <module> bench` 出该模块数据；根目录 `make bench` 汇总，缺任一处即失败。
