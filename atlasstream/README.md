# atlasstream

图集分块流式加载 + 多级顶替：细 mip 层按区域分块，缺块时用上一级
（粗块 → 常驻基础金字塔）顶替，块解码完成后在帧边界原子替换，无闪烁。

- 块状态机 MISSING→QUEUED→LOADING→DECODED→READY，只有 READY 可被采样，
  解码中的块永不可见；替换只发生在 commit，不出现一帧闪烁或细节回退。
- 收敛后与一次性全量加载逐像素一致（自检逐字节 memcmp）。
- LRU 缓存上限约束常驻块数；自检报告内存峰值与块数关系：
  `peak = base_pyramid + resident_chunks * chunk_bytes`。

```bash
make test
```
