# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度；以及粒子拖尾用的环形位置历史。

| 二进制 | 源 | 内容 |
|---|---|---|
| `particle-pool` | `src/particle.c` | 固定容量粒子池、每帧配额、发射器优先级 |
| `trail-history` | `src/trail.c` | 每粒子环形位置历史（拖尾） |

## trail 环形历史

- 每粒子固定长度位置环（2 的幂，上限 `TRAIL_HIST_MAX=16`），按帧写入，掩码回绕覆盖最旧槽
- 全程零 `malloc`：扁平静态数组，容量与历史长度在 `trail_init` 一次性确定
- 渲染读取按年龄 `max_age` / 段数 `max_segments` 取最近若干段：
  - `trail_spans()`：零拷贝，最多返回两段物理连续区间（回绕必然 ≤2 段）
  - `trail_read()`：拷贝进调用方缓冲，输出逻辑连续（最旧 → 最新），最多两次 `memcpy`
- 粒子死亡/槽位复用调用 `trail_clear()` 复位环
- 基准（10000 粒子 × 16 段，单帧 1.92 MB）：拷贝中位约 150 μs，硬校验 < 1 ms

```bash
make test
```
