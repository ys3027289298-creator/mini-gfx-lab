# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度，以及环形缓冲分帧重放。

```bash
make test   # 默认场景 + 重放检查
```

- `./particle-pool` — 关闭重放的原始场景，输出与加入重放功能前逐字节一致
  （`spawn_frame` / `update` 原路径未改动）。
- `./particle-pool replay` — 分帧重放检查。

## 分帧重放

- 每帧只记录输入，不记录状态：`FrameInput` 含帧号、`dt`、发射预算、各发射器
  配额、外部风力（外部输入）。
- 环形缓冲固定容量；写满后覆盖最旧记录并累加 `overwrites` 计数。覆盖前先把
  该记录作用到 `base` 世界快照，使缓冲内任何帧仍可从已知状态逐帧重算。
- `history_rollback(h, out, target)` 按帧号回退：从 `base` 快照重放到 `target`，
  回退后的逐粒子状态（整个 `World` 内存比较）与不中断运行完全相同；回退点及
  之后的记录被丢弃，可换输入重新推进。
- 错误显式返回，绝不静默忽略：
  - 容量 `<= 0` → `REPLAY_ERR_CAPACITY`
  - 帧号跳跃（不等于期望的下一帧，含倒退）→ `REPLAY_ERR_FRAME_JUMP`，
    且被拒记录不会推进世界
  - 回退目标超出保留窗口（含负帧号、未来帧号）→ `REPLAY_ERR_FRAME_MISSING`
- 零风力时重放路径的物理计算与 `update()` 逐位一致。
