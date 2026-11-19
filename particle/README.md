# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度 + 预热（prewarm）。

预热：进入场景前按给定时长（`world_prewarm(w, duration, dt, budget)`）先把
模拟跑一遍再显示。随机源内嵌于 World，预热与正式运行共用同一份 RNG 状态，
序列连续；预热期间抑制 on_spawn/on_death 外部回调；预热 N 步的结果与连续
运行 N 帧逐字节一致；耗时为步数 × 单帧成本（测试验证 ≤ 100× 单帧）。

```bash
make test
```
