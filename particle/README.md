# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度 + 帧时间驱动的降载（load shedding）。

```bash
make test
```

## 降载机制

8 个降载等级，每帧观测帧时间后等级至多变化一级：

| level | 发射倍率 | 更新步长（帧） |
|------:|---------:|---------------:|
| 0 | 1.00 | 1 |
| 1 | 0.85 | 1 |
| 2 | 0.70 | 2 |
| 3 | 0.55 | 2 |
| 4 | 0.40 | 4 |
| 5 | 0.28 | 4 |
| 6 | 0.18 | 8 |
| 7 | 0.10 | 8 |

- 帧时间超阈值：每帧升一级（饱和后仍计数），发射预算按倍率收缩，更新改为每 stride 帧补算一次；
- 帧时间回到阈值内：每帧只降一级，倍率单调逐步回升，无突变；
- 跳帧只累加 `dt`、到补算时机一次性积分，线性积分下已有粒子轨迹与逐帧更新完全一致，发射降载只影响新粒子；
- `shed_frames` 与 `over_frames` 同步累加，降载次数严格等于超阈值帧数。

## API

```c
LoadShed s; shed_init(&s, 0.020f);          /* 20ms 阈值 */
int level  = shed_observe(&s, frame_time);  /* 每帧先观测 */
int budget = shed_budget(&s, base_budget);  /* 降载后的发射预算 */
spawn_frame(&world, budget);
shed_update(&world, &s, dt);                /* 内部按 stride 补算 */
shed_flush(&world, &s);                     /* 收尾补齐残留 dt */
```
