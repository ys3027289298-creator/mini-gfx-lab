# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度 + 池满时最旧优先淘汰。

- **分配 O(1)**：单链表维护空闲槽位，`pool_alloc` 直接取表头。
- **回收 O(1)**：侵入式双向年龄链表，按出生序排列，表头即存活最久的粒子；
  淘汰时摘表头、释放时按索引摘链，都不做扫描。
- **淘汰计数**：`evicted_total`，另有 `spawned_total`、`expired_total`、
  `refused_total`、`evicted_age_sum` 用于观测淘汰粒子的平均存活时长。
- **策略**：`POOL_REJECT_ON_FULL`（旧行为，满即丢）与 `POOL_EVICT_OLDEST`
  （默认推荐）可切换，便于对比。

```bash
make test
```

`make test` 会打印：

- `priority_quota_ok` 等调度 / 淘汰正确性断言
- 突发负载对比：旧池 vs 最旧优先淘汰池 vs 无限池的 `drop_frame_rate`
  及 `drop_frame_rate_reduction`（要求 ≥ 80%）
- 容量 10000、全满时的单帧 `update` 耗时（要求 < 2 ms）
