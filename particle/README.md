# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度，以及按帧批量投递的碰撞事件。

```bash
make test
```

## 碰撞事件

- 订阅：`world_subscribe(w, cb, ctx)` 返回订阅 id；`world_unsubscribe(w, id)` 注销。回调签名 `int cb(const ParticleEvent*, int count, void* ctx)`，`count == 0` 的帧不会回调。
- 事件内容：`particle_id`（发射时分配的稳定 id，不复用）、`slot`（当帧池索引）、`x/y`（撞击位置）、`collider_id`。
- 计数一致：每个粒子对每个碰撞体只报告一次首次接触（`hit` 锁存），事件总数 = 碰撞次数，不会逐帧重复。
- 批量零分配：`update()` 先把本帧碰撞收进 `World.events`，再一次性把同一个只读数组投递给所有订阅者；不拷贝、不 malloc，`events_total` 累计总数。
- 回调约束：投递期间 `particle_spawn`/`spawn_frame`/`update` 一律拒绝并计入 `dispatch_rejections`；订阅增删只对下一帧生效。
- 异常隔离：回调返回非 0 记为失败（`callback_errors`），其余订阅者与后续帧照常执行。若宿主以 longjmp / C++ 异常跳出回调，捕获后须调用 `world_end_dispatch(w)`，否则下一次 `update` 会被当作重入拒绝。
- 碰撞体：`collider_add_box` / `collider_add_circle`，点测试，快速粒子可能穿透薄碰撞体。
