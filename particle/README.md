# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度。

```bash
make test        # 验收套件（失败点注入 + 零分配断言）
make acceptance  # 套件 + 失败点覆盖 + 成功路径 golden 逐字节比对
```

失败点（`common/failpoints.h`）：`emitter_capacity` `duplicate_emitter`
`bad_budget` `pool_full` `priority_inversion` `output_overflow`。
每个失败点在 `tests/faults/` 有夹具，`DIAG_FAULT=particle:<point>` 可注入。
