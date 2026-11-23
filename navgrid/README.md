# mini-navgrid

8 方向 A*、对角禁止切角、视线可达平滑、增量重规划实验平台。

```bash
make test        # 验收套件（失败点注入 + 零分配断言）
make acceptance  # 套件 + 失败点覆盖 + 成功路径 golden 逐字节比对
```

失败点：`grid_load_empty` `grid_load_ragged` `no_path` `astar_alloc_failed`
`smooth_overflow` `output_overflow`。夹具在 `tests/faults/`。
