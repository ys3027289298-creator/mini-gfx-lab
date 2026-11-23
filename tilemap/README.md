# tilemap

分层瓦片地图 + 4 邻接自动拼接 + 区域剔除。

```bash
make test        # 验收套件（失败点注入 + 零分配断言）
make acceptance  # 套件 + 失败点覆盖 + 成功路径 golden 逐字节比对
```

失败点：`map_set_out_of_range` `autotile_bad_layer` `cull_invalid_rect`
`selftest_failed` `output_overflow`。夹具在 `tests/faults/`。
