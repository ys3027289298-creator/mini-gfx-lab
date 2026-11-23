# mini-spatial-audio

双耳定位：方位角→ITD/IID、反距离衰减、多声源软限幅混音。

```bash
make test        # 验收套件（失败点注入 + 零分配断言）
make acceptance  # 套件 + 失败点覆盖 + 成功路径 golden 逐字节比对
```

失败点：`bad_frame_count` `too_many_sources` `nonfinite_gain`
`nonfinite_sample` `clip_violation` `output_overflow`。夹具在 `tests/faults/`。
