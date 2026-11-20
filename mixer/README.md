# mixer

多总线混音：每总线增益、侧链闪避、软限幅；共享混响带每总线预延迟与独立干湿比。

```bash
make test
```

## 混响

- `reverb_init(rv, sr, len)`：一套共享混响，`len` 为冲激响应长度（样点，1..1024）。
- `reverb_set_bus(rv, bus, predelay, dry, send)`：每总线设预延迟与干/湿发送量。
  `predelay < 0` 返回 `REVERB_ERR_PREDELAY_NEG`，`predelay > len` 返回
  `REVERB_ERR_PREDELAY_LONG`，非法配置不会写入状态。
- `reverb_process(rv, bus_in, out, frames)`：直达声 `dry` 不经延迟，湿声按
  `predelay` 延后，两路独立相加后再软限幅。
- `reverb_check_alignment(rv, &bad_bus)`：所有总线共用同一绝对帧时钟；任何一条
  总线的实际延迟与设定值相差超过 1 个采样即返回 `REVERB_ERR_ALIGN`，并把
  `bad_bus` 置为第一条出错的总线索引。

自检里还有一组逐点对比：引擎的分块输出与手工拼接的
`x[t - predelay - k] * ir[k]` 参考实现按采样比较，并打印最大误差；
同时故意把一条总线错移 1 个采样，确认该误差会被检出。
