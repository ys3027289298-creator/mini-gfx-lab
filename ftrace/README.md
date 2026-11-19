# ftrace

帧时间线采集：记录每帧各阶段耗时，聚合成直方图与百分位，导出 CSV / Chrome Trace。
`make test` 跑自检（含采集开销量测，断言开销 < 2%）。

## 用法

```c
FtTrace t; ft_init(&t);
int s_update = ft_stage(&t, "update");
int s_draw   = ft_stage(&t, "draw");

for (each frame) {
    ft_frame_begin(&t);
    ft_begin(&t, s_update);
    /* ... */
    ft_end(&t, s_update);
    ft_begin(&t, s_draw);
    /* ... */
    ft_end(&t, s_draw);
    ft_frame_end(&t);
}

FtReport rep = ft_report_build(&t);
ft_report_print(&rep, stdout);          /* 控制台表 */
ft_stats_write_csv(&rep, f_stats);      /* 每阶段统计 CSV */
ft_hist_write_csv(&rep, f_hist);        /* 直方图 CSV（41 桶 log2） */
ft_timeline_write_csv(&t, f_tl);        /* 逐帧逐阶段时间线 CSV */
ft_trace_write_json(&t, f_json);        /* Chrome Trace，chrome://tracing / Perfetto 加载 */
```

## 设计约束

- 采集路径无 malloc、无 I/O、无锁；时钟为 `CLOCK_MONOTONIC`（Linux 上走 vdso）。
- 原始样本存 4096 帧定长环形缓冲（最旧帧被覆盖）；直方图永不轮换，覆盖全量观测。
- 百分位用 nearest-rank（1-indexed）对环形缓冲内样本排序计算。
- 聚合只执行一次生成 `FtReport`，控制台与 CSV 渲染同一结构，数值不可能分叉。
- 自检的 `overhead_under_2pct` 在 ~50µs 真实量级帧上，用相邻开/关帧配对交错（4096 对、逐对差值取中位数）量测，实测约 0.15%（每帧采集成本 ~100ns），断言 < 2%。
- `ft_inject_frame` 是测试专用注入口，绕过时钟产生确定性数据。
