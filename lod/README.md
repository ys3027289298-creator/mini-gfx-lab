# lod

屏幕误差 LOD 选择 + 迟滞切换，并记录切换历史与误差累积。

```bash
make test
```

## 输出

```
levels=275,270,270,185 switches=30
hist_total=30 hist_kept=16 hist_cap=16
history_ok=1 direction_ok=1 error_sign_ok=1
window_ms=500 window_cap=64 max_samples=64 span_ms=315.0
window_consistency_ok=1 window_bound_ok=1
alert_px=20 alerts=35 first_alert_t=355 alert_ok=1
trace_bytes=2240 history_bytes=640
memory_ok=1 cap_bound=1 levels_ok=1
```

## 切换历史

每次 `lod_select` 变级都会写入一条 `LodSwitch`：

| 字段 | 含义 |
|---|---|
| `frame` / `t_ms` | 帧号与时间戳 |
| `from` / `to` | 切换前后的级别 |
| `dir` | `+1` 变细、`-1` 变粗 |
| `levels` | 跨越的级数（幅度） |
| `error_from` / `error_to` | 切换前后的屏幕投影误差（px） |

历史为固定容量环形缓冲（`HISTORY_CAP`，默认 16），只保留最近的事件；
`hist_total` 记录累计切换次数，不受容量影响。

## 误差累积与告警

每帧记录所选级别的投影误差到环形窗口（`WINDOW_CAP`，默认 64），
`trace_window` 按时间窗（`window_ms`，默认 500ms）聚合样本数、切换次数、
均值与峰值误差；窗口均值超过 `alert_px` 时告警，两次告警间隔不短于
`alert_cooldown_ms`。

## 内存上界

两条环形缓冲都是静态定长：`LodTrace` 仅 2240 字节，其中切换历史 640 字节，
与运行帧数无关。高帧率下样本数先被 `WINDOW_CAP` 截断，时间窗只做进一步过滤。

## 一致性校验

测试内独立地从"逐帧原始记录"重算窗口统计，与环形缓冲聚合结果逐帧比对
（`window_consistency_ok`）；该重算采用与环形缓冲相同的两条限制（最多
`WINDOW_CAP` 帧、不早于时间窗）。同时校验保留历史恰为最近 `HISTORY_CAP`
条事件（`history_ok`）、方向与幅度自洽且净变化等于当前级别（`direction_ok`）、
变细误差必降、变粗误差必升（`error_sign_ok`），以及阈值不可达时零告警。
