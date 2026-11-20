# mixer

三总线（sfx / music / voice）混音、侧链闪避、软限幅，以及曲线驱动的多总线自动化。
所有缓冲区都是交错立体声。`make test` 跑自检。

## 曲线自动化

一条曲线可以带多个目标，每个目标把同一条曲线的归一化值映射到一条总线：

```c
Automation a;
auto_init(&a);
auto_add_target(&a, 0, AUTO_UNIT_LINEAR, AUTO_SHAPE_COS, 0.0f, 1.0f);  /* sfx  */
auto_add_target(&a, 1, AUTO_UNIT_LINEAR, AUTO_SHAPE_SIN, 0.0f, 1.0f);  /* music */

AutoCurve c;
curve_init(&c, 2.0f, 0);
curve_add(&c, 0.0f, 0.0f, AUTO_LINEAR);
curve_add(&c, 2.0f, 1.0f, AUTO_LINEAR);
auto_set_curve(&a, &c, 0.0f);

/* 每个音频块： */
auto_tick(&a, (float)FRAMES / 48000.0f);
auto_apply(&a, &mx);          /* 等价于把每条总线推子手动设到映射值 */
mix(&mx, srcs, n, in, out, FRAMES);
```

- **多目标与映射**：每个目标有独立的总线、单位（线性增益或 dBFS）、传递曲线
  （线性 / sqrt / square / cos / sin）以及 `lo..hi` 端点。所有目标在同一时刻
  用同一个曲线值求值，因此联动天生同步。每条总线只允许一个目标
  （`auto_add_target` 拒绝重复总线）。
- **按时间求值**：`auto_tick(dt)` 推进时间并求值一次；关键帧支持线性、smooth、
  ease-in/out、step 缓动，支持循环与首尾夹取（`duration`/`loop` 同 `tween`）。
- **切换曲线要过渡**：`auto_set_curve(curve, transition_s)` 先快照当前已生效的
  增益，再在 `transition_s` 内用 smoothstep 从快照过渡到新曲线的映射值。
  过渡中再次切换会从混合后的增益继续快照，所以输出不会跳变。过渡结束时增益
  精确落在映射值上（不是差 1 ulp 的近似）。
- **与手动设定一致**：过渡结束后（或 `transition_s = 0`）驱动增益等于逐参数
  手动设置的映射值；`manual_equiv_worst` 逐块比较自动化输出与手动推子输出，
  差值为 0。
- **总能量波动 < 1 dB**：等功率交叉淡化（sfx 用 cos、music 用 sin，两路等功率
  且不相关）扫过整条曲线时，48 kHz 下按 4096 帧窗口测得总能量起伏
  `spread=0.097 dB`（要求 < 1 dB）。
- **越界夹取并记录**：曲线关键帧的值允许越界，求值后先夹取到 `[0,1]` 再映射；
  每次发生夹取都会计数（`clamp_count`）并写入容量为 8 的事件环
  （`auto_last_clamp()` 取最近一条，含时间、原始值、夹取后的值、受影响目标数）。
  `auto_reset_clamp_log()` 清空计数与事件环。

## 自检输出

```
sum_ok=1 ... bad_bus_ignored=1
curve_add_ok=1 ... curve_reject_range=1
map_linear_start=1 ... map_equal_power=1
equal_power_worst=0.000000060
target_add=1 ... clamp_reset=1
switch_start_gain=1 ... switch_mid_end=1
manual_equiv_worst=0.000000000
manual_equiv_ok=1
energy_windows=11 min_db=-14.028 max_db=-13.931 mean_db=-13.976 spread=0.097
energy_windows_enough=1
energy_flat=1
energy_absolute=1
all_ok=1
```
