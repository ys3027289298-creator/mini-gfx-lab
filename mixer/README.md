# mixer

三总线（sfx / music / voice）混音 + 软限幅 + 人声对音乐的侧链压缩。48 kHz，交错立体声。

```bash
make test
```

## 侧链压缩

人声总线（`KEY_BUS = 2`）作为键控信号，压缩音乐总线（默认 `target_bus = 1`）。
SFX 和人声自身只受各自推子控制，不受压缩影响。

**可配参数**（`sidechain_config`）：

| 参数 | 含义 |
|---|---|
| `threshold_db` | 检测器电平（dBFS，1.0 = 满刻度）开始产生增益衰减的门限 |
| `ratio` | 压缩比 ≥ 1，例如 4 表示 4:1 |
| `attack_ms` / `release_ms` | 包络时间常数，`0 ms` 表示瞬时 |

**检测与增益**：

- 检测器取人声左右声道的绝对值峰值，经单极点峰值包络（分别使用 attack / release 系数）平滑。
- 增益折线为硬拐点静态曲线：`GR = (level_dB − threshold) · (1 − 1/ratio)`，低于门限为 0。
  折线在 dB 域是精确的，因此与参考压缩器的静态曲线偏差为 0（`static_curve_worst_err_db`）；
  用 1 kHz 正弦测得的增益偏离折线 < 0.3 dB（内层峰值包络的纹波，仍在 1 dB 内）。
- 增益 `g = 10^(−GR/20)` 直接乘到音乐总线。

**零延迟**：同一个采样索引的语音样本驱动该索引音乐样本的增益，
没有前瞻（lookahead）、没有延迟线。`zero_latency_ok` 用一个位于第 64 帧的语音脉冲验证
音乐增益第一次变化正好发生在第 64 个采样。

**增益衰减可观测**：

- `mixer_gain_reduction_db(&mx)` —— 最近一个处理采样的衰减量（正 dB）。
- `mixer_gain_reduction_max_db(&mx)` —— 最近一个处理块内的最大衰减量，适合驱动电平表
  （块末瞬时值可能在 release 期间已经回落，峰值更能反映块内最深压缩）。
- 该读数与实际输出增益一致（`gr_meter_ok`）。

**时间行为单调**：攻击越长，同一时间窗内达到的衰减越少（`attack_monotonic`：1 ms→13.5 dB，
20 ms→6.3 dB）；释放越长，静音 20 ms 后仍保留的衰减越多（`release_monotonic`：20 ms→7.0 dB，
500 ms→13.2 dB）。

## 输出（`make test`）

```
sum_ok=1
no_clip_when_low=1
sidechain_lowers_music=1
sidechain_gr_visible=1
sfx_voice_unaffected=1
sidechain_idle_passthrough=1
static_curve_worst_err_db=0.0000
static_curve_ok=1
sine_gr_meas_db=13.2715 ref_db=13.4846
static_curve_sine_ok=1
attack_gr_1ms=13.482 20ms=6.256
attack_monotonic=1
release_gr_20ms=6.970 500ms=13.224
release_monotonic=1
gr_measured_db=13.4845 reported_db=13.4845 max_db=13.4845
gr_meter_ok=1
first_changed_sample=64 (voice onset 64) music_at_onset=0.04235
zero_latency_ok=1
soft_clip_bounds=1
bad_bus_ignored=1
all_ok=1
```
