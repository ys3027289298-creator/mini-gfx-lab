# footstep

脚步声事件调度 + 44.1kHz → 48kHz 线性重采样的对齐修复。

## 问题

脚步事件（`t = k/7 s`，落点含小数采样相位）进入混音队列后按 480 帧
（10ms）块边界派发，重采样相位从块起点归零起步：实际发声位置比事件时间
晚 68~480 个采样（1.4~10ms），听起来"慢半拍"，且重采样相位与事件时间
对不齐。

## 修复

`align=1`：事件对齐到精确输出采样位置（`floor(pos)` + 小数相位），
重采样器携带亚采样偏移，落点偏差 ≤ 1 个采样（实测最大 0.8571）。
`align=0` 路径保持原样，输出与修复前逐字节相同。

## 复现步骤

```sh
cd footstep
make test      # 自检：对齐后偏差 ≤1 采样；align=0 输出匹配黄金哈希（逐字节同现状）
make report    # 重新生成交付物到 out/
```

预期输出：

```
max_dev_off=480.0000 max_dev_on=0.8571 (samples)
aligned_max_dev_le_1_sample=1
unaligned_actually_late=1
off_mode_byte_identical=1
fix_changes_output=1
resample_len_exact=1
onsets_found=1
peak_sane=1
all_ok=1
```

## 交付物（`make report` 生成于 `out/`）

| 文件 | 内容 |
|---|---|
| `waveform_summary.txt` | 波形摘要：帧数、峰值、RMS、两种模式的 FNV-1a64 指纹 |
| `event_deviation_table.csv` | 事件偏差表：每个事件的精确落点、两种模式的实测起点与偏差（采样） |
| `resample_log.txt` | 重采样记录：采样率比、每事件起始相位、写出帧数、源消耗位置 |
| `mix_align_off.f32` / `mix_align_on.f32` | 原始波形（float32 小端 48kHz 单声道） |
| `sha256.txt` | 两个波形文件的 SHA-256 |

## 逐字节一致性

`align=0` 输出指纹 FNV-1a64 = `ae95f32f8610dacf`，作为 `GOLDEN_OFF_HASH`
固化在 `src/footstep.c` 自检中；任何改动若让关闭对齐的输出偏离现状，
`make test` 的 `off_mode_byte_identical` 即失败。
