# mixer

总线混音、侧链闪避、软限幅，以及四环节离线导出链。

## 自检

`make test` 跑原有混音自检（6 项，行为未改动）。

## 导出链与采样率统一（pipeline.c）

四个环节及 legacy 采样率：

| 环节 | 作用 | legacy 采样率 |
|---|---|---|
| stage 1 | 三源合成（sfx/music/voice） | 8000 Hz |
| stage 2 | 重采样到混音输入 | 44100 Hz |
| stage 3 | 总线混音 + 侧链闪避 | 22050 Hz |
| stage 4 | 重采样 + 软限幅 + 16-bit WAV 导出 | 48000 Hz |

**咔哒成因**：重采样以 2 秒块为单位、每块读位置归零（最近邻），非整数
采样率比在块边界产生样本级跳变，于是每 2 秒一次咔哒。

**统一开关**（两条等价路径）：

- 编译期：`pipeline_unify`（`-DMIXER_UNIFY_RATE=1`），四环全部 48000 Hz，
  两个重采样器退化为 1:1 直拷，块边界不再有相位重置；
- 运行时：`./pipeline <dir> --unify`，与编译期产物逐字节一致。

开关关闭时（默认 `pipeline`）走的是与原来完全相同的代码路径与速率表，
输出与特性引入前的基线逐字节相同（由 verify 脚本的 sha256 基线守护）。

`make analysis` 一次产出四份验证材料到 `../artifacts/mixer/`：

1. `click_report.txt` / `click_counts.json` — 咔哒计数，逐环 + 导出 WAV；
   硬指标：legacy 每 2 秒边界各 1 次（共 3 次），unified 为 0。
2. `waveform_summary.json` + `waveform_{legacy,unified}.png` — 导出波形
   数值摘要（peak/RMS/DC/crest/最大步长）与边界放大图。
3. `spectrogram_{legacy,unified}.png` + `spectrogram_compare.png` —
   逐环 STFT 频谱图（标注 2/4/6 秒块边界）与边界频谱对比。
4. `verify_report.txt` / `verify_report.json` — 逐环采样率/帧数/时长核对、
   运行时与编译期统一开关一致性、legacy 输出与基线的字节级一致性。

分析脚本位于 `../artifacts/scripts/`，均以 `python3 -I` 隔离模式运行。
基线需要重建时：`python3 -I ../artifacts/scripts/verify_stages.py --rebuild-baseline`。
