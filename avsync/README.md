# avsync

`make test` 跑自检（用 `test/` 下预生成的事件时刻表，不依赖音频仓库）。

把每个游戏事件的画面动作呈现到帧网格上，输出**帧时刻**（`frames.csv`）与
**逐帧偏差**（`deviation.csv`）。帧 k 的上屏时刻为 (k+0.5)/fps；音频事件时刻表
由 mini-audio-lab 仓库的 `avsync/` 模块生成，本模块按 schema 校验后逐帧比对。

## 复现步骤

```sh
# 音频侧（mini-audio-lab）生成事件时刻表
cd mini-audio-lab/avsync && make
./avsync                                # 对齐模式，写 events.csv
./avsync --align=off --out=events_legacy.csv   # 关掉对齐，复现当前行为

# 画面侧（本目录）
cd ../../mini-gfx-lab/avsync && make
./avsync --events=../../mini-audio-lab/avsync/events.csv
./avsync --align=off --events=../../mini-audio-lab/avsync/events_legacy.csv
```

偏差超过一帧时退出码为 1；`--align` 与 CSV 头记录不一致、fps 不一致、
schema 不是 `avsync-events/1` 都会报错退出（退出码 2）。

## 输出

- `frames.csv`：帧时刻表，列 `frame,present_s`（present_s = (frame+0.5)/fps）
- `deviation.csv`：逐帧偏差，列 `frame,event,dev_s,dev_frames`
- stdout：帧数/事件数、dev_min/mean/max_ms、`dev_maxabs_frames`、
  `within_1_frame=1` 表示偏差不超过一帧

## 兼容清单

- CSV schema：`avsync-events/1`（首行校验，其他版本直接报错）
- 时刻表来源：mini-audio-lab `avsync/`，采样率 44100 / 48000 Hz，缓冲 128 / 256 / 512 帧
- 帧率：30 / 60 / 120 fps（`--fps` 必须与时刻表头一致）
- `--align` 必须与音频侧一致（时刻表头有记录，不匹配即报错）
- 偏差预算：≤ 1 帧；`--align=off` 消费 legacy 时刻表时输出与当前行为一致
  （音频相对上屏时刻早至多半帧）
- 自检固件：`test/events_aligned.csv`、`test/events_legacy.csv` 由音频侧
  默认参数（48000/256/60，120 事件）生成
