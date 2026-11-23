# colorpipe

四环节渲染链（覆盖合成 → 光照/增益 → 雾混合 → 饱和度）的色彩管线。

## 问题

旧链路里每个环节各自持有传递曲线：拿到 8 位缓冲后用自己的曲线解码、做完混合
再用自己的曲线编码并存回 8 位。四组互不相同的解码/编码对叠加四次重量化，整张
色卡整体偏亮偏蓝：平均明度差 `ΔL* = +4.08`，24 个色块全部超出量化上界，最大
CIE76 偏差 `9.48`。

## 统一色彩空间

改成只在入口解码 sRGB 一次、四个环节都在同一份线性光浮点缓冲上运算、出口
编码一次。每像素传递函数调用从 `24`（4×(解码+编码)）降到 `6`，8 位存储点从
`12` 降到 `3`。

## 色卡偏差上界

8 位输出与"连续值参考"的差只来自舍入：理想值 `p` 与存储值每通道相差不超过
0.5 个码值。上界就取 `p ± 0.5` 这个 1 码宽立方体内能到达的最大 CIELAB 距离
（5×5×5 采样），逐色块计算；参考 `p` 由另一条 `long double`、无中间 8 位存储
的独立实现给出。

自检断言：

- `chart_max_de_unified <= chart_bound`，且逐色块都有 `de_unified <= bound`。
  实测 `0.5927 <= 0.7663`，24 个色块全部通过。
- 全图每个像素都在半码内：`max_rounding_error = 0.499997`，
  `pixels_over_half_code = 0/13824`。
- 旧链路对照：`legacy_over_bound = 24/24`，最大偏差 `9.4759`，是"偏色"的量化证据。

## 关闭统一

`unify = 0` 走回四段各用自己曲线的旧路径。自检对该路径重算一遍，把四个环节
的输出缓冲和最终图像逐一 `memcmp`，并额外与冻结的基线二进制
`tests/golden.legacy`（4×41472 字节）比对：

```
unify_off_identity_stage0=1 stage1=1 stage2=1 stage3=1 image=1
golden_legacy_bytes=1
```

## 验收材料

```bash
make test       # 全部断言 + 基线逐字节比对
make artifacts  # 生成 artifacts/ 四个文件
```

| 文件 | 内容 |
|---|---|
| `pixel_summary.txt` | 输出图像与四个环节的 FNV-1a 逐位摘要、通道 min/max/mean、最大偏差 |
| `chart_deviation.csv` | 24 色块的理想/统一/旧链路 RGB、`de_unified`、`bound`、`de_legacy`、是否在界内 |
| `conversions.txt` | 两条链的解码/编码/重量化次数与每像素计数 |
| `repro.txt` | 复现命令与预期输出 |

gcc 与 clang、`-O0` 与 `-O2` 产出的 `artifacts/` 逐字节相同。
