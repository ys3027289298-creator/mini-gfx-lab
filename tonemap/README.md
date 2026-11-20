# tonemap

HDR → LDR 色调映射链：曝光、曲线、抖动三段按配置顺序串联，用来替代
“直接钳到 [0,1]”导致亮部全白的做法。

- **曝光**：线性缩放 `x · 2^stops`，刻意不钳位，让高光保持 >1 交给曲线处理。
- **曲线**：Reinhard 肩部 `f(x) = kx/(kx+1)`，单调、`f(0)=0`、`f(∞)→1`，
  闭式可逆 `x = y/(k(1-y))`；`curve_on` 可整体关闭。它趋近 1 较慢，
  热点不会像快肩曲线那样直接塌成纯白。
- **抖动**：层间梯度噪声（IGN），零均值、幅度一个量化级，量化前施加。

## 自检关注点

`make test` 用合成场景（平滑天空 + 亮盘 + 镜面尖峰）输出以下数据：

| 指标 | 含义 |
|---|---|
| `clipped_truncated` / `clipped_chain` | 直接截断与整条链留下的白像素数，后者应显著更少 |
| `matches_manual_*` | 分阶段循环与逐像素手算公式逐点一致（按完整排列分派） |
| `curve_roundtrip_err` | 曲线 `inv(fwd(x))` 的最大误差 |
| `subcode_ramp_codes_*` | 跨度不足一个量化级的渐变，抖动前只出 1 个码，抖动后 >1 |
| `stripe_dev_undithered/dither/bayer` | 平坦区行/列均值相对理想码的最大偏差。无抖动 0.5 码、IGN 0.03 码、Bayer 0.25 码 |
| `order_changed_px` / `order_mean_abs_code_diff` | 交换曝光与曲线后的像素差异量，证明换序可对比 |
| `highlight_std_*` | 高光码值离散度；抖动放最前会先把高光钳死，离散度归零 |

抖动图案选 IGN 而非 4×4 Bayer，是因为 Bayer 矩阵各行/列均值不相等，
平坦区会出现 4 像素周期的条纹（上表 `stripe_dev_bayer`）。README 里的数值
均由自检实际打印，不是估计值。

```bash
make test
```
