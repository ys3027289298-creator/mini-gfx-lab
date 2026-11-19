# tilemap

分层瓦片地图：4 邻接自动拼接、区域剔除，以及按图层配置的混合模式。

```bash
make test
```

## 混合模式

图层自下而上合成进 RGBA8 帧缓冲。每层可单独配置模式与透明度：

```c
map_set_blend(&m, layer, BLEND_MULTIPLY, 0.6f);   // 模式 + 0..1 透明度
```

| 模式 | 公式（cb 为下层底色，cs 为本层源色，均为 0..1） |
|---|---|
| `BLEND_NORMAL` | `B = cs` |
| `BLEND_MULTIPLY` | `B = cb * cs` |
| `BLEND_SCREEN` | `B = cb + cs - cb * cs` |
| `BLEND_OVERLAY` | `B = 2*cs*cb`（`cb <= 0.5`），否则 `1 - 2*(1-cs)*(1-cb)` |

合成采用 source-over：

```
Co = (1 - alpha) * Cb + alpha * B(Cb, Cs)
Ao = alpha + (1 - alpha) * Ab
```

帧缓冲是 RGBA8，因此每个图层 pass 读 8 位、写舍入后的 8 位结果。
`blend_px8()` 是这一 pass 的权威定义，其余路径必须逐字节一致。

## 实现与开销

- `composite_ref()` 逐像素直接套公式，作为参照实现。
- `composite_frame()` 在图层混合配置变化时构建一次 256×256 查找表
  （模式与透明度都烘焙进表），之后每像素只做一次查表加一次写入；
  配置不变时不会重建，逐帧渲染没有公式计算。
- 完全不透明的覆盖结果与原图逐像素相同；`alpha = 0` 或隐藏图层不产生
  任何写入，也不会触发建表。
- 自测内含与逐像素加法基线（`composite_additive()`）的耗时对比，
  要求混合每像素开销不超过加法。

`make test` 的自检覆盖：原有自动拼接/剔除用例、三种模式的公式值、
随机地图上查表路径与公式路径逐字节一致、全不透明覆盖与原图相同、
配置校验（非法模式/透明度/NaN），以及混合与加法的耗时比。
