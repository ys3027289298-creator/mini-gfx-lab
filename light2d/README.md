# light2d

二维光照与阴影遮罩：按光源位置生成遮挡多边形、按距离衰减、多层叠加，配合昼夜环境光。

- **遮挡多边形**：每个光源生成一个闭合的星形可见性多边形。
  - `RING_RAYS` 条固定环射线覆盖整圆；
  - 每个遮挡矩形角点发两条射线（角左右各偏 `CORNER_EPS`），但**都发射在角的角度上**，
    于是形成一条零宽度的径向边。射线判定在角处从“打到面”跳到“射到半径”，
    多边形用这条径向边精确复现这个跳变，不会在阴影角上漏出一条光缝。
  - 推论：多边形硬边界与逐像素射线判定逐格一致。`polygon_matches_raycast` 在
    2 套遮挡布局 × 5 个光源位置 × 320×180 网格上逐格比较，全 0；
    `silhouette_boundary_exact` 再以 72000 个角度扫描边界半径，最大误差 < 1e-3。
- **距离衰减**：`light_falloff` —— 15% 半径内为热核平台，之后 smoothstep 到半径归零。
  多边形按扫描线整行填充后，衰减是距离的纯函数，不依赖三角形插值。
- **多层叠加**：
  - 遮挡体带高度带 `[z0,z1]`，光源带高度 `z`，只有 `z∈[z0,z1]` 才被挡。
    低矮的墙不会挡住悬挂在它上方的路灯（`wall_misses_high_light`）。
  - 光源按高度归入 3 个图层，自底向上合成；每层可用 `layer_tint` 单独调色，
    合成模式支持 `BLEND_ADD` 与 `BLEND_MAX`（`layers_both_contribute`）。
- **昼夜**：
  - `ambient_from_clock(t)`：0=午夜、0.25=日出、0.5=正午、0.75=日落，smoothstep 插值，环绕闭合；
  - `daylight_from_clock(t)`：缩放 `LIGHT_SUN` 类光源强度，夜间为 0；
  - 阴影区在正午保留天空环境光而不是全黑（`day_shadow_keeps_sky_fill`）。
- **连续性与性能**：
  - 光源或遮挡体每步 0.01 单位移动时，多边形面积相对变化 < 0.2%；
    掩码不会出现整格亮暗翻转（`light_motion_continuous`、`occluder_motion_continuous`、`mask_steps_are_local`）。
  - 构建期不做任何分配；压力测试为 8 个光源 × 32 个遮挡体 × 320×180 网格、每帧全部移动，
    最坏帧约 0.8 ms（`frame_budget_3ms`，预算 3 ms）。

```bash
make test
```
