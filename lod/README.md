# lod

屏幕误差 LOD 选择 + 迟滞切换，以及与该选择解耦的动画采样 LOD。

```bash
make test
```

## 动画 LOD

- 动画播放相位始终按真实帧间隔累积，与渲染帧率无关。
- 姿态采样按 LOD 级降频：`stride = {1, 2, 4, 8}` 帧，60 fps 下即 60/30/15/7.5 Hz。
- 切换 LOD 只改采样步长，不碰相位与已采样姿态，因此切级处相位连续。
- 测试输出与全帧率动画的姿态差：`pose_err_max` / `pose_err_mean`（世界单位）、
  `pose_angle_err_max_deg`、`lag_max_ms`，以及 `phase_discontinuities`（要求为 0）。
