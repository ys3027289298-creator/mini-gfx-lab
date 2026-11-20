# camera

2D 跟随相机。`make test` 跑自检。

- **跟随**：临界阻尼弹簧（GPG4 smooth damp），带目标速度预测（lookahead），
  弹簧状态放在目标坐标系内，匀速目标零稳态滞后，净提前量 = 目标速度 × `lead_time`
- **约束**：视野矩形钳制在世界边界内；墙体用子步扫掠 + 分轴滑动，大 dt 也不穿墙
- **震动**：可叠加多个 shake，各自衰减过期，总幅度钳制在 `max_shake` 内；
  震动是渲染偏移（`shake_x/shake_y`），不参与碰撞
- **参数切换**：`cam_set_params` 硬切只改参数不动状态，`cam_tween_params` 渐变，
  lead 本身也做平滑，切换不打嗝
- 目标/相机瞬移后调 `cam_snap` 重新锚定，避免长距离回拉
