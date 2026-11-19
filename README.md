# mini-gfx-lab

游戏与图形方向的小项目集合，每个子目录可独立构建、可自测。

| 目录 | 内容 |
|---|---|
| `simenv/` | 可注入时间源/随机源、固定序列重放、录制回放磁带 |
| `navgrid/` | 8 方向 A*、禁止切角、视线平滑 |
| `spatial/` | 双耳定位 ITD/IID、距离衰减、软限幅混音 |
| `particle/` | 固定容量粒子池、每帧配额、优先级调度（经 simenv 取时间/随机） |
| `lod/` | 屏幕误差 LOD 选择与迟滞 |
| `tilemap/` | 分层瓦片地图、4 邻接自动拼接、区域剔除 |
| `mixer/` | 总线混音、侧链闪避、软限幅 |
| `fontatlas/` | 字形货架装箱与字距调整 |
| `tween/` | 关键帧时间线与缓动（经 simenv 驱动时钟） |

## 可复现仿真

时间与随机统一经 `simenv/` 接口获取，禁止模块直连 `rand()`/`clock_gettime()`
（`make -C simenv guard` 守卫）。同序列两次运行逐帧逐粒子逐字节一致，并可通过
tape 文件跨进程重放，详见 [`simenv/README.md`](simenv/README.md)。
