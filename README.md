# mini-gfx-lab

游戏与图形方向的小项目集合，每个子目录可独立构建、可自测。

| 目录 | 内容 |
|---|---|
| `navgrid/` | 8 方向 A*、禁止切角、视线平滑 |
| `spatial/` | 双耳定位 ITD/IID、距离衰减、软限幅混音 |
| `particle/` | 固定容量粒子池、每帧配额、优先级调度 |
| `lod/` | 屏幕误差 LOD 选择与迟滞 |
| `tilemap/` | 分层瓦片地图、4 邻接自动拼接、区域剔除 |
| `mixer/` | 总线混音、侧链闪避、软限幅 |
| `fontatlas/` | 字形货架装箱与字距调整 |
| `tween/` | 关键帧时间线与缓动 |
| `diag/` | 统一诊断与降级（particle / tilemap / navgrid / fontatlas） |

## 统一诊断与降级（diag/）

四个绘制模块各带验收检查，策略统一、证据格式各自不同：

| 模块 | 证据格式 |
|---|---|
| `particle` | 数值表（逐帧 spawned/alive/high_water） |
| `tilemap` | 运行日志（逐项 check 记录） |
| `navgrid` | 产物哈希（路径/网格 FNV-1a 64） |
| `fontatlas` | 可复制复现命令 |

约定：

- 默认构建（`make`）诊断完全关闭：零额外分配、零额外输出，行为与之前一致。
- 诊断构建（`make -C <mod> diag-test`，或根目录 `make diag-test`）：
  验收全过 → stdout 与默认构建逐字节相同；
  任一失败 → 只丢该模块绘制（stdout 为空、证据走 stderr、退出码 2），其余模块照常。
- 故障注入（仅诊断构建）：`MINI_GFX_FAULT=<模块名>` 或 `MINI_GFX_FAULT=all`。
  根目录 `make diag-demo MODULE=tilemap` 可演示单模块降级。
