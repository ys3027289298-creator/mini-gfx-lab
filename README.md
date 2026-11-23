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
| `common/` | 共享诊断层（`diag`）、失败点注册表、画布、验收工具 |
| `chain/` | 把 particle / tilemap / navgrid / spatial 合成一帧的渲染链，支持降级 |

## 统一诊断与降级渲染

四个参与渲染的模块（`particle` `tilemap` `navgrid` `spatial`）共用
`common/diag`：

- **失败点统一注册**在 `common/failpoints.h`，每条都有 `module:point` 与
  `DiagCode`。所有失败返回都走 `DIAG_FAIL_RETURN`，返回 `DIAG_ERR(code)`
  （`-100-code`），不会与模块内部的 `-1` 哨兵混淆。
- **降级渲染**：`chain/src/chain.c` 把画面分成四条带，任一阶段失败只清空并
  标注它自己的那条带，其余三条逐字节不变；返回码与 `degraded=N` 表明失败数。
- **诊断关闭零额外分配**：`diag` 只用静态环形缓冲，禁用时连格式化都不做；
  验收用 `--wrap=malloc/calloc/realloc/free` 计数，断言开关前后分配次数相同。
- **未覆盖失败点可自建列查**：`tools/coverage_report.py` 列出每个失败点是否
  有源码站点、故障夹具、验收用例，并把绕过诊断层的 `return -1` 报为
  `UNREGISTERED`。

### 运行

```bash
make acceptance            # 跑齐四个模块 + chain 的全部验收
make -C particle acceptance # 单个模块：套件 + 覆盖 + golden 逐字节比对
DIAG_ENABLE=1 ./chain/render-chain           # 失败时把结构化诊断打到 stderr
DIAG_FAULT=navgrid:no_path ./chain/render-chain  # 注入失败，仅 navgrid 那条带降级
```

每个模块的验收数据都在 `<module>/tests/`：`golden.stdout` / `golden.rc` 是
成功路径的原始输出，`faults/<point>.fault` 是失败点夹具，`harness.c` 是
数据驱动的验收用例。成功路径的输出与改动前的原始构建逐字节一致。
