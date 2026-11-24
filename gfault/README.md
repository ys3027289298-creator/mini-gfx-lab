# gfault — 五段统一失败编码与按段隔离

## 背景

粒子、图块、网格、字体、调色板五处各写各的失败：裸 `-1`、静默 `return`、
打印 `"unknown"`，程序侧无法判断是谁倒了，美术侧观察到的现象是整块画面
消失。gfault 把五处失败归到同一套全局编码，按模块分段，并让合成器在某段
失败时只抽掉该段的绘制，其余段照常出画。

## 1. 统一编码（按模块分段）

每个模块占一个 16 码的段，从码本身即可反推出归属模块。函数返回值统一为
码的负数（成功返回 0），兼容既有的 `rc < 0` 调用约定。

| 段 | 模块 | 码段 | 具体失败码 |
|---|---|---|---|
| 0 | particle 粒子 | `0x10–0x1F` | `0x10` 发射器表满 `EMITTER_FULL`、`0x11` 粒子池耗尽 `POOL_EXHAUSTED`、`0x12` 未知发射器 `BAD_EMITTER`、`0x13` 非法参数 `BAD_ARG` |
| 1 | tilemap 图块 | `0x20–0x2F` | `0x20` 坐标越界 `OUT_OF_RANGE`、`0x21` 非法图层 `BAD_LAYER`、`0x22` 预留 `BAD_TILE` |
| 2 | navgrid 网格 | `0x30–0x3F` | `0x30` 端点不可走 `UNWALKABLE`、`0x31` 无路径 `NO_PATH`、`0x32` 网格畸形 `BAD_GRID`、`0x33` 暂存分配失败 `ALLOC`、`0x34` 平滑路径超容量 `SMOOTH_CAP` |
| 3 | fontatlas 字体 | `0x40–0x4F` | `0x40` 字形尺寸被拒 `GLYPH_REJECTED`、`0x41` 货架空间耗尽 `ATLAS_FULL`、`0x42` 字形表满 `GLYPH_TABLE`、`0x43` 未装箱码点 `NO_GLYPH` |
| 4 | palette 调色板 | `0x50–0x5F` | `0x50` 索引越界/未注册 `INDEX_RANGE`、`0x51` LUT 满 `LUT_FULL`、`0x52` 颜色非法 `BAD_COLOR`、`0x53` 非法参数 `BAD_ARG` |

- 所有码在 `gfault.h` 中集中定义，`gfault_strerror()` 返回字符串字面量，
  任何真实失败都不会再落成 `"unknown"`（有测试 `no_unknown_strings` 兜底）。
- `gfault_segment_of(code)` 从码反查模块；合成器据此决定抽哪一段。

## 2. 诊断汇（零堆分配）

`gfault_report()` 把失败写进一个 **64 条的静态环形缓冲**（`gfault.c` 内
`static Diag`），记录序号、帧号、段、码、源码行。环形满后覆盖最旧记录并
累加 `dropped` 计数，不崩溃、不阻塞渲染。

- 全部存储为静态/栈内存，**无 malloc/calloc/realloc**，无变参格式化。
- `gfault_set_enabled(0)` 可让汇直接短路；渲染路径在两种模式下都不分配。
- 每帧 `gfault_frame_begin()` 复位各段锁存；段内首次失败被锁存，
  `gfault_segment_failed()` 供合成器查询。
- 故障注入 `gfault_inject.h`：测试用，按段武装一次性/多帧故障，状态也是
  静态整型，不引入堆流量。

## 3. 按段隔离的合成模型

`render/framebuf.c` 提供 320×240 RGBA8888 软件帧缓冲，五个面板各有固定
区域（见 `fb_region()`）。合成器（`test/compositor.c`）每帧：

1. `gfault_frame_begin()`；
2. 逐段调用桥接渲染；某段返回负码或该段被锁存失败，就把**该面板清回背景**
   （抽掉这处绘制），其余四段继续；
3. 导出一张合成图 + 五张分段截图。

任何单段失败都不会中止整帧，验证见 `artifacts/evidence/isolation_manifest.txt`
（5 段 × 5 面板 = 25 项像素级隔离检查全过）。

## 4. 验证证据（`artifacts/evidence/`）

- **逐像素一致（五张分开，不用一张代表五处）**：用 git worktree 取改动前
  提交 `517b0fe` 的四个模块源码出 golden 图，当前源码出 current 图，逐段
  SHA-256 + 二进制 `cmp`。五段全部 `0 differing bytes`，见
  `pixel_manifest.txt`。palette 为本次新增模块，其 golden 是首版冻结输出。
- **零额外堆分配**：`test/alloc_counter.c` 以强符号拦截 libc 分配器并计数。
  12 帧渲染在诊断开/关时都是 24 次分配（全部来自 navgrid A* 既有的
  calloc/malloc 暂存，与诊断无关），**诊断新增分配差值 = 0**；纯诊断循环
  1000 次迭代分配差值 = 0。见 `logs/alloc_test.log`。
- **真实失败路径**：`test/natural_fault_test.c` 触发各模块真实失败（非注入），
  确认码落对段、无 unknown，并覆盖环形缓冲溢出丢弃策略。
- 复跑：`python3 -I scripts/collect_evidence.py`（依赖 `/tmp/mgl-golden`
  worktree，可用 `git worktree add /tmp/mgl-golden 517b0fe` 重建）。

## 5. 本次没有兜住的失败点（明确列出）

1. **进程级崩溃不在编码范围内**：段内的空指针解引用、栈溢出、`abort()`、
   信号（SIGSEGV 等）不会转成 gfault 码——编码只能表达「函数返回」级别的
   失败。需要崩溃隔离得另上进程/看门狗方案。
2. **环形缓冲容量有限（64 条）**：故障风暴下最旧记录被覆盖（有 `dropped`
   计数和测试，但不回溯保留）。高频持续故障应接外部日志汇，当前未做。
3. **单线程假设**：`Diag` 是普通静态变量，无锁；多线程并发 report 的数据
   竞争未覆盖。本仓库各模块本身也是单线程模型。
4. **未触发/仅预留的码**：`GFAULT_P_BAD_EMITTER`、`GFAULT_T_BAD_TILE`、
   `GFAULT_F_GLYPH_TABLE`、`GFAULT_F_NO_GLYPH`、`GFAULT_PAL_BAD_COLOR` 在
   当前 API 的自然调用下走不到（容量/入参约束使其不可达），只经代码审查与
   注入验证，没有自然路径测试。
5. **navgrid 的暂存分配失败 `N_ALLOC (0x33)`**：malloc 失败无法在不模拟
   分配器的情况下自然触发，目前仅注入路径覆盖；且该模块仍保留堆分配
   （诊断不增加分配，但 A* 本身不是零分配的）。
6. **帧缓冲落盘失败未纳入分段码**：`fb_write_ppm()` 的 fopen/fwrite 失败
   目前返回通用负值，不属于五个绘制段，未占用 0x10–0x5F 码段。
7. **范围外的三个音频/动画模块**：spatial、mixer、tween、lod 不在本次五段
   清单内，它们的失败没有接入 gfault；后续如要统一需再分码段（建议
   0x60 起）。
8. **调色板的颜色有效性**：`color_valid()` 当前恒真（8 位通道天然合法），
   `BAD_COLOR` 是为 HDR/色域校验预留的钩子，暂无实际触发条件。
