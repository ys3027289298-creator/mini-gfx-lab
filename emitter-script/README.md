# emitters

发射器描述脚本：极简语法、单遍解析、执行语义与 `particle/` 的固定粒子池一致。
改发射器参数不再需要重新编译——脚本在运行时加载，也可用 `--watch` 热重载。

```bash
make          # ems-repl / ems-test / ems-bench
make test     # 十个示例脚本 vs 手写参照 + 语法错误行列号 + 边界用例（86 项）
make bench    # 解析与执行耗时
./ems-repl examples/02_priority.emitters
./ems-repl my.emitters --watch   # 每按一次回车重读脚本
```

## 语法

注释以 `#` 开始直至行尾；缩进与换行不参与解析，但每条语句要有空白分隔。

```
pool   64                  # 粒子池容量
budget 100                 # 每个 tick 的总发射配额
ticks  12                  # 运行 tick 数
dt     0.1                 # 每 tick 时长

curve slide(0, 9, linear)  # 曲线名(起点, 终点, 形状)

emitter spark priority 1 {
  quota 3                  # 每个 tick 的配额，可写曲线引用 @slide
  pos   0, 0               # x, y
  vel   1, 0.5             # vx, vy
  life  20                 # 寿命，单位与 dt 相同
}
```

| 元素 | 取值 |
|---|---|
| 数值 | 整数、小数、科学计数法，可带 `+`/`-` |
| 曲线引用 | `@name`，可用于 `quota` / `pos` / `vel` / `life` 的任一位置 |
| 曲线形状 | `linear` `quad_in` `quad_out` `cubic_io` `step` |

- `quota`、`pos`、`vel`、`life` 每个发射器必须各出现一次；缺失、重复、未知字段均为错误。
- 曲线必须先定义再引用；名字不可重复。定义顺序与发射器无关（可后置）。
- 省略 `budget` 视为不限；省略 `ticks` 为 1，省略 `dt` 为 1.0，省略 `pool` 则不允许发射。

## 求值语义

- 值在发射时刻求值：`t = tick * dt`，首个 tick 的 `t = 0`。
- 非 `step` 曲线以 1 为周期在 `from` 与 `to` 之间按形状插值（相位 `t - floor(t)`）。
  因此引用发射器自身的起始时刻时，曲线正好在头 1 秒内扫完一遍。
- `step` 在 `t < 1` 时取 `from`，之后取 `to`，适合做“闸门”。
- 小数配额按 tick 进位：`quota 0.25` 表示平均每 4 个 tick 发射 1 个。

## 执行语义

与 `particle/src/particle.c` 保持一致：

1. 每 tick 按优先级从高到低处理发射器（优先级相同按脚本顺序）。
2. 每个发射器消耗 `min(quota(t), 剩余预算)` 个名额；池满时拿不到槽位，预算不消耗。
3. 新粒子当帧就参与老化与移动；寿命到期在老化阶段释放槽位，下一 tick 才能复用。

## 错误报告

词法、语法、语义错误都带有 1 起始的行列号（列按字节计），并附源码行与 `^` 指针：

```
my.emitters:3:9: error: unknown curve '@missing'
  quota @missing
        ^
```

## 十个示例

| 脚本 | 覆盖点 |
|---|---|
| `01_single` | 单发射器、恒定参数 |
| `02_priority` | 预算 50 按优先级分配（对应 particle.c 场景） |
| `03_curve_quota` | 配额曲线 |
| `04_curve_velocity` | 速度曲线（quad_in） |
| `05_curve_life` | 寿命曲线 |
| `06_step` | `step` 闸门 |
| `07_cubic` | 位置曲线（cubic_io） |
| `08_pool_saturation` | 池满后高优先级占满、低优先级饿死 |
| `09_fractional_quota` | 小数配额进位 |
| `10_budget_split` | 预算不足时按优先级切分 |

`src/ref.h` 是对应十个脚本的手写 C 实现：常数已折叠、曲线已内联、没有解析器与通用调度，
与脚本结果逐字段比对（spawned / expired / alive / 每帧存活 / 峰值 / 位移与年龄和）。

## 开销

在本机（gcc -O2）实测：单个脚本解析约 **0.5–1.3 µs**，而运行十个 tick 约 **15 µs**；
解析只在加载时发生一次，相对每帧执行可忽略。benchmark 见 `make bench`。
