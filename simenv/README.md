# simenv

可注入的时间源与随机源。所有模块对"现在几点 / 这一帧多长 / 下一个随机数"的
需求都只经过本模块的统一接口，因此同一份输入序列可以逐帧、逐粒子地重放，
替换实现不需要改任何业务模块源码。

## 设计

```
模块 (particle / tween / ...)
   │  只调用 static inline 访问器：sim_env_dt / sim_env_randf / sim_env_randf_n ...
   ▼
SimEnv { Clock* clock; Rng* rng }     ← 两个函数表（vtable）
   │
   ├── Clock: realtime / scripted / recorder
   └── Rng  : pcg32 / xorshift32 / seq / recorder
```

- `SimEnv` 只持有两个指针，访问器是 `static inline`，调用成本是一次间接跳转。
- `env == NULL` 时走 ambient 默认实现（实时钟 + PCG32），模块也可以完全不存
  `SimEnv*`，直接用 `sim_now()/sim_randf()` 这类简写。
- 换实现有两条路，都不碰模块源码：
  - `sim_env_set_clock/set_rng`：换某个 env 上的单个源；
  - `sim_set_ambient`：换全局默认 env。

### 时间源

| 构造 | 行为 |
|---|---|
| `sim_clock_realtime()` | `now()` 读 `CLOCK_MONOTONIC`，`tick()` 返回真实帧间隔 |
| `sim_clock_scripted(dts,n)` | 按固定 dt 序列推进，确定性，耗尽后重复最后一个值 |
| `sim_clock_recorder(inner)` | 装饰器，记录每次 `tick()` 的 dt |

### 随机源

| 构造 | 行为 |
|---|---|
| `sim_rng_pcg32(seed)` | PCG-XSH-RR，默认 PRNG，跨平台位级一致 |
| `sim_rng_xorshift32(seed)` | 第二种实现，用于证明接口可替换 |
| `sim_rng_seq(values,n)` | 重放固定 uint32 序列（循环），不依赖原 RNG 算法 |
| `sim_rng_recorder(inner)` | 装饰器，记录每次产出的 uint32 |

- 浮点统一用 `sim_env_randf`：`(u32 >> 8) / 2^24`，落在 `[0,1)`，任何 RNG 实现
  产出的浮点口径完全相同。
- 批量接口 `sim_env_randf_n(env, out, n)` 一次间接调用取回 n 个值，取值顺序和
  数值与 n 次标量调用**逐位相同**（见自检 `fill_matches_scalar`）。生成热点用它
  把每粒子 3 次分派降为 1 次。

## 录制与回放（tape）

一次运行 = 一串帧 dt + 一串 RNG 产出。`sim_tape_save` 把两者写成小端定宽文件，
`sim_tape_load` + `sim_tape_env` 在另一个进程里用 `scripted` 时钟 + `seq` 随机源
重建出完全相同的输入，原实时钟和原 RNG 算法都不再参与。文件头带 magic/version
校验，损坏文件加载失败。

## 确定性保证（验证方式）

`make test` 依次运行：

1. `simenv-selftest`：同种子流一致、reset/peek 语义、批量与标量逐位相同、
   tape 存取往返、损坏检测、env 热替换。
2. `replay-demo`：对真实粒子模块做 FNV-1a 全池哈希，逐帧比较——
   - A 同序列两次独立运行逐字节一致；
   - B 录制→tape 文件→换实现重放，逐字节一致；且与全新 pcg 运行一致；
   - C 仅通过 `sim_env_set_rng` 换成 xorshift，流改变且自身可复现。
3. `tape-cross`：一个进程写 tape 文件，另一个进程读回，哈希列 `diff` 为空。
4. `dispatch-micro`：纯间接跳转增量 × 每帧真实分派次数，得到不依赖计时稳定性的
   开销上界。
5. `guard`：grep 守卫，业务模块禁止直接出现 `rand/srand/random/clock_gettime/
   gettimeofday/time(...)`，时间与随机只能走 simenv。

确定性在 `-O0/-O2/-O3` 下逐字节成立；所有测试在 ASan+UBSan 下零泄漏、零告警。

## 性能

间接调用增量约 1.3 ns/次（x86-64）。粒子帧每帧 50 次批量分派 + 1 次时钟 tick，
实测帧时长约 8.4 µs，解析上界稳定在 **0.2%–0.4%**，预算 2%（`dispatch-micro`
硬性判定）。墙钟直接对比在有调度噪声的机器上波动很大，只作诊断输出，不作判据。

## 用法

```c
#include "simenv/simenv.h"

/* 确定性运行：脚本时钟 + 固定种子 */
SimEnv env = { sim_clock_scripted(dts, n), sim_rng_pcg32(seed) };
for (int f = 0; f < n; f++) {
    float dt = (float)sim_env_dt(&env);
    float r = sim_env_randf(&env);
    step(&world, dt, r);
}

/* 热替换：业务代码一行不改 */
sim_env_set_rng(&env, sim_rng_xorshift32(seed));
```
