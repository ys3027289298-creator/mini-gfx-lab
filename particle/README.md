# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度，以及同源特效的叠加规则。

## 叠加规则（按来源 `source` 注册）

| 规则 | 行为 |
| --- | --- |
| `RULE_STACK` | 同源特效独立共存，每次施放生成新实例 |
| `RULE_MERGE` | 同源只保留一个实例，重复施放按 `max_alive` 补齐，不超上限 |
| `RULE_REPLACE` | 新实例接管，旧实例粒子转入 FADING，剩余寿命截断到 0.5s 优雅退场 |
| `RULE_REJECT` | 同源特效活跃期间拒绝新施放，返回 `ERR_REJECTED` |

池满时先回收 FADING 粒子，再淘汰优先级**严格更低**的最弱活跃实例（同级不可互盖）；无对象可覆盖时返回 `ERR_FULL`。过期粒子与粒子耗尽的实例在 `update` 中自动回收。规则非法或同源重复注册不一致返回 `ERR_BAD_RULE` / `ERR_RULE_CONFLICT`，未注册来源施放返回 `ERR_UNKNOWN_SOURCE`。

```bash
make test
```
