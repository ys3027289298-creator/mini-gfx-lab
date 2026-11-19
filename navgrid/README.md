# mini-navgrid

8 方向 A*、对角禁止切角、视线可达平滑，以及编队移动实验。

```bash
make test
```

## 编队移动

整队共用一条路线，成员按固定相对偏移围绕锚点（route anchor）保持队形，而不是各自寻路。

| 需求 | 实现 |
|---|---|
| 按相对偏移维持队形 | `slots_formation()`：成员站位 = 锚点 + 固定偏移 |
| 遇障碍整体改道 | 先规划「队形足迹可通行」的路线（`build_clearance()` 把会让任一成员撞墙的锚点标记为不可通行）；整队走同一条改道路线 |
| 窄口临时收拢并记录 | `mode_at()` 在队形放不下前 n 格时切到单列纵队；`compress` / `expand` 事件记录 tick 与位置 |
| 收拢后展开不会自我堵死 | 收拢状态保持到队尾成员也通过最后一个窄点后才展开 |
| 掉队单位重新靠拢 | 落后超过 `LAG_GAP` 格触发 `regroup` 事件；锚点等待（`waits`），掉队者两步追赶；直线被挡时用带占用的 A* 绕回槽位 |
| 单位不互相穿插 | 移动按 chebyshev 距离单调下降，对角步要求两侧正交格同时空出 |
| 到达误差 < 1 格 | 终点处每个成员到理想槽位的最大欧氏距离 < 1 才算 `arrived` |

整队无法维持开阔队形时（路线退化），会改用单列纵队穿过窄口；若收拢状态一直持续到终点，到达判定改用纵队槽位。

## 与逐单位独立行进对比

`make test` 输出三组场景。独立基线是每个单位各跑一次普通 A*，互不协调。

```
detour (narrow shortcut y=4, group can still cross at y=6..9):
detour formation: ticks=23 steps=121 max_spread=2 rel_err=0.000 compress=0 expand=0 regroup=0 waits=0 replans=0 route=21.24
detour independent: ticks=18 steps=105 route=18.41 pinch_uses=6
detour step_delta=+16 (+15.2%)

narrow (only a 1-wide mouth at y=4):
narrow formation: ticks=23 steps=115 max_spread=2 rel_err=0.000 compress=1 expand=1 regroup=3 waits=3 replans=1 route=17.00
narrow independent: ticks=18 steps=105 route=18.41 pinch_uses=6
narrow step_delta=+10 (+9.5%)

lag (unit 5 spawns 4 cells behind its slot):
lag formation: ticks=17 steps=84 max_spread=2 rel_err=0.000 compress=0 expand=0 regroup=1 waits=1 replans=0 route=13.00
lag independent: ticks=13 steps=78 route=13.00 pinch_uses=0
lag step_delta=+6 (+7.7%)

total (3 scenes) formation_steps=320 independent_steps=288 delta=+32 (+11.1%)
```

| 场景 | 编队 | 独立 | 结论 |
|---|---|---|---|
| 窄口 + 可绕行的宽通道 | 整队改走 y=6..9，0 单位挤窄口 | 6 单位全部各挤窄口 | 编队多走 15% 格数，换来整队不分散 |
| 只有一个窄口 | 收拢 1 次 / 展开 1 次，3 次掉队等待 | 6 单位逐个挤过 | 编队多 9.5% 格数，保持队列 |
| 一名成员起步落后 4 格 | 1 次 regroup，先头等 1 tick | 该单位自行追 | 编队多 7.7% 格数，到达误差仍 < 1 格 |

三场合计 320 对 288，编队多 11.1%。多出的代价全部来自「整队绕远」和「等掉队者」；独立行进更省总步数，但单位会分散穿口、且没有队形约束。

注意基线口径：独立组是「每单位各规划一次静态 A*」，不计单位间碰撞与排队。所以它的步数是路径代价下界，不是可实际执行的时间；`pinch_uses=6` 表示 6 个单位都**规划**着去挤同一格窄口，真跑时会互相堵住。编队组的 `steps` 则是逐 tick 带占用检测执行出来的，两者在窄口场景下不完全同口径，宜按「路径代价 vs 协调执行」理解。
