# mini-dubins

曲率受限最短路径：按起点/终点朝向构造最短 Dubins 路径（LSL/RSR/LSR/RSL/RLR/LRL 六字枚举取优），
给定最小转弯半径，离散为数量可配的跟随采样点（含航向），曲率处处 ≤ 1/R；
并用独立的数值优化（直接配点 + 罚函数 + Adam）交叉验证路径长度差在 5% 以内。

```bash
make test
```
