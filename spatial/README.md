# mini-spatial-audio

双耳定位 + 多抽头早期反射扩散。

- 直达声：方位角→每耳 ITD（≤ 0.6 ms）、IID、反距离衰减
- 扩散：抽头数与逐抽头衰减可配；抽头按确定性低差异序列铺在听者周围的球壳上，
  每个抽头有自己到听者的路程延迟和到达方位，左右耳分别累加
- 抽头数 1..64，衰减 0 ≤ decay < 1；抽头数增加时能量收敛到 1/(1-decay²)，
  不发散；抽头数为 0、衰减为负或 ≥1 都被拒绝
- 渲染结果与解析抽头叠加逐样本比较，误差给出浮点累加上界

```bash
make test
```

自检输出形如：

```
right45_peakL=84 right45_peakR=64
itd_right_ear_leads=1
...
tap_binaural_itd=1
ref_err_max=8.395e-08 ref_err_bound=3.797e-05
analytic_tap_superposition=1
energy taps=64 E=2.777778 limit=2.777778 gap=-1.952e-07
tap_energy_converges=1
tap_energy_tail_small=1
all_ok=1
```
