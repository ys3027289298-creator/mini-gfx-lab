# mixer

总线混音：每总线增益、人声侧链闪避、软限幅；可选 5.1 低音管理。

## 低音管理

`BassManager` 把 5.1 的 FL/FR/FC/SL/SR 在交叉频率处一分为二：低通部分
汇总进 LFE，高通部分留在本声道；LFE 输入原样通过。默认关闭，关闭时输出
与输入逐位相同。

```c
BassManager bm;
bm_init(&bm, 48000.0f, 80.0f, 24);   /* fs, 交叉频率, 斜率 12|24 dB/oct */
bm_set_enabled(&bm, 1);
bm_process(&bm, in, out, lfe_out, frames);
bm_crossover_report(&bm, CH_FL, &lp_db, &hp_db, &phase_deg);
```

- 斜率是 Linkwitz-Riley 阶数：24 → LR4（两节二阶 Butterworth），
  12 → LR2（两节一阶）。两支在交叉点都是 -6 dB。
- LR4 的低通+高通是全通，同相叠加；LR2 的两支在交叉点相差 180°，
  汇总时需注意极性。
- 交叉频率必须严格低于 fs/2，否则 `bm_init` 返回 -1 且保持禁用。
- `bm_set_route` 控制哪些声道把低频送入 LFE（默认为全部非 LFE 声道）。

```bash
make test
```
