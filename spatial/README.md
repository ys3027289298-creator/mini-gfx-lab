# mini-spatial-audio

双耳定位：方位角→ITD/IID、反距离衰减、多声源软限幅混音。

网络路径：每帧每声源 31 bit（`src/spatial_quant.h`），代替 3×float 位置。
ITD 码与 IID 档位随码字显式传输，方位角只在两者都不变的“决策区间”内细分 16 格，
因此解码端重现的 ITD/IID 与浮点端逐位一致（`binaural.h` 为两条路径共用）。

```bash
make test
```
