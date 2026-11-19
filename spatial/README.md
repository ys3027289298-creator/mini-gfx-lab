# mini-spatial-audio

双耳定位：方位角→ITD/ILD、反距离衰减、多声源软限幅混音。

ITD 支持两种模型（`SpatialConfig.model` 可配）：

- `HEAD_MODEL_SPHERE`（默认）：Woodworth 球头模型，由头半径（默认 8.75 cm）
  与声速（默认 343 m/s）推导方位→延迟映射；远耳增益按归一化 Woodworth
  曲线插值（90° 衰减 `max_ild_db`，默认 20 dB），等功率归一。
- `HEAD_MODEL_SINE`：原简单正弦模型（`max_itd * sin(az)` + sqrt 声像分配），
  作为退化选项保留。

`make test` 会打印两种模型与 KEMAR 实测 ITD（文献近似值）的逐角度误差、
RMS/最大误差，以及左右耳增益表。球头模型 RMS 误差 ~10 μs，原正弦模型
~30 μs（90° 处最大 60 μs）。

```bash
make test
```
