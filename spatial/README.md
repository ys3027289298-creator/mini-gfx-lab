# mini-spatial-audio

双耳定位，水平角与仰角两条路径解耦：

- **方位角** → ITD（最大 0.6 ms）+ IID
- **仰角** → ITD 投影因子 `cos(el)`（水平最大、两极归零）+ 耳廓高频搁架（2 kHz 一阶低架，仰角不对称，头顶与脚下音色不同）
- 反距离衰减、多声源软限幅混音
- 仰角超出 `[-π/2, +π/2]`（含 NaN/inf）直接返回 `SPATIAL_ERR_ELEVATION`，**不夹边界**

```bash
make test              # 构建并自检
make regen-reference   # 重新生成逐角度参考曲线表 src/reference_data.h
```

## 参考曲线

`tools/gen_reference.py` 是可复现的逐角度“测量”生成器（固定随机种子），输出 ITD 因子与高频衰减两条参考曲线，嵌入 `src/reference_data.h`。自检时：

- ITD 与几何模型 `cos(el)` 逐角度比对（容差 0.03）
- 高频曲线先最小二乘拟合 `c0 + c1·sin(el) + c2·sin²(el)`，再逐角度报误差（容差 0.25 dB）
- 每个角度单独打印 `OK/FAIL`，任一点超差即整体失败
