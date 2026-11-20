# volfog

体积雾：视线步进积分 + 密度场衰减（Beer-Lambert）+ Halton 低差异序列抖动消带状。

- 关闭体积雾时走 legacy 均匀雾路径，画面与原来逐位一致
- 自测与高步数（2048 步）参考渲染比对，按 sky/horizon/floor 三个区域报 RMSE/max 误差
- 步数减半时用 Richardson 外推给出误差上界并对参考验证
- 负密度、零步数直接拒绝

```bash
make test
```
