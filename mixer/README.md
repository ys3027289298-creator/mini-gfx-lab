# mixer

`make test` 跑自检。

延迟补偿：各总线 insert 的处理延迟用 `measure_latency()` 脉冲测得，
写入 `Mixer.latency[]`；混音前每条总线被延迟 `max_latency - latency[bus]`
帧，对齐到最慢的总线。`mixer_compensation()` 可查询当前补偿量。
补偿是相对于本次参与混音的总线集合的，所以单条总线 solo 时补偿为 0，
输出样本逐位不变。自检里附多音信号的频谱对比：未对齐时奇数倍
500 Hz 被梳状抵消，对齐后恢复平坦。
