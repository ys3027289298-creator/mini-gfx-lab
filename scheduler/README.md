# scheduler

采样级音频事件调度。`make test` 跑自检。

- 事件按帧投递（`sched_post` / `sched_post_frame`），渲染时量化到整数采样位置，最大对齐误差 < 1 个采样（四舍五入，理论上界 0.5）。
- 延迟补偿可配：`sched_set_delay_samples`（采样数）或 `sched_set_delay_ms`（毫秒），从目标采样位置整体前挪；落在已渲染区间的事件计入 `late` 并丢弃。
- 同一音频块内事件按 `(采样位置, 入队序号)` 排序后触发，保证同帧多事件触发顺序确定、可复现。
- 输出与逐样本手工排布的参考实现（`reference_render`）逐字节一致（`memcmp` 校验，`waveform_bit_exact=1`）。
- 固定容量事件队列（4096），满队投递计入 `dropped` 并返回 -1。
- 自检末尾打印 `bench n=...` 报告：事件数与调度耗时（纯 dispatch 的 ns 与每事件均摊，以及含波形渲染的整块耗时）的关系。
