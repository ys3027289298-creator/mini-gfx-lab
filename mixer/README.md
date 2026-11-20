# mixer

多总线立体声混音，每条总线与主输出各持有一条**可配置插件链**。`make test` 跑自检。

## 架构

- 链由有序节点组成，节点带类型化参数（`Node.params[]`）与内部状态（`Node.state[]`）。
  现有节点：`gain`、`duck`（侧链闪避，参数含侧链总线号）、`lowpass`（有状态一阶）、`softclip`。
- 链编辑：`chain_add / chain_insert / chain_remove / chain_move` 运行时调整顺序；
  `node_set_param` 改参数；`node_set_bypass` 切旁路。固定容量数组，处理路径零分配。
- 旁路无爆音：`wet` 按采样在 5ms 内交叉淡入淡出，端点吸附到精确的 0/1，
  因此稳态输出与硬切换**位级一致**；旁路期间有状态节点（lowpass）继续跟踪干信号（latent 更新），
  恢复时不产生突变。
- 整体状态：`mixer_save / mixer_load` 序列化为带 magic/version 的版本化 blob，
  覆盖链顺序、节点类型、全部参数、旁路标志与节点内部状态。载入先校验到临时对象，
  任何字段非法（magic/version/计数/类型/NaN/Inf）都拒绝且不破坏当前混音器。
- 线性开销：每块只做固定栈上 scratch 缓冲，节点数翻倍则处理时间近似翻倍（自检打印 ratio）。

## 逐样本一致性

`ref_mix()` 保留原始硬编码信号路径（汇流排 → 闪避 → 总线增益 → 求和 → 软限幅）作为参考。
`build_legacy()` 用插件链按相同运算顺序复刻它（浮点乘法不满足结合律，故 gain→duck 的
顺序与求和的结合方式都与原表达式一致）。`chain_matches_ref_exact` 对原始场景与 20 组
随机参数/信号做**位级**（非 epsilon）比对；旁路切换块因斜坡天然不同，斜坡收敛后的稳态块
与参考逐样本相等。

## 自检清单（`make test`，退出码非 0 即失败）

| 检查 | 验收点 |
|---|---|
| `chain_matches_ref_exact` | 插件链与硬编码路径逐样本相同（含随机用例） |
| `order_and_params` | 节点参数、增删、移动、插入、容量与非法参数拒绝 |
| `state_roundtrip_fields/processes_exact` | 状态往返后每个参数/状态逐个相同，后续渲染位级一致 |
| `state_rejects_corrupt` | 损坏/截断 blob 被拒绝且不污染现有状态 |
| `bypass_no_click` / `bypass_endpoints_exact` | 旁路斜坡最大跳变远小于硬切换，端点精确 |
| `bypass_resume_continuous` / `filter_alive_after_restore` | 旁路可恢复，状态保活，恢复后滤波器仍工作 |
| `linear_cost` | 链长 8→16 的耗时比（期望约 2，上界 3） |
