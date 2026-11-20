# levelstream

关卡资源依赖图 + 后台流式加载：解析关卡 DAG（缺依赖 / 环会拒绝并报名字），
按 bottom-level（关键路径优先）调度固定 worker 池，依赖未就绪的资源不会开跑；
可玩门在必需闭包全部驻留前保持阻塞并列出未就绪资源。流式完成后的可玩摘要按
拓扑序计算，与全量预加载逐字节一致，因此“首帧可玩”不靠降低内容量换取。

同时输出：

- `first_playable_ms`：首帧可玩时间（门打开的最早帧边界）
- `critical_path_ms`：理论下界，用于判断调度浪费
- `concurrency,first_playable_ms,peak_entry_kb,peak_total_kb,total_ms` 表：
  峰值内存与并发数的关系

```bash
make test
```
