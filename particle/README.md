# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度。

```bash
make test
```

诊断构建：`make diag-test`。验收失败时只丢本模块绘制，证据为逐帧数值表（stderr），退出码 2。`MINI_GFX_FAULT=particle ./diag-bin` 可注入故障演练。
