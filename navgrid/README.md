# mini-navgrid

8 方向 A*、对角禁止切角、视线可达平滑、增量重规划实验平台。

```bash
make test
```

诊断构建：`make diag-test`。验收失败时只丢本模块绘制，证据为路径/网格的 FNV-1a 64 产物哈希（stderr），退出码 2。`MINI_GFX_FAULT=navgrid ./diag-bin` 可注入故障演练。
