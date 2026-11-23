# tilemap

`make test` 跑自检。

诊断构建：`make diag-test`。验收失败时只丢本模块绘制，证据为逐项 check 的运行日志（stderr），退出码 2。`MINI_GFX_FAULT=tilemap ./diag-bin` 可注入故障演练。
