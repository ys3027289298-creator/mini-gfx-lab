# fontatlas

`make test` 跑自检。

诊断构建：`make diag-test`。验收失败时只丢本模块绘制，证据为可复制的复现命令（stderr），退出码 2。`MINI_GFX_FAULT=fontatlas ./diag-bin` 可注入故障演练。
