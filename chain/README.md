# chain

把 `particle` `tilemap` `navgrid` `spatial` 四个阶段合成一帧。画面分成四条带，
任一阶段失败只丢自己的那条带，其余三条保持不变。

```bash
make test        # 每条带各自失败时只降级自己；零分配断言
make acceptance  # 套件 + 覆盖报告 + 干净帧可重现
make && ./render-chain                          # 正常一帧，degraded=0
DIAG_FAULT=spatial:clip_violation ./render-chain    # 仅 spatial 带降级
DIAG_ENABLE=1 DIAG_FAULT=navgrid:no_path ./render-chain  # 结构化诊断到 stderr
```
