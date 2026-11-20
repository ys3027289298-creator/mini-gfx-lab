# renderdiag

渲染链统一诊断：粒子（`particle`)、图块（`tilemap`)、网格（`navgrid`)、声学（`spatial`) 四个模块的失败点归一到同一套错误码，按模块分段（位 8-15 为模块段，位 0-7 为模块内码）。

- `src/renderdiag.h` — 错误码 + 映射表 + `RD_REPORT` 宏，header-only，各模块 `-I../renderdiag/src` 引用。
- `check_coverage.sh` — 扫描四个模块的失败点（`return -1` / `return;` / `return 1;` / `return rc;` 哨兵返回），列出未被 `RD_REPORT` 或 `rd-waive:` 注释覆盖的失败点；同时校验模块引用的错误码都在映射表内。

约定：

- 模块渲染步骤返回 `RD_OK`(0) 表示成功；返回非零错误码时，帧合成器（见 `src/renderdiag.c` 的 `rd_run_frame`）只摘掉该模块、本帧继续渲染，不再整帧丢弃。
- 诊断只写 stderr，stdout 逐字节不变。
- 未定义 `RD_DIAG` 时 `RD_REPORT` 编译为空语句：无额外分配，画面与之前逐像素一致。开启方式：

```bash
make CFLAGS='-O2 -std=c11 -Wall -Wextra -DRD_DIAG=1'
```

```bash
make test   # 自测映射表 + 降级合成器 + 失败点覆盖检查
```
