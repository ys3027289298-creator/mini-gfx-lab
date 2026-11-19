# tilemap

分层瓦片地图、4 邻接自动拼接、区域剔除，以及可持久化的编辑命令日志。

`make test` 跑自检。

## 编辑与撤销

- `editor_set(&ed, layer, x, y, id)` 记录 `{层, 格, 旧值, 新值}` 并压入撤销栈。
- `editor_undo` / `editor_redo` 按后进先出恢复；撤销后再编辑会丢弃重做分支。
- 撤销栈上界为 `UNDO_CAP`（8 条），溢出时淘汰最旧记录。
- 所有 API 返回 `EdStatus`：越界为 `ED_ERR_RANGE`，空栈为 `ED_ERR_UNDO_EMPTY` /
  `ED_ERR_REDO_EMPTY`，写入相同值返回 `ED_NO_CHANGE`（不入栈、不记日志），
  栈与地图不一致返回 `ED_ERR_STATE`。

## 日志与重放

- `editor_save_log` 写出头部（尺寸）、每条变更以及全图 FNV-1a 校验和。
- 撤销/重做本身也记入日志，因此任意时刻落盘后重放，结果都与当前地图逐格一致。
- `replay_file` 只接受边界内、且当前值与记录旧值吻合的条目，最后校验校验和；
  格式错误、旧值不符、校验和不匹配、校验和之后仍有数据均返回 `ED_ERR_REPLAY`，
  文件无法打开返回 `ED_ERR_IO`。
