# imgbuf

共享图像缓冲：引用计数的 RGBA8 像素缓冲。

- `imgbuf_share` 零拷贝共享同一份像素；`imgbuf_copy_into` 是唯一拷贝路径并计入 `imgbuf_copy_count`
- `imgbuf_live_bytes` / `imgbuf_peak_bytes` 记账存活字节与峰值，`imgbuf_reset_peak` 以当前存活为基线
- 非法参数（空指针、非正尺寸、尺寸不匹配）返回错误并可通过 `imgbuf_last_error` 读取原因

`make test` 跑自检。

接入方（各自 `make bench` 出峰值内存与耗时数据）：`tilemap`、`fontatlas`、`particle`、`lod`。
