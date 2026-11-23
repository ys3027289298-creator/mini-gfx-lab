# imgshare

共享图像缓冲：一张图只分配一次，多个消费者拿到的是同一块像素的引用计数句柄，
不再各拷一份。写者通过 `imgs_make_writable()` 走 copy-on-write，拿到私有副本。

```bash
make test       # 单元测试（共享 / COW / intern / 非法接口）
make evidence   # 采集并校验四份证据
```

## API 摘要

| 调用 | 作用 | 非法输入 |
|---|---|---|
| `imgs_image_init/destroy` | 创建/销毁镜像容器 | NULL → `IMGS_EINVAL` |
| `imgs_create(img,w,h,fmt,key,name)` | 分配并登记（key≠0 时 intern） | 空指针/尺寸≤0/超限/坏格式/重复 key/容量满 → `NULL` |
| `imgs_open(img,key,&ref)` | 按 key 取共享句柄 | NULL/key=0 → `IMGS_EINVAL`；无此 key → `IMGS_ENOENT` |
| `imgs_acquire(img,ref)` | 再取一个共享句柄 | 坏句柄/不属于该镜像 → `NULL` |
| `imgs_make_writable(img,ref)` | 私有副本（COW） | 同上 → `NULL` |
| `imgs_release(img,ref)` | 释放一个句柄，归零即 free | 坏句柄/不属于该镜像 → `IMGS_EINVAL` |
| `imgs_pixel_fill(h,seed)` | 填充像素（仅当 `refs==1`） | 共享中的句柄 → `IMGS_EBUSY` |
| `imgs_pixel_hash(ref)` | FNV-1a64（含宽高格式） | 坏句柄 → `0` |

所有非法调用都会累加 `imgs_stats()->illegal_calls`。

## 四处接入（`src/demo.c`）

1. **峰值内存表** —— `emit_memtable()`，`peak_ratio = peak_bytes / input_bytes`，上限 2.0。
2. **拷贝次数日志** —— `emit_copylog()` / 每次 `legacy_copy()` 打印一行。
3. **像素哈希** —— `emit_pixelhash()`，四个消费者各算一次并与输入比对。
4. **复现命令** —— `emit_repro()`，`./imgshare-demo --share|--noshare`。

`--share` 是接入后的路径，`--noshare` 是每位消费者各自 `memcpy` 的旧路径。
两者像素哈希必须逐位一致；只有内存与拷贝证据应当不同。

## 证据（`make evidence`）

| 文件 | 内容 | 判定 |
|---|---|---|
| `evidence/peak.txt` | 两模式峰值内存 | share ≤ 2×input |
| `evidence/copies.txt` | 拷贝/共享次数 | share=0 拷贝，noshare>0 |
| `evidence/hashes.txt` | 四消费者像素哈希 | 两模式逐行相同 |
| `evidence/repro.txt` | 复现命令 | 重跑仍能产出证据 |

`scripts/check_evidence.sh` 全部通过才打印 `EVIDENCE_OK`；任一项缺失或不合规即
`EVIDENCE_FAILED` 并以非零退出。
