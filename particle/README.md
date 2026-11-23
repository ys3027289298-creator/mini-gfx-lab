# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度。

```bash
make test
```

`make bench` 跑图像管线基准：共享缓冲 vs 私有拷贝，输出峰值内存/耗时/零拷贝依据。
