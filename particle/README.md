# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度。

两种存储布局锁步运行：

- **AoS（基准）**：`Particle pool[MAX]`，原实现保留作参照；
- **SoA**：每个字段一条数组。

字段表 `PARTICLE_FIELDS`（X-macro）是唯一事实来源：AoS 结构体、SoA
结构体、逐字段对拍器都由它生成。增删字段只改这一处；若某个 `update`
忘了同步更新新字段，当帧对拍即报 `MISMATCH` 并以非零退出。

`update` 热循环接入了组相联缓存模型（32KB / 8 路 / 64B 行 / LRU），
输出两种布局的模拟命中率。

```bash
make test   # 锁步对拍 + 缓存模拟 + desync-selftest（故意漏更新字段，验证能检出）
```
