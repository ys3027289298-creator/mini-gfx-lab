# particle

固定容量粒子池，**运行期零堆分配**：所有状态预分配在 `World` 内，
生成/死亡 O(1)，容量满时按旋转游标 round-robin 淘汰而不是扩容。

## 设计

- `pool[]` 槽位数组 + 槽内空闲链表 `free_next[]`：spawn 取表头、死亡插回表头，均为 O(1)
- 容量满（`free_head == -1`）时，`evict_cursor` 旋转游标淘汰最久未轮转的槽，
  计入 `evictions`；容量永不增长，spawn 永不失败
- 优先级排序用的临时数组 `order[]` 也在 `World` 内预分配
- 容量是编译期常量，`-DMAX_PARTICLES=N`（默认 10000）可覆盖；每粒子恒定 36 字节

旧实现用线性扫描找空槽，spawn 代价随存活数漂移，是帧时间抖动的来源；
对照基准保留在 `src/oldbench.c`。

## 构建与验证

```bash
make test       # 功能自测：优先级配额、满池淘汰不变量、死亡回收
make bench      # 零分配审计 + 逐帧计时（ld --wrap 拦截整个分配函数族）
make oldbench   # 旧线性扫描策略的帧时间对照
make audit-obj  # nm -u 符号审计：对象文件不引用任何分配器符号
```

容量线性扫描（footprint 与容量严格线性、各容量下分配计数均为 0）：

```bash
for c in 2500 5000 10000 20000 40000; do make clean && make bench CAP=$c && ./bench; done
```

## 验证结论（本机 WSL2, gcc 13 -O2）

- **运行期 malloc 计数为 0**：`--wrap` 拦截 malloc/calloc/realloc/aligned_alloc/
  posix_memalign/valloc/memalign/pvalloc/free，900 帧测量段
  `alloc_calls=0 realloc_calls=0 free_calls=0 bytes=0`；内置 canary 证明拦截器存活
- **符号审计**：`nm -u particle.o` 无任何未定义符号，实现完全自包含
- **峰值内存与容量线性**：2500→40000 五档，`sizeof(World)` 每粒子恒为 36.00 字节，
  容量翻倍 World 精确翻倍；运行中 `ru_maxrss` 无新增高水位、主缺页为 0
- **10000 粒子单帧 < 2ms**：最坏工况（每帧满池生成 10000、全程淘汰路径）
  单帧最大约 0.05–0.11ms，裕量 20x 以上
- **抖动改善**：同负载下旧策略 max≈24ms、stddev≈5ms；新策略 max≈0.07ms、stddev≈0.005ms
