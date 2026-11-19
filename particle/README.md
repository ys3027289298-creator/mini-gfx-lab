# particle

固定容量粒子池 + 每帧发射配额 + 发射器优先级调度。

状态序列化：`world_save`/`world_load` 把发射器配额、xorshift64* 随机流位置和
存活粒子（含池槽位）落成紧凑二进制（魔数 + 版本号 + FNV-1a 校验和，体积 ≤ 裸状态
1.3 倍）。版本不匹配返回 `SAVE_ERR_VERSION`，损坏/截断输入返回 `SAVE_ERR_CORRUPT`，
恢复后继续模拟与不中断运行逐粒子一致。

```bash
make test
```
