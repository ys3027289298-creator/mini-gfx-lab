# savemig

存档版本迁移链：旧档逐级迁移到当前版本（v1 → v2 → v3 → v4），每级是独立的结构体到结构体变换，可单独测试。

- 文件头：`magic / version / payload_len / crc32`，CRC 覆盖定长部分 + 变长 blob，校验和不符直接拒绝（`LOAD_ERR_CHECKSUM`），不做任何迁移。
- 迁移链注册表 `Migrator.step[v-1]` 存 v → v+1 的迁移函数；缺级报 `LOAD_ERR_MISSING_STEP`，不猜、不跳级。
- 每级迁移对载荷逐字节只复制 O(1) 次，总耗时与档大小成线性（自检里用字节计数断言严格线性，无二次项）。
- 自检验证：v1 老档走完整链迁移的结果，与直接写入的 v4 新档逐字段一致。

`make test` 跑自检。
