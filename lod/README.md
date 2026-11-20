# lod

屏幕误差 LOD 选择 + 迟滞切换，支持变体落盘加载：按索引读入各级网格元数据、
校验面数与包围盒，失败级退回最近的可用级而不是整页失败。

## 构建模式

| 宏 | 行为 |
|---|---|
| `-DLOD_ENABLE_LOAD`（默认） | 带落盘加载、结构校验、逐级降级；选级数学与原实现完全相同 |
| `-ULOD_ENABLE_LOAD` | 不编译任何加载代码，行为与原始运行时现算版本逐字节一致 |

`make test` 会把无加载构建的输出与 git HEAD 中原始版本的实际输出做 `cmp`，
字节不同即失败。

## 磁盘格式（小端、定长 196 字节）

```
header 52B: magic "LOD1" | version=1 | levels=4 | reserved=0
            f32 radius, tau | f32 bbox_min[3], bbox_max[3] | u32 header_crc
level  36B ×4: u32 face_count | f32 error | f32 bbox_min[3], bbox_max[3]
               | u32 record_crc
```

- 全部多字节字段经显式 LE 编解码，不依赖主机字节序；CRC 为 CRC32/IEEE 802.3。
- 校验顺序：先语义后校验和。语义错误能定位到具体级与字段
  （如 `level[2].face_count`、`level[1].bbox_max`）；语义合法但比特翻转的
  损坏由 checksum 检出。
- 逐级失败只标记 `valid[i]=0` 并继续校验其余级；选级时向更低的可用级回落。
  头部整体无效（魔数/版本/头校验和等）或 0 级不可用时返回
  `LOD_E_NO_FALLBACK`，没有可回落对象；此时所有级都标记为不可信。
- `lod_load` 返回首个错误码，`error_level`/`error_field` 给出定位
  （头部错误 `error_level=-1`，如 `header.magic`；逐级错误如
  `level[2].face_count`）。头部可用时即使有逐级错误也保留真实的
  `radius/tau`，保证降级资产的迟滞行为不退化；头部坏时两者不赋值。

## 测试

```bash
make test
```

覆盖：干净加载的位级一致性、40 帧选级与现算版本逐帧一致（含切换次数）、
魔数/版本/头校验和/截断/超长/面数单调性/包围盒/NaN-Inf/记录校验和等
损坏定位、逐级回落、0 级无回落，以及无加载路径的逐字节回归。
另有 2 万次随机位翻转 fuzz（ASan/UBSan 下）保证任何损坏都不崩溃且
选级永远不会返回死级。
