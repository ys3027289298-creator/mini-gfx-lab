# assetpipe

确定性资源构建流水线。纯 Python 标准库实现，无第三方依赖；`make test` 跑自检（40 项）。

## 解决什么问题

资源构建全手工时，纹理压不压、图集怎么打、音频转不转码各凭习惯，包体没人管。
本模块把构建收敛到一份 plan + 一条命令，保证：

1. **按平台 × 画质档位出变体**：每档有独立预设（纹理最长边、色彩模式、
   音频采样率/位深/声道），可在 plan 中覆盖；
2. **来源与体积可追溯**：`manifest.json` 记录每个产物的源文件相对路径、
   源/产物 sha256 与字节数；
3. **预算超支报出是哪一档**：按 target 配置 textures/audios/atlases/total
   字节预算，超支信息含 target id、类别、实际/上限/超出量；
4. **字节级可复现**：同一份源资源两次产出完全一致（固定 PNG 过滤类型与
   zlib 参数、整数域缩放/重采样、排序确定的图集装箱与 JSON 输出）；
5. **缺失 / 格式不支持即报错且带文件名**；
6. **不改源目录**：所有写操作只落在输出目录，路径越界（`../`）直接拒绝，
   输出目录禁止位于源目录之内。

## 用法

```
python3 -I src/assetpipe.py --plan plan.json --source assets/ --out build/ [--only target-id ...]
```

退出码：`0` 成功；`1` 资源缺失/格式/预算错误；`2` plan 配置错误。

## plan 结构

```json
{
  "sourceRoot": "assets",
  "targets": [
    { "id": "pc-high", "platform": "pc", "quality": "high" },
    { "id": "mobile-low", "platform": "mobile", "quality": "low" }
  ],
  "presets": { "low": { "maxTexture": 512, "textureFormat": "G" } },
  "textures": [{ "name": "hero", "src": "tex/hero.png" }],
  "audios":   [{ "name": "bgm",  "src": "audio/bgm.wav" }],
  "atlases":  [{ "name": "ui", "members": ["btn_ok", "btn_no"],
                 "width": 1024, "height": 1024 }],
  "budgets":  { "mobile-low": { "textures": 20000, "total": 1048576 } }
}
```

- `platform`: `pc` / `mobile` / `web`；`quality`: `low` / `medium` / `high`
- 预设默认值见 `src/plan.py` 的 `_DEFAULT_PRESETS`，`presets` 只写要覆盖的键
- 支持格式：纹理 8 位非隔行 PNG（灰度/RGB/RGBA），音频未压缩 PCM WAV
  （8/16/24/32 位）；其余格式在 plan 校验或解码阶段报错

## 模块划分

| 文件 | 职责 |
|---|---|
| `src/plan.py` | plan 加载与静态校验 |
| `src/pngcodec.py` | 确定性 PNG 读写、灰度/RGBA 转换、整数盒缩放 |
| `src/wavcodec.py` | PCM WAV 读写、整数重采样与降混 |
| `src/atlas.py` | 货架装箱与图集合成（与 fontatlas 同思路） |
| `src/builder.py` | 流水线主体、manifest、预算核对、源目录保护 |
| `src/pipecommon.py` | 异常类型 |
| `tests/selftest.py` | 编解码、装箱、plan、端到端、CLI 退出码自检 |

## 可复现性是怎么保证的

- PNG 写出只用过滤类型 0、固定 zlib `compressLevel/memLevel/wbits`，块顺序
  固定 IHDR/IDAT/IEND，无时间戳；
- 缩放用整数盒平均、重采样用 16 位定点线性插值、灰度用整数 Rec.601 权重，
  全程不依赖浮点环境；
- 图集装箱先按（高、宽、名）确定排序；JSON 全部 `sort_keys` 输出、LF 换行；
- WAV 只写标准 RIFF 头与 PCM data，不含 LIST 等元数据块。
