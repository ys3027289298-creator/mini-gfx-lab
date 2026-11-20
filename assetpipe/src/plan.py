"""构建计划（plan）的加载与静态校验。

plan 描述：目标平台 × 画质档位、画质预设、纹理/音频/图集清单、体积预算。
校验只检查 JSON 结构与配置语义；源文件是否存在由 builder 阶段报错。
"""
import json
import os

from pipecommon import ConfigError

KNOWN_PLATFORMS = ("pc", "mobile", "web")
KNOWN_QUALITIES = ("low", "medium", "high")
TEXTURE_EXTS = (".png",)
AUDIO_EXTS = (".wav",)
IMAGE_EXTS = (".png",)

_DEFAULT_PRESETS = {
    "low":    {"maxTexture": 512,  "textureFormat": "G",    "compressLevel": 9,
               "audioRate": 22050, "audioBits": 16, "audioChannels": 1},
    "medium": {"maxTexture": 1024, "textureFormat": "RGB",  "compressLevel": 9,
               "audioRate": 44100, "audioBits": 16, "audioChannels": 2},
    "high":   {"maxTexture": 2048, "textureFormat": "RGBA", "compressLevel": 9,
               "audioRate": 48000, "audioBits": 24, "audioChannels": 2},
}
_PRESET_KEYS = ("maxTexture", "textureFormat", "compressLevel",
                "audioRate", "audioBits", "audioChannels")
_VALID_FORMATS = ("G", "RGB", "RGBA")


def _require(cond, msg):
    if not cond:
        raise ConfigError(msg)


def load_plan(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
    except FileNotFoundError:
        raise ConfigError("计划文件不存在: %s" % path)
    except json.JSONDecodeError as e:
        raise ConfigError("计划文件 JSON 解析失败 %s: %s" % (path, e))
    if not isinstance(raw, dict):
        raise ConfigError("计划文件顶层必须是对象: %s" % path)

    source_root = raw.get("sourceRoot")
    _require(isinstance(source_root, str) and source_root,
             "sourceRoot 必须是非空字符串")

    presets = _validate_presets(raw.get("presets", {}))
    targets = _validate_targets(raw.get("targets"), presets)
    textures = _validate_asset_list(raw.get("textures", []), "textures",
                                    TEXTURE_EXTS)
    audios = _validate_asset_list(raw.get("audios", []), "audios",
                                  AUDIO_EXTS)
    atlases = _validate_atlases(raw.get("atlases", []))
    budgets = _validate_budgets(raw.get("budgets", {}))

    # 图集成员必须在 textures 中声明过（同一来源体系）
    tex_names = {t["name"] for t in textures}
    for a in atlases:
        for member in a["members"]:
            _require(member in tex_names,
                     "图集 %s 引用了未在 textures 声明的成员 %s"
                     % (a["name"], member))

    return {
        "plan_path": os.path.abspath(path),
        "source_root": source_root,
        "targets": targets,
        "presets": presets,
        "textures": textures,
        "audios": audios,
        "atlases": atlases,
        "budgets": budgets,
    }


def _validate_presets(custom):
    _require(isinstance(custom, dict), "presets 必须是对象")
    presets = {q: dict(p) for q, p in _DEFAULT_PRESETS.items()}
    for q, overrides in custom.items():
        _require(q in KNOWN_QUALITIES,
                 "presets 中出现未知画质档位 %r（可选 %s）"
                 % (q, "/".join(KNOWN_QUALITIES)))
        _require(isinstance(overrides, dict),
                 "presets.%s 必须是对象" % q)
        for k, v in overrides.items():
            _require(k in _PRESET_KEYS,
                     "presets.%s 含未知键 %r" % (q, k))
            presets[q][k] = v
    for q in KNOWN_QUALITIES:
        p = presets[q]
        _require(p["textureFormat"] in _VALID_FORMATS,
                 "presets.%s.textureFormat 非法: %r"
                 % (q, p["textureFormat"]))
        _require(isinstance(p["maxTexture"], int) and p["maxTexture"] > 0,
                 "presets.%s.maxTexture 必须是正整数" % q)
        _require(isinstance(p["compressLevel"], int)
                 and 0 <= p["compressLevel"] <= 9,
                 "presets.%s.compressLevel 必须在 0..9" % q)
        _require(isinstance(p["audioRate"], int) and p["audioRate"] > 0,
                 "presets.%s.audioRate 必须是正整数" % q)
        _require(p["audioBits"] in (8, 16, 24, 32),
                 "presets.%s.audioBits 非法" % q)
        _require(p["audioChannels"] in (1, 2),
                 "presets.%s.audioChannels 仅支持 1 或 2" % q)
    return presets


def _validate_targets(raw_targets, presets):
    _require(isinstance(raw_targets, list) and raw_targets,
             "targets 必须是非空数组")
    targets = []
    seen = set()
    for i, t in enumerate(raw_targets):
        _require(isinstance(t, dict), "targets[%d] 必须是对象" % i)
        tid = t.get("id")
        _require(isinstance(tid, str) and tid,
                 "targets[%d].id 必须是非空字符串" % i)
        _require(tid not in seen, "target id 重复: %s" % tid)
        seen.add(tid)
        platform = t.get("platform")
        quality = t.get("quality")
        _require(platform in KNOWN_PLATFORMS,
                 "target %s 的 platform %r 非法（可选 %s）"
                 % (tid, platform, "/".join(KNOWN_PLATFORMS)))
        _require(quality in KNOWN_QUALITIES,
                 "target %s 的 quality %r 非法（可选 %s）"
                 % (tid, quality, "/".join(KNOWN_QUALITIES)))
        targets.append({"id": tid, "platform": platform, "quality": quality})
    return targets


def _validate_asset_list(raw_list, key, exts):
    _require(isinstance(raw_list, list), "%s 必须是数组" % key)
    out = []
    seen = set()
    for i, item in enumerate(raw_list):
        _require(isinstance(item, dict), "%s[%d] 必须是对象" % (key, i))
        name = item.get("name")
        src = item.get("src")
        _require(isinstance(name, str) and name,
                 "%s[%d].name 必须是非空字符串" % (key, i))
        _require(name not in seen, "%s 中 name 重复: %s" % (key, name))
        seen.add(name)
        _require(isinstance(src, str) and src,
                 "%s[%d].src 必须是非空字符串" % (key, i))
        ext = os.path.splitext(src)[1].lower()
        _require(ext in exts,
                 "%s %s 的文件 %r 格式不受支持（扩展名 %s，仅允许 %s）"
                 % (key, name, src, ext or "（无）", "/".join(exts)))
        out.append({"name": name, "src": src})
    return out


def _validate_atlases(raw_atlases):
    _require(isinstance(raw_atlases, list), "atlases 必须是数组")
    out = []
    seen = set()
    for i, a in enumerate(raw_atlases):
        _require(isinstance(a, dict), "atlases[%d] 必须是对象" % i)
        name = a.get("name")
        _require(isinstance(name, str) and name,
                 "atlases[%d].name 必须是非空字符串" % i)
        _require(name not in seen, "atlases 中 name 重复: %s" % name)
        seen.add(name)
        members = a.get("members")
        _require(isinstance(members, list) and members,
                 "图集 %s 的 members 必须是非空数组" % name)
        for m in members:
            _require(isinstance(m, str),
                     "图集 %s 的成员名必须是字符串" % name)
        width = a.get("width", 1024)
        height = a.get("height", 1024)
        _require(isinstance(width, int) and width > 0,
                 "图集 %s 的 width 必须是正整数" % name)
        _require(isinstance(height, int) and height > 0,
                 "图集 %s 的 height 必须是正整数" % name)
        out.append({"name": name, "members": list(members),
                    "width": width, "height": height})
    return out


def _validate_budgets(raw_budgets):
    _require(isinstance(raw_budgets, dict), "budgets 必须是对象")
    budgets = {}
    for tid, b in raw_budgets.items():
        _require(isinstance(b, dict), "budgets.%s 必须是对象" % tid)
        entry = {}
        for key in ("textures", "audios", "atlases", "total"):
            v = b.get(key)
            if v is None:
                continue
            _require(isinstance(v, int) and v >= 0,
                     "budgets.%s.%s 必须是非负整数（字节）" % (tid, key))
            entry[key] = v
        budgets[tid] = entry
    return budgets
