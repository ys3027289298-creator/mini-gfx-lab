"""流水线主体：校验来源 -> 按目标构建变体 -> 记录 manifest -> 预算核对。

不变量：
1. 源目录只读：任何写操作都落在 out 目录，源路径越界直接报错；
2. 可复现：同一 plan + 同一源文件，任意机器两次构建字节一致；
3. 错误带文件名：缺失 / 不支持的格式逐条收集后统一报出。
"""
import hashlib
import json
import os

import atlas as atlas_mod
import pngcodec
import wavcodec
from pipecommon import (AtlasOverflow, ConfigError, MissingSource,
                        UnsupportedFormat)

MANIFEST_NAME = "manifest.json"


def _resolve_source(source_dir, rel):
    """拼源文件绝对路径并拒绝任何跳出 source_dir 的写法。"""
    source_dir = os.path.abspath(source_dir)
    joined = os.path.normpath(os.path.join(source_dir, rel))
    if joined != source_dir and not joined.startswith(source_dir + os.sep):
        raise ConfigError("源路径越界: %r 跳出源目录" % rel)
    return joined


def _ensure_within(base, path):
    base = os.path.abspath(base)
    path = os.path.normpath(os.path.abspath(path))
    if path != base and not path.startswith(base + os.sep):
        raise ConfigError("输出路径越界: %r" % path)
    return path


def _sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def _scaled_size(w, h, limit):
    """按最长边等比缩到 limit 以内；已经够小则原样返回。"""
    longest = max(w, h)
    if longest <= limit:
        return w, h
    scale = limit / longest
    # 先浮点取整再交给整数盒缩放，尺寸本身确定
    return max(1, round(w * scale)), max(1, round(h * scale))


def _apply_texture_format(img, fmt):
    if fmt == "G":
        return pngcodec.to_gray(img)
    if fmt == "RGB":
        img = pngcodec.to_rgba(img)
        # RGBA -> RGB：直接丢 alpha，逐像素整数拷贝
        w, h = img["w"], img["h"]
        src = img["pixels"]
        out = bytearray(w * h * 3)
        for i in range(w * h):
            out[3 * i:3 * i + 3] = src[4 * i:4 * i + 3]
        return {"w": w, "h": h, "mode": "RGB", "ch": 3, "pixels": bytes(out)}
    return pngcodec.to_rgba(img)


class _TargetBuilder:
    def __init__(self, plan, source_dir, out_dir, target):
        self.plan = plan
        self.source_dir = source_dir
        self.out_dir = out_dir
        self.target = target
        self.preset = plan["presets"][target["quality"]]
        self.errors = []
        # 已解码纹理缓存：name -> rgba 图像（同一目标内复用给图集）
        self._images = {}
        self.record = {"target": target["id"], "platform": target["platform"],
                       "quality": target["quality"], "preset": self.preset,
                       "textures": [], "audios": [], "atlases": [],
                       "budgets": {}, "budgetErrors": []}

    def _source_path(self, rel):
        return _resolve_source(self.source_dir, rel)

    def _load_image(self, name, rel):
        if name in self._images:
            return self._images[name]
        path = self._source_path(rel)
        if not os.path.isfile(path):
            self.errors.append(MissingSource(
                "目标 %s 纹理缺失: %s（%s）" % (self.target["id"], name, path)))
            return None
        try:
            img = pngcodec.to_rgba(pngcodec.read_png(path))
        except UnsupportedFormat as e:
            self.errors.append(e)
            return None
        self._images[name] = img
        return img

    def build_textures(self):
        p = self.preset
        for tex in self.plan["textures"]:
            img = self._load_image(tex["name"], tex["src"])
            if img is None:
                continue
            dst_w, dst_h = _scaled_size(img["w"], img["h"], p["maxTexture"])
            if (dst_w, dst_h) != (img["w"], img["h"]):
                img = pngcodec.resize_box(img, dst_w, dst_h)
            img = _apply_texture_format(img, p["textureFormat"])

            rel_out = os.path.join(
                self.target["id"], "textures", tex["name"] + ".png")
            out_path = _ensure_within(
                self.out_dir, os.path.join(self.out_dir, rel_out))
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            pngcodec.write_png(out_path, img["w"], img["h"], img["mode"],
                               img["pixels"], p["compressLevel"])
            self.record["textures"].append(self._file_record(
                tex["name"], rel_out, self._source_path(tex["src"])))

    def build_audios(self):
        p = self.preset
        for aud in self.plan["audios"]:
            src_path = self._source_path(aud["src"])
            if not os.path.isfile(src_path):
                self.errors.append(MissingSource(
                    "目标 %s 音频缺失: %s（%s）"
                    % (self.target["id"], aud["name"], src_path)))
                continue
            try:
                wav = wavcodec.read_wav(src_path)
            except UnsupportedFormat as e:
                self.errors.append(e)
                continue
            wav = wavcodec.downmix(wav, p["audioChannels"])
            wav = wavcodec.resample(wav, p["audioRate"])

            rel_out = os.path.join(
                self.target["id"], "audios", aud["name"] + ".wav")
            out_path = _ensure_within(
                self.out_dir, os.path.join(self.out_dir, rel_out))
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            wavcodec.write_wav(out_path, wav["rate"], wav["channels"],
                               p["audioBits"], wav["samples"])
            self.record["audios"].append(self._file_record(
                aud["name"], rel_out, src_path))

    def build_atlases(self):
        # 先确保所有成员纹理已解码（缺失会被记入 errors 并跳过该图集）
        tex_by_name = {t["name"]: t for t in self.plan["textures"]}
        for spec in self.plan["atlases"]:
            sources = []
            ok = True
            for member in spec["members"]:
                tex = tex_by_name[member]
                img = self._load_image(member, tex["src"])
                if img is None:
                    ok = False
                    continue
                sources.append((member, img))
            if not ok:
                continue
            try:
                canvas, rects = atlas_mod.build_atlas(
                    sources, spec["width"], spec["height"])
            except AtlasOverflow as e:
                self.errors.append(AtlasOverflow(
                    "目标 %s 图集 %s 装箱失败: %s"
                    % (self.target["id"], spec["name"], e)))
                continue
            canvas = _apply_texture_format(canvas, self.preset["textureFormat"])

            rel_png = os.path.join(
                self.target["id"], "atlases", spec["name"] + ".png")
            rel_json = os.path.join(
                self.target["id"], "atlases", spec["name"] + ".json")
            png_path = _ensure_within(
                self.out_dir, os.path.join(self.out_dir, rel_png))
            json_path = _ensure_within(
                self.out_dir, os.path.join(self.out_dir, rel_json))
            os.makedirs(os.path.dirname(png_path), exist_ok=True)
            pngcodec.write_png(png_path, canvas["w"], canvas["h"],
                               canvas["mode"], canvas["pixels"],
                               self.preset["compressLevel"])
            # 矩形按名字排序写入，JSON key 顺序确定
            ordered_rects = {k: rects[k] for k in sorted(rects)}
            with open(json_path, "w", encoding="utf-8", newline="\n") as f:
                json.dump({"name": spec["name"],
                           "width": canvas["w"], "height": canvas["h"],
                           "frames": ordered_rects},
                          f, ensure_ascii=False, sort_keys=True, indent=2)
                f.write("\n")
            entry = self._file_record(spec["name"], rel_png, None)
            entry["layout"] = rel_json
            entry["layoutSize"] = os.path.getsize(json_path)
            entry["members"] = list(spec["members"])
            self.record["atlases"].append(entry)

    def _file_record(self, name, rel_out, src_path):
        out_path = os.path.join(self.out_dir, rel_out)
        rec = {"name": name, "path": rel_out.replace(os.sep, "/"),
               "size": os.path.getsize(out_path),
               "sha256": _sha256_file(out_path)}
        if src_path is not None:
            rec["source"] = os.path.relpath(src_path, self.source_dir)
            rec["sourceSize"] = os.path.getsize(src_path)
            rec["sourceSha256"] = _sha256_file(src_path)
        return rec

    def check_budgets(self):
        b = self.plan["budgets"].get(self.target["id"], {})
        totals = {
            "textures": sum(r["size"] for r in self.record["textures"]),
            "audios": sum(r["size"] for r in self.record["audios"]),
            "atlases": (sum(r["size"] for r in self.record["atlases"])
                        + sum(r.get("layoutSize", 0)
                              for r in self.record["atlases"])),
        }
        totals["total"] = (totals["textures"] + totals["audios"]
                           + totals["atlases"])
        self.record["budgets"]["actual"] = totals
        self.record["budgets"]["limit"] = b
        for key, limit in b.items():
            actual = totals[key]
            entry = {"category": key, "limit": limit, "actual": actual,
                     "over": max(0, actual - limit)}
            if actual > limit:
                self.record["budgetErrors"].append(entry)
        return self.record["budgetErrors"]


def _dump_manifest(out_dir, manifest):
    path = os.path.join(out_dir, MANIFEST_NAME)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(manifest, f, ensure_ascii=False, sort_keys=True, indent=2)
        f.write("\n")


def run_pipeline(plan, source_dir, out_dir, only_targets=None):
    """执行构建，返回 (manifest, 错误字符串列表)。manifest 始终落盘。"""
    out_dir = os.path.abspath(out_dir)
    source_dir = os.path.abspath(source_dir)
    if os.path.commonpath([out_dir, source_dir]) == source_dir and out_dir != source_dir:
        raise ConfigError("输出目录不允许位于源目录之内: %s" % out_dir)
    os.makedirs(out_dir, exist_ok=True)

    targets = plan["targets"]
    if only_targets:
        wanted = set(only_targets)
        unknown = wanted - {t["id"] for t in targets}
        if unknown:
            raise ConfigError("命令行指定了不存在的目标: %s"
                              % ", ".join(sorted(unknown)))
        targets = [t for t in targets if t["id"] in wanted]

    manifest = {"tool": "assetpipe", "targets": []}
    all_errors = []
    for target in targets:
        tb = _TargetBuilder(plan, source_dir, out_dir, target)
        tb.build_textures()
        tb.build_audios()
        tb.build_atlases()
        tb.check_budgets()
        manifest["targets"].append(tb.record)
        all_errors.extend(str(e) for e in tb.errors)
        for be in tb.record["budgetErrors"]:
            all_errors.append(
                "目标 %s 预算超支 [%s]: 实际 %d 字节 / 上限 %d 字节（超 %d）"
                % (target["id"], be["category"], be["actual"],
                   be["limit"], be["over"]))

    _dump_manifest(out_dir, manifest)
    return manifest, all_errors
