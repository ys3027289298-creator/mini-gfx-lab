#!/usr/bin/env python3
"""assetpipe 自检：覆盖可复现、来源与体积记录、预算超支报错、
缺失/不支持格式报错带文件名、源目录只读、确定性编解码。

用法: python3 -I selftest.py
"""
import filecmp
import hashlib
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src"))

import atlas as atlas_mod
import pngcodec
import wavcodec
from pipecommon import (AtlasOverflow, ConfigError, MissingSource,
                        UnsupportedFormat)
import plan as plan_mod
import builder

FAILURES = 0


def check(name, ok):
    print("check_%s=%d" % (name, 1 if ok else 0))
    if not ok:
        global FAILURES
        FAILURES += 1


# ---------- 测试夹具生成 ----------

def make_png(path, w, h, mode, fill):
    ch = {"G": 1, "RGB": 3, "RGBA": 4}[mode]
    pixels = bytes((fill * ch)[:ch]) * (w * h)
    pngcodec.write_png(path, w, h, mode, pixels)


def make_wav(path, rate, channels, bits, n_frames, step=1000):
    samples = []
    for i in range(n_frames * channels):
        samples.append((i * step) % 20000 - 10000)
    wavcodec.write_wav(path, rate, channels, bits, samples)


def write_plan(path, source_root, budgets=None, broken=None):
    plan = {
        "sourceRoot": source_root,
        "targets": [
            {"id": "pc-high", "platform": "pc", "quality": "high"},
            {"id": "mobile-low", "platform": "mobile", "quality": "low"},
        ],
        "textures": [
            {"name": "hero", "src": "tex/hero.png"},
            {"name": "ui_ok", "src": "tex/ui_ok.png"},
            {"name": "ui_cancel", "src": "tex/ui_cancel.png"},
        ],
        "audios": [
            {"name": "bgm", "src": "audio/bgm.wav"},
        ],
        "atlases": [
            {"name": "ui", "members": ["ui_ok", "ui_cancel"],
             "width": 256, "height": 256},
        ],
        "budgets": budgets or {},
    }
    if broken == "bad_ext":
        plan["textures"].append({"name": "x", "src": "tex/x.BMP"})
    if broken == "unknown_quality":
        plan["targets"].append(
            {"id": "weird", "platform": "pc", "quality": "ultra"})
    with open(path, "w", encoding="utf-8") as f:
        json.dump(plan, f)


# ---------- 1. PNG 确定性与往返 ----------

def test_png(tmp):
    d = os.path.join(tmp, "png")
    os.makedirs(d)
    p1, p2 = os.path.join(d, "a.png"), os.path.join(d, "b.png")
    pixels = bytes(((i * 37) % 256 for i in range(4 * 6 * 4)))
    pngcodec.write_png(p1, 4, 6, "RGBA", pixels)
    pngcodec.write_png(p2, 4, 6, "RGBA", pixels)
    check("png_write_deterministic", filecmp.cmp(p1, p2, shallow=False))

    img = pngcodec.read_png(p1)
    check("png_roundtrip_dims", img["w"] == 4 and img["h"] == 6
          and img["mode"] == "RGBA")
    check("png_roundtrip_pixels", img["pixels"] == pixels)

    # 灰度 / RGB 模式也能读回
    gp = os.path.join(d, "g.png")
    gpix = bytes((i % 256 for i in range(8)))
    pngcodec.write_png(gp, 4, 2, "G", gpix)
    gimg = pngcodec.read_png(gp)
    check("png_gray_roundtrip", gimg["mode"] == "G" and gimg["pixels"] == gpix)

    # 损坏文件：错误消息带文件名
    bad = os.path.join(d, "bad.png")
    with open(bad, "wb") as f:
        f.write(b"not a png at all")
    try:
        pngcodec.read_png(bad)
        check("png_bad_rejected", False)
    except UnsupportedFormat as e:
        check("png_bad_rejected", "bad.png" in str(e))

    # 盒缩放确定性与尺寸
    big = pngcodec.new_canvas(8, 8, "RGB")
    big["pixels"] = bytes(((i * 13) % 256 for i in range(8 * 8 * 3)))
    s1 = pngcodec.resize_box(big, 4, 4)
    s2 = pngcodec.resize_box(big, 4, 4)
    check("resize_deterministic", s1["pixels"] == s2["pixels"]
          and s1["w"] == 4 and s1["h"] == 4)
    gray = pngcodec.to_gray(big)
    check("to_gray_shape", gray["mode"] == "G"
          and len(gray["pixels"]) == 64)
    rgba = pngcodec.to_rgba(big)
    check("to_rgba_shape", rgba["mode"] == "RGBA"
          and len(rgba["pixels"]) == 256)


# ---------- 2. WAV 确定性与往返 ----------

def test_wav(tmp):
    d = os.path.join(tmp, "wav")
    os.makedirs(d)
    p1, p2 = os.path.join(d, "a.wav"), os.path.join(d, "b.wav")
    samples = [((i * 1103) % 40000) - 20000 for i in range(100)]
    wavcodec.write_wav(p1, 48000, 2, 16, samples)
    wavcodec.write_wav(p2, 48000, 2, 16, samples)
    check("wav_write_deterministic", filecmp.cmp(p1, p2, shallow=False))

    w = wavcodec.read_wav(p1)
    check("wav_roundtrip", w["rate"] == 48000 and w["channels"] == 2
          and w["bits"] == 16 and w["samples"] == samples)

    mono = wavcodec.downmix(w, 1)
    check("wav_downmix", mono["channels"] == 1
          and len(mono["samples"]) == 50)
    rs1 = wavcodec.resample(w, 24000)
    rs2 = wavcodec.resample(w, 24000)
    check("wav_resample_deterministic",
          rs1["samples"] == rs2["samples"] and rs1["rate"] == 24000
          and len(rs1["samples"]) == len(rs2["samples"]))

    # 非 PCM（fmt 标签 3=IEEE float）应被拒绝且带文件名
    bad = os.path.join(d, "float.wav")
    fmt_body = struct.pack("<HHIIHH", 3, 1, 48000, 96000, 4, 32)
    with open(bad, "wb") as f:
        f.write(b"RIFF" + struct.pack("<I", 36) + b"WAVE")
        f.write(b"fmt " + struct.pack("<I", 16) + fmt_body)
        f.write(b"data" + struct.pack("<I", 0))
    try:
        wavcodec.read_wav(bad)
        check("wav_bad_rejected", False)
    except UnsupportedFormat as e:
        check("wav_bad_rejected", "float.wav" in str(e))


# ---------- 3. 图集装箱 ----------

def test_atlas():
    items = [("a", 32, 32), ("b", 64, 16), ("c", 16, 48)]
    rects = atlas_mod.pack(items, 128, 128, padding=1)
    names = [r[0] for r in rects]
    check("atlas_all_placed", sorted(names) == ["a", "b", "c"])
    # 两两不重叠
    overlap = False
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            _, ax, ay, aw, ah = rects[i]
            _, bx, by, bw, bh = rects[j]
            if not (ax + aw <= bx or bx + bw <= ax
                    or ay + ah <= by or by + bh <= ay):
                overlap = True
    check("atlas_no_overlap", not overlap)
    try:
        atlas_mod.pack([("big", 300, 300)], 128, 128, 1)
        check("atlas_overflow_rejected", False)
    except AtlasOverflow:
        check("atlas_overflow_rejected", True)


# ---------- 4. plan 校验 ----------

def test_plan(tmp):
    plan_path = os.path.join(tmp, "plan-bad-ext.json")
    write_plan(plan_path, "assets", broken="bad_ext")
    try:
        plan_mod.load_plan(plan_path)
        check("plan_bad_ext_rejected", False)
    except ConfigError as e:
        # 扩展名被归一化为小写，消息须带具体文件名
        check("plan_bad_ext_rejected", "x.BMP" in str(e) or "x.bmp" in str(e))
    plan_path2 = os.path.join(tmp, "plan-bad-q.json")
    write_plan(plan_path2, "assets", broken="unknown_quality")
    try:
        plan_mod.load_plan(plan_path2)
        check("plan_bad_quality_rejected", False)
    except ConfigError as e:
        check("plan_bad_quality_rejected", "ultra" in str(e))


# ---------- 5. 端到端流水线 ----------

def _setup_sources(src):
    os.makedirs(os.path.join(src, "tex"))
    os.makedirs(os.path.join(src, "audio"))
    make_png(os.path.join(src, "tex", "hero.png"), 64, 64, "RGBA",
             (200, 100, 50, 255))
    make_png(os.path.join(src, "tex", "ui_ok.png"), 32, 16, "RGB",
             (10, 20, 30, 0))
    make_png(os.path.join(src, "tex", "ui_cancel.png"), 24, 24, "RGBA",
             (210, 30, 30, 255))
    make_wav(os.path.join(src, "audio", "bgm.wav"), 48000, 2, 16, 200)


def _snapshot(root):
    snap = {}
    for dirpath, _, files in os.walk(root):
        for fn in files:
            p = os.path.join(dirpath, fn)
            with open(p, "rb") as f:
                snap[os.path.relpath(p, root)] = hashlib.sha256(
                    f.read()).hexdigest()
    return snap


def test_pipeline(tmp):
    src = os.path.join(tmp, "assets")
    _setup_sources(src)
    before = _snapshot(src)

    plan_path = os.path.join(tmp, "plan.json")
    write_plan(plan_path, src)
    cfg = plan_mod.load_plan(plan_path)

    out1 = os.path.join(tmp, "build1")
    manifest1, errors1 = builder.run_pipeline(cfg, src, out1)
    check("pipeline_no_errors", errors1 == [])

    # 源目录构建前后字节一致
    check("source_untouched", _snapshot(src) == before)

    # 第二次构建：逐字节一致
    out2 = os.path.join(tmp, "build2")
    manifest2, errors2 = builder.run_pipeline(cfg, src, out2)
    check("pipeline_repeat_no_errors", errors2 == [])
    cmp_ok = True
    for dirpath, _, files in os.walk(out1):
        for fn in files:
            p1 = os.path.join(dirpath, fn)
            rel = os.path.relpath(p1, out1)
            p2 = os.path.join(out2, rel)
            if not os.path.isfile(p2) or not filecmp.cmp(p1, p2, shallow=False):
                cmp_ok = False
    check("reproducible_byte_identical", cmp_ok)

    # manifest 记录来源、源体积、产物体积与 sha256
    t = next(x for x in manifest1["targets"] if x["target"] == "pc-high")
    hero = next(x for x in t["textures"] if x["name"] == "hero")
    check("manifest_provenance",
          hero["source"].replace(os.sep, "/") == "tex/hero.png"
          and "sourceSha256" in hero and "sha256" in hero)
    check("manifest_sizes",
          hero["size"] > 0 and hero["sourceSize"] > 0)
    check("manifest_atlas", len(t["atlases"]) == 1
          and t["atlases"][0]["layout"].endswith(".json"))

    # 档位差异：low 档纹理应转灰度且受 maxTexture 限制
    low = next(x for x in manifest1["targets"]
               if x["target"] == "mobile-low")
    low_hero_path = os.path.join(out1, low["textures"][0]["path"])
    low_img = pngcodec.read_png(low_hero_path)
    check("quality_tier_gray", low_img["mode"] == "G")
    low_bgm = os.path.join(out1, low["audios"][0]["path"])
    low_wav = wavcodec.read_wav(low_bgm)
    check("quality_tier_audio",
          low_wav["rate"] == 22050 and low_wav["channels"] == 1)

    # 预算超支：报出是哪一档、哪一类、超多少
    tight = {"mobile-low": {"textures": 1}}
    plan_b = os.path.join(tmp, "plan-budget.json")
    write_plan(plan_b, src, budgets=tight)
    cfg_b = plan_mod.load_plan(plan_b)
    out_b = os.path.join(tmp, "build-budget")
    mb, errs_b = builder.run_pipeline(cfg_b, src, out_b)
    joined = "\n".join(errs_b)
    check("budget_error_names_target", "mobile-low" in joined)
    check("budget_error_names_category", "textures" in joined)
    low_rec = next(x for x in mb["targets"] if x["target"] == "mobile-low")
    check("budget_error_recorded",
          len(low_rec["budgetErrors"]) == 1
          and low_rec["budgetErrors"][0]["over"] > 0)

    # 缺失文件：报错且带文件名
    os.remove(os.path.join(src, "tex", "hero.png"))
    out_c = os.path.join(tmp, "build-missing")
    _, errs_c = builder.run_pipeline(cfg, src, out_c)
    check("missing_file_error",
          any("hero.png" in e and isinstance(e, str) for e in errs_c))
    check("missing_is_missing_source",
          any("缺失" in e for e in errs_c))

    # 不支持的格式：写入假 bmp 并在 plan 中引用
    with open(os.path.join(src, "tex", "fake.bmp"), "wb") as f:
        f.write(b"BMfake")
    plan_d_path = os.path.join(tmp, "plan-fmt.json")
    write_plan(plan_d_path, src)
    with open(plan_d_path, "r", encoding="utf-8") as f:
        pd = json.load(f)
    pd["textures"].append({"name": "fake", "src": "tex/fake.bmp"})
    with open(plan_d_path, "w", encoding="utf-8") as f:
        json.dump(pd, f)
    try:
        plan_mod.load_plan(plan_d_path)
        check("unsupported_format_error", False)
    except ConfigError as e:
        check("unsupported_format_error", "fake.bmp" in str(e))

    # 路径越界：拒绝跳出源目录的引用
    plan_e_path = os.path.join(tmp, "plan-traversal.json")
    write_plan(plan_e_path, src)
    with open(plan_e_path, "r", encoding="utf-8") as f:
        pe = json.load(f)
    pe["textures"][0]["src"] = "../../etc/passwd.png"
    with open(plan_e_path, "w", encoding="utf-8") as f:
        json.dump(pe, f)
    cfg_e = plan_mod.load_plan(plan_e_path)
    try:
        builder.run_pipeline(cfg_e, src, os.path.join(tmp, "build-e"))
        check("path_traversal_blocked", False)
    except ConfigError:
        check("path_traversal_blocked", True)

    # 源目录在错误构建后依然保持快照（变化只来自本测试自己的删/加文件，
    # 流水线本身从未写过 src）
    after = _snapshot(src)
    untouched = all(after.get(k) == h for k, h in before.items()
                    if k in after)
    check("source_untouched_after_errors", untouched)


# ---------- 6. CLI 退出码 ----------

def test_cli(tmp):
    src = os.path.join(tmp, "cli_assets")
    _setup_sources(src)
    plan_path = os.path.join(tmp, "cli_plan.json")
    write_plan(plan_path, src)
    entry = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "src", "assetpipe.py")
    out = os.path.join(tmp, "cli_build")

    ok = subprocess.run(
        [sys.executable, "-I", entry, "--plan", plan_path,
         "--source", src, "--out", out],
        capture_output=True, text=True)
    check("cli_success_exit0", ok.returncode == 0)
    check("cli_manifest_written",
          os.path.isfile(os.path.join(out, "manifest.json")))

    tight_plan = os.path.join(tmp, "cli_tight.json")
    write_plan(tight_plan, src, budgets={"pc-high": {"total": 1}})
    tight_out = os.path.join(tmp, "cli_tight_build")
    fail = subprocess.run(
        [sys.executable, "-I", entry, "--plan", tight_plan,
         "--source", src, "--out", tight_out],
        capture_output=True, text=True)
    check("cli_budget_exit1", fail.returncode == 1
          and "pc-high" in fail.stderr)

    bad_plan = os.path.join(tmp, "cli_bad.json")
    write_plan(bad_plan, src, broken="unknown_quality")
    cfg_err = subprocess.run(
        [sys.executable, "-I", entry, "--plan", bad_plan,
         "--source", src, "--out", os.path.join(tmp, "cli_bad_build")],
        capture_output=True, text=True)
    check("cli_config_exit2", cfg_err.returncode == 2)


def main():
    tmp = tempfile.mkdtemp(prefix="assetpipe-selftest-")
    try:
        test_png(tmp)
        test_wav(tmp)
        test_atlas()
        test_plan(tmp)
        test_pipeline(tmp)
        test_cli(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print("\n%s，失败 %d 项"
          % ("全部通过" if FAILURES == 0 else "存在失败", FAILURES))
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())
