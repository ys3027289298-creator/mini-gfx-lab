#!/usr/bin/env python3
"""End-to-end evidence collector for the gfault unification work.

Runs (read-only against the source tree, writes only under artifacts/):
  * per-module stdout regression vs artifacts/baseline
  * healthy compositor on current and golden (pre-change) sources
  * per-segment SHA-256 + byte-exact pixel comparison (5 separate checks)
  * one fault injection per segment with pixel-level isolation checks
  * zero-allocation counter results
  * natural failure-path encoding results
  * PPM -> PNG conversion for every screenshot

All paths are passed explicitly; this script imports nothing from the
artifact data directory.
"""
import hashlib
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EVID = os.path.join(ROOT, "artifacts", "evidence")
SHOTS = os.path.join(EVID, "screenshots")
LOGS = os.path.join(EVID, "logs")
GOLDEN_TREE = "/tmp/mgl-golden"
SEGMENTS = ["particle", "tilemap", "navgrid", "fontatlas", "palette"]

# panel geometry, must match gfault/render/framebuf.c
W, H = 320, 240
REGIONS = {
    "particle": (0, 0, 160, 120), "tilemap": (160, 0, 160, 120),
    "navgrid": (0, 120, 160, 120), "fontatlas": (160, 120, 160, 60),
    "palette": (160, 180, 160, 60),
}
BG = bytes([18, 18, 24])

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ppm2png import read_ppm, write_png  # noqa: E402


def run(cmd, cwd=None, logpath=None):
    p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if logpath:
        with open(logpath, "w") as f:
            f.write(f"$ {' '.join(cmd)}\n--- stdout ---\n{p.stdout}\n--- stderr ---\n{p.stderr}\n")
    return p


def sha256_file(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def region_bytes(data, seg):
    x, y, w, h = REGIONS[seg]
    out = bytearray()
    for ry in range(y, y + h):
        out += data[(ry * W + x) * 3:(ry * W + x + w) * 3]
    return bytes(out)


def region_is_background(data, seg):
    x, y, w, h = REGIONS[seg]
    for ry in range(y, y + h):
        for rx in range(x, x + w):
            base = (ry * W + rx) * 3
            if data[base:base + 3] != BG:
                return False
    return True


def main():
    results = []

    def record(name, ok, detail=""):
        results.append((name, ok, detail))
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")

    if os.path.isdir(EVID):
        shutil.rmtree(EVID)
    for d in (SHOTS, LOGS):
        os.makedirs(d, exist_ok=True)

    # ---- build everything ----
    modules = ["particle", "tilemap", "navgrid", "fontatlas", "palette"]
    for m in modules:
        p = run(["make", "-C", os.path.join(ROOT, m), "clean", "test"],
                logpath=os.path.join(LOGS, f"{m}.log"))
        record(f"build+selftest:{m}", p.returncode == 0)

    p = run(["make", "-C", os.path.join(ROOT, "gfault"), "clean"],
            logpath=os.path.join(LOGS, "gfault_clean.log"))
    p = run(["make", "-C", os.path.join(ROOT, "gfault"), "all",
             f"GOLDEN_TREE={GOLDEN_TREE}"],
            logpath=os.path.join(LOGS, "gfault_build.log"))
    record("build:gfault harness (current+golden+alloc+natural)", p.returncode == 0)
    if p.returncode != 0:
        print(p.stdout, p.stderr)
        return 1

    # ---- 1. module stdout regression vs archived baseline ----
    bins = {"particle": "particle/particle-pool", "tilemap": "tilemap/tilemap",
            "navgrid": "navgrid/mini-navgrid", "fontatlas": "fontatlas/fontatlas"}
    for m, rel in bins.items():
        p = run([os.path.join(ROOT, rel)])
        base = os.path.join(ROOT, "artifacts", "baseline", f"{m}.stdout.txt")
        baseline = "\n".join(
            ln for ln in open(base).read().splitlines() if not ln.startswith("./"))
        record(f"stdout-identical:{m}", p.stdout.strip() == baseline.strip())

    # ---- 2. healthy screenshots, current vs golden ----
    cur_dir, gold_dir = (os.path.join(SHOTS, "current"),
                         os.path.join(SHOTS, "golden"))
    os.makedirs(cur_dir, exist_ok=True)
    os.makedirs(gold_dir, exist_ok=True)
    p = run([os.path.join(ROOT, "gfault/compositor"), "healthy", cur_dir],
            logpath=os.path.join(LOGS, "compositor_current.log"))
    record("compositor healthy (current sources)", p.returncode == 0)
    p = run([os.path.join(ROOT, "gfault/compositor_golden"), "healthy", gold_dir],
            logpath=os.path.join(LOGS, "compositor_golden.log"))
    record("compositor healthy (golden pre-change sources)", p.returncode == 0)

    manifest = ["# pixel comparison manifest (5 separate per-segment checks)", ""]
    for seg in SEGMENTS:
        g, c = (os.path.join(gold_dir, f"seg_{seg}.ppm"),
                os.path.join(cur_dir, f"seg_{seg}.ppm"))
        hg, hc = sha256_file(g), sha256_file(c)
        same = hg == hc
        record(f"pixel-identical:{seg}", same, f"sha256={hc[:16]}...")
        manifest.append(f"{seg:10s} identical={int(same)}")
        manifest.append(f"  golden  {hg}")
        manifest.append(f"  current {hc}")
    comp_same = sha256_file(os.path.join(gold_dir, "composite.ppm")) == \
        sha256_file(os.path.join(cur_dir, "composite.ppm"))
    record("pixel-identical:composite", comp_same)
    open(os.path.join(EVID, "pixel_manifest.txt"), "w").write("\n".join(manifest) + "\n")

    # ---- 3. fault injection + pixel isolation ----
    healthy_data = read_ppm(os.path.join(cur_dir, "composite.ppm"))[2]
    iso_lines = ["# fault isolation: one injected fault per segment", ""]
    for victim in SEGMENTS:
        d = os.path.join(SHOTS, f"fault_{victim}")
        os.makedirs(d, exist_ok=True)
        p = run([os.path.join(ROOT, "gfault/compositor"), "fault", victim, d],
                logpath=os.path.join(LOGS, f"fault_{victim}.log"))
        record(f"fault-injection runs:{victim}", p.returncode == 0)
        data = read_ppm(os.path.join(d, "composite.ppm"))[2]
        for seg in SEGMENTS:
            if seg == victim:
                ok = region_is_background(data, seg)
                detail = "panel pulled (background)"
            else:
                ok = region_bytes(data, seg) == region_bytes(healthy_data, seg)
                detail = "untouched vs healthy"
            record(f"isolation inject={victim} panel={seg}", ok, detail)
            iso_lines.append(f"inject={victim:9s} panel={seg:9s} ok={int(ok)} ({detail})")
    open(os.path.join(EVID, "isolation_manifest.txt"), "w").write("\n".join(iso_lines) + "\n")

    # ---- 4. zero allocation ----
    p = run([os.path.join(ROOT, "gfault/alloc_test")],
            logpath=os.path.join(LOGS, "alloc_test.log"))
    added_zero = "diagnostics_added_allocations=0" in p.stdout
    diag_only_zero = "diag_only_loop_alloc_delta=0" in p.stdout
    record("zero extra allocations with diagnostics on",
           p.returncode == 0 and added_zero and diag_only_zero,
           p.stdout.strip().replace("\n", " | "))

    # ---- 5. natural failure encoding ----
    p = run([os.path.join(ROOT, "gfault/natural_fault_test")],
            logpath=os.path.join(LOGS, "natural_fault_test.log"))
    record("natural failure paths encode to segment blocks",
           p.returncode == 0 and "all_ok=1" in p.stdout)

    # ---- 6. convert every PPM to PNG for viewing ----
    n_png = 0
    for dirpath, _dirs, files in os.walk(SHOTS):
        for fn in files:
            if fn.endswith(".ppm"):
                w, h, rgb = read_ppm(os.path.join(dirpath, fn))
                out = os.path.join(dirpath, fn[:-4] + ".png")
                write_png(out, w, h, rgb)
                n_png += 1
    record(f"PNG screenshots exported", n_png > 0, f"count={n_png}")

    total = len(results)
    passed = sum(1 for _, ok, _ in results if ok)
    summary = f"\nSUMMARY {passed}/{total} checks passed"
    print(summary)
    open(os.path.join(EVID, "summary.txt"), "w").write(
        "\n".join(f"{'PASS' if ok else 'FAIL'} {n} {d}" for n, ok, d in results)
        + "\n" + summary + "\n")
    return 0 if passed == total else 1


if __name__ == "__main__":
    raise SystemExit(main())
