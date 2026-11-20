#!/usr/bin/env python3
"""Regression harness for tilemap/src/tilemap.c.

Checks, in order:
  1. --legacy output is byte-identical to the frozen pre-incremental output.
  2. The default run passes (pixel equality, seam scan, static index buffer,
     full-rebuild byte equality, upload-byte budget).
  3. --seam-scan exits 0 on the clean build.
  4. --seam-scan --inject-seam exits non-zero (the checker catches a dropped
     cross-chunk dirty mark).
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(HERE, "tilemap")
GOLDEN = os.path.join(HERE, "tests", "legacy_baseline.txt")

# Pebble-count budget for the walk test: 32 frames, 2 touched chunks worst case.
MAX_WALK_VB_BYTES = 4096


def run(*args, env=None):
    return subprocess.run([BIN, *args], capture_output=True, text=True, env=env)


def parse(out):
    vals = {}
    for line in out.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            vals[k] = v
    return vals


def main():
    failures = []

    def fail(msg):
        failures.append(msg)
        print("FAIL " + msg)

    if not os.path.exists(BIN):
        print("missing binary, run make first")
        return 2

    # legacy byte identity
    r = run("--legacy")
    with open(GOLDEN, "rb") as f:
        golden = f.read()
    if r.returncode != 0:
        fail("--legacy exit=%d" % r.returncode)
    if r.stdout.encode() != golden:
        fail("--legacy output differs from frozen baseline")

    # the incremental path can also be switched off from the environment; the
    # program must then print exactly what the pre-chunked version printed
    r_env = run(env=dict(os.environ, NO_DIRTY="1"))
    if r_env.returncode != 0:
        fail("NO_DIRTY=1 exit=%d" % r_env.returncode)
    if r_env.stdout.encode() != golden:
        fail("NO_DIRTY=1 output differs from frozen baseline")

    r_flag = run("--no-dirty")
    if r_flag.stdout.encode() != golden:
        fail("--no-dirty output differs from frozen baseline")

    # 2. default run
    r = run()
    vals = parse(r.stdout)
    print(r.stdout, end="")
    if r.returncode != 0 or vals.get("all_ok") != "1":
        fail("default run failed")
    for key in ("interior_pixels_match", "interior_seams_clean", "border_pixels_match",
                "border_seams_clean", "walk_pixels_match", "walk_seams_clean",
                "index_buffer_static", "no_dirty_byte_identical",
                "injected_seam_detected"):
        if vals.get(key) != "1":
            fail("%s=%s" % (key, vals.get(key)))

    walk_bytes = int(vals.get("walk_dirty_vb_bytes", "-1"))
    full_bytes = int(vals.get("full_rebuild_bytes_per_frame", "0"))
    if walk_bytes < 0 or walk_bytes > MAX_WALK_VB_BYTES:
        fail("walk_dirty_vb_bytes=%d exceeds budget %d" % (walk_bytes, MAX_WALK_VB_BYTES))
    if full_bytes <= 0:
        fail("full_rebuild_bytes_per_frame missing")
    print("upload_ratio=%.4f (dirty/full per frame, walk avg)" %
          (walk_bytes / (full_bytes * int(vals.get("walk_frames", 1)))))

    # 3. clean seam scan
    r = run("--seam-scan")
    if r.returncode != 0:
        fail("--seam-scan exit=%d on clean build" % r.returncode)

    # 4. injected seam must be caught
    r = run("--seam-scan", "--inject-seam")
    if r.returncode == 0:
        fail("--seam-scan --inject-seam did not report the dropped border mark")

    if failures:
        print("check_tilemap: %d failure(s)" % len(failures))
        return 1
    print("check_tilemap: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
