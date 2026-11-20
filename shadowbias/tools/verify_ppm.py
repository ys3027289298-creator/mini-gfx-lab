#!/usr/bin/env python3
"""Cross-check the shadowbias C harness output against the PPM montages.

Reads each out/frameN.ppm (four rows: truth / fixed / adaptive / diff) and
verifies that the red/orange pixels in the diff row equal the streak/leak
counts the C program printed for the adaptive bias.  Run with -I so nothing
from this directory can shadow stdlib modules.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RED = (220, 60, 50)
ORANGE = (240, 170, 40)
SKY = (15, 15, 20)
AGREE_LIT = (240, 240, 240)
AGREE_SHD = (90, 90, 96)


def read_ppm(path):
    with open(path, "rb") as fh:
        data = fh.read()
    assert data[:2] == b"P6", "not a binary PPM"
    tokens = data.split(b"\n", 3)
    width, height = (int(x) for x in tokens[1].split())
    maxval = int(tokens[2])
    assert maxval == 255
    pixels = tokens[3]
    rows = []
    row_h = height // 4
    for r in range(4):
        row = []
        base = r * row_h * width * 3
        for y in range(row_h):
            for x in range(width):
                off = base + (y * width + x) * 3
                row.append(tuple(pixels[off:off + 3]))
        rows.append(row)
    return width, row_h, rows


def main():
    # rebuild and run the C harness with dump so ppm files and counts are fresh
    out_proc = subprocess.run(
        ["make", "-s", "dump"], cwd=ROOT, check=True,
        capture_output=True, text=True,
    )
    text = out_proc.stdout
    frame_re = re.compile(
        r"frame=(\d+) elev=.*?adapt streak=(\d+) leak=(\d+)"
    )
    expected = {int(m.group(1)): (int(m.group(2)), int(m.group(3)))
                for m in frame_re.finditer(text)}
    assert expected, "no per-frame counts parsed"

    failures = 0
    for frame, (streak, leak) in sorted(expected.items()):
        path = os.path.join(ROOT, "out", f"frame{frame}.ppm")
        width, row_h, rows = read_ppm(path)
        diff = rows[3]
        reds = sum(1 for px in diff if px == RED)
        oranges = sum(1 for px in diff if px == ORANGE)
        known = {RED, ORANGE, SKY, AGREE_LIT, AGREE_SHD}
        unknown = sum(1 for px in diff if px not in known)
        ok = reds == streak and oranges == leak and unknown == 0
        print(f"frame{frame}: ppm red={reds} (report {streak}), "
              f"orange={oranges} (report {leak}), unknown={unknown} "
              f"-> {'ok' if ok else 'FAIL'}")
        failures += not ok

    # sanity: the totals line must equal the sum of per-frame counts
    totals = re.search(
        r"adapt streak=(\d+) leak=(\d+) errors=(\d+) fallback=(\d+)", text
    ).groups()
    sums = (
        sum(v[0] for v in expected.values()),
        sum(v[1] for v in expected.values()),
    )
    ok = (int(totals[0]), int(totals[1])) == sums
    print(f"totals consistent: {ok} ({totals[0]}/{totals[1]} vs {sums[0]}/{sums[1]})")
    failures += not ok
    print("verify_ok=%d" % (failures == 0))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
