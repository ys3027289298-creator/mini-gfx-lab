#!/usr/bin/env bash
# Two-stage soft-knee floor validation. Run from this directory (validation/).
# Inputs under input/ are untrusted data and stay isolated from the toolchain.
set -euo pipefail
BLOOM=../bloom

# 1. library self-test (kernel normalization, W <= (L-t0)+ bound, inversion)
make -C .. clean test

# 2. real photo: PNG -> PPM (ffmpeg only touches data inside input/)
ffmpeg -loglevel error -y -i input/astronaut.png -pix_fmt rgb24 input/astronaut.ppm

# 3. two-stage validation driver
mkdir -p out
gcc -O2 -std=c11 -Wall -Wextra -DBLOOM_NO_MAIN -I../src scripts/validate.c \
    -o scripts/validate -lm
./scripts/validate | tee out/report.txt

# 4. CLI cross-check: inverted floor must match the driver's t0
"$BLOOM" solve input/astronaut.ppm input/photo_points.txt \
    -eps 2 -delta 16 -sigma 20 -radius 60 -gain 1 | tee out/cli_solve.txt
T0=$(awk '{for (i=1;i<=NF;i++) if ($i ~ /^t0=/) {sub("t0=","",$i); print $i}}' out/cli_solve.txt)
"$BLOOM" process input/astronaut.ppm out/cli_soft.pgm -mode soft \
    -t0 "$T0" -t1 "$(awk "BEGIN{print $T0+16}")" -sigma 20 -radius 60 -gain 1

# 5. PGM -> PNG for visual inspection
for f in out/*.pgm; do ffmpeg -loglevel error -y -i "$f" "${f%.pgm}.png"; done
echo "OK: see out/report.txt"
