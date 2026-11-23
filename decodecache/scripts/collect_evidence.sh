#!/usr/bin/env bash
# collect_evidence.sh: run the four decode pipelines (cache on vs off) and
# capture the four pieces of evidence the task requires, one file each:
#   1. decode timing table -> evidence/decode_time.txt
#   2. cache hit log       -> evidence/hit_log.txt
#   3. pixel hashes        -> evidence/pixel_hash.txt
#   4. reproduce commands  -> evidence/repro.txt
# check_evidence.sh then enforces every acceptance condition on these files.
set -euo pipefail
cd "$(dirname "$0")/../.."   # repo root

out=decodecache/evidence
mkdir -p "$out"
modules="tilemap fontatlas particle lod"

: > "$out/decode_time.txt"
: > "$out/hit_log.txt"
: > "$out/pixel_hash.txt"
: > "$out/repro.txt"

for m in $modules; do
  log="$out/$m.log"
  make -s -C "$m" bench | tee "$log" >/dev/null

  # 1. decode timing table: every per-frame decode/measurement line
  {
    echo "# $m decode time (us); cache_on frame 0 is the only real decode"
    grep -E '^decode_time:' "$log"
    echo
  } >> "$out/decode_time.txt"

  # 2. cache hit log: cache_hit / cache_miss / cache_off events
  {
    echo "# $m cache events (decodes must stay 1 after the first hit)"
    grep -E '^hitlog:' "$log"
    echo
  } >> "$out/hit_log.txt"

  # 3. pixel hashes: input and cache-on/off outputs must agree
  {
    echo "# $m FNV-1a pixel hashes"
    grep -E '^pixel_hash:' "$log"
    grep -E '^counts:' "$log"
    echo
  } >> "$out/pixel_hash.txt"

  # 4. reproduce command
  grep -E '^repro=' "$log" >> "$out/repro.txt"
done

# canonical top-level reproduction path
echo "make evidence   # collect + verify all four evidence kinds" >> "$out/repro.txt"

echo "wrote $out/{decode_time,hit_log,pixel_hash,repro}.txt"
