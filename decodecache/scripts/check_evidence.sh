#!/usr/bin/env bash
# check_evidence.sh: hard gate for every acceptance condition in the task.
#   1. all four evidence files exist and are non-empty
#   2. cache on: exactly one decode per module, every later frame a cache hit
#      (re-decode count after the first hit must be zero)
#   3. cache off: legacy path re-decodes every frame and never hits
#   4. cache-off pixels are byte-identical to cache-on (identical=1 and equal
#      output hashes)
# Any failure prints EVIDENCE_FAIL and exits non-zero.
set -euo pipefail
cd "$(dirname "$0")/../.."   # repo root

evi=decodecache/evidence
modules="tilemap fontatlas particle lod"
fail=0
bad() { echo "EVIDENCE_FAIL: $*"; fail=1; }

need_file() { [ -s "$1" ] || bad "missing or empty evidence file $1"; }
need_file "$evi/decode_time.txt"
need_file "$evi/hit_log.txt"
need_file "$evi/pixel_hash.txt"
need_file "$evi/repro.txt"

for m in $modules; do
  log="$evi/$m.log"
  [ -s "$log" ] || { bad "$m: missing bench log"; continue; }

  # self-test assertions emitted by the pipeline must all have passed
  if ! grep -q '^all_ok=1$' "$log"; then bad "$m: self-test did not report all_ok=1"; fi
  if ! grep -q '^on_zero_redecode=1$' "$log"; then bad "$m: zero-redecode contract missing"; fi
  if ! grep -q '^off_decodes_every_frame=1$' "$log"; then bad "$m: legacy re-decode contract missing"; fi
  if ! grep -q '^identical=1$' "$log"; then bad "$m: cache-off pixels not byte-identical"; fi

  # decode counts: cache_on decodes=1 hits=3 ; cache_off decodes=4 hits=0
  counts=$(grep -E '^counts:' "$log")
  echo "$counts" | grep -Eq 'cache_on decodes=1 hits=3' || bad "$m: cache on did not decode exactly once ($counts)"
  echo "$counts" | grep -Eq 'cache_off decodes=4 hits=0' || bad "$m: cache off did not re-decode every frame ($counts)"

  # the hit log must show real cache_hit events and no decode after the first
  hits=$(grep -c '^hitlog: cache_hit' "$log" || true)
  [ "$hits" -eq 3 ] || bad "$m: expected 3 cache_hit lines, found $hits"
  if grep -E '^hitlog: cache_(hit|miss)' "$log" | grep -Eq 'decodes=[2-9]'; then
    bad "$m: a re-decode happened after the cache was warm"
  fi

  # cache-on and cache-off output hashes on the pixel_hash line must be equal
  hline=$(grep -E '^pixel_hash:' "$log")
  on=$(echo "$hline" | sed -E 's/.*output_on=([0-9a-f]+).*/\1/')
  off=$(echo "$hline" | sed -E 's/.*output_off=([0-9a-f]+).*/\1/')
  [ -n "$on" ] && [ "$on" = "$off" ] || bad "$m: output hashes differ ($hline)"

  # the timing table really recorded this module's per-frame measurements
  grep -q "^# $m decode time" "$evi/decode_time.txt" || bad "$m: absent from decode_time.txt"
  grep -Eq 'decode_time: frame=3' "$evi/decode_time.txt" || bad "$m: incomplete timing rows"

  # and a reproduce command was recorded for it
  grep -q "make -C $m bench" "$evi/repro.txt" || bad "$m: no reproduce command"

  echo "OK $m: decodes_on=1 hits=3 decodes_off=4 identical=1 hash=$on"
done

echo "---"
if [ "$fail" -eq 0 ]; then
  echo "EVIDENCE_OK: all four evidence kinds present and every gate passed"
else
  echo "EVIDENCE_FAILED"
fi
exit "$fail"
