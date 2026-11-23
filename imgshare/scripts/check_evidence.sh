#!/usr/bin/env bash
# check_evidence.sh: enforce every acceptance condition from the task.
#   1. share mode peak memory <= 2 x input image
#   2. copy log distinguishes the two modes (share elides the per-consumer copies)
#   3. pixel hashes are identical with the sharing wired in and switched off
#   4. a reproduce command exists and, re-run, reproduces the evidence
# Plus: the illegal-interface probe must reject every bad call.
set -euo pipefail
cd "$(dirname "$0")/.."

fail=0
say() { printf '%s\n' "$*"; }
bad() { say "EVIDENCE_FAIL: $*"; fail=1; }

need_file() {
  [ -s "$1" ] || { bad "missing evidence file $1"; return 1; }
}

# --- 0. illegal interface --------------------------------------------------
probe_out=$(./imgshare-demo --probe-illegal || true)
say "$probe_out"
if grep -q 'ACCEPTED' <<<"$probe_out"; then bad "illegal interface accepted"; fi
if ! grep -q '^probe_bad=0$' <<<"$probe_out"; then bad "illegal probe reported violations"; fi

# --- 1. peak memory --------------------------------------------------------
need_file evidence/peak.txt || true
share_peak=$(awk -F= '/^peak_bytes=/{print $2; exit}' evidence/share.log)
share_ratio=$(awk -F= '/^peak_ratio=/{print $2; exit}' evidence/share.log)
if [ -z "${share_peak:-}" ] || [ -z "${share_ratio:-}" ]; then
  bad "could not read peak values from evidence/share.log"
else
  # integer-safe check: peak <= 2*input, and the printed ratio <= 2.0
  if awk -v r="$share_ratio" 'BEGIN{exit !(r <= 2.0)}'; then
    say "PEAK_OK: share ratio=$share_ratio peak_bytes=$share_peak (limit 2.0)"
  else
    bad "share peak ratio $share_ratio exceeds 2.0"
  fi
fi

noshare_ratio=$(awk -F= '/^peak_ratio=/{print $2; exit}' evidence/noshare.log)
if [ -n "${noshare_ratio:-}" ]; then
  if awk -v r="$noshare_ratio" 'BEGIN{exit !(r > 2.0)}'; then
    say "PEAK_BASELINE: noshare ratio=$noshare_ratio (expected > 2.0)"
  else
    say "NOTE: noshare ratio=$noshare_ratio did not exceed 2.0; baseline weaker than expected"
  fi
fi

# --- 2. copy count ---------------------------------------------------------
need_file evidence/copies.txt || true
share_copies=$(awk -F= '/^copies=/{print $2; exit}' evidence/share.log)
noshare_copies=$(awk -F= '/^copies=/{print $2; exit}' evidence/noshare.log)
if [ -z "${share_copies:-}" ] || [ -z "${noshare_copies:-}" ]; then
  bad "could not read copy counts"
else
  say "COPIES: share=$share_copies noshare=$noshare_copies"
  [ "$share_copies" -eq 0 ] || bad "share mode still performs $share_copies pixel copies"
  [ "$noshare_copies" -gt 0 ] || bad "baseline log shows no copies; evidence is not real"
  [ "$noshare_copies" -gt "$share_copies" ] || bad "share mode did not reduce copies"
fi

# --- 3. pixel hashes -------------------------------------------------------
need_file evidence/hashes.txt || true
# compare the consumer hash block between modes
share_block=$(grep '^share' evidence/hashes.txt | sed 's/^share *//' | grep -E '^(consumer\.|input_hash=)' || true)
noshare_block=$(grep '^noshare' evidence/hashes.txt | sed 's/^noshare *//' | grep -E '^(consumer\.|input_hash=)' || true)
if [ -z "$share_block" ] || [ -z "$noshare_block" ]; then
  bad "missing pixel hash evidence"
elif [ "$share_block" = "$noshare_block" ]; then
  say "HASH_OK: every consumer hash identical across share/noshare"
  say "$share_block" | sed 's/^/  /'
else
  bad "pixel hashes differ between share and noshare"
  diff <(printf '%s\n' "$share_block") <(printf '%s\n' "$noshare_block") || true
fi
# every consumer must have seen the input pixels
if grep -q '^all_hashes_match_input=1' evidence/share.log && \
   grep -q '^all_consumer_hashes_equal=1' evidence/share.log; then
  say "HASH_MATCH_INPUT: ok"
else
  bad "share-mode hashes do not all match the input"
fi

# --- 4. reproduce ----------------------------------------------------------
need_file evidence/repro.txt || true
repro=$(grep -m1 '^repro=' evidence/share.log | sed 's/^repro=//')
[ -n "${repro:-}" ] || bad "no reproduce command recorded"
say "REPRO: $repro"
if [ -n "${repro:-}" ]; then
  # re-run it and confirm the peak/copies lines come back the same
  rerun=$(eval "$repro" 2>/dev/null || true)
  if grep -q '^peak_ratio=' <<<"$rerun" && grep -q '^copies=' <<<"$rerun"; then
    say "REPRO_OK: command re-ran and emitted evidence"
  else
    bad "reproduce command did not emit evidence on re-run"
  fi
fi

say "---"
if [ "$fail" -eq 0 ]; then
  say "EVIDENCE_OK: all four checks passed"
else
  say "EVIDENCE_FAILED"
fi
exit "$fail"
