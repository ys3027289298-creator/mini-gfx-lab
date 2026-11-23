#!/usr/bin/env bash
# collect_evidence.sh: run the demo in both modes and capture the four pieces
# of evidence the task requires:
#   1. peak-memory table       -> evidence/peak.txt
#   2. copy-count log          -> evidence/copies.txt
#   3. pixel hashes            -> evidence/hashes.txt
#   4. reproduce command       -> evidence/repro.txt
# Then verify them in check_evidence.sh.
set -euo pipefail
cd "$(dirname "$0")/.."
out=evidence
mkdir -p "$out"

share_log=$out/share.log
noshare_log=$out/noshare.log

./imgshare-demo --share    | tee "$share_log"
./imgshare-demo --noshare  | tee "$noshare_log"

{
  echo "# peak-memory table (input_bytes is the source image size)"
  echo "# mode      peak_bytes  peak_ratio  limit  ok"
  grep -h '^peak_bytes=' "$share_log"   | sed 's/^/share   /'
  grep -h '^peak_ratio=' "$share_log"   | sed 's/^/share   /'
  grep -h '^peak_ok='    "$share_log"   | sed 's/^/share   /'
  grep -h '^peak_bytes=' "$noshare_log" | sed 's/^/noshare /'
  grep -h '^peak_ratio=' "$noshare_log" | sed 's/^/noshare /'
  grep -h '^peak_ok='    "$noshare_log" | sed 's/^/noshare /'
} > "$out/peak.txt"

{
  echo "# copy-count log"
  grep -h '^copies=' "$share_log"   | sed 's/^/share   /'
  grep -h '^shares=' "$share_log"   | sed 's/^/share   /'
  grep -h '^copies=' "$noshare_log" | sed 's/^/noshare /'
  grep -h '^shares=' "$noshare_log" | sed 's/^/noshare /'
} > "$out/copies.txt"

{
  echo "# pixel hashes, share vs noshare must be byte-for-byte identical"
  grep -hE '^(consumer\.|input_hash=|all_)' "$share_log"   | sed 's/^/share   /'
  grep -hE '^(consumer\.|input_hash=|all_)' "$noshare_log" | sed 's/^/noshare /'
} > "$out/hashes.txt"

{
  echo "# reproduce commands"
  grep -h '^repro=' "$share_log"   | sed 's/^/share   /'
  grep -h '^repro=' "$noshare_log" | sed 's/^/noshare /'
  echo "make -C imgshare evidence"   # from the repo root
} > "$out/repro.txt"

echo "wrote $out/{peak,copies,hashes,repro}.txt"
