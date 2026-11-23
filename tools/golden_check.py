#!/usr/bin/env python3
"""Verify that the success path of a module is byte-identical to its golden
output (stdout, stderr and exit status)."""
import subprocess
import sys
import pathlib


def main():
    if len(sys.argv) != 5:
        print("usage: golden_check.py <module> <binary> <golden.stdout> <golden.rc>",
              file=sys.stderr)
        return 2
    module, binary, golden_out, golden_rc = sys.argv[1:]
    golden_out = pathlib.Path(golden_out)
    golden_rc = pathlib.Path(golden_rc)

    proc = subprocess.run([binary], capture_output=True)
    want_out = golden_out.read_bytes()
    want_rc = int(golden_rc.read_text().strip())

    ok = True
    if proc.stdout != want_out:
        ok = False
        print(f"{module}: stdout differs from golden")
        for i, (a, b) in enumerate(zip(proc.stdout.splitlines(), want_out.splitlines())):
            if a != b:
                print(f"  line {i+1}: got {a!r} want {b!r}")
                break
    if proc.stderr != b"":
        ok = False
        print(f"{module}: stderr not empty on success path: {proc.stderr!r}")
    if proc.returncode != want_rc:
        ok = False
        print(f"{module}: rc={proc.returncode} want {want_rc}")
    if ok:
        print(f"{module}: success path byte-identical to golden")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
