#!/usr/bin/env python3
"""Top-level acceptance driver.

Runs, for each module that must fail independently:

    make -C <module> acceptance      (suite + coverage + golden bytes)
    make -C chain   acceptance       (independent degradation + no-alloc)

and finally the self-built coverage report over all four modules.  Any missing
module, fixture, harness case, or golden mismatch makes this exit non-zero.
"""
import subprocess
import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
MODULES = ["particle", "tilemap", "navgrid", "spatial"]


def run(cmd, cwd=None):
    print(f"\n$ {' '.join(cmd)}")
    proc = subprocess.run(cmd, cwd=cwd)
    return proc.returncode


def main():
    rc = 0
    for m in MODULES:
        d = ROOT / m
        for required in ("tests/harness.c", "tests/golden.stdout",
                         "tests/golden.rc", "tests/faults"):
            if not (d / required).exists():
                print(f"MISSING {m}/{required}")
                rc = 1
        rc |= run(["make", "-s", "acceptance"], cwd=str(d))

    rc |= run(["make", "-s", "acceptance"], cwd=str(ROOT / "chain"))
    rc |= run([sys.executable, str(ROOT / "tools/coverage_report.py"), "chain"])

    print("\n" + ("ACCEPTANCE FAILED" if rc else "ALL ACCEPTANCE CHECKS PASSED"))
    return rc


if __name__ == "__main__":
    sys.exit(main())
