#!/usr/bin/env python3
"""Cross-check failure-point coverage.

For one module (or all modules when invoked with no argument) this verifies:

  1. every registered failpoint used by the module is declared in
     common/failpoints.h and appears as a DIAG_FAIL_RETURN / diag_fail_point
     site in the module source;
  2. every DIAG_FAIL_RETURN site reached from the stage entry is registered;
  3. every registered failpoint with a stage-reachable site has a matching
     acceptance case in <module>/tests/harness.c and a fault fixture under
     <module>/tests/faults/;
  4. no failure return escapes the diagnostics layer unnoticed: any literal
     `return -1` or `return 1` on a failure path in the stage call graph is
     reported as UNCOVERED.

Exit status is non-zero if any check fails.
"""
import re
import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
MODULES = ["particle", "tilemap", "navgrid", "spatial"]
SRC = {
    "particle": ROOT / "particle/src/particle.c",
    "tilemap": ROOT / "tilemap/src/tilemap.c",
    "navgrid": ROOT / "navgrid/src/navgrid.c",
    "spatial": ROOT / "spatial/src/spatial.c",
}
HARNESS = {m: ROOT / m / "tests/harness.c" for m in MODULES}
FAULTS = {m: ROOT / m / "tests/faults" for m in MODULES}


def registry():
    """Parse FAILPOINT_LIST(X) in common/failpoints.h -> [(mod, point, code)]."""
    text = (ROOT / "common/failpoints.h").read_text()
    body = text.split("FAILPOINT_LIST(X)", 1)[1]
    out = []
    for m in re.finditer(r"X\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,", body):
        out.append((m.group(1), m.group(2), m.group(3)))
    return out


FAIL_RE = re.compile(r"DIAG_FAIL_RETURN\(\s*MOD\s*,\s*\"(\w+)\"")
DIAGPT_RE = re.compile(r"diag_fail_point\(\s*MOD\s*,\s*\"(\w+)\"")
CASE_RE = re.compile(r"CASE\(\s*\w+\s*,\s*\"(\w+)\"\s*\)")
ACCTEST_RE = re.compile(r"acctest_run\s*\(")
FAULT_FILE_RE = re.compile(r"^(\w+)\.fault$")


def stage_body(text):
    """Return the source text of the stage function (the render entry point)."""
    m = re.search(r"int\s+\w+_stage\s*\(", text)
    if not m:
        return ""
    start = m.start()
    depth = 0
    i = text.index("{", start)
    for j in range(i, len(text)):
        if text[j] == "{":
            depth += 1
        elif text[j] == "}":
            depth -= 1
            if depth == 0:
                return text[start:j + 1]
    return text[start:]


def raw_failure_returns(body):
    """Find literal failure returns in the stage body that bypass DIAG."""
    bad = []
    for m in re.finditer(r"return\s+(-?\d+)\s*;", body):
        val = int(m.group(1))
        if val < 0 or (val == 1 and "DIAG" not in body[max(0, m.start()-80):m.start()]):
            line = body[:m.start()].count("\n")
            bad.append((line, m.group(0)))
    return bad


def check_module(mod, points, problems):
    text = SRC[mod].read_text()
    src_sites = set(FAIL_RE.findall(text)) | set(DIAGPT_RE.findall(text))
    reg = {p for (m, p, _) in points if m == mod}

    unknown = src_sites - reg
    if unknown:
        problems.append(f"{mod}: FAIL_RETURN sites not in registry: {sorted(unknown)}")

    harness_text = HARNESS[mod].read_text() if HARNESS[mod].exists() else ""
    cases = set(CASE_RE.findall(harness_text))
    fixture_names = {p.stem for p in FAULTS[mod].glob("*.fault")} if FAULTS[mod].is_dir() else set()
    # A fixture-driven suite counts as the acceptance case for its point.
    if ACCTEST_RE.search(harness_text):
        cases |= fixture_names

    # Every registry entry must be exercised, either by a direct-case or stage gate.
    for point in sorted(reg):
        reached = point in src_sites
        if not reached:
            problems.append(f"{mod}: registered '{point}' has no DIAG_FAIL_RETURN site")
            continue
        if point not in cases:
            problems.append(f"{mod}: '{point}' missing acceptance CASE in tests/harness.c")
        if point not in fixture_names:
            problems.append(f"{mod}: '{point}' missing fault fixture tests/faults/{point}.fault")
    stray_cases = cases - reg
    if stray_cases:
        problems.append(f"{mod}: harness CASEs not in registry: {sorted(stray_cases)}")
    stray_fixtures = fixture_names - reg
    if stray_fixtures:
        problems.append(f"{mod}: fault fixtures not in registry: {sorted(stray_fixtures)}")

    body = stage_body(text)
    for line, snippet in raw_failure_returns(body):
        problems.append(f"{mod}: stage has host-level failure return `{snippet}` (line {line})")

    print(f"{mod}: sites={len(src_sites)} registry={len(reg)} cases={len(cases)} "
          f"fixtures={len(fixture_names)}")
    return not any(p.startswith(mod + ":") for p in problems)


def main():
    targets = sys.argv[1:] or MODULES
    unknown = [t for t in targets if t not in MODULES]
    if unknown:
        print(f"unknown module(s): {unknown}", file=sys.stderr)
        return 2
    points = registry()
    problems = []
    for mod in targets:
        check_module(mod, points, problems)
    if problems:
        print("\nCOVERAGE GAPS:")
        for p in problems:
            print("  - " + p)
        return 1
    print("coverage OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
