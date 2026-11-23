#!/usr/bin/env python3
"""Self-built check that lists every failure point and its coverage status.

For the requested scope ("chain" = all four modules, or a single module) it
prints one line per registered failure point:

    COVERED    <module>:<point>   (source site + acceptance fixture)
    UNCOVERED  <module>:<point>   (missing site, fixture, or harness case)
    UNREGISTERED <file>:<line>    (a failure return that bypasses diag)

The exit status is non-zero when anything is UNCOVERED or UNREGISTERED, so the
same command can gate CI.  Pass --list to print only registered points and
exit 0 regardless of coverage.
"""
import re
import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
MODULES = ["particle", "tilemap", "navgrid", "spatial"]
SRC = {m: ROOT / m / "src" / f"{m}.c" for m in MODULES}

FAIL_RE = re.compile(r"DIAG_FAIL_RETURN\(\s*MOD\s*,\s*\"(\w+)\"")
BARE_RE = re.compile(r"^\s*return\s+(-?\d+)\s*;\s*(?://.*)?$")


def registry():
    text = (ROOT / "common/failpoints.h").read_text()
    body = text.split("FAILPOINT_LIST(X)", 1)[1]
    out = []
    for m in re.finditer(r"X\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,", body):
        out.append(m.groups())
    return out


def has_fixture(mod, point):
    return (ROOT / mod / "tests/faults" / f"{point}.fault").is_file()


def harness_covers(mod):
    h = ROOT / mod / "tests/harness.c"
    return h.is_file() and "acctest_run(" in h.read_text()


def _strip_comments(text):
    text = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"),
                  text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def stage_bodies(text):
    """Yield (start_line, body) for every <mod>_stage function."""
    text = _strip_comments(text)
    for m in re.finditer(r"int\s+\w+_stage\s*\([^)]*\)\s*\{", text):
        start = text.index("{", m.start())
        depth = 0
        for j in range(start, len(text)):
            if text[j] == "{":
                depth += 1
            elif text[j] == "}":
                depth -= 1
                if depth == 0:
                    line = text[:m.start()].count("\n") + 1
                    yield line, text[start:j + 1]
                    break


def unregistered_returns(mod):
    """Failure-looking returns in stage bodies that bypass DIAG_FAIL_RETURN.

    A literal negative return, or a bare `return 1;`, inside a stage is a
    failure path that would be invisible to the diagnostics layer.
    """
    text = SRC[mod].read_text()
    out = []
    for start_line, body in stage_bodies(text):
        # Walk statement-sized chunks so an inline `if (x) return -1;` is
        # caught just like one on its own line.
        for m in re.finditer(r"return\s+(-?\d+)\s*;", body):
            line_no = start_line + body[:m.start()].count("\n")
            line_start = body.rfind("\n", 0, m.start()) + 1
            line_end = body.find("\n", m.end())
            snippet = body[line_start:line_end if line_end >= 0 else len(body)].strip()
            if "DIAG" in snippet:
                continue
            val = int(m.group(1))
            if val < 0 or val == 1:
                out.append((line_no, snippet))
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    list_only = "--list" in sys.argv
    scope = args[0] if args else "chain"
    mods = MODULES if scope == "chain" else [scope]
    if any(m not in MODULES for m in mods):
        print(f"unknown scope {scope}", file=sys.stderr)
        return 2

    points = registry()
    source_sites = {m: set(FAIL_RE.findall(SRC[m].read_text())) for m in mods}

    covered = uncovered = 0
    for mod, point, code in points:
        if mod not in mods:
            continue
        problems = []
        if point not in source_sites[mod]:
            problems.append("no source site")
        if not has_fixture(mod, point):
            problems.append("no fault fixture")
        if not harness_covers(mod):
            problems.append("no harness case")
        if list_only:
            print(f"{mod}:{point}:{code}  ({', '.join(problems) if problems else 'covered'})")
            continue
        if problems:
            uncovered += 1
            print(f"UNCOVERED    {mod}:{point:<24} {', '.join(problems)}")
        else:
            covered += 1
            print(f"COVERED      {mod}:{point:<24} {code}")

    if list_only:
        return 0

    stray = 0
    for mod in mods:
        reg = {p for (m, p, _) in points if m == mod}
        for site in sorted(source_sites[mod] - reg):
            print(f"UNREGISTERED {mod}:{site} (source site, not in registry)")
            stray += 1
        for line, snippet in unregistered_returns(mod):
            print(f"UNREGISTERED {mod}/src/{mod}.c:{line}: {snippet}")
            stray += 1

    print(f"\n{scope}: {covered} covered, {uncovered} uncovered, "
          f"{stray} unregistered")
    return 0 if uncovered == 0 and stray == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
