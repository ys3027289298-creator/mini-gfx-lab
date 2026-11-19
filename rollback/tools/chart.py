#!/usr/bin/env python3
"""Render rollback/metrics.csv as an SVG chart.

Two panels, same three series, same y-axis semantics (one axis per panel):
  top    - the whole match, showing the three clients' error over time
  bottom - a zoom on the first impulse with one mark per tick, so the rollback's
           exact return to zero is visible rather than a one-pixel triangle.

Pure stdlib; no plotting dependency. Colours are validated categorical slots
1-3 (light mode, surface #fcfcfb); see the rollback README.

usage: python3 tools/chart.py metrics.csv metrics.svg
"""
import csv
import sys
from pathlib import Path

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
SERIES = {
    "err_rollback": ("#2a78d6", "rollback (predict + reconcile)"),
    "err_ignore": ("#eb6834", "ignore (predict only)"),
    "err_snap": ("#1baf7a", "snap (draw packets as they land)"),
}
ORDER = ["err_rollback", "err_ignore", "err_snap"]
FONT = ("'WenQuanYi Zen Hei','Noto Sans CJK SC','Microsoft YaHei',"
        "system-ui,-apple-system,'Segoe UI',sans-serif")

W, H = 1000, 830
PL, PR = 68, 96


def parse(path):
    cfg, rows, summary, impulses = {}, [], {}, []
    buf = []
    with open(path, newline="") as fh:
        for raw in fh:
            line = raw.rstrip("\n")
            if line.startswith("#"):
                body = line[1:].strip()
                if body.startswith("cfg "):
                    for tok in body[4:].split():
                        k, _, v = tok.partition("=")
                        cfg[k] = v
                elif body.startswith("summary "):
                    for tok in body[8:].split():
                        k, _, v = tok.partition("=")
                        summary[k] = v
                elif body.startswith("impulses"):
                    impulses = [int(x) for x in body.split()[1:]]
                continue
            if line.strip():
                buf.append(line)
    for r in csv.DictReader(buf):
        rows.append({
            "tick": int(r["tick"]),
            "time": float(r["time_s"]),
            "err_rollback": float(r["err_rollback"]),
            "err_ignore": float(r["err_ignore"]),
            "err_snap": float(r["err_snap"]),
            "event": int(r["rollback_event"]),
        })
    return cfg, rows, summary, impulses


def nice_ticks(vmax, target=5):
    for step in (0.25, 0.5, 1.0, 1.5, 2.0, 2.5, 5.0, 10.0):
        if vmax / step <= target:
            out, v = [], 0.0
            while v <= vmax + 1e-9:
                out.append(round(v, 6))
                v += step
            return out
    return [0.0, vmax]


def fmt(v):
    if abs(v - round(v)) < 1e-9:
        return f"{int(round(v))}"
    return f"{v:.2f}".rstrip("0").rstrip(".")


def polyline(rows, key, sx, sy):
    pts = [(sx(r["tick"]), sy(r[key])) for r in rows]
    return "M " + " L ".join(f"{x:.2f} {y:.2f}" for x, y in pts)


def text(x, y, s, size=12, fill=INK_SECONDARY, anchor="start", weight=None,
         extra=""):
    w = f' font-weight="{weight}"' if weight else ""
    return (f'<text x="{x:.2f}" y="{y:.2f}" font-family="{FONT}" font-size="{size}" '
            f'fill="{fill}" text-anchor="{anchor}"{w}{extra}>{s}</text>')


def panel(out, rows, impulses, t0, t1, vmax, plot_y, plot_h, hz, title, subtitle):
    sx0, sx1 = PL, W - PR
    pw = sx1 - sx0
    ticks = nice_ticks(vmax)
    ymax = ticks[-1]

    def sx(tick):
        return sx0 + (tick - t0) / (t1 - t0) * pw

    def sy(v):
        return plot_y + plot_h - v / ymax * plot_h

    out.append(text(PL, plot_y - 34, title, size=13, fill=INK, weight=600))
    out.append(text(PL, plot_y - 16, subtitle, size=11.5, fill=MUTED))

    for v in ticks:
        y = sy(v)
        out.append(f'<line x1="{sx0}" y1="{y:.2f}" x2="{sx1}" y2="{y:.2f}" '
                   f'stroke="{GRID}" stroke-width="1"/>')
        out.append(text(sx0 - 10, y + 4, fmt(v), size=12, fill=MUTED, anchor="end",
                        extra=' style="font-variant-numeric:tabular-nums"'))

    span_s = (t1 - t0) / hz
    step_s = next((s for s in (0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0)
                   if span_s / s <= 9), span_s)
    tsec = (int(t0 / hz / step_s)) * step_s
    while tsec <= t1 / hz + 1e-9:
        x = sx(tsec * hz)
        out.append(f'<line x1="{x:.2f}" y1="{plot_y+plot_h}" x2="{x:.2f}" '
                   f'y2="{plot_y+plot_h+5}" stroke="{AXIS}" stroke-width="1"/>')
        out.append(text(x, plot_y + plot_h + 21, f"{tsec:g}", size=12, fill=MUTED,
                        anchor="middle", extra=' style="font-variant-numeric:tabular-nums"'))
        tsec += step_s
    out.append(text(sx1, plot_y + plot_h + 42, "time (s)", size=11.5, fill=MUTED,
                    anchor="end"))
    out.append(f'<line x1="{sx0}" y1="{plot_y+plot_h}" x2="{sx1}" '
               f'y2="{plot_y+plot_h}" stroke="{AXIS}" stroke-width="1"/>')

    for it in impulses:
        if t0 <= it <= t1:
            x = sx(it)
            out.append(f'<line x1="{x:.2f}" y1="{plot_y}" x2="{x:.2f}" '
                       f'y2="{plot_y+plot_h}" stroke="{AXIS}" stroke-width="1" '
                       f'stroke-dasharray="3 4"/>')

    for key in ("err_snap", "err_ignore", "err_rollback"):
        color, _ = SERIES[key]
        out.append(f'<path d="{polyline(rows, key, sx, sy)}" fill="none" stroke="{color}" '
                   f'stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>')
    return sx, sy, ymax


def render(cfg, rows, summary, impulses, out_path):
    hz = float(cfg.get("tick_hz", 20))
    tmax = max(r["tick"] for r in rows)
    vmax = max(max(r[k] for k in ORDER) for r in rows)

    s = []
    s.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
             f'viewBox="0 0 {W} {H}" role="img" '
             f'aria-label="Client prediction error over time">')
    s.append(f'<rect width="{W}" height="{H}" fill="{SURFACE}"/>')
    s.append(text(PL, 40, "预测误差在回滚后收敛到权威状态", size=20, fill=INK,
                  weight=600))
    s.append(text(PL, 63,
                  f"|drawn position - authoritative position| · {int(hz)} Hz tick · "
                  f"one-way {cfg.get('one_way_ms','?')} ms ±{cfg.get('jitter_ms','?')} ms · "
                  f"{cfg.get('loss_pct','0')}% loss · {cfg.get('dup_pct','0')}% duplicate · "
                  f"{cfg.get('impulses','?')} unpredicted impulses",
                  size=12.5, fill=INK_SECONDARY))

    lx = PL
    for key in ORDER:
        color, label = SERIES[key]
        s.append(f'<line x1="{lx:.1f}" y1="98" x2="{lx+18:.1f}" y2="98" '
                 f'stroke="{color}" stroke-width="2" stroke-linecap="round"/>')
        s.append(text(lx + 24, 102, label, size=12, fill=INK_SECONDARY))
        lx += 24 + len(label) * 6.6 + 24

    # ---- top panel: the whole match --------------------------------------
    top_y, top_h = 172, 300
    sx, sy, _ = panel(s, rows, impulses, 0, tmax, vmax, top_y, top_h, hz,
                      "整场对局 · full match",
                      f"每段抬升对应一次不可预测的击退；回滚客户端在 {summary.get('max_recovery_ticks','?')} 个 tick 内回到 0")

    last = rows[-1]
    ends = sorted(((k, sy(last[k])) for k in ORDER), key=lambda kv: kv[1])
    placed = []
    for key, y in ends:
        ly = y if not placed or y - placed[-1] >= 18 else placed[-1] + 18
        placed.append(ly)
        color, _ = SERIES[key]
        s.append(f'<circle cx="{sx(last["tick"]):.2f}" cy="{y:.2f}" r="4" fill="{color}" '
                 f'stroke="{SURFACE}" stroke-width="2"/>')
        s.append(text(sx(last["tick"]) + 12, ly + 4, fmt(last[key]), size=12,
                      fill=INK_SECONDARY,
                      extra=' style="font-variant-numeric:tabular-nums"'))
    s.append(f'<text x="18" y="{top_y+top_h/2}" transform="rotate(-90 18 {top_y+top_h/2})" '
             f'text-anchor="middle" font-family="{FONT}" font-size="12" '
             f'fill="{MUTED}">position error (units)</text>')

    # ---- bottom panel: zoom on the first impulse -------------------------
    if impulses:
        first = impulses[0]
        z0 = max(0, first - 3)
        z1 = min(tmax, first + 11)
        zrows = [r for r in rows if z0 <= r["tick"] <= z1]
        zvmax = max(max(r[k] for k in ORDER) for r in zrows)
        rbmax = max(r["err_rollback"] for r in zrows)
        if zvmax > max(1.0, rbmax * 3):
            zvmax = max(0.75, rbmax * 1.6)
        zy, zh = 618, 104
        zsx, zsy, zmax = panel(s, zrows, impulses, z0, z1, zvmax, zy, zh, hz,
                               f"放大 · first impulse at tick {first}（每 tick 一点）",
                               "回滚在确认权威点的同一 tick 把状态钉回权威值，此后误差保持 0")

        for key in ("err_ignore", "err_rollback"):
            color, _ = SERIES[key]
            for r in zrows:
                s.append(f'<circle cx="{zsx(r["tick"]):.2f}" cy="{zsy(r[key]):.2f}" '
                         f'r="2.6" fill="{color}" stroke="{SURFACE}" stroke-width="1"/>')

        ev = [r for r in zrows if r["event"]]
        if ev:
            e = ev[0]
            x, y = zsx(e["tick"]), zsy(e["err_rollback"])
            s.append(f'<line x1="{x:.2f}" y1="{y+6:.2f}" x2="{x:.2f}" y2="{y+26:.2f}" '
                     f'stroke="{MUTED}" stroke-width="1"/>')
            s.append(text(x, y + 40, f"rollback @ tick {e['tick']}", size=10.5,
                          fill=MUTED, anchor="middle"))
        s.append(f'<line x1="{PL}" y1="{zsy(0):.2f}" x2="{W-PR}" y2="{zsy(0):.2f}" '
                 f'stroke="{AXIS}" stroke-width="1"/>')
        s.append(text(W - PR + 12, zsy(0) + 4, "0", size=11, fill=MUTED))

    # ---- footer -----------------------------------------------------------
    foot = (f"rollbacks {summary.get('rollbacks','?')} · "
            f"replay mismatch {summary.get('mismatch','?')} "
            f"(bit-exact {summary.get('accepted','?')}/{summary.get('replays_checked','?')}) · "
            f"replayed ticks {summary.get('replayed_ticks','?')} · "
            f"max rollback depth {summary.get('depth_max','?')} · "
            f"max recovery {summary.get('max_recovery_ticks','?')} ticks "
            f"({int(summary.get('max_recovery_ticks','0')) / hz:.2f} s) · "
            f"ignore-client error at end {summary.get('err_ignore_end','?')}")
    s.append(text(PL, H - 22, foot, size=11.5, fill=MUTED))

    s.append("</svg>")
    Path(out_path).write_text("\n".join(s) + "\n")


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: chart.py metrics.csv metrics.svg")
    cfg, rows, summary, impulses = parse(sys.argv[1])
    render(cfg, rows, summary, impulses, sys.argv[2])
    print(f"wrote {sys.argv[2]} ({len(rows)} ticks)")


if __name__ == "__main__":
    main()
