#!/usr/bin/env python3
"""Compose the six comparison images and the profile chart from the PGM/csv
output of `./atten`.

Outputs (in out/):
  compare_group1.png .. compare_group3.png   screen vs world, one per rack
  compare_six.png                             all six in one 3x2 grid
  profile.png                                 attenuation curves + crossing
"""
import csv
import os
import sys

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "out")
FONT_CJK = "/home/ys/.fonts/NotoSansCJKsc-Regular.otf"
FONT_CJK_BOLD = "/home/ys/.fonts/NotoSansCJKsc-Bold.otf"
FONT_MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"

SURFACE = (252, 252, 251)
INK = (11, 11, 11)
INK_2 = (82, 81, 78)
INK_MUTED = (128, 127, 123)
RULE = (228, 227, 223)
RULE_AXIS = (180, 179, 175)
BAND = (243, 242, 238)
SCREEN_C = (42, 120, 214)   # categorical slot 1
WORLD_C = (235, 104, 52)    # categorical slot 2

MODEL_CN = {"screen": "屏幕空间衰减（原实现）", "world": "世界空间距离衰减（修正）"}
MODEL_EN = {"screen": "screen-space  1 − smoothstep(d / R)",
            "world": "world-space  1 / (1 + (d / d0)²)"}

GROUPS = [
    ("group1", "第 1 组", "12 m"),
    ("group2", "第 2 组", "32 m"),
    ("group3", "第 3 组", "52 m"),
]

PANEL_SCALE = 2
PANEL_PAD = 16
CAPTION_H = 62


def read_pgm(path):
    with open(path, "rb") as f:
        if f.readline().strip() != b"P5":
            raise ValueError(f"{path}: not a binary PGM")
        w, h = map(int, f.readline().split())
        f.readline()  # maxval
        data = f.read(w * h)
    return Image.frombytes("L", (w, h), data)


def font(path, size):
    return ImageFont.truetype(path, size)


def read_csv():
    rows = []
    with open(os.path.join(OUT, "profile.csv"), newline="") as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    return rows


def read_summary():
    vals = {}
    with open(os.path.join(OUT, "summary.txt"), newline="") as f:
        for line in f:
            k, _, v = line.strip().partition("=")
            vals[k] = float(v)
    return vals


def interp(rows, key, d):
    prev = rows[0]
    for r in rows:
        if r["d_m"] >= d:
            if r["d_m"] == prev["d_m"]:
                return r[key]
            u = (d - prev["d_m"]) / (r["d_m"] - prev["d_m"])
            return prev[key] + u * (r[key] - prev[key])
        prev = r
    return rows[-1][key]


def encode(linear):
    return round(max(0.0, min(1.0, linear)) ** (1.0 / 2.2) * 255)


def crop_panel(key, which):
    """Framed crop with a caption band: model name + the two sampled grays."""
    crop = read_pgm(os.path.join(OUT, f"{key}_{which}.pgm"))
    crop = crop.resize((crop.width * PANEL_SCALE, crop.height * PANEL_SCALE),
                       Image.LANCZOS)
    w = crop.width + 2 * PANEL_PAD
    h = crop.height + 2 * PANEL_PAD + CAPTION_H
    img = Image.new("RGB", (w, h), SURFACE)
    img.paste(crop.convert("RGB"), (PANEL_PAD, PANEL_PAD))
    d = ImageDraw.Draw(img)
    d.rectangle((PANEL_PAD, PANEL_PAD,
                 PANEL_PAD + crop.width - 1, PANEL_PAD + crop.height - 1),
                outline=RULE)
    d.rectangle((PANEL_PAD, PANEL_PAD, PANEL_PAD + 4, PANEL_PAD + crop.height - 1),
                fill=SCREEN_C if which == "screen" else WORLD_C)
    return img


def caption(img, x, y, cn, en, detail):
    d = ImageDraw.Draw(img)
    d.text((x, y), cn, font=font(FONT_CJK_BOLD, 19), fill=INK)
    d.text((x, y + 26), en, font=font(FONT_MONO, 12), fill=INK_MUTED)
    if detail:
        d.text((x, y + 42), detail, font=font(FONT_CJK, 13), fill=INK_2)


def panel_with_caption(key, which, rack_dist):
    img = crop_panel(key, which)
    pole = encode(0.95 * interp(READ.rows, "screen_gain" if which == "screen"
                                else "world_gain", rack_dist))
    bg = encode(0.30 * interp(READ.rows, "screen_gain" if which == "screen"
                              else "world_gain", rack_dist + 6.0))
    img.info["detail"] = f"杆心灰度 {pole} · 裁切右缘地面 {bg}"
    return img


class _Read:
    rows = None


READ = _Read()


def compose_group(idx):
    key, cn, en = GROUPS[idx]
    dist = float(en.split()[0])
    panels = [panel_with_caption(key, "screen", dist),
              panel_with_caption(key, "world", dist)]
    gap = 24
    header = 128
    footer = 46
    w = panels[0].width + panels[1].width + gap + 2 * 36
    h = header + panels[0].height + 2 * 18 + footer
    img = Image.new("RGB", (w, h), SURFACE)
    d = ImageDraw.Draw(img)

    title = font(FONT_CJK_BOLD, 28)
    sub = font(FONT_CJK, 16)
    small = font(FONT_CJK, 13)
    d.text((36, 20), f"{cn} · 距相机 {en}  两种衰减模型对比",
           font=title, fill=INK)
    d.text((36, 58), "俯视正交相机 · 5 根等距标杆 · 灰度 = 线性光强经 2.2 γ 编码",
           font=sub, fill=INK_2)
    for i, which in enumerate(("screen", "world")):
        x = 36 + i * (panels[0].width + gap) + 2
        d.ellipse((x, 97, x + 11, 108),
                  fill=SCREEN_C if which == "screen" else WORLD_C)
        d.text((x + 17, 92), MODEL_CN[which], font=small, fill=INK_2)

    img.paste(panels[0], (36, header))
    img.paste(panels[1], (36 + panels[0].width + gap, header))
    for i, which in enumerate(("screen", "world")):
        x = 36 + i * (panels[0].width + gap) + PANEL_PAD
        y = header + panels[0].height - CAPTION_H + 4
        d.text((x, y), MODEL_CN[which], font=font(FONT_CJK_BOLD, 18), fill=INK)
        d.text((x, y + 24), MODEL_EN[which], font=font(FONT_MONO, 12), fill=INK_MUTED)
        d.text((x, y + 40), panels[i].info["detail"], font=font(FONT_CJK, 13), fill=INK_2)

    rows = READ.rows
    ps = interp(rows, "screen_gain", dist)
    pw_ = interp(rows, "world_gain", dist)
    rel = (pw_ - ps) / ps
    if dist <= READ.summary["near_limit_m"]:
        note = (f"同组两图几乎一致：距相机 {dist:.0f} m 仍在近场（≤ 20 m）内，"
                f"世界空间比屏幕空间亮 {rel*100:.2f}%。")
    elif dist < READ.summary["five_pct_m"]:
        note = (f"已越过交叉点 {READ.summary['cross_m']:.2f} m：世界空间比屏幕空间亮 "
                f"{rel*100:.1f}%，尚未到 5% 判据线。")
    else:
        note = (f"越过交叉点后偏差持续放大：世界空间比屏幕空间亮 {rel*100:.1f}%，"
                f"屏幕空间的衰减已明显失真。")
    d.text((36, h - footer + 6), note, font=small, fill=INK_MUTED)

    out = os.path.join(OUT, f"compare_{key}.png")
    img.save(out)
    return img


def compose_six(panels_by_group):
    rows_n = len(panels_by_group)
    gap_x, gap_y = 26, 30
    gutter = 132
    header = 118
    col_w = max(p.width for row in panels_by_group for p in row)
    row_h = max(p.height for row in panels_by_group for p in row)
    left = gutter + 18
    w = left + 2 * col_w + gap_x + 36
    h = header + rows_n * row_h + (rows_n - 1) * gap_y + 44
    img = Image.new("RGB", (w, h), SURFACE)
    d = ImageDraw.Draw(img)

    d.text((36, 20), "三组标杆 · 屏幕空间 vs 世界空间", font=font(FONT_CJK_BOLD, 28),
           fill=INK)
    d.text((36, 58), "同一场景各渲染两次；每组两格的杆心灰度差即两种模型的偏差。",
           font=font(FONT_CJK, 16), fill=INK_2)
    for i, which in enumerate(("screen", "world")):
        x = left + i * (col_w + gap_x) + 2
        d.ellipse((x, 94, x + 11, 105),
                  fill=SCREEN_C if which == "screen" else WORLD_C)
        d.text((x + 17, 89), MODEL_CN[which], font=font(FONT_CJK, 14), fill=INK_2)

    for r, row in enumerate(panels_by_group):
        y = header + r * (row_h + gap_y)
        cn, dist = GROUPS[r][1], GROUPS[r][2]
        gx = 36
        d.text((gx, y + row_h // 2 - 44), cn, font=font(FONT_CJK_BOLD, 18), fill=INK)
        d.text((gx, y + row_h // 2 - 18), dist, font=font(FONT_MONO, 16), fill=INK_2)
        d.text((gx, y + row_h // 2 + 8), "离相机", font=font(FONT_CJK, 12), fill=INK_MUTED)
        for c, p in enumerate(row):
            img.paste(p, (left + c * (col_w + gap_x), y))

    out = os.path.join(OUT, "compare_six.png")
    img.save(out)
    return img


def compose_profile():
    rows = READ.rows
    S_ = READ.summary
    S = 2
    W = 1400 * S
    # top panel: the two gains. bottom panel: their relative difference,
    # sharing the same x axis, so the 5 % criterion is read off directly.
    ml, mr = 112 * S, 330 * S
    pw = W - ml - mr
    top_mt, top_h = 118 * S, 364 * S
    bot_mt, bot_h = top_mt + top_h + 84 * S, 196 * S
    H = bot_mt + bot_h + 92 * S
    x_max = 70.0
    y_max = 1.05
    err_hi, err_lo = 0.40, -1.04

    def X(d):
        return ml + d / x_max * pw

    def Y(a):
        return top_mt + (1.0 - a / y_max) * top_h

    def E(v):
        return bot_mt + (err_hi - v) / (err_hi - err_lo) * bot_h

    img = Image.new("RGB", (W, H), SURFACE)
    d = ImageDraw.Draw(img)

    f_tick = font(FONT_MONO, 15 * S)
    f_axis = font(FONT_CJK, 16 * S)
    f_note = font(FONT_CJK_BOLD, 15 * S)
    f_note2 = font(FONT_MONO, 13 * S)
    f_small = font(FONT_CJK, 13 * S)

    # near-field acceptance band, behind both panels
    d.rectangle((X(0), top_mt, X(S_["near_limit_m"]), bot_mt + bot_h), fill=BAND)
    d.text((X(0) + 12 * S, Y(0.58)),
           f"近场 ≤ {S_['near_limit_m']:.0f} m\n最大偏差 {S_['near_max_err_pct']:.2f}%",
           font=f_small, fill=INK_MUTED, spacing=6 * S)

    for t in range(0, 71, 10):
        x = X(t)
        d.line((x, top_mt, x, top_mt + top_h), fill=RULE, width=1 * S)
        d.line((x, bot_mt, x, bot_mt + bot_h), fill=RULE, width=1 * S)
    for t in range(0, 11, 2):
        a = t / 10.0
        y = Y(a)
        d.line((ml, y, ml + pw, y), fill=RULE, width=1 * S)
        lab = f"{a:.1f}"
        tw = d.textlength(lab, font=f_tick)
        d.text((ml - 16 * S - tw, y - 9 * S), lab, font=f_tick, fill=INK_MUTED)
    for t in (20, 0, -20, -40, -60, -80, -100):
        y = E(t / 100.0)
        d.line((ml, y, ml + pw, y), fill=RULE, width=1 * S)
        lab = f"{t:+d}%" if t else "0"
        tw = d.textlength(lab, font=f_tick)
        d.text((ml - 16 * S - tw, y - 9 * S), lab, font=f_tick, fill=INK_MUTED)
    d.line((ml, top_mt + top_h, ml + pw, top_mt + top_h), fill=RULE_AXIS, width=1 * S)
    d.line((ml, top_mt, ml, top_mt + top_h), fill=RULE_AXIS, width=1 * S)
    d.line((ml, bot_mt + bot_h, ml + pw, bot_mt + bot_h), fill=RULE_AXIS, width=1 * S)
    d.line((ml, bot_mt, ml, bot_mt + bot_h), fill=RULE_AXIS, width=1 * S)

    # rack distances on the shared x axis
    for (_, cn, dist) in GROUPS:
        gx = X(float(dist.split()[0]))
        d.line((gx, bot_mt + bot_h, gx, bot_mt + bot_h + 7 * S), fill=RULE_AXIS, width=1 * S)
        lab = cn.replace(" ", "")
        tw = d.textlength(lab, font=f_small)
        d.text((gx - tw / 2, bot_mt + bot_h + 12 * S), lab, font=f_small, fill=INK_MUTED)

    xl = "到相机中心的距离 d（米）"
    d.text((ml + pw / 2 - d.textlength(xl, font=f_axis) / 2, bot_mt + bot_h + 42 * S),
           xl, font=f_axis, fill=INK_2)
    d.text((ml - 76 * S, top_mt - 30 * S), "增益", font=f_axis, fill=INK_2)
    d.text((ml - 34 * S, bot_mt - 28 * S), "相对偏差", font=f_axis, fill=INK_2)

    def poly(col, color, y_of):
        pts = [(X(r["d_m"]), y_of(r[col])) for r in rows]
        d.line(pts, fill=color, width=2 * S, joint="curve")

    poly("screen_gain", SCREEN_C, Y)
    poly("world_gain", WORLD_C, Y)
    poly("rel_error_pct", INK_2, lambda p: E(p / 100.0))

    # zero line and the +/-5 % limits in the error panel
    d.line((ml, E(0.0), ml + pw, E(0.0)), fill=RULE_AXIS, width=1 * S)
    for sign, lab, dy in ((1, "+5%", -18), (-1, "−5%", 4)):
        y = E(sign * 0.05)
        d.line((ml, y, ml + pw, y), fill=(214, 168, 160), width=1 * S)
        tw = d.textlength(lab, font=f_note2)
        d.text((ml + pw - tw - 4 * S, y + dy * S), lab, font=f_note2, fill=(150, 90, 78))

    # crossing: dot with a surface ring, leader down to the zero line
    cd = S_["cross_m"]
    ca = interp(rows, "world_gain", cd)
    cx, cy = X(cd), Y(ca)
    rr = 7 * S
    d.ellipse((cx - rr - 2 * S, cy - rr - 2 * S, cx + rr + 2 * S, cy + rr + 2 * S),
              fill=SURFACE)
    d.ellipse((cx - rr, cy - rr, cx + rr, cy + rr), fill=INK_2)
    d.line((cx, cy + rr, cx, E(0.0)), fill=(160, 159, 155), width=1 * S)
    tx = cx + 18 * S
    d.text((tx, cy - 62 * S), f"交叉点 d = {cd:.2f} m", font=f_note, fill=INK)
    d.text((tx, cy - 40 * S), f"两边增益 = {ca:.3f}", font=f_small, fill=INK_2)
    d.ellipse((cx - 4 * S, E(0.0) - 4 * S, cx + 4 * S, E(0.0) + 4 * S), fill=INK_2)

    # 5 % departure from the world curve
    fd = S_["five_pct_m"]
    fx, fy = X(fd), E(interp(rows, "rel_error_pct", fd) / 100.0)
    d.ellipse((fx - 5 * S, fy - 5 * S, fx + 5 * S, fy + 5 * S), fill=SCREEN_C)
    lab = f"−5% @ {fd:.1f} m"
    d.text((fx + 10 * S, fy - 26 * S), lab, font=f_small, fill=INK_2)

    # the error curve leaves the panel at the far end; say where it is heading
    d.text((X(66), E(-0.80)), "→ −100%", font=f_note2, fill=INK_MUTED)

    # legend block in the right margin
    lx, ly = ml + pw + 28 * S, top_mt + 4 * S
    for i, (col, title, formula) in enumerate((
            (SCREEN_C, "屏幕空间衰减", "1 − smoothstep(d / R)"),
            (WORLD_C, "世界空间距离衰减", "1 / (1 + (d / d0)^2)"))):
        y = ly + i * 76 * S
        d.line((lx, y + 8 * S, lx + 28 * S, y + 8 * S), fill=col, width=2 * S)
        d.text((lx + 38 * S, y - 3 * S), title, font=font(FONT_CJK, 16 * S), fill=INK)
        d.text((lx + 38 * S, y + 22 * S), formula, font=f_note2, fill=INK_MUTED)
    y = ly + 168 * S
    for line in (f"R  = 半视场对角线 = {S_['R_m']:.2f} m",
                 f"d0 = R / √3       = {S_['d0_m']:.2f} m",
                 f"视口 {int(S_['viewport_w_px'])}×{int(S_['viewport_h_px'])} px"
                 f" @ {S_['px_per_m']:.0f} px/m"):
        d.text((lx, y), line, font=f_small, fill=INK_2)
        y += 24 * S

    d.text((ml, 26 * S), "水平剖面亮度曲线：两种衰减在同一坐标系",
           font=font(FONT_CJK_BOLD, 29 * S), fill=INK)
    d.text((ml, 64 * S),
           "近处两条曲线几乎重合，交点之后分离；下方面板是同一条剖面的相对偏差，"
           "灰带内（≤ 20 m）未超过 5%。",
           font=font(FONT_CJK, 16 * S), fill=INK_2)

    img = img.resize((W // S, H // S), Image.LANCZOS)
    out = os.path.join(OUT, "profile.png")
    img.save(out)
    return out


def main():
    READ.rows = read_csv()
    READ.summary = read_summary()
    groups = []
    for i in range(3):
        groups.append(compose_group(i))
    keys = [GROUPS[i][0] for i in range(3)]
    dists = [float(GROUPS[i][2].split()[0]) for i in range(3)]
    panels = [[panel_with_caption(k, w, d) for w in ("screen", "world")]
              for k, d in zip(keys, dists)]
    compose_six(panels)
    compose_profile()
    print("wrote compare_group1..3.png, compare_six.png, profile.png in", OUT)


if __name__ == "__main__":
    sys.exit(main())
