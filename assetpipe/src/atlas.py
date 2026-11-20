"""货架（shelf）装箱器与图集构建，与 fontatlas 模块的装箱思路一致。

排序与放置规则全部确定化：先按高降序（高相同按宽降序、再按名），
同高度图片从左到右排一行，放不下换下一货架。
"""
import pngcodec
from pipecommon import AtlasOverflow


def pack(items, atlas_w, atlas_h, padding):
    """items: [(name, w, h)]；返回 [(name, x, y, w, h)]，失败抛 AtlasOverflow。"""
    ordered = sorted(items, key=lambda t: (-t[2], -t[1], t[0]))
    placed = []
    used_h = 0
    shelf_x = 0
    shelf_h = 0
    for name, w, h in ordered:
        if w > atlas_w or h > atlas_h:
            raise AtlasOverflow(
                "图片 %s 尺寸 %dx%d 超过图集 %dx%d"
                % (name, w, h, atlas_w, atlas_h))
        if shelf_x + w > atlas_w:
            used_h += shelf_h
            shelf_x = 0
            shelf_h = 0
        if used_h + h > atlas_h:
            raise AtlasOverflow(
                "图集 %dx%d 装不下，%s 放置后超出底边（已用高度 %d）"
                % (atlas_w, atlas_h, name, used_h))
        placed.append((name, shelf_x, used_h, w, h))
        shelf_x += w + padding
        if h > shelf_h:
            shelf_h = h
    return placed


def build_atlas(sources, atlas_w, atlas_h, padding=1):
    """sources: [(逻辑名, rgba 图像)]；返回 (canvas, 名称->矩形 字典)。"""
    items = [(name, img["w"], img["h"]) for name, img in sources]
    placements = pack(items, atlas_w, atlas_h, padding)
    canvas = pngcodec.new_canvas(atlas_w, atlas_h, "RGBA")
    rects = {}
    by_name = {name: img for name, img in sources}
    for name, x, y, w, h in placements:
        pngcodec.blit(canvas, by_name[name], x, y)
        rects[name] = {"x": x, "y": y, "w": w, "h": h}
    return canvas, rects
