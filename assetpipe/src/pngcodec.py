"""确定性 PNG 读写与基础像素运算，仅依赖标准库。

读：8 位灰度 / RGB / RGBA、非隔行 PNG；其余格式显式报错。
写：固定过滤类型 0、固定 zlib 参数，同输入永远产生同字节。
"""
import struct
import zlib

from pipecommon import UnsupportedFormat

PNG_SIG = b"\x89PNG\r\n\x1a\n"

# color type -> (通道数, 模式名)
_COLOR_TYPES = {0: (1, "G"), 2: (3, "RGB"), 6: (4, "RGBA")}


def _paeth(a, b, c):
    p = a + b - c
    pa = abs(p - a)
    pb = abs(p - b)
    pc = abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def _unfilter(raw, width, height, ch):
    stride = width * ch
    out = bytearray(stride * height)
    prev = bytearray(stride)
    pos = 0
    for y in range(height):
        ftype = raw[pos]
        pos += 1
        line = bytearray(raw[pos:pos + stride])
        pos += stride
        if ftype == 1:  # sub
            for i in range(ch, stride):
                line[i] = (line[i] + line[i - ch]) & 0xFF
        elif ftype == 2:  # up
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif ftype == 3:  # average
            for i in range(stride):
                left = line[i - ch] if i >= ch else 0
                line[i] = (line[i] + ((left + prev[i]) >> 1)) & 0xFF
        elif ftype == 4:  # paeth
            for i in range(stride):
                left = line[i - ch] if i >= ch else 0
                up_left = prev[i - ch] if i >= ch else 0
                line[i] = (line[i] + _paeth(left, prev[i], up_left)) & 0xFF
        elif ftype != 0:
            raise ValueError("不支持的 PNG 过滤类型 %d" % ftype)
        out[y * stride:(y + 1) * stride] = line
        prev = line
    return bytes(out)


def read_png(path):
    """返回 {w, h, mode, ch, pixels}；失败抛 UnsupportedFormat(path, ...)。"""
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError as e:
        raise UnsupportedFormat("无法读取 %s: %s" % (path, e))

    if not data.startswith(PNG_SIG):
        raise UnsupportedFormat("%s 不是 PNG 文件（签名不匹配）" % path)

    pos = len(PNG_SIG)
    width = height = bit_depth = color_type = interlace = None
    idat = bytearray()
    saw_ihdr = saw_iend = False
    while pos + 8 <= len(data):
        length, ctype = struct.unpack(">I4s", data[pos:pos + 8])
        pos += 8
        if pos + length + 4 > len(data):
            raise UnsupportedFormat("%s 的 %s 块被截断" % (path, ctype))
        body = data[pos:pos + length]
        pos += length + 4  # 跳过 CRC
        if ctype == b"IHDR":
            if saw_ihdr:
                raise UnsupportedFormat("%s 含多个 IHDR 块" % path)
            saw_ihdr = True
            (width, height, bit_depth, color_type,
             compression, filt, interlace) = struct.unpack(">IIBBBBB", body)
            if compression != 0 or filt != 0:
                raise UnsupportedFormat("%s 使用了不支持的压缩/过滤方式" % path)
        elif ctype == b"IDAT":
            idat += body
        elif ctype == b"IEND":
            saw_iend = True
            break
    if not saw_ihdr:
        raise UnsupportedFormat("%s 缺少 IHDR 块" % path)
    if not saw_iend:
        raise UnsupportedFormat("%s 缺少 IEND 块" % path)
    if bit_depth != 8:
        raise UnsupportedFormat(
            "%s 位深为 %d，仅支持 8 位 PNG" % (path, bit_depth))
    if interlace != 0:
        raise UnsupportedFormat("%s 使用隔行扫描，仅支持非隔行 PNG" % path)
    if color_type not in _COLOR_TYPES:
        raise UnsupportedFormat(
            "%s 的色彩类型 %d 不受支持（仅灰度/RGB/RGBA）" % (path, color_type))

    ch, mode = _COLOR_TYPES[color_type]
    try:
        raw = zlib.decompress(bytes(idat))
    except zlib.error as e:
        raise UnsupportedFormat("%s 的 IDAT 解压失败: %s" % (path, e))
    expect = height * (1 + width * ch)
    if len(raw) != expect:
        raise UnsupportedFormat(
            "%s 解压后长度 %d 与期望 %d 不符，文件已损坏" % (path, len(raw), expect))
    pixels = _unfilter(raw, width, height, ch)
    return {"w": width, "h": height, "mode": mode, "ch": ch, "pixels": pixels}


def _chunk(ctype, body):
    out = struct.pack(">I", len(body)) + ctype + body
    out += struct.pack(">I", zlib.crc32(ctype + body) & 0xFFFFFFFF)
    return out


def write_png(path, width, height, mode, pixels, compress_level=9):
    """确定性写 PNG：仅 IHDR/IDAT/IEND，逐行过滤类型 0，固定 zlib 参数。"""
    ch = {"G": 1, "RGB": 3, "RGBA": 4}[mode]
    if len(pixels) != width * height * ch:
        raise ValueError("像素缓冲长度与尺寸不符: %s" % path)
    color_type = {1: 0, 3: 2, 4: 6}[ch]
    ihdr = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)

    stride = width * ch
    raw = bytearray()
    for y in range(height):
        raw.append(0)  # 过滤类型 0：None
        raw += pixels[y * stride:(y + 1) * stride]

    # memLevel/wbits 固定，压缩结果不受环境影响
    compressor = zlib.compressobj(
        compress_level, zlib.DEFLATED, 15, 8, zlib.Z_DEFAULT_STRATEGY)
    blob = compressor.compress(bytes(raw)) + compressor.flush()

    with open(path, "wb") as f:
        f.write(PNG_SIG)
        f.write(_chunk(b"IHDR", ihdr))
        f.write(_chunk(b"IDAT", blob))
        f.write(_chunk(b"IEND", b""))


def to_rgba(img):
    """任意输入图像统一转成 RGBA。"""
    if img["mode"] == "RGBA":
        return dict(img, pixels=bytes(img["pixels"]))
    src, w, h, ch = img["pixels"], img["w"], img["h"], img["ch"]
    out = bytearray(w * h * 4)
    if ch == 1:  # 灰度
        for i in range(w * h):
            v = src[i]
            out[4 * i:4 * i + 3] = bytes((v, v, v))
            out[4 * i + 3] = 255
    else:  # RGB
        for i in range(w * h):
            out[4 * i:4 * i + 3] = src[3 * i:3 * i + 3]
            out[4 * i + 3] = 255
    return {"w": w, "h": h, "mode": "RGBA", "ch": 4, "pixels": bytes(out)}


def to_gray(img):
    """RGBA/RGB 转 8 位灰度，整数加权（Rec.601 权重），确定性舍入。"""
    if img["mode"] == "G":
        return dict(img, pixels=bytes(img["pixels"]))
    src, w, h, ch = img["pixels"], img["w"], img["h"], img["ch"]
    out = bytearray(w * h)
    for i in range(w * h):
        r = src[ch * i]
        g = src[ch * i + 1]
        b = src[ch * i + 2]
        out[i] = (299 * r + 587 * g + 114 * b) // 1000
    return {"w": w, "h": h, "mode": "G", "ch": 1, "pixels": bytes(out)}


def resize_box(img, dst_w, dst_h):
    """整数盒平均缩放：目标像素取对应源矩形各通道平均值，纯整数运算。"""
    if dst_w <= 0 or dst_h <= 0:
        raise ValueError("缩放目标尺寸必须为正数")
    src, w, h, ch = img["pixels"], img["w"], img["h"], img["ch"]
    if (dst_w, dst_h) == (w, h):
        return dict(img, pixels=bytes(src))
    if dst_w > w or dst_h > h:
        raise ValueError("仅支持向下缩放: %dx%d -> %dx%d" % (w, h, dst_w, dst_h))
    out = bytearray(dst_w * dst_h * ch)
    for dy in range(dst_h):
        sy0 = dy * h // dst_h
        sy1 = (dy + 1) * h // dst_h
        for dx in range(dst_w):
            sx0 = dx * w // dst_w
            sx1 = (dx + 1) * w // dst_w
            area = (sx1 - sx0) * (sy1 - sy0)
            base = (dy * dst_w + dx) * ch
            for c in range(ch):
                total = 0
                for sy in range(sy0, sy1):
                    row = sy * w + sx0
                    for sx in range(sx0, sx1):
                        total += src[(row + (sx - sx0)) * ch + c]
                out[base + c] = total // area
    return {"w": dst_w, "h": dst_h, "mode": img["mode"], "ch": ch,
            "pixels": bytes(out)}


def new_canvas(width, height, mode):
    ch = {"G": 1, "RGB": 3, "RGBA": 4}[mode]
    return {"w": width, "h": height, "mode": mode, "ch": ch,
            "pixels": bytes(width * height * ch)}


def blit(dst, src, x, y):
    """把 src（必须与 dst 同模式）贴到 dst 的 (x,y)，越界即报错。"""
    if dst["mode"] != src["mode"]:
        raise ValueError("blit 模式不一致: %s vs %s" % (dst["mode"], src["mode"]))
    ch = dst["ch"]
    w, h, dw, dh = src["w"], src["h"], dst["w"], dst["h"]
    if x < 0 or y < 0 or x + w > dw or y + h > dh:
        raise ValueError("blit 越界: (%d,%d) %dx%d 贴入 %dx%d"
                         % (x, y, w, h, dw, dh))
    sp, dp = src["pixels"], dst["pixels"]
    for row in range(h):
        si = row * w * ch
        di = ((y + row) * dw + x) * ch
        # bytearray 切片赋值
        dst_pixels = bytearray(dp) if isinstance(dp, bytes) else dp
        dst_pixels[di:di + w * ch] = sp[si:si + w * ch]
        dp = bytes(dst_pixels)
    dst["pixels"] = dp
