#!/usr/bin/env python3
"""Minimal PPM(P6) -> PNG converter using only the stdlib (zlib/struct).

Usage: ppm2png.py input.ppm output.png
Used to turn the compositor's pixel-exact PPM screenshots into viewable
PNGs without pulling in third-party image libraries. The PPM remains the
authoritative byte-for-byte comparison artifact; this only re-encodes.
"""
import struct
import sys
import zlib


def read_ppm(path):
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P6":
            raise SystemExit(f"not a P6 PPM: {path}")
        # skip comments / blank header tokens
        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if line.startswith(b"#"):
                continue
            tokens.extend(line.split())
        w, h, maxv = (int(t) for t in tokens[:3])
        if maxv != 255:
            raise SystemExit("only maxval 255 supported")
        data = f.read(w * h * 3)
    return w, h, data


def write_png(path, w, h, rgb):
    def chunk(tag, payload):
        c = tag + payload
        return struct.pack(">I", len(payload)) + c + struct.pack(">I", zlib.crc32(c))

    raw = bytearray()
    stride = w * 3
    for y in range(h):
        raw.append(0)  # filter type 0 (none) per scanline
        raw.extend(rgb[y * stride:(y + 1) * stride])
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)  # 8-bit RGB
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", ihdr)
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 9))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: ppm2png.py in.ppm out.png")
    w, h, data = read_ppm(sys.argv[1])
    write_png(sys.argv[2], w, h, data)
    print(f"wrote {sys.argv[2]} {w}x{h}")


if __name__ == "__main__":
    main()
