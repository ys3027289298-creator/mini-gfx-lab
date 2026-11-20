"""确定性 WAV 读写与整数域重采样/降混，仅支持未压缩 PCM。

所有样本以有符号整数保存，运算全程整数，避免浮点平台差异。
"""
import struct

from pipecommon import UnsupportedFormat


def read_wav(path):
    """返回 {rate, channels, bits, samples(list[int] 交错存储)}。"""
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError as e:
        raise UnsupportedFormat("无法读取 %s: %s" % (path, e))

    if len(data) < 12 or data[:4] != b"RIFF" or data[8:12] != b"WAVE":
        raise UnsupportedFormat("%s 不是 WAV(RIFF/WAVE) 文件" % path)

    pos = 12
    fmt = None
    pcm = None
    while pos + 8 <= len(data):
        cid = data[pos:pos + 4]
        size = struct.unpack("<I", data[pos + 4:pos + 8])[0]
        body_start = pos + 8
        body = data[body_start:body_start + size]
        if cid == b"fmt ":
            if len(body) < 16:
                raise UnsupportedFormat("%s 的 fmt 块过短" % path)
            tag, channels, rate, _, _, bits = struct.unpack("<HHIIHH", body[:16])
            fmt = (tag, channels, rate, bits)
        elif cid == b"data":
            pcm = body
        pos = body_start + size + (size & 1)  # 块按偶数字节对齐

    if fmt is None:
        raise UnsupportedFormat("%s 缺少 fmt 块" % path)
    if pcm is None:
        raise UnsupportedFormat("%s 缺少 data 块" % path)

    tag, channels, rate, bits = fmt
    if tag != 1:
        raise UnsupportedFormat(
            "%s 格式标签 %d 非 PCM（压缩 WAV 不受支持）" % (path, tag))
    if bits not in (8, 16, 24, 32):
        raise UnsupportedFormat(
            "%s 位深 %d 不受支持（仅 8/16/24/32 位 PCM）" % (path, bits))
    if channels < 1:
        raise UnsupportedFormat("%s 声道数 %d 非法" % (path, channels))

    frame_bytes = channels * bits // 8
    n_frames = len(pcm) // frame_bytes
    if len(pcm) % frame_bytes != 0:
        raise UnsupportedFormat("%s 的 data 长度与帧大小不整除" % path)

    samples = []
    for i in range(n_frames * channels):
        off = i * bits // 8
        if bits == 8:  # 8 位 WAV 是无符号，中心 128
            samples.append(pcm[off] - 128)
        elif bits == 16:
            samples.append(struct.unpack_from("<h", pcm, off)[0])
        elif bits == 24:
            lo, mid, hi = pcm[off], pcm[off + 1], pcm[off + 2]
            v = lo | (mid << 8) | (hi << 16)
            samples.append(v - 0x1000000 if v & 0x800000 else v)
        else:  # 32 位有符号
            samples.append(struct.unpack_from("<i", pcm, off)[0])

    return {"rate": rate, "channels": channels, "bits": bits,
            "samples": samples}


def write_wav(path, rate, channels, bits, samples):
    """确定性写 PCM WAV：标准 RIFF 头，无 LIST 等额外块，无填充。"""
    if bits not in (8, 16, 24, 32):
        raise ValueError("不支持的输出位深 %d" % bits)
    lo = -(1 << (bits - 1))
    hi = (1 << (bits - 1)) - 1
    pcm = bytearray()
    for v in samples:
        v = max(lo, min(hi, int(v)))
        if bits == 8:
            pcm.append(v + 128)
        elif bits == 16:
            pcm += struct.pack("<h", v)
        elif bits == 24:
            v &= 0xFFFFFF
            pcm += bytes((v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF))
        else:
            pcm += struct.pack("<i", v)

    frame_bytes = channels * bits // 8
    data_size = len(samples) * bits // 8
    header = b"RIFF" + struct.pack("<I", 36 + data_size) + b"WAVE"
    fmt_body = struct.pack("<HHIIHH",
                           1, channels, rate, rate * frame_bytes,
                           frame_bytes, bits)
    with open(path, "wb") as f:
        f.write(header)
        f.write(b"fmt " + struct.pack("<I", 16) + fmt_body)
        f.write(b"data" + struct.pack("<I", data_size) + bytes(pcm))


def downmix(wav, channels):
    """声道数变换：升混直接复制，降混做整数平均，全程不引入浮点。"""
    src_ch, n = wav["channels"], len(wav["samples"])
    if channels == src_ch:
        return dict(wav, samples=list(wav["samples"]))
    n_frames = n // src_ch
    out = []
    if channels < src_ch:
        for i in range(n_frames):
            frame = wav["samples"][i * src_ch:(i + 1) * src_ch]
            for c in range(channels):
                group = frame[c::channels]  # 源声道按目标声道分组平均
                out.append(sum(group) // len(group))
    else:  # 升混：循环复用源声道
        for i in range(n_frames):
            for c in range(channels):
                out.append(wav["samples"][i * src_ch + (c % src_ch)])
    return {"rate": wav["rate"], "channels": channels, "bits": wav["bits"],
            "samples": out}


def resample(wav, dst_rate):
    """线性整数重采样（定点 16 位小数），同输入同输出。"""
    src_rate = wav["rate"]
    if dst_rate == src_rate:
        return dict(wav, samples=list(wav["samples"]))
    if dst_rate <= 0:
        raise ValueError("目标采样率必须为正数")
    ch, src = wav["channels"], wav["samples"]
    n_frames = len(src) // ch
    out_frames = n_frames * dst_rate // src_rate
    out = []
    for i in range(out_frames):
        # 源帧位置（定点）：pos_fixed / 65536
        pos_fixed = i * src_rate * 65536 // dst_rate
        idx = pos_fixed >> 16
        frac = pos_fixed & 0xFFFF
        nxt = min(idx + 1, n_frames - 1)
        for c in range(ch):
            a = src[idx * ch + c]
            b = src[nxt * ch + c]
            out.append(a + ((b - a) * frac >> 16))
    return {"rate": dst_rate, "channels": ch, "bits": wav["bits"],
            "samples": out}
