#!/usr/bin/env python3
import struct
import sys
from pathlib import Path


data = Path(sys.argv[1]).read_bytes()
u32 = lambda offset: struct.unpack_from("<I", data, offset)[0]

assert data[:12] == b"RIFF" + data[4:8] + b"AVI "
assert u32(4) == len(data) - 8
assert data[212:224] == b"LIST" + data[216:220] + b"movi"

index_offset = data.find(b"idx1", 224)
assert index_offset > 224
assert u32(44) & 0x10
assert u32(index_offset + 4) == u32(48) * 16

offset = 224
frames = 0
while offset < index_offset:
    assert data[offset : offset + 4] == b"00dc"
    size = u32(offset + 4)
    jpeg = data[offset + 8 : offset + 8 + size]
    assert jpeg[:2] == b"\xff\xd8" and jpeg[-2:] == b"\xff\xd9"
    entry = index_offset + 8 + frames * 16
    assert data[entry : entry + 4] == b"00dc"
    assert u32(entry + 4) & 0x10
    assert u32(entry + 8) == offset - 220
    assert u32(entry + 12) == size
    offset += 8 + size + (size & 1)
    frames += 1

assert offset == index_offset
assert index_offset + 8 + u32(index_offset + 4) == len(data)
assert frames == u32(48) == u32(140) and frames > 0
print(f"valid AVI: {frames} MJPEG frames")
