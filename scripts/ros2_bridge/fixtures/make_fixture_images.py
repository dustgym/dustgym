"""Author the fixture camera PNGs for sensor_bridge_contract.md §2.4.

Produces, under ``fixtures/000/``:
  * ``front_left.png``  -- 1280x720 grey background with a rendered ``tag36h11`` id-0 marker
    pasted at a known location, so AprilTag detection can be proven IN-CONTAINER independent
    of the Godot (G1) track (contract §1 "the integration test ... is the acceptance check").
  * ``front_right.png`` -- a plain grey placeholder (no tag); enough for the bag-writer /
    schema / CameraInfo path.

The marker bitmap is the canonical AprilRobotics ``apriltag-imgs/tag36h11/tag36_11_00000.png``
(10x10 px: a 1px white quiet ring around the 8x8 tag = its black border + 6x6 payload).  That
file is BSD-licensed *data* (a codebook bitmap, not relicensed art); it is fetched once into
``fixtures/_assets/`` and committed so this script is offline-reproducible.  We nearest-
neighbour upscale it (preserving the white quiet zone the detector needs) and paste it.

Pure stdlib (``zlib``, ``struct``) -- NO Pillow / numpy required, so it runs on the host with
the repo's bare ``python3`` and needs nothing in ``.venv``.  Re-run:

    python3 fixtures/make_fixture_images.py

CC0-1.0 for this script (see ../../LICENSE).  The embedded tag bitmap is BSD (AprilRobotics).
"""

from __future__ import annotations

import os
import struct
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
ASSET = os.path.join(HERE, "_assets", "tag36_11_00000.png")
OUT = os.path.join(HERE, "000")

W, H = 1280, 720
GREY = 128            # background grey level
TAG_SCALE = 24        # px per tag cell -> 10*24 = 240 px tag block (incl. quiet ring)
# Paste so the tag's BLACK-BORDER square centre sits near image centre.  The 10x10 block has a
# 1-cell quiet ring, so the black border occupies cells [1..8]; centre of that is cell 4.5.
TAG_TOP = 240
TAG_LEFT = 520


def _decode_png_gray(path: str) -> tuple[int, int, list[bytes]]:
    """Minimal PNG decoder for 8-bit RGBA (colortype 6); returns (w,h, list-of-rows[R bytes])."""
    d = open(path, "rb").read()
    assert d[:8] == b"\x89PNG\r\n\x1a\n", "not a PNG"
    i = 8
    w = h = 0
    idat = b""
    while i < len(d):
        ln = struct.unpack(">I", d[i:i + 4])[0]
        typ = d[i + 4:i + 8]
        body = d[i + 8:i + 8 + ln]
        i += 12 + ln
        if typ == b"IHDR":
            w, h, bitdepth, colortype = struct.unpack(">IIBB", body[:10])
            assert bitdepth == 8 and colortype == 6, (bitdepth, colortype)
        elif typ == b"IDAT":
            idat += body
        elif typ == b"IEND":
            break
    raw = zlib.decompress(idat)
    stride = 1 + w * 4
    bpp = 4
    prev = bytearray(w * 4)
    rows = []

    def paeth(a, b, c):
        p = a + b - c
        pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
        return a if pa <= pb and pa <= pc else (b if pb <= pc else c)

    for r in range(h):
        f = raw[r * stride]
        line = bytearray(raw[r * stride + 1:r * stride + 1 + w * 4])
        for x in range(len(line)):
            a = line[x - bpp] if x >= bpp else 0
            b = prev[x]
            c = prev[x - bpp] if x >= bpp else 0
            if f == 1:
                line[x] = (line[x] + a) & 255
            elif f == 2:
                line[x] = (line[x] + b) & 255
            elif f == 3:
                line[x] = (line[x] + ((a + b) >> 1)) & 255
            elif f == 4:
                line[x] = (line[x] + paeth(a, b, c)) & 255
        prev = line
        rows.append(bytes(line[0::4]))  # R channel only (tag is monochrome)
    return w, h, rows


def _write_png_gray(path: str, w: int, h: int, pixels: bytearray) -> None:
    """Write an 8-bit grayscale (colortype 0) PNG.  `pixels` is row-major w*h bytes."""
    def chunk(typ: bytes, body: bytes) -> bytes:
        return (struct.pack(">I", len(body)) + typ + body
                + struct.pack(">I", zlib.crc32(typ + body) & 0xFFFFFFFF))

    raw = bytearray()
    for r in range(h):
        raw.append(0)  # filter type 0 (none)
        raw.extend(pixels[r * w:(r + 1) * w])
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)
    data = (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(bytes(raw), 9))
            + chunk(b"IEND", b""))
    open(path, "wb").write(data)


def build() -> None:
    os.makedirs(OUT, exist_ok=True)

    # right cam: plain grey placeholder.
    grey = bytearray([GREY]) * (W * H)
    _write_png_gray(os.path.join(OUT, "front_right.png"), W, H, grey)

    # left cam: grey + upscaled tag block.
    tw, th, trows = _decode_png_gray(ASSET)
    canvas = bytearray([GREY]) * (W * H)
    for ty in range(th):
        src = trows[ty]
        for dy in range(TAG_SCALE):
            row = TAG_TOP + ty * TAG_SCALE + dy
            if not (0 <= row < H):
                continue
            base = row * W
            for tx in range(tw):
                val = 255 if src[tx] > 127 else 0
                col0 = TAG_LEFT + tx * TAG_SCALE
                for dx in range(TAG_SCALE):
                    col = col0 + dx
                    if 0 <= col < W:
                        canvas[base + col] = val
    _write_png_gray(os.path.join(OUT, "front_left.png"), W, H, canvas)
    print(f"wrote {OUT}/front_left.png ({W}x{H}, tag36h11 id-0 @ {TAG_TOP},{TAG_LEFT}, "
          f"{tw * TAG_SCALE}px block)")
    print(f"wrote {OUT}/front_right.png ({W}x{H}, grey placeholder)")


if __name__ == "__main__":
    build()
