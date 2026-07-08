#!/usr/bin/env python3
# MP4 -> EPV1: raw 16-grey packed video for EPD_Painter2's video_player.
#
# Header (16 bytes, little-endian):
#   0  "EPV1"
#   4  uint16 width
#   6  uint16 height
#   8  uint8  fps
#   9  uint8  flags (0)
#   10 uint16 reserved
#   12 uint32 frame count
# Frames: width*height/2 bytes each, 2 px/byte, HIGH nibble = left pixel,
# values are display grey 0=white .. 15=black (pre-inverted, ready to memcpy).

import subprocess, struct, sys
import numpy as np

SRC = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else "video.epv"
W, H, FPS = 360, 270, 10

# 4x4 Bayer ordered-dither offsets in [-0.5, +0.5)
B = np.array([[ 0, 8, 2,10],
              [12, 4,14, 6],
              [ 3,11, 1, 9],
              [15, 7,13, 5]], np.float32)
bayer = np.tile((B + 0.5) / 16.0 - 0.5,
                ((H + 3) // 4, (W + 3) // 4))[:H, :W]

proc = subprocess.Popen(
    ["ffmpeg", "-v", "error", "-i", SRC,
     "-vf", f"fps={FPS},scale={W}:{H}",
     "-f", "rawvideo", "-pix_fmt", "gray", "-"],
    stdout=subprocess.PIPE)

frames = 0
prev = None      # temporal deadband: pixels flip only on a decisive change,
DEAD = 0.8       # so h264 flutter doesn't churn the whole panel every frame
with open(OUT, "wb") as f:
    f.write(struct.pack("<4sHHBBHI", b"EPV1", W, H, FPS, 0, 0, 0))
    while True:
        raw = proc.stdout.read(W * H)
        if len(raw) < W * H:
            break
        a = np.frombuffer(raw, np.uint8).astype(np.float32).reshape(H, W)
        ideal = a * (15.0 / 255.0) + bayer
        q = np.clip(np.rint(ideal), 0, 15)
        if prev is not None:
            q = np.where(np.abs(ideal - prev) >= DEAD, q, prev)
        prev = q
        inv = (15 - q).astype(np.uint8)                # 0=white .. 15=black
        packed = (inv[:, 0::2] << 4) | inv[:, 1::2]    # high nibble = left px
        f.write(packed.tobytes())
        frames += 1
    f.seek(12)
    f.write(struct.pack("<I", frames))

proc.wait()
print(f"{frames} frames, {W}x{H}@{FPS}fps -> {OUT} "
      f"({(16 + frames * W * H // 2) / 1e6:.1f} MB)")
