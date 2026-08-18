"""Stitch the montage: PNG sequences + title cards -> 1080p mp4, with cv2.

cv2, not ffmpeg: there is no ffmpeg binary anywhere on this box, but OpenCV is built
against libavcodec and writes H.264 through the `avc1` fourcc. Verified before the
pipeline was designed around it.

Runs in the nav container (that is where cv2 lives). Reads a JSON plan so the clip
list is data, not code:

  [{"card": {...}, "frames": ["/path/f0000.png", ...], "hold": 2.0, "fps": 30}, ...]
"""
from __future__ import annotations

import json
import os
import sys

import cv2
import numpy as np

W, H, FPS = 1920, 1080, 30


def load(path):
    im = cv2.imread(path, cv2.IMREAD_COLOR)
    if im is None:
        raise SystemExit(f"unreadable frame: {path}")
    if (im.shape[1], im.shape[0]) != (W, H):
        s = min(W / im.shape[1], H / im.shape[0])
        r = cv2.resize(im, (max(1, int(im.shape[1]*s)), max(1, int(im.shape[0]*s))),
                       interpolation=cv2.INTER_AREA)
        canvas = np.full((H, W, 3), (24, 20, 18), dtype=np.uint8)
        y0 = (H - r.shape[0]) // 2
        x0 = (W - r.shape[1]) // 2
        canvas[y0:y0+r.shape[0], x0:x0+r.shape[1]] = r
        im = canvas
    return im


def main() -> int:
    plan = json.load(open(sys.argv[1]))
    out = sys.argv[2]
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"avc1"), FPS, (W, H))
    if not vw.isOpened():
        raise SystemExit("VideoWriter would not open with avc1")
    total = 0
    for clip in plan:
        name = clip.get("name", "?")
        if clip.get("card"):
            f = load(clip["card"])
            n = int(FPS * float(clip.get("card_hold", 2.0)))
            for _ in range(n):
                vw.write(f)
            total += n
        frames = clip.get("frames", [])
        if not frames:
            print(f"  {name}: card only")
            continue
        secs = float(clip.get("secs", 0)) or len(frames) / FPS
        want = max(1, int(FPS * secs))
        # resample the sequence to the requested duration, forward then back if the
        # clip asks to ping-pong (an orbit reads better returning than cutting)
        idx = np.linspace(0, len(frames) - 1, want).round().astype(int)
        if clip.get("pingpong") and len(frames) > 2:
            half = want // 2
            fwd = np.linspace(0, len(frames)-1, half).round().astype(int)
            idx = np.concatenate([fwd, fwd[::-1]])
        for i in idx:
            vw.write(load(frames[int(i)]))
        total += len(idx)
        print(f"  {name}: {len(idx)} frames ({len(idx)/FPS:.1f} s) from {len(frames)} sources")
    vw.release()
    dur = total / FPS
    sz = os.path.getsize(out)
    print(f"\n{out}: {total} frames, {dur:.1f} s, {sz/1e6:.2f} MB")
    if dur > 180:
        print(f"WARNING: {dur:.1f} s exceeds the 3-minute cap")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
