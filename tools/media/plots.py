"""Polar-scan, point-cloud and map renderers for the montage. PIL + numpy only.

No matplotlib: it lives in the Isaac image, the data lives elsewhere, and a polar
plot is a coordinate transform and some dots. Drawing it here keeps the montage
buildable on the host from .npz files alone -- which also means any of these frames
can be re-rendered later without the sim.
"""
from __future__ import annotations

import math
import numpy as np
from PIL import Image, ImageDraw

import cards

W, H = cards.W, cards.H
BG = cards.BG
GRID = (52, 58, 68)
FG = cards.FG
DIM = cards.DIM


def _axes_polar(d, cx, cy, R, rmax, title, sub=""):
    for frac in (0.25, 0.5, 0.75, 1.0):
        r = R * frac
        d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=GRID)
        d.text((cx + 6, cy - r - 22), f"{rmax*frac:.1f} m",
               font=cards.font(20, bold=False), fill=DIM)
    for a in range(0, 360, 30):
        t = math.radians(a)
        d.line([cx, cy, cx + R * math.cos(t), cy + R * math.sin(t)], fill=GRID)
    d.text((cx - R, cy - R - 62), title, font=cards.font(38), fill=FG)
    if sub:
        d.text((cx - R, cy - R - 26), sub, font=cards.font(24, bold=False), fill=DIM)


def polar_pair(sim_ranges, sim_meta, real_ranges, real_meta, highlight=None,
               caption="", sim_label="TWIN", real_label="ROBOT (bag)"):
    """Two polar plots side by side. highlight = (lo_m, hi_m) drawn in warn colour."""
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)
    R = 380
    for k, (rr, meta, lab, cx) in enumerate((
            (sim_ranges, sim_meta, sim_label, 500),
            (real_ranges, real_meta, real_label, 1420))):
        if rr is None:
            d.text((cx - 200, H // 2), "not captured", font=cards.font(34), fill=DIM)
            continue
        rmax = float(meta["range_max"])
        finite = np.isfinite(rr) & (rr > 0)
        n = int(finite.sum())
        _axes_polar(d, cx, H // 2 + 20, R, rmax, lab,
                    f"{len(rr)} rays, {n} finite ({100*n/len(rr):.0f}%)")
        a0 = float(meta["angle_min"]); da = float(meta["angle_increment"])
        for i in np.nonzero(finite)[0]:
            rho = float(rr[i]) / rmax
            th = a0 + i * da
            x = cx + R * rho * math.cos(th)
            y = H // 2 + 20 + R * rho * math.sin(th)
            col = cards.ACCENT
            if highlight and highlight[0] <= rr[i] <= highlight[1] and k == 0:
                col = (255, 90, 90)
                d.ellipse([x-5, y-5, x+5, y+5], fill=col)
            else:
                d.ellipse([x-2, y-2, x+2, y+2], fill=col)
    if caption:
        im = cards.overlay(im, [(c, 30, FG) for c in cards._wrap(caption, 100)],
                           corner="bl")
    return im


def cloud_frame(pts, title, sub="", caption="", elev=28.0, azim=-58.0, scale=64.0):
    """Orthographic scatter of a cloud in base_link, coloured by height."""
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)
    cx, cy = W // 2, H // 2 + 60
    e, a = math.radians(elev), math.radians(azim)
    ca, sa, ce, se = math.cos(a), math.sin(a), math.cos(e), math.sin(e)
    # ground grid, so the scale is legible
    for gx in range(-6, 7, 2):
        for seg in ((gx, -6, gx, 6), (-6, gx, 6, gx)):
            x0, y0, x1, y1 = seg
            p = []
            for (X, Y) in ((x0, y0), (x1, y1)):
                u = (X * ca - Y * sa) * scale
                v = ((X * sa + Y * ca) * se - (-0.8) * ce) * scale
                p.append((cx + u, cy - v))
            d.line([p[0], p[1]], fill=GRID)
    if pts is not None and len(pts):
        z = pts[:, 2]
        lo, hi = np.percentile(z, 2), np.percentile(z, 98)
        for X, Y, Z in pts:
            u = (X * ca - Y * sa) * scale
            v = ((X * sa + Y * ca) * se - Z * ce) * scale
            t = 0.0 if hi <= lo else float(np.clip((Z - lo) / (hi - lo), 0, 1))
            col = (int(60 + 195 * t), int(150 + 60 * (1 - abs(t - 0.5) * 2)),
                   int(250 - 150 * t))
            px, py = cx + u, cy - v
            if 0 <= px < W and 0 <= py < H:
                d.point((px, py), fill=col)
    d.text((90, 60), title, font=cards.font(44), fill=FG)
    if sub:
        d.text((90, 118), sub, font=cards.font(28, bold=False), fill=DIM)
    if caption:
        im = cards.overlay(im, [(c, 30, FG) for c in cards._wrap(caption, 100)],
                           corner="bl")
    return im


def map_frame(grid, res, ox, oy, path=None, robot=None, title="", sub="", reveal=1.0):
    """Occupancy grid, optionally revealed left-to-right to animate mapping."""
    im = Image.new("RGB", (W, H), BG)
    h, w = grid.shape
    s = min((H - 260) / max(1, h), (W - 400) / max(1, w))
    img = Image.new("RGB", (w, h), (34, 38, 46))
    px = img.load()
    cut = int(w * reveal)
    for j in range(h):
        for i in range(min(w, cut)):
            v = grid[j, i]
            px[i, j] = ((235, 238, 242) if v == 0 else
                        (24, 26, 30) if v < 0 else (255, 110, 110))
    img = img.resize((int(w * s), int(h * s)), Image.NEAREST).transpose(Image.FLIP_TOP_BOTTOM)
    x0, y0 = (W - img.width) // 2, (H - img.height) // 2 + 40
    im.paste(img, (x0, y0))
    d = ImageDraw.Draw(im)

    def to_px(X, Y):
        return (x0 + (X - ox) / res * s, y0 + img.height - (Y - oy) / res * s)

    if path is not None and len(path) > 1:
        d.line([to_px(*p) for p in path], fill=(90, 200, 250), width=5)
    if robot is not None:
        rx, ry = to_px(*robot)
        d.ellipse([rx-10, ry-10, rx+10, ry+10], fill=(120, 220, 140))
    d.text((90, 60), title, font=cards.font(44), fill=FG)
    if sub:
        d.text((90, 118), sub, font=cards.font(28, bold=False), fill=DIM)
    return im
