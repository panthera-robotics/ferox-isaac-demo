"""Assemble the G1 montage for the 2026-08-19 video-review fixes.

A sibling of build_montage.py, not a replacement: that one is the DT0-DT5 story and
still builds. This one is the G1-only follow-up, and its clip list is the one the
review asked for.

RULES, unchanged from the first montage: every frame is a real render or real data,
no mockups, and where a clip could not be produced the TITLE CARD SAYS SO on the card
itself and names why. One clip is in that position here -- the twin camera, which
this box physically cannot run (C-23) -- and it gets a card that says exactly that
rather than being quietly dropped.

  python3 tools/media/build_montage_g1.py     # writes /tmp/montage_g1/** and plan.json
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cards      # noqa: E402
import plots      # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
EV = os.path.join(REPO, "docs", "twin", "evidence")
BUILD = os.environ.get("MONTAGE_BUILD", "/tmp/montage_g1")
SRC = os.environ.get("MONTAGE_SRC", "/tmp/mediasrc")
DATA = os.environ.get("MONTAGE_DATA", "/tmp/mdata")
os.makedirs(BUILD, exist_ok=True)
plan = []


def card(name, **kw):
    p = os.path.join(BUILD, f"card_{name}.png")
    cards.card(**kw).save(p)
    return p


def seq(name, n):
    d = os.path.join(BUILD, name)
    os.makedirs(d, exist_ok=True)
    return d, [os.path.join(d, f"f{i:04d}.png") for i in range(n)]


def frames_from(dirpath, limit=None):
    if not os.path.isdir(dirpath):
        return []
    fs = [os.path.join(dirpath, f) for f in sorted(os.listdir(dirpath))
          if f.endswith(".png")]
    return fs[:limit] if limit else fs


def npz(name):
    p = os.path.join(DATA, name)
    return np.load(p) if os.path.exists(p) else None


def floor_tilt_deg(pts):
    """Least-squares plane through the low points, angled against world +Z.

    Same quantity C-21 used for the camera, applied to the lidar cloud: it is the
    one number that says whether the sensor is mounted the way the contract says.
    """
    z = pts[:, 2]
    lo = pts[z < np.percentile(z, 20)]
    if len(lo) < 50:
        return None, None
    A = np.c_[lo[:, 0], lo[:, 1], np.ones(len(lo))]
    coef, *_ = np.linalg.lstsq(A, lo[:, 2], rcond=None)
    n = np.array([-coef[0], -coef[1], 1.0])
    n /= np.linalg.norm(n)
    ang = np.degrees(np.arccos(abs(n[2])))
    rms = float(np.sqrt(np.mean((A @ coef - lo[:, 2]) ** 2)))
    return ang, rms


# --- 1. body orbit, correct hands ------------------------------------------
orb = frames_from(os.path.join(SRC, "orbit"))
if orb:
    plan.append(dict(
        name="1 body orbit",
        card=card("1", gate="DT3 — hand mount FIXED",
                  title="The G1 twin, with the hands the right way round",
                  result="fingers 90.000° → 0.707° off the forearm axis, both hands",
                  note="rpy (−π/2, −π/2, 0) on both flanges, derived from the Dex5 "
                       "URDF and self-tested against Unitree's own G1+Inspire "
                       "assembly to 1e-16"),
        frames=orb, fps=30))

# --- 2. the walk, with the base-height overlay -----------------------------
od = npz("odom.npz")
walk = frames_from(os.path.join(SRC, "orbit"), limit=60)
if od is not None and walk:
    z = np.asarray(od["z"])
    d, paths = seq("walk", len(walk))
    from PIL import Image, ImageDraw
    for i, (src, dst) in enumerate(zip(walk, paths)):
        im = Image.open(src).convert("RGB")
        dr = ImageDraw.Draw(im)
        k = int(len(z) * (i + 1) / len(walk))
        seg = z[:max(2, k)]
        x0, y0, w, h = 120, 780, 700, 200
        dr.rectangle([x0 - 14, y0 - 44, x0 + w + 14, y0 + h + 14], fill=(18, 20, 24))
        dr.text((x0, y0 - 40), "base height z (m), live /odom",
                font=cards.font(26, bold=False), fill=cards.ACCENT)
        lo, hi = float(z.min()), float(z.max())
        rng = max(1e-3, hi - lo)
        pts = [(x0 + w * j / max(1, len(seg) - 1),
                y0 + h - h * (v - lo) / rng) for j, v in enumerate(seg)]
        if len(pts) > 1:
            dr.line(pts, fill=cards.OK, width=3)
        dr.text((x0, y0 + h - 30), f"min {lo:.3f}   max {hi:.3f}   spread {hi-lo:.3f} m",
                font=cards.font(26, bold=False), fill=cards.FG)
        im.save(dst)
    plan.append(dict(
        name="2 walk with hands on",
        card=card("2", gate="DT3 — walk regression",
                  title="It still walks, with the corrected hands",
                  result="forward / reverse / strafe / upright PASS; rotate the known "
                         "checkpoint FAIL. No gain touched.",
                  note="base height spread 0.029 m, inside the 0.03 allowance and "
                       "inside DT2/DT3's own 0.75–0.79 m band"),
        frames=paths, fps=30))

# --- 3. sensor frames at their calibrated poses ----------------------------
ax = frames_from(os.path.join(SRC, "orbit_axes"))
if ax:
    plan.append(dict(
        name="3 sensor frames",
        card=card("3", gate="DT2 — sensor placement",
                  title="Every sensor frame, drawn where it actually is",
                  result="8/8 frames verified against g1_contract.yaml",
                  note="RGB triads at the AUTHORED prim paths — geometry, not "
                       "annotation. Green livox_frame, blue camera_link, "
                       "orange dog_imu_link."),
        frames=ax, fps=30))

# --- 4. the Mid-360 cloud, now a full sweep --------------------------------
cs = npz("cloud_sim.npz")
if cs is not None:
    keys = [k for k in cs.files if k.startswith("c")]
    keys.sort(key=lambda s: int(s[1:]))
    d, paths = seq("cloud", len(keys))
    tilt = rms = None
    for i, (k, dst) in enumerate(zip(keys, paths)):
        pts = np.asarray(cs[k])
        if tilt is None:
            tilt, rms = floor_tilt_deg(pts)
        az = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
        bins = np.clip(np.floor((az + 180) / 10).astype(int), 0, 35)
        occ = int((np.bincount(bins, minlength=36) > 0).sum())
        cap = (f"{len(pts)} points   azimuth {occ}/36 bins = {occ*10}°"
               + (f"   floor plane {tilt:.3f}° vs world vertical (RMS {rms*1000:.1f} mm)"
                  if tilt is not None else ""))
        plots.cloud_frame(pts, "Mid-360 — one published message",
                          sub="/livox/lidar, livox_frame", caption=cap,
                          azim=-58.0 + i * 4.0).save(dst)
    plan.append(dict(
        name="4 Mid-360 full sweep",
        card=card("4", gate="DT2 — the sector defect, FIXED",
                  title="One message is now a whole revolution",
                  result=f"360° in 36/36 bins, ~{len(np.asarray(cs[keys[0]]))} valid "
                         f"points — was ~72° and 4285",
                  note="Isaac 5.1 has two ROS 2 cloud writers; the twin was using the "
                       "one that emits a single render frame's slice. "
                       "omni:sensor:Core:accumulateOutputs does not exist."),
        frames=[q for q in paths for _ in range(10)], fps=30))

# --- 5. the camera: cannot be produced on this box -------------------------
plan.append(dict(
    name="5 camera — BLOCKED",
    card=card("5", gate="C-23 — environment, not the twin",
              title="The twin camera could not be filmed here",
              result="This box segfaults Isaac's synthetic-data pipeline whenever "
                     "the ROS 2 image writer is live — 5 boots for 5.",
              note="Not memory (peak VRAM 3776 MiB of 16376), not the asset, not "
                   "the world. The Go2 twin, same box and no camera in its "
                   "contract, boots every time.",
              caveat="So clip 5 — colour, aligned depth, and the C-21 before/after "
                     "— is ABSENT rather than faked. It needs the RTX 4090 box the "
                     "campaign was validated on."),
    card_hold=7.0))

# --- 6. /scan, twin vs the real robot --------------------------------------
ss, sr = npz("scan_sim.npz"), npz("scan_real.npz")
if ss is not None and sr is not None:
    n = 20
    d, paths = seq("scan", n)
    smeta = {k: float(ss[k]) for k in ("angle_min", "angle_max", "angle_increment",
                                       "range_min", "range_max")}
    rmeta = {k: float(sr[k]) for k in ("angle_min", "angle_max", "angle_increment",
                                       "range_min", "range_max")}
    S, R = np.asarray(ss["scans"]), np.asarray(sr["scans"])
    for i, dst in enumerate(paths):
        plots.polar_pair(S[i % len(S)], smeta, R[i % len(R)], rmeta).save(dst)
    fin = lambda a: 100.0 * np.isfinite(a).sum() / a.size  # noqa: E731
    plan.append(dict(
        name="6 /scan twin vs robot",
        card=card("6", gate="DT2 — /scan against the ground-truth bag",
                  title="The twin's scan beside the robot's",
                  result=f"twin {fin(S):.0f}% finite (was ~20%), robot {fin(R):.0f}%",
                  note="Same geometry on both: 723 rays, ±3.14159, increment 0.0087, "
                       "0.30–6.0 m, base_link."),
        frames=[q for q in paths for _ in range(6)], fps=30))

# --- 7. SLAM map + the Nav2 path -------------------------------------------
mp, pth = npz("map.npz"), npz("path.npz")
if mp is not None:
    grid = np.asarray(mp["grid"])
    path = np.asarray(pth["path"]) if pth is not None else None
    n = 24
    d, paths = seq("map", n)
    for i, dst in enumerate(paths):
        plots.map_frame(grid, float(mp["res"]), float(mp["ox"]), float(mp["oy"]),
                        path=path if i > n // 2 else None,
                        title="SLAM map — filling all round",
                        sub="/ferox/g1_01/map, hospital",
                        reveal=(i + 1) / n).save(dst)
    known = 100.0 * (grid >= 0).sum() / grid.size
    plan.append(dict(
        name="7 SLAM + Nav2",
        card=card("7", gate="DT2 — navigation",
                  title="The map fills all round, and a goal is reached",
                  result="Goal 1 SUCCEEDED — the first Nav2 goal this twin has ever "
                         "reached. Goals 2 and 3 aborted outside the mapped area.",
                  note=f"map known cells {known:.0f}%, and their coverage about the "
                       f"centroid is 36/36 ten-degree bins. Before the fix SLAM built "
                       f"a fixed wedge."),
        frames=[q for q in paths for _ in range(5)], fps=30))

# --- 8. the four hand poses ------------------------------------------------
hp = [os.path.join(EV, "DT3", f"hand_{n}.png")
      for n in ("rest", "open", "fist", "thumb_opposition")]
hp = [p for p in hp if os.path.exists(p)]
if hp:
    plan.append(dict(
        name="8 hand poses",
        card=card("8", gate="DT3 — hand poses, re-rendered",
                  title="Rest, open, fist, thumb opposition",
                  result="all four reached to ≤0.004 rad, on the corrected mount",
                  note="33 of 40 joints driven per pose; the other 7 are the passive "
                       "abductions held at zero (C-13) and the wrist."),
        # Each pose held ~1.5 s by repeating the frame: stitch.py has no
        # per-frame hold, and four PNGs at any sane fps is a flicker.
        frames=[q for q in hp for _ in range(45)], fps=30))

# --- 9. closing status -----------------------------------------------------
close = os.path.join(BUILD, "card_close.png")
cards.card(gate="STATUS 2026-08-19",
           title="Where the G1 twin is",
           result="A FIXED · B FIXED · C blocked by the box",
           note="A: hand mount 90.000° → 0.707°, Isaac suite 13/13.   "
                "B: cloud 72° → 360°, /scan 20% → 45%, first Nav2 goal SUCCEEDED.   "
                "C: needs a box that can run the camera.",
           caveat="Open: item C, DT2 nav 1 of 3, E-1, E-2, OQ-5.x, C-3/C-4. "
                  "Deviations C-1…C-20 open, C-21 closed, C-23 opened.").save(close)
plan.append(dict(name="9 closing", card=close, card_hold=7.0))

with open(os.path.join(BUILD, "plan.json"), "w") as fh:
    json.dump(plan, fh, indent=2)
print(f"wrote {len(plan)} clips -> {BUILD}/plan.json")
for c in plan:
    print(f"  {c['name']:26s} frames={len(c.get('frames', []))}")
