"""Assemble the progress montage: title cards + frame sequences + a plan for stitch.py.

Host-side. Every frame is a real render or real data -- no mockups. Where a clip could
not be produced offscreen, the title card says so on the card itself, per the montage
rules, and the substitute is named.

  python3 tools/media/build_montage.py            # writes /tmp/montage/** and plan.json
"""
from __future__ import annotations

import json
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cards
import plots

REPO = os.path.dirname(os.path.dirname(HERE))
EV = os.path.join(REPO, "docs", "twin", "evidence")
BUILD = os.environ.get("MONTAGE_BUILD", "/tmp/montage")
SRC = os.environ.get("MONTAGE_SRC", "/tmp/mediasrc")   # frames copied out of Isaac
DATA = os.environ.get("MONTAGE_DATA", "/tmp/mdata")
os.makedirs(BUILD, exist_ok=True)
plan = []


def _seq(name, n):
    d = os.path.join(BUILD, name)
    os.makedirs(d, exist_ok=True)
    return d, [os.path.join(d, f"f{i:04d}.png") for i in range(n)]


def card(name, **kw):
    p = os.path.join(BUILD, f"card_{name}.png")
    cards.card(**kw).save(p)
    return p


def frames_from(dirpath):
    if not os.path.isdir(dirpath):
        return []
    return [os.path.join(dirpath, f) for f in sorted(os.listdir(dirpath))
            if f.endswith(".png")]


def have(p):
    return os.path.exists(p)


def meta_of(z):
    return {k: z[k] for k in ("angle_min", "angle_max", "angle_increment",
                              "range_min", "range_max")}


# ---------------------------------------------------------------- 1. G1 body
orbit = frames_from(f"{SRC}/orbit_g1")
if orbit:
    plan.append(dict(
        name="1 G1 body orbit",
        card=card("1", gate="DT2 + DT3 — twin-DT2 / twin-DT3",
                  title="G1 twin body",
                  result="Unitree meshes, Dex5-1P hands, head shell, no floating sensors",
                  note="Offscreen orbit render (front -> side -> top). "
                       "No GUI viewport on this box: it has no logged-in X session."),
        frames=orbit, secs=10.0))

def walk_table(base_img, rows, upto):
    """Reveal the DT3 validate_motion table row by row over a still.

    NOT a rendered walk and not a live z trace: the twin was standing still during the
    media capture, so a z plot would have been a flat line dressed up as motion. These
    are the actual DT3 numbers, from evidence/DT3/validate_motion_dex5.txt.
    """
    from PIL import ImageDraw
    im = base_img.copy()
    d = ImageDraw.Draw(im, "RGBA")
    x0, y0, ww, hh = 70, 470, 1400, 560
    d.rectangle([x0, y0, x0 + ww, y0 + hh], fill=(10, 12, 16, 225))
    d.text((x0 + 30, y0 + 22), "DT3  validate_motion.py  —  hands ON vs DT2 baseline",
           font=cards.font(34), fill=cards.FG)
    d.text((x0 + 30, y0 + 68),
           "motion            cmd            body vel measured        zmin",
           font=cards.font(24, bold=False), fill=cards.DIM)
    y = y0 + 108
    for i, (name, cmd, meas, zmin, ok) in enumerate(rows[:upto]):
        col = cards.OK if ok else cards.WARN
        d.text((x0 + 30, y), f"{name:<16s}{cmd:<15s}{meas:<24s}{zmin}",
               font=cards.font(26, bold=False), fill=col)
        y += 40
    d.text((x0 + 30, y0 + hh - 62),
           "max base-height delta vs DT2: 0.02 m   (allowance 0.03)   no gains touched",
           font=cards.font(28), fill=cards.OK)
    return im


# --------------------------------------------------- 2. walking, DT3 measurements
WALK = [
    ("forward", "vx +0.50", "vx +0.506", "0.75", True),
    ("reverse", "vx -0.40", "vx -0.349", "0.77", True),
    ("strafe L", "vy +0.30", "vy +0.245", "0.77", True),
    ("strafe R", "vy -0.30", "vy -0.222", "0.77", True),
    ("rot ccw", "wz +0.20", "wz -0.015", "0.79", False),
    ("rot cw", "wz -0.20", "wz -0.010", "0.79", False),
    ("walk+strafe", "0.30/0.30", "vx +0.307 vy +0.232", "0.76", True),
    ("walk+turn", "0.30/0.20", "vx +0.318 wz +0.155", "0.76", True),
]
if orbit:
    from PIL import Image as _Im
    d, paths = _seq("walk", len(WALK) + 2)
    base = cards.fit(_Im.open(orbit[len(orbit) // 4]))
    for i, pp in enumerate(paths):
        walk_table(base, WALK, min(len(WALK), i)).save(pp)
    plan.append(dict(
        name="2 walk (DT3 measurements)",
        card=card("2", gate="DT3 — twin-DT3",
                  title="Walks with the hands on",
                  result="Same verdicts as DT2; max base-height delta 0.02 m (allowance 0.03)",
                  note="Rotate-in-place still FAILs -- a known property of this "
                       "checkpoint, unchanged by the hands, and deliberately not tuned.",
                  caveat="SUBSTITUTE: no rendered walk. The twin was standing still "
                         "during the media capture, so rather than plot a flat z line "
                         "these are the actual DT3 validate_motion numbers over an "
                         "orbit still."),
        frames=paths, secs=10.0))


# ------------------------------------------------ 3. sensors where the real ones are
axes = frames_from(f"{SRC}/orbit_g1_axes")
if axes:
    plan.append(dict(
        name="3 sensor frames",
        card=card("3", gate="DT2 — twin-DT2",
                  title="Sensors where the real ones are",
                  result="livox_frame, camera_link, dog_imu_link at their calibrated poses",
                  note="RGB triads drawn as real geometry at the AUTHORED prim paths -- "
                       "so this is where the sensor is, not an annotation. "
                       "Audit: tf_static exact to 0.00e+00 on every edge."),
        frames=axes, secs=6.0))
elif have(f"{EV}/DT2/g1_twin_tf_tree.png"):
    plan.append(dict(
        name="3 sensor frames (TF tree substitute)",
        card=card("3", gate="DT2 — twin-DT2",
                  title="Sensors where the real ones are",
                  result="tf_static exact to 0.00e+00 on every edge",
                  caveat="SUBSTITUTE: axis-triad orbit not produced in the time-box; "
                         "showing the live TF tree from tf2_tools view_frames instead."),
        frames=[f"{EV}/DT2/g1_twin_tf_tree.png"], secs=6.0))

# ------------------------------------------------------------- 4. Mid-360 cloud
cl = f"{DATA}/cloud_sim.npz"
if have(cl):
    z = np.load(cl)
    keys = sorted(z.files)
    d, paths = _seq("cloud", len(keys))
    for k, p in zip(keys, paths):
        plots.cloud_frame(z[k], "Mid-360 twin — /livox/lidar in base_link",
                          f"{len(z[k])*2} points/sweep (subsampled for the plot), 10 Hz",
                          caption="Floor plane fit vs world vertical: 0.0039 deg "
                                  "(tolerance 0.5) — DT2 geometry check").save(p)
    plan.append(dict(name="4 Mid-360 cloud", card=card(
        "4", gate="DT2 — twin-DT2", title="Mid-360 twin",
        result="Floor plane 0.0039 deg against world vertical",
        note="Real cloud frames from the live twin, transformed into base_link by the "
             "twin's own TF. C-6: rosette modelled as a uniform rotary grid."),
        frames=paths, secs=3.0))

# ------------------------------------------------------------------ 5. camera
before = f"{EV}/DT2/twin_camera_color.png"
after = f"{EV}/C21/twin_camera_color_after.png"
depth = f"{EV}/DT2/twin_camera_depth.png"
if have(after):
    from PIL import Image
    d, paths = _seq("cam", 4)
    pair = Image.new("RGB", (cards.W, cards.H), cards.BG)
    a = Image.open(after).convert("RGB").resize((940, 529))
    pair.paste(a, (20, 250))
    if have(depth):
        b = Image.open(depth).convert("RGB").resize((940, 529))
        pair.paste(b, (960, 250))
    pair = cards.label(pair, "colour rgb8 1280x720          |          aligned depth 16UC1 mm")
    pair = cards.overlay(pair, [("chair + mustard bottle in view -- what a detector fires on", 30, cards.FG)])
    pair.save(paths[0]); pair.save(paths[1])
    if have(before):
        ba = Image.new("RGB", (cards.W, cards.H), cards.BG)
        ba.paste(Image.open(before).convert("RGB").resize((940, 529)), (20, 250))
        ba.paste(a, (960, 250))
        ba = cards.label(ba, "C-21  BEFORE: camera pointed out the robot's right side   |   AFTER")
        ba = cards.overlay(ba, [("59.6 deg -> 0.82 deg tilt;  1.18 m -> 53 mm placement", 32, cards.OK)])
        ba.save(paths[2]); ba.save(paths[3])
    plan.append(dict(name="5 camera + C-21", card=card(
        "5", gate="DT2 / C-21 — closed", title="Camera twin",
        result="rgb8 + 16UC1 mm on the hardware topics, K exact; C-21 CLOSED",
        note="C-21: the optical flip was applied twice, so the camera pointed out of "
             "the robot's right side while every string-level check passed."),
        frames=paths, secs=8.0))

# ------------------------------------------------------------------- 6. /scan
ss, sr = f"{DATA}/scan_sim.npz", f"{DATA}/scan_real.npz"
if have(ss) and have(sr):
    zs, zr = np.load(ss), np.load(sr)
    sims, reals = zs["scans"], zr["scans"]
    n = min(30, len(sims), len(reals))
    d, paths = _seq("scan", n)
    for i, p in enumerate(paths):
        plots.polar_pair(sims[i % len(sims)], meta_of(zs),
                         reals[i % len(reals)], meta_of(zr),
                         caption="723 rays, angle_increment 0.0087, range 0.30-6.0 m -- "
                                 "identical geometry on both sides").save(p)
    plan.append(dict(name="6 /scan sim vs real", card=card(
        "6", gate="DT1 + twin-gt-g1", title="/scan — twin next to the robot",
        result="723 rays, geometry exact on every field",
        note="Right-hand plot is the real G1 from the ground-truth bag "
             "(CAPTURES.md). Same p2l parameters, same ray count."),
        frames=paths, secs=4.0))

# ------------------------------------------------------------------- 7. SLAM map
mp = f"{DATA}/map.npz"
if have(mp):
    z = np.load(mp)
    grid, res, ox, oy = z["grid"], float(z["res"]), float(z["ox"]), float(z["oy"])
    pth = np.load(f"{DATA}/path.npz")["path"] if have(f"{DATA}/path.npz") else None
    od = np.load(f"{DATA}/odom.npz") if have(f"{DATA}/odom.npz") else None
    rob = (float(od["x"][-1]), float(od["y"][-1])) if od is not None else None
    n = 30
    d, paths = _seq("map", n)
    for i, pp in enumerate(paths):
        plots.map_frame(grid, res, ox, oy, path=pth, robot=rob,
                        title="SLAM Toolbox map from the twin's /scan",
                        sub=f"{grid.shape[1]}x{grid.shape[0]} @ {res:.2f} m  "
                            f"= {grid.shape[1]*res:.1f} x {grid.shape[0]*res:.1f} m",
                        reveal=(i + 1) / n).save(pp)
    plan.append(dict(name="7 SLAM map", card=card(
        "7", gate="DT2 — twin-DT2", title="Nav2 + SLAM close the loop",
        result="twin /scan -> SLAM Toolbox -> costmap, live",
        note="The map is small because the sensor is honest: range_max 6.0 m in a "
             "corridor. The DT0 stand-in lidar (30 m, horizontal) would have mapped "
             "the whole floor.",
        caveat="NO Nav2 PATH SHOWN: goals reach the tolerance boundary and time out "
               "(DT2 recorded PARTIAL and was deliberately not tuned), so the planner "
               "published no plan to draw."),
        frames=paths, secs=10.0))

# ---------------------------------------------------------------- 8. hand poses
poses = [("rest", "rest"), ("open", "open"), ("fist", "fist"),
         ("thumb_opposition", "thumb opposition")]
hp = [(f"{EV}/DT3/hand_{a}.png", b) for a, b in poses]
if all(have(f) for f, _ in hp):
    from PIL import Image
    d, paths = _seq("hands", len(hp))
    for (f, lab), pp in zip(hp, paths):
        im = cards.fit(Image.open(f))
        im = cards.label(im, f"Dex5-1P  —  {lab}")
        cards.overlay(im, [("commanded by NAME, never by index (C-14, RULE-HAND-NAME)", 28, cards.FG),
                           ("pose reached to <= 0.001 rad", 28, cards.OK)]).save(pp)
    plan.append(dict(name="8 hand poses", card=card(
        "8", gate="DT3 — twin-DT3", title="Dex5-1P hand poses",
        result="rest / open / fist / thumb opposition, all reached to <= 0.001 rad",
        note="Isaac interleaves the hand DOFs (left 29..63, right 34..68), so every "
             "pose is built by joint name. C-13: the four passive joints are held at "
             "zero, not mimic-coupled."),
        frames=paths, secs=8.0))

# ------------------------------------------------------------------- 9. Go2
GO2 = os.environ.get("MONTAGE_GO2", "/tmp/mgo2")
go2o = frames_from(f"{SRC}/orbit_go2")
go2s = f"{GO2}/scan_sim.npz"
g9 = []
if go2o:
    g9 = go2o
if have(f"{GO2}/cloud_sim.npz"):
    zc = np.load(f"{GO2}/cloud_sim.npz")
    keys = sorted(zc.files)
    d, cp = _seq("go2cloud", len(keys))
    for k, pp in zip(keys, cp):
        plots.cloud_frame(zc[k], "Go2 Mid-360 twin — /unitree/slam_lidar/points",
                          "20 Hz  (render_dt 0.025 s = 40 Hz, decimation step 2)",
                          caption="Top mount (0.187, 0, 0.0803), pitch 0.2249 rad",
                          scale=90.0).save(pp)
    g9 = g9 + cp
if have(go2s):
    zg = np.load(go2s)
    n_hit = int(((zg["scans"][0] >= 0.29) & (zg["scans"][0] <= 0.32) &
                 np.isfinite(zg["scans"][0])).sum())
    d, gp = _seq("go2scan", 20)
    for i, pp in enumerate(gp):
        plots.polar_pair(zg["scans"][i % len(zg["scans"])], meta_of(zg), None, None,
                         highlight=(0.29, 0.32), sim_label="GO2 TWIN /scan",
                         caption=f"C-17: {n_hit} rays 0.30-0.31 m off the robot's own "
                                 "nose (red) -- hardware bag decides tomorrow").save(pp)
    g9 = g9 + gp
if g9:
    plan.append(dict(name="9 Go2 twin", card=card(
        "9", gate="DT5 — twin-DT5", title="Go2 twin",
        result="Interface Class-A conformant (45 pass / 0 Class-A); nav blocked by C-17",
        note="Mid-360 on the top mount at (0.187, 0, 0.0803), pitch 0.2249 rad. "
             "Root /scan and /odom, exactly as the driver publishes them.",
        caveat=("" if have(go2s) else
                "SUBSTITUTE: the self-hit polar plot needs the Go2 sim live and the "
                "time-box ran out; the numbers are in evidence/DT5/self_hit_cluster.txt.")),
        frames=g9, secs=9.0))

# --------------------------------------------------------------- 10. closing card
close = os.path.join(BUILD, "card_10.png")
from PIL import ImageDraw
im = cards.card(gate="STATUS 2026-08-18", title="Where the twin is",
                result="", note="")
d = ImageDraw.Draw(im)
rows = [
    ("G1 twin", "0 Class-A vs contract; bag-confirmed OQ-2 + OQ-3", cards.OK),
    ("Go2 twin", "0 Class-A vs contract; nav blocked by C-17", cards.WARN),
    ("Gates", "DT0 DT1 DT2 DT3 DT5  +  fastpath, gt-g1, persist", cards.FG),
    ("Deviations", "C-1 .. C-20 open;  C-21 CLOSED", cards.FG),
    ("Next", "Go2 ground-truth bag -> decides C-17", cards.ACCENT),
    ("Open", "E-1 detection half, E-2 Go2 bracket, OQ-5.2/5.3, OQ-6, C-3/C-4", cards.DIM),
]
y = 470
for k, v, col in rows:
    d.text((110, y), k, font=cards.font(34), fill=cards.DIM)
    d.text((430, y), v, font=cards.font(34, bold=False), fill=col)
    y += 68
d.text((110, y + 30), "docs/twin/RESUME.md  --  resumable from GitHub alone",
       font=cards.font(30, bold=False), fill=cards.ACCENT)
im.save(close)
plan.append(dict(name="10 closing", card=close, card_hold=7.0))

json.dump(plan, open(os.path.join(BUILD, "plan.json"), "w"), indent=1)
print(f"planned {len(plan)} clips")
for c in plan:
    print(f"  {c['name']}: {len(c.get('frames', []))} frames, {c.get('secs', 0)} s")
