#!/usr/bin/env python3
"""Film the twin: chase + fixed cameras, per-frame convergence, ghosting test.

Runs INSIDE the Isaac container, offscreen. This box has no logged-in desktop X
session, so any GUI viewport renders empty -- and offscreen is the route the DT
campaign already proved (RESUME §2).

WHY THIS EXISTS RATHER THAN render_orbit.py
-------------------------------------------
render_orbit.py orbits a STANDING robot. Every motion gate from MM1 on needs the
robot to be MOVING while the camera follows it, and the DT progress montage showed
what happens if you do that naively: motion ghosting -- a trail behind the robot,
because the path tracer accumulates samples across frames and a frame grabbed
mid-accumulation still carries the previous pose.

So the two things this tool must do that the old one did not are: follow, and
converge. Both are load-bearing, and the second is tested rather than asserted --
see `ghost_test()`.

THE GHOSTING TEST, AND WHY IT IS SHAPED LIKE THIS
-------------------------------------------------
"No trails" is hard to measure on a moving sequence, because a per-frame diff of
genuine motion is large by construction and says nothing about accumulation. The
question that IS decidable is whether a rendered frame depends on what was
rendered before it:

  1. put the robot at pose A, render, discard
  2. jump to pose B, render            -> frame X   (B, immediately after A)
  3. stay at B, render again           -> frame Y   (B, with A further behind)

If the renderer converges per frame, X and Y are the same picture and the mean
absolute difference is ~0. If it accumulates across frames, X still carries some
of A and X != Y. The score is `mean|X-Y| / 255`, and it is reported on every clip
this tool produces, not only when asked.

That also makes the failure legible: a large score names the mechanism (the frame
depends on its predecessor) rather than just saying "looks smeared".

CONVERGENCE
-----------
`rep.orchestrator.step(rt_subframes=N)` runs N path-tracer subframes for one
output frame. N is exposed as --subframes; the default is chosen by the ghost test
in `--calibrate` mode rather than guessed.

PiP
---
The D435i picture-in-picture track is NOT produced here on a box that cannot run
the camera (C-23). `--pip` raises rather than silently emitting a montage with a
missing track, and the campaign's status header lists PiP as a 4090 item.

USAGE
  # inside the sim container
  python3 film.py --shots chase,front,side --frames 600 --out /tmp/film/walk
  python3 film.py --calibrate                 # pick --subframes from the ghost test
  python3 film.py --ghost-test --out /tmp/film/ghost
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

from isaacsim import SimulationApp  # noqa: E402  (must precede other omni imports)

_ARGS_PRESCAN = " ".join(sys.argv)
app = SimulationApp({"headless": True})

import numpy as np                                          # noqa: E402
from isaacsim.core.api import World                         # noqa: E402
from isaacsim.core.prims import Articulation                # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.sensors.camera import Camera                  # noqa: E402
from PIL import Image                                       # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux                    # noqa: E402

RES = (1920, 1080)
ASSET_DEFAULT = "/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd"

# The sim's own standing init state (env.yaml). Used so a filmed robot looks like
# the robot every other gate measures, not like a T-pose.
STAND = {
    "left_hip_pitch_joint": -0.1, "right_hip_pitch_joint": -0.1,
    "left_knee_joint": 0.3, "right_knee_joint": 0.3,
    "left_ankle_pitch_joint": -0.2, "right_ankle_pitch_joint": -0.2,
    "left_shoulder_pitch_joint": 0.3, "right_shoulder_pitch_joint": 0.3,
    "left_shoulder_roll_joint": 0.25, "right_shoulder_roll_joint": -0.25,
    "left_elbow_joint": 0.97, "right_elbow_joint": 0.97,
    "left_wrist_roll_joint": 0.15, "right_wrist_roll_joint": -0.15,
}


# --------------------------------------------------------------------------
# camera authoring
# --------------------------------------------------------------------------
def look_at(cam_prim, eye, target, up=(0.0, 0.0, 1.0)):
    """Author a look-at directly on the prim.

    Not via Camera(orientation=...) or set_world_pose(): the wrapper silently
    ignores orientation (RESULTS_DT3 §4 F-6) and applies its own world-to-USD
    conversion on top of anything it does accept (C-21). Authoring the matrix is
    the one route that behaves.
    """
    eye = Gf.Vec3d(*[float(v) for v in eye])
    tgt = Gf.Vec3d(*[float(v) for v in target])
    fwd = (tgt - eye).GetNormalized()
    upv = Gf.Vec3d(*up)
    right = Gf.Cross(fwd, upv).GetNormalized()
    if right.GetLength() < 1e-6:                    # looking straight down
        right = Gf.Vec3d(1.0, 0.0, 0.0)
    trueup = Gf.Cross(right, fwd).GetNormalized()
    # USD cameras look along -Z with +Y up.
    m = Gf.Matrix4d(
        right[0], right[1], right[2], 0.0,
        trueup[0], trueup[1], trueup[2], 0.0,
        -fwd[0], -fwd[1], -fwd[2], 0.0,
        eye[0], eye[1], eye[2], 1.0)
    x = UsdGeom.Xformable(cam_prim)
    x.ClearXformOpOrder()
    x.AddTransformOp().Set(m)


class Shot:
    """One camera. `chase` follows the robot with lag; the others are bolted down."""

    def __init__(self, name, world, kind, res=RES):
        self.name, self.kind = name, kind
        path = f"/World/film_{name}"
        self.cam = Camera(prim_path=path, name=f"film_{name}", resolution=res)
        self.cam.initialize()
        self.cam.set_clipping_range(0.1, 200.0)
        self.prim = self.cam.prim
        self.lagged = None

    def place(self, robot_xy, robot_z, lag=0.12):
        x, y = float(robot_xy[0]), float(robot_xy[1])
        if self.kind == "chase":
            # Follow with lag: the camera eases toward the robot instead of being
            # welded to it, so the shot reads as a follow rather than a tripod
            # bolted to the pelvis.
            if self.lagged is None:
                self.lagged = np.array([x, y], dtype=float)
            else:
                self.lagged += lag * (np.array([x, y]) - self.lagged)
            lx, ly = self.lagged
            eye = (lx - 3.4, ly - 2.6, robot_z + 1.5)
            look_at(self.prim, eye, (x, y, robot_z + 0.15))
        elif self.kind == "front":
            look_at(self.prim, (x + 4.2, y, robot_z + 0.9), (x, y, robot_z + 0.1))
        elif self.kind == "side":
            look_at(self.prim, (x, y + 4.2, robot_z + 0.9), (x, y, robot_z + 0.1))
        elif self.kind == "top":
            look_at(self.prim, (x, y, robot_z + 6.0), (x, y, robot_z),
                    up=(1.0, 0.0, 0.0))
        else:
            raise SystemExit(f"unknown shot kind {self.kind!r}")

    def grab(self):
        a = self.cam.get_rgba()
        if a is None or a.size == 0 or a.ndim != 3:
            return None
        return a[:, :, :3].astype(np.uint8)


def scene(asset, world_usd=None):
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    UsdLux.DomeLight.Define(world.stage, Sdf.Path("/World/dome")).CreateIntensityAttr(1200.0)
    k = UsdLux.DistantLight.Define(world.stage, Sdf.Path("/World/key"))
    k.CreateIntensityAttr(2600.0)
    k.CreateAngleAttr(1.0)
    add_reference_to_stage(asset, "/World/G1")
    world.reset()
    art = Articulation("/World/G1")
    art.initialize()
    names = list(art.dof_names)
    q = art.get_joint_positions()
    q = np.array(q[0] if getattr(q, "ndim", 1) > 1 else q, dtype=np.float32)
    for jn, v in STAND.items():
        if jn in names:
            q[names.index(jn)] = v
    art.set_joint_positions(q[None, :] if q.ndim == 1 else q)
    world.step(render=False)
    return world, art


def robot_pose(art):
    p, _ = art.get_world_poses()
    p = np.asarray(p)[0]
    return (float(p[0]), float(p[1])), float(p[2])


def converge(subframes):
    """One output frame, with the path tracer given `subframes` to settle."""
    import omni.replicator.core as rep
    rep.orchestrator.step(rt_subframes=int(subframes), delta_time=0.0, pause_timeline=True)


# --------------------------------------------------------------------------
# the ghosting test
# --------------------------------------------------------------------------
def ghost_test(world, art, shot, subframes, out_dir, path="converged"):
    """Does a rendered frame depend on the frame before it? Lower is better.

    Returns the mean absolute difference, normalised to 0..1, between two renders
    of the SAME pose where only the pose that preceded them differs.
    """
    os.makedirs(out_dir, exist_ok=True)
    names = list(art.dof_names)
    q0 = np.asarray(art.get_joint_positions())
    q0 = q0[0] if q0.ndim > 1 else q0

    def render_at(joint, value):
        q = np.array(q0, dtype=np.float32)
        if joint in names:
            q[names.index(joint)] = value
        art.set_joint_positions(q[None, :])
        if path == "legacy":
            # The path render_orbit.py uses, and the one the DT progress montage
            # was shot with: step the world WITH rendering and grab whatever the
            # render product currently holds. No convergence barrier. This exists
            # as the NEGATIVE CONTROL -- a ghost test that has never been seen to
            # fail is not yet a test, and this is what makes it fail.
            world.step(render=True)
            shot.place(*robot_pose(art))
            return shot.grab()
        world.step(render=False)
        shot.place(*robot_pose(art))
        converge(subframes)
        return shot.grab()

    # A: a big, obvious arm swing. B: the pose we actually measure.
    render_at("left_shoulder_pitch_joint", -1.2)          # pose A, discarded
    x = render_at("left_shoulder_pitch_joint", 0.3)       # B, straight after A
    y = render_at("left_shoulder_pitch_joint", 0.3)       # B again, A further back
    if x is None or y is None:
        raise SystemExit("ghost test: renderer returned no frame")
    d = np.abs(x.astype(np.int16) - y.astype(np.int16)).mean() / 255.0
    Image.fromarray(x).save(os.path.join(out_dir, "ghost_x_after_jump.png"))
    Image.fromarray(y).save(os.path.join(out_dir, "ghost_y_settled.png"))
    diff = np.abs(x.astype(np.int16) - y.astype(np.int16)).astype(np.uint8)
    Image.fromarray((diff * 8).clip(0, 255).astype(np.uint8)).save(
        os.path.join(out_dir, "ghost_diff_x8.png"))
    return float(d)


GHOST_MAX = 0.01     # 1 % of full scale; a visible trail scores far above this


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--asset", default=os.environ.get("FILM_ASSET", ASSET_DEFAULT))
    ap.add_argument("--shots", default="chase,front,side")
    ap.add_argument("--frames", type=int, default=int(os.environ.get("FILM_FRAMES", "300")))
    ap.add_argument("--subframes", type=int, default=int(os.environ.get("FILM_SUBFRAMES", "16")))
    ap.add_argument("--out", default=os.environ.get("FILM_OUT", "/tmp/film/out"))
    ap.add_argument("--ghost-test", action="store_true")
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--negative-control", action="store_true",
                    help="also score the legacy (unconverged) render path, to show "
                         "the ghost test can fail")
    ap.add_argument("--pip", action="store_true")
    ap.add_argument("--drive", default="orbit_walk",
                    help="orbit_walk (scripted gait-like arm/leg swing) or hold")
    args = ap.parse_args()

    if args.pip:
        raise SystemExit(
            "--pip needs the D435i, and this box cannot run the camera (C-23). "
            "The PiP track is a 4090 item -- see docs/mm/CAMPAIGN.md status header.")

    os.makedirs(args.out, exist_ok=True)
    world, art = scene(args.asset)
    kinds = [k.strip() for k in args.shots.split(",") if k.strip()]
    shots = [Shot(k, world, k) for k in kinds]
    for s in shots:
        s.place(*robot_pose(art))
    for _ in range(6):                  # warm the render products
        converge(args.subframes)
        for s in shots:
            s.grab()

    report = {"asset": args.asset, "subframes": args.subframes,
              "shots": kinds, "ghost_max": GHOST_MAX}

    if args.calibrate:
        rows = []
        for n in (1, 4, 8, 16, 32, 64):
            g = ghost_test(world, art, shots[0], n,
                           os.path.join(args.out, f"cal_{n}"))
            rows.append({"subframes": n, "ghost": g, "pass": g <= GHOST_MAX})
            print(f"  subframes {n:3d}  ghost {g:.5f}  "
                  f"{'PASS' if g <= GHOST_MAX else 'FAIL'}", flush=True)
        ok = [r for r in rows if r["pass"]]
        report["calibration"] = rows
        report["recommended_subframes"] = ok[0]["subframes"] if ok else None
        print(f"\nrecommended --subframes: {report['recommended_subframes']}")
        json.dump(report, open(os.path.join(args.out, "film_report.json"), "w"), indent=2)
        app.close()
        return 0 if ok else 1

    if args.negative_control:
        gl = ghost_test(world, art, shots[0], args.subframes,
                        os.path.join(args.out, "ghost_legacy"), path="legacy")
        report["ghost_score_legacy"] = gl
        print(f"negative control (legacy unconverged path): ghost {gl:.5f} "
              f"{'FAIL as expected' if gl > GHOST_MAX else 'ALSO PASSES -- test has no power here'}",
              flush=True)

    g = ghost_test(world, art, shots[0], args.subframes,
                   os.path.join(args.out, "ghost"))
    report["ghost_score"] = g
    report["ghost_pass"] = g <= GHOST_MAX
    print(f"ghost score {g:.5f} (max {GHOST_MAX}) "
          f"{'PASS' if g <= GHOST_MAX else 'FAIL'}", flush=True)

    if args.ghost_test:
        json.dump(report, open(os.path.join(args.out, "film_report.json"), "w"), indent=2)
        app.close()
        return 0 if report["ghost_pass"] else 1

    # ---- the actual shoot --------------------------------------------------
    names = list(art.dof_names)
    q0 = np.asarray(art.get_joint_positions())
    q0 = q0[0] if q0.ndim > 1 else q0
    dirs = {k: os.path.join(args.out, k) for k in kinds}
    for d in dirs.values():
        os.makedirs(d, exist_ok=True)

    for i in range(args.frames):
        t = i / 30.0
        if args.drive == "orbit_walk":
            # A scripted gait-shaped swing. NOT the policy -- this is the tool's
            # own self-test motion, and any gate that films the real walk drives
            # the articulation itself and calls shoot_frame().
            q = np.array(q0, dtype=np.float32)
            for jn, amp, ph in (("left_shoulder_pitch_joint", 0.35, 0.0),
                                ("right_shoulder_pitch_joint", 0.35, math.pi),
                                ("left_hip_pitch_joint", 0.30, math.pi),
                                ("right_hip_pitch_joint", 0.30, 0.0),
                                ("left_knee_joint", 0.25, math.pi),
                                ("right_knee_joint", 0.25, 0.0)):
                if jn in names:
                    q[names.index(jn)] = q0[names.index(jn)] + amp * math.sin(2 * math.pi * 1.2 * t + ph)
            art.set_joint_positions(q[None, :])
        world.step(render=False)
        for s in shots:
            s.place(*robot_pose(art))
        converge(args.subframes)
        for s in shots:
            fr = s.grab()
            if fr is not None:
                Image.fromarray(fr).save(os.path.join(dirs[s.name], f"f{i:05d}.png"))
        if i % 30 == 0:
            print(f"  frame {i}/{args.frames}", flush=True)

    report["frames"] = args.frames
    report["dirs"] = dirs
    json.dump(report, open(os.path.join(args.out, "film_report.json"), "w"), indent=2)
    print(f"wrote {args.frames} frames x {len(shots)} shots -> {args.out}")
    app.close()
    return 0 if report["ghost_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
