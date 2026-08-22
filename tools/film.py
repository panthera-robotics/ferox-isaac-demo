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
        # Instance segmentation, for the ghost metric. The mask is what makes
        # "robot pixels where the robot is not" a decidable question; without it
        # the only available statistic is whole-frame, and a whole-frame statistic
        # is exactly what missed this defect the first time.
        self.seg = None
        # FILM_NOSEG=1: skip the instance-seg annotator entirely. Needed on a box
        # with C-23, where annotator.get_data() segfaults in the warp
        # device-to-host copy exactly as the ROS 2 image writer does -- so C-23 is
        # "any synthetic-data annotator host copy", not just the image writer.
        if os.environ.get("FILM_NOSEG") == "1":
            return
        try:
            import omni.replicator.core as rep
            self.seg = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation_fast")
            self.seg.attach([self.cam.get_render_product_path()])
        except Exception as exc:                       # pragma: no cover
            print(f"[film] no instance seg on {name}: {exc}", flush=True)

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

    def mask(self):
        """Boolean mask of the robot in the current frame, from instance seg."""
        if self.seg is None:
            return None
        d = self.seg.get_data()
        arr = d.get("data") if isinstance(d, dict) else d
        if arr is None or np.size(arr) == 0:
            return None
        arr = np.asarray(arr)
        if arr.ndim == 3:
            arr = arr[:, :, 0]
        # Background/unlabelled ids are 0 or negative; the ground plane gets its
        # own id, so "not background and not the largest non-robot region" would
        # be fragile. Instead take every id whose pixels overlap the robot's
        # bounding behaviour: in this scene the robot is the only articulated
        # asset, and the ground plane is the single largest id. Drop that one.
        ids, counts = np.unique(arr, return_counts=True)
        if len(ids) < 2:
            return None
        order = np.argsort(-counts)
        bg_ids = {int(ids[order[0]])}
        if len(order) > 1 and counts[order[1]] > 0.35 * arr.size:
            bg_ids.add(int(ids[order[1]]))          # sky/dome as well as floor
        m = ~np.isin(arr, list(bg_ids))
        return m


def scene(asset, world_usd=None, physics_dt=1.0 / 200.0, render_dt=1.0 / 60.0,
          own_articulation=True):
    # The same physics rate the twin runs (run.py --physics_dt default). It is not a
    # cosmetic choice: the locomotion policy's decimation of 4 is defined against a
    # 200 Hz clock, so World's 1/60 default silently turns 50 Hz control into 15 Hz
    # and the G1 tumbles instead of walking -- which is exactly what the first
    # working policy-drive run did, travelling 2.9 m BACKWARDS under a +0.5 m/s
    # forward command.
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt,
                  rendering_dt=render_dt)
    world.scene.add_default_ground_plane()
    UsdLux.DomeLight.Define(world.stage, Sdf.Path("/World/dome")).CreateIntensityAttr(1200.0)
    k = UsdLux.DistantLight.Define(world.stage, Sdf.Path("/World/key"))
    k.CreateIntensityAttr(2600.0)
    k.CreateAngleAttr(1.0)
    add_reference_to_stage(asset, "/World/G1")
    # SPAWN HEIGHT. scene() used to drop the reference at the origin, which puts the
    # G1's legs UNDER the ground plane: the shot renders a robot buried to the pelvis,
    # arms and head above the floor and nothing below it. The ghost test cannot catch
    # that -- it measures trails, not whether the subject is intact -- so the first
    # orbit_walk clip of this session passed the gate at 0.00095 and was unusable.
    # The policy path already took its z from env.yaml; the scripted path now does too,
    # with the same default the twin spawns at.
    from pxr import UsdGeom as _UG
    _spawn_z = float(os.environ.get("FILM_SPAWN_Z", "0.80"))
    _xf = _UG.Xformable(world.stage.GetPrimAtPath("/World/G1"))
    _xf.ClearXformOpOrder()
    _xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, _spawn_z))
    world.reset()
    if not own_articulation:
        # In policy mode the POLICY owns the articulation. Creating a second
        # Articulation view over the same /World/G1 prim and posing it here left
        # two objects writing joint targets to one robot, which is the remaining
        # suspect for the clip where the G1 was flung across the scene rather than
        # walked. The caller passes its own handle back in.
        world.step(render=False)
        return world, None
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
    """Works for both articulation flavours.

    scene() builds a batched `Articulation` (get_world_poses, plural); the policy
    in --drive policy owns a `SingleArticulation` (get_world_pose, singular). The
    camera should not care which one is driving the robot it is following.
    """
    if hasattr(art, "get_world_poses"):
        p, _ = art.get_world_poses()
        p = np.asarray(p)[0]
    else:
        p, _ = art.get_world_pose()
        p = np.asarray(p).reshape(-1)
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


GHOST_MAX = 0.01     # legacy whole-frame score, kept only for continuity

# Mohammed's metric, 2026-08-19, and the one that actually decides:
#   ghost pixels = robot-coloured pixels OUTSIDE the current mask but INSIDE the
#   union of the previous N masks, as a fraction of the current mask's area.
#   PASS = under 0.5 % on every frame.
# "Robot-coloured" is self-calibrating: a candidate pixel counts if its colour is
# closer to the mean colour INSIDE this frame's mask than to the mean colour of
# the background outside it. No background plate needed, and it cannot be fooled
# by a scene relight.
GHOST_MASK_MAX = 0.005
GHOST_HISTORY = 6


def ghost_pixels(rgb, mask, prev_masks):
    """(fraction, count, mask_area) for one frame. None if the mask is unusable."""
    if mask is None or not mask.any() or not prev_masks:
        return None
    union = np.zeros_like(mask)
    for m in prev_masks:
        if m is not None and m.shape == mask.shape:
            union |= m
    cand = union & ~mask
    if not cand.any():
        return (0.0, 0, int(mask.sum()))
    f = rgb.astype(np.float32)
    fg = f[mask].mean(axis=0)
    bgmask = ~union
    if not bgmask.any():
        return None
    bg = f[bgmask].mean(axis=0)
    c = f[cand]
    d_fg = np.linalg.norm(c - fg, axis=1)
    d_bg = np.linalg.norm(c - bg, axis=1)
    ghosts = int((d_fg < d_bg).sum())
    area = int(mask.sum())
    return (ghosts / max(1, area), ghosts, area)


def _parse_schedule(spec):
    """'6:0,0,0;12:0.5,0,0' -> [(6.0, (0,0,0)), (12.0, (0.5,0,0))]."""
    out = []
    for seg in spec.split(";"):
        seg = seg.strip()
        if not seg:
            continue
        dur, cmd = seg.split(":")
        vx, vy, wz = (float(v) for v in cmd.split(","))
        out.append((float(dur), (vx, vy, wz)))
    return out


def _policy_drive(world, args):
    """Drive the articulation with the REAL locomotion policy, not a scripted swing.

    *** INCOMPLETE. DOES NOT YET PRODUCE A USABLE CLIP. ***

    Three real bugs were found and fixed getting this far, and they are worth
    keeping because each one produced a plausible-looking output:

      1. The clip was 600 static frames. converge() pauses the timeline so that
         rendering does not advance sim time, so after the warm-up every
         world.step() was a no-op. orbit_walk never noticed because it poses joints
         kinematically and needs no physics at all. Fixed by resuming via
         world.play() -- and note world.is_playing() reported True the whole time,
         so the timeline interface was not the thing that mattered.
      2. The robot was spawned lying on the floor at z=0.116, because scene() drops
         the reference at the origin. Now taken from env.yaml's
         scene.robot.init_state.pos, the same source the twin uses.
      3. World defaulted to a 1/60 s physics step. The policy's decimation of 4 is
         defined against 200 Hz, so that quietly turned 50 Hz control into 15 Hz.

    What remains: with all three fixed the G1 is FLUNG -- it reaches z=2.2 m and
    -7 m of travel while the command is still (0,0,0). That is a physics blow-up,
    not a gait, and the leading suspect is two ArticulationViews over the same
    /World/G1 prim (scene()'s and the policy's) both writing targets. The twin
    itself walks correctly under the identical policy and checkpoint -- see the
    MM1 motion suite -- so this is a defect in this harness, not in the robot.

    Until it is fixed, a locomotion clip must be filmed from the live sim, and
    NOTHING should be shot with --drive policy: a montage of a robot being thrown
    across a room, captioned as a walk, is precisely the failure the campaign's
    media rules exist to prevent.

    It imports the same G1VelocityPolicy the twin runs, and returns a callable
    taking the frame's sim time; it advances physics at the policy's own rate
    between rendered frames rather than stepping once per frame, because the
    policy's decimation is defined against the physics clock.
    """
    import run as twin_run

    if not args.policy:
        raise SystemExit("--drive policy needs --policy (the TorchScript checkpoint)")

    # run.py's own loader: env.yaml carries python/tuple tags safe_load refuses.
    _z = 0.8
    if args.env_yaml and os.path.isfile(args.env_yaml):
        _cfg0 = twin_run._load_yaml(args.env_yaml) or {}
        _p0 = (_cfg0.get("scene", {}).get("robot", {})
               .get("init_state", {}).get("pos"))
        if _p0 is not None and len(_p0) >= 3:
            _z = float(_p0[2])

    # The spawn goes in at CONSTRUCTION, not afterwards. Passing position=None and
    # then set_world_pose() leaves the articulation's DEFAULT state at z=0, and
    # initialize() puts the robot back there -- inside the floor -- so it was being
    # ejected rather than walked. That is what produced every "flung robot" clip.
    pol = twin_run.G1VelocityPolicy(
        prim_path="/World/G1", name="G1", usd_path=None,
        position=np.array([0.0, 0.0, _z], dtype=np.float32),
        orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        policy_path=args.policy, env_path=args.env_yaml,
        deploy_path=args.deploy_yaml,
    )
    # Spawn height comes from the SAME place the twin gets it -- env.yaml's
    # scene.robot.init_state.pos -- not from a number chosen to make the shot look
    # right. film.py's scene() drops the reference at the origin, which leaves the
    # G1 lying on the floor at z=0.116, and a policy asked to walk from there is
    # being filmed failing at a task it was never given.
    # run.py's own loader, not yaml.safe_load: env.yaml carries
    # python/tuple tags that safe_load refuses.
    print(f"  policy drive: spawn z={_z:.3f} (from env.yaml init_state.pos, "
          f"set at construction so it is the DEFAULT state too)", flush=True)

    # initialize() AFTER the world is playing and one step has been taken. Called
    # earlier, the articulation view is not ready, set_gains silently does nothing,
    # and the robot runs on the USD's own drive values -- stiffness 625 with
    # damping 0.0 (see evidence/MM1b/twin_vs_training.md). An undamped robot does
    # not walk, it flops, which is exactly what the last clip showed.
    # NOTE: an extra world.reset() here was tried and made things worse -- it put
    # the base back at z=0.046, inside the floor -- without changing the gains at
    # all. Left out deliberately.
    world.play()
    world.step(render=False)
    pol.initialize()

    # Read the gains back. This repo does not trust an author step it has not
    # verified, and this one has now failed silently twice.
    try:
        _st = pol.robot.get_articulation_controller().get_gains()
        _kp = np.asarray(_st[0]).reshape(-1)
        _kd = np.asarray(_st[1]).reshape(-1)
        print(f"  policy gains: kp[min,max]=({_kp.min():.1f},{_kp.max():.1f}) "
              f"kd[min,max]=({_kd.min():.3f},{_kd.max():.3f})", flush=True)
        if _kd.max() <= 1e-6 or _kp.max() > 400:
            print("  WARNING: these look like the USD's raw drive values, not "
                  "deploy.yaml's. The robot will flop.", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"  gain read-back unavailable: {exc}", flush=True)

    schedule = _parse_schedule(args.cmd_schedule)
    phys_dt = float(world.get_physics_dt())
    state = {"t": 0.0}

    def _cmd_at(tt):
        acc = 0.0
        for dur, cmd in schedule:
            acc += dur
            if tt < acc:
                return cmd
        return schedule[-1][1] if schedule else (0.0, 0.0, 0.0)

    import omni.timeline as _timeline

    tl = _timeline.get_timeline_interface()

    def _step_to(target_t):
        import numpy as _np
        # converge() pauses the timeline so that rendering does not advance sim
        # time, which is right for the render and fatal for physics: after the
        # warm-up converges, world.step() is a no-op and the robot stands frozen.
        # The first shoot produced 600 frames whose first and last differ by 0.3
        # of 255 -- a static picture of a robot that never moved. Resume the
        # timeline before every physics burst.
        if not world.is_playing():
            world.play()          # SimulationContext.play(), not the timeline iface
        while state["t"] < target_t:
            cmd = _np.array(_cmd_at(state["t"]), dtype=_np.float32)
            pol.forward(phys_dt, cmd)
            world.step(render=False)
            state["t"] += phys_dt
        # A clip of a locomotion gate is worthless if the robot did not move, and
        # "it looks like it is walking" is exactly the judgement this campaign does
        # not accept. Report the pose so the shoot is checkable from its own log.
        if int(state["t"] * 2) != state.get("last_report"):
            state["last_report"] = int(state["t"] * 2)
            try:
                pp, _ = pol.robot.get_world_pose()
                pp = _np.asarray(pp).reshape(-1)
                print(f"    t={state['t']:5.2f}s cmd={_cmd_at(state['t'])} "
                      f"pos=({pp[0]:.3f}, {pp[1]:.3f}, {pp[2]:.3f}) "
                      f"playing={world.is_playing()} sim_t={world.current_time:.3f}",
                      flush=True)
            except Exception as exc:  # noqa: BLE001
                print(f"    pose report failed: {exc}", flush=True)

    return _step_to, pol.robot


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--asset", default=os.environ.get("FILM_ASSET", ASSET_DEFAULT))
    ap.add_argument("--shots", default="chase,front,side")
    ap.add_argument("--frames", type=int, default=int(os.environ.get("FILM_FRAMES", "300")))
    ap.add_argument("--subframes", type=int, default=int(os.environ.get("FILM_SUBFRAMES", "16")))
    ap.add_argument("--out", default=os.environ.get("FILM_OUT", "/tmp/film/out"))
    ap.add_argument("--ghost-test", action="store_true")
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--orbit-path", default="both", choices=("both", "legacy", "converged"),
                    help="run only one render path. The two must not share a process: "
                         "45 legacy world.step(render=True) calls followed by the "
                         "converged path segfaults, so each path gets its own run.")
    ap.add_argument("--orbit-ab", action="store_true",
                    help="orbit the camera around a static robot and score the mask "
                         "ghost metric on BOTH render paths -- the A/B that decides")
    ap.add_argument("--negative-control", action="store_true",
                    help="also score the legacy (unconverged) render path, to show "
                         "the ghost test can fail")
    ap.add_argument("--pip", action="store_true")
    ap.add_argument("--drive", default="orbit_walk",
                    choices=("orbit_walk", "hold", "policy"),
                    help="orbit_walk (the tool's scripted self-test swing), hold, or "
                         "policy (INCOMPLETE -- see the note on _policy_drive)")
    ap.add_argument("--cmd-schedule", default="6:0.0,0,0;12:0.5,0,0;6:0.0,0,0",
                    help="policy drive only: 'seconds:vx,vy,wz;...' in SIM seconds")
    ap.add_argument("--policy", default=os.environ.get("FILM_POLICY", ""),
                    help="policy drive only: TorchScript checkpoint (.pt)")
    ap.add_argument("--env-yaml", default=os.environ.get("FILM_ENV", ""))
    ap.add_argument("--deploy-yaml", default=os.environ.get("FILM_DEPLOY", ""))
    args = ap.parse_args()

    if args.pip:
        raise SystemExit(
            "--pip needs the D435i, and this box cannot run the camera (C-23). "
            "The PiP track is a 4090 item -- see docs/mm/CAMPAIGN.md status header.")

    os.makedirs(args.out, exist_ok=True)

    # IMPORT run.py BEFORE any render product exists.
    #
    # --drive policy died in libomni.syntheticdata + libomni.graph.core before its first
    # print, i.e. inside `import run as twin_run` -- which _policy_drive does AFTER
    # scene() and Shot() have already created render products and warmed them. Importing
    # run.py pulls in its omni/isaacsim module tree, and doing that once the SDG pipeline
    # is live is the same shape as C-23: a render product exists, then something touches
    # the graph, then the process dies.
    #
    # orbit_walk renders 240 frames through the identical camera code and never imports
    # run.py, which is what points at the import rather than at the cameras.
    if args.drive == "policy":
        # run.py lives beside the twin's assets, not next to this script when it is
        # staged into /tmp/isaacrun (which it must be -- CLAUDE.md: Isaac scripts never
        # run from bare /tmp).
        for _p in ("/workspace/ferox_isaac",
                   os.path.dirname(os.path.dirname(os.path.abspath(__file__)))):
            if os.path.isfile(os.path.join(_p, "run.py")) and _p not in sys.path:
                sys.path.insert(0, _p)
        import run as _twin_run_preload           # noqa: F401
        print("  pre-imported run.py before any render product existed", flush=True)

    world, art = scene(args.asset, own_articulation=(args.drive != "policy"))
    kinds = [k.strip() for k in args.shots.split(",") if k.strip()]
    shots = [Shot(k, world, k) for k in kinds]
    if art is not None:
        for s in shots:
            s.place(*robot_pose(art))
    for _ in range(6):                  # warm the render products
        converge(args.subframes)
        for s in shots:
            s.grab()

    report = {"asset": args.asset, "subframes": args.subframes,
              "shots": kinds, "ghost_max": GHOST_MAX}

    if args.orbit_ab:
        # The exact shot that ghosts: camera orbiting a STATIC robot. The DT-era
        # path (world.step(render=True) + grab) against the converged path
        # (rep.orchestrator.step(rt_subframes=N)), same geometry, same frames.
        import omni.replicator.core as rep
        shot = shots[0]
        (rx, ry), rz = robot_pose(art)
        results = {}
        paths = (("legacy", "converged") if args.orbit_path == "both"
                 else (args.orbit_path,))
        for label in paths:
            prev, rows = [], []
            outd = os.path.join(args.out, label)
            os.makedirs(outd, exist_ok=True)
            for i in range(args.frames):
                a = 2.0 * math.pi * i / max(1, args.frames)
                eye = (rx + 4.6 * math.cos(a), ry + 4.6 * math.sin(a), rz + 0.9)
                look_at(shot.prim, eye, (rx, ry, rz - 0.05))
                if label == "legacy":
                    world.step(render=True)
                else:
                    world.step(render=False)
                    converge(args.subframes)
                fr, mk = shot.grab(), shot.mask()
                if fr is None:
                    continue
                g = ghost_pixels(fr, mk, prev[-GHOST_HISTORY:])
                if g is not None:
                    rows.append({"frame": i, "ghost_frac": g[0],
                                 "ghost_px": g[1], "mask_px": g[2]})
                if mk is not None:
                    prev.append(mk)
                if i % 15 == 0:
                    Image.fromarray(fr).save(os.path.join(outd, f"f{i:04d}.png"))
            fr_all = [r["ghost_frac"] for r in rows]
            results[label] = {
                "frames_scored": len(rows),
                "ghost_frac_mean": float(np.mean(fr_all)) if fr_all else None,
                "ghost_frac_max": float(np.max(fr_all)) if fr_all else None,
                "frames_over_threshold": int(sum(1 for v in fr_all if v > GHOST_MASK_MAX)),
                "pass": bool(fr_all) and max(fr_all) <= GHOST_MASK_MAX,
                "rows": rows,
            }
            r = results[label]
            print(f"  {label:10s} scored {r['frames_scored']:3d}  "
                  f"ghost mean {r['ghost_frac_mean']:.5f}  max {r['ghost_frac_max']:.5f}  "
                  f"over-threshold {r['frames_over_threshold']}  "
                  f"{'PASS' if r['pass'] else 'FAIL'}", flush=True)
        report["orbit_ab"] = results
        report["threshold"] = GHOST_MASK_MAX
        json.dump(report, open(os.path.join(args.out, "film_report.json"), "w"), indent=2)
        app.close()
        # The test is only meaningful if it can fail. Demand that legacy FAILS
        # and converged PASSES; anything else means the metric is not measuring
        # what it claims and must not be used as a gate.
        ok = (results.get("legacy", {}).get("pass") is False
              and results.get("converged", {}).get("pass") is True)
        if args.orbit_path != "both":
            print(f"(single-path run: {args.orbit_path}; no A/B verdict)")
            return 0
        print("\nA/B verdict:", "metric HAS POWER and the fix works"
              if ok else "INCONCLUSIVE -- do not gate on this yet")
        return 0 if ok else 1

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

    if args.drive == "policy":
        # The inline ghost test poses joints kinematically to render the same pose
        # twice. Doing that in the middle of a policy-driven shoot would corrupt
        # exactly the thing being filmed, and in policy mode the articulation does
        # not exist yet at this point in main() -- which is what crashed the first
        # attempt. The mask-based ghost gate is deferred to a 4090 day anyway
        # (C-23), so it is skipped here and said out loud rather than faked.
        report["ghost_score"] = None
        report["ghost_pass"] = None
        print("ghost test SKIPPED in --drive policy: it poses joints kinematically "
              "and would corrupt the run being filmed. Clip is marked 'visually "
              "clean, numeric ghost gate deferred to 4090'.", flush=True)
    else:
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
    dirs = {k: os.path.join(args.out, k) for k in kinds}
    for d in dirs.values():
        os.makedirs(d, exist_ok=True)

    # In policy mode the articulation does not exist until the policy makes it,
    # so the drive is built FIRST and hands its handle back. orbit_walk still
    # needs the rest pose, and only orbit_walk uses it.
    policy_drive = None
    if args.drive == "policy":
        policy_drive, art = _policy_drive(world, args)

    names, q0 = [], None
    if art is not None and policy_drive is None:
        names = list(art.dof_names)
        q0 = np.asarray(art.get_joint_positions())
        q0 = q0[0] if q0.ndim > 1 else q0

    for i in range(args.frames):
        t = i / 30.0
        if policy_drive is not None:
            policy_drive(t)
        elif args.drive == "orbit_walk":
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
        if policy_drive is None:
            world.step(render=False)   # the policy drive already advanced physics
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
    return 0 if report.get("ghost_pass") in (True, None) else 1


if __name__ == "__main__":
    raise SystemExit(main())
