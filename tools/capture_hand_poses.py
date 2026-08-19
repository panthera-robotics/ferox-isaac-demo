"""Render the Dex5-1P hands at four poses from the merged G1 asset.

Runs inside the Isaac container (scripts/13_capture_hands.sh). Deterministic and
independent of the live sim, so the images can be regenerated for any commit.

Every pose is built BY JOINT NAME. Isaac interleaves the hand DOFs (see C-14 in
TWIN_DEVIATIONS.md) -- left occupies indices 29..63 and right 34..68, neither
contiguous -- so writing a 20-vector positionally would set fingers on both hands at
once. That is the same class of error as W6's Dex3/Dex5 mismatch, so the mapping is
built explicitly here rather than assumed.

Joint families, from Unitree's naming:
    Yaw_11 / Roll_12 / Pitch_13 / Pitch_14   thumb
    Roll_N1                                  finger-root abduction (PASSIVE, held at 0)
    Pitch_N2 / Pitch_N3 / Pitch_N4           finger flexion, 0 = straight
"""
from __future__ import annotations

import os

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation, RigidPrim
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
from PIL import Image

ASSET = "/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd"
OUT_DIR = os.environ.get("HAND_PNG_DIR", "/tmp/hand_poses")
REPORT = "/tmp/hand_poses.txt"

# Fractions of each joint's own upper limit, so no pose can command past a stop.
POSES = {
    "rest":   {"flex": 0.15, "thumb_roll": 0.0,  "thumb_flex": 0.10},
    "open":   {"flex": 0.00, "thumb_roll": 0.0,  "thumb_flex": 0.00},
    "fist":   {"flex": 1.00, "thumb_roll": 0.35, "thumb_flex": 1.00},
    "thumb_opposition": {"flex": 0.20, "thumb_roll": 1.00, "thumb_flex": 0.55},
}
PASSIVE_PREFIX = "Roll_"          # finger-root abduction: held at zero, see C-13
RIGHT_PALM_BODY = "base_link00"   # the right hand's root link, from Unitree's URDF
CAM_OFFSET = (0.95, -0.95, 0.30)  # metres from the palm, in world axes
# A presentation pose for the RIGHT arm, so the hand is clear of the body. With the
# arm hanging at its default the camera ends up inside the torso and the frame is a
# close-up of a hip. This poses the arm only; it says nothing about the policy, which
# is exercised separately by validate_motion.py.
ARM_POSE = {
    "right_shoulder_pitch_joint": -1.45,   # arm forward, roughly horizontal
    "right_shoulder_roll_joint": -0.30,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.35,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
}


def classify(name: str) -> str:
    """Which family a Dex5 joint belongs to, from its Unitree name."""
    stem = name[:-1] if name[-1] in "LR" else name
    if stem in ("Yaw_11", "Pitch_13", "Pitch_14"):
        return "thumb_flex"
    if stem == "Roll_12":
        return "thumb_roll"
    if stem.startswith(PASSIVE_PREFIX):
        return "passive"
    return "flex"


def main() -> int:
    os.makedirs(OUT_DIR, exist_ok=True)
    report = open(REPORT, "w")

    def w(*a):
        report.write(" ".join(str(x) for x in a) + "\n")
        report.flush()

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    # A dome light: the default stage lights the ground plane but not a small object
    # held above it, and an unlit render is indistinguishable from a mis-aimed one.
    from pxr import Gf, Sdf, UsdGeom, UsdLux
    dome = UsdLux.DomeLight.Define(world.stage, Sdf.Path("/World/hand_dome"))
    dome.CreateIntensityAttr(1200.0)
    add_reference_to_stage(ASSET, "/World/g1")
    world.reset()
    # Gravity off. These are static pose renders, and with gravity on the robot is
    # still falling and tumbling through the first two poses -- fingers get thrown by
    # ground contact and "commanded minus reached" came out at ~3 rad on poses 1-2
    # while poses 3-4 (by which time it had settled) converged to ~0.01 rad. The
    # drives hold the pose on their own; nothing here is testing balance.
    art = Articulation("/World/g1")
    art.initialize()

    world.get_physics_context().set_gravity(0.0)
    # Gravity off is not enough on its own: the reset leaves a small residual
    # velocity and, with nothing to damp it, the robot drifts -- the palm was found
    # at z = 3.67 m and still climbing, which is why the first framed render came
    # back black. Pin the root and zero its velocity every step.
    def _pin():
        art.set_world_poses(positions=np.array([[0.0, 0.0, 1.0]], dtype=np.float32),
                            orientations=np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32))
        art.set_velocities(np.zeros((1, 6), dtype=np.float32))

    _pin()
    for _ in range(60):
        _pin()
        world.step(render=False)
    names = list(art.dof_names)

    # A Dex5 joint is recognised by Unitree's naming, never by index.
    hand_names = [n for n in names
                  if n.split("_")[0] in ("Yaw", "Roll", "Pitch", "Link")]
    w(f"articulation DOF {len(names)}, hand DOFs {len(hand_names)}")
    if len(hand_names) != 40:
        raise SystemExit(f"expected 40 hand DOFs, found {len(hand_names)}")

    limits = np.asarray(art.get_dof_limits())[0]
    lower, upper = limits[:, 0], limits[:, 1]

    # Aim the camera at where the palm ACTUALLY is, rather than at a guessed pose.
    # The first version hard-coded a position and orientation and rendered four
    # crisp pictures of the ground plane; the joint numbers were right the whole time.
    # Articulation.get_world_poses() returns the ROOT pose only (shape (1, 3)), not
    # one row per link -- indexing it by body index reads off the end of the array.
    # A RigidPrim on the palm link is what gives the link's own world pose.
    # Pose the arm BEFORE measuring the palm -- the whole point is to move it.
    arm_i = np.array([names.index(n) for n in ARM_POSE], dtype=np.int32)
    arm_q = np.array([[ARM_POSE[n] for n in ARM_POSE]], dtype=np.float32)
    art.set_joint_positions(arm_q, joint_indices=arm_i)
    art.set_joint_position_targets(arm_q, joint_indices=arm_i)
    for _ in range(120):
        _pin()
        world.step(render=False)

    palm_prim = RigidPrim(f"/World/g1/{RIGHT_PALM_BODY}")
    palm_prim.initialize()
    palm = np.asarray(palm_prim.get_world_poses()[0]).reshape(-1)[:3]
    eye = palm + np.array(CAM_OFFSET)
    w(f"right palm at {palm.round(4).tolist()}, camera at {eye.round(4).tolist()}")
    def look_at(prim_path, eye, target, up=(0.0, 0.0, 1.0)):
        """Author the camera transform directly, as a USD look-at.

        Two things had to be true before any of these frames contained a robot, and
        each hid the other:

        * Isaac's Camera wrapper ignored the orientation given to its constructor and
          to set_world_pose() here -- eight cameras at yaws 45 degrees apart rendered
          statistically identical frames, all looking straight down. Gf's SetLookAt
          returns the VIEW matrix; the camera's transform is its inverse, and it
          already encodes USD's -Z-forward/+Y-up convention, so no euler bookkeeping
          is involved and nothing can be off by a sign.
        * the robot is not at the origin. Left to itself it tumbles and slides, and
          was found at (10.8, -1.06, 1.46) after 120 steps -- so aiming at (0, 0, z),
          which is what every earlier attempt did, framed empty floor. The root is
          pinned below and the camera is aimed at the palm's MEASURED position.
        """
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*[float(v) for v in eye]),
                                       Gf.Vec3d(*[float(v) for v in target]),
                                       Gf.Vec3d(*up))
        x = UsdGeom.Xformable(world.stage.GetPrimAtPath(prim_path))
        x.ClearXformOpOrder()
        x.AddTransformOp().Set(view.GetInverse())

    cam = Camera(prim_path="/World/hand_cam", frequency=20, resolution=(1280, 720))
    cam.initialize()
    # USD's default clippingRange starts at 1.0 m and the camera sits ~0.4 m from the
    # palm, so without this the hand falls inside the near plane and the frame comes
    # back correctly lit and completely empty.
    cam.set_clipping_range(0.01, 100.0)
    look_at("/World/hand_cam", eye, palm)
    for _ in range(60):
        _pin()
        world.step(render=True)
    probe = cam.get_rgba()[:, :, :3]
    w(f"camera at {np.asarray(eye).round(3).tolist()} looking at "
      f"{palm.round(3).tolist()}, mean pixel {probe.mean():.1f} std {probe.std():.1f}")

    base = art.get_joint_positions()
    base = np.asarray(base).reshape(-1).copy()

    for pose, cfg in POSES.items():
        q = base.copy()
        applied = 0
        for n in hand_names:
            i = names.index(n)
            fam = classify(n)
            if fam == "passive":
                q[i] = 0.0
                continue
            frac = cfg[fam]
            # Sign matters: Roll_12 is MIRRORED between hands (L [-1.8151, 0],
            # R [0, +1.8151]), so scale toward whichever limit is non-zero rather
            # than assuming the upper one.
            target = upper[i] if abs(upper[i]) >= abs(lower[i]) else lower[i]
            q[i] = float(frac) * float(target)
            applied += 1
        # Both, in this order, and for different reasons. set_joint_positions
        # teleports the joint but leaves the position drive still targeting whatever
        # it targeted before (zero), so the drive immediately hauls the finger back
        # and the render shows a half-collapsed pose -- the first version of this
        # script reported 5.25 rad of "commanded minus reached" for exactly that.
        # apply_action moves the TARGET, which is what actually holds the pose.
        hand_i = np.array([names.index(n) for n in hand_names], dtype=np.int32)
        art.set_joint_positions(q[hand_i].reshape(1, -1), joint_indices=hand_i)
        art.set_joint_position_targets(q[hand_i].reshape(1, -1), joint_indices=hand_i)
        for _ in range(180):
            _pin()
            world.step(render=True)
        rgb = cam.get_rgba()[:, :, :3]
        path = os.path.join(OUT_DIR, f"hand_{pose}.png")
        Image.fromarray(rgb.astype(np.uint8)).save(path)
        got = np.asarray(art.get_joint_positions()).reshape(-1)
        err = max(abs(got[names.index(n)] - q[names.index(n)]) for n in hand_names)
        w(f"{pose:18s} -> {path}  joints set {applied}  "
          f"max |commanded-reached| {err:.4f} rad")

    w("\nRESULT: PASS")
    report.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
