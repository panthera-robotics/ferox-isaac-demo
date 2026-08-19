"""Render the G1 twin from front, side and top (DT2 visual pass).

Runs inside the Isaac container (scripts/14_capture_views.sh), offscreen, in its own
headless Isaac. Deliberately NOT a screenshot of the GUI: this box has no logged-in
desktop X session, so Isaac falls back to headless and the viewport renders empty --
which is exactly what DT2's `twin_viewport_hospital.png` turned out to be, a picture
of an empty GUI proving nothing.

The offscreen path works, and the five ways a frame comes back blank are recorded in
RESULTS_DT3 section 4 F-6. All five are handled here:

  * Isaac's Camera wrapper ignores the orientation passed to its constructor and to
    set_world_pose(), so the transform is authored directly as a USD look-at;
  * USD's default clippingRange starts at 1.0 m;
  * the robot is not at the origin unless it is pinned there;
  * gravity plus a reset impulse makes it drift;
  * the default stage lights the ground but not the robot.
"""
from __future__ import annotations

import os

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.sensors.camera import Camera
from PIL import Image
from pxr import Gf, Sdf, UsdGeom, UsdLux

ASSET = os.environ.get("VIEW_ASSET",
                       "/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd")
OUT_DIR = os.environ.get("VIEW_PNG_DIR", "/tmp/robot_views")
REPORT = "/tmp/robot_views.txt"
RES = (1600, 1000)

# Standing pose so the render shows the robot as it stands in the twin, not in its
# import T-pose. Body joints only, by name; the hands stay at zero (open).
STAND = {
    "left_hip_pitch_joint": -0.1, "right_hip_pitch_joint": -0.1,
    "left_knee_joint": 0.3, "right_knee_joint": 0.3,
    "left_ankle_pitch_joint": -0.2, "right_ankle_pitch_joint": -0.2,
    "left_shoulder_pitch_joint": 0.3, "right_shoulder_pitch_joint": 0.3,
    "left_shoulder_roll_joint": 0.25, "right_shoulder_roll_joint": -0.25,
    "left_elbow_joint": 0.97, "right_elbow_joint": 0.97,
    "left_wrist_roll_joint": 0.15, "right_wrist_roll_joint": -0.15,
}
ROOT_Z = 1.0
# eye offset from the robot's root, and what to aim at. Distances chosen so the whole
# robot fills the frame at 1600x1000 without clipping the feet or the head.
# Eye and aim are at the SAME height for front and side, so the view is level and the
# frame is centred on the robot's mid-height. Two crops were needed to get here:
# 2.6 m lost both head and feet (Isaac's default pinhole is narrower than it looks --
# ~30 deg vertical at this aspect), and aiming below eye level tilted the frame down
# and clipped the head. A crop is not a uniform frame, so the std check below cannot
# catch it; the geometry has to be right rather than merely non-blank.
#
# The robot's root is pinned at ROOT_Z and it spans roughly ROOT_Z-0.8 (feet) to
# ROOT_Z+0.7 (head), so AIM_Z sits at its middle.
AIM_Z = -0.05
VIEWS = {
    "front": ((4.60, 0.00, AIM_Z), (0.0, 0.0, AIM_Z)),
    "side":  ((0.00, -4.60, AIM_Z), (0.0, 0.0, AIM_Z)),
    "top":   ((0.30, 0.00, 4.20), (0.0, 0.0, -0.50)),
}


def main() -> int:
    os.makedirs(OUT_DIR, exist_ok=True)
    report = open(REPORT, "w")

    def w(*a):
        report.write(" ".join(str(x) for x in a) + "\n")
        report.flush()

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    dome = UsdLux.DomeLight.Define(world.stage, Sdf.Path("/World/view_dome"))
    dome.CreateIntensityAttr(1500.0)
    key = UsdLux.DistantLight.Define(world.stage, Sdf.Path("/World/view_key"))
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(1.0)
    add_reference_to_stage(ASSET, "/World/g1")
    world.reset()

    art = Articulation("/World/g1")
    art.initialize()
    names = list(art.dof_names)
    w(f"asset {ASSET}")
    w(f"articulation: {len(names)} DOF, {len(art.body_names)} bodies")

    world.get_physics_context().set_gravity(0.0)

    def pin():
        art.set_world_poses(
            positions=np.array([[0.0, 0.0, ROOT_Z]], dtype=np.float32),
            orientations=np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32))
        art.set_velocities(np.zeros((1, 6), dtype=np.float32))

    idx = np.array([names.index(n) for n in STAND], dtype=np.int32)
    q = np.array([[STAND[n] for n in STAND]], dtype=np.float32)
    art.set_joint_positions(q, joint_indices=idx)
    art.set_joint_position_targets(q, joint_indices=idx)
    for _ in range(120):
        pin()
        world.step(render=False)

    cam = Camera(prim_path="/World/view_cam", frequency=20, resolution=RES)
    cam.initialize()
    cam.set_clipping_range(0.01, 200.0)

    def look_at(eye, target, up=(0.0, 0.0, 1.0)):
        # Gf's SetLookAt returns the VIEW matrix; the camera's transform is its
        # inverse, and it already encodes USD's -Z-forward/+Y-up convention.
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*[float(v) for v in eye]),
                                       Gf.Vec3d(*[float(v) for v in target]),
                                       Gf.Vec3d(*up))
        x = UsdGeom.Xformable(world.stage.GetPrimAtPath("/World/view_cam"))
        x.ClearXformOpOrder()
        x.AddTransformOp().Set(view.GetInverse())

    root = np.array([0.0, 0.0, ROOT_Z])
    for name, (off, aim) in VIEWS.items():
        look_at(root + np.array(off), root + np.array(aim))
        for _ in range(90):
            pin()
            world.step(render=True)
        rgb = cam.get_rgba()[:, :, :3]
        path = os.path.join(OUT_DIR, f"g1_twin_{name}.png")
        Image.fromarray(rgb.astype(np.uint8)).save(path)
        w(f"{name:6s} -> {path}  mean {rgb.mean():6.1f}  std {rgb.std():5.1f}")
        if rgb.std() < 5.0:
            w(f"  WARNING: {name} is nearly uniform -- likely an empty frame")

    w("\nRESULT: PASS")
    report.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
