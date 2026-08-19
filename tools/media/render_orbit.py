"""Orbit-render a twin robot offscreen, optionally with sensor frames drawn.

Runs inside the Isaac container. Offscreen only -- this box has no logged-in desktop
X session, so any GUI viewport renders empty. Same five blank-frame traps handled as
tools/capture_robot_views.py: the Camera wrapper ignores orientation (so the transform
is authored as a USD look-at), USD's near clip starts at 1.0 m, the robot is not at the
origin unless pinned, gravity plus a reset impulse makes it drift, and the default
stage lights the ground but not the robot.

  ROBOT=g1|go2  FRAMES=n  AXES=0|1  OUT=/tmp/orbit  python3 render_orbit.py

With AXES=1 the sensor frames are drawn as real RGB axis triads at their authored
prim paths -- so what you see is where the sensor actually is, not an annotation.
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import math, os, sys
import numpy as np
sys.path.insert(0, "/workspace/ferox_tools")
sys.path.insert(0, "/workspace/ferox_isaac/twin")
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.sensors.camera import Camera
from PIL import Image
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

ROBOT = os.environ.get("ROBOT", "g1")
FRAMES = int(os.environ.get("FRAMES", "90"))
AXES = os.environ.get("AXES", "0") == "1"
OUT = os.environ.get("OUT", "/tmp/orbit")
RES = (1920, 1080)

ASSET = {
    "g1": "/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd",
    "go2": "/workspace/ferox_isaac/assets/go2/usd/go2.usd",
}[ROBOT]
# Frames to mark, and the colour of the triad. Paths are the AUTHORED prim paths, so
# a frame that is not in the asset fails loudly rather than being quietly skipped.
FRAME_PRIMS = {
    "g1": [("torso_link/livox_frame", (0.1, 0.9, 0.2)),
           ("torso_link/camera_link", (0.2, 0.5, 1.0)),
           ("pelvis/dog_imu_link", (1.0, 0.6, 0.1))],
    "go2": [("base/livox_frame", (0.1, 0.9, 0.2)),
            ("base/utlidar_imu", (1.0, 0.6, 0.1)),
            ("base/robot_center", (0.9, 0.2, 0.9))],
}[ROBOT]
STAND = {
    "g1": {"left_hip_pitch_joint": -0.1, "right_hip_pitch_joint": -0.1,
           "left_knee_joint": 0.3, "right_knee_joint": 0.3,
           "left_ankle_pitch_joint": -0.2, "right_ankle_pitch_joint": -0.2,
           "left_shoulder_pitch_joint": 0.3, "right_shoulder_pitch_joint": 0.3,
           "left_shoulder_roll_joint": 0.25, "right_shoulder_roll_joint": -0.25,
           "left_elbow_joint": 0.97, "right_elbow_joint": 0.97,
           "left_wrist_roll_joint": 0.15, "right_wrist_roll_joint": -0.15},
    "go2": {},
}[ROBOT]
ROOT_Z = {"g1": 1.00, "go2": 0.42}[ROBOT]
RADIUS = {"g1": 4.6, "go2": 2.6}[ROBOT]
AIM_Z = {"g1": -0.05, "go2": 0.0}[ROBOT]


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    UsdLux.DomeLight.Define(world.stage, Sdf.Path("/World/dome")).CreateIntensityAttr(1500.0)
    k = UsdLux.DistantLight.Define(world.stage, Sdf.Path("/World/key"))
    k.CreateIntensityAttr(2500.0); k.CreateAngleAttr(1.0)
    add_reference_to_stage(ASSET, "/World/R")
    world.reset()

    art = Articulation("/World/R"); art.initialize()
    names = list(art.dof_names)
    print(f"{ROBOT}: {len(names)} DOF, {len(art.body_names)} bodies", flush=True)

    world.get_physics_context().set_gravity(0.0)

    def pin():
        art.set_world_poses(
            positions=np.array([[0.0, 0.0, ROOT_Z]], dtype=np.float32),
            orientations=np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32))
        art.set_velocities(np.zeros((1, 6), dtype=np.float32))

    if STAND:
        idx = np.array([names.index(n) for n in STAND if n in names], dtype=np.int32)
        q = np.array([[STAND[n] for n in STAND if n in names]], dtype=np.float32)
        art.set_joint_positions(q, joint_indices=idx)
        art.set_joint_position_targets(q, joint_indices=idx)
    for _ in range(90):
        pin(); world.step(render=False)

    if AXES:
        stage = world.stage
        made = 0
        for rel, col in FRAME_PRIMS:
            base = f"/World/R/{rel}"
            if not stage.GetPrimAtPath(base).IsValid():
                raise SystemExit(f"frame prim missing: {base} -- run 08_build_twin_assets.sh")
            for ax, (dx, dy, dz) in enumerate(((1, 0, 0), (0, 1, 0), (0, 0, 1))):
                p = f"{base}/axis_{ax}"
                cube = UsdGeom.Cube.Define(stage, Sdf.Path(p))
                cube.CreateSizeAttr(1.0)
                x = UsdGeom.Xformable(cube)
                x.ClearXformOpOrder()
                L = 0.11
                x.AddTranslateOp().Set(Gf.Vec3d(dx*L/2, dy*L/2, dz*L/2))
                x.AddScaleOp().Set(Gf.Vec3f(dx*L + 0.008, dy*L + 0.008, dz*L + 0.008))
                c = [col[0]*0.35, col[1]*0.35, col[2]*0.35]
                c[ax] = 1.0
                cube.CreateDisplayColorAttr([Gf.Vec3f(*c)])
                made += 1
        print(f"drew {made} axis bars on {len(FRAME_PRIMS)} frames", flush=True)

    cam = Camera(prim_path="/World/orbitcam", frequency=20, resolution=RES)
    cam.initialize(); cam.set_clipping_range(0.01, 200.0)
    # Warm the render product. get_rgba() returns a 1-D empty array for the first
    # frames, and indexing it [:, :, :3] raises "too many indices" -- which reads like
    # a shape bug in the caller and is actually just "not ready yet".
    for _ in range(20):
        pin(); world.step(render=True)

    def grab():
        for _ in range(30):
            a = cam.get_rgba()
            if getattr(a, "ndim", 0) == 3:
                return a[:, :, :3]
            pin(); world.step(render=True)
        raise SystemExit("camera never produced a 3-D frame")
    target = np.array([0.0, 0.0, ROOT_Z + AIM_Z])

    def look_at(eye):
        v = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*[float(a) for a in eye]),
                                    Gf.Vec3d(*[float(a) for a in target]),
                                    Gf.Vec3d(0, 0, 1))
        x = UsdGeom.Xformable(world.stage.GetPrimAtPath("/World/orbitcam"))
        x.ClearXformOpOrder(); x.AddTransformOp().Set(v.GetInverse())

    # front -> side -> top, as one continuous move: azimuth 0->-90 while the
    # elevation lifts on the last third.
    for i in range(FRAMES):
        f = i / max(1, FRAMES - 1)
        az = math.radians(-90.0 * min(1.0, f / 0.66))
        el = math.radians(58.0 * max(0.0, (f - 0.66) / 0.34))
        r = RADIUS * (1.0 - 0.12 * math.sin(math.pi * f))
        eye = target + np.array([r*math.cos(az)*math.cos(el),
                                 r*math.sin(az)*math.cos(el),
                                 r*math.sin(el)])
        look_at(eye)
        pin(); world.step(render=True)
        world.step(render=True)          # let the RTX accumulate before grabbing
        rgb = grab()
        Image.fromarray(rgb.astype(np.uint8)).save(f"{OUT}/f{i:04d}.png")
        if i % 20 == 0:
            print(f"  frame {i}/{FRAMES} mean {rgb.mean():.1f}", flush=True)
    print(f"wrote {FRAMES} frames to {OUT}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
