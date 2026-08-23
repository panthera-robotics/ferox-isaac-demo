"""Render a SCRIPTED pick for the montage. This is choreography, not a grasp.

CAMPAIGN 0.6 permits a cheat-attach for plumbing/telling the story, "logged as such, and
never counts as a grasp". This file is that: the can is parented to the palm for the carry
and the arm is driven through an interpolated trajectory. No contact, no grip, no IK.
It exists so the reel can show the INTENDED pipeline shape while the real grasp is
blocked on descent/IK (right_wrist_pitch/yaw pinned -- docs/mm/evidence/MM5/TASK1_IK_VERDICT.md).

Every frame is captioned by the montage build; the clip is never presented as a real grasp.
"""
import os, math
import numpy as np

OUT = os.environ.get("CHOREO_OUT", "/tmp/choreo")
os.makedirs(OUT, exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import sys
sys.path.insert(0, "/workspace/ferox_isaac")
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.storage.native import get_assets_root_path
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.stage as _st
from pxr import Gf, UsdGeom
import imageio.v2 as imageio

world = World(stage_units_in_meters=1.0, physics_dt=1/200.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
root = get_assets_root_path()
_st.add_reference_to_stage(root + "/Isaac/Environments/Simple_Room/simple_room.usd", "/World/Env")
prim = define_prim("/World/G1", "Xform")
prim.GetReferences().AddReference("/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd")
xf = UsdGeom.Xformable(world.stage.GetPrimAtPath("/World/G1"))
xf.ClearXformOpOrder(); xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.80))
_st.add_reference_to_stage(root + "/Isaac/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd", "/World/can")
cxf = UsdGeom.Xformable(world.stage.GetPrimAtPath("/World/can"))
cxf.ClearXformOpOrder()
can_t = cxf.AddTranslateOp(); can_t.Set(Gf.Vec3d(0.35, -0.22, 0.95))
world.reset(); world.play()

art = SingleArticulation("/World/G1"); art.initialize()
names = list(art.dof_names)
ARM = ["right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
       "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"]
aidx = np.array([names.index(n) for n in ARM if n in names], np.int32)
HAND = [n for n in names if n.startswith(("Yaw_1","Roll_1","Pitch_1","Roll_2","Pitch_2",
                                          "Roll_3","Pitch_3","Roll_4","Pitch_4","Roll_5","Pitch_5"))
        and n.endswith("R")]
hidx = np.array([names.index(n) for n in HAND], np.int32)

cam = Camera(prim_path="/World/choreo_cam", position=np.array([2.35, -2.05, 1.55]),
             frequency=30, resolution=(1280, 720))
cam.initialize()
from isaacsim.core.utils.rotations import euler_angles_to_quat
# FRAMING CHECKED ON A RENDERED STILL, not assumed. The first pose (1.6,-1.5,1.5) with
# pitch 0.30 put the robot at the very top edge and filled the frame with floor -- the
# ghost gate cannot see that and neither can a frame count. Pulled in and levelled so the
# torso, arm and can are all in shot.
cam.set_world_pose(np.array([2.35, -2.05, 1.55]),
                   euler_angles_to_quat(np.array([0.0, 0.16, 2.42])))

KEY = [   # (t, arm pose, grip 0..1) -- an interpolated CHOREOGRAPHY, not a solved reach
    (0.0,  [ 0.30,-0.25, 0.00, 0.97, 0.00, 0.00, 0.00], 0.0),
    (1.5,  [-0.30,-0.20, 0.15, 1.20, 0.00,-0.30, 0.00], 0.0),
    (3.0,  [-0.55,-0.15, 0.20, 1.35, 0.00,-0.45, 0.00], 0.0),
    (4.0,  [-0.55,-0.15, 0.20, 1.35, 0.00,-0.45, 0.00], 1.0),
    (5.5,  [-0.20,-0.20, 0.15, 1.05, 0.00,-0.25, 0.00], 1.0),
    (7.0,  [ 0.10,-0.45, 0.10, 0.95, 0.00,-0.15, 0.00], 1.0),
    (8.5,  [-0.35,-0.45, 0.15, 1.25, 0.00,-0.40, 0.00], 1.0),
    (9.5,  [-0.35,-0.45, 0.15, 1.25, 0.00,-0.40, 0.00], 0.0),
    (11.0, [ 0.30,-0.25, 0.00, 0.97, 0.00, 0.00, 0.00], 0.0),
]
FPS, DUR = 30, 11.0
n = int(FPS * DUR)
palm_prim = None
for p in world.stage.Traverse():
    if p.GetName() == "base_link00":
        palm_prim = p; break

def lerp(t):
    for i in range(len(KEY) - 1):
        t0, a0, g0 = KEY[i]; t1, a1, g1 = KEY[i + 1]
        if t0 <= t <= t1:
            u = (t - t0) / max(t1 - t0, 1e-6)
            u = u * u * (3 - 2 * u)
            return (np.array(a0) * (1 - u) + np.array(a1) * u, g0 * (1 - u) + g1 * u)
    return np.array(KEY[-1][1]), KEY[-1][2]

written = 0
for i in range(n):
    t = i / FPS
    a, g = lerp(t)
    # PIN THE BASE. Setting joint positions on a floating-base humanoid under gravity
    # does not hold it up -- the first render put the robot flat on the floor at the edge
    # of frame. This is a kinematic hold, exactly what the MM3/MM5 test rig does, and it
    # is declared: the clip is choreography, and the robot is not balancing in it.
    art.set_world_pose(np.array([0.0, 0.0, 0.80], np.float32),
                       np.array([1.0, 0.0, 0.0, 0.0], np.float32))
    try:
        art.set_linear_velocity(np.zeros(3, np.float32))
        art.set_angular_velocity(np.zeros(3, np.float32))
    except Exception:
        pass
    q = np.asarray(art.get_joint_positions(), np.float32).copy()
    q[aidx] = a[:len(aidx)]
    q[hidx] = 1.30 * g
    art.set_joint_positions(q)
    # CHEAT-ATTACH: the can follows the palm between close and release. Declared.
    if 4.0 <= t <= 9.5 and palm_prim is not None:
        m = UsdGeom.Xformable(palm_prim).ComputeLocalToWorldTransform(0)
        p = m.ExtractTranslation()
        can_t.Set(Gf.Vec3d(p[0] + 0.02, p[1] + 0.13, p[2] - 0.01))
    world.step(render=True)
    f = cam.get_rgba()
    a8 = None
    if f is not None:
        arr = np.asarray(f)
        # get_rgba can hand back an EMPTY or FLAT buffer on frames where the render
        # product has not resolved yet; slicing that as HxWx4 raises
        # "too many indices for array: array is 1-dimensional".
        if arr.ndim == 3 and arr.shape[-1] >= 3:
            a8 = arr[:, :, :3]
        elif arr.ndim == 1 and arr.size >= 1280 * 720 * 4:
            a8 = arr.reshape(720, 1280, 4)[:, :, :3]
    if a8 is not None and a8.size:
        if a8.dtype != np.uint8:
            a8 = (np.clip(a8, 0, 1) * 255).astype(np.uint8)
        imageio.imwrite(f"{OUT}/f{i:05d}.png", np.ascontiguousarray(a8)); written += 1
print(f"CHOREO wrote {written} frames (SCRIPTED PLACEMENT -- not a grasp)", flush=True)
app.close()
