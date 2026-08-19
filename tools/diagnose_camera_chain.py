"""C-21 diagnosis: print every rotation in the camera chain, a -> d.

(a) the camera prim's world pose in the STAGE, and its view axis
(b) the TF chain base_link -> camera_link -> camera_color_frame ->
    camera_color_optical_frame, as the CONTRACT declares it (which is what
    setup_tf_static publishes, and what the audit verified against the robot)
(c) the frame_id the images and the cloud carry
(d) the frame a back-projected depth pixel actually lands in

An optical convention applied twice shows up as an extra Rx(-90)Rz(-90), or as the
180-degree child rotation, appearing in one of these and not the others.
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import math, sys
import numpy as np
sys.path.insert(0, "/workspace/ferox_tools")
sys.path.insert(0, "/workspace/ferox_isaac/twin")
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Usd, UsdGeom
import twin_contract
import sensors as twin_sensors

out = open("/tmp/diag_c21.txt", "w")
def w(*a): out.write(" ".join(str(x) for x in a) + "\n"); out.flush()

def M(label, R):
    w(f"  {label}")
    for r in R:
        w("      [" + "  ".join(f"{v:+.6f}" for v in r) + "]")

def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx

CONTRACT = "/workspace/ferox_isaac/twin/g1_contract.yaml"
c = twin_contract.load(CONTRACT)
edges = {(e["parent"], e["child"]): e for e in c["tf_static"]}

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
add_reference_to_stage("/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd", "/World/G1")
world.reset()

cam, K_got = twin_sensors.create_camera(c, "/World/G1")
for _ in range(5):
    world.step(render=True)

w("=" * 78)
w("(b) THE CONTRACT / TF CHAIN  -- Class A, cannot change")
w("=" * 78)
chain = [("base_link", "camera_link"),
         ("camera_link", "camera_color_frame"),
         ("camera_color_frame", "camera_color_optical_frame")]
R_tf = np.eye(3)
for parent, child in chain:
    e = edges[(parent, child)]
    R = rpy_to_R(*e["rpy"])
    w(f"\n{parent} -> {child}   rpy={[round(v,6) for v in e['rpy']]}  xyz={e['xyz']}")
    M("R", R)
    R_tf = R_tf @ R
w("\nCOMPOSED base_link -> camera_color_optical_frame (per TF):")
M("R_tf", R_tf)
w(f"  optical +Z (view axis) expressed in base_link: "
  f"{np.round(R_tf @ np.array([0,0,1.0]), 6).tolist()}")
w(f"  optical +Y (image down)  in base_link: "
  f"{np.round(R_tf @ np.array([0,1.0,0]), 6).tolist()}")

w("")
w("=" * 78)
w("(a) THE STAGE  -- what Isaac actually has")
w("=" * 78)
stage = world.stage
cache = UsdGeom.XformCache(Usd.TimeCode.Default())
paths = {
    "base_link (pelvis prim)": "/World/G1/pelvis",
    "camera_link": "/World/G1/torso_link/camera_link",
    "camera_color_frame": "/World/G1/torso_link/camera_link/camera_color_frame",
    "camera_color_optical_frame":
        "/World/G1/torso_link/camera_link/camera_color_frame/camera_color_optical_frame",
    "camera prim": "/World/G1/torso_link/camera_link/camera_color_frame"
                   "/camera_color_optical_frame/camera",
}
Rw = {}
for label, p in paths.items():
    prim = stage.GetPrimAtPath(p)
    if not prim or not prim.IsValid():
        w(f"\n{label}: MISSING at {p}")
        continue
    m = cache.GetLocalToWorldTransform(prim)
    R = np.array([[m[0][0], m[1][0], m[2][0]],
                  [m[0][1], m[1][1], m[2][1]],
                  [m[0][2], m[1][2], m[2][2]]])
    Rw[label] = R
    t = m.ExtractTranslation()
    w(f"\n{label}  world xyz=({t[0]:+.5f}, {t[1]:+.5f}, {t[2]:+.5f})")
    M("R_world", R)

if "base_link (pelvis prim)" in Rw and "camera_color_optical_frame" in Rw:
    R_stage = Rw["base_link (pelvis prim)"].T @ Rw["camera_color_optical_frame"]
    w("\nSTAGE base_link -> camera_color_optical_frame:")
    M("R_stage", R_stage)
    w("\nDELTA  R_tf^T * R_stage   (identity => stage and TF agree):")
    D = R_tf.T @ R_stage
    M("delta", D)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(D) - 1) / 2))))
    w(f"  rotation angle of the delta: {ang:.4f} deg")

if "camera prim" in Rw and "camera_color_optical_frame" in Rw:
    R_child = Rw["camera_color_optical_frame"].T @ Rw["camera prim"]
    w("\nLOCAL camera_color_optical_frame -> camera prim (the 180 deg child):")
    M("R_child", R_child)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R_child) - 1) / 2))))
    w(f"  rotation angle: {ang:.4f} deg")
    w(f"  camera USD -Z (its view axis) in base_link: "
      f"{np.round(Rw['base_link (pelvis prim)'].T @ Rw['camera prim'] @ np.array([0,0,-1.0]), 6).tolist()}")
    w(f"  (compare with the TF optical +Z above -- these must be the SAME direction)")

out.close()
app.close()
