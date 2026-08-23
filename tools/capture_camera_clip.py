"""Capture the twin's D435i colour + depth to PNG frames for the montage PiP track.

Uses the offscreen annotator route (C-23 is an OmniGraph image-writer path; the fix was
headless:False, and this route avoids the writer entirely). Depth is colourised for
display only -- the numeric depth is 16UC1 millimetres and is what the aligned-depth
check reads.
"""
import os
import numpy as np

OUT = os.environ.get("CAMCLIP_OUT", "/tmp/camclip")
N = int(os.environ.get("CAMCLIP_FRAMES", "150"))
os.makedirs(OUT + "/rgb", exist_ok=True)
os.makedirs(OUT + "/depth", exist_ok=True)

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import sys
sys.path.insert(0, "/workspace/ferox_isaac")
sys.path.insert(0, "/workspace/ferox_isaac/twin")
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge"); app.update()
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.storage.native import get_assets_root_path
import isaacsim.core.utils.stage as _st
from pxr import Gf, UsdGeom
import yaml, imageio.v2 as imageio

world = World(stage_units_in_meters=1.0, physics_dt=1/200.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
_st.add_reference_to_stage(get_assets_root_path() + "/Isaac/Environments/Hospital/hospital.usd", "/World/Env")
prim = define_prim("/World/G1", "Xform")
prim.GetReferences().AddReference("/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd")
xf = UsdGeom.Xformable(world.stage.GetPrimAtPath("/World/G1"))
xf.ClearXformOpOrder(); xf.AddTranslateOp().Set(Gf.Vec3d(7.8, 2.0, 0.8))
world.reset()

contract = yaml.safe_load(open("/workspace/ferox_isaac/twin/g1_contract.yaml"))
from twin import sensors as twin_sensors
from twin import camera_annotator as cam_ann
cam, K = twin_sensors.create_camera(contract, "/World/G1")
oc = cam_ann.AnnotatorCamera(route="annotator"); oc.attach(cam)

import math
n_rgb = n_d = 0
for i in range(N):
    # slow yaw so the PiP track has motion without needing the policy
    ang = 0.6 * math.sin(i / 25.0)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(7.8, 2.0, 0.8))
    xf.AddRotateZOp().Set(math.degrees(ang))
    world.step(render=True)
    rgb = oc.rgb()
    if rgb is not None:
        imageio.imwrite(f"{OUT}/rgb/f{i:05d}.png", rgb); n_rgb += 1
    d = oc.depth_mm()
    if d is not None:
        dm = np.clip(d.astype(np.float32), 0, 4000) / 4000.0
        img = (np.stack([dm, 1.0 - dm, np.zeros_like(dm)], -1) * 255).astype(np.uint8)
        imageio.imwrite(f"{OUT}/depth/f{i:05d}.png", img); n_d += 1
print(f"CAMCLIP wrote rgb={n_rgb} depth={n_d} K.fx={K['fx']:.1f}", flush=True)
app.close()
