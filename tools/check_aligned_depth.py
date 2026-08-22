"""MM0.2 aligned-depth check — the item C-23 blocked since 2026-08-19.

Runs the twin's OWN camera factory and the offscreen annotator route (no OmniGraph image
writer), then reports what the depth frame actually contains: min/median/max in
millimetres, the zero fraction, and whether colour and depth agree in shape so a pixel in
one indexes the same ray in the other.

Headless by construction. C-23 was `headless: False` asking Kit for a windowed renderer
on a box with no logged-in X session; see run.py's TWIN_HEADLESS note.
"""
import json
import numpy as np

out = {}
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import sys, os
sys.path.insert(0, "/workspace/ferox_isaac")
sys.path.insert(0, "/workspace/ferox_isaac/twin")
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge"); app.update()
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.storage.native import get_assets_root_path
import isaacsim.core.utils.stage as _st
import yaml

world = World(stage_units_in_meters=1.0, physics_dt=1/200.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()
_st.add_reference_to_stage(get_assets_root_path() + "/Isaac/Environments/Hospital/hospital.usd", "/World/Env")
prim = define_prim("/World/G1", "Xform")
prim.GetReferences().AddReference("/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd")
from pxr import Gf, UsdGeom
xf = UsdGeom.Xformable(world.stage.GetPrimAtPath("/World/G1"))
xf.ClearXformOpOrder(); xf.AddTranslateOp().Set(Gf.Vec3d(7.8, 2.0, 0.8))
world.reset()

contract = yaml.safe_load(open("/workspace/ferox_isaac/twin/g1_contract.yaml"))
from twin import sensors as twin_sensors
from twin import camera_annotator as cam_ann
cam, K = twin_sensors.create_camera(contract, "/World/G1")
oc = cam_ann.AnnotatorCamera(route="annotator")
oc.attach(cam)
for _ in range(12):
    world.step(render=True)

rgb = oc.rgb()
d = oc.depth_mm()
out["K"] = {k: float(v) for k, v in K.items()}
out["rgb_shape"] = None if rgb is None else list(rgb.shape)
out["depth_shape"] = None if d is None else list(d.shape)
if d is not None:
    nz = d[d > 0]
    out["depth_mm"] = {
        "min": int(nz.min()) if nz.size else 0,
        "median": int(np.median(nz)) if nz.size else 0,
        "max": int(nz.max()) if nz.size else 0,
        "zero_fraction": float((d == 0).mean()),
        "finite_fraction": float((d > 0).mean()),
    }
out["aligned"] = (rgb is not None and d is not None
                  and rgb.shape[0] == d.shape[0] and rgb.shape[1] == d.shape[1])
out["route"] = "annotator (no OmniGraph image writer)"
out["headless"] = True
open("/tmp/aligned_depth.json", "w").write(json.dumps(out, indent=2))
print(json.dumps(out, indent=2), flush=True)
app.close()
