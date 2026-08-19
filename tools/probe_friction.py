from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdShade, UsdPhysics, PhysxSchema
import json

def mats(stage, label, filt=None):
    out=[]
    for prim in stage.Traverse():
        if prim.IsA(UsdShade.Material) or prim.HasAPI(UsdPhysics.MaterialAPI):
            m=UsdPhysics.MaterialAPI(prim)
            try:
                sf=m.GetStaticFrictionAttr().Get(); df=m.GetDynamicFrictionAttr().Get(); r=m.GetRestitutionAttr().Get()
            except Exception:
                continue
            if sf is None and df is None: continue
            p=str(prim.GetPath())
            if filt and filt not in p: continue
            cm=None
            if prim.HasAPI(PhysxSchema.PhysxMaterialAPI):
                try: cm=PhysxSchema.PhysxMaterialAPI(prim).GetFrictionCombineModeAttr().Get()
                except Exception: pass
            out.append({"path":p,"static":sf,"dynamic":df,"restitution":r,"combine":str(cm)})
    print(f"--- {label} ---")
    for o in out: print("   ", json.dumps(o))
    if not out: print("    (no physics materials found)")
    return out

res={}
st=Usd.Stage.Open("/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd")
res["robot"]=mats(st,"G1+Dex5 asset (feet/body materials)")
try:
    import omni.isaac.core.utils.nucleus as nu
except Exception: pass
from isaacsim.core.api import World
w=World(stage_units_in_meters=1.0); w.scene.add_default_ground_plane(); w.reset()
res["default_ground"]=mats(w.stage,"default ground plane","Ground") or mats(w.stage,"default ground plane (all)")
json.dump(res,open("/tmp/friction_probe.json","w"),indent=2)
app.close()
