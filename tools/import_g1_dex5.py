"""Import the merged G1 + Dex5-1P URDF and verify it is ONE articulation.

Runs inside the Isaac Sim container (scripts/12_import_g1_dex5.sh). The merged URDF
comes from tools/merge_dex5_urdf.py, which explains why a merge is needed at all.

The verification that matters here is the articulation DOF count. Everything else --
joint count, mass, mount offset, limits -- was already green on the USD-composition
attempt that could not move a finger, because those properties live in the stage and
the articulation lives in PhysX. 69 DOF is the only number that proves the hands are
actually part of the robot.
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import omni.kit.commands
from pxr import Usd, UsdPhysics

URDF = os.environ.get("G1DEX5_URDF", "/tmp/g1_dex5_urdf/g1_29dof_dex5_1p.urdf")
OUT_DIR = os.environ.get("G1DEX5_OUT_DIR", "/tmp/g1_dex5_usd")
REPORT = "/tmp/g1_dex5_import.txt"

EXPECT_DOF = 69                    # 29 body + 2 x 20 hand
EXPECT_TOTAL_MASS = 35.004757      # 33.001142 body + 1.025045 L + 0.978570 R
#   body is 33.341142 minus the two 0.170 kg *_rubber_hand caps the Dex5 replaces
MASS_TOL = 0.01
DRIVE_STIFFNESS = 20.0
DRIVE_DAMPING = 2.0
PASSIVE_SUFFIX_INDICES = (4, 8, 12, 16)


def main() -> int:
    os.makedirs(OUT_DIR, exist_ok=True)
    report = open(REPORT, "w")

    def w(*a):
        report.write(" ".join(str(x) for x in a) + "\n")
        report.flush()

    root = ET.parse(URDF).getroot()
    urdf_rev = [j.get("name") for j in root.findall("joint")
                if j.get("type") in ("revolute", "continuous")]
    urdf_mass = sum(float(m.get("value")) for m in root.iter("mass"))
    w(f"source URDF: {URDF}")
    w(f"  revolute joints {len(urdf_rev)}, mass {urdf_mass:.6f} kg")

    dest = f"{OUT_DIR}/g1_dex5_1p.usd"
    status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    # Identical to tools/import_dex5.py -- see that file for why each is set.
    cfg.distance_scale = 1.0
    cfg.merge_fixed_joints = False
    cfg.fix_base = False
    cfg.make_default_prim = True
    cfg.create_physics_scene = False
    cfg.import_inertia_tensor = True
    cfg.convex_decomp = True
    cfg.self_collision = False
    from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
    cfg.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
    cfg.default_drive_strength = DRIVE_STIFFNESS
    cfg.default_position_drive_damping = DRIVE_DAMPING
    if hasattr(cfg, "parse_mimic"):
        cfg.parse_mimic = True

    omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=URDF,
                              import_config=cfg, dest_path=dest)
    w(f"\nimported -> {dest}")

    failures = 0
    stage = Usd.Stage.Open(dest)
    if stage is None:
        w(f"FAIL: could not open {dest}")
        report.close()
        return 1

    default = stage.GetDefaultPrim()
    w(f"  default prim: {default.GetName()}")
    if default.GetName() != "g1_29dof_rev_1_0":
        w("  FAIL: default prim renamed -- the twin sensor layer overrides "
          "/g1_29dof_rev_1_0/... and would silently stop composing")
        failures += 1

    joints, mass, roots = [], 0.0, []
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joints.append(prim.GetName())
        if prim.HasAPI(UsdPhysics.MassAPI):
            m = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
            if m:
                mass += float(m)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            roots.append(str(prim.GetPath()))

    w(f"  revolute joints: {len(joints)} (URDF {len(urdf_rev)})")
    if len(joints) != len(urdf_rev):
        w("  FAIL: joint count differs from the URDF")
        failures += 1

    err = abs(mass - EXPECT_TOTAL_MASS) / EXPECT_TOTAL_MASS
    w(f"  total mass: {mass:.6f} kg vs {EXPECT_TOTAL_MASS:.6f} "
      f"({err:.2%}, tol {MASS_TOL:.0%}) => {'PASS' if err <= MASS_TOL else 'FAIL'}")
    if err > MASS_TOL:
        failures += 1

    w(f"  articulation roots: {len(roots)} {roots}")
    if len(roots) != 1:
        w("  FAIL: expected exactly one articulation root")
        failures += 1

    missing = [n for n in urdf_rev if n not in joints]
    w(f"  every URDF joint present in the USD: "
      f"{'PASS' if not missing else f'FAIL {missing[:5]}'}")
    if missing:
        failures += 1

    w(f"\nRESULT: {'PASS' if failures == 0 else f'{failures} FAILURE(S)'}")
    report.close()
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
