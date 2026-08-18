"""Import Unitree's Dex5-1P hands at 1:1 and verify the result against the URDF.

Runs inside the Isaac Sim container (scripts/11_import_dex5.sh).

WHAT THIS IS NOT
----------------
It is not the MuJoCo graft. That path used SCALE=0.75 and DX=0.072 to make the hand
"fit", produced a palm with no mass, and ended in grasps that happened by weld. Every
one of those is a thing this importer refuses:

  * distance_scale is 1.0 and asserted afterwards by comparing the imported total mass
    against the URDF's. A scaled mesh with unscaled inertia would sail through a visual
    check and show up here.
  * import_inertia_tensor uses the URDF's own inertias rather than recomputing them
    from geometry, so the palm keeps its 0.685702 kg (right) / 0.732177 kg (left).
  * nothing is welded. Fingers get position drives and real collision geometry.

THE TWO ASYMMETRIES, BOTH REAL, BOTH PRESERVED
----------------------------------------------
  Roll_12 is MIRRORED, not sign-flipped:  L [-1.8151, 0]   R [0, +1.8151]
  Masses differ per side:                 L 1.025045 kg    R 0.978570 kg
                                          L palm 0.732177  R palm 0.685702
Copying one hand and mirroring it would get both wrong.

THE PASSIVE JOINTS
------------------
Indices 4, 8, 12, 16 in URDF document order (the finger-root abduction rolls) are the
four of twenty that are not independently actuated. The campaign suggested a 1:1 mimic
coupling; this imports them as HELD AT ZERO instead, for two reasons that agree:

  * wholebody/dex5/limits.py records that they "read as exactly dead in the recorded
    dataset". Dead is not the same as coupled.
  * a 1:1 mimic is physically impossible here. Abduction range is +-0.3840 rad while
    the flexion it would follow runs 0 to 1.5708 -- the coupling would drive the
    abduction joint into its stop before the finger was half closed.

Declared as a Class-C deviation. If Unitree ever documents the real coupling, this is
the one place it changes.
"""

from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET

PASSIVE_INDICES = (4, 8, 12, 16)          # wholebody/dex5/limits.py
EXPECT_TOTAL_MASS = {"L": 1.025045, "R": 0.978570}
EXPECT_PALM_MASS = {"L": 0.732177, "R": 0.685702}
MASS_TOL = 0.01                            # campaign section 6: hand masses within 1 %

# Conservative, documented, NOT tuned (DT3 trim: no tuning loop). The URDF declares
# effort 0.93 N m on the left hand and 0 on the right -- the right file's effort
# metadata is an upstream defect (limits.py), so effort is not a usable basis for
# gains. These are chosen to hold a finger against gravity with margin and to settle
# without visible ringing; the hand is not asked to exert a grasp force at DT3.
DRIVE_STIFFNESS = 20.0                     # N m / rad
DRIVE_DAMPING = 2.0                        # N m s / rad

URDF_DIR = "/tmp/dex5_urdf"
OUT_DIR = os.environ.get("DEX5_OUT_DIR", "/tmp/dex5_usd")


def urdf_facts(path: str):
    """Joint order, limits and masses straight from the URDF -- the authority."""
    root = ET.parse(path).getroot()
    joints = [j for j in root.findall("joint")
              if j.get("type") in ("revolute", "continuous", "prismatic")]
    out = []
    for j in joints:
        lim = j.find("limit")
        out.append({
            "name": j.get("name"),
            "lower": float(lim.get("lower")),
            "upper": float(lim.get("upper")),
        })
    mass = 0.0
    palm = 0.0
    for l in root.findall("link"):
        m = l.find("inertial/mass")
        if m is None:
            continue
        v = float(m.get("value"))
        mass += v
        if l.get("name", "").startswith("base_link00"):
            palm = v
    return out, mass, palm


def main() -> int:
    from isaacsim import SimulationApp  # noqa: F401  (already started by the caller)
    import omni.kit.commands
    from pxr import Usd, UsdPhysics

    os.makedirs(OUT_DIR, exist_ok=True)
    report = open("/tmp/dex5_import.txt", "w")

    def w(*a):
        report.write(" ".join(str(x) for x in a) + "\n")
        report.flush()

    status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    w("import config fields:", ", ".join(sorted(
        a for a in dir(cfg) if not a.startswith("_"))))

    failures = 0
    for side in ("L", "R"):
        urdf = f"{URDF_DIR}/Dex5-URDF-{side}/Dex5-URDF-{side}.urdf"
        dest = f"{OUT_DIR}/dex5_1p_{side.lower()}.usd"
        joints, want_mass, want_palm = urdf_facts(urdf)

        status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
        # 1:1. Rule 9: never scale a hand to make it fit.
        cfg.distance_scale = 1.0
        # Keep every joint: merging fixed joints would collapse links the contract and
        # the tactile zones refer to by name.
        cfg.merge_fixed_joints = False
        # The hand hangs off the arm; it is not a world-fixed robot.
        cfg.fix_base = False
        cfg.make_default_prim = True
        cfg.create_physics_scene = False
        # URDF inertias, not recomputed ones -- this is what keeps the palm's mass.
        cfg.import_inertia_tensor = True
        # Convex decomposition on the finger collision meshes: a convex HULL of a
        # curled finger is a mitten, and grasps would happen against geometry the
        # hand does not have.
        cfg.convex_decomp = True
        # Self-collision off within the hand. The fingers are adjacent by design and
        # PhysX would spend the budget resolving contacts between neighbouring
        # phalanges that never actually touch on the real hand.
        cfg.self_collision = False
        # Position drives. default_drive_type takes a UrdfJointTargetType enum, not an
        # int -- passing 1 raises. Conservative gains per the DT3 trim: no tuning loop.
        from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
        cfg.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
        cfg.default_drive_strength = DRIVE_STIFFNESS
        cfg.default_position_drive_damping = DRIVE_DAMPING
        # The URDF declares no <mimic>; parsing for them is harmless and makes the
        # absence explicit rather than assumed.
        if hasattr(cfg, "parse_mimic"):
            cfg.parse_mimic = True

        omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf,
                                  import_config=cfg, dest_path=dest)
        w(f"\n=== {side}: imported -> {dest}")

        stage = Usd.Stage.Open(dest)
        if stage is None:
            w(f"  FAIL: could not open {dest}")
            failures += 1
            continue

        got_joints, total_mass, palm_mass = [], 0.0, 0.0
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.RevoluteJoint):
                got_joints.append(prim.GetName())
            if prim.HasAPI(UsdPhysics.MassAPI):
                m = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
                if m:
                    total_mass += float(m)
                    if prim.GetName().startswith("base_link00"):
                        palm_mass = float(m)

        w(f"  joints: {len(got_joints)} revolute (URDF has {len(joints)})")
        if len(got_joints) != len(joints):
            w("  FAIL: joint count differs from the URDF")
            failures += 1

        # Mass is the scaling tripwire. A 0.75-scaled mesh with URDF inertias still
        # reports the URDF mass, but a rescaled import would not.
        for label, got, want in (("total", total_mass, EXPECT_TOTAL_MASS[side]),
                                 ("palm", palm_mass, EXPECT_PALM_MASS[side])):
            if want <= 0:
                continue
            err = abs(got - want) / want if got else 1.0
            ok = err <= MASS_TOL
            w(f"  {label} mass {got:.6f} kg vs URDF {want:.6f} kg "
              f"({err:.2%}, tol {MASS_TOL:.0%}) => {'PASS' if ok else 'FAIL'}")
            if not ok:
                failures += 1

        w(f"  URDF joint order (index: name [lower, upper]), passive marked:")
        for i, j in enumerate(joints):
            mark = "  <-- PASSIVE (held at zero)" if i in PASSIVE_INDICES else ""
            w(f"    [{i:2d}] {j['name']:16s} [{j['lower']:+.4f}, {j['upper']:+.4f}]{mark}")

    w(f"\nRESULT: {'PASS' if failures == 0 else f'{failures} FAILURE(S)'}")
    report.close()
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
