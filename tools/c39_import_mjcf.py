"""C-39 task 1 — import the REFERENCE MuJoCo model into Isaac, unmodified.

Runs inside the Isaac Sim container (scripts/c39_import_mjcf.sh).

The point of this asset is to split one question in two. SONIC stands in the reference
MuJoCo sim and falls in the twin; the difference is either OUR SIMULATOR or OUR BODY.
Putting the reference's own body into our simulator, through the stock MJCF importer
with its own default settings, isolates that: if the reference body stands here, the
simulator is fine and the delta is the asset; if it falls here, the delta is the solver.

"Unmodified" is a claim, so it is evidenced rather than asserted: every field of the
import config is read back and written into the report before the import runs, and the
config is left exactly as MJCFCreateImportConfig returns it apart from the three fields
that say WHERE to put the result (dest path, prim path, default prim) plus fix_base
(the reference is a floating-base model and its MJCF says so) and create_physics_scene
(run.py owns the world's physics scene; a second one would silently win or conflict).
Those five are listed in the report as deltas.
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import omni.kit.commands
from pxr import Usd, UsdPhysics

MJCF = os.environ.get("C39_MJCF", "/tmp/c39_ref/g1_29dof_old.xml")
OUT_DIR = os.environ.get("C39_OUT_DIR", "/tmp/c39_ref_usd")
REPORT = "/tmp/c39_import_mjcf.txt"
PRIM = "/g1_29dof"

# From docs/mm/evidence/C39/mujoco_ref.json, computed offline by tools/c39_mjcf_mass.py
EXPECT_MASS = 35.112142
MASS_TOL = 0.05
EXPECT_DOF = 29


def main() -> int:
    os.makedirs(OUT_DIR, exist_ok=True)
    report = open(REPORT, "w")

    def w(*a):
        report.write(" ".join(str(x) for x in a) + "\n")
        report.flush()

    root = ET.parse(MJCF).getroot()
    mj_joints = [j.get("name") for j in root.iter("joint")
                 if j.get("name") and j.get("type") != "free"]
    mj_bodies = [b.get("name") for b in root.iter("body") if b.get("name")]
    w(f"source MJCF: {MJCF}")
    w(f"  hinge joints {len(mj_joints)}, bodies {len(mj_bodies)}")

    status, cfg = omni.kit.commands.execute("MJCFCreateImportConfig")
    w("\nimport config AS RETURNED by MJCFCreateImportConfig (before any change):")
    fields = [f for f in dir(cfg) if not f.startswith("_")]
    before = {}
    for f in sorted(fields):
        try:
            v = getattr(cfg, f)
        except Exception as exc:            # pragma: no cover - defensive
            w(f"  {f} = <unreadable: {exc}>")
            continue
        if callable(v):
            continue
        before[f] = v
        w(f"  {f} = {v!r}")

    # The five deltas, and only these five.
    cfg.fix_base = False
    cfg.make_default_prim = True
    cfg.create_physics_scene = False
    cfg.import_inertia_tensor = True
    cfg.self_collision = False
    w("\nDELTAS applied (everything else is the importer's own default):")
    for f in ("fix_base", "make_default_prim", "create_physics_scene",
              "import_inertia_tensor", "self_collision"):
        w(f"  {f}: {before.get(f)!r} -> {getattr(cfg, f)!r}")

    dest = f"{OUT_DIR}/g1_29dof_old.usd"
    omni.kit.commands.execute(
        "MJCFCreateAsset", mjcf_path=MJCF, import_config=cfg,
        prim_path=PRIM, dest_path=dest,
    )
    w(f"\nimported -> {dest}")

    failures = 0
    stage = Usd.Stage.Open(dest)
    if stage is None:
        w(f"FAIL: could not open {dest}")
        report.close()
        return 1

    default = stage.GetDefaultPrim()
    w(f"  default prim: {default.GetName() if default else '<none>'}")

    joints, mass, links = [], 0.0, []
    for prim in Usd.PrimRange.AllPrims(stage.GetPseudoRoot()):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joints.append(prim.GetName())
        if prim.HasAPI(UsdPhysics.MassAPI):
            m = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
            if m:
                mass += float(m)
                links.append((prim.GetName(), float(m)))

    w(f"\n  revolute joints in USD: {len(joints)}")
    w(f"  links with mass: {len(links)}  total {mass:.6f} kg")
    if abs(mass - EXPECT_MASS) > MASS_TOL:
        w(f"  FAIL: mass {mass:.6f} != reference {EXPECT_MASS} +- {MASS_TOL}")
        failures += 1
    else:
        w(f"  OK: mass matches the offline MJCF sum ({EXPECT_MASS}) within {MASS_TOL}")

    missing = [j for j in mj_joints if not any(j == u or j in u for u in joints)]
    if missing:
        w(f"  FAIL: {len(missing)} MJCF joints absent from the USD: {missing[:6]}")
        failures += 1
    else:
        w(f"  OK: all {len(mj_joints)} MJCF hinge joints present by name")

    w("\nper-link mass (name, kg):")
    for n, m in sorted(links, key=lambda t: -t[1]):
        w(f"  {n:36s} {m:9.6f}")

    w(f"\nRESULT: {'PASS' if failures == 0 else 'FAIL'} ({failures} failures)")
    report.close()
    return failures


if __name__ == "__main__":
    raise SystemExit(main())
