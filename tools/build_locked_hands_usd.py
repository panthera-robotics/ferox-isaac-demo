#!/usr/bin/env python3
"""Author a training variant of the twin whose Dex5 finger joints are FIXED.

MM1b only. The locomotion retrain diverged with the articulated hands attached:
base_linear_velocity −963 and base_angular_velocity −2125 against every other
reward term at ~0.01, i.e. base |vz| ~22 m/s. Forty light links on stiff joints at
a 200 Hz step is a stiff-ODE blow-up, and damping them (stiffness 20→50,
damping 2→10) improved the angular term 8.5x without curing it.

WHAT THIS DOES, AND WHY IT IS NOT A CHEAT
    Every finger joint is converted from revolute to FIXED. The links, their
    geometry, their masses and their inertias all stay exactly where they are --
    nothing is deleted, nothing is lumped by hand, so the robot's mass and mass
    distribution are preserved BIT FOR BIT rather than to some tolerance. PhysX
    folds fixed-jointed links into their parent, so the finger dynamics disappear
    while the hand still weighs what a hand weighs and hangs where a hand hangs.
    That is exactly what a locomotion policy needs to learn against.

    The deployed robot is unaffected: run.py loads the articulated twin and holds
    the hands by position, as it does today. This asset exists only so training
    sees the right mass without the wrong dynamics.

Joints are found BY NAME from isaac/twin/isaaclab/g1_dex5.py (RULE-HAND-NAME);
nothing here indexes into the hand DOF block.
"""
from __future__ import annotations

import argparse
import os
import sys

from isaacsim import SimulationApp

_app = SimulationApp({"headless": True})

from pxr import Sdf, Usd, UsdPhysics  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def hand_names():
    try:
        from twin.isaaclab.g1_dex5 import hand_joint_names
        return set(hand_joint_names())
    except Exception:
        sys.path.insert(0, "/workspace/ferox_isaac")
        from twin.isaaclab.g1_dex5 import hand_joint_names
        return set(hand_joint_names())


def total_mass(stage):
    m = 0.0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.MassAPI):
            v = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
            if v:
                m += float(v)
    return m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--report", default="")
    a = ap.parse_args()

    src = Usd.Stage.Open(a.src)
    if src is None:
        print("cannot open", a.src)
        return 1
    src_mass = total_mass(src)
    src_rev = [p for p in src.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]

    targets = hand_names()
    lines = [f"source: {a.src}",
             f"  revolute joints: {len(src_rev)}",
             f"  summed link mass: {src_mass:.6f} kg",
             f"  hand joints to lock (by name): {len(targets)}"]

    if os.path.exists(a.out):
        os.remove(a.out)
    dst = Usd.Stage.CreateNew(a.out)
    root = dst.OverridePrim("/g1_dex5_1p_lockedhands")
    root.GetReferences().AddReference(os.path.abspath(a.src))
    dst.SetDefaultPrim(root)

    locked, missed = [], []
    for prim in src.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        if prim.GetName() not in targets:
            continue
        rel = str(prim.GetPath())
        # The referenced tree lands under the override root, so rebase the path.
        src_default = src.GetDefaultPrim()
        if src_default and rel.startswith(str(src_default.GetPath())):
            rel = rel[len(str(src_default.GetPath())):]
        new_path = Sdf.Path(str(root.GetPath()) + rel)

        j = UsdPhysics.RevoluteJoint(prim)
        b0 = j.GetBody0Rel().GetTargets()
        b1 = j.GetBody1Rel().GetTargets()
        p0 = j.GetLocalPos0Attr().Get()
        p1 = j.GetLocalPos1Attr().Get()
        r0 = j.GetLocalRot0Attr().Get()
        r1 = j.GetLocalRot1Attr().Get()

        # Deactivate the revolute joint in the override, then define a fixed joint
        # beside it carrying the identical frames. Deactivating alone would leave
        # the child link unattached and it would fall off the robot.
        over = dst.OverridePrim(new_path)
        over.SetActive(False)

        fixed = UsdPhysics.FixedJoint.Define(
            dst, Sdf.Path(str(new_path) + "_locked"))

        def rebase(targets_):
            out = []
            for t in targets_:
                ts = str(t)
                if src_default and ts.startswith(str(src_default.GetPath())):
                    ts = ts[len(str(src_default.GetPath())):]
                out.append(Sdf.Path(str(root.GetPath()) + ts))
            return out

        if b0:
            fixed.CreateBody0Rel().SetTargets(rebase(b0))
        if b1:
            fixed.CreateBody1Rel().SetTargets(rebase(b1))
        if p0 is not None:
            fixed.CreateLocalPos0Attr(p0)
        if p1 is not None:
            fixed.CreateLocalPos1Attr(p1)
        if r0 is not None:
            fixed.CreateLocalRot0Attr(r0)
        if r1 is not None:
            fixed.CreateLocalRot1Attr(r1)
        locked.append(prim.GetName())

    for n in sorted(targets - set(locked)):
        missed.append(n)

    dst.GetRootLayer().Save()
    lines.append(f"  locked: {len(locked)}")
    if missed:
        lines.append(f"  NOT FOUND IN SOURCE: {missed}")

    # ---- read back -------------------------------------------------------
    chk = Usd.Stage.Open(a.out)
    out_mass = total_mass(chk)
    out_rev = [p for p in chk.Traverse()
               if p.IsA(UsdPhysics.RevoluteJoint) and p.IsActive()]
    out_hand_rev = [p.GetName() for p in out_rev if p.GetName() in targets]
    out_fixed = [p for p in chk.Traverse() if p.IsA(UsdPhysics.FixedJoint)]

    fails = []

    def chk_(cond, msg):
        lines.append(("  ok   " if cond else "  FAIL ") + msg)
        if not cond:
            fails.append(msg)

    lines.append(f"result: {a.out}")
    chk_(len(out_hand_rev) == 0,
         f"no active revolute hand joints remain (found {len(out_hand_rev)})")
    chk_(len(out_rev) == len(src_rev) - len(targets),
         f"active revolute joints {len(out_rev)} == {len(src_rev)} - {len(targets)}")
    chk_(len(out_fixed) >= len(targets),
         f"fixed joints authored: {len(out_fixed)} (>= {len(targets)})")
    rel_err = abs(out_mass - src_mass) / src_mass if src_mass else 1.0
    chk_(rel_err <= 0.01,
         f"summed link mass {out_mass:.6f} kg vs {src_mass:.6f} kg "
         f"({rel_err*100:.4f} % — gate 1 %)")

    txt = "\n".join(lines) + f"\n{'PASS' if not fails else 'FAIL'}: {len(fails)} failed\n"
    print(txt)
    if a.report:
        open(a.report, "w").write(txt)
    return 0 if not fails else 1


if __name__ == "__main__":
    rc = main()
    _app.close()
    raise SystemExit(rc)
