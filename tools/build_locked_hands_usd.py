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

from pxr import Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def hand_names():
    try:
        from twin.isaaclab.g1_dex5 import hand_joint_names
        return set(hand_joint_names())
    except Exception:
        sys.path.insert(0, "/workspace/ferox_isaac")
        from twin.isaaclab.g1_dex5 import hand_joint_names
        return set(hand_joint_names())


def _palms(src, targets):
    """Hand root links: drive a hand joint, are never driven by one."""
    driven, roots = set(), set()
    for p in src.Traverse():
        if p.IsA(UsdPhysics.RevoluteJoint) and p.GetName() in targets:
            for t in UsdPhysics.RevoluteJoint(p).GetBody1Rel().GetTargets():
                driven.add(str(t))
    for p in src.Traverse():
        if p.IsA(UsdPhysics.RevoluteJoint) and p.GetName() in targets:
            for t in UsdPhysics.RevoluteJoint(p).GetBody0Rel().GetTargets():
                if str(t) not in driven:
                    roots.add(str(t))
    return roots


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
    ap.add_argument("--palm-collider", action="store_true",
                    help="keep ONE collider per hand -- the palm's own geometry -- "
                         "and drop the 19 finger colliders. Fingers cannot "
                         "self-collide or hit the forearm, the hand still has a "
                         "body the world can touch, and no shape is invented.")
    ap.add_argument("--no-hand-collision", action="store_true",
                    help="also disable collision on the hand links. Fixing a joint "
                         "removes its DOF, not its collider: 40 finger colliders "
                         "still self-collide and still hit the forearm at spawn.")
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
    hand_link_paths = set()
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
        for t in (b1 or []):
            hand_link_paths.add(str(t))

    for n in sorted(targets - set(locked)):
        missed.append(n)

    # Colliders. A fixed joint removes the DOF and leaves the geometry: 40 finger
    # colliders per robot still collide with each other and with the forearm.
    # Reference run (a) on the upstream asset is healthy at episode length 181;
    # the same config on this asset with joints already fixed collapses to 1.00
    # with 236 of 1024 envs terminating on bad_orientation. Colliders are the
    # remaining difference.
    disabled = 0
    keep = set()
    if a.no_hand_collision:
        # Each link is an Xform with `visuals` and `collisions` child Xforms; the
        # meshes themselves live in a referenced payload. Deactivating the
        # `collisions` prim prunes the whole subtree from composition, which
        # removes every collider under it whatever type they are -- and does not
        # depend on HasAPI(CollisionAPI), which reports False for every prim in
        # this asset and already sent one probe down a blind alley.
        # The hand's ROOT link (the palm) is the one that drives a hand joint but
        # is never driven by one -- found from the joint graph, not by name
        # guessing. Keeping its collider gives the hand a body the world can touch
        # while the 19 finger colliders, which are what actually explode, go away.
        keep = set()
        if a.palm_collider:
            driven = set()
            for _p in src.Traverse():
                if _p.IsA(UsdPhysics.RevoluteJoint) and _p.GetName() in targets:
                    for _t in UsdPhysics.RevoluteJoint(_p).GetBody1Rel().GetTargets():
                        driven.add(str(_t))
            for _p in src.Traverse():
                if _p.IsA(UsdPhysics.RevoluteJoint) and _p.GetName() in targets:
                    for _t in UsdPhysics.RevoluteJoint(_p).GetBody0Rel().GetTargets():
                        if str(_t) not in driven:
                            keep.add(str(_t))
            lines.append(f"  palm links kept as colliders: {sorted(keep)}")

        # The palms are included whether or not --palm-collider asked to keep them,
        # unless it did. They are body0-only in the joint graph, so they were never
        # in hand_link_paths and their colliders survived every earlier variant --
        # which is why two supposedly different assets plateaued at the identical
        # 9.05. The spawn-contact check found 8794 N and 7909 N between each palm
        # and its OWN wrist_pitch_link: the collider interpenetrates the link it
        # hangs from, so the solver starts every episode resolving a 9 kN
        # penetration. The upstream reference reports zero contacts at spawn.
        _all_hand = set(hand_link_paths)
        if not a.palm_collider:
            _all_hand |= _palms(src, targets)
        for lp in sorted(_all_hand):
            if lp in keep:
                continue
            rel = lp
            if src_default and rel.startswith(str(src_default.GetPath())):
                rel = rel[len(str(src_default.GetPath())):]
            col_src = src.GetPrimAtPath(lp + "/collisions")
            if not col_src:
                continue
            ov = dst.OverridePrim(Sdf.Path(str(root.GetPath()) + rel + "/collisions"))
            ov.SetActive(False)
            disabled += 1
        lines.append(f"  hand links: {len(hand_link_paths)}, "
                     f"collision subtrees deactivated: {disabled}")

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
    if a.no_hand_collision:
        still = [lp for lp in sorted(hand_link_paths - keep)
                 if (lambda pr: bool(pr) and pr.IsActive())(
                     chk.GetPrimAtPath(
                         str(root.GetPath())
                         + (lp[len(str(src_default.GetPath())):] if src_default
                            and lp.startswith(str(src_default.GetPath())) else lp)
                         + "/collisions"))]
        chk_(not still,
             f"no hand collision subtree is still active "
             f"({len(still)} of {len(hand_link_paths)} remain)")

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
