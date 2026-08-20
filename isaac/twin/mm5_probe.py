"""MM5 feasibility probe: object pose, arm Jacobian, end-effector link.

Three facts have to be true before a manipulation pipeline is worth writing:
the object's world pose is readable, the articulation exposes a Jacobian for the
arm chain, and there is a nameable end-effector link to solve to. Checked once,
loudly, rather than discovered halfway through an IK loop.
"""
from __future__ import annotations


def probe(art, stage) -> None:
    import numpy as np
    from pxr import UsdGeom, Usd

    print("\n=== MM5 PROBE ===", flush=True)

    # 1. objects
    # Walk the stage rather than guessing a path: the lab is added as a reference and
    # the prim path it lands on is a property of how run.py stages it, not of the
    # builder that wrote it.
    wanted = {"soup_can", "mustard", "cracker_box", "sugar_box", "mug", "banana",
              "cube_5cm", "brochure_box"}
    found = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.GetName() in wanted:
            xf = UsdGeom.Xformable(prim)
            m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            t = m.ExtractTranslation()
            found.append((prim.GetPath().pathString,
                          (round(t[0], 4), round(t[1], 4), round(t[2], 4))))
    print(f"[probe] objects found: {len(found)}")
    for path, t in found:
        print(f"    {path}  world xyz={t}")

    # 2. end-effector links
    names = list(art.dof_names)
    print(f"[probe] articulation has {len(names)} DOFs")
    try:
        bodies = list(art._articulation_view.body_names)
        cand = [b for b in bodies if any(k in b.lower() for k in
                ("palm", "wrist_yaw", "hand_base", "base_link0"))]
        print(f"[probe] {len(bodies)} bodies; end-effector candidates: {cand[:10]}")
        print(f"[probe] non-G1 bodies (hand side): "
              f"{[b for b in bodies if not b.endswith('_link')][:12]}")
    except Exception as exc:
        print(f"[probe] body_names unavailable: {exc!r}")
        bodies = []

    # 3. Jacobian
    try:
        J = art._articulation_view.get_jacobians()
        print(f"[probe] jacobian shape {np.asarray(J).shape}")
    except Exception as exc:
        print(f"[probe] get_jacobians FAILED: {exc!r}")
    try:
        print(f"[probe] jacobian_shape attr: {art._articulation_view.jacobian_shape}")
    except Exception:
        pass
    # colliders and physics scenes -- the objects were observed in free fall from t=0,
    # which is what a table with no collider looks like.
    from pxr import UsdPhysics
    for path in ("/World/Env/table", "/World/Env/objects/soup_can", "/World/Env/floor",
                 "/World/Env/room/floor"):
        pr = stage.GetPrimAtPath(path)
        if pr and pr.IsValid():
            print(f"[probe] {path}: collision={pr.HasAPI(UsdPhysics.CollisionAPI)} "
                  f"rigidbody={pr.HasAPI(UsdPhysics.RigidBodyAPI)} type={pr.GetTypeName()}")
        else:
            print(f"[probe] {path}: MISSING")
    for root in ("/World/Env/furniture", "/World/Env/objects/soup_can", "/World/Env/shell"):
        pr = stage.GetPrimAtPath(root)
        if pr and pr.IsValid():
            kids = []
            for c in Usd.PrimRange(pr):
                if c.HasAPI(UsdPhysics.CollisionAPI):
                    kids.append(c.GetPath().pathString)
            print(f"[probe] {root}: {len(kids)} prims with CollisionAPI; "
                  f"first={kids[:3]}")
    scenes = [p.GetPath().pathString for p in Usd.PrimRange(stage.GetPseudoRoot())
              if p.IsA(UsdPhysics.Scene)]
    print(f"[probe] PhysicsScenes: {scenes}")
    env = stage.GetPrimAtPath("/World/Env")
    if env and env.IsValid():
        print(f"[probe] /World/Env children: {[c.GetName() for c in env.GetChildren()][:14]}")
    # Palm pose source and a finite-difference check of the Jacobian column mapping.
    # If either is wrong the servo drives the arm somewhere confidently and stalls,
    # which is exactly the symptom.
    try:
        bp, bq = art._articulation_view.get_body_poses()
        print(f"[probe] get_body_poses OK: shape {np.asarray(bp).shape}")
        bi = list(art._articulation_view.body_names).index("base_link00")
        print(f"[probe] base_link00 idx={bi} pos={np.round(np.asarray(bp[0][bi]),4)}")
    except Exception as exc:
        print(f"[probe] get_body_poses UNAVAILABLE -> USD fallback in use: {exc!r}")
        for cand in ("/World/G1/base_link00", "/World/G1/right_wrist_yaw_link"):
            pr = stage.GetPrimAtPath(cand)
            print(f"[probe]   {cand}: valid={bool(pr and pr.IsValid())}")
    print("=== END MM5 PROBE ===\n", flush=True)
