"""C-39 capture: per-link mass/CoM/inertia, foot colliders, and a contact-API hunt.

Written to run ONCE at sim startup and dump to disk, because these are the numbers the
offline diff needs and the offline diff has no GPU and no Isaac.

Three rules learned the hard way in this campaign and encoded here:
  * whole-robot geometry comes from ONE tensor read, never from a SingleRigidPrim per
    body -- constructing 79 of those invalidates the running physics view mid-episode;
  * every lookup says WHICH api answered, because "absent" and "returned nothing" look
    identical in a log and mean opposite things;
  * the link set is stated explicitly, because a whole-body mass sum that silently
    includes something extra is exactly the bug this capture exists to settle.
"""
from __future__ import annotations

import json
import os

import numpy as np


def _tensor(view, names, attrs):
    """First of `attrs` that answers, from the wrapper or the physics view."""
    tried = []
    for hn, h in (("view", view), ("physx", getattr(view, "_physics_view", None))):
        if h is None:
            continue
        for a in attrs:
            fn = getattr(h, a, None)
            if fn is None:
                tried.append(f"{hn}.{a}: absent")
                continue
            try:
                return np.asarray(fn()), f"{hn}.{a}", tried
            except Exception as exc:
                tried.append(f"{hn}.{a}: {type(exc).__name__}")
    return None, None, tried


def capture(art, stage, out_dir: str, variant: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    view = art._articulation_view
    names = list(view.body_names)
    n = len(names)
    report = {"variant": variant, "n_links": n, "sources": {}}

    M, src, tried = _tensor(view, names, ("get_masses",))
    report["sources"]["masses"] = src or tried
    T, tsrc, ttried = _tensor(view, names, ("get_link_transforms", "get_transforms"))
    report["sources"]["transforms"] = tsrc or ttried
    C, csrc, ctried = _tensor(view, names, ("get_coms",))
    report["sources"]["coms"] = csrc or ctried
    I, isrc, itried = _tensor(view, names, ("get_inertias",))
    report["sources"]["inertias"] = isrc or itried

    rows = []
    if M is not None:
        M = M.reshape(-1)[:n]
    if T is not None:
        T = T.reshape(-1, n, T.shape[-1])[0]
    if C is not None:
        C = C.reshape(-1, n, C.shape[-1])[0]
    if I is not None:
        I = I.reshape(-1, n, I.shape[-1])[0]

    for i, nm in enumerate(names):
        r = {"link": nm, "mass_kg": None, "com_local": None, "world_xyz": None,
             "inertia_diag": None}
        if M is not None:
            r["mass_kg"] = round(float(M[i]), 6)
        if C is not None:
            r["com_local"] = [round(float(v), 6) for v in C[i][:3]]
        if T is not None:
            r["world_xyz"] = [round(float(v), 6) for v in T[i][:3]]
            q = T[i][3:7]
            r["quat_wxyz"] = [round(float(q[3]), 6), round(float(q[0]), 6),
                              round(float(q[1]), 6), round(float(q[2]), 6)]
        if I is not None:
            v = I[i]
            # 9-vector (row-major 3x3) or 6-vector; take the diagonal either way.
            d = [v[0], v[4], v[8]] if len(v) >= 9 else list(v[:3])
            r["inertia_diag"] = [round(float(x), 8) for x in d]
        rows.append(r)

    csv = os.path.join(out_dir, f"links_{variant}.csv")
    with open(csv, "w") as fh:
        fh.write("link,mass_kg,com_local_x,com_local_y,com_local_z,"
                 "world_x,world_y,world_z,qw,qx,qy,qz,ixx,iyy,izz\n")
        for r in rows:
            c = r["com_local"] or [None] * 3
            w = r["world_xyz"] or [None] * 3
            d = r["inertia_diag"] or [None] * 3
            q = r.get("quat_wxyz") or [None] * 4
            fh.write(f"{r['link']},{r['mass_kg']},{c[0]},{c[1]},{c[2]},"
                     f"{w[0]},{w[1]},{w[2]},{q[0]},{q[1]},{q[2]},{q[3]},"
                     f"{d[0]},{d[1]},{d[2]}\n")

    if M is not None:
        tot = float(M.sum())
        report["total_mass_kg"] = round(tot, 6)
        # THE +4.000 kg QUESTION. The earlier whole-body sum said 39.005 where DT3/MM1b
        # assert 35.004757 with hands. State exactly what is being summed, and list the
        # heaviest links, so an extra body or a duplicated link is visible rather than
        # inferred. Every link in `names` belongs to THIS articulation -- the lab
        # objects and the rig are not in it -- so if the sum is wrong the excess is a
        # link, not a stray prim.
        order = np.argsort(-M)
        report["heaviest_links"] = [
            {"link": names[int(i)], "mass_kg": round(float(M[int(i)]), 6)}
            for i in order[:12]]
        dup = {}
        for nm in names:
            dup[nm] = dup.get(nm, 0) + 1
        report["duplicate_link_names"] = {k: v for k, v in dup.items() if v > 1}
        if T is not None:
            # TRUE CoM: link origin + R_link * com_local. The first pass weighted link
            # ORIGINS, which is not the centre of mass and gave +0.054 m of margin where
            # the corrected sum gives something else entirely. The static-margin
            # argument rests on this number, so it has to be the real one.
            P = np.zeros((n, 3))
            for i in range(n):
                q = T[i][3:7]
                R = _quat_R_xyzw(q)
                cl = C[i][:3] if C is not None else np.zeros(3)
                P[i] = T[i][:3] + R @ cl
            com = (P * M[:, None]).sum(axis=0) / max(tot, 1e-9)
            report["com_method"] = ("link origin + R*com_local" if C is not None
                                    else "link origins only (get_coms unavailable)")
            report["com_world"] = [round(float(v), 6) for v in com]
            feet = [i for i, nm in enumerate(names) if "ankle_roll" in nm]
            if feet:
                fx = float(np.mean([T[i][0] for i in feet]))
                report["ankle_mean_x"] = round(fx, 6)
                report["com_x_minus_ankle_x"] = round(float(com[0]) - fx, 6)

    # ---- foot colliders (2b) -------------------------------------------------
    feet_out = []
    try:
        from pxr import Usd, UsdPhysics, PhysxSchema
        # Traverse the WHOLE stage WITH INSTANCE PROXIES. The URDF import puts link
        # geometry under /Flattened_Prototype_NNN/... and instances it, so a PrimRange
        # rooted at /World/G1 finds no colliders at all -- which is what the first pass
        # reported, and it looked exactly like "the feet have no colliders".
        rng = Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
        for p in rng:
            path = p.GetPath().pathString
            if "ankle_roll" not in path:
                continue
            e = {"path": path, "type": str(p.GetTypeName()),
                 "collisionAPI": bool(p.HasAPI(UsdPhysics.CollisionAPI))}
            if p.HasAPI(UsdPhysics.MeshCollisionAPI):
                a = UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get()
                e["approximation"] = str(a)
            if p.HasAPI(PhysxSchema.PhysxCollisionAPI):
                px = PhysxSchema.PhysxCollisionAPI(p)
                e["contactOffset"] = _f(px.GetContactOffsetAttr().Get())
                e["restOffset"] = _f(px.GetRestOffsetAttr().Get())
            ext = p.GetAttribute("extent").Get() if p.HasAttribute("extent") else None
            if ext:
                e["extent"] = [[round(float(v), 6) for v in pt] for pt in ext]
            # Sphere colliders carry a radius and an xform, not points -- and the
            # spheres ARE the foot's contact geometry, so their centres in the ankle
            # frame are the fore/aft reach the diff needs.
            if p.GetTypeName() == "Sphere":
                r = p.GetAttribute("radius").Get() if p.HasAttribute("radius") else None
                e["radius"] = _f(r)
                try:
                    from pxr import UsdGeom, Usd as _U
                    xf = UsdGeom.Xformable(p)
                    m = xf.ComputeLocalToWorldTransform(_U.TimeCode.Default())
                    t = m.ExtractTranslation()
                    e["world_xyz"] = [round(float(t[0]), 6), round(float(t[1]), 6),
                                      round(float(t[2]), 6)]
                except Exception as exc:
                    e["xform_error"] = repr(exc)
            pts = p.GetAttribute("points").Get() if p.HasAttribute("points") else None
            if pts:
                a = np.array([[float(v[0]), float(v[1]), float(v[2])] for v in pts])
                e["n_points"] = int(len(a))
                # THE NUMBER THE DIFF NEEDS: how far the contact geometry reaches fore
                # and aft of the ankle. MuJoCo's foot spheres sit at -0.05 (heel) and
                # +0.12 (toe); if the twin's foot has little toe, the ankle has to hold
                # a gravity moment the reference lets the GROUND hold.
                e["local_x_min"] = round(float(a[:, 0].min()), 6)
                e["local_x_max"] = round(float(a[:, 0].max()), 6)
                e["local_y_min"] = round(float(a[:, 1].min()), 6)
                e["local_y_max"] = round(float(a[:, 1].max()), 6)
                e["local_z_min"] = round(float(a[:, 2].min()), 6)
            if e["collisionAPI"] or "approximation" in e or "extent" in e or pts:
                feet_out.append(e)
    except Exception as exc:
        feet_out.append({"error": repr(exc)})
    report["foot_colliders"] = feet_out

    # HAND COLLIDERS. Grasp v6 could not attach a ContactSensor to any finger tip
    # because no prim under those links carries CollisionAPI. Whether the fingers have
    # collision geometry AT ALL is a bigger question than the sensor -- a hand with no
    # colliders closes THROUGH the object, which would explain every NO_GRIP row in
    # v1..v6 -- so it is answered here directly.
    hand_out = {"collision_prims": [], "scanned": 0}
    try:
        from pxr import Usd, UsdPhysics
        for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
            path = p.GetPath().pathString
            if "Link_" not in path and "base_link00" not in path:
                continue
            hand_out["scanned"] += 1
            if p.HasAPI(UsdPhysics.CollisionAPI):
                hand_out["collision_prims"].append(
                    {"path": path, "type": str(p.GetTypeName())})
    except Exception as exc:
        hand_out["error"] = repr(exc)
    hand_out["n_collision_prims"] = len(hand_out["collision_prims"])
    hand_out["collision_prims"] = hand_out["collision_prims"][:20]
    report["hand_colliders"] = hand_out

    # Export the foot subtree as usda text, so the offline session can read what the
    # binary usdc would not give up.
    try:
        from pxr import Usd
        sub = Usd.Stage.CreateInMemory()
        for side in ("left", "right"):
            src = stage.GetPrimAtPath(f"/World/G1/{side}_ankle_roll_link")
            if src and src.IsValid():
                sub.DefinePrim(f"/{side}_ankle_roll_link").GetReferences()
        txt = stage.ExportToString(addSourceFileComment=False)
        # Only the ankle subtree matters and the whole stage is enormous.
        keep = [ln for ln in txt.splitlines() if "ankle" in ln.lower()]
        with open(os.path.join(out_dir, f"foot_subtree_{variant}.usda"), "w") as fh:
            fh.write("\n".join(keep))
        report["foot_usda_lines"] = len(keep)
    except Exception as exc:
        report["foot_usda_error"] = repr(exc)

    # ---- contact API hunt (2c) ----------------------------------------------
    hunt = {}
    for hn, h in (("view", view), ("physx", getattr(view, "_physics_view", None))):
        if h is None:
            hunt[hn] = "None"
            continue
        hunt[hn] = sorted(a for a in dir(h)
                          if any(k in a.lower() for k in
                                 ("contact", "force", "sensor", "prepare")))
    for mod, attr in (("isaacsim.core.prims", "RigidContactView"),
                      ("isaacsim.sensors.physics", "ContactSensor"),
                      ("omni.physx", "get_physx_simulation_interface")):
        try:
            m = __import__(mod, fromlist=[attr])
            hunt[f"{mod}.{attr}"] = "present" if hasattr(m, attr) else "module ok, attr absent"
        except Exception as exc:
            hunt[f"{mod}.{attr}"] = f"import failed: {type(exc).__name__}"
    report["contact_api_hunt"] = hunt

    with open(os.path.join(out_dir, f"capture_{variant}.json"), "w") as fh:
        json.dump(report, fh, indent=2)
    print(f"[c39] capture written: {csv} and capture_{variant}.json "
          f"(total_mass={report.get('total_mass_kg')} kg, "
          f"links={n}, dup={report.get('duplicate_link_names')})", flush=True)


def _quat_R_xyzw(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def _f(v):
    return None if v is None else round(float(v), 6)
