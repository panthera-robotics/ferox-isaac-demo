"""Measure where the Dex5's fingers actually close, in the palm frame.

v1, v2 and v3 all set `grasp_standoff` by argument and then bisected it when the hand
came up empty: 0.072 was "palm above the lid" reasoning carried over from a top-down
grasp, 0.045 was the same number re-argued for a side approach, and moving between them
traded `NO_GRIP` for `DESCEND_TIMEOUT` without either being measured. The stand-off is
not a tuning parameter -- it is a property of the hand, and the hand is right there.

So: drive the fingers to the closed pose, let physics settle, then read every link of
the right hand out of PhysX and express it in the PALM frame. The fingertips are the
distal links; where they converge is where an object has to be for the fingers to close
around it rather than beside it. That distance, projected on the approach axis, IS the
stand-off.

Discovery is by measurement rather than by a hard-coded link list: the Dex5's link
naming is upstream's and has already caught this campaign out once (`base_link00` for
the right palm, `base_link00L` for the left). Bodies are selected by being NEAR the
palm and by MOVING when the hand closes, which is what "part of the hand" means
operationally.
"""
from __future__ import annotations

import numpy as np


def _R(q):
    w, x, y, z = (float(v) for v in q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def link_transforms(pipe):
    """(names, positions Nx3, quats Nx4 wxyz) for every link, in ONE call.

    NOT via SingleRigidPrim. Constructing a prim wrapper per body -- 79 of them --
    invalidates the running physics view outright ("Simulation view object is
    invalidated and cannot be used again"), and the next set_world_pose raises. The
    articulation already exposes every link transform in a single tensor read; that is
    the safe way to look at the whole hand at once.
    """
    view = pipe.view
    names = list(view.body_names)
    T = None
    tried = []
    # The wrapper and the underlying physics-tensor view expose different APIs, and
    # which one carries link transforms varies by Isaac build. Search both and SAY
    # what was searched -- "not measurable" with no reason is how a missing API gets
    # mistaken for a hand that does not move.
    holders = [("view", view)]
    inner = getattr(view, "_physics_view", None)
    if inner is not None:
        holders.append(("view._physics_view", inner))
    for hname, h in holders:
        for attr in ("get_link_transforms", "get_body_transforms", "get_link_poses",
                     "get_transforms"):
            fn = getattr(h, attr, None)
            if fn is None:
                continue
            try:
                T = np.asarray(fn())
                tried.append(f"{hname}.{attr}->OK{T.shape}")
                break
            except Exception as exc:
                tried.append(f"{hname}.{attr}->{type(exc).__name__}")
                T = None
        if T is not None:
            break
    if T is None:
        print(f"[mm5][v4] no link-transform API found; tried: {tried or 'nothing'}; "
              f"available on view._physics_view: "
              f"{[a for a in dir(inner) if 'link' in a.lower() or 'transform' in a.lower()][:12]}",
              flush=True)
        return None
    T = T.reshape(-1, len(names), T.shape[-1])[0]
    pos = T[:, :3].astype(float)
    q = T[:, 3:7].astype(float)
    if q.shape[1] == 4:
        # Physics tensors give xyzw; the rest of this codebase is wxyz.
        q = np.column_stack([q[:, 3], q[:, 0], q[:, 1], q[:, 2]])
    return names, pos, q


def snapshot(pipe):
    """{body: world_pos} for every link, safely."""
    lt = link_transforms(pipe)
    if lt is None:
        return {}
    names, pos, _ = lt
    return {n: pos[i].copy() for i, n in enumerate(names)}


def measure(pipe, open_snapshot):
    """Hand geometry in the palm frame at the CLOSED pose, or None."""
    lt = link_transforms(pipe)
    if lt is None:
        return None
    names, pos, _ = lt
    palm_p, palm_q = pipe.palm_pose()
    Rp = _R(palm_q)

    moved = {}
    for i, b in enumerate(names):
        d = pos[i] - np.asarray(palm_p, float)
        if float(np.linalg.norm(d)) > 0.30:      # not part of the hand
            continue
        prev = open_snapshot.get(b)
        travel = (float(np.linalg.norm(pos[i] - np.asarray(prev, float)))
                  if prev is not None else 0.0)
        moved[b] = (Rp.T @ d, travel)

    fingers = {b: v for b, v in moved.items() if v[1] > 0.008}
    if not fingers:
        return None
    # The fingertips are the links that travelled FURTHEST when the hand closed --
    # distal links sweep more than proximal ones on the same finger. Five fingers.
    tips = sorted(fingers.items(), key=lambda kv: -kv[1][1])[:5]
    P = np.array([v[0] for _, v in tips])
    centre = P.mean(axis=0)
    return {
        "palm_pos": [round(float(v), 5) for v in palm_p],
        "n_hand_bodies": len(moved),
        "n_moved": len(fingers),
        "tips": [{"body": b, "palm_xyz": [round(float(c), 5) for c in v[0]],
                  "travel_m": round(float(v[1]), 5)} for b, v in tips],
        "closed_centre_palm": [round(float(v), 5) for v in centre],
        "centre_dist_m": round(float(np.linalg.norm(centre)), 5),
        "tip_spread_m": round(float(np.linalg.norm(P - centre, axis=1).mean()), 5),
        # Every link that belongs to the hand, for v5's contact counting. Discovered
        # the same way the fingertips are -- by being near the palm -- rather than by
        # a hard-coded name list, which the Dex5's upstream naming has already broken
        # once (base_link00 / base_link00L).
        "hand_bodies": sorted(moved.keys()),
    }
