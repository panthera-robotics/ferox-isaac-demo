"""Penetration depth between two intersecting triangle meshes (CPU, numpy + coal/hppfcl).

coal's distance query reports 0 for intersecting non-convex meshes and no signed depth, so the audit measures the depth itself:
the vertices of mesh A that lie inside mesh B (ray-parity test along +z, triangles pre-filtered by their xy bounding box) and
their distance to B's surface (coal distance from a point-sized sphere to the BVH), symmetrically for B in A. The reported depth is
the larger of the two maxima; the counts say how much of each mesh is inside the other. Ray parity assumes closed meshes; a
vertex whose ray grazes an edge is resolved by a second ray along +y (parity must agree, else the vertex is ignored).
"""
from __future__ import annotations

import numpy as np

try:
    import coal as fcl
except ImportError:  # pragma: no cover
    import hppfcl as fcl


def mesh_arrays(bvh):
    V = np.asarray(bvh.vertices(), dtype=float)
    T = np.array([[bvh.tri_indices(i)[k] for k in range(3)] for i in range(bvh.num_tris)], dtype=np.int64)
    return V, T


def _parity(P, V, T, axis):
    """Number of triangle crossings of a ray from each point along +axis (mod 2), triangles filtered by the two other coordinates."""
    o = [k for k in range(3) if k != axis]; A, B, C = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    lo = np.minimum(np.minimum(A, B), C); hi = np.maximum(np.maximum(A, B), C); inside = np.zeros(len(P), dtype=bool)
    for s in range(0, len(P), 64):
        p = P[s:s + 64]
        cand = (p[:, None, o[0]] >= lo[None, :, o[0]]) & (p[:, None, o[0]] <= hi[None, :, o[0]]) & (p[:, None, o[1]] >= lo[None, :, o[1]]) & (p[:, None, o[1]] <= hi[None, :, o[1]])
        for r in range(len(p)):
            idx = np.nonzero(cand[r])[0]
            if not len(idx): continue
            a, b, c = A[idx], B[idx], C[idx]; q = p[r]
            # 2D point-in-triangle in the projection, then the crossing height along the axis
            u, v, w = o[0], o[1], axis
            d = (b[:, v] - c[:, v]) * (a[:, u] - c[:, u]) + (c[:, u] - b[:, u]) * (a[:, v] - c[:, v]); ok = np.abs(d) > 1e-18
            l1 = np.where(ok, ((b[:, v] - c[:, v]) * (q[u] - c[:, u]) + (c[:, u] - b[:, u]) * (q[v] - c[:, v])) / np.where(ok, d, 1), -1)
            l2 = np.where(ok, ((c[:, v] - a[:, v]) * (q[u] - c[:, u]) + (a[:, u] - c[:, u]) * (q[v] - c[:, v])) / np.where(ok, d, 1), -1); l3 = 1 - l1 - l2
            hit = ok & (l1 >= 0) & (l2 >= 0) & (l3 >= 0); z = l1 * a[:, w] + l2 * b[:, w] + l3 * c[:, w]
            inside[s + r] = (np.count_nonzero(hit & (z > q[w])) % 2) == 1
    return inside


def inside_mask(P, V, T):
    a = _parity(P, V, T, 2); b = _parity(P, V, T, 1); agree = a == b
    return a & agree, agree


def _fcl_tf(R, t):
    return fcl.Transform3s(R, t) if hasattr(fcl, 'Transform3s') else fcl.Transform3f(R, t)


def surface_distance(P, bvh, tf):
    """Unsigned distance from each point to the mesh surface (coal distance, point-sized sphere)."""
    req = fcl.DistanceRequest(); res = fcl.DistanceResult(); sph = fcl.Sphere(1e-9); out = np.empty(len(P))
    for i, p in enumerate(P):
        res.clear(); fcl.distance(sph, _fcl_tf(np.eye(3), p), bvh, tf, req, res); out[i] = max(res.min_distance, 0.0)
    return out


def penetration(bvh_a, tf_a, bvh_b, tf_b, max_points=4000):
    """tf_* are pinocchio SE3 placements. -> dict(depth_m, a_in_b, b_in_a, a_in_b_count, b_in_a_count); vertices restricted to the overlap of the two world AABBs."""
    Va, Ta = mesh_arrays(bvh_a); Vb, Tb = mesh_arrays(bvh_b)
    Ra, ta = np.asarray(tf_a.rotation), np.asarray(tf_a.translation); Rb, tb = np.asarray(tf_b.rotation), np.asarray(tf_b.translation)
    tf_a, tf_b = _fcl_tf(Ra, ta), _fcl_tf(Rb, tb)
    Wa = Va @ Ra.T + ta; Wb = Vb @ Rb.T + tb
    lo = np.maximum(Wa.min(0), Wb.min(0)) - 1e-4; hi = np.minimum(Wa.max(0), Wb.max(0)) + 1e-4
    out = {'depth_m': 0.0, 'a_in_b': 0.0, 'b_in_a': 0.0, 'a_in_b_count': 0, 'b_in_a_count': 0}
    if np.any(lo > hi): return out
    for key, W, Vo, To, bvh_o, tf_o in (('a_in_b', Wa, Wb, Tb, bvh_b, tf_b), ('b_in_a', Wb, Wa, Ta, bvh_a, tf_a)):
        sel = np.all((W >= lo) & (W <= hi), axis=1); P = W[sel]
        if len(P) > max_points: P = P[np.linspace(0, len(P) - 1, max_points).astype(int)]
        if not len(P): continue
        ins, _ = inside_mask(P, Vo, To); out[key + '_count'] = int(ins.sum())
        if ins.any(): out[key] = float(surface_distance(P[ins], bvh_o, tf_o).max())
    out['depth_m'] = max(out['a_in_b'], out['b_in_a']); return out
