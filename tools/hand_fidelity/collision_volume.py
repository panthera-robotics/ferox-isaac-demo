"""Collision-volume versus visible-surface check per hand link, from the source STL meshes (CPU): the closed-mesh
volume (divergence theorem over the triangles), the single convex-hull volume (the upper bound of what one convex
collider would add) and the ratio. The runtime colliders are per-link convex DECOMPOSITIONS plus declared palm slabs;
their exact cooked hulls are recorded per run by the probe, so this check bounds, not measures, the runtime overlap.
Datum: link frame of each STL as authored (metres); volumes in cm^3.
"""
from __future__ import annotations

import struct
from pathlib import Path

import numpy as np

from .donor_profile import HandUrdf


def read_stl_triangles(path):
    data = Path(path).read_bytes()
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if 84 + n * 50 == len(data):
            rec = np.frombuffer(data[84:], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n)
            return rec['v'].astype(float)
    tris, cur = [], []
    for line in data.decode('ascii', errors='ignore').splitlines():
        t = line.split()
        if len(t) == 4 and t[0] == 'vertex':
            cur.append([float(t[1]), float(t[2]), float(t[3])])
            if len(cur) == 3:
                tris.append(cur); cur = []
    return np.asarray(tris, dtype=float)


def mesh_volume_m3(tris):
    """Signed volume sum of tetrahedra (origin, triangle); exact for closed, consistently oriented meshes."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    return float(abs(np.einsum('ij,ij->i', a, np.cross(b, c)).sum()) / 6.0)


def hull_volume_m3(points):
    from scipy.spatial import ConvexHull
    return float(ConvexHull(points).volume)


def link_table(hand: HandUrdf):
    rows = {}
    for link in hand.hand_links:
        g = hand.links[link].find('visual/geometry/mesh')
        if g is None:
            continue
        tris = read_stl_triangles(hand.path.parent / g.get('filename'))
        if tris.size == 0:
            continue
        vm = mesh_volume_m3(tris); pts = tris.reshape(-1, 3); hv = hull_volume_m3(pts)
        extent = (pts.max(0) - pts.min(0))
        rows[link] = {'triangles': int(len(tris)), 'mesh_volume_cm3': round(vm * 1e6, 3), 'convex_hull_volume_cm3': round(hv * 1e6, 3),
                      'hull_over_mesh': round(hv / vm, 3) if vm > 0 else None, 'bbox_extent_mm': (extent * 1e3).round(1).tolist(),
                      'collision_mesh_same_file': (hand.links[link].find('collision/geometry/mesh') is not None and hand.links[link].find('collision/geometry/mesh').get('filename') == g.get('filename'))}
    return rows


def summary(hand: HandUrdf):
    rows = link_table(hand)
    worst = sorted(((v['hull_over_mesh'] or 0, k) for k, v in rows.items()), reverse=True)[:5]
    return {'side': hand.side, 'urdf_sha256': hand.sha256, 'links': rows, 'all_visual_equal_collision_file': all(v['collision_mesh_same_file'] for v in rows.values()),
            'largest_single_hull_overestimates': [{'link': k, 'hull_over_mesh': r} for r, k in worst],
            'reading': 'a single convex hull would overestimate the concave links (palm/base with the wrist cylinder, curled phalanges) by the stated ratio; the runtime uses convex decompositions + declared palm slabs, so the true runtime overlap lies between 1.0 and this bound and is recorded per run in backend_shapes.json / collision_candidate.json'}
