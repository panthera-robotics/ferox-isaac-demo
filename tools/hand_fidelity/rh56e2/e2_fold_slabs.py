"""Q02-r4 asset-representation step for the PUBLIC exact-E2 prior hand (physics topology + palm collider; geometry, joint limits
and masses unchanged):

  fold_fixed_hand_links(hand_xml, side): the hand's fixed sub-links that carry inertials (palm_1, palm_2, palm_force_sensor, the
      16 finger/thumb force-sensor pads, tcp) are folded into their parent bodies — inertials composed exactly (parallel-axis
      theorem), visual/collision elements re-parented with the composed fixed transform; the folded links stay in the URDF as
      EMPTY links on their fixed joints, i.e. published frames (the importer path turns them into nonphysical Xforms). Result:
      one palm body adjacent to every proximal joint (PhysX parent-child filtering covers palm <-> finger roots as on the donor)
      and no sub-gram articulation links.
  slab_palm_collider(hand_xml, side, mesh_root, out_dir): the merged palm body's three palm mesh colliders (base_link, plam_1,
      plam_2) are replaced by the declared collider e2_palm_yz_slabs_v1 = conservative convex pieces from 4 mm x/z cells of the
      EXACT palm triangles in the hand_base_link frame (the donor's generic inspire_collision.source_slab_hulls; columns along y =
      across the palm), written as binary STL pieces; visuals untouched. The hollow-shell convex decomposition of the importer
      can no longer bulge over the finger roots / thumb cavity.
Both are declared in the manifest (representation record) and audited on the actual collision pieces.
"""
from __future__ import annotations

import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

FOLD_SUFFIXES = ('palm_1', 'palm_2', 'palm_force_sensor', 'tcp')
SLAB_ID = 'e2_palm_yz_slabs_v1'; SLAB_WIDTH_M = 0.004; SLAB_AXES = (0, 2)   # bounded x (palm normal) and z (finger direction) in the hand_base_link frame; columns along y
# r5 (Q02-r5): thumb-cavity carve-out — the smoke hdR-r4-bench-smoke showed right_thumb_metacarpal in permanent contact with the r4 palm pieces from
# initialization (the thumb rests in the palm cavity with ~0.4 mm design clearance; slab conservatism + PhysX contact offset close it). Pieces whose
# hull comes within CARVE_MARGIN_M of the thumb's swept collision surface (yaw x bend sweep at the URDF open fingers) are rebuilt at CARVE_WIDTH_M
# cells and every sub-piece still within the margin is dropped: the palm collider loses only the thin skin the thumb sweeps through.
SLAB_ID_V2 = 'e2_palm_yz_slabs_v2_thumb_cavity'; CARVE_WIDTH_M = 0.001; CARVE_MARGIN_M = 0.0015


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def R_to_rpy(R):
    p = -math.asin(max(-1.0, min(1.0, R[2, 0]))); return [math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])]


def origin_of(el):
    o = el.find('origin')
    xyz = np.array([float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]); rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    return rpy_to_R(*rpy), xyz


def set_origin(el, R, t):
    o = el.find('origin')
    if o is None: o = ET.SubElement(el, 'origin')
    o.set('xyz', ' '.join('%.9g' % v for v in t)); o.set('rpy', ' '.join('%.9g' % v for v in R_to_rpy(R)))


def read_inertial(link):
    ine = link.find('inertial'); Ro, t = origin_of(ine); m = float(ine.find('mass').get('value')); I = ine.find('inertia'); g = lambda k: float(I.get(k))
    return m, t, Ro @ np.array([[g('ixx'), g('ixy'), g('ixz')], [g('ixy'), g('iyy'), g('iyz')], [g('ixz'), g('iyz'), g('izz')]]) @ Ro.T   # inertia about the COM in the link frame


def write_inertial(link, m, com, I):
    ine = link.find('inertial'); set_origin(ine, np.eye(3), com); ine.find('mass').set('value', '%.9g' % m); node = ine.find('inertia')
    for k, v in (('ixx', I[0, 0]), ('ixy', I[0, 1]), ('ixz', I[0, 2]), ('iyy', I[1, 1]), ('iyz', I[1, 2]), ('izz', I[2, 2])): node.set(k, '%.9g' % v)


def compose_inertials(mp, cp, Ip, mc, cc, Ic):
    m = mp + mc; c = (mp * cp + mc * cc) / m
    S = lambda d: float(d @ d) * np.eye(3) - np.outer(d, d)
    return m, c, Ip + mp * S(cp - c) + Ic + mc * S(cc - c)


def fold_fixed_hand_links(hand, side):
    """Fold in place; returns the record {folded child: {parent, xyz, rpy, mass, moved_elements}}."""
    links = {l.get('name'): l for l in hand.findall('link')}; joints = {j.find('child').get('link'): j for j in hand.findall('joint')}
    def foldable(n):
        s = n[len(side) + 1:]; return links[n].find('inertial') is not None and (s in FOLD_SUFFIXES or 'force_sensor' in s)
    record = {}
    # leaves first: repeat until no foldable link has a foldable descendant left
    pending = [n for n in links if n.startswith(side + '_') and foldable(n)]
    kids = {}
    for j in hand.findall('joint'): kids.setdefault(j.find('parent').get('link'), []).append(j.find('child').get('link'))
    while pending:
        progressed = False
        for n in list(pending):
            if any(k in pending for k in kids.get(n, [])): continue
            j = joints[n]; assert j.get('type') == 'fixed', n; parent = j.find('parent').get('link'); Rj, tj = origin_of(j); child = links[n]; P = links[parent]
            mc, cc, Ic = read_inertial(child); cc_p = Rj @ cc + tj; Ic_p = Rj @ Ic @ Rj.T
            mp, cp, Ip = read_inertial(P); m, c, I = compose_inertials(mp, cp, Ip, mc, cc_p, Ic_p); write_inertial(P, m, c, I)
            moved = 0
            for tag in ('visual', 'collision'):
                for el in list(child.findall(tag)):
                    Re, te = origin_of(el); set_origin(el, Rj @ Re, Rj @ te + tj); child.remove(el); P.append(el); moved += 1
            child.remove(child.find('inertial'))
            record[n] = {'parent': parent, 'xyz_m': tj.round(9).tolist(), 'rpy_rad': [round(v, 9) for v in R_to_rpy(Rj)], 'mass_kg': mc, 'moved_elements': moved, 'kept_as': 'frame (empty link on its fixed joint)'}
            pending.remove(n); progressed = True
        assert progressed, pending
    return record


def read_stl(path):
    raw = Path(path).read_bytes(); n = struct.unpack('<I', raw[80:84])[0]
    a = np.frombuffer(raw[84:84 + 50 * n], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return a['v'].astype(float)


def write_stl(path, points, faces):
    P = np.asarray(points, dtype=float); tri = np.array([[P[a], P[b], P[c]] for a, b, c in faces]); nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-18)
    rec = np.zeros(len(tri), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])); rec['n'] = nrm; rec['v'] = tri
    Path(path).write_bytes(b'e2_palm_yz_slabs_v1 piece'.ljust(80, b'\0') + struct.pack('<I', len(tri)) + rec.tobytes())


def thumb_swept_cloud(urdf_path, package_dir, side, n_yaw=12, n_pitch=6, every=2):
    """Collision-surface points of the thumb links over the yaw x bend envelope (fingers open), in the <side>_hand_base_link frame."""
    import pinocchio as pin
    m = pin.buildModelFromUrdf(str(urdf_path)); g = pin.buildGeomFromUrdf(m, str(urdf_path), pin.GeometryType.COLLISION, package_dirs=[str(package_dir)]); d = m.createData(); gd = pin.GeometryData(g)
    link = {o.name: m.frames[o.parentFrame].name for o in g.geometryObjects}; thumb = [i for i, o in enumerate(g.geometryObjects) if link[o.name].startswith(side + '_thumb')]
    verts = {i: np.asarray(g.geometryObjects[i].geometry.vertices(), dtype=float)[::every] for i in thumb}
    J = {'yaw': side + '_thumb_proximal_yaw_joint', 'pitch': side + '_thumb_proximal_pitch_joint', 'inter': side + '_thumb_intermediate_joint', 'distal': side + '_thumb_distal_joint'}
    cloud = []
    for yaw in np.linspace(0.0, 1.658, n_yaw):
        for pitch in np.linspace(0.0, 0.62, n_pitch):
            q = pin.neutral(m); q[m.idx_qs[m.getJointId(J['yaw'])]] = yaw; q[m.idx_qs[m.getJointId(J['pitch'])]] = pitch
            q[m.idx_qs[m.getJointId(J['inter'])]] = min(0.8392 * pitch, 0.5410520681); q[m.idx_qs[m.getJointId(J['distal'])]] = min(0.7477272 * pitch, 0.45553093)
            pin.forwardKinematics(m, d, q); pin.updateFramePlacements(m, d); pin.updateGeometryPlacements(m, d, g, gd); H = d.oMf[m.getFrameId(side + '_hand_base_link')]
            for i in thumb:
                T = H.inverse() * gd.oMg[i]; cloud.append(verts[i] @ np.asarray(T.rotation).T + np.asarray(T.translation))
    return np.vstack(cloud)


def _hull_eqs(points):
    from scipy.spatial import ConvexHull
    try: return ConvexHull(np.asarray(points, dtype=float)).equations
    except Exception: return None


def _within(eqs, cloud, margin):
    """True if any cloud point lies inside the hull dilated by margin."""
    if eqs is None or not len(cloud): return False
    s = eqs[:, :3] @ cloud.T + eqs[:, 3:4]
    return bool(np.any(np.all(s <= margin, axis=0)))


def slab_palm_collider(hand, side, mesh_root, out_dir, ferox_tools, carve_cloud=None):
    """Replace the merged palm body's palm mesh colliders with slab pieces. mesh_root: directory holding meshes/<side>/*.STL as referenced
    by the URDF (filename attribute is relative to the URDF dir); out_dir: the asset dir (pieces go to meshes/<side>/palm_slabs/)."""
    sys.path.insert(0, str(ferox_tools))
    from inspire_collision import source_slab_hulls
    palm = [l for l in hand.findall('link') if l.get('name') == side + '_hand_base_link'][0]
    tris = []; replaced = []
    for col in list(palm.findall('collision')):
        mesh = col.find('geometry/mesh'); fn = mesh.get('filename') if mesh is not None else ''
        if not any(k in fn for k in ('base_link.STL', 'plam_1.STL', 'plam_2.STL')): continue
        R, t = origin_of(col); V = read_stl(Path(mesh_root) / fn); tris.extend([[tuple(R @ p + t) for p in tri] for tri in V]); palm.remove(col); replaced.append(fn)
    pieces, zero = source_slab_hulls(tris, axes=SLAB_AXES, width_m=SLAB_WIDTH_M)
    carve = None; cid = SLAB_ID
    if carve_cloud is not None:
        cid = SLAB_ID_V2; cloud = np.asarray(carve_cloud, dtype=float)
        offending = {tuple(pc['slab_cell']) for pc in pieces if _within(_hull_eqs(pc['points']), cloud, CARVE_MARGIN_M)}
        fine, _ = source_slab_hulls(tris, axes=SLAB_AXES, width_m=CARVE_WIDTH_M) if offending else ([], [])
        ratio = int(round(SLAB_WIDTH_M / CARVE_WIDTH_M)); kept = []; dropped = 0; sub_used = 0
        for pc in pieces:
            if tuple(pc['slab_cell']) not in offending: kept.append(pc)
        for fp in fine:
            coarse = tuple(math.floor(c / ratio) for c in fp['slab_cell'])
            if coarse not in offending: continue
            if _within(_hull_eqs(fp['points']), cloud, CARVE_MARGIN_M): dropped += 1; continue
            kept.append(dict(fp, carved_from_cell=list(coarse))); sub_used += 1
        carve = {'margin_m': CARVE_MARGIN_M, 'fine_width_m': CARVE_WIDTH_M, 'coarse_cells_rebuilt': len(offending), 'coarse_pieces_replaced': sum(1 for pc in pieces if tuple(pc['slab_cell']) in offending), 'fine_pieces_kept': sub_used, 'fine_pieces_dropped': dropped,
                 'swept_cloud_points': int(len(cloud)), 'sweep': 'thumb links (incl. folded pads) over yaw [0,1.658] x bend [0,0.62], fingers open, 12 x 6 poses'}
        pieces = kept
    pdir = Path(out_dir) / 'meshes' / side / 'palm_slabs'; pdir.mkdir(parents=True, exist_ok=True); names = []; vol = 0.0
    for i, pc in enumerate(pieces):
        name = 'meshes/%s/palm_slabs/%s_%03d.stl' % (side, cid, i); write_stl(Path(out_dir) / name, pc['points'], pc['faces']); names.append(name); vol += pc.get('volume_m3', 0.0)
        col = ET.SubElement(palm, 'collision'); o = ET.SubElement(col, 'origin'); o.set('xyz', '0 0 0'); o.set('rpy', '0 0 0'); g = ET.SubElement(col, 'geometry'); m = ET.SubElement(g, 'mesh'); m.set('filename', name)
    return {'candidate_id': cid, 'slab_axes': ['XYZ'[a] for a in SLAB_AXES], 'slab_width_m': SLAB_WIDTH_M, 'frame': side + '_hand_base_link (palm normal +x, across y, fingers -z)', 'source_meshes_replaced': replaced, 'source_triangles': len(tris),
            'pieces': len(pieces), 'zero_volume_cells': len(zero), 'pieces_volume_cm3': round(vol * 1e6, 2), 'piece_files': names, 'thumb_cavity_carve': carve,
            'method': 'inspire_collision.source_slab_hulls on the exact E2 palm triangles (conservative closed convex hull per 4 mm x/z cell, split to <= 120 vertices); visuals untouched' + ('; r5: cells within %.1f mm of the thumb swept collision surface rebuilt at %.0f mm and sub-pieces still within the margin dropped' % (CARVE_MARGIN_M * 1e3, CARVE_WIDTH_M * 1e3) if carve else '')}
