"""Source-to-model profile of one hand in a URDF: frames, axes, limits, coupling, chirality, FK sweeps and mass
properties, all computed on the CPU from the asset itself (no simulator, no generated imagery).

Frames: every position is reported in the hand ROOT frame (``<side>_base_link``, the flange-side link of the hand)
and, when the URDF contains the wrist, also in the ``<side>_wrist_yaw_link`` frame. The two are never mixed in one
table. Units are metres, radians and kilograms.
"""
from __future__ import annotations

import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .coupling import coupling_table, range_limit_findings, read_joints

FINGERS = ('index', 'middle', 'ring', 'little')
# semantic actuator -> donor joint suffix (RULE-HAND-NAME: by name, never by index)
ACTUATOR_JOINT = {'index': 'index_1_joint', 'middle': 'middle_1_joint', 'ring': 'ring_1_joint', 'little': 'little_1_joint',
                  'thumb_bend': 'thumb_2_joint', 'thumb_rotation': 'thumb_1_joint'}
NATIVE_ORDER = ('little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation')


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, angle):
    a = np.asarray(axis, dtype=float); n = np.linalg.norm(a)
    if n < 1e-12:
        raise ValueError('zero joint axis')
    x, y, z = a / n
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def read_stl(path):
    """Vertices (N, 3) of a binary or ASCII STL, without third-party mesh libraries."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if 84 + n * 50 == len(data):
            rec = np.frombuffer(data[84:], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n)
            return rec['v'].reshape(-1, 3).astype(float)
    verts = []
    for line in data.decode('ascii', errors='ignore').splitlines():
        t = line.split()
        if len(t) == 4 and t[0] == 'vertex':
            verts.append([float(t[1]), float(t[2]), float(t[3])])
    if not verts:
        raise ValueError('unreadable STL ' + str(path))
    return np.asarray(verts, dtype=float)


class HandUrdf:
    """One hand (by side prefix) inside a merged robot URDF or a bench URDF whose root is the hand base link."""

    def __init__(self, urdf_path, side):
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        self.path = Path(urdf_path); self.side = side
        self.sha256 = sha256_file(self.path)
        self.root_xml = ET.parse(self.path).getroot()
        self.joints = read_joints(self.path)
        self.links = {l.get('name'): l for l in self.root_xml.findall('link')}
        self.base = side + '_base_link'
        if self.base not in self.links:
            raise ValueError('URDF has no %s' % self.base)
        # hand subtree = base link and everything below it
        children = {}
        for name, j in self.joints.items():
            children.setdefault(j['parent'], []).append((name, j['child']))
        self.hand_links, self.hand_joints = [self.base], []
        stack = [self.base]
        while stack:
            link = stack.pop()
            for jn, child in children.get(link, []):
                self.hand_joints.append(jn); self.hand_links.append(child); stack.append(child)
        self.mount = None
        for name, j in self.joints.items():
            if j['child'] == self.base:
                self.mount = dict(name=name, parent=j['parent'], xyz=j['xyz'], rpy=j['rpy'], type=j['type'])
        self.independent = [n for n in self.hand_joints if self.joints[n]['type'] == 'revolute' and not self.joints[n]['mimic']]
        self.coupled = [n for n in self.hand_joints if self.joints[n]['type'] == 'revolute' and self.joints[n]['mimic']]
        self.actuators = {}
        for act, suffix in ACTUATOR_JOINT.items():
            jn = side + '_' + suffix
            if jn not in self.independent:
                raise ValueError('expected independent joint %s' % jn)
            self.actuators[act] = jn
        if len(self.independent) != 6:
            raise ValueError('expected six independent hand joints, found %s' % self.independent)

    # ---- kinematics ------------------------------------------------------------------------------------------
    def joint_values(self, q):
        """Independent values by name -> all hand joint values with mimics composed (unknown independents = 0)."""
        unknown = set(q) - set(self.independent)
        if unknown:
            raise ValueError('not independent hand joints: %s' % sorted(unknown))
        values = {n: 0.0 for n in self.hand_joints}
        for n in self.independent:
            v = float(q.get(n, 0.0))
            if not math.isfinite(v):
                raise ValueError('nonfinite joint value ' + n)
            values[n] = v
        table = coupling_table(self.joints)
        for n in self.coupled:
            row = table[n]; values[n] = row['composed_multiplier'] * values[row['driver']] + row['composed_offset']
        return values

    def frames(self, q, in_wrist_frame=False):
        """{link: 4x4} for the hand links, in the base-link frame (default) or the wrist_yaw frame."""
        values = self.joint_values(q)
        T0 = np.eye(4)
        if in_wrist_frame:
            if not self.mount or self.mount['type'] != 'fixed':
                raise ValueError('no fixed wrist mount in this URDF')
            T0[:3, :3] = rpy_matrix(*self.mount['rpy']); T0[:3, 3] = self.mount['xyz']
        frames = {self.base: T0}
        pending = list(self.hand_joints)
        while pending:
            progressed = False
            for jn in list(pending):
                j = self.joints[jn]
                if j['parent'] in frames:
                    T = np.eye(4); T[:3, :3] = rpy_matrix(*j['rpy']); T[:3, 3] = j['xyz']
                    if j['type'] == 'revolute':
                        T[:3, :3] = T[:3, :3] @ axis_angle(j['axis'], values[jn])
                    frames[j['child']] = frames[j['parent']] @ T
                    pending.remove(jn); progressed = True
            if not progressed:
                raise ValueError('disconnected hand joints ' + ','.join(pending))
        return frames

    def actuator_command(self, closure):
        """Closure fractions per actuator (0 = URDF lower limit = donor open, 1 = upper limit) -> joint values."""
        q = {}
        for act, c in closure.items():
            jn = self.actuators[act]; lo, hi = self.joints[jn]['limit']
            c = float(c)
            if not 0.0 <= c <= 1.0:
                raise ValueError('closure must be within [0, 1] for %s' % act)
            q[jn] = lo + c * (hi - lo)
        return q

    # ---- meshes ------------------------------------------------------------------------------------------------
    def link_vertices(self, link, kind='visual'):
        g = self.links[link].find(kind + '/geometry/mesh')
        if g is None:
            return None
        v = read_stl(self.path.parent / g.get('filename'))
        if g.get('scale'):
            v = v * np.array([float(s) for s in g.get('scale').split()])
        o = self.links[link].find(kind + '/origin')
        if o is not None:
            R = rpy_matrix(*[float(s) for s in o.get('rpy', '0 0 0').split()]); t = [float(s) for s in o.get('xyz', '0 0 0').split()]
            v = v @ R.T + t
        return v

    def meshes_available(self):
        try:
            for link in self.hand_links:
                self.link_vertices(link)
            return True
        except (FileNotFoundError, ValueError):
            return False

    # ---- profile -----------------------------------------------------------------------------------------------
    def chirality(self):
        """Right hand iff, facing the palm with the fingers up, the thumb base is on the viewer's right.
        fingers f = mean MCP->distal-joint direction (open); flexion normal n = direction the index tip moves when
        index_1 closes (projected off f); viewer's right = f x n."""
        open_f = self.frames({})
        mcps = np.array([open_f[self.side + '_' + f + '_1'][:3, 3] for f in FINGERS])
        dists = np.array([open_f[self.side + '_' + f + '_2'][:3, 3] for f in FINGERS])
        f = (dists - mcps).mean(0); f /= np.linalg.norm(f)
        tip0 = open_f[self.side + '_index_2'][:3, 3]
        tip1 = self.frames({self.actuators['index']: 0.3})[self.side + '_index_2'][:3, 3]
        n = tip1 - tip0; n -= f * (n @ f); n /= np.linalg.norm(n)
        thumb = open_f[self.side + '_thumb_1'][:3, 3] - mcps.mean(0)
        lateral = np.cross(f, n)
        s = float(thumb @ lateral)
        return {'fingers_direction_root': f.round(6).tolist(), 'flexion_normal_root': n.round(6).tolist(), 'viewer_right_root': lateral.round(6).tolist(),
                'thumb_base_offset_along_viewer_right_m': round(s, 6), 'verdict': 'RIGHT' if s > 0 else 'LEFT', 'declared_side': self.side,
                'consistent': (s > 0) == (self.side == 'right')}

    def thumb_rotation_datum(self):
        """Angle between the thumb (thumb_1 origin -> thumb_2 origin... -> thumb_4 origin line) and the palm plane
        spanned by the finger direction and the viewer-right axis, at the open and closed rotation endpoints, with
        the thumb bend open. Reported so the E2 'rotation angle beta' (measured from the metacarpal plane) can be
        compared with a datum instead of a bare travel."""
        ch = self.chirality(); f = np.array(ch['fingers_direction_root']); n = np.array(ch['flexion_normal_root'])
        jn = self.actuators['thumb_rotation']; lo, hi = self.joints[jn]['limit']
        out = {}
        for label, q in (('open', lo), ('closed', hi)):
            fr = self.frames({jn: q})
            base = fr[self.side + '_thumb_1'][:3, 3]; tip = fr[self.side + '_thumb_4'][:3, 3]
            d = tip - base; d /= np.linalg.norm(d)
            out[label] = {'q_rad': q, 'thumb_direction_root': d.round(6).tolist(),
                          'elevation_from_palm_plane_deg': round(math.degrees(math.asin(float(np.clip(d @ n, -1, 1)))), 3),
                          'note': 'elevation > 0: thumb lifted toward the flexion (palm) side; 0: thumb lies in the palm plane'}
        out['travel_deg'] = round(math.degrees(hi - lo), 3)
        out['axis_root'] = self.frames({})[self.side + '_thumb_1'][:3, :3] @ np.asarray(self.joints[jn]['axis'], float)
        out['axis_root'] = out['axis_root'].round(6).tolist()
        out['axis_alignment_with_fingers'] = round(float(np.dot(out['axis_root'], f)), 4)
        return out

    def fk_sweep(self, closures=(0.0, 0.5, 1.0), in_wrist_frame=False):
        """Positions of the MCP/base joint origins, the distal joint origins and the fingertip sensor frames for
        all-actuator closure fractions, plus per-actuator single sweeps. Frame is stated in the result."""
        frame = (self.side + '_wrist_yaw_link') if in_wrist_frame else self.base
        res = {'frame': frame, 'closure_definition': '0 = URDF lower limit (donor open), 1 = URDF upper limit (donor closed); NOT an E2 register value', 'all_actuators': {}, 'single_actuator': {}}

        def points(fr):
            pts = {}
            for fname in FINGERS:
                pts[fname] = {'mcp_joint_origin': fr[self.side + '_' + fname + '_1'][:3, 3].round(5).tolist(),
                              'distal_joint_origin': fr[self.side + '_' + fname + '_2'][:3, 3].round(5).tolist()}
                s3 = self.side + '_' + fname + '_force_sensor_3'
                if s3 in fr:
                    pts[fname]['tip_sensor_frame'] = fr[s3][:3, 3].round(5).tolist()
            pts['thumb'] = {'rotation_joint_origin': fr[self.side + '_thumb_1'][:3, 3].round(5).tolist(), 'bend_joint_origin': fr[self.side + '_thumb_2'][:3, 3].round(5).tolist(),
                            'distal_joint_origin': fr[self.side + '_thumb_4'][:3, 3].round(5).tolist()}
            s4 = self.side + '_thumb_force_sensor_4'
            if s4 in fr:
                pts['thumb']['tip_sensor_frame'] = fr[s4][:3, 3].round(5).tolist()
            return pts

        for c in closures:
            q = self.actuator_command({a: c for a in self.actuators})
            fr = self.frames(q, in_wrist_frame)
            res['all_actuators']['closure_%.2f' % c] = {'joint_values_rad': {k: round(v, 5) for k, v in self.joint_values(q).items()}, 'points_m': points(fr)}
        for act in self.actuators:
            res['single_actuator'][act] = {}
            for c in closures:
                q = self.actuator_command({act: c})
                res['single_actuator'][act]['closure_%.2f' % c] = points(self.frames(q, in_wrist_frame))
        return res

    def mesh_extents(self):
        """Open-hand mesh-derived lengths (source STL vertices transformed by FK); None when meshes are missing."""
        if not self.meshes_available():
            return None
        fr = self.frames({})
        out = {}
        for fname in FINGERS:
            mcp = fr[self.side + '_' + fname + '_1'][:3, 3]
            v2 = self.link_vertices(self.side + '_' + fname + '_2'); T = fr[self.side + '_' + fname + '_2']
            w = v2 @ T[:3, :3].T + T[:3, 3]
            tip = w[np.argmax(np.linalg.norm(w - mcp, axis=1))]
            out[fname + '_mcp_to_distal_tip_m'] = round(float(np.linalg.norm(tip - mcp)), 5)
            out[fname + '_proximal_segment_m'] = round(float(np.linalg.norm(fr[self.side + '_' + fname + '_2'][:3, 3] - mcp)), 5)
        tb = fr[self.side + '_thumb_1'][:3, 3]
        v4 = self.link_vertices(self.side + '_thumb_4'); T = fr[self.side + '_thumb_4']; w = v4 @ T[:3, :3].T + T[:3, 3]
        out['thumb_base_to_tip_m'] = round(float(np.max(np.linalg.norm(w - tb, axis=1))), 5)
        base = self.link_vertices(self.base)
        allv = [base] + [self.link_vertices(l) @ fr[l][:3, :3].T + fr[l][:3, 3] for l in self.hand_links if l != self.base and self.link_vertices(l) is not None]
        allv = np.vstack(allv)
        out['open_envelope_root_min_m'] = allv.min(0).round(4).tolist(); out['open_envelope_root_max_m'] = allv.max(0).round(4).tolist()
        ch = self.chirality(); f = np.array(ch['fingers_direction_root']); n = np.array(ch['flexion_normal_root']); r = np.array(ch['viewer_right_root'])
        band = base[(base @ f > 0.09) & (base @ f < 0.15)]
        out['palm_band_thickness_along_flexion_normal_m'] = round(float((band @ n).max() - (band @ n).min()), 5)
        out['palm_band_width_along_viewer_right_m'] = round(float((band @ r).max() - (band @ r).min()), 5)
        out['palm_band_definition'] = 'base-link vertices with 0.09 m < (v . fingers) < 0.15 m; thickness measured along the flexion normal, width along viewer-right'
        out['base_link_note'] = 'the base mesh includes the wrist cylinder; extents are mesh extremities, not manufacturer datums'
        # unit scaling: a hand's open envelope along the finger axis is 0.15-0.35 m; a millimetre or inch STL would be 25-1000x off
        length = float((allv @ f).max() - (allv @ f).min())
        out['open_length_along_fingers_m'] = round(length, 5)
        out['units_check'] = {'status': 'METRES_PLAUSIBLE' if 0.15 <= length <= 0.35 else 'UNIT_SCALE_SUSPECT', 'rule': 'open hand length along the finger axis within [0.15, 0.35] m; URDF mesh scale attribute absent or 1'}
        out['mesh_scale_attributes'] = sorted({(self.links[l].find('visual/geometry/mesh').get('scale') or '1 1 1') for l in self.hand_links if self.links[l].find('visual/geometry/mesh') is not None})
        return out

    def mass_properties(self, q=None, in_wrist_frame=False):
        """Total mass, centre of mass and inertia about the COM of the hand subtree at configuration q, expressed
        in the base-link (default) or wrist_yaw frame. Links without <inertial> are listed, never assumed."""
        q = q or {}
        fr = self.frames(q, in_wrist_frame)
        total, first, links_no_inertial, per_link = 0.0, np.zeros(3), [], []
        for link in self.hand_links:
            inert = self.links[link].find('inertial')
            if inert is None:
                links_no_inertial.append(link); continue
            m = float(inert.find('mass').get('value'))
            o = inert.find('origin')
            xyz = np.array([float(s) for s in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()])
            rpy = [float(s) for s in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            I = inert.find('inertia')
            Il = np.array([[float(I.get('ixx')), float(I.get('ixy')), float(I.get('ixz'))],
                           [float(I.get('ixy')), float(I.get('iyy')), float(I.get('iyz'))],
                           [float(I.get('ixz')), float(I.get('iyz')), float(I.get('izz'))]])
            T = fr[link]; Rl = T[:3, :3] @ rpy_matrix(*rpy); c = T[:3, :3] @ xyz + T[:3, 3]
            per_link.append((link, m, c, Rl @ Il @ Rl.T))
            total += m; first += m * c
        if total <= 0:
            raise ValueError('no authored mass in the hand subtree')
        com = first / total
        Itot = np.zeros((3, 3))
        for link, m, c, Ic in per_link:
            d = c - com
            Itot += Ic + m * ((d @ d) * np.eye(3) - np.outer(d, d))
        eig = np.linalg.eigvalsh(Itot)
        return {'frame': (self.side + '_wrist_yaw_link') if in_wrist_frame else self.base, 'configuration_rad': {k: float(v) for k, v in q.items()} or 'URDF zero (donor open)',
                'mass_kg': round(total, 6), 'com_m': com.round(6).tolist(), 'inertia_about_com_kg_m2': Itot.round(9).tolist(),
                'principal_moments_kg_m2': eig.round(9).tolist(), 'positive_definite': bool(eig.min() > 0),
                'links_included': [l for l, *_ in per_link], 'links_without_inertial': links_no_inertial,
                'provenance': 'URDF <inertial> per link (source-authored), composed rigidly at the stated configuration; not measured'}

    def profile(self, with_meshes=True):
        table = coupling_table(self.joints)
        table = {k: v for k, v in table.items() if k in self.coupled}
        act = {}
        for a, jn in self.actuators.items():
            j = self.joints[jn]
            act[a] = {'joint': jn, 'axis_local': j['axis'], 'limit_rad': j['limit'], 'travel_deg': round(math.degrees(j['limit'][1] - j['limit'][0]), 3),
                      'effort_nm': j['effort'], 'velocity_rad_s': j['velocity'], 'open_rad_donor_datum': j['limit'][0], 'closed_rad_donor_datum': j['limit'][1]}
        prof = {
            'schema': 'hand_fidelity_donor_profile_v1', 'side': self.side, 'urdf': self.path.name, 'urdf_sha256': self.sha256,
            'root_frame': self.base, 'mount': self.mount, 'link_count': len(self.hand_links), 'independent_joints': self.independent, 'coupled_joints': self.coupled,
            'native_order_reference': list(NATIVE_ORDER), 'actuators': act, 'coupling': table, 'coupling_findings': range_limit_findings(table),
            'chirality': self.chirality(), 'thumb_rotation_datum': self.thumb_rotation_datum(), 'fk_sweep_root': self.fk_sweep(),
            'mass_properties_open_root': self.mass_properties(), 'mass_properties_closed_root': self.mass_properties(self.actuator_command({a: 1.0 for a in self.actuators})),
        }
        if self.mount and self.mount['type'] == 'fixed':
            prof['fk_sweep_wrist'] = self.fk_sweep(in_wrist_frame=True)
            prof['mass_properties_open_wrist'] = self.mass_properties(in_wrist_frame=True)
        prof['mesh_extents_root'] = self.mesh_extents() if with_meshes else None
        prof['profile_sha256'] = hashlib.sha256(json.dumps({k: v for k, v in prof.items()}, sort_keys=True, default=str).encode()).hexdigest()
        return prof


def main(argv=None):
    import argparse
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('urdf'); p.add_argument('--side', default='right', choices=('left', 'right'))
    p.add_argument('--out'); p.add_argument('--no-meshes', action='store_true')
    a = p.parse_args(argv)
    prof = HandUrdf(a.urdf, a.side).profile(with_meshes=not a.no_meshes)
    text = json.dumps(prof, indent=1, default=str)
    if a.out:
        Path(a.out).write_text(text)
    else:
        print(text)
    return prof


if __name__ == '__main__':
    main()
