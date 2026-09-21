#!/usr/bin/env python3
"""Static audit of the PUBLIC exact-E2 prior asset (before any physics pass). Every check is CPU-only (pinocchio + hpp-fcl on the
merged URDF and its meshes) and is reported PASS / FAIL / INFO with the measured numbers, never a bare verdict.

Checks (per hand unless stated): chirality against the frozen donor (thumb anterior, palm normal towards the midline, left = y-mirror
of right), 6 active coordinates + 12 mechanical joints, coupling graph (every mimic follows an active joint of the same hand, no
double-driven child, no mimic chains), finite limits, finite FK at open / closed / thumb-opposed, thumb rotation reaches the 1.658 rad
prior, left inertials = mirrored right inertials (in the hand-base frame), left pinky-intermediate mass, finite positive masses and
the declared mass policy, mesh scale (bounding boxes in metres, scale attributes), no LFS stubs (from the manifest), 17 tactile
frames, self-collision preflight (all non-adjacent pairs inside each hand at open / closed / opposed), palm/thumb cavity sweep.
usage: e2prior_audit.py --asset-dir <generated/e2prior> --donor-dir <generated/ftp_donor> --out <audit.json> [--md <audit.md>]
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import hppfcl
import numpy as np
import pinocchio as pin

try:
    from . import mesh_penetration as mp
except ImportError:
    import mesh_penetration as mp

PRIOR = {'finger_proximal_rad': 1.4381, 'finger_distal_multiplier': 1.0843, 'finger_distal_cap_rad': 1.476374, 'thumb_bend_rad': 0.62,
         'thumb_intermediate_multiplier': 0.8392, 'thumb_distal_multiplier': 0.7477272, 'thumb_rotation_rad': 1.658}
SIDES = ('right', 'left')
BODY_KEYS = ('shoulder', 'elbow', 'wrist', 'hip', 'knee', 'ankle')
ACTIVE = ['thumb_proximal_yaw_joint', 'thumb_proximal_pitch_joint', 'index_proximal_joint', 'middle_proximal_joint', 'ring_proximal_joint', 'pinky_proximal_joint']


def urdf_joints(robot_xml, side):
    out = {}
    for j in robot_xml.findall('joint'):
        n = j.get('name')
        if not n.startswith(side + '_') or j.get('type') != 'revolute' or any(k in n for k in BODY_KEYS): continue
        l = j.find('limit'); m = j.find('mimic')
        out[n] = {'parent': j.find('parent').get('link'), 'child': j.find('child').get('link'), 'axis': [float(v) for v in j.find('axis').get('xyz').split()],
                  'lower': float(l.get('lower')), 'upper': float(l.get('upper')), 'effort': float(l.get('effort')), 'velocity': float(l.get('velocity')),
                  'mimic': None if m is None else {'joint': m.get('joint'), 'multiplier': float(m.get('multiplier', 1)), 'offset': float(m.get('offset', 0))}}
    return out


def hand_config(model, joints, side, active_values):
    """q for one hand from the six active values with the public coupling applied (mimic multiplier, clamped to the mimic limit)."""
    q = pin.neutral(model)
    for name, info in joints.items():
        if info['mimic'] is None: val = active_values.get(name.replace(side + '_', '', 1), 0.0)
        else: val = info['mimic']['multiplier'] * active_values.get(info['mimic']['joint'].replace(side + '_', '', 1), 0.0) + info['mimic']['offset']
        val = min(max(val, info['lower']), info['upper']); q[model.idx_qs[model.getJointId(name)]] = val
    return q


def hand_links(robot_xml, side):
    return [l.get('name') for l in robot_xml.findall('link') if l.get('name').startswith(side + '_') and not any(k in l.get('name') for k in BODY_KEYS)]


def adjacent_pairs(robot_xml):
    adj = set()
    for j in robot_xml.findall('joint'):
        adj.add(frozenset((j.find('parent').get('link'), j.find('child').get('link'))))
    return adj


def fixed_groups(robot_xml):
    """Map link -> representative of its rigidly connected group (links joined by fixed joints move together: skip those pairs)."""
    parent = {}
    def find(x):
        while parent.get(x, x) != x: x = parent[x]
        return x
    for j in robot_xml.findall('joint'):
        if j.get('type') == 'fixed':
            a, b = find(j.find('parent').get('link')), find(j.find('child').get('link'))
            if a != b: parent[a] = b
    return find


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--asset-dir', required=True); ap.add_argument('--donor-dir', required=True); ap.add_argument('--out', required=True); ap.add_argument('--md')
    a = ap.parse_args(); asset = Path(a.asset_dir); donor = Path(a.donor_dir)
    manifest = json.loads((asset / 'E2_PRIOR_ASSET_MANIFEST.json').read_text())
    urdf = asset / manifest['merged_urdf']['name']; xml = ET.parse(urdf).getroot()
    model = pin.buildModelFromUrdf(str(urdf)); data = model.createData()
    geom = pin.buildGeomFromUrdf(model, str(urdf), pin.GeometryType.COLLISION, package_dirs=[str(asset)])
    dmodel = pin.buildModelFromUrdf(str(donor / 'g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf')); ddata = dmodel.createData()
    checks = []
    def check(name, ok, detail, side=None, level=None):
        checks.append({'check': name, 'side': side, 'result': level or ('PASS' if ok else 'FAIL'), 'detail': detail})

    # ---- joints, coupling, limits
    J = {s: urdf_joints(xml, s) for s in SIDES}
    for s in SIDES:
        js = J[s]; active = [n for n, i in js.items() if i['mimic'] is None]; mim = [n for n, i in js.items() if i['mimic'] is not None]
        check('mechanical_joints_12', len(js) == 12, {'count': len(js)}, s); check('active_coordinates_6', len(active) == 6 and sorted(n.replace(s + '_', '', 1) for n in active) == sorted(ACTIVE), {'active': active}, s)
        children = {}; problems = []
        for n, i in js.items():
            children.setdefault(i['child'], []).append(n)
            if i['mimic']:
                src = i['mimic']['joint']
                if src not in js: problems.append('%s mimics %s which is not a joint of this hand' % (n, src))
                elif js[src]['mimic'] is not None: problems.append('%s mimics a mimic joint %s' % (n, src))
                if i['mimic']['offset'] != 0: problems.append('%s has a non-zero mimic offset' % n)
        double = {c: v for c, v in children.items() if len(v) > 1}
        check('coupling_graph_valid', not problems and not double, {'mimics': {n: js[n]['mimic'] for n in mim}, 'problems': problems, 'double_driven_children': double}, s)
        finite = all(math.isfinite(i['lower']) and math.isfinite(i['upper']) and i['upper'] > i['lower'] for i in js.values())
        check('limits_finite_ordered', finite, {n: [i['lower'], i['upper']] for n, i in js.items()}, s)
        placeholders = {n: (i['effort'], i['velocity']) for n, i in js.items()}
        p = {'thumb_rotation_upper': js[s + '_thumb_proximal_yaw_joint']['upper'], 'thumb_bend_upper': js[s + '_thumb_proximal_pitch_joint']['upper'], 'finger_proximal_upper': js[s + '_index_proximal_joint']['upper'],
             'finger_intermediate_upper': js[s + '_index_intermediate_joint']['upper'], 'finger_multiplier': js[s + '_index_intermediate_joint']['mimic']['multiplier'],
             'thumb_intermediate_multiplier': js[s + '_thumb_intermediate_joint']['mimic']['multiplier'], 'thumb_distal_multiplier': js[s + '_thumb_distal_joint']['mimic']['multiplier']}
        ok = (abs(p['thumb_rotation_upper'] - PRIOR['thumb_rotation_rad']) < 1e-6 and abs(p['thumb_bend_upper'] - PRIOR['thumb_bend_rad']) < 1e-6 and abs(p['finger_proximal_upper'] - PRIOR['finger_proximal_rad']) < 1e-6
              and abs(p['finger_intermediate_upper'] - PRIOR['finger_distal_cap_rad']) < 1e-6 and abs(p['finger_multiplier'] - PRIOR['finger_distal_multiplier']) < 1e-6
              and abs(p['thumb_intermediate_multiplier'] - PRIOR['thumb_intermediate_multiplier']) < 1e-6 and abs(p['thumb_distal_multiplier'] - PRIOR['thumb_distal_multiplier']) < 1e-6)
        check('limits_match_public_prior', ok, {'urdf': p, 'prior': PRIOR}, s)
        check('thumb_rotation_reaches_prior', abs(p['thumb_rotation_upper'] - 1.658) < 1e-6, {'upper_rad': p['thumb_rotation_upper'], 'deg': math.degrees(p['thumb_rotation_upper'])}, s)
        sat = {}
        for n, i in js.items():
            if not i['mimic']: continue
            src = js[i['mimic']['joint']]; composed = i['mimic']['multiplier'] * src['upper'] + i['mimic']['offset']
            sat[n] = {'child_upper': i['upper'], 'composed_at_root_upper': round(composed, 6), 'child_saturates': composed > i['upper'] + 1e-9, 'root_value_at_child_limit': round((i['upper'] - i['mimic']['offset']) / i['mimic']['multiplier'], 6) if composed > i['upper'] + 1e-9 else None}
        check('mimic_child_limits_vs_composed_range', True, {'per_mimic': sat, 'policy': 'public limits kept verbatim; the coupled TARGET is clamped at the child limit (E2_KINEMATIC_CONTRACT.json coupling.clamp_at_child_limit); a child that saturates before its root reaches the root limit is a public-source fact, not a twin defect'}, s, level='INFO')
        dp = manifest.get('drive_limit_policy', {}); per = dp.get('per_joint', {})
        ok = bool(per) and all(n in per and abs(i['effort'] - per[n]['twin_effort']) < 1e-9 and abs(i['velocity'] - per[n]['twin_velocity']) < 1e-9 for n, i in js.items()) and len({(i['effort'], i['velocity']) for i in js.values()}) == 1
        check('drive_limits_declared_policy', ok, {'merged_urdf_values': {n: (i['effort'], i['velocity']) for n, i in js.items()}, 'policy': {k: v for k, v in dp.items() if k != 'per_joint'}, 'public_placeholders': {n: (per.get(n, {}).get('public_effort'), per.get(n, {}).get('public_velocity')) for n in js}}, s)

    # ---- FK finite at open / closed / opposed, chirality vs donor
    poses = {'open': {}, 'closed': {n: PRIOR['finger_proximal_rad'] for n in ACTIVE[2:]} | {'thumb_proximal_pitch_joint': PRIOR['thumb_bend_rad'], 'thumb_proximal_yaw_joint': PRIOR['thumb_rotation_rad']},
             'opposed_open_fingers': {'thumb_proximal_yaw_joint': PRIOR['thumb_rotation_rad'], 'thumb_proximal_pitch_joint': PRIOR['thumb_bend_rad']}}
    qs = {}
    for pname, vals in poses.items():
        q = pin.neutral(model)
        for s in SIDES:
            qh = hand_config(model, J[s], s, vals); q = np.where(qh != 0, qh, q)
        qs[pname] = q; pin.forwardKinematics(model, data, q); pin.updateFramePlacements(model, data)
        finite = all(np.all(np.isfinite(data.oMf[i].homogeneous)) for i in range(model.nframes))
        tips = {s: data.oMf[model.getFrameId(s + '_index_force_sensor_3')].translation.round(4).tolist() for s in SIDES}
        check('fk_finite_' + pname, finite, {'index_tip_pelvis_frame_m': tips}, level=None if finite else 'FAIL')
    q0 = qs['open']; pin.forwardKinematics(model, data, q0); pin.updateFramePlacements(model, data); pin.forwardKinematics(dmodel, ddata, pin.neutral(dmodel)); pin.updateFramePlacements(dmodel, ddata)
    chir = {}
    DONOR = {'index_proximal': 'index_1', 'pinky_proximal': 'little_1', 'thumb_proximal': 'thumb_1'}
    for s in SIDES:
        base = data.oMf[model.getFrameId(s + '_base')]; R = base.rotation
        mid = data.oMf[model.getFrameId(s + '_middle_proximal')].translation; thumb = data.oMf[model.getFrameId(s + '_thumb_proximal')].translation; idx = data.oMf[model.getFrameId(s + '_index_proximal')].translation; pk = data.oMf[model.getFrameId(s + '_pinky_proximal')].translation
        ttip = data.oMf[model.getFrameId(s + '_thumb_force_sensor_3')].translation
        dbase = ddata.oMf[dmodel.getFrameId(s + '_base_link')]; dj = {k: ddata.oMf[dmodel.getFrameId(s + '_' + v)].translation for k, v in DONOR.items()}
        ej = {'index_proximal': idx, 'pinky_proximal': pk, 'thumb_proximal': thumb}
        n = R[:, 0]; f = R[:, 2]; across = idx - pk; hand_sign = float(np.dot(np.cross(f, across), n))   # right hand < 0, left hand > 0 (palm facing you, fingers up: right index is on your right)
        toward_midline = float(n[1]) * (1 if s == 'right' else -1); thumb_index_side = float(np.dot(ttip - mid, across))
        chir[s] = {'palm_normal_pelvis': n.round(4).tolist(), 'palm_normal_towards_midline': toward_midline > 0.9, 'fingers_direction_pelvis': f.round(4).tolist(), 'index_minus_pinky_pelvis': across.round(4).tolist(),
                   'handedness_triple_product': round(hand_sign, 6), 'handedness_ok': (hand_sign < 0) if s == 'right' else (hand_sign > 0), 'thumb_tip_on_index_side': thumb_index_side > 0,
                   'donor_palm_normal_pelvis': dbase.rotation[:, 1].round(4).tolist(), 'palm_normal_vs_donor_deg': round(math.degrees(math.acos(min(1.0, float(np.dot(n, dbase.rotation[:, 1]))))), 4),
                   'joint_datum_vs_donor_mm': {k: round(float(np.linalg.norm(ej[k] - dj[k])) * 1000, 2) for k in DONOR}, 'e2_thumb_base_pelvis': thumb.round(4).tolist(), 'donor_thumb_base_pelvis': dj['thumb_proximal'].round(4).tolist()}
    mirror = {}
    for f in ('index_force_sensor_3', 'thumb_force_sensor_3', 'pinky_force_sensor_3', 'palm_force_sensor', 'thumb_proximal'):
        r = data.oMf[model.getFrameId('right_' + f)].translation; l = data.oMf[model.getFrameId('left_' + f)].translation
        mirror[f] = round(float(np.linalg.norm(l - r * np.array([1, -1, 1]))) * 1000, 3)
    ok = all(chir[s]['palm_normal_towards_midline'] and chir[s]['handedness_ok'] and chir[s]['thumb_tip_on_index_side'] and chir[s]['palm_normal_vs_donor_deg'] < 0.01 for s in SIDES) and max(mirror.values()) < 0.5 and all(chir[s]['joint_datum_vs_donor_mm'][k] < 0.5 for s in SIDES for k in ('index_proximal', 'pinky_proximal'))
    check('chirality_and_mount', ok, {'per_side': chir, 'left_vs_mirrored_right_mm_at_q0': mirror, 'rule': 'handedness triple product (fingers x (index-pinky)) . palm_normal < 0 for a right hand, > 0 for a left; thumb tip on the index side; palm normal towards the midline and equal to the donor palm normal; index/pinky proximal joints on the donor datum (< 0.5 mm); left = y-mirror of right at q=0; the thumb base is expected to differ from the donor (E2 geometry)'})

    # ---- inertials, masses
    inert = {}
    for s in SIDES:
        base = data.oMf[model.getFrameId(s + '_base')]; inert[s] = {}
        for ln in hand_links(xml, s):
            jid = model.frames[model.getFrameId(ln)].parentJoint
            if model.frames[model.getFrameId(ln)].type != pin.FrameType.BODY: continue
            # link inertia from the URDF (pinocchio lumps fixed links into the parent joint; read the XML directly)
            lx = xml.find("link[@name='%s']" % ln); ie = lx.find('inertial')
            if ie is None: continue
            o = ie.find('origin'); xyz = np.array([float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]); rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            Ro = pin.rpy.rpyToMatrix(*rpy); I = ie.find('inertia'); g = lambda k: float(I.get(k)); Im = np.array([[g('ixx'), g('ixy'), g('ixz')], [g('ixy'), g('iyy'), g('iyz')], [g('ixz'), g('iyz'), g('izz')]])
            oMl = data.oMf[model.getFrameId(ln)]; bMl = base.inverse() * oMl
            inert[s][ln.replace(s + '_', '', 1)] = {'mass': float(ie.find('mass').get('value')), 'com_base': (bMl.rotation @ xyz + bMl.translation), 'I_base': bMl.rotation @ Ro @ Im @ Ro.T @ bMl.rotation.T}
    M = np.diag([1.0, -1.0, 1.0]); worst = {'main': {'com_mm': 0.0, 'inertia_rel': 0.0, 'mass_rel': 0.0}, 'sensor': {'com_mm': 0.0, 'inertia_rel': 0.0, 'mass_rel': 0.0}}; per = {}
    for ln, r in inert['right'].items():
        l = inert['left'].get(ln)
        if l is None: continue
        kind = 'sensor' if ('force_sensor' in ln or ln.endswith('tcp')) else 'main'
        dc = float(np.linalg.norm(l['com_base'] - M @ r['com_base'])) * 1000; dI = float(np.abs(l['I_base'] - M @ r['I_base'] @ M).max() / max(np.abs(r['I_base']).max(), 1e-15)); dm = abs(l['mass'] - r['mass']) / r['mass']
        per[ln] = {'kind': kind, 'com_mm': round(dc, 4), 'inertia_rel': round(dI, 6), 'mass_rel': round(dm, 7)}; worst[kind] = {k: max(worst[kind][k], v) for k, v in (('com_mm', dc), ('inertia_rel', dI), ('mass_rel', dm))}
    ok = worst['main']['com_mm'] < 0.01 and worst['main']['inertia_rel'] < 1e-3 and worst['main']['mass_rel'] < 1e-5 and worst['sensor']['com_mm'] < 1.0 and worst['sensor']['inertia_rel'] < 0.01 and worst['sensor']['mass_rel'] < 1e-3
    check('left_inertials_mirror_right', ok, {'worst': worst, 'per_link': per, 'tolerance': 'main links (mirrored right inertials; after the r4 fold they also carry the exporter-mirrored sensor plates, which agree to <= 0.6 mm): 0.01 mm / 0.1 % / 1e-5 mass; exporter-mirrored force-sensor links when unfolded: 1 mm / 1 % / 0.1 % mass'})
    pm_l, pm_r = inert['left']['pinky_intermediate']['mass'], inert['right']['pinky_intermediate']['mass']; folded = 'representation_r4' in manifest
    check('left_pinky_intermediate_mass', abs(pm_l - pm_r) < 1e-6 and (folded or abs(pm_r - 0.01166) < 1e-6) and pm_r < 0.0125, {'left_kg': pm_l, 'right_kg': pm_r, 'public_left_file_kg': manifest['public_urdf_corrections']['left_pinky_intermediate_mass_kg']['public_file'], 'note': 'r4: link mass includes the folded sensor plates (0.01166 + pads)' if folded else 'bare link mass'})
    masses = {s: {ln: v['mass'] for ln, v in inert[s].items()} for s in SIDES}; totals = {s: round(sum(masses[s].values()), 6) for s in SIDES}
    check('masses_finite_positive', all(math.isfinite(m) and m > 0 for s in SIDES for m in masses[s].values()), {'hand_total_kg': totals, 'min_link_kg': {s: min(masses[s].values()) for s in SIDES}, 'links': {s: len(masses[s]) for s in SIDES}})
    mpol = manifest.get('mass_policy'); check('mass_policy_declared', bool(mpol) and mpol.get('primary') is not None, {'policy': mpol, 'hand_total_kg': totals})
    check('total_mass_vs_donor', True, {'e2_merged_kg': round(pin.computeTotalMass(model), 6), 'donor_merged_kg': round(pin.computeTotalMass(dmodel), 6), 'delta_kg': round(pin.computeTotalMass(model) - pin.computeTotalMass(dmodel), 6)}, level='INFO')

    # ---- meshes: scale attributes, bounding boxes, stubs
    scales = set(); boxes = {}
    for go in geom.geometryObjects:
        if not go.name.startswith(('right_', 'left_')) or any(k in go.name for k in BODY_KEYS): continue
        scales.add(tuple(np.round(go.meshScale, 9))); g = go.geometry
        if isinstance(g, hppfcl.BVHModelBase):
            pts = np.array([g.vertices(i) for i in range(g.num_vertices)]) if hasattr(g, 'vertices') else None
            if pts is not None and len(pts): boxes[go.name] = (pts.max(0) - pts.min(0)).round(4).tolist()
    ext = {s: np.max([np.array(b) for n, b in boxes.items() if n.startswith(s + '_')], axis=0).round(4).tolist() for s in SIDES if any(n.startswith(s + '_') for n in boxes)}
    scale_ok = scales == {(1.0, 1.0, 1.0)}; extent_ok = all(0.05 < max(v) < 0.30 for v in ext.values())
    for s in SIDES:
        mm = [s + '_palm_1', s + '_hand_base_link']
    check('mesh_scale_metres', scale_ok and extent_ok, {'scale_attributes': sorted(scales), 'largest_link_extent_m': ext, 'palm_box_m': {s: boxes.get(s + '_palm_1_0', boxes.get(s + '_palm_1')) for s in SIDES}, 'n_meshes': len(boxes)})
    stubs = [k for k, v in manifest['source']['meshes'].items() if not v.get('lfs_pointer_verified')]
    check('no_lfs_stubs', not stubs and len(manifest['source']['meshes']) == 64, {'meshes': len(manifest['source']['meshes']), 'unverified': stubs, 'method': manifest['source']['lfs']})

    # ---- tactile frames
    for s in SIDES:
        fr = [f.name for f in model.frames if f.name.startswith(s + '_') and 'force_sensor' in f.name and f.type == pin.FrameType.BODY]
        check('tactile_frames_17', len(fr) == 17, {'frames': fr}, s)

    # ---- self-collision preflight (intersection = coal distance <= 0; depth measured by mesh_penetration)
    adj = adjacent_pairs(xml); grp = fixed_groups(xml); names = [go.name for go in geom.geometryObjects]; link_of = {go.name: model.frames[go.parentFrame].name for go in geom.geometryObjects}
    def pairs_for(side, a_links=None, b_links=None):
        geom.removeAllCollisionPairs(); hl = set(hand_links(xml, side))
        ids = [i for i, go in enumerate(geom.geometryObjects) if link_of[go.name] in hl]
        for i, j in itertools.combinations(ids, 2):
            a, b = link_of[names[i]], link_of[names[j]]
            if a == b or frozenset((a, b)) in adj or grp(a) == grp(b): continue
            if a_links is not None and not ((a in a_links and b in b_links) or (b in a_links and a in b_links)): continue
            geom.addCollisionPair(pin.CollisionPair(i, j))
        return pin.GeometryData(geom)
    def scan(gdata, q, with_depth):
        pin.computeDistances(model, data, geom, gdata, q); res = []
        for k, cp in enumerate(geom.collisionPairs):
            d = gdata.distanceResults[k].min_distance; a, b = link_of[names[cp.first]], link_of[names[cp.second]]
            if d <= 1e-9 and with_depth:
                pen = mp.penetration(geom.geometryObjects[cp.first].geometry, gdata.oMg[cp.first], geom.geometryObjects[cp.second].geometry, gdata.oMg[cp.second], max_points=1500)
                res.append({'pair': [a, b], 'distance_mm': 0.0, 'depth_mm': round(pen['depth_m'] * 1000, 2), 'inside_vertices': [pen['a_in_b_count'], pen['b_in_a_count']]})
            else:
                res.append({'pair': [a, b], 'distance_mm': round(d * 1000, 2)})
        res.sort(key=lambda r: (r['distance_mm'], -r.get('depth_mm', 0))); return res
    coll = {}
    for s in SIDES:
        gdata = pairs_for(s); coll[s] = {}
        for pname, q in qs.items():
            res = scan(gdata, q, True); inter = [r for r in res if r['distance_mm'] <= 0]
            coll[s][pname] = {'pairs': len(res), 'intersecting': inter, 'max_depth_mm': max([r.get('depth_mm', 0.0) for r in inter], default=0.0), 'closest_5': res[:5] if not inter else [r for r in res if r['distance_mm'] > 0][:5]}
        open_ok = not coll[s]['open']['intersecting']
        check('self_collision_preflight_open', open_ok, coll[s]['open'], s)
        check('self_collision_closed_and_opposed', True, {k: v for k, v in coll[s].items() if k != 'open'} | {'note': 'intersections at the coupled full-closure poses are the mechanical contacts of a fist with the opposed thumb (the real hand stalls on contact); the twin must not command them: see self_collision_envelope'}, s, level='INFO')
    # ---- self-collision envelope: thumb (opposed) vs closing fingers, and fingers (closed) vs the thumb pitch
    env = {}
    thumb_links = lambda s: {l for l in hand_links(xml, s) if l.startswith(s + '_thumb')}; finger_links = lambda s: {l for l in hand_links(xml, s) if any(l.startswith(s + '_' + f) for f in ('index', 'middle', 'ring', 'pinky'))}
    for s in SIDES:
        gdata = pairs_for(s, thumb_links(s), finger_links(s)); e = {}
        first = None
        for fing in np.linspace(0, PRIOR['finger_proximal_rad'], 30):
            q = hand_config(model, J[s], s, {'thumb_proximal_yaw_joint': PRIOR['thumb_rotation_rad'], 'thumb_proximal_pitch_joint': PRIOR['thumb_bend_rad']} | {n: float(fing) for n in ACTIVE[2:]})
            res = scan(gdata, q, False)
            if res and res[0]['distance_mm'] <= 0: first = (round(float(fing), 4), res[0]['pair']); break
        e['thumb_fully_opposed_first_contact_at_finger_closure_rad'] = first
        first = None
        for pitch in np.linspace(0, PRIOR['thumb_bend_rad'], 20):
            q = hand_config(model, J[s], s, {'thumb_proximal_yaw_joint': PRIOR['thumb_rotation_rad'], 'thumb_proximal_pitch_joint': float(pitch)} | {n: PRIOR['finger_proximal_rad'] for n in ACTIVE[2:]})
            res = scan(gdata, q, False)
            if res and res[0]['distance_mm'] <= 0: first = (round(float(pitch), 4), res[0]['pair']); break
        e['fingers_closed_thumb_yaw_max_first_contact_at_pitch_rad'] = first
        first = None
        for yaw in np.linspace(0, PRIOR['thumb_rotation_rad'], 30):
            q = hand_config(model, J[s], s, {'thumb_proximal_yaw_joint': float(yaw), 'thumb_proximal_pitch_joint': PRIOR['thumb_bend_rad']} | {n: PRIOR['finger_proximal_rad'] for n in ACTIVE[2:]})
            res = scan(gdata, q, False)
            if res and res[0]['distance_mm'] <= 0: first = (round(float(yaw), 4), res[0]['pair']); break
        e['fingers_closed_thumb_bent_first_contact_at_yaw_rad'] = first
        # open fingers, thumb sweep: does the opposed thumb touch open fingers anywhere?
        mind = 1e9; at = None
        for yaw in np.linspace(0, PRIOR['thumb_rotation_rad'], 9):
            for pitch in np.linspace(0, PRIOR['thumb_bend_rad'], 5):
                q = hand_config(model, J[s], s, {'thumb_proximal_yaw_joint': float(yaw), 'thumb_proximal_pitch_joint': float(pitch)}); res = scan(gdata, q, False)
                if res and res[0]['distance_mm'] < mind: mind, at = res[0]['distance_mm'], {'yaw': round(float(yaw), 4), 'pitch': round(float(pitch), 4), 'pair': res[0]['pair']}
        e['fingers_open_thumb_sweep_min_distance_mm'] = {'min_mm': mind, 'at': at}
        env[s] = e
        check('self_collision_envelope', True, e | {'note': 'first thumb-vs-finger intersection along three one-parameter sweeps of the coupled hand; the twin controller must keep commands inside this envelope or accept the contact as a grasp closure event'}, s, level='INFO')
    # ---- palm / thumb cavity sweep (thumb links vs palm/base over yaw x pitch x fingers {open, closed}), depth for intersections
    cav = {}
    for s in SIDES:
        gdata = pairs_for(s, thumb_links(s), {s + '_palm_1', s + '_palm_2', s + '_hand_base_link', s + '_palm_force_sensor', s + '_base'}); grid = []; worst = (0.0, None)
        for yaw in np.linspace(0, PRIOR['thumb_rotation_rad'], 9):
            for pitch in np.linspace(0, PRIOR['thumb_bend_rad'], 5):
                for fing in (0.0, PRIOR['finger_proximal_rad']):
                    q = hand_config(model, J[s], s, {'thumb_proximal_yaw_joint': float(yaw), 'thumb_proximal_pitch_joint': float(pitch)} | {n: fing for n in ACTIVE[2:]}); res = scan(gdata, q, True)
                    inter = [r for r in res if r['distance_mm'] <= 0]; depth = max([r.get('depth_mm', 0.0) for r in inter], default=0.0)
                    grid.append({'yaw': round(float(yaw), 4), 'pitch': round(float(pitch), 4), 'fingers': fing, 'min_distance_mm': res[0]['distance_mm'] if res else None, 'depth_mm': depth, 'pair': res[0]['pair'] if res else None})
                    if depth > worst[0]: worst = (depth, grid[-1])
        cav[s] = {'worst_depth_mm': worst[0], 'worst_at': worst[1], 'intersecting_samples': [g for g in grid if g['depth_mm'] > 0 or (g['min_distance_mm'] is not None and g['min_distance_mm'] <= 0)], 'samples': len(grid), 'grid': grid}
        check('palm_thumb_cavity_sweep', worst[0] < 1.5, {k: v for k, v in cav[s].items() if k != 'grid'} | {'rule': 'thumb links vs palm/base over yaw [0,1.658] x pitch [0,0.62] x fingers {open, closed}: PASS when no intersection deeper than 1.5 mm (a sub-millimetre skin overlap at the yaw-0 full-bend limit is recorded, not a cavity fill; the exact meshes overlap 0.68 mm there, the r4 slab pieces add <= 0.4 mm of conservatism)'}, s)
    # ---- left meshes vs mirrored right meshes (surface deviation, enclosed volume, triangle counts)
    pin.forwardKinematics(model, data, q0); pin.updateFramePlacements(model, data); gdata = pin.GeometryData(geom); pin.updateGeometryPlacements(model, data, geom, gdata)
    bR = data.oMf[model.getFrameId('right_base')]; bL = data.oMf[model.getFrameId('left_base')]; M = np.diag([1.0, -1.0, 1.0]); mm = {}
    def vol(W, T):
        a, b, c = W[T[:, 0]], W[T[:, 1]], W[T[:, 2]]; return abs(float(np.einsum('ij,ij->i', a, np.cross(b, c)).sum() / 6))
    def mesh_file(go):
        return Path(go.meshPath).name.lower() if getattr(go, 'meshPath', '') else go.name
    for i, go in enumerate(geom.geometryObjects):
        ln = link_of[go.name]
        if not ln.startswith('right_') or any(k in ln for k in BODY_KEYS) or 'palm_slabs' in str(getattr(go, 'meshPath', '')): continue
        lname = ln.replace('right_', 'left_', 1); key = mesh_file(go).replace('right_', 'left_', 1)
        cand = [k for k, oo in enumerate(geom.geometryObjects) if link_of[oo.name] == lname and mesh_file(oo) == key]
        if not cand:
            cand = [k for k, oo in enumerate(geom.geometryObjects) if link_of[oo.name] == lname and 'palm_slabs' not in str(getattr(oo, 'meshPath', ''))]
        if not cand: continue
        j = cand[0]
        VR, TR = mp.mesh_arrays(go.geometry); VL, TL = mp.mesh_arrays(geom.geometryObjects[j].geometry)
        WR = VR @ np.asarray(gdata.oMg[i].rotation).T + np.asarray(gdata.oMg[i].translation); WL = VL @ np.asarray(gdata.oMg[j].rotation).T + np.asarray(gdata.oMg[j].translation)
        BR = (WR - bR.translation) @ bR.rotation; BL = (WL - bL.translation) @ bL.rotation
        P = (BR @ M)[np.linspace(0, len(BR) - 1, min(len(BR), 1200)).astype(int)] @ bL.rotation.T + bL.translation
        dist = mp.surface_distance(P, geom.geometryObjects[j].geometry, gdata.oMg[j])
        P2 = (BL @ M)[np.linspace(0, len(BL) - 1, min(len(BL), 1200)).astype(int)] @ bR.rotation.T + bR.translation
        dist2 = mp.surface_distance(P2, go.geometry, gdata.oMg[i])
        mm[mesh_file(go)] = {'link': ln.replace('right_', '', 1), 'triangles': [go.geometry.num_tris, geom.geometryObjects[j].geometry.num_tris], 'enclosed_volume_cm3': [round(vol(BR, TR) * 1e6, 2), round(vol(BL, TL) * 1e6, 2)],
                                            'bbox_shift_mm': round(float(np.abs(np.c_[(BR @ M).min(0), (BR @ M).max(0)] - np.c_[BL.min(0), BL.max(0)]).max()) * 1000, 3),
                                            'right_mirrored_to_left_surface_mm': {'p95': round(float(np.percentile(dist, 95)) * 1000, 3), 'max': round(float(dist.max()) * 1000, 3)},
                                            'left_mirrored_to_right_surface_mm': {'p95': round(float(np.percentile(dist2, 95)) * 1000, 3), 'max': round(float(dist2.max()) * 1000, 3)}}
    env_ok = all(v['bbox_shift_mm'] < 1.0 for v in mm.values())
    check('left_meshes_mirror_right_envelope', env_ok, {'per_link': mm, 'rule': 'every left collision mesh occupies the mirrored right bounding box within 1 mm; interior/triangulation differences are recorded (the public left STLs are separate exports, not mirror copies)'})
    summary = {'PASS': sum(c['result'] == 'PASS' for c in checks), 'FAIL': sum(c['result'] == 'FAIL' for c in checks), 'INFO': sum(c['result'] == 'INFO' for c in checks)}
    out = {'asset_id': manifest['asset_id'], 'label': manifest['label'], 'merged_urdf_sha256': manifest['merged_urdf']['sha256'], 'summary': summary, 'verdict': 'STATIC_AUDIT_PASS' if summary['FAIL'] == 0 else 'STATIC_AUDIT_FAIL', 'checks': checks, 'cavity_grids': {s: cav[s]['grid'] for s in SIDES}, 'envelope': env}
    Path(a.out).write_text(json.dumps(out, indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)) + '\n')
    print(json.dumps({'verdict': out['verdict'], 'summary': summary, 'fails': [c['check'] + ('/' + c['side'] if c['side'] else '') for c in checks if c['result'] == 'FAIL']}, indent=1))


if __name__ == '__main__':
    main()
