"""Spawn-time hand/scene clearance (sprint L, hand lane): geometry only, no simulator.

The probe spawns the robot at the first-row body targets (else the controller home) with the hands at the URDF open pose and
ramps them to the first row over the lead-in; a fixture that reaches into the hands' spawn envelope makes the solver explode at
step 0 (sL-reference-fixture-ep0-state-replay-01/-02: a wide table at pelvis +0.10 / +0.05 inside the home-pose finger pads).
This module computes the hand links' axis-aligned boxes at that pose from the URDF forward kinematics (frame origins padded
by a link radius) and intersects them with the scene's table/object/support boxes (pelvis frame). The packager refuses a
package whose fixture overlaps either hand pose it will spawn. Pure Python + xml; no mesh reading, so the envelope is the
padded skeleton of each hand — a fixture that clears it by less than the pad is reported as a margin, not as a pass.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

DEFAULT_PAD_M = 0.02      # link radius stand-in around every hand frame origin (finger phalanx ~0.012 m, palm ~0.02 m)


class ClearanceError(ValueError):
    pass


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]]


def _axis_angle(axis, angle):
    n = math.sqrt(sum(a * a for a in axis))
    if n < 1e-12:
        raise ClearanceError('zero joint axis')
    x, y, z = (a / n for a in axis); c, s = math.cos(angle), math.sin(angle); C = 1.0 - c
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s], [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _apply(R, p, v):
    return [p[i] + sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def read_urdf_joints(urdf_path):
    root = ET.parse(Path(urdf_path)).getroot()
    joints = {}
    for j in root.findall('joint'):
        o = j.find('origin'); ax = j.find('axis'); mim = j.find('mimic')
        joints[j.get('name')] = {'type': j.get('type'), 'parent': j.find('parent').get('link'), 'child': j.find('child').get('link'),
                                 'xyz': [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()],
                                 'rpy': [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()],
                                 'axis': [float(v) for v in ax.get('xyz').split()] if ax is not None else [1.0, 0.0, 0.0],
                                 'mimic': None if mim is None else {'joint': mim.get('joint'), 'multiplier': float(mim.get('multiplier', 1.0)), 'offset': float(mim.get('offset', 0.0))}}
    return joints


def link_frames(urdf_path, q_by_name):
    """{link: (R, p)} in the root link's frame for the whole robot at joint values q (unnamed joints 0; mimics composed)."""
    joints = read_urdf_joints(urdf_path)
    children = set(j['child'] for j in joints.values()); parents = set(j['parent'] for j in joints.values())
    roots = sorted(parents - children)
    if len(roots) != 1:
        raise ClearanceError('URDF must have exactly one root link, found %s' % roots)
    values = {}
    for name, j in joints.items():
        if j['type'] in ('revolute', 'continuous'):
            v = q_by_name.get(name)
            if v is not None and not (isinstance(v, (int, float)) and math.isfinite(v)):
                raise ClearanceError('non-finite joint value for %s' % name)
            values[name] = float(v) if v is not None else None
    for name, j in joints.items():
        if j['mimic'] and values.get(name) is None:
            drv = values.get(j['mimic']['joint']) or 0.0
            values[name] = j['mimic']['multiplier'] * drv + j['mimic']['offset']
    frames = {roots[0]: ([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]], [0.0, 0.0, 0.0])}
    pending = dict(joints)
    while pending:
        progressed = False
        for name in list(pending):
            j = pending[name]
            if j['parent'] in frames:
                Rp, pp = frames[j['parent']]
                R = _mul(Rp, _rpy(*j['rpy'])); p = _apply(Rp, pp, j['xyz'])
                if j['type'] in ('revolute', 'continuous'):
                    R = _mul(R, _axis_angle(j['axis'], values.get(name) or 0.0))
                elif j['type'] != 'fixed':
                    raise ClearanceError('unsupported joint type %s for %s' % (j['type'], name))
                frames[j['child']] = (R, p); del pending[name]; progressed = True
        if not progressed:
            raise ClearanceError('disconnected joints: %s' % sorted(pending))
    return frames


def hand_links(urdf_path, side):
    """Every link at or below <side>_base_link."""
    joints = read_urdf_joints(urdf_path); base = side + '_base_link'
    kids = {}
    for j in joints.values():
        kids.setdefault(j['parent'], []).append(j['child'])
    out, stack = [], [base]
    while stack:
        l = stack.pop(); out.append(l); stack.extend(kids.get(l, []))
    if len(out) < 2:
        raise ClearanceError('no hand subtree below %s' % base)
    return out


def hand_points(urdf_path, side, q_by_name):
    """{link: origin (pelvis frame)} for every hand link of one side at the joint values q."""
    frames = link_frames(urdf_path, q_by_name)
    return {l: frames[l][1] for l in hand_links(urdf_path, side) if l in frames}


def hand_box(urdf_path, side, q_by_name, pad_m=DEFAULT_PAD_M):
    pts = list(hand_points(urdf_path, side, q_by_name).values())
    return {'min_m': [min(p[i] for p in pts) - pad_m for i in range(3)], 'max_m': [max(p[i] for p in pts) + pad_m for i in range(3)], 'frames': len(pts), 'pad_m': pad_m}


def scene_boxes(scene):
    """Solid boxes of a scene JSON (pelvis frame): the table slab, the object's bounding box, any 'supports' entries."""
    boxes = []
    t = scene.get('table')
    if t:
        cx, cy = t['center_xy_m']; sx, sy, sz = t['size_m']; top = t['top_z_pelvis_m']
        boxes.append(('table', [cx - sx / 2, cy - sy / 2, top - sz], [cx + sx / 2, cy + sy / 2, top]))
    o = scene.get('object')
    if o and 'center_pelvis_m' in o:
        c = o['center_pelvis_m']; r = float(o.get('radius_m', 0.0)); h = float(o.get('length_m', o.get('height_m', 0.0)))
        hx = float(o.get('size_m', [2 * r, 2 * r, h])[0]) / 2 if 'size_m' in o else r; hy = float(o.get('size_m', [0, 2 * r, 0])[1]) / 2 if 'size_m' in o else r
        boxes.append(('object', [c[0] - hx, c[1] - hy, c[2] - h / 2], [c[0] + hx, c[1] + hy, c[2] + h / 2]))
    for i, s in enumerate(scene.get('supports') or []):
        cx, cy = s['center_xy_m']; sx, sy, sz = s['size_m']; top = s['top_z_pelvis_m']
        boxes.append((s.get('name', 'support_%d' % i), [cx - sx / 2, cy - sy / 2, top - sz], [cx + sx / 2, cy + sy / 2, top]))
    return boxes


def _point_box_distance(pt, lo, hi):
    """Euclidean distance from a point to an axis-aligned box (0 inside) and the per-axis signed penetration depth."""
    d = [max(lo[i] - pt[i], 0.0, pt[i] - hi[i]) for i in range(3)]
    dist = math.sqrt(sum(v * v for v in d))
    inside = [min(pt[i] - lo[i], hi[i] - pt[i]) for i in range(3)]      # > 0 on every axis == inside
    return dist, inside


def check_spawn_clearance(urdf_path, scene, body_q_rad, hand_poses, pad_m=DEFAULT_PAD_M):
    """hand_poses: {label: {hand joint: rad}} — every hand pose the probe will command before the first row (the URDF open pose
    and the first-row targets). Each hand link is a sphere of radius pad_m at its frame origin; OVERLAP when any such sphere
    reaches into a scene box. Returns the per-link overlaps (depth = pad - distance, m) and the smallest free margin."""
    boxes = scene_boxes(scene)
    report = {'status': 'CLEAR', 'pad_m': pad_m, 'boxes': [{'name': n, 'min_m': lo, 'max_m': hi} for n, lo, hi in boxes], 'hands': {}, 'overlaps': [], 'min_margin_m': None, 'closest': None}
    best = None
    for label, hand_q in hand_poses.items():
        q = dict(body_q_rad); q.update(hand_q or {})
        for side in ('left', 'right'):
            pts = hand_points(urdf_path, side, q); key = '%s:%s' % (label, side)
            report['hands'][key] = {'links': len(pts), 'min_m': [round(min(p[i] for p in pts.values()), 4) for i in range(3)], 'max_m': [round(max(p[i] for p in pts.values()), 4) for i in range(3)]}
            for link, pt in pts.items():
                for name, lo, hi in boxes:
                    dist, inside = _point_box_distance(pt, lo, hi)
                    margin = dist - pad_m
                    if margin < 0:
                        report['overlaps'].append({'hand': key, 'link': link, 'box': name, 'depth_m': round(-margin, 4), 'point_m': [round(v, 4) for v in pt]})
                    if best is None or margin < best[0]:
                        best = (margin, key, link, name)
    if report['overlaps']:
        report['status'] = 'OVERLAP'
    if best is not None:
        report['min_margin_m'] = round(best[0], 4); report['closest'] = {'hand': best[1], 'link': best[2], 'box': best[3]}
    return report
