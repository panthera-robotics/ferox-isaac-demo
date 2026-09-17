"""Mimic-coupling algebra for a hand URDF: transitive composition, range-versus-limit margins and equivalence
between two constraint layouts of the same mechanism.

A mimic joint declares q_child = multiplier * q_parent + offset. When the parent is itself a mimic joint the
relation composes: with q_b = a*q_a + b and q_c = c*q_b + d the equivalent single-reference relation is
q_c = (c*a)*q_a + (c*b + d). Algebraic equivalence says nothing about the dynamics of a different PhysX
constraint layout; that is a runtime question and stays with the runtime owner.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

REVOLUTE = ('revolute', 'continuous', 'prismatic')


def _finite(v, what):
    v = float(v)
    if not math.isfinite(v):
        raise ValueError('%s must be finite' % what)
    return v


def read_joints(urdf_path):
    """{joint_name: {type, parent, child, axis, limit, effort, velocity, mimic}} for every joint in the URDF."""
    root = ET.parse(Path(urdf_path)).getroot()
    out = {}
    for j in root.findall('joint'):
        name = j.get('name')
        if name in out:
            raise ValueError('duplicate joint name ' + name)
        lim = j.find('limit'); mim = j.find('mimic'); ax = j.find('axis'); o = j.find('origin')
        out[name] = {
            'type': j.get('type'), 'parent': j.find('parent').get('link'), 'child': j.find('child').get('link'),
            'xyz': [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()],
            'rpy': [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()],
            'axis': [float(v) for v in ax.get('xyz').split()] if ax is not None else None,
            'limit': [_finite(lim.get('lower'), name + '.lower'), _finite(lim.get('upper'), name + '.upper')] if lim is not None else None,
            'effort': float(lim.get('effort')) if lim is not None and lim.get('effort') else None,
            'velocity': float(lim.get('velocity')) if lim is not None and lim.get('velocity') else None,
            'mimic': {'joint': mim.get('joint'), 'multiplier': _finite(mim.get('multiplier', 1), name + '.multiplier'),
                      'offset': _finite(mim.get('offset', 0), name + '.offset')} if mim is not None else None,
        }
    for name, j in out.items():
        if j['mimic'] and j['mimic']['joint'] not in out:
            raise ValueError('%s mimics unknown joint %s' % (name, j['mimic']['joint']))
    return out


def compose(joints, name):
    """Flatten the mimic chain of `name` to its independent driver: returns (driver, A, B, chain) with q = A*q_driver + B.
    `chain` lists the hops from `name` down to the driver. Raises on cycles."""
    A, B = 1.0, 0.0
    chain = []
    seen = {name}
    cur = name
    while joints[cur]['mimic']:
        m = joints[cur]['mimic']
        # q_cur = m.mult * q_parent + m.off, and q_name = A*q_cur + B so far
        A, B = A * m['multiplier'], A * m['offset'] + B
        chain.append((cur, m['joint'], m['multiplier'], m['offset']))
        cur = m['joint']
        if cur in seen:
            raise ValueError('mimic cycle through ' + cur)
        seen.add(cur)
    return cur, A, B, chain


def coupling_table(joints, prefix=None):
    """Composed coupling for every mimic joint (optionally only names starting with prefix)."""
    table = {}
    for name, j in joints.items():
        if not j['mimic'] or (prefix and not name.startswith(prefix)):
            continue
        driver, A, B, chain = compose(joints, name)
        lo, hi = joints[driver]['limit']
        implied = sorted((A * lo + B, A * hi + B))
        own = j['limit']
        table[name] = {
            'declared': dict(j['mimic']), 'driver': driver, 'composed_multiplier': A, 'composed_offset': B,
            'chain_depth': len(chain), 'chain': [{'child': c, 'parent': p, 'multiplier': m, 'offset': o} for c, p, m, o in chain],
            'driver_limit_rad': [lo, hi], 'implied_range_rad': implied, 'own_limit_rad': own,
            # margin > 0: the coupled joint's own limit lies strictly outside the range the coupling can command
            'margin_low_rad': implied[0] - own[0], 'margin_high_rad': own[1] - implied[1],
            'axis': j['axis'],
        }
    return table


def range_limit_findings(table, boundary_tol=1e-9):
    """Classify each coupled joint: EXCEEDS (coupling can command beyond its own limit), BOUNDARY (coupling range
    endpoint coincides with its own limit: mimic and limit constraints are simultaneously active there) or INSIDE."""
    out = {}
    for name, row in table.items():
        flags = []
        for end, margin in (('low', row['margin_low_rad']), ('high', row['margin_high_rad'])):
            if margin < -boundary_tol:
                flags.append('EXCEEDS_%s_LIMIT_BY_%.4f_rad' % (end.upper(), -margin))
            elif abs(margin) <= boundary_tol:
                flags.append('BOUNDARY_COINCIDENT_%s' % end.upper())
        out[name] = {'status': 'EXCEEDS' if any(f.startswith('EXCEEDS') for f in flags) else 'BOUNDARY' if flags else 'INSIDE', 'flags': flags,
                     'margin_low_rad': row['margin_low_rad'], 'margin_high_rad': row['margin_high_rad']}
    return out


def equivalent_layouts(joints_a, joints_b, prefix=None, samples=25, tol=1e-9):
    """Two URDF joint sets describe the same coupling iff every mimic joint composes to the same driver with the
    same (A, B) and the same own limits, and the sampled joint values agree over the driver's range."""
    ta, tb = coupling_table(joints_a, prefix), coupling_table(joints_b, prefix)
    if set(ta) != set(tb):
        return False, {'reason': 'different mimic joint sets', 'only_a': sorted(set(ta) - set(tb)), 'only_b': sorted(set(tb) - set(ta))}
    diffs = {}
    for name in ta:
        a, b = ta[name], tb[name]
        if a['driver'] != b['driver']:
            diffs[name] = 'different driver %s vs %s' % (a['driver'], b['driver']); continue
        if a['own_limit_rad'] != b['own_limit_rad'] or a['axis'] != b['axis']:
            diffs[name] = 'different own limit or axis'; continue
        lo, hi = a['driver_limit_rad']
        worst = 0.0
        for k in range(samples):
            q = lo + (hi - lo) * k / (samples - 1)
            worst = max(worst, abs((a['composed_multiplier'] * q + a['composed_offset']) - (b['composed_multiplier'] * q + b['composed_offset'])))
        if worst > tol:
            diffs[name] = 'sampled disagreement %.3e rad' % worst
    return (not diffs), {'differences': diffs, 'table_a': ta, 'table_b': tb}
