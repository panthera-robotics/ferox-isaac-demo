"""Machine-readable load-property record for one wrist: hand, wrist adapter and held tool as separate components with
explicit inclusion rules. Unknown components stay null; a null is never composed as zero, and a composed total is
only reported when every included component is known.

All components are expressed in ONE frame (the side's ``<side>_wrist_yaw_link``): mass in kg, COM in m, inertia about
the component's own COM in kg·m² (3x3, symmetric, positive definite).
"""
from __future__ import annotations

import hashlib
import json
import math

import numpy as np

SCHEMA = 'hand_load_record_v1'
PROVENANCE_CLASSES = ('URDF', 'MEASURED', 'NOMINAL', 'declared', 'assumed')


def _component(c, name, frame):
    if c is None:
        return None
    out = {'mass_kg': None, 'com_m': None, 'inertia_about_com_kg_m2': None, 'provenance': None, 'frame': frame, 'configuration': None, 'note': None}
    out.update({k: c[k] for k in c if k in out})
    if out['provenance'] not in PROVENANCE_CLASSES:
        raise ValueError('%s.provenance must be one of %s' % (name, PROVENANCE_CLASSES))
    return out


def _compose(parts):
    """Rigid composition of known components; returns None when any part lacks mass or COM; inertia only when all have it."""
    if any(p is None or p['mass_kg'] is None or p['com_m'] is None for p in parts):
        return None
    total = float(sum(p['mass_kg'] for p in parts))
    com = sum(np.asarray(p['com_m'], float) * p['mass_kg'] for p in parts) / total
    inertia = None
    if all(p['inertia_about_com_kg_m2'] is not None for p in parts):
        I = np.zeros((3, 3))
        for p in parts:
            d = np.asarray(p['com_m'], float) - com
            I += np.asarray(p['inertia_about_com_kg_m2'], float) + p['mass_kg'] * ((d @ d) * np.eye(3) - np.outer(d, d))
        inertia = I.round(12).tolist()
    return {'mass_kg': round(total, 9), 'com_m': com.round(9).tolist(), 'inertia_about_com_kg_m2': inertia, 'components': [p['name'] for p in parts]}


def load_record(side, *, hand, wrist_adapter, tool, asset_hashes=None, notes=None):
    if side not in ('left', 'right'):
        raise ValueError('side must be left or right')
    frame = side + '_wrist_yaw_link'
    comps = {'hand': _component(hand, 'hand', frame), 'wrist_adapter': _component(wrist_adapter, 'wrist_adapter', frame), 'tool': _component(tool, 'tool', frame)}
    named = {k: (dict(v, name=k) if v else None) for k, v in comps.items()}
    rec = {
        'schema': SCHEMA, 'side': side, 'frame': frame,
        'inclusion_rules': {
            'hand_only': 'hand subtree below <side>_base_link (URDF inertials); excludes the G1 wrist links, the adapter/flange hardware and any tool',
            'hand_and_adapter': 'hand_only + wrist adapter/flange hardware between wrist_yaw_link and base_link (unknown until measured: stays null)',
            'hand_and_tool': 'hand_only + declared held tool (mass/COM in the wrist frame; a null tool inertia keeps the total inertia null)',
            'hand_adapter_and_tool': 'all three; null unless every component is known',
        },
        'components': comps,
        'totals': {'hand_only': _compose([named['hand']]), 'hand_and_adapter': _compose([named['hand'], named['wrist_adapter']]),
                   'hand_and_tool': _compose([named['hand'], named['tool']]), 'hand_adapter_and_tool': _compose([named['hand'], named['wrist_adapter'], named['tool']])},
        'asset_hashes': asset_hashes or {}, 'notes': notes or [],
        'rule': 'a null component cannot enter a qualification path as zero; consumers must refuse a total that is null',
    }
    rec['record_sha256'] = hashlib.sha256(json.dumps({k: v for k, v in rec.items() if k != 'record_sha256'}, sort_keys=True).encode()).hexdigest()
    return rec


def validate_load_record(rec):
    if rec.get('schema') != SCHEMA:
        raise ValueError('unexpected schema')
    frame = rec['frame']
    for name, c in rec['components'].items():
        if c is None:
            continue
        if c.get('frame') != frame:
            raise ValueError('%s is expressed in %s, record frame is %s' % (name, c.get('frame'), frame))
        m = c.get('mass_kg')
        if m is not None:
            if isinstance(m, bool) or not isinstance(m, (int, float)) or not math.isfinite(m) or m <= 0:
                raise ValueError('%s.mass_kg must be finite and > 0 (unknown stays null, never 0)' % name)
        com = c.get('com_m')
        if com is not None:
            a = np.asarray(com, float)
            if a.shape != (3,) or not np.isfinite(a).all():
                raise ValueError('%s.com_m must be three finite numbers' % name)
        I = c.get('inertia_about_com_kg_m2')
        if I is not None:
            a = np.asarray(I, float)
            if a.shape != (3, 3) or not np.isfinite(a).all() or not np.allclose(a, a.T, atol=1e-12) or np.linalg.eigvalsh(a).min() <= 0:
                raise ValueError('%s inertia must be a finite symmetric positive-definite 3x3' % name)
        if c.get('provenance') == 'assumed' and m is not None:
            raise ValueError('%s: an assumed mass is not admissible; leave it null or measure it' % name)
    for name, t in rec['totals'].items():
        if t is not None and (t['mass_kg'] is None or t['com_m'] is None):
            raise ValueError('total %s must be null or complete' % name)
    return True
