"""Build the offline source-to-model hand-fidelity delta (HAND_FIDELITY_DELTA.json) for both hands of a donor URDF:
profile, coupling findings, datum-reconciled nominal comparison, command-convention discrepancies and the load-property
record. Runs on the CPU from the asset alone; every value is labelled by evidence class.

    cd tools && python -m hand_fidelity.report --donor-dir /path/to/generated/ftp_donor --out HAND_FIDELITY_DELTA.json
"""
from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
from pathlib import Path

from . import command_semantics as cs
from .coupling import equivalent_layouts, read_joints
from .donor_profile import HandUrdf, sha256_file
from .load_record import load_record, validate_load_record
from .nominal_reference import compare

DONOR_URDF = 'g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf'
THUMBCHAIN2_URDF = 'g1_29dof_rev_1_0_with_inspire_hand_FTP_thumbchain2.urdf'


def mirror_symmetry(right: HandUrdf, left: HandUrdf, tol_m=0.001, tol_rel=0.05):
    """Mirrored links must carry mirrored inertials: each link COM is taken to the hand ROOT frame at the open pose
    (the two root frames are mirror images across x = 0), so local-frame conventions of mirrored joints do not matter."""
    import numpy as np
    fr_r, fr_l = right.frames({}), left.frames({})
    rows = {}
    for rl in right.hand_links:
        ll = rl.replace('right_', 'left_', 1)
        if ll not in left.links:
            continue
        a, b = right.links[rl].find('inertial'), left.links[ll].find('inertial')
        if a is None or b is None:
            continue
        ma, mb = float(a.find('mass').get('value')), float(b.find('mass').get('value'))
        ca = np.array([float(v) for v in (a.find('origin').get('xyz') if a.find('origin') is not None else '0 0 0').split()])
        cb = np.array([float(v) for v in (b.find('origin').get('xyz') if b.find('origin') is not None else '0 0 0').split()])
        ra = fr_r[rl][:3, :3] @ ca + fr_r[rl][:3, 3]; rb = fr_l[ll][:3, :3] @ cb + fr_l[ll][:3, 3]
        dev = float(np.max(np.abs(ra - rb * np.array([-1.0, 1.0, 1.0]))))
        ok = abs(ma - mb) <= tol_rel * max(ma, mb) and dev <= tol_m
        if not ok:
            rows[rl] = {'mass_right_kg': ma, 'mass_left_kg': mb, 'com_right_root_m': ra.round(6).tolist(), 'com_left_root_m': rb.round(6).tolist(), 'max_mirrored_com_deviation_m': round(dev, 6)}
    # kinematic mirror check: joint origins (link frame origins) in the root frames at the open pose
    joints = {}
    for rl in right.hand_links:
        ll = rl.replace('right_', 'left_', 1)
        if ll in fr_l and rl in fr_r:
            dev = float(np.max(np.abs(fr_r[rl][:3, 3] - fr_l[ll][:3, 3] * np.array([-1.0, 1.0, 1.0]))))
            if dev > tol_m:
                joints[rl] = {'origin_right_root_m': fr_r[rl][:3, 3].round(6).tolist(), 'origin_left_root_m': fr_l[ll][:3, 3].round(6).tolist(), 'max_mirrored_deviation_m': round(dev, 6)}
    return {'frame': 'hand root frames (mirror plane x = 0), open pose', 'tolerance_m': tol_m, 'tolerance_mass_rel': tol_rel, 'asymmetric_links': rows,
            'asymmetric_link_frames': joints, 'status': 'PASS' if not rows and not joints else 'FAIL'}


def build(donor_dir, *, with_meshes=True):
    donor_dir = Path(donor_dir)
    urdf = donor_dir / DONOR_URDF
    hands = {s: HandUrdf(urdf, s) for s in ('right', 'left')}
    profiles = {s: h.profile(with_meshes=with_meshes) for s, h in hands.items()}
    out = {
        'schema': 'hand_fidelity_delta_v1', 'generated_utc': _dt.datetime.now(_dt.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
        'asset': {'urdf': DONOR_URDF, 'sha256': sha256_file(urdf), 'kind': 'PROVISIONAL RH56DFTP donor (Unitree unitree_ros, BSD-3-Clause)', 'exact_rh56e2': False},
        'evidence_classes': {'DONOR': 'derived from the donor URDF/meshes', 'NOMINAL': 'manufacturer specification', 'MEASURED': 'installed hardware (none available)', 'ASSUMED': 'declared convention'},
        'profiles': profiles,
        'coupling_findings': {s: profiles[s]['coupling_findings'] for s in profiles},
        'mirror_symmetry': mirror_symmetry(hands['right'], hands['left']),
        'nominal_comparison': {s: compare(hands[s], profiles[s]['mass_properties_open_root']['mass_kg']) for s in hands},
        'command_conventions': {name: {k: v for k, v in conv.items() if k != 'per_axis_endpoints'} | ({'per_axis_endpoints': conv['per_axis_endpoints']} if conv.get('per_axis_endpoints') else {}) for name, conv in cs.CONVENTIONS.items()},
        'radian_identity_discrepancy': {'unitree_inspire_hand_urdf_radians_v1 -> ftp_donor_radians_v1': cs.endpoint_discrepancy_table('unitree_inspire_hand_urdf_radians_v1', 'ftp_donor_radians_v1')},
        'load_records': {},
        'installed_similarity_percent': None,
    }
    tc2 = donor_dir / THUMBCHAIN2_URDF
    if tc2.exists():
        ja, jb = read_joints(urdf), read_joints(tc2)
        out['thumbchain2_equivalence'] = {'variant_sha256': sha256_file(tc2), **{s: {'equivalent': equivalent_layouts(ja, jb, s + '_')[0]} for s in ('right', 'left')}}
    for s, h in hands.items():
        mp = profiles[s].get('mass_properties_open_wrist')
        hand_comp = None if mp is None else {'mass_kg': mp['mass_kg'], 'com_m': mp['com_m'], 'inertia_about_com_kg_m2': mp['inertia_about_com_kg_m2'], 'provenance': 'URDF', 'configuration': 'donor open (URDF zero)',
                                             'note': 'right/left base_link inertials are not mirror images in the source (see mirror_symmetry)'}
        rec = load_record(s, hand=hand_comp, wrist_adapter=None, tool=None, asset_hashes={'urdf_sha256': out['asset']['sha256'], 'profile_sha256': profiles[s]['profile_sha256']},
                          notes=['wrist adapter/flange hardware mass: UNKNOWN (null); held tool: none declared here (the driver profile declares held_tool_payload_kg per job)'])
        validate_load_record(rec)
        out['load_records'][s] = rec
    out['delta_sha256'] = hashlib.sha256(json.dumps({k: v for k, v in out.items() if k not in ('generated_utc',)}, sort_keys=True, default=str).encode()).hexdigest()
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--donor-dir', required=True); p.add_argument('--out', required=True); p.add_argument('--no-meshes', action='store_true')
    a = p.parse_args(argv)
    out = build(a.donor_dir, with_meshes=not a.no_meshes)
    Path(a.out).write_text(json.dumps(out, indent=1, default=str))
    print('wrote', a.out, 'delta_sha256', out['delta_sha256'])
    for s in ('right', 'left'):
        for r in out['nominal_comparison'][s]['rows']:
            print('%-5s %-38s donor %-9s nominal %-8s diff %-8s %s' % (s, r['property'], r['donor'], r['nominal'], r['donor_minus_nominal'], r['datum_status']))
    print('mirror symmetry:', out['mirror_symmetry']['status'], sorted(out['mirror_symmetry']['asymmetric_links']))
    return out


if __name__ == '__main__':
    main()
