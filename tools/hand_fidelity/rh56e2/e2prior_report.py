#!/usr/bin/env python3
"""Render E2_PRIOR_ASSET_AUDIT.md (verdict + exact-vs-inferred per field + every check with its numbers) and
DONOR_VS_E2_PUBLIC_PRIOR.md (frozen FTP donor vs the public exact-E2 prior: geometry, ranges, coupling, masses, frames, mount)
from the audit JSON, the asset manifest, the contract and the two URDFs. CPU (pinocchio) only.
usage: e2prior_report.py --asset-dir <generated/e2prior> --donor-dir <generated/ftp_donor> --audit <audit.json> --contract <E2_KINEMATIC_CONTRACT.json> --out-dir <dir>
"""
from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pinocchio as pin

EXACTNESS = [
    ('finger joint axes/origins, link tree, mesh files', 'EXACT_PUBLIC_SOURCE', 'macro xacro @ 81bdb56 expanded verbatim (only ${prefix}/${parent}/origin/mesh path substitution)'),
    ('finger proximal range 0–1.4381, intermediate ratio 1.0843, cap 1.476374', 'EXACT_PUBLIC_SOURCE', 'URDF limits/mimic'),
    ('thumb bend 0–0.62, intermediate ×0.8392, distal ×0.7477272 (cap 0.45553), rotation 0–1.658', 'EXACT_PUBLIC_SOURCE', 'URDF limits/mimic'),
    ('coupled-target clamp at the child limit', 'DECLARED_POLICY', 'public child limits are below the composed range; the twin clamps the target rather than driving into the limit'),
    ('right-hand link masses / COM / inertia', 'EXACT_PUBLIC_SOURCE', 'right macro inertials as published (CAD export; NOT measured)'),
    ('left-hand main-link COM / inertia', 'INFERRED', 'right inertials mirrored across the hand-base y=0 plane (public left file copies the right values unmirrored)'),
    ('left_pinky_intermediate mass 0.01166 kg', 'INFERRED', 'right value; the public left file carries the proximal mass 0.01869 kg'),
    ('left force-sensor link inertials', 'EXACT_PUBLIC_SOURCE', 'kept from the public left file (they are properly mirrored there, agreement ≤ 0.6 mm / 0.3 %)'),
    ('total hand mass 0.7767 kg (mass policy PUBLIC_URDF_LINK_MASSES)', 'DECLARED_POLICY', 'sum of the public link masses = E2_CAD_MASS; E2_NOMINAL_MASS 0.790 ± 0.010 optional diagnostic only; installed mass NOT_MEASURED'),
    ('drive limits effort 10 N·m / velocity 1.0 rad/s (merged twin URDF)', 'DECLARED_POLICY', 'donor values carried; public fields are placeholders (1 / 1–2) and asymmetric between sides; installed NOT_MEASURED'),
    ('wrist mount = donor flange · R_z(+90°)', 'INFERRED', 'chosen so the E2 finger joints land on the donor datum (0.0 mm); not a hardware measurement'),
    ('chirality (left = y-mirror of right, thumb on the index side, palm normal to the midline)', 'EXACT_PUBLIC_SOURCE', 'verified by the audit (handedness triple product)'),
    ('collision geometry = public STL meshes (64, LFS oids verified)', 'EXACT_PUBLIC_SOURCE', 'left palm_1 STL is a separate export (interior differs); PhysX cooking/decomposition is a runtime choice'),
    ('count mapping c = 1 − count/1000, rad = c × closed_rad', 'PUBLIC_PRIOR (UNVERIFIED linearity)', 'manual direction; installed native-to-radian map = hand-req-Q01'),
    ('thumb rotation count 1000 = out = URDF yaw 0', 'INFERRED', 'manual convention applied to the URDF zero pose; hardware-unverified'),
    ('17 tactile frames per hand, 1062 taxels', 'EXACT_PUBLIC_SOURCE (frames) / TACTILE_LAYOUT_PUBLIC_PRIOR (arrays)', 'array-to-frame assignment inferred from mesh size and position; orientation unverified'),
    ('generated USD', 'NOT_GENERATED_ON_CPU', 'Kit URDF importer run pending the coordinator GPU slot; hashed when produced'),
]


def fmt(v):
    return json.dumps(v, default=float)


def audit_md(audit, manifest, contract):
    L = ['# E2_PRIOR_ASSET_AUDIT — %s' % manifest['asset_id'], '', 'Label: **%s**. Verdict: **%s** (%s).' % (manifest['label'], audit['verdict'], ', '.join('%s %d' % (k, v) for k, v in audit['summary'].items())),
         'Merged twin URDF sha256 `%s`; source %s @ %s; donor status %s.' % (manifest['merged_urdf']['sha256'], manifest['source']['url'], manifest['source']['commit'], manifest['donor_status'].split(' (')[0]), '',
         'Nothing in this asset is an installed measurement. "Do not task-tune the hand until this audit passes" — the static audit passes; the physics pass (drive stiffness, contact offsets, cooking) is the next gate and is not part of this verdict.', '',
         '## Exact vs inferred, per field', '', '| field | label | basis |', '|---|---|---|']
    L += ['| %s | %s | %s |' % r for r in EXACTNESS]
    L += ['', '## Checks', '']
    for c in audit['checks']:
        d = c['detail']; side = ' (%s)' % c['side'] if c['side'] else ''
        L.append('### %s%s — %s' % (c['check'], side, c['result']))
        if c['check'] == 'chirality_and_mount':
            for s, v in d['per_side'].items():
                L.append('- %s: palm normal %s (towards midline: %s), handedness triple product %.4f (ok %s), thumb tip on the index side %s, palm normal vs donor %.4f°, joint datum vs donor mm %s, thumb base E2 %s vs donor %s' % (s, v['palm_normal_pelvis'], v['palm_normal_towards_midline'], v['handedness_triple_product'], v['handedness_ok'], v['thumb_tip_on_index_side'], v['palm_normal_vs_donor_deg'], v['joint_datum_vs_donor_mm'], v['e2_thumb_base_pelvis'], v['donor_thumb_base_pelvis']))
            L.append('- left vs mirrored right at q=0 (mm): %s' % d['left_vs_mirrored_right_mm_at_q0'])
        elif c['check'] == 'left_inertials_mirror_right':
            L.append('- worst: %s; tolerance: %s' % (fmt(d['worst']), d['tolerance']))
        elif c['check'] in ('self_collision_preflight_open',):
            L.append('- pairs %d, intersecting %d, closest: %s' % (d['pairs'], len(d['intersecting']), fmt(d['closest_5'][:3])))
        elif c['check'] == 'self_collision_closed_and_opposed':
            for k in ('closed', 'opposed_open_fingers'):
                L.append('- %s: pairs %d, intersecting %d, max depth %.2f mm, deepest: %s' % (k, d[k]['pairs'], len(d[k]['intersecting']), d[k]['max_depth_mm'], fmt([(r['pair'], r['depth_mm']) for r in d[k]['intersecting'][:4]])))
            L.append('- %s' % d['note'])
        elif c['check'] == 'self_collision_envelope':
            for k, v in d.items():
                if k != 'note': L.append('- %s: %s' % (k, fmt(v)))
        elif c['check'] == 'palm_thumb_cavity_sweep':
            L.append('- worst depth %.2f mm at %s; samples %d; intersecting samples %d' % (d['worst_depth_mm'], fmt(d['worst_at']), d['samples'], len(d['intersecting_samples'])))
        elif c['check'] == 'left_meshes_mirror_right_envelope':
            L.append('| link | triangles R/L | enclosed cm³ R/L | bbox shift mm | R→L surface p95/max mm | L→R surface p95/max mm |'); L.append('|---|---|---|---|---|---|')
            for k, v in sorted(d['per_link'].items(), key=lambda kv: -kv[1]['right_mirrored_to_left_surface_mm']['max']):
                L.append('| %s | %s | %s | %.3f | %.3f / %.3f | %.3f / %.3f |' % (k, '/'.join(map(str, v['triangles'])), '/'.join(map(str, v['enclosed_volume_cm3'])), v['bbox_shift_mm'], v['right_mirrored_to_left_surface_mm']['p95'], v['right_mirrored_to_left_surface_mm']['max'], v['left_mirrored_to_right_surface_mm']['p95'], v['left_mirrored_to_right_surface_mm']['max']))
        elif c['check'] == 'mesh_scale_metres':
            L.append('- scale attributes %s; largest link extent (m) %s; palm box (m) %s; meshes %d' % (d['scale_attributes'], d['largest_link_extent_m'], d['palm_box_m'], d['n_meshes']))
        elif c['check'] == 'mimic_child_limits_vs_composed_range':
            for k, v in d['per_mimic'].items(): L.append('- %s: child upper %.6f, composed at root upper %.6f, saturates %s%s' % (k, v['child_upper'], v['composed_at_root_upper'], v['child_saturates'], (' at root %.6f' % v['root_value_at_child_limit']) if v['child_saturates'] else ''))
            L.append('- %s' % d['policy'])
        elif c['check'] == 'drive_limits_declared_policy':
            L.append('- merged URDF (effort, velocity) on all 12 joints: %s; policy %s; public placeholders %s' % (sorted({tuple(v) for v in d['merged_urdf_values'].values()}), d['policy'].get('policy'), sorted({tuple(v) for v in d['public_placeholders'].values()})))
        elif c['check'] == 'tactile_frames_17':
            L.append('- %d frames: %s' % (len(d['frames']), ', '.join(n.replace(c['side'] + '_', '') for n in d['frames'])))
        else:
            L.append('- ' + fmt({k: v for k, v in d.items() if k not in ('per_link', 'grid')})[:900])
        L.append('')
    L += ['## Self-collision envelope (declared in E2_KINEMATIC_CONTRACT.json)', '', '```json', json.dumps(contract['self_collision_envelope'], indent=1), '```', '',
          '## Pre-existing test failures on the branch base', '', 'tools/tests/test_isaaclab_cfg.py (3 tests: deploy-yaml joint order / gains) fail at the branch base 5163c1e as well; unrelated to this lane. tools/tests/test_twin_isaac.py needs the Isaac runtime.', '']
    return '\n'.join(L)


def donor_vs_e2_md(asset, donor_dir, audit, manifest, contract):
    du = donor_dir / 'g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf'; eu = asset / manifest['merged_urdf']['name']
    dm = pin.buildModelFromUrdf(str(du)); em = pin.buildModelFromUrdf(str(eu)); dd = dm.createData(); ed = em.createData()
    dx = ET.parse(du).getroot(); ex = ET.parse(eu).getroot()
    def hand_mass(x, side, keys):
        return sum(float(l.find('inertial/mass').get('value')) for l in x.findall('link') if l.get('name').startswith(side + '_') and not any(k in l.get('name') for k in keys) and l.find('inertial') is not None)
    body = ('shoulder', 'elbow', 'wrist', 'hip', 'knee', 'ankle')
    dmass = {s: round(hand_mass(dx, s, body), 4) for s in ('right', 'left')}; emass = {s: round(hand_mass(ex, s, body), 4) for s in ('right', 'left')}
    def lim(x, name):
        j = x.find("joint[@name='%s']" % name); l = j.find('limit'); m = j.find('mimic'); return (float(l.get('lower')), float(l.get('upper')), l.get('effort'), l.get('velocity'), None if m is None else (m.get('joint'), float(m.get('multiplier'))))
    rows = [('index proximal', 'right_index_1_joint', 'right_index_proximal_joint'), ('index intermediate (coupled)', 'right_index_2_joint', 'right_index_intermediate_joint'),
            ('thumb rotation', 'right_thumb_1_joint', 'right_thumb_proximal_yaw_joint'), ('thumb bend', 'right_thumb_2_joint', 'right_thumb_proximal_pitch_joint'),
            ('thumb intermediate (coupled)', 'right_thumb_3_joint', 'right_thumb_intermediate_joint'), ('thumb distal (coupled)', 'right_thumb_4_joint', 'right_thumb_distal_joint')]
    # closed-pose fingertip / thumb-tip positions in the hand base frame
    def closed_q(model, is_e2):
        q = pin.neutral(model)
        for j in range(1, model.njoints):
            n = model.names[j]
            if any(k in n for k in body) or not n.startswith(('right_', 'left_')): continue
            lo, hi = model.lowerPositionLimit[model.idx_qs[j]], model.upperPositionLimit[model.idx_qs[j]]
            x = ex if is_e2 else dx; l = lim(x, n)
            if l[4] is not None:
                parent_hi = lim(x, l[4][0])[1]; q[model.idx_qs[j]] = min(l[4][1] * parent_hi, hi)
            else: q[model.idx_qs[j]] = hi
        return q
    def tips(model, data, q, base, names):
        pin.forwardKinematics(model, data, q); pin.updateFramePlacements(model, data); b = data.oMf[model.getFrameId(base)]
        return {k: (b.inverse() * data.oMf[model.getFrameId(v)]).translation.round(4).tolist() for k, v in names.items()}
    dn = {'index_tip': 'right_index_force_sensor_3', 'thumb_tip': 'right_thumb_force_sensor_3', 'palm_sensor': 'right_palm_force_sensor', 'thumb_base': 'right_thumb_1'}
    en = {'index_tip': 'right_index_force_sensor_3', 'thumb_tip': 'right_thumb_force_sensor_3', 'palm_sensor': 'right_palm_force_sensor', 'thumb_base': 'right_thumb_proximal'}
    # express the donor in the E2 base frame: E2 base = donor base · R_z(+90°): p_e2 = R_z(-90) p_donor
    Rz = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=float)
    d_open = tips(dm, dd, pin.neutral(dm), 'right_base_link', dn); e_open = tips(em, ed, pin.neutral(em), 'right_base', en)
    d_closed = tips(dm, dd, closed_q(dm, False), 'right_base_link', dn); e_closed = tips(em, ed, closed_q(em, True), 'right_base', en)
    conv = lambda p: (Rz @ np.array(p)).round(4).tolist()
    L = ['# DONOR_VS_E2_PUBLIC_PRIOR — frozen FTP donor vs %s' % manifest['asset_id'], '',
         'Donor: `%s` (sha256 %s…), status DONOR_BASELINE_FROZEN, provisional RH56DFTP geometry. E2: public exact-model description @ %s, label %s. All E2 numbers are public priors; none is an installed measurement.' % (du.name, manifest['donor_body']['urdf_sha256'][:12], manifest['source']['commit_short'], manifest['label']), '',
         '## Ranges, coupling, drive fields (right hand; left identical by construction)', '', '| joint | donor [lo, hi] effort/vel mimic | E2 [lo, hi] effort/vel (merged) mimic | Δ closed (deg) |', '|---|---|---|---|']
    for label, dj, ej in rows:
        a = lim(dx, dj); b = lim(ex, ej)
        L.append('| %s | [%g, %g] %s/%s %s | [%g, %g] %s/%s %s | %+.1f |' % (label, a[0], a[1], a[2], a[3], '' if a[4] is None else '×%g of %s' % (a[4][1], a[4][0].replace('right_', '')), b[0], b[1], b[2], b[3], '' if b[4] is None else '×%g of %s' % (b[4][1], b[4][0].replace('right_', '')), math.degrees(b[1] - a[1])))
    L += ['', 'Δ closed for the coupled rows compares the URDF child limits (the donor 3.14 is a placeholder; effective donor intermediate closure = 1.0843 × 1.4381 = 1.5593 vs E2 1.476374, −4.8°).', '', 'Thumb rotation: donor 1.1641 rad (66.7°) → E2 1.658 rad (95.0°), +28.3°: the E2 thumb reaches full opposition the donor never had. **The donor range is not retained**; grasps planned on the donor thumb (rod30 family C1–C5, v8u2 mission rows) must be re-planned through the closure-preserving map (E2_KINEMATIC_CONTRACT.donor_to_e2), not replayed as radians.',
          'Thumb bend: 0.5864 → 0.62 (+1.9°); thumb chain ratios 0.8024/0.9487 (donor) → 0.8392/0.7477272 (E2, composed on the bend joint). Finger roots identical (1.4381, ×1.0843); the E2 intermediate cap 1.476374 replaces the donor\'s 3.14 (child saturates when the root passes 1.3616 rad).',
          'Drive fields: donor effort 10 / velocity 1.0 carried into the merged E2 twin URDF as a declared policy (public E2 fields are placeholders: effort 1, velocity right fingers 1.0 / left fingers 2.0 / thumbs 2.0).', '',
          '## Mass', '', '| | donor | E2 (primary) | Δ |', '|---|---|---|---|',
          '| right hand total (kg) | %.4f | %.4f | %+.4f |' % (dmass['right'], emass['right'], emass['right'] - dmass['right']), '| left hand total (kg) | %.4f | %.4f | %+.4f |' % (dmass['left'], emass['left'], emass['left'] - dmass['left']),
          '| merged robot (kg) | %.4f | %.4f | %+.4f |' % (pin.computeTotalMass(dm), pin.computeTotalMass(em), pin.computeTotalMass(em) - pin.computeTotalMass(dm)), '',
          'Mass policy of the primary asset: **%s** (%s). E2_NOMINAL_MASS 0.790 ± 0.010 kg is documented as an optional diagnostic scaling, not selected. The body now carries 0.1016 kg less per hand than every donor-qualified run; body-lane A/B (variant B = E2 geometry + donor per-link masses) is the declared way to separate the mass effect from the geometry effect.' % (manifest['mass_policy']['primary'], manifest['mass_policy']['statement'].split('. ')[1]), '',
          '## Frames and mount', '',
          '- Donor hand base frame (`right_base_link`): fingers +z, across x (index −x), palm normal +y. E2 base (`right_base`): fingers +z, across y (index +y), palm normal +x. Relation: E2 base = donor base · R_z(+90°); mount on the wrist: donor flange (0.0415, 0, 0; rpy 0, π/2, 0) → E2 (0.0415, 0, 0; rpy π/2, 0, π/2) right, (−π/2, 0, −π/2) left. INFERRED (not measured).',
          '- Positions in the E2 base frame (m), donor rotated into it, right hand:', '', '| point | donor open | E2 open | donor closed | E2 closed |', '|---|---|---|---|---|']
    for k in ('index_tip', 'thumb_tip', 'palm_sensor', 'thumb_base'):
        L.append('| %s | %s | %s | %s | %s |' % (k, conv(d_open[k]), e_open[k], conv(d_closed[k]), e_closed[k]))
    ch = [c for c in audit['checks'] if c['check'] == 'chirality_and_mount'][0]['detail']['per_side']['right']
    L += ['', '- Index/pinky proximal joints coincide with the donor datum (%s mm); the E2 thumb base sits %.1f mm further along the fingers (donor thumb base %s vs E2 %s in the pelvis frame at q=0).' % (ch['joint_datum_vs_donor_mm'], ch['joint_datum_vs_donor_mm']['thumb_proximal'], ch['donor_thumb_base_pelvis'], ch['e2_thumb_base_pelvis']),
          '- Tactile: donor 17 force-sensor frames (no arrays); E2 17 frames with the Table 56 array assignment (1062 taxels, TACTILE_LAYOUT_PUBLIC_PRIOR).',
          '- Collision: donor used provisional palm slabs (ftp_palm_yz_slabs_v2 / ftp_left_palm_yz_slabs_v1) + meshes; E2 uses the public STL meshes for every link (64 files); PhysX cooking is a runtime declaration for the physics pass.', '',
          '## Consequences for existing recipes', '',
          '- Finger geometry is on the donor datum → the fingers-horizontal transit / seat rows of the v8u2 mission recipe are geometrically compatible; the thumb closure differs (range, chain ratios, base position) → every donor grasp that relied on the thumb (rod30 C1–C5, standing pick lEc) is re-planned, not replayed; the six-axis rows go through donor_rows_to_e2.',
          '- Self-collision envelope (contract): with the thumb fully opposed the fingers may close only to %.3f rad before the index chain meets the thumb (right); with the fingers closed the opposed thumb touches at a bend of %.3f rad. The real hand stalls on contact; the twin must treat these as closure events, not command them as free targets.' % (contract['self_collision_envelope']['right']['thumb_fully_opposed_first_contact_at_finger_closure_rad'][0], contract['self_collision_envelope']['right']['fingers_closed_thumb_yaw_max_first_contact_at_pitch_rad'][0]),
          '- Historical F7 / FAIL verdicts on the donor are not relabelled by this migration.', '']
    return '\n'.join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--asset-dir', required=True); ap.add_argument('--donor-dir', required=True); ap.add_argument('--audit', required=True); ap.add_argument('--contract', required=True); ap.add_argument('--out-dir', required=True); a = ap.parse_args()
    asset = Path(a.asset_dir); manifest = json.loads((asset / 'E2_PRIOR_ASSET_MANIFEST.json').read_text()); audit = json.loads(Path(a.audit).read_text()); contract = json.loads(Path(a.contract).read_text()); out = Path(a.out_dir); out.mkdir(parents=True, exist_ok=True)
    (out / 'E2_PRIOR_ASSET_AUDIT.md').write_text(audit_md(audit, manifest, contract)); (out / 'DONOR_VS_E2_PUBLIC_PRIOR.md').write_text(donor_vs_e2_md(asset, Path(a.donor_dir), audit, manifest, contract))
    print('wrote', out / 'E2_PRIOR_ASSET_AUDIT.md', out / 'DONOR_VS_E2_PUBLIC_PRIOR.md')


if __name__ == '__main__':
    main()
