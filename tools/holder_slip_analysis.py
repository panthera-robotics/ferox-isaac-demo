#!/usr/bin/env python3
"""Per-stroke holder-in-palm motion from a contact-writing run: rotation about the marker axis, axis
tilt and translation of the held holder relative to the measured right palm (right_base_link), with
the contact normal force. CPU post hoc; reads ink_samples.jsonl + state.jsonl; writes holder_slip.json.
Distinguishes marker motion in the hand (grip slip) from wrist tracking (which moves palm and holder
together) and from spring compression (nib travel along the axis, reported separately).
"""
import argparse
import json
from pathlib import Path

import numpy as np


def rot(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', type=Path, required=True); ap.add_argument('--reference-window', type=float, nargs=2, default=(1.0, 1.45))
    a = ap.parse_args(argv)
    ink = [json.loads(l) for l in (a.run / 'ink_samples.jsonl').open()]
    state = {r['sequence']: r for r in map(json.loads, (a.run / 'state.jsonl').open())}
    rel = []
    for r in ink:
        s = state.get(r['sequence'])
        if s is None or 'holder_pose_world_xyzw' not in r:
            continue
        palm = s['link_poses_world_xyzw']['right_base_link']; Rp = rot(palm[3:7]); pp = np.asarray(palm[:3])
        Rh = rot(r['holder_pose_world_xyzw'][3:7]); ph = np.asarray(r['holder_pose_world_xyzw'][:3])
        R_rel = Rp.T @ Rh; p_rel = Rp.T @ (ph - pp)
        rel.append((r, R_rel, p_rel))
    ref = [(R, p) for r, R, p in rel if a.reference_window[0] <= r['physics_s'] < a.reference_window[1] and not r.get('support_active')]
    if len(ref) < 10:
        raise SystemExit('too few reference samples')
    R0 = ref[len(ref) // 2][0]; p0 = np.mean([p for _, p in ref], axis=0)
    axis0 = R0 @ np.array([0., 0., -1.])   # holder -z = pen axis, in palm frame
    per_stroke = {}
    for r, R, p in rel:
        if not r.get('pen_down') or r.get('stroke_id') is None:
            continue
        d = R0.T @ R   # holder rotation relative to the reference grip, expressed in the reference holder frame
        angle = np.degrees(np.arccos(np.clip((np.trace(d) - 1) / 2, -1, 1)))
        axis = R @ np.array([0., 0., -1.])
        tilt = np.degrees(np.arccos(np.clip(axis @ axis0, -1, 1)))
        # rotation about the marker axis = rotation of the holder x-axis projected on the plane normal to the pen axis
        x0 = R0 @ np.array([1., 0., 0.]); x1 = R @ np.array([1., 0., 0.])
        x0p = x0 - (x0 @ axis0) * axis0; x1p = x1 - (x1 @ axis0) * axis0
        spin = np.degrees(np.arctan2(np.cross(x0p, x1p) @ axis0, x0p @ x1p))
        dp = p - p0; along = float(dp @ axis0); lateral = float(np.linalg.norm(dp - along * axis0))
        per_stroke.setdefault(str(r['stroke_id']), []).append({'t': r['physics_s'], 'spin_deg': float(spin), 'tilt_deg': float(tilt), 'total_rot_deg': float(angle), 'along_axis_mm': 1000 * along, 'lateral_mm': 1000 * lateral,
                                                               'normal_force_n': r.get('normal_force_n'), 'compression_mm': 1000 * r.get('spring_compression_m', 0.0), 'hand_contacts': len(r.get('hand_contact_links') or [])})
    summary = {}
    for sid, rows in per_stroke.items():
        f = lambda k: [x[k] for x in rows]
        summary[sid] = {'samples': len(rows), 'spin_deg': {'start': rows[0]['spin_deg'], 'end': rows[-1]['spin_deg'], 'min': min(f('spin_deg')), 'max': max(f('spin_deg'))},
                        'tilt_deg': {'start': rows[0]['tilt_deg'], 'end': rows[-1]['tilt_deg'], 'max': max(f('tilt_deg'))},
                        'along_axis_mm': {'start': rows[0]['along_axis_mm'], 'end': rows[-1]['along_axis_mm'], 'min': min(f('along_axis_mm')), 'max': max(f('along_axis_mm'))},
                        'lateral_mm': {'start': rows[0]['lateral_mm'], 'end': rows[-1]['lateral_mm'], 'max': max(f('lateral_mm'))},
                        'normal_force_n_mean': float(np.mean([x for x in f('normal_force_n') if x is not None])), 'hand_contacts_min': min(f('hand_contacts'))}
    out = {'kind': 'holder_in_palm_motion_per_stroke', 'run': str(a.run), 'reference': 'unloaded held window %s s (support released, job not started)' % list(a.reference_window),
           'frames': 'holder pose expressed in the measured right_base_link (palm) frame; spin = rotation about the pen axis, tilt = pen-axis angle change, along/lateral = holder translation in the palm',
           'per_stroke': summary, 'note': 'grip slip is what moves here; wrist tracking moves palm and holder together and does not appear; spring compression is the nib, not the holder'}
    (a.run / 'holder_slip.json').write_text(json.dumps(out, indent=2) + '\n')
    for sid, s in summary.items():
        print('%-6s n=%4d spin %+6.2f -> %+6.2f deg (max %6.2f)  tilt end %5.2f max %5.2f deg  along %+5.2f -> %+5.2f mm  lateral end %5.2f max %5.2f mm  F %.2f N  contacts>=%d' % (
            sid, s['samples'], s['spin_deg']['start'], s['spin_deg']['end'], s['spin_deg']['max'], s['tilt_deg']['end'], s['tilt_deg']['max'], s['along_axis_mm']['start'], s['along_axis_mm']['end'], s['lateral_mm']['end'], s['lateral_mm']['max'], s['normal_force_n_mean'], s['hand_contacts_min']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
