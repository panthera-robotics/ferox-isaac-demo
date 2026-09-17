#!/usr/bin/env python3
"""Measure the held marker's tool frame in the assembled hand from a contact-writing run.

Window: after the declared support release and before the job moves (the marker is held, unloaded,
the nib is free). Output: unloaded nib tip and pen axis (holder -z) expressed in the planner's
`right_rubber_hand` frame at the measured joint state (planner model FK), with per-sample spread,
plus the loaded (pen-down) tip for the compression check. This is a SIMULATOR-derived frame for the
exact grasp/asset/holder of the run; any change to those invalidates it. It is not a hardware calibration.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from urdf_kinematics import UrdfKinematics  # noqa: E402


def rot(qxyzw):
    x, y, z, w = qxyzw
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', type=Path, required=True); ap.add_argument('--fixture', type=Path, required=True)
    ap.add_argument('--window', type=float, nargs=2, default=(1.0, 1.45), help='physics seconds of the unloaded window (after support release, before the job runs)')
    ap.add_argument('--out', type=Path)
    a = ap.parse_args(argv)
    k = UrdfKinematics(a.fixture / 'planner_donor_open_hands.urdf')
    pelvis = np.eye(4); pelvis[:3, 3] = [0., 0., 1.]
    ink = [json.loads(l) for l in (a.run / 'ink_samples.jsonl').open()]
    state = {r['sequence']: r for r in map(json.loads, (a.run / 'state.jsonl').open())}
    metrics = json.loads((a.run / 'metrics.json').read_text())
    tips, axes, loaded = [], [], []
    for r in ink:
        s = state.get(r['sequence'])
        if s is None or r['support_active']:
            continue
        W = k.transforms(dict(zip(s['runtime_names'], s['q_rad'])), pelvis)['right_rubber_hand']
        tip_local = (np.linalg.inv(W) @ np.append(np.asarray(r['tip_world_m']), 1.))[:3]
        axis_local = W[:3, :3].T @ (rot(r['holder_pose_world_xyzw'][3:7]) @ np.array([0., 0., -1.]))
        if a.window[0] <= r['physics_s'] < a.window[1] and r['job_state'] != 'running':
            tips.append(tip_local); axes.append(axis_local)
        elif r['pen_down'] and r['nib_board_contact']:
            loaded.append((tip_local, r['spring_compression_m']))
    if len(tips) < 20:
        raise SystemExit('too few unloaded samples in the window (%d)' % len(tips))
    tips = np.asarray(tips); axis = np.mean(axes, axis=0); axis /= np.linalg.norm(axis)
    result = {'schema_version': 1, 'kind': 'simulator_measured_held_marker_tool_frame', 'source_run': str(a.run), 'fixture_used_by_run': str(a.fixture),
              'grasp_sha256': metrics.get('held_marker_grasp', {}).get('config_sha256'), 'held_marker_grasp_path': metrics.get('held_marker_grasp', {}).get('source'),
              'frame': 'right_rubber_hand (planner model FK at the measured joint state)', 'window_physics_s': list(a.window), 'samples': int(len(tips)),
              'tip_xyz_m': tips.mean(0).round(6).tolist(), 'tip_std_m': tips.std(0).round(6).tolist(), 'pen_axis_unit': axis.round(6).tolist(),
              'axis_spread_deg': round(float(np.degrees(np.max([np.arccos(np.clip(ax @ axis, -1, 1)) for ax in axes]))), 3),
              'loaded_pen_down_tip_mean_m': (np.mean([t for t, _ in loaded], axis=0).round(6).tolist() if loaded else None),
              'loaded_compression_mean_m': (round(float(np.mean([c for _, c in loaded])), 6) if loaded else None),
              'not_hardware_calibration': True, 'invalidated_by': 'any change of grasp file, asset revision, holder parameters, drive gains or wrist mount'}
    if loaded:
        expected = tips.mean(0) - result['loaded_compression_mean_m'] * axis
        result['loaded_vs_unloaded_minus_compression_mm'] = ((np.asarray(result['loaded_pen_down_tip_mean_m']) - expected) * 1000).round(3).tolist()
    out = a.out or (a.run / 'measured_tool_frame.json')
    out.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ('samples', 'tip_xyz_m', 'tip_std_m', 'pen_axis_unit', 'axis_spread_deg', 'loaded_vs_unloaded_minus_compression_mm')}, indent=1))


if __name__ == '__main__':
    main()
