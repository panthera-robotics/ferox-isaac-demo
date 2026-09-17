#!/usr/bin/env python3
"""Post-hoc error chain of one contact-writing run, link by link, in the board frame (u, v, n):

  A  planned board path (approved job 'tip')            -> FK(approved q) with the declared tool tip   [IK/plan residual]
  B  approved q at the streamed index                   -> effective actuator reference q              [streamer clamp/lag]
  C  effective reference q                              -> measured q                                  [joint tracking]
  D  FK(measured q) with the declared tool tip          -> measured nib tip (PhysX body)               [tool frame + grip slip + spring]
  E  measured nib tip                                   -> impulse-weighted contact point / board plane [contact geometry, compression]

Per pen-down sample the total (planned -> measured nib) equals A+B+C+D; the chain is reported per
phase with mean / p95 along u, v, n and with the tangential (along-stroke) component separated from
the geometric (across-stroke) component. Stdlib + numpy + urdf_kinematics only; nothing is rescored.
Usage: writer_error_chain.py --run <evidence dir> --fixture <writer fixture dir> [--out error_chain.json]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from urdf_kinematics import UrdfKinematics  # noqa: E402

ARM_JOINTS = ['left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
              'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint']
WRIST_T_RUBBER = np.eye(4); WRIST_T_RUBBER[:3, 3] = [0.0415, -0.003, 0.]   # right_rubber_hand on right_wrist_yaw_link (G1 URDF)


def stats(rows):
    if not rows:
        return None
    a = np.asarray(rows, dtype=float)
    return {'n': int(len(a)), 'mean_mm': (a.mean(0) * 1000).round(3).tolist(), 'p95_abs_mm': (np.percentile(np.abs(a), 95, axis=0) * 1000).round(3).tolist(),
            'max_abs_mm': (np.abs(a).max(0) * 1000).round(3).tolist()}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', type=Path, required=True); ap.add_argument('--fixture', type=Path, required=True); ap.add_argument('--out', type=Path)
    a = ap.parse_args(argv)
    run = a.run
    k = UrdfKinematics(run / 'assembled_physical.urdf')
    tb = yaml.safe_load((a.fixture / 'scene_tool_board.yaml').read_text())
    U, V, N = [np.asarray(tb['board'][key], dtype=float) for key in ('u_axis_unit', 'v_axis_unit', 'normal_unit')]
    O = np.asarray(tb['board']['origin_xyz_m'], dtype=float) + np.array([0., 0., 1.])   # pelvis fixed 1 m above the origin
    tip_w = np.asarray(tb['tool']['tip_xyz_m'], dtype=float); axis_w = np.asarray(tb['tool']['pen_axis_unit'], dtype=float)
    compression_nominal = float(tb['tool']['compression_m_nominal'])
    pelvis = np.eye(4); pelvis[:3, 3] = [0., 0., 1.]
    job = json.loads(next((run / 'writer_bridge').glob('approved_job_*.json')).read_text())['content']
    path = job['path']
    ink = [json.loads(l) for l in (run / 'ink_samples.jsonl').open()]
    state = {r['sequence']: r for r in map(json.loads, (run / 'state.jsonl').open())}
    refs = {}
    for line in (run / 'writer_bridge' / 'effective_body_commands.jsonl').open():
        r = json.loads(line); refs[r['physics_sequence']] = r['joints']
    to_board = lambda p: np.array([(p - O) @ U, (p - O) @ V, (p - O) @ N])

    def fk_tip(q_named, compression):
        W = k.transforms(q_named, pelvis)['right_wrist_yaw_link'] @ WRIST_T_RUBBER
        return (W @ np.append(tip_w - compression * axis_w, 1.))[:3]

    chain = {'A_plan_minus_fk_plan': {}, 'B_fk_plan_minus_fk_reference': {}, 'C_fk_reference_minus_fk_measured': {}, 'D_fk_measured_minus_nib': {}, 'total_plan_minus_nib': {}, 'E_nib_minus_contact': {}}
    per = {key: {} for key in chain}
    tangential = {}
    samples = []
    for r in ink:
        idx = r.get('job_index'); seq = r['sequence']
        if not isinstance(idx, int) or seq not in state or seq not in refs or r['job_state'] != 'running':
            continue
        kk = max(0, min(len(path) - 1, idx - 1)); sample = path[kk]
        phase = ('pen_down' if sample['pen_down'] else sample['phase'])
        q_plan = dict(zip(ARM_JOINTS, sample['q']))
        base = dict(zip(state[seq]['runtime_names'], state[seq]['q_rad']))          # measured
        q_plan_full = dict(base, **q_plan)
        q_ref_full = dict(base, **{n: refs[seq][n]['implicit_target_rad'] - refs[seq][n]['feedforward_bias_rad'] for n in ARM_JOINTS if n in refs[seq]})
        comp = float(r['spring_compression_m']) if r['spring_compression_m'] is not None else 0.
        planned = np.asarray(sample['tip'], dtype=float) + np.array([0., 0., 1.])
        fk_plan = fk_tip(q_plan_full, compression_nominal)      # the planner places the nominally compressed tip on the plane
        fk_ref = fk_tip(q_ref_full, comp)
        fk_meas = fk_tip(base, comp)
        nib = np.asarray(r['tip_world_m'], dtype=float)
        contact = np.asarray(r['position_board_m'], dtype=float) if r['nib_board_contact'] else None
        row = {'A': to_board(planned) - to_board(fk_plan), 'B': to_board(fk_plan) - to_board(fk_ref), 'C': to_board(fk_ref) - to_board(fk_meas),
               'D': to_board(fk_meas) - to_board(nib), 'T': to_board(planned) - to_board(nib)}
        if contact is not None:
            row['E'] = to_board(nib) - contact
        for key, name in (('A', 'A_plan_minus_fk_plan'), ('B', 'B_fk_plan_minus_fk_reference'), ('C', 'C_fk_reference_minus_fk_measured'), ('D', 'D_fk_measured_minus_nib'), ('T', 'total_plan_minus_nib'), ('E', 'E_nib_minus_contact')):
            if key in row:
                per[name].setdefault(phase, []).append(row[key].tolist())
        # tangential vs geometric split of the total error along the planned stroke direction (pen-down only)
        if sample['pen_down'] and 0 < kk < len(path) - 1:
            d = np.asarray(path[kk + 1]['tip']) - np.asarray(path[kk - 1]['tip']); d_uv = np.array([d @ U, d @ V])
            if np.linalg.norm(d_uv) > 1e-9:
                t_hat = d_uv / np.linalg.norm(d_uv); e_uv = row['T'][:2]
                tangential.setdefault(sample['char'] + '_' + str(sample['stroke_index']), []).append([float(e_uv @ t_hat), float(e_uv @ np.array([-t_hat[1], t_hat[0]]))])
        samples.append({'sequence': seq, 'job_index': idx, 'phase': phase, **{key: (v.tolist() if isinstance(v, np.ndarray) else v) for key, v in row.items()}, 'compression_m': comp, 'contact': bool(contact is not None), 'normal_force_n': r['normal_force_n']})
    for name in per:
        chain[name] = {phase: stats(rows) for phase, rows in per[name].items()}
    tangential_stats = {stroke: {'n': len(v), 'along_stroke_mean_mm': round(float(np.mean([x[0] for x in v]) * 1000), 3), 'along_stroke_p95_abs_mm': round(float(np.percentile(np.abs([x[0] for x in v]), 95) * 1000), 3),
                                 'across_stroke_mean_mm': round(float(np.mean([x[1] for x in v]) * 1000), 3), 'across_stroke_p95_abs_mm': round(float(np.percentile(np.abs([x[1] for x in v]), 95) * 1000), 3)} for stroke, v in tangential.items()}
    report = {'schema_version': 1, 'kind': 'writer_error_chain', 'run': str(run), 'fixture': str(a.fixture), 'board_axes': 'u, v, n of the fixture board frame (mm)', 'samples': len(samples),
              'declared_tool_tip_wrist_m': tip_w.tolist(), 'compression_nominal_m': compression_nominal,
              'chain_by_phase': chain, 'tangential_vs_geometric_pen_down': tangential_stats,
              'reading': 'A: IK/plan residual; B: streamer/reference lag (approved q vs effective reference at the same physics step); C: joint tracking; D: tool frame + grip slip + spring geometry; E: nib body vs contact point. Total = A+B+C+D per sample. Along-stroke error is phase lag, across-stroke error is geometric path error.'}
    out = a.out or (run / 'error_chain.json')
    out.write_text(json.dumps(report, indent=2))
    (run / 'error_chain_samples.jsonl').write_text('\n'.join(json.dumps(s) for s in samples) + '\n')
    print(json.dumps({'samples': len(samples), 'pen_down_total': chain['total_plan_minus_nib'].get('pen_down'), 'pen_down_C_tracking': chain['C_fk_reference_minus_fk_measured'].get('pen_down'),
                      'pen_down_B_reference_lag': chain['B_fk_plan_minus_fk_reference'].get('pen_down'), 'pen_down_D_tool': chain['D_fk_measured_minus_nib'].get('pen_down'), 'tangential': tangential_stats}, indent=1))


if __name__ == '__main__':
    main()
