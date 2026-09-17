#!/usr/bin/env python3
"""Compare a command-replay run with the recording's observed responses (host CPU, post hoc).

Mode 3 (real-response comparison) is only meaningful where physical provenance, observations, timing
and controller semantics support it; this tool therefore reports, per joint and per hand axis, the
discrepancy between the simulated feedback and the recorded readback at the same source time, plus
the discrepancy of each against the shared command — and lists the declared mismatches verbatim from
the source contract. It never adjusts masses, gains or friction. Output: comparison.json in the run.
Usage: replay_compare.py --run <evidence dir> --package <replay package dir>
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', type=Path, required=True); ap.add_argument('--package', type=Path, required=True)
    a = ap.parse_args(argv)
    observed = json.loads((a.package / 'observed.json').read_text()) if (a.package / 'observed.json').exists() else None
    sequence = json.loads((a.package / 'sequence.json').read_text())
    metrics = json.loads((a.run / 'metrics.json').read_text())
    if observed is None:
        result = {'kind': 'real_response_comparison', 'status': 'NOT_TESTED', 'reason': 'source carries no observed responses (synthetic or command-only source)', 'source': sequence['source']}
        (a.run / 'comparison.json').write_text(json.dumps(result, indent=2) + '\n'); print(json.dumps(result, indent=1)); return 0
    state = [json.loads(l) for l in (a.run / 'state.jsonl').open()]
    replay = [r for r in state if r['phase'] == 'replay' and r['source_row'] is not None]
    names = replay[0]['runtime_names']; body_names = replay[0]['body_command_names']
    obs_by_row = {i: o for i, o in enumerate(observed)}
    sim_by_row = {}
    for r in replay:   # last physics sample of each held source row = the simulated response at the end of that command interval
        sim_by_row[r['source_row']] = r
    per_joint = {}
    for n in body_names:
        i = names.index(n); ci = body_names.index(n)
        d_sim_real, d_sim_cmd, d_real_cmd = [], [], []
        for row, r in sim_by_row.items():
            o = obs_by_row.get(row)
            if o is None:
                continue
            q_sim, q_real, q_cmd = r['q_rad'][i], o['robot_q_current_rad'][n], r['body_command_rad'][ci]
            d_sim_real.append(q_sim - q_real); d_sim_cmd.append(q_sim - q_cmd); d_real_cmd.append(q_real - q_cmd)
        arr = lambda v: {'rms_rad': float(np.sqrt(np.mean(np.square(v)))), 'max_abs_rad': float(np.max(np.abs(v))), 'mean_rad': float(np.mean(v))} if v else None
        per_joint[n] = {'sim_minus_real': arr(d_sim_real), 'sim_minus_command': arr(d_sim_cmd), 'real_minus_command': arr(d_real_cmd), 'samples': len(d_sim_real)}
    # hands: simulated closure (inverse adapter, normalized to the source scale) vs recorded hand_state on the same scale
    hands = {}
    for side, contract in sequence['hand_contracts'].items():
        manifest = json.loads((a.package / 'manifest.json').read_text())
        act = manifest['hands'][side]['actuators']
        per_axis = {}
        for k, axis in enumerate(contract['axis_order']):
            spec = act[axis]; i = names.index(spec['joint'])
            d_sr, d_sc, d_rc = [], [], []
            for row, r in sim_by_row.items():
                o = obs_by_row.get(row)
                if o is None or sequence['rows'][row]['hands'].get(side) is None:
                    continue
                c_sim = (r['q_rad'][i] - spec['open_rad']) / (spec['closed_rad'] - spec['open_rad'])
                v_sim = contract['open_value'] + c_sim * (contract['closed_value'] - contract['open_value'])
                v_real = o['hand_state'][side][k]; v_cmd = sequence['rows'][row]['hands'][side][k]
                d_sr.append(v_sim - v_real); d_sc.append(v_sim - v_cmd); d_rc.append(v_real - v_cmd)
            f = lambda v: {'rms': float(np.sqrt(np.mean(np.square(v)))), 'max_abs': float(np.max(np.abs(v))), 'mean': float(np.mean(v))} if v else None
            per_axis[axis] = {'sim_minus_real_source_units': f(d_sr), 'sim_minus_command_source_units': f(d_sc), 'real_minus_command_source_units': f(d_rc), 'samples': len(d_sr)}
        hands[side] = per_axis
    body_rms = [v['sim_minus_real']['rms_rad'] for v in per_joint.values() if v['sim_minus_real']]
    result = {'kind': 'real_response_comparison', 'status': 'COMPARED_WITH_DECLARED_MISMATCHES', 'source': sequence['source'],
              'declared_mismatches': [sequence['source'].get('declared_retargeting'), sequence['source'].get('missing_controller_information'), sequence['source'].get('camera_information'),
                                      'simulated hand closes on air (no object): hand-state agreement after contact events is not expected',
                                      'real balance controller vs fixed pelvis: leg/waist joints are commanded identically but carry no ground reaction in the twin'],
              'timing': {'source_rate_hz': 30.0, 'physics_dt_s': metrics['physics_dt'], 'zero_order_hold': True, 'offline_replay': metrics.get('offline_replay'), 'real_time_factor_loop': metrics.get('real_time_factor_loop')},
              'body_joints': per_joint, 'body_sim_minus_real_rms_rad_median': float(np.median(body_rms)) if body_rms else None,
              'body_sim_minus_real_rms_rad_max': float(np.max(body_rms)) if body_rms else None, 'hands': hands,
              'interpretation': 'sim_minus_command isolates the twin controller surrogate; real_minus_command isolates the real robot tracking; sim_minus_real is their difference and inherits every declared mismatch. No parameter was fitted.'}
    (a.run / 'comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    worst = sorted(per_joint.items(), key=lambda kv: -(kv[1]['sim_minus_real']['rms_rad'] if kv[1]['sim_minus_real'] else 0))[:6]
    print(json.dumps({'status': result['status'], 'median_rms_rad': result['body_sim_minus_real_rms_rad_median'], 'max_rms_rad': result['body_sim_minus_real_rms_rad_max'],
                      'worst_joints': [(n, round(v['sim_minus_real']['rms_rad'], 4), round(v['real_minus_command']['rms_rad'], 4), round(v['sim_minus_command']['rms_rad'], 4)) for n, v in worst],
                      'hands': {s: {ax: (round(v['sim_minus_real_source_units']['rms'], 3) if v['sim_minus_real_source_units'] else None) for ax, v in h.items()} for s, h in hands.items()}}, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
