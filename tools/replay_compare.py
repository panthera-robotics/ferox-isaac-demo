#!/usr/bin/env python3
"""Compare a command-replay run with the recording's observed responses at matched source times (v2).

Host CPU, post hoc. Mode 3 (real-response comparison) is only meaningful where physical provenance,
observations, timing and controller semantics support it; this tool reports, per joint and per hand
axis, the discrepancy between the simulated feedback interpolated at each source observation time and
the recorded readback at that time, plus each one's discrepancy against the command active at that
instant (last row issued strictly before it) — and lists the declared mismatches verbatim from the
source contract. It never adjusts masses, gains or friction and fits no time shift.

v1 (Sprint G) paired the LAST simulator sample of each held row with that row's observation taken at
the row START, an offset of one source interval; it is kept only as ``compare_v1_last_sample`` for
regression tests. Output: comparison_v2.json in the run directory (v1 files are left untouched).
Usage: replay_compare.py --run <evidence dir> --package <replay package dir>
"""
import argparse
import hashlib
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from replay_alignment import AlignmentError, active_row_before, align_series, diagnostic_lag_scan, stats  # noqa: E402

COMPARATOR_VERSION = 'replay_compare_v2_2026-09-17'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def compare_v1_last_sample(row_times, obs_values, sim_rows):
    """Sprint G pairing (defective): last sim sample per source row vs that row's initial observation.
    sim_rows: list of (source_row_index, value). Returns list of sim-real diffs."""
    last = {}
    for row, v in sim_rows:
        last[row] = v
    return [last[i] - obs_values[i] for i in range(len(row_times)) if i in last]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run', type=Path, required=True); ap.add_argument('--package', type=Path, required=True)
    ap.add_argument('--max-gap-s', type=float, default=0.02, help='largest simulator sample gap bridged by interpolation (4 physics steps)')
    a = ap.parse_args(argv)
    sequence = json.loads((a.package / 'sequence.json').read_text())
    metrics = json.loads((a.run / 'metrics.json').read_text())
    probe_cfg = json.loads((a.package / 'probe-config.json').read_text())
    lead_in = float(metrics.get('lead_in_s', probe_cfg.get('lead_in_s', 0.0)))
    observed = json.loads((a.package / 'observed.json').read_text()) if (a.package / 'observed.json').exists() else None
    provenance = {'comparator': COMPARATOR_VERSION, 'run': str(a.run), 'package': str(a.package), 'computed_utc': datetime.now(timezone.utc).isoformat(),
                  'hashes': {n: sha(a.package / n) for n in ('manifest.json', 'sequence.json', 'controller.json', 'package.json') if (a.package / n).exists()},
                  'trace_hashes': {n: sha(a.run / n) for n in ('state.jsonl', 'commands.jsonl', 'metrics.json') if (a.run / n).exists()},
                  'timeline': {'source_time': 'segment seconds t_s (LeRobot frame timestamp - segment start)', 'sim_source_time': 'physics_s - lead_in_s (post-step samples)',
                               'comparison_instant': 'source observation time', 'interpolation': 'linear between bracketing post-step samples, gap <= max_gap_s, never extrapolated',
                               'active_command': 'last row issued strictly before the instant (zero-order hold)', 'lead_in_s': lead_in, 'max_gap_s': a.max_gap_s, 'fitted_time_shift': None}}
    if observed is None:
        result = {'kind': 'real_response_comparison', 'status': 'NOT_TESTED', 'reason': 'source carries no observed responses (synthetic or command-only source)', 'source': sequence['source'], 'provenance': provenance}
        (a.run / 'comparison_v2.json').write_text(json.dumps(result, indent=2) + '\n'); print(json.dumps({'status': 'NOT_TESTED'})); return 0
    provenance['hashes']['observed.json'] = sha(a.package / 'observed.json')
    state = [json.loads(l) for l in (a.run / 'state.jsonl').open()]
    if not state:
        raise SystemExit('empty state trace')
    names = state[0]['runtime_names']; body_names = state[0]['body_command_names']
    sim_t = [r['physics_s'] for r in state]
    row_times = [row['t_s'] for row in sequence['rows']]
    obs_times = [o['t_s'] for o in observed]
    if len(observed) != len(sequence['rows']) or any(abs(a_ - b) > 1e-9 for a_, b in zip(obs_times, row_times)):
        raise SystemExit('observed rows must align one-to-one with the command rows (same t_s)')
    per_joint, missing_total = {}, 0
    for n in body_names:
        i = names.index(n)
        sim_v = [r['q_rad'][i] for r in state]
        obs_v = [o['robot_q_current_rad'][n] for o in observed]
        cmd_v = [row['body_q_rad'][n] if row['body_q_rad'] and n in row['body_q_rad'] else float('nan') for row in sequence['rows']]
        try:
            al = align_series(source_times=obs_times, source_values=obs_v, sim_times=sim_t, sim_values=sim_v, lead_in_s=lead_in, max_gap_s=a.max_gap_s,
                              command_row_times=row_times, command_values=cmd_v)
        except AlignmentError as exc:
            raise SystemExit('alignment refused for %s: %s' % (n, exc))
        pairs = [p for p in al['pairs'] if p['cmd_active'] is not None and math.isfinite(p['cmd_active'])]
        per_joint[n] = {'sim_minus_real_rad': stats([p['sim'] - p['real'] for p in al['pairs']]), 'sim_minus_command_rad': stats([p['sim'] - p['cmd_active'] for p in pairs]),
                        'real_minus_command_rad': stats([p['real'] - p['cmd_active'] for p in pairs]), 'matched': al['matched'], 'missing_instants': len(al['missing']),
                        'time_residual_max_s': al['time_residual_max_s']}
        missing_total = max(missing_total, len(al['missing']))
    hands = {}
    manifest = json.loads((a.package / 'manifest.json').read_text())
    for side, contract in sequence['hand_contracts'].items():
        act = manifest['hands'][side]['actuators']; per_axis = {}
        for k, axis in enumerate(contract['axis_order']):
            spec = act[axis]; i = names.index(spec['joint'])
            span = contract['closed_value'] - contract['open_value']
            sim_v = [contract['open_value'] + (r['q_rad'][i] - spec['open_rad']) / (spec['closed_rad'] - spec['open_rad']) * span for r in state]
            obs_v = [o['hand_state'][side][k] for o in observed]
            cmd_v = [row['hands'][side][k] if row['hands'].get(side) is not None else float('nan') for row in sequence['rows']]
            al = align_series(source_times=obs_times, source_values=obs_v, sim_times=sim_t, sim_values=sim_v, lead_in_s=lead_in, max_gap_s=a.max_gap_s,
                              command_row_times=row_times, command_values=cmd_v)
            pairs = [p for p in al['pairs'] if p['cmd_active'] is not None and math.isfinite(p['cmd_active'])]
            per_axis[axis] = {'sim_minus_real_source_units': stats([p['sim'] - p['real'] for p in al['pairs']]), 'sim_minus_command_source_units': stats([p['sim'] - p['cmd_active'] for p in pairs]),
                              'real_minus_command_source_units': stats([p['real'] - p['cmd_active'] for p in pairs]), 'matched': al['matched'], 'missing_instants': len(al['missing'])}
        hands[side] = per_axis
    # diagnostic lag scan on the arm joints only (labelled diagnostic; not used above)
    arm = [n for n in body_names if any(s in n for s in ('shoulder', 'elbow', 'wrist'))]
    lag_scan = {}
    for n in arm[:14]:
        i = names.index(n)
        lag_scan[n] = diagnostic_lag_scan(source_times=obs_times, source_values=[o['robot_q_current_rad'][n] for o in observed], sim_times=sim_t, sim_values=[r['q_rad'][i] for r in state],
                                          lead_in_s=lead_in, max_gap_s=a.max_gap_s, lags_s=[-0.1, -0.05, -0.033, 0.0, 0.033, 0.05, 0.1])
    body_rms = [v['sim_minus_real_rad']['rms'] for v in per_joint.values() if v['sim_minus_real_rad']]
    executed = metrics.get('rows_applied'); total = metrics.get('rows_total')
    result = {'kind': 'real_response_comparison', 'version': COMPARATOR_VERSION, 'status': 'COMPARED_WITH_DECLARED_MISMATCHES' if body_rms else 'NO_MATCHED_INSTANTS',
              'scope': 'trustworthy discrepancy measurement at matched source instants; NOT a fidelity PASS (fixed support, no object contact, unknown real-controller semantics)',
              'source': sequence['source'], 'rows_executed': executed, 'rows_total': total, 'run_status': metrics.get('status'), 'abort': metrics.get('abort'),
              'declared_mismatches': [sequence['source'].get('declared_retargeting'), sequence['source'].get('missing_controller_information'), sequence['source'].get('camera_information'),
                                      'simulated hand closes on air (no object): hand-state agreement after contact events is not expected',
                                      'real balance controller vs fixed pelvis: leg/waist joints are commanded identically but carry no ground reaction in the twin'],
              'units': {'body': 'rad', 'hands': 'source units (dataset 0..1 scale)'}, 'body_joints': per_joint, 'hands': hands,
              'body_sim_minus_real_rms_rad_median': sorted(body_rms)[len(body_rms) // 2] if body_rms else None, 'body_sim_minus_real_rms_rad_max': max(body_rms) if body_rms else None,
              'missing_instants_max_per_joint': missing_total, 'diagnostic_lag_scan_arm_joints': {'note': 'DIAGNOSTIC ONLY: RMS(sim(t+lag) - real(t)); not a latency measurement; the primary metric above uses lag 0', 'values': lag_scan},
              'interpretation': 'sim_minus_command isolates the twin controller surrogate; real_minus_command isolates the real robot against its recorded desired targets; sim_minus_real is their difference and inherits every declared mismatch. No parameter or time shift was fitted.',
              'provenance': provenance}
    (a.run / 'comparison_v2.json').write_text(json.dumps(result, indent=2) + '\n')
    worst = sorted(per_joint.items(), key=lambda kv: -(kv[1]['sim_minus_real_rad']['rms'] if kv[1]['sim_minus_real_rad'] else 0))[:6]
    print(json.dumps({'status': result['status'], 'rows_executed': executed, 'rows_total': total, 'median_rms_rad': result['body_sim_minus_real_rms_rad_median'], 'max_rms_rad': result['body_sim_minus_real_rms_rad_max'],
                      'worst_joints (sim-real, real-cmd, sim-cmd rms)': [(n, round(v['sim_minus_real_rad']['rms'], 4), round(v['real_minus_command_rad']['rms'], 4) if v['real_minus_command_rad'] else None, round(v['sim_minus_command_rad']['rms'], 4) if v['sim_minus_command_rad'] else None) for n, v in worst],
                      'hands_sim_minus_real_rms': {s: {ax: (round(v['sim_minus_real_source_units']['rms'], 3) if v['sim_minus_real_source_units'] else None) for ax, v in h.items()} for s, h in hands.items()}}, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
