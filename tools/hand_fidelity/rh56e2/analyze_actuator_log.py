#!/usr/bin/env python3
"""Summarise an actuator_logger JSONL log: per-axis readback range, command->readback direction, deadband estimate, step
response (time to 10 %/90 % of the step, settle within +-5 counts), achieved sample rate, error codes seen. Reports
NOT_MEASURED for anything the log does not contain (a dry-run log yields no numbers). Never extrapolates.
usage: analyze_actuator_log.py <log.jsonl> [--out summary.json]"""
from __future__ import annotations

import argparse
import json
import sys

from .register_map import NATIVE_ORDER


def load(path):
    recs = [json.loads(l) for l in open(path)]
    hdr = next((r for r in recs if r.get('kind') == 'header'), {}); return hdr, recs


def series(recs, group):
    out = []
    for r in recs:
        if r.get('kind') == 'read' and r.get('group') == group and r.get('decoded'):
            out.append((r['mono_s'], r['decoded']))
    return out


def analyze(hdr, recs):
    res = {'schema': hdr.get('schema'), 'side': hdr.get('side'), 'hardware_model': hdr.get('hardware_model'), 'transport': hdr.get('transport'), 'dry_run': hdr.get('dry_run'),
           'values_are_installed_measurements': hdr.get('values_are_installed_measurements', False), 'axes': {}, 'sample_rate_hz': None, 'error_codes_seen': {}}
    ang = series(recs, 'angle_actual'); pos = series(recs, 'actuator_position_actual')
    if len(ang) >= 2:
        res['sample_rate_hz'] = round((len(ang) - 1) / (ang[-1][0] - ang[0][0]), 2)
    cmds = [(r['mono_s'], r['values']) for r in recs if r.get('kind') == 'write' and r.get('group') == 'angle_set' and r.get('sent')]
    for ax in NATIVE_ORDER:
        a = {'angle_actual_min_max': None, 'actuator_position_min_max': None, 'direction': 'NOT_MEASURED', 'deadband_counts': 'NOT_MEASURED', 'steps': []}
        va = [d[ax] for _, d in ang if ax in d]; vp = [d[ax] for _, d in pos if ax in d]
        if va: a['angle_actual_min_max'] = [min(va), max(va)]
        if vp: a['actuator_position_min_max'] = [min(vp), max(vp)]
        # steps: each sent command with a non -1 value for this axis
        my = [(t, v[ax]) for t, v in cmds if v.get(ax, -1) != -1]
        for k, (tc, target) in enumerate(my):
            t_end = my[k + 1][0] if k + 1 < len(my) else float('inf')
            before = [(t, d[ax]) for t, d in ang if t <= tc and ax in d][-3:]; after = [(t, d[ax]) for t, d in ang if tc < t < t_end and ax in d]
            if not before or not after: continue
            y0 = before[-1][1]; yf = after[-1][1]; dy = yf - y0
            st = {'t_cmd_s': round(tc, 3), 'target_counts': target, 'readback_before': y0, 'readback_final': yf, 'delta_counts': dy, 'samples': len(after)}
            if abs(dy) >= 5:
                t10 = next((t for t, y in after if abs(y - y0) >= 0.1 * abs(dy)), None); t90 = next((t for t, y in after if abs(y - y0) >= 0.9 * abs(dy)), None)
                settle = next((t for i, (t, y) in enumerate(after) if all(abs(yy - yf) <= 5 for _, yy in after[i:])), None)
                st.update({'t10_s': round(t10 - tc, 3) if t10 else None, 't90_s': round(t90 - tc, 3) if t90 else None, 'settle_5counts_s': round(settle - tc, 3) if settle else None})
                st['direction_sign'] = 1 if (target - y0) * dy > 0 else -1     # +1: readback moves toward the commanded value in the same count sense
            a['steps'].append(st)
        signs = [s['direction_sign'] for s in a['steps'] if 'direction_sign' in s]
        if signs:
            a['direction'] = 'readback_follows_command_counts' if all(s == 1 for s in signs) else 'readback_opposes_command_counts' if all(s == -1 for s in signs) else 'INCONSISTENT'
        small = [abs(s['target_counts'] - s['readback_before']) for s in a['steps'] if abs(s['delta_counts']) >= 1]
        nomove = [abs(s['target_counts'] - s['readback_before']) for s in a['steps'] if abs(s['delta_counts']) < 1 and s['samples'] >= 3]
        if small:
            a['deadband_counts'] = {'largest_command_change_without_motion': max(nomove) if nomove else 0, 'smallest_command_change_with_motion': min(small), 'note': 'deadband lies between these two bounds; refine with smaller steps'}
        res['axes'][ax] = a
    for r in recs:
        if r.get('kind') == 'read' and r.get('group') == 'actuator_error_code' and r.get('decoded'):
            for ax, code in r['decoded'].items():
                if code: res['error_codes_seen'].setdefault(ax, set()).add(code)
    res['error_codes_seen'] = {k: sorted(v) for k, v in res['error_codes_seen'].items()}
    if not ang:
        res['verdict'] = 'NOT_MEASURED: no decoded angle_actual readbacks in this log' + (' (dry run)' if hdr.get('dry_run') else '')
    else:
        res['verdict'] = 'readbacks present; direction/deadband/step values above are per this log only'
    return res


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('log'); p.add_argument('--out'); a = p.parse_args(argv)
    hdr, recs = load(a.log); res = analyze(hdr, recs); s = json.dumps(res, indent=1)
    if a.out: open(a.out, 'w').write(s)
    print(s)


if __name__ == '__main__':
    sys.exit(main())
