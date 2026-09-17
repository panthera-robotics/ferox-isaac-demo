"""Compare two command-replay evidence directories (A vs B) of the hand-map diagnostic: commanded vs measured hand joints,
coupled-joint error, fingertip positions in the palm frame (FK of the measured hand joints), hand-object contact sets,
object pose relative to the palm, and the frozen evaluator verdicts. CPU only; reads immutable traces; writes nothing
into them.

Inputs per run: state.jsonl (q_rad, hand_command_rad, coupling_error_rad, link poses), commands.jsonl, object.jsonl
(optional), contacts.jsonl (optional, streamed), task_eval_v2.json (optional). Runs are aligned on `source_t_s`
(the replayed row time) — both packages replay the same rows, so the same source_t_s means the same command instant.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from .donor_profile import HandUrdf

FINGERS = ('index', 'middle', 'ring', 'little')


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_state(path, side='right'):
    t, src_t, phase, q, cmd, cerr, palm, effort, src_row = [], [], [], [], [], [], [], [], []
    names = None; hand_idx = None; cmd_idx = None; eff_idx = None
    with open(path) as f:
        for line in f:
            r = json.loads(line)
            if names is None:
                names = r['runtime_names']
                hand_idx = {n: names.index(n) for n in names if n.startswith(side + '_') and ('_1_joint' in n or '_2_joint' in n or '_3_joint' in n or '_4_joint' in n)}
                cmd_idx = {n: r['hand_command_names'].index(n) for n in r['hand_command_names'] if n.startswith(side + '_')}
                eff_idx = hand_idx
            t.append(r['physics_s']); src_t.append(r['source_t_s'] if r['source_t_s'] is not None else math.nan); phase.append(r['phase']); src_row.append(r.get('source_row'))
            q.append({n: r['q_rad'][i] for n, i in hand_idx.items()})
            cmd.append({n: r['hand_command_rad'][i] for n, i in cmd_idx.items()})
            cerr.append({k: v for k, v in r.get('coupling_error_rad', {}).items() if k.startswith(side + '_')})
            effort.append({n: r['measured_generalized_effort_nm'][i] for n, i in eff_idx.items()})
            lp = r.get('link_poses_world_xyzw', {}).get(side + '_base_link')
            palm.append(lp)
    return {'t': np.array(t), 'source_t': np.array(src_t), 'phase': phase, 'source_row': src_row, 'q': q, 'cmd': cmd, 'coupling_error': cerr, 'palm_pose': palm, 'effort': effort}


def load_object(path):
    out = []
    if not Path(path).exists():
        return None
    with open(path) as f:
        for line in f:
            r = json.loads(line)
            out.append((r['physics_s'], r['pose_world_xyzw'], r.get('right_palm_pose_world_xyzw')))
    return out


def stream_hand_object_contacts(path, side='right', bin_s=0.05):
    """{time_bin: set(hand links in contact with the object)} streamed from contacts.jsonl (large)."""
    bins = {}
    if not Path(path).exists():
        return None
    key = '/World/Scene/Object'
    with open(path) as f:
        for line in f:
            if key not in line or side + '_' not in line:
                continue
            r = json.loads(line)
            a0, a1 = r['actor0'], r['actor1']
            other = a1 if key in a0 else a0
            if side + '_' not in other:
                continue
            link = other.rsplit('/', 1)[-1]
            b = round(math.floor(float(r['physics_s']) / bin_s) * bin_s, 3)
            bins.setdefault(b, set()).add(link)
    return bins


def fingertips_palm(hand: HandUrdf, qrow):
    """Fingertip sensor frames and mesh tips in the palm (base_link) frame from measured hand joints (independent joints only;
    coupled joints follow the composed coupling in this FK — their measured values are compared separately)."""
    q = {j: qrow[j] for j in hand.independent if j in qrow}
    fr = hand.frames(q)
    out = {}
    for f in FINGERS:
        out[f + '_tip_sensor'] = fr[hand.side + '_' + f + '_force_sensor_3'][:3, 3].round(4).tolist()
    out['thumb_tip_sensor'] = fr[hand.side + '_thumb_force_sensor_4'][:3, 3].round(4).tolist()
    return out


def summarize_run(evidence_dir, hand: HandUrdf, *, side='right', source_spec=None):
    """source_spec: the replayed spec/sequence JSON (rows carry `stage`); phases are then the replayed stages, else the probe phases."""
    ev = Path(evidence_dir)
    st = load_state(ev / 'state.jsonl', side)
    if source_spec is not None:
        rows = json.loads(Path(source_spec).read_text())['rows']
        stage_of_row = {i: r.get('stage', 'row%d' % i) for i, r in enumerate(rows)}
        st['phase'] = [stage_of_row.get(sr, ph) if sr is not None else ph for sr, ph in zip(st['source_row'], st['phase'])]
    obj = load_object(ev / 'object.jsonl')
    contacts = stream_hand_object_contacts(ev / 'contacts.jsonl', side)
    te = json.loads((ev / 'task_eval_v2.json').read_text()) if (ev / 'task_eval_v2.json').exists() else None
    metrics = json.loads((ev / 'metrics.json').read_text()) if (ev / 'metrics.json').exists() else {}
    phases = st['phase']
    # per-phase summary of the measured hand joints, commands, coupling error and fingertips
    summary = {'evidence_dir': str(ev), 'samples': len(st['t']), 'phases': sorted(set(phases)), 'per_phase': {}, 'task_eval_v2': None if te is None else {k: te[k].get('pass') if isinstance(te.get(k), dict) else te.get(k) for k in ('C1_grasp', 'C2_lift_retention', 'C3_place', 'C4_release', 'C5_prohibited', 'overall')},
               'probe_status': metrics.get('status'), 'max_coupling_error_rad': 0.0, 'first_hand_object_contact_s': None, 'contact_links_in_hold': None}
    independent = [hand.actuators[a] for a in ('index', 'middle', 'ring', 'little', 'thumb_bend', 'thumb_rotation')]
    for ph in sorted(set(phases)):
        idx = [i for i, p in enumerate(phases) if p == ph]
        if not idx:
            continue
        last = idx[-1]
        summary['per_phase'][ph] = {'t_start_s': float(st['t'][idx[0]]), 't_end_s': float(st['t'][last]),
                                    'measured_end_rad': {j: round(st['q'][last][j], 4) for j in independent},
                                    'commanded_end_rad': {j: round(st['cmd'][last].get(j, math.nan), 4) for j in independent},
                                    'tracking_error_end_rad': {j: round(st['q'][last][j] - st['cmd'][last].get(j, math.nan), 4) for j in independent},
                                    'coupled_measured_end_rad': {j: round(st['q'][last][j], 4) for j in st['q'][last] if j not in independent},
                                    'max_abs_coupling_error_rad': round(max((abs(v) for i in idx for v in st['coupling_error'][i].values()), default=0.0), 5),
                                    'fingertips_palm_end_m': fingertips_palm(hand, st['q'][last]),
                                    'max_abs_hand_effort_nm': round(max(abs(v) for i in idx for v in st['effort'][i].values()), 4)}
    summary['max_coupling_error_rad'] = round(max((abs(v) for c in st['coupling_error'] for v in c.values()), default=0.0), 5)
    if contacts:
        times = sorted(contacts)
        summary['first_hand_object_contact_s'] = times[0]
        hold = [i for i, p in enumerate(phases) if p == 'hold']
        if hold:
            t0, t1 = st['t'][hold[0]], st['t'][hold[-1]]
            links = set()
            for b in times:
                if t0 <= b <= t1:
                    links |= contacts[b]
            summary['contact_links_in_hold'] = sorted(links)
        summary['contact_link_timeline'] = {str(b): sorted(contacts[b]) for b in times[:400]}
    if obj:
        # object position in the palm frame at the end of close/hold/lower
        for ph in ('close', 'hold', 'lower'):
            idx = [i for i, p in enumerate(phases) if p == ph]
            if not idx:
                continue
            i = idx[-1]; tphys = st['t'][i]
            k = min(range(len(obj)), key=lambda j: abs(obj[j][0] - tphys))
            tp, pose, palm = obj[k]
            if palm is None:
                continue
            R = quat_to_R(palm[3:]); p = np.array(pose[:3]) - np.array(palm[:3])
            summary['per_phase'][ph]['object_in_palm_m'] = (R.T @ p).round(4).tolist()
    return summary


def compare(summary_a, summary_b):
    """Differences B - A at matching phases; the C-verdicts side by side."""
    out = {'phases': {}, 'task_eval': {'A': summary_a.get('task_eval_v2'), 'B': summary_b.get('task_eval_v2')},
           'first_contact_s': {'A': summary_a.get('first_hand_object_contact_s'), 'B': summary_b.get('first_hand_object_contact_s')},
           'contact_links_in_hold': {'A': summary_a.get('contact_links_in_hold'), 'B': summary_b.get('contact_links_in_hold')},
           'max_coupling_error_rad': {'A': summary_a['max_coupling_error_rad'], 'B': summary_b['max_coupling_error_rad']}}
    for ph in sorted(set(summary_a['per_phase']) & set(summary_b['per_phase'])):
        a, b = summary_a['per_phase'][ph], summary_b['per_phase'][ph]
        d = {'measured_end_rad_B_minus_A': {j: round(b['measured_end_rad'][j] - a['measured_end_rad'][j], 4) for j in a['measured_end_rad']},
             'commanded_end_rad_B_minus_A': {j: round(b['commanded_end_rad'][j] - a['commanded_end_rad'][j], 4) for j in a['commanded_end_rad']},
             'fingertip_B_minus_A_m': {k: (np.array(b['fingertips_palm_end_m'][k]) - np.array(a['fingertips_palm_end_m'][k])).round(4).tolist() for k in a['fingertips_palm_end_m']}}
        if 'object_in_palm_m' in a and 'object_in_palm_m' in b:
            d['object_in_palm_B_minus_A_m'] = (np.array(b['object_in_palm_m']) - np.array(a['object_in_palm_m'])).round(4).tolist()
        out['phases'][ph] = d
    return out
