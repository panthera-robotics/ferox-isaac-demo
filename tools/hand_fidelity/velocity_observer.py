"""Deterministic finger-velocity observer/comparator (SIMULATION_DIAGNOSTIC, SHADOW mode).

For every hand joint and every recorded sample it computes, from the raw trace only:
  sample time and sequence, sampling phase (relative to world.step: the probe records the state AFTER the step),
  q[k-1], q[k], dt, dq_reported (the articulation velocity readback), dq_interval = angle_difference(q[k], q[k-1]) / dt,
  the coupled joint's expected q and dq from its driven parent under the declared mapping (q_c = A q_p + B),
  contact context (links in contact at the sample, when a contact stream is given), palm motion context,
  the LEGACY flags exactly as the existing scorer (> 1x URDF velocity field) and the existing online abort (> 2x) use them,
  and a PROSPECTIVE classification with an explanation.

The prospective decision never replaces the legacy channels: both are reported, and in shadow mode nothing acts on the
prospective one. It is also NOT a finite-difference substitute, a smoother or a two-sample debounce:
  * a sustained exceedance in EITHER channel (>= `persist` consecutive samples) is real motion or a real fault (prospective abort);
  * a single-sample reported exceedance whose interval velocity, parent (driver) interval velocity and coupling error stay
    inside their bands, coincident with a contact onset or an external body transient, is classified READBACK_IMPULSE
    (shadow: no abort) — the reported value is the solver's constraint/drive solution for that step, which the sampled
    positions do not confirm;
  * anything the single interval cannot resolve (opposing within-step motion, irregular dt, missing samples, resets,
    non-finite values, coupling faults) stays AMBIGUOUS or INVALID and keeps the legacy decision.
What one interval average cannot resolve: a within-step excursion that returns before the next sample (aliasing) looks like
a small interval velocity; the observer therefore requires persistence over `persist` samples and consistency with the
parent joint before it calls a reported peak an artifact, and it never clears a coupling fault.
"""
from __future__ import annotations

import math

import numpy as np

CLASSES = ('OK', 'SUSTAINED_MOTION', 'READBACK_IMPULSE', 'AMBIGUOUS', 'INVALID_SAMPLE', 'COUPLING_FAULT', 'INTERVAL_SPIKE')


def angle_difference(q1, q0, *, continuous=False):
    d = float(q1) - float(q0)
    if continuous:
        d = (d + math.pi) % (2 * math.pi) - math.pi
    return d


class JointSpec:
    __slots__ = ('name', 'velocity_field', 'continuous', 'parent', 'A', 'B', 'lower', 'upper')

    def __init__(self, name, velocity_field, *, continuous=False, parent=None, A=1.0, B=0.0, lower=None, upper=None):
        self.name, self.velocity_field, self.continuous, self.parent, self.A, self.B, self.lower, self.upper = name, float(velocity_field), bool(continuous), parent, float(A), float(B), lower, upper


def observe(samples, joints, *, nominal_dt, persist=2, contacts=None, palm_speed=None, dt_tolerance=0.25, coupling_tol_rad=0.03, resets=None, coupled_field='per_joint', interval_rule='single'):
    """samples: list of dicts {sequence, physics_s, phase, q: {name: rad}, dq: {name: rad/s}} in recording order.
    joints: {name: JointSpec}. contacts: optional {sequence: set(links)} (any hand link in contact at that sample).
    palm_speed: optional {sequence: m/s}. resets: optional set of sequences that are the FIRST sample after a declared
    pose write / reset: the interval ending there is not comparable (INVALID for that sample only; every later sample is
    evaluated normally, so a reset never excuses post-reset motion). coupled_field: 'per_joint' (legacy: every joint against
    its own URDF field) or 'multiplier' (a mimic child against |A| x its parent's field, the ratio-aware proposal).
    interval_rule: 'single' (one interval > 2x is real motion) or 'persist' (needs `persist` consecutive same-sign intervals
    > 2x; a lone one that the readback does NOT also see is logged as INTERVAL_SPIKE and never aborts; bounded delay =
    persist x dt). The readback persistence stays sign-agnostic: an out-and-back excursion that both channels see on two
    consecutive samples is SUSTAINED — a persistence rule may delay, never erase, a fast motion seen by both channels.
    Returns {'rows': [...], 'events': [...], 'summary': {...}}."""
    rows, events = [], []
    prev = None
    exceed_run = {n: 0 for n in joints}
    int_run = {n: [] for n in joints}          # recent signed interval velocities beyond 2x (for the persist rule)
    resets = set(resets or ())
    for k, s in enumerate(samples):
        seq, t, phase = s.get('sequence'), s.get('physics_s'), s.get('phase')
        valid_sample = isinstance(t, (int, float)) and math.isfinite(t)
        dt = None
        reset = False
        if prev is not None and valid_sample and isinstance(prev.get('physics_s'), (int, float)):
            dt = t - prev['physics_s']
            if not (dt > 0):
                reset = True                     # duplicate or out-of-order timestamp: never differentiate across it
            elif abs(dt - nominal_dt) > dt_tolerance * nominal_dt:
                reset = True                     # gap or variable step: interval velocity not comparable
            if prev.get('phase') != phase and phase in ('initialization', 'reset'):
                reset = True
        if seq in resets:
            reset = True                         # declared pose write: the interval ending here is not motion
        for name, js in joints.items():
            q = s['q'].get(name); dq_rep = s['dq'].get(name)
            row = {'joint': name, 'sequence': seq, 'physics_s': t, 'phase': phase, 'sampling_phase': 'after world.step (probe records post-step state)',
                   'q_prev': None if prev is None else prev['q'].get(name), 'q': q, 'dt': dt, 'dq_reported': dq_rep, 'dq_interval': None,
                   'parent': js.parent, 'q_expected_from_parent': None, 'dq_expected_from_parent': None, 'coupling_error_rad': None,
                   'contact_links': sorted(contacts.get(seq, ())) if contacts else None, 'palm_speed_m_s': None if palm_speed is None else palm_speed.get(seq),
                   'legacy_score_flag_1x': None, 'legacy_abort_flag_2x': None, 'interval_flag_1x': None, 'interval_flag_2x': None, 'readback_ratio': None,
                   'classification': 'OK', 'explanation': ''}
            finite = all(isinstance(v, (int, float)) and math.isfinite(v) for v in (q, dq_rep) if v is not None) and q is not None and dq_rep is not None
            if not finite or not valid_sample:
                row['classification'] = 'INVALID_SAMPLE'; row['explanation'] = 'non-finite or missing q/dq/time: no legacy flag evaluated, no differentiation'
                rows.append(row); exceed_run[name] = 0; continue
            field = js.velocity_field
            if coupled_field == 'multiplier' and js.parent is not None and js.parent in joints:
                field = abs(js.A) * joints[js.parent].velocity_field       # a child at |A| x a capped parent is not an actuator overspeed
            v1, v2 = field, 2 * field
            row['field_rad_s'] = field
            row['legacy_score_flag_1x'] = abs(dq_rep) > v1
            row['legacy_abort_flag_2x'] = abs(dq_rep) > v2
            if prev is not None and row['q_prev'] is not None and dt is not None and not reset and math.isfinite(row['q_prev']):
                dqi = angle_difference(q, row['q_prev'], continuous=js.continuous) / dt
                row['dq_interval'] = dqi
                row['interval_flag_1x'] = abs(dqi) > v1; row['interval_flag_2x'] = abs(dqi) > v2
                if interval_rule == 'persist':
                    int_run[name] = (int_run[name] + [dqi])[-persist:] if abs(dqi) > v2 else []
                if abs(dqi) > 1e-6:
                    row['readback_ratio'] = dq_rep / dqi
            if js.parent is not None and js.parent in s['q']:
                qp = s['q'][js.parent]
                row['q_expected_from_parent'] = js.A * qp + js.B
                row['coupling_error_rad'] = q - row['q_expected_from_parent']
                if prev is not None and dt is not None and not reset and js.parent in prev['q']:
                    row['dq_expected_from_parent'] = js.A * angle_difference(qp, prev['q'][js.parent], continuous=False) / dt
            # ---- prospective classification (shadow) ----
            if row['coupling_error_rad'] is not None and abs(row['coupling_error_rad']) > coupling_tol_rad:
                row['classification'] = 'COUPLING_FAULT'; row['explanation'] = 'coupled joint departs from its declared mapping by %.4f rad (> %.3f): never cleared by the velocity channels' % (row['coupling_error_rad'], coupling_tol_rad)
                exceed_run[name] += 1
            elif row['legacy_abort_flag_2x'] or (row['interval_flag_2x'] is True):
                exceed_run[name] += 1
                interval_confirms = bool(row['interval_flag_2x'])
                if interval_rule == 'persist' and interval_confirms:
                    run = int_run[name]
                    interval_confirms = len(run) >= persist and all((x > 0) == (run[0] > 0) for x in run)
                    if not interval_confirms and not row['legacy_abort_flag_2x']:
                        row['classification'] = 'INTERVAL_SPIKE'; row['explanation'] = 'one interval > 2x (%.3f rad/s) without %d consecutive same-sign exceedances: logged, never acted on (persist rule, bounded delay %.0f ms)' % (row['dq_interval'], persist, 1e3 * persist * nominal_dt)
                        rows.append(row); events.append({k2: row[k2] for k2 in ('joint', 'sequence', 'physics_s', 'dq_reported', 'dq_interval', 'dq_expected_from_parent', 'coupling_error_rad', 'contact_links', 'legacy_score_flag_1x', 'legacy_abort_flag_2x', 'classification', 'explanation')}); continue
                if row['dq_interval'] is None:
                    row['classification'] = 'AMBIGUOUS'; row['explanation'] = 'reported > 2x with no comparable interval (first sample, reset, gap or irregular dt): legacy decision kept'
                elif interval_confirms or exceed_run[name] >= persist:
                    row['classification'] = 'SUSTAINED_MOTION'; row['explanation'] = 'exceedance confirmed by the interval channel or persisting %d samples: real motion or fault' % exceed_run[name]
                elif row['interval_flag_1x'] or (row['dq_expected_from_parent'] is not None and abs(row['dq_expected_from_parent']) > v1):
                    row['classification'] = 'AMBIGUOUS'; row['explanation'] = 'reported > 2x, interval or parent between 1x and 2x: cannot be resolved by one interval; legacy decision kept'
                else:
                    ctx = []
                    if row['contact_links']:
                        ctx.append('contact on %s' % ','.join(row['contact_links'][:4]))
                    if row['palm_speed_m_s'] is not None and row['palm_speed_m_s'] > 0.05:
                        ctx.append('palm moving %.2f m/s' % row['palm_speed_m_s'])
                    row['classification'] = 'READBACK_IMPULSE'
                    row['explanation'] = 'single-sample reported %.3f rad/s vs interval %.3f rad/s (ratio %.1f), parent interval %s, coupling error %s; %s — solver readback not confirmed by the sampled positions; SHADOW: would not abort unless it persists' % (
                        dq_rep, row['dq_interval'], row['readback_ratio'] if row['readback_ratio'] is not None else float('nan'),
                        'n/a' if row['dq_expected_from_parent'] is None else '%.3f' % row['dq_expected_from_parent'], 'n/a' if row['coupling_error_rad'] is None else '%.4f' % row['coupling_error_rad'], '; '.join(ctx) or 'no recorded context')
            else:
                exceed_run[name] = 0
            if row['classification'] != 'OK':
                events.append({k2: row[k2] for k2 in ('joint', 'sequence', 'physics_s', 'dq_reported', 'dq_interval', 'dq_expected_from_parent', 'coupling_error_rad', 'contact_links', 'legacy_score_flag_1x', 'legacy_abort_flag_2x', 'classification', 'explanation')})
            rows.append(row)
        prev = s
    # ---- summary: legacy vs prospective, per channel ----
    def first(pred):
        for r in rows:
            if pred(r):
                return {'joint': r['joint'], 'sequence': r['sequence'], 'physics_s': r['physics_s'], 'dq_reported': r['dq_reported'], 'dq_interval': r['dq_interval']}
        return None
    summary = {
        'samples': len(samples), 'joints': len(joints), 'nominal_dt': nominal_dt, 'persist_samples': persist, 'coupled_field': coupled_field, 'interval_rule': interval_rule, 'declared_resets': sorted(resets), 'bounded_delay_s': persist * nominal_dt,
        'legacy_score_1x': {'reported_over_1x_samples': sum(1 for r in rows if r['legacy_score_flag_1x']), 'first': first(lambda r: r['legacy_score_flag_1x'])},
        'legacy_abort_2x': {'reported_over_2x_samples': sum(1 for r in rows if r['legacy_abort_flag_2x']), 'first': first(lambda r: r['legacy_abort_flag_2x']), 'would_abort': any(r['legacy_abort_flag_2x'] for r in rows)},
        'interval_channel': {'over_1x_samples': sum(1 for r in rows if r['interval_flag_1x']), 'over_2x_samples': sum(1 for r in rows if r['interval_flag_2x']), 'max_abs_interval_rad_s': max((abs(r['dq_interval']) for r in rows if r['dq_interval'] is not None), default=0.0),
                             'max_abs_reported_rad_s': max((abs(r['dq_reported']) for r in rows if r['dq_reported'] is not None), default=0.0)},
        'prospective_shadow': {'classes': {c: sum(1 for r in rows if r['classification'] == c) for c in CLASSES},
                               'would_abort': any(r['classification'] in ('SUSTAINED_MOTION', 'COUPLING_FAULT') for r in rows) or any(r['classification'] == 'AMBIGUOUS' and r['legacy_abort_flag_2x'] for r in rows),
                               'first_abort': first(lambda r: r['classification'] in ('SUSTAINED_MOTION', 'COUPLING_FAULT') or (r['classification'] == 'AMBIGUOUS' and r['legacy_abort_flag_2x'])),
                               'rule': 'abort on SUSTAINED_MOTION, COUPLING_FAULT, or AMBIGUOUS with a reported > 2x; READBACK_IMPULSE only logged; shadow mode: nothing acts on this decision'},
        'note': 'the interval channel is a sampled-position bound, not a within-step peak measurement; legacy channels remain authoritative until the runtime comparison (hand-req-02) is analysed',
    }
    return {'rows': rows, 'events': events, 'summary': summary}


def joints_from_urdf(urdf_path, side, *, velocity_default=1.0):
    """JointSpecs for one hand from the URDF (velocity field, limits, composed mimic mapping)."""
    from .coupling import coupling_table, read_joints
    js = read_joints(urdf_path); table = coupling_table(js, side + '_')
    out = {}
    for n, j in js.items():
        if not n.startswith(side + '_') or j['type'] not in ('revolute', 'continuous') or j['limit'] is None and j['type'] == 'revolute':
            continue
        if not any(tag in n for tag in ('_1_joint', '_2_joint', '_3_joint', '_4_joint')):
            continue
        row = table.get(n)
        out[n] = JointSpec(n, j['velocity'] or velocity_default, continuous=(j['type'] == 'continuous'), parent=row['driver'] if row else None, A=row['composed_multiplier'] if row else 1.0, B=row['composed_offset'] if row else 0.0,
                           lower=j['limit'][0] if j['limit'] else None, upper=j['limit'][1] if j['limit'] else None)
    return out


def samples_from_state_jsonl(path, side, *, max_rows=None):
    import json
    out = []
    with open(path) as f:
        for i, line in enumerate(f):
            if max_rows is not None and i >= max_rows:
                break
            r = json.loads(line)
            names = r['runtime_names']
            idx = [(n, j) for j, n in enumerate(names) if n.startswith(side + '_') and any(tag in n for tag in ('_1_joint', '_2_joint', '_3_joint', '_4_joint'))]
            out.append({'sequence': r.get('sequence'), 'physics_s': r.get('physics_s'), 'phase': r.get('phase'), 'q': {n: r['q_rad'][j] for n, j in idx}, 'dq': {n: r['dq_rad_s'][j] for n, j in idx},
                        'palm': (r.get('link_poses_world_xyzw') or {}).get(side + '_base_link')})
    return out


def contacts_by_sequence(contacts_path, side, sequences_by_time, *, bin_s=0.005):
    """{sequence: set(hand links in contact with anything)} streamed from contacts.jsonl, matched by physics time."""
    import json
    times = sorted(sequences_by_time.items())
    out = {}
    with open(contacts_path) as f:
        for line in f:
            if side + '_' not in line:
                continue
            r = json.loads(line)
            t = r.get('physics_s')
            if t is None:
                continue
            key = round(math.floor(float(t) / bin_s) * bin_s, 6)
            seq = sequences_by_time.get(key)
            if seq is None:
                continue
            for a in (r['actor0'], r['actor1']):
                if side + '_' in a:
                    out.setdefault(seq, set()).add(a.rsplit('/', 1)[-1])
    return out
