"""SIMULATION_OPERATIONAL_OBSERVER v1 — the hand/joint velocity guard for NEW simulation runs (Sprint O owner decision A).

Why it exists: the legacy guards judge the REPORTED joint velocity of every joint against its own URDF velocity field. On the
provisional donor the mimic children (`*_2`, thumb_3, thumb_4) are geared 1.0843x / 0.8024x / 0.9487x to a capped parent, and
PhysX reports solver readbacks on them that the sampled positions never show (K/L/M/N: "parent pinned at 1.000, child readback
2-10 rad/s, positions static"). Sprint M validated the prospective rule on an isolated diagnostic asset with positive and negative
controls (20/20 real excursions detected on both physics steps, 0 false events vs 3 for the legacy-equivalent rule).

Rule (per sample, per joint):
  * field       = the joint's URDF velocity field; for a mimic child |composed multiplier| x its root parent's field.
  * v_interval  = (q - q_prev) / dt  (sampled positions; the authoritative channel). No decision on the first sample, after a
                  declared reset, or across an irregular dt (|dt - nominal| > 25 %).
  * ABORT when |v_interval| > threshold_multiple x field on `persist` consecutive samples of the same sign, or when a mimic child
    departs from its declared mapping by more than coupling_tol_rad (a coupling fault is never cleared by a velocity channel).
  * The reported velocity is never acted on; it is scored in the shadow (`legacy_shadow`: over-1x / over-2x joints and whether the
    legacy rule would have aborted) so every historical verdict stays comparable. No hardware implication: SIM-only, donor twin.
"""
from __future__ import annotations

import math

LEGACY = 'legacy'
OPERATIONAL_V1 = 'simulation_operational_v1'
OBSERVERS = (LEGACY, OPERATIONAL_V1)


def composed_fields(joint_limits, mimic_map):
    """{joint: field} with mimic children at |composed multiplier| x their root parent's URDF field (thumb_4 <- thumb_3 <- thumb_2)."""
    fields = {n: float(l['velocity']) for n, l in joint_limits.items()}
    for child in mimic_map:
        mult, parent, seen = 1.0, child, set()
        while parent in mimic_map and parent not in seen:
            seen.add(parent); m = mimic_map[parent]; mult *= float(m['multiplier']); parent = m['parent']
        if parent in fields:
            fields[child] = abs(mult) * fields[parent]
    return fields


class OperationalObserver:
    """Streaming SIMULATION_OPERATIONAL_OBSERVER v1 over the full joint state of a probe. `update()` returns the decision for the
    sample; the caller aborts on decision['abort'] and writes decision['legacy_shadow'] to its trace/metrics."""

    def __init__(self, joint_limits, mimic_map, nominal_dt, *, threshold_multiple=2.0, persist=2, coupling_tol_rad=0.03, dt_tolerance=0.25, reported_cap=None):
        if threshold_multiple <= 0 or persist < 1 or nominal_dt <= 0:
            raise ValueError('observer parameters must be positive')
        self.limits = joint_limits; self.mimic = dict(mimic_map or {}); self.dt = float(nominal_dt); self.k = float(threshold_multiple)
        self.persist = int(persist); self.tol = float(coupling_tol_rad); self.dt_tol = float(dt_tolerance); self.cap = reported_cap
        self.fields = composed_fields(joint_limits, self.mimic)
        self.prev_q = None; self.prev_t = None; self.runs = {}; self.samples = 0
        self.summary = {'observer': OPERATIONAL_V1, 'threshold_multiple': self.k, 'persist_samples': self.persist, 'coupled_field': 'multiplier', 'samples': 0, 'interval_over_threshold_samples': 0, 'coupling_faults': 0,
                        'max_abs_interval_rad_s': 0.0, 'max_abs_reported_rad_s': 0.0, 'legacy_shadow': {'over_1x_samples': 0, 'over_2x_samples': 0, 'would_abort_1x': False, 'would_abort_2x': False, 'first_1x': None, 'first_2x': None}}

    def reset(self):
        """Declared pose write / reset: the next interval is not comparable."""
        self.prev_q = None; self.prev_t = None; self.runs = {}

    def _legacy_field(self, name):
        f = float(self.limits[name]['velocity']); return min(self.cap, f) if self.cap is not None else f

    def update(self, sequence, t, q_by_name, dq_reported_by_name):
        self.samples += 1; self.summary['samples'] = self.samples
        shadow = {'over_1x': {}, 'over_2x': {}}
        for n, v in dq_reported_by_name.items():
            if n not in self.limits or not math.isfinite(v):
                continue
            f = self._legacy_field(n); self.summary['max_abs_reported_rad_s'] = max(self.summary['max_abs_reported_rad_s'], abs(v))
            if abs(v) > f: shadow['over_1x'][n] = float(v)
            if abs(v) > 2 * f: shadow['over_2x'][n] = float(v)
        ls = self.summary['legacy_shadow']
        if shadow['over_1x']:
            ls['over_1x_samples'] += 1; ls['would_abort_1x'] = True
            if ls['first_1x'] is None: j = next(iter(shadow['over_1x'])); ls['first_1x'] = {'sequence': sequence, 'physics_s': t, 'joint': j, 'dq_reported': shadow['over_1x'][j]}
        if shadow['over_2x']:
            ls['over_2x_samples'] += 1; ls['would_abort_2x'] = True
            if ls['first_2x'] is None: j = next(iter(shadow['over_2x'])); ls['first_2x'] = {'sequence': sequence, 'physics_s': t, 'joint': j, 'dq_reported': shadow['over_2x'][j]}
        decision = {'sequence': sequence, 'physics_s': t, 'abort': False, 'reason': None, 'violations': {}, 'coupling_faults': {}, 'interval_velocity': {}, 'legacy_shadow': shadow, 'comparable': False}
        # coupling faults (position channel, always evaluated)
        for child, m in self.mimic.items():
            if child in q_by_name and m['parent'] in q_by_name:
                err = q_by_name[child] - (float(m['multiplier']) * q_by_name[m['parent']] + float(m.get('offset', 0.0)))
                if abs(err) > self.tol: decision['coupling_faults'][child] = float(err)
        if decision['coupling_faults']:
            self.summary['coupling_faults'] += 1; decision['abort'] = True; decision['reason'] = 'coupling_fault'
        comparable = self.prev_q is not None and self.prev_t is not None and t > self.prev_t and abs((t - self.prev_t) - self.dt) <= self.dt_tol * self.dt
        if comparable:
            dt = t - self.prev_t; decision['comparable'] = True
            for n, q in q_by_name.items():
                if n not in self.fields or n not in self.prev_q or not (math.isfinite(q) and math.isfinite(self.prev_q[n])):
                    continue
                v = (q - self.prev_q[n]) / dt; decision['interval_velocity'][n] = float(v)
                self.summary['max_abs_interval_rad_s'] = max(self.summary['max_abs_interval_rad_s'], abs(v))
                thr = self.k * self.fields[n]
                if abs(v) > thr:
                    run = self.runs.get(n, [])
                    run = (run + [v])[-self.persist:] if (not run or (run[-1] > 0) == (v > 0)) else [v]
                    self.runs[n] = run
                    if len(run) >= self.persist:
                        decision['violations'][n] = float(v)
                else:
                    self.runs[n] = []
            if decision['violations']:
                self.summary['interval_over_threshold_samples'] += 1
                if not decision['abort']: decision['abort'] = True; decision['reason'] = 'sampled_interval_overspeed'
        else:
            self.runs = {}
        self.prev_q = dict(q_by_name); self.prev_t = t
        return decision


def make_observer(name, joint_limits, mimic_map, nominal_dt, **kw):
    """`legacy` -> None (the caller keeps its historical rule); `simulation_operational_v1` -> OperationalObserver."""
    if name not in OBSERVERS:
        raise ValueError('observer must be one of %s' % (OBSERVERS,))
    return None if name == LEGACY else OperationalObserver(joint_limits, mimic_map, nominal_dt, **kw)
