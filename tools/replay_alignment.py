"""Time alignment for command-replay comparisons (pure functions, CPU, tested).

Timeline conventions (all in seconds):
* Source rows carry ``t_s`` = source observation/command timestamp relative to the segment start. In a
  LeRobot frame the observation and the action share the frame timestamp: the observation is the
  state at that instant, the action is the command issued at that instant (it cannot have acted yet).
* The replay probe holds each converted row from physics time ``lead_in_s + t_s`` (zero-order hold) and
  records the state AFTER every physics step at ``physics_s`` (post-step sample). Source time of a
  simulator sample is therefore ``physics_s - lead_in_s``.
* Comparison instant = source observation time. The simulator state is linearly interpolated between
  the two post-step samples that bracket that instant (bounded gap; never extrapolated). The command
  "active" at that instant is the last row issued STRICTLY before it (ZOH), for the real robot and the
  simulator alike; the row issued at the instant itself has not acted yet.
* No time shift is fitted in the primary metric. ``diagnostic_lag_scan`` is a separate, labelled
  diagnostic and is not evidence of transport latency.
"""
from __future__ import annotations

import math


class AlignmentError(ValueError):
    pass


def _finite(v, name):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise AlignmentError('%s must be finite' % name)
    return float(v)


def validate_monotonic(times, name, *, allow_equal=False):
    """Refuse duplicate, out-of-order or non-finite timestamps."""
    last = None
    for i, t in enumerate(times):
        t = _finite(t, '%s[%d]' % (name, i))
        if last is not None and (t < last or (t == last and not allow_equal)):
            raise AlignmentError('%s not strictly increasing at index %d' % (name, i))
        last = t
    return [float(t) for t in times]


def interpolate(times, values, t, *, max_gap_s):
    """Linear interpolation of a scalar series at t between bracketing samples.

    Returns (value, residual_s) where residual_s is the distance from t to the nearest bracketing
    sample; None when t is outside the series or the bracketing gap exceeds max_gap_s.
    """
    n = len(times)
    if n == 0 or t < times[0] or t > times[-1]:
        return None
    lo, hi = 0, n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid
    if times[lo] == t:
        return float(values[lo]), 0.0
    if times[hi] == t:
        return float(values[hi]), 0.0
    gap = times[hi] - times[lo]
    if gap > max_gap_s:
        return None
    w = (t - times[lo]) / gap
    return float(values[lo] + w * (values[hi] - values[lo])), float(min(t - times[lo], times[hi] - t))


def active_row_before(row_times, t):
    """Index of the last row issued strictly before t (ZOH), or None before the first row has acted."""
    active = None
    for i, rt in enumerate(row_times):
        if rt < t:
            active = i
        else:
            break
    return active


def align_series(*, source_times, source_values, sim_times, sim_values, lead_in_s, max_gap_s, command_row_times=None, command_values=None):
    """Pair one observed source series with one simulated series at the source observation times.

    Returns dict with matched pairs (t, real, sim, cmd_active) and counts; missing instants are
    reported, never filled. sim_times are physics times (post-step); source times are segment times.
    """
    st = validate_monotonic(source_times, 'source_times')
    pt = validate_monotonic(sim_times, 'sim_times')
    if len(source_values) != len(st) or len(sim_values) != len(pt):
        raise AlignmentError('value/time length mismatch')
    lead = _finite(lead_in_s, 'lead_in_s')
    sim_source_times = [p - lead for p in pt]
    pairs, missing, residuals = [], [], []
    for i, t in enumerate(st):
        got = interpolate(sim_source_times, sim_values, t, max_gap_s=max_gap_s)
        if got is None:
            missing.append(i); continue
        v, res = got
        cmd = None
        if command_row_times is not None:
            k = active_row_before(command_row_times, t)
            cmd = None if k is None else float(command_values[k])
        pairs.append({'i': i, 't_s': t, 'real': float(source_values[i]), 'sim': v, 'cmd_active': cmd})
        residuals.append(res)
    return {'pairs': pairs, 'matched': len(pairs), 'missing': missing, 'time_residual_max_s': max(residuals) if residuals else None}


def stats(diffs):
    if not diffs:
        return None
    a = sorted(abs(d) for d in diffs)
    n = len(a)
    p95 = a[min(n - 1, max(0, math.ceil(0.95 * n) - 1))]
    return {'rms': math.sqrt(sum(d * d for d in diffs) / n), 'mean': sum(diffs) / n, 'p95_abs': p95, 'max_abs': a[-1], 'n': n}


def diagnostic_lag_scan(*, source_times, source_values, sim_times, sim_values, lead_in_s, max_gap_s, lags_s):
    """DIAGNOSTIC ONLY: RMS of sim(t + lag) - real(t) over candidate lags. Not used by the primary metric
    and not a transport-latency measurement."""
    out = []
    for lag in lags_s:
        a = align_series(source_times=source_times, source_values=source_values, sim_times=sim_times, sim_values=sim_values,
                         lead_in_s=lead_in_s + lag, max_gap_s=max_gap_s)
        s = stats([p['sim'] - p['real'] for p in a['pairs']])
        out.append({'lag_s': lag, 'rms': None if s is None else s['rms'], 'matched': a['matched']})
    return out
