"""CPU tests for matched-time replay comparison (H1b): reproduces the Sprint G pairing defect and pins the v2 rules."""
import math
import unittest

from tools.replay_alignment import AlignmentError, active_row_before, align_series, diagnostic_lag_scan, interpolate, stats, validate_monotonic
from tools.replay_compare import compare_v1_last_sample

DT = 0.005
LEAD = 1.5


def trajectory(t):
    return 1.0 * t   # 1 rad/s exact motion


def source(n=60, rate=30.0):
    times = [i / rate for i in range(n)]
    return times, [trajectory(t) for t in times]


def sim(duration, delay=0.0, dt=DT, lead=LEAD):
    """Post-step samples at physics time (tick+1)*dt following the trajectory in source time, optionally delayed."""
    times, values = [], []
    k = 1
    while (k * dt) <= lead + duration + 1e-9:
        p = k * dt; ts = p - lead
        times.append(p); values.append(trajectory(max(0.0, ts - delay)) if ts >= 0 else 0.0)
        k += 1
    return times, values


class V1DefectReproduction(unittest.TestCase):
    def test_last_sample_pairing_shows_bias_on_exact_match(self):
        st, sv = source(); pt, pv = sim(st[-1] + 1 / 30)
        sim_rows = []
        for p, v in zip(pt, pv):
            ts = p - LEAD
            if ts < 0:
                continue
            row = max(i for i, rt in enumerate(st) if rt <= ts + 1e-12) if ts >= st[0] else None
            if row is not None:
                sim_rows.append((row, v))
        diffs = compare_v1_last_sample(st, sv, sim_rows)
        rms = math.sqrt(sum(d * d for d in diffs) / len(diffs))
        self.assertGreater(rms, 0.025); self.assertLess(rms, 0.04)   # ≈ one source interval of 1 rad/s motion (review: 0.0317 rad)


class AlignmentTests(unittest.TestCase):
    def test_exact_match_is_near_zero(self):
        st, sv = source(); pt, pv = sim(st[-1] + DT)   # the probe runs one extra step past the last row
        al = align_series(source_times=st, source_values=sv, sim_times=pt, sim_values=pv, lead_in_s=LEAD, max_gap_s=0.02)
        self.assertEqual(al['matched'], len(st)); self.assertEqual(al['missing'], [])
        s = stats([p['sim'] - p['real'] for p in al['pairs']])
        self.assertLess(s['max_abs'], 1e-9); self.assertLessEqual(al['time_residual_max_s'], DT)

    def test_known_delay_remains_in_primary_metric(self):
        st, sv = source(); pt, pv = sim(st[-1] + DT, delay=0.05)
        al = align_series(source_times=st, source_values=sv, sim_times=pt, sim_values=pv, lead_in_s=LEAD, max_gap_s=0.02)
        s = stats([p['sim'] - p['real'] for p in al['pairs'][2:]])
        self.assertAlmostEqual(s['mean'], -0.05, places=6)
        scan = diagnostic_lag_scan(source_times=st, source_values=sv, sim_times=pt, sim_values=pv, lead_in_s=LEAD, max_gap_s=0.02, lags_s=[0.0, 0.05])
        self.assertGreater(scan[0]['rms'], scan[1]['rms'])   # diagnostic sees the lag; the primary metric above did not shift

    def test_nonuniform_timestamps_and_row_boundaries(self):
        st = [0.0, 0.03, 0.07, 0.08, 0.15]; sv = [trajectory(t) for t in st]
        pt, pv = sim(0.2)
        al = align_series(source_times=st, source_values=sv, sim_times=pt, sim_values=pv, lead_in_s=LEAD, max_gap_s=0.02,
                          command_row_times=st, command_values=[10, 20, 30, 40, 50])
        self.assertEqual(al['matched'], 5)
        self.assertEqual([p['cmd_active'] for p in al['pairs']], [None, 10, 20, 30, 40])   # row issued at the instant has not acted yet
        self.assertLess(max(abs(p['sim'] - p['real']) for p in al['pairs']), 1e-9)

    def test_pre_post_step_alignment_and_lead_in(self):
        st, sv = source(5); pt, pv = sim(st[-1] + DT)
        wrong = align_series(source_times=st, source_values=sv, sim_times=pt, sim_values=pv, lead_in_s=LEAD + DT, max_gap_s=0.02)
        right = align_series(source_times=st, source_values=sv, sim_times=pt, sim_values=pv, lead_in_s=LEAD, max_gap_s=0.02)
        self.assertGreater(stats([p['sim'] - p['real'] for p in wrong['pairs']])['max_abs'], 0.004)
        self.assertLess(stats([p['sim'] - p['real'] for p in right['pairs']])['max_abs'], 1e-9)

    def test_missing_duplicate_out_of_order_refused_or_reported(self):
        st, sv = source(10); pt, pv = sim(st[-1] + DT)
        with self.assertRaises(AlignmentError):
            align_series(source_times=st, source_values=sv, sim_times=[pt[0]] + pt, sim_values=[pv[0]] + pv, lead_in_s=LEAD, max_gap_s=0.02)
        with self.assertRaises(AlignmentError):
            align_series(source_times=st[::-1], source_values=sv[::-1], sim_times=pt, sim_values=pv, lead_in_s=LEAD, max_gap_s=0.02)
        with self.assertRaises(AlignmentError):
            align_series(source_times=st, source_values=sv, sim_times=pt, sim_values=pv[:-1], lead_in_s=LEAD, max_gap_s=0.02)
        with self.assertRaises(AlignmentError):
            validate_monotonic([0.0, float('nan')], 'x')
        # a hole in the simulator trace larger than the bounded gap is reported missing, not bridged
        cut_t = [p for p in pt if not (LEAD + 0.10 < p < LEAD + 0.20)]; cut_v = [v for p, v in zip(pt, pv) if not (LEAD + 0.10 < p < LEAD + 0.20)]
        al = align_series(source_times=st, source_values=sv, sim_times=cut_t, sim_values=cut_v, lead_in_s=LEAD, max_gap_s=0.02)
        self.assertEqual(al['missing'], [4, 5]); self.assertEqual(al['matched'], 8)

    def test_partial_failed_run_and_no_observations(self):
        st, sv = source(60); pt, pv = sim(1.0)   # trace ends after 1.0 s of a 2.0 s segment
        al = align_series(source_times=st, source_values=sv, sim_times=pt, sim_values=pv, lead_in_s=LEAD, max_gap_s=0.02)
        self.assertEqual(al['matched'], 31); self.assertEqual(al['missing'], list(range(31, 60)))   # never extrapolated
        self.assertIsNone(interpolate([], [], 0.0, max_gap_s=0.02))
        self.assertIsNone(stats([]))
        self.assertIsNone(active_row_before([0.1, 0.2], 0.05)); self.assertEqual(active_row_before([0.1, 0.2], 0.2), 0)


if __name__ == '__main__':
    unittest.main()
