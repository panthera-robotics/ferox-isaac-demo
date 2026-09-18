"""CPU corpus for the shadow finger-velocity observer: stationary, known-speed ramp, sustained excessive motion, a single
plausible impulse, an aliasing counterexample, stale/out-of-order/non-finite samples, variable dt, wrap handling, broken
coupling, commanded-vs-observed identity, a legitimate fast closure, a contact-related artifact. Legacy flags are always
computed exactly as the scorer (1x) and the online abort (2x) do; the prospective classification is shadow only."""
import math
import os
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hand_fidelity.velocity_observer import CLASSES, JointSpec, angle_difference, observe   # noqa: E402

DT = 0.005
J = {'p': JointSpec('p', 1.0, lower=0.0, upper=1.4381), 'c': JointSpec('c', 1.0, parent='p', A=1.0843, B=0.0, lower=0.0, upper=3.14)}


def make(qs_p, qs_c=None, dq_p=None, dq_c=None, dt=DT, t0=0.0, phase='replay', seq0=0):
    n = len(qs_p); qs_c = qs_c if qs_c is not None else [1.0843 * q for q in qs_p]
    dq_p = dq_p if dq_p is not None else [0.0] + [(qs_p[i] - qs_p[i - 1]) / dt for i in range(1, n)]
    dq_c = dq_c if dq_c is not None else [0.0] + [(qs_c[i] - qs_c[i - 1]) / dt for i in range(1, n)]
    return [{'sequence': seq0 + i, 'physics_s': t0 + i * dt, 'phase': phase, 'q': {'p': qs_p[i], 'c': qs_c[i]}, 'dq': {'p': dq_p[i], 'c': dq_c[i]}} for i in range(n)]


def classes(res, joint):
    return [r['classification'] for r in res['rows'] if r['joint'] == joint]


class ObserverCorpus(unittest.TestCase):
    def test_stationary(self):
        res = observe(make([0.3] * 20), J, nominal_dt=DT)
        self.assertEqual(set(classes(res, 'p')), {'OK'}); self.assertFalse(res['summary']['legacy_abort_2x']['would_abort']); self.assertFalse(res['summary']['prospective_shadow']['would_abort'])

    def test_known_speed_ramp_reported_equals_interval(self):
        qs = [0.5 * i * DT for i in range(40)]                                  # 0.5 rad/s
        res = observe(make(qs), J, nominal_dt=DT)
        rows = [r for r in res['rows'] if r['joint'] == 'p' and r['dq_interval'] is not None]
        self.assertTrue(all(abs(r['dq_interval'] - 0.5) < 1e-9 and abs(r['readback_ratio'] - 1.0) < 1e-9 for r in rows))
        self.assertEqual(set(classes(res, 'p')), {'OK'})

    def test_sustained_excessive_motion_aborts_in_both(self):
        qs = [3.0 * i * DT for i in range(20)]                                  # 3 rad/s sustained
        res = observe(make(qs), J, nominal_dt=DT)
        self.assertTrue(res['summary']['legacy_abort_2x']['would_abort']); self.assertTrue(res['summary']['prospective_shadow']['would_abort'])
        self.assertIn('SUSTAINED_MOTION', classes(res, 'p'))

    def test_single_plausible_impulse_is_readback_only_in_shadow(self):
        qs = [1.19 + 0.4 * DT * min(i, 1) for i in range(12)]                     # position moves 0.4 rad/s for one step then rests
        dq_c = [0.0] * 12; dq_c[1] = 2.233                                       # solver readback impulse on the coupled joint
        qs_c = [1.0843 * 1.19 + 0.002 * min(i, 1) for i in range(12)]
        samples = make([1.19] * 12, qs_c=qs_c, dq_c=dq_c)
        res = observe(samples, J, nominal_dt=DT, contacts={1: {'right_little_2'}})
        c = classes(res, 'c')
        self.assertEqual(c[1], 'READBACK_IMPULSE'); self.assertTrue(res['summary']['legacy_abort_2x']['would_abort']); self.assertFalse(res['summary']['prospective_shadow']['would_abort'])
        ev = [e for e in res['events'] if e['joint'] == 'c'][0]; self.assertIn('contact', ev['explanation']); self.assertTrue(ev['legacy_abort_flag_2x'])

    def test_aliasing_counterexample_persisting_reported_exceedance_aborts(self):
        # within-step back-and-forth: positions barely move, but the reported channel exceeds 2x on three consecutive samples
        qs = [0.5 + (0.001 if i % 2 else 0.0) for i in range(12)]
        dq_p = [0.0] * 12; dq_p[4] = 2.5; dq_p[5] = -2.6; dq_p[6] = 2.4
        res = observe(make(qs, dq_p=dq_p), J, nominal_dt=DT)
        c = classes(res, 'p'); self.assertEqual(c[4], 'READBACK_IMPULSE'); self.assertEqual(c[5], 'SUSTAINED_MOTION')
        self.assertTrue(res['summary']['prospective_shadow']['would_abort'])

    def test_stale_out_of_order_and_nonfinite_samples_are_not_differentiated(self):
        s = make([0.2] * 6)
        s[3]['physics_s'] = s[2]['physics_s']                                     # duplicate timestamp
        s[4]['q']['p'] = float('nan')                                              # non-finite
        s[5]['physics_s'] = s[1]['physics_s']                                      # out of order
        res = observe(s, J, nominal_dt=DT)
        rows = {r['sequence']: r for r in res['rows'] if r['joint'] == 'p'}
        self.assertIsNone(rows[3]['dq_interval']); self.assertEqual(rows[4]['classification'], 'INVALID_SAMPLE'); self.assertIsNone(rows[5]['dq_interval'])

    def test_variable_dt_uses_the_actual_step_and_marks_gaps(self):
        s = make([0.1 * i * DT for i in range(6)])
        s[3]['physics_s'] += 0.01                                                  # a 3x gap before sample 3 (and a short step after)
        res = observe(s, J, nominal_dt=DT)
        rows = {r['sequence']: r for r in res['rows'] if r['joint'] == 'p'}
        self.assertIsNone(rows[3]['dq_interval']); self.assertIsNotNone(rows[2]['dq_interval'])
        s2 = make([0.1 * i * 0.004 for i in range(6)], dt=0.004)
        res2 = observe(s2, J, nominal_dt=0.004)
        self.assertTrue(all(abs(r['dq_interval'] - 0.1) < 1e-9 for r in res2['rows'] if r['joint'] == 'p' and r['dq_interval'] is not None))

    def test_wrap_handling_for_continuous_joints_only(self):
        self.assertAlmostEqual(angle_difference(-3.10, 3.10, continuous=True), 0.0832, 4)
        self.assertAlmostEqual(angle_difference(-3.10, 3.10, continuous=False), -6.2, 9)
        Jc = {'w': JointSpec('w', 1.0, continuous=True)}
        s = [{'sequence': i, 'physics_s': i * DT, 'phase': 'replay', 'q': {'w': [3.10, -3.10][i % 2]}, 'dq': {'w': 0.0}} for i in range(2)]
        res = observe(s, Jc, nominal_dt=DT)
        self.assertAlmostEqual(res['rows'][1]['dq_interval'], 0.0832 / DT, 2)

    def test_broken_coupling_is_a_fault_not_a_readback_artifact(self):
        qs_p = [0.5] * 8; qs_c = [1.0843 * 0.5] * 4 + [1.0843 * 0.5 + 0.2] * 4     # child departs from the mapping by 0.2 rad
        res = observe(make(qs_p, qs_c=qs_c, dq_c=[0.0] * 8), J, nominal_dt=DT)
        self.assertIn('COUPLING_FAULT', classes(res, 'c')); self.assertTrue(res['summary']['prospective_shadow']['would_abort'])

    def test_commanded_versus_observed_identity_is_not_confused(self):
        # observer consumes MEASURED q/dq only: a command channel is never passed, and a mismatch between command and state does not flag
        qs = [0.3] * 5
        res = observe(make(qs), J, nominal_dt=DT)
        self.assertTrue(all('cmd' not in r for r in res['rows'])); self.assertEqual(set(classes(res, 'p')), {'OK'})

    def test_legitimate_fast_closure_is_reported_as_real_motion(self):
        qs = [min(1.4, 1.6 * i * DT) for i in range(60)]                         # 1.6 rad/s closure: > 1x, < 2x on the parent
        res = observe(make(qs), J, nominal_dt=DT)
        self.assertGreater(res['summary']['legacy_score_1x']['reported_over_1x_samples'], 0)
        self.assertFalse(res['summary']['legacy_abort_2x']['would_abort'])
        # the coupled child moves 1.0843x faster: 1.735 rad/s -> still < 2x; both channels agree -> OK, no artifact claimed
        self.assertEqual(set(classes(res, 'c')), {'OK'})
        qs2 = [min(1.4, 2.4 * i * DT) for i in range(60)]                        # 2.4 rad/s: real motion above 2x -> abort in both
        res2 = observe(make(qs2), J, nominal_dt=DT)
        self.assertTrue(res2['summary']['legacy_abort_2x']['would_abort']); self.assertTrue(res2['summary']['prospective_shadow']['would_abort'])

    def test_contact_artifact_with_moving_parent_is_ambiguous_not_cleared(self):
        # reported impulse on the child while the parent's interval velocity is already above 1x: one interval cannot resolve it
        qs_p = [1.2 * i * DT for i in range(10)]
        dq_c = [1.0843 * 1.2] * 10; dq_c[5] = 2.9
        res = observe(make(qs_p, dq_c=dq_c), J, nominal_dt=DT)
        self.assertEqual(classes(res, 'c')[5], 'AMBIGUOUS'); self.assertTrue(res['summary']['prospective_shadow']['would_abort'])

    def test_summary_reports_channels_separately(self):
        res = observe(make([0.3] * 3), J, nominal_dt=DT)
        for k in ('legacy_score_1x', 'legacy_abort_2x', 'interval_channel', 'prospective_shadow'):
            self.assertIn(k, res['summary'])
        self.assertEqual(set(res['summary']['prospective_shadow']['classes']), set(CLASSES))


if __name__ == '__main__':
    unittest.main()
