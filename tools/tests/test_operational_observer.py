import unittest

from isaac.twin.inspire.operational_observer import LEGACY, OPERATIONAL_V1, OperationalObserver, composed_fields, make_observer

LIMITS = {'right_index_1_joint': {'velocity': 1.0}, 'right_index_2_joint': {'velocity': 1.0}, 'right_thumb_2_joint': {'velocity': 1.0}, 'right_thumb_3_joint': {'velocity': 1.0}, 'right_thumb_4_joint': {'velocity': 1.0}, 'right_elbow_joint': {'velocity': 37.0}}
MIMIC = {'right_index_2_joint': {'parent': 'right_index_1_joint', 'multiplier': 1.0843, 'offset': 0.0}, 'right_thumb_3_joint': {'parent': 'right_thumb_2_joint', 'multiplier': 0.8024, 'offset': 0.0}, 'right_thumb_4_joint': {'parent': 'right_thumb_3_joint', 'multiplier': 0.9487, 'offset': 0.0}}
DT = 0.005


def run(obs, rows):
    """rows: list of (t, q dict, dq dict); returns the decisions."""
    return [obs.update(i, t, q, dq) for i, (t, q, dq) in enumerate(rows)]


def coupled(q1, th2):
    return {'right_index_1_joint': q1, 'right_index_2_joint': 1.0843 * q1, 'right_thumb_2_joint': th2, 'right_thumb_3_joint': 0.8024 * th2, 'right_thumb_4_joint': 0.9487 * 0.8024 * th2, 'right_elbow_joint': 0.5}


class OperationalObserverTests(unittest.TestCase):
    def test_composed_fields_scale_mimic_children_by_the_chain(self):
        f = composed_fields(LIMITS, MIMIC)
        self.assertAlmostEqual(f['right_index_2_joint'], 1.0843); self.assertAlmostEqual(f['right_thumb_3_joint'], 0.8024); self.assertAlmostEqual(f['right_thumb_4_joint'], 0.9487 * 0.8024); self.assertEqual(f['right_index_1_joint'], 1.0)

    def test_readback_impulse_on_a_static_mimic_child_is_shadow_only(self):
        obs = OperationalObserver(LIMITS, MIMIC, DT, threshold_multiple=1.0, reported_cap=5.0)     # the writer guard's 1x rule
        rows = [(DT * i, coupled(0.4, 0.36), {**{n: 0.0 for n in LIMITS}, 'right_thumb_3_joint': (1.015 if i == 5 else 0.0)}) for i in range(8)]
        d = run(obs, rows)
        self.assertFalse(any(x['abort'] for x in d)); self.assertEqual(d[5]['legacy_shadow']['over_1x'], {'right_thumb_3_joint': 1.015})
        self.assertTrue(obs.summary['legacy_shadow']['would_abort_1x']); self.assertFalse(obs.summary['legacy_shadow']['would_abort_2x']); self.assertEqual(obs.summary['legacy_shadow']['first_1x']['sequence'], 5)

    def test_real_excursion_seen_by_the_positions_aborts_after_persist(self):
        obs = OperationalObserver(LIMITS, MIMIC, DT, threshold_multiple=2.0, persist=2)
        q = [0.0, 0.0, 0.0, 0.03, 0.06, 0.09]                                                       # 6 rad/s sampled on thumb_2 from sample 3
        rows = [(DT * i, coupled(0.2, q[i]), {n: 0.0 for n in LIMITS}) for i in range(6)]
        d = run(obs, rows)
        self.assertFalse(d[3]['abort'])                                                            # one interval > 2x: not yet (persist 2)
        self.assertTrue(d[4]['abort']); self.assertEqual(d[4]['reason'], 'sampled_interval_overspeed'); self.assertIn('right_thumb_2_joint', d[4]['violations']); self.assertIn('right_thumb_3_joint', d[4]['violations'])

    def test_one_sample_interval_spike_is_logged_not_acted_on(self):
        obs = OperationalObserver(LIMITS, MIMIC, DT, threshold_multiple=2.0, persist=2)
        q = [0.0, 0.0, 0.03, 0.03, 0.03]
        d = run(obs, [(DT * i, coupled(q[i], 0.1), {n: 0.0 for n in LIMITS}) for i in range(5)])
        self.assertFalse(any(x['abort'] for x in d)); self.assertEqual(obs.summary['interval_over_threshold_samples'], 0); self.assertGreater(obs.summary['max_abs_interval_rad_s'], 2.0)

    def test_mimic_child_at_its_gear_ratio_is_not_an_overspeed(self):
        obs = OperationalObserver(LIMITS, MIMIC, DT, threshold_multiple=1.0, persist=1)
        q = [0.0, 0.0049, 0.0098, 0.0147, 0.0196]                                                    # parent at 0.98 rad/s -> child 1.063 rad/s (> 1.0 but < 1.0843)
        d = run(obs, [(DT * i, coupled(q[i], 0.0), {n: 0.0 for n in LIMITS}) for i in range(5)])
        self.assertFalse(any(x['abort'] for x in d))

    def test_coupling_fault_aborts_regardless_of_velocity(self):
        obs = OperationalObserver(LIMITS, MIMIC, DT)
        q = coupled(0.3, 0.2); q['right_thumb_3_joint'] += 0.05
        d = obs.update(0, 0.0, q, {n: 0.0 for n in LIMITS})
        self.assertTrue(d['abort']); self.assertEqual(d['reason'], 'coupling_fault'); self.assertIn('right_thumb_3_joint', d['coupling_faults'])

    def test_first_sample_reset_and_irregular_dt_give_no_interval_decision(self):
        obs = OperationalObserver(LIMITS, MIMIC, DT, threshold_multiple=1.0, persist=1)
        d0 = obs.update(0, 0.0, coupled(0.0, 0.0), {n: 0.0 for n in LIMITS}); self.assertFalse(d0['comparable'])
        d1 = obs.update(1, 0.5, coupled(0.9, 0.0), {n: 0.0 for n in LIMITS}); self.assertFalse(d1['comparable']); self.assertFalse(d1['abort'])   # gap: not comparable
        obs.reset(); d2 = obs.update(2, 0.505, coupled(0.0, 0.0), {n: 0.0 for n in LIMITS}); self.assertFalse(d2['comparable'])

    def test_make_observer_names(self):
        self.assertIsNone(make_observer(LEGACY, LIMITS, MIMIC, DT)); self.assertIsInstance(make_observer(OPERATIONAL_V1, LIMITS, MIMIC, DT), OperationalObserver)
        with self.assertRaises(ValueError):
            make_observer('anything_else', LIMITS, MIMIC, DT)


if __name__ == '__main__':
    unittest.main()
