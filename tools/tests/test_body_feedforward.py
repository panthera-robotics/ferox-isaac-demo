"""CPU tests for the bounded gravity feed-forward accounting (controller v4) and its controller-declaration validation."""
import json, unittest
from pathlib import Path
import numpy as np
from isaac.twin.inspire.body_feedforward import bounded_gravity_feedforward
from isaac.twin.inspire.embodiment import EmbodimentManifest
from tools.replay_commands import validate_controller_gains

MANIFEST = Path(__file__).resolve().parents[2] / 'isaac/twin/inspire/embodiments/g1_edu29_rh56dftp_donor_v1.json'


class FeedforwardAccounting(unittest.TestCase):
    def test_sign_convention_holding_pose_needs_no_pd_when_feedforward_equals_gravity_hold(self):
        # at the target with zero velocity the PD estimate is zero; the applied effort equals the hold term (positive sign = counteracts gravity)
        ff, total, pd, capped = bounded_gravity_feedforward([60.0], [1.5], [0.3], [0.3], [0.0], [7.0], [25.0])
        self.assertAlmostEqual(pd[0], 0.0); self.assertAlmostEqual(ff[0], 7.0); self.assertAlmostEqual(total[0], 7.0); self.assertFalse(capped[0])

    def test_combined_cap_reduces_feedforward_not_the_pd_estimate(self):
        # PD estimate 60*(0.4)=24 plus a 7 N·m hold term would exceed the 25 N·m limit: feed-forward reduced to 1, event flagged
        ff, total, pd, capped = bounded_gravity_feedforward([60.0], [0.0], [0.7], [0.3], [0.0], [7.0], [25.0])
        self.assertAlmostEqual(pd[0], 24.0); self.assertAlmostEqual(total[0], 25.0); self.assertAlmostEqual(ff[0], 1.0); self.assertTrue(capped[0])
        # opposite sign: PD -24 with hold -7 -> total clipped at -25, applied -1
        ff, total, pd, capped = bounded_gravity_feedforward([60.0], [0.0], [0.3], [0.7], [0.0], [-7.0], [25.0])
        self.assertAlmostEqual(total[0], -25.0); self.assertAlmostEqual(ff[0], -1.0); self.assertTrue(capped[0])

    def test_ramp_and_scale_and_damping(self):
        ff, total, pd, capped = bounded_gravity_feedforward([60.0], [1.5], [0.3], [0.3], [2.0], [7.0], [25.0], ramp=0.5)
        self.assertAlmostEqual(pd[0], -3.0); self.assertAlmostEqual(ff[0], 3.5)
        with self.assertRaises(ValueError):
            bounded_gravity_feedforward([60.0], [1.5], [0.3], [0.3], [0.0], [7.0], [25.0], ramp=1.5)
        with self.assertRaises(ValueError):
            bounded_gravity_feedforward([60.0], [1.5], [0.3], [0.3], [0.0], [float('nan')], [25.0])

    def test_controller_declaration_validation(self):
        m = EmbodimentManifest.load(MANIFEST)
        base = {'body_kp_nm_rad': {n: 60.0 for n in m.body_names}, 'body_kd_nm_s_rad': {n: 1.5 for n in m.body_names}, 'hand_kp_nm_rad': 1.0, 'hand_kd_nm_s_rad': 0.05, 'provenance': 'unit test controller declaration for the feed-forward validator'}
        self.assertEqual(validate_controller_gains(m, dict(base, gravity_feedforward={'enabled': False})), [])
        ok = {'enabled': True, 'source': 'articulation_generalized_gravity_forces_current_configuration', 'joints': 'commanded_body_joints', 'combined_effort_limit': 'urdf_effort_limit', 'ramp_in_s': 1.0, 'scale': 1.0}
        self.assertEqual(validate_controller_gains(m, dict(base, gravity_feedforward=ok)), [])
        self.assertTrue(validate_controller_gains(m, dict(base, gravity_feedforward=dict(ok, source='pinocchio'))))
        self.assertTrue(validate_controller_gains(m, dict(base, gravity_feedforward=dict(ok, scale=1.5))))
        self.assertTrue(validate_controller_gains(m, dict(base, gravity_feedforward=dict(ok, combined_effort_limit='none'))))
        self.assertTrue(validate_controller_gains(m, dict(base, gravity_feedforward='yes')))


if __name__ == '__main__':
    unittest.main()
