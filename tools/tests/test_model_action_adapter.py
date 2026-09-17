"""CPU harness for the learned-policy action adapter (validate-only). Inputs here are explicitly MOCK unless noted."""
import math
import unittest
from pathlib import Path

from isaac.twin.inspire.embodiment import ContractError, EmbodimentManifest
from isaac.twin.inspire.embodiment import HandCommandAdapter, HAND_ACTUATORS
from isaac.twin.inspire.model_action_adapter import ADAPTER_VERSION, PISTON_HAND_ORDER, mock_chunk, validate_chunk, validate_piston_chunk

MANIFEST = Path(__file__).resolve().parents[2] / 'isaac/twin/inspire/embodiments/g1_edu29_rh56dftp_donor_v1.json'


class AdapterHarness(unittest.TestCase):
    def setUp(self):
        self.m = EmbodimentManifest.load(MANIFEST)

    def test_mock_full_scope_is_never_executable(self):
        chunk, label = mock_chunk(self.m); self.assertEqual(label, 'MOCK')
        r = validate_chunk(self.m, chunk)
        self.assertEqual(r['adapter'], ADAPTER_VERSION); self.assertFalse(r['executable']); self.assertIn('refusal', r)
        self.assertIn('left_hand', r['rejected']); self.assertIn('navigate_command', r['rejected']); self.assertEqual(r['horizon'], 40)

    def test_mock_arms_and_waist_scope_executable_when_in_limits(self):
        chunk, _ = mock_chunk(self.m, horizon=8)
        r = validate_chunk(self.m, chunk, scope='arms_and_waist_only', horizon_steps=8)
        self.assertTrue(r['executable']); self.assertEqual(len(r['body_targets_rad']), 8)
        self.assertEqual(set(r['body_targets_rad'][0]), {n for n in self.m.body_names if 'shoulder' in n or 'elbow' in n or 'wrist' in n or 'waist' in n})
        self.assertEqual(r['values_outside_limits'], 0)

    def test_out_of_limit_nonfinite_shape_and_key_errors_are_refusals_not_clamps(self):
        chunk, _ = mock_chunk(self.m, horizon=4)
        chunk['right_arm'][2][3] = 9.0   # right elbow far outside its limit
        r = validate_chunk(self.m, chunk, scope='arms_and_waist_only')
        self.assertFalse(r['executable']); self.assertEqual(r['values_outside_limits'], 1); self.assertIsNone(r['body_targets_rad'])
        chunk, _ = mock_chunk(self.m, horizon=4); chunk['waist'][0][0] = math.nan
        with self.assertRaises(ContractError):
            validate_chunk(self.m, chunk, scope='arms_and_waist_only')
        chunk, _ = mock_chunk(self.m, horizon=4); chunk['left_arm'][1] = chunk['left_arm'][1][:6]
        with self.assertRaises(ContractError):
            validate_chunk(self.m, chunk, scope='arms_and_waist_only')
        chunk, _ = mock_chunk(self.m, horizon=4); del chunk['right_hand']
        with self.assertRaises(ContractError):
            validate_chunk(self.m, chunk, scope='arms_and_waist_only')
        chunk, _ = mock_chunk(self.m, horizon=4); chunk['left_arm'] = chunk['left_arm'][:3]
        with self.assertRaises(ContractError):
            validate_chunk(self.m, chunk, scope='arms_and_waist_only')   # keys must share one horizon

    def piston_adapters(self):
        limits = {a: self.m.hand_actuator('right', a)['closed_rad'] for a in HAND_ACTUATORS}
        c = {'axis_order': list(PISTON_HAND_ORDER), 'open_value': 0.0, 'closed_value': 1.0, 'per_axis_endpoints': {a: {'open_value': 0.0, 'closed_value': limits[a]} for a in HAND_ACTUATORS}, 'saturation_policy': 'clip_declared'}
        return {s: HandCommandAdapter(self.m, s, c) for s in ('left', 'right')}

    def piston_mock(self, H=30, base=0.76, nav=(0.0, 0.0, 0.0)):
        row = {'left_arm': [0.0] * 7, 'right_arm': [-0.15, 0.2, 0.25, -0.2, -0.2, 0.4, -0.25], 'left_hand': [0.0] * 6, 'right_hand': [1.3, 1.3, 1.3, 1.3, 0.0, -0.1], 'base_height': [base], 'navigate_command': list(nav)}
        return {k: [list(v) for _ in range(H)] for k, v in row.items()}   # MOCK, shaped like the piston checkpoint output

    def test_piston_mock_chunk_converts_with_declared_thumb_clip(self):
        r = validate_piston_chunk(self.m, self.piston_mock(), hand_adapters=self.piston_adapters(), prefix_steps=15)
        self.assertTrue(r['executable']); self.assertEqual(len(r['rows']), 15); self.assertEqual(r['rows'][1]['t_s'], 0.02)
        self.assertEqual(r['rows'][0]['body_q_rad']['right_elbow_joint'], -0.2)
        self.assertEqual(r['clipped_steps'], 15); self.assertEqual(r['interventions'][0]['clipped_axes'], ['thumb_rotation'])   # thumb_yaw -0.1 below the donor limit, recorded not hidden
        self.assertEqual(r['rows'][0]['hands']['right'][3], 1.3)   # index radians pass through by name

    def test_piston_non_neutral_base_or_out_of_limit_arm_refused(self):
        r = validate_piston_chunk(self.m, self.piston_mock(nav=(0.3, 0, 0)), hand_adapters=self.piston_adapters())
        self.assertFalse(r['executable']); self.assertIn('base', r['rejected'])
        r = validate_piston_chunk(self.m, self.piston_mock(base=0.5), hand_adapters=self.piston_adapters()); self.assertFalse(r['executable'])
        chunk = self.piston_mock(); chunk['right_arm'][3][3] = 5.0
        r = validate_piston_chunk(self.m, chunk, hand_adapters=self.piston_adapters()); self.assertFalse(r['executable']); self.assertIn('right_arm', r['rejected'])
        chunk = self.piston_mock(); chunk['right_hand'][0] = [9.0, 0, 0, 0, 0, 0]
        r = validate_piston_chunk(self.m, chunk, hand_adapters=self.piston_adapters()); self.assertTrue(r['executable']); self.assertEqual(r['interventions'][0]['clipped_axes'], ['little'])
        chunk = self.piston_mock(); chunk['left_hand'][0] = [math.nan] * 6
        with self.assertRaises(ContractError):
            validate_piston_chunk(self.m, chunk, hand_adapters=self.piston_adapters())

    def test_droid_output_shape_is_rejected_as_a_different_embodiment(self):
        # a real DROID chunk has keys eef_9d/gripper_position/joint_position — not a G1 action space
        with self.assertRaises(ContractError):
            validate_chunk(self.m, {'eef_9d': [[0.0] * 9], 'gripper_position': [[0.0]], 'joint_position': [[0.0] * 7]})


if __name__ == '__main__':
    unittest.main()
