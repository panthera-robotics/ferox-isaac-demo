"""CPU harness for the learned-policy action adapter (validate-only). Inputs here are explicitly MOCK unless noted."""
import math
import unittest
from pathlib import Path

from isaac.twin.inspire.embodiment import ContractError, EmbodimentManifest
from isaac.twin.inspire.model_action_adapter import ADAPTER_VERSION, mock_chunk, validate_chunk

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

    def test_droid_output_shape_is_rejected_as_a_different_embodiment(self):
        # a real DROID chunk has keys eef_9d/gripper_position/joint_position — not a G1 action space
        with self.assertRaises(ContractError):
            validate_chunk(self.m, {'eef_9d': [[0.0] * 9], 'gripper_position': [[0.0]], 'joint_position': [[0.0] * 7]})


if __name__ == '__main__':
    unittest.main()
