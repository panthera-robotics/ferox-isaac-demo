"""Encode/decode reference tests for the cached piston model interface: the declared transforms preserve a physical pose,
constant columns are pinned by the statistics, relative arms decode to absolute exactly once (double application is a type
error), hand endpoints and units, per-step delta bounds and clipping. Uses the cached statistics.json when present
(HAND_FIDELITY_PISTON_STATS), otherwise a synthetic statistics dict with the same structure."""
import json
import os
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hand_fidelity import piston_model_interface as pmi   # noqa: E402

STATS_PATH = os.environ.get('HAND_FIDELITY_PISTON_STATS')


def synthetic_stats():
    s = {'state': {}, 'action': {}, 'relative_action': {}}
    for k in pmi.STATE_KEYS:
        d = pmi.DIMS[k]; s['state'][k] = {'min': [-1.0] * d, 'max': [1.3] * d}
    s['action']['left_arm'] = {'min': [0.0] * 7, 'max': [0.0] * 7}
    s['action']['right_arm'] = {'min': [-1.29] * 7, 'max': [0.7] * 7}
    s['action']['left_hand'] = {'min': [0.0] * 6, 'max': [0.0] * 6}
    s['action']['right_hand'] = {'min': [0, 0, 0, 0, 0, -0.1], 'max': [1.3, 1.3, 1.3, 1.3, 0.0, -0.1]}
    s['action']['base_height'] = {'min': [0.76], 'max': [0.76]}; s['action']['navigate_command'] = {'min': [0.0] * 3, 'max': [0.0] * 3}
    s['relative_action']['left_arm'] = {'min': [[-0.04] * 7] * 30, 'max': [[0.0] * 7] * 30}
    s['relative_action']['right_arm'] = {'min': [[-0.6 - 0.001 * t] * 7 for t in range(30)], 'max': [[0.2 + 0.001 * t] * 7 for t in range(30)]}
    return {'new_embodiment': s}


def stats():
    return pmi.Stats(STATS_PATH) if STATS_PATH and Path(STATS_PATH).exists() else pmi.Stats(synthetic_stats())


def pose(right_arm=(0.1, -0.2, 0.3, -0.4, 0.05, 0.6, -0.07), right_hand=(0.5, 0.5, 0.5, 0.5, 0.0, -0.1)):
    arm = {n: 0.0 for n in pmi.arm_joint_names('left')}; arm.update(dict(zip(pmi.arm_joint_names('right'), right_arm)))
    hand = {n: 0.0 for n in pmi.hand_joint_names('left')}; hand.update(dict(zip(pmi.hand_joint_names('right'), right_hand)))
    return pmi.urdf_pose(arm, hand, {n: 0.0 for n in pmi.WAIST})


class InterfaceTests(unittest.TestCase):
    def test_names_orders_and_dims_are_the_dataset_ones(self):
        self.assertEqual(pmi.arm_joint_names('right'), ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint'])
        self.assertEqual(pmi.hand_joint_names('right'), ['right_little_1_joint', 'right_ring_1_joint', 'right_middle_1_joint', 'right_index_1_joint', 'right_thumb_2_joint', 'right_thumb_1_joint'])
        self.assertEqual(sum(pmi.DIMS[k] for k in pmi.STATE_KEYS), 29); self.assertEqual(sum(pmi.DIMS[k] for k in pmi.ACTION_KEYS), 30)

    def test_encode_is_raw_radians_without_sincos_or_normalization(self):
        st = pmi.encode_state(pose())
        self.assertEqual(st['type'], 'dataset_state_rad'); self.assertFalse(st['sincos'])
        self.assertTrue(np.allclose(st['values']['right_arm'][0], [0.1, -0.2, 0.3, -0.4, 0.05, 0.6, -0.07]))
        self.assertTrue(np.allclose(st['values']['right_hand'][0], [0.5, 0.5, 0.5, 0.5, 0.0, -0.1]))
        with self.assertRaises(TypeError):
            pmi.encode_state({'type': 'absolute_target_rad'})

    def test_relative_round_trip_preserves_the_physical_pose_exactly_once(self):
        s = stats(); st = pmi.encode_state(pose())
        p = s.action_params('right_arm'); mn = np.asarray(p['min'], float); mx = np.asarray(p['max'], float)
        deltas = 0.5 * (mn + mx) + 0.25 * (mx - mn) * np.sin(np.arange(pmi.CHUNK))[:, None]     # inside every per-step bound
        chunk_abs = st['values']['right_arm'][0] + deltas
        normalized = pmi.dataset_action_to_relative(chunk_abs, st['values']['right_arm'], s, 'right_arm')   # training-side transform
        net = {k: np.zeros((pmi.CHUNK, pmi.DIMS[k])) for k in pmi.ACTION_KEYS}; net['right_arm'] = normalized
        out = pmi.server_unapply(net, st, s)
        self.assertEqual(out['type'], 'absolute_target_rad')
        self.assertTrue(np.allclose(out['values']['right_arm'], chunk_abs, atol=1e-9))            # exactly the absolute target
        by_name = pmi.targets_by_name(out, 0)
        self.assertAlmostEqual(by_name['right_shoulder_pitch_joint'], float(chunk_abs[0, 0]), 9)
        # double application: adding the state again would be a second decode — refused by type
        with self.assertRaises(TypeError):
            pmi.targets_by_name({'type': 'dataset_state_rad', 'values': out['values']}, 0)
        with self.assertRaises(TypeError):
            pmi.server_unapply(net, out, s)                                                       # a target chunk is not a reference state

    def test_constant_columns_are_pinned_whatever_the_network_outputs(self):
        s = stats(); st = pmi.encode_state(pose())
        pinned = s.pinned_action_columns()
        self.assertIn('base_height', pinned); self.assertIn('navigate_command', pinned); self.assertIn('left_hand', pinned)
        self.assertNotIn('left_arm', pinned)     # RELATIVE keys use the relative_action bounds: the left arm is not pinned, only tiny (<= 0.04 rad/step)
        self.assertEqual(pinned['right_hand']['columns'], [4, 5]); self.assertTrue(np.allclose(pinned['right_hand']['values'], [0.0, -0.1]))
        for fill in (-1.0, 0.0, 1.0, 5.0):
            net = {k: np.full((pmi.CHUNK, pmi.DIMS[k]), fill) for k in pmi.ACTION_KEYS}
            out = pmi.server_unapply(net, st, s)['values']
            self.assertTrue(np.allclose(out['right_hand'][:, 4], 0.0)); self.assertTrue(np.allclose(out['right_hand'][:, 5], -0.1))
            self.assertTrue(np.allclose(out['base_height'], 0.76)); self.assertTrue(np.allclose(out['navigate_command'], 0.0))
            self.assertTrue(np.allclose(out['left_hand'], 0.0))

    def test_hand_endpoints_units_and_clipping(self):
        s = stats(); st = pmi.encode_state(pose())
        net = {k: np.zeros((pmi.CHUNK, pmi.DIMS[k])) for k in pmi.ACTION_KEYS}
        net['right_hand'][:, :4] = 1.0; out = pmi.server_unapply(net, st, s)['values']['right_hand']
        self.assertTrue(np.allclose(out[:, :4], 1.3))                        # normalized +1 -> the dataset max 1.3 rad
        net['right_hand'][:, :4] = -1.0; out = pmi.server_unapply(net, st, s)['values']['right_hand']
        self.assertTrue(np.allclose(out[:, :4], 0.0))                        # -1 -> 0 rad (open)
        net['right_hand'][:, :4] = 7.0; out = pmi.server_unapply(net, st, s)['values']['right_hand']
        self.assertTrue(np.allclose(out[:, :4], 1.3))                        # out-of-range outputs clip to the bound, never beyond

    def test_relative_arm_bounds_are_per_step_and_saturate_at_the_row_bound(self):
        s = stats(); st = pmi.encode_state(pose())
        p = s.action_params('right_arm'); mn = np.asarray(p['min'], float); mx = np.asarray(p['max'], float)
        self.assertEqual(mn.shape, (pmi.CHUNK, 7))
        net = {k: np.zeros((pmi.CHUNK, pmi.DIMS[k])) for k in pmi.ACTION_KEYS}; net['right_arm'][:] = -1.0
        out = pmi.server_unapply(net, st, s)['values']['right_arm']
        self.assertTrue(np.allclose(out - st['values']['right_arm'][0], mn, atol=1e-9))   # saturated network output = the per-step min delta
        net['right_arm'][:] = 1.0
        out = pmi.server_unapply(net, st, s)['values']['right_arm']
        self.assertTrue(np.allclose(out - st['values']['right_arm'][0], mx, atol=1e-9))


if __name__ == '__main__':
    unittest.main()
