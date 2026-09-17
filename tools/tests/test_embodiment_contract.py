"""CPU-only tests for the versioned embodiment manifest, hand command adapter and replay sequence contract."""
import copy
import json
import math
import unittest
from pathlib import Path

from isaac.twin.inspire.embodiment import (
    BODY_JOINT_ORDER_UNITREE_29, ContractError, EmbodimentManifest, HAND_ACTUATORS, HandCommandAdapter, ReplaySequence,
)

MANIFEST_PATH = Path(__file__).resolve().parents[2] / 'isaac/twin/inspire/embodiments/g1_edu29_rh56dftp_donor_v1.json'
DATASET_CONTRACT = {'axis_order': ['index', 'middle', 'ring', 'little', 'thumb_bend', 'thumb_rotation'], 'open_value': 0.0, 'closed_value': 1.0}
NATIVE_CONTRACT = {'axis_order': ['little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation'], 'open_value': 1000.0, 'closed_value': 0.0}
SOURCE = {'source_id': 'unit-test', 'kind': 'synthetic_test_sequence', 'provenance': 'test fixture'}
# Distinct sentinel closures per actuator so a permutation cannot pass by coincidence.
SENTINEL = {'index': 0.11, 'middle': 0.23, 'ring': 0.37, 'little': 0.41, 'thumb_bend': 0.59, 'thumb_rotation': 0.73}


def manifest():
    return EmbodimentManifest.load(MANIFEST_PATH)


class ManifestTests(unittest.TestCase):
    def test_donor_manifest_loads_with_expected_structure(self):
        m = manifest()
        self.assertEqual(m.body_names, BODY_JOINT_ORDER_UNITREE_29)
        self.assertEqual(m.hand_joint_names('right'), ('right_index_1_joint', 'right_middle_1_joint', 'right_ring_1_joint', 'right_little_1_joint', 'right_thumb_2_joint', 'right_thumb_1_joint'))
        self.assertFalse(m.data['source_asset']['exact_hand_model'])
        self.assertIsNone(m.data['qualification']['installed_hand_similarity_percent'])
        self.assertEqual(len(m.data['hands']['left']['coupled_joints']), 6)

    def test_missing_required_field_is_refused(self):
        data = json.loads(MANIFEST_PATH.read_text())
        del data['controller']['clock_domains']
        with self.assertRaises(ContractError):
            EmbodimentManifest(data)
        data = json.loads(MANIFEST_PATH.read_text())
        del data['hands']['right']['actuators']['thumb_rotation']
        with self.assertRaises(ContractError):
            EmbodimentManifest(data)

    def test_wrong_body_order_or_count_is_refused(self):
        data = json.loads(MANIFEST_PATH.read_text())
        data['body']['joint_names'][0], data['body']['joint_names'][1] = data['body']['joint_names'][1], data['body']['joint_names'][0]
        with self.assertRaises(ContractError):
            EmbodimentManifest(data)

    def test_e2_target_profile_with_unknown_endpoints_is_rejected_as_embodiment(self):
        data = json.loads(MANIFEST_PATH.read_text())
        data['manifest_id'] = 'rh56e2_t1_target_unmeasured'
        for spec in data['hands']['right']['actuators'].values():
            spec['open_rad'] = None; spec['closed_rad'] = None
        with self.assertRaises(ContractError):
            EmbodimentManifest(data)

    def test_cross_side_joint_mapping_is_refused(self):
        data = json.loads(MANIFEST_PATH.read_text())
        data['hands']['right']['actuators']['index']['joint'] = 'left_index_1_joint'
        with self.assertRaises(ContractError):
            EmbodimentManifest(data)

    def test_validity_hashes_invalidate_dependent_qualification(self):
        m = manifest()
        ok = m.check_validity({'urdf_sha256': m.data['source_asset']['urdf_sha256']})
        self.assertTrue(ok['valid']); self.assertEqual(ok['qualification']['contact_writing'], 'EXECUTED_NOT_QUALIFIED')
        changed = m.check_validity({'urdf_sha256': 'deadbeef'})
        self.assertFalse(changed['valid'])
        self.assertEqual(changed['qualification']['contact_writing'], 'INVALIDATED_BY_CHANGE')
        self.assertEqual(changed['transforms']['right.wrist_to_hand']['status'], 'INVALID')
        self.assertEqual(m.data['qualification']['contact_writing'], 'EXECUTED_NOT_QUALIFIED')   # never mutated
        missing = m.check_validity({})
        self.assertFalse(missing['valid'])


class HandAdapterTests(unittest.TestCase):
    def test_named_reordering_with_sentinels(self):
        m = manifest()
        ds = HandCommandAdapter(m, 'right', DATASET_CONTRACT)
        native = HandCommandAdapter(m, 'right', NATIVE_CONTRACT)
        ds_values = [SENTINEL[a] for a in DATASET_CONTRACT['axis_order']]
        native_values = [1000.0 * (1.0 - SENTINEL[a]) for a in NATIVE_CONTRACT['axis_order']]
        t1, i1 = ds.to_joint_targets(ds_values)
        t2, i2 = native.to_joint_targets(native_values)
        for a in HAND_ACTUATORS:
            self.assertAlmostEqual(i1['closure'][a], SENTINEL[a]); self.assertAlmostEqual(i2['closure'][a], SENTINEL[a])
        for j in t1:
            self.assertAlmostEqual(t1[j], t2[j], places=9)
        self.assertAlmostEqual(t1['right_index_1_joint'], 0.11 * 1.4381)
        self.assertAlmostEqual(t1['right_thumb_1_joint'], 0.73 * m.hand_actuator('right', 'thumb_rotation')['closed_rad'])

    def test_left_right_isolation(self):
        m = manifest()
        left, _ = HandCommandAdapter(m, 'left', DATASET_CONTRACT).to_joint_targets([SENTINEL[a] for a in DATASET_CONTRACT['axis_order']])
        right, _ = HandCommandAdapter(m, 'right', DATASET_CONTRACT).to_joint_targets([SENTINEL[a] for a in DATASET_CONTRACT['axis_order']])
        self.assertTrue(all(j.startswith('left_') for j in left)); self.assertTrue(all(j.startswith('right_') for j in right))
        self.assertFalse(set(left) & set(right))
        coupled = set(m.data['hands']['right']['coupled_joints']) | set(m.data['hands']['left']['coupled_joints'])
        self.assertFalse(coupled & (set(left) | set(right)))   # coupled joints are never commanded

    def test_open_close_endpoints(self):
        m = manifest()
        a = HandCommandAdapter(m, 'right', DATASET_CONTRACT)
        opened, _ = a.to_joint_targets([0.0] * 6); closed, _ = a.to_joint_targets([1.0] * 6)
        for act in HAND_ACTUATORS:
            spec = m.hand_actuator('right', act)
            self.assertEqual(opened[spec['joint']], spec['open_rad']); self.assertEqual(closed[spec['joint']], spec['closed_rad'])
        n = HandCommandAdapter(m, 'right', NATIVE_CONTRACT)
        self.assertEqual(n.to_joint_targets([1000.0] * 6)[0], opened); self.assertEqual(n.to_joint_targets([0.0] * 6)[0], closed)

    def test_round_trip(self):
        m = manifest()
        for contract in (DATASET_CONTRACT, NATIVE_CONTRACT):
            a = HandCommandAdapter(m, 'left', contract)
            span = contract['closed_value'] - contract['open_value']
            values = [contract['open_value'] + span * SENTINEL[x] for x in contract['axis_order']]
            targets, _ = a.to_joint_targets(values)
            back = a.from_joint_state(targets)
            for v, b in zip(values, back):
                self.assertAlmostEqual(v, b, places=9)

    def test_coupling_semantics_match_asset_ratios(self):
        m = manifest()
        c = m.data['hands']['right']['coupled_joints']
        self.assertEqual(c['right_index_2_joint'], {'parent': 'right_index_1_joint', 'multiplier': 1.0843, 'offset': 0.0, 'limit_rad': [0.0, 3.14]})
        self.assertEqual(c['right_thumb_3_joint']['parent'], 'right_thumb_2_joint'); self.assertEqual(c['right_thumb_4_joint']['parent'], 'right_thumb_3_joint')
        # a closed thumb bend drives thumb_3 within its own limit
        spec = m.hand_actuator('right', 'thumb_bend')
        self.assertLessEqual(spec['closed_rad'] * c['right_thumb_3_joint']['multiplier'], c['right_thumb_3_joint']['limit_rad'][1])

    def test_nonfinite_out_of_range_and_wrong_width_are_refused_before_conversion(self):
        a = HandCommandAdapter(manifest(), 'right', DATASET_CONTRACT)
        for bad in ([math.nan] + [0.2] * 5, [0.2] * 5 + [math.inf], [1.5] + [0.2] * 5, [-0.01] + [0.2] * 5, [0.2] * 5, [0.2] * 7, [True] + [0.2] * 5, ['0.2'] + [0.2] * 5):
            with self.assertRaises(ContractError):
                a.to_joint_targets(bad)

    def test_clipping_only_under_declared_policy_and_reported(self):
        m = manifest()
        clip = HandCommandAdapter(m, 'right', dict(DATASET_CONTRACT, saturation_policy='clip_declared'))
        targets, info = clip.to_joint_targets([1.2, 0.2, 0.2, 0.2, -0.3, 0.2])
        self.assertEqual(info['clipped_axes'], ['index', 'thumb_bend'])
        self.assertEqual(targets['right_index_1_joint'], m.hand_actuator('right', 'index')['closed_rad'])
        self.assertEqual(targets['right_thumb_2_joint'], m.hand_actuator('right', 'thumb_bend')['open_rad'])
        with self.assertRaises(ContractError):
            HandCommandAdapter(m, 'right', dict(DATASET_CONTRACT, saturation_policy='silently_clip'))

    def test_invalid_normalization_and_axis_order_refused(self):
        m = manifest()
        with self.assertRaises(ContractError):
            HandCommandAdapter(m, 'right', dict(DATASET_CONTRACT, open_value=0.5, closed_value=0.5))
        with self.assertRaises(ContractError):
            HandCommandAdapter(m, 'right', dict(DATASET_CONTRACT, axis_order=['index'] * 6))
        with self.assertRaises(ContractError):
            HandCommandAdapter(m, 'right', dict(DATASET_CONTRACT, axis_order=['index', 'middle', 'ring', 'little', 'thumb', 'thumb_rotation']))
        with self.assertRaises(ContractError):
            HandCommandAdapter(m, 'up', DATASET_CONTRACT)


def rows(n=5, dt=0.1):
    return [{'t_s': i * dt, 'body_q_rad': {'right_elbow_joint': -0.5 + 0.01 * i, 'right_shoulder_pitch_joint': 0.1},
             'hands': {'right': [SENTINEL[a] for a in DATASET_CONTRACT['axis_order']], 'left': None}} for i in range(n)]


class ReplaySequenceTests(unittest.TestCase):
    def test_valid_sequence_converts_and_hashes(self):
        seq = ReplaySequence(manifest(), rows(), hand_contracts={'right': DATASET_CONTRACT}, source=SOURCE)
        s = seq.summary()
        self.assertEqual(s['rows'], 5); self.assertAlmostEqual(s['duration_s'], 0.4); self.assertEqual(s['hands_commanded'], ['right'])
        self.assertEqual(s['body_joints_commanded'], ['right_elbow_joint', 'right_shoulder_pitch_joint'])
        self.assertEqual(seq.active_row(0.25)['row'], 2); self.assertIsNone(seq.active_row(-0.01))
        self.assertIn('right_thumb_1_joint', seq.converted[0]['hands']['right']['targets_rad'])
        self.assertEqual(len(seq.contract_sha256), 64)

    def test_stale_duplicate_gapped_and_reversed_times_are_refused(self):
        m = manifest()
        for mutate in (lambda r: r[2].update(t_s=r[1]['t_s']), lambda r: r[2].update(t_s=r[1]['t_s'] - 0.05), lambda r: r[2].update(t_s=r[1]['t_s'] + 5.0)):
            r = rows(); mutate(r)
            with self.assertRaises(ContractError):
                ReplaySequence(m, r, hand_contracts={'right': DATASET_CONTRACT}, source=SOURCE)

    def test_missing_fields_unknown_joint_out_of_range_and_undeclared_hand_refused(self):
        m = manifest()
        r = rows(); del r[1]['hands']
        with self.assertRaises(ContractError):
            ReplaySequence(m, r, hand_contracts={'right': DATASET_CONTRACT}, source=SOURCE)
        r = rows(); r[1]['body_q_rad']['right_wrist_x_joint'] = 0.0
        with self.assertRaises(ContractError):
            ReplaySequence(m, r, hand_contracts={'right': DATASET_CONTRACT}, source=SOURCE)
        r = rows(); r[1]['body_q_rad']['right_elbow_joint'] = 9.0
        with self.assertRaises(ContractError):
            ReplaySequence(m, r, hand_contracts={'right': DATASET_CONTRACT}, source=SOURCE)
        r = rows(); r[1]['hands']['left'] = [0.1] * 6
        with self.assertRaises(ContractError):
            ReplaySequence(m, r, hand_contracts={'right': DATASET_CONTRACT}, source=SOURCE)
        with self.assertRaises(ContractError):
            ReplaySequence(m, rows(), hand_contracts={'right': DATASET_CONTRACT}, source={'source_id': 'x', 'kind': 'real_data_maybe', 'provenance': 'p'})

    def test_incompatible_manifest_changes_contract_hash(self):
        m = manifest()
        base = ReplaySequence(m, rows(), hand_contracts={'right': DATASET_CONTRACT}, source=SOURCE).contract_sha256
        data = json.loads(MANIFEST_PATH.read_text()); data['manifest_id'] = 'g1_edu29_other_hand_v2'
        data['hands']['right']['actuators']['index']['closed_rad'] = 1.2
        other = ReplaySequence(EmbodimentManifest(data), rows(), hand_contracts={'right': DATASET_CONTRACT}, source=SOURCE).contract_sha256
        self.assertNotEqual(base, other)
        again = ReplaySequence(m, copy.deepcopy(rows()), hand_contracts={'right': DATASET_CONTRACT}, source=SOURCE).contract_sha256
        self.assertEqual(base, again)


if __name__ == '__main__':
    unittest.main()
