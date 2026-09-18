"""CPU tests for the single six-axis hand action contract (sprint M): axis order fixed; the scripted route reproduces the
existing normalized-closure contract through HandCommandAdapter; the model route reproduces the closed-loop probe's radian
contract (same applied-profile hash) and is fail-closed without an explicit exploratory declaration; the installed-hand target
never yields a contract while fields are UNRESOLVED; UNRESOLVED fields cannot carry values; the hash tracks every field."""
import copy
import json
import unittest
from pathlib import Path

from isaac.twin.inspire.embodiment import HAND_ACTUATORS, ContractError, EmbodimentManifest, HandCommandAdapter, hand_contract_descriptor
from isaac.twin.inspire.hand_action_contract import HandActionContract
from isaac.twin.inspire.model_action_adapter import PISTON_HAND_ORDER

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / 'isaac/twin/inspire/embodiments/g1_edu29_rh56dftp_donor_v1.json'
CONTRACT = ROOT / 'isaac/twin/inspire/embodiments/hand_action_contract_rh56dftp_donor_v1.json'


class HandActionContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = EmbodimentManifest.load(MANIFEST); cls.c = HandActionContract.load(CONTRACT); cls.data = json.loads(CONTRACT.read_text())

    def test_instance_binds_the_twin_manifest_and_fixed_axis_order(self):
        self.assertEqual(self.data['twin_manifest_sha256'], self.m.sha256)
        for side in ('left', 'right'):
            self.assertEqual(tuple(self.data['sides'][side]['axes']), HAND_ACTUATORS)
            for a in HAND_ACTUATORS:
                ax = self.data['sides'][side]['axes'][a]['twin']; act = self.m.hand_actuator(side, a)
                self.assertEqual((ax['joint']['value'], ax['open']['value'], ax['closed']['value']), (act['joint'], act['open_rad'], act['closed_rad']))
                self.assertEqual(ax['velocity_cap_rad_s']['value'], 1.0)

    def test_scripted_route_reproduces_the_normalized_closure_contract(self):
        src, decl = self.c.route('scripted', 'right')
        self.assertEqual(decl['fields_not_verified'], []); self.assertFalse(decl['exploratory'])
        legacy = {'axis_order': list(HAND_ACTUATORS), 'open_value': 0.0, 'closed_value': 1.0, 'saturation_policy': 'reject'}
        a, b = HandCommandAdapter(self.m, 'right', src), HandCommandAdapter(self.m, 'right', legacy)
        for vals in ([0.0] * 6, [1.0] * 6, [0.3, 0.5, 0.7, 0.9, 0.2, 0.1]):
            self.assertEqual(a.to_joint_targets(vals)[0], b.to_joint_targets(vals)[0])
        self.assertEqual(a.profile_sha256, b.profile_sha256)                                   # same applied profile -> same qualification binding

    def test_model_route_is_fail_closed_and_matches_the_probe_radian_contract(self):
        with self.assertRaisesRegex(ContractError, 'fail-closed'):
            self.c.route('model', 'right', model_map='piston_n16')
        src, decl = self.c.route('model', 'right', model_map='piston_n16', exploratory=True)
        self.assertEqual(decl['fields_not_verified'], ['model_maps.piston_n16.axis_identity:EXPLORATORY']); self.assertTrue(decl['exploratory'])
        limits = {a: self.m.hand_actuator('right', a)['closed_rad'] for a in HAND_ACTUATORS}
        probe = {'axis_order': list(PISTON_HAND_ORDER), 'open_value': 0.0, 'closed_value': 1.0, 'per_axis_endpoints': {a: {'open_value': 0.0, 'closed_value': limits[a]} for a in HAND_ACTUATORS}, 'saturation_policy': 'clip_declared'}   # tools/probes/command_replay.py closed loop
        a, b = HandCommandAdapter(self.m, 'right', src), HandCommandAdapter(self.m, 'right', probe)
        self.assertEqual(a.profile_sha256, b.profile_sha256)
        targets, info = a.to_joint_targets([1.3, 1.3, 1.3, 1.3, 0.0, -0.1])                    # a dataset row: pinky..index 1.3, thumb_pitch 0, thumb_yaw -0.1
        self.assertAlmostEqual(targets['right_index_1_joint'], 1.3); self.assertAlmostEqual(targets['right_thumb_1_joint'], 0.0)
        self.assertEqual(info['clipped_axes'], ['thumb_rotation'])                             # the pinned -0.1 is clipped and RECORDED, never silent
        self.assertEqual(decl['profile_sha256'], hand_contract_descriptor({'right': a}))

    def test_installed_hand_target_never_yields_a_contract_while_unresolved(self):
        with self.assertRaisesRegex(ContractError, 'installed hand'):
            self.c.route('scripted', 'right', target='installed_e2', exploratory=True)
        self.assertEqual(len(self.c.unresolved('right', 'installed_e2', 'scripted')), 24)     # 4 fields x 6 axes not VERIFIED (3 UNRESOLVED + travel NOMINAL)

    def test_unresolved_fields_cannot_carry_values_and_verified_fields_need_them(self):
        d = copy.deepcopy(self.data); d['sides']['right']['axes']['index']['installed_e2']['speed']['value'] = 2.0
        with self.assertRaisesRegex(ContractError, 'UNRESOLVED but carries a value'):
            HandActionContract(d)
        d = copy.deepcopy(self.data); d['sides']['right']['axes']['index']['twin']['velocity_cap_rad_s']['value'] = None
        with self.assertRaisesRegex(ContractError, 'has no value'):
            HandActionContract(d)
        d = copy.deepcopy(self.data); d['sides']['right']['axes'] = dict(reversed(list(d['sides']['right']['axes'].items())))
        with self.assertRaisesRegex(ContractError, 'in this order'):
            HandActionContract(d)

    def test_hash_tracks_every_field_and_the_declaration(self):
        d = copy.deepcopy(self.data); d['sides']['left']['axes']['little']['installed_e2']['tactile']['what_would_resolve'] = 'changed'
        self.assertNotEqual(HandActionContract(d).sha256, self.c.sha256)
        _, decl = self.c.route('scripted', 'left'); self.assertEqual(decl['contract_sha256'], self.c.sha256); self.assertEqual(decl['side'], 'left')
        self.assertEqual(self.c.velocity_caps('right'), {a: 1.0 for a in HAND_ACTUATORS})


if __name__ == '__main__':
    unittest.main()
