"""CPU tests: the six-axis command sets reproduce the qualified rod30 hand path through the single contract (same joint targets
as the qualified spec's own contract for every row, when the campaign spec is present; embedded rows otherwise), the v12 writer
grasp round-trips to the bench's closed targets, the learned-route set is refused without the exploratory declaration, and the
binding value equals the applied-profile hash the qualification graph uses."""
import json
import os
import unittest
from pathlib import Path

from isaac.twin.inspire.embodiment import HAND_ACTUATORS, ContractError, EmbodimentManifest, HandCommandAdapter, hand_contract_descriptor
from isaac.twin.inspire.hand_command_sets import HandCommandSets, verify_against_spec

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / 'isaac/twin/inspire/embodiments/g1_edu29_rh56dftp_donor_v1.json'
SPEC = Path(os.environ.get('ROD30_SPEC', '/home/ubuntu/panthera/sim-workspace/campaign-20260913/sessions/sprintJ-20260917T165919Z/configs/source-spec-rod30-pick-v2.json'))
ROD30_CONTRACT = {'axis_order': list(HAND_ACTUATORS), 'open_value': 0.0, 'closed_value': 1.0, 'saturation_policy': 'reject'}
# rows copied from the qualified spec (sha256 d038f523…): home/open, mid-close, closed, mid-open
ROD30_ROWS = [('home', [0.013907, 0.013907, 0.013907, 0.013907, 0.034106, 0.017181]), ('close', [0.4569, 0.4569, 0.4569, 0.4569, 0.2921, 0.2586]), ('close', [0.9, 0.9, 0.9, 0.9, 0.55, 0.5]), ('open', [0.8989, 0.8989, 0.8989, 0.8989, 0.5487, 0.4988])]


class HandCommandSetsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = EmbodimentManifest.load(MANIFEST); cls.sets = HandCommandSets.load()

    def test_rod30_rows_convert_identically_through_the_contract(self):
        legacy = HandCommandAdapter(self.m, 'right', ROD30_CONTRACT); ours, decl = self.sets.adapter(self.m, 'right')
        self.assertEqual(decl['fields_not_verified'], [])                                     # the scripted route on the twin is fully verified: no exploratory declaration needed
        for stage, vals in ROD30_ROWS:
            a, ia = legacy.to_joint_targets(vals); b, ib = ours.to_joint_targets(vals)
            self.assertEqual(a, b, stage); self.assertEqual(ia['clipped_axes'], [])
        closed, info = self.sets.joint_targets(self.m, 'rod30_pick_v2', 'closed')
        self.assertAlmostEqual(closed['right_index_1_joint'], 0.9 * 1.4381, 9); self.assertAlmostEqual(closed['right_thumb_2_joint'], 0.55 * 0.5864, 9); self.assertAlmostEqual(closed['right_thumb_1_joint'], 0.5 * 1.1641, 9)
        opened, _ = self.sets.joint_targets(self.m, 'rod30_pick_v2', 'open')
        for j, v in opened.items():
            self.assertAlmostEqual(v, 0.02, 5, j)                                             # the 0.02 rad open margin on every actuator
        self.assertEqual(ours.profile_sha256, legacy.profile_sha256)                          # same applied profile ...
        self.assertEqual(info['profile_sha256'], hand_contract_descriptor({'right': legacy}))    # ... -> same qualification binding value
        self.assertEqual(self.sets.binding(self.m), hand_contract_descriptor({'right': legacy}))

    @unittest.skipUnless(SPEC.exists(), 'qualified rod30 spec not available in this checkout')
    def test_every_row_of_the_qualified_spec(self):
        r = verify_against_spec(self.sets, self.m, SPEC, 'rod30_pick_v2')
        self.assertEqual(r['rows'], 1132); self.assertLessEqual(r['max_abs_diff_rad'], 1e-9); self.assertIn('close', r['stages'])

    def test_ramp_reproduces_the_open_to_closed_shape(self):
        rows = self.sets.ramp('rod30_pick_v2', 'open', 'closed', 2.0, 30.0)
        self.assertEqual(len(rows), 61); self.assertEqual(rows[0][1], self.sets.closure('rod30_pick_v2', 'open')); self.assertEqual([round(v, 6) for v in rows[-1][1]], [0.9, 0.9, 0.9, 0.9, 0.55, 0.5])
        self.assertAlmostEqual(rows[30][1][0], 0.013907 + (0.9 - 0.013907) * 0.5, 6)      # cosine blend midpoint

    def test_v12_writer_grasp_round_trips_to_the_bench_closed_targets(self):
        t, info = self.sets.joint_targets(self.m, 'held_marker_grasp_v12', 'closed')
        self.assertAlmostEqual(t['right_index_1_joint'], 1.21298, 6); self.assertAlmostEqual(t['right_middle_1_joint'], 1.4381, 6); self.assertAlmostEqual(t['right_little_1_joint'], 1.4248, 6)
        self.assertAlmostEqual(t['right_thumb_2_joint'], 0.41494, 6); self.assertAlmostEqual(t['right_thumb_1_joint'], 0.62442, 6)   # sM-writer-v11 hand_hold_gains.right_grasp.closed_targets_rad
        self.assertEqual(info['clipped_axes'], [])
        i, _ = self.sets.joint_targets(self.m, 'held_marker_grasp_v12', 'initial'); self.assertAlmostEqual(i['right_index_1_joint'], 1.16298, 6)

    def test_learned_route_set_needs_the_exploratory_declaration(self):
        with self.assertRaisesRegex(ContractError, 'fail-closed'):
            self.sets.contract.route('model', 'right', model_map='piston_n16')
        src, decl = self.sets.contract.route('model', 'right', model_map='piston_n16', exploratory=True)
        ad = HandCommandAdapter(self.m, 'right', src); t, info = ad.to_joint_targets([1.30, 1.30, 1.30, 1.30, 0.0, -0.1])
        self.assertAlmostEqual(t['right_index_1_joint'], 1.30, 9); self.assertEqual(info['clipped_axes'], ['thumb_rotation'])

    def test_sets_bind_the_contract_and_refuse_a_foreign_one(self):
        d = json.loads((ROOT / 'isaac/twin/inspire/embodiments/hand_command_sets_v1.json').read_text()); d['contract_sha256'] = '0' * 64
        with self.assertRaisesRegex(ContractError, 'written for contract'):
            HandCommandSets(d, self.sets.contract)


if __name__ == '__main__':
    unittest.main()
