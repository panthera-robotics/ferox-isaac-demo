"""Adversarial measured-state tests; no simulator or policy launch."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assembled_balance import (BalanceConfig, FEET, GATES, SourceEquilibriumPD, evaluate, ground_contacts,
                               initial_height, material_self_penetrations, source_adjacency, source_foot_spheres)


class BalanceEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.names = [f'joint_{i}' for i in range(53)]
        cls.limits = {n: {'lower': -1., 'upper': 1.} for n in cls.names}
        cls.poses = {'pelvis': [0., 0., .8, 0., 0., 0., 1.],
                     FEET[0]: [0., .1, .04, 0., 0., 0., 1.], FEET[1]: [0., -.1, .04, 0., 0., 0., 1.]}
        cls.initial = {'physics_s': .01, 'link_poses_world_xyzw': cls.poses}
        cls.rows = []
        for i in range(2000):
            now = .01 + (i + 1) * .005
            cls.rows.append({'sequence': i, 'physics_s': now, 'runtime_names': cls.names,
                'q_rad': [0.] * 53, 'dq_rad_s': [0.] * 53, 'measured_generalized_effort_nm': [0.] * 53,
                'command_velocity': [0., 0., 0.], 'body_command_owner': 'named_policy_single_writer',
                'body_command_names': cls.names[:29], 'body_command_rad': [0.] * 29,
                'policy_observation': [0.] * 480, 'policy_action': [0.] * 29,
                'link_poses_world_xyzw': cls.poses,
                'contacts': [{'actor0': '/World/Ground', 'actor1': '/World/G1/' + n,
                    'impulse_ns': [0., 0., .5], 'sequence': i, 'physics_s': now} for n in FEET]})

    def result(self, rows=None, **kw):
        return evaluate(self.rows if rows is None else rows, self.initial, self.limits,
                        {'joint_52': {'parent': 'joint_51', 'multiplier': 1., 'offset': 0.}},
                        supporting_constraints=kw.pop('supporting_constraints', []),
                        integrity_checks=kw.pop('integrity_checks', {'live_inertia': True}), **kw)

    def mutated(self, change):
        rows = self.rows.copy(); rows[500] = copy.deepcopy(rows[500]); change(rows[500]); return self.result(rows)

    def test_full_physical_evidence_passes(self):
        self.assertEqual(self.result()['status'], 'PASS')

    def test_partial_or_duplicate_time_fails(self):
        self.assertEqual(self.result(self.rows[:-1])['status'], 'FAIL')
        r = self.mutated(lambda r: r.update(physics_s=self.rows[499]['physics_s']))
        self.assertFalse(r['checks']['contiguous_physics_sequence'])

    def test_duplicate_sequence_fails(self):
        self.assertEqual(self.mutated(lambda r: r.update(sequence=499))['status'], 'FAIL')

    def test_missing_contact_is_not_a_standing_pass(self):
        rows = [dict(r, contacts=[]) for r in self.rows]
        self.assertFalse(self.result(rows)['checks']['both_feet_have_measured_loaded_contact'])
        self.assertEqual(self.mutated(lambda r: r.pop('contacts'))['status'], 'FAIL')

    def test_zero_impulse_proximity_is_not_loading(self):
        feet, other = ground_contacts([{'actor0': '/World/Ground', 'actor1': '/World/G1/pelvis', 'impulse_ns': [0., 0., 0.]}])
        self.assertEqual(other, []); self.assertEqual(sum(feet.values()), 0.)

    def test_nonfoot_ground_and_contact_clock_mismatch_fail(self):
        r = self.mutated(lambda r: r['contacts'][0].update(actor1='/World/G1/right_base_link'))
        self.assertFalse(r['checks']['no_loaded_nonfoot_ground_contact'])
        self.assertEqual(self.mutated(lambda r: r['contacts'][0].update(physics_s=0.))['status'], 'FAIL')

    def test_nan_in_any_policy_or_state_fails(self):
        for key in ('q_rad', 'dq_rad_s', 'policy_observation', 'policy_action', 'body_command_rad'):
            with self.subTest(key=key):
                self.assertEqual(self.mutated(lambda r: r[key].__setitem__(0, float('nan')))['status'], 'FAIL')

    def test_missing_named_coordinates_or_actions_fail(self):
        for key in ('runtime_names', 'q_rad', 'policy_observation', 'policy_action'):
            with self.subTest(key=key):
                self.assertEqual(self.mutated(lambda r: r[key].pop())['status'], 'FAIL')

    def test_pose_gate_uses_measurements(self):
        for link, coordinate, value, gate in [
            ('pelvis', 2, .64, 'minimum_pelvis_height'), ('pelvis', 0, .101, 'pelvis_xy_drift_within_gate'),
            (FEET[0], 0, .031, 'each_foot_xy_drift_within_gate'), (FEET[1], 2, .071, 'each_foot_height_rise_within_gate')]:
            r = self.mutated(lambda r: r['link_poses_world_xyzw'][link].__setitem__(coordinate, value))
            self.assertFalse(r['checks'][gate])

    def test_lean_nonunit_and_infinite_quaternion_fail(self):
        def lean(r):
            r['link_poses_world_xyzw']['pelvis'][3:] = [math.sin(.105), 0., 0., math.cos(.105)]
        self.assertFalse(self.mutated(lean)['checks']['roll_pitch_within_gate'])
        self.assertEqual(self.mutated(lambda r: r['link_poses_world_xyzw']['pelvis'].__setitem__(6, 2.))['status'], 'FAIL')

    def test_limit_mimic_support_abort_and_audit_fail(self):
        self.assertFalse(self.mutated(lambda r: r['q_rad'].__setitem__(0, 1.031))['checks']['source_joint_limits_within_gate'])
        self.assertFalse(self.mutated(lambda r: r['q_rad'].__setitem__(52, .031))['checks']['hand_coupling_within_gate'])
        self.assertEqual(self.result(supporting_constraints=['/World/FixedPelvis'])['status'], 'FAIL')
        self.assertEqual(self.result(abort={'reason': 'fall'})['status'], 'FAIL')
        self.assertEqual(self.result(integrity_checks={'live_inertia': False})['status'], 'FAIL')

    def test_nonzero_command_or_multiple_owners_fail(self):
        self.assertFalse(self.mutated(lambda r: r.update(command_velocity=[.01, 0., 0.]))['checks']['single_zero_command_body_owner'])
        self.assertFalse(self.mutated(lambda r: r.update(body_command_owner='two_writers'))['checks']['single_zero_command_body_owner'])

    def test_config_cannot_relax_gates_or_change_private_mount(self):
        self.assertEqual(BalanceConfig.from_dict({}).policy_path, '/policy')
        for value in ({'duration_s': 1.}, {'policy_path': '/tmp/model'}, {'ground_static_friction': float('inf')},
                      {'ground_dynamic_friction': 1.5}, {'ground_static_friction': True},
                      {'controller_mode': 'source_contact_equilibrium_pd'}, {'controller_mode': 'invented'},
                      {'source_contact_equilibrium': {}}):
            with self.assertRaises(ValueError): BalanceConfig.from_dict(value)

    def controller(self, reference=None):
        reference = reference or {'source_sha256': 'a'*64, 'hardware_authorized': False,
            'source_named29_home_rad': dict.fromkeys(self.names[:29], 0.),
            'named29_contact_equilibrium_effort_nm': dict.fromkeys(self.names[:29], .5),
            'equilibrium_base_wrench_residual': [0.] * 6,
            'minimum_norm_positive_normal_contact_forces': [{'equilibrium_vertical_force_n': 1.}] * 8}
        return SourceEquilibriumPD(reference, self.names[:29], [0.]*29, [10.]*29, [2.]*29, [3.]*29,
                                   source_sha256='a'*64)

    def test_equilibrium_names_pd_and_unchanged_cap_are_exact(self):
        c = self.controller()
        q, dq = [0.]*53, [0.]*53; q[0] = -.4; dq[1] = .25
        result = c.compute(self.names, q, dq)
        self.assertEqual(result['body_effort_unclipped_nm'][:2], [4.5, 0.])
        self.assertEqual(result['body_effort_nm'][:2], [3., 0.])
        self.assertEqual(result['body_effort_saturated_names'], ['joint_0'])
        reversed_result = c.compute(list(reversed(self.names)), list(reversed(q)), list(reversed(dq)))
        self.assertEqual(result, reversed_result)
        with self.assertRaises(ValueError): c.compute(self.names[:-1], q[:-1], dq[:-1])
        q[52] = float('nan')
        with self.assertRaises(ValueError): c.compute(self.names, q, dq)

    def test_equilibrium_source_pose_feasibility_and_caps_cannot_be_relabelled(self):
        reference = self.controller().reference
        mutations = [lambda r: r.update(source_sha256='b'*64),
                     lambda r: r['source_named29_home_rad'].update(joint_0=.1),
                     lambda r: r['named29_contact_equilibrium_effort_nm'].pop('joint_0'),
                     lambda r: r['named29_contact_equilibrium_effort_nm'].update(joint_0=3.01),
                     lambda r: r['equilibrium_base_wrench_residual'].__setitem__(0, .01),
                     lambda r: r['minimum_norm_positive_normal_contact_forces'][0].update(equilibrium_vertical_force_n=-1.)]
        for mutate in mutations:
            changed = copy.deepcopy(reference); mutate(changed)
            with self.assertRaises(ValueError): self.controller(changed)

    def test_equilibrium_full_evidence_needs_torque_and_true_feedback_not_policy480(self):
        controller = self.controller()
        initial = dict(self.initial, runtime_names=self.names, q_rad=[0.]*53, dq_rad_s=[0.]*53)
        rows = []
        for i, original in enumerate(self.rows):
            row = {k: v for k, v in original.items() if k not in ('policy_observation', 'policy_action')}
            row.update(controller.compute(self.names, row['q_rad'], row['dq_rad_s']))
            row.update(controller_mode='source_contact_equilibrium_pd', body_effort_writes_this_step=1,
                body_implicit_position_writes_this_step=0, body_command_owner='source_equilibrium_effort_single_writer',
                body_feedback_source_sequence=i-1, body_feedback_physics_s=row['physics_s']-.005,
                applied_generalized_actuation_effort_nm=row['body_effort_nm'] + [0.]*24)
            rows.append(row)
        def check(values):
            return evaluate(values, initial, self.limits, {}, supporting_constraints=[], integrity_checks={},
                controller_mode='source_contact_equilibrium_pd', controller_reference=controller.receipt())
        self.assertEqual(check(rows)['status'], 'PASS')
        for key, value in [('body_effort_nm', 4.), ('body_feedback_q_rad', .1),
                           ('applied_generalized_actuation_effort_nm', 4.)]:
            changed = copy.deepcopy(rows); changed[10][key][0] = value
            self.assertEqual(check(changed)['status'], 'FAIL')
        changed = copy.deepcopy(rows); changed[10]['body_effort_writes_this_step'] = 2
        self.assertFalse(check(changed)['checks']['single_zero_command_body_owner'])

    def test_loaded_nonadjacent_self_penetration_uses_pair_load_and_source_adjacency(self):
        contact = {'actor0': '/World/G1/palm', 'actor1': '/World/G1/thumb2',
                   'impulse_ns': [0., 0., .003], 'separation_m': -.0011}
        self.assertEqual(material_self_penetrations([contact], set()), [])
        self.assertEqual(len(material_self_penetrations([contact, contact], set())), 1)
        self.assertEqual(material_self_penetrations([contact, contact], {('palm', 'thumb2')}), [])
        self.assertEqual(material_self_penetrations([dict(contact, impulse_ns=[0., 0., 0.])]*10, set()), [])
        self.assertEqual(material_self_penetrations([dict(contact, separation_m=-.0009)]*10, set()), [])
        rows = self.rows.copy(); rows[500] = copy.deepcopy(rows[500])
        rows[500]['contacts'].extend([dict(contact, sequence=500, physics_s=rows[500]['physics_s'])]*2)
        self.assertFalse(self.result(rows)['checks']['no_material_loaded_nonadjacent_self_penetration'])

    def test_source_adjacency_is_direct_and_does_not_exempt_grandchildren(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'source.urdf'
            path.write_text('<robot><joint><parent link="palm"/><child link="thumb1"/></joint>'
                            '<joint><parent link="thumb1"/><child link="thumb2"/></joint></robot>')
            self.assertEqual(source_adjacency(path), {('palm', 'thumb1'), ('thumb1', 'thumb2')})

    def test_source_sphere_clearance_uses_radius_and_fk(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest('numpy required for independent source FK arithmetic')
        xml = '<robot name="test">' + ''.join('<link name="' + n + '">' +
            '<collision><origin xyz="0 0 -0.02"/><geometry><sphere radius="0.005"/></geometry></collision>' * 4 + '</link>' for n in FEET) + '</robot>'
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / 'source.urdf'; p.write_text(xml); spheres = source_foot_spheres(p)
            fk = {n: np.eye(4) for n in FEET}; fk[FEET[0]][2, 3] = -.7; fk[FEET[1]][2, 3] = -.6
            self.assertAlmostEqual(initial_height(spheres, fk), .727)
            p.write_text(xml.replace('left_ankle_roll_link', 'not_a_foot'))
            with self.assertRaises(ValueError): source_foot_spheres(p)


if __name__ == '__main__':
    unittest.main()
