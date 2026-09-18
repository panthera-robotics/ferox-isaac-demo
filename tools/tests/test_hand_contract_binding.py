"""CPU tests for binding the applied hand conversion profile, the arm reference datum and the actual physics step into the
manifest/qualification graph (sprint L H3): specific dependency edges — a changed hand contract stales the hand-command
dependent claims only; a changed controller or camera never invalidates the measured wrist mount; a DIAGNOSTIC physics step
stales every dt-bound claim by construction; a manifest revision that does not bind the profile can never be active."""
import json
import unittest
from pathlib import Path

from isaac.twin.inspire.embodiment import (ARM_DATUM_SCRIPTED, HAND_ACTUATORS, HAND_COMMAND_DEPENDENT_CLAIMS, ContractError, EmbodimentManifest,
                                           HandCommandAdapter, hand_contract_descriptor, hand_profile_sha256, runtime_dependency_values)
from tools import replay_commands

MANIFEST = Path(__file__).resolve().parents[2] / 'isaac/twin/inspire/embodiments/g1_edu29_rh56dftp_donor_v1.json'
URDF = Path(__file__).resolve().parents[3] / 'generated/ftp_donor/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf'
COOKING = 'right=ftp_palm_yz_slabs_v2;left=ftp_left_palm_yz_slabs_v1;contact_offset_m=0.0012860533315688372;rest_offset_m=0'
CONTROLLER = 'implicit_biased_drive_v1 replay controller (package-hashed gains)'
CLOSURE = {'axis_order': list(HAND_ACTUATORS), 'open_value': 0.0, 'closed_value': 1.0, 'saturation_policy': 'reject'}


def radian_contract(manifest, closed_scale=1.0):
    limits = {a: manifest.hand_actuator('right', a)['closed_rad'] * closed_scale for a in HAND_ACTUATORS}
    return {'axis_order': list(HAND_ACTUATORS), 'open_value': 0.0, 'closed_value': 1.0, 'saturation_policy': 'clip_declared',
            'per_axis_endpoints': {a: {'open_value': 0.0, 'closed_value': limits[a]} for a in HAND_ACTUATORS}}


def bound_manifest(contracts, arm_datum=ARM_DATUM_SCRIPTED):
    """The v1 manifest with command_replay_integration bound to the given applied profile and datum (what the builder now writes)."""
    data = json.loads(MANIFEST.read_text())
    cfg = data['qualification']['claims']['command_replay_integration']['configuration']
    cfg['hand_contract_sha256'] = hand_contract_descriptor(contracts); cfg['arm_datum'] = arm_datum
    return EmbodimentManifest(data)


def live_from(manifest, support='FIXED_PELVIS', **extra):
    """Live values equal to every bound value of the manifest's claims (so a claim with this support is ACTIVE_COMPATIBLE), then overridden."""
    claims = manifest.data['qualification']['claims']
    live = {}
    for name in ('mechanism_checks', 'retention_60s_grasp_v12', 'retention_60s_grasp_v13', 'acquisition_cycles_kd05', 'command_replay_integration'):
        live.update(claims[name]['configuration'])
    live['support'] = support
    live.update(extra); return live


class ProfileHash(unittest.TestCase):
    def test_profile_hash_is_manifest_independent_and_specific(self):
        m = EmbodimentManifest.load(MANIFEST)
        other = json.loads(MANIFEST.read_text()); other['qualification']['claims']['standing']['evidence'] = 'changed'; m2 = EmbodimentManifest(other)
        self.assertNotEqual(m.sha256, m2.sha256)
        a1, a2 = HandCommandAdapter(m, 'right', CLOSURE), HandCommandAdapter(m2, 'right', CLOSURE)
        self.assertNotEqual(a1.contract_sha256, a2.contract_sha256)          # manifest-bound hash differs (unchanged behaviour)
        self.assertEqual(a1.profile_sha256, a2.profile_sha256)               # the applied profile is the same
        self.assertEqual(a1.profile_sha256, hand_profile_sha256('right', CLOSURE))
        self.assertNotEqual(hand_profile_sha256('left', CLOSURE), hand_profile_sha256('right', CLOSURE))
        for mutation in ({'saturation_policy': 'clip_declared'}, {'closed_value': 1000.0}, {'endpoint_tolerance': 0.01},
                         {'axis_order': list(reversed(HAND_ACTUATORS))}, {'per_axis_endpoints': {'thumb_bend': {'open_value': 0.0, 'closed_value': 0.5}}}):
            self.assertNotEqual(hand_profile_sha256('right', dict(CLOSURE, **mutation)), a1.profile_sha256, mutation)
        with self.assertRaises(ContractError):
            hand_profile_sha256('right', dict(CLOSURE, saturation_policy='ignore'))

    def test_descriptor_from_adapters_equals_descriptor_from_contracts(self):
        m = EmbodimentManifest.load(MANIFEST)
        contracts = {'left': radian_contract(m), 'right': radian_contract(m)}
        adapters = {sd: HandCommandAdapter(m, sd, contracts[sd]) for sd in contracts}
        self.assertEqual(hand_contract_descriptor(adapters), hand_contract_descriptor(contracts))
        self.assertNotEqual(hand_contract_descriptor({'right': contracts['right']}), hand_contract_descriptor(contracts))   # the set of applied sides matters
        self.assertEqual(hand_contract_descriptor({}), 'none')


class DependencyEdges(unittest.TestCase):
    def setUp(self):
        self.m = EmbodimentManifest.load(MANIFEST)
        self.contracts = {'right': CLOSURE}
        self.bound = bound_manifest(self.contracts)

    def test_every_dependency_equal_is_active(self):
        r = self.bound.check_validity(live_from(self.bound))
        self.assertEqual(r['claims']['command_replay_integration']['active_compatibility'], 'ACTIVE_COMPATIBLE')
        self.assertEqual(r['claims']['command_replay_integration']['unbound'], [])
        self.assertEqual(r['transforms']['right.wrist_to_hand']['status'], 'VALID')

    def assert_unrelated_untouched(self, **change):
        # same support as the replay claim: the mechanism checks and the measured wrist mount stay valid
        r = self.bound.check_validity(live_from(self.bound, **change))
        self.assertEqual(r['claims']['mechanism_checks']['active_compatibility'], 'ACTIVE_COMPATIBLE')
        self.assertEqual(r['transforms']['right.wrist_to_hand']['status'], 'VALID')                            # measured wrist mount untouched
        # the grasp / acquisition / contact-writing evidence never bound a hand conversion profile: each stays active under
        # its own bound configuration plus the change (specific edges, not a global invalidation)
        for unrelated in ('retention_60s_grasp_v12', 'retention_60s_grasp_v13', 'acquisition_cycles_kd05', 'contact_writing'):
            own = dict(self.bound.data['qualification']['claims'][unrelated]['configuration'], **change)
            self.assertEqual(self.bound.check_validity(own)['claims'][unrelated]['active_compatibility'], 'ACTIVE_COMPATIBLE', unrelated)
        return r

    def test_changed_hand_contract_stales_replay_claim_only(self):
        changed = {'right': dict(CLOSURE, saturation_policy='clip_declared')}
        r = self.bound.check_validity(live_from(self.bound, hand_contract_sha256=hand_contract_descriptor(changed)))
        c = r['claims']['command_replay_integration']
        self.assertEqual(c['active_compatibility'], 'STALE'); self.assertIn('hand_contract_sha256', c['mismatched'])
        self.assertEqual(c['historical_status'], 'PASS')                                                     # history keeps its PASS
        self.assert_unrelated_untouched(hand_contract_sha256=hand_contract_descriptor(changed))

    def test_changed_arm_datum_stales_replay_claim_only(self):
        r = self.bound.check_validity(live_from(self.bound, arm_datum='closed_loop:handover_pose'))
        self.assertEqual(r['claims']['command_replay_integration']['active_compatibility'], 'STALE')
        self.assertIn('arm_datum', r['claims']['command_replay_integration']['mismatched'])
        self.assert_unrelated_untouched(arm_datum='closed_loop:handover_pose')

    def test_controller_or_camera_change_never_invalidates_the_wrist_mount(self):
        r = self.bound.check_validity(live_from(self.bound, controller='another controller', camera_mount_sha256='moved'))
        self.assertEqual(r['claims']['command_replay_integration']['active_compatibility'], 'STALE')
        self.assertEqual(sorted(r['claims']['command_replay_integration']['mismatched']), ['camera_mount_sha256', 'controller'])
        self.assertEqual(r['transforms']['right.wrist_to_hand']['status'], 'VALID')
        self.assertEqual(r['claims']['mechanism_checks']['active_compatibility'], 'ACTIVE_COMPATIBLE')

    def test_unbound_manifest_revision_is_unverified_never_active(self):
        r = self.m.check_validity(live_from(self.m, hand_contract_sha256=hand_contract_descriptor(self.contracts), arm_datum=ARM_DATUM_SCRIPTED))
        c = r['claims']['command_replay_integration']
        self.assertEqual(c['active_compatibility'], 'UNVERIFIED')
        self.assertEqual(c['unbound'], ['hand_contract_sha256', 'arm_datum'])
        self.assertFalse(r['valid'])
        r = self.m.check_validity(live_from(self.m, camera_mount_sha256='moved', hand_contract_sha256='x', arm_datum='y'))
        self.assertEqual(r['claims']['command_replay_integration']['active_compatibility'], 'STALE')   # a real mismatch still reads STALE
        for name in HAND_COMMAND_DEPENDENT_CLAIMS:
            if self.m.data['qualification']['claims'][name]['status'] == 'NOT_RUN':
                self.assertEqual(r['claims'][name]['active_compatibility'], 'NOT_APPLICABLE')          # NOT_RUN claims are untouched
        self.assertNotIn('unbound', ()); self.assertEqual(r['claims']['mechanism_checks']['unbound'], [])   # only hand-command claims can be unbound

    def test_missing_live_profile_is_unverified(self):
        live = live_from(self.bound); del live['hand_contract_sha256']
        r = self.bound.check_validity(live)
        self.assertEqual(r['claims']['command_replay_integration']['active_compatibility'], 'UNVERIFIED')
        self.assertEqual(r['claims']['command_replay_integration']['missing'], ['hand_contract_sha256'])


@unittest.skipUnless(URDF.exists(), 'donor URDF not available in this checkout')
class RuntimeValues(unittest.TestCase):
    def test_runtime_values_bind_actual_dt_profile_and_datum(self):
        m = EmbodimentManifest.load(MANIFEST); contracts = {'right': CLOSURE}; bound = bound_manifest(contracts)
        live = runtime_dependency_values(URDF, collision_cooking=COOKING, physics_dt_s=0.005, support='FIXED_PELVIS', controller=CONTROLLER, hand_contracts=contracts, arm_datum=ARM_DATUM_SCRIPTED)
        self.assertEqual(live['physics_dt_s'], '0.005'); self.assertEqual(live['arm_datum'], ARM_DATUM_SCRIPTED)
        self.assertEqual(live['hand_contract_sha256'], hand_contract_descriptor({'right': HandCommandAdapter(m, 'right', CLOSURE)}))
        r = bound.check_validity(live)
        self.assertEqual(r['claims']['command_replay_integration']['active_compatibility'], 'ACTIVE_COMPATIBLE')
        self.assertEqual(r['claims']['mechanism_checks']['active_compatibility'], 'ACTIVE_COMPATIBLE')
        # DIAGNOSTIC refinement: the actual step stales every dt-bound claim by construction, transforms stay valid
        fine = runtime_dependency_values(URDF, collision_cooking=COOKING, physics_dt_s=0.0025, support='FIXED_PELVIS', controller=CONTROLLER, hand_contracts=contracts, arm_datum=ARM_DATUM_SCRIPTED)
        self.assertEqual(fine['physics_dt_s'], '0.0025')
        r = bound.check_validity(fine)
        for name, claim in bound.data['qualification']['claims'].items():
            if claim['status'] != 'NOT_RUN' and 'physics_dt_s' in claim['configuration']:
                self.assertIn('physics_dt_s', r['claims'][name]['mismatched'], name)
        self.assertEqual(r['transforms']['right.wrist_to_hand']['status'], 'VALID')
        with self.assertRaises(ContractError):
            runtime_dependency_values(URDF, collision_cooking=COOKING, physics_dt_s=0.0, support='FIXED_PELVIS', controller=CONTROLLER, hand_contracts=contracts, arm_datum=ARM_DATUM_SCRIPTED)

    def test_packager_admits_diagnostic_dt_with_stale_bindings_on_record_and_refuses_it_as_qualification(self):
        m = EmbodimentManifest.load(MANIFEST)
        spec = replay_commands.synthetic_hand_open_close({n: 0.0 for n in m.body_names})
        bound = bound_manifest(spec['hand_contracts'])
        controller = {'body_home_rad': {n: 0.0 for n in bound.body_names}, 'body_kp_nm_rad': {n: 50.0 for n in bound.body_names}, 'body_kd_nm_s_rad': {n: 2.0 for n in bound.body_names},
                      'hand_kp_nm_rad': 1.0, 'hand_kd_nm_s_rad': 0.05, 'provenance': 'unit test fixture gains (not hardware values)', 'manifest_sha256': bound.sha256}
        fine = runtime_dependency_values(URDF, collision_cooking=COOKING, physics_dt_s=0.0025, support='FIXED_PELVIS', controller=CONTROLLER, hand_contracts=spec['hand_contracts'], arm_datum=ARM_DATUM_SCRIPTED)
        seq, report = replay_commands.validate(bound, spec, controller, live_dependencies=fine)
        self.assertIsNone(seq); self.assertTrue(any('STALE' in p for p in report['problems']))                 # not admissible as a qualification package
        seq, report = replay_commands.validate(bound, spec, controller, live_dependencies=fine, diagnostic=True)
        self.assertIsNotNone(seq); self.assertEqual(report['status'], 'VALID')
        self.assertIn('physics_dt_s', report['diagnostic_stale_bindings'])                                     # admitted, stale bindings on record
        self.assertEqual(report['qualification_validity']['claims']['command_replay_integration']['active_compatibility'], 'STALE')
        # a changed applied profile is refused on the qualification route even at the bound dt
        other = runtime_dependency_values(URDF, collision_cooking=COOKING, physics_dt_s=0.005, support='FIXED_PELVIS', controller=CONTROLLER,
                                          hand_contracts={'right': dict(CLOSURE, saturation_policy='clip_declared')}, arm_datum=ARM_DATUM_SCRIPTED)
        seq, report = replay_commands.validate(bound, spec, controller, live_dependencies=other)
        self.assertIsNone(seq); self.assertTrue(any('hand_contract_sha256' in p for p in report['problems']))


if __name__ == '__main__':
    unittest.main()
