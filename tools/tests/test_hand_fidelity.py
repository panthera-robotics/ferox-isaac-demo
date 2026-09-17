"""CPU-only tests for tools/hand_fidelity: coupling algebra, donor profile geometry/mass, command semantics, load record
and measurement intake. Tests that need the donor URDF/meshes skip when the campaign asset is not in this checkout."""
import copy
import json
import math
import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # tools/ (same convention as test_urdf_kinematics)
from hand_fidelity import command_semantics as cs
from hand_fidelity.coupling import compose, coupling_table, equivalent_layouts, range_limit_findings, read_joints
from hand_fidelity.donor_profile import HandUrdf, read_stl
from hand_fidelity.load_record import load_record, validate_load_record
from hand_fidelity.measurement_intake import IntakeError, validate_intake
from isaac.twin.inspire.embodiment import ContractError, EmbodimentManifest, HandCommandAdapter, ReplaySequence
CARD = ("index", "middle", "ring", "little", "thumb_bend", "thumb_rotation")
import hand_fidelity.command_semantics as cs

ROOT = Path(__file__).resolve().parents[2]
# donor assets live outside the repository (campaign generated/ftp_donor); override with HAND_FIDELITY_DONOR_DIR
DONOR_DIR = Path(os.environ.get('HAND_FIDELITY_DONOR_DIR', ROOT.parent / 'generated/ftp_donor'))
DONOR_URDF = DONOR_DIR / 'g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf'
THUMBCHAIN2_URDF = DONOR_DIR / 'g1_29dof_rev_1_0_with_inspire_hand_FTP_thumbchain2.urdf'
MANIFEST = ROOT / 'isaac/twin/inspire/embodiments/g1_edu29_rh56dftp_donor_v1.json'


def _urdf(tmp, joints_xml, links=('base', 'a', 'b', 'c')):
    p = Path(tmp) / 'chain.urdf'
    p.write_text('<robot name="t">' + ''.join('<link name="%s"/>' % n for n in links) + joints_xml + '</robot>')
    return p


def _joint(name, parent, child, lower, upper, mimic=None, axis='0 0 1'):
    m = '' if mimic is None else '<mimic joint="%s" multiplier="%s" offset="%s"/>' % mimic
    return ('<joint name="%s" type="revolute"><parent link="%s"/><child link="%s"/><origin xyz="0 0.03 0" rpy="0 0 0"/>'
            '<axis xyz="%s"/><limit lower="%s" upper="%s" effort="1" velocity="1"/>%s</joint>') % (name, parent, child, axis, lower, upper, m)


class CouplingAlgebraTests(unittest.TestCase):
    def test_transitive_composition_matches_hand_algebra(self):
        # q_b = a*q_a + b ; q_c = c*q_b + d  ->  q_c = (c*a)*q_a + (c*b + d)
        with tempfile.TemporaryDirectory() as tmp:
            j = read_joints(_urdf(tmp, _joint('a', 'base', 'a', 0, 1) + _joint('b', 'a', 'b', 0, 3, ('a', 0.8, 0.1)) + _joint('c', 'b', 'c', 0, 3, ('b', 0.5, 0.2))))
        driver, A, B, chain = compose(j, 'c')
        self.assertEqual(driver, 'a'); self.assertAlmostEqual(A, 0.5 * 0.8, 12); self.assertAlmostEqual(B, 0.5 * 0.1 + 0.2, 12); self.assertEqual(len(chain), 2)
        for q in (0.0, 0.37, 1.0):
            self.assertAlmostEqual(A * q + B, 0.5 * (0.8 * q + 0.1) + 0.2, 12)

    def test_cycle_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            j = read_joints(_urdf(tmp, _joint('a', 'base', 'a', 0, 1, ('b', 1, 0)) + _joint('b', 'a', 'b', 0, 1, ('a', 1, 0))))
        with self.assertRaises(ValueError):
            compose(j, 'a')

    def test_range_versus_limit_classification(self):
        with tempfile.TemporaryDirectory() as tmp:
            j = read_joints(_urdf(tmp, _joint('a', 'base', 'a', 0, 1) + _joint('b', 'a', 'b', 0, 3, ('a', 1.1, 0)) + _joint('c', 'b', 'c', 0.05, 0.5, ('a', 1.0, 0))))
        f = range_limit_findings(coupling_table(j))
        self.assertEqual(f['b']['status'], 'BOUNDARY'); self.assertIn('BOUNDARY_COINCIDENT_LOW', f['b']['flags'])
        self.assertEqual(f['c']['status'], 'EXCEEDS'); self.assertTrue(any(x.startswith('EXCEEDS_LOW') for x in f['c']['flags']))
        self.assertTrue(any(x.startswith('EXCEEDS_HIGH') for x in f['c']['flags']))

    def test_layout_equivalence_detects_a_changed_multiplier(self):
        with tempfile.TemporaryDirectory() as tmp:
            ja = read_joints(_urdf(tmp, _joint('a', 'base', 'a', 0, 1) + _joint('b', 'a', 'b', 0, 3, ('a', 0.8, 0)) + _joint('c', 'b', 'c', 0, 3, ('b', 0.9, 0))))
            jb = read_joints(_urdf(tmp, _joint('a', 'base', 'a', 0, 1) + _joint('b', 'a', 'b', 0, 3, ('a', 0.8, 0)) + _joint('c', 'b', 'c', 0, 3, ('a', 0.72, 0))))
            jc = read_joints(_urdf(tmp, _joint('a', 'base', 'a', 0, 1) + _joint('b', 'a', 'b', 0, 3, ('a', 0.8, 0)) + _joint('c', 'b', 'c', 0, 3, ('a', 0.7, 0))))
        self.assertTrue(equivalent_layouts(ja, jb)[0]); self.assertFalse(equivalent_layouts(ja, jc)[0])


@unittest.skipUnless(DONOR_URDF.exists(), 'donor URDF not in this checkout')
class DonorAssetTests(unittest.TestCase):
    def test_every_coupled_joint_open_endpoint_touches_its_own_lower_limit(self):
        for side in ('right', 'left'):
            h = HandUrdf(DONOR_URDF, side)
            f = range_limit_findings({k: v for k, v in coupling_table(h.joints).items() if k in h.coupled})
            self.assertEqual(len(f), 6)
            for name, row in f.items():
                self.assertEqual(row['status'], 'BOUNDARY', name); self.assertAlmostEqual(row['margin_low_rad'], 0.0, 9); self.assertGreater(row['margin_high_rad'], 0.02)

    @unittest.skipUnless(THUMBCHAIN2_URDF.exists(), 'thumbchain2 variant not in this checkout')
    def test_thumbchain2_is_the_exact_composed_coupling(self):
        ja, jb = read_joints(DONOR_URDF), read_joints(THUMBCHAIN2_URDF)
        for side in ('right', 'left'):
            ok, info = equivalent_layouts(ja, jb, side + '_')
            self.assertTrue(ok, info.get('differences'))
            self.assertAlmostEqual(info['table_a'][side + '_thumb_4_joint']['composed_multiplier'], 0.9487 * 0.8024, 12)
            self.assertEqual(jb[side + '_thumb_4_joint']['mimic']['joint'], side + '_thumb_2_joint')

    def test_six_independent_actuators_and_chirality_per_side(self):
        for side in ('right', 'left'):
            h = HandUrdf(DONOR_URDF, side)
            self.assertEqual(len(h.independent), 6); self.assertEqual(len(h.coupled), 6); self.assertEqual(h.link_count if hasattr(h, 'link_count') else len(h.hand_links), 30)
            ch = h.chirality(); self.assertTrue(ch['consistent'], ch); self.assertEqual(ch['verdict'], side.upper())

    def test_thumb_rotation_axis_is_parallel_to_the_finger_axis_and_travel(self):
        h = HandUrdf(DONOR_URDF, 'right'); d = h.thumb_rotation_datum()
        self.assertGreater(abs(d['axis_alignment_with_fingers']), 0.99); self.assertAlmostEqual(d['travel_deg'], math.degrees(1.1641), 3)
        self.assertGreater(d['closed']['elevation_from_palm_plane_deg'], d['open']['elevation_from_palm_plane_deg'])

    def test_mass_is_conserved_and_com_moves_with_configuration(self):
        h = HandUrdf(DONOR_URDF, 'right')
        o = h.mass_properties(); c = h.mass_properties(h.actuator_command({a: 1.0 for a in h.actuators}))
        self.assertAlmostEqual(o['mass_kg'], 0.8783, 6); self.assertAlmostEqual(c['mass_kg'], 0.8783, 6); self.assertEqual(o['links_without_inertial'], [])
        self.assertTrue(o['positive_definite']); self.assertGreater(np.linalg.norm(np.array(o['com_m']) - np.array(c['com_m'])), 0.003)
        w = h.mass_properties(in_wrist_frame=True)
        self.assertAlmostEqual(w['mass_kg'], o['mass_kg'], 9)
        # rigid re-expression: principal moments are frame independent
        self.assertTrue(np.allclose(sorted(w['principal_moments_kg_m2']), sorted(o['principal_moments_kg_m2']), atol=1e-12))

    def test_left_right_base_link_inertials_are_not_mirror_images(self):
        # documented donor source asymmetry (Unitree FTP URDF): keep as a regression guard until the source is corrected
        r, l = HandUrdf(DONOR_URDF, 'right'), HandUrdf(DONOR_URDF, 'left')
        rb = r.links['right_base_link'].find('inertial/origin').get('xyz').split(); lb = l.links['left_base_link'].find('inertial/origin').get('xyz').split()
        self.assertGreater(abs(float(rb[2]) - float(lb[2])), 0.010)

    def test_fk_sweep_reports_one_frame_and_monotone_finger_flexion(self):
        h = HandUrdf(DONOR_URDF, 'right'); s = h.fk_sweep()
        self.assertEqual(s['frame'], 'right_base_link')
        tips = [s['all_actuators']['closure_%.2f' % c]['points_m']['index']['tip_sensor_frame'] for c in (0.0, 0.5, 1.0)]
        self.assertGreater(tips[0][2], tips[1][2]); self.assertGreater(tips[1][2], tips[2][2])   # tip descends along the finger axis while closing
        w = h.fk_sweep(in_wrist_frame=True); self.assertEqual(w['frame'], 'right_wrist_yaw_link')
        self.assertAlmostEqual(w['all_actuators']['closure_0.00']['points_m']['index']['tip_sensor_frame'][0], tips[0][2] + 0.0415, 4)

    def test_mesh_extents_when_meshes_present(self):
        h = HandUrdf(DONOR_URDF, 'right')
        if not h.meshes_available():
            self.skipTest('donor meshes not in this checkout')
        e = h.mesh_extents()
        self.assertAlmostEqual(e['index_proximal_segment_m'], 0.0328, 3); self.assertGreater(e['index_mcp_to_distal_tip_m'], 0.08)


class StlReaderTests(unittest.TestCase):
    def test_ascii_and_binary_stl_agree(self):
        tri = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], float)
        with tempfile.TemporaryDirectory() as tmp:
            a = Path(tmp) / 'a.stl'; a.write_text('solid t\nfacet normal 0 0 1\nouter loop\n' + ''.join('vertex %g %g %g\n' % tuple(v) for v in tri) + 'endloop\nendfacet\nendsolid t\n')
            import struct
            b = Path(tmp) / 'b.stl'; b.write_bytes(b'\0' * 80 + struct.pack('<I', 1) + struct.pack('<3f', 0, 0, 1) + b''.join(struct.pack('<3f', *v) for v in tri) + b'\0\0')
            self.assertTrue(np.allclose(read_stl(a), read_stl(b)))


class CommandSemanticsTests(unittest.TestCase):
    def test_native_register_endpoints_and_round_trip(self):
        counts = [1000, 900, 500, 100, 0, 750]
        c, clipped = cs.to_closure('inspire_e2_angle_register_v1', counts)
        self.assertEqual(clipped, []); self.assertAlmostEqual(c['little'], 0.0); self.assertAlmostEqual(c['thumb_bend'], 1.0); self.assertAlmostEqual(c['index'], 0.9)
        self.assertTrue(np.allclose(cs.from_closure('inspire_e2_angle_register_v1', c), counts))
        donor = cs.from_closure('ftp_donor_radians_v1', c)
        self.assertAlmostEqual(donor[0], 0.0); self.assertAlmostEqual(donor[4], 0.5864); self.assertAlmostEqual(donor[3], 0.9 * 1.4381, 9)

    def test_module_agrees_with_manifest_hand_command_adapter(self):
        m = EmbodimentManifest.load(MANIFEST)
        for name in ('inspire_e2_angle_register_v1', 'unitree_inspire_dds_normalized_v1', 'g1_wbt_pick_up_drinks_card_v1', 'unitree_inspire_hand_urdf_radians_v1'):
            conv = cs.convention(name); ad = HandCommandAdapter(m, 'right', cs.adapter_contract(name))
            vec = [cs.endpoints(conv, a)[0] + 0.3 * (cs.endpoints(conv, a)[1] - cs.endpoints(conv, a)[0]) for a in conv['axis_order']]
            targets, _ = ad.to_joint_targets(vec)
            mine = dict(zip(cs.convention('ftp_donor_radians_v1')['axis_order'], cs.convert(name, 'ftp_donor_radians_v1', vec)[0]))
            for axis, q in mine.items():
                self.assertAlmostEqual(targets[m.hand_actuator('right', axis)['joint']], q, 9, name)

    def test_dds_normalized_one_is_open_and_maps_to_urdf_lower_limits(self):
        v, r = cs.convert('unitree_inspire_dds_normalized_v1', 'unitree_inspire_hand_urdf_radians_v1', [1.0] * 6)
        self.assertTrue(np.allclose(v, [0, 0, 0, 0, 0, -0.1]))           # thumb yaw open endpoint is -0.1 rad in that URDF
        v, r = cs.convert('unitree_inspire_dds_normalized_v1', 'unitree_inspire_hand_urdf_radians_v1', [0.0] * 6)
        self.assertTrue(np.allclose(v, [1.7, 1.7, 1.7, 1.7, 0.5, 1.3]))

    def test_piston_thumb_yaw_minus_0p1_is_the_open_endpoint_not_a_clip(self):
        v, r = cs.convert('unitree_inspire_hand_urdf_radians_v1', 'ftp_donor_radians_v1', [1.3, 1.3, 1.3, 1.3, 0.0, -0.1])
        self.assertEqual(r['clipped_axes'], []); self.assertAlmostEqual(v[5], 0.0); self.assertAlmostEqual(r['closure']['thumb_rotation'], 0.0)
        self.assertAlmostEqual(v[0], 1.3 / 1.7 * 1.4381, 9)

    def test_radian_identity_reports_its_departure_and_refuses_out_of_range(self):
        v, r = cs.convert('unitree_inspire_hand_urdf_radians_v1', 'ftp_donor_radians_v1', [1.3, 1.3, 1.3, 1.3, 0.0, 0.0], policy='radian_identity')
        self.assertAlmostEqual(r['radian_identity_minus_closure_preserving_rad']['index'], 1.3 - 1.3 / 1.7 * 1.4381, 9)
        with self.assertRaises(ContractError):
            cs.convert('unitree_inspire_hand_urdf_radians_v1', 'ftp_donor_radians_v1', [1.3, 1.3, 1.3, 1.3, 0.0, -0.1], policy='radian_identity')
        with self.assertRaises(ContractError):
            cs.convert('inspire_e2_angle_register_v1', 'ftp_donor_radians_v1', [1000] * 6, policy='radian_identity')
        t = cs.endpoint_discrepancy_table('unitree_inspire_hand_urdf_radians_v1', 'ftp_donor_radians_v1')
        self.assertAlmostEqual(t['index']['identity_error_at_source_closed_rad'], 1.7 - 1.4381, 9); self.assertAlmostEqual(t['thumb_rotation']['identity_error_at_source_open_rad'], -0.1, 9)

    def test_refusals_never_pad_truncate_or_accept_nonfinite(self):
        for bad in ([1000] * 7, [1000] * 5, [float('nan')] + [500] * 5, [float('inf')] + [500] * 5, [True] + [500] * 5, [1001] + [500] * 5, [-1] + [500] * 5):
            with self.assertRaises(ContractError):
                cs.to_closure('inspire_e2_angle_register_v1', bad)
        with self.assertRaises(ContractError):
            cs.convention('dex3_seven_channel')
        with self.assertRaises(ContractError):
            cs.from_closure('ftp_donor_radians_v1', {'index': 0.5})
        c, clipped = cs.to_closure('inspire_e2_angle_register_v1', [1000.4] + [500] * 5, tolerance=0.5)
        self.assertEqual(clipped, ['little']); self.assertAlmostEqual(c['little'], 0.0)

    def test_unverified_axes_are_named_per_convention(self):
        self.assertEqual(cs.unsupported_axes('unitree_inspire_dds_normalized_v1'), [])
        self.assertIn('axis_order', cs.unsupported_axes('g1_wbt_pick_up_drinks_card_v1'))
        self.assertIn('direction_thumb_rotation', cs.unsupported_axes('inspire_e2_angle_register_v1'))


class LoadRecordTests(unittest.TestCase):
    def _hand(self):
        return {'mass_kg': 0.8783, 'com_m': [0.138, 0.001, 0.005], 'inertia_about_com_kg_m2': (np.eye(3) * 2e-3).tolist(), 'frame': 'right_wrist_yaw_link', 'provenance': 'URDF', 'configuration': 'open'}

    def test_hand_plus_tool_composes_mass_and_com_and_keeps_unknowns_null(self):
        rec = load_record('right', hand=self._hand(), wrist_adapter=None, tool={'mass_kg': 0.05, 'com_m': [0.20, 0.0, 0.0], 'inertia_about_com_kg_m2': None, 'provenance': 'declared'})
        validate_load_record(rec)
        self.assertAlmostEqual(rec['totals']['hand_and_tool']['mass_kg'], 0.9283, 9)
        self.assertAlmostEqual(rec['totals']['hand_and_tool']['com_m'][0], (0.8783 * 0.138 + 0.05 * 0.20) / 0.9283, 9)
        self.assertIsNone(rec['totals']['hand_and_tool']['inertia_about_com_kg_m2'])       # a null tool inertia never becomes zero
        self.assertIsNone(rec['components']['wrist_adapter']); self.assertIsNone(rec['totals']['hand_adapter_and_tool'])

    def test_validation_rejects_zero_filled_unknowns_and_bad_frames(self):
        rec = load_record('right', hand=self._hand(), wrist_adapter=None, tool=None)
        bad = copy.deepcopy(rec); bad['components']['wrist_adapter'] = {'mass_kg': 0.0, 'com_m': [0, 0, 0], 'inertia_about_com_kg_m2': None, 'provenance': 'assumed', 'frame': 'right_wrist_yaw_link'}
        with self.assertRaises(ValueError):
            validate_load_record(bad)
        bad = copy.deepcopy(rec); bad['components']['hand']['frame'] = 'right_base_link'
        with self.assertRaises(ValueError):
            validate_load_record(bad)
        bad = copy.deepcopy(rec); bad['components']['hand']['inertia_about_com_kg_m2'] = (np.eye(3) * -1e-3).tolist()
        with self.assertRaises(ValueError):
            validate_load_record(bad)


class MeasurementIntakeTests(unittest.TestCase):
    def _doc(self, **over):
        d = {'schema_version': 2, 'side': 'right', 'hardware_model': 'RH56E2-2R-T1', 'measured_utc': '2026-09-20T10:00:00Z', 'operator': 'named operator',
             'evidence_class': 'installed_measurement', 'split': 'held_out_validation', 'datums': {'palm_reference': 'flange face', 'angle_zero': 'readback 1000'},
             'source_evidence': {'files': [{'name': 'palm.jpg', 'sha256': 'a' * 64}]},
             'measurements': [{'key': 'palm_width_mm', 'value': 85.0, 'uncertainty': 0.2, 'unit': 'mm', 'repeats': [84.9, 85.0, 85.1]},
                              {'key': 'hand_mass_g', 'value': None, 'uncertainty': 5.0, 'unit': 'g', 'repeats': []}],
             'command_response': {'per_axis': [{'axis': 'index', 'target_counts': [1000, 500], 'readback_counts': [998, 503], 'timestamps_s': [0.0, 0.5]}]}}
        d.update(over); return d

    def test_valid_document_passes_with_coverage_report(self):
        rep = validate_intake(self._doc(), min_measured=1)
        self.assertEqual(rep['measured_keys'], ['palm_width_mm']); self.assertEqual(rep['evidence_class'], 'installed_measurement')

    def test_rejects_nonfinite_negative_uncertainty_units_dims_hashes_and_coverage(self):
        with self.assertRaises(IntakeError):
            validate_intake(self._doc(measurements=[{'key': 'palm_width_mm', 'value': float('nan'), 'uncertainty': 0.2, 'unit': 'mm', 'repeats': []}]))
        with self.assertRaises(IntakeError):
            validate_intake(self._doc(measurements=[{'key': 'palm_width_mm', 'value': 85.0, 'uncertainty': -0.2, 'unit': 'mm', 'repeats': []}]))
        with self.assertRaises(IntakeError):
            validate_intake(self._doc(measurements=[{'key': 'palm_width_mm', 'value': 85.0, 'uncertainty': 0.2, 'unit': 'cm', 'repeats': []}]))
        with self.assertRaises(IntakeError):
            validate_intake(self._doc(command_response={'per_axis': [{'axis': 'index', 'target_counts': [1000, 500], 'readback_counts': [998], 'timestamps_s': [0.0, 0.5]}]}))
        with self.assertRaises(IntakeError):
            validate_intake(self._doc(source_evidence={'files': [{'name': 'palm.jpg', 'sha256': 'not-a-hash'}]}))
        with self.assertRaises(IntakeError):
            validate_intake(self._doc(), min_measured=2)
        with self.assertRaises(IntakeError):
            validate_intake(self._doc(evidence_class='synthetic_fixture'))
        rep = validate_intake(self._doc(evidence_class='synthetic_fixture'), allow_synthetic=True)
        self.assertTrue(rep['synthetic'])

    def test_repeats_must_agree_with_the_reported_value(self):
        with self.assertRaises(IntakeError):
            validate_intake(self._doc(measurements=[{'key': 'palm_width_mm', 'value': 85.0, 'uncertainty': 0.2, 'unit': 'mm', 'repeats': [80.0, 80.1, 80.2]}]))


if __name__ == '__main__':
    unittest.main()


@unittest.skipUnless(DONOR_URDF.exists(), 'donor URDF not in this checkout')
class CandidateInvalidationTests(unittest.TestCase):
    """Item 7: a candidate profile change must mark the dependent qualifications STALE, computed on a temp copy only."""
    LIVE = {'support': 'FIXED_PELVIS', 'physics_dt_s': '0.005', 'solver': 'TGS_32_8', 'hand_drive_gains': 'kp=2.0;kd=0.5',
            'grasp_sha256': 'e80befb00b34cae04d2ab4ba8b750b41d94138689c5534b24831a94f9e31ce88', 'source_image': 'sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54',
            'tool_frame': 'measured in sF-writer-contact-15-v13-measure (simulator, invalidated by grasp/mount/asset change)', 'controller': 'implicit_biased_drive_v1 replay controller (package-hashed gains)',
            'probe_config_sha256': '7052810259bd935a0477c48c6481b90b75e7b239fe097cce27b758e3a7b38c9c'}

    def test_widened_coupled_limits_change_the_coupling_map_and_stale_every_bound_claim(self):
        from hand_fidelity.candidate_invalidation import invalidation_report, widen_coupled_lower_limits
        r = invalidation_report(MANIFEST, DONOR_URDF, widen_coupled_lower_limits(0.02), live_extra=self.LIVE)
        self.assertEqual(r['changed_dependencies'], ['coupling_map_sha256', 'urdf_sha256'])
        self.assertEqual(r['original']['mechanism_checks'], 'ACTIVE_COMPATIBLE')
        for name, status in r['candidate'].items():
            if r['original'][name] != 'NOT_APPLICABLE':
                self.assertEqual(status, 'STALE', name)
        self.assertEqual(r['transforms_candidate']['right.wrist_to_hand'], 'INVALID')   # bound to urdf_sha256 as well

    def test_mirrored_base_link_inertial_changes_only_the_urdf_hash_but_still_stales_claims(self):
        from hand_fidelity.candidate_invalidation import invalidation_report, mirror_base_link_inertial
        r = invalidation_report(MANIFEST, DONOR_URDF, mirror_base_link_inertial(), live_extra=self.LIVE)
        self.assertEqual(r['changed_dependencies'], ['urdf_sha256'])
        self.assertTrue(all(s == 'STALE' for n, s in r['candidate'].items() if r['original'][n] != 'NOT_APPLICABLE'))

    def test_active_asset_and_manifest_are_untouched(self):
        import hashlib
        from hand_fidelity.candidate_invalidation import invalidation_report, widen_coupled_lower_limits
        before = hashlib.sha256(DONOR_URDF.read_bytes()).hexdigest(), hashlib.sha256(MANIFEST.read_bytes()).hexdigest()
        invalidation_report(MANIFEST, DONOR_URDF, widen_coupled_lower_limits(0.05), live_extra=self.LIVE)
        after = hashlib.sha256(DONOR_URDF.read_bytes()).hexdigest(), hashlib.sha256(MANIFEST.read_bytes()).hexdigest()
        self.assertEqual(before, after)


@unittest.skipUnless(DONOR_URDF.exists(), 'donor URDF not in this checkout')
class UnitScalingTests(unittest.TestCase):
    def test_donor_meshes_are_in_metres_and_unscaled(self):
        h = HandUrdf(DONOR_URDF, 'right')
        if not h.meshes_available():
            self.skipTest('donor meshes not in this checkout')
        e = h.mesh_extents()
        self.assertEqual(e['units_check']['status'], 'METRES_PLAUSIBLE'); self.assertEqual(e['mesh_scale_attributes'], ['1 1 1'])
        self.assertAlmostEqual(e['open_length_along_fingers_m'], 0.25, delta=0.02)


class AbSourceSpecTests(unittest.TestCase):
    """Paired hand-map import-policy specs: identical rows, two contracts, predictable converted endpoints, refusals."""

    def _admitted(self):
        # synthetic donor-closure schedule in the admitted card order: open margin -> plateau 0.9 -> open margin (like rod30 v2)
        mo = [0.013907, 0.013907, 0.013907, 0.013907, 0.034106, 0.017181]
        rows = []
        for i, (stage, c) in enumerate([('retract', mo), ('close', [0.45, 0.45, 0.45, 0.45, 0.034106, 0.017181]), ('close', [0.9, 0.9, 0.9, 0.9, 0.55, 0.5]),
                                        ('hold', [0.9, 0.9, 0.9, 0.9, 0.55, 0.5]), ('open', [0.45, 0.45, 0.45, 0.45, 0.275, 0.25]), ('open', mo)]):
            rows.append({'t_s': round(i / 30.0, 6), 'stage': stage, 'body_q_rad': {'right_elbow_joint': 0.5}, 'hands': {'right': c, 'left': None}})
        return {'source': {'source_id': 'synthetic-admitted', 'kind': 'synthetic_test_sequence', 'provenance': 'unit test'},
                'hand_contracts': {'right': {'axis_order': list(CARD), 'open_value': 0.0, 'closed_value': 1.0, 'saturation_policy': 'reject'}}, 'rows': rows, 'maximum_step_s': 0.2}

    def test_pair_rows_identical_and_converted_finger_difference_is_0p20_rad(self):
        from hand_fidelity.ab_source_specs import build_pair, expected_converted
        pair, deriv = build_pair(self._admitted(), source_label='t')
        a, b = pair['closure_preserving'], pair['radian_identity']
        self.assertEqual(json.dumps(a['rows'], sort_keys=True), json.dumps(b['rows'], sort_keys=True))
        self.assertEqual(a['rows'][2]['hands']['right'][:4], [1.3] * 4)                      # plateau = the piston dataset grasp value
        m = EmbodimentManifest.load(MANIFEST)
        ta, _ = expected_converted(a, m, at_row=2); tb, _ = expected_converted(b, m, at_row=2)
        self.assertAlmostEqual(tb['right_index_1_joint'] - ta['right_index_1_joint'], 1.3 - 1.3 / 1.7 * 1.4381, 6)   # 0.2003 rad
        self.assertAlmostEqual(tb['right_index_1_joint'], 1.3, 9); self.assertAlmostEqual(ta['right_index_1_joint'], 1.0997, 4)
        self.assertAlmostEqual(tb['right_thumb_2_joint'], 0.55 * 0.5864, 6); self.assertAlmostEqual(tb['right_thumb_1_joint'], 0.5 * 1.1641, 6)   # B reproduces the admitted thumb exactly
        oa, _ = expected_converted(a, m, at_row=0); ob, _ = expected_converted(b, m, at_row=0)
        for t in (oa, ob):
            for j, q in t.items():
                self.assertGreaterEqual(q, 0.02 - 1e-9, j)                                        # operating margin respected under both policies
        for spec in (a, b):
            seq = ReplaySequence(m, spec['rows'], hand_contracts=spec['hand_contracts'], source=spec['source'], maximum_step_s=spec['maximum_step_s'])
            self.assertEqual(seq.summary()['clipped_rows'], [])
        self.assertEqual(deriv['finger_source_plateau_rad'], 1.3); self.assertGreaterEqual(deriv['finger_source_open_rad'], 0.02)

    def test_identity_contract_refuses_the_dataset_thumb_yaw_open_endpoint_and_closure_contract_maps_it_to_open(self):
        from hand_fidelity.ab_source_specs import contract
        m = EmbodimentManifest.load(MANIFEST)
        row = [0.5, 0.5, 0.5, 0.5, 0.0, -0.1]
        with self.assertRaises(ContractError):
            HandCommandAdapter(m, 'right', contract('radian_identity')).to_joint_targets(row)
        t, info = HandCommandAdapter(m, 'right', contract('closure_preserving')).to_joint_targets(row)
        self.assertAlmostEqual(t['right_thumb_1_joint'], 0.0, 9); self.assertEqual(info['clipped_axes'], []); self.assertAlmostEqual(info['closure']['thumb_rotation'], 0.0, 9)

    def test_free_sweep_rows_hold_the_arm_and_replay_close_then_open(self):
        from hand_fidelity.ab_source_specs import free_sweep_rows
        rows = free_sweep_rows(self._admitted()['rows'], rate_hz=30.0, open_hold_s=0.1, plateau_hold_s=0.1, tail_hold_s=0.1)
        stages = [r['stage'] for r in rows]
        self.assertEqual(stages[:3], ['hold_open'] * 3); self.assertIn('close', stages); self.assertIn('open', stages); self.assertEqual(stages[-1], 'hold_open_end')
        self.assertTrue(all(r['body_q_rad'] == {'right_elbow_joint': 0.5} for r in rows))
        ts = [r['t_s'] for r in rows]; self.assertTrue(all(b > a for a, b in zip(ts, ts[1:])))

    @unittest.skipUnless(os.environ.get('HAND_FIDELITY_ADMITTED_SPEC'), 'private admitted spec not provided')
    def test_private_admitted_spec_pair_is_valid(self):
        from hand_fidelity.ab_source_specs import build_pair
        adm = json.loads(Path(os.environ['HAND_FIDELITY_ADMITTED_SPEC']).read_text())
        pair, _ = build_pair({k: adm[k] for k in ('source', 'hand_contracts', 'rows', 'maximum_step_s')}, source_label='t')
        m = EmbodimentManifest.load(MANIFEST)
        for spec in pair.values():
            self.assertEqual(ReplaySequence(m, spec['rows'], hand_contracts=spec['hand_contracts'], source=spec['source'], maximum_step_s=spec['maximum_step_s']).summary()['clipped_rows'], [])


class ConversionProfileTests(unittest.TestCase):
    """Opt-in typed conversion profile: types, bounds kinds, evidence, refusals, margins, trace, dependencies."""

    def _profile(self, **kw):
        from hand_fidelity.conversion_profile import ConversionProfile, PISTON_ROUTE_PROVENANCE
        m = EmbodimentManifest.load(MANIFEST)
        args = dict(profile_id='piston-closure-v1', side='right', source_convention='unitree_inspire_hand_urdf_radians_v1', manifest=m, policy='closure_preserving',
                    operating_margin_rad=0.02, provenance=PISTON_ROUTE_PROVENANCE, urdf_sha256='63097d73')
        args.update(kw)
        return ConversionProfile(**args)

    def test_trace_types_bounds_evidence_and_margin(self):
        p = self._profile()
        tr = p.apply([1.3, 1.3, 1.3, 1.3, 0.0, -0.1], side='right', declared_order=cs.NATIVE_ORDER, declared_type='radians')
        self.assertEqual(tr['raw_type'], 'radians'); self.assertEqual(tr['clipped_axes'], []); self.assertEqual(tr['interventions'], [])
        self.assertAlmostEqual(tr['closure']['thumb_rotation'], 0.0, 9)                                       # -0.1 is that model's open endpoint
        self.assertAlmostEqual(tr['target_rad_unmargined']['right_thumb_1_joint'], 0.0, 9)
        self.assertAlmostEqual(tr['effective_rad']['right_thumb_1_joint'], 0.02, 9)                           # operating margin applied at the open end
        self.assertAlmostEqual(tr['margin_applied']['right_thumb_1_joint'], 0.02, 9)
        self.assertAlmostEqual(tr['effective_rad']['right_index_1_joint'], 1.3 / 1.7 * 1.4381, 9)
        self.assertIn('coordinate_endpoint', tr['bounds']['index']['target_bound_kind']); self.assertIn('operating_margin', tr['bounds']['index']['margin_kind'])
        self.assertEqual(tr['evidence']['thumb_rotation']['direction'], 'VERIFIED_SOURCE'); self.assertEqual(tr['evidence']['index']['scale'], 'UNRESOLVED')   # datum identity unverified

    def test_refusals_side_order_type_dimension_nonfinite_range(self):
        p = self._profile()
        good = [0.5, 0.5, 0.5, 0.5, 0.1, 0.3]
        with self.assertRaises(ContractError):
            p.apply(good, side='left')
        with self.assertRaises(ContractError):
            p.apply(good, side='right', declared_order=cs.CARD_ORDER)
        with self.assertRaises(ContractError):
            p.apply(good, side='right', declared_type='closure')
        with self.assertRaises(ContractError):
            p.apply(good[:5], side='right')
        with self.assertRaises(ContractError):
            p.apply([float('nan')] + good[1:], side='right')
        with self.assertRaises(ContractError):
            p.apply([1.8, 0.5, 0.5, 0.5, 0.1, 0.3], side='right')                        # above the source range under a reject policy
        with self.assertRaises(ContractError):
            self._profile(require_verified=True)                                          # scale UNRESOLVED -> a qualified profile cannot be built
        with self.assertRaises(ContractError):
            self._profile(policy='radian_identity', exploratory=False, declared_clip_tolerance=0.1)   # a clip needs an exploratory profile

    def test_radian_identity_profile_refuses_the_dataset_thumb_yaw_unless_a_clip_is_declared(self):
        pid = self._profile(profile_id='piston-identity-v1', policy='radian_identity')
        with self.assertRaises(ContractError):
            pid.apply([1.3, 1.3, 1.3, 1.3, 0.0, -0.1], side='right')
        clip = self._profile(profile_id='piston-identity-clip-v1', policy='radian_identity', declared_clip_tolerance=0.1)
        tr = clip.apply([1.3, 1.3, 1.3, 1.3, 0.0, -0.1], side='right')
        self.assertEqual(tr['clipped_axes'], ['thumb_rotation']); self.assertEqual(tr['interventions'][0]['kind'], 'declared_clip')
        self.assertAlmostEqual(tr['effective_rad']['right_index_1_joint'], 1.3, 9); self.assertAlmostEqual(tr['effective_rad']['right_thumb_1_joint'], 0.02, 9)
        self.assertNotEqual(pid.sha256, clip.sha256)

    def test_monotone_round_trip_and_endpoints(self):
        p = self._profile()
        last = -1.0
        for q in (0.0, 0.3, 0.85, 1.3, 1.7):
            eff = p.apply([q] * 4 + [0.0, -0.1], side='right')['effective_rad']['right_index_1_joint']
            self.assertGreater(eff, last); last = eff
        self.assertAlmostEqual(p.apply([1.7] * 4 + [0.5, 1.3], side='right')['target_rad_unmargined']['right_index_1_joint'], 1.4381, 9)
        c, _ = cs.to_closure('unitree_inspire_hand_urdf_radians_v1', [0.85] * 4 + [0.25, 0.6])
        back = cs.from_closure('unitree_inspire_hand_urdf_radians_v1', c)
        self.assertTrue(all(abs(a - b) < 1e-12 for a, b in zip(back, [0.85] * 4 + [0.25, 0.6])))

    @unittest.skipUnless(DONOR_URDF.exists(), 'donor URDF not in this checkout')
    def test_coupled_joints_stay_inside_their_limits_with_the_margin(self):
        p = self._profile()
        tr = p.apply([1.7, 1.7, 1.7, 1.7, 0.5, 1.3], side='right')
        cc = p.coupled_consistency(tr, DONOR_URDF)
        self.assertEqual(len(cc), 6); self.assertTrue(all(v['inside'] for v in cc.values()), cc)
        tr0 = p.apply([0.0, 0.0, 0.0, 0.0, 0.0, -0.1], side='right'); cc0 = p.coupled_consistency(tr0, DONOR_URDF)
        self.assertTrue(all(v['margin_to_lower_rad'] > 0.015 for v in cc0.values()), cc0)   # margin keeps the coupled joints off their lower limits

    def test_dependencies_and_profile_hash_change_with_policy_and_margin(self):
        a = self._profile(); b = self._profile(operating_margin_rad=0.03); c = self._profile(policy='radian_identity')
        self.assertEqual(len({a.sha256, b.sha256, c.sha256}), 3)
        d = a.dependencies(); self.assertEqual(d['manifest_sha256'], a.manifest.sha256); self.assertIn('invalidates_when_changed', d)
        self.assertEqual(a.describe()['axis_order'], list(cs.NATIVE_ORDER))


class LoadContractTests(unittest.TestCase):
    def _bundle(self):
        import numpy as np
        def hand(side, x, y):
            I = [[6.6e-4, 0, 0], [0, 2.1e-3, 0], [0, 0, 2.6e-3]]
            return {'mass_kg': 0.8783, 'com_m': [x, y, 0.005], 'inertia_about_com_kg_m2': I, 'provenance': 'URDF', 'frame': side + '_wrist_yaw_link'}
        recs = {}
        for side, y in (('right', 0.0012), ('left', -0.0008)):
            recs[side] = {}
            for cfg, x in (('open', 0.138 if side == 'right' else 0.1454), ('closed', 0.1316 if side == 'right' else 0.139)):
                h = hand(side, x, y)
                recs[side][cfg] = {'frame': side + '_wrist_yaw_link', 'components': {'hand': h, 'wrist_adapter': None, 'tool': None},
                                   'totals': {'hand_only': {'mass_kg': h['mass_kg'], 'com_m': h['com_m'], 'inertia_about_com_kg_m2': h['inertia_about_com_kg_m2']}, 'hand_and_adapter': None, 'hand_and_tool': None, 'hand_adapter_and_tool': None}}
        return {'schema': 'hand_load_record_bundle_v1', 'records': recs}

    def test_valid_bundle_reports_sides_and_asymmetry(self):
        from hand_fidelity.load_contract import validate_bundle
        r = validate_bundle(self._bundle())
        self.assertEqual(r['sides']['right']['open']['mass_kg'], 0.8783); self.assertAlmostEqual(r['sides']['right']['open']['gravity_moment_at_horizontal_extension_N_m'], 9.81 * 0.8783 * 0.138, 3)
        self.assertIn('ASYMMETRIC', r['mirror_consistency']['open']['status']); self.assertIn('never_add_when', r['consumer_rules'])

    def test_parallel_axis_and_reexpression_round_trips(self):
        import numpy as np
        from hand_fidelity.load_contract import parallel_axis, reexpress
        I = np.diag([1e-3, 2e-3, 2.5e-3]); m, r = 0.8, np.array([0.1, 0.02, -0.01])
        Io = parallel_axis(I, m, r)
        self.assertTrue(np.allclose(Io - m * ((r @ r) * np.eye(3) - np.outer(r, r)), I))
        th = 0.3; R = np.array([[np.cos(th), 0, np.sin(th)], [0, 1, 0], [-np.sin(th), 0, np.cos(th)]])
        self.assertTrue(np.allclose(reexpress(reexpress(I, R), R.T), I)); self.assertTrue(np.allclose(np.linalg.eigvalsh(reexpress(I, R)), np.linalg.eigvalsh(I)))

    def test_rejections(self):
        from hand_fidelity.load_contract import LoadContractError, validate_bundle
        b = self._bundle(); b['records']['right']['open']['components']['hand']['inertia_about_com_kg_m2'] = [[1e-3, 0, 0], [0, 1e-3, 0], [0, 0, 3e-3]]   # triangle inequality
        with self.assertRaises(LoadContractError):
            validate_bundle(b)
        b = self._bundle(); b['records']['right']['open']['components']['wrist_adapter'] = {'mass_kg': 0.05, 'com_m': [0.02, 0, 0], 'inertia_about_com_kg_m2': None, 'provenance': 'assumed', 'frame': 'right_wrist_yaw_link'}
        with self.assertRaises(LoadContractError):
            validate_bundle(b)
        b = self._bundle(); b['records']['left']['closed']['components']['hand']['mass_kg'] = 0.9
        with self.assertRaises(LoadContractError):
            validate_bundle(b)   # open/closed are samples of one rigid body
        b = self._bundle(); b['records']['right']['open']['totals']['hand_and_tool'] = b['records']['right']['open']['totals']['hand_only']
        with self.assertRaises(LoadContractError):
            validate_bundle(b)   # a total that needs an unknown component must be null

    @unittest.skipUnless(DONOR_URDF.exists(), 'donor URDF not in this checkout')
    def test_actual_configuration_load_sits_between_the_samples_for_finger_closure(self):
        from hand_fidelity.load_contract import compare_with_samples, load_at_configuration
        h = HandUrdf(DONOR_URDF, 'right')
        b = self._bundle()
        for side in ('right', 'left'):
            hh = HandUrdf(DONOR_URDF, side)
            for cfg, c in (('open', 0.0), ('closed', 1.0)):
                mp = hh.mass_properties(hh.actuator_command({a: c for a in hh.actuators}), in_wrist_frame=True)
                b['records'][side][cfg]['components']['hand'].update(mass_kg=mp['mass_kg'], com_m=mp['com_m'], inertia_about_com_kg_m2=mp['inertia_about_com_kg_m2'])
                b['records'][side][cfg]['totals']['hand_only'] = {'mass_kg': mp['mass_kg'], 'com_m': mp['com_m'], 'inertia_about_com_kg_m2': mp['inertia_about_com_kg_m2']}
        from hand_fidelity.load_contract import validate_bundle
        validate_bundle(b)
        half = load_at_configuration(h, h.actuator_command({a: 0.5 for a in ('index', 'middle', 'ring', 'little')}))
        self.assertTrue(compare_with_samples(b, 'right', half)['between_samples'])


@unittest.skipUnless(DONOR_URDF.exists(), 'donor URDF not in this checkout')
class AbAnalysisTests(unittest.TestCase):
    """The A/B trace comparator on a tiny synthetic evidence pair: aligned stages, measured/commanded deltas, contact sets, object-in-palm."""

    def _evidence(self, tmp, name, finger_rad, contact_links):
        from hand_fidelity.donor_profile import HandUrdf
        h = HandUrdf(DONOR_URDF, 'right')
        d = Path(tmp) / name; d.mkdir()
        names = h.hand_joints + ['right_shoulder_pitch_joint']
        rows, cmds, objs, cons = [], [], [], []
        for i in range(12):
            t = 0.005 * (i + 1); stage_row = 0 if i < 6 else 1
            q = h.joint_values({h.actuators['index']: finger_rad, h.actuators['middle']: finger_rad, h.actuators['ring']: finger_rad, h.actuators['little']: finger_rad})
            qv = [q[n] if n in q else 0.0 for n in names]
            rows.append({'sequence': i, 'physics_s': t, 'wall_s': t, 'phase': 'replay', 'source_row': stage_row, 'source_t_s': stage_row * 0.03, 'runtime_names': names, 'q_rad': qv, 'dq_rad_s': [0.0] * len(names),
                         'measured_generalized_effort_nm': [0.01] * len(names), 'body_command_names': [], 'body_command_rad': [],
                         'hand_command_names': [h.actuators[a] for a in ('index', 'middle', 'ring', 'little', 'thumb_bend', 'thumb_rotation')], 'hand_command_rad': [finger_rad] * 4 + [0.02, 0.02],
                         'link_poses_world_xyzw': {'right_base_link': [0.2, -0.15, 1.1, 0, 0, 0, 1]}, 'coupling_error_rad': {'right_index_2_joint': 0.001}, 'body_feedforward': {}})
            objs.append({'sequence': i, 'physics_s': t, 'phase': 'replay', 'source_row': stage_row, 'pose_world_xyzw': [0.23, -0.12, 1.2, 0, 0, 0, 1], 'linear_velocity_m_s': [0, 0, 0], 'angular_velocity_rad_s': [0, 0, 0], 'right_palm_pose_world_xyzw': [0.2, -0.15, 1.1, 0, 0, 0, 1]})
            for link in contact_links:
                cons.append({'sequence': i, 'physics_s': t, 'phase': 'replay', 'actor0': '/World/Scene/Object', 'actor1': '/World/G1/' + link, 'position_world_m': [0, 0, 0], 'normal_world': [0, 0, 1], 'impulse_ns': [0, 0, 0.1], 'separation_m': 0.0, 'source': 'simulated_proxy'})
        for sr in (0, 1):
            cmds.append({'sequence': sr, 'physics_s': 0.005 * (1 + 6 * sr), 'source_row': sr, 'source_t_s': sr * 0.03, 'body_targets_rad': {}, 'hand_targets_rad': {'right': {}}, 'hand_closure': {'right': {}}, 'clipped_axes': {'right': []}})
        (d / 'state.jsonl').write_text('\n'.join(json.dumps(r) for r in rows) + '\n'); (d / 'commands.jsonl').write_text('\n'.join(json.dumps(c) for c in cmds) + '\n')
        (d / 'object.jsonl').write_text('\n'.join(json.dumps(o) for o in objs) + '\n'); (d / 'contacts.jsonl').write_text('\n'.join(json.dumps(c) for c in cons) + '\n')
        (d / 'task_eval_v2.json').write_text(json.dumps({'C1_grasp': {'pass': True}, 'C2_lift_retention': {'pass': name == 'B'}, 'C3_place': {'pass': True}, 'C4_release': {'pass': True}, 'C5_prohibited': {'pass': True}, 'overall': 'PASS' if name == 'B' else 'FAIL'}))
        spec = d / 'spec.json'; spec.write_text(json.dumps({'rows': [{'stage': 'close'}, {'stage': 'hold'}]}))
        return d, spec, h

    def test_compare_reports_measured_delta_contacts_and_object_offset(self):
        from hand_fidelity.ab_analysis import compare, summarize_run
        with tempfile.TemporaryDirectory() as tmp:
            a, spec, h = self._evidence(tmp, 'A', 1.0997, ['right_base_link'])
            b, _, _ = self._evidence(tmp, 'B', 1.3, ['right_base_link', 'right_index_2', 'right_middle_2'])
            sa, sb = summarize_run(a, h, source_spec=spec), summarize_run(b, h, source_spec=spec)
            self.assertEqual(sa['phases'], ['close', 'hold']); self.assertEqual(sa['contact_links_in_hold'], ['right_base_link']); self.assertEqual(len(sb['contact_links_in_hold']), 3)
            self.assertEqual(sa['per_phase']['hold']['object_in_palm_m'], [0.03, 0.03, 0.1])
            c = compare(sa, sb)
            self.assertAlmostEqual(c['phases']['hold']['measured_end_rad_B_minus_A']['right_index_1_joint'], 0.2003, 3)
            self.assertGreater(abs(c['phases']['hold']['fingertip_B_minus_A_m']['index_tip_sensor'][2]), 0.01)
            self.assertEqual(c['task_eval']['A']['overall'], 'FAIL'); self.assertEqual(c['task_eval']['B']['overall'], 'PASS')
