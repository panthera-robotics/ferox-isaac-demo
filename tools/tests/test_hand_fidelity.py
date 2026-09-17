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
from isaac.twin.inspire.embodiment import ContractError, EmbodimentManifest, HandCommandAdapter

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
