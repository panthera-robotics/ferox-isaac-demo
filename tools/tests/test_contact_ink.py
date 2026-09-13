"""Adversarial measured-contact evaluation checks; no simulator or private task import."""
import csv
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from isaac.twin.inspire.contact_ink import (
    ContactSample, IntendedStroke, InvalidTrace, MarkingRule,
    evaluate, export_csv, export_svg, samples_from_columns,
    _capsule_intervals, _union_length,
)


def trace(strokes, contacts=True, offset=(0.0, 0.0), count=21):
    samples = []
    for stroke in strokes:
        for index, (a,b) in enumerate(zip(stroke.points_board_m, stroke.points_board_m[1:])):
            for i in range(count):
                fraction = i/(count-1)
                seq = len(samples)
                samples.append(ContactSample(
                    physics_sequence=seq, physics_time_s=seq*.005, physics_dt_s=.005,
                    nib_position_board_m=(a[0]+fraction*(b[0]-a[0])+offset[0],
                                          a[1]+fraction*(b[1]-a[1])+offset[1], 0.0),
                    nib_board_contact=contacts, pen_down=True, spring_compression_m=.005,
                    stroke_id=stroke.stroke_id, segment_index=index, reference_fraction=fraction,
                    attachment_active=False, fixture_support_active=False, holder_bottomed_out=False,
                    normal_force_n=1.0 if contacts else 0.0))
    return samples


class ContactInkTests(unittest.TestCase):
    def setUp(self):
        self.strokes = [IntendedStroke('L', ((0.,0.), (.1,0.), (.1,.1)))]
        self.samples = trace(self.strokes)

    def test_measured_ordered_contacts_cover_each_segment(self):
        report = evaluate(self.strokes, self.samples)
        self.assertTrue(report['valid']); self.assertTrue(report['accepted'])
        self.assertAlmostEqual(report['coverage_fraction'], 1.0)
        self.assertEqual([s['coverage_fraction'] for s in report['segments']], [1.0, 1.0])
        self.assertEqual(report['path_error_p95_m'], 0.0)
        self.assertEqual(report['endpoint_errors_m'], {'L:start': 0.0, 'L:end': 0.0})
        self.assertEqual(report['corner_errors_m'], {'L:corner1': 0.0})
        self.assertFalse(report['physics_grasp_or_standing_certified'])
        json.dumps(report, allow_nan=False)

    def test_perfect_requested_path_without_contacts_has_zero_ink_and_coverage(self):
        report = evaluate(self.strokes, trace(self.strokes, contacts=False))
        self.assertTrue(report['valid']); self.assertFalse(report['accepted'])
        self.assertEqual(report['path_error_max_m'], 0.0)
        self.assertEqual(report['coverage_fraction'], 0.0)
        self.assertEqual(report['ink_paths_board_m'], [])

    def test_continuous_contact_between_separate_strokes_is_visible_unintended_ink(self):
        strokes = [IntendedStroke('A', ((0.,0.),(.1,0.))),
                   IntendedStroke('B', ((0.,.1),(.1,.1)))]
        report = evaluate(strokes, trace(strokes))
        # Every recorded point is exactly on its requested stroke, but the actual
        # continuous contact draws a diagonal joining the two strokes.
        self.assertTrue(report['valid'])
        self.assertEqual(report['coverage_fraction'], 1.0)
        self.assertEqual(report['path_error_max_m'], 0.0)
        self.assertFalse(report['accepted'])
        self.assertIn('unintended_contact_bridge', report['failure_reasons'])
        self.assertAlmostEqual(report['unintended_contact_bridge_extent_m'], math.sqrt(.02))
        self.assertEqual(report['pen_up_marked_samples'], 0)
        self.assertEqual(report['ink_paths_board_m'][0][20:22], [(0.1,0.0),(0.0,0.1)])

    def test_observed_lift_between_strokes_breaks_ink_and_passes(self):
        strokes = [IntendedStroke('A', ((0.,0.),(.1,0.))),
                   IntendedStroke('B', ((0.,.1),(.1,.1)))]
        samples = trace(strokes)
        samples.insert(21, replace(samples[20], nib_board_contact=False, pen_down=False,
                                   normal_force_n=0.0))
        # This constructs a complete synthetic physics trace with one actual
        # contact-free sample, rather than dropping a sample from existing data.
        samples = [replace(s, physics_sequence=i, physics_time_s=i*.005)
                   for i,s in enumerate(samples)]
        report = evaluate(strokes, samples)
        self.assertTrue(report['accepted'])
        self.assertEqual(report['unintended_contact_bridge_extent_m'], 0.0)
        self.assertEqual(len(report['ink_paths_board_m']), 2)

    def test_corner_shortcut_is_measured_between_samples_not_hidden_by_reference_labels(self):
        strokes = [IntendedStroke('L', ((0.,0.),(1.,0.),(1.,1.)))]
        samples = [s for s in trace(strokes, count=101)
                   if not (s.segment_index == 0 and s.reference_fraction == 1.0)
                   and not (s.segment_index == 1 and s.reference_fraction == 0.0)]
        samples = [replace(s, physics_sequence=i, physics_time_s=i*.005)
                   for i,s in enumerate(samples)]
        report = evaluate(strokes, samples)
        self.assertTrue(report['valid'])
        self.assertEqual(report['path_error_max_m'], 0.0)
        self.assertGreater(report['coverage_fraction'], .95)
        self.assertFalse(report['accepted'])
        self.assertIn('unintended_contact_bridge', report['failure_reasons'])
        self.assertAlmostEqual(report['unintended_contact_bridge_extent_m'], .004*math.sqrt(2))
        ordinary = evaluate(self.strokes, self.samples)
        self.assertTrue(ordinary['accepted'])
        self.assertEqual(ordinary['unintended_contact_bridge_extent_m'], 0.0)

    def test_unavailable_contacts_are_invalid_not_measured_zero(self):
        samples = [replace(s, nib_board_contact=None, normal_force_n=None) for s in self.samples]
        report = evaluate(self.strokes, samples)
        self.assertFalse(report['valid']); self.assertFalse(report['accepted'])
        self.assertIn('contact_observation_unavailable', report['invalid_reasons'])
        self.assertEqual(report['coverage_fraction'], 0.0)

    def test_unavailable_force_cannot_create_ink(self):
        report = evaluate(self.strokes, [replace(s, normal_force_n=None) for s in self.samples])
        self.assertFalse(report['valid']); self.assertEqual(report['marked_samples'], 0)

    def test_wrong_order_cannot_score_by_nearest_other_letter(self):
        # Deliberately coincident strokes: nearest-any-path would be perfect.
        strokes = [IntendedStroke('A', ((0.,0.),(.1,0.))), IntendedStroke('B', ((0.,0.),(.1,0.)))]
        report = evaluate(strokes, trace(list(reversed(strokes))))
        self.assertEqual(report['path_error_max_m'], 0.0)
        self.assertFalse(report['accepted'])
        self.assertIn('stroke_order_violation', report['failure_reasons'])
        self.assertEqual(report['segments'][1]['coverage_fraction'], 0.0)

    def test_reversed_reference_progress_is_an_order_failure(self):
        samples = list(self.samples)
        samples[10] = replace(samples[10], reference_fraction=.1)
        report = evaluate(self.strokes, samples)
        self.assertIn('stroke_order_violation', report['failure_reasons'])

    def test_error_uses_synchronized_reference_fraction(self):
        stroke = [IntendedStroke('I', ((0.,0.),(.1,0.)))]
        samples = trace(stroke)
        # All measured points are on the intended segment, but most are 20 mm late.
        samples = [replace(s, nib_position_board_m=(max(0.,s.nib_position_board_m[0]-.02),0.,0.)) for s in samples]
        report = evaluate(stroke, samples)
        self.assertAlmostEqual(report['path_error_p95_m'], .02)
        self.assertIn('path_error_exceeds_target', report['failure_reasons'])

    def test_geometric_coverage_does_not_count_time_or_duplicate_locations(self):
        stroke = [IntendedStroke('I', ((0.,0.),(.1,0.)))]
        samples = [replace(s, nib_position_board_m=(0.,0.,0.)) for s in trace(stroke, count=501)]
        report = evaluate(stroke, samples)
        self.assertLess(report['coverage_fraction'], .031)
        self.assertFalse(report['accepted'])

    def test_missing_short_segment_cannot_hide_in_total_length(self):
        strokes = [IntendedStroke('long', ((0.,0.),(1.,0.))),
                   IntendedStroke('short', ((0.,.01),(.01,.01)))]
        report = evaluate(strokes, trace(strokes[:1], count=501))
        self.assertGreater(report['coverage_fraction'], .95)
        self.assertFalse(report['accepted'])
        self.assertEqual(report['segments'][1]['coverage_fraction'], 0.)

    def test_contact_gap_breaks_visible_ink_and_coverage(self):
        stroke = [IntendedStroke('I', ((0.,0.),(.1,0.)))]
        samples = trace(stroke)
        samples[7:14] = [replace(s, nib_board_contact=False, normal_force_n=0.) for s in samples[7:14]]
        report = evaluate(stroke, samples)
        self.assertEqual(len(report['ink_paths_board_m']), 2)
        self.assertLess(report['coverage_fraction'], .8)

    def test_pen_up_contact_still_draws_and_fails_leakage(self):
        samples = list(self.samples)
        samples[5:8] = [replace(s, pen_down=False, stroke_id=None, segment_index=None,
                                reference_fraction=None) for s in samples[5:8]]
        report = evaluate(self.strokes, samples)
        self.assertEqual(report['marked_samples'], len(samples))
        self.assertEqual(report['pen_up_marked_samples'], 3)
        self.assertGreater(report['pen_up_leakage_extent_m'], .01)
        self.assertIn('pen_up_ink_leakage', report['failure_reasons'])

    def test_a_stationary_pen_up_dot_is_not_zero_leakage(self):
        sample = replace(self.samples[0], pen_down=False, stroke_id=None,
                         segment_index=None, reference_fraction=None)
        report = evaluate(self.strokes, [sample])
        self.assertAlmostEqual(report['pen_up_leakage_extent_m'], MarkingRule().ink_width_m)
        self.assertIn('pen_up_ink_leakage', report['failure_reasons'])

    def test_attachment_support_or_bottomout_invalidates_writing(self):
        for name in ('attachment_active', 'fixture_support_active', 'holder_bottomed_out'):
            with self.subTest(name=name):
                samples=list(self.samples); samples[10]=replace(samples[10], **{name:True})
                report=evaluate(self.strokes, samples)
                self.assertFalse(report['accepted']); self.assertIn(name, report['failure_reasons'])
        samples=list(self.samples); samples[10]=replace(samples[10], spring_compression_m=.020)
        self.assertIn('holder_bottomed_out', evaluate(self.strokes,samples)['failure_reasons'])

    def test_missing_qualification_flags_do_not_default_to_no_cheating(self):
        for field in ('attachment_active', 'fixture_support_active', 'holder_bottomed_out'):
            samples=list(self.samples);samples[0]=replace(samples[0], **{field:None})
            report=evaluate(self.strokes,samples)
            self.assertFalse(report['valid']);self.assertFalse(report['accepted'])

    def test_force_window_and_board_plane_are_checked(self):
        for replacement, reason in [({'normal_force_n':21.}, 'normal_force_outside_window'),
                                    ({'nib_position_board_m':(0.,0.,.01)}, 'contact_point_off_board_plane')]:
            samples=list(self.samples);samples[0]=replace(samples[0],**replacement)
            report=evaluate(self.strokes,samples)
            self.assertFalse(report['accepted'])
            self.assertIn(reason, report['failure_reasons']+report['invalid_reasons'])

    def test_impulse_is_converted_using_the_physics_step(self):
        samples=[replace(s, normal_force_n=None, normal_impulse_ns=.005) for s in self.samples]
        report=evaluate(self.strokes,samples)
        self.assertTrue(report['accepted']);self.assertEqual(report['normal_force_max_n'],1.)
        with self.assertRaises(InvalidTrace): replace(samples[0], normal_force_n=2.)
        with self.assertRaises(InvalidTrace):
            replace(samples[0], normal_impulse_ns=1e308, physics_dt_s=1e-308)

    def test_nan_inf_and_boolean_numbers_refused(self):
        for bad in (math.nan, math.inf, -math.inf, True):
            for field in ('physics_time_s','physics_dt_s','spring_compression_m','normal_force_n'):
                with self.subTest(field=field, bad=bad), self.assertRaises(InvalidTrace):
                    replace(self.samples[0], **{field:bad})
        with self.assertRaises(InvalidTrace): replace(self.samples[0], nib_position_board_m=(math.nan,0.,0.))
        with self.assertRaises(InvalidTrace): MarkingRule(coverage_tolerance_m=math.inf)

    def test_repeated_reordered_or_reset_physics_sample_refused(self):
        for bad in (self.samples[0], replace(self.samples[1], physics_time_s=0.)):
            with self.assertRaises(InvalidTrace): evaluate(self.strokes, [self.samples[0], bad])

    def test_missing_physics_samples_invalidate_and_break_line(self):
        report=evaluate(self.strokes, self.samples[:10]+self.samples[11:])
        self.assertFalse(report['valid']);self.assertEqual(len(report['ink_paths_board_m']),2)
        self.assertIn('missing_physics_samples', report['invalid_reasons'])

    def test_physics_time_and_declared_step_must_agree(self):
        samples=list(self.samples);samples[1]=replace(samples[1],physics_time_s=.007)
        report=evaluate(self.strokes,samples)
        self.assertFalse(report['valid']);self.assertIn('physics_step_time_mismatch',report['invalid_reasons'])

    def test_capsule_coverage_clips_to_measured_endpoints(self):
        intervals=_capsule_intervals((0.,0.),(1.,0.),(.2,0.),(.4,0.),.1)
        self.assertAlmostEqual(_union_length(intervals),.4)
        # Crossing a target stroke only covers a small neighbourhood of the crossing.
        intervals=_capsule_intervals((0.,0.),(1.,0.),(.4,-1.),(.4,1.),.003)
        self.assertAlmostEqual(_union_length(intervals),.006)

    def test_mismatched_recording_columns_are_not_truncated(self):
        rows=[asdict(s) for s in self.samples]
        columns={name:[row[name] for row in rows] for name in rows[0]}
        self.assertEqual(samples_from_columns(columns),tuple(self.samples))
        columns['normal_force_n']=columns['normal_force_n'][:-1]
        with self.assertRaises(InvalidTrace): samples_from_columns(columns)

    def test_unknown_segment_and_duplicate_stroke_identity_refused(self):
        with self.assertRaises(InvalidTrace):
            evaluate(self.strokes,[replace(self.samples[0], segment_index=20)])
        with self.assertRaises(InvalidTrace):
            evaluate(self.strokes+self.strokes,self.samples)

    def test_missing_endpoint_samples_stay_explicitly_unmeasured(self):
        samples=[s for s in self.samples if s.reference_fraction not in (0.,1.)]
        # Renumbering is deliberately avoided: absent records remain a recording gap.
        report=evaluate(self.strokes,samples)
        self.assertEqual(report['endpoint_errors_m'],{'L:start':None,'L:end':None})
        self.assertEqual(report['corner_errors_m'],{'L:corner1':None})

    def test_empty_trace_reports_missing_metrics(self):
        report=evaluate(self.strokes, [])
        self.assertFalse(report['valid']);self.assertIsNone(report['path_error_p95_m'])
        self.assertIsNone(report['normal_force_max_n']);self.assertEqual(report['coverage_fraction'],0.)
        json.dumps(report,allow_nan=False)

    def test_exports_contain_measured_contacts_only_and_preserve_unknowns(self):
        with tempfile.TemporaryDirectory() as directory:
            svg=Path(directory)/'ink.svg';csv_file=Path(directory)/'contacts.csv'
            empty=evaluate(self.strokes,trace(self.strokes,contacts=False));export_svg(empty,svg)
            tree=ET.parse(svg)
            self.assertEqual(len(tree.findall('.//{http://www.w3.org/2000/svg}polyline')),0)
            measured=evaluate(self.strokes,self.samples);export_svg(measured,svg)
            tree=ET.parse(svg)
            self.assertGreater(len(tree.findall('.//{http://www.w3.org/2000/svg}polyline')),0)
            self.assertIn('plumbing artifact',svg.read_text())
            export_csv([replace(self.samples[0],nib_board_contact=None,normal_force_n=None)],csv_file)
            with csv_file.open() as stream: row=next(csv.DictReader(stream))
            self.assertEqual(row['nib_board_contact'],'');self.assertEqual(row['normal_force_n'],'')


if __name__ == '__main__': unittest.main(verbosity=2)
