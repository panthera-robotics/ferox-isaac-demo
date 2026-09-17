"""Adversarial replay tests for the PROPOSED pen-up ink specification (OWNER_REVIEW_REQUIRED).
Every case shows the old (authoritative) score unchanged next to the proposal on the identical trace."""
import sys
from dataclasses import replace
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from isaac.twin.inspire.contact_ink import ContactSample, IntendedStroke, MarkingRule, evaluate
from isaac.twin.inspire.contact_ink_proposal import PenUpProposal, evaluate_v2, STATUS

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_contact_ink import trace


def pen_up(seq, t, xy, compression, contact=True):
    return ContactSample(physics_sequence=seq, physics_time_s=t, physics_dt_s=.005, nib_position_board_m=(xy[0], xy[1], 0.), nib_board_contact=contact,
                         pen_down=False, spring_compression_m=compression, attachment_active=False, fixture_support_active=False, holder_bottomed_out=False,
                         normal_force_n=(1.0 if contact else 0.0))


class PenUpProposalTests(unittest.TestCase):
    def setUp(self):
        self.strokes = [IntendedStroke('I', ((0., .01), (0., -.01)))]
        self.samples = trace(self.strokes)
        self.t_end = self.samples[-1].physics_time_s; self.n = len(self.samples)

    def unloading(self, radius_m=0.0, steps=20, drift=0.0):
        """Stationary (or drifting) unloading ink at the endpoint after the pen-up flag: compression decays 5 -> 0 mm."""
        out = []
        for i in range(steps):
            comp = .005 * (1 - i / steps)
            out.append(pen_up(self.n + i, self.t_end + .005 * (i + 1), (radius_m + drift * i, -.01), comp, contact=comp > 0))
        return out

    def test_status_and_old_score_untouched(self):
        report = evaluate_v2(self.strokes, self.samples + self.unloading())
        old = evaluate(self.strokes, self.samples + self.unloading())
        self.assertEqual(report['proposal']['status'], STATUS)
        for key in ('accepted', 'pen_up_leakage_extent_m', 'coverage_fraction', 'path_error_p95_m', 'failure_reasons'):
            self.assertEqual(report[key], old[key])

    def test_spring_unloading_at_the_endpoint_is_allowed_only_by_the_proposal(self):
        report = evaluate_v2(self.strokes, self.samples + self.unloading())
        self.assertFalse(report['accepted']); self.assertIn('pen_up_ink_leakage', report['failure_reasons'])   # old rule: any pen-up mark fails
        self.assertEqual(report['proposal']['pen_up_leakage_v2_m'], 0.0)
        self.assertGreater(report['proposal']['allowed_unloading_ink_m'], 0.0)
        self.assertTrue(report['proposal']['accepted_under_proposal'])

    def test_unloading_ink_that_travels_away_from_the_endpoint_is_leakage(self):
        report = evaluate_v2(self.strokes, self.samples + self.unloading(drift=.0005))   # 0.5 mm per step -> leaves the 1 mm capsule
        self.assertGreater(report['proposal']['pen_up_leakage_v2_m'], 0.0)
        self.assertFalse(report['proposal']['accepted_under_proposal'])

    def test_ink_after_the_unloading_window_is_leakage_even_at_the_endpoint(self):
        late = [pen_up(self.n + i, self.t_end + .5 + .005 * i, (0., -.01), .004) for i in range(5)]
        report = evaluate_v2(self.strokes, self.samples + late)
        self.assertGreater(report['proposal']['pen_up_leakage_v2_m'], 0.0)

    def test_re_press_without_compression_decay_is_not_unloading(self):
        pressed = [pen_up(self.n + i, self.t_end + .005 * (i + 1), (0., -.01), .0065) for i in range(5)]   # above the unloading compression bound
        report = evaluate_v2(self.strokes, self.samples + pressed)
        self.assertGreater(report['proposal']['pen_up_leakage_v2_m'], 0.0)

    def test_bridge_between_strokes_stays_forbidden(self):
        strokes = [IntendedStroke('A', ((0., 0.), (0., .01))), IntendedStroke('B', ((.02, 0.), (.02, .01)))]
        samples = trace([strokes[0]]); n = len(samples); t = samples[-1].physics_time_s
        bridge = [pen_up(n + i, t + .005 * (i + 1), (0.002 * (i + 1), .01), .004) for i in range(9)]   # drags ink 2..18 mm toward B
        after = trace([strokes[1]]); after = [replace(s, physics_sequence=n + 9 + s.physics_sequence, physics_time_s=t + .05 + .005 * (s.physics_sequence + 1)) for s in after]
        report = evaluate_v2(strokes, samples + bridge + after)
        self.assertGreater(report['proposal']['pen_up_leakage_v2_m'] + report['proposal']['unintended_bridge_v2_m'], 0.0)
        self.assertFalse(report['proposal']['accepted_under_proposal'])

    def test_proposal_never_rescues_a_geometric_failure(self):
        report = evaluate_v2(self.strokes, trace(self.strokes, offset=(.005, 0.)) + self.unloading(radius_m=.005))
        self.assertFalse(report['accepted']); self.assertFalse(report['proposal']['accepted_under_proposal'])

    def test_parameters_are_bounded(self):
        for bad in (dict(settle_radius_m=0.), dict(unloading_window_s=float('nan')), dict(maximum_unloading_compression_m=-1.)):
            with self.assertRaises(ValueError): PenUpProposal(**bad)
