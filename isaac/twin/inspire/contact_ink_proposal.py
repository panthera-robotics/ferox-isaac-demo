"""PROPOSED (OWNER_REVIEW_REQUIRED) pen-up ink specification for a spring-loaded nib — NOT a gate.

The accepted rule (`contact_ink.MarkingRule`, allowed_pen_up_leakage_m = 0) counts every marked sample
after the planner's pen-up flag, including the physically unavoidable contact while a 5 mm-compressed
spring nib unloads at the planner's lift speed (0.1-0.3 s), and the ink-width minimum of the first
pen-up mark. That makes the zero-leakage gate unpassable for any compliant marker independent of
tracking quality. This module proposes a replacement that still forbids all *unwanted* ink:

  pen-up ink is ALLOWED only while (a) the nib is unloading (compression still > 0) within
  `unloading_window_s` after the pen-up flag AND (b) the mark stays inside a capsule of radius
  `settle_radius_m` around the intended endpoint of the stroke that just ended;
  pre-pen-down ink is ALLOWED only within `settle_radius_m` of the next stroke's intended start and
  within `unloading_window_s` before the pen-down flag.
  Everything else counts as leakage/bridge with the old zero allowance.

`evaluate_v2` returns the OLD report unchanged plus a `proposal` block; nothing here changes
`contact_ink.evaluate`, and no probe or gate imports this module. Units: metres, seconds.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields

from .contact_ink import InvalidTrace, MarkingRule, _finite, evaluate

PROPOSAL_VERSION = 'pen_up_v2_proposal_2026-09-17'
STATUS = 'OWNER_REVIEW_REQUIRED'


@dataclass(frozen=True)
class PenUpProposal:
    settle_radius_m: float = 0.0010          # endpoint capsule for allowed unloading ink (below the 3 mm coverage tolerance)
    unloading_window_s: float = 0.40         # spring travel / retract speed with margin: 0.005 m at >= 0.02 m/s after the accel ramp
    maximum_unloading_compression_m: float = 0.0060   # ink during unloading must come from a compressed nib, not a re-press

    def __post_init__(self):
        if any(not _finite(getattr(self, f.name)) or getattr(self, f.name) <= 0 for f in fields(self)):
            raise InvalidTrace('proposal parameters must be finite and positive')


def _endpoint(strokes, stroke_id, start):
    for s in strokes:
        if s.stroke_id == stroke_id:
            return s.points_board_m[0] if start else s.points_board_m[-1]
    return None


def evaluate_v2(strokes, samples, rule=MarkingRule(), proposal=PenUpProposal()):
    """Old report (authoritative) + proposed pen-up/bridge accounting on the SAME trace."""
    report = evaluate(strokes, samples, rule)
    ordered = sorted(samples, key=lambda s: s.physics_sequence)
    last_stroke = None; last_pen_up_time = None; last_pen_down = None
    next_stroke_by_time = {}
    # next stroke start (for pre-pen-down contact) per sample index
    upcoming = None
    for i in range(len(ordered) - 1, -1, -1):
        if ordered[i].pen_down:
            upcoming = (ordered[i].stroke_id, ordered[i].physics_time_s)
        next_stroke_by_time[i] = upcoming
    allowed_unloading, leakage_v2, bridge_v2, allowed_prepress = 0.0, 0.0, 0.0, 0.0
    previous = None
    for i, s in enumerate(ordered):
        marked = bool(s.nib_board_contact) and (s.force_n is None or s.force_n >= rule.minimum_force_n)
        if s.pen_down:
            last_stroke = s.stroke_id; last_pen_up_time = None; last_pen_down = s
        elif last_pen_down is not None and last_pen_up_time is None:
            # the pen-up flag time is the first physics step after the last pen-down sample, not the first observed mark
            last_pen_up_time = last_pen_down.physics_time_s + last_pen_down.physics_dt_s
        if marked and not s.pen_down:
            # a first pen-up mark is a dot of at least one ink width; later contiguous marks add their travel
            fresh = previous is None or not previous.nib_board_contact or previous.pen_down
            step = rule.ink_width_m if fresh else math.dist(previous.nib_position_board_m[:2], s.nib_position_board_m[:2])
            allowed = False
            if last_stroke is not None and last_pen_up_time is not None:
                end = _endpoint(strokes, last_stroke, start=False)
                within_time = s.physics_time_s - last_pen_up_time <= proposal.unloading_window_s
                near_end = end is not None and math.dist(s.nib_position_board_m[:2], end) <= proposal.settle_radius_m
                unloading = 0.0 < s.spring_compression_m <= proposal.maximum_unloading_compression_m
                if within_time and near_end and unloading:
                    allowed = True; allowed_unloading += step
            nxt = next_stroke_by_time.get(i)
            if not allowed and nxt is not None:
                start = _endpoint(strokes, nxt[0], start=True)
                if start is not None and nxt[1] - s.physics_time_s <= proposal.unloading_window_s and math.dist(s.nib_position_board_m[:2], start) <= proposal.settle_radius_m:
                    allowed = True; allowed_prepress += step
            if not allowed:
                if nxt is not None and last_stroke is not None and nxt[0] != last_stroke:
                    bridge_v2 += step
                else:
                    leakage_v2 += step
        previous = s
    accepted_v2 = bool(report.get('valid')) and not ((set(report.get('failure_reasons', [])) - {'pen_up_ink_leakage', 'unintended_contact_bridge'})) \
        and leakage_v2 <= rule.allowed_pen_up_leakage_m and bridge_v2 <= rule.allowed_unintended_bridge_m
    report['proposal'] = {'version': PROPOSAL_VERSION, 'status': STATUS, 'parameters': {f.name: getattr(proposal, f.name) for f in fields(proposal)},
                          'allowed_unloading_ink_m': allowed_unloading, 'allowed_prepress_ink_m': allowed_prepress,
                          'pen_up_leakage_v2_m': leakage_v2, 'unintended_bridge_v2_m': bridge_v2,
                          'old_pen_up_leakage_m': report.get('pen_up_leakage_extent_m'), 'old_unintended_bridge_m': report.get('unintended_contact_bridge_extent_m'),
                          'old_accepted': report.get('accepted'), 'accepted_under_proposal': accepted_v2,
                          'note': 'proposal only; the authoritative gate remains the old rule until the owner reviews this specification'}
    return report
