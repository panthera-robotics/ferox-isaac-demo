"""Build paired replay source specs for the hand-map import-policy diagnostic.

Starting from an admitted donor-closure source spec (rows whose right-hand values are the donor's internal closure
c in [0, 1], card order index, middle, ring, little, thumb_bend, thumb_rotation), the same hand schedule is re-expressed
as SOURCE-MODEL radians (the Unitree `inspire_hand` joint space that Unitree teleop / Isaac Lab / the piston dataset use)
and emitted twice with two declared import contracts for `replay_commands.py`:

  A  closure_preserving : contract endpoints = the source model's own limits (fingers 0..1.7, thumb pitch 0..0.5,
                          thumb yaw -0.1..1.3) -> HandCommandAdapter maps closure to the donor limits;
  B  radian_identity    : contract endpoints = the donor's own limits -> the source radian is taken as a donor radian.

Both contracts are EXPLORATORY (datum identity between the two Unitree models is unverified); the rows are byte-identical
between A and B — only the contract differs — so any runtime difference is the import policy. The source finger schedule
is scaled so its grasp plateau is exactly `finger_plateau_rad` (1.3 rad = the value recorded in the piston dataset) and
its open margin maps to >= the declared operating margin under BOTH policies. Thumb source values equal the admitted
donor radians (so B reproduces the admitted thumb clamp exactly).
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path

from . import command_semantics as cs

CARD_ORDER = ('index', 'middle', 'ring', 'little', 'thumb_bend', 'thumb_rotation')
FINGERS = ('index', 'middle', 'ring', 'little')


def _sha(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def derive_source_rows(donor_rows, *, finger_plateau_rad=1.3, margin_rad=0.02, src_limits=cs.UNITREE_INSPIRE_HAND_URDF_LIMITS, donor_limits=cs.FTP_DONOR_LIMITS):
    """Donor-closure rows (card order) -> source-model radian rows (card order). Returns (rows, derivation)."""
    fc = [r['hands']['right'][0] for r in donor_rows if r['hands'].get('right')]
    c_open, c_plateau = min(fc), max(fc)
    if not (0.0 <= c_open < c_plateau <= 1.0):
        raise ValueError('finger closure schedule must open below its plateau')
    # source open margin: >= margin under B (identity) and under A (closure over the source range)
    src_open = max(margin_rad, margin_rad * src_limits['index'][1] / donor_limits['index'][1])
    src_open = math.ceil(src_open * 1e6) / 1e6      # rounded UP so the 6-decimal row value still maps to >= the margin under both policies
    rows = []
    for r in donor_rows:
        rr = copy.deepcopy(r)
        if rr['hands'].get('right') is None:
            rows.append(rr); continue
        c = rr['hands']['right']
        out = []
        for axis, ci in zip(CARD_ORDER, c):
            if axis in FINGERS:
                q = src_open + (ci - c_open) / (c_plateau - c_open) * (finger_plateau_rad - src_open)
            else:
                lo, hi = donor_limits[axis]
                q = lo + ci * (hi - lo)          # admitted donor radian, read as a source radian (B reproduces it exactly)
            out.append(round(q, 6))
        rr['hands']['right'] = out
        rows.append(rr)
    return rows, {'finger_source_open_rad': round(src_open, 6), 'finger_source_plateau_rad': finger_plateau_rad, 'donor_closure_open': c_open, 'donor_closure_plateau': c_plateau,
                  'finger_map': 'q_src = src_open + (c - c_open)/(c_plateau - c_open) * (plateau - src_open)', 'thumb_map': 'q_src = donor_lower + c * (donor_upper - donor_lower) (numerically the admitted donor radian)'}


def contract(policy, *, src_limits=cs.UNITREE_INSPIRE_HAND_URDF_LIMITS, donor_limits=cs.FTP_DONOR_LIMITS):
    if policy == 'closure_preserving':
        per = {a: {'open_value': lo, 'closed_value': hi} for a, (lo, hi) in src_limits.items()}
        note = 'EXPLORATORY closure-preserving import: source radians in the Unitree inspire_hand joint space (limits 1.7 / 0.5 / [-0.1, 1.3]); closure = (q - open)/(closed - open) mapped onto the donor limits'
    elif policy == 'radian_identity':
        per = {a: {'open_value': lo, 'closed_value': hi} for a, (lo, hi) in donor_limits.items()}
        note = 'EXPLORATORY radian-identity import: the source radian is taken as a donor radian (assumes identical joint datums/travels between the inspire_hand model and the FTP donor — UNVERIFIED); values outside the donor range are refused'
    else:
        raise ValueError('policy must be closure_preserving or radian_identity')
    sem = {a: {'direction': 'VERIFIED', 'order': 'VERIFIED', 'scale': 'INFERRED'} for a in CARD_ORDER}
    return {'axis_order': list(CARD_ORDER), 'open_value': 0.0, 'closed_value': 1.7 if policy == 'closure_preserving' else 1.4381, 'per_axis_endpoints': per,
            'saturation_policy': 'reject', 'axis_semantics': sem, 'import_policy': policy, 'note': note,
            'scale_note': 'scale INFERRED: which policy matches the physical E2 is unverified; a qualified route must refuse both until resolved (replay_commands --require-verified-hand-semantics)'}


def build_pair(admitted_spec, *, source_label, finger_plateau_rad=1.3, margin_rad=0.02, provenance_extra=''):
    """Two specs (A closure_preserving, B radian_identity) with identical rows. Returns {policy: spec} and the derivation."""
    rows, deriv = derive_source_rows(admitted_spec['rows'], finger_plateau_rad=finger_plateau_rad, margin_rad=margin_rad)
    rows_sha = _sha(rows)
    out = {}
    for policy, tag in (('closure_preserving', 'A'), ('radian_identity', 'B')):
        spec = {'source': dict(admitted_spec['source']), 'hand_contracts': {'right': contract(policy)}, 'rows': rows}
        if 'maximum_step_s' in admitted_spec:
            spec['maximum_step_s'] = admitted_spec['maximum_step_s']
        spec['source']['source_id'] = '%s-%s-%s' % (source_label, tag, policy)
        spec['source']['kind'] = 'synthetic_test_sequence'
        spec['source']['provenance'] = ('hand-map import-policy diagnostic %s (%s): arm rows and stage timing copied from the admitted spec %s; right-hand values '
                                        're-expressed as Unitree inspire_hand-space radians (finger plateau %.3f rad = piston dataset grasp value, open margin %.4f rad; thumb = admitted donor radians); '
                                        'rows sha256 %s are identical in A and B, only the import contract differs. EXPLORATORY: not a recording, not model-generated, not a claim about the installed hand. %s'
                                        % (tag, policy, admitted_spec['source'].get('source_id'), finger_plateau_rad, deriv['finger_source_open_rad'], rows_sha[:16], provenance_extra)).strip()
        out[policy] = spec
    return out, dict(deriv, rows_sha256=rows_sha)


def expected_converted(spec, manifest, *, at_row):
    """Converted donor targets for one row through the manifest adapter (what the admitted pipeline will send)."""
    from isaac.twin.inspire.embodiment import HandCommandAdapter
    ad = HandCommandAdapter(manifest, 'right', spec['hand_contracts']['right'])
    targets, info = ad.to_joint_targets(spec['rows'][at_row]['hands']['right'])
    return targets, info


def write_specs(pair, out_dir, *, stem):
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    paths = {}
    for policy, spec in pair.items():
        tag = 'A' if policy == 'closure_preserving' else 'B'
        p = out_dir / ('%s-%s-%s.json' % (stem, tag, policy))
        p.write_text(json.dumps(spec, indent=1) + '\n')
        paths[policy] = {'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
    return paths


def free_sweep_rows(admitted_rows, *, hold_pose_stage='retract', rate_hz=30.0, open_hold_s=1.0, plateau_hold_s=2.0, tail_hold_s=1.0):
    """Hand-only sweep for the no-object stage: the arm is held at the admitted sequence's last `hold_pose_stage` pose
    while the right hand runs the admitted close schedule, holds its plateau, runs the admitted open schedule and holds.
    Rows keep the admitted card-order values (donor closure or source radians, whichever the caller passes)."""
    pose_rows = [r for r in admitted_rows if r['stage'] == hold_pose_stage]
    if not pose_rows:
        raise ValueError('no rows with stage %r' % hold_pose_stage)
    body = dict(pose_rows[-1]['body_q_rad'])
    closing = [r for r in admitted_rows if r['stage'] == 'close']
    opening = [r for r in admitted_rows if r['stage'] == 'open']
    if not closing or not opening:
        raise ValueError('admitted rows need close and open stages')
    first_open = admitted_rows[0]['hands']['right']
    dt = 1.0 / rate_hz
    rows, t = [], 0.0

    def add(stage, hand):
        nonlocal t
        rows.append({'t_s': round(t, 6), 'stage': stage, 'body_q_rad': dict(body), 'hands': {'right': list(hand), 'left': None}}); t += dt

    for _ in range(int(round(open_hold_s * rate_hz))):
        add('hold_open', first_open)
    for r in closing:
        add('close', r['hands']['right'])
    for _ in range(int(round(plateau_hold_s * rate_hz))):
        add('hold_closed', closing[-1]['hands']['right'])
    for r in opening:
        add('open', r['hands']['right'])
    for _ in range(int(round(tail_hold_s * rate_hz))):
        add('hold_open_end', opening[-1]['hands']['right'])
    return rows
