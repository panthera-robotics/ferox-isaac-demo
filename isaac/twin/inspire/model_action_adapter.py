"""Named, versioned adapter from a learned-policy action chunk to the twin's command contract — VALIDATE-ONLY.

Version ``gr00t_n17_real_g1_to_contract_v1`` describes GR00T N1.7's ``real_g1_relative_eef_relative_joints``
action space and states, per key, whether the twin can consume it:

* ``left_arm`` / ``right_arm`` (7 each, RELATIVE joint deltas in the checkpoint's representation, returned by the
  policy as ABSOLUTE targets after its own composition with the current state): consumable BY NAME in the Unitree
  arm order; each target must be finite and inside the manifest joint limits.
* ``waist`` (3, absolute): consumable by name.
* ``left_hand`` / ``right_hand`` (7, absolute, Unitree Dex3-1 three-finger hand): NOT consumable — the twin's hands are
  Inspire six-actuator hands; there is no physical correspondence and no channel may be invented, dropped or
  reinterpreted silently. The adapter refuses the whole chunk unless the caller explicitly declares an
  ``arms_and_waist_only`` exploratory scope, in which case the hand keys are reported as REJECTED and the hands are
  left uncommanded (held at their current state) — never fabricated.
* ``left_wrist_eef_9d`` / ``right_wrist_eef_9d`` (relative EEF, XYZ_ROT6D): no IK/controller path in the twin — REJECTED.
* ``base_height_command`` / ``navigate_command``: whole-body-controller inputs — no consumer on a fixed pelvis — REJECTED.

No clamping is applied: out-of-range targets are refusals. The adapter never sends anything; it produces a
validated report (accepted keys, rejected keys with reasons, converted body targets by name, fraction of values
outside limits) that a replay package could be built from only if the report is executable.
"""
from __future__ import annotations

import math
from typing import Mapping

from .embodiment import BODY_JOINT_ORDER_UNITREE_29, ContractError, EmbodimentManifest

ADAPTER_VERSION = 'gr00t_n17_real_g1_to_contract_v1'
ARM_ORDER = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')
WAIST_ORDER = ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint')
EXPECTED_DIMS = {'left_wrist_eef_9d': 9, 'right_wrist_eef_9d': 9, 'left_hand': 7, 'right_hand': 7, 'left_arm': 7, 'right_arm': 7, 'waist': 3, 'base_height_command': 1, 'navigate_command': 3}


def _rows(value):
    """Accept (H, D) or (B=1, H, D) nested lists/arrays; return list of rows."""
    v = value.tolist() if hasattr(value, 'tolist') else value
    if isinstance(v, list) and v and isinstance(v[0], list) and v[0] and isinstance(v[0][0], list):
        if len(v) != 1:
            raise ContractError('batched chunks with B > 1 are not accepted')
        v = v[0]
    if not isinstance(v, list) or not v or not all(isinstance(r, list) for r in v):
        raise ContractError('action value must be a (H, D) chunk')
    return v


def validate_chunk(manifest: EmbodimentManifest, actions: Mapping[str, object], *, scope: str = 'full', horizon_steps: int | None = None):
    """Validate one GR00T REAL_G1 action chunk against the contract. scope: 'full' (everything must be consumable) or
    'arms_and_waist_only' (explicit exploratory declaration: hands/EEF/base keys are reported REJECTED, not executed)."""
    if scope not in ('full', 'arms_and_waist_only'):
        raise ContractError('scope must be full or arms_and_waist_only')
    report = {'adapter': ADAPTER_VERSION, 'scope': scope, 'accepted': {}, 'rejected': {}, 'executable': False, 'horizon': None, 'values_outside_limits': 0, 'values_total': 0, 'MOCK': False}
    missing = [k for k in EXPECTED_DIMS if k not in actions]
    unexpected = [k for k in actions if k not in EXPECTED_DIMS]
    if missing or unexpected:
        raise ContractError('action keys mismatch: missing %s, unexpected %s' % (missing, unexpected))
    chunks = {k: _rows(v) for k, v in actions.items()}
    horizons = {len(v) for v in chunks.values()}
    if len(horizons) != 1:
        raise ContractError('all keys must share one horizon; got %s' % sorted(horizons))
    H = horizons.pop()
    if horizon_steps is not None and H != horizon_steps:
        raise ContractError('horizon %d differs from the declared %d' % (H, horizon_steps))
    report['horizon'] = H
    for k, dim in EXPECTED_DIMS.items():
        if any(len(r) != dim for r in chunks[k]):
            raise ContractError('%s rows must have %d entries' % (k, dim))
        for r in chunks[k]:
            for x in r:
                if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
                    raise ContractError('%s contains a non-finite or non-numeric value' % k)
    for side in ('left', 'right'):
        report['rejected']['%s_hand' % side] = 'Unitree Dex3-1 seven-channel hand action; the twin has Inspire six-actuator hands (physical embodiment mismatch) — no mapping, nothing invented'
        report['rejected']['%s_wrist_eef_9d' % side] = 'relative EEF (XYZ_ROT6D) needs an IK/controller path the twin does not expose — not decodable here'
    report['rejected']['base_height_command'] = 'whole-body-controller input; no consumer on FIXED_PELVIS'
    report['rejected']['navigate_command'] = 'whole-body-controller input; no consumer on FIXED_PELVIS'
    targets = []
    for h in range(H):
        row = {}
        for side in ('left', 'right'):
            for j, name in enumerate(ARM_ORDER):
                joint = '%s_%s_joint' % (side, name)
                lo, hi = manifest.body_limit(joint)
                v = float(chunks['%s_arm' % side][h][j]); report['values_total'] += 1
                if not lo <= v <= hi:
                    report['values_outside_limits'] += 1
                row[joint] = v
        for j, joint in enumerate(WAIST_ORDER):
            lo, hi = manifest.body_limit(joint)
            v = float(chunks['waist'][h][j]); report['values_total'] += 1
            if not lo <= v <= hi:
                report['values_outside_limits'] += 1
            row[joint] = v
        targets.append(row)
    report['accepted'] = {'left_arm': 'absolute joint targets by name (Unitree arm order)', 'right_arm': 'absolute joint targets by name (Unitree arm order)', 'waist': 'absolute joint targets by name'}
    report['body_targets_rad'] = targets if report['values_outside_limits'] == 0 else None
    report['executable'] = scope == 'arms_and_waist_only' and report['values_outside_limits'] == 0
    if scope == 'full':
        report['executable'] = False
        report['refusal'] = 'full scope not executable: hands, EEF and base commands have no consumer in this twin'
    return report


def mock_chunk(manifest: EmbodimentManifest, horizon: int = 40, *, label='MOCK'):
    """Explicitly MOCK chunk for harness tests (mid-range, finite, in-limits values). Never confuse with model output."""
    assert label == 'MOCK'
    row = {}
    for side in ('left', 'right'):
        row['%s_arm' % side] = [0.5 * (manifest.body_limit('%s_%s_joint' % (side, n))[0] + manifest.body_limit('%s_%s_joint' % (side, n))[1]) for n in ARM_ORDER]
        row['%s_hand' % side] = [0.0] * 7
        row['%s_wrist_eef_9d' % side] = [0.0] * 9
    row['waist'] = [0.5 * sum(manifest.body_limit(j)) for j in WAIST_ORDER]
    row['base_height_command'] = [0.74]; row['navigate_command'] = [0.0, 0.0, 0.0]
    return {k: [list(v) for _ in range(horizon)] for k, v in row.items()}, 'MOCK'


__all__ = ['ADAPTER_VERSION', 'validate_chunk', 'mock_chunk', 'BODY_JOINT_ORDER_UNITREE_29']
