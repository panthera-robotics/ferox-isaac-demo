"""Pure reference of the cached piston model's state/action interface (GR00T N1.6 fine-tune `birbirll/g1-inspire-piston-n16`,
processor `Gr00tN1d6Processor`, code Isaac-GR00T n1d6 @ 9b37aa1c), reproduced from the processor configuration, the
checkpoint statistics and the processor source — NOT from the appearance of any frame.

Per-field evidence classes (see H2_MODEL_CONVENTIONS.md in the private evidence for the full table):
  VERIFIED_SOURCE  state keys/dims/order, action keys/dims/order, RELATIVE arms / ABSOLUTE rest, single-frame state
                   (delta_indices [0]), 30-step chunk, 50 Hz (dataset fps), min-max normalization to [-1, 1] with
                   clipping, NO sin/cos on this embodiment (sin_cos_embedding_keys null), relative = action - state[-1]
                   elementwise, unapply = clip -> unnormalize -> + reference state, constant columns pinned (min == max),
                   dataset joint names and their URDF counterparts, radians.
  INFERRED         the dataset arm datum equals the URDF zero configuration (FK of episode 0 gives a coherent reach /
                   close / lift under it; the collector `replay_piston_csv.py` is not cached), hand datum identity with
                   the FTP donor (rod30 runtime pair + dataset statistics).
  UNRESOLVED       exact camera intrinsics/mount of the recording scene, the physical rightness of any hand map for the
                   installed RH56E2, and what the first all-zero row means (idle pose vs placeholder).
This module performs no normalization on the caller's behalf that the server already performs; it exists to state the
transforms, to test them, and to refuse double application (types are explicit tags).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

STATE_KEYS = ('left_arm', 'right_arm', 'left_hand', 'right_hand', 'waist')
ACTION_KEYS = ('left_arm', 'right_arm', 'left_hand', 'right_hand', 'base_height', 'navigate_command')
DIMS = {'left_arm': 7, 'right_arm': 7, 'left_hand': 6, 'right_hand': 6, 'waist': 3, 'base_height': 1, 'navigate_command': 3}
ACTION_REP = {'left_arm': 'RELATIVE', 'right_arm': 'RELATIVE', 'left_hand': 'ABSOLUTE', 'right_hand': 'ABSOLUTE', 'base_height': 'ABSOLUTE', 'navigate_command': 'ABSOLUTE'}
CHUNK = 30
STEP_S = 0.02
ARM_SUFFIX = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')          # dataset feature order == Unitree SDK arm order
HAND_DATASET = ('pinky', 'ring', 'middle', 'index', 'thumb_pitch', 'thumb_yaw')
HAND_DONOR_IDENTITY = {'pinky': 'little_1_joint', 'ring': 'ring_1_joint', 'middle': 'middle_1_joint', 'index': 'index_1_joint', 'thumb_pitch': 'thumb_2_joint', 'thumb_yaw': 'thumb_1_joint'}   # EXPLORATORY identity contract
WAIST = ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint')
STATE_TACTILE_UNUSED = 'observation.state[29:63] (34 tactile channels) are not in any modality key: unused by the model'


def arm_joint_names(side):
    return ['%s_%s_joint' % (side, s) for s in ARM_SUFFIX]


def hand_joint_names(side):
    return ['%s_%s' % (side, HAND_DONOR_IDENTITY[h]) for h in HAND_DATASET]


class Stats:
    """Checkpoint statistics (statistics.json['new_embodiment']) with the processor's exact min-max formulas."""

    def __init__(self, statistics_json):
        d = json.loads(Path(statistics_json).read_text()) if not isinstance(statistics_json, dict) else statistics_json
        self.s = d['new_embodiment'] if 'new_embodiment' in d else d
        for grp in ('state', 'action', 'relative_action'):
            assert grp in self.s, grp
        for k in STATE_KEYS:
            assert len(self.s['state'][k]['min']) == DIMS[k], k
        for k in ACTION_KEYS:
            assert len(self.s['action'][k]['min']) == DIMS[k], k
        for k in ('left_arm', 'right_arm'):
            assert np.asarray(self.s['relative_action'][k]['min']).shape == (CHUNK, DIMS[k]), k   # per-step bounds (T, D)

    @staticmethod
    def normalize(values, params):
        mn, mx = np.asarray(params['min'], float), np.asarray(params['max'], float)
        v = np.asarray(values, float); out = np.zeros_like(v)
        mask = ~np.isclose(mx, mn)
        out[..., mask] = 2 * (v[..., mask] - mn[..., mask]) / (mx[..., mask] - mn[..., mask]) - 1
        return out                                                    # constant columns -> 0 (utils.normalize_values_minmax)

    @staticmethod
    def unnormalize(normalized, params):
        mn, mx = np.asarray(params['min'], float), np.asarray(params['max'], float)
        return (np.clip(np.asarray(normalized, float), -1.0, 1.0) + 1.0) / 2.0 * (mx - mn) + mn   # utils.unnormalize_values_minmax

    def state_params(self, key):
        return self.s['state'][key]

    def action_params(self, key):
        return self.s['relative_action'][key] if ACTION_REP[key] == 'RELATIVE' else self.s['action'][key]

    def pinned_action_columns(self):
        """Columns whose output is pinned to a constant by min == max (the model can never move them)."""
        out = {}
        for k in ACTION_KEYS:
            p = self.action_params(k); mn, mx = np.asarray(p['min'], float), np.asarray(p['max'], float)
            if mn.ndim == 2:
                mn, mx = mn[0], mx[0]
            pinned = np.isclose(mn, mx)
            if pinned.any():
                out[k] = {'columns': np.flatnonzero(pinned).tolist(), 'values': mn[pinned].tolist()}
        return out


# ---- typed values (double application is a type error) --------------------------------------------------------------
def urdf_pose(arm_by_name: dict, hand_by_name: dict, waist_by_name: dict):
    """A physical configuration in URDF radians, by joint name (source-of-truth type for encode)."""
    return {'type': 'urdf_abs_rad', 'arm': dict(arm_by_name), 'hand': dict(hand_by_name), 'waist': dict(waist_by_name)}


def encode_state(pose):
    """URDF pose -> the raw state dict the policy server consumes ({key: (1, D) radians}); the processor normalizes and
    (for this embodiment) applies NO sin/cos. Under the INFERRED datum the URDF radians are the dataset radians."""
    if pose.get('type') != 'urdf_abs_rad':
        raise TypeError('encode_state needs a urdf_abs_rad pose, got %r' % pose.get('type'))
    out = {}
    for side in ('left', 'right'):
        out[side + '_arm'] = np.array([[pose['arm'][n] for n in arm_joint_names(side)]], float)
        out[side + '_hand'] = np.array([[pose['hand'][n] for n in hand_joint_names(side)]], float)
    out['waist'] = np.array([[pose['waist'][n] for n in WAIST]], float)
    for k, v in out.items():
        if not np.isfinite(v).all():
            raise ValueError('non-finite state in %s' % k)
    return {'type': 'dataset_state_rad', 'values': out, 'datum': 'URDF zero (INFERRED)', 'sincos': False, 'normalization': 'server-side min-max (statistics.json); not applied here'}


def server_unapply(normalized_action, reference_state, stats: Stats):
    """What the policy server does with the network output: clip -> unnormalize -> (+ reference state for RELATIVE keys).
    Returns an ABSOLUTE target chunk {key: (T, D)} tagged as such."""
    if reference_state.get('type') != 'dataset_state_rad':
        raise TypeError('reference state must be dataset_state_rad')
    out = {}
    for k in ACTION_KEYS:
        n = np.asarray(normalized_action[k], float)
        if n.shape != (CHUNK, DIMS[k]):
            raise ValueError('%s: expected (%d, %d), got %s' % (k, CHUNK, DIMS[k], n.shape))
        raw = stats.unnormalize(n, stats.action_params(k))
        if ACTION_REP[k] == 'RELATIVE':
            raw = raw + reference_state['values'][k][-1]           # JointActionChunk.to_absolute_chunking: elementwise add
        out[k] = raw
    return {'type': 'absolute_target_rad', 'values': out, 'relative_keys': [k for k in ACTION_KEYS if ACTION_REP[k] == 'RELATIVE']}


def targets_by_name(absolute_chunk, step):
    """Absolute target chunk -> URDF joint targets by name for one step (arms + hands); refuses any other type."""
    if absolute_chunk.get('type') != 'absolute_target_rad':
        raise TypeError('targets need an absolute_target_rad chunk (never add the state again): got %r' % absolute_chunk.get('type'))
    v = absolute_chunk['values']
    out = {}
    for side in ('left', 'right'):
        out.update({n: float(v[side + '_arm'][step, i]) for i, n in enumerate(arm_joint_names(side))})
        out.update({n: float(v[side + '_hand'][step, i]) for i, n in enumerate(hand_joint_names(side))})
    return out


def dataset_action_to_relative(action_abs, state, stats: Stats, key):
    """Training-side transform for a RELATIVE key: relative = action_abs - state[-1] (elementwise), then min-max normalized
    with the per-step relative_action statistics (reference for round-trip tests)."""
    rel = np.asarray(action_abs, float) - np.asarray(state, float)[-1]
    return stats.normalize(rel, stats.action_params(key))
