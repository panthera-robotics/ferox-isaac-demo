"""Versioned embodiment and command contract for the G1 + Inspire twin (simulation and data replay).

One manifest separates, per version: hardware identity from source-asset identity and qualification
status; the 29 named body coordinates, the six independent hand actuators per side and the coupled
(mimic) joints they drive; command versus feedback conventions (units, signs, endpoints,
normalization, saturation policy); wrist/hand/tool transforms with provenance and validity hashes;
the controller description (type, gains provenance, rate, latency, clock domains); camera and
tactile/force descriptors with their availability. Nothing here is a hardware calibration: unknown
E2 parameters stay ``None`` and are reported as unknown.

A logical 29 + 6 + 6 interface is not a packet width: sources declare their own axis order and scale,
conversion is by name, coupled joints are never commanded, and invalid commands fail before any
transmission. Changing the asset, mount, grasp or holder invalidates the dependent transforms and the
qualifications that rest on them (``check_validity``).
"""
from __future__ import annotations

import hashlib
import json
import math
from typing import Mapping

MANIFEST_SCHEMA_VERSION = 1

# Unitree G1 29-DoF SDK joint index order (G1JointIndex); the twin's body arbiter maps these by name.
BODY_JOINT_ORDER_UNITREE_29 = (
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint')
HAND_ACTUATORS = ('index', 'middle', 'ring', 'little', 'thumb_bend', 'thumb_rotation')
SIDES = ('left', 'right')
SATURATION_POLICIES = ('reject', 'clip_declared')


class ContractError(ValueError):
    """Refusal before transmission: the command, manifest or source contract is invalid."""


def _finite(value, name, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ContractError('%s must be finite numeric data' % name)
    if (low is not None and value < low) or (high is not None and value > high):
        raise ContractError('%s outside its admitted bound [%s, %s]' % (name, low, high))
    return float(value)


def canonical_sha256(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


class EmbodimentManifest:
    """Validated, hashed view of one manifest version. ``data`` is never mutated."""

    REQUIRED = ('schema_version', 'manifest_id', 'hardware_identity', 'source_asset', 'qualification', 'body', 'hands',
                'transforms', 'controller', 'cameras', 'tactile_force')

    def __init__(self, data: Mapping):
        if not isinstance(data, Mapping):
            raise ContractError('manifest must be a mapping')
        missing = [k for k in self.REQUIRED if k not in data]
        if missing:
            raise ContractError('manifest missing required fields: %s' % ', '.join(missing))
        if data['schema_version'] != MANIFEST_SCHEMA_VERSION:
            raise ContractError('unsupported manifest schema version')
        if not isinstance(data['manifest_id'], str) or not data['manifest_id']:
            raise ContractError('manifest_id required')
        ident = data['hardware_identity']
        for k in ('robot', 'right_hand', 'left_hand', 'identity_evidence'):
            if k not in ident:
                raise ContractError('hardware_identity.%s required' % k)
        asset = data['source_asset']
        for k in ('asset_id', 'kind', 'exact_hand_model', 'urdf_sha256'):
            if k not in asset:
                raise ContractError('source_asset.%s required' % k)
        if asset['exact_hand_model'] is not True and asset['exact_hand_model'] is not False:
            raise ContractError('source_asset.exact_hand_model must be an explicit boolean')
        body = data['body']
        names = body.get('joint_names')
        if not isinstance(names, list) or len(names) != 29 or len(set(names)) != 29:
            raise ContractError('body.joint_names must list the 29 distinct body coordinates')
        if tuple(names) != BODY_JOINT_ORDER_UNITREE_29:
            raise ContractError('body.joint_names must follow the declared Unitree 29-DoF order by name')
        lim = body.get('limits_rad')
        if not isinstance(lim, Mapping) or set(lim) != set(names):
            raise ContractError('body.limits_rad must cover exactly the 29 body joints')
        for n, v in lim.items():
            lo, hi = _finite(v[0], n + '.lower'), _finite(v[1], n + '.upper')
            if lo >= hi:
                raise ContractError('%s lower limit must precede upper' % n)
        if body.get('floating_base', {}).get('state_fields') is None or body.get('floating_base', {}).get('support') is None:
            raise ContractError('body.floating_base must declare state_fields and support')
        hands = data['hands']
        if set(hands) != set(SIDES):
            raise ContractError('hands must describe exactly left and right')
        self._hand_joint_names = {}
        for side, hand in hands.items():
            act = hand.get('actuators')
            if not isinstance(act, Mapping) or tuple(act) != HAND_ACTUATORS:
                raise ContractError('%s hand must declare the six actuators in the contract order' % side)
            seen = set()
            for a, spec in act.items():
                for k in ('joint', 'open_rad', 'closed_rad', 'limit_rad'):
                    if k not in spec:
                        raise ContractError('%s.%s.%s required' % (side, a, k))
                if not isinstance(spec['joint'], str) or not spec['joint'].startswith(side + '_'):
                    raise ContractError('%s.%s joint must belong to that side' % (side, a))
                if spec['joint'] in seen:
                    raise ContractError('%s hand maps two actuators to one joint' % side)
                seen.add(spec['joint'])
                lo, hi = _finite(spec['limit_rad'][0], a + '.limit.lower'), _finite(spec['limit_rad'][1], a + '.limit.upper')
                o, c = _finite(spec['open_rad'], a + '.open_rad', lo, hi), _finite(spec['closed_rad'], a + '.closed_rad', lo, hi)
                if o == c:
                    raise ContractError('%s.%s open and closed endpoints coincide' % (side, a))
            coupled = hand.get('coupled_joints')
            if not isinstance(coupled, Mapping) or not coupled:
                raise ContractError('%s hand must declare its coupled joints' % side)
            for child, m in coupled.items():
                if child in seen or not child.startswith(side + '_'):
                    raise ContractError('coupled joint %s is not a distinct joint of the %s hand' % (child, side))
                if m.get('parent') not in seen and m.get('parent') not in coupled:
                    raise ContractError('coupled joint %s has no declared parent' % child)
                _finite(m.get('multiplier'), child + '.multiplier')
                _finite(m.get('offset', 0.0), child + '.offset')
            if hand.get('feedback', {}).get('independent_axes_measured') is None:
                raise ContractError('%s hand must declare whether independent axes are measured' % side)
            self._hand_joint_names[side] = tuple(spec['joint'] for spec in act.values())
        for key in ('wrist_to_hand', 'hand_to_tool'):
            for side in SIDES:
                t = data['transforms'].get(side, {}).get(key)
                if t is None:
                    continue
                for k in ('matrix_4x4', 'frame', 'provenance', 'valid_for'):
                    if k not in t:
                        raise ContractError('transforms.%s.%s.%s required' % (side, key, k))
                if len(t['matrix_4x4']) != 4 or any(len(r) != 4 for r in t['matrix_4x4']):
                    raise ContractError('transform matrix must be 4x4')
                if not isinstance(t['valid_for'], Mapping) or not t['valid_for']:
                    raise ContractError('transform validity hashes required')
        ctl = data['controller']
        for k in ('type', 'gains_provenance', 'rate_hz', 'latency_assumption', 'clock_domains'):
            if k not in ctl:
                raise ContractError('controller.%s required' % k)
        _finite(ctl['rate_hz'], 'controller.rate_hz', 1.0, 10000.0)
        for cam_id, cam in data['cameras'].items():
            for k in ('mount_frame', 'image_format', 'calibration', 'timestamp_domain'):
                if k not in cam:
                    raise ContractError('cameras.%s.%s required' % (cam_id, k))
        for f, spec in data['tactile_force'].items():
            if spec.get('availability') not in ('measured', 'estimated', 'simulator_only', 'unavailable'):
                raise ContractError('tactile_force.%s.availability must be declared' % f)
            if 'units' not in spec:
                raise ContractError('tactile_force.%s.units required' % f)
        self.data = json.loads(json.dumps(data))
        self.sha256 = canonical_sha256(self.data)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls(json.load(f))

    @property
    def body_names(self):
        return tuple(self.data['body']['joint_names'])

    def body_limit(self, name):
        lo, hi = self.data['body']['limits_rad'][name]
        return float(lo), float(hi)

    def hand_joint_names(self, side):
        return self._hand_joint_names[side]

    def hand_actuator(self, side, actuator):
        return self.data['hands'][side]['actuators'][actuator]

    def check_validity(self, current: Mapping[str, str]):
        """Compare the transform validity hashes with the live asset/grasp/mount hashes.

        Returns the manifest's qualification block with every transform-dependent status demoted to
        ``INVALIDATED_BY_CHANGE`` when any hash disagrees; a missing live hash counts as a disagreement.
        Never mutates the manifest.
        """
        report = {'transforms': {}, 'qualification': json.loads(json.dumps(self.data['qualification'])), 'valid': True}
        for side, block in self.data['transforms'].items():
            for key, t in block.items():
                if t is None:
                    continue
                bad = {k: (current.get(k), v) for k, v in t['valid_for'].items() if current.get(k) != v}
                status = 'VALID' if not bad else 'INVALID'
                report['transforms']['%s.%s' % (side, key)] = {'status': status, 'mismatched': bad}
                if bad:
                    report['valid'] = False
                    for dep in t.get('dependent_qualifications', []):
                        report['qualification'][dep] = 'INVALIDATED_BY_CHANGE'
        return report


class HandCommandAdapter:
    """Convert one side's six actuator values between a declared source contract and joint targets.

    A source contract declares ``axis_order`` (names from HAND_ACTUATORS, any permutation), the
    numeric ``scale`` (``open_value``, ``closed_value``), and its ``saturation_policy``. Values are
    normalized to closure c in [0, 1] (0 = open endpoint, 1 = closed endpoint) and mapped by name to the
    manifest's joint endpoints. Coupled joints are never emitted. Left and right never mix.
    """

    def __init__(self, manifest: EmbodimentManifest, side: str, source_contract: Mapping):
        if side not in SIDES:
            raise ContractError('side must be left or right')
        order = source_contract.get('axis_order')
        if not isinstance(order, (list, tuple)) or sorted(order) != sorted(HAND_ACTUATORS):
            raise ContractError('source axis_order must be a permutation of the six named actuators')
        self.open_value = _finite(source_contract.get('open_value'), 'open_value')
        self.closed_value = _finite(source_contract.get('closed_value'), 'closed_value')
        if self.open_value == self.closed_value:
            raise ContractError('source open and closed values coincide')
        policy = source_contract.get('saturation_policy', 'reject')
        if policy not in SATURATION_POLICIES:
            raise ContractError('unknown saturation policy')
        self.policy = policy
        self.tolerance = _finite(source_contract.get('endpoint_tolerance', 0.0), 'endpoint_tolerance', 0.0)
        self.manifest, self.side, self.order = manifest, side, tuple(order)
        self.contract_sha256 = canonical_sha256({'axis_order': self.order, 'open_value': self.open_value, 'closed_value': self.closed_value,
                                                 'saturation_policy': policy, 'endpoint_tolerance': self.tolerance, 'manifest': manifest.sha256, 'side': side})

    def closure(self, value, name):
        """Normalized closure from a source value; rejects or clips per the declared policy."""
        v = _finite(value, name)
        lo, hi = sorted((self.open_value, self.closed_value))
        clipped = False
        if v < lo - self.tolerance or v > hi + self.tolerance:
            if self.policy == 'reject':
                raise ContractError('%s = %r outside the source range [%s, %s]' % (name, value, lo, hi))
            clipped = True
        v = min(max(v, lo), hi)
        return (v - self.open_value) / (self.closed_value - self.open_value), clipped

    def to_joint_targets(self, values):
        """Source vector (declared order) -> {joint_name: target_rad}; also returns clipping/closure info."""
        if not isinstance(values, (list, tuple)) or len(values) != 6:
            raise ContractError('hand command needs exactly six values in the declared order')
        targets, closures, clipped_axes = {}, {}, []
        for actuator, value in zip(self.order, values):
            c, clipped = self.closure(value, '%s.%s' % (self.side, actuator))
            spec = self.manifest.hand_actuator(self.side, actuator)
            q = spec['open_rad'] + c * (spec['closed_rad'] - spec['open_rad'])
            lo, hi = spec['limit_rad']
            targets[spec['joint']] = _finite(q, spec['joint'] + '.target', lo, hi)
            closures[actuator] = c
            if clipped:
                clipped_axes.append(actuator)
        return targets, {'closure': closures, 'clipped_axes': clipped_axes}

    def from_joint_state(self, joint_values: Mapping[str, float]):
        """Joint positions by name -> source vector (declared order); inverse of to_joint_targets."""
        out = []
        for actuator in self.order:
            spec = self.manifest.hand_actuator(self.side, actuator)
            if spec['joint'] not in joint_values:
                raise ContractError('missing joint %s for %s' % (spec['joint'], actuator))
            q = _finite(joint_values[spec['joint']], spec['joint'])
            c = (q - spec['open_rad']) / (spec['closed_rad'] - spec['open_rad'])
            out.append(self.open_value + c * (self.closed_value - self.open_value))
        return out


class ReplaySequence:
    """Validated command rows for one replay: monotonic time, complete fields, finite values, bounds.

    Each row: ``{"t_s": float, "body_q_rad": {name: rad} | null, "hands": {"right": [6 values] | null, "left": ...}}``.
    Body targets are by name (any subset of the 29; unnamed joints keep the initial hold). Rows are
    converted once, before transmission; any refusal aborts the whole sequence with the row index.
    """

    def __init__(self, manifest: EmbodimentManifest, rows, *, hand_contracts: Mapping[str, Mapping], source: Mapping,
                 maximum_step_s=1.0):
        if not isinstance(rows, list) or not rows:
            raise ContractError('replay needs at least one command row')
        for k in ('source_id', 'kind', 'provenance'):
            if k not in source:
                raise ContractError('source.%s required' % k)
        if source['kind'] not in ('real_recording', 'simulator_recording', 'synthetic_test_sequence'):
            raise ContractError('source.kind must be real_recording, simulator_recording or synthetic_test_sequence')
        self.manifest, self.source = manifest, dict(source)
        self.adapters = {s: HandCommandAdapter(manifest, s, c) for s, c in hand_contracts.items()}
        self.rows, self.converted, self.clipped_rows = list(rows), [], []
        last_t = None
        for i, row in enumerate(self.rows):
            if not isinstance(row, Mapping) or 't_s' not in row or 'body_q_rad' not in row or 'hands' not in row:
                raise ContractError('row %d missing required fields (t_s, body_q_rad, hands)' % i)
            t = _finite(row['t_s'], 'row %d t_s' % i, 0.0)
            if last_t is not None and not (0.0 < t - last_t <= maximum_step_s):
                raise ContractError('row %d time does not advance within (0, %s] s (stale, duplicate or gapped sample)' % (i, maximum_step_s))
            last_t = t
            body = {}
            if row['body_q_rad'] is not None:
                if not isinstance(row['body_q_rad'], Mapping) or not row['body_q_rad']:
                    raise ContractError('row %d body_q_rad must be a non-empty name map or null' % i)
                for n, v in row['body_q_rad'].items():
                    if n not in manifest.body_names:
                        raise ContractError('row %d names unknown body joint %s' % (i, n))
                    lo, hi = manifest.body_limit(n)
                    body[n] = _finite(v, 'row %d %s' % (i, n), lo, hi)
            hands = {}
            if not isinstance(row['hands'], Mapping) or set(row['hands']) - set(SIDES):
                raise ContractError('row %d hands must map sides to six values or null' % i)
            for side, values in row['hands'].items():
                if values is None:
                    continue
                if side not in self.adapters:
                    raise ContractError('row %d commands the %s hand without a declared source contract' % (i, side))
                targets, info = self.adapters[side].to_joint_targets(values)
                hands[side] = {'targets_rad': targets, **info}
                if info['clipped_axes']:
                    self.clipped_rows.append(i)
            self.converted.append({'row': i, 't_s': t, 'body_targets_rad': body, 'hands': hands})
        self.duration_s = last_t - self.converted[0]['t_s']
        self.contract_sha256 = canonical_sha256({'manifest': manifest.sha256, 'hands': {s: a.contract_sha256 for s, a in self.adapters.items()},
                                                 'source': self.source, 'rows': self.rows})

    def active_row(self, t_s):
        """Zero-order hold: the last converted row whose time is <= t_s (None before the first)."""
        active = None
        for c in self.converted:
            if c['t_s'] <= t_s:
                active = c
            else:
                break
        return active

    def summary(self):
        return {'rows': len(self.rows), 'duration_s': self.duration_s, 'clipped_rows': list(self.clipped_rows),
                'body_joints_commanded': sorted({n for c in self.converted for n in c['body_targets_rad']}),
                'hands_commanded': sorted({s for c in self.converted for s in c['hands']}), 'contract_sha256': self.contract_sha256,
                'source': self.source, 'manifest_id': self.manifest.data['manifest_id'], 'manifest_sha256': self.manifest.sha256}
