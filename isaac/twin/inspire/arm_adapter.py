"""Generic SIM-ONLY consumer of version 1 named upper-body references.

This module has no task planner, ROS, DDS, SDK, socket or hardware transport.
It evaluates explicit PD/feedforward exactly once. Its caller is the sole body
effort writer and must disable implicit USD drives. A fault requires stopping
physics and resetting the episode: continuing with the previous effort is invalid.

The hybrid blend is an identified simulation surrogate. It does not reproduce
Unitree firmware. Hands have separate owners and never enter this 29-body map.
"""

from dataclasses import dataclass
import math
import re
from typing import Mapping


UPPER_NAMES = ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint') + tuple(
    '%s_%s_joint' % (side, joint)
    for side in ('left', 'right')
    for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
                  'wrist_roll', 'wrist_pitch', 'wrist_yaw'))
CHANNELS = frozenset(('q', 'dq', 'kp', 'kd', 'tau'))
REFERENCE_FIELDS = frozenset((
    'schema_version', 'kind', 'target', 'simulator_id', 'run_id', 'sequence',
    'sim_time_s', 'source_monotonic_time_s', 'valid_for_s', 'blend_weight', 'joints',
    'semantics', 'hand_authority', 'leg_authority'))


class SimulationAdmissionError(ValueError):
    """Latched refusal: the caller must stop physics and reset the episode."""


def _number(value, field, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('%s must be finite numeric data' % field)
    if (low is not None and value < low) or (high is not None and value > high):
        raise ValueError('%s exceeds its admitted bound' % field)
    return float(value)


def _identifier(value):
    if not isinstance(value, str) or not re.fullmatch('[a-z][a-z0-9_]{7,63}', value):
        raise ValueError('simulator and run IDs must match [a-z][a-z0-9_]{7,63}')
    return value


@dataclass(frozen=True)
class JointBound:
    q_min: float
    q_max: float
    max_velocity: float
    max_kp: float
    max_kd: float
    max_effort: float

    def __post_init__(self):
        _number(self.q_min, 'q_min')
        _number(self.q_max, 'q_max')
        if self.q_min >= self.q_max:
            raise ValueError('q_min must precede q_max')
        for name in ('max_velocity', 'max_kp', 'max_kd', 'max_effort'):
            if _number(getattr(self, name), name, 0.0) == 0.0:
                raise ValueError('%s must be positive' % name)


@dataclass(frozen=True)
class BodyEffort:
    """One physics sample's body effort; indices explicitly come from the asset map."""
    joint_names: tuple
    articulation_indices: tuple
    effort_nm: tuple
    physics_sequence: int
    sim_time_s: float
    final_writer: str
    reference_owners: tuple
    mode: str
    actuation_semantics: str = 'explicit_pd_feedforward_once; implicit_drives_disabled'


class NamedBodyArbiter:
    """One fixed owner/mode per admitted run; switching requires a fresh run ID.

    ``body_indices`` must map exactly the 29 body joint names to distinct actual
    articulation indices. Interleaved hand joints are valid and never sliced.
    Bounds must come from the qualified embodiment, not array-position guesses.
    ``observe_physics`` is called for each new sample; ``compose`` at most once
    for that sample. ``check_freshness`` must also run during physics pauses.
    """

    def __init__(self, *, body_indices: Mapping[str, int], bounds: Mapping[str, JointBound],
                 simulator_id: str, run_id: str, mode: str, controller_id: str,
                 simulation_authorized: bool, implicit_drives_disabled: bool,
                 target='isaacsim', maximum_age_s=0.10):
        if simulation_authorized is not True or target != 'isaacsim':
            raise ValueError('independent simulation authorization required')
        if implicit_drives_disabled is not True:
            raise ValueError('implicit drives must be disabled before explicit PD actuation')
        self.simulator_id, self.run_id = _identifier(simulator_id), _identifier(run_id)
        if mode not in ('hybrid', 'fullbody'):
            raise ValueError('exactly one of hybrid/fullbody must own the body')
        if not isinstance(controller_id, str) or not controller_id.strip():
            raise ValueError('an identified balancing or whole-body controller is required')
        if len(body_indices) != 29 or set(bounds) != set(body_indices):
            raise ValueError('body name map and bounds must contain the same 29 names')
        if not set(UPPER_NAMES).issubset(body_indices):
            raise ValueError('body map is missing named waist/arm joints')
        if any(type(i) is not int or i < 0 for i in body_indices.values()):
            raise ValueError('articulation indices must be nonnegative integers')
        if len(set(body_indices.values())) != 29:
            raise ValueError('articulation indices must be unique')
        if any(not isinstance(v, JointBound) for v in bounds.values()):
            raise ValueError('each body joint needs a JointBound')
        self._indices = dict(body_indices)
        self._bounds = dict(bounds)
        self._names = tuple(sorted(body_indices, key=body_indices.__getitem__))
        self._mode, self._controller_id = mode, controller_id
        self.maximum_age_s = _number(maximum_age_s, 'maximum_age_s', 0.001, 0.10)
        self.fault_reason = None
        self._physics = None
        self._upper = None
        self._last_applied = -1
        self._last_now = None

    @property
    def mode(self):
        return self._mode

    @property
    def fault_action(self):
        return 'stop_physics_and_reset_episode'

    def _refuse(self, reason):
        if self.fault_reason is None:
            self.fault_reason = str(reason)
        self._upper = None
        raise SimulationAdmissionError(self.fault_reason)

    def _ready(self):
        if self.fault_reason is not None:
            raise SimulationAdmissionError(self.fault_reason)

    def _now(self, now):
        now = _number(now, 'now_monotonic_s', 0.0)
        if self._last_now is not None and now < self._last_now:
            raise ValueError('receiver monotonic clock moved backwards')
        self._last_now = now
        return now

    def _joint_values(self, values, names):
        if not isinstance(values, Mapping) or set(values) != set(names):
            raise ValueError('joint reference name set mismatch')
        result = {}
        for name in names:
            item = values[name]
            if not isinstance(item, Mapping) or set(item) != CHANNELS:
                raise ValueError('joint reference channel set mismatch')
            bound = self._bounds[name]
            result[name] = dict(
                q=_number(item['q'], name + '.q', bound.q_min, bound.q_max),
                dq=_number(item['dq'], name + '.dq', -bound.max_velocity, bound.max_velocity),
                kp=_number(item['kp'], name + '.kp', 0.0, bound.max_kp),
                kd=_number(item['kd'], name + '.kd', 0.0, bound.max_kd),
                tau=_number(item['tau'], name + '.tau', -bound.max_effort, bound.max_effort))
        return result

    def observe_physics(self, *, sequence, sim_time_s, source_monotonic_s,
                        position, velocity, now_monotonic_s):
        self._ready()
        try:
            now = self._now(now_monotonic_s)
            if type(sequence) is not int or sequence < 0:
                raise ValueError('physics sequence must be a nonnegative integer')
            sim_t = _number(sim_time_s, 'physics sim_time_s', 0.0)
            source_t = _number(source_monotonic_s, 'physics source_monotonic_s', 0.0)
            _number(now - source_t, 'physics sample age', 0.0, self.maximum_age_s)
            if self._physics is not None:
                old = self._physics
                if sequence <= old['sequence'] or sim_t <= old['sim_t']:
                    raise ValueError('physics replay, pause or reset requires a fresh run')
                if source_t <= old['source_t']:
                    raise ValueError('physics source clock failed to advance')
            if set(position) != set(self._names) or set(velocity) != set(self._names):
                raise ValueError('physics joint name set mismatch')
            q = {name: _number(position[name], name + '.measured_q') for name in self._names}
            dq = {name: _number(velocity[name], name + '.measured_dq') for name in self._names}
            self._physics = dict(sequence=sequence, sim_t=sim_t, source_t=source_t, q=q, dq=dq)
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self._refuse(exc)

    def check_freshness(self, now_monotonic_s):
        self._ready()
        try:
            now = self._now(now_monotonic_s)
            if self._physics is None:
                raise ValueError('no admitted physics sample')
            _number(now - self._physics['source_t'], 'physics sample age', 0.0,
                    self.maximum_age_s)
            if self._upper is not None:
                _number(now - self._upper['source_monotonic_time_s'], 'task age', 0.0,
                        self._upper['valid_for_s'])
                _number(self._physics['sim_t'] - self._upper['sim_time_s'],
                        'task physics age', 0.0, self._upper['valid_for_s'])
            return True
        except (ValueError, TypeError) as exc:
            self._refuse(exc)

    def accept_upper_reference(self, message, *, now_monotonic_s):
        self._ready()
        try:
            if self.mode != 'hybrid':
                raise ValueError('upper-body writer conflicts with fullbody ownership')
            self.check_freshness(now_monotonic_s)
            if not isinstance(message, Mapping) or set(message) != REFERENCE_FIELDS:
                raise ValueError('upper-body schema fields mismatch')
            expected = dict(schema_version=1, kind='upper_body_reference', target='isaacsim',
                            simulator_id=self.simulator_id, run_id=self.run_id,
                            semantics='arm_sdk_surrogate_reference_not_final_actuation',
                            hand_authority='none', leg_authority='none')
            if type(message['schema_version']) is not int or any(
                    message[k] != v for k, v in expected.items()):
                raise ValueError('upper-body identity or semantics mismatch')
            seq = message['sequence']
            if type(seq) is not int or seq < 0:
                raise ValueError('task sequence must be a nonnegative integer')
            sim_t = _number(message['sim_time_s'], 'task sim_time_s', 0.0)
            source_t = _number(message['source_monotonic_time_s'], 'task monotonic time', 0.0)
            ttl = _number(message['valid_for_s'], 'valid_for_s', 0.001, self.maximum_age_s)
            _number(now_monotonic_s - source_t, 'task age', 0.0, ttl)
            _number(self._physics['sim_t'] - sim_t, 'task physics age', 0.0, ttl)
            if self._upper is not None:
                if seq <= self._upper['sequence'] or sim_t <= self._upper['sim_time_s']:
                    raise ValueError('task replay or frozen simulation clock')
                if source_t <= self._upper['source_monotonic_time_s']:
                    raise ValueError('task source clock failed to advance')
            weight = _number(message['blend_weight'], 'blend_weight', 0.0, 1.0)
            joints = self._joint_values(message['joints'], UPPER_NAMES)
            if any(v['kp'] <= 0.0 or v['kd'] <= 0.0 for v in joints.values()):
                raise ValueError('all blended joints require positive gains')
            self._upper = dict(sequence=seq, sim_time_s=sim_t,
                               source_monotonic_time_s=source_t, valid_for_s=ttl,
                               blend_weight=weight, joints=joints)
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self._refuse(exc)

    def compose(self, body_reference, *, controller_id, physics_sequence, now_monotonic_s):
        """Return exactly one explicit PD effort vector for the admitted physics sample.

        A body reference must have been evaluated for this exact physics sequence.
        The caller must tag the reference with the physics sample it evaluated.
        Hybrid blends *efforts* from the two PD references, avoiding products of
        blended gains and blended positions. Fullbody admits only its named WBC.
        """
        self._ready()
        try:
            self.check_freshness(now_monotonic_s)
            physics = self._physics
            if controller_id != self._controller_id:
                raise ValueError('conflicting or unidentified final body controller')
            if type(physics_sequence) is not int or physics_sequence != physics['sequence']:
                raise ValueError('body reference is not for the current physics sample')
            if physics_sequence <= self._last_applied:
                raise ValueError('second body write for the same physics sample')
            body = self._joint_values(body_reference, self._names)
            efforts, owners = [], []
            for name in self._names:
                def pd(reference):
                    return (reference['tau'] + reference['kp'] *
                            (reference['q'] - physics['q'][name]) + reference['kd'] *
                            (reference['dq'] - physics['dq'][name]))
                effort = pd(body[name])
                owner = self._controller_id
                if self._upper is not None and name in UPPER_NAMES:
                    alpha = self._upper['blend_weight']
                    effort = (1.0 - alpha) * effort + alpha * pd(self._upper['joints'][name])
                    owner = 'hybrid(%s,upper_body_reference)' % self._controller_id
                bound = self._bounds[name]
                efforts.append(_number(effort, name + '.final_effort',
                                       -bound.max_effort, bound.max_effort))
                owners.append(owner)
            self._last_applied = physics_sequence
            return BodyEffort(self._names, tuple(self._indices[n] for n in self._names),
                              tuple(efforts), physics_sequence, physics['sim_t'],
                              'simulation_body_arbiter', tuple(owners), self.mode)
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self._refuse(exc)

    def reset_for_run(self, *, run_id, mode, controller_id, simulation_authorized):
        """Admit a new episode after caller stops/resets physics; never clear in place."""
        if run_id == self.run_id:
            raise ValueError('reset/handover needs a different run ID')
        return NamedBodyArbiter(
            body_indices=self._indices, bounds=self._bounds,
            simulator_id=self.simulator_id, run_id=run_id, mode=mode,
            controller_id=controller_id, simulation_authorized=simulation_authorized,
            implicit_drives_disabled=True, maximum_age_s=self.maximum_age_s)
