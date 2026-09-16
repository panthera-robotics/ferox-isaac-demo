"""Versioned SIM actuator realization of the existing named reference contract.

The sole body owner configures ONE implicit drive per joint. Its gain-weighted
target represents the blended PD/feedforward field exactly, before solver
integration and the source effort cap. There is no additional explicit torque,
invented rotor inertia, modified approved trajectory or firmware equivalence.
"""
from dataclasses import dataclass
from .arm_adapter import NamedBodyArbiter, UPPER_NAMES, _number


@dataclass(frozen=True)
class BodyDrive:
    joint_names: tuple
    articulation_indices: tuple
    target_position_rad: tuple
    target_velocity_rad_s: tuple
    stiffness_nm_rad: tuple
    damping_nm_s_rad: tuple
    source_effort_caps_nm: tuple
    feedforward_target_bias_rad: tuple
    current_pd_estimate_nm: tuple
    physics_sequence: int
    sim_time_s: float
    final_writer: str
    reference_owners: tuple
    mode: str
    actuation_semantics: str = 'implicit_biased_drive_v1; zero_additive_explicit_effort; one_source_cap'


class NamedBodyDriveArbiter(NamedBodyArbiter):
    def __init__(self, *, explicit_efforts_disabled, source_caps_verified,
                 maximum_feedforward_bias_rad=.25, **kwargs):
        if explicit_efforts_disabled is not True or source_caps_verified is not True:
            raise ValueError('zero explicit efforts and live source caps must be verified')
        self.maximum_feedforward_bias_rad = _number(maximum_feedforward_bias_rad,
            'maximum_feedforward_bias_rad', .001, .25)
        super().__init__(implicit_drives_disabled=False,
            actuation_backend='implicit_biased_drive_v1', **kwargs)

    def compose_drives(self, body_reference, *, controller_id, physics_sequence, now_monotonic_s):
        self._ready()
        try:
            self.check_freshness(now_monotonic_s)
            p = self._physics
            if controller_id != self._controller_id:
                raise ValueError('conflicting final body controller')
            if type(physics_sequence) is not int or physics_sequence != p['sequence']:
                raise ValueError('body reference is not for current physics sample')
            if physics_sequence <= self._last_applied:
                raise ValueError('second body write for same physics sample')
            body = self._joint_values(body_reference, self._names)
            positions, velocities, stiffness, damping, caps, biases, estimates, owners = ([] for _ in range(8))
            for name in self._names:
                a, b, alpha = body[name], body[name], 0.
                owner = self._controller_id
                if self._upper is not None and name in UPPER_NAMES:
                    b, alpha = self._upper['joints'][name], self._upper['blend_weight']
                    owner = 'hybrid(%s,upper_body_reference)' % self._controller_id
                kp = (1-alpha)*a['kp']+alpha*b['kp']
                kd = (1-alpha)*a['kd']+alpha*b['kd']
                if kp <= 0 or kd <= 0:
                    raise ValueError('implicit realization requires positive blended gains')
                ff = (1-alpha)*a['tau']+alpha*b['tau']
                q = ((1-alpha)*a['kp']*a['q']+alpha*b['kp']*b['q'])/kp
                dq = ((1-alpha)*a['kd']*a['dq']+alpha*b['kd']*b['dq'])/kd
                bound = self._bounds[name]
                bias = _number(ff/kp, name+'.feedforward_bias',
                    -self.maximum_feedforward_bias_rad, self.maximum_feedforward_bias_rad)
                target = _number(q+bias, name+'.effective_drive_target', bound.q_min, bound.q_max)
                estimate = _number(kp*(target-p['q'][name])+kd*(dq-p['dq'][name]), name+'.current_PD_estimate')
                positions.append(target); velocities.append(dq); stiffness.append(kp); damping.append(kd)
                caps.append(bound.max_effort); biases.append(bias); estimates.append(estimate); owners.append(owner)
            self._last_applied = physics_sequence
            return BodyDrive(self._names, tuple(self._indices[n] for n in self._names),
                tuple(positions),tuple(velocities),tuple(stiffness),tuple(damping),tuple(caps),tuple(biases),
                tuple(estimates),physics_sequence,p['sim_t'],'simulation_implicit_body_arbiter',tuple(owners),self.mode)
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self._refuse(exc)

    def reset_for_run(self, *, run_id, mode, controller_id, simulation_authorized):
        if run_id == self.run_id:
            raise ValueError('reset/handover needs a different run ID')
        return NamedBodyDriveArbiter(body_indices=self._indices,bounds=self._bounds,
            simulator_id=self.simulator_id,run_id=run_id,mode=mode,controller_id=controller_id,
            simulation_authorized=simulation_authorized,explicit_efforts_disabled=True,source_caps_verified=True,
            maximum_feedforward_bias_rad=self.maximum_feedforward_bias_rad,maximum_age_s=self.maximum_age_s)
