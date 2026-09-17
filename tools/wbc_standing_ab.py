"""Bare-versus-donor unsupported standing A/B: config, rig support and evaluator (CPU).

No Isaac import. The probe (tools/probes/wbc_standing_ab.py) uses these helpers; the
evaluator scores raw measured rows only, never commanded poses. Gates are the existing
campaign standing gates (assembled_balance.GATES) applied to the UNSUPPORTED interval,
plus explicit support-removal checks: the rig wrench must be exactly zero after the
recorded release and the feet must carry the body weight.
"""
from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
from pathlib import PurePosixPath

from assembled_balance import FEET, GATES, finite_tree, ground_contacts, material_self_penetrations, roll_pitch

ARMS = {
    'bare': {'source_urdf': 'g1_29dof_rev_1_0.urdf', 'hands_expected': False, 'joint_count': 29,
             'label': 'REFERENCE_ASSET bare g1_29dof_rev_1_0 (rubber hands 0.17 kg each; the g1_omni training embodiment)'},
    'donor': {'source_urdf': 'g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf', 'hands_expected': True, 'joint_count': 53,
              'label': 'DONOR assembled FTP hands (12 independent + 12 mimic joints; PROVISIONAL RH56DFTP donor)'},
}
PHASES = ('supported_settle', 'unsupported')
G = 9.81


@dataclass(frozen=True)
class StandingABConfig:
    arm: str
    locomotion_path: str = '/workspace/locomotion'
    policy_path: str = '/policy'
    ground_static_friction: float = 1.0
    ground_dynamic_friction: float = 1.0
    seed: int = 0
    hand_margin_rad: float = 0.02
    supported_settle_steps: int = 400
    unsupported_steps: int = 2000
    perturbation: dict = field(default_factory=lambda: {'joint_rad': 0.0, 'pelvis_xy_m': 0.0, 'pelvis_yaw_rad': 0.0})
    rig: dict = field(default_factory=lambda: {'kp_n_m': 4000.0, 'kd_n_s_m': 400.0, 'kr_nm_rad': 400.0, 'kdr_nm_s_rad': 40.0,
                                               'max_force_n': 800.0, 'max_torque_nm': 200.0})
    frame_every: int = 20
    execution_label: str = 'UNSUPPORTED_STANDING_AB'

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) - set(cls.__dataclass_fields__):
            raise ValueError('Unknown standing A/B configuration fields')
        obj = cls(**value)
        if obj.arm not in ARMS:
            raise ValueError('arm must be one of %s' % sorted(ARMS))
        for key, expected in [('locomotion_path', '/workspace/locomotion'), ('policy_path', '/policy')]:
            if getattr(obj, key) != expected or not PurePosixPath(getattr(obj, key)).is_absolute():
                raise ValueError('Use the admitted immutable private mounts')
        for x in (obj.ground_static_friction, obj.ground_dynamic_friction):
            if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or not 0 <= x <= 2:
                raise ValueError('Declared ground friction must be finite in [0,2]')
        if obj.ground_dynamic_friction > obj.ground_static_friction:
            raise ValueError('Dynamic friction exceeds static friction')
        if type(obj.seed) is not int or obj.seed < 0:
            raise ValueError('seed must be a nonnegative integer')
        if not (isinstance(obj.hand_margin_rad, (int, float)) and 0 < obj.hand_margin_rad <= 0.1):
            raise ValueError('hand_margin_rad must be in (0, 0.1]: exact-open zero margin is a separate diagnostic')
        if type(obj.supported_settle_steps) is not int or not 100 <= obj.supported_settle_steps <= 2000:
            raise ValueError('supported_settle_steps must be an integer in [100, 2000]')
        if type(obj.unsupported_steps) is not int or obj.unsupported_steps not in (2000, 6000):
            raise ValueError('unsupported_steps must be 2000 (10 s smoke) or 6000 (30 s hold)')
        p = obj.perturbation
        if set(p) != {'joint_rad', 'pelvis_xy_m', 'pelvis_yaw_rad'} or not finite_tree(p):
            raise ValueError('perturbation must declare joint_rad, pelvis_xy_m, pelvis_yaw_rad')
        if not (0 <= p['joint_rad'] <= 0.05 and 0 <= p['pelvis_xy_m'] <= 0.05 and 0 <= p['pelvis_yaw_rad'] <= 0.2):
            raise ValueError('perturbation magnitudes outside the declared envelope')
        r = obj.rig
        if set(r) != {'kp_n_m', 'kd_n_s_m', 'kr_nm_rad', 'kdr_nm_s_rad', 'max_force_n', 'max_torque_nm'} or not finite_tree(r):
            raise ValueError('rig gains must be complete and finite')
        if any(v <= 0 for v in r.values()):
            raise ValueError('rig gains and caps must be positive')
        if type(obj.frame_every) is not int or not 8 <= obj.frame_every <= 100:
            raise ValueError('frame_every must be an integer in [8, 100]')
        if obj.execution_label != 'UNSUPPORTED_STANDING_AB':
            raise ValueError('execution_label is fixed for this probe')
        return obj

    @property
    def unsupported_seconds(self):
        return self.unsupported_steps * GATES['physics_dt_s']


def config_sha256(cfg):
    return hashlib.sha256(json.dumps(asdict(cfg), sort_keys=True).encode()).hexdigest()


class Lcg:
    """Deterministic perturbation source recorded in the receipt; no numpy RNG state."""

    def __init__(self, seed):
        self.x = (seed * 2654435761 + 12345) & 0x7FFFFFFF

    def uniform(self, low, high):
        self.x = (1103515245 * self.x + 12345) & 0x7FFFFFFF
        return low + (high - low) * self.x / 0x7FFFFFFF


def perturbed_initial(default_q, limits, cfg):
    """Seeded joint offsets clipped inside the source limits, and a pelvis xy/yaw jitter."""
    rng = Lcg(cfg.seed)
    p = cfg.perturbation
    q = {}
    for name, value in default_q.items():
        lo, hi = limits[name]['lower'], limits[name]['upper']
        q[name] = min(hi, max(lo, value + rng.uniform(-p['joint_rad'], p['joint_rad'])))
    pelvis = {'dx_m': rng.uniform(-p['pelvis_xy_m'], p['pelvis_xy_m']), 'dy_m': rng.uniform(-p['pelvis_xy_m'], p['pelvis_xy_m']),
              'yaw_rad': rng.uniform(-p['pelvis_yaw_rad'], p['pelvis_yaw_rad'])}
    return q, pelvis


def rig_wrench(cfg, pose_xyzw, linear_velocity, angular_velocity, target_xyzw):
    """PD wrench holding the pelvis at the target pose; capped; returned in world frame.

    Orientation error uses the small-angle vector part of q_err = q_target * conj(q).
    """
    r = cfg.rig
    px, py, pz = pose_xyzw[:3]
    tx, ty, tz = target_xyzw[:3]
    force = [r['kp_n_m'] * (tx - px) - r['kd_n_s_m'] * linear_velocity[0],
             r['kp_n_m'] * (ty - py) - r['kd_n_s_m'] * linear_velocity[1],
             r['kp_n_m'] * (tz - pz) - r['kd_n_s_m'] * linear_velocity[2]]
    x, y, z, w = pose_xyzw[3:]
    tx_, ty_, tz_, tw_ = target_xyzw[3:]
    # q_err = q_t * conj(q)
    cx, cy, cz, cw = -x, -y, -z, w
    ex = tw_ * cx + tx_ * cw + ty_ * cz - tz_ * cy
    ey = tw_ * cy - tx_ * cz + ty_ * cw + tz_ * cx
    ez = tw_ * cz + tx_ * cy - ty_ * cx + tz_ * cw
    ew = tw_ * cw - tx_ * cx - ty_ * cy - tz_ * cz
    sign = 1.0 if ew >= 0 else -1.0
    torque = [r['kr_nm_rad'] * 2 * sign * ex - r['kdr_nm_s_rad'] * angular_velocity[0],
              r['kr_nm_rad'] * 2 * sign * ey - r['kdr_nm_s_rad'] * angular_velocity[1],
              r['kr_nm_rad'] * 2 * sign * ez - r['kdr_nm_s_rad'] * angular_velocity[2]]
    fn = math.sqrt(sum(v * v for v in force))
    if fn > r['max_force_n']:
        force = [v * r['max_force_n'] / fn for v in force]
    tn = math.sqrt(sum(v * v for v in torque))
    if tn > r['max_torque_nm']:
        torque = [v * r['max_torque_nm'] / tn for v in torque]
    return force, torque


def foot_ground_load(contacts):
    """|impulse| of loaded foot-ground contacts (N*s) per step, by foot."""
    feet, _ = ground_contacts(contacts)
    return {n: float(feet[n]) for n in FEET}


def evaluate_standing(rows, events, cfg, *, joint_count, limits, mimics, source_mass_kg, integrity_checks,
                      abort=None, adjacent_pairs=()):
    """Score the unsupported interval; reconstruct the first causal events.

    rows: campaign state rows extended with 'phase' and 'support'. events: list of
    {'sequence', 'name'} with exactly one 'support_release'.
    """
    dt = GATES['physics_dt_s']
    required = cfg.unsupported_steps
    release = [e for e in events if e.get('name') == 'support_release']
    checks = {'single_recorded_release': len(release) == 1, 'finite_complete_measured_state': bool(rows),
              'contiguous_physics_sequence': bool(rows), 'single_body_owner_named_policy': bool(rows),
              'supported_settle_completed': False, 'no_support_after_release': True,
              'feet_carry_body_weight_after_release': False, 'full_unsupported_duration': False,
              'minimum_pelvis_height': True, 'roll_pitch_within_gate': True, 'pelvis_xy_drift_within_gate': True,
              'each_foot_xy_drift_within_gate': True, 'each_foot_height_rise_within_gate': True,
              'no_loaded_nonfoot_ground_contact': True, 'no_material_loaded_nonadjacent_self_penetration': True,
              'both_feet_have_measured_loaded_contact': False, 'source_joint_limits_within_gate': True,
              'hand_coupling_within_gate': True, 'controller_active_throughout': True,
              'no_sustained_drive_saturation': True, 'no_overspeed': True, 'no_abort': abort is None}
    checks.update({'initialization_' + k: bool(v) for k, v in integrity_checks.items()})
    peak = {'pelvis_xy_drift_m': 0., 'abs_roll_pitch_rad': 0., 'joint_limit_violation_rad': 0., 'coupling_error_rad': 0.,
            'foot_xy_drift_m': {n: 0. for n in FEET}, 'foot_height_rise_m': {n: 0. for n in FEET},
            'support_force_after_release_n': 0., 'max_abs_joint_velocity_rad_s': 0., 'drive_near_cap_steps': 0,
            'drive_near_cap_consecutive_max': 0, 'settle_rig_force_peak_n': 0., 'settle_rig_torque_peak_nm': 0.}
    events_out = []
    min_height = None
    release_seq = release[0]['sequence'] if release else None
    reference = None
    unsupported = 0
    contact_steps = {n: 0 for n in FEET}
    load_sum = 0.
    inference_steps = 0
    near_cap_run = 0
    self_pen = []
    def event(seq, name, detail=None):
        if not any(e['name'] == name for e in events_out):
            events_out.append({'sequence': seq, 'physics_s': seq * dt, 'name': name, 'detail': detail})
    try:
        last_seq = None
        for row in rows:
            if not finite_tree(row):
                raise ValueError('Nonfinite raw evidence')
            seq = row['sequence']
            if last_seq is not None and seq != last_seq + 1:
                checks['contiguous_physics_sequence'] = False
            last_seq = seq
            names = row['runtime_names']
            if len(names) != joint_count or len(set(names)) != joint_count or set(names) != set(limits):
                raise ValueError('Incomplete named articulation (expected %d joints)' % joint_count)
            q, dq = row['q_rad'], row['dq_rad_s']
            if len(q) != joint_count or len(dq) != joint_count:
                raise ValueError('Incomplete measured coordinates')
            if len(row['policy_observation']) != 480 or len(row['policy_action']) != 29:
                raise ValueError('Missing actual policy computation')
            if (row['command_velocity'] != [0., 0., 0.] or row['body_command_owner'] != 'named_policy_single_writer'
                    or len(row['body_command_names']) != 29 or len(set(row['body_command_names'])) != 29
                    or not set(row['body_command_names']).issubset(names) or len(row['body_command_rad']) != 29):
                checks['single_body_owner_named_policy'] = False
            if row.get('policy_inference_this_step'):
                inference_steps += 1
            sup = row['support']
            positions = dict(zip(names, q))
            for n, lim in limits.items():
                v = max(lim['lower'] - positions[n], positions[n] - lim['upper'])
                if v > peak['joint_limit_violation_rad']:
                    peak['joint_limit_violation_rad'] = v
                if v > GATES['maximum_joint_limit_violation_rad']:
                    event(seq, 'joint_limit_violation', {'joint': n, 'rad': v})
                if abs(dq[names.index(n)]) > lim['velocity']:
                    checks['no_overspeed'] = False
                    event(seq, 'overspeed', {'joint': n, 'rad_s': dq[names.index(n)]})
            peak['max_abs_joint_velocity_rad_s'] = max(peak['max_abs_joint_velocity_rad_s'], max(abs(v) for v in dq))
            for n, m in mimics.items():
                peak['coupling_error_rad'] = max(peak['coupling_error_rad'], abs(positions[n] - positions[m['parent']] * m['multiplier'] - m['offset']))
            near = row.get('drive_estimate_near_cap_names', [])
            if near:
                peak['drive_near_cap_steps'] += 1
                near_cap_run += 1
                peak['drive_near_cap_consecutive_max'] = max(peak['drive_near_cap_consecutive_max'], near_cap_run)
                event(seq, 'drive_near_effort_cap', {'joints': near[:6]})
            else:
                near_cap_run = 0
            feet_contacts, other = ground_contacts(row['contacts'])
            if other:
                event(seq, 'nonfoot_ground_contact', {'links': sorted({c['actor0'].split('/')[-1] + '|' + c['actor1'].split('/')[-1] for c in other})[:4]})
            pens = material_self_penetrations(row['contacts'], adjacent_pairs) if adjacent_pairs else []
            if pens:
                self_pen.extend(pens)
                event(seq, 'material_self_penetration', {'pairs': [p.get('link_pair') for p in pens[:3]]})
            if row['phase'] == 'supported_settle':
                if release_seq is not None and seq > release_seq:
                    raise ValueError('supported_settle row after the recorded release')
                if sup.get('kind') != 'RIG_WRENCH':
                    raise ValueError('settle rows must declare the rig wrench support')
                f = math.sqrt(sum(v * v for v in sup['force_n']))
                t = math.sqrt(sum(v * v for v in sup['torque_nm']))
                peak['settle_rig_force_peak_n'] = max(peak['settle_rig_force_peak_n'], f)
                peak['settle_rig_torque_peak_nm'] = max(peak['settle_rig_torque_peak_nm'], t)
                continue
            if row['phase'] != 'unsupported':
                raise ValueError('unknown phase ' + repr(row['phase']))
            if release_seq is None or seq <= release_seq:
                raise ValueError('unsupported row without a preceding recorded release')
            if sup.get('kind') != 'NONE' or any(v != 0. for v in sup['force_n']) or any(v != 0. for v in sup['torque_nm']):
                checks['no_support_after_release'] = False
                event(seq, 'hidden_support_after_release', {'kind': sup.get('kind')})
            if reference is None:
                reference = row
                checks['supported_settle_completed'] = True
            unsupported += 1
            pose = row['link_poses_world_xyzw']['pelvis']
            roll, pitch = roll_pitch(pose)
            min_height = pose[2] if min_height is None else min(min_height, pose[2])
            if max(abs(roll), abs(pitch)) > peak['abs_roll_pitch_rad']:
                peak['abs_roll_pitch_rad'] = max(abs(roll), abs(pitch))
            if max(abs(roll), abs(pitch)) > 0.1:
                event(seq, 'base_tilt_exceeds_0.1rad', {'roll': roll, 'pitch': pitch})
            ref = reference['link_poses_world_xyzw']['pelvis']
            peak['pelvis_xy_drift_m'] = max(peak['pelvis_xy_drift_m'], math.hypot(pose[0] - ref[0], pose[1] - ref[1]))
            load = foot_ground_load(row['contacts'])
            load_sum += sum(load.values())
            for n in FEET:
                p = row['link_poses_world_xyzw'][n]
                ref = reference['link_poses_world_xyzw'][n]
                d = math.hypot(p[0] - ref[0], p[1] - ref[1])
                peak['foot_xy_drift_m'][n] = max(peak['foot_xy_drift_m'][n], d)
                peak['foot_height_rise_m'][n] = max(peak['foot_height_rise_m'][n], p[2] - ref[2])
                if d > GATES['maximum_each_foot_xy_drift_m']:
                    event(seq, 'foot_drift_exceeds_gate', {'foot': n, 'm': d})
                if load[n] > GATES['loaded_contact_impulse_threshold_ns']:
                    contact_steps[n] += 1
                elif unsupported > 1:
                    event(seq, 'foot_unloaded', {'foot': n})
            if other:
                checks['no_loaded_nonfoot_ground_contact'] = False
            if pens:
                checks['no_material_loaded_nonadjacent_self_penetration'] = False
        if abort is not None:
            event(abort.get('sequence', last_seq if last_seq is not None else -1), 'abort', abort.get('reason'))
    except ValueError as exc:
        checks['finite_complete_measured_state'] = False
        checks['no_abort'] = False
        event(last_seq if last_seq is not None else -1, 'evidence_error', str(exc))
    checks['full_unsupported_duration'] = unsupported >= required
    checks['minimum_pelvis_height'] = min_height is not None and min_height >= GATES['minimum_pelvis_height_m']
    checks['roll_pitch_within_gate'] = peak['abs_roll_pitch_rad'] <= GATES['maximum_abs_roll_pitch_rad']
    checks['pelvis_xy_drift_within_gate'] = peak['pelvis_xy_drift_m'] <= GATES['maximum_pelvis_xy_drift_m']
    checks['each_foot_xy_drift_within_gate'] = all(v <= GATES['maximum_each_foot_xy_drift_m'] for v in peak['foot_xy_drift_m'].values())
    checks['each_foot_height_rise_within_gate'] = all(v <= GATES['maximum_each_foot_height_rise_m'] for v in peak['foot_height_rise_m'].values())
    checks['both_feet_have_measured_loaded_contact'] = unsupported > 0 and all(contact_steps[n] >= unsupported - 1 for n in FEET)
    checks['source_joint_limits_within_gate'] = peak['joint_limit_violation_rad'] <= GATES['maximum_joint_limit_violation_rad']
    checks['hand_coupling_within_gate'] = peak['coupling_error_rad'] <= GATES['maximum_coupling_error_rad']
    expected_inferences = math.ceil(len(rows) / 4)
    checks['controller_active_throughout'] = inference_steps >= expected_inferences - 1
    checks['no_sustained_drive_saturation'] = (peak['drive_near_cap_steps'] <= 0.05 * max(1, len(rows))
                                               and peak['drive_near_cap_consecutive_max'] <= 20)
    mean_load_n = (load_sum / unsupported / dt) if unsupported else 0.
    peak['mean_foot_ground_load_n'] = mean_load_n
    peak['body_weight_n'] = source_mass_kg * G
    checks['feet_carry_body_weight_after_release'] = unsupported > 0 and mean_load_n >= 0.9 * source_mass_kg * G
    status = 'PASS' if all(checks.values()) else 'FAIL'
    events_out.sort(key=lambda e: e['sequence'])
    return {'status': status, 'checks': checks, 'peaks': peak, 'steps': len(rows), 'unsupported_steps': unsupported,
            'unsupported_seconds': unsupported * dt, 'required_unsupported_steps': required,
            'recorded_release_sequence': release_seq, 'minimum_pelvis_height_m': min_height,
            'first_causal_events': events_out[:12], 'first_failed_gate': next((k for k, v in checks.items() if not v), None),
            'abort': abort, 'arm': cfg.arm, 'arm_label': ARMS[cfg.arm]['label'], 'seed': cfg.seed,
            'hand_margin_rad': cfg.hand_margin_rad, 'execution_label': cfg.execution_label,
            'controller_mode': 'policy', 'source_mass_kg': source_mass_kg, 'physics_dt': dt,
            'material_self_penetrations': self_pen[:20], 'gates': GATES,
            'scope': 'UNSUPPORTED after a recorded rig release; %s; no support after release; PhysX twin' % ARMS[cfg.arm]['label']}
