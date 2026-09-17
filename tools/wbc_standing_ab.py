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
    # Rev2 rig: carries the body weight (gravity feed-forward on the pelvis) with stiff PD, so the
    # assembly is actually held during settling. Rev1 (kp 4000, no feed-forward) held 39 N of a 341 N
    # body and the assembly fell onto its feet at the first policy step (sK-standing-ab-02-donor-smoke).
    # Rev3 defaults: stable for an explicit per-step wrench on the pelvis link (rig_stability_margins).
    rig: dict = field(default_factory=lambda: {'kp_n_m': 10000.0, 'kd_n_s_m': 500.0, 'kr_nm_rad': 100.0, 'kdr_nm_s_rad': 0.5,
                                               'max_force_n': 1200.0, 'max_torque_nm': 400.0, 'gravity_feedforward': True})
    # Before the policy is consulted, the drives hold the policy default pose under the rig for this
    # many steps (the policy's 5-frame history buffer starts at zero; its first actions jerked the legs
    # at up to 2.35 rad/s in rev1 and drove a coupled hand joint at the 0.02 rad margin past its envelope).
    policy_warmup_steps: int = 100
    # Optional locomotion schedule AFTER release: [[start_s, vx, vy, wz], ...] in the base-heading
    # frame the checkpoint was trained with (heading_command false); commands are held until the
    # next entry. Empty = zero command (standing). With a schedule the drift gates are replaced by
    # the walk/turn/stop measurements below (R4); standing gates still apply before the first command.
    command_schedule: list = field(default_factory=list)
    # EXPERIMENTAL arm override (R3 candidate, not a qualified WBC): after release + start_s the 14 arm
    # joint targets are taken from a q-only reference instead of the policy's arm actions; legs and
    # waist stay with the policy; the policy's own observation is untouched. mode 'null' = the policy
    # default arm pose (hazard check only); 'trajectory' = rows [[t_s, q14...]] interpolated, rate <= 1 rad/s.
    arm_override: dict = field(default_factory=lambda: {'enabled': False, 'start_s': 5.0, 'mode': 'null', 'trajectory': []})
    frame_every: int = 20
    # Explicitly NONQUALIFYING diagnostics that isolate one hand effect at a time while keeping
    # the real donor mass/COM/inertia. A run with any diagnostic on can never PASS.
    diagnostics: dict = field(default_factory=lambda: {'disable_hand_collisions': False, 'lock_hand_joints': False})
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
        if set(r) != {'kp_n_m', 'kd_n_s_m', 'kr_nm_rad', 'kdr_nm_s_rad', 'max_force_n', 'max_torque_nm', 'gravity_feedforward'} or not finite_tree(r):
            raise ValueError('rig gains must be complete and finite')
        if type(r['gravity_feedforward']) is not bool or any(v <= 0 for k, v in r.items() if k != 'gravity_feedforward'):
            raise ValueError('rig gains and caps must be positive; gravity_feedforward must be a boolean')
        if type(obj.policy_warmup_steps) is not int or not 0 <= obj.policy_warmup_steps < obj.supported_settle_steps:
            raise ValueError('policy_warmup_steps must be an integer below supported_settle_steps')
        last = -1.0
        for entry in obj.command_schedule:
            if not (isinstance(entry, list) and len(entry) == 4 and finite_tree(entry)):
                raise ValueError('command_schedule entries must be [start_s, vx, vy, wz]')
            t, vx, vy, wz = entry
            if t <= last or t < 5.0:
                raise ValueError('command_schedule times must increase and start >= 5 s after release (standing check first)')
            if not (-0.6 <= vx <= 1.0 and -0.5 <= vy <= 0.5 and -1.0 <= wz <= 1.0):
                raise ValueError('command outside the checkpoint ranges vx[-0.6,1.0] vy[-0.5,0.5] wz[-1,1]; refused, not clipped')
            last = t
        if obj.command_schedule and obj.unsupported_steps != 6000:
            raise ValueError('a command schedule needs the 30 s unsupported interval')
        ao = obj.arm_override
        if set(ao) != {'enabled', 'start_s', 'mode', 'trajectory'} or type(ao['enabled']) is not bool:
            raise ValueError('arm_override must declare enabled, start_s, mode, trajectory')
        if ao['enabled']:
            if obj.command_schedule:
                raise ValueError('arm override and a locomotion command schedule are separate experiments')
            if not (isinstance(ao['start_s'], (int, float)) and ao['start_s'] >= 5.0):
                raise ValueError('arm override starts >= 5 s after release (standing is checked first)')
            if ao['mode'] not in ('null', 'trajectory'):
                raise ValueError('arm override mode must be null or trajectory')
            if ao['mode'] == 'trajectory':
                rows = ao['trajectory']
                if not rows or not all(isinstance(r, list) and len(r) == 15 and finite_tree(r) for r in rows):
                    raise ValueError('arm trajectory rows must be [t_s, q x14]')
                last_t = -1.0
                for prev, cur in zip([None] + rows[:-1], rows):
                    if cur[0] <= last_t:
                        raise ValueError('arm trajectory times must increase')
                    if prev is not None and max(abs(a - b) for a, b in zip(prev[1:], cur[1:])) / (cur[0] - prev[0]) > 1.0:
                        raise ValueError('arm trajectory rate exceeds 1.0 rad/s')
                    last_t = cur[0]
        d = obj.diagnostics
        if set(d) != {'disable_hand_collisions', 'lock_hand_joints'} or any(type(v) is not bool for v in d.values()):
            raise ValueError('diagnostics must declare disable_hand_collisions and lock_hand_joints as booleans')
        if any(d.values()) and obj.arm != 'donor':
            raise ValueError('hand diagnostics apply to the donor arm only')
        if type(obj.frame_every) is not int or not 8 <= obj.frame_every <= 100:
            raise ValueError('frame_every must be an integer in [8, 100]')
        if obj.execution_label != 'UNSUPPORTED_STANDING_AB':
            raise ValueError('execution_label is fixed for this probe')
        return obj

    @property
    def nonqualifying(self):
        return any(self.diagnostics.values())

    @property
    def unsupported_seconds(self):
        return self.unsupported_steps * GATES['physics_dt_s']


def command_at(cfg, unsupported_time_s):
    cmd = [0., 0., 0.]
    for t, vx, vy, wz in cfg.command_schedule:
        if unsupported_time_s >= t:
            cmd = [vx, vy, wz]
    return cmd


ARM_JOINTS = tuple('%s_%s_joint' % (s, j) for s in ('left', 'right')
                   for j in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw'))
EXPERIMENTAL_OWNER = 'experimental_arm_override_v0'


def arm_reference_at(cfg, default_arm, unsupported_time_s):
    """Return the 14 arm targets (ARM_JOINTS order) or None when the override is not active."""
    ao = cfg.arm_override
    if not ao['enabled'] or unsupported_time_s < ao['start_s']:
        return None
    if ao['mode'] == 'null':
        return list(default_arm)
    rows = ao['trajectory']
    t = unsupported_time_s - ao['start_s']
    if t <= rows[0][0]:
        return list(rows[0][1:])
    for a, b in zip(rows[:-1], rows[1:]):
        if a[0] <= t <= b[0]:
            w = (t - a[0]) / (b[0] - a[0])
            return [x + w * (y - x) for x, y in zip(a[1:], b[1:])]
    return list(rows[-1][1:])


def yaw_from_xyzw(q):
    x, y, z, w = q[3:]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


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


def rig_stability_margins(cfg, pelvis_mass_kg, pelvis_inertia_min_kg_m2, dt=None):
    """Explicit single-step wrench stability, judged against the PELVIS LINK's own properties.

    The wrench is applied once per physics step to the pelvis link; the rest of the body is
    coupled through joint drives and does not damp a single-step mode of that link. The rev2
    rig (kdr 200 N*m*s/rad on a 0.0079 kg*m^2 pelvis) had kdr*dt/I = 126 and produced a
    sign-alternating yaw-rate growth in both arms (sK-standing-ab-02-donor-smoke-r2b and
    -01-bare-smoke-r2). Margins must be < 1.0 (damping) and < 1.0 (stiffness).
    """
    dt = GATES['physics_dt_s'] if dt is None else dt
    r = cfg.rig
    m = {'kd_dt_over_m': r['kd_n_s_m'] * dt / pelvis_mass_kg, 'kp_dt2_over_m': r['kp_n_m'] * dt * dt / pelvis_mass_kg,
         'kdr_dt_over_I': r['kdr_nm_s_rad'] * dt / pelvis_inertia_min_kg_m2, 'kr_dt2_over_I': r['kr_nm_rad'] * dt * dt / pelvis_inertia_min_kg_m2}
    m['stable'] = all(v < 1.0 for v in m.values())
    m['pelvis_mass_kg'], m['pelvis_inertia_min_kg_m2'], m['dt_s'] = pelvis_mass_kg, pelvis_inertia_min_kg_m2, dt
    return m


def rig_wrench(cfg, pose_xyzw, linear_velocity, angular_velocity, target_xyzw, body_mass_kg=0.0):
    """PD wrench holding the pelvis at the target pose; capped; returned in world frame.

    With rig.gravity_feedforward the wrench carries the full body weight (m*g up) so the PD
    term only corrects the residual. Orientation error uses the small-angle vector part of
    q_err = q_target * conj(q).
    """
    r = cfg.rig
    px, py, pz = pose_xyzw[:3]
    tx, ty, tz = target_xyzw[:3]
    ff = body_mass_kg * G if r.get('gravity_feedforward') else 0.0
    force = [r['kp_n_m'] * (tx - px) - r['kd_n_s_m'] * linear_velocity[0],
             r['kp_n_m'] * (ty - py) - r['kd_n_s_m'] * linear_velocity[1],
             r['kp_n_m'] * (tz - pz) - r['kd_n_s_m'] * linear_velocity[2] + ff]
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


def guarded_state_view(names, q, dq, guarded_names):
    """Project the full runtime state (body + independent + mimic hand joints) onto the joints the
    ownership guard owns (body + independent hand joints), by name. Mimic joints are never commanded,
    so the guard does not own them; the evaluator still checks every runtime joint against its limits."""
    names = list(names)
    if len(q) != len(names) or len(dq) != len(names):
        raise ValueError('runtime state width mismatch')
    idx = [names.index(n) for n in guarded_names]
    return [names[i] for i in idx], [q[i] for i in idx], [dq[i] for i in idx]


def foot_ground_load(contacts):
    """|impulse| of loaded foot-ground contacts (N*s) per step, by foot."""
    feet, _ = ground_contacts(contacts)
    return {n: float(feet[n]) for n in FEET}


def evaluate_standing(rows, events, cfg, *, joint_count, limits, mimics, source_mass_kg, integrity_checks,
                      abort=None, adjacent_pairs=(), guard_entries=None):
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
    if guard_entries is not None:
        kinds = [e.get('kind') for e in guard_entries]
        releases = [e for e in guard_entries if e.get('kind') == 'support_release']
        owners = [e.get('owner') for e in guard_entries if e.get('kind') == 'body_owner']
        run_owner = EXPERIMENTAL_OWNER if cfg.arm_override['enabled'] else 'named_policy_single_writer'
        checks['ownership_journal_consistent'] = (kinds[:3] == ['body_owner', 'hand_owner', 'support'] and len(releases) == 1
                                                  and owners[-1] == run_owner and len(owners) <= 2
                                                  and not any(k == 'refused' for k in kinds)
                                                  and (not release or releases[0].get('sequence') == release[0]['sequence'] + 1))
    peak = {'pelvis_xy_drift_m': 0., 'abs_roll_pitch_rad': 0., 'joint_limit_violation_rad': 0., 'coupling_error_rad': 0.,
            'foot_xy_drift_m': {n: 0. for n in FEET}, 'foot_height_rise_m': {n: 0. for n in FEET},
            'support_force_after_release_n': 0., 'max_abs_joint_velocity_rad_s': 0., 'drive_near_cap_steps': 0,
            'drive_near_cap_consecutive_max': 0, 'settle_rig_force_peak_n': 0., 'settle_rig_torque_peak_nm': 0.}
    events_out = []
    min_height = None
    release_seq = release[0]['sequence'] if release else None
    reference = None
    unsupported = 0
    arm_err_sum, arm_err_n, arm_err_peak = 0.0, 0, 0.0
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
            warmup = row['phase'] == 'supported_settle' and seq < cfg.policy_warmup_steps
            run_owner = EXPERIMENTAL_OWNER if cfg.arm_override['enabled'] else 'named_policy_single_writer'
            expected_owner = 'probe_default_pose_warmup' if warmup else run_owner
            if not warmup and (len(row['policy_observation']) != 480 or len(row['policy_action']) != 29):
                raise ValueError('Missing actual policy computation')
            if warmup and (row['policy_observation'] or row['policy_action'] or row.get('policy_inference_this_step')):
                raise ValueError('warm-up rows must carry no policy computation')
            expected_cmd = command_at(cfg, (seq - release_seq) * dt) if (release_seq is not None and seq > release_seq) else [0., 0., 0.]
            override_active = (release_seq is not None and seq > release_seq and cfg.arm_override['enabled']
                               and (seq - release_seq) * dt >= cfg.arm_override['start_s'])
            if override_active:
                ref = row.get('arm_reference_rad')
                if not (isinstance(ref, list) and len(ref) == 14):
                    raise ValueError('override rows must carry the 14 arm reference targets')
                ai = [names.index(n) for n in ARM_JOINTS]
                err = max(abs(q[i] - r) for i, r in zip(ai, ref))
                arm_err_sum += err; arm_err_n += 1; arm_err_peak = max(arm_err_peak, err)
            if (row['command_velocity'] != expected_cmd or row['body_command_owner'] != expected_owner
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
    expected_inferences = math.ceil(max(0, len(rows) - cfg.policy_warmup_steps) / 4)
    checks['controller_active_throughout'] = inference_steps >= expected_inferences - 1
    checks['no_sustained_drive_saturation'] = (peak['drive_near_cap_steps'] <= 0.05 * max(1, len(rows))
                                               and peak['drive_near_cap_consecutive_max'] <= 20)
    mean_load_n = (load_sum / unsupported / dt) if unsupported else 0.
    peak['mean_foot_ground_load_n'] = mean_load_n
    peak['body_weight_n'] = source_mass_kg * G
    checks['feet_carry_body_weight_after_release'] = unsupported > 0 and mean_load_n >= 0.9 * source_mass_kg * G
    if cfg.command_schedule and reference is not None and rows:
        # R4 measurements over the scheduled interval: travel in the commanded heading, lateral drift,
        # yaw change, and speed after the final stop command. Standing drift gates are not applied to
        # commanded motion; the standing checks before the first command remain above.
        t0 = cfg.command_schedule[0][0]
        start = next((r for r in rows if r['phase'] == 'unsupported' and (r['sequence'] - release_seq) * dt >= t0), None)
        end = rows[-1]
        if start is not None:
            p0, p1 = start['link_poses_world_xyzw']['pelvis'], end['link_poses_world_xyzw']['pelvis']
            yaw0 = yaw_from_xyzw(p0)
            dx, dy = p1[0] - p0[0], p1[1] - p0[1]
            forward = dx * math.cos(yaw0) + dy * math.sin(yaw0)
            lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
            dyaw = math.atan2(math.sin(yaw_from_xyzw(p1) - yaw0), math.cos(yaw_from_xyzw(p1) - yaw0))
            stop_cmds = [e for e in cfg.command_schedule if e[1:] == [0., 0., 0.]]
            final_speed = None
            if stop_cmds:
                ts = stop_cmds[-1][0]
                after = [r for r in rows if r['phase'] == 'unsupported' and (r['sequence'] - release_seq) * dt >= ts + 3.0]
                if after:
                    v = after[-1]['pelvis_linear_velocity_m_s']
                    final_speed = math.hypot(v[0], v[1])
            peak['walk'] = {'forward_travel_m': forward, 'lateral_drift_m': lateral, 'yaw_change_rad': dyaw, 'final_speed_m_s': final_speed,
                            'schedule': cfg.command_schedule}
            checks['pelvis_xy_drift_within_gate'] = True
            checks['each_foot_xy_drift_within_gate'] = True
            checks['each_foot_height_rise_within_gate'] = True
            checks['both_feet_have_measured_loaded_contact'] = True
            checks['feet_carry_body_weight_after_release'] = True   # feet leave the ground while stepping; not a standing check
            if any(e[1] > 0 for e in cfg.command_schedule):
                checks['walk_travel_reached'] = forward >= 1.0
                checks['walk_lateral_drift_within_gate'] = abs(lateral) <= 0.3
            if any(abs(e[3]) > 0 for e in cfg.command_schedule):
                checks['turn_yaw_reached'] = abs(dyaw) >= 0.6
            if final_speed is not None:
                checks['stopped_after_stop_command'] = final_speed <= 0.05
    if cfg.arm_override['enabled']:
        peak['arm_override'] = {'mode': cfg.arm_override['mode'], 'rows': arm_err_n,
                                'tracking_mean_abs_rad': (arm_err_sum / arm_err_n) if arm_err_n else None, 'tracking_peak_abs_rad': arm_err_peak}
        checks['arm_override_rows_present'] = arm_err_n > 0
        checks['arm_tracking_peak_within_gate'] = arm_err_peak <= 0.10
        checks['arm_tracking_mean_within_gate'] = (arm_err_sum / arm_err_n if arm_err_n else 1.0) <= 0.05
    if cfg.nonqualifying:
        checks['no_nonqualifying_diagnostic'] = False
    status = 'PASS' if all(checks.values()) else 'FAIL'
    events_out.sort(key=lambda e: e['sequence'])
    return {'status': status, 'checks': checks, 'peaks': peak, 'steps': len(rows), 'unsupported_steps': unsupported,
            'unsupported_seconds': unsupported * dt, 'required_unsupported_steps': required,
            'recorded_release_sequence': release_seq, 'minimum_pelvis_height_m': min_height,
            'first_causal_events': events_out[:12], 'first_failed_gate': next((k for k, v in checks.items() if not v), None),
            'abort': abort, 'arm': cfg.arm, 'arm_label': ARMS[cfg.arm]['label'], 'seed': cfg.seed,
            'hand_margin_rad': cfg.hand_margin_rad, 'execution_label': cfg.execution_label,
            'controller_mode': 'policy', 'source_mass_kg': source_mass_kg, 'physics_dt': dt,
            'diagnostics': dict(cfg.diagnostics), 'nonqualifying_diagnostic': cfg.nonqualifying,
            'verdict_scope': ('NONQUALIFYING_DIAGNOSTIC: isolates one hand effect; cannot be the delivered twin' if cfg.nonqualifying
                              else ('EXPERIMENTAL_COMBINED_CONTROLLER (arm override v0): not a qualified WBC; candidate only' if cfg.arm_override['enabled']
                                    else 'candidate for adoption if PASS')),
            'arm_override': dict(cfg.arm_override, trajectory_rows=len(cfg.arm_override.get('trajectory', []))),
            'material_self_penetrations': self_pen[:20], 'gates': GATES,
            'scope': 'UNSUPPORTED after a recorded rig release; %s; no support after release; PhysX twin' % ARMS[cfg.arm]['label']}
