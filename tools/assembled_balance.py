"""Admission and measured-state gates for the provisional standing diagnostic.

No Isaac, policy, or private calibration import is needed to evaluate evidence.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import PurePosixPath
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class BalanceConfig:
    locomotion_path: str = '/workspace/locomotion'
    policy_path: str = '/policy'
    ground_static_friction: float = 1.0
    ground_dynamic_friction: float = 1.0
    controller_mode: str = 'policy'
    source_contact_equilibrium: dict | None = None

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) - set(cls.__dataclass_fields__):
            raise ValueError('Unknown balance configuration fields')
        obj = cls(**value)
        for key, expected in [('locomotion_path', '/workspace/locomotion'), ('policy_path', '/policy')]:
            if getattr(obj, key) != expected or not PurePosixPath(getattr(obj, key)).is_absolute():
                raise ValueError('Use the admitted immutable private mounts')
        for x in (obj.ground_static_friction, obj.ground_dynamic_friction):
            if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or not 0 <= x <= 2:
                raise ValueError('Declared ground friction must be finite in [0,2]')
        if obj.ground_dynamic_friction > obj.ground_static_friction:
            raise ValueError('Dynamic friction exceeds static friction')
        if obj.controller_mode not in ('policy', 'source_contact_equilibrium_pd', 'source_equilibrium_implicit_target_bias_v1'):
            raise ValueError('Unknown balance controller')
        if (obj.controller_mode == 'policy') != (obj.source_contact_equilibrium is None):
            raise ValueError('Equilibrium mode alone requires its immutable source reference')
        if obj.source_contact_equilibrium is not None and (not isinstance(obj.source_contact_equilibrium, dict)
                or not finite_tree(obj.source_contact_equilibrium)):
            raise ValueError('Invalid source equilibrium reference')
        return obj


GATES = {'physics_dt_s': .005, 'required_steps': 2000, 'duration_s': 10.,
         'initial_foot_clearance_m': .002, 'minimum_pelvis_height_m': .65,
         'maximum_abs_roll_pitch_rad': .2, 'maximum_pelvis_xy_drift_m': .1,
         'maximum_each_foot_xy_drift_m': .03, 'maximum_each_foot_height_rise_m': .03,
         'maximum_joint_limit_violation_rad': .03, 'maximum_coupling_error_rad': .03,
         'loaded_contact_impulse_threshold_ns': 1e-9,
         'maximum_loaded_nonadjacent_self_penetration_m': .001,
         'material_self_penetration_pair_impulse_ns': .005}
FEET = ('left_ankle_roll_link', 'right_ankle_roll_link')
GROUND = '/World/Ground'


class SourceEquilibriumPD:
    """Fixed source-equilibrium feedforward plus exported PD; no base wrench.

    All model-specific constants arrive through the immutable private config.
    The controller accepts named measured coordinates and returns capped body
    efforts. It never calls an articulation or simulates a support constraint.
    """
    def __init__(self, reference, names, default, kp, kd, caps, *, source_sha256):
        self.names = list(names)
        if len(self.names) != 29 or len(set(self.names)) != 29:
            raise ValueError('Equilibrium controller requires exactly 29 unique body names')
        self.reference = reference
        if (not isinstance(reference, dict) or not finite_tree(reference)
                or reference.get('source_sha256') != source_sha256
                or reference.get('hardware_authorized') is not False):
            raise ValueError('Equilibrium source identity or authorization mismatch')
        def vector(values, label, *, positive=False, nonnegative=False):
            values = list(values)
            if (len(values) != 29 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                    or not math.isfinite(v) or (positive and v <= 0) or (nonnegative and v < 0) for v in values)):
                raise ValueError('Invalid named controller ' + label)
            return [float(v) for v in values]
        def named(key):
            mapping = reference.get(key)
            if not isinstance(mapping, dict) or set(mapping) != set(self.names):
                raise ValueError('Incomplete source equilibrium map: ' + key)
            return vector([mapping[n] for n in self.names], key)
        self.default = vector(default, 'default')
        home = named('source_named29_home_rad')
        if any(abs(a-b) > 1e-12 for a, b in zip(home, self.default)):
            raise ValueError('Source equilibrium was computed at a different nominal pose')
        self.ff = named('named29_contact_equilibrium_effort_nm')
        self.kp, self.kd, self.caps = vector(kp, 'kp', nonnegative=True), vector(kd, 'kd', nonnegative=True), vector(caps, 'caps', positive=True)
        if any(abs(t) > cap for t, cap in zip(self.ff, self.caps)):
            raise ValueError('Source equilibrium alone exceeds an unchanged source effort limit')
        residual = reference.get('equilibrium_base_wrench_residual', [])
        forces = reference.get('minimum_norm_positive_normal_contact_forces', [])
        if (len(residual) != 6 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in residual)
                or math.sqrt(sum(x*x for x in residual)) > 1e-6 or len(forces) != 8
                or any(not isinstance(r, dict) or not isinstance(r.get('equilibrium_vertical_force_n'), (int, float))
                       or not math.isfinite(r['equilibrium_vertical_force_n']) or r['equilibrium_vertical_force_n'] <= 0 for r in forces)):
            raise ValueError('Source reference lacks positive eight-point free-base equilibrium')

    def compute(self, runtime_names, q, dq):
        if (len(runtime_names) != 53 or len(set(runtime_names)) != 53 or len(q) != 53 or len(dq) != 53
                or not set(self.names).issubset(runtime_names) or not finite_tree([q, dq])):
            raise ValueError('Expected complete finite named53 measured coordinates')
        indices = [runtime_names.index(n) for n in self.names]
        position, velocity = [float(q[i]) for i in indices], [float(dq[i]) for i in indices]
        if not all(math.isfinite(float(v)) for v in list(q) + list(dq)):
            raise ValueError('Nonfinite measured joint feedback')
        raw = [f + k*(home-p) - d*v for f, k, home, p, d, v in
               zip(self.ff, self.kp, self.default, position, self.kd, velocity)]
        applied = [min(cap, max(-cap, value)) for value, cap in zip(raw, self.caps)]
        return {'body_feedback_q_rad': position, 'body_feedback_dq_rad_s': velocity,
                'body_effort_unclipped_nm': raw, 'body_effort_nm': applied,
                'body_effort_saturated_names': [n for n, a, b in zip(self.names, raw, applied) if a != b]}

    def receipt(self):
        return {'controller_mode': 'source_contact_equilibrium_pd', 'body_joint_names': self.names,
            'body_nominal_rad': self.default, 'body_kp_nm_rad': self.kp, 'body_kd_nm_s_rad': self.kd,
            'body_source_caps_nm': self.caps, 'body_feedforward_nm': self.ff,
            'source_equilibrium_reference': self.reference,
            'source_equilibrium_canonical_sha256': hashlib.sha256(json.dumps(self.reference, sort_keys=True,
                separators=(',', ':'), allow_nan=False).encode()).hexdigest(),
            'controller_equation': 'clip(tau_source_equilibrium + kp*(q_nominal-q_measured) - kd*dq_measured, -source_cap, source_cap)',
            'learned_policy_inference': False, 'external_base_wrench_applied': False,
            'source_contact_forces_applied_to_simulator': False,
            'scope': 'Contact forces derive fixed joint feedforward only; actual free-body stance must pass measured gates'}

    def implicit_receipt(self, limits):
        if any(k <= 0 for k in self.kp):
            raise ValueError('An implicit feedforward target bias requires strictly positive body stiffness')
        targets = [home + ff/k for home, ff, k in zip(self.default, self.ff, self.kp)]
        if any(not limits[n]['lower'] <= target <= limits[n]['upper'] for n, target in zip(self.names, targets)):
            raise ValueError('Equivalent feedforward target would exceed an unchanged source joint limit')
        return self.receipt() | {'controller_mode': 'source_equilibrium_implicit_target_bias_v1',
            'body_implicit_target_rad': targets, 'body_target_bias_rad': [f/k for f, k in zip(self.ff, self.kp)],
            'controller_equation': 'q_drive=q_nominal+tau_equilibrium/kp; existing implicit drive applies capped kp*(q_drive-q)-kd*dq',
            'explicit_effort_command': False, 'implicit_drive_source_caps_unchanged': True,
            'integration_scope': 'PhysX implicit drive; algebraic constant-feedforward equivalence, no additive effort and no artificial inertia',
            'scope': 'Declared SIM drive representation; source nominal pose, gains, caps and masses unchanged; actual stance still requires measured gates'}


def source_adjacency(source):
    """Direct URDF parent-child links only; does not modify collision filters."""
    root = ET.parse(source).getroot()
    return {tuple(sorted((j.find('parent').get('link'), j.find('child').get('link'))))
            for j in root.findall('joint')}


def material_self_penetrations(contacts, adjacent_pairs):
    """Return materially loaded penetrating source-nonadjacent actor pairs.

    Sum impulse magnitudes only at points deeper than 1 mm. The 0.005 Ns
    per-pair/step threshold corresponds to 1 N over the fixed 5 ms step.
    Zero-impulse proximity and direct source adjacency are not violations.
    """
    pairs = {}
    for row in contacts:
        actors = [row['actor0'], row['actor1']]
        if not all(a.startswith('/World/G1/') for a in actors):
            continue
        names = tuple(sorted(a.removeprefix('/World/G1/') for a in actors))
        if names[0] == names[1] or names in adjacent_pairs:
            continue
        if not finite_tree(row) or len(row['impulse_ns']) != 3:
            raise ValueError('Invalid self-contact evidence')
        magnitude = math.sqrt(sum(float(x)**2 for x in row['impulse_ns']))
        penetration = max(0., -float(row['separation_m']))
        if penetration > GATES['maximum_loaded_nonadjacent_self_penetration_m']:
            item = pairs.setdefault(names, {'link_pair': list(names), 'penetrating_impulse_ns': 0.,
                                          'maximum_penetration_m': 0., 'point_count': 0})
            item['penetrating_impulse_ns'] += magnitude
            item['maximum_penetration_m'] = max(item['maximum_penetration_m'], penetration)
            item['point_count'] += 1
    return [value for value in pairs.values()
            if value['penetrating_impulse_ns'] > GATES['material_self_penetration_pair_impulse_ns']]


def finite_tree(value):
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(finite_tree(v) for v in value.values())
    if isinstance(value, (tuple, list)):
        return all(finite_tree(v) for v in value)
    return True


def safe_json(value):
    if hasattr(value, 'tolist'):
        value = value.tolist()
    if isinstance(value, float) and not math.isfinite(value):
        return {'invalid_numeric': repr(value)}
    if isinstance(value, dict):
        return {str(k): safe_json(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe_json(v) for v in value]
    return value


def source_foot_spheres(source):
    result = []
    for link in ET.parse(source).getroot().findall('link'):
        for collision in link.findall('collision'):
            sphere = collision.find('geometry/sphere')
            if sphere is None:
                continue
            if link.get('name') not in FEET:
                raise ValueError('Unexpected sphere outside source feet')
            origin = collision.find('origin')
            center = [float(x) for x in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            radius = float(sphere.get('radius'))
            if len(center) != 3 or not all(math.isfinite(x) for x in center + [radius]) or radius <= 0:
                raise ValueError('Invalid source foot sphere')
            result.append({'link': link.get('name'), 'center_link_m': center, 'radius_m': radius})
    if len(result) != 8 or any(sum(s['link'] == n for s in result) != 4 for n in FEET):
        raise ValueError('Expected exactly four original contact spheres per foot')
    return result


def initial_height(spheres, source_fk):
    import numpy as np
    bottoms = []
    for sphere in spheres:
        center = np.asarray(source_fk[sphere['link']]) @ np.asarray(sphere['center_link_m'] + [1.])
        bottoms.append(float(center[2]) - sphere['radius_m'])
    if not bottoms or not all(math.isfinite(v) for v in bottoms):
        raise ValueError('Cannot derive a finite source foot clearance')
    return GATES['initial_foot_clearance_m'] - min(bottoms)


def roll_pitch(pose):
    if len(pose) != 7 or not all(math.isfinite(float(v)) for v in pose):
        raise ValueError('Invalid measured pose')
    x, y, z, w = pose[3:]
    if abs(x*x+y*y+z*z+w*w-1) > 1e-4:
        raise ValueError('Nonunit measured orientation')
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))))


def ground_contacts(contacts):
    """Classify actual actor pairs; nearby zero-impulse reports are not loading."""
    feet = {n: 0. for n in FEET}
    other = []
    for row in contacts:
        impulse = row['impulse_ns']
        if len(impulse) != 3 or not finite_tree(row):
            raise ValueError('Malformed contact evidence')
        magnitude = math.sqrt(sum(float(v)**2 for v in impulse))
        actors = [row['actor0'], row['actor1']]
        if GROUND not in actors or magnitude <= GATES['loaded_contact_impulse_threshold_ns']:
            continue
        partner = actors[1] if actors[0] == GROUND else actors[0]
        name = partner.removeprefix('/World/G1/')
        if name in feet:
            feet[name] += magnitude
        else:
            other.append(row)
    return feet, other


def evaluate(rows, initial, limits, mimics, *, supporting_constraints, integrity_checks, abort=None,
             controller_mode='policy', controller_reference=None, adjacent_pairs=()):
    """Evaluate contiguous raw measured states, never commanded poses."""
    checks = {'full_10_second_duration': len(rows) == GATES['required_steps'],
              'finite_complete_measured_state': bool(rows), 'contiguous_physics_sequence': bool(rows),
              'single_zero_command_body_owner': bool(rows), 'free_dynamic_without_supports': supporting_constraints == [],
              'minimum_pelvis_height': bool(rows), 'roll_pitch_within_gate': bool(rows),
              'pelvis_xy_drift_within_gate': bool(rows), 'each_foot_xy_drift_within_gate': bool(rows),
              'each_foot_height_rise_within_gate': bool(rows), 'no_loaded_nonfoot_ground_contact': bool(rows),
              'no_material_loaded_nonadjacent_self_penetration': bool(rows),
              'both_feet_have_measured_loaded_contact': False, 'source_joint_limits_within_gate': bool(rows),
              'hand_coupling_within_gate': bool(rows), 'no_abort': abort is None}
    peak = {'pelvis_xy_drift_m': 0., 'abs_roll_pitch_rad': 0., 'joint_limit_violation_rad': 0.,
            'coupling_error_rad': 0., 'foot_xy_drift_m': {n: 0. for n in FEET},
            'foot_height_rise_m': {n: 0. for n in FEET}}
    min_height = None
    contact_steps = {n: 0 for n in FEET}
    ground_impulses = {n: 0. for n in FEET}
    unsupported_contact_count = 0
    self_penetrations = []
    if controller_mode not in ('policy', 'source_contact_equilibrium_pd', 'source_equilibrium_implicit_target_bias_v1'):
        raise ValueError('Unknown evaluated controller mode')
    if controller_mode == 'source_contact_equilibrium_pd':
        checks['source_equilibrium_effort_matches_capped_pd'] = bool(rows)
    elif controller_mode == 'source_equilibrium_implicit_target_bias_v1':
        checks['source_equilibrium_implicit_target_bias_verified'] = bool(rows)
    try:
        if not finite_tree(initial):
            raise ValueError('Nonfinite initial reference')
        for n in ('pelvis',) + FEET:
            roll_pitch(initial['link_poses_world_xyzw'][n])
        for seq, row in enumerate(rows):
            if not finite_tree(row):
                raise ValueError('Nonfinite raw evidence')
            names = row['runtime_names']
            if len(names) != 53 or len(set(names)) != 53 or set(names) != set(limits):
                raise ValueError('Incomplete named articulation')
            q, dq = row['q_rad'], row['dq_rad_s']
            if len(q) != 53 or len(dq) != 53 or len(row['measured_generalized_effort_nm']) != 53:
                raise ValueError('Incomplete measured coordinates')
            if controller_mode == 'policy' and (len(row['policy_observation']) != 480 or len(row['policy_action']) != 29):
                raise ValueError('Missing actual policy computation')
            if row['sequence'] != seq or abs(row['physics_s']-initial['physics_s']-(seq+1)*.005) > 1e-7:
                checks['contiguous_physics_sequence'] = False
            owner = {'policy': 'named_policy_single_writer',
                     'source_contact_equilibrium_pd': 'source_equilibrium_effort_single_writer',
                     'source_equilibrium_implicit_target_bias_v1': 'source_equilibrium_implicit_single_writer'}[controller_mode]
            if (row['command_velocity'] != [0., 0., 0.] or row['body_command_owner'] != owner
                    or len(row['body_command_names']) != 29 or len(set(row['body_command_names'])) != 29
                    or not set(row['body_command_names']).issubset(names) or len(row['body_command_rad']) != 29):
                checks['single_zero_command_body_owner'] = False
            if controller_mode == 'source_contact_equilibrium_pd':
                reference = controller_reference
                if (not isinstance(reference, dict) or row.get('controller_mode') != controller_mode
                        or row.get('body_effort_writes_this_step') != 1 or row.get('body_implicit_position_writes_this_step') != 0
                        or row['body_command_names'] != reference['body_joint_names']
                        or row['body_command_rad'] != reference['body_nominal_rad']):
                    checks['single_zero_command_body_owner'] = False
                    raise ValueError('Missing deterministic controller ownership/reference')
                vectors = [row[key] for key in ('body_feedback_q_rad', 'body_feedback_dq_rad_s',
                                               'body_effort_unclipped_nm', 'body_effort_nm')]
                if any(len(v) != 29 for v in vectors):
                    raise ValueError('Incomplete named29 effort/feedback export')
                if (row['body_feedback_source_sequence'] != seq - 1
                        or abs(row['body_feedback_physics_s'] - (row['physics_s'] - GATES['physics_dt_s'])) > 1e-7):
                    raise ValueError('Effort feedback is not from the immediately preceding state')
                previous = initial if seq == 0 else rows[seq-1]
                previous_indices = [previous['runtime_names'].index(n) for n in row['body_command_names']]
                previous_feedback = ([previous['q_rad'][i] for i in previous_indices]
                                     + [previous['dq_rad_s'][i] for i in previous_indices])
                if any(abs(a-b) > 1e-7 for a, b in zip(vectors[0] + vectors[1], previous_feedback)):
                    raise ValueError('Exported effort feedback differs from retained measured state')
                raw = [f+k*(home-p)-d*v for f, k, home, p, d, v in zip(reference['body_feedforward_nm'],
                    reference['body_kp_nm_rad'], reference['body_nominal_rad'], vectors[0], reference['body_kd_nm_s_rad'], vectors[1])]
                expected = [min(cap, max(-cap, t)) for t, cap in zip(raw, reference['body_source_caps_nm'])]
                if any(abs(a-b) > 1e-5 for a, b in zip(vectors[2] + vectors[3], raw + expected)):
                    checks['source_equilibrium_effort_matches_capped_pd'] = False
                live_effort = row['applied_generalized_actuation_effort_nm']
                if len(live_effort) != 53:
                    raise ValueError('Missing complete applied-effort backend readback')
                effort_by_name = dict(zip(row['body_command_names'], vectors[3]))
                if any(abs(value-effort_by_name.get(name, 0.)) > 1e-7 for name, value in zip(names, live_effort)):
                    checks['source_equilibrium_effort_matches_capped_pd'] = False
            elif controller_mode == 'source_equilibrium_implicit_target_bias_v1':
                reference = controller_reference
                if (not isinstance(reference, dict) or row.get('controller_mode') != controller_mode
                        or row.get('body_effort_writes_this_step') != 0 or row.get('body_implicit_position_writes_this_step') != 1
                        or row['body_command_names'] != reference['body_joint_names']):
                    checks['single_zero_command_body_owner'] = False
                    raise ValueError('Missing implicit equilibrium controller ownership/reference')
                nominal, ff, kp = (reference[key] for key in ('body_nominal_rad', 'body_feedforward_nm', 'body_kp_nm_rad'))
                if any(len(v) != 29 for v in (nominal, ff, kp)) or any(k <= 0 for k in kp):
                    raise ValueError('Invalid implicit equilibrium reference')
                target = [home+force/k for home, force, k in zip(nominal, ff, kp)]
                readback = row['body_implicit_target_backend_rad']
                effort = row['applied_generalized_actuation_effort_nm']
                if len(readback) != 29 or len(effort) != 53:
                    raise ValueError('Missing implicit target or explicit force backend evidence')
                if (any(abs(a-b) > 1e-7 for a, b in zip(row['body_command_rad'] + readback, target + target))
                        or any(value != 0. for value in effort)
                        or any(not limits[n]['lower'] <= value <= limits[n]['upper'] for n, value in zip(row['body_command_names'], target))):
                    checks['source_equilibrium_implicit_target_bias_verified'] = False
            pose = row['link_poses_world_xyzw']['pelvis']
            roll, pitch = roll_pitch(pose)
            min_height = pose[2] if min_height is None else min(min_height, pose[2])
            peak['abs_roll_pitch_rad'] = max(peak['abs_roll_pitch_rad'], abs(roll), abs(pitch))
            ref = initial['link_poses_world_xyzw']['pelvis']
            peak['pelvis_xy_drift_m'] = max(peak['pelvis_xy_drift_m'], math.hypot(pose[0]-ref[0], pose[1]-ref[1]))
            for n in FEET:
                p = row['link_poses_world_xyzw'][n]; roll_pitch(p)
                ref = initial['link_poses_world_xyzw'][n]
                peak['foot_xy_drift_m'][n] = max(peak['foot_xy_drift_m'][n], math.hypot(p[0]-ref[0], p[1]-ref[1]))
                peak['foot_height_rise_m'][n] = max(peak['foot_height_rise_m'][n], p[2]-ref[2])
            positions = dict(zip(names, q))
            for n, lim in limits.items():
                peak['joint_limit_violation_rad'] = max(peak['joint_limit_violation_rad'], lim['lower']-positions[n], positions[n]-lim['upper'])
            for n, m in mimics.items():
                peak['coupling_error_rad'] = max(peak['coupling_error_rad'], abs(positions[n]-positions[m['parent']]*m['multiplier']-m['offset']))
            feet, other = ground_contacts(row['contacts'])
            if any(c['sequence'] != seq or abs(c['physics_s']-row['physics_s']) > 1e-7 for c in row['contacts']):
                raise ValueError('Contact/state timestamps differ')
            unsupported_contact_count += len(other)
            self_penetrations.extend(dict(value, sequence=seq, physics_s=row['physics_s'])
                                     for value in material_self_penetrations(row['contacts'], adjacent_pairs))
            for n in FEET:
                contact_steps[n] += int(feet[n] > 0)
                ground_impulses[n] += feet[n]
    except (KeyError, TypeError, ValueError, IndexError, OverflowError) as exc:
        checks['finite_complete_measured_state'] = False
        evidence_error = str(exc)
    else:
        evidence_error = None
    checks.update({'minimum_pelvis_height': min_height is not None and min_height >= .65,
                   'roll_pitch_within_gate': peak['abs_roll_pitch_rad'] <= .2,
                   'pelvis_xy_drift_within_gate': peak['pelvis_xy_drift_m'] <= .1,
                   'each_foot_xy_drift_within_gate': all(v <= .03 for v in peak['foot_xy_drift_m'].values()),
                   'each_foot_height_rise_within_gate': all(v <= .03 for v in peak['foot_height_rise_m'].values()),
                   'source_joint_limits_within_gate': peak['joint_limit_violation_rad'] <= .03,
                   'hand_coupling_within_gate': peak['coupling_error_rad'] <= .03,
                   'no_loaded_nonfoot_ground_contact': unsupported_contact_count == 0,
                   'no_material_loaded_nonadjacent_self_penetration': not self_penetrations,
                   'both_feet_have_measured_loaded_contact': all(v > 0 for v in contact_steps.values())})
    for key, value in integrity_checks.items():
        if type(value) is not bool or key in checks:
            raise ValueError('Invalid independent integrity check')
        checks[key] = value
    return {'status': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks, 'steps': len(rows),
            'minimum_pelvis_height_m': min_height, 'peaks': peak, 'foot_loaded_contact_steps': contact_steps,
            'foot_ground_impulse_sum_ns': ground_impulses, 'loaded_nonfoot_ground_contacts': unsupported_contact_count,
            'controller_mode': controller_mode, 'material_loaded_nonadjacent_self_penetrations': self_penetrations,
            'evidence_error': evidence_error, 'abort': abort}
