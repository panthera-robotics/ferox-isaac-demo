"""Admission and measured-state gates for the provisional standing diagnostic.

No Isaac, policy, or private calibration import is needed to evaluate evidence.
"""
from dataclasses import asdict, dataclass
import math
from pathlib import PurePosixPath
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class BalanceConfig:
    locomotion_path: str = '/workspace/locomotion'
    policy_path: str = '/policy'
    ground_static_friction: float = 1.0
    ground_dynamic_friction: float = 1.0

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
        return obj


GATES = {'physics_dt_s': .005, 'required_steps': 2000, 'duration_s': 10.,
         'initial_foot_clearance_m': .002, 'minimum_pelvis_height_m': .65,
         'maximum_abs_roll_pitch_rad': .2, 'maximum_pelvis_xy_drift_m': .1,
         'maximum_each_foot_xy_drift_m': .03, 'maximum_each_foot_height_rise_m': .03,
         'maximum_joint_limit_violation_rad': .03, 'maximum_coupling_error_rad': .03,
         'loaded_contact_impulse_threshold_ns': 1e-9}
FEET = ('left_ankle_roll_link', 'right_ankle_roll_link')
GROUND = '/World/Ground'


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


def evaluate(rows, initial, limits, mimics, *, supporting_constraints, integrity_checks, abort=None):
    """Evaluate contiguous raw measured states, never commanded poses."""
    checks = {'full_10_second_duration': len(rows) == GATES['required_steps'],
              'finite_complete_measured_state': bool(rows), 'contiguous_physics_sequence': bool(rows),
              'single_zero_command_body_owner': bool(rows), 'free_dynamic_without_supports': supporting_constraints == [],
              'minimum_pelvis_height': bool(rows), 'roll_pitch_within_gate': bool(rows),
              'pelvis_xy_drift_within_gate': bool(rows), 'each_foot_xy_drift_within_gate': bool(rows),
              'each_foot_height_rise_within_gate': bool(rows), 'no_loaded_nonfoot_ground_contact': bool(rows),
              'both_feet_have_measured_loaded_contact': False, 'source_joint_limits_within_gate': bool(rows),
              'hand_coupling_within_gate': bool(rows), 'no_abort': abort is None}
    peak = {'pelvis_xy_drift_m': 0., 'abs_roll_pitch_rad': 0., 'joint_limit_violation_rad': 0.,
            'coupling_error_rad': 0., 'foot_xy_drift_m': {n: 0. for n in FEET},
            'foot_height_rise_m': {n: 0. for n in FEET}}
    min_height = None
    contact_steps = {n: 0 for n in FEET}
    ground_impulses = {n: 0. for n in FEET}
    unsupported_contact_count = 0
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
            if len(row['policy_observation']) != 480 or len(row['policy_action']) != 29:
                raise ValueError('Missing actual policy computation')
            if row['sequence'] != seq or abs(row['physics_s']-initial['physics_s']-(seq+1)*.005) > 1e-7:
                checks['contiguous_physics_sequence'] = False
            if (row['command_velocity'] != [0., 0., 0.] or row['body_command_owner'] != 'named_policy_single_writer'
                    or len(row['body_command_names']) != 29 or len(set(row['body_command_names'])) != 29
                    or not set(row['body_command_names']).issubset(names) or len(row['body_command_rad']) != 29):
                checks['single_zero_command_body_owner'] = False
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
                   'both_feet_have_measured_loaded_contact': all(v > 0 for v in contact_steps.values())})
    for key, value in integrity_checks.items():
        if type(value) is not bool or key in checks:
            raise ValueError('Invalid independent integrity check')
        checks[key] = value
    return {'status': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks, 'steps': len(rows),
            'minimum_pelvis_height_m': min_height, 'peaks': peak, 'foot_loaded_contact_steps': contact_steps,
            'foot_ground_impulse_sum_ns': ground_impulses, 'loaded_nonfoot_ground_contacts': unsupported_contact_count,
            'evidence_error': evidence_error, 'abort': abort}
