#!/usr/bin/env python3
"""Generate the versioned donor embodiment manifest from the pinned source URDF (CPU, no Isaac).

Joint limits, the six independent hand actuators per side and the mimic-coupled joints are read from
the URDF; identity, qualification, controller, camera and tactile descriptors are declared here with
their provenance. Unknown real-hand parameters are written as null. Usage:
  build_embodiment_manifest.py --urdf <donor.urdf> --out isaac/twin/inspire/embodiments/<id>.json
"""
import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from urdf_kinematics import UrdfKinematics  # noqa: F401  (keeps the public kinematics module the single URDF reader)

BODY_ORDER = ('left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
              'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
              'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
              'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
              'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint')
# Donor joint per contract actuator (thumb_1 = rotation/lateral tilt, thumb_2 = bend), mapped by name.
ACTUATOR_JOINT = {'index': '{side}_index_1_joint', 'middle': '{side}_middle_1_joint', 'ring': '{side}_ring_1_joint', 'little': '{side}_little_1_joint',
                  'thumb_bend': '{side}_thumb_2_joint', 'thumb_rotation': '{side}_thumb_1_joint'}


def fixed_joint_matrix(root, parent, child):
    """4x4 transform of the URDF fixed joint parent->child (xyz + rpy, URDF convention Rz*Ry*Rx)."""
    for j in root.findall('joint'):
        if j.get('type') == 'fixed' and j.find('parent').get('link') == parent and j.find('child').get('link') == child:
            o = j.find('origin'); xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()]; r, p_, y = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
            cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p_), math.sin(p_), math.cos(y), math.sin(y)
            R = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]]
            return [[*R[0], xyz[0]], [*R[1], xyz[1]], [*R[2], xyz[2]], [0.0, 0.0, 0.0, 1.0]]
    raise SystemExit('no fixed joint %s -> %s' % (parent, child))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--urdf', type=Path, required=True); ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--manifest-id', default='g1_edu29_rh56dftp_donor_v1')
    a = ap.parse_args(argv)
    root = ET.parse(a.urdf).getroot()
    joints = {j.get('name'): j for j in root.findall('joint') if j.get('type') in ('revolute', 'prismatic')}
    limits = {n: [float(j.find('limit').get('lower')), float(j.find('limit').get('upper'))] for n, j in joints.items()}
    missing = [n for n in BODY_ORDER if n not in joints]
    if missing:
        raise SystemExit('URDF lacks body joints: %s' % missing)
    hands = {}
    for side in ('left', 'right'):
        actuators = {}
        for act, pattern in ACTUATOR_JOINT.items():
            jn = pattern.format(side=side)
            if joints[jn].find('mimic') is not None:
                raise SystemExit('%s is coupled in the URDF; it cannot be an independent actuator' % jn)
            lo, hi = limits[jn]
            # Donor convention: URDF zero = open hand (grasp bench initial state), upper limit = fully closed.
            actuators[act] = {'joint': jn, 'open_rad': lo, 'closed_rad': hi, 'limit_rad': [lo, hi],
                              'endpoint_provenance': 'donor URDF joint limits; open = URDF zero pose (bench convention), closed = upper limit; NOT an E2 angle datum'}
        coupled = {n: {'parent': j.find('mimic').get('joint'), 'multiplier': float(j.find('mimic').get('multiplier', 1)), 'offset': float(j.find('mimic').get('offset', 0)),
                       'limit_rad': limits[n]} for n, j in joints.items() if n.startswith(side + '_') and j.find('mimic') is not None}
        hands[side] = {
            'model_nominal': 'Inspire RH56E2-2%s-T1 (owner context; right confirmed by nameplate photo, left context-reported)' % side[0].upper(),
            'source_joints': 'RH56DFTP donor (provisional)', 'actuators': actuators, 'coupled_joints': coupled,
            'native_interface': {'axis_order_owner_context': ['little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation'],
                                 'scale_owner_context': '0..1000 counts, 1000 = open', 'status': 'UNVERIFIED_PROJECT_SOURCE_STARTING_POINT',
                                 'note': 'verify per recording/interface/firmware revision before use; datasets declare their own order and scale in their source contract'},
            'feedback': {'independent_axes_measured': True, 'coupled_joints_measured': False,
                         'note': 'simulator returns all 12 joint positions; the real hand reports six native angle readbacks (counts), not twelve measured joint angles'}}
    manifest = {
        'schema_version': 1, 'manifest_id': a.manifest_id,
        'hardware_identity': {'robot': 'Unitree G1 EDU 29-DoF', 'right_hand': 'Inspire RH56E2-2R-T1 (nameplate photo, owner)', 'left_hand': 'Inspire RH56E2-2L-T1 (context-reported, unverified)',
                              'identity_evidence': 'owner nameplate photo (private, not in this repository); left hand not photographed'},
        'source_asset': {'asset_id': 'unitree_ftp_g1_29dof_rev_1_0_with_inspire_hand_FTP', 'kind': 'provisional_donor', 'exact_hand_model': False,
                         'urdf_sha256': hashlib.sha256(a.urdf.read_bytes()).hexdigest(), 'urdf_name': a.urdf.name,
                         'collision': 'declared provisional palm/thumb colliders (ftp_palm_yz_slabs_v2 right, ftp_left_palm_yz_slabs_v1 left)'},
        'qualification': {'exact_asset_qualified': False, 'mechanism_checks': 'assembled-body 13/13, moving wrist 37/37 (session records)',
                          'retention_60s': 'PASS on driven-wrist fixture with grasp v12/v13 (provisional donor)', 'acquisition_cycles': 'PASS 3/3 driven-wrist fixture',
                          'contact_writing': 'EXECUTED_NOT_QUALIFIED', 'standing': 'NOT_QUALIFIED', 'installed_hand_similarity_percent': None},
        'body': {'joint_names': list(BODY_ORDER), 'limits_rad': {n: limits[n] for n in BODY_ORDER},
                 'order_provenance': 'Unitree G1 29-DoF SDK joint index order; the twin maps by name (articulation indices are never assumed)',
                 'units': {'position': 'rad', 'velocity': 'rad/s', 'effort': 'N*m'}, 'sign': 'URDF axis sign',
                 'floating_base': {'state_fields': ['root_position_xyz_m', 'root_orientation_wxyz'], 'support': 'FIXED_PELVIS in every replay of this version (pelvis fixed to world; root fields ignored and reported)'}},
        'hands': hands,
        'transforms': {'right': {'wrist_to_hand': {'matrix_4x4': fixed_joint_matrix(root, 'right_wrist_yaw_link', 'right_base_link'), 'frame': 'right_wrist_yaw_link -> right_base_link', 'provenance': 'donor URDF fixed joint (not measured on hardware)',
                                                   'valid_for': {'urdf_sha256': hashlib.sha256(a.urdf.read_bytes()).hexdigest()}, 'dependent_qualifications': ['contact_writing']},
                                 'hand_to_tool': None},
                       'left': {'wrist_to_hand': {'matrix_4x4': fixed_joint_matrix(root, 'left_wrist_yaw_link', 'left_base_link'), 'frame': 'left_wrist_yaw_link -> left_base_link', 'provenance': 'donor URDF fixed joint (not measured on hardware)',
                                                  'valid_for': {'urdf_sha256': hashlib.sha256(a.urdf.read_bytes()).hexdigest()}, 'dependent_qualifications': []},
                                'hand_to_tool': None}},
        'controller': {'type': 'implicit_biased_drive_v1 (PhysX capped position drives) for body targets; implicit position drives for the six hand actuators; coupled joints follow mimic constraints',
                       'gains_provenance': 'declared per replay package (hashed); NOT hardware firmware gains', 'rate_hz': 200.0,
                       'latency_assumption': 'zero-order hold of the source command at simulator time; no transport latency modelled',
                       'clock_domains': {'physics': 'simulator physics time (5 ms steps)', 'wall': 'host monotonic (watchdogs only)', 'source': 'recording timestamps mapped to physics time from the first row'}},
        'cameras': {'policy_head_d435_color_nominal': {'mount_frame': 'd435_link (torso_link fixed joint from the donor URDF)', 'image_format': 'RGB8 640x480',
                                                       'calibration': {'status': 'NOMINAL_NOT_MEASURED', 'horizontal_fov_deg': 69.0, 'source': 'Intel D435 colour datasheet nominal'},
                                                       'timestamp_domain': 'physics time of the rendered step'},
                    'external_front': {'mount_frame': 'world', 'image_format': 'RGBA8 640x640', 'calibration': {'status': 'simulator_only'}, 'timestamp_domain': 'physics time'},
                    'external_side': {'mount_frame': 'world', 'image_format': 'RGBA8 640x640', 'calibration': {'status': 'simulator_only'}, 'timestamp_domain': 'physics time'}},
        'tactile_force': {'contact_normal_force': {'availability': 'simulator_only', 'units': 'N', 'note': 'PhysX contact impulses / dt; no real tactile array on the donor'},
                          'tactile_arrays_e2_t1': {'availability': 'unavailable', 'units': 'counts 0..4095 (manual)', 'note': 'E2 T1 arrays are not modelled; the donor carries force-sensor frames only'},
                          'fingertip_force_e2': {'availability': 'unavailable', 'units': 'N', 'note': 'nominal 28 N fingertip / 30 N thumb (product page); no measured conversion'}}}
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'manifest': str(a.out), 'sha256': hashlib.sha256(a.out.read_bytes()).hexdigest(), 'body_joints': len(BODY_ORDER),
                      'hand_actuators': {s: list(h['actuators']) for s, h in hands.items()}, 'coupled': {s: list(h['coupled_joints']) for s, h in hands.items()}}))


if __name__ == '__main__':
    main()
