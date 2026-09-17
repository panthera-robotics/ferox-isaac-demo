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

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'isaac' / 'twin'))
from inspire.embodiment import dependency_values_from_urdf  # noqa: E402

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


def fixed_joint_origin(root, parent, child):
    for j in root.findall('joint'):
        if j.get('type') == 'fixed' and j.find('parent').get('link') == parent and j.find('child').get('link') == child:
            o = j.find('origin')
            return {'parent_link': parent, 'child_link': child, 'xyz_m': [float(v) for v in (o.get('xyz') or '0 0 0').split()], 'rpy_rad': [float(v) for v in (o.get('rpy') or '0 0 0').split()]}
    raise SystemExit('no fixed joint %s -> %s' % (parent, child))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--urdf', type=Path, required=True); ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--manifest-id', default='g1_edu29_rh56dftp_donor_v1')
    ap.add_argument('--grasp-v12-sha256', default='eef219de8f088b7953b773ba513aa41b9158a830bf0cb47de9054e670d1bfa1e', help='sha256 of the retention-qualified grasp file v12 (also the retention probe config)')
    ap.add_argument('--grasp-v13-sha256', default='e80befb00b34cae04d2ab4ba8b750b41d94138689c5534b24831a94f9e31ce88')
    ap.add_argument('--acquisition-config-sha256', default='7052810259bd935a0477c48c6481b90b75e7b239fe097cce27b758e3a7b38c9c', help='sha256 of the rack-acquisition probe config (kd 0.5 regression)')
    ap.add_argument('--hand-image', default='sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54')
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
    collision = 'right=ftp_palm_yz_slabs_v2;left=ftp_left_palm_yz_slabs_v1;contact_offset_m=0.0012860533315688372;rest_offset_m=0'
    live = dependency_values_from_urdf(a.urdf, collision_cooking=collision)
    urdf_sha, wrist_mount_sha, camera_mount_sha = live['urdf_sha256'], live['wrist_mount_sha256'], live['camera_mount_sha256']
    base = {k: live[k] for k in ('urdf_sha256', 'coupling_map_sha256', 'collision_cooking', 'wrist_mount_sha256', 'physics_dt_s', 'solver')}
    claims = {
        'mechanism_checks': {'status': 'PASS', 'evidence': 'assembled-body 13/13, moving wrist 37/37 (session records)', 'configuration': dict(base, support='FIXED_PELVIS')},
        'retention_60s_grasp_v12': {'status': 'PASS', 'evidence': 'evidence/sF-grasp-v12-retention60-01 (tip 0.102 mm, axis 0.042 deg)',
                                    'configuration': dict(base, grasp_sha256=a.grasp_v12_sha256, hand_drive_gains='kp=1.0;kd=0.5;thumb_kp=inherited', support='DRIVEN_WRIST', source_image=a.hand_image)},
        'retention_60s_grasp_v13': {'status': 'PASS', 'evidence': 'evidence/sF-grasp-v13-retention60-01 (tip 0.094 mm, axis 0.038 deg)',
                                    'configuration': dict(base, grasp_sha256=a.grasp_v13_sha256, hand_drive_gains='kp=2.0;kd=0.5', support='DRIVEN_WRIST', source_image=a.hand_image)},
        'acquisition_cycles_kd05': {'status': 'PASS', 'evidence': 'evidence/sF-rack-acquire-cycles-kd05-01 (3/3 cycles)',
                                    'configuration': dict(base, probe_config_sha256=a.acquisition_config_sha256, support='DRIVEN_WRIST', source_image=a.hand_image)},
        'contact_writing': {'status': 'EXECUTED_NOT_QUALIFIED', 'evidence': 'evidence/sF-writer-contact-16-v9e-G1OK55 (p95 3.53 mm, coverage 70 %)',
                            'configuration': dict(base, grasp_sha256=a.grasp_v13_sha256, tool_frame='measured in sF-writer-contact-15-v13-measure (simulator, invalidated by grasp/mount/asset change)', support='FIXED_PELVIS')},
        'command_replay_integration': {'status': 'PASS', 'evidence': 'evidence/sG-replay-pickup-ep0-03 (121/121 rows)',
                                       'configuration': dict(base, camera_mount_sha256=camera_mount_sha, support='FIXED_PELVIS', controller='implicit_biased_drive_v1 replay controller (package-hashed gains)')},
        'standing': {'status': 'NOT_QUALIFIED', 'evidence': 'evidence/takeover-balance-implicit-03 (topples)', 'configuration': dict(base, support='NONE')},
        'real_data_agreement': {'status': 'NOT_RUN', 'evidence': None, 'configuration': None},
        'learned_policy_evaluation': {'status': 'NOT_RUN', 'evidence': None, 'configuration': None},
    }
    manifest = {
        'schema_version': 1, 'manifest_id': a.manifest_id,
        'hardware_identity': {'robot': 'Unitree G1 EDU 29-DoF', 'right_hand': 'Inspire RH56E2-2R-T1 (nameplate photo, owner)', 'left_hand': 'Inspire RH56E2-2L-T1 (context-reported, unverified)',
                              'identity_evidence': 'owner nameplate photo (private, not in this repository); left hand not photographed'},
        'source_asset': {'asset_id': 'unitree_ftp_g1_29dof_rev_1_0_with_inspire_hand_FTP', 'kind': 'provisional_donor', 'exact_hand_model': False,
                         'urdf_sha256': hashlib.sha256(a.urdf.read_bytes()).hexdigest(), 'urdf_name': a.urdf.name,
                         'collision': 'declared provisional palm/thumb colliders (ftp_palm_yz_slabs_v2 right, ftp_left_palm_yz_slabs_v1 left)'},
        'qualification': {'exact_asset_qualified': False, 'installed_hand_similarity_percent': None, 'claims': claims,
                          'rule': 'historical status stays attached to its bound configuration; active compatibility is ACTIVE_COMPATIBLE only when every bound dependency is present and identical (check_validity), STALE when any differs, UNVERIFIED when any is missing; a hash proves identity, not physical correctness'},
        'body': {'joint_names': list(BODY_ORDER), 'limits_rad': {n: limits[n] for n in BODY_ORDER},
                 'order_provenance': 'Unitree G1 29-DoF SDK joint index order; the twin maps by name (articulation indices are never assumed)',
                 'units': {'position': 'rad', 'velocity': 'rad/s', 'effort': 'N*m'}, 'sign': 'URDF axis sign',
                 'floating_base': {'state_fields': ['root_position_xyz_m', 'root_orientation_wxyz'], 'support': 'FIXED_PELVIS in every replay of this version (pelvis fixed to world; root fields ignored and reported)'}},
        'hands': hands,
        'transforms': {'right': {'wrist_to_hand': {'matrix_4x4': fixed_joint_matrix(root, 'right_wrist_yaw_link', 'right_base_link'), 'frame': 'right_wrist_yaw_link -> right_base_link', 'provenance': 'donor URDF fixed joint (not measured on hardware)',
                                                   'valid_for': {'urdf_sha256': urdf_sha, 'wrist_mount_sha256': wrist_mount_sha}, 'dependent_qualifications': ['contact_writing']},
                                 'hand_to_tool': None},
                       'left': {'wrist_to_hand': {'matrix_4x4': fixed_joint_matrix(root, 'left_wrist_yaw_link', 'left_base_link'), 'frame': 'left_wrist_yaw_link -> left_base_link', 'provenance': 'donor URDF fixed joint (not measured on hardware)',
                                                  'valid_for': {'urdf_sha256': urdf_sha}, 'dependent_qualifications': []},
                                'hand_to_tool': None}},
        'controller': {'type': 'implicit_biased_drive_v1 (PhysX capped position drives) for body targets; implicit position drives for the six hand actuators; coupled joints follow mimic constraints',
                       'gains_provenance': 'declared per replay package (hashed); NOT hardware firmware gains', 'rate_hz': 200.0,
                       'latency_assumption': 'zero-order hold of the source command at simulator time; no transport latency modelled',
                       'clock_domains': {'physics': 'simulator physics time (5 ms steps)', 'wall': 'host monotonic (watchdogs only)', 'source': 'recording timestamps mapped to physics time from the first row'}},
        'cameras': {'policy_head_d435_color_nominal': {'mount_frame': 'd435_link (torso_link fixed joint from the donor URDF)', 'mount': fixed_joint_origin(root, 'torso_link', 'd435_link'), 'image_format': 'RGB8 640x480',
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
