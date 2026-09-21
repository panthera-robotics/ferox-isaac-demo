#!/usr/bin/env python3
"""Write the ferox `isaac.twin.inspire.embodiment` manifest for the PUBLIC exact-E2 prior asset (g1_edu29_rh56e2_e2prior_v1),
derived from the donor manifest (body, cameras, controller carried) with the E2 hands from e2_adapter and the asset manifest hashes.
Label PUBLIC_E2_PRIOR__INSTALLED_CALIBRATION_INCOMPLETE; every qualification claim starts NOT_RUN; the donor manifest is not modified.
usage: e2_embodiment_manifest.py --asset-dir <generated/e2prior> --donor-manifest isaac/twin/inspire/embodiments/g1_edu29_rh56dftp_donor_v1.json --audit <audit.json> --out <manifest.json>
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

try:
    from . import e2_adapter as ad
except ImportError:
    import e2_adapter as ad

LABEL = 'PUBLIC_E2_PRIOR__INSTALLED_CALIBRATION_INCOMPLETE'


def rpy_matrix(xyz, rpy):
    r, p, y = rpy; cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    R = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]]
    return [[R[i][0], R[i][1], R[i][2], xyz[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def build(asset_manifest, donor, audit=None):
    m = copy.deepcopy(donor); a = asset_manifest
    m['manifest_id'] = 'g1_edu29_rh56e2_e2prior_v1'; m['label'] = LABEL
    m['hardware_identity'] = {**donor['hardware_identity'], 'asset_relation': 'exact model family (RH56E2) from the public vendor-partner description; installed unit NOT measured'}
    m['source_asset'] = {'asset_id': a['asset_id'], 'kind': 'public_exact_model_prior', 'exact_hand_model': True, 'installed_calibration': 'INCOMPLETE', 'urdf_sha256': a['merged_urdf']['sha256'], 'urdf_name': a['merged_urdf']['name'],
                         'public_source': {'url': a['source']['url'], 'commit': a['source']['commit']}, 'collision': 'public STL meshes as collision geometry (64 files, LFS-verified); no provisional slab colliders',
                         'collision_model': ({'kind': 'e2_r4_folded_slabs', 'cooking': 'e2_r4:fixed_hand_links_folded;palm=e2_palm_yz_slabs_v1(4mm_xz_exact_meshes);fingers=convexDecomposition(importer);offsets=importer_default;baked_into_urdf'} if 'representation_r4' in a
                                             else {'kind': 'public_stl_meshes', 'cooking': 'public_stl_meshes;approximation=convexDecomposition(importer);offsets=importer_default;no_slab_substitution'}),
                         'corrections_applied': ['left main-link inertials = right mirrored across the hand-base y=0 plane', 'left_pinky_intermediate mass 0.01869 -> 0.01166 kg (right value)', 'drive limits: declared donor policy effort 10 / velocity 1.0 on all 24 hand joints (public placeholders recorded)']
                                               + (['r4 representation: fixed hand links (palm_1/palm_2/palm_force_sensor, 16 sensor pads, tcp) folded into their parent bodies (inertials composed exactly, frames kept), palm collider e2_palm_yz_slabs_v1 (coord-dec-R1-02)'] if 'representation_r4' in a else []),
                         'mass_policy': a['mass_policy']['primary'], 'hand_total_mass_kg': a['mass_policy']['hand_total_kg'], 'donor_status': 'DONOR_BASELINE_FROZEN (manifest g1_edu29_rh56dftp_donor_v1 unchanged)'}
    claims = {k: {'status': 'NOT_RUN', 'evidence': None, 'configuration': None} for k in donor['qualification']['claims']}
    claims['static_asset_audit'] = {'status': 'PASS' if audit and audit.get('verdict') == 'STATIC_AUDIT_PASS' else 'NOT_RUN', 'evidence': ('E2_PRIOR_ASSET_AUDIT (hand-req-Q02): %s' % json.dumps(audit['summary'], sort_keys=True)) if audit else None,
                                    'configuration': {'urdf_sha256': a['merged_urdf']['sha256']} if audit else None}
    m['qualification'] = {'exact_asset_qualified': False, 'installed_hand_similarity_percent': None, 'claims': claims, 'rule': donor['qualification']['rule'],
                          'label': LABEL, 'note': 'geometry/kinematics/inertials are public priors of the exact model family; installed calibration (hand-req-Q01) is INCOMPLETE; no physics claim is inherited from the donor'}
    hands = {}
    for side in ad.SIDES:
        acts = {}
        for axis in ad.CONTRACT_ORDER:
            lo, hi = ad.E2_LIMITS[axis]
            acts[axis] = {'joint': ad.e2_joint(side, axis), 'open_rad': lo, 'closed_rad': hi, 'limit_rad': [lo, hi], 'endpoint_provenance': 'public E2 URDF joint limits (EXACT_PUBLIC_SOURCE); open = lower = URDF zero pose, closed = upper; NOT an installed datum'}
        hands[side] = {'model_nominal': donor['hands'][side]['model_nominal'], 'source_joints': 'public exact-E2 description renesas-rdk/inspire_rh56e2_hand @ 81bdb56 (macro expansion, ' + side + ' corrections applied)' if side == 'left' else 'public exact-E2 description renesas-rdk/inspire_rh56e2_hand @ 81bdb56 (macro expansion)',
                       'actuators': acts, 'coupled_joints': ad.coupled_joints(side),
                       'native_interface': {'axis_order_owner_context': list(ad.NATIVE_ORDER), 'scale_owner_context': '0..1000 counts, 1000 = open (all six; thumb rotation 1000 = out = yaw 0)', 'status': 'PUBLIC_PRIOR_UNVERIFIED_ON_INSTALLED_UNIT',
                                            'count_mapping': 'E2_KINEMATIC_CONTRACT.json count_mapping (c = 1 - count/1000, rad = c * closed_rad, coupled targets clamped at the child limit)'},
                       'feedback': {'independent_axes_measured': True, 'coupled_joints_measured': False, 'note': donor['hands'][side]['feedback']['note']},
                       'drive_model': ad.DRIVE_MODEL['kind'], 'donor_to_e2_name_map': ad.name_map(side),
                       'link_names': {'base_link': side + '_base', 'palm_body': side + '_hand_base_link', 'palm_body_rpy_from_base': [3.14159, 0.0, 0.0],
                                      'note': 'base_link is the pure flange frame under <side>_wrist_yaw_link (folded into the palm body on USD import); palm_body is the first physical link (public hand_base_joint rpy 3.14159 0 0, xyz 0)'}}
    m['hands'] = hands
    tr = {}
    right = a['hands']['right']['wrist_mount']
    # the twin binds sha256(json.dumps(fixed_joint_matrix(origin))) of the RIGHT flange joint (embodiment.dependency_values_from_urdf); same formula here
    wrist_mount_sha = hashlib.sha256(json.dumps(rpy_matrix(right['xyz_m'], right['rpy_rad'])).encode()).hexdigest()
    for side in ad.SIDES:
        h = a['hands'][side]['wrist_mount']; T = rpy_matrix(h['xyz_m'], h['rpy_rad'])
        tr[side] = {'wrist_to_hand': {'matrix_4x4': T, 'frame': '%s_wrist_yaw_link -> %s_base' % (side, side), 'provenance': h['provenance'],
                                      'valid_for': {'urdf_sha256': a['merged_urdf']['sha256'], 'wrist_mount_sha256': wrist_mount_sha},
                                      'dependent_qualifications': ['contact_writing', 'command_replay_integration']}, 'hand_to_tool': None}
    m['transforms'] = tr
    m['controller'] = {**donor['controller'], 'type': donor['controller']['type'].replace('coupled joints follow mimic cons', 'coupled joints follow the E2 coupling table with child-limit clamps; original: coupled joints follow mimic cons'), 'hand_drive_model': ad.DRIVE_MODEL}
    m['tactile_force'] = {**donor['tactile_force'],
                          'tactile_arrays_e2_t1': {'availability': 'unavailable', 'units': 'counts 0..4095 (manual)', 'note': '17 force-sensor frames per hand carried from the public URDF with the Table 56 array assignment (E2_TACTILE_FRAMES.json, TACTILE_LAYOUT_PUBLIC_PRIOR); no taxel simulation, no measured layout'},
                          'contact_normal_force': {**donor['tactile_force']['contact_normal_force'], 'note': 'PhysX contact impulses / dt on the 17 sensor links (public geometry)'}}
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--asset-dir', required=True); ap.add_argument('--donor-manifest', required=True); ap.add_argument('--audit'); ap.add_argument('--out', required=True); a = ap.parse_args()
    asset = json.loads((Path(a.asset_dir) / 'E2_PRIOR_ASSET_MANIFEST.json').read_text()); donor = json.loads(Path(a.donor_manifest).read_text()); audit = json.loads(Path(a.audit).read_text()) if a.audit else None
    m = build(asset, donor, audit); Path(a.out).write_text(json.dumps(m, indent=1) + '\n')
    try:
        import sys; sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
        from isaac.twin.inspire.embodiment import EmbodimentManifest
        em = EmbodimentManifest.load(a.out); print(json.dumps({'manifest_id': em.data['manifest_id'], 'sha256': em.sha256, 'right_thumb_rotation_closed_rad': em.hand_actuator('right', 'thumb_rotation')['closed_rad'], 'validated': True}))
    except ImportError as e:
        print(json.dumps({'validated': False, 'reason': str(e)}))


if __name__ == '__main__':
    main()
