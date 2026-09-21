#!/usr/bin/env python3
"""Build the PUBLIC exact-E2 prior asset `g1_edu29_rh56e2_e2prior_v1` from the pinned public RH56E2 description
(renesas-rdk/inspire_rh56e2_hand @ 81bdb56) and the G1 29-DoF body of the frozen donor asset, WITHOUT touching the donor.

Label: PUBLIC_E2_PRIOR__INSTALLED_CALIBRATION_INCOMPLETE. Nothing here is an installed measurement. Steps:
  1. verify the pinned source (commit, every STL a real mesh whose sha256 equals its git-lfs pointer oid) and hash every file used;
  2. expand the right/left xacro macros to plain URDF (prefix right_/left_), mesh paths -> copied meshes;
  3. correct the public LEFT file: the main links' inertials are copied from the right (not mirrored) and left_pinky_intermediate carries
     the proximal mass -> use the right inertials mirrored mathematically across the hand-base y=0 plane and the right pinky_intermediate mass;
  4. merge both hands onto the donor's G1 body (the FTP hand subtrees removed) at the donor flange transform composed with R_z(+90 deg)
     (the E2 base frame is the FTP base frame rotated: fingers +z, across y, palm normal +x) so the finger datum lands on the donor's;
  5. write the merged URDF, per-side hand and bench URDFs, meshes, and E2_PRIOR_ASSET_MANIFEST.json with every hash and the mass policy.
usage: e2prior_asset.py --source-root <clone> --donor-dir <generated/ftp_donor> --output <generated/e2prior>
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import re
import shutil
import struct
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

SOURCE_URL = 'https://github.com/renesas-rdk/inspire_rh56e2_hand'
SOURCE_SHA = '81bdb56'
DESC = Path('inspire_rh56e2_hand_description')
MOUNT = {'right': ('0.0415 0 0', '1.5707963 0 1.5707963'), 'left': ('0.0415 0 0', '-1.5707963 0 -1.5707963')}   # donor flange . R_z(+90 deg)
PRIOR = {'finger_proximal_rad': 1.4381, 'finger_distal_multiplier': 1.0843, 'finger_distal_cap_rad': 1.4764, 'thumb_bend_rad': 0.62,
         'thumb_intermediate_multiplier': 0.8392, 'thumb_distal_multiplier': 0.7477, 'thumb_rotation_rad': 1.658}
# Declared drive-limit policy for the MERGED twin URDF only (the per-hand E2 URDFs keep the public placeholders byte-for-byte):
# the public effort (1 N·m) and velocity (right fingers 1.0, left fingers 2.0, thumbs 2.0 rad/s) fields are placeholders, and the
# URDF velocity field becomes the PhysX joint velocity cap on import (Sprint M). The twin therefore carries the frozen donor's values
# on all 24 hand joints, equal on both sides, recorded per joint; nothing here is an installed measurement.
DRIVE_POLICY = {'effort_Nm': 10.0, 'velocity_rad_s': 1.0, 'policy': 'DONOR_DRIVE_LIMITS_CARRIED', 'installed': 'NOT_MEASURED (hand-req-Q01 actuator step-response logger)',
                'provenance': 'unitree_ros FTP donor URDF (sha256 63097d73...) effort 10 / velocity 1 on every hand joint; the public E2 placeholders are recorded per joint below'}


def sha256(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def git(source, *args):
    return subprocess.check_output(['git', '-C', str(source)] + list(args), text=True)


def verify_source(source):
    head = git(source, 'rev-parse', '--short=7', 'HEAD').strip()
    if head != SOURCE_SHA:
        raise ValueError('source must be checked out at %s (is %s)' % (SOURCE_SHA, head))
    full = git(source, 'rev-parse', 'HEAD').strip(); meshes = {}
    for stl in sorted((source / DESC / 'meshes').glob('*/*.STL')):
        raw = stl.read_bytes()
        if raw[:40].startswith(b'version https://git-lfs.github.com/spec'):
            raise ValueError('LFS pointer stub, not a mesh: %s' % stl)
        ptr = git(source, 'show', 'HEAD:' + str(stl.relative_to(source)))
        oid = re.search(r'oid sha256:([0-9a-f]{64})', ptr).group(1); size = int(re.search(r'size (\d+)', ptr).group(1))
        h = hashlib.sha256(raw).hexdigest()
        if h != oid or len(raw) != size:
            raise ValueError('mesh %s does not match its LFS pointer (%s vs %s)' % (stl, h[:12], oid[:12]))
        if len(raw) < 84 or len(raw) != 84 + 50 * struct.unpack('<I', raw[80:84])[0]:
            raise ValueError('not a binary STL of consistent size: %s' % stl)
        meshes[str(stl.relative_to(source))] = {'sha256': h, 'bytes': len(raw), 'triangles': struct.unpack('<I', raw[80:84])[0], 'lfs_pointer_verified': True}
    return full, meshes


def expand(macro_text, prefix, parent, xyz, rpy, mesh_dir):
    body = re.search(r'<xacro:macro name="inspire_hand_e2_(?:right|left)" params="prefix parent \*origin">(.*)</xacro:macro>\s*</robot>', macro_text, re.S).group(1)
    gm = re.search(r'<xacro:macro name="grasp_tcp" params="name xyz rpy">(.*?)</xacro:macro>', body, re.S); gbody = gm.group(1); body = body.replace(gm.group(0), '')
    def tcp(m):
        d = dict(re.findall(r'(\w+)="([^"]*)"', m.group(1))); return gbody.replace('${name}', d['name']).replace('${xyz}', d['xyz']).replace('${rpy}', d['rpy'])
    body = re.sub(r'<xacro:grasp_tcp ([^/]*)/>', tcp, body)
    body = body.replace('<xacro:insert_block name="origin" />', '<origin xyz="%s" rpy="%s"/>' % (xyz, rpy)).replace('${prefix}', prefix).replace('${parent}', parent)
    body = re.sub(r'package://inspire_rh56e2_hand_description/meshes/(?:right|left)/', mesh_dir.rstrip('/') + '/', body)
    if '${' in body or 'xacro:' in body:
        raise ValueError('unexpanded xacro remains')
    return ET.fromstring('<robot name="x">%s</robot>' % body)


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def R_to_rpy(R):
    p = -math.asin(max(-1.0, min(1.0, R[2, 0]))); return (math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0]))


def link_frames(robot, root_link):
    """Pose of every link in the root link frame at the zero configuration (fixed and revolute joints at 0)."""
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}; poses = {root_link: (np.eye(3), np.zeros(3))}
    def pose(link):
        if link in poses: return poses[link]
        j = joints[link]; Rp, tp = pose(j.find('parent').get('link')); o = j.find('origin')
        xyz = np.array([float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]); rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
        R = Rp @ rpy_to_R(*rpy); t = tp + Rp @ xyz; poses[link] = (R, t); return poses[link]
    for l in robot.findall('link'): pose(l.get('name'))
    return poses


def inertial(link):
    i = link.find('inertial')
    if i is None: return None
    o = i.find('origin'); xyz = np.array([float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]); rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    m = float(i.find('mass').get('value')); I = i.find('inertia'); g = lambda k: float(I.get(k))
    return m, xyz, rpy, np.array([[g('ixx'), g('ixy'), g('ixz')], [g('ixy'), g('iyy'), g('iyz')], [g('ixz'), g('iyz'), g('izz')]])


def mirror_left_inertials(right, left, right_root, left_root):
    """Replace every left MAIN link inertial by the right link's inertial mirrored across the hand-base y=0 plane, expressed in the left
    link frame (inertial rpy 0); the exporter's own force-sensor links are left untouched but used as the method check."""
    PR = link_frames(right, right_root); PL = link_frames(left, left_root); M = np.diag([1.0, -1.0, 1.0]); report = {}
    lefts = {l.get('name'): l for l in left.findall('link')}
    for rl in right.findall('link'):
        name = rl.get('name'); lname = name.replace('right_', 'left_', 1); ine = inertial(rl)
        if ine is None or lname not in lefts: continue
        m, xyz, rpy, I = ine; RR, tR = PR[name]; RL, tL = PL[lname]; Ro = rpy_to_R(*rpy)
        com_b = RR @ xyz + tR; I_b = RR @ Ro @ I @ Ro.T @ RR.T; com_m = M @ com_b; I_m = M @ I_b @ M
        com_l = RL.T @ (com_m - tL); I_l = RL.T @ I_m @ RL
        old = inertial(lefts[lname]); old_m, old_xyz, old_rpy, old_I = old; RoL = rpy_to_R(*old_rpy); old_I_link = RoL @ old_I @ RoL.T
        dcom = float(np.linalg.norm(com_l - old_xyz)); dI = float(np.abs(I_l - old_I_link).max() / max(np.abs(old_I_link).max(), 1e-15))
        sensor = 'force_sensor' in name or name.endswith('_tcp')
        report[lname] = {'kind': 'exporter_mirrored_check' if sensor else 'main_link_replaced', 'com_shift_m': round(dcom, 6), 'inertia_rel_diff': round(dI, 5), 'mass_kg': m}
        if sensor: continue
        node = lefts[lname].find('inertial'); node.find('origin').set('xyz', '%.9g %.9g %.9g' % tuple(com_l)); node.find('origin').set('rpy', '0 0 0')
        node.find('mass').set('value', '%.9g' % m)
        for k, v in (('ixx', I_l[0, 0]), ('ixy', I_l[0, 1]), ('ixz', I_l[0, 2]), ('iyy', I_l[1, 1]), ('iyz', I_l[1, 2]), ('izz', I_l[2, 2])): node.find('inertia').set(k, '%.9g' % v)
    return report


def mass_policy(per_side_mass):
    return {'primary': 'PUBLIC_URDF_LINK_MASSES', 'hand_total_kg': per_side_mass,
            'statement': 'The primary asset carries the public URDF per-link masses unchanged (right file as published; left = right mirrored, left_pinky_intermediate corrected to the right value). '
                         'This is the CAD-export mass of the public description (E2_CAD_MASS), not an installed measurement, and it was not chosen for locomotion convenience: the frozen donor '
                         'hand is 0.8783 kg, so the body carries 0.1016 kg less per hand than before and any policy trained on the donor mass sees that change explicitly.',
            'installed_mass': 'NOT_MEASURED (RH56E2_INSTALLED_CALIBRATION_INCOMPLETE; hand-req-Q01 mass/COM sheet)',
            'diagnostics': {'E2_CAD_MASS': {'value_kg': per_side_mass, 'definition': 'sum of the public URDF link masses (identical to the primary)'},
                            'E2_NOMINAL_MASS': {'value_kg': 0.790, 'tolerance_kg': 0.010, 'definition': 'manufacturer nominal for the tactile variant (owner packet); optional diagnostic build only: every hand link mass scaled by 0.790/E2_CAD_MASS, COM and inertia shape unchanged; NOT the primary asset'}}}


def build(source_root, donor_dir, output):
    source, donor, out = Path(source_root).resolve(), Path(donor_dir).resolve(), Path(output).resolve()
    if out.exists() and any(out.iterdir()): raise ValueError('output must be absent or empty: %s' % out)
    full_sha, meshes = verify_source(source)
    out.mkdir(parents=True); (out / 'meshes').mkdir()
    src_files = {}
    for rel in ['urdf/inspire_hand_e2_right_macro.xacro', 'urdf/inspire_hand_e2_left_macro.xacro', 'urdf/inspire_hand_e2_right.urdf.xacro', 'urdf/inspire_hand_e2_left.urdf.xacro',
                'urdf/reference/RH56E2_R_2025_9_11.urdf', 'urdf/reference/RH56E2_L_2025_9_10.urdf', '.gitattributes']:
        src_files[str(DESC / rel)] = sha256(source / DESC / rel)
    for rel in ['inspire_rh56e2_hand_ros2_control/doc/rh56e2_register_map.md', 'inspire_rh56e2_hand_ros2_control/urdf/inspire_rh56e2_hand_macro.ros2_control.xacro',
                'inspire_rh56e2_hand_ros2_control/config/inspire_rh56e2_hand_motion_mode_controller.yaml', 'inspire_rh56e2_hand_bringup/urdf/inspire_rh56e2_hand_right.urdf.xacro', 'inspire_rh56e2_hand_bringup/urdf/inspire_rh56e2_hand_left.urdf.xacro']:
        if (source / rel).exists(): src_files[rel] = sha256(source / rel)
    copied = {}
    for rel, info in meshes.items():
        side = Path(rel).parts[-2]; dest = out / 'meshes' / side / Path(rel).name; dest.parent.mkdir(exist_ok=True); shutil.copyfile(source / rel, dest); copied[str(dest.relative_to(out))] = info
    hands = {}
    for side in ('right', 'left'):
        macro = (source / DESC / 'urdf' / ('inspire_hand_e2_%s_macro.xacro' % side)).read_text()
        hands[side] = expand(macro, side + '_', side + '_wrist_yaw_link', *MOUNT[side], mesh_dir='meshes/%s' % side)
    # left corrections
    right_links = {l.get('name'): l for l in hands['right'].findall('link')}; left_links = {l.get('name'): l for l in hands['left'].findall('link')}
    pm_r = inertial(right_links['right_pinky_intermediate'])[0]; pm_l_before = inertial(left_links['left_pinky_intermediate'])[0]
    mirror_report = mirror_left_inertials(hands['right'], hands['left'], 'right_base', 'left_base')
    corrections = {'left_inertials': 'main-link inertials replaced by the right inertials mirrored across the hand-base y=0 plane (method validated on the exporter-mirrored force-sensor links: COM agreement <= 0.6 mm)',
                   'left_pinky_intermediate_mass_kg': {'public_file': pm_l_before, 'used': pm_r, 'source': 'right_pinky_intermediate (public right file)'},
                   'mirror_report': mirror_report}
    public_hands = {side: copy.deepcopy(hands[side]) for side in ('right', 'left')}   # public expansion + left corrections only (record: *_public.urdf)
    # r4 asset representation: fixed sub-gram hand links folded into their parent bodies (frames kept) + declared slab palm collider
    from . import e2_fold_slabs as fs
    representation = {'policy': 'E2_HAND_PHYSICS_REPRESENTATION_r4', 'why': 'first E2 physics runs (bench evR-R1-B-right, whole body evR-W2-B-e2-s184828) went non-finite at step 1: palm split over fixed links left palm<->finger-root pairs unfiltered, the hollow-shell convex decomposition bulged over the 0.2 mm-inset finger roots / thumb cavity, and the contacts landed on 17-280 mg sensor bodies and a 1 ug tcp body',
                      'class': 'asset REPRESENTATION (physics topology + palm collider); geometry, joint limits, masses and inertia totals unchanged', 'fold': {}, 'palm_collider': {}}
    for side in ('right', 'left'):
        representation['fold'][side] = fs.fold_fixed_hand_links(hands[side], side)
        representation['palm_collider'][side] = fs.slab_palm_collider(hands[side], side, out, out, Path(__file__).resolve().parents[2])
    # declared drive limits on the merged twin URDF (public placeholders recorded per joint)
    drive_record = {}
    for side in ('right', 'left'):
        for j in hands[side].findall('joint'):
            if j.get('type') != 'revolute': continue
            lim = j.find('limit'); drive_record[j.get('name')] = {'public_effort': float(lim.get('effort')), 'public_velocity': float(lim.get('velocity')), 'twin_effort': DRIVE_POLICY['effort_Nm'], 'twin_velocity': DRIVE_POLICY['velocity_rad_s']}
    merged_hands = {side: copy.deepcopy(hands[side]) for side in ('right', 'left')}
    for side in ('right', 'left'):
        for j in merged_hands[side].findall('joint'):
            if j.get('type') == 'revolute':
                j.find('limit').set('effort', '%g' % DRIVE_POLICY['effort_Nm']); j.find('limit').set('velocity', '%g' % DRIVE_POLICY['velocity_rad_s'])
    # merge onto the donor body
    donor_urdf = donor / 'g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf'; tree = ET.parse(donor_urdf); robot = tree.getroot(); robot.set('name', 'g1_29dof_rev_1_0_with_inspire_hand_E2')
    removed = {'links': [], 'joints': []}
    for side in ('right', 'left'):
        hand_links = {l.get('name') for l in ET.parse(donor / ('FTP_%s_hand.urdf' % side)).getroot().findall('link')} - {side + '_wrist_yaw_link'}
        for j in list(robot.findall('joint')):
            if j.find('child').get('link') in hand_links or j.find('parent').get('link') in hand_links: robot.remove(j); removed['joints'].append(j.get('name'))
        for l in list(robot.findall('link')):
            if l.get('name') in hand_links: robot.remove(l); removed['links'].append(l.get('name'))
    for side in ('right', 'left'):
        for el in list(merged_hands[side]):
            robot.append(el)
    body_meshes = {}
    for mesh in robot.iter('mesh'):
        fn = Path(mesh.get('filename'))
        if fn.parts[0] == 'meshes' and len(fn.parts) == 2:
            src = donor / fn; dest = out / fn
            if not dest.exists(): shutil.copyfile(src, dest)
            body_meshes[str(fn)] = {'sha256': sha256(dest), 'source': 'donor generated dir (G1 body mesh, unchanged)'}
    merged = out / 'g1_29dof_rev_1_0_with_inspire_hand_E2.urdf'; ET.indent(tree, space=' '); tree.write(merged, encoding='utf-8', xml_declaration=True)
    per_side = {}
    for side in ('right', 'left'):
        hp = ET.Element('robot', name='E2_%s_hand_public' % side); ET.SubElement(hp, 'link', name=side + '_wrist_yaw_link')
        for el in public_hands[side]: hp.append(copy.deepcopy(el))
        pp = out / ('E2_%s_hand_public.urdf' % side); tp = ET.ElementTree(hp); ET.indent(tp, space=' '); tp.write(pp, encoding='utf-8', xml_declaration=True)
        hr = ET.Element('robot', name='E2_%s_hand' % side); ET.SubElement(hr, 'link', name=side + '_wrist_yaw_link')
        for el in merged_hands[side]: hr.append(copy.deepcopy(el))
        p = out / ('E2_%s_hand.urdf' % side); t = ET.ElementTree(hr); ET.indent(t, space=' '); t.write(p, encoding='utf-8', xml_declaration=True)
        # bench: root = <side>_base (the flange joint removed, transform retained in the manifest); declared limits + representation like the twin (bench parity)
        br = ET.Element('robot', name='E2_%s_hand_bench' % side)
        for el in merged_hands[side]:
            if el.tag == 'joint' and el.get('name') == side + '_hand_connection_joint': continue
            br.append(copy.deepcopy(el))
        pb = out / ('E2_%s_hand_bench.urdf' % side); tb = ET.ElementTree(br); ET.indent(tb, space=' '); tb.write(pb, encoding='utf-8', xml_declaration=True)
        per_side[side] = {'hand_urdf': p.name, 'hand_urdf_sha256': sha256(p), 'bench_urdf': pb.name, 'bench_urdf_sha256': sha256(pb), 'bench_root': side + '_base', 'public_expansion_urdf': pp.name, 'public_expansion_urdf_sha256': sha256(pp),
                          'note': 'hand/bench URDFs carry the declared drive limits and the r4 representation like the merged twin (bench parity with the twin); the public expansion (placeholders, public sub-links) is kept as *_public.urdf for the record',
                          'wrist_mount': {'parent': side + '_wrist_yaw_link', 'child': side + '_base', 'xyz_m': [float(v) for v in MOUNT[side][0].split()], 'rpy_rad': [float(v) for v in MOUNT[side][1].split()],
                                          'provenance': 'donor flange transform (0.0415, 0, 0; rpy 0, +-pi/2, 0) composed with R_z(+90 deg) so that the E2 base (fingers +z, across y, palm normal +x) reproduces the donor finger datum (index/pinky joints and index tip within 0.1 mm; the thumb base sits 13.7 mm further along the fingers on the E2); NOT measured on hardware'}}
    per_side_mass = {s: round(sum(inertial(l)[0] for l in hands[s].findall('link') if inertial(l) is not None), 6) for s in ('right', 'left')}
    manifest = {'schema_version': 1, 'asset_id': 'g1_edu29_rh56e2_e2prior_v1', 'label': 'PUBLIC_E2_PRIOR__INSTALLED_CALIBRATION_INCOMPLETE',
                'donor_status': 'DONOR_BASELINE_FROZEN (generated/ftp_donor untouched; the G1 body links/joints and body meshes are copied from it)',
                'source': {'url': SOURCE_URL, 'commit': full_sha, 'commit_short': SOURCE_SHA, 'license': 'see source repository', 'files_sha256': src_files, 'meshes': copied, 'lfs': 'all 64 STL objects fetched through the git-lfs batch API and verified against their pointer oids (git-lfs binary not installed on the host)'},
                'donor_body': {'urdf': str(donor_urdf), 'urdf_sha256': sha256(donor_urdf), 'removed_ftp_hand': removed, 'body_meshes': body_meshes},
                'merged_urdf': {'name': merged.name, 'sha256': sha256(merged)}, 'hands': per_side, 'public_urdf_corrections': corrections,
                'public_prior': PRIOR,
                'drive_limit_policy': {**DRIVE_POLICY, 'applies_to': 'merged twin, per-hand and bench URDFs (E2_<side>_hand_public.urdf keeps the public placeholders)', 'per_joint': drive_record},
                'representation_r4': representation,
                'placeholders_not_used': {'urdf_effort_velocity': 'the public URDF effort/velocity attributes (1 / 1-2) are placeholders; the merged twin URDF carries the declared drive_limit_policy instead', 'motion_mode_2': 'unconfirmed; not used'},
                'command_direction_truth': '1000 = fully open, 0 = closed (manual 2.6.11/2.6.12 and the public driver open-pose reset use 1000); the public rh56e2_register_map.md line "0 (Fully open) to 1000 (Fully closed)" is reversed and is NOT followed',
                'mass_policy': mass_policy(per_side_mass), 'generated_usd': {'status': 'NOT_GENERATED_ON_CPU', 'note': 'the twin imports the merged URDF with the Kit URDF importer at run time (tools/inspire_body_asset.py import_body); its hash is recorded by the first admitted run'}}
    return manifest, out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--source-root', required=True); ap.add_argument('--donor-dir', required=True); ap.add_argument('--output', required=True); a = ap.parse_args()
    manifest, out = build(a.source_root, a.donor_dir, a.output)
    (out / 'E2_PRIOR_ASSET_MANIFEST.json').write_text(json.dumps(manifest, indent=1, allow_nan=False) + '\n')
    print(json.dumps({'output': str(out), 'merged_urdf_sha256': manifest['merged_urdf']['sha256'], 'meshes': len(manifest['source']['meshes'])}, indent=1))


if __name__ == '__main__':
    main()
