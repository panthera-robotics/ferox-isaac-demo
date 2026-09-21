#!/usr/bin/env python3
"""E2_TACTILE_FRAMES.json: the 17 force-sensor frames per hand of the public E2 URDF with their poses (hand-base frame at q=0 and
parent-link frame), mesh extents, and the taxel-array assignment of the manual's piezoresistive layout (Table 56: 1062 points per
hand) matched to the frames by mesh size class. Label TACTILE_LAYOUT_PUBLIC_PRIOR: the frame poses are EXACT_PUBLIC_SOURCE, the
array-to-frame assignment is INFERRED (mesh area), array orientation/row order on the installed hand is UNVERIFIED.
usage: e2_tactile_frames.py --asset-dir <generated/e2prior> --out E2_TACTILE_FRAMES.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pinocchio as pin

try:
    from . import mesh_penetration as mp
    from .register_map import TACTILE_PIEZORESISTIVE_TABLE56, TACTILE_CAPACITIVE_TABLE58
except ImportError:  # run as a script
    import mesh_penetration as mp
    from register_map import TACTILE_PIEZORESISTIVE_TABLE56, TACTILE_CAPACITIVE_TABLE58

FINGER = {'index': 'index', 'middle': 'middle', 'ring': 'ring', 'pinky': 'little'}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--asset-dir', required=True); ap.add_argument('--out', required=True); a = ap.parse_args(); asset = Path(a.asset_dir)
    manifest = json.loads((asset / 'E2_PRIOR_ASSET_MANIFEST.json').read_text()); urdf = asset / manifest['merged_urdf']['name']
    model = pin.buildModelFromUrdf(str(urdf)); data = model.createData(); geom = pin.buildGeomFromUrdf(model, str(urdf), pin.GeometryType.COLLISION, package_dirs=[str(asset)])
    gdata = pin.GeometryData(geom); q = pin.neutral(model); pin.forwardKinematics(model, data, q); pin.updateFramePlacements(model, data); pin.updateGeometryPlacements(model, data, geom, gdata)
    link_of = {go.name: model.frames[go.parentFrame].name for go in geom.geometryObjects}
    import xml.etree.ElementTree as ET
    xml = ET.parse(urdf).getroot(); parents = {}; joints = {}
    for j in xml.findall('joint'):
        o = j.find('origin'); parents[j.find('child').get('link')] = j.find('parent').get('link')
        joints[j.get('name')] = ([float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()], [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()])
    table = {n: {'address': ad, 'rows': r, 'cols': c, 'bytes': L} for n, ad, r, c, L in TACTILE_PIEZORESISTIVE_TABLE56}
    out = {'label': 'TACTILE_LAYOUT_PUBLIC_PRIOR', 'asset_id': manifest['asset_id'], 'merged_urdf_sha256': manifest['merged_urdf']['sha256'], 'taxels_per_hand': sum(v['rows'] * v['cols'] for v in table.values()),
           'raw_range': '0..4095 per point, 16-bit little-endian (manual Table 56 text)', 'dds': 'touch topics exist on the Unitree side (owner packet); topic names/rates NOT verified here',
           'variant_applicability': 'the manual lists a piezoresistive layout (Table 56, 17 arrays) and a capacitive record (Table 58, 5 fingers x 58 bytes); which one the installed E2-T1 serves is UNVERIFIED',
           'capacitive_table58': [{'finger': f, 'address': ad, 'bytes': L} for f, ad, L in TACTILE_CAPACITIVE_TABLE58], 'hands': {}}
    for side in ('right', 'left'):
        base = data.oMf[model.getFrameId(side + '_base')]; frames = {}
        for f in model.frames:
            if not (f.name.startswith(side + '_') and 'force_sensor' in f.name and f.type == pin.FrameType.BODY): continue
            oMf = data.oMf[model.getFrameId(f.name)]; bMf = base.inverse() * oMf; parent_link = parents[f.name]
            gi = [i for i, go in enumerate(geom.geometryObjects) if link_of[go.name] == f.name]
            ext = None; area_cm2 = None
            if gi:
                V, T = mp.mesh_arrays(geom.geometryObjects[gi[0]].geometry); ext = (V.max(0) - V.min(0)); s_ext = sorted(ext); area_cm2 = round(float(s_ext[2] * s_ext[1]) * 1e4, 2); ext = ext.round(4).tolist()
            frames[f.name] = {'parent_link': parent_link, 'fixed_joint': f.name + '_joint', 'pose_in_hand_base_q0': {'xyz_m': bMf.translation.round(6).tolist(), 'rotation': bMf.rotation.round(6).tolist()},
                              'pose_in_parent_link': {'xyz_m': [round(float(v), 6) for v in joints[f.name + '_joint'][0]], 'rpy_rad': [round(float(v), 6) for v in joints[f.name + '_joint'][1]]}, 'mesh_extent_m': ext, 'mesh_face_area_cm2': area_cm2}
        # array assignment by size class within each finger: the largest face = 12x8, middle = 10x8, smallest = 3x3; thumb: two large (12x8) two small (3x3); palm 8x14
        assign = {}
        for finger, tname in FINGER.items():
            names = sorted([n for n in frames if n.startswith(side + '_' + finger + '_force_sensor')], key=lambda n: -frames[n]['mesh_face_area_cm2'])
            for n, arr in zip(names, (tname + '_nail', tname + '_pad', tname + '_tip')): assign[n] = arr
        tn = sorted([n for n in frames if n.startswith(side + '_thumb_force_sensor')], key=lambda n: -frames[n]['mesh_face_area_cm2'])
        # the two large thumb arrays: 'thumb_nail' (4498) and 'thumb_tip_12x8' (4708); the two small: 'thumb_tip' (4480) and 'thumb_middle' (4690). Distal-most large = nail, distal-most small = tip.
        big = sorted(tn[:2], key=lambda n: -np.linalg.norm(frames[n]['pose_in_hand_base_q0']['xyz_m'])); small = sorted(tn[2:], key=lambda n: -np.linalg.norm(frames[n]['pose_in_hand_base_q0']['xyz_m']))
        assign[big[0]] = 'thumb_nail'; assign[big[1]] = 'thumb_tip_12x8'; assign[small[0]] = 'thumb_tip'; assign[small[1]] = 'thumb_middle'
        assign[side + '_palm_force_sensor'] = 'palm'
        for n in frames:
            arr = assign.get(n); frames[n]['array'] = None if arr is None else {'name': arr, **table[arr], 'taxels': table[arr]['rows'] * table[arr]['cols'], 'assignment': 'INFERRED_FROM_MESH_SIZE_AND_POSITION (finger: largest face = 12x8 nail, next = 10x8 pad, smallest = 3x3 tip; the two phalanx faces differ by < 10 % in area, so nail/pad may be swapped; verify on the installed hand)', 'orientation': 'UNVERIFIED'}
        out['hands'][side] = {'frames': frames, 'count': len(frames), 'taxels': sum(f['array']['taxels'] for f in frames.values() if f['array'])}
    Path(a.out).write_text(json.dumps(out, indent=1) + '\n')
    print(json.dumps({s: (out['hands'][s]['count'], out['hands'][s]['taxels']) for s in out['hands']}))


if __name__ == '__main__':
    main()
