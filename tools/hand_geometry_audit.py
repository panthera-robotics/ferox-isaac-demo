#!/usr/bin/env python3
"""Hand geometry audit from the ACTUAL source URDF/meshes (CPU, read-only, no simulator).

Produces a fidelity ledger and labelled projections (palm/back/side, single-actuator
sweeps, task pose, wrist mounting) of the real link meshes: no generated imagery.
Vertex projections are exact geometry; they are not a collision-cooking proof.
"""
import argparse, json, math, sys, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def load_urdf(path):
    root = ET.parse(path).getroot()
    links = {l.get('name'): l for l in root.findall('link')}
    joints = {}
    for j in root.findall('joint'):
        o = j.find('origin'); a = j.find('axis'); lim = j.find('limit'); m = j.find('mimic')
        xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
        rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
        joints[j.get('name')] = dict(name=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'),
            child=j.find('child').get('link'), xyz=xyz, rpy=rpy,
            axis=[float(v) for v in a.get('xyz').split()] if a is not None else None,
            limit=(float(lim.get('lower')), float(lim.get('upper'))) if lim is not None else None,
            effort=float(lim.get('effort')) if lim is not None and lim.get('effort') else None,
            velocity=float(lim.get('velocity')) if lim is not None and lim.get('velocity') else None,
            mimic=(m.get('joint'), float(m.get('multiplier', 1)), float(m.get('offset', 0))) if m is not None else None)
    return root, links, joints


def fk(joints, q, root_link):
    """Return {link: 4x4} with mimics resolved; q maps independent joint name -> angle."""
    values = {}
    for name, j in joints.items():
        if j['type'] != 'revolute':
            values[name] = 0.0
        elif j['mimic']:
            values[name] = None
        else:
            values[name] = float(q.get(name, 0.0))
    changed = True
    while changed:
        changed = False
        for name, j in joints.items():
            if values[name] is None and values[j['mimic'][0]] is not None:
                values[name] = j['mimic'][1] * values[j['mimic'][0]] + j['mimic'][2]; changed = True
    frames = {root_link: np.eye(4)}
    pending = dict(joints)
    while pending:
        progressed = False
        for name in list(pending):
            j = pending[name]
            if j['parent'] in frames:
                T = np.eye(4); T[:3, :3] = rpy_matrix(*j['rpy']); T[:3, 3] = j['xyz']
                if j['type'] == 'revolute':
                    ax = np.asarray(j['axis'], dtype=float); ax /= np.linalg.norm(ax); th = values[name]
                    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                    R = np.eye(3) + math.sin(th)*K + (1-math.cos(th))*K@K
                    J = np.eye(4); J[:3, :3] = R; T = T @ J
                frames[j['child']] = frames[j['parent']] @ T
                del pending[name]; progressed = True
        if not progressed:
            raise ValueError('disconnected joints: ' + ','.join(pending))
    return frames, values


def link_mesh(urdf_dir, link, kind='visual'):
    g = link.find(kind + '/geometry/mesh')
    if g is None:
        return None
    m = trimesh.load_mesh(urdf_dir / g.get('filename'), process=False)
    scale = g.get('scale')
    if scale:
        m.apply_scale([float(v) for v in scale.split()])
    o = link.find(kind + '/origin')
    if o is not None:
        T = np.eye(4); T[:3, :3] = rpy_matrix(*[float(v) for v in o.get('rpy', '0 0 0').split()]); T[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
        m.apply_transform(T)
    return m


def world_vertices(meshes, frames):
    out = {}
    for name, m in meshes.items():
        if m is None or name not in frames:
            continue
        T = frames[name]; out[name] = m.vertices @ T[:3, :3].T + T[:3, 3]
    return out


def project(ax, verts_by_link, plane, labels=True, color_by=None, scale_bar=True, title=None, triad_frames=None):
    i, j = {'xy': (0, 1), 'xz': (0, 2), 'yz': (1, 2), 'zx': (2, 0), 'zy': (2, 1), 'yx': (1, 0)}[plane]
    for name, v in verts_by_link.items():
        c = (color_by or {}).get(name, ('tab:red' if 'thumb' in name else 'tab:blue' if 'sensor' in name else 'k'))
        ax.plot(v[:, i], v[:, j], '.', ms=0.6, color=c, alpha=0.5, rasterized=True)
        if labels:
            ctr = v.mean(axis=0); ax.annotate(name.replace('right_', ''), (ctr[i], ctr[j]), fontsize=4, alpha=0.9)
    if triad_frames:
        for fname, T in triad_frames.items():
            o = T[:3, 3]
            for k, col, lab in ((0, 'r', 'x'), (1, 'g', 'y'), (2, 'b', 'z')):
                d = T[:3, k] * 0.02
                ax.annotate('', xy=(o[i]+d[i], o[j]+d[j]), xytext=(o[i], o[j]), arrowprops=dict(arrowstyle='->', color=col, lw=1))
                ax.annotate(lab, (o[i]+d[i], o[j]+d[j]), fontsize=6, color=col)
    ax.set_aspect('equal'); ax.grid(True, lw=0.3, alpha=0.4)
    ax.set_xlabel('%s [m]' % 'xyz'[i]); ax.set_ylabel('%s [m]' % 'xyz'[j])
    if scale_bar:
        x0, x1 = ax.get_xlim(); y0, y1 = ax.get_ylim()
        ax.plot([x0+0.01, x0+0.03], [y0+0.01, y0+0.01], 'k-', lw=2); ax.annotate('20 mm', (x0+0.01, y0+0.013), fontsize=6)
    if title:
        ax.set_title(title, fontsize=8)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--root-link', default='right_base_link')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--grasp-config', type=Path, default=None, help='GraspConfig JSON (independent joint targets) for a task pose view')
    p.add_argument('--mount-urdf', type=Path, default=None, help='merged robot URDF for the wrist mounting transform')
    p.add_argument('--model-id', default='provisional Unitree FTP donor (unqualified vs RH56E2-2R-T1)')
    a = p.parse_args(argv)
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    a.out.mkdir(parents=True, exist_ok=True)
    root, links, joints = load_urdf(a.urdf)
    urdf_dir = a.urdf.parent
    meshes = {n: link_mesh(urdf_dir, l) for n, l in links.items()}
    cmeshes = {n: link_mesh(urdf_dir, l, 'collision') for n, l in links.items()}
    independent = [n for n, j in joints.items() if j['type'] == 'revolute' and not j['mimic']]
    coupled = [n for n, j in joints.items() if j['type'] == 'revolute' and j['mimic']]
    open_frames, _ = fk(joints, {}, a.root_link)
    # --- ledger: per-link mesh stats in link frame
    ledger = {'schema_version': 1, 'kind': 'hand_geometry_audit_from_source', 'model_id': a.model_id, 'urdf': str(a.urdf),
              'root_link': a.root_link, 'independent_joints': independent, 'coupled_joints': {n: joints[n]['mimic'] for n in coupled},
              'units_check': 'mesh vertices interpreted in metres; extents below must be O(0.01-0.2) for a human-scale hand', 'links': {}}
    for n, m in meshes.items():
        if m is None: continue
        ext = (m.vertices.max(0) - m.vertices.min(0)).tolist()
        ledger['links'][n] = {'vertices': int(len(m.vertices)), 'faces': int(len(m.faces)), 'extent_m_link_frame': ext,
                              'watertight': bool(m.is_watertight), 'visual_equals_collision_mesh': links[n].find('visual/geometry/mesh').get('filename') == (links[n].find('collision/geometry/mesh').get('filename') if links[n].find('collision/geometry/mesh') is not None else None),
                              'origin_in_root_open_pose_m': open_frames[n][:3, 3].round(6).tolist()}
    # --- joints: origins and axes in root frame at open pose; segment lengths
    jl = {}
    for n, j in joints.items():
        if j['type'] != 'revolute': continue
        T = open_frames[j['child']]; parent_T = open_frames[j['parent']]
        axis_root = (T[:3, :3] @ np.asarray(j['axis'])).round(4).tolist()
        jl[n] = {'origin_root_m': T[:3, 3].round(5).tolist(), 'axis_root_open_pose': axis_root, 'limit_rad': j['limit'], 'effort_nm': j['effort'], 'velocity_rad_s': j['velocity'], 'mimic': j['mimic'], 'parent': j['parent'], 'child': j['child']}
    ledger['joints'] = jl
    seg = {}
    for f in ('index', 'middle', 'ring', 'little'):
        a1 = np.asarray(jl[f'right_{f}_1_joint']['origin_root_m']); a2 = np.asarray(jl[f'right_{f}_2_joint']['origin_root_m'])
        tip = world_vertices({f'right_{f}_2': meshes[f'right_{f}_2']}, open_frames)[f'right_{f}_2']
        reach = float(np.max(tip @ (a2 - a1) / np.linalg.norm(a2 - a1)) - np.dot(a1, (a2 - a1) / np.linalg.norm(a2 - a1)))
        seg[f] = {'mcp_root_m': a1.tolist(), 'proximal_length_m': float(np.linalg.norm(a2 - a1)), 'mcp_to_distal_tip_along_finger_m': reach}
    th = [np.asarray(jl[f'right_thumb_{k}_joint']['origin_root_m']) for k in (1, 2, 3, 4)]
    seg['thumb'] = {'segment_lengths_m': [float(np.linalg.norm(th[k+1]-th[k])) for k in range(3)], 'thumb1_root_m': th[0].tolist()}
    ledger['segments'] = seg
    # --- chirality: fingers direction, palm normal (flexion direction of the index proximal at small angle), thumb offset
    zero_tip = open_frames['right_index_2'][:3, 3]
    flex_frames, _ = fk(joints, {'right_index_1_joint': 0.3}, a.root_link)
    flex_tip = flex_frames['right_index_2'][:3, 3]
    fingers_dir = (zero_tip - open_frames['right_index_1'][:3, 3]); fingers_dir /= np.linalg.norm(fingers_dir)
    palm_normal = flex_tip - zero_tip; palm_normal -= fingers_dir * np.dot(palm_normal, fingers_dir); palm_normal /= np.linalg.norm(palm_normal)
    index_mcp = np.asarray(jl['right_index_1_joint']['origin_root_m']); little_mcp = np.asarray(jl['right_little_1_joint']['origin_root_m'])
    thumb_side = th[0] - 0.5*(index_mcp + little_mcp); thumb_side -= fingers_dir*np.dot(thumb_side, fingers_dir); thumb_side /= np.linalg.norm(thumb_side)
    # Right hand: with palm normal n toward a viewer and fingers f up, the thumb lies on the viewer's right = (viewing dir = -n) x up(f) = -(n x f) = f x n.
    handedness = float(np.dot(thumb_side, np.cross(fingers_dir, palm_normal)))
    ledger['chirality'] = {'fingers_direction_root': fingers_dir.round(4).tolist(), 'palm_normal_root_flexion_side': palm_normal.round(4).tolist(),
        'thumb_side_root': thumb_side.round(4).tolist(), 'thumb_dot_fingers_cross_palm_normal': handedness,
        'verdict': 'RIGHT hand geometry' if handedness > 0.5 else 'LEFT hand geometry' if handedness < -0.5 else 'ambiguous',
        'rule': 'right hand: thumb lies along fingers x palm_normal (viewer facing the palm with fingers up sees the thumb on the right)'}
    # --- palm thickness and MCP placement (base link y-range vs finger hinge line)
    base = meshes[a.root_link].vertices
    palm_band = base[(base[:, 2] > 0.09) & (base[:, 2] < 0.15)]
    ledger['palm_body'] = {'y_min_m': float(palm_band[:, 1].min()), 'y_max_m': float(palm_band[:, 1].max()),
        'thickness_m_z_0p09_to_0p15': float(palm_band[:, 1].max() - palm_band[:, 1].min()),
        'width_m_x': float(palm_band[:, 0].max() - palm_band[:, 0].min()),
        'finger_mcp_y_m': float(np.mean([jl[f'right_{f}_1_joint']['origin_root_m'][1] for f in ('index', 'middle', 'ring', 'little')])),
        'note': 'source visual mesh band z in [0.09,0.15] m; MCP hinge line relative to the palm faces'}
    # --- thumb opposition reachability: min distance thumb tip (thumb_4 distal extreme) to index distal tip over joint grid
    def tip_of(link, frames):
        v = world_vertices({link: meshes[link]}, frames)[link]; o = frames[link][:3, 3]
        return v[np.argmax(np.linalg.norm(v - o, axis=1))]
    best = None
    for yaw in np.linspace(*joints['right_thumb_1_joint']['limit'], 13):
        for bend in np.linspace(*joints['right_thumb_2_joint']['limit'], 9):
            for fq in np.linspace(*joints['right_index_1_joint']['limit'], 13):
                fr, _ = fk(joints, {'right_thumb_1_joint': yaw, 'right_thumb_2_joint': bend, 'right_index_1_joint': fq}, a.root_link)
                d = float(np.linalg.norm(tip_of('right_thumb_4', fr) - tip_of('right_index_2', fr)))
                if best is None or d < best[0]:
                    best = (d, yaw, bend, fq)
    ledger['thumb_index_opposition'] = {'min_tip_distance_m': best[0], 'at_thumb_yaw_rad': best[1], 'at_thumb_bend_rad': best[2], 'at_index_rad': best[3],
        'scope': 'kinematic tip-to-tip distance on source meshes (13x9x13 grid); collision not evaluated; <~15 mm means a pinch is reachable'}
    # --- overall envelope
    allv = np.vstack(list(world_vertices(meshes, open_frames).values()))
    ledger['open_hand_envelope_root_m'] = {'min': allv.min(0).round(4).tolist(), 'max': allv.max(0).round(4).tolist(), 'extent': (allv.max(0)-allv.min(0)).round(4).tolist()}
    # --- mounting
    if a.mount_urdf:
        mroot = ET.parse(a.mount_urdf).getroot()
        for j in mroot.findall('joint'):
            if j.find('child').get('link') == a.root_link:
                o = j.find('origin'); ledger['mounting'] = {'joint': j.get('name'), 'parent': j.find('parent').get('link'), 'xyz': o.get('xyz'), 'rpy': o.get('rpy'),
                    'note': 'source flange transform, not installed wrist extrinsics'}
    (a.out / 'hand_geometry_ledger.json').write_text(json.dumps(ledger, indent=2))
    # --- figures: open hand three views
    vw = world_vertices(meshes, open_frames)
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.5))
    triads = {'root': open_frames[a.root_link]}
    # x right / z up on the page is the view from -y (the BACK of the hand: palm normal is +y);
    # mirroring x gives the view from +y (the PALM), thumb then on the viewer's right = right hand.
    project(axes[0], {k: v*np.array([-1, 1, 1]) for k, v in vw.items()}, 'xz', title='PALM view (viewer at +y looking along -y; page x = -x): thumb on viewer RIGHT => right hand', triad_frames=None)
    project(axes[1], vw, 'xz', title='BACK view (viewer at -y looking along +y): x right, z up', triad_frames=triads)
    project(axes[2], vw, 'yz', title='SIDE view (viewer at -x looking along +x): +y = palm/flexion side, +z = fingers', triad_frames=triads)
    fig.suptitle(f'{a.model_id}\nopen hand, source visual meshes (== collision meshes), root={a.root_link}; chirality: {ledger["chirality"]["verdict"]}', fontsize=9)
    fig.tight_layout(); fig.savefig(a.out / 'open_hand_three_views.png', dpi=170); plt.close(fig)
    # --- single actuator sweeps
    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    for ax, jn in zip(axes.ravel(), independent):
        lo, hi = joints[jn]['limit']
        for k, frac in enumerate((0.0, 0.5, 1.0)):
            fr, vals = fk(joints, {jn: lo + frac*(hi-lo)}, a.root_link)
            moved = {n: v for n, v in world_vertices(meshes, fr).items() if n != a.root_link}
            col = ['k', 'tab:orange', 'tab:red'][k]
            project(ax, moved, 'yz' if 'thumb_1' not in jn else 'xy', labels=(k == 0), color_by={n: col for n in moved}, scale_bar=(k == 0))
        coupled_here = [c for c in coupled if joints[c]['mimic'][0] == jn or (joints[c]['mimic'][0] in coupled and joints[joints[c]['mimic'][0]]['mimic'][0] == jn)]
        ax.set_title(f'{jn}: {lo:.3f}->{hi:.3f} rad (black/orange/red = 0/50/100%)\ncoupled: {", ".join(c.replace("right_","") for c in coupled_here) or "none"}; axis(root)={jl[jn]["axis_root_open_pose"]}', fontsize=7)
    fig.suptitle('Single-actuator sweeps of the six independent axes (source kinematics, mimic ratios applied); native units are NOT radians', fontsize=9)
    fig.tight_layout(); fig.savefig(a.out / 'single_actuator_sweeps.png', dpi=150); plt.close(fig)
    # --- task pose
    if a.grasp_config:
        cfg = json.loads(a.grasp_config.read_text())
        q = {}
        fingers = cfg.get('finger_initial_rad') or [cfg.get('four_finger_initial_rad', 0.0)]*4
        for f, v in zip(('index', 'middle', 'ring', 'little'), fingers): q[f'right_{f}_1_joint'] = v
        q['right_thumb_1_joint'] = cfg.get('thumb_yaw_initial_rad', 0.0); q['right_thumb_2_joint'] = cfg.get('thumb_flexion_initial_rad', 0.0)
        fr, vals = fk(joints, q, a.root_link); tv = world_vertices(meshes, fr)
        fig, axes = plt.subplots(1, 3, figsize=(15, 5.5))
        c = cfg.get('holder_center_palm_m'); r = 0.012
        for ax, plane in zip(axes, ('xz', 'yz', 'xy')):
            project(ax, tv, plane, title=f'task pose ({plane})')
            if c is not None:
                i, j = {'xz': (0, 2), 'yz': (1, 2), 'xy': (0, 1)}[plane]
                if plane == 'yz':
                    ax.add_patch(plt.Circle((c[1], c[2]), r, fill=False, color='tab:green', lw=1.5))
                else:
                    ax.plot([c[i]-0.05 if plane != 'yz' else c[i], c[i]+0.05], [c[j], c[j]], color='tab:green', lw=6, alpha=0.4)
        fig.suptitle(f'Grasp candidate pose: {json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in q.items()})}\nholder centre (palm) {c} r=12 mm; resolved coupled joints: ' +
                     ', '.join(f'{k.replace("right_","")}={vals[k]:.3f}' for k in coupled), fontsize=7)
        fig.tight_layout(); fig.savefig(a.out / 'task_pose_views.png', dpi=150); plt.close(fig)
        ledger['task_pose'] = {'independent_rad': q, 'coupled_rad': {k: vals[k] for k in coupled}}
        (a.out / 'hand_geometry_ledger.json').write_text(json.dumps(ledger, indent=2))
    print(json.dumps({'chirality': ledger['chirality']['verdict'], 'envelope_extent_m': ledger['open_hand_envelope_root_m']['extent'], 'segments': seg, 'independent': independent}, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
