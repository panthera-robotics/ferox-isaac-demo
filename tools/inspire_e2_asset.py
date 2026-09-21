"""Import the PUBLIC exact-E2 prior assembly (G1 body + renesas-rdk RH56E2 hands, generated/e2prior) with the SAME importer
settings, inertial handling and frame ownership as the FTP donor (inspire_body_asset.import_body), and the collision model the
E2 manifest declares: the public STL meshes as delivered (convex decomposition by the importer), no palm/thumb slab substitution.

Differences from the donor path, all declared in the returned facts:
  * the hand flange frame <side>_base is a pure frame between the wrist and the palm body (two fixed joints); the importer would
    invent a rigid body for it, so it is folded into the palm joint (origin = base_joint · hand_base_joint) and kept as a
    nonphysical USD Xform under the wrist link, like the IMU/camera frames;
  * the public convenience frames <side>_tcp_{three_fingers,pinch,full_hand} (no inertial, no geometry) become nonphysical
    Xforms under their parent; <side>_tcp carries a 1 µg inertial in the public file and stays a physical body as published;
  * the little finger is called 'pinky' in the public description (hand joint detection includes it).
Variant B (E2 geometry + donor per-link masses) is produced by variant_b_urdf() on CPU before the import.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

FINGERS = ['thumb', 'index', 'middle', 'ring', 'little', 'pinky']
NONPHYSICAL_LEAVES = {'imu_in_torso', 'imu_in_pelvis', 'd435_link', 'mid360_link'}
COLLISION_MODEL_E2 = 'public_stl_meshes;approximation=convexDecomposition(importer);offsets=importer_default;no_slab_substitution'
# The representation is baked into the delivered URDF (Q02-r4/r5): the declared collision model is derived from what the URDF carries.
COLLISION_MODELS_BY_CANDIDATE = {
    'e2_palm_yz_slabs_v2_thumb_cavity': ('e2_r5_folded_slabs', 'e2_r5:fixed_hand_links_folded;palm=e2_palm_yz_slabs_v2_thumb_cavity(4mm_xz_exact_meshes+1mm_thumb_cavity_carve_1.5mm_margin);fingers=convexDecomposition(importer);offsets=importer_default;baked_into_urdf'),
    'e2_palm_yz_slabs_v1': ('e2_r4_folded_slabs', 'e2_r4:fixed_hand_links_folded;palm=e2_palm_yz_slabs_v1(4mm_xz_exact_meshes);fingers=convexDecomposition(importer);offsets=importer_default;baked_into_urdf'),
}


def collision_model_of(root):
    """(kind, cooking) of a merged E2 URDF from the baked collision pieces it references."""
    names = ' '.join(m.get('filename', '') for m in root.iter('mesh'))
    for cand, (kind, cooking) in COLLISION_MODELS_BY_CANDIDATE.items():
        if cand in names: return kind, cooking
    return 'public_stl_meshes', COLLISION_MODEL_E2


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]]


def _to_rpy(R):
    p = -math.asin(max(-1.0, min(1.0, R[2][0]))); return [math.atan2(R[2][1], R[2][2]), p, math.atan2(R[1][0], R[0][0])]


def _origin(j):
    o = j.find('origin'); xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]; rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    return xyz, rpy


def _compose(xyz1, rpy1, xyz2, rpy2):
    """T1 · T2 as (xyz, rpy)."""
    R1, R2 = _rpy(*rpy1), _rpy(*rpy2)
    R = [[sum(R1[i][k] * R2[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    t = [xyz1[i] + sum(R1[i][k] * xyz2[k] for k in range(3)) for i in range(3)]
    return t, _to_rpy(R)


def prepare_source_e2(source, output):
    """Derived URDF for the importer: the flange frames folded, every empty link turned into a recorded nonphysical frame,
    mesh paths absolute. Returns the facts (hashes, frames, masses) like inspire_body_asset.prepare_source."""
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    root = ET.parse(source).getroot(); derived = deepcopy(root)
    joints = {j.get('name'): j for j in derived.findall('joint')}
    by_child = {j.find('child').get('link'): j for j in derived.findall('joint')}
    kids = {}
    for j in derived.findall('joint'):
        kids.setdefault(j.find('parent').get('link'), []).append(j)
    frames = []; folded = []
    # 1. fold <side>_base (empty link with one fixed parent joint and one fixed child joint)
    for side in ('right', 'left'):
        base = side + '_base'; link = derived.find("link[@name='%s']" % base)
        if link is None or list(link):
            raise ValueError('%s must be an empty flange frame in the public E2 assembly' % base)
        up = by_child[base]; downs = kids.get(base, [])
        if up.get('type') != 'fixed' or len(downs) != 1 or downs[0].get('type') != 'fixed':
            raise ValueError('%s must hang on one fixed joint and carry one fixed child joint' % base)
        down = downs[0]; xyz1, rpy1 = _origin(up); xyz2, rpy2 = _origin(down); xyz, rpy = _compose(xyz1, rpy1, xyz2, rpy2)
        frames.append({'name': base, 'parent': up.find('parent').get('link'), 'joint': up.get('name'), 'xyz_m': xyz1, 'rpy_rad': rpy1, 'source_inertia_or_geometry': False, 'representation': 'nonphysical_coordinate_frame',
                       'note': 'hand flange frame; the palm joint below carries the composed transform'})
        down.find('parent').set('link', up.find('parent').get('link')); o = down.find('origin'); o.set('xyz', ' '.join('%.9g' % v for v in xyz)); o.set('rpy', ' '.join('%.9g' % v for v in rpy))
        folded.append({'removed_link': base, 'removed_joint': up.get('name'), 'rewritten_joint': down.get('name'), 'new_parent': up.find('parent').get('link'), 'xyz_m': xyz, 'rpy_rad': rpy})
        derived.remove(link); derived.remove(up)
    # 2. every remaining empty link (leaf or with fixed children) on a fixed joint -> nonphysical frame; children of an empty link are
    #    re-parented to its parent with the composed origin (r4: the folded palm parts / sensor pads / tcp are empty links, some nested)
    while True:
        empties = [p for p in derived.findall('link') if not list(p)]
        if not empties: break
        link = empties[0]; name = link.get('name'); parents = [j for j in derived.findall('joint') if j.find('child').get('link') == name]
        assert len(parents) == 1 and parents[0].get('type') == 'fixed', name
        joint = parents[0]; xyz, rpy = _origin(joint); parent = joint.find('parent').get('link')
        for cj in [j for j in derived.findall('joint') if j.find('parent').get('link') == name]:
            assert cj.get('type') == 'fixed', (name, cj.get('name'))
            cx, cr = _origin(cj); nx, nr = _compose(xyz, rpy, cx, cr); cj.find('parent').set('link', parent); o = cj.find('origin')
            if o is None: o = ET.SubElement(cj, 'origin')
            o.set('xyz', ' '.join('%.9g' % v for v in nx)); o.set('rpy', ' '.join('%.9g' % v for v in nr))
        frames.append({'name': name, 'parent': parent, 'joint': joint.get('name'), 'xyz_m': xyz, 'rpy_rad': rpy, 'source_inertia_or_geometry': False, 'representation': 'nonphysical_coordinate_frame'})
        derived.remove(link); derived.remove(joint)
    physical = {p.get('name') for p in derived.findall('link')}
    assert all(f['parent'] in physical for f in frames), [f for f in frames if f['parent'] not in physical]
    names = {f['name'] for f in frames}
    assert NONPHYSICAL_LEAVES <= names and {'right_base', 'left_base'} <= names, sorted(names)
    for mesh in derived.iter('mesh'):
        filename = Path(mesh.get('filename'))
        if not filename.is_absolute():
            mesh.set('filename', str((source.parent / filename).resolve()))
    masses = {p.get('name'): float(p.find('inertial/mass').get('value')) for p in derived.findall('link')}
    assert len(masses) == len(derived.findall('link')), 'Every physical body needs explicit source inertia'
    ET.indent(derived); ET.ElementTree(derived).write(output, encoding='utf-8', xml_declaration=True)
    return {'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(), 'prepared_source_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
            'coordinate_frames': frames, 'folded_flange_frames': folded, 'physical_link_mass_kg': masses, 'source_physical_mass_kg': sum(masses.values()),
            'physical_mass_inertia_geometry_and_joint_origins_preserved': True, 'exact_RH56E2_equivalence_verified': False,
            'collision_model': collision_model_of(root)[1], 'collision_model_kind': collision_model_of(root)[0]}


def import_body_e2(source, output_dir, *, fixed_base):
    """Called after SimulationApp startup. Importer settings identical to inspire_body_asset.import_body; no palm/thumb builders."""
    import numpy as np
    import omni.kit.commands
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension('isaacsim.asset.importer.urdf')
    from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
    source, out = Path(source), Path(output_dir)
    prepared = out / 'assembled_physical.urdf'
    facts = prepare_source_e2(source, prepared)
    root = ET.parse(prepared).getroot()
    ok, cfg = omni.kit.commands.execute('URDFCreateImportConfig'); assert ok
    cfg.distance_scale = 1.; cfg.merge_fixed_joints = False; cfg.fix_base = fixed_base
    cfg.make_default_prim = True; cfg.create_physics_scene = False
    cfg.import_inertia_tensor = True; cfg.convex_decomp = True; cfg.self_collision = True; cfg.parse_mimic = True
    cfg.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_NONE
    dest = out / 'assembled_body.usd'
    ok, path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=str(prepared), import_config=cfg, dest_path=str(dest))
    assert ok and dest.is_file()
    stage = Usd.Stage.Open(str(dest)); prefix = str(stage.GetDefaultPrim().GetPath())
    masses = {p.GetName(): float(UsdPhysics.MassAPI(p).GetMassAttr().Get()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.MassAPI) and UsdPhysics.MassAPI(p).GetMassAttr().Get() is not None}
    assert set(masses) == set(facts['physical_link_mass_kg']), (set(masses) ^ set(facts['physical_link_mass_kg']))
    assert all(abs(masses[n] - m) < 1e-5 for n, m in facts['physical_link_mass_kg'].items())
    facts['imported_physical_mass_kg'] = sum(masses.values()); facts['imported_link_mass_kg'] = masses
    from urdf_kinematics import UrdfKinematics
    from rigid_inertia import source_properties, author_source_properties
    source_zero = UrdfKinematics(prepared).transforms({}); source_to_imported = {}
    for name in masses:
        prim = stage.GetPrimAtPath(prefix + '/' + name); assert prim
        row = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        imported = np.array([[float(row[i][j]) for j in range(4)] for i in range(4)]).T
        source_to_imported[name] = np.linalg.inv(imported) @ source_zero[name]
    facts['expected_source_rigid_properties_in_imported_frame'] = source_properties(prepared, source_to_imported)
    facts['source_inertial_frame_correction'] = author_source_properties(stage, prefix, facts['expected_source_rigid_properties_in_imported_frame'])
    for frame in facts['coordinate_frames']:
        parent = stage.GetPrimAtPath(prefix + '/' + frame['parent']); assert parent, frame
        f = UsdGeom.Xform.Define(stage, str(parent.GetPath()) + '/' + frame['name'])
        x, y, z = frame['rpy_rad']; cx, sx = np.cos(x), np.sin(x); cy, sy = np.cos(y), np.sin(y); cz, sz = np.cos(z), np.sin(z)
        rotation = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]) @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        m = Gf.Matrix4d(1.)
        for i in range(3):
            for j in range(3): m[i, j] = float(rotation[j, i])   # Gf row-vector convention
        m.SetTranslateOnly(Gf.Vec3d(*frame['xyz_m'])); f.AddTransformOp().Set(m); frame['usd_path'] = str(f.GetPath())
    joints = {j.get('name'): j for j in root.findall('joint') if j.get('type') == 'revolute'}
    hand_joints = {n: j for n, j in joints.items() if any('_' + finger + '_' in n for finger in FINGERS)}
    hand_independent = {n: j for n, j in hand_joints.items() if j.find('mimic') is None}
    assert len(joints) == 53 and len(hand_joints) == 24 and len(hand_independent) == 12, (len(joints), len(hand_joints), len(hand_independent))
    usd_joints = {p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)}
    assert set(usd_joints) == set(joints)
    for name, prim in usd_joints.items():
        source_joint = joints[name]; mimic = source_joint.find('mimic')
        if mimic is not None:
            assert prim.HasAPI(PhysxSchema.PhysxMimicJointAPI)
            axis = 'rot' + UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get()
            prim.CreateAttribute(f'physxMimicJoint:{axis}:naturalFrequency', Sdf.ValueTypeNames.Float).Set(0.)
        elif name in hand_independent:
            drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
            drive.CreateTypeAttr('force'); drive.CreateMaxForceAttr(float(source_joint.find('limit').get('effort')))
            drive.CreateStiffnessAttr(1.); drive.CreateDampingAttr(.05); drive.CreateTargetPositionAttr(0.)
    instance_roots = set()
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsInstanceProxy():
            ancestor = prim.GetParent()
            while ancestor and not ancestor.IsInstance(): ancestor = ancestor.GetParent()
            assert ancestor; instance_roots.add(str(ancestor.GetPath()))
    for p in sorted(instance_roots): stage.GetPrimAtPath(p).SetInstanceable(False)
    analytic = [{'prim': str(p.GetPath()), 'type': p.GetTypeName()} for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI) and p.GetTypeName() in {'Sphere', 'Cylinder', 'Capsule', 'Cube', 'Cone'}]
    assert len(analytic) == 12, 'the G1 body carries eight foot spheres and four shoulder cylinders'
    relocations = []
    for wrapper in [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI) and p.IsA(UsdGeom.Xform)]:
        meshes = [p for p in Usd.PrimRange(wrapper) if p.IsA(UsdGeom.Mesh)]; assert len(meshes) == 1
        mesh = meshes[0]; enabled = UsdPhysics.CollisionAPI(wrapper).GetCollisionEnabledAttr().Get(); approximation = UsdPhysics.MeshCollisionAPI(wrapper).GetApproximationAttr().Get()
        UsdPhysics.CollisionAPI.Apply(mesh).CreateCollisionEnabledAttr(enabled); UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr(approximation)
        wrapper.RemoveAPI(UsdPhysics.CollisionAPI); wrapper.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        relocations.append({'from': str(wrapper.GetPath()), 'to': str(mesh.GetPath()), 'approximation': approximation})
    hand_meshes = {}
    for side in ('right', 'left'):
        body = prefix + '/' + side + '_hand_base_link'; prim = stage.GetPrimAtPath(body); assert prim, body
        hand_meshes[side] = [str(p.GetPath()) for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)]
    facts.update(fixed_base=fixed_base, root_prim=prefix, collision_candidates={}, collision_api_relocations=relocations, thumb_collision_candidates={},
                 palm_collision_meshes_as_delivered=hand_meshes, source_analytic_colliders_preserved=analytic,
                 imported_inertia_validation='source tensors requested and expected frame-converted properties saved; live comparison requires physics initialization',
                 hand_joint_names=list(hand_joints), hand_independent_names=list(hand_independent), body_joint_names=[n for n in joints if n not in hand_joints],
                 mimic_map={n: {'parent': j.find('mimic').get('joint'), 'multiplier': float(j.find('mimic').get('multiplier', 1)), 'offset': float(j.find('mimic').get('offset', 0))} for n, j in hand_joints.items() if j.find('mimic') is not None},
                 joint_limits={n: {k: float(j.find('limit').get(k)) for k in ['lower', 'upper', 'effort', 'velocity']} for n, j in joints.items()},
                 hands_expected=True, parameter_provenance='public_exact_E2_prior_renesas_rdk_81bdb56_plus_pinned_Unitree_G1_body;PUBLIC_E2_PRIOR__INSTALLED_CALIBRATION_INCOMPLETE', hardware_calibration_verified=False)
    stage.GetRootLayer().Save()
    (out / 'assembled_asset.json').write_text(json.dumps(facts, indent=2, allow_nan=False))
    return dest, facts


def variant_b_urdf(merged_e2_urdf, donor_urdf, output, name_map=None, fold_record=None):
    """E2 geometry + FTP donor per-link masses: every E2 hand link takes the mass of its donor counterpart (the three E2 palm
    links share the donor's lumped base_link mass pro rata to their E2 masses); COM unchanged, inertia scaled with the mass.
    Returns the mapping record. Body links untouched."""
    src, dsrc, out = Path(merged_e2_urdf), Path(donor_urdf), Path(output)
    if out.exists():
        raise FileExistsError(out)
    e = ET.parse(src); er = e.getroot(); dr = ET.parse(dsrc).getroot()
    dmass = {l.get('name'): float(l.find('inertial/mass').get('value')) for l in dr.findall('link') if l.find('inertial') is not None}
    def donor_name(side, suffix):
        m = {'thumb_proximal': 'thumb_1', 'thumb_metacarpal': 'thumb_2', 'thumb_intermediate': 'thumb_3', 'thumb_distal': 'thumb_4'}
        for f_e, f_d in (('index', 'index'), ('middle', 'middle'), ('ring', 'ring'), ('pinky', 'little')):
            m[f_e + '_proximal'] = f_d + '_1'; m[f_e + '_intermediate'] = f_d + '_2'
            for k in ('1', '2', '3'): m[f_e + '_force_sensor_' + k] = f_d + '_force_sensor_' + k
        for k in ('1', '2', '3', '4'): m['thumb_force_sensor_' + k] = 'thumb_force_sensor_' + k
        m['palm_force_sensor'] = 'palm_force_sensor'
        return side + '_' + m[suffix] if suffix in m else None
    record = {'policy': 'E2_GEOMETRY_DONOR_LINK_MASSES (diagnostic variant B)', 'links': {}, 'hand_total_kg': {}, 'fold_record_applied': bool(fold_record)}
    # r4: folded sub-links (sensor pads, palm parts, tcp) carry no inertial; their DONOR masses are added to the donor counterpart of the body they were folded into
    extra = {}
    for side in ('right', 'left'):
        for child, rec in (fold_record or {}).get(side, {}).items():
            dn = donor_name(side, child[len(side) + 1:]); parent = rec['parent']
            # resolve the parent through the fold chain (palm_2 -> palm_1 -> hand_base_link)
            while parent in (fold_record or {}).get(side, {}): parent = fold_record[side][parent]['parent']
            if dn in dmass: extra[parent] = extra.get(parent, 0.0) + dmass[dn]
    for side in ('right', 'left'):
        palm = [n for n in (side + '_hand_base_link', side + '_palm_1', side + '_palm_2') if er.find("link[@name='%s']/inertial" % n) is not None]; pm = {n: float(er.find("link[@name='%s']/inertial/mass" % n).get('value')) for n in palm}; tot = sum(pm.values())
        donor_base = dmass[side + '_base_link'] + (extra.get(side + '_hand_base_link', 0.0) if len(palm) == 1 else 0.0)
        for l in er.findall('link'):
            n = l.get('name')
            if not n.startswith(side + '_') or l.find('inertial') is None: continue
            suffix = n[len(side) + 1:]
            if n in palm: new = donor_base * pm[n] / tot; src_name = side + '_base_link (pro rata %.4f)' % (pm[n] / tot)
            else:
                dn = donor_name(side, suffix)
                if dn is None or dn not in dmass: record['links'][n] = {'kept_e2_mass': float(l.find('inertial/mass').get('value')), 'reason': 'no donor counterpart'}; continue
                new = dmass[dn] + extra.get(n, 0.0); src_name = dn + (' + folded donor sensor masses %.4f' % extra[n] if n in extra else '')
            old = float(l.find('inertial/mass').get('value')); k = new / old; ine = l.find('inertial'); ine.find('mass').set('value', '%.9g' % new)
            I = ine.find('inertia')
            for key in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'): I.set(key, '%.9g' % (float(I.get(key)) * k))
            record['links'][n] = {'e2_mass_kg': old, 'variant_b_mass_kg': new, 'from': src_name, 'inertia_scale': k}
        record['hand_total_kg'][side] = round(sum(float(l.find('inertial/mass').get('value')) for l in er.findall('link') if l.get('name').startswith(side + '_') and l.find('inertial') is not None and not any(k in l.get('name') for k in ('shoulder', 'elbow', 'wrist', 'hip', 'knee', 'ankle'))), 6)
    er.set('name', er.get('name') + '_variantB_donor_masses'); ET.indent(e); e.write(out, encoding='utf-8', xml_declaration=True)
    record['output_sha256'] = hashlib.sha256(out.read_bytes()).hexdigest(); record['source_sha256'] = hashlib.sha256(src.read_bytes()).hexdigest()
    return record
