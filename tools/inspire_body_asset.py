"""Import the public FTP/G1 donor with explicit physical and frame ownership.

Sensor-only URDF frames carry no source inertial or geometry. Keep them as USD
frames attached to their source parent, rather than let the importer invent a
rigid-body mass. Source physical link mass/inertia and all collision pairs stay.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare_source(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    root = ET.parse(source).getroot()
    derived = deepcopy(root)
    empty = [p for p in derived.findall('link') if not list(p)]
    frames = []
    for link in empty:
        name = link.get('name')
        parents = [j for j in derived.findall('joint') if j.find('child').get('link') == name]
        assert len(parents) == 1 and parents[0].get('type') == 'fixed', name
        assert not any(j.find('parent').get('link') == name for j in derived.findall('joint')), name
        joint = parents[0]
        origin = joint.find('origin')
        frames.append({'name': name, 'parent': joint.find('parent').get('link'),
            'joint': joint.get('name'), 'xyz_m': [float(x) for x in origin.get('xyz','0 0 0').split()],
            'rpy_rad': [float(x) for x in origin.get('rpy','0 0 0').split()],
            'source_inertia_or_geometry': False, 'representation': 'nonphysical_coordinate_frame'})
        derived.remove(link)
        derived.remove(joint)
    assert {r['name'] for r in frames} == {'imu_in_torso','imu_in_pelvis','d435_link','mid360_link'}
    for mesh in derived.iter('mesh'):
        filename = Path(mesh.get('filename'))
        if not filename.is_absolute():
            mesh.set('filename', str((source.parent/filename).resolve()))
    physical_names = {p.get('name') for p in derived.findall('link')}
    masses = {p.get('name'): float(p.find('inertial/mass').get('value')) for p in derived.findall('link')}
    assert len(masses) == len(physical_names), 'Every physical body needs explicit source inertia'
    ET.indent(derived)
    ET.ElementTree(derived).write(output, encoding='utf-8', xml_declaration=True)
    return {'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'prepared_source_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
        'coordinate_frames': frames, 'physical_link_mass_kg': masses,
        'source_physical_mass_kg': sum(masses.values()),
        'physical_mass_inertia_geometry_and_joint_origins_preserved': True,
        'exact_RH56E2_equivalence_verified': False}


def import_body(source, output_dir, *, fixed_base, palm_builder, left_thumb_builder=None):
    """Called after SimulationApp startup; palm_builder owns candidate generation."""
    import numpy as np
    import omni.kit.commands
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension('isaacsim.asset.importer.urdf')
    from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
    source, out = Path(source), Path(output_dir)
    prepared = out/'assembled_physical.urdf'
    facts = prepare_source(source, prepared)
    root = ET.parse(prepared).getroot()
    ok,cfg = omni.kit.commands.execute('URDFCreateImportConfig')
    assert ok
    cfg.distance_scale=1.; cfg.merge_fixed_joints=False; cfg.fix_base=fixed_base
    cfg.make_default_prim=True; cfg.create_physics_scene=False
    cfg.import_inertia_tensor=True; cfg.convex_decomp=True; cfg.self_collision=True; cfg.parse_mimic=True
    cfg.default_drive_type=UrdfJointTargetType.JOINT_DRIVE_NONE
    dest=out/'assembled_body.usd'
    ok,path=omni.kit.commands.execute('URDFParseAndImportFile',urdf_path=str(prepared),import_config=cfg,dest_path=str(dest))
    assert ok and dest.is_file()
    stage=Usd.Stage.Open(str(dest)); prefix=str(stage.GetDefaultPrim().GetPath())
    masses={p.GetName():float(UsdPhysics.MassAPI(p).GetMassAttr().Get()) for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.MassAPI) and UsdPhysics.MassAPI(p).GetMassAttr().Get() is not None}
    assert set(masses)==set(facts['physical_link_mass_kg']), (set(masses)^set(facts['physical_link_mass_kg']))
    assert all(abs(masses[n]-m)<1e-5 for n,m in facts['physical_link_mass_kg'].items())
    facts['imported_physical_mass_kg']=sum(masses.values())
    facts['imported_link_mass_kg']=masses
    from urdf_kinematics import UrdfKinematics
    from rigid_inertia import source_properties,author_source_properties
    source_zero=UrdfKinematics(prepared).transforms({})
    source_to_imported={}
    for name in masses:
        prim=stage.GetPrimAtPath(prefix+'/'+name);assert prim
        row=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        imported=np.array([[float(row[i][j]) for j in range(4)] for i in range(4)]).T
        source_to_imported[name]=np.linalg.inv(imported)@source_zero[name]
    facts['expected_source_rigid_properties_in_imported_frame']=source_properties(prepared,source_to_imported)
    facts['source_inertial_frame_correction']=author_source_properties(stage,prefix,facts['expected_source_rigid_properties_in_imported_frame'])
    for frame in facts['coordinate_frames']:
        parent=stage.GetPrimAtPath(prefix+'/'+frame['parent']); assert parent
        f=UsdGeom.Xform.Define(stage,str(parent.GetPath())+'/'+frame['name'])
        x,y,z=frame['rpy_rad']; cx,sx=np.cos(x),np.sin(x);cy,sy=np.cos(y),np.sin(y);cz,sz=np.cos(z),np.sin(z)
        rotation=np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])@np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])@np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
        # Gf stores row-vector transforms; URDF rotation above acts on columns.
        m=Gf.Matrix4d(1.)
        for i in range(3):
            for j in range(3): m[i,j]=float(rotation[j,i])
        m.SetTranslateOnly(Gf.Vec3d(*frame['xyz_m']))
        f.AddTransformOp().Set(m)
        frame['usd_path']=str(f.GetPath())
    joints={j.get('name'):j for j in root.findall('joint') if j.get('type')=='revolute'}
    hand_joints={n:j for n,j in joints.items() if any('_'+finger+'_' in n for finger in ['thumb','index','middle','ring','little'])}
    hand_independent={n:j for n,j in hand_joints.items() if j.find('mimic') is None}
    assert len(joints)==53 and len(hand_joints)==24 and len(hand_independent)==12
    usd_joints={p.GetName():p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)}
    assert set(usd_joints)==set(joints)
    for name,prim in usd_joints.items():
        source_joint=joints[name]; mimic=source_joint.find('mimic')
        if mimic is not None:
            assert prim.HasAPI(PhysxSchema.PhysxMimicJointAPI)
            axis='rot'+UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get()
            prim.CreateAttribute(f'physxMimicJoint:{axis}:naturalFrequency',Sdf.ValueTypeNames.Float).Set(0.)
        elif name in hand_independent:
            drive=UsdPhysics.DriveAPI.Apply(prim,'angular')
            drive.CreateTypeAttr('force');drive.CreateMaxForceAttr(float(source_joint.find('limit').get('effort')))
            drive.CreateStiffnessAttr(1.);drive.CreateDampingAttr(.05);drive.CreateTargetPositionAttr(0.)
    instance_roots=set()
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsInstanceProxy():
            ancestor=prim.GetParent()
            while ancestor and not ancestor.IsInstance():ancestor=ancestor.GetParent()
            assert ancestor
            instance_roots.add(str(ancestor.GetPath()))
    for p in sorted(instance_roots):stage.GetPrimAtPath(p).SetInstanceable(False)
    analytic=[{'prim':str(p.GetPath()),'type':p.GetTypeName()} for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.CollisionAPI) and p.GetTypeName() in {'Sphere','Cylinder','Capsule','Cube','Cone'}]
    assert len(analytic)==12, 'Pinned donor has eight source foot spheres and four shoulder cylinders'
    relocations=[]
    for wrapper in [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI) and p.IsA(UsdGeom.Xform)]:
        meshes=[p for p in Usd.PrimRange(wrapper) if p.IsA(UsdGeom.Mesh)]
        assert len(meshes)==1
        mesh=meshes[0]; enabled=UsdPhysics.CollisionAPI(wrapper).GetCollisionEnabledAttr().Get()
        approximation=UsdPhysics.MeshCollisionAPI(wrapper).GetApproximationAttr().Get()
        UsdPhysics.CollisionAPI.Apply(mesh).CreateCollisionEnabledAttr(enabled)
        UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr(approximation)
        wrapper.RemoveAPI(UsdPhysics.CollisionAPI);wrapper.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        relocations.append({'from':str(wrapper.GetPath()),'to':str(mesh.GetPath()),'approximation':approximation})
    candidates={}
    for side in ['right','left']:
        palm=prefix+'/'+side+'_base_link'
        meshes=[p for p in Usd.PrimRange(stage.GetPrimAtPath(palm)) if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)]
        assert len(meshes)==1
        candidates[side]=palm_builder(stage,str(meshes[0].GetPath()),palm,side)
    thumbs={}
    if left_thumb_builder is not None:
        body=prefix+'/left_thumb_2'
        meshes=[p for p in Usd.PrimRange(stage.GetPrimAtPath(body)) if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)]
        assert len(meshes)==1
        thumbs['left']=left_thumb_builder(stage,str(meshes[0].GetPath()),body)
    facts.update(fixed_base=fixed_base,root_prim=prefix,collision_candidates=candidates,collision_api_relocations=relocations,
        thumb_collision_candidates=thumbs,
        source_analytic_colliders_preserved=analytic,
        imported_inertia_validation='source tensors requested and expected frame-converted properties saved; live comparison requires physics initialization',
        hand_joint_names=list(hand_joints),hand_independent_names=list(hand_independent),body_joint_names=[n for n in joints if n not in hand_joints],
        mimic_map={n:{'parent':j.find('mimic').get('joint'),'multiplier':float(j.find('mimic').get('multiplier',1)),
            'offset':float(j.find('mimic').get('offset',0))} for n,j in hand_joints.items() if j.find('mimic') is not None},
        joint_limits={n:{k:float(j.find('limit').get(k)) for k in ['lower','upper','effort','velocity']} for n,j in joints.items()},
        parameter_provenance='public_pinned_Unitree_FTP_donor_plus_declared_provisional_collider',hardware_calibration_verified=False)
    stage.GetRootLayer().Save()
    (out/'assembled_asset.json').write_text(json.dumps(facts,indent=2,allow_nan=False))
    return dest,facts
