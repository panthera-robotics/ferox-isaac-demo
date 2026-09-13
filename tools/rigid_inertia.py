"""Compare source and runtime rigid-body inertia in the same body frame.

Inertia is at the centre of mass; changing its orientation does not require a
parallel-axis term. This audit never rewrites physics properties.
"""
import xml.etree.ElementTree as ET
import numpy as np


def rpy_matrix(rpy):
    x,y,z=map(float,rpy);cx,sx=np.cos(x),np.sin(x);cy,sy=np.cos(y),np.sin(y);cz,sz=np.cos(z),np.sin(z)
    return np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])@np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])@np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])


def source_properties(source, source_link_to_runtime_link):
    """Transforms map source-link column vectors into runtime-link coordinates."""
    rows={}
    for link in ET.parse(source).getroot().findall('link'):
        inertial=link.find('inertial')
        if inertial is None:continue
        name=link.get('name');t=np.asarray(source_link_to_runtime_link[name],dtype=float)
        if t.shape!=(4,4) or not np.isfinite(t).all():raise ValueError('Invalid body transform')
        if not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6):raise ValueError('Nonrigid body transform')
        origin=inertial.find('origin')
        xyz=np.array([0.,0.,0.] if origin is None else list(map(float,origin.get('xyz','0 0 0').split())))
        rpy=[0.,0.,0.] if origin is None else list(map(float,origin.get('rpy','0 0 0').split()))
        a=inertial.find('inertia').attrib
        inertia=np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],
                          [float(a['ixy']),float(a['iyy']),float(a['iyz'])],
                          [float(a['ixz']),float(a['iyz']),float(a['izz'])]])
        rotation=t[:3,:3]@rpy_matrix(rpy)
        rotated=rotation@inertia@rotation.T
        if not np.isfinite(rotated).all() or np.linalg.eigvalsh(rotated).min()<=0:raise ValueError('Invalid source inertia')
        rows[name]={'mass_kg':float(inertial.find('mass').get('value')),
            'com_runtime_link_m':(t@np.r_[xyz,1.])[:3].tolist(),
            'inertia_at_com_runtime_link_kg_m2':rotated.tolist()}
    return rows


def compare_properties(expected, measured):
    """Fixed pre-evaluation budgets cover float32 storage, not mass tuning."""
    if set(expected)!=set(measured):raise ValueError('Named body set mismatch')
    rows={}
    for name,want in expected.items():
        got=measured[name]
        arrays=[np.asarray(got[k],dtype=float) for k in ['mass_kg','com_runtime_link_m','inertia_at_com_runtime_link_kg_m2']]
        if not all(np.isfinite(a).all() for a in arrays):raise ValueError('Nonfinite runtime property')
        mass_error=abs(float(got['mass_kg'])-want['mass_kg'])
        com_error=float(np.linalg.norm(np.asarray(got['com_runtime_link_m'])-want['com_runtime_link_m']))
        difference=np.asarray(got['inertia_at_com_runtime_link_kg_m2'])-want['inertia_at_com_runtime_link_kg_m2']
        tensor_error=float(np.max(np.abs(difference)))
        tensor_budget=max(1e-8,1e-3*float(np.max(np.abs(want['inertia_at_com_runtime_link_kg_m2']))))
        rows[name]={'mass_error_kg':mass_error,'com_error_m':com_error,'inertia_max_element_error_kg_m2':tensor_error,
            'inertia_element_budget_kg_m2':tensor_budget,'pass':bool(mass_error<=1e-5 and com_error<=1e-6 and tensor_error<=tensor_budget)}
    return {'checks':{'all_named_source_mass_com_inertia_preserved':all(r['pass'] for r in rows.values())},
        'budgets':{'mass_absolute_kg':1e-5,'com_distance_m':1e-6,'inertia_element_absolute_floor_kg_m2':1e-8,'inertia_relative_max_element':1e-3},
        'expected':expected,'measured':measured,'comparison':rows,
        'frame':'inertia_at_COM_expressed_in_imported_body_prim_frame','properties_modified':False}


def audit_live_properties(sim_view, root_path, expected):
    measured={}
    for name in expected:
        view=sim_view.create_rigid_body_view(root_path+'/'+name)
        if view.count!=1:raise ValueError('Missing or ambiguous physical body '+name)
        # PhysX tensor API returns column-major 3x3 inertia in the body frame.
        measured[name]={'mass_kg':float(np.asarray(view.get_masses()).reshape(-1)[0]),
            'com_runtime_link_m':np.asarray(view.get_coms())[0,:3].tolist(),
            'inertia_at_com_runtime_link_kg_m2':np.asarray(view.get_inertias())[0].reshape(3,3,order='F').tolist()}
    return compare_properties(expected,measured)


def author_source_properties(stage,root_path,expected):
    """Repair importer frame-rounding without fitting or rescaling source inertia.

    Some imported body axes differ slightly from source axes while MassAPI COM
    coordinates are copied verbatim. Transform the source properties into the
    actual imported frame. Preserve this explicit correction in the asset receipt.
    """
    from pxr import Gf,UsdPhysics
    changes={}
    for name,properties in expected.items():
        prim=stage.GetPrimAtPath(root_path+'/'+name)
        if not prim or not prim.HasAPI(UsdPhysics.MassAPI):raise ValueError('Missing source mass API '+name)
        api=UsdPhysics.MassAPI(prim)
        prior={a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.GetName() in
            {'physics:mass','physics:centerOfMass','physics:diagonalInertia','physics:principalAxes'}}
        tensor=np.asarray(properties['inertia_at_com_runtime_link_kg_m2'],dtype=float)
        values,vectors=np.linalg.eigh(tensor)
        if values.min()<=0:raise ValueError('Invalid source tensor')
        if np.linalg.det(vectors)<0:vectors[:,0]*=-1
        quaternion=Gf.Matrix3d(*vectors.T.flatten().tolist()).ExtractRotation().GetQuat()
        api.GetMassAttr().Set(float(properties['mass_kg']))
        api.GetCenterOfMassAttr().Set(Gf.Vec3f(*properties['com_runtime_link_m']))
        api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*map(float,values)))
        api.GetPrincipalAxesAttr().Set(Gf.Quatf(quaternion))
        changes[name]={'importer_authored_before':prior,'source_properties_in_imported_frame':properties}
    return {'reason':'preserve_source_world_COM_and_inertia_after_importer_body_frame_reorientation',
        'source_mass_scaled':False,'measured_hardware_calibration':False,'source_geometry_or_mount_changed':False,
        'links':changes}
