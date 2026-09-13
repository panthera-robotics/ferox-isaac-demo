"""Fixed-pelvis assembled donor, physical mechanism and independent FK check.

No grasp, writing or standing acceptance. A private, hashed simulator profile
supplies named body home and gains. This file contains no private calibration.
"""
import hashlib,json,os,sys,time
from pathlib import Path
assert os.environ.get('PANTHERA_SIM_AUTHORIZED')=='1'
assert sorted(p.name for p in Path('/sys/class/net').iterdir())==['lo']
assert os.environ.get('PANTHERA_PROBE_MODE')=='assembled-fixed-home'
profile_path=Path(os.environ['PANTHERA_PROBE_CONFIG']);profile=json.loads(profile_path.read_text())
assert profile['schema_version']==1 and profile['hardware_authorized'] is False
out=Path('/evidence');sys.path.insert(0,'/workspace/ferox_tools')
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting'})
import numpy as np
from PIL import Image
from pxr import Gf,PhysxSchema,PhysicsSchemaTools,Usd,UsdGeom,UsdLux,UsdPhysics
from omni.physx import get_physx_simulation_interface,get_physxunittests_interface
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.camera import Camera
enable_extension('omni.pip.compute')
from inspire_body_asset import import_body
from inspire_collision import replace_palm_with_components
from urdf_kinematics import UrdfKinematics
from rigid_inertia import audit_live_properties
source=Path('/source-assets/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf')
def palm_builder(stage,mesh,body,side):
    return replace_palm_with_components(stage,mesh,body,contact_offset_m=.0012860533315688372,rest_offset_m=0.,
        candidate_id='ftp_palm_yz_slabs_v2' if side=='right' else 'ftp_left_palm_yz_slabs_v1')
asset,facts=import_body(source,out,fixed_base=True,palm_builder=palm_builder)
body_names=list(profile['body_home_rad']);assert set(body_names)==set(facts['body_joint_names']) and len(body_names)==29
for key in ['kp_nm_rad','kd_nm_s_rad']:
    assert set(profile[key])==set(body_names)
    assert all(np.isfinite(v) and v>=0 for v in profile[key].values())
assert all(np.isfinite(v) and facts['joint_limits'][n]['lower']<=v<=facts['joint_limits'][n]['upper'] for n,v in profile['body_home_rad'].items())
steps=profile['steps'];assert type(steps) is int and 400<=steps<=2000
stage=Usd.Stage.Open(str(asset))
for prim in stage.Traverse():
    n=prim.GetName()
    if prim.IsA(UsdPhysics.RevoluteJoint) and n in body_names:
        drive=UsdPhysics.DriveAPI.Apply(prim,'angular');drive.CreateTypeAttr('force')
        drive.CreateMaxForceAttr(facts['joint_limits'][n]['effort'])
        drive.CreateStiffnessAttr(profile['kp_nm_rad'][n]);drive.CreateDampingAttr(profile['kd_nm_s_rad'][n])
        drive.CreateTargetPositionAttr(float(np.rad2deg(profile['body_home_rad'][n])))
stage.GetRootLayer().Save()
world=World(stage_units_in_meters=1.,physics_dt=.005,rendering_dt=.02)
scene=next(p for p in world.stage.Traverse() if p.IsA(UsdPhysics.Scene))
PhysxSchema.PhysxSceneAPI.Apply(scene).CreateEnableExternalForcesEveryIterationAttr(True)
light=UsdLux.DomeLight.Define(world.stage,'/World/Fill');light.CreateIntensityAttr(350.);light.CreateColorAttr(Gf.Vec3f(.45,.52,.65))
key=UsdLux.DistantLight.Define(world.stage,'/World/Key');key.CreateIntensityAttr(500.)
UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-35,-25,-40))
add_reference_to_stage(str(asset),'/World/G1')
root=UsdGeom.Xformable(world.stage.GetPrimAtPath('/World/G1'));root.ClearXformOpOrder();root.AddTranslateOp().Set(Gf.Vec3d(0,0,1.))
for p in world.stage.Traverse():
    if p.HasAPI(PhysxSchema.PhysxArticulationAPI):
        api=PhysxSchema.PhysxArticulationAPI(p);api.CreateSolverPositionIterationCountAttr(32);api.CreateSolverVelocityIterationCountAttr(8)
    if p.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
contacts=[];trace=[];phase='initialization'
contact_file=(out/'contacts.jsonl').open('w',buffering=1)
def on_contact(headers,data):
    for h in headers:
        for k in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
            d=data[k];r={'sequence':None if phase=='initialization' else len(trace),'physics_s':world.current_time,'phase':phase,
                'actor0':str(PhysicsSchemaTools.intToSdfPath(h.actor0)),'actor1':str(PhysicsSchemaTools.intToSdfPath(h.actor1)),
                'position_world_m':list(map(float,d.position)),'normal_world':list(map(float,d.normal)),
                'impulse_ns':list(map(float,d.impulse)),'separation_m':float(d.separation),'source':'simulated_proxy'}
            contacts.append(r);contact_file.write(json.dumps(r,allow_nan=False)+'\n')
subscription=get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
robot=SingleArticulation('/World/G1',name='provisional_assembled_g1')
world.reset();robot.initialize()
inertia_audit=audit_live_properties(SimulationManager.get_physics_sim_view(),'/World/G1',facts['expected_source_rigid_properties_in_imported_frame'])
(out/'live_inertia_audit.json').write_text(json.dumps(inertia_audit,indent=2,allow_nan=False))
names=list(robot.dof_names);assert set(names)==set(facts['joint_limits']) and len(names)==53
body_ids=np.asarray([names.index(n) for n in body_names],dtype=np.int32)
hand_names=facts['hand_independent_names'];hand_ids=np.asarray([names.index(n) for n in hand_names],dtype=np.int32)
kp=np.zeros(53,dtype=np.float32);kd=np.zeros(53,dtype=np.float32)
kp[body_ids]=[profile['kp_nm_rad'][n] for n in body_names];kd[body_ids]=[profile['kd_nm_s_rad'][n] for n in body_names]
kp[hand_ids]=1.;kd[hand_ids]=.05
robot._articulation_view.set_gains(kp,kd)
got=robot.get_articulation_controller().get_gains();assert np.allclose(np.ravel(got[0]),kp) and np.allclose(np.ravel(got[1]),kd)
q0=np.zeros(53,dtype=np.float32);q0[body_ids]=[profile['body_home_rad'][n] for n in body_names]
# Initial episode state only. No joint/base pose writes occur during evaluation.
robot.set_joint_positions(q0);robot.set_joint_velocities(np.zeros(53,dtype=np.float32))
views={n:SimulationManager.get_physics_sim_view().create_rigid_body_view('/World/G1/'+n) for n in ['pelvis','torso_link','right_wrist_yaw_link','left_wrist_yaw_link','right_base_link','left_base_link']}
assert all(v.count==1 for v in views.values())
shapes={'kind':'live_physx_tensor_shapes','statistics':get_physxunittests_interface().get_physics_stats(),'palms':{}}
for side in ['right','left']:
    v=views[side+'_base_link'];expected=facts['collision_candidates'][side]['expected_palm_hulls_if_all_cooking_succeeds']
    shapes['palms'][side]={'live_shape_count':v.max_shapes,'expected_shape_count':expected,
        'contact_offsets_m':np.asarray(v.get_contact_offsets()).tolist(),'rest_offsets_m':np.asarray(v.get_rest_offsets()).tolist()}
    assert v.max_shapes==expected, (side,v.max_shapes,expected)
assert shapes['statistics']['numTriMeshShapes']==0
(out/'backend_shapes.json').write_text(json.dumps(shapes,indent=2,allow_nan=False))
cameras={}
for label,position,target in [('front',(2.2,-2.3,1.8),(.1,0,1.05)),('side',(-.3,2.9,1.65),(.1,0,1.05))]:
    camera=Camera('/World/'+label+'Camera',resolution=(640,640));camera.initialize();camera.set_clipping_range(.01,10.)
    x=UsdGeom.Xformable(camera.prim);x.ClearXformOpOrder();x.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*position),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse())
    cameras[label]=camera;(out/'frames'/label).mkdir(parents=True)
frame_file=(out/'frames.jsonl').open('w',buffering=1);state_file=(out/'state.jsonl').open('w',buffering=1)
kinematics=UrdfKinematics(source)
def transform(pose):
    x,y,z,w=pose[3:];r=np.eye(4)
    r[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
               [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
               [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    r[:3,3]=pose[:3];return r
phase='fixed_home_hold'
aborted=None
for tick in range(steps):
    robot.apply_action(ArticulationAction(joint_positions=q0[body_ids],joint_indices=body_ids))
    robot.apply_action(ArticulationAction(joint_positions=np.zeros(12,dtype=np.float32),joint_indices=hand_ids))
    world.step(render=False)
    if (tick+1)%4==0:world.render()
    q=np.ravel(robot.get_joint_positions());dq=np.ravel(robot.get_joint_velocities());effort=np.ravel(robot.get_measured_joint_efforts())
    if not (np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(effort).all()):
        # Keep rejected values without invalid JSON or feeding NaN into FK.
        aborted={'sequence':tick,'reason':'nonfinite_physics_state','q_rad':[repr(float(v)) for v in q],
            'dq_rad_s':[repr(float(v)) for v in dq],'measured_effort_nm':[repr(float(v)) for v in effort]}
        break
    poses={n:np.asarray(v.get_transforms())[0].tolist() for n,v in views.items()}
    pelvis=transform(poses['pelvis']);source_fk=kinematics.transforms(dict(zip(names,map(float,q))),pelvis)
    errors={}
    for n in ['right_wrist_yaw_link','left_wrist_yaw_link','right_base_link','left_base_link']:
        actual=transform(poses[n]);expected=source_fk[n];d=expected[:3,:3].T@actual[:3,:3]
        errors[n]={'translation_m':float(np.linalg.norm(actual[:3,3]-expected[:3,3])),
            'rotation_rad':float(np.arccos(np.clip((np.trace(d)-1)/2,-1,1)))}
    coupling={n:float(q[names.index(n)]-(m['multiplier']*q[names.index(m['parent'])]+m['offset'])) for n,m in facts['mimic_map'].items()}
    r={'sequence':tick,'physics_s':world.current_time,'phase':phase,'runtime_names':names,'q_rad':q.tolist(),'dq_rad_s':dq.tolist(),
       'measured_generalized_effort_nm':effort.tolist(),'body_command_names':body_names,'body_command_rad':q0[body_ids].tolist(),
       'body_command_owners':{'all29':'fixed_home_position_fixture'},'hand_command_names':hand_names,'hand_command_rad':[0.]*12,
       'link_poses_world_xyzw':poses,'source_fk_error':errors,'coupling_error_rad':coupling}
    trace.append(r);state_file.write(json.dumps(r,allow_nan=False)+'\n')
    # Diagnostic stop envelope is separate from (and looser than) acceptance.
    # Stop at the first clear explosion, retaining this finite failed sample.
    violated={n:float(q[i]) for i,n in enumerate(names) if q[i]<facts['joint_limits'][n]['lower']-.1 or q[i]>facts['joint_limits'][n]['upper']+.1}
    overspeed={n:float(dq[i]) for i,n in enumerate(names) if abs(dq[i])>2*facts['joint_limits'][n]['velocity']}
    if violated or overspeed:
        aborted={'sequence':tick,'reason':'source_envelope_abort','joint_limit_violations_rad':violated,'joint_velocity_violations_rad_s':overspeed}
        break
    if (tick+1)%20==0:
        frame=(tick+1)//20-1;files={}
        for label,camera in cameras.items():
            pixels=camera.get_rgba();assert pixels is not None and pixels.shape==(640,640,4)
            f=f'frames/{label}/{frame:06d}.png';Image.fromarray(pixels.astype(np.uint8)).save(out/f);files[label]=f
        frame_file.write(json.dumps({'frame':frame,'sequence':tick,'physics_s':world.current_time,'phase':phase,
            'captured_after_same_step_render':True,'views':files})+'\n')
state_file.close();contact_file.close();frame_file.close()
max_position=max((e['translation_m'] for r in trace for e in r['source_fk_error'].values()),default=0.)
max_rotation=max((e['rotation_rad'] for r in trace for e in r['source_fk_error'].values()),default=0.)
max_coupling=max((abs(e) for r in trace for e in r['coupling_error_rad'].values()),default=0.)
limits=max((max(lim['lower']-r['q_rad'][names.index(n)],r['q_rad'][names.index(n)]-lim['upper'],0.) for n,lim in facts['joint_limits'].items() for r in trace),default=0.)
checks={'exact53_named_coordinates':len(names)==53,'physical_mass_preserved':abs(facts['source_physical_mass_kg']-facts['imported_physical_mass_kg'])<1e-4,
    'fixed_pelvis_matches_declared_pose':bool(all(np.linalg.norm(np.asarray(r['link_poses_world_xyzw']['pelvis'][:3])-[0,0,1])<1e-4 for r in trace)),
    'source_fk_translation_below_0_2mm':max_position<=.0002,'source_fk_rotation_below_0_2deg':max_rotation<=np.deg2rad(.2),
    'hand_coupling_below_0_03rad':max_coupling<.03,'joint_limits_below_0_03rad':limits<.03,
    'exact_steps':len(trace)==steps,'numerical_abort_absent':aborted is None,'source_mass_com_inertia_preserved':all(inertia_audit['checks'].values()),
    'physics_dt':len(trace)>1 and bool(np.allclose(np.diff([r['physics_s'] for r in trace]),.005,atol=1e-8)),
    'no_static_hand_triangles':shapes['statistics']['numTriMeshShapes']==0,'bilateral_live_palm_shape_counts':all(v['live_shape_count']==v['expected_shape_count'] for v in shapes['palms'].values())}
metrics={'checks':checks,'steps':len(trace),'physics_dt':.005,'runtime_names':names,'source_model':'Unitree_FTP_G1_provisional_donor',
    'source_mass_kg':facts['source_physical_mass_kg'],'profile_sha256':hashlib.sha256(profile_path.read_bytes()).hexdigest(),
    'fixed_base':True,'support_constraints':['pelvis_fixed_to_world_1m_above_origin'],'ground_present':False,
    'measured_initial_pelvis_pose_world_xyzw':trace[0]['link_poses_world_xyzw']['pelvis'] if trace else None,
    'abort':aborted,'source_property_audit':'live_inertia_audit.json',
    'exact_asset_qualified':False,'tool_attached':False,'grasp_qualification':'NOT_RUN','writing_qualification':'NOT_RUN','standing_qualification':'NOT_RUN',
    'max_source_fk_position_error_m':max_position,'max_source_fk_rotation_error_rad':max_rotation,
    'coupling_error_max_rad':max_coupling,'joint_limit_violation_rad':limits,'actual_contact_points':len(contacts),
    'media_labels':{'fixture':'FIXED PELVIS - gravity loaded assembled-body check','embodiment':'PROVISIONAL G1 + bilateral FTP hands','qualification':'Physical frame/mechanism check only; no grasp, writing or standing qualification'},
    'controller':'one_named29_position_owner_plus12_hand_root_drives','body_gains_source':profile['gain_provenance'],
    'body_target_error_max_rad':float(max((abs(r['q_rad'][names.index(n)]-profile['body_home_rad'][n]) for r in trace for n in body_names),default=0.))}
(out/'metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False))
world.stage.GetRootLayer().Export(str(out/'assembled_scene.usda'))
artifacts=[str(p.relative_to(out)) for p in out.rglob('*') if p.is_file() and p.name not in ['run.json','probe.json','console.log','executed_probe.py','executed_launcher.py','uncommitted.patch']]
(out/'probe.json').write_text(json.dumps({'status':'PASS' if all(checks.values()) else 'FAIL','scope':'provisional_assembled_fixed_pelvis_mechanism_and_frame_check','metrics':'metrics.json','artifacts':artifacts}))
app.close()
