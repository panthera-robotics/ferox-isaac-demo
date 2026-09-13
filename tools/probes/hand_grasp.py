"""Freely dynamic preloaded marker retention in a supported provisional donor hand.

No rack acquisition, object attachment, kinematic marker, object-follow controller,
or object pose writes after reset. Initial preload is explicitly authored before
physics. Only six finger drives and six declared wrist-fixture drives are actuated.
"""
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import sys
import traceback
import xml.etree.ElementTree as ET


def main():
    assert os.environ.get('PANTHERA_SIM_AUTHORIZED')=='1'
    assert sorted(p.name for p in Path('/sys/class/net').iterdir())==['lo']
    sys.path[:0]=['/workspace/ferox_tools','/workspace/ferox_isaac']
    from inspire_grasp import GraspConfig,relative_measurement,drift,wrist_target,retention_result
    from inspire_asset import audit_urdf
    from inspire_wrist_fixture import write_wrist_fixture
    from urdf_kinematics import UrdfKinematics
    from twin.inspire.whiteboard_scene import SceneConfig,BoardFrame,build_scene,rotate,compression_from_poses
    mode=os.environ.get('PANTHERA_PROBE_MODE','preloaded-close-hold')
    assert mode in {'preloaded-free-control','preloaded-close-hold','preloaded-retention-60s'}
    empty_control=mode=='preloaded-free-control'
    cfg=GraspConfig.from_dict(json.loads(Path(os.environ['PANTHERA_PROBE_CONFIG']).read_text()) if os.environ.get('PANTHERA_PROBE_CONFIG') else {})
    out=Path('/evidence');app=None
    def write(name,data):out.joinpath(name).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    scope={'holder_attachment_active':False,'holder_fixture_support_active':False,'wrist_fixed_support':True,
           'initialization':'declared_geometric_preload_before_reset','rack_acquisition_qualified':False,
           'exact_E2_qualified':False,'standing_qualified':False,'writing_qualified':False,
           'source':'provisional_FTP_right_donor_not_verified_RH56E2_bytes'}
    write('grasp_config.json',{'config':asdict(cfg),'config_sha256':cfg.sha256,'scope':scope,'mode':mode})
    try:
        source=Path('/source-assets/FTP_right_hand_bench.urdf');facts=audit_urdf(source);root=ET.parse(source).getroot()
        independent=[j.get('name') for j in root.findall('joint') if j.get('type')=='revolute' and j.find('mimic') is None]
        mimics={j.get('name'):{'parent':j.find('mimic').get('joint'),'multiplier':float(j.find('mimic').get('multiplier',1)),
                             'offset':float(j.find('mimic').get('offset',0))} for j in root.findall('joint') if j.find('mimic') is not None}
        limits={j.get('name'):[float(j.find('limit').get(k)) for k in ['lower','upper']] for j in root.findall('joint') if j.get('type')=='revolute'}
        assert len(independent)==6 and len(mimics)==6 and set(independent)==set(cfg.initial_targets())
        initial_q=cfg.initial_targets();target_q=cfg.closed_targets(limits)
        def position(name):
            if name in initial_q:return initial_q[name]
            m=mimics[name];return position(m['parent'])*m['multiplier']+m['offset']
        initial_all={n:position(n) for n in limits}
        for n,q in initial_all.items():assert limits[n][0]-1e-8<=q<=limits[n][1]+1e-8
        fixture=write_wrist_fixture(source,out/'supported_wrist.urdf')
        kinematics=UrdfKinematics(out/'supported_wrist.urdf');initial_fk=kinematics.transforms(initial_all)
        write('preload_geometry.json',{'initial_independent_rad':initial_q,'initial_all_rad':initial_all,'closing_targets_rad':target_q,
              'sensor_origins_palm_m':{n:t[:3,3].tolist() for n,t in initial_fk.items() if 'force_sensor' in n},
              'holder_center_palm_m':cfg.holder_center_palm_m,'holder_orientation_palm_qwxyz':cfg.holder_orientation_palm_qwxyz,
              'candidate_derivation':'source FK plus deterministic sampled-mesh/cylinder CPU search; not a collision proof',
              'fixture':fixture,'source_audit':facts})
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'renderer':'RaytracedLighting'})
        import numpy as np
        from PIL import Image
        import omni.kit.commands
        from pxr import Gf,Sdf,Usd,UsdGeom,UsdLux,UsdShade,UsdPhysics,PhysxSchema,PhysicsSchemaTools
        from omni.physx import get_physx_simulation_interface,get_physxunittests_interface
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.sensors.camera import Camera
        enable_extension('isaacsim.asset.importer.urdf');enable_extension('omni.pip.compute')
        from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
        ok,imp=omni.kit.commands.execute('URDFCreateImportConfig');assert ok
        imp.distance_scale=1.;imp.merge_fixed_joints=False;imp.fix_base=True;imp.make_default_prim=True
        imp.create_physics_scene=False;imp.import_inertia_tensor=True;imp.convex_decomp=True
        imp.self_collision=True;imp.parse_mimic=True;imp.default_drive_type=UrdfJointTargetType.JOINT_DRIVE_NONE
        imported=out/'ftp_preloaded.usd'
        ok,_=omni.kit.commands.execute('URDFParseAndImportFile',urdf_path=str(out/'supported_wrist.urdf'),import_config=imp,dest_path=str(imported));assert ok
        stage=Usd.Stage.Open(str(imported))
        mass=sum(float(UsdPhysics.MassAPI(p).GetMassAttr().Get()) for p in stage.Traverse()
                 if p.HasAPI(UsdPhysics.MassAPI) and UsdPhysics.MassAPI(p).GetMassAttr().Get() is not None)
        source_mass=sum(float(m.get('value')) for m in root.iter('mass'))
        assert abs(mass-source_mass-fixture['fixture_added_mass_kg'])<.001
        instances=set()
        for p in stage.Traverse(Usd.TraverseInstanceProxies()):
            if p.HasAPI(UsdPhysics.CollisionAPI) and p.IsInstanceProxy():
                a=p.GetParent()
                while a and not a.IsInstance():a=a.GetParent()
                assert a;instances.add(str(a.GetPath()))
        for path in instances:stage.GetPrimAtPath(path).SetInstanceable(False)
        wrappers=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI) and not p.IsA(UsdGeom.Mesh)]
        assert len(wrappers)==30
        for wrapper in wrappers:
            meshes=[p for p in Usd.PrimRange(wrapper) if p.IsA(UsdGeom.Mesh)];assert len(meshes)==1
            UsdPhysics.CollisionAPI.Apply(meshes[0]).CreateCollisionEnabledAttr(UsdPhysics.CollisionAPI(wrapper).GetCollisionEnabledAttr().Get())
            UsdPhysics.MeshCollisionAPI.Apply(meshes[0]).CreateApproximationAttr('convexDecomposition')
            wrapper.RemoveAPI(UsdPhysics.CollisionAPI);wrapper.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        from inspire_collision import replace_palm_with_components
        candidate=replace_palm_with_components(stage,'/Rhand/right_base_link/collisions/right_base_link/node_STL_BINARY_/mesh',
            '/Rhand/right_base_link',contact_offset_m=.0012860533315688372,rest_offset_m=0.,candidate_id=cfg.palm_candidate_id)
        write('collision_candidate.json',candidate)
        for p in stage.Traverse():
            if p.GetName() in limits and p.IsA(UsdPhysics.RevoluteJoint):
                name=p.GetName();state=PhysxSchema.JointStateAPI.Apply(p,'angular')
                state.CreatePositionAttr(math.degrees(initial_all[name]));state.CreateVelocityAttr(0.)
                if name in mimics:
                    assert p.HasAPI(PhysxSchema.PhysxMimicJointAPI)
                    axis='rot'+UsdPhysics.RevoluteJoint(p).GetAxisAttr().Get()
                    p.CreateAttribute(f'physxMimicJoint:{axis}:naturalFrequency',Sdf.ValueTypeNames.Float).Set(0.)
                else:
                    d=UsdPhysics.DriveAPI.Apply(p,'angular');d.CreateTypeAttr('force');d.CreateMaxForceAttr(10.)
                    d.CreateStiffnessAttr(cfg.finger_stiffness_nm_rad);d.CreateDampingAttr(cfg.finger_damping_nm_s_rad)
                    d.CreateTargetPositionAttr(math.degrees(initial_q[name]))
        for axis in fixture['axes']:
            joints=[p for p in stage.Traverse() if p.GetName()==axis['name'] and p.IsA(UsdPhysics.Joint)];assert len(joints)==1
            d=UsdPhysics.DriveAPI.Apply(joints[0],'linear' if axis['type']=='prismatic' else 'angular')
            d.CreateTypeAttr('force');d.CreateStiffnessAttr(axis['kp']);d.CreateDampingAttr(axis['kd'])
            d.CreateMaxForceAttr(axis['effort_limit']);d.CreateTargetPositionAttr(0.)
        # Both link poses and joint state are consistent at the declared preload.
        # These authored initial conditions are never rewritten after reset.
        for name,t in initial_fk.items():
            p=stage.GetPrimAtPath('/Rhand/'+name);assert p
            xf=UsdGeom.Xformable(p);xf.ClearXformOpOrder();xf.AddTransformOp().Set(Gf.Matrix4d(*t.T.flatten().tolist()))
        stage.GetRootLayer().Save()
        world=World(stage_units_in_meters=1.,physics_dt=.005,rendering_dt=.005)
        scenes=[p for p in world.stage.Traverse() if p.IsA(UsdPhysics.Scene)];assert len(scenes)==1
        physics=PhysxSchema.PhysxSceneAPI.Apply(scenes[0]);physics.CreateEnableExternalForcesEveryIterationAttr(True);physics.CreateSolverTypeAttr('TGS')
        add_reference_to_stage(str(imported),'/World/Hand')
        for p in world.stage.Traverse():
            if p.HasAPI(PhysxSchema.PhysxArticulationAPI):
                a=PhysxSchema.PhysxArticulationAPI(p);a.CreateSolverPositionIterationCountAttr(32);a.CreateSolverVelocityIterationCountAttr(8);a.CreateSleepThresholdAttr(0.)
        mat=UsdShade.Material.Define(world.stage,'/World/DeclaredGraspMaterial');m=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        m.CreateStaticFrictionAttr(cfg.contact_static_friction);m.CreateDynamicFrictionAttr(cfg.contact_dynamic_friction);m.CreateRestitutionAttr(0.)
        for p in world.stage.Traverse():
            if p.HasAPI(UsdPhysics.CollisionAPI):UsdShade.MaterialBindingAPI.Apply(p).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
        scene_cfg=SceneConfig(frame=BoardFrame(origin_world_m=(1.,1.,.85)),holder_mode='free_dynamic')
        marker=build_scene(world.stage,scene_cfg)
        # Stand remains far from the hand; free marker alone receives a declared
        # initial preload pose. Its one internal compression joint is unchanged.
        center=(0.,.6,.14) if empty_control else cfg.holder_center_palm_m
        rot=cfg.holder_orientation_palm_qwxyz
        for path,offset in [(marker['holder_body'],0.),(marker['nib_body'],-scene_cfg.holder.tip_offset_m+scene_cfg.holder.nib_radius_m)]:
            p=world.stage.GetPrimAtPath(path);xf=UsdGeom.Xformable(p);xf.ClearXformOpOrder()
            delta=rotate(rot,(0.,0.,offset));xf.AddTranslateOp().Set(Gf.Vec3d(*(a+b for a,b in zip(center,delta))))
            xf.AddOrientOp().Set(Gf.Quatf(rot[0],Gf.Vec3f(*rot[1:])))
        marker['declared_initial_pose_override_before_reset']={'center_world_m':center,'orientation_world_qwxyz':rot,
                                                              'away_from_hand_for_empty_control':empty_control}
        write('marker_scene.json',marker)
        assert len([p for p in Usd.PrimRange(world.stage.GetPrimAtPath('/World/Marker')) if p.IsA(UsdPhysics.Joint)])==1
        for path in [marker['holder_body'],marker['nib_body']]:assert UsdPhysics.RigidBodyAPI(world.stage.GetPrimAtPath(path)).GetKinematicEnabledAttr().Get() is False
        UsdLux.DomeLight.Define(world.stage,'/World/Light').CreateIntensityAttr(500.)
        floor=UsdGeom.Cube.Define(world.stage,'/World/CatchFloor');floor.CreateSizeAttr(1.)
        floor.AddTranslateOp().Set(Gf.Vec3d(0.,0.,-.32));floor.AddScaleOp().Set(Gf.Vec3f(2.,2.,.04))
        floor.CreateDisplayColorAttr([Gf.Vec3f(.22,.24,.27)]);UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        sequence=None;phase='initialization';contacts=[];step_contacts=[];rows=[];frames=[]
        contact_file=out.joinpath('contacts.jsonl').open('w',buffering=1);state_file=out.joinpath('state.jsonl').open('w',buffering=1)
        frame_file=out.joinpath('frames.jsonl').open('w',buffering=1)
        for p in world.stage.Traverse():
            if p.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
        def on_contact(headers,data):
            for h in headers:
                paths={k:str(PhysicsSchemaTools.intToSdfPath(getattr(h,k))) for k in ['actor0','actor1','collider0','collider1']}
                for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                    d=data[i];r=dict(paths,sequence=sequence,physics_s=float(world.current_time),phase=phase,
                        position_world_m=list(map(float,d.position)),normal_world=list(map(float,d.normal)),
                        impulse_ns=list(map(float,d.impulse)),separation_m=float(d.separation),source='PhysX_simulated_contact_proxy')
                    contacts.append(r);step_contacts.append(r);contact_file.write(json.dumps(r,allow_nan=False)+'\n')
        subscription=get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
        hand=SingleArticulation('/World/Hand',name='supported_preloaded_hand')
        marker_rig=SingleArticulation('/World/Marker',name='free_passive_marker')
        world.stage.GetRootLayer().Export(str(out/'scene_before_reset.usda'))
        world.reset();hand.initialize();marker_rig.initialize()
        names=list(hand.dof_names);assert set(names)==set(limits)|{a['name'] for a in fixture['axes']}
        ids=np.array([names.index(n) for n in independent],dtype=np.int32)
        fixture_ids=np.array([names.index(a['name']) for a in fixture['axes']],dtype=np.int32)
        kp=np.zeros(len(names),dtype=np.float32);kd=kp.copy();kp[ids]=cfg.finger_stiffness_nm_rad;kd[ids]=cfg.finger_damping_nm_s_rad
        for a,i in zip(fixture['axes'],fixture_ids):kp[i],kd[i]=a['kp'],a['kd']
        hand._articulation_view.set_gains(kp,kd)
        got_kp,got_kd=hand.get_articulation_controller().get_gains();assert np.allclose(got_kp,kp) and np.allclose(got_kd,kd)
        views={k:SimulationManager.get_physics_sim_view().create_rigid_body_view(p) for k,p in
               [('palm','/World/Hand/right_base_link'),('holder',marker['holder_body']),('nib',marker['nib_body'])]}
        assert all(v.count==1 for v in views.values())
        def poses():return {k:np.asarray(v.get_transforms())[0].astype(float).tolist() for k,v in views.items()}
        initial_poses=poses();initial_measurement=relative_measurement(initial_poses['palm'],initial_poses['holder'],initial_poses['nib'],scene_cfg.holder.nib_radius_m)
        initial_positions=np.asarray(hand.get_joint_positions()).ravel()
        initial_error=max(abs(float(initial_positions[names.index(n)])-q) for n,q in initial_all.items())
        write('initial_state.json',{'poses_xyzw':initial_poses,'relative':initial_measurement,'names':names,'q':initial_positions.tolist(),
              'initial_joint_error_rad':initial_error,'kp_readback':np.asarray(got_kp).tolist(),'kd_readback':np.asarray(got_kd).tolist(),
              'physics_statistics':get_physxunittests_interface().get_physics_stats(),'source_hand_mass_kg':source_mass,
              'imported_hand_plus_wrist_mass_kg':mass,'marker_free_mass_kg':marker['total_free_object_mass_kg'],
              'hand_friction':{'static':float(m.GetStaticFrictionAttr().Get()),'dynamic':float(m.GetDynamicFrictionAttr().Get()),'source':'declared_sim_fixture'},
              'sleep_disabled_for_contact_observability':True,'scope':scope})
        assert initial_error<.01,'Authored initial joint preload was not realized'
        cameras={}
        for label,eye in [('front',(.35,.43,.30)),('side',(-.40,.22,.24))]:
            c=Camera('/World/'+label.title()+'Camera',resolution=(640,640));c.initialize();c.set_clipping_range(.005,10.)
            x=UsdGeom.Xformable(c.prim);x.ClearXformOpOrder();x.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
                Gf.Vec3d(*eye),Gf.Vec3d(-.015,.028,.145),Gf.Vec3d(0.,0.,1.)).GetInverse());cameras[label]=c
            out.joinpath('frames',label).mkdir(parents=True)
        for _ in range(8):world.render()
        duration=60. if mode=='preloaded-retention-60s' else 4.;total_steps=round((2.+duration)/.005)
        reference=None;previous=float(world.current_time);aborted=None
        for sequence in range(total_steps):
            elapsed=sequence*.005;phase='preloaded_close' if elapsed<1. else 'grip_settle' if elapsed<2. else 'retention_wrist' if mode.endswith('60s') else 'retention_static'
            fraction=min(1.,elapsed/.5);fraction=fraction*fraction*(3-2*fraction)
            command=[initial_q[n]+fraction*(target_q[n]-initial_q[n]) for n in independent]
            wrist=wrist_target(max(0.,elapsed-2.)) if elapsed>=2. and mode.endswith('60s') else [0.]*6
            step_contacts.clear();hand.apply_action(ArticulationAction(joint_positions=np.asarray(command,dtype=np.float32),joint_indices=ids))
            hand.apply_action(ArticulationAction(joint_positions=np.asarray(wrist,dtype=np.float32),joint_indices=fixture_ids))
            world.step(render=False);now=float(world.current_time);dt=now-previous;previous=now
            assert math.isclose(dt,.005,rel_tol=1e-5,abs_tol=1e-8)
            p=poses();measurement=relative_measurement(p['palm'],p['holder'],p['nib'],scene_cfg.holder.nib_radius_m)
            if reference is None and elapsed>=2.:reference=measurement
            slip=drift(reference or initial_measurement,measurement)
            q=np.asarray(hand.get_joint_positions()).ravel();dq=np.asarray(hand.get_joint_velocities()).ravel()
            assert np.isfinite(q).all() and np.isfinite(dq).all()
            object_contacts=[];hand_contacts=[];external=[]
            for c in step_contacts:
                actors=[c['actor0'],c['actor1']]
                if not any(a.startswith('/World/Marker/') for a in actors) or np.linalg.norm(c['impulse_ns'])<=1e-10:continue
                object_contacts.append(c)
                if any(a.startswith('/World/Hand/') for a in actors):hand_contacts.append(c)
                elif not all(a.startswith('/World/Marker/') for a in actors):external.append(c)
            hp=p['holder'];nq=np.asarray(marker_rig.get_joint_positions()).ravel();ndq=np.asarray(marker_rig.get_joint_velocities()).ravel()
            compression=compression_from_poses(hp[:3],(hp[6],*hp[3:6]),p['nib'][:3],scene_cfg.holder)
            row={'sequence':sequence,'physics_s':now,'physics_dt_s':dt,'phase':phase,'runtime_joint_names':names,
                 'q_rad_or_m':q.tolist(),'dq_rad_or_m_s':dq.tolist(),'command_names':independent,'command_rad':command,
                 'wrist_command':wrist,'wrist_q':q[fixture_ids].tolist(),'poses_world_xyzw':p,'relative':measurement,**slip,
                 'holder_hand_contact':bool(hand_contacts),'external_object_contact':bool(external),
                 'object_contact_count':len(object_contacts),'contact_hand_links':sorted({a for c in hand_contacts for a in [c['actor0'],c['actor1']] if a.startswith('/World/Hand/')}),
                 'contact_impulse_vector_norm_sum_ns':float(sum(np.linalg.norm(c['impulse_ns']) for c in hand_contacts)),
                 'measured_joint_effort_nm_or_n':np.asarray(hand.get_measured_joint_efforts()).ravel().tolist(),
                 'effective_position_targets':np.asarray(hand.get_applied_action().joint_positions).ravel().tolist(),
                 'marker_compression_raw_m':compression,'marker_slider_q_m':nq.tolist(),'marker_slider_velocity_m_s':ndq.tolist(),
                 'marker_spring_force_toward_tip_n':scene_cfg.holder.force_model_toward_tip_n(float(nq[0]),float(ndq[0])),
                 'coupling_error_rad':{n:float(q[names.index(n)]-(m['multiplier']*q[names.index(m['parent'])]+m['offset'])) for n,m in mimics.items()},
                 'scope':scope}
            rows.append(row);state_file.write(json.dumps(row,allow_nan=False)+'\n')
            if (sequence+1)%20==0:
                before=float(world.current_time);world.render();assert float(world.current_time)==before
                saved={};frame_id=len(frames)
                for label,c in cameras.items():
                    rgba=c.get_rgba();assert rgba is not None and rgba.shape==(640,640,4)
                    name=f'frames/{label}/{frame_id:06d}.png';Image.fromarray(rgba.astype(np.uint8)).save(out/name);saved[label]=name
                frame={'frame':frame_id,'sequence':sequence,'physics_s':now,'phase':phase,'captured_after_same_step_render':True,'views':saved}
                frames.append(frame);frame_file.write(json.dumps(frame)+'\n')
            elif (sequence+1)%4==0:world.render()
            if not empty_control and elapsed>.2 and (math.dist(measurement['holder_center_palm_m'],cfg.holder_center_palm_m)>.08 or external):
                aborted='object_escaped_or_received_external_support';break
        retained=[r for r in rows if r['phase'].startswith('retention')]
        retention=retention_result(retained,expected_seconds=duration,dt_s=.005)
        initial_object=[c for c in contacts if c['sequence'] is None and any(c[a].startswith('/World/Marker/') for a in ['actor0','actor1'])]
        deepest=min((c['separation_m'] for c in initial_object),default=0.)
        self_contacts=[c for c in contacts if all(c[a].startswith('/World/Hand/') for a in ['actor0','actor1']) and np.linalg.norm(c['impulse_ns'])>1e-10]
        self_pairs={}
        for c in self_contacts:
            key=' <-> '.join(sorted([c['actor0'],c['actor1']]))
            entry=self_pairs.setdefault(key,{'points':0,'impulse_vector_norm_sum_ns':0.,'minimum_separation_m':0.,'initialization_points':0})
            entry['points']+=1;entry['impulse_vector_norm_sum_ns']+=float(np.linalg.norm(c['impulse_ns']))
            entry['minimum_separation_m']=min(entry['minimum_separation_m'],c['separation_m'])
            entry['initialization_points']+=int(c['sequence'] is None)
        write('self_contact_summary.json',self_pairs)
        checks={'complete_steps':len(rows)==total_steps,'initial_preload_realized':initial_error<.01,
                'initial_object_no_deep_penetration':deepest>=-.0005,'retention_window':retention['accepted'],
                'front_and_side_recorded':bool(frames),'no_early_abort':aborted is None}
        if empty_control:
            settled=rows[-100:]
            checks={'complete_steps':len(rows)==total_steps,'initial_preload_realized':initial_error<.01,
                    'no_hand_object_contacts':not any(r['holder_hand_contact'] for r in rows),
                    'coupling':max(abs(v) for r in rows for v in r['coupling_error_rad'].values())<.03,
                    'joint_limits':all(limits[n][0]-.03<=r['q_rad_or_m'][names.index(n)]<=limits[n][1]+.03 for r in rows for n in limits),
                    'settled_target_tracking':all(abs(r['q_rad_or_m'][names.index(n)]-target_q[n])<.05 for r in settled for n in independent),
                    'settled_velocity':all(abs(r['dq_rad_or_m_s'][names.index(n)])<.02 for r in settled for n in independent),
                    'front_and_side_recorded':bool(frames)}
        metrics={'schema_version':1,'checks':checks,'steps':len(rows),'mode':mode,'scope':scope,'retention':retention,
                 'supported_preloaded_retention_60s_qualified':mode.endswith('60s') and all(checks.values()),
                 'static_preloaded_diagnostic_pass':mode=='preloaded-close-hold' and all(checks.values()),
                 'empty_hand_preload_control_pass':empty_control and all(checks.values()),
                 'pickup_qualification':'NOT_RUN','writing_qualification':'NOT_RUN','standing_qualification':'NOT_RUN',
                 'initial_contact_minimum_separation_m':deepest,'initial_object_contact_points':len(initial_object),
                 'nonzero_self_contact_points':len(self_contacts),'self_contact_summary':'self_contact_summary.json',
                 'maximum_retention_tip_drift_m':max((r['tip_drift_m'] for r in retained),default=None),
                 'maximum_retention_axis_drift_deg':max((r['axis_drift_deg'] for r in retained),default=None),'abort_reason':aborted,
                 'backend':'CPU_PhysX_with_GPU_rendering','media_labels':{'fixture':'Dynamically driven six-axis supported wrist; freely dynamic marker',
                 'embodiment':'PROVISIONAL FTP RIGHT HAND - exact E2 unverified',
                 'qualification':'Empty-hand preload control; no grasp test' if empty_control else 'Preloaded supported-hand retention only; pickup, writing and standing unqualified'}}
        write('metrics.json',metrics);write('retention.json',retention)
        world.stage.GetRootLayer().Export(str(out/'scene_final.usda'))
        contact_file.close();state_file.close();frame_file.close()
        artifacts=['grasp_config.json','preload_geometry.json','collision_candidate.json','marker_scene.json','initial_state.json',
                   'scene_before_reset.usda','scene_final.usda','metrics.json','retention.json','self_contact_summary.json','state.jsonl','frames.jsonl','supported_wrist.urdf']
        if out.joinpath('contacts.jsonl').stat().st_size:artifacts.append('contacts.jsonl')
        artifacts += [str(p.relative_to(out)) for p in sorted(out.joinpath('frames').rglob('*.png'))]
        status='PASS' if all(checks.values()) else 'FAIL';write('probe.json',{'status':status,'scope':'provisional_supported_preloaded_hand_only','artifacts':artifacts})
        return 0 if status=='PASS' else 1
    except BaseException as e:
        write('failure.json',{'error':f'{type(e).__name__}: {e}','traceback':traceback.format_exc(),'scope':scope})
        artifacts=['grasp_config.json','failure.json']+[n for n in ['preload_geometry.json','marker_scene.json','collision_candidate.json',
            'scene_before_reset.usda','initial_state.json','state.jsonl','contacts.jsonl','frames.jsonl'] if out.joinpath(n).is_file() and out.joinpath(n).stat().st_size]
        write('probe.json',{'status':'FAIL','scope':'provisional_supported_preloaded_hand_only','artifacts':artifacts});print(traceback.format_exc(),flush=True);return 1
    finally:
        if app is not None:app.close()


if __name__=='__main__':raise SystemExit(main())
