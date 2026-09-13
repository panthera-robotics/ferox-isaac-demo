"""Actual-contact I/L instrumentation on a declared driven-carriage bench.

This does not run the private writer or grasp a marker. A physically driven carriage
attaches the holder, automatically excluding grasp/writing/standing qualification.
All ink geometry is built from measured tip/board contact points, never targets.
"""
from dataclasses import asdict
import csv
import json
import math
import os
from pathlib import Path
import sys
import traceback


def trajectory(config):
    """Independent instrument motion, explicitly not a task-node/IK writing pipeline."""
    from twin.inspire.contact_ink import IntendedStroke
    dt=config.physics_dt_s
    strokes=(IntendedStroke('I',((-.070,.030),(-.070,-.030))),
             IntendedStroke('L',((.010,.030),(.010,-.030),(.060,-.030))))
    initial=config.initial_xy_board_m
    if initial != strokes[0].points_board_m[0]:
        raise ValueError('This bounded bench starts at I (-0.070,0.030); configure initial XY accordingly')
    press=-(config.initial_gap_m+config.holder.nominal_compression_m)
    plan=[]
    def phase(name,start,end,seconds,pen_down=False,stroke=None,segment=None,start_fraction=None,end_fraction=None):
        count=max(2,round(seconds/dt))
        for i in range(count):
            t=i/(count-1); f=t*t*(3-2*t)
            plan.append({'phase':name,'carriage_target_m':tuple(a+f*(b-a) for a,b in zip(start,end)),
                         'pen_down':pen_down,'stroke_id':stroke,'segment_index':segment,
                         'reference_fraction':None if start_fraction is None else start_fraction+f*(end_fraction-start_fraction)})
    start=(*initial,0.)
    phase('settle_air',start,start,.5)
    current=start
    for stroke in strokes:
        xy=stroke.points_board_m[0];air=(*xy,0.);down=(*xy,press)
        if current!=air:phase(stroke.stroke_id+'_air_transfer',current,air,.8)
        phase(stroke.stroke_id+'_approach',air,down,.6,True,stroke.stroke_id,0,0.,0.)
        phase(stroke.stroke_id+'_contact_hold',down,down,.4,True,stroke.stroke_id,0,0.,0.)
        for index,(a,b) in enumerate(zip(stroke.points_board_m,stroke.points_board_m[1:])):
            duration=math.dist(a,b)/.025
            phase(stroke.stroke_id+f'_segment_{index}',(*a,press),(*b,press),duration,True,stroke.stroke_id,index,0.,1.)
        end=stroke.points_board_m[-1];current=(*end,0.)
        phase(stroke.stroke_id+'_release',(*end,press),current,.6)
    phase('final_air',current,current,.5)
    return strokes,plan


def main():
    assert os.environ.get('PANTHERA_SIM_AUTHORIZED')=='1'
    assert sorted(p.name for p in Path('/sys/class/net').iterdir())==['lo']
    assert os.environ.get('PANTHERA_PROBE_MODE','default') in {'default','contact-il-bench'}
    sys.path.insert(0,'/workspace/ferox_isaac')
    from twin.inspire.whiteboard_scene import (SceneConfig,build_scene,compression_from_poses,reduce_tip_contacts,rotate)
    from twin.inspire.contact_ink import ContactSample,MarkingRule,evaluate,export_svg,export_csv
    payload=json.loads(Path(os.environ['PANTHERA_PROBE_CONFIG']).read_text()) if os.environ.get('PANTHERA_PROBE_CONFIG') else {}
    config=SceneConfig.from_dict({'holder_mode':'driven_carriage',**payload})
    assert config.holder_mode=='driven_carriage','This probe measures an explicitly attached instrumentation carriage'
    strokes,plan=trajectory(config)
    out=Path('/evidence'); app=None
    manifest={'schema_version':1,'config':asdict(config),'config_sha256':config.sha256,'scope':config.scope}
    (out/'scene_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False))
    def write(name,value):
        (out/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'renderer':'RaytracedLighting'})
        import numpy as np
        from PIL import Image
        from pxr import Gf,Usd,UsdGeom,UsdLux,UsdPhysics,PhysxSchema,PhysicsSchemaTools
        from omni.physx import get_physx_simulation_interface,get_physxunittests_interface
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.sensors.camera import Camera
        world=World(stage_units_in_meters=1.,physics_dt=config.physics_dt_s,rendering_dt=config.physics_dt_s)
        scenes=[p for p in world.stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        assert len(scenes)==1
        physics=PhysxSchema.PhysxSceneAPI.Apply(scenes[0])
        physics.CreateEnableExternalForcesEveryIterationAttr(True)
        physics.CreateSolverTypeAttr('TGS')
        UsdLux.DomeLight.Define(world.stage,'/World/Light').CreateIntensityAttr(700.)
        # Static authored stand is explicit, and a local analytic floor avoids network assets.
        floor=UsdGeom.Cube.Define(world.stage,'/World/Floor');floor.CreateSizeAttr(1.)
        floor.AddTranslateOp().Set(Gf.Vec3d(0.,0.,config.board.ground_z_m-.025))
        floor.AddScaleOp().Set(Gf.Vec3f(2.,2.,.05));floor.CreateDisplayColorAttr([Gf.Vec3f(.22,.23,.24)])
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        manifest=build_scene(world.stage,config);write('scene_manifest.json',manifest)
        world.stage.GetRootLayer().Export(str(out/'scene_authored_before_reset.usda'))
        trace=[];contacts=[];contact_step=[];samples=[];frames=[];ink_paths=[];last_mark_sequence=None
        sequence=None;phase='initialization'
        contact_file=(out/'contacts.jsonl').open('w',buffering=1)
        state_file=(out/'state.jsonl').open('w',buffering=1)
        frame_file=(out/'frames.jsonl').open('w',buffering=1)
        def on_contacts(headers,data):
            for header in headers:
                paths={key:str(PhysicsSchemaTools.intToSdfPath(getattr(header,key)))
                       for key in ['actor0','actor1','collider0','collider1']}
                for index in range(header.contact_data_offset,header.contact_data_offset+header.num_contact_data):
                    point=data[index]
                    row=dict(paths,sequence=sequence,physics_s=float(world.current_time),phase=phase,
                             position_world_m=list(map(float,point.position)),normal_world=list(map(float,point.normal)),
                             impulse_ns=list(map(float,point.impulse)),separation_m=float(point.separation),source='PhysX_simulated_contact_proxy')
                    contacts.append(row);contact_step.append(row)
                    contact_file.write(json.dumps(row,allow_nan=False)+'\n')
        subscription=get_physx_simulation_interface().subscribe_contact_report_events(on_contacts)
        rig=SingleArticulation(manifest['articulation_path'],name='instrumented_marker_carriage')
        world.reset();rig.initialize()
        names=list(rig.dof_names); expected=set(manifest['carriage_joint_names']+[manifest['slider_joint_name']])
        assert len(names)==len(expected) and set(names)==expected,'Exact named carriage/slider map required'
        carriage_ids=[names.index(name) for name in manifest['carriage_joint_names']];slider_id=names.index(manifest['slider_joint_name'])
        holder_view=SimulationManager.get_physics_sim_view().create_rigid_body_view(manifest['holder_body'])
        nib_view=SimulationManager.get_physics_sim_view().create_rigid_body_view(manifest['nib_body'])
        assert holder_view.count==1 and nib_view.count==1
        def pose(view):
            data=np.asarray(view.get_transforms()).reshape(1,7)[0]
            assert np.isfinite(data).all()
            return tuple(map(float,data[:3])),(float(data[6]),*map(float,data[3:6]))
        kp,kd=rig.get_articulation_controller().get_gains();kp=np.asarray(kp).ravel();kd=np.asarray(kd).ravel()
        assert np.isclose(kp[slider_id],config.holder.stiffness_n_m) and np.isclose(kd[slider_id],config.holder.damping_n_s_m)
        for i in carriage_ids:
            assert np.isclose(kp[i],config.carriage_stiffness_n_m) and np.isclose(kd[i],config.carriage_damping_n_s_m)
        slider_prim=world.stage.GetPrimAtPath(manifest['articulation_path']+'/'+manifest['slider_joint_name'])
        drive=UsdPhysics.DriveAPI(slider_prim,'linear')
        initial={'runtime_joint_names':names,'runtime_gains':{'stiffness_n_m':kp.tolist(),'damping_n_s_m':kd.tolist()},
                 'slider_target_m_authored':float(drive.GetTargetPositionAttr().Get()),'spring_force_sign':'negative generalized force extends nib',
                 'slider_target_m_runtime':float(np.asarray(rig.get_applied_action().joint_positions).ravel()[slider_id]),
                 'gravity':str(world.get_physics_context().get_gravity()),'physics_statistics':get_physxunittests_interface().get_physics_stats(),
                 'physics_configuration':{str(p.GetPath()):{a.GetName():str(a.Get()) for a in p.GetAttributes()
                                          if a.GetName().startswith(('physics:','physx'))} for p in world.stage.Traverse()
                                          if p.IsA(UsdPhysics.Scene) or p.HasAPI(PhysxSchema.PhysxArticulationAPI)}}
        assert math.isclose(initial['slider_target_m_authored'],config.holder.spring_target_m,abs_tol=1e-7)
        assert math.isclose(initial['slider_target_m_runtime'],config.holder.spring_target_m,abs_tol=1e-7),'Runtime spring preload target changed'
        write('initial_state.json',initial)
        cameras={}
        for label,eye_board in [('front',(.02,.015,.50)),('side',(.42,-.015,.10))]:
            camera=Camera('/World/'+label.title()+'Camera',resolution=(640,640));camera.initialize();camera.set_clipping_range(.005,10.)
            transform=UsdGeom.Xformable(camera.prim);transform.ClearXformOpOrder()
            transform.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*config.frame.to_world(eye_board)),
                Gf.Vec3d(*config.frame.to_world((-.01,0.,.035))),Gf.Vec3d(*rotate(config.frame.orientation_world_qwxyz,(0.,1.,0.)))).GetInverse())
            cameras[label]=camera;(out/'frames'/label).mkdir(parents=True)
        rule=MarkingRule(spring_travel_m=config.holder.slider_travel_m,maximum_sample_interval_s=config.physics_dt_s*1.1)
        ink=UsdGeom.BasisCurves.Define(world.stage,'/World/MeasuredContactInk')
        ink.CreateTypeAttr('linear');ink.CreateWrapAttr('nonperiodic');ink.CreateWidthsAttr([rule.ink_width_m]);ink.SetWidthsInterpolation('constant')
        ink.CreateDisplayColorAttr([Gf.Vec3f(.015,.020,.080)])
        write('intended_strokes.json',[asdict(s) for s in strokes])
        write('instrument_motion.json',plan)
        world.stage.GetRootLayer().Export(str(out/'scene_initial.usda'))
        for _ in range(8):world.render()
        last_time=float(world.current_time)
        for sequence,command in enumerate(plan):
            phase=command['phase'];contact_step.clear()
            rig.apply_action(ArticulationAction(joint_positions=np.asarray(command['carriage_target_m'],dtype=np.float32),joint_indices=carriage_ids))
            world.step(render=False)
            now=float(world.current_time);dt=now-last_time;last_time=now
            assert math.isclose(dt,config.physics_dt_s,rel_tol=1e-6,abs_tol=1e-8),'Physics dt changed'
            q=np.asarray(rig.get_joint_positions()).ravel();dq=np.asarray(rig.get_joint_velocities()).ravel()
            assert np.isfinite(q).all() and np.isfinite(dq).all()
            holder_pos,holder_rot=pose(holder_view);nib_pos,nib_rot=pose(nib_view)
            tip_delta=rotate(nib_rot,(0.,0.,-config.holder.nib_radius_m));tip_world=tuple(a+b for a,b in zip(nib_pos,tip_delta))
            geometric_q=compression_from_poses(holder_pos,holder_rot,nib_pos,config.holder)
            observation=reduce_tip_contacts(contact_step,tip_collider=manifest['tip_collider'],board_collider=manifest['board_collider'],
                                             frame=config.frame,nib_position_world=tip_world,dt_s=dt)
            bottomed=geometric_q>=config.holder.slider_travel_m-config.holder.bottomout_margin_m
            sample=ContactSample(physics_sequence=sequence,physics_time_s=now,physics_dt_s=dt,
                nib_position_board_m=observation['position_board_m'],nib_board_contact=observation['nib_board_contact'],
                pen_down=command['pen_down'],spring_compression_m=max(0.,geometric_q),stroke_id=command['stroke_id'],
                segment_index=command['segment_index'],reference_fraction=command['reference_fraction'],
                normal_impulse_ns=observation['normal_impulse_ns'],normal_force_n=observation['normal_force_n'],
                attachment_active=True,fixture_support_active=True,holder_bottomed_out=bottomed)
            samples.append(sample)
            row={'sequence':sequence,'physics_s':now,'physics_dt_s':dt,'phase':phase,'command':command,'joint_names':names,
                 'joint_position_m':q.tolist(),'joint_velocity_m_s':dq.tolist(),'slider_q_m':float(q[slider_id]),
                 'geometric_compression_raw_m':geometric_q,'slider_velocity_m_s':float(dq[slider_id]),
                 'spring_force_model_toward_tip_n':config.holder.force_model_toward_tip_n(float(q[slider_id]),float(dq[slider_id])),
                 'measured_joint_effort_n':np.asarray(rig.get_measured_joint_efforts()).ravel().tolist(),
                 'commanded_feedforward_force_n':np.asarray(rig.get_applied_joint_efforts()).ravel().tolist(),
                 'effective_position_target_m':np.asarray(rig.get_applied_action().joint_positions).ravel().tolist(),
                 'holder_position_world_m':holder_pos,'holder_orientation_world_qwxyz':holder_rot,
                 'nib_position_world_m':nib_pos,'tip_world_m':tip_world,'tip_board_m':config.frame.to_board(tip_world),
                 'contact':observation,'holder_bottomed_out':bottomed,'scope':config.scope}
            trace.append(row);state_file.write(json.dumps(row,allow_nan=False)+'\n')
            mark=(observation['nib_board_contact'] is True and rule.minimum_force_n<=sample.force_n<=rule.maximum_force_n
                  and abs(sample.nib_position_board_m[2])<=rule.contact_plane_tolerance_m and not bottomed)
            if mark:
                if last_mark_sequence!=sequence-1:ink_paths.append([])
                ink_paths[-1].append(sample.nib_position_board_m[:2]);last_mark_sequence=sequence
            if (sequence+1)%20==0:
                visible=[path for path in ink_paths if len(path)>=2]
                points=[Gf.Vec3f(*config.frame.to_world((x,y,.0002))) for path in visible for x,y in path]
                ink.GetCurveVertexCountsAttr().Set([len(path) for path in visible]);ink.GetPointsAttr().Set(points)
                before=float(world.current_time);world.render();assert float(world.current_time)==before
                views={};frame_id=len(frames)
                for label,camera in cameras.items():
                    pixels=camera.get_rgba();assert pixels is not None and pixels.shape==(640,640,4)
                    name=f'frames/{label}/{frame_id:06d}.png';Image.fromarray(pixels.astype(np.uint8)).save(out/name);views[label]=name
                frame={'frame':frame_id,'sequence':sequence,'physics_s':now,'phase':phase,'captured_after_same_step_render':True,'views':views}
                frames.append(frame);frame_file.write(json.dumps(frame)+'\n')
            elif (sequence+1)%4==0:world.render()
        report=evaluate(strokes,samples,rule);write('contact_ink.json',report);export_svg(report,out/'contact_ink.svg');export_csv(samples,out/'contact_samples.csv')
        with (out/'joint_state.csv').open('w',newline='') as stream:
            columns=['sequence','physics_s','phase','compression_raw_m','normal_contact_force_n']
            columns += ['q_m:'+name for name in names]+['dq_m_s:'+name for name in names]
            writer=csv.writer(stream);writer.writerow(columns)
            for row in trace:
                writer.writerow([row['sequence'],row['physics_s'],row['phase'],row['geometric_compression_raw_m'],row['contact']['normal_force_n']]
                                +row['joint_position_m']+row['joint_velocity_m_s'])
        with (out/'contacts.csv').open('w',newline='') as stream:
            columns=['sequence','physics_s','phase','actor0','actor1','collider0','collider1','position_world_m','normal_world','impulse_ns','separation_m','source']
            writer=csv.DictWriter(stream,fieldnames=columns);writer.writeheader()
            for row in contacts:writer.writerow({key:json.dumps(value) if isinstance(value,(list,tuple)) else value for key,value in row.items()})
        final=[r for r in trace if r['phase']=='final_air'][-40:]
        hold=[r for r in trace if r['phase']=='I_contact_hold'][-40:]
        mean_force=sum(r['contact']['normal_force_n'] for r in hold)/len(hold)
        mean_model=sum(r['spring_force_model_toward_tip_n'] for r in hold)/len(hold)
        compression_max=max(r['geometric_compression_raw_m'] for r in trace)
        compression_consistency=max(abs(r['slider_q_m']-r['geometric_compression_raw_m']) for r in trace)
        checks={'complete_steps':len(trace)==len(plan),'measured_contact_onset':any(s.nib_board_contact for s in samples),
                'spring_compressed_by_contact':compression_max>=.002,
                'spring_force_direction_and_preload':mean_force>.2 and mean_model>0 and abs(mean_force-mean_model)<=max(.5,.3*mean_model),
                'pose_and_joint_compression_agree':compression_consistency<.0002,
                'no_bottomout':not any(r['holder_bottomed_out'] for r in trace),
                'contact_released':all(r['contact']['nib_board_contact'] is False for r in final),
                'slider_returned':all(abs(r['geometric_compression_raw_m'])<.0003 for r in final),
                'contact_only_ink_nonempty':report['marked_samples']>0,
                'intended_geometric_coverage_diagnostic':all(segment['coverage_fraction']>=rule.minimum_segment_coverage for segment in report['segments']),
                'path_error_diagnostic':report['path_error_p95_m'] is not None and report['path_error_p95_m']<=rule.maximum_path_p95_m and report['path_error_max_m']<=rule.maximum_path_error_m,
                'bench_acceptance_excluded':report['accepted'] is False and config.scope['grasp_qualified'] is False,
                'physics_samples_valid':report['valid'],'front_and_side_recorded':len(frames)>0}
        metrics={'schema_version':1,'checks':checks,'scope':config.scope,'grasp_qualification':'NOT_RUN','writing_qualification':'NOT_RUN',
                 'media_labels':{'fixture':'Holder attached to driven XYZ carriage; board on fixed physical stand',
                                 'embodiment':'GENERIC INSTRUMENTED BOARD - no hand',
                                 'qualification':'Contact instrumentation only; grasp and writing qualification false'},
                 'standing_qualification':'NOT_RUN','task_node_integration':'NOT_RUN','hardware_calibration':'UNAVAILABLE',
                 'steps':len(trace),'physics_dt_s':config.physics_dt_s,'backend':'CPU_PhysX_with_GPU_rendering',
                 'randomized':False,'seed':0,'physics_enable_gpu_dynamics_authored':str(physics.GetEnableGPUDynamicsAttr().Get()),
                 'normal_hold_mean_force_n':mean_force,'normal_hold_mean_spring_model_n':mean_model,
                 'maximum_compression_m':compression_max,'maximum_pose_joint_compression_difference_m':compression_consistency,
                 'actual_contact_points':len(contacts),'tip_board_loaded_samples':sum(s.nib_board_contact is True for s in samples),
                 'ink_coverage_fraction_diagnostic':report['coverage_fraction'],'ink_error_p95_m_diagnostic':report['path_error_p95_m'],
                 'ink_error_max_m_diagnostic':report['path_error_max_m'],'pen_up_marked_samples':report['pen_up_marked_samples'],
                 'initialization_contacts':sum(r['sequence'] is None for r in contacts),
                 'media_capture':{'views':['front','side'],'physics_steps_per_frame':20,'frame_trace_map':'frames.jsonl'},
                 'ink_render_source':'measured_contact_points_only; display lift 0.2 mm does not alter physics',
                 'lateral_snag_breakaway_test':'NOT_RUN','stand_perturbation_test':'NOT_RUN'}
        write('metrics.json',metrics)
        world.stage.GetRootLayer().Export(str(out/'scene_final.usda'))
        contact_file.close();state_file.close();frame_file.close()
        artifacts=['scene_manifest.json','initial_state.json','intended_strokes.json','instrument_motion.json','metrics.json','state.jsonl',
                   'frames.jsonl','scene_authored_before_reset.usda','scene_initial.usda','scene_final.usda','contact_ink.json','contact_ink.svg','contact_samples.csv','joint_state.csv','contacts.csv']
        if (out/'contacts.jsonl').stat().st_size:artifacts.append('contacts.jsonl')
        artifacts += [str(p.relative_to(out)) for p in sorted((out/'frames').rglob('*.png'))]
        status='PASS' if all(checks.values()) else 'FAIL'
        write('probe.json',{'status':status,'scope':'attached_carriage_contact_instrumentation_only','artifacts':artifacts,'metrics':'metrics.json'})
        return 0 if status=='PASS' else 1
    except BaseException as error:
        write('failure.json',{'error':f'{type(error).__name__}: {error}','traceback':traceback.format_exc(),'scope':config.scope})
        artifacts=['scene_manifest.json','failure.json']
        artifacts += [name for name in ['scene_authored_before_reset.usda','initial_state.json','state.jsonl','contacts.jsonl','frames.jsonl']
                      if (out/name).is_file() and (out/name).stat().st_size]
        write('probe.json',{'status':'FAIL','scope':'attached_carriage_contact_instrumentation_only','artifacts':artifacts})
        print(traceback.format_exc(),flush=True)
        return 1
    finally:
        if app is not None:app.close()


if __name__=='__main__':raise SystemExit(main())
