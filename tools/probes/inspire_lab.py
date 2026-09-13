"""Bounded Isaac Lab plumbing check using the same assembled donor and scene.

Requires the pinned, cached Lab runtime image. No policy weights, training,
hardware calibration, or network endpoints are embedded in this public probe.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

def camera_schedule(capture, steps, control_dt_s):
    """Optional10Hz evidence, at most40 paired frames per admitted run."""
    if type(capture) is not bool:raise ValueError('capture_cameras must be an explicit boolean')
    if type(steps) is not int or not 4<=steps<=200 or not math.isclose(control_dt_s,.02,abs_tol=1e-12):
        raise ValueError('Camera schedule requires admitted20ms control steps and bounded duration')
    return tuple(i for i in range(steps) if capture and (i+1)%5==0)


def camera_plan(origins):
    """Bound both views around actual clone origins and a declared scene box."""
    if not isinstance(origins,list) or not 1<=len(origins)<=8:raise ValueError('Expected1..8 measured clone origins')
    if any(len(p)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in p) for p in origins):
        raise ValueError('Invalid measured clone origin')
    low=[min(p[i] for p in origins)-(1.2 if i<2 else .15) for i in range(3)]
    high=[max(p[i] for p in origins)+(1.2 if i<2 else 2.1) for i in range(3)]
    target=[(a+b)/2 for a,b in zip(low,high)]
    radius=math.dist(low,high)/2
    # 24mm focal,36mm horizontal aperture,960x640 =>24mm vertical aperture.
    distance=radius/math.sin(math.atan(12./24.))*1.12
    views={}
    for label,direction in [('front',(1.,-2.,.65)),('side',(-2.,.7,.55))]:
        norm=math.sqrt(sum(x*x for x in direction))
        views[label]={'position_world_m':[p+distance*d/norm for p,d in zip(target,direction)],
            'target_world_m':target,'resolution':[960,640],'focal_length_mm':24.,'horizontal_aperture_mm':36.}
    return {'environment_origins_world_m':origins,'scene_bounds_world_m':[low,high],
        'bounds_scope':'actual clone origins plus declared1.2m lateral,−0.15..2.1m vertical source-scene envelope','views':views}


def render_state_record(*,step,physics_s,observation,environments,transitions,reset_flags):
    """Map the rendered returned observation to distinct pre-reset transitions."""
    count=len(environments)
    if not count or not len(observation)==len(transitions)==len(reset_flags)==count:raise ValueError('Render mapping environment lengths differ')
    if type(step) is not int or step<0 or not math.isfinite(physics_s):raise ValueError('Invalid rendered clock')
    links=[]
    for i,(obs,current,previous,reset) in enumerate(zip(observation,environments,transitions,reset_flags)):
        if type(reset) is not bool or current['env_id']!=i or previous['env_id']!=i:raise ValueError('Ambiguous render environment identity')
        if len(obs)!=119 or len(current['q_rad'])!=53 or len(current['dq_rad_s'])!=53 or len(current['root_state_world_p_qwxyz_v_w'])!=13:
            raise ValueError('Missing actual named53/current119 observation')
        names=current['measurement_joint_names']
        if len(names)!=53 or len(set(names))!=53 or not all(isinstance(n,str) and n for n in names):
            raise ValueError('Invalid actual measurement name map')
        if len(current['environment_origin_world_m'])!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in current['environment_origin_world_m']):
            raise ValueError('Invalid actual environment origin')
        if any(type(x) not in (int,float) or not math.isfinite(x) for x in obs+current['q_rad']+current['dq_rad_s']+current['root_state_world_p_qwxyz_v_w']):
            raise ValueError('Nonfinite rendered observation')
        root=list(current['root_state_world_p_qwxyz_v_w'])
        root[:3]=[x-y for x,y in zip(root[:3],current['environment_origin_world_m'])]
        measured=current['q_rad']+current['dq_rad_s']+root
        if any(not math.isclose(a,b,rel_tol=1e-5,abs_tol=1e-6) for a,b in zip(obs,measured)):
            raise ValueError('Returned observation differs from rendered current state')
        if current['episode_id']!=previous['episode_id']+int(reset):raise ValueError('Reset/episode identity differs from actual returned state')
        if current['episode_step']!=(0 if reset else previous['episode_step']):raise ValueError('Rendered episode step does not describe post-reset state')
        if abs(float(previous['physics_time_s'])-physics_s)>1e-7:raise ValueError('Transition and rendered observation world clocks differ')
        links.append({'env_id':i,'transition_sequence':previous['sequence'],'transition_episode_id':previous['episode_id'],
            'transition_episode_step':previous['episode_step'],'transition_state_phase':previous['state_phase'],
            'reset_before_image':reset,'render_episode_id':current['episode_id'],'render_episode_step':current['episode_step']})
    return {'sequence':step,'physics_s':physics_s,'phase':'returned_observation_after_step_and_any_automatic_reset',
        'returned_policy_observation':observation,'environments':environments,'source_transitions':links,
        'state_source':'actual Isaac Lab buffers after env.step, kinematic forward and explicit render; no physics advance',
        'not_the_pre_reset_transition_image':any(reset_flags)}


def validate_camera_receipts(before,after,physics_s):
    if set(before)!={'front','side'} or set(after)!={'front','side'}:raise ValueError('Paired camera receipt missing')
    for label,value in after.items():
        if type(value['rendering_frame']) is not int or value['rendering_frame']<=before[label]:
            raise ValueError('Camera did not acquire a new frame after explicit render')
        timestamp=value['rendering_time']
        if type(timestamp) not in (int,float) or not math.isfinite(timestamp) or not math.isclose(timestamp,physics_s,abs_tol=1e-7,rel_tol=0):
            raise ValueError('Camera timestamp differs from measured physics state')
    if after['front']['rendering_frame']!=after['side']['rendering_frame']:raise ValueError('Camera pair refers to different render frames')


def main():
    assert os.environ.get("PANTHERA_SIM_AUTHORIZED") == "1"
    assert sorted(p.name for p in Path("/sys/class/net").iterdir()) == ["lo"]
    assert os.environ.get("PANTHERA_PROBE_MODE") == "inspire-lab-debug"
    sys.path.insert(0, "/workspace/sim-source")
    sys.path.insert(0, "/workspace/ferox_tools")
    from isaac.twin.isaaclab.g1_inspire import (InspireLabSpec, TransitionWriter,
                                              evaluate_snapshot, LAB_SOURCE_SHA)
    from isaac.twin.inspire.whiteboard_scene import SceneConfig

    profile_path = Path(os.environ["PANTHERA_PROBE_CONFIG"])
    profile = json.loads(profile_path.read_text())
    assert profile["schema_version"] == 1 and profile["hardware_authorized"] is False
    settings = profile["lab"]
    assert set(settings) <= {"num_envs", "steps", "episode_steps", "seed", "scene_config", "capture_cameras"}
    num_envs, steps = settings["num_envs"], settings["steps"]
    episode_steps, seed = settings["episode_steps"], settings.get("seed", 17)
    assert type(num_envs) is int and 1 <= num_envs <= 8
    assert type(steps) is int and 4 <= steps <= 200
    assert type(episode_steps) is int and 2 <= episode_steps < steps
    assert type(seed) is int and 0 <= seed <= 2**31 - 1
    capture_cameras=settings.get('capture_cameras',False)
    capture_steps=camera_schedule(capture_cameras,steps,.02)
    scene = SceneConfig.from_dict(settings["scene_config"])
    actual_lab_sha = subprocess.check_output(["git", "-c", "safe.directory=/workspace/IsaacLab",
        "-C", "/workspace/IsaacLab", "rev-parse", "HEAD"], text=True).strip()
    assert actual_lab_sha == LAB_SOURCE_SHA, (actual_lab_sha, LAB_SOURCE_SHA)

    from isaaclab.app import AppLauncher
    launcher = AppLauncher({"headless": True, "device": "cpu", "enable_cameras": capture_cameras})
    app = launcher.app
    import numpy as np
    import torch
    import omni.usd
    from omni.physx import get_physxunittests_interface
    from pxr import PhysxSchema, UsdPhysics
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("omni.pip.compute")
    if capture_cameras:
        # The pinned Lab experience enables rendering but does not load the
        # standalone Isaac Camera extension's Python namespace by default.
        enable_extension("isaacsim.sensors.camera")
    from inspire_body_asset import import_body
    from inspire_collision import replace_palm_with_components, replace_left_thumb_with_slabs
    from rigid_inertia import audit_live_properties
    from isaac.twin.isaaclab.inspire_env import create_env

    out = Path("/evidence")
    source = Path("/source-assets/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf")
    def palm_builder(stage, mesh, body, side):
        return replace_palm_with_components(stage, mesh, body,
            contact_offset_m=.0012860533315688372, rest_offset_m=0.,
            candidate_id="ftp_palm_yz_slabs_v2" if side == "right" else "ftp_left_palm_yz_slabs_v1")

    asset, facts = import_body(source, out, fixed_base=True, palm_builder=palm_builder,
                               left_thumb_builder=replace_left_thumb_with_slabs)
    spec = InspireLabSpec.from_manifests(facts, profile, settings["scene_config"], episode_steps=episode_steps)
    omni.usd.get_context().new_stage()
    env = create_env(spec, asset, num_envs=num_envs, seed=seed)
    observation, extras = env.reset(seed=seed)
    assert observation["policy"].shape == (num_envs, 119)
    assert env.robot.num_joints == 53 and env.single_action_space.shape == (41,)
    assert set(env.robot.body_names) == set(facts["physical_link_mass_kg"])
    ids = env.measurement_ids
    live = env.robot.root_physx_view
    readbacks = {
        "stiffness_nm_rad": live.get_dof_stiffnesses()[:, ids].cpu().tolist(),
        "damping_nm_s_rad": live.get_dof_dampings()[:, ids].cpu().tolist(),
        "independent_drive_effort_limit_nm": live.get_dof_max_forces()[:, ids].cpu().tolist(),
        "body_names": list(env.robot.body_names), "mass_kg": live.get_masses().cpu().tolist()}
    for key, expected in [("stiffness_nm_rad", spec.stiffness), ("damping_nm_s_rad", spec.damping),
                          ("independent_drive_effort_limit_nm", spec.drive_effort_limits)]:
        assert np.allclose(readbacks[key], [expected] * num_envs, rtol=1e-6, atol=1e-6), key
    expected_mass = [facts["physical_link_mass_kg"][n] for n in env.robot.body_names]
    assert np.allclose(readbacks["mass_kg"], [expected_mass] * num_envs, atol=1e-5, rtol=1e-6)
    inertia_audits = [audit_live_properties(env.sim.physics_sim_view, f"/World/envs/env_{i}/Robot",
        facts["expected_source_rigid_properties_in_imported_frame"]) for i in range(num_envs)]
    (out / "live_inertia_audit.json").write_text(json.dumps(inertia_audits, indent=2, allow_nan=False))
    marker_live = env.marker.root_physx_view
    readbacks["marker_spring"] = {
        "stiffness_n_m": marker_live.get_dof_stiffnesses().cpu().tolist(),
        "damping_n_s_m": marker_live.get_dof_dampings().cpu().tolist(),
        "force_limit_n": marker_live.get_dof_max_forces().cpu().tolist(),
        "target_m": env.marker.data.joint_pos_target.cpu().tolist()}
    assert np.allclose(readbacks["marker_spring"]["stiffness_n_m"], scene.holder.stiffness_n_m)
    assert np.allclose(readbacks["marker_spring"]["target_m"], scene.holder.spring_target_m)
    readbacks["palms"] = []
    for environment in range(num_envs):
        for side in ("left", "right"):
            path = f"/World/envs/env_{environment}/Robot/{side}_base_link"
            rigid = env.sim.physics_sim_view.create_rigid_body_view(path)
            expected = facts["collision_candidates"][side]["expected_palm_hulls_if_all_cooking_succeeds"]
            assert rigid.count == 1 and rigid.max_shapes == expected, (path, rigid.count, rigid.max_shapes, expected)
            readbacks["palms"].append({"path": path, "live_shapes": rigid.max_shapes,
                "contact_offsets_m": rigid.get_contact_offsets().cpu().tolist(),
                "rest_offsets_m": rigid.get_rest_offsets().cpu().tolist()})
    readbacks["left_thumb"] = []
    for environment in range(num_envs):
        path = f"/World/envs/env_{environment}/Robot/left_thumb_2"
        rigid = env.sim.physics_sim_view.create_rigid_body_view(path)
        expected = facts["thumb_collision_candidates"]["left"]["expected_live_hulls"]
        assert rigid.count == 1 and rigid.max_shapes == expected
        readbacks["left_thumb"].append({"path": path, "live_shapes": rigid.max_shapes, "expected_shapes": expected})
    (out / "lab_runtime_contract.json").write_text(json.dumps(readbacks, indent=2, allow_nan=False))

    cameras={};frame_records=[];render_records=[];frame_file=None;render_file=None
    if capture_cameras:
        from PIL import Image
        from pxr import Gf,UsdGeom
        from isaacsim.sensors.camera import Camera
        plan=camera_plan(env.scene.env_origins.detach().cpu().tolist())
        (out/'camera_plan.json').write_text(json.dumps(plan,indent=2,allow_nan=False))
        for label,view in plan['views'].items():
            camera=Camera('/World/'+label+'LabCamera',resolution=tuple(view['resolution']))
            camera.initialize();camera.set_clipping_range(.01,100.)
            # Isaac camera setters use stage units (meters), not millimeters.
            camera.set_focal_length(view['focal_length_mm']/1000.);camera.set_horizontal_aperture(view['horizontal_aperture_mm']/1000.)
            xf=UsdGeom.Xformable(camera.prim);xf.ClearXformOpOrder()
            xf.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*view['position_world_m']),
                Gf.Vec3d(*view['target_world_m']),Gf.Vec3d(0.,0.,1.)).GetInverse())
            cameras[label]=camera;(out/'frames'/label).mkdir(parents=True)
        before=float(env.sim.current_time)
        for _ in range(8):env.sim.render()
        assert float(env.sim.current_time)==before
        frame_file=(out/'frames.jsonl').open('w',buffering=1)
        render_file=(out/'render_state.jsonl').open('w',buffering=1)

    writer = TransitionWriter(out / "transitions.jsonl")
    resets = [0] * num_envs
    failures, parity_errors = [], []
    invalid_state_records = 0
    invalid_contact_records = 0
    base_action = list(spec.reset_action())
    for step in range(steps):
        # Small diagnostic motion in a source-validated range; no reaching/grasp reward.
        action = list(base_action)
        for index in range(29, 41):
            name = spec.action_names[index]
            lo, hi = dict(zip(spec.measurement_names, spec.limits))[name]
            action[index] = min(hi, max(lo, .01 * (1. - math.cos(2. * math.pi * step / 20.))))
        action_tensor = torch.tensor(action, dtype=torch.float32, device=env.device).repeat(num_envs, 1)
        observation, reward, terminated, truncated, extras = env.step(action_tensor)
        assert observation["policy"].shape == (num_envs, 119)
        assert len(env.last_transition_records) == num_envs
        for i, record in enumerate(env.last_transition_records):
            # Re-evaluate the exported pre-reset measurements. Returned observations
            # may already belong to the next episode and must never replace these.
            expected = evaluate_snapshot(spec, record["q_rad"]["values"], record["dq_rad_s"]["values"],
                record["root_state_world_p_qwxyz_v_w"]["values"], record["command_position_rad"]["values"], record["episode_step"])
            same = expected == record["evaluation"] and bool(terminated[i]) == expected["terminated"] and bool(truncated[i]) == expected["truncated"]
            same = same and math.isclose(float(reward[i]), expected["reward"], rel_tol=1e-5, abs_tol=1e-6)
            if not same: parity_errors.append({"step": step, "env": i})
            if expected["failures"]: failures.append({"step": step, "env": i, "failures": expected["failures"]})
            invalid_state_records += int(not all(record[k]["valid"] for k in ["q_rad", "dq_rad_s", "root_state_world_p_qwxyz_v_w"]))
            invalid_contact_records += int(not record["contacts"]["valid"])
            resets[i] += int(bool(terminated[i]) or bool(truncated[i]))
            writer.append(record)
        if step in capture_steps:
            before=float(env.sim.current_time)
            before_frame={label:int(camera.get_current_frame()['rendering_frame']) for label,camera in cameras.items()}
            # Refresh reset articulation poses and fabric without a physics step.
            env.sim.forward();env.sim.render()
            assert float(env.sim.current_time)==before
            q=env.robot.data.joint_pos[:,ids].detach().cpu().tolist()
            dq=env.robot.data.joint_vel[:,ids].detach().cpu().tolist()
            roots=env.robot.data.root_state_w.detach().cpu().tolist()
            body_poses=env.robot.data.body_link_pose_w.detach().cpu().tolist()
            marker_roots=env.marker.data.root_state_w.detach().cpu().tolist()
            environments=[]
            for i in range(num_envs):
                environments.append({'env_id':i,'episode_id':int(env.episode_ids[i]),'episode_step':int(env.episode_length_buf[i]),
                    'measurement_joint_names':list(spec.measurement_names),'q_rad':q[i],'dq_rad_s':dq[i],
                    'root_state_world_p_qwxyz_v_w':roots[i],
                    'environment_origin_world_m':env.scene.env_origins[i].detach().cpu().tolist(),
                    'body_names':list(env.robot.body_names),'body_link_poses_world_p_qwxyz':body_poses[i],
                    'marker_root_state_world_p_qwxyz_v_w':marker_roots[i]})
            reset_flags=[bool(terminated[i]) or bool(truncated[i]) for i in range(num_envs)]
            rendered=render_state_record(step=step,physics_s=before,observation=observation['policy'].detach().cpu().tolist(),
                environments=environments,transitions=env.last_transition_records,reset_flags=reset_flags)
            files={};frame=len(frame_records);camera_receipts={}
            for label,camera in cameras.items():
                pixels=camera.get_rgba();assert pixels is not None and pixels.shape==(640,960,4)
                stamp=camera.get_current_frame()
                camera_receipts[label]={'rendering_frame':int(stamp['rendering_frame']),'rendering_time':float(stamp['rendering_time'])}
                name=f'frames/{label}/{frame:06d}.png';Image.fromarray(pixels.astype(np.uint8)).save(out/name);files[label]=name
            assert float(env.sim.current_time)==before
            validate_camera_receipts(before_frame,camera_receipts,before)
            record={'frame':frame,'sequence':step,'physics_s':before,'phase':rendered['phase'],
                'captured_after_same_step_render':True,'views':files,'state_file':'render_state.jsonl',
                'state_record_index':len(render_records),'reset_before_image':reset_flags,
                'render_episode_ids':[e['episode_id'] for e in environments],
                'render_episode_steps':[e['episode_step'] for e in environments],
                'camera_receipts':camera_receipts,
                'source_transitions':rendered['source_transitions']}
            render_records.append(rendered);frame_records.append(record)
            render_file.write(json.dumps(rendered,allow_nan=False)+'\n');frame_file.write(json.dumps(record,allow_nan=False)+'\n')
    writer.close()
    if frame_file is not None:frame_file.close();render_file.close()
    statistics = get_physxunittests_interface().get_physics_stats()
    physics_scene = next(p for p in env.sim.stage.Traverse() if p.IsA(UsdPhysics.Scene))
    physics_api = PhysxSchema.PhysxSceneAPI(physics_scene)
    checks = {"named53_measured_41_independent": True, "source_mass_gains_effort_preserved": True,
        "source_mass_com_inertia_preserved": all(all(a["checks"].values()) for a in inertia_audits),
        "bilateral_live_palm_shapes": True, "exact_transition_count": writer.count == num_envs * steps,
        "scalar_export_runtime_reward_done_parity": not parity_errors, "automatic_resets_observed_each_env": all(resets),
        "finite_actual_states": invalid_state_records == 0, "valid_contact_measurements": invalid_contact_records == 0,
        "mechanism_thresholds": not failures, "no_triangle_hand_substitution": statistics["numTriMeshShapes"] == 0}
    if capture_cameras:
        checks['paired_frames_match_declared_schedule']=len(frame_records)==len(capture_steps) and 0<len(frame_records)<=40
        checks['render_states_match_returned_observations']=len(render_records)==len(frame_records)
        checks['captured_world_clock_is_contiguous']=all(math.isclose(b['physics_s']-a['physics_s'],.1,abs_tol=1e-7)
            for a,b in zip(frame_records,frame_records[1:]))
    metrics = {"checks": checks, "lab_commit": actual_lab_sha, "spec_sha256": spec.sha256,
        "profile_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(), "scene_config_sha256": scene.sha256,
        "asset_source_sha256": facts["source_sha256"], "collision_candidate_ids": spec.candidate_ids,
        "measurement_names": spec.measurement_names, "action_names": spec.action_names, "observation_width": 119,
        "num_envs": num_envs, "seed": seed, "steps": steps, "automatic_reset_counts": resets,
        "physics_dt_s": spec.physics_dt, "control_dt_s": spec.physics_dt * spec.decimation,
        "physx_gpu_dynamics_enabled": physics_api.GetEnableGPUDynamicsAttr().Get(),
        "physx_statistics": statistics, "failures": failures, "parity_errors": parity_errors,
        "physics_scope": "fixed_pelvis_source_assembly_and_free_dynamic_marker_debug",
        "training_run": False, "checkpoint_created": False, "exact_asset_qualified": False,
        "standing_qualified": False, "grasp_qualified": False, "writing_qualified": False,
        "contact_scope": "synthetic_net_rigid_body_forces; no hardware taxels or pair impulses",
        "runtime_parity_scope": "shared evaluation versus actual exported pre-reset state; not bitwise independent-physics equivalence"}
    metrics.update(capture_cameras=capture_cameras,paired_frame_count=len(frame_records),
        media_state_file='render_state.jsonl' if capture_cameras else None,
        media_state_mapping='frames.state_record_index indexes render_state.jsonl; sequence is original control step',
        media_labels={'fixture':'FIXED PELVIS Isaac Lab clones; free dynamic markers and physical boards',
            'embodiment':f'{num_envs} PROVISIONAL G1 + bilateral FTP hands; seed{seed}',
            'qualification':'Lab plumbing/reset milestone only; no manipulation, grasp or standing qualification'})
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False))
    env.sim.stage.GetRootLayer().Export(str(out / "lab_scene.usda"))
    artifacts = [str(p.relative_to(out)) for p in out.rglob("*") if p.is_file() and p.name not in
        {"run.json", "probe.json", "console.log", "executed_probe.py", "executed_launcher.py", "uncommitted.patch"}]
    (out / "probe.json").write_text(json.dumps({"status": "PASS" if all(checks.values()) else "FAIL",
        "scope": "Isaac_Lab_source_specific_debug_only", "metrics": "metrics.json", "artifacts": artifacts}))
    env.close()
    app.close()


if __name__ == "__main__":
    main()
