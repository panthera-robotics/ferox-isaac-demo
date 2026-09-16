"""Actual-state, fixed-pelvis air task integration through one explicit body owner.

Private task implementation and calibration are imported only from explicit
read-only mounts. This public probe contains neither. There is no marker, board
contact, grasp or standing qualification. CPU imports do not start Isaac.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time
import traceback
import xml.etree.ElementTree as ET


def finite(value, name, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + ' must be finite numeric data')
    if (low is not None and value < low) or (high is not None and value > high):
        raise ValueError(name + ' exceeds its declared bound')
    return float(value)


def validate_config(config):
    required = {'schema_version', 'hardware_authorized', 'private_driver_path',
        'private_dependency_path', 'private_profile_path', 'planner_frames_path',
        'planner_frames_sha256', 'body_home_rad', 'kp_nm_rad', 'kd_nm_s_rad',
        'tau_ff_nm', 'gain_provenance', 'feedforward_provenance', 'workflow_mode',
        'letter_height_m', 'maximum_steps', 'maximum_wall_s'}
    if not isinstance(config, dict) or not required <= set(config) <= required | {'actuation_backend'}:
        raise ValueError('explicit assembled writer config fields required')
    if config.get('actuation_backend', 'explicit_pd') not in ('explicit_pd', 'implicit_biased_drive_v1'):
        raise ValueError('unknown versioned body actuation backend')
    if type(config['schema_version']) is not int or config['schema_version'] != 1 or config['hardware_authorized'] is not False:
        raise ValueError('simulator-only config version1 required')
    for key in ['private_driver_path', 'private_dependency_path', 'private_profile_path', 'planner_frames_path']:
        path = Path(config[key])
        if not path.is_absolute() or '..' in path.parts or path.parts[:2] != ('/', 'workspace'):
            raise ValueError('private input must be an explicit /workspace mount: ' + key)
    names = set(config['body_home_rad'])
    if len(names) != 29 or any(not isinstance(n, str) for n in names):
        raise ValueError('exact named29 body home required')
    for key in ['body_home_rad', 'kp_nm_rad', 'kd_nm_s_rad', 'tau_ff_nm']:
        if not isinstance(config[key], dict) or set(config[key]) != names:
            raise ValueError('named29 field mismatch: ' + key)
        for name, value in config[key].items():
            finite(value, key + '.' + name, 0. if key in ('kp_nm_rad', 'kd_nm_s_rad') else None,
                   200. if key == 'kp_nm_rad' else 5. if key == 'kd_nm_s_rad' else None)
    if config['workflow_mode'] not in ('complete', 'prefix_cancel'):
        raise ValueError('one explicit workflow required')
    finite(config['letter_height_m'], 'letter_height_m', .005, .08)
    finite(config['maximum_wall_s'], 'maximum_wall_s', 10., 600.)
    if type(config['maximum_steps']) is not int or not 200 <= config['maximum_steps'] <= 120000:
        raise ValueError('bounded maximum_steps required')
    for key in ('gain_provenance', 'feedforward_provenance'):
        if not isinstance(config[key], str) or not config[key].strip():
            raise ValueError('explicit provenance required')
    digest = config['planner_frames_sha256']
    if not isinstance(digest, str) or len(digest) != 64 or set(digest) - set('0123456789abcdef'):
        raise ValueError('planner frame artifact hash required')
    return config


def observed_checks(names, q, dq, effort, limits, mimic):
    """Fixed physical guards; do not soften these using a successful trajectory."""
    if len(names) != 53 or len(set(names)) != 53 or set(names) != set(limits):
        raise ValueError('observed articulation does not match exact named53 source')
    if any(len(values) != 53 for values in (q, dq, effort)):
        raise ValueError('observed state width mismatch')
    by_name = {}
    violation = speed = 0.
    for name, position, velocity, measured in zip(names, q, dq, effort):
        position, velocity, measured = [finite(v, name) for v in (position, velocity, measured)]
        limit = limits[name]
        violation = max(violation, limit['lower'] - position, position - limit['upper'], 0.)
        speed = max(speed, abs(velocity))
        if abs(velocity) > min(5., limit['velocity']):
            raise ValueError('observed velocity exceeded source/5rad_s guard: ' + name)
        by_name[name] = position
    if violation > .03:
        raise ValueError('observed joint limit violation exceeded .03rad')
    coupling = {name: by_name[name] - (m['multiplier'] * by_name[m['parent']] + m['offset'])
                for name, m in mimic.items()}
    if coupling and max(abs(v) for v in coupling.values()) > .03:
        raise ValueError('observed hand mimic error exceeded .03rad')
    return {'joint_limit_violation_rad': violation, 'maximum_velocity_rad_s': speed,
            'coupling_error_rad': coupling}


def pose_matrix(pose):
    import numpy as np
    if len(pose) != 7:
        raise ValueError('position plus xyzw quaternion required')
    px, py, pz, x, y, z, w = [finite(v, 'measured_pose') for v in pose]
    if not math.isclose(x*x + y*y + z*z + w*w, 1., abs_tol=1e-5):
        raise ValueError('measured quaternion must be unit length')
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = [px, py, pz]
    return result


def frame_error(expected, actual):
    import numpy as np
    expected, actual = np.asarray(expected, dtype=float), np.asarray(actual, dtype=float)
    for value in (expected, actual):
        if value.shape != (4, 4) or not np.isfinite(value).all() or not np.allclose(value[3], [0., 0., 0., 1.], atol=1e-8):
            raise ValueError('invalid homogeneous frame')
        if not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-6) or not math.isclose(np.linalg.det(value[:3, :3]), 1., abs_tol=1e-6):
            raise ValueError('frame rotation must be proper orthonormal')
    delta = expected[:3, :3].T @ actual[:3, :3]
    return {'translation_m': float(np.linalg.norm(actual[:3, 3]-expected[:3, 3])),
            'rotation_rad': float(np.arccos(np.clip((np.trace(delta)-1)/2, -1, 1)))}


def copy_writable_template(source,destination):
    """Copy immutable input bytes into a newly owned runtime directory."""
    source,destination=Path(source),Path(destination)
    if any(p.is_symlink() for p in source.rglob('*')):raise ValueError('Template symlinks are not admitted')
    shutil.copytree(source,destination)
    for path in [destination,*destination.rglob('*')]:
        path.chmod(0o755 if path.is_dir() else 0o644)


def main():
    if os.environ.get('PANTHERA_SIM_AUTHORIZED') != '1' or sorted(p.name for p in Path('/sys/class/net').iterdir()) != ['lo']:
        raise RuntimeError('isolated simulator authorization required')
    if os.environ.get('PANTHERA_PROBE_MODE') != 'assembled-writer-air':
        raise RuntimeError('explicit assembled-writer-air mode required')
    config_path = Path(os.environ['PANTHERA_PROBE_CONFIG'])
    cfg = validate_config(json.loads(config_path.read_text()))
    out = Path('/evidence')
    metrics = {'status': 'FAIL', 'scope': 'fixed_pelvis_actual_writer_air_integration',
        'hardware_authorized': False, 'fixed_base': True, 'body_final_writers': 1,
        'tool_attached': False, 'ground_present': False,
        'support_constraints': ['pelvis_fixed_to_world_1m_above_origin'],
        'writing_qualification': 'NOT_QUALIFIED', 'grasp_qualification': 'NOT_RUN',
        'standing_qualification': 'NOT_RUN', 'exact_asset_qualified': False,
        'physical_virtual_tip_meaning': 'kinematic reference only; no physical tool or ink',
        'physics_dt_s': .005, 'gpu_dynamics': False, 'checks': {},
        'profile_sha256': hashlib.sha256(config_path.read_bytes()).hexdigest(),
        'gain_provenance': cfg['gain_provenance'], 'feedforward_provenance': cfg['feedforward_provenance'],
        'media_labels': {'fixture': 'FIXED PELVIS - ACTUAL WRITER AIR TASK',
            'embodiment': 'PROVISIONAL G1 + bilateral FTP hands',
            'qualification': 'No marker, grasp, contact writing or standing qualification'}}
    app = world = bridge = subscription = None
    files = []
    started = time.monotonic()
    try:
        sys.path[:0] = ['/workspace/ferox_tools', '/workspace/ferox_isaac/twin',
                        str(Path(cfg['private_driver_path']) / 'src/g1_arm_tasks')]
        # Configure the real loopback namespace before enabling any ROS extension.
        from g1_arm_tasks.sim_transport import configure_environment, load_profile
        configure_environment()
        template_profile = Path(cfg['private_profile_path'])
        runtime = out/'writer-runtime'
        copy_writable_template(template_profile.parent, runtime)
        profile_data = json.loads(template_profile.read_text())
        template_root = template_profile.parent.resolve()
        # The mounted template describes files in its own directory. Rewrite
        # only run custody/paths; calibration/home bytes and their hashes stay.
        def copied_input(old):
            relative = Path(old).resolve().relative_to(template_root)
            return str(runtime/relative)
        profile_data.update(run_id=os.environ['PANTHERA_SIM_RUN_ID'],
            tool_board_path=copied_input(profile_data['tool_board_path']),
            pose_dir=copied_input(profile_data['pose_dir']), record_dir=str(runtime/'recording'))
        if profile_data.get('planner_model_path'):
            profile_data['planner_model_path'] = copied_input(profile_data['planner_model_path'])
        private_profile_path = runtime/'profile.json'
        private_profile_path.write_text(json.dumps(profile_data, indent=2, allow_nan=False))
        metrics.update(private_profile_template_sha256=hashlib.sha256(template_profile.read_bytes()).hexdigest(),
            private_profile_derived_sha256=hashlib.sha256(private_profile_path.read_bytes()).hexdigest(),
            private_profile_derivation='copied explicit template; only runtime paths and admitted run_id rewritten')
        task_profile = load_profile(private_profile_path)
        frames_path = Path(cfg['planner_frames_path'])
        if hashlib.sha256(frames_path.read_bytes()).hexdigest() != cfg['planner_frames_sha256']:
            raise ValueError('planner frame export hash mismatch')
        frames = json.loads(frames_path.read_text())
        planner = frames['full_model_fk']['commanded_waist']
        if planner['named_q_rad'] != cfg['body_home_rad'] or planner['tool_reference_frame'] != 'right_rubber_hand':
            raise ValueError('planner export must use the exact admitted named29 home/tool frame')
        if any(cfg['body_home_rad'][n] != v for n, v in task_profile.fixed_waist_rad.items()):
            raise ValueError('body home differs from approved fixed waist')
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting', 'fast_shutdown': False})
        import numpy as np
        from PIL import Image
        from pxr import Gf, PhysxSchema, PhysicsSchemaTools, Usd, UsdGeom, UsdLux, UsdPhysics
        from omni.physx import get_physx_simulation_interface, get_physxunittests_interface
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.sensors.camera import Camera
        enable_extension('isaacsim.ros2.bridge')
        enable_extension('omni.pip.compute')
        for _ in range(5):
            app.update()
        from g1_arm_tasks.sim_process_bridge import WriterProcessBridge
        from inspire.arm_adapter import NamedBodyArbiter, JointBound
        from inspire.implicit_arm_adapter import NamedBodyDriveArbiter
        implicit_backend = cfg.get('actuation_backend', 'explicit_pd') == 'implicit_biased_drive_v1'
        metrics['actuation_backend'] = 'implicit_biased_drive_v1' if implicit_backend else 'explicit_pd'
        from inspire_body_asset import import_body
        from inspire_collision import replace_palm_with_components, replace_left_thumb_with_slabs
        from rigid_inertia import audit_live_properties
        from urdf_kinematics import UrdfKinematics
        source = Path('/source-assets/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf')
        def palm_builder(stage, mesh, body, side):
            return replace_palm_with_components(stage, mesh, body,
                contact_offset_m=.0012860533315688372, rest_offset_m=0.,
                candidate_id='ftp_palm_yz_slabs_v2' if side == 'right' else 'ftp_left_palm_yz_slabs_v1')
        # Existing public helper owns source import, mass/inertia and mimic joints.
        asset, facts = import_body(source, out, fixed_base=True, palm_builder=palm_builder,
                                  left_thumb_builder=replace_left_thumb_with_slabs)
        body_names = list(cfg['body_home_rad'])
        if set(body_names) != set(facts['body_joint_names']):
            raise ValueError('config body names differ from actual source donor')
        limits = facts['joint_limits']
        for n, value in cfg['body_home_rad'].items():
            finite(value, n, limits[n]['lower'], limits[n]['upper'])
            finite(cfg['tau_ff_nm'][n], n+'.feedforward', -limits[n]['effort'], limits[n]['effort'])
        kinematics = UrdfKinematics(source)
        adjacent = {frozenset((j.find('parent').get('link'),j.find('child').get('link')))
                    for j in ET.parse(source).getroot().findall('joint')}
        donor_home = kinematics.transforms(cfg['body_home_rad'])
        comparison = frame_error(donor_home['right_wrist_yaw_link'], planner['right_wrist_yaw_link_T_pelvis'])
        comparison.update(frame_name='right_wrist_yaw_link', reference_frame='pelvis',
            donor_T_pelvis=donor_home['right_wrist_yaw_link'].tolist(),
            planner_T_pelvis=planner['right_wrist_yaw_link_T_pelvis'],
            separate_tool_T_wrist=planner['right_rubber_hand_T_right_wrist_yaw_link'],
            artifact_sha256=cfg['planner_frames_sha256'], translation_budget_m=.001,
            budget_scope='air_only_declared_model_mismatch_not_contact_writing',
            source_urdf_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
        (out/'planner_donor_frame_comparison.json').write_text(json.dumps(comparison, indent=2))
        if comparison['translation_m'] > .001 or comparison['rotation_rad'] > math.radians(.2):
            raise ValueError('planner/donor geometry exceeds declared air-only1mm/0.2deg budget')
        metrics['planner_donor_frame_comparison'] = comparison
        stage = Usd.Stage.Open(str(asset))
        for p in stage.Traverse():
            if p.IsA(UsdPhysics.RevoluteJoint):
                state = PhysxSchema.JointStateAPI.Apply(p, 'angular')
                state.CreatePositionAttr(math.degrees(cfg['body_home_rad'].get(p.GetName(), 0.)))
                state.CreateVelocityAttr(0.)
            if p.IsA(UsdPhysics.RevoluteJoint) and p.GetName() in body_names:
                drive = UsdPhysics.DriveAPI.Apply(p, 'angular')
                drive.CreateTypeAttr('force'); drive.CreateStiffnessAttr(0.); drive.CreateDampingAttr(0.)
                drive.CreateMaxForceAttr(limits[p.GetName()]['effort'])
                # The implicit backend receives its declared radian gains through the
                # tensor path after reset; the authored target must already be the
                # named home, otherwise PhysX's default zero target would yank every
                # body joint toward 0 on the first controlled step (writer05).
                drive.CreateTargetPositionAttr(math.degrees(cfg['body_home_rad'][p.GetName()]))
        stage.GetRootLayer().Save()
        # The pinned SimulationManager integrates during warm-up before it
        # exposes articulation handles. Use a declared microstep for those
        # uncontrolled initialization integrations, then restore the evaluated
        # 5ms timestep. No state teleport or temporary holding drive is used.
        initialization_dt = 1e-6
        world = World(stage_units_in_meters=1., physics_dt=initialization_dt, rendering_dt=.02)
        scene = next(p for p in world.stage.Traverse() if p.IsA(UsdPhysics.Scene))
        scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene)
        scene_api.CreateEnableGPUDynamicsAttr(False)
        scene_api.CreateEnableExternalForcesEveryIterationAttr(True)
        UsdLux.DomeLight.Define(world.stage, '/World/Fill').CreateIntensityAttr(600.)
        add_reference_to_stage(str(asset), '/World/G1')
        root = UsdGeom.Xformable(world.stage.GetPrimAtPath('/World/G1'))
        root.ClearXformOpOrder(); root.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1))
        for p in world.stage.Traverse():
            if p.HasAPI(PhysxSchema.PhysxArticulationAPI):
                api = PhysxSchema.PhysxArticulationAPI(p)
                api.CreateSolverPositionIterationCountAttr(32); api.CreateSolverVelocityIterationCountAttr(8)
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
        contact_file = (out/'contacts.jsonl').open('w', buffering=1); files.append(contact_file)
        sample_number = -1
        contact_count = 0
        collision_faults = []
        def on_contact(headers, data):
            nonlocal contact_count
            for h in headers:
                for i in range(h.contact_data_offset, h.contact_data_offset+h.num_contact_data):
                    d = data[i]
                    actor0, actor1 = str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1))
                    record = {'sequence': sample_number, 'physics_s': float(world.current_time),
                        'actor0': actor0, 'actor1': actor1,
                        'position_world_m': list(map(float, d.position)), 'normal_world': list(map(float, d.normal)),
                        'impulse_ns': list(map(float, d.impulse)), 'separation_m': float(d.separation),
                        'source': 'actual_PhysX_contact_report_simulated_proxy'}
                    pair = frozenset((Path(actor0).name, Path(actor1).name))
                    if len(pair)==2 and pair not in adjacent and (record['separation_m'] < -.001 or
                            sum(v*v for v in record['impulse_ns'])**.5 > .025):
                        collision_faults.append(record)
                    contact_file.write(json.dumps(record, allow_nan=False)+'\n')
                    contact_count += 1
        subscription = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
        robot = SingleArticulation('/World/G1', name='assembled_writer_fixture')
        world.reset(); robot.initialize()
        world.set_simulation_dt(physics_dt=.005, rendering_dt=.02)
        if not math.isclose(world.get_physics_dt(), .005, rel_tol=0, abs_tol=1e-12):
            raise ValueError('controlled physics timestep differs from declared5ms')
        metrics['initialization'] = {'warmup_physics_dt_s':initialization_dt,
            'controlled_physics_dt_s':float(world.get_physics_dt()),
            'runtime_physics_step_count_after_warmup':int(SimulationManager.get_num_physics_steps()),
            'world_time_after_warmup_s':float(world.current_time),
            'source':'pinned Isaac5.1 SimulationManager performs gravity integration before articulation handles exist',
            'post_reset_state_writes':False, 'temporary_body_holding_drives':False}
        names = list(robot.dof_names)
        if len(names) != 53 or set(names) != set(limits):
            raise ValueError('actual runtime named53 map differs from donor')
        indices = {n: names.index(n) for n in body_names}
        body_ids = np.array([indices[n] for n in body_names], dtype=np.int32)
        hand_names = facts['hand_independent_names']
        hand_ids = np.array([names.index(n) for n in hand_names], dtype=np.int32)
        kp, kd = np.zeros(53, dtype=np.float32), np.zeros(53, dtype=np.float32)
        kp[hand_ids], kd[hand_ids] = 1., .05
        if implicit_backend:
            # Declared body gains live inside the capped implicit drives (same
            # radian tensor path as the hands); no explicit body effort follows.
            kp[body_ids] = [cfg['kp_nm_rad'][n] for n in body_names]
            kd[body_ids] = [cfg['kd_nm_s_rad'][n] for n in body_names]
        robot._articulation_view.set_gains(kp, kd)
        gains = robot.get_articulation_controller().get_gains()
        live_caps = np.ravel(robot._articulation_view.get_max_efforts()).astype(float)
        gain_receipt = {'source': 'live_articulation_gain_readback_before_body_writes',
            'backend': metrics['actuation_backend'],
            'runtime_names': names, 'kp': np.ravel(gains[0]).tolist(), 'kd': np.ravel(gains[1]).tolist(),
            'max_efforts': live_caps.tolist(), 'body_indices': indices, 'hand_root_names': hand_names}
        (out/'implicit_gain_readback.json').write_text(json.dumps(gain_receipt, indent=2))
        if not np.array_equal(np.ravel(gains[0]), kp) or not np.array_equal(np.ravel(gains[1]), kd):
            raise ValueError('implicit gains differ from declared body/hand gains')
        if any(abs(live_caps[indices[n]] - limits[n]['effort']) > 1e-6 for n in body_names):
            raise ValueError('live body drive caps differ from source effort limits')
        gain_writes = 0
        if implicit_backend:
            # One pre-loop implicit target write: exact named home, no bias yet.
            home_targets = np.asarray([cfg['body_home_rad'][n] for n in body_names], dtype=np.float32)
            robot.apply_action(ArticulationAction(joint_positions=home_targets, joint_velocities=np.zeros(29, dtype=np.float32), joint_indices=body_ids))
            pre_targets = np.asarray(robot._articulation_view._physics_view.get_dof_position_targets()).reshape(-1)
            if pre_targets.shape != (53,) or not np.array_equal(pre_targets[body_ids], home_targets):
                raise ValueError('pre-loop implicit home target readback differs from named home')
            gain_receipt['pre_loop_implicit_home_target_rad'] = home_targets.astype(float).tolist()
            gain_receipt['pre_loop_target_readback_rad'] = pre_targets.astype(float).tolist()
            (out/'implicit_gain_readback.json').write_text(json.dumps(gain_receipt, indent=2))
        q0 = np.zeros(53, dtype=np.float32)
        q0[body_ids] = [cfg['body_home_rad'][n] for n in body_names]
        # JointState was authored before physics initialization. No body pose,
        # position or velocity setters are called after reset; the implicit
        # backend's only post-reset write is the declared home drive target above.
        initial_q = np.ravel(robot.get_joint_positions())
        (out/'initial_joint_state.json').write_text(json.dumps({'names':names,
            'authored_pre_reset_q_rad':q0.astype(float).tolist(), 'actual_q_rad':initial_q.astype(float).tolist(),
            'actual_dq_rad_s':np.ravel(robot.get_joint_velocities()).astype(float).tolist(),
            'initialization':metrics['initialization'],
            'maximum_initial_error_rad':float(np.max(np.abs(initial_q-q0)))}, indent=2))
        if np.max(np.abs(initial_q-q0)) > .001:
            raise ValueError('pre-reset joint-state preload differs from actual initial state')
        sim_view = SimulationManager.get_physics_sim_view()
        inertia = audit_live_properties(sim_view, '/World/G1', facts['expected_source_rigid_properties_in_imported_frame'])
        (out/'live_inertia_audit.json').write_text(json.dumps(inertia, indent=2, allow_nan=False))
        if not all(inertia['checks'].values()):
            raise ValueError('source/import/live mass COM inertia audit failed before actuation')
        views = {n: sim_view.create_rigid_body_view('/World/G1/'+n) for n in
                 ['pelvis', 'right_wrist_yaw_link', 'left_wrist_yaw_link', 'right_base_link', 'left_base_link']}
        if any(v.count != 1 for v in views.values()):
            raise ValueError('named link pose readback absent/ambiguous')
        stats = get_physxunittests_interface().get_physics_stats()
        shapes = {'statistics': stats, 'palms': {}}
        left_thumb = sim_view.create_rigid_body_view('/World/G1/left_thumb_2')
        left_thumb_expected = facts['thumb_collision_candidates']['left']['expected_live_hulls']
        shapes['left_thumb'] = {'live_shape_count':left_thumb.max_shapes, 'expected_shape_count':left_thumb_expected}
        if left_thumb.count != 1 or left_thumb.max_shapes != left_thumb_expected:
            raise ValueError('repaired left thumb live hull count differs from declared candidate')
        for side in ('left', 'right'):
            v = views[side+'_base_link']
            expected = facts['collision_candidates'][side]['expected_palm_hulls_if_all_cooking_succeeds']
            shapes['palms'][side] = {'live_shape_count': v.max_shapes, 'expected_shape_count': expected,
                'contact_offsets_m': np.asarray(v.get_contact_offsets()).tolist(),
                'rest_offsets_m': np.asarray(v.get_rest_offsets()).tolist()}
            if v.max_shapes != expected:
                raise ValueError('live palm shape count differs from declared candidate')
        if stats['numTriMeshShapes'] != 0:
            raise ValueError('unexpected live triangle collision shapes')
        (out/'backend_shapes.json').write_text(json.dumps(shapes, indent=2))
        cameras = {}
        for label, position in [('front',(2.2,-2.3,1.8)),('side',(-.3,2.9,1.65))]:
            camera = Camera('/World/'+label+'AirCamera', resolution=(640,640))
            camera.initialize(); camera.set_clipping_range(.01,10.)
            camera_x=UsdGeom.Xformable(camera.prim); camera_x.ClearXformOpOrder()
            camera_x.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*position),Gf.Vec3d(.1,0,1.05),Gf.Vec3d(0,0,1)).GetInverse())
            (out/'frames'/label).mkdir(parents=True)
            cameras[label]=camera
        # Allocate the annotators before starting the independently timed ROS
        # writer. Render warm-up must not advance the measured physics clock.
        camera_warmup_time = float(world.current_time)
        for _ in range(8):
            world.render()
        if float(world.current_time) != camera_warmup_time:
            raise ValueError('camera initialization advanced physics unexpectedly')
        frame_file = (out/'frames.jsonl').open('w', buffering=1); files.append(frame_file)
        state_file = (out/'state.jsonl').open('w', buffering=1); files.append(state_file)
        bridge = WriterProcessBridge(private_profile_path, out/'writer_bridge',
            dependency_path=cfg['private_dependency_path'], approved_fixture=True,
            letter_height_m=cfg['letter_height_m'], state_provenance='physx_measured_joint_effort')
        bounds = {n: JointBound(limits[n]['lower'], limits[n]['upper'], limits[n]['velocity'], 200., 5., limits[n]['effort']) for n in body_names}
        if implicit_backend:
            arbiter = NamedBodyDriveArbiter(body_indices=indices, bounds=bounds,
                simulator_id=task_profile.simulator_id, run_id=task_profile.run_id, mode='hybrid',
                controller_id='fixed_pelvis_home_fixture', simulation_authorized=True,
                explicit_efforts_disabled=True, source_caps_verified=True)
        else:
            arbiter = NamedBodyArbiter(body_indices=indices, bounds=bounds,
                simulator_id=task_profile.simulator_id, run_id=task_profile.run_id, mode='hybrid',
                controller_id='fixed_pelvis_home_fixture', simulation_authorized=True, implicit_drives_disabled=True)
        explicit_body_effort_observed = 0
        base = {n: {'q': cfg['body_home_rad'][n], 'dq': 0., 'kp': cfg['kp_nm_rad'][n],
                    'kd': cfg['kd_nm_s_rad'][n], 'tau': cfg['tau_ff_nm'][n]} for n in body_names}
        metrics.update(runtime_names=names, source_mass_kg=facts['source_physical_mass_kg'],
            steps=0, effective_body_writes=0, returned_references=0)
        maximum_error = maximum_rotation = maximum_coupling = maximum_waist = 0.
        workflow = None
        next_step_wall = time.monotonic()
        for sample_number in range(cfg['maximum_steps']):
            if time.monotonic()-started > cfg['maximum_wall_s']:
                raise TimeoutError('admitted experiment wall-time exceeded')
            wait = next_step_wall-time.monotonic()
            if wait > 0:
                time.sleep(wait)
            next_step_wall = max(next_step_wall+.005, time.monotonic())
            if sample_number and not world.is_playing():
                arbiter.check_freshness(time.monotonic())
                raise RuntimeError('physics pause requires stopping this admitted run')
            robot.apply_action(ArticulationAction(joint_positions=np.zeros(12, dtype=np.float32), joint_indices=hand_ids))
            world.step(render=False)
            q = np.ravel(robot.get_joint_positions()).astype(float).tolist()
            dq = np.ravel(robot.get_joint_velocities()).astype(float).tolist()
            effort = np.ravel(robot.get_measured_joint_efforts()).astype(float).tolist()
            poses = {n: np.asarray(v.get_transforms())[0].astype(float).tolist() for n, v in views.items()}
            observed_wall = time.monotonic(); sim_time = float(world.current_time)
            # Raw readback is retained before any aborting physical guard.
            record = {'sequence': sample_number, 'physics_s': sim_time, 'source_monotonic_s': observed_wall,
                'runtime_names': names, 'q_rad': q, 'dq_rad_s': dq, 'measured_generalized_effort_nm': effort,
                'link_poses_world_xyzw': poses, 'workflow': workflow,
                'phase':workflow['stage'] if workflow is not None else 'initial_measured_state'}
            state_file.write(json.dumps(record, allow_nan=True)+'\n')
            metrics['steps'] += 1
            if collision_faults:
                metrics['material_nonadjacent_collision_faults'] = collision_faults[:100]
                raise ValueError('material nonadjacent self-collision before body write: penetration>1mm or impulse>0.025Ns')
            checks = observed_checks(names, q, dq, effort, limits, facts['mimic_map'])
            maximum_coupling = max(maximum_coupling, max(map(abs, checks['coupling_error_rad'].values()), default=0.))
            pelvis = pose_matrix(poses['pelvis'])
            pelvis_rotation = np.eye(4)
            pelvis_rotation[:3, :3] = pelvis[:3, :3]
            if np.linalg.norm(pelvis[:3, 3]-[0., 0., 1.]) > .0001 or frame_error(np.eye(4), pelvis_rotation)['rotation_rad'] > 1e-4:
                raise ValueError('fixed pelvis moved from declared level pose')
            if sample_number == 0 or sample_number % 20 == 0:
                source_fk = kinematics.transforms(dict(zip(names, q)), pelvis)
                errors = {n: frame_error(source_fk[n], pose_matrix(poses[n])) for n in views if n != 'pelvis'}
                maximum_error = max(maximum_error, max(v['translation_m'] for v in errors.values()))
                maximum_rotation = max(maximum_rotation, max(v['rotation_rad'] for v in errors.values()))
                with (out/'source_fk_checks.jsonl').open('a') as stream:
                    stream.write(json.dumps({'sequence': sample_number, 'physics_s': sim_time, 'errors': errors})+'\n')
                if maximum_error > .0002 or maximum_rotation > math.radians(.2):
                    raise ValueError('actual/source FK exceeds .2mm/.2deg budget before body write')
            position = {n: q[indices[n]] for n in body_names}
            velocity = {n: dq[indices[n]] for n in body_names}
            measured = {n: effort[indices[n]] for n in body_names}
            waist = sum(abs(position[n]-v) for n, v in task_profile.fixed_waist_rad.items())
            maximum_waist = max(maximum_waist, waist)
            metrics.update(max_source_fk_position_error_m=maximum_error, max_source_fk_rotation_error_rad=maximum_rotation,
                coupling_error_max_rad=maximum_coupling, fixed_waist_l1_error_max_rad=maximum_waist,
                actual_contact_points=contact_count, workflow=workflow)
            rotation = pelvis[:3, :3]
            rpy = [math.atan2(rotation[2,1],rotation[2,2]), math.asin(max(-1.,min(1.,-rotation[2,0]))), math.atan2(rotation[1,0],rotation[0,0])]
            bridge.publish_state(sequence=sample_number, sim_time_s=sim_time, position=position, velocity=velocity,
                measured_effort=measured, imu_rpy_rad=rpy,
                pelvis_pose_world={'position_m':poses['pelvis'][:3], 'orientation_qwxyz':[poses['pelvis'][6]]+poses['pelvis'][3:6]},
                physics_dt_s=.005, source_monotonic_time_s=observed_wall)
            arbiter.observe_physics(sequence=sample_number, sim_time_s=sim_time, source_monotonic_s=observed_wall,
                position=position, velocity=velocity, now_monotonic_s=time.monotonic())
            references = bridge.spin_and_get_reference()
            for reference in references:
                arbiter.accept_upper_reference(reference, now_monotonic_s=time.monotonic())
            metrics['returned_references'] += len(references)
            workflow = bridge.workflowadvance(mode=cfg['workflow_mode'], text='I')
            if workflow['finished']:
                break
            if implicit_backend:
                drive = arbiter.compose_drives(base, controller_id='fixed_pelvis_home_fixture', physics_sequence=sample_number, now_monotonic_s=time.monotonic())
                drive_ids = np.asarray(drive.articulation_indices, dtype=np.int32)
                new_kp, new_kd = kp.copy(), kd.copy()
                new_kp[drive_ids] = drive.stiffness_nm_rad; new_kd[drive_ids] = drive.damping_nm_s_rad
                if not np.array_equal(new_kp, kp) or not np.array_equal(new_kd, kd):
                    robot._articulation_view.set_gains(new_kp, new_kd); kp, kd = new_kp, new_kd; gain_writes += 1
                    readback = robot.get_articulation_controller().get_gains()
                    if not np.array_equal(np.ravel(readback[0]), kp) or not np.array_equal(np.ravel(readback[1]), kd):
                        raise ValueError('blended implicit gain readback differs from the composed drive')
                targets = np.asarray(drive.target_position_rad, dtype=np.float32)
                target_velocities = np.asarray(drive.target_velocity_rad_s, dtype=np.float32)
                # Exactly ONE implicit named29 body target write for this sample; zero explicit body effort.
                robot.apply_action(ArticulationAction(joint_positions=targets, joint_velocities=target_velocities, joint_indices=drive_ids))
                tensor = robot._articulation_view._physics_view
                target_readback = np.asarray(tensor.get_dof_position_targets()).reshape(-1)
                live_efforts = np.asarray(tensor.get_dof_actuation_forces()).reshape(-1)
                if target_readback.shape != (53,) or not np.array_equal(target_readback[drive_ids], targets):
                    raise ValueError('implicit target readback differs from the composed drive')
                if np.any(live_efforts[body_ids] != 0.):
                    explicit_body_effort_observed += 1
                metrics['effective_body_writes'] += 1
                bridge.record_effective_command(physics_sequence=sample_number, sim_time_s=sim_time,
                    final_owner=drive.final_writer,
                    joints={n:{'implicit_target_rad':float(t), 'implicit_target_velocity_rad_s':float(v), 'kp_nm_rad':float(k), 'kd_nm_s_rad':float(d),
                               'feedforward_bias_rad':float(b), 'current_pd_estimate_nm':float(e)}
                            for n,t,v,k,d,b,e in zip(drive.joint_names,targets,target_velocities,drive.stiffness_nm_rad,drive.damping_nm_s_rad,
                                                     drive.feedforward_target_bias_rad,drive.current_pd_estimate_nm)},
                    metadata={'applied_to_physics': True, 'applies_to_next_physics_step': True,
                              'actuation_semantics': drive.actuation_semantics,
                              'explicit_body_effort_written': False, 'source_effort_caps_nm': list(drive.source_effort_caps_nm),
                              'reference_owners': list(drive.reference_owners), 'implicit_body_gains': 'declared_kp_kd_in_capped_drive',
                              'gain_writes_so_far': gain_writes})
            else:
                effective = arbiter.compose(base, controller_id='fixed_pelvis_home_fixture', physics_sequence=sample_number, now_monotonic_s=time.monotonic())
                # Exactly ONE explicit named29 body effort write for this sample.
                applied_efforts = np.asarray(effective.effort_nm, dtype=np.float32)
                robot.apply_action(ArticulationAction(joint_efforts=applied_efforts,
                    joint_indices=np.asarray(effective.articulation_indices, dtype=np.int32)))
                metrics['effective_body_writes'] += 1
                bridge.record_effective_command(physics_sequence=sample_number, sim_time_s=sim_time,
                    final_owner=effective.final_writer, joints={n:{'effort_nm':float(v)} for n,v in zip(effective.joint_names,applied_efforts)},
                    metadata={'applied_to_physics': True, 'applies_to_next_physics_step': True,
                              'arbiter_pre_float32_effort_nm': list(effective.effort_nm),
                              'reference_owners': list(effective.reference_owners), 'implicit_body_gains': 'verified_zero'})
            if (sample_number+1) % 20 == 0:
                # Same held-physics capture interval as the balance probe: the
                # first frame follows 20 controlled steps, never the first 5ms.
                capture_time = float(world.current_time); world.render()
                frame = (sample_number+1)//20-1
                frame_views = {}
                for label, camera in cameras.items():
                    pixels = camera.get_rgba(); extra_renders = 0
                    while (pixels is None or pixels.shape != (640,640,4)) and extra_renders < 3:
                        world.render(); extra_renders += 1; pixels = camera.get_rgba()
                    if float(world.current_time) != capture_time:
                        raise ValueError('camera capture advanced physics')
                    metrics['camera_extra_held_renders'] = max(metrics.get('camera_extra_held_renders', 0), extra_renders)
                    if pixels is None or pixels.shape != (640,640,4):
                        raise ValueError('actual camera frame absent after bounded held renders: '+label)
                    filename='frames/%s/%06d.png'%(label,frame)
                    Image.fromarray(pixels.astype(np.uint8)).save(out/filename)
                    frame_views[label]=filename
                frame_file.write(json.dumps({'frame':frame, 'sequence':sample_number, 'physics_s':sim_time,
                    'phase':record['phase'], 'captured_after_same_step_render':True, 'views':frame_views})+'\n')
            metrics.update(max_source_fk_position_error_m=maximum_error, max_source_fk_rotation_error_rad=maximum_rotation,
                coupling_error_max_rad=maximum_coupling, fixed_waist_l1_error_max_rad=maximum_waist,
                actual_contact_points=contact_count, workflow=workflow)
        else:
            raise TimeoutError('admitted maximum steps ended before actual workflow completion')
        metrics['workflow'] = workflow
        final_gains = robot.get_articulation_controller().get_gains()
        final_caps = np.ravel(robot._articulation_view.get_max_efforts()).astype(float)
        metrics['actuation_receipt'] = {'backend': metrics['actuation_backend'], 'gain_writes': gain_writes,
            'explicit_body_effort_samples': explicit_body_effort_observed,
            'final_kp': np.ravel(final_gains[0]).tolist(), 'final_kd': np.ravel(final_gains[1]).tolist(),
            'final_max_efforts': final_caps.tolist(),
            'final_gains_match_last_composed': bool(np.array_equal(np.ravel(final_gains[0]), kp) and np.array_equal(np.ravel(final_gains[1]), kd)),
            'source_caps_unchanged': bool(np.array_equal(final_caps, live_caps))}
        metrics['checks'] = {'actual_workflow_finished': bool(workflow and workflow['finished']),
            'body_writes_observed': metrics['effective_body_writes'] > 0,
            'actual_writer_references_observed': metrics['returned_references'] > 0,
            'live_inertia_preserved': all(inertia['checks'].values()),
            'body_actuation_ownership_verified': (explicit_body_effort_observed == 0 and metrics['actuation_receipt']['final_gains_match_last_composed']
                                                  and metrics['actuation_receipt']['source_caps_unchanged']) if implicit_backend else True,
            'actual_source_fk_within_budget': maximum_error <= .0002,
            'hand_coupling_within_budget': maximum_coupling <= .03,
            'full_job_complete': bool(workflow and workflow['full_job_completion']) if cfg['workflow_mode']=='complete' else True}
        if implicit_backend:
            metrics['checks']['implicit_body_gains_match_declared'] = bool(gain_receipt['kp'][indices[n]] == cfg['kp_nm_rad'][n]
                and gain_receipt['kd'][indices[n]] == cfg['kd_nm_s_rad'][n] for n in body_names)
        else:
            metrics['checks']['implicit_body_gains_zero'] = True
        metrics['status'] = 'PASS' if all(metrics['checks'].values()) else 'FAIL'
    except BaseException:
        metrics['error'] = traceback.format_exc()
        print(metrics['error'], flush=True)
    finally:
        cleanup_errors = []
        try:
            (out/'metrics.json').write_text(json.dumps(dict(metrics,status='FAIL',evaluation_status=metrics['status'],
                cleanup_complete=False,wall_seconds=time.monotonic()-started),indent=2,allow_nan=False))
            (out/'probe.json').write_text(json.dumps({'status':'FAIL','scope':metrics['scope'],
                'metrics':'metrics.json','reason':'cleanup_completion_pending','artifacts':['metrics.json']}))
        except BaseException as error:
            cleanup_errors.append('initial shutdown receipt: '+repr(error))
        # Dropping the carb.Subscription unsubscribes before physics shutdown;
        # no contact callback can race a subsequently closed output stream.
        subscription = None
        physics_stopped = world is None
        if world is not None:
            try:
                world.stop(); physics_stopped = True
            except BaseException as error:
                cleanup_errors.append('world.stop: '+repr(error))
                if app is not None:
                    try:
                        app.close(); app = None; physics_stopped = True
                    except BaseException as close_error:
                        cleanup_errors.append('early app.close: '+repr(close_error))
        if bridge is not None:
            try:
                if not physics_stopped:
                    raise RuntimeError('physics stop could not be established; refusing false shutdown receipt')
                metrics['bridge_shutdown'] = bridge.shutdown_receipt(physics_stopped=True)
                if metrics['bridge_shutdown']['status'] != 'STOPPED':
                    cleanup_errors.append('writer bridge shutdown did not return STOPPED')
            except BaseException as error:
                cleanup_errors.append('bridge shutdown: '+repr(error))
                if bridge.process is not None and bridge.process.poll() is None:
                    try:
                        bridge.process.terminate(); bridge.process.wait(timeout=12.)
                    except BaseException as process_error:
                        cleanup_errors.append('owned writer process termination: '+repr(process_error))
        for stream in files:
            try:
                stream.close()
            except BaseException as error:
                cleanup_errors.append('output close: '+repr(error))
        # Persist an explicitly incomplete receipt before Kit tears down its
        # framework. A process exit during close cannot leave a raw PASS.
        pending = dict(metrics, status='FAIL', evaluation_status=metrics['status'],
                       cleanup_complete=False, cleanup_errors=cleanup_errors,
                       wall_seconds=time.monotonic()-started)
        try:
            (out/'metrics.json').write_text(json.dumps(pending, indent=2, allow_nan=False))
            (out/'probe.json').write_text(json.dumps({'status':'FAIL','scope':metrics['scope'],
                'metrics':'metrics.json','reason':'cleanup_completion_pending','artifacts':['metrics.json']}))
        except BaseException as error:
            cleanup_errors.append('pending receipt write: '+repr(error))
        if app is not None:
            try:
                app.close()
            except BaseException as error:
                cleanup_errors.append('app.close: '+repr(error))
        metrics['cleanup_errors'] = cleanup_errors
        metrics['cleanup_complete'] = True
        if cleanup_errors:
            metrics['status'] = 'FAIL'
        metrics['wall_seconds'] = time.monotonic()-started
        (out/'metrics.json').write_text(json.dumps(metrics, indent=2, allow_nan=False))
        artifacts = [str(p.relative_to(out)) for p in out.rglob('*') if p.is_file() and p.name not in
                     ['run.json','probe.json','console.log','executed_probe.py','executed_launcher.py','uncommitted.patch']]
        (out/'probe.json').write_text(json.dumps({'status':metrics['status'], 'scope':metrics['scope'],
            'metrics':'metrics.json', 'artifacts':artifacts}))
    return 0 if metrics['status']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
