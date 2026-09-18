"""Bare-versus-donor unsupported standing probe: supported settle, recorded release, hold.

Arm 'bare' imports the g1_omni training embodiment (g1_29dof_rev_1_0 with rubber hands);
arm 'donor' imports the assembled FTP-donor twin. Same policy, adapter, drives, physics
step, ground, gates and initialization path. The pelvis is held by an explicit, measured
PD wrench (the rig) during settling; the rig is removed at one recorded physics sequence
and nothing supports the body afterwards. The named policy adapter is the only actuator
writer; hands are commanded to the declared operating margin, never to exact zero.
"""
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import traceback


def main():
    assert os.environ.get('PANTHERA_SIM_AUTHORIZED') == '1'
    assert sorted(p.name for p in Path('/sys/class/net').iterdir()) == ['lo']
    assert os.environ.get('PANTHERA_PROBE_MODE') == 'wbc-standing-ab'
    sys.path[:0] = ['/workspace/ferox_tools', '/workspace/sim-source', '/workspace/ferox_isaac']
    from assembled_balance import GATES, FEET, GROUND, source_foot_spheres, initial_height, roll_pitch, finite_tree, safe_json, source_adjacency
    from wbc_standing_ab import (ARMS, ARM_JOINTS, EXPERIMENTAL_OWNER, StandingABConfig, config_sha256, perturbed_initial, rig_wrench,
                                 evaluate_standing, command_at, arm_reference_at, guarded_state_view, rig_stability_margins, rig_scale_at, interval_persists)
    from wbc_runtime_guard import GuardRefused, RuntimeOwnershipGuard
    cfg = StandingABConfig.from_dict(json.loads(Path(os.environ['PANTHERA_PROBE_CONFIG']).read_text()))
    arm = ARMS[cfg.arm]
    sys.path.insert(0, cfg.locomotion_path)
    out = Path('/evidence'); app = None; world = None; subscription = None; streams = []
    rows = []; contacts = []; step_contacts = []; contact_faults = []; sequence = None; events = []
    phase = 'configuration'; last = {}; initial = {}; facts = {}; abort = None; integrity = {}
    media = {'fixture': ('TRAINING-RESET START (spawned in the default pose, no rig at any time, policy from step 0); free standing on physical ground' if cfg.training_reset else 'RIG-SUPPORTED SETTLE then RECORDED RELEASE; free standing on physical ground afterwards'),
             'embodiment': arm['label'], 'qualification': 'SIM-only zero-command standing A/B; no firmware or manipulation qualification'}
    scope = {'fixed_base': False, 'support_constraints': [], 'external_forces': 'rig wrench during supported_settle only; zero after release',
             'pose_writes_after_reset': 0, 'joint_state_writes_after_reset': 0, 'exact_E2_qualified': False,
             'firmware_qualified': False, 'hardware_authorized': False, 'grasp_qualification': 'NOT_RUN',
             'writing_qualification': 'NOT_RUN', 'arm': cfg.arm, 'source': arm['label'],
             'physics': 'PhysX CPU dynamics with GPU rendering', 'execution_label': cfg.execution_label}
    def write(name, value):
        (out / name).write_text(json.dumps(safe_json(value), indent=2, allow_nan=False) + '\n')
    def receipt(metrics):
        metrics.update(scope=scope, media_labels=media, gates=GATES, phase=phase, config=asdict(cfg), config_sha256=config_sha256(cfg))
        write('metrics.json', metrics)
        artifacts = [str(p.relative_to(out)) for p in out.rglob('*') if p.is_file() and p.name not in
                     ['run.json', 'probe.json', 'console.log', 'executed_probe.py', 'executed_launcher.py', 'uncommitted.patch']]
        write('probe.json', {'status': metrics['status'], 'scope': 'unsupported_standing_ab_%s' % cfg.arm,
                             'metrics': 'metrics.json', 'artifacts': artifacts})
    write('standing_ab_config.json', {'config': asdict(cfg), 'config_sha256': config_sha256(cfg), 'arm': arm, 'gates': GATES,
                                      'scope': scope, 'gates_authored_before_physics': True})
    try:
        from g1_policy.contract import PolicyContract
        from g1_policy.isaac_adapter import named_policy_class
        import g1_policy.isaac_adapter as adapter_module
        import numpy as np
        from urdf_kinematics import UrdfKinematics
        source = Path('/source-assets') / arm['source_urdf']
        assert source.is_file(), source
        contract = PolicyContract.load(cfg.policy_path)
        assert contract.observation_width == 480 and len(contract.policy_names) == 29
        default = dict(zip(contract.policy_names, contract.default_position))
        kinematics = UrdfKinematics(source); zero_fk = kinematics.transforms({}); posed_fk = kinematics.transforms(default)
        spheres = source_foot_spheres(source); height = initial_height(spheres, posed_fk)
        assert .65 < height < 1.2
        os.environ['FEROX_REUSE_KIT_APP'] = '1'
        from isaacsim import SimulationApp
        phase = 'asset_import'; app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting'})
        from PIL import Image
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, PhysxSchema, PhysicsSchemaTools
        from omni.physx import get_physx_simulation_interface, get_physxunittests_interface
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.sensors.camera import Camera
        enable_extension('omni.pip.compute')
        from isaac.run import G1VelocityPolicy
        from inspire_body_asset import import_body
        from inspire_collision import replace_palm_with_components, replace_left_thumb_with_slabs
        from rigid_inertia import audit_live_properties
        def palm_builder(stage, mesh, body, side):
            return replace_palm_with_components(stage, mesh, body, contact_offset_m=.0012860533315688372,
                rest_offset_m=0., candidate_id='ftp_palm_yz_slabs_v2' if side == 'right' else 'ftp_left_palm_yz_slabs_v1')
        asset, facts = import_body(source, out, fixed_base=False, palm_builder=palm_builder,
                                   left_thumb_builder=replace_left_thumb_with_slabs, hands_expected=arm['hands_expected'])
        # Rig gains are checked against the imported pelvis link's own mass/inertia BEFORE physics.
        pel = facts['expected_source_rigid_properties_in_imported_frame']['pelvis']
        inertia = pel['inertia_at_com_runtime_link_kg_m2']
        margins = rig_stability_margins(cfg, float(pel['mass_kg']), min(inertia[i][i] for i in range(3)))
        write('rig_stability_margins.json', margins)
        if not margins['stable']:
            raise ValueError('rig gains unstable for an explicit per-step wrench on the pelvis link: %s' % json.dumps(margins))
        assert set(default) == set(facts['body_joint_names'])
        assert len(facts['joint_limits']) == arm['joint_count']
        hand_names = list(facts['hand_independent_names'])
        # Seeded initial perturbation on the 29 body joints only; hands at the declared margin.
        body_initial, pelvis_jitter = perturbed_initial(default, facts['joint_limits'], cfg)
        initial_q = dict(body_initial)
        for n in hand_names:
            initial_q[n] = cfg.hand_margin_rad
        for n, m in facts['mimic_map'].items():
            initial_q[n] = m['multiplier'] * initial_q[m['parent']] + m['offset']
        assert set(initial_q) == set(facts['joint_limits'])
        for n, lim in facts['joint_limits'].items():
            assert lim['lower'] <= initial_q[n] <= lim['upper'], (n, initial_q[n], lim)
        hand_command = [cfg.hand_margin_rad] * len(hand_names)
        write('initialization_plan.json', {'body_default_rad': default, 'body_initial_rad': body_initial, 'pelvis_jitter': pelvis_jitter,
              'hand_independent_initial_rad': cfg.hand_margin_rad, 'hand_command_rad': cfg.hand_margin_rad,
              'pelvis_height_m': height, 'foot_spheres_source': spheres, 'clearance_m': .002, 'seed': cfg.seed,
              'derivation': 'minimum source foot sphere surface at policy default FK + 2 mm; seeded joint offsets inside limits; no geometry rescaling'})
        stage = Usd.Stage.Open(str(asset)); cache = UsdGeom.XformCache()
        corrections = {}
        posed_initial_fk = kinematics.transforms({n: v for n, v in initial_q.items() if n in facts['body_joint_names']})
        for name in facts['physical_link_mass_kg']:
            prim = stage.GetPrimAtPath(facts['root_prim'] + '/' + name); assert prim
            imported = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float).T
            corrections[name] = np.linalg.inv(zero_fk[name]) @ imported
        for name, correction in corrections.items():
            prim = stage.GetPrimAtPath(facts['root_prim'] + '/' + name)
            xf = UsdGeom.Xformable(prim); xf.ClearXformOpOrder()
            xf.AddTransformOp().Set(Gf.Matrix4d(*(posed_initial_fk[name] @ correction).T.flatten().tolist()))
        for prim in stage.Traverse():
            name = prim.GetName()
            if prim.IsA(UsdPhysics.RevoluteJoint) and name in initial_q:
                state = PhysxSchema.JointStateAPI.Apply(prim, 'angular')
                state.CreatePositionAttr(math.degrees(initial_q[name])); state.CreateVelocityAttr(0.)
                if name in default:
                    i = contract.policy_names.index(name); drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
                    drive.CreateTypeAttr('force'); drive.CreateStiffnessAttr(contract.stiffness[i]); drive.CreateDampingAttr(contract.damping[i])
                    drive.CreateMaxForceAttr(facts['joint_limits'][name]['effort'])
                    # spawn drive target: the default pose (K) or the joint's own authored initial position (q == target at reset)
                    drive.CreateTargetPositionAttr(math.degrees(initial_q[name] if cfg.reset_targets == 'initial_pose' else default[name]))
                elif name in hand_names:
                    drive = UsdPhysics.DriveAPI.Apply(prim, 'angular'); drive.CreateTargetPositionAttr(math.degrees(cfg.hand_margin_rad))
        training_caps = None
        actuator_record = {'profile': cfg.actuator_profile}
        if cfg.actuator_profile == 'checkpoint_training_env':
            from g1_policy.provenance import TrainingProvenance
            table = TrainingProvenance.load(cfg.policy_path).actuator_table(list(contract.sdk_names))
            training_caps = {n: v['effort_limit_sim'] for n, v in table.items()}
            actuator_record.update(table=table, armature_applied=[], depenetration_applied=[], solver_iterations=[8, 4])
            for prim in stage.Traverse():
                name = prim.GetName()
                if prim.IsA(UsdPhysics.RevoluteJoint) and name in table:
                    PhysxSchema.PhysxJointAPI.Apply(prim).CreateArmatureAttr(float(table[name]['armature']))
                    actuator_record['armature_applied'].append(name)
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateMaxDepenetrationVelocityAttr(1.0)
                    actuator_record['depenetration_applied'].append(str(prim.GetPath()))
        write('actuator_profile_applied.json', actuator_record)
        diagnostic_record = {'disable_hand_collisions': [], 'lock_hand_joints': []}
        if cfg.diagnostics['disable_hand_collisions']:
            hand_links = {n for n in facts['physical_link_mass_kg'] if any(k in n for k in ('_base_link', 'thumb', 'index', 'middle', 'ring', 'little', 'palm'))}
            for prim in stage.Traverse():
                if prim.HasAPI(UsdPhysics.CollisionAPI) and any('/' + n + '/' in str(prim.GetPath()) + '/' for n in hand_links):
                    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
                    diagnostic_record['disable_hand_collisions'].append(str(prim.GetPath()))
        if cfg.diagnostics['lock_hand_joints']:
            for prim in stage.Traverse():
                name = prim.GetName()
                if prim.IsA(UsdPhysics.RevoluteJoint) and name in hand_names:
                    drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
                    drive.CreateStiffnessAttr(200.); drive.CreateDampingAttr(2.)
                    diagnostic_record['lock_hand_joints'].append(name)
        stage.GetRootLayer().Save()
        write('initialization_frame_corrections.json', {n: t.tolist() for n, t in corrections.items()})
        write('diagnostics_applied.json', {'config': cfg.diagnostics, 'applied': diagnostic_record,
                                           'scope': 'NONQUALIFYING_DIAGNOSTIC' if cfg.nonqualifying else 'none'})
        PHYSICS_DT = .005
        world = World(stage_units_in_meters=1., physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)
        scene = next(p for p in world.stage.Traverse() if p.IsA(UsdPhysics.Scene))
        physics = PhysxSchema.PhysxSceneAPI.Apply(scene)
        physics.CreateSolverTypeAttr('TGS'); physics.CreateEnableExternalForcesEveryIterationAttr(True)
        physics.CreateEnableGPUDynamicsAttr(False)
        add_reference_to_stage(str(asset), '/World/G1')
        root = UsdGeom.Xformable(world.stage.GetPrimAtPath('/World/G1'))
        root.ClearXformOpOrder()
        spawn_z = cfg.training_reset_z if cfg.training_reset else height + .002
        root.AddTranslateOp().Set(Gf.Vec3d(pelvis_jitter['dx_m'], pelvis_jitter['dy_m'], spawn_z))
        root.AddRotateZOp().Set(math.degrees(pelvis_jitter['yaw_rad']))
        floor = UsdGeom.Cube.Define(world.stage, GROUND); floor.CreateSizeAttr(1.)
        floor.AddTranslateOp().Set(Gf.Vec3d(0., 0., -.05)); floor.AddScaleOp().Set(Gf.Vec3f(6., 6., .1))
        floor.CreateDisplayColorAttr([Gf.Vec3f(.22, .24, .27)]); UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        material = UsdShade.Material.Define(world.stage, '/World/DeclaredGroundMaterial')
        mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        mat.CreateStaticFrictionAttr(cfg.ground_static_friction); mat.CreateDynamicFrictionAttr(cfg.ground_dynamic_friction); mat.CreateRestitutionAttr(0.)
        supporting_constraints = []
        for prim in world.stage.Traverse():
            if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
                api = PhysxSchema.PhysxArticulationAPI(prim)
                pos_it, vel_it = (8, 4) if cfg.actuator_profile == 'checkpoint_training_env' else (32, 8)
                api.CreateSolverPositionIterationCountAttr(pos_it); api.CreateSolverVelocityIterationCountAttr(vel_it); api.CreateSleepThresholdAttr(0.)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, 'physics')
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                assert not UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get()
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
            if prim.IsA(UsdPhysics.Joint):
                joint = UsdPhysics.Joint(prim)
                if not joint.GetBody0Rel().GetTargets() or not joint.GetBody1Rel().GetTargets():
                    supporting_constraints.append(str(prim.GetPath()))
        assert not supporting_constraints, supporting_constraints
        UsdLux.DomeLight.Define(world.stage, '/World/Fill').CreateIntensityAttr(450.)
        key = UsdLux.DistantLight.Define(world.stage, '/World/Key'); key.CreateIntensityAttr(550.)
        UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-35., -25., -40.))
        world.stage.GetRootLayer().Export(str(out / 'scene_authored_before_reset.usda'))
        contact_file = (out / 'contacts.jsonl').open('w', buffering=1)
        state_file = (out / 'state.jsonl').open('w', buffering=1)
        frame_file = (out / 'frames.jsonl').open('w', buffering=1)
        event_file = (out / 'support_events.jsonl').open('w', buffering=1)
        journal_file = (out / 'ownership_journal.jsonl').open('w', buffering=1); streams += [contact_file, state_file, frame_file, event_file, journal_file]
        def on_contacts(headers, data):
            try:
                for header in headers:
                    paths = {k: str(PhysicsSchemaTools.intToSdfPath(getattr(header, k))) for k in ('actor0', 'actor1', 'collider0', 'collider1')}
                    for index in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                        point = data[index]
                        row = dict(paths, sequence=sequence, physics_s=float(world.current_time), phase=phase,
                            position_world_m=list(map(float, point.position)), normal_world=list(map(float, point.normal)),
                            impulse_ns=list(map(float, point.impulse)), separation_m=float(point.separation), source='PhysX_simulated_contact_proxy')
                        contacts.append(row); step_contacts.append(row)
                        contact_file.write(json.dumps(safe_json(row), allow_nan=False) + '\n')
                        if not finite_tree(row): contact_faults.append('nonfinite_contact')
            except Exception as exc:
                contact_faults.append(repr(exc))
        phase = 'initialization'
        subscription = get_physx_simulation_interface().subscribe_contact_report_events(on_contacts)
        robot = SingleArticulation('/World/G1', name='standing_ab_' + cfg.arm)
        world.reset(); robot.initialize()
        names = list(robot.dof_names); assert len(names) == arm['joint_count'] and set(names) == set(facts['joint_limits'])
        policy = named_policy_class(G1VelocityPolicy)(robot=robot, policy_dir=cfg.policy_path, source_urdf=source, physics_dt=.005)
        policy_receipt = policy.initialize(initialize_articulation=False, training_effort_caps=training_caps)
        policy_receipt['additional_source_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (Path(adapter_module.__file__), Path(cfg.policy_path) / 'exported/policy.pt', Path(__file__), source)}
        write('controller_receipt.json', policy_receipt)
        sim_view = SimulationManager.get_physics_sim_view()
        inertia = audit_live_properties(sim_view, '/World/G1', facts['expected_source_rigid_properties_in_imported_frame'])
        write('live_inertia_audit.json', inertia)
        view_names = ('pelvis', 'torso_link') + FEET + ('right_wrist_yaw_link', 'left_wrist_yaw_link')
        if arm['hands_expected']:
            view_names += ('right_base_link', 'left_base_link')
        views = {n: sim_view.create_rigid_body_view('/World/G1/' + n) for n in view_names}
        assert all(v.count == 1 for v in views.values())
        shapes = {'statistics': get_physxunittests_interface().get_physics_stats(), 'palms': {}}
        if arm['hands_expected']:
            for side in ('left', 'right'):
                shapes['palms'][side] = {'live_shape_count': views[side + '_base_link'].max_shapes,
                    'expected_shape_count': facts['collision_candidates'][side]['expected_palm_hulls_if_all_cooking_succeeds']}
            thumb = sim_view.create_rigid_body_view('/World/G1/left_thumb_2')
            shapes['left_thumb'] = {'live_shape_count': thumb.max_shapes,
                                   'expected_shape_count': facts['thumb_collision_candidates']['left']['expected_live_hulls']}
            assert thumb.count == 1
        write('backend_shapes.json', shapes)
        pelvis_view = views['pelvis']
        def read_state():
            vel = np.asarray(pelvis_view.get_velocities())[0].tolist()
            return {'runtime_names': names, 'q_rad': np.asarray(robot.get_joint_positions()).ravel().tolist(),
                    'dq_rad_s': np.asarray(robot.get_joint_velocities()).ravel().tolist(),
                    'measured_generalized_effort_nm': np.asarray(robot.get_measured_joint_efforts()).ravel().tolist(),
                    'link_poses_world_xyzw': {n: np.asarray(v.get_transforms())[0].tolist() for n, v in views.items()},
                    'pelvis_linear_velocity_m_s': vel[:3], 'pelvis_angular_velocity_rad_s': vel[3:],
                    'physics_s': float(world.current_time)}
        initial = read_state(); last = initial
        initial_error = max(abs(initial['q_rad'][names.index(n)] - q) for n, q in initial_q.items())
        initial['source_initial_q_error_max_rad'] = initial_error
        initial['authored_pelvis_height_m'] = spawn_z
        initial['training_reset'] = cfg.training_reset
        initial['reset_targets'] = cfg.reset_targets
        initial['initialization_contact_count'] = len(contacts)
        write('initial_state.json', initial)
        integrity = {'source_mass_com_inertia_preserved': all(inertia['checks'].values()),
            'both_palm_shape_counts_match': (not arm['hands_expected']) or all(v['live_shape_count'] == v['expected_shape_count'] for v in shapes['palms'].values()),
            'left_thumb_shape_count_matches': (not arm['hands_expected']) or shapes['left_thumb']['live_shape_count'] == shapes['left_thumb']['expected_shape_count'],
            'no_static_triangle_shapes': shapes['statistics']['numTriMeshShapes'] == 0,
            'initial_pose_matches_source_fk': finite_tree(initial) and initial_error < .01,
            'policy_named_mapping_and_live_gains_caps_verified': policy_receipt['observation_width'] == 480 and policy_receipt['action_width'] == 29,
            'runtime_joint_count_matches_arm': len(names) == arm['joint_count'],
            'contact_instrumentation_valid': not contact_faults}
        write('initialization_gates.json', integrity)
        if not all(integrity.values()): raise ValueError('Initialization failed immutable admission gates')
        body_names = list(contract.policy_names)
        indices = np.asarray([names.index(n) for n in body_names + hand_names], dtype=np.int32)
        # Enforced ownership boundary: one body owner, one hand owner, declared support,
        # named 29-joint commands, fresh finite state, journaled role/support changes.
        guard = RuntimeOwnershipGuard(body_names=body_names, hand_names=hand_names,
                                      limits={n: facts['joint_limits'][n] for n in body_names + hand_names},
                                      physics_dt=.005, decimation=policy._decimation, hand_margin_rad=cfg.hand_margin_rad,
                                      max_target_step_rad=1.0, journal=journal_file)
        guard.claim_body('probe_default_pose_warmup'); guard.claim_hands('probe_margin_hold')
        if not cfg.training_reset:
            guard.declare_support('RIG_WRENCH')
        default_targets = {n: float(default[n]) for n in body_names}
        default_arm = [float(default[n]) for n in ARM_JOINTS]
        kp = np.asarray([policy_receipt['live_stiffness_by_name'][n] for n in names])
        kd = np.asarray([policy_receipt['live_damping_by_name'][n] for n in names])
        caps = np.asarray([policy_receipt['live_drive_effort_caps_by_name'][n] for n in names])
        cameras = {}
        for label, position, target in [('front', (2.6, -2.8, 1.55), (0., 0., .8)), ('side', (-.4, 3., 1.45), (0., 0., .8))]:
            camera = Camera('/World/' + label + 'Camera', resolution=(640, 640)); camera.initialize(); camera.set_clipping_range(.01, 12.)
            xf = UsdGeom.Xformable(camera.prim); xf.ClearXformOpOrder()
            xf.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*position), Gf.Vec3d(*target), Gf.Vec3d(0., 0., 1.)).GetInverse())
            cameras[label] = camera; (out / 'frames' / label).mkdir(parents=True)
        for _ in range(8): world.render()
        assert float(world.current_time) == initial['physics_s']
        rig_target = list(initial['link_poses_world_xyzw']['pelvis'])
        release_sequence = -1 if cfg.training_reset else cfg.supported_settle_steps - 1
        total_steps = (cfg.landing_steps if cfg.training_reset else cfg.supported_settle_steps) + cfg.unsupported_steps
        phase = ('landing' if cfg.landing_steps > 0 else 'unsupported') if cfg.training_reset else 'supported_settle'
        if cfg.training_reset:
            # Never supported: declare it so, prime the history as at an Isaac Lab reset, own the body from step 0.
            guard.declare_never_supported(EXPERIMENTAL_OWNER if cfg.arm_override['enabled'] else 'named_policy_single_writer')
            events.append({'sequence': -1, 'physics_s': float(world.current_time), 'name': 'support_release',
                           'detail': 'training_reset: no rig at any time; policy from step 0 with a first-value-filled history'})
            event_file.write(json.dumps(events[-1]) + '\n')
            if cfg.handover['history_priming']:
                policy.prime_history_from_current([0., 0., 0.])
                events.append({'sequence': -1, 'physics_s': float(world.current_time), 'name': 'policy_history_primed', 'detail': 'at spawn'})
                event_file.write(json.dumps(events[-1]) + '\n')
        observation_sequence = None; observation_physics_s = None; prev_q_abort = None; interval_persist = {}
        for sequence in range(total_steps):
            step_contacts.clear()
            supported = sequence <= release_sequence
            try:
                guard.begin_step(sequence, float(world.current_time))
                if sequence == release_sequence + 1 and not cfg.training_reset:   # training_reset: never supported, declared before step 0
                    phase = 'unsupported'
                    guard.release_support()
                    events.append({'sequence': release_sequence, 'physics_s': float(world.current_time), 'name': 'support_release',
                                   'detail': 'rig wrench zeroed from this step on; no constraint or force acts on the body afterwards'})
                    event_file.write(json.dumps(events[-1]) + '\n')
            except GuardRefused as exc:
                abort = {'sequence': sequence, 'reason': 'guard_refused: ' + str(exc), 'phase': phase}; break
            if cfg.training_reset and sequence == cfg.landing_steps and phase == 'landing':
                phase = 'unsupported'
                events.append({'sequence': sequence, 'physics_s': float(world.current_time), 'name': 'landing_window_end',
                               'detail': 'standing gates scored from this row (declared landing_settle_s)'})
                event_file.write(json.dumps(events[-1]) + '\n')
            warmup = (not cfg.training_reset) and sequence < cfg.policy_warmup_steps
            command_velocity = [0., 0., 0.]
            arm_reference = None
            if warmup:
                # Pre-policy hold: the drives hold the policy default pose under the rig while the
                # articulation settles; the policy's history buffer is not fed synthetic data.
                inference = False
                owner = 'probe_default_pose_warmup'
                targets = default_targets
            else:
                # The body owner for the whole run after warm-up is fixed here, while still supported:
                # the named policy alone, or the EXPERIMENTAL policy+arm-override combination.
                run_owner = EXPERIMENTAL_OWNER if cfg.arm_override['enabled'] else 'named_policy_single_writer'
                if sequence == cfg.policy_warmup_steps and not cfg.training_reset:
                    try:
                        guard.claim_body(run_owner)
                    except GuardRefused as exc:
                        abort = {'sequence': sequence, 'reason': 'guard_refused: ' + str(exc), 'phase': phase}; break
                    if cfg.handover['history_priming']:
                        primed = policy.prime_history_from_current([0., 0., 0.])
                        events.append({'sequence': sequence, 'physics_s': float(world.current_time), 'name': 'policy_history_primed',
                                       'detail': 'all history slots filled from the current measured state; last_action zero'})
                        event_file.write(json.dumps(events[-1]) + '\n')
                owner = run_owner
                inference = policy._policy_counter % policy._decimation == 0
                if inference:
                    observation_sequence = sequence - 1
                    observation_physics_s = float(world.current_time)
                command_velocity = command_at(cfg, (sequence - release_sequence) * .005) if not supported else [0., 0., 0.]
                targets = policy.forward(.005, command_velocity)
                arm_reference = arm_reference_at(cfg, default_arm, (sequence - release_sequence) * .005) if not supported else None
                if arm_reference is not None:
                    # EXPERIMENTAL combined controller: legs+waist from the policy, arms from the q-only
                    # reference. The policy's observation is NOT edited; its arm actions are discarded.
                    targets = dict(targets)
                    for n, v in zip(ARM_JOINTS, arm_reference):
                        targets[n] = float(v)
            assert set(targets) == set(body_names)
            command = np.asarray([targets[n] for n in body_names] + hand_command, dtype=np.float32)
            if supported:
                pose = np.asarray(pelvis_view.get_transforms())[0].tolist()
                vel = np.asarray(pelvis_view.get_velocities())[0].tolist()
                scale = rig_scale_at(cfg, sequence)
                if sequence == cfg.handover['ramp_end_step']:
                    rig_target = list(pose)   # re-anchor: the residual PD holds the settled stance, not the hanging pose
                    events.append({'sequence': sequence, 'physics_s': float(world.current_time), 'name': 'rig_retargeted_to_settled_pose',
                                   'detail': {'pelvis_xyzw': rig_target, 'scale': scale}})
                    event_file.write(json.dumps(events[-1]) + '\n')
                force, torque = rig_wrench(cfg, pose, vel[:3], vel[3:], rig_target, body_mass_kg=facts['source_physical_mass_kg'])
                force = [scale * v for v in force]; torque = [scale * v for v in torque]
                # Applied at the pelvis link transform in the world frame (is_global=True); one body in the view.
                pelvis_view.apply_forces_and_torques_at_position(np.asarray([force], dtype=np.float32), np.asarray([torque], dtype=np.float32),
                                                                 None, np.asarray([0], dtype=np.uint32), True)
                support = {'kind': 'RIG_WRENCH', 'force_n': force, 'torque_nm': torque, 'target_xyzw': rig_target, 'scale': scale}
            else:
                support = {'kind': 'NONE', 'force_n': [0., 0., 0.], 'torque_nm': [0., 0., 0.]}
            try:
                guard.assert_support_row(support)
                guard.admit_body_command(owner, body_names, [float(targets[n]) for n in body_names])
                guard.admit_hand_command('probe_margin_hold', hand_names, hand_command)
            except GuardRefused as exc:
                abort = {'sequence': sequence, 'reason': 'guard_refused: ' + str(exc), 'phase': phase}; break
            # The only runtime actuator writer. No state or base pose setters.
            robot.apply_action(ArticulationAction(joint_positions=command, joint_indices=indices))
            world.step(render=False)
            row = read_state(); last = row
            try:
                guard.observe_state(*guarded_state_view(names, row['q_rad'], row['dq_rad_s'], body_names + hand_names))
            except GuardRefused as exc:
                abort = {'sequence': sequence, 'reason': 'guard_refused: ' + str(exc), 'phase': phase}
                row.update(sequence=sequence, phase=phase, support=support, command_velocity=command_velocity,
                    body_command_owner=owner, body_command_names=body_names,
                    body_command_rad=[float(targets[n]) for n in body_names], hand_command_names=hand_names, hand_command_rad=hand_command)
                rows.append(row); break
            row.update(sequence=sequence, phase=phase, support=support, command_velocity=command_velocity,
                arm_reference_rad=(arm_reference if (not warmup and not supported) else None),
                body_command_owner=owner, body_command_names=body_names,
                body_command_rad=[float(targets[n]) for n in body_names], hand_command_names=hand_names, hand_command_rad=hand_command,
                policy_inference_this_step=inference,
                policy_observation=([] if warmup else policy.last_observation.tolist()),
                policy_observation_source_sequence=observation_sequence, policy_observation_physics_s=observation_physics_s,
                policy_action=([] if warmup else policy.action.tolist()), contacts=list(step_contacts))
            if not finite_tree(row):
                write('nonfinite_state.json', row); abort = {'sequence': sequence, 'reason': 'nonfinite_state'}; break
            q = np.asarray(row['q_rad']); dq = np.asarray(row['dq_rad_s'])
            estimate = kp[indices] * (command - q[indices]) - kd[indices] * dq[indices]
            row['drive_estimate_nm'] = estimate.tolist()
            row['drive_estimate_names'] = body_names + hand_names
            row['drive_estimate_near_cap_names'] = [n for n, e, cap in zip(body_names + hand_names, estimate, caps[indices]) if abs(e) >= .99 * cap]
            row['drive_estimate_scope'] = 'Unclipped PD estimate; measured generalized efforts also include constraint reactions'
            hand_set = set(hand_names)
            legacy_overspeed = {n: float(dq[i]) for i, n in enumerate(names) if abs(dq[i]) > 2 * facts['joint_limits'][n]['velocity']}
            if cfg.finger_velocity_channel == 'interval':
                # PROSPECTIVE profile (wb-cand-07 r2, hand-an-05 review): hand joints on the finite difference of position over the
                # physics step (PHYSICS_DT; the standing probe has no pose write after the spawn, so every interval is a physics step);
                # ONLINE abort only on >= 2 CONSECUTIVE SAME-SIGN intervals beyond 2x the field (open-stop chatter alternates sign and
                # never persists); a single-interval exceedance is logged as interval_spike and counted, never acted on. Body joints
                # stay on the readback. The readback verdict is logged on every row. PARENT_CAP_SOLVE witness logged, never acted on.
                interval = ({} if prev_q_abort is None else {n: float((q[i] - prev_q_abort[i]) / PHYSICS_DT) for i, n in enumerate(names) if n in hand_set})
                overspeed = {n: v for n, v in legacy_overspeed.items() if n not in hand_set}
                spikes = {}
                for n, v in interval.items():
                    lim2 = 2 * facts['joint_limits'][n]['velocity']
                    if abs(v) > lim2:
                        spikes[n] = v
                        if interval_persists(interval_persist.get(n), v, facts['joint_limits'][n]['velocity']):
                            overspeed[n] = v   # persisted: two consecutive same-sign intervals beyond 2x
                    interval_persist[n] = v
                witness = {}
                for child, m in facts['mimic_map'].items():
                    if child not in hand_set or child not in interval:
                        continue
                    ci, pi = names.index(child), names.index(m['parent']); fld = facts['joint_limits'][child]['velocity']; pfld = facts['joint_limits'][m['parent']]['velocity']
                    if abs(dq[ci]) <= fld:
                        continue
                    coupling = float(q[ci] - m['multiplier'] * q[pi] - m['offset'])
                    witness[child] = {'child_readback': float(dq[ci]), 'child_interval': interval[child], 'parent': m['parent'], 'parent_readback': float(dq[pi]),
                                      'parent_interval': interval.get(m['parent']), 'coupling_error_rad': coupling,
                                      'parent_cap_solve': bool(abs(dq[ci]) > 2 * fld and abs(dq[pi]) >= .99 * pfld and abs(interval.get(m['parent']) or 0.) <= pfld and abs(coupling) <= .03)}
                row['hand_dq_interval_rad_s'] = interval
                row['legacy_finger_overspeed_readback'] = {n: v for n, v in legacy_overspeed.items() if n in hand_set}
                row['interval_spike'] = spikes
                row['finger_witness'] = witness
            else:
                overspeed = legacy_overspeed   # default: byte-identical K/L rows and decisions
            prev_q_abort = list(map(float, q))
            rows.append(row); state_file.write(json.dumps(row, allow_nan=False) + '\n')
            pelvis = row['link_poses_world_xyzw']['pelvis']; roll, pitch = roll_pitch(pelvis)
            violated = {n: float(q[i]) for i, n in enumerate(names) if q[i] < facts['joint_limits'][n]['lower'] - .1 or q[i] > facts['joint_limits'][n]['upper'] + .1}
            if violated or overspeed or contact_faults or (not supported and (pelvis[2] < .65 or max(abs(roll), abs(pitch)) > .35)):
                abort = {'sequence': sequence, 'reason': 'fall_or_source_envelope_abort', 'phase': phase, 'joint_limit_violations': violated,
                         'overspeed': overspeed, 'contact_faults': contact_faults,
                         **({'finger_velocity_channel': 'interval', 'legacy_finger_overspeed_readback': row['legacy_finger_overspeed_readback']} if cfg.finger_velocity_channel == 'interval' else {})}
                guard.fault('fall_or_source_envelope_abort')
                break
            if (sequence + 1) % cfg.frame_every == 0:
                before = float(world.current_time); world.render(); assert float(world.current_time) == before
                frame = (sequence + 1) // cfg.frame_every - 1; files = {}
                for label, camera in cameras.items():
                    pixels = camera.get_rgba(); assert pixels is not None and pixels.shape == (640, 640, 4)
                    name = f'frames/{label}/{frame:06d}.png'; Image.fromarray(pixels.astype(np.uint8)).save(out / name); files[label] = name
                frame_file.write(json.dumps({'frame': frame, 'sequence': sequence, 'physics_s': before, 'phase': phase,
                    'captured_after_same_step_render': True, 'views': files}) + '\n')
        integrity['contact_instrumentation_valid'] = not contact_faults
        result = evaluate_standing(rows, events, cfg, joint_count=arm['joint_count'], limits=facts['joint_limits'],
                                   mimics=facts['mimic_map'], source_mass_kg=facts['source_physical_mass_kg'],
                                   integrity_checks=integrity, abort=abort, adjacent_pairs=source_adjacency(out / 'assembled_physical.urdf'),
                                   guard_entries=guard.entries)
        result['ownership_guard'] = guard.summary()
        result.update(initial_pelvis_height_m=initial['link_poses_world_xyzw']['pelvis'][2], actual_contact_points=len(contacts),
            controller_receipt='controller_receipt.json', source_property_audit='live_inertia_audit.json', support_events=events)
        receipt(result)
        world.stage.GetRootLayer().Export(str(out / 'scene_final.usda'))
    except BaseException as exc:
        write('failure.json', {'exception': repr(exc), 'traceback': traceback.format_exc(), 'phase': phase, 'steps': len(rows),
                               'last_observation': last, 'initialization_gates': integrity, 'contact_faults': contact_faults})
        receipt({'status': 'FAIL', 'steps': len(rows), 'checks': {'probe_completed_without_exception': False},
                 'exception': repr(exc), 'abort': abort})
    finally:
        cleanup_errors = []
        if subscription is not None:
            try: subscription.unsubscribe()
            except Exception as exc: cleanup_errors.append(repr(exc))
        for stream in streams:
            try: stream.close()
            except Exception as exc: cleanup_errors.append(repr(exc))
        if app is not None:
            try: app.close()
            except Exception as exc: cleanup_errors.append(repr(exc))
        if cleanup_errors:
            write('cleanup_failure.json', cleanup_errors)
            receipt({'status': 'FAIL', 'steps': len(rows), 'checks': {'orderly_cleanup': False}, 'cleanup_errors': cleanup_errors})


if __name__ == '__main__':
    main()
