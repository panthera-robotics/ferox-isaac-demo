"""Ten-second, zero-command, freely standing provisional G1/FTP diagnostic.

The private adapter provides named targets using the public policy's exact
observation/inference methods. This caller alone writes actuator targets.
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
    assert os.environ.get('PANTHERA_PROBE_MODE') == 'assembled-zero-command'
    sys.path[:0] = ['/workspace/ferox_tools', '/workspace/sim-source', '/workspace/ferox_isaac']
    from assembled_balance import (BalanceConfig, GATES, FEET, GROUND, evaluate, SourceEquilibriumPD,
                                   source_foot_spheres, source_adjacency, material_self_penetrations,
                                   initial_height, roll_pitch, finite_tree, safe_json)
    cfg = BalanceConfig.from_dict(json.loads(Path(os.environ['PANTHERA_PROBE_CONFIG']).read_text())
                                  if os.environ.get('PANTHERA_PROBE_CONFIG') else {})
    sys.path.insert(0, cfg.locomotion_path)
    out = Path('/evidence'); app = None; world = None; subscription = None; streams = []
    rows = []; contacts = []; step_contacts = []; contact_faults = []; sequence = None
    phase = 'configuration'; last = {}; initial = {}; facts = {}; abort = None; integrity = {}
    media = {'fixture': 'FREE STANDING - physical ground contacts; no base support',
             'embodiment': 'PROVISIONAL G1 + bilateral FTP hands',
             'qualification': 'SIM-only zero-command balance surrogate; no firmware or manipulation qualification'}
    scope = {'fixed_base': False, 'support_constraints': [], 'external_forces': False,
             'pose_writes_after_reset': 0, 'joint_state_writes_after_reset': 0,
             'exact_E2_qualified': False, 'firmware_qualified': False, 'hardware_authorized': False,
             'grasp_qualification': 'NOT_RUN', 'writing_qualification': 'NOT_RUN',
             'source': 'provisional FTP donor; exact E2 asset identity unverified',
             'physics': 'PhysX CPU dynamics with GPU rendering'}
    def write(name, value):
        (out / name).write_text(json.dumps(safe_json(value), indent=2, allow_nan=False) + '\n')
    def receipt(metrics):
        metrics.update(scope=scope, media_labels=media, gates=GATES, phase=phase)
        metrics['controller_mode'] = cfg.controller_mode
        for stream in streams:
            if not stream.closed: stream.flush()
        write('metrics.json', metrics)
        artifacts = [str(p.relative_to(out)) for p in out.rglob('*') if p.is_file() and p.name not in
                     ['run.json', 'probe.json', 'console.log', 'executed_probe.py', 'executed_launcher.py', 'uncommitted.patch']]
        write('probe.json', {'status': metrics['status'], 'scope': 'provisional_free_standing_zero_command_diagnostic',
                             'metrics': 'metrics.json', 'artifacts': artifacts,
                             'artifact_sha256': {name: hashlib.sha256((out / name).read_bytes()).hexdigest() for name in artifacts}})
    write('balance_config.json', {'config': asdict(cfg), 'gates': GATES, 'scope': scope,
                                  'gates_authored_before_physics': True})
    try:
        from g1_policy.contract import PolicyContract
        from g1_policy.isaac_adapter import named_policy_class
        import g1_policy.isaac_adapter as adapter_module
        import numpy as np
        from urdf_kinematics import UrdfKinematics
        source = Path('/source-assets/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf')
        contract = PolicyContract.load(cfg.policy_path)
        assert contract.observation_width == 480 and len(contract.policy_names) == 29
        default = dict(zip(contract.policy_names, contract.default_position))
        equilibrium_mode = cfg.controller_mode == 'source_contact_equilibrium_pd'
        adjacent_pairs = source_adjacency(source)
        write('source_self_contact_gate.json', {'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'direct_source_adjacency_pairs': [list(p) for p in sorted(adjacent_pairs)],
            'maximum_penetration_m': GATES['maximum_loaded_nonadjacent_self_penetration_m'],
            'minimum_penetrating_pair_impulse_ns': GATES['material_self_penetration_pair_impulse_ns'],
            'interpretation': '1 N equivalent pair load over 5 ms at points deeper than 1 mm; no filtering or geometry changes',
            'authored_before_physics': True})
        kinematics = UrdfKinematics(source); zero_fk = kinematics.transforms({}); posed_fk = kinematics.transforms(default)
        spheres = source_foot_spheres(source); height = initial_height(spheres, posed_fk)
        assert .65 < height < 1.2
        write('initialization_plan.json', {'body_default_rad': default, 'hand_independent_default_rad': 0.,
              'pelvis_height_m': height, 'foot_spheres_source': spheres, 'clearance_m': .002,
              'derivation': 'minimum original source foot sphere surface at policy default FK; no geometry rescaling'})
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
                                   left_thumb_builder=replace_left_thumb_with_slabs)
        assert set(default) == set(facts['body_joint_names'])
        initial_q = {n: float(default.get(n, 0.)) for n in facts['joint_limits']}
        for n, lim in facts['joint_limits'].items():
            assert lim['lower'] <= initial_q[n] <= lim['upper'], (n, initial_q[n], lim)
        stage = Usd.Stage.Open(str(asset)); cache = UsdGeom.XformCache()
        corrections = {}
        # Preserve each imported link frame and its local inertia/collider/joint
        # coordinates while authoring one consistent nonzero episode pose.
        for name in facts['physical_link_mass_kg']:
            prim = stage.GetPrimAtPath(facts['root_prim'] + '/' + name); assert prim
            imported = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float).T
            corrections[name] = np.linalg.inv(zero_fk[name]) @ imported
        for name, correction in corrections.items():
            prim = stage.GetPrimAtPath(facts['root_prim'] + '/' + name)
            xf = UsdGeom.Xformable(prim); xf.ClearXformOpOrder()
            xf.AddTransformOp().Set(Gf.Matrix4d(*(posed_fk[name] @ correction).T.flatten().tolist()))
        for prim in stage.Traverse():
            name = prim.GetName()
            if prim.IsA(UsdPhysics.RevoluteJoint) and name in initial_q:
                state = PhysxSchema.JointStateAPI.Apply(prim, 'angular')
                state.CreatePositionAttr(math.degrees(initial_q[name])); state.CreateVelocityAttr(0.)
                if name in default:
                    i = contract.policy_names.index(name); drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
                    drive.CreateTypeAttr('force')
                    drive.CreateStiffnessAttr(0. if equilibrium_mode else contract.stiffness[i])
                    drive.CreateDampingAttr(0. if equilibrium_mode else contract.damping[i])
                    drive.CreateMaxForceAttr(facts['joint_limits'][name]['effort']); drive.CreateTargetPositionAttr(math.degrees(default[name]))
        stage.GetRootLayer().Save()
        write('initialization_frame_corrections.json', {n: t.tolist() for n, t in corrections.items()})
        world = World(stage_units_in_meters=1., physics_dt=.005, rendering_dt=.005)
        scene = next(p for p in world.stage.Traverse() if p.IsA(UsdPhysics.Scene))
        physics = PhysxSchema.PhysxSceneAPI.Apply(scene)
        physics.CreateSolverTypeAttr('TGS'); physics.CreateEnableExternalForcesEveryIterationAttr(True)
        physics.CreateEnableGPUDynamicsAttr(False)
        add_reference_to_stage(str(asset), '/World/G1')
        root = UsdGeom.Xformable(world.stage.GetPrimAtPath('/World/G1'))
        root.ClearXformOpOrder(); root.AddTranslateOp().Set(Gf.Vec3d(0., 0., height))
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
                api.CreateSolverPositionIterationCountAttr(32); api.CreateSolverVelocityIterationCountAttr(8); api.CreateSleepThresholdAttr(0.)
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
        frame_file = (out / 'frames.jsonl').open('w', buffering=1); streams += [contact_file, state_file, frame_file]
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
        robot = SingleArticulation('/World/G1', name='provisional_free_standing_g1')
        world.reset(); robot.initialize()
        names = list(robot.dof_names); assert len(names) == 53 and set(names) == set(facts['joint_limits'])
        # In the deterministic mode the adapter configures named gains/caps only.
        # A sentinel prevents loading the actor, and forward is never invoked.
        policy = named_policy_class(G1VelocityPolicy)(robot=robot, policy_dir=cfg.policy_path, source_urdf=source, physics_dt=.005,
            body_actuation_mode='external_effort' if equilibrium_mode else 'implicit_position',
            policy=object() if equilibrium_mode else None)
        policy_receipt = policy.initialize(initialize_articulation=False)
        body_names = list(contract.policy_names); hand_names = list(facts['hand_independent_names'])
        equilibrium = None; controller_reference = None
        if equilibrium_mode:
            equilibrium = SourceEquilibriumPD(cfg.source_contact_equilibrium, body_names, contract.default_position,
                contract.stiffness, contract.damping, [facts['joint_limits'][n]['effort'] for n in body_names],
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
            assert abs(cfg.source_contact_equilibrium['source_mass_kg'] - facts['source_physical_mass_kg']) < 1e-8
            controller_reference = equilibrium.receipt()
            policy_receipt.update(controller_reference)
            assert all(policy_receipt['live_stiffness_by_name'][n] == policy_receipt['live_damping_by_name'][n] == 0.
                       for n in body_names)
        policy_receipt['learned_actor_loaded'] = not equilibrium_mode
        policy_receipt['learned_actor_inference_executed'] = False
        policy_receipt['additional_source_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (Path(adapter_module.__file__), Path(cfg.policy_path) / 'exported/policy.pt', Path(__file__))}
        write('controller_receipt.json', policy_receipt)
        sim_view = SimulationManager.get_physics_sim_view()
        inertia = audit_live_properties(sim_view, '/World/G1', facts['expected_source_rigid_properties_in_imported_frame'])
        write('live_inertia_audit.json', inertia)
        views = {n: sim_view.create_rigid_body_view('/World/G1/' + n) for n in ('pelvis', 'torso_link') + FEET +
                 ('right_wrist_yaw_link', 'left_wrist_yaw_link', 'right_base_link', 'left_base_link')}
        assert all(v.count == 1 for v in views.values())
        shapes = {'statistics': get_physxunittests_interface().get_physics_stats(), 'palms': {}}
        for side in ('left', 'right'):
            shapes['palms'][side] = {'live_shape_count': views[side + '_base_link'].max_shapes,
                'expected_shape_count': facts['collision_candidates'][side]['expected_palm_hulls_if_all_cooking_succeeds']}
        thumb = sim_view.create_rigid_body_view('/World/G1/left_thumb_2')
        shapes['left_thumb'] = {'live_shape_count': thumb.max_shapes,
                               'expected_shape_count': facts['thumb_collision_candidates']['left']['expected_live_hulls']}
        assert thumb.count == 1
        write('backend_shapes.json', shapes)
        def read_state():
            return {'runtime_names': names, 'q_rad': np.asarray(robot.get_joint_positions()).ravel().tolist(),
                    'dq_rad_s': np.asarray(robot.get_joint_velocities()).ravel().tolist(),
                    'measured_generalized_effort_nm': np.asarray(robot.get_measured_joint_efforts()).ravel().tolist(),
                    'link_poses_world_xyzw': {n: np.asarray(v.get_transforms())[0].tolist() for n, v in views.items()},
                    'physics_s': float(world.current_time)}
        initial = read_state(); last = initial
        initial_error = max(abs(initial['q_rad'][names.index(n)] - q) for n, q in initial_q.items())
        initial['source_initial_q_error_max_rad'] = initial_error
        initial['authored_pelvis_height_m'] = height
        initial['initialization_contact_count'] = len(contacts)
        def pose_matrix(pose):
            x, y, z, w = pose[3:]
            value = np.eye(4)
            value[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
            value[:3, 3] = pose[:3]
            return value
        initial['measured_source_foot_sphere_bottoms_m'] = [float((
            pose_matrix(initial['link_poses_world_xyzw'][s['link']]) @ np.linalg.inv(corrections[s['link']])
            @ np.asarray(s['center_link_m'] + [1.]))[2]) - s['radius_m'] for s in spheres]
        write('initial_state.json', initial)
        integrity = {'source_mass_com_inertia_preserved': all(inertia['checks'].values()),
            'both_palm_shape_counts_match': all(v['live_shape_count'] == v['expected_shape_count'] for v in shapes['palms'].values()),
            'left_thumb_shape_count_matches': shapes['left_thumb']['live_shape_count'] == shapes['left_thumb']['expected_shape_count'],
            'no_static_triangle_shapes': shapes['statistics']['numTriMeshShapes'] == 0,
            'initial_pose_matches_source_fk': finite_tree(initial) and initial_error < .01,
            'named_mapping_and_live_gains_caps_verified': policy_receipt['action_width'] == 29 and len(policy_receipt['runtime_joint_names']) == 53,
            'initialization_no_material_loaded_nonadjacent_self_penetration': not material_self_penetrations(contacts, adjacent_pairs),
            'contact_instrumentation_valid': not contact_faults}
        if equilibrium_mode:
            integrity['body29_implicit_drives_zero_verified'] = all(
                policy_receipt['live_stiffness_by_name'][n] == policy_receipt['live_damping_by_name'][n] == 0. for n in body_names)
            integrity['source_equilibrium_reference_verified'] = equilibrium is not None
        else:
            integrity['policy_observation_contract_width_480'] = policy_receipt['observation_width'] == 480
        write('initialization_gates.json', integrity)
        if not all(integrity.values()): raise ValueError('Initialization failed immutable admission gates')
        indices = np.asarray([names.index(n) for n in body_names + hand_names], dtype=np.int32)
        body_indices, hand_indices = indices[:29], indices[29:]
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
        phase = 'source_equilibrium_free_standing' if equilibrium_mode else 'zero_command_free_standing'
        observation_sequence = None; observation_physics_s = None
        for sequence in range(GATES['required_steps']):
            step_contacts.clear()
            if equilibrium_mode:
                feedback = read_state()
                control = equilibrium.compute(names, feedback['q_rad'], feedback['dq_rad_s'])
                targets = default
                command = np.asarray([default[n] for n in body_names] + [0.] * len(hand_names), dtype=np.float32)
                # Exactly one capped body29 effort write. The disjoint hand12
                # position write retains the existing implicit root mechanism.
                applied = np.asarray(control['body_effort_nm'], dtype=np.float32)
                robot.apply_action(ArticulationAction(joint_efforts=applied, joint_indices=body_indices))
                robot.apply_action(ArticulationAction(joint_positions=np.zeros(12, dtype=np.float32), joint_indices=hand_indices))
                live_efforts = np.asarray(robot._articulation_view._physics_view.get_dof_actuation_forces()).reshape(-1)
                expected_efforts = np.zeros(53, dtype=np.float32); expected_efforts[body_indices] = applied
                assert live_efforts.shape == (53,) and np.array_equal(live_efforts, expected_efforts)
                control['body_effort_nm'] = applied.tolist()
                controller_row = dict(control, controller_mode=cfg.controller_mode,
                    applied_generalized_actuation_effort_nm=live_efforts.tolist(),
                    body_feedback_source_sequence=sequence-1, body_feedback_physics_s=feedback['physics_s'],
                    body_effort_writes_this_step=1, body_implicit_position_writes_this_step=0,
                    body_command_owner='source_equilibrium_effort_single_writer', learned_policy_inference=False)
            else:
                inference = policy._policy_counter % policy._decimation == 0
                if inference:
                    observation_sequence = sequence - 1
                    observation_physics_s = float(world.current_time)
                targets = policy.forward(.005, [0., 0., 0.])
                assert set(targets) == set(body_names)
                command = np.asarray([targets[n] for n in body_names] + [0.] * len(hand_names), dtype=np.float32)
                robot.apply_action(ArticulationAction(joint_positions=command, joint_indices=indices))
                controller_row = dict(controller_mode=cfg.controller_mode, body_command_owner='named_policy_single_writer',
                    policy_inference_this_step=inference, policy_observation=policy.last_observation.tolist(),
                    policy_observation_source_sequence=observation_sequence, policy_observation_physics_s=observation_physics_s,
                    policy_action=policy.action.tolist())
            world.step(render=False)
            row = read_state(); last = row
            row.update(sequence=sequence, phase=phase, command_velocity=[0., 0., 0.],
                body_command_names=body_names, body_command_rad=[float(targets[n]) for n in body_names],
                hand_command_names=hand_names, hand_command_rad=[0.] * len(hand_names),
                contacts=list(step_contacts), **controller_row)
            if not finite_tree(row):
                write('nonfinite_state.json', row); abort = {'sequence': sequence, 'reason': 'nonfinite_state'}; break
            q = np.asarray(row['q_rad']); dq = np.asarray(row['dq_rad_s'])
            estimate = kp[indices] * (command - q[indices]) - kd[indices] * dq[indices]
            row['drive_estimate_nm'] = estimate.tolist()
            row['drive_estimate_names'] = body_names + hand_names
            row['drive_estimate_near_cap_names'] = [n for n, e, cap in zip(body_names + hand_names, estimate, caps[indices]) if abs(e) >= .99 * cap]
            row['drive_estimate_scope'] = 'Unclipped PD estimate; measured generalized efforts also include constraint reactions'
            if equilibrium_mode:
                row['drive_estimate_scope'] = 'Implicit hand12 estimate only; body29 gains are zero, actual explicit body torque is exported separately'
            rows.append(row); state_file.write(json.dumps(row, allow_nan=False) + '\n')
            pelvis = row['link_poses_world_xyzw']['pelvis']; roll, pitch = roll_pitch(pelvis)
            violated = {n: float(q[i]) for i, n in enumerate(names) if q[i] < facts['joint_limits'][n]['lower'] - .1 or q[i] > facts['joint_limits'][n]['upper'] + .1}
            overspeed = {n: float(dq[i]) for i, n in enumerate(names) if abs(dq[i]) > 2 * facts['joint_limits'][n]['velocity']}
            material_self = material_self_penetrations(row['contacts'], adjacent_pairs)
            if pelvis[2] < .65 or max(abs(roll), abs(pitch)) > .35 or violated or overspeed or contact_faults or material_self:
                abort = {'sequence': sequence, 'reason': 'fall_or_source_envelope_abort', 'joint_limit_violations': violated,
                         'overspeed': overspeed, 'contact_faults': contact_faults, 'material_self_penetrations': material_self}
                break
            if (sequence + 1) % 20 == 0:
                before = float(world.current_time); world.render(); assert float(world.current_time) == before
                frame = (sequence + 1) // 20 - 1; files = {}
                for label, camera in cameras.items():
                    pixels = camera.get_rgba(); assert pixels is not None and pixels.shape == (640, 640, 4)
                    name = f'frames/{label}/{frame:06d}.png'; Image.fromarray(pixels.astype(np.uint8)).save(out / name); files[label] = name
                frame_file.write(json.dumps({'frame': frame, 'sequence': sequence, 'physics_s': before, 'phase': phase,
                    'captured_after_same_step_render': True, 'views': files}) + '\n')
        integrity['contact_instrumentation_valid'] = not contact_faults
        policy_receipt['learned_actor_inference_executed'] = any(r.get('policy_inference_this_step', False) for r in rows)
        policy_receipt['body_effort_write_count'] = sum(r.get('body_effort_writes_this_step', 0) for r in rows)
        policy_receipt['body_effort_saturated_steps'] = sum(bool(r.get('body_effort_saturated_names')) for r in rows)
        write('controller_receipt.json', policy_receipt)
        result = evaluate(rows, initial, facts['joint_limits'], facts['mimic_map'], supporting_constraints=supporting_constraints,
                          integrity_checks=integrity, abort=abort, controller_mode=cfg.controller_mode,
                          controller_reference=controller_reference, adjacent_pairs=adjacent_pairs)
        result.update(initial_pelvis_height_m=initial['link_poses_world_xyzw']['pelvis'][2],
            source_mass_kg=facts['source_physical_mass_kg'], physics_dt=.005, actual_contact_points=len(contacts),
            drive_estimate_near_cap_steps=sum(bool(r['drive_estimate_near_cap_names']) for r in rows),
            controller_receipt='controller_receipt.json', source_property_audit='live_inertia_audit.json')
        world.stage.GetRootLayer().Export(str(out / 'scene_final.usda'))
        receipt(result)
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
