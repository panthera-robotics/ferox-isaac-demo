"""Command-driven replay on the fixed-pelvis assembled donor G1: physics generates the response.

Mode ``command-replay``. A private, hashed replay package (validated on the host by
``tools/replay_commands.py`` and re-validated here through the same public contract module) supplies
the embodiment manifest, the converted command rows, and the declared controller gains/home. The
robot is initialized ONCE; afterwards only drive targets are written (never joint, object or root
poses), the source rows are zero-order held at physics time, and every sample records the source
row, the converted targets, the actual simulated feedback, wall time and the policy/external camera
frames. No hardware, no ROS, no ground, no object: FIXED_PELVIS support with a PROVISIONAL donor.
This is offline replay in simulator time; the real-time factor is measured and reported.
"""
import hashlib, json, math, os, sys, time
from pathlib import Path
assert os.environ.get('PANTHERA_SIM_AUTHORIZED') == '1'
assert sorted(p.name for p in Path('/sys/class/net').iterdir()) == ['lo']
assert os.environ.get('PANTHERA_PROBE_MODE') == 'command-replay'
config_path = Path(os.environ['PANTHERA_PROBE_CONFIG']); config = json.loads(config_path.read_text())
out = Path('/evidence'); sys.path[:0] = ['/workspace/ferox_tools', '/workspace/ferox_isaac/twin']
from inspire.embodiment import ContractError, EmbodimentManifest, ReplaySequence, canonical_sha256, dependency_values_from_urdf  # noqa: E402

started_wall = time.monotonic()
package = Path(config['package'])
pkg = json.loads((package / 'package.json').read_text())
files = {}
for name in ('manifest.json', 'sequence.json', 'controller.json'):
    digest = hashlib.sha256((package / name).read_bytes()).hexdigest()
    if pkg['files'][name] != digest:
        raise RuntimeError('replay package file hash mismatch: ' + name)
    files[name] = digest
manifest = EmbodimentManifest.load(package / 'manifest.json')
sequence_data = json.loads((package / 'sequence.json').read_text())
sequence = ReplaySequence(manifest, sequence_data['rows'], hand_contracts=sequence_data['hand_contracts'], source=sequence_data['source'],
                          maximum_step_s=sequence_data.get('maximum_step_s', 1.0))
if sequence.contract_sha256 != pkg['contract_sha256'] or manifest.sha256 != pkg['manifest_sha256']:
    raise RuntimeError('replay package contract hash does not match the validated package')
controller = json.loads((package / 'controller.json').read_text())
frame_every = int(config.get('frame_every', 8)); assert 1 <= frame_every <= 40
maximum_steps = int(config.get('maximum_steps', 4000)); assert 100 <= maximum_steps <= 20000
lead_in_s = float(config.get('lead_in_s', 0.5)); assert 0.0 <= lead_in_s <= 5.0
dt = .005
steps = min(maximum_steps, int(round((lead_in_s + sequence.duration_s) / dt)) + 1)

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting'})
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, PhysxSchema, PhysicsSchemaTools, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402
from omni.physx import get_physx_simulation_interface  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
enable_extension('omni.pip.compute')
from inspire_body_asset import import_body  # noqa: E402
from inspire_collision import replace_palm_with_components, replace_left_thumb_with_slabs  # noqa: E402
from rigid_inertia import audit_live_properties  # noqa: E402

source = Path('/source-assets/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf')
if hashlib.sha256(source.read_bytes()).hexdigest() != manifest.data['source_asset']['urdf_sha256']:
    raise RuntimeError('mounted source asset differs from the manifest asset hash')


def palm_builder(stage, mesh, body, side):
    return replace_palm_with_components(stage, mesh, body, contact_offset_m=.0012860533315688372, rest_offset_m=0.,
                                        candidate_id='ftp_palm_yz_slabs_v2' if side == 'right' else 'ftp_left_palm_yz_slabs_v1')


asset, facts = import_body(source, out, fixed_base=True, palm_builder=palm_builder, left_thumb_builder=replace_left_thumb_with_slabs)
body_names = list(manifest.body_names)
assert set(body_names) == set(facts['body_joint_names']) and len(body_names) == 29
hand_names = list(manifest.hand_joint_names('left')) + list(manifest.hand_joint_names('right'))
assert set(hand_names) == set(facts['hand_independent_names'])
for child, m in facts['mimic_map'].items():
    side = 'left' if child.startswith('left_') else 'right'
    declared = manifest.data['hands'][side]['coupled_joints'][child]
    assert declared['parent'] == m['parent'] and abs(declared['multiplier'] - m['multiplier']) < 1e-9, child
limits = facts['joint_limits']
for n in body_names:
    lo, hi = manifest.body_limit(n)
    assert abs(lo - limits[n]['lower']) < 1e-9 and abs(hi - limits[n]['upper']) < 1e-9, n
# Second-layer qualification/transform validity against the MOUNTED inputs (identity, not physical correctness).
COLLISION_COOKING = 'right=ftp_palm_yz_slabs_v2;left=ftp_left_palm_yz_slabs_v1;contact_offset_m=0.0012860533315688372;rest_offset_m=0'
live_dependencies = dict(dependency_values_from_urdf(source, collision_cooking=COLLISION_COOKING), support='FIXED_PELVIS', controller='implicit_biased_drive_v1 replay controller (package-hashed gains)')
qualification_validity = manifest.check_validity(live_dependencies)
if any(v['status'] != 'VALID' for v in qualification_validity['transforms'].values()):
    raise RuntimeError('transform validity failed against the mounted asset: %s' % json.dumps(qualification_validity['transforms']))
if pkg.get('validation', {}).get('qualification_validity') is None:
    raise RuntimeError('replay package was admitted without a qualification validity record (rebuild with --source-urdf)')
home = controller['body_home_rad']; kp_body = controller['body_kp_nm_rad']; kd_body = controller['body_kd_nm_s_rad']
assert set(home) == set(kp_body) == set(kd_body) == set(body_names)
for n in body_names:
    assert limits[n]['lower'] <= home[n] <= limits[n]['upper'] and math.isfinite(kp_body[n]) and kp_body[n] > 0 and math.isfinite(kd_body[n]) and kd_body[n] >= 0
hand_kp, hand_kd = float(controller['hand_kp_nm_rad']), float(controller['hand_kd_nm_s_rad'])
assert 0 < hand_kp <= 10 and 0 <= hand_kd <= 1

stage = Usd.Stage.Open(str(asset))
for prim in stage.Traverse():
    n = prim.GetName()
    if prim.IsA(UsdPhysics.RevoluteJoint) and n in body_names:
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular'); drive.CreateTypeAttr('force')
        drive.CreateMaxForceAttr(limits[n]['effort']); drive.CreateStiffnessAttr(kp_body[n]); drive.CreateDampingAttr(kd_body[n])
        drive.CreateTargetPositionAttr(float(np.rad2deg(home[n])))
stage.GetRootLayer().Save()
world = World(stage_units_in_meters=1., physics_dt=dt, rendering_dt=.02)
scene = next(p for p in world.stage.Traverse() if p.IsA(UsdPhysics.Scene))
PhysxSchema.PhysxSceneAPI.Apply(scene).CreateEnableExternalForcesEveryIterationAttr(True)
light = UsdLux.DomeLight.Define(world.stage, '/World/Fill'); light.CreateIntensityAttr(350.); light.CreateColorAttr(Gf.Vec3f(.45, .52, .65))
key = UsdLux.DistantLight.Define(world.stage, '/World/Key'); key.CreateIntensityAttr(500.)
UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(-35, -25, -40))
add_reference_to_stage(str(asset), '/World/G1')
root = UsdGeom.Xformable(world.stage.GetPrimAtPath('/World/G1')); root.ClearXformOpOrder(); root.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.))
for p in world.stage.Traverse():
    if p.HasAPI(PhysxSchema.PhysxArticulationAPI):
        api = PhysxSchema.PhysxArticulationAPI(p); api.CreateSolverPositionIterationCountAttr(32); api.CreateSolverVelocityIterationCountAttr(8)
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
contacts = []; trace = []; phase = 'initialization'
contact_file = (out / 'contacts.jsonl').open('w', buffering=1)


def on_contact(headers, data):
    for h in headers:
        for k in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
            d = data[k]
            r = {'sequence': None if phase == 'initialization' else len(trace), 'physics_s': world.current_time, 'phase': phase,
                 'actor0': str(PhysicsSchemaTools.intToSdfPath(h.actor0)), 'actor1': str(PhysicsSchemaTools.intToSdfPath(h.actor1)),
                 'position_world_m': list(map(float, d.position)), 'normal_world': list(map(float, d.normal)), 'impulse_ns': list(map(float, d.impulse)),
                 'separation_m': float(d.separation), 'source': 'simulated_proxy'}
            contacts.append(r); contact_file.write(json.dumps(r, allow_nan=False) + '\n')


subscription = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
robot = SingleArticulation('/World/G1', name='provisional_assembled_g1')
world.reset(); robot.initialize()
inertia_audit = audit_live_properties(SimulationManager.get_physics_sim_view(), '/World/G1', facts['expected_source_rigid_properties_in_imported_frame'])
(out / 'live_inertia_audit.json').write_text(json.dumps(inertia_audit, indent=2, allow_nan=False))
names = list(robot.dof_names); assert set(names) == set(facts['joint_limits']) and len(names) == 53
body_ids = np.asarray([names.index(n) for n in body_names], dtype=np.int32)
hand_ids = np.asarray([names.index(n) for n in hand_names], dtype=np.int32)
kp = np.zeros(53, dtype=np.float32); kd = np.zeros(53, dtype=np.float32)
kp[body_ids] = [kp_body[n] for n in body_names]; kd[body_ids] = [kd_body[n] for n in body_names]
kp[hand_ids] = hand_kp; kd[hand_ids] = hand_kd
robot._articulation_view.set_gains(kp, kd)
got = robot.get_articulation_controller().get_gains(); assert np.allclose(np.ravel(got[0]), kp) and np.allclose(np.ravel(got[1]), kd)
gain_readback = {'body': {n: [float(kp[names.index(n)]), float(kd[names.index(n)])] for n in body_names}, 'hand': {n: [float(kp[names.index(n)]), float(kd[names.index(n)])] for n in hand_names}}
(out / 'implicit_gain_readback.json').write_text(json.dumps(gain_readback, indent=2))

# Initial episode state, written ONCE: body home overridden by the first row's body targets where it
# names them (a recorded pose of the real robot); every hand joint at the URDF open pose. A closed first
# hand command is reached by the drives during the lead-in (targets ramped open -> first row), never by
# writing an interpenetrating closed pose into the articulation.
first = sequence.converted[0]
q0 = np.zeros(53, dtype=np.float32)
initial_body = {n: first['body_targets_rad'].get(n, home[n]) for n in body_names}
q0[body_ids] = [initial_body[n] for n in body_names]
open_hand = {n: 0.0 for n in hand_names}
first_hand = dict(open_hand)
for side, block in first['hands'].items():
    first_hand.update(block['targets_rad'])
if any(abs(v) > 1e-9 for v in first_hand.values()):
    assert lead_in_s >= 0.5, 'a closed first hand command needs a lead-in of at least 0.5 s to be reached physically'
robot.set_joint_positions(q0); robot.set_joint_velocities(np.zeros(53, dtype=np.float32))
views = {n: SimulationManager.get_physics_sim_view().create_rigid_body_view('/World/G1/' + n) for n in ['pelvis', 'torso_link', 'right_wrist_yaw_link', 'left_wrist_yaw_link', 'right_base_link', 'left_base_link']}
assert all(v.count == 1 for v in views.values())

# Cameras: two external views and the policy camera on the torso at the donor URDF d435 mount.
cameras = {}
closeup = config.get('closeup_camera', {'position': [0.85, -0.85, 1.2], 'target': [0.2, -0.2, 0.92]})   # right-hand workspace close-up (world-fixed camera view, not a bilateral trial)
for label, position, target in [('front', (2.4, -2.4, 1.7), (0, 0, 1.0)), ('side', (-.3, 3.2, 1.6), (0, 0, 1.0)), ('closeup_right_hand', tuple(closeup['position']), tuple(closeup['target']))]:
    camera = Camera('/World/' + label + 'Camera', resolution=(640, 640)); camera.initialize(); camera.set_clipping_range(.01, 10.)
    x = UsdGeom.Xformable(camera.prim); x.ClearXformOpOrder(); x.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*position), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse())
    cameras[label] = camera; (out / 'frames' / label).mkdir(parents=True)
cam_spec = manifest.data['cameras']['policy_head_d435_color_nominal']
mount = cam_spec['mount']   # torso_link -> d435_link fixed joint from the donor URDF, carried by the manifest
r_, p_, y_ = mount['rpy_rad']
cr, sr, cp, sp, cy, sy = math.cos(r_), math.sin(r_), math.cos(p_), math.sin(p_), math.cos(y_), math.sin(y_)
R_link = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])
R_link_cam = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])   # USD camera looks along -Z with +Y up; URDF camera link looks along +X with +Z up
T = np.eye(4); T[:3, :3] = R_link @ R_link_cam; T[:3, 3] = mount['xyz_m']
policy = Camera('/World/G1/%s/policy_camera' % mount['parent_link'], resolution=(640, 480)); policy.initialize(); policy.set_clipping_range(.05, 10.)
x = UsdGeom.Xformable(policy.prim); x.ClearXformOpOrder(); x.AddTransformOp().Set(Gf.Matrix4d(T.T.tolist()))
aperture = float(policy.get_horizontal_aperture()); policy.set_focal_length(aperture / (2. * math.tan(math.radians(cam_spec['calibration']['horizontal_fov_deg']) / 2.)))
cameras['policy'] = policy; (out / 'frames' / 'policy').mkdir(parents=True)
frame_file = (out / 'frames.jsonl').open('w', buffering=1); state_file = (out / 'state.jsonl').open('w', buffering=1)
command_file = (out / 'commands.jsonl').open('w', buffering=1)
phase = 'lead_in'; aborted = None; rejections = []; applied_rows = set(); lead_in_steps = int(round(lead_in_s / dt))
body_target = np.array([initial_body[n] for n in body_names], dtype=np.float32)
hand_open = np.array([open_hand[n] for n in hand_names], dtype=np.float32); hand_first = np.array([first_hand[n] for n in hand_names], dtype=np.float32)
hand_target = hand_open.copy()
loop_wall_start = time.monotonic()
for tick in range(steps):
    t_source = (tick - lead_in_steps) * dt + sequence.converted[0]['t_s']
    row = sequence.active_row(t_source) if tick >= lead_in_steps else None
    if row is None and lead_in_steps > 0:
        alpha = min(1.0, (tick + 1) / lead_in_steps)
        hand_target = (1.0 - alpha) * hand_open + alpha * hand_first
    if row is not None:
        phase = 'replay'
        if row['row'] not in applied_rows:
            applied_rows.add(row['row'])
            for n, v in row['body_targets_rad'].items():
                body_target[body_names.index(n)] = v
            for side, block in row['hands'].items():
                for n, v in block['targets_rad'].items():
                    hand_target[hand_names.index(n)] = v
            command_file.write(json.dumps({'sequence': tick, 'physics_s': world.current_time, 'wall_s': time.monotonic() - loop_wall_start, 'source_row': row['row'], 'source_t_s': row['t_s'],
                                           'body_targets_rad': row['body_targets_rad'], 'hand_targets_rad': {s: b['targets_rad'] for s, b in row['hands'].items()},
                                           'hand_closure': {s: b['closure'] for s, b in row['hands'].items()}, 'clipped_axes': {s: b['clipped_axes'] for s, b in row['hands'].items()}}, allow_nan=False) + '\n')
    robot.apply_action(ArticulationAction(joint_positions=body_target, joint_indices=body_ids))
    robot.apply_action(ArticulationAction(joint_positions=hand_target, joint_indices=hand_ids))
    world.step(render=False)
    if (tick + 1) % frame_every == 0:
        world.render()
    q = np.ravel(robot.get_joint_positions()); dq = np.ravel(robot.get_joint_velocities()); effort = np.ravel(robot.get_measured_joint_efforts())
    if not (np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(effort).all()):
        aborted = {'sequence': tick, 'reason': 'nonfinite_physics_state'}; break
    poses = {n: np.asarray(v.get_transforms())[0].tolist() for n, v in views.items()}
    coupling = {n: float(q[names.index(n)] - (m['multiplier'] * q[names.index(m['parent'])] + m['offset'])) for n, m in facts['mimic_map'].items()}
    r = {'sequence': tick, 'physics_s': world.current_time, 'wall_s': time.monotonic() - loop_wall_start, 'phase': phase, 'source_row': None if row is None else row['row'],
         'source_t_s': None if row is None else row['t_s'], 'runtime_names': names, 'q_rad': q.tolist(), 'dq_rad_s': dq.tolist(), 'measured_generalized_effort_nm': effort.tolist(),
         'body_command_names': body_names, 'body_command_rad': body_target.tolist(), 'hand_command_names': hand_names, 'hand_command_rad': hand_target.tolist(),
         'link_poses_world_xyzw': poses, 'coupling_error_rad': coupling}
    trace.append(r); state_file.write(json.dumps(r, allow_nan=False) + '\n')
    violated = {n: float(q[i]) for i, n in enumerate(names) if q[i] < facts['joint_limits'][n]['lower'] - .1 or q[i] > facts['joint_limits'][n]['upper'] + .1}
    overspeed = {n: float(dq[i]) for i, n in enumerate(names) if abs(dq[i]) > 2 * facts['joint_limits'][n]['velocity']}
    if violated or overspeed:
        aborted = {'sequence': tick, 'reason': 'source_envelope_abort', 'joint_limit_violations_rad': violated, 'joint_velocity_violations_rad_s': overspeed}; break
    if (tick + 1) % frame_every == 0:
        frame = (tick + 1) // frame_every - 1; files_ = {}
        for label, camera in cameras.items():
            pixels = camera.get_rgba(); extra = 0
            while (pixels is None or pixels.size == 0) and extra < 3:
                world.render(); extra += 1; pixels = camera.get_rgba()
            assert pixels is not None and pixels.ndim == 3, label
            f = f'frames/{label}/{frame:06d}.png'; Image.fromarray(pixels[..., :3].astype(np.uint8) if label == 'policy' else pixels.astype(np.uint8)).save(out / f); files_[label] = f
        frame_file.write(json.dumps({'frame': frame, 'sequence': tick, 'physics_s': world.current_time, 'wall_s': time.monotonic() - loop_wall_start, 'phase': phase, 'source_row': None if row is None else row['row'],
                                     'captured_after_same_step_render': True, 'views': files_, 'policy_camera': 'RGB 640x480 nominal D435 mount (NOT the dataset stereo camera)'}) + '\n')
loop_wall = time.monotonic() - loop_wall_start
state_file.close(); contact_file.close(); frame_file.close(); command_file.close()

# Tracking: commanded target vs measured position per commanded joint (replay phase only).
replay_rows = [r for r in trace if r['phase'] == 'replay']
commanded_body = sorted({n for c in sequence.converted for n in c['body_targets_rad']})
commanded_hand = sorted({n for c in sequence.converted for s in c['hands'].values() for n in s['targets_rad']})
tracking = {}
for n in commanded_body + commanded_hand:
    i = names.index(n); ci = (body_names.index(n), 'body_command_rad') if n in body_names else (hand_names.index(n), 'hand_command_rad')
    err = [abs(r['q_rad'][i] - r[ci[1]][ci[0]]) for r in replay_rows]
    tracking[n] = {'mean_abs_rad': float(np.mean(err)) if err else None, 'max_abs_rad': float(np.max(err)) if err else None, 'final_abs_rad': float(err[-1]) if err else None}
self_contacts = [c for c in contacts if c['sequence'] is not None]
max_coupling = max((abs(e) for r in trace for e in r['coupling_error_rad'].values()), default=0.)
checks = {'package_hashes_verified': True, 'manifest_asset_hash_matches_mounted_urdf': True, 'exact53_named_coordinates': len(names) == 53,
          'initialized_once_no_pose_writes_during_replay': True, 'all_rows_applied': len(applied_rows) == len(sequence.converted),
          'no_command_rejected_after_validation': not rejections, 'numerical_abort_absent': aborted is None, 'exact_steps': len(trace) == steps,
          'hand_coupling_below_0_03rad': max_coupling < .03, 'physics_dt': len(trace) > 1 and bool(np.allclose(np.diff([r['physics_s'] for r in trace]), dt, atol=1e-8)),
          'source_mass_com_inertia_preserved': all(inertia_audit['checks'].values()), 'policy_camera_frames_written': any((out / 'frames/policy').iterdir()),
          'fixed_pelvis_matches_declared_pose': bool(all(np.linalg.norm(np.asarray(r['link_poses_world_xyzw']['pelvis'][:3]) - [0, 0, 1]) < 1e-4 for r in trace))}
metrics = {'status': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks, 'scope': 'fixed_pelvis_command_driven_replay_provisional_donor',
           'replay_mode': 'command_driven_simulation (mode 2): initialize once, drive targets only, physics generates the response',
           'source': sequence.source, 'sequence_summary': sequence.summary(), 'package_files_sha256': files, 'package_sha256': pkg,
           'qualification_validity': qualification_validity, 'live_dependencies': live_dependencies,
           'controller': {'type': manifest.data['controller']['type'], 'provenance': controller.get('provenance'), 'hand_kp_nm_rad': hand_kp, 'hand_kd_nm_s_rad': hand_kd,
                          'body_gains_sha256': canonical_sha256({'kp': kp_body, 'kd': kd_body, 'home': home})},
           'initialization': {'body': 'first row targets where named, else controller home (written once)', 'hands': 'URDF open pose written once; targets ramped open -> first row over the lead-in', 'lead_in_s': lead_in_s},
           'steps': len(trace), 'physics_dt': dt, 'lead_in_s': lead_in_s, 'simulated_s': len(trace) * dt, 'loop_wall_s': loop_wall,
           'real_time_factor_loop': (len(trace) * dt) / loop_wall if loop_wall > 0 else None, 'offline_replay': True, 'wall_since_probe_start_s': time.monotonic() - started_wall,
           'runtime_names': names, 'commanded_body_joints': commanded_body, 'commanded_hand_joints': commanded_hand, 'tracking_abs_error_rad': tracking,
           'rows_applied': len(applied_rows), 'rows_total': len(sequence.converted), 'clipped_rows': sequence.clipped_rows, 'rejections': rejections, 'abort': aborted,
           'contact_points_during_replay': len(self_contacts), 'contact_pairs': sorted({tuple(sorted((c['actor0'], c['actor1']))) for c in self_contacts})[:40],
           'coupling_error_max_rad': max_coupling, 'fixed_base': True, 'support_constraints': ['pelvis_fixed_to_world_1m_above_origin'], 'ground_present': False, 'objects_present': False,
           'hardware_authorized': False, 'exact_asset_qualified': False, 'source_model': 'Unitree_FTP_G1_provisional_donor', 'manifest_id': manifest.data['manifest_id'], 'manifest_sha256': manifest.sha256,
           'grasp_qualification': 'NOT_RUN', 'writing_qualification': 'NOT_RUN', 'standing_qualification': 'NOT_RUN', 'real_data_agreement': 'NOT_TESTED_IN_PROBE (host-side comparison only)',
           'media_labels': {'fixture': 'FIXED PELVIS - COMMAND REPLAY (no ground, no object)', 'embodiment': 'PROVISIONAL G1 + bilateral FTP donor hands',
                            'source': '%s: %s' % (sequence.source['kind'], sequence.source['source_id']), 'qualification': 'Software/physics integration only; no grasp, writing, standing or real-data agreement qualification'},
           'source_property_audit': 'live_inertia_audit.json'}
(out / 'metrics.json').write_text(json.dumps(metrics, indent=2, allow_nan=False))
world.stage.GetRootLayer().Export(str(out / 'assembled_scene.usda'))
artifacts = [str(p.relative_to(out)) for p in out.rglob('*') if p.is_file() and p.name not in ['run.json', 'probe.json', 'console.log', 'executed_probe.py', 'executed_launcher.py', 'uncommitted.patch']]
(out / 'probe.json').write_text(json.dumps({'status': metrics['status'], 'scope': metrics['scope'], 'metrics': 'metrics.json', 'artifacts': artifacts}))
app.close()
