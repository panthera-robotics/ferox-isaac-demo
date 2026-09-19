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
from inspire.embodiment import ARM_DATUM_SCRIPTED, ContractError, EmbodimentManifest, ReplaySequence, canonical_sha256, runtime_dependency_values  # noqa: E402
from inspire.body_feedforward import bounded_gravity_feedforward  # noqa: E402
from inspire import loop_ipc  # noqa: E402
from inspire.closed_loop_targets import ClosedLoopTargets, TargetError  # noqa: E402
from inspire.model_action_adapter import validate_piston_chunk, PISTON_DIMS, PISTON_HAND_ORDER  # noqa: E402
from inspire.embodiment import HandCommandAdapter, HAND_ACTUATORS  # noqa: E402
from inspire.operational_observer import OBSERVERS, LEGACY, make_observer  # noqa: E402

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
# Declared physics step: the manifest binds physics_dt_s = 0.005 for every qualification; any other value is a DIAGNOSTIC
# refinement (hand-req-02 F7 convergence check) and stales the bound claims by construction (recorded in metrics['physics_dt']).
dt = float(config.get('physics_dt_s', 0.005)); assert 0.001 <= dt <= 0.005 and abs(round(0.005 / dt) * dt - 0.005) < 1e-9, 'physics_dt_s must divide 0.005 s'
# Sprint O owner decision A: `observer` selects the velocity guard for NEW runs. `legacy` (default) keeps the historical rule (reported
# velocity > 2x the joint's own URDF field aborts). `simulation_operational_v1` aborts on the SAMPLED-INTERVAL velocity (mimic children
# at |multiplier| x parent field, persist 2) or a coupling fault; the legacy rule is then scored only in `legacy_envelope_shadow`.
observer_name = str(config.get('observer', LEGACY)); assert observer_name in OBSERVERS, 'observer must be one of %s' % (OBSERVERS,)
steps = min(maximum_steps, int(round((lead_in_s + sequence.duration_s) / dt)) + 1)
# Closed-loop mode (sprint K K4): the package's single row is the START pose; after the lead-in the probe publishes fresh
# observations to a co-admitted model sidecar over private file IPC and executes a short validated prefix of each
# returned chunk through the same controller. Physics pauses while inferring (non-real-time closed-loop simulation).
closed_loop = config.get('closed_loop')
if closed_loop:
    assert closed_loop.get('schema') == 'closed_loop_v1' and closed_loop['control_source'] in ('MODEL_CLOSED_LOOP', 'HYBRID_MODEL_ARMS_SCRIPTED_HANDS')
    cl_model_ticks = int(round(float(closed_loop.get('model_step_s', 0.02)) / dt)); assert cl_model_ticks >= 1
    cl_prefix = int(closed_loop['prefix_steps']); cl_iters = int(closed_loop['iterations']); assert 1 <= cl_prefix <= 30 and 1 <= cl_iters <= 200
    cl_tail_ticks = int(round(float(closed_loop.get('tail_hold_s', 0.0)) / dt))
    # Optional scripted prefix: the package rows are replayed for handover_after_s after the lead-in (e.g. a proven
    # approach to a hover pose), then the model takes over from the last scripted targets. 0 = hand over at the lead-in end.
    cl_handover_ticks = int(round(float(closed_loop.get('handover_after_s', 0.0)) / dt)); assert 0 <= cl_handover_ticks
    # Declared arm-state datum hypothesis: 'urdf_zero' sends absolute URDF radians (default); 'handover_pose' sends arm
    # states relative to the arm targets held at the hand-over and adds the same offset back to the model's arm outputs
    # before validation (limits are checked on absolute radians). The offset is recorded in every iteration record.
    cl_datum_mode = closed_loop.get('arm_state_datum', 'urdf_zero'); assert cl_datum_mode in ('urdf_zero', 'handover_pose')
    steps = min(maximum_steps, int(round(lead_in_s / dt)) + cl_handover_ticks + cl_iters * cl_prefix * cl_model_ticks + cl_tail_ticks + 1)
    cl_token = os.environ['PANTHERA_SIM_RUN_ID']; assert len(cl_token) >= 8   # the admitted run id; the launcher gives the sidecar the same token
    cl_ipc = loop_ipc.layout(out / 'ipc'); cl_timeout_first = float(closed_loop.get('inference_timeout_first_s', 90.0)); cl_timeout = float(closed_loop.get('inference_timeout_s', 20.0))
    cl_arm_names = [n for side in ('left', 'right') for n in ('%s_shoulder_pitch_joint' % side, '%s_shoulder_roll_joint' % side, '%s_shoulder_yaw_joint' % side, '%s_elbow_joint' % side, '%s_wrist_roll_joint' % side, '%s_wrist_pitch_joint' % side, '%s_wrist_yaw_joint' % side)]
    cl_state = {'iteration': 0, 'chunk': None, 'chunk_pos': 0, 'model_tick': 0, 'records': [], 'refusals': [], 'hand_phase': 'open', 'close_started_tick': None, 'obs_count': 0, 'interventions': [], 'ramp_to': None}

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

source = Path('/source-assets') / manifest.data['source_asset'].get('urdf_name', 'g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf')   # the manifest names the asset variant; its hash is checked below
if hashlib.sha256(source.read_bytes()).hexdigest() != manifest.data['source_asset']['urdf_sha256']:
    raise RuntimeError('mounted source asset differs from the manifest asset hash')


def palm_builder(stage, mesh, body, side):
    return replace_palm_with_components(stage, mesh, body, contact_offset_m=.0012860533315688372, rest_offset_m=0.,
                                        candidate_id='ftp_palm_yz_slabs_v2' if side == 'right' else 'ftp_left_palm_yz_slabs_v1')


asset, facts = import_body(source, out, fixed_base=True, palm_builder=palm_builder, left_thumb_builder=replace_left_thumb_with_slabs)
operational_observer = make_observer(observer_name, facts['joint_limits'], facts['mimic_map'], dt, threshold_multiple=2.0, persist=2)
legacy_shadow_summary = {'over_2x_samples': 0, 'first': None, 'would_abort': False}
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
# The live values carry what this run ACTUALLY applies: the mounted asset, the actual physics step (a DIAGNOSTIC refinement
# stales every dt-bound claim), the applied hand conversion profile(s) and the arm datum. Scripted replay applies the
# package's hand contracts on absolute URDF radians; a closed-loop run rebinds both below once its adapters exist.
CONTROLLER_DESCRIPTOR = 'implicit_biased_drive_v1 replay controller (package-hashed gains)'
live_dependencies = runtime_dependency_values(source, collision_cooking=COLLISION_COOKING, physics_dt_s=dt, support='FIXED_PELVIS', controller=CONTROLLER_DESCRIPTOR,
                                              hand_contracts=sequence.adapters, arm_datum=ARM_DATUM_SCRIPTED)
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
# Optional declared body gravity feed-forward (controller v4+): the implicit PD drives stay as declared; a bounded
# model-based joint effort equal to the articulation's generalized gravity force at the CURRENT configuration (the
# executed asset's masses/inertias/COMs incl. the mounted donor hands, fixed base) is added on the commanded body joints
# only, ramped in over the lead-in. The combined effort (estimated PD term + feed-forward) is capped at the URDF effort
# limit per joint; every cap is counted. Hand joints and coupled joints never receive feed-forward.
gff = controller.get('gravity_feedforward') or {'enabled': False}
assert isinstance(gff, dict) and gff.get('enabled') in (True, False)
if gff['enabled']:
    assert gff.get('source') == 'articulation_generalized_gravity_forces_current_configuration' and gff.get('joints') == 'commanded_body_joints'
    gff_ramp_s = float(gff.get('ramp_in_s', 1.0)); assert 0.0 <= gff_ramp_s <= 5.0
    gff_scale = float(gff.get('scale', 1.0)); assert 0.0 < gff_scale <= 1.0
    gff_limit = gff.get('combined_effort_limit'); assert gff_limit == 'urdf_effort_limit'

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
scene = config.get('scene')
scene_facts = None
if scene:
    # Authored diagnostic manipulation scene (pelvis frame at (0,0,1)): static table, ONE free rigid object (retained holder dimensions),
    # a visual destination disk. The object is a genuine rigid body: no weld, attachment, kinematic hold or external force at any time.
    from pxr import UsdShade
    pz = 1.0
    def material(path, static, dynamic):
        obj = UsdShade.Material.Define(world.stage, path); api = UsdPhysics.MaterialAPI.Apply(obj.GetPrim())
        api.CreateStaticFrictionAttr(static); api.CreateDynamicFrictionAttr(dynamic); api.CreateRestitutionAttr(0.); return obj
    mat = material('/World/Scene/ContactMaterial', float(scene.get('static_friction', .7)), float(scene.get('dynamic_friction', .6)))
    def collide(prim):
        UsdPhysics.CollisionAPI.Apply(prim); capi = PhysxSchema.PhysxCollisionAPI.Apply(prim); capi.CreateContactOffsetAttr(.001); capi.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, 'physics')
    t = scene['table']   # {"center_xy_m": [x, y], "size_m": [sx, sy, thickness], "top_z_pelvis_m": z}
    table = UsdGeom.Cube.Define(world.stage, '/World/Scene/Table'); table.CreateSizeAttr(1.)
    table.AddTranslateOp().Set(Gf.Vec3d(t['center_xy_m'][0], t['center_xy_m'][1], pz + t['top_z_pelvis_m'] - t['size_m'][2] / 2.)); table.AddScaleOp().Set(Gf.Vec3f(*t['size_m']))
    table.CreateDisplayColorAttr([Gf.Vec3f(.92, .92, .9)]); collide(table.GetPrim())   # static collider (no RigidBodyAPI)
    o = scene['object']  # {"center_pelvis_m": [x,y,z], "radius_m": r, "length_m": L, "mass_kg": m}
    obj_x = UsdGeom.Xform.Define(world.stage, '/World/Scene/Object'); obj_x.AddTranslateOp().Set(Gf.Vec3d(o['center_pelvis_m'][0], o['center_pelvis_m'][1], pz + o['center_pelvis_m'][2]))
    obj_prim = obj_x.GetPrim(); UsdPhysics.RigidBodyAPI.Apply(obj_prim).CreateKinematicEnabledAttr(False)
    massapi = UsdPhysics.MassAPI.Apply(obj_prim); massapi.CreateMassAttr(float(o['mass_kg']))
    r_, L_, m_ = float(o['radius_m']), float(o['length_m']), float(o['mass_kg'])
    massapi.CreateDiagonalInertiaAttr(Gf.Vec3f(m_ * (3 * r_ * r_ + L_ * L_) / 12., m_ * (3 * r_ * r_ + L_ * L_) / 12., m_ * r_ * r_ / 2.)); massapi.CreatePrincipalAxesAttr(Gf.Quatf(1.))
    PhysxSchema.PhysxContactReportAPI.Apply(obj_prim).CreateThresholdAttr(0.); PhysxSchema.PhysxRigidBodyAPI.Apply(obj_prim).CreateSleepThresholdAttr(0.)
    cyl = UsdGeom.Cylinder.Define(world.stage, '/World/Scene/Object/Body'); cyl.CreateAxisAttr('Z'); cyl.CreateRadiusAttr(r_); cyl.CreateHeightAttr(L_)
    cyl.CreateDisplayColorAttr([Gf.Vec3f(.15, .15, .18)]); collide(cyl.GetPrim())
    # Sprint O (declared props, optional): a base disc under the rod (paper-roll-holder / stand shape) as a second collider of the SAME
    # rigid body: object.base = {"radius_m": R, "thickness_m": T, "mass_kg": M}; the disc sits under the rod's bottom end (its top at the
    # rod bottom); total mass = rod + base with the combined inertia about the combined COM; dimensions declared before physics.
    base_ = o.get('base')
    if base_:
        Rb, Tb, Mb = float(base_['radius_m']), float(base_['thickness_m']), float(base_['mass_kg'])
        disc = UsdGeom.Cylinder.Define(world.stage, '/World/Scene/Object/Base'); disc.CreateAxisAttr('Z'); disc.CreateRadiusAttr(Rb); disc.CreateHeightAttr(Tb)
        disc.AddTranslateOp().Set(Gf.Vec3d(0., 0., -L_ / 2. - Tb / 2.)); disc.CreateDisplayColorAttr([Gf.Vec3f(.25, .25, .28)]); collide(disc.GetPrim())
        Mt = m_ + Mb; zc = (m_ * 0. + Mb * (-L_ / 2. - Tb / 2.)) / Mt   # combined COM along the rod axis (rod centre at 0)
        Irod_xy = m_ * (3 * r_ * r_ + L_ * L_) / 12.; Idisc_xy = Mb * (3 * Rb * Rb + Tb * Tb) / 12.
        Ixy = Irod_xy + m_ * zc * zc + Idisc_xy + Mb * (-L_ / 2. - Tb / 2. - zc) ** 2; Iz = m_ * r_ * r_ / 2. + Mb * Rb * Rb / 2.
        massapi.CreateMassAttr(Mt); massapi.CreateCenterOfMassAttr(Gf.Vec3f(0., 0., zc)); massapi.CreateDiagonalInertiaAttr(Gf.Vec3f(Ixy, Ixy, Iz))
    # Sprint O (declared props, optional): static stems/pedestals standing on the table top, e.g. a pen-holder stem under a tube,
    # so that a fingers-down hook grasp can reach the object with the fingertips hanging below its bottom. Static colliders
    # (no RigidBodyAPI), same contact material, dimensions declared in the probe config before physics; recorded in scene_facts.
    pedestals = scene.get('pedestals') or []
    for k_, ped in enumerate(pedestals):   # {"center_xy_m": [x, y], "radius_m": r, "height_m": h}
        stem = UsdGeom.Cylinder.Define(world.stage, '/World/Scene/Pedestal%d' % k_); stem.CreateAxisAttr('Z'); stem.CreateRadiusAttr(float(ped['radius_m'])); stem.CreateHeightAttr(float(ped['height_m']))
        stem.AddTranslateOp().Set(Gf.Vec3d(ped['center_xy_m'][0], ped['center_xy_m'][1], pz + t['top_z_pelvis_m'] + float(ped['height_m']) / 2.)); stem.CreateDisplayColorAttr([Gf.Vec3f(.55, .55, .58)]); collide(stem.GetPrim())
    d = scene['destination']  # {"center_xy_m": [x, y], "radius_m": r}
    disk = UsdGeom.Cylinder.Define(world.stage, '/World/Scene/DestinationMarker'); disk.CreateAxisAttr('Z'); disk.CreateRadiusAttr(float(d['radius_m'])); disk.CreateHeightAttr(.002)
    disk_z = pz + t['top_z_pelvis_m'] + .001 + max([float(ped['height_m']) for ped in pedestals if abs(ped['center_xy_m'][0] - d['center_xy_m'][0]) < 1e-6 and abs(ped['center_xy_m'][1] - d['center_xy_m'][1]) < 1e-6] or [0.0])
    disk.AddTranslateOp().Set(Gf.Vec3d(d['center_xy_m'][0], d['center_xy_m'][1], disk_z)); disk.CreateDisplayColorAttr([Gf.Vec3f(.2, .6, .9)])   # visual only, no collision
    scene_facts = {'table_top_z_world_m': pz + t['top_z_pelvis_m'], 'object_center_world_m': [o['center_pelvis_m'][0], o['center_pelvis_m'][1], pz + o['center_pelvis_m'][2]], 'object': o, 'table': t, 'destination': d,
                   'pedestals': pedestals, 'object_base': o.get('base'), 'object_prim': '/World/Scene/Object', 'object_is_free_rigid_body': True, 'attachments_or_welds': None, 'material': {'static': float(scene.get('static_friction', .7)), 'dynamic': float(scene.get('dynamic_friction', .6))},
                   'floor_present': False, 'support': 'pelvis fixed to the world at z = 1.0 m (shown in all views)'}
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


# DIAGNOSTIC only (sprint L C4): declared self-collision pair filters, e.g. the donor torso vs the right upper arm, to test
# reachability of a recorded posture that the twin's own collision meshes block. Non-qualifying by construction: refused
# unless the execution label is DIAGNOSTIC; every applied pair is recorded in metrics['diagnostic_collision_filters'].
diagnostic_collision_filters = config.get('diagnostic_collision_filters') or []
if diagnostic_collision_filters:
    assert config.get('execution_label') == 'DIAGNOSTIC', 'diagnostic_collision_filters is DIAGNOSTIC-only'
    for a_, b_ in diagnostic_collision_filters:
        pa_, pb_ = world.stage.GetPrimAtPath('/World/G1/' + a_), world.stage.GetPrimAtPath('/World/G1/' + b_)
        assert pa_.IsValid() and pb_.IsValid(), 'unknown link in diagnostic_collision_filters: %s / %s' % (a_, b_)
        UsdPhysics.FilteredPairsAPI.Apply(pa_).CreateFilteredPairsRel().AddTarget(pb_.GetPath())
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
# Pure readback (sprint L, hand-an-04): the URDF velocity field of every hand joint (1.0 rad/s) is suspected to act as the PhysX
# joint velocity limit (driven joints' reported velocity saturates at exactly 1.000). Record what the stage and the view hold;
# never fail the run on it.
hand_velocity_limit_readback = {'usd_physxJoint_maxJointVelocity': {}, 'usd_unit': 'PhysX USD convention: degrees/s for revolute joints', 'urdf_velocity_field_rad_s': {n: facts['joint_limits'][n]['velocity'] for n in facts['joint_limits'] if n in hand_names or n in facts['mimic_map']}, 'articulation_view_max_joint_velocities': None, 'errors': []}
try:
    _hand_all = set(hand_names) | set(facts['mimic_map'])
    for _prim in world.stage.Traverse():
        if _prim.IsA(UsdPhysics.RevoluteJoint) and _prim.GetName() in _hand_all:
            _attr = _prim.GetAttribute('physxJoint:maxJointVelocity')
            hand_velocity_limit_readback['usd_physxJoint_maxJointVelocity'][_prim.GetName()] = (float(_attr.Get()) if _attr and _attr.HasValue() else None)
except Exception as _exc:   # noqa: BLE001 - readback only
    hand_velocity_limit_readback['errors'].append('usd: %r' % (_exc,))
try:
    _mv = robot._articulation_view.get_max_joint_velocities()
    _mv = np.ravel(np.asarray(_mv.cpu() if hasattr(_mv, 'cpu') else _mv, dtype=float))
    hand_velocity_limit_readback['articulation_view_max_joint_velocities'] = {n: float(_mv[names.index(n)]) for n in names if n in _hand_all} if _mv.size == len(names) else {'raw_size': int(_mv.size)}
except Exception as _exc:   # noqa: BLE001 - readback only (API may not exist in this Isaac version)
    hand_velocity_limit_readback['errors'].append('view: %r' % (_exc,))
(out / 'hand_velocity_limit_readback.json').write_text(json.dumps(hand_velocity_limit_readback, indent=2))

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
# Optional (config hand_init = 'first_row_if_nearly_open'): when every first-row hand target is within 0.15 rad of the
# URDF open pose (an OBSERVED nearly-open hand, no interpenetration possible), the actuated hand joints and their
# coupled joints start AT the first row instead of at the 0 rad hard stop; the sprint I model-action run 01 aborted
# in its lead-in on the chained-mimic thumb chattering at that stop with the arm at the observed pose.
hand_init = config.get('hand_init', 'urdf_open'); assert hand_init in ('urdf_open', 'first_row_if_nearly_open')
hand_init_applied = False
if hand_init == 'first_row_if_nearly_open' and all(abs(v) <= 0.15 for v in first_hand.values()):
    for n, v in first_hand.items():
        q0[names.index(n)] = v
    for side in ('left', 'right'):
        for child, spec in manifest.data['hands'][side]['coupled_joints'].items():   # parent-first order in the manifest
            q0[names.index(child)] = spec['multiplier'] * q0[names.index(spec['parent'])] + spec.get('offset', 0.0)
    hand_init_applied = True
robot.set_joint_positions(q0); robot.set_joint_velocities(np.zeros(53, dtype=np.float32))
views = {n: SimulationManager.get_physics_sim_view().create_rigid_body_view('/World/G1/' + n) for n in ['pelvis', 'torso_link', 'right_wrist_yaw_link', 'left_wrist_yaw_link', 'right_base_link', 'left_base_link']}
assert all(v.count == 1 for v in views.values())
object_view = SimulationManager.get_physics_sim_view().create_rigid_body_view('/World/Scene/Object') if scene else None
if object_view is not None:
    assert object_view.count == 1
object_file = (out / 'object.jsonl').open('w', buffering=1) if scene else None

# Cameras: two external views and the policy camera on the torso at the donor URDF d435 mount.
cameras = {}
closeup = config.get('closeup_camera', {'offset': [0.42, -0.30, 0.28]})   # right-hand close-up: diagnostic camera re-aimed at the measured right palm every frame (follows the hand); not a policy input, not a bilateral trial
for label, position, target in [('front', (2.4, -2.4, 1.7), (0, 0, 1.0)), ('side', (-.3, 3.2, 1.6), (0, 0, 1.0)), ('closeup_right_hand', (0.85, -0.85, 1.2), (0.2, -0.2, 0.92))]:
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
cl_start = lead_in_steps + (cl_handover_ticks if closed_loop else 0)
cl_targets = None   # ClosedLoopTargets, created at the first model row (sprint L C1)
if closed_loop:
    _limits = {a: manifest.hand_actuator('right', a)['closed_rad'] for a in HAND_ACTUATORS}
    _radian_contract = {'axis_order': list(PISTON_HAND_ORDER), 'open_value': 0.0, 'closed_value': 1.0, 'per_axis_endpoints': {a: {'open_value': 0.0, 'closed_value': _limits[a]} for a in HAND_ACTUATORS}, 'saturation_policy': 'clip_declared'}
    cl_adapters = {sd: HandCommandAdapter(manifest, sd, _radian_contract) for sd in ('left', 'right')}
    # closed loop applies the radian contract above (not the package's) and the declared arm datum: rebind and re-check
    live_dependencies = runtime_dependency_values(source, collision_cooking=COLLISION_COOKING, physics_dt_s=dt, support='FIXED_PELVIS', controller=CONTROLLER_DESCRIPTOR,
                                                  hand_contracts=cl_adapters, arm_datum='closed_loop:' + cl_datum_mode)
    qualification_validity = manifest.check_validity(live_dependencies)
    cl_hybrid = closed_loop.get('hybrid') if closed_loop['control_source'] == 'HYBRID_MODEL_ARMS_SCRIPTED_HANDS' else None
    if cl_hybrid:
        _closure_contract = {'axis_order': list(HAND_ACTUATORS), 'open_value': 0.0, 'closed_value': 1.0, 'saturation_policy': 'reject'}
        cl_hand_script = HandCommandAdapter(manifest, 'right', _closure_contract)
        cl_open_targets = np.array([first_hand[n] for n in hand_names], dtype=np.float32)   # the package's first-row hand targets (open margin)
        _closed_targets_dict, _ = cl_hand_script.to_joint_targets([float(v) for v in cl_hybrid['closure']])
        cl_closed_targets = cl_open_targets.copy()
        for n_, v_ in _closed_targets_dict.items():
            cl_closed_targets[hand_names.index(n_)] = v_
    def cl_sidecar_alive():
        return not (cl_ipc / 'EXIT').exists()
    def cl_observe(tick):
        world.render(); pixels = cameras['policy'].get_rgba(); extra = 0
        while (pixels is None or pixels.size == 0) and extra < 3:
            world.render(); extra += 1; pixels = cameras['policy'].get_rgba()
        import io
        buf = io.BytesIO(); Image.fromarray(pixels[..., :3].astype(np.uint8)).save(buf, format='PNG'); png = buf.getvalue()
        q_ = np.ravel(robot.get_joint_positions()); qd = dict(zip(names, q_.tolist()))
        if cl_state.get('datum') is None:
            cl_state['datum'] = {n: (float(body_target[body_names.index(n)]) if cl_datum_mode == 'handover_pose' else 0.0) for n in cl_arm_names}
        dz = cl_state['datum']
        hand_of = lambda side: [qd[manifest.hand_actuator(side, a)['joint']] for a in PISTON_HAND_ORDER]   # dataset order pinky..thumb_yaw, donor radians (identity, declared)
        state = {'left_arm': [qd[n] - dz[n] for n in cl_arm_names[:7]], 'right_arm': [qd[n] - dz[n] for n in cl_arm_names[7:]], 'left_hand': hand_of('left'), 'right_hand': hand_of('right'), 'waist': [qd['waist_yaw_joint'], qd['waist_roll_joint'], qd['waist_pitch_joint']]}
        k = cl_state['obs_count']; cl_state['obs_count'] += 1
        loop_ipc.write_observation(cl_ipc, k, cl_token, {'state': state, 'instruction': closed_loop['instruction'], 'physics_s': world.current_time, 'tick': tick, 'camera': 'policy_head_d435_color_nominal 640x480 RGB'}, image_bytes=png)
        (out / 'frames' / 'policy' / ('obs%06d.png' % k)).write_bytes(png)
        return k, hashlib.sha256(png).hexdigest(), state
    def cl_request_chunk(tick):
        k, png_sha, state = cl_observe(tick); t_wait = time.monotonic()
        act = loop_ipc.wait_for_action(cl_ipc, k, run_token=cl_token, dims=PISTON_DIMS, timeout_s=cl_timeout_first if k == 0 else cl_timeout, sidecar_alive=cl_sidecar_alive)
        msg = json.loads((cl_ipc / 'act' / ('%06d.json' % k)).read_text())
        raw_act = act; dz = cl_state['datum']
        if cl_datum_mode == 'handover_pose':
            act = dict(act); act['left_arm'] = [[v + dz[n] for v, n in zip(row, cl_arm_names[:7])] for row in act['left_arm']]; act['right_arm'] = [[v + dz[n] for v, n in zip(row, cl_arm_names[7:])] for row in act['right_arm']]
        rep = validate_piston_chunk(manifest, act, hand_adapters=cl_adapters, prefix_steps=cl_prefix)
        rec = {'iteration': cl_state['iteration'], 'obs_id': k, 'tick': tick, 'physics_s': world.current_time, 'image_sha256': png_sha, 'state_sent': state, 'inference_id': msg.get('inference_id'), 'latency_s': msg.get('latency_s'), 'wall_wait_s': time.monotonic() - t_wait,
               'raw_first_step': {kk: raw_act[kk][0] for kk in raw_act}, 'arm_state_datum': cl_datum_mode, 'datum_offset_rad': dz, 'adapter': {kk: rep[kk] for kk in ('executable', 'rejected', 'interventions', 'prefix_steps', 'horizon') if kk in rep}, 'preprocessing': msg.get('preprocessing')}
        if not rep['executable']:
            rec['refused'] = rep.get('refusal'); cl_state['records'].append(rec); cl_state['refusals'].append(rec['refused']); return None
        rec['decoded_rows'] = rep['rows']; rec['hand_outputs_used'] = 'MODEL' if not cl_hybrid else 'UNUSED (scripted hand owns the grasp); raw values logged'
        cl_state['records'].append(rec); return rep['rows']
body_target = np.array([initial_body[n] for n in body_names], dtype=np.float32)
hand_open = np.array([open_hand[n] for n in hand_names], dtype=np.float32); hand_first = np.array([first_hand[n] for n in hand_names], dtype=np.float32)
hand_target = hand_first.copy() if hand_init_applied else hand_open.copy()
ff_cap_count = 0; ff_cap_max = 0.0
loop_wall_start = time.monotonic()
for tick in range(steps):
    t_source = (tick - lead_in_steps) * dt + sequence.converted[0]['t_s']
    row = sequence.active_row(t_source) if tick >= lead_in_steps else None
    if closed_loop and tick >= cl_start:
        phase = 'replay'
        if (tick - cl_start) % cl_model_ticks == 0:
            if cl_state['chunk'] is None or cl_state['chunk_pos'] >= cl_prefix:
                if cl_state['iteration'] < cl_iters:
                    rows_ = cl_request_chunk(tick)
                    if rows_ is None:
                        aborted = {'sequence': tick, 'reason': 'model_chunk_refused', 'detail': cl_state['refusals'][-1]}; break
                    cl_state['chunk'] = rows_; cl_state['chunk_pos'] = 0; cl_state['iteration'] += 1
                else:
                    cl_state['chunk'] = None   # tail hold: keep the last targets
            if cl_state['chunk'] is not None:
                mrow = cl_state['chunk'][cl_state['chunk_pos']]; cl_state['chunk_pos'] += 1; cl_state['model_tick'] += 1
                if cl_targets is None:   # first model row: the tracker starts from the targets held at the hand-over
                    cl_targets = ClosedLoopTargets(arm_names=cl_arm_names, hand_names=hand_names,
                                                   initial_arm={n_: float(body_target[body_names.index(n_)]) for n_ in cl_arm_names},
                                                   initial_hand={n_: float(hand_target[hand_names.index(n_)]) for n_ in hand_names},
                                                   model_ticks=cl_model_ticks, max_step_rad=closed_loop.get('max_step_rad'),
                                                   interpolate=closed_loop.get('interpolate_within_step', True),
                                                   hand_owner='SCRIPTED' if cl_hybrid else 'MODEL',
                                                   hand_decoder=(None if cl_hybrid else (lambda sd_, vals_: cl_adapters[sd_].to_joint_targets(vals_))))   # (targets, info) -> clip axes recorded
                try:
                    chunk_rec = cl_targets.begin_row(tick=tick, physics_s=world.current_time, body_q_rad=mrow['body_q_rad'], hands=mrow.get('hands'),
                                                     iteration=cl_state['iteration'], chunk_pos=cl_state['chunk_pos'] - 1, obs_id=cl_state['obs_count'] - 1)
                except TargetError as exc_:
                    aborted = {'sequence': tick, 'reason': 'model_row_refused', 'detail': str(exc_)}; break
                chunk_rec['wall_s'] = time.monotonic() - loop_wall_start
                command_file.write(json.dumps(chunk_rec, allow_nan=False) + '\n')   # exactly once per model row, outside any joint loop
            elif cl_targets is not None:
                cl_targets.ramp_from = dict(cl_targets.ramp_to)   # tail hold: no new row
        if cl_targets is not None:
            applied_rec = cl_targets.advance(tick=tick, physics_s=world.current_time)   # arm interpolation on the declared axes only; hands untouched here
            for n_ in cl_arm_names:
                body_target[body_names.index(n_)] = applied_rec['arm_targets_rad'][n_]
            if not cl_hybrid:
                for n_ in hand_names:
                    hand_target[hand_names.index(n_)] = applied_rec['hand_targets_rad'][n_]
            cl_state['interventions'] = cl_targets.interventions
        if cl_hybrid:
            palm_now = np.asarray(views['right_base_link'].get_transforms())[0][:3]
            if cl_state['hand_phase'] == 'open' and np.linalg.norm(palm_now - np.asarray(cl_hybrid['close_trigger']['palm_target_world'])) <= cl_hybrid['close_trigger']['radius_m']:
                cl_state['hand_phase'] = 'closing'; cl_state['close_started_tick'] = tick
            if cl_state['hand_phase'] == 'closing':
                alpha = min(1.0, (tick - cl_state['close_started_tick'] + 1) * dt / float(cl_hybrid['close_duration_s']))
                hand_target = (1.0 - alpha) * cl_open_targets + alpha * cl_closed_targets
                if cl_targets is not None:
                    cl_targets.set_scripted_hand({n_: float(hand_target[hand_names.index(n_)]) for n_ in hand_names})
                if alpha >= 1.0:
                    cl_state['hand_phase'] = 'closed'
        if cl_targets is not None:   # the applied record carries what is actually commanded this tick (hybrid: the scripted hand targets)
            applied_rec['hand_targets_rad'] = {n_: float(hand_target[hand_names.index(n_)]) for n_ in hand_names}
            command_file.write(json.dumps(applied_rec, allow_nan=False) + '\n')   # exactly once per physics tick
        row = {'row': cl_state['model_tick'], 't_s': world.current_time, 'body_targets_rad': {}, 'hands': {}}
        applied_rows.add(0)
    if row is None and lead_in_steps > 0 and not hand_init_applied:
        alpha = min(1.0, (tick + 1) / lead_in_steps)
        hand_target = (1.0 - alpha) * hand_open + alpha * hand_first
    if row is not None and (not closed_loop or tick < cl_start):
        phase = 'scripted_prefix' if closed_loop else 'replay'
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
    ff_record = None
    if gff['enabled']:
        q_now = np.ravel(robot.get_joint_positions()); dq_now = np.ravel(robot.get_joint_velocities())
        g_all = np.ravel(robot._articulation_view.get_generalized_gravity_forces())   # model-based: effort needed to hold the current pose against gravity (PhysX, executed asset)
        ramp = 1.0 if gff_ramp_s <= 0 else min(1.0, (tick + 1) * dt / gff_ramp_s)
        lim = np.asarray([limits[n]['effort'] for n in body_names], dtype=np.float64)
        try:
            ff_applied, capped, pd_est, cap_mask = bounded_gravity_feedforward(kp[body_ids], kd[body_ids], body_target, q_now[body_ids], dq_now[body_ids], g_all[body_ids], lim, ramp=ramp, scale=gff_scale)
        except ValueError as exc:
            aborted = {'sequence': tick, 'reason': 'nonfinite_gravity_feedforward', 'detail': str(exc)}; break
        ff_capped_joints = [body_names[i] for i in np.where(cap_mask)[0]]; total = pd_est + g_all[body_ids] * gff_scale * ramp
        robot.set_joint_efforts(ff_applied.astype(np.float32), joint_indices=body_ids)
        ff_record = {'gravity_model_nm': g_all[body_ids].tolist(), 'ramp': ramp, 'pd_estimate_nm': pd_est.tolist(), 'feedforward_applied_nm': ff_applied.tolist(), 'estimated_total_nm': capped.tolist(), 'capped_joints': ff_capped_joints}
        ff_cap_count += len(ff_capped_joints); ff_cap_max = max(ff_cap_max, float(np.max(np.abs(total) - lim)))
    world.step(render=False)
    if (tick + 1) % frame_every == 0:
        world.render()
    q = np.ravel(robot.get_joint_positions()); dq = np.ravel(robot.get_joint_velocities()); effort = np.ravel(robot.get_measured_joint_efforts())
    if not (np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(effort).all()):
        aborted = {'sequence': tick, 'reason': 'nonfinite_physics_state'}; break
    poses = {n: np.asarray(v.get_transforms())[0].tolist() for n, v in views.items()}
    coupling = {n: float(q[names.index(n)] - (m['multiplier'] * q[names.index(m['parent'])] + m['offset'])) for n, m in facts['mimic_map'].items()}
    reported_over_2x = {n: float(dq[i]) for i, n in enumerate(names) if abs(dq[i]) > 2 * facts['joint_limits'][n]['velocity']}
    decision = None if operational_observer is None else operational_observer.update(tick, float(world.current_time), dict(zip(names, q.tolist())), dict(zip(names, dq.tolist())))
    r = {'sequence': tick, 'physics_s': world.current_time, 'wall_s': time.monotonic() - loop_wall_start, 'phase': phase, 'source_row': None if row is None else row['row'],
         'source_t_s': None if row is None else row['t_s'], 'runtime_names': names, 'q_rad': q.tolist(), 'dq_rad_s': dq.tolist(), 'measured_generalized_effort_nm': effort.tolist(),
         'body_command_names': body_names, 'body_command_rad': body_target.tolist(), 'hand_command_names': hand_names, 'hand_command_rad': hand_target.tolist(),
         'link_poses_world_xyzw': poses, 'coupling_error_rad': coupling, 'body_feedforward': ff_record}
    if decision is not None:
        r['legacy_envelope_shadow'] = {'reported_over_2x_rad_s': reported_over_2x, 'would_abort': bool(reported_over_2x)}
    trace.append(r); state_file.write(json.dumps(r, allow_nan=False) + '\n')
    if object_view is not None:
        op = np.asarray(object_view.get_transforms())[0].tolist(); ov = np.asarray(object_view.get_velocities())[0].tolist()
        object_file.write(json.dumps({'sequence': tick, 'physics_s': world.current_time, 'phase': phase, 'source_row': None if row is None else row['row'], 'pose_world_xyzw': op, 'linear_velocity_m_s': ov[:3], 'angular_velocity_rad_s': ov[3:],
                                      'right_palm_pose_world_xyzw': poses['right_base_link']}, allow_nan=False) + '\n')
    violated = {n: float(q[i]) for i, n in enumerate(names) if q[i] < facts['joint_limits'][n]['lower'] - .1 or q[i] > facts['joint_limits'][n]['upper'] + .1}
    if decision is None:
        overspeed = reported_over_2x                                                    # legacy rule, unchanged
        if violated or overspeed:
            aborted = {'sequence': tick, 'reason': 'source_envelope_abort', 'joint_limit_violations_rad': violated, 'joint_velocity_violations_rad_s': overspeed, 'observer': LEGACY}; break
    else:
        if reported_over_2x:
            legacy_shadow_summary['over_2x_samples'] += 1; legacy_shadow_summary['would_abort'] = True
            if legacy_shadow_summary['first'] is None:
                legacy_shadow_summary['first'] = {'sequence': tick, 'physics_s': float(world.current_time), 'joints': reported_over_2x}
        if violated or decision['abort']:
            aborted = {'sequence': tick, 'reason': 'source_envelope_abort' if violated else 'operational_observer_abort', 'joint_limit_violations_rad': violated, 'joint_velocity_violations_rad_s': decision['violations'],
                       'coupling_faults_rad': decision['coupling_faults'], 'observer': observer_name, 'observer_reason': decision['reason'], 'legacy_envelope_shadow': r['legacy_envelope_shadow']}; break
    if (tick + 1) % frame_every == 0:
        frame = (tick + 1) // frame_every - 1; files_ = {}
        palm = np.asarray(poses['right_base_link'][:3]); eye = palm + np.asarray(closeup['offset'])
        xc = UsdGeom.Xformable(cameras['closeup_right_hand'].prim); xc.ClearXformOpOrder()
        xc.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye.tolist()), Gf.Vec3d(*palm.tolist()), Gf.Vec3d(0, 0, 1)).GetInverse())
        world.render()   # re-render after re-aiming the diagnostic close-up (policy/external cameras unchanged)
        for label, camera in cameras.items():
            pixels = camera.get_rgba(); extra = 0
            while (pixels is None or pixels.size == 0) and extra < 3:
                world.render(); extra += 1; pixels = camera.get_rgba()
            assert pixels is not None and pixels.ndim == 3, label
            f = f'frames/{label}/{frame:06d}.png'; Image.fromarray(pixels[..., :3].astype(np.uint8) if label == 'policy' else pixels.astype(np.uint8)).save(out / f); files_[label] = f
        frame_file.write(json.dumps({'frame': frame, 'sequence': tick, 'physics_s': world.current_time, 'wall_s': time.monotonic() - loop_wall_start, 'phase': phase, 'source_row': None if row is None else row['row'],
                                     'captured_after_same_step_render': True, 'views': files_, 'policy_camera': 'RGB 640x480 nominal D435 mount (NOT the dataset stereo camera)',
                                     'closeup_camera': {'eye_world_m': eye.tolist(), 'target_world_m': palm.tolist(), 'follows': 'right_base_link (diagnostic view)'}}) + '\n')
loop_wall = time.monotonic() - loop_wall_start
if closed_loop:
    (out / 'closed_loop_iterations.json').write_text(json.dumps(cl_state['records'], indent=1, allow_nan=False))
    (out / 'closed_loop_interventions.json').write_text(json.dumps(cl_state['interventions'], indent=1, allow_nan=False))
    (cl_ipc / 'STOP').write_text('stop')
state_file.close(); contact_file.close(); frame_file.close(); command_file.close()
if object_file is not None:
    object_file.close()

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
                          'gravity_feedforward': ({**gff, 'combined_effort_cap_events': ff_cap_count, 'max_requested_over_limit_nm': ff_cap_max, 'accounting': 'implicit PD drive (declared gains, URDF max force) + applied joint effort = model gravity term at the current configuration, ramped over ramp_in_s, reduced so that estimated PD + feed-forward stays within the URDF effort limit; measured_generalized_effort_nm in the trace is the solver joint effort (not motor torque)'} if gff['enabled'] else {'enabled': False}),
                          'body_gains_sha256': canonical_sha256({'kp': kp_body, 'kd': kd_body, 'home': home})},
           'initialization': {'body': 'first row targets where named, else controller home (written once)', 'hands': ('first-row (observed, nearly open) pose written once with consistent coupled joints; targets held at the first row' if hand_init_applied else 'URDF open pose written once; targets ramped open -> first row over the lead-in'), 'hand_init': hand_init, 'lead_in_s': lead_in_s},
           'steps': len(trace), 'physics_dt': dt, 'lead_in_s': lead_in_s, 'simulated_s': len(trace) * dt, 'loop_wall_s': loop_wall,
           'real_time_factor_loop': (len(trace) * dt) / loop_wall if loop_wall > 0 else None, 'offline_replay': True, 'wall_since_probe_start_s': time.monotonic() - started_wall,
           'runtime_names': names, 'commanded_body_joints': commanded_body, 'commanded_hand_joints': commanded_hand, 'tracking_abs_error_rad': tracking,
           'observer': ({'active': LEGACY, 'note': 'historical rule: reported velocity > 2x own URDF field aborts'} if operational_observer is None else {'active': observer_name, **operational_observer.summary, 'legacy_envelope_shadow': legacy_shadow_summary, 'note': 'SIM-only guard (owner decision A, Sprint O): sampled-interval velocity, mimic children at |multiplier| x parent field, persist 2; legacy reported-velocity rule scored in shadow, never acting'}),
           'rows_applied': len(applied_rows), 'rows_total': len(sequence.converted), 'clipped_rows': sequence.clipped_rows, 'rejections': rejections, 'abort': aborted,
           'closed_loop': ({'schema': 'closed_loop_v1', 'control_source': closed_loop['control_source'], 'timing': 'non-real-time closed-loop simulation (physics paused while inferring)', 'iterations_completed': cl_state['iteration'], 'iterations_planned': cl_iters, 'target_component': {'module': 'isaac/twin/inspire/closed_loop_targets.py', 'sha256': hashlib.sha256(Path(sys.modules['inspire.closed_loop_targets'].__file__).read_bytes()).hexdigest(), 'hand_owner': ('SCRIPTED' if cl_hybrid else 'MODEL'), 'model_rows_applied': (cl_targets.rows if cl_targets else 0), 'applied_records': (cl_targets.applied_records if cl_targets else 0)}, 'arm_state_datum': cl_datum_mode, 'datum_offset_rad': cl_state.get('datum'), 'handover_after_s': closed_loop.get('handover_after_s', 0.0), 'prefix_steps': cl_prefix, 'model_step_s': cl_model_ticks * dt,
                            'observations_published': cl_state['obs_count'], 'distinct_image_hashes': len({r['image_sha256'] for r in cl_state['records']}), 'refusals': cl_state['refusals'], 'interventions': {'count': len(cl_state['interventions']), 'by_axis': {a_: sum(1 for i_ in cl_state['interventions'] if i_['axis'] == a_) for a_ in {i_['axis'] for i_ in cl_state['interventions']}}, 'max_step_rad': closed_loop.get('max_step_rad'), 'interpolate_within_step': closed_loop.get('interpolate_within_step', True)}, 'hand_phase_final': cl_state['hand_phase'], 'close_started_tick': cl_state['close_started_tick'],
                            'instruction': closed_loop['instruction'], 'sidecar_ready': (cl_ipc / 'READY').exists(), 'sidecar_exit': json.loads((cl_ipc / 'EXIT').read_text()) if (cl_ipc / 'EXIT').exists() else None} if closed_loop else None),
           'contact_points_during_replay': len(self_contacts), 'contact_pairs': sorted({tuple(sorted((c['actor0'], c['actor1']))) for c in self_contacts})[:40],
           'coupling_error_max_rad': max_coupling, 'hand_velocity_limit_readback': hand_velocity_limit_readback, 'fixed_base': True, 'support_constraints': ['pelvis_fixed_to_world_1m_above_origin'], 'ground_present': False, 'objects_present': False,
           'hardware_authorized': False, 'exact_asset_qualified': False, 'source_model': 'Unitree_FTP_G1_provisional_donor', 'manifest_id': manifest.data['manifest_id'], 'manifest_sha256': manifest.sha256,
           'grasp_qualification': 'NOT_RUN', 'writing_qualification': 'NOT_RUN', 'standing_qualification': 'NOT_RUN', 'real_data_agreement': 'NOT_TESTED_IN_PROBE (host-side comparison only)',
           'diagnostic_collision_filters': diagnostic_collision_filters, 'media_labels': {'fixture': 'FIXED PELVIS - %s%s%s' % (config.get('execution_label', 'COMMAND REPLAY'), (' [SELF-COLLISION PAIRS FILTERED: %s — NON-QUALIFYING]' % '; '.join('%s/%s' % tuple(x) for x in diagnostic_collision_filters)) if diagnostic_collision_filters else '', ': free rigid object on a table (no floor)' if scene else ' (no ground, no object)'), 'embodiment': 'PROVISIONAL G1 + bilateral FTP donor hands',
                            'source': '%s: %s' % (sequence.source['kind'], sequence.source['source_id']), 'qualification': 'Software/physics integration only; no grasp, writing, standing or real-data agreement qualification'},
           'source_property_audit': 'live_inertia_audit.json', 'scene': scene_facts, 'task_evaluation': 'host-side (tools/task_eval.py) against the frozen criteria; not computed in the probe'}
(out / 'metrics.json').write_text(json.dumps(metrics, indent=2, allow_nan=False))
world.stage.GetRootLayer().Export(str(out / 'assembled_scene.usda'))
artifacts = [str(p.relative_to(out)) for p in out.rglob('*') if p.is_file() and p.name not in ['run.json', 'probe.json', 'console.log', 'executed_probe.py', 'executed_launcher.py', 'uncommitted.patch']]
(out / 'probe.json').write_text(json.dumps({'status': metrics['status'], 'scope': metrics['scope'], 'metrics': 'metrics.json', 'artifacts': artifacts}))
app.close()
