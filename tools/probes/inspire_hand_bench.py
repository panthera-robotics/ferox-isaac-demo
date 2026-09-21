"""Six-axis fixed-base HAND BENCH probe, hand-agnostic (RH56E2 acceptance lane B, R1/R2).

Same physics, import settings, drives (kp 1 / kd 0.05, force 10), sweeps, coupled closure and blocked-index
obstruction test as tools/probes/inspire_hand.py (mode 'default' / 'blocked-index'), with the hand named by
environment instead of hard-wired donor names, so the SAME probe runs the donor (A) and the E2 prior (B):
  (probe-config.json {"bench": {...}} takes precedence over these environment keys; the hashed config is the provenance)
  PANTHERA_BENCH_URDF        bench URDF under /source-assets (default FTP_right_hand_bench.urdf)
  PANTHERA_BENCH_INDEX_JOINT index root joint for the blocked-index test (default right_index_1_joint)
  PANTHERA_BENCH_INDEX_TIP   distal index link whose measured closed pose places the obstacle (default right_index_force_sensor_3)
  PANTHERA_BENCH_INDEX_MATCH substring identifying index links in contact actors (default /right_index)
Additions over inspire_hand.py (all recorded, thresholds live in the acceptance protocol, not here): an R1 sequence after the
historical sweeps (full-range simultaneous close/open x3 with the 0.02 rad limit margin, 5 s closed hold, open), per-segment
endpoint tracking error, per-joint peak |dq|, startup/initial-phase contact count and the imported joint velocity-cap readback.
Modes: default | blocked-index | tgs-forces-blocked-index. No static palm fixture, no collision refinement (donor-mesh specific).
This is a fixed-base diagnostic, never a grasp, tactile or standing qualification.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

assert os.environ.get('PANTHERA_SIM_AUTHORIZED') == '1'
assert sorted(p.name for p in Path('/sys/class/net').iterdir()) == ['lo']
sys.path.insert(0, '/workspace/ferox_tools')
from inspire_asset import audit_urdf
# Bench naming: from the hashed /probe-config.json (--probe-config, keys bench_urdf / index_joint / index_tip / index_match)
# when present, else environment, else the donor defaults. The record keeps whichever was used.
_pc = {}
if os.environ.get('PANTHERA_PROBE_CONFIG') and Path(os.environ['PANTHERA_PROBE_CONFIG']).is_file():
    _pc = json.loads(Path(os.environ['PANTHERA_PROBE_CONFIG']).read_text()).get('bench', {})
BENCH_URDF = _pc.get('bench_urdf', os.environ.get('PANTHERA_BENCH_URDF', 'FTP_right_hand_bench.urdf'))
INDEX_JOINT = _pc.get('index_joint', os.environ.get('PANTHERA_BENCH_INDEX_JOINT', 'right_index_1_joint'))
INDEX_TIP = _pc.get('index_tip', os.environ.get('PANTHERA_BENCH_INDEX_TIP', 'right_index_force_sensor_3'))
INDEX_MATCH = _pc.get('index_match', os.environ.get('PANTHERA_BENCH_INDEX_MATCH', '/right_index'))
LIMIT_MARGIN = 0.02   # declared donor rule (sprint J): commanded targets stay 0.02 rad inside the URDF limits
source = Path('/source-assets') / BENCH_URDF
assert source.is_file(), f'bench URDF missing: {source}'
facts = audit_urdf(source)
root = ET.parse(source).getroot()
independent = [j.get('name') for j in root.findall('joint')
               if j.get('type') == 'revolute' and j.find('mimic') is None]
mimics = {j.get('name'): {'parent': j.find('mimic').get('joint'),
                        'multiplier': float(j.find('mimic').get('multiplier', 1)),
                        'offset': float(j.find('mimic').get('offset', 0))}
          for j in root.findall('joint') if j.find('mimic') is not None}
limits = {j.get('name'): [float(j.find('limit').get(k)) for k in ('lower', 'upper')]
          for j in root.findall('joint') if j.get('type') == 'revolute'}
assert len(independent) == 6 and len(mimics) == 6
out = Path('/evidence')
mode = os.environ.get('PANTHERA_PROBE_MODE', 'default')
assert mode in {'default', 'blocked-index', 'tgs-forces-blocked-index'}
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting'})
import numpy as np
import omni.kit.commands
from omni.physx import get_physx_simulation_interface, get_physx_cooking_interface
from pxr import Gf, PhysxSchema, PhysicsSchemaTools, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdUtils
from PIL import Image
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
enable_extension('isaacsim.asset.importer.urdf')
from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
ok, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
assert ok
cfg.distance_scale = 1.0
cfg.merge_fixed_joints = False
cfg.fix_base = True  # Declared bench fixture, not a balancing controller.
cfg.make_default_prim = True
cfg.create_physics_scene = False
cfg.import_inertia_tensor = True
cfg.convex_decomp = True
cfg.self_collision = True
cfg.parse_mimic = True
cfg.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_NONE
dest = out / 'hand_bench.usd'
ok, path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=str(source),
                                    import_config=cfg, dest_path=str(dest))
assert ok and dest.is_file(), 'URDF import failed'
stage = Usd.Stage.Open(str(dest))
usd_masses = {str(p.GetPath()): float(UsdPhysics.MassAPI(p).GetMassAttr().Get())
              for p in stage.Traverse() if p.HasAPI(UsdPhysics.MassAPI)
              and UsdPhysics.MassAPI(p).GetMassAttr().Get() is not None}
mass = sum(usd_masses.values())
want_mass = sum(float(m.get('value')) for m in root.iter('mass'))
assert abs(mass - want_mass) < .001, (mass, want_mass)
usd_joints = {p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)}
assert set(usd_joints) == set(limits), 'Imported joint set differs'
for name, prim in usd_joints.items():
    if name in mimics:
        assert prim.HasAPI(PhysxSchema.PhysxMimicJointAPI), f'Missing physical coupling {name}'
        # URDF gives an algebraic mimic, but importer 1.15+ defaults to a
        # compliant naturalFrequency=25 / dampingRatio=.005 constraint. PhysX
        # defines frequency <= 0 as hard coupling. This is a declared nominal
        # rigid-linkage surrogate; hardware compliance remains unqualified.
        axis = 'rot' + UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get()
        # These documented runtime attributes are not in the 5.1 Python schema
        # wrapper; use the raw attribute as NVIDIA's own mimic tests do.
        frequency = prim.CreateAttribute(f'physxMimicJoint:{axis}:naturalFrequency', Sdf.ValueTypeNames.Float)
        frequency.Set(0.)
        assert frequency.Get() == 0.
    # Only six independent drives. No strong child servos fighting constraints.
    if name in independent:
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
        drive.CreateTypeAttr('force')
        drive.CreateMaxForceAttr(10.)  # Donor effort limit, not measured E2 capability.
        drive.CreateStiffnessAttr(1.)
        drive.CreateDampingAttr(.05)
        drive.CreateTargetPositionAttr(0.)
collision_refinement = []
collision_api_relocations = []
clearance_control = False
static_palm_bench = False
diagnostic_filtered_pairs = []
stage.GetRootLayer().Save()
world = World(stage_units_in_meters=1., physics_dt=.005, rendering_dt=.02)
if mode.startswith('zero-gravity'):
    world.get_physics_context().set_gravity(0.)
if mode.startswith('tgs-forces-'):
    # TGS's once-per-frame gravity application can report nonzero joint velocity
    # at a steady position. Apply gravity proportionally at each internal step:
    # PhysX docs Simulation.html#tgs-steady-state-velocity-and-position-discrepancy.
    scenes = [p for p in world.stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    assert len(scenes) == 1
    flag = PhysxSchema.PhysxSceneAPI.Apply(scenes[0]).CreateEnableExternalForcesEveryIterationAttr(True)
    assert flag.Get() is True
UsdLux.DomeLight.Define(world.stage, '/World/Light').CreateIntensityAttr(1400.)
add_reference_to_stage(str(dest), '/World/Hand')
if 'velocity8-' in mode:
    # One-factor contact velocity convergence diagnostic; preserve position
    # iterations, geometry, time step, limits and drives.
    roots = [p for p in world.stage.Traverse() if p.HasAPI(PhysxSchema.PhysxArticulationAPI)]
    assert len(roots) == 1
    PhysxSchema.PhysxArticulationAPI(roots[0]).CreateSolverVelocityIterationCountAttr(8)
collision_fixture = None
# Retain PhysX's cooking representation at the authored zero pose. This is a
# cooking-service result, not a live actor-shape query. It allows collision
# approximation volume to be compared with source-mesh intersections.
cooked = []
for prim in world.stage.Traverse(Usd.TraverseInstanceProxies()):
    selected = str(prim.GetPath()).startswith('/World/Hand/') and '/collisions/' in str(prim.GetPath()) and any(
        k in str(prim.GetPath()).split('/collisions/')[0].rsplit('/', 1)[-1] for k in ('base', 'plam', 'palm', 'thumb'))
    if (not selected or not prim.HasAPI(UsdPhysics.CollisionAPI)
            or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False):
        continue
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    entry = {'prim': str(prim.GetPath()), 'prim_type': prim.GetTypeName(),
             'kind': 'pre_reset_cooking_representation_not_live_actor_shape_dump',
             'local_to_world_row_matrix_at_zero_pose': [[float(matrix[i][j]) for j in range(4)] for i in range(4)],
             'hulls': []}
    def receive_cooked(result, hulls, entry=entry):
        entry['result'] = str(result)
        for hull in hulls:
            entry['hulls'].append({'vertices': [[float(v.x), float(v.y), float(v.z)] for v in hull.vertices],
                'indices': list(map(int, hull.indices)),
                'polygons': [{'index_base': int(p.index_base), 'num_vertices': int(p.num_vertices),
                              'plane': list(map(float, p.plane))} for p in hull.polygons]})
    get_physx_cooking_interface().request_convex_collision_representation(
        stage_id=UsdUtils.StageCache.Get().GetId(world.stage).ToLongInt(),
        collision_prim_id=PhysicsSchemaTools.sdfPathToInt(prim.GetPath()),
        run_asynchronously=False, on_result=receive_cooked)
    cooked.append(entry)
out.joinpath('cooked_colliders.json').write_text(json.dumps(cooked))
contacts, trace, frames = [], [], []
contact_file = out.joinpath('contacts.jsonl').open('w', buffering=1)
trace_file = out.joinpath('state.jsonl').open('w', buffering=1)
phase = 'initialization'
# Configure reporting before PhysX creates the actors. In particular, reset can
# already step into an initial self-collision before the command loop begins.
for p in world.stage.Traverse():
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
def on_contact(headers, data):
    for h in headers:
        a = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
        b = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        for index in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
            d = data[index]
            contacts.append({'sequence': None if phase == 'initialization' else len(trace),
                             'physics_s': world.current_time, 'phase': phase,
                             'actor0': a, 'actor1': b, 'position_world_m': list(map(float, d.position)),
                             'normal_world': list(map(float, d.normal)),
                             'impulse_ns': list(map(float, d.impulse)), 'source': 'simulated_proxy'})
            contact_file.write(json.dumps(contacts[-1], allow_nan=False) + '\n')
subscription = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
hand = SingleArticulation('/World/Hand', name='hand_bench')
physics_configuration = {str(p.GetPath()): {a.GetName(): str(a.Get()) for a in p.GetAttributes()
    if a.GetName().startswith(('physics:', 'physx'))} for p in world.stage.Traverse()
    if p.IsA(UsdPhysics.Scene) or p.HasAPI(PhysxSchema.PhysxArticulationAPI)}
world.reset()
hand.initialize()
from isaacsim.core.simulation_manager import SimulationManager
from omni.physx import get_physxunittests_interface
tensor_articulation = hand._articulation_view._physics_view
backend_shapes = {'kind': 'live_physx_tensor_shape_properties_after_reset',
                  'physics_statistics': get_physxunittests_interface().get_physics_stats(), 'links': []}
for path in tensor_articulation.link_paths[0]:
    view = SimulationManager.get_physics_sim_view().create_rigid_body_view(path)
    assert view.count == 1
    backend_shapes['links'].append({'path': path, 'max_shapes': view.max_shapes,
        'contact_offsets_m': np.asarray(view.get_contact_offsets()).tolist() if view.max_shapes else [],
        'rest_offsets_m': np.asarray(view.get_rest_offsets()).tolist() if view.max_shapes else []})
out.joinpath('backend_shapes.json').write_text(json.dumps(backend_shapes, indent=2, allow_nan=False))
names = list(hand.dof_names)
velocity_cap_readback = {}
for name, prim in usd_joints.items():   # diagnostic readback of the imported PhysX joint velocity cap (USD deg/s) vs the URDF field; never fatal
    try:
        live = world.stage.GetPrimAtPath('/World/Hand/' + prim.GetPath().pathString.split('/', 2)[-1])
        attr = live.GetAttribute('physxJoint:maxJointVelocity') if live and live.IsValid() else None
        cap = float(attr.Get()) if attr and attr.HasAuthoredValue() else None
    except Exception as exc:  # noqa: BLE001
        cap = f'readback_error: {exc}'
    velocity_cap_readback[name] = {'usd_physxJoint_maxJointVelocity_deg_s': cap,
                                   'urdf_velocity_field_rad_s': float(next(j.find('limit').get('velocity') for j in root.findall('joint') if j.get('name') == name))}
assert len(names) == len(set(names)), 'Duplicate articulation coordinates'
assert set(names) == set(limits), f'Importer reduced coordinates unexpectedly: {names}'
ids = np.array([names.index(n) for n in independent], dtype=np.int32)
kp = np.zeros(len(names), dtype=np.float32)
kd = np.zeros(len(names), dtype=np.float32)
kp[ids], kd[ids] = 1., .05
hand._articulation_view.set_gains(kp, kd)
got_kp, got_kd = hand.get_articulation_controller().get_gains()
assert np.allclose(np.asarray(got_kp).ravel(), kp)
assert np.allclose(np.asarray(got_kd).ravel(), kd)
out.joinpath('initial_state.json').write_text(json.dumps({
    'names': names, 'q_rad': np.asarray(hand.get_joint_positions()).ravel().tolist(),
    'dq_rad_s': np.asarray(hand.get_joint_velocities()).ravel().tolist(),
    'kp_readback': np.asarray(got_kp).ravel().tolist(), 'kd_readback': np.asarray(got_kd).ravel().tolist(),
    'gravity': str(world.get_physics_context().get_gravity()), 'mode': mode}, indent=2))
camera = Camera('/World/Camera', resolution=(640, 640))
camera.initialize()
camera.set_clipping_range(.01, 10.)
cx = UsdGeom.Xformable(camera.prim)
cx.ClearXformOpOrder()
cx.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
    Gf.Vec3d(.38, .42, .31), Gf.Vec3d(0., 0., .12), Gf.Vec3d(0., 0., 1.)).GetInverse())
phase = 'initial'
def step(target, count):
    for _ in range(count):
        hand.apply_action(ArticulationAction(joint_positions=np.asarray(target, dtype=np.float32), joint_indices=ids))
        world.step(render=False)
        if len(trace) % 4 == 0:
            world.render()
        q = np.asarray(hand.get_joint_positions()).ravel()
        dq = np.asarray(hand.get_joint_velocities()).ravel()
        assert np.isfinite(q).all() and np.isfinite(dq).all()
        errors = {child: float(q[names.index(child)] - (m['multiplier'] * q[names.index(m['parent'])] + m['offset']))
                  for child, m in mimics.items()}
        trace.append({'sequence': len(trace), 'physics_s': world.current_time, 'phase': phase,
                      'command_names': independent, 'command_rad': list(map(float, target)),
                      'q_rad': q.tolist(), 'dq_rad_s': dq.tolist(), 'coupling_error_rad': errors,
                      'effective_q_target': np.asarray(hand.get_applied_action().joint_positions).ravel().tolist(),
                      'commanded_feedforward_effort_nm': np.asarray(hand.get_applied_joint_efforts()).ravel().tolist(),
                      'measured_joint_effort_nm': np.asarray(hand.get_measured_joint_efforts()).ravel().tolist()})
        trace_file.write(json.dumps(trace[-1], allow_nan=False) + '\n')
        if len(trace) % 20 == 0:
            pixels = camera.get_rgba()
            if pixels is not None and pixels.size:
                frames.append(Image.fromarray(pixels.astype(np.uint8)))

zero = [limits[n][0] for n in independent]
step(zero, 40)
for axis, name in enumerate(independent):
    phase = 'sweep_' + name
    for tick in range(100):
        target = zero.copy()
        fraction = .4 * (1. - np.cos(2. * np.pi * tick / 100)) / 2.
        target[axis] += fraction * (limits[name][1] - limits[name][0])
        step(target, 1)
    phase = 'reset_' + name
    step(zero, 40)
phase = 'coupled_closure'
for tick in range(100):
    target = [limits[n][0] + .35 * tick / 99 * (limits[n][1] - limits[n][0]) for n in independent]
    step(target, 1)
step(target, 60)
# ---- R1 sequence (acceptance lane B): full-range simultaneous close/open x3 inside the declared limit margin, 5 s closed hold, open
segments = []
def run_segment(label, target_list, count):
    global phase
    phase = label
    n0 = len(trace)
    step(target_list, count)
    seg = trace[n0:]
    q_end = np.asarray(seg[-1]['q_rad']); dq_peak = float(np.max(np.abs([r['dq_rad_s'] for r in seg])))
    err_end = {n: float(q_end[names.index(n)] - target_list[independent.index(n)]) for n in independent}
    segments.append({'label': label, 'steps': count, 'endpoint_error_rad': err_end, 'endpoint_error_max_abs_rad': float(max(abs(v) for v in err_end.values())),
                     'dq_peak_abs_rad_s': dq_peak, 'coupling_error_max_rad': float(max(abs(e) for r in seg for e in r['coupling_error_rad'].values())),
                     'contact_points': sum(1 for c in contacts if c['phase'] == label and c['sequence'] is not None)})
closed = [limits[n][1] - LIMIT_MARGIN for n in independent]
opened = [limits[n][0] + LIMIT_MARGIN for n in independent]
run_segment('r1_open', opened, 100)
for k in range(3):
    run_segment(f'r1_close_{k}', closed, 300)
    run_segment(f'r1_open_{k}', opened, 300)
run_segment('r1_close_hold', closed, 1000)   # 5 s at physics_dt 0.005
run_segment('r1_open_end', opened, 300)
R1_STEPS = 100 + 3 * 600 + 1000 + 300
blocked_index = None
if 'blocked-index' in mode:
    index_name = INDEX_JOINT
    assert index_name in independent, (index_name, independent)
    axis = independent.index(index_name)
    goal = zero.copy()
    goal[axis] += .4 * (limits[index_name][1] - limits[index_name][0])
    def index_trajectory(label):
        global phase
        phase = label + '_ramp'
        for tick in range(160):
            qref = zero.copy()
            qref[axis] += (goal[axis] - zero[axis]) * tick / 159
            step(qref, 1)
        phase = label + '_hold'
        step(goal, 120)
    phase = 'before_free_index'
    step(zero, 240)
    free_start = {'q': np.asarray(hand.get_joint_positions()).tolist(), 'dq': np.asarray(hand.get_joint_velocities()).tolist()}
    index_trajectory('free_index')
    index_id = names.index(index_name)
    free_window = trace[-60:]
    free_q = float(np.mean([r['q_rad'][index_id] for r in free_window]))
    tip = SimulationManager.get_physics_sim_view().create_rigid_body_view('/World/Hand/' + INDEX_TIP)
    assert tip.count == 1
    # Place the later obstacle using the measured closed finger pose, then open
    # before spawning it. This prevents an initially intersecting fixture from
    # masquerading as resistance to the commanded closing trajectory.
    measured_tip_pose = np.asarray(tip.get_transforms())[0].copy()
    obstacle_position = measured_tip_pose[:3].copy()
    phase = 'before_blocked_index'
    step(zero, 240)
    material = PhysicsMaterial('/World/IndexBlockMaterial', static_friction=.5,
                               dynamic_friction=.5, restitution=0.)
    block = world.scene.add(FixedCuboid('/World/IndexBlock', name='index_block', position=obstacle_position,
        scale=np.asarray([.016, .016, .016]), color=np.asarray([.9, .2, .1]), physics_material=material))
    # FixedCuboid defaults (100 mm contact margin, metre-scale torsional patch)
    # are inappropriate for a 16 mm obstacle. Set and retain explicit readback.
    block.set_contact_offset(.0005)
    block.set_rest_offset(0.)
    block.set_torsional_patch_radius(0.)
    block.set_min_torsional_patch_radius(0.)
    obstacle_settings = {'contact_offset_m': float(block.get_contact_offset()),
        'rest_offset_m': float(block.get_rest_offset()), 'torsional_patch_radius_m': float(block.get_torsional_patch_radius()),
        'min_torsional_patch_radius_m': float(block.get_min_torsional_patch_radius()),
        'static_friction': float(material.get_static_friction()), 'dynamic_friction': float(material.get_dynamic_friction()),
        'restitution': float(material.get_restitution()), 'provenance': 'declared_diagnostic_not_measured_hardware',
        'readback_kind': 'authored_USD_properties_not_live_tensor_shape_properties'}
    phase = 'block_spawn_settle'
    step(zero, 20)
    blocked_start = {'q': np.asarray(hand.get_joint_positions()).tolist(), 'dq': np.asarray(hand.get_joint_velocities()).tolist()}
    index_trajectory('blocked_index')
    blocked_window = trace[-60:]
    blocked_q = float(np.mean([r['q_rad'][index_id] for r in blocked_window]))
    block_contacts = [r for r in contacts if '/World/IndexBlock' in (r['actor0'], r['actor1'])]
    active_contacts = [r for r in block_contacts if r['phase'].startswith('blocked_index')]
    nonzero = [r for r in active_contacts if np.linalg.norm(r['impulse_ns']) > 1e-10]
    hold_contact_sequences = {r['sequence'] for r in nonzero if r['phase'] == 'blocked_index_hold'}
    unexpected = [r for r in nonzero if not any(INDEX_MATCH in r[k] for k in ['actor0', 'actor1'])]
    free_targets = [r['command_rad'] for r in trace if r['phase'].startswith('free_index')]
    blocked_targets = [r['command_rad'] for r in trace if r['phase'].startswith('blocked_index')]
    blocked_checks = {'same_command_trajectory': free_targets == blocked_targets,
        'matching_open_start': bool(np.allclose(free_start['q'], blocked_start['q'], atol=.005, rtol=0)
                                    and np.allclose(free_start['dq'], blocked_start['dq'], atol=.02, rtol=0)),
        'settled_final_windows': all(np.ptp([r['q_rad'][index_id] for r in window]) < .005
                                    and abs(np.mean([r['dq_rad_s'][index_id] for r in window])) < .02
                                    for window in [free_window, blocked_window]),
        'no_initial_block_contact': not any(r['phase'] == 'block_spawn_settle' for r in block_contacts),
        'measured_index_resistance': free_q - blocked_q > .03,
        'sustained_measured_block_impulse': len(hold_contact_sequences) >= 20,
        'no_other_finger_block_contact': not unexpected}
    blocked_index = {'scope': 'fixed_palm_bench_external_obstruction_only', 'checks': blocked_checks,
        'free_index_rad': free_q, 'blocked_index_rad': blocked_q,
        'position_difference_rad': free_q - blocked_q, 'block_center_world_m': obstacle_position.tolist(),
        'placement_source_body': '/World/Hand/' + INDEX_TIP, 'index_joint': index_name, 'index_match': INDEX_MATCH,
        'measured_body_pose_xyzw': measured_tip_pose.tolist(), 'free_start': free_start, 'blocked_start': blocked_start,
        'final_window_samples': 60, 'hold_contact_sequences': len(hold_contact_sequences),
        'block_size_m': [.016, .016, .016], 'block_physics_settings': obstacle_settings,
        'block_contacts': len(block_contacts),
        'nonzero_block_contact_points': len(nonzero),
        'contact_impulse_norm_sum_ns': float(sum(np.linalg.norm(r['impulse_ns']) for r in active_contacts)),
        'force_source': 'PhysX_contact_impulses_simulated_proxy_not_hardware_tactile'}
    out.joinpath('blocked_index.json').write_text(json.dumps(blocked_index, indent=2))
    world.stage.GetRootLayer().Export(str(out / 'blocked_bench_scene.usda'))
    saved_scene = Usd.Stage.Open(str(out / 'blocked_bench_scene.usda'))
    saved_scene.GetPrimAtPath('/World/Hand').GetReferences().ClearReferences()
    saved_scene.GetPrimAtPath('/World/Hand').GetReferences().AddReference('hand_bench.usd')
    saved_scene.GetRootLayer().Save()
Image.fromarray(camera.get_rgba().astype(np.uint8)).save(out / 'hand.png')
if frames:
    frames[0].save(out / 'hand_sweeps.gif', save_all=True, append_images=frames[1:], duration=100, loop=0)
trace_file.close()
contact_file.close()
coupling_max = max(abs(e) for r in trace for e in r['coupling_error_rad'].values())
excursions = {n: float(np.ptp([r['q_rad'][names.index(n)] for r in trace if r['phase'] == 'sweep_' + n])) for n in independent}
step_dts = np.diff([r['physics_s'] for r in trace])
joint_limit_violation = max(max(lo - r['q_rad'][names.index(n)], r['q_rad'][names.index(n)] - hi, 0.)
                            for n, (lo, hi) in limits.items() for r in trace)
checks = {'all_axes_move': all(excursions[n] >= .1 * (limits[n][1] - limits[n][0]) for n in independent),
          'coupling': coupling_max < .03, 'joint_limits': joint_limit_violation < .03,
          'complete_steps': len(trace) == (1040 + R1_STEPS + 1060 if blocked_index is not None else 1040 + R1_STEPS),
          'physics_dt': bool(np.allclose(step_dts, .005, atol=1e-8))}
if blocked_index is not None:
    checks.update({'blocked_' + k: v for k, v in blocked_index['checks'].items()})
startup_contacts = [c for c in contacts if c['phase'] in ('initialization', 'initial')]
limit_events = {n: int(sum(1 for r in trace if r['q_rad'][names.index(n)] < lo - LIMIT_MARGIN or r['q_rad'][names.index(n)] > hi + LIMIT_MARGIN)) for n, (lo, hi) in limits.items()}
dq_peak_per_joint = {n: float(max(abs(r['dq_rad_s'][names.index(n)]) for r in trace)) for n in names}
metrics = {'probe': 'inspire_hand_bench (hand-agnostic variant of inspire_hand.py; acceptance lane B)', 'bench_urdf': BENCH_URDF, 'probe_config_bench_block': _pc,
           'bench_names': {'index_joint': INDEX_JOINT, 'index_tip': INDEX_TIP, 'index_match': INDEX_MATCH}, 'limit_margin_rad': LIMIT_MARGIN,
           'r1_segments': segments, 'startup_contact_points': len(startup_contacts),
           'startup_contact_pairs': sorted({(c['actor0'], c['actor1']) for c in startup_contacts}, key=str)[:50],
           'limit_events_beyond_margin': limit_events, 'dq_peak_abs_rad_s_per_joint': dq_peak_per_joint,
           'velocity_cap_readback': velocity_cap_readback,
           'source_model': 'declared by the mounted source asset (donor or E2 public prior); exact installed-hand equivalence unverified',
           'target_model': 'RH56E2-2R-T1', 'exact_asset_qualified': False,
           'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
           'source_mass_kg': want_mass, 'imported_mass_kg': mass,
           'manufacturer_nominal_E2_T1_hand_kg': .79, 'mass_rescaled': False,
           'fixed_base': True, 'tool_attached': False, 'controller': 'six_independent_position_drives',
           'diagnostic_mode': mode, 'gravity': str(world.get_physics_context().get_gravity()),
           'collision_refinement': collision_refinement,
           'collision_api_relocations': collision_api_relocations,
           'diagnostic_filtered_pairs': diagnostic_filtered_pairs,
           'collision_qualification_excluded': clearance_control or static_palm_bench,
           'collision_fixture': collision_fixture,
           'physics_configuration': physics_configuration, 'randomized': False,
           'gains': {'kp': 1., 'kd': .05, 'provenance': 'declared_diagnostic_not_hardware_calibrated'},
           'runtime_names': names, 'independent_names': independent, 'mimic_map': mimics,
           'coupling_model': 'nominal_algebraic_URDF_mimic_not_measured_hardware_compliance',
           'mimic_natural_frequency_attribute': 0., 'importer_default_natural_frequency_attribute': 25.,
           'steps': len(trace), 'physics_dt': .005, 'coupling_error_max_rad': coupling_max,
           'axis_excursion_rad': excursions, 'joint_limit_violation_rad': joint_limit_violation,
           'checks': checks, 'measured_physics_dt_min': float(step_dts.min()), 'measured_physics_dt_max': float(step_dts.max()),
           'actual_contact_points': len(contacts),
           'blocked_finger_test': ('PASS' if all(checks.values()) else 'FAIL') if blocked_index is not None else 'NOT_RUN',
           'contact_reporting_configured_before_physics': True,
           'initialization_contact_points': sum(r['phase'] == 'initialization' for r in contacts),
           'grasp_qualification': 'NOT_RUN', 'writing_qualification': 'NOT_RUN'}
out.joinpath('metrics.json').write_text(json.dumps(metrics, indent=2))
artifacts = ['metrics.json', 'initial_state.json', 'state.jsonl', 'hand.png', 'hand_sweeps.gif', 'hand_bench.usd', 'cooked_colliders.json', 'backend_shapes.json']
artifacts += [str(p.relative_to(out)) for p in sorted(out.joinpath('configuration').rglob('*.usd'))]
if contacts:
    artifacts.append('contacts.jsonl')
if static_palm_bench:
    artifacts.append('bench_scene.usda')
if blocked_index is not None:
    artifacts += ['blocked_index.json', 'blocked_bench_scene.usda']
out.joinpath('probe.json').write_text(json.dumps({'status': 'PASS' if all(checks.values()) else 'FAIL',
                                                'scope': ('fixed_base_bench_unloaded_sweeps_plus_blocked_finger_obstruction' if blocked_index is not None
                                                          else 'fixed_base_bench_unloaded_sweeps_only'),
                                                'metrics': 'metrics.json',
                                                'artifacts': artifacts}))
app.close()
