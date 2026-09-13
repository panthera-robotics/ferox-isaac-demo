"""Evaluate geometry-derived moving-palm colliders on a provisional donor.

This is a fixed-base donor diagnostic, never an exact RH56E2, grasp, tactile,
or standing qualification. Six root drives actuate twelve coupled coordinates.
All commands and measured contact impulses are saved, including failed checks.
This version first compares the candidate with the fixed-wrist hand18 control.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def hand_probe_configuration(config, mode):
    """Resolve explicit source chirality before any source audit or simulator import."""
    if not isinstance(config, dict) or set(config) - {'hand_side', 'palm_candidate_id', 'solver_velocity_iterations'}:
        raise ValueError('Unrecognized hand probe configuration')
    side = config.get('hand_side', 'right')
    if type(side) is not str or side not in {'left', 'right'}:
        raise ValueError('hand_side must be left or right')
    if mode not in {'component-palm-sweeps', 'component-palm-blocked', 'component-palm-wrist'}:
        raise ValueError('Unknown hand probe mode')
    if side == 'left' and mode == 'component-palm-wrist':
        raise ValueError('The virtual wrist fixture currently has a right-only source contract')
    allowed = {'ftp_palm_components_v1', 'ftp_palm_yz_slabs_v2'} if side == 'right' else {'ftp_left_palm_yz_slabs_v1'}
    candidate = config.get('palm_candidate_id', 'ftp_palm_components_v1' if side == 'right' else 'ftp_left_palm_yz_slabs_v1')
    if type(candidate) is not str or candidate not in allowed:
        raise ValueError('Palm candidate does not match the explicit source side')
    iterations = config.get('solver_velocity_iterations', 8)
    if type(iterations) is not int or iterations not in {8, 16, 32}:
        raise ValueError('Unsupported diagnostic velocity iteration count')
    return {'hand_side': side, 'palm_candidate_id': candidate, 'solver_velocity_iterations': iterations,
        'source_filename': f'FTP_{side}_hand_bench.urdf',
        'imported_root': '/Rhand' if side == 'right' else '/Lhand',
        'usd_filename': f'ftp_{side}_bench.usd'}


assert os.environ.get('PANTHERA_SIM_AUTHORIZED') == '1'
assert sorted(p.name for p in Path('/sys/class/net').iterdir()) == ['lo']
sys.path.insert(0, '/workspace/ferox_tools')
from inspire_asset import audit_urdf
mode = os.environ.get('PANTHERA_PROBE_MODE', 'default')
probe_config = json.loads(Path(os.environ['PANTHERA_PROBE_CONFIG']).read_text()) if os.environ.get('PANTHERA_PROBE_CONFIG') else {}
selection = hand_probe_configuration(probe_config, mode)
hand_side = selection['hand_side']
hand_prefix = hand_side + '_'
imported_root = selection['imported_root']
base_link = hand_prefix + 'base_link'
palm_candidate_id = selection['palm_candidate_id']
solver_velocity_iterations = selection['solver_velocity_iterations']
source = Path('/source-assets') / selection['source_filename']
facts = audit_urdf(source)
root = ET.parse(source).getroot()
assert root.get('name') == imported_root.lstrip('/'), 'Source root differs from declared hand side'
independent = [j.get('name') for j in root.findall('joint')
               if j.get('type') == 'revolute' and j.find('mimic') is None]
mimics = {j.get('name'): {'parent': j.find('mimic').get('joint'),
                        'multiplier': float(j.find('mimic').get('multiplier', 1)),
                        'offset': float(j.find('mimic').get('offset', 0))}
          for j in root.findall('joint') if j.find('mimic') is not None}
limits = {j.get('name'): [float(j.find('limit').get(k)) for k in ('lower', 'upper')]
          for j in root.findall('joint') if j.get('type') == 'revolute'}
assert len(independent) == 6 and len(mimics) == 6
assert all(n.startswith(hand_prefix) for n in limits), 'Source joint names differ from declared hand side'
out = Path('/evidence')
wrist_fixture = None
import_source = source
if mode == 'component-palm-wrist':
    from inspire_wrist_fixture import write_wrist_fixture
    import_source = out / 'supported_wrist.urdf'
    wrist_fixture = write_wrist_fixture(source, import_source)
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
dest = out / selection['usd_filename']
ok, path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=str(import_source),
                                    import_config=cfg, dest_path=str(dest))
assert ok and dest.is_file(), 'URDF import failed'
stage = Usd.Stage.Open(str(dest))
usd_masses = {str(p.GetPath()): float(UsdPhysics.MassAPI(p).GetMassAttr().Get())
              for p in stage.Traverse() if p.HasAPI(UsdPhysics.MassAPI)
              and UsdPhysics.MassAPI(p).GetMassAttr().Get() is not None}
mass = sum(usd_masses.values())
want_mass = sum(float(m.get('value')) for m in root.iter('mass'))
fixture_mass = wrist_fixture['fixture_added_mass_kg'] if wrist_fixture else 0.
assert abs(mass - want_mass - fixture_mass) < .001, (mass, want_mass, fixture_mass)
usd_joints = {p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)}
fixture_axes = wrist_fixture['axes'] if wrist_fixture else []
assert set(usd_joints) == set(limits) | {a['name'] for a in fixture_axes if a['type'] == 'revolute'}, 'Imported joint set differs'
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
for axis in fixture_axes:
    prims = [p for p in stage.Traverse() if p.GetName() == axis['name'] and p.IsA(UsdPhysics.Joint)]
    assert len(prims) == 1
    drive = UsdPhysics.DriveAPI.Apply(prims[0], 'linear' if axis['type'] == 'prismatic' else 'angular')
    drive.CreateTypeAttr('force')
    drive.CreateMaxForceAttr(axis['effort_limit'])
    drive.CreateStiffnessAttr(axis['kp'])
    drive.CreateDampingAttr(axis['kd'])
    drive.CreateTargetPositionAttr(0.)
collision_refinement = []
collision_api_relocations = []
clearance_control = False
static_palm_bench = False
instance_roots = set()
for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
    if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsInstanceProxy():
        ancestor = prim.GetParent()
        while ancestor and not ancestor.IsInstance():
            ancestor = ancestor.GetParent()
        assert ancestor, f'No editable instance root for {prim.GetPath()}'
        instance_roots.add(str(ancestor.GetPath()))
for path in sorted(instance_roots):
    stage.GetPrimAtPath(path).SetInstanceable(False)
wrappers = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)
            and not p.IsA(UsdGeom.Mesh)]
assert len(wrappers) == 30, 'Pinned donor collision wrapper topology changed'
for wrapper in wrappers:
    meshes = [p for p in Usd.PrimRange(wrapper) if p.IsA(UsdGeom.Mesh)]
    assert len(meshes) == 1, f'Ambiguous collision geometry at {wrapper.GetPath()}'
    mesh = meshes[0]
    enabled = UsdPhysics.CollisionAPI(wrapper).GetCollisionEnabledAttr().Get()
    approximation = UsdPhysics.MeshCollisionAPI(wrapper).GetApproximationAttr().Get()
    assert approximation == 'convexDecomposition'
    UsdPhysics.CollisionAPI.Apply(mesh).CreateCollisionEnabledAttr(enabled)
    UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr(approximation)
    wrapper.RemoveAPI(UsdPhysics.CollisionAPI)
    wrapper.RemoveAPI(UsdPhysics.MeshCollisionAPI)
    collision_api_relocations.append({'from': str(wrapper.GetPath()), 'to': str(mesh.GetPath()),
        'collision_enabled': enabled, 'approximation': approximation,
        'triangles_transforms_mass_and_collision_pairs_changed': False})
assert len([p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI) and p.IsA(UsdGeom.Mesh)]) == 30
from inspire_collision import replace_palm_with_components
if palm_candidate_id in {'ftp_palm_yz_slabs_v2', 'ftp_left_palm_yz_slabs_v1'}:
    enable_extension('omni.pip.compute')
candidate = replace_palm_with_components(stage,
    f'{imported_root}/{base_link}/collisions/{base_link}/node_STL_BINARY_/mesh',
    f'{imported_root}/{base_link}', contact_offset_m=.0012860533315688372, rest_offset_m=0.,
    candidate_id=palm_candidate_id)
out.joinpath('collision_candidate.json').write_text(json.dumps(candidate, indent=2, allow_nan=False))
diagnostic_filtered_pairs = []
stage.GetRootLayer().Save()
world = World(stage_units_in_meters=1., physics_dt=.005, rendering_dt=.02)
if mode.startswith('zero-gravity'):
    world.get_physics_context().set_gravity(0.)
# TGS's once-per-frame gravity application can report nonzero joint velocity
# at a steady position. Apply gravity proportionally at each internal step:
# PhysX docs Simulation.html#tgs-steady-state-velocity-and-position-discrepancy.
scenes = [p for p in world.stage.Traverse() if p.IsA(UsdPhysics.Scene)]
assert len(scenes) == 1
flag = PhysxSchema.PhysxSceneAPI.Apply(scenes[0]).CreateEnableExternalForcesEveryIterationAttr(True)
assert flag.Get() is True
light = UsdLux.DomeLight.Define(world.stage, '/World/Light')
light.CreateIntensityAttr(350.)
light.CreateColorAttr(Gf.Vec3f(.45, .52, .65))
key_light = UsdLux.DistantLight.Define(world.stage, '/World/KeyLight')
key_light.CreateIntensityAttr(450.)
key_light.CreateColorAttr(Gf.Vec3f(1., .95, .88))
UsdGeom.Xformable(key_light).AddRotateXYZOp().Set(Gf.Vec3f(-35., -25., -40.))
add_reference_to_stage(str(dest), '/World/Hand')
# One-factor contact velocity convergence diagnostic; preserve position
# iterations, geometry, time step, limits and drives.
roots = [p for p in world.stage.Traverse() if p.HasAPI(PhysxSchema.PhysxArticulationAPI)]
assert len(roots) == 1
PhysxSchema.PhysxArticulationAPI(roots[0]).CreateSolverVelocityIterationCountAttr(solver_velocity_iterations)
collision_fixture = None
# Retain PhysX's cooking representation at the authored zero pose. This is a
# cooking-service result, not a live actor-shape query. It allows collision
# approximation volume to be compared with source-mesh intersections.
cooked = []
for prim in world.stage.Traverse(Usd.TraverseInstanceProxies()):
    selected = any(str(prim.GetPath()).startswith('/World/Hand/' + link + '/collisions/')
                   for link in [hand_prefix + name for name in ['base_link', 'thumb_1', 'thumb_2', 'thumb_force_sensor_1']])
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
                             'impulse_ns': list(map(float, d.impulse)),
                             'separation_m': float(d.separation), 'source': 'simulated_proxy'})
            contact_file.write(json.dumps(contacts[-1], allow_nan=False) + '\n')
subscription = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
hand = SingleArticulation('/World/Hand', name=f'ftp_{hand_side}_hand')
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
assert backend_shapes['physics_statistics']['numTriMeshShapes'] == 0, 'Candidate requires moving colliders, no static palm'
palm_cooked = [r for r in cooked if r['prim'].startswith(f'/World/Hand/{base_link}/collisions/')]
assert len(palm_cooked) == candidate['expected_authored_palm_collider_count'], 'Every candidate collider must have its own cooking result'
assert {r['prim'] for r in palm_cooked} == {r['prim'].replace(imported_root + '/', '/World/Hand/', 1) for r in candidate['components']}, 'Cooked collider identity set differs from the candidate'
assert all(r.get('result', '').endswith('RESULT_VALID') and r['hulls'] for r in palm_cooked)
assert not world.stage.GetPrimAtPath(f'/World/Hand/{base_link}/collisions/{base_link}/node_STL_BINARY_/mesh').HasAPI(UsdPhysics.CollisionAPI)
palm_live_count = next(x['max_shapes'] for x in backend_shapes['links'] if x['path'].endswith('/' + base_link))
assert palm_live_count == sum(len(r['hulls']) for r in palm_cooked), 'Runtime palm shapes differ from component cooking'
candidate['runtime_verification'] = {'component_cooking_results': len(palm_cooked),
    'all_cooking_results_valid': True, 'live_palm_shapes': palm_live_count,
    'whole_palm_collider_present': False, 'static_triangle_shapes': 0}
out.joinpath('collision_candidate.json').write_text(json.dumps(candidate, indent=2, allow_nan=False))
runtime_names = list(hand.dof_names)
assert len(runtime_names) == len(set(runtime_names)), 'Duplicate articulation coordinates'
assert set(runtime_names) == set(limits) | {a['name'] for a in fixture_axes}, f'Importer reduced coordinates unexpectedly: {runtime_names}'
names = [n for n in runtime_names if n in limits]
hand_ids = np.array([runtime_names.index(n) for n in names], dtype=np.int32)
ids = np.array([runtime_names.index(n) for n in independent], dtype=np.int32)
fixture_ids = np.array([runtime_names.index(a['name']) for a in fixture_axes], dtype=np.int32)
kp = np.zeros(len(runtime_names), dtype=np.float32)
kd = np.zeros(len(runtime_names), dtype=np.float32)
kp[ids], kd[ids] = 1., .05
for axis, index in zip(fixture_axes, fixture_ids):
    kp[index], kd[index] = axis['kp'], axis['kd']
hand._articulation_view.set_gains(kp, kd)
got_kp, got_kd = hand.get_articulation_controller().get_gains()
assert np.allclose(np.asarray(got_kp).ravel(), kp)
assert np.allclose(np.asarray(got_kd).ravel(), kd)
out.joinpath('initial_state.json').write_text(json.dumps({
    'names': names, 'q_rad': np.asarray(hand.get_joint_positions()).ravel()[hand_ids].tolist(),
    'dq_rad_s': np.asarray(hand.get_joint_velocities()).ravel()[hand_ids].tolist(),
    'all_runtime_names': runtime_names, 'fixture_axes': fixture_axes,
    'kp_readback': np.asarray(got_kp).ravel().tolist(), 'kd_readback': np.asarray(got_kd).ravel().tolist(),
    'gravity': str(world.get_physics_context().get_gravity()), 'mode': mode, 'hand_side': hand_side}, indent=2))
camera = Camera('/World/Camera', resolution=(640, 640))
camera.initialize()
camera.set_clipping_range(.01, 10.)
cx = UsdGeom.Xformable(camera.prim)
cx.ClearXformOpOrder()
cx.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
    Gf.Vec3d(.50, .56, .40), Gf.Vec3d(0., 0., .15), Gf.Vec3d(0., 0., 1.)).GetInverse())
side_camera = Camera('/World/SideCamera', resolution=(640, 640))
side_camera.initialize()
side_camera.set_clipping_range(.01, 10.)
sx = UsdGeom.Xformable(side_camera.prim)
sx.ClearXformOpOrder()
sx.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
    Gf.Vec3d(-.50, .36, .33), Gf.Vec3d(0., 0., .15), Gf.Vec3d(0., 0., 1.)).GetInverse())
for view_name in ['front', 'side']:
    out.joinpath('frames', view_name).mkdir(parents=True)
frame_file = out.joinpath('frames.jsonl').open('w', buffering=1)
phase = 'initial'
fixture_target = np.zeros(len(fixture_ids), dtype=np.float32)
palm_view = SimulationManager.get_physics_sim_view().create_rigid_body_view('/World/Hand/' + base_link)
assert palm_view.count == 1
def step(target, count):
    for _ in range(count):
        hand.apply_action(ArticulationAction(joint_positions=np.asarray(target, dtype=np.float32), joint_indices=ids))
        if wrist_fixture:
            hand.apply_action(ArticulationAction(joint_positions=fixture_target, joint_indices=fixture_ids))
        world.step(render=False)
        if (len(trace) + 1) % 4 == 0:
            world.render()
        q_all = np.asarray(hand.get_joint_positions()).ravel()
        dq_all = np.asarray(hand.get_joint_velocities()).ravel()
        assert np.isfinite(q_all).all() and np.isfinite(dq_all).all()
        q, dq = q_all[hand_ids], dq_all[hand_ids]
        errors = {child: float(q[names.index(child)] - (m['multiplier'] * q[names.index(m['parent'])] + m['offset']))
                  for child, m in mimics.items()}
        trace.append({'sequence': len(trace), 'physics_s': world.current_time, 'phase': phase,
                      'command_names': independent, 'command_rad': list(map(float, target)),
                      'q_rad': q.tolist(), 'dq_rad_s': dq.tolist(), 'coupling_error_rad': errors,
                      'effective_q_target': np.asarray(hand.get_applied_action().joint_positions).ravel()[hand_ids].tolist(),
                      'commanded_feedforward_effort_nm': np.asarray(hand.get_applied_joint_efforts()).ravel()[hand_ids].tolist(),
                      'measured_joint_effort_nm': np.asarray(hand.get_measured_joint_efforts()).ravel()[hand_ids].tolist(),
                      'palm_pose_world_xyzw': np.asarray(palm_view.get_transforms())[0].tolist(),
                      'palm_velocity_world_m_s_rad_s': np.asarray(palm_view.get_velocities())[0].tolist()})
        if wrist_fixture:
            trace[-1]['fixture'] = {'names': [a['name'] for a in fixture_axes],
                'position_units': [a['position_unit'] for a in fixture_axes],
                'effort_units': [a['effort_unit'] for a in fixture_axes],
                'command': fixture_target.tolist(), 'q': q_all[fixture_ids].tolist(),
                'dq': dq_all[fixture_ids].tolist(),
                'measured_effort': np.asarray(hand.get_measured_joint_efforts()).ravel()[fixture_ids].tolist()}
        trace_file.write(json.dumps(trace[-1], allow_nan=False) + '\n')
        if len(trace) % 20 == 0:
            pixels = camera.get_rgba()
            if pixels is not None and pixels.size:
                frames.append(Image.fromarray(pixels.astype(np.uint8)))
                frame_id = len(frames) - 1
                captured = {}
                for label, sensor in [('front', camera), ('side', side_camera)]:
                    raw = sensor.get_rgba()
                    assert raw is not None and raw.shape == (640, 640, 4)
                    file = f'frames/{label}/{frame_id:06d}.png'
                    Image.fromarray(raw.astype(np.uint8)).save(out / file)
                    captured[label] = file
                frame_file.write(json.dumps({'frame': frame_id, 'sequence': trace[-1]['sequence'],
                    'physics_s': trace[-1]['physics_s'], 'phase': phase,
                    'captured_after_same_step_render': True, 'views': captured}) + '\n')

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
blocked_index = None
if 'blocked' in mode or wrist_fixture:
    index_name = hand_prefix + 'index_1_joint'
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
    free_start = {'q': np.asarray(hand.get_joint_positions()).ravel()[hand_ids].tolist(), 'dq': np.asarray(hand.get_joint_velocities()).ravel()[hand_ids].tolist()}
    index_trajectory('free_index')
    index_id = names.index(index_name)
    free_window = trace[-60:]
    free_q = float(np.mean([r['q_rad'][index_id] for r in free_window]))
    tip = SimulationManager.get_physics_sim_view().create_rigid_body_view('/World/Hand/' + hand_prefix + 'index_force_sensor_3')
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
    blocked_start = {'q': np.asarray(hand.get_joint_positions()).ravel()[hand_ids].tolist(), 'dq': np.asarray(hand.get_joint_velocities()).ravel()[hand_ids].tolist()}
    index_trajectory('blocked_index')
    blocked_window = trace[-60:]
    blocked_q = float(np.mean([r['q_rad'][index_id] for r in blocked_window]))
    block_contacts = [r for r in contacts if '/World/IndexBlock' in (r['actor0'], r['actor1'])]
    active_contacts = [r for r in block_contacts if r['phase'].startswith('blocked_index')]
    nonzero = [r for r in active_contacts if np.linalg.norm(r['impulse_ns']) > 1e-10]
    hold_contact_sequences = {r['sequence'] for r in nonzero if r['phase'] == 'blocked_index_hold'}
    unexpected = [r for r in nonzero if not any('/' + hand_prefix + 'index' in r[k] for k in ['actor0', 'actor1'])]
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
        'placement_source_body': '/World/Hand/' + hand_prefix + 'index_force_sensor_3',
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
    saved_scene.GetPrimAtPath('/World/Hand').GetReferences().AddReference(selection['usd_filename'])
    saved_scene.GetRootLayer().Save()
wrist_result = None
if wrist_fixture:
    # These limits are declared before evaluation. A driven six-axis laboratory
    # support is eligible only for a supported moving-contact mechanism result.
    # The obstacle stays in the world throughout release and wrist motion.
    phase = 'blocked_release'
    step(zero, 240)
    release_end = trace[-1]['sequence']
    release_contacts = [r for r in contacts if r['sequence'] is not None
        and release_end - 59 <= r['sequence'] <= release_end
        and '/World/IndexBlock' in (r['actor0'], r['actor1'])
        and np.linalg.norm(r['impulse_ns']) > 1e-10]
    fixture_start = trace[-1]['fixture']['q']
    palm_start = trace[-1]['palm_pose_world_xyzw']
    axis_results = []
    for axis_index, axis in enumerate(fixture_axes):
        amplitude = .02 if axis['type'] == 'prismatic' else .15
        begin = len(trace)
        phase = 'wrist_out_' + axis['name']
        for tick in range(120):
            u = tick / 119
            fixture_target[axis_index] = amplitude * (10*u**3 - 15*u**4 + 6*u**5)
            step(zero, 1)
        phase = 'wrist_hold_' + axis['name']
        step(zero, 80)
        held = trace[-40:]
        phase = 'wrist_return_' + axis['name']
        for tick in range(120):
            u = tick / 119
            fixture_target[axis_index] = amplitude * (1 - (10*u**3 - 15*u**4 + 6*u**5))
            step(zero, 1)
        phase = 'wrist_reset_' + axis['name']
        step(zero, 80)
        settled = trace[-40:]
        actual_delta = float(np.mean([r['fixture']['q'][axis_index] for r in held]) - fixture_start[axis_index])
        reset_delta = float(np.mean([r['fixture']['q'][axis_index] for r in settled]) - fixture_start[axis_index])
        hold_speed = float(max(abs(r['fixture']['dq'][axis_index]) for r in held))
        motion = trace[begin:]
        observed_speed = max(abs(r['fixture']['dq'][axis_index]) for r in motion)
        observed_effort = max(abs(r['fixture']['measured_effort'][axis_index]) for r in motion)
        # Measured generalized joint effort includes the constraint reaction;
        # report it separately from the force-limited drive estimate.
        drive_effort = [axis['kp']*(r['fixture']['command'][axis_index] - r['fixture']['q'][axis_index])
                        - axis['kd']*r['fixture']['dq'][axis_index] for r in motion]
        saturation_samples = sum(abs(e) >= axis['effort_limit']*.99 for e in drive_effort)
        excursion_tol = .003 if axis['type'] == 'prismatic' else .02
        reset_tol = .001 if axis['type'] == 'prismatic' else .01
        speed_tol = .005 if axis['type'] == 'prismatic' else .03
        axis_results.append({'axis': axis['name'], 'position_unit': axis['position_unit'],
            'command_delta': amplitude, 'measured_hold_delta': actual_delta,
            'reset_delta': reset_delta, 'hold_max_abs_speed': hold_speed,
            'observed_max_abs_speed': observed_speed, 'observed_max_abs_generalized_effort': observed_effort,
            'estimated_drive_saturation_samples': saturation_samples,
            'drive_estimate_kind': 'kp_position_error_minus_kd_velocity_before_force_limit_not_direct_force_readback',
            'samples': len(motion), 'thresholds': {'excursion_error': excursion_tol,
                'reset_error': reset_tol, 'hold_speed': speed_tol, 'velocity_limit': axis['velocity_limit']},
            'checks': {'measured_excursion': abs(actual_delta-amplitude) <= excursion_tol,
                'returned_to_start': abs(reset_delta) <= reset_tol, 'settled_hold': hold_speed <= speed_tol,
                'velocity_limit': observed_speed <= axis['velocity_limit']*1.05}})
    wrist_result = {'scope': 'supported_virtual_fixture_provisional_hand',
        'fixture': wrist_fixture, 'axis_results': axis_results,
        'palm_start_world_xyzw': palm_start, 'palm_end_world_xyzw': trace[-1]['palm_pose_world_xyzw'],
        'block_removed_or_teleported': False, 'contact_loss_window_steps': 60,
        'release_nonzero_block_contact_points': len(release_contacts),
        'checks': {'released_block_contact': not release_contacts,
                   **{a['axis']+'_'+k: v for a in axis_results for k,v in a['checks'].items()}},
        'robot_wrist_qualification': False, 'standing_qualification': False}
    out.joinpath('wrist_motion.json').write_text(json.dumps(wrist_result, indent=2, allow_nan=False))
Image.fromarray(camera.get_rgba().astype(np.uint8)).save(out / 'hand.png')
if frames:
    frames[0].save(out / 'hand_sweeps.gif', save_all=True, append_images=frames[1:], duration=100, loop=0)
trace_file.close()
contact_file.close()
frame_file.close()
coupling_max = max(abs(e) for r in trace for e in r['coupling_error_rad'].values())
excursions = {n: float(np.ptp([r['q_rad'][names.index(n)] for r in trace if r['phase'] == 'sweep_' + n])) for n in independent}
step_dts = np.diff([r['physics_s'] for r in trace])
joint_limit_violation = max(max(lo - r['q_rad'][names.index(n)], r['q_rad'][names.index(n)] - hi, 0.)
                            for n, (lo, hi) in limits.items() for r in trace)
checks = {'all_axes_move': all(excursions[n] >= .1 * (limits[n][1] - limits[n][0]) for n in independent),
          'coupling': coupling_max < .03, 'joint_limits': joint_limit_violation < .03,
          'complete_steps': len(trace) == ((2100 if blocked_index is not None else 1040) + (2640 if wrist_fixture else 0)),
          'physics_dt': bool(np.allclose(step_dts, .005, atol=1e-8))}
if blocked_index is not None:
    checks.update({'blocked_' + k: v for k, v in blocked_index['checks'].items()})
if wrist_result is not None:
    checks.update({'wrist_' + k: v for k, v in wrist_result['checks'].items()})
metrics = {'source_model': 'Unitree_FTP_donor_exact_E2_equivalence_unverified',
           'target_model': 'RH56E2-2R-T1' if hand_side == 'right' else 'RH56E2-2L-T1',
           'hand_side': hand_side, 'exact_asset_qualified': False,
           'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
           'source_mass_kg': want_mass, 'imported_hand_mass_kg': mass-fixture_mass,
           'fixture_added_mass_kg': fixture_mass, 'imported_mass_kg': mass,
           'manufacturer_nominal_E2_T1_hand_kg': .79, 'mass_rescaled': False,
           'fixed_base': True, 'tool_attached': False, 'controller': 'six_independent_position_drives',
           'diagnostic_mode': mode, 'gravity': str(world.get_physics_context().get_gravity()),
           'collision_candidate': candidate, 'mechanism_track': 'provisional_engineering_candidate',
           'qualification_track': 'exact_RH56E2_fidelity_pending',
           'support_constraints': ['fixed_virtual_six_axis_fixture_anchor' if wrist_fixture else 'fixed_wrist_bench_root'],
           'wrist_motion_qualification': ('PASS' if all(checks.values()) else 'FAIL') if wrist_fixture else 'NOT_RUN',
           'wrist_motion_scope': 'supported_provisional_mechanism_not_robot_wrist_or_balance',
           'wrist_fixture': wrist_fixture,
           'media_capture': {'views': ['front', 'side'], 'physics_steps_per_frame': 20,
                            'physics_dt_s': .005, 'frame_trace_map': 'frames.jsonl',
                            'render_profile': 'full_hand_two_view_key_fill_v2',
                            'source': 'actual_Isaac_camera_readback_no_generated_imagery'},
           'collision_refinement': collision_refinement,
           'collision_api_relocations': collision_api_relocations,
           'diagnostic_filtered_pairs': diagnostic_filtered_pairs,
           'collision_qualification_excluded': clearance_control or static_palm_bench,
           'collision_fixture': collision_fixture,
           'physics_configuration': physics_configuration, 'randomized': False,
           'gains': {'kp': 1., 'kd': .05, 'provenance': 'declared_diagnostic_not_hardware_calibrated'},
           'runtime_names': names, 'all_articulation_runtime_names': runtime_names,
           'independent_names': independent, 'mimic_map': mimics,
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
artifacts = ['metrics.json', 'initial_state.json', 'state.jsonl', 'hand.png', 'hand_sweeps.gif', selection['usd_filename'], 'cooked_colliders.json', 'backend_shapes.json']
artifacts += ['collision_candidate.json', 'frames.jsonl']
artifacts += [str(p.relative_to(out)) for p in sorted(out.joinpath('frames').rglob('*.png'))]
artifacts += [str(p.relative_to(out)) for p in sorted(out.joinpath('configuration').rglob('*.usd'))]
if contacts:
    artifacts.append('contacts.jsonl')
if blocked_index is not None:
    artifacts += ['blocked_index.json', 'blocked_bench_scene.usda']
if wrist_result is not None:
    artifacts += ['wrist_motion.json', 'supported_wrist.urdf']
out.joinpath('probe.json').write_text(json.dumps({'status': 'PASS' if all(checks.values()) else 'FAIL',
                                                'scope': 'provisional_palm_candidate_supported_moving_wrist' if wrist_fixture else 'provisional_palm_collision_candidate_fixed_wrist_bench',
                                                'metrics': 'metrics.json',
                                                'artifacts': artifacts}))
app.close()
