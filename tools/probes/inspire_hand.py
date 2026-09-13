"""Qualify a provisional FTP donor's import and six-axis bench dynamics.

This is a fixed-base donor diagnostic, never an exact RH56E2, grasp, tactile,
or standing qualification. Six root drives actuate twelve coupled coordinates.
All commands and measured contact impulses are saved, including failed checks.
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
source = Path('/source-assets/FTP_right_hand_bench.urdf')
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
assert mode in {'default', 'zero-gravity', 'refined-palm', 'zero-gravity-refined-palm',
                'mesh-colliders', 'zero-gravity-mesh-colliders'}
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting'})
import numpy as np
import omni.kit.commands
from omni.physx import get_physx_simulation_interface
from pxr import Gf, PhysxSchema, PhysicsSchemaTools, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics
from PIL import Image
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
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
dest = out / 'ftp_right_bench.usd'
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
if mode in {'mesh-colliders', 'zero-gravity-mesh-colliders'}:
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
if mode in {'refined-palm', 'zero-gravity-refined-palm'}:
    # The source palm has 43 disconnected components; the default 32-hull
    # decomposition may bridge its thumb cavity. Test a declared finer collision
    # approximation, preserving source triangles, mass and every collision pair.
    collision_root = stage.GetPrimAtPath('/Rhand/right_base_link/collisions')
    assert collision_root.IsInstance(), 'Expected pinned donor collision instance'
    collision_root.SetInstanceable(False)
    for prim in Usd.PrimRange(collision_root):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            assert prim.GetPath().pathString.endswith('/right_base_link/node_STL_BINARY_')
            assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == 'convexDecomposition'
            api = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            api.CreateMaxConvexHullsAttr().Set(128)
            api.CreateErrorPercentageAttr().Set(1.)
            api.CreateVoxelResolutionAttr().Set(1000000)
            api.CreateShrinkWrapAttr().Set(True)
            collision_refinement.append({'prim': str(prim.GetPath()), 'preset': 'palm_cavity_v1',
                'maxConvexHulls': api.GetMaxConvexHullsAttr().Get(),
                'errorPercentage': api.GetErrorPercentageAttr().Get(),
                'voxelResolution': api.GetVoxelResolutionAttr().Get(),
                'shrinkWrap': api.GetShrinkWrapAttr().Get(),
                'collision_pairs_filtered': False, 'source_triangles_changed': False})
    assert len(collision_refinement) == 1, 'Pinned palm collider not found'
stage.GetRootLayer().Save()
world = World(stage_units_in_meters=1., physics_dt=.005, rendering_dt=.02)
if mode.startswith('zero-gravity'):
    world.get_physics_context().set_gravity(0.)
UsdLux.DomeLight.Define(world.stage, '/World/Light').CreateIntensityAttr(1400.)
add_reference_to_stage(str(dest), '/World/Hand')
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
hand = SingleArticulation('/World/Hand', name='ftp_right_hand')
physics_configuration = {str(p.GetPath()): {a.GetName(): str(a.Get()) for a in p.GetAttributes()
    if a.GetName().startswith(('physics:', 'physx'))} for p in world.stage.Traverse()
    if p.IsA(UsdPhysics.Scene) or p.HasAPI(PhysxSchema.PhysxArticulationAPI)}
world.reset()
hand.initialize()
names = list(hand.dof_names)
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
          'complete_steps': len(trace) == 1040, 'physics_dt': bool(np.allclose(step_dts, .005, atol=1e-8))}
metrics = {'source_model': 'Unitree_FTP_donor_exact_E2_equivalence_unverified',
           'target_model': 'RH56E2-2R-T1', 'exact_asset_qualified': False,
           'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
           'source_mass_kg': want_mass, 'imported_mass_kg': mass,
           'manufacturer_nominal_E2_T1_hand_kg': .79, 'mass_rescaled': False,
           'fixed_base': True, 'tool_attached': False, 'controller': 'six_independent_position_drives',
           'diagnostic_mode': mode, 'gravity': str(world.get_physics_context().get_gravity()),
           'collision_refinement': collision_refinement,
           'collision_api_relocations': collision_api_relocations,
           'physics_configuration': physics_configuration, 'randomized': False,
           'gains': {'kp': 1., 'kd': .05, 'provenance': 'declared_diagnostic_not_hardware_calibrated'},
           'runtime_names': names, 'independent_names': independent, 'mimic_map': mimics,
           'coupling_model': 'nominal_algebraic_URDF_mimic_not_measured_hardware_compliance',
           'mimic_natural_frequency_attribute': 0., 'importer_default_natural_frequency_attribute': 25.,
           'steps': len(trace), 'physics_dt': .005, 'coupling_error_max_rad': coupling_max,
           'axis_excursion_rad': excursions, 'joint_limit_violation_rad': joint_limit_violation,
           'checks': checks, 'measured_physics_dt_min': float(step_dts.min()), 'measured_physics_dt_max': float(step_dts.max()),
           'actual_contact_points': len(contacts), 'blocked_finger_test': 'NOT_RUN',
           'contact_reporting_configured_before_physics': True,
           'initialization_contact_points': sum(r['phase'] == 'initialization' for r in contacts),
           'grasp_qualification': 'NOT_RUN', 'writing_qualification': 'NOT_RUN'}
out.joinpath('metrics.json').write_text(json.dumps(metrics, indent=2))
artifacts = ['metrics.json', 'initial_state.json', 'state.jsonl', 'hand.png', 'hand_sweeps.gif', 'ftp_right_bench.usd']
artifacts += [str(p.relative_to(out)) for p in sorted(out.joinpath('configuration').rglob('*.usd'))]
if contacts:
    artifacts.append('contacts.jsonl')
out.joinpath('probe.json').write_text(json.dumps({'status': 'PASS' if all(checks.values()) else 'FAIL',
                                                'scope': 'donor_import_and_unloaded_sweeps_only',
                                                'metrics': 'metrics.json',
                                                'artifacts': artifacts}))
app.close()
