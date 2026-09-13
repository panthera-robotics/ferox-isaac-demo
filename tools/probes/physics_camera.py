"""S0: original Dex5 asset, bounded free-base policy episode and actual RGB/depth.

Run with tools/run_isolated_isaac.py and --policy pointing to the existing
locomotion checkpoint. This is diagnostic evidence for the legacy embodiment.
No hand or writing qualification is inferred from this probe.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
import time

assert sorted(p.name for p in Path('/sys/class/net').iterdir()) == ['lo']
assert os.environ.get('PANTHERA_SIM_AUTHORIZED') == '1'
out = Path('/evidence')
out.joinpath('routes.txt').write_text(Path('/proc/net/route').read_text())
from isaacsim import SimulationApp

app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting',
                     'width': 640, 'height': 480})
print('PROBE: SimulationApp initialized', flush=True)
import numpy as np
from PIL import Image
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.sensors.camera import Camera

sys.path[:0] = ['/workspace/ferox_isaac', '/workspace/ferox_isaac/twin', '/workspace/ferox_tools']
enable_extension('isaacsim.ros2.bridge')
for _ in range(10):
    app.update()
os.environ['FEROX_REUSE_KIT_APP'] = '1'
import run as twin_run
import sensors
import publishers
import twin_contract

asset = '/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd'
policy_dir = Path('/policy')
config = twin_run.parse_env_config(str(policy_dir / 'params/env.yaml'))
spawn = np.asarray(config['scene']['robot']['init_state']['pos'], dtype=np.float32)
world = World(stage_units_in_meters=1.0, physics_dt=0.005, rendering_dt=0.02)
# Local primitives avoid an implicit download of the default grid environment.
world.scene.add(FixedCuboid('/World/Ground', name='ground', position=np.array([0., 0., -.05]),
                           scale=np.array([8., 8., .1]), color=np.array([.2, .24, .3])))
world.scene.add(FixedCuboid('/World/Board', name='board', position=np.array([1.4, 0., 1.1]),
                           scale=np.array([.04, 1.2, .9]), color=np.array([.92, .94, .96])))
for index, (y, rgb) in enumerate([(-.30, [1., .08, .05]), (.0, [.05, .7, .1]), (.3, [.05, .2, 1.])]):
    world.scene.add(FixedCuboid(f'/World/Target{index}', name=f'target{index}',
                               position=np.array([1., y, .20]), scale=np.array([.2, .2, .4]),
                               color=np.asarray(rgb)))
UsdLux.DomeLight.Define(world.stage, '/World/Light').CreateIntensityAttr(1200.)
add_reference_to_stage(asset, '/World/G1')
robot_xform = UsdGeom.Xformable(world.stage.GetPrimAtPath('/World/G1'))
robot_xform.ClearXformOpOrder()
robot_xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, spawn)))
controller = twin_run.G1VelocityPolicy(
    prim_path='/World/G1', name='g1_legacy', usd_path=None, position=spawn,
    orientation=np.array([1., 0., 0., 0.], dtype=np.float32),
    policy_path=str(policy_dir / 'exported/policy.pt'),
    env_path=str(policy_dir / 'params/env.yaml'),
    deploy_path=str(policy_dir / 'params/deploy.yaml'))
world.reset()
controller.initialize()
print('PROBE: articulation initialized', flush=True)
names = list(controller.robot.dof_names)
assert len(names) == len(set(names)) == 69
root_joints = []
for prim in world.stage.Traverse():
    if prim.IsA(UsdPhysics.FixedJoint):
        joint = UsdPhysics.FixedJoint(prim)
        if not joint.GetBody0Rel().GetTargets() or not joint.GetBody1Rel().GetTargets():
            root_joints.append(str(prim.GetPath()))
assert not root_joints, f'Unexpected support constraints: {root_joints}'
contract = twin_contract.load('/workspace/ferox_isaac/twin/g1_contract.yaml')
camera, intrinsic = sensors.create_camera(contract, '/World/G1', want_depth_frame=True)
publishers.setup_camera_color(contract, camera, '/sim/legacy')
publishers.setup_camera_depth_raw(contract, camera, '/sim/legacy')
print('PROBE: camera and ROS image writers attached', flush=True)
overview = Camera('/World/Overview', name='overview', resolution=(640, 480))
overview.initialize()
overview.prim.GetAttribute('focalLength').Set(24.)
overview_xform = UsdGeom.Xformable(overview.prim)
overview_xform.ClearXformOpOrder()
overview_xform.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
    Gf.Vec3d(2.4, -2.4, 1.6), Gf.Vec3d(.2, 0., .75), Gf.Vec3d(0., 0., 1.)).GetInverse())

# Receive the real ROS messages inside the same network-isolated namespace.
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage
rclpy.init()
node = rclpy.create_node('legacy_camera_evidence')
received = {'rgb': [], 'depth': []}
def record(kind, msg):
    received[kind].append({'stamp_s': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                           'height': msg.height, 'width': msg.width, 'encoding': msg.encoding,
                           'bytes': len(msg.data), 'row_bytes': msg.step, 'frame_id': msg.header.frame_id})
subscriptions = [node.create_subscription(RosImage, topic, lambda msg, k=kind: record(k, msg),
                                           qos_profile_sensor_data)
                 for kind, topic in [('rgb', '/sim/legacy/camera/color/image_raw'),
                                     ('depth', '/sim/legacy/camera/depth/image_rect_raw_32f')]]
trace = []
trace_file = out.joinpath('state.jsonl').open('w', buffering=1)
frames = []
start = time.monotonic()
fell = False
for step in range(400):
    controller.forward(0.005, np.zeros(3, dtype=np.float32))
    world.step(render=False)
    if step % 4 == 0:
        world.render()
    rclpy.spin_once(node, timeout_sec=0.)
    position, quaternion = controller.robot.get_world_pose()
    q = controller.robot.get_joint_positions()
    dq = controller.robot.get_joint_velocities()
    if not np.isfinite(np.concatenate([position, quaternion, q, dq])).all():
        raise RuntimeError('Nonfinite physics state')
    trace.append({'sequence': step, 'physics_s': world.current_time,
                  'wall_s': time.monotonic() - start, 'position': position.tolist(),
                  'quaternion_wxyz': quaternion.tolist(), 'q': q.tolist(), 'dq': dq.tolist(),
                  'effective_q_target': np.asarray(controller.robot.get_applied_action().joint_positions).tolist()})
    trace_file.write(json.dumps(trace[-1], allow_nan=False) + '\n')
    if step % 20 == 0:
        rgba = overview.get_rgba()
        if rgba is not None and rgba.size:
            frames.append(Image.fromarray(np.asarray(rgba[:, :, :3], dtype=np.uint8)))
    if position[2] < 0.5:
        fell = True
        break
rgba = camera.get_rgba()
assert rgba is not None and rgba.shape == (720, 1280, 4), f'RGB shape: {getattr(rgba, "shape", None)}'
spatial_std = np.std(rgba[:, :, :3].astype(float), axis=(0, 1))
assert np.max(spatial_std) > 1., f'No spatial image content: {spatial_std}'
Image.fromarray(rgba.astype(np.uint8)).save(out / 'camera.png')
Image.fromarray(overview.get_rgba().astype(np.uint8)).save(out / 'overview.png')
depth = camera.get_current_frame().get('distance_to_image_plane')
assert depth is not None and depth.shape == (720, 1280), 'Missing aligned depth'
np.save(out / 'depth_m.npy', depth)
trace_file.close()
if frames:
    frames[0].save(out / 'camera_episode.gif', save_all=True, append_images=frames[1:], duration=100, loop=0)
step_dts = np.diff([row['physics_s'] for row in trace])
assert np.allclose(step_dts, .005, atol=1e-8), 'Physics step duration differs from the declared dt'
cam_world = UsdGeom.XformCache().GetLocalToWorldTransform(camera.prim)
metrics = {'embodiment': 'legacy_g1_dex5_diagnostic', 'physics_dt': .005,
           'controller': 'existing_G1VelocityPolicy_omni_surrogate',
           'body_writers': 1, 'support_constraints': root_joints, 'tool_attachment': False,
           'runtime_joint_names': names, 'policy_joint_indices': controller._policy_dofs.tolist(),
           'steps': len(trace), 'simulation_seconds': world.current_time,
           'wall_seconds': time.monotonic() - start, 'fell': fell,
           'min_pelvis_z': min(row['position'][2] for row in trace),
           'camera_K_readback': intrinsic, 'ros_messages': received,
           'camera_spatial_rgb_std': spatial_std.tolist(),
           'authored_camera_world_matrix_not_runtime_Fabric_pose': [[float(cam_world[i][j]) for j in range(4)] for i in range(4)],
           'measured_physics_dt_min': float(step_dts.min()), 'measured_physics_dt_max': float(step_dts.max()),
           'finite_positive_depth_fraction': float(np.mean(np.isfinite(depth) & (depth > 0))),
           'asset_sha256': hashlib.sha256(Path(asset).read_bytes()).hexdigest(),
           'policy_sha256': hashlib.sha256((policy_dir / 'exported/policy.pt').read_bytes()).hexdigest(),
           'inspire_qualification': 'NOT_RUN', 'writing_qualification': 'NOT_RUN'}
out.joinpath('metrics.json').write_text(json.dumps(metrics, indent=2))
ros_checks = {}
for kind, encoding in [('rgb', 'rgb8'), ('depth', '32FC1')]:
    messages = received[kind]
    bytes_per_pixel = 3 if kind == 'rgb' else 4
    ros_checks[kind] = bool(len(messages) >= 2 and all(
        m['width'] == 1280 and m['height'] == 720 and m['encoding'] == encoding
        and m['row_bytes'] >= 1280 * bytes_per_pixel and m['bytes'] == m['row_bytes'] * 720
        and m['frame_id'] == 'camera_color_optical_frame' for m in messages)
        and all(a['stamp_s'] < b['stamp_s'] for a, b in zip(messages, messages[1:])))
rgb_stamps = {m['stamp_s'] for m in received['rgb']}
depth_stamps = {m['stamp_s'] for m in received['depth']}
ros_checks['paired_timestamps'] = len(rgb_stamps & depth_stamps) >= max(len(rgb_stamps), len(depth_stamps)) - 1
passed = all(ros_checks.values()) and not fell and metrics['finite_positive_depth_fraction'] >= .95
out.joinpath('probe.json').write_text(json.dumps({'status': 'PASS' if passed else 'FAIL',
    'ros_checks': ros_checks, 'metrics': 'metrics.json',
    'artifacts': ['metrics.json', 'state.jsonl', 'camera.png', 'overview.png', 'depth_m.npy', 'camera_episode.gif']}))
node.destroy_node()
rclpy.shutdown()
assert received['rgb'] and received['depth'], 'Image writer configured but ROS samples absent'
assert not fell, 'Legacy policy fell in the measured episode; see trace'
app.close()
