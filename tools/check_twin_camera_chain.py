"""End-to-end test of the twin camera chain, via the twin's own TF.

FOUND C-21 WITH THIS. As of 2026-08-18 it FAILS: the floor lands 1.2 m too low and
59.7 degrees off. Keep it failing until C-21 is fixed -- it is the check that closes
it.

Better question than "what is the camera's roll": does a depth pixel land where the
robot thinks it is? Back-project the aligned depth with the published K, transform
into base_link with the published TF, and check the floor.

If the mount pose, the optical-frame convention and the camera prim's own 180-degree
flip are all right, the floor lands level at the robot's standing height below
base_link. Any error in that chain shows up here, and fitting in base_link means the
floor can be selected BY HEIGHT rather than by guessing which part of the frame it
occupies -- the mistake that made the previous attempt fit a wall.
"""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener

rclpy.init(); n = Node("cam_chain")
q = QoSProfile(depth=5); q.reliability = ReliabilityPolicy.RELIABLE
d, ci = [], []
n.create_subscription(Image, "/ferox/g1_01/camera/aligned_depth_to_color/image_raw",
                      lambda m: d.append(m), q)
n.create_subscription(CameraInfo, "/ferox/g1_01/camera/color/camera_info",
                      lambda m: ci.append(m), q)
buf = Buffer(); TransformListener(buf, n)
t0 = time.time(); tr = None
while time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.2)
    if d and ci and tr is None:
        try:
            tr = buf.lookup_transform("base_link", "camera_color_optical_frame",
                                      rclpy.time.Time())
        except Exception:
            pass
    if d and ci and tr is not None and len(d) >= 2:
        break
if tr is None:
    print("no base_link <- camera_color_optical_frame (is CAMERA_TF=1?)"); raise SystemExit(1)

t = tr.transform.translation; r = tr.transform.rotation
x_, y_, z_, w_ = r.x, r.y, r.z, r.w
R = np.array([[1-2*(y_*y_+z_*z_), 2*(x_*y_-z_*w_),   2*(x_*z_+y_*w_)],
              [2*(x_*y_+z_*w_),   1-2*(x_*x_+z_*z_), 2*(y_*z_-x_*w_)],
              [2*(x_*z_-y_*w_),   2*(y_*z_+x_*w_),   1-2*(x_*x_+y_*y_)]])
off = np.array([t.x, t.y, t.z])
print(f"TF base_link <- camera_color_optical_frame: xyz ({off[0]:+.5f}, {off[1]:+.5f}, {off[2]:+.5f})")

m, info = d[-1], ci[-1]
K = np.array(info.k).reshape(3, 3)
fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
zs = np.frombuffer(bytes(m.data), dtype=np.uint16).reshape(m.height, m.width) / 1000.0
v, u = np.nonzero(zs > 0)
z = zs[v, u]
cam = np.stack([(u-cx)*z/fx, (v-cy)*z/fy, z], axis=1)
b = (R @ cam.T).T + off
print(f"{len(b)} valid pixels -> base_link z range {b[:,2].min():+.3f} .. {b[:,2].max():+.3f} m")
hist = np.histogram(b[:,2], bins=np.arange(-1.2, 1.4, 0.2))
print("  z hist:", " ".join(f"{e:+.1f}:{c}" for e, c in zip(hist[1][:-1], hist[0])))

# The floor is the lowest surface. Robot stands ~0.79, so look below -0.60.
cand = b[b[:,2] < -0.60]
print(f"floor candidates (z < -0.60): {len(cand)}")
if len(cand) < 500:
    print("  too few floor pixels to fit"); raise SystemExit(1)
A = np.c_[cand[:,0], cand[:,1], np.ones(len(cand))]
coef, *_ = np.linalg.lstsq(A, cand[:,2], rcond=None)
res = cand[:,2] - A @ coef
keep = np.abs(res) < 2*res.std()
cand = cand[keep]; A = np.c_[cand[:,0], cand[:,1], np.ones(len(cand))]
(aa, bb, cc), *_ = np.linalg.lstsq(A, cand[:,2], rcond=None)
nrm = np.array([-aa, -bb, 1.0]); nrm /= np.linalg.norm(nrm)
tilt = math.degrees(math.acos(min(1.0, abs(nrm[2]))))
rms = float(np.sqrt(((cand[:,2] - A @ np.array([aa,bb,cc]))**2).mean()))
print(f"floor in base_link: z = {cc:+.4f} m under base_link, tilt {tilt:.3f} deg, RMS {rms:.4f} m ({len(cand)} pts)")
print(f"  robot standing height (odom z) : ~0.790 m")
print(f"  floor depth vs standing height : {abs(abs(cc)-0.790):.4f} m")
print(f"  => chain verdict: {'PASS' if tilt < 2.0 and abs(abs(cc)-0.790) < 0.08 else 'FAIL'}"
      f"  (tilt < 2 deg, |floor| within 0.08 m of 0.790)")
rclpy.shutdown()
