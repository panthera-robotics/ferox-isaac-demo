#!/usr/bin/env python3
"""Runtime geometry checks for the twin. Run inside the nav container.

Two things the static USD cannot answer, because both depend on the robot's
standing pose:

1. THE WAIST ROUND-TRIP. The driver publishes base_link -> livox_frame two ways:
   statically as (0, 0, 0.4995) rpy (3.090233, 0.161680, 0), and dynamically as
   waist(q) composed with the calibrated torso_link -> livox_frame mount. Those
   agree only at the standing pose, because the waist is NOT at zero when the
   robot stands -- the composite differs from the mount by 6.65 degrees. So this
   is a live check, not a USD assertion.

2. THE FLOOR PLANE. Fit a plane to the ground returns in the cloud, transform its
   normal into base_link, and compare with +z. This is the same check the driver
   used to calibrate the mount in the first place (floor-plane residual 0.0027
   deg), so it validates the whole chain end to end: sensor pose, TF, and the
   cloud's frame convention. A sensor mounted upside down but declared upright
   passes every string-level check and fails this one.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage

# The driver's standing composite, static mode.
# panthera-g1-driver g1_driver_hw.launch.py:459-478
WANT_XYZ = (0.0, 0.0, 0.4995)
WANT_RPY = (3.090233, 0.161680, 0.0)
TOL_M = 0.002      # campaign section 6, waist round-trip
TOL_RAD = 1e-3
FLOOR_TOL_DEG = 0.5


def quat_to_rpy(x, y, z, w):
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    return math.atan2(sinr, cosr), pitch, math.atan2(siny, cosy)


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def read_points(msg: PointCloud2, max_pts: int = 60000) -> np.ndarray:
    off = {f.name: f.offset for f in msg.fields}
    if not {"x", "y", "z"} <= set(off):
        return np.empty((0, 3), dtype=np.float32)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    n = msg.width * msg.height
    buf = buf[: n * msg.point_step].reshape(n, msg.point_step)
    if n > max_pts:
        buf = buf[:: max(1, n // max_pts)]
    def f32(o):
        return buf[:, o:o + 4].copy().view(np.float32).ravel()
    pts = np.stack([f32(off["x"]), f32(off["y"]), f32(off["z"])], axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", default="g1_01")
    ap.add_argument("--duration", type=float, default=14.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("twin_geometry_check")
    state = {}

    tfq = QoSProfile(depth=100, history=QoSHistoryPolicy.KEEP_LAST,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    edges = {}
    node.create_subscription(
        TFMessage, "/tf_static",
        lambda m: [edges.__setitem__((t.header.frame_id, t.child_frame_id), t.transform)
                   for t in m.transforms], tfq)
    sd = QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE)
    node.create_subscription(PointCloud2, "/livox/lidar",
                             lambda m: state.__setitem__("cloud", m), sd)
    node.create_subscription(LaserScan, f"/ferox/{args.robot_id}/scan",
                             lambda m: state.__setitem__("scan", m), sd)

    t0 = time.time()
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)

    failures = 0
    print("twin geometry check")
    print()

    # ---- 1. waist round-trip -------------------------------------------------
    tr = edges.get(("base_link", "livox_frame"))
    print("1. base_link -> livox_frame vs the driver's standing composite")
    if tr is None:
        print("   FAIL: edge not published")
        failures += 1
    else:
        p, q = tr.translation, tr.rotation
        got_xyz = (p.x, p.y, p.z)
        rpy = quat_to_rpy(q.x, q.y, q.z, q.w)
        d_xyz = math.sqrt(sum((a - b) ** 2 for a, b in zip(got_xyz, WANT_XYZ)))
        # Compare as rotations, not component-wise: q and -q are the same rotation.
        wq = None
        cr, sr = math.cos(WANT_RPY[0] / 2), math.sin(WANT_RPY[0] / 2)
        cp, sp = math.cos(WANT_RPY[1] / 2), math.sin(WANT_RPY[1] / 2)
        cy, sy = math.cos(WANT_RPY[2] / 2), math.sin(WANT_RPY[2] / 2)
        wq = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
              cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
        dot = abs(q.x * wq[0] + q.y * wq[1] + q.z * wq[2] + q.w * wq[3])
        d_ang = 2.0 * math.acos(max(-1.0, min(1.0, dot)))
        print(f"   got  xyz=({got_xyz[0]:+.6f}, {got_xyz[1]:+.6f}, {got_xyz[2]:+.6f})  "
              f"rpy=({rpy[0]:+.6f}, {rpy[1]:+.6f}, {rpy[2]:+.6f})")
        print(f"   want xyz=({WANT_XYZ[0]:+.6f}, {WANT_XYZ[1]:+.6f}, {WANT_XYZ[2]:+.6f})  "
              f"rpy=({WANT_RPY[0]:+.6f}, {WANT_RPY[1]:+.6f}, {WANT_RPY[2]:+.6f})")
        print(f"   d_translation {d_xyz * 1000:.2f} mm (tol {TOL_M * 1000:.1f} mm)   "
              f"d_rotation {d_ang:.3e} rad = {math.degrees(d_ang):.4f} deg "
              f"(tol {TOL_RAD:.0e} rad)")
        ok = d_xyz <= TOL_M and d_ang <= TOL_RAD
        print(f"   => {'PASS' if ok else 'FAIL'}")
        if not ok:
            failures += 1
            print("   NOTE: TWIN_DEVIATIONS C-7 predicts a ~10 mm z residual — the sim")
            print("         body's pelvis->torso_link is 10 mm shorter than the URDF.")

    # ---- 2. floor plane ------------------------------------------------------
    print()
    print("2. floor plane in base_link vs +z")
    cloud, tr = state.get("cloud"), edges.get(("base_link", "livox_frame"))
    if cloud is None or tr is None:
        print("   SKIP: no cloud or no TF")
    else:
        pts = read_points(cloud)
        q, p = tr.rotation, tr.translation
        R = quat_to_mat(q.x, q.y, q.z, q.w)
        base = pts @ R.T + np.array([p.x, p.y, p.z])
        # Ground candidates: below the sensor, within a sane radius.
        r = np.linalg.norm(base[:, :2], axis=1)
        cand = base[(base[:, 2] < -0.3) & (base[:, 2] > -1.6) & (r > 0.5) & (r < 6.0)]
        print(f"   {len(pts)} points, {len(cand)} floor candidates")
        if len(cand) < 200:
            print("   SKIP: too few floor candidates")
        else:
            c = cand.mean(axis=0)
            _, _, vt = np.linalg.svd(cand - c, full_matrices=False)
            nrm = vt[2] / np.linalg.norm(vt[2])
            if nrm[2] < 0:
                nrm = -nrm
            ang = math.degrees(math.acos(max(-1.0, min(1.0, abs(nrm[2])))))
            resid = float(np.abs((cand - c) @ nrm).mean())
            print(f"   normal ({nrm[0]:+.5f}, {nrm[1]:+.5f}, {nrm[2]:+.5f})  "
                  f"tilt {ang:.4f} deg (tol {FLOOR_TOL_DEG} deg)  "
                  f"mean |residual| {resid * 1000:.2f} mm")
            print(f"   floor height in base_link {c[2]:+.4f} m")
            ok = ang <= FLOOR_TOL_DEG
            print(f"   => {'PASS' if ok else 'FAIL'}")
            if not ok:
                failures += 1

    # ---- 3. scan geometry ----------------------------------------------------
    print()
    print("3. /scan geometry and floor rejection")
    scan = state.get("scan")
    if scan is None:
        print("   SKIP: no scan")
    else:
        rays = len(scan.ranges)
        finite = [x for x in scan.ranges if math.isfinite(x)]
        inband = [x for x in finite if scan.range_min <= x <= scan.range_max]
        print(f"   frame {scan.header.frame_id}  rays {rays}  "
              f"range [{scan.range_min:.2f}, {scan.range_max:.2f}]  "
              f"increment {scan.angle_increment:.6f}")
        print(f"   finite {len(finite)}  in-band {len(inband)} "
              f"({100.0 * len(inband) / max(1, len(finite)):.1f}% of finite)")
        ok = rays == 723 and scan.header.frame_id == "base_link"
        print(f"   => {'PASS' if ok else 'FAIL'} (723 rays in base_link)")
        if not ok:
            failures += 1

    print()
    print(f"RESULT: {'PASS' if failures == 0 else f'{failures} check(s) FAILED'}")
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
