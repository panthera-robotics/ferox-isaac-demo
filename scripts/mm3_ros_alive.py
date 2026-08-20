#!/usr/bin/env python3
"""MM3 test (d): the twin's ROS topics keep publishing while G1_CONTROL=lowcmd.

Measures SIM-TIME stamp deltas, not wall-clock arrival. That distinction is the whole
point of this file. DT2 gated this twin's sensors at "10.00 Hz EXACTLY in sim time",
and the twin runs below real time whenever physics is pushed up -- so `ros2 topic hz`,
which counts wall arrivals, reports contract_rate x RTF and makes a correct twin look
broken. The first pass of this check read 15.4 Hz on a 51.4 Hz topic and every topic
was off by the same factor, which is what a real-time-factor artefact looks like and
not what a broken publisher looks like.
"""

from __future__ import annotations

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage

NS = "/ferox/g1_01"
# topic -> (type, contract rate, reliability). Rates from isaac/twin/g1_contract.yaml.
SPEC = [
    (f"{NS}/odom", Odometry, 51.4, "reliable"),
    (f"{NS}/imu/data", Imu, 93.7, "best_effort"),
    (f"{NS}/scan", LaserScan, 10.0, "best_effort"),
    ("/livox/lidar", PointCloud2, 10.0, "reliable"),
    ("/livox/imu", Imu, 200.0, "reliable"),
    ("/tf", TFMessage, 100.0, "reliable"),
]


def stamp_of(msg):
    h = getattr(msg, "header", None)
    if h is not None:
        return h.stamp.sec + h.stamp.nanosec * 1e-9
    if isinstance(msg, TFMessage) and msg.transforms:
        st = msg.transforms[0].header.stamp
        return st.sec + st.nanosec * 1e-9
    return None


def main() -> int:
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
    rclpy.init()
    node = Node("mm3_ros_alive")
    got: dict[str, list] = {t: [] for t, _, _, _ in SPEC}
    wall: dict[str, list] = {t: [] for t, _, _, _ in SPEC}

    for topic, typ, _, rel in SPEC:
        qos = QoSProfile(
            depth=50, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=(QoSReliabilityPolicy.RELIABLE if rel == "reliable"
                         else QoSReliabilityPolicy.BEST_EFFORT),
            durability=QoSDurabilityPolicy.VOLATILE)
        node.create_subscription(
            typ, topic,
            lambda m, t=topic: (got[t].append(stamp_of(m)), wall[t].append(time.perf_counter())),
            qos)

    t0 = time.perf_counter()
    while time.perf_counter() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"== MM3 (d) ROS topics under G1_CONTROL=lowcmd, {dur:.0f}s window ==")
    print(f"{'topic':28s} {'n':>6s} {'sim Hz':>9s} {'contract':>9s} {'err%':>7s} "
          f"{'wall Hz':>9s} {'verdict':>8s}")
    rtfs, ok = [], True
    for topic, _, want, _ in SPEC:
        ss = [s for s in got[topic] if s is not None]
        ws = wall[topic]
        if len(ss) < 3:
            print(f"{topic:28s} {len(ss):6d} {'-':>9s} {want:9.1f} {'-':>7s} {'-':>9s} "
                  f"{'ABSENT':>8s}")
            ok = False
            continue
        sim_hz = (len(ss) - 1) / (ss[-1] - ss[0]) if ss[-1] > ss[0] else 0.0
        wall_hz = (len(ws) - 1) / (ws[-1] - ws[0])
        err = 100.0 * (sim_hz - want) / want
        # Class B is +-10% (contract header). Judged in SIM time, per DT2.
        verdict = "PASS" if abs(err) <= 10.0 else "FAIL"
        ok &= verdict == "PASS"
        if sim_hz > 0:
            rtfs.append(wall_hz / sim_hz)
        print(f"{topic:28s} {len(ss):6d} {sim_hz:9.3f} {want:9.1f} {err:+7.2f} "
              f"{wall_hz:9.3f} {verdict:>8s}")
    if rtfs:
        print(f"\nimplied real-time factor (wall/sim, mean over topics): {np.mean(rtfs):.3f}")
    print(f"\n(d) VERDICT: {'PASS' if ok else 'FAIL'}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
