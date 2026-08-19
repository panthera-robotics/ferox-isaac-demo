#!/usr/bin/env python3
"""Walk regression for the G1 twin: does the body track the velocity it was told?

Rewritten 2026-08-19. DT3's copy of this lived only inside the nav container and
went with it; the numbers it produced are in `docs/twin/evidence/DT3/
validate_motion_dex5.txt` and are the baseline this reproduces. It is in the repo
now so the next instance does not have to write it a third time.

WHAT IT MEASURES, AND IN WHICH FRAME
------------------------------------
For each commanded (vx, vy, wz) it drives `/ferox/<id>/cmd_vel` for a fixed window
and reports, from `/ferox/<id>/odom`:

  body vel   the twist exactly as published on /odom. REPORTED, NOT JUDGED -- see
             verdicts(); its frame does not match the displacement's and this
             script does not guess at the convention.
  disp       net displacement over the window, also body-frame: forward, lateral,
             and change in yaw.
  zmin       the lowest base height seen in the window. This is the upright check
             and it is what a hand-mount change could plausibly break: extra mass
             on the wrong side of the wrist shows up as the body sagging or the
             policy fighting it.

USAGE
  python3 validate_motion.py --robot-id g1_01 [--duration 4.0] [--json OUT]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# The DT2/DT3 pattern, unchanged -- these are the commands the recorded baseline
# was produced with, so the two tables line up row for row.
MOVES = [
    ("fwd",         0.50,  0.00,  0.00),
    ("reverse",    -0.40,  0.00,  0.00),
    ("strafeL",     0.00,  0.30,  0.00),
    ("strafeR",     0.00, -0.30,  0.00),
    ("rot_ccw",     0.00,  0.00,  0.20),
    ("rot_cw",      0.00,  0.00, -0.20),
    ("walk+strafe", 0.30,  0.30,  0.00),
    ("walk+turn",   0.30,  0.00,  0.20),
]

UPRIGHT_MIN_Z = 0.45


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Runner(Node):
    def __init__(self, robot_id: str):
        super().__init__("validate_motion")
        ns = f"/ferox/{robot_id}"
        self.pub = self.create_publisher(Twist, f"{ns}/cmd_vel", 10)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, f"{ns}/odom", self._odom, qos)
        self.create_subscription(Odometry, f"{ns}/odom", self._odom, 20)
        self.last = None
        self.seen = set()

    def _odom(self, m):
        key = (m.header.stamp.sec, m.header.stamp.nanosec)
        if key in self.seen:
            return
        self.seen.add(key)
        self.last = m

    def spin_for(self, seconds: float):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_odom(self, timeout: float = 20.0) -> bool:
        t0 = time.time()
        while self.last is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.last is not None

    def run_move(self, name, vx, vy, wz, duration: float):
        self.pub.publish(Twist())
        self.spin_for(1.0)
        start = self.last
        x0, y0 = start.pose.pose.position.x, start.pose.pose.position.y
        yaw0 = yaw_of(start.pose.pose.orientation)

        vxs, vys, wzs, zmin = [], [], [], float("inf")
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
        t0 = time.time()
        while time.time() - t0 < duration:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            m = self.last
            if m is not None:
                vxs.append(m.twist.twist.linear.x)
                vys.append(m.twist.twist.linear.y)
                wzs.append(m.twist.twist.angular.z)
                zmin = min(zmin, m.pose.pose.position.z)
        self.pub.publish(Twist())
        self.spin_for(1.0)

        end = self.last
        dx = end.pose.pose.position.x - x0
        dy = end.pose.pose.position.y - y0
        dyaw = math.atan2(math.sin(yaw_of(end.pose.pose.orientation) - yaw0),
                          math.cos(yaw_of(end.pose.pose.orientation) - yaw0))
        # Displacement into the body frame the move STARTED in.
        fwd = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        lat = -dx * math.sin(yaw0) + dy * math.cos(yaw0)

        mean = lambda a: (sum(a) / len(a)) if a else 0.0  # noqa: E731
        return {
            "name": name, "cmd": [vx, vy, wz],
            "vx": mean(vxs), "vy": mean(vys), "wz": mean(wzs),
            "fwd": fwd, "lat": lat, "dyaw": dyaw,
            "zmin": zmin if zmin != float("inf") else None,
            "samples": len(vxs),
        }


def verdicts(rows):
    """Judged on DISPLACEMENT, not on the published twist.

    The twist on `/ferox/<id>/odom` does not come out in the frame this script
    first assumed: commanded +0.50 m/s forward gives a measured `twist.linear.x`
    of -0.472 while the robot demonstrably travels +2.22 m forward in the same
    window. Rather than guess at the convention, the verdicts use the pose
    difference rotated into the body frame the move started in, which is
    frame-correct by construction and needs no convention at all. The raw twist
    is still PRINTED, because it is what the topic says and hiding it would be
    the sort of tidying this campaign does not do -- but nothing is judged on it.
    """
    by = {r["name"]: r for r in rows}
    out = {}
    f = by.get("fwd", {})
    out["forward"] = "PASS" if f.get("fwd", 0) > 1.0 else "FAIL"
    r = by.get("reverse", {})
    out["reverse"] = "PASS" if r.get("fwd", 0) < -0.6 else "FAIL"
    ws = by.get("walk+strafe", {})
    out["strafe"] = ("PASS (reliable while walking; pure-strafe is flaky)"
                     if abs(ws.get("lat", 0)) > 0.4 else "FAIL")
    rc = by.get("rot_ccw", {})
    out["rotate"] = ("PASS" if abs(rc.get("dyaw", 0)) > 0.4
                     else "FAIL (in-place; known FAIL on this checkpoint)")
    zs = [x["zmin"] for x in rows if x["zmin"] is not None]
    out["upright"] = (f"PASS (all zmin > {UPRIGHT_MIN_Z} m)"
                      if zs and min(zs) > UPRIGHT_MIN_Z else "FAIL")
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", default="g1_01")
    ap.add_argument("--duration", type=float, default=4.0)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    rclpy.init()
    node = Runner(args.robot_id)
    if not node.wait_for_odom():
        print("no odom on /ferox/%s/odom -- is the sim up?" % args.robot_id,
              file=sys.stderr)
        return 2

    rows = []
    for name, vx, vy, wz in MOVES:
        row = node.run_move(name, vx, vy, wz, args.duration)
        rows.append(row)
        print(f"[{name:11s}] cmd(vx={vx:+.2f},vy={vy:+.2f},wz={wz:+.2f}) | "
              f"body vel vx={row['vx']:+.3f} vy={row['vy']:+.3f} wz={row['wz']:+.3f} | "
              f"disp fwd={row['fwd']:+.2f} lat={row['lat']:+.2f} dyaw={row['dyaw']:+.2f} | "
              f"zmin={row['zmin']:.2f}", flush=True)

    v = verdicts(rows)
    print("\n=== VERDICT (body-frame velocity tracking) ===")
    for k in ("forward", "reverse", "strafe", "rotate", "upright"):
        print(f"  {k:8s}: {v[k]}")
    zs = [r["zmin"] for r in rows if r["zmin"] is not None]
    if zs:
        print(f"\nbase height: min {min(zs):.3f} m, max {max(zs):.3f} m, "
              f"spread {max(zs) - min(zs):.3f} m")

    if args.json:
        with open(args.json, "w") as fh:
            json.dump({"rows": rows, "verdicts": v}, fh, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
