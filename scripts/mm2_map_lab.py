#!/usr/bin/env python3
"""Drive panthera_lab to build a complete SLAM map, without ever turning in place.

WHY IT IS SHAPED LIKE THIS
    MM1 measured that this locomotion policy turns while WALKING (~70 % of a
    ±0.5 rad/s command) and does not turn IN PLACE at all (four of six pure
    rotations came out at 0.0000 rad/s). So every controller here keeps vx > 0
    whenever it wants yaw. A conventional "rotate to face the target, then drive"
    controller would sit still forever, which is precisely what happened to Nav2:
    its recovery behaviour is Spin, the robot cannot spin, and the behaviour
    server logged "Exceeded time allowance before reaching the Spin goal".

    The consequence for MM2 is not cosmetic. With a partial map, Nav2's global
    planner found no valid path, fell back to Spin, and could not recover -- so
    the venue has to be mapped by driving it, and the driving has to obey the
    gait's actual capability rather than the one Nav2 assumes.

    It also stops short of walls using odometry rather than trusting the scan,
    because the twin's /scan is a single Mid-360 frame (C-24) and is sparse.
"""
from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

# Interior tour of the 8 x 6 m room, kept >=1.0 m off every wall and clear of the
# table (x 1.60..2.80, y -2.00..-1.20), counter (x -3.80..-1.40, y 2.10..2.70)
# and shelf (x -4.20..-2.60, y -2.00..-1.60).
TOUR = [(2.80, 0.00), (2.80, -0.60), (0.00, -0.60), (-1.50, -0.60),
        (-1.50, 1.20), (0.50, 1.40), (2.60, 1.40), (2.00, 2.20),
        (0.00, 1.60), (-2.40, 0.60), (-2.40, -0.80), (0.00, -0.60),
        (2.00, 0.00), (-2.60, 0.00)]


class Driver(Node):
    def __init__(self, rid):
        super().__init__("mm2_map_lab")
        ns = f"/ferox/{rid}"
        self.odom = None
        self.map = None
        self.create_subscription(Odometry, f"{ns}/odom",
                                 lambda m: setattr(self, "odom", m), 10)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, f"{ns}/map",
                                 lambda m: setattr(self, "map", m), q)
        self.pub = self.create_publisher(Twist, f"{ns}/cmd_vel", 10)

    def pose(self):
        o = self.odom
        p = o.pose.pose.position
        q = o.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return p.x, p.y, yaw

    def free_cells(self):
        m = self.map
        return sum(1 for v in m.data if 0 <= v < 50) if m else -1

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive_to(self, tx, ty, tol=0.35, timeout=45.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is None:
                continue
            x, y, yaw = self.pose()
            d = math.hypot(tx - x, ty - y)
            if d <= tol:
                return True, d
            err = math.atan2(math.sin(math.atan2(ty - y, tx - x) - yaw),
                             math.cos(math.atan2(ty - y, tx - x) - yaw))
            c = Twist()
            # ALWAYS moving forward. vx never drops to zero while a heading error
            # is being corrected, because at vx=0 this policy produces no yaw.
            c.linear.x = 0.40 if abs(err) < 1.0 else 0.22
            c.angular.z = max(-0.6, min(0.6, 1.2 * err))
            self.pub.publish(c)
        x, y, _ = self.pose()
        return False, math.hypot(tx - x, ty - y)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", default="g1_01")
    ap.add_argument("--laps", type=int, default=1)
    a = ap.parse_args()

    rclpy.init()
    n = Driver(a.robot_id)
    t0 = time.time()
    while n.odom is None and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.spin(3.0)
    print(f"map free at start: {n.free_cells()}", flush=True)

    for lap in range(a.laps):
        for (tx, ty) in TOUR:
            ok, d = n.drive_to(tx, ty)
            x, y, _ = n.pose()
            print(f"  lap{lap} -> ({tx:5.2f},{ty:5.2f}) "
                  f"{'ok ' if ok else 'MISS'} at ({x:6.2f},{y:6.2f}) "
                  f"d={d:.2f} free={n.free_cells()}", flush=True)
    n.pub.publish(Twist())
    n.spin(4.0)
    print(f"map free at end: {n.free_cells()}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
