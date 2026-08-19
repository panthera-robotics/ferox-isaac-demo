#!/usr/bin/env python3
"""MM2: drive the twin to every panthera_lab waypoint and record the outcome.

Terminal status is read from bt_navigator's LOG, not from the action client.
`ros2 action send_goal` does not return on this stack even after bt_navigator has
logged a terminal status, and it survives `timeout -s KILL` -- established at MM0
and unchanged since. So the goal is sent from rclpy, and success is judged the way
a person would judge it: did the robot end up at the waypoint.

A goal counts as REACHED when the robot is within --tol metres of it. The Nav2
goal checker's own xy tolerance is 0.25 m (set at MM1), so --tol defaults to 0.35 m
to allow for the pose settling after the controller stops.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Runner(Node):
    def __init__(self, rid):
        super().__init__("mm2_waypoints")
        self.ns = f"/ferox/{rid}"
        self.odom = None
        self.create_subscription(Odometry, f"{self.ns}/odom",
                                 lambda m: setattr(self, "odom", m), 10)
        self.ac = ActionClient(self, NavigateToPose, f"{self.ns}/navigate_to_pose")

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def xy(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return p.x, p.y

    def go(self, name, x, y, yaw, timeout, tol):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        goal.pose = ps

        start = self.xy()
        if not self.ac.wait_for_server(timeout_sec=15.0):
            return dict(name=name, sent=False, reached=False,
                        note="navigate_to_pose action server never appeared")
        fut = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        gh = fut.result() if fut.done() else None
        accepted = bool(gh and gh.accepted)

        t0, best = time.time(), 1e9
        while time.time() - t0 < timeout:
            self.spin(0.5)
            cur = self.xy()
            if cur is None:
                continue
            d = math.hypot(cur[0] - x, cur[1] - y)
            best = min(best, d)
            if d <= tol:
                self.spin(2.0)          # let it settle, then re-measure
                cur = self.xy()
                d = math.hypot(cur[0] - x, cur[1] - y)
                return dict(name=name, sent=True, accepted=accepted, reached=d <= tol,
                            final=[round(cur[0], 3), round(cur[1], 3)],
                            goal=[x, y], err_m=round(d, 3),
                            closest_m=round(best, 3),
                            secs=round(time.time() - t0, 1),
                            start=[round(start[0], 3), round(start[1], 3)] if start else None)
        cur = self.xy() or (float("nan"), float("nan"))
        return dict(name=name, sent=True, accepted=accepted, reached=False,
                    final=[round(cur[0], 3), round(cur[1], 3)], goal=[x, y],
                    err_m=round(math.hypot(cur[0] - x, cur[1] - y), 3),
                    closest_m=round(best, 3), secs=round(time.time() - t0, 1),
                    start=[round(start[0], 3), round(start[1], 3)] if start else None,
                    note="timed out")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--venue", required=True)
    ap.add_argument("--robot-id", default="g1_01")
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--tol", type=float, default=0.35)
    ap.add_argument("--only", default="", help="comma-separated waypoint names")
    ap.add_argument("--json", default="/tmp/mm2_waypoints.json")
    a = ap.parse_args()

    v = yaml.safe_load(open(a.venue))
    wps = v["waypoints"]
    if a.only:
        keep = {s.strip() for s in a.only.split(",")}
        wps = [w for w in wps if w["name"] in keep]

    rclpy.init()
    n = Runner(a.robot_id)
    t0 = time.time()
    while n.odom is None and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.odom is None:
        print("no odom -- is the sim up?", file=sys.stderr)
        return 2

    rows = []
    for w in wps:
        p = w["pose"]
        print(f"-> {w['name']} ({p['x']}, {p['y']})", flush=True)
        r = n.go(w["name"], p["x"], p["y"], p.get("yaw", 0.0), a.timeout, a.tol)
        rows.append(r)
        print(f"   {'REACHED' if r['reached'] else 'not reached'} "
              f"err {r.get('err_m')} m closest {r.get('closest_m')} m "
              f"in {r.get('secs')} s {r.get('note','')}", flush=True)

    ok = sum(1 for r in rows if r["reached"])
    print(f"\n{ok}/{len(rows)} waypoints reached")
    json.dump({"reached": ok, "total": len(rows), "tol_m": a.tol, "rows": rows},
              open(a.json, "w"), indent=2)
    return 0 if ok == len(rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
