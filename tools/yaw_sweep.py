#!/usr/bin/env python3
"""Yaw tracking for the G1 twin -- and whether the robot was still standing.

WHY THIS TOOL EXISTS IN THIS SHAPE
----------------------------------
Two mistakes are baked into its design because both were made for real on MM1:

1. YAW MUST BE ACCUMULATED, NOT DIFFERENCED. Differencing endpoint yaw with atan2
   wraps at +-pi, so a -3.600 rad turn reads as +2.683 -- a wrong-direction turn at
   35% where the truth is a correct-direction turn at 47%. Every sample is
   unwrapped and summed instead.

2. A FALL AND A TURN LOOK IDENTICAL IF YOU ONLY WATCH YAW. A humanoid that loses
   balance rotates as it goes down, and that reads as beautiful yaw tracking. So
   base height and roll/pitch are recorded alongside, and any run whose base drops
   below UPRIGHT_MIN_Z or whose tilt exceeds TILT_MAX_DEG is reported as FELL and
   its yaw number is disqualified rather than averaged in.

Repeats matter for the same reason: one lucky row is not a result.

  python3 yaw_sweep.py --robot-id g1_01 [--repeats 3] [--duration 6] [--json OUT]
"""
from __future__ import annotations
import argparse, json, math, statistics, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

UPRIGHT_MIN_Z = 0.55      # standing pelvis is ~0.79; 0.55 is unambiguously down
TILT_MAX_DEG = 25.0


def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def rp_of(q):
    sinr = 2*(q.w*q.x+q.y*q.z); cosr = 1-2*(q.x*q.x+q.y*q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2*(q.w*q.y-q.z*q.x)))
    return roll, math.asin(sinp)


class Sweep(Node):
    def __init__(s, robot_id):
        super().__init__("yaw_sweep")
        ns = f"/ferox/{robot_id}"
        s.pub = s.create_publisher(Twist, f"{ns}/cmd_vel", 10)
        q = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        s.create_subscription(Odometry, f"{ns}/odom", s._cb, q)
        s.create_subscription(Odometry, f"{ns}/odom", s._cb, 20)
        s.last = None; s.seen = set()

    def _cb(s, m):
        k = (m.header.stamp.sec, m.header.stamp.nanosec)
        if k in s.seen: return
        s.seen.add(k); s.last = m

    def spin(s, t):
        t0 = time.time()
        while time.time()-t0 < t: rclpy.spin_once(s, timeout_sec=0.05)

    def wait(s, t=20.0):
        t0 = time.time()
        while s.last is None and time.time()-t0 < t: rclpy.spin_once(s, timeout_sec=0.2)
        return s.last is not None

    def run(s, wz, dur):
        s.pub.publish(Twist()); s.spin(2.0)
        a = s.last
        prev = yaw_of(a.pose.pose.orientation)
        t0 = a.header.stamp.sec + a.header.stamp.nanosec*1e-9
        acc = 0.0; zmin = a.pose.pose.position.z; tilt_max = 0.0
        c = Twist(); c.angular.z = float(wz)
        t = time.time()
        while time.time()-t < dur:
            s.pub.publish(c); rclpy.spin_once(s, timeout_sec=0.05)
            m = s.last
            if m is None: continue
            y = yaw_of(m.pose.pose.orientation)
            acc += math.atan2(math.sin(y-prev), math.cos(y-prev)); prev = y
            zmin = min(zmin, m.pose.pose.position.z)
            r, p = rp_of(m.pose.pose.orientation)
            tilt_max = max(tilt_max, math.degrees(math.hypot(r, p)))
        s.pub.publish(Twist()); s.spin(1.5)
        m = s.last
        y = yaw_of(m.pose.pose.orientation)
        acc += math.atan2(math.sin(y-prev), math.cos(y-prev))
        t1 = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        dt = max(1e-3, t1-t0)
        fell = (zmin < UPRIGHT_MIN_Z) or (tilt_max > TILT_MAX_DEG)
        return {"cmd_wz": wz, "dyaw_rad": acc, "sim_dt": dt, "rate": acc/dt,
                "track_pct": 100*(acc/dt)/wz if wz else 0.0,
                "zmin": zmin, "tilt_max_deg": tilt_max, "fell": fell}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", default="g1_01")
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--commands", default="0.2,0.5,1.0,-0.2,-0.5,-1.0")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    rclpy.init(); n = Sweep(a.robot_id)
    if not n.wait():
        print("no odom -- is the sim up?", file=sys.stderr); return 2

    cmds = [float(x) for x in a.commands.split(",")]
    rows, summary = [], []
    for wz in cmds:
        got = []
        for i in range(a.repeats):
            r = n.run(wz, a.duration); rows.append(r); got.append(r)
            flag = "FELL" if r["fell"] else "ok  "
            print(f"  wz={wz:+.2f} run{i+1} {flag} dyaw={r['dyaw_rad']:+7.3f} rad "
                  f"rate={r['rate']:+.4f} ({r['track_pct']:+6.1f}%) "
                  f"zmin={r['zmin']:.3f} tilt={r['tilt_max_deg']:.1f}deg", flush=True)
        ok = [g for g in got if not g["fell"]]
        if ok:
            med = statistics.median([g["track_pct"] for g in ok])
            summary.append({"cmd_wz": wz, "valid_runs": len(ok), "median_track_pct": med,
                            "median_rate": statistics.median([g["rate"] for g in ok])})
        else:
            summary.append({"cmd_wz": wz, "valid_runs": 0, "median_track_pct": None,
                            "note": "every run FELL -- yaw disqualified"})

    print("\n=== SUMMARY (falls disqualified) ===")
    for srow in summary:
        if srow["valid_runs"]:
            print(f"  wz={srow['cmd_wz']:+.2f}  {srow['valid_runs']}/{a.repeats} upright  "
                  f"median {srow['median_track_pct']:+.1f}% of commanded")
        else:
            print(f"  wz={srow['cmd_wz']:+.2f}  0/{a.repeats} upright -- {srow['note']}")
    if a.json:
        json.dump({"rows": rows, "summary": summary}, open(a.json, "w"), indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
