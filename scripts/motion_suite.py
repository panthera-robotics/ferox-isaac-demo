#!/usr/bin/env python3
"""MM1 locomotion suite: 8 directions x 3 speeds, rotate, turn-while-walk, stop.

Extends validate_motion.py rather than replacing it. Three things it does that
the DT-era script did not, each because the DT-era script got caught by it:

  * YAW IS ACCUMULATED WITH UNWRAPPING. Endpoint atan2 wraps at +-pi and turns a
    -3.600 rad turn into +2.683 -- a wrong-direction turn at 35% where the truth
    is a correct-direction turn at 47%.
  * FALLS ARE DISQUALIFIED, NOT AVERAGED. A humanoid losing balance rotates and
    translates as it goes down, which reads as excellent tracking. Base height and
    tilt are recorded and any run outside the band is marked FELL.
  * DISPLACEMENT, NOT THE PUBLISHED TWIST. /odom's twist is in a frame that does
    not match the displacement (commanded +0.50 m/s forward reads as
    twist.linear.x = -0.472 while the robot travels +2.22 m forward). Verdicts use
    the pose difference rotated into the body frame the move started in.

NOTHING ELSE MAY BE PUBLISHING ON cmd_vel. Nav2 running with a stale goal put a
constant ang=-0.40 on the topic and silently contaminated an entire yaw
investigation. The suite refuses to start if it sees another publisher.

  python3 motion_suite.py --robot-id g1_01 [--duration 60] [--json OUT] [--md OUT]
"""
from __future__ import annotations
import argparse, json, math, statistics, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

UPRIGHT_MIN_Z, TILT_MAX_DEG = 0.55, 25.0
SPEEDS = (0.2, 0.5, 0.8)
# +x forward, +y left. E is the robot's right, so E = (0, -1).
# NE was written (1, 1) here, which is NW -- the same test run twice under two
# names, and no north-east row at all. Caught by reading the table, not by the run.
DIRECTIONS = [("N", 1, 0), ("NE", 1, -1), ("E", 0, -1), ("SE", -1, -1),
              ("S", -1, 0), ("SW", -1, 1), ("W", 0, 1), ("NW", 1, 1)]
YAW_RATES = (0.3, 0.6, 1.0, -0.3, -0.6, -1.0)


def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def rp_of(q):
    roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
    return roll, math.asin(max(-1.0, min(1.0, 2*(q.w*q.y-q.z*q.x))))


class Suite(Node):
    def __init__(s, rid):
        super().__init__("motion_suite")
        ns = f"/ferox/{rid}"
        s.topic = f"{ns}/cmd_vel"
        s.pub = s.create_publisher(Twist, s.topic, 10)
        s.reset_pub = s.create_publisher(Empty, "/twin/reset", 10)
        q = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        s.create_subscription(Odometry, f"{ns}/odom", s._cb, q)
        s.create_subscription(Odometry, f"{ns}/odom", s._cb, 20)
        s.last, s.seen = None, set()

    def _cb(s, m):
        k = (m.header.stamp.sec, m.header.stamp.nanosec)
        if k in s.seen: return
        s.seen.add(k); s.last = m

    def spin(s, t):
        t0 = time.time()
        while time.time()-t0 < t: rclpy.spin_once(s, timeout_sec=0.05)

    def other_publishers(s):
        return max(0, s.count_publishers(s.topic) - 1)

    def reset(s, timeout=25.0):
        """Put the robot back on its feet and wait until it is actually up.

        Every test starts from a re-spawn. Without this the first fall poisons
        every later row: the no-reset run in
        docs/mm/evidence/MM1/motion_suite_noreset_invalid.txt reports lateral and
        diagonal "speeds" for a robot lying on the floor, and one row has zmin
        1.395 m, which is the base being dragged upward by the re-spawn of the
        NEXT test. Returns False if the robot never comes back upright, and the
        row is then marked no_reset rather than quietly measured anyway.
        """
        s.pub.publish(Twist())
        s.reset_pub.publish(Empty())
        t0 = time.time()
        settled = 0
        while time.time() - t0 < timeout:
            rclpy.spin_once(s, timeout_sec=0.05)
            m = s.last
            if m is None:
                continue
            r, p = rp_of(m.pose.pose.orientation)
            up = (m.pose.pose.position.z >= UPRIGHT_MIN_Z
                  and math.degrees(math.hypot(r, p)) <= TILT_MAX_DEG)
            settled = settled + 1 if up else 0
            if settled >= 40:          # ~2 s of continuously upright odom
                s.spin(1.0)
                return True
        return False

    def move(s, name, vx, vy, wz, dur):
        if not s.reset():
            return {"name": name, "cmd": [vx, vy, wz], "no_reset": True, "fell": True,
                    "sim_dt": None, "fwd": None, "lat": None, "dyaw": None,
                    "speed_cmd": math.hypot(vx, vy), "speed_meas": None,
                    "speed_err_pct": None, "yaw_rate_meas": None, "yaw_err_abs": None,
                    "zmin": None, "tilt_max_deg": None}
        s.pub.publish(Twist()); s.spin(2.0)
        a = s.last
        x0, y0 = a.pose.pose.position.x, a.pose.pose.position.y
        yaw0 = yaw_of(a.pose.pose.orientation); prev = yaw0
        t0 = a.header.stamp.sec + a.header.stamp.nanosec*1e-9
        acc = 0.0; zmin = a.pose.pose.position.z; tilt = 0.0
        c = Twist(); c.linear.x, c.linear.y, c.angular.z = vx, vy, wz
        t = time.time()
        while time.time()-t < dur:
            s.pub.publish(c); rclpy.spin_once(s, timeout_sec=0.05)
            m = s.last
            if m is None: continue
            yv = yaw_of(m.pose.pose.orientation)
            acc += math.atan2(math.sin(yv-prev), math.cos(yv-prev)); prev = yv
            zmin = min(zmin, m.pose.pose.position.z)
            r, p = rp_of(m.pose.pose.orientation)
            tilt = max(tilt, math.degrees(math.hypot(r, p)))
        s.pub.publish(Twist()); s.spin(1.5)
        b = s.last
        yv = yaw_of(b.pose.pose.orientation)
        acc += math.atan2(math.sin(yv-prev), math.cos(yv-prev))
        t1 = b.header.stamp.sec + b.header.stamp.nanosec*1e-9
        dt = max(1e-3, t1-t0)
        dx = b.pose.pose.position.x - x0; dy = b.pose.pose.position.y - y0
        fwd = dx*math.cos(yaw0) + dy*math.sin(yaw0)
        lat = -dx*math.sin(yaw0) + dy*math.cos(yaw0)
        speed_cmd = math.hypot(vx, vy)
        speed_meas = math.hypot(fwd, lat)/dt
        return {"name": name, "cmd": [vx, vy, wz], "sim_dt": dt,
                "fwd": fwd, "lat": lat, "dyaw": acc,
                "speed_cmd": speed_cmd, "speed_meas": speed_meas,
                "speed_err_pct": (100*abs(speed_meas-speed_cmd)/speed_cmd) if speed_cmd else None,
                "yaw_rate_meas": acc/dt,
                "yaw_err_abs": abs(acc/dt - wz) if wz else None,
                "zmin": zmin, "tilt_max_deg": tilt,
                "no_reset": False,
                "fell": (zmin < UPRIGHT_MIN_Z) or (tilt > TILT_MAX_DEG)}


def fmt(r):
    tag = "NO-RESET" if r.get("no_reset") else ("FELL" if r["fell"] else "ok  ")
    out = f"{r['name']:13s} {tag:8s}"
    if r.get("no_reset"):
        return out + " robot never came back upright"
    if r["speed_cmd"]:
        out += f" speed {r['speed_meas']:.3f}/{r['speed_cmd']:.2f} m/s err {r['speed_err_pct']:.1f}%"
    if r["cmd"][2]:
        out += f" yaw {r['yaw_rate_meas']:+.4f}/{r['cmd'][2]:+.2f} rad/s"
    return out + f"  zmin {r['zmin']:.3f} tilt {r['tilt_max_deg']:.1f}deg"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", default="g1_01")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--json", default="/tmp/motion_suite.json")
    ap.add_argument("--md", default="/tmp/motion_suite.md")
    ap.add_argument("--allow-other-publishers", action="store_true")
    a = ap.parse_args()

    rclpy.init(); n = Suite(a.robot_id)
    t0 = time.time()
    while n.last is None and time.time()-t0 < 25: rclpy.spin_once(n, timeout_sec=0.2)
    if n.last is None:
        print("no odom -- is the sim up?", file=sys.stderr); return 2
    n.spin(2.0)
    others = n.other_publishers()
    if others and not a.allow_other_publishers:
        print(f"REFUSING TO RUN: {others} other publisher(s) on {n.topic}.\n"
              f"Nav2 with a stale goal put a constant ang=-0.40 there once and\n"
              f"contaminated an entire investigation. Stop the nav stack, or pass\n"
              f"--allow-other-publishers if you really mean it.", file=sys.stderr)
        return 3

    rows = []
    for label, sx, sy in DIRECTIONS:
        for sp in SPEEDS:
            norm = math.hypot(sx, sy) or 1.0
            rows.append(n.move(f"{label}@{sp}", sp*sx/norm, sp*sy/norm, 0.0, a.duration))
            print("  " + fmt(rows[-1]), flush=True)
    for wz in YAW_RATES:
        rows.append(n.move(f"rot{wz:+.1f}", 0.0, 0.0, wz, a.duration))
        r = rows[-1]
        print("  " + fmt(r), flush=True)
    for wz in (0.5, -0.5):
        rows.append(n.move(f"walk+turn{wz:+.1f}", 0.4, 0.0, wz, a.duration))
        print("  " + fmt(rows[-1]), flush=True)
    rows.append(n.move("stop_from_0.8", 0.8, 0.0, 0.0, a.duration))

    lines = ["| test | cmd | measured | error | zmin | tilt | verdict |", "|---|---|---|---|---|---|---|"]
    for r in rows:
        if r["cmd"][2]:
            cmd = f"wz {r['cmd'][2]:+.2f}"; meas = f"{r['yaw_rate_meas']:+.4f} rad/s"
            err = f"{r['yaw_err_abs']:.3f} rad/s"; ok = (r['yaw_err_abs'] <= 0.1) and not r['fell']
        else:
            cmd = f"{r['speed_cmd']:.2f} m/s"; meas = f"{r['speed_meas']:.3f} m/s"
            err = f"{r['speed_err_pct']:.1f} %"; ok = (r['speed_err_pct'] <= 20) and not r['fell']
        lines.append(f"| {r['name']} | {cmd} | {meas} | {err} | {r['zmin']:.3f} | "
                     f"{r['tilt_max_deg']:.1f}° | {'PASS' if ok else 'FAIL'} |")
    md = "\n".join(lines)
    open(a.md, "w").write(md + "\n")
    json.dump({"rows": rows, "duration_s": a.duration}, open(a.json, "w"), indent=2)
    print("\n" + md)
    print(f"\nfalls: {sum(1 for r in rows if r['fell'])} of {len(rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
