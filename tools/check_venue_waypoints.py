#!/usr/bin/env python3
"""Check a Ferox venue's waypoints against the world that was actually built.

Stdlib + yaml only: no Isaac, no ROS, no GPU. It reads the builder's own
objects.json rather than re-deriving the layout, so the two cannot drift.

WHY THIS EXISTS
    The first draft of venues/panthera_lab.yaml put `home` inside the shelf's
    footprint and `door_outside` on a patch of world with no floor under it.
    Neither is visible by reading the YAML, and both would have surfaced as a
    confusing nav failure hours later. Coordinates written by hand against a
    world built by a script get checked by a script.

WHAT IT CHECKS, per waypoint
    * it is over floor (room slab or the outside apron)
    * it is not inside any furniture footprint
    * it clears every obstacle and wall by at least the Nav2 inflation radius,
      because a goal inside the inflated band is one Nav2 will not plan to --
      that was the MM0 nav failure, and MM1 raised the radius to 0.55 m.
"""
from __future__ import annotations

import argparse
import json
import math
import sys

import yaml

# MM1's local_costmap inflation_radius. A goal closer than this to an obstacle
# sits in the inflated band and never becomes a valid pose.
INFLATION_R = 0.55


def _foot(cfg):
    return (cfg["x"] - cfg["sx"] / 2, cfg["x"] + cfg["sx"] / 2,
            cfg["y"] - cfg["sy"] / 2, cfg["y"] + cfg["sy"] / 2)


def _dist_to_box(x, y, box):
    x0, x1, y0, y1 = box
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True, help="objects.json from build_lab_world.py")
    ap.add_argument("--venue", required=True, help="the venue YAML")
    ap.add_argument("--report", default="")
    a = ap.parse_args()

    w = json.load(open(a.world))
    v = yaml.safe_load(open(a.venue))
    rx, ry, _ = w["room_m"]
    hx, hy = rx / 2.0, ry / 2.0

    furniture = {n: _foot(w[n]) for n in ("table", "counter", "shelf")}
    apron = w.get("apron")

    lines, fails = [], []

    def say(ok, msg):
        line = ("  ok   " if ok else "  FAIL ") + msg
        print(line)
        lines.append(line)
        if not ok:
            fails.append(msg)

    hdr = (f"venue {v['venue_id']} vs {a.world}\n"
           f"room interior x [{-hx:.2f}, {hx:.2f}] y [{-hy:.2f}, {hy:.2f}], "
           f"inflation {INFLATION_R} m")
    print(hdr)
    lines.append(hdr)

    for wp in v["waypoints"]:
        x, y = wp["pose"]["x"], wp["pose"]["y"]
        name = wp["name"]

        in_room = (-hx <= x <= hx) and (-hy <= y <= hy)
        on_apron = bool(apron) and (apron["x0"] <= x <= apron["x1"]
                                    and apron["y0"] <= y <= apron["y1"])
        say(in_room or on_apron,
            f"{name} ({x:.2f}, {y:.2f}) is over floor "
            f"({'room' if in_room else 'apron' if on_apron else 'NOTHING'})")

        for fn, box in furniture.items():
            d = _dist_to_box(x, y, box)
            inside = d == 0.0
            say(not inside, f"{name} is not inside {fn}")
            if not inside:
                say(d >= INFLATION_R,
                    f"{name} clears {fn} by {d:.2f} m (need {INFLATION_R})")

        if in_room:
            wall = min(hx - abs(x), hy - abs(y))
            # A waypoint in the doorway is meant to be near the north wall.
            doorish = "door" in name
            say(doorish or wall >= INFLATION_R,
                f"{name} clears the nearest wall by {wall:.2f} m"
                + (" (door waypoint, wall clearance not required)" if doorish else ""))

    tail = f"{'PASS' if not fails else 'FAIL'}: {len(fails)} failed check(s)"
    print("\n" + tail)
    lines += ["", tail]
    if a.report:
        open(a.report, "w").write("\n".join(lines) + "\n")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
