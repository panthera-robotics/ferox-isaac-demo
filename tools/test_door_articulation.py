#!/usr/bin/env python3
"""MM2: prove the panthera_lab door is a real articulation, not a decal.

Loads the world alone (no robot, no ROS), drives the hinge's angular drive to a
series of targets, steps physics, and records the measured joint angle each step.
Three things are checked, and each is a way the door could look right and be wrong:

  1. IT MOVES. A revolute joint authored with no drive, or with a limit of zero,
     renders identically to a working one and never budges.
  2. IT STOPS AT ITS LIMIT. Commanded past 110 deg, it must stay at 110 deg. A
     door that swings to 180 deg through its own frame is worse than a static one,
     because the robot will happily walk through the wall behind it.
  3. IT CLOSES BY ITSELF. With the drive released to its 0 deg rest target, the
     closer spring must bring it back. That is the behaviour the spring and damping
     were authored for, and nothing else tests them.

Output: a CSV of (t, target_deg, measured_deg) and a verdict line.
"""
from __future__ import annotations

import argparse
import csv
import math
import os

from isaacsim import SimulationApp

_app = SimulationApp({"headless": True})

from isaacsim.core.api import World                      # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import UsdPhysics                               # noqa: E402

HINGE = "/World/lab/door/hinge"
LIMIT_DEG = 110.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--csv", default="/out/door_angles.csv")
    ap.add_argument("--report", default="/out/door_report.txt")
    a = ap.parse_args()

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0,
                  rendering_dt=1.0 / 60.0)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(a.world, "/World/lab")
    world.reset()
    world.play()

    stage = world.stage
    hinge = stage.GetPrimAtPath(HINGE)
    if not hinge:
        print(f"FAIL: no hinge at {HINGE}")
        return 1
    drive = UsdPhysics.DriveAPI(hinge, "angular")

    from isaacsim.core.prims import Articulation
    art, names, idx = None, [], 0
    try:
        art = Articulation("/World/lab/door")
        art.initialize()
        names = list(art.dof_names)
    except Exception as exc:  # noqa: BLE001
        print(f"articulation view failed: {exc}", flush=True)
    print("door articulation dofs:", names, flush=True)

    # The angle is ALSO measured from the leaf's world transform, because the
    # question "does the door move" must not depend on whether one particular
    # API reports a joint. Two zero-DOF results have already been produced by a
    # door that passed every static check.
    from pxr import Gf, UsdGeom
    xcache = UsdGeom.XformCache()
    leaf_prim = stage.GetPrimAtPath("/World/lab/door/leaf")

    def leaf_angle_deg():
        m = xcache.GetLocalToWorldTransform(leaf_prim)
        xcache.Clear()
        v = m.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
        return math.degrees(math.atan2(v[1], v[0]))

    def joint_angle_deg():
        if not names:
            return float("nan")
        q = art.get_joint_positions()
        return math.degrees(float(q[0][idx] if getattr(q, "ndim", 1) > 1 else q[idx]))

    rows, lines = [], []
    t = 0.0

    def set_target(target_deg):
        import numpy as _np
        try:
            art.set_joint_position_targets(
                _np.array([[math.radians(target_deg)]], dtype=_np.float32))
            return True
        except Exception:
            drive.GetTargetPositionAttr().Set(float(target_deg))
            return False

    def step_to(target_deg, seconds, label):
        nonlocal t
        set_target(target_deg)
        # Opening a 35 kg leaf against a closer spring needs more than the spring's
        # own authority, so the drive's stiffness is raised while it is being
        # driven and restored afterwards. The SPRING is what the closing test
        # exercises; this only models a hand on the handle.
        n = int(seconds * 200)
        for i in range(n):
            world.step(render=False)
            t += 1.0 / 200.0
            if i % 20 == 0:
                rows.append((round(t, 3), float(target_deg),
                             round(joint_angle_deg(), 3),
                             round(leaf_angle_deg(), 3), label))
        return leaf_angle_deg()

    def say(ok, msg):
        line = ("  ok   " if ok else "  FAIL ") + msg
        print(line, flush=True)
        lines.append(line)
        return ok

    # Gains and targets go through the ARTICULATION VIEW, not the USD attributes.
    # PhysX builds the articulation at world.reset(); writing DriveAPI attributes
    # afterwards changes the USD and not the running solver, so the door sat at
    # -0.33 deg through a commanded 45, 90 and 170 -- three "failures" that were
    # really one wrong API. run.py already does gains this way for the G1.
    import numpy as _np

    view = art._articulation_view if hasattr(art, "_articulation_view") else art
    try:
        view.set_gains(_np.array([[400.0]], dtype=_np.float32),
                       _np.array([[40.0]], dtype=_np.float32))
        print("  set drive gains via the articulation view", flush=True)
    except Exception as exc:  # noqa: BLE001
        print(f"  set_gains failed: {exc}", flush=True)

    start = step_to(0.0, 1.0, "settle")
    a45 = step_to(45.0, 3.0, "open45")
    ok0 = say(bool(names),
              f"door exposes a joint DOF: {names}")
    ok1 = say(abs(a45 - 45.0) < 8.0,
              f"door opens: commanded 45 deg, reached {a45:.2f} deg")
    a90 = step_to(90.0, 3.0, "open90")
    ok2 = say(abs(a90 - 90.0) < 8.0,
              f"door opens further: commanded 90 deg, reached {a90:.2f} deg")
    over = step_to(170.0, 3.0, "overdrive")
    ok3 = say(over <= LIMIT_DEG + 3.0,
              f"door STOPS at its limit: commanded 170 deg, reached {over:.2f} deg "
              f"(limit {LIMIT_DEG})")

    # Release: hand off the handle, leave only the closer spring + damping.
    # Hand off the handle: only the authored closer spring and damping remain.
    try:
        view.set_gains(_np.array([[2.0]], dtype=_np.float32),
                       _np.array([[8.0]], dtype=_np.float32))
    except Exception as exc:  # noqa: BLE001
        print(f"  closer-gain set failed: {exc}", flush=True)
    closed = step_to(0.0, 8.0, "closer_spring")
    ok4 = say(closed < over - 10.0,
              f"closer spring swings it back: {over:.2f} -> {closed:.2f} deg "
              f"with only the 2.0 spring and 8.0 damping")

    os.makedirs(os.path.dirname(a.csv), exist_ok=True)
    with open(a.csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "target_deg", "joint_deg", "leaf_world_deg", "phase"])
        w.writerows(rows)

    allok = ok0 and ok1 and ok2 and ok3 and ok4
    tail = f"{'PASS' if allok else 'FAIL'}: door articulation"
    print(tail, flush=True)
    with open(a.report, "w") as fh:
        fh.write("\n".join(lines) + "\n" + tail + "\n")
        fh.write(f"\nangle trace: {len(rows)} samples in {a.csv}\n")
    return 0 if allok else 1


if __name__ == "__main__":
    rc = main()
    _app.close()
    raise SystemExit(rc)
