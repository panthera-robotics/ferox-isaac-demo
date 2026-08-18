# `planner_server` segfaults when the robot is outside the costmap

**Filed from:** DT5 of the Ferox Digital Twin campaign (OQ-5.3)
**Component:** `nav2_planner` 1.1.20, ROS 2 Humble, inside the Ferox nav image
**Severity:** high — takes the whole navigation lifecycle down with it
**Ferox code changed:** none. This is a report, not a patch.

---

## Summary

With the robot positioned outside (or exactly on the boundary of) the global
costmap, `planner_server` exits with **signal 11**. The Nav2 lifecycle manager then
loses its heartbeat and shuts down `velocity_smoother`, `waypoint_follower`,
`bt_navigator` and `behavior_server`, leaving the stack unusable until it is
relaunched.

The expected behaviour is that the planner rejects the goal — `nav2_costmap_2d`
already detects the condition and logs it as a warning, so the situation is
recognised, just not survived.

## Observed

```
[planner_server-4] [WARN] [1787060562.693137076] [nav2_costmap_2d]:
    Robot is out of bounds of the costmap!
[planner_server-4] [WARN] [1787060562.693186327] [ferox.go2_01.global_costmap.global_costmap]:
    Sensor origin at (7.75, 1.85) is out of map bounds (7.75, -2.76) to (11.18, 4.07).
    The costmap cannot raytrace for it.
[planner_server-4] [WARN] ... Robot is out of bounds of the costmap!        (x3)
[ERROR] [planner_server-4]: process has died [pid 1099, exit code -11, cmd
    '/opt/ros/humble/lib/nav2_planner/planner_server --ros-args
     -r __node:=planner_server -r __ns:=/ferox/go2_01
     --params-file .../config/nav2/go2_nav2.yaml ...'].

[lifecycle_manager-9] [INFO]  Have not received a heartbeat from planner_server.
[lifecycle_manager-9] [ERROR] CRITICAL FAILURE: SERVER planner_server IS DOWN after
    not receiving a heartbeat for 4000 ms. Shutting down related nodes.
[lifecycle_manager-9] [INFO]  Deactivating velocity_smoother
[lifecycle_manager-9] [INFO]  Deactivating waypoint_follower
[lifecycle_manager-9] [INFO]  Deactivating bt_navigator
[bt_navigator-6]    [ERROR] Failed to cancel action server for compute_path_to_pose
[bt_navigator-6]    [ERROR] Failed to get result for compute_path_to_pose in node halt!
[lifecycle_manager-9] [INFO]  Deactivating behavior_server
[lifecycle_manager-9] [INFO]  Deactivating planner_server
```

Note the robot's x (**7.75**) equals the map's minimum x (**7.75**) exactly. The
robot was not far outside the map — it was *on the boundary*, which is the case a
`>=`/`>` comparison is most likely to disagree about.

## How it arose (and why it is not exotic)

Nothing unusual was being done. The Go2 twin was running SLAM Toolbox from a cold
start in a corridor. The lidar is honest at `range_max 6.0`, so the first map is
small and its origin lands **on** the robot rather than around it. Nav2 was then
launched and a goal sent. That is the ordinary "bring the stack up and drive"
sequence, and it is reachable on hardware for the same reason: a real Go2 starting
against a wall maps forward, not behind.

## Reproduction

```bash
ROBOT=go2 TWIN=1 SIM_WORLD=hospital ./scripts/01_start_sim.sh
ROBOT=go2 MODE=twin ./scripts/02_start_ferox.sh
./docs/twin/repro/planner_oob_segfault.sh 3
```

The script drives the robot past the edge of the SLAM map, sends a goal so the
planner must plan from an out-of-bounds pose, and reports whether
`planner_server` died. Exit code `0` = reproduced, `2` = not reproduced this run.

**It is intermittent, and the report is honest about that.** A later session logged

```
Robot is out of bounds of the costmap!
Sensor origin at (7.32, 0.19) is out of map bounds (1.74, -2.76) to (7.27, 4.17).
```

continuously for minutes with `planner_server` staying `active [3]` and zero deaths.
So **being out of bounds is necessary but not sufficient**; something about the
first case — most plausibly a plan request arriving while the pose is out of bounds,
since `bt_navigator` was mid-`compute_path_to_pose` when the shutdown ran — is the
other half. A clean run of the script is not evidence the bug is absent.

## What would make this diagnosable

A stack trace. The crash produced only `exit code -11` because the container has no
core pattern set and the node is not run under a debugger. Either of these would
turn the next occurrence into a filable upstream bug:

```bash
# a) core dump
docker exec -u root ferox_nav bash -lc \
  'ulimit -c unlimited; echo "/tmp/core.%e.%p" > /proc/sys/kernel/core_pattern'

# b) run the planner under gdb, via a prefix in the launch file
#    prefix=['gdb -batch -ex run -ex bt --args']
```

Neither is applied here: (a) is a host-wide sysctl and (b) is a Ferox launch change,
and this issue deliberately changes no Ferox code.

## Suggested fix (Ferox side, not applied)

`nav2_costmap_2d` already knows the robot is out of bounds — it logs it. The
planner's `computePlan` path should treat that as a goal rejection
(`ComputePathToPose` failure) rather than continuing into whatever dereferences a
cell index that does not exist. Rejecting is also the behaviour the BT expects: it
has a recovery for a failed plan, and none for a dead server.

## Related

* `docs/twin/RESULTS_DT5.md` §4 — the navigation results this came out of.
* **C-17** in `docs/twin/TWIN_DEVIATIONS.md` — the *other* reason Go2 navigation
  fails, which is separate and independently diagnosed. Do not conflate them: C-17
  makes goals abort after recoveries; this makes the planner process die.
