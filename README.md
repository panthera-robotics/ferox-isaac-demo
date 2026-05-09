# ferox-isaac-demo

Run Ferox (Nav2 + SLAM + waypoints) against Isaac Sim 5.1 with the Go2 or
G1 walking policy. **Zero OM1 dependency** — the walking policy ships in
this repo as a frozen `.pt` tensor; nothing pulls or runs OM1 at runtime.

## Layout

```
ferox-isaac-demo/
├── isaac/
│   ├── run.py           Standalone Isaac Sim launcher (Go2/G1 + policy + ROS bridge)
│   ├── sim_utils.py     ROS2 helpers (rclpy cmd_vel listener / sensor publishers)
│   ├── diag_utils.py    Optional import-path diagnostic
│   ├── checkpoints/     Frozen walking policies (go2/g1 — policy.pt + params)
│   └── assets/          USDs and meshes
└── scripts/
    ├── lib/env.sh       Shared env (ROBOT, ROBOT_ID, paths, container names)
    ├── 00_bootstrap.sh  Fresh-machine setup (Isaac Sim image + Ferox build + cache)
    ├── 01_start_sim.sh  Start Isaac Sim with the walking policy
    ├── 02_start_ferox.sh Start Ferox nav (sim mode → bridges sim topics into /ferox/<id>/)
    ├── 03_teleop.sh     Manual cmd_vel sanity drive
    ├── 04_view_rviz.sh  RViz with /ferox/<id>/* displays
    ├── 05_send_goal.sh  Send Nav2 goal (or named waypoint, status, cancel)
    └── 09_stop.sh       Tear down
```

## Quick start (fresh machine)

Prereqs: Docker + nvidia-container-toolkit (for GPU), the Ferox repo
cloned to `~/panthera/Ferox/ferox`.

```bash
git clone <ferox-isaac-demo-remote> ~/panthera/ferox-isaac-demo
cd ~/panthera/ferox-isaac-demo

./scripts/00_bootstrap.sh        # one-time: pull Isaac Sim, build Ferox images, cache dirs
./scripts/01_start_sim.sh        # ~60 sec to enter walking-policy main loop
./scripts/02_start_ferox.sh      # ~30 sec for Nav2 lifecycle to come up

./scripts/04_view_rviz.sh        # optional: RViz with map/costmap/path on /ferox/go2_01/*

./scripts/05_send_goal.sh 2 0    # autonomous goal: walk to (2, 0)
./scripts/05_send_goal.sh status # diagnostic
./scripts/05_send_goal.sh cancel # stop

./scripts/09_stop.sh             # clean shutdown
```

## Pick the robot

```bash
ROBOT=g1 ROBOT_ID=g1_01 ./scripts/01_start_sim.sh
ROBOT=g1 ROBOT_ID=g1_01 ./scripts/02_start_ferox.sh
ROBOT=g1 ROBOT_ID=g1_01 ./scripts/05_send_goal.sh 2 0
```

`env.sh` exports `ROBOT`, `ROBOT_ID`, `VENUE` (empty → SLAM mapping mode).

## How the topic plumbing works

Isaac Sim publishes default-namespace topics:

```
/scan, /odom, /imu          (sensors, sim publishes)
/cmd_vel                    (sim consumes, walking policy listens)
```

Ferox runs in `/ferox/<robot_id>/` namespace. The `mode:=sim` flag in the
top-level launch activates the Ferox sim bridge
(`ferox_nav_sim/launch/isaac_bridge.launch.py`), which relays:

```
/scan       → /ferox/<id>/scan
/odom       → /ferox/<id>/odom
/imu        → /ferox/<id>/imu/data
/ferox/<id>/cmd_vel → /cmd_vel
```

So Nav2 plans inside `/ferox/<id>/...` and the sim drives transparently.

## Real hardware: same scripts, different mode

When you have a real Go2/G1 driver running on the same host, swap
`02_start_ferox.sh` for the hardware-mode invocation:

```bash
docker exec ferox_nav bash -lc \
  '/workspace/scripts/launch_robot.sh go2 hw go2_01 dso_block_a'
```

The driver container (separate repo) publishes/consumes the
`/ferox/<robot_id>/...` topics directly — no relay needed.

## Why this is OM1-free

- **Walking policy** (`isaac/checkpoints/<robot>/exported/policy.pt`) is a
  frozen TorchScript tensor. Loaded by Isaac Sim's `python.sh`. Nothing
  imports the OM1 SDK.
- **`isaac/run.py`** uses Isaac Sim's `isaacsim.core.api`, `omni.isaac.*`,
  and `rclpy` (via `sim_utils.py`). No `import om1*` anywhere.
- **`sim_utils.py`** is a small ROS2 helper for cmd_vel/sensor wiring.
  Renamed from the upstream OM1 helper, with the OM1 SDK runtime path
  removed.
- **No OM1 image** is ever pulled. Bootstrap pulls only
  `nvcr.io/nvidia/isaac-sim:5.1.0` and builds Ferox locally.
