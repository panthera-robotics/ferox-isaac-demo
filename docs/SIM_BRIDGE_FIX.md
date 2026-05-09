# Sim Bridge / Namespace + TF Fix

Postmortem and design note for the empty-RViz / blocked-costmaps incident in
the Isaac Sim ↔ Ferox nav bring-up. Authoritative description of which
problems were fixed surgically, which were deliberately deferred, and what
the full architectural path looks like if and when we want to take it.

---

## 1. Observed symptoms

After running `01_start_sim.sh`, `02_start_ferox.sh`, `04_view_rviz.sh` in
order:

- RViz showed Grid + axes only. **No** map, scan, costmaps, plan, or
  odometry.
- The TF panel logged "Invalid frame ID 'map'" and never recovered.
- `ros2 topic list` showed `/ferox/go2_01/local_costmap/costmap` etc., but
  `ros2 topic info` reported `Publisher count: 0` for the local costmap.
- `/tmp/nav.log` did not exist inside the `ferox_nav` container.
- Manual teleop (`03_teleop.sh forward`) emitted QoS-incompatibility warnings
  from `behavior_server` and the cmd_vel relay; the robot moved
  intermittently or not at all.
- Re-running `02_start_ferox.sh` produced a partial stack — `controller_server`,
  `bt_navigator`, and `lifecycle_manager_navigation` would silently disappear
  even though the launch reported success.

---

## 2. Root causes

Five compounding bugs, ordered by blast radius.

### 2.1 Nav2 launch silently died

`02_start_ferox.sh` invoked the launch as:

```bash
docker exec -d "$NAV_CONTAINER" bash -lc "
  source /opt/ros/humble/setup.bash
  source /opt/ferox_msgs_ws/install/setup.bash
  source /workspace/install/setup.bash
  ros2 launch ferox_nav_bringup bringup.launch.py ... > /tmp/nav.log 2>&1
"
```

In bash, `> /tmp/nav.log 2>&1` only redirects the *single command it is
attached to* — the `ros2 launch` line. Each `source` ran with the default
streams, which under `docker exec -d` are `/dev/null`. **Any source-step
failure was discarded silently and `nav.log` was never created.** This
left the wait loop with nothing to grep and no way to surface the failure.

### 2.2 Re-running the script orphaned the prior Nav2 stack

`02_start_ferox.sh` had this pre-launch step:

```bash
docker exec "$NAV_CONTAINER" bash -lc 'pkill -9 -f "ros2 launch" 2>/dev/null || true'
```

`pkill -f "ros2 launch"` matches only the launch parent. The Nav2 lifecycle
nodes it spawned (`controller_server`, `planner_server`, `behavior_server`,
`bt_navigator`, `slam_toolbox`, the static_transform_publishers, the
relays, the ferox_nav helpers) were *not* matched and survived,
re-parented to PID 1.

When the next `ros2 launch` came up it tried to register the same DDS
node names (`/ferox/go2_01/controller_server`, ...). DDS doesn't allow
two participants with the same fully-qualified node name on the same
domain — one wins, the other dies. In practice the loser was usually
`controller_server`, `bt_navigator`, and `lifecycle_manager`. They went
silent and the rest of the stack stayed up looking healthy, so RViz had
costmap subscribers but no costmap publishers.

### 2.3 cmd_vel relay caused a QoS war

`isaac_bridge.launch.py` ran a `topic_tools/relay` from
`/ferox/<id>/cmd_vel` → `/cmd_vel`. Three publishers ended up on
`/ferox/<id>/cmd_vel`:

| Publisher | Reliability | Durability |
|---|---|---|
| Nav2 `velocity_smoother` | RELIABLE | VOLATILE |
| Nav2 `behavior_server` (× 5 plugins) | RELIABLE | TRANSIENT_LOCAL |
| `ros2 topic pub` (default in `03_teleop.sh`) | RELIABLE | TRANSIENT_LOCAL |

`topic_tools/relay` adapts its subscriber QoS to whatever it sees first.
With mixed durabilities it **falls back to VOLATILE and refuses to receive
from TRANSIENT_LOCAL publishers**, dropping them with the log line:

> `New publisher discovered on topic '/ferox/go2_01/cmd_vel', offering
> incompatible QoS. No messages will be sent to it. Last incompatible
> policy: DURABILITY_QOS_POLICY`

So manual teleop and behavior_server-driven recoveries silently failed to
reach the sim, depending on which publisher the relay had latched onto.

### 2.4 `09_stop.sh` couldn't find the Ferox repo

[scripts/lib/env.sh](../scripts/lib/env.sh) defaulted `FEROX_REPO` to
`$HOME/panthera/Ferox/ferox`, but the checked-out tree is flat at
`$HOME/panthera/Ferox`. `09_stop.sh` did
`( cd "$FEROX_REPO" && docker compose ... down ) >/dev/null 2>&1`, swallowed
the `cd` error, and left `ferox_nav` running. The next start saw a stale
container.

### 2.5 The lifecycle wait loop was slow and racy

The original wait loop polled `ros2 lifecycle get /ferox/<id>/<server>` for
each of 7 servers, with `timeout 1` per call, in a 60-iteration loop —
worst case ~7 minutes of `docker exec` round-trips on top of the actual
boot time. On cold daemons, `ros2 lifecycle get` returned empty even when
the server was active, so the loop frequently reported `0/7` for healthy
stacks.

---

## 3. Decision: surgical vs architectural

There were two ways to fix this:

### Option A — Surgical (chosen)

Keep the existing topology — Isaac Sim publishes sensors at the root
namespace, `topic_tools/relay` mirrors them into `/ferox/<id>/...`, Nav2
runs under that namespace. Fix the five concrete bugs above. ~6 file
edits, no container rebuild.

### Option B — Architectural

Parameterize Isaac Sim's ROS2 graph with a `--ros_namespace` argument so
every publisher (`/scan`, `/odom`, `/imu/data`, `/joint_states`, the
camera topics) emits directly under `/ferox/<robot_id>/...`. Delete
`isaac_bridge.launch.py` entirely. Touches `run.py` and `sim_utils.py`,
~12 file edits. Result: sim is indistinguishable from a real Ferox
driver.

**We took Option A.** Rationale:

- Bugs 2.1, 2.2, 2.4, 2.5 are pure script/launch reliability — they have
  no architectural component, and Option B doesn't fix any of them.
- Bug 2.3 (the cmd_vel relay) is the *only* one with an architectural
  component. The surgical fix removes the relay for cmd_vel only and lets
  Isaac Sim subscribe directly to the namespaced topic via the existing
  `--cmd_vel_topic` arg. This gives us the Option-B benefit on the most
  problematic topic (multiple publishers, mixed QoS) at zero refactor
  cost. Sensor topics have a single producer and stable QoS, so the
  relay there is harmless.
- The OmniGraph sensor publishers in `sim_utils.py` are mostly in their
  own physically-defined render pipelines; threading a namespace prefix
  through every `topicName=` and `Pub.inputs:topicName` is mechanical
  but invasive and changes a lot of working code at once.

---

## 4. What changed (Option A)

| File | What |
|---|---|
| [Ferox/src/ferox_nav_sim/launch/isaac_bridge.launch.py](../../Ferox/src/ferox_nav_sim/launch/isaac_bridge.launch.py) | Drop `isaac_relay_cmd_vel_out` and the no-op `/clock` relay. Module docstring now spells out the relay/no-relay rationale per topic class. |
| [scripts/01_start_sim.sh](../scripts/01_start_sim.sh) | Pass `--cmd_vel_topic /ferox/${ROBOT_ID}/cmd_vel` to `run.py` so Isaac Sim subscribes to the namespaced topic directly — no relay, no QoS bridging. |
| [scripts/02_start_ferox.sh](../scripts/02_start_ferox.sh) | Four fixes: (a) `exec > /tmp/nav.log 2>&1` at the top of the launch wrapper so source-step failures are logged. (b) Robust kill via base64-staged `/tmp/kill_nav.sh` that catches every orphan child by colcon install path. (c) Wait loop greps `/tmp/nav.log` for `lifecycle_manager`'s `"Managed nodes are active"` marker — deterministic, ~4s vs 60s. (d) Fast-fail if the launch process dies mid-wait with the last 30 lines of the log. |
| [scripts/03_teleop.sh](../scripts/03_teleop.sh) | Explicit `--qos-reliability reliable --qos-durability volatile` so manual teleop matches Nav2's QoS and never triggers a fallback in any subscriber. |
| [scripts/09_stop.sh](../scripts/09_stop.sh) | Surface compose-down failures, fall back to `docker stop` by container name. |
| [scripts/lib/env.sh](../scripts/lib/env.sh) | `FEROX_REPO` auto-detects flat (`Ferox/`) vs nested (`Ferox/ferox/`) layout and falls back to flat. |

---

## 5. What was deliberately not changed

These are correct as-is for the surgical scope, but worth listing so
nobody touches them by accident.

- **Sensor relays** in [Ferox/src/ferox_nav_sim/launch/isaac_bridge.launch.py](../../Ferox/src/ferox_nav_sim/launch/isaac_bridge.launch.py)
  for `/scan`, `/odom`, `/imu`. Each has exactly one publisher and one
  consistent QoS profile, so `topic_tools/relay` works correctly here.
- **Static TF bridges** in [Ferox/src/ferox_nav/launch/ferox_nav.launch.py](../../Ferox/src/ferox_nav/launch/ferox_nav.launch.py)
  (`base_link → base`, `base_link → velodyne_base_link`,
  `lidar_l1_link → laser`). The "right" home for these is inside the
  USD's TF tree (so sim publishes the chain matching what
  [Ferox/src/ferox_nav/config/robots/go2.yaml](../../Ferox/src/ferox_nav/config/robots/go2.yaml)
  declares), but moving them is invasive. The runtime bridges are
  functionally correct.
- **`/tf` and `/tf_static` global topics**. Multiple Isaac Sim
  `_StaticTFGraph_TF*` nodes publish to `/tf_static` with TRANSIENT_LOCAL
  durability; `topic_tools/relay` cannot aggregate multiple
  TRANSIENT_LOCAL publishers, so SLAM and Nav2 are remapped (in
  `ferox_nav.launch.py`) to read TF from the global topics rather than
  the namespaced ones. This is intentional and documented in the launch
  file's inline comments.
- **Topic names hardcoded in `isaac/sim_utils.py`** (`/scan`, `/odom`,
  `/imu/data`, `/joint_states`, `/tf`, `/tf_static`, the camera and
  lidar topics). Parameterizing these is the entry point to Option B
  and is out of scope here.

---

## 6. Verified post-fix state

Run from `ferox-isaac-demo/`:

```bash
./scripts/09_stop.sh
./scripts/01_start_sim.sh        # ~55s warm, ~3min cold
FEROX_REPO=/root/panthera/Ferox ./scripts/02_start_ferox.sh
FEROX_REPO=/root/panthera/Ferox ./scripts/04_view_rviz.sh
```

Expected:

- `02_start_ferox.sh` reports `✓ Nav2 fully active after 4s`.
- `ros2 node list` shows controller_server, planner_server, behavior_server,
  bt_navigator, smoother_server, velocity_smoother, waypoint_follower,
  lifecycle_manager_navigation, both costmap nodes, slam_toolbox, the
  three sim_*_tf static publishers, and the three isaac_relay_* nodes.
- `ros2 topic info` returns `Publisher count: 1` (or higher) for
  `/ferox/go2_01/{scan,odom,map,global_costmap/costmap,local_costmap/costmap}`.
- `tf2_echo` resolves all of `map → base_link`, `map → laser`,
  `odom → base_link`, `base_link → base`, `lidar_l1_link → laser`.
- `ros2 topic info /ferox/go2_01/cmd_vel` lists 6 publishers (Nav2) and
  1 subscriber (Isaac Sim's `ferox_cmd_vel_listener`), all uniformly
  RELIABLE+VOLATILE.
- RViz shows scan, both costmaps, the SLAM map, odometry markers, and a
  fully connected TF tree.
- `02_start_ferox.sh` is idempotent: re-running cleanly kills the prior
  stack via `/tmp/kill_nav.sh` and brings up a fresh one with no
  controller_server / bt_navigator / lifecycle_manager dropouts.

---

## 7. Next steps (Option B path, when ready)

If we ever want sim to look exactly like a real Ferox driver, in priority
order:

1. **Add a `--ros_namespace` argument to `run.py`**.
   [isaac/run.py](../isaac/run.py) already takes `--cmd_vel_topic`; add a
   matching namespace argument and propagate it to `RobotRosRunner`.

2. **Thread the prefix through every `topicName=` and
   `Pub.inputs:topicName` in [isaac/sim_utils.py](../isaac/sim_utils.py)**.
   Affected setup helpers (search for these by name):
   `setup_2d_lidar`, `setup_static_tf_graph`, `setup_odom_publisher`,
   `setup_ros_publishers` (camera), `setup_color_camerainfo_graph`,
   `setup_depth_camerainfo_graph`, `setup_joint_states_publisher`,
   `setup_imu_graph`, `setup_cmd_vel_graph`. Leave `/tf` and `/tf_static`
   global; they have to stay that way.

3. **Delete [Ferox/src/ferox_nav_sim/launch/isaac_bridge.launch.py](../../Ferox/src/ferox_nav_sim/launch/isaac_bridge.launch.py)
   and remove its inclusion from [Ferox/src/ferox_nav_bringup/launch/bringup.launch.py](../../Ferox/src/ferox_nav_bringup/launch/bringup.launch.py)**.
   With every sim topic emerging under `/ferox/<id>/...` directly, the
   bridge has nothing to do.

4. **Move the runtime static TF bridges (`base_link → base`,
   `base_link → velodyne_base_link`, `lidar_l1_link → laser`) into the
   USD itself**, so the sim's TF tree matches the frame names declared
   in [Ferox/src/ferox_nav/config/robots/go2.yaml](../../Ferox/src/ferox_nav/config/robots/go2.yaml)
   without runtime patchwork. Then remove the `sim_*_tf` Node
   declarations from [Ferox/src/ferox_nav/launch/ferox_nav.launch.py](../../Ferox/src/ferox_nav/launch/ferox_nav.launch.py).

5. **Replace the lifecycle wait loop in
   [scripts/02_start_ferox.sh](../scripts/02_start_ferox.sh) with a
   service call to `lifecycle_manager_navigation/is_active`** — the log
   grep is good but a service call is the documented, version-stable
   way and survives launch logging changes.

6. **Add a multi-robot smoke test**. With Option B done, two
   `01_start_sim.sh` invocations with different `ROBOT_ID`s on different
   `ROS_DOMAIN_ID`s (or a shared domain plus distinct `--ros_namespace`)
   should bring up two fully isolated Ferox stacks. Today's relay-based
   topology can't support that without bespoke per-robot bridge configs.
