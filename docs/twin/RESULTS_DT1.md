# RESULTS_DT1 — contract files + audit tool

Host: Vast.ai VM (KVM), RTX 4090 49140 MiB, driver 580.105.08
Date: 2026-08-18
Verdict: **PASS**

The gate's criterion — *"audit runs against the DT0 baseline sim and correctly reports it as
non-conformant"* — is met for **both** robots, and the contracts additionally pass a
Class-A cross-check against the robot's own recorded evidence.

---

## Scorecard

| # | Requirement | Status | Evidence |
|---|---|---|---|
| 1 | `isaac/twin/g1_contract.yaml` populated with provenance | PASS — 11 topics, 7 TF edges, 4 sensors, full p2l | `isaac/twin/g1_contract.yaml` |
| 1 | `isaac/twin/go2_contract.yaml` populated with provenance | PASS — 7 topics, 5 TF edges, 4 sensors, full p2l | `isaac/twin/go2_contract.yaml` |
| 1 | `docs/twin/TWIN_DEVIATIONS.md` skeleton | PASS — structure + 5 open Class-C entries | `docs/twin/TWIN_DEVIATIONS.md` |
| 2 | `tools/twin_audit.py`, live ROS 2 graph | PASS | `evidence/DT1/audit_live_{g1,go2}_baseline.txt` |
| 2 | …or a rosbag | PASS — `--bag` implemented (rosbag2_py present in the nav image); untested, no bag exists yet (OQ-4) | `tools/twin_sources.py` `observe_bag` |
| 2 | table: topic/type/frame/rate/QoS/encoding/present/match | PASS | audit output |
| 2 | TF static edge diff ≤1e-4 m / ≤1e-4 rad | PASS | `check_tf_static` |
| 2 | camera_info K diff | PASS — 1 % tolerance | `check_payloads` |
| 2 | LaserScan geometry incl. fraction below `range_min` | PASS — sim reports 27.94 % below | audit output |
| 2 | PointCloud2 field layout diff | PASS | `check_payloads` |
| 2 | exit non-zero on any Class-A mismatch | PASS — exit 1 on both robots, exit 0 on the evidence cross-check | `echo $?` |
| 2 | `--against-evidence` mode | PASS — parses 5 evidence formats incl. TF values | `evidence/DT1/audit_against_evidence_g1.txt` |
| 3 | Unit tests: contract ↔ driver constants | PASS — 29/29 | `tools/tests/test_twin_contract.py` |
| 3 | Unit tests: contract schema | PASS — 8 rejection tests | same |
| — | **Decision 2**: delete the sim's static `map→odom`, both modes | PASS — `/tf_static` 11→10 edges; `slam_toolbox` sole writer on `/tf`; RViz Global Status Ok; Nav2 goal still SUCCEEDED | `evidence/DT1/map_odom_ownership.txt`, `rviz_tf_after_fix.png`, `nav_goal_after_map_odom_fix.txt` |
| — | **Decision 1**: record the 60 mm height gap as Class C | PASS — with your numbers | `TWIN_DEVIATIONS.md` C-1 |
| — | **Decision 4**: KDE autolock in RESULTS_DT0 "Reproduce" | PASS | `RESULTS_DT0.md` |
| — | Push DT0 + clone the two ref repos | PASS — token used once, no residue | §Token below |

---

## Audit results

```
G1  live sim:  20 pass, 32 Class-A FAIL,  6 Class-B fail,  6 skipped, 64 checks  -> exit 1
GO2 live sim:   8 pass, 22 Class-A FAIL,  6 Class-B fail,  8 skipped, 44 checks  -> exit 1
G1  evidence:  32 pass,  0 Class-A FAIL,  1 Class-B fail, 43 skipped, 76 checks  -> exit 0
```

The evidence run is the important one: it audits **the contract against the robot's own
recorded truth** rather than the sim against the contract. Zero Class-A differences means the
contract was transcribed from the driver repos correctly — all 7 G1 `/tf_static` edges match
Session A's set, and the TF **values** printed in `session_a/30_evidence.txt` match the
contract to print precision. The one Class-B miss is `/livox/imu`, which the robot publishes
but no capture in the repo covers.

Representative Class-A failures against the baseline sim (both robots):

```
laserscan/ray_count        /scan   723        -> 3200
laserscan/angle_increment  /scan   0.0087     -> 0.0019635
laserscan/range_min        /scan   0.3        -> 0.05
laserscan/range_max        /scan   6          -> 30
topic/frame_id             /scan   base_link  -> laser
tf_static/edge             livox_frame -> livox_imu       present -> MISSING
tf_static/edge             base_link -> livox_frame     present -> MISSING
tf_static/extra            base -> velodyne_base_link   not in contract -> PUBLISHED ANYWAY
camera_info/K              worst 62.50% at K[2]         (tolerance 1%)
```

---

## What changed

| File | One line |
|---|---|
| `isaac/twin/g1_contract.yaml` | G1 contract: sensors, TF, p2l, topics — every value with provenance + `repo/path:line` (new) |
| `isaac/twin/go2_contract.yaml` | Go2 contract, same shape (new) |
| `tools/twin_contract.py` | loader + validator: provenance enum, one-parent-per-frame, cycle check, over-determined LaserScan geometry, quaternion helpers (new) |
| `tools/twin_sources.py` | three observation sources — live graph, rosbag, driver evidence — behind one shape (new) |
| `tools/twin_audit.py` | comparison engine, findings table, Class-A exit code (new) |
| `tools/tests/test_twin_contract.py` | 29 tests: 8 schema-rejection, 21 drift tripwires (new) |
| `scripts/07_twin_audit.sh` | stages `tools/` + contract into `ferox_nav` and runs the audit there (new) |
| `isaac/sim_utils.py` | **removed the static `map→odom` publisher** (decision 2) |
| `docs/twin/TWIN_DEVIATIONS.md` | C-1 … C-5 (new) |
| `docs/twin/RESULTS_DT0.md` | KDE autolock added to Reproduce (decision 4) |

---

## Findings that contradict the campaign brief

Per §3 *"if a number disagrees with a file you read, the file wins and you report the
discrepancy"*. These are corrections to the brief, already applied to the contracts.

| # | Brief says | The files say | Why it matters |
|---|---|---|---|
| **F-1** | Go2 publishes `/ferox/go2_01/scan`, `/ferox/go2_01/odom` | Go2 publishes **`/scan` and `/odom` at ROOT**. `go2_driver_hw.launch.py:5-6`: *"No namespace push here."* Ferox depends on it in both directions — `go2_nav2.yaml:43-47` subscribes absolute `/scan` and warns a relative name "resolves to `/ferox/<robot_id>/scan`, which nobody publishes → AMCL silently starves"; `g1_nav2.yaml:19-27` warns that copying `/scan` from the Go2 file is "the mistake this header prevents" | DT5 would have built a Go2 twin publishing into a namespace nothing reads. The two robots genuinely differ and must not be harmonised. |
| **F-2** | Go2 publishes `/ferox/go2_01/imu/data` | **No IMU republisher exists.** The package ships `clock_util`, `cloud_accumulator`, `cmd_vel_to_sport`, `odom_tf_bridge`, `rate_limiter`, `sport_watchdog` — no IMU node. `/utlidar/imu` is consumed only as the clock-estimator feed | The twin must not invent a topic the robot does not publish. |
| **F-3** | G1 `/ferox/g1_01/odom` ~51.4 Hz; §4.3 asks for 50 Hz odom | **26.20 Hz** measured on that exact topic by a QoS-matched subscriber (`session_a/30_evidence.txt`: 524 msgs/20 s, gap median 38.54 ms). 51.4 Hz is the *source* topic via `ros2 topic hz`, and the collector's own header says "QoS-matched rclpy subscribers, **never** `ros2 topic hz`". The driver does not decimate (`odom_publish_rate_hz` default 0.0). A gap **median** of 38.54 ms rules out a startup gap | See OQ-3 — the one open item where I did not simply follow the file. |
| **F-4** | Go2 `/tf_static` = 3 edges | **5 edges.** Also `base_link→utlidar_imu` (ungated) and `base_link→robot_center` (gated on `publish_robot_center_alias`, default **true**) | Two missing edges = two Class-A audit failures at DT5. |
| **F-5** | Mid-360 at 10 Hz | G1 10 Hz, **Go2 20 Hz** (`publish_freq:=20.0`, measured 19.8–19.9) | Building the Go2 at 10 Hz halves the real costmap update rate. |
| **F-6** | DT2: publish `base_link→camera_link` "matching the driver's optional edge" | The driver ships it **disabled** (`camera_tf_enable: false`) and it is absent from Session A. `camera_link` is an orphan root on the real robot | The twin reproduces the orphan. |
| **F-7** | G1 p2l `angle_min -π, angle_max π` | Correct for the G1 (`-math.pi`), but the **Go2 uses truncated `-3.14159`** | Both yield 723 rays; recorded as written rather than tidied. |

Confirmed exactly as briefed: every G1 calibrated extrinsic (lidar mount, standing composite,
waist offsets, dog_imu residual, livox_imu datasheet offset, D435i URDF pose), the Go2 Mid-360
mount and L1 placeholders, and both p2l slices.

---

## Deviations (Class C)

| ID | Subject |
|---|---|
| C-1 | G1 standing height 0.791 m sim vs 0.731 m real (~60 mm); p2l stays Class A per your decision |
| C-2 | Mid-360 `PointCloud2` field layout unrecorded for both live streams; contracts declare the captured retired-stream layout (`point_step 22`) |
| C-3 | D435i `camera_info` K/D are factory-typical placeholders |
| C-4 | RealSense intra-camera TF values unknown — edges confirmed, numbers absent from every repo and from both captures |
| C-5 | Aligned-depth metric scale (mm) is librealsense convention, never written down |

All five are in `docs/twin/TWIN_DEVIATIONS.md` with the real value, the sim value, the
consequence, and what closes them.

---

## Open questions for Mohammed

1. **OQ-1 — one camera capture closes C-3, C-4 and C-5 at once.** On G1 #1, with the RealSense
   container up:
   ```bash
   ros2 topic echo --once /ferox/g1_01/camera/color/camera_info
   ros2 topic echo --once /ferox/g1_01/camera/aligned_depth_to_color/camera_info
   ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
   ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame
   ```
   *Default taken:* factory-typical K, identity `camera_link→*_frame` translations, REP-103
   optical rotations, millimetres — all marked `assumed`, all surfaced by the audit.
2. **OQ-2 — cloud field layout (C-2).** One line per robot:
   `ros2 topic echo --once --no-arr /livox/lidar` and
   `ros2 topic echo --once --no-arr /unitree/slam_lidar/points`.
   *Default taken:* the measured `/utlidar/cloud_livox_mid360` layout (x,y,z,intensity FLOAT32;
   ring UINT16; time FLOAT32; `point_step 22`), declared as C-2.
3. **OQ-3 — G1 odom rate: 26.2 or 51.4 Hz?** This is the only place I chose against the brief.
   The contract says 26.2 because that is what a QoS-matched subscriber measured on the topic
   itself, using the method the driver repo mandates. If 26.2 was an artifact of that capture,
   say so and I will flip the contract and the tripwire test. *Default taken:* 26.2 Hz.
4. **OQ-4 — a 30 s bag from each robot** would let `--bag` be exercised for real and would
   close OQ-1/OQ-2 as a side effect. *Default taken:* `--against-evidence`, which now parses
   the topic census, rate table, QoS table, the Session-A TF set, `30_evidence.txt` (rates +
   TF values) and `13_livox_lidar.txt`.
5. **Go2 `cmd_vel` clamps** are currently sourced from Ferox `go2.yaml` (0.8/0.3/1.0), not from
   `cmd_vel_to_sport`. Flagged in the contract; I will confirm against the driver at DT5.

---

## Token handling

Used once via `GIT_CONFIG_COUNT`/`GIT_CONFIG_KEY_0`/`GIT_CONFIG_VALUE_0` so it never reached
argv, a file, a remote URL or a log; unset immediately after. Verified afterwards: no match for
the token under `~/panthera`, no `extraheader`/`Authorization` in any `.git/config`, clean
remote URLs, empty global git config, no token env vars. **Safe to revoke.**

Note for later gates: GitHub's **git transport rejects `Authorization: Bearer`** for
fine-grained PATs (the REST API accepts it, which is why the brief's snippet looks right).
`Basic base64("x-access-token:<pat>")` works.

---

## Reproduce

```bash
cd ~/panthera/ferox-isaac-demo

# contract validity + drift tripwires (no ROS, no sim)
python3 tools/tests/test_twin_contract.py

# the contract against the robot's own recorded evidence (no ROS, no sim)
python3 tools/twin_audit.py --contract isaac/twin/g1_contract.yaml \
        --against-evidence ~/panthera/ref/panthera-g1-driver/evidence

# the sim against the contract (needs the sim + ferox_nav up)
ROBOT=g1  ./scripts/01_start_sim.sh && ROBOT=g1  ./scripts/02_start_ferox.sh
ROBOT=g1  ./scripts/07_twin_audit.sh --duration 12
ROBOT=go2 ./scripts/01_start_sim.sh
ROBOT=go2 ./scripts/07_twin_audit.sh --duration 12

# map->odom ownership after the hygiene fix
docker exec ferox_nav bash -lc 'source /opt/ros/humble/setup.bash;
  source /workspace/install/setup.bash; export ROS_DOMAIN_ID=42;
  ros2 topic info /tf -v | grep -A3 "Node name"'
```
