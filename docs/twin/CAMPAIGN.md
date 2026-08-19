# Ferox Digital Twin campaign (DT0–DT8) — Claude Code prompt

> ## STATUS — 2026-08-18
>
> **Fast path complete. Start at [`RESUME.md`](RESUME.md), not here.**
> This file is the original brief and the standing rules (§0); it is not the state.
>
> | Gate | Verdict | Tag |
> |---|---|---|
> | **DT0** | PASS-with-deviations (accepted) | `twin-DT0` |
> | **DT1** | PASS (accepted) | `twin-DT1` |
> | **DT2** | PASS-with-deviations — Class-A conformant; waist fork resolved Option A | `twin-DT2` |
> | **DT3** | PASS — Dex5-1P hands, 69 DOF one articulation, robot walks | `twin-DT3` |
> | **DT5** | Interface PASS (Class-A conformant); navigation PARTIAL (C-17) | `twin-DT5` |
> | **DT4 / DT6 / DT7 / DT8** | **deferred by Mohammed — do not start** | — |
> | Fast-path housekeeping | scorecard, self-hit report, RULE-HAND-NAME, planner issue | `twin-fastpath` |
> | G1 ground truth | OQ-2 + OQ-3 closed; C-18/19/20 opened; OQ-6 raised | `twin-gt-g1` |
> | DT7-lite | Isaac Lab cfg for the merged asset (config + tests only) | — |
> | Persistence | RESUME.md, CAPTURES.md, release assets | `twin-persist` |
>
> **Deviations:** C-1 … C-20 open, **C-21 closed**. See [`TWIN_DEVIATIONS.md`](TWIN_DEVIATIONS.md).
> **Next work item:** the Go2 ground-truth capture (OQ-5.1), which decides C-17.
> **Still open:** E-1 detection half (needs the `ferox_vision` image published),
> E-2 Go2 puck/bracket, OQ-5.2, OQ-5.3, OQ-6, C-3/C-4.
> **One-page status:** [`RESULTS_FASTPATH.md`](RESULTS_FASTPATH.md).

You are the agent for the **Ferox Digital Twin campaign**. Goal: make the Isaac Sim 5.1 G1 and Go2 in
`ferox-isaac-demo` a hardware twin of Panthera's real robots — same body, same hands (Dex5-1P default,
alternates selectable), same sensors at the same calibrated poses with the same FOV/resolution/rates, and
**the same ROS 2 / DDS interface strings the real drivers publish** — so anything developed or trained
against the sim (Ferox nav, ferox_vision, GR00T/SONIC data, Isaac Lab policies, Cosmos SDG) transfers to the
robot with minimal change. This is the fix for what happened on MuJoCo (W6): sim pixels that could not
transfer, a hand observed as Dex3 but commanded as Dex5, a 0.75-scaled hand graft, a palm link with no mass,
grasps that happened by weld. None of that gets repeated here.

You run on a **fresh Vast.ai VM with one RTX 4090 (x86_64)**. Isaac Sim does not run on the DGX Spark
(aarch64); nothing in this campaign touches the Spark or any robot. Mohammed relays gate reports.

---

## 0. Standing rules (non-negotiable)

1. **Investigate before changing. Show diffs before applying. One gate per turn**; report PASS/FAIL with
   evidence, then stop and wait unless the gate says "continue overnight".
2. **Git**: branch `mohammed/twin-campaign` in every repo you touch. Commit per gate, tag `twin-DT<n>`,
   **push immediately after tagging** — instances die with unpushed commits. **No `Co-Authored-By: Claude`
   trailers.** Mohammed owns all commits. Any PAT pasted in chat is used once and Mohammed revokes it — never
   write a PAT into a file, remote URL, `.env`, or log; use `GIT_ASKPASS`/`http.extraheader` from an env var
   and unset it.
3. **Docker immutability**: anything apt/pip-installed inside a running container is gone on the next
   `up`. Bake it into a Dockerfile in the repo. (`ros-humble-topic-tools` was the recurring canary.)
4. **DDS**: `network_mode: host`, CycloneDDS, `MaxAutoParticipantIndex=120`, `ParticipantIndex=auto`,
   `presence_required=true`, unicast pinned to `tailscale0` (or the VM's own IP), `ROS_DOMAIN_ID=42` for the
   Ferox side. `FEROX_DDS_PEERS` = **live participants only, own IP only** on this single-host VM. The
   Vast.ai tailscale IP is ephemeral — never hardcode it.
5. **Ferox seven-bug checklist** stays in force (full Nav2 plugin specs, no `stvl_layer`, `/**/` YAML
   prefixes, manual `Node` spawns, explicit `/map` remap, sim-only static TF bridges only where the USD
   has a real gap, `topic_tools/relay` for sensor fan-out only — **never** on control topics).
6. **Sim/nav robot-mismatch guard** (`/tmp/sim_robot_type`, `FEROX_SKIP_SIM_CHECK=1`) must keep working.
   Isaac Sim runs as **UID 1234**; `00_bootstrap.sh` before `01_start_sim.sh` or the viewport is black.
7. **Sed/derived artifacts get audited.** Every generated USD/YAML is followed by a diff or a scripted
   assertion; silent fallbacks to defaults are invisible (this is how FollowPath→DWB slipped through once).
8. Old containers `panthera_om1_demo`, `panthera_nav`, `cmd_vel_publisher` must never be running on
   domain 42. `docker rm -f` them if you see them.
9. **Never scale a hand, a mesh, or an offset "to make it fit".** Real dimensions or a flagged TODO.
10. Ask before any destructive action (deleting assets, force-push, dropping a variant, rewriting the
    physics layer). If a decision is not covered here, pick the option closest to "sim string == robot
    string" and flag it in the gate report.

---

## 1. Read these first (ground truth, in this order)

| Repo / path | What it gives you | Authority |
|---|---|---|
| `ferox-isaac-demo/README.md` (no `CLAUDE.md` yet — DT7 creates one), `docs/DEV_LOG.md`, `docs/NEXT_SESSION.md`, `scripts/lib/env.sh`, `scripts/01_start_sim.sh`, `isaac/run.py`, `isaac/sim_utils.py` | Current sim: how the robot/world are spawned, `_resolve_usd_path`, `SIM_WORLDS`, sensor creation (`setup_sensors_delayed`), publishers, the `/ferox/<id>/cmd_vel` subscription, the sim/nav guard, the G1 policy overlay seam | you will change this |
| `ferox-isaac-demo/isaac/assets/{g1,go2}/usd/` | Today's USDs: `<robot>.usd` + `configuration/<name>_{base,physics,sensor}.usd` (URDF-importer layers of `g1_29dof_rev_1_0` and `go2_description`) | body physics layer = **keep** |
| `ferox-g1-locomotion/CLAUDE.md`, `policy/params/deploy.yaml` | The G1 omni policy contract: 29 joints by name/order, gains, obs 480 → act 29, cmd ranges vx [-0.6,1.0] vy [-0.5,0.5] wz [-1,1] | must keep working unchanged |
| `Ferox/src/ferox_nav/config/robots/{g1,go2}.yaml`, `Ferox/src/ferox_nav/launch/ferox_nav.launch.py`, `Ferox/src/ferox_nav_sim/launch/isaac_bridge.launch.py` | What Nav2/SLAM consume: topics (relative under `/ferox/<id>/`), frame ids, ranges, `mode:=sim|hw`, `use_sim_time` policy, the relay bridge you will retire for the twin | consumer contract |
| `panthera-g1-driver/src/panthera_g1_driver/launch/g1_driver_hw.launch.py`, `config/g1_driver.yaml`, `evidence/fw2026q3/g1_29dof.urdf`, `evidence/fw2026q3/session_a/*`, `evidence/fw2026q3/{rates_host,qos_summary_host,topic_list_raw}.txt`, `evidence/phase0/lidar_geometry.json`, `doc/05-validation.md`, `FEROX_PAIRING_SPEC.md` | **The real G1 interface**: every topic, type, frame, QoS, rate, and the calibrated extrinsics (numbers in §3) | **truth for the G1 twin** |
| `panthera-go2-driver/src/panthera_go2_driver/launch/go2_driver_hw.launch.py`, `config/go2_driver.yaml`, `CLAUDE.md`, `DEV_LOG.md` | **The real Go2 interface** and Mid-360 extrinsic | **truth for the Go2 twin** |
| `realsense_driver/README.md`, `params.yaml`, `compose.yaml` | The RealSense wire: topics, encodings, frames, QoS, depth filters, resolution/rate | **truth for the camera** |
| `panthera-g1-wbc` branch `mohammed/g1wb-review-w7-docs`: `RESULTS_W2.md`, `wholebody/dex5/limits.py`, `wholebody/dex5/interface.py`, `tools/fix_dex5_hand_mass.py`, `wholebody/models/dex5_1/reference_scripts/graft_dex5_both.py`, `configs/wbc/*.yaml` | Dex5-1P facts already established: URDF-derived limits + joint order, `PASSIVE_INDICES=(4,8,12,16)`, mirrored `Roll_12`, right-URDF effort=0 defect, real mass (palm 0.685702 kg; L 1.025045 kg, R 0.978570 kg, asymmetric), palm CoM ~104 mm from wrist. **Also the anti-pattern**: `SCALE=0.75`, `DX=0.072` in the MuJoCo graft — do not port those | reference; do not copy the hack |
| Upstream (read-only): `unitreerobotics/unitree_ros` (`robots/g1_description/g1_29dof_rev_1_0.urdf`, `g1_29dof_with_hand.urdf`, `*inspire*`, `robots/dexterous_hand_description/dex5_1/Dex5-URDF-{L,R}` + meshes, `go2_description`), `unitreerobotics/unitree_sim_isaaclab` (Dex3/Dex1/Inspire DDS emulation pattern, Apache-2.0), `unitreerobotics/unitree_sdk2_python` (`unitree_hg` `HandCmd_/HandState_`, `LowCmd_/LowState_` IDL) | Meshes, URDFs, wire types | vendor truth; keep LICENSE files |
| Isaac Sim 5.1 docs inside the container (`/isaac-sim/exts/isaacsim.sensors.rtx/**/supported_lidar_configs.py`, `isaacsim.ros2.bridge` OmniGraph nodes, `isaacsim.asset.importer.urdf`), plus assets `Isaac/Sensors/Intel/RealSense/rsd455.usd` (D455 template — we need D435i values) | How to author sensors in 5.x: RTX lidar is configured through `omni:sensor:Core:*` prim attributes (custom JSON configs are not drop-in in 5.x); `omni:sensor:tickRate` must equal `scanRateBaseHz` for full-scan accumulation | tooling truth |

Read the driver launch files line by line. Comments in them are calibration provenance, not decoration.

---

## 2. What "twin" means here (parity classes)

| Class | Meaning | Examples | Acceptance |
|---|---|---|---|
| **A — exact** | Strings and structure identical to hardware | topic names, msg types, encodings, `frame_id`s, TF tree topology and static edge values, QoS reliability, robot namespace `/ferox/<robot_id>/`, joint names & order, hand wire (`rt/dex3/*`, 20-entry) | zero diffs in `twin_audit` |
| **B — modeled** | Physical property reproduced from datasheet/URDF/calibration | sensor pose, FOV, resolution, intrinsics K, rates, ranges, masses/inertias, joint limits, body meshes at 1:1 | within stated tolerance (§6) |
| **C — approximated, declared** | Known sim limits, written down, never silently | Mid-360 non-repetitive rosette (RTX rotary/solid-state proxy), tactile 12 contact zones vs 94 taxels, PhysX finger-contact grasp physics, head-shell self-hit cluster | listed in `docs/TWIN_DEVIATIONS.md` |

Non-goals: multi-robot in one stage (still deferred; do not touch TF relay design), retraining any policy,
robot hardware sessions, `rt/lowcmd` low-level emulation (DT8 stretch, sim-only), MuJoCo.

---

## 3. Numbers you must not re-derive (copy from source, cite the file)

**G1 (EDU U4, 29 DoF `g1_29dof_rev_1_0`, root `pelvis`; hardware `base_link` ≡ pelvis, no `pelvis` frame exists in the hardware TF tree — the driver relabels).**

| Item | Value | Source |
|---|---|---|
| Mid-360 mount, waist-independent, `torso_link → livox_frame` (INVERTED head mount) | roll 3.128688, pitch 0.052979, yaw 0.018520 rad; xyz (-0.0440849, -0.0185556, 0.4429585) m | `g1_driver.yaml` `waist_tf_bridge` (calibrated standing 2026-08-07, floor-plane residual 0.0027°) |
| Composite standing `base_link → livox_frame` (static mode) | roll 3.090233, pitch 0.161680, yaw 0; z 0.4995 (x,y 0) | `g1_driver_hw.launch.py` `static_lidar` |
| Waist chain offsets | waist_roll [-0.0039635, 0, 0.035], waist_pitch [0, 0, 0.019]; `/lowstate` joints 12 yaw / 13 roll / 14 pitch | same |
| URDF nominal `mid360_joint` (**rejected** by calibration: 60.6 mm, 1.5° off) | xyz (0.0002835, 0.00003, 0.40618) rpy (0, 0.04014, 0) | `evidence/fw2026q3/g1_29dof.urdf` |
| D435i, `torso_link → d435_link` (= `camera_link`, body-style axes) | xyz (0.0576235, 0.01753, 0.41987), pitch 0.8307767 rad = 47.600° down; **nominal, unverified ±60 mm/±1.5°** | URDF `d435_joint`; `g1_driver.yaml` camera block (`camera_tf_enable: false` until floor-plane check) |
| `base_link → dog_imu_link` | near-identity: -0.055° roll, -0.623° pitch (residual, standing) | launch file |
| `livox_frame → livox_imu` | Livox datasheet offset | launch file |
| Real `/tf_static` set (Session A) | `base_link→dog_imu_link`, `base_link→livox_frame`, `livox_frame→livox_imu`, `camera_link→camera_color_frame→camera_color_optical_frame`, `camera_link→camera_depth_frame→camera_depth_optical_frame` | `evidence/fw2026q3/session_a/31_tf_static.txt` |
| Cloud | `/livox/lidar` PointCloud2, frame `livox_frame`, ~9.95–10 Hz, Jetson-stamped, raw sensor frame, not self-filtered | launch + `13_livox_lidar.txt` |
| `/ferox/g1_01/scan` | LaserScan in `base_link`, ~10 Hz, 723 rays: `angle_min -π`, `angle_max π`, `angle_increment 0.0087`, `scan_time 0.1`, z-slice `min_height -0.556` `max_height 0.50` (standing pelvis 0.731 m; lidar-to-floor 1.2304 m), `range_min 0.30` (head-shell self-hit cluster at 0.10–0.20 m), `range_max 6.0` | launch + Ferox `g1.yaml` |
| `/ferox/g1_01/odom` | nav_msgs/Odometry, `odom → base_link`, ~51.4 Hz, covariance all-zero, from `/state_estimator/odom_pelvis` (child `pelvis` relabeled) | launch |
| `/ferox/g1_01/imu/data` | sensor_msgs/Imu, frame `dog_imu_link`, 100 Hz (from `/lowstate.imu_state` ~1041 Hz) | `g1_driver.yaml` `imu_republisher` |
| `/ferox/g1_01/cmd_vel` clamps | +0.4/-0.2 x, ±0.2 y, ±0.5 wz | `cmd_vel_to_loco` |
| Camera wire (`/ferox/g1_01/camera/…`) | `color/image_raw` 1280×720 `rgb8` 30 Hz RELIABLE, `frame_id camera_color_optical_frame`; `color/camera_info`; `aligned_depth_to_color/image_raw` 1280×720 `16UC1` (mm) 30 Hz RELIABLE **same stamp as color**; `aligned_depth_to_color/camera_info`; `depth/color/points` xyzrgb, frame `camera_depth_optical_frame`, SENSOR_DATA. Depth module 848×480, decimation 2 (cloud 424×240), `clip_distance 4.0` (Z), MinZ ~0.28 m, High-Accuracy preset, spatial+temporal in disparity domain | `realsense_driver/README.md`, `params.yaml` |
| Hands | Dex5-1P: 20 DoF (16 active + passive at URDF-order indices 4, 8, 12, 16), 94 tactile, wire `rt/dex3/{left,right}/{cmd,state}` `unitree_hg HandCmd_/HandState_` with 20-entry `motor_cmd/motor_state` + `press_sensor_state`, 1 kHz; masses L 1.025045 / R 0.978570 kg (Unitree URDF, asymmetric; palm 0.685702 kg); `Roll_12` mirrored L/R; right URDF effort metadata = 0 (clamp on position only) | W7 branch `limits.py`, `RESULTS_W2.md`, `interface.py` |
| Body | height 1.32 m, footprint radius 0.35 m, standing pelvis 0.731 m | Ferox `g1.yaml`, driver |

**Go2 (EDU, `go2_description`, root `base_link`).**

| Item | Value | Source |
|---|---|---|
| Mid-360 `base_link → livox_frame` | xyz (0.187, 0, 0.0803), pitch 0.2249 rad (13°) about Y (Unitree SLAM docs, IMU frame ≡ base_link decision) | `go2_driver_hw.launch.py` `livox_*` args |
| `livox_frame → livox_imu` | identity by assumption | same |
| L1 (built-in 4D lidar) `base_link → utlidar_imu` / lidar | (0.28, 0, 0.075) nominal | same |
| Cloud | `/unitree/slam_lidar/points` (Mid-360S sidecar SDK stream), frame `livox_frame`, Jetson-stamped, BEST_EFFORT, raw not self-filtered; accumulator republishes `/mid360/points_accum` | launch + `cloud_accumulator.py` |
| `/ferox/go2_01/scan` | p2l: `min_height -0.20`, `max_height 0.50`, `range_min 0.30`, `angle_increment 0.0087`; range starvation p95 ~4.5 m | launch presets |
| Odom / IMU | `/utlidar/robot_odom → /ferox/go2_01/odom` (+ `odom→base_link`), `/utlidar/imu` 250 Hz MCU-clock estimator feed, `/ferox/go2_01/imu/data` | launch |
| Front camera (built-in) | 1280×720, ~120° HFOV wide-angle, on the head, streamed by Unitree not ROS by default | Go2 docs |
| D435i on Go2 | ships with the EDU dock kit; **mount not in the driver — ask Mohammed/Brandon; default: not mounted, variant `camera=none`** | open question |

If a number above disagrees with a file you read, **the file wins** and you report the discrepancy.

---

## 4. Locked architecture decisions

**4.1 One source of truth for geometry & wire: `isaac/twin/<robot>_contract.yaml`** (new). Per robot: every
sensor (name, parent link, xyz/rpy, provenance = `calibrated|urdf_nominal|datasheet|assumed`, model
params), every published topic (name, type, frame, rate, QoS, encoding), TF static edges, p2l parameters,
namespace. The sim reads this file to (a) place sensor prims, (b) configure publishers, (c) emit
`/tf_static`. Provenance strings are mandatory. `tools/twin_audit.py` (§6) reads the same file. Values are
copied from the driver repos with a comment naming the exact file:line; a unit test asserts the calibrated
G1 lidar constants and Go2 Mid-360 constants equal the driver values (hardcode expected numbers in the test
so drift is caught either way).

**4.2 Asset structure (NVIDIA layering, deterministic build).** `tools/build_twin_assets.py` regenerates
`isaac/assets/<robot>/usd/`:
- `<robot>_base.usd` visual + collision (Unitree meshes, 1:1, materials approximating the real shell:
  matte dark grey body, white/grey covers; head shell present),
- `<robot>_physics.usd` — **do not re-import the body.** Reuse the existing tuned physics layer
  (`configuration/g1_29dof_rev_1_0_physics.usd`, `go2_description_physics.usd`) verbatim; add hand
  bodies/joints/drives only. Assert body joint names/order == today's USD (29 for G1, 12 for Go2).
- `<robot>_sensor.usd` — Xform prims for `livox_frame`, `livox_imu`, `dog_imu_link`/IMU site,
  `camera_link` + RealSense subtree (color/depth frames + optical frames with the real intra-camera
  offsets taken from the robot's own `/tf_static`), Go2 `utlidar_*`, front camera; all under the correct
  parent link (`torso_link` for the G1 head sensors) at the contract poses. Sensor visual meshes: Mid-360
  puck visible only where it is visible on the real robot (Go2 top mount: yes, with a bracket; G1: inside
  the head, hidden). No sensor "flying" anywhere.
- `<robot>.usd` composes the layers and carries **variant sets**:
  - G1 `hand` ∈ {`dex5_1p` (default), `dex3_1`, `dex1_gripper`, `inspire_ftp`, `none`}, one common wrist
    mount frame per side (`{left,right}_wrist_yaw_link` flange), each hand USD carrying its own joints,
    limits, drives, masses, collision (convex decomposition on finger meshes, self-collision between hand
    and forearm chain excluded), tactile contact-zone prims. Dex5-1P at **1:1 scale**, mount offset from
    Unitree's assembly (their G1+Dex5 adapter); if you cannot source it, mount at the flange with the same
    offset Unitree uses for Dex3 in `g1_29dof_with_hand.urdf`, and flag `assumed` in the contract.
  - Go2 `camera` ∈ {`none` (default until confirmed), `d435i_dock`}; `lidar` ∈ {`mid360_top` (default)}.
- Keep the repo diet: hand meshes are small (42 STL); big generated USDs are committed only if <60 MB total
  growth, otherwise stage them via a manifest + build step (the wbc `g1_meshes.manifest` pattern) and commit
  the manifest. Unitree meshes keep their BSD-3 LICENSE next to them.

**4.3 Sensor models.**
- **Mid-360**: RTX lidar (OmniLidar) with attributes set from the contract: rotary/360° azimuth, elevation
  −7° … +52°, near 0.1 m, far 40 m (indoor returns will look like the real p95 ~4.5 m anyway), base scan
  10 Hz (`omni:sensor:tickRate == scanRateBaseHz`), ~200 k pts/s equivalent emitters/channels,
  range accuracy ~0.02 m, small azimuth/elevation noise. Start from the Mid-360 attribute dict in
  IsaacSim GitHub discussion #183 (verify each key exists in 5.1's supported set). Publish `PointCloud2`
  with the same **field layout as the real stream** (x, y, z, intensity, plus tag/line/timestamp if the
  real `/livox/lidar` from `livox_ros_driver2` carries them — check `qos_summary_host.txt` /
  `topic_info_verbose_host.txt`; if not derivable, xyz+intensity and declare). Frame `livox_frame`,
  stamped in sim time. Publish the Livox IMU on the sidecar's real topic name too (`/livox/imu`, 200 Hz, frame `livox_imu`) —
  `fast_lio2_dockerized` will want it later.
- **G1 head occlusion**: the sensor pose is truth; if the head-shell mesh fully occludes the lidar in RTX,
  exclude the shell from lidar visibility (document the mechanism) rather than moving the sensor. The real
  self-hit cluster (0.10–0.20 m) is a Class-C deviation unless you reproduce it cheaply.
- **D435i**: two camera prims under `camera_link` at the real color/depth offsets: color 1280×720 @30 Hz,
  HFOV ≈69°/VFOV ≈42° with **K taken from a real `camera_info` capture** (ask Mohammed for one message
  from `/ferox/g1_01/camera/color/camera_info` and `aligned_depth_to_color/camera_info`; until then use
  D435i factory-typical values and mark `assumed`); depth 848×480 native @30 Hz, aligned to color and
  published as **`16UC1` millimetres** on `aligned_depth_to_color/image_raw` with the **same stamp** as the
  color frame (a small converter node in the twin bridge is fine if the ROS 2 helper only emits `32FC1`);
  `clip_distance 4.0` and MinZ 0.28 emulated (zero-out beyond/below); `depth/color/points` xyzrgb in
  `camera_depth_optical_frame`, SENSOR_DATA; `camera_info` distortion model `plumb_bob` with the real
  coefficients. Use `Isaac/Sensors/Intel/RealSense/rsd455.usd` only as a prim-structure template.
- **IMU**: 100 Hz Imu on `imu/data`, frame `dog_imu_link` (G1) / real Go2 frame per launch file.
- **Odom**: 50 Hz nav_msgs/Odometry `odom → base_link`, child `base_link` (G1: pelvis pose relabeled;
  never publish a `pelvis` frame on ROS), covariance all-zero like hardware.
- **Rates**: lidar 10 Hz ±10 %, camera 30 Hz target (≥20 Hz accepted on the 4090 with a note), IMU 100 Hz,
  odom 50 Hz. Set the render/physics dt so these hold; report measured `ros2 topic hz`.

**4.4 Interface parity replaces relays.** The sim publishes directly under `/ferox/<robot_id>/` with the
hardware topic names and frames, and publishes `/tf_static` for exactly the hardware edge set. Add
`mode:=twin` in `Ferox/src/ferox_nav/launch/ferox_nav.launch.py` (behaviour: `use_sim_time true`, no
`isaac_bridge` relays, no sim-only static TF bridges, hardware-shaped topics). Existing `mode:=sim` keeps
working until DT7 signs it off as retired for both robots. `pointcloud_to_laserscan` runs in the twin
bridge with the **driver's exact parameters** (copied into the contract with provenance; test asserts
equality with the driver launch defaults). Nav2/SLAM/`ferox_vision` must run against the twin unchanged.

**4.5 Hands speak the robot's wire (DT6).** Per hand variant, a DDS bridge process (pattern:
`unitree_sim_isaaclab`, `unitree_sdk2py`) publishes `HandState_` (20 entries for Dex5, 7 for Dex3, plus
`press_sensor_state` fed from the tactile contact zones) and consumes `HandCmd_` on `rt/dex3/*`; Inspire on
`rt/inspire/*` (`unitree_go MotorCmds_/MotorStates_`, 12 values); Dex1 gripper per its service. Joint index
order = `wholebody/dex5/limits.py` URDF-document order; passive 4/8/12/16 mirror their coupled joint;
commands are clamped to URDF position limits (never effort). Domain and interface come from env, defaults
mirror the robot (domain 0) but must be overridable to keep the VM's Ferox bus (42) clean. Success test: the
W7 `wholebody/dex5` driver with the `dds` backend talks to the sim hand unmodified.

**4.6 Isaac Lab / NVIDIA-tools readiness (DT7).** Provide `isaac/twin/isaaclab/` with `ArticulationCfg`s
for `g1_29dof_dex5_1p`, `g1_29dof_dex3_1`, `go2_mid360` pointing at the twin USDs (actuator groups mirroring
`unitree_rl_lab`'s G1/Go2 configs, hand joints in their own group), a `CameraCfg`/`TiledCameraCfg` for the
D435i from the contract, a `RayCasterCfg` Mid-360 pattern for RL and the RTX prim for SDG, and a
Replicator writer smoke (rgb/depth/normals/semantic/instance from the D435i prim → `outputs/sdg_smoke/`)
so Cosmos-Transfer / GR00T-Mimic / GR00T-Dreams pipelines can consume our sensor exactly as mounted.
Assert the 29 body joint names/order in the twin USD equal the omni policy's `deploy.yaml` map.

---

## 5. Gates

Each gate ends with `docs/twin/RESULTS_DT<n>.md` (verdict, evidence table, deviations, exact commands to
reproduce), screenshots/PNGs under `docs/twin/evidence/DT<n>/`, a commit and tag `twin-DT<n>` pushed.

### DT0 — box ready (½ day)
1. `nvidia-smi` (driver version, 24 GB), `docker`, `nvidia-container-toolkit`, disk ≥150 GB free,
   `tailscale ip -4`. If this Vast.ai instance is a container without Docker, stop and report — the demo is
   docker-first (fallback = native `pip isaacsim==5.1` venv, only with Mohammed's OK).
2. Clone `Ferox` → `~/panthera/Ferox`, `ferox-isaac-demo`, `ferox-g1-locomotion` alongside; the three
   driver repos + `realsense_driver` read-only under `~/panthera/ref/`; the wbc W7 branch under
   `~/panthera/ref/panthera-g1-wbc` (sparse: `wholebody/`, `tools/`, `RESULTS_*.md`, `configs/`).
3. `.env` at demo root: `FEROX_DDS_INTERFACE=tailscale0`, `FEROX_DDS_PEERS=<own tailscale IP>`.
4. `./scripts/00_bootstrap.sh`, `./scripts/01_start_sim.sh` (Go2, default world), then
   `ROBOT=g1 …`; `VENUE=dso_block_a ./scripts/02_start_ferox.sh`; one goal each; save
   `docker exec ferox_isaac_sim tail -100 /tmp/sim.log`, `ros2 topic list`, `ros2 topic hz` for
   `/scan /odom /imu`, and a viewport PNG (headless capture via Replicator or, if X is needed, the
   bare GPU-backed X server with `NVIDIA_DRIVER_CAPABILITIES=all` — see DEV_LOG lessons).
   **PASS**: both robots reach the walking-policy main loop and Nav2 7/7 lifecycle active on this box.
5. Write `docs/twin/DT0_BASELINE.md`: current topics/frames/rates/prim paths for both robots — the
   "before" for every later diff. Note the current arbitrary sensor offsets (G1 camera (0,0,0.75),
   lidar (0.2,0,0.4), (0.15,0,0.5); Go2 (0.3,0,0.10), (0.15,0,0.15), (0.1,0,0.2)) and the
   `Example_Rotary` / `Slamtec_RPLIDAR_S2E` stand-ins.

### DT1 — contract files + audit tool (½ day)
1. `isaac/twin/g1_contract.yaml`, `isaac/twin/go2_contract.yaml` (§4.1) populated from the driver repos
   with provenance; `docs/twin/TWIN_DEVIATIONS.md` skeleton.
2. `tools/twin_audit.py`: input = a live ROS 2 graph (default) **or** a rosbag; output = table
   (topic, type, frame, rate, QoS, encoding, present?, match?) + TF static edge diff (translation ≤1e-4 m,
   rotation ≤1e-4 rad) + camera_info K diff + LaserScan geometry (ray count, angle params, range
   bounds, fraction of rays < `range_min`) + PointCloud2 field layout diff. Exit non-zero on any Class-A
   mismatch. Include a `--against-evidence` mode that checks the static facts in
   `panthera-g1-driver/evidence/*` when no bag exists.
3. Unit tests: contract ↔ driver constants (§4.1), contract schema.
   **PASS**: audit runs against the DT0 baseline sim and correctly reports it as non-conformant.

### DT2 — G1 body + sensors at real poses, real wire (1 day)
1. Sensor layer per §4.2/§4.3 for the G1: Mid-360 inverted in the head at the calibrated mount, D435i at
   the URDF pose, IMU, RealSense subtree frames from the robot's `/tf_static`. Remove `Example_Rotary`,
   the RPLIDAR stand-in, `camera_optical`, `laser`, `/unitree_lidar` from the G1 path.
2. Publishers: `/ferox/g1_01/{scan,odom,imu/data}`, `/livox/lidar` (+ Livox IMU), the five camera
   topics, `/tf` (`odom→base_link`) and `/tf_static` (exact hardware edge set; `base_link→camera_link`
   published, matching the driver's optional edge). p2l with the driver's parameters. `mode:=twin` in
   Ferox launch.
3. Waist-dependence: the head sensors move with the 3 waist joints in the USD by construction. Verify with
   the audit that `base_link→livox_frame` at the standing pose reproduces the driver's static composite
   (roll 3.090233, pitch 0.161680, z 0.4995) to ≤1e-3 rad / ≤2 mm — this is the same round-trip the
   driver documents in `doc/05-validation.md §11.2`.
4. Body visual pass: Unitree meshes, materials, head shell, hidden puck; four PNGs (front/side/top/RViz TF).
5. Nav2 + SLAM in `mode:=twin` on `dso_block_a`: 3 goals, map screenshot; `ferox_vision` container against
   the twin camera (topics/encodings satisfied, detections on the FEROX_SIM_TEST_PROPS chair/bottle).
   **PASS**: `twin_audit` zero Class-A diffs for the G1 (except items explicitly deferred to DT3/DT6),
   `/scan` = 723 rays with the driver's angle params, camera 1280×720 rgb8/16UC1 same-stamp, rates within
   tolerance, floor-plane-vs-TF check on the sim cloud ≤0.5°, policy tracking unchanged vs
   `ferox-g1-locomotion/scripts/validate_motion.py`.

### DT3 — Dex5-1P variant (1 day)
1. Import Unitree `Dex5-URDF-{L,R}` at 1:1 with the Isaac Sim 5.1 URDF importer (convex decomposition on
   finger collision meshes; check whether the importer emits mimic joints — the URDF has no `<mimic>`, so
   add coupling for the passive 4/8/12/16 explicitly (PhysX mimic joint API, ratio 1:1 = `assumed`, flagged).
2. Masses/inertias: Unitree URDF values (palm included; L 1.025045 kg / R 0.978570 kg); no scaling.
3. Mount on `{left,right}_wrist_yaw_link` per §4.2; drives: position drives, gains from
   `configs/wbc/*dex5*.yaml` if present, else conservative documented defaults; limits from URDF; joint
   order = `limits.py`; `Roll_12` mirrored.
4. Tactile: 12 contact-sensor zones per hand (palm, 5 pads, 5 tips, root) → published later by DT6.
5. Tests: FK round-trip vs `limits.py` order (open/close sequence, screenshot), self-collision off with
   forearm chain, no interpenetration at rest, robot still walks (omni policy, `validate_motion.py`) with
   the hands attached — report base-height delta vs DT2 (real mass is now on the arms).
   **PASS**: `hand=dex5_1p` default loads, walks, opens/closes; body joint order test green; PNGs of hands
   at rest, open, fist, thumb opposition.

### DT4 — alternate hands (½–1 day)
`dex3_1` (from `g1_29dof_with_hand.urdf`, 7 DoF, 33 tactile), `dex1_gripper`, `inspire_ftp`
(Unitree's inspire URDF variant), `none`. Same mount frame, same tests as DT3 (reduced). Variant switch via
`HAND=` env in `01_start_sim.sh` → `run.py --hand`; the sim/nav guard records `robot:hand`.
**PASS**: every variant loads and walks; a table of mass/DoF/wire per variant in the results file.

### DT5 — Go2 twin (1 day)
Mid-360 on the top mount (0.187, 0, 0.0803, pitch 0.2249) with bracket + visible puck, L1 stand-in at
(0.28, 0, 0.075) publishing on the real L1 topics only if the driver consumes them (it retired L1 for
`/scan`; keep `/utlidar/imu` 250 Hz for the estimator feed only if cheap), front camera 120° at the head,
`camera` variant `none|d435i_dock` (pose TODO), publishers = `go2_driver_hw.launch.py` outputs, p2l with the
Go2 preset (`-0.20/0.50/0.30`), `mode:=twin` for Go2. Nav2+SLAM 3 goals. Point-cloud accumulator
(`cloud_accumulator.py`) is a driver node — run it in the twin bridge as on hardware or declare why not.
**PASS**: `twin_audit` zero Class-A diffs for the Go2, PNGs, nav evidence, sport-mode clamps respected.

### DT6 — hand DDS wire + tactile (1–2 days)
Per §4.5. Deliverables: `isaac/twin/hand_dds_bridge/` (Dockerfile baked deps: `unitree_sdk2py`,
CycloneDDS), config per variant, `press_sensor_state` from contact zones, clamping, health topic. Test:
W7 `wholebody/dex5` driver `dds` backend + `tests/test_dex5_guard.py` pass against sim; xr_teleoperate
retargeting (cherry-pick `39b79bc`, topic constants configurable) drives the sim hand — screenshot.
**PASS**: 20-entry `HandState_` at ≥200 Hz on `rt/dex3/*` from sim (real hand is 1 kHz — declare the
rate as Class C), cmd round-trip < 20 ms, over-pressure guard trips on a scripted crush.

### DT7 — Isaac Lab + SDG readiness, retire `mode:=sim` (1 day)
Per §4.6. Also: `docs/twin/TWIN_DEVIATIONS.md` final, `README.md` updated and a root `CLAUDE.md` created (second-person
imperative: variants, contract files, audit usage, `mode:=twin`, the standing rules of §0), `DEV_LOG.md` entries, delete dead sim-only bridges only after both
robots pass in twin mode; `isaac_bridge.launch.py` marked deprecated (removed in a later PR, not this one).
**PASS**: `isaaclab` config imports on this box (Isaac Lab install allowed here for the smoke only), SDG
smoke writes 10 frames with correct K, `mode:=sim` and `mode:=twin` both still boot for both robots.

### DT8 (stretch, sim-only) — `rt/lowstate` / `rt/lowcmd` + loco shim
Emulate `unitree_hg LowState_` (29 motors + IMU) at 500 Hz and a `loco` `SetVelocity` service shim that
feeds the RL policy, so `panthera-g1-driver` itself (`imu_republisher`, `waist_tf_bridge`,
`cmd_vel_to_loco` dry-run) can run against the sim. Only after DT0–DT7 are green and only if Mohammed says go.
Mutual exclusion rule stays: nothing here ever coexists with SONIC `rt/lowcmd` on a real bus.

Suggested order: DT0 → DT1 → DT2 → DT3 → DT5 → DT4 → DT6 → DT7 (→ DT8). Overnight is pre-authorized for
DT2–DT5 once DT1 passes; ping (write `docs/twin/PING.md` and stop) if a gate fails twice, if a decision
needs Mohammed (mount offsets, D435i on Go2, camera_info capture), or before anything destructive.

---

## 6. `twin_audit` tolerances

| Check | Class | Tolerance |
|---|---|---|
| Topic name, msg type, encoding, `frame_id`, QoS reliability | A | exact |
| TF static edges (set + values) | A | set exact; ≤1e-4 m, ≤1e-4 rad vs contract; ≤1e-3 rad / ≤2 mm for the waist round-trip |
| Rates | B | lidar/odom/imu ±10 %; camera ≥20 Hz (target 30) |
| LaserScan geometry | A/B | ray count, angle_min/max/increment exact; range_min/max exact; ≥99 % of returns in [range_min, range_max] |
| PointCloud2 fields | A | names/types/order exact vs real stream (or declared) |
| camera_info | B | width/height/model exact; K within 1 % of the real capture; D exact if captured |
| Extrinsics vs driver constants | A | equal to the digit (unit test) |
| Body joint names/order vs `deploy.yaml` | A | exact |
| Hand joint order vs `limits.py`, passive indices | A | exact |
| Masses (hands) vs Unitree URDF | B | ≤1 % |
| Sensor FOV/resolution | B | exact resolution; FOV ≤1 ° |

---

## 7. Reporting format (every gate)

```
# RESULTS_DT<n> — <title>
Host: <vast id / GPU / driver>   Date:   Verdict: PASS | FAIL | PASS-with-deviations
## Scorecard  (requirement | status | evidence path)
## What changed (files, one line each; link to commit/tag)
## twin_audit output (verbatim, trimmed)
## Deviations (Class C, one line each, cross-ref TWIN_DEVIATIONS.md)
## Open questions for Mohammed (numbered; default you took while waiting)
## Reproduce (exact commands)
```

Keep prose short. Tables over paragraphs. Numbers with units and provenance.

---

## 8. Open questions (ask at the first gate that needs them; proceed with the stated default)

1. Real `camera_info` messages (color + aligned depth) from G1 #1 → exact K/D. Default: D435i typical values, `assumed`.
2. One 30 s bag from each robot (`/livox/lidar` or `/unitree/slam_lidar/points`, `/ferox/<id>/{scan,odom,imu/data}`, camera_info, `/tf`, `/tf_static`) as `twin_audit` ground truth. Default: `--against-evidence` mode.
3. Dex5-1P wrist adapter offset/orientation on the G1 (Unitree assembly). Default: Dex3 flange offset from `g1_29dof_with_hand.urdf`, `assumed`.
4. Is a D435i mounted on Go2 #1, and where? Default: variant `none`.
5. Is this Vast.ai instance a VM with Docker (required) or a bare container? Default: stop at DT0 step 1.
6. Materials/colour reference photos of both robots for the visual pass. Default: Unitree product renders.

---

## 9. Session kickoff (Mohammed, once, on the VM)

1. `tmux new -s twin`; install the Claude Code CLI (`npm i -g @anthropic-ai/claude-code`) — the VS Code
   extension's bundled binary is not shared with the shell.
2. Mint a fresh fine-grained PAT: `contents:read` on `panthera-g1-driver`, `panthera-go2-driver`,
   `realsense_driver`, `panthera-g1-wbc`; `contents:write` on `ferox-isaac-demo`, `Ferox`,
   `ferox-g1-locomotion`. Revoke it once the clones are done; pushes use a new one the same way, per gate.

   **Use Basic auth, not Bearer.** GitHub's REST API accepts `Authorization: Bearer <pat>`, but the
   **git transport rejects it** for fine-grained PATs — you get
   `fatal: could not read Username for 'https://github.com'`, which reads like a missing credential
   rather than a rejected one. Git wants `Basic base64("x-access-token:<pat>")`.

   Pass it through `GIT_CONFIG_*` rather than `git -c`, so the token stays out of `ps` output too
   (`git -c` puts it in argv). It never touches a file, a remote URL, or `.git/config`:

   ```bash
   T='github_pat_…'                                    # this shell only, never a file
   B=$(printf 'x-access-token:%s' "$T" | base64 -w0)
   export GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=http.extraheader \
          GIT_CONFIG_VALUE_0="Authorization: Basic $B" GIT_TERMINAL_PROMPT=0

   git clone https://github.com/panthera-robotics/<repo>.git
   git push origin mohammed/twin-campaign && git push origin twin-DT<n>

   unset T B GIT_CONFIG_COUNT GIT_CONFIG_KEY_0 GIT_CONFIG_VALUE_0 GIT_TERMINAL_PROMPT
   ```

   Then verify nothing leaked before reporting done:
   ```bash
   grep -rl 'github_pat_' ~/panthera/ ; \
   grep -il 'extraheader\|authorization' ~/panthera/*/.git/config ~/panthera/ref/*/.git/config ; \
   git -C <repo> remote get-url origin
   ```
3. Save this file as `~/panthera/ferox-isaac-demo/docs/twin/CAMPAIGN.md` (committed in DT0).
4. First message to Claude Code (cwd `~/panthera/ferox-isaac-demo`):
   `Read docs/twin/CAMPAIGN.md end to end. Then execute DT0 only. Stop after RESULTS_DT0.md and wait.`
5. Every later turn: `Proceed to DT<n>.` (or `Overnight: DT2–DT5, ping rules per §5.`)

---

## Amendments (applied)

1. §1: `ferox-isaac-demo` has no `CLAUDE.md` yet — DT7 creates one at repo root (second-person imperative:
   variants, contract files, audit usage, `mode:=twin`, the §0 rules).
2. §4.3 Mid-360: also publish the Livox IMU on the sidecar's real topic `/livox/imu` (200 Hz, frame
   `livox_imu`).
3. DT6 PASS: `HandState_` ≥200 Hz from sim is acceptable; the real hand runs 1 kHz — declare that rate gap
   as Class C in `TWIN_DEVIATIONS.md`.
4. Reproduce/report format unchanged.
