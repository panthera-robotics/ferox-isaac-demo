# DT0_BASELINE — the sim as it exists before the twin campaign

Captured 2026-08-17 on the Vast.ai RTX 4090 box. This file is the **"before"** for every later
diff. Every number here is measured off the live ROS 2 graph or read at a cited `file:line`.
Nothing in this file is aspirational — it is what the sim does today.

Evidence: `docs/twin/evidence/DT0/`.

---

## 1. Host

| Item | Value |
|---|---|
| GPU | NVIDIA GeForce RTX 4090, 49140 MiB VRAM |
| Driver / CUDA | 580.105.08 / CUDA 13.0 |
| Docker | 29.0.3, storage `overlayfs`, runtimes `runc` + **`nvidia`** |
| nvidia-container-toolkit | 1.18.0-1 (`nvidia-ctk --version`) |
| CPU / RAM | AMD EPYC 7V73X, 30 vCPU / 98 GiB |
| Disk | `/dev/root` 291 G total, **241 G free** (≥150 G required) |
| Tailscale | `tailscale0` = 100.70.223.52 (ephemeral — never commit) |
| Virtualisation | `systemd-detect-virt` = **kvm**, no `/.dockerenv` → a VM, not a bare container; Docker path is valid |
| Isaac Sim image | `nvcr.io/nvidia/isaac-sim:5.1.0` |
| Sim container UID | 1234 (`isaac-sim`) |
| Forbidden containers | `panthera_om1_demo`, `panthera_nav`, `cmd_vel_publisher` — **none present** |

`.env` at demo root holds `FEROX_DDS_INTERFACE=tailscale0` and `FEROX_DDS_PEERS=<own tailscale IP>`
(own IP only — single-host VM, per §0.4).

---

## 2. What the sim publishes today

### 2.1 Topic set — identical shape for both robots

Sim-only graph (nav stack down), `ROS_DOMAIN_ID=42`:

```
/clock                                      rosgraph_msgs/Clock
/scan                                       sensor_msgs/LaserScan
/odom                                       nav_msgs/Odometry
/joint_states                               sensor_msgs/JointState
/unitree_lidar                              sensor_msgs/PointCloud2
/tf  /tf_static                             tf2_msgs/TFMessage
/ferox/<id>/cmd_vel                         geometry_msgs/Twist   (sim SUBSCRIBES)
/ferox/<id>/camera/color/image_raw          sensor_msgs/Image
/ferox/<id>/camera/color/camera_info        sensor_msgs/CameraInfo
/ferox/<id>/camera/depth/image_raw          sensor_msgs/Image
/ferox/<id>/camera/depth/camera_info        sensor_msgs/CameraInfo
```
Go2 additionally declares `/camera/go2/image_raw` and `/unitree/slam_lidar/points`
(the latter was **silent** on every probe).

Evidence: `g1_sim_topics.txt`, `go2_sim_topics.txt`.

> **There is no IMU topic on either robot — and two independent faults sit on that path.**
>
> 1. The publisher *is* built. `sim_utils.py:973-1006` creates an `/ImuGraph`
>    (`IsaacReadIMU` → `ROS2PublishImu`, `frameId "imu_link"`, `topicName "imu/data"`,
>    `readGravity True`) off the IMU prim created at `sim_utils.py:466-472` with
>    `frequency=50`. The log even prints `[ROS2] IMU publisher -> imu/data`. Yet **no imu
>    topic is advertised at runtime** — neither `/imu` nor `/imu/data` appears in
>    `ros2 topic list` for either robot. The graph is built and silently emits nothing.
> 2. The names would not match even if it worked. The graph sets no `nodeNamespace`, so it
>    would advertise **`/imu/data`**, while the Ferox sim bridge relays **`/imu`**
>    (`isaac_bridge.launch.py:54`: `_relay("imu", "/imu", "/imu/data", robot_id)`).
>
> Net effect: `/ferox/<id>/imu/data` is silent in every run. Hardware publishes it at 100 Hz
> in frame `dog_imu_link`.

### 2.2 Measured rates (`ros2 topic hz`, 10 s windows, sim only)

| Topic | G1 | Go2 | Hardware target |
|---|---|---|---|
| `/scan` | 4.24–4.83 Hz | 4.36–4.49 Hz | **10 Hz** |
| `/odom` | 27.7–29.6 Hz | 26.1–29.1 Hz | **~51.4 Hz** (G1) |
| `/imu` | absent | absent | **100 Hz** |
| `/clock` | 19.8–20.2 Hz | 29.0–29.1 Hz | — |
| `/joint_states` | 24.9–25.4 Hz | 29.8 Hz | — |
| `/unitree_lidar` | 17.8–19.3 Hz | 26.9–28.1 Hz | 10 Hz (`/livox/lidar`) |
| `camera/color/image_raw` | 25.5–26.3 Hz | 28.5–28.9 Hz | 30 Hz |
| `camera/depth/image_raw` | 22.8–23.1 Hz | 30.1–30.3 Hz | 30 Hz |

Evidence: `g1_sim_rates.txt`, `go2_sim_rates.txt`.

Nothing throttles these rates. `physics_dt` defaults to 1/200 s and `render_dt` to 1/60 s
(`run.py:1202-1203`); every ROS graph is driven by a plain `omni.graph.action.OnTick`, so the
nominal publish rate is one per render tick = 60 Hz. The measured 20–30 Hz is what the 4090
actually sustains with RTX lidar + two cameras in the warehouse. `setup_sensors_delayed`
accepts a `render_hz` argument and never uses it (`sim_utils.py:323`), and four of
`setup_ros_publishers`' seven parameters — `camera_link_pos`, `lidar_l1_pos`, `lidar_velo_pos`,
`robot_type` — are likewise dead (`sim_utils.py:933-941`). Rate control is a DT2 problem, not a
tuning knob that exists today.

### 2.3 `/scan` geometry — same for both robots, matches neither

| Field | Sim today | G1 hardware (`§3`) | Go2 hardware |
|---|---|---|---|
| `frame_id` | `laser` | `base_link` | `base_link` |
| ray count | **3200** | **723** | — |
| `angle_min` / `angle_max` | −3.1415927 / 3.1396291 | −π / π | — |
| `angle_increment` | **0.0019634953** | **0.0087** | 0.0087 |
| `scan_time` | 0.1 | 0.1 | — |
| `range_min` / `range_max` | **0.05 / 30.0** | **0.30 / 6.0** | 0.30 / — |
| producer | RTX lidar `Slamtec_RPLIDAR_S2E` stand-in | `pointcloud_to_laserscan` off `/livox/lidar` | p2l |

### 2.4 Cameras

| Field | Sim today | G1 hardware |
|---|---|---|
| color topic | `/ferox/<id>/camera/color/image_raw` | same |
| color resolution | **480×270** | **1280×720** |
| color encoding | `rgb8` ✔ | `rgb8` |
| depth topic | `/ferox/<id>/camera/depth/image_raw` | **`aligned_depth_to_color/image_raw`** |
| depth encoding | **`32FC1`** (metres) | **`16UC1`** (millimetres) |
| depth resolution | 480×270 | 1280×720 (module 848×480) |
| `frame_id` (both) | **`camera_optical`** | **`camera_color_optical_frame`** / `camera_depth_optical_frame` |
| `camera_info` K | fx=fy=349.2021, cx=240.0, cy=135.0 (derived from the sim prim) | from real capture (not yet supplied) |
| distortion | `plumb_bob`, D all zero | `plumb_bob`, real coefficients |
| `depth/color/points` | **absent** | present, xyzrgb, SENSOR_DATA |
| stamp alignment | color and depth carry **different stamps** | same stamp |

Colour/depth share one prim, so one intrinsic set covers both
(`sim.log`: `derived intrinsics fx=349.202 fy=349.202 cx=240.000 cy=135.000 480x270`).

### 2.5 Point cloud

`/unitree_lidar`, frame `lidar_l1_link`, `point_step 12`, **3 fields (xyz only, no intensity)**,
`is_dense true`, ~62 k–72 k points/msg. Hardware publishes `/livox/lidar` in `livox_frame`
(G1) and `/unitree/slam_lidar/points` in `livox_frame` (Go2).

---

## 3. Sensor placement — prims vs `/tf_static` **disagree**

`isaac/run.py:967-976` places the sensors per robot:

| Sensor | G1 prim offset | Go2 prim offset |
|---|---|---|
| `camera_link` | **(0.0, 0.0, 0.75)** | (0.3, 0.0, 0.10) |
| `lidar_l1_link` | **(0.2, 0.0, 0.4)** | (0.15, 0.0, 0.15) |
| `velodyne_base_link` | **(0.15, 0.0, 0.5)** | (0.1, 0.0, 0.2) |

but `isaac/sim_utils.py:566` `setup_static_tfs(simulation_app)` takes **no pose arguments** and
hardcodes one table, labelled in the source itself:

```python
# Define all the static transforms for Go2
("base", "lidar_l1_link",      [0.15, 0.0, 0.15], ...)
("base", "velodyne_base_link", [0.1,  0.0, 0.2 ], ...)
("base", "camera_link",        [0.3,  0.0, 0.1 ], ...)
```

Measured `/tf_static` on the **G1** is byte-identical to the Go2's — 11 edges, Go2 values
(`g1_sim_tf_static.txt` vs `go2_sim_tf_static.txt`).

> **Baseline defect B-1 (Class A).** On the G1 the sensor prims sit at the G1 offsets while
> `/tf_static` advertises the Go2 offsets. The TF tree misreports the camera by 0.30 m in x and
> 0.65 m in z, and the scan origin by 0.05 m in x and 0.30 m in z. Anything that projects sim
> pixels or sim scans through TF is wrong on the G1 today. This is precisely the silent-fallback
> class §0.7 exists to catch.

### 3.1 The 11 static edges the sim publishes (both robots)

```
base_link          -> base                    (0,0,0)
base               -> lidar_l1_link           (0.15, 0, 0.15)
base               -> velodyne_base_link      (0.1, 0, 0.2)
velodyne_base_link -> laser                   (0, 0, 0.0377)
base               -> imu_link                (0,0,0)
base               -> camera_link             (0.3, 0, 0.1)      quat (0.5,-0.5,-0.5,0.5)
camera_link        -> realsense_depth_camera  (0,0,0)
camera_link        -> realsense_rgb_camera    (0, 0.05, 0)
base_link          -> realsense_depth_camera  (0.3, 0, 0.1)
base_link          -> camera_optical          (0.3, 0, 0.1)      tilt −25°
map                -> odom                    (0,0,0)
```

> **Baseline defect B-2 (Class A).** The sim publishes a **static identity `map → odom`**.
> `slam_toolbox` publishes the *same edge* dynamically. Two publishers own one TF edge.
>
> **Baseline defect B-3 (Class A).** `laser` has two parents: the sim says
> `velodyne_base_link → laser`, the Ferox sim bridge spawns `lidar_l1_link → laser`.
>
> None of the hardware `/tf_static` edge names exist in the sim:
> hardware has `base_link→dog_imu_link`, `base_link→livox_frame`, `livox_frame→livox_imu`,
> `camera_link→camera_color_frame→camera_color_optical_frame`,
> `camera_link→camera_depth_frame→camera_depth_optical_frame`.
> The sim shares **zero** frame names with it apart from `base_link` and `camera_link`.

---

## 4. Prim paths (the "before" for the DT2/DT5 asset work)

| | G1 | Go2 |
|---|---|---|
| robot root | `/World/G1` | `/World/Go2` |
| sensor parent | **`/World/G1/torso_link`** | `/World/Go2/base` |
| IMU | `<parent>/imu_link` | same |
| camera link | `<parent>/camera_link` | same |
| depth camera | `<parent>/camera_link/realsense_depth_camera` | same |
| rgb camera | `<parent>/camera_link/realsense_rgb_camera` | same |
| front camera | `<parent>/camera_link/go2_rgb_camera` | same |
| L1 lidar | `<parent>/lidar_l1_link/lidar_l1_rtx` | same |
| 2D lidar | `<parent>/velodyne_base_link/laser/velodyne_vlp16_rtx` | same |
| world env | `/World/Env` | `/World/Env` |

Source: `isaac/run.py:275-310`, `isaac/sim_utils.py:16-65`.

The G1 already parents its head sensors to `torso_link`, which is the correct parent for the
twin (the hardware `waist_tf_bridge` is defined `torso_link → livox_frame`). The offsets are
what change, not the parent.

### 4.1 Sensor stand-ins to be removed

| Prim | Config today | Twin target |
|---|---|---|
| `lidar_l1_rtx` | `config="Example_Rotary"` (`sim_utils.py:492`) | Livox Mid-360 (G1 + Go2), or dropped |
| `velodyne_vlp16_rtx` | `config="Slamtec_RPLIDAR_S2E"` (`sim_utils.py:537`) | removed; `/scan` comes from p2l |
| `realsense_*_camera` | 480×270, tilt `CAMERA_TILT_DEG = -25.0` (`sim_utils.py:33`) | D435i 1280×720 at the URDF pose (pitch 47.6° down) |
| IMU | `frequency=50` (`sim_utils.py:469`), unpublished | 100 Hz on `imu/data`, frame `dog_imu_link` |

---

## 5. Bodies

### 5.1 G1 — 29 joints, articulation order as published on `/joint_states`

```
 0 left_hip_pitch_joint        10 right_knee_joint            20 right_shoulder_yaw_joint
 1 right_hip_pitch_joint       11 left_shoulder_pitch_joint   21 left_elbow_joint
 2 waist_yaw_joint             12 right_shoulder_pitch_joint  22 right_elbow_joint
 3 left_hip_roll_joint         13 left_ankle_pitch_joint      23 left_wrist_roll_joint
 4 right_hip_roll_joint        14 right_ankle_pitch_joint     24 right_wrist_roll_joint
 5 waist_roll_joint            15 left_shoulder_roll_joint    25 left_wrist_pitch_joint
 6 left_hip_yaw_joint          16 right_shoulder_roll_joint   26 right_wrist_pitch_joint
 7 right_hip_yaw_joint         17 left_ankle_roll_joint       27 left_wrist_yaw_joint
 8 waist_pitch_joint           18 right_ankle_roll_joint      28 right_wrist_yaw_joint
 9 left_knee_joint             19 left_shoulder_yaw_joint
```

This is USD/articulation (breadth-first) order, **not** the policy's `deploy.yaml` order —
`run.py` maps between them. DT2/DT7 assert the mapping stays exact.

No hands: the wrists end bare (`g1_viewport.png`).

### 5.2 Odom

`nav_msgs/Odometry`, `frame_id odom`, `child_frame_id base_link` ✔ (matches hardware —
no `pelvis` frame is published).

Standing pelvis height reads **z ≈ 0.791 m** in sim (single sample, `g1_sim_msgs.txt`) against
**0.731 m** on hardware — a ~60 mm delta. It matters: the driver's p2l z-slice
(`min_height -0.556`, `max_height 0.50`) is defined against the standing pose, so DT2 has to
either reproduce the hardware standing height or re-derive the slice. Flagged, not resolved.

---

## 6. Baseline nav behaviour (for reproducibility, not a defect)

At the warehouse spawn (0,0) the nearest lidar return is **8.8 m** (0 of 2423 valid returns
within 6 m). The two robots then behave differently, because their SLAM profiles differ:

| | `max_laser_range` | `scan_topic` | Effect at spawn |
|---|---|---|---|
| Go2 | **6.0** (`go2_slam.yaml:59`) | **`/scan`** (absolute → the raw sim topic) | map stays **0×0**; Nav2 logs `Received map message is malformed. Rejecting.` until the robot has driven ~3.5 m |
| G1 | **12.0** (`g1_slam.yaml:35`) | **`scan`** (relative → `/ferox/g1_01/scan`, the relayed copy) | map builds immediately (398×267 before the robot moved) |

Both robots plan and reach goals once a map exists. This is a **world/parameter interaction, not
a code fault** — recorded so the next operator does not re-diagnose it. Note the incidental
asymmetry: the Go2 profile's absolute `/scan` bypasses the namespace, so `/ferox/go2_01/scan`
had **0 subscribers** during the Go2 run even though the relay was publishing it.

The scan also carries occasional ~0.06 m self-hit returns (below any sensible `range_min`);
hardware handles the same effect on the G1 with `range_min 0.30`.

---

## 7. Summary of deltas the campaign has to close

| # | Baseline | Twin target | Gate |
|---|---|---|---|
| 1 | `/scan` 3200 rays, `laser`, 0.05–30 m | 723 rays, `base_link`, 0.30–6.0 m | DT2/DT5 |
| 2 | No `/imu` at all | `imu/data` 100 Hz, `dog_imu_link` | DT2/DT5 |
| 3 | Camera 480×270 `32FC1` in `camera_optical` | 1280×720 `rgb8` + `16UC1` mm, real optical frames, same stamp | DT2 |
| 4 | `/unitree_lidar` xyz-only, `lidar_l1_link` | `/livox/lidar` (+`/livox/imu`), `livox_frame`, real field layout | DT2/DT5 |
| 5 | 11 sim-invented `/tf_static` edges, Go2 values on the G1 | exact hardware edge set at calibrated values | DT2/DT5 |
| 6 | Static `map→odom` from the sim | removed (SLAM owns it) | DT2 |
| 7 | `Example_Rotary` / `Slamtec_RPLIDAR_S2E` stand-ins | Mid-360 RTX model | DT2/DT5 |
| 8 | Sensor poses arbitrary | calibrated poses from the driver repos | DT2/DT5 |
| 9 | Rates 4–30 Hz, none matching | lidar 10, camera 30, imu 100, odom 50 | DT2/DT5 |
| 10 | No hands | Dex5-1P default + variants | DT3/DT4 |
| 11 | Topics reach Nav2 via `topic_tools/relay` | published directly under `/ferox/<id>/` | DT2/DT5 |
