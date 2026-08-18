# DT5 — the Go2 twin

**Verdict: PASS-with-deviations on the interface, PARTIAL on navigation.**
The audit is **Class-A conformant** — 45 pass, 0 Class-A failures. The Go2 twin
publishes what the driver publishes: root `/scan` and `/odom`, both clouds,
`/utlidar/imu`, no camera, no namespaced IMU, and five static edges exact to
0.00e+00. Navigation does **not** work, for one specific and fully diagnosed reason
recorded as C-17.

Run on 2026-08-18, branch `mohammed/twin-campaign`, `SIM_WORLD=hospital`, `TWIN=1`.

---

## 1. Interface (`evidence/DT5/audit_twin_go2.txt`)

```
summary: 45 pass, 0 Class-A FAIL, 3 Class-B fail, 10 skipped, 58 checks
RESULT: conformant on Class A.
```

| what the driver does | what the twin does |
|---|---|
| `/scan` at the **root**, not namespaced | same |
| `/odom` at the **root** | same |
| `/unitree/slam_lidar/points`, 20 Hz, livox_frame | same |
| `/mid360/points_accum` from cloud_accumulator | same, node ported verbatim |
| `/utlidar/imu`, utlidar_imu frame | same |
| **no** `/ferox/go2_01/imu/data` | absent |
| **no** camera | absent |
| 5 static edges incl. `utlidar_imu`, `robot_center` | all 5, exact to 0.00e+00 |
| p2l `angle_min/max = ∓3.14159` (truncated) | same, 723 rays |
| `cmd_vel` clamps ±(0.8/−0.3, 0.3, 1.0) | verified, see §3 |

LaserScan geometry is exact on every field: 723 rays, `angle_min −3.14159012`,
`angle_max 3.14159012`, `angle_increment 0.00870000012`, `range_min 0.300000012`,
`range_max 6`, `scan_time 0.100000001`.

The three remaining Class-B failures are all declared and all arithmetic:

| topic | contract | twin | why |
|---|---|---|---|
| `/odom` | 148.7 Hz | 100.0 Hz | C-15 — 200/148.7 rounds to a 2-step limiter |
| `/utlidar/imu` | 250 Hz | 200.0 Hz | C-16 — 250 Hz is **above** the physics rate |
| `/ferox/go2_01/driver_heartbeat` | published | absent | driver-only; the sim does not emulate the driver |

---

## 2. The rate bug that was fixed, because it is the kind that hides

The first audit had `/scan`, `/unitree/slam_lidar/points` and `/mid360/points_accum`
all at **25 Hz against a 20 Hz contract** — a 25 % error that reads as sensor jitter.

The RTX lidar is decimated off the render clock by an **integer** step, so the render
rate must be a whole multiple of the contract's lidar rate:

```
G1   10 Hz lidar:  0.020 s = 4 physics substeps = 50 Hz render, step 5  -> 10 Hz   OK
Go2  20 Hz lidar:  0.020 s = 4 physics substeps = 50 Hz render, 50/20 = 2.5
                   -> the gate rounds to 2                             -> 25 Hz   WRONG
Go2  20 Hz lidar:  0.025 s = 5 physics substeps = 40 Hz render, step 2 -> 20 Hz   OK
```

The Go2 twin now renders at 0.025 s. `physics_dt` is untouched, so nothing about the
robot's dynamics changed. This is the third time in this campaign that a rate came out
wrong because of a divisor that is not an integer, and it is now commented at the
point where the number is chosen.

---

## 3. cmd_vel clamps — verified as the campaign asked

Three independent readings, all agreeing exactly:

| source | max_linear_x | min_linear_x | max_linear_y | max_angular_z |
|---|---|---|---|---|
| `cmd_vel_to_sport.py:146-149` (defaults) | 0.8 | −0.3 | 0.3 | 1.0 |
| `cmd_vel_to_sport.py:252-254` (the clamp) | same | same | same | same |
| Ferox `go2.yaml:26-29` | 0.8 | −0.3 | 0.3 | 1.0 |

Contract provenance raised `configured` → `calibrated`. The driver names `go2.yaml`
as its own source, so this is one value with two readers rather than two values that
happen to agree — but both were read, and both said the same thing.

Noted while there: `go2.yaml:31-32` additionally caps **acceleration**
(`max_linear_x 1.5`, `max_angular_z 2.0`). The driver does not enforce it and neither
does the twin. Recorded as OQ-5.2 rather than silently implemented.

---

## 4. Navigation: PARTIAL, and exactly why

The control path works. Driving `/ferox/go2_01/cmd_vel` directly, with Nav2 down so
nothing could override it, moved the robot **1.616 m in 6 s at a commanded 0.4 m/s**,
and a six-segment exploration drive steered it around a 3 m loop
(`evidence/DT5/control_path.txt`).

Every Nav2 goal failed. Three attempts, three different failures, and each one taught
something:

1. **Goals outside the map.** The first set used DT2's G1 coordinates. The Go2's SLAM
   map spanned x[7.75, 11.20] — one goal was outside it. Operator error, same class as
   the DT2 lesson, fixed by selecting goals from the map's own known-free cells.
2. **`planner_server` segfaulted.** With the robot sitting exactly on the map's x
   boundary, the costmap logged *"Robot is out of bounds of the costmap"* and
   `planner_server` died with exit code −11, taking the lifecycle manager's other
   nodes down with it. Recorded as a **Ferox/Nav2 robustness finding** — a planner
   should refuse a goal, not crash. Worked around by driving the robot away from the
   boundary so the map grew around it.
3. **A permanent obstacle 0.30 m ahead.** With Nav2 healthy, the robot at a valid
   pose and goals in known-free space, the goal was accepted, ran **21 recoveries**,
   and aborted with the robot never moving.

The cause of (3) is measured and reproducible, and is written up as **C-17**:

```
sensor frame : 134 returns at 0.100 .. 0.152 m  (the robot's own nose)
base_link    : x +0.272 .. +0.332
/scan        : 13-14 rays at 0.300 .. 0.313 m, bearings -6.5 .. -0.1 deg
```

`pointcloud_to_laserscan` applies `range_min` in its **target frame**. These returns
are 0.10–0.15 m from the sensor — far inside the driver's 0.30 m — but the sensor sits
0.187 m forward of `base_link`, so in `base_link` they are at 0.27–0.33 m and the ones
past 0.30 survive. Nothing is misconfigured. The threshold does exactly what it
documents, in a frame where these points are genuinely far enough away.

The cluster is body-fixed: identical across consecutive scans, and unchanged after the
robot drove 1.4 m and rotated. Between 0.35 m and 2.0 m the scan is completely empty —
the cluster is the only near return, and it is the robot.

**This was not "fixed" by hiding geometry from the sensor**, which is the campaign's
listed fallback and would have made the number go away in ten minutes. It is not yet
established that this is a sim artefact: if the real Go2's Mid-360 also sees its own
nose, hardware `/scan` carries the same cluster and Nav2 behaves the same way on the
robot — which would make this a faithful reproduction of a real problem and precisely
the kind of finding this campaign exists to surface. That question needs one capture
from hardware, and it is OQ-5.1.

---

## 5. Floor-plane check: measured, and deliberately not used as a criterion

`evidence/DT5/floor_plane.txt`. The fit puts the floor at −0.400 m under `base_link`
with a 0.027 m residual, and `min_height −0.20` sits 0.20 m above it — so the slice
rejects the floor, as designed.

The **tilt** number from that fit is not trustworthy and is not used. Two samples
minutes apart put the floor at −0.355 m and −0.400 m and **flipped the sign** of the
x-gradient (−0.0217 → +0.0317). A mount error is a constant; a quantity that changes
sign between samples is a fit artefact. The Go2's Mid-360 sits 0.08 m up and yields
~80 floor returns scattered to 6 m, which cannot resolve a 0.5° mount error. The G1's
equivalent check works because its sensor is 0.50 m up and sees far more floor.

The mount is verified by three things that are conclusive instead: the asset builder
authored and read back 5/5 frames against the contract; the audit compares published
`tf_static` against the contract and finds 0.00e+00 on every edge; and no floor
returns enter `/scan`.

---

## 6. Open questions

* **OQ-5.1** — does the real Go2's Mid-360 see the robot's own nose? One capture of
  `/scan` and `/unitree/slam_lidar/points` from the robot standing still settles
  C-17, and decides whether the fix belongs in the sim or in the driver.
* **OQ-5.2** — `go2.yaml` caps acceleration (1.5 m/s², 2.0 rad/s²) and nothing
  enforces it, on the robot or in the twin. Intentional, or a gap?
* **OQ-5.3** — `planner_server` segfaults when the robot is outside the costmap
  rather than rejecting the goal. Ferox-side; reproducible by starting Nav2 with the
  robot on the SLAM map's boundary.
* **OQ-5.4** — the Go2 twin has no visible Mid-360 puck or mount bracket. The frames
  are authored and correct; the geometry is not there. Deferred with the rest of the
  visual pass.

## 7. Deviations opened

* **C-15** — `/odom` 100 Hz, not 148.7 Hz (physics-step quantisation).
* **C-16** — `/utlidar/imu` 200 Hz, not 250 Hz (target above the physics rate).
* **C-17** — the Mid-360 sees the robot's own front face and `/scan` keeps it.
* **C-8** extended to the Go2's cloud (Isaac's RTX writer is xyz-only).

## 8. Reproduce

```bash
./scripts/08_build_twin_assets.sh go2            # 5/5 frames verified
ROBOT=go2 TWIN=1 SIM_WORLD=hospital ./scripts/01_start_sim.sh
ROBOT=go2 MODE=twin ./scripts/02_start_ferox.sh
ROBOT=go2 ./scripts/07_twin_audit.sh --duration 25
```
