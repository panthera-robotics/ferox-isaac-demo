# TWIN_DEVIATIONS — every place the sim is knowingly not the robot

A deviation that is written down is engineering. A deviation that is not is the MuJoCo W6
failure mode: a hand observed as Dex3 but commanded as Dex5, a 0.75-scaled graft, grasps that
happened by weld. **Nothing gets approximated silently. If it is not in this file, the sim is
claiming parity.**

## Parity classes (campaign §2)

| Class | Meaning | Where it is enforced |
|---|---|---|
| **A — exact** | Strings and structure identical to hardware: topic names, msg types, encodings, `frame_id`s, TF topology and static edge values, QoS reliability, namespace, joint names/order, hand wire | `tools/twin_audit.py` — zero diffs, exit non-zero otherwise |
| **B — modeled** | Physical property reproduced from datasheet / URDF / calibration within a stated tolerance | `twin_audit` tolerances, campaign §6 |
| **C — approximated, declared** | A known sim limit we accept | **this file** |

A Class-C entry is only valid with: the real value, the sim value, the consequence, the decision
that made it acceptable, and what re-checks it.

## How to add one

Append a row to the table, then a full entry below it. Never delete an entry — mark it
`RESOLVED` with the gate that closed it, so the history of what we accepted stays readable.

---

## Register

| ID | Class | Subject | Opened | Status |
|---|---|---|---|---|
| [C-1](#c-1) | C | G1 standing height: ~0.82 m sim vs 0.731 m real (~90 mm, re-measured at DT2) | DT1 | RE-CHECKED at DT2 — no floor returns in `/scan`, 0.2655 m margin |
| [C-2](#c-2) | C | Mid-360 PointCloud2 field layout is not recorded for either robot's live stream | DT1 | OPEN — G1 defaulted to the sidecar driver's layout at DT2; Go2 still unknown |
| [C-3](#c-3) | C | D435i intrinsics (K/D) are factory-typical placeholders | DT1 | OPEN — needs OQ-1 |
| [C-4](#c-4) | C | RealSense intra-camera TF values unknown (edges confirmed, numbers absent) | DT1 | OPEN — needs OQ-1 |
| [C-5](#c-5) | C | Aligned-depth metric scale (mm) is convention, never written down | DT1 | OPEN — needs OQ-1 |
| C-23 addendum | — | camera also blocks filming the LIVE sim; both media routes blocked on a 16 GB box | MM1/MM2 | OPEN — 4090 item |
| [C-6](#c-6) | C | Mid-360 non-repetitive rosette modelled as a uniform rotary grid | DT2 | OPEN — inherent to Isaac's RTX lidar; **quantified at MM2**: twin 45–58 % finite vs robot 70 %, pose-independent |
| [C-24](#c-24) | — | **WITHDRAWN** — claimed a missing `cloud_accumulator`; the real G1 driver has none either. Duplicate of C-6 | MM2 | WITHDRAWN same day |
| [C-7](#c-7) | B→C | Sim USD waist chain is 10 mm shorter than the robot's URDF | DT2 | OPEN — body asset, not patched by design |
| [C-8](#c-8) | C | Mid-360 cloud is xyz-only; Isaac has no node that emits the livox field layout | DT2 | OPEN — Isaac bridge limitation |
| [C-9](#c-9) | C | Rendered depth has no stereo baseline, so no occlusion shadows | DT2 | OPEN — inherent to rendered depth |
| [C-10](#c-10) | C | IMU rates capped by the render/physics step coupling; aligned depth below the 20 Hz floor | DT2 | OPEN — floor relaxed to 15 Hz for DT2 |
| [C-11](#c-11) | C | The sim stands with waist = 0; the real G1 stands pitched ~6.2° | DT2 | OPEN — policy property, not geometry |
| [C-12](#c-12) | C | No head-shell self-hit cluster: sim r_min 1.10 m vs real 0.0985 m | DT2 | OPEN — accepted, `range_min` still reproduced |
| [C-23](#c-23) | C | Isaac's synthetic-data HOST COPIES segfault on this box (RTX 4080 SUPER 16 GB) — ROS 2 image writer *and* segmentation annotators | 2026-08-19 | **ENVIRONMENT, accepted — it is the GPU.** Camera work *and any mask-based metric* wait for a 4090; check `nvidia-smi` first |

---

## C-1 — G1 standing height is ~60 mm taller in sim than on hardware {#c-1}

> **Updated 2026-08-18 by the ground-truth capture.** The odom-z line of this entry's
> evidence is **withdrawn**: the robot's `/odom` z is ground-referenced at +0.00675 m,
> not a standing height, so "0.791 sim vs 0.731 real" was never comparing like with
> like (see [[C-20]]). **C-1 itself stands**, on the floor-plane measurement, which
> never depended on odometry: the p2l slice sits ~60 mm higher off the floor in sim,
> fitted independently on both sides.

**Opened:** DT1 (2026-08-18) · **Class:** C · **Status:** OPEN, re-check at DT2

| | Sim | Hardware |
|---|---|---|
| Standing pelvis / `base_link` height | **0.791 m** (`/odom` sample, `docs/twin/evidence/DT0/g1_sim_msgs.txt`) | **0.731 m** |

**Consequence.** The driver's `pointcloud_to_laserscan` z-slice is expressed relative to
`base_link`, so a taller base lifts the whole slice off the floor by the same 60 mm:

| p2l bound | Height above floor — sim | Height above floor — real | Δ |
|---|---|---|---|
| `min_height = -0.556` | **0.235 m** | **0.175 m** | +0.060 m |
| `max_height = 0.50` | **1.291 m** | **1.231 m** | +0.060 m |

**Decision (Mohammed, DT0 acceptance).**
- The walking policy is **not** touched.
- The p2l slice is **not** re-derived. **Driver p2l parameters stay Class A — reproduced exactly.**
- The height gap is carried here as Class C instead.

Rationale: the slice bounds are part of the hardware interface the twin exists to reproduce. Bending
them to fit the sim body would make the sim self-consistent and hardware-wrong, which is exactly the
trade this campaign exists to refuse.

**Caveat — the 0.731 m reference is itself provisional.** It is a post-firmware value whose
`odom_pelvis` z semantics are unresolved: `/odommodestate` reports `body_height 0.6726`, which does
not reconcile with 0.731 m. Until that is settled on hardware, the *magnitude* of this deviation is
uncertain even though its *existence* is not. Do not tune anything against 60 mm as if it were exact.

**Re-check (DT2) — DONE, and the numbers moved.** Measured on the live twin with the waist
bridge running (`docs/twin/evidence/DT2/geometry_check.txt`):

| | value |
|---|---|
| floor height in `base_link` (plane fit, 2438 pts, 1.78 mm residual) | **−0.8215 m** |
| p2l `min_height` | −0.556 m |
| **margin below the slice** | **0.2655 m** |
| floor returns entering `/scan` | **none** |

Note the standing height itself: −0.8215 m implies a pelvis **0.8215 m** above the floor, not the
0.791 m single `/odom` sample recorded at DT0. The plane fit is the better measurement — 2438
points against one message — so the gap to the robot's 0.731 m is nearer **90 mm** than 60 mm, and
the derived clearances shift with it (0.2655 m sim vs 0.175 m real, rather than 0.235 vs 0.175).
The *decision* is unchanged: the p2l slice stays Class A and exact.

The 90 mm of extra clearance makes floor bleed *less* likely in sim than on hardware, so this clean
result does **not** clear the hardware case — it only fails loudly if something else is wrong.

---

## C-2 — the real Mid-360 cloud's field layout is not recorded anywhere {#c-2}

**Opened:** DT1 · **Class:** C · **Status:** OPEN

The twin must publish a `PointCloud2` whose field layout matches the robot's byte for byte
(campaign §6: PointCloud2 fields are Class A). **The layout of the streams actually in use is
recorded nowhere in either driver repo.**

What we do have:

| Stream | Status | Field layout |
|---|---|---|
| `/utlidar/cloud_livox_mid360` (G1, **retired**) | captured | `point_step 22`: x,y,z,intensity `FLOAT32`; ring `UINT16`; time `FLOAT32` — `evidence/phase0/streams_90s.json`, 897 clouds, 9.982 Hz, frame `livox_frame` |
| `/livox/lidar` (G1, **live**) | rate + frame + point count only | `evidence/fw2026q3/session_a/13_livox_lidar.txt`: 10.000 Hz, `livox_frame`, 19968–20064 pts. **No fields.** |
| `/unitree/slam_lidar/points` (Go2, **live**) | rate + frame + QoS only | **No fields.** |

The live G1 stream comes from our own `livox_ros_driver2` sidecar, not from Unitree firmware,
so the retired Unitree layout is a *plausible* but unproven stand-in — `livox_ros_driver2`'s
own PointCloud2 output conventionally carries `tag`/`line`/`timestamp`, not `ring`/`time`.

**Decision (Mohammed, DT1 acceptance).** The two robots are defaulted differently, because the
provenance differs:

| Robot | Default layout | Why |
|---|---|---|
| **G1** `/livox/lidar` | **livox_ros_driver2's**: x,y,z,intensity `FLOAT32`; tag,line `UINT8`; timestamp `FLOAT64`; `point_step 26` | The stream comes from **our own** `livox_ros_driver2` sidecar, not Unitree firmware (`cloud_restamper.py` header, `phase1_diffs.patch:446`). The publisher is known even though its output was never captured. |
| **Go2** `/unitree/slam_lidar/points` | the captured `/utlidar/cloud_livox_mid360` layout (`point_step 22`) | Publisher is the Mid-360S sidecar SDK; nothing pins its layout, so the only measured layout on this hardware stands in. |

Both are `assumed`, and `twin_audit` lists every `assumed` value in its own section.

**Closes when** OQ-2 lands: one `ros2 topic echo --once --no-arr` of each live cloud.

---

## C-3 — D435i intrinsics are factory-typical placeholders {#c-3}

**Opened:** DT1 · **Class:** C · **Status:** OPEN

`camera_info` K and D come from the device at runtime via librealsense. `realsense_driver`
configures nothing about them and contains **zero** intrinsics (no distortion model, no K, no D —
zero grep hits for `distortion`/`intrinsic`). Both contracts therefore carry
`K = [908, 0, 640, 0, 908, 360, 0, 0, 1]`, a factory-typical D435i at 1280×720 (HFOV ≈69°), with
`provenance: assumed`.

`twin_audit` prints every `assumed` value in its own section precisely so this cannot quietly
become load-bearing. The K tolerance is 1 % (campaign §6), and a placeholder will not meet it
against a real capture.

**Closes when** OQ-1 lands.

---

## C-4 — RealSense intra-camera TF values are unknown {#c-4}

**Opened:** DT1 · **Class:** C · **Status:** OPEN

Four edges are **confirmed to exist** on the robot
(`evidence/fw2026q3/session_a/31_tf_static.txt`):
`camera_link→camera_color_frame`, `camera_color_frame→camera_color_optical_frame`,
`camera_link→camera_depth_frame`, `camera_depth_frame→camera_depth_optical_frame`.
Session A recorded **edge names only, no transform values**, and `realsense_driver` publishes no
TF into the robot's chain (`README.md:9`) — the numbers live in the device.

The contract splits these honestly:
- the two `*_frame → *_optical_frame` rotations are the fixed REP-103 body→optical convention
  `(-π/2, 0, -π/2)` — certain, though still marked `assumed` so they surface for confirmation;
- the two `camera_link → *_frame` **translations are placeholder identities**. On a real D435i
  the colour and depth frames are physically offset from `camera_link` by tens of millimetres.
  Publishing identity means the twin's colour and depth optical origins coincide when the
  robot's do not.

**Consequence.** Anything that reprojects depth into colour geometrically (not via the
already-aligned image) will be wrong by that offset. The aligned-depth image itself is unaffected,
because alignment happens on-device before publication.

**Related, and NOT a deviation:** `base_link → camera_link` is absent from the robot's
`/tf_static` because the driver ships `camera_tf_enable: false` (`g1_driver.yaml:186`) pending a
floor-plane check. The camera subtree is an orphan root on the real robot today, and the twin
reproduces that. Campaign DT2 step 2 assumed this edge is published; it is not.

**Closes when** OQ-1 lands.

---

## C-5 — the aligned-depth metric scale is convention, not a recorded fact {#c-5}

**Opened:** DT1 · **Class:** C · **Status:** OPEN

`realsense_driver/README.md:24` states the encoding `16UC1`. **Nowhere** does the repo state
whether a count is a millimetre or something else: `depth_module.depth_units` is never set, and
mm is never written. The campaign brief asserts millimetres; the file does not.

The twin publishes millimetres (librealsense default `depth_units = 0.001`). This is almost
certainly right and is still `assumed`, because a silent factor-of-1000 in depth is exactly the
class of error that survives every sim test and fails on the robot.

**Closes when** OQ-1 lands — one message of
`/ferox/g1_01/camera/aligned_depth_to_color/image_raw` with a known-distance target is enough.

---

## C-6 — the Mid-360's non-repetitive rosette is modelled as a uniform rotary grid {#c-6}

**Opened:** DT2 · **Class:** C · **Status:** OPEN (inherent; no closing action)

The real Mid-360 draws a **non-repetitive rosette**: no two revolutions trace the same pattern,
so coverage density *grows with dwell time* and fine structure emerges the longer the robot
looks at something. Isaac Sim 5.1's RTX lidar has no such `scanType` — the primitives are
`ROTARY` and solid-state. We model the sensor as `ROTARY` and spend the real point budget on a
uniform grid.

| | Real Mid-360 | Twin |
|---|---|---|
| Scan pattern | non-repetitive rosette | uniform grid, identical every revolution |
| Points/second | 200 000 | **200 000** (exact) |
| Elevation FOV | −7° … +52° | **−7° … +52°** (exact), 40 emitters, 1.5128° step |
| Azimuth FOV | 360° | **360°**, G1 500 columns (0.72°), Go2 250 columns (1.44°) |
| Rate | G1 10 Hz, Go2 20 Hz | **same** |
| Range / accuracy | 0.1–40 m, ±2 cm | **same** |
| Angular precision | < 0.15° | σ = 0.05° assumed (read as ~3σ) |

**What transfers:** per-frame point count, the range and FOV envelope, the raw sensor-frame
un-self-filtered character that makes `range_min 0.30` necessary, and therefore everything
`pointcloud_to_laserscan` derives from it.

**What does not:** anything depending on rosette coverage growth — a thin obstacle that the real
sensor resolves only after dwelling on it appears immediately (or never) in the twin, and
accumulation over N frames adds no new angles here whereas on hardware it does. This matters for
`cloud_accumulator` behaviour (Go2) and for any perception tuned on dwell.

**Note on the mechanism, because it is a trap.** Isaac ships JSON lidar profiles whose folder
README says the file name is the `config` argument to `IsaacSensorCreateRtxLidar`. That is stale:
5.1 resolves `config` against a hardcoded dict of **USD asset paths**, and an unknown name does
**not** raise — measured in this container, `config="Livox_Mid360"` produced a warning, a
fully-formed **default** lidar, and a silent rename to `/World/World_bogus_lidar`. `isaac/twin/lidar.py`
therefore builds from the supported `Example_Rotary` asset, overwrites the `omni:sensor:Core:*`
attributes from the contract, and **reads every one back**, raising if any did not take.

---

## C-7 — the sim's G1 USD has torso_link 10 mm lower than the robot's URDF {#c-7}

**Opened:** DT2 · **Class:** B measured, carried as C · **Status:** OPEN, deliberately not patched

The pelvis→torso_link offset differs between the sim body asset and the robot's URDF:

| | x | y | z |
|---|---|---|---|
| Robot URDF (`evidence/fw2026q3/g1_29dof.urdf`): `waist_yaw` (0,0,0) + `waist_roll` (−0.0039635, 0, 0.035) + `waist_pitch` (0, 0, 0.019) | −0.0039635 | 0 | **0.054** |
| Sim USD `g1_29dof_rev_1_0` (composed, waist at zero) | −0.0039635 | 0 | **0.044** |
| Δ | 0 | 0 | **−0.010 m** |

x matches to the digit, so this is not a units or convention error — the sim asset is a
different sub-revision of the same model.

**Consequence.** Every torso-mounted sensor — Mid-360, D435i, and their whole subtree — sits
**10 mm lower in `base_link`** in the sim than on the robot. Each sensor's pose *relative to
`torso_link`* is contract-exact; the error is entirely in the body between pelvis and torso.
It also fully explains the 10 mm the D435i z appeared to be "off" by: the camera frame is right,
the link it hangs from is low.

**Why it is not patched.** Campaign §4.2 says reuse the tuned physics layer verbatim, and rule 9
forbids adjusting an offset to make something fit. Editing the body to close a 10 mm gap would
mean re-deriving mass and inertia from an asset we did not import, to chase a discrepancy whose
own reference (the standing pelvis height) is itself provisional — see [C-1](#c-1). Recorded,
not silently corrected.

**Where it shows up.** In the `base_link → livox_frame` and `base_link → camera_link` composites,
as a −10 mm z bias. It does NOT affect `torso_link`-relative geometry, the sensor mount
attitudes (which compose exactly), or the p2l slice, which is expressed in `base_link` against a
floor the sim also renders.

**Re-check** if the body asset is ever regenerated from the current URDF.

---

## C-8 — the Mid-360 cloud carries xyz only {#c-8}

**Opened:** DT2 · **Class:** C · **Status:** OPEN (Isaac bridge limitation)

| | Fields | `point_step` |
|---|---|---|
| Contract (livox_ros_driver2) | x, y, z, intensity `FLOAT32`; tag, line `UINT8`; timestamp `FLOAT64` | 26 |
| Twin | x, y, z `FLOAT32` | **12** |

`RtxLidarROS2PublishPointCloud` is the only node in the Isaac Sim 5.1 ROS 2 bridge that turns
an RTX lidar into a `PointCloud2`, and it emits xyz only — measured, and consistent with the
legacy `/unitree_lidar` stream this sim has always produced (3 fields, `point_step 12`). No
bridge node adds intensity, tag, line or per-point timestamp.

**Consequence.** Anything consuming intensity (reflectivity-based segmentation), `line`
(ring-based ground removal), or per-point `timestamp` (motion compensation / deskewing) has
nothing to consume. `pointcloud_to_laserscan` needs only xyz, so `/scan` — and therefore Nav2
and SLAM — is unaffected. FAST-LIO2 would care, which is why this is worth closing eventually.

**How it is reported.** The contract keeps the REAL layout and marks these two checks as
declared deviations, so `twin_audit` still runs them and still prints the difference every
run; they simply no longer gate the exit code. Deleting the checks would make the sim look
conformant, which is the failure mode this campaign exists to prevent.

**Would close by** publishing the cloud from the RTX lidar annotator through a custom node
that packs the full layout, in the same place the camera converter already lives.

---

## C-9 — rendered depth has no stereo occlusion shadows {#c-9}

**Opened:** DT2 · **Class:** C · **Status:** OPEN (inherent)

A real D435i derives depth by stereo matching, so it produces occlusion shadows: a band of
invalid pixels the colour sensor can see but the right IR sensor cannot, whose width scales
with the baseline and inversely with range. Rendered depth has no baseline and therefore no
shadows.

**Consequence.** Twin depth is denser and cleaner at depth discontinuities than the robot's,
particularly along the near edges of foreground objects. Perception tuned on shadow geometry —
or on the invalid-pixel fraction as a confidence proxy — will not transfer. The compensating
approximation is deliberate: because colour and depth are rendered from ONE camera prim, the
alignment and the shared timestamp are exact rather than approximate, which is the parity that
actually matters to the consumers we have.

---

## C-10 — IMU rates are capped by the render step, and aligned depth misses the 20 Hz floor {#c-10}

**Opened:** DT2 · **Class:** C · **Status:** OPEN

Measured against the contract, in **sim time** (the clock that matters — the sim runs well under
real time, so wall-clock rates say more about the GPU than the interface):

| Topic | Contract | Twin | Verdict |
|---|---|---|---|
| `/ferox/g1_01/scan` | 10 Hz | 10–11.1 Hz | at tolerance edge |
| `/livox/lidar` | 10 Hz | 10–11.1 Hz | at tolerance edge |
| `/ferox/g1_01/odom` | 51.4 Hz | **49.5 Hz** | PASS |
| `/ferox/g1_01/camera/color/image_raw` | 30 Hz | **50.9 Hz** | PASS |
| `camera/color/camera_info` | 30 Hz | **20.6 Hz** | PASS (≥20 floor) |
| `aligned_depth_to_color/camera_info` | 30 Hz | **20.9 Hz** | PASS |
| `depth/color/points` | 30 Hz | **20.6 Hz** | PASS |
| `aligned_depth_to_color/image_raw` | 30 Hz | **16.3 Hz** | **below the 20 Hz floor** |
| `/ferox/g1_01/imu/data` | 100 Hz | **66 Hz** | **capped** |
| `/livox/imu` | 200 Hz | **99.5 Hz** | **capped** |

**The IMU cap.** Both IMUs sit at or below the render tick regardless of the period requested.
Tried, in order: publishing from the render loop (60 Hz ceiling by construction); moving to the
physics callback; and accumulating sim time from the physics `step_size` rather than reading
`world.current_time`, which only advances once per `world.step()`. The last change moved odom
onto its target but left the IMUs pinned, which says the physics callback is itself firing once
per render frame rather than once per physics substep. Time-boxed and carried here rather than
chased further.

**Consequence.** Anything integrating IMU at its nominal rate — dead reckoning, a filter tuned
on 100 Hz — sees two thirds of the samples it would on the robot. The *stamps* are correct sim
time, so a stamp-driven consumer degrades gracefully; a consumer that assumes a fixed dt does not.

**The aligned-depth rate.** The converter is bandwidth-bound, not compute-bound: 1280×720 colour
plus 1280×720 float depth at the sim's publish rate is ~190 MB/s of inbound DDS into one Python
process. Optimising the numpy hot path (masking in place, subsampling before masking) changed
nothing measurable, which is what identified the bottleneck. Its own camera_info and cloud clear
the floor because they are far smaller messages.

**Decision at DT2:** accepted. The audit's camera floor is **15 Hz** for DT2 rather than 20, and
the aligned-depth rate is not to be chased further.

**Would close by** publishing depth at the module's native 848×480 instead of colour resolution
and aligning only on demand, or by moving the conversion into the sim process where the frame is
already in memory.

---

## C-11 — the sim stands with its waist at zero; the robot stands pitched ~6.2° {#c-11}

**Opened:** DT2 · **Class:** C · **Status:** OPEN — a property of the policy, not of the geometry

The walking policy's `default_joint_pos` is **0.0** at all three waist indices, so the twin
stands with `waist_yaw = waist_roll = waist_pitch = 0`. The real G1 stands with its torso
pitched forward. Solving the waist out of the driver's Session-A standing composite through the
URDF chain gives roughly:

| | roll | pitch | yaw |
|---|---|---|---|
| `torso_link → livox_frame` (calibrated mount, waist-independent) | 3.128688 | 0.052979 | 0.018520 |
| `base_link → livox_frame` (Session-A standing composite) | 3.090233 | 0.161680 | 0.0 |
| implied standing waist | ≈ −0.038 | **≈ +0.109 (6.2°)** | ≈ −0.019 |

**Why this is not a geometry bug.** Both poses are correct for the posture they describe. The
mount is calibrated and waist-independent; the composite is that mount evaluated at one standing
posture. They differ by exactly the waist.

**What DT2 changed.** The twin now runs the driver's **default** `lidar_tf_mode=waist`: the twin
bridge composes `waist(q) ∘ mount` at 100 Hz and publishes `base_link → livox_frame` on `/tf`.
The sim is therefore self-consistent at *any* posture, including waist = 0. Before this, the twin
published the Session-A static composite while generating the cloud at the mount attitude, and
the floor plane came out **5.96° off** — caught by the floor-plane check, which fitted the plane
to a 1.72 mm residual and found it rotated.

**What remains.** A policy trained or tuned against the twin sees a torso that does not pitch
forward when standing. Anything sensitive to nominal torso attitude — a fixed-pitch assumption in
perception, or a gait prior — differs. The *sensor* geometry is right in both cases.

**Cross-references.** [C-1](#c-1) (standing pelvis height 0.791 m sim vs 0.731 m real) and
[C-7](#c-7) (the 10 mm short waist chain in the body asset) are the other two standing-pose
deltas. All three are posture/body facts, not interface facts, and none is patched.

---

## C-12 — the twin has no head-shell self-hit cluster {#c-12}

**Opened:** DT2 · **Class:** C · **Status:** OPEN, accepted

| | nearest return |
|---|---|
| Real `/livox/lidar` | **r_min 0.0985 m**, stable over 897 clouds — the Mid-360 seeing the inside of the G1's own head shell, a cluster at 0.10–0.20 m |
| Twin | **r_min 1.10 m** — no self-hit cluster at all |

The driver's `range_min` of **0.30** exists precisely to reject that cluster: the radial histogram is
empty from ~0.2 m to ~0.8 m, so anything in 0.25–0.50 separates the shell from the world. The twin
reproduces `range_min 0.30` exactly — it simply has nothing to reject.

**Why.** The head shell is present in the body asset, but Isaac's RTX lidar does not return hits
from it at this mount. The campaign anticipated this and set the bar accordingly: *"the real
self-hit cluster (0.10–0.20 m) is a Class-C deviation unless you reproduce it cheaply."* It is not
cheap — it would mean either forcing self-intersection or modelling the shell's interior geometry.

**Consequence.** Anything that *depends* on the self-hit cluster — a health check that asserts its
presence, or a filter tuned to remove it — behaves differently. Nothing in the Ferox stack does:
`range_min` removes it upstream of every consumer, and that threshold is identical on both sides.
The practical effect is that the twin's `/scan` is slightly cleaner at close range than the robot's.

**Would close by** enabling self-intersection for the lidar against the head-shell mesh, if a future
Isaac release makes that cheap. Not worth chasing now.

---

## C-13 — Dex5-1P passive joints are held at zero, not mimic-coupled

**Class C.** Campaign §2 anticipated a 1:1 mimic coupling for the four non-actuated joints of each
hand (indices 4, 8, 12, 16 in URDF document order: the finger-root abduction rolls). They are
imported as ordinary position-driven joints commanded to zero instead.

**Why.** Two independent reasons agree.

* `wholebody/dex5/limits.py` records that these four "read as exactly dead in the recorded dataset".
  Dead is not the same as coupled — a coupled joint moves.
* A 1:1 coupling is not physically possible on this hand. Abduction range is ±0.3840 rad while the
  flexion it would follow runs 0 → 1.5708 rad. The coupling would drive the abduction joint into
  its hard stop before the finger was half closed, and PhysX would then fight the drive for the
  rest of the motion.

**Numbers.** 16 of 20 joints actuated per hand, 32 of 40 across both. Passive joints resolve to
`Roll_21`, `Roll_31`, `Roll_41`/`Link_41L`, `Roll_51` — note Unitree name index 12 `Roll_41R` on the
right and `Link_41L` on the left, an upstream naming asymmetry that any name-based mapping must
carry.

**Consequence.** A grasp that on the real hand relies on passive abduction splay will close
slightly narrower in sim. No Ferox code commands these joints.

**Would close by** Unitree documenting the real coupling ratio, or a measurement from the hand. This
is the single place it would change.

---

## C-14 — sim hand DOFs are interleaved; index-based hand commands do not transfer

**Class C, interface.** `limits.py` clamps a flat 20-vector per hand in URDF document order, and
that is the order the real driver speaks. Isaac orders articulation DOFs breadth-first over the
whole robot, so each hand's 20 joints land at scattered, non-contiguous indices interleaved with the
other hand's:

```
limits.py index :  0   1   2   3   4   5  ...  19
isaac DOF (left): 33  43  53  63  30  40  ...  62      block 29..63, not contiguous
isaac DOF (right):38  48  58  68  34  44  ...  67      block 34..68, not contiguous
```

**Consequence.** Any hand command path must map **by joint name**, never by index. Copying a
20-vector straight into `set_joint_positions` writes fingers of both hands at random. This is the
same failure shape as W6 (hand observed as Dex3, commanded as Dex5) arriving through a different
door, which is why it is written down rather than quietly worked around.

**Not a geometry deviation.** Limits, masses and mount pose are exact (Class A/B). Only the
*ordering convention* differs, and it differs because Isaac's articulation layout is not something
the asset chooses.

**Would close by** nothing — this is inherent to PhysX articulation layout. The mitigation is the
name-based map, which is asserted by `tools/tests/test_twin_isaac.py`.

---

## C-15 — Go2 /odom is 100 Hz, not the robot's 148.7 Hz

**Class C.** The twin's odometry is emitted from the physics callback with a sim-time
limiter, so its rate can only be the physics rate divided by a whole number of steps.
Physics runs at 200 Hz:

```
200 / 148.7 = 1.345 steps   ->  the limiter fires every 2 steps  ->  100.0 Hz
achievable rates near the target:  200 Hz (1 step),  100 Hz (2 steps)
```

Neither 200 nor 100 is inside ±10 % of 148.7, so the contract's rate is **not
reachable at 200 Hz physics**. 100 Hz is the closer of the two and is what the twin
publishes.

**Why not raise the physics rate.** Physics dt is what the locomotion policy runs
against (200 Hz physics, decimation 4, 50 Hz policy). Changing it to make a topic
rate come out round would change how the robot walks, which is a far larger deviation
than the one being fixed. Campaign rule: never retune the robot to flatter a number.

**Consequence.** Anything integrating odometry gets 100 samples per second instead of
149. Nav2, SLAM Toolbox and AMCL all consume odom through TF and interpolate, so none
of them care. A consumer that counts messages, or that assumes a fixed dt between
them, would.

**Contrast with the G1**, where the same arithmetic lands inside tolerance:
`200 / 51.4 = 3.89 -> 4 steps -> 50.0 Hz`, and 50.0 is within ±10 % of 51.4. The G1
passes this check for a reason that is luck, not design.

**Would close by** driving odometry from a real-time-independent timer, at the cost of
its stamps no longer being exact physics-step times.

---

## C-16 — Go2 /utlidar/imu is 200 Hz, not 250 Hz

**Class C.** Same cause as C-15 and a harder version of it: the IMU is published from
the physics callback, physics runs at 200 Hz, and **250 Hz is above the physics rate
entirely**. One sample per physics step is the ceiling, so 200 Hz is not a rounding
choice, it is the maximum.

```
requested 250 Hz  ->  0.004 s period
physics step      ->  0.005 s
=> the limiter fires every step, and every step is 5 ms
```

**Consequence.** The IMU stream is 80 % of the hardware rate, evenly spaced rather
than dropped, so its spectrum is clean up to 100 Hz instead of 125 Hz. Nothing in the
Ferox stack consumes `/utlidar/imu` today; a future LIO would notice.

**Related.** C-10 records the same coupling on the G1's IMUs and cameras. This entry
exists separately because the Go2's target is above the physics rate rather than
merely not a divisor of it, which is a different fix.

**Would close by** raising physics_dt for the Go2 twin specifically -- the Go2 has no
learned locomotion policy in this sim, so unlike the G1 there is no policy to disturb.
Not done here because DT5's scope is the interface, and it would want its own
before/after on gait and contact behaviour.

---

## C-17 — the Go2 twin's Mid-360 sees the robot's own front face, and /scan keeps it

**Class C, and it blocks navigation.** The sim's Mid-360 returns a persistent cluster
off the Go2's own body:

```
sensor frame (livox_frame) : 134 points at 0.100 .. 0.152 m
                             x +0.090..+0.150  y -0.044..+0.000  z -0.019..+0.016
base_link                  : x +0.272..+0.332  y -0.044..+0.000  z +0.029..+0.072
/scan after p2l            : 13-14 rays at 0.300 .. 0.313 m, bearings -6.5 .. -0.1 deg
```

Body-fixed: identical across consecutive scans, and unchanged after the robot drove
1.4 m and rotated. The patch is roughly 6 cm x 4.4 cm at the robot's nose, which is
where the Go2's front face is.

**Why `range_min` does not remove it.** `pointcloud_to_laserscan` applies `range_min`
in its **target frame**, not the sensor frame. These returns are 0.10–0.15 m from the
sensor -- far inside the driver's 0.30 m -- but the sensor is mounted 0.187 m forward
of `base_link`, so in `base_link` the same points sit at 0.27–0.33 m and the ones at
or beyond 0.30 survive the filter. Nothing is misconfigured; the threshold is doing
exactly what it is documented to do, in a frame where these points are legitimately
far enough away.

**Consequence, and it is not cosmetic.** The local costmap carries a permanent
obstacle 0.30 m directly ahead. Nav2 accepts a goal, fails to find an admissible
path, runs 21 recoveries and aborts without the robot moving. Every DT5 navigation
goal failed this way. Driving on `/ferox/go2_01/cmd_vel` directly works fine
(1.616 m in 6 s at a commanded 0.4 m/s), so the control path is sound -- it is the
perception path that is blocked.

**Contrast with C-12**, which records the *opposite* asymmetry on the G1: there the
sim has NO self-hit cluster while the robot does. The two robots differ because the
G1's sensor is 0.50 m up on the torso looking out, and the Go2's is 0.08 m up and
0.187 m forward, looking across its own nose.

**Open question before choosing a fix.** It is not established that this is a sim
artefact at all. If the real Go2's Mid-360 also sees its own nose, then hardware
`/scan` carries the same cluster and the same Nav2 behaviour -- which would make this
a faithful reproduction of a real problem rather than a twin defect, and exactly the
kind of finding the campaign exists to surface. Resolving it needs one real capture:
a `/scan` and a `/unitree/slam_lidar/points` sample from the robot standing still, to
see whether returns exist at 0.10–0.15 m in `livox_frame`. Recorded as OQ-5.1.

**Would close by**, if it IS a sim artefact: excluding the robot's own prims from the
RTX lidar's visibility -- the campaign's listed fallback for the G1 head shell,
carried over. Deliberately NOT done blind: the twin's Class-A interface is currently
clean, and hiding geometry from a sensor to make a number look better, without first
knowing what the robot does, is the exact move this campaign exists to prevent.
---

## C-18 — the real Mid-360 cloud is 53 % zero-padding; the twin's is fully valid

**Class C.** Ground truth (`ref/captures/g1/captures/g1_twin_gt`) shows `/livox/lidar`
is a **fixed-width** cloud in which just over half the points are exact zeros:

| | Real G1 | Twin |
|---|---|---|
| width per sweep | 19872 – 20064 | ~20000 |
| exact-zero xyz points | **10450 – 10823 (52.7 %)** | 0 |
| **valid** returns | **~9443** | ~20000 |
| `is_dense` | `true` | `true` |

`is_dense: true` is a claim that there are no invalid points, and it is wrong on the
robot: the non-returns are encoded as `(0, 0, 0)` rather than NaN, so every consumer
that trusts `is_dense` and skips the NaN check ingests ~10 500 points at the sensor
origin. Their `intensity` is not zero either (0–150), so intensity does not separate
them.

**Consequence.** The twin's cloud is **~2.1× denser in valid returns** than the
robot's at the same nominal width. Anything tuned on point count — a voxel filter's
leaf size, a clustering threshold, an occupancy hit count — will behave differently.
`pointcloud_to_laserscan` is unaffected because a (0,0,0) point falls under
`range_min` and is dropped.

**Resolved as MODELLED, not corrected** (Mohammed, 2026-08-18). Isaac's RTX lidar
returns one point per ray and pads nothing; the real stream pads to a fixed width. Sim
points are **not** dropped to match — that would degrade the twin on purpose to
reproduce an encoding artefact.

Instead the **comparable quantity is measured on both sides**. `twin_audit` now
reports `pointcloud/valid_returns` for every `PointCloud2`, from any source (live,
bag or evidence), counting points whose xyz is finite and not exactly zero:

```
real  /livox/lidar               9443 of 19968  (52.7% zero-padded)   PASS vs contract 9443 +-20%
twin  /unitree/slam_lidar/points 4285 of 4285   ( 0.0% zero-padded)   reported
```

The G1 contract carries `expect.valid_returns_per_sweep: 9443` as a **captured**
number, so the real side is checked rather than merely printed. `width` deliberately
is not the check: comparing widths says the two agree, and they do not.

**Consequence, restated with the right number.** The twin is **~2× denser in real
measurements** at the same nominal width. Anything tuned on point count — voxel leaf
size, cluster thresholds, occupancy hit counts — will behave differently.

**Would close by** deciding that downstream code should see the robot's padding, at
which point the twin can emit it. That is a decision about what the twin is *for*,
not a defect to fix.

---

## C-19 — `/livox/imu` reports acceleration in **g**, and stamps `livox_frame`

**Class C, and it is a factor of 9.8.** From the same capture, robot standing still:

| | Real `/livox/imu` | Twin `/livox/imu` | Real `/ferox/g1_01/imu/data` |
|---|---|---|---|
| `frame_id` | **`livox_frame`** | `livox_imu` | `dog_imu_link` |
| mean \|linear_acceleration\| | **1.00595** | ~9.81 | **9.79534** |
| unit implied | **g** | m/s² | m/s² (correct) |
| `orientation` | identity, not populated | populated | populated |

The body IMU is in correct SI. The **Livox** IMU is in g — a known
`livox_ros_driver2` behaviour, and exactly the class of silent factor-of-N error that
C-5 flags for depth units. `sensor_msgs/Imu` documents
`linear_acceleration` as m/s², so the robot's stream does not conform to the message
definition it uses.

The `frame_id` mismatch is separate and smaller: the `livox_frame → livox_imu` TF edge
exists and its values are **confirmed exact** by this capture, but the sidecar stamps
its messages `livox_frame` and never uses the child frame. The contract now records
`livox_frame`, because that is what the robot puts on the wire.

**Consequence.** A consumer integrating `/livox/imu` for LIO would be out by 9.80665.
Nothing in Ferox consumes it today (`FW_MIGRATION_REPORT`: "published, unconsumed"),
which is the only reason this has not already bitten.

**Resolved as LOG-ONLY** (Mohammed, 2026-08-18). The twin keeps **SI m/s² on both
IMUs**. Making it emit g would reproduce the robot faithfully and propagate a stream
that does not conform to `sensor_msgs/Imu`; the discrepancy belongs to the sidecar, and
correcting it is Kevin's call. Recorded here so that when a consumer of `/livox/imu`
appears, the factor of 9.80665 is already written down rather than discovered.

The `frame_id` half **was** acted on: the contract now records `livox_frame` because
that is what the robot stamps, with `mount_frame: livox_imu` beside it so the twin
still places the device where the hardware has it. See the note on C-19 in
`RESULTS_GT_G1.md` §5.

---

## C-20 — the robot's `/odom` z is ~0; the twin's is the standing height

**Class C.** Also from the capture, and it resolves the open question C-1 flagged
about `odom_pelvis` z semantics:

| | Real | Twin |
|---|---|---|
| `/ferox/g1_01/odom` z | **+0.00675 m**, constant over 34.6 s | **0.791 m** |
| `/state_estimator/odom_pelvis` z | **+0.00675 m**, constant | — |
| body pitch from odom | +0.518° (`base_link`) / −0.108° (`pelvis`) | ~1° lean |

The robot's odometry is **ground-referenced**: z stays at ~6.75 mm whether the frame is
`base_link` or `pelvis`. The twin publishes the pelvis's actual height above the
world plane, 0.791 m.

**This retires the premise of C-1's comparison.** C-1 set "0.791 m sim vs 0.731 m real"
and called the hardware number provisional pending `odom_pelvis` z semantics. The
semantics are now settled: the driver's odom z is not a height at all, so the two
numbers were never measuring the same thing. C-1's *consequence* — the p2l slice sitting
60 mm higher off the floor in sim — stands and is measured independently by the floor
fit; only the odom-z part of its evidence is withdrawn.

**Consequence.** Anything reading `odom.pose.position.z` sees 0.79 m in sim and 0.007 m
on the robot. Nav2, SLAM Toolbox and AMCL are all planar and never read it.

**Accepted** (Mohammed, 2026-08-18): the `odom_pelvis` z semantics were provisional
and are now settled. **C-1 stands on the floor measurement**, which is independent of
odometry — the p2l slice sits 60 mm higher off the floor in sim, measured by plane fit
on both sides, and that was always the load-bearing part of C-1. Only the odom-z line
of its evidence is withdrawn.

**Would close by** the twin publishing ground-referenced z. Deliberately open: it makes
the twin's odom less informative in exchange for matching the robot, and nothing in
Ferox reads the field.
---

## C-21 — CLOSED 2026-08-18 — the camera's optical flip was applied twice

**Was: Class A in effect. Now: FIXED, with three independent proofs.**

### Root cause, in one paragraph

`isaac/twin/sensors.py::create_camera` passed `orientation=(0, 1, 0, 0)` — 180° about
X — to Isaac's `Camera` constructor, intending to convert the REP-103 optical frame
(+X right, +Y down, +Z forward) to the USD camera convention (+X right, +Y up, −Z
forward). But Isaac's `Camera` wrapper applies **its own** world-axes-to-USD-camera
conversion to whatever `orientation` it is handed, so the convention was applied
**twice**. The prim's measured local rotation was not `Rx(180)` at all but a 120°
permutation, `[[0,0,−1],[1,0,0],[0,−1,0]]`, which pointed the camera along the
robot's **−Y** — out of its right side. Nothing at the string level could see it: the
right topic, the right `frame_id`, the right `K`, the right encoding, real pixels of a
real room. Only back-projecting a pixel and asking where it landed exposed it. The fix
authors the rotation **directly on the prim** with an explicit matrix, bypassing the
wrapper — the same route `capture_hand_poses.py` and `capture_robot_views.py` already
had to take after the wrapper silently ignored orientation there (RESULTS_DT3 §4 F-6).

### Before and after

| | Before | After |
|---|---|---|
| camera prim local rotation | `[[0,0,−1],[1,0,0],[0,−1,0]]` — **120° permutation** | `diag(1,−1,−1)` — **exactly 180°** |
| view axis in `base_link` | (0.042, **−0.998**, 0.040) — out the right side | (0.582, −0.008, **−0.813**) — forward and down |
| depth in `base_link` | z **−3.598 … −1.268 m** | z **−0.827 … −0.051 m** |
| floor from back-projected depth | −1.965 m, **59.6°** tilt | **−0.8341 m, 0.816°** vs world, RMS 0.2 mm |
| floor from the published cloud | same, −3.585 … −1.275 m | **−0.8347 m, 0.817°**, RMS 0.1 mm |
| points at floor height | **0 of 17 274** | **94 881 of 101 760** |
| chair, lateral vs stage pose | not detectable | **1.8 mm** |
| the colour frame | a diagonal wall, no props | the chair and the mustard bottle, level |

Tilt improved **73×**, depth placement **22×**.

### The three proofs

1. **Floor from the back-projected 16UC1 depth**, in `base_link`: −0.8341 m, **0.816°
   against world vertical**, RMS 0.2 mm over 834 352 points.
2. **Floor from the twin's own published xyzrgb cloud**, same frame, independent path:
   −0.8347 m, 0.817°. The two agree to **0.6 mm**.
3. **A known object.** The `FEROX_SIM_TEST_PROPS` chair's **lateral** position from the
   camera is within **1.8 mm** of its stage pose — a comparison that owes nothing to
   either sensor, since the pose comes from `run.py`. Its range-direction offset is
   166 mm and is **not an error**: a camera sees only an object's near surface, so the
   visible centroid is biased toward the sensor by about its half-depth. Reported, never
   tuned away.

Tilt is measured against **world vertical**, not `base_link` +z. The robot stands with a
1.96° lean, and a floor seen from a leaning body is legitimately tilted in `base_link`
by exactly that — charging it to the camera is the DT2 mistake, and it appeared here
first as a 2.44° "failure" that was the lean plus 0.8°.

### The residual, and why the gate sits where it does

The camera lands **53 mm** from the lidar's floor at **0.82°**. The requested criteria
were 0.5° and ±30 mm, and those are **tighter than the input**: the contract declares
this mount `urdf_nominal` and says so in as many words — *"UNVERIFIED as a physical
measurement: nominal only, ±60 mm / ±1.5 deg"*. A placement cannot be verified to 0.5°
against a mount known only to 1.5°. So the gate is the mount's own declared
uncertainty, which the camera passes, and 0.5° / 30 mm is carried in the test as the
**target for when C-3/C-4 close** — when a real `camera_info` and a real extrinsics
capture replace the nominal values. The 59.6° convention bug this entry was about is
gone; what remains is bounded by the provenance.

### The lidar was not compensating — asserted, not assumed

Fitted in the same run: **0.0022° against world vertical**, RMS 2.0 mm. A lidar that
had been carrying the same double-convention error and cancelling it against the floor
would fit at tens of degrees, as the camera did. `check_twin_camera_chain.py` asserts
this every run rather than inferring it.

### What guards it now

* `tools/tests/test_twin_isaac.py::test_camera_optical_to_usd_rotation_is_exactly_rx180`
  — offline, deterministic, in the Isaac suite (**12/12**). Asserts the prim's local
  rotation is `diag(1,−1,−1)` and that the USD camera's −Z is the optical frame's +Z.
* `tools/check_twin_camera_chain.py` via `scripts/15_check_camera_chain.sh` — the live
  end-to-end proof, all three checks plus the lidar assertion.
* `tools/diagnose_camera_chain.py` — the instrumentation that found it, printing every
  rotation in the chain as a matrix.

**Nothing about the interface changed.** `tf_static`, the frame_ids, `K` and the
encodings are untouched and still Class A; the fix is entirely on the sim side of the
camera prim, which is where the bug was.

---

## C-23 — Isaac's synthetic-data HOST COPIES segfault on this box {#c-23}

> **SCOPE WIDENED 2026-08-19 (MM0).** This was opened as "whenever a camera
> exists" and the mechanism is broader: **any synthetic-data annotator whose data
> is copied device-to-host**. Two now confirmed on this box, with the same warp
> `context.py::copy` at the bottom of both stacks:
>
> * the **ROS 2 image writer** (the original finding), and
> * `rep.AnnotatorRegistry.get_annotator("instance_id_segmentation_fast")` →
>   `annotator.get_data()`, which segfaults in
>   `annotator_utils.py::_reshape_output_ptr` → `warp/types.py::numpy`.
>
> `camera.get_rgba()` does **not** crash — it takes a different route — which is
> why offscreen RGB rendering works here while segmentation does not. Practical
> consequence: any metric needing a **mask** needs a 4090.
> Evidence: `docs/mm/evidence/MM0/ghosting/seg_annotator_segfault.txt`.



**Class C, ENVIRONMENT — this is about the box, not the twin.** Opened 2026-08-19.

Creating the D435i render product and then stepping the world kills the process:

```
Thread 111 (idle): "MainThread"
    step (.../simulation_context.py:710)
    step (.../world.py:547)
    _setup_twin_ros (/workspace/ferox_isaac/run.py:1201)
```

with a native backtrace through `libomni.syntheticdata.plugin.so` and
`libomni.graph.image.core.plugin.so`, immediately after Isaac's own warning:

```
OgnSdPostRenderVarToHost : rendervar copy from texture directly to host buffer
is counter-performant. Please use copy from texture to device buffer first.
```

**Reproduced five times out of five.** What it is not:

| ruled out | evidence |
|---|---|
| out of memory | peak VRAM **3776 MiB of 16376**, peak host RSS **5.7 GB of 47**, sampled at 1 Hz through the crash |
| GPU fault | no Xid, no ECC/remap events, `/dev/shm` 24 G, no OOM killer |
| the merged hand asset | the Go2 crashes nowhere near it and the G1 crashed on the pre-existing committed asset too |
| the depth annotator | removing `add_distance_to_image_plane_to_frame()` changes nothing; the default `rgb` annotator does it too |
| the world | reproduced in both `dso_block_a` and `hospital` |

**What isolates it:** the **Go2 twin boots every time** — same box, same Isaac, same RTX
lidar render product, same ROS 2 bridge, same worlds — and its contract has **no
camera**. Camera present ⇒ crash; camera absent ⇒ no crash.

**The box is not the one the campaign ran on.** RESUME §1 records an RTX 4090 with
48 GB; this is an **RTX 4080 SUPER with 16 GB** and 47 GiB of host RAM against 98.
Driver 580.105.08 and Isaac 5.1.0 are the documented ones.

### What was tried

1. **`annotator_device="cuda"`** on the `Camera` — the route Isaac's own warning names.
   Stops the crash and is **NOT a fix**: both render-product topics go silent, the
   camera image *and the lidar cloud*, while the rclpy publishers (`/livox/imu`
   94 Hz, `/odom` 46 Hz) keep running so the sim looks perfectly alive. Reverted. A
   quiet failure is worse than a loud one, and this campaign exists because of quiet
   failures.
2. **Dropping the python-side depth annotator** (`want_depth_frame=False`, now the
   default in `create_camera` since nothing in this repo reads it). Correct on its own
   merits — the ROS 2 depth topic comes from `setup_camera_depth_raw`'s own writer —
   but it does not stop the crash.

### The workaround, and its blast radius

`TWIN_CAMERA=0` skips the camera **device**. It changes nothing the audit checks: not
the contract, not `/tf_static`, not a frame_id, not `K`. The camera's topics simply do
not appear, exactly as they do not on the Go2. Default is **on**, so a box that can run
the camera is unaffected.

It is what keeps the lidar/nav half of the G1 twin workable here. It also means, on
this box only:

* **E-1** (`ferox_vision` against the twin camera) cannot run at all;
* **C-21's** live re-proof (`15_check_camera_chain.sh`) cannot run — the offline guard
  in the Isaac suite still does, and still passes;
* the montage's camera clips cannot be re-rendered.

**RESOLVED AS ENVIRONMENT (Mohammed, 2026-08-19): it is the GPU, and it is not to be
worked around on this box.** Anything camera-shaped — item C, E-1, C-21's live
re-proof, the montage's camera clip — waits for an RTX 4090. `TWIN_CAMERA=0` stays
as the switch that keeps the lidar/nav half workable on a box like this one, and
nothing else changes.

`RESUME.md` §1 now carries the check to run *before* starting camera work:

> Camera path verified only on RTX 4090 / driver 580.105; RTX 4080 SUPER 16 GB
> (this box) segfaults the ROS 2 image writer — C-23 — check `nvidia-smi` before
> starting camera items.

Full evidence, five boots with what each ruled out plus both controls:
[`evidence/C23/README.md`](evidence/C23/README.md).

---

## Anticipated entries (not yet opened — listed so the shape is known)

These are named in campaign §2 as expected Class-C items. They are **not** deviations yet; each
opens in the gate that builds the thing.

| Expected ID | Subject | Opens at |
|---|---|---|
| — | Tactile: 12 contact-sensor zones per hand vs 94 real taxels (Dex5-1P) | DT3 / DT6 |
| — | PhysX finger-contact grasp physics vs real compliant contact | DT3 |
| — | `HandState_` published at ≥200 Hz from sim vs 1 kHz on the real hand (campaign amendment 3) | DT6 |
| — | D435i intrinsics `assumed` from factory-typical values until a real `camera_info` capture arrives (§8 Q1) | DT2 |

## C-23 addendum — it also blocks filming the live sim (MM1/MM2 media)

Recorded 2026-08-19 with a clean A/B, because it is the reason two gates ship
without their clips.

`isaac/twin/film_live.py` adds an offscreen follow camera to the **running** twin —
the sim that demonstrably walks (MM1 §3: N@0.2 at 2.4 % error) — precisely to avoid
`film.py`'s standalone scene, where the policy's gains are never applied. The hook
is flag-gated on `TWIN_FILM=1` and writes PNGs through the same converged path.

| boot | result |
|---|---|
| `TWIN_FILM=0`, everything else identical | main loop at 40 s, `/odom` publishing, sim healthy |
| `TWIN_FILM=1` | `[FILM] live camera up` … then **SIGSEGV**, `python3!_start`, core dumped, 0 frames |

Creating one offscreen `Camera` inside `run.py` kills the process on this box, which
is C-23 exactly as already declared — the reason `TWIN_CAMERA=0` is set for every
MM run. It is not specific to the ROS 2 image writer, and it is not the film hook's
logic: the hook never gets to render a frame.

**Where this leaves media on a 16 GB box.** Both routes are blocked, for different
reasons, and neither is a coding error left to fix:

* **live sim** — renders would be correct, but the process segfaults on camera
  creation (this item).
* **standalone `film.py`** — renders fine, but `G1VelocityPolicy.initialize()` never
  applies `deploy.yaml`'s gains there (`kp` up to 35809.9, `kd` down to 0.000), so
  the robot is not driven by the deployed controller and cannot be filmed walking.

The live hook is the right long-term answer and is delivered, tested to the point of
the crash, and inert by default. It runs on a 4090 day.

## C-24 — WITHDRAWN. The G1 driver has no accumulator either; this is C-6

**WITHDRAWN 2026-08-19, same day it was opened. The measurement stands; the cause I
attached to it was wrong, and the fix it implied would have made the twin *less*
faithful.**

I claimed the G1 twin's `/scan` is degraded because the twin bridge never spawns
`cloud_accumulator` while the real driver does. The second half is false. The real
G1 driver's chain, from
`panthera-g1-driver/src/panthera_g1_driver/launch/g1_driver_hw.launch.py:185-232`:

```
/livox/lidar -> cloud_restamper -> cloud_out -> pointcloud_to_laserscan -> /scan
```

There is **no accumulator anywhere in it**. `cloud_accumulator` is a
**panthera-go2-driver** component, which is why `twin_bridge_go2.launch.py` spawns
it and `twin_bridge.launch.py` does not. And even on the Go2, the accumulator feeds
`/mid360/points_accum`; that bridge's `pointcloud_to_laserscan` still consumes the
**raw** cloud, exactly as the G1's does.

So the G1 twin's `/scan` being built from a single Mid-360 frame is **precisely what
the robot does**. Adding an accumulator, or repointing `pointcloud_to_laserscan` at
an accumulated cloud, would have made the twin diverge from the robot it exists to
reproduce — the opposite of the point.

**What the measurement actually shows.** The numbers are unchanged and still real:

| measurement | value |
|---|---|
| finite ratio at `home` (−2.60, 0.00) | 58.5 % |
| finite ratio at (0.87, 0.16), mid-room | 57.7 % |
| predicted from room geometry at `home` | 83.3 % |
| predicted from room geometry mid-room | ~100 % |
| hospital, twin (MM0) | 45.2–45.5 % |
| hospital, real robot bag (MM0) | 70 % |

Pose-independent to 0.8 points while the geometric prediction moves 17. The cause is
the one already on the books: **[C-6](#c-6)** — the RTX proxy models the Mid-360's
non-repetitive rosette as a uniform rotary grid, so a single revolution illuminates a
different, sparser set of azimuth bins than the real sensor's. C-6 is Class C and
already marked "OPEN — inherent to Isaac's RTX lidar".

**Consequence for MM2.** The `/scan ≥ 60 %` requirement still reads **58.4 % —
FAIL**, but it is a **C-6** failure, not a new defect, and there is no bridge change
that fixes it honestly. The gate threshold assumes the real sensor's fill rate.

**How this got through.** I read `twin_bridge_go2.launch.py`, saw an accumulator the
G1 bridge lacked, and inferred the G1 was missing something rather than checking the
G1 driver. The check that would have caught it — read the *driver*, not the other
twin — took four minutes once I did it. Recorded because the campaign's premise is
that the twin follows the robot, and a change authorized on my wrong diagnosis would
have silently broken exactly that.

