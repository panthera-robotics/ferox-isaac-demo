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
| [C-6](#c-6) | C | Mid-360 non-repetitive rosette modelled as a uniform rotary grid | DT2 | OPEN — inherent to Isaac's RTX lidar |
| [C-7](#c-7) | B→C | Sim USD waist chain is 10 mm shorter than the robot's URDF | DT2 | OPEN — body asset, not patched by design |
| [C-8](#c-8) | C | Mid-360 cloud is xyz-only; Isaac has no node that emits the livox field layout | DT2 | OPEN — Isaac bridge limitation |
| [C-9](#c-9) | C | Rendered depth has no stereo baseline, so no occlusion shadows | DT2 | OPEN — inherent to rendered depth |
| [C-10](#c-10) | C | IMU rates capped by the render/physics step coupling; aligned depth below the 20 Hz floor | DT2 | OPEN — floor relaxed to 15 Hz for DT2 |
| [C-11](#c-11) | C | The sim stands with waist = 0; the real G1 stands pitched ~6.2° | DT2 | OPEN — policy property, not geometry |
| [C-12](#c-12) | C | No head-shell self-hit cluster: sim r_min 1.10 m vs real 0.0985 m | DT2 | OPEN — accepted, `range_min` still reproduced |

---

## C-1 — G1 standing height is ~60 mm taller in sim than on hardware {#c-1}

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

## Anticipated entries (not yet opened — listed so the shape is known)

These are named in campaign §2 as expected Class-C items. They are **not** deviations yet; each
opens in the gate that builds the thing.

| Expected ID | Subject | Opens at |
|---|---|---|
| — | Tactile: 12 contact-sensor zones per hand vs 94 real taxels (Dex5-1P) | DT3 / DT6 |
| — | PhysX finger-contact grasp physics vs real compliant contact | DT3 |
| — | `HandState_` published at ≥200 Hz from sim vs 1 kHz on the real hand (campaign amendment 3) | DT6 |
| — | D435i intrinsics `assumed` from factory-typical values until a real `camera_info` capture arrives (§8 Q1) | DT2 |
