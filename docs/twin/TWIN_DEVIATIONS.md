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
| [C-1](#c-1) | C | G1 standing height: 0.791 m sim vs 0.731 m real (~60 mm) | DT1 | OPEN — re-check at DT2 |
| [C-2](#c-2) | C | Mid-360 PointCloud2 field layout is not recorded for either robot's live stream | DT1 | OPEN — G1 defaulted to the sidecar driver's layout at DT2; Go2 still unknown |
| [C-3](#c-3) | C | D435i intrinsics (K/D) are factory-typical placeholders | DT1 | OPEN — needs OQ-1 |
| [C-4](#c-4) | C | RealSense intra-camera TF values unknown (edges confirmed, numbers absent) | DT1 | OPEN — needs OQ-1 |
| [C-5](#c-5) | C | Aligned-depth metric scale (mm) is convention, never written down | DT1 | OPEN — needs OQ-1 |
| [C-6](#c-6) | C | Mid-360 non-repetitive rosette modelled as a uniform rotary grid | DT2 | OPEN — inherent to Isaac's RTX lidar |
| [C-7](#c-7) | B→C | Sim USD waist chain is 10 mm shorter than the robot's URDF | DT2 | OPEN — body asset, not patched by design |

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

**Re-check (DT2).** With the twin G1 standing, confirm **no floor returns enter `/scan`**. The
60 mm of extra clearance makes floor bleed *less* likely in sim than on hardware, so a clean sim
result does **not** clear the hardware case — it only fails loudly if something else is wrong.
Record the measured minimum return height in `RESULTS_DT2.md`.

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

## Anticipated entries (not yet opened — listed so the shape is known)

These are named in campaign §2 as expected Class-C items. They are **not** deviations yet; each
opens in the gate that builds the thing.

| Expected ID | Subject | Opens at |
|---|---|---|
| — | G1 head-shell self-hit cluster (real returns at 0.10–0.20 m) not reproduced unless cheap | DT2 |
| — | Tactile: 12 contact-sensor zones per hand vs 94 real taxels (Dex5-1P) | DT3 / DT6 |
| — | PhysX finger-contact grasp physics vs real compliant contact | DT3 |
| — | `HandState_` published at ≥200 Hz from sim vs 1 kHz on the real hand (campaign amendment 3) | DT6 |
| — | D435i intrinsics `assumed` from factory-typical values until a real `camera_info` capture arrives (§8 Q1) | DT2 |
