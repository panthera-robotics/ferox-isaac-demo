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
| [C-2](#c-2) | C | Mid-360 PointCloud2 field layout is not recorded for either robot's live stream | DT1 | OPEN — needs OQ-2 |
| [C-3](#c-3) | C | D435i intrinsics (K/D) are factory-typical placeholders | DT1 | OPEN — needs OQ-1 |
| [C-4](#c-4) | C | RealSense intra-camera TF values unknown (edges confirmed, numbers absent) | DT1 | OPEN — needs OQ-1 |
| [C-5](#c-5) | C | Aligned-depth metric scale (mm) is convention, never written down | DT1 | OPEN — needs OQ-1 |

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

**Decision.** Both contracts declare the captured `/utlidar/cloud_livox_mid360` layout, marked
in-file as a declared approximation. It is the only field layout anyone has actually measured
on this hardware. The campaign's stated fallback (§4.3: "if not derivable, xyz+intensity and
declare") would discard real information we hold, so we keep the richer measured layout instead
and declare it here.

**Closes when** OQ-2 lands: one `ros2 topic echo --no-arr` of each live cloud.

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

## Anticipated entries (not yet opened — listed so the shape is known)

These are named in campaign §2 as expected Class-C items. They are **not** deviations yet; each
opens in the gate that builds the thing.

| Expected ID | Subject | Opens at |
|---|---|---|
| — | Mid-360 non-repetitive rosette scan pattern approximated by an RTX rotary/solid-state proxy | DT2 (G1), DT5 (Go2) |
| — | G1 head-shell self-hit cluster (real returns at 0.10–0.20 m) not reproduced unless cheap | DT2 |
| — | Tactile: 12 contact-sensor zones per hand vs 94 real taxels (Dex5-1P) | DT3 / DT6 |
| — | PhysX finger-contact grasp physics vs real compliant contact | DT3 |
| — | `HandState_` published at ≥200 Hz from sim vs 1 kHz on the real hand (campaign amendment 3) | DT6 |
| — | D435i intrinsics `assumed` from factory-typical values until a real `camera_info` capture arrives (§8 Q1) | DT2 |
