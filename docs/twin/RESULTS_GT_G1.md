# G1 ground truth — the twin measured against the robot

**Source:** `ref/captures/g1/captures/g1_twin_gt` — rosbag2, **34.571 s**, 52 210
messages, 10 topics. G1 #1 standing still, driver + livox sidecar up, **no camera
container**. Tarball kept at `~/panthera/g1_twin_gt.tgz`.

**Verdict: the twin holds up.** Every Class-A check the bag can answer passes, and two
things the contract had been carrying on faith since DT1 are now confirmed exactly
right. One Class-A mismatch is real and is a **PING** (`docs/twin/PING.md`): the robot
publishes `base_link → livox_frame` as a *static* edge, and the contract says dynamic.

`/lowstate` was **not decoded** — the nav image has no `unitree_hg`, so 35 998 of the
bag's messages are unreadable here. The standing waist pitch below comes from the TF
edge instead, which the task allowed.

---

## 1. Headline

| | Result |
|---|---|
| Audit against the bag | 35 pass, **11 Class-A FAIL**, 1 Class-B fail, 14 skipped |
| Of those 11 Class-A | **9 are the absent camera** (5 topics + 4 TF edges) — the capture had no camera container |
| Genuine Class-A mismatches | **1** — `base_link → livox_frame` static vs dynamic → **PING** |
| Genuine Class-B mismatches | **1** — `/livox/imu` frame_id → contract number, fixed |
| Closed by this capture | **OQ-2**, **OQ-3**, four provenance upgrades to `captured` |
| New deviations | **C-18** (zero padding), **C-19** (IMU in g), **C-20** (odom z semantics) |

---

## 2. Sim vs real, per check

`captured` = read out of this bag. Sim values from DT2 (`evidence/DT2/`).

### Topics, types, frames, rates

| Check | Real (bag) | Twin (sim) | Verdict |
|---|---|---|---|
| `/ferox/g1_01/scan` type + frame | LaserScan, `base_link` | same | **PASS** |
| `/ferox/g1_01/scan` rate | **10.00 Hz** | 10.0 Hz | **PASS** |
| `/ferox/g1_01/odom` type + frame | Odometry, `odom`, child `base_link` | same | **PASS** |
| `/ferox/g1_01/odom` rate | **51.46 Hz** | 49.5 Hz | **PASS** — closes OQ-3 |
| `/ferox/g1_01/imu/data` frame | `dog_imu_link` | `dog_imu_link` | **PASS** |
| `/ferox/g1_01/imu/data` rate | **93.70 Hz** | ~94 Hz | **PASS** (contract now 93.7, was 100) |
| `/livox/lidar` type + frame | PointCloud2, `livox_frame` | same | **PASS** |
| `/livox/lidar` rate | **10.00 Hz** | 9.78 Hz | **PASS** |
| `/livox/imu` rate | **200.00 Hz** | 99.57 Hz | real PASS; **sim fails its own contract** (C-10) |
| `/livox/imu` frame_id | **`livox_frame`** | `livox_imu` | **FAIL (B)** → contract corrected, C-19 |
| 5 camera topics | absent (no camera container) | present | not answerable |

### `/tf_static` — the edge set

The bag carries **3** edges. Session-A lists **7**. The four missing ones are exactly
the four camera edges, and the camera was not running — so the set reconciles: **3
present + 4 camera = 7**, with nothing unexpected and nothing missing.

| Edge | Real | Contract | Verdict |
|---|---|---|---|
| `base_link → dog_imu_link` | xyz `[0,0,0]`, rpy `[-0.000966, -0.010877, 0]` | identical | **PASS**, d = 0.00e+00 → `captured` |
| `livox_frame → livox_imu` | xyz `[0.011, 0.02329, -0.04412]`, rpy `[0,0,0]` | identical | **PASS**, d = 0.00e+00 → `captured` |
| `base_link → livox_frame` | **present, static**, xyz `[0,0,0.4995]`, rpy `[3.090233, 0.161680, 0]` | **dynamic on /tf** | **FAIL (A)** → **PING** |
| 4 × camera edges | absent | present | not answerable |

`/tf` carries **one** edge: `odom → base_link`, 400/400 samples. No waist-composed
lidar edge appears on it at all.

### LaserScan geometry

| Field | Real | Contract | Verdict |
|---|---|---|---|
| ray_count | **723** | 723 | **PASS** |
| angle_min / max | **∓3.14159274** | ∓3.14159265 | **PASS** |
| angle_increment | **0.00870000012** | 0.0087 | **PASS** |
| scan_time | **0.100000001** | 0.1 | **PASS** |
| range_min / max | **0.300000012 / 6.0** | 0.30 / 6.0 | **PASS** |
| finite fraction | **69.7 – 70.3 %** | — | reported |
| in-range fraction | **100.0 %** | 100 % | **PASS** |

### PointCloud2 layout — OQ-2, closed

| | Real | Contract (was `assumed`) |
|---|---|---|
| fields | `x,y,z,intensity` FLOAT32 @0,4,8,12; `tag,line` UINT8 @16,17; `timestamp` FLOAT64 @18 | **identical** |
| `point_step` | **26** | 26 |
| width / height | 19872 – 20064 / 1 | ~20000 |
| frame | `livox_frame` | `livox_frame` |

**The assumption was exactly right**, field for field and offset for offset. Provenance
`assumed → captured`. The twin still emits xyz-only (`point_step` 12) — that is C-8,
unchanged and still declared.

---

## 3. The self-hit signature, real vs sim

This is the comparison the Go2 audit's self-hit report was built for, and the G1 bag
answers it for the G1 first.

| | Real G1 (`/livox/lidar`) | Twin G1 | Go2 twin (C-17) |
|---|---|---|---|
| `r_min`, valid points | **0.0986 – 0.0991 m** | 1.10 m | 0.100 m |
| valid points < 0.30 m | **243 – 299** | 0 | 134 |
| cluster span | **0.099 – 0.206 m** | — | 0.100 – 0.152 m |
| cluster extent | x ±0.13, y ±0.15, z 0…0.14 m | — | 6 cm × 4.4 cm patch |
| **rays reaching `/scan` under 0.35 m** | **0** | 0 | **13–14** |
| `/scan` nearest return | **0.5599 – 0.5668 m** | 1.10 m | 0.300 m |

**Three things fall out of this.**

1. **C-12 is confirmed on the raw cloud and refined.** The real G1 *does* have a
   head-shell cluster — 243–299 points at 0.099–0.206 m, `r_min` 0.0986 m against the
   0.0985 m C-12 recorded — and the twin has none. That half of C-12 stands.
2. **But it never reaches `/scan` on either side.** The real scan's nearest return is
   **0.56 m**; there is nothing at all below 0.50 m. So the deviation is invisible to
   every `/scan` consumer, which is what C-12 predicted and is now measured rather
   than argued.
3. **This is exactly why the Go2 is different, and it is mount geometry, not a bug.**
   The G1's sensor is **0.4995 m up** on the torso, so its self-hits sit close to
   `base_link`'s axis and `range_min = 0.30` (applied in `base_link`) removes them.
   The Go2's sensor is **0.0803 m up and 0.187 m forward**, so its self-hits project to
   0.27–0.33 m in `base_link` and the ones past 0.30 survive. Same sensor, same
   filter, opposite outcome — because of where the sensor is bolted.

The Go2's hardware capture is still needed to close C-17; this does not substitute for
it. But it does show the mechanism is real and understood.

---

## 4. Standing waist pitch

`/lowstate` could not be decoded (no `unitree_hg` in the nav image), so this is from
the TF edge, as permitted.

| Quantity | Value |
|---|---|
| `base_link → livox_frame` pitch (the static composite) | **+0.161680 rad = +9.264°** |
| contract mount-only pitch | +0.052979 rad = +3.036° |
| **waist contribution baked into the edge** | **+0.108701 rad = +6.228°** |
| the 6.19° composite figure carried since DT2 | **+6.19°** |
| difference | **+0.038°** |

So the composite agrees with the figure DT2 carried, to 0.04°.

**But the robot was not in that pose.** Its own odometry reports body pitch **+0.518°**
(`base_link`) and **−0.108°** (`pelvis`), constant across all 34.6 s, and the floor
fitted through the bag's own TF comes out tilted **5.87 – 6.97°** with a 5–21 mm fit
RMS — i.e. tilted by the waist term that the edge assumes and the body did not have.

| sweep | floor z under `base_link` | tilt | RMS |
|---|---|---|---|
| 0 | −0.62527 m | 5.874° | 0.0211 m |
| 1 | −0.61723 m | **6.974°** | **0.0049 m** |
| 2 | −0.61750 m | 6.873° | 0.0094 m |

For contrast the **twin's** floor, with the dynamic edge DT2 chose, fits at
**0.0039°** against world vertical.

**RESOLVED** (Mohammed, 2026-08-18): **the contract stays dynamic — Option A stands.**
The bag shows the robot was running `lidar_tf_mode=static`, i.e. the Session-A
composite. That is a **robot-side configuration/verification item, not a twin defect**,
and it goes to Kevin as **OQ-6**.

So the twin is correct as it stands, and the 6.6° floor tilt measured above is a
property of how the robot was configured for this capture, not of the twin.

---

### OQ-6 — the robot ran `lidar_tf_mode=static`

**For Kevin.** In this capture the driver published `base_link → livox_frame` as the
static Session-A composite rather than composing it live from the waist. In that mode
the edge is only correct at the one waist angle it bakes in (+6.228°), and the robot
was standing level (odom pitch +0.518°), so the cloud it produced is tilted by very
nearly the whole waist term — measured at **5.87–6.97°** with a 5–21 mm plane-fit RMS.

Two questions worth putting together:

1. Was `static` deliberate for this capture, or is it the default?
2. Should the static fallback exist at all, given it is correct at exactly one waist
   angle and silently wrong at every other?

The one-line check on the robot, driver up:

```bash
ros2 param get /waist_tf_bridge lidar_tf_mode      # or: ros2 node list | grep waist
ros2 topic echo --once /tf | grep -A2 livox_frame  # is the edge ever on /tf?
```

**Expected if the default is `waist`:** the param reads `waist`, the node is listed,
and `base_link → livox_frame` appears on `/tf` at ~100 Hz — in which case this capture
used the fallback and the twin already matches the default.
**Expected if the default is `static`:** no such node or param, and `/tf` carries only
`odom → base_link` as it does in this bag.

The twin is unaffected either way: the contract keeps the dynamic edge, and the sim's
own floor fits at **0.0039°**.

---

## 5. What changed in the contract

| Item | Before | After | Why |
|---|---|---|---|
| `/livox/lidar` provenance | `assumed` | **`captured`** | OQ-2 closed — layout matched exactly |
| `/ferox/g1_01/odom` provenance | `configured` | **`captured`** | OQ-3 closed — 51.46 Hz measured |
| `/ferox/g1_01/imu/data` rate | 100.0 | **93.7**, `captured` | the robot delivers 93.70 Hz |
| `/livox/imu` frame_id | `livox_imu` | **`livox_frame`**, `captured` | what the sidecar actually stamps |
| `base_link → dog_imu_link` | `calibrated` | **`captured`** | confirmed to 0.00e+00 |
| `livox_frame → livox_imu` | `datasheet` | **`captured`** | confirmed to 0.00e+00 |
| `base_link → livox_frame` | dynamic | **unchanged — Option A stands** | robot ran `lidar_tf_mode=static`; robot-side item, now **OQ-6** |
| all camera items | `assumed` | **unchanged** | no camera in the capture, as instructed |

`captured` is a new provenance tier — stronger than `measured`, meaning "read out of a
rosbag recorded on the robot" — added to `tools/twin_contract.py` so a value a capture
has proven cannot be confused with one that merely agrees with a written-down number.

The Go2 contract is untouched and its audit stays in evidence mode until its own
capture lands.

## 6. New deviations

* **C-18** — the real cloud is **52.7 % exact zeros** with `is_dense: true`; ~9 443
  valid returns per sweep against the twin's ~20 000. **Modelled, not corrected:** no
  sim points are dropped. `twin_audit` now reports `pointcloud/valid_returns` for both
  sides, and the G1 contract carries `valid_returns_per_sweep: 9443` as a captured
  check. Real: `9443 of 19968 (52.7% zero-padded)`. Sim: `4285 of 4285 (0.0%)`.
* **C-19** — `/livox/imu` reports acceleration in **g** (|a| = 1.006), not m/s², and
  stamps `livox_frame`. The body IMU is correct SI (9.795). **Log-only:** the twin
  keeps SI on both IMUs; the sidecar discrepancy is Kevin's. The `frame_id` half was
  acted on, with `mount_frame` keeping the device where the hardware has it.
* **C-20** — the robot's `/odom` z is **+0.00675 m** (ground-referenced), the twin's is
  **0.791 m** (standing height). **Accepted:** the semantics were provisional and are
  now settled. **C-1 stands on the floor measurement**, which never depended on odom.

## 7. Reproduce

```bash
tar -xzf ~/panthera/g1_twin_gt.tgz -C ~/panthera/ref/captures/g1/
docker cp ~/panthera/ref/captures/g1/captures/g1_twin_gt ferox_nav:/tmp/gt
ROBOT=g1 ./scripts/07_twin_audit.sh --bag /tmp/gt
```

Evidence: `docs/twin/evidence/GT_G1/`.
