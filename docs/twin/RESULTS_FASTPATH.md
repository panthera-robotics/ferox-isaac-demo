# Fast path — DT2 · DT3 · DT5

One page for the whole fast path: what each gate proved, every declared deviation in one
table, and the work that is still outstanding. Written to be read on its own; the
per-gate reports (`RESULTS_DT2.md`, `RESULTS_DT3.md`, `RESULTS_DT5.md`) carry the
detail and the evidence paths.

Branch `mohammed/twin-campaign`. Tags `twin-DT2`, `twin-DT3`, `twin-DT5`, `twin-fastpath`.

---

## 1. Scorecards

### DT2 — the G1 twin — **PASS with deviations**

| Criterion | Result |
|---|---|
| Interface conformance | **84 pass / 0 Class-A FAIL / 5 Class-B** |
| `/tf_static` set | 6 static edges + `base_link→livox_frame` dynamic at 100 Hz (Option A) |
| Waist round-trip | agrees with the driver's kinematics **to machine precision** |
| Floor plane vs world vertical | **0.0039°** (tolerance 0.5°) |
| Floor returns in `/scan` | **none** — `min_height` sits 0.2655 m above the floor |
| Camera K read-back | within **1 %** of the contract, solved from K not guessed |
| Nav2 + SLAM, 3 goals | **PARTIAL** — loop closes and the robot navigates; no goal reached SUCCEEDED inside 220 s |
| Deviations opened | C-6 … C-12 |

The Nav2 shortfall is Ferox-side tuning, deliberately not tuned: footprint 0.35 m against
an inflation radius of 0.35 m (Nav2 itself warns the inscribed radius is 0.360), tight
`cmd_vel` clamps, and a small map because the sensor is honest at `range_max 6.0`.

### DT3 — Dex5-1P hands on the G1 — **PASS**

| Criterion | Result |
|---|---|
| Articulation | **69 DOF, 79 bodies, one root** at `/g1_29dof_rev_1_0/pelvis` |
| Total mass | **35.004757 kg** vs 35.004757 expected — **0.00 %** |
| Body joint order | **bit-identical** to the pre-hand order (the walk policy indexes it) |
| Hand joints present | 40/40, limits match the URDF to **≤0.01°**, `Roll_12` mirror preserved |
| Mount offset | exact to **0.0000 mm**, read from the G1's own `*_hand_palm_joint` |
| Walks with hands on | **yes** — same verdicts as DT2, max base-height delta **0.02 m** (allowance 0.03) |
| Hand poses | rest / open / fist / thumb opposition, all reached to **≤0.001 rad** |
| Isaac test suite | **11/11** |
| Deviations opened | C-13, C-14 |

### DT5 — the Go2 twin — **PASS on the interface, PARTIAL on navigation**

| Criterion | Result |
|---|---|
| Interface conformance | **45 pass / 0 Class-A FAIL / 3 Class-B** |
| Namespacing | `/scan` and `/odom` at the **root**, control namespaced — as the driver does |
| Absent by design | no camera, no `/ferox/go2_01/imu/data` |
| `/tf_static` set | **5 edges**, incl. `utlidar_imu` and `robot_center`, all exact to **0.00e+00** |
| Lidar rate | **20.0 Hz** (render 40 Hz, decimation step 2) |
| LaserScan geometry | exact: 723 rays, ∓3.14159 truncated, increment 0.0087, range 0.30–6.0 |
| `cmd_vel` clamps | verified against **three** agreeing sources; provenance → `calibrated` |
| Control path | **works** — 1.616 m in 6 s at a commanded 0.4 m/s |
| Nav2 + SLAM, 3 goals | **BLOCKED** — accepted, 21 recoveries, aborted, robot never moved (**C-17**) |
| Deviations opened | C-15, C-16, C-17; C-8 extended to the Go2 cloud |

---

## 2. Every declared deviation, C-1 … C-17

Class C throughout: approximated or unknown, declared rather than silently carried.
"Closes on" says what would actually retire it — several close on one robot capture.

| ID | Subject | The number | Opened | Closes on |
|---|---|---|---|---|
| **C-1** | G1 stands ~60 mm taller in sim | pelvis 0.791 m sim vs 0.731 m real; p2l slice lifted +0.060 m | DT1 | firmware settling `odom_pelvis` z semantics |
| **C-2** | Real Mid-360 cloud field layout recorded nowhere | `/livox/lidar` and `/unitree/slam_lidar/points`: rate + frame captured, **fields never** | DT1 | **OQ-5.1 capture** (`--no-arr` echo) |
| **C-3** | D435i intrinsics are factory-typical placeholders | K = [908, 0, 640, 0, 908, 360, 0, 0, 1], `assumed` | DT1 | one real `camera_info` |
| **C-4** | RealSense intra-camera TF values unknown | 4 edges confirmed to exist, **no transform values** | DT1 | one `tf2_echo` per edge |
| **C-5** | Aligned-depth metric scale is convention | 16UC1; mm assumed, `depth_units` never set anywhere | DT1 | one `depth_module.depth_units` read |
| **C-6** | Non-repetitive rosette modelled as a uniform grid | Isaac RTX has no such `scanType`; point budget spent on a uniform grid | DT2 | inherent — no closing action |
| **C-7** | Sim USD waist chain 10 mm short | `pelvis→torso_link` 0.044 vs URDF 0.054 | DT2 | recorded, deliberately not patched |
| **C-8** | Mid-360 cloud carries xyz only | `point_step` 12 vs 26 (G1) / 22 (Go2) | DT2 | an Isaac release with more channels |
| **C-9** | Rendered depth has no stereo occlusion shadows | no baseline ⇒ no invalid-pixel band | DT2 | inherent |
| **C-10** | IMU/camera rates capped by the render step | aligned depth **16.3 Hz** against a 20 Hz floor (relaxed to 15 for DT2) | DT2 | 848×480 native + aligned on demand |
| **C-11** | Sim stands waist=0; robot stands pitched ~6.2° | policy property, not geometry | DT2 | a policy that stands like the robot |
| **C-12** | Twin has **no** head-shell self-hit cluster | real r_min **0.0985 m**; twin r_min **1.10 m** | DT2 | RTX self-intersection becoming cheap |
| **C-13** | Dex5 passive joints held at zero, not mimic-coupled | indices 4, 8, 12, 16; abduction ±0.3840 rad vs flexion 0→1.5708 | DT3 | Unitree documenting the real coupling |
| **C-14** | Sim hand DOFs are **interleaved** | left 29…63, right 34…68, neither contiguous | DT3 | inherent — mitigated by **RULE-HAND-NAME** (§4) |
| **C-15** | Go2 `/odom` 100 Hz, not 148.7 Hz | 200/148.7 = 1.345 ⇒ 2-step limiter ⇒ 100.0 Hz | DT5 | a rate source independent of physics steps |
| **C-16** | Go2 `/utlidar/imu` 200 Hz, not 250 Hz | 250 Hz is **above** the 200 Hz physics rate | DT5 | raising `physics_dt` for the Go2 only |
| **C-17** | Go2 Mid-360 sees the robot's own nose; `/scan` keeps it | 134 pts at 0.100–0.152 m sensor / 0.272–0.332 m base_link ⇒ 13–14 rays at 0.300–0.313 m | DT5 | **OQ-5.1** decides sim-side or real |
| **C-18** | Real Mid-360 cloud is 52.7 % zero padding | valid returns 9443 real vs ~20 000 twin | GT-G1 | **modelled**, both sides measured |
| **C-19** | `/livox/imu` reports **g**, not m/s², stamps `livox_frame` | \|a\| = 1.006 vs 9.795 on the body IMU | GT-G1 | log-only; Kevin's |
| **C-20** | Robot `/odom` z is ground-referenced | +0.00675 m real vs 0.791 m twin | GT-G1 | accepted; C-1 stands on the floor fit |
| **C-21** | **Twin camera 3D points miss by metres** | floor lands **−1.97 m** at **59.6°** tilt vs −0.79 m level | E-1 | confirm the double-convention hypothesis |

Note the pairing of **C-12** and **C-17**: on the G1 the sim is *missing* a self-hit
cluster the robot has; on the Go2 the sim *has* one whose hardware counterpart is
unverified. Same physical question, opposite signs, and one capture answers both.

---

## 3. Outstanding evidence — two items

Both are deliberate carry-overs, not oversights. Each is scoped and can be done
independently of the robot.

### E-1 — `ferox_vision` against the twin camera — **half done, and it found C-21**

**Interface half: PASS.** `tools/check_twin_camera.py`, evidence in
`evidence/DT2/twin_camera_check.txt`:

| topic | result |
|---|---|
| `camera/color/image_raw` | **`rgb8`**, 1280×720, step 3840, 19.95 Hz — PASS |
| `camera/aligned_depth_to_color/image_raw` | **`16UC1`**, 1280×720, step 2560, 3.96 Hz — PASS |
| both `camera_info` | **K = [908, 0, 640, 0, 908, 360, 0, 0, 1]** exact, `plumb_bob`, `camera_color_optical_frame` — PASS |
| `camera/depth/color/points` | 17 050 pts, `point_step` 16, fields `[x, y, z, rgb]` — PASS |

So a consumer's *subscriptions* are satisfied bit-for-bit: the encodings, sizes, steps
and intrinsics are the hardware ones. Aligned depth at 3.96 Hz is well under C-10's
already-relaxed floor, because this run carries the hands and a heavier world — noted,
not a new deviation.

**Detection half: CANNOT RUN HERE.** There is no `ferox_vision` image on this box
(`docker images` has only `ferox/nav`, `ferox/msgs`, `isaac-sim`), no detector runtime
in the nav image (no torch, no onnxruntime, no TensorRT) and no rtdetr weights
anywhere. Nothing was stubbed in to make a number appear. When the image is available:

```bash
ROBOT=g1 TWIN=1 HAND=dex5_1p SIM_WORLD=dso_block_a \
  FEROX_SIM_TEST_PROPS=1 CAMERA_TF=1 ./scripts/01_start_sim.sh
ROBOT=g1 MODE=twin ./scripts/02_start_ferox.sh
python3 tools/check_twin_camera.py          # interface half, expect RESULT: PASS
# then bring ferox_vision up against /ferox/g1_01/camera/color/image_raw
```

**Expected:** mustard bottle ~0.75 confidence, chair ~0.4 on its wheeled base
(`run.py:_add_test_props` documents both, and why the cracker box is omitted).

**And it found a defect: C-21.** Closing this item is what prompted actually
back-projecting a pixel, and the twin's 3D camera points land **1.2 m too low and
59.6° off** from where its own TF says. 2D is fine; anything doing 3D from the camera
is metres out. `tools/check_twin_camera_chain.py` is the check, it currently FAILS by
design, and it is what will close C-21.

### E-2 — the visual pass — **G1 done; Go2 bracket still open**

**Four PNGs delivered** (`scripts/14_capture_views.sh`, `tools/capture_robot_views.py`):

| PNG | Shows |
|---|---|
| `evidence/DT2/g1_twin_front.png` | full body, head shell, Unitree livery, **both Dex5 hands attached and open** |
| `evidence/DT2/g1_twin_side.png` | profile, head shell, wrist flange, leg chain |
| `evidence/DT2/g1_twin_top.png` | from above — **no puck visible on the G1**, which is correct: its Mid-360 is internal |
| `evidence/DT2/g1_twin_tf_tree.png` | the live TF tree with per-edge rates |

Plus the camera's own view: `twin_camera_color.png` (real `dso_block_a` pixels through
the twin D435i) and `twin_camera_depth.png`.

**On the TF image: it is `tf2_tools view_frames`, not an RViz screenshot.** RViz needs a
logged-in desktop X session and this box has none — Isaac falls back to headless and any
GUI viewport renders empty, which is exactly what DT2's `twin_viewport_hospital.png`
turned out to be. The `view_frames` graph is arguably the better artefact anyway: it
shows the whole tree *and* every edge's rate, so `base_link → livox_frame` at
**50.46 Hz** (Option A's dynamic waist edge) and `map → odom` at 10.48 Hz from SLAM are
both visible as numbers rather than as a picture of some axes.

Two framing iterations were needed and are recorded in the tool: at 2.6 m Isaac's
default pinhole cropped both head and feet, and aiming below eye level tilted the frame
and clipped the head. A crop is not a blank frame, so the std check cannot catch it —
the geometry has to be set from the robot's extent.

**Still open:** the Go2's Mid-360 puck and mount bracket (**OQ-5.4**). Its frames are
authored and verified 5/5 exact, but they carry no geometry.

```bash
./scripts/14_capture_views.sh     # the G1 views
```

---

## 4. OQ-5.x — for the robot session

Each item below is one command on the robot and one comparison against a number the sim
has already produced. Nothing here needs the sim running.

### OQ-5.1 — does the real Go2's Mid-360 see its own nose? **(the important one)**

Settles **C-17**, and with it whether DT5's navigation failure is a sim artefact or a
faithful reproduction. Also supplies the field layout **C-2** has been missing since DT1.

**On the robot, standing still, driver up:**

```bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# (a) the cloud's field layout -- C-2
ros2 topic echo --once --no-arr /unitree/slam_lidar/points

# (b) near returns in the SENSOR frame -- C-17
ros2 topic echo --once /unitree/slam_lidar/points > /tmp/hw_cloud.txt

# (c) the same rays after p2l -- the one-to-one comparison
ros2 topic echo --once /scan > /tmp/hw_scan.txt
```

**Expected, if the twin is faithful:** `point_step 22` with fields
`x,y,z,intensity FLOAT32; ring UINT16; time FLOAT32`; a cluster of ~100–150 points at
**0.10–0.15 m** in `livox_frame`; and **13–14 rays at 0.300–0.313 m** in `/scan` at
bearings **−6.5°…−0.1°**.

**If (c) shows those rays** → C-17 is real, the twin is right, and the fix belongs in the
driver (a self-hit filter before p2l, or `range_min` applied in the sensor frame).
**If (c) is empty there** → C-17 is a sim artefact and the fix is an RTX visibility
exclusion on the Go2's own prims.

**The sim's side of that comparison, printed by the audit today:**

```
$ ROBOT=go2 ./scripts/07_twin_audit.sh --duration 20
C   SKIP   laserscan/self_hit      /scan   reported in base_link (<0.35 m)
                                           ->  14 rays 0.300-0.313 m in 1 run(s)
C   SKIP   laserscan/self_hit_run  /scan   azimuth span of one contiguous run
                                           ->  14 rays  -6.5..-0.1 deg
```

The report is emitted **unconditionally**, pass or fail, and always in the scan's
target frame — because its whole job is to sit next to the hardware capture and be
compared line-for-line. Run the same audit against a bag from the robot
(`--bag /path/to/bag`) and the two blocks line up directly.

Run count is reported as contiguous **bearing runs**, not just a total: one run is one
object. The Go2's is a single 6.4° run off its own nose. A wall ahead would be one wide
run; sensor noise would be many tiny ones. The shape is what tells them apart.

### OQ-5.2 — is the unenforced acceleration cap intentional?

`Ferox src/ferox_nav/config/robots/go2.yaml:31-32` caps acceleration
(`max_linear_x 1.5 m/s²`, `max_angular_z 2.0 rad/s²`). `cmd_vel_to_sport` clamps velocity
only and never reads these; the twin does the same.

```bash
grep -n "max_linear_x\|max_angular_z" \
  ~/panthera/ref/panthera-go2-driver/src/panthera_go2_driver/panthera_go2_driver/cmd_vel_to_sport.py
```

**Expected:** hits at lines 146–149 only (the velocity clamps). No acceleration reference.
**Question for Kevin/Mohammed:** dead config, or a gap the driver should close?

### OQ-5.3 — `planner_server` segfault

Reproduced and written up in **`docs/twin/ISSUE_planner_segfault.md`**, with a repro
script at `docs/twin/repro/planner_oob_segfault.sh`. No Ferox code was changed.

### OQ-5.4 — Go2 Mid-360 puck and bracket

Folded into **E-2** above.

### Carried from DT2

* `doc/05-validation.md` §1 has a stale line. The driver repo is read-only here — a
  Kevin/Mohammed item, unchanged since DT2.
* **OQ-1** (four robot-side outputs) and **OQ-2** (G1 `/livox/lidar` field layout) remain
  as accepted at DT1. OQ-2 is answered by OQ-5.1(a) run against the G1 instead.

---

## 5. What changed in the tooling on the fast path

* `tools/twin_audit.py` gained a **self-hit report** — every `/scan` ray under 0.35 m in
  `base_link`, with count, range span and azimuths, printed whether or not it passes.
  Purpose-built so the sim's C-17 numbers and the OQ-5.1 capture compare one-to-one.
* **RULE-HAND-NAME** is now a named rule in both contracts and in `CLAUDE.md`, with a test
  that fails if any consumer slices hand DOFs by index (§C-14).
* `tools/merge_dex5_urdf.py` + `tools/import_g1_dex5.py` — the merged-URDF path, and the
  record of why USD composition cannot work here.
* `tools/capture_hand_poses.py` — the offscreen-render pattern, with the failure modes.
* Isaac scripts stage in `/tmp/isaacrun` and set `PYTHONDONTWRITEBYTECODE=1`; both guard
  environment failures that cost real time on this run (RESULTS_DT3 §4 F-5).
