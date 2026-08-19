# RESULTS_DT2 — G1 body + sensors at real poses, real wire

Host: Vast.ai VM (KVM), RTX 4090 49140 MiB, driver 580.105.08
Date: 2026-08-18
Verdict: **PASS-with-deviations** — every parity criterion passes; two behavioural criteria are
PARTIAL and are called out in their own rows rather than folded into a green verdict.

---

> ## AMENDED 2026-08-19 — the Mid-360 was publishing a SECTOR, not a sweep
>
> Everything below about geometry, frames, rates and the waist round-trip stands.
> What none of it asked is how much of the CIRCLE one published cloud covers, and
> the answer was about a fifth of it: each message was one render frame's slice of
> the sweep, and the decimation then sampled the same phase every time, so nothing
> downstream ever saw the rest. That is why `/scan` was ~20 % finite against the
> robot's 70 %, why SLAM built a fixed wedge, and — almost certainly — why no Nav2
> goal had ever succeeded.
>
> | | before | after |
> |---|---|---|
> | azimuth per message | ~72° | **360°**, 36/36 bins |
> | valid points per message | 4285 | **15466** |
> | `/scan` finite | ~20 % | **45.0–45.8 %** |
> | SLAM known cells about the centroid | a wedge | **36/36 bins** |
> | Nav2 goals SUCCEEDED | **0 of 3** | **1 of 3** |
> | cloud rate (sim time) | 10 Hz | **10.00 Hz**, unchanged |
>
> Cause: Isaac 5.1 registers two ROS 2 cloud writers and the twin used the
> non-accumulating one. `omni:sensor:Core:accumulateOutputs` **does not exist** —
> accumulation is an annotator choice, not a prim attribute. `twin_audit` now
> measures `pointcloud/azimuth_coverage` on every cloud, from any source, and it
> passes on the real bag too (340° in 34/36 bins).
>
> ### `/scan` at 45 %, and why that is the right answer for this scene
>
> **Accepted (Mohammed, 2026-08-19) as the correct scene answer, not a shortfall.**
> The brief asked for ≥ 50 % finite in `hospital` and the twin measures
> **45.0–45.8 %**. The missing rays are not missing — they are *correctly infinite*.
>
> The `hospital` spawn is **mid-corridor** at (7.8, 2.0) facing +y, with roughly 9 m
> of clear floor ahead and ~3.3 m to each side wall. `range_max` is **6.0 m**, copied
> from the driver. Every ray pointing down the long axis of that corridor therefore
> has nothing to return within range, and `pointcloud_to_laserscan` reports it as
> `inf` — which is the sensor being honest about a corridor longer than its range.
>
> The robot's own 70 % in the ground-truth bag was recorded **standing in a room**,
> where the walls are inside 6 m in most directions. The two numbers are measuring
> different scenes, not different sensors. A twin that reported > 50 % here would be
> inventing returns.
>
> Against the pre-fix ~20 %, the comparable statement is that the finite fraction
> **more than doubled** and the geometry never moved: 723 rays, ∓3.14159, increment
> 0.0087, 0.30–6.0 m, `base_link`.
>
> Navigation is **still PARTIAL**: goals 2 and 3 aborted outside the ~7.5 × 7.0 m
> the map had grown by then. Nothing was tuned to get goal 1. Accepted as **not a
> defect** — the evidence run was scoped wrong, and RESUME §0 carries the measured
> free-space bounds to scope the next one against.

## Scorecard

| # | Requirement | Status | Evidence |
|---|---|---|---|
| 1 | Mid-360 inverted in the head at the calibrated mount | PASS — `/World/G1/torso_link/livox_frame/mid360`, 40×500×10 Hz, 200 000 pts/s exactly | `twin_startup.txt` |
| 1 | D435i at the URDF pose, IMU, RealSense subtree frames | PASS — 8 frames authored into the USD sensor layer, verified 8/8 against the contract | `scripts/08_build_twin_assets.sh` |
| 1 | Stand-ins removed from the G1 path | PASS — no `Example_Rotary`, no RPLIDAR, no `camera_optical`, no `/unitree_lidar` in twin mode | `audit_twin_g1.txt` |
| 2 | `/ferox/g1_01/{scan,odom,imu/data}` | PASS | audit |
| 2 | `/livox/lidar` + `/livox/imu` | PASS — 200 Hz Livox IMU | audit |
| 2 | Five camera topics | PASS — colour, colour info, aligned depth, aligned depth info, xyzrgb cloud | audit |
| 2 | `/tf` (`odom→base_link`) and `/tf_static` = hardware edge set | PASS — 6 static edges + 1 dynamic; `camera_link` an orphan root at `CAMERA_TF=0` | audit, `geometry_check.txt` |
| 2 | p2l with the driver's parameters | PASS — reproduced to the digit | `twin_bridge.launch.py` |
| 2 | `mode:=twin` in Ferox | PASS — enumerated; `mode:=sim` unchanged | Ferox `52eb95e` |
| 3 | Waist round-trip ≤1e-3 rad / ≤2 mm | **PASS** — both halves, see below | `geometry_check.txt` |
| 4 | Body visual pass | PARTIAL — viewport captured; the 4-angle set was not produced | `twin_viewport_hospital.png` |
| 5 | Nav2 + SLAM, 3 goals | **PARTIAL** — loop closes and the robot navigates; no goal reaches terminal SUCCEEDED | `nav_goals_twin.txt` |
| 5 | `ferox_vision` against the twin camera | NOT RUN — deferred | — |
| P | `twin_audit` zero Class-A diffs | **PASS** — 0 Class-A, 83 pass, 98 checks | `audit_twin_g1.txt` |
| P | `/scan` 723 rays, driver's angle params | **PASS** — exactly 10.00 Hz | audit |
| P | Camera 1280×720 `rgb8`/`16UC1`, same stamp | **PASS** — exact-stamp paired | audit |
| P | Rates within tolerance | PARTIAL — 5 Class-B, all in C-10 | audit |
| P | Floor-plane vs TF ≤0.5° | **PASS** — 0.0039° | `geometry_check.txt` |
| P | Policy tracking unchanged | **PASS** — verdicts identical, velocities within noise | `validate_motion_{baseline,twin}.txt` |

---

## The two results worth reading in full

### Waist round-trip — the geometry is right to machine precision

```
(a) live vs mount-composed   d_t 0.106 mm (tol 2.0)   d_r 2.45e-04 rad (tol 1e-03)   PASS
(b) Session-A composite implies waist yaw -0.012232 roll -0.039958 pitch +0.108101 rad
    (+6.19 deg pitch); the chain reproduces it: d_t 0.000 mm, d_r 0.000e+00 rad        PASS
```

(b) is the load-bearing one. Solving the waist back out of the driver's Session-A standing
composite lands on **+6.19° pitch**, and our URDF chain reproduces that composite *exactly*. Our
kinematics and the robot's calibration agree to machine precision — and it independently confirms
the posture delta recorded as C-11.

### Floor plane — and why the first version of this check was wrong

```
robot body lean (odom) : 0.9783 deg
tilt vs base_link +z   : 0.9761 deg    <- that IS the lean
tilt vs world vertical : 0.0039 deg    (tol 0.5)                          PASS
```

The check originally compared the fitted normal against `base_link`'s own +z, which charges the
sensor calibration for the robot's posture: the policy balances, so the body leans ~1° and the
floor genuinely *is* tilted in that frame. Against gravity-aligned world vertical it is
**0.0039°** — the same order as the driver's own 0.0027° calibration residual.

### Policy tracking — unchanged

| leg | baseline vx / vy / wz | twin vx / vy / wz |
|---|---|---|
| fwd | +0.507 / +0.002 / +0.011 | +0.508 / +0.002 / +0.012 |
| reverse | −0.350 / −0.006 / +0.007 | −0.349 / −0.011 / +0.015 |
| strafeR | +0.020 / −0.260 / +0.015 | +0.021 / −0.260 / +0.016 |
| walk+strafe | +0.320 / +0.232 / −0.023 | +0.320 / +0.233 / −0.015 |
| walk+turn | +0.307 / −0.002 / +0.164 | +0.311 / −0.007 / +0.157 |

Verdicts identical line for line: forward PASS, reverse PASS, strafe PASS, **rotate FAIL**,
upright PASS. `rotate` fails in **both** runs and the harness labels it "known FAIL on this
checkpoint" — pre-existing, not caused by DT2. `zmin` 0.77–0.79 in both, so adding the real
sensors did not change the standing height.

---

## Nav2: PARTIAL, and not tuned until green

| goal | recoveries | distance_remaining at last feedback | terminal |
|---|---|---|---|
| (8.2, 1.2) | 1 | 0.88 m | timeout (220 s) |
| (8.8, −0.2) | 3 | 0.28 m | timeout (220 s) |
| (7.4, −1.4) | 1 | 1.11 m | timeout (220 s) |

The loop closes — pose feedback tracked (7.80, 1.99) → (8.07, 1.35) → (8.74, 0.24) — but no goal
reached terminal SUCCEEDED. Goal 2 stalling at 0.28 m with recoveries firing is a controller
sitting just outside `xy_goal_tolerance`, not a broken interface. Three contributing factors, all
Nav2-side and all real:

* the map is small **because the sensor is honest** — `range_max` 6.0 in a corridor yields ~3.4 m²
  of free space, where DT0's stand-in (30 m, horizontal) would have mapped the whole floor;
* the G1's `cmd_vel` clamps are tight (+0.4/−0.2 x, ±0.2 y, ±0.5 wz), leaving little authority to
  close a final 0.3 m;
* footprint 0.35 m against inflation 0.35 m — **Nav2 warns about this itself**: *"configured
  inflation radius (0.350) is smaller than the computed inscribed radius (0.360)"*.

This is Ferox tuning work, not twin work, and it is the campaign functioning as designed: the same
gap will appear on hardware, because the sensor envelope is now identical. **Goal coordinates were
not tuned until three passed** — a result tuned until it looks right is not evidence.

### Venue: `hospital`, not `dso_block_a`

`dso_block_a` cannot exercise this sensor. At x = 4.36 m — where DT0's stand-in saw returns at
5.87 m — the twin reports **in-band 0 of 723 rays** and a 0×0 map. The raw cloud explains it:
p50 3.24 m (floor), a gap, then walls at p95 19.85 m. Nothing but floor inside `range_max` 6.0.
DT0 only cleared it because its stand-in was a *horizontal* 30 m lidar. In `hospital` (corridor
walls ~3.3 m) the twin gives **136 of 723 rays in-band**, nearest 3.99 / p50 4.88 / farthest 5.76,
and SLAM maps immediately. Interface evidence is world-independent and unaffected.

---

## Deviations (Class C)

| ID | Subject |
|---|---|
| C-1 | Standing height — **re-checked at DT2**; floor at −0.8215 m by plane fit, so the gap is ~90 mm not 60. **No floor returns enter `/scan`**, 0.2655 m margin |
| C-6 | Mid-360 rosette modelled as a uniform rotary grid |
| C-7 | Sim USD waist chain 10 mm shorter than the URDF |
| C-8 | Isaac RTX cloud is xyz-only (`point_step` 12, not 26) |
| C-9 | Rendered depth has no stereo occlusion shadows |
| C-10 | IMU rates capped by the render/physics coupling; aligned depth 16.1 Hz |
| C-11 | Sim stands with waist = 0; the robot pitched ~6.2° |
| C-12 | No head-shell self-hit cluster (sim r_min 1.10 m vs 0.0985 m real) |

---

## Open questions for Mohammed

1. **`validate_motion.py:28` is legacy-sim-shaped.** `ODOM_TOPIC = '/odom'` — the twin and the
   robot both publish `/ferox/<id>/odom`. I ran the harness with a ROS remap rather than editing it
   (identical bytes is the whole basis of the comparison), but the constant should move to the
   namespaced topic or become a parameter in `ferox-g1-locomotion`. Exactly the "works in sim, not
   on the robot" gap this campaign exists to remove — found in our own tooling.
2. **`doc/05-validation.md §1` in `panthera-g1-driver` is stale**: it states the G1 does *not* run a
   Livox sidecar, which contradicts the `fw2026q3` evidence (`livox_driver_node`, `/livox/lidar` at
   10.000 Hz) and `phase1_diffs.patch`. Read-only repo, not edited — a Kevin/Mohammed item.
3. **Nav2 tuning for a 6 m sensor.** Inflation 0.35 vs inscribed 0.360 is a live warning; the
   goal-tolerance behaviour above needs a look before hardware.
4. **Deferred from DT2**: `ferox_vision` against the twin camera, and the 4-angle visual pass.

---

## Reproduce

```bash
cd ~/panthera/ferox-isaac-demo
./scripts/08_build_twin_assets.sh g1          # regenerate + self-verify the sensor layer
./scripts/10_test_twin_isaac.sh               # 6 Isaac sensor tests
python3 tools/tests/test_twin_contract.py     # 32 contract tests

TWIN=1 ROBOT=g1 ROBOT_ID=g1_01 SIM_WORLD=hospital ./scripts/01_start_sim.sh
MODE=twin ROBOT=g1 ROBOT_ID=g1_01 VENUE=dso_block_a ./scripts/02_start_ferox.sh
ROBOT=g1 ./scripts/07_twin_audit.sh --duration 20

docker cp tools/twin_geometry_check.py ferox_nav:/tmp/
docker exec ferox_nav bash -lc 'source /opt/ros/humble/setup.bash;
  source /workspace/install/setup.bash; export ROS_DOMAIN_ID=42;
  python3 /tmp/twin_geometry_check.py --robot-id g1_01'

# policy regression — nav stack MUST be down (6 publishers on cmd_vel otherwise)
source scripts/lib/kill_nav_stack.sh && kill_nav_stack ferox_nav --verify
docker exec ferox_nav bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42;
  python3 /tmp/validate_motion.py --ros-args -r /odom:=/ferox/g1_01/odom'
```
