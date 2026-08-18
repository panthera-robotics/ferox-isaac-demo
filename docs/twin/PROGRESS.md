# PROGRESS — overnight run 2026-08-18

## RUN SUMMARY — stopped on a PING at 03:55Z

| Gate | Verdict |
|---|---|
| **DT0** | PASSED (accepted), tag `twin-DT0` pushed |
| **DT1** | PASSED (accepted), tag `twin-DT1` pushed |
| **DT2** | **BLOCKED on a decision** — interface is Class-A conformant (84 pass / **0 Class-A FAIL** / 5 Class-B); geometry has one 5.96° fork. Not tagged. |
| **DT3** | not started |
| **DT5** | not started |

### The three decisions I need

1. **The waist pose fork — the blocker.** The sim's policy stands with all three waist joints at
   0; the real G1 stands pitched ~6.2°. So the cloud is generated at the calibrated *mount*
   attitude while `/tf_static` announces the driver's standing *composite*, and the floor plane
   comes out 5.96° off (tolerance 0.5°, fit residual 1.72 mm over 1884 points — the plane is real
   and flat, just rotated). Three options with costs are in **`docs/twin/PING.md`**;
   **I recommend option A** (publish `base_link→livox_frame` dynamically, which is the driver's
   *default* waist-bridge mode — Session A captured the *static* fallback).
2. **`/ferox/g1_01/scan` and `/livox/lidar` sit at 10–11.1 Hz** against 10 Hz ±10 %. Isaac rounds
   the render step to a whole number of physics substeps (0.015 s = 66.67 Hz) while
   `get_rendering_dt()` still reports the requested 0.016667. Clean fix is `--render_dt 0.02`
   (exactly 4 physics substeps → 50 Hz → decimation 5 → exactly 10 Hz) which leaves `physics_dt`
   and therefore the policy untouched. Want me to apply it?
3. **Aligned depth runs 16.3 Hz** against the ≥20 Hz floor — the converter is bandwidth-bound at
   ~190 MB/s inbound, not compute-bound (optimising numpy changed nothing). Fix is to publish
   depth at the module's native 848×480 and upsample in the converter. Accept 16.3 Hz as
   Class C ([C-10](TWIN_DEVIATIONS.md)), or spend the time?

### Exact next command

```bash
cd ~/panthera/ferox-isaac-demo
# after deciding the PING: reboot the twin, then finish the DT2 evidence
TWIN=1 ROBOT=g1 ROBOT_ID=g1_01 ./scripts/01_start_sim.sh
MODE=twin ROBOT=g1 ROBOT_ID=g1_01 VENUE=dso_block_a ./scripts/02_start_ferox.sh
ROBOT=g1 ./scripts/07_twin_audit.sh --duration 20
docker exec ferox_nav bash -lc 'source /opt/ros/humble/setup.bash;
  source /workspace/install/setup.bash; export ROS_DOMAIN_ID=42;
  python3 /tmp/twin_geometry_check.py --robot-id g1_01'
```

### DT2 state — what is done and pushed

Contract-driven twin publishing the hardware interface: `/tf_static` exactly the 7 Session-A
edges at contract values (`camera_link` an orphan root under `CAMERA_TF=0`), `/livox/lidar` full
10 Hz sweeps, `/livox/imu`, `imu/data`, `odom` at 49.5 Hz, colour at 50.9 Hz with K reading back
908.000/908.000/640.000/360.000, `/scan` 723 rays in `base_link` with 100 % of finite returns in
band, the converter supplying 16UC1 mm aligned depth and an xyzrgb cloud with the colour stamp,
`mode:=twin` in Ferox, and the USD sensor layer generated from the contract and self-verified.

Still to do once the fork is settled: 3 nav goals, 4 visual PNGs, `validate_motion` re-run against
the pre-change baseline, `ferox_vision`, `RESULTS_DT2.md`, tag `twin-DT2`.

### Pushes this run (all token-clean, verified after each)

| Repo | Range |
|---|---|
| ferox-isaac-demo | `0a3a99d..ef1cbac..5507023..1d3b543` |
| Ferox | `b405014..306724e..52eb95e` |

---


Running log. Updated at every task boundary and after every failure/retry, so this
file is readable if the instance dies mid-gate.

Health at start: disk 240 G free, `ferox_isaac_sim` + `ferox_nav` up, GPU 4.4/49 GiB.

| timestamp (UTC) | gate | task | status | note |
|---|---|---|---|---|
| 2026-08-18T01:51 | DT2 | overnight run start | START | decisions received: converter in ferox_nav_sim, C-7 into audit as runtime round-trip, stale driver doc line -> RESULTS open question |
| 2026-08-18T01:58 | DT2 | run.py twin wiring | RETRY | twin booted, Mid-360 authored at /World/G1/torso_link/livox_frame/mid360; camera creation died silently. Cause: optical frame authored as Xform but Camera() needs a Camera prim, and a camera at identity in a ROS optical frame faces backwards. Fix: camera prim as child, 180 deg about X. Also made twin setup failures print a traceback instead of a bare app shutdown. |
| 2026-08-18T02:05 | DT2 | run.py twin wiring | DONE | twin up: Mid-360 at torso_link/livox_frame/mid360, D435i K readback exactly 908/908/640/360 (HFOV 70.36 deg derived), /tf_static exactly the 7 hardware edges, camera_link an orphan root with CAMERA_TF=0 |
| 2026-08-18T02:12 | DT2 | twin publishers | RETRY | /clock and odom->base_link TF lived inside the legacy setup_ros_publishers monolith; extracted to setup_clock_publisher()/setup_odom_tf_publisher() and called from both paths. IMU reproduced baseline defect B-4 (ROS2PublishImu builds, logs success, advertises nothing) -> switched to rclpy, matching the precedent sim_utils already set for cmd_vel. |
| 2026-08-18T02:20 | DT2 | twin publishers | DONE | live twin topics: /clock /tf /tf_static(7 hw edges) /livox/lidar /ferox/g1_01/{odom,imu/data,camera/color/image_raw,camera/depth/image_rect_raw_32f} /joint_states. B-4 fixed for the twin path via rclpy. |
| 2026-08-18T02:20 | DT2 | converter node | DONE(code) | ferox_nav_sim/realsense_twin_bridge.py + twin_bridge.launch.py + bringup include gated on mode==twin. Deps (numpy/rclpy/sensor_msgs) already baked in ferox/nav — nothing installed at runtime. |
| 2026-08-18T02:35 | DT2 | twin bridge launch | RETRY | mode:=twin failed: "libexec directory .../lib/ferox_nav_sim does not exist". Cause: ferox_nav_sim had no setup.cfg, so setuptools installed the console_script to bin/ instead of lib/<pkg>. ferox_nav already carries that file; added the matching one. |
| 2026-08-18T02:50 | DT2 | twin bridge running | RETRY→DONE | converter received nothing: subscribed BEST_EFFORT while the sim publishes RELIABLE, and 2.7/3.7 MB frames fragment and drop wholesale over best-effort. Switched to RELIABLE. |
| 2026-08-18T02:58 | DT2 | audit vs twin | PASS | **0 Class-A failures — conformant on Class A.** 76 pass / 9 Class-B / 10 skipped / 95 checks. Fixed a real audit bug first: it measured WALL-clock rate, but the sim runs well under real time, so every rate looked like a failure about the GPU rather than the interface. Now measured from header stamps (sim time). |
| 2026-08-18T03:05 | DT2 | rate parity | IN PROGRESS | Class-B rates showed everything at the ~66 Hz render tick. /livox/lidar at 66 Hz means every message is a PARTIAL SWEEP, not a full revolution — a partial sweep still looks like a valid cloud to every consumer. Added SimulationGate decimation to the sensor's scan rate, and moved the IMU to the physics callback (render loop tops out ~60 Hz sim, so the contract's 100 Hz was unreachable from there). |
| 2026-08-18T03:20 | DT2 | audit vs twin (2nd) | PASS | 82 pass / **0 Class-A** / 7 Class-B / 99 checks. /scan 10.99 Hz and /livox/lidar 10.87 Hz now PASS — full sweeps, not partial. camera colour 52.77 Hz PASS. |
| 2026-08-18T03:20 | DT2 | push | DONE | ferox-isaac-demo ef1cbac..5507023, Ferox 306724e..52eb95e. Token via GIT_CONFIG_* env only; residue check clean. |
| 2026-08-18T03:25 | DT2 | rate parity (IMU/odom) | IN PROGRESS | IMUs still pinned at the render rate: world.current_time only advances per world.step(), so the sim-time limiter let exactly one publish through per render frame. Switched to accumulating physics step_size. Odom moved to rclpy at the contract rate (no divisor of 60 lands on 51.4); the TF graph stays. |
| 2026-08-18T03:40 | DT2 | rate parity | TIME-BOXED | 84 pass / 0 Class-A / 5 Class-B. odom 49.5 Hz PASS after the rclpy move; camera_info + points cleared the 20 Hz floor. Root-caused the lidar overshoot: Isaac quantises rendering_dt to a multiple of physics_dt, so 1/60 became 0.015 s = 66.67 Hz and my decimation divided by an assumed 60. Now reads the actual rendering dt. IMU cap and aligned-depth rate recorded as C-10 and moved on per the 90-min rule. |
| 2026-08-18T03:55 | DT2 | geometry check | **PING** | waist round-trip PASS (but tautological — twin publishes the contract's own static values). Floor plane FAIL: 5.96 deg tilt, 1.72 mm residual over 1884 pts. Root cause: the sim's policy stands with waist joints at 0, the real G1 stands pitched ~6.2 deg, so the cloud is generated at the MOUNT attitude while TF announces the COMPOSITE. Took the listed fallback (publish static composite + PING). docs/twin/PING.md has the three options and a recommendation. /scan geometry PASS: 723 rays in base_link, 100% in band. |
| 2026-08-18T04:30 | DT2 | Option A (waist bridge) | PASS | geometry check RESULT: PASS. Round-trip (a) live-vs-composed 0.106 mm / 2.45e-4 rad; (b) Session-A composite solves back to waist pitch +0.108101 rad (+6.19 deg) and the chain reproduces it EXACTLY (0.000 mm, 0.000e+00 rad). Floor plane: body lean 0.9783 deg, tilt vs base_link +z 0.9761 deg (that IS the lean), **tilt vs world vertical 0.0039 deg** — comparable to the driver's own 0.0027 deg residual. |
| 2026-08-18T04:32 | DT2 | audit | PASS | 83 pass / **0 Class-A** / 5 Class-B / 98 checks. /scan exactly 10.00 Hz after --render_dt 0.02; tf_dynamic base_link->livox_frame PASS on /tf at 50 Hz. |
