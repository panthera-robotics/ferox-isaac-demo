# PROGRESS — overnight run 2026-08-18 (DT2 → DT3 → DT5)

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
