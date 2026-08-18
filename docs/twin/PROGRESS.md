# PROGRESS — overnight run 2026-08-18

## RUN SUMMARY — DT2, DT3 and DT5 complete

Ordered DT2 -> DT3 -> DT5, unattended, per the overnight authorisation. All three are
tagged and pushed. No PING was raised: nothing hit the section-5 stop rules.

| Gate | Verdict | Tag |
|---|---|---|
| **DT0** | PASSED (accepted) | `twin-DT0` |
| **DT1** | PASSED (accepted) | `twin-DT1` |
| **DT2** | **PASS-with-deviations** — Class-A conformant; waist fork resolved by Option A | `twin-DT2` |
| **DT3** | **PASS** — Dex5-1P hands, 69 DOF in one articulation, robot walks | `twin-DT3` |
| **DT5** | **PASS on the interface, PARTIAL on navigation** — Class-A conformant (45/0); nav blocked by C-17 | `twin-DT5` |

### What changed

**DT3 — hands.** Mounting the hands by USD reference composes a stage that passes every
geometric check (40 joints, 2.003615 kg, mount exact to the micron) and cannot move a
finger: PhysX builds its articulation from ONE robot description. Switched to merging the
URDFs and importing once, which is NVIDIA's documented answer and Unitree's own. Result:
69 DOF, 79 bodies, one articulation root, mass 35.004757 kg at 0.00 %, and the first 29
body DOFs bit-identical to the pre-hand order. The robot walks with the hands on — same
verdicts as DT2, max base-height delta **0.02 m** against the 0.03 m allowance, no gains
touched. Four hand poses rendered, all reached to <=0.001 rad.

**DT5 — Go2.** Interface is Class-A clean and matches the driver: root `/scan` and `/odom`,
both clouds, `/utlidar/imu`, no camera, no namespaced IMU, five static edges exact to
0.00e+00. `cmd_vel` clamps verified against three agreeing sources. Navigation does not
work, for one measured reason (C-17).

### The three things worth your attention

1. **C-17, and the fix I did not apply.** The Go2's Mid-360 sees the robot's own nose —
   134 returns at 0.100–0.152 m in `livox_frame`, which sit at 0.272–0.332 m in `base_link`.
   `pointcloud_to_laserscan` applies `range_min` in the *target* frame, so the ones past
   0.30 survive into `/scan` as 13–14 rays dead ahead. The local costmap therefore carries
   a permanent obstacle 0.30 m in front: goal accepted, 21 recoveries, abort, robot never
   moves. The campaign's listed fallback (hide the geometry from the sensor) would clear
   this in ten minutes and I deliberately did not take it — **it is not established that
   the real robot differs**, and if it does not, hardware `/scan` carries the same cluster
   and Nav2 fails the same way there. That is the finding, not a bug to paper over.
   **OQ-5.1 settles it with one capture: `/scan` and `/unitree/slam_lidar/points` from the
   robot standing still.**

2. **`planner_server` segfaults** (exit −11) when the robot sits on the costmap boundary,
   taking the lifecycle manager's other nodes down, instead of rejecting the goal.
   Ferox-side, reproducible, OQ-5.3.

3. **Two Class-C rates on the Go2 are arithmetic, not tuning.** `/odom` is 100 Hz because
   200/148.7 rounds to a 2-step limiter (C-15); `/utlidar/imu` is 200 Hz because 250 Hz is
   above the physics rate entirely (C-16). Both would need `physics_dt` changed, which is
   what the locomotion policy runs against — a far larger deviation than the one it closes.

### Deviations opened this run

C-13 (passive hand joints held at zero), C-14 (sim hand DOFs interleaved — hand commands
must map by name), C-15, C-16, C-17, and C-8 extended to the Go2's cloud.

### Still open from the campaign

DT2 leftovers (`ferox_vision` against the twin camera, the 4-angle visual pass) and the
Go2's Mid-360 puck and bracket (OQ-5.4) are not done. DT4, DT6, DT7 and DT8 were deferred
by you and were not started.

### Token

The overnight PAT was used from the shell environment only, never written to a file, a
remote URL or argv, and `http.https://github.com/.extraheader` was verified absent from
git config after every push. **It is still live — please revoke it.**


2026-08-18 12:09 UTC | DT3 | hand mount composed + verified | PASS | Hand variant set {None,dex5_1p} on g1.usd. Composed stage: 29 body + 40 hand revolute joints, hand mass 2.003615 kg (exact), 2 flange fixed joints, mount offsets exact to 0.0000 mm. 4 new permanent tests; suite 10/10. Two real bugs caught by composition assertions: reference resolved one dir too shallow (silent empty hand), and open-and-edit authoring appended a stale reference via USD list-op semantics.
2026-08-18 12:33 UTC | DT3 | hands in the PhysX articulation | PASS | USD composition mounts the hands but PhysX never absorbs them (29 DOF, fingers uncommandable) -- three placements tried, all identical. Switched to merging the URDFs and importing once, per NVIDIA's documented method and Unitree's own inspire-hand merge notebook. Result: 69 DOF / 79 bodies, mass 35.004757 kg at 0.00%, one articulation root, first 29 body DOFs bit-identical to the pre-hand order. Found en route: the G1 URDF already defines left/right_hand_palm_joint for its rubber-hand caps, so the merge was creating duplicate joint names and the importer silently rooted the robot at a hand; the caps (0.170 kg each) are now replaced, and the flange origin is read from the G1's own joint rather than assumed from the Dex3 file (it matches, which retires that 'assumed' flag).
2026-08-18 12:38 UTC | DT3 | wire merged asset into the sim | PASS | HAND=none|dex5_1p selects the asset in run.py (none stays the default so a hand problem can be bisected against the bare robot); 01_start_sim.sh passes it through. Sensor layer built into the merged asset from the SAME g1_contract.yaml -- 8/8 frames verified. Removed the dead Hand variant set from g1.usd and the build_hands code behind it: it composed hands PhysX ignores, which is a trap to leave in an asset. Isaac suite 11/11. Two environment gotchas found and guarded: a scratch file named bisect.py in /tmp shadowed the stdlib module for Isaac's own startup and broke every Isaac script in that directory, and a root-owned __pycache__ under Isaac's package tree broke every later run as UID 1234.
2026-08-18 13:21 UTC | DT3 | walk regression + hand poses + RESULTS | PASS | Policy now drives a name-defined 29-joint slice (deploy.yaml asserted 29 == full DOF count, which is 69 with hands -- the robot stood inert until fixed). Walk: same verdicts as DT2, max base-height delta 0.02 m vs the 0.03 m allowance, no gains touched. Four hand PNGs at rest/open/fist/thumb-opposition, all poses reached to <=0.001 rad. Isaac suite 11/11. RESULTS_DT3.md written.
2026-08-18 13:28 UTC | DT5 | Go2 contract verified + twin path generalised | PASS | cmd_vel clamps verified against three sources that agree exactly (driver cmd_vel_to_sport defaults, its clamp call, Ferox go2.yaml) -- provenance configured -> calibrated. Go2 sensor layer built, 5/5 frames verified. _setup_twin_ros is now genuinely contract-driven rather than G1-shaped: camera only if the contract has one, one IMU publisher per Imu topic, odom looked up by message type (G1 /ferox/g1_01/odom vs Go2 root /odom), cloud selected by lidar frame with the accumulator marked produced_by: bridge. Frames resolve by name in the built stage rather than by re-deriving the builder's parent rules -- the G1 mounts livox_frame on torso_link but publishes it under base_link, so any second implementation would disagree. Isaac 11/11, contract 32/32.
2026-08-18 13:41 UTC | DT5 | Go2 twin interface up + audit Class-A clean | PASS | 45 pass, 0 Class-A FAIL. Root /scan and /odom, /unitree/slam_lidar/points, /mid360/points_accum, /utlidar/imu, no camera, no namespaced IMU, 5 static edges all exact to 0.00e+00, LaserScan geometry exact incl. the truncated 3.14159 and 723 rays. Fixed: render_dt 0.02 gave 50/20 = 2.5, a non-integer decimation, so every cloud and scan came out at 25 Hz -- Go2 twin now renders at 40 Hz (0.025 s = 5 physics substeps), step 2, exactly 20 Hz. cloud_accumulator ported verbatim from the driver into ferox_nav_sim (the driver package is not in the nav image). Remaining 3 fails are all declared: /odom 100 Hz (C-15) and /utlidar/imu 200 Hz (C-16) are physics-step quantisation, driver_heartbeat is driver-only.
2026-08-18 13:55 UTC | DT5 | nav goals + geometry + RESULTS | PARTIAL | Interface PASS (Class-A conformant, 45/0). Navigation PARTIAL: control path proven (1.616 m in 6 s on cmd_vel with Nav2 down), but every Nav2 goal aborts. Cause found and measured, not guessed: the Mid-360 returns 134 points off the robot's own nose at 0.100-0.152 m in livox_frame, which land at 0.272-0.332 m in base_link -- and p2l applies range_min in the TARGET frame, so the ones past 0.30 survive into /scan as 13-14 rays dead ahead. Local costmap therefore carries a permanent obstacle 0.30 m in front; goal accepted, 21 recoveries, abort, robot never moves. Recorded C-17. Deliberately NOT fixed by hiding geometry from the sensor: it is not established that hardware differs, and that needs one real capture (OQ-5.1). Also found: planner_server segfaults (exit -11) when the robot sits on the costmap boundary rather than rejecting the goal -- Ferox-side, OQ-5.3.
2026-08-18 14:12 UTC | FASTPATH | housekeeping | PASS | RESULTS_FASTPATH.md (DT2/DT3/DT5 scorecards, C-1..C-17 in one table, E-1/E-2 evidence items, OQ-5.x with commands and expected outputs). Self-hit report added to the audit -- unconditional, in the scan's target frame, with contiguous bearing runs so one object reads as one run; reproduces C-17 as 14 rays 0.300-0.313 m in a single -6.5..-0.1 deg run, ready to sit next to a hardware bag. C-14 promoted to RULE-HAND-NAME in both contracts and a new CLAUDE.md, enforced by an AST test that flags any literal index into the hand DOF block [29,68] (verified against a planted violation, and against a name-based lookup to check it does not false-positive). planner_server segfault: repro script + issue text, no Ferox code touched -- the script ran and correctly reported NOT reproduced, which is the documented intermittency. Contract 35/35, Isaac 11/11, Go2 audit still 0 Class-A FAIL.
2026-08-18 14:36 UTC | GT-G1 | ground truth vs the twin | PASS-with-PING | Bag: 34.571 s, 52210 msgs, 10 topics, G1 standing still, no camera container. /lowstate NOT decoded -- nav image has no unitree_hg -- so waist pitch came from the TF edge. Audit against the bag: 36 pass, 0 Class-B FAIL, 11 Class-A FAIL of which 9 are the absent camera. OQ-2 CLOSED: the assumed cloud layout was exactly right (7 fields, point_step 26, offsets identical). OQ-3 CLOSED: 51.46 Hz. Four provenance upgrades to a new 'captured' tier. Self-hit signature settled: real G1 has 243-299 valid pts at 0.099-0.206 m (r_min 0.0986, matching C-12's 0.0985) but ZERO rays reach /scan (nearest 0.56 m) -- the G1's sensor is 0.4995 m up so range_min removes them, while the Go2's is 0.08 m up and 0.187 m forward so they survive. Same filter, opposite outcome, mount geometry. New: C-18 (52.7% zero padding, ~9.4k valid not ~20k), C-19 (/livox/imu in g not m/s2, |a|=1.006, stamps livox_frame), C-20 (odom z ~0.0068 real vs 0.791 sim -- retires C-1's odom-z premise). PING raised: robot publishes base_link->livox_frame STATIC, contract says dynamic, and the floor fitted through the bag's own TF is tilted 5.87-6.97 deg = the 6.23 deg waist term the edge bakes in, while odom says the body was level. Contract NOT changed on that edge.
2026-08-18 15:07 UTC | GT-G1 | PING resolved, C-18/19/20 dispositioned | PASS | PING closed: contract stays dynamic (Option A stands); the bag shows the robot ran lidar_tf_mode=static, a robot-side item recorded as OQ-6 with the check command and both expected outcomes. C-18 MODELLED not corrected -- no sim points dropped; twin_audit now reports pointcloud/valid_returns for both sides from any source, and the G1 contract carries valid_returns_per_sweep 9443 as a captured check. Verified on both: real 9443 of 19968 (52.7% padded) PASS, sim 4285 of 4285 (0.0%) reported. C-19 log-only, twin keeps SI on both IMUs; the frame_id half stays acted on with mount_frame holding the device in place. C-20 accepted, C-1 annotated to withdraw its odom-z line while the floor measurement stands. Contract 38/38, Isaac 11/11, live Go2 audit still 0 Class-A FAIL.
