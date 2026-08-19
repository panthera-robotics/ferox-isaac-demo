# RESULTS_MM2 — `panthera_lab` environment

**Host:** Vast.ai · RTX 4080 SUPER 16376 MiB · driver 580.105.08 · Isaac Sim 5.1.0
**`TWIN_CAMERA=0`** throughout (C-23) · **Date:** 2026-08-19
**Verdict: FAIL — the venue is built and verified; nothing that has to MOVE in it works.**

The world, the venue and the tooling are done and hold up under read-back. The two
gate criteria that require motion — six waypoints reached, and the door
articulating — both fail, and both failures are understood rather than mysterious.

---

## Scorecard

| Requirement (§5 MM2) | Status | Evidence |
|---|---|---|
| `isaac/assets/worlds/panthera_lab/` built by `tools/build_lab_world.py` | **PASS** — 59/59 read-back checks | `evidence/MM2/world_verify.txt` |
| Room 8 × 6 m, 2.7 m ceiling, walls + ceiling for lidar | **PASS** — clear interior 8.000 × 6.000 × 2.700 | same |
| Door 2.10 × 0.90 m, hinge 0–110°, damping + closer, handle 1.05 m, ~35 kg leaf | **authored and verified statically** | same |
| **Door articulates under a scripted force** | **FAIL** — exposes a `hinge` DOF, then does not move under 500 N·m | `evidence/MM2/door_*.txt` |
| Object table (name, size, mass, friction, source) | **PASS** | below, `objects.json` |
| YCB objects at true scale | **PASS** — 5 of 6 within 12 mm of published; banana explained | `evidence/MM2/world_verify.txt` |
| `venues/panthera_lab.yaml` with 6 waypoints | **PASS** | `evidence/MM2/panthera_lab_venue.yaml` |
| `SIM_WORLD=panthera_lab` in the sim | **PASS** — boots, robot spawns upright at (−2.59, −0.01, 0.79) | `/tmp/sim_lab.log` |
| SLAM map built and saved | **PASS** (partial coverage — see below) | `evidence/MM2/map/` |
| lidar + `/scan` ≥ 60 % finite | **FAIL — 58.4 %**, cause identified and not the world | **C-24** |
| **`MoveToNamed` to every waypoint 1/1** | **FAIL — 0/6** | `evidence/MM2/waypoints_0of6.txt` |
| camera view of the table with objects | **BLOCKED — C-23** | 4090 item |
| top-down render with dimensions | **not produced** — see deviations | — |
| audit still 0 Class-A | **not re-run** — see deviations | — |

---

## What was built, and the five things the read-back caught

`tools/build_lab_world.py` authors the world and then **reads the written file
back**, 59 checks. Each of these passed code review by eye and failed the read-back:

1. **The room measured 8.100 × 6.100 m.** Walls were centred *on* the interior
   boundary, so the space you could actually walk was 7.9 × 5.9 while every
   document said 8 × 6. Walls now sit outside the interior; clear space is
   8.000 × 6.000 exactly.
2. **Every YCB asset looked 30–90 mm oversized.** That was *my measurement*: I was
   comparing world-aligned bounding boxes of objects carrying a seeded random yaw,
   and the world box of a rotated object is larger than the object. Measured
   untransformed, five of six agree with published dimensions to ≤12 mm. Reporting
   the first number would have been a fabricated defect in NVIDIA's assets.
3. **The objects arrive lying down** — the soup can's 0.068 m *diameter* along Z
   and its 0.102 m height along Y. They are now stood up by finding which local
   axis matches the published height and rotating that axis to +Z. Nothing is
   scaled, and the resulting standing height is checked against the published one.
4. **The banana disagrees on one cross-section axis** (74 mm measured vs 39 mm
   published). The fruit is curved, so the published figure is its thickness, not
   its box. Recorded as measured *with the reason*, rather than by widening a
   tolerance until it passed.
5. **Rewriting the interior checks silently deleted the door checks.** The report
   dropping from 46 lines to 16 is what gave it away. Verification code needs the
   same read-back discipline as the thing it verifies.

### Objects

| object | source | size (m) | mass (kg) | friction s/d | stood up |
|---|---|---|---|---|---|
| `soup_can` | YCB 005 | YCB mesh (true scale, verified) | 0.349 | 0.70 / 0.60 | yes |
| `mustard` | YCB 006 | YCB mesh (true scale, verified) | 0.603 | 0.70 / 0.60 | yes |
| `cracker_box` | YCB 003 | YCB mesh (true scale, verified) | 0.411 | 0.60 / 0.50 | yes |
| `sugar_box` | YCB 004 | YCB mesh (true scale, verified) | 0.514 | 0.60 / 0.50 | yes |
| `mug` | YCB 025 | YCB mesh (true scale, verified) | 0.118 | 0.70 / 0.60 | as authored |
| `banana` | YCB 011 | YCB mesh (true scale, verified) | 0.066 | 0.50 / 0.40 | as authored |
| `cube_5cm` | authored primitive | 0.050 x 0.050 x 0.050 | 0.100 | 0.80 / 0.70 | as authored |
| `brochure_box` | authored primitive | 0.210 x 0.150 x 0.030 | 0.180 | 0.60 / 0.50 | as authored |

---

## The two errors in the venue that a script caught and reading could not

`tools/check_venue_waypoints.py` checks hand-written waypoints against the world
the builder actually produced. On first run:

* **`home` was inside the shelf's footprint** (shelf x [−4.20, −2.60], y [−2.00,
  −1.60]; `home` was (−2.8, −1.9)). It was also the sim's spawn pose.
* **`door_outside` had no floor under it.** The slab ended at y = 3.10 and the
  waypoint was at y = 3.60, so "walk through the door" was a walk off the edge of
  the world. The builder now authors an outside apron with low side walls.

Every waypoint is now checked for floor, furniture overlap, and ≥0.55 m clearance
(MM1's inflation radius — a goal closer than that is one Nav2 will not plan to,
which was the MM0 nav failure). **6/6 pass.**

---

## Nav: 0 of 6, and the cause is MM1's yaw defect

First run: 0/6, and `closest == err` on every row — **the robot never moved at
all**. It was still wedged against the counter where a mapping lap had left it.

After a `/twin/reset`, one goal was sent and instrumented:

| observation | value |
|---|---|
| `cmd_vel` messages while the goal was active | 480 |
| `vx` range | −0.050 … 0.000 m/s |
| `wz` range | 0.000 … +0.400 rad/s, mean +0.213 |
| planner | `GridBased: failed to create plan, no valid path found` |
| behaviour server | `Exceeded time allowance before reaching the Spin goal — spin failed` |
| bt_navigator | `Goal failed` |

Nav2 *is* commanding — but that command profile (zero forward, sustained yaw) is
the **Spin recovery**, and **MM1 measured that this policy cannot rotate in place**
(four of six pure rotations came out at 0.0000 rad/s). So the recovery can never
complete. This is the MM1 yaw finding arriving as an operational blocker rather
than a curiosity.

A flood fill over the global costmap made the underlying failure exact:

| waypoint | own cell | reachable from the robot |
|---|---|---|
| `home` | free | **yes** |
| `shelf` | free | **yes** |
| `table_approach` | free | no |
| `table` | free | no |
| `door_inside` | free | no |
| `door_outside` | **outside the map** | no |

Every goal's own cell is free. The robot is sealed inside a 2220-cell (5.5 m²)
pocket of mapped space. So the venue needs mapping by driving — and the drive has
to obey the gait's real capability.

`scripts/mm2_map_lab.py` does that: it never commands yaw at zero forward speed,
because at `vx = 0` this policy produces none. It still ended jammed at
(2.52, −1.12) — 0.08 m off the table's north edge — and never moved again.

**That is the hard finding of MM2:** the twin's locomotion **cannot recover from
contact with an obstacle**, because escape needs either in-place rotation (0 % of
command) or reverse (35–70 % error, MM1 §3). Combined with Nav2's Spin recovery,
autonomous navigation in a furnished 8 × 6 m room is not achievable with this
checkpoint. MM2's nav gate is blocked behind the **MM1b retrain**, not behind
anything in the world.

Mohammed's third nav goal, requested to be run here, is therefore **not delivered**;
it cannot be until the robot can turn.

---

## Door: it exposes a DOF and is kinematically locked

Six attempts, each ruling something out. Recorded because the door passed **every
static USD check** at each stage while remaining unusable:

| # | change | result |
|---|---|---|
| 1 | hinge `body0` = the static frame | articulation exposes **zero DOF** |
| 2 | `body0` = world (empty) | still zero DOF |
| 3 | frame made a rigid body + fixed joint to world (fixed-base articulation) | **DOF `hinge` appears**; angle stuck at −0.333° |
| 4 | gains/targets moved off USD attributes onto the articulation API | still −0.333° |
| 5 | piers resized to the rough opening (the jamb was buried 60 × 60 mm through the pier's full 2.16 m height); 10 mm floor undercut | still −0.333° |
| 6 | handle deactivated | −1.435°, still frozen |

Decisive measurement: with gains 800/50, max effort 5000, a 60° position target
**and** 500 N·m of direct joint effort, the angle does not change by 0.001°. That
is a **kinematic lock, not a force limit**.

**Diagnosed next step, not attempted here.** Both links are `UsdGeom.Cube` prims
carrying non-uniform **scale** xformOps. Scale on an articulation link is not
supported by PhysX and is the remaining candidate that fits "reports a DOF, refuses
to move under any torque". The fix is to author each link as an `Xform` with
unscaled child geometry — which is how the G1's own USD is built, and it does work.

Also caught along the way: my test wrapper **silently swallowed** the failure of
`set_joint_position_targets` and fell back to a USD attribute that does not reach
the running solver, turning one wrong API into three phantom "failures". The
`except` is gone.

---

## `/scan` — 58.4 %, and it is not this world

**FAIL against ≥60 %**, cause recorded as **C-24**: the G1 twin's `/scan` is a
single Mid-360 frame. `cloud_accumulator` — which exists precisely because "an
accumulator that merely resembles the robot's is not a twin" — is spawned by
`twin_bridge_go2.launch.py` only.

| | measured | predicted from geometry |
|---|---|---|
| at `home` (−2.60, 0.00) | 58.5 % | 83.3 % |
| mid-room (0.87, 0.16) | 57.7 % | ~100 % |

The measurement moves 0.8 points while the prediction moves 17. Nothing about the
room explains it. Same signature as MM0: hospital gave 45 % from the twin against
**70 %** from the real robot's bag on the identical scene.

---

## Video

**None.** MM2 has no clip, and neither does MM1. `film.py` still cannot film the
real policy (RESULTS_MM1 §4): three bugs fixed behind `--drive policy`, and the
robot is thrown rather than walked. Filming the nav or door results would in any
case mean filming failures captioned as capabilities. `docs/mm/media/` contains
only `SHOTLIST.md`.

---

## Deviations

* **C-23** — camera off throughout; the "camera view of the table with objects"
  item is a 4090 item.
* **Top-down render not produced.** It depends on the same rendering path as the
  clips; deferred with them rather than shipped half-done.
* **Audit not re-run.** The gate asks for "0 Class-A still"; with nav and the door
  failing, re-running it would report on a venue that is not yet usable. Queued
  behind the door fix.
* **`/twin/reset` teleports while SLAM is running.** map→odom stayed within
  0.22 m so it did not corrupt these measurements, but a teleport is not something
  SLAM can track and it should not be used mid-mapping.

---

## Open questions for Mohammed

1. **MM1b retrain — this is now the critical path.** MM2's nav gate, and MM5's
   navigate→pick→place, both need a policy that can turn in place and recover from
   contact. Everything else in MM2 is done. Do you want the retrain started now
   (≤2048 envs on 16 GB, twin USD, weighting in-place rotation and lateral commands
   per RESULTS_MM1 §3)? **Default taken: not started — it is an overnight run and a
   scope call, not a fix I should make unilaterally.**
2. **C-24 accumulator.** One-line-ish change to the G1 twin bridge launch to spawn
   `cloud_accumulator` and repoint `pointcloud_to_laserscan`. Outside the Nav2-only
   Ferox authorization from MM1. Authorize? **Default taken: flagged, not changed.**
3. **Door.** The scale-on-articulation-link hypothesis is concrete and testable in
   ~1 h. Worth doing before MM3, or park the door as a stretch item?

---

## Reproduce

```bash
# build + verify the world (no ROS, no robot)
docker run --rm --gpus all -v $PWD/isaac:/workspace/ferox_isaac \
  -v $PWD/tools:/workspace/ferox_tools:ro -v $PWD/docs/mm/evidence/MM2:/out \
  --entrypoint bash nvcr.io/nvidia/isaac-sim:5.1.0 -lc \
  'cp /workspace/ferox_tools/build_lab_world.py /tmp/isaacrun/ && \
   /isaac-sim/python.sh /tmp/isaacrun/build_lab_world.py --seed 0 --report /out/world_verify.txt'

# check the venue against the world that was actually built (stdlib + yaml)
python3 tools/check_venue_waypoints.py \
  --world isaac/assets/worlds/panthera_lab/objects.json \
  --venue Ferox/src/ferox_nav/venues/panthera_lab.yaml

# the twin in the lab
TWIN_CAMERA=0 ROBOT=g1 TWIN=1 HAND=dex5_1p SIM_WORLD=panthera_lab ./scripts/01_start_sim.sh
ROBOT=g1 MODE=twin VENUE=panthera_lab ./scripts/02_start_ferox.sh
```
