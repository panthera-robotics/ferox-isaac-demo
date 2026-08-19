# RESULTS_MM2 — `panthera_lab` environment

**Host:** Vast.ai · RTX 4080 SUPER 16376 MiB · driver 580.105.08 · Isaac Sim 5.1.0
**`TWIN_CAMERA=0`** throughout (C-23 — this box cannot run the camera)
**Date:** 2026-08-19 · **Verdict: PARTIAL — the venue is built and verified; the
robot cannot yet navigate it, and the door does not open**

Two of the gate's PASS criteria are blocked, both by defects with fully
characterised causes rather than by unfinished work. They are stated plainly below
and neither is worked around.

---

## Scorecard

| Requirement | Status | Evidence |
|---|---|---|
| `panthera_lab` world built by `tools/build_lab_world.py` | **PASS** — 59/59 read-back checks | `evidence/MM2/world_verify.txt` |
| Room 8 × 6 m, 2.7 m ceiling, walls + ceiling for lidar | **PASS** — clear interior 8.000 × 6.000 × 2.700 m | same |
| Table / counter / shelf at spec dimensions | **PASS** | `evidence/MM2/object_table.md` |
| YCB objects at true scale, masses + friction explicit | **PASS** — 6 objects, ≤12 mm from published dims | same |
| Object placement randomised by `--seed` | **PASS** — seed recorded in the layer's `customLayerData` | `panthera_lab.usd` |
| Door is a PhysX articulation | **PASS** — exposes DOF `hinge`, 35 kg leaf, limits 0–110°, closer spring | `evidence/MM2/world_verify.txt` |
| **Door articulates under a scripted force + angle plot** | **FAIL — C-25** | `evidence/MM2/door_report.txt` |
| Ferox `venues/panthera_lab.yaml`, 6 waypoints | **PASS** — and machine-checked against the built world | `evidence/MM2/waypoint_check.txt` |
| `SIM_WORLD=panthera_lab` in the sim | **PASS** — boots, robot stands at spawn | `evidence/MM2/` |
| SLAM map built and saved | **PASS** — 16 887 free cells | `evidence/MM2/map/panthera_lab.{pgm,yaml}` |
| lidar cloud + `/scan` ≥ 60 % finite | **FAIL — 58.4 %, cause C-24** | `TWIN_DEVIATIONS.md` C-24 |
| Camera view of the table with objects | **BLOCKED — C-23** | queued for a 4090 day |
| **`MoveToNamed` to all six waypoints 1/1** | **FAIL — 0/6, cause C-26** | `evidence/MM2/` |
| audit still 0 Class-A | **PASS, modulo C-23** — 6 Class-A, all six are the camera topics | `evidence/MM2/audit_classA.txt` |

Audit summary in this world: **64 pass, 6 Class-A fail, 7 Class-B fail, 10 skip, 87
checks.** Every Class-A failure is `camera/*` with `TWIN_CAMERA=0`; there are no
non-camera Class-A regressions from the new world.

---

## What the world is

`tools/build_lab_world.py` authors `isaac/assets/worlds/panthera_lab/` with no
hand-editing, and a separate pass reads the written file back. The full object
table, including every mass, friction pair and placement, is in
`evidence/MM2/object_table.md`.

The room has a **ceiling as well as walls**, because the Mid-360 sweeps a sphere and
an open-topped box returns nothing above the horizon — that would have failed the
scan gate for a reason that has nothing to do with the sensor.

`run.py` gains `SIM_WORLD=panthera_lab`. Worlds may now live in this repo rather
than on the Nucleus asset server, via an `@LOCAL@` prefix resolved against the
`isaac/` tree; the existing `omni.client.stat` check is unchanged, so a missing
local world fails exactly as loudly as a missing remote one.

---

## Six things the read-back caught that reading the code would not

Each produced an artefact that looked correct.

1. **The room measured 8.100 × 6.100 m.** Walls were centred *on* the interior
   boundary, so the space you can walk was 7.9 × 5.9 while every document said
   8 × 6. Walls now sit outside the interior; the check measures the clear span
   between wall faces.
2. **Every YCB asset looked 30–90 mm oversized.** That was the *measurement*: I was
   comparing world-aligned bounding boxes of objects carrying a seeded random yaw,
   and the world box of a rotated object is larger than the object. Measured
   untransformed, five of six agree with published dimensions to ≤12 mm. Reporting
   the first version would have been a fabricated defect in NVIDIA's assets.
3. **The objects arrive lying down** — the soup can's 0.068 m *diameter* along Z and
   its 0.102 m height along Y. They are now stood up by finding which local axis
   matches the published height and rotating that axis to +Z. Nothing is scaled, and
   the resulting standing height is checked against the published figure.
4. **The banana disagrees on one cross-section axis** (74 mm measured against 39 mm
   published). It is curved, so the published figure is its thickness and not its
   bounding box. Recorded as measured *with the reason*, rather than by quietly
   widening a tolerance.
5. **Restoring the interior checks silently deleted the door checks.** The report
   dropping from 46 lines to 16 is what gave it away. Verification code needs the
   same read-back discipline as the thing it verifies.
6. **`home` was inside the shelf and `door_outside` had no floor under it.** Neither
   is visible in the YAML. `tools/check_venue_waypoints.py` now checks every
   waypoint against the world the builder actually produced — over floor, clear of
   furniture, and ≥0.55 m from any obstacle (MM1's inflation radius; a goal closer
   than that is one Nav2 will not plan to). The builder gained an outside apron so
   "walk through the door" is no longer a walk off the edge of the world.

---

## C-25 — the door has a DOF and cannot be moved

**Class B, new, open.** `tools/test_door_articulation.py` drives the hinge and
records the angle. The door **exposes DOF `hinge`** and reports sane gains, limits
and drive type. It then does not move — at all — under any command:

| attempt | what changed | result |
|---|---|---|
| 1 | hinge `body0` = the static frame | articulation with **zero DOF** — every static USD check passed |
| 2 | `body0` = world | still zero DOF (a world-anchored joint is maximal-coordinate, not an articulation) |
| 3 | frame made a rigid body + fixed joint to world (fixed-base articulation) | **DOF appears**; angle stays −0.333° |
| 4 | gains/targets moved from USD attributes to the articulation API | −0.333° |
| 5 | rough opening widened to leaf + 2 jambs (the frame was buried 60 × 60 mm through the pier's full 2.16 m height), 10 mm undercut added | −0.333° |
| 6 | 800 stiffness, 5000 N·m max effort, **500 N·m applied directly** via `set_joint_efforts` | −0.333°, unchanged to three decimals |

500 N·m moving a 35 kg leaf by 0.000° is a **kinematic lock, not a force limit**.
Deactivating the lever handle changes the resting angle (−0.333° → −1.435°), so the
handle — a static collider parented under the articulation root, coincident with the
leaf — is *one* constraint, but removing it does not free the door.

**Stopped at six attempts per §5.** The next thing to try is restructuring the leaf
as an Xform link containing the slab and handle as child geometry, so the handle
belongs to the moving link instead of being adopted by the fixed base. Not attempted
here.

Two findings from this are worth keeping regardless of the door:
* **A USD articulation can pass every static check and have zero DOF.** Joint
  present, axis right, limits right, drive right, `ArticulationRootAPI` applied — and
  no articulation, because a static collider cannot be a link. Only a
  physics-stepping test finds it.
* **Writing `DriveAPI` attributes after `world.reset()` does nothing.** PhysX builds
  the articulation at reset; the USD change is real and the solver never sees it.
  Three of the six "failures" above were that one wrong API, and a silent
  `except: pass` fallback in my own test hid it for two attempts.

---

## C-26 — the twin cannot navigate a furnished room, and the cause is MM1's yaw defect

**Class A for the campaign, open.** `MoveToNamed` reached **0 of 6** waypoints.
This is not a Nav2 tuning problem and not a venue problem.

**First run: the robot never moved at all.** `closest == err` on all six rows — it
was still wedged against the counter where a mapping lap had left it.

**After a clean `/twin/reset`, one goal, instrumented:**

| observation | value |
|---|---|
| cmd_vel messages while the goal was active | 480 |
| commanded `vx` | −0.05 … 0.00 m/s |
| commanded `wz` | 0.00 … +0.40, mean +0.213 |
| `planner_server` | `GridBased: failed to create plan, no valid path found` |
| `behavior_server` | `Exceeded time allowance before reaching the Spin goal — spin failed` |
| distance moved | 0.00 m |

Nav2 *was* commanding — it was running its **Spin recovery**, and MM1 measured that
this policy's in-place yaw is **0.0000 rad/s**. The recovery Nav2 falls back to is
the one manoeuvre the robot cannot perform, so it can never recover.

**Why there was no valid path.** A flood fill over the global costmap from the
robot's own cell:

| waypoint | its own cell | reachable from the robot |
|---|---|---|
| `home` | free (cost 0) | yes |
| `shelf` | free (cost 28) | yes |
| `table_approach` | free (cost 0) | **no** |
| `table` | free (cost 0) | **no** |
| `door_inside` | free (cost 0) | **no** |
| `door_outside` | **outside the map** | **no** |

Every goal's own cell is free. The robot is simply sealed in a 2 220-cell (5.5 m²)
pocket of *mapped* space, because unmapped cells are untraversable and the map is
partial. `map → odom` was only (0.216, 0.065, 0.038 rad), so this is not a
localisation artefact.

**And the map stays partial**, because mapping requires driving the room, and:

* `scripts/mm2_map_lab.py` drives with `vx > 0` whenever it wants yaw — the only way
  this gait turns. It still ended jammed against the table at (2.52, −1.12), 0.08 m
  from the table's north edge, and **never moved again for the remaining 12
  waypoints of the tour**.
* Escaping a contact needs either in-place rotation (measured 0 %) or reverse
  (measured 35–70 % error at MM1). The twin has neither.

**So MM2's navigation gate is blocked on MM1b, the retrain.** The MM1 yaw finding is
not cosmetic and not deferrable past this point: a robot that cannot turn in place
cannot map a furnished room, cannot recover from touching furniture, and cannot use
Nav2's recovery behaviours. MM3–MM5 all sit downstream of moving around this room.

---

## Deviations

* **C-23** — camera off; all 6 Class-A audit failures are the camera topics.
* **C-24** — `/scan` is a single Mid-360 frame, so 58.4 % finite against a ≥60 %
  gate. Proven pose-independent (58.5 % at `home`, 57.7 % mid-room) against a
  geometric prediction that moves 17 points between those poses.
* **C-25** — the door, above.
* **C-26** — navigation, above.
* No MM2 clip. `film.py --drive policy` is still incomplete (RESULTS_MM1 §4), and the
  live-sim route needs the robot to be able to drive the room, which is C-26.

---

## Open questions for Mohammed

1. **MM1b retrain — this is now blocking, not optional.** C-26 shows the yaw defect
   stops the campaign at MM2 and everything after it. The recipe is recorded in
   RESULTS_MM1 §2.12 and sharpened by §3: weight in-place rotation and lateral
   velocity commands explicitly, ≤2048 envs on 16 GB, twin USD. Do you want it run
   now on this box? **Default taken: not started — it is an overnight run and a
   scope call.**
2. **C-24's fix is a Ferox launch-graph change** (spawn `cloud_accumulator` in the
   G1 twin bridge as the Go2 bridge already does, and point
   `pointcloud_to_laserscan` at `/mid360/points_accum`). The only Ferox change
   authorised so far was MM1's Nav2 inflation item. Authorise this one?
   **Default taken: not changed.**
3. **C-25** — worth one more attempt (restructure the leaf as an Xform link), or
   leave the door static and drop MM5's door stretch goal? **Default taken: stopped
   at six attempts, documented.**

---

## Reproduce

```bash
# build + verify the world (no GPU work, but pxr needs kit)
docker run --rm --gpus all -v $PWD/isaac:/workspace/ferox_isaac \
  -v $PWD/tools:/workspace/ferox_tools:ro -v $PWD/docs/mm/evidence/MM2:/out \
  --entrypoint bash nvcr.io/nvidia/isaac-sim:5.1.0 -lc \
  'cp /workspace/ferox_tools/build_lab_world.py /tmp/isaacrun/ && \
   /isaac-sim/python.sh /tmp/isaacrun/build_lab_world.py --seed 0 --report /out/world_verify.txt'

# check the venue against the world that was built (stdlib + yaml only)
python3 tools/check_venue_waypoints.py \
  --world isaac/assets/worlds/panthera_lab/objects.json \
  --venue Ferox/src/ferox_nav/venues/panthera_lab.yaml

# run it
TWIN_CAMERA=0 ROBOT=g1 TWIN=1 HAND=dex5_1p SIM_WORLD=panthera_lab ./scripts/01_start_sim.sh
ROBOT=g1 MODE=twin VENUE=panthera_lab ./scripts/02_start_ferox.sh
docker exec probe python3 /hostscripts/mm2_map_lab.py       # map it
docker exec probe python3 /hostscripts/mm2_waypoints.py --venue /tmp/panthera_lab.yaml
```
