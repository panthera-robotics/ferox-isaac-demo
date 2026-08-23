# Manipulation envelope verdict — what this hand can and cannot grasp

**Status: manipulation PARKED, kinematically characterized.**
Not mistuned. More tuning will not change these numbers.

Date 2026-08-23. Ferox G1 29-DoF + Dex5-1P, Isaac Sim 5.1.0, PhysX articulation,
omni locomotion policy as balancer (SONIC parked at C-39). Base rig-held
(`MM5_FIX_BASE=1`), `pelvis_z` 0.80 m, `target_r` 0.315 m from the right shoulder,
declared friction `MM5_GRASP_MU=1.2` (CAMPAIGN 4.4).

Every number below was taken with the 4-gauge preflight GREEN on the same day
(`docs/mm/evidence/MM5/preflight_latest.json`): tilt-at-rest 1.11 deg, a scripted
30 deg tip reads 29.86 deg, a scripted +80 mm rise reads +78.7 mm, sustained contact
distinguishes a steady 20 N x 0.6 s from a single 91 N spike.

## 1. The two hard numbers

| quantity | measured | how |
|---|---|---|
| **Dex5 closed fingertip spread** | **21.5 mm** | direct hand measurement, `MM5_MEASURE_HAND=1` |
| **Palm vertical ceiling** | **0.946 m max, 0.935 m median** | 10 trials, converged REACH equilibria, `pelvis_z` 0.80, fixed base |

The palm ceiling is new this session and it retro-explains the whole campaign: the
66 mm soup can centres at 0.95 m -- *at* the ceiling -- and it is the only object
that has ever reached closure.

## 2. The envelope map (object size x surface height)

| object | size | surface | object centre z | REACHABLE? | CAGES? | LIFTS? | evidence |
|---|---|---|---|---|---|---|---|
| soup can | 66 mm dia | counter 0.90 | 0.950 | **yes** | **no -- pinch** | no | 6 finger links, 43.83 N, but **0 links within 45 mm of the object axis, 0 below its centre** |
| cube | 50 mm | counter 0.90 | 0.950 | marginal | no closure | no | 0/8, no closure reached |
| block | 30 mm | counter 0.90 | 0.915 | **no** | untested | no | 0/10, converged residual 67-173 mm, elbow 2.059/2.044, wrist 1.565/1.564 |
| block | 30 mm | counter **1.02** | 1.037 | **no** | untested | no | 0/10, converged FLAT 95-118 mm -- object is ~100 mm **above the 0.946 m palm ceiling** |
| block | 30 mm | counter **0.933** | 0.952 | **yes (5/10 in <1.3 s)** | **no -- topples it** | no | 1 TOPPLED at 36.9 deg during closure; 5 DESCEND stalls 47-152 mm |
| block | 30 mm | riser pad 1.02 | 1.037 | **VOID** | -- | -- | apparatus defect, see 4 |

## 3. The verdict, in one paragraph

**The reachable band and the graspable band do not overlap.**
A 30 mm object -- the only size that could fit inside a 21.5 mm fingertip spread
without being met tip-first -- is reachable only in a narrow height window around
0.95 m, and when the hand does reach it, closure **topples it (36.9 deg) rather than
caging it**. Larger objects (50-66 mm) are reachable at that same height but exceed
the tip spread outright and can only be pinched: at 43.83 N of contact across 6
finger links, **zero links sat within 45 mm of the object's axis and zero below its
centre**. There is no object size in this scene that this hand both reaches and cages.

**"Dex5 power-grasp envelope exceeds available objects in this scene"**, with the
refinement this session bought: it is not size alone. Size and surface height are
**coupled** -- a smaller object sits lower, and lower is further from a shoulder that
is already at its limit, so shrinking the object to fit the hand moves it out of reach.

## 4. What was VOID and why (recorded, not hidden)

Three apparatus defects were caught during these runs. All three produced numbers
that looked like robot limits and were mine:

1. **`descend_timeout_s` 45 s.** Three trials stalled at `SERVO_SLOW pinned_joints=0`,
   residual 47-60 mm -- still converging when my timer fired. Raised to 120 s. The
   residuals then *grew* to 67-173 mm, proving the arm reaches a pinned equilibrium
   and sits there; the conclusion survived, but it had been resting on my timeout.
2. **The riser pad (VOID row).** A 0.12 m pad on the counter put a vertical face at
   hand height in the approach corridor. The DLS servo carries no obstacle term, so
   it drove the hand into that face: palm stalled at y=2.02, **45-54 mm short of the
   pad's front face at y=2.07, inside its 0.90-1.02 z band, on 9 of 10 trials.**
   Replaced by raising the counter itself -- same height change, no new obstacle.
3. **`MM5_COUNTER_H` never crossed the container boundary.** `01_start_sim.sh` passes
   an explicit `-e` allowlist; the new variable was not on it, so the runner kept
   `counter_h=0.90` and staged the block at 0.917 m *inside* a 1.02 m slab. PhysX
   ejected it and all ten trials chased a block on the floor at z=0.015, **568 mm away
   -- the identical value on every row**, which is what gave it away. A constant that
   precise is a rig fault, not ten independent reach failures. Variable added to the
   allowlist with a 0.90 default.

## 5. What would actually change this (none of it is tuning)

* **A graspable object.** Something with a graspable feature under ~21.5 mm at its
  contact band -- a handle, a stem, a thin neck -- staged with its centre near 0.95 m.
* **A pinch-grasp primitive.** The hand demonstrably delivers 43.83 N across 6 links
  tip-first. That is a pinch, and a pinch controller for small/flat objects is a
  different primitive from the power-grasp close this pipeline runs -- not a gain.
* **A gripper.** A parallel-jaw end effector sidesteps the tip-spread limit entirely.
* **Obstacle-aware IK.** Defect 2 above is a real planner gap: the DLS servo has no
  obstacle term and will drive the hand through furniture. Needed before any cluttered
  scene, independent of grasping.

Raising `kp`, changing `grasp_clearance`, re-tuning the null space, or adding friction
will not move any number in section 2. That is what "kinematically characterized" means.

## 6. Instruments kept (all verified, all retained)

* `tools/mm5_preflight.py` + `scripts/mm5_preflight.sh` -- 4 gauges, mandatory before
  any grasp number is quoted. Non-zero exit means DO NOT TRUST GRASP NUMBERS.
* The **enclosure logger** -- links within N mm of the object axis, and links below its
  centre. This is the instrument that turned "43.83 N of contact" into "pinch, not cage".
* The **IK classifier** -- `IK_INFEASIBLE` (names the pinned joints and their limits)
  vs `SERVO_SLOW` (converging, no joint at a stop). This is what separates a kinematic
  limit from a timeout, and it is why defect 1 was caught.
