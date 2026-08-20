# RESULTS_MM5 — scripted navigate → pick → place

Host: RTX 4080 SUPER 16 GB / driver 580.105.08 / Isaac Sim 5.1.0 · `TWIN_CAMERA=0` (C-23)
Date: 2026-08-20
Verdict: **FAIL. Mobile 0/20 (single cause, C-39). Fixed-base 0/20 (two causes, both
manipulation).** Both numbers are reported; the fixed-base variant is the substantive
one, because it is the first time this program has exercised the grasp stack at all —
9 of 20 trials reached the pre-grasp and closed the hand on the can.

**Balancer: the omni locomotion policy.** SONIC is parked at C-39 and `CAMPAIGN §4.4`
permits "or omni standing if MM4 slips — say which". This says which. The seam SONIC
slots into is already built and is not in this pipeline: it is `G1_CONTROL=handoff` in
`run.py`, which hands the whole articulation over once and latches. When C-39 closes,
MM5 runs behind that switch unchanged.

---

## Scorecard

| requirement | status | evidence |
|---|---|---|
| pipeline runs end to end without cheat-attach | **PASS** — 0 trials used it | `evidence/MM5/mm5_results.json` (`cheat_attach: false` on every row) |
| N=20 trials, object pose randomised by seed | **PASS** — seeds 20260821–20260840 | same |
| success rate reported | **PASS** — **0/20 = 0.0 %** | same |
| each failure classified | **PASS** — one bucket, `ROBOT_FELL` ×20 | same |
| per-stage timing | **PASS** | same |
| IK solver choice documented | **PASS** — damped least squares, rationale in `mm5/ik.py` | — |
| Dex5 closes on real contact | **NOT REACHED** — no trial survived to GRASP | — |
| `MoveToNamed(table_approach)` first | **NOT ATTEMPTED — C-26** | see below |
| clip per stage | **DEFERRED (C-23)** | no media on this box |

---

## The number, and the one cause behind it

```
20 trials, soup_can, seeds 20260821..20260840
success 0/20 = 0.0%
taxonomy:  ROBOT_FELL  20
stage timing: APPROACH n=20 mean 4.32 s   REACH n=12 mean 2.93 s (max 11.25)
```

Twelve trials got as far as REACH and toppled a mean 2.93 s into it. The other eight
fell during APPROACH — the reset put the robot back upright but it went over again
before the settle window elapsed, which is the same failure arriving earlier.

**The twin cannot hold its balance while the arm reaches.** The omni policy commands all
29 body joints and was trained with a `joint_deviation_arms` term, so an arm driven away
from its nominal pose is outside the distribution it learned; and each Dex5 hand is
**1.0 kg at the longest lever on the robot**. Extending the arm 0.6 m toward the table
moves the CoM further than the policy can answer.

This is the same finding as **C-39**, arrived at from the other direction. MM4 found that
SONIC crouches and falls when handed a robot in the wrong stance; MM5 finds that the omni
policy falls when the arm leaves its stance. Both say the twin's balance is fragile to
exactly the thing manipulation requires. **Whichever controller closes first unblocks
both gates.**

### It is not the IK, and that took three bugs to establish

The reach converges once the servo is correct. All three were silent:

1. **`target = measured + dq` cannot outrun its own tracking error.** The arm lags its
   target by a few tenths of a radian against kp 40 with a kilo on the wrist, so a
   target pinned one small step ahead of a lagging arm stalls the instant `kp·error`
   balances gravity. Measured: the palm crept **0.11 m of a 0.70 m reach in 25 s** and
   stopped. Integrating from the previous *target* fixed it and the arm extends.
2. **`ArticulationController` keeps one pending action per step.** The policy's action
   covers all 29 body joints, arms included, so every `set_joint_position_targets` write
   was silently replaced — the palm sat at z = 0.70 for 25 s while the IK dutifully
   computed targets nothing applied. Arm and fingers now go out as one `apply_action`
   *after* the policy. This is the MM3 lesson repeating verbatim.
3. **Reach geometry.** The `table` waypoint is 1.10 m from the objects and the G1's arm
   reaches ~0.65 m, so "navigate to the waypoint, then reach" is geometrically
   impossible. MM5 stands the robot at a reach pose and stages the can on the table's
   near edge. Stated as a scenario choice, not hidden — the pick is still a real pick.

## C-41 — the lab's objects had no colliders

`tools/build_lab_world.py` applies `RigidBodyAPI` and `MassAPI` to every YCB object and
never `CollisionAPI`. Each object was therefore a body gravity acts on and **nothing can
touch**: measured in free fall from t=0, reaching **−43 m after 3 s**, which is exactly
½gt². Any grasp attempt would have closed on an object already through the floor.

MM2 verified 59 properties of this world and passed, because MM2 never tried to touch an
object. That is the honest shape of the miss: not a wrong check, an absent one.

Fixed at load in `mm5/runner.py::ensure_colliders` — 6 objects, 6 meshes, `convexHull`.
Not in the builder: the objects arrive as *references* to the Isaac asset server and
their meshes are not composed into the stage at build time, so the builder has nothing to
attach a collider to. A builder-side fix was written, produced no colliders for exactly
that reason, and was reverted rather than shipped unverified. **The builder remains the
better home if those references are ever payload-loaded at build time.**

`convexHull` and not the triangle mesh, because PhysX will not simulate a non-convex
triangle mesh as a *dynamic* collider. The mug's handle hole is filled in as a result —
declared, not hidden.

## The navigation stage was not attempted, and why

`MoveToNamed` is **0/6 in this venue (C-26)**, and the cause is not Nav2 tuning: the omni
policy's in-place yaw is **0.0000 rad/s**, so the Spin recovery Nav2 falls back to is the
one manoeuvre this robot cannot perform, and the map stays partial because mapping needs
driving the room. Running it again to watch it fail a seventh time would not have
produced information. MM5 places the robot at the reach pose and says so.

SONIC *can* turn — that is why it is the turning controller of record — so the navigation
stage and the balance stage are blocked on the same thing.

## What changed

| file | one line |
|---|---|
| `isaac/twin/mm5/ik.py` | damped-least-squares differential IK; solver choice argued in the module docstring |
| `isaac/twin/mm5/pipeline.py` | per-trial stage machine, taxonomy, actuation |
| `isaac/twin/mm5/runner.py` | N trials, seeded object pose, collider fix, JSON + markdown |
| `isaac/run.py` | `MM5=1` hook; MM5 steps before the policy and re-asserts after it |
| `isaac/twin/mm5_probe.py` | one-shot feasibility probe (object pose, Jacobian, EE link) |

## Deviations

| id | one line |
|---|---|
| C-41 | lab YCB objects shipped with no `CollisionAPI`; applied at load as `convexHull`, mug handle filled in |
| C-42 | MM5 stands the robot at a reach pose instead of navigating to it — C-26 makes `MoveToNamed` 0/6 |
| C-43 | MM5 stages the object on the table's near edge; mid-table is outside the arm's workspace |

## Open questions for Mohammed

1. **C-39 and MM5's 0/20 are the same problem.** Is the priority now (a) close C-39 so
   SONIC balances and both gates unblock, or (b) retrain the omni policy with arm motion
   in-distribution? PROVENANCE_v2 already argues the twin's hand mass needs reward
   re-shaping; this is a second, independent reason to want it.
2. **Should MM5 keep the omni balancer at all?** Given the above, the SONIC variant may be
   the only one that can ever pass, in which case MM5 is blocked on MM4 rather than
   partially independent of it.
3. **Is a fixed-base or gantry variant of MM5 worth having** as an interim, to prove the
   grasp/lift/carry/place stages that no trial has yet reached? It would test everything
   downstream of the balance problem, at the cost of not being the robot.

## Reproduce

```bash
ROBOT=g1 TWIN=1 TWIN_CAMERA=0 HAND=dex5_1p SIM_WORLD=panthera_lab \
  G1_CONTROL=policy TWIN_ARMATURE=0.01 MM5=1 MM5_TRIALS=20 \
  bash scripts/01_start_sim.sh
# results land in isaac/mm5_out/mm5_results.{json,md} and mm5_log.txt
```

---

# The fixed-base variant — the reported MM5 result

**Verdict: 0/20 = 0.0 %.** Base held kinematically so that everything
downstream of balance is exercised for real. **This is not the robot** and every row says
so: the carry stage is skipped (carrying a payload 1 m is meaningless when the base
cannot move), and the base stands 0.38 m from the object, inside the clearance Nav2 would
demand of the table.

```
20 trials, soup_can, seeds 20260821..20260840, cheat_attach false on every row
success 0/20 = 0.0%
taxonomy: `REACH_TIMEOUT` 11 · `LIFT_FAILED` 9
stage timing (mean): APPROACH=6.0s(n=20)  REACH=20.5s(n=20)  GRASP=2.0s(n=9)  LIFT=8.0s(n=9)
```

**9 of 20 trials reached the pre-grasp and closed the hand**, and
9 got as far as LIFT — the first time in this program the pipeline has gone
past the reach at all. W4's hand never reached the can.

## The two false positives, and why this number is the second one

The first N=20 fixed-base run reported **2/20 success. Both were artefacts and neither is
in the number above.**

`obj_start` was captured by re-reading the object immediately after `set_world_pose` —
which does not take effect until physics steps, so it returned the *previous* trial's end
pose. When that trial had knocked the can onto the floor, `obj_start` came back at
z = 0.032, and the LIFT test (`obj_z − obj_start_z > 0.05`) passed in a single step on a
can that had never been touched. Both "successes" showed `LIFT: 0.005 s`.

The runner knows exactly where it placed the object, so it now passes that in rather than
re-reading, and a trial whose object is not staged on the table fails as
`HARNESS_OBJ_NOT_STAGED` instead of being scored. **The run was repeated from scratch
before any number was reported, and the corrected number is 0/20** — so the entire
apparent success rate was the bug. Reporting the 2/20 would have claimed the program's
first Dex5 grasp on the strength of a stale pose read.

## What limits it

* **Workspace, not solver.** With the base 0.72 m back the IK converges rock-stable to
  354 mm and stops — the right arm's effective forward reach is ~0.39 m from the pelvis,
  because it is also reaching *down* from a 1.1 m shoulder to a 0.80 m can. Moving the
  base to 0.38 m fixed it; moving it to 0.32 m put the body inside the table and knocked
  the can onto the floor before the arm arrived.
* **Position-only IK.** `pose_error` is called with no orientation target, so the palm's
  orientation at the pre-grasp is whatever the null space leaves it. A side grasp needs
  the palm facing the can, and that is the most likely single cause of the
  `LIFT_FAILED` rows: the hand closes in the right place, in the wrong attitude.
  Adding an orientation target is the obvious next step and the module already takes one.
* **Two distinct `LIFT_FAILED` modes**, visible in the detail column and worth splitting
  in the taxonomy next time: rows reading `object rose -0.015 m` are a hand that closed
  and did not grip, while `object rose -0.765 m` is a hand that **knocked the can off the
  table** — 0.765 m is the table height. Same bucket today, different problems.

## Four solver bugs found getting here, all silent

1. **The palm pose was frozen.** `get_body_poses()` does not exist on this Articulation,
   and the USD fallback reads `ComputeLocalToWorldTransform` — which PhysX never writes
   back. The IK servoed against a constant z = 0.706 for 25 s while the arm was visibly
   moving. `SingleRigidPrim` reads the physics pose.
2. **`target = measured + dq` cannot outrun its own tracking error** — the palm crept
   0.11 m of a 0.70 m reach and stalled. Integrate from the previous *target*.
3. **No joint-limit awareness** — DLS wound `right_shoulder_yaw` into its −2.618 stop and
   held it there while the task error oscillated 370–435 mm.
4. **No null-space use** — the arm is 7-DoF against a 6-DoF task, and without projecting
   a posture bias through `(I − J⁺J)` the solver could not escape the configuration it
   had chosen.

And one that was not the solver: at kp 40 the arm lags its target ~0.1 rad under a kilo
of hand, and that lag *is* the servo problem — a small lead cap is consumed by it and a
large one closes the loop through it and limit-cycles. With the base held there is
nothing to destabilise, so the fixed-base variant stiffens the arm to kp 400 and the
oscillation disappears.
