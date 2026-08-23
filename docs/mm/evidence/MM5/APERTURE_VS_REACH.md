# Aperture vs reach — they are in direct conflict, and the limiting joint is the ELBOW

2026-08-23, after Task 1 closed as `reachable-but-pinches`. Preflight green before every
number.

## The two configurations, measured

| `grasp_clearance` | reach | enclosure | outcome |
|---|---|---|---|
| **−0.015** | **works** — closure repeatable | **0 links within 45 mm of the object axis, 0 below its centre** | 6 contacts at 43.8 N, **pinch**, no lift |
| **−0.030** | **fails** — 0 closures in 10 | not measurable (never closes) | 8 `DESCEND_TIMEOUT`, 2 `REACH_TIMEOUT` |

## Why −0.030 fails, per the classifier

    IK_INFEASIBLE x3   pinned joint 3 (right_elbow)  2.058 / 2.087 / 2.091  vs limit 2.044
    SERVO_SLOW    x5   residual 49-119 mm, nothing pinned

**The elbow runs out of travel.** Bringing the palm 15 mm closer — which is what putting
the object between the finger segments requires — asks `right_elbow` to flex past
2.044 rad. This is not the wrist limit that Task 1a fixed; that fix holds (wrists are
clear). It is a different joint and a different constraint.

## What this means, stated plainly

**At this base position and counter geometry the hand cannot both reach the object and
enclose it.** Reach and aperture are in direct conflict:

* far enough out that the elbow is comfortable → the object sits at the fingertips → pinch
* close enough to cage → the elbow exceeds its limit → never closes

That is a **kinematic/geometry** result, not a tuning one. No gain, gate or contact
parameter changes it.

## The lever this points at

**Base repositioning** — lever (b) from the Task-1 plan, which was correctly skipped then
(reach was fine at −0.015, so moving the base addressed a non-binding constraint) and is
now the *motivated* next step: place the robot so the caging stand-off falls inside the
elbow's travel rather than beyond it. The measured shoulder-optimal reach is ~0.315 m; the
counter-edge staging is what forces the over-flex.

**Not opened here.** This document records the conflict and the joint; the base change is
Mohammed's call.

## Reproduce

    bash scripts/mm5_preflight.sh            # must print ALL GAUGES VERIFIED
    # then, with grasp_clearance = -0.030 in isaac/twin/mm5/runner.py:
    ROBOT=g1 TWIN=1 TWIN_HEADLESS=1 HAND=dex5_1p SIM_WORLD=panthera_lab MM5=1 \
      MM5_OBJECT=cube_5cm MM5_TRIALS=10 MM5_FIX_BASE=1 MM5_SURFACE=counter \
      MM5_MEASURE_HAND=1 MM5_GRASP_MU=1.2 bash scripts/01_start_sim.sh

---

## Lever (b) tested and it does NOT resolve the conflict

`MM5_TARGET_R` 0.315 → **0.375** (base moved back for elbow room), caging clearance
`−0.030`, N=10 cube, preflight green:

    IK_INFEASIBLE x5   elbow (3) 2.071 / 2.094 vs 2.044
                       wrists (5) -1.610 / 1.614 and (6) 1.565 vs 1.564
    SERVO_SLOW    x1   residual 55 mm
    TOPPLED       x1   30.4 deg (verified gauge)
    closures       0

Standing further back does not buy the elbow room the caging pose needs, and it brings the
**wrists** back to their stops as well — the arm simply runs out of configuration in a
different way.

## Verdict — the grasp line stops here

**Both authorised levers are exhausted.** (a) wrist-aware IK fixed the wrist limit and made
pre-grasp reachable and closure repeatable — but only at a stand-off that **pinches**.
(b) base repositioning does not make the **caging** stand-off reachable.

**At the 0.90 m counter, with this arm and this hand, reach and enclosure cannot be
satisfied together.** That is a kinematic property of the configuration, not a parameter
that has been mistuned. Everything downstream of contact — force, friction, contact count,
thumb opposition, lift vector, lift rate, grip maintenance — is verified working and is
irrelevant while the hand cannot get around the object.

## Options for Mohammed — none started, all outside the authorised levers

1. **Surface height.** The table (0.75 m) was abandoned as "below this arm's workspace" and
   the counter (0.90 m) forces the over-flex at caging depth. An **intermediate height**
   is untested and is the cheapest remaining experiment.
2. **Object size.** The 66 mm can and 50 mm cube both exceed the 21.5 mm tip spread. A
   slimmer object would sit between the finger segments at a reachable stand-off.
3. **Hand.** The Dex5's finger convergence (0.1366 m from the palm) is what sets the
   caging stand-off. Nothing in software changes that geometry.

**No further grasp sub-hypothesis is opened.** The enclosure logger, the IK classifier and
the preflight are all in place, so whichever option is chosen can be measured on the first
run rather than argued.
