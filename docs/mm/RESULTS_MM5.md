

---

# Grasp v2 — orientation-constrained IK, N=20

**Verdict: 0/20 = 0.0 %.** Fixed base, top-down grasp, real contacts,
cheat-attach off on every row.

```
20 trials, soup_can, seeds 20260821..20260840
success 0/20
taxonomy: `REACH_TIMEOUT` 12 · `NO_GRIP` 7 · `DESCEND_TIMEOUT` 1
stage reach:  REACH 20/20   DESCEND 8/20   GRASP 7/20   LIFT 7/20
```

## What v2 changed

* **6-DoF IK with the palm orientation constrained.** The approach axis is measured, not
  chosen: at the pre-grasp the palm's local z lands on world `[-0.096, -0.081, -0.992]`,
  so this arm's natural reaching posture already presents the palm *downward* and the
  grasp this robot wants is **top-down**. `topdown_quat()` snaps that axis to straight
  down and leaves the roll about it free — a 5-DoF constraint, which is all a
  rotationally symmetric can needs. Orientation error is weighted 0.35 against position
  so it cannot steal authority from a reach that is already workspace-limited.
  The effect on REACH was immediate: pre-grasp reached in ~8 s where position-only IK
  had been timing out at 35 s.
* **A separate DESCEND stage.** Failing to get *above* the can and failing to come *down*
  onto it are different problems and now land in different rows.
* **The `LIFT_FAILED` bucket is split**, as it should have been the first time:
  `NO_GRIP` is a hand that closed and did not hold; `KNOCKED_OFF_IN_LIFT` /
  `KNOCKED_OFF_IN_DESCEND` are the can leaving the surface. The old bucket was hiding
  `rose -0.765 m`, which is the table height, next to `rose -0.015 m`, which is a grip
  that slipped.

## What still stops it

The arm is at the floor of its workspace at table height. From a fixed base 0.38 m away
the palm tops out around z = 0.95 with the can's centre at 0.80; it reaches the pre-grasp
0.13 m above the can and then cannot descend the last few centimetres, so the hand closes
too high to wrap the can. The `NO_GRIP` rows are that, not a friction or gain problem.

**The fix is height, not tuning**, and it was attempted: the lab's counter is 0.90 m,
15 cm higher, which would put the can inside the workspace rather than at its edge.
Staging there needs its own base placement — the approach is from −y, which puts the
right arm on the +x side — and that geometry was not tuned inside the time box (it
reached only 577–641 mm). It is the documented next step and `surface: "counter"` is
already wired.

---

# Grasp v3 — the base pose computed instead of tuned

**0/20 again. Still no lift, so the program still has no real grasp.** What changed is
where the failure lives: v2 lost 12 of 20 trials before the arm ever got near the can,
v3 loses none there. **20/20 reach the pre-grasp and 20/20 enter the descent.** The
whole distribution has moved onto the last 50 mm.

| | v1 | v2 | **v3** |
|---|---|---|---|
| REACH reached | 9/20 | 20/20 | **20/20** |
| DESCEND reached | — | 8/20 | **20/20** |
| GRASP reached | 9/20 | 7/20 | 2/20 |
| LIFT reached | 9/20 | 7/20 | 2/20 |
| success | 0/20 | 0/20 | **0/20** |

v3 taxonomy: `DESCEND_TIMEOUT` 14, `KNOCKED_OFF_IN_DESCEND` 4, `NO_GRIP` 2.

## What v3 changed, and why each change was a measurement

**The base pose is solved, not iterated.** v1 and v2 both picked a stand-off, ran 20
trials, read the error and picked another; both landed at the workspace edge, and the
counter never converged at all inside its box (577-641 mm). v3 measures the right
shoulder in the robot's own frame at the settled home pose --
`forward -0.035, right +0.178, up +0.198` from the pelvis -- and then solves the one
remaining unknown from `target_r^2 = lateral^2 + forward^2 + vertical^2`. One
measurement, one solve: `base_offset [-0.178, -0.276]`, putting the can 0.315 m from
the shoulder and 0.276 m forward of the pelvis. If the vertical term alone exceeds
`target_r` it says the target is unreachable at that pelvis height instead of
returning a complex root, which is the information the counter attempt lacked.

**The approach axis is computed per target.** v2's top-down grasp was itself a
measurement -- the palm's local z came out at `[-0.096,-0.081,-0.992]` reaching DOWN
to a 0.75 m table -- but it was then hard-coded, and on the 0.90 m counter the can
sits 0.05 m below the shoulder rather than 0.30 m, where a top-down grasp asks the
elbow to climb over the hand. Measured at the counter pre-grasp, the palm's z was
`[-0.698,-0.700,0.153]`: nearly horizontal and pointing AWAY from the can. v3 picks
the axis from the can's height relative to the shoulder -- straight down below
0.10 m of drop, horizontal above 0.25 m, blended between -- and the palm now arrives
with its z on the can (`[-0.011, 1.000, 0.009]`).

**Three bugs found and fixed on the way**, all mine, all worth recording because each
one produced a plausible-looking result:

- the `forward`/`right` basis vectors were swapped (`[-sin, cos]` is the robot's
  LEFT), so the solve put the base beside the counter instead of in front of it --
  invisible in the output, because both are unit vectors and the solve still
  "converged";
- `counter_xy` staged the can at y = 2.05, five centimetres in FRONT of a counter
  whose near edge is y = 2.10, so it fell to the floor on trial 1 before the arm moved;
- the approach axis was recomputed every step from the shoulder, and the shoulder
  MOVES with the arm -- a target in a feedback loop with the thing chasing it. A reach
  sitting at 164 mm walked steadily out to 1426 mm. Frozen per trial, it holds.

Fixing the third turned DESCEND from a 102 mm timeout into a 25 mm arrival.

## What is actually stopping the grasp now

The descent stalls **47-59 mm short in most timeouts**, and that number barely moves
even though the can is randomised over a ±6 cm box. A workspace boundary travels with
the target; a fixed stall distance does not. That is a cap signature, and it is the
same one v1 recorded one stage earlier ("the reach stalled at a repeatable 350 mm
short") when the IK lead cap was 0.10.

**Tested: lead cap 0.25 -> 0.60. Reverted.** The stall stayed at 47-59 mm, and the
faster arm knocked the can over in 9 of 20 trials against 4, with `REACH_TIMEOUT`
reappearing in 3. So the cap is NOT what holds the last 50 mm; raising it only trades
a stall for a collision. That probe is kept as
`evidence/MM5/graspv3_leadprobe_results.json` rather than quietly dropped.

The two trials that did close the hand reported `object rose -0.017 m` -- the can is
being pushed, not gripped. The next measurement is the one v3 did for the palm axis
and has not yet done for the fingers: where the Dex5's finger contacts actually sit in
the palm frame, so `grasp_standoff` is measured against the surface the fingers close
on rather than assumed. `grasp_standoff` was moved from 0.072 to 0.045 on exactly that
reasoning (0.072 meant "above the lid" for a top-down grasp and "39 mm of air" for a
side one), and it traded `NO_GRIP` for `DESCEND_TIMEOUT` -- which says the number
matters and that the right value is between them, but it should be measured, not
bisected.

## Media

No video, per C-23 — deferred to the 4090 day. A **joint trace** is written instead:
`evidence/MM5/graspv3_trace.jsonl`, 18063 samples at 50 Hz, carrying per-sample arm
joint angles, hand joint angles, palm position and quaternion, object pose and pelvis
height for every trial. That is enough for the 4090 day to replay an episode for
camera without re-running twenty trials of physics to find a good one.

---

# Grasp v4 — the hand measured, and the axis was wrong

**0/20 (10 soup can, 10 × 5 cm cube). Still no lift.** But v4 found the defect that
made every previous version close on air, and it was not the stand-off everyone was
bisecting — it was the **axis**.

## The measurement

Drive the fingers closed, then read every link of the hand out of PhysX in ONE tensor
call and express it in the palm frame (`dex5_geom.py`). At the closed pose the four
distal finger links converge here:

| link | palm-frame xyz | travel when closing |
|---|---|---|
| `Link_34R` | `[-0.012, 0.142, -0.003]` | 0.0872 m |
| `Link_44R` | `[-0.034, 0.138, -0.003]` | 0.0870 m |
| `Link_54R` | `[-0.056, 0.134, -0.003]` | 0.0869 m |
| `Link_24R` | `[+0.010, 0.138, -0.003]` | 0.0859 m |

**The fingers close about `[-0.021, 0.135, -0.003]`, 0.1366 m from the palm origin,
along the palm's local +Y.** Two things follow, and both are corrections to work that
looked finished:

1. **The stand-off is 0.147 m, not 0.045 or 0.072.** Every version of this pipeline
   placed the palm roughly THREE TIMES too close — the palm was essentially at the can,
   so the fingers had nothing to close behind and the palm simply pushed it. That is
   what `object rose -0.017 m` had been reporting all along: not a weak grip, a shove.
2. **The grasp axis is the palm's +Y. v2 and v3 constrained its +Z.** `topdown_quat`
   and then `approach_quat` both aligned an axis the fingers do not use. Every "the
   palm now arrives with its z on the can" line in the v3 write-up was true and
   irrelevant. The palm was pointed correctly for the wrong axis.

`approach_quat` is now a minimal rotation that carries a *measured* palm-local axis
onto the world approach direction, rather than a hand-written "flatten +z" construction.

## Result

| | v3 | **v4 can** | **v4 cube** |
|---|---|---|---|
| REACH / DESCEND reached | 20/20 | 10/10 | 10/10 |
| GRASP reached | 2/20 (10%) | **6/10 (60%)** | 1/10 |
| LIFT reached | 2/20 | **6/10** | 1/10 |
| success | 0/20 | 0/10 | 0/10 |

v4 can: `NO_GRIP` 6, `KNOCKED_OFF_IN_DESCEND` 2, `DESCEND_TIMEOUT` 2.
v4 cube: `DESCEND_TIMEOUT` 6, `KNOCKED_OFF_IN_DESCEND` 3, `NO_GRIP` 1.

The measured geometry moved GRASP from 10% to 60% on the can — the single largest
improvement any version has produced — and the hand now closes around the can rather
than beside it. It still does not hold it.

**The cube is the control and it says something useful.** A 5 cm cube is lighter
(0.100 kg vs 0.349) and boxy, so if the failure were grip mechanics the cube should do
BETTER. It does worse on reach (`DESCEND_TIMEOUT` 6 of 10) because a smaller object
puts the grasp pose deeper in, but its one `NO_GRIP` reports `object rose -0.002 m`
against the can's -0.017: the cube is barely disturbed. So the palm is no longer
shoving the object — that half is fixed — and what remains is closure force or finger
travel, not placement.

## Not done, and stated rather than skipped

The brief also asked to **trigger finger closure on the DT3 contact zones firing rather
than on distance.** That is not implemented; closure is still distance-triggered at
`grasp_tol`. It is the right next change and it is now the ONLY item left from the v4
brief — but it affects closure TIMING, and the surviving failure is a hand that closes
fully and still does not hold, so it was not the thing standing between v4 and a lift.
Recorded as open rather than quietly dropped.

---

# Grasp v5 — closure made force-generating

**0/20 (10 can, 10 cube). Still no lift.** Two of the four v5 items landed and changed
the behaviour; two could not run, for a reason worth recording.

| | v4 can | **v5 can** | **v5 cube** |
|---|---|---|---|
| GRASP reached | 6/10 | 6/10 | 5/10 |
| `NO_GRIP` | 6 | **4** | 3 |
| `KNOCKED_OFF_IN_LIFT` | 0 | **2** | **2** |
| success | 0/10 | 0/10 | 0/10 |

**(b) Overclose — done.** Finger targets are driven `overclose_rad = 0.25` past the
contact pose. A position drive makes force out of ERROR, so a target set AT contact
produces a hand that touches with almost none; the fingers cannot reach the overclosed
target and the residual error is the grip.

**(c) Phase gains — done.** C-38 capped the finger drives at kp 5.0, and that is the
SONIC-IDLE value, chosen so an idle hand does not fight itself — not a grasping value.
GRASP and LIFT now run at kp 20 / kd 0.5 and revert to 5.0 / 0.1 when the trial ends,
so the idle hand is unchanged and the 0.93 Nm URDF effort clamp still bounds the result.
The log shows the transition on every trial.

**The effect is real and visible in the taxonomy.** `NO_GRIP` drops from 6 to 4 on the
can, and **`KNOCKED_OFF_IN_LIFT` appears for the first time on both objects (2 + 2)** —
the hand now moves the can while lifting, where v4's hand simply let it sit. That is a
hand that has begun to grip and cannot yet hold. It is progress and it is not a lift.

**(a) Contact counting and (d) the DT3 lift gate — COULD NOT RUN.**
`get_net_contact_forces` is **absent on both the articulation wrapper and the physics
view** in this Isaac build:

    NO net-contact-force API: ['view: absent', 'physx: absent']

Both items depend on it, so both are inactive, and the code says so at runtime rather
than silently returning zero contacts. **This matters for how the rows above are read:
no `NO_GRIP` row here is evidence about colliders**, because the question "are the
fingers touching the object at all" was never answerable. The same absence corrected a
C-39 result in the other workstream — the foot "contact" forces there are actually
`get_link_incoming_joint_force`, a joint reaction, not ground contact.

Finding the contact route that does work in this build is the next step for both
workstreams, and it is one piece of work, not two.

---

# Grasp v6 — the contact route, attempted twice, still not attached

**0/20 (can 0/10, cube 0/10). No lift.** v6 kept v5's overclose and phase gains and
tried to add the contact-triggered close and lift gate on the route task 1 identified.
**The route did not attach, so v6 is behaviourally v5** and its rows carry the same
caveat: `grip_contacts` is `-1` on every trial, so **no `NO_GRIP` row in this campaign
is evidence about finger colliders.**

| | v5 can | **v6 can** | **v6 cube** |
|---|---|---|---|
| GRASP reached | 6/10 | 5/10 | 2/10 |
| `NO_GRIP` | 4 | 4 | 2 |
| `KNOCKED_OFF_IN_DESCEND` | 1 | 1 | 3 |
| `KNOCKED_OFF_IN_LIFT` | 2 | 1 | 0 |
| `DESCEND_TIMEOUT` | 3 | 4 | 5 |
| success | 0/10 | 0/10 | 0/10 |

## Why the contact route did not attach, precisely

Two attempts, both instructive, neither successful:

1. **Sensors on the link Xform.** `ContactSensor` prims were created at world setup on
   `/World/G1/Link_{1..5}4R` — creation reported 5/5 — and then every one failed at
   `initialize()` with *"Contact Sensor needs to be created under another prim that has
   collision api enabled on"*. The link Xform has no collision; the collider lives at
   `<link>/collisions/...`. **Creation succeeding and initialisation failing much later
   is the trap here** — the setup log says 5/5 and looks fine.

2. **Sensors on the collision child.** Re-parenting to the first descendant carrying
   `CollisionAPI` found **0/5**, because the search used `Usd.PrimRange(prim, predicate)`
   — which does not descend into instance proxies. Corrected to `PrimRange.Stage` with a
   path-prefix filter, and it still reported 0/5 at setup time.

The most useful thing v6 produced is the answer to the question underneath all of it:
**the fingers DO have collision geometry.** A stage scan with instance proxies finds
**42 prims with `CollisionAPI`** across the two hands —
`/World/G1/Link_44L/collisions/Link_44L/node_STL_BINARY_` and its 20 siblings per hand,
plus each palm. So the hand is *not* closing through the object, and the collider
hypothesis behind grasp (a) is dead. What remains unexplained is why a sensor cannot be
parented to those prims at setup time, and that is one focused piece of work rather than
a campaign.

## Standing caveat on every grasp result so far

`grip_contacts = -1` in v5 and v6 means the lift gate never armed and no trial's outcome
was decided by measured contact. The taxonomy is honest about stage reach and object
motion; it is silent on whether the fingers touched.

---

# Grasp v7 — contact-report route, counter surface, N=10 soup can

**Verdict: 0/10 = 0.0 %.** Fixed base, real contacts, `cheat_attach` off on every row.
Balancer: omni policy standing — **SONIC is parked** (C-39, `BISECT_VERDICT.md`), which
CAMPAIGN §4.4 permits explicitly ("or omni standing if MM4 slips").

```
10 trials, soup_can, seeds 20260823..20260832, counter (0.90 m)
success 0/10
taxonomy: DESCEND_TIMEOUT 9 · REACH_TIMEOUT 1
```

## What v7 changed, and what each change actually bought

| change | bought |
|---|---|
| **PhysX contact-report route** | The v6 blocker is gone: `contact-report route ACTIVE: 22 prim(s), 21 finger link(s)`. No sensor prim, no re-parenting. **Still unexercised** — no trial reached GRASP, so `grip_contacts` is *still* `-1` and this remains unproven at closure |
| **`obj_pose` reads physics, not USD** | Real defect, fixed. It was returning the authored pose — 138 mm from the truth. **Did not move the outcome** (93→91 mm reach stall) |
| **staging read-back** | Staging was always applying; the *read* was the liar. Now verified at 0.0 mm |
| **counter (0.90 m) instead of table (0.75 m)** | **The one change that moved a number.** REACH went from timing out at 30 s to succeeding in **~2.4 s on 9/10** |

## Where the failure now lives, precisely

It has moved from REACH to DESCEND, and the object table says why:

| trial | obj start → obj end | displaced |
|---|---|---|
| 3 | [-2.5697, 2.250] → [-2.4061, 2.5827] | **0.37 m** |
| 6 | [-2.5588, 2.2367] → [-2.4919, 2.6022] | **0.37 m** |
| 10 | [-2.5979, 2.256] → [-2.5590, 2.5516] | **0.30 m** |
| 9 | [-2.6406, 2.2672] → unchanged | 0 (never touched it) |

In 8 of 10 the can is **shoved a third of a metre across the counter** and drops ~17 mm
in z (0.9498 → 0.932) — it is being knocked over. The descent is driving the palm *into*
the can rather than around it, and then timing out 80–203 mm from the grasp pose.

That is a different failure from every previous version. v2–v6 could not get the arm
*near* the can; v7 gets there in 2.4 s and then collides with it. The remaining defect is
in the **descent geometry** — the approach axis and stand-off are computed for a target
below the shoulder, and on the 0.90 m counter the can sits roughly *level* with it, which
v3 already identified as needing a per-target axis rather than a top-down one.

## Honest statement of what is NOT known

**Whether the fingers can hold this can is still untested.** No trial has reached GRASP,
so the contact route has never reported a closure, `grip_contacts` is `-1` on every row,
and **no `NO_GRIP` row in this campaign is evidence about colliders or grip force.** That
caveat has been carried since v5 and it still stands.

## Time-box

Grasp exceeded its 90-minute box. Per the brief's fallback rule the deliverable is this
table with an honest taxonomy rather than an open-ended chase of the descent servo; the
next session starts at the descent geometry, with REACH already solved and the contact
route already attached.
