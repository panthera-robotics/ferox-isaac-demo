

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
