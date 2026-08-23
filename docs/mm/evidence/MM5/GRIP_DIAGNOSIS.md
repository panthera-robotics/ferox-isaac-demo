# Grasp force — diagnosed. It was never force.

2026-08-22. The brief asked why ~0.5 N was delivered against the ~3.4 N a 0.349 kg can
needs. The answer is that **0.5 N was not a measurement** — and once the instrument was
correct, the force turned out to be ample and the defect was somewhere else entirely.

## Three defects in my own instrument, found before any tuning

| # | defect | effect |
|---|---|---|
| 1 | the contact callback summed PhysX **impulses** (N·s) and printed them as newtons — and as an **L1 norm**, inflating a diagonal contact by up to √3 | "0.48 N" was a dimensionless artefact; it could not be compared to 3.4 N |
| 2 | first fix reported only the **last** substep, so a contact mid-closure vanished if the final substep was free | under-reported `contacts=0` where there was a real touch |
| 3 | count without **identity** | 4 contacts pushing one way and 2 opposing look identical |

Tuning gains against any of these would have produced a better-looking number and no
better grip.

## What the corrected instrument says

    q[max]=+1.600  target[max]=+1.600   err[max]=+0.920 rad
    kp=20.0 -> demand=18.39 Nm     |tau|max=0.006 Nm
    contacts=1  force=38.05 N
    contact links: Link_14R 6.8 N, Link_24R 2.8 N, Link_34R 0.5 N

Point by point against the brief's three hypotheses:

1. **Overclose commanding past contact?** *Yes, working.* Target 1.600 rad is past
   `close_rad` 1.35, and joint 0 sits **blocked at 0.680** — a real residual, i.e. a
   finger jammed on the can.
2. **Gain / effort clamp the ceiling?** *No.* Demand 18.39 Nm produces `|tau|max` of
   0.006 Nm — the URDF clamp would read 0.93. And the peak contact force is **38 N**
   against 3.4 N needed. Raising `kp` was never going to help.
3. **Contact count?** *This is the blocker, and it is geometric.* At the original
   stand-off, **11 of 20 joints reached target** — they closed on air.

## The two geometry fixes, and what each bought

| `grasp_clearance` | result |
|---|---|
| `+0.010` (convergence 10 mm **short** of centre) | 1 contact, 11/20 joints closing on air |
| `−0.015` (convergence **past** centre) | **3–4 contacts, LIFT reached**, thumb opposing: `Link_14R` 6.8 N vs `Link_24R`/`Link_34R` |
| `−0.030` (seat the can deeper) | under test — every contact above is a **distal** link, i.e. a fingertip pinch on a smooth cylinder, and it slips (`rose +0.001 m`) no matter the force |

## Retracted

I suspected the thumb was not opposing, because `_set_hand` drives all 16 actuated joints
to the same 1.6 rad while indices 0/1 (`Yaw_11R`, `Roll_12R`) are the thumb's abduction
and rotation axes rather than flexion. **The contact identities refute that**: `Link_14R`
is the thumb's distal link and it carries the *highest* force of the three. The uniform
target is still questionable on its own terms, but it is **not** what stops the lift.

---

# Session close, 2026-08-23 — no lift. What is ruled out, and what the last number says.

**0 lifts.** Best configuration reaches 3 opposing contacts at 23.9–35.6 N and the can
does not rise. Every hypothesis in the brief is now eliminated **by measurement**:

| ruled out | evidence |
|---|---|
| grip **force** | 38.05 N peak vs the 3.4 N a 0.349 kg can needs |
| finger **gain** | demand 18.39 Nm at kp 20; raising it changes nothing |
| URDF **effort clamp** | `\|tau\|max` 0.006 Nm — the clamp is 0.93, nowhere near it |
| **overclose** not commanding past contact | joint 0 blocked at 0.680 against a 1.600 target |
| **thumb not opposing** | `Link_14R` (thumb distal) carries the highest force of the three |
| **too few contacts** | 1 → 3–4, and the ≥2 gate now passes |
| **friction** | μ 0.5 → 1.2 raised force to 35.6 N and reached *middle* links; still no lift |
| **LIFT target geometry** | fixed (was a legacy vertical offset); still no lift |

## What the residual number means

`object rose -0.017 m` with a good grip. A 96 mm can that **topples** drops its centre
from 48 mm to ~33 mm — **−15 mm**. That matches. The can is not slipping out of the
fingers; **it is being tipped over as they close**, and then the hand is holding a can
that is lying down.

That also explains why every lever above failed: more force, more friction and a better
lift vector all act on an object that has already fallen over before the lift begins.

## The next experiment, precisely

1. **Instrument the object's ORIENTATION, not just its height.** The taxonomy cannot see
   a topple: it reports `NO_GRIP` because the can stays on the counter and never drops
   below `surface − 0.05`. Add can tilt to the trial row and a `TOPPLED` outcome. This is
   a one-line measurement and it is the thing that has been invisible all session.
2. **Then fix the cause of the topple** — the fingers close asymmetrically, so the first
   contact pushes the can before the opposing one arrives. Candidates: close the thumb
   *first* and the fingers into it, or slow the closure ramp so contacts arrive together.

## Bracket, measured, for whoever picks this up

| `grasp_clearance` | result |
|---|---|
| `+0.010` | 1 contact, 11/20 joints closing on air |
| **`−0.015`** | **3–4 opposing contacts, LIFT reached** — the working value |
| `−0.030` | 0 closures; the arm cannot reach the pose |

`MM5_GRASP_MU` is a **declared** deviation (CAMPAIGN §4.4), off by default, logged every
run. It is **not** load-bearing for any result: it improved contact quality and changed
no outcome.
