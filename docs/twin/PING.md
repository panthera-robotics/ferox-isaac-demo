# PING — RESOLVED 2026-08-18

> **Outcome: the contract stays dynamic. Option A stands.**
> The bag shows the robot was running `lidar_tf_mode=static` (the Session-A
> composite). That is a robot-side configuration/verification item, not a twin
> defect, and it goes to Kevin as **OQ-6** — see `RESULTS_GT_G1.md` §4, which
> carries the check command and both expected outcomes.
>
> Nothing in the contract changed. The text below is kept as the record of the
> measurement that raised the question.

---

# PING — the ground-truth bag contradicts the DT2 Option-A decision

**Raised:** 2026-08-18, from `ref/captures/g1/captures/g1_twin_gt`
**Class:** A (contract vs robot), one item
**Blocks:** nothing right now — the twin runs and is Class-A clean against its own
contract. But the contract and the robot disagree, and only you can say which moves.

---

## The mismatch

`base_link → livox_frame`:

| | Contract (after DT2 Option A) | Robot, in this capture |
|---|---|---|
| where | **dynamic**, on `/tf` at ≥50 Hz | **static**, in `/tf_static` |
| in the static set | absent | **present** |
| value | composed live from the waist joints | `xyz [0, 0, 0.4995]`, `rpy [3.090233, 0.161680, 0]` |

`/tf` in the bag carries **one** edge only — `odom → base_link`, 400 of 400 samples.
There is no waist-composed lidar edge on it at all.

The static value is **byte-identical** to the Session-A edge the contract used to
carry before DT2, so nothing has drifted; the robot is publishing exactly what DT0
recorded. What changed is that DT2 moved the contract to Option A on the premise that
the driver's default is `lidar_tf_mode=waist` and the static edge is only its
fallback.

## Why it is not just bookkeeping

The static composite bakes in a waist pitch. Measured from the edge itself:

```
edge pitch                     +0.161680 rad  = +9.264 deg
contract mount-only pitch      +0.052979 rad  = +3.036 deg
waist contribution baked in    +0.108701 rad  = +6.228 deg
```

Fitting the floor from `/livox/lidar` **through the bag's own TF** — valid points
only, 896–918 points per sweep, robust fit — gives:

| sweep | floor z under `base_link` | tilt | fit RMS |
|---|---|---|---|
| 0 | −0.62527 m | **5.874°** | 0.0211 m |
| 1 | −0.61723 m | **6.974°** | 0.0049 m |
| 2 | −0.61750 m | **6.873°** | 0.0094 m |

A 5 mm RMS is a genuine plane, not a mixture. And the tilt is, within fit noise, the
**6.228° waist term**. Meanwhile the robot's own odometry says its body was level:
pitch **+0.518°** (`base_link`) and **−0.108°** (`pelvis`), constant across the capture.

The floor height is consistent with the same error: −0.617 m under `base_link`, where
C-1's recorded 0.175 m p2l margin implies −0.731 m.

**Reading:** in this configuration the robot's published lidar TF describes an
attitude the robot was not in, by about 6.6°. Everything derived from that cloud in
`base_link` — the p2l slice, the costmap, any map built from it — inherits the tilt.

Two ways that can be true, and I cannot tell them apart from one bag:

1. **The capture ran with the fallback.** `lidar_tf_mode=static` was selected (or
   `waist_tf_bridge` was not up), so the driver emitted the static composite. The
   default really is `waist`, DT2's premise holds, and the twin is right — but then
   the *fallback itself* is wrong whenever the robot is not at 6.2°, which is most of
   the time.
2. **The default is static.** DT2's premise was wrong, the twin should publish the
   static edge to match, and the robot has a real ~6.6° geometry error in its default
   configuration.

## What I did and did not change

* **Did not** touch the contract's `base_link → livox_frame`. It still says dynamic.
  Reversing your DT2 decision on one bag is not mine to do, and the sim's own floor
  check passes at **0.0039°** with the dynamic edge — the twin is currently *more*
  correct than this capture.
* **Did** close everything the bag settles cleanly: OQ-2, OQ-3, and four
  provenance upgrades to `captured`. Those are in `RESULTS_GT_G1.md`.

## What would settle it

On the robot, driver up, one line:

```bash
ros2 param get /waist_tf_bridge lidar_tf_mode      # or: ros2 node list | grep waist
ros2 topic echo --once /tf | grep -A2 livox_frame  # is it ever on /tf?
```

If `lidar_tf_mode` is `waist` and the node is running, this capture was the fallback
and reading 1 applies. If there is no such node or the mode is `static`, reading 2
applies and the contract should move back.

**Either way there is a second question worth asking Kevin:** should the static
fallback exist at all, given it is only correct at one waist angle?
