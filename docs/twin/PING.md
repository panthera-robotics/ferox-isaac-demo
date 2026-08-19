# PING — RESOLVED 2026-08-19

> **Outcome, both items, from Mohammed:**
>
> 1. **C-23 is the GPU. Do not work around it on this box.** The camera path is
>    verified only on RTX 4090 / driver 580.105. `RESUME.md` §1 now carries the
>    `nvidia-smi` check to run before any camera item, and the five-boot evidence
>    is at [`evidence/C23/README.md`](evidence/C23/README.md). Item C, E-1, C-21's
>    live re-proof and the montage's camera clip all wait for a 4090 box.
> 2. **Nav 1 of 3 is not a defect.** The evidence run was scoped wrong, not the
>    twin. `RESUME.md` §0 next-steps carries the measured free-space bounds to
>    scope the next run against, or the alternative of spawning in an enclosed
>    room; ≤ 1 h on the next box.
>
> Also accepted in the same pass: `/scan` at **45 %** is the correct answer for a
> mid-corridor spawn against a 6.0 m `range_max` (noted in `RESULTS_DT2.md`), and
> the azimuth gate stays at **300°** against the bag's measured 310–360°.
>
> The text below is kept as the record of what was asked.

---

# PING — 2026-08-19 — the box cannot run the camera, and one decision follows

**Raised:** 2026-08-19, after the video-review fixes
**Class:** environment (C-23), plus one scoping question
**Blocks:** item C entirely, E-1 entirely, C-21's live re-proof, and the montage's
camera clip. Does **not** block anything about the lidar, the hands, nav or the audit.

---

## 1. This box segfaults Isaac whenever the ROS 2 image writer is live

Five boots out of five, at `run.py:1201` on the first `world.step(render=True)` after
a camera render product exists, with a native backtrace through
`libomni.syntheticdata` and `libomni.graph.image.core`.

What it is **not**, each ruled out by measurement rather than by argument:

| ruled out | evidence |
|---|---|
| memory | peak VRAM **3776 MiB of 16376**, peak host RSS **5.7 GB of 47**, sampled at 1 Hz through the crash; no OOM, no Xid, `/dev/shm` 24 G |
| the merged hand asset | crashes identically on the pre-existing committed asset |
| the world | reproduced in both `dso_block_a` and `hospital` |
| the depth annotator | removing it changes nothing; the default `rgb` annotator does it too |
| "any camera" | the **offscreen** render path works fine — both capture scripts and both orbit renders complete |

What isolates it: the **Go2 twin boots every time** on this same box with the same
RTX lidar and the same bridge, and its contract has no camera.

**The box is not the one the campaign was validated on.** RESUME §1 records an RTX
4090 with 48 GB. This is an **RTX 4080 SUPER with 16 GB**, 47 GiB of RAM against 98,
and no tailnet. Driver and Isaac version are the documented ones.

`annotator_device="cuda"` was tried and reverted: it stops the crash by stopping the
synthetic-data pipeline, and both render-product topics go silent — the camera image
*and the lidar cloud* — while the rclpy publishers keep running so the sim looks
perfectly alive. That is a worse failure than the crash, and this campaign exists
because of quiet failures.

`TWIN_CAMERA=0` (default **on**) skips the camera device and changes nothing the
audit checks. It is what kept the lidar/nav half of today's work possible.

### The question

**Do you want a 4090 box to finish item C, E-1 and the montage's camera clip?**
Everything else is done and pushed. If C-23 is the GPU, a 4090 instance closes all
three with no code change. If you would rather I chase it on this hardware, say so —
but I have no evidence left that points at the twin rather than the box.

---

## 2. DT2 navigation: 1 of 3, not 3 of 3

The brief asked for three Nav2 goals SUCCEEDED. **Goal 1 SUCCEEDED — the first this
twin has ever reached.** Goals 2 and 3 ABORTED, and the reason is measured: both
targets sat outside the ~7.5 × 7.0 m the SLAM map had grown by then.

Nothing was tuned to get goal 1, and I did not tune anything to chase 2 and 3 —
DT2's standing decision is that the footprint/inflation interaction is Ferox-side
and deliberately untouched. The obvious next try is a longer mapping drive before
the goals, which is minutes of work but is a change of method rather than a fix, so
it is yours to call.

---

*Everything below is the resolved PING from 2026-08-18 and is kept as the record.*

---

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
