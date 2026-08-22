# C-39 task 1 — the decisive A/B. **The asset is exonerated. It is the SIMULATOR side.**

> **CONFIRMED under proven sole occupancy, 2026-08-22.** The first pair of runs was
> taken while a second agent instance existed and was void under Mohammed's rule
> (`PING.md`). Both were repeated as `bisect/baseline_twin` and `bisect/baseline_ref`
> with sole occupancy asserted before each, and **the result reproduces**: the reference
> body falls in our simulator, `base_z +0.098`, `pitch +86.3°`. The conclusion below is
> a result, not a hypothesis.

## Result

| | **twin_bare** (our 29-DoF body) | **ref** (`g1_29dof_old.xml`, MJCF-imported) |
|---|---|---|
| rig released at | 11.86 s | 11.05 s |
| base_z at release | **+0.781** | **+0.778** |
| base_z 2.5 s later | **+0.110** | **+0.110** |
| final base_z | +0.107 | +0.095 |
| final pitch | **+88.3°** | **+85.7°** |
| final roll | −179.7° | +179.4° |
| SONIC aborts | 0 | 0 |
| `crc_bad` | 0 | 0 |
| lowcmd | 87.2 Hz | 81.7 Hz |
| **verdict** | **FALLS** | **FALLS** |

**The reference body falls in our simulator, in the same time, to the same height, at
the same angle, as ours does.** Face-down, ~87°, base at 0.10 m, within 2.5 s of the rig
letting go — the two traces are barely distinguishable.

## What that settles

Per the brief's decision tree this is the **FALLS** branch: *"Falls → SOLVER delta."*

The same body **stands** under the same deploy binary in the reference MuJoCo sim —
`evidence/MM4/ab/mujoco.json`, 990 lowstate samples over 5 s at 197.03 Hz with
quat w = 0.999753. So:

* it is **not our asset** — the reference's own body fails here too;
* it is **not the wire** — cleared in turn 8 and again here (`crc_bad=0` both sides);
* it is **not the hands** — this is the hand-less body on both sides;
* it is **not SONIC** — same binary, same flags, live and commanding, zero aborts.

**What is left is our simulator's configuration**: solver type and iteration counts,
contact offsets, `dt`, drive mode, depenetration velocity, friction combine mode,
self-collision — i.e. everything the twin sets that the MJCF importer's defaults, and
MuJoCo, do differently. That is the task-1 "Falls" list, and it is now the whole
remaining search space.

## Conditions, identical on both sides

* `G1_CONTROL=lowcmd`, MM3 bridge on `lo`, SONIC `v1.1-x86_64`, planner over ZMQ.
* `TWIN=1` with **all three devices off** (`TWIN_CAMERA/TWIN_LIDAR/TWIN_IMU=0`).
* `--disable-crc-check`, which also disables upstream's `body_dq > 35` guard
  (`SONIC_ABORT.md`). Needed because the twin's own rig-held hold rings a joint past
  35 rad/s; applied to both sides so it cannot bias the comparison.
* Rig releases after **5 s** of unbroken authority; `HAND=none` both sides; same world,
  same spawn (both traces start at the hospital spawn yaw, quat w ≈ 0.7074).
* **The lowcmd seqlock fix is in** (`LOWCMD_SEQLOCK.md`). Without it neither side is
  measurable at all — the sim never completed a single command read and the rig never
  released.

## Provenance

    reference MJCF : gear_sonic/data/robots/g1/g1_29dof_old.xml @ NVlabs/GR00T-WholeBodyControl 54d0b10
    imported USD   : isaac/assets/g1_ref_mjcf/  (29 joints, 30 links, 35.112142 kg)
    import report  : evidence/C39/import_mjcf.txt   (every config field, before/after)
    raw            : evidence/C39/ab/{twin_bare,ref}_{sim,sonic,bridge,drive}.log
    verdicts       : evidence/C39/ab/{twin_bare,ref}_verdict.txt

## Honest caveat

Both sides run with the velocity guard disabled and a 5 s authority release, which is a
change from every earlier C-39 run. That makes the two sides comparable **to each other**
— which is all this experiment claims. It does not make either directly comparable to a
pre-fix number, and after `LOWCMD_SEQLOCK.md` no pre-fix torque/tracking number should be
quoted without re-measuring anyway.
