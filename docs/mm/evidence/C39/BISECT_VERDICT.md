# C-39 task 1, "Falls" branch — the whole list is negative. **SONIC is parked.**

2026-08-22, nine runs, sole occupancy asserted before every one (0 `SECOND_INSTANCE`
rows). Each row is the identical fork with **one** property changed.

| label | property | released | base_z | pitch | verdict |
|---|---|---|---|---|---|
| `baseline_twin` | — (control) | 9.88 | +0.107 | +88.3° | FALLS |
| **`baseline_ref`** | **reference body, unmodified** | 11.58 | +0.098 | **+86.3°** | **FALLS** |
| `solver_iters` | `G1_SOLVER_ITERS=64,64` | 9.18 | +0.108 | +88.4° | FALLS |
| `friction_mult` | `friction_combine=multiply` | 11.98 | +0.107 | +88.3° | FALLS |
| `contact_off` | `contact_offset=0.002, rest_offset=0.0` | 11.96 | +0.107 | +88.3° | FALLS |
| `depen_vel` | `max_depen_vel=1.0` | 15.66 | +0.107 | +88.4° | FALLS |
| `self_coll` | `self_collision=1` | 11.40 | +0.107 | +88.4° | FALLS |
| `dt_200hz` | `G1_PHYSICS_HZ=200` (the reference's rate) | 27.57 | +0.067 | −86.6° | FALLS |
| `implicit_drive` | `G1_LL_PD=implicit` | 11.04 | +0.095 | +85.5° | FALLS |

`sha256.txt` fixes the harness these rows were taken with.

## The two things this settles

**1. The asset is exonerated — confirmed, not provisional.** `baseline_ref` is the
reference MuJoCo body, imported unmodified, falling in our simulator at +86.3° against
our own body's +88.3°. The same body stands under the same deploy binary in the
reference MuJoCo sim (`evidence/MM4/ab/mujoco.json`, quat w = 0.999753). So the twin's
mass distribution, its torso, its hands and its sensor masses are **not** C-39, and the
mass-bisect the brief queued behind this experiment is moot.

**2. It is not any single PhysX property on the brief's list.** Seven candidates,
including the reference's own 200 Hz rate and Isaac's implicit drives, all fall — and
they fall to *the same place*: base ~0.10 m, pitch ~87°, within a few seconds of
release. The uniformity is the finding. A solver parameter that mattered would move the
number; none of these moves it at all.

Note `dt_200hz` is the one row that looks different (release at 27.57 s, pitch −86.6°)
and it is not a near-miss: at 200 Hz our explicit PD is in the regime `run.py` itself
flags as C-35-unstable, so it took longer to get authority and then fell the other way.

## Disposition, per the brief

> *"If our real-robot body simply won't balance under SONIC, report the margin, park
> SONIC (finetune becomes a Spark item), and go to 3."*

**SONIC is parked.** The margin, stated plainly: it is not a margin at all — the robot
does not partially hold and then lose it, it goes from a released stance to face-down in
about 2.5 s, and it does so identically whichever of the nine configurations it runs in
and whichever of the two bodies it is.

What is genuinely still open, and is now the *whole* remaining C-39 question, is the
difference between our Isaac harness and the MuJoCo one **taken together** rather than
one property at a time: the actuator model (MuJoCo's own actuators against our explicit
PD over a DDS seam), the contact solver family, and the fact that MuJoCo integrates the
whole robot at 200 Hz with its own constraint formulation. That is a larger piece of
work than a parameter sweep and it does not belong in front of the manipulation gates.

**Next per the brief: grasp v7 (task 3), then MM5 with omni-hold, camera backlog, montage.**
