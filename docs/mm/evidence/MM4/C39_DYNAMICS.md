# C-39 — the lie-bisect and the dynamics diff

One-day box. Order as set: the MuJoCo-compat lie first, then the dynamics list.
Every run below is SONIC alone, released from a rig, no handoff, real hand mass,
against the same x86 deploy image and the same DDS seam that stands the reference
MuJoCo sim on this box.

**Nothing here stands SONIC. But the wire is now fully exonerated, one real defect
in the experiment itself was found and fixed, and the search has moved from "what
does the twin say" to "what does the twin's body do".**

---

## 1a. The lie-bisect — NEGATIVE, and it clears the wire completely

`MJC=1 SONICFLAGS=--disable-crc-check`, with the compat path completed first.

The turn-7 attempt at this was invalid twice over, and both faults were mine:
the runner script had been mangled by a `sed` edit into
`... >/dev/null 2>--input-type zmq_manager ... 2>&11`, which fed the deploy binary
duplicate arguments and redirected its stderr into a file named `--input-type`; and
the CRC was zeroed without `--disable-crc-check`, so SONIC rejected every message
and never commanded (`lowcmd=0`). Both are fixed. This run has `lowcmd` flowing at
**366-373 Hz with `crc_bad=0`**, so it is the first valid test of the hypothesis.

The compat path was also INCOMPLETE: the A/B diff flagged `ddq` (MuJoCo 0.832,
twin 0.0) and the compat flag did not populate it, leaving a hole in exactly one of
the six fields under test. `ddq` is now finite-differenced from `dq` under the flag.
The full lie is: `mode_machine` 0, motor `mode` 0, `accelerometer` zeroed, `rpy`
zeroed, `imu` temperature 0, `ddq` populated, `crc` 0.

**Result: the twin still falls.** `base_z 0.060, pitch -1.545`, identical to the
un-lied run to three decimals. **Every field the A/B diff identified is now
eliminated as a cause.** There is no remaining observable difference between what
the twin puts on the wire and what the reference puts on the wire that SONIC could
be reacting to. C-39 is not in the message.

---

## 2. The torque trace — the finding that redirected the whole search

Re-running the fork at 0.25 s report resolution instead of 2.5 s shows the fall in
full, and it does not look like anything that had been assumed:

| t (s) | mode | base_z | pitch | \|tau\|max | sat |
|---|---|---|---|---|---|
| 24.75 | hold | 0.731 | +0.000 | **42.20** | 0/29 |
| 25.00 | cmd  | 0.737 | -0.090 | **31.60** | 0/29 |
| 25.25 | cmd  | 0.729 | -0.250 | **23.49** | 0/29 |
| 25.50 | cmd  | 0.685 | -0.441 | **21.29** | 0/29 |
| 25.75 | cmd  | 0.503 | -0.739 | **20.16** | 0/29 |
| 26.00 | cmd  | 0.092 | -1.428 | **16.09** | 0/29 |
| 27.50 | cmd  | 0.061 | -1.545 | **7.77**  | 0/29 |

The commanded torque **falls monotonically as the robot goes over**, and `sat=0/29`
throughout -- nothing is being clipped. A balancer losing a fight commands MORE
torque, not less. SONIC never fights at all: it is under-torqued from the first
control interval and the robot is already pitching within 250 ms of getting
authority. This is what sent the search to the dynamics rather than to the message.

---

## 3. The dynamics list, in the order set

### (1) Foot-ground friction — the fix existed and was NOT being used

MM1 measured the twin at the PhysX default (0.5 both surfaces) against a training
value of 1.0, and `_apply_training_contact_material` was written to correct it --
**but it is gated behind `TWIN_CONTACT_MATERIAL=1`, which no C-39 fork ever set.**
Every run of this investigation before this one, on both branches of the turn-6 fork
and both sides of the turn-7 A/B, ran at half the trained friction.

The reference confirms 1.0 is the right target: `scene_29dof_mujoco.xml` gives the
floor geom no explicit `friction`, so it takes MuJoCo's default of **1.0**.

Bound and read back on both surfaces:
`contact material static=1.0 dynamic=1.0 restitution=0.0 combine=multiply -> ['/World/G1', '/World/Env']`

**Result: still falls.** The fall changes character slightly -- it now rolls as well
as pitches (roll -0.587 at t=25.75 where it had been 0.000) -- but it falls. Friction
was a real defect in the experimental setup and it is now fixed; it is not C-39.

### (2) Joint armature, damping and dry friction

Read out of the reference model `g1_29dof_old.xml` rather than assumed:

| | reference (MuJoCo) | twin |
|---|---|---|
| armature | 0.01 all joints | 0.01 (`TWIN_ARMATURE`) — matches |
| damping | 0.05 all joints | from `deploy.yaml` |
| frictionloss | **0.2** (0.1 wrists) | **none — the URDF importer wrote no dry friction at all** |

Dry joint friction had no knob in the twin, so one was added
(`TWIN_JOINT_FRICTION`, applied to every revolute joint and read back, the same
shape as the armature path). It cost two runs to get right, both my fault: the
first raised inside stage setup and took Isaac down at 12.5 s with no traceback,
and the second was guarded with a 1e-9 read-back tolerance against a float32 store
(0.2 reads back as 0.20000000298023224), so a CORRECT write was rejected and the
run silently proceeded at zero friction. A read-back guard that rejects good writes
is worse than no guard; it is now a float32 epsilon and the failure path warns
loudly instead of exiting.

Applied and read back on **69 joints** at the reference's 0.2 Nm, with the exact
release stance from section 4 also in force: **still falls**
(`base_z 0.097, pitch +1.482, roll -3.033`). Armature already matched. Item (2) is
closed and negative.

### (3) Physics rate — the assumption was inverted

The brief expected the twin at 500 Hz against MuJoCo at 1 kHz. Both halves are
wrong. The twin runs physics at **1000 Hz** (`G1_PHYSICS_HZ`, restored when 500 Hz
was found to alias three sensor rates). The reference runs at **200 Hz**:
`sim_frequency: int = 200` in `mujoco_sim/configs.py`, and `SIMULATE_DT: 0.005` in
the SONIC WBC yaml. The reference is FIVE TIMES COARSER than the twin, not twice as
fine, which also explains its 197 Hz lowstate rate directly.

Run at the reference's actual 200 Hz: **still falls**, same signature.

### (4) PD parity — the formulas agree, and the twin applies what it computes

Side by side:

    MuJoCo  base_sim.compute_body_torques:
        tau = tau_ff + kp * (q_des - q) + kd * (dq_des - dq)
    twin    sim_side._apply_pd:
        tau = kp * (q_d - q) + kd * (dq_d - dq) + tau_ff,  clipped to URDF effort

Identical. The twin does not clamp gains, and `sat=0/29` says the effort clip is
never reached. Both sims read kp/kd off the wire -- the reference's own `MOTOR_KP`
/`MOTOR_KD` yaml block is loaded into a legacy container that the SONIC sim path
does not use for torque -- so both are handed exactly the same numbers.

What SONIC actually sends (`mm4_lowcmd_dump.py`, 2503 messages at 500.6 Hz):

    kp  99.098 (hip p/r, knee) | 40.179 (hip yaw, waist yaw) | 28.501 (ankles, waist r/p)
        14.251 (arms) | 16.778 (wrist p/y)
    kd  6.309 | 2.558 | 1.814 | 0.907 | 1.068
    dq_d  all zero      tau_ff  all zero, 0 of 72587 nonzero

Pure position PD, no feedforward. Knee targets 1.05/0.65 rad -- a deep crouch, and
the same crouch the reference sim visibly tracks (its knees sat at 1.401/0.975).
Only 1 of 29 mean targets is outside its joint limit, and only just
(`L_shoulder_roll` -1.6039 against a -1.5882 stop).

A first pass comparing WINDOW MEANS of `rt/lowcmd` against `rt/lowstate` appeared to
show the twin computing 80.4 Nm on `L_hip_pitch` and applying 6.5. That was worth
checking directly rather than believing, so an in-PD probe was added that prints
kp, q_d, q and tau from the record the PD actually uses. It shows `kp*e` and `tau`
agreeing to three significant figures on every joint. **There is no missing torque.
The mean-of-window comparison was the wrong instrument and its conclusion was
wrong.** PD parity holds.

### (5) Hand mass

Covered by the turn-6 fork: palm masses zeroed to 0.34 kg/hand reproduced the fall
to three figures, so there is no payload margin to bisect. Not repeated.

---

## 4. The defect the probe found: every fork so far had the wrong initial condition

The same probe, one interval before the rig releases, printing what SONIC asks for
against what the twin is actually holding:

| joint | SONIC commands | twin holds | error |
|---|---|---|---|
| `L_hip_pitch` | -0.100 | **-0.5224** | 0.42 rad |
| `R_hip_pitch` | 0.000 | **-0.4119** | 0.41 rad |
| `waist_roll` | 0.000 | **+0.5200** | at its ±0.52 **hard stop** |
| `waist_pitch` | 0.000 | **+0.5200** | at its ±0.52 **hard stop** |
| `L_ankle_pitch` | -0.200 | **+0.2401** | 0.44 rad |

`G1_LL_HOLD_POSE=sonic` sets `q_hold = SONIC_DEFAULT_Q` (hip -0.1, knee 0.3, ankle
-0.2) but `HOLD_KP` is a TORQUE hold, and it never reaches that stance -- the rig
pins the pelvis at 0.731 m, the legs must fold to keep the feet on the floor, and
the stance settles 0.4 rad away at every major joint with the waist jammed against
its mechanical stop.

**So the turn-6 diagnostic fork -- "SONIC from ITS OWN nominal stance" -- never
started from SONIC's nominal stance, and neither did either side of the turn-7 A/B.**
The initial condition was ~0.4 rad off at both hips throughout. That is a real
defect in the experiment, not in the twin, and it is the same class of mistake as
the mangled runner and the CRC: a control that was believed rather than measured.

**Fixed and re-run.** The first fix attempt used `apply_action(joint_positions=...)`
and moved the stance by 0.0000 rad -- in effort control mode with the drive gains
zeroed a position TARGET is ignored, so that was a no-op dressed up as an
experiment. A state write (`set_joint_positions`) plus the rig at spawn height does
place the joints. The probe then reads, one interval before release:

    sdk[ 7] kp=99.098 q_d=+0.0000 q=+0.0000 dq=+0.000 -> kp*e=-0.00 tau=-0.00
    sdk[ 1] kp=99.098 q_d=+0.0000 q=+0.0000 dq=+0.000 -> kp*e=-0.00 tau=-0.00
    ... |tau|max = 0.00 across all 29

SONIC is now handed a robot standing in exactly the pose it is asking for, with zero
torque error, and released.

**Result: still falls** -- `base_z 0.115, pitch +1.533, roll +2.784`, backwards and
over rather than forwards. A clean negative on a hypothesis that had never actually
been tested.

---

## 5. Where C-39 stands

Eliminated this pass, each costing a run: the complete MuJoCo wire lie (all six
fields including `ddq`), foot-ground friction, joint dry friction, physics rate,
PD formula and gain handling, and the release stance. Added to the seven already dead from turns 6-7
(stance-as-believed, mass, conventions, IDL, missing topic, spawn heading,
publication rate).

What is left is genuinely the BODY, not the message and not the controller's
interface to it: the twin's Isaac articulation against the reference's MuJoCo model
-- link inertias, collision geometry at the feet, joint limits and the actuator
model. The one measured asymmetry already in hand is that the reference gives every
joint 0.2 Nm of dry friction that the twin has none of, and the twin is a 69-DoF
robot with Dex5 hands where the reference `g1_29dof_old.xml` is a bare 29-DoF arm-
and-leg model with no hands at all.

The honest next step is a mass/inertia diff -- total mass, per-link mass and the CoM
of each of the two models, twin USD against `g1_29dof_old.xml` -- rather than another
control experiment. That is a comparison of two robots, and it should be done as
one, not one field at a time.

## Run table

| # | change under test | result |
|---|---|---|
| 1a | full MuJoCo wire lie + `--disable-crc-check` | falls, `base_z 0.060 pitch -1.545` |
| 1b(1) | contact material 1.0/1.0 both surfaces | falls, `base_z 0.061 pitch -1.538` |
| 1b(3) | physics at the reference's real 200 Hz | falls, `base_z 0.060 pitch -1.549` |
| 1b(4) | PD parity | no defect: formulas agree, `kp*e == tau` |
| 1b(6) | kinematic hold via `apply_action` | **invalid** — no-op in effort mode |
| 1b(6b) | release from the EXACT nominal stance | falls, `base_z 0.115 pitch +1.533 roll +2.784` |
| 1b(2) | joint dry friction 0.2 on 69 joints, exact stance | falls, `base_z 0.097 pitch +1.482 roll -3.033` |
