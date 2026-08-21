# C-39 — observability first, then the bare body

The torque-decay signature (commanded torque falling monotonically as the robot goes
over, never saturating) is what an OPEN-LOOP controller looks like. So before any
mass/inertia work: does SONIC actually SEE the fall?

**All four tests below are conclusive, and the answer to the first three is yes.
SONIC's state is live, correctly framed, current, and it responds. The fourth removes
the hands entirely and the robot still falls.**

---

## 1. Tilt probe — the IMU wire tracks a known tilt exactly

The rig pitches the held base 0→18° over 6 s and holds. Both IMU sources must follow.

| | commanded | reported | span |
|---|---|---|---|
| `rt/lowstate.imu_state` | +18.000° | **+18.000°** | 0 → 18.000 |
| `rt/secondary_imu` | +18.000° | **+18.000°** | 0 → 18.000 |

96 samples, and the two sources agree with each other to **0.0031°**. Not frozen, not
identity, not wrong-sign, not wrong-frame, not decimated.

**Honest limit of this test:** the gyro reads zero throughout, because `_hold_base`
calls `set_angular_velocity(0)` every step while the rig is on. So this validates the
QUATERNION path only. The gyro is validated instead by test 2, which reads it during a
real fall with the rig released.

## 2. SONIC's own eyes — its logged state matches ground truth

There is no `LogFullState` in this build, but there is something better for the
purpose: `--enable-csv-logs` with `--logs-dir` makes the deploy binary write **its own
state** -- `base_quat.csv`, `base_ang_vel.csv`, `torso_quat.csv`, `torso_accel.csv`,
`q.csv`, `dq.csv`, `policy_input.csv` and more. That is not what the twin sent; it is
what SONIC believed, after its own parsing and framing.

Differenced against the sim's ground truth (`G1_LL_GT_TRACE=1`, base pose straight from
the articulation at 20 Hz) over one twin fall:

- **SONIC's own base pitch runs 0° → 89.09°. Ground truth runs 0° → 88.63°.**
- Once settled the two agree to about **0.1°**.
- `base_ang_vel` is live throughout the fall (0.53, 1.31, 3.16 rad/s through the
  transient), which also closes the gyro gap test 1 left open.

**Is it DELAYED?** That would explain everything -- a controller answering a stale
upright robot. Scanning the alignment offset for the minimum RMS error over the fall
transient gives a clean symmetric minimum at **exactly 0.000 s**:

    extra offset -0.100 s -> RMS 11.563 deg
    extra offset -0.020 s -> RMS  9.342 deg
    extra offset +0.000 s -> RMS  9.258 deg   <-- minimum
    extra offset +0.020 s -> RMS  9.378 deg
    extra offset +0.100 s -> RMS 11.586 deg

No systematic lag. The residual 9.3° RMS is what a 68°/s transient costs against a
20 Hz ground-truth trace, not a disagreement.

## 3. Closed-loop probe — SONIC's OUTPUT responds to a state change

**Deviation from the brief, stated:** the brief asked for a 50 N lateral push during a
rig-held stand. A rig-held base is KINEMATICALLY PINNED -- a force cannot move it, the
observation would not change, and lowcmd would not respond. That is a guaranteed
non-response and would have read as "open-loop confirmed" while proving nothing. The
tilt from test 1 is the same experiment with an input the rig cannot cancel: a known
state perturbation, rig still holding, SONIC live and commanding.

Base tilted 15° mid-run. `rt/lowcmd` sampled for 5 s before and 5 s during:

| joint | q_d before | q_d during | delta |
|---|---|---|---|
| `waist_pitch` | +0.9803 | +2.1347 | **+1.1544** |
| `R_hip_pitch` | -2.4317 | -3.2732 | **-0.8415** |
| `waist_roll` | +1.5782 | +2.1482 | +0.5700 |
| `L_hip_roll` | +1.3720 | +0.9382 | -0.4338 |

**max |Δq_d| = 1.1544 rad, RMS 0.3607 rad across 29 joints**, with kp/kd identical and
the message rate unchanged at 500 Hz. SONIC is closed-loop on the twin's state.

## 4. Bare-body A/B — no hands, and it still falls

Tests 1-3 all show live, correct, responsive state, so per the decision matrix: load
the bare 29-DoF robot and run the identical fork.

**Asset note, because the name is a trap.** `assets/g1_dex5/g1_29dof_trainhands.usd`
sounds like the bare model and is not -- it references `g1_dex5_1p_lockedhands` and
carries `base_link00`, so it HAS hands. The genuinely bare robot is `HAND=none` ->
`assets/g1/usd/g1.usd`, the model every pre-hand gate ran against.

Same fork, corrected rig height, trained friction, real everything else:

**Falls.** Pitch **88.89°** off `rt/lowstate` -- flat on its face. 60 s after release
the knees sit at 0.272 and the hips at -0.088 (SONIC's nominal stance) because there is
no load on the legs of a robot lying down, and the largest torque errors are on the
ARMS (11.7 Nm on shoulder pitch), not the hips.

Per the decision matrix this is the **PING** branch, not the locked-hands/inertia
branch: the hands are not the wall, because removing them entirely changes nothing.

A side observation from the same run, and it is C-36 working correctly rather than a
fault: `secondary_imu` reads 85.53° where `lowstate` reads 88.89°. The 3.39° gap is
exactly the bent waist between pelvis and torso.

---

## A correction to the turn-8 finding

Turn 8 reported that `HOLD_KP` "never reaches the stance it is given" -- hip settling at
-0.522 against a target of -0.100, waist driven to its ±0.52 stop. That observation was
real but the diagnosis was wrong, and the fix was wrong with it.

With `G1_LL_RIG_LIFT_M=0` the same torque hold reaches the stance exactly:
`knee_L=+0.300 hip_p_L=-0.100` with `|tau|max` of **0.01-0.08 Nm**. The sag was not the
gain. **The rig was holding the pelvis 6.65 cm too low** (`G1_LL_RIG_LIFT_M=-0.0665`),
so the legs had to fold to keep the feet on the floor, and the waist bent to its stop
taking up the rest. `G1_LL_HOLD_KINEMATIC` was built to force what was already correct
at the right rig height; it is kept as a diagnostic but is not needed.

## Where this leaves C-39

Sixteen hypotheses dead. The message is clear, the state SONIC reads is clear, the
control loop is closed, and the hands are not the wall. The remaining difference is the
twin's 29-DoF BODY against the reference's -- inertia, collision geometry at the feet,
actuator model -- and the bare-body result is what says so, because it holds everything
else constant and removes the one large mass difference that was left.

**PINGING per the decision matrix. Traces:** `obs/tilt_probe.json`,
`obs/bare_stand_check.json`, `obs/lowcmd_before_tilt.json`, `obs/lowcmd_during_tilt.json`,
SONIC's own CSV state logs, and the `[GT]` ground-truth trace.
