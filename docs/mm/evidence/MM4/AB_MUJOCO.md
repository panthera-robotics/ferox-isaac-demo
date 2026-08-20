# C-39 A/B — SONIC against the reference MuJoCo sim on this box

Same x86_64 image (`ferox/sonic-deploy:v1.1-x86_64`), same deploy binary, same driver
(`mm4_sonic_drive.py` on upstream's own builders), same DDS domain and interface (`lo`).
Only the simulator differs.

**Result: SONIC STANDS IN MUJOCO. It falls in the twin.**

```
MuJoCo, SONIC driving:   quat w = 0.9992 (upright)   knee L/R 1.401 / 0.975   hip_p -0.826
                         alive, "Replanning with mode: SLOW_WALK", executing the
                         lateral weight shift the driver asked for
Twin,   SONIC driving:   base_z 0.060, pitch -1.549 (flat)   |tau|max 6.5   sat 0/29
```

**The x86 port is exonerated.** It builds, runs, initialises its planner and balances a
robot — against the reference sim, on this box, today. Whatever C-39 is, it is in the
twin↔SONIC interaction and not in the port.

Standing the reference up needed three things worth recording: the vendored
`external_dependencies/unitree_sdk2_python` (stock sdk2py has no `OdoState_`), a
`git lfs pull` of `gear_sonic/data/robot_model/**` (the STLs were pointers, and pinocchio
fails with "Failed to determine STL storage representation"), and **no** `MUJOCO_GL` at
all — `egl` fails on this class of box and `osmesa` needs a library that is not installed,
and both make `import OpenGL` fail before the sim starts. Image: `ferox/sonic-mujoco:humble`.

## Numerical diff — 5 s each, `ab/mujoco.json` vs `ab/twin.json`

| field | MuJoCo (stands) | twin (falls) | notes |
|---|---|---|---|
| `lowstate_hz` | 197.03 | 1012.20 | 5.1× — **tested, not the cause** |
| `secondary_imu_hz` | 196.83 | 1001.19 | |
| `tick_rate_per_s` | 985.13 | 543.08 | twin's is RTF-scaled (C-29) |
| `mode_machine` | **0** | **5** | twin matches the robot; MuJoCo never sets it |
| `motor_mode` | **0** | **1** | twin matches the robot |
| `ddq_absmax` | **0.832** | **0.0** | MuJoCo populates; twin and the ROBOT leave it zero |
| `imu_accel` | **[0,0,0]** | **[0,0,9.807]** | MuJoCo sends world linear accel; twin sends specific force, like the robot |
| `imu_rpy` | **[0,0,0]** | populated | twin matches the robot |
| `imu_temp` | **0** | **80** | twin matches the bag |
| `crc` | **0** | valid | **MuJoCo does not compute a CRC at all** — which is why W3 passes `--disable-crc-check` |
| `imu_quat_wxyz` | ≈identity | 90° yaw | **tested, not the cause** |
| `sec_imu_accel` / `rpy` | zeros | populated | twin composes them (C-36) |

Every field where the two differ in *content* is one where **the twin is more
robot-faithful than the reference**. SONIC was only ever validated against the less
faithful one. That is the shape of what is left.

## Hypotheses eliminated (each cost a run)

1. **Stance transition** — no. SONIC falls from its own nominal stance.
2. **Hand mass** — no. Palms zeroed to 0.34 kg/hand matches the real-mass run to three
   figures. No payload margin exists to report.
3. **Observation conventions** — no. `CONVENTION_TABLE.md`, now confirmed empirically by
   the MuJoCo capture on every point it predicted.
4. **IDL mismatch** — no. The vendored `LowState_` is field-for-field identical to stock.
5. **A missing topic like `secondary_imu` was** — no. MuJoCo publishes `rt/odostate`; the
   C++ deploy never subscribes to it.
6. **Spawn heading** — no. Forcing the twin's held yaw to identity (`G1_LL_RIG_YAW=0`,
   quat w=1.0000) changes nothing: base_z 0.060, pitch −1.545.
7. **Publication rate** — no. Dropping the twin to the reference's 197 Hz changes
   nothing: base_z 0.060, pitch −1.545.

## The next experiment, set up but not completed inside the box

`G1_LL_MUJOCO_COMPAT=1` makes the bridge lie in exactly MuJoCo's way — `mode_machine` 0,
motor `mode` 0, accel/rpy/temperature zeroed, `crc` 0. It is a **diagnostic** and shipping
it would mean degrading the twin to match a reference bridge's omissions.

The first attempt was invalid: zeroing the CRC without also passing `--disable-crc-check`
means SONIC rejects every message, and it never commanded (`lowcmd=0`). The second
attempt ran out of the time box. **Run it with `MJC=1 SONICFLAGS=--disable-crc-check`.**
If the twin then stands, bisect the six fields; if it does not, the remaining difference
is dynamics — MuJoCo's G1 model against the twin's Isaac articulation — and the honest
next step is comparing the two models' actuator limits and inertias rather than the wire.

## Artifact hashes, for diffing against the Spark's proven W3 copies

```
fb97de22819b2057b41459802128d91723d91a25f0ad73e7bfc41a9cf8365bae  policy/sonic_v1_1/model_encoder.onnx
34bae8570d4a4421a5391a5c2befd745d4a02d182ec539e5f9da44c091c67509  policy/sonic_v1_1/model_decoder.onnx
4a67713b310932e50aca81f19188c8d76013148e98b15c8b5bbea995f12e59f0  policy/sonic_v1_1/observation_config.yaml
39b553e197f62f077975ba38512bc04781a3fc37c2af7c6756e04629f760edea  planner/target_vel/V2/planner_sonic.onnx
```

Downloaded from the PUBLIC `nvidia/GEAR-SONIC` HF repo, not from NGC. Note that the same
four artifacts drive SONIC correctly in MuJoCo on this box, which is itself evidence they
are intact — a corrupt engine or a mis-loaded normalisation would not stand a robot in
the reference either.
