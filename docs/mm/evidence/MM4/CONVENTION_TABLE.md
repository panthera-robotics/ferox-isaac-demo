# MM4 (a) — field-by-field convention diff: twin bridge vs the reference MuJoCo bridge

**Reference:** `gear_sonic/utils/mujoco_sim/unitree_sdk2py_bridge.py` + `base_sim.py`, from
NVlabs/GR00T-WholeBodyControl @ 54d0b102 — the sim side of `deploy.sh sim`, i.e. the
bridge that fed W3's 17.53 m SONIC walk. Read from source; no Spark access needed.
**Under test:** `isaac/twin/lowlevel_bridge/{sim_side,dds_side}.py`.
**Third authority:** the DT bag, a real standing G1 (`docs/mm/evidence/MM3/lowstate_decode_bag.txt`).

**Result: the conventions match. C-39 is not a convention bug.** Every field SONIC's
policy actually consumes agrees with both the reference and the real robot. The three
mismatches found are all in fields SONIC only *logs*.

---

## `rt/lowstate` — `imu_state`

| field | MuJoCo reference | twin bridge | real robot (bag) | verdict |
|---|---|---|---|---|
| `quaternion` | `qpos[3:7]`, world, **w,x,y,z** (source comment: `# quaternion: w, x, y, z`) | Isaac `get_world_pose()`, world, w-first | `[0.999875, 0.01359, −0.001455, −0.007973]`, w-first near identity | **MATCH** |
| `gyroscope` | `qvel[3:6]` — MuJoCo free-joint angular velocity, **body frame** | `R_worldᵀ · ω_world` → body frame | `[−0.0021, 0.0128, 0.0032]` ≈ 0 standing | **MATCH** |
| `accelerometer` | `qacc[:3]` — world **linear acceleration**, ≈0 standing, gravity NOT included | `R_worldᵀ · [0,0,9.80665]` — **specific force**, +9.8 z upright | `[0.130, 0.064, 9.801]` — specific force | **differs from MuJoCo, matches the ROBOT** — and it is log-only (see below) |
| `rpy` | never set → zeros | computed from quaternion | populated by the robot | differs from MuJoCo, matches the robot; not consumed |
| `temperature` | never set | 80 (bag value) | 80 | cosmetic |

Twin measured standing: `quat [0.70283, −0.01451, 0.01417, 0.71107]` (a 90° **spawn yaw**;
projected gravity is invariant to yaw, so this is heading, not error), `gyro ≈ 0`,
`accel [−0.398, −0.002, 9.799]` — consistent with the measured pitch of 0.041 rad, since
`−g·sin(pitch) = −0.40`. Self-consistent and robot-shaped.

## `rt/lowstate` — `motor_state`

| field | MuJoCo reference | twin bridge | verdict |
|---|---|---|---|
| joint **order** | MJCF declaration order filtered by name | SDK order (`G1JointIndex`) | **MATCH** — independently confirmed: the reference config `g1_29dof_sonic_model12.yaml` lists `DEFAULT_DOF_ANGLES` in exactly this order |
| `q` / `dq` | `qpos`/`qvel` at `body_joint_index` | Isaac joint state, SDK-mapped by name | **MATCH** |
| `ddq` | `qacc` — **populated** | 0 | differs from MuJoCo, **matches the robot** (bag: `ddq` identically zero in all 35 998 msgs); not consumed |
| `tau_est` | `actuator_force` | `get_measured_joint_efforts()` | MATCH |
| `tick` | `int(time·1e3)` — ms | `round(sim_time·1000)` — ms | **MATCH** |
| `mode_machine` | never set → 0 | 5 (bag value) | SONIC only echoes it back into `lowcmd`; no branch |

## `rt/secondary_imu` — the torso IMU

| field | MuJoCo reference | twin bridge | verdict |
|---|---|---|---|
| source link | `mj_data.xquat[body("torso_link")]` — **torso_link**, absolute **world** orientation | pelvis world quat ⊗ waist(yaw→roll→pitch) = torso world orientation | **MATCH in intent**; the twin composes rather than measures (C-36) |
| `quaternion` | world, w-first | world, w-first | **MATCH** |
| `gyroscope` | `mj_objectVelocity(torso_link, flg_local=1)[3:6]` — **torso local frame** | `R_waistᵀ · ω_pelvis_body` | **MATCH in frame**; omits the waist joints' own rates (C-36) |
| `accelerometer` | **never set → zeros** | ≈ +9.8 z | differs — log-only |
| `rpy` | **never set → zeros** | computed | differs — log-only |

## Why the three mismatches do not explain C-39

`base_accel`, `body_torso_quat`, `body_torso_ang_vel` and `body_torso_accel` appear
**exactly once** in the deploy, at `g1_deploy_onnx_ref.cpp:2929`, and it is a call to
`state_logger_->LogFullState(...)`. They never reach the policy.

What the policy actually consumes:

* **projected gravity**, computed at `g1_deploy_onnx_ref.cpp:1622-1624` as
  `quat_rotate(quat_conjugate(base_quat), {0, 0, −1})` — from the **quaternion only**,
  not from any accelerometer. Verified w-first on both sides and against the bag.
* **base angular velocity** — body frame on both sides.
* **joint `q` / `dq`** — same order on both sides.

`rt/secondary_imu` is a **liveness requirement**, not an observation: without it the
control loop refuses to run ("LowState or IMUState is not available"), but its values
are only logged. That is why publishing it at all was the unlock, and why getting its
accelerometer "wrong" costs nothing.

## What this leaves

C-39 stays open and is **dynamic, not representational**. The reference config pins
SONIC's nominal stance at hip −0.1 / **knee 0.30** / ankle −0.2 with **all arms at 0.0**.
The twin hands over from the omni policy at knee ≈0.11 with elbows at 0.97 and a 1 kg
Dex5 hand on each — a straighter, taller, more forward-weighted robot than SONIC's
nominal. SONIC answers by crouching (observed commanding knee 0.669, more than double
its own default) and the twin goes over.

Blending the hand-over target to SONIC's own `DEFAULT_DOF_ANGLES` over 4 s was tried and
does not fix it, so it is not simply rate-of-change either.

**Next, for whoever picks this up:** the gold test is still a numerical diff against a
live MuJoCo run (`deploy.sh sim` + `scripted_walk.py`) at a matched pose — but the value
of that has dropped now that the source diff above shows the conventions agree. The
higher-value experiments are (i) hand over with the arms already at SONIC's nominal 0.0
so the CoM matches what it expects, and (ii) check whether SONIC's `reinitialize_heading_`
path copes with the twin's 90° spawn yaw.
