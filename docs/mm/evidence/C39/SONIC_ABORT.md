# C-39 — SONIC does not "fail to balance" the Dex5 twin. It ABORTS, before the rig lets go.

Found 2026-08-22 while running the task-1 A/B. Recorded on its own because it changes
what several earlier rows in this file were measuring.

## What happens

With `HAND=dex5_1p`, the rig holding the base, and SONIC commanding normally
(`G1 type: 5`, `Planner initialized successfully!`, `Init Done`, lowcmd at ~167 Hz
average, `crc_bad=0`), SONIC stops itself:

```
Loop timing - LowState age: 25.731ms, ... | HandCloseRatio: 1
✗ Error: body_dq[24] = 35.367 > 35.
✗ Error: Failed to gather robot state to logger in the middle of the control loop!
Stopping control system.
[DEBUG] Stopping G1Deploy...
```

Measured directly out of the shared-memory command record afterwards:

    cmd_count=46162  stamp_age=45308 ms and rising   kp[0..3]=[0 0 0 0]  q_d[0..3]=[0 0 0 0]

— the command channel is **frozen**, its last content all zeros. The bridge is doing
exactly the right thing with that (`mode=hold`, its own idle stance), and the rig's
auto-release needs sustained authority, so:

    rig released at: NEVER
    RESULT: INVALID (the rig never released, so the body was never free)

**The robot never became free-standing. Nothing about balance was tested.**

## Which joint, exactly

The index is printed in the deploy's *mujoco* order, not SDK order, and the two differ.
From `policy_parameters.hpp`:

    mujoco_to_isaaclab[24] = 26        # body_dq[i] = unitree_joint_state[mujoco_to_isaaclab[i]].dq()

SDK index 26 is **`right_wrist_roll_joint`**. That is the joint this campaign has
already recorded as saturating: *"sat=2/29 the whole time and those two are the WRISTS
(5 Nm limit carrying a 1 kg hand)"* (`RESULTS_MM4`, force-path row). The Dex5 hand hangs
a ~1 kg mass off a wrist whose URDF effort limit is 5 Nm; the wrist saturates, rings,
and crosses 35 rad/s — and SONIC's guard then takes the whole controller down.

## The guard, from upstream source

`gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp:2832`

```cpp
body_dq[i] = unitree_joint_state[mujoco_to_isaaclab[i]].dq(); // URDF order
if (body_dq[i] > 35 && !disable_crc_check_) {
  std::cout << "✗ Error: body_dq[" << i << "] = " << body_dq[i] << " > 35." << std::endl;
  return false;
}
```

Three properties worth carrying, none of them obvious:

1. **It is one-sided.** `body_dq[i] > 35`, not `|body_dq[i]| > 35`. A joint at
   **−40 rad/s passes**. So the guard fires on the sign of the ringing, which makes it
   look intermittent when it is not.
2. **`--disable-crc-check` also disables it.** The same flag gates both. Any earlier run
   in this campaign that carried `SONICFLAGS=--disable-crc-check` had this guard OFF —
   which is why those runs reached the rig release and this one did not. **The flag is
   not the no-op its name implies.**
3. It aborts the **whole control system**, not the offending joint.

## What this retro-explains, and what it does not

It does **not** overturn the C-39 A/B result — the reference MuJoCo sim's stand
(`evidence/MM4/ab/mujoco.json`, quat w = 0.999753) and the twin's falls are still what
they were. What it does is add a failure mode nobody had separated out: on the Dex5
twin, *with the guard active*, SONIC is not a balancer losing a fight, it is a
controller that has already stopped. Any run that showed "SONIC fell" needs its SONIC
log checked for this line before the fall is attributed to balance at all.

## Consequence for the A/B

The reference model `g1_29dof_old.xml` **has no hands**. Comparing it against the Dex5
twin would differ in two things at once. The twin side of the A/B is therefore
`HAND=none` (`twin_bare`) — the same 29-DoF body the reference is, which is also the
side that does not trip this guard. The Dex5 run is kept as
`evidence/C39/ab/twin_dex5_abort_*` because the abort is itself a finding.

**Open, and cheap to settle later:** raise the wrist effort limit, or couple the Dex5
mass properly, and see whether the Dex5 twin then survives the guard. That is an asset
question, not a solver one, and it is not on the critical path for C-39.
