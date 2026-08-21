# C-39 — the force path and the feet

Ordered list, run in order. **Item 1 is the discriminator and it fails, which takes
SONIC out of the loop entirely. Item 3's fix does not change the result. Per the
decision matrix this is the PING branch, and no mass/inertia diff has been started.**

---

## 1. Static discriminator — FAILS, and that is the finding

PD-hold the nominal stance, base FREE, **no SONIC, no bridge process, no `rt/lowcmd` at
all** — nothing driving the robot but the bridge's own idle hold (`HOLD_KP`, knee 150,
ankle 40). Rig released on a timer at t=15 s, which needed a new
`G1_LL_RIG_RELEASE_AT_S` because `until_commanded` cannot fire when nothing is
commanding.

    t=15.00  base_z=+0.798  pitch=+0.000   |tau|max= 0.00   <- rig holding
    t=15.50  base_z=+0.779  pitch=+0.124   |tau|max=13.52
    t=16.00  base_z=+0.584  pitch=+0.868   |tau|max=16.83
    t=16.50  base_z=+0.158  pitch=+1.476   |tau|max=16.32
    t=17.00  base_z=+0.128  pitch=+1.549   |tau|max=11.42   <- down, 2 s after release

**It falls with SONIC not running.** SONIC is fully out of the loop, and this retro-
explains the decaying-torque signature that started this whole line: the torque decays
because a joint-space PD is holding joint targets while the BODY goes over. It was
never a controller giving up.

`sat=2/29` throughout — and those two are the **wrists**, whose 5 Nm limit is carrying a
1 kg hand. No leg joint ever reaches its clamp; peak leg torque is 16.8 Nm against
limits of 35 (ankle), 88 (hip) and 139 (knee).

## 2. Foot and ankle audit — PARTIAL, and the partial half is stated

**What the numbers actually are.** The per-foot force figures below came from the
articulation's `get_link_incoming_joint_force`, because **`get_net_contact_forces` is
ABSENT on both the wrapper and the physics view in this build** — confirmed separately
when the same call returned nothing for the fingers. So these are **ankle joint
reaction forces, not ground contact forces**, and the brief's contact POINT COUNT and
contact POSITIONS were not obtainable by this route at all. Recorded as unfinished
rather than presented as done.

    t=15.00 (rig holding)  left 0.00 N    right 0.00 N
    t=15.50 (released)     left 100.05 N  right 154.58 N   = 254.6 N
    t=16.00                left 131.18 N  right 115.43 N
    t=17.00 (down)         left 4.59 N    right 2.03 N

Zero during the hold is correct, not a fault: the rig is pinning the base, so it takes
the weight and the legs are unloaded (which is also why `|tau|max` is 0.00 there). The
254.6 N at release is the right order for the load path of a 39 kg robot.

**MuJoCo's foot, for comparison — this half IS resolved.** In `g1_29dof_old.xml` the
foot's visual mesh is `contype="0" conaffinity="0"`, i.e. **it does not collide**. All
foot contact is four 5 mm spheres:

    heel  (-0.05, ±0.025, -0.03)
    toe   (+0.12, ±0.03,  -0.03)

So the reference support polygon is **0.17 m long with the ankle at x=0**: 0.05 m of
heel behind and **0.12 m of toe ahead**.

**Unresolved:** the twin's own foot collider type/approximation and contact/rest
offsets. The physics USD is binary `usdc` and could not be read offline, and the
in-sim read was not completed inside the box. This is the one comparison the brief
asked for that is still open.

## 3. The fix candidate — implicit PhysX drives — does not change the result

Read the reference first, as instructed. `unitree_sim_isaaclab` does **not** integrate a
PD in Python: `action_provider_wh_dds.py` calls
`set_joint_position_target(full_action)` onto `ImplicitActuatorCfg` drives. Its gains
are its own config's and NOT the wire's — waist stiffness 10000, arms 300-400,
`effort_limit=1000` ("set a large torque limit") — which makes it a rigid-target teleop
sim rather than a torque-faithful one.

Implemented as `G1_LL_PD=implicit`, the faithful hybrid: implicit drives, but
stiffness/damping taken from the kp/kd **SONIC actually sends**, position target `q_d`,
velocity target `dq_d`, `tau_ff` as applied effort, and the same `URDF_EFFORT_LIMIT`
ceiling the explicit path clips to — so the A/B isolates the force PATH and not the
torque budget. The explicit path stays the default behind the same switch.

    explicit  ->  base_z=+0.123  pitch=+1.561   (falls)
    implicit  ->  base_z=+0.129  pitch=+1.544   (falls)

**Identical.** The force path is not the wall either.

---

## The arithmetic that explains all three

Measured in the twin at the held nominal stance (`[COM]` report, from the twin's own
per-link masses and transforms — a CoM location, not a mass/inertia diff):

| | |
|---|---|
| total mass | **39.005 kg** |
| CoM | `[7.8197, 2.0005, 0.7634]` |
| ankle joints x | `7.7721 / 7.7734` |
| **CoM ahead of the ankles** | **+0.054 m** |

Holding that stance therefore needs an ankle pitch torque of
`39.005 × 9.81 × 0.054 = 20.7 Nm`. A joint-space PD makes torque only out of ERROR:

| gain | error needed for 20.7 Nm |
|---|---|
| `HOLD_KP` ankle = 40 Nm/rad | **0.52 rad (30°)** |
| SONIC's wire ankle kp = 28.5 | **0.73 rad (42°)** |

**The ankle pitch joint's own travel is +0.52 rad.** The deflection required to generate
the holding torque is at or beyond the joint's range — so no joint-space PD at these
gains can hold this stance in ANY force path, which is exactly why explicit and implicit
fall identically and why removing SONIC changes nothing. And SONIC sends **`tau_ff` = 0
on every joint** (measured: 0 of 72587 samples non-zero), so there is no gravity
feedforward on the wire either.

The reference stands with the same wire gains, and its foot spheres put 0.12 m of toe
ahead of the ankle. Whether the twin's foot presents the same lever is precisely the
comparison left open in item 2.

## PING

Items 1-3 done, all negative, decision matrix says stop here. Traces:
`force/contact_com_report.txt` (CoM + per-foot joint reaction, both A/B runs),
`force/ankle_trace.csv` (ankle q/dq/tau at 1 kHz), and the explicit/implicit run logs.
**No mass/inertia diff started.**
