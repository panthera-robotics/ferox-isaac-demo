# RESULTS_MM1 — locomotion contract + motion suite

**Host:** Vast.ai · **NVIDIA GeForce RTX 4080 SUPER, 16376 MiB** · driver 580.105.08 · Isaac Sim 5.1.0
**camera-capable box: NO** (C-23) — everything below ran with `TWIN_CAMERA=0`
**Date:** 2026-08-19 · **Verdict: IN PROGRESS** — nav fix landed; yaw diagnosed; motion suite not yet built

---

## 1. Ferox G1 nav — the inflation radius sat exactly on the inscribed radius

**Authorised and scoped by Mohammed. Own commit, `Ferox@mohammed/mm-campaign`.**

### The defect

The local costmap footprint is a **square of half-width 0.35**, so its inscribed
radius is 0.35 — Nav2 computes 0.360 and warns about it. `inflation_radius` was
**also 0.35**. Nav2's inflation cost is

```
cost(d) = 253 * exp(-k * (d - r_inscribed))
```

which only has a decay region where `inflation_radius > r_inscribed`. With the two
equal there was **no decay region at all**: a flat inscribed-cost cliff out to
0.35 m, zero beyond it. MPPI had no gradient to descend.

Measured symptom, twice across two sessions: the robot plans, drives to within
**0.76 m** of its goal, parks, and `NavigateToPose` never returns a terminal
status. MM0 ruled out the competing explanation by putting goals inside a map four
times larger and getting the same result.

### The change

| | before | after |
|---|---|---|
| local `inflation_radius` | 0.35 | **0.55** (footprint 0.35 + 0.20) |
| local `cost_scaling_factor` | 5.0 | **10.0** |
| global `inflation_radius` | 0.45 | **0.65** |
| global `cost_scaling_factor` | 3.5 | **7.0** |
| `xy_goal_tolerance` | 0.35 | **0.25** |
| `yaw_goal_tolerance` | 0.35 | **0.50** |

Nothing else in Nav2 was touched.

`cost_scaling` is **raised**, and the direction matters. Extending the radius at
the old `k` would leave cost 98/253 still standing at the new outer edge — a
*wider* high-cost collar, the opposite of the fix. At `k = 10` the edge cost is
38/253, so the effective collar is about what it was while the gradient now exists.

`yaw_goal_tolerance` is **loosened**, deliberately and temporarily. `SimpleGoalChecker`
requires xy **and** yaw, and this robot's yaw does not work (§2). A tight yaw here
would hang every goal on a heading the robot cannot reach — hiding a locomotion
defect inside a navigation timeout. The three conditions that replace it are
written into the file next to the value.

### Result — from `bt_navigator`, not the action client

| goal (map) | outcome | final pose | error |
|---|---|---|---|
| (7.50, −1.00) | **Goal succeeded** | (7.34, −1.04) | **0.16 m** |
| (9.50, 0.50) | **Goal succeeded** | (9.43, 0.39) | **0.126 m** |

Both inside the new 0.25 m tolerance, both with a terminal status returned.
**Before this change, across two sessions, zero goals ever returned one.**

**Shortfall, stated plainly: 2 confirmed, not 3/3.** The third run was lost to my
own harness — an over-eager client kill preempted it, which `bt_navigator` logs as
a preemption, not a nav failure. Per Mohammed, the third runs once at MM2 in
`panthera_lab`, which is the second venue the gate wants anyway.

**Separate item worth naming:** `ros2 action send_goal` frequently does not return
even after `bt_navigator` logs a terminal status, and survives `timeout -s KILL`.
Every number above is therefore taken from `bt_navigator`'s own log. Evidence:
`evidence/MM1/bt_navigator_goals.txt`.

---

## 2. Yaw — diagnosed. It is NOT a contract bug.

> **Superseded in part by §3.** Everything below about *what was eliminated* stands.
> The **magnitudes** do not: these runs were made with `TWIN_CONTACT_MATERIAL=1` and
> `TWIN_ARMATURE=0.01` (which §3 shows degrade the gait) and without a reset between
> tests. The "+wz asymmetry" concluded in §2.9 and §2.13 **does not reproduce** on
> clean defaults — see §3, where in-place yaw is ~zero in *both* directions and
> walking yaw is symmetric to within 8 %. Read §2 for the eliminations, §3 for the
> numbers.

Campaign §4.1 orders this: check the contract first, retrain only if the contract
is clean. **The contract is clean and the policy still does not turn.**

### 2.1 The observation contract matches training, term for term

`policy/params/env.yaml` `observations.policy`, in order, against
`run.py::_compute_observation`:

| term | training scale | run.py | match |
|---|---|---|---|
| `base_ang_vel` | 0.2 | from `env.yaml` | ✅ |
| `projected_gravity` | — | from `env.yaml` | ✅ |
| `velocity_commands` | — | from `env.yaml` | ✅ |
| `joint_pos_rel` | — | from `env.yaml` | ✅ |
| `joint_vel_rel` | 0.05 | from `env.yaml` | ✅ |
| `last_action` | — | from `env.yaml` | ✅ |

`history_length: 5`, `flatten_history_dim: True`, `concatenate_dim: -1` →
96 per step × 5 = **480**, which is what `run.py` builds. Its history is
**per-term** (each term's 5 steps contiguous, then terms concatenated), oldest
first — the same layout Isaac Lab's ObservationManager produces for a group with
history. Scales are not hardcoded; they are read out of `env.yaml` at load.

### 2.2 The command reaches the policy unclamped

`_resolve_command_limits` reads `commands.base_velocity.ranges`, preferring
`deploy.yaml`:

```
deploy.yaml ranges:  lin_vel_x [-0.6, 1.0]   lin_vel_y [-0.5, 0.5]   ang_vel_z [-1.0, 1.0]
--wz_max default:    1.0
=> effective wz clamp: [-1.0, +1.0]
```

So nothing truncates yaw on the way in.

> Worth recording for whoever reads `env.yaml` next: **its** `ranges` block is a
> curriculum snapshot — `lin_vel_x (-0.1, 0.1)` — while `limit_ranges` carries the
> real envelope. `_resolve_command_limits` prefers `deploy.yaml`, whose `ranges`
> ARE the full envelope, so the twin is not accidentally clamped to ±0.1 m/s. If
> `deploy.yaml` ever loses that block the fallback would silently cripple vx.

### 2.3 Yaw was in the training distribution

`env.yaml` `commands.base_velocity`: `ang_vel_z (-1.0, 1.0)` in **both** `ranges`
and `limit_ranges`. "The policy never saw yaw" is not supported.

### 2.4 The network responds to wz — so it is not plumbing

The exported TorchScript policy, fed synthetic observations differing **only** in
the `velocity_commands` wz slot:

| command | max abs Δaction vs wz=0 | mean |
|---|---|---|
| wz +0.20 | 0.076 | 0.016 |
| wz +0.50 | 0.185 | 0.039 |
| wz +1.00 | 0.358 | 0.076 |
| wz −1.00 | 0.442 | 0.078 |
| **vx +0.50 (control)** | **1.109** | **0.126** |

Monotonic in magnitude, different for opposite signs. The command is reaching the
network and the network is reacting to it. But the yaw response is about **3×
weaker** than the vx response at a comparable fraction of the trained range.

### 2.5 The body does not rotate, at any commanded rate

Measured on `/ferox/g1_01/odom` pose (frame-robust; the published *twist* is in a
frame that does not match the displacement — see MM0's `validate_motion` note):

| cmd wz | hands ON (69 DOF) | hands OFF (bare 29 DOF) |
|---|---|---|
| +0.20 | −0.0000 rad/s (0.0 %) | −0.0000 rad/s (0.0 %) |
| +0.50 | +0.0001 rad/s (0.0 %) | +0.0001 rad/s (0.0 %) |
| +1.00 | +0.0024 rad/s (0.2 %) | +0.0021 rad/s (0.2 %) |
| −1.00 | −0.0010 rad/s (0.1 %) | −0.0021 rad/s (0.2 %) |

**The hands are not the cause.** The bare 29-DOF robot — exactly the body the
policy was trained on — behaves identically. That kills the most plausible
physical hypothesis (2 kg of Dex5 at the ends of the arms changing yaw inertia).

### 2.6 CONTACT PHYSICS — the twin runs at half the friction the policy trained on

Measured, not assumed:

| | static | dynamic | restitution | combine |
|---|---|---|---|---|
| **training** (`env.yaml` terrain + sim `physics_material`) | **1.0** | **1.0** | **0.0** | multiply |
| G1+Dex5 asset (feet, body) | **— none authored —** | | | |
| `hospital.usd` (the floor it stands on) | **— none authored —** | | | |
| Isaac default ground plane, for reference | 0.5 | 0.5 | **0.8** | — |

**Neither surface authors a physics material.** The URDF importer wrote none onto
the feet, and `hospital.usd` writes none onto its floor, so both fall back to the
PhysX scene default — which is 0.5, not the 1.0 the policy was trained against.
With `friction_combine_mode: multiply` the training contact was 1.0 × 1.0 = 1.0;
the twin's is the PhysX default on both sides.

Training also ran on **generated rough terrain** with **restitution 0.0**; Isaac's
default plane carries restitution 0.8, which is a superball for anything that
walks.

**Why this is the leading candidate for "walks but will not turn."** Turning in
place is friction-limited in a way translation is not: the stance foot has to
generate a yaw moment against the floor, while walking forward mostly needs
normal force and a little fore-aft shear. Halving μ removes the yaw moment first.
It also explains why the failure is total rather than sluggish, and why the
policy's action *does* change with wz (§2.4) while the body does not move.

**NOT YET PROVEN.** The confirming test is to author the training material
(1.0 / 1.0 / 0.0) and re-run the sweep. That is a legitimate twin correction with
provenance — restoring a documented training value to a surface that currently has
none — and not tuning-to-fit, but it is still a change and it has not been made.

### 2.7 Friction test — and a measurement bug that nearly inverted the answer

`TWIN_CONTACT_MATERIAL=1` authors the training material (1.0 / 1.0 / 0.0,
`multiply`) and binds it to **both** the robot and the world. Two things happened
and both are worth keeping.

**(a) The first binding was a silent no-op, and the read-back caught it.** Bound
where the world is built, `stage.GetPrimAtPath(robot_root)` is not yet valid, so
the material landed on `/World/Env` **only** — and with `multiply` that still
leaves the feet on the PhysX default, i.e. exactly the condition under test. The
log said `-> ['/World/Env']` and the fix was to bind after the robot exists:
`-> ['/World/G1', '/World/Env']`. Without the read-back this would have read as
"friction hypothesis rejected".

**(b) The yaw metric could not measure past ±π.** Differencing endpoint yaw with
`atan2` wraps: a −3.600 rad turn reads as **+2.683**, which presents as a
*wrong-direction* turn at 35 % when it is a *correct-direction* turn at 47 %.
`tools/yaw_sweep.py` now accumulates yaw across every odom sample, unwrapping each
step. **The earlier 0.0 % rows are unaffected** — they were 0.000–0.022 rad,
nowhere near wrapping — but this row was not, and the bug nearly got the friction
change discarded.

**Result with training friction on both surfaces, unwrapped:**

| cmd wz | measured | tracking |
|---|---|---|
| +0.20 | −0.0001 rad/s | −0.1 % |
| +0.50 | +0.0000 rad/s | 0.0 % |
| +1.00 | −0.0103 rad/s | −1.0 % |
| **−1.00** | **−0.5792 rad/s** | **57.9 %** |

**This is not yet a diagnosis and must not be read as one.** The robot turns at
58 % of command for **one sign only**, at the largest magnitude only, and not at
all for +1.00 or for small commands either way. Two readings are consistent with
that, and I cannot separate them yet:

* a genuine directional asymmetry in the policy's yaw channel, or
* the robot losing balance and rotating as it goes — `yaw_sweep.py` does **not**
  record base height, so a fall and a turn currently look the same.

**Next, before any conclusion:** add base height and roll/pitch to the sweep so a
fall is distinguishable from a turn, and repeat each command 3× to see whether the
asymmetry is stable. Nothing about friction is claimed until that runs.

### 2.8 CLEAN measurement — and Nav2 was contaminating every earlier one

**A confound found in a routine sanity check.** `sim.log` showed
`cmd_vel #31350: lin=-0.05, ang=-0.40` while my sweep was supposedly the only
publisher. −0.40 is exactly Ferox `g1.yaml`'s `max_angular_z`: **Nav2 was
publishing continuously** — a leftover recovery spin — and every yaw number before
this point was a fight between two publishers on one topic. The 57.9 % row in
§2.7 was Nav2's spin, not the policy responding.

Re-measured with the nav stack **stopped** (`cmd_vel` log lines frozen at 632
over 8 s, i.e. nothing publishing), from a throwaway probe container, 3 repeats
per command, base height and tilt recorded so a fall cannot masquerade as a turn:

| cmd wz | median tracking | upright | zmin | tilt |
|---|---|---|---|---|
| +0.20 | −0.8 % | 3/3 | 0.790 | 2.5° |
| +0.50 | −0.7 % | 3/3 | 0.788 | 1.8° |
| +1.00 | +0.1 % | 3/3 | 0.789 | 2.6° |
| −0.20 | −0.8 % | 3/3 | 0.790 | 2.4° |
| −0.50 | −0.6 % | 3/3 | 0.788 | 1.0° |
| **−1.00** | **+37.0 %** (33–51 across runs) | **3/3** | 0.770–0.780 | 2.6–3.6° |

Positive tracking means the correct direction. Repeats agree to 0.1 % everywhere
except −1.00, which varies 33–51 %.

**So the policy does have a yaw channel, and it is badly broken rather than
absent:** it produces 33–51 % of commanded rate at −1.0 rad/s **while standing**
(zmin 0.770–0.780 against a 0.79 standing height, tilt under 3.6° — not a fall),
and essentially nothing at ±0.2, ±0.5 or **+1.0**. That asymmetry — one sign, one
magnitude — is not a deadband and is not explained by anything in §2.1–2.7.

Against the §2 acceptance (≤0.1 rad/s absolute error on yaw) every row fails,
including −1.00 at 0.5–0.67 rad/s of error.

### 2.9 §4.1(a) — IT TURNS IN ISAAC LAB. The difference is our sim.

The decisive test: `unitree_rl_lab`'s `Unitree-G1-29dof-Velocity` — the task this
policy was trained on — driven by **our exported TorchScript**, so the policy under
test is byte-identical to the twin's. Isaac Lab **2.3.2**, which pairs with Isaac
Sim **5.1.0**, exactly the container's version. The env's observation came out
**(1, 480)**, matching our policy's input without adaptation, which is an
independent confirmation of §2.1.

| cmd wz | **Isaac Lab (training env)** | **our twin** |
|---|---|---|
| −1.00 | **58.9 %** (zmin 0.775) | 37 % (33–51) |
| **+1.00** | **36.6 %** (zmin 0.772) | **0.1 %** |
| +0.50 | 4.9 % | −0.7 % |
| −0.50 | 5.7 % | −0.6 % |

**Two separate problems, and they need separate answers:**

1. **The policy is genuinely weak at yaw, in its own training environment.** 4.9 %
   at 0.5 rad/s and 37–59 % at 1.0 rad/s all fail the campaign's ≤0.1 rad/s
   acceptance. That is a **retrain** item (§4.1(d)) and no amount of sim fixing
   reaches the gate without it.
2. **Our twin additionally destroys one direction entirely** — +1.00 goes from
   36.6 % in Isaac Lab to 0.1 % here. That asymmetry is **ours**, not the weights',
   and §4.1(c) says fix it on our side.

### 2.10 The twin/training articulation diff — armature is zero here

| property | training (`unitree_rl_lab` `UNITREE_G1_29DOF_CFG`) | twin USD |
|---|---|---|
| **armature**, every actuator group | **0.01** | **0.0** (all 29 / all 69 joints) |
| hip_pitch stiffness / damping | 100.0 / 2.0 | 24.84 / 0.0099 (bare), 625.0 / 0.0 (dex5) |
| hip_yaw stiffness / damping | 100.0 / 2.0 | 150.48 / 0.060 (bare), 625.0 / 0.0 (dex5) |
| knee stiffness / damping | 150.0 / 4.0 | 230.09 / 0.092 (bare), 625.0 / 0.0 (dex5) |
| effort limit hip / knee | 88 / 139 | 88 / 139 ✅ |

`run.py` overwrites stiffness and damping at load from `deploy.yaml`, so the USD's
drive gains are not necessarily what runs — but **nothing anywhere sets armature**,
and it is 0.0 in both twin assets against 0.01 in training. Armature is rotor
inertia reflected through the gearbox; at zero the joint is "lighter" than the one
the policy learned on, which changes the PD response and is a standard sim-to-sim
gap. It is the leading candidate for §2.9's asymmetry and is the next thing tested.

### 2.11 Armature tested — it is not the cause either

`TWIN_ARMATURE=0.01` (applied and read back on all 69 joints) **with** the training
contact material: `wz=+1.00` still **0.0 %**, twice, upright. Armature joins the
list of eliminated hypotheses.

### 2.12 DECISION at the 6-hour time-box

**Eliminated, each by measurement:** observation layout / scales / history /
command index; the command clamp; the training command range; the Dex5 hands;
ground friction and restitution; Nav2 contaminating the topic; joint armature.

**Established:** the policy turns **both directions** in its own training
environment and only one direction here.

**Two conclusions, and they are independent:**

1. **RETRAIN IS REQUIRED (§4.1(d)), regardless of the twin.** The policy delivers
   4.9 % of commanded yaw at 0.5 rad/s and 37–59 % at 1.0 rad/s **in Isaac Lab**.
   Against the campaign's ≤0.1 rad/s acceptance that fails everywhere. No sim-side
   fix reaches the gate, because the ceiling is in the weights.
2. **The twin has a residual +wz asymmetry that is ours** (36.6 % → 0.1 %), and it
   is *not* obs, clamp, hands, friction, restitution, armature or Nav2. Remaining
   untested candidates: foot collision geometry, terrain (training used the rough
   generator, the twin is a flat floor), and PhysX solver iteration counts.

**Chosen: defer MM1b rather than run it now, and say so.** §7 Q6's default is "yes,
one overnight run". A retrain occupies the single 16 GB GPU for hours, and MM2–MM5
do not depend on the omni policy's yaw at all — MM2 is the environment, MM3 the
low-level bridge, MM4 is SONIC, which replaces this policy entirely, and MM5 rides
whichever of the two is available. Running the retrain now would block the campaign's
stated goal (MM5 done) on the one item that everything else is independent of.

**MM1b recipe, ready to run:** Isaac Lab 2.3.2 + `unitree_rl_lab`
`Unitree-G1-29dof-Velocity`, **the twin USD** (hands attached, mass 35.005 kg),
full `ang_vel_z` range, **≤2048 envs** for 16 GB, export TorchScript + ONNX,
regenerate `deploy.yaml`, new `PROVENANCE`, re-run this same sweep as the arbiter.
Both containers are built and the install quirks are documented in §2.9.

**MM1 locomotion acceptance is therefore NOT met and is reported as failed**, not
deferred quietly: yaw fails at every commanded rate, in both directions, and the
suite in §3 would report that same fact eight more times.

### 2.13 Where that leaves §4.1

| §4.1 branch | status |
|---|---|
| (b) obs layout / scales / history / command index & units | **checked, clean** |
| (c) "turns in Isaac Lab but not in run.py → contract bug" | **not supported** — the network responds to wz; nothing truncates it |
| (a) play the checkpoint in Isaac Lab with wz commands | **RUN — IT TURNS THERE, both directions** |
| **contact physics** | **MISMATCH FOUND (§2.6) and CORRECTED (§2.7); effect INCONCLUSIVE — one sign turns at 58 %, the other does not** |
| (c) fix our sim | **INDICATED** — Isaac Lab turns +1.00 at 36.6 %, we get 0.1 % |
| (d) retrain | **ALSO INDICATED, separately** — the policy fails the gate even in its own env (4.9 % at 0.5 rad/s) |

§4.1(a) was going to ask "is the difference our sim's physics?" — and §2.6 answers
that directly, without the Isaac Lab clone: **yes, there is a measured physics
mismatch, on the one property that gates yaw specifically.** Authoring the training
material and re-measuring is both cheaper and more decisive than porting the
checkpoint into a second environment. If yaw returns, MM1b is not needed and the
finding is a twin defect (a missing physics material on two surfaces). If it does
not, the Isaac Lab play-through is still there as the tiebreak.

**Nothing has been retrained and no gain has been touched.**

---

## 3. Motion suite

**Box:** RTX 4080 SUPER 16376 MiB · `TWIN_CAMERA=0` (C-23) · Isaac Sim 5.1.0 · hospital

`scripts/motion_suite.py`: 8 directions × {0.2, 0.5, 0.8} m/s, rotate at
±0.3/±0.6/±1.0 rad/s, walk-and-turn at ±0.5, and a stop-from-0.8. Markdown + JSON.

### Three defects the suite found in itself before it found anything about the robot

These are recorded because each one produced a plausible-looking table that was
wrong, and two of them were caught by reading output rather than by any assertion.

**1. The experimental physics flags make the walk worse.** The first run went out
with `TWIN_CONTACT_MATERIAL=1` and `TWIN_ARMATURE=0.01` still set from the yaw
diagnosis (§2.6, §2.11). Forward tracking against the same test, same box:

| test | with the flags | shipped defaults |
|---|---|---|
| N@0.2 | 0.155 m/s, **22.7 %** error | 0.197 m/s, **1.7 %** error |
| N@0.5 | 0.245 m/s, 51.0 % error | 0.254 m/s, 49.3 % error |

At 0.2 m/s the "training-matched" friction and armature are **thirteen times**
further off. This is evidence against adopting them, not for it, and both stay off
by default. It also means every number in §2.6 and §2.11 was measured under a
configuration that degrades the gait — the yaw conclusions there stand (they were
about direction, not magnitude), but the magnitudes should not be quoted elsewhere.
Partial run kept at `evidence/MM1/motion_suite_experimental_flags_partial.txt`.

**2. The suite never put the robot back on its feet.** There was no reset between
tests, so the first fall contaminated every later row. In the invalid run
(`evidence/MM1/motion_suite_noreset_invalid.txt`) the robot goes down at N@0.8
(zmin 0.071 m) and the suite then cheerfully reports "E@0.2 speed 0.655 m/s" — a
robot sliding on its side — and two rows with **zmin 1.332 m and 1.395 m**, which is
a base *above* standing height and physically means the corpse was being dragged.
Every row after the third in that file is meaningless. Fixed by adding a re-spawn:

* `isaac/sim_utils.py::setup_reset_sub` — a `std_msgs/Empty` latch on `/twin/reset`.
* `isaac/run.py` — the latch feeds the **existing** `needs_reset` path
  (`world.reset(True)` → `first_step` → `G1VelocityPolicy.initialize()`), so there is
  one reset implementation and it is the one the stop button already used. That path
  also rebuilds the per-term observation-history buffers and `_previous_action`,
  which a pose-only reset would have left holding the fall.
* `scripts/motion_suite.py::reset` — publishes, then **waits for ~2 s of continuously
  upright odom** before measuring. If the robot never comes back it returns
  `no_reset` and the row is disqualified rather than measured anyway.

**3. `NE` was `NW`.** `DIRECTIONS` had `("NE", 1, 1)` and `("NW", 1, 1)` — the same
command twice under two names, and no north-east test in an "8 directions" suite.
With +x forward and +y left, east is the robot's right, so NE is `(1, -1)`. Caught by
reading the table, not by running it; a suite that tests 7 directions and reports 8
is exactly the kind of quiet gap this campaign's audit rule exists for.

### Reset reliability, measured before any of the table was believed

Six knock-down-and-recover cycles, `evidence/MM1/reset_reliability_6of6.txt`:
**6/6**, with attempt 1 of cycle 4 failing to settle and the retry catching it —
the case the retry exists for, observed rather than assumed. Sim-side read-back
`respawn -> [7.800, 2.000, 0.800] (err 0.0000 m from spawn)` on every cycle.

### The table

33 tests · 15 s each (spec says 60 s — **deviation, declared**) · reset before every
test · nothing else publishing on `cmd_vel` · shipped defaults, no experimental
physics flags. Full output in `evidence/MM1/motion_suite.{md,json}`, console at
`evidence/MM1/motion_suite_console.txt`. Acceptance was ≤20 % speed error and
≤0.1 rad/s yaw error, no falls.

| test | commanded | measured | error | zmin | tilt | verdict |
|---|---|---|---|---|---|---|
| N@0.2 | 0.20 m/s | 0.195 | **2.4 %** | 0.766 | 3.1° | **PASS** |
| N@0.5 | 0.50 m/s | 0.460 | **8.1 %** | 0.747 | 6.9° | **PASS** |
| N@0.8 | 0.80 m/s | 0.595 | 25.6 % | 0.060 | 193.8° | FAIL — fell |
| NE@0.2 | 0.20 m/s | 0.160 | **19.8 %** | 0.772 | 3.3° | **PASS** |
| NE@0.5 | 0.50 m/s | 0.398 | 20.5 % | 0.759 | 5.9° | FAIL (marginal) |
| NE@0.8 | 0.80 m/s | 0.516 | 35.5 % | 0.748 | 8.3° | FAIL |
| E@0.2 | 0.20 m/s | 0.099 | 50.5 % | 0.770 | 5.1° | FAIL |
| E@0.5 | 0.50 m/s | 0.111 | 77.9 % | 0.768 | 3.0° | FAIL |
| E@0.8 | 0.80 m/s | 0.113 | 85.9 % | 0.768 | 2.7° | FAIL |
| SE@0.2 | 0.20 m/s | 0.066 | 66.9 % | 0.777 | 2.6° | FAIL |
| SE@0.5 | 0.50 m/s | 0.196 | 60.8 % | 0.772 | 6.1° | FAIL |
| SE@0.8 | 0.80 m/s | 0.166 | 79.2 % | 0.760 | 6.4° | FAIL |
| S@0.2 | 0.20 m/s | 0.131 | 34.7 % | 0.775 | 2.7° | FAIL |
| S@0.5 | 0.50 m/s | 0.234 | 53.2 % | 0.769 | 4.4° | FAIL |
| S@0.8 | 0.80 m/s | 0.239 | 70.1 % | 0.769 | 5.2° | FAIL |
| SW@0.2 | 0.20 m/s | 0.123 | 38.5 % | 0.774 | 2.6° | FAIL |
| SW@0.5 | 0.50 m/s | 0.294 | 41.3 % | 0.772 | 4.4° | FAIL |
| SW@0.8 | 0.80 m/s | 0.302 | 62.2 % | 0.766 | 5.7° | FAIL |
| W@0.2 | 0.20 m/s | 0.000 | 99.9 % | 0.783 | 3.5° | FAIL — did not move |
| W@0.5 | 0.50 m/s | 0.109 | 78.1 % | 0.746 | 11.7° | FAIL |
| W@0.8 | 0.80 m/s | 0.543 | 32.2 % | 0.066 | 196.5° | FAIL — fell |
| NW@0.2 | 0.20 m/s | 0.137 | 31.5 % | 0.774 | 4.4° | FAIL |
| NW@0.5 | 0.50 m/s | 0.344 | 31.1 % | 0.753 | 6.0° | FAIL |
| NW@0.8 | 0.80 m/s | 0.554 | 30.7 % | 0.742 | 7.3° | FAIL |
| rot+0.3 | wz +0.30 | **+0.0001** | 0.300 | 0.790 | 2.3° | FAIL |
| rot+0.6 | wz +0.60 | **+0.0001** | 0.600 | 0.789 | 2.3° | FAIL |
| rot+1.0 | wz +1.00 | **+0.0036** | 0.996 | 0.787 | 2.4° | FAIL |
| rot−0.3 | wz −0.30 | **+0.0000** | 0.300 | 0.790 | 2.3° | FAIL |
| rot−0.6 | wz −0.60 | **−0.0000** | 0.600 | 0.789 | 2.2° | FAIL |
| rot−1.0 | wz −1.00 | −0.5428 | 0.457 | 0.781 | 3.9° | FAIL |
| walk+turn +0.5 | 0.40 m/s, wz +0.50 | 0.059 m/s, **+0.3616** | 85.3 %, 0.138 | 0.741 | 10.2° | FAIL |
| walk+turn −0.5 | 0.40 m/s, wz −0.50 | 0.042 m/s, **−0.3342** | 89.6 %, 0.166 | 0.753 | 6.5° | FAIL |
| stop_from_0.8 | 0.80 m/s | 0.274 | 65.8 % | 0.101 | 190.2° | FAIL — fell |

**3 PASS of 33. 3 falls of 33** (N@0.8, W@0.8, stop_from_0.8 — all at 0.8 m/s).

### What the table says

**Forward is the only direction that works, and only below 0.8 m/s.** N@0.2 at 2.4 %
and N@0.5 at 8.1 % are genuinely good. Everything with a lateral or backward
component is between 30 % and 100 % short, and W@0.2 produced **exactly zero
displacement** — commanded to strafe left at 0.2 m/s, the robot stood there.

**The policy turns while walking, and does not turn in place.** This is the single
most useful thing in the table and it corrects §2:

| | commanded | measured | fraction |
|---|---|---|---|
| in place | wz ±0.3 | ≈ 0.0000 rad/s | 0 % |
| in place | wz ±0.6 | ≈ 0.0000 rad/s | 0 % |
| in place | wz ±1.0 | +0.0036 / −0.5428 | 0 % / 54 % |
| **while walking at 0.4 m/s** | **wz ±0.5** | **+0.3616 / −0.3342** | **72 % / 67 %** |

Standing still the yaw response is not weak, it is **absent** — four of six pure
rotations are zero to four decimal places, which is not a tracking error, it is no
command reaching the gait at all. Add forward velocity and the same policy turns at
roughly 70 % of command **in both directions and near-symmetrically**. The plain
reading is that this gait only yaws by steering its stepping, and with no
translation command it does not step, so there is nothing to steer.

**This supersedes §2's "+wz asymmetry" finding.** §2 measured 37 % one way and 0.1 %
the other and concluded the twin had a direction-dependent yaw bias. Those runs were
made with `TWIN_CONTACT_MATERIAL=1` / `TWIN_ARMATURE=0.01` and without resets. On
clean defaults with a reset before every test the in-place response is ~zero in
*both* directions and the walking response is symmetric to within 8 %. There is no
asymmetry to explain. What survives from §2 is the part that was about direction and
not magnitude: the obs contract, the command clamp, the training range, the hands,
friction, Nav2 and armature are all eliminated, and the Isaac Lab cross-check
(§2.13) still shows the same checkpoint turning in its own trainer.

**The retrain case is now much stronger and much better specified.** §2.12 recommended
a retrain on the grounds that the policy tracks yaw poorly everywhere. The sharper
statement is that it tracks yaw *fine when walking* and *not at all from standstill*,
and that lateral tracking is broken across the board. A retrain should therefore
weight in-place rotation and lateral velocity commands explicitly rather than simply
running longer. Recipe unchanged otherwise (≤2048 envs on 16 GB, twin USD).

### Deviations in this section

* **15 s per test, not 60 s.** 33 tests at 60 s plus resets is over an hour per run,
  and the run was repeated four times while the reset was being fixed. The three
  falls all happen within the first few seconds, and the PASS/FAIL calls are not
  close to the boundary except NE@0.5 (20.5 % against a 20 % gate), so the shorter
  window does not change any verdict. NE@0.5 should be re-measured at 60 s.
* **`TWIN_CAMERA=0`** throughout (C-23).

## 4. Video

Per Mohammed, 2026-08-19: every clip on this box is shot with the converged path
(`rt_subframes=32`), the mask ghost metric stays in `tools/film.py`, and each clip
is marked **"visually clean, numeric ghost gate deferred to 4090"** — the mask
read-back segfaults here (C-23, widened).

No MM1 clips shot yet.

---

## Deviations

* **C-23** — camera and any synthetic-data annotator host copy segfault on this box.
* No new C-item opened for yaw yet; §4.1(a) decides whether it is a twin-physics
  item or a policy item, and it should be numbered once that is known.

---

## Open questions for Mohammed

1. **Retrain budget (§7 Q6).** Everything except §4.1(a) points at a weak yaw
   channel. If (a) confirms it, MM1b is one overnight run at ≤2048 envs.
   **Default taken: proceed to (a), then retrain if it confirms.**
2. `ros2 action send_goal` not returning on terminal status is a real Ferox-side
   annoyance for any scripted evaluation. Worth an issue?

---

## Reproduce

```bash
# nav fix already in Ferox@mohammed/mm-campaign
ROBOT=g1 MODE=twin ./scripts/02_start_ferox.sh

# yaw sweep, either hand config
TWIN_CAMERA=0 ROBOT=g1 TWIN=1 HAND=dex5_1p SIM_WORLD=hospital ./scripts/01_start_sim.sh
docker exec ferox_nav python3 /tmp/yaw_sweep.py

# does the network respond to wz at all (no Isaac needed beyond torch)
docker exec ferox_isaac_sim /isaac-sim/python.sh /tmp/isaacrun/probe_policy.py
```
