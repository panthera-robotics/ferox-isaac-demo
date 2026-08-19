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

### 2.9 Where that leaves §4.1

| §4.1 branch | status |
|---|---|
| (b) obs layout / scales / history / command index & units | **checked, clean** |
| (c) "turns in Isaac Lab but not in run.py → contract bug" | **not supported** — the network responds to wz; nothing truncates it |
| (a) play the checkpoint in Isaac Lab with wz commands | **not yet run** |
| **contact physics** | **MISMATCH FOUND (§2.6) and CORRECTED (§2.7); effect INCONCLUSIVE — one sign turns at 58 %, the other does not** |
| (d) retrain | **still not indicated** — §2.7's follow-up runs first |

§4.1(a) was going to ask "is the difference our sim's physics?" — and §2.6 answers
that directly, without the Isaac Lab clone: **yes, there is a measured physics
mismatch, on the one property that gates yaw specifically.** Authoring the training
material and re-measuring is both cheaper and more decisive than porting the
checkpoint into a second environment. If yaw returns, MM1b is not needed and the
finding is a twin defect (a missing physics material on two surfaces). If it does
not, the Isaac Lab play-through is still there as the tiebreak.

**Nothing has been retrained and no gain has been touched.**

---

## 3. Motion suite — not yet built

`scripts/motion_suite.py` (8 directions × 3 speeds, rotate ±0.3/±0.6/±1.0,
turn-while-walking, stop-from-0.8, 60 s each, hands on) is the next item. It is
deliberately after the yaw diagnosis: the rotate rows would all read 0 % today and
the table would say nothing that §2 has not already said with fewer runs.

---

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
