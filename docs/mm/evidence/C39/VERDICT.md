# C-39 — the diff, and the verdict row

> **SUPERSEDED IN PART, 2026-08-22 — read `CORRECTION.md` §4 first.** The closing
> section of this file reads the static-PD fall as pointing "at the SOLVER or the
> articulation's own constraint behaviour". **That inference is withdrawn.** A
> fixed-target PD hold at ankle kp 40 (80 Nm/rad across both ankles) cannot stabilise
> an inverted pendulum whose toppling stiffness is 264 Nm/rad (twin) or 234 Nm/rad
> (the **reference** model) — so the hold falls in any simulator and discriminates
> nothing. The per-link mass diff and the foot/contact-API findings below stand
> unchanged; only the conclusion drawn from the static hold is void.

## The +4.000 kg is four placeholder sensor masses

The runtime articulation sums to **39.004757 kg** against DT3/MM1b's asserted
**35.004757 kg** — a difference of **exactly 4.000000 kg**, the fractional parts
identical. Differencing the two twin variants (`links_dex5.csv` minus
`links_none.csv`) finds it:

| link | twin mass | real part weighs |
|---|---|---|
| `d435_link` | **1.000000** | RealSense D435 ≈ 0.072 kg |
| `mid360_link` | **1.000000** | Livox MID360 ≈ 0.265 kg |
| `imu_in_torso` | **1.000000** | a few grams |
| `imu_in_pelvis` | **1.000000** | a few grams |
| | **= 4.000000 kg** | |

Four links at exactly 1.000 kg is the URDF importer's default for a link with no
inertial block. **Three of the four sit high** — torso, head-mounted camera, lidar —
so this is not just 4 kg of ballast, it is 3 kg of phantom mass placed where it does
the most damage to a balance controller, plus the inertia that goes with it.

**What was summed, precisely:** `physx.get_masses()` over the articulation's own
`body_names` — 79 links for `dex5_1p`, 30 for `HAND=none`, **no duplicate names on
either**. The rig is not a prim, and the lab objects are not in this articulation, so
nothing external can enter the total.

## Verdict row of the bare-vs-reference diff

`tools/c39_diff.py links_none.csv links_mujoco_ref.csv` — 30 links matched, **no links
unique to either side**:

| link | twin | reference | Δ |
|---|---|---|---|
| **`torso_link`** | **7.816999** | **9.598** | **−1.781001** |
| `waist_roll_link` | 0.086 | 0.047 | +0.039 |
| `waist_yaw_link` | 0.214 | 0.244 | −0.030 |
| everything else | | | ≤ 0.001 |

Total −1.771 kg, essentially all of it in one link. Note the direction: against the
MuJoCo reference the twin's torso is **lighter**, not heavier.

## What this does NOT explain, stated plainly

The static-margin story is dead and the corrected numbers say so:

    total mass                39.004757 kg
    CoM (true, origin+R*com)  [7.7993, 2.0079, 0.7376]
    stance width              0.2370 m in X  -> at this spawn yaw X is LATERAL, +Y is FORWARD
    foot spheres vs ankle     heel -0.0240 m   toe +0.1460 m   width 0.0600 m
    CoM - ankle, FORWARD      +0.0339 m
    margin to toe             0.1121 m        margin to heel  0.0579 m
    => CoM is INSIDE the support polygon

**The twin is statically stable at the nominal stance**, with a foot whose toe reaches
0.146 m ahead of the ankle against the reference's 0.120 m. Two earlier numbers were
wrong and are withdrawn: the CoM report weighted link ORIGINS rather than centres of
mass, and `com_x_minus_ankle_x` measures the LATERAL offset at this spawn yaw, not the
static margin. So "required bias 0.73 rad > ankle travel 0.52 rad" is not the
conviction — there is no static-margin defect to convict.

The 4.000 kg of phantom sensor mass is a **real asset defect worth fixing on its own
terms** — it is 11% of the robot's mass, sitting high — but the corrected geometry says
it is not, by itself, a proof of why the twin falls. Fixing it and re-running the
discriminator is the test.

## Foot colliders — same architecture as the reference

Read with instance proxies (`Usd.TraverseInstanceProxies`); a `PrimRange` rooted at
`/World/G1` finds nothing because URDF import instances link geometry under
`/Flattened_Prototype_NNN/...`, which is what made the first read look like "no
colliders".

- visual mesh: `Mesh`, **collisionAPI False** — non-colliding, as in the reference
- collision: **four `Sphere` prims per foot, r = 0.005**, at forward −0.024 / +0.146 and
  lateral ±0.030 relative to the ankle
- reference: four spheres r = 0.005 at forward −0.05 / +0.12, lateral ±0.025..0.03

Same design, comparable extent, twin slightly toe-forward. Not a suspect.

## Contact API in this build (unblocks grasp (a)/(d))

| route | status |
|---|---|
| `isaacsim.core.prims.RigidContactView` | **absent** (module imports, attribute missing) |
| `isaacsim.sensors.physics.ContactSensor` | **present** |
| `omni.physx.get_physx_simulation_interface` | **present** |
| articulation `get_net_contact_forces` | **absent** on wrapper and physx view |
| physx `get_link_incoming_joint_force` | present — **joint reaction, NOT contact** |

Two viable routes: a `ContactSensor` prim per body (must exist before stepping), or the
physx contact-report callback with `PhysxContactReportAPI` on the bodies of interest.
Probe: `scripts/c39_contact_probe.py`. The last route is what previously got misread as
ground contact in the foot audit.

## MuJoCo reference standing under SONIC — already on disk

`evidence/MM4/ab/mujoco.json`: 990 lowstate samples over 5 s at 197.03 Hz with
quat w = 0.999753, captured while the deploy binary drove the reference sim. Not
re-run.

---

## Task 3: the fix was applied, asserted, and does NOT stand the robot

`TWIN_SENSOR_MASS_FIX=1` replaces the four placeholder masses with the real components'
published figures (D435 0.072, MID-360 0.265, IMUs 0.005 each), reads every one back,
and asserts the total:

    [TWIN] sensor mass d435_link:     1.000000 -> 0.072000 kg
    [TWIN] sensor mass mid360_link:   1.000000 -> 0.265000 kg
    [TWIN] sensor mass imu_in_torso:  1.000000 -> 0.005000 kg
    [TWIN] sensor mass imu_in_pelvis: 1.000000 -> 0.005000 kg
    [TWIN] total mass 39.004749 -> 35.351749 kg (removed 3.653000 kg)

The remaining 0.347 kg over the asserted 35.004757 is the real sensors' own mass, which
the assertion evidently carried as zero.

**Static PD hold, base free, nothing running: still falls.** `base_z 0.125,
pitch 1.562` — indistinguishable from the 39 kg run. That is what the corrected
geometry predicted: the CoM was already inside the support polygon with 0.112 m of
margin to the toe, so taking 3.65 kg off the top cannot change whether the pose is
statically holdable. The defect is real and is now fixed on its own merits — 11% of
the robot's mass, wrongly placed — but **it is not C-39**.

One implementation note worth keeping: `set_masses` on this build requires a positional
`indices` argument, and omitting it raises a `TypeError` that reads exactly like an
absent API. The first attempt failed silently-ish for that reason and produced a
bit-identical run, which is the most misleading possible outcome for a "fix".

## Where C-39 actually stands after tasks 1-3

Everything the brief proposed as the cause has now been measured and cleared:

| hypothesis | status |
|---|---|
| wire / message content | dead (turn 8: full MuJoCo lie changes nothing) |
| SONIC's observations | dead (tilt tracks to 0.003°, own logs match ground truth, zero lag) |
| control loop closed | confirmed (15° tilt moves q_d by 1.15 rad) |
| the hands | dead (bare 29-DoF robot falls identically) |
| force path (explicit vs implicit) | dead (identical fall) |
| foot-ground friction, joint friction, physics rate | dead |
| static margin / CoM over feet | **dead — CoM is inside the polygon, 0.112 m from the toe** |
| placeholder sensor mass (+4.000 kg) | **real defect, fixed, does not stand the robot** |

What has NOT been tested is the one thing every one of these shares: the twin falls
**with no controller at all**, from a pose that is statically stable, with correct feet,
correct mass and a closed control loop. A statically stable pose that will not stand
under a stiff joint-space hold, in either force path, points at the SOLVER or the
articulation's own constraint behaviour rather than at any parameter yet examined —
joint drive limits and `set_max_efforts` actually taking effect, articulation
self-collision, or the root/base handling when the rig releases. That is the next
session's first experiment and it needs the sim, not a diff.

---

## Withdrawn 2026-08-22 — the closing inference

See `CORRECTION.md` §4. "A statically stable pose that will not stand under a stiff
joint-space hold ... points at the SOLVER" assumed the hold was capable of standing it.
It was not: `2·kp = 80 Nm/rad` against `m·g·h = 264 Nm/rad`. The hold is an unstable
equilibrium by a factor of three, in the twin and in the reference alike. The live
question is SONIC-in-twin vs SONIC-in-MuJoCo, and nothing else.
