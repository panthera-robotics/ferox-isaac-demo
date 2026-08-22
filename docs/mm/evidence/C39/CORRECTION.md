# C-39 — correction: the static-margin conviction was wrong, and so was the mass sum

Recorded before anything else because two numbers in the previous session's write-up
are wrong and were used to draw a conclusion.

## 1. The static-PD fall does not convict the stance

Stated correctly in the brief and accepted: a controller biases its targets, so a
robot falling under a *fixed-target* PD hold is not evidence that the pose cannot be
held. The proposed conviction was `required bias (0.73 rad) > ankle travel (0.52 rad)`.

**That conviction also fails, and the reference is what fails it.** Running the same
arithmetic on the reference MJCF at the same nominal stance (`tools/c39_mjcf_mass.py`,
no GPU):

| | twin (as previously computed) | MuJoCo reference |
|---|---|---|
| total mass | 39.005 kg | **35.112 kg** |
| CoM − ankle x | +0.054 m | **+0.0497 m** |
| required ankle torque | 20.7 Nm | **17.13 Nm** |
| bias at wire kp 28.5 | 0.73 rad | **0.60 rad** |
| ankle pitch travel | 0.5236 rad | **0.5236 rad — identical** |

The reference needs 0.60 rad of bias against 0.52 rad of travel and **it stands**. So
"required bias exceeds travel" cannot be the defect; something else carries the moment.

## 2. The mass sum was wrong, and so was the CoM

The `+4.000 kg` was real and is now measured exactly: the runtime articulation sums to
**39.004749 kg** over **79 links with no duplicate names**, from
`physx.get_masses` over the articulation's own body list. The rig is not a prim, the
lab objects are not in this articulation, and there is no duplicated link — so the
excess is genuinely inside the robot. Against DT3/MM1b's 35.004757 with hands that is
**+3.999992 kg**.

Two visible contributors, from the per-link table: the twin carries sensor links the
29-DoF MJCF has no equivalent for (`d435_link` **1.000 kg**, plus head/imu/lidar
links), while its `torso_link` is **6.780 kg** against the reference's **9.598 kg**.
So this is not "4 kg bolted on" — it is a different mass *distribution*, and the
per-link diff is the right instrument. That diff is now unblocked: `links_dex5.csv`.

**And the CoM figure was an artifact.** The `[COM]` report weighted link ORIGINS as if
they were centres of mass. Recomputed properly as `origin + R·com_local`:

    com_world        [7.79931, 2.007902, 0.737577]
    ankle_mean_x      7.799979
    com_x - ankle_x  -0.000669 m

**The twin's CoM at the nominal stance is directly over its ankles** — better centred
than the reference's +0.0497 m — and the required static ankle torque is therefore
approximately zero, not 20.7 Nm. **The static-margin hypothesis is dead.** The twin
still falls, so the cause is elsewhere, and the previous session's explanation of *why*
it falls should be disregarded. What survives from that session is the observation
itself: it falls with nothing running, and implicit drives change nothing.

## 3. The feet are the same architecture, not a suspect

Read properly this time. The first attempt found zero foot colliders and that looked
like "the feet have no colliders"; in fact URDF import puts link geometry under
`/Flattened_Prototype_NNN/...` and instances it, so a `PrimRange` rooted at `/World/G1`
finds nothing. Traversing the stage **with instance proxies**:

- `left/right_ankle_roll_link/visuals/.../mesh` — `Mesh`, **collisionAPI False**
- `left/right_ankle_roll_link/collisions/mesh_{0,1,2,3}/sphere` — **four `Sphere`
  prims per foot, collisionAPI True**

That is the same design as the reference: a non-colliding visual mesh plus four contact
spheres. The visual mesh spans `x ∈ [-0.0658, +0.1424]` in the ankle frame, so the
foot's physical extent is comparable to the reference's sphere placement
(heel −0.05, toe +0.12). Sphere centres and radii are the remaining number to pull.

---

## 4. The static-hold fall is not a discriminator at all (2026-08-22, recorded first)

**Correction accepted from Mohammed, and it voids the "solver" inference in
`VERDICT.md` §"Where C-39 actually stands".** A fixed-target joint-space PD hold at
these gains **cannot** stabilise this robot, in this simulator or any other, and the
arithmetic is one line.

An upright biped held by ankle PD is an inverted pendulum. Its toppling moment grows
with lean as **m·g·h** per radian; the ankle PD restores **kp** per radian per ankle.
The equilibrium is stable only if `2·kp > m·g·h`. Measured from this campaign's own
evidence files (`links_none.csv`, `links_dex5.csv`, `mujoco_ref.json`; h = CoM height
above the ankle-roll joint):

| model | mass | CoM z | ankle z | h | **m·g·h** |
|---|---|---|---|---|---|
| twin, Dex5 hands | 39.0048 kg | 0.737576 | 0.048315 | 0.689261 m | **263.7 Nm/rad** |
| twin, bare 29-DoF (`HAND=none`) | 33.3411 kg | 0.702113 | 0.048315 | 0.653798 m | **213.8 Nm/rad** |
| **reference `g1_29dof_old.xml`** | 35.1121 kg | 0.723205 | 0.043798 | 0.679407 m | **234.0 Nm/rad** |

Against that, the hold's actual restoring stiffness:

| gain | one ankle | both ankles |
|---|---|---|
| `HOLD_KP` ankle 40 | 40 Nm/rad | **80 Nm/rad** |
| SONIC's wire kp 28.5 | 28.5 Nm/rad | **57 Nm/rad** |

**80 ≪ 264, and 80 ≪ 234.** The margin is a factor of 3, not a few percent, so no
measurement error in h or in the gain changes the sign. Mohammed's figure of
≈190 Nm/rad is the same argument taken at a slightly lower effective CoM; every value
in the plausible range is far above 80.

Three consequences, stated plainly:

1. **The static discriminator was always going to fall.** Every run of it — 39 kg and
   35.35 kg, explicit force path and implicit, SONIC running and SONIC absent — was
   measuring an unstable equilibrium. The identical results across all of those are
   what the arithmetic predicts, not a clue.
2. **It would fall in MuJoCo too.** The reference model's own m·g·h is 234 Nm/rad
   against the same 80. So "it falls under a static hold" is **not evidence about the
   simulator, the solver, the asset, or the twin** — it does not discriminate between
   any pair of them. The `VERDICT.md` line that reads this as pointing "at the SOLVER
   or the articulation's own constraint behaviour" **does not follow and is withdrawn.**
3. **This is why a balancer moves its targets.** SONIC does not rely on stiffness; it
   re-commands `q_d` at 50 Hz, so its ankle torque comes from a *deliberately biased*
   target rather than from lean error against a fixed one. That is the mechanism a
   fixed-target hold does not have, and it is why only a run with SONIC actually
   driving can say anything.

**The live question is therefore exactly one comparison: SONIC-in-twin vs
SONIC-in-MuJoCo.** Nothing else in the C-39 file discriminates. That is what task 1
runs, and it is why it runs the *reference asset* inside *our simulator* — the single
experiment that splits "our simulator" from "our body".

**Standing caveat for anyone reading the older rows:** the static-hold results are not
wrong as observations, they are simply uninformative. Do not cite them as evidence for
or against any hypothesis.
