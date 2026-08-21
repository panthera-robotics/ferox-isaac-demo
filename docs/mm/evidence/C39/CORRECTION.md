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
