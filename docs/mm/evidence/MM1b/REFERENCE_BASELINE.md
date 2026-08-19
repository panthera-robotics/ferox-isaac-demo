# MM1b (a)/(b)/(c) — the reference baseline, 300-iteration runs

Same config throughout: the **deployed checkpoint's own recipe**, read from
`isaac/checkpoints/g1/params/env.yaml` — yaw pinned at ±1.0 from iteration 0,
linear under `lin_vel_cmd_levels`, no yaw curriculum, no weighted sampler.
1024 envs, 200 Hz physics, 50 Hz control. Only the **asset** differs.

| | (a) upstream G1 | (b) twin, hands locked | (b2) + hand colliders off |
|---|---|---|---|
| actions / obs | 29 / 480 | 29 / 480 | 29 / 480 |
| mean reward | **+3.96** | −359,324 | −1.29 |
| `base_linear_velocity` | **−0.0017** | −10,926 | **−0.0054** |
| `base_angular_velocity` | **−0.0138** | −7,094 | **−0.0116** |
| episode length, final | **181.36** | 1.00 | 9.07 |
| `bad_orientation`, final step | 2 / 1024 | 236 / 1024 | 109 / 1024 |

## Episode length over the run — the number that separates all three

| iteration | (a) | (b2) |
|---|---|---|
| ~1 | 13.00 | 24.00 |
| ~30 | 15.49 | 13.12 |
| ~60 | 9.05 | 9.43 |
| ~90 | 13.63 | 9.16 |
| ~120 | 18.99 | 9.13 |
| ~150 | 22.74 | 9.05 |
| ~180 | 29.91 | 9.07 |
| ~210 | 68.91 | 9.06 |
| ~240 | 84.82 | 9.10 |
| ~270 | 100.58 | 9.05 |
| final | **181.36** | **9.05** |

## What this establishes

1. **The install is fine.** (a) learns: episode length climbs from 13 to 181 and
   mean reward is positive. Isaac Lab, rsl-rl 3.1.2 and unitree_rl_lab are not the
   problem, and neither is the box.
2. **The twin asset is the cause of the divergence**, exactly as predicted, and
   **fixing the joints was not enough** — a fixed joint removes a DOF, not a
   collider. (b) explodes with 236 of 1024 envs terminating on `bad_orientation`.
3. **The hand colliders are the specific cause.** Deactivating the 40 `collisions`
   subtrees takes `base_linear_velocity` from −10,926 to −0.0054 — into the same
   regime as the reference — with mass still preserved exactly at 35.004757 kg.
4. **But (b2) does not learn.** Episode length is pinned flat at 9.05 while the
   reference climbs past 180. Stable is not the same as healthy, and the health gate
   is the trend, not the absence of an explosion.

## And on "episode length 1.00"

It is explained, not dismissed. In (b) the termination manager reports
`bad_orientation` firing on 236 of 1024 envs at a single step and `time_out` at
0.7695 of episode ends — the robots are tipping past the 0.8 rad limit almost
immediately, so episodes are one step long because the robot is destroyed on contact,
not because the metric is broken. In (a), the same counter reads 2 of 1024 and the
length climbs. The metric was always meaningful; my earlier runs simply never had a
healthy comparison to read it against.

## What is left

The remaining gap between (b2) and (a) is the hands' **mass and inertia at the end of
the arms** — ~1.5 kg carried well outside the CoM, against reward shaping tuned for
the bare robot. That is real physics the policy has to learn, not a defect.

The next step in Mohammed's plan is the one not yet built: `g1_29dof_train.usd` with
**one rigid link per hand** carrying the hand's total mass, CoM and inertia plus a
single convex-hull collider. That both restores a collider (b2 has none, so the hands
currently pass through the world) and gives PhysX one well-conditioned body instead of
twenty fixed-jointed ones, which is the more likely fix for the flat curve.
