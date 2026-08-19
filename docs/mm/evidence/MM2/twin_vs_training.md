# Twin USD vs unitree_rl_lab training env — MM1b pre-flight

USD: `/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd`  ·  revolute joints found: **69**

## 1. Armature

- training imposes **0.01** on every actuator group
- twin USD: 0 of 69 joints carry a non-zero armature

**DIFF — the twin USD's armature is unset/zero.** Isaac Lab overrides it at spawn, so TRAINING is unaffected; the gap is on the twin side, and `TWIN_ARMATURE=0.01` exists to close it. MM1 §2.11 measured that it does not fix yaw, and MM1 §3 measured that it makes walking worse.

## 2. Drive gains (what the USD says vs what training imposes)

| group | USD stiffness | training | USD damping | training | USD maxForce | training effort |
|---|---|---|---|---|---|---|
| ankle | 625.0 | 40.0 | 0.0 | 2.0 | 35.0 | 25 |
| elbow | 625.0 | 40.0 | 0.0 | 1.0 | 25.0 | 25 |
| hip_pitch | 625.0 | 100.0 | 0.0 | 2.0 | 88.0 | 88 |
| hip_roll | 625.0 | 100.0 | 0.0 | 2.0 | 139.0 | 139 |
| hip_yaw | 625.0 | 100.0 | 0.0 | 2.0 | 88.0 | 88 |
| knee | 625.0 | 150.0 | 0.0 | 4.0 | 139.0 | 139 |
| shoulder | 625.0 | 40.0 | 0.0 | 1.0 | 25.0 | 25 |
| wrist_pitch | 625.0 | 40.0 | 0.0 | 1.0 | 5.0 | 5 |
| wrist_roll | 625.0 | 40.0 | 0.0 | 1.0 | 25.0 | 25 |
| wrist_yaw | 625.0 | 40.0 | 0.0 | 1.0 | 5.0 | 5 |
| waist_yaw | 625.0 | 200.0 | 0.0 | 5.0 | 88.0 | 88 |

Isaac Lab replaces these at spawn from `ImplicitActuatorCfg`, and `run.py` replaces them at load from `deploy.yaml`. Neither side uses the USD values, so a difference here is not itself a sim-to-sim gap — what matters is whether `deploy.yaml` equals the training config.

## 3. Friction

| | static | dynamic | restitution |
|---|---|---|---|
| training ground material | 1.0 | 1.0 | 0.0 |
| training, **randomised every episode** | (0.3, 1.0) | (0.3, 1.0) | (0.0, 0.0) |
| twin USD | *(no UsdPhysics.MaterialAPI found)* | | |

**This is the important one.** Training does not train against a single friction: an event term samples static and dynamic friction in **[0.3, 1.0]** every episode, combined `multiply` on top of a 1.0/1.0 ground. PhysX's default 0.5 — what the twin used before `TWIN_CONTACT_MATERIAL` — sits INSIDE that training distribution. Forcing 1.0 puts the twin at the distribution's edge, which is consistent with MM1 §3 measuring the walk as *worse* with the flag on. The retrain should keep this randomisation; the twin should not pin friction to either end of it.

## 4. Terrain

- training: **generator COBBLESTONE_ROAD_CFG (50% flat sub-terrain)**
- twin: a single flat floor

Half the training sub-terrains are flat, so flat is in-distribution. Untested as a yaw cause (MM1 §2.12) and left alone here.

## 5. Foot collision geometry

**Probe inconclusive — and the probe is what is wrong, not the asset.** Traversing
`g1_dex5_1p.usd` for `UsdPhysics.CollisionAPI` returns **0 prims**, which cannot be
true of a robot that demonstrably stands on a floor and collides with a table. The
detection is at fault (the collision APIs are applied inside the referenced
`*_physics.usd` layer and are not being matched by `HasAPI` as written). Recorded as
a gap in this tool rather than reported as "the twin has no collision geometry",
which would be a fabricated defect of exactly the kind §2 of this file already
caught once.

`UNITREE_MODEL_DIR` in `unitree_rl_lab` is the literal placeholder `path/to/unitree_model`, and our G1 USD was staged there (MM1 §2.9). Training and the twin therefore load **the same mesh**, so foot collision cannot differ between them — it is shared by construction. That closes the last of MM1 §2.12's untested candidates that the retrain could inherit.

