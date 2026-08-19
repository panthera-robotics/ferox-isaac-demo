# MM1b divergence bisect — 2026-08-19

Acceptance cannot be attempted until training is stable. Each row is a 60-iteration
smoke on the same box. The two reward terms are the diagnostic ones: they are
`lin_vel_z_l2` (weight −2.0) and `ang_vel_xy_l2` (weight −0.05) on the BASE, so they
measure the robot being thrown rather than anything commanded.

| # | configuration | envs | phys Hz | base_lin_vel | base_ang_vel | mean reward |
|---|---|---|---|---|---|---|
| 1 | articulated hands, default PhysX buffers | 2048 | 200 | −963 | −2125 | −32,673 |
| 2 | articulated hands, buffers raised | 512 | 200 | −2839 | −2376 | −46,082 |
| 3 | hands damped (20→50 / 2→10 / 0.001→0.01) | 2048 | 200 | −467 | −252 | −3,120 |
| 4 | **hands LOCKED** (fixed joints, mass exact) | 1024 | 200 | −4031 | −1757 | −74,601 |
| 5 | hands locked, **dt bisect 500 Hz** | 1024 | 500 | −23,261 | −10,329 | −408,719 |
| 6 | **hands locked, VANILLA commands** | 1024 | 200 | **−18.3** | **−75.6** | **−2,771** |

## What each row rules out

* **Contact-patch overflow** — 1375 PhysX "Patch buffer overflow" errors on row 1,
  fixed by raising `gpu_max_rigid_patch_count`; 0 thereafter. Not the cause.
* **Env count** — 512 (row 2) is no better than 2048. Not the cause.
* **The env itself** — at 64 envs with zero actions the robot spawns at z=0.800
  upright, settles to 0.754 over five steps, and **nothing terminates**
  (`mm1b_termination.txt`). The scene is fine.
* **The hands** — locking them (row 4) did not cure it, so per Mohammed's rule the
  hands were a symptom. Damping them (row 3) helped a lot, which is consistent with
  them *amplifying* an instability rather than causing it.
* **Integration step** — 500 Hz made it **5× worse** (row 5). A genuine stiff-ODE
  problem improves with a smaller step. Ruled out, and informative: whatever this
  is, more substeps per control tick amplify it.
* **PD gains** — all 29 joints resolve to exactly the training values
  (100/2.0, 150/4.0, 40/1.0, armature 0.01 throughout), zero joints left on the
  USD's 625/0.0. `gains.txt`. Ruled out.

## What row 6 shows

Same asset, same box, same physics — **only the command configuration reverted to
upstream** — and the base is 220× calmer. The divergence is in the MM1b command
changes, not in the twin, the hands, the timestep, or the actuators.

The three changes are widened `limit_ranges`, the newly-registered
`ang_vel_cmd_levels` curriculum, and the weighted in-place/lateral/reverse sampler.
They have **not yet been bisected against each other** — that is the next step, and
it is three more 2-minute smokes, not another 47-minute run.

Worth noting: the curriculum never expanded in any row (`ang_vel_cmd_levels` and
`lin_vel_cmd_levels` both pinned at 0.1000), so the widened limits were never
actually reached. That makes the weighted sampler the leading suspect, since it is
the only one of the three that changes what the policy is asked to do at range 0.1.
