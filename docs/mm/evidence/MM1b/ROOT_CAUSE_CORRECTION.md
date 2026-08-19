# Correction: MM1b's stated root cause was wrong

**Withdrawn 2026-08-19, before any conclusion was built on it further.**

## What I claimed

That the deployed `g1_omni` policy "was never asked to turn faster than 0.1 rad/s",
because `unitree_rl_lab`'s default `limit_ranges.ang_vel_z` is `(-0.2, 0.2)` and its
`ang_vel_cmd_levels` curriculum is defined but registered nowhere — dead code. I
reported that as "two lines of upstream config, not physics".

## What the checkpoint's own config says

`isaac/checkpoints/g1/params/env.yaml`, the snapshot written by the run that
produced the deployed weights:

```
commands/base_velocity/ranges        lin_vel_x (-0.1, 0.1)   lin_vel_y (-0.1, 0.1)   ang_vel_z (-1.0, 1.0)
commands/base_velocity/limit_ranges  lin_vel_x (-0.6, 1.0)   lin_vel_y (-0.5, 0.5)   ang_vel_z (-1.0, 1.0)
curriculum terms                     terrain_levels, lin_vel_cmd_levels
```

And `PROVENANCE_g1_omni.md` says so in words: *"widened command envelope:
lin_vel_x [-0.6,1.0], lin_vel_y [-0.5,0.5], ang_vel_z [-1.0,1.0] (deploy.yaml
gate-verified)"*, applied through `g1_omni_ranges.patch`.

So the training run **did** sample yaw commands across the full ±1.0 rad/s, from the
first iteration. The patch set `ranges.ang_vel_z` straight to the limit rather than
leaving it to a curriculum — which is why the absence of `ang_vel_cmd_levels` did
not matter, and why the yaw entry in `ranges` is at full range while the linear
entries are still at their curriculum starting value.

**My claim is false for this checkpoint.** What remains true is narrower and much
less interesting: upstream's *default* config caps yaw at ±0.2 and its yaw
curriculum is unused. Neither applies to the weights we deploy.

## What this changes

* MM1b's premise — "fix the config and the policy will turn" — is **unsupported**.
  The configuration was already what MM1b set out to change it to.
* The yaw failure goes back to what MM1 §2 measured directly and did not
  over-explain: the policy turns **weakly even inside Isaac Lab** — 4.9 % of
  commanded yaw at 0.5 rad/s, 37–59 % at 1.0 — while tracking forward velocity well.
  That is a property of the learned weights under this reward shaping, not of the
  command envelope.
* The MM1b overlay's "widened ranges" were therefore not new. What *was* new in my
  runs, and what actually differed from the checkpoint's recipe, is: the twin asset,
  a registered `ang_vel_cmd_levels`, the weighted sampler, and starting
  `ranges.ang_vel_z` at (-0.1, 0.1) under curriculum instead of pinning it to the
  full envelope as the original run did.

## Consequence for the plan

The reference run must reproduce **the checkpoint's own recipe** — yaw pinned at
±1.0 from iteration 0, linear under curriculum, no yaw curriculum — because that is
the configuration known to have produced a policy that walks. Anything that departs
from it needs to justify itself against that baseline rather than against upstream's
defaults.

## How this got through

I read upstream's `velocity_env_cfg.py` and never opened the checkpoint's own
`env.yaml` sitting in `isaac/checkpoints/g1/params/`, which records exactly what the
deployed run used. Six hours of MM1 elimination work made a config explanation feel
overdue, and I stopped checking when I found one that fit.
