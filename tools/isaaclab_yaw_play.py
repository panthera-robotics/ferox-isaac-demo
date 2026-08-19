#!/usr/bin/env python3
"""4.1(a): does the g1_omni checkpoint turn in ITS OWN training environment?

The decisive discriminator for MM1's yaw item. Everything on our side has been
eliminated -- obs layout/scales/history, the command clamp, the training command
range, the Dex5 hands, ground friction, and Nav2 contaminating the topic -- and
the policy still yields 33-51% of commanded yaw at -1.0 rad/s and ~0% everywhere
else, while standing.

So the question is no longer "what is different about our sim". It is "does this
checkpoint turn AT ALL, anywhere". This script asks that in unitree_rl_lab's
Unitree-G1-29dof-Velocity env -- the task the policy was trained on -- driving it
with OUR exported TorchScript so the policy under test is byte-identical to the
one the twin runs.

If it turns here      -> the difference is our sim, keep diffing (PD gains,
                         armature, foot collision shape, terrain).
If it does not        -> the yaw channel is weak in the weights themselves and
                         4.1(d) retrain is the answer.

Run inside an Isaac Sim container with IsaacLab + unitree_rl_lab installed:
  /isaac-sim/python.sh isaaclab_yaw_play.py --wz -1.0 --seconds 8
"""
from __future__ import annotations
import argparse, json, math, sys

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--task", default="Unitree-G1-29dof-Velocity")
ap.add_argument("--policy", default="/loco/policy/exported/policy.pt")
ap.add_argument("--wz", type=float, nargs="+", default=[-1.0, 1.0, 0.5, -0.5])
ap.add_argument("--seconds", type=float, default=8.0)
ap.add_argument("--json", default="/tmp/isaaclab_yaw.json")
AppLauncher.add_app_launcher_args(ap)
args, _ = ap.parse_known_args()
args.headless = True
app = AppLauncher(args).app

import gymnasium as gym            # noqa: E402
import torch                        # noqa: E402
import isaaclab_tasks               # noqa: E402,F401
import unitree_rl_lab.tasks         # noqa: E402,F401
from isaaclab_tasks.utils import parse_env_cfg   # noqa: E402


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    cfg = parse_env_cfg(args.task, device="cuda:0", num_envs=1)
    env = gym.make(args.task, cfg=cfg)
    policy = torch.jit.load(args.policy, map_location="cuda:0").eval()

    out = {"task": args.task, "policy": args.policy, "rows": []}
    for wz in args.wz:
        obs, _ = env.reset()
        o = obs["policy"] if isinstance(obs, dict) else obs
        print(f"[obs] shape {tuple(o.shape)}", flush=True)

        # Pin the velocity command: the env resamples it on its own schedule, so
        # overwriting once is not enough -- it is re-pinned every step below.
        cm = env.unwrapped.command_manager
        dt = env.unwrapped.step_dt
        steps = int(args.seconds / dt)

        def pin():
            try:
                cm.get_term("base_velocity").vel_command_b[:] = torch.tensor(
                    [[0.0, 0.0, wz]], device=o.device)
            except Exception as exc:                     # pragma: no cover
                print(f"[warn] could not pin command: {exc}", flush=True)

        pin()
        root = env.unwrapped.scene["robot"].data
        y_prev = yaw_of(root.root_quat_w[0].tolist())
        acc = 0.0
        zmin = float(root.root_pos_w[0, 2])
        for i in range(steps):
            pin()
            with torch.inference_mode():
                a = policy(o)
            obs, _, term, trunc, _ = env.step(a)
            o = obs["policy"] if isinstance(obs, dict) else obs
            root = env.unwrapped.scene["robot"].data
            y = yaw_of(root.root_quat_w[0].tolist())
            acc += math.atan2(math.sin(y - y_prev), math.cos(y - y_prev))
            y_prev = y
            zmin = min(zmin, float(root.root_pos_w[0, 2]))
            if bool(term[0]) or bool(trunc[0]):
                print(f"[wz={wz:+.2f}] episode ended at step {i}", flush=True)
                break
        elapsed = max(1e-3, (i + 1) * dt)
        rate = acc / elapsed
        row = {"wz": wz, "dyaw_rad": acc, "seconds": elapsed, "rate": rate,
               "track_pct": 100 * rate / wz if wz else 0.0, "zmin": zmin,
               "steps": i + 1}
        out["rows"].append(row)
        print(f"  wz={wz:+.2f}  dyaw={acc:+7.3f} rad over {elapsed:.2f}s  "
              f"rate={rate:+.4f} ({row['track_pct']:+6.1f}%)  zmin={zmin:.3f}",
              flush=True)

    env.close()
    json.dump(out, open(args.json, "w"), indent=2)
    print("\n=== ISAAC LAB (training env) ===")
    for r in out["rows"]:
        print(f"  wz={r['wz']:+.2f}  {r['track_pct']:+6.1f}% of commanded  zmin={r['zmin']:.3f}")
    app.close()


main()
