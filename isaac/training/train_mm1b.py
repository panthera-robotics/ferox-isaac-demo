"""MM1b trainer — self-contained, because upstream's train.py cannot run this task.

Two reasons it cannot:
  * `scripts/rsl_rl/train.py` imports gymnasium at module scope, BEFORE AppLauncher
    starts kit. Isaac's gymnasium lives in kit's pip3-envs, so that import fails
    outside `isaaclab.sh`.
  * its `--task` choices are filtered to ids containing "Unitree", so
    `Ferox-G1-29dof-Velocity-v2` is rejected by argparse before anything runs.

Everything after the app launch mirrors upstream's main() so the run is comparable:
same OnPolicyRunner, same agent cfg entry point, same deploy-cfg export.
"""
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="MM1b: retrain the G1 omni policy.")
parser.add_argument("--task", default="Ferox-G1-29dof-Velocity-v2")
parser.add_argument("--num_envs", type=int, default=2048)
parser.add_argument("--max_iterations", type=int, default=6000)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--log_root", default="/logs")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os          # noqa: E402
import shutil      # noqa: E402
import sys         # noqa: E402
from datetime import datetime  # noqa: E402

import gymnasium as gym        # noqa: E402
import torch                   # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

import isaaclab_tasks  # noqa: F401,E402
from isaaclab.utils.io import dump_yaml     # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg     # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

import unitree_rl_lab.tasks  # noqa: F401,E402

sys.path.insert(0, "/workspace/ferox_isaac")
import training.mm1b_g1_omni_v2  # noqa: F401,E402  registers the task

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True


def main() -> None:
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device,
                            num_envs=args_cli.num_envs)
    agent_cfg = load_cfg_from_registry(args_cli.task, "rsl_rl_cfg_entry_point")
    agent_cfg.max_iterations = args_cli.max_iterations
    agent_cfg.seed = args_cli.seed
    env_cfg.seed = args_cli.seed

    # Fail loudly rather than train something undeployable. The pre-flight already
    # checks this; repeating it here means a config edit between the two cannot
    # quietly cost an overnight run (it nearly did once: joint_names=[".*"] on the
    # 69-DoF twin gave 1080 obs / 69 actions against a 480 / 29 contract).
    log_dir = os.path.join(args_cli.log_root, "rsl_rl", agent_cfg.experiment_name,
                           datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_mm1b")
    os.makedirs(log_dir, exist_ok=True)

    env = gym.make(args_cli.task, cfg=env_cfg)
    n_act = env.action_space.shape[-1]
    n_obs = env.observation_space["policy"].shape[-1]
    print(f"[MM1b] actions={n_act} obs={n_obs}", flush=True)
    if n_act != 29 or n_obs != 480:
        raise RuntimeError(
            f"deployment contract broken: actions={n_act} (want 29), "
            f"obs={n_obs} (want 480). run.py could not load this policy.")

    env = RslRlVecEnvWrapper(env, clip_actions=getattr(agent_cfg, "clip_actions", None))
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir,
                            device=agent_cfg.device)

    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
    try:
        from unitree_rl_lab.utils.export_deploy_cfg import export_deploy_cfg
        export_deploy_cfg(env.unwrapped, log_dir)
    except Exception as exc:  # noqa: BLE001
        print(f"[MM1b] export_deploy_cfg failed (not fatal now, needed at export): {exc}",
              flush=True)
    try:
        shutil.copy("/workspace/ferox_isaac/training/mm1b_g1_omni_v2.py",
                    os.path.join(log_dir, "params", "mm1b_g1_omni_v2.py"))
    except Exception:
        pass

    print(f"[MM1b] logging to {log_dir}", flush=True)
    runner.learn(num_learning_iterations=agent_cfg.max_iterations,
                 init_at_random_ep_len=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
