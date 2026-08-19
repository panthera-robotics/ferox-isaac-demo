"""MM1b — retrain the G1 omni locomotion policy so it can actually turn.

Registers task `Ferox-G1-29dof-Velocity-v2`. It is a thin overlay on
`unitree_rl_lab`'s `Unitree-G1-29dof-Velocity`: same robot, same observations, same
reward terms and weights. Three things change, and each is a measured cause rather
than a knob.

WHY THE OLD POLICY CANNOT TURN — two facts from the upstream config
-------------------------------------------------------------------
1. `limit_ranges.ang_vel_z = (-0.2, 0.2)`. The yaw command envelope was capped at
   0.2 rad/s. MM1 measured the deployed policy at ~0.000 rad/s for commands of
   ±0.3, ±0.6 and ±1.0 -- every one of those is outside anything it ever saw.
2. Worse: `CurriculumCfg` registers only `lin_vel_cmd_levels`, and that function
   expands `lin_vel_x` and `lin_vel_y` and **never touches `ang_vel_z`**. The
   sibling `ang_vel_cmd_levels` exists in the same module and **is referenced from
   nowhere in the repository** -- dead code. So the yaw command range stayed pinned
   at its *initial* value of (-0.1, 0.1) for the whole of training.

The policy was never asked to turn faster than 0.1 rad/s. MM1 spent six hours
eliminating obs layout, command clamps, hands, friction, restitution, armature and
Nav2 before this config was read closely enough. That is the finding.

WHAT CHANGES
------------
* `limit_ranges` widened to Mohammed's MM1b spec: vx [-0.6, 1.0], vy [-0.5, 0.5],
  wz [-1.0, 1.0].
* `ang_vel_cmd_levels` REGISTERED in the curriculum, so yaw actually expands.
* A command sampler weighted toward the three regimes MM1 measured as broken:
  in-place rotation, lateral, and reverse. Uniform sampling over a box makes pure
  in-place rotation vanishingly rare -- it is the thin slice where both linear
  components are near zero -- which is exactly the regime Nav2's Spin recovery
  needs and the twin cannot do (MM2).

WHAT DOES NOT CHANGE
--------------------
Reward terms and weights, observations, action scale, terrain, and the friction
randomisation of [0.3, 1.0] that the MM1b pre-flight found (evidence/MM1b/
twin_vs_training.md). The twin should live inside that distribution rather than
pinning friction to either end of it.
"""
from __future__ import annotations

import gymnasium as gym
import torch
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.utils import configclass

from isaaclab.envs.mdp import UniformVelocityCommand
from unitree_rl_lab.tasks.locomotion import mdp
from unitree_rl_lab.tasks.locomotion.mdp.commands.velocity_command import (
    UniformLevelVelocityCommandCfg,
)

# The upstream package directory is literally named "29dof", which is not a valid
# Python identifier, so it cannot be reached with import syntax. Upstream only ever
# refers to it through gym's entry-point strings, which go via importlib.
import importlib  # noqa: E402

_base = importlib.import_module(
    "unitree_rl_lab.tasks.locomotion.robots.g1.29dof.velocity_env_cfg")
RobotEnvCfg = _base.RobotEnvCfg

# Mohammed's MM1b envelope is vx (-0.6, 1.0), vy (-0.5, 0.5), wz (-1.0, 1.0).
# WIDENING ALL THREE AT ONCE DIVERGES. The bisect in evidence/MM1b/BISECT.md is
# unambiguous: on the identical asset and physics, turning the widened ranges back
# to upstream's takes base_linear_velocity from -4031 to -32. So the envelope is
# widened in the axis the defect is actually in -- YAW -- and the linear axes keep
# upstream's limits for this run. Yaw is what MM1 measured at ~0.000 rad/s, what
# blocks Nav2's Spin recovery, and what MM2's nav gate is waiting on; lateral and
# reverse are a second, separate problem that a second run can widen once this one
# is known to converge. Widening everything and getting a policy that trains to
# nothing would deliver neither.
LIMIT_VX = (-0.5, 1.0)          # upstream; Mohammed's (-0.6, 1.0) deferred
LIMIT_VY = (-0.3, 0.3)          # upstream; Mohammed's (-0.5, 0.5) deferred
LIMIT_WZ = (-1.0, 1.0)          # MM1b: the axis the defect is in

# Fractions of each resample forced into a regime. They are disjoint and drawn
# per-env, so the remainder stays plain uniform and the policy does not lose the
# ordinary forward walk it is currently good at (MM1 §3: N@0.2 at 2.4 % error).
FRAC_TURN_IN_PLACE = 0.25
FRAC_LATERAL = 0.15
FRAC_REVERSE = 0.10
MIN_TURN_RATE = 0.3          # a "turn" command below this teaches nothing


class WeightedVelocityCommand(UniformVelocityCommand):
    """Uniform sampling, then a weighted push into the under-represented regimes."""

    def _resample_command(self, env_ids):
        super()._resample_command(env_ids)
        if len(env_ids) == 0:
            return
        n = len(env_ids)
        dice = torch.rand(n, device=self.device)
        r = torch.empty(n, device=self.device)

        # 1. In-place rotation: zero BOTH linear components, force a real yaw rate.
        turn = dice < FRAC_TURN_IN_PLACE
        if turn.any():
            idx = env_ids[turn]
            self.vel_command_b[idx, 0] = 0.0
            self.vel_command_b[idx, 1] = 0.0
            lo, hi = self.cfg.ranges.ang_vel_z
            mag = torch.empty(int(turn.sum()), device=self.device).uniform_(
                min(MIN_TURN_RATE, abs(hi)), max(abs(lo), abs(hi)))
            sign = torch.where(
                torch.rand(int(turn.sum()), device=self.device) < 0.5, -1.0, 1.0)
            self.vel_command_b[idx, 2] = mag * sign

        # 2. Pure lateral: zero forward, force a sideways command.
        lat = (dice >= FRAC_TURN_IN_PLACE) & (dice < FRAC_TURN_IN_PLACE + FRAC_LATERAL)
        if lat.any():
            idx = env_ids[lat]
            self.vel_command_b[idx, 0] = 0.0
            lo, hi = self.cfg.ranges.lin_vel_y
            m = torch.empty(int(lat.sum()), device=self.device).uniform_(
                0.0, max(abs(lo), abs(hi)))
            s = torch.where(
                torch.rand(int(lat.sum()), device=self.device) < 0.5, -1.0, 1.0)
            self.vel_command_b[idx, 1] = m * s

        # 3. Reverse: force the x command negative.
        lo_f = FRAC_TURN_IN_PLACE + FRAC_LATERAL
        rev = (dice >= lo_f) & (dice < lo_f + FRAC_REVERSE)
        if rev.any():
            idx = env_ids[rev]
            lo, _ = self.cfg.ranges.lin_vel_x
            self.vel_command_b[idx, 0] = torch.empty(
                int(rev.sum()), device=self.device).uniform_(lo, 0.0)

        # Standing envs must stay standing; the parent set that flag already.
        _ = r


@configclass
class WeightedVelocityCommandCfg(UniformLevelVelocityCommandCfg):
    class_type: type = WeightedVelocityCommand


import os as _os

# FEROX_MM1B_USD lets the A/B run the bare 29-DoF G1 against the twin without
# editing this file, so "are the hands the cause?" is one env var, not a diff.
TWIN_USD = _os.environ.get(
    "FEROX_MM1B_USD",
    "/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p_lockedhands.usd")

# The locked-hands variant (tools/build_locked_hands_usd.py) keeps every link,
# its geometry, mass and inertia, and converts the 40 finger joints to FIXED. Mass
# is preserved exactly -- 35.004757 kg, 0.0000 % -- so the policy learns against the
# real hand weight with none of the finger dynamics that made the first runs
# explode. With it, the articulation exposes 29 DOF and needs no hand actuator
# group and no joint restriction: what is left IS the body.
# Matches every hands-fixed variant, not one filename. Keying on the literal
# "lockedhands" meant g1_dex5_1p_trainhands.usd was treated as articulated, so the
# dex5 actuator group was applied to 40 joints that no longer exist and the run
# died listing them.
LOCKED_HANDS = any(k in TWIN_USD for k in ("lockedhands", "trainhands"))
ARTICULATED_HANDS = ("dex5" in TWIN_USD) and not LOCKED_HANDS


# The 29 body joints, expressed the way upstream expresses them: the union of the
# four Unitree motor groups' regexes. Isaac Lab resolves these against the
# articulation, so the hands are excluded by construction rather than by index --
# RULE-HAND-NAME. The pre-flight asserts the resolved count is 29.
BODY_JOINT_EXPRS = [
    ".*_hip_pitch_.*", ".*_hip_yaw_.*", "waist_yaw_joint",
    ".*_hip_roll_.*", ".*_knee_.*",
    ".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_roll.*", ".*_ankle_.*",
    "waist_roll_joint", "waist_pitch_joint",
    ".*_wrist_pitch.*", ".*_wrist_yaw.*",
]


# Mohammed's MM1b envelope.
LIMIT_VX = (-0.6, 1.0)
LIMIT_VY = (-0.5, 0.5)
LIMIT_WZ = (-1.0, 1.0)

# Fractions of each resample forced into a regime. They are disjoint and drawn
# per-env, so the remainder stays plain uniform and the policy does not lose the
# ordinary forward walk it is currently good at (MM1 §3: N@0.2 at 2.4 % error).
FRAC_TURN_IN_PLACE = 0.25
FRAC_LATERAL = 0.15
FRAC_REVERSE = 0.10
MIN_TURN_RATE = 0.3          # a "turn" command below this teaches nothing


class WeightedVelocityCommand(UniformVelocityCommand):
    """Uniform sampling, then a weighted push into the under-represented regimes."""

    def _resample_command(self, env_ids):
        super()._resample_command(env_ids)
        if len(env_ids) == 0:
            return
        n = len(env_ids)
        dice = torch.rand(n, device=self.device)
        r = torch.empty(n, device=self.device)

        # 1. In-place rotation: zero BOTH linear components, force a real yaw rate.
        turn = dice < FRAC_TURN_IN_PLACE
        if turn.any():
            idx = env_ids[turn]
            self.vel_command_b[idx, 0] = 0.0
            self.vel_command_b[idx, 1] = 0.0
            lo, hi = self.cfg.ranges.ang_vel_z
            mag = torch.empty(int(turn.sum()), device=self.device).uniform_(
                min(MIN_TURN_RATE, abs(hi)), max(abs(lo), abs(hi)))
            sign = torch.where(
                torch.rand(int(turn.sum()), device=self.device) < 0.5, -1.0, 1.0)
            self.vel_command_b[idx, 2] = mag * sign

        # 2. Pure lateral: zero forward, force a sideways command.
        lat = (dice >= FRAC_TURN_IN_PLACE) & (dice < FRAC_TURN_IN_PLACE + FRAC_LATERAL)
        if lat.any():
            idx = env_ids[lat]
            self.vel_command_b[idx, 0] = 0.0
            lo, hi = self.cfg.ranges.lin_vel_y
            m = torch.empty(int(lat.sum()), device=self.device).uniform_(
                0.0, max(abs(lo), abs(hi)))
            s = torch.where(
                torch.rand(int(lat.sum()), device=self.device) < 0.5, -1.0, 1.0)
            self.vel_command_b[idx, 1] = m * s

        # 3. Reverse: force the x command negative.
        lo_f = FRAC_TURN_IN_PLACE + FRAC_LATERAL
        rev = (dice >= lo_f) & (dice < lo_f + FRAC_REVERSE)
        if rev.any():
            idx = env_ids[rev]
            lo, _ = self.cfg.ranges.lin_vel_x
            self.vel_command_b[idx, 0] = torch.empty(
                int(rev.sum()), device=self.device).uniform_(lo, 0.0)

        # Standing envs must stay standing; the parent set that flag already.
        _ = r


@configclass
class WeightedVelocityCommandCfg(UniformLevelVelocityCommandCfg):
    class_type: type = WeightedVelocityCommand


# (the TWIN_USD definition lives at the top of this file, driven by
#  FEROX_MM1B_USD. A second assignment used to sit here and silently
#  clobbered it, so every run loaded the articulated asset no matter
#  what was configured -- the smoke reported 69 actions while the USD
#  on disk had 29.)


# (_ordered_joint_names was removed: it was dead code, and this campaign has
#  already been bitten once by a function that existed and was never called.)

@configclass
class FeroxG1VelocityV2Cfg(RobotEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # 0. train on the TWIN, not the bare 29-DoF model: hands attached, real
        #    mass (35.005 kg). The upstream cfg points at UNITREE_MODEL_DIR, which
        #    ships as the literal placeholder "/tmp/unitree_model"; pointing the
        #    spawn at the twin USD explicitly is auditable, where staging files at
        #    a magic path is not. The hand joints need their own actuator group or
        #    40 of the 69 DOF arrive undriven -- isaac/twin/isaaclab/g1_dex5.py
        #    already carries that group, built at DT7 and named BY JOINT (C-14).
        import sys

        sys.path.insert(0, "/workspace/ferox_isaac")
        from twin.isaaclab.g1_dex5 import ACTUATORS as TWIN_ACTUATORS

        self.scene.robot.spawn.usd_path = TWIN_USD
        _hand = TWIN_ACTUATORS.get("dex5_1p") if ARTICULATED_HANDS else None
        if _hand is not None:
            from isaaclab.actuators import ImplicitActuatorCfg

            # The hands are ballast for this task, so they are damped hard rather
            # than left as a lightly-held 40-joint chain. With the DT7 gains
            # (stiffness 20, damping 2, armature 0.001) the first runs exploded:
            # base |vz| ~ 22 m/s and |w| ~ 206 rad/s inferred from the
            # base_linear_velocity / base_angular_velocity penalties, which
            # dominated every other reward term by five orders of magnitude. Light,
            # stiffly-coupled links at a 200 Hz step are a classic stiff-ODE
            # blow-up. Nothing here reaches the deployed hand controller: run.py
            # drives the Dex5 from its own limits, and this group only exists so the
            # 40 joints are not undriven during a LOCOMOTION retrain.
            _hand = dict(_hand)
            _hand["stiffness"] = 50.0
            _hand["damping"] = 10.0
            _hand["armature"] = 0.01
            self.scene.robot.actuators["dex5_1p"] = ImplicitActuatorCfg(**_hand)

        # 0b. THE HANDS ARE MASS, NOT ACTUATORS.
        #     The upstream action term is `joint_names=[".*"]` and the observation
        #     terms take every joint, so spawning the 69-DoF twin silently produced
        #     an env with **1080-dim observations and 69 actions**. The deployed
        #     contract is 480 obs (96/step x history 5) and 29 actions -- `run.py`
        #     could not have loaded the result. Hours of GPU time would have bought
        #     an undeployable policy, and it would only have shown up at export.
        #     Caught by the pre-flight, which is why the pre-flight exists.
        #
        #     "Twin USD, hands on, real mass" means the hands are PRESENT --
        #     inertia, collision, 35.005 kg total -- not that a locomotion policy
        #     drives 40 finger joints. Restricting obs and actions to the 29 body
        #     joints keeps the deployment contract exactly as it is.
        from isaaclab.managers import SceneEntityCfg

        # CONCRETE names, not the regexes: SceneEntityCfg resolves joint_names to
        # joint_ids and then checks the two agree, so 13 regexes resolving to 29
        # joints fails as "not consistent". BODY_JOINTS_SIM is the 29 body joints in
        # Isaac's own DOF order, derived at DT7 from deploy.yaml's joint_ids_map and
        # asserted by the twin tests -- and it names them, per RULE-HAND-NAME.
        from twin.isaaclab.g1_dex5 import BODY_JOINTS_SIM

        if len(BODY_JOINTS_SIM) != 29:
            raise RuntimeError(f"expected 29 body joints, got {len(BODY_JOINTS_SIM)}")
        self.actions.JointPositionAction.joint_names = (
            list(BODY_JOINTS_SIM) if ARTICULATED_HANDS else [".*"])
        for grp in ("policy", "critic"):
            g = getattr(self.observations, grp, None)
            if g is None:
                continue
            for term in ("joint_pos_rel", "joint_vel_rel"):
                t = getattr(g, term, None)
                if t is not None:
                    t.params = dict(t.params or {})
                    # A FRESH cfg per term: SceneEntityCfg.resolve() mutates the
                    # object it is given, so one shared instance would carry the
                    # first term's resolved ids into the second.
                    if ARTICULATED_HANDS:
                        t.params["asset_cfg"] = SceneEntityCfg(
                            "robot", joint_names=list(BODY_JOINTS_SIM))

        # 1. widen the envelope
        lim = self.commands.base_velocity.limit_ranges
        lim.lin_vel_x = LIMIT_VX
        lim.lin_vel_y = LIMIT_VY
        lim.ang_vel_z = LIMIT_WZ

        # 2. wire in the yaw curriculum upstream defined and never registered
        self.curriculum.ang_vel_cmd_levels = CurrTerm(mdp.ang_vel_cmd_levels)

        # 3. weighted sampling toward turn-in-place / lateral / reverse.
        #    FEROX_MM1B_VANILLA=1 turns off items 1-3 so the ONLY difference from
        #    upstream is the asset. That is the control experiment for "is the
        #    divergence mine or the twin's?", and it should be run before blaming
        #    either.
        # Bisect switches: each of the three MM1b command changes can be turned
        # off on its own, so "which of my changes diverges" is three 2-minute
        # smokes rather than a guess. FEROX_MM1B_OFF=ranges,curriculum,sampler
        _off = {t.strip() for t in _os.environ.get("FEROX_MM1B_OFF", "").split(",")
                if t.strip()}

        # FEROX_MM1B_REFERENCE=1 reproduces the recipe that actually produced the
        # deployed g1_omni weights, read from the checkpoint's own
        # params/env.yaml -- NOT upstream's defaults, which differ:
        #   ranges       lin_vel (-0.1, 0.1)  ang_vel_z (-1.0, 1.0)   <- yaw pinned
        #   limit_ranges lin_vel (-0.6, 1.0)/(-0.5, 0.5)  ang_vel_z (-1.0, 1.0)
        #   curriculum   terrain_levels, lin_vel_cmd_levels only
        # The patch set yaw straight to the limit instead of curriculum-expanding
        # it, which is why the unregistered ang_vel_cmd_levels never mattered.
        if _os.environ.get("FEROX_MM1B_REFERENCE", "0") == "1":
            lim.lin_vel_x, lim.lin_vel_y, lim.ang_vel_z = \
                (-0.6, 1.0), (-0.5, 0.5), (-1.0, 1.0)
            rng = self.commands.base_velocity.ranges
            rng.lin_vel_x, rng.lin_vel_y = (-0.1, 0.1), (-0.1, 0.1)
            rng.ang_vel_z = (-1.0, 1.0)
            if hasattr(self.curriculum, "ang_vel_cmd_levels"):
                delattr(self.curriculum, "ang_vel_cmd_levels")
            # ...unless the caller explicitly asks for the weighting on top, which
            # is what step (c) is: the v1 recipe PLUS in-place/lateral weighting.
            if _os.environ.get("FEROX_MM1B_SAMPLER", "0") != "1":
                _off.add("sampler")
            print("[MM1b] REFERENCE recipe (from the checkpoint's env.yaml): "
                  "yaw pinned at +-1.0, linear under curriculum, no yaw curriculum, "
                  + ("WITH the in-place/lateral weighted sampler."
                     if _os.environ.get("FEROX_MM1B_SAMPLER", "0") == "1"
                     else "no weighted sampler."), flush=True)
        if "ranges" in _off:
            lim.lin_vel_x, lim.lin_vel_y, lim.ang_vel_z = \
                (-0.5, 1.0), (-0.3, 0.3), (-0.2, 0.2)
        if "curriculum" in _off and hasattr(self.curriculum, "ang_vel_cmd_levels"):
            delattr(self.curriculum, "ang_vel_cmd_levels")
        if _off:
            print(f"[MM1b] bisect: disabled {sorted(_off)}", flush=True)

        if _os.environ.get("FEROX_MM1B_VANILLA", "0") == "1":
            print("[MM1b] VANILLA: upstream ranges, upstream sampler, no yaw "
                  "curriculum. Only the asset differs.", flush=True)
            lim.lin_vel_x = (-0.5, 1.0)
            lim.lin_vel_y = (-0.3, 0.3)
            lim.ang_vel_z = (-0.2, 0.2)
            if hasattr(self.curriculum, "ang_vel_cmd_levels"):
                delattr(self.curriculum, "ang_vel_cmd_levels")
        elif _os.environ.get("FEROX_MM1B_SAMPLER", "0") == "1" and "sampler" not in _off:
            # OFF by default: the bisect showed the weighted sampler is the second
            # largest destabiliser (-139/-227 with it off against -4031/-1757 with
            # everything on). The registered ang_vel_cmd_levels curriculum now does
            # the job it was added for -- it walks the yaw command out to +-1.0 as
            # the policy earns it, which is gentler than forcing hard turns on an
            # untrained policy from step zero.
            self.commands.base_velocity.class_type = WeightedVelocityCommand

        # 4. PhysX contact-patch buffer. The first 6000-iteration run DIVERGED --
        #    mean episode length 1.00, value loss 4.5e7 -- and the cause was in the
        #    log from the first seconds: 1375 x "Patch buffer overflow detected,
        #    please increase its size to at least 615685 in the scene desc". 2048
        #    envs of a 69-DoF robot whose Dex5 hands carry 40 jointed collision
        #    meshes each blow straight past the default 163840. With the buffer
        #    overflowing, contacts are silently wrong, the robots explode, and
        #    every episode terminates on step 1. Training on broken physics for 47
        #    minutes produced a policy of exactly no value.
        #    Sized for the LOCKED-hands asset. The articulated one needed
        #    615685 patches and 2**21 headroom, and that much GPU buffer put the
        #    process at 15.14 GiB of a 15.57 GiB card -- PPO then OOMed trying to
        #    allocate 24 MiB. With the fingers fixed the contact demand drops
        #    sharply, so the buffers come back down and leave room for the policy.
        self.sim.physx.gpu_max_rigid_patch_count = 2**19      # 524,288
        self.sim.physx.gpu_found_lost_pairs_capacity = 2**20
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 2**20

        # 5. Physics rate. FEROX_MM1B_PHYS_HZ bisects the integration step while
        #    HOLDING the 50 Hz control rate the deployment contract fixes: the
        #    decimation is recomputed, never the policy's own rate. Locking the
        #    hands cut the blow-up but did not cure it, so per Mohammed the hands
        #    were a symptom and dt is the next suspect.
        _hz = float(_os.environ.get("FEROX_MM1B_PHYS_HZ", "200"))
        _ctrl_hz = 50.0
        self.sim.dt = 1.0 / _hz
        self.decimation = max(1, int(round(_hz / _ctrl_hz)))
        print(f"[MM1b] physics {_hz:.0f} Hz, decimation {self.decimation} "
              f"-> control {_hz / self.decimation:.1f} Hz", flush=True)

        # 16 GB box: 2048 envs is Mohammed's cap for this run.
        self.scene.num_envs = min(self.scene.num_envs, 2048)


@configclass
class FeroxG1VelocityV2PlayCfg(FeroxG1VelocityV2Cfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges


gym.register(
    id="Ferox-G1-29dof-Velocity-v2",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}:FeroxG1VelocityV2Cfg",
        "play_env_cfg_entry_point": f"{__name__}:FeroxG1VelocityV2PlayCfg",
        "rsl_rl_cfg_entry_point":
            "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    },
)
