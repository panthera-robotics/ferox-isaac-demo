# ruff: noqa: E402
"""
Isaac Sim robot simulation with ROS2 integration.
Supports Go2 (quadruped) and G1 (humanoid) robots.

Control robot velocity:
  ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3, y: 0.0}, angular: {z: 0.2}}"

Examples
--------
  python run.py --robot_type go2   # Run Go2 quadruped (default)
  python run.py --robot_type g1    # Run G1 humanoid
"""

import os

from isaacsim import SimulationApp

# tools/film.py imports this module to reuse G1VelocityPolicy, so that a
# locomotion clip shows the real policy rather than a scripted joint swing. It has
# already launched the app by then, and creating a second SimulationApp aborts the
# process.
#
# The switch is an explicit environment variable and NOT a "is kit already
# running?" probe. A probe would be one wrong answer away from leaving
# simulation_app as None on the normal entry path and breaking the shipped sim,
# and it would fail silently. With the variable, the default path is exactly what
# it was before, and only a caller that knowingly owns the app opts out.
if os.environ.get("FEROX_REUSE_KIT_APP", "0") == "1":
    simulation_app = None
else:
    simulation_app = SimulationApp(
        {"renderer": "RaytracedLighting", "headless": False}
    )

import argparse
import logging
import os
import sys
import time
from typing import Optional, Tuple

import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
import sim_utils as ros_utils
import viewport_follow  # PANTHERA: flag-gated viewport-follow for x11grab capture
import yaml
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.robot.policy.examples.controllers.config_loader import (
    get_action,
    get_observations,
    parse_env_config,
)
from isaacsim.storage.native import get_assets_root_path

DEFAULT_GO2_POLICY_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "checkpoints", "go2"
)
DEFAULT_G1_POLICY_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "checkpoints", "g1"
)

ROBOT_GO2 = "go2"
ROBOT_G1 = "g1"

G1_INIT_HEIGHT = 1.05
G1_HISTORY_LENGTH = 5
CMD_VEL_TIMEOUT = 0.5  # seconds – stop if no new /cmd_vel received

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# World selection — INDEPENDENT of robot selection.
#
# SIM_WORLD picks the environment USD loaded under WORLD_ENV_PRIM. Default is
# "office" (NVIDIA's built-in Office scene). The robot (ROBOT=go2|g1) is chosen
# separately, so any robot loads into any world.
#
# `usd` is RELATIVE to the Isaac assets root resolved by get_assets_root_path()
# — the SAME mechanism the robot-USD fallback already uses (local pack /
# nucleus / S3). `spawn` is the open-floor (x, y) and yaw (radians) the robot is
# placed at; the z standing height comes from the robot (env.yaml init height),
# NOT the world, so one world entry works for every robot.
#
# Adding a world = ONE line here:  name -> {usd, spawn: {xy, yaw}}.
WORLD_ENV_PRIM = "/World/Env"


TRAIN_ARMATURE = 0.01


# C-39. Four links carry the URDF importer's default mass of exactly 1.000 kg because
# they have NO <inertial> block in any of the three G1 URDFs on this box -- checked, not
# assumed. Together they are exactly 4.000000 kg, which is the whole of the 39.004757 vs
# 35.004757 discrepancy, and three of the four sit high (torso, head camera, lidar) so
# the phantom mass lands where it damages a balancer most.
#
# Replaced with the real components' published masses. These are NOT ground truth from
# the URDF -- the URDF has nothing to give -- so they are stated here as the source of
# record and the total is asserted after applying them.
SENSOR_MASS_KG = {
    "d435_link": 0.072,      # Intel RealSense D435, 72 g
    "mid360_link": 0.265,    # Livox MID-360, 265 g
    "imu_in_torso": 0.005,   # a board-level IMU
    "imu_in_pelvis": 0.005,
}


def _apply_sensor_masses(robot) -> None:
    """Replace placeholder sensor-link masses, read back, and assert the total."""
    view = robot._articulation_view
    pv = getattr(view, "_physics_view", None)
    if pv is None or not hasattr(pv, "get_masses"):
        print("[TWIN] sensor-mass fix: no masses API; NOT applied", flush=True)
        return
    import numpy as _np
    names = list(view.body_names)
    M = _np.asarray(pv.get_masses()).reshape(-1)[:len(names)].astype(_np.float32)
    before = float(M.sum())
    hit = {}
    for i, n in enumerate(names):
        if n in SENSOR_MASS_KG:
            hit[n] = (float(M[i]), SENSOR_MASS_KG[n])
            M[i] = SENSOR_MASS_KG[n]
    if not hit:
        print("[TWIN] sensor-mass fix: none of the four links present", flush=True)
        return
    # set_masses on this build's view wants (data, indices) -- the indices argument is
    # positional and required, and omitting it fails with a TypeError that reads like an
    # API-absent error rather than a signature one.
    import warp as _wp  # noqa: F401  (present wherever the physx view is)
    applied = False
    for call in (lambda: pv.set_masses(M.reshape(1, -1), _np.array([0], dtype=_np.int32)),
                 lambda: pv.set_masses(M.reshape(1, -1), None),
                 lambda: pv.set_masses(M.reshape(1, -1))):
        try:
            call()
            applied = True
            break
        except Exception as exc:
            last = exc
    if not applied:
        print(f"[TWIN] sensor-mass fix FAILED to apply: {last!r}", flush=True)
        return
    got = _np.asarray(pv.get_masses()).reshape(-1)[:len(names)]
    for n, (old_m, new_m) in hit.items():
        i = names.index(n)
        if abs(float(got[i]) - new_m) > 1e-6:
            raise RuntimeError(f"sensor mass on {n} read back {got[i]}, wanted {new_m}")
        print(f"[TWIN] sensor mass {n}: {old_m:.6f} -> {float(got[i]):.6f} kg", flush=True)
    print(f"[TWIN] total mass {before:.6f} -> {float(got.sum()):.6f} kg "
          f"(removed {before - float(got.sum()):.6f} kg of placeholder sensor mass)",
          flush=True)


# Grasp v6. The contact route in THIS build: RigidContactView is absent and the
# articulation has no get_net_contact_forces, so finger contact has to come from
# ContactSensor prims -- and those must EXIST BEFORE the sim starts stepping, which is
# why this runs at world setup and not from the MM5 pipeline.
# (get_link_incoming_joint_force is present but is a joint reaction, not contact; it was
# misread as ground contact once already and is deliberately not used here.)
DEX5_TIP_LINKS = ("Link_14R", "Link_24R", "Link_34R", "Link_44R", "Link_54R")


def _collision_child(stage, link_path: str):
    """First descendant of `link_path` carrying CollisionAPI, or None.

    A ContactSensor must be parented to a prim that HAS collision, and the link Xform
    does not -- URDF import puts the collider under `<link>/collisions/mesh_N/...` and
    instances it. Parenting to the link itself is rejected at initialize() with
    "needs to be created under another prim that has collision api enabled on", which
    happens well after creation appears to have succeeded.
    """
    # PrimRange.Stage + a path-prefix filter, NOT PrimRange(prim, predicate): the
    # per-prim form does not descend into instance proxies, so it returns nothing for
    # links whose geometry the URDF importer instanced -- which is all of them. The
    # stage form found 42 hand collision prims where the per-prim form found 0.
    from pxr import Usd, UsdPhysics
    pref = link_path.rstrip("/") + "/"
    for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        sp = p.GetPath().pathString
        if not sp.startswith(pref):
            continue
        if p.HasAPI(UsdPhysics.CollisionAPI):
            return sp
    return None


def _make_hand_contact_sensors(robot_root: str, stage=None) -> list:
    made = []
    try:
        from isaacsim.sensors.physics import ContactSensor
    except Exception as exc:
        print(f"[TWIN] hand contact sensors unavailable: {exc!r}", flush=True)
        return made
    for link in DEX5_TIP_LINKS:
        parent = (_collision_child(stage, f"{robot_root}/{link}") if stage is not None
                  else None)
        if parent is None:
            print(f"[TWIN] no collision prim under {link}; no sensor", flush=True)
            continue
        path = f"{parent}/contact_sensor"
        try:
            ContactSensor(prim_path=path, name=f"cs_{link}", frequency=60,
                          translation=None, min_threshold=0.0, max_threshold=1e7,
                          radius=-1.0)
            made.append((link, path))
        except Exception as exc:
            print(f"[TWIN] contact sensor on {link} failed: {exc!r}", flush=True)
    print(f"[TWIN] hand contact sensors created: {len(made)}/{len(DEX5_TIP_LINKS)} "
          f"{[m[0] for m in made]}", flush=True)
    return made


def _apply_joint_friction(stage, robot_root: str, value: float) -> None:
    """Set PhysX joint FRICTION on every revolute joint under the robot.

    C-39, 1b(2).  The reference MuJoCo model the SONIC deploy was validated against
    (`g1_29dof_old.xml`) gives every actuated joint `frictionloss="0.2"` -- 0.1 on the
    wrists -- alongside `damping="0.05"` and `armature="0.01"`.  The twin matches the
    armature (TWIN_ARMATURE) but the URDF importer wrote no dry friction at all, so
    every joint in the twin is free where the reference joint has a 0.2 Nm breakaway.
    Across 29 joints that is real quasi-static holding torque that the reference robot
    has and this one does not.

    Read back and counted, for the same reason armature is: a silent no-op here reads
    exactly like "joint friction was not the problem".
    """
    from pxr import PhysxSchema, Usd, UsdPhysics

    root = stage.GetPrimAtPath(robot_root)
    if not root or not root.IsValid():
        raise RuntimeError(f"joint friction: {robot_root} not in stage")
    n = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        api = PhysxSchema.PhysxJointAPI.Apply(prim)
        # The schema accessor is not present on every Isaac build; the underlying
        # attribute is. Fall back to it rather than taking the whole sim down --
        # the first attempt at this raised during stage setup and Isaac exited at
        # 12.5 s with no traceback, which costs a run to learn nothing.
        try:
            api.CreateJointFrictionAttr().Set(value)
            got = api.GetJointFrictionAttr().Get()
        except AttributeError:
            from pxr import Sdf
            at = prim.CreateAttribute("physxJoint:jointFriction", Sdf.ValueTypeNames.Float)
            at.Set(value)
            got = at.Get()
        # float32 storage: 0.2 reads back as 0.20000000298023224, so the tolerance
        # has to be a float32 epsilon and not the 1e-9 used for armature. The first
        # attempt used 1e-9, rejected a CORRECT write, and cost a run that then had
        # to be discarded -- the read-back guard has to be right or it is worse than
        # no guard at all.
        if got is None or abs(float(got) - value) > 1e-6:
            raise RuntimeError(f"joint friction on {prim.GetPath()} read back {got}")
        n += 1
    if n == 0:
        raise RuntimeError(f"joint friction: no revolute joints under {robot_root}")
    print(f"[TWIN] joint friction {value} applied and read back on {n} joints",
          flush=True)


def _apply_training_armature(stage, robot_root: str, value: float) -> None:
    """Set PhysX joint armature on every revolute joint under the robot.

    Read back and counted, because a silent no-op here reads exactly like
    "armature was not the problem".
    """
    from pxr import PhysxSchema, Usd, UsdPhysics

    root = stage.GetPrimAtPath(robot_root)
    if not root or not root.IsValid():
        raise RuntimeError(f"armature: {robot_root} not in stage")
    n = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        api = PhysxSchema.PhysxJointAPI.Apply(prim)
        api.CreateArmatureAttr().Set(value)
        got = api.GetArmatureAttr().Get()
        if got is None or abs(float(got) - value) > 1e-9:
            raise RuntimeError(f"armature on {prim.GetPath()} read back {got}")
        n += 1
    if n == 0:
        raise RuntimeError(f"armature: no revolute joints found under {robot_root}")
    print(f"[TWIN] armature {value} applied and read back on {n} joints", flush=True)


TRAIN_STATIC_FRICTION = 1.0
TRAIN_DYNAMIC_FRICTION = 1.0
TRAIN_RESTITUTION = 0.0
TRAIN_FRICTION_COMBINE = "multiply"


def _apply_training_contact_material(stage, robot_root: str) -> None:
    """Bind the policy's TRAINING contact material to the robot and the world.

    Bound to BOTH surfaces on purpose. `friction_combine_mode` is "multiply" in
    training, so a material on only one side still multiplies against the PhysX
    default on the other and lands at half the intended figure -- which is the
    very mistake this exists to correct.

    Read back after binding: a silent no-op here would look exactly like "the
    friction hypothesis was wrong", which is the most expensive kind of wrong.
    """
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics, UsdShade

    path = Sdf.Path("/World/Physics_Materials/twin_training_material")
    if not stage.GetPrimAtPath(path):
        stage.DefinePrim(path, "Material")
    mat_prim = stage.GetPrimAtPath(path)
    mat = UsdPhysics.MaterialAPI.Apply(mat_prim)
    mat.CreateStaticFrictionAttr().Set(TRAIN_STATIC_FRICTION)
    mat.CreateDynamicFrictionAttr().Set(TRAIN_DYNAMIC_FRICTION)
    mat.CreateRestitutionAttr().Set(TRAIN_RESTITUTION)
    px = PhysxSchema.PhysxMaterialAPI.Apply(mat_prim)
    px.CreateFrictionCombineModeAttr().Set(TRAIN_FRICTION_COMBINE)
    px.CreateRestitutionCombineModeAttr().Set(TRAIN_FRICTION_COMBINE)

    bound = []
    for target in (robot_root, WORLD_ENV_PRIM):
        prim = stage.GetPrimAtPath(target)
        if not prim or not prim.IsValid():
            logger.warning("[contact] %s not in stage, not bound", target)
            continue
        api = UsdShade.MaterialBindingAPI.Apply(prim)
        api.Bind(UsdShade.Material(mat_prim), UsdShade.Tokens.weakerThanDescendants,
                 "physics")
        bound.append(target)

    got = (UsdPhysics.MaterialAPI(mat_prim).GetStaticFrictionAttr().Get(),
           UsdPhysics.MaterialAPI(mat_prim).GetDynamicFrictionAttr().Get(),
           UsdPhysics.MaterialAPI(mat_prim).GetRestitutionAttr().Get())
    if got != (TRAIN_STATIC_FRICTION, TRAIN_DYNAMIC_FRICTION, TRAIN_RESTITUTION):
        raise RuntimeError(f"contact material read back {got}, refusing to continue")
    print(f"[TWIN] contact material static={got[0]} dynamic={got[1]} "
          f"restitution={got[2]} combine={TRAIN_FRICTION_COMBINE} -> {bound}",
          flush=True)


DEFAULT_SIM_WORLD = "office"

SIM_WORLDS = {
    # NVIDIA built-in Office — the default world.
    "office": {
        "usd": "/Isaac/Environments/Office/office.usd",
        "spawn": {"xy": (0.0, 0.0), "yaw": 0.0},
    },
    # The original world (Simple Warehouse) preserved verbatim, so nothing is
    # lost: SIM_WORLD=dso_block_a loads exactly what the sim loaded before, at
    # the same origin spawn.
    "dso_block_a": {
        "usd": "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
        "spawn": {"xy": (0.0, 0.0), "yaw": 0.0},
    },
    # NVIDIA built-in Hospital. Origin (0,0) is the waiting room (reception
    # counter + chairs ~2 m away), so spawn instead mid-corridor on open floor:
    # ~3.3 m to each side wall, ~5 m to the south doors, ~9 m clear north to the
    # stairs. yaw ~+90deg faces down the open corridor. (Spawn found by probing
    # hospital.usd geometry; floor is flat z=0 like office/warehouse.)
    "hospital": {
        "usd": "/Isaac/Environments/Hospital/hospital.usd",
        "spawn": {"xy": (7.8, 2.0), "yaw": 1.57},
    },
    # Panthera's own lab (MM2, campaign §4.3), authored by
    # tools/build_lab_world.py. 8 x 6 m CLEAR interior centred on the origin, so
    # the walls are at x=+-4.0 and y=+-3.0. Spawn is the "home" waypoint: the
    # west side facing +x down the long axis, with ~6 m of clear floor ahead. NOT
    # (-2.8, -1.9), which is inside the shelf's footprint. Unlike the built-in worlds this one is a LOCAL
    # path, so it is resolved relative to the isaac/ tree rather than the Nucleus
    # asset root.
    "panthera_lab": {
        "usd": "@LOCAL@/assets/worlds/panthera_lab/panthera_lab.usd",
        "spawn": {"xy": (-2.6, 0.0), "yaw": 0.0},
    },
}


def _load_yaml(path: str) -> dict:
    try:
        with open(path, "r", encoding="utf-8") as file:
            return yaml.safe_load(file) or {}
    except Exception:
        return {}


def _expand_param(value, size: int, default: float) -> np.ndarray:
    if value is None:
        return np.full(size, default, dtype=np.float32)
    if isinstance(value, (int, float)):
        return np.full(size, float(value), dtype=np.float32)
    arr = np.array(value, dtype=np.float32)
    if arr.size != size:
        return np.full(size, default, dtype=np.float32)
    return arr


def _resolve_command_limits(
    deploy_cfg: dict, env_cfg: dict
) -> Tuple[np.ndarray, np.ndarray]:
    def _from_cfg(cfg: dict):
        ranges = (
            cfg.get("commands", {}).get("base_velocity", {}).get("ranges")
            if cfg
            else None
        )
        if not isinstance(ranges, dict):
            return None

        def _pair(key: str, fallback: Tuple[float, float]):
            val = ranges.get(key)
            if isinstance(val, (list, tuple)) and len(val) == 2:
                try:
                    return float(val[0]), float(val[1])
                except Exception:
                    return fallback
            return fallback

        return np.array(
            [
                _pair("lin_vel_x", (-1.0, 1.0)),
                _pair("lin_vel_y", (-1.0, 1.0)),
                _pair("ang_vel_z", (-1.0, 1.0)),
            ],
            dtype=np.float32,
        )

    pairs = _from_cfg(deploy_cfg)
    if pairs is None:
        pairs = _from_cfg(env_cfg)
    if pairs is None:
        pairs = np.array([[-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0]], dtype=np.float32)

    cmd_min = pairs[:, 0]
    cmd_max = pairs[:, 1]
    return cmd_min, cmd_max


# Unitree name the Dex5-1P joints Yaw_11L, Roll_12L, Pitch_13L, ... and Link_41L.
# Every G1 body joint ends in "_joint", so the two sets cannot be confused -- but
# matching on the hand's own prefixes states the intent and fails loudly if Unitree
# ever renames them, rather than silently reclassifying a finger as a body joint.
_HAND_JOINT_PREFIXES = ("Yaw_", "Roll_", "Pitch_", "Link_")


def _is_hand_joint(name: str) -> bool:
    return name.startswith(_HAND_JOINT_PREFIXES)


def _resolve_usd_path(env_cfg: dict, robot_type: str = ROBOT_GO2) -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))

    if robot_type == ROBOT_G1:
        # G1_USD_OVERRIDE points the G1 at an arbitrary USD, ahead of every other rule.
        # It exists for the C-39 A/B (docs/mm/evidence/C39/): the reference MuJoCo model
        # imported unmodified through the MJCF importer has to run under the IDENTICAL
        # fork as the twin, and the only thing that may differ between the two runs is
        # the body. It is deliberately loud -- an override that picks a different robot
        # than the operator thinks would invalidate the experiment silently.
        override = os.environ.get("G1_USD_OVERRIDE", "").strip()
        if override:
            if not os.path.isfile(override):
                raise SystemExit(f"G1_USD_OVERRIDE={override!r} is not a file")
            logger.warning("G1_USD_OVERRIDE in effect -- NOT the twin asset: %s",
                           override)
            return override

        # HAND=dex5_1p selects the merged G1+Dex5 asset. It is a SEPARATE asset, not a
        # variant on the bare one, because the hands have to be present at URDF import
        # time to end up in the same PhysX articulation -- see tools/merge_dex5_urdf.py.
        # HAND=none (the default) keeps the bare-wristed robot every earlier gate ran
        # against, so a hand problem can always be bisected against it.
        hand = os.environ.get("HAND", "none").strip().lower()
        if hand not in ("none", "dex5_1p"):
            raise SystemExit(f"HAND={hand!r} unknown; expected 'none' or 'dex5_1p'")
        if hand == "dex5_1p":
            hand_usd = os.path.join(script_dir, "assets", "g1_dex5", "g1_dex5_1p.usd")
            if not os.path.isfile(hand_usd):
                raise SystemExit(f"HAND=dex5_1p but {hand_usd} is missing; run "
                                 "./scripts/12_import_g1_dex5.sh")
            logger.info("Using merged G1+Dex5-1P USD model: %s", hand_usd)
            return hand_usd

        # First priority: check for local G1 assets directory
        local_usd_path = os.path.join(script_dir, "assets", "g1", "usd", "g1.usd")
        if os.path.isfile(local_usd_path):
            logger.info("Using local G1 USD model: %s", local_usd_path)
            return local_usd_path

        # Second priority: check env.yaml usd_path
        usd_path = (
            env_cfg.get("scene", {}).get("robot", {}).get("spawn", {}).get("usd_path")
        )
        if usd_path:
            if os.path.isabs(usd_path) and os.path.isfile(usd_path):
                logger.info("Using G1 USD model from absolute path: %s", usd_path)
                return usd_path
            relative_path = os.path.join(script_dir, usd_path)
            if os.path.isfile(relative_path):
                logger.info("Using G1 USD model from relative path: %s", relative_path)
                return relative_path
            logger.warning("USD path from env.yaml not found: %s", usd_path)
            return usd_path

        # Fallback to Isaac Sim assets
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return ""
        fallback_path = assets_root_path + "/Isaac/Robots/Unitree/G1/g1.usd"
        logger.info("Using Isaac Sim default G1 USD model: %s", fallback_path)
        return fallback_path

    # First priority: check for local Go2 assets directory
    local_usd_path = os.path.join(script_dir, "assets", "go2", "usd", "go2.usd")
    if os.path.isfile(local_usd_path):
        logger.info("Using local Go2 USD model: %s", local_usd_path)
        return local_usd_path

    # Second priority: check env.yaml usd_path
    usd_path = (
        env_cfg.get("scene", {}).get("robot", {}).get("spawn", {}).get("usd_path")
    )
    if usd_path:
        # If it's an absolute path that exists, use it
        if os.path.isabs(usd_path) and os.path.isfile(usd_path):
            logger.info("Using Go2 USD model from absolute path: %s", usd_path)
            return usd_path
        # If it's a relative path, try to resolve it from the script directory
        relative_path = os.path.join(script_dir, usd_path)
        if os.path.isfile(relative_path):
            logger.info("Using Go2 USD model from relative path: %s", relative_path)
            return relative_path
        # Otherwise use as-is (might be resolved by Isaac Sim)
        logger.warning("USD path from env.yaml not found: %s", usd_path)
        return usd_path

    # Fallback to Isaac Sim assets
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return ""
    fallback_path = assets_root_path + "/Isaac/Robots/Unitree/Go2/go2.usd"
    logger.info("Using Isaac Sim default Go2 USD model: %s", fallback_path)
    return fallback_path


def _resolve_world(assets_root_path: str) -> Tuple[str, dict]:
    """Resolve SIM_WORLD to an absolute world USD path + spawn pose.

    World choice is independent of robot. The USD path is built from the SAME
    assets root the robot USD uses, then VERIFIED with omni.client.stat before
    it is referenced — so a missing/unreachable world fails loud here instead of
    silently loading an empty stage. Returns (absolute_usd_path, spawn_dict).
    """
    name = (os.environ.get("SIM_WORLD") or DEFAULT_SIM_WORLD).strip() or DEFAULT_SIM_WORLD
    entry = SIM_WORLDS.get(name)
    if entry is None:
        known = ", ".join(sorted(SIM_WORLDS))
        raise RuntimeError(
            f"Unknown SIM_WORLD='{name}'. Known worlds: {known}. "
            f"Add one with a single line in SIM_WORLDS (name -> usd + spawn)."
        )

    # A world can live in this repo rather than on the Nucleus asset server.
    # @LOCAL@ resolves against run.py's own directory, which is the isaac/ tree
    # inside the container, so the same entry works wherever that tree is mounted.
    # The stat check below is unchanged and applies to local worlds too -- a
    # missing local file must fail as loudly as a missing remote one.
    if entry["usd"].startswith("@LOCAL@"):
        world_usd = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            entry["usd"][len("@LOCAL@/"):])
    else:
        world_usd = assets_root_path + entry["usd"]
    try:
        import omni.client

        result = omni.client.stat(world_usd)[0]
        ok = result == omni.client.Result.OK
    except Exception as exc:  # stat backend / resolver failure
        raise RuntimeError(
            f"SIM_WORLD='{name}': could not stat world USD {world_usd}: {exc}"
        ) from exc
    if not ok:
        raise RuntimeError(
            f"SIM_WORLD='{name}': world USD not found / not resolvable: "
            f"{world_usd} (omni.client.stat -> {result}). "
            f"Check the assets root and network."
        )

    logger.info("[World] SIM_WORLD=%s -> %s", name, world_usd)
    # print() (not just logger.info) so the selected world is visible in
    # /tmp/sim.log — Isaac's logging swallows INFO, and the operator/health
    # check needs to see which world actually loaded.
    print(f"[World] SIM_WORLD={name} -> {world_usd}", flush=True)
    return world_usd, entry["spawn"]


def _configure_ros_utils_paths(robot_root: str, robot_type: str = ROBOT_GO2) -> None:
    """Configure ROS utils prim paths based on robot type."""
    ros_utils.GO2_STAGE_PATH = robot_root

    if robot_type == ROBOT_G1:
        base_link = f"{robot_root}/torso_link"
        ros_utils.IMU_PRIM = f"{base_link}/imu_link"
        ros_utils.CAMERA_LINK_PRIM = f"{base_link}/camera_link"
        ros_utils.REALSENSE_DEPTH_CAMERA_PRIM = (
            f"{ros_utils.CAMERA_LINK_PRIM}/realsense_depth_camera"
        )
        ros_utils.REALSENSE_RGB_CAMERA_PRIM = (
            f"{ros_utils.CAMERA_LINK_PRIM}/realsense_rgb_camera"
        )
        ros_utils.GO2_RGB_CAMERA_PRIM = f"{ros_utils.CAMERA_LINK_PRIM}/go2_rgb_camera"
        ros_utils.L1_LINK_PRIM = f"{base_link}/lidar_l1_link"
        ros_utils.L1_LIDAR_PRIM = f"{ros_utils.L1_LINK_PRIM}/lidar_l1_rtx"
        ros_utils.VELO_BASE_LINK_PRIM = f"{base_link}/velodyne_base_link"
        ros_utils.VELO_LASER_LINK_PRIM = f"{ros_utils.VELO_BASE_LINK_PRIM}/laser"
        ros_utils.VELO_LIDAR_PRIM = (
            f"{ros_utils.VELO_LASER_LINK_PRIM}/velodyne_vlp16_rtx"
        )
    else:
        ros_utils.IMU_PRIM = f"{robot_root}/base/imu_link"
        ros_utils.CAMERA_LINK_PRIM = f"{robot_root}/base/camera_link"
        ros_utils.REALSENSE_DEPTH_CAMERA_PRIM = (
            f"{ros_utils.CAMERA_LINK_PRIM}/realsense_depth_camera"
        )
        ros_utils.REALSENSE_RGB_CAMERA_PRIM = (
            f"{ros_utils.CAMERA_LINK_PRIM}/realsense_rgb_camera"
        )
        ros_utils.GO2_RGB_CAMERA_PRIM = f"{ros_utils.CAMERA_LINK_PRIM}/go2_rgb_camera"
        ros_utils.L1_LINK_PRIM = f"{robot_root}/base/lidar_l1_link"
        ros_utils.L1_LIDAR_PRIM = f"{ros_utils.L1_LINK_PRIM}/lidar_l1_rtx"
        ros_utils.VELO_BASE_LINK_PRIM = f"{robot_root}/base/velodyne_base_link"
        ros_utils.VELO_LASER_LINK_PRIM = f"{ros_utils.VELO_BASE_LINK_PRIM}/laser"
        ros_utils.VELO_LIDAR_PRIM = (
            f"{ros_utils.VELO_LASER_LINK_PRIM}/velodyne_vlp16_rtx"
        )


def _validate_policy_paths(policy_dir: str) -> Tuple[str, str, str]:
    policy_path = os.path.join(policy_dir, "exported", "policy.pt")
    env_path = os.path.join(policy_dir, "params", "env.yaml")
    deploy_path = os.path.join(policy_dir, "params", "deploy.yaml")

    missing = []
    if not os.path.isfile(policy_path):
        missing.append(policy_path)
    if not os.path.isfile(env_path):
        missing.append(env_path)

    if missing:
        for path in missing:
            carb.log_error(f"Missing required policy file: {path}")
        raise FileNotFoundError("Required policy files not found.")

    if not os.path.isfile(deploy_path):
        logger.warning(
            "deploy.yaml not found at %s; using env.yaml ranges only", deploy_path
        )

    return policy_path, env_path, deploy_path


class Go2VelocityPolicy(PolicyController):
    """The Unitree Go2 running a velocity tracking locomotion policy."""

    def __init__(
        self,
        prim_path: str,
        policy_path: str,
        env_path: str,
        root_path: Optional[str] = None,
        name: str = "go2",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        super().__init__(name, prim_path, root_path, usd_path, position, orientation)
        self.load_policy(policy_path, env_path)

        self._obs_order = [
            "base_ang_vel",
            "projected_gravity",
            "velocity_commands",
            "joint_pos_rel",
            "joint_vel_rel",
            "last_action",
        ]

        obs_cfg = get_observations(self.policy_env_params) or {}
        self._obs_scales = {}
        for name in self._obs_order:
            scale = obs_cfg.get(name, {}).get("scale")
            if scale is None:
                self._obs_scales[name] = 1.0
            elif isinstance(scale, (int, float)):
                self._obs_scales[name] = float(scale)
            else:
                self._obs_scales[name] = np.array(scale, dtype=np.float32)

        action_terms = get_action(self.policy_env_params) or {}
        self._action_cfg = next(iter(action_terms.values()), {})

        self._action_scale = None
        self._action_offset = None
        self._previous_action = None
        self._policy_counter = 0

    def initialize(self, physics_sim_view=None) -> None:
        """Initialize the robot controller with physics simulation view and control mode."""
        super().initialize(physics_sim_view=physics_sim_view, control_mode="position")
        dof_count = len(self.default_pos)
        self._action_scale = _expand_param(
            self._action_cfg.get("scale"), dof_count, default=1.0
        )
        if self._action_cfg.get("use_default_offset", False):
            self._action_offset = np.array(self.default_pos, dtype=np.float32)
        else:
            self._action_offset = _expand_param(
                self._action_cfg.get("offset"), dof_count, default=0.0
            )

        self._previous_action = np.zeros(dof_count, dtype=np.float32)
        self.action = np.zeros(dof_count, dtype=np.float32)

    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        ang_vel_I = self.robot.get_angular_velocity()
        _, q_IB = self.robot.get_world_pose()

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()
        joint_pos_rel = current_joint_pos - self.default_pos

        obs = np.concatenate(
            [
                ang_vel_b * self._obs_scales["base_ang_vel"],
                gravity_b * self._obs_scales["projected_gravity"],
                command * self._obs_scales["velocity_commands"],
                joint_pos_rel * self._obs_scales["joint_pos_rel"],
                current_joint_vel * self._obs_scales["joint_vel_rel"],
                self._previous_action * self._obs_scales["last_action"],
            ],
            axis=0,
        ).astype(np.float32)
        return obs

    def forward(self, dt: float, command: np.ndarray) -> None:
        """Execute one forward step of the policy with the given command."""
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self.action = np.array(self._compute_action(obs), dtype=np.float32)
            self._previous_action = self.action.copy()

        target_pos = self._action_offset + (self._action_scale * self.action)
        action = ArticulationAction(joint_positions=target_pos)
        self.robot.apply_action(action)
        self._policy_counter += 1


class G1VelocityPolicy(PolicyController):
    """
    The Unitree G1 humanoid running a velocity tracking locomotion policy with observation history.
    """

    def __init__(
        self,
        prim_path: str,
        policy_path: str,
        env_path: str,
        deploy_path: Optional[str] = None,
        root_path: Optional[str] = None,
        name: str = "g1",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        history_length: int = G1_HISTORY_LENGTH,
    ) -> None:
        import torch
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.prims import define_prim, get_prim_at_path

        prim = get_prim_at_path(prim_path)
        if not prim.IsValid():
            prim = define_prim(prim_path, "Xform")
            if usd_path:
                prim.GetReferences().AddReference(usd_path)
            else:
                carb.log_error("unable to add robot usd, usd_path not provided")

        if root_path is None:
            self.robot = SingleArticulation(
                prim_path=prim_path,
                name=name,
                position=position,
                orientation=orientation,
            )
        else:
            self.robot = SingleArticulation(
                prim_path=root_path,
                name=name,
                position=position,
                orientation=orientation,
            )

        self._deploy_cfg = {}
        if deploy_path and os.path.isfile(deploy_path):
            self._deploy_cfg = _load_yaml(deploy_path)
            logger.info("[G1] Loaded deploy config from %s", deploy_path)
        else:
            raise FileNotFoundError(f"deploy.yaml required for G1: {deploy_path}")

        import io

        import omni

        file_content = omni.client.read_file(policy_path)[2]
        file = io.BytesIO(memoryview(file_content).tobytes())
        self.policy = torch.jit.load(file)
        logger.info("[G1] Loaded policy from %s", policy_path)

        self._decimation = int(
            self._deploy_cfg.get("step_dt", 0.02) / 0.005
        )  # Assume sim dt is 0.005
        if "decimation" in self._deploy_cfg:
            self._decimation = self._deploy_cfg["decimation"]

        self._history_length = history_length

        # Joint reordering: joint_ids_map[sim_idx] = sdk_idx
        self._joint_ids_map = self._deploy_cfg.get("joint_ids_map")
        if self._joint_ids_map:
            logger.info(
                "[G1] Joint reordering enabled: %d joints", len(self._joint_ids_map)
            )

        deploy_obs_cfg = self._deploy_cfg.get("observations", {})
        self._obs_scales = {}
        obs_names = [
            "base_ang_vel",
            "projected_gravity",
            "velocity_commands",
            "joint_pos_rel",
            "joint_vel_rel",
            "last_action",
        ]
        for obs_name in obs_names:
            scale = deploy_obs_cfg.get(obs_name, {}).get("scale")
            if scale is None:
                self._obs_scales[obs_name] = 1.0
            elif isinstance(scale, (int, float)):
                self._obs_scales[obs_name] = float(scale)
            else:
                self._obs_scales[obs_name] = np.array(scale, dtype=np.float32)

        self._action_cfg = self._deploy_cfg.get("actions", {}).get(
            "JointPositionAction", {}
        )

        self._action_scale = None
        self._action_offset = None
        self._previous_action = None
        self._policy_counter = 0

        self._default_pos_sim = np.array(
            self._deploy_cfg.get("default_joint_pos", []), dtype=np.float32
        )
        self._stiffness_sdk = np.array(
            self._deploy_cfg.get("stiffness", []), dtype=np.float32
        )
        self._damping_sdk = np.array(
            self._deploy_cfg.get("damping", []), dtype=np.float32
        )

        self.default_pos = None
        self.default_vel = None

        # Per-term observation history buffers
        self._obs_term_histories = None
        self._obs_term_names = [
            "base_ang_vel",
            "projected_gravity",
            "velocity_commands",
            "joint_pos_rel",
            "joint_vel_rel",
            "last_action",
        ]

    def _sdk_to_sim(self, sdk_array: np.ndarray) -> np.ndarray:
        """
        Convert SDK-order array to simulation order.
        """
        if self._joint_ids_map is None or len(sdk_array) != len(self._joint_ids_map):
            return sdk_array
        sim_array = np.zeros_like(sdk_array)
        for sim_idx, sdk_idx in enumerate(self._joint_ids_map):
            sim_array[sim_idx] = sdk_array[sdk_idx]
        return sim_array

    def _compute_action(self, obs: np.ndarray) -> np.ndarray:
        import torch

        with torch.no_grad():
            obs_tensor = torch.from_numpy(obs).view(1, -1).float()
            action = self.policy(obs_tensor).detach().view(-1).numpy()
        return action

    def post_reset(self) -> None:
        """Reset robot state after an episode."""
        self.robot.post_reset()

    def initialize(self, physics_sim_view=None) -> None:
        """Initialize robot articulation and physics simulation."""
        from omni.physx import get_physx_simulation_interface

        self.robot.initialize(physics_sim_view=physics_sim_view)
        self.robot.get_articulation_controller().set_effort_modes("force")

        get_physx_simulation_interface().flush_changes()

        self.robot.get_articulation_controller().switch_control_mode("position")

        all_names = list(self.robot.dof_names)
        logger.info("[G1] Articulation has %d DOFs", len(all_names))
        logger.info("[G1] Joint names: %s", all_names)

        # The locomotion policy drives the 29 BODY joints. With HAND=dex5_1p the
        # articulation also carries 40 finger joints, and Isaac interleaves them
        # (left 29..63, right 34..68, neither contiguous -- C-14), so the policy's
        # vectors are mapped onto a name-defined slice rather than a prefix. Hand
        # joints keep the position drives the URDF import gave them and hold their
        # commanded pose; the policy never writes them.
        self._policy_dofs = np.array(
            [i for i, n in enumerate(all_names) if not _is_hand_joint(n)],
            dtype=np.int32,
        )
        self._hand_dofs = np.array(
            [i for i, n in enumerate(all_names) if _is_hand_joint(n)], dtype=np.int32
        )
        dof_count = len(self._policy_dofs)
        if len(self._hand_dofs):
            logger.info("[G1] %d hand DOFs held outside the policy: %s",
                        len(self._hand_dofs),
                        [all_names[i] for i in self._hand_dofs[:4]] + ["..."])

        # DIAGNOSTIC ONLY -- C-39 payload bisect. NEVER a deliverable.
        #
        # TWIN_HAND_KG sets the TOTAL mass of each Dex5 hand by adjusting the palm body,
        # which carries 0.685702 kg of the hand's 1.025045 kg (contract, Unitree URDF).
        # The rest of the hand -- 0.339343 kg of fingers -- is left alone, so the floor
        # of this knob is ~0.34 kg per hand.
        #
        # Why this exists: the C-39 fork showed SONIC falls even when handed a robot in
        # its OWN nominal stance, which rules out the stance transition and leaves the
        # payload. The W campaign proved SONIC's balance against hands 0.69 kg/hand
        # LIGHTER than reality, so "SONIC balances the G1" was never established at this
        # robot's true hand mass. This measures the margin. Running the twin with a
        # lightened hand and calling it a pass would be reproducing that same defect,
        # which is exactly why every run under this flag is labelled diagnostic.
        _hand_kg = os.environ.get("TWIN_HAND_KG")
        if _hand_kg and len(self._hand_dofs):
            target = float(_hand_kg)
            palm_kg = max(0.001, target - 0.339343)
            try:
                view = self.robot._articulation_view
                bodies = list(view.body_names)
                idx = [bodies.index(b) for b in ("base_link00L", "base_link00")
                       if b in bodies]
                m = view.get_body_masses()
                before = [float(np.ravel(m)[i]) for i in idx]
                new_m = np.array(m, dtype=np.float32).copy()
                for i in idx:
                    new_m[0][i] = palm_kg
                view.set_body_masses(new_m)
                after = [float(np.ravel(view.get_body_masses())[i]) for i in idx]
                logger.warning("[G1] DIAGNOSTIC TWIN_HAND_KG=%.3f -> palm mass %.4f kg "
                               "on %d bodies (was %s, now %s) -- NOT A DELIVERABLE",
                               target, palm_kg, len(idx),
                               [round(v, 4) for v in before], [round(v, 4) for v in after])
                print(f"[G1] DIAGNOSTIC hand mass -> {target:.3f} kg/hand "
                      f"(palm {palm_kg:.4f}, was {[round(v,4) for v in before]}) "
                      f"-- NOT A DELIVERABLE", flush=True)
            except Exception as exc:
                print(f"[G1] TWIN_HAND_KG failed: {exc!r}", flush=True)

        # C-38: cap the Dex5 finger drive gains.
        #
        # The URDF import leaves them at kp 35809.9 with kd 0.000 -- read back and
        # measured, not guessed. That was never a real gain: the Dex5's own URDF gives
        # its finger joints a 0.93 N m effort limit, so a 35810 N m/rad stiffness
        # saturates the actuator at 26 MICRORADIANS of error. Its practical effect is a
        # rigid, undamped finger that slams to its target and shakes the ARM it hangs
        # from -- which is how it first showed up, as SONIC's safety guard tripping on
        # body_dq[20] and body_dq[24], both arm joints.
        #
        # 5.0 / 0.1 is a documented conservative set rather than a value from
        # configs/wbc: that config declares the Dex5 fingers PASSIVE
        # (NUM_HAND_MOTORS: 0), so it has no finger gains to copy. 5 N m/rad reaches
        # the 0.93 N m limit at 0.19 rad of error, which is a sensible band for a
        # finger, and kd 0.1 gives it damping it previously had none of.
        if len(self._hand_dofs):
            hand_kp = float(os.environ.get("TWIN_HAND_KP", "5.0"))
            hand_kd = float(os.environ.get("TWIN_HAND_KD", "0.1"))
            if hand_kp > 0:
                n_hand = len(self._hand_dofs)
                self.robot._articulation_view.set_gains(
                    np.full(n_hand, hand_kp, dtype=np.float32),
                    np.full(n_hand, hand_kd, dtype=np.float32),
                    joint_indices=self._hand_dofs)
                logger.info("[G1] Dex5 finger gains capped to kp=%.2f kd=%.2f on %d "
                            "joints (URDF import gave kp=35809.9 kd=0.0) -- C-38",
                            hand_kp, hand_kd, n_hand)

        if len(self._default_pos_sim) != dof_count:
            raise ValueError(
                f"deploy.yaml default_joint_pos has {len(self._default_pos_sim)} values, "
                f"expected {dof_count} body joints "
                f"(articulation has {len(all_names)} DOFs, {len(self._hand_dofs)} of them hand)"
            )

        self.default_pos = self._default_pos_sim.copy()
        self.default_vel = np.zeros(dof_count, dtype=np.float32)

        if len(self._stiffness_sdk) == dof_count:
            stiffness_sim = self._sdk_to_sim(self._stiffness_sdk)
            damping_sim = (
                self._sdk_to_sim(self._damping_sdk)
                if len(self._damping_sdk) == dof_count
                else None
            )
            self.robot._articulation_view.set_gains(
                stiffness_sim, damping_sim, joint_indices=self._policy_dofs
            )
            logger.info("[G1] Applied stiffness/damping from deploy.yaml")

        self.robot.set_joint_positions(self.default_pos, joint_indices=self._policy_dofs)
        self.robot.set_joint_velocities(self.default_vel, joint_indices=self._policy_dofs)
        if len(self._hand_dofs):
            zeros = np.zeros(len(self._hand_dofs), dtype=np.float32)
            self.robot.set_joint_positions(zeros, joint_indices=self._hand_dofs)
            self.robot.set_joint_velocities(zeros, joint_indices=self._hand_dofs)
        logger.info("[G1] Set initial joint positions")

        self._action_scale = _expand_param(
            self._action_cfg.get("scale"), dof_count, default=0.25
        )
        offset_val = self._action_cfg.get("offset")
        if (
            offset_val is not None
            and isinstance(offset_val, (list, np.ndarray))
            and len(offset_val) == dof_count
        ):
            self._action_offset = np.array(offset_val, dtype=np.float32)
        else:
            self._action_offset = self.default_pos.copy()

        self._previous_action = np.zeros(dof_count, dtype=np.float32)
        self.action = np.zeros(dof_count, dtype=np.float32)

        # Initialize PER-TERM observation history buffers
        term_sizes = {
            "base_ang_vel": 3,
            "projected_gravity": 3,
            "velocity_commands": 3,
            "joint_pos_rel": dof_count,
            "joint_vel_rel": dof_count,
            "last_action": dof_count,
        }
        self._obs_term_histories = {}
        for term_name in self._obs_term_names:
            size = term_sizes[term_name]
            self._obs_term_histories[term_name] = [
                np.zeros(size, dtype=np.float32) for _ in range(self._history_length)
            ]

        total_obs_size = sum(
            term_sizes[name] * self._history_length for name in self._obs_term_names
        )
        logger.info(
            "[G1] Initialization complete - %d DOFs, obs size: %d",
            dof_count,
            total_obs_size,
        )

    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        """Compute observation with per-term history."""
        ang_vel_I = self.robot.get_angular_velocity()
        _, q_IB = self.robot.get_world_pose()

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        current_joint_pos = self.robot.get_joint_positions()[self._policy_dofs]
        current_joint_vel = self.robot.get_joint_velocities()[self._policy_dofs]
        joint_pos_rel = current_joint_pos - self.default_pos

        current_terms = {
            "base_ang_vel": (ang_vel_b * self._obs_scales["base_ang_vel"]).astype(
                np.float32
            ),
            "projected_gravity": (
                gravity_b * self._obs_scales["projected_gravity"]
            ).astype(np.float32),
            "velocity_commands": (
                command * self._obs_scales["velocity_commands"]
            ).astype(np.float32),
            "joint_pos_rel": (joint_pos_rel * self._obs_scales["joint_pos_rel"]).astype(
                np.float32
            ),
            "joint_vel_rel": (
                current_joint_vel * self._obs_scales["joint_vel_rel"]
            ).astype(np.float32),
            "last_action": (
                self._previous_action * self._obs_scales["last_action"]
            ).astype(np.float32),
        }

        for term_name in self._obs_term_names:
            self._obs_term_histories[term_name].pop(0)
            self._obs_term_histories[term_name].append(current_terms[term_name])

        obs_parts = []
        for term_name in self._obs_term_names:
            term_history = np.concatenate(self._obs_term_histories[term_name], axis=0)
            obs_parts.append(term_history)

        return np.concatenate(obs_parts, axis=0)

    def forward(self, dt: float, command: np.ndarray) -> None:
        """Step policy forward and apply actions to robot."""
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self.action = np.array(self._compute_action(obs), dtype=np.float32)
            self._previous_action = self.action.copy()

        target_pos = self._action_offset + (self._action_scale * self.action)
        action = ArticulationAction(
            joint_positions=target_pos, joint_indices=self._policy_dofs
        )
        self.robot.apply_action(action)
        self._policy_counter += 1


def _add_test_props(assets_root_path: str) -> None:
    """Spawn floor-level COCO objects ~1-1.5 m ahead of the robot (origin, +X
    forward) on the ground (z=0), spread laterally to sit in the down-tilted
    camera's FOV. Vision perception integration test only; gated by
    FEROX_SIM_TEST_PROPS. Objects/positions are tuned to what rtdetr actually
    fires on (the parity gate against the live frame)."""
    from pxr import Gf, UsdGeom

    # Kept to what rtdetr actually fires on with sane boxes (parity gate on the
    # live frame): mustard bottle (conf ~0.75, fully on the floor) and the chair
    # (fires ~0.4 on its wheeled base — the seat/back are above the down-tilted
    # FOV). The cracker box only reached "suitcase" ~0.28 (< 0.4) so it is omitted.
    props = [
        ("chair", "/Isaac/Environments/Hospital/Props/SM_Chair_01a.usd", (1.3, 0.25, 0.0), 90.0),
        ("bottle", "/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd", (0.9, -0.25, 0.0), 0.0),
    ]
    for name, rel, pos, yaw in props:
        prim = define_prim(f"/World/TestProps/{name}", "Xform")
        prim.GetReferences().AddReference(assets_root_path + rel)
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        if yaw:
            xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, yaw))
        logger.info(f"[TestProps] {name} <- {rel} at {pos}")


class RobotRosRunner(object):
    """Runner class for Unitree robots (Go2/G1) with ROS2 integration and sensor support."""

    def __init__(
        self,
        physics_dt: float,
        render_dt: float,
        policy_dir: str,
        cmd_vel_topic: str,
        vx_max: float,
        vy_max: float,
        wz_max: float,
        robot_root: str,
        cmd_vel_only: bool,
        enable_sensors: bool,
        enable_keyboard: bool,
        robot_type: str = ROBOT_GO2,
        ros_namespace: str = "",
        twin: bool = False,
    ) -> None:
        """
        Creates the simulation world with preset physics_dt and render_dt and creates a robot inside the warehouse.

        Argument:
        physics_dt {float} -- Physics downtime of the scene.
        render_dt {float} -- Render downtime of the scene.
        robot_type {str} -- Robot type: "go2" or "g1".

        """
        self._robot_type = robot_type
        policy_path, env_path, deploy_path = _validate_policy_paths(policy_dir)

        env_cfg = parse_env_config(env_path)
        deploy_cfg = _load_yaml(deploy_path)

        usd_path = _resolve_usd_path(env_cfg, robot_type)

        # Robot standing height (z) is robot-specific and stays env.yaml-driven:
        # this is the existing per-robot spawn input, left intact. World
        # selection (below) only sets the open-floor (x, y) + yaw, so the world
        # choice is independent of which robot is loaded.
        default_z = G1_INIT_HEIGHT if robot_type == ROBOT_G1 else 0.4
        env_pos = (
            env_cfg.get("scene", {})
            .get("robot", {})
            .get("init_state", {})
            .get("pos", (0.0, 0.0, default_z))
        )
        robot_z = (
            float(env_pos[2])
            if env_pos is not None and len(env_pos) >= 3
            else default_z
        )

        self._world = World(
            stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt
        )
        self._physics_dt = physics_dt
        self._render_dt = render_dt
        self._ros_namespace = ros_namespace

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            raise RuntimeError("Could not find Isaac Sim assets folder")

        # --- World selection (independent of robot) -------------------------
        # SIM_WORLD picks the environment USD (default = office). Resolved +
        # verified against the same assets root as the robot USD, then
        # referenced under /World/Env. Fails loud if it can't resolve.
        world_usd, world_spawn = _resolve_world(assets_root_path)
        prim = define_prim(WORLD_ENV_PRIM, "Xform")
        prim.GetReferences().AddReference(world_usd)

        # --- CONTACT MATERIAL (TWIN_CONTACT_MATERIAL=1) ----------------------
        # The locomotion policy was trained against static 1.0 / dynamic 1.0 /
        # restitution 0.0 with friction_combine_mode "multiply"
        # (ferox-g1-locomotion policy/params/env.yaml, terrain.physics_material
        # and sim.physics_material). NEITHER surface in the twin authors a
        # physics material: the URDF importer wrote none onto the feet and the
        # environment USDs write none onto their floors, so both fall back to
        # the PhysX scene default -- roughly HALF the trained friction.
        #
        # That gates YAW specifically. Turning in place needs the stance foot to
        # make a yaw moment against the floor; walking forward mostly needs
        # normal force and a little fore-aft shear, so halving mu takes the turn
        # away first and leaves the walk. Measured: 0.0-0.2% of commanded yaw
        # rate at every rate including the training maximum, with the policy's
        # own action demonstrably responding to wz.
        #
        # This is NOT tuning-to-fit. It restores a documented training value to
        # a surface that currently has none, and it is off by default so that it
        # is never applied silently -- see RESULTS_MM1 2.6 for the before/after.
        # (applied AFTER the robot exists -- see below. Binding here would bind the
        # world only, and the read-back proved it: "-> ['/World/Env']".)

        # Per-world spawn: open-floor (x, y) + yaw from the world map; z (the
        # standing height) from the robot above. yaw=0 -> identity quat (the
        # robots' env.yaml default orientation), so a yaw-0 entry reproduces the
        # old upright spawn exactly (regression-safe for dso_block_a).
        spawn_xy = world_spawn.get("xy", (0.0, 0.0))
        spawn_yaw = float(world_spawn.get("yaw", 0.0))
        init_pos = np.array(
            [float(spawn_xy[0]), float(spawn_xy[1]), robot_z], dtype=np.float32
        )
        _half_yaw = spawn_yaw / 2.0
        init_quat = np.array(
            [np.cos(_half_yaw), 0.0, 0.0, np.sin(_half_yaw)], dtype=np.float32
        )  # wxyz, yaw about +Z
        logger.info(
            "[World] spawn pos=%s yaw=%.3frad (robot=%s)",
            init_pos.tolist(),
            spawn_yaw,
            robot_type,
        )
        print(
            f"[World] spawn pos={init_pos.tolist()} yaw={spawn_yaw:.3f}rad "
            f"(robot={robot_type})",
            flush=True,
        )

        # Floor-level COCO test props, gated by FEROX_SIM_TEST_PROPS=1 (off by
        # default — never disturbs the nav/demo scene). The nav camera is tilted
        # ~25deg down and sees the FLOOR, so the vision integration test uses
        # ground objects it can see, not standing people (2026-06-13 finding).
        if os.environ.get("FEROX_SIM_TEST_PROPS", "0") == "1":
            _add_test_props(assets_root_path)

        if not usd_path:
            raise RuntimeError(f"{robot_type.upper()} USD path could not be resolved")

        # Create robot based on type
        if robot_type == ROBOT_G1:
            self._robot = G1VelocityPolicy(
                prim_path=robot_root,
                name="G1",
                usd_path=usd_path,
                position=init_pos,
                orientation=init_quat,
                policy_path=policy_path,
                env_path=env_path,
                deploy_path=deploy_path,
            )
        else:
            self._robot = Go2VelocityPolicy(
                prim_path=robot_root,
                name="Go2",
                usd_path=usd_path,
                position=init_pos,
                orientation=init_quat,
                policy_path=policy_path,
                env_path=env_path,
            )

        # Keep the spawn so /twin/reset can restore it explicitly. The robot is
        # never added to world.scene, so world.reset() has no default state for it
        # and restores the joints but leaves the base wherever it fell: the first
        # A/B knocked the G1 down and it "reset" to (10.7, 11.4) instead of the
        # spawn (7.8, 2.0). Storing the numbers and writing them back is explicit
        # and can be read back, which the scene-managed path could not be.
        self._spawn_pos = np.array(init_pos, dtype=np.float32).copy()
        self._spawn_quat = np.array(init_quat, dtype=np.float32).copy()

        # Contact material AFTER the robot prim exists. Binding it with the world
        # (where this used to live) silently bound only /World/Env, leaving the
        # feet on the PhysX default -- and with combine_mode "multiply" that is
        # still half the trained friction, i.e. exactly the condition under test.
        # The read-back printed "-> ['/World/Env']" and caught it.
        if os.environ.get("TWIN_CONTACT_MATERIAL") == "1":
            _apply_training_contact_material(self._world.stage, robot_root)

        # Before any stepping -- see DEX5_TIP_LINKS.
        self._hand_contact_sensors = (
            _make_hand_contact_sensors(robot_root, self._world.stage)
            if os.environ.get("TWIN_HAND_CONTACT_SENSORS") == "1" else [])

        # TWIN_ARMATURE: rotor inertia, which the twin's USD does not have.
        # unitree_rl_lab's UNITREE_G1_29DOF_CFG sets armature=0.01 on EVERY
        # actuator group; the URDF importer wrote 0.0 on all 29 (and all 69)
        # joints, and nothing else sets it -- run.py overwrites stiffness and
        # damping from deploy.yaml but has never touched armature. At zero the
        # joint is lighter than the one the policy learned on, which changes the
        # PD response. Off by default; see RESULTS_MM1 2.10.
        if os.environ.get("TWIN_ARMATURE"):
            _apply_training_armature(self._world.stage, robot_root,
                                     float(os.environ["TWIN_ARMATURE"]))
        if os.environ.get("TWIN_JOINT_FRICTION"):
            try:
                _apply_joint_friction(self._world.stage, robot_root,
                                      float(os.environ["TWIN_JOINT_FRICTION"]))
            except Exception as exc:
                print(f"[TWIN] joint friction NOT applied: {exc!r} -- continuing "
                      f"WITHOUT it; any result from this run is a run at zero dry "
                      f"friction and must not be reported as testing it", flush=True)

        cmd_min, cmd_max = _resolve_command_limits(deploy_cfg, env_cfg)
        args_min = np.array([-vx_max, -vy_max, -wz_max], dtype=np.float32)
        args_max = np.array([vx_max, vy_max, wz_max], dtype=np.float32)
        self._cmd_min = np.maximum(cmd_min, args_min)
        self._cmd_max = np.minimum(cmd_max, args_max)

        self._cmd_vel_topic = cmd_vel_topic
        self._cmd_vel_only = cmd_vel_only
        self._enable_sensors = enable_sensors
        self._enable_keyboard = enable_keyboard
        self._twin = twin
        # G1_CONTROL selects who drives the articulation (MM3, CAMPAIGN 4.2):
        #   "policy" (default) -- the omni locomotion policy, unchanged behaviour
        #   "lowcmd"           -- rt/lowcmd via the low-level DDS bridge, and the
        #                         policy never runs. The two must NEVER co-drive;
        #                         on_physics_step asserts that by branching, not by
        #                         summing.
        self._g1_control = os.environ.get("G1_CONTROL", "policy").strip().lower()
        self._lowlevel = None
        self._mm5 = None
        #   "handoff"          -- the omni policy holds the robot standing until a live
        #                         rt/lowcmd stream has been up for G1_HANDOFF_S seconds,
        #                         then the low-level bridge takes the articulation ONCE
        #                         and the policy is never stepped again. This is the
        #                         pattern CAMPAIGN 4.4 already specifies for MM5, and it
        #                         is the only way to give SONIC a standing robot: it
        #                         needs 20-60 s to boot and MM3 (a2) established the twin
        #                         cannot stand unaided for even one second. Still never
        #                         co-driving -- the switch is one-way and latched.
        if self._g1_control not in ("policy", "lowcmd", "handoff"):
            raise ValueError(
                f"G1_CONTROL must be 'policy', 'lowcmd' or 'handoff', got {self._g1_control!r}")
        self._handoff_s = float(os.environ.get("G1_HANDOFF_S", "10"))
        self._handoff_since = None
        self._handoff_last_fresh = 0.0
        self._handed_over = False
        if self._g1_control in ("lowcmd", "handoff") and robot_type != ROBOT_G1:
            raise ValueError("G1_CONTROL=lowcmd is only meaningful for --robot_type g1")
        self._twin_report = {}

        self._vx_max = vx_max
        self._vy_max = vy_max
        self._wz_max = wz_max

        self._linear_attr = None
        self._angular_attr = None
        self._sensors = {}

        self._robot_root = robot_root
        _configure_ros_utils_paths(robot_root, robot_type)

        self._base_command = np.zeros(3, dtype=np.float32)
        self._last_cmd_vel_time: Optional[float] = None
        self._last_cmd_vel_count: int = 0
        self._cmd_vel_count_attr = None
        self._reset_latch = None

        cmd_scale = np.maximum(np.abs(self._cmd_min), np.abs(self._cmd_max))
        vx, vy, wz = cmd_scale.tolist()
        self._input_keyboard_mapping = {
            "NUMPAD_8": [vx, 0.0, 0.0],
            "UP": [vx, 0.0, 0.0],
            "NUMPAD_2": [-vx, 0.0, 0.0],
            "DOWN": [-vx, 0.0, 0.0],
            "NUMPAD_6": [0.0, -vy, 0.0],
            "RIGHT": [0.0, -vy, 0.0],
            "NUMPAD_4": [0.0, vy, 0.0],
            "LEFT": [0.0, vy, 0.0],
            "NUMPAD_7": [0.0, 0.0, wz],
            "N": [0.0, 0.0, wz],
            "NUMPAD_9": [0.0, 0.0, -wz],
            "M": [0.0, 0.0, -wz],
        }
        self.needs_reset = False
        self.first_step = True

    def setup(self) -> None:
        """
        Set up keyboard listener and add physics callback.

        """
        if self._enable_keyboard:
            self._appwindow = omni.appwindow.get_default_app_window()
            if self._appwindow is not None:
                self._input = carb.input.acquire_input_interface()
                self._keyboard = self._appwindow.get_keyboard()
                self._sub_keyboard = self._input.subscribe_to_keyboard_events(
                    self._keyboard, self._sub_keyboard_event
                )
            else:
                logger.warning("No app window found; keyboard control disabled")
                self._enable_keyboard = False
        self._world.add_physics_callback(
            "go2_ros2_step", callback_fn=self.on_physics_step
        )

    def setup_ros(self) -> None:
        """Set up ROS2 nodes for command velocity and sensor publishers."""
        print("[PANTHERA-MARK] E1: entered setup_ros() body", flush=True)
        print(f"[PANTHERA-MARK] E2: self._cmd_vel_topic = {self._cmd_vel_topic}", flush=True)
        self._linear_attr, self._angular_attr, self._cmd_vel_count_attr = (
            ros_utils.setup_cmd_vel_graph(self._cmd_vel_topic)
        )
        # A re-spawn request from outside. See setup_reset_sub for why this
        # exists; it reuses the world.reset path the stop-button already takes,
        # so there is exactly one reset implementation and it is the tested one.
        self._reset_latch = ros_utils.setup_reset_sub("/twin/reset")
        if not self._enable_sensors:
            return

        for _ in range(10):
            self._world.step(render=True)

        # TWIN PATH. Everything below the branch is the legacy mode:=sim wiring —
        # arbitrary sensor offsets, Go2 static TFs on both robots, camera at
        # 480x270 in a frame no robot publishes. It stays until DT7 retires it.
        # The twin path shares none of it: poses come from the USD sensor layer,
        # topics and frames come from isaac/twin/<robot>_contract.yaml.
        if self._twin:
            # A twin that fails to come up must say so. Without this the exception
            # unwinds into Isaac's shutdown path and the log shows only
            # "Simulation App Shutting Down" -- no traceback, no cause.
            try:
                self._setup_twin_ros()
            except Exception:
                import traceback
                print("[TWIN] FATAL: twin interface setup failed", flush=True)
                traceback.print_exc()
                raise
            return

        render_hz = None
        if self._render_dt:
            render_hz = 1.0 / self._render_dt

        # G1: sensors on torso_link at higher positions
        # Go2: sensors on base at lower positions
        if self._robot_type == ROBOT_G1:
            camera_link_pos = (0.0, 0.0, 0.75)
            lidar_l1_pos = (0.2, 0.0, 0.4)
            lidar_velo_pos = (0.15, 0.0, 0.5)
            enable_lidar = True
        else:
            camera_link_pos = (0.3, 0.0, 0.10)
            lidar_l1_pos = (0.15, 0.0, 0.15)
            lidar_velo_pos = (0.1, 0.0, 0.2)
            enable_lidar = True

        self._sensors = ros_utils.setup_sensors_delayed(
            simulation_app,
            render_hz=render_hz,
            camera_link_position=camera_link_pos,
            enable_lidar=enable_lidar,
            lidar_l1_position=lidar_l1_pos,
            lidar_velo_position=lidar_velo_pos,
            robot_type=self._robot_type,
        )

        # Additional render steps to initialize camera render products
        for _ in range(5):
            self._world.step(render=True)

        ros_ns = self._ros_namespace

        ros_utils.setup_ros_publishers(
            self._sensors,
            simulation_app,
            robot_type=self._robot_type,
            camera_link_pos=camera_link_pos,
            lidar_l1_pos=lidar_l1_pos,
            lidar_velo_pos=lidar_velo_pos,
            ros_namespace=ros_ns,
        )

        # Intrinsics DERIVED from the actual sim camera (no hardcoded magic
        # numbers). Color + depth share the realsense_depth_camera prim, so one
        # set of intrinsics applies to both. Falls back to the legacy values if
        # the camera API is unavailable.
        try:
            dcam = self._sensors["realsense_depth_camera"]
            K = dcam.get_intrinsics_matrix()
            res = dcam.get_resolution()
            cam_fx, cam_fy = float(K[0][0]), float(K[1][1])
            cam_cx, cam_cy = float(K[0][2]), float(K[1][2])
            cam_w, cam_h = int(res[0]), int(res[1])
            print(f"[PANTHERA-MARK] derived intrinsics fx={cam_fx:.3f} fy={cam_fy:.3f} "
                  f"cx={cam_cx:.3f} cy={cam_cy:.3f} {cam_w}x{cam_h}", flush=True)
        except Exception as e:
            logger.info(f"[WARN] intrinsics derive failed ({e}); using legacy defaults")
            cam_fx = cam_fy = 242.479
            cam_cx, cam_cy = 242.736, 133.273
            cam_w, cam_h = 480, 270

        ros_utils.setup_depth_camerainfo_graph(
            simulation_app,
            topic="camera/depth/camera_info",
            frame_id="camera_optical",
            node_namespace=ros_ns,
            width=cam_w,
            height=cam_h,
            fx=cam_fx,
            fy=cam_fy,
            cx=cam_cx,
            cy=cam_cy,
        )

        ros_utils.setup_odom_publisher(simulation_app)
        ros_utils.setup_color_camera_publishers(
            self._sensors, simulation_app, ros_namespace=ros_ns
        )
        ros_utils.setup_color_camerainfo_graph(
            simulation_app,
            topic="camera/color/camera_info",
            frame_id="camera_optical",
            node_namespace=ros_ns,
            width=cam_w,
            height=cam_h,
            fx=cam_fx,
            fy=cam_fy,
            cx=cam_cx,
            cy=cam_cy,
        )

        ros_utils.setup_joint_states_publisher(
            simulation_app, robot_type=self._robot_type
        )

    def _setup_twin_ros(self) -> None:
        """Publish the hardware interface, entirely from the contract.

        Only the G1 is wired for the twin at DT2; the Go2 follows at DT5. A robot
        without a contract raises rather than quietly falling back to the legacy
        path, because a twin that is silently not a twin is the failure this whole
        campaign exists to prevent.
        """
        import sys

        for extra in ("/workspace/ferox_tools", "/workspace/ferox_isaac/twin"):
            if extra not in sys.path:
                sys.path.insert(0, extra)
        import twin_contract
        import sensors as twin_sensors
        import publishers as twin_pub

        contract_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "twin",
            f"{self._robot_type}_contract.yaml")
        contract = twin_contract.load(contract_path)
        ns = contract["robot"]["namespace"]
        if ns != self._ros_namespace:
            raise RuntimeError(
                f"contract namespace {ns} != --ros_namespace {self._ros_namespace}; "
                "refusing to publish a twin under the wrong namespace")

        root = self._robot_root
        camera_tf = os.environ.get("CAMERA_TF", "0") == "1"

        print(f"[TWIN] contract {contract_path}", flush=True)
        print(f"[TWIN] namespace {ns}  CAMERA_TF={int(camera_tf)}", flush=True)

        # --- devices, as identity children of the authored frames --------------
        # TWIN_LIDAR=0 skips the lidar DEVICE only, the exact mirror of TWIN_CAMERA=0
        # below: the contract, the TF edges and the frame_ids are untouched and the
        # audit still checks them; the Mid-360's topics simply do not appear.
        #
        # Two reasons it exists. (1) Mohammed turned the lidar off by switch for the
        # 2026-08-22 session. (2) The C-39 A/B loads the REFERENCE MJCF body, which has
        # no sensor frames at all -- no mid360_link, no d435_link -- so an RTX lidar has
        # nothing to mount on and the run dies a world-load later with "frame
        # 'livox_frame' is not in the stage". Balance experiments need no sensors.
        #
        # Default is ON, so every existing gate behaves exactly as before.
        lidar_prim = None
        lidar_derived = None
        want_lidar = os.environ.get("TWIN_LIDAR", "1") != "0"
        if not want_lidar:
            print("[TWIN] Mid-360 SKIPPED (TWIN_LIDAR=0) -- device only, "
                  "contract untouched", flush=True)
        else:
            lidar_prim, lidar_derived = twin_sensors.create_lidar(contract, root)
            print(f"[TWIN] Mid-360 at {lidar_prim.GetPath()} {lidar_derived}", flush=True)

        # Which devices exist is a property of the ROBOT and is read off the
        # contract, not branched on robot_type. The G1 carries a D435i and two IMUs;
        # the Go2 carries neither camera nor body IMU and publishes the L1's IMU
        # instead. A `if robot_type == GO2` here would be a second place for the two
        # robots to disagree with their own contracts.
        # TWIN_CAMERA=0 skips the camera DEVICE only. It does not touch the contract,
        # the TF edges, the frame_ids or anything the audit checks -- the camera's
        # topics simply do not appear, exactly as they do not on the Go2.
        #
        # It exists because Isaac 5.1's synthetic-data pipeline SEGFAULTS on this box
        # (RTX 4080 SUPER 16 GB) on the first world.step(render=True) after a camera
        # render product is created, inside libomni.syntheticdata +
        # libomni.graph.image.core. Reproduced five times for five, at run.py:1201
        # every time. Not a resource limit (peak VRAM 3776 MiB of 16376, host RSS
        # 5.7 GB of 47) and not the depth annotator (removing it changes nothing --
        # the default rgb annotator does it too). The Go2 twin, same box and same
        # bridge with no camera in its contract, boots every time.
        #
        # Default is ON. This flag is how the lidar/nav half of the twin stays
        # workable on a box where the camera half cannot run; see C-23.
        camera = None
        want_camera = os.environ.get("TWIN_CAMERA", "1") != "0"
        if not want_camera:
            print("[TWIN] camera SKIPPED (TWIN_CAMERA=0) -- C-23, device only, "
                  "contract untouched", flush=True)
        elif any(t["name"].endswith("/camera/color/image_raw")
                 for t in contract.get("topics", [])):
            camera, K_got = twin_sensors.create_camera(contract, root)
            print(f"[TWIN] D435i K readback fx={K_got['fx']:.3f} fy={K_got['fy']:.3f} "
                  f"cx={K_got['cx']:.3f} cy={K_got['cy']:.3f} "
                  f"(HFOV {K_got['hfov_deg']:.2f} deg, VFOV {K_got['vfov_deg']:.2f} deg)",
                  flush=True)
        else:
            print("[TWIN] no camera in the contract", flush=True)

        imu_topics = [t for t in contract.get("topics", [])
                      if t["type"] == "sensor_msgs/msg/Imu"]
        imus = []
        for t in imu_topics:
            sensor = twin_sensors.create_imu_for(
                contract, root, t["name"], t["frame_id"].replace("/", "_"))
            imus.append((t, sensor))
            try:
                sensor.initialize()
            except Exception:
                pass

        # Render products need a few frames before they yield anything.
        for _ in range(5):
            self._world.step(render=True)

        # --- the wire ---------------------------------------------------------
        edges = twin_pub.setup_tf_static(contract, camera_tf=camera_tf)
        print(f"[TWIN] /tf_static: {len(edges)} edges "
              f"({', '.join(e['parent'] + '->' + e['child'] for e in edges)})", flush=True)

        # Use the ACTUAL rendering dt, not the requested one. Isaac quantises
        # rendering_dt to a multiple of physics_dt, so --render_dt 1/60 against
        # --physics_dt 1/200 becomes 0.015 s = 66.67 Hz. Dividing by the requested
        # 60 gave decimation step 6 and a 11.11 Hz lidar against a 10 Hz contract --
        # a miss that looks like sensor jitter and is actually arithmetic.
        try:
            render_dt = float(self._world.get_rendering_dt())
        except Exception:
            render_dt = self._render_dt or (1.0 / 60.0)
        render_hz = 1.0 / render_dt if render_dt else 60.0
        print(f"[TWIN] rendering_dt={render_dt:.6f}s ({render_hz:.2f} Hz) "
              f"requested {self._render_dt:.6f}s", flush=True)
        # setup_lidar_cloud prints the topic it selected -- it is the only thing
        # that knows which one the contract chose, and hardcoding "/livox/lidar"
        # here printed the G1's topic during a Go2 run.
        if lidar_prim is not None:
            twin_pub.setup_lidar_cloud(contract, lidar_prim, render_hz=render_hz)
        else:
            print("[TWIN] lidar cloud publisher SKIPPED (TWIN_LIDAR=0)", flush=True)

        # IMU via rclpy, NOT OmniGraph. ROS2PublishImu builds without error, logs
        # success, and advertises nothing -- baseline defect B-4, still reproducible.
        # sim_utils already publishes cmd_vel through rclpy for the same reason.
        import rclpy_pub as twin_rclpy
        self._twin_rclpy = twin_rclpy.TwinRclpyPublishers()
        self._twin_imus = []
        for spec, sensor in imus:
            key = spec["frame_id"].replace("/", "_")
            self._twin_rclpy.add_imu(key, spec["name"], reliable=True)
            self._twin_imus.append({
                "key": key, "sensor": sensor, "frame": spec["frame_id"],
                "period": 1.0 / float(spec["rate_hz"]), "next": 0.0,
            })
            print(f"[TWIN] IMU -> {spec['name']} (frame {spec['frame_id']}, "
                  f"{spec['rate_hz']} Hz, rclpy)", flush=True)

        if camera is not None:
            twin_pub.setup_camera_color(contract, camera, ns)
            depth_topic = twin_pub.setup_camera_depth_raw(contract, camera, ns)
            print(f"[TWIN] camera colour -> {ns}/camera/color/image_raw", flush=True)
            print(f"[TWIN] raw depth (32FC1, converter seam) -> {ns}/{depth_topic}",
                  flush=True)

        # /clock first: without it every use_sim_time consumer sits at time zero
        # and Nav2 silently never plans.
        ros_utils.setup_clock_publisher()
        odom_spec = twin_pub._topic_of_type(contract, "nav_msgs/msg/Odometry")
        # Odometry via rclpy, not the OmniGraph publisher. The graph is OnTick, so
        # it emits once per RENDER frame (~66 Hz measured) and there is no divisor
        # of 60 that lands on the contract's 51.4. Driving it from the physics
        # callback with a sim-time limiter hits the rate exactly. The TF graph
        # stays -- odom -> base_link is a transform, not a topic, and consumers
        # want it at the highest rate available.
        self._twin_rclpy.add_odom("odom", odom_spec["name"])
        self._twin_odom_frame = odom_spec["frame_id"]
        self._twin_odom_period = 1.0 / float(odom_spec["rate_hz"])
        self._twin_odom_next = 0.0
        ros_utils.setup_odom_tf_publisher()
        print(f"[TWIN] odom -> {odom_spec['name']}", flush=True)

        ros_utils.setup_joint_states_publisher(
            simulation_app, robot_type=self._robot_type)

        self._sensors = {"twin_lidar": lidar_prim}
        if camera is not None:
            self._sensors["twin_camera"] = camera
        for spec, sensor in imus:
            self._sensors[f"twin_imu_{spec['frame_id']}"] = sensor
        self._twin_report = {
            "lidar": lidar_derived,
            "tf_static_edges": [f"{e['parent']}->{e['child']}" for e in edges],
            "imu_topics": [spec["name"] for spec, _ in imus],
        }
        if camera is not None:
            self._twin_report["camera_K"] = K_got
        print("[TWIN] interface up", flush=True)

    def _twin_pump(self) -> None:
        """Publish the rclpy-side twin topics, rate-limited on SIM time.

        Called from the PHYSICS callback, not the render loop. The render loop steps
        at render_dt (60 Hz of sim time), so an IMU driven from there tops out around
        60 Hz and can never reach the contract's 100 -- measured 65.75 Hz before this
        moved. Physics runs at 200 Hz, so 100 Hz is a clean 2:1 decimation of it.

        Rate-limiting on SIM time rather than wall time is the other half: the sim
        runs at well under real time, so a wall-clock limiter would emit 100 Hz of
        wall time and several hundred Hz of sim time -- a rate no consumer expects
        and the audit would correctly flag.
        """
        if getattr(self, "_twin_rclpy", None) is None:
            return
        t = getattr(self, "_twin_sim_time", None)
        if t is None:
            return
        for entry in getattr(self, "_twin_imus", ()):
            if t < entry["next"]:
                continue
            entry["next"] = t + entry["period"]
            try:
                frame = entry["sensor"].get_current_frame()
            except Exception:
                continue
            lin, ang = frame.get("lin_acc"), frame.get("ang_vel")
            ori = frame.get("orientation")
            if lin is None or ang is None:
                continue
            self._twin_rclpy.publish_imu(entry["key"], entry["frame"], t, lin, ang,
                                         orientation_wxyz=ori if ori is not None else None)

        if getattr(self, "_twin_odom_next", None) is not None and t >= self._twin_odom_next:
            self._twin_odom_next = t + self._twin_odom_period
            try:
                pos_w, quat_wxyz = self._robot.robot.get_world_pose()
                lin_vel = self._robot.robot.get_linear_velocity()
                ang_vel = self._robot.robot.get_angular_velocity()
            except Exception:
                return
            self._twin_rclpy.publish_odom(
                "odom", self._twin_odom_frame, "base_link", t,
                pos_w, quat_wxyz, lin_vel, ang_vel)

    def _get_cmd_vel(self) -> Optional[np.ndarray]:
        if self._linear_attr is None or self._angular_attr is None:
            return None
        lin = self._linear_attr.get()
        ang = self._angular_attr.get()
        if lin is None or ang is None:
            return None

        vx = ros_utils.clamp(float(lin[0]), -self._vx_max, self._vx_max)
        vy = ros_utils.clamp(float(lin[1]), -self._vy_max, self._vy_max)
        wz = ros_utils.clamp(float(ang[2]), -self._wz_max, self._wz_max)
        return np.array([vx, vy, wz], dtype=np.float32)

    def _update_odom(self) -> None:
        try:
            pos_w, quat_wxyz = self._robot.robot.get_world_pose()
            lin_vel = self._robot.robot.get_linear_velocity()
            ang_vel = self._robot.robot.get_angular_velocity()
            quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]

            ros_utils.update_odom_tf(pos_w, quat_xyzw)
            ros_utils.update_odom(pos_w, quat_xyzw, lin_vel, ang_vel)
        except Exception:
            return

    def _respawn_base(self) -> bool:
        """Put the base back at the spawn pose with zero velocity, and read it back.

        world.reset() is not enough on its own -- see the note at self._spawn_pos.
        Joint state is handled by the reset path re-running the policy's
        initialize(), which also rebuilds the observation history; this only has to
        deal with the floating base.
        """
        art = getattr(getattr(self, "_robot", None), "robot", None)
        if art is None or getattr(self, "_spawn_pos", None) is None:
            return False
        try:
            art.set_world_pose(position=self._spawn_pos, orientation=self._spawn_quat)
            zeros6 = np.zeros(6, dtype=np.float32)
            try:
                art.set_velocities(zeros6)
            except Exception:
                art.set_linear_velocity(zeros6[:3])
                art.set_angular_velocity(zeros6[3:])
            # Joint velocities too. The base was zeroed above, but a robot that
            # fell still has spinning limbs, and dropping it upright with those
            # retained is a good way to make it fall straight back over -- which
            # is what the second A/B cycle did after a respawn that read back at
            # err 0.0000.
            try:
                art.set_joint_velocities(
                    np.zeros(art.num_dof, dtype=np.float32)
                )
            except Exception as exc:  # noqa: BLE001
                print(f"[PANTHERA] respawn: joint-velocity zero failed: {exc}", flush=True)
            pos, _ = art.get_world_pose()
            err = float(np.linalg.norm(np.asarray(pos, dtype=np.float32) - self._spawn_pos))
            print(f"[PANTHERA] respawn -> {np.asarray(pos).tolist()} "
                  f"(err {err:.4f} m from spawn)", flush=True)
            return err < 0.05
        except Exception as exc:  # noqa: BLE001
            print(f"[PANTHERA] respawn FAILED: {exc}", flush=True)
            return False

    def on_physics_step(self, step_size) -> None:
        """
        Physics call back, initialize robot (first frame) and call controller forward function.

        """
        if self._twin:
            # Accumulate sim time from the PHYSICS step. self._world.current_time
            # only advances once per world.step(), i.e. per RENDER frame, so using
            # it here let exactly one publish through per render tick and pinned
            # both IMUs at ~66 Hz no matter what period was requested. step_size is
            # the physics dt, which is the clock these sensors actually run on.
            self._twin_sim_time = getattr(self, "_twin_sim_time", 0.0) + float(step_size)
            self._twin_pump()
        if self.first_step:
            self._robot.initialize()
            self.first_step = False
            if os.environ.get("MM5") == "1" and self._mm5 is None:
                from twin.mm5.runner import MM5Runner, MM5Config
                cfg = MM5Config(
                    robot_root=self._robot_root,
                    object_name=os.environ.get("MM5_OBJECT", "soup_can"),
                    trials=int(os.environ.get("MM5_TRIALS", "20")),
                    seed=int(os.environ.get("MM5_SEED", "20260820")),
                    cheat_attach=os.environ.get("MM5_CHEAT_ATTACH", "0") == "1",
                    fix_base=os.environ.get("MM5_FIX_BASE", "0") == "1",
                    surface=os.environ.get("MM5_SURFACE", "table"),
                    measure_hand=os.environ.get("MM5_MEASURE_HAND", "0") == "1",
                    contact_sensor_paths=[p for _, p in
                                          getattr(self, "_hand_contact_sensors", [])],
                    place_from_workspace=(
                        os.environ.get("MM5_PLACE_FROM_WORKSPACE", "0") == "1"),
                    target_r=float(os.environ.get("MM5_TARGET_R", "0.315")),
                    pelvis_z=float(os.environ.get("MM5_PELVIS_Z", "0.80")),
                    out_dir=os.environ.get("MM5_OUT", "/workspace/ferox_isaac/mm5_out"),
                )
                self._mm5 = MM5Runner(self._robot.robot, self._world.stage, cfg)

            if os.environ.get("TWIN_SENSOR_MASS_FIX") == "1":
                try:
                    _apply_sensor_masses(self._robot.robot)
                except Exception as exc:
                    print(f"[TWIN] sensor-mass fix error: {exc!r}", flush=True)

            if os.environ.get("C39_CAPTURE"):
                try:
                    from twin.c39_capture import capture
                    capture(self._robot.robot, self._world.stage,
                            os.environ.get("C39_OUT", "/workspace/ferox_isaac/c39_out"),
                            os.environ["C39_CAPTURE"])
                except Exception as exc:
                    import traceback
                    print(f"[c39] capture FAILED: {exc!r}", flush=True)
                    traceback.print_exc()

            if os.environ.get("MM5_PROBE") == "1":
                try:
                    from twin.mm5_probe import probe
                    probe(self._robot.robot, self._world.stage)
                except Exception as exc:
                    print(f"[probe] failed: {exc!r}", flush=True)

            if self._g1_control in ("lowcmd", "handoff") and self._lowlevel is None:
                # Built HERE, not in setup(): the articulation has no DOF names until
                # initialize() has run, and the bridge maps every joint by name.
                # It also has to land after initialize(), which sets position-control
                # gains the bridge then zeroes in favour of its own torques.
                from twin.lowlevel_bridge.sim_side import LowLevelSimBridge

                self._lowlevel = LowLevelSimBridge(
                    self._robot.robot,
                    physics_dt=self._physics_dt,
                    pd_hz=float(os.environ.get("G1_PD_HZ", "500")),
                    cmd_timeout_ms=float(os.environ.get("G1_CMD_TIMEOUT_MS", "85")),
                    passive=(self._g1_control == "handoff"),
                )
                print("[PANTHERA-MARK] low-level bridge attached (G1_CONTROL=lowcmd)",
                      flush=True)
            return

        if self._g1_control == "handoff" and not self._handed_over:
            # Publish state so SONIC can boot, but let the POLICY keep the robot up.
            if self._lowlevel is not None:
                self._lowlevel.on_physics_step(step_size)
                now = time.time()
                if self._lowlevel._cmd_fresh():
                    self._handoff_last_fresh = now
                    if self._handoff_since is None:
                        self._handoff_since = now
                        print(f"[PANTHERA-MARK] rt/lowcmd is live; handing over in "
                              f"{self._handoff_s:.0f}s", flush=True)
                    elif now - self._handoff_since >= self._handoff_s:
                        self._lowlevel.activate()
                        self._handed_over = True
                        print("[PANTHERA-MARK] HANDOVER complete: the policy will not be "
                              "stepped again", flush=True)
                        # Return on the activation step. Falling through would call
                        # on_physics_step a SECOND time in the same step, double-counting
                        # sim_time -- and rt/lowstate.tick is derived from sim_time.
                        self._update_odom()
                        return
                elif (self._handoff_since is not None
                      and now - self._handoff_last_fresh > 1.0):
                    # Only a SUSTAINED loss of rt/lowcmd cancels the hand-over. Resetting
                    # on a single miss never let it fire at all: the seqlock is read at
                    # 1 kHz against a 500 Hz writer, so isolated torn reads are normal
                    # and the countdown restarted forever while the robot stood happily
                    # under the policy and SONIC was never given the robot.
                    print("[PANTHERA-MARK] rt/lowcmd went quiet; hand-over cancelled",
                          flush=True)
                    self._handoff_since = None
            # fall through to the policy path below for this step

        if self._g1_control == "lowcmd" or self._handed_over:
            # rt/lowcmd owns the articulation. The locomotion policy is not stepped
            # at all -- not zeroed, not blended, not run-and-discarded -- so there is
            # no path by which both can write joint targets in the same step.
            if self._lowlevel is not None:
                self._lowlevel.on_physics_step(step_size)
            self._update_odom()
            return
        if self._reset_latch is not None and self._reset_latch.take():
            self.needs_reset = True
        if self.needs_reset:
            self._world.reset(True)
            self._respawn_base()
            self.needs_reset = False
            self.first_step = True
            return

        cmd = self._base_command.copy()

        # Check if a new /cmd_vel message arrived via the OmniGraph counter
        now = time.time()
        if self._cmd_vel_count_attr is not None:
            count = self._cmd_vel_count_attr.get()
            if count != self._last_cmd_vel_count:
                self._last_cmd_vel_count = count
                self._last_cmd_vel_time = now

        timed_out = (
            self._last_cmd_vel_time is None
            or (now - self._last_cmd_vel_time) > CMD_VEL_TIMEOUT
        )

        if not timed_out:
            cmd_vel = self._get_cmd_vel()
            if cmd_vel is not None:
                if self._cmd_vel_only:
                    cmd = cmd_vel
                else:
                    cmd = cmd + cmd_vel

        cmd = np.minimum(np.maximum(cmd, self._cmd_min), self._cmd_max)
        if self._mm5 is not None:
            self._mm5.step(step_size)
        self._robot.forward(step_size, cmd)
        if self._mm5 is not None:
            # AFTER forward(): the locomotion policy commands all 29 body joints every
            # step, arms included, so MM5's arm and finger targets have to be re-asserted
            # or they are overwritten the instant they are set. Legs and torso remain the
            # policy's -- this is the same split MM4's POSE mode uses.
            self._mm5.reapply()
        self._update_odom()

    def run(self, real_time: bool) -> None:
        """
        Step simulation based on rendering downtime.

        """
        # Offscreen follow camera on THIS sim -- the one that actually walks.
        # Inert unless TWIN_FILM=1, so the default graph is unchanged.
        try:
            from twin.film_live import LiveFilm

            self._film = LiveFilm(lambda: getattr(
                getattr(self, "_robot", None), "robot", None))
            self._film.start()
        except Exception as exc:  # noqa: BLE001
            print(f"[FILM] disabled: {exc}", flush=True)
            self._film = None

        while simulation_app.is_running():
            t0 = time.time()
            self._world.step(render=True)
            if getattr(self, "_film", None) is not None:
                self._film.tick(getattr(self, "_twin_sim_time", 0.0) or time.time())
            viewport_follow.maybe_step(self)  # PANTHERA: no-op unless VIEWPORT_FOLLOW
            if self._world.is_stopped():
                self.needs_reset = True
            if real_time:
                sleep = self._physics_dt - (time.time() - t0)
                if sleep > 0:
                    time.sleep(sleep)
        return

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """
        Keyboard subscriber callback to when kit is updated.

        """
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(
                    self._input_keyboard_mapping[event.input.name], dtype=np.float32
                )

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(
                    self._input_keyboard_mapping[event.input.name], dtype=np.float32
                )
        return True


def main():
    """
    Instantiate the robot runner with ROS2 + sensors.
    Supports Go2 (quadruped) and G1 (humanoid) robots.

    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot_type",
        type=str,
        default=ROBOT_GO2,
        choices=[ROBOT_GO2, ROBOT_G1],
        help="Robot type: go2 (quadruped) or g1 (humanoid)",
    )
    parser.add_argument(
        "--policy_dir",
        default=None,
        help="Policy directory (auto-detected based on robot_type if not set)",
    )
    parser.add_argument("--cmd_vel_topic", default="/cmd_vel")
    parser.add_argument(
        "--ros_namespace",
        default="",
        help="ROS namespace for the camera topics, e.g. /ferox/go2_01. Empty => "
        "derived from --cmd_vel_topic by stripping a trailing /cmd_vel.",
    )
    parser.add_argument("--vx_max", type=float, default=1.0)
    parser.add_argument("--vy_max", type=float, default=1.0)
    parser.add_argument("--wz_max", type=float, default=1.0)
    parser.add_argument(
        "--robot_root",
        default=None,
        help="Robot prim path (auto-detected based on robot_type if not set)",
    )
    parser.add_argument("--cmd_vel_only", action="store_true", default=False)
    parser.add_argument(
        "--no_sensors", action="store_true", help="Disable sensor setup"
    )
    parser.add_argument(
        "--no_keyboard", action="store_true", help="Disable keyboard control"
    )
    # TWIN MODE. Publishes the hardware interface from isaac/twin/<robot>_contract.yaml
    # instead of the legacy root-namespace topics and invented frames. Env TWIN=1 is
    # equivalent, so scripts/01_start_sim.sh can pass it through without an argv change.
    parser.add_argument(
        "--twin", action="store_true",
        default=os.environ.get("TWIN", "0") == "1",
        help="publish the hardware-shaped twin interface (contract-driven) instead "
             "of the legacy sim topics")
    parser.add_argument("--real_time", action="store_true", default=False)
    parser.add_argument("--physics_dt", type=float, default=1 / 200.0)
    parser.add_argument("--render_dt", type=float, default=1 / 60.0)
    args, _ = parser.parse_known_args()

    # Set defaults based on robot type
    if args.policy_dir is None:
        args.policy_dir = (
            DEFAULT_G1_POLICY_DIR
            if args.robot_type == ROBOT_G1
            else DEFAULT_GO2_POLICY_DIR
        )
    if args.robot_root is None:
        args.robot_root = "/World/G1" if args.robot_type == ROBOT_G1 else "/World/Go2"

    # Under G1_CONTROL=lowcmd the physics rate is part of the wire contract, not a
    # performance knob. rt/lowstate.tick is a MILLISECOND counter (measured 1000.064/s
    # on the robot -- docs/mm/evidence/MM3/PREREQS.md), so physics must step in whole
    # milliseconds or the published tick advances in twos and stops looking like the
    # robot's +1. But tick fidelity turned out to be the SMALLER reason.
    #
    # The twin's publishers decimate an INTEGER number of steps off the physics and
    # render clocks, so physics_dt decides which contract rates are even reachable --
    # the same arithmetic that produced C-15 and C-16 on the Go2. Measured in sim time
    # under G1_CONTROL=lowcmd (docs/mm/evidence/MM3/ros_topics_under_lowcmd.txt):
    #
    #   500 Hz physics : /livox/imu 500/3 = 166.67 against 200 (-16.7%, FAIL)
    #                    imu/data   500/6 =  83.33 against 93.7 (-11.1%, FAIL)
    #                    /tf        500/10 = 50.00 against 100  (-50.0%, FAIL)
    #  1000 Hz physics : 1000/5 = 200 exactly, 1000/11 = 90.9 (-3.0%), 1000/10 = 100
    #                    exactly -- every rate lands, because 1000 is a multiple of
    #                    both 200 and 100 and 500 is not.
    #
    # So 1000 Hz is the arithmetically correct choice and 500 Hz silently breaks three
    # interfaces that already passed. PD still runs at the 500 Hz CAMPAIGN 4.2 asks
    # for, applied every 2nd physics step -- pd_apply_hz and the physics rate are
    # separate numbers, exactly like the two lowstate rates.
    #
    # The cost is real-time factor (~0.30 at 1000 Hz, 0.73 at 500). That moves WALL
    # rates only; the twin's sensors are stamped and gated in SIM time, which is why
    # `ros2 topic hz` reads every topic low by the same factor and is the wrong
    # instrument here. It does mean rt/lowstate, which IS wall-paced, repeats state
    # between physics steps -- measured and declared as C-29, not hidden.
    # "handoff" must take this branch too. It did not, and every hand-off run silently
    # ran at the 200 Hz default with PD at 200 Hz -- the exact regime C-35 measured as
    # unstable -- which looked like SONIC tripping its own 35 rad/s velocity guard for
    # mysterious reasons rather than like a physics rate that was never applied.
    if os.environ.get("G1_CONTROL", "policy").strip().lower() in ("lowcmd", "handoff") \
            and "--physics_dt" not in sys.argv:
        args.physics_dt = 1.0 / float(os.environ.get("G1_PHYSICS_HZ", "1000"))
        logger.info("G1_CONTROL=lowcmd -> physics_dt %.6f s (%.0f Hz)",
                    args.physics_dt, 1.0 / args.physics_dt)

    logger.info("Running %s robot simulation", args.robot_type.upper())

    from isaacsim.core.utils import extensions

    extensions.enable_extension("isaacsim.ros2.bridge")
    extensions.enable_extension("isaacsim.sensors.physics")
    extensions.enable_extension("isaacsim.sensors.rtx")
    simulation_app.update()

    try:
        print("[PANTHERA-MARK] before RobotRosRunner()", flush=True)
        # Camera namespace: explicit --ros_namespace, else derive from the
        # cmd_vel topic (which already carries /ferox/<robot_id>/).
        ros_ns = args.ros_namespace
        if not ros_ns and args.cmd_vel_topic.endswith("/cmd_vel"):
            ros_ns = args.cmd_vel_topic[: -len("/cmd_vel")]
        print(f"[PANTHERA-MARK] camera ros_namespace = {ros_ns!r}", flush=True)
        runner = RobotRosRunner(
            physics_dt=args.physics_dt,
            render_dt=args.render_dt,
            policy_dir=args.policy_dir,
            cmd_vel_topic=args.cmd_vel_topic,
            vx_max=args.vx_max,
            vy_max=args.vy_max,
            wz_max=args.wz_max,
            robot_root=args.robot_root,
            cmd_vel_only=args.cmd_vel_only,
            enable_sensors=not args.no_sensors,
            enable_keyboard=not args.no_keyboard,
            robot_type=args.robot_type,
            ros_namespace=ros_ns,
            twin=args.twin,
        )
        print("[PANTHERA-MARK] RobotRosRunner constructed, before reset", flush=True)
        simulation_app.update()
        print("[PANTHERA-MARK] A: app.update done, calling _world.reset()", flush=True)
        runner._world.reset()
        print("[PANTHERA-MARK] B: world.reset() done", flush=True)
        simulation_app.update()
        print("[PANTHERA-MARK] C: app.update done, calling setup()", flush=True)
        runner.setup()
        print("[PANTHERA-MARK] D: setup() done", flush=True)
        simulation_app.update()
        print("[PANTHERA-MARK] E: app.update done, SKIPPING setup_ros() entirely", flush=True)
        runner.setup_ros()
        print("[PANTHERA-MARK] F: setup_ros() done", flush=True)
        simulation_app.update()
        print("[PANTHERA-MARK] G: before runner.run() — entering main loop", flush=True)
        runner.run(real_time=args.real_time)
        print("[PANTHERA-MARK] H: runner.run() returned", flush=True)
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
