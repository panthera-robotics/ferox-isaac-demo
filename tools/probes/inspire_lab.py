"""Bounded Isaac Lab plumbing check using the same assembled donor and scene.

Requires the pinned, cached Lab runtime image. No policy weights, training,
hardware calibration, or network endpoints are embedded in this public probe.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

assert os.environ.get("PANTHERA_SIM_AUTHORIZED") == "1"
assert sorted(p.name for p in Path("/sys/class/net").iterdir()) == ["lo"]
assert os.environ.get("PANTHERA_PROBE_MODE") == "inspire-lab-debug"
sys.path.insert(0, "/workspace/sim-source")
sys.path.insert(0, "/workspace/ferox_tools")
from isaac.twin.isaaclab.g1_inspire import (InspireLabSpec, TransitionWriter,
                                          evaluate_snapshot, LAB_SOURCE_SHA)
from isaac.twin.inspire.whiteboard_scene import SceneConfig

profile_path = Path(os.environ["PANTHERA_PROBE_CONFIG"])
profile = json.loads(profile_path.read_text())
assert profile["schema_version"] == 1 and profile["hardware_authorized"] is False
settings = profile["lab"]
assert set(settings) <= {"num_envs", "steps", "episode_steps", "seed", "scene_config"}
num_envs, steps = settings["num_envs"], settings["steps"]
episode_steps, seed = settings["episode_steps"], settings.get("seed", 17)
assert type(num_envs) is int and 1 <= num_envs <= 8
assert type(steps) is int and 4 <= steps <= 200
assert type(episode_steps) is int and 2 <= episode_steps < steps
assert type(seed) is int and 0 <= seed <= 2**31 - 1
scene = SceneConfig.from_dict(settings["scene_config"])
actual_lab_sha = subprocess.check_output(["git", "-c", "safe.directory=/workspace/IsaacLab",
    "-C", "/workspace/IsaacLab", "rev-parse", "HEAD"], text=True).strip()
assert actual_lab_sha == LAB_SOURCE_SHA, (actual_lab_sha, LAB_SOURCE_SHA)

from isaaclab.app import AppLauncher
launcher = AppLauncher({"headless": True, "device": "cpu", "enable_cameras": False})
app = launcher.app
import numpy as np
import torch
import omni.usd
from omni.physx import get_physxunittests_interface
from pxr import PhysxSchema, UsdPhysics
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.pip.compute")
from inspire_body_asset import import_body
from inspire_collision import replace_palm_with_components
from isaac.twin.isaaclab.inspire_env import create_env

out = Path("/evidence")
source = Path("/source-assets/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf")
def palm_builder(stage, mesh, body, side):
    return replace_palm_with_components(stage, mesh, body,
        contact_offset_m=.0012860533315688372, rest_offset_m=0.,
        candidate_id="ftp_palm_yz_slabs_v2" if side == "right" else "ftp_left_palm_yz_slabs_v1")

asset, facts = import_body(source, out, fixed_base=True, palm_builder=palm_builder)
spec = InspireLabSpec.from_manifests(facts, profile, settings["scene_config"], episode_steps=episode_steps)
omni.usd.get_context().new_stage()
env = create_env(spec, asset, num_envs=num_envs, seed=seed)
observation, extras = env.reset(seed=seed)
assert observation["policy"].shape == (num_envs, 119)
assert env.robot.num_joints == 53 and env.single_action_space.shape == (41,)
assert set(env.robot.body_names) == set(facts["physical_link_mass_kg"])
ids = env.measurement_ids
live = env.robot.root_physx_view
readbacks = {
    "stiffness_nm_rad": live.get_dof_stiffnesses()[:, ids].cpu().tolist(),
    "damping_nm_s_rad": live.get_dof_dampings()[:, ids].cpu().tolist(),
    "independent_drive_effort_limit_nm": live.get_dof_max_forces()[:, ids].cpu().tolist(),
    "body_names": list(env.robot.body_names), "mass_kg": live.get_masses().cpu().tolist()}
for key, expected in [("stiffness_nm_rad", spec.stiffness), ("damping_nm_s_rad", spec.damping),
                      ("independent_drive_effort_limit_nm", spec.drive_effort_limits)]:
    assert np.allclose(readbacks[key], [expected] * num_envs, rtol=1e-6, atol=1e-6), key
expected_mass = [facts["physical_link_mass_kg"][n] for n in env.robot.body_names]
assert np.allclose(readbacks["mass_kg"], [expected_mass] * num_envs, atol=1e-5, rtol=1e-6)
marker_live = env.marker.root_physx_view
readbacks["marker_spring"] = {
    "stiffness_n_m": marker_live.get_dof_stiffnesses().cpu().tolist(),
    "damping_n_s_m": marker_live.get_dof_dampings().cpu().tolist(),
    "force_limit_n": marker_live.get_dof_max_forces().cpu().tolist(),
    "target_m": env.marker.data.joint_pos_target.cpu().tolist()}
assert np.allclose(readbacks["marker_spring"]["stiffness_n_m"], scene.holder.stiffness_n_m)
assert np.allclose(readbacks["marker_spring"]["target_m"], scene.holder.spring_target_m)
readbacks["palms"] = []
for environment in range(num_envs):
    for side in ("left", "right"):
        path = f"/World/envs/env_{environment}/Robot/{side}_base_link"
        rigid = env.sim.physics_sim_view.create_rigid_body_view(path)
        expected = facts["collision_candidates"][side]["expected_palm_hulls_if_all_cooking_succeeds"]
        assert rigid.count == 1 and rigid.max_shapes == expected, (path, rigid.count, rigid.max_shapes, expected)
        readbacks["palms"].append({"path": path, "live_shapes": rigid.max_shapes,
            "contact_offsets_m": rigid.get_contact_offsets().cpu().tolist(),
            "rest_offsets_m": rigid.get_rest_offsets().cpu().tolist()})
(out / "lab_runtime_contract.json").write_text(json.dumps(readbacks, indent=2, allow_nan=False))

writer = TransitionWriter(out / "transitions.jsonl")
resets = [0] * num_envs
failures, parity_errors = [], []
invalid_state_records = 0
invalid_contact_records = 0
base_action = list(spec.reset_action())
for step in range(steps):
    # Small diagnostic motion in a source-validated range; no reaching/grasp reward.
    action = list(base_action)
    for index in range(29, 41):
        name = spec.action_names[index]
        lo, hi = dict(zip(spec.measurement_names, spec.limits))[name]
        action[index] = min(hi, max(lo, .01 * (1. - math.cos(2. * math.pi * step / 20.))))
    action_tensor = torch.tensor(action, dtype=torch.float32, device=env.device).repeat(num_envs, 1)
    observation, reward, terminated, truncated, extras = env.step(action_tensor)
    assert observation["policy"].shape == (num_envs, 119)
    assert len(env.last_transition_records) == num_envs
    for i, record in enumerate(env.last_transition_records):
        # Re-evaluate the exported pre-reset measurements. Returned observations
        # may already belong to the next episode and must never replace these.
        expected = evaluate_snapshot(spec, record["q_rad"]["values"], record["dq_rad_s"]["values"],
            record["root_state_world_p_qwxyz_v_w"]["values"], record["command_position_rad"]["values"], record["episode_step"])
        same = expected == record["evaluation"] and bool(terminated[i]) == expected["terminated"] and bool(truncated[i]) == expected["truncated"]
        same = same and math.isclose(float(reward[i]), expected["reward"], rel_tol=1e-5, abs_tol=1e-6)
        if not same: parity_errors.append({"step": step, "env": i})
        if expected["failures"]: failures.append({"step": step, "env": i, "failures": expected["failures"]})
        invalid_state_records += int(not all(record[k]["valid"] for k in ["q_rad", "dq_rad_s", "root_state_world_p_qwxyz_v_w"]))
        invalid_contact_records += int(not record["contacts"]["valid"])
        resets[i] += int(bool(terminated[i]) or bool(truncated[i]))
        writer.append(record)
writer.close()
statistics = get_physxunittests_interface().get_physics_stats()
physics_scene = next(p for p in env.sim.stage.Traverse() if p.IsA(UsdPhysics.Scene))
physics_api = PhysxSchema.PhysxSceneAPI(physics_scene)
checks = {"named53_measured_41_independent": True, "source_mass_gains_effort_preserved": True,
    "bilateral_live_palm_shapes": True, "exact_transition_count": writer.count == num_envs * steps,
    "scalar_export_runtime_reward_done_parity": not parity_errors, "automatic_resets_observed_each_env": all(resets),
    "finite_actual_states": invalid_state_records == 0, "valid_contact_measurements": invalid_contact_records == 0,
    "mechanism_thresholds": not failures, "no_triangle_hand_substitution": statistics["numTriMeshShapes"] == 0}
metrics = {"checks": checks, "lab_commit": actual_lab_sha, "spec_sha256": spec.sha256,
    "profile_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(), "scene_config_sha256": scene.sha256,
    "asset_source_sha256": facts["source_sha256"], "collision_candidate_ids": spec.candidate_ids,
    "measurement_names": spec.measurement_names, "action_names": spec.action_names, "observation_width": 119,
    "num_envs": num_envs, "seed": seed, "steps": steps, "automatic_reset_counts": resets,
    "physics_dt_s": spec.physics_dt, "control_dt_s": spec.physics_dt * spec.decimation,
    "physx_gpu_dynamics_enabled": physics_api.GetEnableGPUDynamicsAttr().Get(),
    "physx_statistics": statistics, "failures": failures, "parity_errors": parity_errors,
    "physics_scope": "fixed_pelvis_source_assembly_and_free_dynamic_marker_debug",
    "training_run": False, "checkpoint_created": False, "exact_asset_qualified": False,
    "standing_qualified": False, "grasp_qualified": False, "writing_qualified": False,
    "contact_scope": "synthetic_net_rigid_body_forces; no hardware taxels or pair impulses",
    "runtime_parity_scope": "shared evaluation versus actual exported pre-reset state; not bitwise independent-physics equivalence"}
(out / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False))
env.sim.stage.GetRootLayer().Export(str(out / "lab_scene.usda"))
artifacts = [str(p.relative_to(out)) for p in out.rglob("*") if p.is_file() and p.name not in
    {"run.json", "probe.json", "console.log", "executed_probe.py", "executed_launcher.py", "uncommitted.patch"}]
(out / "probe.json").write_text(json.dumps({"status": "PASS" if all(checks.values()) else "FAIL",
    "scope": "Isaac_Lab_source_specific_debug_only", "metrics": "metrics.json", "artifacts": artifacts}))
env.close()
app.close()
