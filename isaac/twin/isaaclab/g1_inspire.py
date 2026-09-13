"""Named G1/FTP Isaac Lab contracts, shared with the measured source scene.

No Isaac Lab import is required for the data, reset/evaluation, or export
contracts. Runtime builders consume an already prepared USD and its measured
asset manifest; they do not substitute a generic G1 or the historical Dex5 asset.
"""
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path

from isaac.twin.inspire.whiteboard_scene import SceneConfig


LAB_SOURCE_SHA = "37ddf626871758333d6ed89cf64ad702aef127d0"
EXPORT_SCHEMA = "panthera_inspire_lab_transition_v1"


def _finite(values, width, label):
    if len(values) != width:
        raise ValueError(f"{label}: expected {width} values")
    if any(type(v) not in (int, float) for v in values):
        raise ValueError(label + ": numeric values required")
    result = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in result):
        raise ValueError(label + ": nonfinite values")
    return result


@dataclass(frozen=True)
class InspireLabSpec:
    measurement_names: tuple
    body_names: tuple
    hand_root_names: tuple
    mimic_entries: tuple
    limits: tuple
    reset_positions: tuple
    stiffness: tuple
    damping: tuple
    drive_effort_limits: tuple
    source_effort_limits: tuple
    fixed_base: bool
    scene_config: dict
    asset_source_sha256: str
    candidate_ids: tuple
    asset_manifest_sha256: str = ''
    physics_dt: float = .005
    decimation: int = 4
    episode_steps: int = 100
    limit_tolerance_rad: float = .03
    coupling_tolerance_rad: float = .03
    minimum_root_height_m: float = .45

    @classmethod
    def from_manifests(cls, asset, profile, scene_config, *, episode_steps=100):
        scene = SceneConfig.from_dict(scene_config)
        if scene.holder_mode != "free_dynamic":
            raise ValueError("Lab debug task requires the free dynamic marker, without carriage support")
        names = tuple(asset["joint_limits"])
        body = tuple(profile["body_home_rad"])
        roots = tuple(asset["hand_independent_names"])
        mimics = asset["mimic_map"]
        if (len(names) != 53 or len(set(names)) != 53 or len(body) != 29 or len(roots) != 12
                or len(mimics) != 12 or set(names) != set(body) | set(roots) | set(mimics)
                or (set(body) & set(roots)) or (set(body) & set(mimics)) or (set(roots) & set(mimics))):
            raise ValueError("Expected disjoint named 29-body/12-hand-root/12-mimic ownership")
        if set(body) != set(asset["body_joint_names"]):
            raise ValueError("Body profile differs from the source embodiment")
        if profile.get("hardware_authorized") is not False:
            raise ValueError("Lab profile must remain simulation-only")
        if type(asset["fixed_base"]) is not bool:
            raise ValueError("Explicit Boolean source fixture state required")
        if type(episode_steps) is not int or not 2 <= episode_steps <= 10000:
            raise ValueError("Bounded episode length required")
        if abs(scene.physics_dt_s - .005) > 1e-12:
            raise ValueError("Shared scene must retain measured 5 ms physics")
        kp, kd, reset, drive, source_effort, limits = [], [], [], [], [], []
        for name in names:
            limit = asset["joint_limits"][name]
            lo, hi, effort = _finite([limit[k] for k in ("lower", "upper", "effort")], 3, name)
            if lo > hi or effort <= 0:
                raise ValueError("Invalid source limits: " + name)
            position = float(profile["body_home_rad"][name]) if name in body else 0.
            if not lo <= position <= hi:
                raise ValueError("Reset position violates source limit: " + name)
            reset.append(position); limits.append((lo, hi)); source_effort.append(effort)
            kp.append(float(profile["kp_nm_rad"][name]) if name in body else (1. if name in roots else 0.))
            kd.append(float(profile["kd_nm_s_rad"][name]) if name in body else (.05 if name in roots else 0.))
            drive.append(0. if name in mimics else effort)
        if not all(math.isfinite(v) and v >= 0 for v in kp + kd):
            raise ValueError("Invalid source body gains")
        entries = []
        for child, mimic in mimics.items():
            if mimic["parent"] not in set(roots) | set(mimics):
                raise ValueError("Mimic parent must remain within the explicit hand map")
            multiplier, offset = _finite([mimic["multiplier"], mimic["offset"]], 2, child)
            entries.append((child, mimic["parent"], multiplier, offset))
            visited = {child}
            parent = mimic["parent"]
            while parent in mimics:
                if parent in visited:
                    raise ValueError("Cyclic mimic ownership")
                visited.add(parent)
                parent = mimics[parent]["parent"]
        by_name = dict(zip(names, reset))
        if any(abs(by_name[c] - m * by_name[p] - o) > 1e-8 for c, p, m, o in entries):
            raise ValueError("Reset state violates source mimic coupling")
        candidates=tuple((side, asset["collision_candidates"][side]["candidate_id"]) for side in ("left", "right"))
        candidates+=tuple((side+"_thumb2", candidate["candidate_id"]) for side,candidate in sorted(asset.get("thumb_collision_candidates",{}).items()))
        return cls(names, body, roots, tuple(entries), tuple(limits), tuple(reset), tuple(kp), tuple(kd),
            tuple(drive), tuple(source_effort), bool(asset["fixed_base"]), asdict(scene), asset["source_sha256"],
            candidates, asset_manifest_sha256=hashlib.sha256(json.dumps(asset,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest(),
            episode_steps=episode_steps)

    @property
    def action_names(self):
        return self.body_names + self.hand_root_names

    @property
    def observation_width(self):
        return 2 * len(self.measurement_names) + 13

    @property
    def sha256(self):
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()

    def bind(self, runtime_names):
        actual = tuple(runtime_names)
        if len(actual) != len(set(actual)) or set(actual) != set(self.measurement_names):
            raise ValueError("Runtime named coordinates differ from the source asset")
        index = {name: i for i, name in enumerate(actual)}
        return {"measurement_indices": tuple(index[n] for n in self.measurement_names),
                "action_indices": tuple(index[n] for n in self.action_names)}

    def reset_action(self):
        by_name = dict(zip(self.measurement_names, self.reset_positions))
        return tuple(by_name[n] for n in self.action_names)

    def validate_action(self, values):
        action = _finite(values, 41, "named action")
        limits = dict(zip(self.measurement_names, self.limits))
        if any(not limits[n][0] <= v <= limits[n][1] for n, v in zip(self.action_names, action)):
            raise ValueError("Command violates a source joint limit")
        return action


def evaluate_snapshot(spec, q, dq, root_state, action, step):
    """One scalar definition used unchanged by CPU and batched Lab stepping.

    Reward is diagnostic target tracking, not grasp/writing success. Termination
    thresholds retain the mechanism's 0.03 rad limits/coupling acceptance.
    """
    failures = []
    try:
        q = _finite(q, 53, "q")
        _finite(dq, 53, "dq")
        root_state = _finite(root_state, 13, "root")
        action = spec.validate_action(action)
    except ValueError as exc:
        return {"reward": 0., "terminated": True, "truncated": False,
                "failures": [str(exc)], "limit_violation_rad": None, "coupling_error_rad": None}
    by_name = dict(zip(spec.measurement_names, q))
    limit_error = max(max(lo - value, value - hi, 0.) for value, (lo, hi) in zip(q, spec.limits))
    coupling = max(abs(by_name[child] - multiplier * by_name[parent] - offset)
                   for child, parent, multiplier, offset in spec.mimic_entries)
    if limit_error > spec.limit_tolerance_rad: failures.append("joint_limit_violation")
    if coupling > spec.coupling_tolerance_rad: failures.append("mimic_coupling_violation")
    if not spec.fixed_base and root_state[2] < spec.minimum_root_height_m: failures.append("root_below_height_floor")
    error = sum((by_name[name] - target) ** 2 for name, target in zip(spec.action_names, action)) / 41
    return {"reward": -error, "terminated": bool(failures), "truncated": step >= spec.episode_steps,
            "failures": failures, "limit_violation_rad": limit_error, "coupling_error_rad": coupling}


def transition_record(spec, *, env_id, episode_id, sequence, physics_time_s, q, dq,
                      root_state, action, evaluation, contact_forces=None, contact_names=()):
    """Deterministic, explicit export; unavailable contacts never become zeros."""
    def measured(values):
        finite = all(math.isfinite(float(x)) for x in values)
        return {"valid": finite, "values": [float(x) if math.isfinite(float(x)) else None for x in values]}
    contacts = None
    for values, width, label in [(q, 53, "q"), (dq, 53, "dq"), (root_state, 13, "root"), (action, 41, "action")]:
        if len(values) != width: raise ValueError(label + ": export width mismatch")
    if contact_forces is not None:
        if len(contact_forces) != len(contact_names):
            raise ValueError("Contact body-name/measurement width mismatch")
        if any(len(force) != 3 for force in contact_forces):
            raise ValueError("Contact force must have three world coordinates")
        contacts = [measured(force) for force in contact_forces]
    return {"schema": EXPORT_SCHEMA, "spec_sha256": spec.sha256, "env_id": int(env_id),
            "episode_id": int(episode_id), "sequence": int(sequence), "physics_time_s": float(physics_time_s),
            "measurement_joint_names": list(spec.measurement_names), "q_rad": measured(q), "dq_rad_s": measured(dq),
            "root_state_world_p_qwxyz_v_w": measured(root_state), "action_joint_names": list(spec.action_names),
            "command_position_rad": measured(action), "evaluation": evaluation,
            "contacts": {"source": "simulated_net_rigid_body_contact_force", "valid": contacts is not None and all(c["valid"] for c in contacts),
                "body_names": list(contact_names), "forces_world_n": contacts, "pair_impulses": None,
                "hardware_taxels": None, "hardware_calibration_verified": False},
            "exact_RH56E2_equivalence": False, "grasp_qualified": False, "writing_qualified": False,
            "state_phase": "post_physics_before_any_automatic_reset"}


class TransitionWriter:
    def __init__(self, path):
        self.path = Path(path)
        self.stream = self.path.open("x", buffering=1)
        self.count = 0
    def append(self, record):
        self.stream.write(json.dumps(record, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n")
        self.count += 1
    def close(self): self.stream.close()


def build_articulation_cfg(spec, usd_path):
    """Preserve source physics, with exact named gains/caps shared by the bench."""
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    import isaaclab.sim as sim_utils
    return ArticulationCfg(prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=str(usd_path), activate_contact_sensors=True,
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=True,
                solver_position_iteration_count=32, solver_velocity_iteration_count=8)),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0., 0., 1. if spec.fixed_base else .8),
            joint_pos=dict(zip(spec.measurement_names, spec.reset_positions)), joint_vel={".*": 0.}),
        soft_joint_pos_limit_factor=1.,
        actuators={"source_named_drives": ImplicitActuatorCfg(joint_names_expr=list(spec.measurement_names),
            stiffness=dict(zip(spec.measurement_names, spec.stiffness)),
            damping=dict(zip(spec.measurement_names, spec.damping)),
            effort_limit_sim=dict(zip(spec.measurement_names, spec.drive_effort_limits)),
            armature=None, friction=None, dynamic_friction=None, viscous_friction=None)})
