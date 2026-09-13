"""CPU contracts for the new source-specific task; no simulated success claims."""
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import random
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from isaac.twin.inspire.whiteboard_scene import SceneConfig
from isaac.twin.isaaclab.g1_inspire import (InspireLabSpec, evaluate_snapshot,
                                         transition_record, TransitionWriter)


def fixture():
    body = [f"body_{i}" for i in range(29)]
    roots = [f"{side}_root_{i}" for side in ("left", "right") for i in range(6)]
    children = [f"child_{i}" for i in range(12)]
    names = body + roots + children
    random.Random(3).shuffle(names)
    asset = {"joint_limits": {n: {"lower": -1., "upper": 1., "effort": 5.} for n in names},
        "body_joint_names": body, "hand_independent_names": roots,
        "mimic_map": {c: {"parent": p, "multiplier": -.5, "offset": 0.} for c, p in zip(children, roots)},
        "fixed_base": True, "source_sha256": "a" * 64,
        "collision_candidates": {s: {"candidate_id": s + "_fixture"} for s in ("left", "right")}}
    profile = {"body_home_rad": {n: .1 for n in body}, "hardware_authorized": False,
        "kp_nm_rad": {n: 20. + i for i, n in enumerate(body)}, "kd_nm_s_rad": {n: 2. for n in body}}
    return asset, profile, asdict(SceneConfig())


class NamedLabTests(unittest.TestCase):
    def setUp(self):
        self.asset, self.profile, self.scene = fixture()
        self.spec = InspireLabSpec.from_manifests(self.asset, self.profile, self.scene, episode_steps=12)
        self.root = [0., 0., 1., 1., 0., 0., 0.] + [0.] * 6

    def test_cooked_candidate_and_inertia_metadata_change_export_identity(self):
        asset=deepcopy(self.asset)
        asset['thumb_collision_candidates']={'left':{'candidate_id':'source_thumb_v1'}}
        revised=InspireLabSpec.from_manifests(asset,self.profile,self.scene,episode_steps=12)
        self.assertNotEqual(revised.sha256,self.spec.sha256)
        self.assertIn(('left_thumb2','source_thumb_v1'),revised.candidate_ids)
        asset['source_inertial_frame_correction']={'version':'source_preserving_v1'}
        corrected=InspireLabSpec.from_manifests(asset,self.profile,self.scene,episode_steps=12)
        self.assertNotEqual(corrected.asset_manifest_sha256,revised.asset_manifest_sha256)
        self.assertEqual(corrected.action_names,revised.action_names)

    def test_shuffled_runtime_binds_41_independent_targets_and_53_measurements(self):
        runtime = list(reversed(self.spec.measurement_names))
        binding = self.spec.bind(runtime)
        self.assertEqual(tuple(runtime[i] for i in binding["measurement_indices"]), self.spec.measurement_names)
        self.assertEqual(tuple(runtime[i] for i in binding["action_indices"]), self.spec.action_names)
        self.assertEqual(len(self.spec.action_names), 41)
        self.assertEqual(self.spec.observation_width, 119)
        with self.assertRaises(ValueError): self.spec.bind(runtime[:-1] + [runtime[0]])
        with self.assertRaises(ValueError): self.spec.validate_action([0.] * 53)
        with self.assertRaises(ValueError): self.spec.validate_action([True] * 41)

    def test_reset_gains_and_effort_ownership_match_source_not_legacy_defaults(self):
        for i, name in enumerate(self.spec.measurement_names):
            if name in self.spec.body_names:
                self.assertEqual(self.spec.stiffness[i], self.profile["kp_nm_rad"][name])
            elif name in self.spec.hand_root_names:
                self.assertEqual((self.spec.stiffness[i], self.spec.damping[i]), (1., .05))
            else:
                self.assertEqual((self.spec.stiffness[i], self.spec.damping[i], self.spec.drive_effort_limits[i]), (0., 0., 0.))
                self.assertEqual(self.spec.source_effort_limits[i], 5.)
        self.assertEqual(self.spec.scene_config, asdict(SceneConfig.from_dict(self.scene)))
        result = evaluate_snapshot(self.spec, self.spec.reset_positions, [0.] * 53, self.root, self.spec.reset_action(), 0)
        self.assertEqual(result["reward"], 0.)
        self.assertFalse(result["terminated"])

    def test_mimic_cycles_and_inconsistent_reset_fail_before_runtime(self):
        asset = deepcopy(self.asset)
        asset["mimic_map"]["child_0"]["parent"] = "child_1"
        asset["mimic_map"]["child_1"]["parent"] = "child_0"
        with self.assertRaises(ValueError): InspireLabSpec.from_manifests(asset, self.profile, self.scene)
        asset = deepcopy(self.asset)
        asset["mimic_map"]["child_0"]["offset"] = .1
        with self.assertRaises(ValueError): InspireLabSpec.from_manifests(asset, self.profile, self.scene)

    def test_scalar_and_batch_reward_termination_reset_parity(self):
        q_bad = list(self.spec.reset_positions)
        q_bad[self.spec.measurement_names.index("child_0")] = .1
        cases = [(list(self.spec.reset_positions), 0), (list(self.spec.reset_positions), 12), (q_bad, 1)]
        batch = [evaluate_snapshot(self.spec, q, [0.] * 53, self.root, self.spec.reset_action(), step) for q, step in cases]
        for (q, step), expected in zip(cases, batch):
            self.assertEqual(expected, evaluate_snapshot(self.spec, q, [0.] * 53, self.root, self.spec.reset_action(), step))
        self.assertFalse(batch[0]["terminated"])
        self.assertTrue(batch[1]["truncated"])
        self.assertIn("mimic_coupling_violation", batch[2]["failures"])
        nan = list(self.spec.reset_positions); nan[0] = float("nan")
        self.assertTrue(evaluate_snapshot(self.spec, nan, [0.] * 53, self.root, self.spec.reset_action(), 1)["terminated"])

    def test_exports_preserve_failure_and_invalid_telemetry_deterministically(self):
        q = list(self.spec.reset_positions); q[0] = float("nan")
        evaluation = evaluate_snapshot(self.spec, q, [0.] * 53, self.root, self.spec.reset_action(), 1)
        record = transition_record(self.spec, env_id=0, episode_id=0, sequence=0, physics_time_s=.02,
            q=q, dq=[0.] * 53, root_state=self.root, action=self.spec.reset_action(), evaluation=evaluation)
        self.assertFalse(record["q_rad"]["valid"])
        self.assertIsNone(record["q_rad"]["values"][0])
        self.assertFalse(record["contacts"]["valid"])
        self.assertIsNone(record["contacts"]["forces_world_n"])
        with tempfile.TemporaryDirectory() as directory:
            paths = [Path(directory) / name for name in ("a.jsonl", "b.jsonl")]
            for path in paths:
                writer = TransitionWriter(path); writer.append(record); writer.close()
            self.assertEqual(paths[0].read_bytes(), paths[1].read_bytes())
            self.assertTrue(json.loads(paths[0].read_text())["evaluation"]["terminated"])
            with self.assertRaises(FileExistsError): TransitionWriter(paths[0])


if __name__ == "__main__": unittest.main()
