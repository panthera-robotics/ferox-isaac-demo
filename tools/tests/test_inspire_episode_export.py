"""Round-trip harness for the Inspire-native episode export (schema inspire_episode_v1) on a SYNTHETIC run directory."""
import json
import tempfile
import unittest
from pathlib import Path

from isaac.twin.inspire.embodiment import EmbodimentManifest, HandCommandAdapter, HAND_ACTUATORS
from tools.export_inspire_episode import DATASET_HAND_ORDER, SCHEMA, export, radian_contract

MANIFEST = Path(__file__).resolve().parents[2] / 'isaac/twin/inspire/embodiments/g1_edu29_rh56dftp_donor_v1.json'


class EpisodeExportRoundTrip(unittest.TestCase):
    def _synthetic_run(self, root, manifest):
        body = list(manifest.body_names); hands = [manifest.hand_actuator(s, a)['joint'] for s in ('left', 'right') for a in HAND_ACTUATORS]
        coupled = ['right_thumb_3_joint', 'right_index_2_joint']; names = body + hands + coupled
        q = {n: 0.0 for n in names}; q['right_index_1_joint'] = 0.7; q['right_thumb_2_joint'] = 0.3; q['right_thumb_1_joint'] = 0.1; q['right_elbow_joint'] = 0.5
        cmd_hand = {n: 0.0 for n in hands}; cmd_hand['right_index_1_joint'] = 1.0; cmd_hand['right_thumb_2_joint'] = 0.35
        run = root / 'run'; run.mkdir(); (run / 'frames.jsonl').write_text('')
        rows = [{'sequence': i, 'physics_s': 0.005 * (i + 1), 'wall_s': 0.01 * i, 'phase': 'lead_in' if i == 0 else 'replay', 'source_row': None if i == 0 else i - 1, 'source_t_s': None if i == 0 else 0.02 * (i - 1),
                 'runtime_names': names, 'q_rad': [q[n] for n in names], 'dq_rad_s': [0.0] * len(names), 'body_command_names': body, 'body_command_rad': [0.1] * len(body),
                 'hand_command_names': hands, 'hand_command_rad': [cmd_hand[n] for n in hands]} for i in range(3)]
        (run / 'state.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
        (run / 'object.jsonl').write_text(json.dumps({'sequence': 1, 'physics_s': 0.01, 'phase': 'replay', 'source_row': 0, 'pose_world_xyzw': [0.5, 0, 1.2, 0, 0, 0, 1], 'linear_velocity_m_s': [0, 0, 0]}) + '\n')
        (run / 'contacts.jsonl').write_text(json.dumps({'sequence': 1, 'actor0': '/World/G1/right_index_2', 'actor1': '/World/Scene/Object'}) + '\n' + json.dumps({'sequence': None, 'actor0': '/World/Scene/Object', 'actor1': '/World/Scene/Table'}) + '\n')
        (run / 'metrics.json').write_text(json.dumps({'physics_dt': 0.005, 'support_constraints': ['pelvis_fixed_to_world_1m_above_origin'], 'scene': {'table': {}}, 'media_labels': {'fixture': 'FIXED PELVIS - SCRIPTED BASELINE'}}))
        (run / 'run.json').write_text(json.dumps({'run_id': 'synthetic'}))
        pkg = root / 'pkg'; pkg.mkdir()
        (pkg / 'sequence.json').write_text(json.dumps({'source': {'source_id': 'synthetic', 'kind': 'synthetic_test_sequence'}, 'rows': []}))
        (pkg / 'controller.json').write_text('{}'); (pkg / 'manifest.json').write_text(MANIFEST.read_text())
        return run, pkg, q, cmd_hand

    def test_load_normalize_denormalize_decode_round_trip(self):
        m = EmbodimentManifest.load(MANIFEST)
        with tempfile.TemporaryDirectory() as d:
            run, pkg, q, cmd_hand = self._synthetic_run(Path(d), m)
            episode, rows = export(run, pkg, MANIFEST, Path(d) / 'out', language='pick up the cylinder.')
            self.assertEqual(episode['schema'], SCHEMA); self.assertEqual(episode['frames'], 2); self.assertIn('SIMULATION', episode['origin'])
            obs = rows[0]['observation']; right = obs['hand_state_dataset_order_rad']['right']
            self.assertEqual(len(right), 6); self.assertEqual(list(DATASET_HAND_ORDER)[3], 'index')
            self.assertAlmostEqual(right[3], 0.7); self.assertAlmostEqual(right[4], 0.3); self.assertAlmostEqual(right[5], 0.1); self.assertAlmostEqual(right[0], 0.0)   # identity radians in dataset order
            adapter = HandCommandAdapter(m, 'right', radian_contract(m, 'right'))
            targets, info = adapter.to_joint_targets(right); self.assertEqual(info['clipped_axes'], [])
            for a in HAND_ACTUATORS:
                j = m.hand_actuator('right', a)['joint']; self.assertAlmostEqual(targets[j], q[j], places=9)   # denormalize reproduces the joint state
            issued = rows[0]['issued_targets']['hand_target_dataset_order_rad']['right']
            self.assertAlmostEqual(issued[3], 1.0); self.assertAlmostEqual(issued[4], 0.35)
            self.assertEqual(rows[0]['observation']['hand_object_contact_links'], ['right_index_2']); self.assertIsNone(rows[1]['observation']['object_pose_world_xyzw'])
            self.assertEqual(rows[0]['observation']['object_pose_world_xyzw'][0], 0.5)
            reloaded = [json.loads(l) for l in (Path(d) / 'out' / 'frames.jsonl').open()]
            self.assertEqual(reloaded[0]['observation']['hand_state_dataset_order_rad']['right'], right)   # decode after load is bit-identical
            self.assertTrue((Path(d) / 'out' / 'episode.json').exists())


if __name__ == '__main__':
    unittest.main()
