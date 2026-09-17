#!/usr/bin/env python3
"""Export one executed command_replay run as a versioned Inspire-native episode (schema inspire_episode_v1).

Observations (what the twin measured) and issued targets (what the controller was asked) are kept in separate
fields; nothing is relabelled. Per frame: timestamps (physics, source row time), body joint positions/velocities by
name (29), hand joint positions by name (12 independent + coupled), the six-actuator hand state per side in the
dataset order (pinky, ring, middle, index, thumb_pitch, thumb_yaw) in donor radians, issued body/hand targets, object
pose/velocity, hand–object contact links, policy-camera frame path, language. Episode-level: origin (simulation),
donor identity, camera profile, scene, task criteria result, hashes of the source traces, and the hand contract.
Round trip: ``load -> normalize (HandCommandAdapter.from_joint_state) -> denormalize (to_joint_targets)`` is tested.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'isaac' / 'twin'))
from inspire.embodiment import ContractError, EmbodimentManifest, HandCommandAdapter, HAND_ACTUATORS  # noqa: E402

SCHEMA = 'inspire_episode_v1'
DATASET_HAND_ORDER = ('little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation')


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def _maybe(adapter, joint_values):
    try:
        return adapter.from_joint_state(joint_values)
    except ContractError:
        return None


def radian_contract(manifest, side='right'):
    limits = {a: manifest.hand_actuator(side, a)['closed_rad'] for a in HAND_ACTUATORS}
    return {'axis_order': list(DATASET_HAND_ORDER), 'open_value': 0.0, 'closed_value': 1.0, 'per_axis_endpoints': {a: {'open_value': 0.0, 'closed_value': limits[a]} for a in HAND_ACTUATORS}, 'saturation_policy': 'clip_declared'}


def export(run: Path, package: Path, manifest_path: Path, out: Path, *, language: str, task_eval: Path | None = None, split: str = 'unassigned'):
    manifest = EmbodimentManifest.load(manifest_path)
    adapters = {s: HandCommandAdapter(manifest, s, radian_contract(manifest, s)) for s in ('left', 'right')}
    metrics = json.loads((run / 'metrics.json').read_text()); receipt = json.loads((run / 'run.json').read_text())
    sequence = json.loads((package / 'sequence.json').read_text())
    state = [json.loads(l) for l in (run / 'state.jsonl').open()]
    frames = {f['sequence']: f for f in map(json.loads, (run / 'frames.jsonl').open())} if (run / 'frames.jsonl').exists() else {}
    objects = {o['sequence']: o for o in map(json.loads, (run / 'object.jsonl').open())} if (run / 'object.jsonl').exists() else {}
    contacts = {}
    if (run / 'contacts.jsonl').exists():
        for c in map(json.loads, (run / 'contacts.jsonl').open()):
            if c['sequence'] is None:
                continue
            a0, a1 = c['actor0'], c['actor1']
            if '/World/Scene/Object' in (a0, a1):
                other = a1 if a0.endswith('/Object') else a0
                contacts.setdefault(c['sequence'], set()).add(other.split('/')[-1])
    names = state[0]['runtime_names']; body_names = state[0]['body_command_names']; hand_names = state[0]['hand_command_names']
    coupled = [n for n in names if n not in body_names and n not in hand_names]
    out.mkdir(parents=True, exist_ok=False)
    rows = []
    with (out / 'frames.jsonl').open('w') as f:
        for r in state:
            if r['phase'] != 'replay':
                continue
            q = dict(zip(names, r['q_rad'])); dq = dict(zip(names, r['dq_rad_s']))
            hand_obs = {s: adapters[s].from_joint_state(q) for s in ('left', 'right')}   # donor radians in the dataset order (identity endpoints)
            issued_hand = dict(zip(hand_names, r['hand_command_rad']))
            row = {'physics_s': r['physics_s'], 'source_row': r['source_row'], 'source_t_s': r['source_t_s'], 'wall_s': r['wall_s'],
                   'observation': {'body_q_rad': {n: q[n] for n in body_names}, 'body_dq_rad_s': {n: dq[n] for n in body_names}, 'hand_joint_q_rad': {n: q[n] for n in hand_names + coupled},
                                   'hand_state_dataset_order_rad': hand_obs, 'object_pose_world_xyzw': objects.get(r['sequence'], {}).get('pose_world_xyzw'), 'object_linear_velocity_m_s': objects.get(r['sequence'], {}).get('linear_velocity_m_s'),
                                   'hand_object_contact_links': sorted(contacts.get(r['sequence'], [])), 'policy_image': frames.get(r['sequence'], {}).get('views', {}).get('policy')},
                   'issued_targets': {'body_q_rad': dict(zip(body_names, r['body_command_rad'])), 'hand_joint_q_rad': issued_hand,
                                      'hand_target_dataset_order_rad': {s: _maybe(adapters[s], issued_hand) for s in ('left', 'right')}},   # None for an uncommanded hand
                   'language': language}
            f.write(json.dumps(row, allow_nan=False) + '\n'); rows.append(row)
    episode = {'schema': SCHEMA, 'origin': 'SIMULATION (Isaac Sim 5.1, CPU PhysX, RTX render); NOT a real recording', 'robot': 'Unitree G1 EDU 29-DoF twin; hands = PROVISIONAL RH56DFTP donor (right hand active); target hands RH56E2-2R-T1 / 2L-T1 (unverified equivalence)',
               'support': metrics.get('support_constraints'), 'camera_profile': 'policy_head_d435_color_nominal (URDF torso mount, RGB 640x480, nominal 69 deg HFOV)', 'source_run': str(run), 'run_id': receipt.get('run_id'), 'execution_label': metrics.get('media_labels', {}).get('fixture'),
               'command_source': sequence['source'], 'hand_contract_dataset_order': {'order': list(DATASET_HAND_ORDER), 'units': 'donor joint radians, 0 = URDF open, closed = donor limits per axis', 'unresolved': 'angle datum vs any real Inspire hand: UNVERIFIED'},
               'scene': metrics.get('scene'), 'frames': len(rows), 'sample_interval_s': metrics.get('physics_dt'), 'split': split, 'task_evaluation': json.loads(task_eval.read_text()) if task_eval and task_eval.exists() else None,
               'hashes': {n: sha(run / n) for n in ('state.jsonl', 'commands.jsonl', 'metrics.json') if (run / n).exists()} | {'package/' + n: sha(package / n) for n in ('sequence.json', 'controller.json', 'manifest.json')}, 'manifest_sha256': manifest.sha256,
               'note': 'observations are measured simulator states; issued_targets are the commands the controller received — never motor-command ground truth of a real robot'}
    (out / 'episode.json').write_text(json.dumps(episode, indent=2) + '\n')
    return episode, rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', type=Path, required=True); ap.add_argument('--package', type=Path, required=True); ap.add_argument('--manifest', type=Path, required=True); ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--language', default='pick up the cylinder.'); ap.add_argument('--task-eval', type=Path); ap.add_argument('--split', default='unassigned')
    a = ap.parse_args(argv)
    episode, rows = export(a.run, a.package, a.manifest, a.out, language=a.language, task_eval=a.task_eval, split=a.split)
    print(json.dumps({'out': str(a.out), 'frames': episode['frames'], 'schema': SCHEMA, 'split': a.split}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
