#!/usr/bin/env python3
"""CPU preflight for one bounded simulator job: fail configuration errors BEFORE a reservation.

Checks (all host-side, no container, no GPU): probe compiles and its pure modules import; the probe
config validates through the probe's own validator and is JSON-clean; private inputs and files the
config references exist inside declared private roots; fixture hashes pinned in the config match the
fixture on disk; worktrees are clean; the active session authorization is live; the measured-runtime
allowance and the experimental cutoff admit the requested reservation; the requested reservation
covers a runtime estimate from the measured wall time per step of earlier receipts of the same mode.
"""
import argparse
import importlib.util
import json
import math
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from sim_admission import AdmissionError, Clock, active_session, admit, digest, read_ledger, session_accounting  # noqa: E402

STARTUP_WALL_S = 110.   # measured Isaac 5.1 container start to first controlled step (s8 receipts: 95-105 s)
SHUTDOWN_WALL_S = 40.


def load_probe(path):
    spec = importlib.util.spec_from_file_location('preflight_probe', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def json_clean(value, where='config'):
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f'{where}: non-finite number')
    if isinstance(value, dict):
        for k, v in value.items():
            if not isinstance(k, str):
                raise ValueError(f'{where}: non-string key')
            json_clean(v, where + '.' + k)
    elif isinstance(value, (list, tuple)):
        for i, v in enumerate(value):
            json_clean(v, f'{where}[{i}]')
    elif value is not None and not isinstance(value, (str, int, float, bool)):
        raise ValueError(f'{where}: not JSON-serializable ({type(value).__name__})')


def wall_per_step(campaign, mode):
    """Median measured wall seconds per physics step over completed receipts of this probe mode
    (mode read from the recorded container command; wall and steps from the probe metrics)."""
    samples = []
    for run in sorted((campaign / 'evidence').glob('*/run.json')):
        try:
            manifest = json.loads(run.read_text())
            if ('PANTHERA_PROBE_MODE=' + mode) not in manifest.get('command', []):
                continue
            metrics = json.loads((run.parent / 'metrics.json').read_text())
            steps = metrics.get('steps'); wall = metrics.get('wall_seconds')
            if isinstance(steps, int) and steps > 200 and isinstance(wall, (int, float)) and wall > STARTUP_WALL_S:
                samples.append((wall - STARTUP_WALL_S) / steps)
        except (OSError, ValueError, KeyError):
            continue
    if not samples:
        return None, 0
    samples.sort()
    return samples[len(samples) // 2], len(samples)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign', type=Path, required=True)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--probe-mode', required=True)
    parser.add_argument('--probe-config', type=Path)
    parser.add_argument('--private-input', action='append', default=[])
    parser.add_argument('--seconds', type=float, required=True)
    parser.add_argument('--category', default='diagnostic')
    parser.add_argument('--expected-steps', type=int, help='physics steps the job will take if it completes')
    parser.add_argument('--cleanup-seconds', type=float, default=30.)
    args = parser.parse_args(argv)
    campaign = args.campaign.resolve()
    problems, notes = [], {}

    # 1) probe compiles; pure modules import on the host
    try:
        subprocess.check_output([sys.executable, '-m', 'py_compile', str(args.probe)], stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as exc:
        problems.append('probe does not compile: ' + exc.output.decode()[-300:])
    for module in ('inspire_grasp', 'inspire_rack', 'urdf_kinematics'):
        try:
            importlib.import_module(module)
        except Exception as exc:  # noqa: BLE001
            problems.append(f'{module} import failed on host: {exc!r}')
    sys.path.insert(0, str(HERE.parent / 'isaac' / 'twin'))
    for module in ('inspire.contact_ink', 'inspire.whiteboard_scene'):
        try:
            importlib.import_module(module)
        except Exception as exc:  # noqa: BLE001
            problems.append(f'{module} import failed on host: {exc!r}')

    # 2) config validates through the probe's validator and is JSON-clean
    config = None
    if args.probe_config:
        try:
            config = json.loads(args.probe_config.read_text())
            json_clean(config)
            probe = load_probe(args.probe)
            if hasattr(probe, 'validate_config'):
                probe.validate_config(config)
            notes['config_sha256'] = digest(args.probe_config)
        except Exception as exc:  # noqa: BLE001
            problems.append(f'probe config invalid: {exc!r}')

    # 3) private inputs and referenced files
    lock = json.loads((campaign / 'workspace-lock.json').read_text())
    roots = [Path(r).resolve() for r in lock.get('private_input_roots', [])] + [Path(r['worktree']).resolve() for r in lock['repositories'] if r['visibility'] == 'private']
    mounts = {}
    for spec in args.private_input:
        name, _, raw = spec.partition('=')
        path = Path(raw).resolve()
        if not path.is_dir() or not any(path.is_relative_to(root) for root in roots):
            problems.append(f'private input {name} missing or outside declared private roots: {path}')
        mounts[name] = path
    if config and isinstance(config.get('contact_writing'), dict):
        grasp_path = Path(config['contact_writing']['held_marker_grasp_path'])
        fixture = mounts.get('writer-fixture')
        if fixture is None:
            problems.append('contact writing needs --private-input writer-fixture=...')
        else:
            local = fixture / grasp_path.relative_to('/workspace/writer-fixture')
            if not local.is_file():
                problems.append(f'held marker grasp missing in fixture: {local}')
            else:
                try:
                    from inspire_grasp import GraspConfig
                    grasp = GraspConfig.from_dict(json.loads(local.read_text()))
                    notes['grasp_sha256'] = grasp.sha256
                    for key, attr in (('grasp_finger_kp_nm_rad', 'finger_stiffness_nm_rad'), ('grasp_finger_kd_nm_s_rad', 'finger_damping_nm_s_rad')):
                        if abs(float(config['contact_writing'][key]) - getattr(grasp, attr)) > 1e-9:
                            problems.append(f'config {key} differs from grasp file {attr}')
                except Exception as exc:  # noqa: BLE001
                    problems.append(f'held marker grasp invalid: {exc!r}')
            frames = fixture / 'planner_frames.json'
            if not frames.is_file():
                problems.append('fixture lacks planner_frames.json')
            elif config.get('planner_frames_sha256') != digest(frames):
                problems.append('config planner_frames_sha256 does not match the fixture planner_frames.json')
            for required in ('profile.json', 'scene_tool_board.yaml', 'poses/write_home.yaml', 'model_provenance.json'):
                if not (fixture / required).is_file():
                    problems.append(f'fixture missing {required}')

    # 4) clean worktrees
    for repo in lock['repositories']:
        dirty = subprocess.check_output(['git', '-C', repo['worktree'], 'status', '--porcelain'], text=True).strip()
        if dirty:
            problems.append(f"{repo['repository']} worktree dirty:\n{dirty[:300]}")

    # 5) live session, allowance and cutoff
    clock = Clock.now()
    try:
        auth_path, auth = active_session(campaign, clock)
        events = read_ledger(auth_path.parent / 'ledger.jsonl')
        account = session_accounting(auth, events)
        notes['accounting'] = {k: account[k] for k in ('charged_completed_seconds', 'active_reservation_seconds', 'reserved_history_seconds', 'unresolved_runs', 'unreconciled_finished_runs')}
        admit(auth, events, category=args.category, name='preflight', justification='preflight dry run', seconds=args.seconds,
              cleanup_seconds=args.cleanup_seconds, training_prerequisites=None, clock=clock, authorization_sha256=digest(auth_path))
        notes['experimental_seconds_remaining'] = auth['experimental_deadline_unix'] - clock.utc
    except (AdmissionError, OSError, KeyError, ValueError) as exc:
        problems.append(f'admission would be refused: {exc}')

    # 6) runtime estimate from measured receipts
    per_step, n = wall_per_step(campaign, args.probe_mode)
    notes['measured_wall_per_step_s'] = per_step; notes['receipts_used'] = n
    if args.expected_steps and per_step:
        estimate = STARTUP_WALL_S + args.expected_steps * per_step + SHUTDOWN_WALL_S
        notes['estimated_launcher_seconds'] = estimate
        if estimate > args.seconds - args.cleanup_seconds:
            problems.append(f'requested {args.seconds:.0f} s cannot cover the estimated {estimate:.0f} s (steps x measured wall/step + startup/shutdown)')
    elif args.expected_steps:
        notes['estimate'] = 'no completed receipt of this mode yet; request must include startup, steps at ~0.04-0.07 s/step and shutdown'

    report = {'status': 'PASS' if not problems else 'FAIL', 'problems': problems, 'notes': notes,
              'probe': str(args.probe), 'probe_mode': args.probe_mode, 'seconds': args.seconds, 'category': args.category}
    print(json.dumps(report, indent=2))
    return 0 if not problems else 1


if __name__ == '__main__':
    raise SystemExit(main())
