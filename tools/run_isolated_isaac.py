#!/usr/bin/env python3
"""Run one bounded Isaac probe with no network, hardware mounts, or host IPC.

The image and all mount paths are recorded before execution. This launcher is
for simulation diagnostics; it deliberately cannot launch a hardware profile.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import uuid
import sys
import re

from sim_admission import (AdmissionError, Clock, admit, atomic_json, digest, ledger_event,
                           read_ledger, reconcile, remaining_execution, snapshot_public, snapshot_tree, tree_hashes, private_input)
from sim_watchdog import cleanup_container, finish_event, process_identity


def command(*args, timeout=30):
    return subprocess.check_output(args, text=True, timeout=timeout).strip()


def validate_receipt(output):
    receipt = json.loads((output / "probe.json").read_text())
    if not isinstance(receipt, dict) or receipt.get("status") not in {"PASS", "FAIL"}:
        raise ValueError("Invalid probe receipt")
    artifacts = receipt.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts or len(artifacts) != len(set(artifacts)):
        raise ValueError("Probe receipt must identify its evidence artifacts")
    hashes = {}
    for name in artifacts:
        if not isinstance(name, str) or Path(name).is_absolute():
            raise ValueError("Evidence paths must be relative")
        path = (output / name).resolve()
        if not path.is_relative_to(output.resolve()) or not path.is_file() or not path.stat().st_size:
            raise ValueError(f"Missing, empty, or escaped evidence artifact: {name}")
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    receipt["artifact_sha256"] = hashes
    return receipt


def input_hashes(roots):
    hashes = {}
    for label, root in roots:
        for path in sorted(root.rglob("*")):
            if path.is_file() and not {'.git', '__pycache__', '.pytest_cache'}.intersection(path.parts):
                hashes[label + "/" + str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return hashes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('script', help='Python probe relative to this public repository')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=int, default=300, help='Total job allocation INCLUDING initialization and cleanup')
    parser.add_argument('--cleanup-seconds', type=int, default=30)
    parser.add_argument('--category', choices=['diagnostic', 'asset-build', 'initialization', 'integration', 'evaluation', 'training'], default='diagnostic')
    parser.add_argument('--job-name', default='')
    parser.add_argument('--justification', default='')
    parser.add_argument('--training-prerequisites', type=Path, help='Private JSON: working_environment/controller_gap/evaluation_criterion/checkpoint_plan')
    parser.add_argument('--cache', type=Path, default=Path('/data/isaac_cache'))
    parser.add_argument('--image', default='nvcr.io/nvidia/isaac-sim:5.1.0')
    parser.add_argument('--policy', type=Path)
    parser.add_argument('--source-assets', type=Path)
    parser.add_argument('--private-input', action='append', default=[], metavar='NAME=PATH', help='Explicit private snapshot mounted read-only at /workspace/NAME')
    parser.add_argument('--workspace-lock', type=Path, required=True)
    parser.add_argument('--probe-mode', default='default', help='Bounded mode identifier, validated semantically by the selected probe')
    parser.add_argument('--probe-config', type=Path, help='Private JSON copied and mounted read-only as /probe-config.json')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}', args.probe_mode):
        parser.error('Probe mode must be a simple bounded identifier')
    repo = Path(__file__).resolve().parents[1]
    script = (repo / args.script).resolve()
    if not script.is_relative_to(repo) or not script.is_file() or script.suffix != '.py':
        parser.error('Probe must be an existing Python file inside this repository')
    if not script.is_relative_to(repo / 'tools') and not script.is_relative_to(repo / 'isaac'):
        parser.error('Probe must be in the captured tools or isaac tree')
    # Inherited by the independent watchdog; never unlink this global lock.
    lock = open('/tmp/panthera-isolated-isaac.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    lock_path = args.workspace_lock.resolve()
    workspace = json.loads(lock_path.read_text())
    campaign = lock_path.parent
    if campaign.stat().st_mode & 0o077:
        parser.error('Workspace parent must be private (0700)')
    output = args.output.resolve()
    if output.parent != (campaign / 'evidence').resolve() or output.exists():
        parser.error('Output must be a new child of the private evidence directory')
    repositories = []
    for pinned in workspace['repositories']:
        path = Path(pinned['worktree'])
        repositories.append({'repository': pinned['repository'], 'visibility': pinned['visibility'],
                             'head': command('git', '-C', str(path), 'rev-parse', 'HEAD'),
                             'branch': command('git', '-C', str(path), 'branch', '--show-current'),
                             'dirty': command('git', '-C', str(path), 'status', '--porcelain')})
    if len(repositories) != 4 or workspace.get('hardware_authorized') is not False:
        parser.error('Expected four-repository simulation-only workspace lock')
    identity_path = (campaign / workspace['identity_contract']).resolve()
    if not identity_path.is_relative_to(campaign):
        parser.error('Identity contract escapes private campaign')
    identity = json.loads(identity_path.read_text())
    if identity.get('hardware_authorized') is not False:
        parser.error('Identity contract must deny hardware authority')
    name = 'panthera_sim_' + uuid.uuid4().hex[:16]
    now = Clock.now()
    authorization_ref = workspace.get('session_authorization')
    old_elapsed = None
    if authorization_ref:
        relative = authorization_ref['path'] if isinstance(authorization_ref, dict) else authorization_ref
        authorization_path = (campaign / relative).resolve()
        if not authorization_path.is_relative_to(campaign / 'sessions'):
            parser.error('Session authorization must be inside private sessions')
        auth_hash = digest(authorization_path)
        if isinstance(authorization_ref, dict) and authorization_ref.get('sha256') != auth_hash:
            parser.error('Session authorization hash mismatch')
        authorization = json.loads(authorization_path.read_text())
        session_root = authorization_path.parent
        ledger_path = session_root / 'ledger.jsonl'
        prerequisites = json.loads(args.training_prerequisites.read_text()) if args.training_prerequisites else None
        job = admit(authorization, read_ledger(ledger_path), category=args.category,
                    name=args.job_name, justification=args.justification, seconds=args.seconds,
                    cleanup_seconds=args.cleanup_seconds, training_prerequisites=prerequisites,
                    clock=now, authorization_sha256=auth_hash)
    else:
        # Original allowance stays immutable. A new session must be explicitly referenced.
        if args.category != 'diagnostic' or args.seconds > 300 or not 30 <= args.cleanup_seconds <= 120 or args.cleanup_seconds >= args.seconds - 1:
            parser.error('Legacy scope admits only bounded diagnostics with cleanup reserve')
        budget = workspace['gpu_budget']
        old_elapsed = float(budget.get('unmanifested_diagnostic_reserve_seconds', 0.))
        for run in (campaign / 'evidence').glob('*/run.json'):
            prior = json.loads(run.read_text())
            if 'end_unix' not in prior:
                parser.error(f'Prior run lacks an end receipt: {run}')
            old_elapsed += max(0., prior['end_unix'] - prior['start_unix'])
        if args.seconds > budget['maximum_single_run_seconds'] or old_elapsed + args.seconds > budget['maximum_campaign_gpu_seconds']:
            parser.error('Legacy diagnostic allowance exceeded; no session reset is implied')
        session_root = campaign / 'legacy-launcher-custody'
        session_root.mkdir(mode=0o700, exist_ok=True)
        ledger_path = session_root / 'ledger.jsonl'
        job = {'session_id': 'legacy', 'authorization_sha256': digest(lock_path), 'category': 'diagnostic',
               'job_name': args.job_name, 'justification': args.justification, 'allocation_seconds': args.seconds,
               'cleanup_seconds': args.cleanup_seconds, 'boot_id': now.boot_id, 'utc': now.utc, 'monotonic': now.monotonic,
               'hard_deadline_utc': now.utc + args.seconds, 'hard_deadline_monotonic': now.monotonic + args.seconds,
               'stop_deadline_utc': now.utc + args.seconds - args.cleanup_seconds,
               'stop_deadline_monotonic': now.monotonic + args.seconds - args.cleanup_seconds,
               'experimental_deadline_utc': now.utc + args.seconds,
               'experimental_deadline_monotonic': now.monotonic + args.seconds}
    control = session_root / 'jobs' / name
    control.mkdir(parents=True, mode=0o700)
    output.mkdir(mode=0o777)
    output.chmod(0o777)  # Container UID can write results; controls/snapshots are elsewhere.
    job.update(run_id=name, run_token=uuid.uuid4().hex, output=str(output), ledger_path=str(ledger_path),
               parent=process_identity(os.getpid()))
    ledger_event(ledger_path, dict(job, event='ADMITTED'))
    atomic_json(control / 'job.json', job)
    manifest = {'schema_version': 2, 'kind': 'bounded_simulation_job', 'run_id': name,
                'session_id': job['session_id'], 'admission': job, 'start_unix': now.utc,
                'start_monotonic': now.monotonic, 'hardware_authorized': False, 'status': 'PREPARING',
                'maximum_seconds': args.seconds, 'workspace_repositories': repositories,
                'workspace_lock_sha256': digest(lock_path), 'identity_contract_sha256': digest(identity_path),
                'identity_contract': identity, 'diagnostic_seconds_consumed_before_run': old_elapsed}
    def save():
        atomic_json(output / 'run.json', manifest)
    save()
    created = False
    watchdog = None
    result = 1
    try:
        source = control / 'public-source'
        diff_before = command('git', '-C', str(repo), 'diff', '--binary', 'HEAD')
        sha_before = command('git', '-C', str(repo), 'rev-parse', 'HEAD')
        manifest['input_sha256'] = {'public/' + k: v for k, v in snapshot_public(repo, source).items()}
        if (command('git', '-C', str(repo), 'diff', '--binary', 'HEAD') != diff_before
                or command('git', '-C', str(repo), 'rev-parse', 'HEAD') != sha_before):
            raise AdmissionError('Repository changed during source capture')
        (output / 'uncommitted.patch').write_text(diff_before + '\n')
        shutil.copyfile(source / script.relative_to(repo), output / 'executed_probe.py')
        shutil.copyfile(source / 'tools/run_isolated_isaac.py', output / 'executed_launcher.py')
        manifest['source_snapshot_sha256'] = {n: digest(output / n) for n in
            ['executed_probe.py', 'executed_launcher.py', 'uncommitted.patch']}
        manifest.update(repo_sha=sha_before, branch=command('git', '-C', str(repo), 'branch', '--show-current'),
                        dirty_status=command('git', '-C', str(repo), 'status', '--porcelain'),
                        script_sha256=digest(source / script.relative_to(repo)), source_snapshot=str(source))
        mounts = [(source / 'isaac', '/workspace/ferox_isaac', 'ro'),
                  (source / 'tools', '/workspace/ferox_tools', 'ro'),
                  (source, '/workspace/sim-source', 'ro'), (output, '/evidence', 'rw')]
        immutable = [('public', source)]
        for path, label, target in [(args.policy, 'policy', '/policy'), (args.source_assets, 'source_assets', '/source-assets')]:
            if path is not None:
                destination = control / label
                hashes = snapshot_tree(path.resolve(), destination)
                manifest['input_sha256'].update({label + '/' + k: v for k, v in hashes.items()})
                mounts.append((destination, target, 'ro')); immutable.append((label, destination))
        private_names = set()
        for spec in args.private_input:
            label, path = private_input(spec, workspace)
            if label in private_names or label in {'sim-source', 'ferox_isaac', 'ferox_tools'}:
                raise AdmissionError('Duplicate or reserved private mount name')
            private_names.add(label)
            destination = control / 'private-inputs' / label
            hashes = snapshot_tree(path, destination)
            manifest['input_sha256'].update({'private/' + label + '/' + k: v for k, v in hashes.items()})
            manifest.setdefault('private_inputs', []).append({'name': label, 'source': str(path), 'snapshot': str(destination), 'visibility': 'private'})
            mounts.append((destination, '/workspace/' + label, 'ro')); immutable.append(('private/' + label, destination))
        if args.probe_config:
            config_path = args.probe_config.resolve()
            if not config_path.is_relative_to(campaign):
                raise AdmissionError('Probe config must be in the private campaign')
            raw = config_path.read_bytes()
            json.loads(raw, parse_constant=lambda value: (_ for _ in ()).throw(AdmissionError('Nonfinite probe config')))
            destination = control / 'probe-config.json'
            destination.write_bytes(raw); destination.chmod(0o444)
            manifest['probe_config_sha256'] = digest(destination)
            mounts.append((destination, '/probe-config.json', 'ro'))
        for key, target in {'kit': 'kit/cache', 'ov': '.cache/ov', 'pip': '.cache/pip',
                            'gl': '.cache/nvidia/GLCache', 'compute': '.nv/ComputeCache', 'warp': '.cache/warp'}.items():
            cache = args.cache.resolve() / key
            if not cache.is_dir():
                raise AdmissionError(f'Missing existing cache: {cache}')
            mounts.append((cache, '/isaac-sim/' + target, 'rw'))
        remaining = remaining_execution(job, Clock.now())
        if remaining < 5:
            raise AdmissionError('Preparation consumed the available execution allocation')
        image = json.loads(command('docker', 'image', 'inspect', args.image, timeout=min(10, remaining)))[0]
        create = ['docker', 'create', '--name', name, '--label', 'panthera.run_token=' + job['run_token'],
                  '--label', 'panthera.session_id=' + job['session_id'], '--gpus', 'all', '--user', '1234:1234',
                  '--network', 'none', '--ipc', 'private', '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges',
                  '--shm-size', '2g', '-e', 'ACCEPT_EULA=Y', '-e', 'PRIVACY_CONSENT=Y', '-e', 'PYTHONDONTWRITEBYTECODE=1',
                  '-e', 'ROS_DOMAIN_ID=73', '-e', 'ROS_LOCALHOST_ONLY=1', '-e', 'ROS_DISTRO=humble',
                  '-e', 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp', '-e', 'LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib',
                  '-e', 'PANTHERA_SIM_AUTHORIZED=1', '-e', 'PANTHERA_SIM_RUN_ID=' + name, '-e', 'PANTHERA_PROBE_MODE=' + args.probe_mode]
        if args.probe_config:
            create += ['-e', 'PANTHERA_PROBE_CONFIG=/probe-config.json']
        for path, target, mode in mounts:
            if ',' in str(path):
                raise AdmissionError('Comma in mount path is unsupported')
            create += ['--mount', f'type=bind,source={path},target={target}' + (',readonly' if mode == 'ro' else '')]
        create += ['--entrypoint', '/usr/bin/timeout', image['Id'], '--signal=TERM', '--kill-after=5',
                   str(max(1, int(remaining_execution(job, Clock.now())))), '/isaac-sim/python.sh',
                   '/workspace/sim-source/' + str(script.relative_to(repo))]
        manifest.update(command=create, image_id=image['Id'], image_digests=image.get('RepoDigests', []),
                        architecture=image['Architecture'], status='STARTING',
                        gpu=command('nvidia-smi', '--query-gpu=name,uuid,driver_version,memory.total', '--format=csv,noheader', timeout=10))
        save()
        cid = command(*create, timeout=max(1, min(15, remaining_execution(job, Clock.now()))))
        job['container_id'] = cid; created = True
        atomic_json(control / 'job.json', job)
        inspected = json.loads(command('docker', 'inspect', cid, timeout=5))[0]
        host = inspected['HostConfig']
        if (host['NetworkMode'] != 'none' or host['IpcMode'] != 'private' or host['Privileged']
                or host['Devices'] or host['PortBindings'] or host['CapAdd'] or host['CapDrop'] != ['ALL']
                or 'no-new-privileges' not in host['SecurityOpt'] or inspected['Config']['User'] != '1234:1234'):
            raise AdmissionError('Container isolation readback failed')
        manifest['isolation'] = {key: host[key] for key in ['NetworkMode', 'IpcMode', 'Privileged', 'Devices', 'PortBindings', 'CapAdd', 'CapDrop', 'SecurityOpt']}
        manifest['container_id'] = cid
        save()
        with (control / 'watchdog.log').open('w') as log:
            watchdog = subprocess.Popen([sys.executable, str(source / 'tools/sim_watchdog.py'),
                                         '--job', str(control / 'job.json'), '--lock-fd', str(lock.fileno())],
                                        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                        start_new_session=True, pass_fds=(lock.fileno(),))
        readiness_deadline = time.monotonic() + min(5, remaining_execution(job, Clock.now()))
        while not (control / 'watchdog-ready.json').exists():
            if watchdog.poll() is not None or time.monotonic() >= readiness_deadline:
                raise AdmissionError('Independent watchdog failed to become ready')
            time.sleep(.05)
        remaining = remaining_execution(job, Clock.now())
        if remaining <= 0:
            raise AdmissionError('Execution allocation expired before start')
        with (output / 'console.log').open('w') as log:
            result = subprocess.run(['docker', 'start', '--attach', cid], stdout=log, stderr=subprocess.STDOUT,
                                    timeout=remaining).returncode
        manifest['container_exit'] = json.loads(command('docker', 'inspect', cid, timeout=5))[0]['State']['ExitCode']
        result = result or manifest['container_exit']
        if (output / 'probe.json').is_file():
            manifest['probe'] = validate_receipt(output)
            if manifest['probe']['status'] != 'PASS':
                result = result or 1
        else:
            manifest['probe'] = {'status': 'MISSING', 'reason': 'No completed probe receipt'}
            result = result or 1
        # Changes to LIVE worktrees are expected. Only captured input bytes are evidence.
        after = {label + '/' + k: v for label, root in immutable for k, v in tree_hashes(root).items()}
        if after != manifest['input_sha256']:
            manifest['snapshot_integrity_failure'] = True; result = result or 1
        if args.probe_config and digest(control / 'probe-config.json') != manifest['probe_config_sha256']:
            manifest['probe_config_integrity_failure'] = True; result = result or 1
    except BaseException as exc:
        manifest['error'] = f'{type(exc).__name__}: {exc}'
        result = result or 1
    finally:
        errors = cleanup_container(job) if created else []
        if errors:
            manifest['cleanup_errors'] = errors; result = result or 1
        if (control / 'watchdog.json').exists():
            manifest['watchdog'] = json.loads((control / 'watchdog.json').read_text()); result = result or 1
        end = Clock.now()
        if end.boot_id != job['boot_id'] or end.utc > job['hard_deadline_utc'] or end.monotonic > job['hard_deadline_monotonic']:
            manifest['allocation_overrun'] = True; result = result or 1
        manifest.update(status='PASS' if result == 0 else 'FAIL', exit_code=result,
                        end_unix=end.utc, end_monotonic=end.monotonic)
        save()
        atomic_json(control / 'complete.json', {'status': manifest['status'], 'utc': end.utc, 'monotonic': end.monotonic})
        finish_event(job, manifest['status'], end, cleanup_verified=not errors)
        if authorization_ref and not errors and authorization.get('limits', {}).get('maximum_session_measured_launcher_seconds') is not None:
            # Measured-runtime allowance: charge the launcher's own ADMITTED->FINISHED clocks exactly once and
            # release this job's reservation; the ADMITTED reservation stays in the ledger history.
            try:
                settled = reconcile(authorization, read_ledger(ledger_path), run_id=name, clock=Clock.now(), authorization_sha256=auth_hash)
                ledger_event(ledger_path, settled)
                manifest['reconciliation'] = settled; save()
            except AdmissionError as exc:
                manifest['reconciliation_error'] = str(exc); save()
        if watchdog:
            try:
                watchdog.wait(timeout=3)
            except subprocess.TimeoutExpired:
                # Independent cleanup remains bounded; do not kill or discard its lock.
                manifest['watchdog_finish_pending'] = True; save()
    print(f"{manifest['status']}: {output}")
    return result


if __name__ == '__main__':
    raise SystemExit(main())
