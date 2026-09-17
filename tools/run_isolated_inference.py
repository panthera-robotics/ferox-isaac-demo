#!/usr/bin/env python3
"""Admitted, network-isolated OFFLINE inference job on the campaign GPU (narrow integration, sprint H).

Same admission boundary as the simulator launcher: ADMITTED -> docker run (no network, private IPC,
GPU only, read-only mounts, non-root) -> FINISHED -> RECONCILED (measured wall charged, reservation
released once). Category ``inference`` counts as a named batch (<= maximum_named_batch_seconds).
The job script and every mounted input directory are hashed into run.json before launch; the script
must write metrics.json and probe.json into /evidence.
"""
import argparse
import json
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from sim_admission import AdmissionError, Clock, active_session, admit, atomic_json, digest, ledger_event, read_ledger, reconcile, remaining_execution, tree_hashes  # noqa: E402
from sim_watchdog import finish_event  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--campaign', type=Path, required=True); ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--script', type=Path, required=True, help='python script run inside the container as /job/script.py')
    ap.add_argument('--mount', action='append', default=[], metavar='NAME=PATH', help='read-only mount at /mnt/NAME (hashed unless --large NAME)')
    ap.add_argument('--large', action='append', default=[], help='mount names whose trees are recorded by size only (weights/envs)')
    ap.add_argument('--image', required=True); ap.add_argument('--python', default='/mnt/env/bin/python')
    ap.add_argument('--seconds', type=int, default=600); ap.add_argument('--cleanup-seconds', type=int, default=60)
    ap.add_argument('--job-name', required=True); ap.add_argument('--justification', required=True)
    ap.add_argument('--gpus', default='all'); ap.add_argument('--memory', default='40g'); ap.add_argument('--shm', default='8g')
    a = ap.parse_args(argv)
    campaign = a.campaign.resolve(); clock = Clock.now()
    auth_path, auth = active_session(campaign, clock)
    auth_hash = digest(auth_path); session_root = auth_path.parent; ledger_path = session_root / 'ledger.jsonl'
    if a.output.exists():
        ap.error('output directory exists; runs are never overwritten')
    name = 'panthera_inf_' + uuid.uuid4().hex[:16]
    job = admit(auth, read_ledger(ledger_path), category='inference', name=a.job_name, justification=a.justification, seconds=a.seconds,
                cleanup_seconds=a.cleanup_seconds, training_prerequisites=None, clock=clock, authorization_sha256=auth_hash)
    mounts = []
    for spec in a.mount:
        label, _, raw = spec.partition('=')
        path = Path(raw).resolve()
        if not label.isidentifier() or not path.is_dir():
            ap.error('mount must be NAME=DIR')
        mounts.append((label, path))
    control = session_root / 'jobs' / name; control.mkdir(parents=True, mode=0o700)
    output = a.output.resolve(); output.mkdir(mode=0o777); output.chmod(0o777)
    job.update(run_id=name, output=str(output), ledger_path=str(ledger_path), kind='offline_inference')
    ledger_event(ledger_path, dict(job, event='ADMITTED')); atomic_json(control / 'job.json', job)
    manifest = {'schema_version': 2, 'kind': 'bounded_offline_inference_job', 'run_id': name, 'session_id': job['session_id'], 'admission': job,
                'start_unix': clock.utc, 'start_monotonic': clock.monotonic, 'hardware_authorized': False, 'status': 'PREPARING', 'maximum_seconds': a.seconds,
                'image': a.image, 'script_sha256': digest(a.script), 'script': str(a.script.resolve()), 'input_sha256': {}, 'input_bytes': {}, 'network': 'none', 'ipc': 'private'}
    for label, path in mounts:
        if label in a.large:
            manifest['input_bytes'][label] = sum(p.stat().st_size for p in path.rglob('*') if p.is_file())
        else:
            manifest['input_sha256'].update({label + '/' + k: v for k, v in tree_hashes(path).items()})
    save = lambda: atomic_json(output / 'run.json', manifest)   # noqa: E731
    save()
    result = 1; cname = name
    try:
        remaining = max(1, int(remaining_execution(job, Clock.now()) - a.cleanup_seconds))
        cmd = ['docker', 'run', '--rm', '--name', cname, '--network', 'none', '--ipc', 'private', '--gpus', a.gpus, '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges',
               '--memory', a.memory, '--shm-size', a.shm, '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'HOME=/tmp', '-e', 'HF_HUB_OFFLINE=1', '-e', 'TRANSFORMERS_OFFLINE=1',
               '-e', 'PANTHERA_INFERENCE_AUTHORIZED=1', '-e', 'PANTHERA_RUN_ID=' + name,
               '--mount', f'type=bind,source={a.script.resolve()},target=/job/script.py,readonly', '--mount', f'type=bind,source={output},target=/evidence']
        for label, path in mounts:
            cmd += ['--mount', f'type=bind,source={path},target=/mnt/{label},readonly']
        cmd += ['--entrypoint', '/usr/bin/timeout', a.image, '--signal=TERM', '--kill-after=20', str(remaining), a.python, '/job/script.py']
        manifest['command'] = cmd; manifest['status'] = 'RUNNING'; save()
        with (output / 'console.log').open('w') as log:
            proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, timeout=remaining + 40)
        result = proc.returncode
        probe = json.loads((output / 'probe.json').read_text()) if (output / 'probe.json').exists() else {}
        manifest['probe'] = probe
        if probe.get('status') != 'PASS':
            result = result or 1
    except Exception as exc:  # noqa: BLE001
        manifest['error'] = repr(exc); result = 1
    finally:
        subprocess.run(['docker', 'rm', '--force', cname], capture_output=True, timeout=30)
        end = Clock.now()
        manifest.update(status='PASS' if result == 0 else 'FAIL', exit_code=result, end_unix=end.utc, end_monotonic=end.monotonic); save()
        finish_event(job, manifest['status'], end, cleanup_verified=True)
        try:
            settled = reconcile(auth, read_ledger(ledger_path), run_id=name, clock=Clock.now(), authorization_sha256=auth_hash)
            ledger_event(ledger_path, settled); manifest['reconciliation'] = settled; save()
        except AdmissionError as exc:
            manifest['reconciliation_error'] = str(exc); save()
    print(json.dumps({'status': manifest['status'], 'run_id': name, 'output': str(output), 'measured_launcher_seconds': manifest.get('reconciliation', {}).get('measured_launcher_seconds')}))
    return result


if __name__ == '__main__':
    raise SystemExit(main())
