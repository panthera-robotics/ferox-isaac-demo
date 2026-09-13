#!/usr/bin/env python3
"""Independent bounded cleanup of one explicitly identified simulation container."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time

from sim_admission import Clock, atomic_json, ledger_event, remaining_execution


def process_identity(pid):
    try:
        # Parenthesized command names may contain spaces or ')'. Field 22 follows state.
        fields = Path(f'/proc/{int(pid)}/stat').read_text().rsplit(')', 1)[1].split()
        return {'pid': int(pid), 'start_ticks': fields[19]}
    except (OSError, ValueError, IndexError):
        return None


def matches_container(inspected, job):
    return (bool(re.fullmatch(r'[a-f0-9]{64}', job.get('container_id', '')))
            and inspected.get('Id') == job['container_id']
            and inspected.get('Name') == '/' + job['run_id']
            and inspected.get('Config', {}).get('Labels', {}).get('panthera.run_token') == job['run_token']
            and inspected.get('Config', {}).get('Labels', {}).get('panthera.session_id') == job['session_id'])


def cleanup_container(job, runner=subprocess.run):
    """Never stop by name, prefix, image, or PID; verify the immutable Docker ID and labels."""
    errors = []
    cid = job.get('container_id', '')
    if not re.fullmatch(r'[a-f0-9]{64}', cid):
        return ['Missing or invalid container ID; no cleanup attempted']
    try:
        inspected = runner(['docker', 'inspect', cid], capture_output=True, text=True, timeout=5)
        if inspected.returncode:
            # Only a clear absent-container response is successful idempotent cleanup.
            if 'No such object' in inspected.stderr or 'No such container' in inspected.stderr:
                return []
            return ['Container inspection failed: ' + inspected.stderr.strip()]
        obj = json.loads(inspected.stdout)[0]
        if not matches_container(obj, job):
            return ['Container identity mismatch; no stop/removal attempted']
        for argv in [['docker', 'stop', '--timeout', '3', cid], ['docker', 'rm', '--force', cid]]:
            try:
                result = runner(argv, capture_output=True, text=True, timeout=8)
                if result.returncode and 'No such container' not in result.stderr:
                    errors.append(' '.join(argv[:2]) + ': ' + result.stderr.strip())
            except Exception as exc:
                # A failed graceful stop must not prevent bounded forced removal.
                errors.append(f'{argv[1]}: {type(exc).__name__}: {exc}')
    except Exception as exc:
        errors.append(f'{type(exc).__name__}: {exc}')
    return errors


def finish_event(job, status, clock=None, cleanup_verified=True):
    clock = clock or Clock.now()
    event = {k: job[k] for k in ['session_id', 'authorization_sha256', 'run_id']}
    event.update(event='FINISHED', status=status, utc=clock.utc, monotonic=clock.monotonic,
                 boot_id=clock.boot_id, cleanup_verified=cleanup_verified)
    ledger_event(job['ledger_path'], event)


def run_watchdog(job_path, inherited_lock_fd):
    # Keep the inherited flock descriptor open after the launcher dies. Do not unlink it.
    os.fstat(inherited_lock_fd)
    job_path = Path(job_path)
    job = json.loads(job_path.read_text())
    control = job_path.parent
    atomic_json(control / 'watchdog-ready.json', {'pid': os.getpid(), 'job': job['run_id']})
    last = Clock.now()
    reason = None
    while True:
        if (control / 'complete.json').exists():
            return 0
        now = Clock.now()
        if now.boot_id != job['boot_id']:
            reason = 'boot_identity_changed'
        elif now.utc + .001 < last.utc or now.monotonic + .001 < last.monotonic:
            reason = 'clock_rollback'
        elif process_identity(job['parent']['pid']) != job['parent']:
            reason = 'launcher_process_disappeared_or_reused'
        elif remaining_execution(job, now) <= 0:
            reason = 'execution_deadline_reached'
        if reason:
            break
        last = now
        time.sleep(min(.5, remaining_execution(job, now)))
    errors = cleanup_container(job)
    clock = Clock.now()
    receipt = {'status': 'WATCHDOG_STOPPED', 'reason': reason, 'cleanup_errors': errors,
               'container_id': job['container_id'], 'utc': clock.utc, 'monotonic': clock.monotonic,
               'boot_id': clock.boot_id}
    atomic_json(control / 'watchdog.json', receipt)
    # Recover the durable job receipt even if no launcher survives to run finally.
    output = Path(job['output'])
    manifest_path = output / 'run.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    manifest.update(status='FAIL', watchdog=receipt, end_unix=clock.utc,
                    end_monotonic=clock.monotonic, cleanup_errors=errors)
    atomic_json(manifest_path, manifest)
    finish_event(job, 'WATCHDOG_STOPPED', clock, cleanup_verified=not errors)
    return 1 if errors else 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--job', type=Path, required=True)
    parser.add_argument('--lock-fd', type=int, required=True)
    args = parser.parse_args()
    raise SystemExit(run_watchdog(args.job, args.lock_fd))
