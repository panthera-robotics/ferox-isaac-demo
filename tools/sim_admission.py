"""Simulation session admission and immutable input custody; standard library only."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import stat
import subprocess
import time


class AdmissionError(ValueError):
    pass


@dataclass(frozen=True)
class Clock:
    utc: float
    monotonic: float
    boot_id: str

    @classmethod
    def now(cls):
        return cls(time.time(), time.monotonic(), Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + f'.{os.getpid()}.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def read_ledger(path):
    path = Path(path)
    if not path.exists():
        return []
    with path.open() as stream:
        fcntl.flock(stream, fcntl.LOCK_SH)
        return [json.loads(line) for line in stream if line.strip()]


def ledger_event(path, event):
    encoded = json.dumps(event, allow_nan=False, separators=(',', ':')) + '\n'
    with Path(path).open('a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        stream.write(encoded)
        stream.flush()
        os.fsync(stream.fileno())


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise AdmissionError(f'Invalid {name}')
    return float(value)


def validate_authorization(auth):
    if (auth.get('kind') != 'explicit_user_authorized_overnight_simulation_session'
            or auth.get('hardware_authorized') is not False
            or not isinstance(auth.get('session_id'), str) or not auth['session_id']
            or not isinstance(auth.get('boot_id'), str) or not auth['boot_id']):
        raise AdmissionError('Expected explicit simulation-only session authorization')
    start = _finite(auth.get('start_unix'), 'start')
    _finite(auth.get('start_monotonic'), 'monotonic start')
    experimental = _finite(auth.get('experimental_deadline_unix'), 'experimental deadline')
    final = _finite(auth.get('final_deadline_unix'), 'final deadline')
    if not (0 < experimental - start <= 27000 and 1800 <= final - experimental
            and final - start <= 28800):
        raise AdmissionError('Session window exceeds the authorized envelope')
    for label, numeric in [('start_utc', start), ('experimental_deadline_utc', experimental), ('final_deadline_utc', final)]:
        value = datetime.fromisoformat(auth[label].replace('Z', '+00:00'))
        if value.tzinfo is None or abs(value.timestamp() - numeric) > .001:
            raise AdmissionError('UTC label disagrees with numeric deadline')
    limits = auth['limits']
    caps = {'default_diagnostic_seconds': 300, 'maximum_named_batch_seconds': 1800,
            'maximum_training_job_seconds': 3600, 'maximum_total_training_allocation_seconds': 7200,
            'concurrent_gpu_jobs': 1}
    for key, cap in caps.items():
        value = _finite(limits.get(key), key)
        zero_allowed = key.startswith('maximum_training') or key == 'maximum_total_training_allocation_seconds'
        if not (0 <= value <= cap if zero_allowed else 0 < value <= cap):
            raise AdmissionError('Session limits exceed authorization')
    measured_cap = limits.get('maximum_session_measured_launcher_seconds')
    reserved_cap = limits.get('maximum_session_allocated_seconds')
    if measured_cap is not None and not 0 < _finite(measured_cap, 'maximum_session_measured_launcher_seconds') <= 27000:
        raise AdmissionError('Measured launcher allowance exceeds authorization')
    if reserved_cap is not None and not 0 < _finite(reserved_cap, 'maximum_session_allocated_seconds') <= 27000:
        raise AdmissionError('Reserved launcher allowance exceeds authorization')
    if measured_cap is None and reserved_cap is None and auth.get('schema_version', 1) >= 2:
        raise AdmissionError('Session must declare a launcher allowance')


def session_accounting(auth, events):
    """Measured-runtime accounting of one session: charged completed runtime (RECONCILED events, one
    per run), active reservations (ADMITTED without RECONCILED, including finished-but-unreconciled and
    unresolved jobs) and the audit history of every original reservation."""
    own = [e for e in events if e.get('session_id') == auth['session_id']]
    admitted = {e['run_id']: e for e in own if e['event'] == 'ADMITTED'}
    finished = {e['run_id']: e for e in own if e['event'] == 'FINISHED'}
    reconciled = {}
    for e in own:
        if e['event'] == 'RECONCILED':
            if e['run_id'] in reconciled:
                raise AdmissionError(f'Duplicate reconciliation in ledger: {e["run_id"]}')
            if e['run_id'] not in admitted:
                raise AdmissionError(f'Reconciliation without admission: {e["run_id"]}')
            reconciled[e['run_id']] = e
    charged = sum(_finite(e['charged_seconds'], 'charged seconds') for e in reconciled.values())
    active = {r: _finite(e['allocation_seconds'], 'reservation') for r, e in admitted.items() if r not in reconciled}
    return {'charged_completed_seconds': charged, 'active_reservation_seconds': sum(active.values()),
            'active_reservations': active, 'reserved_history_seconds': sum(_finite(e['allocation_seconds'], 'reservation') for e in admitted.values()),
            'measured_launcher_seconds': sum(_finite(e['measured_launcher_seconds'], 'measured seconds') for e in reconciled.values()),
            'admitted_runs': len(admitted), 'finished_runs': len(finished), 'reconciled_runs': len(reconciled),
            'unreconciled_finished_runs': sorted(r for r in finished if r not in reconciled),
            'unresolved_runs': sorted(r for r in admitted if r not in finished)}


def reconcile(auth, events, *, run_id, clock, authorization_sha256):
    """Measured launcher runtime of one verified-terminal job, charged exactly once; the original
    reservation stays in the ADMITTED history and is released from the active sum by this event."""
    validate_authorization(auth)
    own = [e for e in events if e.get('session_id') == auth['session_id']]
    admitted = [e for e in own if e['event'] == 'ADMITTED' and e['run_id'] == run_id]
    finished = [e for e in own if e['event'] == 'FINISHED' and e['run_id'] == run_id]
    if len(admitted) != 1:
        raise AdmissionError('Reconciliation requires exactly one admission for the run')
    if any(e['event'] == 'RECONCILED' and e['run_id'] == run_id for e in own):
        raise AdmissionError('Run already reconciled; a completion is charged exactly once')
    start = admitted[0]
    if start.get('authorization_sha256') != authorization_sha256:
        raise AdmissionError('Owner mismatch: admission belongs to a different authorization')
    if not finished:
        raise AdmissionError('No terminal receipt; the reservation stays held until independently reconciled')
    end = finished[-1]
    if end.get('cleanup_verified') is not True:
        raise AdmissionError('Terminal receipt without verified cleanup; reservation held')
    if end.get('boot_id') != start.get('boot_id') or end.get('authorization_sha256') != authorization_sha256:
        raise AdmissionError('Terminal receipt from another boot or authorization')
    by_utc = _finite(end['utc'], 'end utc') - _finite(start['utc'], 'start utc')
    by_monotonic = _finite(end['monotonic'], 'end monotonic') - _finite(start['monotonic'], 'start monotonic')
    if by_utc < 0 or by_monotonic < 0:
        raise AdmissionError('Negative measured runtime; ledger clocks inconsistent')
    measured = max(by_utc, by_monotonic)
    reservation = _finite(start['allocation_seconds'], 'reservation')
    if clock.boot_id != auth['boot_id'] or clock.utc + .001 < end['utc'] or clock.monotonic + .001 < end['monotonic']:
        raise AdmissionError('Clock rollback or reboot since the terminal receipt')
    return {'session_id': auth['session_id'], 'authorization_sha256': authorization_sha256, 'run_id': run_id,
            'event': 'RECONCILED', 'status': end.get('status'), 'reservation_seconds': reservation,
            'measured_launcher_seconds': measured, 'charged_seconds': measured,
            'released_reservation_seconds': max(0., reservation - measured), 'overrun_seconds': max(0., measured - reservation),
            'measurement': 'launcher ledger clocks: ADMITTED to verified FINISHED (max of UTC and monotonic differences); independent of probe claims',
            'utc': clock.utc, 'monotonic': clock.monotonic, 'boot_id': clock.boot_id}


def admit(auth, events, *, category, name, justification, seconds, cleanup_seconds,
          training_prerequisites, clock, authorization_sha256):
    """Pure admission; callers hold the global job lock and persist ADMITTED before launch."""
    validate_authorization(auth)
    if clock.boot_id != auth['boot_id']:
        raise AdmissionError('Boot identity changed; original monotonic authorization cannot be reset')
    _finite(clock.utc, 'current UTC'); _finite(clock.monotonic, 'current monotonic time')
    if clock.utc < auth['start_unix'] or clock.monotonic < auth['start_monotonic']:
        raise AdmissionError('Clock precedes session start')
    own = [e for e in events if e.get('session_id') == auth['session_id']]
    for event in own:
        if event.get('authorization_sha256') != authorization_sha256:
            raise AdmissionError('Authorization changed within the existing session ledger')
        if (event.get('boot_id') != clock.boot_id or clock.utc + .001 < event['utc']
                or clock.monotonic + .001 < event['monotonic']):
            raise AdmissionError('Clock rollback or reboot since a durable session event')
    active = {e['run_id'] for e in own if e['event'] == 'ADMITTED'}
    latest_terminal = {e['run_id']: e for e in own if e['event'] == 'FINISHED'}
    terminal = {run_id for run_id, event in latest_terminal.items() if event.get('cleanup_verified') is True}
    if active - terminal:
        raise AdmissionError('Prior admitted job lacks a terminal receipt; inspect/recover it first')
    seconds = _finite(seconds, 'job allocation')
    cleanup_seconds = _finite(cleanup_seconds, 'cleanup allocation')
    if not 30 <= cleanup_seconds <= 120 or seconds <= cleanup_seconds + 1:
        raise AdmissionError('Job must reserve 30..120 seconds for cleanup')
    limits = auth['limits']
    if category == 'diagnostic':
        cap = limits['default_diagnostic_seconds']
    elif category in {'asset-build', 'initialization', 'integration', 'evaluation', 'inference'}:
        cap = limits['maximum_named_batch_seconds']
        if not name or not justification.strip():
            raise AdmissionError('Named batch requires a name and manifest justification')
    elif category == 'training':
        cap = limits['maximum_training_job_seconds']
        if cap <= 0 or limits['maximum_total_training_allocation_seconds'] <= 0:
            raise AdmissionError('Training is not authorized in this session')
        required = {'working_environment', 'controller_gap', 'evaluation_criterion', 'checkpoint_plan'}
        if (not name or not justification.strip() or not isinstance(training_prerequisites, dict)
                or any(not isinstance(training_prerequisites.get(k), str)
                       or not training_prerequisites[k].strip() for k in required)):
            raise AdmissionError('Training requires recorded environment, gap, criterion and checkpoints')
        allocated = sum(e['allocation_seconds'] for e in own
                        if e['event'] == 'ADMITTED' and e.get('category') == 'training')
        if allocated + seconds > limits['maximum_total_training_allocation_seconds']:
            raise AdmissionError('Cumulative training allocation exceeded; failed jobs are not refunded')
    else:
        raise AdmissionError('Unknown job category')
    if seconds > cap:
        raise AdmissionError(f'{category} allocation exceeds its limit')
    session_cap = limits.get('maximum_session_allocated_seconds')
    if session_cap is not None:
        # Cumulative RESERVED launcher time of this session (conservative: allocations,
        # not measured time; failed and stopped jobs are never refunded).
        allocated_total = sum(e['allocation_seconds'] for e in own if e['event'] == 'ADMITTED')
        if allocated_total + seconds > _finite(session_cap, 'maximum_session_allocated_seconds'):
            raise AdmissionError('Cumulative session allocation would exceed the authorized launcher allowance')
    measured_cap = limits.get('maximum_session_measured_launcher_seconds')
    if measured_cap is not None:
        # Measured-runtime allowance: charged completed runtime + active reservations + this reservation.
        account = session_accounting(auth, events)
        if account['charged_completed_seconds'] + account['active_reservation_seconds'] + seconds > _finite(measured_cap, 'maximum_session_measured_launcher_seconds'):
            raise AdmissionError('Charged runtime plus active reservations would exceed the measured launcher allowance')
    monotonic_end = auth['start_monotonic'] + auth['experimental_deadline_unix'] - auth['start_unix']
    remaining = min(auth['experimental_deadline_unix'] - clock.utc, monotonic_end - clock.monotonic)
    if seconds > remaining:
        raise AdmissionError('Whole job including cleanup does not fit experimental deadline')
    return {'session_id': auth['session_id'], 'authorization_sha256': authorization_sha256,
            'category': category, 'job_name': name, 'justification': justification,
            'training_prerequisites': training_prerequisites, 'allocation_seconds': seconds,
            'cleanup_seconds': cleanup_seconds, 'boot_id': clock.boot_id,
            'utc': clock.utc, 'monotonic': clock.monotonic,
            'hard_deadline_utc': clock.utc + seconds,
            'hard_deadline_monotonic': clock.monotonic + seconds,
            'stop_deadline_utc': clock.utc + seconds - cleanup_seconds,
            'stop_deadline_monotonic': clock.monotonic + seconds - cleanup_seconds,
            'experimental_deadline_utc': auth['experimental_deadline_unix'],
            'experimental_deadline_monotonic': monotonic_end}


def remaining_execution(job, clock):
    if clock.boot_id != job['boot_id']:
        return 0.
    if clock.utc < job['utc'] or clock.monotonic < job['monotonic']:
        return 0.
    return max(0., min(job['stop_deadline_utc'] - clock.utc,
                       job['stop_deadline_monotonic'] - clock.monotonic,
                       job['experimental_deadline_utc'] - clock.utc,
                       job['experimental_deadline_monotonic'] - clock.monotonic))


_EXCLUDED = {'.git', '__pycache__', '.pytest_cache', '.mypy_cache'}


def tree_hashes(root):
    root = Path(root)
    result = {}
    for path in sorted(root.rglob('*')):
        rel = path.relative_to(root)
        if _EXCLUDED.intersection(rel.parts):
            continue
        if path.is_symlink():
            raise AdmissionError(f'Symlink input must be explicitly materialized first: {rel}')
        if path.is_file():
            result[str(rel)] = digest(path)
        elif not path.is_dir():
            raise AdmissionError(f'Non-regular input: {rel}')
    return result


def snapshot_tree(source, destination):
    """Copy bytes (never hardlink) and refuse changes during capture; mount only the copy."""
    source, destination = Path(source), Path(destination)
    if source.is_symlink() or not source.is_dir():
        raise AdmissionError('Snapshot source must be an actual directory')
    before = tree_hashes(source)
    destination.mkdir(parents=True, exist_ok=False)
    for rel in before:
        target = destination / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / rel, target)
    after = tree_hashes(source)
    copied = tree_hashes(destination)
    if before != after or copied != before:
        raise AdmissionError('Source changed during capture; immutable snapshot refused')
    for path in destination.rglob('*'):
        executable = not path.is_dir() and bool((source / path.relative_to(destination)).stat().st_mode & 0o111)
        path.chmod(0o555 if path.is_dir() or executable else 0o444)
    destination.chmod(0o555)
    return copied


def snapshot_public(repo, destination):
    repo, destination = Path(repo), Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    before_all = {name + '/' + rel: value for name in ['isaac', 'tools']
                  for rel, value in tree_hashes(repo / name).items()}
    hashes = {}
    for name in ['isaac', 'tools']:
        for rel, value in snapshot_tree(repo / name, destination / name).items():
            hashes[name + '/' + rel] = value
    tracked = subprocess.check_output(['git', '-C', str(repo), 'ls-files', '-z']).decode().split('\0')
    for name in tracked:
        if not name or '/' in name:
            continue
        source = repo / name
        if source.is_symlink() or not source.is_file():
            raise AdmissionError('Tracked root input is missing or a symlink')
        before = digest(source)
        shutil.copyfile(source, destination / name)
        if digest(source) != before or digest(destination / name) != before:
            raise AdmissionError('Root input changed during capture')
        (destination / name).chmod(0o555 if source.stat().st_mode & 0o111 else 0o444)
        hashes[name] = before
    after_all = {name + '/' + rel: value for name in ['isaac', 'tools']
                 for rel, value in tree_hashes(repo / name).items()}
    if before_all != after_all or any(hashes.get(k) != v for k, v in before_all.items()):
        raise AdmissionError('Public tree changed during multi-directory capture')
    destination.chmod(0o555)
    return hashes


def private_input(spec, workspace):
    """Explicit private source directory grant; no automatic private-repository copy."""
    name, separator, raw = spec.partition('=')
    if not separator or not name or not all(c.isalnum() or c in '-_' for c in name):
        raise AdmissionError('Private input must be NAME=PATH with a simple name')
    path = Path(raw).resolve()
    allowed = [Path(r['worktree']).resolve() for r in workspace['repositories'] if r['visibility'] == 'private']
    allowed += [Path(root).resolve() for root in workspace.get('private_input_roots', [])]
    if not path.is_dir() or not any(path.is_relative_to(root) for root in allowed):
        raise AdmissionError('Private input must be inside a declared private repository')
    return name, path


def active_session(campaign, clock):
    """Resolve ACTIVE_SESSION.json to a live authorization; a stale pointer (missing file, session id
    mismatch, other boot, or a passed final deadline) is refused rather than silently reused."""
    campaign = Path(campaign)
    pointer = json.loads((campaign / 'ACTIVE_SESSION.json').read_text())
    path = (campaign / pointer['authorization']).resolve()
    if not path.is_relative_to(campaign / 'sessions') or not path.is_file():
        raise AdmissionError('ACTIVE_SESSION points outside sessions or to a missing authorization')
    auth = json.loads(path.read_text())
    validate_authorization(auth)
    if auth['session_id'] != pointer.get('session_id') or path.parent.name != auth['session_id']:
        raise AdmissionError('ACTIVE_SESSION session id disagrees with the authorization file')
    if clock.boot_id != auth['boot_id']:
        raise AdmissionError('ACTIVE_SESSION authorization belongs to another boot')
    if clock.utc >= auth['final_deadline_unix']:
        raise AdmissionError('ACTIVE_SESSION authorization has passed its final deadline')
    return path, auth


def _cli(argv):
    """python3 sim_admission.py reconcile <campaign> [run_id ...] | account <campaign>"""
    import sys
    command, campaign = argv[0], Path(argv[1]).resolve()
    clock = Clock.now()
    path, auth = active_session(campaign, clock)
    ledger = path.parent / 'ledger.jsonl'
    events = read_ledger(ledger)
    auth_hash = digest(path)
    if command == 'account':
        print(json.dumps(session_accounting(auth, events), indent=2)); return 0
    if command == 'reconcile':
        targets = argv[2:] or session_accounting(auth, events)['unreconciled_finished_runs']
        for run_id in targets:
            event = reconcile(auth, events, run_id=run_id, clock=Clock.now(), authorization_sha256=auth_hash)
            ledger_event(ledger, event); events.append(event)
            print(json.dumps({k: event[k] for k in ('run_id', 'status', 'reservation_seconds', 'measured_launcher_seconds', 'charged_seconds', 'released_reservation_seconds')}))
        return 0
    raise SystemExit(__doc__)


if __name__ == '__main__':
    import sys
    raise SystemExit(_cli(sys.argv[1:]))
