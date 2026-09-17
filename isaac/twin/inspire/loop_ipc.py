"""File-based private IPC between the simulator probe and a co-admitted model sidecar (sprint K K4).

Both containers share one job-private directory (bind-mounted, no network). The probe writes numbered observation
bundles ``obs/<k:06d>.json`` (+ ``obs/<k:06d>.png``) and waits for ``act/<k:06d>.json``; the sidecar answers each
observation exactly once. Every message carries the run token and the observation id, so stale, duplicate, foreign or
malformed answers are refused before anything moves. Writes are atomic (temp file + rename). Pure Python, unit-tested.
"""
import json
import math
import os
import time
from pathlib import Path


class LoopIPCError(RuntimeError):
    pass


def _atomic_write(path: Path, payload: bytes):
    tmp = path.with_suffix(path.suffix + '.tmp')
    with open(tmp, 'wb') as f:
        f.write(payload); f.flush(); os.fsync(f.fileno())
    os.replace(tmp, path)


def layout(root: Path):
    root = Path(root)
    for d in ('obs', 'act', 'log'):
        (root / d).mkdir(parents=True, exist_ok=True)
    return root


def write_observation(root: Path, obs_id: int, run_token: str, payload: dict, image_bytes: bytes | None = None):
    """Publish observation ``obs_id`` (payload must be JSON, finite); returns the JSON path."""
    root = Path(root)
    if not isinstance(obs_id, int) or obs_id < 0:
        raise LoopIPCError('obs_id must be a non-negative integer')
    body = dict(payload); body.update({'obs_id': obs_id, 'run_token': run_token, 'published_unix': time.time()})
    if image_bytes is not None:
        _atomic_write(root / 'obs' / ('%06d.png' % obs_id), image_bytes); body['image'] = 'obs/%06d.png' % obs_id
    _atomic_write(root / 'obs' / ('%06d.json' % obs_id), json.dumps(body, allow_nan=False).encode())
    return root / 'obs' / ('%06d.json' % obs_id)


def write_action(root: Path, obs_id: int, run_token: str, payload: dict):
    body = dict(payload); body.update({'obs_id': obs_id, 'run_token': run_token, 'answered_unix': time.time()})
    _atomic_write(Path(root) / 'act' / ('%06d.json' % obs_id), json.dumps(body, allow_nan=False).encode())


def _finite_matrix(rows, width):
    if not isinstance(rows, list) or not rows:
        return False
    for r in rows:
        if not isinstance(r, list) or len(r) != width:
            return False
        for v in r:
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                return False
    return True


def validate_action_message(msg: dict, *, expected_obs_id: int, run_token: str, dims: dict):
    """Refuse stale/future/foreign/malformed answers. Returns the action dict (key -> rows) or raises LoopIPCError."""
    if not isinstance(msg, dict):
        raise LoopIPCError('action message is not an object')
    if msg.get('run_token') != run_token:
        raise LoopIPCError('action from another run (token mismatch): refused')
    if msg.get('obs_id') != expected_obs_id:
        raise LoopIPCError('action obs_id %r does not match the pending observation %d (stale, future or duplicate): refused' % (msg.get('obs_id'), expected_obs_id))
    if msg.get('status') != 'ok':
        raise LoopIPCError('sidecar reported %s: %s' % (msg.get('status'), msg.get('error')))
    action = msg.get('action')
    if not isinstance(action, dict) or set(action) != set(dims):
        raise LoopIPCError('action keys %s != expected %s' % (sorted(action) if isinstance(action, dict) else action, sorted(dims)))
    horizons = set()
    for k, w in dims.items():
        if not _finite_matrix(action[k], w):
            raise LoopIPCError('action[%s] is not a finite %d-wide matrix' % (k, w))
        horizons.add(len(action[k]))
    if len(horizons) != 1:
        raise LoopIPCError('action keys disagree on the horizon: %s' % sorted(horizons))
    return action


def wait_for_action(root: Path, obs_id: int, *, run_token: str, dims: dict, timeout_s: float, poll_s: float = 0.02, sidecar_alive=lambda: True):
    """Block until act/<obs_id>.json exists and validates; raise LoopIPCError on timeout, a dead sidecar or a refused message."""
    path = Path(root) / 'act' / ('%06d.json' % obs_id); deadline = time.monotonic() + timeout_s
    while True:
        if path.exists():
            try:
                msg = json.loads(path.read_text())
            except json.JSONDecodeError as exc:
                raise LoopIPCError('malformed action JSON: %s' % exc)
            return validate_action_message(msg, expected_obs_id=obs_id, run_token=run_token, dims=dims)
        if not sidecar_alive():
            raise LoopIPCError('sidecar is not alive while observation %d is pending' % obs_id)
        if time.monotonic() >= deadline:
            raise LoopIPCError('no action for observation %d within %.1f s (inference timeout)' % (obs_id, timeout_s))
        time.sleep(poll_s)


def pending_observations(root: Path, answered: set):
    """Sidecar helper: observation ids with a JSON present and no answer yet, in order."""
    ids = []
    for p in sorted((Path(root) / 'obs').glob('*.json')):
        try:
            k = int(p.stem)
        except ValueError:
            continue
        if k not in answered and not (Path(root) / 'act' / p.name).exists():
            ids.append(k)
    return ids
