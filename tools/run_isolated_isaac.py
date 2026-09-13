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


def command(*args):
    return subprocess.check_output(args, text=True).strip()


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
    parser.add_argument("script", help="Python probe relative to this repository")
    parser.add_argument("--output", type=Path, required=True, help="New private run directory")
    parser.add_argument("--seconds", type=int, default=300)
    parser.add_argument("--cache", type=Path, default=Path("/data/isaac_cache"))
    parser.add_argument("--image", default="nvcr.io/nvidia/isaac-sim:5.1.0")
    parser.add_argument("--policy", type=Path, help="Optional local policy directory, mounted read-only")
    parser.add_argument("--source-assets", type=Path, help="Optional local donor assets, mounted read-only")
    parser.add_argument("--workspace-lock", type=Path, required=True,
                        help="Private cross-repository lock; live revisions are read back before execution")
    parser.add_argument("--probe-mode", choices=["default", "zero-gravity", "refined-palm", "zero-gravity-refined-palm",
                        "mesh-colliders", "zero-gravity-mesh-colliders",
                        "refined-palm-mesh-colliders", "zero-gravity-refined-palm-mesh-colliders",
                        "zero-gravity-clearance-control", "static-palm-bench", "zero-gravity-static-palm-bench",
                        "blocked-index-static-palm-bench", "tgs-forces-blocked-index-static-palm-bench",
                        "tgs-forces-velocity8-blocked-index-static-palm-bench"], default="default",
                        help="Explicit diagnostic variant; zero gravity is never a physical qualification")
    args = parser.parse_args()
    if not 10 <= args.seconds <= 900:
        parser.error("Each diagnostic must be bounded to 10..900 seconds; training is out of scope")
    repo = Path(__file__).resolve().parents[1]
    script = (repo / args.script).resolve()
    if not script.is_relative_to(repo) or not script.is_file() or script.suffix != ".py":
        parser.error("Probe must be an existing Python file inside this repository")
    if args.probe_mode != 'default' and script.name != 'inspire_hand.py':
        parser.error("Diagnostic variants are specific to the Inspire hand probe")
    lock = open("/tmp/panthera-isolated-isaac.lock", "a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    lock_path = args.workspace_lock.resolve()
    workspace = json.loads(lock_path.read_text())
    repositories = []
    for pinned in workspace["repositories"]:
        path = Path(pinned["worktree"])
        repositories.append({"repository": pinned["repository"], "visibility": pinned["visibility"],
                             "head": command("git", "-C", str(path), "rev-parse", "HEAD"),
                             "branch": command("git", "-C", str(path), "branch", "--show-current"),
                             "dirty": command("git", "-C", str(path), "status", "--porcelain")})
    if len(repositories) != 4 or workspace.get("hardware_authorized") is not False:
        parser.error("Expected a four-repository simulation-only workspace lock")
    identity_path = lock_path.parent / workspace["identity_contract"]
    identity = json.loads(identity_path.read_text())
    if identity.get("hardware_authorized") is not False:
        parser.error("Identity contract must explicitly deny hardware authority")
    budget = workspace["gpu_budget"]
    elapsed = float(budget.get('unmanifested_diagnostic_reserve_seconds', 0.))
    for run in (lock_path.parent / "evidence").glob("*/run.json"):
        prior = json.loads(run.read_text())
        if "end_unix" not in prior:
            parser.error(f"Prior run lacks an end receipt; inspect it before resuming: {run}")
        elapsed += max(0., prior["end_unix"] - prior["start_unix"])
    if (args.seconds > budget["maximum_single_run_seconds"]
            or elapsed + args.seconds > budget["maximum_campaign_gpu_seconds"]):
        parser.error("Diagnostic exceeds the workspace's bounded compute allowance")
    output = args.output.resolve()
    if output.parent != (lock_path.parent / 'evidence').resolve():
        parser.error("Run output must be one new child of the private workspace evidence directory")
    if lock_path.parent.stat().st_mode & 0o077:
        parser.error("Workspace directory must be private (mode 0700)")
    output.mkdir(parents=True, exist_ok=False, mode=0o777)
    output.chmod(0o777)  # Image UID 1234 writes artifacts; parent should be private.
    shutil.copyfile(script, output / 'executed_probe.py')
    shutil.copyfile(Path(__file__), output / 'executed_launcher.py')
    (output / 'uncommitted.patch').write_text(command('git', '-C', str(repo), 'diff', '--binary', 'HEAD') + '\n')
    image = json.loads(command("docker", "image", "inspect", args.image))[0]
    name = "panthera_sim_" + uuid.uuid4().hex[:12]
    mounts = [(repo / "isaac", "/workspace/ferox_isaac", "ro"),
              (repo / "tools", "/workspace/ferox_tools", "ro"),
              (repo, "/workspace/sim-source", "ro"), (output, "/evidence", "rw")]
    for key, target in {"kit": "kit/cache", "ov": ".cache/ov", "pip": ".cache/pip",
                        "gl": ".cache/nvidia/GLCache", "compute": ".nv/ComputeCache",
                        "warp": ".cache/warp"}.items():
        source = args.cache.resolve() / key
        if not source.is_dir():
            parser.error(f"Missing installed cache {source}; preserve and repair the cache explicitly")
        mounts.append((source, "/isaac-sim/" + target, "rw"))
    for path, target in [(args.policy, "/policy"), (args.source_assets, "/source-assets")]:
        if path is not None:
            if not path.is_dir():
                parser.error(f"Missing read-only source {path}")
            mounts.append((path.resolve(), target, "ro"))
    input_roots = [("isaac", repo / "isaac"), ("tools", repo / "tools")]
    if args.policy:
        input_roots.append(("policy", args.policy.resolve()))
    if args.source_assets:
        input_roots.append(("source_assets", args.source_assets.resolve()))
    create = ["docker", "create", "--name", name, "--gpus", "all", "--user", "1234:1234",
              "--network", "none", "--ipc", "private", "--cap-drop", "ALL",
              "--security-opt", "no-new-privileges", "--shm-size", "2g",
              "-e", "ACCEPT_EULA=Y", "-e", "PRIVACY_CONSENT=Y",
              "-e", "PYTHONDONTWRITEBYTECODE=1", "-e", "ROS_DOMAIN_ID=73",
              "-e", "ROS_LOCALHOST_ONLY=1", "-e", "ROS_DISTRO=humble",
              "-e", "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
              "-e", "LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib",
              "-e", "PANTHERA_SIM_AUTHORIZED=1", "-e", "PANTHERA_SIM_RUN_ID=" + name,
              "-e", "PANTHERA_PROBE_MODE=" + args.probe_mode]
    for source, target, mode in mounts:
        create += ["--mount", f"type=bind,source={source},target={target}" + (",readonly" if mode == "ro" else "")]
    create += ["--entrypoint", "/usr/bin/timeout", image["Id"], "--signal=TERM",
               "--kill-after=15", str(args.seconds), "/isaac-sim/python.sh",
               "/workspace/sim-source/" + str(script.relative_to(repo))]
    manifest = {"schema_version": 1, "kind": "bounded_simulation_diagnostic",
                "run_id": name, "start_unix": time.time(), "hardware_authorized": False,
                "image_id": image["Id"], "image_digests": image.get("RepoDigests", []),
                "architecture": image["Architecture"], "maximum_seconds": args.seconds,
                "repo_sha": command("git", "-C", str(repo), "rev-parse", "HEAD"),
                "branch": command("git", "-C", str(repo), "branch", "--show-current"),
                "dirty_status": command("git", "-C", str(repo), "status", "--porcelain"),
                "script_sha256": hashlib.sha256(script.read_bytes()).hexdigest(),
                "gpu": command("nvidia-smi", "--query-gpu=name,uuid,driver_version,memory.total", "--format=csv,noheader"),
                "command": create, "status": "STARTING"}
    manifest["input_sha256"] = input_hashes(input_roots)
    manifest['source_snapshot_sha256'] = {name: hashlib.sha256((output / name).read_bytes()).hexdigest()
        for name in ['executed_probe.py', 'executed_launcher.py', 'uncommitted.patch']}
    manifest.update(workspace_repositories=repositories,
                    workspace_lock_sha256=hashlib.sha256(lock_path.read_bytes()).hexdigest(),
                    identity_contract_sha256=hashlib.sha256(identity_path.read_bytes()).hexdigest(),
                    identity_contract=identity, diagnostic_seconds_consumed_before_run=elapsed,
                    maximum_campaign_gpu_seconds=budget["maximum_campaign_gpu_seconds"])
    def save():
        (output / "run.json").write_text(json.dumps(manifest, indent=2) + "\n")
    save()
    created = False
    result = 1
    try:
        command(*create)
        created = True
        inspected = json.loads(command("docker", "inspect", name))[0]
        host = inspected["HostConfig"]
        if (host["NetworkMode"] != "none" or host["IpcMode"] != "private"
                or host["Privileged"] or host["Devices"] or host["PortBindings"]
                or host["CapAdd"] or host["CapDrop"] != ["ALL"]):
            raise RuntimeError("Container isolation read-back failed")
        manifest["isolation"] = {key: host[key] for key in
            ["NetworkMode", "IpcMode", "Privileged", "Devices", "PortBindings", "CapAdd", "CapDrop", "SecurityOpt"]}
        save()
        with (output / "console.log").open("w") as log:
            result = subprocess.run(["docker", "start", "--attach", name], stdout=log,
                                    stderr=subprocess.STDOUT, timeout=args.seconds + 30).returncode
        manifest["container_exit"] = json.loads(command("docker", "inspect", name))[0]["State"]["ExitCode"]
        result = result or manifest["container_exit"]
        # Kit shutdown can terminate Python before a trailing assertion/exit.
        # A zero process status therefore needs an explicit completed probe result.
        receipt = output / "probe.json"
        if receipt.is_file():
            manifest["probe"] = validate_receipt(output)
            if manifest["probe"].get("status") != "PASS":
                result = result or 1
        else:
            manifest["probe"] = {"status": "MISSING", "reason": "No completed probe receipt"}
            result = result or 1
        after_hashes = input_hashes(input_roots)
        if after_hashes != manifest["input_sha256"]:
            manifest["changed_inputs_during_run"] = sorted(
                key for key in set(after_hashes) | set(manifest["input_sha256"])
                if after_hashes.get(key) != manifest["input_sha256"].get(key))
            result = result or 1
        manifest.update(status="PASS" if result == 0 else "FAIL", exit_code=result)
    except BaseException as exc:
        manifest.update(status="FAIL", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        if created:
            # A stop timeout must not prevent removal or the final evidence save.
            for cleanup in (["docker", "stop", "--timeout", "5", name], ["docker", "rm", "--force", name]):
                try:
                    cleaned = subprocess.run(cleanup, capture_output=True, text=True, timeout=20)
                    if cleaned.returncode:
                        raise RuntimeError(cleaned.stderr.strip())
                except Exception as exc:
                    manifest.setdefault("cleanup_errors", []).append(f"{cleanup[1]}: {type(exc).__name__}: {exc}")
                    manifest["status"] = "FAIL"
                    result = result or 1
        manifest["end_unix"] = time.time()
        save()
    print(f"{manifest['status']}: {output}")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
