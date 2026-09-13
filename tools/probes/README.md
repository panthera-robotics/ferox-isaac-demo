# Isolated Isaac diagnostics

These probes exercise the installed Isaac Sim 5.1 environment. They do not certify
the RH56E2 embodiment, a grasp, a standing controller, or physical writing. All
artifacts belong in a private workspace outside this public repository.

The launcher uses an existing Docker image and cache, resolves the image to its
immutable ID, denies container networking, uses private IPC, drops capabilities,
and mounts source/assets read-only. It refuses overlapping diagnostics and bounds
both each run and cumulative elapsed time using the private workspace lock.
It does not provision Docker, caches, a GPU driver, or source assets.

Run from this worktree, adjusting private paths:

```sh
python3 tools/run_isolated_isaac.py tools/probes/legacy_asset.py \
  --workspace-lock ../workspace-lock.json \
  --source-assets ../sources/nvidia-5.1 \
  --output ../evidence/asset-new --seconds 300
python3 tools/run_isolated_isaac.py tools/probes/physics_camera.py \
  --workspace-lock ../workspace-lock.json \
  --policy ../ferox-g1-locomotion/policy \
  --output ../evidence/camera-new --seconds 300
python3 tools/run_isolated_isaac.py tools/probes/inspire_hand.py \
  --workspace-lock ../workspace-lock.json \
  --source-assets ../generated/ftp_donor \
  --output ../evidence/hand-new --seconds 300
```

Every output directory must be new, with a private parent. Do not edit simulator
or probe inputs while a run is active: changed input hashes invalidate the run.
The workspace lock lists four repositories with `repository`, `visibility`, and
absolute `worktree` paths, `hardware_authorized: false`, a relative
`identity_contract` JSON path, and `gpu_budget` with
`maximum_single_run_seconds` and `maximum_campaign_gpu_seconds`. The identity
contract must also explicitly deny hardware authority. The launcher reads live
Git revisions, so stale informational HEADs in the lock cannot impersonate the
executed revision. Incomplete earlier run receipts require inspection first.

For the legacy sensor probe, the offline asset root needs `Isaac/` and `NVIDIA/`
directories, and the pinned NVIDIA 5.1 asset at
`Isaac/Sensors/NVIDIA/Example_Rotary.usda`. Preserve its source URL, license and
SHA-256 beside the private asset. The ground fixture is an analytic local plane.
The suite checks authored LiDAR attributes; it does not qualify live scan data.

`physics_camera.py` runs the inherited Dex5 robot and its existing policy for
400 measured physics steps, with actual ROS RGB/depth receivers in the isolated
container. `inspire_hand.py` imports the provisional FTP donor bench URDF from
`tools/inspire_asset.py`, preserving its mass and geometry, driving only six
independent axes and recording twelve coupled coordinates. Its rigid mimic
constraint is a declared nominal model, not measured hardware compliance.

Isaac can exit Python during `SimulationApp.close()` before a trailing assertion
or `finally` block executes. Therefore a zero process status alone is insufficient:
the launcher requires an explicit completed receipt and hashes each listed
nonempty artifact. Read `run.json`, `probe.json`, and the raw trace together.
Failed and incomplete outputs remain evidence; never overwrite them to retry.

CPU checks for the new contracts and evaluator:

```sh
python3 -m unittest tools.tests.test_inspire_asset \
  tools.tests.test_inspire_arm_adapter tools.tests.test_contact_ink \
  tools.tests.test_probe_receipt
python3 tools/tests/test_twin_contract.py
python3 tools/tests/test_isaaclab_cfg.py
```

The contact evaluator consumes measured nib/board contact samples and explicit
attachment/support status. Synthetic traces test evaluator behavior only. Its
SVG exports cannot establish a physical grasp or standing-writing qualification.
