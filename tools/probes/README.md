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

The hand probe uses CPU PhysX dynamics and GPU rendering in the installed
configuration. GPU availability alone does not establish GPU-accelerated physics.
It reports actual scene attributes, live shape counts and offsets, root drive
targets, measured joint coordinates/velocities, and simulated contact impulses.
Contact reporting starts before `World.reset()`, which can already step physics.
Cooking-service hull dumps are explicitly separate from live actor shape readback.

The default donor's convex-decomposed palm obstructs its thumb cavity. Merely
moving CollisionAPI onto the Mesh or increasing the palm decomposition to 128
hulls did not qualify its dynamics. The following diagnostic variants preserve
those competing hypotheses and their scope:

| Mode | Changed assumption | Qualification limit |
| --- | --- | --- |
| `zero-gravity` | Gravity removed | Unloaded diagnostic only |
| `refined-palm` | 128-hull palm decomposition | Source mesh/mass and collision pairs preserved |
| `mesh-colliders` | Collision schemas moved from wrapper to Mesh | No source triangle changes |
| `refined-palm-mesh-colliders` | Both preceding changes | Does not establish cavity clearance |
| `zero-gravity-clearance-control` | One palm/thumb pair filtered | Collision qualification explicitly excluded |
| `static-palm-bench` | Original palm triangles made an exclusive static collider | Fixed bench only; cannot follow a moving hand |
| `blocked-index-static-palm-bench` | Same free/blocked index trajectory and measured obstacle contacts | Fixed bench obstruction test only |
| `tgs-forces-blocked-index-static-palm-bench` | Gravity/external forces at every TGS iteration | Same physical/contact acceptance thresholds |
| `tgs-forces-velocity8-blocked-index-static-palm-bench` | Eight velocity iterations, retaining 32 position iterations | Same thresholds; strongest completed bench diagnostic |

Zero-gravity variants also exist for refined-palm, mesh-colliders, their
combination, and static-palm-bench; see `--help`. The static triangle fixture
retains source geometry, articulation mass and URDF adjacent-pair exclusions.
Nonadjacent thumb collision remains enabled. Every such run sets
`collision_qualification_excluded=true`; a bench PASS cannot qualify the exact
asset, a moving palm, grasping, tactile telemetry, or writing.

The obstruction protocol first measures a free closing trajectory, opens the
finger, then introduces a 16 mm cube at the measured closed finger position.
It repeats the identical command and checks matching initial states, no initial
obstacle contact, stable final windows, actual resistance, and sustained measured
impulse from the index only. Both final position spread and reported velocity
must pass. Cube contact offset is explicitly 0.5 mm, rest offset and torsional
patch radii zero, friction 0.5 and restitution zero; these are diagnostic choices,
not measured hardware parameters. The serialized cube getters read authored USD
properties. Default FixedCuboid settings have a 100 mm contact margin.

The TGS external-force flag addresses the documented steady-position/nonzero-
velocity discrepancy; the velocity-iteration variant tests contact convergence.
See [NVIDIA's solver explanation](https://nvidia-omniverse.github.io/PhysX/physx/5.8.0/docs/Simulation.html#tgs-steady-state-velocity-and-position-discrepancy).
The flag exists in the installed Isaac 5.1 schema; newer documentation does not
imply that the installed engine was upgraded.

Reproduce the strongest bench diagnostic in a fresh output directory, subject to
the remaining private budget:

```sh
python3 tools/run_isolated_isaac.py tools/probes/inspire_hand.py \
  --workspace-lock ../workspace-lock.json \
  --source-assets ../generated/ftp_donor \
  --output ../evidence/hand-blocked-new --seconds 300 \
  --probe-mode tgs-forces-velocity8-blocked-index-static-palm-bench
```

Isaac can exit Python during `SimulationApp.close()` before a trailing assertion
or `finally` block executes. Therefore a zero process status alone is insufficient:
the launcher requires an explicit completed receipt and hashes each listed
nonempty artifact. Read `run.json`, `probe.json`, and the raw trace together.
Failed and incomplete outputs remain evidence; never overwrite them to retry.
New runs also retain the executed probe, launcher and binary Git diff, with
hashes. This improves source custody for uncommitted diagnostic increments;
older runs without these snapshots retain their original evidence limitations.

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
