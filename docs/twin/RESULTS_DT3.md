# DT3 — Dex5-1P hands on the G1 twin

**Verdict: PASS.** `HAND=dex5_1p` loads, the robot walks with the hands attached, and the
fingers open and close. 69 DOF in one articulation, total mass 35.004757 kg at 0.00 % error,
body joint order unchanged, Isaac suite 11/11.

Run on 2026-08-18, branch `mohammed/twin-campaign`, `SIM_WORLD=hospital`, `TWIN=1`.

---

## 1. What was built

Unitree's G1 29-DoF and Dex5-1P descriptions are **merged into one URDF** and imported once
(`tools/merge_dex5_urdf.py`, `tools/import_g1_dex5.py`, `scripts/12_import_g1_dex5.sh`). The
merge is a concatenation: every link, joint, inertial, limit and mesh comes across byte-for-byte
from Unitree's files. Nothing is scaled, no inertia is recomputed, and no joint is renamed
(campaign rule 9).

`HAND=none` (default) still loads the bare-wristed robot every earlier gate ran against, so a
hand problem can always be bisected against it. `HAND=dex5_1p` selects the merged asset.

### Why a merged URDF and not USD composition

The obvious approach — reference each hand into the G1 stage and bolt it on with a
`PhysicsFixedJoint` — was built first and **verified green on every check that could be made
from the stage**:

| check | result |
|---|---|
| hand joints present | 40 (2 × 20) |
| hand mass | 2.003615 kg, exact |
| mount offset, both sides | exact to 0.0000 mm |
| flange fixed joints | 2, both with valid body relationships |
| `Hand` variant set | `{None, dex5_1p}` |

And it could not move a finger. PhysX builds its articulation from **one** robot description; a
body introduced by a later reference joins the scene as a maximal-coordinate rigid body. The
articulation stayed at **29 DOF / 30 bodies**. Three placements were tried and behave
identically: the flange joint under the parent link, the flange joint in the robot's `joints/`
scope, and the hand's own `ArticulationRootAPI` deleted so it could not claim its links.

The lesson is recorded in `tools/merge_dex5_urdf.py` rather than in a commit nobody will read:
**properties of the stage are not evidence about the articulation.** Every number in the table
above was true while the hands were inert.

NVIDIA's documented answer for attaching a gripper to an arm is to import the combined URDF, and
it is what Unitree do themselves (`g1_description/merge_g1_29dof_and_inspire_hand.ipynb`).

---

## 2. Results

### Articulation (`evidence/DT3/import_g1_dex5.txt`, `dof_map.txt`)

| quantity | value | expected | verdict |
|---|---|---|---|
| articulation DOF | 69 | 29 body + 2 × 20 hand | PASS |
| bodies | 79 | — | PASS |
| articulation roots | 1, `/g1_29dof_rev_1_0/pelvis` | 1 | PASS |
| total mass | 35.004757 kg | 35.004757 | PASS (0.00 %) |
| default prim | `g1_29dof_rev_1_0` | unchanged | PASS |
| every URDF joint present | 69/69 | 69 | PASS |
| first 29 body DOFs | bit-identical to pre-hand order | identical | PASS |

Total mass is body 33.001142 + L 1.025045 + R 0.978570. Body is 33.341142 **minus the two
0.170 kg `*_rubber_hand` caps** the Dex5 replaces — see §4.

### Walk regression (`evidence/DT3/validate_motion_dex5.txt`)

Same harness, same commands, same checkpoint as DT2.

| motion | DT2 body vel | DT3 body vel | DT2 zmin | DT3 zmin | Δ base height |
|---|---|---|---|---|---|
| forward | vx +0.508 | vx +0.506 | 0.77 | 0.75 | −0.02 |
| reverse | vx −0.349 | vx −0.349 | 0.78 | 0.77 | −0.01 |
| strafe L | vy +0.200 | vy +0.245 | 0.78 | 0.77 | −0.01 |
| strafe R | vy −0.260 | vy −0.222 | 0.78 | 0.77 | −0.01 |
| rot ccw | wz +0.006 | wz −0.015 | 0.79 | 0.79 | 0.00 |
| rot cw | wz +0.000 | wz −0.010 | 0.79 | 0.79 | 0.00 |
| walk+strafe | vx +0.320 vy +0.233 | vx +0.307 vy +0.232 | 0.77 | 0.76 | −0.01 |
| walk+turn | vx +0.311 wz +0.157 | vx +0.318 wz +0.155 | 0.77 | 0.76 | −0.01 |

**Max base-height delta 0.02 m**, inside the ±0.03 m the campaign set, so this is a PASS on its
own terms and the "report the delta, do not tune" fallback was not needed. No gains were touched.

Verdicts are identical to DT2: forward / reverse / strafe / upright PASS, rotate-in-place FAIL.
The rotate failure is the known property of this checkpoint, unchanged by the hands, and is not
a DT3 finding.

### Hand poses (`evidence/DT3/hand_poses.txt`, four PNGs)

Every pose is built **by joint name** and commanded as a drive target.

| pose | max \|commanded − reached\| |
|---|---|
| rest | 0.0010 rad |
| open | 0.0001 rad |
| fist | 0.0000 rad |
| thumb opposition | 0.0000 rad |

`hand_rest.png`, `hand_open.png`, `hand_fist.png`, `hand_thumb_opposition.png` — 33 of 40 joints
driven per pose; the 7 not driven are the passive abductions held at zero (C-13) and the wrist.

### Tests (`evidence/DT3/isaac_tests.txt`) — 11/11

Five are new and all five are drift tripwires for the merged asset: one articulation of 69 DOF,
the 29 body DOFs in their exact pre-hand order, every URDF hand joint present as a DOF, mass and
mount preserved, rubber caps gone.

---

## 3. Joint order does not transfer — read this before writing hand code

`limits.py` clamps a flat 20-vector per hand in URDF document order, and that is what the real
driver speaks. Isaac orders articulation DOFs breadth-first over the whole robot, so each hand's
20 joints land at scattered, non-contiguous, **interleaved** indices:

```
limits.py index  :  0   1   2   3   4   5  ...  19
isaac DOF, left  : 33  43  53  63  30  40  ...  62     block 29..63
isaac DOF, right : 38  48  58  68  34  44  ...  67     block 34..68
```

Copying a 20-vector straight into `set_joint_positions` writes fingers of **both** hands at
random. This is the W6 failure shape — hand observed as Dex3, commanded as Dex5 — arriving
through a different door, which is why it is written down (C-14) rather than quietly worked
around. `run.py` maps by name; so does `tools/capture_hand_poses.py`.

Note also Unitree's own naming asymmetry at index 12: `Roll_41R` on the right, `Link_41L` on the
left. A name-based map has to carry it.

---

## 4. Findings

**F-1 — the G1 already defines `left_hand_palm_joint` / `right_hand_palm_joint`.** They attach
the `*_rubber_hand` caps the robot ships with (0.170 kg each, visual only, no collision). The
first merge added joints with the same names; the importer resolved the ambiguous tree to a hand
as its root link and wrote a 2.4 kB asset containing nothing, with one `RuntimeError: Used null
prim` and no indication of the cause. The Dex5 replaces the cap, so the merge now deletes it and
reuses the name. The merge asserts a single parentless link and rejects duplicate joint names of
**any** type; the previous check covered revolute joints only.

**F-2 — the flange offset is a property of the arm, not the hand.** That same G1 joint carries
origin (0.0415, ±0.003, 0) — identical to the value taken from `g1_29dof_with_hand_rev_1_0.urdf`
in DT2 and flagged `assumed`. It is now read from the G1's own description and asserted, and the
`assumed` flag is retired. The W6 MuJoCo model is **not** a usable source for this: its Dex5
meshes carry `scale="0.75"` and the palm geom is `contype=0 conaffinity=0 density=0`, i.e. a
decorative shell.

**F-3 — Unitree's Dex5 files declare every visual material as `<material name="">`.** Harmless
when a hand is imported alone; in a robot that also has named materials the importer resolves the
empty name to nothing and dies after writing a stub. The merge names them from their own rgba,
so the colour is untouched.

**F-4 — four links warn at load and it is benign.** `imu_in_pelvis`, `imu_in_torso`, `d435_link`
and `mid360_link` have no visual, collision or inertial in Unitree's own URDF, so their `visuals`
scope references nothing. 4 warnings, 4 links, and the mesh count reconciles exactly:
59 original − 2 caps + 84 hand = 141.

**F-5 — the environment failures cost more than the work.** Both are now guarded in the scripts:

* a scratch probe named `bisect.py` in `/tmp` shadowed the **stdlib** `bisect` module. Python
  puts the script's own directory on `sys.path[0]`, so Isaac's startup imported the probe, and
  every Isaac script in `/tmp` then failed at `from isaacsim import SimulationApp` with a
  circular-import warning naming neither the file nor the module. Isaac scripts now stage in
  `/tmp/isaacrun`.
* a root-owned `__pycache__` under Isaac's package tree, left by a crashed run, broke every
  later run as UID 1234. Isaac execs now set `PYTHONDONTWRITEBYTECODE=1`.

**F-6 — rendering a hand took five wrong answers, each hiding the next.** Recorded because the
next person to point a camera in this stack will hit them: `set_joint_positions` teleports but
leaves the drive target behind, so the pose collapses; gravity plus a reset impulse sends the
robot drifting (found at (10.8, −1.06, 1.46), so every camera aimed at the origin framed empty
floor); USD's default `clippingRange` starts at 1.0 m and a 0.4 m close-up falls inside the near
plane; and **Isaac's `Camera` wrapper ignored orientation entirely**, both at construction and
via `set_world_pose` — eight cameras at yaws 45° apart rendered statistically identical frames.
Authoring the transform directly from `Gf.Matrix4d().SetLookAt(...).GetInverse()` fixed it.

---

## 5. Open questions

* **OQ-3.1** — the `doc/05-validation.md` §1 stale line from DT2 is still a Kevin/Mohammed item.
* **OQ-3.2** — tactile zones are **authored, not published**, per the DT3 trim. The 12-zone
  contact-sensor layout per hand is not yet in the asset; it opens properly at DT6.
* **OQ-3.3** — hand drive gains are the URDF-import defaults (stiffness 20, damping 2), chosen
  as the campaign's "conservative documented gains, no tuning loop". They hold every commanded
  pose to ≤0.001 rad with gravity off and the root pinned. They have **not** been tested against
  a payload, and DT6 grasping is where that matters.
* **OQ-3.4** — the four hand PNGs are rendered with gravity off and the root pinned, and the
  right arm posed clear of the body. That is a presentation pose. It says nothing about the
  policy, which `validate_motion.py` exercises separately.

## 6. Deviations opened

* **C-13** — passive joints held at zero rather than mimic-coupled.
* **C-14** — sim hand DOFs interleaved; index-based hand commands do not transfer.

## 7. Reproduce

```bash
./scripts/12_import_g1_dex5.sh                 # merge + import, ~2 min
./scripts/08_build_twin_assets.sh g1_dex5      # sensor frames onto the merged asset
./scripts/10_test_twin_isaac.sh                # 11/11
ROBOT=g1 TWIN=1 HAND=dex5_1p SIM_WORLD=hospital ./scripts/01_start_sim.sh
ROBOT=g1 MODE=twin ./scripts/02_start_ferox.sh
docker exec ferox_nav bash -lc 'source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  python3 /tmp/validate_motion.py --ros-args -r /odom:=/ferox/g1_01/odom'
./scripts/13_capture_hands.sh                  # four PNGs
```

The Dex5 URDFs come from `~/panthera/ref/unitree_ros` (public, sparse clone, no PAT).
