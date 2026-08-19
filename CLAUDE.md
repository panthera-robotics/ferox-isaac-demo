# ferox-isaac-demo — operating constraints

This repo holds the Isaac Sim **digital twin** of Panthera's G1 and Go2: the same
body, the same sensors at calibrated poses, and the same ROS 2 / DDS interface
strings the real drivers publish. The point is transfer — work developed against the
sim must run on the robot unchanged.

The campaign brief, gate reports and every declared deviation live in `docs/twin/`.
Start at `docs/twin/RESULTS_FASTPATH.md`.

---

## RULE-HAND-NAME — hand joints map by name; never slice by index

**Applies to:** anything that reads or writes Dex5-1P joint state — in this repo, in
Ferox, in `panthera-g1-wbc`, or in a policy.

`wholebody/dex5/limits.py` clamps a flat **20-vector per hand in URDF document
order**, and that is the order the real driver speaks. Isaac orders articulation DOFs
breadth-first over the whole robot, so each hand's 20 joints land at scattered,
non-contiguous, **interleaved** indices:

```
limits.py index  :  0   1   2   3   4   5  ...  19
isaac DOF, left  : 33  43  53  63  30  40  ...  62      block 29..63
isaac DOF, right : 38  48  58  68  34  44  ...  67      block 34..68
```

Neither block is contiguous, and the two hands interleave with each other.

**Therefore:**

* Build the mapping **by joint name**. `names.index("Pitch_13L")`, or
  `_is_hand_joint(name)` in `isaac/run.py`, or the `hand_names` list in
  `tools/capture_hand_poses.py`.
* **Never** slice a joint array with a literal index in the hand block (29–68), and
  never assume a hand occupies a contiguous range.
* Slicing the **body** block by index is fine — `dof_names[:29]` is asserted
  bit-identical to the pre-hand order by
  `tools/tests/test_twin_isaac.py::test_body_dof_order_unchanged_by_the_merge`. It is
  the hand block, and only the hand block, that this rule forbids.

**Why it matters this much.** Copying a 20-vector straight into
`set_joint_positions` writes fingers of **both** hands at random. That is the W6
failure shape — hand observed as Dex3, commanded as Dex5 — arriving through a
different door, and this campaign exists to stop it happening twice.

Unitree's own naming carries an asymmetry a name-based map must handle: index 12 is
`Roll_41R` on the right and `Link_41L` on the left.

Enforced by `tools/tests/test_twin_contract.py::test_rule_hand_name_no_numeric_hand_indexing`,
which fails on any numeric index into the hand DOF block. Recorded as **C-14** in
`docs/twin/TWIN_DEVIATIONS.md`, and declared in both contracts under `rules:`.

---

## The other standing constraints

* **Contracts are the source of truth.** `isaac/twin/<robot>_contract.yaml` holds every
  topic, frame, rate, QoS and payload the twin must reproduce. The sim authors from
  it, the audit checks against it, and the Ferox bridge is passed the same numbers.
  Do not re-derive a value to suit the sim — record a deviation instead.
* **Never scale a hand, a mesh, or an offset to make it fit.** Real dimensions or a
  flagged TODO. The W6 MuJoCo model scaled the Dex5 meshes to 0.75 and gave the palm
  no mass; every grasp after that happened by weld.
* **Docker immutability.** Anything `apt`/`pip`-installed inside a running container
  is gone on the next `up`. Bake it into a Dockerfile, or vendor it into the repo —
  which is why `ferox_nav_sim` carries a verbatim copy of the driver's
  `cloud_accumulator.py` rather than importing it.
* **Audit every generated artefact.** Every authored USD, YAML or merged URDF is
  followed by a read-back or a scripted assertion. On this campaign that caught a
  reference resolving one directory too shallow, a USD list-op silently appending a
  stale path, and an importer writing a 2.4 kB asset containing nothing — none of
  which reported an error.
* **Isaac scripts stage in `/tmp/isaacrun`, never bare `/tmp`,** and run with
  `PYTHONDONTWRITEBYTECODE=1`. `sys.path[0]` is the script's own directory, so a
  scratch file named after a stdlib module breaks Isaac's own startup; and a
  root-owned `__pycache__` breaks every later run as UID 1234.
