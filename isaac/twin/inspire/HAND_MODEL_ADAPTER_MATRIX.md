# Hand model adapter matrix — what can drive the twin's six-actuator hands, and under which declaration

Companion to `hand_action_contract.py` (contract instance `embodiments/hand_action_contract_rh56dftp_donor_v1.json`,
sha256 `4f1facb7b2de5394f9dae2b01e85d89e14a586fdf5be0d75e9d11061daa7fa1b`) and `hand_command_sets.py`
(`embodiments/hand_command_sets_v1.json`, sha256 `a8d42d9cafa90df143b1d515ac3f6109ba80d07836e361b56a09d5f440140c99`).
Twin = PROVISIONAL RH56DFTP donor (`g1_edu29_rh56dftp_donor_v1`, manifest sha `a67e7816…`). Everything here is a
statement about the twin; nothing is a hardware statement about the installed RH56E2 hands (all installed-E2 fields of
the contract are UNRESOLVED/NOMINAL with no values; the contract refuses `target='installed_e2'`).

**Reading rule.** "Mapping available" means an action vector can be converted into the contract's six axes with a
declared order, unit, direction, clip policy and hash binding. It never means the learned policy achieves the task:
the only learned-hand evidence on this twin is the recorded-ACTION replay of one piston episode (CAPTURE, diagnostic
reference, torso pairs filtered) and the K/L closed-loop probes (no capture). Learned-task success is a separate
measurement with its own receipt.

## 1. The common contract (what every adapter converts INTO)

| field | value | status |
|---|---|---|
| axis order | `index, middle, ring, little, thumb_bend, thumb_rotation` (`HAND_ACTUATORS`) | fixed by the contract |
| unit | rad on the twin drive joints (`*_index_1`, `*_middle_1`, `*_ring_1`, `*_little_1`, `*_thumb_2` = bend, `*_thumb_1` = rotation) | VERIFIED |
| direction | + closes; URDF zero = open | VERIFIED |
| endpoints (open → closed) | fingers 0 → 1.4381; thumb_bend 0 → 0.5864; thumb_rotation 0 → 1.1641 | VERIFIED |
| coupled children | `_2` = 1.0843 × parent; thumb_3 = 0.8024 × thumb_2; thumb_4 = 0.9487 × thumb_3 (mimic, uncapped) | VERIFIED |
| velocity cap | 1.0 rad/s on the six drive joints (`physxJoint:maxJointVelocity` == URDF field; hand-req-M01 readback) | VERIFIED |
| clip policy | per route: `reject` (scripted) or `clip_declared` (model) — clipped axes are always recorded, never silent | contract rule |
| hash binding | `declaration = {contract_sha256, route, side, target, model_map, exploratory, fields_not_verified, profile_sha256}`; `profile_sha256` = `hand_contract_descriptor` of the applied profile, the same value the replay packager binds | contract rule |
| fail-closed | any needed field not VERIFIED on the target → `ContractError` unless `exploratory=True` is declared (and recorded in the declaration) | contract rule |

## 2. Matrix

| # | source | hand action space (from source) | order → contract | unit / rep | range → clip | route call | status of the MAPPING | evidence of TASK success on the twin |
|---|---|---|---|---|---|---|---|---|
| A | **Scripted six-axis** (`hand_command_sets`: `rod30_pick_v2`, `held_marker_grasp_v12`, `piston_n16_dataset_closure`) | 6 normalized closures per side | identity (already contract order) | normalized 0–1 → twin rad by the contract endpoints; ABSOLUTE | `reject` (a value outside [0,1] is a refusal) | `contract.route('scripted', side)`; `sets.joint_targets(manifest, set, stage)` | **VERIFIED** — profile hash identical to the qualified rod30 spec (`e03f6b33…`) | rod30 pick/lift/transport/place: sL/sM-rod30-regression-01 PASS (task_eval_v2 C1–C5); N integration schedule `manip-rod30-v1` hand rows IDENTICAL (1,132/1,132, Δ 0.0 rad). v12 writer grasp: held marker through the sM-writer-v11 runs (writing quality is a separate verdict) |
| B | **GR00T N1.6 fine-tune** `birbirll/g1-inspire-piston-n16` (`Gr00tN1d6Processor`, Isaac-GR00T n1d6 @ 9b37aa1c) | `left_hand`/`right_hand` 6-D, dataset names `pinky, ring, middle, index, thumb_pitch, thumb_yaw`; ABSOLUTE; 30-step chunk at 50 Hz; min-max to [−1,1] with clipping; no sin/cos | `MODEL_DATASET_ORDERS['piston_n16']` = `little, ring, middle, index, thumb_bend, thumb_rotation` (permutation to contract order; `piston_model_interface.HAND_DONOR_IDENTITY`) | rad, ABSOLUTE (the RELATIVE form exists only inside the processor); radian identity with the twin joints — **EXPLORATORY** (recorded hand model UNRESOLVED) | per-axis twin endpoints, `clip_declared`; `thumb_yaw` is pinned at −0.1 in the data (min == max) → clip to the open endpoint 0, recorded in `clipped_axes`; NO +0.2 rad gain; the normalized-to-1.7 alternative FAILED hand-an-02-AB and is not offered | `contract.route('model', side, model_map='piston_n16', exploratory=True)` → `HandCommandAdapter(manifest, side, src)` | **EXPLORATORY identity** (order/unit/direction pinned from the processor source; datum unresolved) | recorded-ACTION replay S = 1.75, torso pairs filtered: CAPTURE (+16.8 cm held; diagnostic reference only); closed-loop K/L probes: no capture; learned arm route blocked by the arm–torso collision at the dataset's shoulder roll (H3: 12–15 mm mesh interpenetration at +0.215) |
| C1 | **GR00T N1.7 base** (cached `gr00t-n1.7-3b`, processor_config sha `85c1b469…`, statistics sha `c97b1b07…`, code Isaac-GR00T @ 51d4c89f), pretrain tag `real_g1_relative_eef_relative_joints` | `left_hand`/`right_hand` **7-D**, ABSOLUTE, `NON_EEF/DEFAULT`; horizon 40; `use_relative_action` true (arms/EEF only); percentile normalization (`use_percentiles`, q01/q99); no sin/cos; action q01–q99: right `[0,1.5]×4, [−0.5,0.5], [−0.7,0], [−0.7,0]`, left mirrored in sign | **none** — 7 axes without names in the cached code; the twin has six actuators; no channel may be invented, dropped or reinterpreted | rad presumed (unverified) | n/a | none; `model_action_adapter.validate_chunk(..., scope='arms_and_waist_only')` reports both hand keys REJECTED and leaves the hands uncommanded (`gr00t_n17_real_g1_to_contract_v1`, validate-only) | **NOT AVAILABLE** (the adapter docstring attributes the 7-D space to a Unitree Dex3-1 three-finger hand; the cached code does not name the hand — treat as unverified) | none |
| C2 | **GR00T N1.7 posttrain tag** `unitree_g1_full_body_with_waist_height_nav_cmd` (`EmbodimentTag.UNITREE_G1`, projector id 25 shared with the pretrain tag) | `left_hand`/`right_hand` ABSOLUTE (source comment: "G1 hand is controlled by binary signals like a gripper"); horizon 50; dims and names come from the fine-tune dataset's `modality.json`, not from the code | defined only by a fine-tune dataset: with the piston dataset's `modality.json` (six named axes) the B permutation applies; any other dataset needs its own `model_maps` entry with a written order check | as the dataset declares (rad or binary) | as the dataset declares → `clip_declared` | `route('model', side, model_map=<new entry>, exploratory=True)` after adding the entry to the contract JSON (contract sha changes → new binding) | **CANDIDATE** — no six-axis fine-tune exists in this workspace; no fine-tuning is authorized in Sprint N | none |
| C3 | **GR00T N1.7 posttrain tag** `unitree_g1_sonic` (`EmbodimentTag.UNITREE_G1_SONIC`, projector id 11) | state adds `left_leg/right_leg/projected_gravity`; actions `motion_token` + `left_hand_joints` + `right_hand_joints` (ABSOLUTE); dims from the fine-tune dataset | same as C2 for the hand keys; the body is a SONIC motion token (whole-body lane) | as the dataset declares | as the dataset declares → `clip_declared` | as C2 (`left_hand_joints` naming) | **CANDIDATE** — same conditions as C2 | none |
| D | **UnifoLM-VLA-0** (source `unifolm-vla` @ ff6c39ae, read-only copy held by the model lane; released `UnifoLM-VLA-Base` fine-tuned on the Unitree G1 + **Dex1 gripper** LeRobot datasets; weights = one 18.98 GB pickle, NOT fetched) | default platform `G1_EE_6D`: action = 2 × [EEF XYZ (3) + R6 (6) + **Gripper Open/Close (1)**] + waist roll-pitch-yaw (3) = 23-D, chunk 25, `BOUNDS_Q99` normalization; joint platform `G1`: 2 × [7 joints + gripper (1)] (+ waist 3 per the enum comment; `ACTION_DIM` 16 in `constants.py` vs 19 in the LeRobot→HDF5 converter — inconsistent in the source); converter order `[left_arm/ee, right_arm/ee, right_gripper, left_gripper, body]` (RIGHT gripper before LEFT); gripper keys `observation/action.{left,right}_gripper` (`rlds_dataloader/constants.py`, `oxe/configs.py`, `oxe/transforms.py`, `prepare_data/convert_lerobot_to_hdf5.py`) | **none** — one scalar per hand; expanding 1 → 6 actuators would be an invented mapping | gripper scalar (units/range from the checkpoint `dataset_statistics.json`, pending from the model lane) | n/a for a qualified route; a CANDIDATE trigger adapter (threshold on the scalar → a scripted command-set stage such as `rod30_pick_v2` open/closed) would need `exploratory=True` and its own `model_maps` entry declaring the threshold and the set | none today | **NOT AVAILABLE as a six-axis map** (source read 02:50Z; SCHEMA RESOLVED from source, mapping refused by construction) | none |

## 3. What a learned-model adapter must supply to obtain a contract (checklist)

1. **Axis names and order** as the model emits them, from the processor/modality source (not from frames); add a
   `model_maps.<name>` entry with `dataset_order` (a permutation of the six contract names) and `dataset_names`.
2. **Unit and representation**: hands must be ABSOLUTE in the emitted chunk (a RELATIVE hand action would need state
   composition, which the contract does not perform); rad or a declared normalized scale with its endpoints.
3. **Range → clip policy**: per-axis twin endpoints; any pinned/out-of-range column is declared (`clip_declared`) and
   reported in `clipped_axes` of every conversion. No gain, offset or "correction" without a measured source.
4. **Exploratory declaration** whenever the datum (dataset radians == twin radians) is not VERIFIED — it goes into the
   declaration and the contract hash; a qualified run needs VERIFIED fields only.
5. **Rate and the cap**: the six drive joints saturate at 1.0 rad/s; zero-order-hold rows at 30 Hz burst to the cap
   between rows (1.9 rad/s commanded → 2.39 achieved on the DIAG asset), so chunks at 50 Hz (N1.6/N1.7) are replayed
   as emitted and never resampled below that without a declared retiming (`hand-req-M04` pattern, S declared).
6. **Hash binding**: record `contract_sha256`, `profile_sha256` (descriptor), the model checkpoint sha, the processor
   config sha and the code commit in the run manifest; the observer thresholds come from the qualification asset,
   never from a mounted diagnostic asset.
7. **Target**: `twin` only. `installed_e2` yields no contract while any installed field is UNRESOLVED.

## 4. Sources (all read-only, none modified)

- Contract/command-set modules and tests: `isaac/twin/inspire/{hand_action_contract,hand_command_sets}.py`,
  `tools/tests/test_hand_{action_contract,command_sets}.py`.
- N1.6 route: `tools/hand_fidelity/piston_model_interface.py` (branch `sim/mohammed/hand-fidelity`), H2 model
  conventions (private evidence `evidence/hand_fidelity/H2_MODEL_CONVENTIONS.md`), recorded-ACTION replay analysis
  (`hand-an-M02`).
- N1.7: cached checkpoint `gr00t-n1.7-3b/{processor_config,statistics,embodiment_id}.json`, code
  `Isaac-GR00T-51d4c89f…/gr00t/{configs/data/embodiment_configs.py, data/embodiment_tags.py, model/gr00t_n1d7/processing_gr00t_n1d7.py}`,
  `isaac/twin/inspire/model_action_adapter.py` (validate-only adapter).
- UnifoLM-VLA-0: model-lane pinned copy `parallel-20260917/model/clones/unifolm-vla` @ ff6c39ae (`src/unifolm_vla/rlds_dataloader/{constants.py, datasets/rlds/oxe/configs.py, datasets/rlds/oxe/transforms.py}`, `prepare_data/convert_lerobot_to_hdf5.py`, README model table).
- Velocity cap and burst facts: hand-req-M01 readback + `F7_OBSERVER_DECISION_M.md` (private evidence).
