# tools/hand_fidelity — offline Inspire-hand fidelity checks (CPU, no simulator)

Reusable checks that read the hand URDF (+ STL meshes when present) and the versioned embodiment manifest and answer,
with evidence labels, what the **donor asset** is — never what the installed hand is. The running twin uses a
**provisional RH56DFTP donor**; exact RH56E2 equivalence is not established by anything here.

| module | what it does |
|---|---|
| `coupling.py` | mimic-chain composition (`q_c = (c·a)·q_a + (c·b + d)`), range-vs-own-limit margins, equivalence of two constraint layouts (algebra only; PhysX dynamics stay a runtime test) |
| `donor_profile.py` | per-side profile: independent/coupled joints, axes, limits, wrist mount, chirality, thumb-rotation datum, FK sweeps in **one stated frame** (root or wrist), mass/COM/inertia at open and closed, mesh extents |
| `command_semantics.py` | named command conventions (E2 angle/actuator registers, Unitree DDS normalized, Unitree `inspire_hand` URDF radians, FTP donor radians, dataset card) with per-axis evidence class; conversions through closure; `radian_identity` vs `closure_preserving` import policies and their discrepancy table |
| `load_record.py` | machine-readable load record (hand / wrist adapter / tool) in the wrist frame; unknown components stay `null` and never compose as zero |
| `nominal_reference.py` | RH56E2 manual Figure 5 angle definitions and product-page mass, datum-reconciled comparison rows |
| `measurement_intake.py` | intake validation of installed-measurement files (evidence class, split, finiteness, non-negative uncertainty, units, repeat agreement, vector dimensions, evidence-file sha256, coverage) |
| `candidate_invalidation.py` | what a candidate asset edit (widened coupled-joint lower limits, mirrored base_link inertial) does to the manifest's bound qualifications — on a temporary copy; the active asset and manifest are never written |
| `ab_source_specs.py` | paired replay source specs (identical rows, closure-preserving vs radian-identity contracts) for the hand-map import-policy diagnostic; free-sweep row builder |
| `conversion_profile.py` | opt-in typed conversion profile: distinct value types, bound kinds, per-axis evidence, refusals (side/order/type/dimension/non-finite/range/provenance), operating margin, raw-to-effective trace, coupled-joint consistency, dependency hashes; qualified profiles fail closed on unresolved semantics |
| `load_contract.py` | `hand_load_contract_v1`: validation of a HAND_LOAD_RECORD bundle (frames, units, SPD + triangle inequality, parallel-axis and rigid re-expression round trips, totals vs components, null-never-zero), consumer rules against double counting, donor load at an actual configuration vs the open/closed samples |
| `ab_analysis.py` | compares two command-replay evidence directories (measured vs commanded hand joints per replayed stage, coupled error, fingertip positions in the palm frame, streamed hand-object contact sets, object-in-palm offsets, evaluator verdicts) |
| `media.py` | small labelled PNG diagrams (Pillow only): nominal-vs-donor angle curves, the A/B import-policy map, the wrist-frame load/COM diagram — CPU diagrams, never runtime evidence |
| `report.py` | builds `HAND_FIDELITY_DELTA.json` for both hands |

## Reproduce

```bash
cd ferox-isaac-demo/tools
python -m hand_fidelity.report --donor-dir <campaign>/generated/ftp_donor --out HAND_FIDELITY_DELTA.json
python -m hand_fidelity.donor_profile <campaign>/generated/ftp_donor/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf --side right --out right_profile.json
cd .. && HAND_FIDELITY_DONOR_DIR=<campaign>/generated/ftp_donor python -m pytest -q tools/tests/test_hand_fidelity.py
```

Needs numpy only (pytest for the tests). Without the donor directory the asset-bound tests skip and the algebra,
command-semantics, load-record and intake tests still run.

## Evidence classes used in every output

`DONOR` (derived from the donor URDF/meshes), `NOMINAL` (manufacturer specification, hashed source), `MEASURED`
(installed hardware — none exists yet), `ASSUMED` (declared convention). `installed_similarity_percent` is always
`null`; the intake validator refuses synthetic fixtures as installed evidence.
