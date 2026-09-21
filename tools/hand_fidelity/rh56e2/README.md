# tools/hand_fidelity/rh56e2 — installed RH56E2 measurement preparation (CPU only)

| module | role |
|---|---|
| `register_map.py` | user register map transcribed from the RH56E2 manual V1.0.0 (Tables 41-58, page refs), byte/word conventions, decode/encode of the six-axis groups, CSV sheet; the manual's Table 49 omits the thumb-bending row (flagged, VERIFY) and never states the Modbus register-index convention (three candidates listed, verify on the unit) |
| `transport.py` | dry_run (default, no I/O), rs485_native (manual frames, checksum verified against Tables 6/7/11), modbus_tcp (stdlib), modbus_rtu (pyserial + CRC16); any real transport needs `--hardware-authorized` + `PANTHERA_HARDWARE_AUTHORIZED=1` + `--operator`, writes need `--allow-writes`. Modbus 8-bit groups (error/status/temperature: six byte channels in three words) keep the raw words and both bytes of every word in each record; channel order is only formed under a declared `--modbus-byte-order low_high|high_low` (UNDECLARED by default, never guessed); owner-declared installed profiles `--profile left|right` (192.168.123.210/.211:6000, unit id 1) |
| `actuator_logger.py` / `analyze_actuator_log.py` | six-axis command/readback logger (angle, actuator position, force, current, temperature, error, status) with an optional angle_set step plan; analysis gives readback range, direction, deadband bounds, step response, error codes — NOT_MEASURED when absent |
| `tactile_logger.py` | raw tactile block capture with the variant declared (or `unknown` = raw sweep, nothing decoded) |
| `sheets.py` | blank geometry / mass-COM / serial-firmware (PRIVATE) / actuator-endpoints / tactile sheets + the register map sheet |
| `conversion_template.py` | native-to-radian map + six-axis convention block, refused while any field is null |
| `compare_installed.py` | donor-vs-installed rows (donor profile proxies named), predeclared tolerances, completeness gate → `RH56E2_INSTALLED_CALIBRATION_COMPLETE` / `_INCOMPLETE` |
| `fiducial_sheet.py` | A4 grid/ruler/ArUco sheet (SVG + PNG + detector self-check) |
| `docs/` | MEASUREMENT_CHECKLIST, PHOTO_GUIDE, FIDUCIAL_LAYOUT, LEDGER_UPDATE_PROCEDURE |
| `e2prior_asset.py` | builds the PUBLIC exact-E2 prior asset `g1_edu29_rh56e2_e2prior_v1` (label PUBLIC_E2_PRIOR__INSTALLED_CALIBRATION_INCOMPLETE) from renesas-rdk/inspire_rh56e2_hand @ 81bdb56 + the frozen donor's G1 body: LFS-oid mesh verification, xacro expansion, left inertials mirrored (public left file copies the right), left pinky_intermediate mass, declared drive-limit policy on the merged twin URDF, mass policy, manifest with every hash; donor never modified |
| `e2prior_audit.py` | static audit of that asset (pinocchio + coal, CPU): chirality vs the donor, 6 active / 12 mechanical joints, coupling graph, limits vs the public priors, FK, left-inertial mirror check, masses/policy, mesh scale, LFS stubs, 17 tactile frames, self-collision preflight with penetration depth (`mesh_penetration.py`), self-collision envelope sweeps, palm/thumb cavity sweep, left-vs-mirrored-right mesh comparison → JSON |
| `e2_adapter.py` | six-axis semantic adapter + the E2 kinematic contract: native/contract orders, counts ↔ closure ↔ radians, coupled targets clamped at the child limit, donor→E2 closure-preserving map and joint name map |
| `e2_tactile_frames.py` / `e2_embodiment_manifest.py` / `e2prior_report.py` | E2_TACTILE_FRAMES.json (17 frames per hand, Table 56 array assignment, TACTILE_LAYOUT_PUBLIC_PRIOR), the ferox `inspire.embodiment` manifest for the E2 asset (validated by `EmbodimentManifest.load`), E2_PRIOR_ASSET_AUDIT.md + DONOR_VS_E2_PUBLIC_PRIOR.md renderers |

Tests: `python -m pytest -q tools/tests/test_rh56e2_prep.py tools/tests/test_e2prior.py` (no hardware, no network; the byte-group regression runs a loopback fake-Modbus server in-process). Nothing in this package is an installed
measurement; the hand in the twin remains the PROVISIONAL RH56DFTP donor until the label is COMPLETE.
