# tools/hand_fidelity/rh56e2 — installed RH56E2 measurement preparation (CPU only)

| module | role |
|---|---|
| `register_map.py` | user register map transcribed from the RH56E2 manual V1.0.0 (Tables 41-58, page refs), byte/word conventions, decode/encode of the six-axis groups, CSV sheet; the manual's Table 49 omits the thumb-bending row (flagged, VERIFY) and never states the Modbus register-index convention (three candidates listed, verify on the unit) |
| `transport.py` | dry_run (default, no I/O), rs485_native (manual frames, checksum verified against Tables 6/7/11), modbus_tcp (stdlib), modbus_rtu (pyserial + CRC16); any real transport needs `--hardware-authorized` + `PANTHERA_HARDWARE_AUTHORIZED=1` + `--operator`, writes need `--allow-writes` |
| `actuator_logger.py` / `analyze_actuator_log.py` | six-axis command/readback logger (angle, actuator position, force, current, temperature, error, status) with an optional angle_set step plan; analysis gives readback range, direction, deadband bounds, step response, error codes — NOT_MEASURED when absent |
| `tactile_logger.py` | raw tactile block capture with the variant declared (or `unknown` = raw sweep, nothing decoded) |
| `sheets.py` | blank geometry / mass-COM / serial-firmware (PRIVATE) / actuator-endpoints / tactile sheets + the register map sheet |
| `conversion_template.py` | native-to-radian map + six-axis convention block, refused while any field is null |
| `compare_installed.py` | donor-vs-installed rows (donor profile proxies named), predeclared tolerances, completeness gate → `RH56E2_INSTALLED_CALIBRATION_COMPLETE` / `_INCOMPLETE` |
| `fiducial_sheet.py` | A4 grid/ruler/ArUco sheet (SVG + PNG + detector self-check) |
| `docs/` | MEASUREMENT_CHECKLIST, PHOTO_GUIDE, FIDUCIAL_LAYOUT, LEDGER_UPDATE_PROCEDURE |

Tests: `python -m pytest -q tools/tests/test_rh56e2_prep.py` (no hardware, no network). Nothing in this package is an installed
measurement; the hand in the twin remains the PROVISIONAL RH56DFTP donor until the label is COMPLETE.
