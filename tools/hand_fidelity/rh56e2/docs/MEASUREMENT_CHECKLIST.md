# RH56E2 installed-hand measurement checklist (preparation; nothing here authorizes hardware work)

Scope: the RH56E2-2R (right) and RH56E2-2L (left) hands as mounted on the owner's G1 EDU. Every step needs a named human
operator and the owner's explicit hardware authorization; the loggers refuse to open a port or socket without three
independent switches (`--hardware-authorized`, `PANTHERA_HARDWARE_AUTHORIZED=1`, `--operator`), and refuse writes without
`--allow-writes`. Record each side separately. Never invent a value: a field stays `null` until it is measured, and the
comparison tool then reports NOT_MEASURED. Serial numbers and photos are private and never enter the public repository.

Sheets (blank, one set per side): `python -m hand_fidelity.rh56e2.sheets --side right --hardware-model <nameplate> --out-dir <private dir>`
→ geometry_sheet.json, mass_com_sheet.json, serial_firmware_sheet.json, actuator_endpoints_sheet.json, tactile_sheet.json,
register_map_sheet.csv. Fill `measured_utc`, `operator`, `datums`, `source_evidence` first.

## 0. Before touching anything (CPU, today)
- [ ] Print `fiducial_A4.svg` (or the PNG) at 100 % scale; measure the 100 mm check bar with a steel rule and write the printed length on the photo log (the sheet's JSON carries the marker centres).
- [ ] Decide and write the datums in every sheet: D0 flange face (origin at the flange centre), D1 palm centre line (through the middle-finger MCP hinge), D2 metacarpal plane (through the four MCP hinge axes, manual Figure 5).
- [ ] Identify the transport actually in use on the G1 (RS485 native / Modbus RTU / Modbus TCP / CAN / Unitree DDS bridge) from the owner's configuration — do not probe the network to find out.
- [ ] Dry-run both loggers (`--transport dry_run`) to check the output paths and the record format.

## 1. Identity (passive; PRIVATE sheet: serial_firmware_sheet.json)
- [ ] Nameplate photo (model code, serial, tactile variant code) — private storage only.
- [ ] Firmware version from the vendor tool, label or bridge readback. The manual lists no user register for it (Table 41); leave `null` rather than infer.
- [ ] Register readbacks: hand id (1000), baud (1002), IP octets (1700-1703), power-on speed (1032), power-on force threshold (1044), finger motion mode (1625), dof_status raw (1612, meaning undocumented).
- [ ] Modbus register-index convention: read `angle_actual` (1546) under the declared convention and confirm the six values match the bridge/RS485 reading; record which convention reproduced them (`register_map.ADDRESSING_CONVENTIONS`).

## 2. Geometry (passive, fingers open = command 1000; geometry_sheet.json)
- [ ] Palm thickness at the marker pocket, palm width across the MCP row, flange-to-MCP-row, flange-to-index-tip: caliper, three repeats each.
- [ ] Hinge positions: MCP and PIP hinge centres of index/middle/ring/little, thumb rotation-axis point and direction, thumb MP and IP hinge centres — caliper from D0 plus scale photos on the fiducial sheet (photo guide views V1-V4).
- [ ] Segment lengths between hinge centres (proximal, MCP-to-tip; thumb segments 1/2/3).
- [ ] Pad dimensions: length, width, thickness of each fingertip pad and the thumb pad; palm pad length/width.
- [ ] Wrist mounting: flange-to-`wrist_yaw_link` translation and rotation on the adapter plate (caliper + square + scale photo).

## 3. Open / closed endpoints (owner-operated motion, one axis at a time; geometry + actuator sheets)
- [ ] For each finger: alpha at command 1000 and at command 0 (protractor overlay on a D2-plane photo, V5/V6); fingertip position at both (scale photo).
- [ ] Thumb: theta at bend 1000/0, beta at rotation 1000/0; thumb rotation sense (toward the palm centre line when 1000 -> 0? CW/CCW seen from the palm side).
- [ ] Aperture thumb tip to index tip, open and closed.
- [ ] Native readbacks at both endpoints: `angle_actual` (1546) and `actuator_position_actual` (1534) per axis (from the actuator logger, `--plan` with 1000 then 0 for one axis; others -1).

## 4. Actuator command/readback (actuator_logger.py + analyze_actuator_log.py; actuator_endpoints_sheet.json)
- [ ] Log at 20-50 Hz while the operator runs each axis 1000 -> 0 -> 1000 at speed 1000 (registers 1522 not written by the tool; note the speed in force at the time).
- [ ] From the analysis: native min/max readback, direction (readback follows/opposes command counts), t10/t90/settle per step (speed response), deadband bounds (largest command change without motion / smallest with motion — refine with 5-10 count steps), command and readback units (counts), error codes seen.
- [ ] Repeat for the thumb rotation with its own endpoints: its counts-to-beta scale is a separate affine, never assumed equal to the fingers'.
- [ ] Note any axis whose readback saturates or does not follow its target.

## 5. Mass and COM (only if the hand is already detached for another reason; mass_com_sheet.json)
- [ ] Scale identity and a known-mass check reading. Hand only, hand + cable, wrist adapter, as-mounted total (or NOT_MEASURED).
- [ ] COM: three-point suspension with plumb lines photographed against the fiducial sheet (or two-edge balance); hand open (command 1000 all axes); attachment points in D0; repeat once with the hand closed to detect a cable pulling on the scale.

## 6. Tactile (tactile_logger.py; tactile_sheet.json)
- [ ] Raw sweep with `--tactile-variant unknown` (3000-5123) at rest; inspect which spans respond (touch test per pad) and match against Table 56 (piezoresistive) or Tables 57-58 (capacitive); only then declare the variant.
- [ ] Channel count, array list (name, address, rows x cols, row-1/col-1 physical location from the touch test), units (raw 0-4095 or Table 57 floats), achieved rate, non-zero channels at rest.

## 7. Close-out (CPU)
- [ ] `python -m hand_fidelity.rh56e2.conversion_template <actuator_endpoints_sheet.json>` → native-to-radian map (refuses on any null).
- [ ] `python -m hand_fidelity.rh56e2.compare_installed <sheets dir> --donor-profile <right_profile.json> --ledger <FIDELITY_LEDGER json>` → DONOR_VS_INSTALLED.json/.md and the label.
- [ ] Follow LEDGER_UPDATE_PROCEDURE.md. The label `RH56E2_INSTALLED_CALIBRATION_COMPLETE` is printed only when every required item is measured with an evidence reference; anything less prints `..._INCOMPLETE` with the missing list.
